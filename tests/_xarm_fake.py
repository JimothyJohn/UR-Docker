"""A tiny UFACTORY controller for unit tests: the control port and the report port
over real loopback sockets, a motion queue that drains over time, and the replies
the SDK documents. It knows no kinematics — a joint move lands on its joints, a
linear move on its pose, and the "planner" refuses anything beyond 0.85 m. The
behaviour that matters (does the toolkit wait, refuse, convert units, re-arm) is
checked here; whether the real firmware agrees is the job of
``tests/test_ufactory_sim.py`` against UFACTORY's simulator."""

from __future__ import annotations

import math
import socket
import struct
import threading
import time

from urctl import xarm
from urctl.xarm import fp32_le, from_fp32_le

MOVE_S = 0.15


class FakeXArm:
    def __init__(self, *, version: str = "6,12,XS1303FAKE,AC1303FAKE,v2.5.105"):
        self.version = version
        self.joints = [0.0] * 7
        self.pose = [300.0, 0.0, 200.0, math.pi, 0.0, 0.0]  # mm + rotvec
        self.offset_rpy = [0.0] * 6
        self.state = xarm.STATE_SLEEPING
        self.mode = xarm.MODE_POSITION
        self.error = 0
        self.warn = 0
        self.enabled = False
        self.queue: list[tuple[float, str, list[float]]] = []  # (dwell, kind, target)
        self.busy_until = 0.0
        self.requests: list[tuple[int, bytes]] = []
        self.refuse_moves_with_error = 0  # latch this error on the next move
        # registers that are acted on but never answered: UFACTORY's simulator (v2.4.0)
        # never replies to MOTION_EN, to the SDK either
        self.unanswered: set[int] = set()
        self.enable_takes = True  # False: MOTION_EN is accepted and changes nothing
        # modes SET_MODE answers INVALID to: the simulator refuses joint teaching (2)
        self.refused_modes: set[int] = set()
        # verdicts the planner gives before its real one: the simulator's first check
        # after a move can answer 24 (Speed Exceeds Limit) with the arm at rest
        self.check_verdicts: list[int] = []
        self._invalid = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.control = self._listen()
        self.report = self._listen()
        self.ports = (self.control.getsockname()[1], self.report.getsockname()[1])
        for srv, fn in ((self.control, self._serve_control), (self.report, self._serve_report)):
            threading.Thread(target=self._accept, args=(srv, fn), daemon=True).start()

    @staticmethod
    def _listen() -> socket.socket:
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", 0))
        s.listen(8)
        s.settimeout(0.2)
        return s

    def close(self) -> None:
        self._stop.set()
        self.control.close()
        self.report.close()

    def _accept(self, srv, fn) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = srv.accept()
            except (TimeoutError, OSError):
                continue
            threading.Thread(target=fn, args=(conn,), daemon=True).start()

    # -- motion queue -----------------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        while self.queue and now >= self.busy_until:
            dwell, kind, target = self.queue.pop(0)
            if kind == "joints":
                self.joints = target
            elif kind == "line":
                self.pose = target
            self.busy_until = now + MOVE_S + dwell
        if self.state == xarm.STATE_MOVING and not self.queue and now >= self.busy_until:
            self.state = xarm.STATE_SLEEPING
        elif self.queue or now < self.busy_until:
            if self.state in (xarm.STATE_READY, xarm.STATE_SLEEPING, xarm.STATE_MOVING):
                self.state = xarm.STATE_MOVING

    def _status(self) -> int:
        status = 0
        if self.error:
            status |= xarm.STATUS_ERROR
        if self.warn:
            status |= xarm.STATUS_WARN
        if not self.enabled or self.state >= xarm.STATE_STOPPED or self.mode != xarm.MODE_POSITION:
            status |= xarm.STATUS_NOT_READY
        return status

    def _enqueue(self, kind: str, target: list[float], dwell: float = 0.0) -> None:
        if self.refuse_moves_with_error:
            self.error, self.refuse_moves_with_error = self.refuse_moves_with_error, 0
            self.state = xarm.STATE_STOPPED
            self.queue.clear()
            return
        if self._status() & xarm.STATUS_NOT_READY:
            return
        self.queue.append((dwell, kind, target))
        self.state = xarm.STATE_MOVING

    def handle(self, register: int, p: bytes) -> bytes:
        self.requests.append((register, p))
        self._tick()
        if register == xarm.REG_GET_VERSION:
            return self.version.encode().ljust(40, b"\0")
        if register == xarm.REG_GET_STATE:
            return bytes([self.state])
        if register == xarm.REG_GET_CMDNUM:
            return struct.pack(">H", len(self.queue))
        if register == xarm.REG_GET_ERROR:
            return bytes([self.error, self.warn])
        if register == xarm.REG_CLEAN_ERR:
            if self.error not in xarm.ESTOP_ERRORS:
                self.error = 0
            return b""
        if register == xarm.REG_CLEAN_WAR:
            self.warn = 0
            return b""
        if register == xarm.REG_MOTION_EN:
            if self.enable_takes:
                self.enabled = bool(p[1]) and not self.error
            return b""
        if register == xarm.REG_SET_MODE:
            if p[0] in self.refused_modes:
                self._invalid = True
            else:
                self.mode = p[0]
            return b""
        if register == xarm.REG_SET_STATE:
            if p[0] == xarm.STATE_STOPPED:
                self.queue.clear()
                self.busy_until = 0.0
            if p[0] == xarm.STATE_READY and self.state == xarm.STATE_PAUSED:
                self.state = xarm.STATE_MOVING if self.queue else xarm.STATE_SLEEPING
            else:
                self.state = p[0]
            return b""
        if register == xarm.REG_GET_JOINT_POS:
            return fp32_le(self.joints)
        if register == xarm.REG_GET_TCP_POSE_AA:
            return fp32_le(self.pose)
        if register == xarm.REG_SET_TCP_OFFSET:
            # the firmware drops to state 5 on a configuration write: not ready until
            # set_state(0) (seen on UFACTORY's simulator, v2.4.0)
            self.offset_rpy = from_fp32_le(p, 6)
            self.state = xarm.STATE_CONFIG_STOPPED
            return b""
        if register == xarm.REG_SLEEP_INSTT:
            if self.queue:
                d, k, t = self.queue[-1]
                self.queue[-1] = (d + from_fp32_le(p, 1)[0], k, t)
            return b""
        if register in (xarm.REG_MOVE_JOINT, xarm.REG_MOVE_JOINTB):
            self._enqueue("joints", from_fp32_le(p, 7))
            return b""
        if register == xarm.REG_MOVE_HOME:
            self._enqueue("joints", [0.0] * 7)
            return b""
        if register == xarm.REG_MOVE_LINE:
            target = from_fp32_le(p, 6)
            check = p[42] if len(p) > 42 else 0
            if check:
                if self._status() & xarm.STATUS_NOT_READY:
                    return bytes([0, 0, 255])  # what the simulator answers when not armed
                if self.check_verdicts:
                    return bytes([0, 0, self.check_verdicts.pop(0)])
                reach = math.dist(target[:3], (0.0, 0.0, 0.0))
                return bytes([0, 0, 0 if reach <= 850.0 else 25])
            self._enqueue("line", target)
            return bytes(3)
        if register == xarm.REG_CGPIO_SET_DIGIT:
            return b""
        return b""

    def _serve_control(self, conn: socket.socket) -> None:
        buf = b""
        with conn:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(4096)
                except OSError:
                    return
                if not chunk:
                    return
                buf += chunk
                frames, buf = xarm.split_frames(buf)
                for f in frames:
                    tid, _prot, _len, reg = struct.unpack(">HHHB", f[:7])
                    with self._lock:
                        data = self.handle(reg, f[7:])
                        status = self._status() | (xarm.STATUS_INVALID if self._invalid else 0)
                        self._invalid = False
                    if reg in self.unanswered:
                        continue
                    conn.sendall(struct.pack(">HHHB", tid, 2, len(data) + 2, reg) + bytes([status]) + data)

    def report_frame(self) -> bytes:
        with self._lock:
            self._tick()
            enable = 0x3F if self.enabled else 0
            body = bytes([(self.mode << 4) | self.state]) + struct.pack(">H", len(self.queue))
            body += fp32_le(self.joints) + fp32_le([*self.pose[:3], 0.0, 0.0, 0.0]) + fp32_le([0.0] * 7)
            body += bytes([0, enable, self.error, self.warn]) + fp32_le(self.offset_rpy) + fp32_le([0.0] * 4)
            body += bytes([3, 3])
        body += bytes(xarm.REPORT_NORMAL_MIN - 4 - len(body))
        return struct.pack(">I", xarm.REPORT_NORMAL_MIN) + body

    def _serve_report(self, conn: socket.socket) -> None:
        with conn:
            while not self._stop.is_set():
                try:
                    conn.sendall(self.report_frame())
                except OSError:
                    return
                time.sleep(0.02)
