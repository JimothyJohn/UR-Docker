"""Wire client for UFACTORY controllers (850, xArm 5/6/7, Lite 6) — stdlib only.

UFACTORY's SDKs (``xArm-Developer/xArm-Python-SDK``, BSD) speak a private
Modbus-TCP variant to the control box; this module speaks the same bytes
without the SDK, so the toolkit keeps its zero-dependency runtime. Everything
here was read off that SDK's source (``xarm/core/wrapper/uxbus_cmd*.py``,
``xarm/core/comm/base.py``, ``xarm/x3/base.py``, release of 2026-05-27):

* **Control, TCP 502.** Request = ``u16 transaction | u16 protocol (2) |
  u16 length | u8 register | payload``, header big-endian. ``length`` counts
  the register byte plus the payload. The reply echoes transaction, protocol
  and register, then one **status byte** (``0x08`` invalid, ``0x10`` not ready
  to move, ``0x20`` a warning is latched, ``0x40`` an error is latched) and the
  data. Floats in payloads are **little-endian fp32** — the header is
  big-endian, the data is not. A reply whose register byte is ``0xFF`` is an
  unsolicited motion-feedback frame and is skipped.
* **Report, TCP 30001** ("normal", ~5 Hz). A stream of frames, each starting
  with its own ``u32`` big-endian length: state (low nibble) + mode (high
  nibble), command-queue length, 7 joint angles (rad), the TCP pose (mm, RPY
  rad), 7 joint torques, brake/enable bits, error + warning codes, the active
  TCP offset (mm, RPY), the payload, collision/teach sensitivity. Read-only.

Units on the wire are **millimetres and radians**; :mod:`urctl.ufactory`
converts to the metres the rest of the toolkit uses. Motion commands are
**queued** by the controller (unlike URScript on 30001, where a new program
replaces the running one); ``set_state(4)`` stops and flushes the queue.
"""

from __future__ import annotations

import re
import socket
import struct
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass

CONTROL_PORT = 502
REPORT_PORT = 30001  # "normal" report; 30000 = real-time, 30002 = rich

PRIVATE_PROTOCOL = 0x0002
FEEDBACK_REGISTER = 0xFF
TRANSACTION_ID_MAX = 65535

# Register (function) codes, XCONF.UxbusReg in the SDK.
REG_GET_VERSION = 1
REG_GET_ROBOT_SN = 2
REG_MOTION_EN = 11
REG_SET_STATE = 12
REG_GET_STATE = 13
REG_GET_CMDNUM = 14
REG_GET_ERROR = 15
REG_CLEAN_ERR = 16
REG_CLEAN_WAR = 17
REG_SET_MODE = 19
REG_MOVE_LINE = 21
REG_MOVE_JOINT = 23
REG_MOVE_JOINTB = 24
REG_MOVE_HOME = 25
REG_SLEEP_INSTT = 26
REG_SET_TCP_OFFSET = 35
REG_GET_TCP_POSE = 41
REG_GET_JOINT_POS = 42
REG_GET_FK = 44
REG_GET_TCP_POSE_AA = 91
REG_MOVE_LINE_AA = 92
REG_CGPIO_SET_DIGIT = 134

# Status byte flags in a private-protocol reply.
STATUS_INVALID = 0x08
STATUS_NOT_READY = 0x10
STATUS_WARN = 0x20
STATUS_ERROR = 0x40

# Arm state (GET_STATE, report byte 4 low nibble).
STATE_READY = 0  # accepted set_state(0), nothing queued yet
STATE_MOVING = 1
STATE_SLEEPING = 2  # idle: queue empty, motors enabled
STATE_PAUSED = 3
STATE_STOPPED = 4
STATE_CONFIG_STOPPED = 5  # after a configuration write (TCP offset): set_state(0) re-arms
STATE_NAMES = {0: "READY", 1: "MOVING", 2: "SLEEPING", 3: "PAUSED", 4: "STOPPED", 5: "STOPPED"}

# Control mode (SET_MODE, report byte 4 high nibble).
MODE_POSITION = 0
MODE_SERVO = 1
MODE_JOINT_TEACH = 2  # hand-guiding (freedrive)
MODE_CARTESIAN_TEACH = 3
MODE_NAMES = {
    0: "POSITION",
    1: "SERVO",
    2: "JOINT_TEACHING",
    3: "CARTESIAN_TEACHING",
    4: "JOINT_VELOCITY",
    5: "CARTESIAN_VELOCITY",
    6: "JOINT_ONLINE",
    7: "CARTESIAN_ONLINE",
}

ALL_SERVOS = 8  # motion_en's "every joint" id

# Controller error codes → title (ControllerErrorCodeMap, English titles).
CONTROLLER_ERRORS: dict[int, str] = {
    1: "Emergency Stop Button on the controller is pushed in",
    2: "Emergency IO of the Control Box is triggered",
    3: "Emergency Stop Button of the Three-state Switch is pressed",
    10: "Servo motor error",
    **{10 + i: f"Servo motor {i} error" for i in range(1, 8)},
    18: "Force Torque Sensor Communication Error",
    19: "End Effector Communication Error",
    21: "Kinematic Error",
    22: "Self-Collision Error",
    23: "Joints Angle Exceed Limit",
    24: "Speed Exceeds Limit",
    25: "Planning Error",
    26: "Linux RT Error",
    27: "Command Reply Error",
    28: "End Module Communication Error",
    30: "Feedback Speed Exceeds limit",
    31: "Collision Caused Abnormal Current",
    32: "Three-point drawing circle calculation error",
    33: "Controller GPIO Error",
    34: "Recording Timeout",
    35: "Safety Boundary Limit",
    36: "The number of delay commands exceeds the limit",
    37: "Abnormal movement in Manual Mode",
    38: "Abnormal Joint Angle",
    39: "Abnormal Communication Between Master and Slave IC of Power Board",
    40: "Solution failure of error-free joint trajectory",
    41: "The content of the friction file is invalid",
    42: "The content of the calibration file is invalid",
    50: "Six-axis Force Torque Sensor read error",
    51: "Six-axis Force Torque Sensor set mode error",
    52: "Six-axis Force Torque Sensor set zero error",
    53: "Six-axis Force Torque Sensor is overloaded or the reading exceeds the limit",
    60: "Linear speed exceeded limit in servo_j mode",
    110: "Robot Arm Base Board Communication Error",
    111: "Control Box External 485 Device Communication Error",
}
CONTROLLER_WARNINGS: dict[int, str] = {
    11: "Current controller cache is full",
    12: "User instruction parameter error",
    13: "User command control code does not exist",
    14: "User instructions and parameters have no solution",
    15: "Modbus cmd full",
}
# Errors that are a stop the operator has to clear by hand (an E-stop pressed).
ESTOP_ERRORS = frozenset({1, 2, 3})

# Device type in the version string (``…,<axis>,<type>,…``): 12 is the 850.
DEVICE_TYPES = {(5, 5): "xArm5", (6, 6): "xArm6", (7, 7): "xArm7", (6, 9): "Lite6", (6, 12): "850"}


class XArmError(OSError):
    """The controller could not be reached or answered something unusable."""


class XArmTimeout(XArmError):
    """The connection held but no reply came in time (the request may still have acted)."""


def error_text(code: int) -> str:
    return CONTROLLER_ERRORS.get(code, f"controller error {code}") if code else ""


def warning_text(code: int) -> str:
    return CONTROLLER_WARNINGS.get(code, f"controller warning {code}") if code else ""


# --------------------------------------------------------------------------------------
# Framing (pure functions — what the tests pin byte for byte)
# --------------------------------------------------------------------------------------


def fp32_le(values: Sequence[float]) -> bytes:
    return struct.pack(f"<{len(values)}f", *(float(v) for v in values))


def from_fp32_le(data: bytes, n: int) -> list[float]:
    if len(data) < 4 * n:
        raise XArmError(f"expected {n} floats, got {len(data)} bytes")
    return list(struct.unpack(f"<{n}f", bytes(data[: 4 * n])))


def encode_request(trans_id: int, register: int, payload: bytes = b"") -> bytes:
    """One control request: big-endian header, the register, the payload."""
    if not 0 <= trans_id <= TRANSACTION_ID_MAX:
        raise ValueError("transaction id out of range")
    if not 0 <= register <= 0xFF:
        raise ValueError("register out of range")
    if len(payload) + 1 > 0xFFFF:
        raise ValueError("payload too long")
    return struct.pack(">HHHB", trans_id, PRIVATE_PROTOCOL, len(payload) + 1, register) + payload


@dataclass(frozen=True)
class Reply:
    trans_id: int
    register: int
    status: int
    data: bytes

    @property
    def has_error(self) -> bool:
        return bool(self.status & STATUS_ERROR)

    @property
    def has_warning(self) -> bool:
        return bool(self.status & STATUS_WARN)

    @property
    def invalid(self) -> bool:
        return bool(self.status & STATUS_INVALID)

    @property
    def ready_to_move(self) -> bool:
        return not self.status & STATUS_NOT_READY


def split_frames(buffer: bytes) -> tuple[list[bytes], bytes]:
    """Cut whole frames (``u16 length`` at offset 4, plus the 6-byte prefix) off
    the front of ``buffer``; return them and the incomplete tail."""
    frames = []
    while len(buffer) >= 6:
        size = struct.unpack(">H", buffer[4:6])[0] + 6
        if len(buffer) < size:
            break
        frames.append(buffer[:size])
        buffer = buffer[size:]
    return frames, buffer


def decode_reply(frame: bytes) -> Reply:
    """A control reply: header, register echo, status byte, data. A frame too
    short to hold the status byte or with the wrong protocol id is refused."""
    if len(frame) < 8:
        raise XArmError(f"reply too short ({len(frame)} bytes)")
    trans_id, prot, length, register = struct.unpack(">HHHB", frame[:7])
    if prot != PRIVATE_PROTOCOL:
        raise XArmError(f"unexpected protocol id {prot}")
    if length + 6 != len(frame):
        raise XArmError(f"length field {length} does not match a {len(frame)}-byte frame")
    return Reply(trans_id, register, frame[7], bytes(frame[8:]))


@dataclass(frozen=True)
class Report:
    """One "normal" report frame (port 30001)."""

    state: int
    mode: int
    cmd_num: int
    joints: list[float]  # rad, 7 slots (a 6-axis arm leaves the 7th at 0)
    pose_rpy_mm: list[float]  # x, y, z mm; roll, pitch, yaw rad
    torques: list[float]
    brake_bits: int
    enable_bits: int
    error_code: int
    warn_code: int
    tcp_offset_rpy_mm: list[float]
    tcp_load: list[float]  # kg, then centre of mass mm
    collision_sensitivity: int
    teach_sensitivity: int

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, f"STATE_{self.state}")

    @property
    def mode_name(self) -> str:
        return MODE_NAMES.get(self.mode, f"MODE_{self.mode}")


REPORT_NORMAL_MIN = 133


def decode_report(frame: bytes) -> Report:
    """Parse the fixed head of a normal/rich report frame (the layout of
    ``__handle_report_normal`` in the SDK). Refuses a frame whose length field
    disagrees with its size or whose state/mode/sensitivities are out of range —
    the SDK treats that as a desynchronised stream, and so do we."""
    if len(frame) < REPORT_NORMAL_MIN:
        raise XArmError(f"report frame too short ({len(frame)} bytes)")
    length = struct.unpack(">I", frame[0:4])[0]
    if length != len(frame) and not (length == 233 and len(frame) == 245):
        raise XArmError(f"report length field {length} does not match a {len(frame)}-byte frame")
    state, mode = frame[4] & 0x0F, frame[4] >> 4
    collis, teach = frame[131], frame[132]
    if state > 9 or mode > 11 or collis > 5 or teach > 5:
        raise XArmError("report frame out of range (stream desynchronised?)")
    return Report(
        state=state,
        mode=mode,
        cmd_num=struct.unpack(">H", frame[5:7])[0],
        joints=from_fp32_le(frame[7:35], 7),
        pose_rpy_mm=from_fp32_le(frame[35:59], 6),
        torques=from_fp32_le(frame[59:87], 7),
        brake_bits=frame[87],
        enable_bits=frame[88],
        error_code=frame[89],
        warn_code=frame[90],
        tcp_offset_rpy_mm=from_fp32_le(frame[91:115], 6),
        tcp_load=from_fp32_le(frame[115:131], 4),
        collision_sensitivity=collis,
        teach_sensitivity=teach,
    )


_VERSION_FULL = re.compile(r".*?(\d+),(\d+),(.*?),(.*?),.*?[vV]?(\d+)\.(\d+)\.(\d+)")
_VERSION_BARE = re.compile(r".*?[vV]?(\d+)\.(\d+)\.(\d+)")


@dataclass(frozen=True)
class Version:
    raw: str
    number: tuple[int, int, int]
    axis: int | None = None
    device_type: int | None = None
    serial: str = ""

    @property
    def model(self) -> str:
        if self.axis is None or self.device_type is None:
            return ""
        return DEVICE_TYPES.get((self.axis, self.device_type), f"axis{self.axis}-type{self.device_type}")

    def at_least(self, major: int, minor: int, revision: int) -> bool:
        return self.number >= (major, minor, revision)


def parse_version(raw: bytes | str) -> Version:
    """The GET_VERSION string: ``…<axis>,<type>,<arm sn>,<box sn>,…v<maj>.<min>.<rev>…``
    (the SDK's regex), or just a ``v<maj>.<min>.<rev>`` on older firmware."""
    text = raw.decode("ascii", "replace") if isinstance(raw, bytes) else raw
    text = text.split("\0", 1)[0]
    m = _VERSION_FULL.match(text)
    if m:
        axis, dtype, sn, _box, *num = m.groups()
        return Version(text, tuple(int(n) for n in num), int(axis), int(dtype), sn.strip())  # type: ignore[arg-type]
    m = _VERSION_BARE.match(text)
    if m:
        return Version(text, tuple(int(n) for n in m.groups()))  # type: ignore[arg-type]
    return Version(text, (0, 0, 0))


# --------------------------------------------------------------------------------------
# The client
# --------------------------------------------------------------------------------------


class XArmClient:
    """One persistent connection to the control port, request/reply under a lock.

    Unlike UR's Primary port (where every move is a fresh connection and ~10
    rapid reconnects wedge URControl), the UFACTORY controller expects a client
    to hold its socket; the SDK does. A dropped socket is reopened on the next
    request."""

    def __init__(
        self,
        host: str,
        *,
        port: int = CONTROL_PORT,
        report_port: int = REPORT_PORT,
        timeout: float = 5.0,
    ):
        self.host = host
        self.port = port
        self.report_port = report_port
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._buffer = b""
        self._trans_id = 1
        self._lock = threading.Lock()
        self.last_status = 0

    # -- connection ------------------------------------------------------------------

    def _connect(self) -> socket.socket:
        if self._sock is None:
            try:
                self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            except OSError as exc:
                raise XArmError(f"cannot reach the controller at {self.host}:{self.port}: {exc}") from exc
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._buffer = b""
        return self._sock

    def close(self) -> None:
        with self._lock:
            self._drop()

    def _drop(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._buffer = b""

    # -- request / reply ---------------------------------------------------------------

    def request(self, register: int, payload: bytes = b"", *, timeout: float | None = None) -> Reply:
        """Send one request and wait for its reply (skipping motion-feedback
        frames and stale replies to earlier transactions)."""
        with self._lock:
            sock = self._connect()
            trans_id = self._trans_id
            self._trans_id = self._trans_id % TRANSACTION_ID_MAX + 1
            try:
                sock.sendall(encode_request(trans_id, register, payload))
                reply = self._read_reply(sock, trans_id, register, timeout or self.timeout)
            except XArmError:
                self._drop()
                raise
            except OSError as exc:
                self._drop()
                raise XArmError(f"lost the controller at {self.host}:{self.port}: {exc}") from exc
            self.last_status = reply.status
            return reply

    def _read_reply(self, sock: socket.socket, trans_id: int, register: int, timeout: float) -> Reply:
        deadline = time.monotonic() + timeout
        while True:
            frames, self._buffer = split_frames(self._buffer)
            for i, frame in enumerate(frames):
                if len(frame) > 6 and frame[6] == FEEDBACK_REGISTER:
                    continue
                reply = decode_reply(frame)
                if reply.trans_id != trans_id:
                    continue  # a late reply to a request that already timed out
                if reply.register != register:
                    raise XArmError(f"reply register {reply.register} for request {register}")
                # keep anything after this frame for the next request
                self._buffer = b"".join(frames[i + 1 :]) + self._buffer
                return reply
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise XArmTimeout(f"no reply to register {register} within {timeout:.1f} s")
            sock.settimeout(remaining)
            try:
                chunk = sock.recv(4096)
            except TimeoutError as exc:
                raise XArmTimeout(f"no reply to register {register} within {timeout:.1f} s") from exc
            if not chunk:
                raise XArmError("controller closed the connection")
            self._buffer += chunk

    # -- typed commands ----------------------------------------------------------------

    def get_version(self) -> Version:
        return parse_version(self.request(REG_GET_VERSION).data[:40])

    def get_state(self) -> int:
        return self.request(REG_GET_STATE).data[0]

    def get_cmdnum(self) -> int:
        return struct.unpack(">H", self.request(REG_GET_CMDNUM).data[:2])[0]

    def get_error_warn(self) -> tuple[int, int]:
        data = self.request(REG_GET_ERROR).data
        return data[0], data[1]

    def clean_error(self) -> Reply:
        return self.request(REG_CLEAN_ERR)

    def clean_warn(self) -> Reply:
        return self.request(REG_CLEAN_WAR)

    def motion_enable(self, enable: bool, servo: int = ALL_SERVOS) -> Reply:
        return self.request(REG_MOTION_EN, bytes([servo, int(bool(enable))]), timeout=max(self.timeout, 5.0))

    def set_state(self, state: int) -> Reply:
        return self.request(REG_SET_STATE, bytes([state]))

    def set_mode(self, mode: int, version: Version | None = None) -> Reply:
        # Firmware ≥ 1.10.0 takes a detection-parameter byte after the mode.
        if version is not None and version.at_least(1, 10, 0):
            return self.request(REG_SET_MODE, bytes([mode, 0]))
        return self.request(REG_SET_MODE, bytes([mode]))

    def get_joints(self) -> list[float]:
        return from_fp32_le(self.request(REG_GET_JOINT_POS).data, 7)

    def get_pose_aa(self) -> list[float]:
        """TCP pose, mm + axis-angle (rotation vector) rad."""
        return from_fp32_le(self.request(REG_GET_TCP_POSE_AA).data, 6)

    def get_pose_rpy(self) -> list[float]:
        return from_fp32_le(self.request(REG_GET_TCP_POSE).data, 6)

    def forward_kinematics(self, joints: Sequence[float]) -> list[float]:
        """Pose (mm, RPY rad) of the *active TCP* at ``joints``."""
        return from_fp32_le(self.request(REG_GET_FK, fp32_le(_pad7(joints))).data, 6)

    def set_tcp_offset(self, offset_rpy_mm: Sequence[float]) -> Reply:
        return self.request(REG_SET_TCP_OFFSET, fp32_le(list(offset_rpy_mm)[:6]))

    def move_joints(
        self, joints: Sequence[float], speed: float, acc: float, radius: float | None = None
    ) -> Reply:
        """Queue a joint move (rad, rad/s, rad/s²); ``radius`` (mm) blends into
        the next queued move (MOVE_JOINTB)."""
        if radius is not None and radius > 0:
            return self.request(REG_MOVE_JOINTB, fp32_le([*_pad7(joints), speed, acc, radius]))
        return self.request(REG_MOVE_JOINT, fp32_le([*_pad7(joints), speed, acc, 0.0]))

    def move_home(self, speed: float, acc: float) -> Reply:
        """Queue a joint move to the controller's own home (all joints zero on
        the 850 / xArm)."""
        return self.request(REG_MOVE_HOME, fp32_le([speed, acc, 0.0]))

    def sleep(self, seconds: float) -> Reply:
        """Queue a dwell: the controller waits ``seconds`` before the next queued move."""
        return self.request(REG_SLEEP_INSTT, fp32_le([seconds]))

    def move_line_aa(
        self,
        pose_aa_mm: Sequence[float],
        speed: float,
        acc: float,
        *,
        radius: float = -1.0,
        tool_frame: bool = False,
        only_check: int = 0,
        common: bool = True,
    ) -> Reply:
        """Queue a linear move to ``pose_aa_mm`` (mm + rotation vector rad) at
        ``speed`` mm/s, ``acc`` mm/s². ``common`` uses the general MOVE_LINE form
        (firmware ≥ 1.11.100: blend radius, and ``only_check`` 1–3 asks the
        planner whether the path is clear *without moving* — the reply's third
        data byte is the verdict, 0 = clear). ``common=False`` is the legacy
        MOVE_LINE_AA (no radius, no check)."""
        floats = [*list(pose_aa_mm)[:6], speed, acc, 0.0]
        if common:
            tail = bytes([int(tool_frame), 1, int(only_check)])
            return self.request(
                REG_MOVE_LINE, fp32_le([*floats, radius]) + tail, timeout=max(self.timeout, 10.0)
            )
        return self.request(REG_MOVE_LINE_AA, fp32_le(floats) + bytes([int(tool_frame), 0]))

    def set_controller_digital(self, pin: int, value: bool) -> Reply:
        """Control-box digital output ``pin`` 0..15 (CGPIO_SET_DIGIT: a 16-bit
        mask word — high byte selects the pin, low byte sets it)."""
        if not 0 <= pin <= 15:
            raise ValueError("pin must be 0..15")
        bit = pin if pin < 8 else pin - 8
        word = (0x0100 << bit) | ((0x0001 << bit) if value else 0)
        words = [word] if pin < 8 else [0, word]
        return self.request(REG_CGPIO_SET_DIGIT, struct.pack(f">{len(words)}H", *words))

    # -- report stream -----------------------------------------------------------------

    def read_report(self, *, timeout: float | None = None) -> Report:
        """Open the report port, read one whole frame, close. Read-only; a
        report connection never interferes with the control socket."""
        timeout = timeout or self.timeout
        try:
            with socket.create_connection((self.host, self.report_port), timeout=timeout) as sock:
                head = _recv_exact(sock, 4, timeout)
                size = struct.unpack(">I", head)[0]
                if size == 233:
                    size = 245  # the SDK's quirk: some firmware announces 233, sends 245
                if not REPORT_NORMAL_MIN <= size <= 4096:
                    raise XArmError(f"report frame size {size} out of range")
                return decode_report(head + _recv_exact(sock, size - 4, timeout))
        except OSError as exc:
            if isinstance(exc, XArmError):
                raise
            raise XArmError(
                f"cannot read the report stream at {self.host}:{self.report_port}: {exc}"
            ) from exc


def _recv_exact(sock: socket.socket, n: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    out = b""
    while len(out) < n:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise XArmError("report stream timed out")
        sock.settimeout(remaining)
        chunk = sock.recv(n - len(out))
        if not chunk:
            raise XArmError("report stream closed")
        out += chunk
    return out


def _pad7(joints: Sequence[float]) -> list[float]:
    vals = [float(j) for j in joints][:7]
    return vals + [0.0] * (7 - len(vals))


__all__ = [
    "CONTROL_PORT",
    "CONTROLLER_ERRORS",
    "REPORT_PORT",
    "Report",
    "Reply",
    "Version",
    "XArmClient",
    "XArmError",
    "decode_reply",
    "decode_report",
    "encode_request",
    "error_text",
    "parse_version",
    "split_frames",
]
