"""UFACTORY 850 (and xArm 5/6/7, Lite 6) as a :class:`urctl.controller.Controller`.

The second vendor behind the seam. :class:`UFactoryArm` speaks the control box's
own protocol (:mod:`urctl.xarm`, stdlib) and answers the same calls with the
same result dicts as :class:`urctl.robot.Robot`, so the tool registry, both MCP
servers and the CLI drive it unchanged (``UR_PLATFORM=ufactory`` /
``--platform ufactory``).

Units at this surface are the toolkit's — **metres and rotation vectors**, like
a UR pose; the wire wants millimetres, and the TCP offset wants roll/pitch/yaw,
so this module converts. Every motion goes through the same
:class:`~urctl.safety.SafetyEnvelope` and :class:`~urctl.audit.AuditLog` as a UR.

Where UFACTORY differs from a UR, and what that means here:

* **Moves queue.** A UR runs one URScript program at a time (a new one replaces
  the old); the UFACTORY controller appends each move to a queue. A path is the
  legs queued back to back with blend radii; ``stop`` (state 4) flushes the
  queue. Completion is read from the state register (1 = moving), not from a
  ``textmsg`` marker.
* **No power-on / brake-release pair.** ``bring_up`` = clear error + warning,
  enable every servo, position mode, state 0 (ready). An E-stop (errors 1–3)
  must be released by hand first; ``bring_up`` says so.
* **The reach check is the controller's planner** (``only_check_type`` on the
  linear move, firmware ≥ 1.11.100): it plans the path from the live pose and
  reports self-collision / joint-limit / Cartesian-limit / overspeed without
  moving. Older firmware falls back to the datasheet sphere (0.85 m on the 850).
* **No script, no programs over the network.** UFACTORY Studio's Blockly /
  Python projects are not reachable through the control port, so ``run_script``
  and ``load_program`` answer ``ok: false`` with that reason. ``pause`` /
  ``play`` pause and resume the motion queue.
* **Freedrive** is the controller's joint-teaching mode (mode 2); it lasts until
  ``freedrive(False)`` — there is no program to keep alive, so no hold timer.
"""

from __future__ import annotations

import math
import os
import time
from collections.abc import Sequence

from .audit import AuditLog
from .config import PLATFORM_UFACTORY, RobotConfig
from .pose import Transform, matrix_to_rotvec, rotvec_to_matrix
from .robot import (
    DEFAULT_ACCELERATION,
    DEFAULT_TCP_ACCELERATION,
    DEFAULT_TCP_VELOCITY,
    DEFAULT_VELOCITY,
    LANDING_TOLERANCE,
    TCP_LANDING_TOLERANCE,
)
from .safety import SafetyEnvelope, SafetyVerdict
from .xarm import (
    ESTOP_ERRORS,
    MODE_JOINT_TEACH,
    MODE_POSITION,
    STATE_MOVING,
    STATE_PAUSED,
    STATE_READY,
    STATE_STOPPED,
    Report,
    Version,
    XArmClient,
    XArmError,
    XArmTimeout,
    error_text,
    warning_text,
)

DEFAULT_MODEL = "UF850"
HOME_JOINTS = [0.0] * 6  # the controller's own home (MOVE_HOME) on the 850 / xArm 6
POLL_S = 0.05
# Firmware that takes the general MOVE_LINE (axis-angle + blend radius + only_check).
COMMON_MOVE_LINE = (1, 11, 100)
PLANNER_SPEED = 24  # a planner verdict: "Speed Exceeds Limit"
PLANNER_INVALID = 25  # an INVALID or short reply to a check reads as "Planning Error"

ENV_CONTROL_PORT = "UFACTORY_CONTROL_PORT"
ENV_REPORT_PORT = "UFACTORY_REPORT_PORT"


# --------------------------------------------------------------------------------------
# Rotation conventions: UFACTORY's roll/pitch/yaw ↔ the toolkit's rotation vector
# --------------------------------------------------------------------------------------


def rpy_to_matrix(roll: float, pitch: float, yaw: float):
    """Fixed-axis X-Y-Z (``R = Rz(yaw)·Ry(pitch)·Rx(roll)``). The controller's
    TCP pose in RPY and in axis-angle (GET_TCP_POSE / GET_TCP_POSE_AA) agree under
    this convention; the URSim-free integration test asserts it on the sim."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def matrix_to_rpy(m) -> list[float]:
    """Inverse of :func:`rpy_to_matrix`; at pitch = ±90° yaw carries the whole
    rotation about Z (roll = 0)."""
    sp = max(-1.0, min(1.0, -m[2][0]))
    pitch = math.asin(sp)
    if abs(sp) < 1.0 - 1e-9:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    return [roll, pitch, yaw]


def rotvec_to_rpy(rv: Sequence[float]) -> list[float]:
    return matrix_to_rpy(rotvec_to_matrix(rv))


def rpy_to_rotvec(rpy: Sequence[float]) -> list[float]:
    return list(matrix_to_rotvec(rpy_to_matrix(*rpy)))


def pose_to_wire(pose_m: Sequence[float]) -> list[float]:
    """Toolkit pose (m + rotvec) → the linear move's payload (mm + rotvec)."""
    return [pose_m[0] * 1000.0, pose_m[1] * 1000.0, pose_m[2] * 1000.0, *pose_m[3:6]]


def pose_from_wire(pose_mm: Sequence[float]) -> list[float]:
    return [pose_mm[0] / 1000.0, pose_mm[1] / 1000.0, pose_mm[2] / 1000.0, *pose_mm[3:6]]


def offset_to_wire(tcp_m: Sequence[float]) -> list[float]:
    """Toolkit TCP offset (m + rotvec) → SET_TCP_OFFSET's (mm + RPY)."""
    return [tcp_m[0] * 1000.0, tcp_m[1] * 1000.0, tcp_m[2] * 1000.0, *rotvec_to_rpy(tcp_m[3:6])]


def offset_from_wire(offset_rpy_mm: Sequence[float]) -> list[float]:
    o = offset_rpy_mm
    return [o[0] / 1000.0, o[1] / 1000.0, o[2] / 1000.0, *rpy_to_rotvec(o[3:6])]


# --------------------------------------------------------------------------------------
# The controller
# --------------------------------------------------------------------------------------


class UFactoryArm:
    """A UFACTORY arm behind the toolkit's :class:`~urctl.controller.Controller` seam."""

    def __init__(
        self,
        config: RobotConfig | None = None,
        *,
        safety: SafetyEnvelope | None = None,
        audit: AuditLog | None = None,
        dry_run: bool = False,
        client: XArmClient | None = None,
    ):
        self.config = config or RobotConfig.from_env()
        model = self.config.robot_model or DEFAULT_MODEL
        self.safety = safety or SafetyEnvelope.for_model(model, max_reach=self.config.max_reach)
        self.audit = audit or AuditLog()
        self.dry_run = dry_run
        self.client = client or XArmClient(
            self.config.host,
            port=_env_port(ENV_CONTROL_PORT, 502),
            report_port=_env_port(ENV_REPORT_PORT, 30001),
            timeout=self.config.timeout,
        )
        self._version: Version | None = None
        self.last_planner: list[dict] = []  # the last refused check's reasons

    # ----- plumbing --------------------------------------------------------------------

    def close(self) -> None:
        self.client.close()

    def __enter__(self) -> UFactoryArm:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _log(self, action: str, args: dict, *, ok: bool, result: dict | None = None, safety=None) -> dict:
        rec = self.audit.record(
            action,
            host=self.config.host,
            args=args,
            ok=ok,
            result=result or {},
            safety=safety,
            dry_run=self.dry_run,
        )
        out = {"action": action, "ok": ok, "dry_run": self.dry_run, "host": self.config.host}
        if safety is not None:
            out["safety"] = safety
        out.update(result or {})
        out["ts"] = rec.ts
        return out

    def _unreachable(self, action: str, args: dict, exc: Exception) -> dict:
        return self._log(action, args, ok=False, result={"error": str(exc)})

    def version(self) -> Version:
        if self._version is None:
            self._version = self.client.get_version()
        return self._version

    @property
    def axes(self) -> int:
        try:
            return self.version().axis or self.safety.num_joints
        except XArmError:
            return self.safety.num_joints

    def _common_move_line(self) -> bool:
        return self.version().at_least(*COMMON_MOVE_LINE)

    def _robot_mode(self, report: Report) -> str:
        """The UR-style mode word the envelope gates motion on (``RUNNING``)."""
        if report.error_code:
            return "ERROR"
        if (report.enable_bits & ((1 << self.axes) - 1)) != (1 << self.axes) - 1:
            return "DISABLED"
        if report.state >= STATE_STOPPED:
            return "STOPPED"
        return "RUNNING"

    @staticmethod
    def _safety_mode(report: Report) -> str:
        if report.error_code in ESTOP_ERRORS:
            return "EMERGENCY_STOP"
        if report.error_code:
            return "PROTECTIVE_STOP"
        return "NORMAL"

    def _mode_for_motion(self) -> str | None:
        """``RUNNING`` when the arm can take a move — re-arming a plain stop (state
        4 with no error) the way a UR takes the next program after ``stop``."""
        if self.dry_run:
            return None
        report = self.client.read_report()
        mode = self._robot_mode(report)
        if mode == "STOPPED" and report.mode == MODE_POSITION:
            self.client.set_state(STATE_READY)
            mode = "RUNNING"
        if report.mode != MODE_POSITION and mode == "RUNNING":
            # no "RUNNING" in this text: the envelope gates on that substring
            return f"{report.mode_name} mode (freedrive on? call freedrive(False))"
        return mode

    # ----- observation -----------------------------------------------------------------

    def get_state(self, **_ignored) -> dict:
        if self.dry_run:
            return self._log("get_state", {}, ok=True, result={"robot_mode": "(dry-run)"})
        try:
            report = self.client.read_report()
            tcp = pose_from_wire(self.client.get_pose_aa())
            version = self.version()
        except XArmError as exc:
            return self._unreachable("get_state", {}, exc)
        result = {
            "robot_mode": self._robot_mode(report),
            "safety_mode": self._safety_mode(report),
            "program_state": report.state_name,
            "control_mode": report.mode_name,
            "running": self._robot_mode(report) == "RUNNING",
            "joints": report.joints[: self.axes],
            "tcp": tcp,
            "tcp_offset": offset_from_wire(report.tcp_offset_rpy_mm),
            "joint_torques": report.torques[: self.axes],
            "queued_moves": report.cmd_num,
            "error_code": report.error_code,
            "error": error_text(report.error_code),
            "warn_code": report.warn_code,
            "warning": warning_text(report.warn_code),
            "vendor": "ufactory",
            "model": version.model or self.safety.model,
            "firmware": ".".join(str(n) for n in version.number),
        }
        return self._log("get_state", {}, ok=True, result=result)

    def get_flange_pose(self, **_ignored) -> dict:
        """Flange = active TCP ∘ offset⁻¹, from one TCP read and the report's offset."""
        if self.dry_run:
            flange = [0.5, 0.0, 0.5, math.pi, 0.0, 0.0]
            return self._log(
                "get_flange_pose",
                {},
                ok=True,
                result={"flange": flange, "tcp": flange, "tcp_offset": [0.0] * 6, "dry_run": True},
            )
        try:
            tcp = pose_from_wire(self.client.get_pose_aa())
            report = self.client.read_report()
        except XArmError as exc:
            return self._unreachable("get_flange_pose", {}, exc)
        offset = offset_from_wire(report.tcp_offset_rpy_mm)
        flange = Transform.from_pose(tcp).compose(Transform.from_pose(offset).inverse()).to_pose()
        result = {
            "flange": flange,
            "tcp": tcp,
            "tcp_offset": offset,
            "joints": report.joints[: self.axes],
            "source": "ufactory_control",
        }
        return self._log("get_flange_pose", {}, ok=True, result=result)

    def get_tcp_offset(self) -> dict:
        """The active TCP offset, metres + rotation vector (the report carries mm + RPY)."""
        if self.dry_run:
            return self._log("get_tcp_offset", {}, ok=True, result={"tcp_offset": [0.0] * 6, "dry_run": True})
        try:
            report = self.client.read_report()
        except XArmError as exc:
            return self._unreachable("get_tcp_offset", {}, exc)
        return self._log(
            "get_tcp_offset", {}, ok=True, result={"tcp_offset": offset_from_wire(report.tcp_offset_rpy_mm)}
        )

    def set_tcp_offset(self, offset: list[float], *, timeout: float = 30.0) -> dict:
        """SET_TCP_OFFSET (mm + RPY on the wire) after the queue drains — the SDK waits
        for motion to finish first too — then read it back from the report. It stays
        until the next write (UFACTORY Studio's TCP setting included)."""
        off = [float(v) for v in offset]
        if len(off) != 6 or not all(math.isfinite(v) for v in off):
            raise ValueError("offset must be 6 finite numbers [x, y, z, rx, ry, rz]")
        args = {"offset": off}
        if self.dry_run:
            return self._log("set_tcp_offset", args, ok=True, result={"reply": "(dry-run)"})
        try:
            if self.client.get_state() in (STATE_MOVING, STATE_PAUSED) or self.client.get_cmdnum():
                self._wait_idle(timeout)
            self._write_tcp_offset(off)
            deadline = time.monotonic() + 2.0
            while True:
                active = offset_from_wire(self.client.read_report().tcp_offset_rpy_mm)
                ok = _offset_close(active, off)
                if ok or time.monotonic() > deadline:
                    break
                time.sleep(0.1)
        except XArmError as exc:
            return self._unreachable("set_tcp_offset", args, exc)
        return self._log("set_tcp_offset", args, ok=ok, result={"tcp_offset": active})

    # ----- power / recovery ------------------------------------------------------------

    def bring_up(self, *, timeout: float = 30.0) -> dict:
        if self.dry_run:
            return self._log("bring_up", {}, ok=True, result={"reply": "(dry-run)"})
        try:
            err, _warn = self.client.get_error_warn()
            if err in ESTOP_ERRORS:
                return self._log(
                    "bring_up",
                    {},
                    ok=False,
                    result={
                        "error_code": err,
                        "error": error_text(err),
                        "reply": "release the emergency stop by hand, then bring up again",
                    },
                )
            self.client.clean_error()
            self.client.clean_warn()
            unanswered = False
            try:
                self.client.motion_enable(True)
            except XArmTimeout:
                # UFACTORY's simulator (v2.4.0) acts on MOTION_EN without replying, and
                # the SDK carries on past the timeout too. The report's enable bits
                # below are the verdict either way.
                unanswered = True
            self.client.set_mode(MODE_POSITION, self.version())
            self.client.set_state(STATE_READY)
            deadline = time.monotonic() + timeout
            while True:
                report = self.client.read_report()
                mode = self._robot_mode(report)
                if mode == "RUNNING" or time.monotonic() > deadline:
                    break
                time.sleep(POLL_S * 4)
        except XArmError as exc:
            return self._unreachable("bring_up", {}, exc)
        result = {
            "robot_mode": mode,
            "error_code": report.error_code,
            "error": error_text(report.error_code),
            "motion_enable_unanswered": unanswered,
        }
        return self._log("bring_up", {}, ok=mode == "RUNNING", result=result)

    def power_off(self) -> dict:
        """Disable the servos (the brakes hold the arm)."""
        if self.dry_run:
            return self._log("power_off", {}, ok=True, result={"reply": "(dry-run)"})
        try:
            reply = self.client.motion_enable(False)
        except XArmError as exc:
            return self._unreachable("power_off", {}, exc)
        return self._log("power_off", {}, ok=not reply.invalid, result={"status": reply.status})

    def stop(self) -> dict:
        """Stop now and flush the motion queue (state 4), as the SDK's ``emergency_stop``
        does: repeat until the controller reports it stopped."""
        if self.dry_run:
            return self._log("stop", {}, ok=True, result={"reply": "(dry-run)"})
        try:
            deadline = time.monotonic() + 3.0
            self.client.set_state(STATE_STOPPED)
            while self.client.get_state() < STATE_STOPPED and time.monotonic() < deadline:
                self.client.set_state(STATE_STOPPED)
                time.sleep(0.1)
            state = self.client.get_state()
        except XArmError as exc:
            return self._unreachable("stop", {}, exc)
        return self._log("stop", {}, ok=state >= STATE_STOPPED, result={"state": state})

    # ----- motion helpers ----------------------------------------------------------------

    def _wait_idle(self, timeout: float) -> dict:
        """Poll until the queue drains and the arm stops moving. Two idle reads in a
        row (state not MOVING, nothing queued) count as done, as the SDK's
        ``wait_move`` does; a latched error or a stop ends the wait as a failure."""
        deadline = time.monotonic() + timeout
        idle = 0
        while time.monotonic() < deadline:
            state = self.client.get_state()
            status = self.client.last_status
            if state >= STATE_STOPPED or status & 0x40:
                err, _ = self.client.get_error_warn()
                return {"done": False, "state": state, "error_code": err, "error": error_text(err)}
            if state in (STATE_MOVING, STATE_PAUSED) or self.client.get_cmdnum() > 0:
                idle = 0
            else:
                idle += 1
                if idle >= 2:
                    return {"done": True, "state": state}
            time.sleep(POLL_S)
        return {"done": False, "timeout": True}

    def _failure(self, reply, what: str) -> dict | None:
        """``None`` when the controller accepted a queued command; otherwise why not."""
        if reply.has_error:
            err, _ = self.client.get_error_warn()
            return self._protective({"error_code": err, "error": f"{what} refused: {error_text(err)}"})
        if not reply.ready_to_move:
            return {"error": f"{what} refused: the arm is not ready to move (bring_up)"}
        if reply.invalid:
            return {"error": f"{what} refused by the controller (invalid)"}
        return None

    def _protective(self, result: dict) -> dict:
        code = result.get("error_code")
        if code:
            result["protective_stop"] = code not in ESTOP_ERRORS
            result["emergency_stop"] = code in ESTOP_ERRORS
        return result

    def path_is_clear(
        self, poses: Sequence[Sequence[float]], *, tcp: Sequence[float] | None = None
    ) -> list[bool | None]:
        """The controller's planner on a chain of linear moves from the live pose,
        nothing moved (``only_check_type`` 1 → 3 → 2: start from the actual state,
        chain, restore). The first check is always 1: 2 and 3 plan from the
        controller's intermediate state, which goes stale as soon as the arm moves.
        ``None`` per pose when the firmware can't check."""
        self.last_planner = []
        if self.dry_run or not poses:
            return [None] * len(poses)
        try:
            if not self._common_move_line():
                return [None] * len(poses)
            if tcp is not None:
                self._write_tcp_offset(tcp)
            out: list[bool | None] = []
            for i, pose in enumerate(poses):
                check = 1 if i == 0 else 2 if i == len(poses) - 1 else 3
                code = self._check(pose, check)
                if code == PLANNER_SPEED and i == 0:
                    # From rest, "speed exceeds limit" is the planner's stale state, not
                    # the path: UFACTORY's simulator answers it to the first check after a
                    # finished move and clears it on the same check asked again.
                    code = self._check(pose, check)
                if code:
                    self.last_planner.append({"code": code, "reason": error_text(code)})
                out.append(code == 0)
            return out
        except XArmError:
            return [None] * len(poses)

    def _write_tcp_offset(self, offset: Sequence[float]) -> None:
        """SET_TCP_OFFSET, then re-arm: the firmware drops to state 5 on a configuration
        write and refuses every check (255) and move until ``set_state(0)``. An arm that
        was already stopped stays stopped."""
        armed = self.client.get_state() < STATE_STOPPED
        self.client.set_tcp_offset(offset_to_wire(offset))
        if armed:
            self.client.set_state(STATE_READY)

    def _check(self, pose: Sequence[float], check: int) -> int:
        """One planner check: 0 clear, else the controller error code it would raise."""
        reply = self.client.move_line_aa(pose_to_wire(pose), 100.0, 1000.0, only_check=check)
        if reply.invalid or len(reply.data) < 3:
            return PLANNER_INVALID
        return reply.data[2]

    # ----- motion ----------------------------------------------------------------------

    def move_joints(
        self,
        target: list[float],
        *,
        velocity: float = DEFAULT_VELOCITY,
        acceleration: float = DEFAULT_ACCELERATION,
        wait: bool = True,
        timeout: float = 30.0,
    ) -> dict:
        args = {"target": target, "velocity": velocity, "acceleration": acceleration}
        try:
            mode = self._mode_for_motion()
        except XArmError as exc:
            return self._unreachable("move_joints", args, exc)
        verdict: SafetyVerdict = self.safety.validate_move_joints(
            target, velocity=velocity, acceleration=acceleration, robot_mode=mode
        )
        if not verdict.ok:
            return self._log("move_joints", args, ok=False, safety=verdict.as_dict())
        if self.dry_run:
            return self._log(
                "move_joints", args, ok=True, safety=verdict.as_dict(), result={"reply": "(dry-run)"}
            )
        try:
            reply = self.client.move_joints(target, velocity, acceleration)
            refused = self._failure(reply, "move_joints")
            if refused:
                return self._log("move_joints", args, ok=False, safety=verdict.as_dict(), result=refused)
            if not wait:
                return self._log(
                    "move_joints", args, ok=True, safety=verdict.as_dict(), result={"waited": False}
                )
            done = self._wait_idle(timeout)
            landed = self.client.get_joints()[: self.axes]
        except XArmError as exc:
            return self._unreachable("move_joints", args, exc)
        ok = (
            done["done"] and max(abs(a - b) for a, b in zip(landed, target, strict=False)) < LANDING_TOLERANCE
        )
        result = self._protective({"landed": landed if done["done"] else None, "waited": True, **done})
        return self._log("move_joints", args, ok=ok, safety=verdict.as_dict(), result=result)

    def move_home(self, **kwargs) -> dict:
        return self.move_joints(list(HOME_JOINTS[: self.axes]), **kwargs)

    def move_tcp(
        self,
        pose: list[float],
        *,
        relative: bool = False,
        velocity: float = DEFAULT_TCP_VELOCITY,
        acceleration: float = DEFAULT_TCP_ACCELERATION,
        wait: bool = True,
        timeout: float = 30.0,
        tcp: list[float] | None = None,
    ) -> dict:
        """Linear move. ``pose`` is metres + rotation vector; ``relative`` adds a
        base-frame delta to the live TCP (translation added, rotation applied in the
        base frame, URScript ``pose_add``). ``tcp`` sets the controller's TCP offset
        first (``[0]*6`` = the flange); it stays set, as ``set_tcp`` does on a UR."""
        args: dict = {"pose": pose, "relative": relative, "velocity": velocity, "acceleration": acceleration}
        if tcp is not None:
            tcp = [float(v) for v in tcp]
            if len(tcp) != 6 or not all(math.isfinite(v) for v in tcp):
                raise ValueError("tcp must be 6 finite numbers [x, y, z, rx, ry, rz]")
            args["tcp"] = tcp
        try:
            mode = self._mode_for_motion()
        except XArmError as exc:
            return self._unreachable("move_tcp", args, exc)
        target = list(pose)
        clear = None
        self.last_planner = []
        if (
            not self.dry_run
            and len(pose) == 6
            and all(isinstance(v, int | float) and math.isfinite(v) for v in pose)
        ):
            try:
                if tcp is not None:
                    self._write_tcp_offset(tcp)
                if relative:
                    live = pose_from_wire(self.client.get_pose_aa())
                    rot = matrix_to_rotvec(_matmul(rotvec_to_matrix(pose[3:6]), rotvec_to_matrix(live[3:6])))
                    target = [live[0] + pose[0], live[1] + pose[1], live[2] + pose[2], *rot]
                clear = self.path_is_clear([target])[0]
            except XArmError as exc:
                return self._unreachable("move_tcp", args, exc)
        verdict = self.safety.validate_move_tcp(
            pose,
            velocity=velocity,
            acceleration=acceleration,
            relative=relative,
            robot_mode=mode,
            ik_reachable=None if relative else clear,
        )
        if verdict.ok and relative and clear is False:
            verdict = SafetyVerdict(
                ok=False,
                violations=[
                    *verdict.violations,
                    _violation("ik_reach", "the controller's planner refuses this path"),
                ],
            )
        if not verdict.ok:
            refused = {"planner": self.last_planner} if self.last_planner else None
            return self._log("move_tcp", args, ok=False, safety=verdict.as_dict(), result=refused)
        if self.dry_run:
            return self._log(
                "move_tcp", args, ok=True, safety=verdict.as_dict(), result={"reply": "(dry-run)"}
            )
        try:
            reply = self.client.move_line_aa(
                pose_to_wire(target),
                velocity * 1000.0,
                acceleration * 1000.0,
                common=self._common_move_line(),
            )
            refused = self._failure(reply, "move_tcp")
            if refused:
                return self._log("move_tcp", args, ok=False, safety=verdict.as_dict(), result=refused)
            if not wait:
                return self._log(
                    "move_tcp", args, ok=True, safety=verdict.as_dict(), result={"waited": False}
                )
            done = self._wait_idle(timeout)
            landed = pose_from_wire(self.client.get_pose_aa())
        except XArmError as exc:
            return self._unreachable("move_tcp", args, exc)
        ok = done["done"] and _dist(landed, target) < TCP_LANDING_TOLERANCE
        result = self._protective(
            {"landed": landed if done["done"] else None, "target": target, "waited": True, **done}
        )
        return self._log("move_tcp", args, ok=ok, safety=verdict.as_dict(), result=result)

    def move_tcp_path(
        self,
        legs: Sequence[dict],
        *,
        tcp: list[float] | None = None,
        timeout: float = 120.0,
        gripper_first: int | None = None,
    ) -> dict:
        """Queue every leg (blends as radii, dwells as queued sleeps) and wait once.
        Legs are checked as one chain by the controller's planner first; nothing
        is queued if any leg fails the envelope or the planner. Per-leg landing
        is not observable on a queue, so a leg reports ``queued`` and the last
        one the final pose."""
        if not legs:
            raise ValueError("legs must not be empty")
        if gripper_first is not None or any(leg.get("gripper") for leg in legs):
            raise ValueError("no gripper driver for UFACTORY arms yet: drive the gripper separately")
        norm = []
        for idx, leg in enumerate(legs):
            pose = [float(v) for v in leg["pose"]]
            if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
                raise ValueError(f"legs[{idx}].pose must be 6 finite numbers")
            dwell = float(leg.get("dwell_s", 0.0) or 0.0)
            if not math.isfinite(dwell) or not 0.0 <= dwell <= 60.0:
                raise ValueError(f"legs[{idx}].dwell_s must be within 0..60 s")
            blend = float(leg.get("blend_m", 0.0) or 0.0)
            if not math.isfinite(blend) or not 0.0 <= blend <= 0.1:
                raise ValueError(f"legs[{idx}].blend_m must be within 0..0.1 m")
            norm.append(
                {
                    "pose": pose,
                    "velocity": float(leg.get("velocity", DEFAULT_TCP_VELOCITY)),
                    "acceleration": float(leg.get("acceleration", DEFAULT_TCP_ACCELERATION)),
                    "dwell_s": dwell,
                    "blend_m": blend if idx < len(legs) - 1 and dwell == 0.0 else 0.0,
                }
            )
        args: dict = {"legs": norm}
        if tcp is not None:
            args["tcp"] = [float(v) for v in tcp]
        try:
            mode = self._mode_for_motion()
            clear = self.path_is_clear([leg["pose"] for leg in norm], tcp=tcp)
        except XArmError as exc:
            return self._unreachable("move_tcp_path", args, exc)
        violations = []
        for idx, leg in enumerate(norm):
            v = self.safety.validate_move_tcp(
                leg["pose"],
                velocity=leg["velocity"],
                acceleration=leg["acceleration"],
                robot_mode=mode,
                ik_reachable=clear[idx],
            )
            violations += [{"leg": idx, **d} for d in v.as_dict()["violations"]]
        safety = {"ok": not violations, "violations": violations}
        if violations:
            refused = {"planner": self.last_planner} if self.last_planner else None
            return self._log("move_tcp_path", args, ok=False, safety=safety, result=refused)
        if self.dry_run:
            return self._log("move_tcp_path", args, ok=True, safety=safety, result={"reply": "(dry-run)"})
        try:
            common = self._common_move_line()
            for idx, leg in enumerate(norm):
                reply = self.client.move_line_aa(
                    pose_to_wire(leg["pose"]),
                    leg["velocity"] * 1000.0,
                    leg["acceleration"] * 1000.0,
                    radius=leg["blend_m"] * 1000.0 if leg["blend_m"] > 0 else -1.0,
                    common=common,
                )
                refused = self._failure(reply, f"leg {idx}")
                if refused:
                    self.stop()
                    return self._log("move_tcp_path", args, ok=False, safety=safety, result=refused)
                if leg["dwell_s"] > 0:
                    self.client.sleep(leg["dwell_s"])
            done = self._wait_idle(timeout)
            final = pose_from_wire(self.client.get_pose_aa())
        except XArmError as exc:
            return self._unreachable("move_tcp_path", args, exc)
        hit = done["done"] and _dist(final, norm[-1]["pose"]) < TCP_LANDING_TOLERANCE
        leg_results = [
            {"pose": leg["pose"], "dwell_s": leg["dwell_s"], "blend_m": leg["blend_m"], "queued": True}
            for leg in norm
        ]
        leg_results[-1].update({"landed": final if done["done"] else None, "ok": hit})
        result = self._protective(
            {
                "legs": leg_results,
                "completed_legs": len(norm) if hit else 0,
                "landed": final if done["done"] else None,
                "waited": True,
                **done,
            }
        )
        return self._log("move_tcp_path", args, ok=hit, safety=safety, result=result)

    def move_trajectory(
        self,
        waypoints: Sequence[Sequence[float]],
        *,
        velocity: float = DEFAULT_VELOCITY,
        acceleration: float = DEFAULT_ACCELERATION,
        blend_radius: float = 0.0,
        wait: bool = True,
        timeout: float = 120.0,
    ) -> dict:
        """Queue joint waypoints, blending (``blend_radius`` metres) all but the last."""
        args = {
            "waypoints": [list(w) for w in waypoints],
            "velocity": velocity,
            "acceleration": acceleration,
            "blend_radius": blend_radius,
            "n": len(waypoints),
        }
        if not waypoints:
            return self._log("move_trajectory", args, ok=False, result={"error": "no waypoints"})
        try:
            mode = self._mode_for_motion()
        except XArmError as exc:
            return self._unreachable("move_trajectory", args, exc)
        for idx, wp in enumerate(waypoints):
            v = self.safety.validate_move_joints(
                list(wp), velocity=velocity, acceleration=acceleration, robot_mode=mode if idx == 0 else None
            )
            if not v.ok:
                return self._log("move_trajectory", args, ok=False, safety={"waypoint": idx, **v.as_dict()})
        if self.dry_run:
            return self._log("move_trajectory", args, ok=True, result={"reply": "(dry-run)"})
        try:
            for idx, wp in enumerate(waypoints):
                last = idx == len(waypoints) - 1
                radius = None if last or blend_radius <= 0 else blend_radius * 1000.0
                reply = self.client.move_joints(list(wp), velocity, acceleration, radius=radius)
                refused = self._failure(reply, f"waypoint {idx}")
                if refused:
                    self.stop()
                    return self._log("move_trajectory", args, ok=False, result=refused)
            if not wait:
                return self._log("move_trajectory", args, ok=True, result={"waited": False})
            done = self._wait_idle(timeout)
            landed = self.client.get_joints()[: self.axes]
        except XArmError as exc:
            return self._unreachable("move_trajectory", args, exc)
        ok = (
            done["done"]
            and max(abs(a - b) for a, b in zip(landed, waypoints[-1], strict=False)) < LANDING_TOLERANCE
        )
        result = self._protective({"landed": landed if done["done"] else None, "waited": True, **done})
        return self._log("move_trajectory", args, ok=ok, result=result)

    def freedrive(self, enable: bool, **_ignored) -> dict:
        """Joint-teaching mode (hand-guiding) on; position mode back off."""
        args = {"enable": enable}
        if self.dry_run:
            return self._log("freedrive", args, ok=True, result={"reply": "(dry-run)"})
        want = MODE_JOINT_TEACH if enable else MODE_POSITION
        try:
            # The controller can refuse the mode outright (INVALID): UFACTORY's simulator
            # does for joint teaching. Say so now instead of waiting out the poll.
            refused = self.client.set_mode(want, self.version()).invalid
            self.client.set_state(STATE_READY)
            deadline = time.monotonic() + (0.0 if refused else 2.0)
            report = self.client.read_report()
            while report.mode != want and time.monotonic() < deadline:
                time.sleep(0.1)
                report = self.client.read_report()
        except XArmError as exc:
            return self._unreachable("freedrive", args, exc)
        confirmed = report.mode == want
        result = {"confirmed": confirmed, "refused": refused, "control_mode": report.mode_name}
        if refused:
            result["error"] = f"the controller refused {'joint-teaching' if enable else 'position'} mode"
        return self._log("freedrive", args, ok=confirmed, result=result)

    # ----- programs / I/O --------------------------------------------------------------

    def _unsupported(self, action: str, args: dict, why: str) -> dict:
        return self._log(action, args, ok=False, result={"error": why, "supported": False})

    def load_program(self, name: str) -> dict:
        return self._unsupported(
            "load_program", {"name": name}, "UFACTORY Studio projects are not reachable over the control port"
        )

    def run_script(self, script: str, **_ignored) -> dict:
        return self._unsupported(
            "run_script", {"chars": len(script)}, "UFACTORY controllers take no script over the control port"
        )

    def play(self) -> dict:
        """Resume a paused motion queue."""
        return self._set_state("play", STATE_READY)

    def pause(self) -> dict:
        """Pause the motion queue (state 3); ``play`` resumes it."""
        return self._set_state("pause", STATE_PAUSED)

    def _set_state(self, action: str, state: int) -> dict:
        if self.dry_run:
            return self._log(action, {}, ok=True, result={"reply": "(dry-run)"})
        try:
            reply = self.client.set_state(state)
        except XArmError as exc:
            return self._unreachable(action, {}, exc)
        return self._log(action, {}, ok=not reply.invalid, result={"state": self.client.get_state()})

    def set_digital_output(self, pin: int, value: bool) -> dict:
        args = {"pin": pin, "value": bool(value)}
        if self.dry_run:
            return self._log("set_digital_output", args, ok=True, result={"reply": "(dry-run)"})
        try:
            reply = self.client.set_controller_digital(int(pin), bool(value))
        except (XArmError, ValueError) as exc:
            return self._log("set_digital_output", args, ok=False, result={"error": str(exc)})
        return self._log("set_digital_output", args, ok=not reply.invalid)

    def popup(self, text: str) -> dict:
        return self._unsupported("popup", {"text": text}, "UFACTORY controllers have no pendant popup")

    def gripper(self, action: str, **kwargs) -> dict:
        return self._unsupported(
            "gripper", {"action": action, **kwargs}, "no gripper driver for UFACTORY arms yet"
        )

    def set_speed_override(self, fraction: float) -> dict:
        return self._unsupported(
            "set_speed_override", {"fraction": fraction}, "not implemented for UFACTORY arms"
        )

    def dashboard_command(self, command: str) -> dict:
        return self._unsupported(
            "dashboard_command", {"command": command}, "UFACTORY controllers have no Dashboard"
        )

    def rtde_state(self, *, deep: bool = False) -> dict:
        """The report frame, decoded — the nearest thing to RTDE."""
        if self.dry_run:
            return self._log("rtde_state", {"deep": deep}, ok=True, result={"reply": "(dry-run)"})
        try:
            r = self.client.read_report()
        except XArmError as exc:
            return self._unreachable("rtde_state", {"deep": deep}, exc)
        result = {
            "state": r.state_name,
            "mode": r.mode_name,
            "joints": r.joints[: self.axes],
            "joint_torques": r.torques[: self.axes],
            "brake_bits": r.brake_bits,
            "enable_bits": r.enable_bits,
            "error_code": r.error_code,
            "warn_code": r.warn_code,
            "tcp_load": r.tcp_load,
            "collision_sensitivity": r.collision_sensitivity,
            "teach_sensitivity": r.teach_sensitivity,
        }
        return self._log("rtde_state", {"deep": deep}, ok=True, result=result)


def _offset_close(a: Sequence[float], b: Sequence[float]) -> bool:
    """Same TCP: within 0.1 mm, and the same rotation within 1 mrad (compared as
    matrices — two rotation vectors can differ for one rotation)."""
    if any(abs(a[i] - b[i]) > 1e-4 for i in range(3)):
        return False
    ra, rb = rotvec_to_matrix(a[3:6]), rotvec_to_matrix(b[3:6])
    return all(abs(ra[i][j] - rb[i][j]) < 1e-3 for i in range(3) for j in range(3))


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _violation(rule: str, detail: str):
    from .safety import SafetyViolation

    return SafetyViolation(rule, detail)


def _env_port(name: str, default: int) -> int:
    raw = os.environ.get(name, "").strip()
    return int(raw) if raw else default


__all__ = [
    "PLATFORM_UFACTORY",
    "UFactoryArm",
    "matrix_to_rpy",
    "offset_from_wire",
    "offset_to_wire",
    "pose_from_wire",
    "pose_to_wire",
    "rotvec_to_rpy",
    "rpy_to_matrix",
    "rpy_to_rotvec",
]
