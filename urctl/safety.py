"""SafetyEnvelope — pre-execution validation, the ROSClaw "safety envelope".

Before any motion or script reaches the controller, the proposed action is
checked against a configurable envelope: joint range, joint speed/acceleration
caps, and required controller state. A check produces a :class:`SafetyVerdict`
listing every violation; the caller (Robot / the tool layer) refuses to
execute unless the verdict is clear.

The defaults are deliberately conservative and model-agnostic — they keep
URSim and a real UR10 from being commanded somewhere obviously unsafe. They
are *not* a substitute for the controller's own configured safety limits
(which are authoritative on real hardware); they are a first line of defense
that catches agent mistakes before they leave the host.

Tune them for your robot::

    env = SafetyEnvelope(max_joint_speed=1.0, max_joint_position=3.14)
    robot = Robot(safety=env)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# UR e-Series joints rotate +/- 2*pi (360 deg). A single conservative bound is
# fine as a sanity check; per-joint limits can be supplied if needed.
DEFAULT_MAX_JOINT_POSITION = 2 * math.pi
# UR10 joint speed limits are 120 deg/s (base/shoulder/elbow) and 180 deg/s
# (wrists). We default to the lower of the two as a global cap.
DEFAULT_MAX_JOINT_SPEED = math.radians(120)  # ~2.09 rad/s
DEFAULT_MAX_JOINT_ACCEL = 5.0  # rad/s^2 — well below firmware max
NUM_JOINTS = 6

# --- Cartesian (TCP) limits, for movel-style linear moves -------------------
# UR10 max TCP speed is ~1 m/s; we cap there. The default move speed
# (DEFAULT_TCP_VELOCITY in robot.py) is much gentler.
DEFAULT_MAX_TCP_SPEED = 1.0  # m/s
DEFAULT_MAX_TCP_ACCEL = 5.0  # m/s^2 — conservative, below firmware max
# UR10 reach is ~1.3 m from the base. An absolute target whose XYZ distance
# from the base exceeds this is unreachable (and likely a mistake).
DEFAULT_MAX_REACH = 1.3  # m
# A single *relative* TCP step larger than this is almost always a unit error
# (e.g. passing inches as metres: 5 in -> 5.0 "m"). Caught before it executes.
DEFAULT_MAX_RELATIVE_STEP = 1.0  # m
POSE_LEN = 6

# The global speed slider scales the speed of *all* subsequent motion, so an
# operator can cap it (e.g. 0.3 during bring-up on real hardware). 1.0 = full
# programmed speed; the controller itself rejects anything above 1.0.
DEFAULT_MAX_SPEED_FRACTION = 1.0


@dataclass(frozen=True)
class SafetyViolation:
    rule: str
    detail: str

    def __str__(self) -> str:
        return f"{self.rule}: {self.detail}"


@dataclass
class SafetyVerdict:
    ok: bool
    violations: list[SafetyViolation] = field(default_factory=list)

    def raise_if_unsafe(self) -> None:
        if not self.ok:
            joined = "; ".join(str(v) for v in self.violations)
            raise SafetyError(joined)

    def as_dict(self) -> dict:
        return {
            "ok": self.ok,
            "violations": [{"rule": v.rule, "detail": v.detail} for v in self.violations],
        }


class SafetyError(Exception):
    """Raised when an action is rejected by the safety envelope."""


@dataclass
class SafetyEnvelope:
    max_joint_position: float = DEFAULT_MAX_JOINT_POSITION
    max_joint_speed: float = DEFAULT_MAX_JOINT_SPEED
    max_joint_accel: float = DEFAULT_MAX_JOINT_ACCEL
    num_joints: int = NUM_JOINTS
    # Cartesian / linear-move (movel) limits.
    max_tcp_speed: float = DEFAULT_MAX_TCP_SPEED
    max_tcp_accel: float = DEFAULT_MAX_TCP_ACCEL
    max_reach: float = DEFAULT_MAX_REACH
    max_relative_step: float = DEFAULT_MAX_RELATIVE_STEP
    # Global speed-slider cap (RTDE write). 0 < fraction <= this.
    max_speed_fraction: float = DEFAULT_MAX_SPEED_FRACTION
    # When True, motion is only permitted while the controller reports RUNNING.
    require_running: bool = True

    def validate_move_joints(
        self,
        target: list[float],
        *,
        velocity: float,
        acceleration: float,
        robot_mode: str | None = None,
    ) -> SafetyVerdict:
        """Validate a ``movej`` request against the envelope."""
        violations: list[SafetyViolation] = []

        if len(target) != self.num_joints:
            violations.append(
                SafetyViolation(
                    "joint_count", f"expected {self.num_joints} joint values, got {len(target)}"
                )
            )
        else:
            for i, q in enumerate(target):
                if not isinstance(q, int | float) or math.isnan(q) or math.isinf(q):
                    violations.append(
                        SafetyViolation("joint_value", f"joint {i} is not a finite number: {q!r}")
                    )
                elif abs(q) > self.max_joint_position:
                    violations.append(
                        SafetyViolation(
                            "joint_range",
                            f"joint {i} = {q:.4f} rad exceeds +/-{self.max_joint_position:.4f}",
                        )
                    )

        if velocity <= 0:
            violations.append(SafetyViolation("velocity", "velocity must be > 0"))
        elif velocity > self.max_joint_speed:
            violations.append(
                SafetyViolation(
                    "velocity", f"{velocity:.4f} rad/s exceeds max {self.max_joint_speed:.4f}"
                )
            )

        if acceleration <= 0:
            violations.append(SafetyViolation("acceleration", "acceleration must be > 0"))
        elif acceleration > self.max_joint_accel:
            violations.append(
                SafetyViolation(
                    "acceleration",
                    f"{acceleration:.4f} rad/s^2 exceeds max {self.max_joint_accel:.4f}",
                )
            )

        if self.require_running and robot_mode is not None and "RUNNING" not in robot_mode:
            violations.append(
                SafetyViolation(
                    "robot_state",
                    f"robot must be RUNNING to move (current: {robot_mode.strip()!r})",
                )
            )

        return SafetyVerdict(ok=not violations, violations=violations)

    def validate_speed_override(self, fraction: float) -> SafetyVerdict:
        """Validate a global speed-slider value (0 < fraction <= max)."""
        violations: list[SafetyViolation] = []
        if not isinstance(fraction, int | float) or math.isnan(fraction) or math.isinf(fraction):
            violations.append(
                SafetyViolation("speed_override", f"fraction is not a finite number: {fraction!r}")
            )
        elif not 0.0 < fraction <= self.max_speed_fraction:
            violations.append(
                SafetyViolation(
                    "speed_override",
                    f"fraction {fraction} outside (0, {self.max_speed_fraction}]",
                )
            )
        return SafetyVerdict(ok=not violations, violations=violations)

    def validate_move_tcp(
        self,
        pose: list[float],
        *,
        velocity: float,
        acceleration: float,
        relative: bool = False,
        robot_mode: str | None = None,
    ) -> SafetyVerdict:
        """Validate a Cartesian ``movel`` request against the envelope.

        ``pose`` is ``[x, y, z, rx, ry, rz]`` in metres + rotation-vector
        radians. When ``relative`` is True it is a base-frame delta added to the
        current TCP pose; otherwise it is an absolute pose in the base frame.
        Speed/acceleration are in m/s and m/s^2 (distinct from the joint caps).
        """
        violations: list[SafetyViolation] = []

        if len(pose) != POSE_LEN:
            violations.append(
                SafetyViolation("pose_count", f"expected {POSE_LEN} pose values, got {len(pose)}")
            )
        else:
            finite = True
            for i, v in enumerate(pose):
                if not isinstance(v, int | float) or math.isnan(v) or math.isinf(v):
                    finite = False
                    violations.append(
                        SafetyViolation("pose_value", f"pose[{i}] is not a finite number: {v!r}")
                    )
            if finite:
                # Translation magnitude of the XYZ part.
                dist = math.sqrt(pose[0] ** 2 + pose[1] ** 2 + pose[2] ** 2)
                if relative and dist > self.max_relative_step:
                    violations.append(
                        SafetyViolation(
                            "tcp_step",
                            f"relative step {dist:.4f} m exceeds max {self.max_relative_step:.4f} m "
                            "(unit error? metres, not inches/mm)",
                        )
                    )
                elif not relative and dist > self.max_reach:
                    violations.append(
                        SafetyViolation(
                            "tcp_reach",
                            f"target {dist:.4f} m from base exceeds max reach "
                            f"{self.max_reach:.4f} m",
                        )
                    )

        if velocity <= 0:
            violations.append(SafetyViolation("tcp_velocity", "velocity must be > 0"))
        elif velocity > self.max_tcp_speed:
            violations.append(
                SafetyViolation(
                    "tcp_velocity", f"{velocity:.4f} m/s exceeds max {self.max_tcp_speed:.4f}"
                )
            )

        if acceleration <= 0:
            violations.append(SafetyViolation("tcp_acceleration", "acceleration must be > 0"))
        elif acceleration > self.max_tcp_accel:
            violations.append(
                SafetyViolation(
                    "tcp_acceleration",
                    f"{acceleration:.4f} m/s^2 exceeds max {self.max_tcp_accel:.4f}",
                )
            )

        if self.require_running and robot_mode is not None and "RUNNING" not in robot_mode:
            violations.append(
                SafetyViolation(
                    "robot_state",
                    f"robot must be RUNNING to move (current: {robot_mode.strip()!r})",
                )
            )

        return SafetyVerdict(ok=not violations, violations=violations)
