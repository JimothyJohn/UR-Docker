"""Can the arm get there? Inverse kinematics through UR's DH parameters.

The pick server's reach check (2026-09-30, Nick: "I would rather have the robot determine
with its kinematics what's actually pickable"): a part is pickable when the arm has a joint
solution for its grasp, not when it sits inside a ring drawn around the base. This is the
closed-form solution for a UR's spherical-wrist-less 6R chain — up to eight joint vectors
per flange pose — over the same table as :mod:`perceptronics.armfk`.

Every candidate is run back through :func:`perceptronics.armfk.frames` and kept only when
its flange lands on the pose asked for, so a wrong branch or a wrong table yields "no
solution found here", never a wrong yes. The answer is the nominal arm's: no joint
limits, no self-collision, no calibration deltas. The controller's own
``get_inverse_kin_has_solution`` still has the last word in the program; this decides what
the pendant greys out and what the program is sent toward first.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from urctl.pose import Transform

from .armfk import DH, _dh, frames, model_key

POSITION_TOL_M = 1e-5
ROTATION_TOL = 1e-5
WRIST_SWEEP = 24  # joint 6 angles tried when wrist 2 is straight


def _acos_arg(v: float) -> float | None:
    """``v`` clamped into acos's domain when it is out by rounding only, else None."""
    if abs(v) <= 1.0:
        return v
    return math.copysign(1.0, v) if abs(v) <= 1.0 + 1e-9 else None


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _matches(q: Sequence[float], want: Transform, key: str) -> bool:
    got = Transform.from_pose(frames(q, key)[-1])
    if math.dist(got.translation, want.translation) > POSITION_TOL_M:
        return False
    return all(
        abs(got.rotation[i][j] - want.rotation[i][j]) <= ROTATION_TOL for i in range(3) for j in range(3)
    )


def solutions(flange_pose: Sequence[float], model: str) -> list[list[float]]:
    """Every joint vector (rad, each in (-π, π]) that puts the flange at ``flange_pose``
    (base frame, ``[x, y, z, rx, ry, rz]``). Empty when there is none; ValueError for a
    model the table lacks."""
    key = model_key(model)
    if key is None:
        raise ValueError(f"no kinematics for {model!r}; one of {sorted(DH)}")
    d = DH[key][0]
    d4, d6 = d[3], d[5]
    T06 = Transform.from_pose(flange_pose)
    p = T06.translation
    T60 = T06.inverse()
    # the wrist centre: back along the flange Z by d6
    p05 = T06.apply((0.0, 0.0, -d6))
    rxy = math.hypot(p05[0], p05[1])
    if rxy < abs(d4) or rxy < 1e-9:
        return []
    out: list[list[float]] = []
    psi = math.atan2(p05[1], p05[0])
    phi = math.acos(max(-1.0, min(1.0, d4 / rxy)))
    for q1 in (psi + phi + math.pi / 2, psi - phi + math.pi / 2):
        s1, c1 = math.sin(q1), math.cos(q1)
        arg5 = _acos_arg((p[0] * s1 - p[1] * c1 - d4) / d6)
        if arg5 is None:
            continue
        for q5 in (math.acos(arg5), -math.acos(arg5)):
            s5 = math.sin(q5)
            if abs(s5) < 1e-9:
                # wrist 2 straight: joint 6's axis is parallel to joints 2-4's, so the pose is
                # met by a whole family of joint 6 angles — but the axes are d5 apart, so not
                # by every one. A sweep finds members of the family (at the very edge of the
                # reach it can miss the only one: "no solution", the safe answer).
                q6s = [k * math.pi / WRIST_SWEEP * 2 - math.pi for k in range(WRIST_SWEEP)]
            else:
                q6s = [
                    math.atan2(
                        (-T60.rotation[1][0] * s1 + T60.rotation[1][1] * c1) / s5,
                        (T60.rotation[0][0] * s1 - T60.rotation[0][1] * c1) / s5,
                    )
                ]
            for q6 in q6s:
                out.extend(_elbows(T06, q1, q5, q6, key, out))
    return out


def _elbows(
    T06: Transform, q1: float, q5: float, q6: float, key: str, have: list[list[float]]
) -> list[list[float]]:
    """Joints 2-4 (elbow up and down) for a chosen 1, 5 and 6 — only those that check out
    and are not in ``have`` already."""
    d, a, alpha = DH[key]
    a2, a3 = a[1], a[2]
    T01 = _dh(q1, d[0], a[0], alpha[0])
    T45 = _dh(q5, d[4], a[4], alpha[4])
    T56 = _dh(q6, d[5], a[5], alpha[5])
    T14 = T01.inverse().compose(T06).compose(T45.compose(T56).inverse())
    p13 = T14.apply((0.0, -d[3], 0.0))
    n13 = math.hypot(*p13)
    if n13 < 1e-9:
        return []
    arg3 = _acos_arg((n13 * n13 - a2 * a2 - a3 * a3) / (2 * a2 * a3))
    if arg3 is None:
        return []
    out: list[list[float]] = []
    for q3 in (math.acos(arg3), -math.acos(arg3)):
        q2 = -math.atan2(p13[1], -p13[0]) + math.asin(max(-1.0, min(1.0, a3 * math.sin(q3) / n13)))
        T12 = _dh(q2, d[1], a[1], alpha[1])
        T23 = _dh(q3, d[2], a[2], alpha[2])
        T34 = T12.compose(T23).inverse().compose(T14)
        q4 = math.atan2(T34.rotation[1][0], T34.rotation[0][0])
        q = [_wrap(v) for v in (q1, q2, q3, q4, q5, q6)]
        if _matches(q, T06, key) and not any(
            max(abs(_wrap(q[i] - o[i])) for i in range(6)) < 1e-7 for o in have + out
        ):
            out.append(q)
    return out


def has_solution(flange_pose: Sequence[float], model: str | None) -> bool | None:
    """True / False for an arm in the table; None when the model is unknown (nobody here
    can say — leave it to the controller)."""
    if model_key(model) is None:
        return None
    return bool(solutions(flange_pose, model))  # type: ignore[arg-type]
