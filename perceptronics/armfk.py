"""The arm's own geometry: forward kinematics through UR's DH parameters.

Each joint reading (RTDE ``actual_q``) → the base-frame pose of every DH frame,
base to flange — what the cockpit's scan draws the linkage from. The flange that
falls out must agree with the one the controller reports (``tcp ∘ tcp_offset⁻¹``):
:func:`frames` is only trusted when :func:`flange_error_m` says so, which keeps a
wrong table (or a wrong ``UR_ROBOT_MODEL``) from ever drawing a wrong arm.

The UR3e row was checked on the cell 2026-09-27: 0.84 mm from the controller's
flange at the picture pose (the nominal table; the controller's own calibration
accounts for the rest). The other rows are UR's published nominal values
("DH parameters for calculations of kinematics and dynamics", universal-robots.com;
UR30 and UR15 read 2026-10-02), unverified on hardware here — the flange check
guards them.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from urctl.pose import Transform

# model → (d, a, alpha), metres / radians, UR's standard DH convention
DH: dict[str, tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]] = {}
_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
for _name, _d, _a in (
    ("UR3E", (0.15185, 0, 0, 0.13105, 0.08535, 0.0921), (0, -0.24355, -0.2132, 0, 0, 0)),
    ("UR5E", (0.1625, 0, 0, 0.1333, 0.0997, 0.0996), (0, -0.425, -0.3922, 0, 0, 0)),
    ("UR10E", (0.1807, 0, 0, 0.17415, 0.11985, 0.11655), (0, -0.6127, -0.57155, 0, 0, 0)),
    ("UR16E", (0.1807, 0, 0, 0.17415, 0.11985, 0.11655), (0, -0.4784, -0.36, 0, 0, 0)),
    ("UR20", (0.2363, 0, 0, 0.2010, 0.1593, 0.1543), (0, -0.8620, -0.7287, 0, 0, 0)),
    ("UR30", (0.2363, 0, 0, 0.2010, 0.1593, 0.1543), (0, -0.6370, -0.5037, 0, 0, 0)),
    ("UR15", (0.2186, 0, 0, 0.1824, 0.1361, 0.1434), (0, -0.6475, -0.5164, 0, 0, 0)),
):
    DH[_name] = (tuple(map(float, _d)), tuple(map(float, _a)), _ALPHA)
# The UR7e is the UR5e's arm and the UR12e the UR10e's (Nick, 2026-10-01): the same table.
DH["UR7E"] = DH["UR5E"]
DH["UR12E"] = DH["UR10E"]

FLANGE_TOLERANCE_M = 0.02


def model_key(model: str | None) -> str | None:
    """``"UR3e"`` / ``"ur3e"`` / ``"UR3"`` (e-Series) → the table's key, or None."""
    if not model:
        return None
    k = model.strip().upper().replace("-", "").replace(" ", "")
    if k in DH:
        return k
    if k + "E" in DH:  # "UR3" on an e-Series controller
        return k + "E"
    return None


def _dh(theta: float, d: float, a: float, alpha: float) -> Transform:
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    rot = ((ct, -st * ca, st * sa), (st, ct * ca, -ct * sa), (0.0, sa, ca))
    return Transform(rot, (a * ct, a * st, d))  # type: ignore[arg-type]


def frames(q: Sequence[float], model: str) -> list[list[float]]:
    """Base-frame poses of the base and the six DH frames (the last is the flange)."""
    d, a, alpha = DH[model]
    T = Transform.from_pose([0, 0, 0, 0, 0, 0])
    out = [T.to_pose()]
    for i in range(6):
        T = T.compose(_dh(float(q[i]), d[i], a[i], alpha[i]))
        out.append(T.to_pose())
    return out


def flange_error_m(chain: Sequence[Sequence[float]], flange: Sequence[float]) -> float:
    """How far the FK flange lands from the measured one (metres)."""
    return math.dist(chain[-1][:3], flange[:3])
