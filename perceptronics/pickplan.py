"""From a clicked object to one fluid program: sweep in, hover, and (optionally) pick.

The work surface is flat and parallel to the base XY plane (Nick, 2026-09-27), so
an object's **top face** is a horizontal slice of its points in the base frame —
no plane fit, no tilt — and the gripper comes in straight down the base Z axis,
wrist 3 turned so the fingers close across the top face's short side, the jaws
open their full ``stroke_m``, fingertips ``hover_m`` over the top.

:func:`top_face` + :func:`rectangle` measure; :func:`plan` builds the legs of a
single ``move_tcp_path`` program (flange poses, run with the TCP at the flange):

- the **sweep** from where the camera took the picture to the hover, blended so
  the arm never stops on the way: one via over the object (the last ``drop_m``
  straight down the Z axis), or with ``fancy`` two more — a rise with a sideways
  swing, and a wrist flourish over the object that unwinds on the way down;
- with ``pick``: down to ``grasp_below_m`` under the top, close, lift ``lift_m``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from urctl.pose import Transform

from .pickcycle import grasp_rotation, grasp_yaw_deg, tip_pose

HOVER_M = 0.025  # fingertips over the top face at the approach (Nick, 2026-09-27)
GRASP_BELOW_M = 0.015
LIFT_M = 0.05
DROP_M = 0.06  # the last stretch of the sweep runs straight down the Z axis
LOOK_M = 0.24  # the close look: camera to the top, just outside the D435's ~0.2 m blind zone
LOOK_TILT_DEG = 10.0  # ... from a slight angle (a second viewpoint, and it reads as "taking a look")
STROKE_M = 0.05  # Hand-E — and the jaws open this full stroke every time (Nick, 2026-09-28: "maximum slop")
# (opening to 1.2x the part left 3.4 mm a side on a 28 mm block: a fingertip caught its top edge and
# the Hand-E closed on nothing; fully open at the same spot it held — UR3e, 2026-09-27)
MIN_SLOP_M = 0.002  # an object needs at least this much room a side inside the open jaws
# the finger zones checked for clearance before a grasp: each finger comes down just outside the
# open jaws — FINGER_T_M thick along its travel, FINGER_W_M wide across it (Hand-E pads + a margin)
FINGER_T_M = 0.014
FINGER_W_M = 0.032
# Motion (Nick, 2026-09-27: "all moves should be extremely fluid and smooth, not abrupt"):
# gentle accelerations everywhere; the speed only where the arm is clear of the parts.
# Again, 2026-09-27 23:25: "the jerk or accel settings are really high, the stops thud" — and
# 23:50: "I don't mind a high velocity, I just don't want a high acceleration and ideally no stops
# or slowdowns during moves." So: fast, gently accelerated, blended; stop only where physics needs it
# (the close look's still frame, the grasp and the release).
# 23:55: "everything should be much faster … robot moves much quicker since it's a safe demo space":
# double again, still under the 0.8 m/s^2 that thudded; blends keep it flowing.
TRANSIT = {"velocity": 0.60, "acceleration": 0.35}
SETTLE = {"velocity": 0.30, "acceleration": 0.30}
DESCEND = {"velocity": 0.10, "acceleration": 0.25}


def top_face(
    points_base: Sequence[Sequence[float]], band_m: float = 0.006, bin_m: float = 0.003
) -> list[Sequence[float]]:
    """The object's top: its points within ``band_m`` of the **highest dense** height —
    the top 3 mm bin holding at least 8 % of the points — so reflections and flying
    pixels floating above the part (a few, scattered) never set the top."""
    if len(points_base) < 10:
        return []
    bins: dict[int, int] = {}
    for p in points_base:
        k = round(p[2] / bin_m)
        bins[k] = bins.get(k, 0) + 1
    need = max(3, int(0.08 * len(points_base)))
    dense = [k for k, n in bins.items() if n + bins.get(k - 1, 0) + bins.get(k + 1, 0) >= need]
    top = max(dense) * bin_m if dense else sorted(p[2] for p in points_base)[len(points_base) // 2]
    return [p for p in points_base if abs(p[2] - top) <= band_m]


def rectangle(points: Sequence[Sequence[float]]) -> dict | None:
    """PCA in XY of a (horizontal) top face → ``{centre [x y z], theta (major-axis
    heading, rad), major_m, minor_m, n}``: the extents a uniform rectangle of that
    spread would have."""
    n = len(points)
    if n < 10:
        return None
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    cz = sorted(p[2] for p in points)[n // 2]
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    root = math.hypot(sxx - syy, 2 * sxy)
    lam1 = 0.5 * (sxx + syy) + 0.5 * root
    lam2 = max(0.0, 0.5 * (sxx + syy) - 0.5 * root)
    return {
        "centre": [cx, cy, cz],
        "theta": 0.5 * math.atan2(2 * sxy, sxx - syy),
        "major_m": 2 * math.sqrt(3 * lam1 / n),
        "minor_m": 2 * math.sqrt(3 * lam2 / n),
        "n": n,
    }


def clearance(
    rect: dict,
    points_base: Sequence[Sequence[float]],
    *,
    grasp_below_m: float = GRASP_BELOW_M,
    stroke_m: float = STROKE_M,
    finger_t_m: float = FINGER_T_M,
    finger_w_m: float = FINGER_W_M,
    min_points: int = 15,
    margin_z_m: float = 0.003,
    room_m: float | None = None,
) -> dict:
    """Is there room for the open fingers beside ``rect``? With ``room_m`` the question is
    the operator's own (the 3D Pick node, 0.8.0): that much clear space on each side of the
    part along the grip axis, whatever the gripper — the zone starts at the part's edge and
    ``stroke_m`` / ``finger_t_m`` play no part. Otherwise: Each finger comes down just outside
    the open jaws, along the travel axis (across the object's short side), to
    ``grasp_below_m`` under its top. Anything in either finger's zone that stands higher than
    the fingertips will reach — a neighbouring block, the rail — is a collision; the floor is
    below them. Fewer than ``min_points`` points in a zone are flying pixels, not an obstacle.
    ``{clear, side ("+"/"-"/None), count, worst_mm (above the fingertips)}``."""
    cx, cy, top = rect["centre"]
    th = rect["theta"]
    u = (-math.sin(th), math.cos(th))  # the fingers travel along the short side
    v = (math.cos(th), math.sin(th))
    bottom = top - grasp_below_m + margin_z_m
    # never inside the block itself: a wide block's own edge is not a neighbour
    inner = max(stroke_m / 2 - 0.003, rect["minor_m"] / 2 + 0.002)
    outer = stroke_m / 2 + finger_t_m
    if room_m is not None:
        inner = rect["minor_m"] / 2 + 0.002
        outer = rect["minor_m"] / 2 + room_m
    worst = {"+": [0, None], "-": [0, None]}
    for p in points_base:
        if p[2] <= bottom:
            continue
        dx, dy = p[0] - cx, p[1] - cy
        du, dv = dx * u[0] + dy * u[1], dx * v[0] + dy * v[1]
        if abs(dv) > finger_w_m / 2 or not inner <= abs(du) <= outer:
            continue
        side = "+" if du > 0 else "-"
        worst[side][0] += 1
        h = (p[2] - bottom) * 1000
        worst[side][1] = h if worst[side][1] is None else max(worst[side][1], h)
    hit = [s for s in ("+", "-") if worst[s][0] >= min_points]
    if not hit:
        return {"clear": True, "side": None, "count": 0, "worst_mm": None}
    side = max(hit, key=lambda s: worst[s][0])
    return {"clear": False, "side": side, "count": worst[side][0], "worst_mm": round(worst[side][1], 1)}


def robotiq_position(width_m: float, stroke_m: float = STROKE_M) -> int:
    """Robotiq position (0 = open … 255 = closed) for a finger opening of ``width_m``;
    the inverse of pick-cycle's ``(255 − POS) / 255 × stroke`` (POS 114 ≈ 27.6 mm, measured)."""
    w = max(0.0, min(stroke_m, width_m))
    return max(0, min(255, round(255 * (1.0 - w / stroke_m))))


def _blend(prev: Sequence[float], here: Sequence[float], nxt: Sequence[float], cap: float) -> float:
    """A blend radius the controller accepts: under half of both neighbouring segments."""
    a = math.dist(prev[:3], here[:3])
    b = math.dist(here[:3], nxt[:3])
    return math.floor(max(0.0, min(cap, 0.45 * a, 0.45 * b)) * 1e4) / 1e4  # round down: never past it


def look_pose(
    rect: dict,
    rot: Sequence[float],
    yaw_deg: float,
    flange_to_color: Sequence[float],
    range_m: float = LOOK_M,
    tilt_deg: float = LOOK_TILT_DEG,
) -> list[float]:
    """The flange pose (grasp orientation, wrist yawed) that puts the object's top
    centre on the colour camera's optical axis ``range_m`` away — as close as the D435
    still sees depth, and in the middle of the picture, clear of the gripper's body.
    ``tilt_deg`` leans the tool about the horizontal axis square to the base→object
    direction so the camera looks slightly outward at the part from the base's side: a
    second angle on it (Nick, 2026-09-28: "help get more robust angles and help the user
    understand it's taking a closer look, not just frozen"), and the flange comes in
    closer to the column."""
    oriented = Transform.from_pose(tip_pose(rect["centre"], rot, 0.0, yaw_deg))
    c = rect["centre"]
    r = math.hypot(c[0], c[1])
    if tilt_deg and r > 1e-6:
        axis = (-c[1] / r, c[0] / r, 0.0)  # horizontal, square to the radial direction
        T_fc0 = Transform.from_pose(flange_to_color)
        best = None
        for sign in (1.0, -1.0):
            t = math.radians(tilt_deg) * sign
            tilt = Transform.from_pose([0.0, 0.0, 0.0, axis[0] * t, axis[1] * t, 0.0])
            cand = Transform(
                tilt.compose(Transform(oriented.rotation, (0.0, 0.0, 0.0))).rotation, (0.0, 0.0, 0.0)
            )
            cz = cand.rotate(T_fc0.rotate((0.0, 0.0, 1.0)))
            outward = (cz[0] * c[0] + cz[1] * c[1]) / r  # > 0: the camera looks out, from the base side
            if best is None or outward > best[0]:
                best = (outward, cand)
        oriented = best[1]
    T_fc = Transform.from_pose(flange_to_color)
    cam_z = oriented.rotate(T_fc.rotate((0.0, 0.0, 1.0)))
    cam_off = oriented.rotate(T_fc.translation)
    return [c[i] - range_m * cam_z[i] - cam_off[i] for i in range(3)] + list(oriented.to_pose()[3:])


def _raised(pose: Sequence[float], dz: float, twist_deg: float = 0.0) -> list[float]:
    """``pose`` lifted ``dz`` along base Z and turned ``twist_deg`` about its own Z."""
    T = Transform.from_pose(pose).compose(Transform.from_pose([0, 0, 0, 0, 0, math.radians(twist_deg)]))
    out = T.to_pose()
    return [out[0], out[1], out[2] + dz, *out[3:]]


def _legs(start: Sequence[float], stops: list[tuple[str, list[float], dict]], cap: float) -> list[dict]:
    """``stops`` → path legs, each corner blended as far as its neighbours allow
    (the last one never)."""
    path = [list(start)] + [p for _, p, _ in stops]
    out = []
    for i, (name, pose, extra) in enumerate(stops):
        leg = {"name": name, "pose": pose, **extra}
        # a leg that grips or dwells must be *arrived at*: a blended movel returns as the arm
        # enters the blend and skips the sync(), so a close would go out early and the fingers
        # would shut while the arm swept on (every automated pick missed so, 2026-09-27)
        stops_here = extra.get("gripper") or extra.get("dwell_s")
        if i < len(stops) - 1 and extra.get("blend", True) and not stops_here:
            leg["blend_m"] = _blend(path[i], path[i + 1], path[i + 2], cap)
        leg.pop("blend", None)
        out.append(leg)
    return out


def plan(
    rect: dict,
    start_flange: Sequence[float],
    *,
    tip_m: float,
    pick: bool = False,
    fancy: bool = False,
    flange_to_color: Sequence[float] | None = None,
    look_m: float | None = None,
    hover_m: float = HOVER_M,
    grasp_below_m: float = GRASP_BELOW_M,
    lift_m: float = LIFT_M,
    stroke_m: float = STROKE_M,
    finger_axis: str = "y",
    velocity: float = TRANSIT["velocity"],
    acceleration: float = TRANSIT["acceleration"],
    skip: Sequence[str] = (),
    home: Sequence[float] | None = None,
) -> dict:
    """The programs for ``rect`` from ``start_flange`` (flange poses, TCP at the flange):

    - ``sweep``: blended vias from the picture pose — to the **look** pose when
      ``flange_to_color`` and ``look_m`` are given (the close second look), else
      straight on to the hover;
    - ``final``: over → approach (fingertips ``hover_m`` over the top) [→ grasp,
      close → lift and hold → place it back, open → clear] — from the look pose, or as
      the tail of the same program.

    ``legs`` is everything in order (a preview). The caller pre-opens the gripper
    to ``gripper_position``; ``skip`` drops named fancy vias the IK can't solve."""
    cx, cy, top = rect["centre"]
    rot = grasp_rotation(start_flange, [cx, cy, top], 0.0)  # straight down, heading kept
    yaw = grasp_yaw_deg(rot, rect["theta"] + math.pi / 2, finger_axis)
    at = lambda h: tip_pose([cx, cy, top + h], rot, tip_m, yaw)  # noqa: E731
    approach, over = at(hover_m), at(hover_m + DROP_M)
    look = (
        look_pose(rect, rot, yaw, flange_to_color, look_m) if flange_to_color is not None and look_m else None
    )
    goal = look if look is not None else over
    fast = {"velocity": velocity, "acceleration": acceleration}
    stops: list[tuple[str, list[float], dict]] = []
    if fancy:
        # rise out of the picture pose and swing wide, then a flourish over the target
        mid = [(start_flange[i] + goal[i]) / 2 for i in range(3)]
        dx, dy = goal[0] - start_flange[0], goal[1] - start_flange[1]
        d = math.hypot(dx, dy) or 1.0
        side = [-dy / d * min(0.08, 0.35 * d), dx / d * min(0.08, 0.35 * d)]
        swing = [mid[0] + side[0], mid[1] + side[1], max(start_flange[2], goal[2]) + 0.10]
        stops.append(("swing", swing + slerp_rotvec(start_flange, goal, 0.5), fast))
        stops.append(("flourish", _raised(goal, 0.06, 35.0), fast))
    stops = [st for st in stops if st[0] not in skip]
    tail = [
        # picking: one continuous descent over → approach → grasp (the approach only a waypoint)
        ("approach", approach, dict(DESCEND) if pick else dict(SETTLE)),
    ]
    if pick:
        tail.append(("grasp", at(-grasp_below_m), {**DESCEND, "gripper": "close"}))
        tail.append(("lift", at(hover_m + lift_m), {**SETTLE, "dwell_s": 0.3}))  # show it off
        # the drop sequence: set it back exactly where it was, let go, clear — never leave it in the fingers
        tail.append(("place", at(-grasp_below_m + 0.001), {**DESCEND, "gripper": "open"}))
        tail.append(("clear", at(hover_m + lift_m), dict(SETTLE)))
        if home is not None:  # and on home without stopping: clear blends into the sweep back
            tail.append(("home", list(home), dict(TRANSIT)))
    cap = 0.06 if fancy else 0.04
    if look is not None:
        sweep = _legs(
            start_flange,
            stops + [("look", look, {**SETTLE, "dwell_s": 0.15})],
            cap,
        )
        final = _legs(look, [("over", over, dict(SETTLE))] + tail, 0.03)
    else:
        sweep = _legs(start_flange, stops + [("over", over, fast)] + tail, cap)
        final = []
    opening = stroke_m  # full stroke, always: the most slop for the position error
    return {
        "sweep": sweep,
        "final": final,
        "legs": sweep + final,
        "look": look,
        "approach": approach,
        "grasp": at(-grasp_below_m),
        "lift": at(hover_m + lift_m),
        "yaw_deg": yaw,
        "opening_m": opening,
        "gripper_position": robotiq_position(opening, stroke_m),
        "fits": rect["minor_m"] + 2 * MIN_SLOP_M <= stroke_m,
        "vias": [leg["name"] for leg in sweep + final if leg["name"] not in ("approach", "grasp", "lift")],
    }


def slerp_rotvec(a: Sequence[float], b: Sequence[float], t: float) -> list[float]:
    """The rotation ``t`` of the way from pose ``a``'s orientation to ``b``'s (the
    shortest arc), as a rotation vector."""
    ta, tb = Transform.from_pose([0, 0, 0, *a[3:6]]), Transform.from_pose([0, 0, 0, *b[3:6]])
    rel = ta.inverse().compose(tb).to_pose()[3:]
    return ta.compose(Transform.from_pose([0, 0, 0, *(t * v for v in rel)])).to_pose()[3:]
