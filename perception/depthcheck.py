"""Cross-check mono-located objects against the RealSense's own depth.

While the D435 is still on the arm the scan can be *graded* for free: one
aligned RGB-D frame taken at the end of the sweep, plus its flange pose, gives
an independent height for every located object. ``perception scan`` (RealSense)
and the cockpit's Scan do this automatically; the result rides along as
``depth_check`` per object. The mono pipeline never *uses* the depth.
"""

from __future__ import annotations

from collections.abc import Sequence

from urctl.pose import Transform

from .tableplane import Plane

WINDOW_PX = 7


def median_point(frame, u: int, v: int, window: int = WINDOW_PX):
    """Median camera-frame point over a ``window``×``window`` patch of valid depth."""
    w, h = frame.depth.width, frame.depth.height
    r = window // 2
    pts = []
    for y in range(max(0, v - r), min(h, v + r + 1)):
        for x in range(max(0, u - r), min(w, u + r + 1)):
            p = frame.point_at(x, y)
            if p is not None:
                pts.append(p)
    if not pts:
        return None
    med = []
    for k in range(3):
        vals = sorted(p[k] for p in pts)
        n = len(vals)
        med.append(vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2]))
    return tuple(med)


def check_objects(
    objects: Sequence, frame, t_base_cam: Transform, plane: Plane, *, tol_m: float = 0.005
) -> list[dict]:
    """For each located object: project its top-centre into ``frame``, read the
    depth there, lift it to the base frame and compare heights above ``plane``."""
    t_cam_base = t_base_cam.inverse()
    out = []
    for o in objects:
        top = o.center_base_m if hasattr(o, "center_base_m") else o["center_base_m"]
        h_mono = o.height_m if hasattr(o, "height_m") else o["height_m"]
        p_cam = t_cam_base.apply(top)
        entry: dict = {"index": getattr(o, "index", None), "mono_height_m": h_mono}
        if p_cam[2] <= 0:
            entry.update(ok=None, error="behind the camera")
            out.append(entry)
            continue
        u, v = frame.intrinsics.project(*p_cam)
        ui, vi = int(round(u)), int(round(v))
        entry["pixel"] = [ui, vi]
        if not (0 <= ui < frame.depth.width and 0 <= vi < frame.depth.height):
            entry.update(ok=None, error="outside the depth frame")
            out.append(entry)
            continue
        med = median_point(frame, ui, vi)
        if med is None:
            entry.update(ok=None, error="no depth at the object")
            out.append(entry)
            continue
        p_base = t_base_cam.apply(med)
        h_depth = plane.height_of(p_base)
        entry.update(
            depth_height_m=h_depth,
            diff_mm=(h_depth - h_mono) * 1000.0,
            xy_diff_mm=[(p_base[0] - top[0]) * 1000.0, (p_base[1] - top[1]) * 1000.0],
            ok=abs(h_depth - h_mono) <= tol_m,
        )
        out.append(entry)
    return out
