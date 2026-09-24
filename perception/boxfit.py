"""Fit a box (the part's oriented bounding box) to its outlines across a sweep.

The silhouette of a box seen from above is the top face **plus** whichever
side faces are visible, and the side faces change with the camera position —
so any statistic of the raw outline (its centroid, its min-area rectangle)
moves with the camera and biases the parallax height low
(:func:`perception.locate2d.solve_height` is only the seed). The fix is the
model-based matching `docs/mono-scan.md` §5 plans for textureless parts: for a
hypothesised box (centre, yaw, length, width, height) on the table plane,
*predict* the silhouette in every frame (the convex hull of the 8 projected
corners — exact for a box, an outer bound for any convex part) and score it
against the observed outline by intersection-over-union. A Nelder–Mead
search over the pose (and, when no library pins them, the dimensions)
maximises the mean IoU over the frames.

numpy + OpenCV. Fast enough on a laptop CPU: a few hundred evaluations of
K small polygon fills.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

from urctl.pose import Transform

from .monocam import Box
from .rgbd import Intrinsics
from .tableplane import Plane

ROI_MARGIN_PX = 24
MAX_FIT_FRAMES = 10


@dataclass
class ViewMask:
    """One frame's observed outline as a binary mask in a region of interest."""

    t_cam_base: Transform
    x0: int
    y0: int
    mask: Any  # (h, w) uint8, 0/255
    area: float


def _project(intr: Intrinsics, t_cam_base: Transform, corners) -> Any | None:
    import numpy as np

    pts = []
    for c in corners:
        p = t_cam_base.apply(c)
        if p[2] <= 1e-6:
            return None
        pts.append((intr.fx * p[0] / p[2] + intr.ppx, intr.fy * p[1] / p[2] + intr.ppy))
    return np.asarray(pts, dtype=np.float64)


def make_view_masks(
    members: Sequence,
    frames: Sequence[tuple[Any, Transform]],
    *,
    max_frames: int = MAX_FIT_FRAMES,
    margin: int = ROI_MARGIN_PX,
) -> list[ViewMask]:
    """Rasterise each observation's outline into a ROI mask. ``members`` are
    :class:`perception.locate2d.Observation`; ``frames`` ``[(gray, T_base_cam)]``."""
    import cv2
    import numpy as np

    picked = list(members)
    if len(picked) > max_frames:
        idx = np.linspace(0, len(picked) - 1, max_frames).round().astype(int)
        picked = [picked[i] for i in idx]
    out = []
    for obs in picked:
        gray, t_base_cam = frames[obs.frame_index]
        h, w = gray.shape[:2]
        c = obs.contour_px
        x0 = max(0, int(c[:, 0].min()) - margin)
        y0 = max(0, int(c[:, 1].min()) - margin)
        x1 = min(w, int(c[:, 0].max()) + margin + 1)
        y1 = min(h, int(c[:, 1].max()) + margin + 1)
        mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
        poly = np.round((c - [x0, y0]) * 16).astype(np.int32).reshape(1, -1, 2)
        cv2.fillPoly(mask, poly, 255, lineType=cv2.LINE_AA, shift=4)
        out.append(ViewMask(t_base_cam.inverse(), x0, y0, mask, float((mask > 127).sum())))
    return out


def silhouette_cost(
    views: Sequence[ViewMask],
    intr: Intrinsics,
    plane: Plane,
    cx: float,
    cy: float,
    yaw: float,
    length: float,
    w: float,
    h: float,
) -> float:
    """``1 - mean IoU`` between the predicted box silhouette and the observed mask."""
    import cv2
    import numpy as np

    if min(length, w, h) <= 0.002:
        return 1.0
    centre = plane.from_plane_xy(cx, cy, 0.0)
    corners = Box(centre, (length, w, h), yaw).corners_base(plane)
    total = 0.0
    for v in views:
        pix = _project(intr, v.t_cam_base, corners)
        if pix is None:
            total += 1.0
            continue
        hull = cv2.convexHull(((pix - [v.x0, v.y0]) * 16).astype(np.float32)).astype(np.int32)
        pred = np.zeros_like(v.mask)
        cv2.fillPoly(pred, [hull.reshape(-1, 1, 2)], 255, lineType=cv2.LINE_AA, shift=4)
        a = v.mask > 127
        b = pred > 127
        union = float(np.logical_or(a, b).sum())
        inter = float(np.logical_and(a, b).sum())
        total += 1.0 - (inter / union if union > 0 else 0.0)
    return total / len(views)


def nelder_mead(
    f: Callable[[Sequence[float]], float],
    x0: Sequence[float],
    steps: Sequence[float],
    *,
    iters: int = 400,
    xtol: float = 1e-5,
    ftol: float = 1e-5,
) -> tuple[list[float], float]:
    """Plain Nelder–Mead (numpy-free) for a handful of parameters."""
    n = len(x0)
    simplex = [list(x0)]
    for i in range(n):
        p = list(x0)
        p[i] += steps[i]
        simplex.append(p)
    vals = [f(p) for p in simplex]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: vals[i])
        simplex = [simplex[i] for i in order]
        vals = [vals[i] for i in order]
        best, worst = simplex[0], simplex[-1]
        if max(abs(worst[i] - best[i]) for i in range(n)) < xtol and abs(vals[-1] - vals[0]) < ftol:
            break
        centroid = [sum(p[i] for p in simplex[:-1]) / n for i in range(n)]
        refl = [centroid[i] + (centroid[i] - worst[i]) for i in range(n)]
        fr = f(refl)
        if fr < vals[0]:
            exp = [centroid[i] + 2.0 * (centroid[i] - worst[i]) for i in range(n)]
            fe = f(exp)
            if fe < fr:
                simplex[-1], vals[-1] = exp, fe
            else:
                simplex[-1], vals[-1] = refl, fr
        elif fr < vals[-2]:
            simplex[-1], vals[-1] = refl, fr
        else:
            outside = fr < vals[-1]
            con = [centroid[i] + 0.5 * ((refl[i] if outside else worst[i]) - centroid[i]) for i in range(n)]
            fc = f(con)
            if fc < (fr if outside else vals[-1]):
                simplex[-1], vals[-1] = con, fc
            else:
                for j in range(1, n + 1):
                    simplex[j] = [best[i] + 0.5 * (simplex[j][i] - best[i]) for i in range(n)]
                    vals[j] = f(simplex[j])
    i = min(range(n + 1), key=lambda k: vals[k])
    return simplex[i], vals[i]


def fit_box(
    views: Sequence[ViewMask],
    intr: Intrinsics,
    plane: Plane,
    *,
    init: dict,
    dims: tuple[float, float, float] | None = None,
    free_dims: bool = True,
) -> dict:
    """Refine ``init`` = ``{cx, cy, yaw, long_m, short_m, height_m}`` (plane
    coords). With ``dims`` = (l, w, h) from the part library the pose is fitted
    with those fixed and then, if ``free_dims``, all six are released to
    *measure* the deviation. Returns the fitted box + ``iou``."""
    yaw0 = float(init["yaw"])
    if dims is not None:
        l0, w0, h0 = dims
    else:
        l0, w0, h0 = float(init["long_m"]), float(init["short_m"]), float(init["height_m"])
    # A square footprint is 90°-ambiguous: try both the seed yaw and +90° for a rectangle.
    starts = [yaw0] if abs(l0 - w0) < 1e-6 else [yaw0, (yaw0 + math.pi / 2) % math.pi]
    best_pose = None
    for y0 in starts:

        def cost3(p, _l=l0, _w=w0, _h=h0):
            return silhouette_cost(views, intr, plane, p[0], p[1], p[2], _l, _w, _h)

        x, c = nelder_mead(cost3, [init["cx"], init["cy"], y0], [0.004, 0.004, 0.08], iters=250)
        if best_pose is None or c < best_pose[1]:
            best_pose = (x, c)
    assert best_pose is not None
    (cx, cy, yaw), cost = best_pose
    result = {
        "cx": cx,
        "cy": cy,
        "yaw": yaw % math.pi,
        "long_m": l0,
        "short_m": w0,
        "height_m": h0,
        "iou": 1.0 - cost,
        "dims_fixed": dims is not None,
    }
    if free_dims:

        def cost6(p):
            return silhouette_cost(views, intr, plane, p[0], p[1], p[2], p[3], p[4], p[5])

        x, c = nelder_mead(
            cost6, [cx, cy, yaw, l0, w0, h0], [0.002, 0.002, 0.03, 0.003, 0.003, 0.004], iters=400
        )
        if c <= cost + 1e-9:
            result["measured"] = {
                "cx": x[0],
                "cy": x[1],
                "yaw": x[2] % math.pi,
                "long_m": max(x[3], x[4]),
                "short_m": min(x[3], x[4]),
                "height_m": x[5],
                "iou": 1.0 - c,
            }
            if dims is None:
                result.update({k: v for k, v in result["measured"].items()})
    return result
