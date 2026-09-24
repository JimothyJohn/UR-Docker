"""Locate parts on a known plane from a swept sequence of 2D frames.

The measurement model (`docs/mono-scan.md` §4–5, single layer, no bins):

1. In every frame the bright parts are segmented from the darker surface
   (Otsu threshold + a morphological open) and each blob's outline is kept.
2. Each outline is **back-projected** through the frame's camera pose
   (``T_base_cam = T_base_flange · T_flange_color``) onto the plane ``h``
   above the table.
3. Outlines of the same object in different frames are clustered in the
   base frame. If ``h`` is right, the outlines from all frames land on top of
   each other; if it is wrong they slide apart along the sweep direction by
   ``baseline · Δh / (H - h)`` — **that parallax is the height measurement**.
   :func:`solve_height` scans ``h`` for the tightest cluster.
4. At the solved height the outline is fitted with a minimum-area rectangle
   (centre, yaw, long/short side), and the part library names the part +
   resting pose (§6).

numpy + OpenCV required. Nothing here talks to a robot or a camera: the
input is :class:`Sweep`'s frames + poses, which makes it replayable.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

from urctl.pose import Transform

from .partlib import PartLibrary
from .rgbd import Intrinsics
from .tableplane import Plane

DEFAULT_MIN_AREA_PX = 150
DEFAULT_CLUSTER_RADIUS_M = 0.025
DEFAULT_H_MAX_M = 0.15
DEFAULT_H_STEP_M = 0.0005


@dataclass
class Observation:
    """One blob outline in one frame, with everything needed to back-project it."""

    frame_index: int
    contour_px: Any  # (N, 2) float64
    rays_base: Any  # (N, 3) unit-ish directions in the base frame
    origin_base: tuple[float, float, float]
    area_px: float

    def on_plane(self, plane: Plane, h: float):
        """(N, 2) plane coordinates of the outline projected onto ``plane + h``."""
        import numpy as np

        n = np.asarray(plane.normal)
        p0 = np.asarray(plane.point) + h * n
        c = np.asarray(self.origin_base)
        denom = self.rays_base @ n
        denom = np.where(np.abs(denom) < 1e-12, np.nan, denom)
        t = float(n @ (p0 - c)) / denom
        pts = c[None, :] + t[:, None] * self.rays_base
        pts = pts[np.isfinite(t) & (t > 0)]
        e1, e2 = plane.axes()
        d = pts - np.asarray(plane.point)[None, :]
        return np.stack([d @ np.asarray(e1), d @ np.asarray(e2)], axis=1)


def segment_bright(gray, *, min_area_px: int = DEFAULT_MIN_AREA_PX, threshold: int | None = None):
    """Outlines of bright blobs on a darker background: ``[(contour (N,2), area)]``."""
    import cv2
    import numpy as np

    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    if threshold is None:
        _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        _, mask = cv2.threshold(blur, int(threshold), 255, cv2.THRESH_BINARY)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = []
    h, w = gray.shape[:2]
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < min_area_px:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue  # touching the image border: partial silhouette, skip
        pts = c.reshape(-1, 2).astype(np.float64)
        out.append((pts, area))
    return out


def pixel_rays(intr: Intrinsics, pts):
    """(N, 2) pixels → (N, 3) camera-frame directions (z = 1), undistorted."""
    import numpy as np

    pts = np.asarray(pts, dtype=np.float64)
    if any(abs(c) > 0 for c in intr.coeffs) and intr.model != "none":
        rays = np.array([intr.deproject(float(u), float(v), 1.0) for u, v in pts])
        return rays
    x = (pts[:, 0] - intr.ppx) / intr.fx
    y = (pts[:, 1] - intr.ppy) / intr.fy
    return np.stack([x, y, np.ones_like(x)], axis=1)


def observe_frame(
    frame_index: int,
    gray,
    intr: Intrinsics,
    t_base_cam: Transform,
    *,
    min_area_px: int = DEFAULT_MIN_AREA_PX,
    threshold: int | None = None,
) -> list[Observation]:
    import numpy as np

    rot = np.asarray(t_base_cam.rotation)
    out = []
    for contour, area in segment_bright(gray, min_area_px=min_area_px, threshold=threshold):
        rays = pixel_rays(intr, contour) @ rot.T
        out.append(Observation(frame_index, contour, rays, t_base_cam.translation, area))
    return out


def _centroid(pts) -> tuple[float, float]:
    """Area centroid of a closed polygon (shoelace); falls back to the mean."""
    import numpy as np

    if len(pts) < 3:
        m = pts.mean(axis=0)
        return float(m[0]), float(m[1])
    x, y = pts[:, 0], pts[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y1 - x1 * y
    a = cross.sum() / 2.0
    if abs(a) < 1e-12:
        m = pts.mean(axis=0)
        return float(m[0]), float(m[1])
    cx = ((x + x1) * cross).sum() / (6.0 * a)
    cy = ((y + y1) * cross).sum() / (6.0 * a)
    return float(cx), float(cy)


def cluster_observations(
    observations: Sequence[Observation],
    plane: Plane,
    *,
    h0: float = 0.0,
    radius_m: float = DEFAULT_CLUSTER_RADIUS_M,
) -> list[list[Observation]]:
    """Group outlines that land within ``radius_m`` of each other on ``plane + h0``
    (one outline per frame per cluster: the nearest wins)."""
    clusters: list[dict] = []
    for obs in observations:
        c = _centroid(obs.on_plane(plane, h0))
        best = None
        for cl in clusters:
            d = math.hypot(c[0] - cl["c"][0], c[1] - cl["c"][1])
            if d <= radius_m and (best is None or d < best[0]):
                best = (d, cl)
        if best is None:
            clusters.append({"c": c, "members": {obs.frame_index: obs}, "n": 1})
            continue
        cl = best[1]
        if obs.frame_index in cl["members"]:
            continue
        cl["members"][obs.frame_index] = obs
        cl["n"] += 1
        cl["c"] = (
            (cl["c"][0] * (cl["n"] - 1) + c[0]) / cl["n"],
            (cl["c"][1] * (cl["n"] - 1) + c[1]) / cl["n"],
        )
    return [list(cl["members"].values()) for cl in clusters]


def cluster_spread(members: Sequence[Observation], plane: Plane, h: float) -> float:
    """RMS distance (m) of the members' centroids from their mean at height ``h``."""
    import numpy as np

    cs = np.array([_centroid(o.on_plane(plane, h)) for o in members])
    if len(cs) < 2:
        return 0.0
    return float(np.sqrt(((cs - cs.mean(axis=0)) ** 2).sum(axis=1).mean()))


def solve_height(
    members: Sequence[Observation],
    plane: Plane,
    *,
    h_max: float = DEFAULT_H_MAX_M,
    h_step: float = DEFAULT_H_STEP_M,
) -> dict:
    """Scan ``h`` for the tightest cluster. Returns ``{height_m, spread_m,
    spread_at_zero_m, profile}``; a flat profile means no parallax (a
    rotation-only or too-short sweep) and the height is not trustworthy."""
    import numpy as np

    hs = np.arange(0.0, h_max + h_step / 2, h_step)
    spreads = np.array([cluster_spread(members, plane, float(h)) for h in hs])
    i = int(np.argmin(spreads))
    # parabolic refinement around the minimum
    h_best = float(hs[i])
    if 0 < i < len(hs) - 1:
        a, b, c = spreads[i - 1], spreads[i], spreads[i + 1]
        denom = a - 2 * b + c
        if denom > 1e-12:
            h_best = float(hs[i] + 0.5 * (a - c) / denom * h_step)
    return {
        "height_m": h_best,
        "spread_m": float(spreads[i]),
        "spread_at_zero_m": float(spreads[0]),
        "spread_max_m": float(spreads.max()),
        "observable": bool(spreads.max() - spreads[i] > 3 * h_step),
    }


def fit_rectangle(pts) -> dict:
    """Min-area rectangle of plane-coordinate points: centre, yaw of the long
    side (rad, in plane axes, mod π), long/short sides (m)."""
    import cv2
    import numpy as np

    p = np.asarray(pts, dtype=np.float64)
    if len(p) < 3:
        raise ValueError("need at least 3 points")
    (cx, cy), (w, h), ang = cv2.minAreaRect((p * 1000.0).astype(np.float32))
    yaw = math.radians(ang)
    if w < h:
        w, h = h, w
        yaw += math.pi / 2
    yaw = (yaw + math.pi) % math.pi
    return {"center": (cx / 1000.0, cy / 1000.0), "yaw": yaw, "long_m": w / 1000.0, "short_m": h / 1000.0}


def _mod_pi(a: float) -> float:
    a = a % math.pi
    return 0.0 if a > math.pi - 1e-6 else a


def _circular_mean_mod_pi(angles: Sequence[float]) -> float:
    s = sum(math.sin(2 * a) for a in angles)
    c = sum(math.cos(2 * a) for a in angles)
    return (math.atan2(s, c) / 2.0) % math.pi


@dataclass
class LocatedObject:
    index: int
    center_base_m: tuple[float, float, float]  # top-face centre
    footprint_center_base_m: tuple[float, float, float]  # on the table
    height_m: float
    height_observable: bool
    yaw_base_rad: float
    long_m: float
    short_m: float
    views: int
    spread_mm: float
    match: dict | None = None
    per_view: list[dict] = field(default_factory=list)
    fit: dict | None = None
    seed: dict | None = None

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "center_base_m": list(self.center_base_m),
            "footprint_center_base_m": list(self.footprint_center_base_m),
            "height_m": self.height_m,
            "height_observable": self.height_observable,
            "yaw_base_rad": self.yaw_base_rad,
            "yaw_base_deg": math.degrees(self.yaw_base_rad),
            "footprint_m": [self.long_m, self.short_m],
            "views": self.views,
            "spread_mm": self.spread_mm,
            "match": self.match,
            "iou": None if self.fit is None else self.fit.get("iou"),
            "measured": None if self.fit is None else self.fit.get("measured"),
            "seed": self.seed,
        }


def locate_objects(
    frames: Sequence[tuple[Any, Transform]],
    intr: Intrinsics,
    plane: Plane,
    *,
    library: PartLibrary | None = None,
    min_area_px: int = DEFAULT_MIN_AREA_PX,
    threshold: int | None = None,
    min_views: int = 3,
    h_max: float = DEFAULT_H_MAX_M,
    h_step: float = DEFAULT_H_STEP_M,
    refine: bool = True,
    free_dims: bool = True,
) -> list[LocatedObject]:
    """``frames`` = ``[(gray, T_base_cam), ...]`` → objects on ``plane`` in the base frame.

    ``refine=False`` skips the model fit (:mod:`perception.boxfit`) and reports
    the raw parallax/outline estimate — biased low in height, for diagnostics."""
    import numpy as np

    observations: list[Observation] = []
    for i, (gray, t_base_cam) in enumerate(frames):
        observations.extend(
            observe_frame(i, gray, intr, t_base_cam, min_area_px=min_area_px, threshold=threshold)
        )
    h_nominal = 0.0
    if library is not None and library.candidate_heights():
        h_nominal = min(library.candidate_heights())
    objects: list[LocatedObject] = []
    e1, _ = plane.axes()
    plane_x_angle = math.atan2(e1[1], e1[0])  # plane +x in base XY (0 for a level table)
    for members in cluster_observations(observations, plane, h0=h_nominal):
        if len(members) < min_views:
            continue
        hs = solve_height(members, plane, h_max=h_max, h_step=h_step)
        h = hs["height_m"]
        rects = [fit_rectangle(o.on_plane(plane, h)) for o in members]
        seed = {
            "cx": float(np.median([r["center"][0] for r in rects])),
            "cy": float(np.median([r["center"][1] for r in rects])),
            "long_m": float(np.median([r["long_m"] for r in rects])),
            "short_m": float(np.median([r["short_m"] for r in rects])),
            "yaw": _circular_mean_mod_pi([r["yaw"] for r in rects]),
            "height_m": h,
        }
        fit, match = _refine(members, frames, intr, plane, seed, library, refine, free_dims)
        cx, cy, h = fit["cx"], fit["cy"], fit["height_m"]
        long_m, short_m, yaw_plane = fit["long_m"], fit["short_m"], fit["yaw"]
        top = plane.from_plane_xy(cx, cy, h)
        foot = plane.from_plane_xy(cx, cy, 0.0)
        objects.append(
            LocatedObject(
                index=len(objects),
                center_base_m=top,
                footprint_center_base_m=foot,
                height_m=h,
                height_observable=hs["observable"],
                yaw_base_rad=_mod_pi(yaw_plane + plane_x_angle),
                long_m=long_m,
                short_m=short_m,
                views=len(members),
                spread_mm=hs["spread_m"] * 1000.0,
                match=match,
                per_view=[{"frame": o.frame_index, "area_px": o.area_px} for o in members],
                fit=fit,
                seed=seed,
            )
        )
    objects.sort(key=lambda o: (o.center_base_m[0], o.center_base_m[1]))
    for i, o in enumerate(objects):
        o.index = i
    return objects


def _refine(
    members,
    frames,
    intr,
    plane,
    seed: dict,
    library: PartLibrary | None,
    refine: bool,
    free_dims: bool = True,
):
    """Model-fit the seed against the outlines; try every library resting pose
    whose footprint is plausible and keep the best IoU. Returns ``(fit, match)``."""
    if not refine:
        match = library.match((seed["long_m"], seed["short_m"]), seed["height_m"]) if library else None
        return dict(seed, iou=None), match
    from .boxfit import fit_box, make_view_masks

    views = make_view_masks(members, frames)
    candidates: list[tuple[dict | None, tuple[float, float, float] | None]] = []
    if library is not None:
        for part in library.parts:
            for rp in part.resting:
                # the raw outline over-estimates the footprint (side faces), so accept generously
                if (
                    rp.footprint_m[0] <= seed["long_m"] + 0.004
                    and rp.footprint_m[1] <= seed["short_m"] + 0.004
                ):
                    m = {"part": part.name, "pose": rp.as_dict()}
                    candidates.append((m, (rp.footprint_m[0], rp.footprint_m[1], rp.height_m)))
    if not candidates:
        candidates.append((None, None))
    best: tuple[dict, dict | None] | None = None
    for match, dims in candidates:
        fit = fit_box(views, intr, plane, init=seed, dims=dims, free_dims=free_dims or dims is None)
        if best is None or fit["iou"] > best[0]["iou"]:
            best = (fit, match)
    assert best is not None
    fit, match = best
    if match is not None:
        match = dict(match, iou=fit["iou"], measured=fit.get("measured"))
    return fit, match
