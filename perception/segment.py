"""Click-to-segment + feature extraction on an RGB-D frame.

The RealSenseTrainer idea — capture aligned color + depth, label the object,
save it — but instead of training a detector on those labels, segment the
object *now* (a point prompt from a click, or "the nearest thing" from depth)
and extract the features a pick needs: where it is, how big it is, which way
it's lying. Everything here is pure Python; the optional SAM backend
(:mod:`perception.backends.sam`) plugs into the same :class:`Segmenter` seam.

* :class:`Mask` — one binary label image (bytes, 0/1) with geometry helpers.
* :class:`Segmenter` — the pluggable interface: ``segment(rgbd, point) → Mask``.
* :class:`StubSegmenter` — region growing from the clicked pixel by *color
  similarity* (chained neighbour tolerance, like the blob detector) **and**
  *depth continuity* (a neighbour joins only if its depth is within
  ``depth_step_m`` of the pixel it grew from — so a red apple on a red tray
  still separates at the depth edge). Also :meth:`nearest_object`, the
  original RealSenseTrainer heuristic (everything within ``near_ratio`` of the
  closest valid depth).
* :func:`extract_features` — :class:`ObjectFeatures`: centroid, bbox, area,
  mean colour, depth stats, camera-frame 3D centroid, metric extent, principal
  axis orientation and a grasp hint. The hint is an explicit *assumption*
  (top-down approach, close across the minor axis) and is labelled as such.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .rgbd import DepthImage, Intrinsics, RgbdFrame


@dataclass(frozen=True)
class Mask:
    """Binary label image, row-major, one byte (0/1) per pixel."""

    width: int
    height: int
    data: bytes

    def __post_init__(self) -> None:
        if len(self.data) != self.width * self.height:
            raise ValueError(f"mask is {len(self.data)} bytes, expected {self.width * self.height}")

    @property
    def area(self) -> int:
        return self.data.count(1)

    def contains(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and self.data[y * self.width + x] == 1

    def pixels(self) -> list[int]:
        """Flat indices of set pixels."""
        return [i for i, v in enumerate(self.data) if v]

    def bbox(self) -> tuple[int, int, int, int] | None:
        """``(x_min, y_min, x_max, y_max)`` inclusive, or ``None`` if empty."""
        w = self.width
        xmin, ymin, xmax, ymax = w, self.height, -1, -1
        for i, v in enumerate(self.data):
            if v:
                y, x = divmod(i, w)
                if x < xmin:
                    xmin = x
                if x > xmax:
                    xmax = x
                if y < ymin:
                    ymin = y
                if y > ymax:
                    ymax = y
        return None if xmax < 0 else (xmin, ymin, xmax, ymax)

    def to_png(self) -> bytes:
        """8-bit grayscale PNG (0 / 255) — the label file of a capture."""
        from .pngio import encode_png

        return encode_png(self.width, self.height, 1, bytes(255 if v else 0 for v in self.data))

    @classmethod
    def empty(cls, width: int, height: int) -> Mask:
        return cls(width, height, bytes(width * height))


@runtime_checkable
class Segmenter(Protocol):
    """Point-prompted segmentation on an RGB-D frame. The pluggable seam."""

    name: str

    def segment(self, rgbd: RgbdFrame, point: tuple[int, int]) -> Mask: ...


@dataclass
class StubSegmenter:
    """Dependency-free region growing from a seed pixel.

    A 4-neighbour joins the region when *both* hold:

    * its colour is within ``link_tolerance`` (RGB Euclidean) of the pixel it
      is reached from **and** within ``seed_tolerance`` of the seed colour (the
      second bound stops slow colour drift from swallowing the background);
    * depth is *continuous*: both pixels have valid depth and differ by at most
      ``depth_step_m``, or (``allow_invalid_depth``) the neighbour has no depth
      — sensor holes inside an object shouldn't cut it in half.

    ``max_area_fraction`` caps the region so a click on the background can't
    return the whole frame.
    """

    name: str = "stub"
    link_tolerance: float = 40.0
    seed_tolerance: float = 90.0
    depth_step_m: float = 0.02
    allow_invalid_depth: bool = True
    max_area_fraction: float = 0.5
    use_depth: bool = True

    def segment(self, rgbd: RgbdFrame, point: tuple[int, int]) -> Mask:
        frame, depth = rgbd.color, rgbd.depth
        w, h = frame.width, frame.height
        sx, sy = point
        if not (0 <= sx < w and 0 <= sy < h):
            raise ValueError(f"seed ({sx}, {sy}) outside the {w}x{h} frame")
        use_depth = self.use_depth and (depth.width, depth.height) == (w, h)
        c = frame.channels
        px = frame.data
        dvals = depth.values() if use_depth else None
        step_units = self.depth_step_m / depth.scale_m if use_depth else 0.0
        link2 = self.link_tolerance**2
        seed2 = self.seed_tolerance**2
        cap = max(1, int(self.max_area_fraction * w * h))

        seed_i = sy * w + sx
        sr, sg, sb = px[seed_i * c], px[seed_i * c + 1], px[seed_i * c + 2]
        out = bytearray(w * h)
        out[seed_i] = 1
        count = 1
        q: deque[int] = deque([seed_i])
        while q and count < cap:
            i = q.popleft()
            ir, ig, ib = px[i * c], px[i * c + 1], px[i * c + 2]
            y, x = divmod(i, w)
            for j in (
                i - 1 if x > 0 else -1,
                i + 1 if x < w - 1 else -1,
                i - w if y > 0 else -1,
                i + w if y < h - 1 else -1,
            ):
                if j < 0 or out[j]:
                    continue
                jr, jg, jb = px[j * c], px[j * c + 1], px[j * c + 2]
                dr, dg, db = jr - ir, jg - ig, jb - ib
                if dr * dr + dg * dg + db * db > link2:
                    continue
                er, eg, eb = jr - sr, jg - sg, jb - sb
                if er * er + eg * eg + eb * eb > seed2:
                    continue
                if dvals is not None:
                    di, dj = dvals[i], dvals[j]
                    if dj == 0:
                        if not self.allow_invalid_depth:
                            continue
                    elif di != 0 and abs(di - dj) > step_units:
                        continue
                out[j] = 1
                count += 1
                q.append(j)
        return Mask(w, h, bytes(out))

    def nearest_object(self, rgbd: RgbdFrame, near_ratio: float = 1.2) -> Mask:
        """RealSenseTrainer's rule: everything closer than ``near_ratio`` × the
        nearest valid depth is "the object" (holes excluded). Returns the
        connected component containing the nearest pixel."""
        depth = rgbd.depth
        vals = depth.values()
        nearest = min((v for v in vals if v), default=0)
        if nearest == 0:
            return Mask.empty(depth.width, depth.height)
        limit = nearest * near_ratio
        w, h = depth.width, depth.height
        cand = bytearray(1 if (0 < v <= limit) else 0 for v in vals)
        seed = next(i for i, v in enumerate(vals) if v == nearest)
        out = bytearray(w * h)
        out[seed] = 1
        q: deque[int] = deque([seed])
        while q:
            i = q.popleft()
            y, x = divmod(i, w)
            for j in (
                i - 1 if x > 0 else -1,
                i + 1 if x < w - 1 else -1,
                i - w if y > 0 else -1,
                i + w if y < h - 1 else -1,
            ):
                if j >= 0 and cand[j] and not out[j]:
                    out[j] = 1
                    q.append(j)
        return Mask(w, h, bytes(out))


# ----- features ------------------------------------------------------------------


@dataclass(frozen=True)
class ObjectFeatures:
    """What a segmented object looks like in pixels, metres and pose."""

    area_px: int
    bbox: tuple[int, int, int, int]
    centroid_px: tuple[float, float]
    mean_rgb: tuple[int, int, int]
    depth_valid_px: int
    depth_median_m: float | None
    depth_min_m: float | None
    depth_max_m: float | None
    point_m: tuple[float, float, float] | None  # camera-frame centroid at the median depth
    extent_m: tuple[float, float] | None  # bbox width/height in metres at the median depth
    thickness_m: float | None  # depth_max - depth_min over valid pixels
    orientation_deg: float  # principal axis vs image +x, in (-90, 90]
    major_axis_m: float | None
    minor_axis_m: float | None
    elongation: float
    fill_ratio: float
    grasp: dict

    def as_dict(self) -> dict:
        return {
            "area_px": self.area_px,
            "bbox": list(self.bbox),
            "centroid_px": list(self.centroid_px),
            "mean_rgb": list(self.mean_rgb),
            "depth": {
                "valid_px": self.depth_valid_px,
                "median_m": self.depth_median_m,
                "min_m": self.depth_min_m,
                "max_m": self.depth_max_m,
                "thickness_m": self.thickness_m,
            },
            "point_m": list(self.point_m) if self.point_m else None,
            "extent_m": list(self.extent_m) if self.extent_m else None,
            "orientation_deg": self.orientation_deg,
            "major_axis_m": self.major_axis_m,
            "minor_axis_m": self.minor_axis_m,
            "elongation": self.elongation,
            "fill_ratio": self.fill_ratio,
            "grasp": self.grasp,
        }


def extract_features(
    mask: Mask, rgbd: RgbdFrame, intrinsics: Intrinsics | None = None
) -> ObjectFeatures | None:
    """Measure the object under ``mask``. ``None`` for an empty mask.

    Metric quantities use the **median** depth over the object's valid pixels
    (robust to holes and edge bleed) and the depth image's intrinsics.
    """
    intr = intrinsics or rgbd.intrinsics
    frame, depth = rgbd.color, rgbd.depth
    w = mask.width
    if (mask.width, mask.height) != (frame.width, frame.height):
        raise ValueError("mask and frame sizes differ")
    c = frame.channels
    px = frame.data
    same_depth = (depth.width, depth.height) == (w, mask.height)
    dvals = depth.values() if same_depth else None

    n = 0
    sx = sy = 0
    sxx = syy = sxy = 0.0
    r_sum = g_sum = b_sum = 0
    xmin, ymin, xmax, ymax = w, mask.height, -1, -1
    dlist: list[int] = []
    for i, v in enumerate(mask.data):
        if not v:
            continue
        y, x = divmod(i, w)
        n += 1
        sx += x
        sy += y
        sxx += x * x
        syy += y * y
        sxy += x * y
        r_sum += px[i * c]
        g_sum += px[i * c + 1]
        b_sum += px[i * c + 2]
        if x < xmin:
            xmin = x
        if x > xmax:
            xmax = x
        if y < ymin:
            ymin = y
        if y > ymax:
            ymax = y
        if dvals is not None and dvals[i]:
            dlist.append(dvals[i])
    if n == 0:
        return None

    cx, cy = sx / n, sy / n
    mu20 = sxx / n - cx * cx
    mu02 = syy / n - cy * cy
    mu11 = sxy / n - cx * cy
    theta = 0.5 * math.atan2(2 * mu11, mu20 - mu02) if (mu11 or mu20 != mu02) else 0.0
    orientation = math.degrees(theta)
    if orientation <= -90:
        orientation += 180
    common = 0.5 * (mu20 + mu02)
    diff = 0.5 * math.sqrt((mu20 - mu02) ** 2 + 4 * mu11 * mu11)
    lam1, lam2 = max(common + diff, 0.0), max(common - diff, 0.0)
    major_px, minor_px = (
        4 * math.sqrt(lam1),
        4 * math.sqrt(lam2),
    )  # full axis lengths of the equivalent ellipse
    elongation = (major_px / minor_px) if minor_px > 1e-9 else float("inf")

    bbox = (xmin, ymin, xmax, ymax)
    bw, bh = xmax - xmin + 1, ymax - ymin + 1
    fill = n / (bw * bh)
    mean_rgb = (r_sum // n, g_sum // n, b_sum // n)

    median = dmin = dmax = point = extent = thickness = major_m = minor_m = None
    if dlist:
        dlist.sort()
        mid = len(dlist) // 2
        med_units = dlist[mid] if len(dlist) % 2 else 0.5 * (dlist[mid - 1] + dlist[mid])
        median = med_units * depth.scale_m
        dmin, dmax = dlist[0] * depth.scale_m, dlist[-1] * depth.scale_m
        thickness = dmax - dmin
        point = intr.deproject(cx, cy, median)
        extent = (bw * median / intr.fx, bh * median / intr.fy)
        f_mean = 0.5 * (intr.fx + intr.fy)
        major_m, minor_m = major_px * median / f_mean, minor_px * median / f_mean

    grasp = {
        "assumption": "top-down approach along the camera z axis; close the gripper across the minor axis",
        "approach_point_m": list(point) if point else None,
        "gripper_yaw_deg": _wrap180(orientation + 90.0),
        "opening_m": minor_m,
        "confidence": "geometric-only (no learned grasp model)",
    }
    return ObjectFeatures(
        area_px=n,
        bbox=bbox,
        centroid_px=(cx, cy),
        mean_rgb=mean_rgb,
        depth_valid_px=len(dlist),
        depth_median_m=median,
        depth_min_m=dmin,
        depth_max_m=dmax,
        point_m=point,
        extent_m=extent,
        thickness_m=thickness,
        orientation_deg=orientation,
        major_axis_m=major_m,
        minor_axis_m=minor_m,
        elongation=elongation,
        fill_ratio=fill,
        grasp=grasp,
    )


def _wrap180(deg: float) -> float:
    while deg > 90:
        deg -= 180
    while deg <= -90:
        deg += 180
    return deg


__all__ = [
    "Mask",
    "Segmenter",
    "StubSegmenter",
    "ObjectFeatures",
    "extract_features",
    "DepthImage",
]
