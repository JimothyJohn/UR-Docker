"""Blob detection — interface + dependency-free connected-components backend.

A :class:`BlobDetector` takes the RGB :class:`Frame` and its
:class:`DepthMap` and returns a list of :class:`Blob` — compact bright/colored
regions, each annotated with its pixel geometry **and** the depth sampled over
its footprint. That is the contract the user picked: *2D + depth*, no 3D
deprojection yet. A later integration step adds camera intrinsics + a
camera->base transform to turn ``(centroid_px, depth_m)`` into a base-frame
point the arm can pick.

:class:`StubBlobDetector` is the always-available default: threshold the frame
against its background, label 4-connected components, filter by area. The
optional :mod:`perception.backends.blob_cv` backend swaps in OpenCV's
``SimpleBlobDetector`` with no change downstream.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from .depth import DepthMap
from .frame import Frame


@dataclass(frozen=True)
class Blob:
    """One detected region. Pixel-space geometry plus a depth reading."""

    centroid_px: tuple[float, float]
    bbox: tuple[int, int, int, int]  # (x_min, y_min, x_max, y_max), inclusive
    area_px: int
    depth_m: float
    # Mean RGB of the region — handy for downstream class hints (e.g. red apple).
    mean_rgb: tuple[int, int, int] = (0, 0, 0)

    def as_dict(self) -> dict:
        return {
            "centroid_px": list(self.centroid_px),
            "bbox": list(self.bbox),
            "area_px": self.area_px,
            "depth_m": self.depth_m,
            "mean_rgb": list(self.mean_rgb),
        }


@runtime_checkable
class BlobDetector(Protocol):
    """Anything that finds blobs in a Frame given its depth. The pluggable seam."""

    name: str

    def detect(self, frame: Frame, depth: DepthMap) -> list[Blob]: ...


@dataclass
class StubBlobDetector:
    """Colorful-object blob finder — pure Python, no deps.

    Built for the case this repo targets: brightly colored objects (apples,
    produce, parts) on a *neutral* background (brushed steel, a grey/white/black
    table). Neutral surfaces — and white overlay text and specular highlights —
    have near-zero chroma, so segmenting by colorfulness ignores them for free.

    1. Mark a pixel as foreground when its **chroma** (max channel - min
       channel) exceeds ``min_chroma``.
    2. Region-grow 4-connected foreground, joining a neighbor only when its
       color is within ``link_tolerance`` (RGB distance) of the current pixel.
       So two *touching* objects of different color (a red apple and a green
       one) separate into distinct blobs, while a single shaded object — whose
       neighboring pixels are similar — stays whole.
    3. Keep components with ``area_px >= min_area``; report centroid, bbox,
       area, mean color, and the median depth over the component's pixels.

    Iterative (explicit stack) so large blobs don't blow the recursion limit.

    Two *same-colored* touching objects (e.g. two red apples) survive step 2 as
    a single component — connected components cannot split them by color. Step 4
    handles that case when ``split_touching`` is set (the default):

    4. **Instance split** — compute a distance transform of the component, find
       its peaks (one per object center, scale-adaptively spaced), and assign
       each pixel to its nearest peak. Two touching round objects have two DT
       peaks and split apart; a single convex object has one peak and stays
       whole. This is a lightweight, dependency-free watershed; a learned
       segmenter (the upgrade path) handles occlusion and odd shapes better.
    """

    min_area: int = 60
    min_chroma: int = 45
    link_tolerance: int = 40
    split_touching: bool = True
    # A peak must reach this fraction of the component's max distance-transform
    # value to seed an instance; seeds must be at least this multiple of that
    # max value apart. Both are scale-free, so they work across resolutions.
    split_peak_ratio: float = 0.5
    split_separation_ratio: float = 1.4
    name: str = field(default="stub", init=False)

    def detect(self, frame: Frame, depth: DepthMap) -> list[Blob]:
        w, h, d, c = frame.width, frame.height, frame.data, frame.channels
        n = w * h

        fg = bytearray(n)  # 1 where the pixel is colorful enough to be an object
        for i in range(n):
            base = i * c
            r, g, b = d[base], d[base + 1], d[base + 2]
            if max(r, g, b) - min(r, g, b) > self.min_chroma:
                fg[i] = 1

        link2 = self.link_tolerance * self.link_tolerance
        visited = bytearray(n)
        blobs: list[Blob] = []
        for start in range(n):
            if not fg[start] or visited[start]:
                continue
            # Region-grow this component, linking only color-similar neighbors.
            stack = [start]
            visited[start] = 1
            pixels: list[int] = []
            while stack:
                i = stack.pop()
                pixels.append(i)
                bi = i * c
                ri, gi, bi_ = d[bi], d[bi + 1], d[bi + 2]
                x = i % w
                y = i // w
                neighbors = []
                if x > 0:
                    neighbors.append(i - 1)
                if x < w - 1:
                    neighbors.append(i + 1)
                if y > 0:
                    neighbors.append(i - w)
                if y < h - 1:
                    neighbors.append(i + w)
                for j in neighbors:
                    if not fg[j] or visited[j]:
                        continue
                    bj = j * c
                    dr = ri - d[bj]
                    dg = gi - d[bj + 1]
                    db = bi_ - d[bj + 2]
                    if dr * dr + dg * dg + db * db <= link2:
                        visited[j] = 1
                        stack.append(j)

            if len(pixels) < self.min_area:
                continue
            if self.split_touching:
                groups = _split_component(pixels, w, self.split_peak_ratio, self.split_separation_ratio)
            else:
                groups = [pixels]
            for group in groups:
                if len(group) >= self.min_area:
                    blobs.append(_summarize(group, frame, depth))

        # Largest first — the most pickable target leads.
        blobs.sort(key=lambda b: b.area_px, reverse=True)
        return blobs


def _split_component(
    pixels: list[int], frame_width: int, peak_ratio: float, separation_ratio: float
) -> list[list[int]]:
    """Split one connected component into instances via a distance transform.

    Returns one group per detected object center (one group = no split). The DT
    is computed on the component's local bounding box with a two-pass chamfer
    approximation; peaks (local maxima above ``peak_ratio * maxDT``, spaced at
    least ``separation_ratio * maxDT`` apart) seed the instances; every pixel is
    then assigned to its nearest seed.
    """
    xs = [p % frame_width for p in pixels]
    ys = [p // frame_width for p in pixels]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    bw = max_x - min_x + 1
    bh = max_y - min_y + 1

    mask = bytearray(bw * bh)
    for lx, ly in zip((x - min_x for x in xs), (y - min_y for y in ys), strict=True):
        mask[ly * bw + lx] = 1

    dist = _distance_transform(mask, bw, bh)
    max_dt = max(dist)
    if max_dt <= 0:
        return [pixels]

    # Candidate peaks: local maxima tall enough to be an object center.
    min_height = peak_ratio * max_dt
    candidates = []
    for ly in range(bh):
        for lx in range(bw):
            k = ly * bw + lx
            if not mask[k] or dist[k] < min_height:
                continue
            here = dist[k]
            is_peak = True
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = lx + dx, ly + dy
                    if 0 <= nx < bw and 0 <= ny < bh and dist[ny * bw + nx] > here:
                        is_peak = False
                        break
                if not is_peak:
                    break
            if is_peak:
                candidates.append((here, lx, ly))

    # Non-max suppression: keep the tallest peaks, spaced apart.
    candidates.sort(reverse=True)
    sep2 = (separation_ratio * max_dt) ** 2
    seeds: list[tuple[int, int]] = []
    for _, lx, ly in candidates:
        if all((lx - sx) ** 2 + (ly - sy) ** 2 >= sep2 for sx, sy in seeds):
            seeds.append((lx, ly))

    if len(seeds) <= 1:
        return [pixels]

    # Assign each pixel to its nearest seed (Euclidean, local coords).
    groups: list[list[int]] = [[] for _ in seeds]
    for p, lx, ly in zip(pixels, (x - min_x for x in xs), (y - min_y for y in ys), strict=True):
        best = 0
        best_d = None
        for si, (sx, sy) in enumerate(seeds):
            dd = (lx - sx) ** 2 + (ly - sy) ** 2
            if best_d is None or dd < best_d:
                best_d = dd
                best = si
        groups[best].append(p)
    return [g for g in groups if g]


def _distance_transform(mask: bytearray, bw: int, bh: int) -> list[float]:
    """Two-pass chamfer distance to the nearest non-mask pixel (approx Euclidean)."""
    inf = float("inf")
    dist = [inf if m else 0.0 for m in mask]
    a, b = 1.0, 1.4142135623730951
    for ly in range(bh):
        for lx in range(bw):
            k = ly * bw + lx
            if not mask[k]:
                continue
            best = dist[k]
            if lx > 0:
                best = min(best, dist[k - 1] + a)
            if ly > 0:
                best = min(best, dist[k - bw] + a)
            if lx > 0 and ly > 0:
                best = min(best, dist[k - bw - 1] + b)
            if lx < bw - 1 and ly > 0:
                best = min(best, dist[k - bw + 1] + b)
            dist[k] = best
    for ly in range(bh - 1, -1, -1):
        for lx in range(bw - 1, -1, -1):
            k = ly * bw + lx
            if not mask[k]:
                continue
            best = dist[k]
            if lx < bw - 1:
                best = min(best, dist[k + 1] + a)
            if ly < bh - 1:
                best = min(best, dist[k + bw] + a)
            if lx < bw - 1 and ly < bh - 1:
                best = min(best, dist[k + bw + 1] + b)
            if lx > 0 and ly < bh - 1:
                best = min(best, dist[k + bw - 1] + b)
            dist[k] = best
    return dist


def _summarize(pixels: list[int], frame: Frame, depth: DepthMap) -> Blob:
    w, c, d = frame.width, frame.channels, frame.data
    sx = sy = 0
    x_min = y_min = 1 << 30
    x_max = y_max = -1
    r_sum = g_sum = b_sum = 0
    for i in pixels:
        x = i % w
        y = i // w
        sx += x
        sy += y
        if x < x_min:
            x_min = x
        if x > x_max:
            x_max = x
        if y < y_min:
            y_min = y
        if y > y_max:
            y_max = y
        base = i * c
        r_sum += d[base]
        g_sum += d[base + 1]
        b_sum += d[base + 2]

    n = len(pixels)
    return Blob(
        centroid_px=(sx / n, sy / n),
        bbox=(x_min, y_min, x_max, y_max),
        area_px=n,
        depth_m=depth.sample(pixels),
        mean_rgb=(r_sum // n, g_sum // n, b_sum // n),
    )
