"""Monocular depth estimation — interface + dependency-free stub backend.

A :class:`DepthEstimator` turns one RGB :class:`Frame` into a per-pixel
:class:`DepthMap` in metres. The real article is a learned monocular model
(Depth Anything V2, MiDaS, Metric3D); see
:mod:`perception.backends.depth_anything` for the optional torch backend.

:class:`StubDepthEstimator` is the always-available default. It is **not** a
neural network — it is a transparent, cheap heuristic that produces a *plausible*
depth field so the rest of the pipeline (and its tests) can run with no model
weights, no GPU, and no third-party deps. Treat its numbers as structurally
sensible, not metrically true. Swapping in the real backend is a config change
(``depth_backend="depth_anything"``), nothing downstream changes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .frame import Frame, normalize


@dataclass(frozen=True)
class DepthMap:
    """Per-pixel depth in metres, row-major, length ``width*height``."""

    width: int
    height: int
    depth_m: list[float]

    def __post_init__(self) -> None:
        if len(self.depth_m) != self.width * self.height:
            raise ValueError(f"depth has {len(self.depth_m)} values, expected {self.width * self.height}")

    def at(self, x: int, y: int) -> float:
        return self.depth_m[y * self.width + x]

    def sample(self, indices: list[int]) -> float:
        """Median depth over the given flat pixel indices (0.0 if empty)."""
        if not indices:
            return 0.0
        vals = sorted(self.depth_m[i] for i in indices)
        n = len(vals)
        mid = n // 2
        return vals[mid] if n % 2 else 0.5 * (vals[mid - 1] + vals[mid])

    def stats(self) -> dict:
        if not self.depth_m:
            return {"min_m": 0.0, "max_m": 0.0, "mean_m": 0.0}
        return {
            "min_m": min(self.depth_m),
            "max_m": max(self.depth_m),
            "mean_m": sum(self.depth_m) / len(self.depth_m),
        }


@runtime_checkable
class DepthEstimator(Protocol):
    """Anything that maps a Frame to a DepthMap. The pluggable seam."""

    name: str

    def estimate(self, frame: Frame) -> DepthMap: ...


class StubDepthEstimator:
    """Heuristic monocular depth — a placeholder for a learned model.

    Combines two cheap monocular cues that correlate with real depth in
    tabletop scenes:

      * **Vertical position** — pixels lower in the frame are nearer (a ground
        plane receding upward). This dominates the field.
      * **Brightness** — brighter regions read as slightly nearer (closer
        objects catch more light), adding local relief so distinct objects get
        distinct depths rather than a flat ramp.

    The blended cue is normalized to 0..1, then mapped into ``[near_m, far_m]``
    (low cue = near). Deterministic and O(pixels).
    """

    name = "stub"

    def __init__(self, near_m: float = 0.20, far_m: float = 1.50):
        if far_m <= near_m:
            raise ValueError(f"far_m ({far_m}) must exceed near_m ({near_m})")
        self.near_m = near_m
        self.far_m = far_m

    def estimate(self, frame: Frame) -> DepthMap:
        w, h = frame.width, frame.height
        luma = frame.luma()
        luma_n = normalize(luma)  # 0..1 brightness

        # "nearness" cue: high near the bottom, boosted by brightness.
        nearness = [0.0] * (w * h)
        inv_h = 1.0 / max(1, h - 1)
        for y in range(h):
            row_near = y * inv_h  # 0 at top, ~1 at bottom
            base = y * w
            for x in range(w):
                i = base + x
                nearness[i] = 0.7 * row_near + 0.3 * luma_n[i]

        nearness = normalize(nearness)
        span = self.far_m - self.near_m
        # Invert: more "nearness" -> smaller depth.
        depth = [self.far_m - n * span for n in nearness]
        return DepthMap(width=w, height=h, depth_m=depth)
