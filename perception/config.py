"""Perception configuration — *what* camera, *which* models, *how* fast.

Mirrors :mod:`urctl.config`: nothing in the package hardcodes a device index,
resolution, or model backend. Defaults are dev-friendly (a webcam at 640x480,
the dependency-free stub backends) and every field is overridable from the
environment so the same pipeline runs on a laptop webcam, a CI box with no
camera, or a GPU host with the real depth model — no code change::

    cfg = PerceptionConfig.from_env()                       # webcam + stubs
    cfg = PerceptionConfig.from_env(depth_backend="depth_anything")
    PERCEPTION_DEPTH_BACKEND=depth_anything python -m perception capture

This is the parallel to ``RobotConfig`` on the control side; a future bridge
that picks blobs with the arm will hold one of each.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

# Frame geometry / cadence. 640x480 @ 15 fps is a deliberate middle ground:
# enough resolution for blob centroids to be meaningful, slow enough that the
# pure-Python stub backends keep up on a laptop without a GPU.
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 15

# Which camera. -1 lets the device backend auto-pick the first working index.
DEFAULT_DEVICE_INDEX = 0

# Backend selection. "stub" is pure-Python and always importable; the real
# backends ("depth_anything", "blob_cv") are optional extras that lazily import
# torch / OpenCV and raise a clear install hint if missing.
DEFAULT_DEPTH_BACKEND = "stub"
DEFAULT_BLOB_BACKEND = "stub"

# The stub depth estimator emits a normalized 0..1 map; near/far scale it into
# metres so downstream consumers always see physical units. These bracket a
# typical tabletop pick workspace.
DEFAULT_DEPTH_NEAR_M = 0.20
DEFAULT_DEPTH_FAR_M = 1.50

# Blobs smaller than this (in pixels) are noise; drop them.
DEFAULT_MIN_BLOB_AREA = 60

# Stub blob segmentation. The stub finds *colorful* objects against a neutral
# (metallic / grey / white / black) background — the apples-on-steel case this
# repo targets. A pixel is foreground when its chroma (max channel - min
# channel) exceeds MIN_CHROMA; neighboring foreground pixels join the same blob
# only when their colors are within LINK_TOLERANCE (RGB distance), so touching
# objects of *different* colors (a red and a green apple) split apart while a
# single shaded object stays whole.
DEFAULT_BLOB_MIN_CHROMA = 45
DEFAULT_BLOB_LINK_TOLERANCE = 40

# Split a single colored region into instances when it has multiple distance-
# transform peaks (two touching same-colored apples -> two blobs). On by
# default; disable for raw connected-components behavior.
DEFAULT_BLOB_SPLIT_TOUCHING = True


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"environment variable {name}={raw!r} is not an integer") from exc


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"environment variable {name}={raw!r} is not a number") from exc


def _env_str(name: str, default: str) -> str:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else raw


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class PerceptionConfig:
    """Immutable description of the camera and the model backends to use."""

    width: int = DEFAULT_WIDTH
    height: int = DEFAULT_HEIGHT
    fps: int = DEFAULT_FPS
    device_index: int = DEFAULT_DEVICE_INDEX
    depth_backend: str = DEFAULT_DEPTH_BACKEND
    blob_backend: str = DEFAULT_BLOB_BACKEND
    depth_near_m: float = DEFAULT_DEPTH_NEAR_M
    depth_far_m: float = DEFAULT_DEPTH_FAR_M
    min_blob_area: int = DEFAULT_MIN_BLOB_AREA
    blob_min_chroma: int = DEFAULT_BLOB_MIN_CHROMA
    blob_link_tolerance: int = DEFAULT_BLOB_LINK_TOLERANCE
    blob_split_touching: bool = DEFAULT_BLOB_SPLIT_TOUCHING

    @classmethod
    def from_env(cls, **overrides) -> PerceptionConfig:
        """Build from ``PERCEPTION_*`` env vars, with explicit kwargs winning.

        Precedence (highest first): ``**overrides``, then environment, then the
        module defaults — same contract as :meth:`RobotConfig.from_env`.
        """
        values: dict[str, object] = {
            "width": _env_int("PERCEPTION_WIDTH", DEFAULT_WIDTH),
            "height": _env_int("PERCEPTION_HEIGHT", DEFAULT_HEIGHT),
            "fps": _env_int("PERCEPTION_FPS", DEFAULT_FPS),
            "device_index": _env_int("PERCEPTION_DEVICE", DEFAULT_DEVICE_INDEX),
            "depth_backend": _env_str("PERCEPTION_DEPTH_BACKEND", DEFAULT_DEPTH_BACKEND),
            "blob_backend": _env_str("PERCEPTION_BLOB_BACKEND", DEFAULT_BLOB_BACKEND),
            "depth_near_m": _env_float("PERCEPTION_DEPTH_NEAR_M", DEFAULT_DEPTH_NEAR_M),
            "depth_far_m": _env_float("PERCEPTION_DEPTH_FAR_M", DEFAULT_DEPTH_FAR_M),
            "min_blob_area": _env_int("PERCEPTION_MIN_BLOB_AREA", DEFAULT_MIN_BLOB_AREA),
            "blob_min_chroma": _env_int("PERCEPTION_BLOB_MIN_CHROMA", DEFAULT_BLOB_MIN_CHROMA),
            "blob_link_tolerance": _env_int(
                "PERCEPTION_BLOB_LINK_TOLERANCE", DEFAULT_BLOB_LINK_TOLERANCE
            ),
            "blob_split_touching": _env_bool(
                "PERCEPTION_BLOB_SPLIT_TOUCHING", DEFAULT_BLOB_SPLIT_TOUCHING
            ),
        }
        values.update(overrides)
        return cls(**values)  # type: ignore[arg-type]
