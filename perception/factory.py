"""Backend factories — turn a config string into a model instance.

One place that maps ``config.depth_backend`` / ``config.blob_backend`` to a
concrete estimator/detector, importing the optional heavy backends lazily so
selecting ``"stub"`` never touches torch or OpenCV. This is the seam the
pipeline goes through, and the seam a future ``urctl`` integration would reuse.
"""

from __future__ import annotations

from .blobs import BlobDetector, StubBlobDetector
from .config import PerceptionConfig
from .depth import DepthEstimator, StubDepthEstimator

DEPTH_BACKENDS = ("stub", "depth_anything")
BLOB_BACKENDS = ("stub", "blob_cv")


def make_depth_estimator(config: PerceptionConfig) -> DepthEstimator:
    name = config.depth_backend
    if name == "stub":
        return StubDepthEstimator(near_m=config.depth_near_m, far_m=config.depth_far_m)
    if name == "depth_anything":
        from .backends.depth_anything import DepthAnythingEstimator

        return DepthAnythingEstimator(near_m=config.depth_near_m, far_m=config.depth_far_m)
    raise ValueError(f"unknown depth backend {name!r}; choose from {DEPTH_BACKENDS}")


def make_blob_detector(config: PerceptionConfig) -> BlobDetector:
    name = config.blob_backend
    if name == "stub":
        return StubBlobDetector(
            min_area=config.min_blob_area,
            min_chroma=config.blob_min_chroma,
            link_tolerance=config.blob_link_tolerance,
            split_touching=config.blob_split_touching,
        )
    if name == "blob_cv":
        from .backends.blob_cv import CvBlobDetector

        return CvBlobDetector(min_area=config.min_blob_area)
    raise ValueError(f"unknown blob backend {name!r}; choose from {BLOB_BACKENDS}")
