"""perception — RGB -> monocular depth -> blob detection, parallel to ``urctl``.

A self-contained, dependency-free-at-core perception stack that mirrors the
``urctl`` control library's shape: env-driven config, Protocol-based pluggable
backends, a facade (:class:`PerceptionPipeline`), and a plain-data agent tool
registry. It deliberately does *not* import ``urctl`` — the two are parallel and
meet only at a future bridge that turns detected blobs into arm moves.

Quick start (no camera, no model weights)::

    from perception import PerceptionPipeline
    from perception.frame import synthetic_frame

    result = PerceptionPipeline().process(synthetic_frame())
    print(result.as_dict())

Swap in a real camera or the real depth model purely by config::

    PERCEPTION_DEPTH_BACKEND=depth_anything perceive capture
"""

from __future__ import annotations

from .blobs import Blob, BlobDetector, StubBlobDetector
from .config import PerceptionConfig
from .depth import DepthEstimator, DepthMap, StubDepthEstimator
from .factory import make_blob_detector, make_depth_estimator
from .frame import Frame, synthetic_frame
from .pipeline import PerceptionPipeline, PerceptionResult

__all__ = [
    "PerceptionConfig",
    "PerceptionPipeline",
    "PerceptionResult",
    "Frame",
    "synthetic_frame",
    "DepthMap",
    "DepthEstimator",
    "StubDepthEstimator",
    "Blob",
    "BlobDetector",
    "StubBlobDetector",
    "make_depth_estimator",
    "make_blob_detector",
]
