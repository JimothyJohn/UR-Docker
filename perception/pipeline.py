"""PerceptionPipeline — the facade humans and agents drive.

This is the perception-side analogue of :class:`urctl.robot.Robot`: one object
that combines the configured depth estimator and blob detector and turns a
:class:`~perception.frame.Frame` into a structured :class:`PerceptionResult`.

    from perception import PerceptionPipeline, PerceptionConfig
    from perception.frame import synthetic_frame

    pipe = PerceptionPipeline(PerceptionConfig.from_env())
    result = pipe.process(synthetic_frame())
    print(result.as_dict())          # JSON-ready: blobs with bbox + depth_m

The pipeline is backend-agnostic — ``PerceptionConfig`` decides whether depth
comes from the heuristic stub or Depth Anything V2, and whether blobs come from
the pure-Python labeler or OpenCV — without changing this code or its output
shape. :meth:`process_device` wires a live webcam in for an end-to-end run.

**Integration seam (not yet wired):** ``PerceptionResult.blobs`` carries
``centroid_px`` + ``depth_m`` per blob. A future bridge adds camera intrinsics
+ a camera->base transform to deproject each into a base-frame point and hand it
to ``urctl.Robot.move_tcp`` — the reason blobs stop at 2D+depth for now.
"""

from __future__ import annotations

from dataclasses import dataclass

from .blobs import Blob, BlobDetector
from .config import PerceptionConfig
from .depth import DepthEstimator, DepthMap
from .factory import make_blob_detector, make_depth_estimator
from .frame import Frame


@dataclass(frozen=True)
class PerceptionResult:
    """The output of one frame: detected blobs + provenance + depth summary."""

    width: int
    height: int
    blobs: list[Blob]
    depth_stats: dict
    depth_backend: str
    blob_backend: str

    def as_dict(self) -> dict:
        return {
            "ok": True,
            "width": self.width,
            "height": self.height,
            "n_blobs": len(self.blobs),
            "blobs": [b.as_dict() for b in self.blobs],
            "depth_stats": self.depth_stats,
            "backends": {"depth": self.depth_backend, "blob": self.blob_backend},
        }


class PerceptionPipeline:
    """Frame -> depth -> blobs, wired from a :class:`PerceptionConfig`.

    Pass explicit ``depth``/``blob`` instances to override the config-selected
    backends (useful in tests or to inject a custom model).
    """

    def __init__(
        self,
        config: PerceptionConfig | None = None,
        *,
        depth: DepthEstimator | None = None,
        blob: BlobDetector | None = None,
    ):
        self.config = config or PerceptionConfig.from_env()
        self.depth = depth or make_depth_estimator(self.config)
        self.blob = blob or make_blob_detector(self.config)

    def estimate_depth(self, frame: Frame) -> DepthMap:
        return self.depth.estimate(frame)

    def process(self, frame: Frame) -> PerceptionResult:
        """Run depth + blob detection on a single RGB frame."""
        depth = self.depth.estimate(frame)
        blobs = self.blob.detect(frame, depth)
        return PerceptionResult(
            width=frame.width,
            height=frame.height,
            blobs=blobs,
            depth_stats=depth.stats(),
            depth_backend=self.depth.name,
            blob_backend=self.blob.name,
        )

    def process_image(self, path: str, *, max_width: int | None = 480) -> PerceptionResult:
        """Load a PNG, optionally downsample, and run the pipeline on it.

        ``max_width`` shrinks the frame (preserving aspect) so the pure-Python
        backends stay fast; pass ``None`` to process at full resolution (slow on
        large images). Uses the dependency-free :meth:`Frame.from_png` loader.
        """
        frame = Frame.from_png(path)
        if max_width is not None and frame.width > max_width:
            scale = max_width / frame.width
            frame = frame.resized(max_width, max(1, round(frame.height * scale)))
        return self.process(frame)

    def process_device(self, source=None) -> PerceptionResult:
        """Grab one frame from a webcam :class:`DeviceSource` and process it.

        ``source`` defaults to a :class:`~perception.sources.DeviceSource` built
        from this pipeline's config. Importing it here keeps OpenCV out of the
        import path until a live camera is actually used.
        """
        if source is None:
            from .sources import DeviceSource

            source = DeviceSource(self.config)
        frame = source.read_one()
        return self.process(frame)
