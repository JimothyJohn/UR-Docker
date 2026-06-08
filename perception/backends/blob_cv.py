"""OpenCV blob backend — ``cv2.SimpleBlobDetector``.

Optional. Imported only when ``blob_backend="blob_cv"`` is selected. Produces
the same :class:`perception.blobs.Blob` records as the stub (2D geometry +
sampled depth + mean color), so the pipeline output is backend-independent.

OpenCV's detector finds blobs on a grayscale image by thresholding across a
range and grouping stable keypoints; it gives a center and a size (diameter)
rather than an exact pixel mask, so the bbox here is derived from the keypoint
size and the area is the disk area implied by that diameter.
"""

from __future__ import annotations

from ..blobs import Blob
from ..depth import DepthMap
from ..frame import Frame


class CvBlobDetector:
    """``cv2.SimpleBlobDetector`` wrapper matching the BlobDetector protocol."""

    name = "blob_cv"

    def __init__(self, min_area: int = 60, threshold: float = 60.0):
        self.min_area = min_area
        self.threshold = threshold
        self._detector = None

    def _build(self):
        if self._detector is not None:
            return self._detector
        cv2 = _require_cv2()
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = float(self.min_area)
        params.filterByColor = False
        params.filterByConvexity = False
        params.filterByInertia = False
        params.filterByCircularity = False
        self._detector = cv2.SimpleBlobDetector_create(params)
        return self._detector

    def detect(self, frame: Frame, depth: DepthMap) -> list[Blob]:
        cv2 = _require_cv2()
        import numpy as np

        rgb = frame.to_numpy()
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        keypoints = self._build().detect(gray)

        blobs: list[Blob] = []
        for kp in keypoints:
            cx, cy = kp.pt
            r = max(1.0, kp.size / 2.0)
            x_min = max(0, int(cx - r))
            y_min = max(0, int(cy - r))
            x_max = min(frame.width - 1, int(cx + r))
            y_max = min(frame.height - 1, int(cy + r))

            # Sample depth + mean color over the keypoint disk.
            indices: list[int] = []
            r2 = r * r
            for y in range(y_min, y_max + 1):
                for x in range(x_min, x_max + 1):
                    if (x - cx) ** 2 + (y - cy) ** 2 <= r2:
                        indices.append(y * frame.width + x)
            area = len(indices) or int(np.pi * r2)
            patch = rgb[y_min : y_max + 1, x_min : x_max + 1].reshape(-1, frame.channels)
            mean = patch.mean(axis=0).astype(int)
            blobs.append(
                Blob(
                    centroid_px=(cx, cy),
                    bbox=(x_min, y_min, x_max, y_max),
                    area_px=area,
                    depth_m=depth.sample(indices),
                    mean_rgb=(int(mean[0]), int(mean[1]), int(mean[2])),
                )
            )

        blobs.sort(key=lambda b: b.area_px, reverse=True)
        return blobs


def _require_cv2():
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - needs the extra installed
        raise ImportError(
            "OpenCV (cv2) is required for the blob_cv backend. Install with `pip install -e .[perception]`."
        ) from exc
    return cv2
