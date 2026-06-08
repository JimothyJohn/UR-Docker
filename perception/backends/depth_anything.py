"""Depth Anything V2 backend — the real monocular depth model.

Optional. Imported only when ``depth_backend="depth_anything"`` is selected, so
torch / transformers stay out of the dependency-free core. Install with::

    pip install -e .[perception-torch]

The model emits *relative* inverse depth; we normalize it and map into the
configured ``[near_m, far_m]`` window so the output matches the metres contract
of :class:`perception.depth.DepthMap` — exactly what the stub produces, so the
rest of the pipeline is identical regardless of backend.

This file is intentionally light on cleverness: it wraps the HuggingFace
``transformers`` pipeline. If you have a tuned local checkpoint, swap the
``model_id`` or replace ``_load`` — nothing else changes.
"""

from __future__ import annotations

from ..depth import DepthMap
from ..frame import Frame

# Small/fast checkpoint by default; bump to -base / -large for quality.
DEFAULT_MODEL_ID = "depth-anything/Depth-Anything-V2-Small-hf"


class DepthAnythingEstimator:
    """Learned monocular depth via HuggingFace transformers.

    Lazily loads the model on first :meth:`estimate`. Matches the
    :class:`perception.depth.DepthEstimator` protocol.
    """

    name = "depth_anything"

    def __init__(
        self,
        near_m: float = 0.20,
        far_m: float = 1.50,
        model_id: str = DEFAULT_MODEL_ID,
        device: str | None = None,
    ):
        if far_m <= near_m:
            raise ValueError(f"far_m ({far_m}) must exceed near_m ({near_m})")
        self.near_m = near_m
        self.far_m = far_m
        self.model_id = model_id
        self.device = device
        self._pipe = None

    def _load(self):
        if self._pipe is not None:
            return self._pipe
        try:
            import torch
            from transformers import pipeline
        except ImportError as exc:  # pragma: no cover - needs the extra installed
            raise ImportError(
                "Depth Anything V2 needs torch + transformers. "
                "Install with `pip install -e .[perception-torch]`."
            ) from exc
        device = self.device
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self._pipe = pipeline(task="depth-estimation", model=self.model_id, device=device)
        return self._pipe

    def estimate(self, frame: Frame) -> DepthMap:
        import numpy as np
        from PIL import Image

        pipe = self._load()
        rgb = frame.to_numpy()
        out = pipe(Image.fromarray(rgb))
        # transformers returns a PIL "depth" image (relative inverse depth).
        rel = np.asarray(out["depth"], dtype=np.float32)
        if rel.shape != (frame.height, frame.width):
            rel = np.asarray(Image.fromarray(rel).resize((frame.width, frame.height))).astype(np.float32)

        lo, hi = float(rel.min()), float(rel.max())
        span = hi - lo
        norm = (rel - lo) / span if span > 1e-9 else np.zeros_like(rel)
        # Higher inverse-depth value = nearer, so invert into metres.
        depth = self.far_m - norm * (self.far_m - self.near_m)
        return DepthMap(
            width=frame.width,
            height=frame.height,
            depth_m=depth.reshape(-1).tolist(),
        )
