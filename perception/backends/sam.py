"""Segment Anything (SAM) point-prompted segmentation via ``transformers``.

Optional backend for :class:`perception.segment.Segmenter`: install the
``sam`` extra (``uv sync --extra sam`` — torch + transformers + pillow). The
first call downloads the checkpoint (``facebook/sam-vit-base`` by default,
~375 MB) into the Hugging Face cache; pass ``model_id`` for a smaller/larger
variant. Runs on CUDA (the Jetson), Apple ``mps``, or CPU — ``device=None``
picks the best available.

API used (verified against transformers 5.16): ``SamProcessor(images,
input_points=[[[x, y]]], return_tensors="pt")``, ``SamModel(**inputs)`` →
``pred_masks`` (batch, point_batch, 3, h, w) + ``iou_scores``, and
``processor.image_processor.post_process_masks(masks, original_sizes,
reshaped_input_sizes)`` to upsample to the frame size. Of SAM's three
candidate masks we keep the one with the best predicted IoU.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..rgbd import RgbdFrame
from ..segment import Mask

DEFAULT_MODEL_ID = "facebook/sam-vit-base"


@dataclass
class SamSegmenter:
    name: str = "sam"
    model_id: str = DEFAULT_MODEL_ID
    device: str | None = None
    _model: Any = field(default=None, init=False, repr=False)
    _processor: Any = field(default=None, init=False, repr=False)
    _torch: Any = field(default=None, init=False, repr=False)

    def _load(self) -> None:
        if self._model is not None:
            return
        try:
            import torch
            from transformers import SamModel, SamProcessor
        except ImportError as exc:  # pragma: no cover - only without the extra
            raise ImportError(
                "the SAM backend needs torch + transformers: install with `uv sync --extra sam`"
            ) from exc
        if self.device is None:
            if torch.cuda.is_available():
                self.device = "cuda"
            elif getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
                self.device = "mps"
            else:
                self.device = "cpu"
        self._torch = torch
        self._processor = SamProcessor.from_pretrained(self.model_id)
        self._model = SamModel.from_pretrained(self.model_id).to(self.device).eval()

    def segment(self, rgbd: RgbdFrame, point: tuple[int, int]) -> Mask:
        self._load()
        torch = self._torch
        frame = rgbd.color if rgbd.color.channels == 3 else rgbd.color.to_rgb()
        x, y = point
        if not (0 <= x < frame.width and 0 <= y < frame.height):
            raise ValueError(f"seed ({x}, {y}) outside the {frame.width}x{frame.height} frame")
        from PIL import Image

        image = Image.frombytes("RGB", (frame.width, frame.height), frame.data)
        inputs = self._processor(image, input_points=[[[float(x), float(y)]]], return_tensors="pt")
        inputs = {k: (v.to(self.device) if hasattr(v, "to") else v) for k, v in inputs.items()}
        with torch.no_grad():
            outputs = self._model(**inputs)
        masks = self._processor.image_processor.post_process_masks(
            outputs.pred_masks.cpu(),
            inputs["original_sizes"].cpu(),
            inputs["reshaped_input_sizes"].cpu(),
        )
        # masks[0]: (point_batch=1, num_masks=3, H, W); iou_scores: (1, 1, 3)
        best = int(outputs.iou_scores[0, 0].argmax().item())
        m = masks[0][0, best].to(torch.uint8).contiguous().numpy()
        if m.shape != (frame.height, frame.width):
            raise RuntimeError(f"SAM returned a {m.shape} mask for a {frame.height}x{frame.width} frame")
        return Mask(frame.width, frame.height, m.tobytes())
