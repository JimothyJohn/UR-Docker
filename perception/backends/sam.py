"""Segment Anything (SAM) point/box-prompted segmentation via ``transformers``.

Optional backend for :class:`perception.segment.Segmenter`: install the
``sam`` extra (``uv sync --extra sam`` — torch + transformers + pillow). The
first call downloads the checkpoint into the Hugging Face cache. Runs on CUDA
(the Jetson), Apple ``mps``, or CPU — ``device=None`` picks the best available.

Model choice (``model_id`` / ``PERCEPTION_SAM_MODEL`` / ``--sam-model``), all
loadable through the same ``SamModel`` class:

* ``facebook/sam-vit-base`` (default, ~375 MB, 93 M params) — the reference
  quality; ~0.45 s per new frame on an M-series ``mps``.
* ``Zigeng/SlimSAM-uniform-50`` (~110 MB, 27 M params) — the light option;
  ~0.33 s per frame on ``mps``, masks within ~1 % of vit-base on our probe.

Interactive cost is dominated by the image encoder, so the encoder output is
**cached per frame**: the first prompt on a frame pays the ~0.3–0.5 s embed,
every further click / box on the *same* frame is a ~10–50 ms decode. The
cache key is the colour bytes, so a new frame invalidates it automatically.

API used (verified against transformers 5.16.1 on torch 2.13, mps): ``SamProcessor(image,
input_points=[[[x, y]]], input_boxes=[[[x0, y0, x1, y1]]], return_tensors="pt")``,
``SamModel.get_image_embeddings(pixel_values)``, ``SamModel(image_embeddings=…,
input_points=…, input_boxes=…, multimask_output=…)`` → ``pred_masks`` (batch,
point_batch, n, 256, 256) + ``iou_scores``, and
``processor.image_processor.post_process_masks(masks, original_sizes,
reshaped_input_sizes)`` to upsample to the frame size. Point-only prompts ask
for SAM's three candidates and keep the best predicted IoU; a box prompt is
unambiguous, so it asks for one mask (``multimask_output=False``). The
processor emits float64 prompt tensors, which ``mps`` refuses — everything
floating is cast to float32 before ``.to(device)``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..rgbd import RgbdFrame
from ..segment import Box, Mask, normalize_box

DEFAULT_MODEL_ID = "facebook/sam-vit-base"
FAST_MODEL_ID = "Zigeng/SlimSAM-uniform-50"


@dataclass
class SamSegmenter:
    name: str = "sam"
    model_id: str = DEFAULT_MODEL_ID
    device: str | None = None
    _model: Any = field(default=None, init=False, repr=False)
    _processor: Any = field(default=None, init=False, repr=False)
    _torch: Any = field(default=None, init=False, repr=False)
    _cache_key: Any = field(default=None, init=False, repr=False)
    _cache_emb: Any = field(default=None, init=False, repr=False)
    embed_hits: int = field(default=0, init=False)
    embed_misses: int = field(default=0, init=False)

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

    def _to_device(self, t):
        if not hasattr(t, "to"):
            return t
        if t.is_floating_point():
            return t.to(self.device, dtype=self._torch.float32)
        return t.to(self.device)

    def _embeddings(self, image, key: bytes):
        """Image-encoder output for ``image``, reused while the frame bytes match."""
        if self._cache_emb is not None and self._cache_key == key:
            self.embed_hits += 1
            return self._cache_emb
        self.embed_misses += 1
        inputs = self._processor(image, return_tensors="pt")
        with self._torch.no_grad():
            emb = self._model.get_image_embeddings(self._to_device(inputs["pixel_values"]))
        self._cache_key, self._cache_emb = key, emb
        return emb

    def segment(
        self, rgbd: RgbdFrame, point: tuple[int, int] | None = None, *, box: Box | None = None
    ) -> Mask:
        self._load()
        torch = self._torch
        frame = rgbd.color if rgbd.color.channels == 3 else rgbd.color.to_rgb()
        if box is not None:
            box = normalize_box(box, frame.width, frame.height)
        if point is None and box is None:
            raise ValueError("segment needs a point, a box, or both")
        if point is not None:
            x, y = point
            if not (0 <= x < frame.width and 0 <= y < frame.height):
                raise ValueError(f"seed ({x}, {y}) outside the {frame.width}x{frame.height} frame")
        from PIL import Image

        image = Image.frombytes("RGB", (frame.width, frame.height), frame.data)
        prompts: dict[str, Any] = {}
        if point is not None:
            prompts["input_points"] = [[[float(point[0]), float(point[1])]]]
        if box is not None:
            prompts["input_boxes"] = [[[float(v) for v in box]]]
        inputs = self._processor(image, return_tensors="pt", **prompts)
        emb = self._embeddings(image, frame.data)
        prompt_keys = ("input_points", "input_labels", "input_boxes")
        args = {k: self._to_device(v) for k, v in inputs.items() if k in prompt_keys}
        with torch.no_grad():
            outputs = self._model(image_embeddings=emb, multimask_output=box is None, **args)
        masks = self._processor.image_processor.post_process_masks(
            outputs.pred_masks.cpu(),
            inputs["original_sizes"].cpu(),
            inputs["reshaped_input_sizes"].cpu(),
        )
        # masks[0]: (point_batch=1, num_masks, H, W); iou_scores: (1, 1, num_masks)
        best = int(outputs.iou_scores[0, 0].argmax().item())
        m = masks[0][0, best].to(torch.uint8).contiguous().numpy()
        if m.shape != (frame.height, frame.width):
            raise RuntimeError(f"SAM returned a {m.shape} mask for a {frame.height}x{frame.width} frame")
        return Mask(frame.width, frame.height, m.tobytes())
