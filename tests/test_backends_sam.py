"""SAM backend against the real model (``sam`` marker: needs the ``sam`` extra
and a cached/downloadable checkpoint; deselect with ``-m "not sam"``).

Runs the light checkpoint so CI-with-a-cache stays under a minute; the
behaviours (point vs box prompt, embedding cache, mps float32) are the ones
that broke or were guessed before this test existed.
"""

from __future__ import annotations

import pytest

pytest.importorskip("torch")
pytest.importorskip("transformers")

from perception.backends.sam import FAST_MODEL_ID, SamSegmenter  # noqa: E402
from perception.frame import Frame  # noqa: E402
from perception.rgbd import DepthImage, RgbdFrame, synthetic_intrinsics  # noqa: E402

pytestmark = pytest.mark.sam


def _scene(width=320, height=240, rect=(90, 70, 230, 170), rgb=(220, 30, 30)):
    buf = bytearray(bytes((235, 235, 235)) * (width * height))
    x0, y0, x1, y1 = rect
    for y in range(y0, y1):
        for x in range(x0, x1):
            i = (y * width + x) * 3
            buf[i : i + 3] = bytes(rgb)
    return RgbdFrame(
        color=Frame(width, height, bytes(buf)),
        depth=DepthImage.from_metres(width, height, [0.8] * (width * height)),
        intrinsics=synthetic_intrinsics(width, height),
    )


@pytest.fixture(scope="module")
def seg():
    s = SamSegmenter(model_id=FAST_MODEL_ID)
    try:
        s._load()
    except Exception as exc:  # no network / no cache
        pytest.skip(f"SAM checkpoint unavailable: {exc}")
    return s


def test_point_and_box_prompts_find_the_rectangle(seg):
    f = _scene()
    x0, y0, x1, y1 = 90, 70, 230, 170
    expected = (x1 - x0) * (y1 - y0)
    by_point = seg.segment(f, (160, 120))
    by_box = seg.segment(f, box=(x0 - 5, y0 - 5, x1 + 5, y1 + 5))
    both = seg.segment(f, (160, 120), box=(x0 - 5, y0 - 5, x1 + 5, y1 + 5))
    for m in (by_point, by_box, both):
        assert (m.width, m.height) == (320, 240)
        assert abs(m.area - expected) / expected < 0.08
        bb = m.bbox()
        assert bb is not None and abs(bb[0] - x0) <= 4 and abs(bb[1] - y0) <= 4
        assert abs(bb[2] - (x1 - 1)) <= 4 and abs(bb[3] - (y1 - 1)) <= 4


def test_embedding_is_cached_per_frame(seg):
    f = _scene()
    seg._cache_key = seg._cache_emb = None
    seg.embed_hits = seg.embed_misses = 0
    seg.segment(f, (160, 120))
    seg.segment(f, box=(80, 60, 240, 180))
    seg.segment(f, (150, 110))
    assert (seg.embed_misses, seg.embed_hits) == (1, 2)
    seg.segment(_scene(rgb=(30, 30, 220)), (160, 120))  # new pixels -> re-embed
    assert seg.embed_misses == 2


def test_prompt_validation_before_any_inference(seg):
    f = _scene()
    with pytest.raises(ValueError):
        seg.segment(f)
    with pytest.raises(ValueError):
        seg.segment(f, (999, 5))
    with pytest.raises(ValueError):
        seg.segment(f, box=(0, 0, 400, 10))
