"""Unit tests for the perception package — no camera, no model weights, no deps.

Everything runs on synthetic frames through the pure-Python stub backends, so
these exercise the full stack: config, Frame, depth, blobs, the pipeline facade,
the tool registry, and the CLI. The optional torch/OpenCV backends are not
imported here (they're behind extras and the factory's lazy import).
"""

from __future__ import annotations

import json
import struct
import zlib
from pathlib import Path

import pytest

from perception import (
    PerceptionConfig,
    PerceptionPipeline,
    StubBlobDetector,
    StubDepthEstimator,
    synthetic_frame,
)
from perception.cli import main as cli_main
from perception.depth import DepthMap
from perception.factory import make_blob_detector, make_depth_estimator
from perception.frame import Frame, normalize
from perception.pngio import load_png
from perception.tools import ToolError, call_tool, get_tool_schemas

# The real fixture: three apples (two red, one green) on a brushed-steel table —
# the PickAndStack pick target. Lives at <repo>/inputs/image.png.
IMAGE_PATH = Path(__file__).resolve().parent.parent / "inputs" / "image.png"

# ----- config ----------------------------------------------------------------


def test_config_defaults_and_env(monkeypatch):
    assert PerceptionConfig().depth_backend == "stub"
    monkeypatch.setenv("PERCEPTION_WIDTH", "320")
    monkeypatch.setenv("PERCEPTION_DEPTH_BACKEND", "depth_anything")
    cfg = PerceptionConfig.from_env()
    assert cfg.width == 320
    assert cfg.depth_backend == "depth_anything"
    # explicit kwargs win over the environment
    assert PerceptionConfig.from_env(width=64).width == 64


def test_config_bad_env_is_clear(monkeypatch):
    monkeypatch.setenv("PERCEPTION_FPS", "fast")
    with pytest.raises(ValueError, match="PERCEPTION_FPS"):
        PerceptionConfig.from_env()


# ----- frame -----------------------------------------------------------------


def test_frame_validates_buffer_size():
    with pytest.raises(ValueError):
        Frame(width=2, height=2, data=b"\x00\x00\x00")  # too short


def test_synthetic_frame_paints_disks():
    f = synthetic_frame(64, 48)
    assert (f.width, f.height, f.channels) == (64, 48, 3)
    # center of the first (red) disk should be red, a corner should be background
    cx, cy, _, (r, g, b) = (
        64 // 4,
        48 // 2,
        4,
        (220, 40, 40),
    )
    assert f.pixel(cx, cy) == (r, g, b)
    assert f.pixel(0, 0) != (r, g, b)


def test_normalize_flat_is_zeros():
    assert normalize([5.0, 5.0, 5.0]) == [0.0, 0.0, 0.0]
    assert normalize([0.0, 5.0, 10.0]) == [0.0, 0.5, 1.0]


# ----- depth -----------------------------------------------------------------


def test_stub_depth_in_range_and_oriented():
    cfg = PerceptionConfig()
    est = StubDepthEstimator(near_m=cfg.depth_near_m, far_m=cfg.depth_far_m)
    f = synthetic_frame(48, 36)
    dm = est.estimate(f)
    assert isinstance(dm, DepthMap)
    assert all(cfg.depth_near_m - 1e-6 <= d <= cfg.depth_far_m + 1e-6 for d in dm.depth_m)
    # bottom rows read nearer than top rows (the dominant cue)
    top = sum(dm.at(x, 0) for x in range(f.width)) / f.width
    bottom = sum(dm.at(x, f.height - 1) for x in range(f.width)) / f.width
    assert bottom < top


def test_depth_far_must_exceed_near():
    with pytest.raises(ValueError):
        StubDepthEstimator(near_m=1.0, far_m=0.5)


def test_depth_sample_median():
    dm = DepthMap(width=2, height=2, depth_m=[0.1, 0.2, 0.3, 0.4])
    assert dm.sample([0, 1, 2, 3]) == pytest.approx(0.25)
    assert dm.sample([]) == 0.0


# ----- blobs -----------------------------------------------------------------


def test_stub_blob_detector_finds_three_disks():
    f = synthetic_frame(160, 120)
    est = StubDepthEstimator()
    dm = est.estimate(f)
    blobs = StubBlobDetector(min_area=20).detect(f, dm)
    assert len(blobs) == 3
    # sorted largest-first, each carries a physical depth and a mean color
    assert blobs[0].area_px >= blobs[-1].area_px
    for b in blobs:
        assert b.depth_m > 0
        x0, y0, x1, y1 = b.bbox
        assert x0 <= b.centroid_px[0] <= x1
        assert y0 <= b.centroid_px[1] <= y1


def test_blob_min_area_filters_noise():
    # one tiny disk below the area floor -> no blobs
    f = synthetic_frame(80, 60, disks=[(40, 30, 2, (240, 0, 0))])
    dm = StubDepthEstimator().estimate(f)
    assert StubBlobDetector(min_area=200).detect(f, dm) == []


def test_splitter_separates_touching_same_color_disks():
    # Two overlapping red disks form ONE connected color region; the distance-
    # transform splitter recovers two instances.
    f = synthetic_frame(140, 80, disks=[(50, 40, 22, (230, 30, 30)), (88, 40, 22, (230, 30, 30))])
    dm = StubDepthEstimator().estimate(f)
    assert len(StubBlobDetector(min_area=80, split_touching=True).detect(f, dm)) == 2
    # Without the split they merge into one blob.
    assert len(StubBlobDetector(min_area=80, split_touching=False).detect(f, dm)) == 1


def test_splitter_keeps_single_disk_whole():
    # A lone convex disk has one DT peak -> not over-split.
    f = synthetic_frame(120, 90, disks=[(60, 45, 28, (230, 30, 30))])
    dm = StubDepthEstimator().estimate(f)
    assert len(StubBlobDetector(min_area=80, split_touching=True).detect(f, dm)) == 1


# ----- factory + pipeline ----------------------------------------------------


def test_factory_selects_stub_and_rejects_unknown():
    cfg = PerceptionConfig()
    assert make_depth_estimator(cfg).name == "stub"
    assert make_blob_detector(cfg).name == "stub"
    with pytest.raises(ValueError, match="unknown depth backend"):
        make_depth_estimator(PerceptionConfig(depth_backend="bogus"))
    with pytest.raises(ValueError, match="unknown blob backend"):
        make_blob_detector(PerceptionConfig(blob_backend="bogus"))


def test_pipeline_process_result_shape():
    pipe = PerceptionPipeline(PerceptionConfig(min_blob_area=20))
    result = pipe.process(synthetic_frame(160, 120))
    d = result.as_dict()
    assert d["ok"] is True
    assert d["n_blobs"] == 3 == len(d["blobs"])
    assert d["backends"] == {"depth": "stub", "blob": "stub"}
    assert set(d["blobs"][0]) == {"centroid_px", "bbox", "area_px", "depth_m", "mean_rgb"}
    assert d["depth_stats"]["max_m"] >= d["depth_stats"]["min_m"]


def test_pipeline_accepts_injected_backends():
    pipe = PerceptionPipeline(depth=StubDepthEstimator(), blob=StubBlobDetector(min_area=20))
    res = pipe.process(synthetic_frame(120, 90))
    assert len(res.blobs) == 3


# ----- tool registry ---------------------------------------------------------


def test_tool_schemas_are_anthropic_shaped():
    schemas = get_tool_schemas()
    names = {s["name"] for s in schemas}
    assert {"perceive_frame", "perceive_synthetic"} <= names
    for s in schemas:
        assert set(s) == {"name", "description", "input_schema"}
        assert s["input_schema"]["type"] == "object"


def test_call_perceive_synthetic():
    pipe = PerceptionPipeline(PerceptionConfig(min_blob_area=20))
    out = call_tool(pipe, "perceive_synthetic", {"width": 160, "height": 120})
    assert out["ok"] is True
    assert out["n_blobs"] == 3


def test_call_tool_validation():
    pipe = PerceptionPipeline()
    with pytest.raises(ToolError, match="unknown tool"):
        call_tool(pipe, "nope")
    with pytest.raises(ToolError, match="unexpected argument"):
        call_tool(pipe, "perceive_synthetic", {"bogus": 1})
    with pytest.raises(ToolError, match="must be integer"):
        call_tool(pipe, "perceive_synthetic", {"width": "big"})


# ----- CLI -------------------------------------------------------------------


def test_cli_synthetic(capsys):
    rc = cli_main(["--min-blob-area", "20", "synthetic", "--width", "160", "--height", "120"])
    assert rc == 0
    out = json.loads(capsys.readouterr().out)
    assert out["ok"] is True
    assert out["n_blobs"] == 3


def test_cli_tools(capsys):
    rc = cli_main(["tools"])
    assert rc == 0
    schemas = json.loads(capsys.readouterr().out)
    assert any(s["name"] == "perceive_frame" for s in schemas)


def test_cli_call(capsys):
    rc = cli_main(["call", "perceive_synthetic", "--json", '{"width":160,"height":120}'])
    assert rc == 0
    out = json.loads(capsys.readouterr().out)
    assert out["n_blobs"] == 3


# ----- PNG decoder (dependency-free) -----------------------------------------


def _encode_png(width: int, height: int, rgb: bytes) -> bytes:
    """Minimal filter-0 RGB PNG encoder — for round-tripping the decoder."""

    def chunk(tag: bytes, body: bytes) -> bytes:
        return (
            struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8-bit RGB
    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)  # filter type None
        raw += rgb[y * stride : (y + 1) * stride]
    sig = b"\x89PNG\r\n\x1a\n"
    return sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(bytes(raw))) + chunk(b"IEND", b"")


def test_png_decoder_roundtrip_exact(tmp_path):
    rgb = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0])  # 2x2: R G B Y
    p = tmp_path / "tiny.png"
    p.write_bytes(_encode_png(2, 2, rgb))
    w, h, c, buf = load_png(str(p))
    assert (w, h, c) == (2, 2, 3)
    assert buf == rgb
    # round-trips through Frame too
    f = Frame.from_png(str(p))
    assert f.pixel(0, 0) == (255, 0, 0)
    assert f.pixel(1, 1) == (255, 255, 0)


def test_png_decoder_rejects_non_png(tmp_path):
    bad = tmp_path / "bad.png"
    bad.write_bytes(b"not a png")
    with pytest.raises(ValueError, match="not a PNG"):
        load_png(str(bad))


# ----- Frame loading / scaling -----------------------------------------------


def test_frame_resize_and_to_rgb():
    f = synthetic_frame(64, 48)
    small = f.resized(16, 12)
    assert (small.width, small.height) == (16, 12)
    # to_rgb drops alpha
    rgba = Frame(width=1, height=1, data=bytes([10, 20, 30, 40]), channels=4)
    assert rgba.to_rgb().channels == 3
    assert rgba.to_rgb().pixel(0, 0) == (10, 20, 30)


# ----- the real apples-on-steel image ----------------------------------------


@pytest.fixture(scope="module")
def apple_frame() -> Frame:
    if not IMAGE_PATH.exists():
        pytest.skip(f"fixture image missing: {IMAGE_PATH}")
    # Decode once, downsample for the pure-Python pipeline.
    return Frame.from_png(str(IMAGE_PATH)).resized(352, 192)


def test_real_image_loads_at_full_resolution():
    if not IMAGE_PATH.exists():
        pytest.skip(f"fixture image missing: {IMAGE_PATH}")
    f = Frame.from_png(str(IMAGE_PATH))
    assert (f.width, f.height, f.channels) == (1408, 768, 3)  # alpha dropped


def test_real_image_segments_into_three_apples(apple_frame):
    # Default config splits the touching red pair -> 3 instances (2 red, 1 green).
    result = PerceptionPipeline(PerceptionConfig()).process(apple_frame)
    blobs = result.blobs
    assert len(blobs) == 3

    reds = [b for b in blobs if b.mean_rgb[0] > b.mean_rgb[1] and b.mean_rgb[0] > b.mean_rgb[2]]
    greens = [b for b in blobs if b.mean_rgb[1] > b.mean_rgb[0] and b.mean_rgb[1] > b.mean_rgb[2]]
    assert len(reds) == 2  # the two red apples, split by the watershed
    assert len(greens) == 1  # the green apple

    # All are sizable, colorful (chroma high -> steel/white text rejected),
    # and sit in the central band where the apples are (not at the edges).
    w, h = apple_frame.width, apple_frame.height
    for b in blobs:
        assert b.area_px > 300
        cr, cg, cb = b.mean_rgb
        assert max(cr, cg, cb) - min(cr, cg, cb) > 30
        cx, cy = b.centroid_px
        assert 0.2 * w < cx < 0.95 * w
        assert 0.1 * h < cy < 0.9 * h
        assert 0.0 < b.depth_m  # carries a depth reading


def test_real_image_without_splitting_merges_red_pair(apple_frame):
    # Disabling the instance split falls back to color-region segmentation:
    # the touching red apples merge -> 2 regions (red cluster + green).
    result = PerceptionPipeline(PerceptionConfig(blob_split_touching=False)).process(apple_frame)
    assert len(result.blobs) == 2


def test_real_image_background_corners_are_not_detected(apple_frame):
    """Steel + overlay text are near-neutral, so no blob covers a frame corner."""
    result = PerceptionPipeline(PerceptionConfig()).process(apple_frame)
    w, h = apple_frame.width, apple_frame.height
    corners = [(0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)]
    for b in result.blobs:
        x0, y0, x1, y1 = b.bbox
        for cx, cy in corners:
            assert not (x0 <= cx <= x1 and y0 <= cy <= y1)


def test_real_image_via_cli(capsys):
    if not IMAGE_PATH.exists():
        pytest.skip(f"fixture image missing: {IMAGE_PATH}")
    rc = cli_main(["image", str(IMAGE_PATH), "--max-width", "352"])
    assert rc == 0
    out = json.loads(capsys.readouterr().out)
    assert out["ok"] is True
    assert out["n_blobs"] == 3


def test_real_image_tool_call():
    if not IMAGE_PATH.exists():
        pytest.skip(f"fixture image missing: {IMAGE_PATH}")
    pipe = PerceptionPipeline(PerceptionConfig())
    out = call_tool(pipe, "perceive_image", {"path": str(IMAGE_PATH), "max_width": 352})
    assert out["ok"] is True
    assert out["n_blobs"] == 3
