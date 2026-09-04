"""Hardware-in-the-loop: a real RealSense through the ctypes binding.

Marked ``realsense`` and skipped automatically when the SDK is missing or no
camera can be opened (on macOS that means: not running as root). Run with

    sudo uv run pytest -m realsense -q
"""

from __future__ import annotations

import pytest

from perception.realsense import RealSenseCamera, RealSenseError, list_devices
from perception.segment import StubSegmenter, extract_features

pytestmark = pytest.mark.realsense


@pytest.fixture(scope="module")
def devices():
    try:
        devs = list_devices()
    except RealSenseError as exc:
        pytest.skip(f"RealSense not available: {exc}")
    if not devs:
        pytest.skip("no RealSense device attached")
    return devs


def test_enumerates_a_d4xx(devices):
    d = devices[0]
    assert d["serial"] and d["name"] and "RealSense" in d["name"]


def test_stream_aligned_frames_and_measure(devices):
    cam = RealSenseCamera(width=640, height=480, fps=30)
    with cam:
        assert cam.depth_scale and 0.0001 <= cam.depth_scale <= 0.01
        assert set(cam.intrinsics) == {"color", "depth"}
        frames = [cam.read() for _ in range(15)]  # let auto-exposure settle
    f = frames[-1]
    assert f.aligned and (f.color.width, f.color.height) == (640, 480)
    assert (f.depth.width, f.depth.height) == (640, 480)
    assert cam.tuning_applied["preset"]["ok"], cam.tuning_applied  # applied after the first frameset
    assert f.intrinsics == cam.intrinsics["color"]
    assert f.depth.stats()["valid_fraction"] > 0.1
    assert frames[-1].frame_number > frames[0].frame_number
    # the SDK's deprojection and ours agree on the live intrinsics
    api = cam.api
    for uv in ((320, 240), (10, 10), (630, 470)):
        assert f.intrinsics.deproject(*uv, 0.7) == pytest.approx(
            api.deproject(f.intrinsics, *uv, 0.7), abs=1e-5
        )
    m = StubSegmenter().nearest_object(f)
    if m.area:
        ft = extract_features(m, f)
        assert ft is not None and ft.point_m is not None and ft.point_m[2] > 0


def _temporal_noise(frames, box=40) -> float:
    """Mean per-pixel std-dev (metres) over ``frames`` in a centre ``box``×``box``
    patch, counting only pixels valid in every frame — pure Python, small patch."""
    w, h = frames[0].depth.width, frames[0].depth.height
    x0, y0 = w // 2 - box // 2, h // 2 - box // 2
    cols = [
        [f.depth.distance_m(x, y) for f in frames] for y in range(y0, y0 + box) for x in range(x0, x0 + box)
    ]
    stds = []
    for series in cols:
        if any(v is None for v in series):
            continue
        mean = sum(series) / len(series)
        stds.append((sum((v - mean) ** 2 for v in series) / len(series)) ** 0.5)
    assert len(stds) > box * box // 4, "too few pixels valid across the window — point the camera at a wall"
    return sum(stds) / len(stds)


def test_filtered_depth_is_steadier_than_raw(devices):
    """The post-processing chain + native depth mode + High Accuracy preset must
    cut frame-to-frame jitter on a static scene, and the chain must still hand
    back Z16 depth aligned to the colour grid (i.e. the disparity round-trip
    and the filter→align order are wired right)."""
    raw = RealSenseCamera(
        width=640, height=480, fps=30, depth_width=640, depth_height=480, filters=None, tuning=None
    )
    with raw:
        for _ in range(15):
            raw.read()
        raw_frames = [raw.read() for _ in range(20)]
    tuned = RealSenseCamera(width=640, height=480, fps=30)  # defaults: 848x480 depth, filters, tuning
    with tuned:
        assert tuned.intrinsics["depth"].width == 848 and tuned.intrinsics["color"].width == 640
        for _ in range(15):
            tuned.read()
        assert tuned.tuning_applied["preset"]["ok"], tuned.tuning_applied
        assert tuned.tuning_applied["laser_power"]["ok"], tuned.tuning_applied
        tuned_frames = [tuned.read() for _ in range(20)]
    f = tuned_frames[-1]
    assert (
        f.aligned
        and (f.depth.width, f.depth.height) == (640, 480)
        and f.intrinsics == tuned.intrinsics["color"]
    )
    assert f.depth.stats()["valid_fraction"] > 0.1
    noise_raw, noise_tuned = _temporal_noise(raw_frames), _temporal_noise(tuned_frames)
    print(f"temporal noise: raw {noise_raw * 1000:.2f} mm -> filtered {noise_tuned * 1000:.2f} mm")
    assert noise_tuned < noise_raw
