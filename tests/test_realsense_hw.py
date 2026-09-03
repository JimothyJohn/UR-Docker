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
