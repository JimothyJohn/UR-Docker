"""Extra viewpoints (perceptronics.views): the MJPEG splitter, device resolution,
the synthetic stand-in, and the cockpit's /api/view/<i> route + snapshot files."""

from __future__ import annotations

import json
import shutil
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from perceptronics.config import PerceptionConfig
from perceptronics.realsense import SyntheticRgbdCamera
from perceptronics.views import (
    FfmpegView,
    SyntheticView,
    ViewError,
    ffmpeg_input_args,
    open_views,
    parse_avfoundation_devices,
    parse_view_size,
    parse_view_specs,
    resolve_avfoundation,
    split_jpeg,
)
from perceptronics.webapp import ViewerApp, ViewerHandler

JPEG_A = b"\xff\xd8\xff\xe0" + b"\x00" * 10 + b"\xff\x00\xd9" + b"\xff\xd9"  # a stuffed FF inside
JPEG_B = b"\xff\xd8\xff\xdb" + b"\x11" * 5 + b"\xff\xd9"

# ffmpeg -f avfoundation -list_devices true -i "" on the Mac Studio, 2026-09-25
LISTING = """
[AVFoundation indev @ 0x75ac24140] AVFoundation video devices:
[AVFoundation indev @ 0x75ac24140] [0] HD Pro Webcam C920
[AVFoundation indev @ 0x75ac24140] [1] Logi Webcam C920e
[AVFoundation indev @ 0x75ac24140] [2] Intel(R) RealSense(TM) Depth Camera 435 with RGB Module RGB
[AVFoundation indev @ 0x75ac24140] [3] Capture screen 0
[AVFoundation indev @ 0x75ac24140] AVFoundation audio devices:
[AVFoundation indev @ 0x75ac24140] [0] PRO X Wireless Gaming Headset
[AVFoundation indev @ 0x75ac24140] [1] HD Pro Webcam C920
"""


def test_split_jpeg_pops_whole_frames_and_drops_leading_garbage():
    buf = bytearray(b"garbage" + JPEG_A + JPEG_B[:4])
    assert split_jpeg(buf) == JPEG_A  # the stuffed FF 00 D9 is not an end marker
    assert split_jpeg(buf) is None and bytes(buf) == JPEG_B[:4]
    buf.extend(JPEG_B[4:])
    assert split_jpeg(buf) == JPEG_B and not buf
    junk = bytearray(b"\x00\x01\x02\xff")  # no SOI: keep only the last byte (a possible half marker)
    assert split_jpeg(junk) is None and bytes(junk) == b"\xff"


def test_avfoundation_listing_and_name_resolution():
    devices = parse_avfoundation_devices(LISTING)
    assert devices == [
        (0, "HD Pro Webcam C920"),
        (1, "Logi Webcam C920e"),
        (2, "Intel(R) RealSense(TM) Depth Camera 435 with RGB Module RGB"),
        (3, "Capture screen 0"),
    ]  # the audio section (where the C920 appears again) is not read
    assert resolve_avfoundation("c920e", devices) == 1
    assert resolve_avfoundation("HD Pro", devices) == 0
    assert resolve_avfoundation("2", devices) == 2
    with pytest.raises(ViewError, match="matches several"):
        resolve_avfoundation("C920", devices)  # both Logitechs
    with pytest.raises(ViewError, match="attached: \\[0\\] HD Pro"):
        resolve_avfoundation("Brio", devices)


def test_input_args_per_platform(monkeypatch):
    assert ffmpeg_input_args("lavfi:testsrc", width=320, height=240, fps=10) == [
        "-re",
        "-f",
        "lavfi",
        "-i",
        "testsrc=size=320x240:rate=10",
    ]
    monkeypatch.setattr("perceptronics.views.sys.platform", "darwin")
    devs = parse_avfoundation_devices(LISTING)
    assert ffmpeg_input_args("Logi", width=640, height=480, fps=15, devices=devs)[-1] == "1"
    monkeypatch.setattr("perceptronics.views.sys.platform", "linux")
    assert ffmpeg_input_args("/dev/video2", width=640, height=480, fps=15) == [
        "-f",
        "v4l2",
        "-framerate",
        "15",
        "-video_size",
        "640x480",
        "-i",
        "/dev/video2",
    ]
    monkeypatch.setattr("perceptronics.views.sys.platform", "win32")
    assert (
        ffmpeg_input_args("HD Pro Webcam C920", width=640, height=480, fps=15)[-1]
        == "video=HD Pro Webcam C920"
    )


def test_specs_and_size_parsing():
    assert parse_view_specs(" HD Pro Webcam C920, Logi Webcam C920e ,,") == [
        "HD Pro Webcam C920",
        "Logi Webcam C920e",
    ]
    assert parse_view_specs("") == [] and parse_view_specs(None) == []
    assert parse_view_size("") == (640, 480) and parse_view_size("1280×720") == (1280, 720)
    with pytest.raises(ValueError):
        parse_view_size("wide")


def test_synthetic_view_is_png_and_paced():
    with SyntheticView(name="fake", width=32, height=24, fps=0) as v:
        a, b = v.read(), v.read()
        assert a[:8] == b"\x89PNG\r\n\x1a\n" and a != b  # the dot moves
        assert v.describe()["frames"] == 2 and v.describe()["content_type"] == "image/png"
    with pytest.raises(ViewError, match="not open"):
        v.read()
    fake = open_views(["a", "b"], fake=True, fps=30)
    assert [f.name for f in fake] == ["a", "b"] and all(isinstance(f, SyntheticView) for f in fake)
    assert fake[0].fps <= 8


def test_missing_ffmpeg_is_a_clear_error():
    with pytest.raises(ViewError, match="ffmpeg on PATH"):
        FfmpegView("lavfi:testsrc", binary="ffmpeg-definitely-not-installed").open()


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="needs ffmpeg")
def test_ffmpeg_synthetic_source_streams_jpegs():
    with FfmpegView("lavfi:testsrc", width=160, height=120, fps=10) as v:
        frames = [v.read() for _ in range(3)]
        assert all(f[:2] == b"\xff\xd8" and f[-2:] == b"\xff\xd9" for f in frames)
        assert v.describe()["open"] and v.describe()["device"] == "testsrc=size=160x120:rate=10"
    assert not v.describe()["open"]


# -- the cockpit route ----------------------------------------------------------------


@pytest.fixture
def server(tmp_path):
    app = ViewerApp(
        SyntheticRgbdCamera(width=32, height=24, fps=0),
        config=PerceptionConfig(),
        views=[
            SyntheticView(name="left", width=32, height=24, fps=0),
            SyntheticView(name="right", width=16, height=12, fps=0),
        ],
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    deadline = time.monotonic() + 5
    while (
        app.latest()[1] is None or any(v.latest()[1] is None for v in app.views)
    ) and time.monotonic() < deadline:
        time.sleep(0.01)
    yield base, app, tmp_path
    srv.shutdown()
    srv.server_close()
    app.stop()


def _get(url):
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return r.status, dict(r.headers), r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, dict(exc.headers), exc.read()


def test_view_route_serves_images_with_seq_and_long_polls(server):
    base, app, _ = server
    status, headers, body = _get(base + "/api/view/0")
    assert status == 200 and headers["Content-Type"] == "image/png" and body[:8] == b"\x89PNG\r\n\x1a\n"
    seq = int(headers["X-Seq"])
    assert seq >= 1 and "X-Fps" in headers
    status, headers, _ = _get(base + "/api/view/1?after=0&timeout_ms=1000")
    assert status == 200 and int(headers["X-Seq"]) >= 1
    # long-poll: a seq far ahead times out and returns the newest (same seq → nothing new)
    t = time.monotonic()
    status, headers, _ = _get(base + "/api/view/0?after=10000000&timeout_ms=300")
    assert status == 200 and time.monotonic() - t >= 0.25
    status, _, body = _get(base + "/api/view/7")
    assert status == 404 and json.loads(body)["error"] == "no view 7"
    status, _, _ = _get(base + "/api/view/x")
    assert status == 400


def test_info_lists_views_and_snapshot_writes_them(server):
    base, app, tmp_path = server
    info = json.loads(_get(base + "/api/info")[2])
    assert [v["name"] for v in info["views"]] == ["left", "right"]
    assert info["views"][0]["kind"] == "synthetic" and info["views"][0]["seq"] >= 1
    out = app.snapshot(str(tmp_path / "snaps"), "t")
    assert [v["name"] for v in out["views"]] == ["left", "right"]
    assert out["views"][0]["path"].endswith("t_view0.png") and out["views"][1]["path"].endswith("t_view1.png")
    assert (tmp_path / "snaps" / "t_view1.png").read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_no_views_means_no_view_route_and_empty_list(tmp_path):
    app = ViewerApp(
        SyntheticRgbdCamera(width=16, height=12, fps=0),
        config=PerceptionConfig(),
    )
    assert app.info()["views"] == []
    with pytest.raises(IndexError):
        app.view_frame(0, None, 0.1)


def test_stall_hint_names_tcc_on_macos(monkeypatch):
    from perceptronics.views import platform_hint

    monkeypatch.setattr("perceptronics.views.sys.platform", "darwin")
    hint = platform_hint(ViewError("stalled from 'HD Pro Webcam C920': nv12; 0rgb; bgr0"))
    assert "TCC" in hint and "SSH" in hint and "Screen Sharing" in hint
    monkeypatch.setattr("perceptronics.views.sys.platform", "linux")
    assert platform_hint(ViewError("stalled from '/dev/video0'")) == ""
