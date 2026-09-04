"""The RGB-D cockpit server end to end on the synthetic camera: frame
container + long-poll, segmentation, capture, and every bad-input path a
browser (or anything else on loopback) could throw at it."""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
import zlib
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

from perception.capture import CaptureStore
from perception.config import PerceptionConfig
from perception.realsense import RealSenseError, SyntheticRgbdCamera
from perception.rgbd import synthetic_disks, unpack_rgbd
from perception.webapp import ViewerApp, ViewerHandler
from perception.webapp import main as gui_main


class FlakyCamera(SyntheticRgbdCamera):
    """Fails to open the first ``fail_opens`` times, then behaves."""

    def __init__(self, fail_opens: int = 1, **kw):
        super().__init__(**kw)
        self.fail_opens = fail_opens
        self.opens = 0

    def open(self) -> None:
        self.opens += 1
        if self.opens <= self.fail_opens:
            raise RealSenseError("failed to set power state")
        super().open()


@pytest.fixture
def server(tmp_path):
    app = ViewerApp(
        SyntheticRgbdCamera(width=64, height=48, fps=0),
        config=PerceptionConfig(),
        store=CaptureStore(tmp_path / "caps"),
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    app.start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    # wait for the first frame
    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield base, app, tmp_path / "caps"
    srv.shutdown()
    srv.server_close()
    app.stop()


def get(base, path):
    with urllib.request.urlopen(base + path, timeout=5) as r:
        return r.status, r.headers.get("Content-Type", ""), r.read()


def post(base, path, body, raw=False):
    data = body if raw else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_index_and_info(server):
    base, app, _ = server
    status, ctype, body = get(base, "/")
    assert status == 200 and "text/html" in ctype and b"RGB-D cockpit" in body
    status, _, body = get(base, "/api/info")
    info = json.loads(body)
    assert status == 200 and info["ok"] and info["camera"]["kind"] == "synthetic"
    assert info["segmenter"] == "stub" and info["frame"]["width"] == 64 and info["seq"] >= 1
    assert info["last_error"] is None and not info["has_mask"]


def test_frame_container_and_long_poll(server):
    base, app, _ = server
    status, ctype, body = get(base, "/api/rgbd")
    assert status == 200 and ctype == "application/octet-stream"
    header, png, dz = unpack_rgbd(body)
    assert header["width"] == 64 and header["depth_height"] == 48 and "fps" in header
    depth = zlib.decompress(dz)
    assert len(depth) == 64 * 48 * 2 and png[:8] == b"\x89PNG\r\n\x1a\n"
    seq = header["seq"]
    # asking for something newer than `seq` blocks until the pump delivers it
    status, _, body2 = get(base, f"/api/rgbd?after={seq}&timeout_ms=3000")
    assert unpack_rgbd(body2)[0]["seq"] > seq
    # a timeout returns the newest frame rather than erroring
    app._running = False
    status, _, body3 = get(base, f"/api/rgbd?after={10**9}&timeout_ms=50")
    assert status == 200 and unpack_rgbd(body3)[0]["seq"] <= 10**9
    app._running = True


def test_bad_query_params(server):
    base, _, _ = server
    with pytest.raises(urllib.error.HTTPError) as ei:
        get(base, "/api/rgbd?after=abc")
    assert ei.value.code == 400
    with pytest.raises(urllib.error.HTTPError) as ei:
        get(base, "/api/nope")
    assert ei.value.code == 404


def test_segment_capture_clear_flow(server):
    base, app, root = server
    cx, cy, r, rgb, z = synthetic_disks(64, 48)[0]
    status, j = post(base, "/api/segment", {"x": cx, "y": cy})
    assert status == 200 and j["ok"] and j["area_px"] > 0 and j["mask_png_b64"]
    assert j["features"]["depth"]["median_m"] == pytest.approx(z) and j["features"]["mean_rgb"] == list(rgb)
    assert j["prompt"] == {"x": cx, "y": cy} and j["segmenter"] == "stub"
    assert json.loads(get(base, "/api/info")[2])["has_mask"]

    status, c = post(base, "/api/capture", {"name": "apple", "include_mask": True})
    assert status == 200 and c["ok"] and c["with_mask"] and c["index"] == 1
    files = sorted(p.name for p in (root / "apple").iterdir())
    assert files == ["color_00001.png", "depth_00001.png", "mask_00001.png", "meta_00001.json"]
    meta = json.loads((root / "apple" / "meta_00001.json").read_text())
    assert meta["features"]["area_px"] == j["area_px"] and meta["device"]["serial"] == "SYNTH"

    status, c2 = post(base, "/api/capture", {"name": "apple", "include_mask": False})
    assert c2["index"] == 2 and not c2["with_mask"]
    status, lst = get(base, "/api/captures")[0], json.loads(get(base, "/api/captures")[2])
    assert lst["sets"] == [{"name": "apple", "count": 2}]

    status, n = post(base, "/api/nearest", {})
    assert status == 200 and n["ok"] and n["prompt"] == {"nearest": 1.2} and n["area_px"] > 0
    status, cl = post(base, "/api/clear", {})
    assert cl["ok"] and not json.loads(get(base, "/api/info")[2])["has_mask"]
    # capture with no mask falls back to the live frame
    status, c3 = post(base, "/api/capture", {"name": "apple", "include_mask": True})
    assert c3["index"] == 3 and not c3["with_mask"]


@pytest.mark.parametrize(
    "path,body,code",
    [
        ("/api/segment", {"x": 9999, "y": 1}, 400),
        ("/api/segment", {"x": -1, "y": 1}, 400),
        ("/api/segment", {"x": "NaN", "y": 1}, 400),
        ("/api/segment", {"x": float("nan"), "y": 1}, 400),
        ("/api/segment", {"x": True, "y": 1}, 400),
        ("/api/segment", {"y": 1}, 400),
        ("/api/nearest", {"near_ratio": 0.5}, 400),
        ("/api/nearest", {"near_ratio": "big"}, 400),
        ("/api/capture", {"name": "../evil"}, 400),
        ("/api/capture", {"name": ""}, 400),
        ("/api/capture", {"name": "a" * 65}, 400),
        ("/api/bogus", {}, 404),
    ],
)
def test_bad_posts(server, path, body, code):
    base, _, root = server
    status, j = post(base, path, body)
    assert status == code and j["ok"] is False and "error" in j
    assert not (root / "..").exists() or not (root.parent / "evil").exists()


def test_malformed_bodies(server):
    base, _, _ = server
    assert post(base, "/api/segment", b"garbage", raw=True)[0] == 400
    assert post(base, "/api/segment", b"[1,2]", raw=True)[0] == 400
    assert post(base, "/api/segment", b"{" + b"\x00" * 70000 + b"}", raw=True)[0] == 400


def test_pump_recovers_from_open_failure(tmp_path):
    app = ViewerApp(FlakyCamera(fail_opens=1, width=32, height=24, fps=0), store=CaptureStore(tmp_path))
    app.start()
    try:
        deadline = time.monotonic() + 5
        while app.latest()[1] is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert app.latest()[1] is not None
        assert app.last_error is None  # cleared once the reopen succeeded
        assert app.camera.opens == 2
    finally:
        app.stop()


def test_reopen_delay_backs_off_and_caps():
    from perception.webapp import REOPEN_DELAY_S, REOPEN_MAX_DELAY_S, reopen_delay

    delays = [reopen_delay(n) for n in range(1, 12)]
    assert delays[0] == REOPEN_DELAY_S
    assert delays[:5] == [1.0, 2.0, 4.0, 8.0, 16.0]
    assert all(a <= b for a, b in zip(delays, delays[1:], strict=False))
    assert max(delays) == REOPEN_MAX_DELAY_S
    assert reopen_delay(0) == REOPEN_DELAY_S


def test_pump_reports_persistent_failure_with_hint(tmp_path):
    app = ViewerApp(FlakyCamera(fail_opens=10**6, width=32, height=24, fps=0), store=CaptureStore(tmp_path))
    app.start()
    try:
        deadline = time.monotonic() + 3
        while app.last_error is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert app.last_error and "failed to set power state" in app.last_error
        assert "sudo" in app.last_error or "udev" in app.last_error
        with pytest.raises(RuntimeError, match="no frame yet"):
            app.segment(1, 1)
        with pytest.raises(RuntimeError, match="no frame to capture"):
            app.capture("x")
    finally:
        app.stop()


def test_gui_main_help_and_fake_wiring(monkeypatch, tmp_path):
    with pytest.raises(SystemExit) as ei:
        gui_main(["--help"])
    assert ei.value.code == 0
    calls = {}

    def fake_serve(camera, **kw):
        calls["camera"] = camera
        calls.update(kw)

    monkeypatch.setattr("perception.webapp.serve", fake_serve)
    assert (
        gui_main(
            [
                "--fake",
                "--no-browser",
                "--out",
                str(tmp_path),
                "--port",
                "0",
                "--width",
                "32",
                "--height",
                "24",
            ]
        )
        == 0
    )
    assert isinstance(calls["camera"], SyntheticRgbdCamera) and calls["camera"].width == 32
    assert (
        calls["capture_root"] == str(tmp_path)
        and calls["open_browser"] is False
        and calls["bind"] == "127.0.0.1"
    )
    assert isinstance(calls["config"], PerceptionConfig) and Path(calls["capture_root"]).exists()
