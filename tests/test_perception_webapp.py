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


def test_segment_by_box_and_by_box_plus_point(server):
    base, app, _ = server
    cx, cy, r, rgb, z = synthetic_disks(64, 48)[0]
    box = [cx - r - 2, cy - r - 2, cx + r + 2, cy + r + 2]
    status, j = post(base, "/api/segment", {"box": box})
    assert status == 200 and j["ok"] and j["area_px"] > 0
    assert j["prompt"] == {"box": box} and "x" not in j["prompt"]
    # corner order doesn't matter; the echo is normalized
    status, j2 = post(base, "/api/segment", {"box": [box[2], box[3], box[0], box[1]]})
    assert status == 200 and j2["prompt"] == {"box": box} and j2["area_px"] == j["area_px"]
    # point inside the box: both echoed
    status, j3 = post(base, "/api/segment", {"x": cx, "y": cy, "box": box})
    assert status == 200 and j3["prompt"] == {"x": cx, "y": cy, "box": box}
    # the mask never leaks outside the box
    bb = j3["features"]["bbox"]
    assert box[0] <= bb[0] and box[1] <= bb[1] and bb[2] < box[2] and bb[3] < box[3]


@pytest.mark.parametrize(
    "path,body,code",
    [
        ("/api/segment", {}, 400),
        ("/api/segment", {"x": 1}, 400),
        ("/api/segment", {"box": [0, 0, 0, 0]}, 400),
        ("/api/segment", {"box": [0, 0, 65, 10]}, 400),
        ("/api/segment", {"box": [-1, 0, 10, 10]}, 400),
        ("/api/segment", {"box": [0, 0, 10]}, 400),
        ("/api/segment", {"box": "0,0,10,10"}, 400),
        ("/api/segment", {"box": [0, 0, 10, float("nan")]}, 400),
        ("/api/segment", {"box": [0, 0, 10, True]}, 400),
        ("/api/segment", {"box": {"x0": 0}}, 400),
        ("/api/segment", {"x": 60, "y": 40, "box": [0, 0, 10, 10]}, 400),
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


def test_depth_flags_reach_the_camera(monkeypatch, tmp_path):
    """--depth-res / --no-depth-filters / --rs-preset / --laser-power (and their
    env twins) land on the RealSenseCamera the cockpit opens."""
    import argparse

    from perception.realsense import (
        DEFAULT_DEPTH_FILTERS,
        LASER_MAX,
        DepthTuning,
        RealSenseCamera,
    )
    from perception.webapp import camera_from_args, depth_tuning_from, parse_resolution

    assert parse_resolution("848x480") == (848, 480) and parse_resolution("1280×720") == (1280, 720)
    for bad in ("848", "0x480", "wxh", "-1x2"):
        with pytest.raises(ValueError):
            parse_resolution(bad)
    assert depth_tuning_from("high_accuracy", "max") == DepthTuning()
    assert depth_tuning_from("none", "none") is None and depth_tuning_from("", "") is None
    assert depth_tuning_from("None", "90") == DepthTuning(preset=None, laser_power=90.0, emitter=True)
    assert depth_tuning_from("default", "leave") == DepthTuning(
        preset="default", laser_power=None, emitter=None
    )
    assert depth_tuning_from("high_density", "MAX").laser_power == LASER_MAX
    with pytest.raises(ValueError, match="laser power"):
        depth_tuning_from("default", "lots")
    with pytest.raises(ValueError, match="unknown visual preset"):
        depth_tuning_from("turbo", "max")

    def ns(**kw):
        base = dict(fake=False, rs_fps=None, serial=None, no_align=False, library="/nope")
        base.update(kw)
        return argparse.Namespace(**base)

    monkeypatch.delenv("PERCEPTION_WIDTH", raising=False)
    monkeypatch.delenv("PERCEPTION_HEIGHT", raising=False)
    cam = camera_from_args(ns(), PerceptionConfig.from_env())
    assert isinstance(cam, RealSenseCamera)
    assert (cam.depth_width, cam.depth_height) == (848, 480)
    assert (cam.width, cam.height) == (848, 480)  # colour follows depth (mixed sizes -> black colour)
    assert cam.filters is DEFAULT_DEPTH_FILTERS and cam.tuning == DepthTuning()
    # an explicit colour size — flag or env — still wins
    cam = camera_from_args(ns(width=640, height=480), PerceptionConfig.from_env(width=640, height=480))
    assert (cam.width, cam.height) == (640, 480) and cam.depth_width == 848
    monkeypatch.setenv("PERCEPTION_WIDTH", "1280")
    monkeypatch.setenv("PERCEPTION_HEIGHT", "720")
    cam = camera_from_args(ns(), PerceptionConfig.from_env())
    assert (cam.width, cam.height) == (1280, 720) and cam.depth_width == 848
    monkeypatch.delenv("PERCEPTION_WIDTH")
    monkeypatch.delenv("PERCEPTION_HEIGHT")
    cam = camera_from_args(ns(depth_res="640x480"), PerceptionConfig.from_env())
    assert (cam.width, cam.height) == (640, 480) == (cam.depth_width, cam.depth_height)

    cam = camera_from_args(
        ns(depth_res="640x480", no_depth_filters=True, rs_preset="none", laser_power="0"),
        PerceptionConfig.from_env(),
    )
    assert (cam.depth_width, cam.depth_height) == (640, 480) and cam.filters is None
    assert cam.tuning == DepthTuning(preset=None, laser_power=0.0, emitter=True)

    monkeypatch.setenv("PERCEPTION_RS_DEPTH_WIDTH", "1280")
    monkeypatch.setenv("PERCEPTION_RS_DEPTH_HEIGHT", "720")
    monkeypatch.setenv("PERCEPTION_RS_FILTERS", "0")
    monkeypatch.setenv("PERCEPTION_RS_PRESET", "high_density")
    monkeypatch.setenv("PERCEPTION_RS_LASER_POWER", "none")
    cam = camera_from_args(ns(), PerceptionConfig.from_env())
    assert (cam.depth_width, cam.depth_height) == (1280, 720) and cam.filters is None
    assert cam.tuning == DepthTuning(preset="high_density", laser_power=None, emitter=None)
    # the synthetic camera ignores all of it
    assert isinstance(camera_from_args(ns(fake=True), PerceptionConfig.from_env()), SyntheticRgbdCamera)


# ----- robot link: send the segment's point to the robot -----------------------------


@pytest.fixture
def robot_server(tmp_path, monkeypatch):
    """The cockpit with a RobotLink whose Robot talks to test_urctl's fake controller."""
    from perception.robotlink import RobotLink
    from tests.test_urctl import FakeController
    from urctl.config import RobotConfig

    fake = FakeController().install(monkeypatch)
    link = RobotLink(RobotConfig(host="fake-ur"))
    app = ViewerApp(
        SyntheticRgbdCamera(width=64, height=48, fps=0),
        config=PerceptionConfig(),
        store=CaptureStore(tmp_path / "caps"),
        robot=link,
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield base, app, fake
    srv.shutdown()
    srv.server_close()
    app.stop()


def test_robot_panel_info_and_locate_needs_a_segment(robot_server):
    base, app, fake = robot_server
    _, _, body = get(base, "/api/info")
    info = json.loads(body)
    assert (
        info["robot"]["host"] == "fake-ur" and info["robot"]["handeye"]["source"] == "bracket-nominal:eseries"
    )
    # the synthetic camera's identity extrinsics were attached on open
    assert info["robot"]["handeye"]["depth_to_color_translation"] == [0.0, 0.0, 0.0]
    _, _, body = get(base, "/api/robot")
    assert json.loads(body)["robot"]["dry_run"] is False
    code, j = post(base, "/api/robot/locate", {})
    assert code == 400 and "segment an object" in j["error"]
    assert not fake.primary_sends  # nothing touched the controller


def test_robot_locate_then_move_round_trip(robot_server):
    base, app, fake = robot_server
    # segment the synthetic disk so the cockpit has a camera-frame point
    code, seg = post(base, "/api/nearest", {})
    assert code == 200 and seg["features"]["point_m"], seg
    code, loc = post(base, "/api/robot/locate", {"standoff_m": 0.05})
    assert code == 200 and loc["ok"], loc
    assert loc["point_cam_m"] == pytest.approx(seg["features"]["point_m"], abs=1e-9)
    assert len(loc["approach_pose"]) == 6 and loc["standoff_m"] == 0.05
    assert loc["approach_pose"][3:] == pytest.approx(fake.tcp_pose[3:], abs=1e-6)  # orientation kept
    assert loc["robot"]["tcp_offset"] == pytest.approx(fake.tcp_offset, abs=1e-6)
    assert len([s for s in fake.primary_sends if "urctl/flange" in s]) == 1
    # an explicit point overrides the segment's
    code, loc2 = post(base, "/api/robot/locate", {"point_m": [0.0, 0.0, 0.3]})
    assert code == 200 and loc2["point_cam_m"] == [0.0, 0.0, 0.3] and loc2["standoff_m"] == 0.1
    # move to the approach pose: one absolute movel, through the safety envelope
    code, mv = post(base, "/api/robot/move", {"pose": loc["approach_pose"]})
    assert code == 200 and mv["ok"] and mv["action"] == "move_tcp" and mv["safety"]["ok"], mv
    movel = [s for s in fake.primary_sends if "movel(" in s]
    assert len(movel) == 1 and "pose_add" not in movel[0] and "v=0.1" in movel[0]
    code, mv2 = post(base, "/api/robot/move", {"pose": loc["approach_pose"], "velocity": 0.05})
    assert code == 200 and "v=0.05" in [s for s in fake.primary_sends if "movel(" in s][-1]
    # state passes through the registry too
    code, st = post(base, "/api/robot/state", {})
    assert code == 200 and st["ok"] and st["robot_mode"]


@pytest.mark.parametrize(
    "path, body",
    [
        ("/api/robot/locate", {"standoff_m": "far"}),
        ("/api/robot/locate", {"standoff_m": 5.0}),
        ("/api/robot/locate", {"point_m": [0, 0]}),
        ("/api/robot/locate", {"point_m": [0, 0, "z"]}),
        ("/api/robot/locate", {"point_m": [0, 0, True]}),
        ("/api/robot/move", {}),
        ("/api/robot/move", {"pose": [0, 0, 0]}),
        ("/api/robot/move", {"pose": [0, 0, 0, 0, 0, "x"]}),
        ("/api/robot/move", {"pose": [0, 0, 0, 0, 0, 0], "velocity": 0}),
        ("/api/robot/move", {"pose": [0, 0, 0, 0, 0, 0], "velocity": 9}),
        ("/api/robot/move", {"pose": [1e400, 0, 0, 0, 0, 0]}),
    ],
)
def test_robot_bad_inputs_never_reach_the_controller(robot_server, path, body):
    base, app, fake = robot_server
    code, j = post(base, path, body)
    assert code == 400 and not j["ok"], j
    assert not any("movel(" in s for s in fake.primary_sends)


def test_robot_move_refused_by_envelope_is_reported_not_executed(robot_server):
    base, app, fake = robot_server
    fake.robot_mode = "POWER_OFF"
    code, mv = post(base, "/api/robot/move", {"pose": [0.5, 0, 0.3, 0, 3.14159, 0]})
    assert code == 200 and not mv["ok"] and mv["safety"]["ok"] is False
    assert not any("movel(" in s for s in fake.primary_sends)


def test_no_robot_link_is_a_clean_400(server):
    base, app, _ = server
    _, _, body = get(base, "/api/info")
    assert json.loads(body)["robot"] is None
    _, _, body = get(base, "/api/robot")
    assert json.loads(body)["robot"] is None
    for path in ("/api/robot/state", "/api/robot/locate", "/api/robot/move"):
        code, j = post(base, path, {"pose": [0, 0, 0, 0, 0, 0]})
        assert code == 400 and "no robot link" in j["error"]


def test_gui_main_robot_flags(monkeypatch, tmp_path):
    from perception.robotlink import RobotLink

    calls = {}
    monkeypatch.setattr("perception.webapp.serve", lambda camera, **kw: calls.update(kw))
    common = ["--fake", "--no-browser", "--out", str(tmp_path), "--port", "0"]
    assert gui_main([*common, "--no-robot"]) == 0 and calls["robot"] is None
    assert gui_main([*common, "--robot-host", "10.1.2.3", "--robot-dry-run"]) == 0
    link = calls["robot"]
    assert isinstance(link, RobotLink) and link.config.host == "10.1.2.3" and link.dry_run
    monkeypatch.setenv("UR_HOST", "10.9.9.9")
    assert gui_main(common) == 0 and calls["robot"].config.host == "10.9.9.9" and not calls["robot"].dry_run


# ---- pilot endpoints: jog / bring-up / stop / freedrive / doctor / events / snapshot -----


@pytest.fixture
def pilot(tmp_path):
    from perception.robotlink import RobotLink
    from urctl.config import RobotConfig

    app = ViewerApp(
        SyntheticRgbdCamera(width=64, height=48, fps=0),
        config=PerceptionConfig(),
        store=CaptureStore(tmp_path / "caps"),
        robot=RobotLink(RobotConfig(host="fake-ur"), dry_run=True),
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield f"http://127.0.0.1:{srv.server_address[1]}", app, tmp_path
    srv.shutdown()
    srv.server_close()
    app.stop()


def test_jog_is_capped_and_relative(pilot):
    base, app, _ = pilot
    status, j = post(base, "/api/robot/jog", {"delta": [0.01, 0, 0, 0, 0, 0]})
    assert status == 200 and j["ok"] and j["dry_run"] and j["action"] == "move_tcp"
    rec = app.robot.robot.audit.records[-1]
    assert rec.action == "move_tcp" and rec.args["relative"] is True and rec.args["pose"][0] == 0.01
    for bad in ([0.06, 0, 0, 0, 0, 0], [0, 0, 0, 0.5, 0, 0], [0, 0, 0, 0, 0, 0], [1, 2, 3]):
        status, j = post(base, "/api/robot/jog", {"delta": bad})
        assert status == 400 and not j["ok"], bad
    status, j = post(base, "/api/robot/jog", {"delta": [0.01, 0, 0, 0, 0, 0], "velocity": 0.9})
    assert status == 400 and "velocity" in j["error"]
    status, j = post(base, "/api/robot/jog", {"delta": ["a", 0, 0, 0, 0, 0]})
    assert status == 400


def test_bring_up_stop_freedrive_are_logged_events(pilot):
    base, app, _ = pilot
    for route, body in (
        ("/api/robot/bring_up", {}),
        ("/api/robot/stop", {}),
        ("/api/robot/freedrive", {"enable": True}),
    ):
        status, j = post(base, route, body)
        assert status == 200 and j["ok"] and j["dry_run"], route
    status, _, body = get(base, "/api/events")
    ev = json.loads(body)["events"]
    msgs = [e["message"] for e in ev if e["kind"] == "robot"]
    assert any(m.startswith("bring up") for m in msgs) and "stop" in msgs and "freedrive on" in msgs
    assert all(e["ok"] for e in ev if e["kind"] == "robot")
    # tailing: only events after `after`
    last = ev[-1]["seq"]
    status, _, body = get(base, f"/api/events?after={last}")
    assert json.loads(body)["events"] == []
    with pytest.raises(urllib.error.HTTPError) as ei:
        get(base, "/api/events?after=x")
    assert ei.value.code == 400


def test_doctor_endpoint_reports_the_live_stream_and_the_robot(pilot):
    base, app, _ = pilot
    status, _, body = get(base, "/api/doctor")
    doc = json.loads(body)
    names = {c["name"]: c for c in doc["checks"]}
    assert names["stream"]["ok"] is True and "synthetic" in names["stream"]["detail"]
    # a dry-run link to "fake-ur" is unreachable → robot.reach fails with a fix, no crash
    assert names["robot.reach"]["ok"] is False and names["robot.reach"]["fix"]
    status, _, body = get(base, "/api/doctor?robot=0")
    assert "robot.reach" not in {c["name"] for c in json.loads(body)["checks"]}


def test_snapshot_writes_viewable_pngs(pilot):
    base, app, tmp_path = pilot
    from perception.pngio import load_png

    status, j = post(base, "/api/snapshot", {"dir": str(tmp_path / "s"), "name": "look"})
    assert status == 200 and j["ok"] and j["features"] is None
    w, h, ch, data = load_png(j["color_png"])
    assert (w, h, ch) == (64, 48, 3) and len(data) == 64 * 48 * 3
    w, h, ch, data = load_png(j["depth_png"])
    assert (w, h, ch) == (64, 48, 3) and any(data)  # colourised, not blank
    # with a segment active the mask is written and the features ride along
    post(base, "/api/segment", {"x": 32, "y": 24})
    status, j = post(base, "/api/snapshot", {"dir": str(tmp_path / "s"), "name": "look2"})
    assert status == 200 and Path(j["mask_png"]).exists() and j["features"]["area_px"] > 0
    for bad in ({"name": "../x"}, {"name": ""}, {"dir": 3}):
        status, j = post(base, "/api/snapshot", bad)
        assert status == 400, bad


def test_pilot_routes_need_a_robot_link(server):
    base, _, _ = server  # the plain fixture has no robot link
    for route in ("/api/robot/jog", "/api/robot/bring_up", "/api/robot/stop", "/api/robot/freedrive"):
        status, j = post(base, route, {"delta": [0.01, 0, 0, 0, 0, 0], "enable": True})
        assert status == 400 and "no robot link" in j["error"], route
    status, _, body = get(base, "/api/doctor")
    assert "robot.reach" not in {c["name"] for c in json.loads(body)["checks"]}
    status, _, body = get(base, "/api/info")
    assert json.loads(body)["events_seq"] >= 1
