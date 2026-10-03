"""The pre-flight doctor against fakes: closed ports, a platform mismatch, the
synthetic camera stream, a fake controller in every state, and the verdicts."""

from __future__ import annotations

import socket
import threading

import pytest

from perceptronics.doctor import Report, check_stream, run_doctor, sdk_version_text
from perceptronics.realsense import SyntheticRgbdCamera
from tests.test_urctl import FakeController
from urctl.config import RobotConfig
from urctl.robot import Robot


def _closed_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def listener():
    """A TCP port that accepts (and drops) connections — 'open' to the probe."""
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(5)
    stop = threading.Event()

    def run():
        srv.settimeout(0.2)
        while not stop.is_set():
            try:
                c, _ = srv.accept()
                c.close()
            except TimeoutError:
                pass
            except OSError:
                break

    t = threading.Thread(target=run, daemon=True)
    t.start()
    yield srv.getsockname()[1]
    stop.set()
    srv.close()


def _by_name(report: dict) -> dict:
    return {c["name"]: c for c in report["checks"]}


def test_sdk_version_text():
    assert sdk_version_text(25804) == "2.58.4"
    assert sdk_version_text("2.58.4") == "2.58.4"
    assert sdk_version_text(None) == "None"


def test_unreachable_robot_is_a_critical_failure_with_a_fix():
    cfg = RobotConfig(
        host="127.0.0.1", platform="polyscopex", robot_api_port=_closed_port(), dashboard_port=_closed_port()
    )
    rep = run_doctor(robot_config=cfg, camera=False, env={"UR_CELL": "sim"}).as_dict()
    c = _by_name(rep)
    assert rep["ok"] is False and rep["motion_ok"] is False and rep["failed"] == ["robot.reach"]
    assert "unreachable" in c["robot.reach"]["detail"] and c["robot.reach"]["fix"]
    assert c["cell"]["ok"] is None  # cell selected → informational


def test_platform_mismatch_is_diagnosed(listener):
    # cell says polyscopex, but only the Dashboard port answers → e-series controller
    cfg = RobotConfig(
        host="127.0.0.1", platform="polyscopex", robot_api_port=_closed_port(), dashboard_port=listener
    )
    c = _by_name(run_doctor(robot_config=cfg, camera=False, env={}).as_dict())
    assert c["robot.reach"]["ok"] is False and "UR_PLATFORM=e-series" in c["robot.reach"]["fix"]
    assert c["cell"]["ok"] is False  # no cell selected → warn


def test_empty_host_is_named():
    cfg = RobotConfig(host="")
    c = _by_name(run_doctor(robot_config=cfg, camera=False, env={}).as_dict())
    assert c["robot.host"]["ok"] is False and "UR_HOST" in c["robot.host"]["detail"]


def test_synthetic_stream_passes():
    rep = Report()
    check_stream(rep, SyntheticRgbdCamera(width=64, height=48, fps=0), frames=3)
    (c,) = rep.checks
    assert c.ok is True and "3 frames" in c.detail and c.data["depth"]["valid_fraction"] > 0.5


class BlackCamera(SyntheticRgbdCamera):
    def read(self):
        f = super().read()
        from perceptronics.frame import Frame

        return type(f)(
            color=Frame(f.color.width, f.color.height, bytes(len(f.color.data)), f.color.channels),
            depth=f.depth,
            intrinsics=f.intrinsics,
        )


def test_black_colour_is_the_resolution_mismatch_symptom():
    rep = Report()
    check_stream(rep, BlackCamera(width=64, height=48, fps=0), frames=2)
    (c,) = rep.checks
    assert c.ok is False and "black" in c.fix and c.severity == "critical"


class FailingCamera(SyntheticRgbdCamera):
    def open(self):
        from perceptronics.realsense import RealSenseError

        raise RealSenseError("failed to set power state")


def test_stream_failure_carries_the_platform_hint():
    rep = Report()
    check_stream(rep, FailingCamera(width=8, height=8, fps=0))
    (c,) = rep.checks
    assert c.ok is False and "power state" in c.detail and c.fix


def _fake_robot(listener, monkeypatch, **fake_kw):
    """A Robot over the FakeController (see tests/test_urctl.py), reachable on `listener`."""
    cfg = RobotConfig(host="127.0.0.1", dashboard_port=listener, primary_port=listener, rtde_port=listener)
    FakeController(**fake_kw).install(monkeypatch)
    return cfg, Robot(cfg)


def test_running_remote_robot_is_ready(listener, monkeypatch):
    cfg, robot = _fake_robot(listener, monkeypatch)
    rep = run_doctor(
        robot_config=cfg, camera=False, robot_factory=lambda _c: robot, env={"UR_CELL": "sim"}
    ).as_dict()
    c = _by_name(rep)
    assert c["robot.reach"]["ok"] and c["robot.primary"]["ok"] and c["robot.rtde"]["ok"]
    assert c["robot.mode"]["ok"] and c["robot.safety"]["ok"] and c["robot.control"]["ok"]
    assert c["handeye"]["ok"] is None and not c["handeye"]["data"]["calibrated"]
    assert c["robot.tcp"]["ok"] is True
    assert rep["motion_ok"] is True


def test_protective_stop_and_local_mode_gate_motion_only(listener, monkeypatch):
    cfg, robot = _fake_robot(listener, monkeypatch, safety_mode="PROTECTIVE_STOP", remote=False)
    rep = run_doctor(robot_config=cfg, camera=False, robot_factory=lambda _c: robot, env={}).as_dict()
    c = _by_name(rep)
    assert c["robot.safety"]["ok"] is False and "unlock" in c["robot.safety"]["fix"]
    assert c["robot.control"]["ok"] is False and "Remote" in c["robot.control"]["fix"]
    assert rep["ok"] is True and rep["motion_ok"] is False  # state works, motion gated


def test_bracket_model_mismatch_is_flagged(listener, monkeypatch):
    cfg, robot = _fake_robot(listener, monkeypatch)
    env = {"UR_ROBOT_MODEL": "UR20", "PERCEPTRONICS_BRACKET": "eseries"}
    c = _by_name(
        run_doctor(robot_config=cfg, camera=False, robot_factory=lambda _c: robot, env=env).as_dict()
    )
    assert c["handeye.bracket"]["ok"] is False and "PERCEPTRONICS_BRACKET=ur20" in c["handeye.bracket"]["fix"]


def test_a_ufactory_850_expects_the_uf850_print(listener, monkeypatch):
    cfg, robot = _fake_robot(listener, monkeypatch)
    env = {"UR_ROBOT_MODEL": "UF850", "PERCEPTRONICS_BRACKET": "eseries"}
    c = _by_name(
        run_doctor(robot_config=cfg, camera=False, robot_factory=lambda _c: robot, env=env).as_dict()
    )
    assert (
        c["handeye.bracket"]["ok"] is False and "PERCEPTRONICS_BRACKET=uf850" in c["handeye.bracket"]["fix"]
    )
    env["PERCEPTRONICS_BRACKET"] = "uf850"
    c = _by_name(
        run_doctor(robot_config=cfg, camera=False, robot_factory=lambda _c: robot, env=env).as_dict()
    )
    assert "handeye.bracket" not in c


def test_render_lists_fixes_under_failures():
    cfg = RobotConfig(
        host="127.0.0.1", platform="polyscopex", robot_api_port=_closed_port(), dashboard_port=_closed_port()
    )
    text = run_doctor(robot_config=cfg, camera=False, env={"UR_CELL": "sim"}).render()
    assert "[FAIL] robot.reach" in text and "fix:" in text and text.endswith("verdict: NOT READY")


def test_model_check_reports_the_reach_cap_and_catches_a_mismatch(listener, monkeypatch):
    # Nothing configured: the controller's answer sizes the cap.
    cfg, robot = _fake_robot(listener, monkeypatch)
    c = _by_name(run_doctor(robot_config=cfg, camera=False, robot_factory=lambda _c: robot).as_dict())
    assert c["robot.model"]["ok"] is True and c["robot.model"]["data"]["reported"] == "UR10"
    assert c["robot.model"]["data"]["max_reach_m"] == 1.3 and "1.30 m" in c["robot.model"]["detail"]
    # The cell file names a UR3e but the arm on the network says UR10: motion is gated.
    cfg = RobotConfig(
        host="127.0.0.1",
        dashboard_port=listener,
        primary_port=listener,
        rtde_port=listener,
        robot_model="UR3e",
    )
    FakeController().install(monkeypatch)
    rep = run_doctor(robot_config=cfg, camera=False, robot_factory=lambda _c: Robot(cfg)).as_dict()
    c = _by_name(rep)
    assert c["robot.model"]["ok"] is False and "UR_ROBOT_MODEL" in c["robot.model"]["fix"]
    assert c["robot.model"]["data"] == {"configured": "UR3E", "reported": "UR10", "max_reach_m": 0.5}
    assert rep["motion_ok"] is False


def test_approach_check_names_the_tool_offset_every_move_runs_with():
    from perceptronics.doctor import approach_check

    ok = approach_check([0, 0, 0.163, 0, 0, 0], {})  # default: fingertips, Hand-E length
    assert ok.ok is True and "163 mm" in ok.detail and ok.data["matches_active_tcp"] is True
    ur3e = approach_check(
        [0.000256, -0.0352, 0.2204, 0.2572, -0.4104, 1.4319], {"PERCEPTRONICS_TIP_M": "0.163"}
    )
    assert ur3e.ok is True and "differs and is overridden" in ur3e.detail
    flange = approach_check([0] * 6, {"PERCEPTRONICS_APPROACH_REFERENCE": "flange"})
    assert flange.ok is None and flange.severity == "warn" and "fingertip" in flange.fix
    bad = approach_check([0] * 6, {"PERCEPTRONICS_TIP_M": "-1"})
    assert bad.ok is False and bad.severity == "critical"


# ----- the camera a running cockpit owns (first Pi deploy, 2026-10-02) ----------------


@pytest.fixture
def cockpit_info():
    """A real HTTP server answering ``/api/info`` with whatever the test puts in
    ``info`` — the running cockpit, as the doctor sees it."""
    import json
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    info: dict = {}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            body = json.dumps(info).encode()
            self.send_response(200 if self.path == "/api/info" else 404)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield info, f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()


D435 = {"name": "RealSense D435", "serial": "832112072004", "firmware": "5.12.7.100", "usb_type": "3.2"}


def _camera_is_busy(monkeypatch, wait_s: float = 0.0):
    """What librealsense answers a second process while the cockpit streams."""
    from perceptronics import doctor, realsense

    monkeypatch.setattr(doctor, "COCKPIT_OPEN_WAIT_S", wait_s)

    def busy(library=None):
        raise realsense.RealSenseError("rs2_create_device(info_list, index:0): failed to set power state")

    monkeypatch.setattr(doctor, "check_sdk", lambda report, library=None: True)
    monkeypatch.setattr(realsense, "list_devices", busy)


def test_camera_held_by_the_running_cockpit_is_not_a_failure(cockpit_info, monkeypatch):
    info, url = cockpit_info
    info.update(camera={"kind": "realsense", "open": True, "device": D435}, fps=29.97, seq=480)
    _camera_is_busy(monkeypatch)
    report = run_doctor(robot=False, cockpit_url=url, env={})
    cam = _by_name(report.as_dict())["camera"]
    assert cam["ok"] is True
    assert "832112072004" in cam["detail"] and "usb 3.2" in cam["detail"] and "cockpit" in cam["detail"]
    assert report.ok


def test_cockpit_camera_on_usb2_still_warns(cockpit_info, monkeypatch):
    info, url = cockpit_info
    info.update(camera={"kind": "realsense", "open": True, "device": {**D435, "usb_type": "2.1"}})
    _camera_is_busy(monkeypatch)
    cam = _by_name(run_doctor(robot=False, cockpit_url=url, env={}).as_dict())["camera"]
    assert cam["ok"] is False and cam["severity"] == "warn" and "USB 2" in cam["fix"]


@pytest.mark.parametrize(
    "camera",
    [
        {"kind": "realsense", "open": False, "device": D435},  # the cockpit lost it
        {"kind": "realsense", "open": True, "device": None},
        {"kind": "synthetic", "open": True, "device": D435},  # a --fake cockpit owns no camera
        {},
    ],
)
def test_camera_the_cockpit_does_not_hold_is_still_checked(cockpit_info, monkeypatch, camera):
    info, url = cockpit_info
    info.update(camera=camera)
    _camera_is_busy(monkeypatch)
    report = run_doctor(robot=False, cockpit_url=url, env={})
    cam = _by_name(report.as_dict())["camera"]
    assert cam["ok"] is False and "failed to set power state" in cam["detail"]
    assert not report.ok


def test_camera_is_checked_directly_when_no_cockpit_answers(monkeypatch):
    _camera_is_busy(monkeypatch)
    report = run_doctor(robot=False, cockpit_url=f"http://127.0.0.1:{_closed_port()}", env={})
    assert _by_name(report.as_dict())["camera"]["ok"] is False


def test_doctor_waits_for_a_cockpit_that_is_still_opening_the_camera(cockpit_info, monkeypatch):
    """deploy-pi.sh runs the doctor the second the service restarts: the cockpit
    answers, its camera isn't open yet, and the device is already claimed."""
    info, url = cockpit_info
    info.update(camera={"kind": "realsense", "open": False, "device": None}, fps=0.0, seq=0)
    _camera_is_busy(monkeypatch, wait_s=10.0)

    def opens():
        info.update(camera={"kind": "realsense", "open": True, "device": D435}, fps=29.9, seq=12)

    timer = threading.Timer(0.4, opens)
    timer.start()
    try:
        checks = _by_name(run_doctor(robot=False, cockpit_url=url, env={}).as_dict())
    finally:
        timer.cancel()
    assert checks["camera"]["ok"] is True
    assert "seq 12" in checks["cockpit"]["detail"]
