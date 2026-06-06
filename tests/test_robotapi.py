"""Unit tests for the PolyScope X Robot-API client — no simulator required.

The HTTP layer is faked at the ``urctl.robotapi._http_json`` seam (the REST
analogue of the ``urctl.transport`` seam used in ``test_urctl.py``), so these
exercise the full PolyScope X path — config, RobotAPIClient, and the Robot
facade's platform switch — without a network.
"""

from __future__ import annotations

import pytest

from urctl import Robot, RobotConfig, robotapi
from urctl.robotapi import RobotAPIClient

# ----- a fake Robot-API at the HTTP seam -------------------------------------


class FakeRobotAPI:
    """Records requests and returns canned ``(status, json)`` responses.

    Install with ``.install(monkeypatch)``; it patches ``robotapi._http_json``.
    State getters reflect ``robot_mode`` / ``control_mode`` so a power-on flips
    them, letting bring-up-style round-trips resolve.
    """

    def __init__(
        self,
        *,
        robot_mode: str = "POWER_OFF",
        safety_mode: str = "NORMAL",
        program_state: str = "STOPPED",
        control_mode: str = "REMOTE",
        forbid: bool = False,
    ):
        self.robot_mode = robot_mode
        self.safety_mode = safety_mode
        self.program_state = program_state
        self.control_mode = control_mode
        self.forbid = forbid  # simulate Local-mode 403 on every PUT
        self.requests: list[tuple[str, str, dict | None]] = []

    def install(self, monkeypatch) -> FakeRobotAPI:
        monkeypatch.setattr(robotapi, "_http_json", self._http_json)
        return self

    def _http_json(self, method, url, *, body=None, timeout=10.0):
        self.requests.append((method, url, body))
        if method == "GET":
            if url.endswith("/robotstate/v1/robotmode"):
                return 200, {"mode": self.robot_mode, "message": "ok"}
            if url.endswith("/robotstate/v1/safetymode"):
                return 200, {"mode": self.safety_mode, "message": "ok"}
            if url.endswith("/program/v1/state"):
                return 200, {"state": self.program_state, "message": "ok"}
            if url.endswith("/system/v1/controlmode"):
                return 200, {"mode": self.control_mode, "message": "ok"}
            if url.endswith("/system/v1/operationalmode"):
                return 200, {"mode": "AUTOMATIC", "message": "ok"}
            return 404, {"message": "not found"}
        # PUT (mutating)
        if self.forbid:
            return 403, {
                "message": "Forbidden - Operation Not Allowed",
                "details": "The robot must be in remote mode ...",
            }
        if url.endswith("/robotstate/v1/state"):
            action = (body or {}).get("action")
            if action == "POWER_ON":
                self.robot_mode = "IDLE"
            elif action == "BRAKE_RELEASE":
                self.robot_mode = "RUNNING"
            elif action == "POWER_OFF":
                self.robot_mode = "POWER_OFF"
            return 200, {"message": "ok"}
        if url.endswith("/program/v1/loaded"):
            return 200, {"message": "ok"}
        if url.endswith("/program/v1/state"):
            return 200, {"message": "ok"}
        return 400, {"message": "bad request"}


@pytest.fixture
def px_cfg() -> RobotConfig:
    return RobotConfig(host="localhost", platform="polyscopex", robot_api_port=8000)


# ----- config ----------------------------------------------------------------


class TestConfig:
    def test_robot_api_url_built_from_host_port_path(self, px_cfg):
        assert px_cfg.robot_api_url == "http://localhost:8000/universal-robots/robot-api"

    def test_is_polyscopex(self, px_cfg):
        assert px_cfg.is_polyscopex()
        assert not RobotConfig().is_polyscopex()  # default is e-series

    def test_from_env_reads_platform(self, monkeypatch):
        monkeypatch.setenv("UR_PLATFORM", "polyscopex")
        monkeypatch.setenv("UR_ROBOT_API_PORT", "8000")
        cfg = RobotConfig.from_env()
        assert cfg.platform == "polyscopex"
        assert cfg.robot_api_port == 8000


# ----- RobotAPIClient endpoint mapping ---------------------------------------


class TestRobotAPIClient:
    def test_mode_getters_parse_the_mode_field(self, monkeypatch, px_cfg):
        fake = FakeRobotAPI(robot_mode="RUNNING", safety_mode="NORMAL").install(monkeypatch)
        c = RobotAPIClient(px_cfg)
        assert c.robot_mode() == "RUNNING"
        assert c.safety_mode() == "NORMAL"
        assert c.program_state() == "STOPPED"
        assert c.control_mode() == "REMOTE"
        assert c.is_running() is True
        assert c.is_remote_control() is True
        assert all(m == "GET" for m, _, _ in fake.requests)

    def test_power_on_puts_the_right_action(self, monkeypatch, px_cfg):
        fake = FakeRobotAPI().install(monkeypatch)
        RobotAPIClient(px_cfg).power_on()
        method, url, body = fake.requests[-1]
        assert method == "PUT"
        assert url.endswith("/robotstate/v1/state")
        assert body == {"action": "POWER_ON"}

    def test_brake_release_and_unlock_pstop_actions(self, monkeypatch, px_cfg):
        fake = FakeRobotAPI().install(monkeypatch)
        c = RobotAPIClient(px_cfg)
        c.brake_release()
        assert fake.requests[-1][2] == {"action": "BRAKE_RELEASE"}
        c.unlock_protective_stop()
        assert fake.requests[-1][2] == {"action": "UNLOCK_PROTECTIVE_STOP"}

    def test_load_strips_urp_suffix_and_hits_loaded_endpoint(self, monkeypatch, px_cfg):
        fake = FakeRobotAPI().install(monkeypatch)
        reply = RobotAPIClient(px_cfg).load("MotionDemo.urp")
        method, url, body = fake.requests[-1]
        assert method == "PUT" and url.endswith("/program/v1/loaded")
        assert body == {"name": "MotionDemo"}
        # Return string carries the substring Robot.load_program checks for.
        assert "Loading program" in reply

    def test_play_stop_pause_program_actions(self, monkeypatch, px_cfg):
        fake = FakeRobotAPI().install(monkeypatch)
        c = RobotAPIClient(px_cfg)
        assert "Starting program" in c.play()
        assert fake.requests[-1] == ("PUT", px_cfg.robot_api_url + "/program/v1/state", {"action": "play"})
        c.stop()
        assert fake.requests[-1][2] == {"action": "stop"}
        c.pause()
        assert fake.requests[-1][2] == {"action": "pause"}

    def test_403_becomes_actionable_remote_mode_message(self, monkeypatch, px_cfg):
        FakeRobotAPI(forbid=True).install(monkeypatch)
        reply = RobotAPIClient(px_cfg).power_on()
        assert "403" in reply
        assert "Remote" in reply
        # play() ok-detection must fail on a 403 (no "Starting program").
        assert "Starting program" not in RobotAPIClient(px_cfg).play()

    def test_wait_for_polls_until_predicate(self, monkeypatch, px_cfg):
        FakeRobotAPI(robot_mode="IDLE").install(monkeypatch)
        last = RobotAPIClient(px_cfg).wait_for(lambda r: "IDLE" in r, timeout=2.0, interval=0.01)
        assert "IDLE" in last

    def test_command_and_popup_report_unsupported(self, monkeypatch, px_cfg):
        FakeRobotAPI().install(monkeypatch)
        c = RobotAPIClient(px_cfg)
        assert "not available" in c.command("robotmode").lower()
        assert "not available" in c.popup("hi").lower()


# ----- Robot facade platform switch ------------------------------------------


class TestRobotPlatformSwitch:
    def test_polyscopex_config_wires_robotapi_client(self, px_cfg):
        assert isinstance(Robot(px_cfg).dashboard, RobotAPIClient)

    def test_eseries_config_wires_dashboard_client(self):
        from urctl.dashboard import DashboardClient

        assert isinstance(Robot(RobotConfig()).dashboard, DashboardClient)

    def test_get_state_includes_control_mode(self, monkeypatch, px_cfg):
        FakeRobotAPI(robot_mode="POWER_OFF", control_mode="LOCAL").install(monkeypatch)
        result = Robot(px_cfg).get_state()
        assert result["control_mode"] == "LOCAL"
        assert result["robot_mode"] == "POWER_OFF"
        assert result["running"] is False

    def test_bring_up_blocked_in_local_mode(self, monkeypatch, px_cfg):
        FakeRobotAPI(control_mode="LOCAL").install(monkeypatch)
        result = Robot(px_cfg).bring_up()
        assert result["ok"] is False
        assert "Local" in result["reply"]

    def test_bring_up_succeeds_in_remote_mode(self, monkeypatch, px_cfg):
        FakeRobotAPI(control_mode="REMOTE", robot_mode="POWER_OFF").install(monkeypatch)
        result = Robot(px_cfg).bring_up()
        # power_on -> IDLE, brake_release -> RUNNING (driven by the fake).
        assert result["ok"] is True
        assert "RUNNING" in result["robot_mode"]


# ----- live integration (read-only) ------------------------------------------

_PX_INTEGRATION_PORT = 8000


def _px_reachable() -> bool:
    import urllib.error
    import urllib.request

    url = f"http://localhost:{_PX_INTEGRATION_PORT}/universal-robots/robot-api/robotstate/v1/robotmode"
    try:
        with urllib.request.urlopen(url, timeout=3) as resp:
            return resp.status == 200
    except (urllib.error.URLError, OSError):
        return False


@pytest.mark.integration
@pytest.mark.skipif(not _px_reachable(), reason="PolyScope X sim not reachable on :8000")
class TestRobotAPILive:
    """Hits the running PolyScope X sim. Read-only — never mutates the robot."""

    def _client(self) -> RobotAPIClient:
        return RobotAPIClient(
            RobotConfig(host="localhost", platform="polyscopex", robot_api_port=_PX_INTEGRATION_PORT)
        )

    def test_robot_mode_is_a_known_state(self):
        mode = self._client().robot_mode()
        assert mode in (
            "POWER_OFF", "BOOTING", "IDLE", "RUNNING", "POWER_ON", "BACKDRIVE", "CONFIRM_SAFETY"
        ), f"unexpected robotmode: {mode!r}"

    def test_safety_and_control_mode_readable(self):
        c = self._client()
        assert c.safety_mode()  # non-empty (e.g. NORMAL)
        assert c.control_mode() in ("LOCAL", "REMOTE")
