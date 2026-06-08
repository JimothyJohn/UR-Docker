"""Unit tests for the urctl package — no simulator or network required.

The controller is faked at the transport seam (``urctl.transport``), so these
tests run anywhere and exercise the full stack: config, clients, safety
envelope, audit log, the Robot facade, and the agent tool registry.
"""

from __future__ import annotations

import json
import re
import struct

import pytest

from urctl import Robot, RobotConfig, RtdeClient, SafetyEnvelope
from urctl import rtde as urctl_rtde
from urctl import tools as urctl_tools
from urctl.audit import AuditLog
from urctl.cli import main as cli_main
from urctl.rtde import RtdeError
from urctl.safety import SafetyError

# ----- a fake controller at the transport seam -------------------------------

_VEC_RX = re.compile(r"\[([^\]]+)\]")


def _no_rtde(self):
    """Stub for RtdeClient.connect — keeps unit tests off the network so
    get_state() deterministically uses its Primary textmsg fallback."""
    raise ConnectionRefusedError("RTDE disabled in unit tests")


class FakeController:
    """Records what was sent and returns canned replies.

    Install with ``fake.install(monkeypatch)``; it patches the three functions
    in ``urctl.transport`` that every client funnels through.
    """

    def __init__(
        self,
        robot_mode: str = "RUNNING",
        safety_mode: str = "NORMAL",
        confirm_answer: str | None = "yes",
    ):
        self.robot_mode = robot_mode
        self.safety_mode = safety_mode
        # What the "operator" answers a pendant confirm dialog with: "yes",
        # "no", or None (no answer — simulates a timeout).
        self.confirm_answer = confirm_answer
        self.dashboard_sends: list[str] = []
        self.primary_sends: list[str] = []

    def install(self, monkeypatch) -> FakeController:
        from urctl import transport

        monkeypatch.setattr(transport, "request_until_close", self._dashboard)
        monkeypatch.setattr(transport, "send", self._primary_send)
        monkeypatch.setattr(transport, "send_and_collect", self._primary_collect)
        # No RTDE in unit tests: force get_state() onto its textmsg fallback.
        monkeypatch.setattr(urctl_rtde.RtdeClient, "connect", _no_rtde)
        return self

    def _dashboard(self, host, port, payload, timeout=10.0) -> bytes:
        text = payload.decode()
        self.dashboard_sends.append(text)
        lines = []
        for cmd in text.splitlines():
            cmd = cmd.strip()
            if cmd in ("", "quit"):
                continue
            if cmd == "robotmode":
                lines.append(f"Robotmode: {self.robot_mode}")
            elif cmd == "safetymode":
                lines.append(f"Safetymode: {self.safety_mode}")
            elif cmd == "programState":
                lines.append("STOPPED MotionDemo.urp")
            elif cmd.startswith("load "):
                lines.append(f"Loading program: /programs/{cmd[5:]}")
            elif cmd == "play":
                lines.append("Starting program")
            else:
                lines.append("ack")
        return ("Connected: fake\n" + "\n".join(lines) + "\n").encode()

    def _primary_send(self, host, port, payload, timeout=5.0) -> None:
        self.primary_sends.append(payload.decode())

    def _primary_collect(
        self, host, port, payload, *, collect_for=2.0, timeout=5.0, stop_marker=None
    ) -> bytes:
        body = payload.decode()
        self.primary_sends.append(body)
        # A pendant confirm round-trip: echo the marker the "operator" chose
        # (or nothing, to simulate no answer / timeout).
        if "request_boolean_from_primary_client" in body:
            if self.confirm_answer is None:
                return b""
            # A freedrive reteach uses its own markers and echoes the achieved
            # joint pose on OK; a plain confirm uses urctl/confirm=.
            if "freedrive_mode" in body:
                if self.confirm_answer == "yes":
                    return b"urctl/reteach/pose=[1,2,3,4,5,6]\n"
                return b"urctl/reteach=cancel\n"
            return f"urctl/confirm={self.confirm_answer}\n".encode()
        # Echo the first bracketed vector back as the "done"/state marker so
        # move/read round-trips parse successfully.
        m = _VEC_RX.search(body)
        vec = m.group(1) if m else "0,0,0,0,0,0"
        echo = f"urctl/move/done=[{vec}]\nurctl/state/joints=[{vec}]\nurctl/state/tcp=[0,0,0,0,0,0]\n"
        return echo.encode()


@pytest.fixture
def fake(monkeypatch) -> FakeController:
    return FakeController().install(monkeypatch)


# ----- config ----------------------------------------------------------------


class TestConfig:
    def test_defaults_are_localhost(self):
        cfg = RobotConfig()
        assert cfg.host == "localhost"
        assert cfg.dashboard_port == 29999
        assert cfg.primary_port == 30001
        assert cfg.is_loopback()

    def test_from_env_reads_environment(self, monkeypatch):
        monkeypatch.setenv("UR_HOST", "10.0.0.5")
        monkeypatch.setenv("UR_DASH_PORT", "12345")
        cfg = RobotConfig.from_env()
        assert cfg.host == "10.0.0.5"
        assert cfg.dashboard_port == 12345
        assert not cfg.is_loopback()

    def test_explicit_host_overrides_env(self, monkeypatch):
        monkeypatch.setenv("UR_HOST", "10.0.0.5")
        cfg = RobotConfig.from_env(host="192.168.1.9")
        assert cfg.host == "192.168.1.9"

    def test_bad_port_raises_clear_error(self, monkeypatch):
        monkeypatch.setenv("UR_DASH_PORT", "not-a-port")
        with pytest.raises(ValueError, match="UR_DASH_PORT"):
            RobotConfig.from_env()


# ----- safety envelope -------------------------------------------------------


class TestSafety:
    def test_valid_move_passes(self):
        env = SafetyEnvelope()
        v = env.validate_move_joints(
            [0, -1.57, 0, -1.57, 0, 0], velocity=0.5, acceleration=0.8, robot_mode="RUNNING"
        )
        assert v.ok and not v.violations

    def test_joint_out_of_range_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_joints([99, 0, 0, 0, 0, 0], velocity=0.5, acceleration=0.8)
        assert not v.ok
        assert any(viol.rule == "joint_range" for viol in v.violations)

    def test_wrong_joint_count_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_joints([0, 0, 0], velocity=0.5, acceleration=0.8)
        assert not v.ok
        assert any(viol.rule == "joint_count" for viol in v.violations)

    def test_overspeed_rejected(self):
        env = SafetyEnvelope(max_joint_speed=1.0)
        v = env.validate_move_joints([0] * 6, velocity=5.0, acceleration=0.8)
        assert not v.ok
        assert any(viol.rule == "velocity" for viol in v.violations)

    def test_nan_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_joints([float("nan"), 0, 0, 0, 0, 0], velocity=0.5, acceleration=0.8)
        assert not v.ok

    def test_not_running_rejected_when_required(self):
        env = SafetyEnvelope()
        v = env.validate_move_joints(
            [0] * 6, velocity=0.5, acceleration=0.8, robot_mode="Robotmode: POWER_OFF"
        )
        assert not v.ok
        assert any(viol.rule == "robot_state" for viol in v.violations)

    def test_verdict_raises(self):
        env = SafetyEnvelope()
        v = env.validate_move_joints([99] * 6, velocity=0.5, acceleration=0.8)
        with pytest.raises(SafetyError):
            v.raise_if_unsafe()

    def test_speed_override_in_range_passes(self):
        env = SafetyEnvelope()
        assert env.validate_speed_override(0.5).ok

    def test_speed_override_above_max_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_speed_override(1.5)
        assert not v.ok
        assert any(viol.rule == "speed_override" for viol in v.violations)

    def test_speed_override_zero_rejected(self):
        env = SafetyEnvelope()
        assert not env.validate_speed_override(0.0).ok

    def test_speed_override_custom_cap(self):
        env = SafetyEnvelope(max_speed_fraction=0.3)
        assert env.validate_speed_override(0.3).ok
        assert not env.validate_speed_override(0.5).ok


# ----- safety envelope: Cartesian / TCP moves --------------------------------


class TestSafetyTcp:
    def test_valid_relative_move_passes(self):
        env = SafetyEnvelope()
        v = env.validate_move_tcp(
            [0, 0.05, 0, 0, 0, 0],
            velocity=0.25,
            acceleration=1.2,
            relative=True,
            robot_mode="RUNNING",
        )
        assert v.ok and not v.violations

    def test_valid_absolute_move_passes(self):
        env = SafetyEnvelope()
        v = env.validate_move_tcp(
            [0.3, -0.4, 0.3, 0, 3.14, 0],
            velocity=0.25,
            acceleration=1.2,
            relative=False,
            robot_mode="RUNNING",
        )
        assert v.ok and not v.violations

    def test_wrong_pose_count_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_tcp([0, 0, 0], velocity=0.25, acceleration=1.2)
        assert not v.ok
        assert any(viol.rule == "pose_count" for viol in v.violations)

    def test_nan_pose_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_tcp([float("nan"), 0, 0, 0, 0, 0], velocity=0.25, acceleration=1.2)
        assert not v.ok
        assert any(viol.rule == "pose_value" for viol in v.violations)

    def test_tcp_overspeed_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_tcp([0, 0.05, 0, 0, 0, 0], velocity=5.0, acceleration=1.2, relative=True)
        assert not v.ok
        assert any(viol.rule == "tcp_velocity" for viol in v.violations)

    def test_tcp_over_accel_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_tcp([0, 0.05, 0, 0, 0, 0], velocity=0.25, acceleration=99.0, relative=True)
        assert not v.ok
        assert any(viol.rule == "tcp_acceleration" for viol in v.violations)

    def test_absolute_target_beyond_reach_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_tcp([2.0, 0, 0, 0, 0, 0], velocity=0.25, acceleration=1.2, relative=False)
        assert not v.ok
        assert any(viol.rule == "tcp_reach" for viol in v.violations)

    def test_relative_step_unit_error_rejected(self):
        # 5 "metres" relative step is almost certainly 5 inches mis-entered.
        env = SafetyEnvelope()
        v = env.validate_move_tcp([0, 5.0, 0, 0, 0, 0], velocity=0.25, acceleration=1.2, relative=True)
        assert not v.ok
        assert any(viol.rule == "tcp_step" for viol in v.violations)

    def test_large_relative_step_allowed_as_absolute(self):
        # The same magnitude that's rejected as a relative *step* is fine as an
        # absolute target within reach — the caps are distinct.
        env = SafetyEnvelope()
        v = env.validate_move_tcp([0, 1.1, 0, 0, 0, 0], velocity=0.25, acceleration=1.2, relative=False)
        assert v.ok, v.violations

    def test_tcp_not_running_rejected(self):
        env = SafetyEnvelope()
        v = env.validate_move_tcp(
            [0, 0.05, 0, 0, 0, 0],
            velocity=0.25,
            acceleration=1.2,
            relative=True,
            robot_mode="Robotmode: POWER_OFF",
        )
        assert not v.ok
        assert any(viol.rule == "robot_state" for viol in v.violations)


# ----- audit log -------------------------------------------------------------


class TestAudit:
    def test_in_memory_capture(self):
        log = AuditLog(path=None)
        log.record("popup", host="h", args={"text": "hi"}, ok=True)
        assert len(log.records) == 1
        assert log.records[0].action == "popup"

    def test_writes_jsonl_file(self, tmp_path):
        path = tmp_path / "audit.jsonl"
        log = AuditLog(path=str(path))
        log.record("move", host="h", args={"j": [0]}, ok=True, result={"landed": [0]})
        log.record("popup", host="h", args={"text": "x"}, ok=False)
        lines = path.read_text().strip().splitlines()
        assert len(lines) == 2
        rec = json.loads(lines[0])
        assert rec["action"] == "move" and rec["ok"] is True and "ts" in rec


# ----- Robot facade ----------------------------------------------------------


class TestRobot:
    def test_get_state_normalizes_observation(self, fake):
        robot = Robot(RobotConfig())
        state = robot.get_state()
        assert state["ok"]
        assert "RUNNING" in state["robot_mode"]
        assert state["running"] is True
        assert state["joints"] is not None

    def test_move_joints_valid_sends_and_audits(self, fake):
        robot = Robot(RobotConfig())
        target = [0, -1.5708, 0, -1.5708, 0, 0]
        result = robot.move_joints(target)
        assert result["ok"]
        assert any("movej" in s for s in fake.primary_sends)
        assert robot.audit.records[-1].action == "move_joints"

    def test_move_joints_unsafe_does_not_send(self, fake):
        robot = Robot(RobotConfig())
        result = robot.move_joints([99, 0, 0, 0, 0, 0])
        assert result["ok"] is False
        assert "safety" in result
        # Nothing should have been streamed to the Primary client.
        assert not any("movej" in s for s in fake.primary_sends)

    def test_move_joints_blocked_when_not_running(self, monkeypatch):
        FakeController(robot_mode="POWER_OFF").install(monkeypatch)
        robot = Robot(RobotConfig())
        result = robot.move_joints([0, -1.57, 0, -1.57, 0, 0])
        assert result["ok"] is False

    def test_dry_run_validates_but_does_not_send(self, fake):
        robot = Robot(RobotConfig(), dry_run=True)
        result = robot.move_joints([0, -1.57, 0, -1.57, 0, 0])
        assert result["ok"] and result["dry_run"]
        assert not fake.primary_sends  # nothing went out

    # ----- joint-space trajectories (streamed, one program) -----------------

    def test_move_trajectory_sends_one_program_with_all_waypoints(self, fake):
        robot = Robot(RobotConfig())
        a = [0.0, -1.0, 1.2, -1.8, -1.5708, 0.0]
        b = [0.3, -1.0, 1.2, -1.8, -1.5708, 0.5]
        # Loop back to A so the fake (which echoes the *first* bracketed vector as
        # the done marker) reports landing on the final waypoint => ok.
        result = robot.move_trajectory([a, b, a], velocity=1.0, acceleration=1.5)
        assert result["ok"], result
        # The whole path is a single Primary submission (one def, not N programs).
        assert len(fake.primary_sends) == 1
        sent = fake.primary_sends[0]
        assert "def urctl_trajectory" in sent
        assert sent.count("movej(") == 3
        assert robot.audit.records[-1].action == "move_trajectory"

    def test_move_trajectory_rejects_unsafe_waypoint_and_sends_nothing(self, fake):
        robot = Robot(RobotConfig())
        good = [0.0, -1.0, 1.2, -1.8, -1.5708, 0.0]
        bad = [99.0, 0, 0, 0, 0, 0]  # joint 0 out of range
        result = robot.move_trajectory([good, bad, good])
        assert result["ok"] is False
        # The violation points at the offending waypoint, and nothing is streamed.
        assert result["safety"]["waypoint"] == 1
        assert not fake.primary_sends

    def test_move_trajectory_blend_dropped_on_final_waypoint(self, fake):
        robot = Robot(RobotConfig())
        a = [0.0, -1.0, 1.2, -1.8, -1.5708, 0.0]
        b = [0.3, -1.0, 1.2, -1.8, -1.5708, 0.5]
        robot.move_trajectory([a, b, a], velocity=1.0, acceleration=1.5, blend_radius=0.05)
        movej_lines = [ln for ln in fake.primary_sends[0].splitlines() if "movej(" in ln]
        # Intermediate segments blend (r=...); the last comes to rest (no r=) so
        # the robot doesn't error on a blend radius it can't satisfy.
        assert "r=0.05" in movej_lines[0]
        assert "r=" not in movej_lines[-1]

    def test_move_trajectory_dry_run_sends_nothing(self, fake):
        robot = Robot(RobotConfig(), dry_run=True)
        a = [0.0, -1.0, 1.2, -1.8, -1.5708, 0.0]
        result = robot.move_trajectory([a, a], velocity=1.0)
        assert result["ok"] and result["dry_run"]
        assert not fake.primary_sends

    def test_move_trajectory_empty_is_rejected(self, fake):
        robot = Robot(RobotConfig())
        result = robot.move_trajectory([])
        assert result["ok"] is False
        assert not fake.primary_sends

    def test_bring_up_runs_sequence(self, fake):
        robot = Robot(RobotConfig())
        result = robot.bring_up()
        assert result["ok"]
        joined = " ".join(fake.dashboard_sends)
        assert "power on" in joined and "brake release" in joined

    # ----- Cartesian / TCP moves --------------------------------------------

    def test_move_tcp_relative_sends_wrapped_movel(self, fake):
        robot = Robot(RobotConfig())
        result = robot.move_tcp([0, 0.05, 0, 0, 0, 0], relative=True)
        assert result["ok"]
        sent = " ".join(fake.primary_sends)
        # Wrapped in a def (the fix for bare top-level motion silently no-op'ing),
        # uses movel, and computes the target relative to the live pose.
        assert "def urctl_move_tcp" in sent
        assert "movel" in sent
        assert "pose_add(get_actual_tcp_pose()" in sent
        assert robot.audit.records[-1].action == "move_tcp"

    def test_move_tcp_absolute_sends_literal_pose(self, fake):
        robot = Robot(RobotConfig())
        result = robot.move_tcp([0.3, -0.4, 0.3, 0, 3.14, 0], relative=False)
        assert result["ok"]
        sent = " ".join(fake.primary_sends)
        assert "movel(p[0.3" in sent
        # Absolute moves do not reference the current pose.
        assert "pose_add" not in sent

    def test_move_tcp_unsafe_does_not_send(self, fake):
        robot = Robot(RobotConfig())
        # 5 m relative step trips the unit-error guard.
        result = robot.move_tcp([0, 5.0, 0, 0, 0, 0], relative=True)
        assert result["ok"] is False
        assert "safety" in result
        assert not any("movel" in s for s in fake.primary_sends)

    def test_move_tcp_blocked_when_not_running(self, monkeypatch):
        FakeController(robot_mode="POWER_OFF").install(monkeypatch)
        robot = Robot(RobotConfig())
        result = robot.move_tcp([0, 0.05, 0, 0, 0, 0], relative=True)
        assert result["ok"] is False

    def test_move_tcp_dry_run_does_not_send(self, fake):
        robot = Robot(RobotConfig(), dry_run=True)
        result = robot.move_tcp([0, 0.05, 0, 0, 0, 0], relative=True)
        assert result["ok"] and result["dry_run"]
        assert not fake.primary_sends

    def test_move_tcp_protective_stop_is_surfaced(self, monkeypatch):
        """A move that never confirms (no done marker) while the controller is
        in a protective stop reports ok=False AND result.protective_stop=True —
        so the caller sees the reason, not a silent failure. Mirrors a movel
        through a singularity tripping error C154A0 on real hardware/URSim."""

        class StoppedMidMove(FakeController):
            def _primary_collect(self, *a, **kw):
                # Run the move but emit no "urctl/move/done=" marker: the move
                # was aborted (protective stop) before it could complete.
                super()._primary_collect(*a, **kw)
                return b""

        StoppedMidMove(safety_mode="PROTECTIVE_STOP").install(monkeypatch)
        robot = Robot(RobotConfig())
        result = robot.move_tcp([0, 0.05, 0, 0, 0, 0], relative=True)
        assert result["ok"] is False
        assert result["landed"] is None
        assert result["protective_stop"] is True

    # ----- run_script wrap behaviour ----------------------------------------

    def test_run_script_wraps_by_default(self, fake):
        robot = Robot(RobotConfig())
        result = robot.run_script("movel(p[0,0.05,0,0,0,0], a=0.3, v=0.1)")
        assert result["ok"]
        sent = " ".join(fake.primary_sends)
        # Default wrap puts the snippet inside a def so motion actually runs.
        assert "def urctl_snippet" in sent and "movel" in sent

    def test_run_script_raw_sends_verbatim(self, fake):
        robot = Robot(RobotConfig())
        result = robot.run_script('textmsg("hi")', wrap=False)
        assert result["ok"]
        # Exactly what we sent — no def wrapper.
        assert fake.primary_sends == ['textmsg("hi")']


class TestConfirmOnPendant:
    def test_yes_returns_confirmed_true(self, monkeypatch):
        FakeController(confirm_answer="yes").install(monkeypatch)
        robot = Robot(RobotConfig())
        res = robot.confirm_on_pendant("Add this step?")
        assert res["ok"] and res["confirmed"] is True

    def test_no_returns_confirmed_false(self, monkeypatch):
        fake = FakeController(confirm_answer="no").install(monkeypatch)
        robot = Robot(RobotConfig())
        res = robot.confirm_on_pendant("Add this step?")
        assert res["confirmed"] is False
        # It actually raised the pendant dialog over Primary.
        assert any("request_boolean_from_primary_client" in s for s in fake.primary_sends)

    def test_no_answer_returns_none(self, monkeypatch):
        FakeController(confirm_answer=None).install(monkeypatch)
        robot = Robot(RobotConfig())
        # Short timeout so the fake's empty reply resolves quickly.
        res = robot.confirm_on_pendant("Add this step?", timeout=0.2)
        assert res["confirmed"] is None

    def test_dry_run_sends_nothing(self, monkeypatch):
        fake = FakeController().install(monkeypatch)
        robot = Robot(RobotConfig(), dry_run=True)
        res = robot.confirm_on_pendant("Add this step?")
        assert res["confirmed"] is None and res["dry_run"]
        assert fake.primary_sends == []

    def test_prompt_is_sanitized_into_a_single_line_literal(self, monkeypatch):
        fake = FakeController().install(monkeypatch)
        robot = Robot(RobotConfig())
        robot.confirm_on_pendant('Move to "home"\nthen wait')
        sent = " ".join(fake.primary_sends)
        # No embedded double-quote breaking the literal, no newline inside it.
        assert "Move to 'home' then wait" in sent

    def test_reteach_ok_returns_pose_and_wraps_in_freedrive(self, monkeypatch):
        fake = FakeController(confirm_answer="yes").install(monkeypatch)
        robot = Robot(RobotConfig())
        res = robot.reteach_in_freedrive("Position the part")
        assert res["confirmed"] is True
        assert res["joints"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        body = " ".join(fake.primary_sends)
        # The hold is genuinely bracketed by freedrive enable/disable.
        assert "freedrive_mode()" in body and "end_freedrive_mode()" in body

    def test_reteach_decline_returns_false_and_no_pose(self, monkeypatch):
        FakeController(confirm_answer="no").install(monkeypatch)
        robot = Robot(RobotConfig())
        res = robot.reteach_in_freedrive("Position the part")
        assert res["confirmed"] is False and res["joints"] is None

    def test_reteach_no_answer_clears_freedrive(self, monkeypatch):
        fake = FakeController(confirm_answer=None).install(monkeypatch)
        robot = Robot(RobotConfig())
        res = robot.reteach_in_freedrive("Position the part", timeout=0.2)
        assert res["confirmed"] is None and res["joints"] is None
        # Cleanup left the robot out of freedrive (end_freedrive_mode sent).
        assert any("end_freedrive_mode()" in s for s in fake.primary_sends)

    def test_reteach_dry_run_sends_nothing(self, monkeypatch):
        fake = FakeController().install(monkeypatch)
        robot = Robot(RobotConfig(), dry_run=True)
        res = robot.reteach_in_freedrive("Position the part")
        assert res["confirmed"] is None and res["dry_run"]
        assert fake.primary_sends == []

    def test_sanitize_prompt_helper(self):
        from urctl.robot import _sanitize_prompt

        assert _sanitize_prompt('a "b"\n c') == "a 'b' c"
        assert len(_sanitize_prompt("x" * 500)) == 200


# ----- tool registry ---------------------------------------------------------


class TestTools:
    def test_schemas_well_formed(self):
        schemas = urctl_tools.get_tool_schemas()
        names = {s["name"] for s in schemas}
        assert "ur_move_joints" in names and "ur_get_state" in names
        for s in schemas:
            assert s["input_schema"]["type"] == "object"
            assert isinstance(s["description"], str) and s["description"]

    def test_call_unknown_tool_raises(self, fake):
        robot = Robot(RobotConfig())
        with pytest.raises(urctl_tools.ToolError, match="unknown tool"):
            urctl_tools.call_tool(robot, "ur_teleport", {})

    def test_call_missing_required_arg_raises(self, fake):
        robot = Robot(RobotConfig())
        with pytest.raises(urctl_tools.ToolError, match="missing required"):
            urctl_tools.call_tool(robot, "ur_move_joints", {})

    def test_call_unexpected_arg_raises(self, fake):
        robot = Robot(RobotConfig())
        with pytest.raises(urctl_tools.ToolError, match="unexpected"):
            urctl_tools.call_tool(robot, "ur_popup", {"text": "hi", "bogus": 1})

    def test_call_wrong_type_raises(self, fake):
        robot = Robot(RobotConfig())
        with pytest.raises(urctl_tools.ToolError, match="must be string"):
            urctl_tools.call_tool(robot, "ur_popup", {"text": 123})

    def test_call_move_joints_dispatches(self, fake):
        robot = Robot(RobotConfig())
        result = urctl_tools.call_tool(
            robot, "ur_move_joints", {"joints": [0, -1.5708, 0, -1.5708, 0, 0], "velocity": 0.4}
        )
        assert result["ok"]
        assert any("movej" in s for s in fake.primary_sends)

    def test_call_get_state_dispatches(self, fake):
        robot = Robot(RobotConfig())
        result = urctl_tools.call_tool(robot, "ur_get_state", {})
        assert result["ok"] and "robot_mode" in result

    def test_move_tcp_in_schemas(self):
        names = {s["name"] for s in urctl_tools.get_tool_schemas()}
        assert "ur_move_tcp" in names

    def test_call_move_tcp_relative_dispatches(self, fake):
        robot = Robot(RobotConfig())
        result = urctl_tools.call_tool(
            robot, "ur_move_tcp", {"pose": [0, 0.05, 0, 0, 0, 0], "relative": True}
        )
        assert result["ok"]
        assert any("movel" in s for s in fake.primary_sends)

    def test_call_move_tcp_missing_pose_raises(self, fake):
        robot = Robot(RobotConfig())
        with pytest.raises(urctl_tools.ToolError, match="missing required"):
            urctl_tools.call_tool(robot, "ur_move_tcp", {"relative": True})

    def test_call_run_script_defaults_to_wrapped(self, fake):
        robot = Robot(RobotConfig())
        urctl_tools.call_tool(robot, "ur_run_script", {"script": "movel(p[0,0,0,0,0,0])"})
        assert any("def urctl_snippet" in s for s in fake.primary_sends)


# ----- RTDE client (binary protocol, faked at the socket) --------------------


class FakeSocket:
    """A minimal socket stand-in: returns ``data`` in ``chunk``-sized pieces on
    recv (to exercise partial-read reassembly) and records what was sent."""

    def __init__(self, data: bytes = b"", *, chunk: int = 4096):
        self.buf = bytearray(data)
        self.sent = bytearray()
        self.chunk = chunk

    def recv(self, n: int) -> bytes:
        take = min(n, self.chunk, len(self.buf))
        out = bytes(self.buf[:take])
        del self.buf[:take]
        return out

    def sendall(self, b: bytes) -> None:
        self.sent.extend(b)

    def settimeout(self, t) -> None:
        pass

    def close(self) -> None:
        pass


def _pack_data_package(recipe_id: int, recipe: list[tuple[str, str]], values: dict) -> bytes:
    """Build a full RTDE data ('U') frame: >HB header + recipe_id + packed fields."""
    body = bytearray([recipe_id])
    for name, tstr in recipe:
        fmt, n = urctl_rtde._RTDE_TYPES[tstr]
        v = values[name]
        body += struct.pack(fmt, *v) if n > 1 else struct.pack(fmt, v)
    size = 3 + len(body)
    return struct.pack(">HB", size, urctl_rtde._RTDE_DATA_PACKAGE) + bytes(body)


class FakeRtde:
    """A drop-in for the RTDE client injected as ``robot._rtde`` — records writes
    and returns a canned output sample, no socket involved."""

    def __init__(self, sample: dict | None = None):
        self.sample = sample or {
            "actual_q": [0.1, -1.5, 0.2, -1.4, 0.3, 0.0],
            "actual_qd": [0.0] * 6,
            "actual_TCP_pose": [0.4, -0.2, 0.5, 0.0, 3.14, 0.0],
            "actual_TCP_speed": [0.0] * 6,
            "actual_TCP_force": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
            "safety_status_bits": 1,
            "runtime_state": 2,
            "robot_mode": 7,
            "timestamp": 123.5,
        }
        self.speed_calls: list[float] = []
        self.do_calls: list[tuple[int, bool]] = []

    def read_outputs(self) -> dict:
        return self.sample

    def set_speed_slider(self, fraction: float) -> None:
        self.speed_calls.append(fraction)

    def set_standard_digital_output(self, pin: int, value: bool) -> None:
        self.do_calls.append((pin, value))

    def close(self) -> None:
        pass


class TestRtdeClient:
    def test_decodes_mixed_type_recipe(self):
        client = RtdeClient(RobotConfig())
        client._out_recipe_id = 1
        client._out_recipe = [
            ("actual_q", "VECTOR6D"),
            ("timestamp", "DOUBLE"),
            ("robot_mode", "INT32"),
            ("input_bits", "UINT64"),
        ]
        values = {
            "actual_q": [0.0, -1.5708, 0.1, -1.4, 0.2, 0.3],
            "timestamp": 42.25,
            "robot_mode": 7,
            "input_bits": 2**40,
        }
        client._sock = FakeSocket(_pack_data_package(1, client._out_recipe, values))
        out = client.receive()
        assert out["actual_q"] == pytest.approx(values["actual_q"])
        assert out["timestamp"] == 42.25
        assert out["robot_mode"] == 7
        assert out["input_bits"] == 2**40

    def test_recv_exactly_reassembles_split_frames(self):
        # Same frame, but recv hands back one byte at a time.
        client = RtdeClient(RobotConfig())
        client._out_recipe_id = 3
        client._out_recipe = [("actual_TCP_pose", "VECTOR6D")]
        values = {"actual_TCP_pose": [0.4, -0.2, 0.5, 0.0, 3.14, -0.1]}
        frame = _pack_data_package(3, client._out_recipe, values)
        client._sock = FakeSocket(frame, chunk=1)
        out = client.receive()
        assert out["actual_TCP_pose"] == pytest.approx(values["actual_TCP_pose"])

    def test_receive_ignores_other_recipe_ids(self):
        client = RtdeClient(RobotConfig())
        client._out_recipe_id = 1
        client._out_recipe = [("robot_mode", "INT32")]
        wrong = _pack_data_package(9, client._out_recipe, {"robot_mode": 5})
        right = _pack_data_package(1, client._out_recipe, {"robot_mode": 6})
        client._sock = FakeSocket(wrong + right)
        assert client.receive()["robot_mode"] == 6

    def test_parse_recipe_not_found_raises(self):
        client = RtdeClient(RobotConfig())
        body = bytes([1]) + b"VECTOR6D,NOT_FOUND"
        with pytest.raises(RtdeError, match="not available"):
            client._parse_recipe(body, ["actual_q", "bogus_field"])

    def test_parse_recipe_count_mismatch_raises(self):
        client = RtdeClient(RobotConfig())
        body = bytes([1]) + b"VECTOR6D"
        with pytest.raises(RtdeError, match="types"):
            client._parse_recipe(body, ["actual_q", "timestamp"])

    def test_parse_recipe_ok_returns_pairs(self):
        client = RtdeClient(RobotConfig())
        body = bytes([2]) + b"VECTOR6D,DOUBLE"
        rid, recipe = client._parse_recipe(body, ["actual_q", "timestamp"])
        assert rid == 2
        assert recipe == [("actual_q", "VECTOR6D"), ("timestamp", "DOUBLE")]

    def test_type_table_widths_round_trip(self):
        # Every declared type packs and unpacks at its calcsize width.
        for _tstr, (fmt, n) in urctl_rtde._RTDE_TYPES.items():
            sample = [1] * n if n > 1 else [3]
            packed = struct.pack(fmt, *sample)
            assert len(packed) == struct.calcsize(fmt)


class TestRobotRtde:
    def test_get_state_falls_back_without_rtde(self, fake):
        # fake fixture stubs RtdeClient.connect to refuse -> textmsg fallback.
        robot = Robot(RobotConfig())
        state = robot.get_state()
        assert state["ok"]
        assert state["joints"] is not None  # came from the Primary fallback
        # No RTDE-only extras when the fallback path is used.
        assert "tcp_force" not in state

    def test_get_state_disabled_skips_rtde(self, fake):
        robot = Robot(RobotConfig(rtde_enabled=False))
        state = robot.get_state()
        assert state["ok"] and state["joints"] is not None

    def test_get_state_uses_rtde_when_available(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        state = robot.get_state()
        assert state["ok"]
        assert state["joints"] == pytest.approx([0.1, -1.5, 0.2, -1.4, 0.3, 0.0])
        assert state["tcp_force"] == [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]

    def test_rtde_state_dispatches(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        out = robot.rtde_state()
        assert out["ok"]
        assert out["joints"] == pytest.approx([0.1, -1.5, 0.2, -1.4, 0.3, 0.0])
        assert out["tcp_force"] == [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]
        assert out["safety_status"] == 1
        assert robot.audit.records[-1].action == "rtde_state"

    def test_rtde_state_disabled_returns_not_ok(self, fake):
        robot = Robot(RobotConfig(rtde_enabled=False))
        out = robot.rtde_state()
        assert out["ok"] is False
        assert "disabled" in out["error"]

    def test_rtde_state_unreachable_returns_not_ok(self, fake):
        # No injected client + connect refused by the fake fixture.
        robot = Robot(RobotConfig())
        out = robot.rtde_state()
        assert out["ok"] is False

    def test_set_speed_override_valid_sends(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        out = robot.set_speed_override(0.3)
        assert out["ok"]
        assert robot._rtde.speed_calls == [0.3]
        assert robot.audit.records[-1].action == "set_speed_override"

    def test_set_speed_override_unsafe_does_not_send(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        out = robot.set_speed_override(1.5)
        assert out["ok"] is False
        assert "safety" in out
        assert robot._rtde.speed_calls == []

    def test_set_speed_override_dry_run_does_not_send(self, fake):
        robot = Robot(RobotConfig(), dry_run=True)
        robot._rtde = FakeRtde()
        out = robot.set_speed_override(0.5)
        assert out["ok"] and out["dry_run"]
        assert robot._rtde.speed_calls == []

    def test_set_digital_output_valid_sends(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        out = robot.set_digital_output(3, True)
        assert out["ok"]
        assert robot._rtde.do_calls == [(3, True)]

    def test_set_digital_output_bad_pin_rejected(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        out = robot.set_digital_output(9, True)
        assert out["ok"] is False
        assert robot._rtde.do_calls == []


class TestRtdeTools:
    def test_new_tools_in_schemas(self):
        names = {s["name"] for s in urctl_tools.get_tool_schemas()}
        assert {"ur_rtde_state", "ur_set_speed_override", "ur_set_digital_output"} <= names

    def test_call_rtde_state_dispatches(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        out = urctl_tools.call_tool(robot, "ur_rtde_state", {})
        assert out["ok"] and out["joints"] is not None

    def test_call_set_speed_override_dispatches(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        out = urctl_tools.call_tool(robot, "ur_set_speed_override", {"fraction": 0.5})
        assert out["ok"]
        assert robot._rtde.speed_calls == [0.5]

    def test_call_set_speed_override_rejects_bool(self, fake):
        robot = Robot(RobotConfig())
        with pytest.raises(urctl_tools.ToolError, match="number"):
            urctl_tools.call_tool(robot, "ur_set_speed_override", {"fraction": True})

    def test_call_set_digital_output_dispatches(self, fake):
        robot = Robot(RobotConfig())
        robot._rtde = FakeRtde()
        out = urctl_tools.call_tool(robot, "ur_set_digital_output", {"pin": 2, "value": True})
        assert out["ok"]
        assert robot._rtde.do_calls == [(2, True)]

    def test_call_set_digital_output_missing_arg_raises(self, fake):
        robot = Robot(RobotConfig())
        with pytest.raises(urctl_tools.ToolError, match="missing required"):
            urctl_tools.call_tool(robot, "ur_set_digital_output", {"pin": 2})


# ----- CLI -------------------------------------------------------------------


class TestCli:
    def test_move_tcp_relative_dispatches_and_succeeds(self, fake, capsys):
        rc = cli_main(["move-tcp", "0", "0.05", "0", "0", "0", "0", "--relative"])
        assert rc == 0
        out = json.loads(capsys.readouterr().out)
        assert out["action"] == "move_tcp" and out["ok"]
        sent = " ".join(fake.primary_sends)
        assert "movel" in sent and "pose_add(get_actual_tcp_pose()" in sent

    def test_move_tcp_unsafe_exits_nonzero(self, fake, capsys):
        rc = cli_main(["move-tcp", "0", "5", "0", "0", "0", "0", "--relative"])
        assert rc == 1
        out = json.loads(capsys.readouterr().out)
        assert out["ok"] is False
        assert not any("movel" in s for s in fake.primary_sends)

    def test_run_script_raw_flag_sends_verbatim(self, fake, capsys):
        rc = cli_main(["run-script", 'textmsg("x")', "--raw"])
        assert rc == 0
        assert fake.primary_sends == ['textmsg("x")']

    def test_run_script_wraps_by_default(self, fake, capsys):
        rc = cli_main(["run-script", "movel(p[0,0,0,0,0,0])"])
        assert rc == 0
        assert any("def urctl_snippet" in s for s in fake.primary_sends)

    def test_speed_valid_dispatches(self, fake, monkeypatch, capsys):
        calls: list[float] = []
        monkeypatch.setattr(urctl_rtde.RtdeClient, "set_speed_slider", lambda self, f: calls.append(f))
        rc = cli_main(["speed", "0.3"])
        assert rc == 0
        out = json.loads(capsys.readouterr().out)
        assert out["action"] == "set_speed_override" and out["ok"]
        assert calls == [0.3]

    def test_speed_unsafe_exits_nonzero(self, fake, capsys):
        rc = cli_main(["speed", "1.5"])
        assert rc == 1
        out = json.loads(capsys.readouterr().out)
        assert out["ok"] is False and "safety" in out

    def test_set_output_dispatches(self, fake, monkeypatch, capsys):
        calls: list[tuple[int, bool]] = []
        monkeypatch.setattr(
            urctl_rtde.RtdeClient,
            "set_standard_digital_output",
            lambda self, pin, value: calls.append((pin, value)),
        )
        rc = cli_main(["set-output", "1", "on"])
        assert rc == 0
        assert calls == [(1, True)]

    def test_rtde_state_unreachable_exits_nonzero(self, fake, capsys):
        # fake fixture refuses RTDE connect -> rtde-state reports not ok.
        rc = cli_main(["rtde-state"])
        assert rc == 1
