"""UFactoryArm: the UFACTORY 850 behind the Controller seam (urctl/ufactory.py).

Rotation conventions are checked as properties; the controller's behaviour
(bring-up, queued moves, refusals, metres ↔ millimetres, the planner as the
reach check) against tests/_xarm_fake.py over loopback sockets."""

from __future__ import annotations

import math
import time

import pytest
from _xarm_fake import FakeXArm
from hypothesis import given
from hypothesis import strategies as st

from urctl import Controller, xarm
from urctl.cli import main as cli_main
from urctl.config import RobotConfig
from urctl.controller import make_controller
from urctl.pose import rotvec_to_matrix
from urctl.robot import Robot
from urctl.tools import call_tool
from urctl.ufactory import (
    UFactoryArm,
    matrix_to_rpy,
    offset_from_wire,
    offset_to_wire,
    rotvec_to_rpy,
    rpy_to_matrix,
    rpy_to_rotvec,
)
from urctl.xarm import REG_MOVE_LINE, REG_SET_TCP_OFFSET

angle = st.floats(-math.pi, math.pi, allow_nan=False)


def _close(a, b, tol=1e-6):
    return all(abs(a[i][j] - b[i][j]) < tol for i in range(3) for j in range(3))


@given(angle, st.floats(-1.5, 1.5), angle)
def test_rpy_round_trips_through_the_matrix(r, p, y):
    m = rpy_to_matrix(r, p, y)
    assert _close(rpy_to_matrix(*matrix_to_rpy(m)), m)


@given(st.tuples(angle, angle, angle))
def test_rotvec_and_rpy_describe_the_same_rotation(rv):
    assert _close(rpy_to_matrix(*rotvec_to_rpy(rv)), rotvec_to_matrix(rv))
    assert _close(rotvec_to_matrix(rpy_to_rotvec(rotvec_to_rpy(rv))), rotvec_to_matrix(rv))


def test_rpy_convention_is_fixed_axis_xyz():
    """Yaw alone turns X toward Y; roll alone turns Y toward Z; at pitch ±90° the
    decomposition still reproduces the matrix (gimbal lock)."""
    yaw = rpy_to_matrix(0, 0, math.pi / 2)
    assert yaw[1][0] == pytest.approx(1.0)
    roll = rpy_to_matrix(math.pi / 2, 0, 0)
    assert roll[2][1] == pytest.approx(1.0)
    lock = rpy_to_matrix(0.3, math.pi / 2, -0.4)
    assert _close(rpy_to_matrix(*matrix_to_rpy(lock)), lock)


def test_offset_is_millimetres_and_rpy_on_the_wire():
    wire = offset_to_wire([0.0, 0.0, 0.163, 0.0, 0.0, math.pi / 2])
    assert wire == pytest.approx([0, 0, 163, 0, 0, math.pi / 2])
    assert offset_from_wire(wire) == pytest.approx([0.0, 0.0, 0.163, 0.0, 0.0, math.pi / 2])


@pytest.fixture
def arm(monkeypatch):
    fake = FakeXArm()
    monkeypatch.setenv("UFACTORY_CONTROL_PORT", str(fake.ports[0]))
    monkeypatch.setenv("UFACTORY_REPORT_PORT", str(fake.ports[1]))
    a = UFactoryArm(RobotConfig(host="127.0.0.1", platform="ufactory", timeout=2.0))
    yield fake, a
    a.close()
    fake.close()


def test_make_controller_picks_the_vendor():
    assert isinstance(make_controller(RobotConfig(platform="ufactory")), UFactoryArm)
    assert isinstance(make_controller(RobotConfig(platform="e-series")), Robot)


def test_it_is_a_controller_with_the_850s_reach(arm):
    _, a = arm
    assert isinstance(a, Controller)
    assert a.safety.max_reach == pytest.approx(0.85)
    assert a.safety.model == "UF850"


def test_motion_is_refused_until_the_arm_is_brought_up(arm):
    fake, a = arm
    r = a.move_joints([0.1, 0, 0, 0, 0, 0])
    assert not r["ok"] and r["safety"]["violations"][0]["rule"] == "robot_state"
    assert not any(reg == 23 for reg, _ in fake.requests)  # nothing queued


def test_bring_up_survives_a_motion_enable_that_is_never_answered(arm):
    # UFACTORY's simulator (firmware v2.4.0) acts on MOTION_EN but never replies; the
    # SDK times out and carries on. The verdict is the report's enable bits, not the reply.
    fake, a = arm
    fake.unanswered.add(xarm.REG_MOTION_EN)
    up = a.bring_up(timeout=3.0)
    assert up["ok"] and up["robot_mode"] == "RUNNING", up
    assert up["motion_enable_unanswered"] is True
    assert a.move_joints([0.1, 0, 0, 0, 0, 0], velocity=1.0, timeout=5.0)["ok"]


def test_an_unanswered_motion_enable_that_did_nothing_is_still_a_failed_bring_up(arm):
    fake, a = arm
    fake.unanswered.add(xarm.REG_MOTION_EN)
    fake.enable_takes = False
    up = a.bring_up(timeout=1.0)
    assert not up["ok"] and up["robot_mode"] == "DISABLED", up


def test_bring_up_then_state_in_toolkit_units(arm):
    fake, a = arm
    up = a.bring_up(timeout=3.0)
    assert up["ok"] and up["robot_mode"] == "RUNNING"
    st_ = a.get_state()
    assert st_["ok"] and st_["running"] and st_["safety_mode"] == "NORMAL"
    assert st_["model"] == "850" and st_["firmware"] == "2.5.105"
    assert st_["tcp"][:3] == pytest.approx([0.3, 0.0, 0.2], abs=1e-6)  # metres
    assert len(st_["joints"]) == 6


def test_an_estop_is_named_and_not_cleared_remotely(arm):
    fake, a = arm
    fake.error = 1
    up = a.bring_up(timeout=1.0)
    assert not up["ok"] and "by hand" in up["reply"] and up["error_code"] == 1
    assert a.get_state()["safety_mode"] == "EMERGENCY_STOP"


def test_move_joints_waits_for_the_queue_and_lands(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    r = a.move_joints([0.1, -0.2, 0.3, 0.0, 0.5, 0.0], velocity=0.5, acceleration=0.8)
    assert r["ok"] and r["done"] and r["landed"] == pytest.approx([0.1, -0.2, 0.3, 0.0, 0.5, 0.0], abs=1e-6)


def test_move_tcp_sends_millimetres_and_checks_the_path_first(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    target = [0.35, 0.05, 0.25, math.pi, 0.0, 0.0]
    r = a.move_tcp(target, velocity=0.1, acceleration=1.0)
    assert r["ok"], r
    moves = [p for reg, p in fake.requests if reg == REG_MOVE_LINE]
    assert moves[0][42] == 1  # the planner check came first, from the actual state; nothing moved
    assert moves[-1][42] == 0
    from urctl.xarm import from_fp32_le

    sent = from_fp32_le(moves[-1], 8)
    assert sent[:3] == pytest.approx([350, 50, 250], abs=1e-3) and sent[6] == pytest.approx(100.0)


def test_the_planner_refuses_an_unreachable_pose_and_nothing_moves(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    r = a.move_tcp([0.9, 0.0, 0.2, math.pi, 0.0, 0.0])
    assert not r["ok"]
    assert any(v["rule"] == "ik_reach" for v in r["safety"]["violations"])
    assert all(p[42] for reg, p in fake.requests if reg == REG_MOVE_LINE)  # only checks


def test_a_single_pose_reach_check_plans_from_the_actual_state(arm):
    # only_check_type 2/3 plan from the controller's *intermediate* state, which only
    # checks update: after a joint move it is stale. On UFACTORY's simulator a 5 cm
    # move after a movej came back "unreachable" (0x18) under 2 and clear under 1.
    fake, a = arm
    a.bring_up(timeout=3.0)
    assert a.move_joints([0.2, 0, 0, 0, 0, 0])["ok"]
    assert a.move_tcp([0.3, 0.05, 0.2, math.pi, 0.0, 0.0])["ok"]
    checks = [p[42] for reg, p in fake.requests if reg == REG_MOVE_LINE and p[42]]
    assert checks == [1]


def test_a_speed_verdict_from_rest_is_asked_again_and_the_second_answer_stands(arm):
    # UFACTORY's simulator: the first planner check after a finished move can answer
    # 24 (Speed Exceeds Limit) with the joints still on target; the same check again is clear.
    fake, a = arm
    a.bring_up(timeout=3.0)
    fake.check_verdicts = [24]
    r = a.move_tcp([0.3, 0.05, 0.2, math.pi, 0.0, 0.0])
    assert r["ok"], r
    assert [p[42] for reg, p in fake.requests if reg == REG_MOVE_LINE and p[42]] == [1, 1]


def test_a_speed_verdict_twice_is_a_refusal_that_names_the_planners_reason(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    fake.check_verdicts = [24, 24]
    r = a.move_tcp([0.3, 0.05, 0.2, math.pi, 0.0, 0.0])
    assert not r["ok"] and any(v["rule"] == "ik_reach" for v in r["safety"]["violations"])
    assert r["planner"] == [{"code": 24, "reason": "Speed Exceeds Limit"}]


def test_any_other_planner_verdict_is_final(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    fake.check_verdicts = [22]
    r = a.move_tcp([0.3, 0.05, 0.2, math.pi, 0.0, 0.0])
    assert not r["ok"] and r["planner"] == [{"code": 22, "reason": "Self-Collision Error"}]
    assert len([p for reg, p in fake.requests if reg == REG_MOVE_LINE and p[42]]) == 1


def test_a_tcp_offset_write_leaves_an_armed_arm_armed(arm):
    # SET_TCP_OFFSET drops the firmware to state 5; every check then answers 255 and
    # every move is refused until set_state(0). Seen on UFACTORY's simulator.
    fake, a = arm
    a.bring_up(timeout=3.0)
    assert a.set_tcp_offset([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])["ok"]
    assert fake.state < xarm.STATE_STOPPED
    assert a.move_tcp([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], relative=True, tcp=[0.0, 0.0, 0.163, 0, 0, 0])["ok"]
    assert a.move_tcp_path([{"pose": [0.3, 0.0, 0.25, math.pi, 0, 0]}], tcp=[0.0] * 6)["ok"]


def test_a_tcp_offset_write_does_not_re_arm_a_stopped_arm(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    assert a.stop()["ok"]
    a.set_tcp_offset([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
    assert fake.state >= xarm.STATE_STOPPED


def test_relative_move_adds_the_delta_in_the_base_frame(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    r = a.move_tcp([0.0, 0.05, 0.0, 0.0, 0.0, 0.0], relative=True)
    assert r["ok"] and r["target"][:3] == pytest.approx([0.3, 0.05, 0.2], abs=1e-6)


def test_a_relative_step_in_millimetres_by_mistake_is_refused(arm):
    _, a = arm
    a.bring_up(timeout=3.0)
    r = a.move_tcp([0.0, 50.0, 0.0, 0.0, 0.0, 0.0], relative=True)
    assert not r["ok"] and r["safety"]["violations"][0]["rule"] == "tcp_step"


def test_tcp_override_reaches_the_controller_in_mm_and_rpy(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    a.move_tcp([0.3, 0.0, 0.2, math.pi, 0.0, 0.0], tcp=[0.0, 0.0, 0.163, 0.0, 0.0, 0.0])
    assert any(reg == REG_SET_TCP_OFFSET for reg, _ in fake.requests)
    assert fake.offset_rpy[2] == pytest.approx(163.0)
    flange = a.get_flange_pose()
    # the fake's TCP is pointing down (rotvec pi about X): the flange is 163 mm above it
    assert flange["flange"][2] == pytest.approx(0.2 + 0.163, abs=1e-5)


def test_a_fault_during_a_move_is_a_protective_stop_with_its_reason(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    fake.refuse_moves_with_error = 22
    r = a.move_joints([0.2, 0, 0, 0, 0, 0])
    assert not r["ok"]
    assert r.get("protective_stop") is True and "Self-Collision" in r["error"]
    assert a.get_state()["safety_mode"] == "PROTECTIVE_STOP"


def test_path_queues_every_leg_with_blends_and_dwells(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    legs = [
        {"pose": [0.3, 0.0, 0.3, math.pi, 0, 0], "blend_m": 0.01},
        {"pose": [0.3, 0.1, 0.3, math.pi, 0, 0], "dwell_s": 0.1},
        {"pose": [0.3, 0.1, 0.2, math.pi, 0, 0]},
    ]
    r = a.move_tcp_path(legs, timeout=5.0)
    assert r["ok"] and r["completed_legs"] == 3
    real = [p for reg, p in fake.requests if reg == REG_MOVE_LINE and p[42] == 0]
    assert len(real) == 3
    from urctl.xarm import from_fp32_le

    assert from_fp32_le(real[0], 10)[9] == pytest.approx(10.0)  # 0.01 m blend → 10 mm
    assert from_fp32_le(real[2], 10)[9] == -1.0  # the last leg stops
    checks = [p[42] for reg, p in fake.requests if reg == REG_MOVE_LINE and p[42]]
    assert checks == [1, 3, 2]


def test_path_with_a_gripper_leg_is_refused_up_front(arm):
    _, a = arm
    with pytest.raises(ValueError, match="gripper"):
        a.move_tcp_path([{"pose": [0.3, 0, 0.3, math.pi, 0, 0], "gripper": "close"}])


def test_trajectory_blends_all_but_the_last_waypoint(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    r = a.move_trajectory([[0.1, 0, 0, 0, 0, 0], [0.2, 0, 0, 0, 0, 0]], blend_radius=0.02)
    assert r["ok"]
    regs = [reg for reg, _ in fake.requests if reg in (23, 24)]
    assert regs == [24, 23]


def test_stop_flushes_the_queue_and_the_next_move_rearms(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    a.move_joints([0.3, 0, 0, 0, 0, 0], wait=False)
    assert a.stop()["ok"] and fake.queue == []
    assert a.move_joints([0.1, 0, 0, 0, 0, 0])["ok"]


def test_freedrive_is_joint_teaching_mode_and_motion_waits_for_it_to_end(arm):
    fake, a = arm
    a.bring_up(timeout=3.0)
    assert a.freedrive(True)["control_mode"] == "JOINT_TEACHING"
    r = a.move_joints([0.1, 0, 0, 0, 0, 0])
    assert not r["ok"] and "freedrive" in r["safety"]["violations"][0]["detail"]
    assert a.freedrive(False)["ok"]
    assert a.move_joints([0.1, 0, 0, 0, 0, 0])["ok"]


def test_a_refused_joint_teaching_mode_is_reported_at_once_and_the_arm_still_moves(arm):
    # UFACTORY's simulator answers SET_MODE 2 with the INVALID bit (no hand-guiding in a sim)
    fake, a = arm
    a.bring_up(timeout=3.0)
    fake.refused_modes.add(xarm.MODE_JOINT_TEACH)
    t0 = time.monotonic()
    r = a.freedrive(True)
    assert time.monotonic() - t0 < 1.0, "a refusal must not wait out the mode poll"
    assert not r["ok"] and r["refused"] is True and r["control_mode"] == "POSITION", r
    assert a.move_joints([0.1, 0, 0, 0, 0, 0])["ok"]


def test_ur_only_calls_answer_not_supported():
    a = UFactoryArm(RobotConfig(host="127.0.0.1", platform="ufactory"))
    for r in (a.run_script("textmsg(1)"), a.load_program("x"), a.popup("hi"), a.gripper("close")):
        assert r["ok"] is False and r["supported"] is False


def test_an_unreachable_arm_is_ok_false_not_an_exception():
    a = UFactoryArm(RobotConfig(host="127.0.0.1", platform="ufactory", timeout=0.3))
    a.client.port = 1
    a.client.report_port = 1
    assert a.get_state()["ok"] is False
    assert a.move_joints([0.1, 0, 0, 0, 0, 0])["ok"] is False


def test_dry_run_sends_nothing():
    a = UFactoryArm(RobotConfig(host="127.0.0.1", platform="ufactory"), dry_run=True)
    a.client.port = 1  # would fail loudly if touched
    assert a.move_joints([0.1, 0, 0, 0, 0, 0])["ok"]
    assert a.move_tcp([0.3, 0, 0.2, math.pi, 0, 0])["reply"] == "(dry-run)"


def test_the_tool_registry_drives_it(arm):
    fake, a = arm
    assert call_tool(a, "bring_up", {})["ok"]
    assert call_tool(a, "move_joints", {"joints": [0.1, 0, 0, 0, 0, 0]})["ok"]
    assert call_tool(a, "ur_get_state", {})["robot_mode"] == "RUNNING"


def test_cli_selects_ufactory_and_refuses_ur_only_commands(arm, capsys, monkeypatch):
    fake, _ = arm
    assert cli_main(["--host", "127.0.0.1", "--platform", "ufactory", "state"]) == 0
    assert '"vendor": "ufactory"' in capsys.readouterr().out
    cli_main(["--host", "127.0.0.1", "--platform", "ufactory", "programs"])
    assert "needs a UR controller" in capsys.readouterr().out
