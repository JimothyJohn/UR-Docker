"""Camera-on-flange geometry (perception.handeye) and the cockpit's robot bridge
(perception.robotlink) — the latter driven through a real Robot against the
fake controller from test_urctl."""

from __future__ import annotations

import math

import pytest

from perception.handeye import (
    BRACKET_NOMINAL,
    ENV_T_FLANGE_CAMERA,
    HandEye,
    locate,
    parse_pose_text,
    transform_from_extrinsics,
)
from perception.robotlink import RobotLink
from tests.test_urctl import FakeController
from urctl.config import RobotConfig
from urctl.pose import Transform, pose_inv, pose_trans
from urctl.robot import Robot

TOOL_DOWN = [0.0, math.pi, 0.0]  # π about Y: tool z → base -Z, tool x → base -X, tool y kept


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b, strict=True))


def test_bracket_nominal_matches_readme_section_3():
    # depth origin (71.5, -17.5, 3.7) mm; x_cam = +Y, y_cam = -X, z_cam = +Z
    assert _close(BRACKET_NOMINAL.translation, (0.0715, -0.0175, 0.0037))
    assert _close(BRACKET_NOMINAL.rotate((1, 0, 0)), (0, 1, 0))
    assert _close(BRACKET_NOMINAL.rotate((0, 1, 0)), (-1, 0, 0))
    assert _close(BRACKET_NOMINAL.rotate((0, 0, 1)), (0, 0, 1))
    # a point 300 mm straight out of the lens sits 303.7 mm out of the flange face
    assert _close(HandEye().camera_to_flange((0, 0, 0.3)), (0.0715, -0.0175, 0.3037))


def test_extrinsics_move_the_colour_origin():
    ext = {"rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "translation": [0.015, 0.0, 0.0]}
    he = HandEye().with_extrinsics(ext)
    # SDK convention: p_color = R·p_depth + t, so the *depth* origin sits at
    # +15 mm along colour x; a point on the colour axis is at depth x = -15 mm,
    # and camera x is flange +Y.
    assert _close(he.camera_to_flange((0, 0, 0.3)), (0.0715, -0.0175 - 0.015, 0.3037))
    assert _close(transform_from_extrinsics(None).translation, (0, 0, 0))
    assert he.as_dict()["depth_to_color_translation"] == [0.015, 0.0, 0.0]
    assert HandEye().with_extrinsics(None).flange_to_color == HandEye().flange_to_depth


def test_from_env_parses_a_calibration_or_falls_back():
    assert HandEye.from_env({}).source == "bracket-nominal"
    he = HandEye.from_env({ENV_T_FLANGE_CAMERA: "[0.07, -0.02, 0.04, 0, 0, 1.5707963]"})
    assert he.source.startswith("env:") and _close(he.flange_to_depth.translation, (0.07, -0.02, 0.04))
    assert _close(he.flange_to_depth.rotate((1, 0, 0)), (0, 1, 0), 1e-6)
    assert parse_pose_text("1,2,3,4,5,6") == [1, 2, 3, 4, 5, 6]
    assert parse_pose_text('{"pose": [0,0,0,0,0,0]}') == [0.0] * 6
    for bad in ("", "1,2,3", "[1,2,3,4,5,nan]", "abc", "[1,2,3,4,5,6,7]"):
        with pytest.raises(ValueError):
            parse_pose_text(bad)


def test_locate_geometry_tool_down():
    """Flange 0.5 m out, 0.5 m up, tool pointing straight down; the camera sees
    the object 0.3 m ahead (i.e. 0.3 m below the lens)."""
    he = HandEye()
    flange = [0.5, 0.0, 0.5, *TOOL_DOWN]
    tcp = pose_trans(flange, [0, 0, 0.12, 0, 0, 0])  # 120 mm tool
    out = locate(he, flange, (0.0, 0.0, 0.3), tcp_pose=tcp, standoff_m=0.05)
    base_from_flange = Transform.from_pose(flange)
    expect_base = base_from_flange.apply(he.camera_to_flange((0, 0, 0.3)))
    assert _close(out["point_base_m"], expect_base, 1e-9)
    # tool down => the camera looks along base -Z; the object is 0.3037 below the flange
    assert _close(out["view_ray_base"], (0, 0, -1), 1e-9)
    assert out["point_base_m"][2] == pytest.approx(0.5 - 0.3037, abs=1e-9)
    # approach = 50 mm short of the object along that ray, current tool orientation kept
    assert out["approach_pose"][2] == pytest.approx(0.5 - 0.3037 + 0.05, abs=1e-9)
    assert out["approach_pose"][3:] == tcp[3:]
    # the lateral camera offset on the bracket shows up in base xy (flange x flips under the π about Y)
    assert out["point_base_m"][0] == pytest.approx(0.5 - 0.0715, abs=1e-9)
    assert out["point_base_m"][1] == pytest.approx(0.0 - 0.0175, abs=1e-9)
    assert out["handeye"]["source"] == "bracket-nominal" and out["standoff_m"] == 0.05


def test_locate_rejects_bad_input():
    he = HandEye()
    flange = [0.5, 0.0, 0.5, *TOOL_DOWN]
    with pytest.raises(ValueError):
        locate(he, flange, (0.0, 0.0), tcp_pose=flange)
    with pytest.raises(ValueError):
        locate(he, flange, (0.0, float("nan"), 0.3), tcp_pose=flange)
    with pytest.raises(ValueError):
        locate(he, flange, (0.0, 0.0, 0.3), tcp_pose=flange, standoff_m=2.0)
    with pytest.raises(ValueError):
        locate(he, flange, (0.0, 0.0, 0.3), tcp_pose=[0, 0, 0], standoff_m=0.1)


def test_robot_get_flange_pose_agrees_with_controller(monkeypatch):
    fake = FakeController().install(monkeypatch)
    robot = Robot(RobotConfig(host="fake"))
    fp = robot.get_flange_pose()
    assert fp["ok"] and fp["tcp"] == pytest.approx(fake.tcp_pose, abs=1e-6)
    assert fp["tcp_offset"] == pytest.approx(fake.tcp_offset, abs=1e-6)
    expect = pose_trans(fake.tcp_pose, pose_inv(fake.tcp_offset))
    assert fp["flange"][:3] == pytest.approx(expect[:3], abs=1e-5)  # the fake echoes 6 decimals
    assert fp["flange_reported"][:3] == pytest.approx(expect[:3], abs=1e-5)
    assert fp["host_controller_mismatch_m"] < 1e-5
    # the 120 mm tool along +Z with the tool pointing down puts the flange 120 mm *above* the TCP
    assert fp["flange"][2] == pytest.approx(fake.tcp_pose[2] + 0.12, abs=1e-6)
    sent = [s for s in fake.primary_sends if "urctl/flange" in s]
    assert len(sent) == 1 and "get_tcp_offset()" in sent[0] and "pose_inv" in sent[0]


def test_robot_get_flange_pose_dry_run_and_no_answer(monkeypatch):
    fake = FakeController().install(monkeypatch)
    dry = Robot(RobotConfig(host="fake"), dry_run=True).get_flange_pose()
    assert dry["ok"] and dry["dry_run"] and len(dry["flange"]) == 6 and not fake.primary_sends
    from urctl import transport

    monkeypatch.setattr(transport, "send_and_collect", lambda *a, **k: b"noise\n")  # nothing parseable
    fp = Robot(RobotConfig(host="fake")).get_flange_pose()
    assert not fp["ok"] and "no TCP pose" in fp["error"]


def test_robotlink_locate_then_move_goes_through_the_tool_registry(monkeypatch):
    fake = FakeController().install(monkeypatch)
    link = RobotLink(RobotConfig(host="fake"))
    link.attach_camera(
        {
            "extrinsics_depth_to_color": {
                "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "translation": [0.015, 0, 0],
            }
        }
    )
    d = link.describe()
    assert (
        d["host"] == "fake"
        and not d["dry_run"]
        and d["handeye"]["depth_to_color_translation"] == [0.015, 0.0, 0.0]
    )
    loc = link.locate((0.0, 0.0, 0.3), standoff_m=0.08)
    assert loc["robot"]["host_controller_mismatch_m"] < 1e-5  # the controller's pose_trans agrees
    assert (
        loc["ok"]
        and len(loc["approach_pose"]) == 6
        and loc["robot"]["tcp_offset"] == pytest.approx(fake.tcp_offset, abs=1e-6)
    )
    # the approach pose keeps the live TCP orientation
    assert loc["approach_pose"][3:] == pytest.approx(fake.tcp_pose[3:], abs=1e-6)
    mv = link.move(loc["approach_pose"])
    assert mv["ok"] and mv["action"] == "move_tcp" and mv["safety"]["ok"]
    movel = [s for s in fake.primary_sends if "movel(" in s][-1]
    assert "pose_add" not in movel and "v=0.1" in movel and "a=0.3" in movel  # absolute, slow
    # audit trail: both actions logged on the same robot
    actions = [r.action for r in link.robot.audit.records] if hasattr(link.robot.audit, "records") else None
    assert actions is None or ("get_flange_pose" in actions and "move_tcp" in actions)


def test_robotlink_refusals_and_validation(monkeypatch):
    fake = FakeController(robot_mode="POWER_OFF").install(monkeypatch)
    link = RobotLink(RobotConfig(host="fake"))
    mv = link.move([0.5, 0, 0.3, *TOOL_DOWN])
    assert not mv["ok"] and mv["safety"]["ok"] is False  # envelope refused: not RUNNING
    assert not any("movel(" in s for s in fake.primary_sends)
    with pytest.raises(ValueError):
        link.move([0.5, 0, 0.3])
    with pytest.raises(ValueError):
        link.move([0.5, 0, 0.3, *TOOL_DOWN], velocity=0)
    with pytest.raises(ValueError):
        link.move([0.5, 0, float("inf"), *TOOL_DOWN])


def test_robotlink_dry_run_needs_no_controller():
    link = RobotLink(
        RobotConfig(host="127.0.0.1", dashboard_port=1, primary_port=1, timeout=0.2), dry_run=True
    )
    loc = link.locate((0.0, 0.0, 0.3))
    assert loc["ok"] and loc["robot"]["dry_run"] is True
    mv = link.move(loc["approach_pose"])
    assert mv["ok"] and mv["dry_run"]


def test_robotlink_unreachable_is_an_error_not_a_crash():
    link = RobotLink(RobotConfig(host="127.0.0.1", dashboard_port=1, primary_port=1, timeout=0.2))
    loc = link.locate((0.0, 0.0, 0.3))
    assert not loc["ok"] and "unreachable" in loc["error"]
