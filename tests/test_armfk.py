"""perceptronics.armfk: the linkage from the joints, checked against the controller."""

from __future__ import annotations

import math

import pytest

from perceptronics import armfk
from urctl.pose import Transform

# UR3e, 2026-09-27 21:5x at the picture pose: RTDE actual_q and the flange from
# actual_TCP_pose ∘ tcp_offset⁻¹ (the controller's own answer)
Q = [-1.021, -1.557, 2.302, -2.881, -1.852, 0.61]
FLANGE = [-0.2359, 0.1825, 0.2213]


def test_the_ur3e_table_reproduces_the_controllers_flange():
    chain = armfk.frames(Q, "UR3E")
    assert len(chain) == 7 and chain[0][:3] == pytest.approx([0, 0, 0])
    assert armfk.flange_error_m(chain, FLANGE) < 0.002  # 0.84 mm measured; q is rounded here
    assert chain[1][:3] == pytest.approx([0, 0, 0.15185])  # the shoulder sits d1 above the base


def test_the_wrong_table_fails_the_check():
    """A wrong model on the same joints lands the flange far off: the cockpit then draws nothing."""
    for model in ("UR5E", "UR10E", "UR20"):
        assert armfk.flange_error_m(armfk.frames(Q, model), FLANGE) > armfk.FLANGE_TOLERANCE_M


@pytest.mark.parametrize("model", sorted(armfk.DH))
def test_each_frame_is_a_rotation_and_zero_joints_stretch_the_arm_out(model):
    chain = armfk.frames([0.0] * 6, model)
    for pose in chain:
        T = Transform.from_pose(pose)
        x, z = T.rotate((1, 0, 0)), T.rotate((0, 0, 1))
        assert math.hypot(*x) == pytest.approx(1.0) and sum(
            a * b for a, b in zip(x, z, strict=True)
        ) == pytest.approx(0.0, abs=1e-9)
    d, a, _ = armfk.DH[model]
    reach = abs(a[1]) + abs(a[2])
    assert math.hypot(chain[-1][0], chain[-1][1]) == pytest.approx(math.hypot(reach, d[3] + d[5]), abs=1e-6)


@pytest.mark.parametrize(
    ("given", "key"),
    [
        ("UR3e", "UR3E"),
        ("ur3e", "UR3E"),
        ("UR3", "UR3E"),
        ("UR20", "UR20"),
        ("", None),
        (None, None),
        ("KUKA", None),
    ],
)
def test_model_key(given, key):
    assert armfk.model_key(given) == key


def test_frames_carry_the_arm_only_when_it_checks_out(monkeypatch):
    import time

    from perceptronics.config import PerceptionConfig
    from perceptronics.posestream import PoseStream
    from perceptronics.realsense import SyntheticRgbdCamera
    from perceptronics.webapp import ViewerApp
    from urctl.config import RobotConfig

    ps = PoseStream(RobotConfig(host="x"), history_s=60.0)
    app = ViewerApp(
        SyntheticRgbdCamera(width=32, height=24, fps=0), config=PerceptionConfig(), pose_stream=ps
    )
    flange = armfk.frames(Q, "UR3E")[-1]
    t = time.time()
    ps.add(t, flange, Q)
    app._latest_t = t
    monkeypatch.setenv("UR_ROBOT_MODEL", "UR3e")
    arm = app.frame_pose()["arm"]
    assert arm["model"] == "UR3E" and len(arm["frames"]) == 7 and arm["fk_error_mm"] < 0.01
    monkeypatch.setenv("UR_ROBOT_MODEL", "UR10e")  # the wrong arm for these joints
    assert "arm" not in app.frame_pose()
    monkeypatch.delenv("UR_ROBOT_MODEL")
    assert "arm" not in app.frame_pose() and "flange_pose" in app.frame_pose()
