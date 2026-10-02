"""perceptronics.armik: does the arm have a joint solution — checked against its own forward kinematics."""

from __future__ import annotations

import math

import pytest
from hypothesis import assume, given, settings
from hypothesis import strategies as st

from perceptronics import armfk, armik
from urctl.pose import Transform

JOINT = st.floats(-math.pi + 1e-3, math.pi - 1e-3)


@settings(max_examples=300, deadline=None)
@given(st.sampled_from(sorted(armfk.DH)), st.lists(JOINT, min_size=6, max_size=6))
def test_a_pose_the_arm_is_at_has_the_joints_it_is_at_among_its_solutions(model, q):
    assume(abs(math.sin(q[4])) > 1e-3)  # wrist 2 straight has its own test below
    pose = armfk.frames(q, model)[-1]
    sols = armik.solutions(pose, model)
    assert sols, "the arm is at this pose: it is reachable"
    assert len(sols) <= 8
    want = Transform.from_pose(pose)
    for s in sols:  # every answer is a real one, whatever branch it came from
        got = Transform.from_pose(armfk.frames(s, model)[-1])
        assert math.dist(got.translation, want.translation) < 1e-4
        assert all(-math.pi - 1e-9 <= v <= math.pi + 1e-9 for v in s)
    assert any(max(abs(armik._wrap(s[i] - q[i])) for i in range(6)) < 1e-4 for s in sols)


@pytest.mark.parametrize("model", sorted(armfk.DH))
@pytest.mark.parametrize("q5", [0.0, math.pi])
def test_a_straight_wrist_is_still_reachable(model, q5):
    """Wrist 2 straight: a family of joint 6 angles meets the pose, and the sweep finds some."""
    q = [0.4, -1.2, 1.5, -0.9, q5, 0.7]
    pose = armfk.frames(q, model)[-1]
    sols = armik.solutions(pose, model)
    assert sols
    for s in sols:
        assert math.dist(armfk.frames(s, model)[-1][:3], pose[:3]) < 1e-4


@pytest.mark.parametrize("model", sorted(armfk.DH))
def test_past_the_arm_there_is_no_solution(model):
    d, a, _ = armfk.DH[model]
    stretch = abs(a[1]) + abs(a[2]) + d[3] + d[4] + d[5]  # no joint arrangement is longer
    assert armik.solutions([stretch + 0.05, 0.0, d[0], 0.0, math.pi, 0.0], model) == []
    assert armik.has_solution([stretch + 0.05, 0.0, d[0], 0.0, math.pi, 0.0], model) is False


def test_inside_the_shoulder_column_there_is_no_solution():
    # the wrist centre nearer the base axis than the shoulder offset d4: no joint 1 puts it there
    assert armik.solutions([0.0, 0.0, 0.3, 0.0, 0.0, 0.0], "UR3e") == []


def test_the_ur3e_cell_reaches_its_parts_where_the_datasheet_ring_said_no():
    """2026-09-27: parts 0.27 m below the base; flanges 0.505-0.533 m from the base origin
    that the controller solved and a 0.5 m sphere refused."""
    flange = [0.36, 0.0, -0.107, math.pi, 0.0, 0.0]  # fingertips on a part at z = -0.27
    assert armik.has_solution(flange, "UR3e") is True
    low = [0.52, 0.0, -0.107, math.pi, 0.0, 0.0]  # 0.531 m from the base origin: past the sphere
    assert math.hypot(*low[:3]) > 0.5 and armik.has_solution(low, "UR3e") is True
    assert armik.has_solution([0.56, 0.0, -0.107, math.pi, 0.0, 0.0], "UR3") is False


@pytest.mark.parametrize("model", [None, "", "UR30", "Fanuc", "UR3; rm -rf"])
def test_an_arm_the_table_lacks_is_nobodys_to_judge(model):
    assert armik.has_solution([0.3, 0.0, 0.2, 0.0, math.pi, 0.0], model) is None
    if model:
        with pytest.raises(ValueError):
            armik.solutions([0.3, 0.0, 0.2, 0.0, math.pi, 0.0], model)


@settings(max_examples=200, deadline=None)
@given(
    st.lists(st.floats(-3, 3), min_size=3, max_size=3),
    st.lists(st.floats(-3.2, 3.2), min_size=3, max_size=3),
)
def test_any_pose_gets_an_answer_and_every_yes_is_true(xyz, rv):
    pose = xyz + rv
    for s in armik.solutions(pose, "UR10e"):
        got = armfk.frames(s, "UR10E")[-1]
        assert math.dist(got[:3], pose[:3]) < 1e-4


@pytest.mark.parametrize(
    ("model", "same_as"), [("UR7e", "UR5e"), ("UR12e", "UR10e"), ("UR7", "UR5"), ("ur12e", "UR10E")]
)
def test_the_ur7e_and_ur12e_are_the_ur5e_and_ur10e(model, same_as):
    """Nick, 2026-10-01. Before, these two arms got no camera-side reach check at all."""
    assert armfk.DH[armfk.model_key(model)] == armfk.DH[armfk.model_key(same_as)]
    q = [0.4, -1.2, 1.5, -0.9, 1.3, 0.7]
    pose = armfk.frames(q, armfk.model_key(same_as))[-1]
    assert armik.has_solution(pose, model) is True
    assert armik.solutions(pose, model) == armik.solutions(pose, same_as)
    reach = sum(abs(v) for v in armfk.DH[armfk.model_key(same_as)][1]) + 0.6
    assert armik.has_solution([reach, 0.0, 0.2, 0.0, 3.14159, 0.0], model) is False
