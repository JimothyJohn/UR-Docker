"""UFactoryArm against UFACTORY's own firmware simulator (the 850 build).

The simulator is UFACTORY's Docker image (``danielwang123321/uf-ubuntu-docker``,
amd64, documented at docs.ufactory.cc "How to install UFACTORY Studio in
Docker"), started as an 850 with ``/xarm_scripts/xarm_start.sh 6 12``.
``scripts/ufactory-sim.sh up`` does that; CI runs it in
``.github/workflows/ufactory-sim.yml``. Point ``UFACTORY_SIM_HOST`` at it; the
tests skip when nothing answers on its control port.

These are the checks the in-process fake can't make: that the real firmware
agrees with our bytes, units and rotation convention."""

from __future__ import annotations

import math
import os
import socket

import pytest

from urctl.config import RobotConfig
from urctl.pose import rotvec_to_matrix
from urctl.ufactory import UFactoryArm, rpy_to_matrix
from urctl.xarm import XArmClient

pytestmark = pytest.mark.ufactory_sim

HOST = os.environ.get("UFACTORY_SIM_HOST", "127.0.0.1")
# Elbow bent, wrist down: away from the zero pose's singular stretch.
READY = [0.0, -0.3, -0.6, 0.0, 0.9, 0.0]


def _open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=1.0):
            return True
    except OSError:
        return False


@pytest.fixture(scope="module")
def arm():
    if not _open(HOST, 502):
        pytest.skip(f"no UFACTORY simulator at {HOST}:502 (scripts/ufactory-sim.sh up)")
    a = UFactoryArm(RobotConfig(host=HOST, platform="ufactory", timeout=5.0))
    up = a.bring_up(timeout=30.0)
    assert up["ok"], up
    assert a.move_joints(READY, velocity=1.0, acceleration=3.0, timeout=30.0)["ok"]
    yield a
    a.move_tcp([0, 0, 0, 0, 0, 0], tcp=[0.0] * 6, relative=True)  # leave the TCP at the flange
    a.close()


def test_the_simulator_is_an_850(arm):
    v = arm.version()
    assert v.model == "850", v.raw
    assert v.number >= (1, 11, 100), "the planner check needs firmware >= 1.11.100"
    st = arm.get_state()
    assert st["ok"] and st["running"] and len(st["joints"]) == 6


def test_rpy_and_axis_angle_reads_agree_under_our_convention(arm):
    """The convention every TCP-offset write relies on, checked against the firmware:
    GET_TCP_POSE (RPY) and GET_TCP_POSE_AA (rotation vector) at a compound wrist."""
    assert arm.move_joints([0.3, -0.4, -0.5, 0.6, 0.8, -0.7], velocity=1.0, acceleration=3.0)["ok"]
    rpy = arm.client.get_pose_rpy()
    aa = arm.client.get_pose_aa()
    a = rpy_to_matrix(*rpy[3:6])
    b = rotvec_to_matrix(aa[3:6])
    assert all(abs(a[i][j] - b[i][j]) < 1e-3 for i in range(3) for j in range(3)), (rpy, aa)
    assert rpy[:3] == pytest.approx(aa[:3], abs=0.01)
    assert arm.move_joints(READY, velocity=1.0, acceleration=3.0)["ok"]


def test_joint_move_lands(arm):
    target = [0.2, -0.3, -0.6, 0.0, 0.9, 0.1]
    r = arm.move_joints(target, velocity=1.0, acceleration=3.0)
    assert r["ok"], r
    assert r["landed"] == pytest.approx(target, abs=0.01)


def test_linear_moves_in_metres_land_in_metres(arm):
    start = arm.get_state()["tcp"]
    target = [start[0], start[1] + 0.05, start[2] - 0.03, *start[3:]]
    r = arm.move_tcp(target, velocity=0.1, acceleration=1.0)
    assert r["ok"], r
    back = arm.move_tcp([0.0, -0.05, 0.03, 0.0, 0.0, 0.0], relative=True, velocity=0.1, acceleration=1.0)
    assert back["ok"] and back["landed"][:3] == pytest.approx(start[:3], abs=0.002)


def test_the_planner_refuses_a_pose_out_of_reach_and_nothing_moves(arm):
    before = arm.get_state()["joints"]
    r = arm.move_tcp([1.5, 0.0, 0.3, math.pi, 0.0, 0.0])
    assert not r["ok"] and any(v["rule"] == "ik_reach" for v in r["safety"]["violations"])
    assert arm.get_state()["joints"] == pytest.approx(before, abs=1e-4)


def test_a_tcp_offset_moves_the_tcp_and_not_the_flange(arm):
    before = arm.get_flange_pose()
    r = arm.move_tcp([0, 0, 0, 0, 0, 0], relative=True, tcp=[0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
    assert r["ok"], r
    after = arm.get_flange_pose()
    assert after["tcp_offset"][2] == pytest.approx(0.1, abs=1e-4)
    assert after["flange"][:3] == pytest.approx(before["flange"][:3], abs=0.002)
    assert math.dist(after["tcp"][:3], after["flange"][:3]) == pytest.approx(0.1, abs=0.002)


def test_a_path_runs_as_one_queue(arm):
    s = arm.get_state()["tcp"]
    legs = [
        {"pose": [s[0], s[1] + 0.04, s[2], *s[3:]], "blend_m": 0.01},
        {"pose": [s[0], s[1] + 0.04, s[2] - 0.04, *s[3:]], "dwell_s": 0.2},
        {"pose": list(s)},
    ]
    r = arm.move_tcp_path(legs, timeout=60.0)
    assert r["ok"], r


def test_stop_flushes_the_queue_and_the_next_move_rearms(arm):
    s = arm.get_state()["tcp"]
    arm.move_tcp([s[0], s[1] + 0.1, *s[2:]], velocity=0.02, wait=False)
    assert arm.stop()["ok"]
    assert arm.get_state()["queued_moves"] == 0
    assert arm.move_joints(READY, velocity=1.0, acceleration=3.0)["ok"]


def test_freedrive_toggles_teaching_mode_or_says_it_was_refused(arm):
    # UFACTORY's simulator (v2.4.0) answers joint-teaching mode INVALID — the SDK gets
    # code 10 too; a real arm takes it. Either way the answer is explicit and the arm
    # ends in position mode, ready to move.
    on = arm.freedrive(True)
    if on["ok"]:
        assert on["control_mode"] == "JOINT_TEACHING"
    else:
        assert on["refused"] is True and on["control_mode"] == "POSITION", on
    off = arm.freedrive(False)
    assert off["ok"] and off["control_mode"] == "POSITION", off
    assert arm.move_joints(READY, velocity=1.0, acceleration=3.0)["ok"]


def test_report_stream_and_control_port_agree_on_the_joints(arm):
    c = XArmClient(HOST)
    try:
        assert c.read_report().joints[:6] == pytest.approx(c.get_joints()[:6], abs=1e-3)
    finally:
        c.close()


def test_the_three_point_touch_off_finds_the_plane_the_tcp_touched(arm, tmp_path, monkeypatch):
    """The workcell calls on the 850's firmware: a 100 mm TCP, three touches on a
    plane at a known height, a saved position driven back to."""
    from urctl import workcell

    monkeypatch.setenv("URCTL_CELL_STORE", str(tmp_path / "cell.json"))
    tool = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
    assert workcell.tcp_offset(arm, "set", offset=tool)["ok"]
    assert workcell.tcp_offset(arm, "get")["tcp_offset"] == pytest.approx(tool, abs=1e-4)
    s = arm.get_state()["tcp"]
    z0 = s[2] - 0.02
    corners = [(0.0, 0.0), (0.06, 0.0), (0.0, 0.06)]
    for i, (dx, dy) in enumerate(corners, start=1):
        assert arm.move_tcp([s[0] + dx, s[1] + dy, z0, *s[3:]], velocity=0.1, acceleration=1.0)["ok"]
        touched = workcell.workplane(arm, "touch", name="table", index=i)
        assert touched["ok"], touched
    plane = workcell.workplane(arm, "get", name="table")
    assert plane["table_z"] == pytest.approx(z0, abs=0.001)
    assert plane["tilt_deg"] == pytest.approx(0.0, abs=0.1)
    assert plane["tcp_offset"] == pytest.approx(tool, abs=1e-4)

    assert workcell.position(arm, "save", name="last-touch")["ok"]
    assert arm.move_joints(READY, velocity=1.0, acceleration=3.0)["ok"]
    back = workcell.position(arm, "move_to", name="last-touch", velocity=1.0, acceleration=3.0)
    assert back["ok"] and arm.get_state()["tcp"][:3] == pytest.approx([s[0], s[1] + 0.06, z0], abs=0.002)
    assert workcell.tcp_offset(arm, "set", offset=[0.0] * 6)["ok"]
    assert arm.move_joints(READY, velocity=1.0, acceleration=3.0)["ok"]
