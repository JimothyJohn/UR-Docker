"""Positions, TCP offsets and workplanes: one API on every arm (urctl/workcell.py),
and the taught table against the RealSense's (perceptronics/volume.check_surface)."""

from __future__ import annotations

import json
import math

import pytest
from _xarm_fake import FakeXArm
from hypothesis import given, settings
from hypothesis import strategies as st

from perceptronics.partspec import PartSpec
from perceptronics.synthscene import Box, camera_looking_down, render_depth
from perceptronics.volume import Surface, check_surface, find_parts
from tests.test_urctl import FakeController
from urctl.config import RobotConfig
from urctl.pose import Transform
from urctl.robot import Robot
from urctl.tools import call_tool
from urctl.ufactory import UFactoryArm
from urctl.workcell import (
    CellStore,
    WorkcellError,
    Workplane,
    check_name,
    compare_planes,
    default_store_path,
    position,
    tcp_offset,
    workplane,
)

coord = st.floats(-0.8, 0.8, allow_nan=False)
point = st.tuples(coord, coord, st.floats(-0.5, 0.5))


# -- the plane ------------------------------------------------------------------------------


@given(point, point, point)
def test_a_workplane_passes_through_its_three_points_and_faces_up(a, b, c):
    try:
        wp = Workplane.from_points(a, b, c)
    except WorkcellError:
        return  # too close / collinear: refused, which is the other half of the contract
    for p in (a, b, c):
        assert wp.height(p) == pytest.approx(0.0, abs=1e-9)
    assert wp.normal[2] >= 0
    assert sum(v * v for v in wp.normal) == pytest.approx(1.0)
    assert sum(wp.x_axis[i] * wp.normal[i] for i in range(3)) == pytest.approx(0.0, abs=1e-9)


@given(point, point, point)
def test_the_workplane_and_the_pick_surface_agree_on_the_convention(a, b, c):
    """The URCap's PoseMath.plane, Surface.from_points and Workplane are one convention:
    the same three touches give the same frame and the same area."""
    try:
        wp = Workplane.from_points(a, b, c)
    except WorkcellError:
        return
    s = Surface.from_points(a, b, c)
    for mine, theirs in (
        (wp.origin, s.origin),
        (wp.x_axis, s.x_axis),
        (wp.y_axis, s.y_axis),
        (wp.normal, s.normal),
    ):
        assert mine == pytest.approx(theirs, abs=1e-9)
    assert (min(0, wp.size[0]), max(0, wp.size[0])) == pytest.approx(s.area[:2], abs=1e-9)


@given(point, point, point)
@settings(max_examples=50)
def test_the_pose_form_is_the_plane_frame(a, b, c):
    try:
        wp = Workplane.from_points(a, b, c)
    except WorkcellError:
        return
    T = Transform.from_pose(wp.pose())
    assert T.rotate((0, 0, 1)) == pytest.approx(wp.normal, abs=1e-6)
    assert T.translation == pytest.approx(wp.origin, abs=1e-9)
    assert Workplane.from_dict(json.loads(json.dumps(wp.as_dict()))).normal == pytest.approx(wp.normal)


def test_a_level_table_reads_level_and_its_height():
    wp = Workplane.from_points((0.3, -0.1, -0.27), (0.5, -0.1, -0.27), (0.35, 0.1, -0.27))
    d = wp.as_dict()
    assert d["tilt_deg"] == pytest.approx(0.0) and d["table_z"] == pytest.approx(-0.27)
    assert wp.size == pytest.approx((0.2, 0.2))


def test_degenerate_touches_are_refused_with_what_to_do():
    with pytest.raises(WorkcellError, match="apart"):
        Workplane.from_points((0, 0, 0), (0.005, 0, 0), (0, 0.1, 0))
    with pytest.raises(WorkcellError, match="one line"):
        Workplane.from_points((0, 0, 0), (0.2, 0, 0), (0.4, 0.001, 0))
    with pytest.raises(WorkcellError, match="finite"):
        Workplane.from_points((0, 0, math.nan), (0.2, 0, 0), (0, 0.2, 0))


def test_compare_planes_reads_offset_and_tilt():
    wp = Workplane.from_points((0, 0, 0), (0.2, 0, 0), (0, 0.2, 0))
    same = compare_planes(wp, (0, 0, 1), (5, 5, 0))
    assert same == {"offset_mm": 0.0, "tilt_deg": 0.0}
    up = compare_planes(wp, (0, 0, -1), (0, 0, 0.003))  # a downward normal is flipped
    assert up["offset_mm"] == pytest.approx(3.0)
    t = math.radians(1.0)
    tilted = compare_planes(wp, (math.sin(t), 0, math.cos(t)), (0.1, 0.1, 0.0))
    assert tilted["tilt_deg"] == pytest.approx(1.0, abs=1e-3) and tilted["offset_mm"] == pytest.approx(
        0.0, abs=1e-6
    )


# -- names and the store ---------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad", ["", "../../etc/passwd", "a/b", "line\nbreak", "\x1b[31mred", "x" * 65, "-leading", "café", " sp"]
)
def test_names_that_could_bend_a_path_or_a_log_are_refused(bad):
    with pytest.raises(WorkcellError):
        check_name(bad)


def test_store_path_from_the_environment():
    assert str(default_store_path({})).endswith("captures/cell/default.json")
    assert str(default_store_path({"UR_CELL": "ur3"})).endswith("captures/cell/ur3.json")
    assert str(default_store_path({"URCTL_CELL_STORE": "/x/y.json", "UR_CELL": "ur3"})) == "/x/y.json"
    with pytest.raises(WorkcellError):
        default_store_path({"UR_CELL": "../evil"})


def test_store_round_trips_and_saves_atomically(tmp_path):
    path = tmp_path / "cell" / "ur3.json"
    s = CellStore.open(path)
    s.put_tcp("handE", [0, 0, 0.163, 0, 0, 0])
    s.put_position("home", joints=[0, -1.57, 0, -1.57, 0, 0], tcp=None)
    s.put_workplane("table", Workplane.from_points((0, 0, -0.27), (0.2, 0, -0.27), (0, 0.2, -0.27)))
    s.save()
    assert [p.name for p in path.parent.iterdir()] == ["ur3.json"]  # no temp files left
    again = CellStore.open(path)
    assert again.tcp("handE")[2] == 0.163
    assert again.position("home")["joints"][1] == -1.57
    assert again.workplane("table").as_dict()["table_z"] == pytest.approx(-0.27)


def test_a_corrupt_store_is_an_error_not_a_silent_reset(tmp_path):
    p = tmp_path / "c.json"
    p.write_text("{not json")
    with pytest.raises(WorkcellError, match="cannot read"):
        CellStore.open(p)
    p.write_text("[1, 2]")
    with pytest.raises(WorkcellError, match="not a JSON object"):
        CellStore.open(p)


def test_touches_with_a_different_tool_restart_the_plane(tmp_path):
    s = CellStore.open(tmp_path / "c.json")
    assert not s.touch("t", 1, (0, 0, 0), [0, 0, 0.1, 0, 0, 0])["complete"]
    assert not s.touch("t", 2, (0.2, 0, 0), [0, 0, 0.1, 0, 0, 0])["complete"]
    restarted = s.touch("t", 3, (0, 0.2, 0), [0, 0, 0.2, 0, 0, 0])
    assert not restarted["complete"] and restarted["touched"] == [3]


def test_unsupported_values_are_refused(tmp_path):
    s = CellStore.open(tmp_path / "c.json")
    with pytest.raises(WorkcellError):
        s.put_tcp("x", [0, 0, math.inf, 0, 0, 0])
    with pytest.raises(WorkcellError):
        s.put_position("p", joints=None, tcp=None)
    with pytest.raises(WorkcellError):
        s.touch("t", 4, (0, 0, 0), None)


# -- the same calls on every arm --------------------------------------------------------------


@pytest.fixture
def store_env(tmp_path, monkeypatch):
    path = tmp_path / "cell.json"
    monkeypatch.setenv("URCTL_CELL_STORE", str(path))
    return path


@pytest.fixture
def ufactory(monkeypatch, store_env):
    fake = FakeXArm()
    monkeypatch.setenv("UFACTORY_CONTROL_PORT", str(fake.ports[0]))
    monkeypatch.setenv("UFACTORY_REPORT_PORT", str(fake.ports[1]))
    arm = UFactoryArm(RobotConfig(host="127.0.0.1", platform="ufactory", timeout=2.0))
    assert arm.bring_up(timeout=3.0)["ok"]
    yield fake, arm
    arm.close()
    fake.close()


@pytest.fixture
def ur(monkeypatch, store_env):
    fake = FakeController().install(monkeypatch)
    return fake, Robot(RobotConfig(host="fake-ur.invalid"))


def test_tcp_read_write_on_a_ufactory_arm(ufactory):
    fake, arm = ufactory
    r = tcp_offset(arm, "set", offset=[0.0, 0.0, 0.163, 0.0, 0.0, math.pi / 2])
    assert r["ok"] and r["tcp_offset"][2] == pytest.approx(0.163)
    assert fake.offset_rpy[2] == pytest.approx(163.0) and fake.offset_rpy[5] == pytest.approx(math.pi / 2)
    assert tcp_offset(arm, "get")["tcp_offset"][5] == pytest.approx(math.pi / 2, abs=1e-6)


def test_tcp_read_write_on_a_ur(ur):
    fake, robot = ur
    r = tcp_offset(robot, "set", offset=[0.0, 0.0, 0.163, 0.0, 0.0, 0.0])
    assert r["ok"] and fake.tcp_offset[2] == pytest.approx(0.163)
    assert any("set_tcp(p[0.0, 0.0, 0.163" in s for s in fake.primary_sends)


def test_a_ur_that_ignores_scripts_reports_the_write_failed(ur):
    fake, robot = ur
    fake.ignore_scripts = True
    assert tcp_offset(robot, "set", offset=[0, 0, 0.1, 0, 0, 0], store=None)["ok"] is False


def test_named_tcps_save_and_use(ufactory, store_env):
    fake, arm = ufactory
    assert tcp_offset(arm, "save", name="handE", offset=[0, 0, 0.163, 0, 0, 0])["ok"]
    assert tcp_offset(arm, "use", name="handE")["ok"] and fake.offset_rpy[2] == pytest.approx(163.0)
    assert json.loads(store_env.read_text())["tcps"]["handE"][2] == 0.163
    assert tcp_offset(arm, "list")["tcps"] == {"handE": [0, 0, 0.163, 0, 0, 0]}
    assert tcp_offset(arm, "use", name="nope")["ok"] is False


def test_positions_save_and_move_to(ufactory):
    fake, arm = ufactory
    arm.move_joints([0.1, -0.2, 0.3, 0.0, 0.5, 0.0])
    saved = position(arm, "save", name="look")
    assert saved["ok"] and saved["joints"] == pytest.approx([0.1, -0.2, 0.3, 0.0, 0.5, 0.0], abs=1e-6)
    assert saved["tcp_offset"] is not None
    arm.move_joints([0.0] * 6)
    back = position(arm, "move_to", name="look")
    assert back["ok"] and back["landed"] == pytest.approx(saved["joints"], abs=1e-6)
    assert position(arm, "list")["positions"] == ["look"]
    assert position(arm, "delete", name="look")["ok"]
    assert position(arm, "get", name="look")["ok"] is False


def test_three_touches_make_the_table_plane(ufactory):
    fake, arm = ufactory
    arm.set_tcp_offset([0, 0, 0.2, 0, 0, 0])
    for i, xyz in enumerate([(300, -100, -270), (500, -100, -270), (350, 100, -270)], start=1):
        fake.pose = [*xyz, math.pi, 0.0, 0.0]
        r = workplane(arm, "touch", name="table", index=i)
        assert r["ok"]
    assert r["complete"] and r["workplane"]["table_z"] == pytest.approx(-0.27)
    assert r["workplane"]["tilt_deg"] == pytest.approx(0.0, abs=1e-6)
    assert r["workplane"]["tcp_offset"][2] == pytest.approx(0.2)
    arm.set_tcp_offset([0, 0, 0, 0, 0, 0])
    assert workplane(arm, "use_tcp", name="table")["ok"] and fake.offset_rpy[2] == pytest.approx(200.0)


def test_the_tools_are_the_same_on_both_vendors(ufactory, ur):
    for robot in (ufactory[1], ur[1]):
        assert call_tool(robot, "tcp_offset", {"action": "get"})["ok"]
        assert call_tool(
            robot,
            "workplane",
            {"action": "define", "name": "t", "points": [[0, 0, 0], [0.2, 0, 0], [0, 0.2, 0]]},
        )["ok"]
        assert call_tool(robot, "workplane", {"action": "get", "name": "t"})["table_z"] == 0.0
        assert call_tool(robot, "workplane", {"action": "nope"})["ok"] is False


# -- the taught plane against the camera's table ----------------------------------------------


@given(
    dz=st.floats(-0.012, 0.012),
    tilt=st.floats(0.0, 1.5),
    heading=st.floats(-math.pi, math.pi),
)
@settings(max_examples=40, deadline=None)
def test_check_surface_reads_the_offset_and_tilt_of_the_table(dz, tilt, heading):
    """Points on a table ``dz`` above a taught plane and tilted ``tilt``° toward
    ``heading``, with a block standing on it: the check recovers both."""
    taught = Surface.from_points((0.2, -0.15, -0.27), (0.5, -0.15, -0.27), (0.2, 0.15, -0.27))
    t = math.tan(math.radians(tilt))
    gx, gy = t * math.cos(heading), t * math.sin(heading)
    cx, cy = 0.35, 0.0
    pts = []
    for i in range(61):
        for j in range(61):
            x, y = 0.2 + 0.3 * i / 60, -0.15 + 0.3 * j / 60
            z = -0.27 + dz + gx * (x - cx) + gy * (y - cy)
            if abs(x - 0.3) < 0.03 and abs(y) < 0.02:
                z += 0.03  # a part on the table: not the table
            pts.append((x, y, z))
    got = check_surface(pts, taught)
    assert got is not None
    assert got["offset_mm"] == pytest.approx(dz * 1000, abs=0.3)
    assert got["tilt_deg"] == pytest.approx(tilt, abs=0.02)
    assert got["coverage"] == pytest.approx(1.0, abs=0.01)


def test_check_surface_needs_the_plane_in_view():
    taught = Surface.from_points((0, 0, 0), (0.2, 0, 0), (0, 0.2, 0))
    assert check_surface([(1.0, 1.0, 0.0)] * 100, taught) is None


W, H = 320, 180
K = {"fx": 230.0, "fy": 230.0, "ppx": W / 2, "ppy": H / 2}


def test_a_pick_with_a_taught_plane_reports_how_the_camera_sees_it():
    """End to end through the depth renderer: a table at -0.270, the plane touched off
    3 mm low and 0.8° tilted. The scene carries the check and a note about the tilt."""
    T = camera_looking_down(0.35, 0.0, 0.12)
    depth = render_depth(W, H, K, T, [Box(0.35, 0.0, 0.06, 0.04, 0.03)], table_z=-0.27)
    rise = 0.2 * math.tan(math.radians(0.8))
    taught = Surface.from_points((0.25, -0.1, -0.273), (0.45, -0.1, -0.273 + rise), (0.25, 0.1, -0.273))
    sc = find_parts(W, H, depth, 0.001, K, T, spec=PartSpec.from_mm(60, 40, 30), surface=taught)
    # at the area's centre (0.1 m along the rise) the taught plane is 3 mm low less the rise
    taught_z_at_centre = -0.273 + 0.1 * math.tan(math.radians(0.8))
    assert sc.surface_check["offset_mm"] == pytest.approx((-0.27 - taught_z_at_centre) * 1000, abs=0.5)
    assert sc.surface_check["tilt_deg"] == pytest.approx(0.8, abs=0.1)
    assert sc.as_dict()["surface_check"] == sc.surface_check
    assert len(sc.parts) == 1


# -- the cockpit's check of a stored workplane --------------------------------------------------

from tests.test_perceptronics_webapp import get, server  # noqa: E402, F401  (the fixture)


def _get_json(base, path):
    import urllib.error

    try:
        status, _, body = get(base, path)
    except urllib.error.HTTPError as e:
        status, body = e.code, e.read()
    return status, json.loads(body)


def test_the_cockpit_checks_a_stored_workplane_by_name(server, store_env):  # noqa: F811 (pytest fixture)
    base, app, _ = server
    status, body = _get_json(base, "/api/workplane/check?name=..%2Fetc")
    assert status == 400 and "name must be" in body["error"]
    status, body = _get_json(base, "/api/workplane/check?name=missing")
    assert status == 400 and "no workplane named" in body["error"]
    s = CellStore.open(store_env)
    s.put_workplane(
        "table", Workplane.from_points((0.25, -0.1, -0.27), (0.45, -0.1, -0.27), (0.25, 0.1, -0.27))
    )
    s.save()
    status, body = _get_json(base, "/api/workplane/check?name=table")
    # the synthetic camera has no robot: the plane can't be placed in the picture, and it says so
    assert status == 200 and body["ok"] is False and "no live robot pose" in body["error"]
