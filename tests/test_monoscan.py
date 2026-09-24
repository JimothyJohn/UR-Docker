"""Monocular scan (docs/mono-scan.md): part library, table plane, pose
recorder, synthetic sweep → locate, latency fit, CLI. The vision parts need
numpy + OpenCV (the `perception` extra) and skip without them."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import pytest

from perception.partlib import Part, PartLibrary, write_box_stl
from perception.posestream import LinearMotion, PoseRecorder, ReplayClient, interpolate_pose
from perception.tableplane import Plane
from perception.touch import TouchSet
from urctl.pose import Transform, pose_trans

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")


# ----- part library ------------------------------------------------------------


def test_box_stl_roundtrip_and_resting_poses(tmp_path):
    p = write_box_stl(tmp_path / "b.stl", 0.05, 0.03, 0.03)
    part = Part.from_stl(p)
    assert part.triangles == 12
    assert [round(e, 6) for e in part.extents_m] == [0.05, 0.03, 0.03]
    heights = sorted(round(r.height_m, 6) for r in part.resting)
    assert heights == [0.03, 0.03, 0.05]
    lib = PartLibrary([part])
    m = lib.match((0.05, 0.03), 0.03)
    assert m and m["pose"]["height_m"] == pytest.approx(0.03)
    m = lib.match((0.03, 0.03), 0.05)
    assert m and m["pose"]["height_m"] == pytest.approx(0.05)
    assert lib.match((0.08, 0.03), 0.03) is None


def test_obb_is_exact_for_rotated_box(tmp_path):
    p = write_box_stl(tmp_path / "b.stl", 0.05, 0.03, 0.02)
    from perception.partlib import read_stl

    tri = read_stl(p)
    rot = Transform.from_pose([0, 0, 0, 0.4, 0.7, -0.2]).rotation
    tri = tri @ np.asarray(rot).T
    part = Part.from_mesh(tri, name="rot")
    assert [round(e, 6) for e in part.extents_m] == [0.05, 0.03, 0.02]


# ----- table plane -------------------------------------------------------------


def test_plane_from_points_orientation_and_intersection():
    pl = Plane.from_points([0, 0, 0.1], [1, 0, 0.1], [0, 1, 0.1])
    assert pl.normal == pytest.approx((0, 0, 1))
    pl2 = Plane.from_points([0, 0, 0.1], [0, 1, 0.1], [1, 0, 0.1])  # reversed winding
    assert pl2.normal == pytest.approx((0, 0, 1))  # flipped to the up hint
    hit = pl.intersect([0.2, 0.3, 0.5], [0, 0, -1])
    assert hit == pytest.approx((0.2, 0.3, 0.1))
    assert pl.intersect([0, 0, 0.5], [0, 0, 1]) is None
    assert pl.height_of([0, 0, 0.13]) == pytest.approx(0.03)
    x, y = pl.to_plane_xy([0.4, -0.2, 0.1])
    assert pl.from_plane_xy(x, y, 0.02) == pytest.approx((0.4, -0.2, 0.12))


def test_plane_env_and_file(tmp_path, monkeypatch):
    monkeypatch.setenv("PERCEPTION_TABLE_Z", "0.012")
    assert Plane.from_env().point[2] == pytest.approx(0.012)
    monkeypatch.delenv("PERCEPTION_TABLE_Z")
    monkeypatch.setenv("PERCEPTION_TABLE_FILE", str(tmp_path / "t.json"))
    assert Plane.from_env() is None
    Plane.from_z(0.05).save(tmp_path / "t.json")
    assert Plane.from_env().point[2] == pytest.approx(0.05)


# ----- pose recorder -----------------------------------------------------------


def test_interpolate_pose_midpoint():
    a = [0, 0, 0, 0, 0, 0]
    b = [0.1, 0, 0, 0, 0, 1.0]
    m = interpolate_pose(a, b, 0.5)
    assert m[0] == pytest.approx(0.05)
    assert m[5] == pytest.approx(0.5, abs=1e-9)


def test_recorder_interpolates_and_applies_tcp_offset():
    offset = [0, 0, 0.2, 0, 0, 0]
    samples = []
    for i in range(5):
        flange = [0.1 * i, 0, 0.5, math.pi, 0, 0]
        samples.append(
            {"timestamp": i, "actual_TCP_pose": pose_trans(flange, offset), "actual_TCP_speed": [0.1] * 6}
        )
    t = [0.0]

    def clock():
        t[0] += 0.01
        return t[0]

    rec = PoseRecorder(tcp_offset=offset, client=ReplayClient(samples), clock=clock)
    rec.start(warmup_s=0.2)
    rec.stop()
    assert len(rec) == 5
    mid = rec.flange_at(0.025)  # between sample 2 (t=0.03)... samples at 0.01..0.05
    assert mid is not None
    assert mid[0] == pytest.approx(0.15, abs=1e-6)
    assert mid[2] == pytest.approx(0.5, abs=1e-6)  # flange, not the TCP 0.2 m further along -Z
    assert rec.flange_at(1.0) is None


def test_linear_motion_profile():
    m = LinearMotion([0, 0, 0, 0, 0, 0], [0.15, 0, 0, 0, 0, 0], velocity=0.15, acceleration=0.5, t0=10.0)
    assert m.fraction(9.0) == 0.0
    assert m.fraction(10.0 + m.duration + 1) == 1.0
    assert 0.0 < m.fraction(10.0 + m.duration / 2) < 1.0
    assert m.pose_at(10.0 + m.duration)[0] == pytest.approx(0.15)


# ----- synthetic sweep → locate ------------------------------------------------


@pytest.fixture(scope="module")
def fake_sweep():
    from perception.sweep import FakeRig

    rig = FakeRig(latency_s=0.03)
    sweep = rig.run(velocity=0.3, acceleration=1.5)
    return rig, sweep


def test_fake_sweep_records_frames_and_poses(fake_sweep):
    rig, sweep = fake_sweep
    s = sweep.summary()
    assert s["frames"] >= 8
    assert s["frames_with_pose"] == s["frames"]
    assert 0.12 < s["baseline_m"] <= 0.151


def test_locate_recovers_blocks(fake_sweep):
    from perception.locate2d import locate_objects
    from perception.sweep import DEFAULT_FAKE_SCENE

    rig, sweep = fake_sweep
    lib = PartLibrary.from_paths(["parts/block_50x30x30.stl"])
    objs = locate_objects(sweep.frame_pairs(), sweep.intrinsics, sweep.plane, library=lib)
    assert len(objs) == len(DEFAULT_FAKE_SCENE)
    for o in objs:
        truth = min(DEFAULT_FAKE_SCENE, key=lambda b: math.dist(b.center[:2], o.center_base_m[:2]))
        assert math.dist(truth.center[:2], o.center_base_m[:2]) < 0.002
        assert abs(truth.size[2] - o.height_m) < 0.003
        assert o.match is not None and o.match["pose"]["height_m"] == pytest.approx(truth.size[2])
        assert o.fit["iou"] > 0.95
        if abs(truth.size[0] - truth.size[1]) > 1e-6:
            dyaw = abs((o.yaw_base_rad - truth.yaw + math.pi / 2) % math.pi - math.pi / 2)
            assert dyaw < math.radians(2)


def test_wrong_latency_degrades_fit(fake_sweep):
    from perception.locate2d import locate_objects
    from perception.sweep import Sweep, SweepFrame

    rig, sweep = fake_sweep
    # re-stamp the same frames as if the latency were 0: poses from 30 ms later
    wrong = Sweep(
        [
            SweepFrame(f.gray, f.host_t, f.seq, rig.motion.pose_at(f.host_t), f.speed_mps)
            for f in sweep.frames
        ],
        sweep.intrinsics,
        sweep.flange_to_color,
        sweep.plane,
    )
    good = locate_objects(sweep.frame_pairs(), sweep.intrinsics, sweep.plane)
    bad = locate_objects(wrong.frame_pairs(), wrong.intrinsics, wrong.plane)
    assert min(o.fit["iou"] for o in bad) < min(o.fit["iou"] for o in good)


def test_sweep_save_load_roundtrip(fake_sweep, tmp_path):
    from perception.sweep import Sweep

    _, sweep = fake_sweep
    d = sweep.save(tmp_path / "s")
    back = Sweep.load(d)
    assert len(back.frames) == len(sweep.frames)
    assert back.frames[3].flange_pose == pytest.approx(sweep.frames[3].flange_pose)
    assert back.intrinsics.fx == pytest.approx(sweep.intrinsics.fx)
    assert back.plane is not None and back.plane.normal == pytest.approx(sweep.plane.normal)


def test_latency_fit_recovers_camera_delay():
    from perception.latency import fit_latency

    motion = LinearMotion(
        [0.2, 0, 0.35, math.pi, 0, 0], [0.15, 0, 0, 0, 0, 0], velocity=0.15, acceleration=0.5, t0=0.0
    )
    true_lag = 0.037
    measured = []
    t = 0.05
    while t < motion.duration + 0.1:
        measured.append((t + true_lag, motion.pose_at(t)[:3]))  # stamped late, position from the exposure
        t += 1 / 30
    fit = fit_latency(measured, lambda tt: motion.pose_at(tt)[:3])
    assert fit["latency_s"] == pytest.approx(true_lag, abs=0.002)
    assert fit["rms_m"] < 1e-4 and fit["rms_at_zero_m"] > 1e-3


def test_plate_from_touches_and_touchset(tmp_path):
    from perception.charuco import plate_from_touches

    t = plate_from_touches([0.3, 0.1, 0.02], [0.46, 0.1, 0.02], [0.3, 0.26, 0.02])
    assert t.translation == pytest.approx((0.3, 0.1, 0.02))
    assert t.rotate((0, 0, 1)) == pytest.approx((0, 0, 1))
    ts = TouchSet(probe_tcp=[0, 0, 0.05, 0, 0, 0])
    tip = ts.record("plate", "a", [0.3, 0.1, 0.07, math.pi, 0, 0])  # flange pointing down, tip 50 mm below
    assert tip == pytest.approx([0.3, 0.1, 0.02], abs=1e-9)
    with pytest.raises(ValueError):
        ts.record("plate", "D", [0] * 6)
    ts.record("table", "1", [0, 0, 0.05, math.pi, 0, 0])
    ts.record("table", "2", [0.2, 0, 0.05, math.pi, 0, 0])
    ts.record("table", "3", [0, 0.2, 0.05, math.pi, 0, 0])
    plane = ts.table_plane()
    assert plane is not None and plane.point[2] == pytest.approx(0.0, abs=1e-9)
    ts.save(tmp_path / "t.json")
    assert TouchSet.load(tmp_path / "t.json").table_plane() is not None


def test_cli_scan_fake_and_part_info(tmp_path, capsys):
    from perception.cli import main

    rc = main(
        [
            "scan",
            "--fake",
            "--out",
            str(tmp_path),
            "--velocity",
            "0.3",
            "--accel",
            "1.5",
            "--parts",
            "parts/block_50x30x30.stl",
        ]
    )
    out = json.loads(capsys.readouterr().out)
    assert rc == 0 and out["ok"]
    assert len(out["objects"]) == 3
    assert (tmp_path / "scan-001" / "sweep.json").is_file()
    rc = main(["locate", str(tmp_path / "scan-001"), "--parts", "parts/block_50x30x30.stl", "--no-refine"])
    out = json.loads(capsys.readouterr().out)
    assert rc == 0 and len(out["objects"]) == 3
    rc = main(["part-info", "parts/block_50x30x30.stl"])
    out = json.loads(capsys.readouterr().out)
    assert rc == 0 and out["parts"][0]["extents_m"] == pytest.approx([0.05, 0.03, 0.03])


def test_rtde_stream_yields_data_packages(monkeypatch):
    from urctl import rtde as urctl_rtde
    from urctl.config import RobotConfig

    client = urctl_rtde.RtdeClient(RobotConfig(), outputs=["timestamp"])

    class _Sock:
        def close(self):
            pass

    client._sock = _Sock()  # "connected"
    client._out_recipe = [("timestamp", "DOUBLE")]
    client._out_recipe_id = 1
    sent = []
    monkeypatch.setattr(client, "_send", lambda ptype, payload=b"": sent.append(ptype))
    monkeypatch.setattr(client, "_recv_control", lambda expected: b"\x01")
    import struct

    packages = iter(
        [
            (urctl_rtde._RTDE_TEXT_MESSAGE, b"noise"),
            (urctl_rtde._RTDE_DATA_PACKAGE, b"\x01" + struct.pack(">d", 1.5)),
            (urctl_rtde._RTDE_DATA_PACKAGE, b"\x01" + struct.pack(">d", 2.5)),
        ]
    )
    monkeypatch.setattr(client, "_recv_package", lambda: next(packages))
    got = []
    for s in client.stream():
        got.append(s["timestamp"])
        if len(got) == 2:
            client.close()
    assert got == [1.5, 2.5]
    assert sent[0] == urctl_rtde._RTDE_CONTROL_PACKAGE_START


# ----- the cockpit's scan (HTTP + MCP) ---------------------------------------------


@pytest.fixture
def scan_cockpit(tmp_path, monkeypatch):
    """A cockpit whose RGB-D camera and robot link share one fake rig."""
    import threading
    from http.server import ThreadingHTTPServer

    from perception.capture import CaptureStore
    from perception.config import PerceptionConfig
    from perception.robotlink import RobotLink
    from perception.sweep import FakeRig, FakeRigLink, SweepRgbdCamera
    from perception.webapp import ViewerApp, ViewerHandler
    from urctl.config import RobotConfig

    monkeypatch.chdir(tmp_path)
    (tmp_path / "parts").mkdir()
    write_box_stl(tmp_path / "parts" / "block.stl", 0.05, 0.03, 0.03)
    rig = FakeRig(latency_s=0.03)
    link = FakeRigLink(rig, RobotLink(RobotConfig(host="fake-ur"), dry_run=True))
    app = ViewerApp(
        SweepRgbdCamera(rig, fps=30),
        config=PerceptionConfig(),
        store=CaptureStore(tmp_path / "caps"),
        robot=link,
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield f"http://127.0.0.1:{srv.server_address[1]}", app, rig
    srv.shutdown()
    srv.server_close()
    app.stop()


def _post(base, path, body):
    import urllib.error
    import urllib.request

    req = urllib.request.Request(
        base + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"}
    )
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _get(base, path):
    import urllib.request

    with urllib.request.urlopen(base + path, timeout=10) as r:
        return r.status, json.loads(r.read())


def test_cockpit_table_scan_and_approach(scan_cockpit):
    from perception.sweep import DEFAULT_FAKE_SCENE

    base, app, rig = scan_cockpit
    status, j = _post(base, "/api/scan", {})
    assert status == 400 and "table plane" in j["error"]
    status, j = _post(base, "/api/table/from_depth", {"save": True})
    assert status == 200 and j["ok"], j
    assert abs(j["plane"]["point"][2]) < 0.002 and j["stats"]["tilt_from_base_z_deg"] < 1.0
    assert (Path("captures/calibration") / "table.json").is_file()
    status, j = _post(base, "/api/scan", {"velocity": 0.3, "acceleration": 1.5})
    assert status == 200 and j["ok"], j
    assert j["sweep"]["frames"] >= 6 and j["parts"] == ["block"]
    assert len(j["objects"]) == len(DEFAULT_FAKE_SCENE)
    for o in j["objects"]:
        truth = min(DEFAULT_FAKE_SCENE, key=lambda b: math.dist(b.center[:2], o["center_base_m"][:2]))
        assert math.dist(truth.center[:2], o["center_base_m"][:2]) < 0.003
        assert abs(truth.size[2] - o["height_m"]) < 0.004
        assert o["match"]["part"] == "block"
        dc = o["depth_check"]
        assert dc["ok"] is True and abs(dc["diff_mm"]) < 4.0, dc
    assert Path(j["sweep_dir"]).joinpath("sweep.json").is_file()
    status, st = _get(base, "/api/scan")
    assert st["last"]["objects"] == j["objects"] and st["running"] is False
    status, a = _post(base, "/api/scan/approach", {"index": 0, "standoff_m": 0.05, "reference": "flange"})
    assert status == 200 and a["ok"], a
    top = j["objects"][0]["center_base_m"]
    assert a["flange_target_pose"][2] == pytest.approx(top[2] + 0.05, abs=1e-6)
    assert a["flange_target_pose"][:2] == pytest.approx(top[:2], abs=1e-6)
    assert a["tcp"] == [0.0] * 6 and a["handeye"]["source"]
    status, a = _post(base, "/api/scan/approach", {"index": 9})
    assert status == 400
    kinds = [e["kind"] for e in app.events.since(0)]
    assert "scan" in kinds


def test_scan_tools_through_mcp(scan_cockpit):
    from perception.cockpitclient import CockpitClient
    from perception.mcp_server import CockpitTools

    base, app, rig = scan_cockpit
    tools = CockpitTools(CockpitClient(base))
    names = [t["name"] for t in tools.schemas()]
    assert {"cam_scan", "cam_scan_result", "cam_scan_approach", "cell_table_from_depth"} <= set(names)
    assert tools.call("cell_table_from_depth", {"save": False})["ok"]
    r = tools.call("cam_scan", {"velocity": 0.3})
    assert r["ok"] and len(r["objects"]) == 3
    assert tools.call("cam_scan_result")["last"]["objects"][0]["match"]["part"] == "block"
    a = tools.call("cam_scan_approach", {"index": 1, "reference": "tcp"})
    assert a["ok"] and a["reference"] == "tcp" and a["tcp"] is None


def test_realsense_infrared_stream_plumbing():
    from perception.monocam import RealSenseMono
    from perception.realsense import FORMAT_Y8, STREAM_INFRARED, RealSenseCamera
    from tests.test_realsense import COLOR_K, DEPTH_K, FakeApi, balanced

    class IrApi(FakeApi):
        def start_pipeline(self, ctx, *, infrared=False, **kw):
            self.log.append(f"infrared:{infrared}")
            return super().start_pipeline(ctx, **kw)

        def profile_streams(self, profile):
            return super().profile_streams(profile) + [
                {
                    "stream": STREAM_INFRARED,
                    "format": FORMAT_Y8,
                    "index": 1,
                    "uid": 2,
                    "fps": 30,
                    "intrinsics": DEPTH_K,
                }
            ]

        def split_frameset(self, frameset):
            frames = super().split_frameset(frameset)
            w, h = 64, 48
            frames.append(
                {
                    "stream": STREAM_INFRARED,
                    "format": FORMAT_Y8,
                    "width": w,
                    "height": h,
                    "stride": w,
                    "data": bytes(range(w)) * h,
                    "timestamp_ms": 10.0 * self.n,
                    "number": self.n,
                    "intrinsics": DEPTH_K,
                }
            )
            return frames

    api = IrApi()
    with RealSenseCamera(width=64, height=48, api=api, infrared=True) as cam:
        f = cam.read()
        assert f.extra["ir"].channels == 1 and f.extra["ir"].width == 64
        assert f.extra["ir_intrinsics"] == DEPTH_K and cam.intrinsics["infrared"] == DEPTH_K
        assert cam.describe()["infrared"] is True
    assert "infrared:True" in api.log and balanced(api)
    api = IrApi()
    mono = RealSenseMono(RealSenseCamera(width=64, height=48, api=api), stream="ir")
    with mono:
        mf = mono.read()
        assert mf.gray.shape == (48, 64) and mono.frame_kind == "depth" and mono.intrinsics == DEPTH_K
        assert mono.describe()["shutter"] == "global" and mono.last_rgbd is not None
    mono = RealSenseMono(RealSenseCamera(width=64, height=48, api=IrApi()))
    with mono:
        assert mono.frame_kind == "color" and mono.intrinsics == COLOR_K


def test_fit_latency_by_objects_recovers_delay():
    from perception.latency import fit_latency_by_objects
    from perception.sweep import FakeRig

    rig = FakeRig(latency_s=0.05)
    sweep = rig.run(velocity=0.3, acceleration=1.5)
    lib = PartLibrary.from_paths(["parts/block_50x30x30.stl"])
    fit = fit_latency_by_objects(sweep, lib, search_s=(0.0, 0.1), coarse_s=0.02, fine_s=0.005)
    assert abs(fit["latency_s"] - 0.05) <= 0.01
    assert fit["iou"] > fit["iou_at_zero"] + 0.1
    back = sweep.restamp(fit["latency_s"])
    assert back.meta["latency_s"] == fit["latency_s"] and len(back.trajectory) == len(sweep.trajectory)
