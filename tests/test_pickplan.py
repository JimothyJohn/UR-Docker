"""perceptronics.pickplan: the approach Nick specified (2026-09-27) — straight down the
base Z axis, fingers across the short side at 1.2x its width, fingertips 25 mm over
the top — and a blended program the controller will accept."""

from __future__ import annotations

import math

import pytest

from perceptronics import pickplan
from urctl.pose import Transform

HOME = [-0.23587, 0.18254, 0.22134, -2.51512, 0.17760, 0.09959]  # the ur3 cell's picture pose
TIP = 0.163


def block(cx, cy, top, major, minor, theta_deg, n=24):
    """The top face of a rectangular block: a grid of points, plus its sides lower down."""
    th = math.radians(theta_deg)
    u, w = (math.cos(th), math.sin(th)), (-math.sin(th), math.cos(th))
    pts = []
    for i in range(n):
        for j in range(n):
            a, b = (i / (n - 1) - 0.5) * major, (j / (n - 1) - 0.5) * minor
            pts.append([cx + a * u[0] + b * w[0], cy + a * u[1] + b * w[1], top])
            pts.append([cx + a * u[0] + b * w[0], cy + a * u[1] + b * w[1], top - 0.02])  # a side
    return pts


@pytest.mark.parametrize("theta", [-60.0, 0.0, 25.0, 89.0])
def test_rectangle_recovers_a_block(theta):
    r = pickplan.rectangle(pickplan.top_face(block(-0.22, 0.38, -0.268, 0.044, 0.027, theta)))
    assert r["centre"] == pytest.approx([-0.22, 0.38, -0.268], abs=1e-6)
    assert r["major_m"] == pytest.approx(0.044, rel=0.1) and r["minor_m"] == pytest.approx(0.027, rel=0.1)
    d = (math.degrees(r["theta"]) - theta + 90) % 180 - 90
    assert abs(d) < 1.0


def test_top_face_ignores_a_few_flying_pixels():
    pts = block(0.3, 0.0, -0.1, 0.04, 0.02, 0.0) + [[0.3, 0.0, 0.2]] * 5
    assert all(abs(p[2] + 0.1) < 0.007 for p in pickplan.top_face(pts))


def test_robotiq_position_matches_the_measured_reading():
    assert pickplan.robotiq_position(0.0276) == pytest.approx(114, abs=1)  # POS 114 ≈ 27.6 mm (09-25)
    assert pickplan.robotiq_position(0.05) == 0 and pickplan.robotiq_position(0.0) == 255
    assert pickplan.robotiq_position(1.0) == 0 and pickplan.robotiq_position(-1.0) == 255


def _tool(pose):
    T = Transform.from_pose(pose)
    return T, T.rotate((0.0, 0.0, 1.0)), T.rotate((0.0, 1.0, 0.0))


@pytest.mark.parametrize("theta", [-45.0, 10.0, 70.0])
@pytest.mark.parametrize("fancy", [False, True])
@pytest.mark.parametrize("pick", [False, True])
def test_the_approach_nick_asked_for(theta, fancy, pick):
    rect = pickplan.rectangle(pickplan.top_face(block(-0.22, 0.38, -0.268, 0.044, 0.024, theta)))
    p = pickplan.plan(rect, HOME, tip_m=TIP, pick=pick, fancy=fancy)
    T, z, fingers = _tool(p["approach"])
    assert z == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)  # inline with the base Z axis
    tip = [p["approach"][i] + TIP * z[i] for i in range(3)]
    assert tip == pytest.approx(
        [rect["centre"][0], rect["centre"][1], -0.268 + 0.025], abs=1e-6
    )  # 25 mm over the top
    minor_dir = (-math.sin(rect["theta"]), math.cos(rect["theta"]))
    assert abs(fingers[0] * minor_dir[0] + fingers[1] * minor_dir[1]) == pytest.approx(
        1.0, abs=1e-6
    )  # across the short side
    # 20 % wider, but never under 8 mm a side (a 24 mm block: 40 mm, not 28.8)
    # the jaws open the full stroke, every time (2026-09-28: "maximum slop")
    assert p["opening_m"] == pytest.approx(0.05)
    assert p["gripper_position"] == pickplan.robotiq_position(p["opening_m"])
    names = [leg["name"] for leg in p["legs"]]
    assert names[-1 if not pick else -5] == "approach"
    assert ("swing" in names and "flourish" in names) == fancy
    if pick:
        # the drop sequence: grasp, lift and show it, set it back where it was, let go, clear
        assert names[-4:] == ["grasp", "lift", "place", "clear"]  # (no home pose given here)
        grasp, lift, place, clear = p["legs"][-4:]
        assert grasp["gripper"] == "close" and place["gripper"] == "open" and lift.get("dwell_s", 0) > 0
        gz = grasp["pose"][2] - TIP  # the fingertips at the grasp
        assert gz == pytest.approx(-0.268 - 0.015, abs=1e-6)
        assert (
            place["pose"][:2] == pytest.approx(grasp["pose"][:2])
            and abs(place["pose"][2] - grasp["pose"][2]) < 0.002
        )
        assert clear["pose"][2] > place["pose"][2] + 0.05
    # every blend is under half its neighbouring segments, and never on the last leg's successor
    path = [HOME] + [leg["pose"] for leg in p["legs"]]
    for i, leg in enumerate(p["legs"]):
        b = leg.get("blend_m", 0.0)
        if b:
            assert b <= 0.45 * math.dist(path[i][:3], path[i + 1][:3]) + 1e-9
            assert b <= 0.45 * math.dist(path[i + 1][:3], path[i + 2][:3]) + 1e-9
    # the last stretch before the hover runs straight down the Z axis
    over = p["legs"][names.index("over")]["pose"]
    assert over[:2] == pytest.approx(p["approach"][:2], abs=1e-9) and over[2] > p["approach"][2]


def test_skip_drops_a_fancy_via_and_a_wide_block_does_not_fit():
    rect = pickplan.rectangle(pickplan.top_face(block(-0.22, 0.38, -0.268, 0.06, 0.045, 0.0)))
    p = pickplan.plan(rect, HOME, tip_m=TIP, fancy=True, skip=["flourish"])
    assert p["vias"] == ["swing", "over"]
    assert not p["fits"] and p["opening_m"] == pytest.approx(0.05)  # 45 mm x 1.2 > the 50 mm stroke


def test_slerp_is_the_shortest_arc():
    a, b = [0, 0, 0, 0.0, 0.0, 0.0], [0, 0, 0, 0.0, 0.0, 1.2]
    assert pickplan.slerp_rotvec(a, b, 0.5) == pytest.approx([0.0, 0.0, 0.6])
    assert pickplan.slerp_rotvec(a, b, 0.0) == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)


def test_the_cockpit_plans_and_dry_runs_a_pick(monkeypatch):
    import json
    import threading
    import time
    import urllib.request
    from http.server import ThreadingHTTPServer

    from perceptronics.config import PerceptionConfig
    from perceptronics.posestream import PoseStream
    from perceptronics.realsense import SyntheticRgbdCamera
    from perceptronics.robotlink import RobotLink
    from perceptronics.webapp import ViewerApp, ViewerHandler
    from urctl.config import RobotConfig

    monkeypatch.setenv("PERCEPTRONICS_HOME_POSE", ",".join(map(str, HOME)))
    cfg = RobotConfig(host="127.0.0.1")
    ps = PoseStream(cfg, history_s=60.0)
    down = [-0.2, 0.36, 0.3, math.pi, 0.0, 0.0]  # looking straight down from 0.3 m over base
    app = ViewerApp(
        SyntheticRgbdCamera(width=160, height=120, fps=0),
        config=PerceptionConfig(),
        robot=RobotLink(cfg, dry_run=True),
        pose_stream=ps,
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    stop = threading.Event()

    def feed():
        while not stop.is_set():
            ps.add(time.time(), down)
            time.sleep(0.01)

    threading.Thread(target=feed, daemon=True).start()

    def post(path, body):
        req = urllib.request.Request(
            base + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"}
        )
        try:
            return json.load(urllib.request.urlopen(req, timeout=10))
        except urllib.error.HTTPError as e:
            return json.load(e)

    try:
        app.start()
        deadline = time.monotonic() + 5
        while app.latest()[1] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        time.sleep(0.1)
        assert post("/api/robot/pick", {})["ok"] is False  # nothing clicked yet
        seg = post("/api/nearest", {"near_ratio": 1.2})
        assert seg["ok"] and seg["features"]
        # the synthetic disk is 80 mm across: too wide to pick, fine to approach
        wide = post("/api/robot/pick", {"plan_only": True, "pick": True})
        assert wide["ok"] is False and "too wide" in wide["error"]
        plan = post("/api/robot/pick", {"plan_only": True, "fancy": True})
        assert plan["ok"] and plan["plan_only"], plan
        assert [leg["name"] for leg in plan["legs"]] == ["swing", "flourish", "look", "over", "approach"]
        assert plan["opening_mm"] == 50.0  # the full stroke
        # the synthetic disks are not white blocks: the close look finds no block by identity
        # and the pick stops at the look instead of grabbing whatever lies at the old pixel
        # (on the UR3e, 2026-09-27, that was a patch of carpet)
        run = post("/api/robot/pick", {})
        assert run["ok"] is False and run["stage"] == "look" and "lost the target" in run["error"], run
        run = post("/api/robot/pick", {"look": False})
        assert run["ok"] and run["dry_run"], run
        # a stored object stands in for the click: no mask, and it need not be in view
        tgt = {"centre": [-0.205, 0.242, -0.27], "theta": 0.3, "major_m": 0.047, "minor_m": 0.028}
        plan = post("/api/robot/pick", {"target": tgt, "pick": True, "plan_only": True})
        assert (
            plan["ok"]
            and plan["rect"]["centre"] == pytest.approx(tgt["centre"])
            and plan["opening_mm"] == 50.0
        )
        for bad in (
            {"centre": [0, 0]},
            {"centre": [0, 0, 0], "major_m": 0.01, "minor_m": 0.02},
            {"centre": [0, "x", 0], "major_m": 0.04, "minor_m": 0.02},
            {"major_m": 0.04},
        ):
            r = post("/api/robot/pick", {"target": bad, "plan_only": True})
            assert r["ok"] is False and "target" in r["error"], (bad, r)
        run = post("/api/robot/pick", {"target": tgt, "pick": True, "look": False})
        assert run["ok"] and run["dry_run"] and run["rect"]["centre"] == pytest.approx(tgt["centre"])
        home = post("/api/robot/home", {})
        assert home["ok"] and home["pose"] == pytest.approx(HOME)
    finally:
        stop.set()
        srv.shutdown()
        srv.server_close()
        app.stop()


UR3_HANDEYE = [0.04332, 0.05955, 0.01279, 0.06471, 0.17204, -3.08350]  # flange → colour, the ur3 cell


@pytest.mark.parametrize("fancy", [False, True])
@pytest.mark.parametrize("pick", [False, True])
def test_the_close_look_puts_the_object_on_the_optical_axis(fancy, pick):
    rect = pickplan.rectangle(pickplan.top_face(block(-0.22, 0.38, -0.268, 0.044, 0.024, 30.0)))
    p = pickplan.plan(rect, HOME, tip_m=TIP, pick=pick, fancy=fancy, flange_to_color=UR3_HANDEYE, look_m=0.24)
    look = p["look"]
    assert [leg["name"] for leg in p["sweep"]][-1] == "look" and p["sweep"][-1]["pose"] == look
    cam = Transform.from_pose(look).compose(Transform.from_pose(UR3_HANDEYE))
    q = cam.inverse().apply(rect["centre"])  # the object's top centre in the camera frame
    assert q[0] == pytest.approx(0.0, abs=1e-9) and q[1] == pytest.approx(0.0, abs=1e-9)
    assert q[2] == pytest.approx(0.24, abs=1e-9)
    # a slight tilt (2026-09-28): 10 deg off vertical, the camera looking out from the base's side ...
    z = Transform.from_pose(look).rotate((0, 0, 1))
    assert math.degrees(math.acos(-z[2])) == pytest.approx(pickplan.LOOK_TILT_DEG, abs=1e-6)
    cz = cam.rotate((0, 0, 1))
    c = rect["centre"]
    assert (cz[0] * c[0] + cz[1] * c[1]) > 0
    # ... and the final approach is straight down the base Z all the same
    approach = next(leg for leg in p["final"] if leg["name"] == "approach")["pose"]
    assert Transform.from_pose(approach).rotate((0, 0, 1)) == pytest.approx((0, 0, -1), abs=1e-9)
    names = [leg["name"] for leg in p["final"]]
    assert names == (
        ["over", "approach", "grasp", "lift", "place", "clear"] if pick else ["over", "approach"]
    )
    for start, legs in ((HOME, p["sweep"]), (look, p["final"])):
        path = [start] + [leg["pose"] for leg in legs]
        assert "blend_m" not in legs[-1]
        for i, leg in enumerate(legs):
            b = leg.get("blend_m", 0.0)
            assert not b or (
                b
                <= 0.45
                * min(math.dist(path[i][:3], path[i + 1][:3]), math.dist(path[i + 1][:3], path[i + 2][:3]))
                + 1e-9
            )
    # the fingertips at the look stay well clear of the top (camera 0.24 m off it)
    tip_z = look[2] - TIP
    assert tip_z - (-0.268) > 0.04


def test_top_face_ignores_reflections_floating_above_the_part():
    """Nick, 2026-09-27: 'filter out the obvious outlier reflection pixels floating way
    above the surface' — a scattered spray up to 10 cm over the block, 15 % of the points."""
    import random

    random.seed(3)
    pts = block(0.3, 0.0, -0.1, 0.04, 0.02, 0.0)
    spray = [
        [0.3 + random.uniform(-0.03, 0.03), random.uniform(-0.03, 0.03), -0.1 + random.uniform(0.01, 0.10)]
        for _ in range(len(pts) * 15 // 100)
    ]
    top = pickplan.top_face(pts + spray)
    assert top and all(abs(p[2] + 0.1) < 0.007 for p in top)


def test_white_level_keeps_a_shadowed_block_and_drops_the_carpet():
    """The close look on the UR3e (2026-09-27): the block read (169, 180, 178) under the
    camera's and gripper's shadow — grey to the fixed 180 — while the slats read 238 and the
    carpet at most 64. The level is relative to the picture's brightest neutrals."""
    from perceptronics.pickcycle import WHITE_MIN, white_blobs, white_level

    w, h = 160, 120
    px = bytearray()
    for y in range(h):
        for x in range(w):
            if x < 30:
                px += bytes((238, 240, 239))  # the aluminium slats
            elif 70 <= x < 110 and 40 <= y < 80:
                px += bytes((166, 176, 172))  # the shadowed block
            else:
                px += bytes((95, 80, 60))  # carpet
    lv = white_level(w, h, 3, bytes(px))
    assert 110 <= lv < 160 and lv < WHITE_MIN
    blobs = white_blobs(w, h, 3, bytes(px), white_min=lv, min_px=100)
    assert any(75 <= b["cx"] <= 105 and 45 <= b["cy"] <= 75 for b in blobs)  # the block is found
    assert not white_blobs(w, h, 3, bytes(px), min_px=100)[1:]  # the fixed 180 only sees the slats


def test_a_28mm_block_gets_11mm_of_slop_a_side():
    """UR3e, 2026-09-27: at 1.2 x 28 mm = 34 mm (3.4 mm a side) a fingertip caught the
    block's edge and the Hand-E closed on nothing; fully open at the same spot it held.
    Since 2026-09-28 the jaws always open the full 50 mm."""
    rect = pickplan.rectangle(pickplan.top_face(block(-0.205, 0.242, -0.27, 0.047, 0.028, 0.0)))
    p = pickplan.plan(rect, HOME, tip_m=TIP, pick=True)
    assert (p["opening_m"] - rect["minor_m"]) / 2 >= 0.011 - 1e-3 and p["fits"]


@pytest.mark.parametrize("look", [False, True])
def test_a_leg_that_grips_or_dwells_is_never_blended(look):
    """UR3e, 2026-09-27 23:3x: every automated pick closed on nothing while a hand-driven
    grasp at the same spot held. The grasp leg (not the last — lift follows) carried a
    blend; a blended movel returns as the arm enters the blend and skips the sync(), so the
    close went out early and the fingers shut while the arm swept on into the lift."""
    rect = pickplan.rectangle(pickplan.top_face(block(-0.205, 0.242, -0.27, 0.047, 0.028, 0.0)))
    kw = {"flange_to_color": UR3_HANDEYE, "look_m": 0.24} if look else {}
    p = pickplan.plan(rect, HOME, tip_m=TIP, pick=True, fancy=True, **kw)
    for leg in p["sweep"] + p["final"]:
        if leg.get("gripper") or leg.get("dwell_s"):
            assert not leg.get("blend_m"), leg["name"]


def test_a_pick_flows_home_without_stopping_and_moves_fast_but_gently():
    """Nick, 2026-09-27: 'I don't mind a high velocity, I just don't want a high acceleration
    and ideally no stops or slowdowns during moves.'"""
    rect = pickplan.rectangle(pickplan.top_face(block(-0.205, 0.242, -0.27, 0.047, 0.028, 0.0)))
    p = pickplan.plan(rect, HOME, tip_m=TIP, pick=True, flange_to_color=UR3_HANDEYE, look_m=0.24, home=HOME)
    legs = p["sweep"] + p["final"]
    names = [leg["name"] for leg in legs]
    assert names[-5:] == ["grasp", "lift", "place", "clear", "home"]
    by = {leg["name"]: leg for leg in legs}
    assert by["clear"].get("blend_m", 0) > 0 and "blend_m" not in by["home"]  # clear flows into home
    assert by["approach"].get("blend_m", 0) > 0 and not by["approach"].get("dwell_s")  # one descent
    stops = [n for n in names if not legs[names.index(n)].get("blend_m")]
    assert set(stops) <= {"look", "grasp", "lift", "place", "home"}  # only where physics needs one
    # 23:55 "much faster, a safe demo space": fast transit, accels well under the 0.8 m/s^2 that thudded
    assert all(leg["acceleration"] <= 0.35 for leg in legs)
    assert max(leg["velocity"] for leg in legs) >= 0.6


def test_the_jaws_open_the_full_stroke_every_time():
    """Nick, 2026-09-28: 'open the jaws the full 50 mm every time to allow for maximum slop'."""
    for minor in (0.010, 0.024, 0.028, 0.040):
        rect = pickplan.rectangle(pickplan.top_face(block(-0.205, 0.242, -0.27, 0.047, minor, 0.0)))
        p = pickplan.plan(rect, HOME, tip_m=TIP, pick=True)
        assert p["opening_m"] == pytest.approx(0.05) and p["gripper_position"] == 0 and p["fits"]
    wide = pickplan.rectangle(pickplan.top_face(block(-0.205, 0.242, -0.27, 0.06, 0.049, 0.0)))
    assert not pickplan.plan(wide, HOME, tip_m=TIP, pick=True)["fits"]  # no slop left at 49 of 50 mm


def _floor(cx, cy, z=-0.30, half=0.08, step=0.004):
    n = int(2 * half / step)
    return [[cx - half + i * step, cy - half + j * step, z] for i in range(n) for j in range(n)]


def test_clearance_around_the_pick_zone():
    """Nick, 2026-09-28: 'add logic to ensure there's clearance around the target pick zone before
    committing'. The fingers come down just outside the block along their travel axis."""
    rect = pickplan.rectangle(pickplan.top_face(block(-0.205, 0.242, -0.27, 0.047, 0.028, 0.0)))
    scene = _floor(-0.205, 0.242) + block(-0.205, 0.242, -0.27, 0.047, 0.028, 0.0)
    ok = pickplan.clearance(rect, scene)
    assert ok["clear"] and ok["worst_mm"] is None
    # a neighbour 32 mm beside it along the finger travel (theta 0: the minor side runs along y)
    crowded = pickplan.clearance(rect, scene + block(-0.205, 0.242 + 0.032, -0.27, 0.047, 0.012, 0.0))
    assert not crowded["clear"] and crowded["side"] in ("+", "-") and crowded["worst_mm"] > 10
    # the same neighbour 80 mm away, or beside the long ends (not where a finger goes), is fine
    assert pickplan.clearance(rect, scene + block(-0.205, 0.242 + 0.080, -0.27, 0.047, 0.012, 0.0))["clear"]
    assert pickplan.clearance(rect, scene + block(-0.205 + 0.060, 0.242, -0.27, 0.030, 0.028, 0.0))["clear"]
    # a few flying pixels are not an obstacle; a real cluster is
    speck = [[-0.205, 0.242 + 0.031, -0.26]] * 5
    assert pickplan.clearance(rect, scene + speck)["clear"]


def test_a_wide_block_is_not_its_own_obstacle():
    """A block that measures ~46 mm still fits the 50 mm jaws (2 mm a side); its own edges sit
    inside the finger zone's conservative inner bound and must not read as a neighbour."""
    rect = pickplan.rectangle(pickplan.top_face(block(-0.205, 0.242, -0.27, 0.060, 0.044, 0.0)))
    assert rect["minor_m"] / 2 > pickplan.STROKE_M / 2 - 0.003  # its edge is past the old bound
    assert pickplan.plan(rect, HOME, tip_m=TIP, pick=True)["fits"]
    # as dense as the D435's depth at the close look (~1 mm a sample), so its edge rows count
    scene = _floor(-0.205, 0.242) + block(-0.205, 0.242, -0.27, 0.060, 0.044, 0.0, n=60)
    assert pickplan.clearance(rect, scene)["clear"]
