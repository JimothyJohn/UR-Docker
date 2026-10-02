"""The pick server's protocol 2 (the 0.5.0 Perceptronic Pick node): the part by its volume, the
pick order, the per-node queue that lets the next pick skip the picture point, the reach and
area limits, and the wire — against ray-cast scenes seen from a known flange pose."""

from __future__ import annotations

import math
import socket
import threading

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from perceptronics.partspec import PartSpec
from perceptronics.picknode import (
    PROTO2_FIELDS,
    QUEUE_TTL_S,
    PickPlanner,
    PickServer,
    RequestError,
    parse_options,
    parse_request,
    scene_report,
)
from perceptronics.synthscene import Box, render_depth
from urctl.pose import Transform, pose_trans

W, H = 320, 180
K = {"fx": 230.0, "fy": 230.0, "ppx": W / 2, "ppy": H / 2}
TABLE = -0.27
TIP = 0.163
FLANGE = [0.35, 0.0, 0.12, math.pi, 0.0, 0.0]  # tool straight down; flange X = base X, Y = -base Y
EYE = [0.0] * 6  # hand-eye: the colour camera is the flange frame
ROW = [Box(0.28 + 0.07 * c, -0.03, 0.050, 0.035, 0.030, 0.0) for c in range(3)]
OPTS = "part=50x35x30 tol=25 order=LR,FB proto=2"


class Frames:
    """A camera over a scene that can change; counts the frames taken."""

    def __init__(self, boxes, flange=FLANGE):
        self.set(boxes, flange)
        self.seq = 0
        self.taken = 0

    def set(self, boxes, flange=FLANGE):
        self.depth = render_depth(W, H, K, Transform.from_pose(flange), boxes, table_z=TABLE)

    def source(self, after):
        self.seq = max(self.seq, after) + 1
        self.taken += 1
        return self.seq, W, H, 3, bytes(W * H * 3), self.depth, 0.001, K


def planner(frames, **kw):
    return PickPlanner(frames.source, lambda: frames.seq, lambda: EYE, tip_m=TIP, **kw)


def pose_text(p):
    return "p[" + ", ".join(f"{v:.6f}" for v in p) + "]"


def ask(p: PickPlanner, verb: str, extra: str = "", flange=FLANGE) -> dict:
    text = p.answer(f"{verb} {pose_text(flange)} {extra}")
    assert text.startswith("(") and text.endswith(")\n")
    v = [float(x) for x in text[1:-2].split(",")]
    assert len(v) == PROTO2_FIELDS
    keys = ("status", "centre", "pose", "loc", "order", "remaining", "dims")
    return dict(
        zip(keys, (int(v[0]), v[1:4], v[4:10], int(v[10]), int(v[11]), int(v[12]), v[13:16]), strict=True)
    )


# -- FIND, NEXT --------------------------------------------------------------------------------


def test_find_answers_the_first_in_order_and_queues_the_rest():
    f = Frames(ROW)
    p = planner(f)
    r = ask(p, "FIND", f"node=a1 loc=2 locs=3 {OPTS}")
    assert r["status"] == 1 and r["loc"] == 2 and r["order"] == 1 and r["remaining"] == 2
    # the leftmost on the picture: the flange's (camera's) X is base X here
    assert r["centre"][0] == pytest.approx(0.28, abs=0.004)
    assert r["dims"] == pytest.approx([50, 35, 30], abs=5)
    # the fingertips on the top centre, the tool straight down
    tips = pose_trans(r["pose"], [0, 0, TIP, 0, 0, 0])
    assert tips[:3] == pytest.approx(r["centre"], abs=1e-6)
    z = Transform.from_pose(r["pose"]).rotate((0, 0, 1))
    assert z[2] == pytest.approx(-1.0, abs=1e-6)
    # the Hand-E's fingers travel along flange Y: across the part's short side (base Y here)
    y = Transform.from_pose(r["pose"]).rotate((0, 1, 0))
    assert abs(y[1]) == pytest.approx(1.0, abs=0.05)


def test_next_serves_the_queue_without_a_picture_then_sends_the_arm_back_to_look_again():
    f = Frames(ROW)
    p = planner(f)
    ask(p, "FIND", f"node=a1 loc=2 locs=3 {OPTS}")
    taken = f.taken
    second = ask(p, "NEXT", "node=a1 locs=3 proto=2")
    third = ask(p, "NEXT", "node=a1 locs=3 proto=2")
    assert (second["status"], second["order"], second["remaining"]) == (1, 2, 1)
    assert (third["status"], third["order"], third["remaining"]) == (1, 3, 0)
    assert third["centre"][0] == pytest.approx(0.42, abs=0.004)
    assert f.taken == taken  # no frame: the arm goes straight to the close look
    empty = ask(p, "NEXT", "node=a1 locs=3 proto=2")
    assert (empty["status"], empty["loc"]) == (0, 2)  # look at location 2 again: picks uncover parts


def test_an_empty_location_sends_the_arm_on_to_the_next_one_and_wraps():
    f = Frames([])
    p = planner(f)
    assert ask(p, "NEXT", "node=b locs=3 proto=2")["loc"] == 1  # a new node starts at 1
    assert ask(p, "FIND", f"node=b loc=1 locs=3 {OPTS}")["loc"] == 2
    assert ask(p, "NEXT", "node=b locs=3 proto=2")["loc"] == 2
    assert ask(p, "FIND", f"node=b loc=3 locs=3 {OPTS}")["loc"] == 1


def test_a_queue_goes_stale():
    now = {"t": 1000.0}
    f = Frames(ROW)
    p = planner(f, clock=lambda: now["t"])
    ask(p, "FIND", f"node=c loc=1 locs=1 {OPTS}")
    now["t"] += QUEUE_TTL_S + 1
    assert ask(p, "NEXT", "node=c locs=1 proto=2")["status"] == 0


def test_two_nodes_never_share_a_queue():
    f = Frames(ROW)
    p = planner(f)
    ask(p, "FIND", f"node=left loc=1 locs=1 {OPTS}")
    assert ask(p, "NEXT", "node=right locs=1 proto=2")["status"] == 0
    assert ask(p, "NEXT", "node=left locs=1 proto=2")["status"] == 1


def test_parallel_nexts_hand_each_part_out_exactly_once():
    boxes = [Box(0.27 + 0.045 * c, -0.07 + 0.07 * r, 0.03, 0.02, 0.03) for r in range(3) for c in range(4)]
    f = Frames(boxes)
    p = planner(f)
    first = ask(p, "FIND", "node=q loc=1 locs=1 part=30x20x30 order=LR,FB proto=2")
    assert first["status"] == 1 and first["remaining"] >= 8
    got, lock = [], threading.Lock()

    def worker():
        for _ in range(6):
            r = ask(p, "NEXT", "node=q locs=1 proto=2")
            if r["status"] == 1:
                with lock:
                    got.append(r["order"])

    threads = [threading.Thread(target=worker) for _ in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert sorted(got) == list(range(2, first["remaining"] + 2))  # no part twice, none lost


def test_the_order_option_changes_which_part_comes_first():
    f = Frames(ROW)
    lr = ask(planner(f), "FIND", f"node=o loc=1 locs=1 {OPTS}")
    rl = ask(planner(f), "FIND", f"node=o loc=1 locs=1 {OPTS.replace('LR,FB', 'RL,FB')}")
    assert lr["centre"][0] == pytest.approx(0.28, abs=0.004)
    assert rl["centre"][0] == pytest.approx(0.42, abs=0.004)


# -- why nothing -------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("boxes", "extra", "status"),
    [
        ([Box(0.35, -0.03, 0.08, 0.05, 0.03)], "", -7),  # not the part's size
        ([Box(0.35, -0.03, 0.05, 0.035, 0.03)], "reach=0.214,0.30", -10),  # 0.35 m out: past 0.30
        ([Box(0.35, -0.03, 0.05, 0.035, 0.03)], "reach=0.40,0.60", -2),  # inside the inner limit
        ([Box(0.35, -0.03, 0.05, 0.035, 0.03)], "stroke=30", -1),  # the jaws open 30 mm
    ],
)
def test_nothing_pickable_says_why(boxes, extra, status):
    r = ask(planner(Frames(boxes)), "FIND", f"node=w loc=1 locs=1 {OPTS} {extra}")
    assert r["status"] == status


def test_a_taught_plane_and_area_bound_the_pick():
    # the plane at the table, its area covering x 0.25..0.32 only: the middle and right parts are out
    plane = f"plane=p[0.25, -0.10, {TABLE}, 0, 0, 0] area=70x200"
    r = ask(planner(Frames(ROW)), "FIND", f"node=a loc=1 locs=1 {OPTS} {plane}")
    assert r["status"] == 1 and r["remaining"] == 0 and r["centre"][0] == pytest.approx(0.28, abs=0.004)


def test_refine_finds_the_part_again_from_the_close_look():
    f = Frames(ROW)
    p = planner(f)
    first = ask(p, "FIND", f"node=r loc=1 locs=1 {OPTS}")
    near = pose_text(first["centre"] + [0, 0, 0])
    close = [0.28, -0.03, 0.05, math.pi, 0, 0]
    f.set(ROW, close)
    got = ask(p, "REFINE", f"{near} node=r loc=1 locs=1 {OPTS}", flange=close)
    assert got["status"] == 1 and math.dist(got["centre"], first["centre"]) < 0.004


def test_a_part_gone_at_the_close_look_empties_the_queue():
    f = Frames(ROW)
    p = planner(f)
    first = ask(p, "FIND", f"node=g loc=1 locs=1 {OPTS}")
    f.set(ROW[1:])  # someone took the first part
    near = pose_text(first["centre"] + [0, 0, 0])
    assert ask(p, "REFINE", f"{near} node=g loc=1 locs=1 {OPTS}")["status"] == -5
    assert ask(p, "NEXT", "node=g locs=1 proto=2")["status"] == 0  # look again, don't trust the rest


def test_no_hand_eye_no_frame():
    f = Frames(ROW)
    no_eye = PickPlanner(f.source, lambda: f.seq, lambda: None, tip_m=TIP)
    assert ask(no_eye, "FIND", f"node=e loc=1 locs=1 {OPTS}")["status"] == -3
    no_frame = PickPlanner(lambda after: None, lambda: 0, lambda: EYE, tip_m=TIP)
    assert ask(no_frame, "FIND", f"node=e loc=1 locs=1 {OPTS}")["status"] == -4


# -- the request line, adversarially -----------------------------------------------------------


def test_options_parse_to_what_the_node_meant():
    o = parse_options(
        "FIND p[0,0,0,0,0,0] part=60x40x30 tol=20 plane=p[0.2, -0.1, -0.27, 0, 0, 0.3] area=300x200 "
        "order=RL,BF reach=0.214,0.350 grip=12 stroke=50 node=7f3a loc=2 locs=4 proto=2"
    )
    assert o.part == PartSpec.from_mm(60, 40, 30, 20)
    assert o.surface.source == "taught" and o.surface.area == pytest.approx((0, 0.3, 0, 0.2))
    assert o.order == ("RL", "BF") and o.reach.min_m == 0.214 and o.reach.max_m == 0.35
    assert (o.grip_below_m, o.stroke_m, o.node, o.loc, o.locs, o.proto) == (0.012, 0.05, "7f3a", 2, 4, 2)
    # the plane's pose is not taken for the flange
    req = parse_request("FIND plane=p[9, 9, 9, 0, 0, 0] p[0.4, 0, 0.3, 0, 3.14, 0] proto=2 node=x")
    assert req["flange"][0] == 0.4


@pytest.mark.parametrize(
    "bad",
    [
        "order=LR,RL",
        "order=XX,YY",
        "reach=0.5,0.2",
        "grip=90",
        "stroke=2",
        "locs=0",
        "loc=99",
        "proto=7",
        "plane=p[1,2]",
        "area=1x1 plane=p[0,0,0,0,0,0]",
        "plane=p[1e9, 0, 0, 0, 0, 0]",
    ],
)
def test_bad_options_are_refused_and_answered_in_protocol_2(bad):
    with pytest.raises(RequestError):
        parse_options(f"FIND p[0,0,0,0,0,0] {bad}")
    if bad.startswith("proto"):
        return  # a line that says proto=7 is not a protocol-2 line to answer
    text = planner(Frames([])).answer(f"FIND p[0,0,0,0,0,0] node=z proto=2 {bad}")
    assert len(text[1:-2].split(",")) == PROTO2_FIELDS and float(text[1:-2].split(",")[0]) == -9


def test_next_needs_a_node():
    with pytest.raises(RequestError, match="node"):
        parse_request("NEXT p[0,0,0,0,0,0] proto=2")


def test_a_node_id_cannot_smuggle_anything():
    o = parse_options('FIND p[0,0,0,0,0,0] node=../../etc";popup("x proto=2')
    assert o.node == ""  # not an id: ignored, never echoed into a path or a script


@settings(max_examples=200)
@given(st.text(max_size=200))
def test_any_line_is_answered_or_refused_never_crashes(line):
    p = planner(Frames([]))
    text = p.answer(line)
    assert text == "" or (text.startswith("(") and text.endswith(")\n"))


# -- the teach screen --------------------------------------------------------------------------


def test_the_teach_screen_sees_what_find_would_numbered_and_outlined():
    f = Frames(ROW + [Box(0.35, 0.05, 0.068, 0.035, 0.03)])
    p = planner(f)
    out = scene_report(p, FLANGE, parse_options(f"{OPTS} reach=0.214,0.45"))
    assert out["ok"] and out["base_frame"] and out["status"] == 1
    assert [q["order"] for q in out["parts"]] == [1, 2, 3]
    assert all(len(q["corners_px"]) == 4 for q in out["parts"])
    assert [q["why"] for q in out["rejected"]] == ["too long"]
    lefts = [q["pixel"][0] for q in out["parts"]]
    assert lefts == sorted(lefts)  # 1 2 3 left to right on the pendant's picture


def test_the_teach_screen_without_a_robot_pose_still_shows_the_parts():
    out = scene_report(planner(Frames(ROW)), None, parse_options(OPTS))
    assert out["ok"] and not out["base_frame"] and len(out["parts"]) == 3
    assert any("no live robot pose" in n for n in out["notes"])


# -- the wire ----------------------------------------------------------------------------------


def test_a_controller_session_over_a_real_socket():
    f = Frames(ROW)
    server = PickServer("127.0.0.1", 0, planner(f))
    server.start()
    try:
        with socket.create_connection(server.server_address, timeout=5) as s:
            rf = s.makefile("rb")
            s.sendall(b"LOG start\n")  # never answered
            s.sendall(f"NEXT {pose_text(FLANGE)} node=w1 locs=2 proto=2\n".encode())
            a = rf.readline().decode()
            s.sendall(f"FIND {pose_text(FLANGE)} node=w1 loc=1 locs=2 {OPTS}\n".encode())
            b = rf.readline().decode()
        assert float(a[1:-2].split(",")[0]) == 0 and float(a[1:-2].split(",")[10]) == 1
        vals = [float(v) for v in b[1:-2].split(",")]
        assert len(vals) == PROTO2_FIELDS and vals[0] == 1 and vals[11] == 1 and vals[12] == 2
    finally:
        server.stop()


def test_the_cockpit_scene_route_answers_the_teach_screen_over_http():
    import json
    import time
    import urllib.error
    import urllib.parse
    import urllib.request
    from http.server import ThreadingHTTPServer

    from perceptronics.config import PerceptionConfig
    from perceptronics.synthscene import BoxSceneCamera
    from perceptronics.webapp import ViewerApp, ViewerHandler

    app = ViewerApp(BoxSceneCamera(ROW, Transform.from_pose(FLANGE), w=W, h=H), config=PerceptionConfig())
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    try:
        deadline = time.monotonic() + 5
        while app.latest()[1] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        base = f"http://127.0.0.1:{srv.server_address[1]}/api/pick/scene?opts="
        out = json.load(urllib.request.urlopen(base + urllib.parse.quote(OPTS), timeout=10))
        assert out["ok"] and [p["order"] for p in out["parts"]] == [1, 2, 3]
        with pytest.raises(urllib.error.HTTPError) as bad:
            urllib.request.urlopen(base + urllib.parse.quote("order=LR,RL"), timeout=10)
        assert bad.value.code == 400
    finally:
        srv.shutdown()
        srv.server_close()
        app.stop()


def test_check_approach_gets_the_approach_in_polyscopes_active_tcp():
    import time

    from perceptronics.config import PerceptionConfig
    from perceptronics.synthscene import BoxSceneCamera
    from perceptronics.webapp import ViewerApp

    offset = [0.0, 0.0, 0.10, 0.0, 0.0, 0.0]  # the pendant's active TCP: 100 mm out along the flange Z

    class Eye:
        def as_dict(self):
            return {"flange_to_color_pose": EYE}

    class Link:  # the parts of RobotLink the scene route reads
        handeye = Eye()
        tip_m = TIP

        def flange_pose(self):
            return {"ok": True, "flange": FLANGE, "tcp_offset": offset, "tcp_offset_consistent": True}

    app = ViewerApp(BoxSceneCamera(ROW, Transform.from_pose(FLANGE), w=W, h=H), config=PerceptionConfig())
    app.start()
    try:
        deadline = time.monotonic() + 5
        while app.latest()[1] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        app.robot = Link()  # after the camera loop is up: it needs nothing of the robot
        out = app.pick_scene(OPTS, 25.0)
    finally:
        app.stop()
    assert out["ok"], out
    first = out["parts"][0]
    tips = pose_trans(first["grasp_pose"], [0, 0, TIP, 0, 0, 0])
    assert tips[:3] == pytest.approx(first["centre"], abs=1e-3)
    # the active TCP at the approach = the fingertips 25 mm over the top, backed off to where
    # a 100 mm TCP sits: 163 - 100 + 25 = 88 mm above the top, straight down
    tcp = first["polyscope_approach_pose"]
    assert tcp[2] == pytest.approx(first["centre"][2] + TIP - 0.10 + 0.025, abs=1e-3)
    assert tcp[:2] == pytest.approx(first["centre"][:2], abs=1e-3)


# -- 0.7.0 (2026-09-30): the arm's kinematics decide reach, the grip check can be off, cylinders --

FAR = [0.55, 0.0, 0.12, math.pi, 0.0, 0.0]  # a picture point over the far end of the table
ONE = [Box(0.35, -0.03, 0.05, 0.035, 0.03)]


def test_a_part_the_arm_has_no_joint_solution_for_is_out_of_reach_and_a_bigger_arm_picks_it():
    part = [Box(0.70, 0.0, 0.05, 0.035, 0.03)]
    ur3 = ask(planner(Frames(part, FAR)), "FIND", f"node=k loc=1 locs=1 {OPTS} arm=UR3", flange=FAR)
    assert ur3["status"] == -10
    ur10 = ask(planner(Frames(part, FAR)), "FIND", f"node=k loc=1 locs=1 {OPTS} arm=UR10", flange=FAR)
    assert ur10["status"] == 1 and ur10["centre"][0] == pytest.approx(0.70, abs=0.004)


def test_no_ring_is_drawn_round_a_part_the_arm_reaches():
    """0.6.0 sent reach=<base + 150 mm>,<rated reach - 150 mm>: 0.35 m for a UR3e, which left
    out every part past it. The same part, the arm named instead: its kinematics say yes."""
    part = [Box(0.42, -0.03, 0.05, 0.035, 0.03)]
    ring = ask(planner(Frames(part)), "FIND", f"node=r loc=1 locs=1 {OPTS} reach=0.214,0.350")
    assert ring["status"] == -10
    arm = ask(planner(Frames(part)), "FIND", f"node=r loc=1 locs=1 {OPTS} arm=UR3")
    assert arm["status"] == 1


def test_the_base_keeps_its_keep_out_radius():
    near = [0.20, 0.0, 0.12, math.pi, 0.0, 0.0]
    part = [Box(0.18, 0.0, 0.05, 0.035, 0.03)]  # 180 mm out: inside the UR3e's 64 + 150 mm
    assert ask(planner(Frames(part, near)), "FIND", f"node=b loc=1 locs=1 {OPTS} arm=UR3e", flange=near)[
        "status"
    ] == (-2)


@pytest.mark.parametrize("arm", ["UR30", "Fanuc"])
def test_an_arm_nobody_here_has_kinematics_for_is_left_to_the_controller(arm):
    part = [Box(0.70, 0.0, 0.05, 0.035, 0.03)]
    got = ask(planner(Frames(part, FAR)), "FIND", f"node=u loc=1 locs=1 {OPTS} arm={arm}", flange=FAR)
    assert got["status"] == 1


def test_without_the_grip_check_a_part_that_measures_too_wide_for_the_fingers_is_still_picked():
    """Nick, 2026-09-30: the camera reads the part wide, which made it look ungrippable — the
    part's size is known, so the measured width must not veto it."""
    wide = [Box(0.35, -0.03, 0.055, 0.047, 0.03)]
    opts = "part=55x47x30 tol=25 order=LR,FB proto=2"
    assert ask(planner(Frames(wide)), "FIND", f"node=g loc=1 locs=1 {opts}")["status"] == -1
    assert ask(planner(Frames(wide)), "FIND", f"node=g loc=1 locs=1 {opts} gripcheck=1")["status"] == -1
    assert ask(planner(Frames(wide)), "FIND", f"node=g loc=1 locs=1 {opts} gripcheck=0")["status"] == 1


def test_without_the_grip_check_a_crowded_part_is_still_picked():
    crowd = [Box(0.35, -0.03, 0.05, 0.035, 0.03), Box(0.35, 0.015, 0.05, 0.035, 0.03)]  # 10 mm apart
    assert ask(planner(Frames(crowd)), "FIND", f"node=c loc=1 locs=1 {OPTS}")["status"] == -11
    got = ask(planner(Frames(crowd)), "FIND", f"node=c loc=1 locs=1 {OPTS} gripcheck=0")
    assert got["status"] == 1 and got["remaining"] == 1


def test_a_cylinder_is_found_by_its_diameter_and_gripped_without_turning_the_wrist():
    can = [Box(0.35, -0.03, 0.04, 0.04, 0.03, round=True)]
    cyl = "part=40x40x30 tol=25 shape=cyl order=LR,FB proto=2"
    got = ask(planner(Frames(can)), "FIND", f"node=y loc=1 locs=1 {cyl}")
    assert got["status"] == 1 and got["dims"][:2] == pytest.approx([40, 40], abs=5)
    assert got["centre"][:2] == pytest.approx([0.35, -0.03], abs=0.004)
    # the flange heading is the one the arm had: a disc has no short side to turn to
    assert got["pose"][3:] == pytest.approx(FLANGE[3:], abs=1e-6)
    # ... where the same blob taken for a box turns the fingers to whatever side the rectangle fitted
    turned = [Box(0.35, -0.03, 0.05, 0.035, 0.03, 0.6)]
    box = ask(planner(Frames(turned)), "FIND", f"node=y2 loc=1 locs=1 {OPTS}")
    assert box["pose"][3:] != pytest.approx(FLANGE[3:], abs=1e-2)
    # a box is not the cylinder: its long side gives it away
    long_box = [Box(0.35, -0.03, 0.07, 0.035, 0.03)]
    assert ask(planner(Frames(long_box)), "FIND", f"node=y3 loc=1 locs=1 {cyl}")["status"] == -7


@pytest.mark.parametrize(
    "bad",
    ["gripcheck=2", "gripcheck=yes", "arm=", "arm=UR3;stop", "shape=hex", "shape=", "approach=900"],
)
def test_the_new_options_are_refused_when_malformed(bad):
    with pytest.raises(RequestError):
        parse_options(f"FIND p[0,0,0,0,0,0] part=50x50x30 {bad}")


def test_a_cylinder_with_two_different_sides_is_refused():
    with pytest.raises(RequestError):
        parse_options("FIND p[0,0,0,0,0,0] part=50x40x30 shape=cyl")


def test_the_new_options_parse_to_what_the_node_meant():
    o = parse_options("part=40x40x30 tol=20 shape=cyl grip=12 approach=30 gripcheck=0 arm=UR3 proto=2")
    assert o.part == PartSpec.from_mm(40, 40, 30, 20, shape="cyl") and o.part.is_round
    assert (o.approach_m, o.grip_check, o.arm, o.reach) == (0.03, False, "UR3", None)
    old = parse_options(OPTS)  # a 0.6.0 node says none of it: its behaviour is unchanged
    assert (old.grip_check, old.arm, old.part.shape) == (True, "", "box")


def test_the_teach_screen_is_told_which_rejects_are_nearly_the_part():
    """The pendant draws only these: a part a little off its size or out of reach — not the
    clamp, the cable and everything else on the table."""
    scene = ROW + [
        Box(0.35, 0.05, 0.068, 0.035, 0.03),  # a little too long
        Box(0.17, 0.08, 0.05, 0.035, 0.09),  # nothing like the part: three times as tall
    ]
    out = scene_report(planner(Frames(scene)), FLANGE, parse_options(OPTS))
    assert {q["why"]: q["near"] for q in out["rejected"]} == {"too long": True, "too tall": False}
    assert all(q["near"] for q in out["parts"])
    # out of reach is always worth showing: it is the part
    far = scene_report(
        planner(Frames([Box(0.70, 0.0, 0.05, 0.035, 0.03)], FAR)), FAR, parse_options(f"{OPTS} arm=UR3")
    )
    assert [(q["why"], q["near"]) for q in far["rejected"]] == [("out of reach (no joint solution)", True)]


def test_the_cockpit_serves_the_depth_as_a_heatmap_png_the_same_way_as_the_colour(tmp_path):
    import time
    import urllib.error
    import urllib.request
    from http.server import ThreadingHTTPServer

    from perceptronics.config import PerceptionConfig
    from perceptronics.pngio import load_png
    from perceptronics.synthscene import BoxSceneCamera
    from perceptronics.webapp import ViewerApp, ViewerHandler

    app = ViewerApp(BoxSceneCamera(ROW, Transform.from_pose(FLANGE), w=W, h=H), config=PerceptionConfig())
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}/api/depth.png"
    try:
        with pytest.raises(urllib.error.HTTPError) as none_yet:
            urllib.request.urlopen(base, timeout=10)
        assert none_yet.value.code == 503  # no frame: the same answer the colour route gives
        app.start()
        deadline = time.monotonic() + 5
        while app.latest()[1] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        with urllib.request.urlopen(base + "?after=0&timeout_ms=2000", timeout=10) as r:
            seq, png = int(r.headers["X-Seq"]), r.read()
        assert seq >= 1
        (tmp_path / "heat.png").write_bytes(png)
        w, h, ch, rgb = load_png(str(tmp_path / "heat.png"))
        assert (w, h, ch) == (W // 2, H // 2, 3)
        # the table (0.39 m) and the parts' tops (0.36 m) are different colours; nothing is black
        table, top = rgb[0:3], rgb[3 * ((H // 4) * (W // 2) + W // 4) :][:3]
        colours = {bytes(rgb[i : i + 3]) for i in range(0, len(rgb), 3)}
        assert bytes(table) != b"\x00\x00\x00" and len(colours) >= 2 and bytes(top) in colours
        with pytest.raises(urllib.error.HTTPError) as bad:
            urllib.request.urlopen(base + "?after=x", timeout=10)
        assert bad.value.code == 400
    finally:
        srv.shutdown()
        srv.server_close()
        app.stop()


# -- 0.8.0 (2026-10-01): the node drives no gripper; finger room is the operator's number; either side --

ROOM = "part=50x35x30 tol=25 order=LR,FB gripcheck=1 proto=2"


def test_finger_room_is_the_clear_space_asked_for_on_each_side_of_the_part():
    # two parts side by side across their short sides, 10 mm of table between them
    crowd = [Box(0.35, -0.03, 0.05, 0.035, 0.03), Box(0.35, 0.015, 0.05, 0.035, 0.03)]
    assert ask(planner(Frames(crowd)), "FIND", f"node=m loc=1 locs=1 {ROOM} room=20")["status"] == -11
    got = ask(planner(Frames(crowd)), "FIND", f"node=m loc=1 locs=1 {ROOM} room=5")  # 5 mm is there
    assert got["status"] == 1 and got["remaining"] == 1
    # 30 mm apart: 20 mm of room on each side is there
    apart = [Box(0.35, -0.04, 0.05, 0.035, 0.03), Box(0.35, 0.025, 0.05, 0.035, 0.03)]
    assert ask(planner(Frames(apart)), "FIND", f"node=m loc=1 locs=1 {ROOM} room=20")["status"] == 1
    assert ask(planner(Frames(apart)), "FIND", f"node=m loc=1 locs=1 {ROOM} room=40")["status"] == -11
    # the check can still be switched off
    off = ROOM.replace("gripcheck=1", "gripcheck=0")
    assert ask(planner(Frames(crowd)), "FIND", f"node=m loc=1 locs=1 {off} room=20")["status"] == 1


def test_a_node_that_drives_no_gripper_has_no_stroke_to_refuse_a_wide_part_with():
    wide = [Box(0.35, -0.03, 0.09, 0.07, 0.03)]
    opts = "part=90x70x30 tol=25 order=LR,FB gripcheck=1 proto=2"
    assert (
        ask(planner(Frames(wide)), "FIND", f"node=s loc=1 locs=1 {opts}")["status"] == -1
    )  # 0.7.0: 50 mm stroke
    assert ask(planner(Frames(wide)), "FIND", f"node=s loc=1 locs=1 {opts} room=20")["status"] == 1


def test_a_box_is_gripped_across_its_short_side_or_its_long_one():
    part = [Box(0.35, -0.03, 0.06, 0.03, 0.03, 0.0)]  # the long side along base X
    opts = "part=60x30x30 tol=25 order=LR,FB gripcheck=1 room=20 proto=2"
    short = ask(planner(Frames(part)), "FIND", f"node=x loc=1 locs=1 {opts}")
    also_short = ask(planner(Frames(part)), "FIND", f"node=x loc=1 locs=1 {opts} across=short")
    long_ = ask(planner(Frames(part)), "FIND", f"node=x loc=1 locs=1 {opts} across=long")
    assert short["status"] == long_["status"] == 1 and short["pose"] == also_short["pose"]

    def finger_axis(pose):  # the Hand-E's fingers travel along the flange Y
        return Transform.from_pose(pose).rotate((0.0, 1.0, 0.0))

    a, b = finger_axis(short["pose"]), finger_axis(long_["pose"])
    assert abs(a[1]) == pytest.approx(1.0, abs=0.03)  # across the short side: along base Y
    assert abs(b[0]) == pytest.approx(1.0, abs=0.03)  # across the long side: along base X
    # the room is checked where the fingers will be: a neighbour off the part's END is in the
    # way of a long-side grip only
    end_on = part + [Box(0.405, -0.03, 0.03, 0.03, 0.03)]  # 10 mm past the end
    spec = "part=60x30x30 tol=10 order=LR,FB gripcheck=1 room=20 proto=2"
    assert ask(planner(Frames(end_on)), "FIND", f"node=x2 loc=1 locs=1 {spec}")["status"] == 1
    seen = scene_report(planner(Frames(end_on)), FLANGE, parse_options(f"{spec} across=long"))
    longest = max(seen["rejected"], key=lambda q: q["size_mm"][0])  # the part, not its neighbour
    assert seen["parts"] == [] and longest["why"].startswith("no room for a finger beside it")


@pytest.mark.parametrize(
    "bad", ["room=500", "room=-3", "room=", "across=diagonal", "across=", "across=long;x"]
)
def test_the_0_8_options_are_refused_when_malformed(bad):
    with pytest.raises(RequestError):
        parse_options(f"FIND p[0,0,0,0,0,0] part=50x30x30 {bad}")


def test_the_0_8_options_parse_and_older_nodes_are_unchanged():
    o = parse_options("part=50x30x30 gripcheck=1 room=20 across=long proto=2")
    assert (o.grip_check, o.room_m, o.across) == (True, 0.02, "long")
    assert o.fingers() == {"grasp_below_m": 0.015, "stroke_m": 0.05, "across": "long", "room_m": 0.02}
    old = parse_options(OPTS)
    assert (old.room_m, old.across) == (None, "short")
