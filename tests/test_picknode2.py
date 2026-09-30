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
