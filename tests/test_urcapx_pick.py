"""The PolyScope X Perceptronic Pick program node's contract, under node: the URScript
pickscript.js writes (the same script as the PolyScope 5 node's PickScript.java), the
request options it sends (read back by the Python pick server's own parser), the plane,
reach and pose math agreeing with the Python they stand for, and the two behavior
workers answering PolyScope's worker protocol with the shapes its serializers read."""

from __future__ import annotations

import json
import math
import re
import shutil
import subprocess
from functools import cache
from pathlib import Path

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from perceptronics.partspec import PartSpec
from perceptronics.picknode import STATUS, parse_options, parse_request
from perceptronics.volume import BASE_RADIUS_M, Reach, Surface
from urctl.pose import pose_trans
from urctl.safety import MODEL_REACH_M

ROOT = Path(__file__).resolve().parent.parent
FRONTEND = ROOT / "urcap" / "perceptronic" / "perceptronic-frontend"
NODE = shutil.which("node")
pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")

Q1 = [-1.37, -0.49, 1.61, -2.69, -1.57, 0.61]
Q2 = [-0.80, -0.60, 1.50, -2.50, -1.57, 0.90]
PLANE = [0.20, -0.10, -0.27, 0.0, 0.0, 0.3]
NODE_SETTINGS = {
    "host": "192.168.3.10",
    "node": "a1b2c3",
    "points": [{"q": Q1}, {"q": Q2, "plane": PLANE, "area": [300, 200]}],
}

# One node process per distinct request: it loads pickscript.js and answers with JSON.
HARNESS = r"""
globalThis.self = globalThis;
require(process.argv[2]);
const P = self.PerceptronicPick;
const req = JSON.parse(process.argv[3]);
const out = {};
if (req.cmd === "pick") {
  const values = P.values(req.values || {});
  const st = {
    host: req.host === undefined ? "192.168.3.10" : req.host,
    cockpitSet: req.host !== "",
    port: req.port === undefined ? P.DEFAULT_PICK_PORT : req.port,
    nodeId: req.node === undefined ? "a1b2c3" : req.node,
    points: (req.points || []).map((p) => ({
      joints: p.q, area: p.plane ? 0 : -1, plane: p.plane || null,
      areaXmm: p.area ? p.area[0] : 0, areaYmm: p.area ? p.area[1] : 0,
    })),
    values,
    orderFirst: req.order ? req.order[0] : "LR",
    orderRows: req.order ? req.order[1] : "FB",
    gripper: req.gripper || "robotiq",
    popupOnFail: req.popup !== false,
    reachMinM: req.reach ? req.reach[0] : 0,
    reachMaxM: req.reach ? req.reach[1] : 0,
    foundVariable: req.found || P.FOUND_VARIABLE,
    locVariable: req.loc || P.LOC_VARIABLE,
  };
  out.problem = P.problem(st);
  out.values = values;
  if (!out.problem) {
    out.tokens = [-1].concat(st.points.map((_, i) => i)).map((i) => P.tokens(st, i));
    const sc = P.script(st);
    out.before = sc.before; out.after = sc.after; out.childDepth = sc.childDepth;
    out.script = P.render(st, req.children ? req.children.split("\n") : null);
  }
} else if (req.cmd === "settings") {
  const st = P.settings(req.params, req.app, "http://localhost");
  out.settings = st; out.problem = P.problem(st);
  out.tokens = st.points.map((_, i) => P.tokens(st, i));
} else if (req.cmd === "plane") {
  out.plane = P.plane(req.p0, req.p1, req.p2);
  out.tilt = out.plane ? P.tiltDeg(out.plane) : null;
} else if (req.cmd === "reach") {
  out.reach = P.modelReach(req.model); out.limits = P.reachLimits(req.model, req.inner, req.outer);
} else if (req.cmd === "pose_trans") {
  out.pose = P.poseTrans(req.a, req.b); out.inv = P.poseInv(req.a);
} else if (req.cmd === "grid") {
  out.grid = P.orderGrid(req.first, req.rows, req.cols, req.nrows);
} else if (req.cmd === "corners") {
  out.corners = P.areaCorners(req.plane);
} else if (req.cmd === "svg") {
  out.svgs = {
    tile: P.svgOrderTile(req.first || "LR", req.rows || "FB", !!req.selected),
    part: P.svgPart(req.l, req.w, req.h),
    approach: P.svgApproach(req.approach, req.grip, req.lift, req.h, Math.min(req.l, req.w), req.stroke),
    map: P.svgReachMap(req.baseR, req.minR, req.maxR,
      (req.areas || []).map((a) => ({ name: a.name, corners: P.areaCorners(a.plane) })), 0),
  };
} else if (req.cmd === "misc") {
  out.reasons = P.REASONS.map((r) => r[0]);
  out.numbers = P.NUMBERS;
  out.clamped = Object.fromEntries((req.clamp || []).map(([k, v]) => [k, P.clamp(k, v)]));
  out.ids = [P.newNodeId(), P.newNodeId()];
  out.after = P.afterPictureScript(req.locName, req.point || 1);
  out.orders = P.ORDER_TILES.map(([a, b]) => P.isOrder(a, b));
  out.badOrders = [["LR", "RL"], ["FB", "BF"], ["XX", "FB"]].map(([a, b]) => P.isOrder(a, b));
  out.cockpit = (req.cockpit || []).map((v) => P.cockpitBase(v, "http://10.0.0.5"));
}
process.stdout.write(JSON.stringify(out));
"""


@cache
def _ask(request: str) -> dict:
    harness = ROOT / "target" / "urcapx-pick-harness.js"
    harness.parent.mkdir(exist_ok=True)
    harness.write_text(HARNESS, encoding="utf-8")
    proc = subprocess.run(
        [NODE, str(harness), str(FRONTEND / "pickscript.js"), request],
        capture_output=True,
        text=True,
        encoding="utf-8",  # node prints UTF-8 (×, ·); never the Windows codepage
        timeout=30,
        check=True,
    )
    return json.loads(proc.stdout)


def ask(**req) -> dict:
    return _ask(json.dumps(req, sort_keys=True))


def pick(**kw) -> dict:
    return ask(cmd="pick", **{**NODE_SETTINGS, **kw})


def body(text: str) -> list[str]:
    return [line.strip() for line in text.splitlines() if line.strip() and not line.strip().startswith("#")]


def balanced(text: str) -> bool:
    """Every if/elif/else/while block closed: URScript's own nesting rule."""
    depth = 0
    for line in body(text):
        head = line.split()[0]
        if head in ("if", "while") and line.endswith(":"):
            depth += 1
        elif line == "end":
            depth -= 1
            if depth < 0:
                return False
    return depth == 0


# -- the script ---------------------------------------------------------------------------------


def test_the_script_is_ascii_balanced_and_runs_in_the_documented_order():
    out = pick(children="AFTER_PICK()")
    assert out["problem"] is None
    text = out["script"]
    assert text.isascii() and balanced(text)
    order = [
        "global rs_pick_found = False",
        "set_tcp(p[0, 0, 0, 0, 0, 0])",
        'socket_open("192.168.3.10", 7622, "rs_pick")',
        '"NEXT "',
        "movej([-1.370000, -0.490000, 1.610000, -2.690000, -1.570000, 0.610000]",
        '"FIND "',
        '"LOOK "',
        "get_inverse_kin_has_solution(rs_look",
        '"REFINE "',
        "movel(rs_hover",
        "movel(rs_grip, a=0.3, v=0.05)",
        '"SET POS 255"',
        "movel(rs_lift",
        "global rs_pick_found = True",
        "global rs_pick_loc = rs_loc",
        'socket_close("rs_pick")',
        'socket_close("rs_rq")',
        "set_tcp(rs_tcp0)",
        "popup(",
        "if rs_pick_found:",
        "AFTER_PICK()",
    ]
    at = [text.index(marker) for marker in order]
    assert at == sorted(at), "the stages are out of order"
    # the children sit one level inside the final `if <found>:`
    assert "\nif rs_pick_found:\n  AFTER_PICK()\nend\n" in text
    assert out["childDepth"] == 1 and out["after"] == ["end"]
    # no request line the program builds can exceed the server's 1 kB
    assert all(len(line) < 900 for line in text.splitlines())


def test_every_picture_point_is_visited_by_its_own_joints_and_options():
    text = pick()["script"]
    assert text.count("elif rs_loc == 2:") == 2  # the options chain and the movej chain
    assert "movej([-0.800000, -0.600000, 1.500000, -2.500000, -1.570000, 0.900000]" in text
    toks = re.findall(r'rs_tok = "( [^"]+)"', text)
    assert len(toks) == 2 and "loc=1" in toks[0] and "loc=2" in toks[1] and "plane=" in toks[1]


def test_the_options_it_sends_are_what_the_pick_server_reads():
    values = {
        "partLengthMm": 42,
        "partWidthMm": 61,
        "partHeightMm": 27,
        "partTolPct": 20,
        "gripBelowTopMm": 12,
    }
    out = pick(values=values, order=["RL", "BF"], reach=[0.214, 0.35])
    for i, tok in enumerate(out["tokens"]):
        o = parse_options(tok)
        assert o.part == PartSpec.from_mm(61, 42, 27, 20)  # long side first, however it was typed
        assert o.order == ("RL", "BF") and o.reach == Reach(0.214, 0.35)
        assert (o.grip_below_m, o.stroke_m, o.node, o.locs, o.proto) == (0.012, 0.05, "a1b2c3", 2, 2)
        assert o.loc == i  # tokens(-1) (the teach screen with no point yet) carries no loc
    plane = parse_options(out["tokens"][2]).surface
    want = Surface.from_pose(PLANE, (0.3, 0.2))
    assert plane.origin == pytest.approx(want.origin, abs=1e-5) and plane.area == pytest.approx(want.area)
    assert parse_options(out["tokens"][1]).surface is None  # point 1 finds the table live


def test_every_request_line_it_can_send_parses():
    text = pick()["script"]
    pose = "p[0.3, -0.1, 0.12, 3.14159, 0, 0]"
    toks = re.findall(r'rs_tok = "( [^"]+)"', text)
    lines = [f"NEXT {pose} node=a1b2c3 locs=2 proto=2", f"LOOK {pose} p[0.3, 0, -0.24, 0, 0, 0]"]
    lines += [f"FIND {pose}{t}" for t in toks]
    lines += [f"REFINE {pose} p[0.3, 0, -0.24, 0, 0, 0]{t} lean=12" for t in toks]
    for line in lines:
        assert len(line) < 1024
        parse_request(line)  # raises on anything the server would refuse


def test_the_popup_names_every_status_the_server_can_send():
    codes = set(ask(cmd="misc")["reasons"])
    sent = {c for c in STATUS if c not in (1, -6)}  # -6 answers LOOK, which never fails a pick
    assert sent <= codes, sent - codes
    text = pick()["script"]
    for code in codes:
        assert f"rs_st == {code}:" in text


@pytest.mark.parametrize(
    ("kw", "problem"),
    [
        ({"host": ""}, "camera computer's address"),
        ({"host": "bad host"}, "not an address"),
        ({"port": 70000}, "pick port"),
        ({"node": "ZZ"}, "identity"),
        ({"points": []}, "add a picture point"),
        ({"points": [{"q": Q1}] * 13}, "at most 12"),
        ({"points": [{"q": [0, 0, 0]}]}, "not a joint position"),
        ({"points": [{"q": Q1, "plane": PLANE, "area": [2, 200]}]}, "pick area is broken"),
        ({"values": {"gripBelowTopMm": 40, "partHeightMm": 30}}, "fingertips on the table"),
        ({"values": {"partWidthMm": 48, "strokeMm": 50}}, "open wider"),
        ({"order": ["LR", "RL"]}, "pick order"),
        ({"gripper": "suction"}, "unknown gripper"),
        ({"reach": [0.5, 0.4]}, "reach limits"),
        ({"found": "1bad"}, "variable names"),
    ],
)
def test_it_refuses_what_it_cannot_generate_safely(kw, problem):
    out = pick(**kw)
    assert out["problem"] and problem in out["problem"], out["problem"]
    assert "script" not in out


def test_a_digital_output_gripper():
    text = pick(gripper="digital", values={"gripperDo": 3, "gripperWaitS": 0.7})["script"]
    assert "set_standard_digital_out(3, False)" in text and "set_standard_digital_out(3, True)" in text
    assert "sleep(0.70)" in text
    assert "63352" not in text and "rs_rq" not in text


def test_my_own_gripper_nodes_run_at_the_grip_with_the_operators_tcp():
    out = pick(gripper="children", children="CLOSE_MY_GRIPPER()")
    text = out["script"]
    assert out["childDepth"] == 4
    grip = text.index("movel(rs_grip")
    restore = text.index("set_tcp(rs_tcp0)", grip)
    child = text.index("CLOSE_MY_GRIPPER()")
    back = text.index("set_tcp(p[0, 0, 0, 0, 0, 0])", child)
    lift = text.index("movel(rs_lift")
    assert grip < restore < child < back < lift
    assert "\n        CLOSE_MY_GRIPPER()\n" in text  # four blocks deep, where the grip is
    assert "SET POS 255" not in text
    assert balanced(text) and text.rstrip().endswith("end")


def test_a_close_on_nothing_opens_and_tries_the_next_part():
    text = pick()["script"]
    i = text.index('rs_why = "the gripper closed on nothing"')
    assert '"SET POS 0"' in text[i : i + 400]
    assert "while (rs_ok) and (rs_try < 6) and (rs_pick_found == False):" in text  # 2 points + 3 attempts + 1


def test_without_the_popup_a_failed_run_just_leaves_the_result_false():
    text = pick(popup=False)["script"]
    assert "popup(" not in text and "global rs_pick_found = False" in text


def test_speed_scales_the_nodes_own_travel():
    slow = pick(values={"speedPct": 20})["script"]
    fast = pick(values={"speedPct": 100})["script"]
    assert "a=0.28, v=0.21)" in slow and "a=1.40, v=1.05)" in fast  # movej
    assert "movel(rs_hover, a=0.12, v=0.05)" in slow and "movel(rs_hover, a=0.60, v=0.25)" in fast
    assert "movel(rs_grip, a=0.3, v=0.05)" in slow and "movel(rs_grip, a=0.3, v=0.05)" in fast  # never faster


def test_every_default_is_within_its_own_limits_and_the_defaults_generate():
    numbers = ask(cmd="misc")["numbers"]
    assert len(numbers) == 16
    for n in numbers:
        assert n["min"] <= n["def"] <= n["max"], n["key"]
        assert n["step"] > 0
    assert pick()["problem"] is None


@pytest.mark.parametrize(
    ("key", "value", "stored"),
    [
        ("partLengthMm", 1e9, 500),
        ("partLengthMm", -3, 5),
        ("partTolPct", 27, 25),
        ("settleS", 0.33, 0.33),
        ("gripperWaitS", 9, 5),
        ("speedPct", 55, 60),
        ("maxAttempts", 2.6, 3),
        ("partWidthMm", "nope", 30),
    ],
)
def test_values_are_clamped_to_their_limits(key, value, stored):
    assert ask(cmd="misc", clamp=[[key, value]])["clamped"][key] == pytest.approx(stored)


def test_orders_are_one_horizontal_and_one_vertical_direction():
    out = ask(cmd="misc")
    assert out["orders"] == [True] * 8 and out["badOrders"] == [False, False, False]


# -- the application node's part: areas, reach, cockpit ------------------------------------------


@settings(max_examples=40, deadline=None)
@given(
    ox=st.floats(-0.6, 0.6),
    oy=st.floats(-0.6, 0.6),
    oz=st.floats(-0.5, 0.3),
    heading=st.floats(-math.pi, math.pi),
    sx=st.floats(0.05, 0.6),
    sy=st.floats(0.05, 0.6),
    tilt=st.floats(-0.03, 0.03),
)
def test_a_taught_plane_is_the_same_plane_in_javascript_and_python(ox, oy, oz, heading, sx, sy, tilt):
    ch, sh = math.cos(heading), math.sin(heading)
    p0 = [ox, oy, oz]
    p1 = [ox + sx * ch, oy + sx * sh, oz + tilt]
    p2 = [ox - sy * sh, oy + sy * ch, oz - tilt]
    js = ask(cmd="plane", p0=p0, p1=p1, p2=p2)["plane"]
    py = Surface.from_points(p0, p1, p2)
    assert js[:3] == pytest.approx(list(py.origin), abs=1e-9)
    assert Surface.from_pose(js[:6]).normal == pytest.approx(list(py.normal), abs=1e-6)
    assert Surface.from_pose(js[:6]).x_axis == pytest.approx(list(py.x_axis), abs=1e-6)
    assert (abs(js[6]), abs(js[7])) == pytest.approx(
        (py.area[1] - py.area[0], py.area[3] - py.area[2]), abs=1e-6
    )
    assert Surface.from_pose(js[:6], (js[6], js[7])).area == pytest.approx(py.area, abs=1e-6)


def test_three_points_in_a_line_are_not_a_plane():
    assert ask(cmd="plane", p0=[0, 0, 0], p1=[0.3, 0, 0], p2=[0.6, 0, 0])["plane"] is None
    assert ask(cmd="plane", p0=[0, 0, 0], p1=[0, 0, 0], p2=[0.1, 0.2, 0])["plane"] is None
    assert ask(cmd="plane", p0=[0, 0, 0], p1=None, p2=[0.1, 0.2, 0])["plane"] is None
    tilted = ask(cmd="plane", p0=[0, 0, 0], p1=[0.3, 0, 0.03], p2=[0, 0.3, 0])
    assert 5.0 < tilted["tilt"] < 6.0


@settings(max_examples=30, deadline=None)
@given(
    xyz=st.lists(st.floats(-1, 1), min_size=3, max_size=3),
    rv=st.lists(st.floats(-2.5, 2.5), min_size=3, max_size=3),
    b=st.lists(st.floats(-0.3, 0.3), min_size=6, max_size=6),
)
def test_pose_trans_matches_urctl(xyz, rv, b):
    a = xyz + rv
    out = ask(cmd="pose_trans", a=a, b=b)
    want = pose_trans(a, b)
    # rotation vectors are compared through the frame they produce (θ and 2π-θ are one rotation)
    assert Surface.from_pose(out["pose"]).origin == pytest.approx(
        list(Surface.from_pose(want).origin), abs=1e-6
    )
    assert Surface.from_pose(out["pose"]).normal == pytest.approx(
        list(Surface.from_pose(want).normal), abs=1e-6
    )
    assert Surface.from_pose(out["pose"]).x_axis == pytest.approx(
        list(Surface.from_pose(want).x_axis), abs=1e-6
    )
    back = pose_trans(a, out["inv"])
    assert back[:3] == pytest.approx([0, 0, 0], abs=1e-6)


@pytest.mark.parametrize(
    "model", ["UR3e", "UR5e", "UR7e", "UR10e", "UR12e", "UR16e", "ur3", "UR 10 e", "UR20", "Fanuc"]
)
def test_the_reach_table_is_the_pythons(model):
    out = ask(cmd="reach", model=model, inner=150, outer=150)
    key = re.sub(r"[^A-Z0-9]", "", model.upper())
    key = key if key.endswith("E") else key + "E"
    py = Reach.for_model(model)
    if key in BASE_RADIUS_M and py is not None:
        assert out["reach"] == pytest.approx([BASE_RADIUS_M[key], MODEL_REACH_M[key]])
        assert out["limits"] == pytest.approx({"min": py.min_m, "max": py.max_m})
    else:
        assert out["reach"] is None and out["limits"] is None


def test_settings_read_the_application_node_the_way_the_pick_node_does():
    app = {
        "cockpitUrl": "192.168.3.10",
        "areas": [
            {"name": "Bench", "p0": [0.2, -0.1, -0.27], "p1": [0.5, -0.1, -0.27], "p2": [0.2, 0.1, -0.27]},
            {"name": "Half taught", "p0": [0.2, -0.1, -0.27], "p1": None, "p2": None},
        ],
        "tipMm": 163,
        "reachInnerMm": 150,
        "reachOuterMm": 150,
        "robotModel": "UR3e",
    }
    params = {
        "nodeId": "0badf00d",
        "points": [{"q": Q1, "area": -1}, {"q": Q2, "area": 0}],
        "values": {"partWidthMm": 20},
    }
    out = ask(cmd="settings", params=params, app=app)
    st_ = out["settings"]
    assert out["problem"] is None
    assert (st_["host"], st_["port"], st_["nodeId"]) == ("192.168.3.10", 7622, "0badf00d")
    assert st_["reachMinM"] == pytest.approx(0.214) and st_["reachMaxM"] == pytest.approx(0.35)
    assert st_["points"][0]["plane"] is None and st_["points"][1]["areaXmm"] == pytest.approx(300)
    o = parse_options(out["tokens"][1])
    assert o.surface is not None and o.surface.origin == pytest.approx([0.2, -0.1, -0.27], abs=1e-5)
    assert o.part == PartSpec.from_mm(50, 20, 30, 25) and o.reach == Reach(0.214, 0.35)
    # a point that names an untaught area, an unknown robot, no cockpit
    out = ask(cmd="settings", params={**params, "points": [{"q": Q1, "area": 1}]}, app=app)
    assert "not taught" in out["problem"]
    out = ask(cmd="settings", params=params, app={**app, "robotModel": "Fanuc"})
    assert out["problem"] is None and "reach=" not in out["tokens"][0]
    out = ask(cmd="settings", params=params, app={**app, "cockpitUrl": ""})
    assert "camera computer's address" in out["problem"]
    out = ask(cmd="settings", params=params, app=None)
    assert "camera computer's address" in out["problem"]


# -- the drawings (the PolyScope 5 node's Diagrams, as SVG) --------------------------------------


@pytest.mark.parametrize(
    "order",
    [(a, b) for a in ("LR", "RL") for b in ("FB", "BF")]
    + [(b, a) for a in ("LR", "RL") for b in ("FB", "BF")],
)
def test_an_order_tile_numbers_a_grid_the_way_the_detector_will(order):
    from perceptronics.synthscene import Box, camera_looking_down, render_depth
    from perceptronics.volume import find_parts

    W, H = 320, 180
    K = {"fx": 230.0, "fy": 230.0, "ppx": W / 2, "ppy": H / 2}
    T = camera_looking_down(0.35, 0.0, 0.12)  # image right = base +X, image down = base -Y
    boxes = [Box(0.28 + 0.07 * c, 0.035 - 0.07 * r, 0.05, 0.035, 0.03) for r in range(2) for c in range(3)]
    sc = find_parts(
        W,
        H,
        render_depth(W, H, K, T, boxes, table_z=-0.27),
        0.001,
        K,
        T,
        spec=PartSpec.from_mm(50, 35, 30),
        order=order,
    )
    got = [[0] * 3 for _ in range(2)]
    for p in sc.parts:
        c = round((p.centre[0] - 0.28) / 0.07)
        r = round((0.035 - p.centre[1]) / 0.07)  # row 0 = the top of the picture
        got[r][c] = p.order
    assert got == ask(cmd="grid", first=order[0], rows=order[1], cols=3, nrows=2)["grid"]


@settings(max_examples=30, deadline=None)
@given(
    ox=st.floats(-0.5, 0.5),
    oy=st.floats(-0.5, 0.5),
    heading=st.floats(-math.pi, math.pi),
    sx=st.floats(0.03, 0.6),
    sy=st.floats(-0.6, 0.6),
)
def test_area_corners_are_the_taught_planes_corners(ox, oy, heading, sx, sy):
    plane = [ox, oy, -0.27, 0.0, 0.0, heading, sx, sy]
    corners = ask(cmd="corners", plane=plane)["corners"]
    surf = Surface.from_pose(plane[:6], (sx, sy))
    want = [
        [ox, oy],
        [ox + sx * surf.x_axis[0], oy + sx * surf.x_axis[1]],
        [ox + sx * surf.x_axis[0] + sy * surf.y_axis[0], oy + sx * surf.x_axis[1] + sy * surf.y_axis[1]],
        [ox + sy * surf.y_axis[0], oy + sy * surf.y_axis[1]],
    ]
    for got, exp in zip(corners, want, strict=True):
        assert got == pytest.approx(exp, abs=1e-6)


@pytest.mark.parametrize(
    "kw",
    [
        {
            "l": 50,
            "w": 30,
            "h": 30,
            "approach": 25,
            "grip": 15,
            "lift": 60,
            "stroke": 50,
            "baseR": 0.064,
            "minR": 0.214,
            "maxR": 0.35,
        },
        {
            "l": 500,
            "w": 5,
            "h": 500,
            "approach": 200,
            "grip": 60,
            "lift": 300,
            "stroke": 300,
            "baseR": 0.095,
            "minR": 0.245,
            "maxR": 0.0,
            "selected": True,
        },
        {
            "l": 5,
            "w": 5,
            "h": 5,
            "approach": 5,
            "grip": 0,
            "lift": 5,
            "stroke": 10,
            "baseR": 0.064,
            "minR": 0.214,
            "maxR": 0.35,
            "areas": [{"name": "A & <B>", "plane": [0.2, -0.1, -0.27, 0, 0, 0.3, 0.3, 0.2]}],
        },
    ],
)
def test_every_drawing_is_well_formed_svg(kw):
    import xml.etree.ElementTree as ET

    svgs = ask(cmd="svg", **kw)["svgs"]
    assert set(svgs) == {"tile", "part", "approach", "map"}
    for name, svg in svgs.items():
        root = ET.fromstring(svg)
        assert root.tag.endswith("svg") and root.get("viewBox"), name
        assert "NaN" not in svg and "undefined" not in svg, name
    texts = [t.text for t in ET.fromstring(svgs["part"]).iter() if t.tag.endswith("text")]
    assert {str(max(kw["l"], kw["w"])), str(min(kw["l"], kw["w"])), str(kw["h"])} <= set(texts)
    if kw.get("areas"):
        assert "A &amp; &lt;B&gt;" in svgs["map"]  # names are escaped, not injected


def test_cockpit_shorthand_and_node_ids():
    out = ask(
        cmd="misc", cockpit=["", ":7621", "7777", "jetson", "jetson:8080", "http://x:1/", "https://pi.local"]
    )
    assert out["cockpit"] == [
        "http://10.0.0.5:7621",
        "http://10.0.0.5:7621",
        "http://10.0.0.5:7777",
        "http://jetson:7621",
        "http://jetson:8080",
        "http://x:1",
        "https://pi.local",
    ]
    a, b = out["ids"]
    assert re.fullmatch(r"[0-9a-f]{6}", a) and re.fullmatch(r"[0-9a-f]{6}", b) and a != b


def test_the_after_picture_node_guards_on_the_pick_nodes_variable():
    out = ask(cmd="misc", locName="my_loc", point=3)["after"]
    assert out == {"before": ["if my_loc == 3:"], "childDepth": 1, "after": ["end"]}
    assert ask(cmd="misc", locName="1 bad", point=2)["after"]["before"] == ["if rs_pick_loc == 2:"]


# -- the behavior workers under PolyScope's worker protocol --------------------------------------

WORKER_HARNESS = r"""
const path = require("path");
const out = [];
const listeners = [];
globalThis.self = {
  postMessage: (m) => out.push(m),
  addEventListener: (type, fn) => { if (type === "message") listeners.push(fn); },
};
globalThis.importScripts = (f) => require(path.join(path.dirname(process.argv[2]), f));
require(process.argv[2]);
const send = (m) => listeners.forEach((fn) => fn({ data: m }));
(async () => {
  for (const m of JSON.parse(process.argv[3])) send(m);
  await new Promise((r) => setTimeout(r, 50));
  process.stdout.write(JSON.stringify(out));
})();
"""


def run_worker(name: str, messages: list[dict]) -> dict[str, list[dict]]:
    harness = ROOT / "target" / "urcapx-worker-harness.js"
    harness.parent.mkdir(exist_ok=True)
    harness.write_text(WORKER_HARNESS, encoding="utf-8")
    proc = subprocess.run(
        [NODE, str(harness), str(FRONTEND / name), json.dumps(messages)],
        capture_output=True,
        text=True,
        encoding="utf-8",  # node prints UTF-8 (×, ·); never the Windows codepage
        timeout=30,
        check=True,
    )
    msgs = json.loads(proc.stdout)
    by_uid: dict[str, list[dict]] = {"init": [msgs[0]]}
    for m in msgs[1:]:
        by_uid.setdefault(m["uid"], []).append(m)
    return by_uid


def result(by_uid: dict, uid: str):
    msgs = by_uid[uid]
    assert [m["type"] for m in msgs] == ["running", "result"], msgs
    assert msgs[1]["complete"] is True
    return msgs[1]["payload"]


APP_CONTEXT = {
    "type": "$$ApplicationContext",
    "contributions": {
        "contributionList": [
            {"type": "ur-something-else", "parentType": "ur-something-else"},
            {
                "type": "nickarmenta-perceptronic",
                "parentType": "nickarmenta-perceptronic",
                "version": "1.1.0",
                "cockpitUrl": "http://192.168.3.10:7621",
                "areas": [
                    {
                        "name": "Bench",
                        "p0": [0.2, -0.1, -0.27],
                        "p1": [0.5, -0.1, -0.27],
                        "p2": [0.2, 0.1, -0.27],
                    }
                ],
                "tipMm": 163,
                "reachInnerMm": 150,
                "reachOuterMm": 150,
                "robotModel": "UR3e",
            },
        ]
    },
    "frames": {"framesList": []},
}


def pick_node(**parameters) -> dict:
    return {
        "type": "nickarmenta-perceptronic-pick",
        "version": "1.0.0",
        "allowsChildren": True,
        "parameters": {
            "nodeId": "a1b2c3",
            "points": [{"q": Q1, "area": -1}, {"q": Q2, "area": 0}],
            "orderFirst": "LR",
            "orderRows": "FB",
            "gripper": "robotiq",
            "popupOnFail": True,
            "pickPort": 7622,
            "values": {},
            "foundVariable": {
                "id": "v1",
                "name": "rs_pick_found",
                "valueType": "boolean",
                "_IDENTIFIER": "VariableDeclaration",
            },
            "locVariable": {
                "id": "v2",
                "name": "rs_pick_loc",
                "valueType": "integer",
                "_IDENTIFIER": "VariableDeclaration",
            },
            **parameters,
        },
    }


def test_the_pick_worker_speaks_the_protocol_and_answers_script_builders():
    node = pick_node()
    empty = pick_node(points=[])
    own = pick_node(gripper="children")
    by = run_worker(
        "pick-node.worker.js",
        [
            {"type": "run", "uid": "factory", "method": "factory", "args": []},
            {"type": "run", "uid": "label", "method": "programNodeLabel", "args": [node]},
            {"type": "run", "uid": "valid", "method": "validator", "args": [node, {}, APP_CONTEXT]},
            {"type": "run", "uid": "invalid", "method": "validator", "args": [empty, {}, APP_CONTEXT]},
            {
                "type": "run",
                "uid": "noapp",
                "method": "validator",
                "args": [node, {}, {"type": "$$ApplicationContext"}],
            },
            {
                "type": "run",
                "uid": "before",
                "method": "generateCodeBeforeChildren",
                "args": [node, {}, APP_CONTEXT],
            },
            {
                "type": "run",
                "uid": "after",
                "method": "generateCodeAfterChildren",
                "args": [node, {}, APP_CONTEXT],
            },
            {
                "type": "run",
                "uid": "own-before",
                "method": "generateCodeBeforeChildren",
                "args": [own, {}, APP_CONTEXT],
            },
            {
                "type": "run",
                "uid": "own-after",
                "method": "generateCodeAfterChildren",
                "args": [own, {}, APP_CONTEXT],
            },
            {
                "type": "run",
                "uid": "broken",
                "method": "generateCodeBeforeChildren",
                "args": [empty, {}, APP_CONTEXT],
            },
            {"type": "run", "uid": "child", "method": "allowsChild", "args": [node, "ur-folder"]},
            {
                "type": "run",
                "uid": "upgrade",
                "method": "upgradeNode",
                "args": [
                    {
                        "type": "x",
                        "version": "0.0.1",
                        "parameters": {"nodeId": "zz", "values": {"speedPct": 999}},
                    }
                ],
            },
            {
                "type": "run",
                "uid": "paste",
                "method": "onLifeCycleHook",
                "args": ["paste", {"id": "g1", "node": node}],
            },
            {"type": "run", "uid": "nope", "method": "nope", "args": []},
        ],
    )
    methods = by["init"][0]["exposed"]["methods"]
    assert by["init"][0]["type"] == "init" and {
        "factory",
        "programNodeLabel",
        "validator",
        "generateCodeBeforeChildren",
        "generateCodeAfterChildren",
        "upgradeNode",
    } <= set(methods)
    fresh = result(by, "factory")
    assert fresh["type"] == "nickarmenta-perceptronic-pick" and fresh["allowsChildren"] is True
    assert re.fullmatch(r"[0-9a-f]{6}", fresh["parameters"]["nodeId"]) and fresh["parameters"]["points"] == []
    assert (
        fresh["parameters"]["values"]["partLengthMm"] == 50 and fresh["parameters"]["foundVariable"] is None
    )
    label = result(by, "label")
    assert (
        label[0] == {"type": "primary", "value": "50×30×30 mm"}
        and label[1]["value"] == "2 pictures · left to right, rows front to back"
    )
    assert result(by, "valid") == {"isValid": True}
    assert (
        result(by, "invalid")["isValid"] is False
        and "picture point" in result(by, "invalid")["errorMessageKey"]
    )
    assert "camera computer's address" in result(by, "noapp")["errorMessageKey"]
    before = result(by, "before")
    assert before["type"] == "$$ScriptBuilder" and before["currentIndent"] == 1
    assert before["script"].startswith("# Perceptronic Pick") and before["script"].rstrip().endswith(
        "if rs_pick_found:"
    )
    assert 'socket_open("192.168.3.10", 7622, "rs_pick")' in before["script"]
    assert (
        "reach=0.214,0.350" in before["script"] and "plane=p[0.20000, -0.10000, -0.27000" in before["script"]
    )
    assert result(by, "after") == {"type": "$$ScriptBuilder", "script": "end\n", "currentIndent": -1}
    assert result(by, "own-before")["currentIndent"] == 4 and result(by, "own-after")["currentIndent"] == -4
    assert result(by, "own-after")["script"].startswith("        set_tcp(p[0, 0, 0, 0, 0, 0])")
    assert by["broken"][-1]["type"] == "error" and "picture point" in by["broken"][-1]["error"]["message"]
    assert result(by, "child") is True
    up = result(by, "upgrade")
    assert (
        up["version"] == "1.0.0"
        and up["allowsChildren"] is True
        and re.fullmatch(r"[0-9a-f]{6}", up["parameters"]["nodeId"])
    )
    assert up["parameters"]["values"]["speedPct"] == 100 and up["parameters"]["values"]["partLengthMm"] == 50
    pasted = result(by, "paste")
    assert (
        re.fullmatch(r"[0-9a-f]{6}", pasted["node"]["parameters"]["nodeId"])
        and pasted["node"]["parameters"]["nodeId"] != "a1b2c3"
    )
    assert by["nope"][0]["type"] == "error" and by["nope"][0]["error"]["__error_marker"] == "$$error"


def test_the_after_worker_reads_the_enclosing_pick_node():
    after = {
        "type": "nickarmenta-perceptronic-after",
        "version": "1.0.0",
        "allowsChildren": True,
        "parameters": {"point": 2},
    }
    inside = {
        "type": "$$ScriptContext",
        "ancestors": [{"type": "ur-folder"}, pick_node(locVariable={"name": "my_loc"})],
    }
    wrapped = {"traverse": {"ancestors": [{"id": "x", "node": pick_node()}]}}
    by = run_worker(
        "after-node.worker.js",
        [
            {"type": "run", "uid": "factory", "method": "factory", "args": []},
            {"type": "run", "uid": "label", "method": "programNodeLabel", "args": [after]},
            {"type": "run", "uid": "alone", "method": "validator", "args": [after, {"ancestors": []}]},
            {"type": "run", "uid": "inside", "method": "validator", "args": [after, inside]},
            {
                "type": "run",
                "uid": "too-far",
                "method": "validator",
                "args": [{**after, "parameters": {"point": 3}}, inside],
            },
            {"type": "run", "uid": "before", "method": "generateCodeBeforeChildren", "args": [after, inside]},
            {
                "type": "run",
                "uid": "before2",
                "method": "generateCodeBeforeChildren",
                "args": [after, wrapped],
            },
            {"type": "run", "uid": "after", "method": "generateCodeAfterChildren", "args": [after, inside]},
        ],
    )
    assert result(by, "factory") == {
        "type": "nickarmenta-perceptronic-after",
        "version": "1.0.0",
        "allowsChildren": True,
        "parameters": {"point": 1},
    }
    assert result(by, "label")[0]["value"] == "2"
    assert "inside a Perceptronic Pick" in result(by, "alone")["errorMessageKey"]
    assert result(by, "inside") == {"isValid": True}
    assert "2 picture points, not 3" in result(by, "too-far")["errorMessageKey"]
    assert result(by, "before") == {
        "type": "$$ScriptBuilder",
        "script": "if my_loc == 2:\n",
        "currentIndent": 1,
    }
    assert result(by, "before2")["script"] == "if rs_pick_loc == 2:\n"
    assert result(by, "after") == {"type": "$$ScriptBuilder", "script": "end\n", "currentIndent": -1}
