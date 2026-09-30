"""The 0.5.0 Perceptronic Pick node's contract, under a JDK: the URScript it writes, the request
options it sends (read back by the Python pick server's own parser), and the pendant's
drawings and pose math agreeing with the Python side they stand for.

The Java runs in the harness of :mod:`tests.test_urcap5` (no UR API needed)."""

from __future__ import annotations

import json
import math
import re

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from perceptronics.partspec import PartSpec
from perceptronics.picknode import STATUS, parse_options, parse_request
from perceptronics.volume import Reach, Surface
from tests.test_urcap5 import java_client  # noqa: F401 — the fixture
from urctl.pose import pose_trans

Q1 = [-1.37, -0.49, 1.61, -2.69, -1.57, 0.61]
Q2 = [-0.80, -0.60, 1.50, -2.50, -1.57, 0.90]
PLANE = [0.20, -0.10, -0.27, 0.0, 0.0, 0.3]
NODE = {
    "host": "192.168.3.10",
    "node": "a1b2c3",
    "points": [{"q": Q1}, {"q": Q2, "plane": PLANE, "area": [300, 200]}],
}


def pick(java_client, **kw):  # noqa: F811
    return java_client("pick", json.dumps({**NODE, **kw}))


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


def test_the_script_is_ascii_balanced_and_runs_in_the_documented_order(java_client):  # noqa: F811
    out = pick(java_client, children="  AFTER_PICK()")
    assert out["problem"] is None
    text = out["script"]
    assert text.isascii() and balanced(text)
    order = [
        "set_tcp(p[0, 0, 0, 0, 0, 0])",
        'socket_open("192.168.3.10", 7622, "rs_pick")',
        'socket_open("127.0.0.1", 63352, "rs_rq")',  # the Robotiq daemon, controller-local
        '"SET POS 0"',  # fully open while the arm travels
        '"NEXT "',
        "movej([-1.370000, -0.490000, 1.610000, -2.690000, -1.570000, 0.610000]",
        '"FIND "',
        '"LOOK "',
        '"REFINE "',
        "movel(rs_hover",
        '"GET OBJ"',  # the fingers really are open at the approach
        "movel(rs_grip, a=0.3, v=0.05)",
        '"SET POS 255"',
        "movel(rs_lift",
        "rs_pick_found = True",
        "rs_pick_loc = rs_loc",
        'socket_close("rs_pick")',
        "set_tcp(rs_tcp0)",
        "if rs_pick_found:",
        "AFTER_PICK()",
    ]
    at = [text.index(s) for s in order]
    assert at == sorted(at), [s for s, i in zip(order, at, strict=True) if i != sorted(at)[at.index(i)]]
    # nothing moves toward a part before the controller's own IK solved approach, grip and lift
    assert text.index("get_inverse_kin_has_solution(rs_hover") < text.index("movel(rs_hover")
    # the approach: fingertips 25 mm over the top, along the tool axis; grip 15 mm past it
    assert "pose_trans(rs_top, p[0, 0, -0.0250, 0, 0, 0])" in text
    assert "pose_trans(rs_top, p[0, 0, 0.0150, 0, 0, 0])" in text
    # the routine after the pick runs with the operator's TCP back, once, after everything
    assert text.rstrip().endswith("end") and text.count("AFTER_PICK()") == 1


def test_every_picture_point_is_visited_by_its_own_joints_and_options(java_client):  # noqa: F811
    text = pick(java_client)["script"]
    assert text.count("elif rs_loc == 2:") == 2  # the options chain and the movej chain
    assert "movej([-0.800000, -0.600000, 1.500000, -2.500000, -1.570000, 0.900000]" in text
    toks = re.findall(r'rs_tok = "( [^"]+)"', text)
    assert len(toks) == 2 and "loc=1" in toks[0] and "loc=2" in toks[1] and "plane=" in toks[1]


def test_the_options_it_sends_are_what_the_pick_server_reads(java_client):  # noqa: F811
    values = {
        "partLengthMm": 42,
        "partWidthMm": 61,
        "partHeightMm": 27,
        "partTolPct": 20,
        "gripBelowTopMm": 12,
    }
    out = pick(java_client, values=values, order=["RL", "BF"], reach=[0.214, 0.35])
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


def test_every_request_line_it_can_send_parses(java_client):  # noqa: F811
    text = pick(java_client)["script"]
    pose = "p[0.3, -0.1, 0.12, 3.14159, 0, 0]"
    toks = re.findall(r'rs_tok = "( [^"]+)"', text)
    lines = [f"NEXT {pose} node=a1b2c3 locs=2 proto=2", f"LOOK {pose} p[0.3, 0, -0.24, 0, 0, 0]"]
    lines += [f"FIND {pose}{t}" for t in toks]
    lines += [f"REFINE {pose} p[0.3, 0, -0.24, 0, 0, 0]{t} lean=12" for t in toks]
    for line in lines:
        assert len(line) < 1024
        parse_request(line)  # raises on anything the server would refuse


def test_the_popup_names_every_status_the_server_can_send(java_client):  # noqa: F811
    codes = set(java_client("reasons"))
    sent = {c for c in STATUS if c not in (1, -6)}  # -6 answers LOOK, which never fails a pick
    assert sent <= codes, sent - codes
    assert -8 in codes  # the program's own: no answer in 10 s


@pytest.mark.parametrize(
    ("kw", "problem"),
    [
        ({"host": ""}, "camera computer's address"),
        ({"host": 'x"); popup("pwned'}, "not an address"),  # a saved field can't inject URScript
        ({"node": ""}, "identity"),
        ({"node": 'a");stop("'}, "identity"),
        ({"points": []}, "add a picture point"),
        ({"points": [{"q": [0, 0, 0]}]}, "not a joint position"),
        ({"values": {"gripBelowTopMm": 29, "partHeightMm": 30}}, "fingertips on the table"),
        ({"values": {"partWidthMm": 48}}, "wider than 50 mm"),
        ({"order": ["LR", "RL"]}, "one horizontal and one vertical"),
        ({"gripper": "vacuum"}, "unknown gripper"),
        ({"reach": [0.4, 0.3]}, "reach limits"),
        ({"var": "1bad name"}, "variable"),
        ({"values": {"approachMm": 1}}, "approach must be 5..200"),
    ],
)
def test_it_refuses_what_it_cannot_generate_safely(java_client, kw, problem):  # noqa: F811
    out = pick(java_client, **kw)
    assert out["problem"] and problem in out["problem"], out["problem"]
    assert out["script"] is None


def test_a_digital_output_gripper(java_client):  # noqa: F811
    text = pick(java_client, gripper="digital", values={"gripperDo": 3, "gripperWaitS": 0.4})["script"]
    assert balanced(text)
    assert "63352" not in text
    opened = text.index("set_standard_digital_out(3, False)")
    closed = text.index("set_standard_digital_out(3, True)")
    assert opened < text.index("movel(rs_hover") < text.index("movel(rs_grip") < closed
    assert "sleep(0.40)" in text


def test_my_own_gripper_nodes_run_at_the_grip_with_the_operators_tcp(java_client):  # noqa: F811
    text = pick(java_client, gripper="children", children="  CLOSE_IT()")["script"]
    assert balanced(text) and "63352" not in text
    grip, child, lift = text.index("movel(rs_grip"), text.index("CLOSE_IT()"), text.index("movel(rs_lift")
    assert grip < text.index("set_tcp(rs_tcp0)", grip) < child < lift
    assert "if rs_pick_found:" not in text  # no routine after: the program's next nodes are it


def test_a_close_on_nothing_opens_and_tries_the_next_part(java_client):  # noqa: F811
    text = pick(java_client)["script"]
    miss = text.index('rs_why = "the gripper closed on nothing"')
    assert text.index('"SET POS 0"', miss) < text.index(
        "socket_close", miss
    )  # opened again before the next try
    assert re.search(r"while \(rs_ok\) and \(rs_try < 6\) and \(rs_pick_found == False\):", text)  # 2 + 3 + 1


def test_without_the_popup_a_failed_run_just_leaves_the_result_false(java_client):  # noqa: F811
    assert "popup(" not in pick(java_client, popup=False)["script"]
    assert "popup(" in pick(java_client)["script"]


def test_speed_scales_the_nodes_own_travel(java_client):  # noqa: F811
    slow = pick(java_client, values={"speedPct": 10})["script"]
    # movej: 1.4 rad/s^2 and 1.05 rad/s at 10 %
    assert re.search(r"movej\(\[[^]]*\], a=0\.14, v=0\.1[01]\)", slow)
    assert "movel(rs_grip, a=0.3, v=0.05)" in slow  # the last stretch into the part never speeds up


# -- the settings table ---------------------------------------------------------------------------


def test_every_default_is_within_its_own_limits_and_the_defaults_generate(java_client):  # noqa: F811
    nums = java_client("numbers")
    keys = [n["key"] for n in nums]
    assert len(keys) == len(set(keys))
    for n in nums:
        assert n["min"] <= n["def"] <= n["max"], n
        assert n["section"] in ("part", "approach", "gripper", "motion")
    assert pick(java_client)["problem"] is None
    d = {n["key"]: n["def"] for n in nums}
    assert d["approachMm"] == 25  # Nick: fingertips 25 mm over the top, fully open


@pytest.mark.parametrize(
    ("key", "value", "stored"),
    [
        ("partLengthMm", 49.6, 50),
        ("partLengthMm", 9999, 500),
        ("partLengthMm", -3, 5),
        ("settleS", 0.33, 0.33),
        ("approachMm", float("nan"), 25),
    ],
)
def test_values_are_clamped_to_their_limits(java_client, key, value, stored):  # noqa: F811
    arg = "NaN" if math.isnan(value) else repr(value)  # Java's spelling
    assert java_client("set", key, arg) == pytest.approx(stored)


# -- the drawings and the maths agree with the Python they stand for ------------------------------


@pytest.mark.parametrize(
    "order",
    [
        ("LR", "FB"),
        ("RL", "FB"),
        ("LR", "BF"),
        ("RL", "BF"),
        ("FB", "LR"),
        ("FB", "RL"),
        ("BF", "LR"),
        ("BF", "RL"),
    ],
)
def test_an_order_tile_numbers_a_grid_the_way_the_detector_will(java_client, order):  # noqa: F811
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
    assert got == java_client("grid", order[0], order[1], "3", "2")


@settings(max_examples=40, deadline=None)
@given(
    ox=st.floats(-0.5, 0.5),
    oy=st.floats(-0.5, 0.5),
    oz=st.floats(-0.4, 0.1),
    heading=st.floats(-math.pi, math.pi),
    sx=st.floats(0.03, 0.6),
    sy=st.floats(-0.6, 0.6),
    tilt=st.floats(-0.05, 0.05),
)
def test_a_taught_plane_is_the_same_plane_in_java_and_python(java_client, ox, oy, oz, heading, sx, sy, tilt):  # noqa: F811
    if abs(sy) < 0.03:
        return
    a = (ox, oy, oz)
    b = (ox + sx * math.cos(heading), oy + sx * math.sin(heading), oz)
    c = (ox - sy * math.sin(heading), oy + sy * math.cos(heading), oz + tilt * abs(sy))
    got = java_client("plane", json.dumps([a, b, c]))
    want = Surface.from_points(a, b, c)
    back = Surface.from_pose(got[:6], got[6:8])
    assert back.origin == pytest.approx(want.origin, abs=1e-6)
    assert back.normal == pytest.approx(want.normal, abs=1e-6)
    assert back.x_axis == pytest.approx(want.x_axis, abs=1e-6)
    assert back.area == pytest.approx(want.area, abs=1e-6)
    assert got[8] == pytest.approx(want.tilt_deg(), abs=1e-6)


def test_three_points_in_a_line_are_not_a_plane(java_client):  # noqa: F811
    assert java_client("plane", json.dumps([[0, 0, 0], [0.1, 0, 0], [0.2, 0, 0]])) is None
    assert java_client("plane", json.dumps([[0, 0, 0], [0, 0, 0], [0.2, 0.1, 0]])) is None


@settings(max_examples=40, deadline=None)
@given(
    st.lists(st.floats(-1, 1), min_size=3, max_size=3),
    st.lists(st.floats(-3.1, 3.1), min_size=3, max_size=3),
    st.lists(st.floats(-0.3, 0.3), min_size=6, max_size=6),
)
def test_pose_trans_matches_urctl(java_client, xyz, rv, b):  # noqa: F811
    a = xyz + rv
    got = java_client("trans", json.dumps(a), json.dumps(b))
    want = pose_trans(a, b)
    assert got[:3] == pytest.approx(want[:3], abs=1e-7)
    # the rotation vector itself can wrap (θ vs 2π − θ): compare the rotations
    from urctl.pose import Transform

    Rg, Rw = Transform.from_pose(got).rotation, Transform.from_pose(want).rotation
    assert [v for row in Rg for v in row] == pytest.approx([v for row in Rw for v in row], abs=1e-7)


def test_a_touch_measures_the_fingertips_whatever_tcp_polyscope_has(java_client):  # noqa: F811
    flange = [0.35, -0.10, -0.05, math.pi, 0.0, 0.0]  # tool straight down
    for offset in ([0, 0, 0, 0, 0, 0], [0, 0, 0.1, 0, 0, 0], [0, -0.035, 0.22, 0, 0, 0]):
        tcp = pose_trans(flange, offset)  # what PolyScope reports for its active TCP
        tip = java_client("fingertip", json.dumps(tcp), json.dumps(offset), "0.163")
        assert tip == pytest.approx([0.35, -0.10, -0.05 - 0.163], abs=1e-6)


def test_the_screen_parses_what_the_cockpit_sends(java_client):  # noqa: F811
    from perceptronics.picknode import scene_report
    from perceptronics.synthscene import Box
    from tests.test_picknode2 import FLANGE, OPTS, ROW, Frames, planner

    out = scene_report(
        planner(Frames(ROW + [Box(0.35, 0.05, 0.068, 0.035, 0.03)])), FLANGE, parse_options(OPTS)
    )
    got = java_client("scene", json.dumps(out))
    assert got["orders"] == [1, 2, 3] and got["whys"] == ["too long"]
    assert got["surface"] == "fitted" and got["base"] is True and got["width"] == out["width"]
    assert java_client("scene", json.dumps({"ok": False, "error": "x"}))["orders"] == []


@pytest.mark.parametrize("model", ["UR3", "UR3e", "UR5", "UR7e", "UR10", "UR12e", "UR16", "UR20", "UR30"])
def test_the_reach_table_is_the_pythons(java_client, model):  # noqa: F811
    got = java_client("reach", model)
    want = Reach.for_model(model, inner_margin_m=0, outer_margin_m=0)
    if want is None:
        assert got is None
    else:
        assert got == pytest.approx([want.min_m, want.max_m])


# -- older controllers (the PolyScope 5 matrix, 2026-09-29) ---------------------------------------


def _reads(script: str) -> dict[str, set[int]]:
    """Each variable a socket_read_ascii_float result is assigned to -> the counts it reads."""
    out: dict[str, set[int]] = {}
    for var, n in re.findall(r"(\w+) = socket_read_ascii_float\((\d+),", script):
        out.setdefault(var, set()).add(int(n))
    return out


@pytest.mark.parametrize("polyscope", [None, [5, 4, 3], [5, 9, 4], [5, 26, 1]])
@pytest.mark.parametrize("gripper", ["robotiq", "digital", "children"])
def test_every_list_keeps_one_size(java_client, polyscope, gripper):  # noqa: F811
    """PolyScope 5.9.4-5.14.6 stopped the node with "Resizing of 'List' is not supported" when rs_r
    took the close look's 10 numbers after the 16 of FIND: every variable a read lands in
    reads one count, always."""
    spec = {"gripper": gripper} | ({"polyscope": polyscope} if polyscope else {})
    script = pick(java_client, **spec)["script"]
    reads = _reads(script)
    assert reads and all(len(ns) == 1 for ns in reads.values()), reads
    assert reads["rs_r"] == {16} and reads["rs_lk"] == {10}


@pytest.mark.parametrize(
    "polyscope, checked",
    [
        (None, True),  # unknown: the newest
        ([5, 4, 3], False),
        ([5, 8, 2], False),  # compile_error_name_not_found:get_inverse_kin_has_solution
        ([5, 9, 0], False),  # no image to prove it: counted without
        ([5, 9, 3], False),
        ([5, 9, 4], True),  # the oldest image that compiles it
        ([5, 10, 0], True),
        ([5, 26, 1], True),
        ([6, 0, 0], True),
    ],
)
def test_the_ik_check_only_where_the_controller_has_it(java_client, polyscope, checked):  # noqa: F811
    spec = {"polyscope": polyscope} if polyscope else {}
    out = pick(java_client, **spec)
    assert out["problem"] is None and balanced(out["script"])
    assert ("get_inverse_kin_has_solution" in "\n".join(body(out["script"]))) is checked
    # the close look and the approach still happen either way
    assert out["script"].count("movej(get_inverse_kin(rs_look, get_actual_joint_positions())") == 1
    assert "rs_go = True" in out["script"] and "movel(rs_hover" in out["script"]
    if not checked:
        assert (
            f"# PolyScope {polyscope[0]}.{polyscope[1]}.{polyscope[2]}: no get_inverse_kin_has_solution"
            in (out["script"])
        )
        assert (
            "no IK for approach" not in out["script"] and "out of reach - looking again" not in out["script"]
        )
