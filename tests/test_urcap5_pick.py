"""The 3D Pick node's contract (0.7.0), under a JDK: the URScript it writes — one move
sequence from the survey to the gripper clamped on a part, no children — the request options
it sends (read back by the Python pick server's own parser), its screens (nothing scrolls,
two option tabs, only near misses drawn on the picture), and the pendant's drawings and pose
math agreeing with the Python side they stand for.

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


def test_the_script_is_one_move_sequence_from_the_survey_to_the_grip(java_client):  # noqa: F811
    out = pick(java_client)
    assert out["problem"] is None
    text = out["script"]
    assert text.isascii() and balanced(text)
    assert text.startswith("# 3D Pick 0.8.0 ")
    order = [
        "set_tcp(p[0, 0, 0, 0, 0, 0])",
        'socket_open("192.168.3.10", 7622, "rs_pick")',
        '"NEXT "',
        "movej([-1.370000, -0.490000, 1.610000, -2.690000, -1.570000, 0.610000]",  # the survey
        '"FIND "',
        '"LOOK "',
        '"REFINE "',
        "movel(rs_hover",
        "movel(rs_grip, a=0.3, v=0.05)",
        "rs_pick_found = True",
        "rs_pick_loc = rs_loc",
        'socket_close("rs_pick")',
        "set_tcp(rs_tcp0)",
    ]
    at = [text.index(s) for s in order]
    assert at == sorted(at), [s for s, i in zip(order, at, strict=True) if i != sorted(at)[at.index(i)]]
    # nothing moves toward a part before the controller's own IK solved approach and grip
    assert text.index("get_inverse_kin_has_solution(rs_hover") < text.index("movel(rs_hover")
    # the approach: fingertips 25 mm over the top, along the tool axis; grip 15 mm past it
    assert "pose_trans(rs_top, p[0, 0, -0.0250, 0, 0, 0])" in text
    assert "pose_trans(rs_top, p[0, 0, 0.0150, 0, 0, 0])" in text
    # it ends at the grip: the last motion is the descent, and the result is set right after it
    assert text.rindex("movel(") == text.index("movel(rs_grip")
    assert "rs_lift" not in text and "if rs_pick_found:" not in text


def test_the_node_does_not_touch_the_gripper(java_client):  # noqa: F811
    """Nick, 2026-10-01: "the node should not control the gripper whatsoever because the user
    will open the gripper before the node and close it after"."""
    for kw in (
        {},
        {"closeLook": False},
        {"shape": "cyl", "values": {"partLengthMm": 40}},
        {"polyscope": [5, 4, 3]},
    ):
        text = pick(java_client, **kw)["script"]
        for gripper in ("63352", "rs_rq", "SET POS", "SET GTO", "GET OBJ", "SET ACT", "rs_held", "rs_obj"):
            assert gripper not in text, gripper
        assert "set_standard_digital_out" not in text and "set_tool_digital_out" not in text
        assert "closed on nothing" not in text
        assert text.count("socket_open(") == 1  # the pick server, nothing else


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
        "approachMm": 35,
    }
    out = pick(java_client, values=values, order=["RL", "BF"], arm="UR3")
    for i, tok in enumerate(out["tokens"]):
        o = parse_options(tok)
        assert o.part == PartSpec.from_mm(61, 42, 27, 20)  # long side first, however it was typed
        assert o.order == ("RL", "BF")
        # no ring: the arm is named, and the server asks its kinematics
        assert o.reach is None and o.arm == "UR3" and "reach=" not in tok
        assert (o.grip_below_m, o.approach_m) == (0.012, 0.035)
        # the grip check is on, with 20 mm of room on each side; no stroke: the node knows no gripper
        assert (o.grip_check, o.room_m, o.across) == (True, 0.02, "short") and "stroke=" not in tok
        assert (o.node, o.locs, o.proto) == ("a1b2c3", 2, 2)
        assert o.loc == i  # tokens(-1) (the teach screen with no point yet) carries no loc
    plane = parse_options(out["tokens"][2]).surface
    want = Surface.from_pose(PLANE, (0.3, 0.2))
    assert plane.origin == pytest.approx(want.origin, abs=1e-5) and plane.area == pytest.approx(want.area)
    assert parse_options(out["tokens"][1]).surface is None  # point 1 finds the table live


def test_the_grip_check_is_on_with_20_mm_of_finger_room_and_both_are_adjustable(java_client):  # noqa: F811
    o = parse_options(pick(java_client)["tokens"][1])
    assert (o.grip_check, o.room_m) == (True, 0.02)
    o = parse_options(pick(java_client, values={"fingerRoomMm": 35})["tokens"][1])
    assert (o.grip_check, o.room_m) == (True, 0.035)
    assert parse_options(pick(java_client, gripCheck=False)["tokens"][1]).grip_check is False
    # a wide part is nobody's to refuse: the node knows no gripper stroke
    assert pick(java_client, values={"partWidthMm": 300, "partLengthMm": 400})["problem"] is None
    head = pick(java_client)["script"].splitlines()[0]
    assert "finger room 20 mm" in head and "no grip check" in pick(java_client, gripCheck=False)["script"]


def test_a_box_is_gripped_across_its_short_side_unless_the_long_side_is_ticked(java_client):  # noqa: F811
    assert parse_options(pick(java_client)["tokens"][1]).across == "short"
    out = pick(java_client, gripLong=True)
    assert parse_options(out["tokens"][1]).across == "long" and "across the long side" in out["script"]
    # a cylinder has no side to choose: the tick left over from a box is not sent
    cyl = pick(java_client, gripLong=True, shape="cyl", values={"partLengthMm": 40})
    assert "across=" not in cyl["tokens"][1] and "long side" not in cyl["script"]


def test_a_cylinder_is_sent_by_its_diameter(java_client):  # noqa: F811
    out = pick(java_client, shape="cyl", values={"partLengthMm": 40, "partWidthMm": 12, "partHeightMm": 30})
    assert out["problem"] is None and out["words"] == "cylinder D40 x 30 mm +-25 %"
    o = parse_options(out["tokens"][1])
    assert o.part == PartSpec.from_mm(40, 40, 30, 25, shape="cyl")  # the width field plays no part
    assert parse_options(pick(java_client)["tokens"][1]).part.shape == "box"
    assert "unknown part shape" in pick(java_client, shape="hex")["problem"]


def test_every_request_line_it_can_send_parses(java_client):  # noqa: F811
    text = pick(java_client, arm="UR10", shape="cyl", gripCheck=True, values={"partLengthMm": 40})["script"]
    pose = "p[0.3, -0.1, 0.12, 3.14159, 0, 0]"
    toks = re.findall(r'rs_tok = "( [^"]+)"', text)
    look_tail = re.search(r'to_str\(rs_c\), "( [^"]+)"\)\)\)\), "rs_pick"', text).group(1)
    lines = [f"NEXT {pose} node=a1b2c3 locs=2 proto=2", f"LOOK {pose} p[0.3, 0, -0.24, 0, 0, 0]{look_tail}"]
    lines += [f"FIND {pose}{t}" for t in toks]
    lines += [f"REFINE {pose} p[0.3, 0, -0.24, 0, 0, 0]{t} lean=12" for t in toks]
    for line in lines:
        assert len(line) < 1024
        parse_request(line)  # raises on anything the server would refuse
    assert parse_request(lines[1])["options"].stroke_m == 0.05  # the look knows how wide the fingers are


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
        ({"order": ["LR", "RL"]}, "one horizontal and one vertical"),
        ({"arm": 'UR3" stop'}, "robot model"),  # PolyScope's model name can't inject into a request
        ({"var": "1bad name"}, "variable"),
        ({"values": {"approachMm": 1}}, "approach must be 5..200"),
    ],
)
def test_it_refuses_what_it_cannot_generate_safely(java_client, kw, problem):  # noqa: F811
    out = pick(java_client, **kw)
    assert out["problem"] and problem in out["problem"], out["problem"]
    assert out["script"] is None


def test_a_part_the_arm_cannot_get_to_sends_it_on_to_the_next_one(java_client):  # noqa: F811
    text = pick(java_client)["script"]
    assert re.search(r"while \(rs_ok\) and \(rs_try < 6\) and \(rs_pick_found == False\):", text)  # 2 + 3 + 1
    assert "no joint solution for the approach" in text


def test_without_the_popup_a_failed_run_just_leaves_the_result_false(java_client):  # noqa: F811
    assert "popup(" not in pick(java_client, popup=False)["script"]
    assert "popup(" in pick(java_client)["script"]


def test_the_nodes_travel_speed_is_fixed_and_the_last_stretch_is_slow(java_client):  # noqa: F811
    text = pick(java_client)["script"]
    assert re.search(r"movej\(\[[^]]*\], a=0\.84, v=0\.63\)", text)  # 60 % of 1.4 rad/s^2 and 1.05 rad/s
    assert "movel(rs_hover, a=0.36, v=0.15)" in text
    assert "movel(rs_grip, a=0.3, v=0.05)" in text


def test_without_the_closer_look_every_part_is_measured_from_its_picture_point(java_client):  # noqa: F811
    """Options → Approach → Closer look, off: no LOOK, no move in — and no part taken from the
    queue, because a queued part was seen from a picture point the arm is no longer at."""
    text = pick(java_client, closeLook=False)["script"]
    assert balanced(text) and '"LOOK "' not in text and "rs_look" not in text
    assert "next part already seen" not in text
    # the survey is not behind an "unless a part is queued": NEXT, then always the picture point
    lines = text.splitlines()
    nxt = next(i for i, line in enumerate(lines) if '"NEXT "' in line)
    movej = next(i for i, line in enumerate(lines) if "movej([-1.37" in line)
    indent = len(lines[movej]) - len(lines[movej].lstrip())
    assert nxt < movej and indent == 6  # while > if rs_loc == 1 > movej: nothing else around it
    order = ['"FIND "', '"REFINE "', "movel(rs_hover", "movel(rs_grip", "rs_pick_found = True"]
    at = [text.index(s) for s in order]
    assert at == sorted(at)
    assert "- no closer look" in lines[0]
    with_look = pick(java_client)["script"]
    assert '"LOOK "' in with_look and "next part already seen" in with_look


def test_no_closer_look_pose_means_measuring_again_from_where_the_arm_is(java_client):  # noqa: F811
    """0.6.0 fell back to a pose straight over the part — the camera behind its own gripper."""
    text = pick(java_client)["script"]
    assert "lookMm" not in text and "rs_see = False" in text
    seen = text.index("if rs_see:", text.index("rs_see = get_inverse_kin_has_solution"))
    move = text.index("movej(get_inverse_kin(rs_look", seen)
    other = text.index("else:", move)
    assert "no closer look from here" in text[other : text.index('"REFINE "', other)]
    assert "movej(" not in text[other : text.index('"REFINE "', other)]


# -- the settings table ---------------------------------------------------------------------------


def test_every_default_is_within_its_own_limits_and_the_defaults_generate(java_client):  # noqa: F811
    nums = java_client("numbers")
    keys = [n["key"] for n in nums]
    assert len(keys) == len(set(keys))
    for n in nums:
        assert n["min"] <= n["def"] <= n["max"], n
        assert n["section"] in ("part", "approach")  # the two tabs: nothing about speeds or the gripper
    assert set(keys) == {
        "partLengthMm",
        "partWidthMm",
        "partHeightMm",
        "partTolPct",
        "approachMm",
        "gripBelowTopMm",
        "fingerRoomMm",
    }
    assert {n["key"]: n["def"] for n in nums}["fingerRoomMm"] == 20  # Nick: 20 mm on both pick sides
    assert pick(java_client)["problem"] is None
    d = {n["key"]: n["def"] for n in nums}
    assert d["approachMm"] == 25  # Nick: fingertips 25 mm over the top, fully open


@pytest.mark.parametrize(
    ("key", "value", "stored"),
    [
        ("partLengthMm", 49.6, 50),
        ("partLengthMm", 9999, 500),
        ("partLengthMm", -3, 5),
        ("gripBelowTopMm", 12.4, 12),
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
    assert got["drawn"] == ["too long"] and got["summary"] == "3 parts to pick · 1 not (yellow, with why)"
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
@pytest.mark.parametrize("gripper", ["robotiq", "digital"])
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
    # the closer look and the approach still happen either way
    assert out["script"].count("movej(get_inverse_kin(rs_look, get_actual_joint_positions())") == 1
    assert "rs_go = True" in out["script"] and "movel(rs_hover" in out["script"]
    if not checked:
        assert (
            f"# PolyScope {polyscope[0]}.{polyscope[1]}.{polyscope[2]}: no get_inverse_kin_has_solution"
            in (out["script"])
        )
        assert "no IK for approach" not in out["script"]


# -- the screens (0.7.0): nothing scrolls, two option tabs, only near misses on the picture -------

PANELS = [(1000, 560), (1280, 720)]  # the README's framing and the pendant's whole screen


@pytest.mark.parametrize("size", PANELS)
@pytest.mark.parametrize("points", [0, 1, 12])
@pytest.mark.parametrize("view", ["main", "part", "approach"])
@pytest.mark.parametrize("shape", ["box", "cyl"])
def test_everything_on_the_nodes_screen_fits_without_scrolling(java_client, size, points, view, shape):  # noqa: F811
    """Nick, 2026-09-30: "Do NOT use scrolling". No scroll pane exists, and no control is laid
    out past the panel's edge or with no room — at twelve picture points as at none."""
    got = java_client("screen", str(size[0]), str(size[1]), shape, str(points), view)
    assert got["scrollers"] == []
    assert got["clipped"] == [], got["clipped"]


def test_the_options_are_two_tabs_part_and_approach_and_nothing_else(java_client):  # noqa: F811
    part = set(java_client("screen", "1000", "560", "box", "1", "part")["texts"])
    assert {"Length", "Width", "Height", "Tolerance"} <= part and "Grip check" not in part
    approach = set(java_client("screen", "1000", "560", "box", "1", "approach")["texts"])
    assert {"Approach", "Grip depth", "Closer look"} <= approach
    # the grip check lives on the Approach tab, with its room and the side a box is gripped across
    assert {"Grip check", "Finger room", "Grip across the long side"} <= approach
    cyl_approach = set(java_client("screen", "1000", "560", "cyl", "1", "approach")["texts"])
    assert "Grip across the long side" not in cyl_approach and "Finger room" in cyl_approach
    assert "Length" not in approach and "Grip depth" not in part  # one tab at a time
    for gone in (
        "Gripper",
        "Grip force",
        "Finger speed",
        "Speed",
        "Settle",
        "Lift",
        "Open width",
        "Digital output",
    ):
        assert gone not in part | approach
    cyl = set(java_client("screen", "1000", "560", "cyl", "1", "part")["texts"])
    assert "Diameter" in cyl and "Length" not in cyl
    main = set(java_client("screen", "1000", "560", "box", "3", "main")["texts"])
    assert {"3D Pick", "Picture points", "Pick order", "Options", "Check approach"} <= main


def test_the_picture_draws_pickable_parts_green_with_a_number_and_the_rest_yellow_with_why(java_client):  # noqa: F811
    """Nick, 2026-10-01. What is nothing like the part still gets no graphic."""

    def part(u, why=None, near=True, order=0):
        corners = [[u - 40, 200], [u + 40, 200], [u + 40, 260], [u - 40, 260]]
        return {
            "order": order,
            "pixel": [u, 230],
            "corners_px": corners,
            "size_mm": [50, 30],
            "height_mm": 30,
            "why": why,
            "near": near,
        }

    def scene(parts, rejected):
        return json.dumps({"ok": True, "width": 848, "height": 480, "parts": parts, "rejected": rejected})

    picked = java_client("liveview", "depth", scene([part(200, order=1), part(400, order=2)], []))
    assert picked["green"] > 400 and picked["yellow"] == 0
    junk = java_client("liveview", "depth", scene([], [part(300, "too long", near=False)]))
    assert junk["changed"] == 0
    reject = java_client("liveview", "depth", scene([], [part(500, "out of reach (no joint solution)")]))
    assert reject["yellow"] > 100 and reject["green"] == 0
    both = java_client("liveview", "depth", scene([part(200, order=1)], [part(500, "too long")]))
    assert both["green"] > 200 and both["yellow"] > 100


@pytest.mark.parametrize(("tap", "depth"), [("depth", True), ("picture", False)])
def test_the_toggle_in_the_pictures_corner_switches_picture_and_depth(java_client, tap, depth):  # noqa: F811
    empty = json.dumps({"ok": True, "width": 848, "height": 480, "parts": [], "rejected": []})
    got = java_client("liveview", tap, empty)
    assert got["view"] is depth
    assert got["heard"] is True and got["depth"] is depth
    x, y, w, h = java_client("toggle", "640")
    assert x + w <= 640 - 8 and y >= 8 and h >= 36 and w >= 150  # top right, inside the frame, a finger wide
