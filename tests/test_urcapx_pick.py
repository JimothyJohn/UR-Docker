"""The PolyScope X 3D Pick program node's contract (0.6.0), under node: the URScript
pickscript.js writes (the same script as the PolyScope 5 node's PickScript.java — one move
sequence from the survey to the tool at the grip, no children, no gripper), the request
options it sends (read back by the Python pick server's own parser), the plane, reach and
pose math agreeing with the Python they stand for, what the screens say when the camera
computer is gone, and the behavior worker answering PolyScope's worker protocol with the
shapes its serializers read."""

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
from perceptronics.volume import BASE_RADIUS_M, Surface
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
    shape: req.shape || "box",
    gripCheck: req.gripCheck !== false,
    gripLongSide: req.gripLong === true,
    closeLook: req.closeLook !== false,
    arm: req.arm || "",
    popupOnFail: req.popup !== false,
    foundVariable: req.found || P.FOUND_VARIABLE,
    locVariable: req.loc || P.LOC_VARIABLE,
  };
  out.problem = P.problem(st);
  out.values = values;
  if (!out.problem) {
    out.tokens = [-1].concat(st.points.map((_, i) => i)).map((i) => P.tokens(st, i));
    const sc = P.script(st);
    out.before = sc.before; out.after = sc.after; out.childDepth = sc.childDepth;
    out.script = P.render(st);
    out.words = P.partText(st); out.screen = P.partWords(st);
  }
} else if (req.cmd === "settings") {
  const st = P.settings(req.params, req.app, "http://localhost");
  out.settings = st; out.problem = P.problem(st);
  out.tokens = st.points.map((_, i) => P.tokens(st, i));
} else if (req.cmd === "plane") {
  out.plane = P.plane(req.p0, req.p1, req.p2);
  out.tilt = out.plane ? P.tiltDeg(out.plane) : null;
} else if (req.cmd === "reach") {
  out.reach = P.modelReach(req.model); out.keepOut = P.KEEP_OUT_M;
} else if (req.cmd === "pose_trans") {
  out.pose = P.poseTrans(req.a, req.b); out.inv = P.poseInv(req.a);
} else if (req.cmd === "grid") {
  out.grid = P.orderGrid(req.first, req.rows, req.cols, req.nrows);
} else if (req.cmd === "corners") {
  out.corners = P.areaCorners(req.plane);
} else if (req.cmd === "svg") {
  out.svgs = {
    tile: P.svgOrderTile(req.first || "LR", req.rows || "FB", !!req.selected),
    part: P.svgPart(req.l, req.w, req.h, false),
    cylinder: P.svgPart(req.l, req.l, req.h, true),
    approach: P.svgApproach(req.approach, req.grip, req.h, Math.min(req.l, req.w), 20),
    map: P.svgReachMap(req.model || "", req.baseR, req.minR, req.maxR,
      (req.areas || []).map((a) => ({ name: a.name, corners: P.areaCorners(a.plane) })), 0),
  };
  out.farthest = (req.areas || []).map((a) => P.farthest(P.areaCorners(a.plane)));
} else if (req.cmd === "misc") {
  out.reasons = P.REASONS.map((r) => r[0]);
  out.numbers = P.NUMBERS;
  out.clamped = Object.fromEntries((req.clamp || []).map(([k, v]) => [k, P.clamp(k, v)]));
  out.ids = [P.newNodeId(), P.newNodeId()];
  out.advice = Object.fromEntries(
    (req.advise || []).map(([kind, base, detail]) => [kind + " " + base, P.advise(kind, base, detail)]));
  out.near = (req.scenes || []).map(
    (sc) => ({ drawn: P.nearMisses(sc).map((r) => r.why), summary: P.sceneSummary(sc) }));
  out.exports = Object.keys(P);
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


def test_the_script_is_one_move_sequence_from_the_survey_to_the_grip():
    out = pick()
    assert out["problem"] is None
    text = out["script"]
    assert text.isascii() and balanced(text)
    assert text.startswith("# 3D Pick 0.7.0 ")
    order = [
        "global rs_pick_found = False",
        "set_tcp(p[0, 0, 0, 0, 0, 0])",
        'socket_open("192.168.3.10", 7622, "rs_pick")',
        '"NEXT "',
        "movej([-1.370000, -0.490000, 1.610000, -2.690000, -1.570000, 0.610000]",  # the survey
        '"FIND "',
        '"LOOK "',
        "get_inverse_kin_has_solution(rs_look",
        '"REFINE "',
        "movel(rs_hover",
        "movel(rs_grip, a=0.3, v=0.05)",
        "global rs_pick_found = True",
        "global rs_pick_loc = rs_loc",
        'socket_close("rs_pick")',
        "set_tcp(rs_tcp0)",
        "popup(",
    ]
    at = [text.index(marker) for marker in order]
    assert at == sorted(at), "the stages are out of order"
    # it ends at the grip: the last motion is the descent; no children
    assert text.rindex("movel(") == text.index("movel(rs_grip")
    assert "rs_lift" not in text and "if rs_pick_found:" not in text
    assert out["childDepth"] == 0 and out["after"] == [] and "\n".join(out["before"]) + "\n" == text
    # no request line the program builds can exceed the server's 1 kB
    assert all(len(line) < 900 for line in text.splitlines())


def test_the_node_does_not_touch_the_gripper():
    """Nick, 2026-10-01: the program opens the gripper before the node and closes it after."""
    for kw in ({}, {"closeLook": False}, {"shape": "cyl", "values": {"partLengthMm": 40}}):
        text = pick(**kw)["script"]
        for gripper in ("63352", "rs_rq", "SET POS", "SET GTO", "GET OBJ", "SET ACT", "rs_held", "rs_obj"):
            assert gripper not in text, gripper
        assert "set_standard_digital_out" not in text and "closed on nothing" not in text
        assert text.count("socket_open(") == 1  # the pick server, nothing else


def test_the_script_is_the_polyscope_5_nodes_line_for_line():
    """Functional parity is literal: apart from how a result variable is assigned (`global x =`
    on PolyScope X), the version in the header and where the address is set, the two nodes
    write the same program."""
    from tests.test_urcap5 import JAVAC

    if not JAVAC:
        pytest.skip("javac is not installed")
    import tests.test_urcap5 as ps5

    spec = {
        "host": "192.168.3.10",
        "node": "a1b2c3",
        "points": [{"q": Q1}, {"q": Q2, "plane": PLANE, "area": [300, 200]}],
        "arm": "UR3",
        "polyscope": [5, 26, 1],
    }
    for extra in (
        {},
        {"closeLook": False},
        {"shape": "cyl", "gripCheck": False, "values": {"partLengthMm": 40}},
        {"gripLong": True, "values": {"fingerRoomMm": 35}},
    ):
        java = _java_pick(ps5, {**spec, **extra})

        def same(text: str) -> list[str]:
            text = re.sub(r"^# 3D Pick \S+ ", "# 3D Pick ", text, flags=re.M)
            text = re.sub(r"^(\s*)global (\w+ = )", r"\1\2", text, flags=re.M)
            return text.replace("Application > Perceptronic", "Installation > Perceptronic").splitlines()

        js = pick(arm="UR3", **extra)["script"]
        assert same(js) == same(java)


@cache
def _java_harness(javac: str, java_dir: str, harness: str, sources: tuple[str, ...]) -> Path:
    root = ROOT / "target" / "urcapx-parity-java"
    pkg = root / "src" / "io" / "advin" / "perceptronic"
    shutil.rmtree(root, ignore_errors=True)
    pkg.mkdir(parents=True)
    (pkg / "Harness.java").write_text(harness, encoding="utf-8")
    for name in sources:
        shutil.copy(Path(java_dir) / name, pkg / name)
    subprocess.run(
        [javac, "--release", "8", "-Xlint:-options", "-encoding", "UTF-8", "-d", str(root / "c")]
        + [str(f) for f in pkg.glob("*.java")],
        check=True,
        capture_output=True,
        timeout=120,
    )
    return root / "c"


def _java_pick(ps5, spec: dict) -> str:
    classes = _java_harness(ps5.JAVAC, str(ps5.JAVA), ps5.HARNESS, (*ps5.PURE_JAVA, *ps5.SCREEN_JAVA))
    proc = subprocess.run(
        ["java", "-Djava.awt.headless=true", "-cp", str(classes), "io.advin.perceptronic.Harness", "pick"]
        + [json.dumps(spec)],
        capture_output=True,
        timeout=60,
        check=True,
    )
    out = json.loads(proc.stdout.decode("utf-8"))
    assert out["problem"] is None, out["problem"]
    return out["script"]


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
        "approachMm": 35,
    }
    out = pick(values=values, order=["RL", "BF"], arm="UR3e")
    for i, tok in enumerate(out["tokens"]):
        o = parse_options(tok)
        assert o.part == PartSpec.from_mm(61, 42, 27, 20)  # long side first, however it was typed
        assert o.order == ("RL", "BF")
        # no ring: the arm is named, and the server asks its kinematics
        assert o.reach is None and o.arm == "UR3e" and "reach=" not in tok
        assert (o.grip_below_m, o.approach_m) == (0.012, 0.035)
        # the grip check is on, with 20 mm of room on each side; no stroke: the node knows no gripper
        assert (o.grip_check, o.room_m, o.across) == (True, 0.02, "short") and "stroke=" not in tok
        assert (o.node, o.locs, o.proto) == ("a1b2c3", 2, 2)
        assert o.loc == i  # tokens(-1) (the teach screen with no point yet) carries no loc
    plane = parse_options(out["tokens"][2]).surface
    want = Surface.from_pose(PLANE, (0.3, 0.2))
    assert plane.origin == pytest.approx(want.origin, abs=1e-5) and plane.area == pytest.approx(want.area)
    assert parse_options(out["tokens"][1]).surface is None  # point 1 finds the table live


def test_the_grip_check_is_on_with_20_mm_of_finger_room_and_both_are_adjustable():
    o = parse_options(pick()["tokens"][1])
    assert (o.grip_check, o.room_m) == (True, 0.02)
    o = parse_options(pick(values={"fingerRoomMm": 35})["tokens"][1])
    assert (o.grip_check, o.room_m) == (True, 0.035)
    assert parse_options(pick(gripCheck=False)["tokens"][1]).grip_check is False
    assert (
        pick(values={"partWidthMm": 300, "partLengthMm": 400})["problem"] is None
    )  # no stroke to refuse it with


def test_a_box_is_gripped_across_its_short_side_unless_the_long_side_is_ticked():
    assert parse_options(pick()["tokens"][1]).across == "short"
    out = pick(gripLong=True)
    assert parse_options(out["tokens"][1]).across == "long" and "across the long side" in out["script"]
    cyl = pick(gripLong=True, shape="cyl", values={"partLengthMm": 40})
    assert "across=" not in cyl["tokens"][1] and "long side" not in cyl["script"]


def test_a_cylinder_is_sent_by_its_diameter():
    out = pick(shape="cyl", values={"partLengthMm": 40, "partWidthMm": 12, "partHeightMm": 30})
    assert out["problem"] is None and out["words"] == "cylinder D40 x 30 mm +-25 %"
    assert out["screen"] == "Ø40 × 30 mm" and pick()["screen"] == "50 × 30 × 30 mm"
    assert parse_options(out["tokens"][1]).part == PartSpec.from_mm(40, 40, 30, 25, shape="cyl")
    assert parse_options(pick()["tokens"][1]).part.shape == "box"
    assert "unknown part shape" in pick(shape="hex")["problem"]


def test_every_request_line_it_can_send_parses():
    text = pick(arm="UR10e", shape="cyl", values={"partLengthMm": 40})["script"]
    pose = "p[0.3, -0.1, 0.12, 3.14159, 0, 0]"
    toks = re.findall(r'rs_tok = "( [^"]+)"', text)
    look_tail = re.search(r'to_str\(rs_c\), "( [^"]+)"\)\)\)\), "rs_pick"', text).group(1)
    lines = [f"NEXT {pose} node=a1b2c3 locs=2 proto=2", f"LOOK {pose} p[0.3, 0, -0.24, 0, 0, 0]{look_tail}"]
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
        ({"order": ["LR", "RL"]}, "pick order"),
        ({"arm": 'UR3" stop'}, "robot model"),
        ({"found": "1bad"}, "variable names"),
    ],
)
def test_it_refuses_what_it_cannot_generate_safely(kw, problem):
    out = pick(**kw)
    assert out["problem"] and problem in out["problem"], out["problem"]
    assert "script" not in out


def test_a_part_the_arm_cannot_get_to_sends_it_on_to_the_next_one():
    text = pick()["script"]
    assert "while (rs_ok) and (rs_try < 6) and (rs_pick_found == False):" in text  # 2 points + 3 attempts + 1
    assert "no joint solution for the approach" in text


def test_without_the_popup_a_failed_run_just_leaves_the_result_false():
    text = pick(popup=False)["script"]
    assert "popup(" not in text and "global rs_pick_found = False" in text


def test_the_nodes_travel_speed_is_fixed_and_the_last_stretch_is_slow():
    text = pick()["script"]
    assert "a=0.84, v=0.63)" in text  # movej: 60 % of 1.4 rad/s^2 and 1.05 rad/s
    assert "movel(rs_hover, a=0.36, v=0.15)" in text
    assert "movel(rs_grip, a=0.3, v=0.05)" in text


def test_without_the_closer_look_every_part_is_measured_from_its_picture_point():
    text = pick(closeLook=False)["script"]
    assert balanced(text) and '"LOOK "' not in text and "rs_look" not in text
    assert "next part already seen" not in text
    lines = text.splitlines()
    nxt = next(i for i, line in enumerate(lines) if '"NEXT "' in line)
    movej = next(i for i, line in enumerate(lines) if "movej([-1.37" in line)
    assert (
        nxt < movej and len(lines[movej]) - len(lines[movej].lstrip()) == 6
    )  # while > if rs_loc == 1 > movej
    order = ['"FIND "', '"REFINE "', "movel(rs_hover", "movel(rs_grip", "global rs_pick_found = True"]
    at = [text.index(marker) for marker in order]
    assert at == sorted(at) and "- no closer look" in lines[0]
    with_look = pick()["script"]
    assert '"LOOK "' in with_look and "next part already seen" in with_look


def test_every_default_is_within_its_own_limits_and_the_defaults_generate():
    numbers = ask(cmd="misc")["numbers"]
    # the two tabs: nothing about speeds or the gripper
    assert {n["section"] for n in numbers} == {"part", "approach"}
    assert [n["key"] for n in numbers] == [
        "partLengthMm",
        "partWidthMm",
        "partHeightMm",
        "partTolPct",
        "approachMm",
        "gripBelowTopMm",
        "fingerRoomMm",
    ]
    assert {n["key"]: n["def"] for n in numbers}["fingerRoomMm"] == 20  # Nick: 20 mm on both pick sides
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
        ("gripBelowTopMm", 12.4, 12),
        ("approachMm", 3, 5),
        ("partWidthMm", "nope", 30),
    ],
)
def test_values_are_clamped_to_their_limits(key, value, stored):
    assert ask(cmd="misc", clamp=[[key, value]])["clamped"][key] == pytest.approx(stored)


def test_orders_are_one_horizontal_and_one_vertical_direction():
    out = ask(cmd="misc")
    assert out["orders"] == [True] * 8 and out["badOrders"] == [False, False, False]


def test_the_after_picture_node_and_the_gripper_settings_are_gone():
    exports = set(ask(cmd="misc")["exports"])
    assert not exports & {
        "AFTER_TYPE",
        "afterPictureScript",
        "GRIPPER_FIELDS",
        "reachLimits",
        "GRIPPERS",
        "RQ_SOCKET",
    }
    assert not (FRONTEND / "after-node.worker.js").exists()


# -- what the screens say and draw ---------------------------------------------------------------


def test_a_lost_camera_computer_says_what_to_check_and_keeps_the_long_story_off_the_screen():
    """Nick, 2026-09-30: "check your firewall, IP address, cables ... keep the verbose errors
    in the logs". The same words as the PolyScope 5 node's."""
    base, detail = (
        "http://10.0.0.9:7621",
        "TypeError: Failed to fetch (start with --cors http://robot --bind 0.0.0.0)",
    )
    out = ask(
        cmd="misc",
        advise=[
            ["silent", base, detail],
            ["refused", base, detail],
            ["refused", "http://127.0.0.1:7621", detail],
            ["nopicture", base, "HTTP 503"],
            ["cors", base, detail],
            ["outdated", base, "HTTP 404"],
            ["badurl", "ht!tp://x", "bad"],
        ],
    )["advice"]
    silent = out[f"silent {base}"]
    assert silent["summary"] == "No answer from the camera computer at 10.0.0.9."
    assert [c.split(":")[0] for c in silent["checks"]] == ["Cables", "IP address", "Firewall"]
    assert "7621" in silent["checks"][2] and "7622" in silent["checks"][2]
    assert silent["text"] == silent["summary"] + "".join(f"\n• {c}" for c in silent["checks"])
    assert detail in silent["detail"]
    for a in out.values():  # nothing a technician types is on the operator's screen
        for noise in ("--cors", "--bind", "TypeError", "Failed to fetch", "HTTP "):
            assert noise not in a["text"], a["text"]
    assert [c.split(":")[0] for c in out[f"refused {base}"]["checks"]] == [
        "IP address",
        "Camera program",
        "Firewall",
    ]
    assert "the robot itself" in out["refused http://127.0.0.1:7621"]["checks"][0]
    nopicture = out[f"nopicture {base}"]
    assert "gives no picture" in nopicture["summary"] and "USB" in nopicture["text"]
    assert "Firewall" not in nopicture["text"]
    assert "older than this URCap" in out[f"outdated {base}"]["summary"]
    assert "is not an address" in out["badurl ht!tp://x"]["summary"]


def test_the_picture_draws_only_the_near_misses():
    def part(why=None, near=None):
        d = {"pixel": [100, 100], "corners_px": [[0, 0], [1, 0], [1, 1], [0, 1]], "why": why}
        return d if near is None else {**d, "near": near}

    scenes = [
        {"ok": True, "parts": [part(), part()], "rejected": []},
        {"ok": True, "parts": [part()], "rejected": [part("too long", True), part("too tall", False)]},
        {"ok": True, "parts": [], "rejected": [part("too tall", False)]},
        {"ok": True, "parts": [], "rejected": [part("out of reach (no joint solution)")]},  # an older cockpit
        None,
    ]
    out = ask(cmd="misc", scenes=scenes)["near"]
    assert [o["drawn"] for o in out] == [[], ["too long"], [], ["out of reach (no joint solution)"], []]
    assert [o["summary"] for o in out] == [
        "2 parts to pick",
        "1 part to pick · 1 not (yellow, with why)",
        "no part in view",
        "0 parts to pick · 1 not (yellow, with why)",
        "no part in view",
    ]


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
    from perceptronics.volume import REACH_MARGIN_M

    out = ask(cmd="reach", model=model)
    key = re.sub(r"[^A-Z0-9]", "", model.upper())
    key = key if key.endswith("E") else key + "E"
    if key in BASE_RADIUS_M and key in MODEL_REACH_M:
        assert out["reach"] == pytest.approx([BASE_RADIUS_M[key], MODEL_REACH_M[key]])
    else:
        assert out["reach"] is None
    assert out["keepOut"] == REACH_MARGIN_M  # the map's red circle is the server's keep-out


def test_settings_read_the_application_node_the_way_the_pick_node_does():
    app = {
        "cockpitUrl": "192.168.3.10",
        "areas": [
            {"name": "Bench", "p0": [0.2, -0.1, -0.27], "p1": [0.5, -0.1, -0.27], "p2": [0.2, 0.1, -0.27]},
            {"name": "Half taught", "p0": [0.2, -0.1, -0.27], "p1": None, "p2": None},
        ],
        "tipMm": 163,
        "reachInnerMm": 150,  # saved by 0.4.0: nothing reads the ring's margins any more
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
    assert (st_["arm"], st_["shape"], st_["gripCheck"], st_["closeLook"]) == ("UR3e", "box", True, True)
    assert st_["points"][0]["plane"] is None and st_["points"][1]["areaXmm"] == pytest.approx(300)
    o = parse_options(out["tokens"][1])
    assert o.surface is not None and o.surface.origin == pytest.approx([0.2, -0.1, -0.27], abs=1e-5)
    assert o.part == PartSpec.from_mm(50, 20, 30, 25) and o.reach is None and o.arm == "UR3e"
    # the node's own switches
    out = ask(
        cmd="settings", params={**params, "shape": "cyl", "gripCheck": False, "closeLook": False}, app=app
    )
    st_ = out["settings"]
    assert (st_["shape"], st_["gripCheck"], st_["closeLook"]) == ("cyl", False, False)
    o = parse_options(out["tokens"][0])
    assert o.part.is_round and o.grip_check is False
    # a node saved by 0.4.0 / 0.5.0 names a gripper: nothing reads it, the node drives none
    out = ask(cmd="settings", params={**params, "gripper": "children"}, app=app)
    assert "gripper" not in out["settings"] and out["problem"] is None
    # a point that names an untaught area, a robot model that is not a name, no cockpit
    out = ask(cmd="settings", params={**params, "points": [{"q": Q1, "area": 1}]}, app=app)
    assert "not taught" in out["problem"]
    out = ask(cmd="settings", params=params, app={**app, "robotModel": 'UR3e" stop'})
    assert out["problem"] is None and "arm=" not in out["tokens"][0]  # never sent, never injected
    out = ask(cmd="settings", params=params, app={**app, "robotModel": "Fanuc"})
    assert (
        out["problem"] is None and "arm=Fanuc" in out["tokens"][0]
    )  # the server leaves it to the controller
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
        {"l": 50, "w": 30, "h": 30, "approach": 25, "grip": 15, "model": "UR3e"}
        | {"baseR": 0.064, "minR": 0.214, "maxR": 0.5},
        {"l": 500, "w": 5, "h": 500, "approach": 200, "grip": 60, "baseR": 0.095, "minR": 0.245}
        | {"maxR": 0.0, "selected": True},
        {"l": 5, "w": 5, "h": 5, "approach": 5, "grip": 0, "baseR": 0.064, "minR": 0.214, "maxR": 0.5}
        | {"model": "UR3e", "areas": [{"name": "A & <B>", "plane": [0.2, -0.1, -0.27, 0, 0, 0.3, 0.3, 0.2]}]},
    ],
)
def test_every_drawing_is_well_formed_svg(kw):
    import xml.etree.ElementTree as ET

    out = ask(cmd="svg", **kw)
    svgs = out["svgs"]
    assert set(svgs) == {"tile", "part", "cylinder", "approach", "map"}
    for name, svg in svgs.items():
        root = ET.fromstring(svg)
        assert root.tag.endswith("svg") and root.get("viewBox"), name
        assert "NaN" not in svg and "undefined" not in svg, name

    def texts(name):
        return [t.text for t in ET.fromstring(svgs[name]).iter() if t.tag.endswith("text")]

    assert {str(max(kw["l"], kw["w"])), str(min(kw["l"], kw["w"])), str(kw["h"])} <= set(texts("part"))
    assert {f"Ø {kw['l']}", str(kw["h"])} <= set(texts("cylinder"))  # a cylinder: its diameter and height
    assert {f"approach {kw['approach']}", f"grip {kw['grip']}"} <= set(texts("approach"))
    assert not any("lift" in t for t in texts("approach"))  # the node ends at the grip
    # the arm's reach, named on the map — or said to be unknown
    if kw["maxR"] > 0:
        assert f"reach {round(kw['maxR'] * 1000)} mm" in texts("map")
        assert any("the UR3e's reach" in t for t in texts("map"))
    else:
        assert any("robot model unknown" in t for t in texts("map"))
    if kw.get("areas"):
        assert "A &amp; &lt;B&gt;" in svgs["map"]  # names are escaped, not injected
        assert out["farthest"][0] == pytest.approx(math.hypot(0.2, 0.1) + 0.0, abs=0.4)


def test_cockpit_shorthand_and_node_ids():
    out = ask(
        cmd="misc", cockpit=["", ":7621", "7777", "jetson", "jetson:8080", "http://x:1/", "https://pi.local"]
    )
    assert out["cockpit"] == [
        "http://192.168.3.20:7621",  # empty: the pick PC's factory address, not the page's host
        "http://10.0.0.5:7621",
        "http://10.0.0.5:7777",
        "http://jetson:7621",
        "http://jetson:8080",
        "http://x:1",
        "https://pi.local",
    ]
    a, b = out["ids"]
    assert re.fullmatch(r"[0-9a-f]{6}", a) and re.fullmatch(r"[0-9a-f]{6}", b) and a != b


# -- the behavior worker under PolyScope's worker protocol ---------------------------------------

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
                "type": "advin-perceptronic",
                "parentType": "advin-perceptronic",
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
        "type": "advin-perceptronic-pick",
        "version": "1.0.0",
        "allowsChildren": False,
        "parameters": {
            "nodeId": "a1b2c3",
            "points": [{"q": Q1, "area": -1}, {"q": Q2, "area": 0}],
            "orderFirst": "LR",
            "orderRows": "FB",
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
    cyl = pick_node(shape="cyl", closeLook=False, values={"partLengthMm": 40})
    by = run_worker(
        "pick-node.worker.js",
        [
            {"type": "run", "uid": "factory", "method": "factory", "args": []},
            {"type": "run", "uid": "label", "method": "programNodeLabel", "args": [node]},
            {"type": "run", "uid": "cyl-label", "method": "programNodeLabel", "args": [cyl]},
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
                "uid": "cyl-before",
                "method": "generateCodeBeforeChildren",
                "args": [cyl, {}, APP_CONTEXT],
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
                        "allowsChildren": True,
                        "parameters": {"nodeId": "zz", "values": {"partLengthMm": 9999, "speedPct": 999}},
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
    # one line in the program: a move sequence, no child tree
    assert fresh["type"] == "advin-perceptronic-pick" and fresh["allowsChildren"] is False
    assert re.fullmatch(r"[0-9a-f]{6}", fresh["parameters"]["nodeId"]) and fresh["parameters"]["points"] == []
    assert (
        fresh["parameters"]["values"]["partLengthMm"] == 50 and fresh["parameters"]["foundVariable"] is None
    )
    p = fresh["parameters"]
    assert (p["shape"], p["gripCheck"], p["gripLongSide"], p["closeLook"]) == ("box", True, False, True)
    assert "gripper" not in p and "perPointRoutine" not in p
    label = result(by, "label")
    assert (
        label[0] == {"type": "primary", "value": "50 × 30 × 30 mm"}
        and label[1]["value"] == "2 pictures · left to right, rows front to back"
    )
    assert result(by, "cyl-label")[0]["value"] == "Ø40 × 30 mm"
    assert result(by, "valid") == {"isValid": True}
    assert (
        result(by, "invalid")["isValid"] is False
        and "picture point" in result(by, "invalid")["errorMessageKey"]
    )
    assert "camera computer's address" in result(by, "noapp")["errorMessageKey"]
    before = result(by, "before")
    assert before["type"] == "$$ScriptBuilder" and before["currentIndent"] == 0
    assert before["script"].startswith("# 3D Pick 0.7.0") and before["script"].rstrip().endswith("end")
    assert balanced(before["script"])  # the whole program is here: nothing is left for after the children
    assert 'socket_open("192.168.3.10", 7622, "rs_pick")' in before["script"]
    assert "arm=UR3e" in before["script"] and "reach=" not in before["script"]
    assert "plane=p[0.20000, -0.10000, -0.27000" in before["script"]
    assert result(by, "after") == {"type": "$$ScriptBuilder", "script": "", "currentIndent": 0}
    cyl_script = result(by, "cyl-before")["script"]
    assert "shape=cyl" in cyl_script and '"LOOK "' not in cyl_script
    assert by["broken"][-1]["type"] == "error" and "picture point" in by["broken"][-1]["error"]["message"]
    assert result(by, "child") is False
    up = result(by, "upgrade")
    assert (
        up["version"] == "1.0.0"
        and up["allowsChildren"] is False
        and re.fullmatch(r"[0-9a-f]{6}", up["parameters"]["nodeId"])
    )
    # a saved value is clamped; a setting the node no longer has is dropped
    assert up["parameters"]["values"]["partLengthMm"] == 500 and "speedPct" not in up["parameters"]["values"]
    pasted = result(by, "paste")
    assert (
        re.fullmatch(r"[0-9a-f]{6}", pasted["node"]["parameters"]["nodeId"])
        and pasted["node"]["parameters"]["nodeId"] != "a1b2c3"
    )
    assert by["nope"][0]["type"] == "error" and by["nope"][0]["error"]["__error_marker"] == "$$error"
