"""The pick server the PolyScope 5 Perceptronic Pick program node talks to.

Contract, not implementation: a synthetic scene under a known flange pose goes in,
the reply the robot program parses comes out, and the fingertips it implies must
sit on the block's top centre, straight down, fingers across the short side.
Then the wire: a real socket, two requests on one connection the way the node's
script sends them, hostile lines, and many controllers at once.
"""

from __future__ import annotations

import math
import random
import socket
import threading
import time

import pytest

from perceptronics.picknode import (
    MAX_LINE,
    PickOptions,
    PickPlanner,
    PickServer,
    RequestError,
    choose,
    format_reply,
    parse_request,
)
from tests.test_pickcycle import H, K, SceneCamera, W, scene
from urctl.pose import Transform, pose_trans

TIP = 0.163
FLANGE = [0.40, 0.0, 0.50, 0.0, math.pi, 0.0]  # tool straight down, 0.5 m up
CAMERA_AT_FLANGE = [0.0] * 6  # hand-eye: the colour camera is the flange frame


def planner(frames=None, *, handeye=CAMERA_AT_FLANGE, log=None, **kw):
    """A planner over ``frames`` (list of (rgb, depth)); each request gets the next one."""
    frames = list(frames or [scene([(40, 30, 70, 54)])])
    seq = {"n": 0}

    def source(after):
        seq["n"] = max(seq["n"], after) + 1
        rgb, depth = frames[min(len(frames) - 1, seq["n"] % max(1, len(frames)))]
        return seq["n"], W, H, 3, rgb, depth, 0.001, K

    return PickPlanner(source, lambda: seq["n"], lambda: handeye, tip_m=TIP, log=log, **kw)


def reply(p: PickPlanner, line: str) -> list[float]:
    text = p.answer(line)
    assert text.startswith("(") and text.endswith(")\n")
    vals = [float(v) for v in text[1:-2].split(",")]
    assert len(vals) == 10
    return vals


def find(p, flange=FLANGE, extra=""):
    return reply(p, f"FIND p[{', '.join(str(v) for v in flange)}]{extra}")


# -- the answer -------------------------------------------------------------------------------


def test_the_reply_puts_the_fingertips_on_the_top_centre_straight_down():
    st, cx, cy, cz, *pose = find(planner())
    assert st == 1
    tips = pose_trans(pose, [0.0, 0.0, TIP, 0.0, 0.0, 0.0])
    assert tips[:3] == pytest.approx([cx, cy, cz], abs=1e-6)
    # the block's top is 0.36 m from the camera, which looks straight down from 0.5 m
    assert cz == pytest.approx(0.14, abs=0.005)
    tool_z = Transform.from_pose(pose).rotate((0.0, 0.0, 1.0))
    assert tool_z[2] == pytest.approx(-1.0, abs=1e-6)


def test_the_fingers_close_across_the_short_side():
    # a 30 x 24 px block (long along image x) → the finger travel (flange y) along image y
    st, *_, rx, ry, rz = find(planner([scene([(40, 30, 70, 54)])]))
    pose = [0, 0, 0, rx, ry, rz]
    fingers = Transform.from_pose(pose).rotate((0.0, 1.0, 0.0))
    # image x under this flange is base x (identity hand-eye, flange x = base x): the long side
    assert abs(fingers[0]) < 0.05 and abs(abs(fingers[1]) - 1.0) < 0.05


def test_the_taught_tap_chooses_the_block_else_the_one_nearest_the_centre():
    two = [scene([(20, 20, 46, 40), (100, 70, 126, 92)])]
    assert choose([], None, W, H) is None
    near_tap = find(planner(two), extra=" u=112 v=80")
    near_centre = find(planner(two))
    far = find(planner(two), extra=" u=25 v=25")
    assert near_tap[0] == far[0] == near_centre[0] == 1
    assert near_tap[1:4] != far[1:4]
    # centre (80, 60) is nearer the (113, 81) block than the (33, 30) one
    assert near_centre[1:4] == pytest.approx(near_tap[1:4], abs=1e-9)


def test_refine_finds_the_block_near_the_first_answer_and_only_that():
    p = planner()
    first = find(p)
    again = reply(
        p, f"REFINE p[{', '.join(map(str, FLANGE))}] p[{first[1]}, {first[2]}, {first[3]}, 0, 0, 0]"
    )
    assert again[0] == 1 and again[1:4] == pytest.approx(first[1:4], abs=1e-6)
    lost = reply(p, f"REFINE p[{', '.join(map(str, FLANGE))}] p[{first[1] + 0.2}, {first[2]}, 0.1, 0, 0, 0]")
    assert lost[0] == -5 and lost[1:] == [0.0] * 9


def test_lean_tilts_the_tool_outward_and_is_bounded():
    st, *_, x, y, z, rx, ry, rz = find(planner(), extra=" lean=12")
    tool_z = Transform.from_pose([x, y, z, rx, ry, rz]).rotate((0.0, 0.0, 1.0))
    assert st == 1 and math.degrees(math.acos(-tool_z[2])) == pytest.approx(12.0, abs=1e-3)  # 6-decimal reply
    assert find(planner(), extra=" lean=31")[0] == -9


@pytest.mark.parametrize(
    ("kw", "flange", "status"),
    [
        ({"stroke_m": 0.02}, FLANGE, -1),  # 24 px ≈ 43 mm short side vs a 20 mm stroke
        ({}, [0.0, 0.0, 0.5, 0.0, math.pi, 0.0], -2),  # straight above the base column
    ],
)
def test_blocks_the_gripper_cannot_take_are_refused_with_a_reason(kw, flange, status):
    assert find(planner(**kw), flange=flange)[0] == status


def test_no_hand_eye_no_frame_no_blocks():
    assert find(planner(handeye=None))[0] == -3
    empty = PickPlanner(lambda after: None, lambda: 0, lambda: CAMERA_AT_FLANGE, tip_m=TIP)
    assert find(empty)[0] == -4
    assert find(planner([scene([])]))[0] == 0


def test_the_frame_used_is_newer_than_the_request():
    asked = []
    p = PickPlanner(
        lambda after: asked.append(after) or None, lambda: 41, lambda: CAMERA_AT_FLANGE, tip_m=TIP
    )
    find(p)
    assert asked == [42]  # arrival seq 41 + 2 fresh frames - 1: a frame exposed after the arm stopped


# -- the request line --------------------------------------------------------------------------


def test_urscripts_to_str_pose_parses():
    # what to_str(get_actual_tcp_pose()) prints, and the variants URScript uses for small numbers
    req = parse_request("FIND p[0.4, -0.2, 0.3, 3.14159, 0, -1.2e-05] u=412 v=233\n")
    assert req == {
        "verb": "FIND",
        "flange": [0.4, -0.2, 0.3, 3.14159, 0.0, -1.2e-05],
        "lean": 0.0,
        "pixel": (412, 233),
        "part": None,
        "options": PickOptions(),  # protocol 1: every protocol-2 option at its default
    }
    assert "pixel" not in parse_request("FIND p[0,0,0,0,0,0] u=-1 v=-1")  # "any object"


@pytest.mark.parametrize(
    "line",
    [
        "",
        "PICK p[0,0,0,0,0,0]",
        "FIND",
        "FIND p[0,0,0,0,0]",
        "FIND p[nan,0,0,0,0,0]",
        "FIND p[1e308,0,0,0,0,0]",
        "FIND p[0,0,0,0,0,999]",
        "REFINE p[0,0,0,0,0,0]",
        "FIND p[0,0,0,0,0,0] lean=-3",
        "FIND p[0,0,0,0,0,0] part=60",
        "FIND p[0,0,0,0,0,0] part=60x40 tol=0",
        "REFINE p[0,0,0,0,0,0] p[0,0,0,0,0,0] part=900x40",
        "X" * (MAX_LINE + 1),
    ],
)
def test_malformed_requests_are_refused(line):
    with pytest.raises(RequestError):
        parse_request(line)
    assert planner().answer(line) == format_reply(-9)


def test_hostile_text_never_escapes_into_the_log():
    logged = []
    p = planner(log=lambda text, ok: logged.append(text))
    p.answer("\x1b[31mFAKE\nINFO pick FIND: found\x00 p[0,0,0,0,0,0]")
    assert logged and all("\x1b" not in t and "\n" not in t and "\x00" not in t for t in logged)


def test_random_lines_only_ever_get_a_reply():
    rng = random.Random(7622)
    p = planner()
    alphabet = "FINDREFINE p[]0123456789.,-e uv=lean\x00\x1b\té"
    for _ in range(500):
        line = "".join(rng.choice(alphabet) for _ in range(rng.randint(0, 80)))
        out = p.answer(line)
        assert out.startswith("(") and out.endswith(")\n") and len(out.split(",")) == 10


# -- over the wire ---------------------------------------------------------------------------


@pytest.fixture
def server():
    srv = PickServer("127.0.0.1", 0, planner())
    srv.start()
    yield srv.server_address[1]
    srv.stop()


def _rpc(port: int, lines: list[bytes], timeout=5.0) -> list[str]:
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
        out, buf = [], b""
        for line in lines:
            s.sendall(line)
            while b"\n" not in buf:
                chunk = s.recv(4096)
                if not chunk:
                    return out
                buf += chunk
            text, buf = buf.split(b"\n", 1)
            out.append(text.decode())
        return out


def test_find_then_refine_on_one_connection_as_the_node_sends_them(server):
    pose = ", ".join(map(str, FLANGE))
    first = _rpc(server, [f"FIND p[{pose}]\n".encode()])[0]
    c = [float(v) for v in first[1:-1].split(",")][1:4]
    a, b = _rpc(
        server,
        [f"FIND p[{pose}]\n".encode(), f"REFINE p[{pose}] p[{c[0]}, {c[1]}, {c[2]}, 0, 0, 0]\n".encode()],
    )
    assert a.startswith("(1.0") and b.startswith("(1.0")


def test_bad_bytes_get_a_refusal_and_a_huge_line_drops_the_connection(server):
    assert (
        _rpc(server, [b"\xff\xfe FIND\n", b"FIND p[0.4,0,0.5,0,3.14159,0]\n"])[0] == format_reply(-9).strip()
    )
    with socket.create_connection(("127.0.0.1", server), timeout=5) as s:
        s.sendall(b"F" * (MAX_LINE + 10) + b"\n")
        assert s.recv(4096).strip() == format_reply(-9).strip().encode()
        s.settimeout(2)
        assert s.recv(4096) == b""  # closed


def test_many_controllers_at_once_each_get_their_own_answer(server):
    pose = ", ".join(map(str, FLANGE))
    results, errors = [], []

    def one():
        try:
            results.append(_rpc(server, [f"FIND p[{pose}]\n".encode()] * 3))
        except Exception as exc:  # pragma: no cover - reported below
            errors.append(exc)

    threads = [threading.Thread(target=one) for _ in range(16)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(20)
    assert not errors and len(results) == 16
    assert all(len(r) == 3 and all(x == r[0] and x.startswith("(1.0") for x in r) for r in results)


# -- the teach screen's view of the same detector ---------------------------------------------


def test_the_cockpit_detect_route_lists_the_blocks_in_pixels():
    from perceptronics.config import PerceptionConfig
    from perceptronics.robotlink import RobotLink
    from perceptronics.webapp import ViewerApp
    from urctl.config import RobotConfig

    app = ViewerApp(
        SceneCamera(),
        config=PerceptionConfig(),
        robot=RobotLink(RobotConfig(host="fake-ur.invalid"), dry_run=True),
    )
    app.start()
    try:
        deadline = time.monotonic() + 5
        while app.latest()[1] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        out = app.pick_detect()
    finally:
        app.stop()
    assert out["ok"] and out["handeye"] and len(out["blocks"]) == 1
    (b,) = out["blocks"]
    assert abs(b["pixel"][0] - 55) <= 3 and abs(b["pixel"][1] - 42) <= 3


def test_the_teach_time_preview_gives_hover_and_grip_in_the_active_tcp():
    from perceptronics.config import PerceptionConfig
    from perceptronics.robotlink import RobotLink
    from perceptronics.webapp import ViewerApp
    from urctl.config import RobotConfig

    link = RobotLink(RobotConfig(host="fake-ur.invalid"), dry_run=True)
    app = ViewerApp(SceneCamera(), config=PerceptionConfig(), robot=link)
    app.start()
    try:
        deadline = time.monotonic() + 5
        while app.latest()[1] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        out = app.pick_preview(None, grip_below_mm=15.0, hover_mm=40.0)
    finally:
        app.stop()
    assert out["ok"], out
    tips = lambda pose: pose_trans(pose, [0.0, 0.0, link.tip_m, 0.0, 0.0, 0.0])[:3]  # noqa: E731
    # fingertips 40 mm above the top centre, and 15 mm below it, along the tool axis
    assert math.dist(tips(out["hover_pose"]), out["centre"]) == pytest.approx(0.040, abs=1e-6)
    assert math.dist(tips(out["grip_pose"]), out["centre"]) == pytest.approx(0.015, abs=1e-6)
    # the dry-run link's active TCP is the flange: the PolyScope poses equal the flange poses
    assert out["polyscope_hover_pose"] == pytest.approx(out["hover_pose"], abs=1e-9)


# -- the closer look: halfway to the block, the block beside the gripper in the picture -----------


def _camera(flange, handeye):
    return Transform.from_pose(flange).compose(Transform.from_pose(handeye))


@pytest.mark.parametrize(
    "flange",
    [
        [0.30, 0.10, 0.60, 0.0, math.pi, 0.0],  # high and straight down, block off to the side
        [0.10, 0.35, 0.45, 2.9, -1.2, 0.0],  # the UR3e's usual tilted wrist
        [-0.2, 0.30, 0.80, 0.3, 2.9, 0.1],
    ],
)
def test_the_look_pose_puts_the_camera_halfway_and_the_block_clear_of_the_gripper(flange):
    from perceptronics.picknode import LOOK_AIM_DEG, LOOK_MIN_M, gripper_bearing, look_pose

    handeye = [0.0133, 0.0553, 0.0129, 0.10, -0.164, 3.119]  # the UR3e bracket's solve
    top = [0.25, 0.30, -0.26]
    pose = look_pose(flange, top, handeye, TIP)
    assert pose is not None
    before, after = _camera(flange, handeye), _camera(pose, handeye)
    d0 = math.dist(before.translation, top)
    d1 = math.dist(after.translation, top)
    assert d1 == pytest.approx(max(LOOK_MIN_M, d0 / 2), abs=0.02) or d1 > d0 / 2  # halfway, or backed out
    assert LOOK_MIN_M - 1e-9 <= d1 <= d0 + 1e-9
    # the block is LOOK_AIM_DEG off the optical axis, on the side away from the gripper: the
    # open fingers hang in the picture, and a block in its middle is half hidden behind them
    x, y, z = after.inverse().apply(top)
    assert math.degrees(math.atan2(math.hypot(x, y), z)) == pytest.approx(LOOK_AIM_DEG, abs=1e-6)
    gx, gy = gripper_bearing(handeye, TIP)
    assert (x * gx + y * gy) / math.hypot(x, y) == pytest.approx(-1.0, abs=1e-9)
    # ... which puts every fingertip at least 20 deg from it (it was 9 deg from the nearest)
    for side in (0.0, 0.025, -0.025):
        fx, fy, fz = Transform.from_pose(handeye).inverse().apply((0.0, side, TIP))
        cos = (x * fx + y * fy + z * fz) / (math.hypot(x, y, z) * math.hypot(fx, fy, fz))
        assert math.degrees(math.acos(cos)) > 20.0
    # on the line from where the camera was: it moved toward the block, not sideways
    away0 = [(before.translation[i] - top[i]) / d0 for i in range(3)]
    away1 = [(after.translation[i] - top[i]) / d1 for i in range(3)]
    assert away0 == pytest.approx(away1, abs=1e-9)
    # the fingertips stay clear of the top
    tip = Transform.from_pose(pose).apply((0.0, 0.0, TIP))
    assert tip[2] >= top[2] + 0.06 - 1e-9


def test_a_camera_already_close_is_not_moved_nearer_than_the_d435_can_see():
    from perceptronics.picknode import LOOK_MIN_M, look_pose

    top = [0.40, 0.0, 0.0]
    flange = [0.40, 0.0, LOOK_MIN_M + 0.02, 0.0, math.pi, 0.0]  # camera 0.27 m straight above
    pose = look_pose(flange, top, CAMERA_AT_FLANGE, 0.05)
    assert pose is not None
    assert math.dist(Transform.from_pose(pose).translation, top) >= LOOK_MIN_M - 1e-9


def test_a_camera_nearer_than_the_d435_can_see_backs_out_to_look():
    """The next pick starts from wherever the last one ended — often right over the table."""
    from perceptronics.picknode import LOOK_MIN_M, look_pose

    top = [0.40, 0.0, 0.0]
    flange = [0.40, 0.0, 0.12, 0.0, math.pi, 0.0]  # camera 0.12 m above the part: blind
    pose = look_pose(flange, top, CAMERA_AT_FLANGE, 0.05)
    assert pose is not None
    assert math.dist(Transform.from_pose(pose).translation, top) >= LOOK_MIN_M - 1e-9


def test_a_gripper_on_the_optical_axis_gives_no_side_to_aim_away_from():
    from perceptronics.picknode import gripper_bearing, look_pose

    assert gripper_bearing(CAMERA_AT_FLANGE, TIP, stroke_m=0.0) == (0.0, 0.0)
    top = [0.40, 0.0, 0.0]
    pose = look_pose([0.40, 0.0, 0.7, 0.0, math.pi, 0.0], top, CAMERA_AT_FLANGE, 0.05, stroke_m=0.0)
    x, y, _ = Transform.from_pose(pose).inverse().apply(top)
    assert math.hypot(x, y) < 1e-9  # dead centre, as before


def test_no_look_pose_when_the_tool_would_tip_past_the_limit():
    from perceptronics.picknode import look_pose

    # camera level with the block, 0.5 m away sideways: aiming at it means a horizontal tool
    flange = [0.9, 0.0, 0.0, 0.0, math.pi / 2, 0.0]
    assert look_pose(flange, [0.4, 0.0, 0.0], CAMERA_AT_FLANGE, TIP) is None


def test_look_over_the_socket_answers_with_the_pose_and_needs_a_hand_eye():
    line = f"LOOK p[{', '.join(str(v) for v in FLANGE)}] p[0.45, 0.02, 0.0, 0, 0, 0]"
    vals = reply(planner(), line)
    assert vals[0] == 1.0 and vals[1:4] == pytest.approx([0.45, 0.02, 0.0])
    assert reply(planner(handeye=None), line)[0] == -3.0


# -- LOG: the program's own trace ------------------------------------------------------------------


def test_log_lines_are_recorded_cleaned_and_never_answered():
    heard = []
    p = planner(log=lambda text, ok: heard.append(text))
    assert p.answer("LOG FIND status 1\n") == ""
    assert p.answer("LOG " + "x\x1b[2J\x00" * 200 + "\n") == ""
    assert heard[0] == "robot: FIND status 1"
    assert heard[1].startswith("robot: x?[2J?") and len(heard[1]) <= len("robot: ") + 240
    assert all(" " <= c <= "~" for c in heard[1])
    # a LOG line is never answered, even malformed: a stray reply would be read as FIND's
    for odd in ("LOG\n", "log lower case\n", "LOG " + "y" * 5000 + "\n", "LOG \xff\xfe\n"):
        assert p.answer(odd) == ""


def test_log_then_find_on_one_connection_reads_the_find_answer(server):
    with socket.create_connection(("127.0.0.1", server), timeout=10) as s:
        f = s.makefile("rwb")
        f.write(b"LOG start\n")
        f.write(b"LOG \xff\xfe not ascii\n")
        f.write(f"FIND p[{', '.join(str(v) for v in FLANGE)}]\n".encode())
        f.flush()
        first = f.readline().decode()
    assert first.startswith("(1.0")  # LOG sent nothing back, so the first reply is FIND's


# -- the part's rough size: FIND and REFINE only consider candidates that size -----------------

# the scene's default block measures 54 x 40 mm and stands 40 mm proud (tests/test_partspec.py)
PART = " part=54x40x40 tol=15"
TWO_SIZES = [scene([(20, 20, 50, 44), (100, 68, 124, 92)])]  # 54 x 43 mm, and 43 x 43 mm


def test_the_part_size_overrules_the_tap():
    # tapped on the 43 x 43 one; only the 54 x 40 one is the part
    near_small = find(planner(TWO_SIZES), extra=" u=112 v=80")
    as_part = find(planner(TWO_SIZES), extra=" u=112 v=80" + PART)
    assert near_small[0] == as_part[0] == 1
    assert as_part[1:4] != near_small[1:4]
    assert as_part[1:4] == pytest.approx(find(planner(TWO_SIZES), extra=" u=35 v=32")[1:4], abs=1e-9)


def test_nothing_the_size_of_the_part_is_its_own_status_and_says_what_it_saw():
    logged = []
    st = find(planner(log=lambda text, ok: logged.append(text)), extra=" part=100x80")[0]
    assert st == -7
    assert any("not the part" in t and "too short" in t for t in logged), logged
    assert find(planner([scene([])]), extra=" part=100x80")[0] == 0  # nothing at all: still 0


def test_a_flat_look_alike_is_not_the_part():
    flat = [scene([(40, 30, 70, 54)], height=0.008)]
    assert find(planner(flat), extra=PART)[0] == -7
    assert find(planner(flat))[0] == 1  # without a spec the sticker would have been "picked"


def test_refine_holds_the_part_to_the_same_size():
    p = planner()
    first = find(p, extra=PART)
    assert first[0] == 1
    near = f"p[{first[1]}, {first[2]}, {first[3]}, 0, 0, 0]"
    flange = f"p[{', '.join(map(str, FLANGE))}]"
    assert reply(p, f"REFINE {flange} {near}{PART} lean=0")[0] == 1
    assert reply(p, f"REFINE {flange} {near} part=100x80 lean=12")[0] == -5


def test_the_node_teach_screen_sees_the_rejects_and_why():
    from perceptronics.partspec import PartSpec
    from perceptronics.picknode import detect_report

    rgb, depth = TWO_SIZES[0]
    frame = (7, W, H, 3, rgb, depth, 0.001, K)
    out = detect_report(frame, pick_port=7622, handeye=True, tip_m=TIP, part=PartSpec.from_mm(54, 40, 40, 15))
    assert out["part"] == {
        "length_mm": 54.0,
        "width_mm": 40.0,
        "height_mm": 40.0,
        "tol_pct": 15.0,
        "shape": "box",
    }
    assert [b["size_mm"] for b in out["blocks"]] == [[54, 43]] and out["blocks"][0]["height_mm"] == 40
    assert [(r["size_mm"], r["why"]) for r in out["rejected"]] == [([43, 43], "too short")]
    plain = detect_report(frame, pick_port=7622, handeye=True, tip_m=TIP)
    assert plain["part"] is None and len(plain["blocks"]) == 2 and plain["rejected"] == []
