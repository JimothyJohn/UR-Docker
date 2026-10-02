"""Fault campaign for the pick cycle: every failure that can be simulated offline,
thrown at the routine through a fake cockpit (canned scene) and the fake UR
controller (urctl's test double). The contract under test: the routine never
hangs, never leaves the gripper closed on a failure, says what happened, and
keeps going where it safely can."""

from __future__ import annotations

import math

import pytest

from perceptronics.pickcycle import Block, CockpitError, PickCycle, add_pick_cycle_args
from tests.test_pickcycle import H, K, W, scene
from tests.test_urctl import FakeController
from urctl.config import RobotConfig
from urctl.robot import Robot

HANDEYE = [0.0, 0.06, 0.0, 0.0, 0.0, math.pi]  # camera 60 mm off the flange, looking along the tool


class FakeCockpit:
    """Vision only: /api/robot (hand-eye), /api/rgbd (the scene), plus whatever
    a test wants to fail."""

    def __init__(self, blocks=((40, 30, 70, 54),), *, handeye=HANDEYE, no_frame=False):
        self.rgb, self.depth = scene(list(blocks))
        self.handeye = handeye
        self.no_frame = no_frame
        self.posts: list[tuple[str, dict]] = []
        self.base = "fake://cockpit"

    def get(self, path):
        if path == "/api/robot":
            he = {"flange_to_color_pose": self.handeye} if self.handeye else {}
            return {"ok": True, "robot": {"host": "fake-ur.invalid", "handeye": he}}
        raise AssertionError(path)

    def post(self, path, body=None):
        self.posts.append((path, body or {}))
        return {"ok": False, "error": f"unexpected {path}"}

    def frame(self):
        if self.no_frame:
            raise CockpitError("no RGB-D frame from the cockpit")
        hdr = {"intrinsics": K, "depth_scale_m": 0.001, "width": W, "height": H}
        return hdr, W, H, 3, self.rgb, self.depth


@pytest.fixture
def rig(monkeypatch):
    fake = FakeController().install(monkeypatch)
    fake.remote = True
    fake.gripper_object = 140
    robot = Robot(RobotConfig(host="fake-ur.invalid"))
    events: list[str] = []
    cycle = PickCycle(FakeCockpit(), robot=robot, say_fn=lambda ev: events.append(ev["text"]))
    return fake, cycle, events


def test_happy_path_is_one_program_per_block(rig):
    fake, cycle, events = rig
    out = cycle.run()
    assert out["ok"] and [r["stage"] for r in out["results"]] == ["replaced"]
    programs = [b for b in fake.primary_sends if "urctl/path/" in b]
    assert len(programs) == 1  # hover → edge → grasp+close → lift → set+open → clear, one submission
    assert programs[0].count("movel(") == 6 and programs[0].count("SET POS") == 2
    assert any("held at Robotiq 140" in e for e in events)


def test_local_control_refuses_before_any_motion(rig):
    fake, cycle, events = rig
    fake.remote = False
    with pytest.raises(CockpitError, match="Local control"):
        cycle.run()
    assert not [b for b in fake.primary_sends if "movel(" in b]


def test_no_handeye_refuses_before_any_motion(rig, monkeypatch):
    fake, cycle, events = rig
    cycle.cockpit.handeye = None
    with pytest.raises(CockpitError, match="hand-eye"):
        cycle.run()
    assert not [b for b in fake.primary_sends if "movel(" in b]


def test_camera_stall_is_an_error_not_a_hang(rig):
    fake, cycle, events = rig
    cycle.cockpit.no_frame = True
    with pytest.raises(CockpitError, match="no RGB-D frame"):
        cycle.run()


def test_gripper_not_activated_gets_activated_first(rig):
    fake, cycle, events = rig
    # the fake daemon answers ACT 1 STA 3 normally; make the status say inactive once
    original = fake._primary_collect
    calls = {"n": 0}

    def collect(host, port, payload, **kw):
        body = payload.decode()
        if "urctl/rq" in body and "SET ACT" not in body and calls["n"] == 0:
            calls["n"] += 1
            return (
                b"urctl/rq/STA=STA 0\nurctl/rq/ACT=ACT 0\nurctl/rq/POS=POS 3\nurctl/rq/PRE=PRE 0\n"
                b"urctl/rq/OBJ=OBJ 0\nurctl/rq/FLT=FLT 0\nurctl/rq/done=1\n"
            )
        return original(host, port, payload, **kw)

    fake._primary_collect = collect
    import urctl.transport as tr

    tr.send_and_collect = collect
    cycle.run()
    assert any("SET ACT 1" in b for b in fake.primary_sends), "activation cycle was not sent"
    assert any("activation cycle" in e for e in events)


def test_closing_on_nothing_is_reported_and_the_gripper_ends_open(rig):
    fake, cycle, events = rig
    fake.gripper_object = None  # nothing between the fingers
    out = cycle.run()
    assert [r["ok"] for r in out["results"]] == [False]
    assert any("closed on nothing" in e for e in events)
    last_grip = [b for b in fake.primary_sends if "SET POS" in b][-1]
    assert last_grip.rfind("SET POS 0") > last_grip.rfind("SET POS 255")  # the program ends with an open


def test_protective_stop_inside_the_program_recovers_and_moves_on(rig):
    fake, cycle, events = rig
    original = fake._primary_collect
    state = {"stopped": False}

    def collect(host, port, payload, **kw):
        body = payload.decode()
        if "urctl/path/" in body and not state["stopped"]:
            state["stopped"] = True
            fake.safety_mode = "PROTECTIVE_STOP"
            return b"urctl/path/gripper=True\n"  # the program died before any leg landed
        return original(host, port, payload, **kw)

    import urctl.transport as tr

    tr.send_and_collect = collect
    out = cycle.run()
    assert [r["stage"] for r in out["results"]] == ["program"]
    assert any("Protective stop" in e for e in events)
    assert any("unlock protective stop" in s for s in fake.dashboard_sends)  # bring_up ran
    assert cycle.log[-1]["text"] == "Done"


def test_block_wider_than_the_stroke_is_skipped(rig):
    fake, cycle, events = rig
    blk = Block(0, [0.3, 0.2, -0.1], 0.0, 0.06, 0.052, (0, 0), 500)
    res = cycle.cycle_block(blk)
    assert res["stage"] == "stroke" and not [b for b in fake.primary_sends if "movel(" in b]


def test_near_column_blocks_are_skipped_when_asked(rig, monkeypatch):
    fake, cycle, events = rig
    cycle.min_radius_m = 0.6  # everything the fake scene finds sits inside this
    out = cycle.run()
    assert [r["stage"] for r in out["results"]] == ["radius"]
    assert not [b for b in fake.primary_sends if "urctl/path/" in b]


def test_bail_out_stops_the_program_first_then_opens_then_backs_off(rig):
    fake, cycle, events = rig
    n_dash, n_prim = len(fake.dashboard_sends), len(fake.primary_sends)
    cycle.bail_out()
    assert any("stop" in s.splitlines() for s in fake.dashboard_sends[n_dash:]), (
        "Dashboard stop must come first"
    )
    sent = fake.primary_sends[n_prim:]
    assert any("SET POS 0" in b for b in sent) and any("movel(" in b for b in sent)
    assert sent.index(next(b for b in sent if "SET POS 0" in b)) < sent.index(
        next(b for b in sent if "movel(" in b)
    )
    assert any("Backed off" in e for e in events)


def test_interrupt_during_the_run_reaches_the_bail_out(monkeypatch, tmp_path):
    from perceptronics import pickcycle as pc

    calls = []
    monkeypatch.setattr(pc.PickCycle, "run", lambda self, **kw: (_ for _ in ()).throw(KeyboardInterrupt()))
    monkeypatch.setattr(pc.PickCycle, "bail_out", lambda self: calls.append("bail"))
    monkeypatch.setattr(
        pc.Cockpit, "get", lambda self, path: {"ok": True, "robot": {"host": "fake-ur.invalid"}}
    )
    import argparse

    ap = argparse.ArgumentParser()
    pc.add_pick_cycle_args(ap)
    args = ap.parse_args(["--robot-host", "fake-ur.invalid"])
    assert pc.run_pick_cycle(args) == 130 and calls == ["bail"]


def test_phantom_above_the_surface_is_filtered_out(rig):
    fake, cycle, events = rig
    # three real blocks on the floor plus one "block" 60 mm closer to the camera (a strap on the rail)
    rgb, depth = scene([(20, 20, 45, 40), (60, 20, 85, 40), (100, 20, 125, 40)])
    rgb2, depth2 = scene([(100, 70, 125, 90)], floor_z=0.34, height=0.04)  # its top at 0.30 instead of 0.36
    rgb = bytearray(rgb)
    depth = bytearray(depth)
    for y in range(70, 90):
        for x in range(100, 125):
            i = (y * W + x) * 3
            rgb[i : i + 3] = rgb2[i : i + 3]
            depth[2 * (y * W + x) : 2 * (y * W + x) + 2] = depth2[2 * (y * W + x) : 2 * (y * W + x) + 2]
    cycle.cockpit.rgb, cycle.cockpit.depth = bytes(rgb), bytes(depth)
    blocks = cycle.survey()
    assert len(blocks) == 3 and all(abs(b.centre_base[2] - blocks[0].centre_base[2]) < 0.02 for b in blocks)


def test_bad_survey_pose_is_refused_by_the_envelope_not_sent(rig):
    fake, cycle, events = rig
    out = cycle.run(survey_poses=[[5.0, 0.0, 0.2, 0.0, math.pi, 0.0]])  # 5 m out: beyond any arm
    assert out["ok"]  # the run completes: the pose was refused, the survey happened from here
    assert any("survey move refused" in e for e in events)


def test_blocks_beyond_the_working_reach_are_skipped_before_any_program(rig):
    fake, cycle, events = rig
    # 0.5 m reach − 50 mm margin + what the Hand-E buys leaned 24° out (0.163·sin 24° = 66 mm)
    # → 0.516 m working limit; inside it the controller's IK decides per block
    fake.model = "UR3"
    near = Block(0, [0.34, 0.34, -0.25], 0.0, 0.045, 0.028, (0, 0), 500)  # 0.48 m out
    assert cycle._out_of_band(near) is None
    blk = Block(0, [0.39, 0.39, -0.25], 0.0, 0.045, 0.028, (0, 0), 500)  # 0.55 m out
    assert "beyond this arm" in (cycle._out_of_band(blk) or "")
    cycle.max_radius_m = 0.6
    assert cycle._out_of_band(blk) is None
    assert not [b for b in fake.primary_sends if "urctl/path/" in b]


def test_a_program_the_controller_refuses_is_reported_and_leaves_the_gripper_open(rig):
    fake, cycle, events = rig
    import urctl.transport as tr

    original = fake._primary_collect

    def collect(host, port, payload, **kw):
        if "urctl/path/" in payload.decode():
            return b"urctl/path/gripper=True\n"  # the controller ran nothing: no leg marker, no done
        return original(host, port, payload, **kw)

    tr.send_and_collect = collect
    out = cycle.run()
    assert [r["stage"] for r in out["results"]] == ["refused"]
    assert any("never ran a leg" in e for e in events)
    assert "SET POS 0" in [b for b in fake.primary_sends if "urctl/rq" in b][-1]


def test_blocks_hugging_the_base_column_are_skipped_by_default(rig):
    """Regression: the minimum radius was opt-in (0); on the UR3e a block 0.19 m from the
    column protective-stopped twice. Default 0.2 m (Nick, 2026-09-27), CLI and class."""
    fake, cycle, events = rig
    assert PickCycle().min_radius_m == 0.2
    ap = __import__("argparse").ArgumentParser()
    add_pick_cycle_args(ap)
    assert ap.parse_args([]).min_radius_m == 0.2
    near = Block(0, [0.13, 0.13, -0.25], 0.0, 0.045, 0.028, (0, 0), 500)  # 0.18 m out
    assert "too close" in (cycle._out_of_band(near) or "")
    ok = Block(0, [0.16, 0.16, -0.25], 0.0, 0.045, 0.028, (0, 0), 500)  # 0.23 m out
    assert cycle._out_of_band(ok) is None


class NoGripperRouteCockpit(FakeCockpit):
    """A cockpit that predates POST /api/robot/gripper (answers 404 "no route")."""

    def post(self, path, body=None):
        self.posts.append((path, body or {}))
        if path == "/api/robot/gripper":
            return {"ok": False, "error": f"no route {path}"}
        return super().post(path, body)


def test_gripper_fallback_drives_the_cockpits_robot_without_uv(monkeypatch, tmp_path):
    """Regression (2026-09-28): the fallback shelled out to `python3 -m urctl gripper`, so
    any install without it on PATH (the pip-installed Jetson image) couldn't grip."""
    import urctl.transport as tr

    fake = FakeController().install(monkeypatch)
    fake.gripper_object = 140
    hosts: list[str] = []

    def collect(host, port, payload, **kw):
        hosts.append(host)
        return fake._primary_collect(host, port, payload, **kw)

    monkeypatch.setattr(tr, "send_and_collect", collect)
    monkeypatch.setenv("PATH", str(tmp_path))  # no urctl script
    cycle = PickCycle(NoGripperRouteCockpit(), say_fn=lambda ev: None)

    r = cycle._gripper("close")

    assert r["ok"] and r["object_detected"], r
    assert hosts and set(hosts) == {"fake-ur.invalid"}  # the robot the cockpit is linked to
    assert any("SET POS 255" in b for b in fake.gripper_sends)
    assert any("SET FOR 80" in b for b in fake.gripper_sends)  # the cycle's force, not the CLI's 100
