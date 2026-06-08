"""Unit tests for urctl.guided.GuidedSession.

The session is tested against a FakeRobot at the Robot-facade seam, so these
exercise the guided-build logic (confirm → execute → record, and the skip
paths) without URScript, sockets, or a simulator. The pendant-confirm primitive
itself (Robot.confirm_on_pendant) is covered in test_urctl.py against the
transport fake.
"""

from __future__ import annotations

import gzip
from xml.etree import ElementTree as ET

import pytest

from urctl.guided import GuidedSession


class FakeRobot:
    """Programmable stand-in for Robot. ``confirm_answers`` is consumed one per
    confirm call (True/False/None); actions record their calls and report ok."""

    def __init__(self, confirm_answers, *, move_ok=True, io_ok=True, joints_after=None, reteach=None):
        self._answers = list(confirm_answers)
        self.move_ok = move_ok
        self.io_ok = io_ok
        self.joints_after = joints_after or [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        self._reteach = reteach  # dict result for reteach_in_freedrive, or None
        self.calls: list[tuple] = []

    def confirm_on_pendant(self, prompt, *, timeout=120.0):
        self.calls.append(("confirm", prompt, timeout))
        ans = self._answers.pop(0) if self._answers else None
        return {"action": "confirm_on_pendant", "ok": True, "confirmed": ans}

    def reteach_in_freedrive(self, prompt, *, timeout=300.0):
        self.calls.append(("reteach", prompt, timeout))
        r = self._reteach
        if isinstance(r, list):  # a queue: one answer per call (for the loop)
            return r.pop(0) if r else {"confirmed": None, "joints": None}
        return r or {"confirmed": None, "joints": None}

    def move_joints(self, target, **kwargs):
        self.calls.append(("move_joints", list(target), kwargs))
        return {"action": "move_joints", "ok": self.move_ok, "landed": list(target)}

    def move_tcp(self, pose, *, relative=False, **kwargs):
        self.calls.append(("move_tcp", list(pose), relative, kwargs))
        return {"action": "move_tcp", "ok": self.move_ok}

    def set_digital_output(self, pin, value):
        self.calls.append(("set_output", pin, value))
        return {"action": "set_digital_output", "ok": self.io_ok}

    def get_state(self, **kwargs):
        return {"joints": list(self.joints_after)}


def _nodes(session) -> list[str]:
    """Tag names of the program-tree children of a session's program."""
    root = ET.fromstring(gzip.decompress(session.program.to_bytes()))
    mp = root.find("./children/MainProgram/children")
    return [c.tag for c in mp]


class TestApprovePath:
    def test_move_joints_approved_executes_and_records(self):
        r = FakeRobot([True])
        s = GuidedSession(r, "t")
        res = s.move_joints([0, -1.57, 0, -1.57, 0, 0])
        assert res.confirmed is True and res.executed and res.recorded
        # Robot saw a confirm THEN a move (announce-before-act ordering).
        assert [c[0] for c in r.calls] == ["confirm", "move_joints"]
        # Each recorded step is annotated: a Comment precedes its Move node.
        assert _nodes(s) == ["Comment", "Move"]

    def test_move_tcp_records_achieved_joints_as_waypoint(self):
        r = FakeRobot([True], joints_after=[1, 1, 1, 1, 1, 1])
        s = GuidedSession(r, "t")
        res = s.move_tcp([0, 0, -0.05, 0, 0, 0], relative=True)
        assert res.executed and res.recorded
        root = ET.fromstring(gzip.decompress(s.program.to_bytes()))
        move = root.find("./children/MainProgram/children/Move")
        assert move.get("motionType") == "MoveL"
        angles = move.find("./children/Waypoint/position/JointAngles").get("angles")
        assert angles == "1.0, 1.0, 1.0, 1.0, 1.0, 1.0"  # the read-back pose

    def test_set_output_approved_records_set_node(self):
        r = FakeRobot([True])
        s = GuidedSession(r, "t")
        res = s.set_output(0, True)
        assert res.executed and res.recorded
        assert _nodes(s) == ["Comment", "Set"]


class TestRejectAndSkipPaths:
    def test_rejected_step_neither_executes_nor_records(self):
        r = FakeRobot([False])
        s = GuidedSession(r, "t")
        res = s.move_joints([0, 0, 0, 0, 0, 0])
        assert res.confirmed is False and not res.executed and not res.recorded
        assert [c[0] for c in r.calls] == ["confirm"]  # no move attempted
        assert _nodes(s) == []

    def test_no_answer_skips(self):
        r = FakeRobot([None])
        s = GuidedSession(r, "t")
        res = s.move_tcp([0, 0, 0.05, 0, 0, 0])
        assert res.confirmed is None and not res.executed and not res.recorded
        assert _nodes(s) == []

    def test_approved_but_unsafe_move_not_recorded(self):
        # Operator says yes, but the safety envelope refuses (move ok=False).
        r = FakeRobot([True], move_ok=False)
        s = GuidedSession(r, "t")
        res = s.move_joints([99, 0, 0, 0, 0, 0])
        assert res.confirmed is True and not res.executed and not res.recorded
        assert _nodes(s) == []

    def test_approved_io_failure_not_recorded(self):
        r = FakeRobot([True], io_ok=False)
        s = GuidedSession(r, "t")
        res = s.set_output(0, True)
        assert res.confirmed is True and not res.executed and not res.recorded
        assert _nodes(s) == []


class TestSessionMechanics:
    def test_descriptions_flow_into_the_prompt(self):
        r = FakeRobot([True])
        s = GuidedSession(r, "t")
        s.move_joints([0, 0, 0, 0, 0, 0], desc="Go to staging")
        prompt = r.calls[0][1]
        assert "Go to staging" in prompt and "About to:" in prompt
        # The pendant buttons are Yes/No/Cancel — the prompt spells out their effect.
        assert "Yes" in prompt and "Cancel" in prompt

    def test_freedrive_prompt_names_the_buttons_not_ok(self):
        r = FakeRobot([True], reteach={"confirmed": True, "joints": [1, 1, 1, 1, 1, 1]})
        s = GuidedSession(r, "t", freedrive=True)
        s.move_joints([0, 0, 0, 0, 0, 0], desc="pick")
        reteach_prompt = next(c[1] for c in r.calls if c[0] == "reteach")
        assert "Yes" in reteach_prompt and "Cancel" in reteach_prompt
        assert "OK" not in reteach_prompt

    def test_confirm_timeout_is_passed_through(self):
        r = FakeRobot([True])
        s = GuidedSession(r, "t", confirm_timeout=42.0)
        s.move_joints([0, 0, 0, 0, 0, 0])
        assert r.calls[0][2] == 42.0

    def test_comment_records_without_confirm_or_action(self):
        r = FakeRobot([])  # no confirm answers needed
        s = GuidedSession(r, "t")
        s.comment("Pick sequence below")
        assert r.calls == []
        assert _nodes(s) == ["Comment"]

    def test_waypoint_names_come_from_descriptions_and_are_unique(self):
        r = FakeRobot([True, True])
        s = GuidedSession(r, "t")
        s.move_joints([0, 0, 0, 0, 0, 0], desc="pick approach")
        s.move_joints([1, 1, 1, 1, 1, 1], desc="grasp")
        root = ET.fromstring(gzip.decompress(s.program.to_bytes()))
        names = [w.get("name") for w in root.iter("Waypoint")]
        assert names == ["PickApproach", "Grasp"]

    def test_collision_on_distinct_poses_gets_numeric_suffix(self):
        r = FakeRobot([True, True])
        s = GuidedSession(r, "t")
        s.move_joints([0, 0, 0, 0, 0, 0], desc="step")
        s.move_joints([1, 1, 1, 1, 1, 1], desc="step")  # same desc, different pose
        names = [w.get("name") for w in ET.fromstring(gzip.decompress(s.program.to_bytes())).iter("Waypoint")]
        assert names == ["Step", "Step_2"]

    def test_duplicate_position_reuses_earlier_name(self):
        r = FakeRobot([True, True, True])
        s = GuidedSession(r, "t")
        s.move_joints([0, 0, 0, 0, 0, 0], desc="home")
        s.move_joints([1, 2, 3, 4, 5, 6], desc="work")
        s.move_joints([0, 0, 0, 0, 0, 0], desc="back home")  # same pose as Home
        names = [w.get("name") for w in ET.fromstring(gzip.decompress(s.program.to_bytes())).iter("Waypoint")]
        assert names == ["Home", "Work", "Home"]  # returning point reuses the name

    def test_every_recorded_line_gets_a_comment_with_its_description(self):
        r = FakeRobot([True, True, True])
        s = GuidedSession(r, "t")
        s.move_joints([0, 0, 0, 0, 0, 0], desc="approach")
        s.set_output(0, True, desc="close gripper")
        s.move_tcp([0, 0, -0.05, 0, 0, 0], desc="lower onto part")
        assert _nodes(s) == ["Comment", "Move", "Comment", "Set", "Comment", "Move"]
        comments = [
            c.get("comment") for c in ET.fromstring(gzip.decompress(s.program.to_bytes())).iter("Comment")
        ]
        assert comments == ["approach", "close gripper", "lower onto part"]

    def test_skipped_step_adds_no_comment(self):
        r = FakeRobot([False])  # rejected → nothing recorded, no comment
        s = GuidedSession(r, "t")
        s.move_joints([0, 0, 0, 0, 0, 0], desc="approach")
        assert _nodes(s) == []


class TestFreedriveReteach:
    def test_freedrive_records_hand_guided_pose(self):
        # Operator approves the move, then hand-guides to a different pose and OKs.
        r = FakeRobot([True], reteach={"confirmed": True, "joints": [9, 9, 9, 9, 9, 9]})
        s = GuidedSession(r, "t", freedrive=True)
        res = s.move_joints([0, 0, 0, 0, 0, 0], desc="pick")
        assert res.recorded
        assert [c[0] for c in r.calls] == ["confirm", "move_joints", "reteach"]
        angles = ET.fromstring(gzip.decompress(s.program.to_bytes())).find(".//Waypoint/position/JointAngles")
        assert angles.get("angles") == "9.0, 9.0, 9.0, 9.0, 9.0, 9.0"  # the adjusted pose

    def test_freedrive_relative_move_records_adjusted_readback(self):
        r = FakeRobot([True], reteach={"confirmed": True, "joints": [2, 2, 2, 2, 2, 2]})
        s = GuidedSession(r, "t", freedrive=True)
        s.move_tcp([0, 0, -0.05, 0, 0, 0], relative=True, desc="lower")
        angles = ET.fromstring(gzip.decompress(s.program.to_bytes())).find(".//Waypoint/position/JointAngles")
        assert angles.get("angles") == "2.0, 2.0, 2.0, 2.0, 2.0, 2.0"

    def test_freedrive_decline_executes_but_does_not_record(self):
        r = FakeRobot([True], reteach={"confirmed": False, "joints": None})
        s = GuidedSession(r, "t", freedrive=True)
        res = s.move_joints([0, 0, 0, 0, 0, 0], desc="pick")
        assert res.executed and not res.recorded
        assert _nodes(s) == []

    def test_no_freedrive_means_no_reteach_call(self):
        r = FakeRobot([True], reteach={"confirmed": True, "joints": [9, 9, 9, 9, 9, 9]})
        s = GuidedSession(r, "t")  # freedrive off (default)
        s.move_joints([0, 0, 0, 0, 0, 0], desc="pick")
        assert [c[0] for c in r.calls] == ["confirm", "move_joints"]  # no reteach

    def test_summary_tallies_outcomes(self):
        r = FakeRobot([True, False, None])
        s = GuidedSession(r, "t")
        s.move_joints([0, 0, 0, 0, 0, 0])  # recorded
        s.move_joints([0, 0, 0, 0, 0, 0])  # rejected
        s.move_joints([0, 0, 0, 0, 0, 0])  # no answer
        summary = s.summary()
        assert summary == {"steps": 3, "recorded": 1, "rejected": 1, "no_answer": 1}

    def test_save_writes_loadable_gzip(self, tmp_path):
        r = FakeRobot([True])
        s = GuidedSession(r, "demo")
        s.move_joints([0, -1.57, 0, -1.57, 0, 0])
        out = s.save(tmp_path / "demo.urp")
        assert out.exists() and out.read_bytes()[:2] == b"\x1f\x8b"


class TestGuidedConsoleDispatch:
    """The console line parser (urctl.cli.run_guided_command) over a session."""

    def _session(self, answers):
        return GuidedSession(FakeRobot(answers), "t")

    def test_movej_line(self):
        from urctl.cli import run_guided_command

        s = self._session([True])
        res = run_guided_command(s, "movej 0 -1.57 0 -1.57 0 0 ; go staging")
        assert res["kind"] == "step" and res["result"].recorded
        assert res["result"].desc == "go staging"

    def test_movetcp_rel_line_sets_relative(self):
        from urctl.cli import run_guided_command

        s = self._session([True])
        run_guided_command(s, "movetcp 0 0 -0.05 0 0 0 rel ; lower")
        # FakeRobot records (verb, pose, relative, kwargs) for move_tcp.
        tcp_call = next(c for c in s.robot.calls if c[0] == "move_tcp")
        assert tcp_call[2] is True

    def test_out_line(self):
        from urctl.cli import run_guided_command

        s = self._session([True])
        res = run_guided_command(s, "out 0 on ; close gripper")
        assert res["result"].kind == "set_output" and res["result"].recorded

    def test_comment_takes_whole_line(self):
        from urctl.cli import run_guided_command

        s = self._session([])
        res = run_guided_command(s, "comment Pick sequence; with semicolon")
        assert res["result"].kind == "comment"
        assert res["result"].desc == "Pick sequence; with semicolon"

    def test_blank_and_hash_are_noops(self):
        from urctl.cli import run_guided_command

        s = self._session([])
        assert run_guided_command(s, "   ")["kind"] == "noop"
        assert run_guided_command(s, "# a note")["kind"] == "noop"

    def test_control_verbs(self):
        from urctl.cli import run_guided_command

        s = self._session([])
        assert run_guided_command(s, "quit")["action"] == "quit"
        assert run_guided_command(s, "help")["action"] == "help"
        assert run_guided_command(s, "summary")["action"] == "summary"
        save = run_guided_command(s, "save out.urp")
        assert save["action"] == "save" and save["path"] == "out.urp"

    def test_bad_input_raises(self):
        from urctl.cli import run_guided_command

        s = self._session([True])
        with pytest.raises(ValueError):
            run_guided_command(s, "movej 1 2 3")  # wrong count
        with pytest.raises(ValueError):
            run_guided_command(s, "out 0 maybe")  # bad value
        with pytest.raises(ValueError):
            run_guided_command(s, "frobnicate 1 2")  # unknown verb


class TestOnRecordHook:
    def test_hook_fires_once_per_recorded_step(self):
        from urctl.guided import GuidedSession

        seen = []
        s = GuidedSession(FakeRobot([True, False, True]), "t", on_record=seen.append)
        s.move_joints([0, 0, 0, 0, 0, 0])  # recorded -> fires
        s.move_joints([0, 0, 0, 0, 0, 0])  # rejected -> no fire
        s.comment("note")  # recorded -> fires
        assert seen == [s, s]  # exactly the two recorded steps, passed the session

    def test_hook_not_fired_when_move_unsafe(self):
        from urctl.guided import GuidedSession

        calls = []
        s = GuidedSession(FakeRobot([True], move_ok=False), "t", on_record=lambda x: calls.append(x))
        s.move_joints([99, 0, 0, 0, 0, 0])  # approved but not executed/recorded
        assert calls == []


class TestLiveReloader:
    def test_save_place_load_sequence(self, tmp_path):
        from urctl.guided import GuidedSession, LiveReloader

        events = []

        def place(local_urp, name):
            assert local_urp.exists()  # session.save ran before placing
            events.append(("place", name))
            return f"{name}.urp"

        robot = FakeRobot([True])
        robot.load_program = lambda n: events.append(("load", n))  # type: ignore[attr-defined]
        reloader = LiveReloader(robot, "Prog", place, local_path=tmp_path / "Prog.urp")
        session = GuidedSession(robot, "Prog", on_record=reloader)
        session.move_joints([0, -1.57, 0, -1.57, 0, 0])
        assert events == [("place", "Prog"), ("load", "Prog.urp")]
        assert (tmp_path / "Prog.urp").exists()

    def test_reload_failure_is_swallowed_and_logged(self, tmp_path):
        from urctl.guided import GuidedSession, LiveReloader

        logs = []

        def boom(local_urp, name):
            raise OSError("controller unreachable")

        robot = FakeRobot([True])
        session = GuidedSession(robot, "P")
        reloader = LiveReloader(robot, "P", boom, local_path=tmp_path / "P.urp", logger=logs.append)
        reloader(session)  # placer fails mid-reload — must not raise
        assert logs and "live reload failed" in logs[0]


class TestLocalDirPlacer:
    def test_copies_urp_and_mirrors_installation(self, tmp_path):
        from urctl.guided import local_dir_placer

        progdir = tmp_path / "programs"
        progdir.mkdir()
        (progdir / "default.installation").write_text("inst")
        src = tmp_path / "MyProg.urp"
        src.write_bytes(b"\x1f\x8b__urp__")

        name = local_dir_placer(progdir)(src, "MyProg")
        assert name == "MyProg.urp"
        assert (progdir / "MyProg.urp").read_bytes() == b"\x1f\x8b__urp__"
        # installation mirrored beside it so the loader finds the pair.
        assert (progdir / "MyProg.installation").read_text() == "inst"


class TestDockerPlacer:
    def test_runs_cp_and_installation_mirror(self, monkeypatch, tmp_path):
        from urctl import guided

        runs = []
        monkeypatch.setattr(guided.subprocess, "run", lambda cmd, **kw: runs.append(cmd) or None)
        src = tmp_path / "Demo.urp"
        src.write_bytes(b"x")
        name = guided.docker_placer("my-container")(src, "Demo")
        assert name == "Demo.urp"
        # First call cp's the urp in; second mirrors the installation beside it.
        assert runs[0][:2] == ["docker", "cp"]
        assert "my-container:/ursim/programs/Demo.urp" == runs[0][3]
        assert runs[1][:3] == ["docker", "exec", "my-container"]


class TestBuildLiveReloader:
    def _args(self, **kw):
        import argparse

        d = dict(
            live=False,
            live_container="ur-docker-ursim-1",
            live_program_dir=None,
            name="P",
            installation="default",
            save=None,
        )
        d.update(kw)
        return argparse.Namespace(**d)

    def test_none_without_live(self):
        from urctl.cli import build_live_reloader

        assert build_live_reloader(FakeRobot([]), self._args(live=False)) is None

    def test_docker_placer_by_default(self):
        from urctl.cli import build_live_reloader
        from urctl.guided import LiveReloader

        r = build_live_reloader(FakeRobot([]), self._args(live=True))
        assert isinstance(r, LiveReloader) and r.name == "P"

    def test_program_dir_overrides_container(self, tmp_path):
        from urctl.cli import build_live_reloader

        r = build_live_reloader(FakeRobot([]), self._args(live=True, live_program_dir=str(tmp_path)))
        assert r is not None  # local_dir_placer path chosen (no docker needed)


class TestInspection:
    """Multi-point inspection: hand-guide to each point, call a camera subprogram."""

    def test_define_helper_embeds_a_script_file_def(self):
        s = GuidedSession(FakeRobot([]), "t")
        s.define_helper("camera", 'def trigger_camera():\n  textmsg("snap")\nend\n')
        assert _nodes(s) == ["Comment", "Script"]
        root = ET.fromstring(gzip.decompress(s.program.to_bytes()))
        cached = root.find(".//Script[@type='File']/cachedContents")
        assert "trigger_camera" in cached.text

    def test_inspection_point_records_move_then_camera_call(self):
        r = FakeRobot([], reteach={"confirmed": True, "joints": [1, 2, 3, 4, 5, 6]})
        s = GuidedSession(r, "t")
        res = s.inspection_point("inspection point 1", call="trigger_camera()")
        assert res.recorded
        assert _nodes(s) == ["Comment", "Move", "Script"]  # annotate, move, fire camera
        root = ET.fromstring(gzip.decompress(s.program.to_bytes()))
        assert root.find(".//Move").get("motionType") == "MoveJ"
        assert root.find(".//Waypoint").get("name") == "InspectionPoint1"
        token = root.find(".//Script[@type='Line']/expression/ExpressionToken").get("token")
        assert token == "trigger_camera()"

    def test_inspection_point_skips_on_cancel(self):
        r = FakeRobot([], reteach={"confirmed": None, "joints": None})  # Cancel/timeout
        s = GuidedSession(r, "t")
        res = s.inspection_point("inspection point 1")
        assert not res.recorded and _nodes(s) == []

    def test_run_inspection_loops_until_operator_finishes(self):
        from urctl.guided import run_inspection

        r = FakeRobot(
            [],
            reteach=[
                {"confirmed": True, "joints": [1, 1, 1, 1, 1, 1]},
                {"confirmed": True, "joints": [2, 2, 2, 2, 2, 2]},
                {"confirmed": None, "joints": None},  # No/Cancel -> finish
            ],
        )
        s = GuidedSession(r, "t")
        n = run_inspection(s)
        assert n == 2
        root = ET.fromstring(gzip.decompress(s.program.to_bytes()))
        assert root.find(".//Script[@type='File']") is not None  # camera helper defined once
        calls = [e.get("token") for e in root.findall(".//Script[@type='Line']/expression/ExpressionToken")]
        assert calls == ["trigger_camera()", "trigger_camera()"]  # one per captured point
        names = [w.get("name") for w in root.iter("Waypoint")]
        assert names == ["InspectionPoint1", "InspectionPoint2"]
        # The camera-helper def must not be tallied as a "no answer" — only the
        # finishing No/Cancel counts (the loop's terminating inspection_point).
        assert s.summary()["no_answer"] == 1
