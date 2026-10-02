"""``urcap/pick5_e2e.py`` without a controller: it builds the node's scripts with the harness
the URCap tests use, and its pass criteria hold for the session the scripts drive.

The matrix (every PolyScope 5 minor) is where the script really runs; this is what must be
right before it gets there. Found the hard way, 2026-09-30: the e2e compiled the test
harness without the screen classes the harness had started to use, and every one of the 23
controllers failed in a minute on a ``javac`` error no unit test had seen.
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "urcap"))
import pick5_e2e  # noqa: E402

from perceptronics import armfk  # noqa: E402
from perceptronics.picknode import PickPlanner  # noqa: E402
from urctl.pose import pose_trans  # noqa: E402

SPEC = {
    "host": "172.18.0.1",
    "port": 7623,
    "node": "e2e001",
    "points": [{"q": pick5_e2e.READY}],
    "popup": False,
}
needs_javac = pytest.mark.skipif(shutil.which("javac") is None, reason="javac is not installed")


@needs_javac
@pytest.mark.parametrize("polyscope", [[5, 4, 3], [5, 26, 1]])
def test_the_e2e_builds_every_script_it_runs(polyscope):
    spec = {**SPEC, "polyscope": polyscope}
    once = pick5_e2e.generate(spec)
    no_look = pick5_e2e.generate({**spec, "closeLook": False})
    probe = pick5_e2e.generate({**spec, "popup": True})
    assert once.startswith("# 3D Pick ") and '"LOOK "' in once and '"LOOK "' not in no_look
    assert "set_standard_digital_out" not in once and "63352" not in probe  # the node drives no gripper
    assert "popup(" in probe and "popup(" not in once
    for script in (once, no_look):  # each is a whole program on its own: the e2e runs them back to back
        assert script.count("\n") > 100 and script.rstrip().endswith("set_tcp(rs_tcp0)")


def _numbers(reply: str) -> list[float]:
    assert reply.startswith("(") and reply.endswith(")\n"), reply
    return [float(v) for v in reply[1:-2].split(",")]


def _pose(p) -> str:
    return "p[" + ", ".join(f"{v:.6f}" for v in p) + "]"


class Controller:
    """The requests the node's script sends, in its order — with the arm "moving" to whatever
    pose the script would move it to. Not a URScript interpreter: the part of the session the
    pick server can see."""

    def __init__(self, tokens: str):
        self.world = pick5_e2e.World()
        self.seq = 0
        self.log: list[str] = []
        self.captured: list[str] = []
        self.tokens = tokens
        self.ready = armfk.frames(pick5_e2e.READY, "UR5E")[-1]
        self.flange = pose_trans(self.ready, [0.05, 0.02, -0.1, 0.0, 0.0, 0.0])  # wherever the program starts
        planner = PickPlanner(
            self.source,
            lambda: self.seq,
            lambda: [0.0] * 6,
            tip_m=0.163,
            min_radius_m=0.0,
            log=lambda text, ok: self.log.append(text),
        )
        self._answer = planner.answer

    def source(self, after):
        self.seq = max(self.seq, after) + 1
        k = pick5_e2e
        return self.seq, k.W, k.H, 3, bytes(k.W * k.H * 3), self.world.depth(), 0.001, k.K

    def ask(self, line: str) -> list[float]:
        self.world.view(line)
        return _numbers(self._answer(line))

    def run(self, marker: str, close_look: bool) -> None:
        tok = self.tokens
        r = self.ask(f"NEXT {_pose(self.flange)} node=e2e001 locs=1 proto=2")
        queued = close_look and r[0] == 1
        if queued:
            self.captured.append("3D Pick: next part already seen, #1")
        else:
            self.flange = list(self.ready)  # the survey: movej to the picture point
            r = self.ask(f"FIND {_pose(self.flange)}{tok}")
            assert r[0] == 1, self.log
        centre = r[1:4]
        c = _pose([*centre, 0, 0, 0])
        if close_look:
            lk = _numbers(self._answer(f"LOOK {_pose(self.flange)} {c} stroke=50"))
            if lk[0] == 1:
                self.flange = lk[4:10]
        r = self.ask(f"REFINE {_pose(self.flange)} {c}{tok} lean=0")
        assert r[0] == 1, self.log
        self.flange = pose_trans(
            r[4:10], [0.0, 0.0, 0.015, 0.0, 0.0, 0.0]
        )  # down to the grip, and it ends there
        self.captured.append(f"rs_e2e/{marker}= True")
        if marker == "first":
            self.captured.append(f"rs_e2e/loc= {int(r[10])}")


@needs_javac
def test_the_three_runs_the_e2e_drives_meet_its_own_pass_criteria():
    script = pick5_e2e.generate({**SPEC, "polyscope": [5, 26, 1]})
    tok = re.search(r'rs_tok = "( [^"]+)"', script).group(1)  # the options the real script sends
    c = Controller(tok)
    c.run("first", close_look=True)  # survey, closer look, clamp
    c.run("found", close_look=True)  # from the queue: starts at the last grip, 0.16 m over the table
    c.run("nolook", close_look=False)  # the closer look off: the picture point every time
    got = pick5_e2e.verdict(c.captured, c.log)
    assert all(got.values()), ([k for k, v in got.items() if not v], c.log)
    # the second run's closer look backed the camera out of its blind zone before it measured
    looks = [t for t in c.log if t.startswith("pick LOOK")]
    assert len(looks) == 2


def test_the_pass_criteria_fail_a_run_that_did_not_pick():
    ok = {
        "captured": [
            "rs_e2e/first= True",
            "rs_e2e/loc= 1",
            "3D Pick: next part already seen, #1",
            "rs_e2e/found= True",
            "rs_e2e/nolook= True",
        ],
        "log": ["pick FIND [e2e001]: found #1 at 1"] * 2
        + ["pick REFINE [e2e001]: found #0 at 1"] * 3
        + ["pick LOOK: found top [0.1, 0.2, 0.3]"] * 2
        + ["pick NEXT [e2e001]: found #2 at 1"],
    }
    assert all(pick5_e2e.verdict(**ok).values())
    for drop in ok["captured"]:
        less = [c for c in ok["captured"] if c != drop]
        assert not all(pick5_e2e.verdict(less, ok["log"]).values()), drop
    assert not all(pick5_e2e.verdict(ok["captured"], ok["log"][1:]).values())  # one survey short
    third_look = ok["log"] + ["pick LOOK: found top [0.1, 0.2, 0.3]"]
    assert not all(pick5_e2e.verdict(ok["captured"], third_look).values())  # the look was not switched off
