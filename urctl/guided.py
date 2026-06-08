"""GuidedSession — build a PolyScope program step by step, confirming each step
on the robot's own pendant before it runs.

The workflow this implements: the operator issues a step from the console; the
robot raises a Yes/No popup on the PolyScope pendant describing *what it is
about to do*; on **Yes** the step executes live on the robot **and** is appended
to a growing program; on **No** (or no answer) it is skipped. Saving the session
yields a native node-tree ``.urp`` (see :mod:`urctl.urp_builder`) that replays
exactly the steps that were approved — openable and editable in PolyScope as
real program nodes, not an opaque script blob.

Design choices that matter:

* **Confirm, then act** — two Primary round-trips, not one. The confirmation
  (:meth:`Robot.confirm_on_pendant`) is separate from the motion so the motion
  still flows through :class:`urctl.safety.SafetyEnvelope`; inlining the action
  into the confirm script would bypass validation.
* **Record the achieved pose as a joint-space waypoint.** After a move runs we
  read where the robot actually ended up and store a joint-space ``Waypoint``,
  so the saved program reproduces the same physical position whether the step
  was issued in joint or Cartesian space. (Joint waypoints need no calibration
  to replay — see urp_builder.)
* **Only approved + executed steps are recorded.** A rejected step, a move the
  safety envelope refuses, or a move that doesn't confirm completion is logged
  in :attr:`steps` but never added to the program.

    from urctl import Robot, RobotConfig
    from urctl.guided import GuidedSession

    robot = Robot(RobotConfig())
    s = GuidedSession(robot, "PickDemo")
    s.move_joints([0, -1.57, 0, -1.57, 0, 0], desc="Move to safe approach")
    s.move_tcp([0, 0, -0.05, 0, 0, 0], relative=True, desc="Lower 50 mm onto part")
    s.set_output(0, True, desc="Close gripper")
    s.save("programs/PickDemo/PickDemo.urp")
"""

from __future__ import annotations

import re
import shutil
import subprocess
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from .robot import Robot
from .urp_builder import UrpProgram, Waypoint

# Two recorded poses within this joint distance (radians, per joint) are treated
# as the *same* teach point, so a step that returns to an earlier position reuses
# that waypoint's name instead of inventing a new one.
_POSE_TOL = 1e-3


def _slug(text: str) -> str:
    """Turn a step description into a CamelCase waypoint name.

    "lower 50 mm onto part" -> "Lower50MmOntoPart". Returns "" for text with no
    alphanumerics, so callers can fall back to a generic hint.
    """
    return "".join(w[:1].upper() + w[1:] for w in re.findall(r"[A-Za-z0-9]+", text))


# Default camera-trigger subprogram embedded by the inspection app. It pulses a
# digital output (the common "snap" wiring) and logs the pose. This is a
# placeholder: swap the body for your camera's real trigger (a URCap call, a
# different I/O, an RTDE flag) — the program structure (call a named subprogram
# at each taught point) stays the same.
CAMERA_TRIGGER_DEF = (
    "def trigger_camera():\n"
    "  set_standard_digital_out(1, True)\n"
    "  sleep(0.2)\n"
    "  set_standard_digital_out(1, False)\n"
    '  textmsg("inspection: captured at ", get_actual_tcp_pose())\n'
    "end\n"
)


@dataclass
class StepResult:
    """Outcome of one guided step. ``confirmed``/``executed``/``recorded`` make
    the three independent failure points legible: the operator may reject it,
    the safety envelope may refuse it, or it may run but not be recorded."""

    kind: str
    desc: str
    confirmed: bool | None
    executed: bool
    recorded: bool
    detail: dict = field(default_factory=dict)


class GuidedSession:
    def __init__(
        self,
        robot: Robot,
        name: str,
        *,
        installation: str = "default",
        directory: str | None = None,
        confirm_timeout: float = 120.0,
        run_only_once: bool = True,
        freedrive: bool = False,
        on_record: Callable[[GuidedSession], object] | None = None,
    ) -> None:
        self.robot = robot
        self.name = name
        self.confirm_timeout = confirm_timeout
        # When set, each taught position drops into freedrive after the move so
        # the operator can hand-guide it to the exact spot and tap Yes; the
        # achieved pose is what gets recorded, and relative steps build off it.
        self.freedrive = freedrive
        self.program = UrpProgram(
            name,
            installation=installation,
            directory=directory or f"/programs/{name}",
            run_only_once=run_only_once,
        )
        # Called after every *recorded* step with this session. The live-reload
        # path (see LiveReloader) uses it to push + load the growing program so
        # PolyScope's tree updates node-by-node; left None it's a plain builder.
        self._on_record = on_record
        self.steps: list[StepResult] = []
        # (joint pose, name) for every recorded waypoint, so we can give a
        # returning position the same functional name (rule: unique names except
        # for duplicate positions — see docs/program-authoring-best-practices.md).
        self._named: list[tuple[list[float], str]] = []

    # -- internals -------------------------------------------------------- #

    def _confirm(self, desc: str) -> bool | None:
        # The pendant dialog has Yes / No / Cancel buttons; spell out what each
        # does. Yes records the step; No or Cancel skip it (confirmed False/None).
        ans = self.robot.confirm_on_pendant(
            f"About to: {desc}. Yes = add it, No / Cancel = skip it.", timeout=self.confirm_timeout
        )
        return ans.get("confirmed")

    def _waypoint(self, q: list[float], *, desc: str | None, hint: str) -> Waypoint:
        """Name a waypoint by function and keep names unique.

        A pose matching one recorded earlier (within ``_POSE_TOL``) reuses that
        point's name; otherwise the name comes from the step description
        (CamelCased) or, lacking one, the generic ``hint``, de-duplicated with a
        numeric suffix.
        """
        q = [float(v) for v in q]
        for prev_q, prev_name in self._named:
            if all(abs(a - b) <= _POSE_TOL for a, b in zip(prev_q, q, strict=False)):
                return Waypoint(prev_name, q=q)
        base = (_slug(desc) if desc else "") or hint
        existing = {n for _, n in self._named}
        name = base
        i = 2
        while name in existing:
            name = f"{base}_{i}"
            i += 1
        self._named.append((q, name))
        return Waypoint(name, q=q)

    def _teach_pose(self, fallback: list[float], label: str) -> tuple[list[float], bool]:
        """Resolve the pose to record for a just-executed move.

        With freedrive off, the move's own pose (``fallback``) is recorded. With
        freedrive on, the operator hand-guides the robot and presses OK; the
        achieved pose is returned. The bool is False when the operator declines
        (or doesn't answer) the reteach — the caller then skips recording.
        """
        if not self.freedrive:
            return fallback, True
        res = self.robot.reteach_in_freedrive(
            f"Freedrive: hand-guide '{label}' into place. Yes = record this pose, No / Cancel = skip it.",
            timeout=self.confirm_timeout,
        )
        if res.get("confirmed") and res.get("joints"):
            return [float(v) for v in res["joints"]], True
        return fallback, False

    def _record(self, result: StepResult) -> StepResult:
        self.steps.append(result)
        if result.recorded and self._on_record is not None:
            self._on_record(self)
        return result

    # -- steps ------------------------------------------------------------ #

    def move_joints(
        self, target: list[float], *, desc: str | None = None, name: str = "Joints", **move_kwargs
    ) -> StepResult:
        """Confirm on the pendant, then ``movej`` to ``target`` and record it as
        a MoveJ waypoint. Every recorded step is preceded by a Comment node so
        the program reads top-to-bottom when an operator edits it. With
        ``freedrive`` on, the operator hand-guides the final pose before it's
        recorded (see :meth:`_teach_pose`)."""
        label = desc or f"Move joints to [{', '.join(f'{v:.3f}' for v in target)}]"
        confirmed = self._confirm(label)
        if not confirmed:
            return self._record(StepResult("move_joints", label, confirmed, False, False))
        res = self.robot.move_joints(target, **move_kwargs)
        executed = bool(res.get("ok"))
        recorded = False
        if executed:
            joints, keep = self._teach_pose(list(target), label)
            if keep:
                self.program.comment(label)
                self.program.movej(self._waypoint(joints, desc=desc, hint=name))
                recorded = True
        return self._record(StepResult("move_joints", label, confirmed, executed, recorded, {"move": res}))

    def move_tcp(
        self,
        pose: list[float],
        *,
        relative: bool = False,
        desc: str | None = None,
        name: str = "TCP",
        **move_kwargs,
    ) -> StepResult:
        """Confirm on the pendant, then ``movel`` (Cartesian), and record it as a
        MoveL waypoint at the *achieved* joint pose (read back after the move),
        so it replays to the same physical spot without calibration. Every
        recorded step is preceded by a Comment node. With ``freedrive`` on, the
        operator hand-guides the final pose before it's recorded; later relative
        steps then build off that adjusted position."""
        verb = "by" if relative else "to"
        label = desc or f"Move TCP {verb} [{', '.join(f'{v:.3f}' for v in pose)}]"
        confirmed = self._confirm(label)
        if not confirmed:
            return self._record(StepResult("move_tcp", label, confirmed, False, False))
        res = self.robot.move_tcp(pose, relative=relative, **move_kwargs)
        executed = bool(res.get("ok"))
        recorded = False
        joints = None
        if executed:
            landed = self.robot.get_state().get("joints")
            if landed:
                joints, keep = self._teach_pose(list(landed), label)
                if keep:
                    self.program.comment(label)
                    self.program.movel(self._waypoint(joints, desc=desc, hint=name))
                    recorded = True
        return self._record(
            StepResult("move_tcp", label, confirmed, executed, recorded, {"move": res, "joints": joints})
        )

    def set_output(self, pin: int, value: bool, *, desc: str | None = None) -> StepResult:
        """Confirm on the pendant, then set a digital output and record a Set node."""
        desc = desc or f"Set digital output {pin} {'high' if value else 'low'}"
        confirmed = self._confirm(desc)
        if not confirmed:
            return self._record(StepResult("set_output", desc, confirmed, False, False))
        res = self.robot.set_digital_output(pin, value)
        executed = bool(res.get("ok"))
        recorded = False
        if executed:
            self.program.comment(desc)
            self.program.set_output(pin, value)
            recorded = True
        return self._record(StepResult("set_output", desc, confirmed, executed, recorded, {"io": res}))

    def comment(self, text: str) -> StepResult:
        """Add a Comment node. Annotation only — no robot action, no confirm."""
        self.program.comment(text)
        return self._record(StepResult("comment", text, None, False, True))

    # -- subprograms + inspection ---------------------------------------- #

    def define_helper(self, name: str, contents: str) -> StepResult:
        """Embed a helper ``def`` (e.g. a camera trigger) as a Script(File) node.

        Call it later with :meth:`inspection_point` (or any ``script_line``) so the
        visible flow reads as named operations. Place this before the calls — the
        program runs top to bottom, so the def must execute first."""
        self.program.comment(f"{name} helper")
        self.program.script_file(name, contents)
        return self._record(StepResult("helper", name, None, False, True))

    def inspection_point(self, desc: str, *, call: str = "trigger_camera()") -> StepResult:
        """Teach one inspection pose by hand-guiding, then call a subprogram there.

        Pure freedrive: the operator moves the robot to the spot and answers the
        pendant — **Yes** records the pose (a MoveJ waypoint) followed by a
        ``script_line(call)`` to fire the camera; **No/Cancel** records nothing
        (the loop in :func:`run_inspection` treats that as "finished"). There is
        no scripted target — the human positions the robot, which is the point.
        """
        res = self.robot.reteach_in_freedrive(
            f"Hand-guide to {desc}. Yes = capture here, No / Cancel = finish.",
            timeout=self.confirm_timeout,
        )
        confirmed = res.get("confirmed")
        joints = res.get("joints")
        recorded = False
        if confirmed and joints:
            self.program.comment(desc)
            self.program.movej(self._waypoint([float(v) for v in joints], desc=desc, hint="Inspect"))
            self.program.script_line(call)
            recorded = True
        return self._record(
            StepResult("inspection_point", desc, confirmed, bool(joints), recorded, {"call": call})
        )

    # -- output ----------------------------------------------------------- #

    def save(self, path: str | Path) -> Path:
        """Write the accumulated program to a ``.urp``. Pair it with a sibling
        ``<name>.installation`` (e.g. a copy of ``default.installation``) so
        PolyScope will load it."""
        return self.program.save(path)

    def summary(self) -> dict:
        """A compact tally for the console: how many steps confirmed/recorded."""
        return {
            "steps": len(self.steps),
            "recorded": sum(1 for s in self.steps if s.recorded),
            "rejected": sum(1 for s in self.steps if s.confirmed is False),
            # Only confirm-gated steps can be "no answer"; comments and helper
            # defs carry confirmed=None but were never waiting on the pendant.
            "no_answer": sum(
                1 for s in self.steps if s.confirmed is None and s.kind not in ("comment", "helper")
            ),
        }


def run_inspection(
    session: GuidedSession,
    *,
    call: str = "trigger_camera()",
    helper_name: str = "camera",
    helper_def: str | None = CAMERA_TRIGGER_DEF,
    max_points: int | None = None,
) -> int:
    """Drive a pendant-led multi-point inspection on a :class:`GuidedSession`.

    Embeds the camera-trigger subprogram once (``helper_def``; pass ``None`` to
    skip), then loops: the operator hand-guides the robot to a point and taps
    **Yes** to capture (recording a MoveJ + a ``call`` to the subprogram), or
    **No/Cancel** to finish. Returns the number of points captured. ``max_points``
    bounds the loop (mainly for tests); ``None`` means "until the operator stops".
    """
    if helper_def:
        session.define_helper(helper_name, helper_def)
    captured = 0
    while max_points is None or captured < max_points:
        res = session.inspection_point(f"inspection point {captured + 1}", call=call)
        if not res.recorded:
            break
        captured += 1
    return captured


# --------------------------------------------------------------------------- #
# Live reload — grow PolyScope's program tree as the operator confirms steps.
#
# An e-Series controller only refreshes its Program tree by *loading a program
# file*, so "live" means: after each recorded step, write the .urp where the
# controller can load it and re-load it. A LiveReloader is the GuidedSession
# `on_record` hook that does exactly that; a *placer* abstracts the one
# environment-specific bit — getting the file into the controller's program
# directory — so the session itself stays robot-agnostic.
# --------------------------------------------------------------------------- #

# A placer takes (local_urp_path, program_name) and makes the program loadable
# on the controller, returning the name to pass to Dashboard `load`.
Placer = Callable[[Path, str], str]


class LiveReloader:
    """``on_record`` hook: save → place on the controller → load, after each
    recorded step, so PolyScope's tree updates node-by-node.

    There is a brief reload blink per step — unavoidable on e-Series, since the
    controller owns the tree and we don't. Reload failures are swallowed (the
    in-memory program is still intact and saved at the end); the optional
    ``logger`` is called with a short message so the console can surface them.
    """

    def __init__(
        self,
        robot: Robot,
        name: str,
        place: Placer,
        *,
        local_path: str | Path | None = None,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self.robot = robot
        self.name = name
        self.place = place
        self.local_path = Path(local_path) if local_path else Path(f"/tmp/{name}.urp")
        self.logger = logger

    def __call__(self, session: GuidedSession) -> None:
        try:
            session.save(self.local_path)
            load_name = self.place(self.local_path, self.name)
            self.robot.load_program(load_name)
        except (OSError, subprocess.SubprocessError) as exc:
            if self.logger:
                self.logger(f"live reload failed: {exc}")


def docker_placer(
    container: str,
    *,
    program_dir: str = "/ursim/programs",
    installation: str = "default",
) -> Placer:
    """Placer for this repo's URSim: ``docker cp`` the .urp into the container's
    program dir and mirror ``<installation>.installation`` beside it (the loader
    requires a matching installation file — see CLAUDE.md)."""

    def place(local_urp: Path, name: str) -> str:
        subprocess.run(
            ["docker", "cp", str(local_urp), f"{container}:{program_dir}/{name}.urp"],
            check=True,
            capture_output=True,
        )
        subprocess.run(
            [
                "docker",
                "exec",
                container,
                "cp",
                f"{program_dir}/{installation}.installation",
                f"{program_dir}/{name}.installation",
            ],
            check=True,
            capture_output=True,
        )
        return f"{name}.urp"

    return place


def local_dir_placer(program_dir: str | Path, *, installation: str = "default") -> Placer:
    """Placer for a controller whose program directory is reachable on the host
    filesystem (a mounted share, or URSim run with a volume): copy the .urp in
    and mirror the installation file beside it if not already present."""
    program_dir = Path(program_dir)

    def place(local_urp: Path, name: str) -> str:
        shutil.copy(local_urp, program_dir / f"{name}.urp")
        inst_src = program_dir / f"{installation}.installation"
        inst_dst = program_dir / f"{name}.installation"
        if inst_src.exists() and not inst_dst.exists():
            shutil.copy(inst_src, inst_dst)
        return f"{name}.urp"

    return place


__all__ = [
    "GuidedSession",
    "StepResult",
    "LiveReloader",
    "docker_placer",
    "local_dir_placer",
    "Placer",
    "run_inspection",
    "CAMERA_TRIGGER_DEF",
]
