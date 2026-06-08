"""Integration tests that talk to a running URSim simulator.

Skipped automatically when:
- the Dashboard server isn't reachable, OR
- the controller is reporting NO_CONTROLLER, OR
- `docker exec` against the URSim container isn't available.

Run a subset directly:

    pytest tests/test_integration_ursim.py -v -k "load"

The `ursim_ready` fixture ensures the controller is up before each test;
`docker_ready` is required for tests that drop files into the container.
"""

from __future__ import annotations

import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
import urp_convert as uc
from _ursim import (
    PROGRAMS_DIR,
    SCRIPTS_DIR,
    URSIM_CONTAINER,
    URSIM_HOST,
    URSIM_PRIMARY_PORT,
    dash,
    docker_cp_to_container,
    docker_exec,
    wait_for_dash,
)

from urctl.urp_builder import UrpProgram, Waypoint

pytestmark = pytest.mark.integration


# ---------------------------------------------------------------------------
# Dashboard server smoke tests
# ---------------------------------------------------------------------------


class TestDashboard:
    def test_robotmode_reports_a_known_state(self, ursim_ready):
        reply = dash("robotmode")
        assert any(
            state in reply for state in ("POWER_OFF", "BOOTING", "IDLE", "RUNNING", "POWER_ON", "BACKDRIVE")
        ), f"unexpected robotmode reply: {reply!r}"

    def test_safety_mode_is_normal(self, ursim_ready):
        reply = dash("safetymode")
        assert "Safetymode: NORMAL" in reply

    def test_popup_command_acknowledged(self, ursim_ready):
        reply = dash("popup integration test popup")
        assert "showing popup" in reply

    def test_unknown_command_does_not_hang(self, ursim_ready):
        # Sanity check on the test infrastructure itself: a junk command
        # should return without blocking.
        reply = dash("not_a_real_command")
        assert reply  # Dashboard returns some line, even if it's an error


# ---------------------------------------------------------------------------
# poweron.sh wraps the state machine — exercise it end to end.
# ---------------------------------------------------------------------------


class TestPoweronScript:
    def test_poweron_brings_controller_to_running(self, ursim_ready):
        # If we're already RUNNING, drop back to POWER_OFF so the test
        # exercises the full transition. Tests must not depend on which
        # power state a previous test left us in.
        dash("power off")
        wait_for_dash(lambda r: "POWER_OFF" in r, timeout=30.0)

        env = {**os.environ, "UR_HOST": URSIM_HOST}
        r = subprocess.run(
            [str(SCRIPTS_DIR / "poweron.sh")],
            env=env,
            capture_output=True,
            text=True,
            timeout=180,
        )
        assert r.returncode == 0, f"poweron.sh failed:\nstdout:\n{r.stdout}\nstderr:\n{r.stderr}"
        final = dash("robotmode")
        assert "RUNNING" in final, f"expected RUNNING after poweron.sh, got: {final}"


# ---------------------------------------------------------------------------
# URP loading — the actual PolyScope acceptance test for the converter.
# ---------------------------------------------------------------------------


class TestUrpLoading:
    def _drop_program(
        self,
        name: str,
        urp_bytes: bytes,
        installation_source: str = "/ursim/programs/default.installation",
        tmp_path: Path | None = None,
    ) -> str:
        """Place a URP + installation pair into URSim's program dir,
        returning the URP filename for `load`."""
        assert tmp_path is not None
        urp_path = tmp_path / f"{name}.urp"
        urp_path.write_bytes(urp_bytes)
        docker_cp_to_container(urp_path, f"/ursim/programs/{name}.urp")
        # Mirror default.installation as <name>.installation so PolyScope
        # finds the paired installation file the loader requires.
        docker_exec("cp", installation_source, f"/ursim/programs/{name}.installation")
        return f"{name}.urp"

    def test_generated_minimal_urp_loads(self, ursim_ready, docker_ready, tmp_path):
        urp = uc.script_to_urp(
            'def t():\n  textmsg("loaded")\nend\n',
            name="test_minimal",
            installation="test_minimal",
            directory="/programs",
        )
        urp_name = self._drop_program("test_minimal", urp, tmp_path=tmp_path)
        reply = dash(f"load {urp_name}")
        assert "Loading program" in reply, reply
        # programState afterwards should report STOPPED <name> (loaded, idle).
        state = dash("programState")
        assert "STOPPED" in state
        assert urp_name in state

    def test_bundled_inspection_bot_urp_loads(self, ursim_ready, docker_ready, tmp_path):
        # The flagship sample program should load every time.
        src_urp = PROGRAMS_DIR / "InspectionBot" / "InspectionBot.urp"
        src_inst = PROGRAMS_DIR / "InspectionBot" / "InspectionBot.installation"
        if not src_urp.exists() or not src_inst.exists():
            pytest.skip("InspectionBot artifacts not built")
        docker_cp_to_container(src_urp, "/ursim/programs/InspectionBot.urp")
        docker_cp_to_container(src_inst, "/ursim/programs/InspectionBot.installation")
        reply = dash("load InspectionBot.urp")
        assert "Loading program" in reply
        state = dash("programState")
        assert "STOPPED" in state and "InspectionBot.urp" in state

    def test_bundled_elegant_dance_urp_loads(self, ursim_ready, docker_ready, tmp_path):
        # The ElegantDance showcase program should load every time so it can be
        # played from PolyScope later.
        src_urp = PROGRAMS_DIR / "ElegantDance" / "ElegantDance.urp"
        src_inst = PROGRAMS_DIR / "ElegantDance" / "ElegantDance.installation"
        if not src_urp.exists() or not src_inst.exists():
            pytest.skip("ElegantDance artifacts not built")
        docker_cp_to_container(src_urp, "/ursim/programs/ElegantDance.urp")
        docker_cp_to_container(src_inst, "/ursim/programs/ElegantDance.installation")
        reply = dash("load ElegantDance.urp")
        assert "Loading program" in reply
        state = dash("programState")
        assert "STOPPED" in state and "ElegantDance.urp" in state

    def test_bundled_apple_stack_urp_loads(self, ursim_ready, docker_ready, tmp_path):
        # The AppleStack (image-derived pick & stack) program should load every
        # time so it can be played from PolyScope later.
        src_urp = PROGRAMS_DIR / "AppleStack" / "AppleStack.urp"
        src_inst = PROGRAMS_DIR / "AppleStack" / "AppleStack.installation"
        if not src_urp.exists() or not src_inst.exists():
            pytest.skip("AppleStack artifacts not built")
        docker_cp_to_container(src_urp, "/ursim/programs/AppleStack.urp")
        docker_cp_to_container(src_inst, "/ursim/programs/AppleStack.installation")
        reply = dash("load AppleStack.urp")
        assert "Loading program" in reply
        state = dash("programState")
        assert "STOPPED" in state and "AppleStack.urp" in state

    def test_urp_missing_installation_attr_fails_with_diagnostic(
        self,
        ursim_ready,
        docker_ready,
        tmp_path,
    ):
        # Negative test: a URP that omits the installation= attr should be
        # rejected by PolyScope. Protects the documented loader
        # requirements in CLAUDE.md from bit-rot.
        #
        # Asserting via programState rather than the load reply text:
        # PolyScope's load failure path can take >10s to produce a reply,
        # and Dashboard sometimes drops the connection before the error
        # string is flushed. programState is the durable signal.
        import gzip

        xml = (
            '<URProgram name="bad" directory="/programs" '
            'createdIn="5.12.5" lastSavedIn="5.12.5">'
            "<children><MainProgram><children>"
            '<Script type="Code"><cachedContents>x=1</cachedContents></Script>'
            "</children></MainProgram></children></URProgram>"
        )
        urp_path = tmp_path / "bad.urp"
        urp_path.write_bytes(gzip.compress(xml.encode()))
        docker_cp_to_container(urp_path, "/ursim/programs/bad.urp")
        # Try to load; ignore reply (may be empty or partial — see comment).
        try:
            dash("load bad.urp", timeout=20.0)
        except (TimeoutError, OSError):
            pass
        # Give PolyScope time to attempt and abandon the load.
        time.sleep(4)
        # After a failed load, programState must not advertise bad.urp as
        # the active program.
        state = dash("programState")
        assert "bad.urp" not in state, (
            f"PolyScope reported bad.urp as loaded; loader regression?\nprogramState={state!r}"
        )


# ---------------------------------------------------------------------------
# URP builder — proves PolyScope deserializes a from-scratch NODE TREE (not a
# single Script wrapper), so generated programs show up as real, editable UR
# program nodes. Each node type below was confirmed to load before shipping.
# ---------------------------------------------------------------------------


class TestUrpBuilderLoading:
    def _drop(self, name: str, urp_bytes: bytes, tmp_path: Path) -> str:
        urp_path = tmp_path / f"{name}.urp"
        urp_path.write_bytes(urp_bytes)
        docker_cp_to_container(urp_path, f"/ursim/programs/{name}.urp")
        docker_exec(
            "cp",
            "/ursim/programs/default.installation",
            f"/ursim/programs/{name}.installation",
        )
        return f"{name}.urp"

    def test_full_node_tree_urp_loads(self, ursim_ready, docker_ready, tmp_path):
        # Exercises every node type the builder emits in one program: Comment,
        # Folder, Set, Script(File) helper + Script(Line) call, MoveJ/MoveL +
        # Waypoint, If(digital-in), Loop, Popup. A single rejected node fails
        # the whole load, so a clean load is a full-repertoire acceptance test.
        p = UrpProgram("builder_full", installation="builder_full", directory="/programs")
        p.comment("generated by urctl.urp_builder")
        with p.folder("Setup"):
            p.set_output(0, True)
        p.script_file("helpers", 'def retract():\n  textmsg("retract")\nend\n')
        p.movej(Waypoint("Approach", q=[-1.6, -1.72, -2.2, -0.8, 1.595, -0.03]))
        with p.if_digital_in(0, True):
            p.movel(Waypoint("Grasp", q=[-1.6, -1.9, -2.16, -0.68, 1.595, -0.03]))
        with p.loop(3):
            p.movej(Waypoint("Tap", q=[-1.6, -1.72, -2.2, -0.8, 1.595, -0.03]))
        p.script_line("retract()")
        p.popup("Cycle complete")

        urp_name = self._drop("builder_full", p.to_bytes(), tmp_path)
        reply = dash(f"load {urp_name}")
        assert "Loading program" in reply, reply
        state = dash("programState")
        assert "STOPPED" in state and urp_name in state, state

    def test_blended_waypoints_urp_loads(self, ursim_ready, docker_ready, tmp_path):
        # A MoveL with per-waypoint blend radii (blendRadius attribute on
        # <motionParameters>) must load. The schema is read from the controller's
        # own WaypointNodeConversionStrategy, but a clean load is the proof.
        p = UrpProgram("blend_check", installation="blend_check", directory="/programs")
        p.movel(Waypoint("Fly", q=[-1.6, -1.72, -2.2, -0.8, 1.595, -0.03], blend_radius=0.05), group=True)
        p.movel(Waypoint("Via", q=[-1.5, -1.6, -2.1, -0.9, 1.595, -0.03], blend_radius=0.03), group=True)
        p.movel(Waypoint("Stop", q=[-1.4, -1.5, -2.0, -1.0, 1.595, -0.03]), group=True)  # no blend
        urp_name = self._drop("blend_check", p.to_bytes(), tmp_path)
        reply = dash(f"load {urp_name}")
        assert "Loading program" in reply, reply
        state = dash("programState")
        assert "STOPPED" in state and urp_name in state, state

    def test_inspection_program_shape_loads(self, ursim_ready, docker_ready, tmp_path):
        # The multi-point inspection app emits: a camera-trigger Script(File) def,
        # then per point a Comment + MoveJ waypoint + Script(Line) call. Assert
        # that exact shape (with the real CAMERA_TRIGGER_DEF) loads in PolyScope.
        from urctl.guided import CAMERA_TRIGGER_DEF

        p = UrpProgram("inspect_check", installation="inspect_check", directory="/programs")
        p.comment("camera helper")
        p.script_file("camera", CAMERA_TRIGGER_DEF)
        for i, q in enumerate(
            ([-1.6, -1.72, -2.2, -0.8, 1.595, -0.03], [-1.5, -1.6, -2.1, -0.9, 1.595, -0.03]), 1
        ):
            p.comment(f"inspection point {i}")
            p.movej(Waypoint(f"InspectionPoint{i}", q=q))
            p.script_line("trigger_camera()")
        urp_name = self._drop("inspect_check", p.to_bytes(), tmp_path)
        reply = dash(f"load {urp_name}")
        assert "Loading program" in reply, reply
        state = dash("programState")
        assert "STOPPED" in state and urp_name in state, state

    def test_bundled_nodetree_demo_urp_loads(self, ursim_ready, docker_ready, tmp_path):
        # The committed builder-authored sample (programs/NodeTreeDemo) must load
        # as a node tree every time, so it can be opened/edited in PolyScope.
        src_urp = PROGRAMS_DIR / "NodeTreeDemo" / "NodeTreeDemo.urp"
        src_inst = PROGRAMS_DIR / "NodeTreeDemo" / "NodeTreeDemo.installation"
        if not src_urp.exists() or not src_inst.exists():
            pytest.skip("NodeTreeDemo artifacts not built — run programs/NodeTreeDemo/build.py")
        docker_cp_to_container(src_urp, "/ursim/programs/NodeTreeDemo.urp")
        docker_cp_to_container(src_inst, "/ursim/programs/NodeTreeDemo.installation")
        reply = dash("load NodeTreeDemo.urp")
        assert "Loading program" in reply
        state = dash("programState")
        assert "STOPPED" in state and "NodeTreeDemo.urp" in state


# ---------------------------------------------------------------------------
# Pendant confirm — the human-in-the-loop gate behind GuidedSession. We can't
# tap Yes/No from CI, so we verify the round-trip plumbing: the request is sent,
# the call blocks until the (short) timeout, returns confirmed=None, and the
# dangling dialog is cleaned up so the controller stays usable.
# ---------------------------------------------------------------------------


class TestPendantConfirm:
    def test_confirm_times_out_to_none_and_recovers(self, ursim_ready):
        from urctl import Robot, RobotConfig

        robot = Robot(RobotConfig(host=URSIM_HOST))
        res = robot.confirm_on_pendant("integration: add this step?", timeout=4.0)
        assert res["ok"] and res["confirmed"] is None
        # No wedge: the Dashboard still answers promptly after cleanup.
        assert "Robotmode:" in dash("robotmode")

    def test_reteach_times_out_and_leaves_freedrive_cleanly(self, ursim_ready):
        # The freedrive reteach program compiles/runs on the controller; without a
        # human OK it times out to None and the cleanup leaves the robot usable
        # and out of freedrive (no wedge, Dashboard still responsive).
        from urctl import Robot, RobotConfig

        robot = Robot(RobotConfig(host=URSIM_HOST))
        res = robot.reteach_in_freedrive("integration: position the part", timeout=4.0)
        assert res["ok"] and res["confirmed"] is None and res["joints"] is None
        assert "Robotmode:" in dash("robotmode")


class TestLiveReloadPublish:
    """The `--live` machinery without a human: build the program in memory, then
    run the LiveReloader (docker_placer -> cp + installation mirror -> load) and
    assert PolyScope shows the grown program. This is what makes the tree update
    node-by-node; the pendant tap that triggers it is the only un-CI-able part."""

    def test_reloader_publishes_and_loads_growing_program(self, ursim_ready, docker_ready, tmp_path):
        import shutil

        from urctl import Robot, RobotConfig
        from urctl.guided import GuidedSession, LiveReloader, docker_placer
        from urctl.urp_builder import Waypoint

        # docker_placer shells out to a literal `docker`; skip where only
        # `sudo docker` works (the _ursim helpers handle that elsewhere).
        if shutil.which("docker") is None:
            pytest.skip("plain `docker` not on PATH")
        probe = subprocess.run(["docker", "ps"], capture_output=True)
        if probe.returncode != 0:
            pytest.skip("`docker ps` needs privileges here; LiveReloader covered by unit tests")

        robot = Robot(RobotConfig(host=URSIM_HOST))
        session = GuidedSession(robot, "LiveReloadTest")
        reloader = LiveReloader(
            robot, "LiveReloadTest", docker_placer(URSIM_CONTAINER), local_path=tmp_path / "lrt.urp"
        )

        # First node -> publish + load.
        session.program.movej(Waypoint("A_1", q=[0, -1.57, 0, -1.57, 0, 0]))
        reloader(session)
        state = dash("programState")
        assert "LiveReloadTest.urp" in state, state

        # Second node -> republish + reload: the controller now holds 2 nodes.
        session.program.movej(Waypoint("A_2", q=[0.1, -1.57, 0, -1.57, 0, 0]))
        reloader(session)
        assert "LiveReloadTest.urp" in dash("programState")
        # Prove the reloaded file really grew (read it back out of the container).
        out = subprocess.run(
            [
                "docker",
                "exec",
                URSIM_CONTAINER,
                "sh",
                "-c",
                "zcat /ursim/programs/LiveReloadTest.urp | grep -c '<Move'",
            ],
            capture_output=True,
            text=True,
        )
        assert out.stdout.strip() == "2", out.stdout


class TestE2EDrive:
    """Run scripts/e2e_drive.py end to end against the simulator.

    This is the highest-value integration test: it exercises both backend
    ports, the URP converter, the Dashboard load/play state machine, and
    the round-trip from URScript submission to motion verification.
    """

    def test_drive_runs_all_phases(self, ursim_ready, docker_ready):
        # Full power-cycle to guarantee a clean controller state. Prior tests
        # in the same session can leave PolyScope in an odd control mode or
        # with a half-executed program; an explicit `stop` + `power off` +
        # `poweron.sh` gives us a known starting point. Costs ~15 s but is
        # the only reliable reset path on URSim.
        from _ursim import wait_for_dash

        dash("stop")
        time.sleep(1)
        dash("power off")
        wait_for_dash(lambda r: "POWER_OFF" in r, timeout=30.0)
        subprocess.run([str(SCRIPTS_DIR / "poweron.sh")], check=True, capture_output=True, timeout=180)
        wait_for_dash(lambda r: "RUNNING" in r, timeout=60.0)

        r = subprocess.run(
            [sys.executable, str(SCRIPTS_DIR / "e2e_drive.py"), "--host", URSIM_HOST, "--json"],
            capture_output=True,
            text=True,
            timeout=240,
        )
        # Parse the JSON phase records from stdout — one per line.
        import json

        phases = [json.loads(ln) for ln in r.stdout.splitlines() if ln.strip()]
        names = [p["phase"] for p in phases]
        failed = [p for p in phases if not p["ok"]]
        # Every phase must pass — the failure detail goes into the assertion
        # message so a CI log shows exactly which sub-step regressed.
        assert not failed, (
            f"e2e_drive failed phases: {[p['phase'] for p in failed]}\ndetails: {failed}\nstderr: {r.stderr}"
        )
        # And we expect at least Phase 1 through Phase 6.
        assert sum("Phase 1" in n for n in names) == 1
        assert any("Phase 6" in n for n in names)


class TestPrimaryInterface:
    def test_popup_via_primary_client(self, ursim_ready):
        # Sending a single URScript line to Primary 30001 executes it
        # immediately on the controller. We don't have a way to observe
        # the popup from this test, but URControl should accept and parse
        # it without error.
        payload = b'popup("integration test via primary", title="primary")\n'
        with socket.create_connection((URSIM_HOST, URSIM_PRIMARY_PORT), timeout=5.0) as s:
            s.sendall(payload)
        # Primary is a broadcast socket — recvs would return state messages,
        # not a response. The assertion is that the send succeeded and the
        # controller didn't drop the connection.

    def test_primary_port_open(self, ursim_ready):
        # If 30001 isn't open the previous test's send would have failed
        # immediately, but an explicit check produces a clearer message.
        s = socket.create_connection((URSIM_HOST, URSIM_PRIMARY_PORT), timeout=2.0)
        s.close()


# ---------------------------------------------------------------------------
# Cartesian moves via the urctl Robot facade — the capability that replaces
# hand-rolled URScript, plus the regression guard for the def-wrap fix.
# ---------------------------------------------------------------------------


# A dexterous, non-singular joint pose to start Cartesian-move tests from.
# URSim's default startup pose and HOME_JOINTS (0,-90,0,-90,0,0) both leave the
# elbow straight — an elbow singularity where a `movel` spikes joint velocities
# and the controller trips a protective stop (error C154A0). Bending the elbow
# (joint 3 != 0) puts the arm in a well-conditioned configuration where small
# Cartesian nudges are safe. See CLAUDE.md "Cartesian moves and singularities".
READY_JOINTS = [0.0, -1.0, 1.2, -1.8, -1.5708, 0.0]


def _running_robot(*, ready_pose: bool = True):
    """A Robot pointed at URSim, brought up to RUNNING and (by default) parked at
    a dexterous READY pose so Cartesian moves don't hit a singularity.

    Idempotent and self-healing: ``robot_mode`` stays RUNNING during a protective
    stop, so we check ``safety_mode`` too and re-run ``bring_up`` (which unlocks a
    protective stop) when the arm is latched — otherwise one test's protective
    stop cascades into every later motion test.
    """
    from urctl import Robot, RobotConfig

    robot = Robot(RobotConfig.from_env(host=URSIM_HOST))
    if "RUNNING" not in robot.robot_mode() or "PROTECTIVE_STOP" in robot.safety_mode():
        robot.bring_up()
    if ready_pose:
        robot.move_joints(READY_JOINTS)
        # A movej into the ready pose can itself trip a protective stop if the
        # arm started folded against a limit; clear it so the test body starts
        # from NORMAL.
        if "PROTECTIVE_STOP" in robot.safety_mode():
            robot.bring_up()
            robot.move_joints(READY_JOINTS)
    return robot


def _read_tcp(robot, *, collect_for: float = 3.0):
    """Read the TCP pose, retrying briefly — Primary's broadcast is sampled."""
    for _ in range(3):
        tcp = robot.get_state(collect_for=collect_for)["tcp"]
        if tcp is not None:
            return tcp
        time.sleep(1)
    raise AssertionError("TCP pose never surfaced from the Primary broadcast")


class TestMoveTcpIntegration:
    def test_relative_move_changes_tcp_and_returns(self, ursim_ready):
        robot = _running_robot()
        start = _read_tcp(robot)
        step = 0.04  # 40 mm along base +Y

        out = robot.move_tcp([0, step, 0, 0, 0, 0], relative=True, velocity=0.1)
        assert out["ok"], out
        moved = _read_tcp(robot)
        assert abs((moved[1] - start[1]) - step) < 0.003, (
            f"expected +{step} m on Y; start={start[1]:.4f} moved={moved[1]:.4f}"
        )
        # X and Z should be essentially unchanged.
        assert abs(moved[0] - start[0]) < 0.003 and abs(moved[2] - start[2]) < 0.003

        # Return to the original Y so the test is idempotent across runs.
        back = robot.move_tcp([0, -step, 0, 0, 0, 0], relative=True, velocity=0.1)
        assert back["ok"], back
        returned = _read_tcp(robot)
        assert abs(returned[1] - start[1]) < 0.003

    def test_unsafe_relative_step_is_refused(self, ursim_ready):
        robot = _running_robot()
        start = _read_tcp(robot)
        # 5 m relative step trips the unit-error guard — nothing should move.
        out = robot.move_tcp([0, 5.0, 0, 0, 0, 0], relative=True)
        assert out["ok"] is False
        after = _read_tcp(robot)
        assert abs(after[1] - start[1]) < 0.003

    def test_move_tcp_returns_promptly_not_after_full_timeout(self, ursim_ready):
        """Latency guard: move_tcp(wait=True) must return shortly after the move
        lands, not block the full `timeout`. A short move with a generous
        timeout should still return quickly thanks to the stop-marker early
        exit. This is the slowness half of the original 'took too long'."""
        import time as _time

        robot = _running_robot()
        start = _read_tcp(robot)
        t0 = _time.monotonic()
        out = robot.move_tcp([0, 0.03, 0, 0, 0, 0], relative=True, velocity=0.1, timeout=30.0)
        elapsed = _time.monotonic() - t0
        assert out["ok"], out
        # The move itself is ~1 s; without early-exit this would be ~30 s.
        assert elapsed < 15, f"move_tcp blocked {elapsed:.1f}s — early-exit regressed?"
        # Restore.
        moved = _read_tcp(robot)
        robot.move_tcp([0, -(moved[1] - start[1]), 0, 0, 0, 0], relative=True, velocity=0.1)

    def test_run_script_wrapped_capture_executes_motion(self, ursim_ready):
        """The reliable script path: wrapped + captured (connection held open
        until the done marker) runs motion deterministically. (Fire-and-forget
        run_script without capture is racy by nature — closing the Primary
        socket immediately can drop the program; use move_tcp for guarantees.)"""
        robot = _running_robot()
        start = _read_tcp(robot)
        body = (
            "movel(pose_add(get_actual_tcp_pose(), p[0, 0.04, 0, 0, 0, 0]), a=0.3, v=0.1)\n"
            "sync()\n"
            'textmsg("urctl/move/done=", get_actual_tcp_pose())\n'
        )
        out = robot.run_script(body, wrap=True, capture=True, marker="urctl/move", collect_for=30.0)
        assert out["ok"]
        moved = _read_tcp(robot)
        assert (moved[1] - start[1]) > 0.03, (
            f"wrapped+captured movel did not move; start={start[1]:.4f} moved={moved[1]:.4f}"
        )
        # Restore Y.
        robot.move_tcp([0, -(moved[1] - start[1]), 0, 0, 0, 0], relative=True, velocity=0.1)


# ---------------------------------------------------------------------------
# RTDE (port 30004) — structured state + writes. The headline win over the
# Primary textmsg path: state is readable even when no program is running.
# NOTE: on real e-Series hardware RTDE *writes* (speed slider, digital outputs)
# require Remote control mode; the write test below is therefore skip-tolerant.
# ---------------------------------------------------------------------------

URSIM_RTDE_PORT = int(os.environ.get("UR_RTDE_PORT", 30004))


class TestRtdeIntegration:
    def test_rtde_port_open(self, ursim_ready):
        s = socket.create_connection((URSIM_HOST, URSIM_RTDE_PORT), timeout=2.0)
        s.close()

    def test_rtde_state_when_not_running(self, ursim_ready):
        """The headline advantage: RTDE state reads work without bring_up()."""
        from urctl import Robot, RobotConfig

        robot = Robot(RobotConfig.from_env(host=URSIM_HOST))
        out = robot.rtde_state()
        assert out["ok"], out
        assert out["joints"] is not None and len(out["joints"]) == 6
        assert out["tcp"] is not None and len(out["tcp"]) == 6
        assert out["tcp_force"] is not None and len(out["tcp_force"]) == 6

    def test_get_state_uses_rtde_fields(self, ursim_ready):
        robot = _running_robot()
        state = robot.get_state()
        assert state["ok"]
        assert state["joints"] is not None
        # RTDE upgrade adds force/velocity fields the textmsg path can't supply.
        assert "tcp_force" in state and state["tcp_force"] is not None

    def test_speed_override_round_trips(self, ursim_ready):
        robot = _running_robot()
        try:
            out = robot.set_speed_override(0.3)
            if not out["ok"]:
                pytest.skip(f"RTDE speed-slider write not accepted (control mode?): {out}")
            # Restore full speed so the sim is left as we found it.
            robot.set_speed_override(1.0)
        finally:
            # Release the RTDE input claim so a leaked write-session can't hold
            # the speed slider and silently throttle later motion tests.
            robot.close()

    def test_speed_override_unsafe_refused(self, ursim_ready):
        robot = _running_robot()
        out = robot.set_speed_override(1.5)
        assert out["ok"] is False and "safety" in out


# ---------------------------------------------------------------------------
# Trajectory streaming — the fast, robust way to move the robot a lot. A single
# program over one Primary connection, vs the per-move path which pays a
# handshake per move and wedges/powers-off the controller after ~10 rapid
# reconnects (URControl logs Socket::OperatorOverload). These tests both verify
# the motion lands and that driving the arm hard leaves the controller healthy.
# ---------------------------------------------------------------------------


class TestTrajectoryStress:
    # Speeds stay under the safety envelope's joint-speed cap (~2.09 rad/s) so
    # the moves are accepted; the point is to swing the arm through many
    # waypoints, not to trip the limiter.
    def _swing(self, n: int) -> list[list[float]]:
        wps = []
        for i in range(n):
            wps.append(
                [
                    0.4 * math.sin(i * 0.3),
                    -1.0 + 0.3 * math.sin(i * 0.5),
                    1.2,
                    -1.8,
                    -1.5708,
                    0.8 * math.sin(i * 0.6),
                ]
            )
        wps.append(READY_JOINTS)  # come to rest at a known pose
        return wps

    def test_long_trajectory_moves_a_lot_and_lands(self, ursim_ready):
        robot = _running_robot()
        wps = self._swing(24)
        t0 = time.monotonic()
        out = robot.move_trajectory(wps, velocity=1.5, acceleration=2.5, timeout=120.0)
        elapsed = time.monotonic() - t0
        assert out["ok"], out
        # It actually moved through the path (mid-swing waypoint is far from start)
        # and came to rest at READY.
        assert out["landed"] is not None
        landed = out["landed"]
        assert max(abs(a - b) for a, b in zip(landed, READY_JOINTS, strict=False)) < 0.05
        # The whole 25-waypoint path executes in well under the per-move cost it
        # would take (25 confirmed moves ~= 25 s+ and would risk wedging).
        assert elapsed < 90, f"trajectory took {elapsed:.0f}s"

    def test_hard_driving_leaves_controller_healthy(self, ursim_ready):
        """Drive several back-to-back trajectories, then assert the controller is
        still RUNNING + NORMAL — the streamed path must not fault the arm off the
        way rapid per-move reconnects do."""
        robot = _running_robot()
        for _ in range(3):
            out = robot.move_trajectory(self._swing(12), velocity=1.5, acceleration=2.5, timeout=90.0)
            assert out["ok"], out
        assert "RUNNING" in robot.robot_mode()
        assert "NORMAL" in robot.safety_mode()

    def test_trajectory_rejects_out_of_limit_speed(self, ursim_ready):
        """The envelope rejects a too-fast trajectory cleanly (no opaque
        controller-side socket close)."""
        robot = _running_robot()
        out = robot.move_trajectory([READY_JOINTS, READY_JOINTS], velocity=3.0)
        assert out["ok"] is False
        assert out["safety"]["violations"]
