#!/usr/bin/env python3
"""End-to-end driver for the UR-Docker URSim.

Walks a real cobot's operating envelope from "controller booted" to
"program ran successfully", using the documented backend ports:

  * Dashboard (29999)  — orchestration: power on, load program, query state.
  * Primary  (30001)   — execution: stream URScript that runs immediately,
                         and listen for textmsg() output in the broadcast.

Phases:

  1. Smoke test            popup() via Primary; confirm via state stream.
  2. Read state            textmsg the joint positions and TCP pose; capture.
  3. Execute motion        movej to a known safe pose; verify positions changed.
  4. Return home           movej back to the starting pose.
  5. Dashboard load        load MotionDemo.urp; confirm programState=STOPPED.
  6. Dashboard play        attempt play; handle Local-mode rejection by
                           falling back to running the same URScript via
                           Primary, which doesn't need Remote Control mode.

Each phase prints a one-line PASS/FAIL summary and writes a JSON line to
stdout for downstream consumers (the pytest integration test scrapes them).

Usage:
    ./scripts/e2e_drive.py [--host HOST] [--no-motion] [--json]
"""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Connection target. Defaults to the local URSim container, but every value is
# overridable so the same driver runs against a robot at an IP address:
#   UR_HOST=10.0.0.5 ./scripts/e2e_drive.py      (or --host 10.0.0.5)
DEFAULT_HOST = os.environ.get("UR_HOST", "localhost")
DASH_PORT = int(os.environ.get("UR_DASH_PORT", "29999"))
PRIMARY_PORT = int(os.environ.get("UR_PRIMARY_PORT", "30001"))

# A safe candle-ish home pose for UR10 — straight up, all wrists folded.
# Far from any singularity, well within joint limits, no self-collision.
HOME_JOINTS = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
# Small perturbation from home — visibly different but mechanically safe.
DEMO_JOINTS = [0.3, -1.4, 0.5, -1.6, 0.1, 0.2]


# ----- Dashboard (29999) -----------------------------------------------------


def dash(host: str, *cmds: str, timeout: float = 10.0) -> str:
    """Send Dashboard commands, return server reply (framing stripped)."""
    payload = ("\n".join(cmds) + "\nquit\n").encode("utf-8")
    with socket.create_connection((host, DASH_PORT), timeout=timeout) as s:
        s.sendall(payload)
        s.settimeout(timeout)
        chunks: list[bytes] = []
        while True:
            try:
                c = s.recv(4096)
            except TimeoutError:
                break
            if not c:
                break
            chunks.append(c)
    text = b"".join(chunks).decode("utf-8", errors="replace")
    return "\n".join(
        ln for ln in text.splitlines() if ln and not ln.startswith(("Connected:", "Disconnected"))
    )


def wait_until(
    host: str, predicate, *, query: str = "robotmode", timeout: float = 60.0, interval: float = 1.0
) -> str:
    """Poll Dashboard until `predicate(reply)` is truthy. Returns last reply."""
    end = time.monotonic() + timeout
    last = ""
    while time.monotonic() < end:
        try:
            last = dash(host, query)
        except (TimeoutError, OSError):
            last = ""
        if predicate(last):
            return last
        time.sleep(interval)
    raise TimeoutError(f"timed out waiting on {query!r}; last reply: {last!r}")


# ----- Primary (30001) -------------------------------------------------------

# Strings of >=4 printable ASCII chars, used to harvest textmsg output from
# the Primary client's binary state broadcast.
_ASCII_RUN = re.compile(rb"[\x20-\x7e]{4,}")


def primary_send(host: str, urscript: str) -> None:
    """Stream a URScript snippet to the Primary Client. The controller parses
    and executes immediately; this socket does NOT wait for a response —
    Primary is a broadcast channel."""
    with socket.create_connection((host, PRIMARY_PORT), timeout=5.0) as s:
        s.sendall(urscript.encode("utf-8"))


def primary_send_and_capture(
    host: str, urscript: str, *, marker: str = "e2e/", collect_for: float = 2.0
) -> list[str]:
    """Send URScript to Primary, then keep reading the broadcast for
    `collect_for` seconds, returning every ASCII run containing `marker`.

    The Primary stream interleaves binary state packets with textmsg()
    output frames; we don't bother decoding the protocol — we just scan
    for printable ASCII runs and keep the ones that match the marker.
    """
    with socket.create_connection((host, PRIMARY_PORT), timeout=5.0) as s:
        s.sendall(urscript.encode("utf-8"))
        s.settimeout(0.5)
        buf = bytearray()
        end = time.monotonic() + collect_for
        while time.monotonic() < end:
            try:
                chunk = s.recv(8192)
            except TimeoutError:
                continue
            if not chunk:
                break
            buf.extend(chunk)
    marker_b = marker.encode("utf-8")
    return [
        m.group(0).decode("ascii", errors="replace")
        for m in _ASCII_RUN.finditer(bytes(buf))
        if marker_b in m.group(0)
    ]


# Match a 6-element bracketed vector. Each element can be:
#   bare int (`0`), decimal (`-1.5708`), or scientific (`1.11e-16`).
# URSim emits a mix — bare `0` shows up when a value is exactly zero.
_NUM = r"-?\d+(?:\.\d+)?(?:e[+-]?\d+)?"
_JOINTS_RX = re.compile(rf"\[({_NUM}(?:,{_NUM}){{5}})\]")


def parse_joints(captured: list[str], tag: str) -> list[float] | None:
    """Pull a 6-vector out of a `tag=[...]` textmsg line."""
    for line in captured:
        if tag in line:
            m = _JOINTS_RX.search(line)
            if m:
                return [float(x) for x in m.group(1).split(",")]
    return None


# ----- Phase implementations -------------------------------------------------


@dataclass
class PhaseResult:
    name: str
    ok: bool
    detail: dict[str, object] = field(default_factory=dict)

    def line(self) -> str:
        flag = "PASS" if self.ok else "FAIL"
        return f"[{flag}] {self.name}  {json.dumps(self.detail, default=str)}"


# URControl treats each newline-terminated statement as a separate program
# and kills the previous one when a new one starts — fatal for movej (it
# never gets to execute). The workaround is to wrap the whole submission in
# a single `def`, which keeps URControl on one program for the duration.
# Side effect: URControl.log will note a `Compile error: name '<fn>' is not
# defined` after each run; the function body still executes (PolyScope's
# auto-wrapping interferes with the outer call, not the inner body), so
# this is cosmetic log noise we accept.


def _wrap(fn_name: str, body: str) -> str:
    return f"def {fn_name}():\n{body}end\n{fn_name}()\n"


def phase1_smoke(host: str) -> PhaseResult:
    captured = primary_send_and_capture(
        host,
        _wrap(
            "e2e_smoke",
            '  popup("e2e/phase1 popup", title="e2e", blocking=False)\n  textmsg("e2e/phase1/ok")\n',
        ),
        collect_for=2.0,
    )
    ok = any("e2e/phase1/ok" in line for line in captured)
    return PhaseResult("Phase 1: smoke test (Primary 30001)", ok, {"captured": captured})


def phase2_read_state(host: str) -> PhaseResult:
    """Read joint positions + TCP pose via textmsg, parsed out of broadcast."""
    captured = primary_send_and_capture(
        host,
        _wrap(
            "e2e_probe",
            '  textmsg("e2e/phase2/joints=", get_actual_joint_positions())\n'
            '  textmsg("e2e/phase2/tcp=",    get_actual_tcp_pose())\n',
        ),
        collect_for=2.0,
    )
    joints = parse_joints(captured, "e2e/phase2/joints")
    tcp = parse_joints(captured, "e2e/phase2/tcp")
    ok = joints is not None and tcp is not None
    return PhaseResult("Phase 2: read state", ok, {"joints": joints, "tcp": tcp})


def phase3_motion(host: str, target: list[float]) -> PhaseResult:
    """Drive a movej. Verify by reading the post-move joint positions."""
    body = (
        f"  movej({target}, a=0.8, v=0.5)\n"
        "  sync()\n"
        '  textmsg("e2e/phase3/joints=", get_actual_joint_positions())\n'
    )
    urscript = _wrap("e2e_drive", body)
    # A single movej spanning ~3 rad takes ~6 s at v=0.5; give comfortable
    # headroom plus a beat for the final textmsg to land in the broadcast.
    captured = primary_send_and_capture(host, urscript, collect_for=12.0)
    landed = parse_joints(captured, "e2e/phase3/joints")
    if landed is None:
        return PhaseResult("Phase 3: motion", False, {"target": target, "captured": captured})
    # Each joint must land within 0.05 rad (~3 deg) of target — URSim's
    # virtual servos hit exact values, but tolerate a tiny margin.
    err = [abs(a - b) for a, b in zip(landed, target, strict=False)]
    ok = max(err) < 0.05
    return PhaseResult("Phase 3: motion", ok, {"target": target, "landed": landed, "max_error": max(err)})


def phase4_home(host: str) -> PhaseResult:
    """Drive back to the canonical home pose."""
    r = phase3_motion(host, HOME_JOINTS)
    r.name = "Phase 4: return to home pose"
    return r


# ----- MotionDemo program build + load --------------------------------------

MOTION_DEMO_SCRIPT = REPO_ROOT / "programs" / "MotionDemo" / "MotionDemo.script"
MOTION_DEMO_URP = REPO_ROOT / "programs" / "MotionDemo" / "MotionDemo.urp"
MOTION_DEMO_INST = REPO_ROOT / "programs" / "MotionDemo" / "MotionDemo.installation"


def phase5_dashboard_load(host: str, docker: list[str], container: str) -> PhaseResult:
    """Drop MotionDemo into the container and have Dashboard load it."""
    if not MOTION_DEMO_URP.exists():
        return PhaseResult(
            "Phase 5: Dashboard load",
            False,
            {"error": f"missing {MOTION_DEMO_URP}; run make regen-urps"},
        )
    # Mirror the URP + installation into URSim's program dir.
    for src, dst in [
        (MOTION_DEMO_URP, "/ursim/programs/MotionDemo.urp"),
        (MOTION_DEMO_INST, "/ursim/programs/MotionDemo.installation"),
        (MOTION_DEMO_SCRIPT, "/ursim/programs/MotionDemo.script"),
    ]:
        subprocess.run([*docker, "cp", str(src), f"{container}:{dst}"], check=True, capture_output=True)
    load_reply = dash(host, "load MotionDemo.urp")
    state = dash(host, "programState")
    # Loading a URP that's paired with a fresh installation file makes
    # PolyScope re-evaluate the installation, which on URSim sometimes
    # transitions the robot back to POWER_OFF (the "Confirm Installation"
    # safety prompt). Re-power if that happened, so subsequent phases
    # can actually execute motion.
    mode_after = dash(host, "robotmode")
    repowered = False
    if "POWER_OFF" in mode_after:
        dash(host, "power on")
        wait_until(host, lambda r: "IDLE" in r or "RUNNING" in r, timeout=30.0)
        dash(host, "brake release")
        wait_until(host, lambda r: "RUNNING" in r, timeout=30.0)
        repowered = True
    ok = "Loading program" in load_reply and "MotionDemo.urp" in state and "STOPPED" in state
    return PhaseResult(
        "Phase 5: Dashboard load",
        ok,
        {
            "load_reply": load_reply,
            "programState": state,
            "robotmode_after_load": mode_after,
            "repowered_after_load": repowered,
        },
    )


def phase6_play_or_primary_fallback(host: str) -> PhaseResult:
    """Try Dashboard play; if blocked by Local mode, run the same script
    directly via Primary 30001."""
    reply = dash(host, "play")
    if "Starting program" in reply:
        # Wait for it to finish.
        try:
            wait_until(host, lambda r: "STOPPED" in r, query="programState", timeout=30.0)
        except TimeoutError:
            return PhaseResult(
                "Phase 6: play loaded program",
                False,
                {"path": "dashboard", "error": "program never reached STOPPED"},
            )
        return PhaseResult("Phase 6: play loaded program", True, {"path": "dashboard", "play_reply": reply})

    # Dashboard refused (likely Local mode — `is in remote control` would
    # say false). Fall back: run the same motions via Primary 30001, which
    # works regardless of control mode. This is what external drivers
    # (ROS ur_robot_driver, MoveIt) do.
    #
    # Defensive steps before submitting:
    #  - `dash stop` clears any half-armed program state Dashboard might
    #    have left behind after the rejected play.
    #  - sleep 1 s gives URControl time to settle.
    #  - retry once if the first capture comes back empty (rare race
    #    where the Primary socket is opened before URControl finishes
    #    processing the prior Dashboard transaction).
    dash(host, "stop")
    time.sleep(1.0)

    home = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
    poseA = [0.3, -1.4, 0.5, -1.6, 0.1, 0.2]
    poseB = [-0.3, -1.7, -0.5, -1.4, -0.1, -0.2]
    body = (
        '  textmsg("e2e/motiondemo/start")\n'
        f"  movej({home},  a=0.8, v=0.5)\n"
        '  textmsg("e2e/motiondemo/at_home")\n'
        f"  movej({poseA}, a=0.8, v=0.5)\n"
        '  textmsg("e2e/motiondemo/at_poseA")\n'
        f"  movej({poseB}, a=0.8, v=0.5)\n"
        '  textmsg("e2e/motiondemo/at_poseB")\n'
        f"  movej({home},  a=0.8, v=0.5)\n"
        '  textmsg("e2e/motiondemo/done")\n'
    )
    urscript = f"def e2e_motiondemo_fallback():\n{body}end\ne2e_motiondemo_fallback()\n"

    captured: list[str] = []
    for _attempt in (1, 2):
        captured = primary_send_and_capture(host, urscript, marker="e2e/motiondemo", collect_for=25.0)
        if any("e2e/motiondemo/done" in c for c in captured):
            break
        time.sleep(2.0)
    ok = any("e2e/motiondemo/done" in c for c in captured)
    return PhaseResult(
        "Phase 6: play loaded program",
        ok,
        {
            "path": "primary_fallback",
            "dashboard_reply": reply,
            "note": (
                "Dashboard rejected `play` (PolyScope in Local "
                "control mode). Same motion sequence dispatched "
                "via Primary 30001 — control-mode-agnostic."
            ),
            "captured_tail": captured[-6:],
        },
    )


# ----- main ------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--container", default="ur-docker-ursim-1")
    ap.add_argument(
        "--docker",
        # Honour the DOCKER env var (matches tests/_ursim.py and the CI job) so a
        # passwordless-docker host doesn't need --docker; default to sudo docker.
        default=os.environ.get("DOCKER", "sudo docker"),
        help="docker invocation; e.g. 'docker' if no sudo needed (or set DOCKER=docker)",
    )
    ap.add_argument(
        "--no-motion", action="store_true", help="skip motion phases (useful for headless CI smoke)"
    )
    ap.add_argument("--json", action="store_true", help="emit one JSON object per phase to stdout")
    args = ap.parse_args(argv)
    docker = args.docker.split()

    # Sanity check: must be powered on before motion phases.
    mode = dash(args.host, "robotmode")
    if "RUNNING" not in mode:
        print(f"robot not RUNNING (got: {mode!r}). Run scripts/poweron.sh first.", file=sys.stderr)
        return 2

    phases: list[PhaseResult] = []
    phases.append(phase1_smoke(args.host))
    phases.append(phase2_read_state(args.host))

    if not args.no_motion:
        phases.append(phase3_motion(args.host, DEMO_JOINTS))
        phases.append(phase4_home(args.host))

    phases.append(phase5_dashboard_load(args.host, docker, args.container))
    phases.append(phase6_play_or_primary_fallback(args.host))

    for p in phases:
        if args.json:
            print(json.dumps({"phase": p.name, "ok": p.ok, **p.detail}, default=str))
        else:
            print(p.line())

    return 0 if all(p.ok for p in phases) else 1


if __name__ == "__main__":
    sys.exit(main())
