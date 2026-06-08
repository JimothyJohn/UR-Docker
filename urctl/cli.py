"""urctl command-line interface — a human/scripting entry point.

    urctl state                         # read robot state (JSON)
    urctl --host 10.0.0.5 bring-up      # cold start a real robot
    urctl move-joints 0 -1.57 0 -1.57 0 0
    urctl move-tcp 0 0.05 0 0 0 0 --relative   # nudge +50 mm along base +Y
    urctl move-home --velocity 0.3
    urctl load MotionDemo && urctl play
    urctl popup "hello from urctl"
    urctl tools                         # dump the agent tool schemas (JSON)
    urctl call ur_move_joints --json '{"joints":[0,-1.57,0,-1.57,0,0]}'

Connection target comes from ``--host`` or ``UR_HOST`` (default localhost), so
the same commands work against the URSim container and a real controller.
Every command prints its structured result as JSON and exits non-zero if the
action reported ``ok=false``.
"""

from __future__ import annotations

import argparse
import json
import sys

from .config import RobotConfig
from .robot import Robot
from .tools import ToolError, call_tool, get_tool_schemas


def _emit(result: dict) -> int:
    print(json.dumps(result, indent=2, default=str))
    return 0 if result.get("ok", True) else 1


GUIDED_HELP = """Guided build — issue a step; the robot asks Yes/No on its pendant, then
(on Yes) does it AND records it into the program. On quit the program is saved.
  movej J1..J6 [; description]            MoveJ to six joint angles (radians)
  movetcp X Y Z RX RY RZ [rel] [; desc]   MoveL to a TCP pose (m/rad); 'rel' = base-frame delta
  out PIN on|off [; description]          set standard digital output PIN
  comment TEXT                            add a Comment node (no robot action)
  summary                                 print confirmed/recorded tally
  save [PATH]                             write the .urp now (PATH or --save)
  help | quit"""


def run_guided_command(session, line: str) -> dict:
    """Execute one console line against a :class:`GuidedSession`.

    Step verbs (movej/movetcp/out/comment) confirm-then-act-then-record via the
    session and return ``{"kind": "step", "result": StepResult}``. Control verbs
    return ``{"kind": "control", "action": ...}`` for the REPL to handle. Blank
    and ``#``-prefixed lines are ``{"kind": "noop"}``. Bad input raises
    ``ValueError`` (the REPL prints it and continues). Factored out of the REPL
    loop so it can be unit-tested without stdin.
    """
    line = line.strip()
    if not line or line.startswith("#"):
        return {"kind": "noop"}
    head = line.split(None, 1)
    verb = head[0].lower()

    # comment takes the whole rest of the line verbatim (no ';' splitting).
    if verb == "comment":
        text = head[1].strip() if len(head) > 1 else ""
        if not text:
            raise ValueError("comment requires text")
        return {"kind": "step", "result": session.comment(text)}

    cmd_part, _, desc = line.partition(";")
    desc = desc.strip() or None
    rest = cmd_part.split()[1:]

    if verb in ("quit", "exit", "q"):
        return {"kind": "control", "action": "quit"}
    if verb in ("help", "?"):
        return {"kind": "control", "action": "help"}
    if verb == "summary":
        return {"kind": "control", "action": "summary", "summary": session.summary()}
    if verb == "save":
        return {"kind": "control", "action": "save", "path": rest[0] if rest else None}
    if verb in ("movej", "move-joints"):
        joints = [float(x) for x in rest]
        if len(joints) != 6:
            raise ValueError("movej needs 6 joint angles")
        return {"kind": "step", "result": session.move_joints(joints, desc=desc)}
    if verb in ("movetcp", "move-tcp"):
        nums = rest
        relative = bool(nums) and nums[-1].lower() in ("rel", "relative")
        if relative:
            nums = nums[:-1]
        pose = [float(x) for x in nums]
        if len(pose) != 6:
            raise ValueError("movetcp needs 6 pose values (optionally followed by 'rel')")
        return {"kind": "step", "result": session.move_tcp(pose, relative=relative, desc=desc)}
    if verb in ("out", "set-output"):
        if len(rest) < 2 or rest[1].lower() not in ("on", "off"):
            raise ValueError("out needs: <pin> on|off")
        return {
            "kind": "step",
            "result": session.set_output(int(rest[0]), rest[1].lower() == "on", desc=desc),
        }
    raise ValueError(f"unknown command: {verb!r} (try 'help')")


def build_live_reloader(robot, args):
    """Return a LiveReloader for ``--live`` (or None if not requested).

    ``--live-program-dir`` (a host-reachable controller program dir) wins over
    ``--live-container`` (docker cp into this repo's URSim). The reload target
    must be loadable by the controller, hence one of these two placers.
    """
    if not args.live:
        return None
    from .guided import LiveReloader, docker_placer, local_dir_placer

    if args.live_program_dir:
        placer = local_dir_placer(args.live_program_dir, installation=args.installation)
    else:
        placer = docker_placer(args.live_container, installation=args.installation)
    return LiveReloader(
        robot,
        args.name,
        placer,
        local_path=args.save,
        logger=lambda m: print(f"! {m}", file=sys.stderr),
    )


def _guided_repl(robot, args) -> int:
    from .guided import GuidedSession

    session = GuidedSession(
        robot,
        args.name,
        installation=args.installation,
        confirm_timeout=args.confirm_timeout,
        freedrive=args.freedrive,
        on_record=build_live_reloader(robot, args),
    )
    out_path = args.save
    if args.live:
        print("live reload ON — the PolyScope tree updates after each approved step.", file=sys.stderr)
    if args.freedrive:
        print(
            "freedrive ON — after each move, hand-guide the pose and tap Yes to record it.", file=sys.stderr
        )
    print(f"guided build '{args.name}' — 'help' for commands, 'quit' to finish.", file=sys.stderr)
    print(GUIDED_HELP, file=sys.stderr)
    for raw in sys.stdin:
        try:
            res = run_guided_command(session, raw)
        except ValueError as exc:
            print(f"! {exc}", file=sys.stderr)
            continue
        kind = res["kind"]
        if kind == "noop":
            continue
        if kind == "control":
            action = res["action"]
            if action == "quit":
                break
            if action == "help":
                print(GUIDED_HELP, file=sys.stderr)
            elif action == "summary":
                print(json.dumps(res["summary"]), file=sys.stderr)
            elif action == "save":
                path = res["path"] or out_path
                if not path:
                    print("! save needs a path (or start with --save PATH)", file=sys.stderr)
                else:
                    out_path = path
                    print(f"saved {session.save(path)}", file=sys.stderr)
            continue
        r = res["result"]
        status = (
            "recorded"
            if r.recorded
            else "rejected"
            if r.confirmed is False
            else "no-answer"
            if r.confirmed is None
            else "not-recorded"
        )
        print(f"[{status}] {r.kind}: {r.desc}", file=sys.stderr)

    if out_path and any(s.recorded for s in session.steps):
        print(f"saved {session.save(out_path)}", file=sys.stderr)
    return _emit({"action": "guided", "ok": True, **session.summary()})


def _inspect_app(robot, args) -> int:
    from .guided import GuidedSession, run_inspection

    session = GuidedSession(
        robot,
        args.name,
        installation=args.installation,
        confirm_timeout=args.confirm_timeout,
        freedrive=True,  # inspection is hand-guided by definition
        on_record=build_live_reloader(robot, args),
    )
    print(
        "multi-point inspection — hand-guide the robot to each point, then on the "
        "pendant tap Yes to capture (fires the camera subprogram) or No/Cancel to finish.",
        file=sys.stderr,
    )
    if args.live:
        print("live reload ON — the tree grows one point per capture.", file=sys.stderr)
    n = run_inspection(session, call=args.call, max_points=args.max_points)
    print(f"captured {n} inspection point(s).", file=sys.stderr)
    if args.save and any(s.recorded for s in session.steps):
        print(f"saved {session.save(args.save)}", file=sys.stderr)
    return _emit({"action": "inspect", "ok": True, "points": n, **session.summary()})


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="urctl", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--host", default=None, help="controller host/IP (default: $UR_HOST or localhost)")
    ap.add_argument(
        "--platform",
        choices=["e-series", "polyscopex"],
        default=None,
        help="controller software: e-series (Dashboard) or polyscopex (REST Robot-API). "
        "Default: $UR_PLATFORM or e-series.",
    )
    ap.add_argument(
        "--robot-api-port",
        type=int,
        default=None,
        help="PolyScope X Robot-API HTTP port (default: $UR_ROBOT_API_PORT or 80; "
        "this repo's URSim PolyScope X publishes it on 8000)",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="validate, audit, and print actions without sending them",
    )
    ap.add_argument(
        "--audit-log", default=None, help="append a JSON-lines audit record per action to this file"
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("state", help="read and print robot state")
    sub.add_parser("rtde-state", help="read high-rate structured state via RTDE (port 30004)")
    sub.add_parser("bring-up", help="cold start to RUNNING (power on + brake release)")
    sub.add_parser("power-off", help="power off motors")

    sp = sub.add_parser("speed", help="set the global speed slider (0-1) via RTDE")
    sp.add_argument("fraction", type=float, help="speed scale 0-1 (1.0 = full programmed speed)")

    so = sub.add_parser("set-output", help="set a standard digital output pin via RTDE")
    so.add_argument("pin", type=int, help="output pin 0-7")
    so.add_argument("value", choices=["on", "off"])

    mj = sub.add_parser("move-joints", help="movej to six joint angles (radians)")
    mj.add_argument("joints", type=float, nargs=6, metavar="J")
    mj.add_argument("--velocity", type=float, default=None)
    mj.add_argument("--acceleration", type=float, default=None)

    mt = sub.add_parser("move-tcp", help="movel to a TCP pose (absolute or --relative delta)")
    mt.add_argument(
        "pose",
        type=float,
        nargs=6,
        metavar="P",
        help="x y z rx ry rz (metres + rotation-vector radians)",
    )
    mt.add_argument(
        "--relative",
        action="store_true",
        help="treat pose as a base-frame delta from the current TCP (e.g. 0 0.05 0 0 0 0 = +50mm Y)",
    )
    mt.add_argument("--velocity", type=float, default=None, help="linear speed in m/s")
    mt.add_argument("--acceleration", type=float, default=None, help="linear acceleration in m/s^2")

    mh = sub.add_parser("move-home", help="movej to the safe home pose")
    mh.add_argument("--velocity", type=float, default=None)
    mh.add_argument("--acceleration", type=float, default=None)

    fd = sub.add_parser("freedrive", help="enable/disable hand-guiding")
    fd.add_argument("state", choices=["on", "off"])

    pu = sub.add_parser("popup", help="show a popup on the teach pendant")
    pu.add_argument("text")

    ld = sub.add_parser("load", help="load a <name>.urp program")
    ld.add_argument("name")

    sub.add_parser("play", help="play the loaded program")
    sub.add_parser("stop", help="stop the running program")
    sub.add_parser("pause", help="pause the running program")

    rs = sub.add_parser("run-script", help="run URScript (from arg or stdin)")
    rs.add_argument("script", nargs="?", help="URScript text; omit to read stdin")
    rs.add_argument(
        "--raw",
        action="store_true",
        help="send verbatim (no def-wrap); bare top-level motion may silently not run",
    )
    rs.add_argument("--capture", action="store_true", help="capture textmsg output")
    rs.add_argument("--marker", default="", help="only return captured lines with this marker")
    rs.add_argument(
        "--collect-for",
        type=float,
        default=2.0,
        help="seconds to read the broadcast for captured output (default: 2.0)",
    )

    dc = sub.add_parser("dashboard", help="send a raw Dashboard command")
    dc.add_argument("command", nargs="+", help="command words, e.g. robotmode")

    gd = sub.add_parser("guided", help="interactively build a program, confirming each step on the pendant")
    gd.add_argument("name", help="program name (also the recorded URP name)")
    gd.add_argument("--save", default=None, help="path to write the .urp (also auto-saved on quit)")
    gd.add_argument("--installation", default="default", help="sibling .installation name (default: default)")
    gd.add_argument(
        "--confirm-timeout",
        type=float,
        default=120.0,
        help="seconds to wait for a pendant Yes/No before treating a step as skipped",
    )
    gd.add_argument(
        "--freedrive",
        action="store_true",
        help="after each move, drop into freedrive so the operator can hand-guide the "
        "exact pose and tap Yes; the adjusted pose is what gets recorded",
    )
    gd.add_argument(
        "--live",
        action="store_true",
        help="after each approved step, reload the program into PolyScope so its "
        "tree grows node-by-node as you build (needs the .urp on the controller)",
    )
    gd.add_argument(
        "--live-container",
        default="ur-docker-ursim-1",
        help="docker container to publish the .urp into for --live (this repo's URSim)",
    )
    gd.add_argument(
        "--live-program-dir",
        default=None,
        help="host-reachable controller program dir for --live (overrides --live-container; "
        "use for a real robot whose program folder is a mounted share)",
    )

    insp = sub.add_parser(
        "inspect",
        help="multi-point inspection: hand-guide to each point, capture via a camera subprogram",
    )
    insp.add_argument("name", help="program name (also the recorded URP name)")
    insp.add_argument("--save", default=None, help="path to write the .urp")
    insp.add_argument("--installation", default="default", help="sibling .installation name")
    insp.add_argument(
        "--confirm-timeout",
        type=float,
        default=86400.0,
        help="seconds to wait for the pendant answer at each point (default: effectively none)",
    )
    insp.add_argument(
        "--call",
        default="trigger_camera()",
        help="subprogram call inserted at each point (default: trigger_camera())",
    )
    insp.add_argument(
        "--max-points",
        type=int,
        default=None,
        help="stop after N points (default: until the operator answers No/Cancel)",
    )
    insp.add_argument("--live", action="store_true", help="reload the program after each capture")
    insp.add_argument("--live-container", default="ur-docker-ursim-1", help="container for --live")
    insp.add_argument("--live-program-dir", default=None, help="controller program dir for --live")

    sub.add_parser("tools", help="print the agent tool schemas as JSON")

    ct = sub.add_parser("call", help="dispatch a tool by name")
    ct.add_argument("name")
    ct.add_argument(
        "--json",
        dest="json_args",
        default="{}",
        help="tool arguments as a JSON object (default: {})",
    )

    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    # `tools` needs no connection — print and exit.
    if args.cmd == "tools":
        print(json.dumps(get_tool_schemas(), indent=2))
        return 0

    # Only pass through flags the user actually set, so each one falls back to
    # its environment variable (then the default) when omitted.
    overrides: dict = {}
    if args.platform is not None:
        overrides["platform"] = args.platform
    if args.robot_api_port is not None:
        overrides["robot_api_port"] = args.robot_api_port
    config = RobotConfig.from_env(host=args.host, **overrides)
    robot = Robot(config, dry_run=args.dry_run)
    if args.audit_log:
        from .audit import AuditLog

        robot.audit = AuditLog(path=args.audit_log)

    if args.cmd == "state":
        return _emit(robot.get_state())
    if args.cmd == "rtde-state":
        return _emit(robot.rtde_state())
    if args.cmd == "bring-up":
        return _emit(robot.bring_up())
    if args.cmd == "power-off":
        return _emit(robot.power_off())
    if args.cmd == "speed":
        return _emit(robot.set_speed_override(args.fraction))
    if args.cmd == "set-output":
        return _emit(robot.set_digital_output(args.pin, args.value == "on"))
    if args.cmd == "move-joints":
        kwargs = {}
        if args.velocity is not None:
            kwargs["velocity"] = args.velocity
        if args.acceleration is not None:
            kwargs["acceleration"] = args.acceleration
        return _emit(robot.move_joints(args.joints, **kwargs))
    if args.cmd == "move-tcp":
        kwargs = {"relative": args.relative}
        if args.velocity is not None:
            kwargs["velocity"] = args.velocity
        if args.acceleration is not None:
            kwargs["acceleration"] = args.acceleration
        return _emit(robot.move_tcp(args.pose, **kwargs))
    if args.cmd == "move-home":
        kwargs = {}
        if args.velocity is not None:
            kwargs["velocity"] = args.velocity
        if args.acceleration is not None:
            kwargs["acceleration"] = args.acceleration
        return _emit(robot.move_home(**kwargs))
    if args.cmd == "freedrive":
        return _emit(robot.freedrive(args.state == "on"))
    if args.cmd == "popup":
        return _emit(robot.popup(args.text))
    if args.cmd == "load":
        return _emit(robot.load_program(args.name))
    if args.cmd == "play":
        return _emit(robot.play())
    if args.cmd == "stop":
        return _emit(robot.stop())
    if args.cmd == "pause":
        return _emit(robot.pause())
    if args.cmd == "run-script":
        script = args.script if args.script is not None else sys.stdin.read()
        return _emit(
            robot.run_script(
                script,
                wrap=not args.raw,
                capture=args.capture,
                marker=args.marker,
                collect_for=args.collect_for,
            )
        )
    if args.cmd == "guided":
        return _guided_repl(robot, args)
    if args.cmd == "inspect":
        return _inspect_app(robot, args)
    if args.cmd == "dashboard":
        return _emit(robot.dashboard_command(" ".join(args.command)))
    if args.cmd == "call":
        try:
            params = json.loads(args.json_args)
        except json.JSONDecodeError as exc:
            print(f"--json is not valid JSON: {exc}", file=sys.stderr)
            return 2
        try:
            return _emit(call_tool(robot, args.name, params))
        except ToolError as exc:
            print(f"tool error: {exc}", file=sys.stderr)
            return 2

    return 2


if __name__ == "__main__":
    sys.exit(main())
