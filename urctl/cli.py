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


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="urctl", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--host", default=None, help="controller host/IP (default: $UR_HOST or localhost)"
    )
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
