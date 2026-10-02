"""perceive command-line interface — a human/scripting entry point.

    perceive synthetic                     # run the pipeline on a synthetic frame
    perceive capture                       # grab one webcam frame + detect blobs
    perceive --depth-backend depth_anything capture
    perceive tools                         # dump the agent tool schemas (JSON)
    perceive call perceive_synthetic --json '{"width":320,"height":240}'
    perceive --cell ur20 doctor            # pre-flight: SDK, camera, robot reachability + state
    perceive cells                         # the shipped cell profiles (sim / ur3 / ur20)
    perceive rs-info                       # RealSense devices + SDK (needs librealsense2)
    perceive gui --fake                    # the RGB-D cockpit (synthetic scene; drop --fake for the camera)

Camera + backend selection come from ``--device`` / ``--width`` / ``--height``
/ ``--fps`` / ``--depth-backend`` / ``--blob-backend`` or the matching
``PERCEPTRONICS_*`` env vars, so the same commands run against a laptop webcam, a
camera-less CI box (``synthetic``), or a GPU host (the real depth model).

Every command prints its structured result as JSON and exits non-zero if the
result reported ``ok=false``.
"""

from __future__ import annotations

import argparse
import json
import sys

from .cell import ENV_CELL, apply_cell, list_cells
from .config import PerceptionConfig
from .orbitcal import add_calibrate_args, run_calibrate
from .pickcycle import add_pick_cycle_args, run_pick_cycle
from .picksidecar import add_pick_server_args, run_pick_server
from .pipeline import PerceptionPipeline
from .tools import ToolError, call_tool, get_tool_schemas
from .webapp import (
    DEFAULT_PORT,
    add_camera_args,
    add_cors_arg,
    add_pick_port_arg,
    add_robot_args,
    camera_from_args,
    cors_from_args,
    robot_from_args,
    views_from_args,
)


def _emit(result: dict) -> int:
    print(json.dumps(result, indent=2, default=str))
    return 0 if result.get("ok", True) else 1


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="perceptronics", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--cell",
        default=None,
        help=f"cell profile: {'|'.join(list_cells())} or a path to a .env file (default: ${ENV_CELL}); "
        "sets UR_HOST/ports/bracket unless already set in the environment",
    )
    ap.add_argument(
        "--device", type=int, default=None, help="camera index (default: $PERCEPTRONICS_DEVICE or 0)"
    )
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument("--fps", type=int, default=None)
    ap.add_argument(
        "--depth-backend",
        default=None,
        help="stub | depth_anything (default: $PERCEPTRONICS_DEPTH_BACKEND or stub)",
    )
    ap.add_argument(
        "--blob-backend",
        default=None,
        help="stub | blob_cv (default: $PERCEPTRONICS_BLOB_BACKEND or stub)",
    )
    ap.add_argument("--min-blob-area", type=int, default=None, help="drop blobs smaller than this (px)")
    ap.add_argument(
        "--segment-backend",
        default=None,
        help="stub | sam (default: $PERCEPTRONICS_SEGMENT_BACKEND or stub) — click-to-segment in the gui",
    )

    ap.add_argument(
        "--sam-model",
        default=None,
        help="SAM checkpoint id for the sam backend (default: $PERCEPTRONICS_SAM_MODEL)",
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    sy = sub.add_parser("synthetic", help="process a deterministic synthetic frame (no camera)")
    sy.add_argument("--width", type=int, default=None, dest="syn_width")
    sy.add_argument("--height", type=int, default=None, dest="syn_height")

    sub.add_parser("capture", help="grab one webcam frame and detect blobs")

    im = sub.add_parser("image", help="run the pipeline on a PNG file")
    im.add_argument("path", help="path to an 8-bit RGB/RGBA PNG")
    im.add_argument(
        "--max-width",
        type=int,
        default=480,
        help="downsample so width <= this before the pure-Python pipeline (default: 480)",
    )

    sub.add_parser("tools", help="print the agent tool schemas as JSON")

    ce = sub.add_parser("cells", help="list the shipped cell profiles and what each sets")
    ce.add_argument(
        "--export",
        metavar="CELL",
        default=None,
        help='print `export KEY=VALUE` lines for one cell, for `eval "$(perceptronics cells --export ur20)"` '
        "so plain `urctl` commands see the same host/ports",
    )

    dr = sub.add_parser("doctor", help="pre-flight checklist: SDK, camera, robot reachability + state")
    dr.add_argument("--stream", action="store_true", help="also open the camera and judge frames")
    dr.add_argument("--no-camera", action="store_true", help="skip the SDK/camera checks")
    dr.add_argument("--no-robot", action="store_true", help="skip the controller checks")
    dr.add_argument("--json", dest="as_json", action="store_true", help="machine-readable report")
    dr.add_argument("--library", default=None, help="path to librealsense2 (default: $REALSENSE_LIB / auto)")
    dr.add_argument(
        "--cockpit-url",
        default=f"http://127.0.0.1:{DEFAULT_PORT}",
        help="report on a running cockpit at this URL (default: the local one)",
    )

    ct = sub.add_parser("call", help="dispatch a perceptronics tool by name")
    ct.add_argument("name")
    ct.add_argument("--json", dest="json_args", default="{}", help="tool args as a JSON object")

    ri = sub.add_parser("rs-info", help="list attached RealSense cameras + SDK version (no streaming)")
    ri.add_argument("--library", default=None, help="path to librealsense2 (default: $REALSENSE_LIB / auto)")
    ri.add_argument(
        "--options",
        action="store_true",
        help="also dump every sensor option (value/range) per sensor — what a preset or another app left",
    )

    gu = sub.add_parser(
        "gui", help="local RGB-D cockpit: live view, click-to-segment, snapshots, send-to-robot"
    )
    add_camera_args(gu)
    add_robot_args(gu)
    gu.add_argument("--bind", default="127.0.0.1", help="interface to bind (default: loopback only)")
    gu.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"port (default {DEFAULT_PORT})")
    gu.add_argument("--no-browser", action="store_true", help="don't open the browser automatically")
    gu.add_argument("--demo", action="store_true", help="open the demo view: one picture, four big buttons")
    add_cors_arg(gu)
    add_pick_port_arg(gu)

    pc = sub.add_parser(
        "pick-cycle",
        help="heuristic routine on a running cockpit: lift every block in view an inch and set it back "
        "(--drop to shuffle)",
    )
    add_pick_cycle_args(pc)

    ps = sub.add_parser(
        "pick-server",
        help="the PolyScope Perceptronic Pick node's server beside an already-running cockpit "
        "(its pick socket + teach routes; everything else forwarded to the cockpit)",
    )
    add_pick_server_args(ps)

    cb = sub.add_parser(
        "calibrate",
        help="mark-less hand-eye calibration on a running cockpit: orbit the block under the camera, "
        "click its top-face centre into the cockpit's calibration at every view, solve, trim "
        "(--apply to keep)",
    )
    add_calibrate_args(cb)
    return ap


def _config_from_args(args) -> PerceptionConfig:
    overrides: dict[str, object] = {}
    if args.device is not None:
        overrides["device_index"] = args.device
    if args.width is not None:
        overrides["width"] = args.width
    if args.height is not None:
        overrides["height"] = args.height
    if args.fps is not None:
        overrides["fps"] = args.fps
    if args.depth_backend is not None:
        overrides["depth_backend"] = args.depth_backend
    if args.blob_backend is not None:
        overrides["blob_backend"] = args.blob_backend
    if args.min_blob_area is not None:
        overrides["min_blob_area"] = args.min_blob_area
    if getattr(args, "segment_backend", None) is not None:
        overrides["segment_backend"] = args.segment_backend
    if getattr(args, "sam_model", None) is not None:
        overrides["sam_model"] = args.sam_model
    return PerceptionConfig.from_env(**overrides)


def _realsense_command(args) -> int:
    """The RealSense subcommands — kept apart so `perceive synthetic` never
    touches the SDK loader."""
    from .realsense import RealSenseError, list_devices, platform_hint

    config = _config_from_args(args)
    if args.cmd == "rs-info":
        try:
            from .realsense import load_api

            api = load_api(args.library)
            devices = list_devices(args.library)
            options = None
            if args.options:
                from .realsense import list_sensor_options

                options = list_sensor_options(args.library)
        except RealSenseError as exc:
            hint = platform_hint(exc)
            return _emit({"ok": False, "error": str(exc), "hint": hint or None})
        result = {"ok": True, "sdk": {"path": api.path, "api_version": api.version}, "devices": devices}
        if options is not None:
            result["options"] = options
        return _emit(result)

    if args.cmd == "gui":
        from .webapp import serve

        serve(
            camera_from_args(args, config),
            config=config,
            bind=args.bind,
            port=args.port,
            open_browser=not args.no_browser,
            robot=robot_from_args(args),
            demo=args.demo,
            views=views_from_args(args, config),
            cors=cors_from_args(args),
            pick_port=args.pick_port,
        )
        return 0

    raise SystemExit(f"unknown realsense command {args.cmd}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    for stream in (sys.stdout, sys.stderr):  # legacy Windows codepages: replace, don't crash
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    try:
        cell = apply_cell(args.cell)
    except ValueError as exc:
        print(f"--cell: {exc}", file=sys.stderr)
        return 2

    # `tools` / `cells` need no pipeline construction — print and exit.
    if args.cmd == "tools":
        print(json.dumps(get_tool_schemas(), indent=2))
        return 0
    if args.cmd == "cells":
        from .cell import load_cell

        if args.export:
            try:
                values = load_cell(args.export)
            except ValueError as exc:
                print(f"--export: {exc}", file=sys.stderr)
                return 2
            for key, value in values.items():
                if value:
                    print(f"export {key}={value}")
            print(f"export {ENV_CELL}={args.export}")
            return 0
        print(json.dumps({"active": cell, "cells": {n: load_cell(n) for n in list_cells()}}, indent=2))
        return 0
    if args.cmd == "doctor":
        from .doctor import run_doctor

        report = run_doctor(
            perceptronics_config=_config_from_args(args),
            camera=not args.no_camera,
            stream=args.stream,
            robot=not args.no_robot,
            library=args.library,
            cockpit_url=args.cockpit_url or None,
        )
        if args.as_json:
            print(json.dumps(report.as_dict(), indent=2, default=str))
        else:
            print(report.render())
        return 0 if report.ok else 1
    if args.cmd == "pick-cycle":
        return run_pick_cycle(args)
    if args.cmd == "pick-server":
        return run_pick_server(args)
    if args.cmd == "calibrate":
        return run_calibrate(args)
    if args.cmd in ("rs-info", "gui"):
        return _realsense_command(args)
    pipe = PerceptionPipeline(_config_from_args(args))

    if args.cmd == "synthetic":
        params: dict[str, int] = {}
        if args.syn_width is not None:
            params["width"] = args.syn_width
        if args.syn_height is not None:
            params["height"] = args.syn_height
        return _emit(call_tool(pipe, "perceive_synthetic", params))
    if args.cmd == "capture":
        return _emit(call_tool(pipe, "perceive_frame", {}))
    if args.cmd == "image":
        return _emit(call_tool(pipe, "perceive_image", {"path": args.path, "max_width": args.max_width}))
    if args.cmd == "call":
        try:
            params = json.loads(args.json_args)
        except json.JSONDecodeError as exc:
            print(f"--json is not valid JSON: {exc}", file=sys.stderr)
            return 2
        try:
            return _emit(call_tool(pipe, args.name, params))
        except ToolError as exc:
            print(f"tool error: {exc}", file=sys.stderr)
            return 2

    return 2


if __name__ == "__main__":
    sys.exit(main())
