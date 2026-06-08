"""perceive command-line interface — a human/scripting entry point.

    perceive synthetic                     # run the pipeline on a synthetic frame
    perceive capture                       # grab one webcam frame + detect blobs
    perceive --depth-backend depth_anything capture
    perceive tools                         # dump the agent tool schemas (JSON)
    perceive call perceive_synthetic --json '{"width":320,"height":240}'

Camera + backend selection come from ``--device`` / ``--width`` / ``--height``
/ ``--fps`` / ``--depth-backend`` / ``--blob-backend`` or the matching
``PERCEPTION_*`` env vars, so the same commands run against a laptop webcam, a
camera-less CI box (``synthetic``), or a GPU host (the real depth model).

Every command prints its structured result as JSON and exits non-zero if the
result reported ``ok=false``.
"""

from __future__ import annotations

import argparse
import json
import sys

from .config import PerceptionConfig
from .pipeline import PerceptionPipeline
from .tools import ToolError, call_tool, get_tool_schemas


def _emit(result: dict) -> int:
    print(json.dumps(result, indent=2, default=str))
    return 0 if result.get("ok", True) else 1


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="perceive", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--device", type=int, default=None, help="camera index (default: $PERCEPTION_DEVICE or 0)"
    )
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument("--fps", type=int, default=None)
    ap.add_argument(
        "--depth-backend",
        default=None,
        help="stub | depth_anything (default: $PERCEPTION_DEPTH_BACKEND or stub)",
    )
    ap.add_argument(
        "--blob-backend",
        default=None,
        help="stub | blob_cv (default: $PERCEPTION_BLOB_BACKEND or stub)",
    )
    ap.add_argument("--min-blob-area", type=int, default=None, help="drop blobs smaller than this (px)")
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

    ct = sub.add_parser("call", help="dispatch a perception tool by name")
    ct.add_argument("name")
    ct.add_argument("--json", dest="json_args", default="{}", help="tool args as a JSON object")

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
    return PerceptionConfig.from_env(**overrides)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    # `tools` needs no pipeline construction — print and exit.
    if args.cmd == "tools":
        print(json.dumps(get_tool_schemas(), indent=2))
        return 0

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
