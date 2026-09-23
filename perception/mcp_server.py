"""``perception-mcp`` — the keys to the cell for an agent: robot + camera tools
over MCP (stdio), stdlib only.

One server, two tool families, one process for the agent to configure:

* **Robot** — the whole ``urctl`` registry (``ur_get_state``, ``ur_move_tcp``,
  ``ur_bring_up``, ``ur_run_script``, …), driving the controller directly
  through the same safety envelope and audit log as the CLI and the cockpit.
* **Camera + cell** (``cam_*``, ``cell_*``) — served *through a running
  cockpit* (``perception gui``), because one process must own the USB camera
  and the cockpit is it. ``cam_snapshot`` writes PNGs the agent can look at;
  ``cam_segment`` / ``cam_locate`` are the click-to-segment → base-frame
  flow; ``cam_events`` is what happened; ``cell_doctor`` is the pre-flight.

    perception-mcp --cell ur20                 # stdio MCP; env from the cell profile
    perception-mcp --cell sim --dry-run        # robot calls validated + audited, not sent

Claude Code / Desktop config (``.mcp.json`` at the repo root does this)::

    {"mcpServers": {"cell": {"command": "uv", "args": ["run", "perception-mcp"]}}}

The cockpit URL is ``$PERCEPTION_COCKPIT_URL`` (default http://127.0.0.1:7621).
When no cockpit is running, ``cam_*`` tools answer with ``isError`` and the
command to start one — the robot tools keep working.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable
from dataclasses import dataclass

from urctl.config import RobotConfig
from urctl.mcp_server import McpServer
from urctl.robot import Robot
from urctl.tools import ToolError  # the one McpServer reports in-band

from .cell import ENV_CELL, apply_cell, list_cells
from .cockpitclient import DEFAULT_COCKPIT_URL, CockpitClient, CockpitError, CockpitUnavailable
from .tools import ToolError as PerceptionToolError
from .tools import _validate


@dataclass(frozen=True)
class CockpitTool:
    name: str
    description: str
    input_schema: dict
    handler: Callable[[CockpitClient, dict], dict]


def _schema(properties: dict, required: list[str] | None = None) -> dict:
    return {
        "type": "object",
        "properties": properties,
        "required": required or [],
        "additionalProperties": False,
    }


_POINT = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}
_POSE = {"type": "array", "items": {"type": "number"}, "minItems": 6, "maxItems": 6}
_BOX = {
    "type": "array",
    "items": {"type": "integer"},
    "minItems": 4,
    "maxItems": 4,
    "description": "[x0, y0, x1, y1] pixel box, x1/y1 exclusive",
}

COCKPIT_TOOLS: list[CockpitTool] = [
    CockpitTool(
        "cam_info",
        "What the cockpit's camera is doing: kind (realsense/synthetic), device serial/firmware/USB link, "
        "stream size + fps, depth filters/tuning, intrinsics, last error, the cell profile, and the robot "
        "link (host, platform, hand-eye source). Call this first.",
        _schema({}),
        lambda c, p: c.info(),
    ),
    CockpitTool(
        "cam_snapshot",
        "Write the latest RGB-D frame as viewable PNGs (<name>_color.png and a colourised <name>_depth.png, "
        "plus <name>_mask.png when a segment is active) into a directory and return their paths with the "
        "frame summary. Read the PNGs to see what the camera sees.",
        _schema({"dir": {"type": "string"}, "name": {"type": "string"}}),
        lambda c, p: c.snapshot(dir=p.get("dir"), name=p.get("name")),
    ),
    CockpitTool(
        "cam_segment",
        "Segment the object under pixel (x, y) and/or inside a box on the latest frame. Returns the mask "
        "area, and features: centroid, bbox, depth median, 3D point in the camera frame (point_m, metres), "
        "metric extent, orientation, and a grasp hint. The segment stays active for cam_locate/cam_capture.",
        _schema({"x": {"type": "integer"}, "y": {"type": "integer"}, "box": _BOX}),
        lambda c, p: _strip_mask(c.segment(x=p.get("x"), y=p.get("y"), box=p.get("box"))),
    ),
    CockpitTool(
        "cam_nearest",
        "Segment the nearest object (RealSenseTrainer's closest-thing rule). near_ratio (1, 3] widens it.",
        _schema({"near_ratio": {"type": "number"}}),
        lambda c, p: _strip_mask(c.nearest(p.get("near_ratio", 1.2))),
    ),
    CockpitTool(
        "cam_clear",
        "Drop the active segment.",
        _schema({}),
        lambda c, p: c.clear(),
    ),
    CockpitTool(
        "cam_locate",
        "Map the active segment's camera point (or an explicit point_m) into the robot base frame using the "
        "live flange pose and the hand-eye transform, and compute an approach pose standoff_m short of it "
        "along the viewing ray. Reads the robot; moves nothing. Returns every intermediate frame.",
        _schema(
            {
                "standoff_m": {"type": "number"},
                "point_m": _POINT,
                "reference": {"type": "string", "enum": ["tcp", "flange"]},
            }
        ),
        lambda c, p: c.locate(
            standoff_m=p.get("standoff_m"), point_m=p.get("point_m"), reference=p.get("reference")
        ),
    ),
    CockpitTool(
        "cam_move_to_approach",
        "movel the TCP to an absolute base-frame pose — normally the approach_pose cam_locate just returned. "
        "Safety-enveloped and audited; slow by default (0.1 m/s). Needs the robot RUNNING and, on "
        "PolyScope X or a real e-Series, Remote control mode.",
        _schema({"pose": _POSE, "velocity": {"type": "number"}, "tcp": _POSE}, required=["pose"]),
        lambda c, p: c.move(p["pose"], velocity=p.get("velocity"), tcp=p.get("tcp")),
    ),
    CockpitTool(
        "cam_approach_cycle",
        "The test loop: over the current segment at clearance_m, down to the standoff (cell default; "
        "reference tcp|flange), hold hold_s, back up, back to the capture pose — one safety-validated "
        "program. Refused when out of reach. Segment first (cam_segment / cam_nearest).",
        _schema(
            {
                "standoff_m": {"type": "number"},
                "reference": {"type": "string", "enum": ["tcp", "flange"]},
                "clearance_m": {"type": "number"},
                "hold_s": {"type": "number"},
                "velocity": {"type": "number"},
            }
        ),
        lambda c, p: c.approach_cycle(
            standoff_m=p.get("standoff_m"),
            reference=p.get("reference"),
            clearance_m=p.get("clearance_m"),
            hold_s=p.get("hold_s"),
            velocity=p.get("velocity"),
        ),
    ),
    CockpitTool(
        "cam_capture",
        "Save the current frame (and the active mask + features) into the capture dataset "
        "(captures/<name>/color_NNNNN.png, depth_NNNNN.png, mask_NNNNN.png, meta_NNNNN.json).",
        _schema({"name": {"type": "string"}, "include_mask": {"type": "boolean"}}),
        lambda c, p: c.capture(p.get("name", "object"), p.get("include_mask", True)),
    ),
    CockpitTool(
        "cam_events",
        "The cockpit's event log after sequence number `after`: robot actions (jog/move/bring-up with their "
        "outcome and safety verdicts), segments, captures, camera errors. Read it to see what a human just "
        "did in the cockpit, or to confirm what you did.",
        _schema({"after": {"type": "integer"}, "limit": {"type": "integer"}}),
        lambda c, p: c.events(p.get("after", 0), p.get("limit", 200)),
    ),
    CockpitTool(
        "cell_doctor",
        "Pre-flight checklist for the cell: camera stream health, robot reachability (orchestration/primary/"
        "RTDE ports, platform mismatch), robot/safety/control mode, telemetry, TCP offset and hand-eye "
        "calibration status. Each failed check carries a fix. `robot=false` skips the controller.",
        _schema({"robot": {"type": "boolean"}}),
        lambda c, p: c.doctor(robot=p.get("robot", True)),
    ),
    CockpitTool(
        "cal_status",
        "Hand-eye calibration session: the touched mark (base frame), the recorded views, the solve result "
        "if any, and the hand-eye transform currently active (bracket seed / file / env / calibrated).",
        _schema({}),
        lambda c, p: c.get("/api/cal"),
    ),
    CockpitTool(
        "cal_record_mark",
        "Step 1 of touch-and-click calibration: with the tool tip resting ON the mark, record the live TCP "
        "position as the mark's base-frame position. (Freedrive first with ur_freedrive.)",
        _schema({}),
        lambda c, p: c.post("/api/cal/mark"),
    ),
    CockpitTool(
        "cal_add_view",
        "Step 2: from a new pose where the camera sees the mark, add a view: the camera-frame point under "
        "pixel (x, y) (5x5 median of valid depth) plus the live flange pose. Use cam_snapshot to find the "
        "mark's pixel. Vary wrist rotation and tilt between views; 4-6 views is typical.",
        _schema({"x": {"type": "integer"}, "y": {"type": "integer"}}, required=["x", "y"]),
        lambda c, p: c.post("/api/cal/view", {"x": p["x"], "y": p["y"]}),
    ),
    CockpitTool(
        "cal_solve",
        "Step 3: solve T_flange_camera from the mark + views (Levenberg-Marquardt, seeded from the bracket). "
        "Returns the pose, RMS residual, per-view residuals, warnings (low rotation diversity, high RMS), "
        "and the PERCEPTION_T_FLANGE_CAMERA env line. Nothing is applied yet.",
        _schema({}),
        lambda c, p: c.post("/api/cal/solve"),
    ),
    CockpitTool(
        "cal_apply",
        "Use the solved transform from now on and (save=true) write captures/calibration/handeye_<cell>.json "
        "so later starts load it (precedence: PERCEPTION_T_FLANGE_CAMERA env > file > bracket seed). "
        "Refused while the solve has warnings unless force=true.",
        _schema({"save": {"type": "boolean"}, "force": {"type": "boolean"}}),
        lambda c, p: c.post("/api/cal/apply", {"save": p.get("save", True), "force": p.get("force", False)}),
    ),
    CockpitTool(
        "cal_reset",
        "Drop the mark and all views.",
        _schema({}),
        lambda c, p: c.post("/api/cal/reset"),
    ),
    CockpitTool(
        "cell_jog",
        "One relative base-frame nudge of the TCP: delta [dx, dy, dz, drx, dry, drz] (metres, radians), "
        "at most 0.05 m / 0.35 rad per axis, safety-enveloped, logged in the cockpit's events. "
        "For anything bigger use ur_move_tcp / ur_move_trajectory.",
        _schema({"delta": _POSE, "velocity": {"type": "number"}}, required=["delta"]),
        lambda c, p: c.jog(p["delta"], velocity=p.get("velocity")),
    ),
]

_BY_NAME = {t.name: t for t in COCKPIT_TOOLS}


def _strip_mask(result: dict) -> dict:
    """The base64 mask PNG is for the browser; an agent gets the features."""
    out = dict(result)
    out.pop("mask_png_b64", None)
    return out


class CockpitTools:
    """A tool provider (see :class:`urctl.mcp_server.McpServer`) over a cockpit."""

    def __init__(self, client: CockpitClient | None = None):
        self.client = client or CockpitClient()

    def schemas(self) -> list[dict]:
        return [
            {"name": t.name, "description": t.description, "input_schema": t.input_schema}
            for t in COCKPIT_TOOLS
        ]

    def owns(self, name: str) -> bool:
        return name in _BY_NAME

    def call(self, name: str, params: dict | None = None) -> dict:
        tool = _BY_NAME.get(name)
        if tool is None:
            raise ToolError(f"unknown tool: {name!r}. Known: {sorted(_BY_NAME)}")
        params = params or {}
        try:
            _validate(params, tool.input_schema)
        except PerceptionToolError as exc:
            raise ToolError(str(exc)) from None
        try:
            return tool.handler(self.client, params)
        except CockpitUnavailable as exc:
            return {"ok": False, "error": str(exc), "cockpit": self.client.url}
        except CockpitError as exc:
            return {"ok": False, "error": str(exc)}


def build_server(robot: Robot, *, cockpit_url: str | None = None, name: str = "cell") -> McpServer:
    return McpServer(robot, extra=[CockpitTools(CockpitClient(cockpit_url))], name=name)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="perception-mcp", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--cell",
        default=None,
        help=f"cell profile: {'|'.join(list_cells())} or a .env path (default: ${ENV_CELL})",
    )
    ap.add_argument("--host", default=None, help="controller host/IP (default: $UR_HOST or the cell's)")
    ap.add_argument(
        "--cockpit-url",
        default=None,
        help=f"running cockpit (default: $PERCEPTION_COCKPIT_URL or {DEFAULT_COCKPIT_URL})",
    )
    ap.add_argument("--dry-run", action="store_true", help="validate + audit robot tool calls, send nothing")
    args = ap.parse_args(argv)
    try:
        cell = apply_cell(args.cell)
    except ValueError as exc:
        print(f"--cell: {exc}", file=sys.stderr)
        return 2
    config = RobotConfig.from_env(host=args.host)
    robot = Robot(config, dry_run=args.dry_run)
    server = build_server(robot, cockpit_url=args.cockpit_url)
    print(
        f"perception-mcp: cell {cell.get('cell') or '-'}, robot {config.platform} {config.host}"
        f"{' (dry-run)' if args.dry_run else ''}, cockpit {server.extra[0].client.url}; "
        "serving MCP over stdio",
        file=sys.stderr,
    )
    try:
        server.serve_stdio()
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
