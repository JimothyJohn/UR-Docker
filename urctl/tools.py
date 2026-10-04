"""Agent tool registry — the framework-neutral "openclaw friendly" surface.

This is what lets OpenClaw/ROSClaw (or any function-calling agent, or the MCP
server in :mod:`urctl.mcp_server`) drive the robot:

  * **Capability discovery** — :func:`get_tool_schemas` returns each capability
    as a name + description + JSON Schema, ready to drop into a model's
    tool-calling API ("standardized affordance injection").
  * **Pre-execution validation** — :func:`call_tool` checks arguments against
    the schema, then the handler runs them through the robot's safety envelope.
  * **Structured audit logging** — every call goes through :class:`Robot`, so
    it lands in the audit log with its arguments, result, and safety verdict.

The registry is plain data (no SDK), so the same tool definitions feed Claude's
API, an OpenAI-style ``tools=`` array, an MCP server, or a bespoke executive::

    from urctl import Robot
    from urctl.tools import get_tool_schemas, call_tool

    robot = Robot()
    schemas = get_tool_schemas()                  # hand these to the model
    result = call_tool(robot, "move_joints",      # dispatch a model's tool call
                       {"joints": [0, -1.57, 0, -1.57, 0, 0]})

Tool names are vendor-neutral (``move_tcp``, ``get_state`` …) since 2026-09-26; the
original ``ur_``-prefixed names are accepted as aliases (:data:`TOOL_ALIASES`) so
agents, docs and MCP configurations written against them keep working.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from . import workcell
from .robot import GRIPPER_ACTIONS, Robot

# A reusable schema fragment for a 6-element joint vector (radians).
_JOINTS_SCHEMA = {
    "type": "array",
    "items": {"type": "number"},
    "minItems": 6,
    "maxItems": 6,
    "description": "Six joint angles in radians: [base, shoulder, elbow, wrist1, wrist2, wrist3].",
}
_VELOCITY_SCHEMA = {
    "type": "number",
    "exclusiveMinimum": 0,
    "description": "Joint speed in rad/s (capped by the safety envelope).",
}
_ACCELERATION_SCHEMA = {
    "type": "number",
    "exclusiveMinimum": 0,
    "description": "Joint acceleration in rad/s^2 (capped by the safety envelope).",
}
_POSE_SCHEMA = {
    "type": "array",
    "items": {"type": "number"},
    "minItems": 6,
    "maxItems": 6,
    "description": (
        "TCP pose [x, y, z, rx, ry, rz] in metres + rotation-vector radians. "
        "When relative=true it is a base-frame delta added to the current pose."
    ),
}
_TCP_VELOCITY_SCHEMA = {
    "type": "number",
    "exclusiveMinimum": 0,
    "description": "Linear TCP speed in m/s (capped by the safety envelope).",
}
_TCP_ACCELERATION_SCHEMA = {
    "type": "number",
    "exclusiveMinimum": 0,
    "description": "Linear TCP acceleration in m/s^2 (capped by the safety envelope).",
}


@dataclass(frozen=True)
class Tool:
    name: str
    description: str
    input_schema: dict
    handler: Callable[[Robot, dict], dict]


def _object_schema(properties: dict, required: list[str] | None = None) -> dict:
    return {
        "type": "object",
        "properties": properties,
        "required": required or [],
        "additionalProperties": False,
    }


# ----- handlers --------------------------------------------------------------
# Each handler receives the Robot and the (already schema-validated) params and
# returns the Robot method's structured result dict.


def _h_get_state(robot: Robot, p: dict) -> dict:
    return robot.get_state()


def _h_bring_up(robot: Robot, p: dict) -> dict:
    return robot.bring_up()


def _h_power_off(robot: Robot, p: dict) -> dict:
    return robot.power_off()


def _h_move_joints(robot: Robot, p: dict) -> dict:
    kwargs = {}
    if "velocity" in p:
        kwargs["velocity"] = p["velocity"]
    if "acceleration" in p:
        kwargs["acceleration"] = p["acceleration"]
    return robot.move_joints(p["joints"], **kwargs)


def _h_flange_pose(robot: Robot, p: dict) -> dict:
    return robot.get_flange_pose()


def _h_move_tcp(robot: Robot, p: dict) -> dict:
    kwargs = {}
    if "relative" in p:
        kwargs["relative"] = p["relative"]
    if "velocity" in p:
        kwargs["velocity"] = p["velocity"]
    if "acceleration" in p:
        kwargs["acceleration"] = p["acceleration"]
    if p.get("tcp") is not None:
        kwargs["tcp"] = p["tcp"]
    return robot.move_tcp(p["pose"], **kwargs)


def _h_move_tcp_path(robot: Robot, p: dict) -> dict:
    kwargs = {}
    if p.get("tcp") is not None:
        kwargs["tcp"] = p["tcp"]
    if "timeout" in p:
        kwargs["timeout"] = p["timeout"]
    if p.get("gripper_first") is not None:
        kwargs["gripper_first"] = p["gripper_first"]
    return robot.move_tcp_path(p["legs"], **kwargs)


def _h_move_home(robot: Robot, p: dict) -> dict:
    kwargs = {}
    if "velocity" in p:
        kwargs["velocity"] = p["velocity"]
    if "acceleration" in p:
        kwargs["acceleration"] = p["acceleration"]
    return robot.move_home(**kwargs)


def _h_move_trajectory(robot: Robot, p: dict) -> dict:
    kwargs = {}
    for k in ("velocity", "acceleration", "blend_radius"):
        if k in p:
            kwargs[k] = p[k]
    return robot.move_trajectory(p["waypoints"], **kwargs)


def _h_freedrive(robot: Robot, p: dict) -> dict:
    kwargs = {}
    if p.get("hold_s") is not None:
        kwargs["hold_s"] = float(p["hold_s"])
    return robot.freedrive(p["enable"], **kwargs)


def _h_gripper(robot: Robot, p: dict) -> dict:
    kwargs = {k: p[k] for k in ("position", "speed", "force", "timeout_s") if p.get(k) is not None}
    return robot.gripper(p["action"], **kwargs)


def _h_popup(robot: Robot, p: dict) -> dict:
    return robot.popup(p["text"])


def _h_load_program(robot: Robot, p: dict) -> dict:
    return robot.load_program(p["name"])


def _h_play(robot: Robot, p: dict) -> dict:
    return robot.play()


def _h_stop(robot: Robot, p: dict) -> dict:
    return robot.stop()


def _h_pause(robot: Robot, p: dict) -> dict:
    return robot.pause()


def _h_run_script(robot: Robot, p: dict) -> dict:
    return robot.run_script(
        p["script"],
        wrap=p.get("wrap", True),
        capture=p.get("capture", False),
        marker=p.get("marker", ""),
    )


def _h_dashboard_command(robot: Robot, p: dict) -> dict:
    return robot.dashboard_command(p["command"])


def _h_rtde_state(robot: Robot, p: dict) -> dict:
    return robot.rtde_state(deep=p.get("deep", False))


def _sys_inspector(robot: Robot, p: dict):
    from .sysinfo import SystemInspector, runner_for

    access = "ssh" if p.get("ssh") else "docker" if p.get("container") else None
    return SystemInspector(runner_for(robot.config, access=access, target=p.get("ssh") or p.get("container")))


def _h_system_snapshot(robot: Robot, p: dict) -> dict:
    result: dict = {"ok": True, "host": robot.config.host}
    if not p.get("offline", False):
        result["live"] = robot.get_state()
        result["telemetry"] = robot.rtde_state(deep=True)
    result["system"] = _sys_inspector(robot, p).snapshot(installation_name=p.get("installation", "default"))
    return result


def _h_list_programs(robot: Robot, p: dict) -> dict:
    return {"ok": True, **_sys_inspector(robot, p).programs()}


# Shared schema fragments for the filesystem-access tools.
_SSH_SCHEMA = {
    "type": "string",
    "description": "Reach the controller filesystem over SSH as [user@]host (real robot; "
    "default user root, default host = the robot host). Mutually exclusive with 'container'.",
}
_CONTAINER_SCHEMA = {
    "type": "string",
    "description": "Reach a URSim container's filesystem via docker exec (simulator). "
    "When neither ssh nor container is given, a loopback host implies the default "
    "URSim container and a remote host implies SSH.",
}


def _h_set_speed_override(robot: Robot, p: dict) -> dict:
    return robot.set_speed_override(p["fraction"])


def _h_set_digital_output(robot: Robot, p: dict) -> dict:
    return robot.set_digital_output(p["pin"], p["value"])


def _h_tcp_offset(robot: Robot, p: dict) -> dict:
    return workcell.tcp_offset(robot, p["action"], name=p.get("name"), offset=p.get("offset"))


def _h_position(robot: Robot, p: dict) -> dict:
    move = {k: p[k] for k in ("velocity", "acceleration") if k in p}
    return workcell.position(robot, p["action"], name=p.get("name"), **move)


def _h_workplane(robot: Robot, p: dict) -> dict:
    return workcell.workplane(
        robot, p["action"], name=p.get("name"), index=p.get("index"), points=p.get("points")
    )


_NAME_SCHEMA = {
    "type": "string",
    "description": "1-64 of A-Z a-z 0-9 _ . - (starting alphanumeric).",
}
_POINT_SCHEMA = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}


# ----- registry --------------------------------------------------------------

TOOLS: list[Tool] = [
    Tool(
        "get_state",
        "Read the robot's current state: robot mode, safety mode, program "
        "state, and (when RUNNING) joint angles and TCP pose.",
        _object_schema({}),
        _h_get_state,
    ),
    Tool(
        "flange_pose",
        "Read the tool-flange pose in the base frame (plus the active TCP pose "
        "and TCP offset it derives from). Use it to transform measurements from "
        "a flange-mounted camera into base coordinates. Read-only; works in "
        "Local mode.",
        _object_schema({}),
        _h_flange_pose,
    ),
    Tool(
        "bring_up",
        "Bring the controller from a cold start to RUNNING: clear latched "
        "safety, power on motors, release brakes. Idempotent.",
        _object_schema({}),
        _h_bring_up,
    ),
    Tool("power_off", "Power off the robot's motors.", _object_schema({}), _h_power_off),
    Tool(
        "move_joints",
        "Move to a joint-space target with movej. Validated against the "
        "safety envelope (joint range, speed, acceleration, RUNNING state) "
        "before execution.",
        _object_schema(
            {
                "joints": _JOINTS_SCHEMA,
                "velocity": _VELOCITY_SCHEMA,
                "acceleration": _ACCELERATION_SCHEMA,
            },
            required=["joints"],
        ),
        _h_move_joints,
    ),
    Tool(
        "move_tcp",
        "Move the tool linearly with movel. Set relative=true for a base-frame "
        "delta added to the current pose (e.g. pose [0,0.05,0,0,0,0] nudges "
        "+50 mm along base +Y); relative=false for an absolute base-frame "
        "target. Validated against the safety envelope (reach/step, TCP "
        "speed/accel in m/s, RUNNING state) before execution.",
        _object_schema(
            {
                "pose": _POSE_SCHEMA,
                "relative": {
                    "type": "boolean",
                    "description": "Treat pose as a base-frame delta from the current TCP pose.",
                },
                "velocity": _TCP_VELOCITY_SCHEMA,
                "acceleration": _TCP_ACCELERATION_SCHEMA,
                "tcp": {
                    **_POSE_SCHEMA,
                    "description": "Override the active TCP for this move (set_tcp in the same program); "
                    "[0,0,0,0,0,0] makes pose the tool-flange target.",
                },
            },
            required=["pose"],
        ),
        _h_move_tcp,
    ),
    Tool(
        "move_tcp_path",
        "Run several absolute movel legs as ONE program on one connection (an "
        "approach cycle: over → down → dwell → up → back). Each leg: pose "
        "[x,y,z,rx,ry,rz], optional velocity/acceleration (m/s, m/s^2) and dwell_s "
        "to pause after landing. All legs are safety-validated before anything is "
        "sent; tcp overrides the active TCP for the whole path ([0]*6 = flange).",
        _object_schema(
            {
                "legs": {
                    "type": "array",
                    "minItems": 1,
                    "maxItems": 32,
                    "items": _object_schema(
                        {
                            "pose": _POSE_SCHEMA,
                            "velocity": _TCP_VELOCITY_SCHEMA,
                            "acceleration": _TCP_ACCELERATION_SCHEMA,
                            "dwell_s": {"type": "number", "minimum": 0, "maximum": 60},
                            "blend_m": {"type": "number", "minimum": 0, "maximum": 0.1},
                            "gripper": {"type": "string", "enum": ["open", "close"]},
                        },
                        required=["pose"],
                    ),
                },
                "tcp": _POSE_SCHEMA,
                "timeout": {"type": "number", "minimum": 1, "maximum": 600},
                "gripper_first": {"type": "integer", "minimum": 0, "maximum": 255},
            },
            required=["legs"],
        ),
        _h_move_tcp_path,
    ),
    Tool(
        "move_trajectory",
        "Run a joint-space trajectory: a sequence of movej waypoints streamed as "
        "ONE program over a single Primary connection. This is the way to move a "
        "lot quickly — issuing many separate moves pays a per-move handshake and "
        "can wedge the controller after ~10 rapid reconnects. Every waypoint is "
        "validated against the safety envelope before anything is sent. Optional "
        "blend_radius (m) smooths consecutive segments; it is dropped on the "
        "final waypoint so the robot stops on target.",
        _object_schema(
            {
                "waypoints": {
                    "type": "array",
                    "items": _JOINTS_SCHEMA,
                    "minItems": 1,
                    "description": "Ordered list of 6-joint targets (radians) to move through.",
                },
                "velocity": _VELOCITY_SCHEMA,
                "acceleration": _ACCELERATION_SCHEMA,
                "blend_radius": {
                    "type": "number",
                    "minimum": 0,
                    "description": "Segment blend radius in metres (0 = stop at each waypoint).",
                },
            },
            required=["waypoints"],
        ),
        _h_move_trajectory,
    ),
    Tool(
        "move_home",
        "Move to the safe candle home pose (straight up, wrists folded).",
        _object_schema(
            {
                "velocity": _VELOCITY_SCHEMA,
                "acceleration": _ACCELERATION_SCHEMA,
            }
        ),
        _h_move_home,
    ),
    Tool(
        "freedrive",
        "Enable or disable freedrive (hand-guiding) on all six axes. Enabling holds "
        "freedrive for hold_s seconds (default 600) or until disabled — a real "
        "e-Series drops freedrive the moment its script ends, so the hold is a program "
        "that stays running.",
        _object_schema(
            {
                "enable": {"type": "boolean"},
                "hold_s": {"type": "number", "minimum": 1, "maximum": 3600},
            },
            required=["enable"],
        ),
        _h_freedrive,
    ),
    Tool(
        "gripper",
        "Drive the Robotiq gripper (Hand-E / 2F) on the tool flange through its URCap "
        "daemon: status, open, close, move (position 0 = fully open … 255 = fully "
        "closed), or activate. Waits for the motion to settle and returns the status "
        "readback — object_detected is True when the fingers stopped on something.",
        _object_schema(
            {
                "action": {"type": "string", "enum": list(GRIPPER_ACTIONS)},
                "position": {"type": "integer", "minimum": 0, "maximum": 255},
                "speed": {"type": "integer", "minimum": 0, "maximum": 255},
                "force": {"type": "integer", "minimum": 0, "maximum": 255},
                "timeout_s": {"type": "number", "minimum": 0.5, "maximum": 30},
            },
            required=["action"],
        ),
        _h_gripper,
    ),
    Tool(
        "popup",
        "Show a popup message on the PolyScope teach pendant.",
        _object_schema({"text": {"type": "string"}}, required=["text"]),
        _h_popup,
    ),
    Tool(
        "load_program",
        "Load a PolyScope program (<name>.urp) on the controller. A matching "
        "<name>.installation file must already sit beside it on the controller.",
        _object_schema({"name": {"type": "string"}}, required=["name"]),
        _h_load_program,
    ),
    Tool("play", "Play (start) the currently loaded program.", _object_schema({}), _h_play),
    Tool("stop", "Stop the running program.", _object_schema({}), _h_stop),
    Tool("pause", "Pause the running program.", _object_schema({}), _h_pause),
    Tool(
        "run_script",
        "ADVANCED: run arbitrary URScript on the Primary client. Bypasses the "
        "joint/speed safety envelope (it cannot be statically bounded) but is "
        "still audited. Wrapped in a def by default so motion actually runs. "
        "Prefer ur_move_joints / ur_move_tcp for validated motion.",
        _object_schema(
            {
                "script": {"type": "string", "description": "URScript source to execute."},
                "wrap": {
                    "type": "boolean",
                    "description": (
                        "Wrap the script in a single def so it runs as one program "
                        "(default true). Required for bare top-level motion to "
                        "execute; set false only for scripts that define their own "
                        "top-level functions."
                    ),
                },
                "capture": {
                    "type": "boolean",
                    "description": "Capture textmsg output from the broadcast.",
                },
                "marker": {
                    "type": "string",
                    "description": "Only return captured lines containing this marker.",
                },
            },
            required=["script"],
        ),
        _h_run_script,
    ),
    Tool(
        "dashboard_command",
        "ADVANCED: send a raw Dashboard command (e.g. 'robotmode'). Audited; "
        "unvalidated. Escape hatch for commands not covered by other tools.",
        _object_schema({"command": {"type": "string"}}, required=["command"]),
        _h_dashboard_command,
    ),
    Tool(
        "rtde_state",
        "Read high-rate structured state via RTDE (port 30004): joint angles "
        "and velocities, TCP pose/speed/force, and decoded safety/runtime "
        "status. Works even when no program is running (unlike ur_get_state's "
        "joint/TCP fields). Set deep=true for full diagnostics: per-joint "
        "currents/temperatures/voltages/drive modes, supply power, tool "
        "telemetry, analog IO, speed scaling. Returns ok=false if RTDE is "
        "disabled or unreachable.",
        _object_schema(
            {
                "deep": {
                    "type": "boolean",
                    "description": "Subscribe the full diagnostic recipe (default false).",
                }
            }
        ),
        _h_rtde_state,
    ),
    Tool(
        "system_snapshot",
        "Full cell model of the controller: live state + deep RTDE telemetry "
        "plus filesystem-level facts the network APIs never expose — joint "
        "serials/firmware (replacement detection), kinematic-calibration "
        "mismatch, storage health, program inventory, parsed active "
        "installation (active TCP + whether it is at the flange, payload, "
        "named IO, safety limits), and installed URCaps. THE place to start "
        "when meeting a robot or planning a program for it: flange_tcp=false "
        "means author motion as script nodes, not native MoveJ nodes.",
        _object_schema(
            {
                "ssh": _SSH_SCHEMA,
                "container": _CONTAINER_SCHEMA,
                "installation": {
                    "type": "string",
                    "description": "Installation name to parse (default 'default').",
                },
                "offline": {
                    "type": "boolean",
                    "description": "Skip the live network reads (robot powered off).",
                },
            }
        ),
        _h_system_snapshot,
    ),
    Tool(
        "list_programs",
        "List the .urp / .script / .installation files on the controller "
        "(name, size, mtime) — what the operator can load from the pendant.",
        _object_schema({"ssh": _SSH_SCHEMA, "container": _CONTAINER_SCHEMA}),
        _h_list_programs,
    ),
    Tool(
        "set_speed_override",
        "Set the global speed slider (0-1) over RTDE. Scales the speed of ALL "
        "subsequent motion. Validated against the safety envelope's max speed "
        "fraction before it is applied.",
        _object_schema(
            {
                "fraction": {
                    "type": "number",
                    "exclusiveMinimum": 0,
                    "maximum": 1,
                    "description": "Speed scale 0-1 (1.0 = full programmed speed).",
                }
            },
            required=["fraction"],
        ),
        _h_set_speed_override,
    ),
    Tool(
        "set_digital_output",
        "Set a standard digital output pin (0-7) high or low over RTDE.",
        _object_schema(
            {
                "pin": {"type": "integer", "minimum": 0, "maximum": 7},
                "value": {"type": "boolean"},
            },
            required=["pin", "value"],
        ),
        _h_set_digital_output,
    ),
    Tool(
        "tcp_offset",
        "Read or write the tool centre point, on any arm (UR e-Series, PolyScope X, "
        "UFACTORY). get: the controller's active TCP offset [x, y, z, rx, ry, rz] (metres + "
        "rotation vector, flange frame). set: make `offset` the active TCP (read back). "
        "save: store `offset` (or the active one) under `name` in the cell store. use: "
        "make a saved TCP active. list / delete: the saved TCPs.",
        _object_schema(
            {
                "action": {"type": "string", "enum": list(workcell.TCP_ACTIONS)},
                "name": _NAME_SCHEMA,
                "offset": {"type": "array", "items": {"type": "number"}, "minItems": 6, "maxItems": 6},
            },
            required=["action"],
        ),
        _h_tcp_offset,
    ),
    Tool(
        "position",
        "Named robot positions in the cell store, on any arm. save: the current joints + "
        "TCP pose (and the TCP offset) under `name`. get / list / delete. move_to: a joint "
        "move to the saved joints (validated by the safety envelope).",
        _object_schema(
            {
                "action": {"type": "string", "enum": list(workcell.POSITION_ACTIONS)},
                "name": _NAME_SCHEMA,
                "velocity": _VELOCITY_SCHEMA,
                "acceleration": _ACCELERATION_SCHEMA,
            },
            required=["action"],
        ),
        _h_position,
    ),
    Tool(
        "workplane",
        "Workplanes (the table) from three touched points, on any arm. touch: record the "
        "live TCP position as point `index` — 1 the corner, 2 along +X, 3 on the far side; "
        "touch the table with the tool's point, and the third touch computes and saves the "
        "plane. define: three base-frame [x, y, z] `points`. get / list / delete. use_tcp: "
        "re-activate the TCP the plane was touched with. Answers carry tilt_deg (off base "
        "XY) and table_z.",
        _object_schema(
            {
                "action": {"type": "string", "enum": list(workcell.WORKPLANE_ACTIONS)},
                "name": _NAME_SCHEMA,
                "index": {"type": "integer", "minimum": 1, "maximum": 3},
                "points": {"type": "array", "items": _POINT_SCHEMA, "minItems": 3, "maxItems": 3},
            },
            required=["action"],
        ),
        _h_workplane,
    ),
]

ALIAS_PREFIX = "ur_"
# The pre-2026-09-26 names: ``ur_move_tcp`` → ``move_tcp``. Accepted everywhere a
# tool name is (call_tool, the MCP servers, the CLI); listed only on request.
TOOL_ALIASES: dict[str, str] = {ALIAS_PREFIX + t.name: t.name for t in TOOLS}
_BY_NAME = {t.name: t for t in TOOLS}


def canonical_name(name: str) -> str:
    """The registry name for ``name`` (an alias or a canonical name, unchanged otherwise)."""
    return TOOL_ALIASES.get(name, name)


def get_tool_schemas(*, include_aliases: bool = False) -> list[dict]:
    """Return the tools as ``{name, description, input_schema}`` dicts.

    This is the capability-discovery payload — hand it straight to a model's
    tool-calling API (rename ``input_schema`` to ``parameters`` for the
    OpenAI-style shape; it's already the right shape for Anthropic's API).
    ``include_aliases`` appends the ``ur_``-prefixed names as separate entries
    for a client that was configured against them.
    """
    out = [{"name": t.name, "description": t.description, "input_schema": t.input_schema} for t in TOOLS]
    if include_aliases:
        out += [
            {
                "name": alias,
                "description": f"Alias of {t.name}. " + t.description,
                "input_schema": t.input_schema,
            }
            for alias, t in ((a, _BY_NAME[c]) for a, c in TOOL_ALIASES.items())
        ]
    return out


class ToolError(Exception):
    """Raised when a tool call is malformed (unknown tool / invalid args)."""


def _validate(params: dict, schema: dict) -> None:
    """Minimal, dependency-free JSON-Schema check: unknown tool args, required
    fields, and primitive types. The safety envelope does the semantic checks.
    """
    props = schema.get("properties", {})
    if schema.get("additionalProperties") is False:
        unknown = set(params) - set(props)
        if unknown:
            raise ToolError(f"unexpected argument(s): {sorted(unknown)}")
    for key in schema.get("required", []):
        if key not in params:
            raise ToolError(f"missing required argument: {key!r}")
    type_map = {
        "string": str,
        "number": (int, float),
        "integer": int,
        "boolean": bool,
        "array": list,
        "object": dict,
    }
    for key, value in params.items():
        spec = props.get(key)
        if not spec or "type" not in spec:
            continue
        expected = type_map.get(spec["type"])
        # bool is an int subclass — reject it where a number is expected.
        if spec["type"] == "number" and isinstance(value, bool):
            raise ToolError(f"argument {key!r} must be a number, got bool")
        if expected and not isinstance(value, expected):
            raise ToolError(f"argument {key!r} must be {spec['type']}, got {type(value).__name__}")


def call_tool(robot: Robot, name: str, params: dict | None = None) -> dict:
    """Validate and dispatch a tool call against ``robot``.

    Returns the handler's structured result dict (which always carries
    ``ok`` and was audited). Raises :class:`ToolError` for an unknown tool or
    malformed arguments.
    """
    tool = _BY_NAME.get(canonical_name(name))
    if tool is None:
        raise ToolError(
            f"unknown tool: {name!r}. Known: {sorted(_BY_NAME)} (or with the {ALIAS_PREFIX!r} prefix)"
        )
    params = params or {}
    _validate(params, tool.input_schema)
    return tool.handler(robot, params)
