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
    result = call_tool(robot, "ur_move_joints",   # dispatch a model's tool call
                       {"joints": [0, -1.57, 0, -1.57, 0, 0]})
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from .robot import Robot

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


def _h_move_tcp(robot: Robot, p: dict) -> dict:
    kwargs = {}
    if "relative" in p:
        kwargs["relative"] = p["relative"]
    if "velocity" in p:
        kwargs["velocity"] = p["velocity"]
    if "acceleration" in p:
        kwargs["acceleration"] = p["acceleration"]
    return robot.move_tcp(p["pose"], **kwargs)


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
    return robot.freedrive(p["enable"])


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
    return robot.rtde_state()


def _h_set_speed_override(robot: Robot, p: dict) -> dict:
    return robot.set_speed_override(p["fraction"])


def _h_set_digital_output(robot: Robot, p: dict) -> dict:
    return robot.set_digital_output(p["pin"], p["value"])


# ----- registry --------------------------------------------------------------

TOOLS: list[Tool] = [
    Tool(
        "ur_get_state",
        "Read the robot's current state: robot mode, safety mode, program "
        "state, and (when RUNNING) joint angles and TCP pose.",
        _object_schema({}),
        _h_get_state,
    ),
    Tool(
        "ur_bring_up",
        "Bring the controller from a cold start to RUNNING: clear latched "
        "safety, power on motors, release brakes. Idempotent.",
        _object_schema({}),
        _h_bring_up,
    ),
    Tool("ur_power_off", "Power off the robot's motors.", _object_schema({}), _h_power_off),
    Tool(
        "ur_move_joints",
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
        "ur_move_tcp",
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
            },
            required=["pose"],
        ),
        _h_move_tcp,
    ),
    Tool(
        "ur_move_trajectory",
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
        "ur_move_home",
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
        "ur_freedrive",
        "Enable or disable freedrive (hand-guiding) on all six axes.",
        _object_schema({"enable": {"type": "boolean"}}, required=["enable"]),
        _h_freedrive,
    ),
    Tool(
        "ur_popup",
        "Show a popup message on the PolyScope teach pendant.",
        _object_schema({"text": {"type": "string"}}, required=["text"]),
        _h_popup,
    ),
    Tool(
        "ur_load_program",
        "Load a PolyScope program (<name>.urp) on the controller. A matching "
        "<name>.installation file must already sit beside it on the controller.",
        _object_schema({"name": {"type": "string"}}, required=["name"]),
        _h_load_program,
    ),
    Tool("ur_play", "Play (start) the currently loaded program.", _object_schema({}), _h_play),
    Tool("ur_stop", "Stop the running program.", _object_schema({}), _h_stop),
    Tool("ur_pause", "Pause the running program.", _object_schema({}), _h_pause),
    Tool(
        "ur_run_script",
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
        "ur_dashboard_command",
        "ADVANCED: send a raw Dashboard command (e.g. 'robotmode'). Audited; "
        "unvalidated. Escape hatch for commands not covered by other tools.",
        _object_schema({"command": {"type": "string"}}, required=["command"]),
        _h_dashboard_command,
    ),
    Tool(
        "ur_rtde_state",
        "Read high-rate structured state via RTDE (port 30004): joint angles "
        "and velocities, TCP pose/speed/force, and raw safety/runtime status. "
        "Works even when no program is running (unlike ur_get_state's joint/TCP "
        "fields). Returns ok=false if RTDE is disabled or unreachable.",
        _object_schema({}),
        _h_rtde_state,
    ),
    Tool(
        "ur_set_speed_override",
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
        "ur_set_digital_output",
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
]

_BY_NAME = {t.name: t for t in TOOLS}


def get_tool_schemas() -> list[dict]:
    """Return the tools as ``{name, description, input_schema}`` dicts.

    This is the capability-discovery payload — hand it straight to a model's
    tool-calling API (rename ``input_schema`` to ``parameters`` for the
    OpenAI-style shape; it's already the right shape for Anthropic's API).
    """
    return [{"name": t.name, "description": t.description, "input_schema": t.input_schema} for t in TOOLS]


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
    tool = _BY_NAME.get(name)
    if tool is None:
        raise ToolError(f"unknown tool: {name!r}. Known: {sorted(_BY_NAME)}")
    params = params or {}
    _validate(params, tool.input_schema)
    return tool.handler(robot, params)
