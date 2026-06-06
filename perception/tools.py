"""Agent tool registry for perception — the framework-neutral surface.

The perception-side analogue of :mod:`urctl.tools`: plain-data tool definitions
(name + description + JSON Schema + handler) that any function-calling agent,
MCP server, or executive can consume. Built to be *merged* into the robot's
tool list later — an agent that can both perceive blobs and move the arm just
concatenates the two registries.

    from perception import PerceptionPipeline
    from perception.tools import get_tool_schemas, call_tool

    pipe = PerceptionPipeline()
    schemas = get_tool_schemas()                       # hand to the model
    result = call_tool(pipe, "perceive_frame", {})     # dispatch a tool call
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from .frame import synthetic_frame
from .pipeline import PerceptionPipeline


@dataclass(frozen=True)
class Tool:
    name: str
    description: str
    input_schema: dict
    handler: Callable[[PerceptionPipeline, dict], dict]


class ToolError(Exception):
    """Raised when a tool call is malformed (unknown tool / invalid args)."""


def _object_schema(properties: dict, required: list[str] | None = None) -> dict:
    return {
        "type": "object",
        "properties": properties,
        "required": required or [],
        "additionalProperties": False,
    }


# ----- handlers --------------------------------------------------------------


def _h_perceive_frame(pipe: PerceptionPipeline, p: dict) -> dict:
    """Capture one frame from the configured webcam and detect blobs."""
    return pipe.process_device().as_dict()


def _h_perceive_synthetic(pipe: PerceptionPipeline, p: dict) -> dict:
    """Run the pipeline on a deterministic synthetic frame (no camera needed)."""
    frame = synthetic_frame(
        width=p.get("width", pipe.config.width),
        height=p.get("height", pipe.config.height),
    )
    return pipe.process(frame).as_dict()


def _h_perceive_image(pipe: PerceptionPipeline, p: dict) -> dict:
    """Run the full depth + blob pipeline on a PNG file on disk."""
    return pipe.process_image(p["path"], max_width=p.get("max_width", 480)).as_dict()


# ----- registry --------------------------------------------------------------

TOOLS: list[Tool] = [
    Tool(
        "perceive_frame",
        "Capture a single RGB frame from the configured camera, estimate "
        "monocular depth, and detect blobs. Returns each blob's pixel centroid, "
        "bounding box, area, mean color, and sampled depth in metres.",
        _object_schema({}),
        _h_perceive_frame,
    ),
    Tool(
        "perceive_synthetic",
        "Run the full depth + blob pipeline on a deterministic synthetic frame "
        "(colored disks on a flat background). Needs no camera or model "
        "weights; use it to smoke-test the perception stack.",
        _object_schema(
            {
                "width": {"type": "integer", "exclusiveMinimum": 0},
                "height": {"type": "integer", "exclusiveMinimum": 0},
            }
        ),
        _h_perceive_synthetic,
    ),
    Tool(
        "perceive_image",
        "Run the full depth + blob pipeline on a PNG file on disk (8-bit "
        "RGB/RGBA). Downsamples to max_width first so it stays fast. Returns the "
        "same blob records as the camera path.",
        _object_schema(
            {
                "path": {"type": "string", "description": "path to a PNG file"},
                "max_width": {
                    "type": "integer",
                    "exclusiveMinimum": 0,
                    "description": "downsample so width <= this (default 480)",
                },
            },
            required=["path"],
        ),
        _h_perceive_image,
    ),
]

_BY_NAME = {t.name: t for t in TOOLS}


def get_tool_schemas() -> list[dict]:
    """Return each tool as ``{name, description, input_schema}`` (Anthropic shape)."""
    return [
        {"name": t.name, "description": t.description, "input_schema": t.input_schema}
        for t in TOOLS
    ]


def _validate(params: dict, schema: dict) -> None:
    """Dependency-free schema check — same contract as urctl.tools._validate."""
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
        if spec["type"] in ("number", "integer") and isinstance(value, bool):
            raise ToolError(f"argument {key!r} must be {spec['type']}, got bool")
        expected = type_map.get(spec["type"])
        if expected and not isinstance(value, expected):
            raise ToolError(f"argument {key!r} must be {spec['type']}, got {type(value).__name__}")


def call_tool(pipe: PerceptionPipeline, name: str, params: dict | None = None) -> dict:
    """Validate and dispatch a perception tool call against ``pipe``."""
    tool = _BY_NAME.get(name)
    if tool is None:
        raise ToolError(f"unknown tool: {name!r}. Known: {sorted(_BY_NAME)}")
    params = params or {}
    _validate(params, tool.input_schema)
    return tool.handler(pipe, params)
