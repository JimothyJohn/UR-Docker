"""MCP server — exposes the urctl tool registry over the Model Context Protocol.

A thin adapter: it reuses :mod:`urctl.tools` verbatim, so the MCP tools, the
plain function-calling tools, and the CLI all stay in lockstep. Point it at any
controller via ``UR_HOST`` (or ``--host``) and an MCP client (Claude Desktop,
Claude Code, or any MCP-capable agent) can drive the robot.

Run it::

    pip install 'ur-docker[mcp]'      # installs the `mcp` SDK
    urctl-mcp --host 10.0.0.5         # serves over stdio

Example Claude Desktop / Claude Code MCP config::

    {
      "mcpServers": {
        "ur": {
          "command": "urctl-mcp",
          "args": ["--host", "10.0.0.5"],
          "env": {"UR_AUDIT_LOG": "/tmp/ur-audit.jsonl"}
        }
      }
    }

The `mcp` SDK is an optional dependency; this module only imports it when run,
so the rest of urctl works without it installed.
"""

from __future__ import annotations

import argparse
import json

from .config import RobotConfig
from .robot import Robot
from .tools import ToolError, call_tool, get_tool_schemas


def _require_mcp():
    try:
        import mcp.types as types  # noqa: F401
        from mcp.server import Server  # noqa: F401
        from mcp.server.stdio import stdio_server  # noqa: F401
    except ImportError as exc:  # pragma: no cover - exercised only without the SDK
        raise SystemExit(
            "The MCP server needs the 'mcp' package. Install it with:\n"
            "    pip install 'ur-docker[mcp]'   (or: pip install mcp)"
        ) from exc


def build_server(robot: Robot):
    """Build an MCP Server bound to ``robot``. Importable for tests with the SDK."""
    import mcp.types as types
    from mcp.server import Server

    server = Server("urctl")

    @server.list_tools()
    async def list_tools() -> list:
        return [
            types.Tool(name=t["name"], description=t["description"], inputSchema=t["input_schema"])
            for t in get_tool_schemas()
        ]

    @server.call_tool()
    async def handle_call(name: str, arguments: dict | None) -> list:
        try:
            result = call_tool(robot, name, arguments or {})
        except ToolError as exc:
            result = {"ok": False, "error": str(exc)}
        return [types.TextContent(type="text", text=json.dumps(result, default=str))]

    return server


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="urctl-mcp", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
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
        help="PolyScope X Robot-API HTTP port (default: $UR_ROBOT_API_PORT or 80)",
    )
    ap.add_argument(
        "--dry-run", action="store_true", help="validate and audit tool calls without sending them"
    )
    args = ap.parse_args(argv)

    _require_mcp()
    import anyio
    from mcp.server.stdio import stdio_server

    overrides: dict = {}
    if args.platform is not None:
        overrides["platform"] = args.platform
    if args.robot_api_port is not None:
        overrides["robot_api_port"] = args.robot_api_port
    config = RobotConfig.from_env(host=args.host, **overrides)
    robot = Robot(config, dry_run=args.dry_run)
    server = build_server(robot)

    async def _run() -> None:
        async with stdio_server() as (read_stream, write_stream):
            await server.run(read_stream, write_stream, server.create_initialization_options())

    anyio.run(_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
