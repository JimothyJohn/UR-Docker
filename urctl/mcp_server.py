"""MCP server — exposes the urctl tool registry over the Model Context Protocol.

A thin adapter: it reuses :mod:`urctl.tools` verbatim, so the MCP tools, the
plain function-calling tools, the GUI, and the CLI all stay in lockstep. Point
it at any controller via ``UR_HOST`` (or ``--host``) and an MCP client (Claude
Desktop, Claude Code, or any MCP-capable agent) can drive the robot.

**Zero dependencies.** MCP's stdio transport is newline-delimited JSON-RPC
2.0, so this module speaks it directly with the stdlib — no SDK, no asyncio,
one request at a time (tool calls against a robot are serial anyway). That
keeps the whole toolkit installable anywhere Python runs (Windows, macOS,
Linux, any architecture) with ``pip install ur-docker`` and nothing else.

Run it::

    urctl-mcp --host 10.0.0.5         # serves MCP over stdio

Example Claude Desktop / Claude Code MCP config::

    {
      "mcpServers": {
        "ur": {
          "command": "urctl-mcp",
          "args": ["--host", "10.0.0.5"]
        }
      }
    }

Protocol notes (kept deliberately minimal, per the MCP spec):

  * stdio framing: one JSON-RPC message per line, UTF-8, LF-terminated.
  * ``initialize`` echoes the client's ``protocolVersion`` (we don't gate on
    it — the surface used here, tools/list + tools/call, is stable across
    revisions) and advertises only the ``tools`` capability.
  * Notifications (no ``id``) get no response. Unknown methods get -32601.
  * Protocol goes to stdout; diagnostics go to stderr. Nothing else may
    write to stdout — that would corrupt the stream.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from typing import Protocol

from . import __version__
from .config import RobotConfig
from .robot import Robot
from .tools import ToolError, call_tool, get_tool_schemas


class ToolProvider(Protocol):
    """An extra tool family for :class:`McpServer` (same plain-data shape as
    :mod:`urctl.tools`: ``{name, description, input_schema}`` + a dispatcher)."""

    def schemas(self) -> list[dict]: ...
    def owns(self, name: str) -> bool: ...
    def call(self, name: str, params: dict | None = None) -> dict: ...


# The newest spec revision this server knows; echoed back only if the client
# asks for something unknown (per spec, respond with a version you support).
FALLBACK_PROTOCOL_VERSION = "2025-06-18"

# JSON-RPC 2.0 error codes.
_PARSE_ERROR = -32700
_INVALID_REQUEST = -32600
_METHOD_NOT_FOUND = -32601
_INVALID_PARAMS = -32602
_INTERNAL_ERROR = -32603


class McpServer:
    """A synchronous, transport-agnostic MCP server over the tool registry.

    ``handle_message`` maps one decoded JSON-RPC message to a response dict
    (or ``None`` for notifications) — that seam is what the unit tests drive,
    no pipes required. :meth:`serve_stdio` is the production loop.
    """

    def __init__(self, robot: Robot, *, extra: Sequence[ToolProvider] = (), name: str = "urctl"):
        """``extra`` providers add tool families beyond the robot registry (the
        perception cockpit's ``cam_*`` tools); each must answer ``schemas()``,
        ``owns(name)`` and ``call(name, params)``. ``name`` is what the client
        sees in ``serverInfo``."""
        self.robot = robot
        self.extra = list(extra)
        self.name = name

    # -- JSON-RPC dispatch -----------------------------------------------------

    def handle_message(self, msg: dict) -> dict | None:
        if not isinstance(msg, dict) or msg.get("jsonrpc") != "2.0" or "method" not in msg:
            return self._error(
                msg.get("id") if isinstance(msg, dict) else None, _INVALID_REQUEST, "invalid request"
            )
        method = msg["method"]
        msg_id = msg.get("id")
        is_notification = "id" not in msg
        try:
            if method == "initialize":
                result = self._initialize(msg.get("params") or {})
            elif method == "ping":
                result = {}
            elif method == "tools/list":
                result = self._tools_list()
            elif method == "tools/call":
                result = self._tools_call(msg.get("params") or {})
            elif method.startswith("notifications/"):
                return None
            else:
                if is_notification:
                    return None  # unknown notification: ignore per JSON-RPC
                return self._error(msg_id, _METHOD_NOT_FOUND, f"method not found: {method}")
        except ToolError as exc:
            return self._error(msg_id, _INVALID_PARAMS, str(exc))
        except Exception as exc:  # a tool blowing up must not kill the server
            return self._error(msg_id, _INTERNAL_ERROR, f"{type(exc).__name__}: {exc}")
        if is_notification:
            return None
        return {"jsonrpc": "2.0", "id": msg_id, "result": result}

    @staticmethod
    def _error(msg_id, code: int, message: str) -> dict:
        return {"jsonrpc": "2.0", "id": msg_id, "error": {"code": code, "message": message}}

    # -- methods ---------------------------------------------------------------

    def _initialize(self, params: dict) -> dict:
        return {
            # Echo the client's requested revision (the tools surface is
            # stable across revisions); fall back to the newest we know.
            "protocolVersion": params.get("protocolVersion") or FALLBACK_PROTOCOL_VERSION,
            "capabilities": {"tools": {}},
            "serverInfo": {"name": self.name, "version": __version__},
        }

    def _tools_list(self) -> dict:
        schemas = list(get_tool_schemas())
        for provider in self.extra:
            schemas.extend(provider.schemas())
        return {
            "tools": [
                {"name": t["name"], "description": t["description"], "inputSchema": t["input_schema"]}
                for t in schemas
            ]
        }

    def _tools_call(self, params: dict) -> dict:
        name = params.get("name")
        if not name:
            raise ToolError("tools/call requires a 'name'")
        try:
            args = params.get("arguments") or {}
            provider = next((p for p in self.extra if p.owns(name)), None)
            result = provider.call(name, args) if provider is not None else call_tool(self.robot, name, args)
        except ToolError as exc:
            # Tool-level problems are reported in-band (isError) so the model
            # can read and correct them, per the MCP tools contract.
            return {
                "content": [{"type": "text", "text": json.dumps({"ok": False, "error": str(exc)})}],
                "isError": True,
            }
        except OSError as exc:
            # The controller is unreachable: that is a fact about the cell the
            # model should read (and run cell_doctor on), not a server crash.
            body = {"ok": False, "error": f"robot unreachable at {self.robot.config.host}: {exc}"}
            return {"content": [{"type": "text", "text": json.dumps(body)}], "isError": True}
        return {
            "content": [{"type": "text", "text": json.dumps(result, default=str)}],
            "isError": not result.get("ok", True),
        }

    # -- stdio transport -------------------------------------------------------

    def serve_stdio(self, stdin=None, stdout=None) -> None:
        """Blocking serve loop: one JSON-RPC message per line until EOF."""
        stdin = stdin if stdin is not None else sys.stdin
        stdout = stdout if stdout is not None else sys.stdout
        for line in stdin:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError as exc:
                response: dict | None = self._error(None, _PARSE_ERROR, f"parse error: {exc}")
            else:
                response = self.handle_message(msg)
            if response is not None:
                stdout.write(json.dumps(response, default=str) + "\n")
                stdout.flush()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="urctl-mcp", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
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
        help="PolyScope X Robot-API HTTP port (default: $UR_ROBOT_API_PORT or 80)",
    )
    ap.add_argument(
        "--dry-run", action="store_true", help="validate and audit tool calls without sending them"
    )
    args = ap.parse_args(argv)

    overrides: dict = {}
    if args.platform is not None:
        overrides["platform"] = args.platform
    if args.robot_api_port is not None:
        overrides["robot_api_port"] = args.robot_api_port
    config = RobotConfig.from_env(host=args.host, **overrides)
    robot = Robot(config, dry_run=args.dry_run)
    print(f"urctl-mcp serving {config.host} over stdio", file=sys.stderr)
    try:
        McpServer(robot).serve_stdio()
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
