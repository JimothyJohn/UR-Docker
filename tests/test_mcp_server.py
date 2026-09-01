"""Tests for the stdlib MCP server (urctl.mcp_server) — JSON-RPC dispatch and
the stdio framing, against a dry-run Robot. No SDK, no subprocess."""

from __future__ import annotations

import io
import json

from urctl import RobotConfig
from urctl.mcp_server import McpServer
from urctl.robot import Robot


def _server() -> McpServer:
    return McpServer(Robot(RobotConfig(), dry_run=True))


def _req(method: str, params: dict | None = None, msg_id: int | None = 1) -> dict:
    msg: dict = {"jsonrpc": "2.0", "method": method}
    if msg_id is not None:
        msg["id"] = msg_id
    if params is not None:
        msg["params"] = params
    return msg


class TestDispatch:
    def test_initialize_echoes_protocol_and_advertises_tools(self):
        resp = _server().handle_message(
            _req("initialize", {"protocolVersion": "2025-03-26", "capabilities": {}})
        )
        assert resp["id"] == 1
        assert resp["result"]["protocolVersion"] == "2025-03-26"
        assert "tools" in resp["result"]["capabilities"]
        assert resp["result"]["serverInfo"]["name"] == "urctl"

    def test_ping(self):
        assert _server().handle_message(_req("ping"))["result"] == {}

    def test_tools_list_matches_registry(self):
        resp = _server().handle_message(_req("tools/list"))
        tools = resp["result"]["tools"]
        names = {t["name"] for t in tools}
        assert "ur_move_joints" in names and "ur_system_snapshot" in names
        assert len(tools) >= 20
        assert all("inputSchema" in t for t in tools)

    def test_tools_call_dry_run_ok(self):
        resp = _server().handle_message(_req("tools/call", {"name": "ur_popup", "arguments": {"text": "hi"}}))
        result = resp["result"]
        assert result["isError"] is False
        payload = json.loads(result["content"][0]["text"])
        assert payload["ok"] and payload["dry_run"]

    def test_tools_call_unknown_tool_is_in_band_error(self):
        # Tool-level failures are content with isError, not JSON-RPC errors,
        # so the calling model can read and correct them.
        resp = _server().handle_message(_req("tools/call", {"name": "ur_nope", "arguments": {}}))
        result = resp["result"]
        assert result["isError"] is True
        assert "unknown tool" in json.loads(result["content"][0]["text"])["error"]

    def test_tools_call_missing_name_is_invalid_params(self):
        resp = _server().handle_message(_req("tools/call", {}))
        assert resp["error"]["code"] == -32602

    def test_unknown_method_is_method_not_found(self):
        resp = _server().handle_message(_req("wat/huh"))
        assert resp["error"]["code"] == -32601

    def test_notifications_get_no_response(self):
        srv = _server()
        assert srv.handle_message(_req("notifications/initialized", msg_id=None)) is None
        assert srv.handle_message(_req("some/unknown", msg_id=None)) is None

    def test_invalid_request_shape(self):
        resp = _server().handle_message({"jsonrpc": "2.0", "id": 7})  # no method
        assert resp["error"]["code"] == -32600


class TestStdioFraming:
    def test_full_session_over_pipes(self):
        lines = [
            json.dumps(_req("initialize", {"protocolVersion": "2025-06-18"})),
            json.dumps(_req("notifications/initialized", msg_id=None)),
            json.dumps(_req("tools/list", msg_id=2)),
            "",  # blank lines are skipped
            json.dumps(_req("tools/call", {"name": "ur_get_state", "arguments": {}}, msg_id=3)),
            "this is not json",
        ]
        out = io.StringIO()
        _server().serve_stdio(stdin=io.StringIO("\n".join(lines) + "\n"), stdout=out)
        responses = [json.loads(line) for line in out.getvalue().splitlines()]
        # initialize, tools/list, tools/call, parse error — notification and
        # blank line produce nothing.
        assert len(responses) == 4
        assert responses[0]["id"] == 1 and "serverInfo" in responses[0]["result"]
        assert responses[1]["id"] == 2
        assert responses[2]["id"] == 3
        assert responses[3]["error"]["code"] == -32700

    def test_responses_are_single_lines(self):
        out = io.StringIO()
        _server().serve_stdio(stdin=io.StringIO(json.dumps(_req("tools/list")) + "\n"), stdout=out)
        payload = out.getvalue()
        assert payload.endswith("\n") and payload.count("\n") == 1
