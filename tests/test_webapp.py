"""Tests for the GUI web server (urctl.webapp) — routing, action dispatch,
and error shaping, against a dry-run Robot on an ephemeral port. No robot, no
browser: these cover the server half of the cockpit; the UI itself is verified
manually/via Playwright against a live target."""

from __future__ import annotations

import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from urctl import RobotConfig
from urctl.webapp import GuiApp, GuiHandler


@pytest.fixture
def server():
    srv = ThreadingHTTPServer(("127.0.0.1", 0), GuiHandler)
    srv.app = GuiApp(RobotConfig(), dry_run=True)
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()


def _get(url: str) -> tuple[int, dict | bytes]:
    try:
        with urllib.request.urlopen(url, timeout=5) as resp:
            body = resp.read()
            status = resp.status
    except urllib.error.HTTPError as err:
        body = err.read()
        status = err.code
    if b"{" in body[:1]:
        return status, json.loads(body)
    return status, body


def _post(url: str, payload: dict) -> tuple[int, dict]:
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as err:
        return err.code, json.loads(err.read())


class TestRoutes:
    def test_index_serves_cockpit_html(self, server):
        status, body = _get(server + "/")
        assert status == 200
        assert b"urctl cockpit" in body
        assert b"api/stream" in body  # the UI actually wires the SSE endpoint

    def test_config_lists_tools_and_target(self, server):
        status, cfg = _get(server + "/api/config")
        assert status == 200
        assert cfg["ok"] and cfg["dry_run"] is True
        assert "ur_move_joints" in cfg["tools"]
        assert cfg["host"] == "localhost"

    def test_unknown_route_404s(self, server):
        status, body = _get(server + "/api/nope")
        assert status == 404
        assert body["ok"] is False

    def test_action_dispatches_and_lands_in_history(self, server):
        status, res = _post(server + "/api/action", {"tool": "ur_popup", "params": {"text": "hi"}})
        assert status == 200
        assert res["ok"] and res["dry_run"]
        _, hist = _get(server + "/api/actions")
        assert [a["tool"] for a in hist["actions"]] == ["ur_popup"]
        assert hist["actions"][0]["ok"] is True

    def test_action_unknown_tool_is_400_not_500(self, server):
        status, res = _post(server + "/api/action", {"tool": "ur_launch_missiles", "params": {}})
        assert status == 400
        assert res["ok"] is False
        assert "unknown tool" in res["error"]

    def test_action_schema_violation_is_400(self, server):
        status, res = _post(server + "/api/action", {"tool": "ur_move_joints", "params": {}})
        assert status == 400
        assert "missing required" in res["error"]

    def test_action_malformed_body_is_400(self, server):
        req = urllib.request.Request(
            server + "/api/action",
            data=b"not json",
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                status = resp.status
        except urllib.error.HTTPError as err:
            status = err.code
        assert status == 400

    def test_post_to_get_route_404s(self, server):
        status, _ = _post(server + "/api/state", {})
        assert status == 404


class TestGuiApp:
    def test_action_history_is_bounded(self):
        app = GuiApp(RobotConfig(), dry_run=True)
        for i in range(250):
            app.action("ur_popup", {"text": str(i)})
        assert len(app.actions) == 200
        assert app.actions[-1]["params"]["text"] == "249"

    def test_snapshot_is_cached(self, monkeypatch):
        app = GuiApp(RobotConfig(), dry_run=True)
        calls = []

        class FakeInspector:
            def __init__(self, runner):
                pass

            def snapshot(self, **kw):
                calls.append(1)
                return {"identity": {}}

        import urctl.sysinfo as sysinfo

        monkeypatch.setattr(sysinfo, "SystemInspector", FakeInspector)
        monkeypatch.setattr(sysinfo, "runner_for", lambda cfg: None)
        app.snapshot()
        app.snapshot()
        assert len(calls) == 1
        app.snapshot(refresh=True)
        assert len(calls) == 2
