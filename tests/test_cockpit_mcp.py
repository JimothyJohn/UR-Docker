"""The combined robot + camera MCP server: the cockpit tool family through a
live (synthetic) cockpit, the in-band errors when the cockpit is down, and the
robot registry still being served alongside."""

from __future__ import annotations

import json
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

from perception.capture import CaptureStore
from perception.cockpitclient import CockpitClient, CockpitUnavailable
from perception.config import PerceptionConfig
from perception.mcp_server import COCKPIT_TOOLS, CockpitTools, build_server
from perception.realsense import SyntheticRgbdCamera
from perception.robotlink import RobotLink
from perception.webapp import ViewerApp, ViewerHandler
from urctl.config import RobotConfig
from urctl.robot import Robot


@pytest.fixture
def cockpit(tmp_path):
    link = RobotLink(RobotConfig(host="fake-ur"), dry_run=True)
    app = ViewerApp(
        SyntheticRgbdCamera(width=96, height=64, fps=0),
        config=PerceptionConfig(),
        store=CaptureStore(tmp_path / "caps"),
        robot=link,
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield f"http://127.0.0.1:{srv.server_address[1]}", app, tmp_path
    srv.shutdown()
    srv.server_close()
    app.stop()


def _call(server, name, args=None, msg_id=1):
    resp = server.handle_message(
        {
            "jsonrpc": "2.0",
            "id": msg_id,
            "method": "tools/call",
            "params": {"name": name, "arguments": args or {}},
        }
    )
    result = resp["result"]
    return result.get("isError", False), json.loads(result["content"][0]["text"])


def test_tools_list_is_robot_plus_cockpit(cockpit):
    url, _, _ = cockpit
    server = build_server(Robot(RobotConfig(), dry_run=True), cockpit_url=url)
    names = [
        t["name"]
        for t in server.handle_message({"jsonrpc": "2.0", "id": 1, "method": "tools/list"})["result"]["tools"]
    ]
    assert "ur_move_tcp" in names and "ur_get_state" in names
    assert [t.name for t in COCKPIT_TOOLS] == [n for n in names if n.startswith(("cam_", "cell_", "cal_"))]
    assert len(names) == len(set(names))
    init = server.handle_message({"jsonrpc": "2.0", "id": 2, "method": "initialize", "params": {}})
    assert init["result"]["serverInfo"]["name"] == "cell"


def test_snapshot_segment_locate_through_mcp(cockpit):
    url, app, tmp_path = cockpit
    server = build_server(Robot(RobotConfig(), dry_run=True), cockpit_url=url)
    err, snap = _call(server, "cam_snapshot", {"dir": str(tmp_path / "snaps"), "name": "eyes"})
    assert not err and Path(snap["color_png"]).stat().st_size > 100 and Path(snap["depth_png"]).exists()
    assert Path(snap["color_png"]).read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert "mask_png" not in snap
    err, seg = _call(server, "cam_segment", {"x": 48, "y": 32})
    assert not err and seg["area_px"] > 0 and "mask_png_b64" not in seg and seg["features"]["point_m"]
    err, loc = _call(server, "cam_locate", {"standoff_m": 0.05})
    assert not err and loc["ok"] and len(loc["approach_pose"]) == 6 and loc["standoff_m"] == 0.05
    err, mv = _call(server, "cam_move_to_approach", {"pose": loc["approach_pose"]})
    assert not err and mv["ok"] and mv["dry_run"] is True
    err, jog = _call(server, "cell_jog", {"delta": [0.01, 0, 0, 0, 0, 0]})
    assert not err and jog["ok"] and jog["dry_run"]
    err, jog = _call(server, "cell_jog", {"delta": [0.5, 0, 0, 0, 0, 0]})
    assert err and "at most" in jog["error"]
    # the snapshot after a segment carries the mask too
    err, snap2 = _call(server, "cam_snapshot", {"dir": str(tmp_path / "snaps"), "name": "eyes2"})
    assert not err and Path(snap2["mask_png"]).exists() and snap2["features"]["point_m"]
    err, ev = _call(server, "cam_events", {"after": 0})
    kinds = [e["kind"] for e in ev["events"]]
    assert "snapshot" in kinds and "segment" in kinds and "robot" in kinds
    assert any("jog: ValueError" in e["message"] for e in ev["events"])
    err, doc = _call(server, "cell_doctor", {"robot": False})
    assert not err and doc["ok"] and {c["name"] for c in doc["checks"]} >= {"host", "stream", "cell"}


def test_schema_validation_is_in_band(cockpit):
    url, _, _ = cockpit
    server = build_server(Robot(RobotConfig(), dry_run=True), cockpit_url=url)
    err, body = _call(server, "cam_segment", {"x": "12"})
    assert err and "must be integer" in body["error"]
    err, body = _call(server, "cam_segment", {"bogus": 1})
    assert err and "unexpected" in body["error"]
    err, body = _call(server, "cam_move_to_approach", {})
    assert err and "missing required" in body["error"]


def test_cockpit_down_is_an_in_band_error_and_robot_tools_still_serve():
    server = build_server(Robot(RobotConfig(), dry_run=True), cockpit_url="http://127.0.0.1:9")
    err, body = _call(server, "cam_info")
    assert (
        err and "no cockpit at http://127.0.0.1:9" in body["error"] and "perception --cell" in body["error"]
    )
    err, body = _call(server, "ur_move_tcp", {"pose": [0, 0, 0.01, 0, 0, 0], "relative": True})
    assert not err and body["ok"] and body["dry_run"]


def test_client_raises_unavailable():
    with pytest.raises(CockpitUnavailable):
        CockpitClient("http://127.0.0.1:9", timeout=1).info()
    tools = CockpitTools(CockpitClient("http://127.0.0.1:9", timeout=1))
    assert tools.owns("cam_info") and not tools.owns("ur_get_state")


def test_cal_tools_through_mcp(cockpit, tmp_path, monkeypatch):
    url, _, _ = cockpit
    monkeypatch.chdir(tmp_path)
    server = build_server(Robot(RobotConfig(), dry_run=True), cockpit_url=url)
    err, st = _call(server, "cal_status")
    assert not err and st["views"] == [] and st["active_handeye"]["source"].startswith("bracket")
    err, m = _call(server, "cal_record_mark")
    assert not err and m["ok"]
    for _ in range(3):
        err, v = _call(server, "cal_add_view", {"x": 48, "y": 32})
        assert not err and v["ok"]
    err, bad = _call(server, "cal_add_view", {"x": 48})
    assert err and "missing required" in bad["error"]
    err, sol = _call(server, "cal_solve")
    assert not err and sol["ok"] and "env_line" in sol
    err, ap = _call(server, "cal_apply", {"save": False})
    assert err and "warnings" in ap["error"]
    err, ap = _call(server, "cal_apply", {"save": False, "force": True})
    assert not err and ap["handeye"]["calibrated"] and "saved" not in ap
    err, rs = _call(server, "cal_reset")
    assert not err and rs["views"] == []
