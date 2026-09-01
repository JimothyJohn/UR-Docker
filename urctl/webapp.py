"""urctl-gui — a local web control panel ("your own personal GUI") for one robot.

A zero-dependency HTTP server (stdlib ``http.server``) that serves the
single-file cockpit UI in ``urctl/webui/index.html`` and exposes the harness
over a small JSON API. Runs on the laptop, binds **127.0.0.1 only**, and works
equally against URSim and a real e-Series — the same property as the rest of
the toolkit, because every endpoint is just a :class:`urctl.Robot` /
:class:`urctl.sysinfo.SystemInspector` call.

    uv run urctl gui                       # URSim (localhost)
    uv run urctl --host 192.168.1.50 gui   # the real robot
    uv run urctl-gui --host 192.168.1.50   # same, dedicated entry point

Design (see docs/harness.md §5):

  * ``GET  /api/state``      Dashboard-level state (modes, Remote gate).
  * ``GET  /api/telemetry``  one deep RTDE sample.
  * ``GET  /api/stream``     Server-Sent Events: deep samples at ~10 Hz. Each
                             stream holds its *own* RtdeClient so viewers never
                             contend with actions (RTDE output recipes are
                             per-connection; reads don't conflict).
  * ``GET  /api/snapshot``   the cell model (cached; ``?refresh=1`` re-reads).
  * ``GET  /api/programs``   controller program inventory.
  * ``POST /api/action``     ``{"tool": ..., "params": {...}}`` dispatched via
                             :func:`urctl.tools.call_tool` — schema-validated,
                             safety-enveloped, audit-logged, exactly like the
                             CLI and MCP. A lock serializes actions.
  * ``GET  /api/actions``    ring buffer of recent action results (the GUI's
                             action-history panel).

Security model: the server is a *local cockpit*, not a product web app — it
binds loopback and has no auth, the same trust level as running ``urctl``
yourself. Do not bind it to a routable interface.
"""

from __future__ import annotations

import json
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from . import codes
from .config import RobotConfig
from .robot import Robot
from .tools import ToolError, call_tool, get_tool_schemas

DEFAULT_PORT = 7620
STREAM_FREQUENCY = 12.5  # Hz — smooth for a live view, light on the controller.

_WEBUI = Path(__file__).parent / "webui" / "index.html"


class GuiApp:
    """The state behind the HTTP handlers: one Robot, one snapshot cache, one
    action lock, one ring buffer of results."""

    def __init__(self, config: RobotConfig, *, dry_run: bool = False):
        self.config = config
        self.robot = Robot(config, dry_run=dry_run)
        self.action_lock = threading.Lock()
        self.actions: deque[dict] = deque(maxlen=200)
        self._snapshot: dict | None = None
        self._snapshot_lock = threading.Lock()

    # -- API operations --------------------------------------------------------

    def state(self) -> dict:
        with self.action_lock:
            return self.robot.get_state()

    def telemetry(self) -> dict:
        with self.action_lock:
            return self.robot.rtde_state(deep=True)

    def snapshot(self, *, refresh: bool = False) -> dict:
        with self._snapshot_lock:
            if self._snapshot is None or refresh:
                from .sysinfo import SystemInspector, runner_for

                result: dict = {"ok": True, "host": self.config.host}
                result["system"] = SystemInspector(runner_for(self.config)).snapshot()
                self._snapshot = result
            return self._snapshot

    def programs(self) -> dict:
        from .sysinfo import SystemInspector, runner_for

        return {"ok": True, **SystemInspector(runner_for(self.config)).programs()}

    def action(self, tool: str, params: dict) -> dict:
        with self.action_lock:
            result = call_tool(self.robot, tool, params)
        entry = {"tool": tool, "params": params, "ok": result.get("ok"), "ts": result.get("ts", time.time())}
        # Keep results small in the history; the full result went to the caller
        # and the audit log.
        if not result.get("ok"):
            entry["error"] = {k: v for k, v in result.items() if k in ("error", "safety", "reply", "landed")}
        self.actions.append(entry)
        return result

    def stream_samples(self):
        """Generator of shaped deep samples from a dedicated RTDE client.

        Runs in the SSE request's thread. Raises to end the stream on any
        connection problem — the browser's EventSource reconnects by itself.
        """
        from .rtde import DEEP_OUTPUTS, RtdeClient

        client = RtdeClient(self.config, outputs=list(DEEP_OUTPUTS), frequency=STREAM_FREQUENCY, strict=False)
        client.connect()
        client._start()
        try:
            while True:
                raw = client.receive()
                yield codes.shape_rtde_sample(raw, deep=True, unavailable=client.dropped_outputs)
        finally:
            client.close()


class GuiHandler(BaseHTTPRequestHandler):
    """Routes to the :class:`GuiApp` stored on the server instance."""

    protocol_version = "HTTP/1.1"
    server_version = "urctl-gui"

    @property
    def app(self) -> GuiApp:
        return self.server.app  # type: ignore[attr-defined]

    # -- plumbing --------------------------------------------------------------

    def log_message(self, fmt, *args):  # quiet by default; actions are audited anyway
        pass

    def _send_json(self, obj: dict, status: int = 200) -> None:
        body = json.dumps(obj, default=str).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _guarded(self, fn) -> None:
        try:
            self._send_json(fn())
        except ToolError as exc:
            self._send_json({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:  # a failed robot call must not kill the server
            self._send_json({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, status=500)

    # -- routes ----------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 (http.server API)
        url = urlparse(self.path)
        route = url.path.rstrip("/") or "/"
        if route == "/":
            body = _WEBUI.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif route == "/api/config":
            self._send_json(
                {
                    "ok": True,
                    "host": self.app.config.host,
                    "platform": self.app.config.platform,
                    "dry_run": self.app.robot.dry_run,
                    "tools": [t["name"] for t in get_tool_schemas()],
                }
            )
        elif route == "/api/state":
            self._guarded(self.app.state)
        elif route == "/api/telemetry":
            self._guarded(self.app.telemetry)
        elif route == "/api/snapshot":
            refresh = parse_qs(url.query).get("refresh", ["0"])[0] == "1"
            self._guarded(lambda: self.app.snapshot(refresh=refresh))
        elif route == "/api/programs":
            self._guarded(self.app.programs)
        elif route == "/api/actions":
            self._send_json({"ok": True, "actions": list(self.app.actions)})
        elif route == "/api/stream":
            self._stream()
        else:
            self._send_json({"ok": False, "error": f"no route {route}"}, status=404)

    def do_POST(self) -> None:  # noqa: N802
        route = urlparse(self.path).path.rstrip("/")
        if route != "/api/action":
            self._send_json({"ok": False, "error": f"no route {route}"}, status=404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length) or b"{}")
            tool = payload["tool"]
            params = payload.get("params") or {}
        except (json.JSONDecodeError, KeyError) as exc:
            self._send_json({"ok": False, "error": f"bad request: {exc}"}, status=400)
            return
        self._guarded(lambda: self.app.action(tool, params))

    def _stream(self) -> None:
        """Server-Sent Events telemetry. Ends (client reconnects) on any error."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        # SSE is an open-ended response: no Content-Length, close delimits it.
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            for sample in self.app.stream_samples():
                self.wfile.write(b"data: " + json.dumps(sample, default=str).encode() + b"\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass  # viewer closed the tab
        except Exception as exc:
            # Send one structured error event so the UI can say why, then end.
            try:
                msg = json.dumps({"stream_error": str(exc)}).encode()
                self.wfile.write(b"data: " + msg + b"\n\n")
                self.wfile.flush()
            except OSError:
                pass


def serve(
    config: RobotConfig | None = None,
    *,
    port: int = DEFAULT_PORT,
    open_browser: bool = True,
    dry_run: bool = False,
) -> None:
    """Run the GUI server until interrupted (the ``urctl gui`` entry point)."""
    config = config or RobotConfig.from_env()
    server = ThreadingHTTPServer(("127.0.0.1", port), GuiHandler)
    server.app = GuiApp(config, dry_run=dry_run)  # type: ignore[attr-defined]
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"urctl gui -> {url}   (robot: {config.host}, Ctrl-C to stop)")
    if open_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        server.app.robot.close()  # type: ignore[attr-defined]


def main(argv: list[str] | None = None) -> int:
    """Standalone ``urctl-gui`` console script."""
    for stream in (sys.stdout, sys.stderr):  # legacy Windows codepages: replace, don't crash
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    import argparse

    ap = argparse.ArgumentParser(prog="urctl-gui", description="local web control panel for a UR robot")
    ap.add_argument("--host", default=None, help="robot host (default: UR_HOST or localhost)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"GUI port (default {DEFAULT_PORT})")
    ap.add_argument("--no-browser", action="store_true", help="don't open the browser automatically")
    ap.add_argument("--dry-run", action="store_true", help="validate + audit actions, send nothing")
    args = ap.parse_args(argv)
    serve(
        RobotConfig.from_env(host=args.host),
        port=args.port,
        open_browser=not args.no_browser,
        dry_run=args.dry_run,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
