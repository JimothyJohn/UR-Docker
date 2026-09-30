"""``perceptronics pick-server`` — the Perceptronic Pick node's server beside a running cockpit.

The cockpit (``perceptronics gui``) serves the PolyScope 5 **Perceptronic Pick** program node
itself: the ``--pick-port`` socket the node's URScript talks to, and the two teach-screen
routes (``GET /api/pick/detect``, ``POST /api/pick/preview``). One process owns the USB
camera, so a cockpit that is already running — one that predates the node, or one another
session is using — can't grow those without a restart. This process adds them from outside:

* the pick socket (:class:`perceptronics.picknode.PickServer`), answering from the running
  cockpit's frames (``GET /api/rgbd?after=N`` long-polls) and hand-eye (``/api/info``);
* ``/api/pick/detect`` and ``/api/pick/preview`` over HTTP, the flange for the preview read
  from the controller's **state broadcast** only (``get_flange_pose(script_fallback=False)``
  — it never sends a script, so it never replaces the operator's running program);
* every other request forwarded to the cockpit unchanged, so the node's one "cockpit URL"
  (this process) still gets the feed, segment, locate and move routes.

Everything it hears and answers — including the program's own ``LOG`` lines (start,
FIND, look, REFINE, hover, grip, lift, and why it gave up) — goes, timestamped, to
stderr, to ``--log`` (default ``captures/pick-server.log``; when that isn't writable —
a sudo cockpit run leaves ``captures/`` root-owned — the per-user log directory,
:func:`user_log_dir`) and to ``GET /api/pick/log``
(the last lines as text, for a browser next to the pendant).

Point the node's cockpit URL at this process (``http://<host>:7631``); the detect reply
tells it the pick socket's port. It moves nothing; like the cockpit it is unauthenticated,
so bind it to the cell network only.
"""

from __future__ import annotations

import collections
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .partspec import PartSpec, from_payload, from_query
from .pickcycle import DEFAULT_COCKPIT, Cockpit, CockpitError
from .picknode import (
    DEFAULT_PICK_PORT,
    PickPlanner,
    PickServer,
    detect_report,
    parse_preview_request,
    preview,
)

DEFAULT_SIDECAR_PORT = 7631
DEFAULT_LOG = Path("captures") / "pick-server.log"
LOG_TAIL = 400
MAX_BODY = 1 << 20  # the node's requests are small JSON; refuse anything bigger
PROXY_TIMEOUT_S = 30.0  # the cockpit's longest long-poll is 10 s
_HOP_BY_HOP = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailer",
    "transfer-encoding",
    "upgrade",
    "content-length",
    "server",
    "date",
}


def user_log_dir() -> Path:
    """The per-user log directory: ``~/Library/Logs/perceptronics`` (macOS),
    ``%LOCALAPPDATA%\\perceptronics\\logs`` (Windows), ``$XDG_STATE_HOME/perceptronics``
    (else ``~/.local/state/perceptronics``)."""
    if sys.platform == "darwin":
        return Path.home() / "Library" / "Logs" / "perceptronics"
    if os.name == "nt":
        return Path(os.environ.get("LOCALAPPDATA") or Path.home()) / "perceptronics" / "logs"
    return Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state") / "perceptronics"


def writable_log(preferred: Path) -> Path | None:
    """``preferred`` if it can be appended to, else the same name in :func:`user_log_dir`,
    else None (stderr only)."""
    for path in (preferred, user_log_dir() / preferred.name):
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8"):
                pass
        except OSError:
            continue
        return path
    return None


class PickLog:
    """One line per event: stderr, an append-only file, and the last lines in memory."""

    def __init__(self, path: Path | None = None):
        self.path = path
        self.tail: collections.deque[str] = collections.deque(maxlen=LOG_TAIL)
        self._lock = threading.Lock()

    def say(self, text: str) -> None:
        clean = "".join(c if " " <= c <= "~" or c in "°×→—" else " " for c in text)
        line = f"{time.strftime('%H:%M:%S')}.{int(time.time() * 1000) % 1000:03d} {clean}"
        with self._lock:
            self.tail.append(line)
            print(f"[pick-server] {line}", file=sys.stderr, flush=True)
            if self.path is not None:
                try:
                    with self.path.open("a", encoding="utf-8") as f:
                        f.write(line + "\n")
                except OSError as exc:
                    print(f"[pick-server] log file {self.path}: {exc} — stderr only", file=sys.stderr)
                    self.path = None

    def text(self) -> str:
        with self._lock:
            return "\n".join(self.tail) + "\n"


LOG = PickLog()


def _say(text: str) -> None:
    LOG.say(text)


class CockpitFrames:
    """The running cockpit as the planner's frame source, hand-eye and tool length."""

    def __init__(self, base: str, *, timeout_s: float = 15.0):
        self.base = base.rstrip("/")
        self.cockpit = Cockpit(self.base, timeout_s)

    def info(self) -> dict:
        return self.cockpit.get("/api/info")

    def latest_seq(self) -> int:
        try:
            return int(self.info().get("seq") or 0)
        except (OSError, ValueError, TypeError) as exc:
            _say(f"cockpit /api/info: {exc}")
            return 0

    def frame(self, after: int | None, timeout_s: float = 2.0) -> tuple | None:
        """``(seq, w, h, ch, rgb, depth, depth_scale_m, K)`` newer than ``after``, or None."""
        try:
            hdr, w, h, ch, rgb, depth = self.cockpit.frame(after=after, timeout_ms=int(timeout_s * 1000))
            seq = int(hdr["seq"])
            frame = (seq, w, h, ch, rgb, depth, float(hdr["depth_scale_m"]), hdr["intrinsics"])
        except (OSError, CockpitError, ValueError, KeyError, TypeError) as exc:
            _say(f"cockpit /api/rgbd: {exc}")
            return None
        if after is not None and seq <= after:
            try:
                info = self.info()
            except (OSError, ValueError):
                info = {}
            _say(
                f"cockpit frames stalled at seq {seq} (asked for > {after}); "
                f"camera error: {str(info.get('last_error') or 'none')[:200]}"
            )
            return None
        return frame

    def robot_info(self) -> dict:
        try:
            return self.info().get("robot") or {}
        except (OSError, ValueError) as exc:
            _say(f"cockpit /api/info: {exc}")
            return {}

    def handeye(self) -> list[float] | None:
        return (self.robot_info().get("handeye") or {}).get("flange_to_color_pose")

    def tip_m(self) -> float | None:
        tip = (self.robot_info().get("approach") or {}).get("tip_m")
        return float(tip) if isinstance(tip, (int, float)) else None


class Sidecar:
    """The pieces the HTTP handler calls: planner, detect, preview."""

    def __init__(self, frames: CockpitFrames, flange_reader, *, tip_m: float, pick_port: int | None = None):
        self.frames, self.flange_reader, self.pick_port = frames, flange_reader, pick_port
        self.tip_m = frames.tip_m() or tip_m
        self.planner = PickPlanner(
            frames.frame,
            frames.latest_seq,
            frames.handeye,
            tip_m=self.tip_m,
            log=lambda text, ok: _say(text),
        )

    def detect(self, part: PartSpec | None = None) -> dict:
        frame = self.frames.frame(None, 0.0)
        if frame is None:
            return {"ok": False, "error": f"no frame from the cockpit at {self.frames.base}"}
        return detect_report(
            frame,
            pick_port=self.pick_port,
            handeye=self.frames.handeye() is not None,
            tip_m=self.tip_m,
            part=part,
        )

    def preview(self, payload: dict) -> dict:
        pixel, grip, hover = parse_preview_request(payload)
        part = from_payload(payload)
        return preview(
            self.planner, self.flange_reader(), pixel, grip_below_mm=grip, hover_mm=hover, part=part
        )


class _Handler(BaseHTTPRequestHandler):
    server_version = "perceptronics-pick-server"

    @property
    def side(self) -> Sidecar:
        return self.server.sidecar  # type: ignore[attr-defined]

    def log_message(self, format: str, *args) -> None:  # noqa: A002 — the request line is untrusted
        pass

    def do_GET(self) -> None:
        self._route("GET")

    def do_POST(self) -> None:
        self._route("POST")

    def do_OPTIONS(self) -> None:
        self._route("OPTIONS")

    def _json(self, obj: dict, status: int = 200) -> None:
        data = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> bytes | None:
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = -1
        if n < 0 or n > MAX_BODY:
            return None
        return self.rfile.read(n) if n else b""

    def _route(self, method: str) -> None:
        # only origin-form paths: an absolute URI or `//host` must never pick the upstream
        if not self.path.startswith("/") or self.path.startswith("//"):
            self._json({"ok": False, "error": "bad request target"}, 400)
            return
        body = self._body()
        if body is None:
            self._json({"ok": False, "error": f"body missing a length or over {MAX_BODY} bytes"}, 413)
            return
        url = urllib.parse.urlsplit(self.path)
        route = url.path
        try:
            if method == "GET" and route == "/api/pick/log":
                data = LOG.text().encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if method == "GET" and route == "/api/pick/detect":
                try:
                    part = from_query(urllib.parse.parse_qs(url.query))
                except ValueError as exc:
                    self._json({"ok": False, "error": f"bad request: {exc}"}, 400)
                    return
                self._json(self.side.detect(part))
                return
            if method == "POST" and route == "/api/pick/preview":
                try:
                    payload = json.loads(body or b"{}")
                    if not isinstance(payload, dict):
                        raise ValueError("the body must be a JSON object")
                    out = self.side.preview(payload)
                except ValueError as exc:
                    self._json({"ok": False, "error": f"bad request: {exc}"}, 400)
                    return
                self._json(out)
                return
        except Exception as exc:  # a planner/cockpit failure is the node's answer, not a dropped socket
            self._json({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, 500)
            return
        self._proxy(method, body)

    def _proxy(self, method: str, body: bytes) -> None:
        headers = {k: self.headers[k] for k in ("Content-Type", "Accept", "Origin") if self.headers.get(k)}
        req = urllib.request.Request(
            self.side.frames.base + self.path, data=body or None, method=method, headers=headers
        )
        try:
            with urllib.request.urlopen(req, timeout=PROXY_TIMEOUT_S) as resp:
                status, out_headers, data = resp.status, resp.headers, resp.read()
        except urllib.error.HTTPError as exc:
            status, out_headers, data = exc.code, exc.headers, exc.read()
        except (urllib.error.URLError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            self._json(
                {"ok": False, "error": f"cockpit unreachable at {self.side.frames.base}: {reason}"}, 502
            )
            return
        self.send_response(status)
        for key, value in out_headers.items():
            if key.lower() not in _HOP_BY_HOP:
                self.send_header(key, value)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class SidecarServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, bind: str, port: int, sidecar: Sidecar):
        super().__init__((bind, port), _Handler)
        self.sidecar = sidecar

    def start(self) -> None:
        threading.Thread(target=self.serve_forever, name="pick-sidecar-http", daemon=True).start()

    def stop(self) -> None:
        self.shutdown()
        self.server_close()


def add_pick_server_args(ap) -> None:
    ap.add_argument(
        "--cockpit", default=DEFAULT_COCKPIT, help=f"the running cockpit (default {DEFAULT_COCKPIT})"
    )
    ap.add_argument("--bind", default="127.0.0.1", help="interface to bind (default: loopback only)")
    ap.add_argument(
        "--port",
        type=int,
        default=DEFAULT_SIDECAR_PORT,
        help=f"HTTP port the node's cockpit URL points at (default {DEFAULT_SIDECAR_PORT})",
    )
    ap.add_argument(
        "--pick-port",
        type=int,
        default=DEFAULT_PICK_PORT,
        help=f"TCP port for the node's URScript (default {DEFAULT_PICK_PORT})",
    )
    ap.add_argument("--host", default=None, help="the robot (default: UR_HOST / the cell)")
    ap.add_argument(
        "--dry-run", action="store_true", help="stand-in flange pose for the preview; no controller"
    )
    ap.add_argument(
        "--log",
        type=Path,
        default=DEFAULT_LOG,
        help=f"append every request, answer and robot LOG line here (default {DEFAULT_LOG})",
    )


def run_pick_server(args) -> int:
    from urctl import Robot, RobotConfig

    from .handeye import tip_m_from_env

    robot = Robot(RobotConfig.from_env(host=args.host), dry_run=args.dry_run)
    LOG.path = writable_log(args.log)
    if LOG.path != args.log:
        print(
            f"[pick-server] {args.log} is not writable — logging to {LOG.path or 'stderr only'}",
            file=sys.stderr,
        )

    def flange() -> dict:
        return robot.get_flange_pose(script_fallback=False)

    frames = CockpitFrames(args.cockpit)
    probe = frames.robot_info()
    if not probe:
        _say(f"WARNING: no answer from the cockpit at {frames.base} yet — requests will fail until it is up")
    elif not probe.get("handeye"):
        _say("WARNING: the cockpit has no robot link, so no hand-eye — FIND answers -3")
    sidecar = Sidecar(frames, flange, tip_m=tip_m_from_env())
    pick = PickServer(args.bind, args.pick_port, sidecar.planner)
    sidecar.pick_port = pick.server_address[1]
    http = SidecarServer(args.bind, args.port, sidecar)
    pick.start()
    _say(
        f"http://{args.bind}:{http.server_address[1]} (the node's cockpit URL) + pick socket "
        f"{args.bind}:{sidecar.pick_port}, over {frames.base}; robot {robot.config.host}"
        f"{' (dry-run)' if args.dry_run else ''}; tip {sidecar.tip_m:.3f} m; log {LOG.path}. Ctrl-C to stop."
    )
    if args.bind not in ("127.0.0.1", "localhost", "::1"):
        _say(f"WARNING: bound to {args.bind} with no authentication — only on a trusted cell network.")
    try:
        http.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        http.server_close()
        pick.stop()
    return 0
