"""perception-gui — a local RGB-D cockpit for a RealSense (or the synthetic
stand-in): live color + depth, hover-to-measure, click-to-segment, and one-key
capture into the RealSenseTrainer dataset layout.

Zero-dependency (stdlib ``http.server``), same shape as ``urctl gui``: a
single-file page in ``perception/webui/index.html`` over a small API.

    uv run perception gui --fake                 # no camera: synthetic scene
    sudo uv run perception gui                   # the D435 (macOS needs root)
    uv run perception gui --bind 0.0.0.0         # serve off-box (Jetson → laptop)

API:

  * ``GET  /api/info``                camera description, config, backends.
  * ``GET  /api/rgbd?after=N``        the newest frame as one binary container
                                      (header JSON + color PNG + zlib'd uint16
                                      depth — see :func:`perception.rgbd.pack_rgbd`).
                                      With ``after``, long-polls (≤ ``timeout_ms``)
                                      until a frame newer than ``N`` exists, so
                                      the page never re-decodes a duplicate and
                                      never outruns the camera.
  * ``POST /api/segment``  ``{x, y}`` segment the object under a pixel of the
                                      latest frame → mask PNG (base64) + features.
  * ``POST /api/nearest``             RealSenseTrainer's nearest-object mask.
  * ``POST /api/capture``  ``{name, include_mask}`` save color/depth(/mask/meta).
  * ``GET  /api/captures``            what's in the capture root.
  * ``POST /api/clear``               drop the current mask.

One background thread pumps the camera; every consumer reads the latest frame
under a condition variable. Segmentation runs on the request thread against a
pinned copy of the frame it was asked about, so a capture "with mask" saves the
exact frame the mask belongs to.

Security model: like ``urctl gui`` this is a cockpit, not a product — no auth.
It binds loopback by default; ``--bind 0.0.0.0`` is for a trusted cell network
(the Jetson serving the operator's laptop) and prints a warning.
"""

from __future__ import annotations

import base64
import json
import sys
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from .capture import CaptureStore
from .config import PerceptionConfig
from .factory import SEGMENT_BACKENDS, make_segmenter
from .realsense import RealSenseError, RgbdCamera, open_camera, platform_hint
from .rgbd import RgbdFrame, pack_rgbd
from .segment import Mask, StubSegmenter, extract_features

DEFAULT_PORT = 7621
DEFAULT_CAPTURE_ROOT = "captures"
REOPEN_DELAY_S = 1.0

_WEBUI = Path(__file__).parent / "webui" / "index.html"


class ViewerApp:
    """State behind the handlers: one camera, one pump thread, one segmenter."""

    def __init__(
        self,
        camera: RgbdCamera,
        *,
        config: PerceptionConfig | None = None,
        store: CaptureStore | None = None,
        segmenter=None,
    ):
        self.camera = camera
        self.config = config or PerceptionConfig.from_env()
        self.store = store or CaptureStore(Path(DEFAULT_CAPTURE_ROOT))
        self.segmenter = segmenter or make_segmenter(self.config)
        self._cond = threading.Condition()
        self._latest: RgbdFrame | None = None
        self._seq = 0
        self._running = False
        self._thread: threading.Thread | None = None
        self.last_error: str | None = None
        self.frames_read = 0
        self.started_at = 0.0
        self._seg_lock = threading.Lock()
        self.mask: Mask | None = None
        self.mask_frame: RgbdFrame | None = None
        self.mask_seq = 0
        self.features: dict | None = None
        self._fps_window: list[float] = []

    # -- lifecycle -------------------------------------------------------------

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self.started_at = time.time()
        self._thread = threading.Thread(target=self._pump, name="rgbd-pump", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        try:
            self.camera.close()
        except Exception:  # closing must never raise on shutdown
            pass

    def _pump(self) -> None:
        opened = False
        while self._running:
            try:
                if not opened:
                    self.camera.open()
                    opened = True
                    self.last_error = None
                frame = self.camera.read()
            except Exception as exc:
                hint = platform_hint(exc) if isinstance(exc, RealSenseError) else ""
                self.last_error = f"{type(exc).__name__}: {exc}" + (f" — {hint}" if hint else "")
                try:
                    self.camera.close()
                except Exception:
                    pass
                opened = False
                # Back off, then try again (camera unplugged / permission fixed / re-plugged).
                with self._cond:
                    self._cond.wait(REOPEN_DELAY_S)
                continue
            now = time.monotonic()
            with self._cond:
                self._latest = frame
                self._seq += 1
                self.frames_read += 1
                self._fps_window.append(now)
                self._fps_window = [t for t in self._fps_window if now - t <= 2.0]
                self._cond.notify_all()

    # -- frames ----------------------------------------------------------------------

    def fps(self) -> float:
        w = self._fps_window
        return (len(w) - 1) / (w[-1] - w[0]) if len(w) > 1 and w[-1] > w[0] else 0.0

    def latest(self) -> tuple[int, RgbdFrame | None]:
        with self._cond:
            return self._seq, self._latest

    def wait_frame(self, after: int, timeout_s: float) -> tuple[int, RgbdFrame | None]:
        """Newest frame with seq > ``after`` (blocks ≤ ``timeout_s``); else the newest."""
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._running and self._seq <= after:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(remaining)
            return self._seq, self._latest

    def packed_frame(self, after: int | None, timeout_s: float) -> bytes | None:
        seq, frame = self.wait_frame(after, timeout_s) if after is not None else self.latest()
        if frame is None:
            return None
        meta = {"fps": round(self.fps(), 2), "mask_seq": self.mask_seq}
        return pack_rgbd(frame, seq=seq, meta=meta)

    # -- API -------------------------------------------------------------------------

    def info(self) -> dict:
        seq, frame = self.latest()
        return {
            "ok": True,
            "camera": self.camera.describe(),
            "segmenter": getattr(self.segmenter, "name", type(self.segmenter).__name__),
            "segment_backends": list(SEGMENT_BACKENDS),
            "capture_root": str(self.store.root),
            "seq": seq,
            "fps": round(self.fps(), 2),
            "frames_read": self.frames_read,
            "uptime_s": round(time.time() - self.started_at, 1) if self.started_at else 0.0,
            "last_error": self.last_error,
            "frame": frame.summary() if frame else None,
            "has_mask": self.mask is not None,
        }

    def segment(self, x: int, y: int) -> dict:
        seq, frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame yet" + (f" ({self.last_error})" if self.last_error else ""))
        if not (0 <= x < frame.color.width and 0 <= y < frame.color.height):
            raise ValueError(f"point ({x}, {y}) outside the {frame.color.width}x{frame.color.height} frame")
        t0 = time.monotonic()
        with self._seg_lock:
            mask = self.segmenter.segment(frame, (x, y))
        return self._adopt_mask(mask, frame, seq, t0, {"x": x, "y": y})

    def nearest(self, near_ratio: float = 1.2) -> dict:
        seq, frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame yet" + (f" ({self.last_error})" if self.last_error else ""))
        if not (1.0 < near_ratio <= 3.0):
            raise ValueError("near_ratio must be in (1, 3]")
        t0 = time.monotonic()
        with self._seg_lock:
            mask = StubSegmenter().nearest_object(frame, near_ratio=near_ratio)
        return self._adopt_mask(mask, frame, seq, t0, {"nearest": near_ratio})

    def _adopt_mask(self, mask: Mask, frame: RgbdFrame, seq: int, t0: float, prompt: dict) -> dict:
        feats = extract_features(mask, frame)
        self.mask, self.mask_frame, self.mask_seq = mask, frame, seq
        self.features = feats.as_dict() if feats else None
        return {
            "ok": True,
            "seq": seq,
            "prompt": prompt,
            "segmenter": getattr(self.segmenter, "name", "?"),
            "elapsed_ms": round((time.monotonic() - t0) * 1000.0, 1),
            "area_px": mask.area,
            "mask_png_b64": base64.b64encode(mask.to_png()).decode() if mask.area else None,
            "features": self.features,
        }

    def clear(self) -> dict:
        self.mask = self.mask_frame = self.features = None
        self.mask_seq = 0
        return {"ok": True}

    def capture(self, name: str, include_mask: bool = True) -> dict:
        if include_mask and self.mask is not None and self.mask_frame is not None:
            frame, mask, feats = self.mask_frame, self.mask, self.features
        else:
            _seq, frame = self.latest()
            mask, feats = None, None
        if frame is None:
            raise RuntimeError("no frame to capture" + (f" ({self.last_error})" if self.last_error else ""))
        result = self.store.save(
            name, frame, mask=mask, features=feats, device=self.camera.describe().get("device", {})
        )
        result["with_mask"] = mask is not None
        return result

    def captures(self) -> dict:
        return {"ok": True, "root": str(self.store.root), "sets": self.store.list()}


class ViewerHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "perception-gui"

    @property
    def app(self) -> ViewerApp:
        return self.server.app  # type: ignore[attr-defined]

    def log_message(self, fmt, *args):  # quiet
        pass

    def _send(self, body: bytes, ctype: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, obj: dict, status: int = 200) -> None:
        self._send(json.dumps(obj, default=str).encode(), "application/json", status)

    def _guarded(self, fn) -> None:
        try:
            self._send_json(fn())
        except (ValueError, KeyError, TypeError) as exc:
            self._send_json({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            self._send_json({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, status=500)

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length > 65536:
            raise ValueError("request body too large")
        raw = self.rfile.read(length) if length else b""
        payload = json.loads(raw or b"{}")
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def do_GET(self) -> None:  # noqa: N802
        url = urlparse(self.path)
        route = url.path.rstrip("/") or "/"
        qs = parse_qs(url.query)
        if route == "/":
            self._send(_WEBUI.read_bytes(), "text/html; charset=utf-8")
        elif route == "/api/info":
            self._guarded(self.app.info)
        elif route == "/api/captures":
            self._guarded(self.app.captures)
        elif route == "/api/rgbd":
            try:
                after = int(qs["after"][0]) if "after" in qs else None
                timeout_ms = min(10000, max(0, int(qs.get("timeout_ms", ["1500"])[0])))
            except ValueError:
                self._send_json({"ok": False, "error": "after/timeout_ms must be integers"}, status=400)
                return
            blob = self.app.packed_frame(after, timeout_ms / 1000.0)
            if blob is None:
                self._send_json(
                    {"ok": False, "error": "no frame yet", "last_error": self.app.last_error}, status=503
                )
            else:
                self._send(blob, "application/octet-stream")
        else:
            self._send_json({"ok": False, "error": f"no route {route}"}, status=404)

    def do_POST(self) -> None:  # noqa: N802
        route = urlparse(self.path).path.rstrip("/")
        try:
            payload = self._body()
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json({"ok": False, "error": f"bad request: {exc}"}, status=400)
            return
        if route == "/api/segment":
            self._guarded(lambda: self.app.segment(_int(payload, "x"), _int(payload, "y")))
        elif route == "/api/nearest":
            self._guarded(lambda: self.app.nearest(float(payload.get("near_ratio", 1.2))))
        elif route == "/api/capture":
            self._guarded(
                lambda: self.app.capture(
                    str(payload.get("name", "object")), bool(payload.get("include_mask", True))
                )
            )
        elif route == "/api/clear":
            self._guarded(self.app.clear)
        else:
            self._send_json({"ok": False, "error": f"no route {route}"}, status=404)


def _int(payload: dict, key: str) -> int:
    v = payload.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or v in (float("inf"), float("-inf")):
        raise ValueError(f"{key} must be a finite number")
    return int(v)


def serve(
    camera: RgbdCamera,
    *,
    config: PerceptionConfig | None = None,
    capture_root: str = DEFAULT_CAPTURE_ROOT,
    bind: str = "127.0.0.1",
    port: int = DEFAULT_PORT,
    open_browser: bool = True,
) -> None:
    """Run the cockpit until interrupted (the ``perception gui`` entry point)."""
    app = ViewerApp(camera, config=config, store=CaptureStore(Path(capture_root)))
    server = ThreadingHTTPServer((bind, port), ViewerHandler)
    server.daemon_threads = True
    server.app = app  # type: ignore[attr-defined]
    host = "127.0.0.1" if bind in ("0.0.0.0", "") else bind
    url = f"http://{host}:{server.server_address[1]}/"
    kind = camera.describe()["kind"]
    print(f"perception gui -> {url}   (camera: {kind}, captures: {capture_root}, Ctrl-C to stop)")
    if bind not in ("127.0.0.1", "localhost", "::1"):
        print(f"WARNING: bound to {bind} with no authentication — only do this on a trusted cell network.")
    app.start()
    if open_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.stop()


def add_camera_args(ap) -> None:
    """The camera/viewer flags shared by ``perception gui`` and ``perception-gui``."""
    ap.add_argument("--fake", action="store_true", help="synthetic RGB-D scene instead of a RealSense")
    ap.add_argument(
        "--serial", default=None, help="RealSense serial (default: $PERCEPTION_RS_SERIAL or first)"
    )
    ap.add_argument("--rs-fps", type=int, default=None, help="RealSense stream fps (default: 30)")
    ap.add_argument("--no-align", action="store_true", help="don't align depth to the color image")
    ap.add_argument("--library", default=None, help="path to librealsense2 (default: $REALSENSE_LIB / auto)")


def camera_from_args(args, config: PerceptionConfig) -> RgbdCamera:
    return open_camera(
        fake=bool(getattr(args, "fake", False)),
        width=config.width,
        height=config.height,
        fps=args.rs_fps if getattr(args, "rs_fps", None) else config.rs_fps,
        serial=(args.serial if getattr(args, "serial", None) else config.rs_serial) or None,
        align=not getattr(args, "no_align", False),
        library=getattr(args, "library", None),
    )


def main(argv: list[str] | None = None) -> int:
    """Standalone ``perception-gui`` console script."""
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    import argparse

    ap = argparse.ArgumentParser(
        prog="perception-gui", description="local RGB-D cockpit for a RealSense camera"
    )
    add_camera_args(ap)
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument(
        "--segment-backend", default=None, help="stub | sam (default: $PERCEPTION_SEGMENT_BACKEND)"
    )
    ap.add_argument(
        "--out", default=DEFAULT_CAPTURE_ROOT, help=f"capture root (default: {DEFAULT_CAPTURE_ROOT}/)"
    )
    ap.add_argument("--bind", default="127.0.0.1", help="interface to bind (default: loopback only)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"port (default {DEFAULT_PORT})")
    ap.add_argument("--no-browser", action="store_true", help="don't open the browser automatically")
    args = ap.parse_args(argv)
    overrides: dict[str, object] = {}
    if args.width is not None:
        overrides["width"] = args.width
    if args.height is not None:
        overrides["height"] = args.height
    if args.segment_backend is not None:
        overrides["segment_backend"] = args.segment_backend
    config = PerceptionConfig.from_env(**overrides)
    serve(
        camera_from_args(args, config),
        config=config,
        capture_root=args.out,
        bind=args.bind,
        port=args.port,
        open_browser=not args.no_browser,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
