"""perceptronics-gui — a local RGB-D cockpit for a RealSense (or the synthetic
stand-in): live color + depth, hover-to-measure, click-to-segment, and one-key
capture into the RealSenseTrainer dataset layout.

Zero-dependency (stdlib ``http.server``), same shape as ``urctl gui``: a
single-file page in ``perceptronics/webui/index.html`` over a small API.

    python3 -m perceptronics gui --fake                 # no camera: synthetic scene
    sudo python3 -m perceptronics gui                   # the D435 (macOS needs root)
    python3 -m perceptronics gui --bind 0.0.0.0         # serve off-box (Jetson → laptop)

API:

  * ``GET  /api/info``                camera description, config, backends.
  * ``GET  /api/rgbd?after=N``        the newest frame as one binary container
                                      (header JSON + color PNG + zlib'd uint16
                                      depth — see :func:`perceptronics.rgbd.pack_rgbd`).
                                      With ``after``, long-polls (≤ ``timeout_ms``)
                                      until a frame newer than ``N`` exists, so
                                      the page never re-decodes a duplicate and
                                      never outruns the camera.
  * ``POST /api/segment``  ``{x, y}`` segment the object under a pixel of the
                                      latest frame → mask PNG (base64) + features.
  * ``POST /api/nearest``             RealSenseTrainer's nearest-object mask.
  * ``GET  /api/robot``               the robot link: host, hand-eye, dry-run.
  * ``POST /api/robot/state``         ``get_state`` through the tool registry.
  * ``POST /api/robot/approach_cycle`` ``{standoff_m?, reference?, clearance_m?, hold_s?, velocity?}``
    — over the segment → down to the standoff → hold → up → back to the capture pose (one program)
  * ``POST /api/robot/locate``        ``{standoff_m?, point_m?, reference?}`` — the segment's
                                      camera point → base frame + approach pose
                                      (reads the live flange pose; no motion).
  * ``POST /api/robot/move``          ``{pose, velocity?}`` — safety-validated
                                      ``movel`` to that approach pose.
  * ``POST /api/robot/jog``           ``{delta:[dx,dy,dz,drx,dry,drz], velocity?}`` one
                                      relative base-frame nudge (≤ 5 cm / 0.35 rad per axis).
  * ``POST /api/robot/bring_up`` / ``/api/robot/stop`` / ``/api/robot/freedrive`` ``{enable}``
  * ``POST /api/robot/gripper``       ``{action: status|open|close|move|activate, position?}``
                                      the Robotiq gripper through its URCap daemon.
  * ``GET  /api/doctor``              the pre-flight report (:mod:`perceptronics.doctor`)
                                      for the robot side; the camera side is this
                                      process's own stream stats.
  * ``GET  /api/events?after=N``      the cockpit's event log (robot actions,
                                      segments, captures, camera errors) — what an
                                      agent or a human reads to see what just happened.
  * ``POST /api/snapshot`` ``{dir?, name?}`` write the latest frame as
                                      ``<name>_color.png`` + ``<name>_depth.png`` (colourised)
                                      to a directory and return the paths — the
                                      "give the agent eyes" call.
  * ``GET  /api/point?x=&y=``       the camera-frame point under a pixel (median of a
                                      5×5 window) — what a calibration view uses.
  * ``POST /api/cal/mark`` / ``/api/cal/view`` ``{x, y}`` / ``/api/cal/solve`` /
    ``/api/cal/apply`` ``{save?}`` / ``/api/cal/reset`` / ``GET /api/cal`` —
                                      touch-and-click hand-eye calibration
                                      (:mod:`perceptronics.calibrate`).
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
import math
import os
import re
import sys
import threading
import time
import traceback
import webbrowser
from collections import deque
from collections.abc import Sequence
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from urctl.pose import Transform

from .cell import describe_cell
from .config import PerceptionConfig
from .factory import SEGMENT_BACKENDS, make_segmenter
from .partspec import PartSpec, from_payload, from_query
from .picknode import (
    DEFAULT_PICK_PORT,
    PickPlanner,
    PickServer,
    detect_report,
    parse_options,
    parse_preview_request,
    scene_report,
)
from .picknode import preview as pick_preview
from .pngio import encode_png
from .posestream import PoseStream
from .realsense import (
    DEFAULT_DEPTH_FILTERS,
    LASER_MAX,
    VISUAL_PRESETS,
    DepthTuning,
    RealSenseError,
    RgbdCamera,
    open_camera,
    platform_hint,
)
from .rgbd import RgbdFrame, pack_rgbd
from .robotlink import RobotLink, reach_note
from .segment import Mask, StubSegmenter, extract_features, normalize_box
from .views import ViewSource, open_views, parse_view_size, parse_view_specs

DEFAULT_PORT = 7621
DEFAULT_SNAPSHOT_DIR = "captures/snapshots"
REOPEN_DELAY_S = 1.0
REOPEN_MAX_DELAY_S = 30.0
STALL_AFTER_S = 2.0  # no new frame for this long: the stream is stalled, its rate is 0
EVENT_LOG_SIZE = 500


def validate_name(name: str) -> str:
    """A snapshot/set name: 1-64 chars of [A-Za-z0-9_-] (no paths, no dots)."""
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", name or ""):
        raise ValueError("name must be 1-64 characters of letters, digits, _ or -")
    return name


class EventLog:
    """A bounded, numbered log of what the cockpit did — one line per robot
    action, segment, capture, or camera error. ``/api/events?after=N`` tails
    it, the page shows it, and the ``cam_events`` MCP tool reads it, so a
    human and an agent looking at the same cockpit see the same history."""

    def __init__(self, size: int = EVENT_LOG_SIZE):
        self._items: deque[dict] = deque(maxlen=size)
        self._seq = 0
        self._lock = threading.Lock()

    def add(self, kind: str, message: str, *, ok: bool | None = None, data: dict | None = None) -> dict:
        with self._lock:
            self._seq += 1
            item = {"seq": self._seq, "ts": time.time(), "kind": kind, "ok": ok, "message": message}
            if data:
                item["data"] = data
            self._items.append(item)
            return item

    def since(self, after: int = 0, limit: int = 200) -> list[dict]:
        with self._lock:
            return [i for i in self._items if i["seq"] > after][-limit:]

    @property
    def seq(self) -> int:
        return self._seq


def recent_rate(stamps: list[float]) -> float:
    """Events per second among the monotonic ``stamps`` of the last :data:`STALL_AFTER_S`
    seconds — 0 when they have stopped coming."""
    now = time.monotonic()
    w = [t for t in stamps if now - t <= STALL_AFTER_S]
    return (len(w) - 1) / (w[-1] - w[0]) if len(w) > 1 and w[-1] > w[0] else 0.0


def reopen_delay(failures: int) -> float:
    """Seconds to wait before the ``failures``-th consecutive reopen (1-based):
    1, 2, 4, … capped at :data:`REOPEN_MAX_DELAY_S`.

    Every failed open on macOS's libusb backend resets the USB device to try
    to capture it, and each reset re-runs the race against the OS camera
    driver — a tight retry loop just resets the camera dozens of times and
    stalls its control pipe (seen: 36 re-enumerations in nine minutes at a
    fixed 1 s delay), so back off instead of hammering it.
    """
    return min(REOPEN_DELAY_S * 2 ** max(0, failures - 1), REOPEN_MAX_DELAY_S)


_WEBUI = Path(__file__).parent / "webui" / "index.html"
_CLASSIC = Path(__file__).parent / "webui" / "classic.html"  # every control, the pre-2026-09-27 page


class ViewPump:
    """One extra viewpoint (:mod:`perceptronics.views`) behind its own thread:
    the newest encoded image, a sequence number for long-polling, fps, and
    the same open → read → back-off-on-failure loop as the RGB-D pump."""

    def __init__(self, source: ViewSource, index: int, events: EventLog):
        self.source = source
        self.index = index
        self.events = events
        self._cond = threading.Condition()
        self._latest: bytes | None = None
        self._seq = 0
        self._running = False
        self._thread: threading.Thread | None = None
        self.last_error: str | None = None
        self._last_logged_error: str | None = None
        self.frames_read = 0
        self._fps_window: list[float] = []

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._pump, name=f"view-pump-{self.index}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        try:
            self.source.close()
        except Exception:
            pass

    def _pump(self) -> None:
        opened = False
        failures = 0
        label = f"view {self.index} ({self.source.name})"
        while self._running:
            try:
                if not opened:
                    self.source.open()
                    opened = True
                    failures = 0
                    self.last_error = None
                    self._last_logged_error = None
                    desc = self.source.describe()
                    self.events.add(
                        "view",
                        f"{label}: opened {desc.get('kind')} {desc.get('device') or ''}".strip(),
                        ok=True,
                    )
                data = self.source.read()
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                if self.last_error != self._last_logged_error:
                    self.events.add("view", f"{label}: {self.last_error}", ok=False)
                    self._last_logged_error = self.last_error
                try:
                    self.source.close()
                except Exception:
                    pass
                opened = False
                failures += 1
                with self._cond:
                    self._cond.wait(reopen_delay(failures))
                continue
            now = time.monotonic()
            with self._cond:
                self._latest = data
                self._seq += 1
                self.frames_read += 1
                self._fps_window.append(now)
                del self._fps_window[:-30]
                self._cond.notify_all()

    def fps(self) -> float:
        return recent_rate(self._fps_window)

    def latest(self) -> tuple[int, bytes | None]:
        with self._cond:
            return self._seq, self._latest

    def wait_frame(self, after: int | None, timeout_s: float) -> tuple[int, bytes | None]:
        """Newest image with seq > ``after`` (blocks ≤ ``timeout_s``); else the newest."""
        if after is None:
            return self.latest()
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._running and self._seq <= after:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(remaining)
            return self._seq, self._latest

    def describe(self) -> dict:
        seq, data = self.latest()
        return {
            **self.source.describe(),
            "index": self.index,
            "seq": seq,
            "fps": round(self.fps(), 2),
            "frames_read": self.frames_read,
            "last_error": self.last_error,
            "bytes": len(data) if data else 0,
        }


class ViewerApp:
    """State behind the handlers: one camera, one pump thread, one segmenter —
    plus one :class:`ViewPump` per extra viewpoint (``views``)."""

    def __init__(
        self,
        camera: RgbdCamera,
        *,
        config: PerceptionConfig | None = None,
        segmenter=None,
        robot: RobotLink | None = None,
        views: list[ViewSource] | None = None,
        cors: Sequence[str] | None = None,
        pose_stream: PoseStream | None = None,
    ):
        self.camera = camera
        self.robot = robot
        # The live flange pose (RTDE), so each frame carries the pose it was taken at.
        self.pose_stream = pose_stream
        self._latest_t = 0.0  # host time the newest frame arrived
        self.mask_t = 0.0  # ... and the one the current mask was cut from
        # Origins allowed to call the API from another page (a PolyScope X URCap on
        # the pendant, `urcap/perceptronic`). Empty = same-origin only (the default).
        self.cors_origins = []
        for o in (o.strip() for o in (cors or []) if o and o.strip()):
            if o == "*" or _ORIGIN_RE.match(o):
                self.cors_origins.append(o)
            else:  # never matches a browser's Origin, and must never reach a header
                shown = repr(o[:200])[1:-1]
                print(f"CORS: ignoring {shown} — not an origin (scheme://host[:port])", file=sys.stderr)
        # Cross-origin pages the cockpit turned away — the browser only says "Failed to
        # fetch", so the cockpit names the origin to add (stderr once, and /api/info).
        self.cors_refused: set[str] = set()
        self.config = config or PerceptionConfig.from_env()
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
        self.events = EventLog()
        self.views = [ViewPump(v, i, self.events) for i, v in enumerate(views or [])]
        self._last_logged_error: str | None = None
        self.pick_port: int | None = None  # the robot program's pick server (serve(pick_port=…))

    # -- lifecycle -------------------------------------------------------------

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self.started_at = time.time()
        self._thread = threading.Thread(target=self._pump, name="rgbd-pump", daemon=True)
        self._thread.start()
        for view in self.views:
            view.start()

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
        for view in self.views:
            view.stop()
        if self.pose_stream is not None:
            self.pose_stream.stop()

    def view_frame(self, index: int, after: int | None, timeout_s: float) -> tuple[int, bytes | None, str]:
        """``(seq, image bytes | None, content type)`` of view ``index`` (long-poll
        semantics as :meth:`packed_frame`); ``IndexError`` for an unknown view."""
        pump = self.views[index]  # IndexError → 404
        seq, data = pump.wait_frame(after, timeout_s)
        return seq, data, pump.source.content_type

    def _pump(self) -> None:
        opened = False
        failures = 0
        while self._running:
            try:
                if not opened:
                    self.camera.open()
                    opened = True
                    failures = 0
                    self.last_error = None
                    self._last_logged_error = None
                    desc = self.camera.describe()
                    dev = desc.get("device") or {}
                    self.events.add(
                        "camera", f"opened {desc.get('kind')} {dev.get('name') or ''}".strip(), ok=True
                    )
                    if self.robot is not None:
                        self.robot.attach_camera(desc)  # depth→colour extrinsics
                frame = self.camera.read()
            except Exception as exc:
                hint = platform_hint(exc) if isinstance(exc, RealSenseError) else ""
                self.last_error = f"{type(exc).__name__}: {exc}" + (f" — {hint}" if hint else "")
                if self.last_error != self._last_logged_error:  # once per distinct failure, not per retry
                    self.events.add("camera", self.last_error, ok=False)
                    self._last_logged_error = self.last_error
                try:
                    self.camera.close()
                except Exception:
                    pass
                opened = False
                failures += 1
                # Back off, then try again (camera unplugged / permission fixed / re-plugged).
                with self._cond:
                    self._cond.wait(reopen_delay(failures))
                continue
            now = time.monotonic()
            with self._cond:
                self._latest = frame
                self._latest_t = time.time()
                self._seq += 1
                self.frames_read += 1
                self._fps_window.append(now)
                self._fps_window = [t for t in self._fps_window if now - t <= 2.0]
                self._cond.notify_all()

    # -- frames ----------------------------------------------------------------------

    def fps(self) -> float:
        """Frames per second over the last 2 s — 0 once the frames stop. The window is only
        trimmed when a frame arrives, so a camera that dropped out would otherwise keep
        its last rate for ever."""
        return recent_rate(self._fps_window)

    def frame_age_s(self) -> float | None:
        """Seconds since the newest frame arrived (None before the first)."""
        t = self._latest_t
        return max(0.0, time.time() - t) if t else None

    def stalled(self) -> bool:
        """Frames were flowing and have stopped: the picture on screen is old."""
        age = self.frame_age_s()
        return self.frames_read > 0 and age is not None and age > STALL_AFTER_S

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

    def wait_frame_stamped(self, after: int, timeout_s: float) -> tuple[int, RgbdFrame | None, float]:
        """:meth:`wait_frame` plus the frame's host stamp, read under the same lock."""
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
        meta = {"fps": round(self.fps(), 2), "mask_seq": self.mask_seq, **self.frame_pose()}
        return pack_rgbd(frame, seq=seq, meta=meta)

    def frame_pose(self) -> dict:
        """``flange_pose`` (base frame, at the newest frame's arrival), ``pose_age_s``
        and the hand-eye ``flange_to_color_pose`` — what a page needs for
        ``T_base_colour`` — or ``{}`` without a live pose."""
        if self.pose_stream is None:
            return {}
        with self._cond:
            t = self._latest_t
        flange, q, age = self.pose_stream.sample_at(t)
        if flange is None:
            return {}
        out: dict = {"flange_pose": [round(v, 6) for v in flange], "pose_age_s": round(age or 0.0, 3)}
        if self.robot is not None:
            out["flange_to_color_pose"] = self.robot.handeye.as_dict().get("flange_to_color_pose")
        arm = self._arm(q, flange)
        if arm:
            out["arm"] = arm
        return out

    def _arm(self, q: list[float] | None, flange: list[float]) -> dict | None:
        """The linkage at joints ``q`` (``armfk``) — only when its FK flange agrees
        with the measured one, so a wrong DH table or model never draws a wrong arm."""
        from . import armfk

        model = armfk.model_key(os.environ.get("UR_ROBOT_MODEL"))
        if not q or model is None:
            return None
        chain = armfk.frames(q, model)
        err = armfk.flange_error_m(chain, flange)
        if err > armfk.FLANGE_TOLERANCE_M:
            return None
        return {
            "model": model,
            "frames": [[round(v, 5) for v in f] for f in chain],
            "fk_error_mm": round(err * 1000, 2),
        }

    def _flange_at(self, t: float | None) -> list[float] | None:
        """The flange pose at host time ``t`` (the pose stream), else the live one."""
        if self.pose_stream is not None and t:
            pose, _ = self.pose_stream.at(t, max_age_s=0.5)
            if pose is not None:
                return pose
        fp = self.robot.flange_pose() if self.robot is not None else {}
        return list(fp["flange"]) if fp.get("ok") and fp.get("flange") else None

    def _handeye_fc(self) -> list[float]:
        return list(self.robot.handeye.as_dict()["flange_to_color_pose"])

    def _rect_from_mask(self, mask: Mask, frame: RgbdFrame, flange: Sequence[float]) -> dict | None:
        """The masked object's top face in the base frame (``pickplan.rectangle``).

        The height comes from the depth (the top face, ``pickplan.top_face``), the
        outline from the colour mask: white foam leaves holes in the depth — on the UR3e
        the depth-bearing points of a 27 mm block formed a 10 mm strip (2026-09-27) — so
        every mask pixel that is on the top face or a hole in it is cast along its ray
        onto the horizontal plane at the top's height (the table is flat and level).
        Pixels whose depth says side or floor are left out."""
        from . import pickplan

        T = Transform.from_pose(flange).compose(Transform.from_pose(self._handeye_fc()))
        k, w = frame.intrinsics, mask.width
        pix = [(i % w, i // w) for i in mask.pixels()]
        with_depth = []
        for u, v in pix[:: max(1, len(pix) // 4000)]:
            d = frame.depth.distance_m(u, v)
            if d:
                with_depth.append(T.apply(((u - k.ppx) * d / k.fx, (v - k.ppy) * d / k.fy, d)))
        top = pickplan.top_face(with_depth)
        if len(top) < 10:
            return None
        top_z = sorted(p[2] for p in top)[len(top) // 2]
        o = T.translation
        pts = []
        for u, v in pix:
            d = frame.depth.distance_m(u, v)
            if d:
                q = T.apply(((u - k.ppx) * d / k.fx, (v - k.ppy) * d / k.fy, d))
                if abs(q[2] - top_z) > 0.006:
                    continue  # a side or the floor
            ray = T.rotate(((u - k.ppx) / k.fx, (v - k.ppy) / k.fy, 1.0))
            if ray[2] >= -1e-6:
                continue
            t = (top_z - o[2]) / ray[2]
            pts.append((o[0] + t * ray[0], o[1] + t * ray[1], top_z))
        return pickplan.rectangle(pts)

    def _pixel_of(
        self, point: Sequence[float], flange: Sequence[float], frame: RgbdFrame
    ) -> tuple[int, int] | None:
        """Where a base-frame point lands in ``frame`` taken from ``flange``."""
        T = Transform.from_pose(flange).compose(Transform.from_pose(self._handeye_fc()))
        q = T.inverse().apply(point)
        if q[2] <= 0.05:
            return None
        k = frame.intrinsics
        u, v = round(k.fx * q[0] / q[2] + k.ppx), round(k.fy * q[1] / q[2] + k.ppy)
        return (u, v) if 0 <= u < frame.color.width and 0 <= v < frame.color.height else None

    def _refind(
        self, near: Sequence[float], look: Sequence[float], gate_m: float = 0.08
    ) -> tuple[dict | None, list[float], str]:
        """From the look pose: a settled frame, every block in it (the white-block
        detector), the one nearest ``near`` within ``gate_m`` — segmented at its own
        pixel and measured (``_rect_from_mask``). ``(rect, flange, why-not)``."""
        from . import pickplan

        time.sleep(0.1)
        seq, _ = self.latest()
        self.wait_frame(seq, 1.0)
        objs = self.objects()
        f_look = objs.get("flange_pose") or self._flange_at(time.time()) or list(look)
        T = Transform.from_pose(f_look).compose(Transform.from_pose(self._handeye_fc()))
        best = None
        o_cam = T.translation
        for o in objs.get("objects") or []:
            top = pickplan.top_face([T.apply(q) for q in o["points_cam"]])
            if len(top) < 10:
                continue
            # cast every detected pixel along its ray onto the (horizontal) top plane — exact
            # whatever the camera's tilt; the detector's points all sit at one camera depth
            top_z = sorted(q[2] for q in top)[len(top) // 2]
            cast = []
            for q in o["points_cam"]:
                ray = T.rotate(q)
                if ray[2] < -1e-9:
                    t = (top_z - o_cam[2]) / ray[2]
                    cast.append((o_cam[0] + t * ray[0], o_cam[1] + t * ray[1], top_z))
            r = pickplan.rectangle(cast)
            if r is None:
                continue
            d = math.dist(r["centre"][:2], near[:2])
            if d <= gate_m and (best is None or d < best[0]):
                best = (d, o, r)
        if best is None:
            n = len(objs.get("objects") or [])
            return None, f_look, f"{n} block(s) in view, none within {gate_m * 1000:.0f} mm of the estimate"
        _, o, r = best
        # programmatic, not semantic: the detector's own top face (every white pixel cast at the
        # top's depth) is the measurement — no second segmentation pass
        return r, f_look, ""

    def _scene_base(self, step: int = 3) -> list[tuple[float, float, float]]:
        """The newest depth frame as base-frame points (every ``step``-th pixel)."""
        seq, frame = self.latest()
        with self._cond:
            t = self._latest_t
        flange = self._flange_at(t)
        if frame is None or flange is None:
            return []
        T = Transform.from_pose(flange).compose(Transform.from_pose(self._handeye_fc()))
        k, d = frame.intrinsics, frame.depth
        out = []
        for v in range(0, d.height, step):
            for u in range(0, d.width, step):
                z = d.distance_m(u, v)
                if z and z < 2.0:
                    out.append(T.apply(((u - k.ppx) * z / k.fx, (v - k.ppy) * z / k.fy, z)))
        return out

    def _unreachable(self, legs: list[dict]) -> list[str]:
        iks = self.robot.robot.inverse_kin([leg["pose"] for leg in legs], tcp=[0.0] * 6)
        return [leg["name"] for leg, a in zip(legs, iks, strict=True) if a.get("reachable") is False]

    def _run(self, legs: list[dict], gripper_first: int | None = None) -> dict:
        params: dict = {
            "legs": [{k: v for k, v in leg.items() if k != "name"} for leg in legs],
            "tcp": [0.0] * 6,
        }
        if gripper_first is not None:
            params["gripper_first"] = int(gripper_first)  # the fingers travel while the arm does
        return self.robot._tool("move_tcp_path", params)

    def pick(
        self,
        *,
        pick: bool = False,
        fancy: bool = False,
        plan_only: bool = False,
        look: bool = True,
        survey: bool = False,
        target: dict | None = None,
    ) -> dict:
        """Sweep to the clicked object and hover over it (``pickplan``), in two programs:

        1. a blended sweep (``fancy``: with a swing and a flourish) to the **close look**
           — the object on the camera's axis 0.24 m off, as near as the D435 sees;
        2. re-find the object there (its first estimate projected into the new
           picture, segmented again), re-measure its top face, and run the final
           program from that: straight down the base Z axis, fingers across its short
           side opened to 1.2x its width, fingertips 25 mm over its top; with ``pick``
           down 15 mm under the top, close, lift.

        No close look when ``look`` is off or its pose is out of reach (one program).
        ``target`` — a stored object ``{centre [x y z], theta, major_m, minor_m}`` in the
        base frame — stands in for the clicked mask: no segmentation, and the object need
        not be in view. ``survey`` stops after the close look and returns its measurement."""
        from . import pickplan
        from .handeye import tip_m_from_env
        from .pickcycle import grasp_rotation, grasp_yaw_deg

        if self.robot is None:
            raise ValueError("no robot link (started with --no-robot)")
        f_now = self._flange_at(time.time())
        if f_now is None:
            return {"ok": False, "error": "no flange pose (pose stream or controller)"}
        if target is not None:
            try:
                c = [float(v) for v in target["centre"]]
                rect0 = {
                    "centre": c,
                    "theta": float(target.get("theta", 0.0)),
                    "major_m": float(target["major_m"]),
                    "minor_m": float(target["minor_m"]),
                }
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("target needs centre [x y z], major_m, minor_m (theta optional)") from exc
            if (
                len(c) != 3
                or not all(math.isfinite(v) for v in c)
                or not (0 < rect0["minor_m"] <= rect0["major_m"] < 0.5)
            ):
                raise ValueError("target: centre must be 3 finite numbers, 0 < minor_m <= major_m < 0.5 m")
        else:
            with self._seg_lock:
                mask, frame, mask_t = self.mask, self.mask_frame, self.mask_t
            if mask is None or frame is None or not mask.area:
                raise ValueError("no target: click an object first")
            f_then = self._flange_at(mask_t)
            if f_then is None:
                return {"ok": False, "error": "no flange pose for the target's frame"}
            rect0 = self._rect_from_mask(mask, frame, f_then)
            if rect0 is None:
                return {"ok": False, "error": "not enough depth on the target's top face"}
        tip, hfc = tip_m_from_env(), self._handeye_fc()
        notes: list[str] = []

        def build(rect, start, *, with_look, skip=()):
            return pickplan.plan(
                rect,
                start,
                tip_m=tip,
                pick=pick,
                fancy=fancy,
                skip=skip,
                flange_to_color=hfc if with_look else None,
                look_m=pickplan.LOOK_M if with_look else None,
                home=self._home_pose(),
            )

        p = build(rect0, f_now, with_look=look)
        if pick and not p["fits"]:
            return {
                "ok": False,
                "error": f"{rect0['minor_m'] * 1000:.0f} mm is too wide for the fingers",
                "rect": rect0,
            }
        bad = self._unreachable(p["legs"])
        fancy_bad = [b for b in bad if b in ("swing", "flourish")]
        if "look" in bad:
            notes.append("close look out of reach: approaching from the first estimate")
            p = build(rect0, f_now, with_look=False, skip=fancy_bad)
            bad = self._unreachable(p["legs"])
        elif fancy_bad:
            p = build(rect0, f_now, with_look=look, skip=fancy_bad)
            bad = self._unreachable(p["legs"])
        summary: dict = {
            "rect": {**rect0, "theta_deg": round(math.degrees(rect0["theta"]), 1)},
            "yaw_deg": round(p["yaw_deg"], 1),
            "opening_mm": round(p["opening_m"] * 1000, 1),
            "gripper_position": p["gripper_position"],
            "vias": p["vias"],
            "look": p["look"],
            "dropped": fancy_bad,
            "notes": notes,
            "legs": [{"name": leg["name"], "pose": [round(v, 4) for v in leg["pose"]]} for leg in p["legs"]],
        }
        if bad:
            return {"ok": False, "error": f"out of reach: {', '.join(bad)}", **summary}
        if plan_only:
            return {"ok": True, "plan_only": True, **summary}
        run = self._run(p["sweep"], gripper_first=p["gripper_position"])
        if not run.get("ok"):
            return {
                "ok": False,
                "error": run.get("error") or "sweep stopped",
                "stage": "sweep",
                "dry_run": run.get("dry_run"),
                **summary,
            }
        rect, final = rect0, p["final"]
        if p["look"] is not None:
            # 2 — find the block again from up close, by identity (the real block nearest the
            # estimate), re-centre once if it is off the camera's axis, then measure it there
            rect1, f_look, why_not = self._refind(rect0["centre"], p["look"])
            # re-centre only when the block sits far off the camera's axis (the look measures well
            # anywhere near the middle of the picture): one move and one look saved most times
            if rect1 is not None and math.dist(rect1["centre"][:2], rect0["centre"][:2]) > 0.04:
                rot = grasp_rotation(f_look, rect1["centre"], 0.0)
                yaw = grasp_yaw_deg(rot, rect1["theta"] + math.pi / 2, "y")
                look2 = pickplan.look_pose(rect1, rot, yaw, hfc, pickplan.LOOK_M)
                if not self._unreachable([{"name": "look", "pose": look2}]):
                    self._run([{"name": "look", "pose": look2, **pickplan.SETTLE, "dwell_s": 0.15}])
                    again, f2, _ = self._refind(rect1["centre"], look2)
                    if again is not None:
                        d1 = math.dist(rect1["centre"][:2], rect0["centre"][:2]) * 1000
                        d2 = math.dist(again["centre"][:2], rect1["centre"][:2]) * 1000
                        notes.append(
                            f"close look: {d1:.0f} mm from the first estimate; re-centred, {d2:.0f} mm more"
                        )
                        rect1, f_look = again, f2
            if rect1 is None:
                self.events.add("robot", f"pick: lost the target at the close look ({why_not})", ok=False)
                return {
                    "ok": False,
                    "error": f"lost the target at the close look: {why_not}",
                    "stage": "look",
                    **summary,
                }
            rect = rect1
            if survey:  # a survey ends at the look: the fresh measurement, nothing more
                summary.update(rect={**rect1, "theta_deg": round(math.degrees(rect1["theta"]), 1)})
                self.events.add("robot", f"survey → {[round(v, 3) for v in rect1['centre']]}", ok=True)
                return {"ok": True, "survey": True, "notes": notes, **summary}
            notes.append(
                f"close look: {rect1['major_m'] * 1000:.0f} x {rect1['minor_m'] * 1000:.0f} mm at "
                f"{[round(v, 3) for v in rect1['centre']]}"
            )
            # room for the open fingers beside it? checked on the close look's own depth, before committing
            room = pickplan.clearance(rect, self._scene_base())
            summary["clearance"] = room
            if pick and not room["clear"]:
                side = "+" if room["side"] == "+" else "−"
                why_blocked = (
                    f"no room for the {side} finger: {room['count']} points up to "
                    f"{room['worst_mm']:.0f} mm above the fingertips beside it"
                )
                self.events.add("robot", f"pick: {why_blocked}", ok=False)
                return {"ok": False, "error": why_blocked, "stage": "clearance", **summary}
            p2 = pickplan.plan(rect, f_look, tip_m=tip, pick=pick, fancy=False, home=self._home_pose())
            if pick and not p2["fits"]:
                return {
                    "ok": False,
                    "error": f"{rect['minor_m'] * 1000:.0f} mm is too wide for the fingers",
                    **summary,
                }
            final = p2["sweep"]  # no look: over → approach [→ grasp → lift], from the look pose
            bad = self._unreachable(final)
            if bad:
                return {
                    "ok": False,
                    "error": f"out of reach after the close look: {', '.join(bad)}",
                    **summary,
                }
            regrip = (
                p2["gripper_position"] if abs(p2["gripper_position"] - p["gripper_position"]) > 6 else None
            )
            summary.update(
                rect={**rect, "theta_deg": round(math.degrees(rect["theta"]), 1)},
                yaw_deg=round(p2["yaw_deg"], 1),
                opening_mm=round(p2["opening_m"] * 1000, 1),
                gripper_position=p2["gripper_position"],
            )
            run = self._run(final, gripper_first=regrip)
        held = None
        for leg in run.get("legs") or []:
            gr = leg.get("gripper") if isinstance(leg, dict) else None
            if isinstance(gr, dict) and gr.get("action") == "close":
                held = bool(gr.get("object_detected"))
        # with a HOME pose the pick's program ends there itself (clear blends into it)
        homed = bool(run.get("ok")) if pick and self._home_pose() is not None else None
        self.events.add(
            "robot",
            f"{'pick' if pick else 'approach'}{' (fancy)' if fancy else ''} → "
            f"{[round(v, 3) for v in rect['centre']]}" + (f", held {held}" if pick else ""),
            ok=bool(run.get("ok")),
        )
        return {
            "ok": bool(run.get("ok")),
            "error": run.get("error"),
            "completed_legs": run.get("completed_legs"),
            "held": held,
            "homed": homed,
            "dry_run": run.get("dry_run"),
            "protective_stop": run.get("protective_stop"),
            **summary,
        }

    def _home_pose(self) -> list[float] | None:
        raw = os.environ.get("PERCEPTRONICS_HOME_POSE", "").strip()
        try:
            pose = [float(v) for v in raw.strip("[]").split(",")] if raw else []
        except ValueError:
            return None
        return pose if len(pose) == 6 else None

    def home(self) -> dict:
        """One clean movel back to the cell's picture pose (``PERCEPTRONICS_HOME_POSE``,
        a flange pose looking down at the work surface)."""
        from . import pickplan

        if self.robot is None:
            raise ValueError("no robot link (started with --no-robot)")
        raw = os.environ.get("PERCEPTRONICS_HOME_POSE", "").strip()
        if not raw:
            raise ValueError("no PERCEPTRONICS_HOME_POSE in the cell")
        pose = [float(v) for v in raw.strip("[]").split(",")]
        if len(pose) != 6:
            raise ValueError("PERCEPTRONICS_HOME_POSE must be six numbers")
        run = self.robot._tool(
            "move_tcp_path",
            {"legs": [{"pose": pose, **pickplan.TRANSIT}], "tcp": [0.0] * 6},
        )
        self.events.add("robot", "home", ok=bool(run.get("ok")))
        return {
            "ok": bool(run.get("ok")),
            "error": run.get("error"),
            "pose": pose,
            "dry_run": run.get("dry_run"),
        }

    def objects(self) -> dict:
        """Every white block in the newest frame (pick-cycle's detector): per object
        the top-face centre and its white pixels back-projected at the top face's
        depth, camera frame (m), with the frame's pose so the page can place them."""
        from .pickcycle import WHITE_CHROMA, top_face, white_blobs, white_level

        seq, frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame yet" + (f" ({self.last_error})" if self.last_error else ""))
        c, d, k = frame.color, frame.depth, frame.intrinsics
        K = {"fx": k.fx, "fy": k.fy, "ppx": k.ppx, "ppy": k.ppy}
        w, h, ch, rgb = c.width, c.height, c.channels, c.data
        found = []
        level = white_level(w, h, ch, rgb)  # "white" relative to this picture's brightest neutrals
        for b in white_blobs(w, h, ch, rgb, white_min=level):
            x0, y0, x1, y1 = b["bbox"]
            if x0 <= 6 or y0 <= 6 or x1 >= w - 6 or y1 >= h - 6:
                continue  # clipped at the frame edge
            tf = top_face(w, h, ch, rgb, d.data, d.scale_m, K, b["bbox"], white_min=level)
            if tf is None or tf["n"] < 20:
                continue
            z = tf["centre"][2]
            pts = []
            for y in range(y0, min(y1, h), 2):
                for x in range(x0, min(x1, w), 2):
                    i = (y * w + x) * ch
                    r, g, bb = rgb[i], rgb[i + 1], rgb[i + 2]
                    if min(r, g, bb) > level and max(r, g, bb) - min(r, g, bb) < WHITE_CHROMA:
                        pts.append(
                            [round((x - k.ppx) * z / k.fx, 4), round((y - k.ppy) * z / k.fy, 4), round(z, 4)]
                        )
            step = max(1, len(pts) // 300)
            found.append(
                {
                    "centre_cam": [round(v, 4) for v in tf["centre"]],
                    "pixel": [b["cx"], b["cy"]],
                    "bbox": list(b["bbox"]),
                    "points_cam": pts[::step],
                }
            )
        return {"ok": True, "seq": seq, "objects": found, **self.frame_pose()}

    def color_png(self, after: int | None, timeout_s: float) -> tuple[int, bytes | None]:
        """The colour image alone as a PNG (``GET /api/color.png``): what a page that
        cannot inflate the RGB-D container — a URCap's <img> — polls for."""
        seq, frame = self.wait_frame(after, timeout_s) if after is not None else self.latest()
        if frame is None:
            return seq, None
        color = frame.color if frame.color.channels == 3 else frame.color.to_rgb()
        return seq, encode_png(color.width, color.height, 3, color.data)

    def depth_png(self, after: int | None, timeout_s: float) -> tuple[int, bytes | None]:
        """The depth image as a heatmap PNG (``GET /api/depth.png``): the same long-poll as
        :meth:`color_png`, for the URCap's picture/heatmap toggle. Aligned to the colour
        image and half its size each way (a pixel loop in Python: a quarter of the work).
        The ramp spans what the frame holds (:func:`heatmap_range`): a 30 mm part on a
        table 0.4 m away is a few percent of a fixed 0.15–1 m ramp — one colour."""
        seq, frame = self.wait_frame(after, timeout_s) if after is not None else self.latest()
        if frame is None:
            return seq, None
        near, far = heatmap_range(frame)
        return seq, colourise_depth_png(frame, near, far, step=2)

    # -- the robot program's pick server (perceptronics.picknode) ------------------------

    def pick_frame(self, after: int, timeout_s: float = 2.0) -> tuple | None:
        """A frame newer than ``after`` as the detector takes it:
        ``(seq, w, h, ch, rgb, depth, depth_scale_m, K)``, or None."""
        seq, frame = self.wait_frame(after, timeout_s)
        if frame is None or seq <= after:
            return None
        c = frame.color if frame.color.channels == 3 else frame.color.to_rgb()
        return (
            seq,
            c.width,
            c.height,
            3,
            c.data,
            frame.depth.data,
            frame.depth.scale_m,
            frame.intrinsics.as_dict(),
        )

    def _handeye_pose(self) -> list[float] | None:
        if self.robot is None:
            return None
        return self.robot.handeye.as_dict().get("flange_to_color_pose")

    def pick_planner(self) -> PickPlanner:
        from .handeye import tip_m_from_env

        return PickPlanner(
            self.pick_frame,
            lambda: self.latest()[0],
            self._handeye_pose,
            tip_m=self.robot.tip_m if self.robot is not None else tip_m_from_env(),
            log=lambda text, ok: self.events.add("pick", text, ok=ok),
        )

    def pick_preview(
        self,
        pixel: tuple[int, int] | None,
        *,
        grip_below_mm: float = 15.0,
        hover_mm: float = 40.0,
        part: PartSpec | None = None,
    ) -> dict:
        """The node's teach-time check (:func:`perceptronics.picknode.preview`): the
        flange from the robot link (the state broadcast: no script, Local mode works).
        Moves nothing."""
        if self.robot is None:
            return {"ok": False, "error": "the cockpit has no robot link (start it with the cell's robot)"}
        return pick_preview(
            self.pick_planner(),
            self.robot.flange_pose(),
            pixel,
            grip_below_mm=grip_below_mm,
            hover_mm=hover_mm,
            part=part,
        )

    def pick_detect(self, part: PartSpec | None = None) -> dict:
        """What the program node's teach screen draws (:func:`perceptronics.picknode.detect_report`)."""
        seq, frame = self.latest()
        if frame is None:
            return {"ok": False, "error": "no frame yet", "last_error": self.last_error}
        picked = self.pick_frame(seq - 1, 0.0)
        if picked is None:
            return {"ok": False, "error": "no frame yet"}
        return detect_report(
            picked,
            pick_port=self.pick_port,
            handeye=self._handeye_pose() is not None,
            tip_m=self.robot.tip_m if self.robot is not None else None,
            part=part,
        )

    def pick_scene(self, opts_text: str, approach_mm: float | None = None) -> dict:
        """The 0.5.0 node's teach screen (:func:`perceptronics.picknode.scene_report`): the same
        FIND the program would make with these options, from the live flange pose (the pose
        stream, else the robot link; camera-only without either). With ``approach_mm`` each
        part also carries ``polyscope_approach_pose``: the approach (fingertips that far over
        its top, along the tool axis) in the controller's **active** TCP — what PolyScope's
        hold-to-move screen takes."""
        from urctl.pose import pose_trans

        opts = parse_options(opts_text[:1024])
        flange = self.frame_pose().get("flange_pose")
        fp: dict = {}
        if self.robot is not None and (flange is None or approach_mm is not None):
            fp = self.robot.flange_pose()
            if flange is None and fp.get("ok") and fp.get("flange"):
                flange = list(fp["flange"])
        out = scene_report(self.pick_planner(), flange, opts, pick_port=self.pick_port)
        offset = fp.get("tcp_offset") if fp.get("tcp_offset_consistent") is not False else None
        if approach_mm is not None and offset is not None:
            for part in out.get("parts", []):
                if "grasp_pose" in part:
                    hover = pose_trans(part["grasp_pose"], [0.0, 0.0, -approach_mm / 1000.0, 0.0, 0.0, 0.0])
                    part["polyscope_approach_pose"] = [round(v, 6) for v in pose_trans(hover, offset)]
        return out

    def workplane_check(self, name: str) -> dict:
        """A workplane from the cell store (three TCP touches, any arm: ``urctl
        workplane``) against the table in the newest frame: the same FIND as a pick with
        that plane, reduced to ``check`` (:func:`perceptronics.volume.check_surface` —
        ``offset_mm``, ``tilt_deg``, …) and the notes. Needs a live robot pose."""
        from urctl.workcell import CellStore

        plane = CellStore.open().workplane(name)
        pose = ",".join(f"{v:.6f}" for v in plane.pose())
        sx, sy = (max(5.0, abs(v) * 1000.0) * (1 if v >= 0 else -1) for v in plane.size)
        opts = parse_options(f"plane=p[{pose}] area={sx:.1f}x{sy:.1f}")
        flange = self.frame_pose().get("flange_pose")
        if flange is None and self.robot is not None:
            fp = self.robot.flange_pose()
            flange = list(fp["flange"]) if fp.get("ok") and fp.get("flange") else None
        if flange is None:
            return {"ok": False, "error": "no live robot pose: the plane can't be put in the picture"}
        out = scene_report(self.pick_planner(), flange, opts, pick_port=self.pick_port)
        if not out.get("ok"):
            return out
        check = out.get("surface_check")
        return {
            "ok": check is not None,
            "workplane": name,
            "taught": {k: plane.as_dict()[k] for k in ("table_z", "tilt_deg", "size", "pose")},
            "check": check,
            "notes": out.get("notes", []),
            "seq": out.get("seq"),
            **({} if check is not None else {"error": "too little of the plane is in view"}),
        }

    # -- API -------------------------------------------------------------------------

    def info(self) -> dict:
        seq, frame = self.latest()
        return {
            "ok": True,
            "camera": self.camera.describe(),
            "segmenter": getattr(self.segmenter, "name", type(self.segmenter).__name__),
            "segment_backends": list(SEGMENT_BACKENDS),
            "seq": seq,
            "fps": round(self.fps(), 2),
            "frame_age_s": None if self.frame_age_s() is None else round(self.frame_age_s(), 2),
            "stalled": self.stalled(),
            "frames_read": self.frames_read,
            "uptime_s": round(time.time() - self.started_at, 1) if self.started_at else 0.0,
            "last_error": self.last_error,
            "frame": frame.summary() if frame else None,
            "has_mask": self.mask is not None,
            "robot": self.robot.describe() if self.robot is not None else None,
            "cell": describe_cell(),
            "events_seq": self.events.seq,
            "views": [v.describe() for v in self.views],
            "cors": {"allowed": list(self.cors_origins), "refused": sorted(self.cors_refused)},
        }

    def note_cors_refused(self, origin: str) -> None:
        """Remember a cross-origin caller the ``--cors`` list turned away; say so once."""
        # A request header: escape control characters/ANSI and cap it before logging.
        origin = repr(origin[:200])[1:-1]
        if origin in self.cors_refused or len(self.cors_refused) >= 32:
            return
        self.cors_refused.add(origin)
        print(
            f"CORS: refused a page from {origin} — restart with --cors {origin} "
            f"(or add it to ${ENV_CORS}) to let it call the API",
            file=sys.stderr,
        )

    # -- pilot: robot actions, doctor, events, snapshots -----------------------------

    def _robot_action(self, name: str, fn, *, summary) -> dict:
        """Run one robot action and log its outcome as an event."""
        try:
            result = fn()
        except Exception as exc:
            self.events.add("robot", f"{name}: {type(exc).__name__}: {exc}", ok=False)
            raise
        ok = bool(result.get("ok", True)) if isinstance(result, dict) else None
        text = summary(result) if callable(summary) else str(summary)
        if isinstance(result, dict) and not ok:
            err = result.get("error") or result.get("reply") or ""
            viol = (result.get("safety") or {}).get("violations")
            if viol:
                err = f"{err} safety: {viol}".strip()
            text = f"{text}: {err}".rstrip(": ")
        self.events.add("robot", text, ok=ok, data={"action": name})
        return result

    def point_at(self, x: int, y: int, window: int = 5) -> dict:
        """Camera-frame metres under pixel (x, y): the per-axis median over a
        ``window``×``window`` neighbourhood of valid-depth pixels, so a single
        noisy or missing pixel doesn't decide a calibration view."""
        seq, frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame yet" + (f" ({self.last_error})" if self.last_error else ""))
        w, h = frame.color.width, frame.color.height
        if not (0 <= x < w and 0 <= y < h):
            raise ValueError(f"point ({x}, {y}) outside the {w}x{h} frame")
        r = window // 2
        pts = []
        for v in range(max(0, y - r), min(h, y + r + 1)):
            for u in range(max(0, x - r), min(w, x + r + 1)):
                p = frame.point_at(u, v)
                if p is not None:
                    pts.append(p)
        if not pts:
            return {"ok": False, "error": "no valid depth around that pixel", "seq": seq, "x": x, "y": y}
        med = []
        for k in range(3):
            vals = sorted(p[k] for p in pts)
            n = len(vals)
            med.append(vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2]))
        return {"ok": True, "seq": seq, "x": x, "y": y, "point_m": med, "samples": len(pts)}

    # -- hand-eye calibration ------------------------------------------------------------

    def cal_mark(self) -> dict:
        link = self._link()
        return self._robot_action(
            "cal_mark",
            link.cal_record_mark,
            summary=lambda r: f"calibration mark {[round(v, 4) for v in r.get('mark_base') or []]}",
        )

    def cal_view(self, x: int, y: int) -> dict:
        link = self._link()
        pt = self.point_at(x, y)
        if not pt.get("ok"):
            raise ValueError(pt["error"])
        return self._robot_action(
            "cal_view",
            lambda: link.cal_add_view(pt["point_m"], pixel=(x, y), seq=pt["seq"]),
            summary=lambda r: f"calibration view #{len(r.get('views', []))} at ({x}, {y})",
        )

    def cal_solve(self) -> dict:
        link = self._link()
        res = link.cal_solve()
        self.events.add(
            "calibration",
            f"solved: RMS {res['rms_m'] * 1000:.2f} mm over {res['views']} views"
            + (f"; {'; '.join(res['warnings'])}" if res["warnings"] else ""),
            ok=not res["warnings"],
        )
        return res

    def cal_apply(self, save: bool = True, force: bool = False) -> dict:
        link = self._link()
        out = link.cal_apply(save=save, force=force)
        if out.get("ok"):
            self.events.add(
                "calibration",
                f"applied {link.handeye.source}" + (f", saved {out['saved']}" if out.get("saved") else ""),
                ok=True,
            )
        else:
            self.events.add("calibration", f"apply refused: {out.get('error')}", ok=False)
        return out

    def cal_reset(self) -> dict:
        out = self._link().calibration.reset()
        out["ok"] = True
        return out

    def cal_remove(self, index: int) -> dict:
        out = self._link().calibration.remove_view(index)
        out["ok"] = True
        return out

    def cal_status(self) -> dict:
        return self._link().cal_status()

    def robot_approach_cycle(
        self,
        standoff_m: float | None = None,
        reference: str | None = None,
        clearance_m: float | None = None,
        hold_s: float | None = None,
        velocity: float | None = None,
    ) -> dict:
        """Click → segment → **Approach**: over the object, down to the standoff,
        hold, back up, back to the capture pose — one program. Uses the current
        segment's camera point; the cell's standoff/reference unless given."""
        link = self._link()
        if not self.features or not self.features.get("point_m"):
            raise ValueError("segment an object with depth first (no camera point to approach)")
        point_m = self.features["point_m"]
        kwargs: dict = {}
        if standoff_m is not None:
            kwargs["standoff_m"] = float(standoff_m)
        if reference is not None:
            if not isinstance(reference, str):
                raise ValueError("reference must be 'tcp' or 'flange'")
            kwargs["reference"] = reference
        if clearance_m is not None:
            kwargs["clearance_m"] = float(clearance_m)
        if hold_s is not None:
            kwargs["hold_s"] = float(hold_s)
        if velocity is not None:
            kwargs["velocity"] = float(velocity)

        def summary(r):
            loc = r.get("locate") or {}
            base = [round(v, 3) for v in loc.get("point_base_m", [])]
            if r.get("ok"):
                ref, so = loc.get("reference"), loc.get("standoff_m")
                return f"approach cycle → {base} ({ref} standoff {so} m) → back"
            return f"approach cycle → {base} refused/failed"

        return self._robot_action(
            "approach_cycle", lambda: link.approach_cycle(point_m, **kwargs), summary=summary
        )

    def robot_jog(self, delta: Sequence[float], velocity: float | None = None) -> dict:
        link = self._link()
        kwargs = {} if velocity is None else {"velocity": float(velocity)}
        mm = [round(v * 1000.0, 1) for v in delta[:3]]
        return self._robot_action("jog", lambda: link.jog(delta, **kwargs), summary=f"jog {mm} mm")

    def robot_bring_up(self) -> dict:
        link = self._link()
        return self._robot_action(
            "bring_up",
            link.bring_up,
            summary=lambda r: f"bring up → {r.get('robot_mode', r.get('reply', ''))}",
        )

    def robot_stop(self) -> dict:
        link = self._link()
        return self._robot_action("stop", link.stop, summary="stop")

    def robot_freedrive(self, enable: bool) -> dict:
        link = self._link()
        return self._robot_action(
            "freedrive", lambda: link.freedrive(enable), summary=f"freedrive {'on' if enable else 'off'}"
        )

    def robot_gripper(self, action: str, position: int | None = None) -> dict:
        link = self._link()
        label = f"gripper {action}" + (f" {position}" if position is not None else "")
        return self._robot_action("gripper", lambda: link.gripper(action, position), summary=label)

    def doctor(self, *, robot: bool = True) -> dict:
        """The pre-flight report for this cockpit's cell. The camera is *this
        process's* (already open), so the SDK/device checks are replaced by the
        live stream stats; the robot side runs the real checks."""
        from .doctor import Check, run_doctor

        report = run_doctor(
            robot_config=self.robot.config if (self.robot is not None and robot) else None,
            camera=False,
            robot=self.robot is not None and robot,
            robot_factory=(lambda _cfg: self.robot.robot) if self.robot is not None else None,
        )
        seq, frame = self.latest()
        fps = self.fps()
        age = self.frame_age_s()
        stalled = frame is None or self.stalled() or (fps < 1.0 and self.frames_read > 0)
        report.checks.insert(
            1,
            Check(
                "stream",
                not stalled and self.last_error is None,
                f"{self.camera.describe().get('kind')} camera: {fps:.1f} fps, seq {seq}, "
                f"{self.frames_read} frames read"
                + (f"; no new frame for {age:.0f} s" if self.stalled() and age is not None else "")
                + (f"; last error: {self.last_error}" if self.last_error else ""),
                fix=self.last_error or "no frames yet — wait for the camera to open, or check the USB link",
            ),
        )
        return report.as_dict()

    def snapshot(self, directory: str | None = None, name: str | None = None) -> dict:
        """Write the latest frame (and the mask, if any) as viewable PNGs and
        return their paths plus the frame summary and current features."""
        seq, frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame yet" + (f" ({self.last_error})" if self.last_error else ""))
        base = Path(directory or DEFAULT_SNAPSHOT_DIR)
        stem = validate_name(name or "snapshot")
        base.mkdir(parents=True, exist_ok=True)
        color_path = base / f"{stem}_color.png"
        depth_path = base / f"{stem}_depth.png"
        color_path.write_bytes(
            encode_png(frame.color.width, frame.color.height, frame.color.channels, frame.color.data)
        )
        depth_path.write_bytes(colourise_depth_png(frame))
        out = {
            "ok": True,
            "seq": seq,
            "color_png": str(color_path),
            "depth_png": str(depth_path),
            "frame": frame.summary(),
            # The features describe the mask's frame (mask_seq), which may lag `seq`.
            "features": self.features if self.mask is not None else None,
        }
        if self.mask is not None and self.mask.area:
            mask_path = base / f"{stem}_mask.png"
            mask_path.write_bytes(self.mask.to_png())
            out["mask_png"] = str(mask_path)
            out["mask_seq"] = self.mask_seq
        # every extra viewpoint that has a picture, as its own file (jpg or png)
        views_out = []
        for pump in self.views:
            vseq, data = pump.latest()
            entry: dict = {"index": pump.index, "name": pump.source.name, "seq": vseq}
            if data:
                ext = "jpg" if pump.source.content_type == "image/jpeg" else "png"
                view_path = base / f"{stem}_view{pump.index}.{ext}"
                view_path.write_bytes(data)
                entry["path"] = str(view_path)
            else:
                entry["error"] = pump.last_error or "no frame yet"
            views_out.append(entry)
        if views_out:
            out["views"] = views_out
        self.events.add("snapshot", f"snapshot → {color_path}", ok=True)
        return out

    # -- robot -------------------------------------------------------------------------

    def _link(self) -> RobotLink:
        if self.robot is None:
            raise ValueError("no robot link (started with --no-robot)")
        return self.robot

    def robot_state(self) -> dict:
        return self._link().state()

    def robot_locate(
        self,
        standoff_m: float | None = None,
        point_m: Sequence[float] | None = None,
        reference: str | None = None,
    ) -> dict:
        """The current segment's camera point (or an explicit ``point_m``) → base
        frame + approach pose. Reads the flange pose; moves nothing."""
        link = self._link()
        if point_m is None:
            if not self.features or not self.features.get("point_m"):
                raise ValueError("segment an object with depth first (no camera point to send)")
            point_m = self.features["point_m"]
        kwargs = {} if standoff_m is None else {"standoff_m": float(standoff_m)}
        if reference is not None:
            if not isinstance(reference, str):
                raise ValueError("reference must be 'tcp' or 'flange'")
            kwargs["reference"] = reference
        return self._robot_action(
            "locate",
            lambda: link.locate(point_m, **kwargs),
            summary=lambda r: (
                "locate → base "
                + str([round(v, 3) for v in r.get("point_base_m", [])] if r.get("ok") else "failed")
                + (" — OUT OF REACH: " + reach_note(r) if r.get("ok") and r.get("reachable") is False else "")
            ),
        )

    def robot_move(
        self, pose: Sequence[float], velocity: float | None = None, tcp: Sequence[float] | None = None
    ) -> dict:
        link = self._link()
        kwargs = {} if velocity is None else {"velocity": float(velocity)}
        if tcp is not None:
            kwargs["tcp"] = tcp
        return self._robot_action(
            "move",
            lambda: link.move(pose, **kwargs),
            summary=lambda r: (
                f"move to {[round(v, 3) for v in pose[:3]]} → landed"
                if r.get("ok")
                else f"move to {[round(v, 3) for v in pose[:3]]} refused"
            ),
        )

    def segment(self, x: int | None = None, y: int | None = None, box: Sequence[float] | None = None) -> dict:
        """Segment the latest frame from a click (``x, y``), a dragged ``box``
        (``[x0, y0, x1, y1]``), or both (the click disambiguates inside the box)."""
        seq, frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame yet" + (f" ({self.last_error})" if self.last_error else ""))
        w, h = frame.color.width, frame.color.height
        if (x is None) != (y is None):
            raise ValueError("x and y must be given together")
        if x is None and box is None:
            raise ValueError("segment needs x/y, box, or both")
        point = None
        if x is not None and y is not None:
            if not (0 <= x < w and 0 <= y < h):
                raise ValueError(f"point ({x}, {y}) outside the {w}x{h} frame")
            point = (x, y)
        nbox = normalize_box(box, w, h) if box is not None else None
        if point is not None and nbox is not None and not (nbox[0] <= x < nbox[2] and nbox[1] <= y < nbox[3]):
            raise ValueError(f"point ({x}, {y}) outside box {list(nbox)}")
        prompt: dict = {}
        if point is not None:
            prompt.update({"x": x, "y": y})
        if nbox is not None:
            prompt["box"] = list(nbox)
        t0 = time.monotonic()
        with self._seg_lock:
            mask = self.segmenter.segment(frame, point, box=nbox)
        return self._adopt_mask(mask, frame, seq, t0, prompt)

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
        with self._cond:  # the pose the mask's frame was taken at (the newest frame's arrival)
            self.mask_t = self._latest_t if seq == self._seq else time.time()
        self.features = feats.as_dict() if feats else None
        pt = self.features.get("point_m") if self.features else None
        self.events.add(
            "segment",
            f"segment {prompt} → {mask.area} px"
            + (f", point {[round(v, 3) for v in pt]} m" if pt else ", no depth"),
            ok=bool(mask.area),
        )
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


class ViewerHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "perceptronics-gui"

    @property
    def app(self) -> ViewerApp:
        return self.server.app  # type: ignore[attr-defined]

    def log_message(self, fmt, *args):  # quiet
        pass

    def _cors_headers(self) -> None:
        """CORS for the origins the cockpit was started with (``--cors``); nothing otherwise."""
        allowed = getattr(self.app, "cors_origins", None) or []
        origin = self.headers.get("Origin")
        # the value sent is the configured entry, never the request's own bytes
        value = "*" if "*" in allowed else next((a for a in allowed if a == origin), None)
        if value is None:
            # Browsers also send Origin on same-origin POSTs; only a foreign page is news.
            if origin and origin not in ("null", f"http://{self.headers.get('Host', '')}"):
                self.app.note_cors_refused(origin)
            return
        self.send_header("Access-Control-Allow-Origin", value)
        if value != "*":
            self.send_header("Vary", "Origin")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Expose-Headers", "X-Seq, X-Fps")
        self.send_header("Access-Control-Max-Age", "600")
        # Chromium's Private Network Access preflight (a page on a LAN address calling
        # a cockpit on loopback, say) wants an explicit yes.
        if self.headers.get("Access-Control-Request-Private-Network", "").lower() == "true":
            self.send_header("Access-Control-Allow-Private-Network", "true")

    def _send(self, body: bytes, ctype: str, status: int = 200, headers: dict | None = None) -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self._cors_headers()
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self) -> None:  # noqa: N802
        """CORS preflight for the cross-origin POSTs a URCap page makes."""
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self._cors_headers()
        self.end_headers()

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
        elif route == "/classic":
            self._send(_CLASSIC.read_bytes(), "text/html; charset=utf-8")
        elif route == "/api/info":
            self._guarded(self.app.info)
        elif route == "/api/doctor":
            self._guarded(lambda: self.app.doctor(robot=qs.get("robot", ["1"])[0] not in ("0", "false")))
        elif route == "/api/events":
            try:
                after = int(qs.get("after", ["0"])[0])
                limit = min(500, max(1, int(qs.get("limit", ["200"])[0])))
            except ValueError:
                self._send_json({"ok": False, "error": "after/limit must be integers"}, status=400)
                return
            self._send_json(
                {"ok": True, "seq": self.app.events.seq, "events": self.app.events.since(after, limit)}
            )
        elif route == "/api/point":
            try:
                x, y = int(qs["x"][0]), int(qs["y"][0])
            except (KeyError, ValueError, IndexError):
                self._send_json({"ok": False, "error": "x and y must be integers"}, status=400)
                return
            self._guarded(lambda: self.app.point_at(x, y))
        elif route == "/api/cal":
            self._guarded(self.app.cal_status)
        elif route == "/api/robot":
            self._guarded(
                lambda: {"ok": True, "robot": self.app.robot.describe() if self.app.robot else None}
            )
        elif route == "/api/robot/pose":
            ps = self.app.pose_stream
            self._send_json(ps.latest() if ps is not None else {"ok": False, "error": "no pose stream"})
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
        elif route.startswith("/api/view/"):
            try:
                index = int(route[len("/api/view/") :])
                after = int(qs["after"][0]) if "after" in qs else None
                timeout_ms = min(10000, max(0, int(qs.get("timeout_ms", ["1500"])[0])))
            except ValueError:
                self._send_json(
                    {"ok": False, "error": "view index, after and timeout_ms must be integers"}, status=400
                )
                return
            try:
                seq, data, ctype = self.app.view_frame(index, after, timeout_ms / 1000.0)
            except IndexError:
                self._send_json({"ok": False, "error": f"no view {index}"}, status=404)
                return
            if data is None:
                pump = self.app.views[index]
                self._send_json(
                    {"ok": False, "error": "no frame yet", "last_error": pump.last_error}, status=503
                )
            else:
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Seq", str(seq))
                self.send_header("X-Fps", f"{self.app.views[index].fps():.2f}")
                self._cors_headers()
                self.end_headers()
                self.wfile.write(data)
        elif route == "/api/pick/detect":
            try:
                part = from_query(qs)
            except ValueError as exc:
                self._send_json({"ok": False, "error": f"bad request: {exc}"}, status=400)
                return
            self._guarded(lambda: self.app.pick_detect(part))
        elif route == "/api/workplane/check":
            name = (qs.get("name") or [""])[0]
            try:
                from urctl.workcell import check_name

                check_name(name)
            except ValueError as exc:
                self._send_json({"ok": False, "error": f"bad request: {exc}"}, status=400)
                return
            self._guarded(lambda: self.app.workplane_check(name))
        elif route == "/api/pick/scene":
            opts = (qs.get("opts") or [""])[0]
            try:
                approach = float(qs["approach_mm"][0]) if "approach_mm" in qs else None
                if approach is not None and not 0.0 <= approach <= 300.0:
                    raise ValueError
            except ValueError:
                self._send_json({"ok": False, "error": "approach_mm must be 0..300"}, status=400)
                return
            self._guarded(lambda: self.app.pick_scene(opts, approach))
        elif route in ("/api/color.png", "/api/depth.png"):
            try:
                after = int(qs["after"][0]) if "after" in qs else None
                timeout_ms = min(10000, max(0, int(qs.get("timeout_ms", ["1500"])[0])))
            except ValueError:
                self._send_json({"ok": False, "error": "after/timeout_ms must be integers"}, status=400)
                return
            image = self.app.color_png if route == "/api/color.png" else self.app.depth_png
            seq, png = image(after, timeout_ms / 1000.0)
            if png is None:
                self._send_json(
                    {"ok": False, "error": "no frame yet", "last_error": self.app.last_error}, status=503
                )
            else:
                self._send(png, "image/png", headers={"X-Seq": str(seq), "X-Fps": f"{self.app.fps():.2f}"})
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
            self._guarded(
                lambda: self.app.segment(
                    _int(payload, "x") if "x" in payload else None,
                    _int(payload, "y") if "y" in payload else None,
                    _box(payload),
                )
            )
        elif route == "/api/robot/pick":
            self._guarded(
                lambda: self.app.pick(
                    pick=bool(payload.get("pick")),
                    fancy=bool(payload.get("fancy")),
                    plan_only=bool(payload.get("plan_only")),
                    look=payload.get("look", True) is not False,
                    survey=bool(payload.get("survey")),
                    target=payload.get("target"),
                )
            )
        elif route == "/api/robot/home":
            self._guarded(self.app.home)
        elif route == "/api/objects":
            self._guarded(self.app.objects)
        elif route == "/api/nearest":
            self._guarded(lambda: self.app.nearest(float(payload.get("near_ratio", 1.2))))
        elif route == "/api/pick/preview":
            try:
                pixel, grip, hover = parse_preview_request(payload)
                part = from_payload(payload)
            except ValueError as exc:
                self._send_json({"ok": False, "error": f"bad request: {exc}"}, status=400)
                return
            self._guarded(lambda: self.app.pick_preview(pixel, grip_below_mm=grip, hover_mm=hover, part=part))
        elif route == "/api/clear":
            self._guarded(self.app.clear)
        elif route == "/api/robot/state":
            self._guarded(self.app.robot_state)
        elif route == "/api/robot/locate":
            self._guarded(
                lambda: self.app.robot_locate(
                    _number(payload, "standoff_m") if "standoff_m" in payload else None,
                    _vector(payload, "point_m", 3) if "point_m" in payload else None,
                    payload.get("reference"),
                )
            )
        elif route == "/api/robot/move":
            self._guarded(
                lambda: self.app.robot_move(
                    _vector(payload, "pose", 6),
                    _number(payload, "velocity") if "velocity" in payload else None,
                    _vector(payload, "tcp", 6) if payload.get("tcp") is not None else None,
                )
            )
        elif route == "/api/robot/approach_cycle":
            self._guarded(
                lambda: self.app.robot_approach_cycle(
                    _number(payload, "standoff_m") if "standoff_m" in payload else None,
                    payload.get("reference"),
                    _number(payload, "clearance_m") if "clearance_m" in payload else None,
                    _number(payload, "hold_s") if "hold_s" in payload else None,
                    _number(payload, "velocity") if "velocity" in payload else None,
                )
            )
        elif route == "/api/robot/jog":
            self._guarded(
                lambda: self.app.robot_jog(
                    _vector(payload, "delta", 6),
                    _number(payload, "velocity") if "velocity" in payload else None,
                )
            )
        elif route == "/api/robot/bring_up":
            self._guarded(self.app.robot_bring_up)
        elif route == "/api/robot/stop":
            self._guarded(self.app.robot_stop)
        elif route == "/api/robot/freedrive":
            self._guarded(lambda: self.app.robot_freedrive(bool(payload.get("enable", False))))
        elif route == "/api/robot/gripper":
            self._guarded(
                lambda: self.app.robot_gripper(
                    str(payload.get("action", "status")),
                    int(payload["position"]) if payload.get("position") is not None else None,
                )
            )
        elif route == "/api/cal/mark":
            self._guarded(self.app.cal_mark)
        elif route == "/api/cal/view":
            self._guarded(lambda: self.app.cal_view(_int(payload, "x"), _int(payload, "y")))
        elif route == "/api/cal/solve":
            self._guarded(self.app.cal_solve)
        elif route == "/api/cal/apply":
            self._guarded(
                lambda: self.app.cal_apply(bool(payload.get("save", True)), bool(payload.get("force", False)))
            )
        elif route == "/api/cal/reset":
            self._guarded(self.app.cal_reset)
        elif route == "/api/cal/remove":
            self._guarded(lambda: self.app.cal_remove(_int(payload, "index")))
        elif route == "/api/snapshot":
            self._guarded(
                lambda: self.app.snapshot(
                    _optional_str(payload, "dir"),
                    _optional_str(payload, "name"),
                )
            )
        else:
            self._send_json({"ok": False, "error": f"no route {route}"}, status=404)


def _optional_str(payload: dict, key: str) -> str | None:
    v = payload.get(key)
    if v is None:
        return None
    if not isinstance(v, str) or not v.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return v


# Turbo-like anchors, interpolated to 256 entries: index 0 = near, 255 = far.
_DEPTH_ANCHORS = (
    (48, 18, 59),
    (70, 107, 229),
    (27, 208, 213),
    (163, 255, 62),
    (245, 193, 52),
    (220, 52, 23),
    (122, 4, 3),
)


def _depth_palette() -> list[bytes]:
    out = []
    n = len(_DEPTH_ANCHORS) - 1
    for i in range(256):
        t = i / 255.0 * n
        k = min(n - 1, int(t))
        f = t - k
        a, b = _DEPTH_ANCHORS[k], _DEPTH_ANCHORS[k + 1]
        out.append(bytes(round(a[c] * (1 - f) + b[c] * f) for c in range(3)))
    return out


_PALETTE = _depth_palette()


HEATMAP_NEAR_M, HEATMAP_FAR_M = 0.15, 1.0
HEATMAP_STEP_M = 0.01  # the ramp's ends move in steps: frame-to-frame noise does not shimmer it
HEATMAP_MIN_SPAN_M = 0.05


def heatmap_range(frame: RgbdFrame) -> tuple[float, float]:
    """The near and far ends of the heatmap's ramp for this frame: the 2nd and 98th
    percentile of its valid depths (a flying pixel does not stretch it), rounded outward to
    :data:`HEATMAP_STEP_M`, at least :data:`HEATMAP_MIN_SPAN_M` apart. A frame with no
    depth gets the fixed 0.15–1.0 m."""
    w, h, data, scale = frame.depth.width, frame.depth.height, frame.depth.data, frame.depth.scale_m
    vals = []
    for y in range(0, h, 8):
        for k in range(2 * y * w, 2 * (y + 1) * w, 16):
            raw = data[k] | (data[k + 1] << 8)
            if raw:
                vals.append(raw)
    if len(vals) < 20:
        return HEATMAP_NEAR_M, HEATMAP_FAR_M
    vals.sort()
    near = vals[len(vals) * 2 // 100] * scale
    far = vals[min(len(vals) - 1, len(vals) * 98 // 100)] * scale
    near = math.floor(near / HEATMAP_STEP_M) * HEATMAP_STEP_M
    far = math.ceil(far / HEATMAP_STEP_M) * HEATMAP_STEP_M
    if far - near < HEATMAP_MIN_SPAN_M:
        far = near + HEATMAP_MIN_SPAN_M
    return near, far


def colourise_depth_png(
    frame: RgbdFrame, near_m: float | None = None, far_m: float | None = None, step: int = 1
) -> bytes:
    """The depth image as an 8-bit RGB PNG a human (or a vision model) can read:
    near→far runs through the same turbo-like ramp the page uses, invalid
    depth is black. Range defaults to the frame's own valid min/max. ``step`` > 1
    keeps every ``step``-th pixel each way (a smaller, cheaper picture)."""
    stats = frame.depth.stats()
    near = near_m if near_m is not None else (stats["min_m"] or 0.2)
    far = far_m if far_m is not None else (stats["max_m"] or near + 1.0)
    if far <= near:
        far = near + 0.001
    scale = frame.depth.scale_m
    span = far - near
    black = b"\x00\x00\x00"
    pal = _PALETTE
    rows = bytearray()
    if step > 1:
        w, h, data = frame.depth.width, frame.depth.height, frame.depth.data
        for y in range(0, h, step):
            for k in range(2 * y * w, 2 * (y + 1) * w, 2 * step):
                raw = data[k] | (data[k + 1] << 8)
                if not raw:
                    rows += black
                    continue
                t = (raw * scale - near) / span
                rows += pal[0 if t <= 0 else (255 if t >= 1 else int(t * 255))]
        return encode_png(len(range(0, w, step)), len(range(0, h, step)), 3, bytes(rows))
    for raw in frame.depth.values():
        if not raw:
            rows += black
            continue
        t = (raw * scale - near) / span
        idx = 0 if t <= 0 else (255 if t >= 1 else int(t * 255))
        rows += pal[idx]
    return encode_png(frame.depth.width, frame.depth.height, 3, bytes(rows))


def _number(payload: dict, key: str) -> float:
    v = payload.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or v in (float("inf"), float("-inf")):
        raise ValueError(f"{key} must be a finite number")
    return float(v)


def _vector(payload: dict, key: str, n: int) -> list[float]:
    v = payload.get(key)
    if not isinstance(v, list) or len(v) != n:
        raise ValueError(f"{key} must be a list of {n} numbers")
    out = []
    for x in v:
        if (
            isinstance(x, bool)
            or not isinstance(x, (int, float))
            or x != x
            or x in (float("inf"), float("-inf"))
        ):
            raise ValueError(f"{key} must contain finite numbers")
        out.append(float(x))
    return out


def _box(payload: dict) -> list | None:
    v = payload.get("box")
    if v is None:
        return None
    if not isinstance(v, list) or len(v) != 4:
        raise ValueError("box must be [x0, y0, x1, y1]")
    return v


def _int(payload: dict, key: str) -> int:
    v = payload.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or v in (float("inf"), float("-inf")):
        raise ValueError(f"{key} must be a finite number")
    return int(v)


def serve(
    camera: RgbdCamera,
    *,
    config: PerceptionConfig | None = None,
    bind: str = "127.0.0.1",
    port: int = DEFAULT_PORT,
    open_browser: bool = True,
    robot: RobotLink | None = None,
    demo: bool = False,
    views: list[ViewSource] | None = None,
    cors: Sequence[str] | None = None,
    pick_port: int = DEFAULT_PICK_PORT,
) -> None:
    """Run the cockpit until interrupted (the ``perceptronics gui`` entry point).
    ``pick_port`` (0 = off) serves the PolyScope 5 Perceptronic Pick program node's
    line protocol on the same interface (:mod:`perceptronics.picknode`).
    ``demo`` opens the browser on the classic page's demo view
    (``/classic?demo=1``: one picture, four big buttons, one light).
    ``views`` are the extra webcam viewpoints (:mod:`perceptronics.views`)."""
    pose_stream = None
    if robot is not None and not robot.dry_run and os.environ.get("PERCEPTRONICS_POSE_STREAM", "1") != "0":
        pose_stream = PoseStream(robot.config)
        pose_stream.start()
    app = ViewerApp(camera, config=config, robot=robot, views=views, cors=cors, pose_stream=pose_stream)
    server = ThreadingHTTPServer((bind, port), ViewerHandler)
    server.daemon_threads = True
    server.app = app  # type: ignore[attr-defined]
    host = "127.0.0.1" if bind in ("0.0.0.0", "") else bind
    url = f"http://{host}:{server.server_address[1]}/" + ("classic?demo=1" if demo else "")
    kind = camera.describe()["kind"]
    if views:
        kind += " + " + ", ".join(f"view {v.name}" for v in views)
    robot_desc = (
        f"robot: {robot.config.host}{' (dry-run)' if robot.dry_run else ''}"
        if robot is not None
        else "robot: off"
    )
    print(
        f"perceptronics gui -> {url}   (camera: {kind}, {robot_desc}, "
        f"snapshots: {DEFAULT_SNAPSHOT_DIR}, Ctrl-C to stop)"
    )
    if bind not in ("127.0.0.1", "localhost", "::1"):
        print(f"WARNING: bound to {bind} with no authentication — only do this on a trusted cell network.")
    if app.cors_origins:
        print(f"CORS: API callable from {', '.join(app.cors_origins)} (a PolyScope X URCap page, say)")
    pick = None
    if pick_port:
        try:
            pick = PickServer(bind, pick_port, app.pick_planner())
        except OSError as exc:
            print(f"WARNING: pick server not started on {bind}:{pick_port}: {exc}")
        else:
            app.pick_port = pick.server_address[1]
            pick.start()
            print(f"pick server (PolyScope Perceptronic Pick node) on {bind}:{app.pick_port}")
    app.start()
    if open_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if pick is not None:
            pick.stop()
        app.stop()


ENV_CORS = "PERCEPTRONICS_CORS"
# what a browser sends as Origin: scheme://host[:port] — no path, no whitespace, no control characters
_ORIGIN_RE = re.compile(r"[a-z][a-z0-9+.-]*://(\[[0-9a-fA-F:.]+\]|[A-Za-z0-9._~-]+)(:[0-9]{1,5})?\Z")


def add_pick_port_arg(ap) -> None:
    ap.add_argument(
        "--pick-port",
        type=int,
        default=DEFAULT_PICK_PORT,
        help=f"TCP port for the PolyScope 5 Perceptronic Pick node (default {DEFAULT_PICK_PORT}; 0 = off)",
    )


def add_cors_arg(ap) -> None:
    ap.add_argument(
        "--cors",
        default=None,
        help="origin(s) allowed to call the API from another page, comma-separated, or '*' — e.g. "
        f"http://localhost:8000 for the PolyScope X sim's URCap (default: ${ENV_CORS}, else same-origin "
        "only)",
    )


def cors_from_args(args) -> list[str]:
    raw = getattr(args, "cors", None) or os.environ.get(ENV_CORS, "")
    return [o.strip() for o in raw.split(",") if o.strip()]


def add_camera_args(ap) -> None:
    """The camera/viewer flags shared by ``perceptronics gui`` and ``perceptronics-gui``."""
    ap.add_argument(
        "--fake",
        action="store_true",
        help="synthetic RGB-D scene instead of a RealSense (or PERCEPTRONICS_FAKE=1)",
    )
    ap.add_argument(
        "--serial", default=None, help="RealSense serial (default: $PERCEPTRONICS_RS_SERIAL or first)"
    )
    ap.add_argument(
        "--rs-fps", type=int, default=None, help="RealSense stream fps (default: auto — 30, or 15 on USB 2)"
    )
    ap.add_argument("--no-align", action="store_true", help="don't align depth to the color image")
    ap.add_argument(
        "--depth-res",
        default=None,
        metavar="WxH",
        help="depth stream resolution (default: $PERCEPTRONICS_RS_DEPTH_WIDTH x _HEIGHT, 848x480 — the "
        "D435's native mode; aligned depth is resampled onto the colour grid)",
    )
    ap.add_argument(
        "--no-depth-filters",
        action="store_true",
        help="raw sensor depth: skip the SDK's spatial + temporal post-processing",
    )
    ap.add_argument(
        "--rs-preset",
        default=None,
        help=f"depth visual preset at open: {'|'.join(sorted(VISUAL_PRESETS))}|none "
        "(default: $PERCEPTRONICS_RS_PRESET, high_accuracy; none = leave the sensor as is)",
    )
    ap.add_argument(
        "--laser-power",
        default=None,
        metavar="MW",
        help="projector power in mW, or max|none (default: $PERCEPTRONICS_RS_LASER_POWER, max)",
    )
    ap.add_argument(
        "--rs-lean",
        action="store_true",
        help="fewest USB handle opens at start: no USB-type probe or mode enumeration, no preset/laser "
        "writes, global time off (or PERCEPTRONICS_RS_LEAN=1; the macOS claim-race experiment)",
    )
    ap.add_argument("--library", default=None, help="path to librealsense2 (default: $REALSENSE_LIB / auto)")
    ap.add_argument(
        "--view",
        action="append",
        default=None,
        metavar="DEVICE",
        help="an extra webcam viewpoint shown under the colour/depth pair and saved with every snapshot "
        '(repeatable; macOS: a device name from `ffmpeg -f avfoundation -list_devices true -i ""`, '
        "Linux: /dev/videoN, lavfi:testsrc for a synthetic one; "
        "default: $PERCEPTRONICS_VIEWS, comma-separated). "
        "Needs ffmpeg on PATH; --fake makes them synthetic",
    )
    ap.add_argument(
        "--view-res",
        default=None,
        metavar="WxH",
        help="viewpoint capture size (default: $PERCEPTRONICS_VIEW_RES, 640x480)",
    )
    ap.add_argument(
        "--view-fps",
        type=int,
        default=None,
        help="viewpoint capture rate (default: $PERCEPTRONICS_VIEW_FPS, 15)",
    )


def views_from_args(args, config: PerceptionConfig) -> list[ViewSource]:
    """The extra viewpoints named on the command line or in the cell (``PERCEPTRONICS_VIEWS``)."""
    specs = list(getattr(args, "view", None) or parse_view_specs(config.views))
    if not specs:
        return []
    width, height = parse_view_size(getattr(args, "view_res", None) or config.view_res)
    fps = getattr(args, "view_fps", None) or config.view_fps
    fake = bool(getattr(args, "fake", False)) or _env_flag("PERCEPTRONICS_FAKE")
    return open_views(specs, fake=fake, width=width, height=height, fps=int(fps))


def add_robot_args(ap) -> None:
    """The robot-link flags shared by ``perceptronics gui`` and ``perceptronics-gui``."""
    ap.add_argument("--no-robot", action="store_true", help="no robot panel / link at all")
    ap.add_argument(
        "--robot-host", default=None, help="UR controller address (default: $UR_HOST, localhost = URSim)"
    )
    ap.add_argument(
        "--robot-dry-run",
        action="store_true",
        help="validate + audit robot actions but send nothing (a stand-in flange pose is used)",
    )


def robot_from_args(args) -> RobotLink | None:
    if getattr(args, "no_robot", False):
        return None
    from urctl.config import RobotConfig

    link = RobotLink(
        RobotConfig.from_env(host=getattr(args, "robot_host", None)),
        dry_run=bool(getattr(args, "robot_dry_run", False)),
    )
    return link


def _env_flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def parse_resolution(text: str) -> tuple[int, int]:
    """``"848x480"`` → ``(848, 480)``; a clear error otherwise."""
    try:
        w, h = text.lower().replace("×", "x").split("x")
        width, height = int(w), int(h)
    except ValueError:
        raise ValueError(f"resolution must look like 848x480, got {text!r}") from None
    if width <= 0 or height <= 0:
        raise ValueError(f"resolution must be positive, got {text!r}")
    return width, height


def depth_tuning_from(preset: str, laser_power: str) -> DepthTuning | None:
    """The config/CLI strings → :class:`DepthTuning` (``None`` when both say leave-alone)."""
    preset_l = (preset or "").strip().lower()
    laser_l = (laser_power or "").strip().lower()
    preset_v = None if preset_l in ("", "none", "leave") else preset_l
    if laser_l in ("", "none", "leave"):
        laser_v: float | None = None
    elif laser_l == "max":
        laser_v = LASER_MAX
    else:
        try:
            laser_v = float(laser_l)
        except ValueError:
            raise ValueError(f"laser power must be max, none or mW, got {laser_power!r}") from None
    if preset_v is None and laser_v is None:
        return None
    return DepthTuning(preset=preset_v, laser_power=laser_v, emitter=None if laser_v is None else True)


def color_size_is_explicit(args) -> bool:
    """Did the operator ask for a colour size (``--width/--height`` or
    ``PERCEPTRONICS_WIDTH/HEIGHT``)? Otherwise colour follows the depth size —
    a D435 streaming 848x480 depth next to 640x480 colour returned black
    colour frames (see ``realsense.DEFAULT_DEPTH_WIDTH``)."""
    if getattr(args, "width", None) is not None or getattr(args, "height", None) is not None:
        return True
    return bool(os.environ.get("PERCEPTRONICS_WIDTH") or os.environ.get("PERCEPTRONICS_HEIGHT"))


def camera_from_args(args, config: PerceptionConfig) -> RgbdCamera:
    depth_res = getattr(args, "depth_res", None)
    depth_w, depth_h = (
        parse_resolution(depth_res) if depth_res else (config.rs_depth_width, config.rs_depth_height)
    )
    color_w, color_h = (config.width, config.height) if color_size_is_explicit(args) else (depth_w, depth_h)
    filters_on = config.rs_filters and not getattr(args, "no_depth_filters", False)
    tuning = depth_tuning_from(
        getattr(args, "rs_preset", None) or config.rs_preset,
        getattr(args, "laser_power", None) or config.rs_laser_power,
    )
    return open_camera(
        fake=bool(getattr(args, "fake", False)) or _env_flag("PERCEPTRONICS_FAKE"),
        width=color_w,
        height=color_h,
        fps=(args.rs_fps if getattr(args, "rs_fps", None) else config.rs_fps) or None,
        serial=(args.serial if getattr(args, "serial", None) else config.rs_serial) or None,
        align=not getattr(args, "no_align", False),
        library=getattr(args, "library", None),
        depth_width=depth_w,
        depth_height=depth_h,
        filters=DEFAULT_DEPTH_FILTERS if filters_on else None,
        tuning=tuning,
        lean=bool(getattr(args, "rs_lean", False)) or config.rs_lean,
    )


def main(argv: list[str] | None = None) -> int:
    """Standalone ``perceptronics-gui`` console script."""
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    import argparse

    ap = argparse.ArgumentParser(
        prog="perceptronics-gui", description="local RGB-D cockpit for a RealSense camera"
    )
    add_camera_args(ap)
    add_robot_args(ap)
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument(
        "--segment-backend", default=None, help="stub | sam (default: $PERCEPTRONICS_SEGMENT_BACKEND)"
    )
    ap.add_argument(
        "--sam-model",
        default=None,
        help="SAM checkpoint id for the sam backend (default: $PERCEPTRONICS_SAM_MODEL)",
    )
    ap.add_argument("--bind", default="127.0.0.1", help="interface to bind (default: loopback only)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"port (default {DEFAULT_PORT})")
    ap.add_argument("--no-browser", action="store_true", help="don't open the browser automatically")
    ap.add_argument(
        "--demo", action="store_true", help="open the classic page's demo view: one picture, four big buttons"
    )
    add_pick_port_arg(ap)
    add_cors_arg(ap)
    ap.add_argument(
        "--cell", default=None, help="cell profile (sim|ur3|ur20 or a .env path; default: $UR_CELL)"
    )
    args = ap.parse_args(argv)
    from .cell import apply_cell

    try:
        apply_cell(args.cell)
    except ValueError as exc:
        print(f"--cell: {exc}", file=sys.stderr)
        return 2
    overrides: dict[str, object] = {}
    if args.width is not None:
        overrides["width"] = args.width
    if args.height is not None:
        overrides["height"] = args.height
    if args.segment_backend is not None:
        overrides["segment_backend"] = args.segment_backend
    if args.sam_model is not None:
        overrides["sam_model"] = args.sam_model
    config = PerceptionConfig.from_env(**overrides)
    serve(
        camera_from_args(args, config),
        config=config,
        bind=args.bind,
        port=args.port,
        open_browser=not args.no_browser,
        robot=robot_from_args(args),
        demo=bool(getattr(args, "demo", False)),
        views=views_from_args(args, config),
        cors=cors_from_args(args),
        pick_port=args.pick_port,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
