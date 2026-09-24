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
  * ``GET  /api/robot``               the robot link: host, hand-eye, dry-run.
  * ``POST /api/robot/state``         ``ur_get_state`` through the tool registry.
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
  * ``GET  /api/doctor``              the pre-flight report (:mod:`perception.doctor`)
                                      for the robot side; the camera side is this
                                      process's own stream stats.
  * ``GET  /api/events?after=N``      the cockpit's event log (robot actions,
                                      segments, captures, camera errors) — what an
                                      agent or a human reads to see what just happened.
  * ``GET  /api/scan`` / ``POST /api/scan`` ``{delta?, velocity?, latency_s?}`` /
    ``POST /api/scan/approach`` ``{index}`` / ``POST /api/table/from_depth`` —
    the monocular scan (docs/mono-scan.md): one sweep move with frames + RTDE
    poses, parts located on the table plane, graded against the depth.
  * ``POST /api/snapshot`` ``{dir?, name?}`` write the latest frame as
                                      ``<name>_color.png`` + ``<name>_depth.png`` (colourised)
                                      to a directory and return the paths — the
                                      "give the agent eyes" call.
  * ``GET  /api/point?x=&y=``       the camera-frame point under a pixel (median of a
                                      5×5 window) — what a calibration view uses.
  * ``POST /api/cal/mark`` / ``/api/cal/view`` ``{x, y}`` / ``/api/cal/solve`` /
    ``/api/cal/apply`` ``{save?}`` / ``/api/cal/reset`` / ``GET /api/cal`` —
                                      touch-and-click hand-eye calibration
                                      (:mod:`perception.calibrate`).
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
import os
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

from .capture import CaptureStore, validate_name
from .cell import describe_cell
from .config import PerceptionConfig
from .factory import SEGMENT_BACKENDS, make_segmenter
from .pngio import encode_png
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
from .robotlink import RobotLink
from .segment import Mask, StubSegmenter, extract_features, normalize_box

DEFAULT_PORT = 7621
DEFAULT_CAPTURE_ROOT = "captures"
DEFAULT_SNAPSHOT_DIR = "captures/snapshots"
REOPEN_DELAY_S = 1.0
REOPEN_MAX_DELAY_S = 30.0
EVENT_LOG_SIZE = 500


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


class ViewerApp:
    """State behind the handlers: one camera, one pump thread, one segmenter."""

    def __init__(
        self,
        camera: RgbdCamera,
        *,
        config: PerceptionConfig | None = None,
        store: CaptureStore | None = None,
        segmenter=None,
        robot: RobotLink | None = None,
    ):
        self.camera = camera
        self.robot = robot
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
        self.events = EventLog()
        self._last_logged_error: str | None = None
        # monocular scan (perception.sweep / locate2d): the newest frame's host stamp,
        # a tap that collects frames during a sweep, the last result, the table plane
        self._latest_t: float = 0.0
        self._tap: list | None = None
        self.last_scan: dict | None = None
        self.table_plane = None
        self.parts: list[str] | None = None
        self._scan_lock = threading.Lock()

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
            # a camera that knows its exposure instant (the synthetic sweep camera; a
            # triggered camera later) reports it — otherwise the arrival time stands in
            exposure = frame.extra.get("exposure_t") if isinstance(frame.extra, dict) else None
            with self._cond:
                self._latest = frame
                self._latest_t = float(exposure) if exposure is not None else now
                if self._tap is not None:
                    self._tap.append((frame, now))
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

    def wait_frame_stamped(self, after: int, timeout_s: float) -> tuple[int, RgbdFrame | None, float]:
        """:meth:`wait_frame` plus the frame's host stamp, read under the same lock."""
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._running and self._seq <= after:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(remaining)
            return self._seq, self._latest, self._latest_t

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
            "robot": self.robot.describe() if self.robot is not None else None,
            "cell": describe_cell(),
            "events_seq": self.events.seq,
        }

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
        stalled = frame is None or (fps < 1.0 and self.frames_read > 0)
        report.checks.insert(
            1,
            Check(
                "stream",
                not stalled and self.last_error is None,
                f"{self.camera.describe().get('kind')} camera: {fps:.1f} fps, seq {seq}, "
                f"{self.frames_read} frames read"
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
        self.events.add("snapshot", f"snapshot → {color_path}", ok=True)
        return out

    # -- monocular scan (docs/mono-scan.md) -------------------------------------------

    def _table(self):
        """The table plane: set in this session (table_from_depth), else env/file."""
        if self.table_plane is not None:
            return self.table_plane
        from .tableplane import Plane

        plane = Plane.from_env()
        if plane is None:
            from .touch import TouchSet, default_touch_path

            path = default_touch_path()
            if Path(path).is_file():
                plane = TouchSet.load(path).table_plane()
        return plane

    def _library(self):
        import glob

        from .partlib import PartLibrary

        paths = self.parts if self.parts is not None else sorted(glob.glob("parts/*.stl"))
        return PartLibrary.from_paths(paths) if paths else None

    def table_from_depth(self, save: bool = True) -> dict:
        """The table plane from the latest RGB-D frame + the live flange pose."""
        from urctl.pose import Transform

        from .tableplane import default_table_path, plane_from_depth

        link = self._link()
        seq, frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame yet" + (f" ({self.last_error})" if self.last_error else ""))
        fp = link.flange_pose()
        if not fp.get("ok") or not fp.get("flange"):
            return {"ok": False, "error": fp.get("error") or "could not read the flange pose", "robot": fp}
        leg = link.handeye.flange_to_color if frame.aligned else link.handeye.flange_to_depth
        plane, stats = plane_from_depth(frame, Transform.from_pose(fp["flange"]).compose(leg))
        self.table_plane = plane
        out = {"ok": True, "seq": seq, "plane": plane.as_dict(), "stats": stats, "flange": fp["flange"]}
        if stats["tilt_from_base_z_deg"] > 10.0:
            out["warning"] = f"surface is {stats['tilt_from_base_z_deg']:.1f} deg off the base Z axis"
        if save:
            path = default_table_path()
            plane.save(path)
            out["saved"] = path
        self.events.add(
            "scan",
            f"table plane from depth: z {plane.point[2]:.4f} m, "
            f"tilt {stats['tilt_from_base_z_deg']:.1f} deg, "
            f"{stats['inliers']}/{stats['samples']} inliers, rms {stats['rms_m'] * 1000:.1f} mm",
            ok=stats["tilt_from_base_z_deg"] <= 10.0,
        )
        return out

    def scan(
        self,
        delta: Sequence[float] | None = None,
        velocity: float | None = None,
        acceleration: float | None = None,
        latency_s: float | None = None,
        save: bool = True,
    ) -> dict:
        """One sweep move with this cockpit's frames + an RTDE pose recorder,
        then locate the parts on the table plane and grade them against the
        depth of the frame at the end of the sweep. Refused while another scan runs."""
        from urctl.pose import Transform

        from .depthcheck import check_objects
        from .locate2d import locate_objects
        from .monocam import MonoFrame
        from .posestream import PoseRecorder
        from .sweep import (
            DEFAULT_DELTA_M,
            DEFAULT_SWEEP_ACCELERATION,
            DEFAULT_SWEEP_ROOT,
            DEFAULT_SWEEP_VELOCITY,
            latency_from_env,
            run_sweep,
        )

        link = self._link()
        plane = self._table()
        if plane is None:
            raise ValueError("no table plane: run table_from_depth first (or set PERCEPTION_TABLE_Z)")
        _seq, frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame yet" + (f" ({self.last_error})" if self.last_error else ""))
        if not self._scan_lock.acquire(blocking=False):
            raise ValueError("a scan is already running")
        try:
            d = list(delta) if delta is not None else list(DEFAULT_DELTA_M)
            lat = latency_from_env() if latency_s is None else float(latency_s)
            offset = link.tcp_offset() or [0.0] * 6
            recorder = (
                link.recorder() if hasattr(link, "recorder") else PoseRecorder(link.config, tcp_offset=offset)
            )
            app = self

            class _Source:  # frames straight from the pump, stamped on arrival
                def __init__(self):
                    self.after = app.latest()[0]
                    self.seq = 0
                    self.last = None  # (RgbdFrame, host_t) of the newest frame handed out

                def read(self):
                    import numpy as np

                    seq, f, t = app.wait_frame_stamped(self.after, 2.0)
                    if f is None or seq <= self.after:
                        raise RuntimeError("camera stalled during the sweep")
                    self.after = seq
                    self.last = (f, t)
                    rgb = f.color.to_numpy()
                    gray = (0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]).astype(np.uint8)
                    self.seq += 1
                    return MonoFrame(gray, t, self.seq, f.timestamp_ms)

                def describe(self):
                    return app.camera.describe()

            self.events.add("scan", f"sweep {[round(v * 1000) for v in d]} mm starting", ok=None)
            src = _Source()
            sweep = self._robot_action(
                "scan",
                lambda: {
                    "ok": True,
                    "sweep": run_sweep(
                        src,
                        recorder,
                        link,
                        intrinsics=frame.intrinsics,
                        flange_to_color=link.handeye.flange_to_color,
                        delta=d,
                        velocity=DEFAULT_SWEEP_VELOCITY if velocity is None else float(velocity),
                        acceleration=DEFAULT_SWEEP_ACCELERATION
                        if acceleration is None
                        else float(acceleration),
                        latency_s=lat,
                        plane=plane,
                    ),
                },
                summary=lambda r: (
                    f"sweep done: {len(r['sweep'].frames)} frames, "
                    f"baseline {r['sweep'].baseline_m() * 1000:.0f} mm"
                ),
            )["sweep"]
            sweep.meta["handeye"] = link.handeye.as_dict()
            library = self._library()
            objs = locate_objects(sweep.frame_pairs(), sweep.intrinsics, plane, library=library)
            result = {
                "ok": True,
                "sweep": sweep.summary(),
                "latency_s": lat,
                "plane": plane.as_dict(),
                "parts": [p.name for p in library.parts] if library else [],
                "objects": [o.as_dict() for o in objs],
            }
            # grade against the depth of the last frame of the sweep (its pose from the trajectory)
            last, t_last = src.last if src.last is not None else (None, 0.0)
            pose = sweep.flange_at(t_last - lat) if last is not None else None
            if last is not None and pose is not None and objs:
                t_base_cam = Transform.from_pose(pose).compose(link.handeye.flange_to_color)
                for o, c in zip(result["objects"], check_objects(objs, last, t_base_cam, plane), strict=True):
                    o["depth_check"] = c
            if save:
                from .scan_cli import _next_dir

                out_dir = _next_dir(DEFAULT_SWEEP_ROOT, "cockpit")
                sweep.save(out_dir)
                result["sweep_dir"] = str(out_dir)
            self.last_scan = result
            self.events.add(
                "scan",
                f"located {len(objs)} object(s)"
                + (
                    ": "
                    + "; ".join(
                        f"#{o['index']} {[round(v, 3) for v in o['center_base_m']]} "
                        f"h {o['height_m'] * 1000:.0f} mm"
                        + (f" {o['match']['part']}" if o.get("match") else "")
                        + (
                            f" (depth Δ {o['depth_check']['diff_mm']:+.1f} mm)"
                            if o.get("depth_check") and o["depth_check"].get("diff_mm") is not None
                            else ""
                        )
                        for o in result["objects"]
                    )
                    if objs
                    else ""
                ),
                ok=bool(objs),
            )
            return result
        finally:
            self._scan_lock.release()

    def scan_status(self) -> dict:
        plane = self._table()
        lib = self._library()
        return {
            "ok": True,
            "plane": None if plane is None else plane.as_dict(),
            "parts": [p.name for p in lib.parts] if lib else [],
            "last": self.last_scan,
            "running": self._scan_lock.locked(),
        }

    def scan_approach(
        self, index: int, standoff_m: float | None = None, reference: str | None = None
    ) -> dict:
        """Approach pose above scanned object ``index`` (along the table normal)."""
        link = self._link()
        if not self.last_scan or not self.last_scan.get("objects"):
            raise ValueError("no scan result yet")
        objs = self.last_scan["objects"]
        if not (0 <= index < len(objs)):
            raise ValueError(f"no object {index} (have {len(objs)})")
        o = objs[index]
        normal = (self.last_scan.get("plane") or {}).get("normal") or [0.0, 0.0, 1.0]
        kwargs: dict = {"along": normal}
        if standoff_m is not None:
            kwargs["standoff_m"] = float(standoff_m)
        if reference is not None:
            kwargs["reference"] = reference
        res = self._robot_action(
            "scan_approach",
            lambda: link.approach_point_base(o["center_base_m"], **kwargs),
            summary=lambda r: (
                f"approach for object #{index} → {[round(v, 3) for v in r.get('approach_pose', [])[:3]]}"
                + (" — OUT OF REACH" if r.get("reachable") is False else "")
            ),
        )
        res["object"] = o
        return res

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
                + (
                    f" — OUT OF REACH: approach {r['commanded_distance_m']:.3f} m from base, "
                    f"{r.get('model') or 'arm'} reaches {r['max_reach_m']:.2f} m; move the part closer"
                    if r.get("ok") and r.get("reachable") is False
                    else ""
                )
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
        self.events.add(
            "capture", f"capture {name} #{result.get('index', '?')}" + (" +mask" if mask else ""), ok=True
        )
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
        elif route == "/api/scan":
            self._guarded(self.app.scan_status)
        elif route == "/api/robot":
            self._guarded(
                lambda: {"ok": True, "robot": self.app.robot.describe() if self.app.robot else None}
            )
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
            self._guarded(
                lambda: self.app.segment(
                    _int(payload, "x") if "x" in payload else None,
                    _int(payload, "y") if "y" in payload else None,
                    _box(payload),
                )
            )
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
        elif route == "/api/scan":
            self._guarded(
                lambda: self.app.scan(
                    _vector(payload, "delta", 3) if "delta" in payload else None,
                    _number(payload, "velocity") if "velocity" in payload else None,
                    _number(payload, "acceleration") if "acceleration" in payload else None,
                    _number(payload, "latency_s") if "latency_s" in payload else None,
                    bool(payload.get("save", True)),
                )
            )
        elif route == "/api/scan/approach":
            self._guarded(
                lambda: self.app.scan_approach(
                    _int(payload, "index"),
                    _number(payload, "standoff_m") if "standoff_m" in payload else None,
                    payload.get("reference"),
                )
            )
        elif route == "/api/table/from_depth":
            self._guarded(lambda: self.app.table_from_depth(bool(payload.get("save", True))))
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


def colourise_depth_png(frame: RgbdFrame, near_m: float | None = None, far_m: float | None = None) -> bytes:
    """The depth image as an 8-bit RGB PNG a human (or a vision model) can read:
    near→far runs through the same turbo-like ramp the page uses, invalid
    depth is black. Range defaults to the frame's own valid min/max."""
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
    capture_root: str = DEFAULT_CAPTURE_ROOT,
    bind: str = "127.0.0.1",
    port: int = DEFAULT_PORT,
    open_browser: bool = True,
    robot: RobotLink | None = None,
    demo: bool = False,
) -> None:
    """Run the cockpit until interrupted (the ``perception gui`` entry point).
    ``demo`` opens the browser on the demo view (``/?demo=1``: one picture,
    four buttons, one light; the header's *Developer view* toggles back)."""
    app = ViewerApp(camera, config=config, store=CaptureStore(Path(capture_root)), robot=robot)
    server = ThreadingHTTPServer((bind, port), ViewerHandler)
    server.daemon_threads = True
    server.app = app  # type: ignore[attr-defined]
    host = "127.0.0.1" if bind in ("0.0.0.0", "") else bind
    url = f"http://{host}:{server.server_address[1]}/" + ("?demo=1" if demo else "")
    kind = camera.describe()["kind"]
    robot_desc = (
        f"robot: {robot.config.host}{' (dry-run)' if robot.dry_run else ''}"
        if robot is not None
        else "robot: off"
    )
    print(
        f"perception gui -> {url}   (camera: {kind}, {robot_desc}, captures: {capture_root}, Ctrl-C to stop)"
    )
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
    ap.add_argument(
        "--fake",
        action="store_true",
        help="synthetic RGB-D scene instead of a RealSense (or PERCEPTION_FAKE=1)",
    )
    ap.add_argument(
        "--fake-scan",
        action="store_true",
        help="synthetic blocks-on-a-table scene that follows a simulated sweep "
        "(the mono-scan demo; implies a fake robot)",
    )
    ap.add_argument(
        "--serial", default=None, help="RealSense serial (default: $PERCEPTION_RS_SERIAL or first)"
    )
    ap.add_argument(
        "--rs-fps", type=int, default=None, help="RealSense stream fps (default: auto — 30, or 15 on USB 2)"
    )
    ap.add_argument("--no-align", action="store_true", help="don't align depth to the color image")
    ap.add_argument(
        "--depth-res",
        default=None,
        metavar="WxH",
        help="depth stream resolution (default: $PERCEPTION_RS_DEPTH_WIDTH x _HEIGHT, 848x480 — the "
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
        "(default: $PERCEPTION_RS_PRESET, high_accuracy; none = leave the sensor as is)",
    )
    ap.add_argument(
        "--laser-power",
        default=None,
        metavar="MW",
        help="projector power in mW, or max|none (default: $PERCEPTION_RS_LASER_POWER, max)",
    )
    ap.add_argument("--library", default=None, help="path to librealsense2 (default: $REALSENSE_LIB / auto)")


def add_robot_args(ap) -> None:
    """The robot-link flags shared by ``perception gui`` and ``perception-gui``."""
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
        dry_run=bool(getattr(args, "robot_dry_run", False)) or bool(getattr(args, "fake_scan", False)),
    )
    if getattr(args, "fake_scan", False):
        return _fake_scan_link(link)
    return link


_FAKE_SCAN_RIG = None


def _fake_scan_link(link: RobotLink):
    """``--fake-scan``: the camera and the robot link share one rig."""
    from .sweep import FakeRigLink

    return FakeRigLink(_fake_scan_rig(), link)


def _fake_scan_rig():
    global _FAKE_SCAN_RIG
    if _FAKE_SCAN_RIG is None:
        from .sweep import FakeRig

        _FAKE_SCAN_RIG = FakeRig()
    return _FAKE_SCAN_RIG


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
    ``PERCEPTION_WIDTH/HEIGHT``)? Otherwise colour follows the depth size —
    a D435 streaming 848x480 depth next to 640x480 colour returned black
    colour frames (see ``realsense.DEFAULT_DEPTH_WIDTH``)."""
    if getattr(args, "width", None) is not None or getattr(args, "height", None) is not None:
        return True
    return bool(os.environ.get("PERCEPTION_WIDTH") or os.environ.get("PERCEPTION_HEIGHT"))


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
    if getattr(args, "fake_scan", False):
        from .sweep import SweepRgbdCamera

        return SweepRgbdCamera(_fake_scan_rig())
    return open_camera(
        fake=bool(getattr(args, "fake", False)) or _env_flag("PERCEPTION_FAKE"),
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
    add_robot_args(ap)
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument(
        "--segment-backend", default=None, help="stub | sam (default: $PERCEPTION_SEGMENT_BACKEND)"
    )
    ap.add_argument(
        "--sam-model",
        default=None,
        help="SAM checkpoint id for the sam backend (default: $PERCEPTION_SAM_MODEL)",
    )
    ap.add_argument(
        "--out", default=DEFAULT_CAPTURE_ROOT, help=f"capture root (default: {DEFAULT_CAPTURE_ROOT}/)"
    )
    ap.add_argument("--bind", default="127.0.0.1", help="interface to bind (default: loopback only)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"port (default {DEFAULT_PORT})")
    ap.add_argument("--no-browser", action="store_true", help="don't open the browser automatically")
    ap.add_argument("--demo", action="store_true", help="open the demo view: one picture, four big buttons")
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
        capture_root=args.out,
        bind=args.bind,
        port=args.port,
        open_browser=not args.no_browser,
        robot=robot_from_args(args),
        demo=bool(getattr(args, "demo", False)),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
