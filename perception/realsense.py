"""Intel RealSense D4xx capture through librealsense's C API — via ``ctypes``,
no ``pyrealsense2`` (Intel ships no macOS wheel; Jetson builds are from source
anyway). The package stays dependency-free; the only requirement is the SDK's
shared library (``librealsense2``), found via ``$REALSENSE_LIB``,
``ctypes.util.find_library`` or the usual install prefixes.

Layers:

* :class:`Api` — a thin, typed wrapper over the handful of ``rs2_*`` functions
  we use. Every call goes through :meth:`Api._call`, which turns an
  ``rs2_error`` into a :class:`RealSenseError` naming the failed function.
  Enum ordinals are hard-coded from ``librealsense2/h/rs_sensor.h`` and
  **self-checked at load time** against the library's ``*_to_string``
  functions, so an SDK that renumbers an enum fails loudly instead of quietly
  requesting the wrong stream.
* :class:`RealSenseCamera` — open → :meth:`read` → close for one device:
  color (RGB8) + depth (Z16) at one resolution/fps, depth **aligned to the
  color image** by the SDK's ``align`` processing block (so a pixel names the
  same physical point in both), plus device info, intrinsics and depth scale.
  Takes an ``api`` argument so tests drive it with a fake; the hardware test
  (``-m realsense``) runs it for real.
* :class:`RealSenseSource` — the :class:`perception.sources.FrameSource`
  adapter (RGB frames only) so the existing pipeline can read from the D435.
* :class:`SyntheticRgbdCamera` — same surface as :class:`RealSenseCamera`,
  no hardware: what ``--fake`` and the viewer tests run on.

Platform notes (verified 2026-09-02 on macOS 15 / arm64, librealsense 2.58.4
from Homebrew): the library loads and deprojects fine unprivileged, but
*opening the device* needs root — libusb must detach macOS's own UVC driver
from the camera and ``failed to claim usb interface: 0 … RS2_USB_STATUS_ACCESS``
is what you get otherwise. Run the viewer/CLI under ``sudo`` on a Mac. On
Linux (the Jetson target) install the SDK's udev rules instead and no root is
needed.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import sys
import threading
import time
from collections.abc import Iterator
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from .frame import Frame
from .rgbd import DISTORTION_MODELS, DepthImage, Intrinsics, RgbdFrame, synthetic_rgbd

# ----- enum ordinals (librealsense2/h/rs_sensor.h, rs_types.h @ 2.58.4) ---------

STREAM_ANY, STREAM_DEPTH, STREAM_COLOR, STREAM_INFRARED = 0, 1, 2, 3
FORMAT_ANY, FORMAT_Z16, FORMAT_RGB8, FORMAT_BGR8 = 0, 1, 5, 6
INFO_NAME, INFO_SERIAL, INFO_FIRMWARE, INFO_PHYSICAL_PORT, INFO_PRODUCT_ID = 0, 1, 2, 4, 7
INFO_USB_TYPE, INFO_PRODUCT_LINE = 9, 10
EXTENSION_DEPTH_SENSOR = 7
LOG_SEVERITY = {"debug": 0, "info": 1, "warn": 2, "error": 3, "fatal": 4, "none": 5}

# What the library must say for each ordinal — the load-time drift check.
_ENUM_EXPECTATIONS = (
    ("rs2_stream_to_string", STREAM_DEPTH, "Depth"),
    ("rs2_stream_to_string", STREAM_COLOR, "Color"),
    ("rs2_format_to_string", FORMAT_Z16, "Z16"),
    ("rs2_format_to_string", FORMAT_RGB8, "RGB8"),
    ("rs2_camera_info_to_string", INFO_SERIAL, "Serial Number"),
    ("rs2_camera_info_to_string", INFO_USB_TYPE, "Usb Type Descriptor"),
    ("rs2_extension_to_string", EXTENSION_DEPTH_SENSOR, "Depth Sensor"),
)

_LIBRARY_CANDIDATES = (
    "/opt/homebrew/lib/librealsense2.dylib",
    "/usr/local/lib/librealsense2.dylib",
    "/usr/local/lib/librealsense2.so",
    "/usr/lib/aarch64-linux-gnu/librealsense2.so",
    "/usr/lib/x86_64-linux-gnu/librealsense2.so",
    "/usr/lib/librealsense2.so",
    "realsense2.dll",
)


class RealSenseError(RuntimeError):
    """An ``rs2_error`` surfaced from the SDK (or a binding-level failure)."""


class RealSenseLibraryNotFound(RealSenseError):
    """``librealsense2`` could not be located/loaded."""


class _rs2_intrinsics(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("ppx", ctypes.c_float),
        ("ppy", ctypes.c_float),
        ("fx", ctypes.c_float),
        ("fy", ctypes.c_float),
        ("model", ctypes.c_int),
        ("coeffs", ctypes.c_float * 5),
    ]


def find_library_path(explicit: str | None = None) -> str:
    """Locate the SDK shared library or raise :class:`RealSenseLibraryNotFound`."""
    tried: list[str] = []
    for cand in (explicit, os.environ.get("REALSENSE_LIB")):
        if cand:
            if os.path.exists(cand):
                return cand
            tried.append(cand)
    found = ctypes.util.find_library("realsense2")
    if found:
        return found
    for cand in _LIBRARY_CANDIDATES:
        if os.path.exists(cand):
            return cand
        tried.append(cand)
    raise RealSenseLibraryNotFound(
        "librealsense2 not found. Install the RealSense SDK (macOS: `brew install librealsense`; "
        "Ubuntu/Jetson: build librealsense with -DFORCE_RSUSB_BACKEND=ON, or `apt install librealsense2`) "
        f"or point REALSENSE_LIB at the shared library. Tried: {', '.join(tried)}"
    )


class Api:
    """Typed ``ctypes`` surface over the ``rs2_*`` functions the camera uses."""

    def __init__(self, library: str | None = None):
        self.path = find_library_path(library)
        try:
            self.lib = ctypes.CDLL(self.path)
        except OSError as exc:
            raise RealSenseLibraryNotFound(f"could not load {self.path}: {exc}") from exc
        self._declare()
        self.version = self._call("rs2_get_api_version")
        self._check_enums()
        self._ctx = None
        self._ctx_lock = threading.Lock()

    # -- declarations ------------------------------------------------------------

    def _declare(self) -> None:
        P = ctypes.c_void_p
        PP = ctypes.POINTER(ctypes.c_void_p)
        I = ctypes.c_int  # noqa: E741 (mirrors the C header)
        U = ctypes.c_uint
        F = ctypes.c_float
        S = ctypes.c_char_p
        sigs: dict[str, tuple[Any, list]] = {
            "rs2_get_error_message": (S, [P]),
            "rs2_get_failed_function": (S, [P]),
            "rs2_get_failed_args": (S, [P]),
            "rs2_free_error": (None, [P]),
            "rs2_get_api_version": (I, [PP]),
            "rs2_log_to_console": (None, [I, PP]),
            "rs2_stream_to_string": (S, [I]),
            "rs2_format_to_string": (S, [I]),
            "rs2_camera_info_to_string": (S, [I]),
            "rs2_extension_to_string": (S, [I]),
            "rs2_create_context": (P, [I, PP]),
            "rs2_delete_context": (None, [P]),
            "rs2_query_devices": (P, [P, PP]),
            "rs2_get_device_count": (I, [P, PP]),
            "rs2_create_device": (P, [P, I, PP]),
            "rs2_delete_device": (None, [P]),
            "rs2_delete_device_list": (None, [P]),
            "rs2_supports_device_info": (I, [P, I, PP]),
            "rs2_get_device_info": (S, [P, I, PP]),
            "rs2_query_sensors": (P, [P, PP]),
            "rs2_get_sensors_count": (I, [P, PP]),
            "rs2_create_sensor": (P, [P, I, PP]),
            "rs2_delete_sensor": (None, [P]),
            "rs2_delete_sensor_list": (None, [P]),
            "rs2_is_sensor_extendable_to": (I, [P, I, PP]),
            "rs2_get_depth_scale": (F, [P, PP]),
            "rs2_create_config": (P, [PP]),
            "rs2_delete_config": (None, [P]),
            "rs2_config_enable_stream": (None, [P, I, I, I, I, I, I, PP]),
            "rs2_config_enable_device": (None, [P, S, PP]),
            "rs2_create_pipeline": (P, [P, PP]),
            "rs2_delete_pipeline": (None, [P]),
            "rs2_pipeline_start_with_config": (P, [P, P, PP]),
            "rs2_pipeline_stop": (None, [P, PP]),
            "rs2_pipeline_wait_for_frames": (P, [P, U, PP]),
            "rs2_pipeline_profile_get_device": (P, [P, PP]),
            "rs2_pipeline_profile_get_streams": (P, [P, PP]),
            "rs2_delete_pipeline_profile": (None, [P]),
            "rs2_get_stream_profiles_count": (I, [P, PP]),
            "rs2_get_stream_profile": (P, [P, I, PP]),
            "rs2_delete_stream_profiles_list": (None, [P]),
            "rs2_get_stream_profile_data": (
                None,
                [
                    P,
                    ctypes.POINTER(I),
                    ctypes.POINTER(I),
                    ctypes.POINTER(I),
                    ctypes.POINTER(I),
                    ctypes.POINTER(I),
                    PP,
                ],
            ),
            "rs2_get_video_stream_intrinsics": (None, [P, ctypes.POINTER(_rs2_intrinsics), PP]),
            "rs2_embedded_frames_count": (I, [P, PP]),
            "rs2_extract_frame": (P, [P, I, PP]),
            "rs2_release_frame": (None, [P]),
            "rs2_get_frame_stream_profile": (P, [P, PP]),
            "rs2_get_frame_data": (P, [P, PP]),
            "rs2_get_frame_data_size": (I, [P, PP]),
            "rs2_get_frame_width": (I, [P, PP]),
            "rs2_get_frame_height": (I, [P, PP]),
            "rs2_get_frame_stride_in_bytes": (I, [P, PP]),
            "rs2_get_frame_timestamp": (ctypes.c_double, [P, PP]),
            "rs2_get_frame_number": (ctypes.c_ulonglong, [P, PP]),
            "rs2_create_align": (P, [I, PP]),
            "rs2_delete_processing_block": (None, [P]),
            "rs2_create_frame_queue": (P, [I, PP]),
            "rs2_delete_frame_queue": (None, [P]),
            "rs2_start_processing_queue": (None, [P, P, PP]),
            "rs2_process_frame": (None, [P, P, PP]),
            "rs2_wait_for_frame": (P, [P, U, PP]),
            "rs2_deproject_pixel_to_point": (None, [F * 3, ctypes.POINTER(_rs2_intrinsics), F * 2, F]),
        }
        for name, (restype, argtypes) in sigs.items():
            fn = getattr(self.lib, name)
            fn.restype = restype
            fn.argtypes = argtypes

    def _check_enums(self) -> None:
        for fn, ordinal, expected in _ENUM_EXPECTATIONS:
            got = getattr(self.lib, fn)(ordinal)
            got = got.decode() if got else ""
            if got != expected:
                raise RealSenseError(
                    f"enum drift: {fn}({ordinal}) is {got!r} in {self.path} (api {self.version}), "
                    f"expected {expected!r}; this binding was written against librealsense 2.58"
                )

    # -- error-checked call --------------------------------------------------------

    def _call(self, name: str, *args):
        err = ctypes.c_void_p()
        result = getattr(self.lib, name)(*args, ctypes.byref(err))
        if err:
            msg = (self.lib.rs2_get_error_message(err) or b"").decode(errors="replace")
            fn = (self.lib.rs2_get_failed_function(err) or b"").decode(errors="replace")
            fargs = (self.lib.rs2_get_failed_args(err) or b"").decode(errors="replace")
            self.lib.rs2_free_error(err)
            raise RealSenseError(f"{fn or name}({fargs}): {msg}")
        return result

    # -- pythonic surface (what RealSenseCamera and the tests' FakeApi share) -------

    def log_to_console(self, severity: str) -> None:
        self._call("rs2_log_to_console", LOG_SEVERITY[severity])

    def create_context(self):
        return self._call("rs2_create_context", self.version)

    def delete_context(self, ctx) -> None:
        self.lib.rs2_delete_context(ctx)

    def context(self):
        """The process-wide ``rs2_context`` (created once, never deleted).

        librealsense is built around one context per process: each context
        claims the camera's USB interfaces through its own device watcher.
        On macOS's libusb backend every claim that finds an interface held
        resets the USB device and races Apple's ``UVCAssistant`` for the
        re-enumerated interfaces (see docs/realsense.md, Troubleshooting) —
        a second context in the same process re-runs that race and, losing
        the depth interface, logs ``cannot access depth sensor`` and starts
        a pipeline that never delivers a frameset. Enumeration and streaming
        therefore share this handle; the OS reclaims it at exit.
        """
        with self._ctx_lock:
            if self._ctx is None:
                self._ctx = self.create_context()
            return self._ctx

    def list_devices(self, ctx) -> list[dict]:
        """Info dicts for every connected device (each device handle released)."""
        dev_list = self._call("rs2_query_devices", ctx)
        out = []
        try:
            for i in range(self._call("rs2_get_device_count", dev_list)):
                dev = self._call("rs2_create_device", dev_list, i)
                try:
                    out.append(self.device_info(dev))
                finally:
                    self.lib.rs2_delete_device(dev)
        finally:
            self.lib.rs2_delete_device_list(dev_list)
        return out

    def device_info(self, dev) -> dict:
        info = {}
        for key, ordinal in (
            ("name", INFO_NAME),
            ("serial", INFO_SERIAL),
            ("firmware", INFO_FIRMWARE),
            ("physical_port", INFO_PHYSICAL_PORT),
            ("product_id", INFO_PRODUCT_ID),
            ("usb_type", INFO_USB_TYPE),
            ("product_line", INFO_PRODUCT_LINE),
        ):
            if self._call("rs2_supports_device_info", dev, ordinal):
                raw = self._call("rs2_get_device_info", dev, ordinal)
                info[key] = raw.decode(errors="replace") if raw else ""
            else:
                info[key] = None
        return info

    def depth_scale(self, dev) -> float | None:
        """Metres per depth unit from the device's depth sensor (None if none)."""
        sensors = self._call("rs2_query_sensors", dev)
        try:
            for i in range(self._call("rs2_get_sensors_count", sensors)):
                sensor = self._call("rs2_create_sensor", sensors, i)
                try:
                    if self._call("rs2_is_sensor_extendable_to", sensor, EXTENSION_DEPTH_SENSOR):
                        return float(self._call("rs2_get_depth_scale", sensor))
                finally:
                    self.lib.rs2_delete_sensor(sensor)
        finally:
            self.lib.rs2_delete_sensor_list(sensors)
        return None

    def start_pipeline(self, ctx, *, width: int, height: int, fps: int, serial: str | None):
        """Start color RGB8 + depth Z16 streams; returns ``(pipeline, profile)``."""
        cfg = self._call("rs2_create_config")
        try:
            if serial:
                self._call("rs2_config_enable_device", cfg, serial.encode())
            self._call("rs2_config_enable_stream", cfg, STREAM_DEPTH, -1, width, height, FORMAT_Z16, fps)
            self._call("rs2_config_enable_stream", cfg, STREAM_COLOR, -1, width, height, FORMAT_RGB8, fps)
            pipe = self._call("rs2_create_pipeline", ctx)
            try:
                profile = self._call("rs2_pipeline_start_with_config", pipe, cfg)
            except RealSenseError:
                self.lib.rs2_delete_pipeline(pipe)
                raise
        finally:
            self.lib.rs2_delete_config(cfg)
        return pipe, profile

    def stop_pipeline(self, pipe, profile) -> None:
        if profile:
            self.lib.rs2_delete_pipeline_profile(profile)
        try:
            self._call("rs2_pipeline_stop", pipe)
        finally:
            self.lib.rs2_delete_pipeline(pipe)

    def profile_device(self, profile):
        return self._call("rs2_pipeline_profile_get_device", profile)

    def delete_device(self, dev) -> None:
        self.lib.rs2_delete_device(dev)

    def profile_streams(self, profile) -> list[dict]:
        """``[{stream, format, index, uid, fps, intrinsics}]`` for the active profile."""
        lst = self._call("rs2_pipeline_profile_get_streams", profile)
        out = []
        try:
            for i in range(self._call("rs2_get_stream_profiles_count", lst)):
                sp = self._call("rs2_get_stream_profile", lst, i)
                out.append(self._stream_profile(sp))
        finally:
            self.lib.rs2_delete_stream_profiles_list(lst)
        return out

    def _stream_profile(self, sp) -> dict:
        stream, fmt, index, uid, fps = (ctypes.c_int() for _ in range(5))
        self._call(
            "rs2_get_stream_profile_data",
            sp,
            ctypes.byref(stream),
            ctypes.byref(fmt),
            ctypes.byref(index),
            ctypes.byref(uid),
            ctypes.byref(fps),
        )
        intr = None
        try:
            raw = _rs2_intrinsics()
            self._call("rs2_get_video_stream_intrinsics", sp, ctypes.byref(raw))
            intr = _intrinsics_from_struct(raw)
        except RealSenseError:
            pass  # not a video stream (motion, pose)
        return {
            "stream": stream.value,
            "format": fmt.value,
            "index": index.value,
            "uid": uid.value,
            "fps": fps.value,
            "intrinsics": intr,
        }

    def create_align_to_color(self):
        """``(block, queue)`` for depth→color alignment; feed with :meth:`align`."""
        block = self._call("rs2_create_align", STREAM_COLOR)
        try:
            queue = self._call("rs2_create_frame_queue", 1)
            self._call("rs2_start_processing_queue", block, queue)
        except RealSenseError:
            self.lib.rs2_delete_processing_block(block)
            raise
        return block, queue

    def delete_align(self, block, queue) -> None:
        self.lib.rs2_delete_frame_queue(queue)
        self.lib.rs2_delete_processing_block(block)

    def wait_for_frames(self, pipe, timeout_ms: int):
        return self._call("rs2_pipeline_wait_for_frames", pipe, timeout_ms)

    def align(self, block, queue, frameset, timeout_ms: int):
        """Run ``frameset`` through the align block. **Consumes** ``frameset``
        (``rs2_process_frame`` takes the reference) and returns the aligned
        frameset, which the caller must release."""
        self._call("rs2_process_frame", block, frameset)
        return self._call("rs2_wait_for_frame", queue, timeout_ms)

    def release_frame(self, frame) -> None:
        self.lib.rs2_release_frame(frame)

    def split_frameset(self, frameset) -> list[dict]:
        """Extract every embedded frame → ``[{stream, width, height, stride,
        data, timestamp_ms, number, intrinsics}]``; the ``data`` is a *copy* so
        the SDK frame is released before returning."""
        out = []
        for i in range(self._call("rs2_embedded_frames_count", frameset)):
            frame = self._call("rs2_extract_frame", frameset, i)
            try:
                sp = self._call("rs2_get_frame_stream_profile", frame)
                prof = self._stream_profile(sp)
                size = self._call("rs2_get_frame_data_size", frame)
                ptr = self._call("rs2_get_frame_data", frame)
                data = ctypes.string_at(ptr, size)
                out.append(
                    {
                        "stream": prof["stream"],
                        "format": prof["format"],
                        "width": self._call("rs2_get_frame_width", frame),
                        "height": self._call("rs2_get_frame_height", frame),
                        "stride": self._call("rs2_get_frame_stride_in_bytes", frame),
                        "data": data,
                        "timestamp_ms": float(self._call("rs2_get_frame_timestamp", frame)),
                        "number": int(self._call("rs2_get_frame_number", frame)),
                        "intrinsics": prof["intrinsics"],
                    }
                )
            finally:
                self.lib.rs2_release_frame(frame)
        return out

    def deproject(self, intr: Intrinsics, u: float, v: float, depth_m: float) -> tuple[float, float, float]:
        """The SDK's own ``rs2_deproject_pixel_to_point`` (used to cross-check ours)."""
        raw = _intrinsics_to_struct(intr)
        point = (ctypes.c_float * 3)()
        pixel = (ctypes.c_float * 2)(u, v)
        self.lib.rs2_deproject_pixel_to_point(point, ctypes.byref(raw), pixel, depth_m)
        return (point[0], point[1], point[2])


def _intrinsics_from_struct(raw: _rs2_intrinsics) -> Intrinsics:
    return Intrinsics(
        width=raw.width,
        height=raw.height,
        fx=raw.fx,
        fy=raw.fy,
        ppx=raw.ppx,
        ppy=raw.ppy,
        model=DISTORTION_MODELS[raw.model],
        coeffs=tuple(float(c) for c in raw.coeffs),  # type: ignore[arg-type]
    )


def _intrinsics_to_struct(intr: Intrinsics) -> _rs2_intrinsics:
    raw = _rs2_intrinsics()
    raw.width, raw.height = intr.width, intr.height
    raw.fx, raw.fy, raw.ppx, raw.ppy = intr.fx, intr.fy, intr.ppx, intr.ppy
    raw.model = DISTORTION_MODELS.index(intr.model)
    for i, c in enumerate(intr.coeffs):
        raw.coeffs[i] = c
    return raw


_api_lock = threading.Lock()
_api: Api | None = None


def load_api(library: str | None = None) -> Api:
    """Process-wide :class:`Api` (the library is loaded once)."""
    global _api
    with _api_lock:
        if _api is None or (library and _api.path != library):
            _api = Api(library)
        return _api


# ----- cameras -------------------------------------------------------------------


@runtime_checkable
class RgbdCamera(Protocol):
    """What the viewer/CLI need from a camera: open, read RGB-D frames, close."""

    def open(self) -> None: ...
    def read(self) -> RgbdFrame: ...
    def close(self) -> None: ...
    def describe(self) -> dict: ...


@dataclass
class RealSenseCamera:
    """One D4xx device streaming aligned color + depth at ``width x height @ fps``.

    ``serial`` picks a specific camera when several are attached (``None`` =
    the first). ``align`` re-projects depth into the color image (default; what
    click-to-measure needs). ``api`` lets tests inject a fake SDK.
    """

    width: int = 640
    height: int = 480
    fps: int | None = None  # None = 30 on USB 3, 15 on a USB 2 link
    serial: str | None = None
    align: bool = True
    timeout_ms: int = 5000
    log_severity: str = "error"
    library: str | None = None
    api: Any = None
    # Populated by open():
    info: dict = field(default_factory=dict, init=False)
    intrinsics: dict = field(default_factory=dict, init=False)
    depth_scale: float | None = field(default=None, init=False)
    effective_fps: int | None = field(default=None, init=False)
    _ctx: Any = field(default=None, init=False, repr=False)
    _pipe: Any = field(default=None, init=False, repr=False)
    _profile: Any = field(default=None, init=False, repr=False)
    _align: Any = field(default=None, init=False, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)
    _frames_read: int = field(default=0, init=False, repr=False)

    def open(self) -> None:
        if self._pipe is not None:
            return
        self._frames_read = 0
        api = self.api if self.api is not None else load_api(self.library)
        self.api = api
        api.log_to_console(self.log_severity)
        self._ctx = api.context()
        try:
            self.effective_fps = self._choose_fps(api)
            self._pipe, self._profile = api.start_pipeline(
                self._ctx, width=self.width, height=self.height, fps=self.effective_fps, serial=self.serial
            )
            dev = api.profile_device(self._profile)
            try:
                self.info = api.device_info(dev)
                self.depth_scale = api.depth_scale(dev)
            finally:
                api.delete_device(dev)
            if self.depth_scale is None:
                raise RealSenseError("device has no depth sensor")
            self.intrinsics = {}
            for sp in api.profile_streams(self._profile):
                key = {STREAM_DEPTH: "depth", STREAM_COLOR: "color"}.get(sp["stream"])
                if key and sp["intrinsics"] is not None:
                    self.intrinsics[key] = sp["intrinsics"]
            if self.align:
                self._align = api.create_align_to_color()
        except Exception:
            self.close()
            raise

    def _choose_fps(self, api) -> int:
        """Explicit ``fps`` wins; otherwise 30, or 15 when the camera reports a
        USB 2 link (depth + colour at 640x480@30 exceeds USB 2's budget and
        the SDK then simply never delivers a full frameset)."""
        if self.fps:
            return int(self.fps)
        usb = None
        try:
            for d in api.list_devices(self._ctx):
                if not self.serial or d.get("serial") == self.serial:
                    usb = d.get("usb_type")
                    break
        except RealSenseError:
            pass
        if usb and str(usb).startswith("2"):
            print(
                f"realsense: USB {usb} link — capping the stream at 15 fps (pass fps=30 to force)",
                file=sys.stderr,
            )
            return 15
        return 30

    def read(self) -> RgbdFrame:
        """Block for the next color + depth pair (aligned when configured)."""
        if self._pipe is None:
            raise RealSenseError("camera is not open (call open() first)")
        api = self.api
        with self._lock:
            try:
                frameset = api.wait_for_frames(self._pipe, self.timeout_ms)
            except RealSenseError as exc:
                if "arrive" in str(exc) and self._frames_read == 0:
                    raise RealSenseError(
                        f"{exc} — the pipeline started but the first frameset never came "
                        f"(usb {self.info.get('usb_type')}, "
                        f"{self.width}x{self.height}@{self.effective_fps}). "
                        "Usual cause: the camera's USB interfaces were still held by a previous open "
                        "(another process, or one that just exited) — re-plug the camera and retry. "
                        "On a USB 2 link keep fps at 15 or lower the resolution."
                    ) from exc
                raise
            self._frames_read += 1
            if self._align is not None:
                frameset = api.align(self._align[0], self._align[1], frameset, self.timeout_ms)
            try:
                frames = api.split_frameset(frameset)
            finally:
                api.release_frame(frameset)
        color = depth = None
        for f in frames:
            if f["stream"] == STREAM_COLOR and f["format"] == FORMAT_RGB8:
                color = f
            elif f["stream"] == STREAM_DEPTH and f["format"] == FORMAT_Z16:
                depth = f
        if color is None or depth is None:
            have = [f["stream"] for f in frames]
            raise RealSenseError(f"frameset lacks color+depth (streams present: {have})")
        cf = Frame(width=color["width"], height=color["height"], data=_unstride(color, 3), channels=3)
        df = DepthImage(
            width=depth["width"], height=depth["height"], data=_unstride(depth, 2), scale_m=self.depth_scale
        )
        intr = depth["intrinsics"] or (
            self.intrinsics.get("color") if self.align else self.intrinsics.get("depth")
        )
        if intr is None:
            raise RealSenseError("no intrinsics for the depth image")
        return RgbdFrame(
            color=cf,
            depth=df,
            intrinsics=intr,
            timestamp_ms=depth["timestamp_ms"],
            frame_number=depth["number"],
            aligned=self.align,
            extra={"serial": self.info.get("serial")},
        )

    def frames(self) -> Iterator[RgbdFrame]:
        while True:
            yield self.read()

    def close(self) -> None:
        api = self.api
        if api is None:
            return
        if self._align is not None:
            api.delete_align(*self._align)
            self._align = None
        if self._pipe is not None:
            try:
                api.stop_pipeline(self._pipe, self._profile)
            finally:
                self._pipe = self._profile = None
        self._ctx = None  # shared with the process; never deleted here

    def describe(self) -> dict:
        return {
            "kind": "realsense",
            "open": self._pipe is not None,
            "device": self.info,
            "stream": {
                "width": self.width,
                "height": self.height,
                "fps": self.effective_fps or self.fps,
                "aligned": self.align,
            },
            "depth_scale_m": self.depth_scale,
            "intrinsics": {k: v.as_dict() for k, v in self.intrinsics.items()},
            "sdk": {
                "path": getattr(self.api, "path", None),
                "api_version": getattr(self.api, "version", None),
            },
        }

    def __enter__(self) -> RealSenseCamera:
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def _unstride(f: dict, bytes_per_px: int) -> bytes:
    """Drop row padding if the SDK's stride exceeds the tight row width."""
    tight = f["width"] * bytes_per_px
    stride = f["stride"] or tight
    data = f["data"]
    if stride == tight:
        return data if len(data) == tight * f["height"] else data[: tight * f["height"]]
    return b"".join(data[y * stride : y * stride + tight] for y in range(f["height"]))


def list_devices(library: str | None = None) -> list[dict]:
    """Info for every connected RealSense (opens no streams)."""
    api = load_api(library)
    api.log_to_console("error")
    return api.list_devices(api.context())


@dataclass
class SyntheticRgbdCamera:
    """Hardware-free stand-in with the :class:`RgbdCamera` surface.

    Produces :func:`synthetic_rgbd` frames paced at ``fps`` with a slowly
    drifting wall depth so a live viewer visibly updates.
    """

    width: int = 640
    height: int = 480
    fps: int = 15
    _n: int = field(default=0, init=False)
    _open: bool = field(default=False, init=False)
    _t0: float = field(default=0.0, init=False)

    def open(self) -> None:
        self._open = True
        self._t0 = time.monotonic()

    def read(self) -> RgbdFrame:
        if not self._open:
            raise RuntimeError("camera is not open")
        self._n += 1
        if self.fps > 0:
            time.sleep(1.0 / self.fps)
        wall = 1.20 + 0.10 * ((self._n % 60) / 60.0)
        return synthetic_rgbd(
            self.width,
            self.height,
            frame_number=self._n,
            timestamp_ms=(time.monotonic() - self._t0) * 1000.0,
            wall_m=wall,
        )

    def close(self) -> None:
        self._open = False

    def describe(self) -> dict:
        return {
            "kind": "synthetic",
            "open": self._open,
            "device": {"name": "Synthetic RGB-D", "serial": "SYNTH", "usb_type": None},
            "stream": {"width": self.width, "height": self.height, "fps": self.fps, "aligned": True},
            "depth_scale_m": 0.001,
            "intrinsics": {
                "color": synthetic_rgbd(self.width, self.height, holes=False).intrinsics.as_dict()
            },
        }


class RealSenseSource:
    """:class:`perception.sources.FrameSource` adapter: RGB frames from a D4xx.

    Keeps the full :class:`RealSenseCamera` on ``.camera`` for callers that want
    the depth too; :meth:`frames` yields only the color :class:`Frame` so the
    existing (monocular) pipeline runs unchanged on real camera frames.
    """

    def __init__(self, camera: RealSenseCamera | None = None, **camera_kwargs):
        self.camera = camera or RealSenseCamera(**camera_kwargs)

    def open(self) -> None:
        self.camera.open()

    def frames(self) -> Iterator[Frame]:
        while True:
            yield self.camera.read().color

    def read_one(self) -> Frame:
        opened_here = self.camera._pipe is None
        if opened_here:
            self.open()
        try:
            return self.camera.read().color
        finally:
            if opened_here:
                self.close()

    def close(self) -> None:
        self.camera.close()

    def __enter__(self) -> RealSenseSource:
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def open_camera(
    *,
    fake: bool = False,
    width: int = 640,
    height: int = 480,
    fps: int | None = None,
    serial: str | None = None,
    align: bool = True,
    library: str | None = None,
) -> RgbdCamera:
    """Factory used by the CLI/viewer: a real D4xx, or the synthetic stand-in."""
    if fake:
        return SyntheticRgbdCamera(width=width, height=height, fps=min(fps or 15, 15))
    return RealSenseCamera(width=width, height=height, fps=fps, serial=serial, align=align, library=library)


def _is_root() -> bool:
    geteuid = getattr(os, "geteuid", None)
    return bool(geteuid) and geteuid() == 0


def platform_hint(exc: BaseException) -> str:
    """Turn the SDK's opaque access failure into the fix for this OS."""
    text = str(exc)
    if "power state" in text or "RS2_USB_STATUS_ACCESS" in text or "claim usb interface" in text:
        if sys.platform == "darwin":
            if _is_root():
                return (
                    "librealsense could not claim the camera's USB interface even as root, so something "
                    "else holds it: a process that just exited (macOS releases the claim seconds late), "
                    "or an app with the camera open (browser, FaceTime, realsense-viewer). Wait a few "
                    "seconds and retry; if it persists, unplug and re-plug the camera."
                )
            return (
                "librealsense could not claim the camera's USB interface. On macOS the SDK needs root to "
                "detach the built-in UVC driver: re-run under `sudo` "
                "(e.g. `sudo uv run perception rs-info`). "
                "Also make sure no other app (browser, FaceTime, realsense-viewer) has the camera open."
            )
        return (
            "librealsense could not claim the camera's USB interface. On Linux install the SDK's udev rules "
            "(99-realsense-libusb.rules) and re-plug the camera, or run as root / add the user to `plugdev`."
        )
    if "didn't arrive" in text or "did not arrive" in text:
        return (
            "the SDK opened the camera but delivered no frames. On macOS this is almost always a stale USB "
            "claim from a previous open (another process, or one that just exited): unplug and re-plug the "
            "camera, then retry. On a USB 2 link use 15 fps."
        )
    if "No device" in text or "device count" in text.lower():
        return (
            "no RealSense device found — check the USB3 cable and that the camera enumerates "
            "(system_profiler / lsusb)."
        )
    return ""
