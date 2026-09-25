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
  Depth streams at its own resolution (848x480 by default — the D435's native
  depth mode) and runs through the SDK's post-processing chain
  (:class:`DepthFilters`: disparity → spatial → temporal → depth [→ hole
  filling]) *before* alignment, and the depth sensor gets a visual preset +
  laser power (:class:`DepthTuning`) at open. Takes an ``api`` argument so
  tests drive it with a fake; the hardware test (``-m realsense``) runs it for
  real.
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
FORMAT_ANY, FORMAT_Z16, FORMAT_RGB8, FORMAT_BGR8, FORMAT_Y8 = 0, 1, 5, 6, 9
INFO_NAME, INFO_SERIAL, INFO_FIRMWARE, INFO_PHYSICAL_PORT, INFO_PRODUCT_ID = 0, 1, 2, 4, 7
INFO_USB_TYPE, INFO_PRODUCT_LINE = 9, 10
EXTENSION_DEPTH_SENSOR = 7
# rs2_option (rs_option.h): sensor + processing-block options we touch.
OPTION_VISUAL_PRESET, OPTION_LASER_POWER, OPTION_EMITTER_ENABLED = 12, 13, 18
OPTION_MIN_DISTANCE, OPTION_MAX_DISTANCE = 33, 34
OPTION_FILTER_MAGNITUDE, OPTION_FILTER_SMOOTH_ALPHA, OPTION_FILTER_SMOOTH_DELTA = 36, 37, 38
OPTION_HOLES_FILL = 39  # hole-filling mode on the hole filter; *persistency index* on the temporal filter
# rs2_rs400_visual_preset: the depth sensor's recommended option sets.
VISUAL_PRESETS = {
    "custom": 0,
    "default": 1,
    "hand": 2,
    "high_accuracy": 3,
    "high_density": 4,
    "medium_density": 5,
    "remove_ir_pattern": 6,
}
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
    ("rs2_option_to_string", OPTION_VISUAL_PRESET, "Visual Preset"),
    ("rs2_option_to_string", OPTION_LASER_POWER, "Laser Power"),
    ("rs2_option_to_string", OPTION_EMITTER_ENABLED, "Emitter Enabled"),
    ("rs2_option_to_string", OPTION_FILTER_MAGNITUDE, "Filter Magnitude"),
    ("rs2_option_to_string", OPTION_HOLES_FILL, "Holes Fill"),
    ("rs2_rs400_visual_preset_to_string", VISUAL_PRESETS["high_accuracy"], "High Accuracy"),
)

# Processing blocks by short name -> (constructor, fixed args). Every one is fed
# a whole frameset: the SDK filters the depth member and re-composes the set.
_FILTER_FACTORIES: dict[str, tuple[str, tuple]] = {
    "decimation": ("rs2_create_decimation_filter_block", ()),
    "threshold": ("rs2_create_threshold", ()),
    "to_disparity": ("rs2_create_disparity_transform_block", (1,)),
    "spatial": ("rs2_create_spatial_filter_block", ()),
    "temporal": ("rs2_create_temporal_filter_block", ()),
    "to_depth": ("rs2_create_disparity_transform_block", (0,)),
    "hole_filling": ("rs2_create_hole_filling_filter_block", ()),
}

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


class _rs2_extrinsics(ctypes.Structure):
    _fields_ = [("rotation", ctypes.c_float * 9), ("translation", ctypes.c_float * 3)]


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
            "rs2_option_to_string": (S, [I]),
            "rs2_rs400_visual_preset_to_string": (S, [I]),
            "rs2_supports_option": (I, [P, I, PP]),
            "rs2_get_option": (F, [P, I, PP]),
            "rs2_set_option": (None, [P, I, F, PP]),
            "rs2_get_option_range": (
                None,
                [P, I, ctypes.POINTER(F), ctypes.POINTER(F), ctypes.POINTER(F), ctypes.POINTER(F), PP],
            ),
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
            "rs2_get_sensor_info": (S, [P, I, PP]),
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
            "rs2_get_stream_profiles": (P, [P, PP]),  # every profile a *sensor* offers
            "rs2_get_video_stream_resolution": (None, [P, ctypes.POINTER(I), ctypes.POINTER(I), PP]),
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
            "rs2_get_extrinsics": (None, [P, P, ctypes.POINTER(_rs2_extrinsics), PP]),
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
            "rs2_create_decimation_filter_block": (P, [PP]),
            "rs2_create_threshold": (P, [PP]),
            "rs2_create_disparity_transform_block": (P, [ctypes.c_ubyte, PP]),
            "rs2_create_spatial_filter_block": (P, [PP]),
            "rs2_create_temporal_filter_block": (P, [PP]),
            "rs2_create_hole_filling_filter_block": (P, [PP]),
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

    def stream_modes(self, ctx, serial: str | None = None) -> list[dict]:
        """Every video stream profile the device offers, as
        ``[{stream, index, format, width, height, fps}]`` — what
        ``rs-enumerate-devices`` prints. On a USB 2 link this list is *shorter*
        than the datasheet (the D435 drops 848x480 colour entirely), so it is
        the only truthful input for choosing a mode. ``serial`` picks a device
        (``None`` = the first); ``[]`` when no device matches."""
        dev_list = self._call("rs2_query_devices", ctx)
        out: list[dict] = []
        try:
            for i in range(self._call("rs2_get_device_count", dev_list)):
                dev = self._call("rs2_create_device", dev_list, i)
                try:
                    if serial and self.device_info(dev).get("serial") != serial:
                        continue
                    out = self._device_stream_modes(dev)
                    break
                finally:
                    self.lib.rs2_delete_device(dev)
        finally:
            self.lib.rs2_delete_device_list(dev_list)
        return out

    def _device_stream_modes(self, dev) -> list[dict]:
        out: list[dict] = []
        sensors = self._call("rs2_query_sensors", dev)
        try:
            for i in range(self._call("rs2_get_sensors_count", sensors)):
                sensor = self._call("rs2_create_sensor", sensors, i)
                try:
                    lst = self._call("rs2_get_stream_profiles", sensor)
                    try:
                        for j in range(self._call("rs2_get_stream_profiles_count", lst)):
                            sp = self._call("rs2_get_stream_profile", lst, j)
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
                            w, h = ctypes.c_int(), ctypes.c_int()
                            try:
                                self._call(
                                    "rs2_get_video_stream_resolution", sp, ctypes.byref(w), ctypes.byref(h)
                                )
                            except RealSenseError:
                                continue  # motion / pose profile
                            out.append(
                                {
                                    "stream": stream.value,
                                    "index": index.value,
                                    "format": fmt.value,
                                    "width": w.value,
                                    "height": h.value,
                                    "fps": fps.value,
                                }
                            )
                    finally:
                        self.lib.rs2_delete_stream_profiles_list(lst)
                finally:
                    self.lib.rs2_delete_sensor(sensor)
        finally:
            self.lib.rs2_delete_sensor_list(sensors)
        return out

    def sensor_options(self, dev) -> list[dict]:
        """Every sensor of ``dev`` with every option it supports:
        ``[{name, options: {option name: {value, min, max, step, default}}}]``.
        Names come from ``rs2_option_to_string`` so nothing here depends on
        ordinals; a read-only failure on one option is recorded as ``error``."""
        out = []
        sensors = self._call("rs2_query_sensors", dev)
        try:
            for i in range(self._call("rs2_get_sensors_count", sensors)):
                sensor = self._call("rs2_create_sensor", sensors, i)
                try:
                    name = "sensor"
                    try:
                        raw = self._call("rs2_get_sensor_info", sensor, INFO_NAME)
                        name = raw.decode(errors="replace") if raw else name
                    except RealSenseError:
                        pass
                    options: dict[str, dict] = {}
                    for ordinal in range(256):
                        label = (self.lib.rs2_option_to_string(ordinal) or b"").decode()
                        if label == "UNKNOWN":
                            break
                        try:
                            if not self.supports_option(sensor, ordinal):
                                continue
                            lo, hi, step, default = self.option_range(sensor, ordinal)
                            options[label] = {
                                "value": self.get_option(sensor, ordinal),
                                "min": lo,
                                "max": hi,
                                "step": step,
                                "default": default,
                            }
                        except RealSenseError as exc:
                            options[label] = {"error": str(exc)}
                    out.append({"name": name, "options": options})
                finally:
                    self.lib.rs2_delete_sensor(sensor)
        finally:
            self.lib.rs2_delete_sensor_list(sensors)
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

    def depth_sensor(self, dev):
        """Handle of ``dev``'s depth sensor (release with :meth:`delete_sensor`), or None."""
        sensors = self._call("rs2_query_sensors", dev)
        try:
            for i in range(self._call("rs2_get_sensors_count", sensors)):
                sensor = self._call("rs2_create_sensor", sensors, i)
                if self._call("rs2_is_sensor_extendable_to", sensor, EXTENSION_DEPTH_SENSOR):
                    return sensor
                self.lib.rs2_delete_sensor(sensor)
        finally:
            self.lib.rs2_delete_sensor_list(sensors)
        return None

    def delete_sensor(self, sensor) -> None:
        self.lib.rs2_delete_sensor(sensor)

    def depth_scale(self, dev) -> float | None:
        """Metres per depth unit from the device's depth sensor (None if none)."""
        sensor = self.depth_sensor(dev)
        if sensor is None:
            return None
        try:
            return float(self._call("rs2_get_depth_scale", sensor))
        finally:
            self.lib.rs2_delete_sensor(sensor)

    # ``rs2_options*`` is the base of both sensors and processing blocks in the C
    # API, so one set of helpers serves the depth sensor and the filter chain.

    def supports_option(self, handle, option: int) -> bool:
        return bool(self._call("rs2_supports_option", handle, option))

    def get_option(self, handle, option: int) -> float:
        return float(self._call("rs2_get_option", handle, option))

    def set_option(self, handle, option: int, value: float) -> None:
        self._call("rs2_set_option", handle, option, float(value))

    def option_range(self, handle, option: int) -> tuple[float, float, float, float]:
        """``(min, max, step, default)`` of ``option`` on ``handle``."""
        lo, hi, step, default = (ctypes.c_float() for _ in range(4))
        self._call(
            "rs2_get_option_range",
            handle,
            option,
            ctypes.byref(lo),
            ctypes.byref(hi),
            ctypes.byref(step),
            ctypes.byref(default),
        )
        return lo.value, hi.value, step.value, default.value

    def start_pipeline(
        self,
        ctx,
        *,
        width: int,
        height: int,
        fps: int,
        serial: str | None,
        depth_width: int | None = None,
        depth_height: int | None = None,
        infrared: bool = False,
    ):
        """Start color RGB8 (``width x height``) + depth Z16 (``depth_width x
        depth_height``, defaulting to the colour size) and, with ``infrared``,
        the **left** IR imager (index 1, Y8, the depth frame's own optics —
        global shutter on a D435); returns ``(pipeline, profile)``."""
        dw, dh = depth_width or width, depth_height or height
        cfg = self._call("rs2_create_config")
        try:
            if serial:
                self._call("rs2_config_enable_device", cfg, serial.encode())
            self._call("rs2_config_enable_stream", cfg, STREAM_DEPTH, -1, dw, dh, FORMAT_Z16, fps)
            self._call("rs2_config_enable_stream", cfg, STREAM_COLOR, -1, width, height, FORMAT_RGB8, fps)
            if infrared:
                self._call("rs2_config_enable_stream", cfg, STREAM_INFRARED, 1, dw, dh, FORMAT_Y8, fps)
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

    def depth_to_color_extrinsics(self, profile) -> dict | None:
        """``rs2_get_extrinsics(depth, color)`` of the active profile: ``{rotation:
        [[..3]]*3 (row-major), translation: [x, y, z] m}`` mapping a *depth*-frame
        point into the *colour* frame (``p_color = R p_depth + t``); None when
        either stream is absent."""
        lst = self._call("rs2_pipeline_profile_get_streams", profile)
        try:
            handles: dict[int, Any] = {}
            for i in range(self._call("rs2_get_stream_profiles_count", lst)):
                sp = self._call("rs2_get_stream_profile", lst, i)
                handles[self._stream_profile(sp)["stream"]] = sp
            if STREAM_DEPTH not in handles or STREAM_COLOR not in handles:
                return None
            raw = _rs2_extrinsics()
            self._call("rs2_get_extrinsics", handles[STREAM_DEPTH], handles[STREAM_COLOR], ctypes.byref(raw))
        finally:
            self.lib.rs2_delete_stream_profiles_list(lst)
        r = [float(v) for v in raw.rotation]  # librealsense stores this column-major
        return {
            "rotation": [[r[0], r[3], r[6]], [r[1], r[4], r[7]], [r[2], r[5], r[8]]],
            "translation": [float(v) for v in raw.translation],
        }

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

    def _start_block(self, block):
        """Attach a one-deep output queue to ``block`` → ``(block, queue)``."""
        try:
            queue = self._call("rs2_create_frame_queue", 1)
            self._call("rs2_start_processing_queue", block, queue)
        except RealSenseError:
            self.lib.rs2_delete_processing_block(block)
            raise
        return block, queue

    def create_align_to_color(self):
        """``(block, queue)`` for depth→color alignment; feed with :meth:`align`."""
        return self._start_block(self._call("rs2_create_align", STREAM_COLOR))

    def delete_align(self, block, queue) -> None:
        self.lib.rs2_delete_frame_queue(queue)
        self.lib.rs2_delete_processing_block(block)

    def create_filter(self, kind: str, options: dict[int, float] | None = None):
        """``(block, queue)`` for one post-processing filter (a :data:`_FILTER_FACTORIES`
        key) with ``options`` (``OPTION_*`` → value) applied; feed with :meth:`process`."""
        ctor, fixed = _FILTER_FACTORIES[kind]
        block, queue = self._start_block(self._call(ctor, *fixed))
        try:
            for option, value in (options or {}).items():
                self.set_option(block, option, value)
        except RealSenseError:
            self.delete_filter(block, queue)
            raise
        return block, queue

    def process(self, block, queue, frameset, timeout_ms: int):
        """Run ``frameset`` through a filter block; **consumes** ``frameset`` and
        returns the filtered frameset (same contract as :meth:`align`)."""
        self._call("rs2_process_frame", block, frameset)
        return self._call("rs2_wait_for_frame", queue, timeout_ms)

    delete_filter = delete_align

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


# The D435's native depth mode. Its stereo ASIC matches at 848x480 and derives
# the smaller modes by downscaling, so this is the accuracy-optimal choice
# (Intel's D400 tuning guide). Colour defaults to the *same* size: on this
# D435 (fw 5.12.7.100, macOS libusb backend, 2026-09-04) depth at 848x480 with
# colour at 640x480 delivered all-black colour frames — sensor options all at
# factory, lit room — while 640/640 and 848/848 were both fine. No mechanism
# known; matching the two sidesteps it and the hardware test checks for it.
DEFAULT_DEPTH_WIDTH, DEFAULT_DEPTH_HEIGHT = 848, 480
LASER_MAX = -1.0  # DepthTuning.laser_power sentinel: whatever the sensor's range allows


def _usb_type(api, ctx, serial: str | None) -> str | None:
    """The ``usb_type`` descriptor ("2.1", "3.2") of the selected device, or None."""
    try:
        for d in api.list_devices(ctx):
            if not serial or d.get("serial") == serial:
                return d.get("usb_type")
    except RealSenseError:
        pass
    return None


def negotiate_mode(
    modes: list[dict],
    *,
    width: int,
    height: int,
    depth_width: int,
    depth_height: int,
    fps: int,
) -> dict:
    """Pick a colour + depth mode the camera *actually offers*.

    ``modes`` is :meth:`Api.stream_modes` (what ``rs-enumerate-devices``
    lists). The request is kept verbatim when both ``RGB8 width x height @
    fps`` and ``Z16 depth_width x depth_height @ fps`` are in it. Otherwise
    the two streams are put at the **same** size (mixed sizes give black
    colour frames, see :data:`DEFAULT_DEPTH_WIDTH`) at the highest common
    rate not above ``fps``: the requested depth size when both sensors offer
    it, else the size with the fastest shared rate (largest of those) — rate
    before pixels, a 1280x720 @ 6 Hz cockpit is not a live view. An empty ``modes`` (enumeration
    unsupported or failed) keeps the request untouched, as does a list with
    no common size at all: the SDK's own error then says what is wrong.

    Ground truth this encodes (D435 fw 5.12.7.100 on a **USB 2.1** link,
    2026-09-24): colour is offered at 424x240 / 640x480 / 1280x720 only —
    no 848x480 — and depth 848x480 only at 10 / 6 Hz, so the USB 3 default
    (848x480 @ 15) is unresolvable and the pipeline refuses to start
    (``Couldn't resolve requests``). The pair both sensors share at 15 Hz is
    640x480, which is what this returns.

    Returns ``{width, height, depth_width, depth_height, fps, changed, reason}``.
    """
    want = {
        "width": width,
        "height": height,
        "depth_width": depth_width,
        "depth_height": depth_height,
        "fps": fps,
        "changed": False,
        "reason": None,
    }
    if not modes:
        return want
    color = {
        (m["width"], m["height"], m["fps"])
        for m in modes
        if m["stream"] == STREAM_COLOR and m["format"] == FORMAT_RGB8
    }
    depth = {
        (m["width"], m["height"], m["fps"])
        for m in modes
        if m["stream"] == STREAM_DEPTH and m["format"] == FORMAT_Z16
    }
    if not color or not depth:
        return want  # a sensor is missing from the list — let the SDK report it
    if (width, height, fps) in color and (depth_width, depth_height, fps) in depth:
        return want
    # Same-size pairs with a common rate <= the requested one.
    common: dict[tuple[int, int], int] = {}
    for w, h, f in depth:
        if f <= fps and (w, h, f) in color:
            common[(w, h)] = max(common.get((w, h), 0), f)
    if not common:
        return want
    if (depth_width, depth_height) in common:
        size = (depth_width, depth_height)
    else:  # the fastest shared pair, the biggest of those (a live view needs rate before pixels)
        size = max(common, key=lambda s: (common[s], s[0] * s[1]))
    rate = common[size]
    missing = []
    if (width, height, fps) not in color:
        missing.append(f"colour {width}x{height}@{fps}")
    if (depth_width, depth_height, fps) not in depth:
        missing.append(f"depth {depth_width}x{depth_height}@{fps}")
    return {
        "width": size[0],
        "height": size[1],
        "depth_width": size[0],
        "depth_height": size[1],
        "fps": rate,
        "changed": True,
        "reason": (
            f"camera does not offer {' / '.join(missing)}; streaming both at {size[0]}x{size[1]}@{rate}"
        ),
    }


@dataclass(frozen=True)
class DepthFilters:
    """librealsense's post-processing chain, applied to the depth frame *before*
    alignment in Intel's recommended order: (threshold →) depth→disparity →
    spatial → temporal → disparity→depth (→ hole filling). Parameter defaults
    are the SDK's own; ranges are in ``rs_processing.h``.

    * **spatial** — edge-preserving smoothing across the image (``magnitude``
      iterations 1..5, ``alpha`` 0.25..1 with lower = smoother, ``delta`` 1..50
      depth units: the step that counts as an edge and is left alone).
    * **temporal** — per-pixel exponential smoothing over frames (``alpha``
      0..1 with lower = smoother, ``delta`` as above) plus a ``persistence``
      index 0..8 controlling how many recent frames must agree before a pixel
      is trusted (3 = valid in 2 of the last 4; 0 = off; 8 = always). This is
      the one that steadies a static scene at the cost of lag on moving objects.
    * **hole_filling** — ``None`` = off (the default: it *invents* depth where
      the sensor had none, which is wrong for measurement); 0 = fill from the
      left, 1 = farthest neighbour, 2 = nearest neighbour.
    * **min_m / max_m** — the threshold filter; zeroes depth outside the band.
    """

    spatial: bool = True
    spatial_magnitude: int = 2
    spatial_alpha: float = 0.5
    spatial_delta: int = 20
    temporal: bool = True
    temporal_alpha: float = 0.4
    temporal_delta: int = 20
    temporal_persistence: int = 3
    hole_filling: int | None = None
    disparity: bool = True
    min_m: float | None = None
    max_m: float | None = None

    def chain(self) -> list[tuple[str, dict[int, float]]]:
        """``[(filter kind, {OPTION_* : value})]`` in application order."""
        out: list[tuple[str, dict[int, float]]] = []
        if self.min_m is not None or self.max_m is not None:
            opts: dict[int, float] = {}
            if self.min_m is not None:
                opts[OPTION_MIN_DISTANCE] = float(self.min_m)
            if self.max_m is not None:
                opts[OPTION_MAX_DISTANCE] = float(self.max_m)
            out.append(("threshold", opts))
        smoothing = self.spatial or self.temporal
        if smoothing and self.disparity:
            out.append(("to_disparity", {}))
        if self.spatial:
            out.append(
                (
                    "spatial",
                    {
                        OPTION_FILTER_MAGNITUDE: float(self.spatial_magnitude),
                        OPTION_FILTER_SMOOTH_ALPHA: float(self.spatial_alpha),
                        OPTION_FILTER_SMOOTH_DELTA: float(self.spatial_delta),
                    },
                )
            )
        if self.temporal:
            out.append(
                (
                    "temporal",
                    {
                        OPTION_FILTER_SMOOTH_ALPHA: float(self.temporal_alpha),
                        OPTION_FILTER_SMOOTH_DELTA: float(self.temporal_delta),
                        OPTION_HOLES_FILL: float(self.temporal_persistence),
                    },
                )
            )
        if smoothing and self.disparity:
            out.append(("to_depth", {}))
        if self.hole_filling is not None:
            out.append(("hole_filling", {OPTION_HOLES_FILL: float(self.hole_filling)}))
        return out

    def as_dict(self) -> dict:
        return {"chain": [kind for kind, _ in self.chain()], **self.__dict__}


@dataclass(frozen=True)
class DepthTuning:
    """Depth-sensor options set once at open (best effort — an unsupported or
    refused option is recorded in ``RealSenseCamera.tuning_applied``, never
    fatal). ``None`` for any field leaves the sensor as it is (what you want
    when the camera was tuned in realsense-viewer).

    * ``preset`` — a :data:`VISUAL_PRESETS` key. ``high_accuracy`` raises the
      stereo confidence threshold: fewer pixels, far fewer wrong ones.
    * ``laser_power`` — projector power in mW (D435: 0..360, default 150);
      :data:`LASER_MAX` = the sensor's maximum. More texture on flat surfaces.
    * ``emitter`` — projector on/off.
    """

    preset: str | None = "high_accuracy"
    laser_power: float | None = LASER_MAX
    emitter: bool | None = True

    def __post_init__(self) -> None:
        if self.preset is not None and self.preset not in VISUAL_PRESETS:
            raise ValueError(f"unknown visual preset {self.preset!r}; one of {sorted(VISUAL_PRESETS)}")
        if self.laser_power is not None and self.laser_power != LASER_MAX and self.laser_power < 0:
            raise ValueError("laser_power must be >= 0 mW, LASER_MAX, or None")

    def as_dict(self) -> dict:
        return dict(self.__dict__)


DEFAULT_DEPTH_FILTERS = DepthFilters()
DEFAULT_DEPTH_TUNING = DepthTuning()


@dataclass
class RealSenseCamera:
    """One D4xx device streaming aligned color + depth at ``width x height @ fps``.

    ``serial`` picks a specific camera when several are attached (``None`` =
    the first). ``align`` re-projects depth into the color image (default; what
    click-to-measure needs). Depth streams at ``depth_width x depth_height``
    (default 848x480, see :data:`DEFAULT_DEPTH_WIDTH`; colour defaults to the
    same size — see the note there) and, unless ``filters``
    is ``None``, runs through :class:`DepthFilters` before alignment;
    ``tuning`` (:class:`DepthTuning`, or ``None`` to leave the sensor alone)
    is applied at open. ``api`` lets tests inject a fake SDK.
    """

    width: int = DEFAULT_DEPTH_WIDTH
    height: int = DEFAULT_DEPTH_HEIGHT
    fps: int | None = None  # None = 30 on USB 3, 15 on a USB 2 link
    serial: str | None = None
    align: bool = True
    depth_width: int = DEFAULT_DEPTH_WIDTH
    depth_height: int = DEFAULT_DEPTH_HEIGHT
    filters: DepthFilters | None = DEFAULT_DEPTH_FILTERS
    tuning: DepthTuning | None = DEFAULT_DEPTH_TUNING
    infrared: bool = False  # also stream the left IR imager (RgbdFrame.extra["ir"]) — unverified on hardware
    timeout_ms: int = 5000
    log_severity: str = "error"
    library: str | None = None
    api: Any = None
    # Populated by open():
    info: dict = field(default_factory=dict, init=False)
    intrinsics: dict = field(default_factory=dict, init=False)
    depth_scale: float | None = field(default=None, init=False)
    effective_fps: int | None = field(default=None, init=False)
    negotiated: str | None = field(default=None, init=False)  # why open() changed the requested mode
    tuning_applied: dict = field(default_factory=dict, init=False)  # filled on the first read()
    extrinsics_depth_to_color: dict | None = field(default=None, init=False)
    _tuning_pending: bool = field(default=False, init=False, repr=False)
    _ctx: Any = field(default=None, init=False, repr=False)
    _pipe: Any = field(default=None, init=False, repr=False)
    _profile: Any = field(default=None, init=False, repr=False)
    _align: Any = field(default=None, init=False, repr=False)
    _filters: list = field(default_factory=list, init=False, repr=False)
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
            self._negotiate(api)
            start_kwargs: dict = {}
            if self.infrared:
                start_kwargs["infrared"] = True
            self._pipe, self._profile = api.start_pipeline(
                self._ctx,
                width=self.width,
                height=self.height,
                fps=self.effective_fps,
                serial=self.serial,
                depth_width=self.depth_width,
                depth_height=self.depth_height,
                **start_kwargs,
            )
            dev = api.profile_device(self._profile)
            try:
                self.info = api.device_info(dev)
                self.depth_scale = api.depth_scale(dev)
                if self.depth_scale is None:
                    raise RealSenseError("device has no depth sensor")
            finally:
                api.delete_device(dev)
            self.tuning_applied = {}
            self._tuning_pending = self.tuning is not None
            self.intrinsics = {}
            for sp in api.profile_streams(self._profile):
                key = {STREAM_DEPTH: "depth", STREAM_COLOR: "color", STREAM_INFRARED: "infrared"}.get(
                    sp["stream"]
                )
                if key and sp["intrinsics"] is not None:
                    self.intrinsics[key] = sp["intrinsics"]
            try:
                self.extrinsics_depth_to_color = api.depth_to_color_extrinsics(self._profile)
            except RealSenseError:
                self.extrinsics_depth_to_color = None
            if self.filters is not None:
                for kind, options in self.filters.chain():
                    self._filters.append(api.create_filter(kind, options))
            if self.align:
                self._align = api.create_align_to_color()
        except Exception:
            self.close()
            raise

    def _apply_tuning(self, api, dev) -> dict:
        """Push :attr:`tuning` onto the depth sensor; ``{name: {ok, value|error}}``.

        Called from :meth:`read` once the **first frameset has arrived**, never
        at open: writing the preset/laser/emitter right after
        ``rs2_pipeline_start`` on a freshly claimed camera left the pipeline
        without a single frame (macOS libusb backend, D435 fw 5.12.7.100,
        2026-09-04 — deterministic on the process's first open, fine on a
        re-open where the stream was already running). With the stream proven
        alive the same writes take effect immediately.
        """
        applied: dict[str, dict] = {}
        if self.tuning is None:
            return applied
        sensor = api.depth_sensor(dev)
        if sensor is None:
            return applied
        t = self.tuning
        # Preset first: presets rewrite laser power and the emitter, so the
        # explicit values must land after it.
        steps: list[tuple[str, int, float, Any]] = []
        if t.preset is not None:
            steps.append(("preset", OPTION_VISUAL_PRESET, float(VISUAL_PRESETS[t.preset]), t.preset))
        if t.emitter is not None:
            steps.append(("emitter", OPTION_EMITTER_ENABLED, 1.0 if t.emitter else 0.0, bool(t.emitter)))
        if t.laser_power is not None:
            steps.append(("laser_power", OPTION_LASER_POWER, float(t.laser_power), None))
        try:
            for name, option, value, shown in steps:
                try:
                    if not api.supports_option(sensor, option):
                        applied[name] = {"ok": False, "error": "unsupported by this sensor"}
                        continue
                    if name == "laser_power" and value == LASER_MAX:
                        value = api.option_range(sensor, option)[1]
                    api.set_option(sensor, option, value)
                    applied[name] = {"ok": True, "value": value if shown is None else shown}
                except RealSenseError as exc:
                    applied[name] = {"ok": False, "error": str(exc)}
        finally:
            api.delete_sensor(sensor)
        return applied

    def _negotiate(self, api) -> None:
        """Replace the requested mode with one the camera offers (see
        :func:`negotiate_mode`). Enumeration failures are not fatal — the
        request stands and the SDK's start error speaks for itself."""
        self.negotiated = None
        try:
            modes = api.stream_modes(self._ctx, self.serial)
        except RealSenseError as exc:
            print(
                f"realsense: could not enumerate stream modes ({exc}); requesting the configured mode",
                file=sys.stderr,
            )
            return
        pick = negotiate_mode(
            modes,
            width=self.width,
            height=self.height,
            depth_width=self.depth_width,
            depth_height=self.depth_height,
            fps=int(self.effective_fps or 30),
        )
        if not pick["changed"]:
            return
        self.width, self.height = pick["width"], pick["height"]
        self.depth_width, self.depth_height = pick["depth_width"], pick["depth_height"]
        self.effective_fps = pick["fps"]
        self.negotiated = pick["reason"]
        usb = self.info.get("usb_type") or _usb_type(api, self._ctx, self.serial) or "?"
        print(f"realsense: USB {usb} link — {pick['reason']}", file=sys.stderr)

    def _choose_fps(self, api) -> int:
        """Explicit ``fps`` wins; otherwise 30, or 15 when the camera reports a
        USB 2 link (depth + colour at 640x480@30 exceeds USB 2's budget and
        the SDK then simply never delivers a full frameset)."""
        if self.fps:
            return int(self.fps)
        usb = _usb_type(api, self._ctx, self.serial)
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
            if self._tuning_pending:
                self._tuning_pending = False
                dev = api.profile_device(self._profile)
                try:
                    self.tuning_applied = self._apply_tuning(api, dev)
                finally:
                    api.delete_device(dev)
            for block, queue in self._filters:
                frameset = api.process(block, queue, frameset, self.timeout_ms)
            if self._align is not None:
                frameset = api.align(self._align[0], self._align[1], frameset, self.timeout_ms)
            try:
                frames = api.split_frameset(frameset)
            finally:
                api.release_frame(frameset)
        color = depth = ir = None
        for f in frames:
            if f["stream"] == STREAM_COLOR and f["format"] == FORMAT_RGB8:
                color = f
            elif f["stream"] == STREAM_DEPTH and f["format"] == FORMAT_Z16:
                depth = f
            elif f["stream"] == STREAM_INFRARED and f["format"] == FORMAT_Y8:
                ir = f
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
        extra: dict = {"serial": self.info.get("serial")}
        if ir is not None:
            extra["ir"] = Frame(width=ir["width"], height=ir["height"], data=_unstride(ir, 1), channels=1)
            extra["ir_intrinsics"] = (
                ir["intrinsics"] or self.intrinsics.get("infrared") or self.intrinsics.get("depth")
            )
            extra["ir_timestamp_ms"] = ir["timestamp_ms"]
        return RgbdFrame(
            color=cf,
            depth=df,
            intrinsics=intr,
            timestamp_ms=depth["timestamp_ms"],
            frame_number=depth["number"],
            aligned=self.align,
            extra=extra,
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
        while self._filters:
            api.delete_filter(*self._filters.pop())
        if self._pipe is not None:
            try:
                api.stop_pipeline(self._pipe, self._profile)
            finally:
                self._pipe = self._profile = None
        self._tuning_pending = False
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
            "depth": {
                "width": self.depth_width,
                "height": self.depth_height,
                "filters": [kind for kind, _ in self.filters.chain()] if self.filters is not None else [],
                "tuning": self.tuning.as_dict() if self.tuning is not None else None,
                "tuning_applied": self.tuning_applied,
            },
            "negotiated": self.negotiated,
            "depth_scale_m": self.depth_scale,
            "infrared": self.infrared,
            "intrinsics": {k: v.as_dict() for k, v in self.intrinsics.items()},
            "extrinsics_depth_to_color": self.extrinsics_depth_to_color,
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


def list_sensor_options(library: str | None = None, serial: str | None = None) -> list[dict]:
    """Per-sensor option dump (:meth:`Api.sensor_options`) for every attached
    device, or just ``serial`` — ``[{serial, name, sensors}]``. Opens no
    streams; on macOS it still needs root to claim the device."""
    api = load_api(library)
    api.log_to_console("error")
    ctx = api.context()
    dev_list = api._call("rs2_query_devices", ctx)
    out = []
    try:
        for i in range(api._call("rs2_get_device_count", dev_list)):
            dev = api._call("rs2_create_device", dev_list, i)
            try:
                info = api.device_info(dev)
                if serial and info.get("serial") != serial:
                    continue
                out.append(
                    {
                        "serial": info.get("serial"),
                        "name": info.get("name"),
                        "sensors": api.sensor_options(dev),
                    }
                )
            finally:
                api.delete_device(dev)
    finally:
        api.lib.rs2_delete_device_list(dev_list)
    return out


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
            "depth": {"width": self.width, "height": self.height, "filters": [], "tuning": None},
            "depth_scale_m": 0.001,
            "intrinsics": {
                "color": synthetic_rgbd(self.width, self.height, holes=False).intrinsics.as_dict()
            },
            "extrinsics_depth_to_color": {
                "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "translation": [0.0, 0.0, 0.0],
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
    width: int | None = None,
    height: int | None = None,
    fps: int | None = None,
    serial: str | None = None,
    align: bool = True,
    library: str | None = None,
    depth_width: int = DEFAULT_DEPTH_WIDTH,
    depth_height: int = DEFAULT_DEPTH_HEIGHT,
    filters: DepthFilters | None = DEFAULT_DEPTH_FILTERS,
    tuning: DepthTuning | None = DEFAULT_DEPTH_TUNING,
) -> RgbdCamera:
    """Factory used by the CLI/viewer: a real D4xx, or the synthetic stand-in.
    ``width``/``height`` = the colour size; ``None`` follows the depth size."""
    width = width or depth_width
    height = height or depth_height
    if fake:
        return SyntheticRgbdCamera(width=width, height=height, fps=min(fps or 15, 15))
    return RealSenseCamera(
        width=width,
        height=height,
        fps=fps,
        serial=serial,
        align=align,
        library=library,
        depth_width=depth_width,
        depth_height=depth_height,
        filters=filters,
        tuning=tuning,
    )


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
