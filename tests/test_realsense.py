"""RealSenseCamera against a fake SDK: the open/read/close protocol, handle
accounting, alignment, error paths. The ctypes layer itself is exercised by
tests/test_realsense_hw.py (needs a camera) and the deproject cross-check in
tests/test_rgbd.py (needs only the library)."""

from __future__ import annotations

import os

import pytest

from perception.frame import Frame
from perception.realsense import (
    FORMAT_RGB8,
    FORMAT_Z16,
    LASER_MAX,
    OPTION_EMITTER_ENABLED,
    OPTION_FILTER_MAGNITUDE,
    OPTION_FILTER_SMOOTH_ALPHA,
    OPTION_HOLES_FILL,
    OPTION_LASER_POWER,
    OPTION_MIN_DISTANCE,
    OPTION_VISUAL_PRESET,
    STREAM_COLOR,
    STREAM_DEPTH,
    VISUAL_PRESETS,
    DepthFilters,
    DepthTuning,
    RealSenseCamera,
    RealSenseError,
    RealSenseLibraryNotFound,
    RealSenseSource,
    RgbdCamera,
    SyntheticRgbdCamera,
    _unstride,
    find_library_path,
    open_camera,
    platform_hint,
)
from perception.rgbd import Intrinsics

COLOR_K = Intrinsics(64, 48, 60.0, 60.0, 32.0, 24.0, model="inverse_brown_conrady")


def balanced(api) -> bool:
    """Every handle returned except the process-wide context (kept alive by design)."""
    return all(v == 0 for k, v in api.live.items() if k != "ctx") and api.live.get("ctx", 0) == 1


DEPTH_K = Intrinsics(64, 48, 58.0, 58.0, 31.0, 23.5, model="brown_conrady")


class FakeApi:
    """Mimics :class:`perception.realsense.Api`'s pythonic surface with plain
    Python objects and counts every handle it hands out / takes back."""

    def __init__(
        self,
        *,
        fail_start: str | None = None,
        frames: list[list[dict]] | None = None,
        usb_type: str = "3.2",
        fail_wait: str | None = None,
        unsupported: frozenset[int] = frozenset(),
        refuse: frozenset[int] = frozenset(),
        no_depth_sensor: bool = False,
    ):
        self.path = "/fake/librealsense2.so"
        self.version = 25804
        self.fail_start = fail_start
        self.fail_wait = fail_wait
        self.usb_type = usb_type
        self.live: dict[str, int] = {}  # handle kind -> outstanding count
        self.log: list[str] = []
        self.frames = frames
        self.n = 0
        self.stride_pad = 0
        self._ctx = None
        self.unsupported = unsupported  # sensor options rs2_supports_option says no to
        self.refuse = refuse  # sensor options whose set raises
        self.no_depth_sensor = no_depth_sensor
        self.options: dict[str, dict[int, float]] = {}  # handle -> {option: value}

    def _take(self, kind):
        self.live[kind] = self.live.get(kind, 0) + 1
        return f"{kind}#{self.live[kind]}"

    def _give(self, kind):
        self.live[kind] -= 1
        assert self.live[kind] >= 0, f"double free of {kind}"

    # -- surface --------------------------------------------------------------
    def log_to_console(self, severity):
        self.log.append(f"log:{severity}")

    def create_context(self):
        return self._take("ctx")

    def delete_context(self, ctx):
        self._give("ctx")

    def context(self):
        if self._ctx is None:
            self._ctx = self.create_context()
        return self._ctx

    def list_devices(self, ctx):
        assert ctx == self._ctx, "enumeration must use the shared context"
        return [self.device_info(None)]

    def start_pipeline(self, ctx, *, width, height, fps, serial, depth_width=None, depth_height=None):
        self.log.append(f"start:{width}x{height}@{fps}:{serial}")
        self.log.append(f"depth:{depth_width or width}x{depth_height or height}")
        if self.fail_start:
            raise RealSenseError(self.fail_start)
        return self._take("pipe"), self._take("profile")

    def stop_pipeline(self, pipe, profile):
        self._give("profile")
        self._give("pipe")

    def profile_device(self, profile):
        return self._take("dev")

    def delete_device(self, dev):
        self._give("dev")

    def device_info(self, dev):
        return {"name": "Fake D435", "serial": "123456", "firmware": "5.16", "usb_type": self.usb_type}

    def depth_scale(self, dev):
        return None if self.no_depth_sensor else 0.001

    # -- options (sensor + processing blocks share rs2_options) ----------------
    def depth_sensor(self, dev):
        return None if self.no_depth_sensor else self._take("sensor")

    def delete_sensor(self, sensor):
        self._give("sensor")

    def supports_option(self, handle, option):
        return option not in self.unsupported

    def option_range(self, handle, option):
        return (0.0, 360.0, 30.0, 150.0) if option == OPTION_LASER_POWER else (0.0, 1.0, 1.0, 0.0)

    def set_option(self, handle, option, value):
        if option in self.refuse:
            raise RealSenseError(f"rs2_set_option({option}): refused")
        self.options.setdefault(handle, {})[option] = float(value)
        self.log.append(f"set:{handle}:{option}={float(value):g}")

    def get_option(self, handle, option):
        return self.options.get(handle, {}).get(option, 0.0)

    def create_filter(self, kind, options=None):
        block, queue = self._take(f"filter:{kind}"), self._take("queue")
        try:
            for option, value in (options or {}).items():
                self.set_option(block, option, value)
        except RealSenseError:
            self.delete_filter(block, queue)  # the real Api releases its own partial block
            raise
        return block, queue

    def delete_filter(self, block, queue):
        self._give("queue")
        self._give(block.rsplit("#", 1)[0])

    def process(self, block, queue, frameset, timeout_ms):
        self._give("frameset")
        self.log.append(f"process:{block.rsplit('#', 1)[0].split(':', 1)[1]}")
        return self._take("frameset")

    def profile_streams(self, profile):
        return [
            {
                "stream": STREAM_DEPTH,
                "format": FORMAT_Z16,
                "index": 0,
                "uid": 0,
                "fps": 30,
                "intrinsics": DEPTH_K,
            },
            {
                "stream": STREAM_COLOR,
                "format": FORMAT_RGB8,
                "index": 0,
                "uid": 1,
                "fps": 30,
                "intrinsics": COLOR_K,
            },
        ]

    def create_align_to_color(self):
        return self._take("align"), self._take("queue")

    def delete_align(self, block, queue):
        self._give("queue")
        self._give("align")

    def wait_for_frames(self, pipe, timeout_ms):
        self.log.append("wait")
        if self.fail_wait:
            raise RealSenseError(self.fail_wait)
        return self._take("frameset")

    def align(self, block, queue, frameset, timeout_ms):
        self._give("frameset")  # rs2_process_frame consumes the input reference
        self.log.append("align")
        return self._take("frameset")

    def release_frame(self, frame):
        self._give("frameset")

    def split_frameset(self, frameset):
        self.n += 1
        if self.frames is not None:
            return self.frames.pop(0)
        aligned = "align" in self.log[-1:]
        w, h = 64, 48
        pad = self.stride_pad
        color_rows = b"".join(bytes([x % 256, 0, 255]) * 1 for x in range(w))
        color = b"".join(color_rows + b"\x00" * pad for _ in range(h))
        depth = b"".join((b"\xe8\x03" * w) + b"\x00" * pad for _ in range(h))  # 1000 units = 1 m
        return [
            {
                "stream": STREAM_COLOR,
                "format": FORMAT_RGB8,
                "width": w,
                "height": h,
                "stride": w * 3 + pad,
                "data": color,
                "timestamp_ms": 10.0 * self.n,
                "number": self.n,
                "intrinsics": COLOR_K,
            },
            {
                "stream": STREAM_DEPTH,
                "format": FORMAT_Z16,
                "width": w,
                "height": h,
                "stride": w * 2 + pad,
                "data": depth,
                "timestamp_ms": 10.0 * self.n,
                "number": self.n,
                "intrinsics": COLOR_K if aligned else DEPTH_K,
            },
        ]


def test_open_read_close_is_handle_balanced():
    api = FakeApi()
    cam = RealSenseCamera(width=64, height=48, fps=30, api=api)
    assert isinstance(cam, RgbdCamera)
    with cam:
        assert cam.info["serial"] == "123456"
        assert cam.depth_scale == 0.001
        assert cam.intrinsics["color"] == COLOR_K and cam.intrinsics["depth"] == DEPTH_K
        f = cam.read()
        assert f.aligned and f.intrinsics == COLOR_K  # aligned depth carries the colour intrinsics
        assert f.color.width == 64 and f.depth.distance_m(3, 3) == pytest.approx(1.0)
        assert f.color.pixel(5, 0) == (5, 0, 255)
        assert f.frame_number == 1 and f.timestamp_ms == 10.0
        assert f.extra["serial"] == "123456"
        d = cam.describe()
        assert d["kind"] == "realsense" and d["open"] and d["sdk"]["api_version"] == 25804
    assert balanced(api), api.live
    assert "start:64x48@30:None" in api.log and "align" in api.log
    assert "depth:848x480" in api.log  # depth streams at its native mode, colour at 64x48
    assert cam.effective_fps == 30
    assert d["depth"]["width"] == 848
    assert d["depth"]["filters"] == ["to_disparity", "spatial", "temporal", "to_depth"]


def test_unaligned_read_uses_depth_intrinsics_and_skips_align():
    api = FakeApi()
    with RealSenseCamera(width=64, height=48, align=False, api=api) as cam:
        f = cam.read()
    assert not f.aligned and f.intrinsics == DEPTH_K
    assert "align" not in api.log
    assert balanced(api)


def test_serial_is_passed_through():
    api = FakeApi()
    with RealSenseCamera(width=64, height=48, serial="ABC", api=api):
        pass
    assert any(s.endswith(":ABC") for s in api.log)


def test_start_failure_cleans_up_and_propagates():
    api = FakeApi(fail_start="No device connected")
    cam = RealSenseCamera(api=api)
    with pytest.raises(RealSenseError, match="No device"):
        cam.open()
    assert balanced(api)
    assert not cam.describe()["open"]


def test_read_before_open_and_missing_streams():
    cam = RealSenseCamera(api=FakeApi())
    with pytest.raises(RealSenseError, match="not open"):
        cam.read()
    api = FakeApi(frames=[[]])
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        with pytest.raises(RealSenseError, match="lacks color\\+depth"):
            cam.read()
    assert balanced(api)  # the frameset was still released


def test_padded_strides_are_removed():
    api = FakeApi()
    api.stride_pad = 7
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        f = cam.read()
    assert len(f.color.data) == 64 * 48 * 3 and f.color.pixel(63, 47) == (63, 0, 255)
    assert f.depth.distance_m(63, 47) == pytest.approx(1.0)
    assert _unstride({"width": 2, "height": 2, "stride": 3, "data": b"abXcdX"}, 1) == b"abcd"
    assert _unstride({"width": 2, "height": 1, "stride": 0, "data": b"ab"}, 1) == b"ab"


def test_realsense_source_yields_color_frames():
    src = RealSenseSource(RealSenseCamera(width=64, height=48, api=FakeApi()))
    with src:
        frame = next(src.frames())
    assert isinstance(frame, Frame) and frame.width == 64
    assert isinstance(src.read_one(), Frame)  # opens/closes itself


def test_synthetic_camera_and_factory():
    cam = open_camera(fake=True, width=32, height=24, fps=0)
    assert isinstance(cam, SyntheticRgbdCamera)
    with pytest.raises(RuntimeError):
        cam.read()
    cam.open()
    a, b = cam.read(), cam.read()
    assert (a.frame_number, b.frame_number) == (1, 2) and b.timestamp_ms >= a.timestamp_ms
    assert cam.describe()["kind"] == "synthetic"
    cam.close()
    real = open_camera(fake=False, width=640, height=480, serial="X", align=False, library="/nope")
    assert isinstance(real, RealSenseCamera) and real.serial == "X" and not real.align


def test_library_lookup_failure_is_explained(monkeypatch, tmp_path):
    monkeypatch.setenv("REALSENSE_LIB", str(tmp_path / "missing.so"))
    monkeypatch.setattr("ctypes.util.find_library", lambda name: None)
    monkeypatch.setattr("perception.realsense._LIBRARY_CANDIDATES", (str(tmp_path / "nope.so"),))
    with pytest.raises(RealSenseLibraryNotFound) as ei:
        find_library_path()
    msg = str(ei.value)
    assert "brew install librealsense" in msg and "REALSENSE_LIB" in msg and "missing.so" in msg
    # an explicit existing path wins over everything
    lib = tmp_path / "lib.so"
    lib.write_bytes(b"")
    assert find_library_path(str(lib)) == str(lib)
    assert os.path.exists(find_library_path(str(lib)))


def test_platform_hint():
    assert "sudo" in platform_hint(RealSenseError("failed to set power state")) or "udev" in platform_hint(
        RealSenseError("failed to set power state")
    )
    assert "USB3" in platform_hint(RealSenseError("No device connected"))
    assert platform_hint(RealSenseError("something else")) == ""


def test_platform_hint_macos_already_root(monkeypatch):
    """Under sudo the claim failure is a held interface, not a permission problem —
    the hint must not send the operator back to `sudo`."""
    import perception.realsense as rs

    monkeypatch.setattr(rs.sys, "platform", "darwin")
    monkeypatch.setattr(rs, "_is_root", lambda: True)
    hint = platform_hint(RealSenseError("failed to set power state"))
    assert "sudo" not in hint
    assert "re-plug" in hint and "just exited" in hint

    monkeypatch.setattr(rs, "_is_root", lambda: False)
    assert "sudo" in platform_hint(RealSenseError("RS2_USB_STATUS_ACCESS"))


def test_context_is_shared_across_opens_and_enumeration():
    api = FakeApi()
    cam = RealSenseCamera(width=64, height=48, api=api)
    with cam:
        pass
    with cam:  # reopen: no second context
        cam.read()
    assert api.live["ctx"] == 1 and balanced(api)


def test_usb2_link_caps_fps_unless_forced(capsys):
    api = FakeApi(usb_type="2.1")
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        assert cam.effective_fps == 15 and cam.describe()["stream"]["fps"] == 15
    assert any("@15:" in s for s in api.log)
    assert "USB 2.1" in capsys.readouterr().err
    api = FakeApi(usb_type="2.1")
    with RealSenseCamera(width=64, height=48, fps=30, api=api) as cam:
        assert cam.effective_fps == 30


def test_first_frame_timeout_gets_a_hint():
    api = FakeApi(fail_wait="Frame didn't arrive within 5000")
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        with pytest.raises(RealSenseError) as ei:
            cam.read()
    msg = str(ei.value)
    assert "first frameset never came" in msg and "re-plug" in msg and "64x48@30" in msg
    assert "re-plug" in platform_hint(ei.value)


# ----- depth quality: filter chain, sensor tuning, depth resolution ---------------


def _chain_log(api) -> list[str]:
    return [e for e in api.log if e.startswith("process:") or e == "align"]


def test_default_filter_chain_runs_before_align_in_intel_order():
    api = FakeApi()
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        cam.read()
        cam.read()
    # disparity domain for spatial+temporal, back to depth, *then* align — per frame, in order
    per_frame = ["process:to_disparity", "process:spatial", "process:temporal", "process:to_depth", "align"]
    assert _chain_log(api) == per_frame * 2
    assert balanced(api), api.live  # every block, queue and intermediate frameset released
    # the SDK defaults land on the blocks
    spatial = next(h for h in api.options if h.startswith("filter:spatial"))
    temporal = next(h for h in api.options if h.startswith("filter:temporal"))
    assert api.options[spatial][OPTION_FILTER_MAGNITUDE] == 2.0
    assert api.options[spatial][OPTION_FILTER_SMOOTH_ALPHA] == 0.5
    assert api.options[temporal][OPTION_FILTER_SMOOTH_ALPHA] == 0.4
    assert api.options[temporal][OPTION_HOLES_FILL] == 3.0  # persistency index, not hole filling


def test_filters_none_is_raw_depth():
    api = FakeApi()
    with RealSenseCamera(width=64, height=48, filters=None, api=api) as cam:
        cam.read()
        assert cam.describe()["depth"]["filters"] == []
    assert _chain_log(api) == ["align"]
    assert not any(k.startswith("filter:") for k in api.live)


def test_filter_chain_composition():
    assert [k for k, _ in DepthFilters().chain()] == ["to_disparity", "spatial", "temporal", "to_depth"]
    assert [k for k, _ in DepthFilters(disparity=False).chain()] == ["spatial", "temporal"]
    assert [k for k, _ in DepthFilters(spatial=False, temporal=False).chain()] == []
    full = DepthFilters(hole_filling=2, min_m=0.2, max_m=1.5, temporal_persistence=8)
    kinds = [k for k, _ in full.chain()]
    assert kinds == ["threshold", "to_disparity", "spatial", "temporal", "to_depth", "hole_filling"]
    opts = dict(full.chain())
    assert opts["threshold"][OPTION_MIN_DISTANCE] == 0.2 and opts["hole_filling"][OPTION_HOLES_FILL] == 2.0
    assert opts["temporal"][OPTION_HOLES_FILL] == 8.0
    assert DepthFilters(temporal=False).as_dict()["chain"] == ["to_disparity", "spatial", "to_depth"]
    # a filter that fails to configure tears down cleanly and aborts the open
    api = FakeApi(refuse=frozenset({OPTION_FILTER_MAGNITUDE}))
    cam = RealSenseCamera(width=64, height=48, api=api)
    with pytest.raises(RealSenseError, match="refused"):
        cam.open()
    assert balanced(api), api.live


def test_tuning_waits_for_the_first_frameset():
    """Regression (hardware, 2026-09-04): sensor writes issued between pipeline
    start and the first frameset stalled a freshly claimed D435 on macOS — no
    frame ever arrived. The writes must follow the first successful wait, once."""
    api = FakeApi()
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        assert cam.tuning_applied == {}  # nothing touched at open
        assert not any(e.startswith("set:sensor") for e in api.log)
        cam.read()
        cam.read()
    events = [e for e in api.log if e == "wait" or e.startswith("set:sensor")]
    assert events[0] == "wait" and events[-1] == "wait"
    assert events.count("wait") == 2 and len(events) == 5  # 3 writes, all after the first wait
    assert balanced(api), api.live  # the device handle used for the writes was released
    # a first read that never gets a frameset leaves the sensor untouched and the tuning pending
    api = FakeApi(fail_wait="Frame didn't arrive within 5000")
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        with pytest.raises(RealSenseError):
            cam.read()
        assert cam.tuning_applied == {} and not any(e.startswith("set:sensor") for e in api.log)


def test_default_tuning_sets_preset_then_emitter_then_max_laser():
    api = FakeApi()
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        cam.read()
        applied = cam.tuning_applied
        assert cam.describe()["depth"]["tuning_applied"] is applied
    sensor_sets = [e for e in api.log if e.startswith("set:sensor")]
    assert sensor_sets == [
        f"set:sensor#1:{OPTION_VISUAL_PRESET}={VISUAL_PRESETS['high_accuracy']}",
        f"set:sensor#1:{OPTION_EMITTER_ENABLED}=1",
        f"set:sensor#1:{OPTION_LASER_POWER}=360",  # LASER_MAX resolved against the sensor's range
    ]
    assert applied == {
        "preset": {"ok": True, "value": "high_accuracy"},
        "emitter": {"ok": True, "value": True},
        "laser_power": {"ok": True, "value": 360.0},
    }
    assert balanced(api), api.live  # the sensor handle was released


def test_tuning_is_best_effort_and_reported():
    api = FakeApi(unsupported=frozenset({OPTION_LASER_POWER}), refuse=frozenset({OPTION_VISUAL_PRESET}))
    with RealSenseCamera(width=64, height=48, api=api) as cam:
        cam.read()  # still streams
        applied = cam.tuning_applied
    assert applied["preset"]["ok"] is False and "refused" in applied["preset"]["error"]
    assert applied["laser_power"] == {"ok": False, "error": "unsupported by this sensor"}
    assert applied["emitter"]["ok"] is True
    assert balanced(api), api.live


def test_tuning_none_and_partial_leave_the_sensor_alone():
    api = FakeApi()
    with RealSenseCamera(width=64, height=48, tuning=None, api=api) as cam:
        cam.read()
        assert cam.tuning_applied == {} and cam.describe()["depth"]["tuning"] is None
    assert not any(e.startswith("set:sensor") for e in api.log)
    api = FakeApi()
    explicit = DepthTuning(preset=None, laser_power=90.0, emitter=None)
    with RealSenseCamera(width=64, height=48, tuning=explicit, api=api) as cam:
        cam.read()
        assert cam.tuning_applied == {"laser_power": {"ok": True, "value": 90.0}}
    assert [e for e in api.log if e.startswith("set:sensor")] == [f"set:sensor#1:{OPTION_LASER_POWER}=90"]
    assert DepthTuning(laser_power=LASER_MAX).laser_power == LASER_MAX
    with pytest.raises(ValueError, match="unknown visual preset"):
        DepthTuning(preset="turbo")
    with pytest.raises(ValueError, match="laser_power"):
        DepthTuning(laser_power=-5.0)


def test_no_depth_sensor_is_an_error_before_tuning():
    api = FakeApi(no_depth_sensor=True)
    with pytest.raises(RealSenseError, match="no depth sensor"):
        RealSenseCamera(width=64, height=48, api=api).open()
    assert balanced(api), api.live


def test_depth_resolution_is_independent_of_colour():
    api = FakeApi()
    with RealSenseCamera(width=64, height=48, depth_width=32, depth_height=24, api=api) as cam:
        assert cam.describe()["depth"]["width"] == 32
    assert "depth:32x24" in api.log and "start:64x48@30:None" in api.log
    real = open_camera(fake=False, width=640, height=480, depth_width=1280, depth_height=720, filters=None)
    assert isinstance(real, RealSenseCamera) and (real.depth_width, real.depth_height) == (1280, 720)
    assert real.filters is None and real.tuning is not None
    follows = open_camera(fake=False, depth_width=1280, depth_height=720)
    assert (follows.width, follows.height) == (1280, 720)  # colour follows depth unless given
    assert (RealSenseCamera().width, RealSenseCamera().depth_width) == (848, 848)
