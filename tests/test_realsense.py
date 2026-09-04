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
    STREAM_COLOR,
    STREAM_DEPTH,
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

    def start_pipeline(self, ctx, *, width, height, fps, serial):
        self.log.append(f"start:{width}x{height}@{fps}:{serial}")
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
        return 0.001

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
    assert cam.effective_fps == 30


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
