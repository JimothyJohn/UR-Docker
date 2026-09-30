"""Extra viewpoints — plain webcams beside the RGB-D camera.

A *view* is a picture-only feed the cockpit shows under the colour/depth pair
(the 2×2 grid) and writes into every snapshot, so an agent reading the
snapshot PNGs sees the robot from more than the wrist camera. Views measure
nothing: no depth, no intrinsics, no hand-eye — they are eyes for the pilot.

The shipped source is :class:`FfmpegView`: ``ffmpeg`` (an external binary, the
same rule as ``ssh``/``docker`` in ``urctl.sysinfo`` — reached for, never a
Python dependency; the package core stays dep-free) opens the device with the
OS's own capture API and pipes an MJPEG stream back:

- macOS: ``-f avfoundation`` — the device is picked **by name** (``"Logi
  Webcam C920e"``, a case-insensitive substring) from ``ffmpeg -list_devices``,
  because AVFoundation indices shift whenever a camera is plugged in (the
  D435's RGB imager is on that list too). Verified 2026-09-25 on the Mac
  Studio with two C920s. Camera access goes through TCC, so the process that
  launches the cockpit must be allowed to use the camera (a Terminal is
  prompted once; an SSH session is silently denied — ``platform_hint``).
- Linux: ``-f v4l2`` — the device is a path (``/dev/video2``). Unverified.
- Windows: ``-f dshow`` — ``video=<name>``. Unverified.
- ``lavfi:<graph>`` anywhere: ffmpeg's synthetic sources (``lavfi:testsrc``),
  no camera needed — what the tests use when ffmpeg is installed.

:class:`SyntheticView` needs neither ffmpeg nor a camera (``--fake``; CI).
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from .pngio import encode_png

DEFAULT_VIEW_WIDTH, DEFAULT_VIEW_HEIGHT, DEFAULT_VIEW_FPS = 640, 480, 15
JPEG_SOI, JPEG_EOI = b"\xff\xd8", b"\xff\xd9"
_READ_CHUNK = 65536


class ViewError(RuntimeError):
    """A view could not be opened or read; the message names the fix."""


@runtime_checkable
class ViewSource(Protocol):
    """open() → read() (one encoded image) → close(); ``content_type`` says which encoding."""

    name: str
    content_type: str

    def open(self) -> None: ...
    def read(self) -> bytes: ...
    def close(self) -> None: ...
    def describe(self) -> dict: ...


# -- device specs ------------------------------------------------------------------------


def parse_view_specs(text: str | None) -> list[str]:
    """``PERCEPTRONICS_VIEWS`` → device names: comma-separated, blanks dropped."""
    if not text:
        return []
    return [part.strip() for part in text.split(",") if part.strip()]


def parse_view_size(text: str | None) -> tuple[int, int]:
    """``"640x480"`` → ``(640, 480)`` (the default when empty)."""
    if not text:
        return DEFAULT_VIEW_WIDTH, DEFAULT_VIEW_HEIGHT
    m = re.fullmatch(r"\s*(\d+)\s*[xX×]\s*(\d+)\s*", text)
    if not m:
        raise ValueError(f"view size must look like 640x480, got {text!r}")
    return int(m.group(1)), int(m.group(2))


def parse_avfoundation_devices(listing: str) -> list[tuple[int, str]]:
    """The video devices in ``ffmpeg -f avfoundation -list_devices true -i ""`` output."""
    out: list[tuple[int, str]] = []
    in_video = False
    for line in listing.splitlines():
        if "AVFoundation video devices" in line:
            in_video = True
            continue
        if "AVFoundation audio devices" in line:
            break
        if not in_video:
            continue
        m = re.search(r"\[(\d+)\]\s+(.+?)\s*$", line)
        if m:
            out.append((int(m.group(1)), m.group(2)))
    return out


def list_avfoundation_devices(binary: str = "ffmpeg") -> list[tuple[int, str]]:
    """Ask ffmpeg for the AVFoundation video devices (macOS)."""
    proc = subprocess.run(
        [binary, "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
        capture_output=True,
        text=True,
        stdin=subprocess.DEVNULL,
        timeout=20,
        check=False,
    )
    return parse_avfoundation_devices(proc.stderr + proc.stdout)


def resolve_avfoundation(name: str, devices: list[tuple[int, str]]) -> int:
    """``name`` → AVFoundation index: an integer as is, else the one device whose
    name contains it (case-insensitive); the error lists what is attached."""
    if re.fullmatch(r"\d+", name.strip()):
        return int(name)
    needle = name.strip().lower()
    hits = [(i, n) for i, n in devices if needle in n.lower()]
    if len(hits) == 1:
        return hits[0][0]
    listing = ", ".join(f"[{i}] {n}" for i, n in devices) or "none"
    if not hits:
        raise ViewError(f"no AVFoundation video device matches {name!r} (attached: {listing})")
    raise ViewError(f"{name!r} matches several devices (attached: {listing}) — be more specific")


def ffmpeg_input_args(name: str, *, width: int, height: int, fps: int, devices=None) -> list[str]:
    """The ``-f … -i …`` part of the ffmpeg command for ``name`` on this platform."""
    size = f"{width}x{height}"
    if name.startswith("lavfi:"):
        graph = name[len("lavfi:") :] or "testsrc"
        if "=" not in graph:
            graph = f"{graph}=size={size}:rate={fps}"
        return ["-re", "-f", "lavfi", "-i", graph]  # -re: real-time pacing, like a camera
    if sys.platform == "darwin":
        index = resolve_avfoundation(name, devices if devices is not None else [])
        return ["-f", "avfoundation", "-framerate", str(fps), "-video_size", size, "-i", str(index)]
    if sys.platform.startswith("win"):
        return ["-f", "dshow", "-framerate", str(fps), "-video_size", size, "-i", f"video={name}"]
    return ["-f", "v4l2", "-framerate", str(fps), "-video_size", size, "-i", name]


def platform_hint(exc: BaseException) -> str:
    """Turn ffmpeg's capture failure into the fix for this OS."""
    text = str(exc)
    if sys.platform == "darwin" and (
        "not permitted" in text
        or "Operation not permitted" in text
        or "Input/output error" in text
        or "no frame within" in text
        or "stalled" in text
    ):
        return (
            "macOS camera permission (TCC): ffmpeg opened the device but no frame ever came — the "
            "camera is withheld from the process tree that launched the cockpit. An SSH session "
            "(even with sudo) is denied without a prompt; launch the cockpit from a Terminal window "
            "on the Mac itself (Screen Sharing counts) and allow it once under System Settings → "
            "Privacy & Security → Camera. `/usr/bin/log show --last 5m --predicate "
            "'eventMessage CONTAINS \"kTCCServiceCamera\"'` shows the decision."
        )
    if "Cannot open" in text or "No such file" in text or "not found" in text:
        return (
            'check the device: `ffmpeg -f avfoundation -list_devices true -i ""` (macOS), '
            "`ls /dev/video*` (Linux)."
        )
    return ""


# -- the ffmpeg-backed view --------------------------------------------------------------


@dataclass
class FfmpegView:
    """One webcam as an MJPEG stream out of ffmpeg. ``name`` is a device name
    (macOS), a ``/dev/videoN`` path (Linux), a dshow name (Windows) or
    ``lavfi:<graph>``. Frames come back as JPEG bytes."""

    name: str
    width: int = DEFAULT_VIEW_WIDTH
    height: int = DEFAULT_VIEW_HEIGHT
    fps: int = DEFAULT_VIEW_FPS
    quality: int = 5  # ffmpeg -q:v, 2 (best) … 31
    binary: str = "ffmpeg"
    timeout_s: float = 5.0
    content_type: str = field(default="image/jpeg", init=False)
    device: str | None = field(default=None, init=False)  # what ffmpeg was pointed at
    _proc: Any = field(default=None, init=False, repr=False)
    _buf: bytearray = field(default_factory=bytearray, init=False, repr=False)
    _stderr: list[str] = field(default_factory=list, init=False, repr=False)
    _frames: int = field(default=0, init=False, repr=False)

    def open(self) -> None:
        if self._proc is not None:
            return
        binary = shutil.which(self.binary)
        if binary is None:
            raise ViewError(
                f"`{self.binary}` not found — views need ffmpeg on PATH "
                "(brew install ffmpeg / apt install ffmpeg / winget install ffmpeg)"
            )
        devices = None
        if sys.platform == "darwin" and not self.name.startswith("lavfi:"):
            devices = list_avfoundation_devices(binary)
        inp = ffmpeg_input_args(
            self.name, width=self.width, height=self.height, fps=self.fps, devices=devices
        )
        self.device = inp[-1]
        cmd = [
            binary,
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            *inp,
            "-an",
            "-c:v",
            "mjpeg",
            "-q:v",
            str(self.quality),
            "-f",
            "image2pipe",
            "-",
        ]
        self._buf = bytearray()
        self._stderr = []
        self._frames = 0
        self._proc = subprocess.Popen(
            cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )
        threading.Thread(target=self._drain_stderr, name=f"view-stderr:{self.name}", daemon=True).start()

    def _drain_stderr(self) -> None:
        proc = self._proc
        if proc is None or proc.stderr is None:
            return
        for raw in iter(proc.stderr.readline, b""):
            line = raw.decode(errors="replace").rstrip()
            if line:
                self._stderr.append(line)
                del self._stderr[:-20]

    def _fail(self, what: str) -> ViewError:
        tail = "; ".join(self._stderr[-3:])
        code = self._proc.poll() if self._proc is not None else None
        detail = f" (ffmpeg exit {code})" if code is not None else ""
        msg = f"{what} from {self.name!r}{detail}" + (f": {tail}" if tail else "")
        hint = platform_hint(ViewError(msg))
        return ViewError(msg + (f" — {hint}" if hint else ""))

    def read(self) -> bytes:
        """The next JPEG (blocks up to ``timeout_s`` for it)."""
        proc = self._proc
        if proc is None or proc.stdout is None:
            raise ViewError("view is not open (call open() first)")
        deadline = time.monotonic() + self.timeout_s
        while True:
            frame = split_jpeg(self._buf)
            if frame is not None:
                self._frames += 1
                return frame
            if time.monotonic() > deadline:
                raise self._fail(f"no frame within {self.timeout_s:g} s")
            chunk = _read_some(proc.stdout, deadline)
            if not chunk:
                if proc.poll() is not None or time.monotonic() > deadline:
                    raise self._fail("stream ended" if proc.poll() is not None else "stalled")
                continue
            self._buf.extend(chunk)
            if len(self._buf) > 64 * 1024 * 1024:  # never a JPEG; ffmpeg is not sending MJPEG
                raise self._fail("no JPEG boundary in 64 MB")

    def close(self) -> None:
        proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
        finally:
            for stream in (proc.stdout, proc.stderr):
                try:
                    if stream is not None:
                        stream.close()
                except OSError:
                    pass

    def describe(self) -> dict:
        return {
            "kind": "ffmpeg",
            "name": self.name,
            "device": self.device,
            "open": self._proc is not None and self._proc.poll() is None,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "content_type": self.content_type,
            "frames": self._frames,
        }

    def __enter__(self) -> FfmpegView:
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def _read_some(stream, deadline: float) -> bytes:
    """Up to one chunk from ``stream``; ``b""`` on EOF or when nothing arrived
    before ``deadline`` (POSIX: a ``select`` wait; elsewhere a blocking read)."""
    if os.name == "posix":
        import select

        wait = max(0.0, deadline - time.monotonic())
        ready, _, _ = select.select([stream], [], [], min(wait, 0.5))
        if not ready:
            return b""
    return stream.read1(_READ_CHUNK) if hasattr(stream, "read1") else stream.read(_READ_CHUNK)


def split_jpeg(buf: bytearray) -> bytes | None:
    """Pop the first complete JPEG (SOI … EOI) off ``buf``; None if incomplete.
    Bytes before the first SOI are garbage and dropped. Inside entropy-coded
    data every ``0xFF`` is stuffed (``FF 00``) or a restart marker, so the
    first ``FF D9`` after the SOI is the end of image — true for the MJPEG
    ffmpeg emits (no embedded thumbnails)."""
    start = buf.find(JPEG_SOI)
    if start < 0:
        if len(buf) > 1:
            del buf[:-1]
        return None
    if start:
        del buf[:start]
    end = buf.find(JPEG_EOI, 2)
    if end < 0:
        return None
    frame = bytes(buf[: end + 2])
    del buf[: end + 2]
    return frame


# -- the camera-less stand-in --------------------------------------------------------


@dataclass
class SyntheticView:
    """A moving test pattern encoded as PNG (no ffmpeg, no camera): ``--fake``
    views and the tests. ``fps`` paces ``read()``; 0 = as fast as asked."""

    name: str = "synthetic"
    width: int = 320
    height: int = 240
    fps: int = 10
    content_type: str = field(default="image/png", init=False)
    _open: bool = field(default=False, init=False, repr=False)
    _n: int = field(default=0, init=False, repr=False)
    _last: float = field(default=0.0, init=False, repr=False)

    def open(self) -> None:
        self._open = True

    def read(self) -> bytes:
        if not self._open:
            raise ViewError("view is not open (call open() first)")
        if self.fps > 0:
            wait = self._last + 1.0 / self.fps - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        self._last = time.monotonic()
        self._n += 1
        w, h, n = self.width, self.height, self._n
        cx, cy = w / 2 + 0.3 * w * math.cos(n / 7.0), h / 2 + 0.3 * h * math.sin(n / 7.0)
        rows = bytearray()
        for y in range(h):
            for x in range(w):
                d = math.hypot(x - cx, y - cy)
                if d < 18:
                    rows += b"\xf0\xc0\x30"
                else:
                    g = 40 + (x * 60 // max(w, 1)) + (y * 60 // max(h, 1))
                    rows += bytes((g, g, g + 20))
        return encode_png(w, h, 3, bytes(rows))

    def close(self) -> None:
        self._open = False

    def describe(self) -> dict:
        return {
            "kind": "synthetic",
            "name": self.name,
            "device": None,
            "open": self._open,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "content_type": self.content_type,
            "frames": self._n,
        }

    def __enter__(self) -> SyntheticView:
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def open_views(
    specs: list[str],
    *,
    fake: bool = False,
    width: int = DEFAULT_VIEW_WIDTH,
    height: int = DEFAULT_VIEW_HEIGHT,
    fps: int = DEFAULT_VIEW_FPS,
    binary: str = "ffmpeg",
) -> list[ViewSource]:
    """One source per spec: :class:`FfmpegView`, or :class:`SyntheticView`
    under ``fake`` (same names, so the page lays out the same grid)."""
    if fake:
        return [SyntheticView(name=s, width=160, height=120, fps=min(fps, 8)) for s in specs]
    return [FfmpegView(name=s, width=width, height=height, fps=fps, binary=binary) for s in specs]
