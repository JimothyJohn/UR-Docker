"""Frame sources — where RGB frames come from.

A :class:`FrameSource` is an iterable-ish producer of :class:`Frame` objects
that you open, read from, and close (a context manager). The shipped source is
:class:`DeviceSource`, a webcam/capture-device reader backed by OpenCV — the
closest thing to a real deployment camera.

OpenCV is an optional dependency (the package core stays dep-free), so
``DeviceSource`` imports ``cv2`` lazily and raises a clear install hint if it is
missing — the same pattern ``urctl`` uses for its MCP extra. Tests and the
camera-less demo path build :class:`~perception.frame.Frame` objects directly
via :func:`perception.frame.synthetic_frame` instead of going through a source.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Protocol, runtime_checkable

from .config import PerceptionConfig
from .frame import Frame


@runtime_checkable
class FrameSource(Protocol):
    """Open -> frames() -> close. Implementations are context managers."""

    def open(self) -> None: ...
    def frames(self) -> Iterator[Frame]: ...
    def close(self) -> None: ...


class DeviceSource:
    """Read RGB frames from a camera/capture device via OpenCV.

    Configured by :class:`PerceptionConfig` (device index, resolution, fps).
    OpenCV delivers BGR; frames are flipped to RGB so the rest of the pipeline
    sees a consistent channel order.

    Use as a context manager::

        with DeviceSource(PerceptionConfig.from_env()) as src:
            for frame in src.frames():
                ...
    """

    def __init__(self, config: PerceptionConfig | None = None):
        self.config = config or PerceptionConfig.from_env()
        self._cap = None

    def open(self) -> None:
        cv2 = _require_cv2()
        idx = self.config.device_index
        cap = cv2.VideoCapture(idx if idx >= 0 else 0)
        if not cap.isOpened():
            raise RuntimeError(f"could not open camera device {idx} (no camera, or it's in use)")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        cap.set(cv2.CAP_PROP_FPS, self.config.fps)
        self._cap = cap

    def frames(self) -> Iterator[Frame]:
        if self._cap is None:
            raise RuntimeError("DeviceSource.open() must be called before frames()")
        while True:
            ok, bgr = self._cap.read()
            if not ok:
                break
            rgb = bgr[:, :, ::-1]  # BGR -> RGB
            yield Frame.from_numpy(rgb)

    def read_one(self) -> Frame:
        """Grab a single frame (opening/closing automatically if needed)."""
        opened_here = self._cap is None
        if opened_here:
            self.open()
        try:
            for frame in self.frames():
                return frame
            raise RuntimeError("camera returned no frames")
        finally:
            if opened_here:
                self.close()

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> DeviceSource:
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def _require_cv2():
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - exercised only without cv2
        raise ImportError(
            "OpenCV (cv2) is required for DeviceSource. "
            "Install it with `pip install -e .[perception]`."
        ) from exc
    return cv2
