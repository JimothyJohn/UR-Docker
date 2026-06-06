"""Frame — the in-memory RGB image the pipeline operates on.

The whole package is dependency-free at its core, so a :class:`Frame` is just a
flat byte buffer plus geometry — no numpy required to *hold* an image. numpy /
OpenCV only enter through the optional backends, which convert via
:meth:`Frame.to_numpy` / :meth:`Frame.from_numpy`.

Pixels are stored row-major, RGB, one ``int`` (0..255) per channel::

    idx = (y * width + x) * channels

The :func:`synthetic_frame` factory paints colored disks on a flat background.
It is not a shipped "frame source" (the device webcam is — see
:mod:`perception.sources`); it exists so tests and the ``--synthetic`` demo path
can exercise the full depth + blob pipeline on a box with no camera.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Frame:
    """An RGB image as a flat buffer. ``channels`` is 3 (RGB) unless noted."""

    width: int
    height: int
    data: bytes
    channels: int = 3

    def __post_init__(self) -> None:
        expected = self.width * self.height * self.channels
        if len(self.data) != expected:
            raise ValueError(
                f"frame buffer is {len(self.data)} bytes, expected "
                f"{expected} ({self.width}x{self.height}x{self.channels})"
            )

    def pixel(self, x: int, y: int) -> tuple[int, ...]:
        """Return the channel tuple at ``(x, y)``."""
        base = (y * self.width + x) * self.channels
        return tuple(self.data[base : base + self.channels])

    def luma(self) -> list[float]:
        """Grayscale luminance per pixel (Rec. 601), length ``width*height``."""
        d, c = self.data, self.channels
        out = [0.0] * (self.width * self.height)
        for i in range(len(out)):
            b = i * c
            out[i] = 0.299 * d[b] + 0.587 * d[b + 1] + 0.114 * d[b + 2]
        return out

    # ----- numpy / OpenCV interop (optional) --------------------------------

    def to_numpy(self):
        """Return an ``(H, W, channels)`` uint8 numpy array. Requires numpy."""
        np = _require_numpy()
        arr = np.frombuffer(self.data, dtype=np.uint8)
        return arr.reshape(self.height, self.width, self.channels)

    @classmethod
    def from_numpy(cls, arr) -> Frame:
        """Build a Frame from an ``(H, W, C)`` uint8 array (e.g. a cv2 capture).

        Assumes RGB channel order; callers reading BGR from OpenCV should flip
        first (``arr[..., ::-1]``).
        """
        np = _require_numpy()
        arr = np.ascontiguousarray(arr, dtype=np.uint8)
        if arr.ndim == 2:
            arr = arr[:, :, None]
        h, w, c = arr.shape
        return cls(width=w, height=h, data=arr.tobytes(), channels=c)

    # ----- pure-Python loading + scaling (no third-party deps) --------------

    @classmethod
    def from_png(cls, path: str, *, drop_alpha: bool = True) -> Frame:
        """Load a PNG into a Frame via the dependency-free decoder.

        Returns a 3-channel RGB Frame by default (alpha dropped); pass
        ``drop_alpha=False`` to keep an RGBA frame. Decoding is full-resolution;
        downsample with :meth:`resized` before running the pure-Python pipeline.
        """
        from .pngio import load_png

        w, h, c, buf = load_png(path)
        frame = cls(width=w, height=h, data=buf, channels=c)
        return frame.to_rgb() if (drop_alpha and c == 4) else frame

    def to_rgb(self) -> Frame:
        """Drop the alpha channel (RGBA -> RGB); a no-op for an RGB frame."""
        if self.channels == 3:
            return self
        if self.channels != 4:
            raise ValueError(f"cannot convert {self.channels}-channel frame to RGB")
        src = self.data
        out = bytearray(self.width * self.height * 3)
        for i in range(self.width * self.height):
            s = i * 4
            d = i * 3
            out[d] = src[s]
            out[d + 1] = src[s + 1]
            out[d + 2] = src[s + 2]
        return Frame(width=self.width, height=self.height, data=bytes(out), channels=3)

    def resized(self, width: int, height: int) -> Frame:
        """Nearest-neighbour resize to ``width x height`` (keeps channel count).

        Nearest-neighbour is deliberate: cheap, dependency-free, and it does not
        invent colors at object edges — important when the next step segments by
        color/chroma. Use it to shrink a camera/PNG frame to a size the
        pure-Python pipeline handles quickly.
        """
        if width <= 0 or height <= 0:
            raise ValueError("resize target must be positive")
        c, src = self.channels, self.data
        out = bytearray(width * height * c)
        # Precompute source x for each destination x.
        sx = [min(self.width - 1, (x * self.width) // width) for x in range(width)]
        for y in range(height):
            syrow = min(self.height - 1, (y * self.height) // height) * self.width
            drow = y * width
            for x in range(width):
                s = (syrow + sx[x]) * c
                d = (drow + x) * c
                out[d : d + c] = src[s : s + c]
        return Frame(width=width, height=height, data=bytes(out), channels=c)


def _require_numpy():
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - exercised only without numpy
        raise ImportError(
            "numpy is required for Frame numpy/OpenCV interop. "
            "Install it with `pip install -e .[perception]`."
        ) from exc
    return np


def synthetic_frame(
    width: int = 640,
    height: int = 480,
    disks: list[tuple[int, int, int, tuple[int, int, int]]] | None = None,
    background: tuple[int, int, int] = (18, 18, 22),
) -> Frame:
    """Paint ``disks`` of ``(cx, cy, radius, rgb)`` on a flat background.

    Deterministic — no RNG — so tests can assert exact blob counts/locations.
    Defaults to three disks roughly mimicking the apples in the PickAndStack
    sample, which is the integration target on the control side.
    """
    if disks is None:
        disks = [
            (width // 4, height // 2, max(8, width // 12), (220, 40, 40)),
            (width // 2, height // 3, max(8, width // 14), (40, 200, 60)),
            (3 * width // 4, 2 * height // 3, max(8, width // 10), (60, 90, 230)),
        ]
    buf = bytearray()
    br, bg, bb = background
    for _ in range(width * height):
        buf += bytes((br, bg, bb))

    for cx, cy, r, (dr, dg, db) in disks:
        r2 = r * r
        y0, y1 = max(0, cy - r), min(height, cy + r + 1)
        x0, x1 = max(0, cx - r), min(width, cx + r + 1)
        for y in range(y0, y1):
            dy2 = (y - cy) ** 2
            for x in range(x0, x1):
                if (x - cx) ** 2 + dy2 <= r2:
                    base = (y * width + x) * 3
                    buf[base] = dr
                    buf[base + 1] = dg
                    buf[base + 2] = db
    return Frame(width=width, height=height, data=bytes(buf), channels=3)


# Re-exported for callers that want the math helper without importing it from a
# private name (used by the stub depth estimator's normalization).
def normalize(values: list[float]) -> list[float]:
    """Scale ``values`` to 0..1 by their min/max (flat input -> all zeros)."""
    if not values:
        return []
    lo = min(values)
    hi = max(values)
    span = hi - lo
    if span <= 1e-9:
        return [0.0] * len(values)
    return [(v - lo) / span for v in values]
