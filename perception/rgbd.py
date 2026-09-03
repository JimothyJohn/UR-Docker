"""RGB-D types — a color :class:`Frame` plus a metric depth image and the
camera intrinsics that turn a pixel + depth into a 3D point.

This is the data layer under :mod:`perception.realsense`; it holds no camera
code and no third-party imports, so captures can be loaded, deprojected, and
measured on any box (CI, a laptop without the SDK) exactly as they were on the
capture host.

* :class:`Intrinsics` — pinhole model + distortion, with :meth:`deproject`
  (pixel + depth → camera-frame metres). The math mirrors librealsense's
  ``rs2_deproject_pixel_to_point`` (the unit test cross-checks the two whenever
  the SDK is loadable).
* :class:`DepthImage` — raw ``uint16`` depth units + the metres-per-unit scale
  (a D4xx reports 0.001 = millimetres). Zero means "no data" (the sensor could
  not compute depth there), never "at the camera".
* :class:`RgbdFrame` — one aligned color + depth pair with timestamps.
* :func:`synthetic_rgbd` — a deterministic scene for tests and the ``--fake``
  viewer: three disks at known depths on a flat wall.
"""

from __future__ import annotations

import math
import struct
import sys
from array import array
from dataclasses import dataclass, field

from .frame import Frame, synthetic_frame

# librealsense's rs2_distortion names, by ordinal (verified against
# rs2_distortion_to_string on librealsense 2.58.4).
DISTORTION_MODELS = (
    "none",
    "modified_brown_conrady",
    "inverse_brown_conrady",
    "ftheta",
    "brown_conrady",
    "kannala_brandt4",
)


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole camera model for one video stream (pixel <-> camera-frame ray)."""

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    model: str = "none"
    coeffs: tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if self.model not in DISTORTION_MODELS:
            raise ValueError(f"unknown distortion model {self.model!r}; choose from {DISTORTION_MODELS}")
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("focal lengths must be positive")
        if len(self.coeffs) != 5:
            raise ValueError("coeffs must have 5 entries")

    def deproject(self, u: float, v: float, depth_m: float) -> tuple[float, float, float]:
        """Pixel ``(u, v)`` at ``depth_m`` → ``(x, y, z)`` metres in the camera frame.

        Camera frame is librealsense's: +x right, +y down, +z out of the lens.
        Follows ``rs2_deproject_pixel_to_point``: a plain pinhole for zero
        distortion, and an iterative undistort for the Brown-Conrady models.
        Forward-distorted models (``modified_brown_conrady``, ``ftheta``,
        ``kannala_brandt4``) cannot be deprojected this way and raise.
        """
        if self.model in ("modified_brown_conrady", "ftheta", "kannala_brandt4"):
            raise ValueError(f"cannot deproject from a {self.model} image")
        x = (u - self.ppx) / self.fx
        y = (v - self.ppy) / self.fy
        if self.model in ("inverse_brown_conrady", "brown_conrady") and any(self.coeffs):
            k1, k2, p1, p2, k3 = self.coeffs
            inverse = self.model == "inverse_brown_conrady"
            xo, yo = x, y
            for _ in range(10):
                r2 = x * x + y * y
                icdist = 1.0 / (1.0 + ((k3 * r2 + k2) * r2 + k1) * r2)
                # librealsense evaluates the tangential term on the *scaled*
                # point for the inverse model and on the raw point otherwise.
                xq, yq = (x / icdist, y / icdist) if inverse else (x, y)
                delta_x = 2 * p1 * xq * yq + p2 * (r2 + 2 * xq * xq)
                delta_y = 2 * p2 * xq * yq + p1 * (r2 + 2 * yq * yq)
                x = (xo - delta_x) * icdist
                y = (yo - delta_y) * icdist
        return (depth_m * x, depth_m * y, depth_m)

    def project(self, x: float, y: float, z: float) -> tuple[float, float]:
        """Camera-frame point → pixel (pinhole only; distortion ignored)."""
        if z <= 0:
            raise ValueError("cannot project a point at or behind the camera")
        return (self.fx * x / z + self.ppx, self.fy * y / z + self.ppy)

    def fov_deg(self) -> tuple[float, float]:
        """Horizontal and vertical field of view in degrees."""
        h = 2 * math.degrees(math.atan2(self.width * 0.5, self.fx))
        v = 2 * math.degrees(math.atan2(self.height * 0.5, self.fy))
        return (h, v)

    def as_dict(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "fx": self.fx,
            "fy": self.fy,
            "ppx": self.ppx,
            "ppy": self.ppy,
            "model": self.model,
            "coeffs": list(self.coeffs),
            "fov_deg": list(self.fov_deg()),
        }

    @classmethod
    def from_dict(cls, d: dict) -> Intrinsics:
        return cls(
            width=int(d["width"]),
            height=int(d["height"]),
            fx=float(d["fx"]),
            fy=float(d["fy"]),
            ppx=float(d["ppx"]),
            ppy=float(d["ppy"]),
            model=str(d.get("model", "none")),
            coeffs=tuple(float(c) for c in d.get("coeffs", (0, 0, 0, 0, 0))),  # type: ignore[arg-type]
        )


@dataclass(frozen=True)
class DepthImage:
    """Raw sensor depth: one little-endian ``uint16`` per pixel × ``scale_m``.

    ``data`` is exactly ``width*height*2`` bytes in the sensor's native
    little-endian order (what ``RS2_FORMAT_Z16`` delivers), so a frame is a
    zero-copy slice of the SDK buffer. A value of 0 is *invalid* (no depth).
    """

    width: int
    height: int
    data: bytes
    scale_m: float = 0.001

    def __post_init__(self) -> None:
        expected = self.width * self.height * 2
        if len(self.data) != expected:
            raise ValueError(
                f"depth buffer is {len(self.data)} bytes, expected {expected} ({self.width}x{self.height}x2)"
            )
        if self.scale_m <= 0:
            raise ValueError("depth scale must be positive")

    def raw_at(self, x: int, y: int) -> int:
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise IndexError(f"pixel ({x}, {y}) outside {self.width}x{self.height}")
        i = (y * self.width + x) * 2
        return self.data[i] | (self.data[i + 1] << 8)

    def distance_m(self, x: int, y: int) -> float | None:
        """Metres at ``(x, y)``, or ``None`` where the sensor has no data."""
        raw = self.raw_at(x, y)
        return None if raw == 0 else raw * self.scale_m

    def values(self) -> array:
        """The depth units as a host-order ``array('H')`` (a C-speed copy)."""
        a = array("H")
        a.frombytes(self.data)
        if sys.byteorder != "little":  # pragma: no cover - big-endian hosts
            a.byteswap()
        return a

    def stats(self) -> dict:
        """Valid-pixel fraction and min/max/median metres (over valid pixels)."""
        vals = [v for v in self.values() if v]
        n = self.width * self.height
        if not vals:
            return {"valid_fraction": 0.0, "min_m": None, "max_m": None, "median_m": None}
        vals.sort()
        mid = len(vals) // 2
        median = vals[mid] if len(vals) % 2 else 0.5 * (vals[mid - 1] + vals[mid])
        return {
            "valid_fraction": len(vals) / n,
            "min_m": vals[0] * self.scale_m,
            "max_m": vals[-1] * self.scale_m,
            "median_m": median * self.scale_m,
        }

    def to_png16(self) -> bytes:
        """Lossless 16-bit grayscale PNG of the raw units (big-endian per PNG)."""
        from .pngio import encode_png

        a = self.values()
        if sys.byteorder == "little":
            a.byteswap()
        return encode_png(self.width, self.height, 1, a.tobytes(), bit_depth=16)

    @classmethod
    def from_png16(cls, path_or_bytes, scale_m: float = 0.001) -> DepthImage:
        from .pngio import load_png16

        w, h, big = load_png16(path_or_bytes)
        a = array("H")
        a.frombytes(big)
        if sys.byteorder == "little":
            a.byteswap()
        return cls(width=w, height=h, data=a.tobytes(), scale_m=scale_m)

    @classmethod
    def from_metres(cls, width: int, height: int, metres: list[float], scale_m: float = 0.001) -> DepthImage:
        """Build from per-pixel metres (``<= 0`` or ``None`` → invalid/0)."""
        if len(metres) != width * height:
            raise ValueError("metres must have width*height entries")
        a = array("H", (0 if (m is None or m <= 0) else min(65535, int(round(m / scale_m))) for m in metres))
        if sys.byteorder != "little":  # pragma: no cover
            a.byteswap()
        return cls(width=width, height=height, data=a.tobytes(), scale_m=scale_m)


@dataclass(frozen=True)
class RgbdFrame:
    """One color + depth pair. ``intrinsics`` describe the **depth** pixels.

    When ``aligned`` is true the depth image has been re-projected into the
    color camera (same size, same intrinsics as the color stream), so pixel
    ``(u, v)`` names the same physical point in both images — the property the
    viewer's click-to-measure and the segmenter's depth features rely on.
    """

    color: Frame
    depth: DepthImage
    intrinsics: Intrinsics
    timestamp_ms: float = 0.0
    frame_number: int = 0
    aligned: bool = True
    extra: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.aligned and (self.color.width, self.color.height) != (self.depth.width, self.depth.height):
            raise ValueError(
                f"aligned frame but color is {self.color.width}x{self.color.height} "
                f"and depth is {self.depth.width}x{self.depth.height}"
            )

    def point_at(self, u: int, v: int) -> tuple[float, float, float] | None:
        """Camera-frame metres under pixel ``(u, v)``, or ``None`` if no depth."""
        d = self.depth.distance_m(u, v)
        if d is None:
            return None
        return self.intrinsics.deproject(u, v, d)

    def summary(self) -> dict:
        return {
            "width": self.color.width,
            "height": self.color.height,
            "aligned": self.aligned,
            "timestamp_ms": self.timestamp_ms,
            "frame_number": self.frame_number,
            "depth_scale_m": self.depth.scale_m,
            "depth": self.depth.stats(),
            "intrinsics": self.intrinsics.as_dict(),
        }


# ----- synthetic scene ----------------------------------------------------------

# A plausible 640x480 color-stream intrinsic set for a D435 (verified order of
# magnitude: fx ≈ fy ≈ 615 px, principal point near the image centre).
SYNTHETIC_FX = 615.0
SYNTHETIC_WALL_M = 1.20


def synthetic_intrinsics(width: int = 640, height: int = 480) -> Intrinsics:
    scale = width / 640.0
    return Intrinsics(
        width=width,
        height=height,
        fx=SYNTHETIC_FX * scale,
        fy=SYNTHETIC_FX * scale,
        ppx=width / 2.0,
        ppy=height / 2.0,
        model="inverse_brown_conrady",
    )


def synthetic_disks(width: int, height: int) -> list[tuple[int, int, int, tuple[int, int, int], float]]:
    """The three default disks with a depth each: ``(cx, cy, r, rgb, depth_m)``."""
    return [
        (width // 4, height // 2, max(8, width // 12), (220, 40, 40), 0.55),
        (width // 2, height // 3, max(8, width // 14), (40, 200, 60), 0.80),
        (3 * width // 4, 2 * height // 3, max(8, width // 10), (60, 90, 230), 0.65),
    ]


def synthetic_rgbd(
    width: int = 640,
    height: int = 480,
    *,
    frame_number: int = 0,
    timestamp_ms: float = 0.0,
    wall_m: float = SYNTHETIC_WALL_M,
    holes: bool = True,
) -> RgbdFrame:
    """Three colored disks at known depths in front of a flat wall.

    Depth is exact per disk (a flat-fronted puck), the wall sits at ``wall_m``,
    and — mirroring a real sensor — a small strip along the left edge has no
    depth (zeros) so consumers are forced to handle invalid pixels.
    """
    disks = synthetic_disks(width, height)
    color = synthetic_frame(width, height, [(cx, cy, r, rgb) for cx, cy, r, rgb, _ in disks])
    metres = [wall_m] * (width * height)
    for cx, cy, r, _rgb, z in disks:
        r2 = r * r
        for y in range(max(0, cy - r), min(height, cy + r + 1)):
            dy2 = (y - cy) ** 2
            row = y * width
            for x in range(max(0, cx - r), min(width, cx + r + 1)):
                if (x - cx) ** 2 + dy2 <= r2:
                    metres[row + x] = z
    if holes:
        strip = max(1, width // 40)
        for y in range(height):
            for x in range(strip):
                metres[y * width + x] = 0.0
    depth = DepthImage.from_metres(width, height, metres)
    return RgbdFrame(
        color=color,
        depth=depth,
        intrinsics=synthetic_intrinsics(width, height),
        timestamp_ms=timestamp_ms,
        frame_number=frame_number,
        aligned=True,
        extra={"synthetic": True},
    )


def pack_rgbd(frame: RgbdFrame, *, seq: int, meta: dict | None = None) -> bytes:
    """Serialize a frame for the viewer: header JSON + color PNG + zlib'd depth.

    Layout (all big-endian): ``b"RGBD"`` · ``u32`` header length · header JSON
    · ``u32`` PNG length · PNG (RGB8) · ``u32`` depth length · zlib(uint16 LE).
    The browser inflates the depth with ``DecompressionStream`` and colorizes
    it locally, so the server never runs a per-pixel Python loop per frame.
    """
    import json
    import zlib

    from .pngio import encode_png

    c = frame.color if frame.color.channels == 3 else frame.color.to_rgb()
    png = encode_png(c.width, c.height, 3, c.data)
    depth = zlib.compress(frame.depth.data, 1)
    header = {
        "seq": seq,
        "width": c.width,
        "height": c.height,
        "depth_width": frame.depth.width,
        "depth_height": frame.depth.height,
        "depth_scale_m": frame.depth.scale_m,
        "timestamp_ms": frame.timestamp_ms,
        "frame_number": frame.frame_number,
        "aligned": frame.aligned,
        "intrinsics": frame.intrinsics.as_dict(),
    }
    if meta:
        header.update(meta)
    hb = json.dumps(header).encode()
    return b"".join(
        (
            b"RGBD",
            struct.pack(">I", len(hb)),
            hb,
            struct.pack(">I", len(png)),
            png,
            struct.pack(">I", len(depth)),
            depth,
        )
    )


def unpack_rgbd(blob: bytes) -> tuple[dict, bytes, bytes]:
    """Inverse of :func:`pack_rgbd` → ``(header, png_bytes, zlib_depth_bytes)``."""
    import json

    if blob[:4] != b"RGBD":
        raise ValueError("not an RGBD container")
    (hl,) = struct.unpack(">I", blob[4:8])
    header = json.loads(blob[8 : 8 + hl])
    i = 8 + hl
    (pl,) = struct.unpack(">I", blob[i : i + 4])
    png = blob[i + 4 : i + 4 + pl]
    i += 4 + pl
    (dl,) = struct.unpack(">I", blob[i : i + 4])
    depth = blob[i + 4 : i + 4 + dl]
    return header, png, depth
