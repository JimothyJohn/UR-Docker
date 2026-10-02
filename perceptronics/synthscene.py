"""Ray-cast depth of boxes standing on a plane — a ground truth for :mod:`perceptronics.volume`.

Every box stands on the table (``z = table_z`` in the base frame, the table level), its
footprint ``length × width`` centred on ``(x, y)`` with the long side at heading
``theta``, ``height`` tall. A pixel's depth is the camera-frame Z of the nearest hit
(box faces — tops *and* sides — or the table), exactly what an aligned D435 depth frame
holds. Stdlib; a 160×120 frame of a few boxes takes a fraction of a second.

    from perceptronics.synthscene import Box, render_depth
    depth = render_depth(W, H, K, T_bc, [Box(0.35, 0.0, 0.06, 0.04, 0.03, 0.3)], table_z=-0.27)
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from urctl.pose import Transform


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    length: float
    width: float
    height: float
    theta: float = 0.0  # heading of the long side in base XY, rad
    round: bool = False  # an upright cylinder instead: ``length`` is its diameter


def camera_looking_down(x: float, y: float, z: float, yaw: float = 0.0) -> Transform:
    """A camera at ``(x, y, z)`` looking straight down; image right = base heading ``yaw``,
    image down = 90° clockwise from it seen from above (the camera's Y)."""
    c, s = math.cos(yaw), math.sin(yaw)
    x_axis = (c, s, 0.0)
    z_axis = (0.0, 0.0, -1.0)
    y_axis = (
        z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
        z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
        0.0,
    )
    return Transform.from_axes(x_axis, y_axis, z_axis, (x, y, z))


def render_depth(
    w: int,
    h: int,
    K: dict,
    T_bc: Transform,
    boxes: Sequence[Box],
    *,
    table_z: float,
    scale_m: float = 0.001,
    holes: Sequence[tuple[int, int, int, int]] = (),
) -> bytes:
    """uint16 little-endian depth, ``scale_m`` per unit. ``holes``: pixel rectangles
    ``(x0, y0, x1, y1)`` read as 0 (no depth), the way white foam drops out."""
    out = bytearray(w * h * 2)
    o = T_bc.translation
    locals_ = [_box_frame(b, table_z) for b in boxes]
    for v in range(h):
        for u in range(w):
            if any(x0 <= u < x1 and y0 <= v < y1 for x0, y0, x1, y1 in holes):
                continue
            dc = ((u - K["ppx"]) / K["fx"], (v - K["ppy"]) / K["fy"], 1.0)  # camera Z = 1
            d = T_bc.rotate(dc)
            best = math.inf
            if d[2] < -1e-9:
                t = (table_z - o[2]) / d[2]
                if t > 0:
                    best = t
            for (T_inv, half), b in zip(locals_, boxes, strict=True):
                hit = _can if b.round else _slab
                t = hit(T_inv.apply(o), T_inv.rotate(d), half)
                if t is not None and t < best:
                    best = t
            if math.isfinite(best):
                q = int(round(best / scale_m))  # t is the camera-frame Z: dc's Z is 1
                if 0 < q < 65536:
                    out[2 * (v * w + u)] = q & 0xFF
                    out[2 * (v * w + u) + 1] = q >> 8
    return bytes(out)


def _box_frame(b: Box, table_z: float) -> tuple[Transform, tuple[float, float, float]]:
    c, s = math.cos(b.theta), math.sin(b.theta)
    T = Transform.from_axes((c, s, 0.0), (-s, c, 0.0), (0.0, 0.0, 1.0), (b.x, b.y, table_z + b.height / 2))
    return T.inverse(), (b.length / 2, b.width / 2, b.height / 2)


def _slab(o: Sequence[float], d: Sequence[float], half: Sequence[float]) -> float | None:
    t0, t1 = -math.inf, math.inf
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if abs(o[i]) > half[i]:
                return None
            continue
        a, b = (-half[i] - o[i]) / d[i], (half[i] - o[i]) / d[i]
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
        if t0 > t1:
            return None
    return t0 if t0 > 0 else None


def shade(w: int, h: int, depth: bytes, scale_m: float = 0.001) -> bytes:
    """An RGB picture of a depth frame for a fake camera: nearer is lighter, no depth is black."""
    zs = [depth[2 * i] | (depth[2 * i + 1] << 8) for i in range(w * h)]
    valid = [z for z in zs if z]
    lo, hi = (min(valid), max(valid)) if valid else (0, 1)
    span = max(1, hi - lo)
    out = bytearray(w * h * 3)
    for i, z in enumerate(zs):
        if z:
            g = 230 - int(150 * (z - lo) / span)
            out[3 * i : 3 * i + 3] = bytes((g, g, min(255, g + 12)))
    return bytes(out)


# A 5 x 7 bitmap font: just enough to stamp a simulated picture as what it is.
_GLYPHS = {
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "C": ("01110", "10001", "10000", "10000", "10000", "10001", "01110"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "I": ("01110", "00100", "00100", "00100", "00100", "00100", "01110"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "-": ("00000", "00000", "00000", "11111", "00000", "00000", "00000"),
    " ": ("00000",) * 7,
}
NO_CAMERA = "NO CAMERA CONNECTED - SIMULATED TEST SCENE"


def banner_height(w: int, h: int, text: str = NO_CAMERA) -> int:
    """How tall :func:`stamp`'s band is for a ``w`` x ``h`` picture."""
    k = max(1, min((w - 16) // (len(text) * 6 - 1), h // 40 + 1))
    return 11 * k


def stamp(rgb: bytearray, w: int, h: int, text: str = NO_CAMERA, *, y0: int | None = None) -> None:
    """Burn ``text`` into a band across ``rgb`` (in place): white capitals on red, as large as
    the width allows — a picture that says it is not a camera's."""
    cols = len(text) * 6 - 1
    k = max(1, min((w - 16) // cols, h // 40 + 1))
    band = 11 * k
    top = (h - band) // 2 if y0 is None else y0
    for y in range(max(0, top), min(h, top + band)):
        rgb[3 * y * w : 3 * (y + 1) * w] = b"\xb4\x23\x23" * w
    x0 = (w - cols * k) // 2
    for n, ch in enumerate(text.upper()):
        glyph = _GLYPHS.get(ch, _GLYPHS[" "])
        for gy, row in enumerate(glyph):
            for gx, bit in enumerate(row):
                if bit != "1":
                    continue
                for dy in range(k):
                    y = top + 2 * k + gy * k + dy
                    if not 0 <= y < h:
                        continue
                    for dx in range(k):
                        x = x0 + (n * 6 + gx) * k + dx
                        if 0 <= x < w:
                            rgb[3 * (y * w + x) : 3 * (y * w + x) + 3] = b"\xff\xff\xff"


class BoxSceneCamera:
    """An RGB-D camera (the cockpit's camera interface) over a fixed box scene seen from a
    fixed pose — for tests and a demo cockpit with no D435. Its colour picture is stamped
    NO CAMERA CONNECTED top and bottom (``banner=False``: none) — the parts are real depth for
    the detector, and nobody should take the picture for a camera's."""

    def __init__(
        self,
        boxes: Sequence[Box],
        T_bc: Transform,
        *,
        w: int = 320,
        h: int = 180,
        fx: float = 230.0,
        table_z: float = -0.27,
        banner: bool = True,
    ):
        self.w, self.h = w, h
        self.K = {"fx": fx, "fy": fx, "ppx": w / 2, "ppy": h / 2}
        self.depth = render_depth(w, h, self.K, T_bc, boxes, table_z=table_z)
        rgb = bytearray(shade(w, h, self.depth))
        if banner:  # top and bottom: the parts in the middle stay visible
            stamp(rgb, w, h, y0=4)
            stamp(rgb, w, h, y0=h - 4 - banner_height(w, h))
        self.rgb = bytes(rgb)
        self._open = False
        self._n = 0

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def read(self):
        import time

        from .frame import Frame
        from .rgbd import DepthImage, Intrinsics, RgbdFrame

        time.sleep(0.02)
        self._n += 1
        K = self.K
        return RgbdFrame(
            color=Frame(width=self.w, height=self.h, data=self.rgb, channels=3),
            depth=DepthImage(width=self.w, height=self.h, data=self.depth, scale_m=0.001),
            intrinsics=Intrinsics(
                width=self.w, height=self.h, fx=K["fx"], fy=K["fy"], ppx=K["ppx"], ppy=K["ppy"]
            ),
            timestamp_ms=0.0,
            frame_number=self._n,
            aligned=True,
            extra={"serial": "BOXES"},
        )

    def describe(self) -> dict:
        return {
            "kind": "boxes",
            "open": self._open,
            "device": {"serial": "BOXES"},
            "intrinsics": dict(self.K),
        }


def _can(o: Sequence[float], d: Sequence[float], half: Sequence[float]) -> float | None:
    """The nearest hit on an upright cylinder of radius ``half[0]``, ``±half[2]`` tall."""
    r, hz = half[0], half[2]
    t0, t1 = -math.inf, math.inf
    a = d[0] * d[0] + d[1] * d[1]
    if a < 1e-18:
        if o[0] * o[0] + o[1] * o[1] > r * r:
            return None
    else:
        b = o[0] * d[0] + o[1] * d[1]
        disc = b * b - a * (o[0] * o[0] + o[1] * o[1] - r * r)
        if disc < 0:
            return None
        root = math.sqrt(disc)
        t0, t1 = (-b - root) / a, (-b + root) / a
    if abs(d[2]) < 1e-12:
        if abs(o[2]) > hz:
            return None
    else:
        za, zb = (-hz - o[2]) / d[2], (hz - o[2]) / d[2]
        if za > zb:
            za, zb = zb, za
        t0, t1 = max(t0, za), min(t1, zb)
    if t0 > t1:
        return None
    return t0 if t0 > 0 else None
