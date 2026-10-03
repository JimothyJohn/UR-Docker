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


# -- what a real D435 does to the perfect frame ------------------------------------------------


def camera_looking_at(
    eye: Sequence[float], target: Sequence[float], right: Sequence[float] = (1.0, 0.0, 0.0)
) -> Transform:
    """A camera at ``eye`` with its optical axis through ``target``; image right as close
    to ``right`` (base frame) as the axis allows — a tilted picture pose or close look."""
    z = _unit(tuple(target[i] - eye[i] for i in range(3)))
    k = sum(right[i] * z[i] for i in range(3))
    x = _unit(tuple(right[i] - k * z[i] for i in range(3)))
    y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0])
    return Transform.from_axes(x, y, z, eye)


@dataclass(frozen=True)
class Sensor:
    """How a filtered D435 departs from the ray-cast truth, each effect sized from a real
    frame (the Pi's D435 over a carpet at 0.63-1.12 m, 2026-10-03: 0.37 mm pixel jitter,
    a plane fit's residual 4 mm RMS in broad swells) and the stereo geometry:

    - ``jitter_m``: per-pixel noise σ at 1 m; grows with z² (stereo disparity error);
    - ``warp_m``: broad low-frequency swells (the D435's bowing, a not-quite-flat table)
      with this RMS at 1 m, ∝ z²;
    - ``baseline_m``: the second imager sits this far along camera +X — background it can't
      see past a part's edge reads 0 (the stereo shadow down one side of every part);
    - ``skirt_px``: depth edges smear over this many pixels (sub-pixel matching and the
      spatial filter put points between the top and the table: flying pixels);
    - ``dropout``: the chance each part's top loses a blob of depth (dark, shiny, or
      white-foam tops that the projector's dots wash out on)."""

    jitter_m: float = 0.0005
    warp_m: float = 0.0055
    baseline_m: float = 0.050
    skirt_px: int = 1
    dropout: float = 0.0


D435 = Sensor()


def sense(
    depth: bytes,
    w: int,
    h: int,
    K: dict,
    T_bc: Transform,
    boxes: Sequence[Box],
    *,
    table_z: float,
    sensor: Sensor = D435,
    seed: int = 0,
    scale_m: float = 0.001,
) -> bytes:
    """``depth`` (from :func:`render_depth` with the same scene) as ``sensor`` would read it.
    Deterministic for a ``seed``."""
    import random

    rng = random.Random(seed)
    z = [(depth[2 * i] | (depth[2 * i + 1] << 8)) * scale_m for i in range(w * h)]
    fx, fy, ppx, ppy = K["fx"], K["fy"], K["ppx"], K["ppy"]
    out = list(z)
    # the stereo shadow: the second imager's line of sight to the point is blocked by a part
    if sensor.baseline_m and boxes:
        eye = T_bc.apply((sensor.baseline_m, 0.0, 0.0))
        frames = [_box_frame(b, table_z) for b in boxes]
        for v in range(h):
            for u in range(w):
                zz = z[v * w + u]
                if not zz:
                    continue
                p = T_bc.apply(((u - ppx) / fx * zz, (v - ppy) / fy * zz, zz))
                d = (p[0] - eye[0], p[1] - eye[1], p[2] - eye[2])
                for (T_inv, half), b in zip(frames, boxes, strict=True):
                    t = (_can if b.round else _slab)(T_inv.apply(eye), T_inv.rotate(d), half)
                    if t is not None and t < 1.0 - 0.002 / max(zz, 0.05):
                        out[v * w + u] = 0.0
                        break
    # flying pixels: within skirt_px of a depth step, somewhere between the two sides
    if sensor.skirt_px:
        src = list(out)
        r = sensor.skirt_px
        for v in range(h):
            for u in range(w):
                a = src[v * w + u]
                if not a:
                    continue
                for dv in range(-r, r + 1):
                    vv = v + dv
                    if not 0 <= vv < h:
                        continue
                    for du in range(-r, r + 1):
                        uu = u + du
                        if not 0 <= uu < w:
                            continue
                        b = src[vv * w + uu]
                        if b and b < a - 0.01:  # a nearer surface next to this one
                            if rng.random() < 0.6:
                                out[v * w + u] = b + (a - b) * rng.random()
                            break
                    else:
                        continue
                    break
    # dropouts on part tops
    if sensor.dropout:
        T_cb = T_bc.inverse()
        for b in boxes:
            if rng.random() >= sensor.dropout:
                continue
            c = T_cb.apply((b.x, b.y, table_z + b.height))
            if c[2] <= 0:
                continue
            cu, cv = fx * c[0] / c[2] + ppx, fy * c[1] / c[2] + ppy
            rad = fx * b.width / c[2] * rng.uniform(0.15, 0.35)
            cu += rng.uniform(-0.3, 0.3) * fx * b.length / c[2]
            for v in range(max(0, int(cv - rad)), min(h, int(cv + rad) + 1)):
                for u in range(max(0, int(cu - rad)), min(w, int(cu + rad) + 1)):
                    if (u - cu) ** 2 + (v - cv) ** 2 <= rad * rad:
                        out[v * w + u] = 0.0
    # broad swells, then pixel jitter, then the 1-unit quantisation
    waves = []
    for _ in range(3):
        ang, lam = rng.uniform(0, math.pi), rng.uniform(0.4, 1.2) * w
        waves.append((math.cos(ang) / lam, math.sin(ang) / lam, rng.uniform(0, 2 * math.pi)))
    amp = sensor.warp_m * math.sqrt(2.0 / len(waves))
    buf = bytearray(w * h * 2)
    for v in range(h):
        for u in range(w):
            zz = out[v * w + u]
            if not zz:
                continue
            swell = sum(math.cos(2 * math.pi * (u * a + v * c) + ph) for a, c, ph in waves) * amp
            zz += (swell + rng.gauss(0.0, sensor.jitter_m)) * zz * zz
            q = int(round(zz / scale_m))
            if 0 < q < 65536:
                buf[2 * (v * w + u)] = q & 0xFF
                buf[2 * (v * w + u) + 1] = q >> 8
    return bytes(buf)


def _unit(a: Sequence[float]) -> tuple[float, float, float]:
    n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    return (a[0] / n, a[1] / n, a[2] / n)
