"""Monocular frame sources for the scan: one 2D image + the host time it arrived.

* :class:`RealSenseMono` — the RealSense **as a plain 2D camera**: colour
  stream only (no depth, no alignment, no filters); intrinsics from the SDK.
  The D435's colour imager is *rolling shutter* — fine for static capture,
  skewed during a fast sweep (`docs/mono-scan.md` §1a); its left IR imager is
  global shutter and is the stream to bind next.
* :class:`SyntheticMono` — renders white boxes on a dark table from any flange
  pose (a pinhole model, painter's algorithm, anti-aliased edges) so the whole
  sweep → locate chain is testable, numerically, without hardware.

Both yield :class:`MonoFrame` (grayscale numpy array + ``host_t`` from the same
monotonic clock the pose recorder stamps with). numpy + OpenCV (the
``perception`` extra) are required by the synthetic renderer and the locator.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

from urctl.pose import Transform

from .rgbd import Intrinsics


@dataclass
class MonoFrame:
    gray: Any  # (H, W) uint8 numpy array
    host_t: float
    seq: int
    camera_t_ms: float | None = None

    @property
    def width(self) -> int:
        return int(self.gray.shape[1])

    @property
    def height(self) -> int:
        return int(self.gray.shape[0])


class RealSenseMono:
    """2D frames from a RealSense via :class:`perception.realsense.RealSenseCamera`.

    ``stream="color"`` (default): the colour imager — rolling shutter, hand-eye
    through the SDK's depth→colour extrinsics. ``stream="ir"``: the **left IR
    imager** — global shutter, *is* the depth frame (so the calibrated hand-eye
    applies directly, no extrinsics), projector turned off so its dots don't
    paint the scene. The depth stream keeps running either way and the newest
    aligned RGB-D frame is kept on :attr:`last_rgbd` for the depth cross-check.
    """

    def __init__(
        self,
        camera=None,
        *,
        stream: str = "color",
        clock: Callable[[], float] = time.monotonic,
        **camera_kwargs,
    ):
        if stream not in ("color", "ir"):
            raise ValueError("stream must be 'color' or 'ir'")
        self.stream = stream
        if camera is None:
            from .realsense import DepthTuning, RealSenseCamera

            camera_kwargs.setdefault("align", True)
            camera_kwargs.setdefault("filters", None)
            if stream == "ir":
                camera_kwargs["infrared"] = True
                camera_kwargs.setdefault("tuning", DepthTuning(preset=None, laser_power=None, emitter=False))
            camera = RealSenseCamera(**camera_kwargs)
        elif stream == "ir":
            camera.infrared = True
        self.camera = camera
        self._clock = clock
        self._seq = 0
        self.last_rgbd = None

    def open(self) -> None:
        self.camera.open()

    def close(self) -> None:
        self.camera.close()

    def __enter__(self) -> RealSenseMono:
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def frame_kind(self) -> str:
        """Which hand-eye leg applies: ``"color"`` or ``"depth"`` (the IR imager)."""
        return "depth" if self.stream == "ir" else "color"

    @property
    def intrinsics(self) -> Intrinsics:
        key = "infrared" if self.stream == "ir" else "color"
        intr = self.camera.intrinsics.get(key) or (
            self.camera.intrinsics.get("depth") if key == "infrared" else None
        )
        if intr is None:
            raise RuntimeError(f"camera is not open (no {key} intrinsics yet)")
        return intr

    def describe(self) -> dict:
        d = self.camera.describe()
        d["kind"] = f"realsense-{self.stream}"
        d["shutter"] = "global" if self.stream == "ir" else "rolling"
        d["frame_kind"] = self.frame_kind
        return d

    def read(self) -> MonoFrame:
        import numpy as np

        f = self.camera.read()
        t = self._clock()
        self.last_rgbd = (f, t)
        if self.stream == "ir":
            ir = f.extra.get("ir")
            if ir is None:
                raise RuntimeError("no infrared frame in the frameset (is the IR stream enabled?)")
            gray = ir.to_numpy()[..., 0]
            ts = f.extra.get("ir_timestamp_ms")
        else:
            rgb = f.color.to_numpy()
            gray = (0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]).astype(np.uint8)
            ts = f.timestamp_ms
        self._seq += 1
        return MonoFrame(np.ascontiguousarray(gray), t, self._seq, ts)


# ----- synthetic scene -------------------------------------------------------


@dataclass
class Box:
    """An upright box on the table: ``center`` is the footprint centre on the
    table plane (base frame), ``size`` = (lx, ly, lz) with lz the height,
    ``yaw`` rotates lx about the table normal."""

    center: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0
    shade: int = 235

    def corners_base(self, plane) -> list[tuple[float, float, float]]:
        e1, e2 = plane.axes()
        n = plane.normal
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ax = [c * e1[i] + s * e2[i] for i in range(3)]
        ay = [-s * e1[i] + c * e2[i] for i in range(3)]
        lx, ly, lz = self.size
        out = []
        for dz in (0.0, lz):
            for sx, sy in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)):
                out.append(
                    tuple(self.center[i] + sx * lx * ax[i] + sy * ly * ay[i] + dz * n[i] for i in range(3))
                )
        return out  # type: ignore[return-value]


_FACES = [  # (corner indices CCW seen from outside, shade multiplier)
    ((4, 5, 6, 7), 1.0),  # top
    ((0, 1, 5, 4), 0.85),
    ((1, 2, 6, 5), 0.78),
    ((2, 3, 7, 6), 0.85),
    ((3, 0, 4, 7), 0.78),
    ((0, 3, 2, 1), 0.5),  # bottom (never visible from above)
]


@dataclass
class SyntheticMono:
    """Render :attr:`boxes` on :attr:`plane` as seen from ``T_base_cam =
    flange_pose_fn(t) · flange_to_color``. ``latency_s`` delays the *reported*
    host time relative to the exposure (frame exposed at ``t``, handed over at
    ``t + latency_s``) so the sync fit has something real to find."""

    intrinsics: Intrinsics
    plane: Any
    boxes: list[Box]
    flange_to_color: Transform
    flange_pose_fn: Callable[[float], Sequence[float]]
    latency_s: float = 0.0
    background: int = 60
    noise: float = 0.0
    clock: Callable[[], float] = time.monotonic
    _seq: int = field(default=0, init=False)

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def __enter__(self) -> SyntheticMono:
        return self

    def __exit__(self, *exc) -> None:
        pass

    def describe(self) -> dict:
        return {
            "kind": "synthetic-mono",
            "shutter": "global",
            "intrinsics": self.intrinsics.__dict__,
            "boxes": len(self.boxes),
            "latency_s": self.latency_s,
        }

    def read(self) -> MonoFrame:
        exposure_t = self.clock()
        gray = self.render(self.flange_pose_fn(exposure_t))
        # hand the frame over latency_s after the exposure (render time is
        # absorbed: the stamp is exposure + latency, not "now")
        rest = exposure_t + self.latency_s - self.clock()
        if rest > 0:
            time.sleep(rest)
        self._seq += 1
        return MonoFrame(gray, exposure_t + self.latency_s, self._seq, exposure_t * 1000.0)

    def render(self, flange_pose: Sequence[float]):
        import cv2
        import numpy as np

        intr = self.intrinsics
        t_base_cam = Transform.from_pose(flange_pose).compose(self.flange_to_color)
        t_cam_base = t_base_cam.inverse()
        img = np.full((intr.height, intr.width), self.background, dtype=np.uint8)
        cam_c = t_base_cam.translation
        polys: list[tuple[float, np.ndarray, int]] = []
        for box in self.boxes:
            corners = box.corners_base(self.plane)
            cam_pts = [t_cam_base.apply(c) for c in corners]
            for idx, mult in _FACES:
                p = [corners[i] for i in idx]
                # outward normal from the CCW winding; visible when facing the camera
                u = [p[1][k] - p[0][k] for k in range(3)]
                v = [p[2][k] - p[0][k] for k in range(3)]
                nrm = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                view = [p[0][k] - cam_c[k] for k in range(3)]
                if sum(nrm[k] * view[k] for k in range(3)) >= 0:
                    continue
                cp = [cam_pts[i] for i in idx]
                if any(c[2] <= 1e-6 for c in cp):
                    continue
                pix = np.array(
                    [[intr.fx * c[0] / c[2] + intr.ppx, intr.fy * c[1] / c[2] + intr.ppy] for c in cp],
                    dtype=np.float64,
                )
                depth = float(np.mean([c[2] for c in cp]))
                polys.append((depth, pix, int(min(255, box.shade * mult))))
        shift = 4
        for _, pix, shade in sorted(polys, key=lambda t: -t[0]):
            pts = np.round(pix * (1 << shift)).astype(np.int32).reshape(1, -1, 2)
            cv2.fillPoly(img, pts, shade, lineType=cv2.LINE_AA, shift=shift)
        if self.noise > 0:
            rng = np.random.default_rng(self._seq)
            img = np.clip(img.astype(np.float64) + rng.normal(0, self.noise, img.shape), 0, 255).astype(
                np.uint8
            )
        return img


def default_intrinsics(width: int = 848, height: int = 480, hfov_deg: float = 69.0) -> Intrinsics:
    """A D435-colour-like pinhole (69° HFOV) for the synthetic camera."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return Intrinsics(width=width, height=height, fx=fx, fy=fx, ppx=width / 2.0, ppy=height / 2.0)
