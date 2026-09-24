"""The work surface as a plane in the robot base frame.

Everything the monocular scan measures is a height *above this plane*
(`docs/mono-scan.md` §3), so the plane is calibration data, taught once:

* ``Plane.from_z(z)`` — a horizontal surface at base-frame ``z`` (the usual
  case for an arm bolted to the same table it picks from; one touch with the
  probe gives ``z``), or ``PERCEPTION_TABLE_Z`` in the cell file;
* ``Plane.from_points(a, b, c)`` — three probe touches (``perception touch``),
  for a surface that isn't level with the base;
* :func:`plane_from_depth` — **no probe needed**: a RANSAC plane through one
  RGB-D frame of the empty surface, moved into the base frame through the
  flange pose + hand-eye (``perception table-from-depth``; needs numpy).

Saved as ``captures/calibration/table_<cell>.json``. Pure stdlib.
"""

from __future__ import annotations

import json
import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

ENV_TABLE_Z = "PERCEPTION_TABLE_Z"
ENV_TABLE_FILE = "PERCEPTION_TABLE_FILE"
DEFAULT_TABLE_DIR = "captures/calibration"

Vec3 = tuple[float, float, float]


def _norm(v: Sequence[float]) -> Vec3:
    n = math.sqrt(sum(float(x) * float(x) for x in v))
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


@dataclass(frozen=True)
class Plane:
    """``normal · (p - point) = 0``; ``normal`` is the *up* direction (unit)."""

    point: Vec3
    normal: Vec3
    source: str = "explicit"

    @classmethod
    def from_z(cls, z: float, *, source: str = "z") -> Plane:
        return cls((0.0, 0.0, float(z)), (0.0, 0.0, 1.0), source)

    @classmethod
    def from_points(
        cls, a: Sequence[float], b: Sequence[float], c: Sequence[float], *, up_hint=(0, 0, 1)
    ) -> Plane:
        """Plane through three base-frame points; the normal is flipped to
        agree with ``up_hint`` so 'above' means away from the table."""
        pa = tuple(float(v) for v in a[:3])
        pb = tuple(float(v) for v in b[:3])
        pc = tuple(float(v) for v in c[:3])
        n = _cross([pb[i] - pa[i] for i in range(3)], [pc[i] - pa[i] for i in range(3)])
        if math.sqrt(_dot(n, n)) < 1e-9:
            raise ValueError("the three points are collinear")
        n = _norm(n)
        if _dot(n, up_hint) < 0:
            n = (-n[0], -n[1], -n[2])
        centroid = tuple((pa[i] + pb[i] + pc[i]) / 3.0 for i in range(3))
        return cls(centroid, n, "points")  # type: ignore[arg-type]

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None) -> Plane | None:
        """``$PERCEPTION_TABLE_Z``, else the saved file for the cell, else None."""
        env = os.environ if env is None else env
        raw = env.get(ENV_TABLE_Z, "").strip()
        if raw:
            return cls.from_z(float(raw), source=f"env:{ENV_TABLE_Z}")
        path = default_table_path(env)
        if os.path.isfile(path):
            return cls.load(path)
        return None

    def height_of(self, p: Sequence[float]) -> float:
        return _dot(self.normal, [float(p[i]) - self.point[i] for i in range(3)])

    def offset(self, h: float) -> Plane:
        """The parallel plane ``h`` above this one."""
        return Plane(tuple(self.point[i] + h * self.normal[i] for i in range(3)), self.normal, self.source)  # type: ignore[arg-type]

    def axes(self) -> tuple[Vec3, Vec3]:
        """In-plane unit axes ``(e1, e2)``: e1 is base +X projected onto the
        plane (falls back to +Y when the plane is vertical along X), e2 = n × e1."""
        n = self.normal
        for seed in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)):
            d = _dot(seed, n)
            e1 = [seed[i] - d * n[i] for i in range(3)]
            if math.sqrt(_dot(e1, e1)) > 1e-6:
                e1n = _norm(e1)
                return e1n, _norm(_cross(n, e1n))
        raise ValueError("degenerate plane normal")

    def to_plane_xy(self, p: Sequence[float]) -> tuple[float, float]:
        e1, e2 = self.axes()
        d = [float(p[i]) - self.point[i] for i in range(3)]
        return _dot(d, e1), _dot(d, e2)

    def from_plane_xy(self, x: float, y: float, h: float = 0.0) -> Vec3:
        e1, e2 = self.axes()
        return tuple(self.point[i] + x * e1[i] + y * e2[i] + h * self.normal[i] for i in range(3))  # type: ignore[return-value]

    def intersect(self, origin: Sequence[float], direction: Sequence[float]) -> Vec3 | None:
        """Where the ray ``origin + t·direction`` (t > 0) meets the plane."""
        denom = _dot(self.normal, direction)
        if abs(denom) < 1e-12:
            return None
        t = _dot(self.normal, [self.point[i] - float(origin[i]) for i in range(3)]) / denom
        if t <= 0:
            return None
        return tuple(float(origin[i]) + t * float(direction[i]) for i in range(3))  # type: ignore[return-value]

    def as_dict(self) -> dict:
        return {"point": list(self.point), "normal": list(self.normal), "source": self.source}

    def save(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.as_dict(), indent=2))
        return p

    @classmethod
    def load(cls, path: str | Path) -> Plane:
        d = json.loads(Path(path).read_text())
        return cls(tuple(float(v) for v in d["point"]), _norm(d["normal"]), f"file:{path}")  # type: ignore[arg-type]


def default_table_path(env: Mapping[str, str] | None = None) -> str:
    env = os.environ if env is None else env
    explicit = env.get(ENV_TABLE_FILE, "").strip()
    if explicit:
        return explicit
    cell = env.get("UR_CELL", "").strip()
    return os.path.join(DEFAULT_TABLE_DIR, f"table_{cell}.json" if cell else "table.json")


def plane_from_depth(
    frame,
    t_base_cam,
    *,
    stride: int = 6,
    iters: int = 300,
    inlier_m: float = 0.004,
    min_depth_m: float = 0.1,
    max_depth_m: float = 2.0,
    seed: int = 0,
) -> tuple[Plane, dict]:
    """RANSAC + least-squares plane through the valid depth of one
    :class:`~perception.rgbd.RgbdFrame` (aligned or not — ``frame.intrinsics``
    describe its depth pixels), returned in the **base** frame via ``t_base_cam``
    (the camera → base :class:`urctl.pose.Transform` of that frame's stream).
    The normal is oriented toward the camera, i.e. *up* off the surface.
    Objects on the surface are outliers and ignored as long as the surface
    dominates the view — take the frame over a clear patch."""
    import numpy as np

    intr = frame.intrinsics
    d = np.frombuffer(frame.depth.data, dtype="<u2").reshape(frame.depth.height, frame.depth.width)
    ys, xs = np.mgrid[0 : d.shape[0] : stride, 0 : d.shape[1] : stride]
    z = d[ys, xs].astype(np.float64) * frame.depth.scale_m
    ok = (z >= min_depth_m) & (z <= max_depth_m)
    xs, ys, z = xs[ok].astype(np.float64), ys[ok].astype(np.float64), z[ok]
    if len(z) < 50:
        raise ValueError(f"only {len(z)} valid depth samples — no surface in view")
    pts_cam = np.stack([(xs - intr.ppx) / intr.fx * z, (ys - intr.ppy) / intr.fy * z, z], axis=1)
    rng = np.random.default_rng(seed)
    best_inl = None
    best_n = 0
    for _ in range(iters):
        i = rng.choice(len(pts_cam), 3, replace=False)
        a, b, c = pts_cam[i]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        dist = np.abs((pts_cam - a) @ n)
        inl = dist < inlier_m
        cnt = int(inl.sum())
        if cnt > best_n:
            best_n, best_inl = cnt, inl
    if best_inl is None or best_n < 50:
        raise ValueError("RANSAC found no plane (depth too noisy or too sparse)")
    inl_pts = pts_cam[best_inl]
    centroid = inl_pts.mean(axis=0)
    _, _, vt = np.linalg.svd(inl_pts - centroid, full_matrices=False)
    n_cam = vt[2]
    if n_cam @ (-centroid) < 0:  # point toward the camera (the camera sits at the origin)
        n_cam = -n_cam
    rms = float(np.sqrt((((inl_pts - centroid) @ n_cam) ** 2).mean()))
    point = t_base_cam.apply(tuple(float(v) for v in centroid))
    normal = _norm(t_base_cam.rotate(tuple(float(v) for v in n_cam)))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, normal[2]))))
    plane = Plane(point, normal, "depth")
    return plane, {
        "samples": int(len(pts_cam)),
        "inliers": int(best_n),
        "inlier_fraction": best_n / len(pts_cam),
        "rms_m": rms,
        "tilt_from_base_z_deg": tilt,
        "camera_height_m": float(-(centroid @ n_cam)),
    }
