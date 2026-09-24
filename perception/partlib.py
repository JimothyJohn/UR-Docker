"""Part library — what the operator's CAD says a part looks like on the table.

An STL (binary or ASCII; STEP goes through CadQuery first, see ``step_to_stl``)
is reduced to what the monocular locator needs (`docs/mono-scan.md` §6):

* an **oriented bounding box** (PCA of the vertices, refined to the mesh's
  extents), and
* its **resting poses**: one per OBB axis pointing up — each gives the
  part's *height* above the table and its top-down *footprint* (the other two
  extents). Exact for boxes; an approximation for other shapes (a hull-based
  stability test is a follow-up).

``PartLibrary.match(footprint, height)`` names the part + resting pose a
detected blob agrees with. numpy is required (the ``perception`` extra); the
box writer that makes ``parts/block_50x30x30.stl`` is pure stdlib.
"""

from __future__ import annotations

import json
import math
import struct
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_DIM_TOL_M = 0.006
DEFAULT_HEIGHT_TOL_M = 0.006


# ----- STL I/O ---------------------------------------------------------------


def write_box_stl(path: str | Path, lx: float, ly: float, lz: float, *, name: str = "box") -> Path:
    """A closed axis-aligned box (12 triangles, outward normals) in **metres**,
    STL units mm, corner at the origin — what ``perception make-box`` writes."""
    if min(lx, ly, lz) <= 0:
        raise ValueError("box dimensions must be positive")
    x, y, z = lx * 1000.0, ly * 1000.0, lz * 1000.0
    v = [(0, 0, 0), (x, 0, 0), (x, y, 0), (0, y, 0), (0, 0, z), (x, 0, z), (x, y, z), (0, y, z)]
    faces = [  # CCW seen from outside
        ((0, 0, -1), [(0, 2, 1), (0, 3, 2)]),
        ((0, 0, 1), [(4, 5, 6), (4, 6, 7)]),
        ((0, -1, 0), [(0, 1, 5), (0, 5, 4)]),
        ((1, 0, 0), [(1, 2, 6), (1, 6, 5)]),
        ((0, 1, 0), [(2, 3, 7), (2, 7, 6)]),
        ((-1, 0, 0), [(3, 0, 4), (3, 4, 7)]),
    ]
    out = bytearray(name.encode("ascii")[:80].ljust(80, b"\0"))
    out += struct.pack("<I", 12)
    for n, tris in faces:
        for a, b, c in tris:
            out += struct.pack("<3f", *n)
            for i in (a, b, c):
                out += struct.pack("<3f", *v[i])
            out += struct.pack("<H", 0)
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(bytes(out))
    return p


def read_stl(path: str | Path):
    """``(N, 3, 3)`` float array of triangle vertices in **metres** (STL is mm).
    Binary or ASCII."""
    np = _np()
    raw = Path(path).read_bytes()
    if len(raw) >= 84:
        n = struct.unpack("<I", raw[80:84])[0]
        if 84 + n * 50 == len(raw):
            arr = np.frombuffer(
                raw[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n
            )
            return arr["v"].reshape(n, 3, 3).astype(np.float64) / 1000.0
    text = raw.decode("ascii", errors="replace")
    if not text.lstrip().startswith("solid"):
        raise ValueError(f"{path}: not an STL file")
    verts = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not verts or len(verts) % 3:
        raise ValueError(f"{path}: malformed ASCII STL")
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3, 3) / 1000.0


def step_to_stl(step_path: str | Path, stl_path: str | Path, *, tolerance: float = 0.05) -> Path:
    """Tessellate a STEP file with CadQuery (``uv run --with cadquery``) into a
    binary STL — the same optional dependency the hardware builds use."""
    try:
        import cadquery as cq
    except ImportError as exc:
        raise ImportError(
            "STEP import needs CadQuery: run under `uv run --with cadquery`, or export an STL from your CAD"
        ) from exc
    shape = cq.importers.importStep(str(step_path))
    cq.exporters.export(shape, str(stl_path), tolerance=tolerance, angularTolerance=0.1)
    return Path(stl_path)


# ----- geometry --------------------------------------------------------------


@dataclass(frozen=True)
class RestingPose:
    """The part lying with OBB axis ``up_axis`` pointing up."""

    up_axis: int
    height_m: float
    footprint_m: tuple[float, float]  # (long, short)
    label: str

    def as_dict(self) -> dict:
        return {
            "up_axis": self.up_axis,
            "height_m": self.height_m,
            "footprint_m": list(self.footprint_m),
            "label": self.label,
        }


@dataclass
class Part:
    name: str
    extents_m: tuple[float, float, float]  # OBB extents along its axes
    axes: list[list[float]]  # OBB axes (rows) in the mesh frame
    center_m: tuple[float, float, float]  # OBB centre in the mesh frame
    triangles: int = 0
    source: str = ""
    resting: list[RestingPose] = field(default_factory=list)

    @classmethod
    def from_mesh(cls, tris, *, name: str, source: str = "") -> Part:
        np = _np()
        tri = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
        pts = tri.reshape(-1, 3)
        uniq = np.unique(np.round(pts, 9), axis=0)
        axes, lo, hi = _min_volume_obb(uniq, tri)
        ext = tuple(float(v) for v in (hi - lo))
        centre = ((lo + hi) / 2.0) @ axes
        part = cls(name, ext, axes.tolist(), tuple(float(v) for v in centre), len(tri), source)
        part.resting = part._resting_poses()
        return part

    @classmethod
    def from_stl(cls, path: str | Path, *, name: str | None = None) -> Part:
        p = Path(path)
        return cls.from_mesh(read_stl(p), name=name or p.stem, source=str(p))

    def _resting_poses(self) -> list[RestingPose]:
        out = []
        for up in range(3):
            others = [self.extents_m[i] for i in range(3) if i != up]
            fp = (max(others), min(others))
            out.append(RestingPose(up, self.extents_m[up], fp, f"{self.name}:axis{up}-up"))
        return out

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "source": self.source,
            "triangles": self.triangles,
            "extents_m": list(self.extents_m),
            "center_m": list(self.center_m),
            "axes": self.axes,
            "resting_poses": [r.as_dict() for r in self.resting],
        }


@dataclass
class PartLibrary:
    parts: list[Part] = field(default_factory=list)

    @classmethod
    def from_paths(cls, paths: Sequence[str | Path]) -> PartLibrary:
        return cls([Part.from_stl(p) for p in paths])

    def add(self, part: Part) -> None:
        self.parts.append(part)

    def match(
        self,
        footprint_m: Sequence[float],
        height_m: float | None = None,
        *,
        dim_tol_m: float = DEFAULT_DIM_TOL_M,
        height_tol_m: float = DEFAULT_HEIGHT_TOL_M,
    ) -> dict | None:
        """Best (part, resting pose) for a measured footprint (long, short) and,
        when known, height. Returns ``{part, pose, score_m, ...}`` or None."""
        fp = sorted((float(footprint_m[0]), float(footprint_m[1])), reverse=True)
        best = None
        for part in self.parts:
            for rp in part.resting:
                d_long = abs(rp.footprint_m[0] - fp[0])
                d_short = abs(rp.footprint_m[1] - fp[1])
                if d_long > dim_tol_m or d_short > dim_tol_m:
                    continue
                d_h = 0.0 if height_m is None else abs(rp.height_m - float(height_m))
                if height_m is not None and d_h > height_tol_m:
                    continue
                score = math.sqrt(d_long * d_long + d_short * d_short + d_h * d_h)
                if best is None or score < best["score_m"]:
                    best = {
                        "part": part.name,
                        "pose": rp.as_dict(),
                        "score_m": score,
                        "footprint_error_m": [d_long, d_short],
                        "height_error_m": None if height_m is None else d_h,
                    }
        return best

    def candidate_heights(self) -> list[float]:
        return sorted({rp.height_m for p in self.parts for rp in p.resting})

    def as_dict(self) -> dict:
        return {"parts": [p.as_dict() for p in self.parts]}

    def save(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.as_dict(), indent=2))
        return p


def _min_volume_obb(uniq, tri):
    """Oriented bounding box: the smallest-volume box among frames built from
    the mesh's dominant face normals (area-weighted) plus the PCA frame. Exact
    for prisms/boxes, where PCA alone is ambiguous whenever two extents match.
    Returns ``(axes (3x3, rows), lo, hi)`` with lo/hi the extents along the axes."""
    np = _np()
    c = uniq.mean(axis=0)
    cov = np.cov((uniq - c).T) if len(uniq) > 3 else np.eye(3)
    _, vecs = np.linalg.eigh(cov)
    frames = [vecs.T[::-1].copy()]
    # dominant normals: cluster face normals by direction (sign-insensitive), rank by area
    e1 = tri[:, 1] - tri[:, 0]
    e2 = tri[:, 2] - tri[:, 0]
    n = np.cross(e1, e2)
    area = np.linalg.norm(n, axis=1)
    ok = area > 1e-14
    n, area = n[ok] / area[ok][:, None], area[ok]
    dirs: list[tuple[np.ndarray, float]] = []
    for v, a in sorted(zip(n, area, strict=True), key=lambda t: -t[1]):
        for i, (d, acc) in enumerate(dirs):
            if abs(float(d @ v)) > 0.9995:
                dirs[i] = (d, acc + a)
                break
        else:
            dirs.append((v, a))
    dirs.sort(key=lambda t: -t[1])
    top = [d for d, _ in dirs[:8]]
    for i, a in enumerate(top):
        for b in top[i + 1 :]:
            b2 = b - float(a @ b) * a
            if np.linalg.norm(b2) < 1e-6:
                continue
            b2 /= np.linalg.norm(b2)
            frames.append(np.stack([a, b2, np.cross(a, b2)]))
    best = None
    for f in frames:
        if np.linalg.det(f) < 0:
            f = f.copy()
            f[2] = -f[2]
        proj = uniq @ f.T
        lo, hi = proj.min(axis=0), proj.max(axis=0)
        vol = float(np.prod(hi - lo))
        if best is None or vol < best[0] - 1e-15:
            best = (vol, f, lo, hi)
    assert best is not None
    _, f, lo, hi = best
    order = np.argsort(-(hi - lo))  # longest extent first
    f, lo, hi = f[order], lo[order], hi[order]
    if np.linalg.det(f) < 0:
        f[2], lo[2], hi[2] = -f[2], -hi[2], -lo[2]
    return f, lo, hi


def _np():
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover
        raise ImportError("numpy is required for the part library (`uv sync --extra perception`)") from exc
    return np
