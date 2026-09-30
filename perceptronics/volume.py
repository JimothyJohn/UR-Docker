"""Parts by their volume: what stands the part's height above the work surface, the part's size.

The pick node's detector (2026-09-28, Nick: "look for a specific volume with expected
dimensions and align to the axes of the bounding box"). Colour plays no part: a part is
whatever stands off the surface, and it is *the* part when its top face measures the
part's length × width and stands its height above the surface — each within the
:class:`~perceptronics.partspec.PartSpec`'s tolerance. The foam blocks' white-threshold
misses in evening light (TODO 2026-09-27) cannot happen here.

One frame in, one :class:`Scene` out:

1. **Sample** the aligned depth on a grid (``stride`` px, chosen so a cell is ~3 mm on
   the surface) and place every sample in the base frame through the camera pose.
2. **The surface.** A taught plane (the Installation screen's pick location: three
   points touched with the fingertips) when there is one — its height nudged to the
   live frame by at most :data:`LIVE_NUDGE_M`, so a table that moved a few mm still
   works and one that moved more is reported. Without one, the surface is *fitted
   live*: level in the base frame (the table is flat and parallel to base XY — Nick,
   2026-09-27), at the most populated height; with no base frame (a camera-only
   preview) a RANSAC plane.
3. **Occupied cells** stand more than :func:`occupied_min_m` off the surface (and
   inside the taught area). Depth holes surrounded by occupied cells are filled; a
   real gap between two parts is not (it has the table's depth).
4. **Each 8-connected blob** is one candidate: its top face is the cells within
   :data:`TOP_BAND_M` of its (robust) highest height, and the top face's
   **minimum-area rectangle** in the surface's plane gives the centre, the axes and
   the length × width. The fingers close across the short side, so the grasp is
   parallel to the rectangle's short axis.
5. **Why not** — every candidate that isn't picked carries a reason the pendant shows:
   the wrong size (:meth:`PartSpec.why_not`), cut off by the picture's edge, outside the
   taught area, too close to the base or out of reach (:class:`Reach`), no room for the
   open fingers beside it (:func:`perceptronics.pickplan.clearance`).
6. **Pick order** — :func:`order_parts` numbers the pickable parts left→right /
   front→back (either way, rows either way) *as the picture shows them*, so the numbers
   the pendant draws are the order the program picks in.

Pure stdlib, O(cells): a Raspberry-Pi-class PC does a 848×480 frame in well under a
second at the default ~3 mm cells.
"""

from __future__ import annotations

import math
import random
from collections.abc import Sequence
from dataclasses import dataclass, field

from urctl.pose import Transform

from .partspec import PartSpec

Vec3 = tuple[float, float, float]

MAX_RANGE_M = 2.0  # a reflection reads metres away: not the part
CELL_M = 0.003  # target sample spacing on the surface
MIN_STRIDE, MAX_STRIDE = 2, 8
TOP_BAND_M = 0.006  # a flat top's own depth noise at working range
OCCUPIED_FLOOR_M = 0.005  # never call anything flatter than this a part
LIVE_NUDGE_M = 0.015  # a taught plane follows the live table by at most this much
SURFACE_BAND_M = 0.004
MIN_TOP_CELLS = 10
EDGE_CELLS = 1  # a blob touching the outermost cells is cut off by the picture's edge

ORDERS = ("LR", "RL", "FB", "BF")  # left→right, right→left, front→back, back→front
ORDER_WORDS = {
    "LR": "left to right",
    "RL": "right to left",
    "FB": "front to back",
    "BF": "back to front",
}
_AXIS = {"LR": "h", "RL": "h", "FB": "v", "BF": "v"}


# -- the work surface -----------------------------------------------------------------------


@dataclass(frozen=True)
class Surface:
    """A plane in the base frame: ``origin``, unit ``x_axis``/``y_axis`` in it and the unit
    ``normal`` pointing up off it (toward the camera). ``area`` — ``(x0, x1, y0, y1)`` in
    the plane's own coordinates — bounds where parts may be; None: anywhere."""

    origin: Vec3
    x_axis: Vec3
    y_axis: Vec3
    normal: Vec3
    area: tuple[float, float, float, float] | None = None
    source: str = "fitted"  # "taught" (the Installation screen's plane) or "fitted" (this frame)
    offset_m: float = 0.0  # a taught plane: how far the live table sat above it (the nudge applied)

    @classmethod
    def level(cls, z: float, source: str = "fitted") -> Surface:
        return cls((0.0, 0.0, z), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), None, source)

    @classmethod
    def from_pose(cls, pose: Sequence[float], size: Sequence[float] | None = None) -> Surface:
        """A plane pose (origin + rotation; its Z is the normal) and the area's
        ``(size_x, size_y)`` from the origin along +X / +Y — the pick node's ``plane=``
        and ``area=``. A normal pointing down is flipped (and Y with it)."""
        T = Transform.from_pose(pose)
        x, y, n = T.rotate((1.0, 0.0, 0.0)), T.rotate((0.0, 1.0, 0.0)), T.rotate((0.0, 0.0, 1.0))
        area = None
        if size is not None:
            sx, sy = float(size[0]), float(size[1])
            area = (min(0.0, sx), max(0.0, sx), min(0.0, sy), max(0.0, sy))
        if n[2] < 0:
            n = (-n[0], -n[1], -n[2])
            y = (-y[0], -y[1], -y[2])
            if area is not None:
                area = (area[0], area[1], -area[3], -area[2])
        return cls(T.translation, x, y, n, area, "taught")

    @classmethod
    def from_points(
        cls, origin: Sequence[float], on_x: Sequence[float], toward_y: Sequence[float]
    ) -> Surface:
        """Three touched points: the area's corner, a point along its X edge, a point on its
        far side. The normal points up; the area spans the corner to the other two."""
        o = _v(origin)
        x = _unit(_sub(_v(on_x), o))
        n = _unit(_cross(x, _sub(_v(toward_y), o)))
        if n[2] < 0:
            n = (-n[0], -n[1], -n[2])
        y = _cross(n, x)
        sx = _dot(_sub(_v(on_x), o), x)
        sy = _dot(_sub(_v(toward_y), o), y)
        area = (min(0.0, sx), max(0.0, sx), min(0.0, sy), max(0.0, sy))
        return cls(o, x, y, n, area, "taught")

    def height(self, p: Sequence[float]) -> float:
        return _dot(_sub(p, self.origin), self.normal)

    def local(self, p: Sequence[float]) -> tuple[float, float]:
        d = _sub(p, self.origin)
        return _dot(d, self.x_axis), _dot(d, self.y_axis)

    def point(self, u: float, v: float, h: float = 0.0) -> Vec3:
        o, x, y, n = self.origin, self.x_axis, self.y_axis, self.normal
        return tuple(o[i] + u * x[i] + v * y[i] + h * n[i] for i in range(3))  # type: ignore[return-value]

    def inside(self, p: Sequence[float], margin: float = 0.0) -> bool:
        if self.area is None:
            return True
        u, v = self.local(p)
        x0, x1, y0, y1 = self.area
        return x0 - margin <= u <= x1 + margin and y0 - margin <= v <= y1 + margin

    def shifted(self, dh: float) -> Surface:
        n = self.normal
        o = tuple(self.origin[i] + dh * n[i] for i in range(3))
        return Surface(o, self.x_axis, self.y_axis, n, self.area, self.source, self.offset_m + dh)  # type: ignore[arg-type]

    def tilt_deg(self) -> float:
        """The plane's tilt from base XY — the table is level, so this is teaching error."""
        return math.degrees(math.acos(max(-1.0, min(1.0, abs(self.normal[2])))))

    def as_dict(self) -> dict:
        return {
            "source": self.source,
            "origin": [round(v, 4) for v in self.origin],
            "normal": [round(v, 4) for v in self.normal],
            "tilt_deg": round(self.tilt_deg(), 2),
            "offset_mm": round(self.offset_m * 1000, 1),
            "area": None if self.area is None else [round(v, 4) for v in self.area],
        }


# -- reach ----------------------------------------------------------------------------------


# The base's outer radius (the round foot, from UR's e-Series datasheets' footprint Ø).
BASE_RADIUS_M: dict[str, float] = {
    "UR3E": 0.064,  # Ø128 mm
    "UR5E": 0.0745,  # Ø149 mm
    "UR7E": 0.0745,
    "UR10E": 0.095,  # Ø190 mm
    "UR12E": 0.095,
    "UR16E": 0.095,
}
REACH_MARGIN_M = 0.150  # Nick, 2026-09-28: "at least 150mm from the outer radius of the base"


@dataclass(frozen=True)
class Reach:
    """Where a part's centre may be, radially from the base's Z axis: ``min_m`` (the
    base's outer radius + the inner margin) to ``max_m`` (the rated reach − the outer
    margin). ``max_m`` 0: no outer limit."""

    min_m: float
    max_m: float = 0.0

    @classmethod
    def for_model(
        cls,
        model: str,
        *,
        inner_margin_m: float = REACH_MARGIN_M,
        outer_margin_m: float = REACH_MARGIN_M,
    ) -> Reach | None:
        from urctl.safety import normalize_model, reach_for_model

        key = normalize_model(model)
        base = BASE_RADIUS_M.get(key) or BASE_RADIUS_M.get(key + "E")
        reach = reach_for_model(model)
        if base is None or reach is None:
            return None
        return cls(base + inner_margin_m, max(0.0, reach - outer_margin_m))

    def why_not(self, centre: Sequence[float]) -> str | None:
        r = math.hypot(centre[0], centre[1])
        if r < self.min_m:
            return f"too close to the base ({r * 1000:.0f} < {self.min_m * 1000:.0f} mm)"
        if self.max_m > 0 and r > self.max_m:
            return f"out of reach ({r * 1000:.0f} > {self.max_m * 1000:.0f} mm)"
        return None

    def token(self) -> str:
        return f"reach={self.min_m:.3f},{self.max_m:.3f}"


# -- the result -----------------------------------------------------------------------------


@dataclass
class Part:
    """One candidate's top face, base frame: the rectangle's centre (at the top's height),
    the long side's heading in base XY, length ≥ width, the height above the surface; the
    corners in base and in picture pixels for the overlay; ``order`` (1…) once numbered;
    ``why`` when it is not going to be picked."""

    centre: Vec3
    theta: float
    length_m: float
    width_m: float
    height_m: float
    corners: list[Vec3]
    pixel: tuple[int, int]
    corners_px: list[tuple[int, int]]
    cells: int
    why: str | None = None
    order: int = 0
    near_edge: bool = field(default=False, repr=False)

    # the names pickcycle / picknode already use for a block
    @property
    def centre_base(self) -> list[float]:
        return list(self.centre)

    @property
    def major_m(self) -> float:
        return self.length_m

    @property
    def minor_m(self) -> float:
        return self.width_m

    @property
    def height_mm(self) -> float:
        return self.height_m * 1000

    def as_dict(self) -> dict:
        return {
            "order": self.order,
            "pixel": list(self.pixel),
            "corners_px": [list(c) for c in self.corners_px],
            "centre": [round(v, 4) for v in self.centre],
            "theta_deg": round(math.degrees(self.theta), 1),
            "size_mm": [round(self.length_m * 1000), round(self.width_m * 1000)],
            "height_mm": round(self.height_m * 1000),
            "why": self.why,
        }


@dataclass
class Scene:
    parts: list[Part]  # pickable, in pick order (``order`` 1…)
    rejected: list[Part]  # everything else found, each with ``why``
    surface: Surface | None
    stride: int
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "parts": [p.as_dict() for p in self.parts],
            "rejected": [p.as_dict() for p in self.rejected],
            "surface": None if self.surface is None else self.surface.as_dict(),
            "stride": self.stride,
            "notes": list(self.notes),
        }


# -- the pipeline ---------------------------------------------------------------------------


def occupied_min_m(spec: PartSpec | None) -> float:
    """Anything standing this far off the surface is *something*: half the part's height
    (never under :data:`OCCUPIED_FLOOR_M`), 8 mm without a height."""
    if spec is not None and spec.height_m is not None:
        return max(OCCUPIED_FLOOR_M, 0.5 * spec.height_m - spec.slack(spec.height_m))
    return 0.008


def choose_stride(w: int, h: int, depth: bytes, scale: float, K: dict, cell_m: float = CELL_M) -> int:
    """The pixel stride that makes one sample ~``cell_m`` on the surface at this frame's
    median range."""
    zs = []
    for y in range(0, h, 16):
        for x in range(0, w, 16):
            d = depth[2 * (y * w + x)] | (depth[2 * (y * w + x) + 1] << 8)
            if d and d * scale <= MAX_RANGE_M:
                zs.append(d * scale)
    if not zs:
        return 4
    zs.sort()
    z = zs[len(zs) // 2]
    return max(MIN_STRIDE, min(MAX_STRIDE, round(cell_m * K["fx"] / z)))


def find_parts(
    w: int,
    h: int,
    depth: bytes,
    depth_scale_m: float,
    K: dict,
    T_bc: Transform | None,
    *,
    spec: PartSpec | None = None,
    surface: Surface | None = None,
    reach: Reach | None = None,
    order: tuple[str, str] = ("LR", "FB"),
    fingers: dict | None = None,
    stride: int | None = None,
) -> Scene:
    """The parts in one aligned depth frame (uint16 LE, ``depth_scale_m`` per unit, colour
    intrinsics ``K``). ``T_bc``: the colour camera's pose in the base frame at the frame's
    instant (flange pose ∘ hand-eye); None for a camera-only preview (no reach, no base
    heading — the camera frame stands in for the base). ``fingers``: keyword arguments for
    :func:`perceptronics.pickplan.clearance` (``stroke_m``, ``grasp_below_m``, …) — the open
    fingers' room is checked when given."""
    level_ok = T_bc is not None
    T = T_bc if T_bc is not None else Transform()
    if stride is None:
        stride = choose_stride(w, h, depth, depth_scale_m, K)
    gw, gh = (w + stride - 1) // stride, (h + stride - 1) // stride
    fx, fy, ppx, ppy = K["fx"], K["fy"], K["ppx"], K["ppy"]
    pts: list[Vec3 | None] = [None] * (gw * gh)
    zc: list[float] = [0.0] * (gw * gh)
    for j in range(gh):
        y = j * stride
        row = y * w
        for i in range(gw):
            x = i * stride
            k = 2 * (row + x)
            d = depth[k] | (depth[k + 1] << 8)
            if not d:
                continue
            z = d * depth_scale_m
            if z > MAX_RANGE_M:
                continue
            pts[j * gw + i] = T.apply(((x - ppx) * z / fx, (y - ppy) * z / fy, z))
            zc[j * gw + i] = z
    notes: list[str] = []
    valid = [p for p in pts if p is not None]
    if len(valid) < 50:
        return Scene([], [], surface, stride, ["almost no depth in this frame"])

    surf = _surface(valid, surface, level_ok, spec, notes)
    if surf is None:
        return Scene([], [], None, stride, notes + ["no work surface found in the picture"])

    occ_min = occupied_min_m(spec)
    hmax = 0.5 if spec is None or spec.height_m is None else 2.5 * spec.height_m + 0.02
    hs: list[float | None] = [None] * (gw * gh)
    occ = bytearray(gw * gh)
    for k, p in enumerate(pts):
        if p is None:
            continue
        hk = surf.height(p)
        hs[k] = hk
        if occ_min < hk < hmax:
            occ[k] = 1
    _fill_holes(occ, pts, gw, gh)

    parts: list[Part] = []
    rejected: list[Part] = []
    T_cb = T.inverse()
    seen = bytearray(gw * gh)
    for start in range(gw * gh):
        if not occ[start] or seen[start]:
            continue
        blob = _flood(occ, seen, start, gw, gh)
        part = _measure(
            blob, pts, hs, zc, gw, gh, stride, surf, T_cb, K, fine=(w, h, depth, depth_scale_m, T)
        )
        if part is None:
            continue
        part.why = _why_not(part, spec, surf, reach, level_ok)
        if part.why is None and fingers is not None:
            part.why = _fingers(part, pts, fingers)
        (parts if part.why is None else rejected).append(part)
    if order:
        order_parts(parts, T, surf, order)
    return Scene(parts, rejected, surf, stride, notes)


def _surface(
    valid: list[Vec3], taught: Surface | None, level_ok: bool, spec: PartSpec | None, notes: list[str]
) -> Surface | None:
    if taught is not None:
        near = sorted(taught.height(p) for p in valid if abs(taught.height(p)) < 2 * LIVE_NUDGE_M)
        if len(near) < 30:
            notes.append("the taught plane is not in view: using it as taught")
            return taught
        dh = near[len(near) // 2]
        if abs(dh) > LIVE_NUDGE_M:
            notes.append(f"the table is {dh * 1000:+.0f} mm off the taught plane: re-teach it")
            dh = math.copysign(LIVE_NUDGE_M, dh)
        return taught.shifted(dh)
    if level_ok:
        return _level_surface(valid, spec)
    return _ransac_surface(valid)


def _level_surface(valid: list[Vec3], spec: PartSpec | None) -> Surface | None:
    """The table, level in base: the most populated 2 mm height band — among the heights
    at least a part's height below the highest band, when parts might cover most of it."""
    bins: dict[int, int] = {}
    for p in valid:
        b = int(math.floor(p[2] / 0.002))
        bins[b] = bins.get(b, 0) + 1
    if not bins:
        return None
    # smooth over three bins so a table's noise that straddles a boundary still wins
    score = {b: bins.get(b - 1, 0) + bins[b] + bins.get(b + 1, 0) for b in bins}
    best = max(score, key=lambda b: (score[b], -b))
    if spec is not None and spec.height_m is not None:
        # parts' tops can outnumber the table in a crowded view: prefer the lower of two
        # strong bands one part-height apart
        low = best - round(spec.height_m / 0.002)
        cand = [b for b in score if abs(b - low) <= 2]
        if cand:
            lb = max(cand, key=lambda b: score[b])
            if score[lb] >= 0.5 * score[best]:
                best = lb
    zs = sorted(p[2] for p in valid if abs(p[2] - (best + 0.5) * 0.002) <= SURFACE_BAND_M)
    if len(zs) < 30:
        return None
    return Surface.level(zs[len(zs) // 2])


def _ransac_surface(valid: list[Vec3]) -> Surface | None:
    rng = random.Random(0)  # deterministic: the same frame, the same plane
    sample = valid if len(valid) <= 3000 else rng.sample(valid, 3000)
    best: tuple[int, Vec3, float] | None = None
    for _ in range(80):
        a, b, c = rng.sample(sample, 3)
        n = _cross(_sub(b, a), _sub(c, a))
        nn = math.sqrt(_dot(n, n))
        if nn < 1e-9:
            continue
        n = (n[0] / nn, n[1] / nn, n[2] / nn)
        d = _dot(n, a)
        count = sum(1 for p in sample if abs(_dot(n, p) - d) < SURFACE_BAND_M)
        if best is None or count > best[0]:
            best = (count, n, d)
    if best is None or best[0] < 0.2 * len(sample):
        return None
    _, n, d = best
    if n[2] > 0:  # camera frame: +Z is away from the camera; up off the table is toward it
        n, d = (-n[0], -n[1], -n[2]), -d
    offs = sorted(_dot(n, p) - d for p in valid if abs(_dot(n, p) - d) < SURFACE_BAND_M)
    d += offs[len(offs) // 2] if offs else 0.0
    o = (n[0] * d, n[1] * d, n[2] * d)
    helper = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    x = _unit(_sub(helper, tuple(_dot(helper, n) * c for c in n)))
    y = _cross(n, x)
    return Surface(o, x, y, n, None, "fitted")


def _fill_holes(occ: bytearray, pts: list, gw: int, gh: int) -> None:
    """A cell with no depth and ≥ 5 occupied neighbours is a hole in a part's top."""
    fill = []
    for j in range(1, gh - 1):
        for i in range(1, gw - 1):
            k = j * gw + i
            if pts[k] is not None or occ[k]:
                continue
            n = 0
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    if (di or dj) and occ[k + dj * gw + di]:
                        n += 1
            if n >= 5:
                fill.append(k)
    for k in fill:
        occ[k] = 2  # occupied, but no point of its own


def _flood(occ: bytearray, seen: bytearray, start: int, gw: int, gh: int) -> list[int]:
    out, stack = [], [start]
    seen[start] = 1
    while stack:
        k = stack.pop()
        out.append(k)
        j, i = divmod(k, gw)
        for dj in (-1, 0, 1):
            jj = j + dj
            if not 0 <= jj < gh:
                continue
            for di in (-1, 0, 1):
                ii = i + di
                if not 0 <= ii < gw:
                    continue
                kk = jj * gw + ii
                if occ[kk] and not seen[kk]:
                    seen[kk] = 1
                    stack.append(kk)
    return out


def _measure(
    blob: list[int],
    pts: list,
    hs: list,
    zc: list[float],
    gw: int,
    gh: int,
    stride: int,
    surf: Surface,
    T_cb: Transform,
    K: dict,
    fine: tuple | None = None,
) -> Part | None:
    heights = sorted(hs[k] for k in blob if hs[k] is not None)
    if len(heights) < MIN_TOP_CELLS:
        return None
    top = heights[int(0.9 * (len(heights) - 1))]
    face = {k for k in blob if hs[k] is not None and hs[k] >= top - TOP_BAND_M}
    # a lone cell of the band (a flying pixel on the side) would stretch the rectangle
    face = {k for k in face if _neighbours(k, face, gw) >= 2}
    if len(face) < MIN_TOP_CELLS:
        return None
    near_edge = False
    for k in blob:
        j, i = divmod(k, gw)
        if i < EDGE_CELLS or j < EDGE_CELLS or i >= gw - EDGE_CELLS or j >= gh - EDGE_CELLS:
            near_edge = True
            break
    uv = [surf.local(pts[k]) for k in face]
    zmed = sorted(zc[k] for k in face)[len(face) // 2]
    step = stride
    hface = sorted(hs[k] for k in face)
    height = hface[len(hface) // 2]
    if fine is not None and stride > 1:
        # the coarse cells found the part; its edges come from every pixel of its top face — a
        # 60 mm part spans ~17 cells of 3 mm, and a rectangle through so few quantises its angle
        # by a few degrees at some wrist headings (hypothesis found 4.1°)
        got = _fine_face(face, gw, stride, surf, K, fine, top)
        if len(got) >= 4 * len(face):
            uv, step = got, 1
    cu, cv, ang, length, width = min_area_rect(uv)
    pad = 0.5 * step * zmed / K["fx"]  # the outermost samples sit ~half a sample inside the edges
    length, width = length + pad, width + pad
    centre = surf.point(cu, cv, height)
    ax = tuple(math.cos(ang) * surf.x_axis[i] + math.sin(ang) * surf.y_axis[i] for i in range(3))
    theta = math.atan2(ax[1], ax[0])
    corners = []
    ca, sa = math.cos(ang), math.sin(ang)
    for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        du, dv = su * length / 2, sv * width / 2
        corners.append(surf.point(cu + du * ca - dv * sa, cv + du * sa + dv * ca, height))
    return Part(
        centre=centre,
        theta=_wrap_half(theta),
        length_m=length,
        width_m=width,
        height_m=height,
        corners=corners,
        pixel=_project(T_cb, K, centre),
        corners_px=[_project(T_cb, K, c) for c in corners],
        cells=len(face),
        near_edge=near_edge,
    )


def _fine_face(
    face: set[int], gw: int, stride: int, surf: Surface, K: dict, fine: tuple, top: float
) -> list[tuple[float, float]]:
    """The top face at full resolution: every pixel whose coarse cell is on the face (or next to
    it, for the edges) and whose height is within :data:`TOP_BAND_M` of the top — in the
    surface's plane coordinates."""
    w, h, depth, scale, T = fine
    fx, fy, ppx, ppy = K["fx"], K["fy"], K["ppx"], K["ppy"]
    near = set()
    for k in face:
        for d in (-gw - 1, -gw, -gw + 1, -1, 0, 1, gw - 1, gw, gw + 1):
            near.add(k + d)
    out = []
    half = stride // 2
    for k in near:
        j, i = divmod(k, gw)
        for y in range(max(0, j * stride - half), min(h, j * stride - half + stride)):
            row = y * w
            for x in range(max(0, i * stride - half), min(w, i * stride - half + stride)):
                d = depth[2 * (row + x)] | (depth[2 * (row + x) + 1] << 8)
                if not d:
                    continue
                z = d * scale
                p = T.apply(((x - ppx) * z / fx, (y - ppy) * z / fy, z))
                if abs(surf.height(p) - top) <= TOP_BAND_M:
                    out.append(surf.local(p))
    return out


def _why_not(
    part: Part, spec: PartSpec | None, surf: Surface, reach: Reach | None, level_ok: bool
) -> str | None:
    if part.near_edge:
        return "cut off by the edge of the picture"
    if spec is not None:
        why = spec.why_not(part.length_m, part.width_m, part.height_m)
        if why is not None:
            return why
    elif part.length_m > 0.07 or part.width_m > 0.06 or part.width_m < 0.010:
        return "not a block"  # no part size: the foam blocks' gate, as before
    if not surf.inside(part.centre):
        return "outside the pick area"
    if reach is not None and level_ok:
        return reach.why_not(part.centre)
    return None


def _fingers(part: Part, pts: list, fingers: dict) -> str | None:
    from .pickplan import clearance

    r = part.length_m + 0.08
    near = [p for p in pts if p is not None and math.dist(p[:2], part.centre[:2]) < r]
    rect = {"centre": list(part.centre), "theta": part.theta, "minor_m": part.width_m}
    got = clearance(rect, near, **fingers)
    if got["clear"]:
        return None
    return f"no room for a finger beside it ({got['worst_mm']:.0f} mm too high)"


# -- pick order -----------------------------------------------------------------------------


def order_parts(
    parts: list[Part], T_bc: Transform, surf: Surface, order: tuple[str, str] = ("LR", "FB")
) -> list[Part]:
    """Number ``parts`` in place (``order`` 1…) and sort them: ``order[0]`` within a row,
    ``order[1]`` from row to row — each one of :data:`ORDERS`, one horizontal and one
    vertical *as the picture shows them* (Front = the bottom of the picture). A row is the
    parts whose across-row positions are within 60 % of the narrowest part's width."""
    first, rows = order
    if first not in ORDERS or rows not in ORDERS or _AXIS[first] == _AXIS[rows]:
        raise ValueError(f"pick order must be one horizontal and one vertical of {ORDERS}, got {order}")
    right = _flat(T_bc.rotate((1.0, 0.0, 0.0)), surf.normal)
    down = _flat(T_bc.rotate((0.0, 1.0, 0.0)), surf.normal)

    def coord(p: Part, key: str) -> float:
        c = p.centre
        return {
            "LR": _dot(c, right),
            "RL": -_dot(c, right),
            "FB": _dot(c, down) * -1.0,  # front = the bottom of the picture = +down first
            "BF": _dot(c, down),
        }[key]

    if not parts:
        return parts
    tol = max(0.02, 0.6 * min(p.width_m for p in parts))
    by_row = sorted(parts, key=lambda p: (coord(p, rows), coord(p, first)))
    grouped: list[list[Part]] = []
    for p in by_row:
        if grouped and coord(p, rows) - coord(grouped[-1][0], rows) <= tol:
            grouped[-1].append(p)
        else:
            grouped.append([p])
    ordered = [p for row in grouped for p in sorted(row, key=lambda q: coord(q, first))]
    for n, p in enumerate(ordered, 1):
        p.order = n
    parts[:] = ordered
    return parts


def parse_order(text: str) -> tuple[str, str]:
    """``LR,FB`` → ``("LR", "FB")``; ValueError otherwise."""
    bits = [b.strip().upper() for b in text.split(",")]
    if len(bits) != 2 or not all(b in ORDERS for b in bits) or _AXIS[bits[0]] == _AXIS[bits[1]]:
        raise ValueError("order must be one horizontal and one vertical, like order=LR,FB")
    return bits[0], bits[1]


# -- geometry -------------------------------------------------------------------------------


def min_area_rect(points: Sequence[tuple[float, float]]) -> tuple[float, float, float, float, float]:
    """The smallest-area rectangle around 2-D ``points`` (rotating calipers over the convex
    hull): ``(cx, cy, angle of the long side, length, width)``, length ≥ width, the angle
    in (-π/2, π/2]."""
    hull = convex_hull(points)
    if len(hull) == 1:
        return hull[0][0], hull[0][1], 0.0, 0.0, 0.0
    if len(hull) == 2:
        (x0, y0), (x1, y1) = hull
        return (
            (x0 + x1) / 2,
            (y0 + y1) / 2,
            _wrap_half(math.atan2(y1 - y0, x1 - x0)),
            math.dist(hull[0], hull[1]),
            0.0,
        )
    best = None
    n = len(hull)
    for k in range(n):
        x0, y0 = hull[k]
        x1, y1 = hull[(k + 1) % n]
        a = math.atan2(y1 - y0, x1 - x0)
        c, s = math.cos(a), math.sin(a)
        us = [x * c + y * s for x, y in hull]
        vs = [-x * s + y * c for x, y in hull]
        area = (max(us) - min(us)) * (max(vs) - min(vs))
        if best is None or area < best[0] - 1e-12:
            best = (area, a, min(us), max(us), min(vs), max(vs))
    _, a, u0, u1, v0, v1 = best  # type: ignore[misc]
    c, s = math.cos(a), math.sin(a)
    um, vm = (u0 + u1) / 2, (v0 + v1) / 2
    cx, cy = um * c - vm * s, um * s + vm * c
    lu, lv = u1 - u0, v1 - v0
    if lv > lu:
        a, lu, lv = a + math.pi / 2, lv, lu
    return cx, cy, _wrap_half(a), lu, lv


def convex_hull(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """Andrew's monotone chain; counter-clockwise, no collinear points."""
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _neighbours(k: int, cells: set[int], gw: int) -> int:
    n = 0
    for d in (-gw - 1, -gw, -gw + 1, -1, 1, gw - 1, gw, gw + 1):
        if k + d in cells:
            n += 1
    return n


def _project(T_cb: Transform, K: dict, p: Sequence[float]) -> tuple[int, int]:
    x, y, z = T_cb.apply(p)
    if z <= 1e-6:
        return (-1, -1)
    return round(K["fx"] * x / z + K["ppx"]), round(K["fy"] * y / z + K["ppy"])


def _flat(d: Sequence[float], n: Sequence[float]) -> Vec3:
    k = _dot(d, n)
    return _unit((d[0] - k * n[0], d[1] - k * n[1], d[2] - k * n[2]))


def _wrap_half(a: float) -> float:
    """An axis direction: (-π/2, π/2]."""
    while a <= -math.pi / 2:
        a += math.pi
    while a > math.pi / 2:
        a -= math.pi
    return a


def _v(p: Sequence[float]) -> Vec3:
    return (float(p[0]), float(p[1]), float(p[2]))


def _sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a: Sequence[float]) -> Vec3:
    n = math.sqrt(_dot(a, a))
    if n < 1e-12:
        raise ValueError("degenerate direction (points coincide or are collinear)")
    return (a[0] / n, a[1] / n, a[2] / n)
