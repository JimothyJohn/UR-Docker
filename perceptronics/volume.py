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
TOP_BAND_M = 0.004  # a flat top's own depth noise at working range, heights off the fitted plane
MAX_TILT_DEG = 8.0  # a fitted table further off level than this is not the table: level it is
TILT_NOTE_DEG = 1.0  # the table reads this far off level: the hand-eye calibration is out
PLANE_WINDOWS_M = (0.025, 0.012, 0.006, 0.003)
FINE_EDGE_PX = 0.75  # the eroded fine face's outermost pixel centres sit this far inside the edge
MAX_LEVELS = 6  # a blob is peeled at most this many heights deep
LEDGE_RING_M = 0.012  # the ring round a top that must not stand higher than it ...
LEDGE_FRAC = 0.2  # ... over this much of it (a ledge, not a part)
SHOULDER_FRAC = 0.35  # a top ringed this much by heights between it and the table: a dome
SHOULDER_LOW = 0.35  # "between": over this fraction of the top's height
LEDGE_SEARCH_CELLS = 10  # grid cells round a face searched for its ring (≥ LEDGE_RING_M at any stride)
NOISE_K = 3.5  # nothing stands off the surface by less than this many of its spreads
LOCAL_TILE_M = 0.15  # the local floor's tiles: at least this, and three part lengths
FLOOR_WINDOW_M = 0.015  # the local floor's first pass: heights this near the fitted surface
MAX_PICK_RANGE_M = 1.6  # past this a D435 can't measure a part (its edges blur by ~2 cm): never picked
RANGE_SLACK = 0.012  # sizes measure no better than this × the range (the D435's blur: 12-16 mm at 1.4 m)
BLUR_PER_M = 0.008  # an edge blurs over about this × the range
RECT_TRIM = 0.02  # a footprint's extents: its 2-98 % quantiles, scaled to the full width
RAMP_EPS_M = 0.0015  # a cell this much lower than its neighbour toward the camera is on a far-side ramp
FOOTPRINT_MARGIN_M = 0.005  # a part owns the cells this close round its top's footprint: its sides
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
    ``why`` when it is not going to be picked — and ``near`` when it is still nearly the
    part (the right size but out of reach, or a little off the size): what the pendant
    draws, so the operator sees the trouble and not every other thing on the table."""

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
    near: bool = True
    near_edge: bool = field(default=False, repr=False)
    rect_uv: tuple = field(default=(0.0, 0.0, 0.0, 0.0, 0.0), repr=False)  # in the surface's plane
    range_m: float = field(default=0.0, repr=False)  # camera to its top: what its tolerances grow with
    margin_m: float = field(
        default=FOOTPRINT_MARGIN_M, repr=False
    )  # its edge's blur: cells this near are its

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
            "near": self.near,
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
    """Anything standing this far off the surface is *something*: half the part's lowest
    height on any face (never under :data:`OCCUPIED_FLOOR_M`), 8 mm without a height."""
    if spec is not None and spec.heights():
        low = spec.heights()[0]
        return max(OCCUPIED_FLOOR_M, 0.5 * low - spec.slack(low))
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
    fingers' room is checked when given, and the parts are then put in the order that
    **clears the way** (:func:`clear_the_way`): a part pinned only by other parts is picked
    after them instead of being turned away. ``order`` breaks the ties."""
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

    hmax = 0.5 if spec is None or not spec.heights() else 2.5 * spec.heights()[-1] + 0.02
    hs: list[float | None] = [None if p is None else surf.height(p) for p in pts]
    uvs = [None if p is None else surf.local(p) for p in pts]
    # the floor near each cell: the surface is never one plane over a whole view (a carpet's
    # undulation, the D435's swells growing with z²) — 5-7 mm of spread over 3 m of carpet, 1.7 mm
    # over 30 cm (2026-10-03); and its own roughness there, below which nothing is told from it
    offs, sig = _local_floor(hs, uvs, spec)
    hs = [None if hk is None else hk - offs[k] for k, hk in enumerate(hs)]
    base = occupied_min_m(spec)
    thr = [max(base, NOISE_K * sg) for sg in sig]
    rough = sorted(sig[k] for k in range(len(hs)) if hs[k] is not None and NOISE_K * sig[k] > base)
    if spec is not None and spec.heights() and len(rough) > 0.25 * len(valid):
        sg = rough[len(rough) // 2]
        notes.append(
            f"the surface reads rough (±{sg * 1000:.1f} mm) over much of the picture: nothing under "
            f"{NOISE_K * sg * 1000:.0f} mm can be told from it there — move the camera closer"
        )
    occ = bytearray(gw * gh)
    for k, hk in enumerate(hs):
        if hk is not None and thr[k] < hk < hmax:
            occ[k] = 1
    _fill_holes(occ, pts, gw, gh)

    parts: list[Part] = []
    rejected: list[Part] = []
    blobs: dict[int, list[int]] = {}  # id(part) → its cells, for who-pins-whom
    ctx = _Ctx(
        pts,
        hs,
        uvs,
        zc,
        thr,
        gw,
        gh,
        stride,
        surf,
        T,
        T.inverse(),
        K,
        (w, h, depth, depth_scale_m),
        offs,
        spec,
    )
    seen = bytearray(gw * gh)
    for start in range(gw * gh):
        if not occ[start] or seen[start]:
            continue
        blob = _flood(occ, seen, start, gw, gh)
        for part, own in _parts_in(blob, ctx, [], MAX_LEVELS):
            part.why = _why_not(part, spec, surf, reach, level_ok)
            part.near = _near(part, spec)
            (parts if part.why is None else rejected).append(part)
            blobs[id(part)] = own
    if order:
        order_parts(parts, T, surf, order)
    if fingers is not None and parts:
        for part in clear_the_way(parts, pts, blobs, fingers):
            part.near = _near(part, spec)
            rejected.append(part)
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
        surf = _level_surface(valid, spec)
        if surf is not None and surf.tilt_deg() > TILT_NOTE_DEG:
            notes.append(
                f"the table reads {surf.tilt_deg():.1f}° off level: the hand-eye calibration is out "
                "(parts are still measured off the table as seen; re-run perceptronics calibrate)"
            )
        return surf
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
    if spec is not None:
        # parts' tops can outnumber the table in a crowded view: prefer the lower of two
        # strong bands one part-height (on any face) apart
        for hgt in spec.heights():
            low = best - round(hgt / 0.002)
            cand = [b for b in score if abs(b - low) <= 2]
            if cand:
                lb = max(cand, key=lambda b: score[b])
                if score[lb] >= 0.5 * score[best]:
                    best = lb
                    break
    zs = sorted(p[2] for p in valid if abs(p[2] - (best + 0.5) * 0.002) <= SURFACE_BAND_M)
    if len(zs) < 30:
        return None
    z0 = zs[len(zs) // 2]
    # The table is level, but the hand-eye that puts the picture in the base frame never quite
    # is: 1.5° of error tilts a 0.4 m view by 10 mm, enough to lift half the table over a low
    # part's occupied height (the bench: scripts/volume_bench.py). So the band only says where
    # the table is; the surface is the plane through it, allowed MAX_TILT_DEG off level.
    plane = _fit_plane_near(valid, z0)
    if plane is None:
        return Surface.level(z0)
    a, b, c, mx, my = plane
    if math.degrees(math.atan(math.hypot(a, b))) > MAX_TILT_DEG:
        return Surface.level(z0)
    n = _unit((-a, -b, 1.0))
    x = _unit((1.0, 0.0, a))
    return Surface((mx, my, a * mx + b * my + c), x, _cross(n, x), n)


def _fit_plane_near(valid: list[Vec3], z0: float) -> tuple[float, float, float, float, float] | None:
    """``z = a x + b y + c`` through the points near height ``z0``, by least squares on a
    window that narrows around the last fit (:data:`PLANE_WINDOWS_M`): the first pass sees a
    tilted table's whole band, the later ones drop the parts standing on it. Also the
    inliers' centroid ``(mx, my)``. None when the points don't pin a plane down."""
    step = max(1, len(valid) // 6000)
    pts = valid[::step]
    fit = (0.0, 0.0, z0)
    mx = my = 0.0
    for win in PLANE_WINDOWS_M:
        a, b, c = fit
        sel = [p for p in pts if abs(p[2] - (a * p[0] + b * p[1] + c)) < win]
        if len(sel) < 30:
            return None
        n = len(sel)
        mx, my, mz = sum(p[0] for p in sel) / n, sum(p[1] for p in sel) / n, sum(p[2] for p in sel) / n
        sxx = sxy = syy = sxz = syz = 0.0
        for p in sel:
            dx, dy, dz = p[0] - mx, p[1] - my, p[2] - mz
            sxx += dx * dx
            sxy += dx * dy
            syy += dy * dy
            sxz += dx * dz
            syz += dy * dz
        det = sxx * syy - sxy * sxy
        if det <= 1e-12 * max(1e-12, sxx * syy):
            return None
        a = (sxz * syy - syz * sxy) / det
        b = (syz * sxx - sxz * sxy) / det
        fit = (a, b, mz - a * mx - b * my)
    return (*fit, mx, my)


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


@dataclass
class _Ctx:
    """One frame's sampled grid, as the measurement needs it."""

    pts: list
    hs: list  # height off the local floor
    uvs: list  # in the surface's plane
    zc: list
    thr: list  # occupied above this, per cell
    gw: int
    gh: int
    stride: int
    surf: Surface
    T: Transform  # camera → base
    T_cb: Transform
    K: dict
    fine: tuple  # (w, h, depth, scale)
    offs: list  # the local floor's offset, per cell
    spec: PartSpec | None


def _local_floor(hs: list, uvs: list, spec: PartSpec | None) -> tuple[list[float], list[float]]:
    """The floor's height off the fitted surface near every cell, and its spread there: the
    median and MAD of the near-floor heights in square tiles of the surface (three part lengths,
    never under :data:`LOCAL_TILE_M` — a part can't outvote the floor round it), each smoothly
    interpolated between tile centres. Two passes, the second on what the first left."""
    n = len(hs)
    offs, sig = [0.0] * n, [0.0] * n
    cells = [k for k in range(n) if hs[k] is not None]
    if len(cells) < 50:
        return offs, sig
    longest = max([spec.length_m, *spec.heights()]) if spec is not None else 0.07
    S = max(LOCAL_TILE_M, 3.0 * longest)
    u0 = min(uvs[k][0] for k in cells)
    v0 = min(uvs[k][1] for k in cells)
    nx = int((max(uvs[k][0] for k in cells) - u0) / S) + 1
    ny = int((max(uvs[k][1] for k in cells) - v0) / S) + 1
    step = max(1, len(cells) // 40000)
    window = [FLOOR_WINDOW_M] * n
    for _ in range(2):
        bins: list[list[float]] = [[] for _ in range(nx * ny)]
        for k in cells[::step]:
            r = hs[k] - offs[k]
            if abs(r) < window[k]:
                t = min(ny - 1, int((uvs[k][1] - v0) / S)) * nx + min(nx - 1, int((uvs[k][0] - u0) / S))
                bins[t].append(hs[k])
        med: list[float | None] = [None] * (nx * ny)
        mad: list[float | None] = [None] * (nx * ny)
        for t, b in enumerate(bins):
            if len(b) >= 20:
                b.sort()
                m = b[len(b) // 2]
                med[t] = m
                d = sorted(abs(x - m) for x in b)
                mad[t] = 1.4826 * d[len(d) // 2]
        _fill_tiles(med, nx, ny)
        _fill_tiles(mad, nx, ny)
        for k in cells:
            fx = min(max((uvs[k][0] - u0) / S - 0.5, 0.0), nx - 1.0)
            fy = min(max((uvs[k][1] - v0) / S - 0.5, 0.0), ny - 1.0)
            x0, y0 = int(fx), int(fy)
            a, b = y0 * nx + x0, y0 * nx + min(nx - 1, x0 + 1)
            cc, dd = min(ny - 1, y0 + 1) * nx + x0, min(ny - 1, y0 + 1) * nx + min(nx - 1, x0 + 1)
            ax, ay = fx - x0, fy - y0
            wa, wb, wc, wd = (1 - ax) * (1 - ay), ax * (1 - ay), (1 - ax) * ay, ax * ay
            offs[k] = med[a] * wa + med[b] * wb + med[cc] * wc + med[dd] * wd
            sg = mad[a] * wa + mad[b] * wb + mad[cc] * wc + mad[dd] * wd
            sig[k] = sg
            window[k] = max(SURFACE_BAND_M, 3.5 * sg)
    return offs, sig


def _fill_tiles(vals: list, nx: int, ny: int) -> None:
    """Tiles with no floor of their own (all part, or no depth) take their neighbours'."""
    if all(v is None for v in vals):
        vals[:] = [0.0] * len(vals)
        return
    while any(v is None for v in vals):
        nxt = list(vals)
        for t, v in enumerate(vals):
            if v is not None:
                continue
            y, x = divmod(t, nx)
            got = [
                vals[yy * nx + xx]
                for yy in (y - 1, y, y + 1)
                for xx in (x - 1, x, x + 1)
                if 0 <= yy < ny and 0 <= xx < nx and vals[yy * nx + xx] is not None
            ]
            if got:
                nxt[t] = sum(got) / len(got)
        vals[:] = nxt


def _pose_height(top: float, spec: PartSpec | None) -> float:
    """The height the part should stand at: the spec's (on any face) nearest the measured top,
    else the top itself."""
    hs = spec.heights() if spec is not None else []
    return min(hs, key=lambda h: abs(h - top)) if hs else top


def _parts_in(blob: list[int], c: _Ctx, found: list[Part], depth: int) -> list[tuple[Part, list[int]]]:
    """Every part in one blob of occupied cells. The footprint is where the blob stands over
    **half the part's height** (Nick, 2026-10-03: "drawing a line halfway up the expected part
    height") — a blurred step's half-height line is its edge, and a white label or a bright patch
    on the top can't split it the way a band at the top did. The heights a far side reads past
    the edge (the floor the part hides, filled in by the stereo) are slid back toward the camera
    onto it (:func:`_unramp`). Footprints that are apart on the surface are separate parts, even
    when the picture joins them (two tall parts' sides). What is left — a lower part beside a
    taller one — is looked at again, ``depth`` levels deep. Each part comes with its cells."""
    hts = sorted(c.hs[k] for k in blob if c.hs[k] is not None)
    if len(hts) < MIN_TOP_CELLS:
        return []
    top = hts[int(0.85 * (len(hts) - 1))]
    half = max(0.5 * _pose_height(top, c.spec), min(c.thr[k] for k in blob))
    away, step = _away(blob, c)
    uv = {k: _unramp(k, c, top, away, step) for k in blob if c.hs[k] is not None}
    foot = {k: uv[k] for k in uv if c.hs[k] >= half}
    level: list[tuple[Part, list[int]]] = []
    for comp in _surface_components(foot, c):
        if len(comp) < MIN_TOP_CELLS:
            continue
        part = _measure(comp, c, half, top, away, step)
        if part is None:
            continue
        beside = [q for q in found + [p for p, _ in level] if _is_the_part(q, c.spec)]
        if _not_a_top(part, comp, c, beside):
            continue
        level.append((part, []))
    left = []
    for k in blob:
        q = uv.get(k) or c.uvs[k]
        owner = next((own for part, own in level if q is not None and _owns(part, q, away, c)), None)
        if owner is not None:
            owner.append(k)
        elif c.hs[k] is not None and c.hs[k] < half:
            left.append(k)
    out = list(level)
    if depth > 1 and left and level:
        for sub in _grid_components(left, c.gw):
            if len(sub) >= MIN_TOP_CELLS and max(c.hs[k] for k in sub) > 2 * min(c.thr[k] for k in sub):
                out.extend(_parts_in(sub, c, found + [p for p, _ in out], depth - 1))
    return out


def _away(blob: list[int], c: _Ctx) -> tuple[tuple[float, float], tuple[int, int]]:
    """In the surface's plane, the unit direction away from the camera at the blob; and the
    grid step (cells) that goes from a cell toward the camera in the picture."""
    k = blob[len(blob) // 2]
    p = c.pts[k]
    if p is None:
        p = next(c.pts[q] for q in blob if c.pts[q] is not None)
    n, cam = c.surf.normal, c.T.translation
    d = _sub(p, cam)
    d = _sub(d, tuple(_dot(d, n) * x for x in n))
    if _dot(d, d) < 1e-12:  # looking straight down at it: nothing hides behind it
        return (0.0, 0.0), (0, 0)
    d = _unit(d)
    a = (_dot(d, c.surf.x_axis), _dot(d, c.surf.y_axis))
    px0 = _project(c.T_cb, c.K, p)
    px1 = _project(c.T_cb, c.K, tuple(p[i] - 0.02 * d[i] for i in range(3)))
    gx, gy = px1[0] - px0[0], px1[1] - px0[1]
    g = math.hypot(gx, gy) or 1.0
    return a, (round(2 * gx / g), round(2 * gy / g))


def _tan_off_normal(p: Sequence[float], c: _Ctx) -> float:
    r = _sub(p, c.T.translation)
    rn = math.sqrt(_dot(r, r)) or 1.0
    cos = min(1.0, abs(_dot(r, c.surf.normal)) / rn)
    return math.sqrt(max(0.0, 1 - cos * cos)) / max(cos, 0.05)


def _unramp(
    k: int, c: _Ctx, top: float, away: tuple[float, float], step: tuple[int, int]
) -> tuple[float, float]:
    """Cell ``k``'s place on the surface, a far-side ramp undone: behind a part the camera can't
    see ``top·tanθ`` of floor, and the stereo fills it with a slope down from the top's far edge.
    A cell lower than its neighbour toward the camera is on such a slope: it belongs ``(top −
    h)·tanθ`` nearer, on the edge."""
    u, v = c.uvs[k]
    if not step[0] and not step[1]:
        return u, v
    j, i = divmod(k, c.gw)
    jj, ii = j + step[1], i + step[0]
    if not (0 <= jj < c.gh and 0 <= ii < c.gw):
        return u, v
    h, hn = c.hs[k], c.hs[jj * c.gw + ii]
    if hn is None or hn <= h + RAMP_EPS_M or h >= top:
        return u, v
    d = (top - h) * _tan_off_normal(c.pts[k], c)
    return u - d * away[0], v - d * away[1]


def _surface_components(foot: dict[int, tuple[float, float]], c: _Ctx) -> list[set[int]]:
    """The footprint cells grouped by where they stand on the surface (8-connected on a raster a
    little coarser than the cells), not by where the picture puts them."""
    if not foot:
        return []
    ks = list(foot)
    z = sorted(c.zc[k] for k in ks)[len(ks) // 2]
    pitch = max(0.002, 1.6 * c.stride * z / c.K["fx"] * (1 + _tan_off_normal(c.pts[ks[0]], c) * 0.5))
    raster: dict[tuple[int, int], list[int]] = {}
    for k, (u, v) in foot.items():
        raster.setdefault((math.floor(u / pitch), math.floor(v / pitch)), []).append(k)
    out, seen = [], set()
    for start in raster:
        if start in seen:
            continue
        comp, stack = set(), [start]
        seen.add(start)
        while stack:
            key = stack.pop()
            comp.update(raster[key])
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    nb = (key[0] + du, key[1] + dv)
                    if nb in raster and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        out.append(comp)
    return out


def _grid_components(cells: list[int], gw: int) -> list[list[int]]:
    have = set(cells)
    out, seen = [], set()
    for start in cells:
        if start in seen:
            continue
        comp, stack = [], [start]
        seen.add(start)
        while stack:
            k = stack.pop()
            comp.append(k)
            j, i = divmod(k, gw)
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    if not 0 <= i + di < gw:
                        continue
                    kk = (j + dj) * gw + i + di
                    if kk in have and kk not in seen:
                        seen.add(kk)
                        stack.append(kk)
        out.append(comp)
    return out


def _in_footprint(part: Part, uv: tuple[float, float], margin: float = FOOTPRINT_MARGIN_M) -> bool:
    cu, cv, ang, length, width = part.rect_uv
    du, dv = uv[0] - cu, uv[1] - cv
    co, si = math.cos(ang), math.sin(ang)
    return abs(du * co + dv * si) <= length / 2 + margin and abs(-du * si + dv * co) <= width / 2 + margin


def _owns(part: Part, uv: tuple[float, float], away: tuple[float, float], c: _Ctx) -> bool:
    """Is the cell at ``uv`` this part's: inside its footprint (a blur's width round it), or in
    its shadow — the floor it hides from the camera, ``height·tanθ`` behind it, which the
    stereo fills with whatever it likes."""
    if _in_footprint(part, uv, part.margin_m):
        return True
    if not away[0] and not away[1]:
        return False
    reach = part.height_m * _tan_off_normal(part.centre, c)
    for f in (0.25, 0.5, 0.75, 1.0):
        q = (uv[0] - f * reach * away[0], uv[1] - f * reach * away[1])
        if _in_footprint(part, q, part.margin_m):
            return True
    return False


def _is_the_part(part: Part, spec: PartSpec | None) -> bool:
    if spec is None:
        return not (part.length_m > 0.07 or part.width_m > 0.06 or part.width_m < 0.010)
    return spec.why_not(part.length_m, part.width_m, part.height_m, RANGE_SLACK * part.range_m) is None


def _not_a_top(part: Part, comp: set[int], c: _Ctx, found: list[Part]) -> bool:
    """Is this footprint something other than a part? A part drops to the floor all round, a
    blur's width past its half-height edge (``margin_m``). In the ring beyond that, out to
    :data:`LEDGE_RING_M` more:

    - higher ground (over :data:`LEDGE_FRAC` of it) makes it a **ledge** — a slice of something
      taller — except where it is a part already found (``found``; a lump's summit is higher
      ground, a part beside a part is not);
    - heights still well off the floor but under the half-height line (over
      :data:`SHOULDER_FRAC`) make it a **dome** — a lump's flank, not a part's edge.

    The real carpet frame, 2026-10-03: a 25-40 mm lump that otherwise yielded part-sized
    slices."""
    js = [k // c.gw for k in comp]
    is_ = [k % c.gw for k in comp]
    r = LEDGE_SEARCH_CELLS
    ring = higher = shoulder = 0
    top = part.height_m
    inner, outer = part.margin_m, part.margin_m + LEDGE_RING_M
    for j in range(max(0, min(js) - r), min(c.gh, max(js) + r + 1)):
        for i in range(max(0, min(is_) - r), min(c.gw, max(is_) + r + 1)):
            k = j * c.gw + i
            if c.hs[k] is None:
                continue
            uv = c.uvs[k]
            if not _in_footprint(part, uv, outer) or _in_footprint(part, uv, inner):
                continue
            if any(_in_footprint(q, uv, q.margin_m) for q in found):
                continue
            ring += 1
            h = c.hs[k]
            if h > top + max(TOP_BAND_M, 0.15 * top):
                higher += 1
            elif max(SHOULDER_LOW * top, c.thr[k]) < h < 0.5 * top:
                shoulder += 1
    return ring >= 8 and (higher >= LEDGE_FRAC * ring or shoulder >= SHOULDER_FRAC * ring)


def _spread(hs: list) -> float:
    """1.4826 × the median absolute height of the points near the surface (within 20 mm)."""
    near = [abs(h) for h in hs[:: max(1, len(hs) // 8000)] if h is not None and abs(h) < 0.02]
    if len(near) < 50:
        return 0.0
    near.sort()
    return 1.4826 * near[len(near) // 2]


def _measure(
    comp: set[int], c: _Ctx, half: float, top: float, away: tuple[float, float], step: tuple[int, int]
) -> Part | None:
    """One footprint (coarse cells) measured: its rectangle from every pixel over the half-height
    line at full resolution when there are enough of them (ramp undone, eroded one pixel), else
    from the cells; its height the top of what it holds."""
    gw, gh = c.gw, c.gh
    near_edge = any(
        i <= EDGE_CELLS or j <= EDGE_CELLS or i >= gw - 1 - EDGE_CELLS or j >= gh - 1 - EDGE_CELLS
        for j, i in (divmod(k, gw) for k in comp)
    )
    hts = sorted(c.hs[k] for k in comp)
    height = hts[int(0.8 * (len(hts) - 1))]
    zmed = sorted(c.zc[k] for k in comp)[len(comp) // 2]
    uv = [_unramp(k, c, top, away, step) for k in comp]
    pitch_step = c.stride
    if c.stride > 1:
        got = _fine_foot(comp, c, half, top, away, step)
        if len(got) >= 0.4 * c.stride * c.stride * len(comp):
            uv, pitch_step = got, 1
    cu, cv, ang, length, width = robust_rect(uv)
    pad = (2 * FINE_EDGE_PX if pitch_step == 1 else 0.5 * pitch_step) * zmed / c.K["fx"]
    length, width = length + pad, width + pad
    surf = c.surf
    # the centre at the part's top, on the local floor (its offset off the fitted surface)
    off = sorted(c.offs[k] for k in comp)[len(comp) // 2]
    centre = surf.point(cu, cv, off + height)
    ax = tuple(math.cos(ang) * surf.x_axis[i] + math.sin(ang) * surf.y_axis[i] for i in range(3))
    theta = math.atan2(ax[1], ax[0])
    corners = []
    ca, sa = math.cos(ang), math.sin(ang)
    for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        du, dv = su * length / 2, sv * width / 2
        corners.append(surf.point(cu + du * ca - dv * sa, cv + du * sa + dv * ca, off + height))
    return Part(
        centre=centre,
        theta=_wrap_half(theta),
        length_m=length,
        width_m=width,
        height_m=height,
        corners=corners,
        pixel=_project(c.T_cb, c.K, centre),
        corners_px=[_project(c.T_cb, c.K, q) for q in corners],
        cells=len(comp),
        near_edge=near_edge,
        rect_uv=(cu, cv, ang, length, width),
        range_m=zmed,
        margin_m=max(FOOTPRINT_MARGIN_M, BLUR_PER_M * zmed * 2),
    )


def _fine_foot(
    comp: set[int], c: _Ctx, half: float, top: float, away: tuple[float, float], step: tuple[int, int]
) -> list[tuple[float, float]]:
    """The footprint at full resolution: every pixel of the footprint's cells (and the ring
    next to them, for the edges) standing over the half-height line off its cell's local floor,
    its far-side ramp undone (:func:`_unramp`, pixel by pixel), kept when its four neighbours
    are too — a flying pixel off the side has one that isn't; the edge loses one pixel, which
    the caller adds back (:data:`FINE_EDGE_PX`)."""
    w, h, depth, scale = c.fine
    gw, stride = c.gw, c.stride
    fx, fy, ppx, ppy = c.K["fx"], c.K["fy"], c.K["ppx"], c.K["ppy"]
    surf, T = c.surf, c.T
    cells = set()
    for k in comp:
        for d in (-gw - 1, -gw, -gw + 1, -1, 0, 1, gw - 1, gw, gw + 1):
            cells.add(k + d)
    half_s = stride // 2
    # only this part's pixels: none much taller than it (a taller neighbour next to it in the
    # picture), none off its own patch of the surface
    ks = list(comp)
    cap = max(top, sorted(c.hs[k] for k in ks)[int(0.95 * (len(ks) - 1))]) + max(TOP_BAND_M, 0.25 * top)
    z = sorted(c.zc[k] for k in ks)[len(ks) // 2]
    pitch = max(0.002, 1.6 * stride * z / fx * (1 + _tan_off_normal(c.pts[ks[0]], c) * 0.5))
    mine = set()
    for k in ks:
        u0, v0 = _unramp(k, c, top, away, step)
        cu, cv = math.floor(u0 / pitch), math.floor(v0 / pitch)
        mine.update((cu + du, cv + dv) for du in (-1, 0, 1) for dv in (-1, 0, 1))
    hgt: dict[tuple[int, int], float] = {}
    pos: dict[tuple[int, int], tuple] = {}

    def at(x: int, y: int):
        key = (x, y)
        if key in hgt:
            return hgt[key]
        if not (0 <= x < w and 0 <= y < h):
            hgt[key] = None
            return None
        d = depth[2 * (y * w + x)] | (depth[2 * (y * w + x) + 1] << 8)
        if not d:
            hgt[key] = None
            return None
        z = d * scale
        p = T.apply(((x - ppx) * z / fx, (y - ppy) * z / fy, z))
        kk = min(c.gh - 1, (y + half_s) // stride) * gw + min(gw - 1, (x + half_s) // stride)
        hv = surf.height(p) - c.offs[kk]
        hgt[key], pos[key] = hv, p
        return hv

    band = {}
    sx, sy = step[0] * max(1, stride), step[1] * max(1, stride)
    for k in cells:
        if not 0 <= k < len(c.pts):
            continue
        j, i = divmod(k, gw)
        for y in range(max(0, j * stride - half_s), min(h, j * stride - half_s + stride)):
            for x in range(max(0, i * stride - half_s), min(w, i * stride - half_s + stride)):
                hv = at(x, y)
                if hv is None or hv < half or hv > cap:
                    continue
                u, v = surf.local(pos[(x, y)])
                if sx or sy:
                    hn = at(x + sx, y + sy)
                    if hn is not None and hn > hv + RAMP_EPS_M and hv < top:
                        dd = (top - hv) * _tan_off_normal(pos[(x, y)], c)
                        u, v = u - dd * away[0], v - dd * away[1]
                if (math.floor(u / pitch), math.floor(v / pitch)) in mine:
                    band[(x, y)] = (u, v)
    return [
        q
        for (x, y), q in band.items()
        if (x - 1, y) in band and (x + 1, y) in band and (x, y - 1) in band and (x, y + 1) in band
    ]


def _why_not(
    part: Part, spec: PartSpec | None, surf: Surface, reach: Reach | None, level_ok: bool
) -> str | None:
    if part.near_edge:
        return "cut off by the edge of the picture"
    if part.range_m > MAX_PICK_RANGE_M:
        return f"too far from the camera to measure ({part.range_m:.1f} m)"
    if spec is not None:
        why = spec.why_not(part.length_m, part.width_m, part.height_m, RANGE_SLACK * part.range_m)
        if why is not None:
            return why
    elif part.length_m > 0.07 or part.width_m > 0.06 or part.width_m < 0.010:
        return "not a block"  # no part size: the foam blocks' gate, as before
    if not surf.inside(part.centre):
        return "outside the pick area"
    if reach is not None and level_ok:
        return reach.why_not(part.centre)
    return None


def _near(part: Part, spec: PartSpec | None) -> bool:
    """Nearly the part? Without a size, everything found is; a piece cut off by the picture's
    edge is when what shows of it is no bigger than the part."""
    if part.range_m > MAX_PICK_RANGE_M:
        return False
    if spec is None or part.why is None:
        return True
    if part.near_edge:
        return part.length_m <= spec.length_m + spec.slack(spec.length_m, RANGE_SLACK * part.range_m) and (
            part.width_m <= spec.width_m + spec.slack(spec.width_m, RANGE_SLACK * part.range_m)
        )
    return spec.near_miss(part.length_m, part.width_m, part.height_m, RANGE_SLACK * part.range_m)


def _fingers(part: Part, pts: list, fingers: dict) -> dict:
    """:func:`perceptronics.pickplan.clearance` for ``part`` against ``pts`` (None entries
    skipped) — only the points near enough to matter."""
    from .pickplan import clearance

    fingers = dict(fingers)
    long_way = fingers.pop("across", "short") == "long"  # the fingers close across the long side
    r = part.length_m + 0.08 + (fingers.get("room_m") or 0.0)
    near = [p for p in pts if p is not None and math.dist(p[:2], part.centre[:2]) < r]
    rect = {"centre": list(part.centre), "theta": part.theta, "minor_m": part.width_m}
    if long_way:
        rect = {**rect, "theta": part.theta + math.pi / 2, "minor_m": part.length_m}
    return clearance(rect, near, **fingers)


def _no_room(got: dict) -> str:
    return f"no room for a finger beside it ({got['worst_mm']:.0f} mm too high)"


def clear_the_way(parts: list[Part], pts: list, blobs: dict[int, list[int]], fingers: dict) -> list[Part]:
    """Put ``parts`` (already in the picture's order) in the order that clears the way, and
    return the ones that can't be picked at all (``why`` set), taken out of ``parts``.

    Nick, 2026-10-02: pick the parts that make other parts easier to pick. A part whose
    finger zone is blocked *only by other parts* can go once they have gone; one blocked by
    anything else (a wall, a wrong-size thing — what never leaves) can't. Greedy: of the
    parts free now, take the one that frees the most others (the picture's order breaks
    ties), and repeat. Parts that pin each other with nothing free to start from stay out."""
    owner: dict[int, int] = {}
    for n, p in enumerate(parts):
        for cell in blobs.get(id(p), ()):
            owner[cell] = n
    background = [q for cell, q in enumerate(pts) if q is not None and cell not in owner]
    cells_of = [[pts[c] for c in blobs.get(id(p), ()) if pts[c] is not None] for p in parts]
    reach = [p.length_m + 0.08 + (fingers.get("room_m") or 0.0) for p in parts]
    # the background near each part, once: a full frame is tens of thousands of points
    around = [
        [q for q in background if math.dist(q[:2], p.centre[:2]) < reach[n]] for n, p in enumerate(parts)
    ]
    blockers: list[set[int] | None] = []  # None: pinned by something that never leaves
    for n, p in enumerate(parts):
        if not _fingers(p, around[n], fingers)["clear"]:
            blockers.append(None)
            continue
        near = [
            m
            for m, q in enumerate(parts)
            if m != n and math.dist(q.centre[:2], p.centre[:2]) < reach[n] + q.length_m
        ]
        if _fingers(p, around[n] + [c for m in near for c in cells_of[m]], fingers)["clear"]:
            blockers.append(set())
            continue
        mine = {m for m in near if not _fingers(p, around[n] + cells_of[m], fingers)["clear"]}
        blockers.append(mine or set(near))  # several together, none alone: all of them
    gone: set[int] = set()
    plan: list[int] = []
    left = [n for n in range(len(parts)) if blockers[n] is not None]

    def free(n: int, after: set[int]) -> bool:
        b = blockers[n]
        return b is not None and b <= after

    while True:
        now = [n for n in left if n not in gone and free(n, gone)]
        if not now:
            break
        pending = [n for n in left if n not in gone and not free(n, gone)]
        pick = max(now, key=lambda n: (sum(free(m, gone | {n}) for m in pending), -n))
        plan.append(pick)
        gone.add(pick)
    out = []
    for n, p in enumerate(parts):
        if n not in gone:
            others = [c for m, cs in enumerate(cells_of) if m != n for c in cs]
            p.why = _no_room(_fingers(p, around[n] + others, fingers))
            out.append(p)
    parts[:] = [parts[n] for n in plan]
    for k, p in enumerate(parts, 1):
        p.order = k
    return out


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


def robust_rect(
    points: Sequence[tuple[float, float]], trim: float = RECT_TRIM
) -> tuple[float, float, float, float, float]:
    """:func:`min_area_rect` that a few stray points can't stretch: the headings are the hull's
    edges, but each heading's extents are the ``trim``..``1 - trim`` quantiles of the points
    along it, scaled back to a full width as for points spread evenly over the rectangle — the
    real frames' footprints carry a few percent of noise and leftover ramp past their edges."""
    pts = list(points)
    if len(pts) < 40:
        return min_area_rect(pts)
    hull = convex_hull(pts)
    if len(hull) < 3:
        return min_area_rect(pts)
    lo_i = int(trim * (len(pts) - 1))
    hi_i = int((1 - trim) * (len(pts) - 1))
    scale = 1.0 / (1.0 - 2.0 * trim)
    sample = pts[:: max(1, len(pts) // 1500)]
    lo_s = int(trim * (len(sample) - 1))
    hi_s = int((1 - trim) * (len(sample) - 1))
    best = None
    for k in range(len(hull)):
        (x0, y0), (x1, y1) = hull[k], hull[(k + 1) % len(hull)]
        a = math.atan2(y1 - y0, x1 - x0)
        c, s = math.cos(a), math.sin(a)
        us = sorted(x * c + y * s for x, y in sample)
        vs = sorted(-x * s + y * c for x, y in sample)
        area = (us[hi_s] - us[lo_s]) * (vs[hi_s] - vs[lo_s])
        if best is None or area < best[0]:
            best = (area, a)
    a = best[1]  # type: ignore[index]
    c, s = math.cos(a), math.sin(a)
    us = sorted(x * c + y * s for x, y in pts)
    vs = sorted(-x * s + y * c for x, y in pts)
    u0, u1, v0, v1 = us[lo_i], us[hi_i], vs[lo_i], vs[hi_i]
    um, vm = (u0 + u1) / 2, (v0 + v1) / 2
    lu, lv = (u1 - u0) * scale, (v1 - v0) * scale
    cx, cy = um * c - vm * s, um * s + vm * c
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
