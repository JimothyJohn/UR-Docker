"""The pick cycle — a heuristic routine that lifts every block in view, no model
in the loop.

``perceptronics pick-cycle`` drives a **running cockpit** (``perceptronics gui``) over
its HTTP API — the cockpit owns the camera and the robot's Primary link, so this
is a client, the same way the MCP tools are. The routine:

1. **Survey** from the current (measuring) pose: white blobs in the wrist
   image, each one's top face as the nearest 3-D plane (points within 5 mm of
   it), its centre and in-plane axes in the base frame through the cockpit's
   hand-eye. The camera must be ≥ 0.25 m from the blocks — the D435 has no
   depth closer than ~0.2 m.
2. For each block, in place: yaw the flange so the fingers close across the
   block's short side, hover with the **fingertips** 40 mm over its top,
   descend to the top edge, then ``grasp_below_mm`` further, close, lift
   ``lift_mm``, set it back down, release, clear. A close that holds nothing
   (Robotiq ``OBJ`` 3) opens and moves on; a protective stop unlocks, lifts and
   moves on.
3. Optionally a **drop pass** (``--drop``): the same grasp, then release from
   ``drop_mm`` up so the pile shuffles.

Every phase is an event with a wall-clock stamp (``events.json``), and
``--record DIR`` saves the three cockpit feeds alongside so
``scripts/pilot/assemble.py`` can cut a subtitled timelapse.

The fingertip length is the one number the routine cannot see: ``tip_m``
(Hand-E 157 mm + the 6 mm bracket adapter by default). The tool axis may be
tilted, so every target is placed for the **tip**, ``flange = tip − R·(0,0,L)``.
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys
import threading
import time
import urllib.error
import urllib.request
import zlib
from collections.abc import Sequence
from dataclasses import dataclass, field

from urctl.config import RobotConfig
from urctl.pose import Transform
from urctl.robot import Robot

from .handeye import DEFAULT_TIP_M, tip_m_from_env
from .partspec import PartSpec
from .pngio import load_png

DEFAULT_COCKPIT = "http://127.0.0.1:7621"
WHITE_MIN, WHITE_CHROMA = 180, 60  # foam reads bluish-white under a cool white balance (207,227,251)

# -- image heuristics ----------------------------------------------------------------------


def white_level(width: int, height: int, channels: int, rgb: bytes, *, step: int = 8) -> int:
    """The "white" threshold for this picture: 60 % of the brightest neutral pixels (the
    99th percentile of min(r, g, b) over low-chroma pixels), never above :data:`WHITE_MIN`
    and never below 110. Up close, under the camera's and the gripper's own shadow, a
    foam block reads (169, 180, 178) — grey to a fixed 180 (UR3e, 2026-09-27: block
    pixels 156-177 against a p99 of 238, the carpet at 64 at most)."""
    vals = []
    for y in range(0, height, step):
        for x in range(0, width, step):
            i = (y * width + x) * channels
            r, g, b = rgb[i], rgb[i + 1], rgb[i + 2]
            if max(r, g, b) - min(r, g, b) < WHITE_CHROMA:
                vals.append(min(r, g, b))
    if not vals:
        return WHITE_MIN
    vals.sort()
    return max(110, min(WHITE_MIN, int(0.6 * vals[min(len(vals) - 1, int(len(vals) * 0.99))])))


def white_blobs(
    width: int,
    height: int,
    channels: int,
    rgb: bytes,
    *,
    step: int = 4,
    min_px: int = 400,
    white_min: int = WHITE_MIN,
) -> list[dict]:
    """Connected components of bright, low-chroma pixels on a ``step`` grid:
    ``[{px, cx, cy, bbox}]`` largest first."""
    cells: set[tuple[int, int]] = set()
    for y in range(0, height, step):
        for x in range(0, width, step):
            i = (y * width + x) * channels
            r, g, b = rgb[i], rgb[i + 1], rgb[i + 2]
            if min(r, g, b) > white_min and max(r, g, b) - min(r, g, b) < WHITE_CHROMA:
                cells.add((x // step, y // step))
    seen: set[tuple[int, int]] = set()
    out = []
    for c in cells:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            cx, cy = stack.pop()
            comp.append((cx, cy))
            for n in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if n in cells and n not in seen:
                    seen.add(n)
                    stack.append(n)
        px = len(comp) * step * step
        if px < min_px:
            continue
        xs = [q[0] * step for q in comp]
        ys = [q[1] * step for q in comp]
        out.append(
            {
                "px": px,
                "cx": sum(xs) // len(xs),
                "cy": sum(ys) // len(ys),
                "bbox": (min(xs), min(ys), max(xs) + step, max(ys) + step),
            }
        )
    return sorted(out, key=lambda b: -b["px"])


def top_face(
    width: int,
    height: int,
    channels: int,
    rgb: bytes,
    depth: bytes,
    scale: float,
    K: dict,
    bbox,
    *,
    white_min: int = WHITE_MIN,
    max_range_m: float = 2.0,
) -> dict | None:
    """The block's top face inside ``bbox``: 3-D points of its white pixels, seeded
    by the nearest 12 mm, refined twice to the plane through them (±5 mm).
    Returns ``{centre (camera m), normal, points, n}`` or None."""
    x0, y0, x1, y1 = bbox
    pts = []
    for y in range(y0, min(y1, height)):
        for x in range(x0, min(x1, width)):
            i = (y * width + x) * channels
            r, g, b = rgb[i], rgb[i + 1], rgb[i + 2]
            if min(r, g, b) > white_min and max(r, g, b) - min(r, g, b) < WHITE_CHROMA:
                d = depth[2 * (y * width + x)] | (depth[2 * (y * width + x) + 1] << 8)
                if d and d * scale <= max_range_m:  # a reflection reads metres away: not the part
                    z = d * scale
                    pts.append(((x - K["ppx"]) * z / K["fx"], (y - K["ppy"]) * z / K["fy"], z))
    if len(pts) < 25:
        return None
    zmin = min(p[2] for p in pts)
    sel = [p for p in pts if p[2] <= zmin + 0.015]
    seed = list(sel)
    normal = [0.0, 0.0, -1.0]
    for _ in range(3):
        c = [sum(p[k] for p in sel) / len(sel) for k in range(3)]
        cov = [[sum((p[a] - c[a]) * (p[b] - c[b]) for p in sel) for b in range(3)] for a in range(3)]
        ex, ey = cov[0], cov[1]
        n = (
            ex[1] * ey[2] - ex[2] * ey[1],
            ex[2] * ey[0] - ex[0] * ey[2],
            ex[0] * ey[1] - ex[1] * ey[0],
        )
        nn = math.sqrt(sum(v * v for v in n)) or 1.0
        normal = [v / nn for v in n]
        sel = [p for p in pts if abs(sum((p[k] - c[k]) * normal[k] for k in range(3))) < 0.007]
        if len(sel) < 20:
            sel = seed  # sparse depth on white foam: the nearest points are the top face
            break
    c = [sum(p[k] for p in sel) / len(sel) for k in range(3)]
    return {"centre": c, "normal": normal, "points": sel, "n": len(sel)}


def plane_axes_xy(points: Sequence[Sequence[float]]) -> dict:
    """PCA of base-frame points in XY: major-axis heading (rad), and the extents
    a uniform rectangle of that spread would have."""
    n = len(points)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    root = math.hypot(sxx - syy, 2 * sxy)
    lam1 = 0.5 * (sxx + syy) + 0.5 * root
    lam2 = max(0.0, 0.5 * (sxx + syy) - 0.5 * root)
    return {
        "theta": theta,
        "major_m": 2 * math.sqrt(3 * lam1 / n),
        "minor_m": 2 * math.sqrt(3 * lam2 / n),
        "centre_xy": (cx, cy),
    }


def surface_height(
    width: int,
    height: int,
    channels: int,
    rgb: bytes,
    depth: bytes,
    scale: float,
    K: dict,
    bbox,
    top: dict,
    *,
    white_min: int = WHITE_MIN,
    max_range_m: float = 2.0,
    step: int = 2,
) -> float | None:
    """How far the top face ``top`` (:func:`top_face`'s result) stands above the surface
    around it: the median distance, along the face's normal, from the face to the
    non-white depth points in a ring just outside ``bbox`` (a quarter of its size, at
    least 6 px). Non-white only, so a neighbouring block's top is not the table.
    Frame-independent (camera metres in, metres out); None when too little of the
    surface is visible (frame edge, depth holes)."""
    x0, y0, x1, y1 = bbox
    margin = max(6, (max(x1 - x0, y1 - y0) + 3) // 4)
    c, n = top["centre"], top["normal"]
    dists = []
    for y in range(max(0, y0 - margin), min(height, y1 + margin), step):
        for x in range(max(0, x0 - margin), min(width, x1 + margin), step):
            if x0 <= x < x1 and y0 <= y < y1:
                continue
            i = (y * width + x) * channels
            r, g, b = rgb[i], rgb[i + 1], rgb[i + 2]
            if min(r, g, b) > white_min and max(r, g, b) - min(r, g, b) < WHITE_CHROMA:
                continue
            d = depth[2 * (y * width + x)] | (depth[2 * (y * width + x) + 1] << 8)
            if not d or d * scale > max_range_m:
                continue
            z = d * scale
            p = ((x - K["ppx"]) * z / K["fx"], (y - K["ppy"]) * z / K["fy"], z)
            dists.append(sum((c[k] - p[k]) * n[k] for k in range(3)))
    if len(dists) < 20:
        return None
    dists.sort()
    return abs(dists[len(dists) // 2])


def grasp_yaw_deg(flange_pose: Sequence[float], minor_heading: float, finger_axis: str = "y") -> float:
    """The smallest rotation about the flange's own Z (degrees) that lines the
    finger travel axis (flange ``finger_axis``, ``"y"`` on the Hand-E as mounted
    here: the pads run along flange X, so the fingers travel along Y) up with
    ``minor_heading`` (base XY, rad) in either sense."""
    ax = (0.0, 1.0, 0.0) if finger_axis == "y" else (1.0, 0.0, 0.0)
    T = Transform.from_pose(flange_pose)
    fa = T.rotate(ax)
    heading = math.atan2(fa[1], fa[0])
    yaw = math.degrees(minor_heading - heading)
    # The rotation is applied about the flange's own Z. With the tool pointing down,
    # +Z is base −Z, so a positive turn about it moves base headings the other way.
    if T.rotate((0.0, 0.0, 1.0))[2] < 0:
        yaw = -yaw
    return (yaw + 90.0) % 180.0 - 90.0


def tip_pose(
    tip: Sequence[float], rotation_pose: Sequence[float], tip_m: float, yaw_deg: float = 0.0
) -> list[float]:
    """Flange pose that puts the fingertips at ``tip`` (base m) with the flange
    orientation of ``rotation_pose`` yawed by ``yaw_deg`` about its own Z."""
    base = Transform.from_pose(rotation_pose)
    yawed = Transform(base.rotation, (0.0, 0.0, 0.0)).compose(
        Transform.from_pose([0, 0, 0, 0, 0, math.radians(yaw_deg)])
    )
    zax = yawed.rotate((0.0, 0.0, 1.0))
    return [tip[i] - zax[i] * tip_m for i in range(3)] + list(yawed.to_pose()[3:])


def grasp_rotation(flange_pose: Sequence[float], top: Sequence[float], lean_deg: float = 0.0) -> list[float]:
    """A pose (zero translation) whose tool Z points straight down — the work surface
    is flat and parallel to the base XY plane (Nick, 2026-09-27) — or leaned
    ``lean_deg`` outward, tipping the fingertips away from the base column toward
    ``top`` so the tool's length buys reach. The flange heading (its X axis, projected)
    is kept from ``flange_pose`` so wrist 3 turns as little as possible."""
    t = math.radians(lean_deg)
    r = math.hypot(top[0], top[1]) or 1.0
    z = (math.sin(t) * top[0] / r, math.sin(t) * top[1] / r, -math.cos(t))
    x0 = Transform.from_pose(flange_pose).rotate((1.0, 0.0, 0.0))
    d = sum(x0[i] * z[i] for i in range(3))
    x = [x0[i] - d * z[i] for i in range(3)]
    if math.hypot(*x) < 1e-6:  # the flange X was along the new Z: any heading will do
        x = [1.0, 0.0, 0.0]
        d = sum(x[i] * z[i] for i in range(3))
        x = [x[i] - d * z[i] for i in range(3)]
    n = math.hypot(*x)
    x = [v / n for v in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    rot = tuple(tuple((x, y, z)[j][i] for j in range(3)) for i in range(3))
    return Transform(rot, (0.0, 0.0, 0.0)).to_pose()  # type: ignore[arg-type]


def reject_off_surface(blocks: list, tolerance_m: float = 0.03) -> list:
    """Blocks lie on one surface: drop any candidate whose top height is more than
    ``tolerance_m`` from the median of the others (a strap end on the rail, a
    reflection, a block perched on the frame). Re-indexes the survivors."""
    if len(blocks) < 3:
        return blocks
    zs = sorted(b.centre_base[2] for b in blocks)
    median = zs[len(zs) // 2]
    kept = [b for b in blocks if abs(b.centre_base[2] - median) <= tolerance_m]
    for i, b in enumerate(kept):
        b.index = i
    return kept


def where_words(p: Sequence[float]) -> str:
    """A base-frame point as a person would say it: how far out from the robot's base."""
    return f"about {math.hypot(p[0], p[1]) * 100:.0f} cm out from my base"


def detect_blocks(
    w: int,
    h: int,
    ch: int,
    rgb: bytes,
    depth: bytes,
    depth_scale_m: float,
    K: dict,
    T_fc: Transform,
    T_bf: Transform,
    *,
    part: PartSpec | None = None,
    rejects: list[dict] | None = None,
) -> list[Block]:
    """White blocks in one aligned RGB-D frame, each top face placed in the base frame
    through the hand-eye ``T_fc`` (flange → colour camera) and the flange pose ``T_bf``
    at the frame's instant. Identity transforms give the camera frame (a preview with
    no robot). Candidates clipped by the frame edge, the wrong size, or off the common
    surface are dropped. The right size is ``part`` when given (its footprint and, if
    set, height above the surface — :class:`perceptronics.partspec.PartSpec`), else the
    foam blocks'. ``rejects``, when a list, collects the size-rejected candidates
    (``pixel``, ``size_mm``, ``height_mm``, ``why``) for the teach screen. Shared by
    :meth:`PickCycle.survey` and the robot program's pick server
    (:mod:`perceptronics.picknode`)."""
    blocks: list[Block] = []
    for b in white_blobs(w, h, ch, rgb):
        x0, y0, x1, y1 = b["bbox"]
        if x0 <= 6 or y0 <= 6 or x1 >= w - 6 or y1 >= h - 6:
            continue  # clipped at the frame edge
        tf = top_face(w, h, ch, rgb, depth, depth_scale_m, K, b["bbox"])
        if tf is None:
            continue
        # The depth has holes on white foam; the colour blob is complete. Take the
        # centre and extents from every white pixel, back-projected at the top face's
        # depth, and let the depth points only say how far away that face is.
        z = tf["centre"][2]
        pts_cam = []
        for y in range(y0, min(y1, h)):
            for x in range(x0, min(x1, w)):
                i = (y * w + x) * ch
                r, g, bb = rgb[i], rgb[i + 1], rgb[i + 2]
                if min(r, g, bb) > WHITE_MIN and max(r, g, bb) - min(r, g, bb) < WHITE_CHROMA:
                    pts_cam.append(((x - K["ppx"]) * z / K["fx"], (y - K["ppy"]) * z / K["fy"], z))
        pts_base = [T_bf.apply(T_fc.apply(p)) for p in pts_cam]
        ax = plane_axes_xy(pts_base)
        cx, cy = ax["centre_xy"]
        centre = [cx, cy, sum(p[2] for p in pts_base) / len(pts_base)]
        # Orientation from the depth points of the top face itself: the colour blob
        # also holds the side faces seen at an angle, and they turn the axes.
        if tf["n"] >= 150:
            ax_depth = plane_axes_xy([T_bf.apply(T_fc.apply(p)) for p in tf["points"]])
            ax["theta"] = ax_depth["theta"]
        tall = surface_height(w, h, ch, rgb, depth, depth_scale_m, K, b["bbox"], tf)
        if part is not None:
            why = part.why_not(ax["major_m"], ax["minor_m"], tall)
        elif ax["major_m"] > 0.07 or ax["minor_m"] > 0.06 or ax["minor_m"] < 0.010:
            why = "not a block"  # a velcro strap (100 x 15 mm), the rail, a speck
        else:
            why = None
        if why is not None:
            if rejects is not None:
                rejects.append(
                    {
                        "pixel": [b["cx"], b["cy"]],
                        "size_mm": [round(ax["major_m"] * 1000), round(ax["minor_m"] * 1000)],
                        "height_mm": None if tall is None else round(tall * 1000),
                        "why": why,
                    }
                )
            continue
        blocks.append(
            Block(
                len(blocks),
                centre,
                ax["theta"],
                ax["major_m"],
                ax["minor_m"],
                (b["cx"], b["cy"]),
                len(pts_cam),
                tall,
            )
        )
    return reject_off_surface(blocks)


# -- the cockpit client ---------------------------------------------------------------------


class CockpitError(RuntimeError):
    pass


@dataclass
class Cockpit:
    base: str = DEFAULT_COCKPIT
    timeout_s: float = 120.0

    def post(self, path: str, body: dict | None = None) -> dict:
        req = urllib.request.Request(
            self.base + path,
            data=json.dumps(body or {}).encode(),
            headers={"Content-Type": "application/json"},
        )
        try:
            return json.load(urllib.request.urlopen(req, timeout=self.timeout_s))
        except urllib.error.HTTPError as exc:
            try:
                return json.load(exc)
            except Exception:
                return {"ok": False, "error": f"HTTP {exc.code}"}

    def get(self, path: str) -> dict:
        return json.load(urllib.request.urlopen(self.base + path, timeout=self.timeout_s))

    def frame(
        self, after: int | None = None, timeout_ms: int | None = None
    ) -> tuple[dict, int, int, int, bytes, bytes]:
        """The newest frame; with ``after``, one newer than that sequence number
        (the cockpit long-polls up to ``timeout_ms``, then answers with its newest)."""
        query = [f"after={int(after)}"] if after is not None else []
        if timeout_ms is not None:
            query.append(f"timeout_ms={int(timeout_ms)}")
        url = self.base + "/api/rgbd" + ("?" + "&".join(query) if query else "")
        b = urllib.request.urlopen(url, timeout=self.timeout_s).read()
        if b[:4] != b"RGBD":
            raise CockpitError("no RGB-D frame from the cockpit")
        hl = struct.unpack(">I", b[4:8])[0]
        hdr = json.loads(b[8 : 8 + hl])
        i = 8 + hl
        pl = struct.unpack(">I", b[i : i + 4])[0]
        png = b[i + 4 : i + 4 + pl]
        i += 4 + pl
        dl = struct.unpack(">I", b[i : i + 4])[0]
        depth = zlib.decompress(b[i + 4 : i + 4 + dl])
        w, h, ch, rgb = load_png_bytes(png)
        return hdr, w, h, ch, rgb, depth


def load_png_bytes(png: bytes):
    import tempfile

    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
        f.write(png)
        name = f.name
    try:
        return load_png(name)
    finally:
        os.unlink(name)


# -- the routine --------------------------------------------------------------------------


@dataclass
class Block:
    index: int
    centre_base: list[float]  # top-face centre, base frame (m)
    theta: float  # major-axis heading, base XY (rad)
    major_m: float
    minor_m: float
    pixel: tuple[int, int]
    n: int
    height_m: float | None = None  # top above the surrounding surface (m); None: not seen


@dataclass
class PickCycle:
    cockpit: Cockpit = field(default_factory=Cockpit)
    robot: Robot | None = None  # direct drive: motion + gripper over Primary as compiled programs
    tip_m: float = DEFAULT_TIP_M
    hover_mm: float = 40.0
    look_mm: float = 90.0  # the second look: camera ≥ 0.25 m from the top (D435 range)
    grasp_below_mm: float = 15.0
    lift_mm: float = 25.4
    clear_mm: float = 60.0
    drop_mm: float = 120.0
    velocity: float = 0.25  # transit (m/s); approach 0.10, grasp 0.05
    accel: float = 0.8
    min_radius_m: float = 0.2  # skip blocks closer than this to the base column (UR3e: 0.19 m stopped)
    max_radius_m: float = 0.0  # skip blocks farther than this (default: the arm's reach less 50 mm)
    force: int = 80
    stroke_m: float = 0.05  # Hand-E
    finger_axis: str = "y"  # flange axis the fingers travel along
    leans_deg: tuple = (0.0, 12.0, 24.0)  # straight down first; lean out only if the IK needs it
    dry_run: bool = False
    log: list[dict] = field(default_factory=list)
    t0: float = field(default_factory=time.time)
    say_fn: object = None

    # -- bookkeeping ------------------------------------------------------------------
    def say(self, text: str, *, think: str | None = None, do: str | None = None) -> None:
        """Log one event. ``text`` is the engineer's line; ``think`` / ``do`` are the
        same moment in plain words (what the robot has concluded, what it is about
        to do) — what ``scripts/pilot/assemble.py`` captions for a lay audience."""
        ev: dict = {"t": round(time.time() - self.t0, 2), "text": text}
        if think:
            ev["think"] = think
        if do:
            ev["do"] = do
        self.log.append(ev)
        if callable(self.say_fn):
            self.say_fn(ev)
        else:
            print(f"[{ev['t']:6.1f}s] {text}", flush=True)

    def _state(self) -> dict:
        if self.robot is not None:
            return self.robot.get_state()
        return self.cockpit.post("/api/robot/state")

    def _flange(self) -> list[float]:
        if self.robot is not None:
            r = self.robot.get_flange_pose()
            if not r.get("ok"):
                raise CockpitError(r.get("error") or "no flange pose")
            return list(r["flange"])
        r = self.cockpit.post(
            "/api/robot/locate", {"standoff_m": 0.2, "reference": "flange", "point_m": [0, 0, 0.3]}
        )
        if not r.get("flange_pose"):
            raise CockpitError(r.get("error") or "no flange pose")
        return r["flange_pose"]

    def _move(self, pose: Sequence[float], v: float | None = None) -> dict:
        if self.dry_run:
            return {"ok": True, "landed": list(pose), "dry_run": True}
        if self.robot is not None:
            return self.robot.move_tcp(
                list(pose), velocity=v or self.velocity, acceleration=self.accel, tcp=[0.0] * 6
            )
        return self.cockpit.post(
            "/api/robot/move", {"pose": list(pose), "velocity": v or self.velocity, "tcp": [0] * 6}
        )

    def _path(self, legs: list[dict]) -> dict:
        """One compiled program: several legs, blends, inline gripper actions."""
        if self.dry_run:
            out_legs = []
            for leg in legs:
                entry = {**leg, "ok": True, "landed": leg["pose"]}
                if leg.get("gripper"):
                    closing = leg["gripper"] == "close"
                    entry["gripper"] = {
                        "action": leg["gripper"],
                        "POS": 110 if closing else 3,
                        "OBJ": 2 if closing else 3,
                        "object_detected": closing,
                    }
                out_legs.append(entry)
            return {"ok": True, "legs": out_legs, "dry_run": True}
        if self.robot is not None:
            return self.robot.move_tcp_path(legs, tcp=[0.0] * 6)
        out = {"ok": True, "legs": []}
        for leg in legs:  # no compiled path through the cockpit: leg by leg
            r = self._move(leg["pose"], leg.get("velocity"))
            entry = {**leg, "ok": bool(r.get("ok")), "landed": r.get("landed")}
            if leg.get("gripper"):
                g = self._gripper(leg["gripper"])
                entry["gripper"] = {
                    "action": leg["gripper"],
                    **(g.get("status") or {}),
                    "object_detected": g.get("object_detected"),
                }
            out["legs"].append(entry)
            if not r.get("ok"):
                out.update({"ok": False, "protective_stop": r.get("protective_stop")})
                break
        return out

    def _gripper(self, action: str) -> dict:
        if self.dry_run:
            return {"ok": True, "object_detected": action == "close", "status": {}}
        if self.robot is not None:
            return self.robot.gripper(action, force=self.force)
        r = self.cockpit.post("/api/robot/gripper", {"action": action})
        if r.get("ok") or "no route" not in (r.get("error") or ""):
            return r
        # A cockpit that predates the gripper route: drive the robot it is linked to
        # in-process (the same Robot.gripper `urctl gripper` calls) — no uv, no PATH.
        host = (self.cockpit.get("/api/robot").get("robot") or {}).get("host") or None
        try:
            return Robot(RobotConfig.from_env(host=host)).gripper(action, force=self.force)
        except Exception as exc:
            return {"ok": False, "error": str(exc)[-200:]}

    def _bring_up(self) -> None:
        if self.robot is not None:
            self.robot.bring_up()
        else:
            self.cockpit.post("/api/robot/bring_up")

    def _recover(self, what: str) -> None:
        self.say(
            f"Protective stop during {what}: unlocking and lifting clear",
            think="I felt more resistance than expected, so I stopped myself — that's my safety reflex.",
            do="Unlocking my joints and backing straight up and away.",
        )
        self._bring_up()
        time.sleep(1.5)
        fl = self._flange()
        zax = Transform.from_pose(fl).rotate((0.0, 0.0, 1.0))
        self._move([fl[i] - zax[i] * self.clear_mm / 1000.0 for i in range(3)] + list(fl[3:]), 0.1)

    def bail_out(self) -> None:
        """Ctrl-C / kill, once the interrupted call has unwound: stop whatever program is
        still running on the controller (Dashboard, not Primary — the in-flight program
        must die first), let go of whatever is held, back off along the tool axis."""
        try:
            self.say(
                "Interrupted: stopping the program, opening the gripper and backing off",
                think="I've been told to stop.",
                do="Stopping, opening my fingers and backing away.",
            )
            if self.robot is not None:
                self.robot.stop()
            else:
                self.cockpit.post("/api/robot/stop")
            time.sleep(0.5)
            self._gripper("open")
            fl = self._flange()
            zax = Transform.from_pose(fl).rotate((0.0, 0.0, 1.0))
            self._move([fl[i] - zax[i] * self.clear_mm / 1000.0 for i in range(3)] + list(fl[3:]), 0.1)
            self.say("Backed off; the gripper is open")
        except Exception as exc:  # best effort on the way out
            self.say(f"bail-out incomplete: {exc}")

    def ensure_gripper(self) -> bool:
        """The Robotiq must be activated (STA 3) before a pick; activate it if not."""
        st = self._gripper("status")
        status = st.get("status") or {}
        if st.get("ok") and status.get("ACT") == 1 and status.get("STA") == 3:
            return True
        if self.dry_run:
            return True
        self.say(
            "Gripper not activated — running the activation cycle",
            think="My hand hasn't been switched on since the restart.",
            do="Opening and closing my fingers once so the hand can calibrate itself.",
        )
        act = self._gripper("activate")
        if not act.get("ok"):
            self.say(f"Gripper activation failed: {act.get('error')}")
            return False
        return True

    def _ok_after(self, r: dict, what: str) -> bool:
        if r.get("ok"):
            return True
        if r.get("protective_stop") or "PROTECTIVE" in (self._state().get("safety_mode") or ""):
            self._recover(what)
        else:
            self.say(f"{what} refused: {r.get('error') or (r.get('safety') or {}).get('violations')}")
        return False

    # -- survey -------------------------------------------------------------------------
    def survey(self) -> list[Block]:
        rob = self.cockpit.get("/api/robot").get("robot") or {}
        he = (rob.get("handeye") or {}).get("flange_to_color_pose")
        if not he:
            raise CockpitError("the cockpit has no hand-eye (calibrate first)")
        flange = self._flange()
        hdr, w, h, ch, rgb, depth = self.cockpit.frame()
        return detect_blocks(
            w,
            h,
            ch,
            rgb,
            depth,
            hdr["depth_scale_m"],
            hdr["intrinsics"],
            Transform.from_pose(he),
            Transform.from_pose(flange),
        )

    def refine(self, blk: Block, radius_m: float = 0.06) -> Block | None:
        """Re-detect ``blk`` from the current (closer) pose: the survey candidate whose
        top centre lies within ``radius_m`` of it, else None."""
        best = None
        for cand in self.survey():
            d = math.hypot(cand.centre_base[0] - blk.centre_base[0], cand.centre_base[1] - blk.centre_base[1])
            if d < radius_m and (best is None or d < best[0]):
                best = (d, cand)
        if best is None:
            return None
        cand = best[1]
        cand.index = blk.index
        dtheta = abs((math.degrees(cand.theta - blk.theta) + 90.0) % 180.0 - 90.0)
        if dtheta > 30.0:
            cand.theta = blk.theta  # the closer look disagrees on orientation: keep the survey's
            cand.major_m, cand.minor_m = blk.major_m, blk.minor_m
        return cand

    def _grasp_legs(
        self, fl: Sequence[float], blk: Block, lean: float
    ) -> tuple[list[float], list[list[float]]]:
        """The rotation for ``lean`` and every flange pose the block's program visits."""
        rot = grasp_rotation(fl, blk.centre_base, lean)
        yaw = grasp_yaw_deg(rot, blk.theta + math.pi / 2, self.finger_axis)
        h0 = tip_pose(blk.centre_base, rot, self.tip_m, yaw)
        zax = Transform.from_pose(h0).rotate((0.0, 0.0, 1.0))
        mms = (
            max(self.hover_mm, self.look_mm),
            self.hover_mm,
            0.0,
            -self.grasp_below_mm,
            self.lift_mm,
            self.clear_mm,
        )
        return rot, [[h0[i] - zax[i] * (mm / 1000.0) for i in range(3)] + list(h0[3:]) for mm in mms]

    def _choose_lean(self, fl: Sequence[float], blk: Block) -> tuple[float, list[float] | None]:
        """The first lean (0 = straight down) whose whole path the controller's IK
        solves. No IK answer (dry run, no direct robot) → straight down, and the
        envelope/controller judge the move as before."""
        for lean in self.leans_deg:
            rot, poses = self._grasp_legs(fl, blk, lean)
            if self.robot is None or self.dry_run:
                return lean, rot
            answers = self.robot.inverse_kin(poses, tcp=[0.0] * 6)
            verdicts = [a.get("reachable") for a in answers]
            if any(v is None for v in verdicts):
                return lean, rot
            if all(verdicts):
                return lean, rot
        return 0.0, None

    # -- one block --------------------------------------------------------------------
    def cycle_block(self, blk: Block, *, drop: bool = False) -> dict:
        label = f"block {blk.index + 1}"
        fl = self._flange()
        if blk.minor_m > self.stroke_m - 0.006:
            self.say(
                f"{label}: short side {blk.minor_m * 1000:.0f} mm is wider than the "
                f"{self.stroke_m * 1000:.0f} mm stroke — skipping"
            )
            return {"block": blk.index, "ok": False, "stage": "stroke"}
        top = blk.centre_base
        lean, rot = self._choose_lean(fl, blk)
        if rot is None:
            self.say(
                f"{label}: no approach the controller can solve (vertical, or leaned up to "
                f"{max(self.leans_deg):.0f} deg) — skipping",
                think=f"Block {blk.index + 1} is {where_words(top)} — past what I can reach, even leaning "
                "my hand out.",
                do="Skipping it.",
            )
            return {"block": blk.index, "ok": False, "stage": "reach"}
        yaw = grasp_yaw_deg(rot, blk.theta + math.pi / 2, self.finger_axis)
        if lean:
            self.say(
                f"{label}: vertical approach unsolvable; leaning the tool {lean:.0f} deg outward",
                think="It's near the edge of my reach, so I'll lean my hand out toward it — "
                "the gripper's length gets me the last few centimetres.",
            )
        self.say(
            f"{label}: top at {[round(v, 3) for v in top]} m, "
            f"{blk.major_m * 1000:.0f} x {blk.minor_m * 1000:.0f} mm; "
            f"wrist 3 turns {yaw:+.0f} deg so the fingers close across the short side",
            think=f"Block {blk.index + 1} is {where_words(top)}. It measures about "
            f"{blk.major_m * 1000:.0f} by {blk.minor_m * 1000:.0f} mm, so I'll grab it across the "
            f"{blk.minor_m * 1000:.0f} mm side — my fingers open to {self.stroke_m * 1000:.0f} mm.",
            do=f"Turning my wrist {abs(yaw):.0f}° to line my fingers up, and moving over it — high enough "
            "for my camera to take a closer look.",
        )
        # everything below runs along the TOOL axis — straight down unless leaned for reach
        hover0 = tip_pose(top, rot, self.tip_m, yaw)  # tip on the top centre, yawed
        zax = Transform.from_pose(hover0).rotate((0.0, 0.0, 1.0))  # tool z, pointing into the part
        along = lambda mm: [hover0[i] - zax[i] * (mm / 1000.0) for i in range(3)] + list(hover0[3:])  # noqa: E731
        look = along(max(self.hover_mm, self.look_mm))
        if not self._ok_after(self._move(look), f"{label} hover"):
            return {"block": blk.index, "ok": False, "stage": "hover"}
        time.sleep(0.5)
        seen = self.refine(blk)
        if seen is not None:
            shift = math.hypot(seen.centre_base[0] - top[0], seen.centre_base[1] - top[1]) * 1000
            blk, top = seen, seen.centre_base
            rot = grasp_rotation(fl, top, lean)
            yaw = grasp_yaw_deg(rot, blk.theta + math.pi / 2, self.finger_axis)
            hover0 = tip_pose(top, rot, self.tip_m, yaw)
            zax = Transform.from_pose(hover0).rotate((0.0, 0.0, 1.0))
            along = lambda mm: [hover0[i] - zax[i] * (mm / 1000.0) for i in range(3)] + list(hover0[3:])  # noqa: E731
            self.say(
                f"{label}: second look from {self.look_mm:.0f} mm — centre moved {shift:.0f} mm, "
                f"{blk.major_m * 1000:.0f} x {blk.minor_m * 1000:.0f} mm, wrist 3 {yaw:+.0f} deg",
                think=(
                    f"Up close it's {shift:.0f} mm from where I first thought — I'll correct for that."
                    if shift >= 3
                    else "Up close it's exactly where I thought it was."
                ),
                do="Lining my fingertips up over its centre.",
            )
        else:
            self.say(
                f"{label}: second look did not find it again — using the survey",
                think="I can't make it out from up here, so I'll trust my first look.",
                do="Lining my fingertips up where I first saw it.",
            )
        # the rest of the block is ONE program on the controller: blended transit, slow
        # approach, close, lift, set down, open, clear — no host round trips in between
        legs = [
            {
                "pose": along(self.hover_mm),
                "velocity": self.velocity,
                "acceleration": self.accel,
                "blend_m": 0.01,
            },
            {"pose": along(0.0), "velocity": 0.10, "acceleration": 0.5},
            {"pose": along(-self.grasp_below_mm), "velocity": 0.05, "acceleration": 0.3, "gripper": "close"},
        ]
        if drop:
            legs += [
                {
                    "pose": along(self.drop_mm),
                    "velocity": self.velocity,
                    "acceleration": self.accel,
                    "gripper": "open",
                },
                {"pose": along(self.drop_mm + 20.0), "velocity": self.velocity, "acceleration": self.accel},
            ]
            self.say(
                f"{label}: one program — in, close, {self.drop_mm:.0f} mm up the tool axis, let go",
                think="Everything lines up. Time to grab it.",
                do=f"Lowering slowly, closing my fingers, lifting it {self.drop_mm / 10:.0f} cm "
                "and letting go.",
            )
        else:
            legs += [
                {"pose": along(self.lift_mm), "velocity": 0.10, "acceleration": 0.5, "dwell_s": 0.4},
                {
                    "pose": along(-self.grasp_below_mm),
                    "velocity": 0.05,
                    "acceleration": 0.3,
                    "gripper": "open",
                },
                {"pose": along(self.clear_mm), "velocity": self.velocity, "acceleration": self.accel},
            ]
            self.say(
                f"{label}: one program — in, close, up {self.lift_mm:.0f} mm, back down, open, clear",
                think="Everything lines up. Time to grab it.",
                do="Lowering slowly past its top edge, closing my fingers, lifting it an inch to test "
                "the grip, then setting it back down and letting go.",
            )
        r = self._path(legs)
        legs_out = r.get("legs") or []
        grip = legs_out[2].get("gripper") if len(legs_out) > 2 else None
        grip = grip if isinstance(grip, dict) else {}
        held = bool(grip.get("object_detected"))
        pos = grip.get("POS")
        if not r.get("ok"):
            if r.get("protective_stop") or "PROTECTIVE" in (self._state().get("safety_mode") or ""):
                self._recover(f"{label} program")
                self._gripper("open")
                return {"block": blk.index, "ok": False, "stage": "program", "held": held}
            done = sum(1 for leg in r.get("legs") or [] if leg.get("ok"))
            if done == 0:
                self.say(
                    f"{label}: the program never ran a leg — the controller refused the first move (reach?)"
                )
                self._gripper("open")
                return {"block": blk.index, "ok": False, "stage": "refused"}
            self.say(
                f"{label}: program stopped after {done} of {len(legs)} legs: "
                f"{r.get('error') or 'no completion'}"
            )
            self._gripper("open")
            return {"block": blk.index, "ok": False, "stage": "program", "held": held}
        if held:
            width_mm = (255 - (pos or 0)) / 255 * self.stroke_m * 1000
            self.say(
                f"{label}: held at Robotiq {pos} "
                f"(about {width_mm:.0f} mm), " + ("dropped" if drop else "lifted, set back, released"),
                think=f"My fingers stopped {width_mm:.0f} mm apart instead of closing all the way — "
                "so I had it.",
                do="On to the next one." if not drop else "It's down; on to the next one.",
            )
        else:
            self.say(
                f"{label}: closed on nothing (POS {pos}) — the program carried on empty",
                think="My fingers closed all the way — I missed it.",
                do="Moving on to the next one.",
            )
        return {"block": blk.index, "ok": held, "stage": "dropped" if drop else "replaced", "pos": pos}

    # -- the whole run ----------------------------------------------------------------
    def _out_of_band(self, blk: Block) -> str | None:
        """Why this block is not for this arm: too close to the column, or beyond
        its reach less a margin (the envelope checks the flange, the IK the whole
        chain — a block at the edge fails on the controller, not in the envelope)."""
        r = math.hypot(blk.centre_base[0], blk.centre_base[1])
        if self.min_radius_m and r < self.min_radius_m:
            return f"{r:.2f} m from the column — too close for this arm"
        limit = self.max_radius_m
        if not limit and self.robot is not None:
            try:
                probe = getattr(self.robot, "_ensure_reach", None)
                if callable(probe):
                    probe()  # the reach cap is sized from the model on first use
                reach = self.robot.max_reach
                lean = math.radians(max(self.leans_deg or (0.0,)))
                limit = float(reach() if callable(reach) else reach) - 0.05 + self.tip_m * math.sin(lean)
            except Exception:
                limit = 0.0
        if limit and r > limit:
            return f"{r:.2f} m out — beyond this arm's {limit:.2f} m working reach"
        return None

    def survey_from(self, poses: Sequence[Sequence[float]] | None) -> list[Block]:
        """Survey from each pose in turn (None = from here) and merge blocks seen
        twice (top centres within 30 mm), keeping the better-supported view."""
        if not poses:
            return self.survey()
        merged: list[Block] = []
        for pose in poses:
            if not self._ok_after(self._move(list(pose)), "survey move"):
                continue
            time.sleep(0.8)
            for b in self.survey():
                twin = None
                for m in merged:
                    if (
                        math.hypot(m.centre_base[0] - b.centre_base[0], m.centre_base[1] - b.centre_base[1])
                        < 0.03
                    ):
                        twin = m
                        break
                if twin is None:
                    b.index = len(merged)
                    merged.append(b)
                elif b.n > twin.n:
                    b.index = twin.index
                    merged[twin.index] = b
        return merged

    def run(
        self,
        *,
        drop: bool = False,
        max_blocks: int = 8,
        survey_poses: Sequence[Sequence[float]] | None = None,
    ) -> dict:
        start = list(survey_poses[0]) if survey_poses else self._flange()
        st = self._state()
        if "REMOTE" not in (st.get("control_mode") or "REMOTE"):
            raise CockpitError("robot is in Local control — motion needs Remote")
        if not self.ensure_gripper():
            raise CockpitError("gripper not ready")
        self.say(
            "Survey: white blocks in the wrist camera, each top face fitted as a plane "
            "and placed in the base frame",
            think="First I need to know where the blocks are.",
            do="Looking down through the camera on my wrist for white shapes, and measuring how far "
            "away each one is.",
        )
        blocks = self.survey_from(survey_poses)
        self.say(
            f"Found {len(blocks)} block(s)",
            think=f"I see {len(blocks)} block{'s' if len(blocks) != 1 else ''}"
            + (". I'll pick them up one at a time." if blocks else " — nothing to pick up."),
        )
        results = []
        for blk in blocks[:max_blocks]:
            why = self._out_of_band(blk)
            if why:
                self.say(
                    f"block {blk.index + 1}: {why}, skipping",
                    think=f"Block {blk.index + 1} is too "
                    f"{'close to my base' if 'column' in why else 'far away'} for me to reach safely.",
                    do="Skipping it.",
                )
                results.append({"block": blk.index, "ok": False, "stage": "radius"})
                continue
            results.append(self.cycle_block(blk))
        n_ok = sum(1 for r in results if r["ok"])
        self.say(
            f"Pass 1 done: {n_ok} of {len(results)} lifted and replaced. Back to the survey pose",
            think=f"I picked up {n_ok} of {len(results)}.",
            do="Going back to where I started.",
        )
        self._move(start)
        if drop:
            time.sleep(0.8)
            blocks = self.survey_from(survey_poses)
            self.say(f"Drop pass: {len(blocks)} block(s) — each one up {self.drop_mm:.0f} mm and let go")
            for blk in blocks[:max_blocks]:
                if self._out_of_band(blk):
                    continue
                results.append(self.cycle_block(blk, drop=True))
                self._move(start)
                time.sleep(0.5)
        self.say("Done", think="All done.")
        return {"ok": True, "results": results, "events": self.log}


# -- recording (the three cockpit feeds with host stamps) -------------------------------


class Recorder:
    """Save ``/api/view/0``, ``/api/view/1`` (JPEG) and ``/api/rgbd`` (PNG) to
    ``out`` with timestamps relative to ``t0``; ``index.json`` + ``t0.json`` are
    what ``scripts/pilot/assemble.py`` reads."""

    def __init__(self, cockpit: Cockpit, out: str, t0: float):
        self.cockpit, self.out, self.t0 = cockpit, out, t0
        self.idx: dict[str, list] = {"v0": [], "v1": [], "c": []}
        self.stop = threading.Event()
        self.lock = threading.Lock()
        for d in ("v0", "v1", "c"):
            os.makedirs(f"{out}/{d}", exist_ok=True)
        json.dump({"t0": t0}, open(f"{out}/t0.json", "w"))
        self.threads = [
            threading.Thread(target=self._view, args=(0,), daemon=True),
            threading.Thread(target=self._view, args=(1,), daemon=True),
            threading.Thread(target=self._wrist, daemon=True),
        ]

    def start(self) -> None:
        for t in self.threads:
            t.start()

    def _view(self, i: int) -> None:
        seq = 0
        while not self.stop.is_set():
            try:
                r = urllib.request.urlopen(
                    f"{self.cockpit.base}/api/view/{i}?after={seq}&timeout_ms=1000", timeout=5
                )
                s = int(r.headers.get("X-Seq", "0"))
                data = r.read()
                if s == seq:
                    continue
                seq = s
                t = time.time() - self.t0
                name = f"{self.out}/v{i}/{int(t * 1000):08d}.jpg"
                open(name, "wb").write(data)
                with self.lock:
                    self.idx[f"v{i}"].append((t, name))
                time.sleep(0.15)
            except Exception:
                time.sleep(0.3)

    def _wrist(self) -> None:
        while not self.stop.is_set():
            try:
                b = urllib.request.urlopen(f"{self.cockpit.base}/api/rgbd", timeout=5).read()
                hl = struct.unpack(">I", b[4:8])[0]
                i = 8 + hl
                pl = struct.unpack(">I", b[i : i + 4])[0]
                png = b[i + 4 : i + 4 + pl]
                t = time.time() - self.t0
                name = f"{self.out}/c/{int(t * 1000):08d}.png"
                open(name, "wb").write(png)
                with self.lock:
                    self.idx["c"].append((t, name))
                time.sleep(0.25)
            except Exception:
                time.sleep(0.3)

    def finish(self) -> dict:
        self.stop.set()
        for t in self.threads:
            t.join(timeout=5)
        json.dump(self.idx, open(f"{self.out}/index.json", "w"))
        return {k: len(v) for k, v in self.idx.items()}


def lock_view_focus(cockpit: Cockpit, spec_text: str | None) -> list[dict]:
    """Before recording: lock the cockpit's webcam views at the focus the cell names
    (``PERCEPTRONICS_VIEW_FOCUS``, ``perceptronics.uvc``) — a C920 hunting for focus blurs
    every other second of a timelapse. Runs from this (unprivileged) process, never
    touches the RealSense; failures are reported and recording goes on."""
    from .uvc import focus_for, parse_focus_spec, set_focus

    spec = parse_focus_spec(spec_text)
    if spec is None:
        return []
    try:
        views = [v.get("name", "") for v in cockpit.get("/api/info").get("views") or []]
    except Exception:
        views = []
    out = []
    for name in views:
        act, focus = focus_for(spec, name)
        if act:
            r = set_focus(name, focus)
            out.append(r)
            state = "autofocus" if focus is None else f"focus locked at {focus}"
            failed = "" if r.get("ok") else f" FAILED: {r.get('error')}"
            print(f"view {name!r}: {state}{failed}", file=sys.stderr)
    return out


# -- CLI --------------------------------------------------------------------------------


def add_pick_cycle_args(ap) -> None:
    ap.add_argument(
        "--cockpit", default=DEFAULT_COCKPIT, help=f"the running cockpit (default {DEFAULT_COCKPIT})"
    )
    ap.add_argument(
        "--drop",
        action="store_true",
        help="after the in-place pass, drop each block from --drop-mm to shuffle",
    )
    ap.add_argument(
        "--tip-m",
        type=float,
        default=None,
        help="flange-to-fingertip length (default: $PERCEPTRONICS_TIP_M, else 0.163 Hand-E + adapter)",
    )
    ap.add_argument("--lift-mm", type=float, default=25.4, help="in-place lift (default 25.4 = one inch)")
    ap.add_argument("--drop-mm", type=float, default=120.0, help="release height for the drop pass")
    ap.add_argument(
        "--grasp-below-mm", type=float, default=15.0, help="how far below the top face the pads close"
    )
    ap.add_argument(
        "--record",
        default=None,
        metavar="DIR",
        help="record the three cockpit feeds + events for a timelapse",
    )
    ap.add_argument(
        "--survey-pose",
        type=float,
        nargs=6,
        action="append",
        default=None,
        metavar="V",
        help="a flange pose [x y z rx ry rz] overlooking the blocks from >= 0.25 m; repeat for several "
        "views, merged by position (default: survey from where the arm is)",
    )
    ap.add_argument(
        "--stroke-m",
        type=float,
        default=0.05,
        help="gripper stroke; a block wider than this is skipped (default 0.05)",
    )
    ap.add_argument(
        "--finger-axis",
        choices=["x", "y"],
        default="y",
        help="flange axis the fingers travel along (default y)",
    )
    ap.add_argument(
        "--via-cockpit",
        action="store_true",
        help="drive the robot through the cockpit's API leg by leg (default: direct — this process compiles "
        "each block into one URScript program over Primary; the cockpit does vision only)",
    )
    ap.add_argument(
        "--robot-host", default=None, help="controller address (default: the cockpit's robot, else UR_HOST)"
    )
    ap.add_argument(
        "--min-radius-m",
        type=float,
        default=0.2,
        help="skip blocks closer than this to the base column (default 0.2; 0 = off)",
    )
    ap.add_argument(
        "--max-radius-m",
        type=float,
        default=0.0,
        help="skip blocks farther than this (default: reach − 50 mm)",
    )
    ap.add_argument("--velocity", type=float, default=0.25, help="transit speed m/s (default 0.25)")
    ap.add_argument("--dry-run", action="store_true", help="survey and plan, send no motion")
    ap.add_argument("--json", action="store_true", help="print the result as JSON")


def run_pick_cycle(args) -> int:
    cockpit = Cockpit(args.cockpit)
    robot = None
    if not args.via_cockpit:
        host = args.robot_host
        if not host:
            try:
                host = (cockpit.get("/api/robot").get("robot") or {}).get("host")
            except Exception:
                host = None
        robot = Robot(
            RobotConfig.from_env(host=host) if host else RobotConfig.from_env(), dry_run=args.dry_run
        )
    cycle = PickCycle(
        cockpit,
        robot=robot,
        velocity=args.velocity,
        min_radius_m=args.min_radius_m,
        max_radius_m=args.max_radius_m,
        tip_m=tip_m_from_env() if args.tip_m is None else args.tip_m,
        lift_mm=args.lift_mm,
        drop_mm=args.drop_mm,
        grasp_below_mm=args.grasp_below_mm,
        stroke_m=args.stroke_m,
        finger_axis=args.finger_axis,
        dry_run=args.dry_run,
    )
    rec = None
    if args.record:
        lock_view_focus(cockpit, os.environ.get("PERCEPTRONICS_VIEW_FOCUS"))
        rec = Recorder(cockpit, args.record, cycle.t0)
        rec.start()
        time.sleep(1.0)
    import signal

    def _interrupt(signum, frame):
        raise KeyboardInterrupt  # unwinds the blocked Primary call (releasing its lock) first

    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    try:
        if args.dry_run:
            blocks = cycle.survey_from(args.survey_pose)
            out = {"ok": True, "dry_run": True, "blocks": [b.__dict__ for b in blocks]}
            for b in blocks:
                cycle.say(
                    f"block {b.index + 1}: top {[round(v, 3) for v in b.centre_base]} m, "
                    f"{b.major_m * 1000:.0f} x {b.minor_m * 1000:.0f} mm at {math.degrees(b.theta):.0f} deg"
                )
        else:
            out = cycle.run(drop=args.drop, survey_poses=args.survey_pose)
    except KeyboardInterrupt:
        cycle.bail_out()
        return 130
    except (CockpitError, urllib.error.URLError, OSError) as exc:
        print(f"pick-cycle: {exc}", file=sys.stderr)
        return 1
    finally:
        if rec is not None:
            time.sleep(2.0)
            counts = rec.finish()
            json.dump(cycle.log, open(f"{args.record}/events.json", "w"), indent=1)
            print(f"recorded {counts} → {args.record}", file=sys.stderr)
    if args.json:
        print(json.dumps(out, indent=2, default=str))
    return 0 if out.get("ok") else 1
