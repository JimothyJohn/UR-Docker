"""The pick server — what the PolyScope 5 **Perceptronic Pick** program node talks to.

The node's URScript runs *inside* the robot program the operator plays from the
pendant, so it runs in Local mode, needs no Primary connection and no cockpit robot
link. At the node the controller opens a plain TCP socket to this server (the
cockpit's host, ``--pick-port``, default 7622), sends one line with its own flange
pose, and reads back one parenthesised list of numbers — the shape URScript's
``socket_read_ascii_float`` parses. The controller then does the moving itself.

Requests (one line each, ASCII, ≤ 1 kB; the pose is URScript's ``to_str(pose)``):

``FIND p[x, y, z, rx, ry, rz] [u=<px> v=<px>] [lean=<deg>]``
    Detect the white blocks in a colour+depth frame taken *after* the request
    arrived (the arm has stopped by then), place them in the base frame through the
    hand-eye and the flange pose sent, and pick one: the block nearest pixel
    ``(u, v)`` — the spot the operator tapped when teaching the node — or, without
    one, the block nearest the image centre.

``REFINE p[x, y, z, rx, ry, rz] p[cx, cy, cz, 0, 0, 0] [lean=<deg>]``
    The second, closer look: the block whose top centre lies within 60 mm of
    ``(cx, cy, cz)`` (the FIND answer), seen from the flange pose sent.

``LOOK p[x, y, z, rx, ry, rz] p[cx, cy, cz, 0, 0, 0]``
    Where to take the closer look from: the flange pose that puts the camera halfway
    from where it is now to the block's top centre (never nearer than
    :data:`LOOK_MIN_M` — the D435 has no depth closer than ~0.2 m), aimed so the block
    is in the middle of the picture, and backed out along that line until the
    fingertips clear the top by :data:`LOOK_TIP_CLEAR_M`. Status -6 when no such pose
    exists (the node then looks from straight over the block, as before).

``part=<L>x<W>[x<H>] [tol=<pct>]`` (FIND and REFINE, optional): the part's rough size in
mm as it lies — footprint and height above the table — and how far off it may measure
(default 25 %). Only candidates that size are considered
(:class:`perceptronics.partspec.PartSpec`); without it, anything foam-block-sized.

``LOG <text>``
    The robot program saying where it is (``start``, ``FIND status 1``, ``hover`` …):
    written to the server's log, printable ASCII only, capped; **no reply**.

``lean`` (0–30°, default 0): the grasp is **straight down** — the work surface is
flat and parallel to the base XY plane (Nick, 2026-09-27) — unless the program's own
IK check found that unsolvable and asks again leaned outward (the pick-cycle's
0 → 12 → 24° ladder, :func:`perceptronics.pickcycle.grasp_rotation`).

Reply: ``(status, cx, cy, cz, x, y, z, rx, ry, rz)``. ``status`` is one of
:data:`STATUS`; on 1, ``c*`` is the block's top-face centre (base, m) and the pose
is the **flange** pose that puts the fingertips (``PERCEPTRONICS_TIP_M`` along flange
+Z) on that centre, tool Z straight down (or leaned), the flange heading kept from the
pose sent and turned so the fingers close across the block's short side. The node
derives hover, grip and lift from it with ``pose_trans`` along the tool axis. On any
other status the numbers are 0.

The server never moves anything and never talks to the robot: it answers questions
about the image. It shares the cockpit's (unauthenticated, trusted-cell) network
exposure — bind it the same way.
"""

from __future__ import annotations

import math
import re
import socketserver
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from urctl.pose import Transform

from . import partspec
from .partspec import PartSpec
from .pickcycle import Block, detect_blocks, grasp_rotation, grasp_yaw_deg, tip_pose
from .volume import Reach, Scene, Surface, find_parts, parse_order

DEFAULT_PICK_PORT = 7622
MAX_LINE = 1024
REFINE_RADIUS_M = 0.06
LOOK_MIN_M = 0.25  # camera to the block's top, the closest the D435 still measures well
LOOK_TIP_CLEAR_M = 0.06  # fingertips above the top at the look pose
LOOK_MAX_TILT_DEG = 60.0  # tool Z from straight down
MAX_LOG_TEXT = 240
DEFAULT_STROKE_M = 0.050  # Hand-E
FRESH_FRAMES = 2  # frames after the request's arrival before one is trusted still

STATUS = {
    1: "found",
    0: "no block in view",
    -1: "the block is wider than the gripper's stroke",
    -2: "the block is too close to the robot's base",
    -3: "the cockpit has no hand-eye calibration",
    -4: "no fresh camera frame",
    -5: "the second look did not find the block again",
    -6: "no look pose keeps the camera in range and the fingertips clear",
    -7: "something is in view, but nothing the size of the part",
    -9: "malformed request",
    # protocol 2 (the 0.5.0 node): why a location had nothing to pick
    -10: "the only parts in view are out of reach",
    -11: "no room for the open fingers beside any part",
    -12: "the parts in view are outside the pick area",
    -13: "the only part in view is cut off by the edge of the picture",
}

_NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
_POSE_RX = re.compile(
    rf"p\[\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*\]"
)
_KV_RX = re.compile(r"\b([uv])=(-?\d{1,5})\b")
_LEAN_RX = re.compile(rf"\blean=({_NUM})")
MAX_LEAN_DEG = 30.0


class RequestError(ValueError):
    """The line is not a request this server answers."""


def parse_request(line: str) -> dict:
    """``{"verb", "flange", "near" (REFINE, LOOK), "pixel" (FIND, optional), "part"
    (FIND, REFINE: a PartSpec or None)}``, ``{"verb": "LOG", "text"}``, or RequestError."""
    text = line.strip()
    if text[:4].upper() in ("LOG", "LOG "):  # the program never reads a reply to LOG: never refuse one
        said = text[3:].strip()[: MAX_LOG_TEXT * 2]
        clean = "".join(c if " " <= c <= "~" else "?" for c in said)[:MAX_LOG_TEXT]
        return {"verb": "LOG", "text": clean}
    if not text or len(text) > MAX_LINE:
        raise RequestError("empty or oversized request")
    verb = text.split(None, 1)[0].upper()
    if verb not in ("FIND", "REFINE", "LOOK", "NEXT"):
        raise RequestError(f"unknown verb {verb[:16]!r}")
    opts = parse_options(text)
    poses = [[float(g) for g in m.groups()] for m in _POSE_RX.finditer(_PLANE_RX.sub("", text))]
    if not poses or not all(math.isfinite(v) and abs(v) < 100.0 for p in poses for v in p):
        raise RequestError("no plausible pose p[x, y, z, rx, ry, rz] in the request")
    out: dict = {"verb": verb, "flange": poses[0], "lean": 0.0, "options": opts}
    lean = _LEAN_RX.search(text)
    if lean:
        out["lean"] = float(lean.group(1))
        if not (0.0 <= out["lean"] <= MAX_LEAN_DEG):
            raise RequestError(f"lean must be within 0..{MAX_LEAN_DEG:.0f} deg")
    out["part"] = opts.part if verb != "LOOK" else None
    if verb == "NEXT":
        if not opts.node:
            raise RequestError("NEXT needs node=<id>")
        return out
    if verb in ("REFINE", "LOOK"):
        if len(poses) < 2:
            raise RequestError(f"{verb} needs the flange pose and the block centre")
        out["near"] = poses[1][:3]
    else:
        kv = {k: int(v) for k, v in _KV_RX.findall(text)}
        if "u" in kv and "v" in kv and kv["u"] >= 0 and kv["v"] >= 0:
            out["pixel"] = (kv["u"], kv["v"])
    return out


# -- protocol 2: the options every request of the 0.5.0 node carries -------------------------

_POSE_BODY = rf"\[\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*\]"
_PLANE_RX = re.compile(r"\bplane=p" + _POSE_BODY)
_AREA_RX = re.compile(rf"\barea=({_NUM})[xX]({_NUM})(?![\w.])")
_ORDER_RX = re.compile(r"\border=([A-Za-z]{2},[A-Za-z]{2})\b")
_REACH_RX = re.compile(rf"\breach=({_NUM}),({_NUM})(?![\w.])")
_MM_RX = {k: re.compile(rf"\b{k}=({_NUM})(?![\w.])") for k in ("grip", "stroke")}
_NODE_RX = re.compile(r"\bnode=([0-9A-Za-z]{1,12})\b")
_INT_RX = {k: re.compile(rf"\b{k}=(\d{{1,3}})\b") for k in ("loc", "locs", "proto")}
MAX_LOCS = 32
QUEUE_TTL_S = 120.0  # a part seen at a picture point stays queued this long
PROTO2_FIELDS = 16


@dataclass(frozen=True)
class PickOptions:
    """What the 0.5.0 node tells the server with every request (all optional; the defaults
    are protocol 1's behaviour): ``part=LxWxH tol=T``, the taught surface ``plane=p[...]``
    with ``area=<x>x<y>`` (mm, along the plane's X / Y from its origin), the pick
    ``order=LR,FB``, ``reach=<min>,<max>`` (m from the base axis), the grip depth
    ``grip=<mm>`` and the gripper's ``stroke=<mm>`` (for the finger-room check), and the
    node's identity ``node=<id> loc=<i> locs=<n> proto=2``."""

    part: PartSpec | None = None
    surface: Surface | None = None
    order: tuple[str, str] = ("LR", "FB")
    reach: Reach | None = None
    grip_below_m: float = 0.015
    stroke_m: float = DEFAULT_STROKE_M
    node: str = ""
    loc: int = 0
    locs: int = 1
    proto: int = 1

    def fingers(self) -> dict:
        return {"grasp_below_m": self.grip_below_m, "stroke_m": self.stroke_m}


def parse_options(text: str) -> PickOptions:
    """The :class:`PickOptions` in a request line (or the teach screen's query); RequestError
    when one is present but malformed or out of range."""
    try:
        part = partspec.parse(text)
    except ValueError as exc:
        raise RequestError(str(exc)) from None
    kw: dict = {"part": part}
    m = _PLANE_RX.search(text)
    if m:
        pose = [float(g) for g in m.groups()]
        if not all(math.isfinite(v) and abs(v) < 100 for v in pose):
            raise RequestError("plane must be a pose p[x, y, z, rx, ry, rz]")
        size = None
        a = _AREA_RX.search(text)
        if a:
            size = (float(a.group(1)) / 1000.0, float(a.group(2)) / 1000.0)
            if not all(0.005 <= abs(v) <= 3.0 for v in size):
                raise RequestError("area must be 5..3000 mm each way")
        kw["surface"] = Surface.from_pose(pose, size)
    elif re.search(r"\bplane=", text):
        raise RequestError("plane must be a pose p[x, y, z, rx, ry, rz]")
    o = _ORDER_RX.search(text)
    if o:
        try:
            kw["order"] = parse_order(o.group(1))
        except ValueError as exc:
            raise RequestError(str(exc)) from None
    elif re.search(r"\border=", text):
        raise RequestError("order must be like order=LR,FB")
    r = _REACH_RX.search(text)
    if r:
        lo, hi = float(r.group(1)), float(r.group(2))
        if not (0.0 <= lo < 3.0 and 0.0 <= hi <= 3.0 and (hi == 0 or hi > lo)):
            raise RequestError("reach must be min,max in metres with max > min (max 0: no limit)")
        kw["reach"] = Reach(lo, hi)
    for key, name, lo, hi in (("grip", "grip_below_m", 0.0, 60.0), ("stroke", "stroke_m", 10.0, 300.0)):
        g = _MM_RX[key].search(text)
        if g:
            mm = float(g.group(1))
            if not lo <= mm <= hi:
                raise RequestError(f"{key} must be {lo:.0f}..{hi:.0f} mm")
            kw[name] = mm / 1000.0
    n = _NODE_RX.search(text)
    if n:
        kw["node"] = n.group(1)
    for key in ("loc", "locs", "proto"):
        i = _INT_RX[key].search(text)
        if i:
            kw[key] = int(i.group(1))
    if not 0 <= kw.get("loc", 0) <= MAX_LOCS or not 1 <= kw.get("locs", 1) <= MAX_LOCS:
        raise RequestError(f"loc and locs must be within 1..{MAX_LOCS}")
    if kw.get("proto", 1) not in (1, 2):
        raise RequestError("proto must be 1 or 2")
    return PickOptions(**kw)


def format_reply2(
    status: int,
    centre: Sequence[float] | None = None,
    pose: Sequence[float] | None = None,
    *,
    loc: int = 0,
    order: int = 0,
    remaining: int = 0,
    dims_mm: Sequence[float] | None = None,
) -> str:
    """Protocol 2's line: ``(status, cx, cy, cz, x, …, rz, loc, order, remaining, L, W, H)``
    — :data:`PROTO2_FIELDS` numbers, the part's measured size in mm last."""
    vals = (
        [float(status)]
        + [float(v) for v in (centre or (0.0,) * 3)]
        + [float(v) for v in (pose or (0.0,) * 6)]
        + [float(loc), float(order), float(remaining)]
        + [float(v) for v in (dims_mm or (0.0,) * 3)]
    )
    return "(" + ",".join(f"{v:.6f}" for v in vals) + ")\n"


_SIZE_WHYS = {"too long", "too wide", "too short", "too narrow", "too tall", "too flat", "not a block"}


def scene_status(scene: Scene) -> int:
    """Why a picture had nothing to pick, as the most useful status code."""
    if scene.parts:
        return 1
    whys = [p.why or "" for p in scene.rejected]
    for code, test in (
        (-7, lambda w: w in _SIZE_WHYS or "touching" in w),
        (-1, lambda w: w.startswith("wider than")),
        (-11, lambda w: w.startswith("no room")),
        (-2, lambda w: w.startswith("too close to the base")),
        (-10, lambda w: w.startswith("out of reach")),
        (-12, lambda w: w.startswith("outside the pick area")),
        (-13, lambda w: w.startswith("cut off")),
    ):
        if any(test(w) for w in whys):
            return code
    return 0


def format_reply(
    status: int, centre: Sequence[float] | None = None, pose: Sequence[float] | None = None
) -> str:
    """One ``socket_read_ascii_float`` line: ``(status, cx, cy, cz, x, …, rz)``."""
    vals = (
        [float(status)]
        + [float(v) for v in (centre or (0.0,) * 3)]
        + [float(v) for v in (pose or (0.0,) * 6)]
    )
    return "(" + ",".join(f"{v:.6f}" for v in vals) + ")\n"


def look_pose(
    flange: Sequence[float],
    top: Sequence[float],
    handeye: Sequence[float],
    tip_m: float,
    *,
    min_m: float = LOOK_MIN_M,
    tip_clear_m: float = LOOK_TIP_CLEAR_M,
    max_tilt_deg: float = LOOK_MAX_TILT_DEG,
) -> list[float] | None:
    """The flange pose for the closer look (see ``LOOK``), or None when none will do.

    The camera goes halfway along the line from where it is to ``top`` (not nearer
    than ``min_m``, never farther than it already is) and turns to aim its optical
    axis at ``top``, keeping its image X as close as it was (the least wrist roll).
    If the fingertips would come within ``tip_clear_m`` of the top's height, the
    camera backs out along the same line; a tool tilted past ``max_tilt_deg`` from
    straight down is refused."""
    T_fc = Transform.from_pose(handeye)
    T_bc = Transform.from_pose(flange).compose(T_fc)
    cam = T_bc.translation
    away = [cam[i] - top[i] for i in range(3)]
    d0 = math.hypot(*away)
    if d0 < 1e-6:
        return None
    unit = [a / d0 for a in away]
    z = [-u for u in unit]  # the optical axis, toward the block
    x_cam = T_bc.rotate((1.0, 0.0, 0.0))
    dot = sum(x_cam[i] * z[i] for i in range(3))
    x = [x_cam[i] - dot * z[i] for i in range(3)]
    if math.hypot(*x) < 1e-6:
        return None
    n = math.hypot(*x)
    x = [v / n for v in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    start = min(d0, max(min_m, d0 / 2.0))
    steps = 20
    for k in range(steps + 1):
        d = start + (d0 - start) * k / steps
        origin = [top[i] + unit[i] * d for i in range(3)]
        T_bf = Transform.from_axes(x, y, z, origin).compose(T_fc.inverse())
        tip = T_bf.apply((0.0, 0.0, tip_m))
        tool_z = T_bf.rotate((0.0, 0.0, 1.0))
        if math.degrees(math.acos(max(-1.0, min(1.0, -tool_z[2])))) > max_tilt_deg:
            return None
        if tip[2] >= top[2] + tip_clear_m:
            return T_bf.to_pose()
    return None


def choose(blocks: list[Block], pixel: tuple[int, int] | None, width: int, height: int) -> Block | None:
    """The block nearest ``pixel`` (the taught tap), else nearest the image centre."""
    if not blocks:
        return None
    u, v = pixel if pixel is not None else (width / 2.0, height / 2.0)
    return min(blocks, key=lambda b: (b.pixel[0] - u) ** 2 + (b.pixel[1] - v) ** 2)


class PickPlanner:
    """Frames in, one answer out. ``frame_source(after_seq)`` returns
    ``(seq, w, h, ch, rgb, depth, depth_scale_m, K)`` for a frame newer than
    ``after_seq`` (or None); ``latest_seq()`` the newest sequence number;
    ``handeye()`` the flange → colour-camera pose (or None)."""

    def __init__(
        self,
        frame_source: Callable[[int], tuple | None],
        latest_seq: Callable[[], int],
        handeye: Callable[[], Sequence[float] | None],
        *,
        tip_m: float,
        stroke_m: float = DEFAULT_STROKE_M,
        min_radius_m: float = 0.2,
        finger_axis: str = "y",
        log: Callable[[str, bool], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.frame_source, self.latest_seq, self.handeye = frame_source, latest_seq, handeye
        self.clock = clock
        self._lock = threading.Lock()
        self._queues: dict[str, dict] = {}  # node id -> {"t", "pointer", "items": [...]}
        self.tip_m, self.stroke_m, self.min_radius_m, self.finger_axis = (
            tip_m,
            stroke_m,
            min_radius_m,
            finger_axis,
        )
        self.log = log or (lambda text, ok: None)

    def answer(self, line: str) -> str:
        try:
            req = parse_request(line)
        except RequestError as exc:
            self.log(f"pick request refused: {exc}", False)
            # a protocol-2 program reads 16 numbers: answer in its shape, or it sees a timeout
            return format_reply2(-9) if re.search(r"\bproto=2\b", line) else format_reply(-9)
        if req["verb"] == "LOG":
            self.log(f"robot: {req['text']}", True)
            return ""  # the program does not read a reply to LOG
        if req["options"].proto == 2 and req["verb"] != "LOOK":
            return self._answer2(req)
        status, centre, pose = self._look(req) if req["verb"] == "LOOK" else self._plan(req)
        what = STATUS.get(status, "?")
        where = f" top {[round(c, 3) for c in centre]}" if centre else ""
        self.log(f"pick {req['verb']}: {what}{where}", status == 1)
        return format_reply(status, centre, pose)

    def plan(
        self,
        flange: Sequence[float],
        pixel: tuple[int, int] | None = None,
        lean: float = 0.0,
        part: PartSpec | None = None,
    ) -> dict:
        """A FIND for a caller that already has the flange pose (the node's teach-time
        check through the cockpit): ``{status, reason, centre, top_pose}``."""
        req = {"verb": "FIND", "flange": [float(v) for v in flange], "lean": float(lean), "part": part}
        if pixel is not None:
            req["pixel"] = pixel
        status, centre, pose = self._plan(req)
        return {"status": status, "reason": STATUS.get(status, "?"), "centre": centre, "top_pose": pose}

    def _look(self, req: dict) -> tuple[int, list[float] | None, list[float] | None]:
        he = self.handeye()
        if not he:
            return -3, None, None
        pose = look_pose(req["flange"], req["near"], he, self.tip_m)
        return (1, list(req["near"]), pose) if pose else (-6, list(req["near"]), None)

    def _plan(self, req: dict) -> tuple[int, list[float] | None, list[float] | None]:
        he = self.handeye()
        if not he:
            return -3, None, None
        arrived = self.latest_seq()
        frame = self.frame_source(arrived + FRESH_FRAMES - 1)
        if frame is None:
            return -4, None, None
        _, w, h, ch, rgb, depth, scale, K = frame
        flange = req["flange"]
        part = req.get("part")
        rejects: list[dict] = []
        blocks = detect_blocks(
            w,
            h,
            ch,
            rgb,
            depth,
            scale,
            K,
            Transform.from_pose(he),
            Transform.from_pose(flange),
            part=part,
            rejects=rejects,
        )
        if part is not None and rejects:
            seen = ", ".join(f"{r['size_mm'][0]}x{r['size_mm'][1]} mm {r['why']}" for r in rejects[:4])
            self.log(f"pick {req['verb']}: not the part ({part.token()}): {seen}", False)
        if req["verb"] == "REFINE":
            near = req["near"]
            close = [b for b in blocks if math.dist(b.centre_base[:2], near[:2]) < REFINE_RADIUS_M]
            if not close:
                return -5, None, None
            blk = min(close, key=lambda b: math.dist(b.centre_base[:2], near[:2]))
        else:
            blk = choose(blocks, req.get("pixel"), w, h)
            if blk is None:
                return (-7 if part is not None and rejects else 0), None, None
        if blk.minor_m > self.stroke_m - 0.006:
            return -1, blk.centre_base, None
        if self.min_radius_m and math.hypot(blk.centre_base[0], blk.centre_base[1]) < self.min_radius_m:
            return -2, blk.centre_base, None
        rot = grasp_rotation(flange, blk.centre_base, req["lean"])
        yaw = grasp_yaw_deg(rot, blk.theta + math.pi / 2, self.finger_axis)
        return 1, list(blk.centre_base), tip_pose(blk.centre_base, rot, self.tip_m, yaw)

    # -- protocol 2 ----------------------------------------------------------------------

    def _answer2(self, req: dict) -> str:
        opts: PickOptions = req["options"]
        verb = req["verb"]
        if verb == "NEXT":
            got = self._next(opts)
        elif verb == "FIND":
            got = self._find2(req, opts)
        else:
            got = self._refine2(req, opts)
        status = got["status"]
        what = "nothing queued" if verb == "NEXT" and status == 0 else STATUS.get(status, "?")
        at = got.get("loc", 0)
        where = f" #{got.get('order', 0)} at {at}" if status == 1 else f" (next: {at})"
        self.log(f"pick {verb} [{opts.node or '-'}]: {what}{where}", status in (0, 1))
        return format_reply2(
            status,
            got.get("centre"),
            got.get("pose"),
            loc=got.get("loc", 0),
            order=got.get("order", 0),
            remaining=got.get("remaining", 0),
            dims_mm=got.get("dims_mm"),
        )

    def _queue(self, node: str) -> dict:
        q = self._queues.get(node)
        if q is None:
            q = self._queues[node] = {"t": self.clock(), "pointer": 1, "items": []}
        return q

    def _next(self, opts: PickOptions) -> dict:
        """The next part already seen (no picture needed), else where to look next."""
        with self._lock:
            q = self._queue(opts.node)
            if q["items"] and self.clock() - q["t"] <= QUEUE_TTL_S:
                item = q["items"].pop(0)
                return {**item, "status": 1, "remaining": len(q["items"])}
            q["items"] = []
            if not 1 <= q["pointer"] <= opts.locs:
                q["pointer"] = 1
            return {"status": 0, "loc": q["pointer"]}

    def scene(self, flange: Sequence[float] | None, opts: PickOptions, after: int | None = None):
        """``(status, scene, frame)`` for a picture newer than ``after`` (default: the
        request's arrival) seen from ``flange`` (None: camera-only)."""
        he = self.handeye()
        if flange is not None and not he:
            return -3, None, None
        arrived = self.latest_seq() if after is None else after
        frame = self.frame_source(arrived + FRESH_FRAMES - 1)
        if frame is None:
            return -4, None, None
        _, w, h, _ch, _rgb, depth, scale, K = frame
        T_bc = None if flange is None else Transform.from_pose(flange).compose(Transform.from_pose(he))
        scene = find_parts(
            w,
            h,
            depth,
            scale,
            K,
            T_bc,
            spec=opts.part,
            surface=opts.surface,
            reach=opts.reach,
            order=opts.order,
            fingers=opts.fingers() if T_bc is not None else None,
        )
        for p in list(scene.parts):
            if p.width_m > opts.stroke_m - 0.006:
                p.why = f"wider than the open gripper ({p.width_m * 1000:.0f} mm)"
                scene.parts.remove(p)
                scene.rejected.append(p)
        for n, p in enumerate(scene.parts, 1):
            p.order = n
        return 1, scene, frame

    def _grasp(self, part, flange: Sequence[float], lean: float) -> list[float]:
        rot = grasp_rotation(flange, part.centre_base, lean)
        yaw = grasp_yaw_deg(rot, part.theta + math.pi / 2, self.finger_axis)
        return tip_pose(part.centre_base, rot, self.tip_m, yaw)

    def _item(self, part, flange: Sequence[float], lean: float, loc: int) -> dict:
        return {
            "centre": list(part.centre_base),
            "pose": self._grasp(part, flange, lean),
            "loc": loc,
            "order": part.order,
            "dims_mm": [round(v * 1000, 1) for v in (part.length_m, part.width_m, part.height_m)],
        }

    def _find2(self, req: dict, opts: PickOptions) -> dict:
        loc = opts.loc or 1
        ok, scene, _ = self.scene(req["flange"], opts)
        if ok != 1:
            return {"status": ok, "loc": loc}
        for note in scene.notes:
            self.log(f"pick FIND at {loc}: {note}", False)
        for p in scene.rejected[:6]:
            self.log(
                f"pick FIND at {loc}: not picking {p.length_m * 1000:.0f}x{p.width_m * 1000:.0f}x"
                f"{p.height_m * 1000:.0f} mm at {[round(c, 3) for c in p.centre]}: {p.why}",
                False,
            )
        items = [self._item(p, req["flange"], req["lean"], loc) for p in scene.parts]
        with self._lock:
            q = self._queue(opts.node or "-")
            q["t"] = self.clock()
            if not items:
                q["items"] = []
                q["pointer"] = loc % max(1, opts.locs) + 1  # this location is empty: the next one
                return {"status": scene_status(scene), "loc": q["pointer"]}
            q["pointer"] = loc  # when the queue drains, look here again: picks may uncover more
            q["items"] = items[1:]
            return {**items[0], "status": 1, "remaining": len(items) - 1}

    def _refine2(self, req: dict, opts: PickOptions) -> dict:
        ok, scene, _ = self.scene(req["flange"], opts)
        if ok != 1:
            return {"status": ok, "loc": opts.loc}
        near = req["near"]
        # the close look sees the part from nearer: its neighbours may now be cut off or out of
        # the area, but the part itself is judged only by its size
        edge = [p for p in scene.rejected if (p.why or "").startswith(("cut off", "no room"))]
        pool = list(scene.parts) + edge
        close = [p for p in pool if math.dist(p.centre[:2], near[:2]) < REFINE_RADIUS_M]
        if not close:
            with self._lock:
                self._queue(opts.node or "-")["items"] = []  # what was queued was seen before this change
            return {"status": -5, "loc": opts.loc}
        part = min(close, key=lambda p: math.dist(p.centre[:2], near[:2]))
        if part.why and part.why.startswith("no room"):
            return {"status": -11, "loc": opts.loc, "centre": list(part.centre)}
        item = self._item(part, req["flange"], req["lean"], opts.loc)
        with self._lock:
            remaining = len(self._queue(opts.node or "-")["items"])
        return {**item, "status": 1, "order": 0, "remaining": remaining}


# -- the node's teach screen (the cockpit's routes and the stand-alone pick server's) ---------


def parse_preview_request(payload: dict) -> tuple[tuple[int, int] | None, float, float]:
    """``POST /api/pick/preview``'s body -> ``(pixel, grip_below_mm, hover_mm)``;
    ValueError when a field isn't a number or is out of range. The body's ``part`` /
    ``tol`` are :func:`perceptronics.partspec.from_payload`'s."""
    try:
        u, v = int(payload.get("u", -1)), int(payload.get("v", -1))
        grip = float(payload.get("grip_below_mm", 15.0))
        hover = float(payload.get("hover_mm", 40.0))
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(str(exc)) from None
    if not (0.0 <= grip <= 60.0 and 0.0 <= hover <= 300.0):
        raise ValueError("grip_below_mm must be 0..60 and hover_mm 0..300")
    return ((u, v) if u >= 0 and v >= 0 else None), grip, hover


def preview(
    planner: PickPlanner,
    flange_pose: dict,
    pixel: tuple[int, int] | None,
    *,
    grip_below_mm: float = 15.0,
    hover_mm: float = 40.0,
    part: PartSpec | None = None,
) -> dict:
    """The node's teach-time check: what the program would do from where the arm is
    now. ``flange_pose`` is a ``get_flange_pose`` result (``flange``, ``tcp_offset``,
    ``tcp_offset_consistent``); one FIND, and the hover and grip poses both as flange
    poses and in the controller's **active** TCP (``polyscope_*``: what PolyScope's
    hold-to-move screen takes). Moves nothing."""
    from urctl.pose import pose_trans

    fp = flange_pose
    if not fp.get("ok") or not fp.get("flange"):
        return {"ok": False, "error": fp.get("error") or "could not read the flange pose", "robot": fp}
    planned = planner.plan(fp["flange"], pixel, part=part)
    out: dict = {"ok": planned["status"] == 1, **planned, "flange_pose": fp["flange"]}
    if planned["status"] != 1:
        out["error"] = planned["reason"]
        return out
    top = planned["top_pose"]
    out["hover_pose"] = pose_trans(top, [0.0, 0.0, -hover_mm / 1000.0, 0.0, 0.0, 0.0])
    out["grip_pose"] = pose_trans(top, [0.0, 0.0, grip_below_mm / 1000.0, 0.0, 0.0, 0.0])
    offset = fp.get("tcp_offset")
    if offset is not None and fp.get("tcp_offset_consistent") is not False:
        out["polyscope_hover_pose"] = pose_trans(out["hover_pose"], offset)
        out["polyscope_grip_pose"] = pose_trans(out["grip_pose"], offset)
    else:
        out["polyscope_note"] = "the controller's active TCP offset is unknown or inconsistent"
    return out


def detect_report(
    frame: tuple,
    *,
    pick_port: int | None,
    handeye: bool,
    tip_m: float | None,
    part: PartSpec | None = None,
) -> dict:
    """What the node's teach screen draws: the blocks the pick server would choose
    among, in image pixels, from one ``(seq, w, h, ch, rgb, depth, scale, K)`` frame
    (camera frame — no robot needed) — and, as ``rejected``, the candidates that were
    the wrong size (for ``part`` when given) with why, so the operator can see what
    the filter is doing."""
    seq, w, h, ch, rgb, depth, scale, K = frame
    rejects: list[dict] = []
    blocks = detect_blocks(
        w, h, ch, rgb, depth, scale, K, Transform(), Transform(), part=part, rejects=rejects
    )
    return {
        "ok": True,
        "seq": seq,
        "width": w,
        "height": h,
        "pick_port": pick_port,
        "handeye": handeye,
        "tip_m": tip_m,
        "part": None if part is None else part.as_dict(),
        "blocks": [
            {
                "pixel": list(b.pixel),
                "size_mm": [round(b.major_m * 1000), round(b.minor_m * 1000)],
                "height_mm": None if b.height_m is None else round(b.height_m * 1000),
                "distance_m": round(b.centre_base[2], 3),
            }
            for b in blocks
        ],
        "rejected": rejects,
    }


def scene_report(
    planner: PickPlanner,
    flange: Sequence[float] | None,
    opts: PickOptions,
    *,
    pick_port: int | None = None,
) -> dict:
    """What the 0.5.0 node's teach screen draws, computed exactly as the program's FIND
    would from ``flange`` (the live pose; None: camera-only, no reach or pick area):
    every part with its outline in picture pixels and its pick-order number, every
    candidate that isn't picked with why, the surface used. Moves nothing."""
    ok, scene, frame = planner.scene(flange, opts, after=max(0, planner.latest_seq() - FRESH_FRAMES))
    if ok != 1:
        return {"ok": False, "status": ok, "error": STATUS.get(ok, "?")}
    seq, w, h = frame[0], frame[1], frame[2]
    out = scene.as_dict()
    if flange is not None:
        # the grasp for each part (fingertips on its top centre, flange pose): the teach screen's
        # "Check approach" backs it off along the tool axis for PolyScope's move screen
        for d, p in zip(out["parts"], scene.parts, strict=True):
            d["grasp_pose"] = [round(v, 6) for v in planner._grasp(p, flange, 0.0)]
    out.update(
        ok=True,
        seq=seq,
        width=w,
        height=h,
        pick_port=pick_port,
        base_frame=flange is not None,
        status=scene_status(scene),
        reason=STATUS.get(scene_status(scene), "?"),
        part=None if opts.part is None else opts.part.as_dict(),
        order=list(opts.order),
    )
    if flange is None:
        out["notes"].append("no live robot pose: reach and the pick area are not checked")
    return out


class _Handler(socketserver.StreamRequestHandler):
    timeout = 30.0  # a controller that connects and goes quiet does not hold a thread

    def handle(self) -> None:
        planner: PickPlanner = self.server.planner  # type: ignore[attr-defined]
        while True:
            try:
                raw = self.rfile.readline(MAX_LINE + 1)
            except OSError:
                return
            if not raw:
                return
            if len(raw) > MAX_LINE:
                self.wfile.write(format_reply(-9).encode())
                return  # a line that long is not a controller; drop the connection
            try:
                line = raw.decode("ascii")
            except UnicodeDecodeError:
                if raw.lstrip()[:3].upper() == b"LOG":  # never answered, whatever it holds
                    planner.answer(raw.decode("ascii", errors="replace"))
                    continue
                self.wfile.write(format_reply(-9).encode())
                continue
            answer = planner.answer(line)
            if answer:
                self.wfile.write(answer.encode())


class PickServer(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, bind: str, port: int, planner: PickPlanner):
        super().__init__((bind, port), _Handler)
        self.planner = planner
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self.serve_forever, name="pick-server", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.shutdown()
        self.server_close()
