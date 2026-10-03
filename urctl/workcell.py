"""Named positions, TCP offsets and workplanes, the same on every controller.

The pick routine is only as good as three numbers it can't see for itself: where
the tool's point is (the **TCP offset**), where the table is (a **workplane**,
touched off with that point at three places) and where the arm should go to look
(**positions**). Each vendor stores these differently — a UR keeps TCPs and
plane features in its installation, PolyScope X in its own model, a UFACTORY
controller a TCP offset and a user frame — so this module keeps the toolkit's
own copy, one JSON file per cell, written by the same calls whatever the arm:

* :class:`Workplane` — a plane from three touched points, in the convention the
  URCap's ``PoseMath.plane`` and :meth:`perceptronics.volume.Surface.from_points`
  already use: point 1 is the corner (origin), point 2 lies along +X, point 3 is
  on the far side (+Y); the normal points up off the table.
* :class:`CellStore` — ``{"positions": …, "tcps": …, "workplanes": …}`` in
  ``captures/cell/<cell>.json`` (``URCTL_CELL_STORE`` overrides the path,
  ``UR_CELL`` names the cell). Writes are atomic (temp file + rename).

Everything is metres and UR-style rotation vectors in the robot's base frame;
:mod:`urctl.ufactory` converts at its own wire. The TCP offset itself is read
and written on the controller by ``get_tcp_offset`` / ``set_tcp_offset``
(:class:`urctl.robot.Robot`, :class:`urctl.ufactory.UFactoryArm`); the store
only remembers named ones so a cell can switch tools by name.
"""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
import time
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

from .pose import Transform

Vec3 = tuple[float, float, float]

ENV_STORE = "URCTL_CELL_STORE"
ENV_CELL = "UR_CELL"
DEFAULT_DIR = Path("captures") / "cell"
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
# Three touches closer than this, or this close to one line, don't pin a plane down.
MIN_SPAN_M = 0.01
MIN_SINE = 0.05  # ≈ 3° between the X edge and the far-side point
SCHEMA = 1


class WorkcellError(ValueError):
    """A bad name, a degenerate plane, or a store that can't be read."""


def check_name(name: str) -> str:
    """Names are keys in a JSON file and appear in logs: 1–64 of ``A-Z a-z 0-9 _ . -``,
    starting alphanumeric — nothing a log line or a path could be bent with."""
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise WorkcellError(f"name must be 1-64 of A-Z a-z 0-9 _ . - (starting alphanumeric), got {name!r}")
    return name


def _vec(p: Sequence[float], what: str) -> Vec3:
    if len(p) < 3:
        raise WorkcellError(f"{what} needs x, y, z")
    v = (float(p[0]), float(p[1]), float(p[2]))
    if not all(math.isfinite(c) for c in v):
        raise WorkcellError(f"{what} must be finite numbers")
    return v


def _sub(a, b) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a) -> Vec3:
    n = _norm(a)
    return (a[0] / n, a[1] / n, a[2] / n)


@dataclass(frozen=True)
class Workplane:
    """A plane in the base frame, touched off at three points with the TCP.

    ``origin`` is point 1; ``x_axis`` points toward point 2; ``normal`` points up
    (+Z in base); ``y_axis = normal × x_axis``. ``size`` is the area from the
    origin to the projections of points 2 and 3 along X and Y (m, signed)."""

    origin: Vec3
    x_axis: Vec3
    y_axis: Vec3
    normal: Vec3
    size: tuple[float, float]
    points: tuple[Vec3, Vec3, Vec3]
    tcp_offset: tuple[float, ...] | None = None  # the TCP the touches were made with
    taught_at: float = 0.0

    @classmethod
    def from_points(
        cls,
        p1: Sequence[float],
        p2: Sequence[float],
        p3: Sequence[float],
        *,
        tcp_offset: Sequence[float] | None = None,
        taught_at: float | None = None,
    ) -> Workplane:
        a, b, c = _vec(p1, "point 1"), _vec(p2, "point 2"), _vec(p3, "point 3")
        ab, ac = _sub(b, a), _sub(c, a)
        if _norm(ab) < MIN_SPAN_M or _norm(ac) < MIN_SPAN_M:
            raise WorkcellError(
                f"touch the points at least {MIN_SPAN_M * 1000:.0f} mm apart "
                f"(2 is {_norm(ab) * 1000:.1f} mm from 1, 3 is {_norm(ac) * 1000:.1f} mm)"
            )
        n = _cross(ab, ac)
        if _norm(n) < MIN_SINE * _norm(ab) * _norm(ac):
            raise WorkcellError("the three points are (nearly) on one line: touch point 3 off the 1-2 edge")
        x = _unit(ab)
        # the normal by Surface.from_points' own operations, so the two agree to the bit:
        # on a wall n_z is ±1e-17 noise and "point it up" must flip the same way in both
        n = _unit(_cross(x, ac))
        if n[2] < 0:
            n = (-n[0], -n[1], -n[2])
        y = _cross(n, x)
        size = (_dot(ab, x), _dot(ac, y))
        offset = None if tcp_offset is None else tuple(float(v) for v in tcp_offset)
        return cls(a, x, y, n, size, (a, b, c), offset, time.time() if taught_at is None else taught_at)

    # -- geometry ---------------------------------------------------------------------

    def pose(self) -> list[float]:
        """The plane frame as a UR pose ``[x, y, z, rx, ry, rz]`` (Z = the normal) —
        what the pick node sends as ``plane=`` and a UR plane feature holds."""
        return Transform.from_axes(self.x_axis, self.y_axis, self.normal, self.origin).to_pose()

    def height(self, p: Sequence[float]) -> float:
        """Signed distance of ``p`` above the plane (m)."""
        return _dot(_sub(_vec(p, "point"), self.origin), self.normal)

    def tilt_deg(self) -> float:
        """Angle between the normal and base +Z. On a level table this is touch-off error
        (or a robot not mounted square)."""
        return math.degrees(math.acos(max(-1.0, min(1.0, self.normal[2]))))

    def centre(self) -> Vec3:
        u, v = self.size[0] / 2.0, self.size[1] / 2.0
        o, x, y = self.origin, self.x_axis, self.y_axis
        return (o[0] + u * x[0] + v * y[0], o[1] + u * x[1] + v * y[1], o[2] + u * x[2] + v * y[2])

    def table_z_at(self, x: float, y: float) -> float:
        """The plane's base-frame Z above ``(x, y)``."""
        n, o = self.normal, self.origin
        if abs(n[2]) < 1e-9:
            raise WorkcellError("the plane is vertical: it has no height above a point")
        return o[2] - (n[0] * (x - o[0]) + n[1] * (y - o[1])) / n[2]

    def as_dict(self) -> dict:
        return {
            "origin": list(self.origin),
            "x_axis": list(self.x_axis),
            "y_axis": list(self.y_axis),
            "normal": list(self.normal),
            "size": list(self.size),
            "points": [list(p) for p in self.points],
            "pose": self.pose(),
            "tilt_deg": round(self.tilt_deg(), 3),
            # a wall (or any plane within ~6° of vertical) has no "height above the base"
            "table_z": round(self.table_z_at(*self.centre()[:2]), 5) if abs(self.normal[2]) > 0.1 else None,
            "tcp_offset": None if self.tcp_offset is None else list(self.tcp_offset),
            "taught_at": self.taught_at,
        }

    @classmethod
    def from_dict(cls, d: dict) -> Workplane:
        """Rebuilt from its touched points (the derived fields are not trusted)."""
        p = d.get("points")
        if not isinstance(p, list) or len(p) != 3:
            raise WorkcellError("a stored workplane needs its three points")
        return cls.from_points(*p, tcp_offset=d.get("tcp_offset"), taught_at=float(d.get("taught_at", 0.0)))


def compare_planes(taught: Workplane, normal: Sequence[float], point: Sequence[float]) -> dict:
    """How far a measured plane (``normal`` + any ``point`` on it, base frame) sits from
    the taught one: ``offset_mm`` — the measured plane's height above the taught one at
    the taught area's centre, along the taught normal — and ``tilt_deg`` between the
    two normals. A level table touched off well and a calibrated camera read ~0 / ~0."""
    n = _unit(_vec(normal, "normal"))
    if n[2] < 0:
        n = (-n[0], -n[1], -n[2])
    q = _vec(point, "point")
    c = taught.centre()
    tn = taught.normal
    denom = _dot(n, tn)
    if abs(denom) < 1e-6:
        raise WorkcellError("the measured plane is perpendicular to the taught one")
    # where the taught normal through the centre meets the measured plane
    t = _dot(_sub(q, c), n) / denom
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, _dot(n, tn)))))
    return {"offset_mm": round(t * 1000.0, 2), "tilt_deg": round(tilt, 3)}


# --------------------------------------------------------------------------------------
# The store
# --------------------------------------------------------------------------------------


def default_store_path(env: dict | None = None) -> Path:
    env = os.environ if env is None else env
    explicit = (env.get(ENV_STORE) or "").strip()
    if explicit:
        return Path(explicit)
    cell = (env.get(ENV_CELL) or "").strip() or "default"
    return DEFAULT_DIR / f"{check_name(cell)}.json"


@dataclass
class CellStore:
    """One cell's positions, named TCPs and workplanes, as a JSON file."""

    path: Path
    data: dict = field(default_factory=dict)

    @classmethod
    def open(cls, path: str | Path | None = None) -> CellStore:
        p = Path(path) if path is not None else default_store_path()
        store = cls(p)
        if p.exists():
            try:
                raw = json.loads(p.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise WorkcellError(f"cannot read the cell store {p}: {exc}") from exc
            if not isinstance(raw, dict):
                raise WorkcellError(f"the cell store {p} is not a JSON object")
            store.data = raw
        for key in ("positions", "tcps", "workplanes"):
            if not isinstance(store.data.get(key), dict):
                store.data[key] = {}
        store.data["schema"] = SCHEMA
        return store

    def save(self) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".cell-", suffix=".json", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(self.data, fh, indent=2, sort_keys=True)
                fh.write("\n")
            os.replace(tmp, self.path)
        except BaseException:
            Path(tmp).unlink(missing_ok=True)
            raise
        return self.path

    # -- positions -----------------------------------------------------------------------

    def put_position(
        self, name: str, *, joints: Sequence[float] | None, tcp: Sequence[float] | None, **extra
    ) -> dict:
        if joints is None and tcp is None:
            raise WorkcellError("a position needs joints or a TCP pose")
        rec = {
            "joints": None if joints is None else [float(v) for v in joints],
            "tcp": None if tcp is None else [float(v) for v in tcp],
            "saved_at": time.time(),
            **extra,
        }
        for key in ("joints", "tcp"):
            if rec[key] is not None and not all(math.isfinite(v) for v in rec[key]):
                raise WorkcellError(f"{key} must be finite numbers")
        self.data["positions"][check_name(name)] = rec
        return rec

    def position(self, name: str) -> dict:
        try:
            return self.data["positions"][check_name(name)]
        except KeyError:
            raise WorkcellError(f"no position named {name!r}") from None

    # -- named TCPs ----------------------------------------------------------------------

    def put_tcp(self, name: str, offset: Sequence[float]) -> list[float]:
        off = [float(v) for v in offset]
        if len(off) != 6 or not all(math.isfinite(v) for v in off):
            raise WorkcellError("a TCP offset is 6 finite numbers [x, y, z, rx, ry, rz]")
        self.data["tcps"][check_name(name)] = off
        return off

    def tcp(self, name: str) -> list[float]:
        try:
            return list(self.data["tcps"][check_name(name)])
        except KeyError:
            raise WorkcellError(f"no TCP named {name!r}") from None

    # -- workplanes ----------------------------------------------------------------------

    def touch(
        self, name: str, index: int, point: Sequence[float], tcp_offset: Sequence[float] | None
    ) -> dict:
        """Record touch ``index`` (1–3) of plane ``name``; with all three, the plane is
        (re)computed and stored. A touch made with a different TCP than the earlier ones
        restarts the plane — mixing tools would tilt it."""
        if index not in (1, 2, 3):
            raise WorkcellError("a workplane is touched at points 1, 2 and 3")
        pending = self.data.setdefault("touches", {})
        rec = pending.get(check_name(name)) or {"points": [None, None, None], "tcp_offset": None}
        offset = None if tcp_offset is None else [float(v) for v in tcp_offset]
        if rec["tcp_offset"] is not None and offset is not None and _tcp_differs(rec["tcp_offset"], offset):
            rec = {"points": [None, None, None], "tcp_offset": None}
        rec["points"][index - 1] = list(_vec(point, f"point {index}"))
        rec["tcp_offset"] = offset if offset is not None else rec["tcp_offset"]
        pending[name] = rec
        if all(p is not None for p in rec["points"]):
            plane = Workplane.from_points(*rec["points"], tcp_offset=rec["tcp_offset"])
            self.data["workplanes"][name] = plane.as_dict()
            del pending[name]
            return {"complete": True, "workplane": plane.as_dict()}
        return {"complete": False, "touched": [i + 1 for i, p in enumerate(rec["points"]) if p is not None]}

    def put_workplane(self, name: str, plane: Workplane) -> dict:
        self.data["workplanes"][check_name(name)] = plane.as_dict()
        return plane.as_dict()

    def workplane(self, name: str) -> Workplane:
        try:
            return Workplane.from_dict(self.data["workplanes"][check_name(name)])
        except KeyError:
            raise WorkcellError(f"no workplane named {name!r}") from None

    def delete(self, kind: str, name: str) -> bool:
        if kind not in ("positions", "tcps", "workplanes"):
            raise WorkcellError(f"unknown kind {kind!r}")
        return self.data[kind].pop(check_name(name), None) is not None

    def names(self, kind: str) -> list[str]:
        return sorted(self.data.get(kind, {}))


def _tcp_differs(a: Sequence[float], b: Sequence[float], tol_m: float = 1e-4, tol_rad: float = 1e-3) -> bool:
    return any(abs(a[i] - b[i]) > tol_m for i in range(3)) or any(
        abs(a[i] - b[i]) > tol_rad for i in range(3, 6)
    )


# --------------------------------------------------------------------------------------
# Operations on any Controller (urctl.controller): the same calls on every arm
# --------------------------------------------------------------------------------------

TCP_ACTIONS = ("get", "set", "save", "use", "list", "delete")
POSITION_ACTIONS = ("save", "get", "list", "delete", "move_to")
WORKPLANE_ACTIONS = ("touch", "define", "get", "list", "delete", "use_tcp")


def _fail(action: str, error: str) -> dict:
    return {"action": action, "ok": False, "error": error}


def tcp_offset(
    robot, action: str, *, name: str | None = None, offset=None, store: CellStore | None = None
) -> dict:
    """``get`` / ``set`` the controller's active TCP; ``save`` a named one (``offset``,
    or the active one), ``use`` a named one (writes it to the controller), ``list``,
    ``delete``."""
    if action not in TCP_ACTIONS:
        return _fail("tcp_offset", f"action must be one of {TCP_ACTIONS}")
    try:
        if action == "get":
            return robot.get_tcp_offset()
        if action == "set":
            if offset is None:
                return _fail("tcp_offset", "set needs offset [x, y, z, rx, ry, rz]")
            return robot.set_tcp_offset(list(offset))
        store = store or CellStore.open()
        if action == "list":
            return {
                "action": "tcp_offset",
                "ok": True,
                "tcps": dict(store.data["tcps"]),
                "store": str(store.path),
            }
        if name is None:
            return _fail("tcp_offset", f"{action} needs a name")
        if action == "delete":
            gone = store.delete("tcps", name)
            store.save()
            return {"action": "tcp_offset", "ok": gone, "deleted": name if gone else None}
        if action == "use":
            return {**robot.set_tcp_offset(store.tcp(name)), "name": name}
        # save
        if offset is None:
            got = robot.get_tcp_offset()
            if not got.get("ok"):
                return got
            offset = got["tcp_offset"]
        saved = store.put_tcp(name, offset)
        store.save()
        return {
            "action": "tcp_offset",
            "ok": True,
            "name": name,
            "tcp_offset": saved,
            "store": str(store.path),
        }
    except WorkcellError as exc:
        return _fail("tcp_offset", str(exc))


def position(robot, action: str, *, name: str | None = None, store: CellStore | None = None, **move) -> dict:
    """``save`` the arm's current joints + TCP pose (+ the TCP offset it was taken
    with) under ``name``; ``get``; ``list``; ``delete``; ``move_to`` it (a joint move
    to the saved joints — the same place whatever TCP is active; a linear move to the
    saved TCP pose when only that was stored)."""
    if action not in POSITION_ACTIONS:
        return _fail("position", f"action must be one of {POSITION_ACTIONS}")
    try:
        store = store or CellStore.open()
        if action == "list":
            return {
                "action": "position",
                "ok": True,
                "positions": store.names("positions"),
                "store": str(store.path),
            }
        if name is None:
            return _fail("position", f"{action} needs a name")
        check_name(name)
        if action == "get":
            return {"action": "position", "ok": True, "name": name, **store.position(name)}
        if action == "delete":
            gone = store.delete("positions", name)
            store.save()
            return {"action": "position", "ok": gone, "deleted": name if gone else None}
        if action == "move_to":
            rec = store.position(name)
            if rec.get("joints"):
                return {**robot.move_joints(rec["joints"], **move), "name": name}
            return {**robot.move_tcp(rec["tcp"], **move), "name": name}
        state = robot.get_state()
        if not state.get("ok") or (state.get("joints") is None and state.get("tcp") is None):
            return _fail("position", state.get("error") or "the arm reported no joints or TCP pose")
        off = robot.get_tcp_offset()
        rec = store.put_position(
            name,
            joints=state.get("joints"),
            tcp=state.get("tcp"),
            tcp_offset=off.get("tcp_offset") if off.get("ok") else None,
        )
        store.save()
        return {"action": "position", "ok": True, "name": name, **rec, "store": str(store.path)}
    except WorkcellError as exc:
        return _fail("position", str(exc))


def workplane(
    robot,
    action: str,
    *,
    name: str | None = None,
    index: int | None = None,
    points=None,
    store: CellStore | None = None,
) -> dict:
    """``touch`` records the live TCP position as point ``index`` (1 corner, 2 along
    +X, 3 on the far side) — touch the table with the tool's point; after the third the
    plane is computed and saved. ``define`` takes the three points directly. ``get``,
    ``list``, ``delete``. The answer always carries ``tilt_deg`` (off base XY) and
    ``table_z``."""
    if action not in WORKPLANE_ACTIONS:
        return _fail("workplane", f"action must be one of {WORKPLANE_ACTIONS}")
    try:
        store = store or CellStore.open()
        if action == "list":
            planes = {n: store.workplane(n).as_dict() for n in store.names("workplanes")}
            return {"action": "workplane", "ok": True, "workplanes": planes, "store": str(store.path)}
        if name is None:
            return _fail("workplane", f"{action} needs a name")
        check_name(name)
        if action == "get":
            return {"action": "workplane", "ok": True, "name": name, **store.workplane(name).as_dict()}
        if action == "delete":
            gone = store.delete("workplanes", name)
            store.save()
            return {"action": "workplane", "ok": gone, "deleted": name if gone else None}
        if action == "use_tcp":
            plane = store.workplane(name)
            if plane.tcp_offset is None:
                return _fail("workplane", f"{name!r} was not touched with a recorded TCP")
            return {**robot.set_tcp_offset(list(plane.tcp_offset)), "name": name}
        if action == "define":
            if not isinstance(points, list | tuple) or len(points) != 3:
                return _fail("workplane", "define needs points: three [x, y, z] in the base frame")
            plane = Workplane.from_points(*points)
            out = store.put_workplane(name, plane)
            store.save()
            return {"action": "workplane", "ok": True, "name": name, **out}
        # touch
        if index not in (1, 2, 3):
            return _fail("workplane", "touch needs index 1 (corner), 2 (along +X) or 3 (far side)")
        state = robot.get_state()
        tcp = state.get("tcp")
        if not state.get("ok") or not tcp:
            return _fail("workplane", state.get("error") or "the arm reported no TCP pose")
        off = robot.get_tcp_offset()
        got = store.touch(name, index, tcp[:3], off.get("tcp_offset") if off.get("ok") else None)
        store.save()
        return {"action": "workplane", "ok": True, "name": name, "index": index, "point": tcp[:3], **got}
    except WorkcellError as exc:
        return _fail("workplane", str(exc))


__all__ = [
    "CellStore",
    "Workplane",
    "WorkcellError",
    "check_name",
    "compare_planes",
    "default_store_path",
    "position",
    "tcp_offset",
    "workplane",
]
