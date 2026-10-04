"""Where the camera sits on the robot, and how a camera-frame point becomes a
base-frame target the robot can move to.

Frames (all right-handed):

* **colour camera** — what :meth:`perceptronics.rgbd.RgbdFrame.point_at` and the
  segment features (``point_m``) are expressed in: x right, y down, z out of
  the lens (aligned depth lives on the colour grid, so the colour optical
  centre is the origin).
* **depth camera** — the left imager. The SDK's extrinsics (``p_color = R
  p_depth + t``, read at open, see ``RealSenseCamera.extrinsics_depth_to_color``)
  relate the two; on a D435 that is ~15 mm along x.
* **flange** — UR's tool-flange frame (origin at the flange face centre, +Z
  out of the flange). The bracket's nominal placement of the depth origin in
  this frame is :data:`BRACKET_NOMINAL`.
* **base** — UR's base frame, what ``get_actual_tcp_pose()`` / ``movel`` use.
  The flange pose in the base comes from the controller
  (``Robot.get_flange_pose``).

``p_base = T_base_flange · T_flange_depth · T_depth_color · p_color``.

:data:`BRACKET_SEEDS` are *seeds*, one per print (``eseries`` / ``ur20``,
picked by ``PERCEPTRONICS_BRACKET``): derived from the bracket geometry
(``hardware/d435-tool-bracket/README.md`` §3) and the camera's published
imager position, not from a calibration. Expect a few mm
and ~1° of error from a printed part; a hand-eye calibration replaces it via
``PERCEPTRONICS_T_FLANGE_CAMERA`` (a UR pose ``[x, y, z, rx, ry, rz]`` of the depth
frame in the flange frame, metres + rotation vector) or :meth:`HandEye.from_pose`.
"""

from __future__ import annotations

import json
import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace

from urctl.pose import Transform, Vec3, pose_inv, pose_trans

# Bracket README §3 (Rev C), one seed per print (``hardware/d435-tool-bracket/
# bracket.py`` VARIANTS; the numbers are ``out/build_info.json`` →
# ``export.variants.<variant>.derived`` and the unit tests hold them to it).
#
# ``eseries`` (UR3e/UR5e/UR10e/UR16e, ISO-50 flange), ARM_ANGLE_DEG = 90 — the
# camera hangs beside the wrist on +Y (the tool-I/O connector side) on a seat
# drafted 5° (CAM_TILT_DEG), so the optical axis tips 5° in toward the flange
# axis: camera axes in flange axes are x_cam = −X, y_cam = (0, −cos 5°, −sin 5°)
# (image-down points at the mounting wall), z_cam = (0, −sin 5°, cos 5°); depth
# origin (left imager) at (17.5, 63.9, 0.6) mm — wall r = 48…51 beside the Ø90
# wrist, the camera leaning about its outer front edge on the tool face (z = 6),
# zero-depth plane 4.3 mm behind the front plate (Intel's URDF: 4.2 glass + 0.1).
_TILT = math.radians(5.0)
_CT, _ST = math.cos(_TILT), math.sin(_TILT)
BRACKET_NOMINAL_ESERIES = Transform.from_axes(
    (-1.0, 0.0, 0.0), (0.0, -_CT, -_ST), (0.0, -_ST, _CT), (0.0175, 0.063922336, 0.000626916)
)
# ``ur20`` (UR20/UR30, ISO-50 + ISO-80 on a Ø96 plate): the wall is clocked to
# 30° (UR20_ARM_ANGLE_DEG, 60° off the M8 socket — Rev C's seat is 95 wide) and
# sits 5 mm further out (wall r = 53…56), so the camera axes are the e-Series
# ones rotated −60° about Z and the depth origin lands at (68.4, 19.3, 0.6) mm.
_C30, _S30 = math.sqrt(3.0) / 2.0, 0.5
BRACKET_NOMINAL_UR20 = Transform.from_axes(
    (-_S30, _C30, 0.0),
    (-_CT * _C30, -_CT * _S30, -_ST),
    (-_ST * _C30, -_ST * _S30, _CT),
    (0.068438494, 0.019305723, 0.000626916),
)
BRACKET_SEEDS: dict[str, Transform] = {"eseries": BRACKET_NOMINAL_ESERIES, "ur20": BRACKET_NOMINAL_UR20}
DEFAULT_BRACKET = "eseries"
# Kept for callers that predate the second print.
BRACKET_NOMINAL = BRACKET_NOMINAL_ESERIES
ENV_BRACKET = "PERCEPTRONICS_BRACKET"
# A saved touch-and-click calibration (perceptronics.calibrate). Env wins over it.
ENV_HANDEYE_FILE = "PERCEPTRONICS_HANDEYE_FILE"
DEFAULT_HANDEYE_DIR = "captures/calibration"


def default_handeye_path(env: Mapping[str, str] | None = None) -> str:
    """``$PERCEPTRONICS_HANDEYE_FILE`` or ``captures/calibration/handeye_<cell>.json``
    (``handeye.json`` when no cell is selected)."""
    env = os.environ if env is None else env
    explicit = env.get(ENV_HANDEYE_FILE, "").strip()
    if explicit:
        return explicit
    cell = env.get("UR_CELL", "").strip()
    return os.path.join(DEFAULT_HANDEYE_DIR, f"handeye_{cell}.json" if cell else "handeye.json")


ENV_T_FLANGE_CAMERA = "PERCEPTRONICS_T_FLANGE_CAMERA"


def seed_calibration_file(path: str | os.PathLike, pose: Sequence[float], *, source: str) -> bool:
    """Write ``pose`` as the saved hand-eye at ``path`` — the file
    :meth:`HandEye.from_env` reads and a calibration's Apply overwrites — unless one is
    already there (a calibration made on this machine is newer than any profile). Returns
    whether it wrote. A pick PC keeps its hand-eye here, not in the environment: an
    environment value would win over every later calibration (``deploy/pi/install.sh``)."""
    vals = [float(v) for v in pose]
    if len(vals) != 6 or not all(math.isfinite(v) for v in vals):
        raise ValueError("a hand-eye pose is 6 finite numbers [x, y, z, rx, ry, rz]")
    target = os.fspath(path)
    if os.path.exists(target):
        return False
    os.makedirs(os.path.dirname(target) or ".", exist_ok=True)
    body = {"flange_to_depth_pose": vals, "source": source}
    tmp = f"{target}.tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(body, fh, indent=2)
    os.replace(tmp, target)
    return True


DEFAULT_STANDOFF_M = 0.10
# What the standoff is measured from:
#   "fingertip" (the default) — the gripper's fingertips, ``tip_m`` along the
#       flange +Z, approached along the tool axis; every move runs with the TCP
#       forced to those fingertips. The controller's active TCP is never trusted
#       for this (the UR3e cell's is a 220 mm training offset the Hand-E doesn't
#       match): the tool offset is the number that decides whether the fingers
#       stop above the part or inside it (Nick, 2026-09-27: "always approach with
#       the fingertips").
#   "flange" — the tool flange, along the camera ray (no tool fitted).
#   "tcp"    — the controller's active TCP, along the camera ray (historical).
APPROACH_REFERENCES = ("fingertip", "flange", "tcp")
DEFAULT_APPROACH_REFERENCE = "fingertip"
ENV_APPROACH_REFERENCE = "PERCEPTRONICS_APPROACH_REFERENCE"
ENV_STANDOFF_M = "PERCEPTRONICS_STANDOFF_M"
# Flange → fingertip length along flange +Z: Robotiq Hand-E 157 mm + the 6 mm
# bracket adapter (measured for pick-cycle, 2026-09-25). A cell with another tool
# sets PERCEPTRONICS_TIP_M.
DEFAULT_TIP_M = 0.163
ENV_TIP_M = "PERCEPTRONICS_TIP_M"


def tip_m_from_env(env: Mapping[str, str] | None = None) -> float:
    """The cell's flange→fingertip length (``PERCEPTRONICS_TIP_M``, default Hand-E + adapter)."""
    env = os.environ if env is None else env
    raw = env.get(ENV_TIP_M, "").strip()
    tip = float(raw) if raw else DEFAULT_TIP_M
    if not (math.isfinite(tip) and 0.0 <= tip <= 0.6):
        raise ValueError(f"{ENV_TIP_M} must be within 0..0.6 m")
    return tip


def fingertip_tcp(tip_m: float) -> list[float]:
    """The fingertip TCP (flange → fingertips) as a UR pose: ``tip_m`` along flange +Z."""
    return [0.0, 0.0, float(tip_m), 0.0, 0.0, 0.0]


def transform_from_extrinsics(ext: Mapping | None) -> Transform:
    """The SDK's ``{rotation: 3x3 row-major, translation: [3]}`` → :class:`Transform`
    (``p_color = R p_depth + t``). ``None`` → identity."""
    if not ext:
        return Transform()
    rot = ext["rotation"]
    rotation = tuple(tuple(float(v) for v in row) for row in rot)
    t = ext["translation"]
    return Transform(rotation, (float(t[0]), float(t[1]), float(t[2])))  # type: ignore[arg-type]


def parse_pose_text(text: str) -> list[float]:
    """``"[x, y, z, rx, ry, rz]"``, ``"x,y,z,rx,ry,rz"`` or ``{"pose": [...]}`` → 6 floats."""
    text = text.strip()
    if not text:
        raise ValueError("empty pose")
    try:
        obj = json.loads(text)
    except json.JSONDecodeError:
        obj = [p for p in text.replace(";", ",").split(",")]
    if isinstance(obj, Mapping):
        obj = obj.get("pose")
    if not isinstance(obj, Sequence) or len(obj) != 6:
        raise ValueError(f"expected 6 numbers [x, y, z, rx, ry, rz], got {text!r}")
    vals = [float(v) for v in obj]
    if not all(math.isfinite(v) for v in vals):
        raise ValueError("pose values must be finite")
    return vals


@dataclass(frozen=True)
class HandEye:
    """``flange_to_depth`` (the calibration, or the bracket seed) and
    ``depth_to_color`` (the SDK extrinsics) — everything needed to move a
    colour-frame point into the flange frame."""

    flange_to_depth: Transform = BRACKET_NOMINAL_ESERIES
    depth_to_color: Transform = Transform()
    source: str = "bracket-nominal:eseries"

    @classmethod
    def from_pose(cls, pose: Sequence[float], *, source: str = "explicit") -> HandEye:
        return cls(Transform.from_pose(pose), Transform(), source)

    @classmethod
    def for_bracket(cls, variant: str = DEFAULT_BRACKET) -> HandEye:
        """The nominal seed for one of the two prints (``eseries`` / ``ur20``)."""
        key = (variant or DEFAULT_BRACKET).strip().lower()
        if key not in BRACKET_SEEDS:
            raise ValueError(f"unknown bracket variant {variant!r}; one of {sorted(BRACKET_SEEDS)}")
        return cls(BRACKET_SEEDS[key], Transform(), f"bracket-nominal:{key}")

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None) -> HandEye:
        """:data:`ENV_T_FLANGE_CAMERA` if set, else a saved calibration file
        (:func:`default_handeye_path`), else the seed for the bracket print
        named by :data:`ENV_BRACKET` (default ``eseries``)."""
        env = os.environ if env is None else env
        raw = env.get(ENV_T_FLANGE_CAMERA, "")
        if raw.strip():
            return cls.from_pose(parse_pose_text(raw), source=f"env:{ENV_T_FLANGE_CAMERA}")
        path = default_handeye_path(env)
        if os.path.isfile(path):
            from .calibrate import load_calibration_pose

            return cls.from_pose(load_calibration_pose(path), source=f"file:{path}")
        return cls.for_bracket(env.get(ENV_BRACKET, DEFAULT_BRACKET))

    @property
    def calibrated(self) -> bool:
        """False while the seed is the printed-bracket nominal (mm + ~1° of error)."""
        return not self.source.startswith("bracket-nominal")

    def with_extrinsics(self, ext: Mapping | None) -> HandEye:
        """Attach the camera's depth→colour extrinsics (from ``describe()``)."""
        return replace(self, depth_to_color=transform_from_extrinsics(ext))

    @property
    def flange_to_color(self) -> Transform:
        # depth_to_color maps depth→colour; we need colour→depth, then depth→flange.
        return self.flange_to_depth.compose(self.depth_to_color.inverse())

    def camera_to_flange(self, point_cam: Sequence[float]) -> Vec3:
        return self.flange_to_color.apply(point_cam)

    def as_dict(self) -> dict:
        return {
            "source": self.source,
            "calibrated": self.calibrated,
            "flange_to_depth_pose": self.flange_to_depth.to_pose(),
            "depth_to_color_translation": list(self.depth_to_color.translation),
            "flange_to_color_pose": self.flange_to_color.to_pose(),
        }


def _check_point(point: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(point) != 3:
        raise ValueError(f"{name} must be [x, y, z], got {len(point)} values")
    vals = tuple(float(v) for v in point)
    if not all(math.isfinite(v) for v in vals):
        raise ValueError(f"{name} must be finite")
    return vals  # type: ignore[return-value]


def locate(
    handeye: HandEye,
    flange_pose: Sequence[float],
    point_cam: Sequence[float],
    *,
    tcp_pose: Sequence[float],
    standoff_m: float = DEFAULT_STANDOFF_M,
    max_reach_m: float | None = None,
    reference: str = DEFAULT_APPROACH_REFERENCE,
    tcp_offset: Sequence[float] | None = None,
    tip_m: float = DEFAULT_TIP_M,
) -> dict:
    """Turn a colour-camera point into base coordinates and an **approach pose**
    for the tool: the TCP placed ``standoff_m`` short of the point along the
    camera's viewing ray, keeping the tool's current orientation.

    ``flange_pose``/``tcp_pose`` are the live UR poses (base frame). The
    approach keeps the current rotation vector, so the tool comes in the way
    it was already pointing — with the camera looking along the flange +Z
    axis that is straight down the tool axis. Returns every intermediate
    frame so the cockpit can show its work before anything moves.

    ``reference`` picks what sits ``standoff_m`` short of the point:

    * ``"fingertip"`` — the fingertips (``tip_m`` along flange +Z) sit
      ``standoff_m`` short of the point **along the tool axis** (the approach
      pick-cycle verified on the UR3e); ``approach_pose`` is then the pose of the
      *fingertip TCP* (``tcp`` in the result) and every move runs with that TCP.
    * ``"flange"`` — the flange, along the camera ray; needs ``tcp_offset`` (the
      live active-TCP offset) to express it as the active-TCP pose.
    * ``"tcp"`` — the active TCP, along the camera ray.

    The move always keeps the tool's current orientation, so ``flange_target_pose``
    is reported whenever it can be computed.

    ``max_reach_m`` (the arm's datasheet reach — ``SafetyEnvelope.max_reach``)
    adds ``reachable`` / ``approach_distance_m`` so the verdict is on screen
    *before* Move: an approach beyond the arm's reach is a move the envelope
    will refuse, and a point beyond it can't be picked from this base at all.
    """
    p_cam = _check_point(point_cam, "point_cam")
    if not math.isfinite(standoff_m) or standoff_m < 0.0 or standoff_m > 1.0:
        raise ValueError("standoff_m must be within 0..1 m")
    base_from_flange = Transform.from_pose(flange_pose)
    tcp = [float(v) for v in tcp_pose]
    if len(tcp) != 6:
        raise ValueError("tcp_pose must have 6 elements")
    if reference not in APPROACH_REFERENCES:
        raise ValueError(f"reference must be one of {APPROACH_REFERENCES}, not {reference!r}")
    offset = None
    if tcp_offset is not None:
        offset = [float(v) for v in tcp_offset]
        if len(offset) != 6 or not all(math.isfinite(v) for v in offset):
            raise ValueError("tcp_offset must be 6 finite numbers")
    if reference == "flange" and offset is None:
        raise ValueError("reference='flange' needs the active tcp_offset")
    if not (math.isfinite(tip_m) and 0.0 <= tip_m <= 0.6):
        raise ValueError("tip_m must be within 0..0.6 m")
    p_flange = handeye.camera_to_flange(p_cam)
    p_base = base_from_flange.apply(p_flange)
    ray_base = base_from_flange.rotate(handeye.flange_to_color.rotate((0.0, 0.0, 1.0)))
    short = tuple(p - standoff_m * r for p, r in zip(p_base, ray_base, strict=True))
    flange_now = [float(v) for v in flange_pose]
    tool_tcp = None
    if reference == "fingertip":
        tool_tcp = fingertip_tcp(tip_m)
        axis = base_from_flange.rotate((0.0, 0.0, 1.0))  # the tool axis, pointing at the part
        tip = [p - standoff_m * a for p, a in zip(p_base, axis, strict=True)]
        target = [*tip, *flange_now[3:]]  # the fingertip TCP's pose (no rotation vs the flange)
        flange_target = pose_trans(target, pose_inv(tool_tcp))
    elif reference == "flange":
        flange_target = [*short, *flange_now[3:]]
        target = pose_trans(flange_target, offset)  # the TCP pose that puts the flange there
    else:
        target = [*short, *tcp[3:]]
        flange_target = pose_trans(target, pose_inv(offset)) if offset is not None else None
    approach = tuple(target[:3])
    approach_dist = math.sqrt(sum(v * v for v in approach))
    point_dist = math.sqrt(sum(v * v for v in p_base))
    # Reach is judged on the flange whenever the moves override the TCP (flange,
    # fingertip): the datasheet radius is to the flange, and the active TCP pose is
    # a virtual point that can sit nearer the base than the flange does.
    commanded = flange_target if reference in ("flange", "fingertip") else target
    commanded_dist = math.sqrt(sum(v * v for v in commanded[:3]))
    reachable = None if max_reach_m is None else bool(commanded_dist <= max_reach_m)
    return {
        "approach_distance_m": approach_dist,
        "commanded_distance_m": commanded_dist,
        "point_distance_m": point_dist,
        "max_reach_m": None if max_reach_m is None else float(max_reach_m),
        "reachable": reachable,
        "reference": reference,
        "tool_tcp": tool_tcp,
        "tip_m": float(tip_m) if reference == "fingertip" else None,
        "flange_target_pose": flange_target,
        "tcp_offset": offset,
        "point_cam_m": list(p_cam),
        "point_flange_m": list(p_flange),
        "point_base_m": list(p_base),
        "view_ray_base": list(ray_base),
        "standoff_m": float(standoff_m),
        "approach_pose": target,
        "flange_pose": [float(v) for v in flange_pose],
        "tcp_pose": tcp,
        "handeye": handeye.as_dict(),
    }
