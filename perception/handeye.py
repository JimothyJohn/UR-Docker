"""Where the camera sits on the robot, and how a camera-frame point becomes a
base-frame target the robot can move to.

Frames (all right-handed):

* **colour camera** — what :meth:`perception.rgbd.RgbdFrame.point_at` and the
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

:data:`BRACKET_NOMINAL` is a *seed*: it is derived from the bracket geometry
(``hardware/d435-tool-bracket/README.md`` §3, ``ARM_ANGLE_DEG = 0``) and the
camera's published imager position, not from a calibration. Expect a few mm
and ~1° of error from a printed part; a hand-eye calibration replaces it via
``PERCEPTION_T_FLANGE_CAMERA`` (a UR pose ``[x, y, z, rx, ry, rz]`` of the depth
frame in the flange frame, metres + rotation vector) or :meth:`HandEye.from_pose`.
"""

from __future__ import annotations

import json
import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace

from urctl.pose import Transform, Vec3

# Bracket README §3 (Rev B), ARM_ANGLE_DEG = 0: camera axes in flange axes are
# x_cam = +Y, y_cam = −X (image-down points at the mounting wall), z_cam = +Z
# (optical axis out of the flange); depth origin (left imager) at
# (71.5, −17.5, 3.7) mm — the camera hangs beside the wrist with its front
# plate flush with the adapter's tool face (z = 8), zero-depth plane 4.3 mm
# behind it (Intel's URDF: 4.2 glass + 0.1).
BRACKET_NOMINAL = Transform.from_axes(
    (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0715, -0.0175, 0.0037)
)
ENV_T_FLANGE_CAMERA = "PERCEPTION_T_FLANGE_CAMERA"
DEFAULT_STANDOFF_M = 0.10


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

    flange_to_depth: Transform = BRACKET_NOMINAL
    depth_to_color: Transform = Transform()
    source: str = "bracket-nominal"

    @classmethod
    def from_pose(cls, pose: Sequence[float], *, source: str = "explicit") -> HandEye:
        return cls(Transform.from_pose(pose), Transform(), source)

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None) -> HandEye:
        """:data:`ENV_T_FLANGE_CAMERA` if set (a calibration), else the bracket seed."""
        env = os.environ if env is None else env
        raw = env.get(ENV_T_FLANGE_CAMERA, "")
        if raw.strip():
            return cls.from_pose(parse_pose_text(raw), source=f"env:{ENV_T_FLANGE_CAMERA}")
        return cls()

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
) -> dict:
    """Turn a colour-camera point into base coordinates and an **approach pose**
    for the tool: the TCP placed ``standoff_m`` short of the point along the
    camera's viewing ray, keeping the tool's current orientation.

    ``flange_pose``/``tcp_pose`` are the live UR poses (base frame). The
    approach keeps the current rotation vector, so the tool comes in the way
    it was already pointing — with the camera looking along the flange +Z
    axis that is straight down the tool axis. Returns every intermediate
    frame so the cockpit can show its work before anything moves.
    """
    p_cam = _check_point(point_cam, "point_cam")
    if not math.isfinite(standoff_m) or standoff_m < 0.0 or standoff_m > 1.0:
        raise ValueError("standoff_m must be within 0..1 m")
    base_from_flange = Transform.from_pose(flange_pose)
    tcp = [float(v) for v in tcp_pose]
    if len(tcp) != 6:
        raise ValueError("tcp_pose must have 6 elements")
    p_flange = handeye.camera_to_flange(p_cam)
    p_base = base_from_flange.apply(p_flange)
    ray_base = base_from_flange.rotate(handeye.flange_to_color.rotate((0.0, 0.0, 1.0)))
    approach = tuple(p - standoff_m * r for p, r in zip(p_base, ray_base, strict=True))
    target = [*approach, *tcp[3:]]
    return {
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
