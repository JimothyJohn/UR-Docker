"""ChArUco plate observations: camera pose from one frame, hand-eye from views.

Pairs with ``hardware/charuco-board/`` (``out/board.json`` carries the pattern
and the plate↔pattern transform). Frames (`docs/mono-scan.md` §3):

* **plate** — the physical plate: origin at dimple A, +X toward B, +Y toward C,
  +Z out of the printed face. ``T_base_plate`` comes from three probe touches
  (:func:`plate_from_touches`).
* **pattern** — OpenCV's CharucoBoard frame (top-left of the print, x right,
  y down, z into the board); ``T_plate_pattern`` from ``board.json``.
* **camera** — the colour camera. PnP gives ``T_cam_pattern``.

So ``T_base_cam = T_base_plate · T_plate_pattern · inv(T_cam_pattern)`` and,
with the flange pose from the robot, each view yields a closed-form
``T_flange_cam = inv(T_base_flange) · T_base_cam`` — averaged over views by
:func:`average_transforms`. OpenCV (``cv2.aruco``) required for detection.
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from urctl.pose import Transform, matrix_to_rotvec, rotvec_to_matrix

from .rgbd import Intrinsics

DEFAULT_BOARD_JSON = "hardware/charuco-board/out/board.json"


@dataclass(frozen=True)
class BoardSpec:
    squares: tuple[int, int]
    square_m: float
    marker_m: float
    dictionary: str
    plate_to_pattern: Transform
    source: str = ""

    @classmethod
    def load(cls, path: str | Path = DEFAULT_BOARD_JSON) -> BoardSpec:
        d = json.loads(Path(path).read_text())
        p = d["pattern"]
        tp = d["T_plate_pattern"]
        rot = tuple(tuple(float(v) for v in row) for row in tp["R"])
        t = tuple(float(v) / 1000.0 for v in tp["t_mm"])
        return cls(
            (int(p["squares"][0]), int(p["squares"][1])),
            float(p["square_m"]),
            float(p["marker_m"]),
            str(p["dictionary"]),
            Transform(rot, t),  # type: ignore[arg-type]
            str(path),
        )


def plate_from_touches(a: Sequence[float], b: Sequence[float], c: Sequence[float]) -> Transform:
    """``T_base_plate`` from the three dimple touches (base-frame points):
    origin A, +X toward B, +Y toward C (orthogonalised), +Z = X × Y."""
    ax = [float(b[i]) - float(a[i]) for i in range(3)]
    n = math.sqrt(sum(v * v for v in ax))
    if n < 1e-6:
        raise ValueError("A and B coincide")
    ax = [v / n for v in ax]
    ay = [float(c[i]) - float(a[i]) for i in range(3)]
    d = sum(ax[i] * ay[i] for i in range(3))
    ay = [ay[i] - d * ax[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in ay))
    if n < 1e-6:
        raise ValueError("A, B, C are collinear")
    ay = [v / n for v in ay]
    az = (ax[1] * ay[2] - ax[2] * ay[1], ax[2] * ay[0] - ax[0] * ay[2], ax[0] * ay[1] - ax[1] * ay[0])
    return Transform.from_axes(ax, ay, az, [float(v) for v in a[:3]])


class CharucoPnP:
    """Detect the board in a grayscale frame and solve the camera pose."""

    def __init__(self, spec: BoardSpec, intr: Intrinsics):
        import cv2

        self.spec = spec
        self.intr = intr
        aruco = cv2.aruco
        dic = aruco.getPredefinedDictionary(getattr(aruco, spec.dictionary))
        self.board = aruco.CharucoBoard(spec.squares, spec.square_m, spec.marker_m, dic)
        self.detector = aruco.CharucoDetector(self.board)
        self.K = [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]]
        self.dist = (
            list(intr.coeffs) if intr.model in ("brown_conrady", "inverse_brown_conrady") else [0.0] * 5
        )

    def solve(self, gray, *, min_corners: int = 6) -> dict | None:
        """``{T_cam_pattern, corners, rms_px}`` or None when the board isn't seen."""
        import cv2
        import numpy as np

        cc, ci, _, _ = self.detector.detectBoard(gray)
        if cc is None or ci is None or len(cc) < min_corners:
            return None
        obj, img = self.board.matchImagePoints(cc, ci)
        if obj is None or len(obj) < min_corners:
            return None
        K = np.asarray(self.K)
        dist = np.asarray(self.dist)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        rms = float(np.sqrt(((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2).sum(axis=1).mean()))
        rv = rvec.reshape(3).tolist()
        t = tvec.reshape(3).tolist()
        return {
            "T_cam_pattern": Transform(rotvec_to_matrix(rv), tuple(t)),
            "corners": int(len(obj)),
            "rms_px": rms,
        }


def camera_in_base(t_base_plate: Transform, spec: BoardSpec, t_cam_pattern: Transform) -> Transform:
    return t_base_plate.compose(spec.plate_to_pattern).compose(t_cam_pattern.inverse())


def handeye_from_view(t_base_flange: Transform, t_base_cam: Transform) -> Transform:
    return t_base_flange.inverse().compose(t_base_cam)


def average_transforms(transforms: Sequence[Transform]) -> dict:
    """Mean translation + chordal-mean rotation (numpy SVD), with the per-view
    spread so a bad view shows up."""
    import numpy as np

    if not transforms:
        raise ValueError("no transforms")
    ts = np.array([t.translation for t in transforms])
    rs = np.array([t.rotation for t in transforms])
    mean_t = ts.mean(axis=0)
    u, _, vt = np.linalg.svd(rs.mean(axis=0))
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    mean = Transform(tuple(tuple(float(v) for v in row) for row in r), tuple(float(v) for v in mean_t))  # type: ignore[arg-type]
    d_t = [float(np.linalg.norm(t - mean_t)) for t in ts]
    d_r = []
    for rot in rs:
        rel = r.T @ rot
        d_r.append(math.degrees(np.linalg.norm(matrix_to_rotvec(tuple(tuple(x) for x in rel)))))
    return {
        "transform": mean,
        "pose": mean.to_pose(),
        "views": len(transforms),
        "translation_spread_m": max(d_t),
        "rotation_spread_deg": max(d_r),
        "per_view_translation_m": d_t,
        "per_view_rotation_deg": d_r,
    }


def measured_camera_positions(
    frames: Sequence[Any], pnp: CharucoPnP, t_base_plate: Transform
) -> list[tuple[float, list[float]]]:
    """For :func:`perception.latency.fit_latency`: ``[(host_t, camera xyz in base)]``
    over the frames (``.gray``, ``.host_t``) where the board is detected."""
    out = []
    for f in frames:
        r = pnp.solve(f.gray)
        if r is None:
            continue
        cam = camera_in_base(t_base_plate, pnp.spec, r["T_cam_pattern"])
        out.append((f.host_t, list(cam.translation)))
    return out
