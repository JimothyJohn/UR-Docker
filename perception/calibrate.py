"""Touch-and-click hand-eye calibration — replace the bracket's nominal
``T_flange_camera`` with a measured one, using nothing but the robot, the
camera, and a mark on the table.

The procedure (the cockpit's **Calibrate** section and the ``cal_*`` tools
drive it; :class:`CalibrationSession` is the state):

1. **Touch.** Freedrive the tool tip onto a mark and *record the mark*: the
   live TCP position is the mark in the **base** frame (``p_mark``). Back off.
2. **Look.** From 4–6 different poses (rotate the wrist, change height and
   angle — the more varied the better) click the same mark in the cockpit and
   *add a view*: each view stores the flange pose ``T_base_flange_i`` and the
   clicked pixel's camera-frame point ``p_cam_i`` (colour frame, metres).
3. **Solve.** Find ``T_flange_color`` = (R, t) minimising
   ``Σ_i |T_base_flange_i · (R p_cam_i + t) − p_mark|²`` — six unknowns, three
   equations per view, Levenberg–Marquardt seeded from the bracket nominal.
   The result is reported as ``T_flange_depth`` (what
   ``PERCEPTION_T_FLANGE_CAMERA`` expects: the depth frame in the flange frame)
   by composing with the camera's own depth→colour extrinsics.

Without a touch (``mark_base=None``) the mark's base position is solved
jointly (nine unknowns; needs ≥ 4 views) — handy when the tip isn't a known
TCP, at the cost of some conditioning.

Everything is stdlib: the normal equations are at most 9×9, solved by Gaussian
elimination; the Jacobian is central differences. The synthetic test recovers
a known transform from noisy views to well under a millimetre.
"""

from __future__ import annotations

import json
import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from urctl.pose import Transform, matrix_to_rotvec, rotvec_to_matrix

from .handeye import ENV_T_FLANGE_CAMERA, HandEye, transform_from_extrinsics

MIN_VIEWS_WITH_MARK = 3
MIN_VIEWS_WITHOUT_MARK = 4
MAX_ITER = 200
RMS_WARN_M = 0.005  # a printed bracket + a click should land well under this
MIN_ROTATION_DIVERSITY_DEG = 10.0


class CalibrationError(ValueError):
    """Not enough / degenerate data, or the solve did not converge."""


@dataclass
class View:
    flange_pose: list[float]  # T_base_flange as a UR pose
    point_cam: list[float]  # the mark in the colour-camera frame, metres
    pixel: list[int] | None = None
    seq: int | None = None
    ts: float = field(default_factory=time.time)

    def as_dict(self) -> dict:
        return {
            "flange_pose": self.flange_pose,
            "point_cam": self.point_cam,
            "pixel": self.pixel,
            "seq": self.seq,
            "ts": self.ts,
        }


# ----- small dense linear algebra -----------------------------------------------------


def _solve(a: list[list[float]], b: list[float]) -> list[float]:
    """Gaussian elimination with partial pivoting for a small square system."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-15:
            raise CalibrationError("singular normal equations (views are degenerate — vary the poses more)")
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, n):
            f = m[r][col] / m[col][col]
            if f:
                for c in range(col, n + 1):
                    m[r][c] -= f * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


def _residuals(params: Sequence[float], views: Sequence[View], mark: Sequence[float] | None) -> list[float]:
    rv, t = params[0:3], params[3:6]
    p_mark = params[6:9] if mark is None else mark
    r_fc = rotvec_to_matrix(rv)
    t_fc = (t[0], t[1], t[2])
    out: list[float] = []
    for v in views:
        base_from_flange = Transform.from_pose(v.flange_pose)
        p_flange = Transform(r_fc, t_fc).apply(v.point_cam)
        p_base = base_from_flange.apply(p_flange)
        out.extend((p_base[0] - p_mark[0], p_base[1] - p_mark[1], p_base[2] - p_mark[2]))
    return out


def _lm(
    params: list[float], views: Sequence[View], mark: Sequence[float] | None
) -> tuple[list[float], float, int]:
    """Levenberg–Marquardt on the residual vector; returns (params, cost, iterations)."""
    n = len(params)
    lam = 1e-3
    r = _residuals(params, views, mark)
    cost = sum(x * x for x in r)
    it = 0
    step = 0.0
    while it < MAX_ITER:
        it += 1
        # numeric Jacobian (central differences)
        jac: list[list[float]] = []
        cols = []
        for k in range(n):
            h = 1e-6 if k < 3 or k >= 6 else 1e-6
            p_plus = list(params)
            p_minus = list(params)
            p_plus[k] += h
            p_minus[k] -= h
            rp = _residuals(p_plus, views, mark)
            rm = _residuals(p_minus, views, mark)
            cols.append([(a - b) / (2 * h) for a, b in zip(rp, rm, strict=True)])
        m = len(r)
        jac = [[cols[k][i] for k in range(n)] for i in range(m)]
        jtj = [[sum(jac[i][a] * jac[i][b] for i in range(m)) for b in range(n)] for a in range(n)]
        jtr = [sum(jac[i][a] * r[i] for i in range(m)) for a in range(n)]
        improved = False
        for _ in range(10):
            a = [[jtj[i][j] + (lam * jtj[i][i] if i == j else 0.0) for j in range(n)] for i in range(n)]
            try:
                delta = _solve(a, [-x for x in jtr])
            except CalibrationError:
                lam *= 10
                continue
            trial = [p + d for p, d in zip(params, delta, strict=True)]
            r_trial = _residuals(trial, views, mark)
            cost_trial = sum(x * x for x in r_trial)
            if cost_trial < cost:
                params, r, cost = trial, r_trial, cost_trial
                lam = max(lam / 10, 1e-9)
                improved = True
                step = math.sqrt(sum(d * d for d in delta))
                break
            lam *= 10
        if not improved:
            break
        if step < 1e-10:
            break
    return params, cost, it


# ----- the session -----------------------------------------------------------------------


@dataclass
class CalibrationSession:
    """Mark + views + the seed, with ``solve()`` and (de)serialisation."""

    seed: HandEye = field(default_factory=HandEye)
    mark_base: list[float] | None = None
    views: list[View] = field(default_factory=list)
    extrinsics: Mapping | None = None  # depth→colour, from camera.describe()
    result: dict | None = None

    def record_mark(self, tcp_position: Sequence[float]) -> dict:
        vals = [float(v) for v in tcp_position[:3]]
        if len(vals) != 3 or not all(math.isfinite(v) for v in vals):
            raise CalibrationError("mark must be [x, y, z] metres in the base frame")
        self.mark_base = vals
        self.result = None
        return self.as_dict()

    def add_view(
        self, flange_pose: Sequence[float], point_cam: Sequence[float], *, pixel=None, seq=None
    ) -> dict:
        fp = [float(v) for v in flange_pose]
        pc = [float(v) for v in point_cam]
        if len(fp) != 6 or len(pc) != 3 or not all(math.isfinite(v) for v in fp + pc):
            raise CalibrationError("a view needs a 6-vector flange pose and a 3-vector camera point")
        if pc[2] <= 0.0:
            raise CalibrationError("camera point has no depth (z ≤ 0) — click a pixel with valid depth")
        self.views.append(View(fp, pc, list(pixel) if pixel else None, seq))
        self.result = None
        return self.as_dict()

    def remove_view(self, index: int) -> dict:
        if not (0 <= index < len(self.views)):
            raise CalibrationError(f"no view {index}")
        del self.views[index]
        self.result = None
        return self.as_dict()

    def reset(self) -> dict:
        self.mark_base = None
        self.views.clear()
        self.result = None
        return self.as_dict()

    # -- the solve -----------------------------------------------------------------------

    def rotation_diversity_deg(self) -> float:
        """Largest angle between any two views' flange orientations — the
        observability of the camera *translation* comes from this."""
        rots = [Transform.from_pose(v.flange_pose).rotation for v in self.views]
        best = 0.0
        for i in range(len(rots)):
            for j in range(i + 1, len(rots)):
                rel = Transform(rots[i]).inverse().compose(Transform(rots[j])).rotation
                ang = math.degrees(math.sqrt(sum(c * c for c in matrix_to_rotvec(rel))))
                best = max(best, ang)
        return best

    def solve(self) -> dict:
        need = MIN_VIEWS_WITH_MARK if self.mark_base is not None else MIN_VIEWS_WITHOUT_MARK
        if len(self.views) < need:
            raise CalibrationError(
                f"need at least {need} views {'with' if self.mark_base is not None else 'without'} a touched "
                f"mark, have {len(self.views)}"
            )
        warnings: list[str] = []
        diversity = self.rotation_diversity_deg()
        if diversity < MIN_ROTATION_DIVERSITY_DEG:
            warnings.append(
                f"flange orientations differ by only {diversity:.1f}°; the camera offset is poorly "
                "observed — add views with the wrist rotated / tilted"
            )
        seed_fc = self.seed.flange_to_color
        params = [*matrix_to_rotvec(seed_fc.rotation), *seed_fc.translation]
        if self.mark_base is None:
            # seed the mark from the first view through the seed transform
            v0 = self.views[0]
            p0 = Transform.from_pose(v0.flange_pose).apply(seed_fc.apply(v0.point_cam))
            params += list(p0)
        params, cost, iters = _lm(params, self.views, self.mark_base)
        res = _residuals(params, self.views, self.mark_base)
        per_view = [math.sqrt(sum(x * x for x in res[3 * i : 3 * i + 3])) for i in range(len(self.views))]
        rms = math.sqrt(cost / len(self.views)) if self.views else 0.0
        if rms > RMS_WARN_M:
            warnings.append(
                f"RMS residual {rms * 1000:.1f} mm is high — a bad click or a moved mark? drop the worst view"
            )
        flange_to_color = Transform(rotvec_to_matrix(params[0:3]), (params[3], params[4], params[5]))
        depth_to_color = transform_from_extrinsics(self.extrinsics)
        flange_to_depth = flange_to_color.compose(depth_to_color)
        seed_pose = self.seed.flange_to_depth.to_pose()
        new_pose = flange_to_depth.to_pose()
        delta_mm = [(a - b) * 1000.0 for a, b in zip(new_pose[:3], seed_pose[:3], strict=True)]
        self.result = {
            "ok": True,
            "flange_to_depth_pose": new_pose,
            "flange_to_color_pose": flange_to_color.to_pose(),
            "mark_base": list(self.mark_base) if self.mark_base is not None else list(params[6:9]),
            "mark_solved": self.mark_base is None,
            "views": len(self.views),
            "rms_m": rms,
            "per_view_residual_m": per_view,
            "iterations": iters,
            "rotation_diversity_deg": diversity,
            "seed_pose": seed_pose,
            "delta_from_seed_mm": delta_mm,
            "warnings": warnings,
            "env_line": f'{ENV_T_FLANGE_CAMERA}="{json.dumps([round(v, 6) for v in new_pose])}"',
        }
        return self.result

    def handeye(self) -> HandEye:
        """The solved :class:`HandEye` (raises if not solved)."""
        if not self.result:
            raise CalibrationError("not solved yet")
        return HandEye.from_pose(
            self.result["flange_to_depth_pose"], source="calibrated:touch-and-click"
        ).with_extrinsics(self.extrinsics)

    # -- persistence ----------------------------------------------------------------------

    def as_dict(self) -> dict:
        return {
            "mark_base": self.mark_base,
            "views": [v.as_dict() for v in self.views],
            "seed": self.seed.as_dict(),
            "extrinsics": dict(self.extrinsics) if self.extrinsics else None,
            "result": self.result,
            "min_views": MIN_VIEWS_WITH_MARK if self.mark_base is not None else MIN_VIEWS_WITHOUT_MARK,
        }

    def save(self, path: str | Path) -> Path:
        """Write the solved calibration (and the raw views, for re-solving)."""
        if not self.result:
            raise CalibrationError("nothing to save — solve first")
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        body = {
            "flange_to_depth_pose": self.result["flange_to_depth_pose"],
            "source": "touch-and-click",
            "saved_at": time.time(),
            "rms_m": self.result["rms_m"],
            "session": self.as_dict(),
        }
        p.write_text(json.dumps(body, indent=2), encoding="utf-8")
        return p


def load_calibration_pose(path: str | Path) -> list[float]:
    body = json.loads(Path(path).read_text(encoding="utf-8"))
    pose = body["flange_to_depth_pose"]
    if len(pose) != 6:
        raise ValueError(f"{path}: flange_to_depth_pose must have 6 numbers")
    return [float(v) for v in pose]
