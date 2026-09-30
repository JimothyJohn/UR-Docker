"""Mark-less hand-eye calibration by orbiting a block — ``perceptronics calibrate``.

A gripper on the flange means nothing can touch a mark, so the mark is a
block's top-face centre and the wrist orbits it (the 2026-09-25 procedure,
``docs/realsense.md`` §Hand-eye without a mark, folded in from the
``scripts/pilot/orbit_cal*.py`` scratch scripts):

1. **Find the mark.** From where the arm is, the white block nearest the
   image centre; its top face is fitted as a plane (:func:`pickcycle.top_face`)
   and its 3-D centroid mapped to the base frame through the *seed* hand-eye
   (the cockpit's current one — bracket nominal or an old solve).
2. **Plan the orbit.** Three ranges (0.21 / 0.30 / 0.40 m); at each, the pose
   that puts the mark on the optical axis, eight rotations about the mark
   (±tilt about base X and Y, ±yaw about Z, two diagonals) and four lateral
   shifts. Range diversity is what makes the camera offset and the mark stop
   trading off against each other.
3. **Look.** Move to every planned pose that keeps the mark inside the frame
   (predicted through the current solve) and within reach; find the block
   again **by identity** — the top face whose 3-D centroid is nearest the
   predicted mark, gated — and click its centroid into the cockpit's
   calibration session (``POST /api/cal/view``). After four views the
   prediction switches from the seed to the running solve.
4. **Solve and trim.** ``POST /api/cal/solve``; drop the worst view while any
   residual is above ``trim_mm`` and re-solve. ``--apply`` puts the result in
   force in the cockpit and saves ``captures/calibration/handeye_<cell>.json``.

Motion goes straight over Primary through :class:`urctl.robot.Robot`
(envelope-checked, ``tcp=[0]*6`` so the pose is the flange), the cockpit does
vision only — the same split as ``pick-cycle``. A protective stop unlocks and
moves on to the next view; a refused move is skipped; Ctrl-C stops the running
program and returns 130.
"""

from __future__ import annotations

import json
import math
import sys
import time
import urllib.error
from collections.abc import Sequence
from dataclasses import dataclass, field

from urctl.config import RobotConfig
from urctl.pose import Transform, matrix_to_rotvec, rotvec_to_matrix
from urctl.robot import Robot

from .calibrate import MIN_VIEWS_WITHOUT_MARK, CalibrationError, CalibrationSession
from .handeye import HandEye
from .pickcycle import DEFAULT_COCKPIT, Cockpit, CockpitError, top_face, white_blobs

DEFAULT_RANGES_M = (0.21, 0.30, 0.40)
DEFAULT_TILT_DEG = (12.0, 15.0, 15.0)
DEFAULT_YAW_DEG = (25.0, 30.0, 30.0)
DEFAULT_SHIFT_M = (0.03, 0.05, 0.07)
DIAGONAL_AXES = ((0.55, 0.0, 0.83), (0.0, -0.55, 0.83))


def orbit_pose(
    flange: Sequence[float], mark: Sequence[float], axis: Sequence[float], deg: float
) -> list[float]:
    """Rotate the flange pose about ``axis`` (base frame, through ``mark``) by ``deg``."""
    norm = math.sqrt(sum(a * a for a in axis)) or 1.0
    rv = [a / norm * math.radians(deg) for a in axis]
    rot = Transform(rotvec_to_matrix(rv), (0.0, 0.0, 0.0))
    t = Transform.from_pose(flange)
    rel = [t.translation[i] - mark[i] for i in range(3)]
    p = rot.apply(rel)
    new_rot = rot.compose(Transform(t.rotation, (0.0, 0.0, 0.0)))
    return [p[0] + mark[0], p[1] + mark[1], p[2] + mark[2], *matrix_to_rotvec(new_rot.rotation)]


def centred_pose(
    track: Transform, mark: Sequence[float], orientation: Sequence[float], range_m: float
) -> list[float]:
    """The flange pose with ``orientation``'s rotation that puts ``mark`` on the
    camera's optical axis ``range_m`` away (``track`` = flange→colour)."""
    r = Transform(Transform.from_pose(orientation).rotation, (0.0, 0.0, 0.0))
    cam_pt = track.apply((0.0, 0.0, range_m))  # the mark in the flange frame
    off = r.rotate(cam_pt)
    return [mark[0] - off[0], mark[1] - off[1], mark[2] - off[2], *list(orientation)[3:]]


def predict_pixel(
    track: Transform, flange: Sequence[float], mark: Sequence[float], K: dict
) -> tuple[float, float, list[float]] | None:
    """Where ``mark`` lands in the colour image from ``flange`` — ``(u, v, p_cam)``,
    or None when it is behind the camera."""
    p_cam = track.inverse().apply(Transform.from_pose(flange).inverse().apply(mark))
    if p_cam[2] <= 0.0:
        return None
    return (
        K["ppx"] + K["fx"] * p_cam[0] / p_cam[2],
        K["ppy"] + K["fy"] * p_cam[1] / p_cam[2],
        list(p_cam),
    )


def plan_orbit(
    track: Transform,
    mark: Sequence[float],
    orientation: Sequence[float],
    *,
    ranges_m: Sequence[float] = DEFAULT_RANGES_M,
    tilt_deg: Sequence[float] = DEFAULT_TILT_DEG,
    yaw_deg: Sequence[float] = DEFAULT_YAW_DEG,
    shift_m: Sequence[float] = DEFAULT_SHIFT_M,
    diagonal_deg: float = 22.0,
) -> list[dict]:
    """``[{name, pose, range_m}]`` — centred, eight orbits and four shifts per range."""
    plan: list[dict] = []
    for i, rng in enumerate(ranges_m):
        tilt = tilt_deg[min(i, len(tilt_deg) - 1)]
        yaw = yaw_deg[min(i, len(yaw_deg) - 1)]
        sh = shift_m[min(i, len(shift_m) - 1)]
        home = centred_pose(track, mark, orientation, rng)
        tag = f"@{rng:.2f}"
        plan.append({"name": "centre" + tag, "pose": home, "range_m": rng})
        for name, axis, deg in (
            ("X+", (1, 0, 0), tilt),
            ("X-", (1, 0, 0), -tilt),
            ("Y+", (0, 1, 0), tilt),
            ("Y-", (0, 1, 0), -tilt),
            ("Z+", (0, 0, 1), yaw),
            ("Z-", (0, 0, 1), -yaw),
            ("XZ", DIAGONAL_AXES[0], diagonal_deg),
            ("YZ", DIAGONAL_AXES[1], diagonal_deg),
        ):
            plan.append({"name": name + tag, "pose": orbit_pose(home, mark, axis, deg), "range_m": rng})
        for name, sx, sy in (("sX+", sh, 0.0), ("sX-", -sh, 0.0), ("sY+", 0.0, sh), ("sY-", 0.0, -sh)):
            plan.append({"name": name + tag, "pose": [home[0] + sx, home[1] + sy, *home[2:]], "range_m": rng})
    return plan


@dataclass
class OrbitCalibration:
    cockpit: Cockpit = field(default_factory=Cockpit)
    robot: Robot | None = None
    ranges_m: Sequence[float] = DEFAULT_RANGES_M
    tilt_deg: Sequence[float] = DEFAULT_TILT_DEG
    yaw_deg: Sequence[float] = DEFAULT_YAW_DEG
    shift_m: Sequence[float] = DEFAULT_SHIFT_M
    diagonal_deg: float = 22.0
    gate_mm: float = 40.0  # identity gate once a solve exists
    first_gate_mm: float = 120.0  # before the first solve the seed may be centimetres off
    trim_mm: float = 9.0
    margin_frac: float = 0.12  # the predicted mark must be this fraction of the frame inside its edges
    min_blob_px: int = 400
    velocity: float = 0.08
    settle_s: float = 0.7
    reach_margin_m: float = 0.05
    dry_run: bool = False
    log: list[dict] = field(default_factory=list)
    t0: float = field(default_factory=time.time)
    say_fn: object = None
    # state
    track: Transform | None = None
    mark: list[float] | None = None
    local: CalibrationSession | None = None

    # -- bookkeeping -----------------------------------------------------------------------
    def say(self, text: str) -> None:
        ev = {"t": round(time.time() - self.t0, 2), "text": text}
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
        return list(r["flange_pose"])

    def _move(self, pose: Sequence[float]) -> dict:
        if self.dry_run:
            return {"ok": True, "landed": list(pose), "dry_run": True}
        if self.robot is not None:
            return self.robot.move_tcp(list(pose), velocity=self.velocity, tcp=[0.0] * 6)
        return self.cockpit.post(
            "/api/robot/move", {"pose": list(pose), "velocity": self.velocity, "tcp": [0] * 6}
        )

    def _bring_up(self) -> None:
        if self.robot is not None:
            self.robot.bring_up()
        else:
            self.cockpit.post("/api/robot/bring_up")

    def _reach_limit(self) -> float:
        if self.robot is None:
            return 0.0
        try:
            probe = getattr(self.robot, "_ensure_reach", None)
            if callable(probe):
                probe()
            reach = self.robot.max_reach
            return float(reach() if callable(reach) else reach) - self.reach_margin_m
        except Exception:
            return 0.0

    def bail_out(self) -> None:
        """Ctrl-C / kill: stop whatever program is running; nothing is held, so no gripper."""
        try:
            self.say("Interrupted: stopping the program")
            if self.robot is not None:
                self.robot.stop()
            else:
                self.cockpit.post("/api/robot/stop")
        except Exception as exc:  # best effort on the way out
            self.say(f"stop failed: {exc}")

    # -- vision -------------------------------------------------------------------------------
    def _faces(self) -> tuple[dict, list[dict]]:
        """Every unclipped white blob's top face in the current frame:
        ``[{centre_cam, pixel, n, blob}]`` plus the frame header."""
        hdr, w, h, ch, rgb, depth = self.cockpit.frame()
        K = hdr["intrinsics"]
        out = []
        for b in white_blobs(w, h, ch, rgb, min_px=self.min_blob_px):
            x0, y0, x1, y1 = b["bbox"]
            if x0 <= 6 or y0 <= 6 or x1 >= w - 6 or y1 >= h - 6:
                continue
            tf = top_face(w, h, ch, rgb, depth, hdr["depth_scale_m"], K, b["bbox"])
            if tf is None:
                continue
            c = tf["centre"]
            px = (int(round(K["ppx"] + K["fx"] * c[0] / c[2])), int(round(K["ppy"] + K["fy"] * c[1] / c[2])))
            out.append({"centre_cam": c, "pixel": px, "n": tf["n"], "blob": b})
        return hdr, out

    def find_mark(self) -> dict:
        """The block nearest the image centre from the current pose; sets ``mark``."""
        hdr, faces = self._faces()
        if not faces:
            raise CockpitError("no white block in view — centre a block under the camera and retry")
        w, h = hdr["width"], hdr["height"]
        face = min(faces, key=lambda f: math.hypot(f["pixel"][0] - w / 2, f["pixel"][1] - h / 2))
        flange = self._flange()
        self.mark = list(Transform.from_pose(flange).apply(self.track.apply(face["centre_cam"])))
        return {
            "mark_base": self.mark,
            "pixel": face["pixel"],
            "range_m": face["centre_cam"][2],
            "flange": flange,
        }

    # -- the routine --------------------------------------------------------------------------
    def prepare(self) -> dict:
        rob = self.cockpit.get("/api/robot").get("robot") or {}
        he = (rob.get("handeye") or {}).get("flange_to_color_pose")
        if not he:
            raise CockpitError("the cockpit has no hand-eye seed (no robot link?)")
        self.track = Transform.from_pose(he)
        self.local = CalibrationSession(seed=HandEye.from_pose(he, source="cockpit-seed"))
        found = self.find_mark()
        self.say(
            f"Mark: block top-face centre at {[round(v, 3) for v in self.mark]} m (base), "
            f"seen at {found['pixel']} from {found['range_m']:.2f} m; "
            f"seed hand-eye {(rob.get('handeye') or {}).get('source', '?')}"
        )
        return found

    def _refresh_track(self) -> None:
        """Once enough views are in, predict through the running solve instead of the seed."""
        if self.local is None or len(self.local.views) < MIN_VIEWS_WITHOUT_MARK:
            return
        try:
            res = self.local.solve()
        except CalibrationError:
            return
        if res["rms_m"] > 0.03:
            return  # worse than the seed could be — keep predicting from what we had
        self.track = Transform.from_pose(res["flange_to_color_pose"])
        self.mark = list(res["mark_base"])

    def look(self, step: dict, K: dict, frame_wh: tuple[int, int]) -> dict:
        """One planned view: move, find the block by identity, click it in."""
        name, pose = step["name"], step["pose"]
        pred = predict_pixel(self.track, pose, self.mark, K)
        w, h = frame_wh
        mx, my = w * self.margin_frac, h * self.margin_frac
        if pred is None or not (mx < pred[0] < w - mx and my < pred[1] < h - my):
            return {"name": name, "status": "skipped", "why": "mark predicted outside the frame"}
        limit = self._reach_limit()
        if limit and math.hypot(pose[0], pose[1], pose[2]) > limit:
            return {"name": name, "status": "skipped", "why": f"beyond the arm's {limit:.2f} m working reach"}
        r = self._move(pose)
        if not r.get("ok"):
            if r.get("protective_stop") or "PROTECTIVE" in (self._state().get("safety_mode") or ""):
                self.say(f"{name}: protective stop — unlocking and moving on")
                self._bring_up()
                time.sleep(1.0)
                return {"name": name, "status": "protective_stop"}
            why = r.get("error") or (r.get("safety") or {}).get("violations")
            return {"name": name, "status": "refused", "why": str(why)}
        if self.dry_run:
            return {"name": name, "status": "planned", "predicted_pixel": [round(pred[0]), round(pred[1])]}
        time.sleep(self.settle_s)
        _, faces = self._faces()
        gate = (
            self.gate_mm if len(self.local.views) >= MIN_VIEWS_WITHOUT_MARK else self.first_gate_mm
        ) / 1000.0
        best = None
        for f in faces:
            d = math.dist(f["centre_cam"], pred[2])
            if d <= gate and (best is None or d < best[0]):
                best = (d, f)
        if best is None:
            return {
                "name": name,
                "status": "no_match",
                "why": f"{len(faces)} top face(s) in view, none within {gate * 1000:.0f} mm "
                "of the predicted mark",
            }
        d, face = best
        flange = self._flange()
        v = self.cockpit.post("/api/cal/view", {"x": face["pixel"][0], "y": face["pixel"][1]})
        if not v.get("ok"):
            return {"name": name, "status": "view_refused", "why": v.get("error")}
        self.local.add_view(flange, face["centre_cam"], pixel=face["pixel"])
        self._refresh_track()
        return {"name": name, "status": "kept", "pixel": list(face["pixel"]), "d_mm": round(d * 1000, 1)}

    def trim(self, result: dict) -> tuple[dict, int]:
        """Drop the worst view while any residual is above ``trim_mm`` (and views remain)."""
        dropped = 0
        while result.get("ok"):
            per = result.get("per_view_residual_m") or []
            if len(per) <= MIN_VIEWS_WITHOUT_MARK:
                break
            worst = max(range(len(per)), key=lambda i: per[i])
            if per[worst] * 1000.0 <= self.trim_mm:
                break
            self.cockpit.post("/api/cal/remove", {"index": worst})
            if self.local is not None and worst < len(self.local.views):
                self.local.remove_view(worst)
            dropped += 1
            self.say(f"Trim: dropped view {worst + 1} at {per[worst] * 1000:.1f} mm; re-solving")
            result = self.cockpit.post("/api/cal/solve")
        return result, dropped

    def run(self, *, apply: bool = False, reset: bool = True) -> dict:
        st = self._state()
        if "REMOTE" not in (st.get("control_mode") or "REMOTE"):
            raise CockpitError("robot is in Local control — motion needs Remote")
        found = self.prepare()
        start = found["flange"]
        hdr, _ = self._faces()
        K, wh = hdr["intrinsics"], (hdr["width"], hdr["height"])
        if reset and not self.dry_run:
            self.cockpit.post("/api/cal/reset")
        ranges = ", ".join(f"{r:.2f}" for r in self.ranges_m)
        self.say(f"Orbit: {len(self.ranges_m)} range(s) at {ranges} m, 13 views each")
        steps = []
        planned = 0
        for i, rng in enumerate(self.ranges_m):
            # Each range is planned from the *current* solve: the seed may put the mark
            # centimetres off, and once four views are in the prediction is far better.
            plan = plan_orbit(
                self.track,
                self.mark,
                start,
                ranges_m=(rng,),
                tilt_deg=(self.tilt_deg[min(i, len(self.tilt_deg) - 1)],),
                yaw_deg=(self.yaw_deg[min(i, len(self.yaw_deg) - 1)],),
                shift_m=(self.shift_m[min(i, len(self.shift_m) - 1)],),
                diagonal_deg=self.diagonal_deg,
            )
            planned += len(plan)
            for step in plan:
                out = self.look(step, K, wh)
                steps.append(out)
                if out["status"] not in ("kept", "planned"):
                    self.say(
                        f"{out['name']}: {out['status']}" + (f" — {out['why']}" if out.get("why") else "")
                    )
        kept = sum(1 for s in steps if s["status"] == "kept")
        summary = {
            "ok": True,
            "mark_base": found["mark_base"],
            "planned": planned,
            "kept": kept,
            "steps": steps,
            "events": self.log,
        }
        if self.dry_run:
            summary["dry_run"] = True
            return summary
        self.say(f"{kept} views kept; solving")
        result = self.cockpit.post("/api/cal/solve")
        if not result.get("ok"):
            summary.update({"ok": False, "error": result.get("error") or "solve failed"})
            self._move(start)
            return summary
        result, dropped = self.trim(result)
        self._move(start)
        self.say(
            f"Solved: RMS {result['rms_m'] * 1000:.1f} mm over {result['views']} views"
            + (f" ({dropped} trimmed)" if dropped else "")
            + (f"; {'; '.join(result['warnings'])}" if result.get("warnings") else "")
        )
        summary.update(
            {
                "trimmed": dropped,
                "views": result["views"],
                "rms_mm": round(result["rms_m"] * 1000, 2),
                "per_view_mm": [round(v * 1000, 1) for v in result.get("per_view_residual_m") or []],
                "flange_to_depth_pose": result.get("flange_to_depth_pose"),
                "flange_to_color_pose": result.get("flange_to_color_pose"),
                "env_line": result.get("env_line"),
                "warnings": result.get("warnings") or [],
            }
        )
        if apply:
            a = self.cockpit.post("/api/cal/apply", {})
            summary["applied"] = bool(a.get("ok"))
            summary["saved"] = a.get("saved")
            self.say(
                "Applied" + (f", saved {a['saved']}" if a.get("saved") else "")
                if a.get("ok")
                else f"Apply refused: {a.get('error')}"
            )
        else:
            self.say("Not applied (pass --apply, or Apply in the cockpit's Calibrate panel)")
        return summary


# -- CLI ---------------------------------------------------------------------------------------


def add_calibrate_args(ap) -> None:
    ap.add_argument(
        "--cockpit", default=DEFAULT_COCKPIT, help=f"the running cockpit (default {DEFAULT_COCKPIT})"
    )
    ap.add_argument(
        "--range-m",
        type=float,
        nargs="+",
        default=list(DEFAULT_RANGES_M),
        help="camera-to-mark ranges to orbit at (default 0.21 0.30 0.40; the D435 has no depth under 0.2)",
    )
    ap.add_argument(
        "--tilt-deg", type=float, nargs="+", default=list(DEFAULT_TILT_DEG), help="tilt per range"
    )
    ap.add_argument("--yaw-deg", type=float, nargs="+", default=list(DEFAULT_YAW_DEG), help="yaw per range")
    ap.add_argument(
        "--shift-m", type=float, nargs="+", default=list(DEFAULT_SHIFT_M), help="lateral shift per range"
    )
    ap.add_argument("--gate-mm", type=float, default=40.0, help="identity gate: candidate vs predicted mark")
    ap.add_argument("--trim-mm", type=float, default=9.0, help="drop views with a residual above this")
    ap.add_argument("--velocity", type=float, default=0.08, help="move speed m/s (default 0.08)")
    ap.add_argument("--apply", action="store_true", help="apply + save the solve in the cockpit")
    ap.add_argument("--no-reset", action="store_true", help="add to the cockpit's existing views")
    ap.add_argument(
        "--via-cockpit",
        action="store_true",
        help="drive the robot through the cockpit's API (default: direct over Primary; "
        "the cockpit does vision only)",
    )
    ap.add_argument(
        "--robot-host", default=None, help="controller address (default: the cockpit's robot, else UR_HOST)"
    )
    ap.add_argument("--dry-run", action="store_true", help="find the mark and print the plan; send no motion")
    ap.add_argument("--json", action="store_true", help="print the result as JSON")


def run_calibrate(args) -> int:
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
    cal = OrbitCalibration(
        cockpit,
        robot=robot,
        ranges_m=args.range_m,
        tilt_deg=args.tilt_deg,
        yaw_deg=args.yaw_deg,
        shift_m=args.shift_m,
        gate_mm=args.gate_mm,
        trim_mm=args.trim_mm,
        velocity=args.velocity,
        dry_run=args.dry_run,
    )
    import signal

    def _interrupt(signum, frame):
        raise KeyboardInterrupt  # unwinds the blocked Primary call (releasing its lock) first

    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    try:
        out = cal.run(apply=args.apply, reset=not args.no_reset)
    except KeyboardInterrupt:
        cal.bail_out()
        return 130
    except (CockpitError, CalibrationError, urllib.error.URLError, OSError) as exc:
        print(f"calibrate: {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(out, indent=1, default=str))
    elif out.get("env_line"):
        print(out["env_line"])
    return 0 if out.get("ok") else 1
