"""The scan itself: one relative move while frames and poses are recorded.

``run_sweep`` starts the :class:`~perception.posestream.PoseRecorder`, grabs
frames from a mono source on a thread, commands **one** relative base-frame
move through the robot link (``ur_move_tcp``, safety-enveloped and audited
like every other move), waits for it to confirm, then stamps every frame with
the flange pose at ``host_t - latency_s`` (`docs/mono-scan.md` §2, tier T0).
The result is a :class:`Sweep`: frames + ``T_base_cam`` per frame + the
intrinsics + the table plane — everything :func:`perception.locate2d.locate_objects`
needs, saved to a directory so it can be replayed and re-fitted offline.

A :class:`FakeRig` wires the same runner to the synthetic camera and a
simulated trajectory (with a deliberate camera latency) so ``perception scan
--fake`` and the tests exercise the whole path with no hardware.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from urctl.pose import Transform, pose_inv, pose_trans

from .handeye import HandEye
from .monocam import Box, MonoFrame, SyntheticMono, default_intrinsics
from .posestream import LinearMotion, PoseRecorder, ReplayClient, interpolate_pose
from .rgbd import Intrinsics
from .tableplane import Plane

DEFAULT_DELTA_M = (0.15, 0.0, 0.0)
DEFAULT_SWEEP_VELOCITY = 0.15  # m/s
DEFAULT_SWEEP_ACCELERATION = 0.5  # m/s^2
MAX_SWEEP_M = 0.30
ENV_CAM_LATENCY = "PERCEPTION_CAM_LATENCY_S"
DEFAULT_SWEEP_ROOT = "captures/sweeps"


@dataclass
class SweepFrame:
    gray: Any
    host_t: float
    seq: int
    flange_pose: list[float] | None = None
    speed_mps: float | None = None
    camera_t_ms: float | None = None

    def t_base_cam(self, flange_to_color: Transform) -> Transform | None:
        if self.flange_pose is None:
            return None
        return Transform.from_pose(self.flange_pose).compose(flange_to_color)


@dataclass
class Sweep:
    """``flange_to_color`` is the flange→camera transform of *whichever* stream
    the frames came from (colour, or the left IR imager = the depth frame);
    ``trajectory`` is the recorder's raw ``(host_t, flange_pose)`` samples so the
    frames can be re-stamped with another latency offline (:meth:`restamp`)."""

    frames: list[SweepFrame]
    intrinsics: Intrinsics
    flange_to_color: Transform
    plane: Plane | None = None
    meta: dict = field(default_factory=dict)
    trajectory: list[tuple[float, list[float]]] = field(default_factory=list)

    def flange_at(self, host_t: float, *, max_gap_s: float = 0.05) -> list[float] | None:
        """Flange pose at a raw host time from the saved trajectory (interpolated)."""
        import bisect

        if not self.trajectory:
            return None
        times = [t for t, _ in self.trajectory]
        i = bisect.bisect_left(times, host_t)
        if i == 0:
            return list(self.trajectory[0][1]) if host_t >= times[0] - max_gap_s else None
        if i >= len(times):
            return list(self.trajectory[-1][1]) if host_t <= times[-1] + max_gap_s else None
        (t0, p0), (t1, p1) = self.trajectory[i - 1], self.trajectory[i]
        if t1 - t0 > max_gap_s:
            return None
        return interpolate_pose(p0, p1, 0.0 if t1 == t0 else (host_t - t0) / (t1 - t0))

    def restamp(self, latency_s: float) -> Sweep:
        """The same frames with poses taken at ``host_t - latency_s`` (needs ``trajectory``)."""
        if not self.trajectory:
            raise ValueError("this sweep carries no trajectory; it cannot be re-stamped")
        frames = [
            SweepFrame(
                f.gray, f.host_t, f.seq, self.flange_at(f.host_t - latency_s), f.speed_mps, f.camera_t_ms
            )
            for f in self.frames
        ]
        return Sweep(
            frames,
            self.intrinsics,
            self.flange_to_color,
            self.plane,
            dict(self.meta, latency_s=float(latency_s)),
            list(self.trajectory),
        )

    def frame_pairs(self, *, moving_only: bool = False) -> list[tuple[Any, Transform]]:
        out = []
        for f in self.frames:
            t = f.t_base_cam(self.flange_to_color)
            if t is None:
                continue
            if moving_only and (f.speed_mps is None or f.speed_mps < 0.005):
                continue
            out.append((f.gray, t))
        return out

    def baseline_m(self) -> float:
        ps = [f.flange_pose[:3] for f in self.frames if f.flange_pose is not None]
        if len(ps) < 2:
            return 0.0
        return max(math.dist(a, b) for a in ps[:: max(1, len(ps) // 8)] for b in ps)

    def summary(self) -> dict:
        posed = sum(1 for f in self.frames if f.flange_pose is not None)
        return {
            "frames": len(self.frames),
            "frames_with_pose": posed,
            "baseline_m": self.baseline_m(),
            "duration_s": (self.frames[-1].host_t - self.frames[0].host_t) if len(self.frames) > 1 else 0.0,
            "plane": None if self.plane is None else self.plane.as_dict(),
            **self.meta,
        }

    def save(self, directory: str | Path) -> Path:
        import cv2

        d = Path(directory)
        d.mkdir(parents=True, exist_ok=True)
        frames = []
        for i, f in enumerate(self.frames):
            name = f"frame_{i:03d}.png"
            cv2.imwrite(str(d / name), f.gray)
            frames.append(
                {
                    "file": name,
                    "host_t": f.host_t,
                    "seq": f.seq,
                    "flange_pose": f.flange_pose,
                    "speed_mps": f.speed_mps,
                    "camera_t_ms": f.camera_t_ms,
                }
            )
        doc = {
            "intrinsics": self.intrinsics.__dict__ | {"coeffs": list(self.intrinsics.coeffs)},
            "flange_to_color_pose": self.flange_to_color.to_pose(),
            "plane": None if self.plane is None else self.plane.as_dict(),
            "meta": self.meta,
            "frames": frames,
            "trajectory": [[t, p] for t, p in self.trajectory],
        }
        (d / "sweep.json").write_text(json.dumps(doc, indent=2, default=str))
        return d

    @classmethod
    def load(cls, directory: str | Path) -> Sweep:
        import cv2

        d = Path(directory)
        doc = json.loads((d / "sweep.json").read_text())
        intr_d = dict(doc["intrinsics"])
        intr_d["coeffs"] = tuple(intr_d.get("coeffs", (0.0,) * 5))
        intr = Intrinsics(**intr_d)
        frames = []
        for fd in doc["frames"]:
            gray = cv2.imread(str(d / fd["file"]), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise FileNotFoundError(d / fd["file"])
            frames.append(
                SweepFrame(
                    gray,
                    fd["host_t"],
                    fd["seq"],
                    fd.get("flange_pose"),
                    fd.get("speed_mps"),
                    fd.get("camera_t_ms"),
                )
            )
        plane = None
        if doc.get("plane"):
            pd = doc["plane"]
            plane = Plane(tuple(pd["point"]), tuple(pd["normal"]), pd.get("source", "file"))
        traj = [(float(t), [float(v) for v in p]) for t, p in doc.get("trajectory", [])]
        return cls(
            frames, intr, Transform.from_pose(doc["flange_to_color_pose"]), plane, doc.get("meta", {}), traj
        )


class FrameGrabber:
    """Read frames from a mono source on a thread until stopped."""

    def __init__(self, source):
        self.source = source
        self.frames: list[MonoFrame] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="frame-grabber", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                self.frames.append(self.source.read())
        except Exception as exc:  # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None


def run_sweep(
    source,
    recorder: PoseRecorder,
    mover,
    *,
    intrinsics: Intrinsics,
    flange_to_color: Transform,
    delta: Sequence[float] = DEFAULT_DELTA_M,
    velocity: float = DEFAULT_SWEEP_VELOCITY,
    acceleration: float = DEFAULT_SWEEP_ACCELERATION,
    latency_s: float = 0.0,
    plane: Plane | None = None,
    pre_roll_s: float = 0.25,
    settle_s: float = 0.25,
    clock: Callable[[], float] = time.monotonic,
) -> Sweep:
    """One relative move (``delta`` = base-frame ``[dx, dy, dz]`` m) with frames
    + poses recorded throughout. ``mover.move_relative(delta6, velocity,
    acceleration)`` must block until the move confirms and return a dict with
    ``ok``. Raises ``RuntimeError`` when the move is refused."""
    d = [float(v) for v in delta] + [0.0] * (3 - len(delta))
    dist = math.sqrt(sum(v * v for v in d[:3]))
    if dist <= 0.0 or dist > MAX_SWEEP_M:
        raise ValueError(f"sweep distance must be within (0, {MAX_SWEEP_M}] m, got {dist:.3f}")
    delta6 = [d[0], d[1], d[2], 0.0, 0.0, 0.0]
    recorder.start()
    if recorder.error:
        raise RuntimeError(f"pose recorder: {recorder.error}")
    grabber = FrameGrabber(source)
    grabber.start()
    try:
        time.sleep(pre_roll_s)
        t_move0 = clock()
        result = mover.move_relative(delta6, velocity=velocity, acceleration=acceleration)
        t_move1 = clock()
        time.sleep(settle_s)
    finally:
        grabber.stop()
        recorder.stop()
    if grabber.error:
        raise RuntimeError(f"camera: {grabber.error}")
    if not result.get("ok"):
        raise RuntimeError(f"sweep move refused: {result.get('error') or result}")
    frames: list[SweepFrame] = []
    for mf in grabber.frames:
        t_exp = mf.host_t - latency_s
        pose = recorder.flange_at(t_exp)
        frames.append(SweepFrame(mf.gray, mf.host_t, mf.seq, pose, recorder.speed_at(t_exp), mf.camera_t_ms))
    meta = {
        "delta_m": d[:3],
        "velocity": velocity,
        "acceleration": acceleration,
        "latency_s": latency_s,
        "move_duration_s": t_move1 - t_move0,
        "move": {k: result.get(k) for k in ("ok", "landed", "dry_run") if k in result},
        "recorder": recorder.as_dict(),
        "camera": _safe_describe(source),
    }
    trajectory = [
        (smp.host_t, pose_trans(smp.tcp_pose, pose_inv(recorder.tcp_offset))) for smp in recorder.samples
    ]
    return Sweep(frames, intrinsics, flange_to_color, plane, meta, trajectory)


def _safe_describe(source) -> dict:
    try:
        return dict(source.describe())
    except Exception as exc:  # noqa: BLE001
        return {"error": str(exc)}


def latency_from_env(env=None) -> float:
    import os

    env = os.environ if env is None else env
    raw = env.get(ENV_CAM_LATENCY, "").strip()
    return float(raw) if raw else 0.0


# ----- the synthetic rig -----------------------------------------------------

DEFAULT_FAKE_SCENE = [
    Box((0.30, 0.02, 0.0), (0.05, 0.03, 0.03), 0.3),
    Box((0.38, -0.08, 0.0), (0.03, 0.03, 0.05), 0.0),
    Box((0.25, -0.12, 0.0), (0.05, 0.03, 0.03), 1.2),
]
FAKE_START_FLANGE = [0.25, 0.0, 0.35, math.pi, 0.0, 0.0]  # camera looking down from 0.35 m
FAKE_TCP_OFFSET = [0.0, 0.0, 0.223, 0.0, 0.0, 0.0]  # the UR3e's training TCP, for realism


class FakeMover:
    """``move_relative`` on a :class:`LinearMotion`; blocks for its duration."""

    def __init__(self, motion: LinearMotion, clock: Callable[[], float] = time.monotonic):
        self.motion = motion
        self._clock = clock

    def move_relative(self, delta6, *, velocity: float, acceleration: float) -> dict:
        m = self.motion
        m.start_pose = m.pose_at(self._clock())
        m.delta = [float(v) for v in delta6]
        m.velocity, m.acceleration = float(velocity), float(acceleration)
        m.t0 = self._clock()
        time.sleep(m.duration)
        return {"ok": True, "landed": m.pose_at(self._clock()), "dry_run": False}


@dataclass
class FakeRig:
    """Synthetic camera + simulated robot sharing one trajectory: the camera
    exposes at ``t`` and hands the frame over at ``t + latency_s``; the
    'RTDE' recorder samples the true TCP at 125 Hz."""

    latency_s: float = 0.04
    boxes: list[Box] = field(default_factory=lambda: list(DEFAULT_FAKE_SCENE))
    plane: Plane = field(default_factory=lambda: Plane.from_z(0.0, source="fake"))
    intrinsics: Intrinsics = field(default_factory=default_intrinsics)
    handeye: HandEye = field(default_factory=lambda: HandEye.for_bracket("eseries"))
    start_flange: list[float] = field(default_factory=lambda: list(FAKE_START_FLANGE))
    tcp_offset: list[float] = field(default_factory=lambda: list(FAKE_TCP_OFFSET))
    noise: float = 3.0
    clock: Callable[[], float] = time.monotonic

    def __post_init__(self) -> None:
        self.motion = LinearMotion(list(self.start_flange))
        self.source = SyntheticMono(
            self.intrinsics,
            self.plane,
            self.boxes,
            self.handeye.flange_to_color,
            self.motion.pose_at,
            latency_s=self.latency_s,
            noise=self.noise,
            clock=self.clock,
        )
        self.mover = FakeMover(self.motion, self.clock)

    def _sample(self) -> dict:
        t = self.clock()
        flange = self.motion.pose_at(t)
        return {
            "timestamp": t,
            "actual_TCP_pose": pose_trans(flange, self.tcp_offset),
            "actual_TCP_speed": [self.motion.speed_at(t), 0.0, 0.0, 0.0, 0.0, 0.0],
            "actual_q": [0.0] * 6,
            "actual_digital_input_bits": 0,
            "actual_digital_output_bits": 0,
        }

    def recorder(self, frequency: float = 125.0) -> PoseRecorder:
        return PoseRecorder(
            tcp_offset=self.tcp_offset,
            client=ReplayClient(self._sample, frequency=frequency),
            clock=self.clock,
        )

    def run(
        self,
        *,
        delta=DEFAULT_DELTA_M,
        velocity=DEFAULT_SWEEP_VELOCITY,
        acceleration=DEFAULT_SWEEP_ACCELERATION,
        latency_s: float | None = None,
    ) -> Sweep:
        return run_sweep(
            self.source,
            self.recorder(),
            self.mover,
            intrinsics=self.intrinsics,
            flange_to_color=self.handeye.flange_to_color,
            delta=delta,
            velocity=velocity,
            acceleration=acceleration,
            latency_s=self.latency_s if latency_s is None else latency_s,
            plane=self.plane,
            clock=self.clock,
        )


class SweepRgbdCamera:
    """An :class:`~perception.realsense.RgbdCamera` whose colour is the fake
    rig's rendering at the rig's *current* pose and whose depth is the table
    plane + the box tops — so the **cockpit** (which owns an RGB-D camera) can
    run a scan, a table-from-depth and the depth cross-check on the synthetic
    scene. ``perception gui --fake-scan`` and the tests use it."""

    def __init__(self, rig: FakeRig, *, fps: int = 30):
        self.rig = rig
        self.fps = fps
        self._open = False
        self._n = 0

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def read(self):
        import numpy as np

        from .frame import Frame
        from .rgbd import DepthImage, RgbdFrame

        if not self._open:
            raise RuntimeError("camera is not open")
        if self.fps > 0:
            time.sleep(1.0 / self.fps)
        self._n += 1
        rig = self.rig
        t = rig.clock()
        flange = rig.motion.pose_at(t)
        gray = rig.source.render(flange)
        intr = rig.intrinsics
        t_base_cam = Transform.from_pose(flange).compose(rig.handeye.flange_to_color)
        # depth: every pixel's ray meets the table; box tops override inside their top face
        ys, xs = np.mgrid[0 : intr.height, 0 : intr.width]
        rays = np.stack(
            [(xs - intr.ppx) / intr.fx, (ys - intr.ppy) / intr.fy, np.ones_like(xs, dtype=float)], -1
        )
        rot = np.asarray(t_base_cam.rotation)
        rays_b = rays.reshape(-1, 3) @ rot.T
        c = np.asarray(t_base_cam.translation)
        n = np.asarray(rig.plane.normal)
        p0 = np.asarray(rig.plane.point)

        def depth_to(h):
            denom = rays_b @ n
            tt = (n @ (p0 + h * n - c)) / np.where(np.abs(denom) < 1e-9, np.nan, denom)
            return tt.reshape(intr.height, intr.width)  # z_cam = t (ray z is 1)

        depth = depth_to(0.0)
        import cv2

        t_cam_base = t_base_cam.inverse()
        for box in rig.boxes:
            corners = box.corners_base(rig.plane)[4:]
            pix = []
            for cb in corners:
                pc = t_cam_base.apply(cb)
                if pc[2] <= 1e-6:
                    break
                pix.append([intr.fx * pc[0] / pc[2] + intr.ppx, intr.fy * pc[1] / pc[2] + intr.ppy])
            else:
                mask = np.zeros((intr.height, intr.width), np.uint8)
                cv2.fillPoly(mask, [np.round(np.array(pix)).astype(np.int32)], 255)
                top = depth_to(box.size[2])
                depth = np.where(mask > 0, top, depth)
        depth = np.where(np.isfinite(depth) & (depth > 0), depth, 0.0)
        units = np.clip(np.round(depth / 0.001), 0, 65535).astype("<u2")
        rgb = np.repeat(gray[..., None], 3, axis=2)
        return RgbdFrame(
            color=Frame.from_numpy(rgb),
            depth=DepthImage(width=intr.width, height=intr.height, data=units.tobytes(), scale_m=0.001),
            intrinsics=intr,
            timestamp_ms=t * 1000.0,
            frame_number=self._n,
            aligned=True,
            extra={"serial": "SYNTH-SWEEP", "exposure_t": t},
        )

    def describe(self) -> dict:
        return {
            "kind": "synthetic-sweep",
            "open": self._open,
            "device": {"name": "Synthetic sweep RGB-D", "serial": "SYNTH-SWEEP", "usb_type": None},
            "stream": {
                "width": self.rig.intrinsics.width,
                "height": self.rig.intrinsics.height,
                "fps": self.fps,
                "aligned": True,
            },
            "depth": {
                "width": self.rig.intrinsics.width,
                "height": self.rig.intrinsics.height,
                "filters": [],
                "tuning": None,
            },
            "depth_scale_m": 0.001,
            "intrinsics": {"color": self.rig.intrinsics.as_dict(), "depth": self.rig.intrinsics.as_dict()},
            "extrinsics_depth_to_color": {
                "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "translation": [0.0, 0.0, 0.0],
            },
            "fake_rig": {"latency_s": self.rig.latency_s, "boxes": len(self.rig.boxes)},
        }


class FakeRigLink:
    """The pieces of :class:`~perception.robotlink.RobotLink` the cockpit's scan
    touches, answered by the fake rig (``move_relative`` drives the motion;
    ``flange_pose`` is the rig's live pose). Wraps a dry-run RobotLink for
    everything else (describe, state, handeye, config)."""

    def __init__(self, rig: FakeRig, link):
        self.rig = rig
        self.link = link

    def __getattr__(self, name):
        return getattr(self.link, name)

    def move_relative(self, delta, *, velocity, acceleration):
        return self.rig.mover.move_relative(delta, velocity=velocity, acceleration=acceleration)

    def tcp_offset(self):
        return list(self.rig.tcp_offset)

    def flange_pose(self) -> dict:
        flange = self.rig.motion.pose_at(self.rig.clock())
        return {
            "ok": True,
            "flange": flange,
            "tcp": pose_trans(flange, self.rig.tcp_offset),
            "tcp_offset": list(self.rig.tcp_offset),
            "dry_run": True,
            "host": self.link.config.host,
        }

    def recorder(self) -> PoseRecorder:
        return self.rig.recorder()
