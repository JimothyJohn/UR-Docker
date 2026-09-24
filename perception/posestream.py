"""PoseRecorder — the robot's pose as a *function of host time*.

The monocular scan (`docs/mono-scan.md`) needs, for every camera frame, where
the tool flange was at the instant of exposure. RTDE streams
``actual_TCP_pose`` at up to 500 Hz with the controller's own ``timestamp``;
this module runs that stream on a thread, stamps each sample with the host's
monotonic clock on arrival, and answers ``flange_at(host_t)`` by interpolating
between the two neighbouring samples. Camera latency (the frame arrives after
the exposure) is a single scalar the caller subtracts —
``flange_at(frame_host_t - latency_s)`` — fitted once by
:mod:`perception.latency`.

Frames: RTDE reports the **TCP**; the hand-eye is flange-based, so the
recorder is given the active TCP offset (``Robot.get_flange_pose()["tcp_offset"]``,
read once) and returns the *flange* pose: ``T_base_flange = T_base_tcp ·
inv(T_flange_tcp)``.

Pure stdlib; a ``client`` (anything with ``stream()`` yielding dicts and
``close()``) is injectable so the sweep runs on a synthetic robot in tests.
"""

from __future__ import annotations

import bisect
import math
import threading
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field

from urctl.config import RobotConfig
from urctl.pose import Transform, matrix_to_rotvec, pose_inv, pose_trans, rotvec_to_matrix

RECORDER_OUTPUTS = [
    "timestamp",
    "actual_TCP_pose",
    "actual_TCP_speed",
    "actual_q",
    "actual_digital_input_bits",
    "actual_digital_output_bits",
]
DEFAULT_RECORDER_HZ = 125.0
FLANGE_TCP = [0.0] * 6


@dataclass(frozen=True)
class PoseSample:
    host_t: float
    robot_t: float
    tcp_pose: list[float]
    tcp_speed: list[float]
    q: list[float]
    din: int
    dout: int


def interpolate_pose(a: Sequence[float], b: Sequence[float], f: float) -> list[float]:
    """UR pose between ``a`` (f=0) and ``b`` (f=1): translation lerped, rotation
    slerped along the relative rotation (exact for the small steps between
    RTDE samples)."""
    f = min(1.0, max(0.0, f))
    t = [a[i] + f * (b[i] - a[i]) for i in range(3)]
    ra = Transform.from_pose(a).rotation
    rb = Transform.from_pose(b).rotation
    rel = Transform(ra).inverse().compose(Transform(rb)).rotation
    rv = matrix_to_rotvec(rel)
    scaled = (rv[0] * f, rv[1] * f, rv[2] * f)
    rot = Transform(ra).compose(Transform(rotvec_to_matrix(scaled))).rotation
    return [*t, *matrix_to_rotvec(rot)]


class PoseRecorder:
    """Record RTDE poses on a thread; query by host time.

    ``tcp_offset`` is the active flange→TCP UR pose (``[0]*6`` when the TCP is
    the flange). ``clock`` defaults to :func:`time.monotonic` and must be the
    same clock the camera frames are stamped with.
    """

    def __init__(
        self,
        config: RobotConfig | None = None,
        *,
        frequency: float = DEFAULT_RECORDER_HZ,
        tcp_offset: Sequence[float] | None = None,
        client=None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.config = config or RobotConfig.from_env()
        self.frequency = float(frequency)
        self.tcp_offset = [float(v) for v in (tcp_offset if tcp_offset is not None else FLANGE_TCP)]
        self._client = client
        self._clock = clock
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._samples: list[PoseSample] = []
        self._times: list[float] = []
        self._stop = threading.Event()
        self.error: str | None = None

    # ----- lifecycle ---------------------------------------------------------

    def _make_client(self):
        if self._client is not None:
            return self._client
        from urctl.rtde import RtdeClient

        return RtdeClient(self.config, outputs=RECORDER_OUTPUTS, frequency=self.frequency, strict=False)

    def start(self, *, warmup_s: float = 0.5) -> None:
        """Connect and start recording; blocks up to ``warmup_s`` for the first
        sample so the caller knows the stream is alive (``error`` says why not)."""
        if self._thread is not None:
            return
        self._stop.clear()
        self.error = None
        client = self._make_client()
        self._client = client
        self._thread = threading.Thread(target=self._run, args=(client,), name="pose-recorder", daemon=True)
        self._thread.start()
        deadline = self._clock() + warmup_s
        while self._clock() < deadline and not self._samples and self.error is None:
            time.sleep(0.005)

    def _run(self, client) -> None:
        try:
            for sample in client.stream():
                if self._stop.is_set():
                    break
                now = self._clock()
                ps = PoseSample(
                    host_t=now,
                    robot_t=float(sample.get("timestamp", 0.0)),
                    tcp_pose=[float(v) for v in sample["actual_TCP_pose"]],
                    tcp_speed=[float(v) for v in sample.get("actual_TCP_speed", [0.0] * 6)],
                    q=[float(v) for v in sample.get("actual_q", [0.0] * 6)],
                    din=int(sample.get("actual_digital_input_bits", 0)),
                    dout=int(sample.get("actual_digital_output_bits", 0)),
                )
                with self._lock:
                    self._samples.append(ps)
                    self._times.append(now)
        except Exception as exc:  # noqa: BLE001 — surfaced via .error, thread must not die silently
            if not self._stop.is_set():
                self.error = f"{type(exc).__name__}: {exc}"

    def stop(self) -> None:
        self._stop.set()
        client = self._client
        if client is not None:
            try:
                client.close()
            except Exception:  # noqa: BLE001
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def __enter__(self) -> PoseRecorder:
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    # ----- queries -----------------------------------------------------------

    @property
    def samples(self) -> list[PoseSample]:
        with self._lock:
            return list(self._samples)

    def __len__(self) -> int:
        return len(self._samples)

    def span(self) -> tuple[float, float] | None:
        with self._lock:
            if not self._times:
                return None
            return self._times[0], self._times[-1]

    def sample_rate_hz(self) -> float | None:
        s = self.span()
        n = len(self._samples)
        if s is None or n < 2 or s[1] <= s[0]:
            return None
        return (n - 1) / (s[1] - s[0])

    def tcp_at(self, host_t: float, *, max_gap_s: float = 0.05) -> list[float] | None:
        """TCP pose at ``host_t`` (interpolated); None outside the recording or
        across a gap larger than ``max_gap_s``."""
        with self._lock:
            if not self._samples:
                return None
            i = bisect.bisect_left(self._times, host_t)
            if i == 0:
                first = self._samples[0]
                return list(first.tcp_pose) if host_t >= first.host_t - max_gap_s else None
            if i >= len(self._samples):
                last = self._samples[-1]
                return list(last.tcp_pose) if host_t <= last.host_t + max_gap_s else None
            a, b = self._samples[i - 1], self._samples[i]
        if b.host_t - a.host_t > max_gap_s:
            return None
        f = 0.0 if b.host_t == a.host_t else (host_t - a.host_t) / (b.host_t - a.host_t)
        return interpolate_pose(a.tcp_pose, b.tcp_pose, f)

    def flange_at(self, host_t: float, *, max_gap_s: float = 0.05) -> list[float] | None:
        tcp = self.tcp_at(host_t, max_gap_s=max_gap_s)
        if tcp is None:
            return None
        return pose_trans(tcp, pose_inv(self.tcp_offset))

    def speed_at(self, host_t: float) -> float | None:
        """|TCP linear velocity| (m/s) at the nearest sample."""
        with self._lock:
            if not self._samples:
                return None
            i = bisect.bisect_left(self._times, host_t)
            i = min(max(i, 0), len(self._samples) - 1)
            s = self._samples[i]
        return math.sqrt(sum(v * v for v in s.tcp_speed[:3]))

    def edges(self, bit: int, *, output: bool = True) -> list[float]:
        """Host times of rising edges on digital ``bit`` (the T2 trigger path:
        one edge per exposure once a tool DO drives the camera)."""
        out: list[float] = []
        prev = None
        for s in self.samples:
            v = ((s.dout if output else s.din) >> bit) & 1
            if prev == 0 and v == 1:
                out.append(s.host_t)
            prev = v
        return out

    def as_dict(self) -> dict:
        s = self.span()
        return {
            "samples": len(self._samples),
            "rate_hz": self.sample_rate_hz(),
            "span_s": None if s is None else s[1] - s[0],
            "tcp_offset": self.tcp_offset,
            "error": self.error,
        }


class ReplayClient:
    """A stand-in RTDE client for tests / the synthetic rig: ``stream()`` yields
    the samples from ``source`` (an iterable of dicts, or a callable returning
    the next sample dict, paced at ``frequency``)."""

    def __init__(self, source: Iterable[dict] | Callable[[], dict | None], *, frequency: float = 125.0):
        self._source = source
        self._period = 1.0 / frequency
        self._closed = False

    def stream(self):
        if callable(self._source):
            while not self._closed:
                t0 = time.monotonic()
                sample = self._source()
                if sample is None:
                    break
                yield sample
                rest = self._period - (time.monotonic() - t0)
                if rest > 0:
                    time.sleep(rest)
        else:
            for sample in self._source:
                if self._closed:
                    break
                yield sample

    def close(self) -> None:
        self._closed = True


@dataclass
class LinearMotion:
    """A straight-line TCP move with a trapezoidal speed profile, as a pose
    function of time — the synthetic robot behind the fake sweep rig."""

    start_pose: list[float]
    delta: list[float] = field(default_factory=lambda: [0.0] * 6)
    velocity: float = 0.1
    acceleration: float = 0.5
    t0: float | None = None

    @property
    def distance(self) -> float:
        return math.sqrt(sum(v * v for v in self.delta[:3]))

    @property
    def duration(self) -> float:
        d, v, a = self.distance, self.velocity, self.acceleration
        if d <= 0.0:
            return 0.0
        t_acc = v / a
        if a * t_acc * t_acc >= d:  # triangular
            return 2.0 * math.sqrt(d / a)
        return 2.0 * t_acc + (d - a * t_acc * t_acc) / v

    def fraction(self, t: float) -> float:
        """Fraction of the path covered at time ``t`` (0 before t0, 1 after)."""
        if self.t0 is None or t <= self.t0 or self.distance <= 0.0:
            return 0.0
        dt = t - self.t0
        d, v, a = self.distance, self.velocity, self.acceleration
        t_acc = v / a
        if a * t_acc * t_acc >= d:
            t_acc = math.sqrt(d / a)
            v = a * t_acc
        total = self.duration
        if dt >= total:
            return 1.0
        if dt < t_acc:
            s = 0.5 * a * dt * dt
        elif dt < total - t_acc:
            s = 0.5 * a * t_acc * t_acc + v * (dt - t_acc)
        else:
            r = total - dt
            s = d - 0.5 * a * r * r
        return min(1.0, max(0.0, s / d))

    def pose_at(self, t: float) -> list[float]:
        f = self.fraction(t)
        return [self.start_pose[i] + f * self.delta[i] for i in range(6)]

    def speed_at(self, t: float) -> float:
        h = 1e-3
        return abs(self.fraction(t + h) - self.fraction(t - h)) * self.distance / (2 * h)
