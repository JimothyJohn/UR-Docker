"""The flange pose, live — so every camera frame can carry the pose it was taken at.

RTDE (port 30004) publishes ``actual_TCP_pose`` and the controller's current
``tcp_offset``; the flange is ``tcp ∘ tcp_offset⁻¹``. Verified on the UR3e
2026-09-27: identical (to 1e-4 m / rad) to the ``get_flange_pose`` script, at the
32 Hz asked for — and unlike the script it needs no Primary program, so it never
replaces a running motion and works in Local mode.

:class:`PoseStream` keeps a short history stamped with host time; the cockpit
looks up the sample nearest each frame's arrival (:meth:`at`) and puts it in the
frame header (``flange_pose``, ``pose_age_s``). With the hand-eye that gives the
page ``T_base_colour`` per frame: a point cloud in table coordinates and a target
rectangle that stays locked on the part as the wrist moves.
"""

from __future__ import annotations

import bisect
import threading
import time
from collections.abc import Callable

from urctl.config import RobotConfig
from urctl.pose import Transform

POSE_OUTPUTS = ["timestamp", "actual_TCP_pose", "tcp_offset", "actual_q"]


def flange_from(sample: dict) -> list[float] | None:
    """``actual_TCP_pose ∘ tcp_offset⁻¹`` for one RTDE sample; None if a field is missing."""
    tcp, off = sample.get("actual_TCP_pose"), sample.get("tcp_offset")
    if not tcp or not off or len(tcp) != 6 or len(off) != 6:
        return None
    return Transform.from_pose(tcp).compose(Transform.from_pose(off).inverse()).to_pose()


class PoseStream:
    """A background RTDE subscription → ``[(host time, flange pose)]`` over the last
    ``history_s`` seconds. Reconnects with back-off; never raises into the caller."""

    def __init__(
        self,
        config: RobotConfig,
        *,
        frequency: float = 30.0,
        history_s: float = 3.0,
        client_factory: Callable | None = None,
        clock: Callable[[], float] = time.time,
    ):
        self.config = config
        self.frequency = frequency
        self.history_s = history_s
        self.clock = clock
        self._factory = client_factory or self._rtde_client
        self._t: list[float] = []
        self._poses: list[list[float]] = []
        self._q: list[list[float] | None] = []  # the joints at each sample (for the arm's linkage)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.last_error: str | None = None
        self.samples = 0

    def _rtde_client(self):
        from urctl.rtde import RtdeClient

        return RtdeClient(self.config, outputs=POSE_OUTPUTS, frequency=self.frequency, strict=False)

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="pose-stream", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def add(self, t: float, flange: list[float], q: list[float] | None = None) -> None:
        with self._lock:
            self._t.append(t)
            self._poses.append(flange)
            self._q.append(q)
            self.samples += 1
            cut = bisect.bisect_left(self._t, t - self.history_s)
            if cut:
                del self._t[:cut], self._poses[:cut], self._q[:cut]

    def _run(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            try:
                with self._factory() as client:
                    self.last_error = None
                    backoff = 1.0
                    for sample in client.stream():
                        if self._stop.is_set():
                            return
                        flange = flange_from(sample)
                        if flange is not None:
                            q = sample.get("actual_q")
                            self.add(self.clock(), flange, list(q) if q and len(q) == 6 else None)
            except Exception as exc:  # unreachable, RTDE disabled, dropped link: retry
                self.last_error = f"{type(exc).__name__}: {exc}"
            self._stop.wait(backoff)
            backoff = min(backoff * 2, 10.0)

    def at(self, t: float, max_age_s: float = 0.25) -> tuple[list[float] | None, float | None]:
        """The flange sample nearest host time ``t`` and its distance from ``t`` (s);
        ``(None, None)`` when there is none within ``max_age_s``."""
        flange, _, age = self.sample_at(t, max_age_s)
        return flange, age

    def sample_at(
        self, t: float, max_age_s: float = 0.25
    ) -> tuple[list[float] | None, list[float] | None, float | None]:
        """:meth:`at` plus the joints of the same sample: ``(flange, q, age)``."""
        with self._lock:
            if not self._t:
                return None, None, None
            i = bisect.bisect_left(self._t, t)
            best = min((j for j in (i - 1, i) if 0 <= j < len(self._t)), key=lambda j: abs(self._t[j] - t))
            age = abs(self._t[best] - t)
            if age > max_age_s:
                return None, None, None
            q = self._q[best]
            return list(self._poses[best]), (list(q) if q else None), age

    def latest(self) -> dict:
        with self._lock:
            if not self._t:
                return {"ok": False, "error": self.last_error or "no pose yet", "samples": self.samples}
            return {
                "ok": True,
                "flange_pose": list(self._poses[-1]),
                "age_s": round(self.clock() - self._t[-1], 3),
                "samples": self.samples,
                "error": self.last_error,
            }
