"""Camera latency fit — the one number the software-timed sweep needs.

A frame is stamped when it *arrives*; it was exposed ``latency_s`` earlier.
Given per-frame camera positions measured independently of the robot (PnP on
the ChArUco plate, :mod:`perception.charuco`) and the recorder's flange
trajectory, the latency is the shift that makes the two agree
(`docs/mono-scan.md` §2.1). A constant offset between the two trajectories
(a hand-eye translation error) is absorbed, so this fits *timing only*.

Two fits share the search: :func:`fit_latency` (measured camera positions vs
the recorder — the ChArUco path) and :func:`fit_latency_by_objects` (**no
plate needed**: re-stamp a sweep over known parts at each candidate latency
and keep the one where the box model fits the outlines best — the IoU the
locator already reports is the objective).

Pure stdlib here; the object path calls the locator (numpy + OpenCV).
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

Vec3 = tuple[float, float, float]


def _rms(residuals: Sequence[Vec3]) -> float:
    n = len(residuals)
    mean = tuple(sum(r[i] for r in residuals) / n for i in range(3))
    return math.sqrt(sum(sum((r[i] - mean[i]) ** 2 for i in range(3)) for r in residuals) / n)


def fit_latency(
    measured: Sequence[tuple[float, Sequence[float]]],
    position_at: Callable[[float], Sequence[float] | None],
    *,
    search_s: tuple[float, float] = (-0.05, 0.25),
    coarse_s: float = 0.002,
) -> dict:
    """``measured`` = ``[(host_t, [x, y, z]), ...]`` camera (or flange) positions;
    ``position_at(t)`` the recorder's position for the same body. Returns
    ``{latency_s, rms_m, rms_at_zero_m, used, ...}``."""
    if len(measured) < 3:
        raise ValueError("need at least 3 measured positions")

    def cost(lag: float) -> float | None:
        res = []
        for t, p in measured:
            q = position_at(t - lag)
            if q is None:
                continue
            res.append((p[0] - q[0], p[1] - q[1], p[2] - q[2]))
        if len(res) < 3:
            return None
        return _rms(res)

    lo, hi = search_s
    grid = [lo + i * coarse_s for i in range(int((hi - lo) / coarse_s) + 1)]
    scored = [(c, g) for g in grid if (c := cost(g)) is not None]
    if not scored:
        raise ValueError("no overlap between the measured frames and the recorded trajectory")
    best_c, best = min(scored)
    a, b = max(lo, best - 2 * coarse_s), min(hi, best + 2 * coarse_s)
    phi = (math.sqrt(5) - 1) / 2
    for _ in range(30):
        c1, c2 = b - phi * (b - a), a + phi * (b - a)
        f1, f2 = cost(c1), cost(c2)
        if f1 is None or f2 is None:
            break
        if f1 < f2:
            b = c2
        else:
            a = c1
    lag = (a + b) / 2
    final = cost(lag)
    zero = cost(0.0)
    return {
        "latency_s": lag,
        "rms_m": final if final is not None else best_c,
        "rms_at_zero_m": zero,
        "used": len(measured),
        "search_s": list(search_s),
        "improvement": None if zero is None or final is None or zero <= 0 else 1.0 - final / zero,
    }


def fit_latency_by_objects(
    sweep,
    library,
    *,
    search_s: tuple[float, float] = (0.0, 0.15),
    coarse_s: float = 0.01,
    fine_s: float = 0.002,
    max_frames: int = 8,
) -> dict:
    """Latency that maximises the mean box-fit IoU over the located objects of
    ``sweep`` (a :class:`perception.sweep.Sweep` with a trajectory) — coarse
    grid then a fine grid around the best. ``library`` pins the part dims.
    Returns ``{latency_s, iou, iou_at_zero, profile: [[lag, iou], ...]}``."""
    from .locate2d import locate_objects

    frames_all = [f for f in sweep.frames]
    if len(frames_all) > max_frames:
        step = len(frames_all) / max_frames
        keep = {int(i * step) for i in range(max_frames)}
        frames_all = [f for i, f in enumerate(frames_all) if i in keep]

    def score(lag: float) -> float | None:
        s = sweep.restamp(lag)
        pairs = []
        for f in s.frames:
            if f.host_t in {g.host_t for g in frames_all}:
                t = f.t_base_cam(s.flange_to_color)
                if t is not None:
                    pairs.append((f.gray, t))
        if len(pairs) < 3:
            return None
        objs = locate_objects(pairs, s.intrinsics, s.plane, library=library, free_dims=False)
        if not objs:
            return None
        return sum(o.fit["iou"] for o in objs if o.fit and o.fit.get("iou") is not None) / len(objs)

    lo, hi = search_s
    grid = [lo + i * coarse_s for i in range(int(round((hi - lo) / coarse_s)) + 1)]
    profile = [(g, score(g)) for g in grid]
    scored = [(v, g) for g, v in profile if v is not None]
    if not scored:
        raise ValueError("no objects located at any latency — check the table plane and the parts")
    best_v, best = max(scored)
    fine = [best + k * fine_s for k in range(-int(coarse_s / fine_s) + 1, int(coarse_s / fine_s))]
    fine_profile = [(g, score(g)) for g in fine if lo <= g <= hi and g not in grid]
    profile = sorted(profile + fine_profile)
    scored = [(v, g) for g, v in profile if v is not None]
    best_v, best = max(scored)
    zero = next((v for g, v in profile if abs(g) < 1e-9), None)
    return {
        "latency_s": best,
        "iou": best_v,
        "iou_at_zero": zero,
        "profile": [[g, v] for g, v in profile],
        "method": "objects",
    }
