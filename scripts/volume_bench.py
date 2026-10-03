#!/usr/bin/env python3
"""How well the volume detector finds the part at arm's length — scored against ray-cast truth.

Random cells, each one frame: a camera 0.28-0.60 m over a level table, straight down or tilted
up to 25° (the picture pose, the closer look), 1-5 copies of a part laid out apart from each
other plus 0-2 things that are not the part, read through :class:`perceptronics.synthscene.Sensor`
(D435 noise, swells, stereo shadow, flying pixels, dropouts) and detected through a hand-eye
that is off by up to ``--cal-deg`` (the LEVEL lamp's error). Each true part is *found* when a
pickable part sits within half its width of where the detector should place it (in the
detector's own, mis-calibrated base frame); anything else pickable is a *false* pick.

    python3 scripts/volume_bench.py                 # 60 cells, realistic sensor
    python3 scripts/volume_bench.py -n 200 --sensor harsh --cal-deg 2.5 --why

Pure stdlib. Prints recall, false picks, centre / angle / size errors, and (``--why``) every
miss with the detector's reason.
"""

from __future__ import annotations

import argparse
import math
import random
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perceptronics.partspec import PartSpec  # noqa: E402
from perceptronics.synthscene import Box, Sensor, camera_looking_at, render_depth, sense  # noqa: E402
from perceptronics.volume import find_parts  # noqa: E402
from urctl.pose import Transform  # noqa: E402

W, H = 424, 240  # the D435's 848×480 at half size: same field of view, a quarter of the pixels
K = {"fx": 308.9, "fy": 309.1, "ppx": 213.3, "ppy": 121.7}
TABLE = -0.27
SENSORS = {
    "clean": Sensor(jitter_m=0.0, warp_m=0.0, baseline_m=0.0, skirt_px=0, dropout=0.0),
    "real": Sensor(dropout=0.15),
    "harsh": Sensor(jitter_m=0.001, warp_m=0.009, skirt_px=2, dropout=0.35),
}
# (length, width, height) mm, round — generic parts across the size range the node takes
PARTS = [
    (50, 30, 30, False),
    (60, 40, 20, False),
    (80, 25, 15, False),
    (30, 20, 12, False),
    (40, 40, 40, True),
    (25, 25, 10, True),
    (100, 50, 25, False),
]


def mm(*dims: float) -> str:
    return "x".join(f"{d * 1000:.0f}" for d in dims)


def cell(rng: random.Random, cal_deg: float):
    """One random cell: (true camera, detector's camera, boxes, the part's index set, spec)."""
    L, Wd, Hh, rnd = rng.choice(PARTS)
    spec = PartSpec.from_mm(L, Wd, Hh, 25.0, "cyl" if rnd else "box")
    height = rng.uniform(0.28, 0.60)
    tilt = math.radians(rng.choice([0, 0, 10, 18, 25]))
    head = rng.uniform(-math.pi, math.pi)
    target = (rng.uniform(0.25, 0.40), rng.uniform(-0.15, 0.15), TABLE)
    eye = (
        target[0] - math.cos(head) * math.tan(tilt) * height,
        target[1] - math.sin(head) * math.tan(tilt) * height,
        TABLE + height,
    )
    T = camera_looking_at(eye, target, (math.cos(head + 1.3), math.sin(head + 1.3), 0.0))
    T_cb = T.inverse()

    def visible(b: Box, margin: float = 0.10) -> bool:
        c, s = math.cos(b.theta), math.sin(b.theta)
        for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
            for zz in (0.0, b.height):
                p = (
                    b.x + su * b.length / 2 * c - sv * b.width / 2 * s,
                    b.y + su * b.length / 2 * s + sv * b.width / 2 * c,
                    TABLE + zz,
                )
                q = T_cb.apply(p)
                if q[2] < 0.12:
                    return False
                u, v = K["fx"] * q[0] / q[2] + K["ppx"], K["fy"] * q[1] / q[2] + K["ppy"]
                if not (margin * W < u < (1 - margin) * W and margin * H < v < (1 - margin) * H):
                    return False
        return True

    def clear(b: Box, others: list[Box], gap: float) -> bool:
        rb = math.hypot(b.length, b.width) / 2
        return all(
            math.hypot(b.x - o.x, b.y - o.y) > rb + math.hypot(o.length, o.width) / 2 + gap for o in others
        )

    boxes: list[Box] = []
    want = rng.randint(1, 5)
    spread = height * 0.45
    for _ in range(400):
        if len(boxes) >= want:
            break
        fl, fw, fh = rng.choice(spec.poses())  # a box may lie on any of its faces
        b = Box(
            target[0] + rng.uniform(-spread, spread),
            target[1] + rng.uniform(-spread, spread),
            fl,
            fw,
            fh,
            rng.uniform(-math.pi, math.pi),
            rnd,
        )
        if visible(b) and clear(b, boxes, 0.03):
            boxes.append(b)
    truth = set(range(len(boxes)))
    for _ in range(rng.randint(0, 2)):  # things that are not the part: a different size altogether
        for _ in range(200):
            k = rng.choice([0.45, 1.8, 2.4])
            b = Box(
                target[0] + rng.uniform(-spread, spread),
                target[1] + rng.uniform(-spread, spread),
                L * k / 1000,
                Wd * min(k, 1.5) / 1000,
                Hh * rng.choice([0.5, 1.0, 1.6]) / 1000,
                rng.uniform(-math.pi, math.pi),
            )
            if visible(b) and clear(b, boxes, 0.03):
                boxes.append(b)
                break
    # the hand-eye as the detector knows it: off by a rotation of up to cal_deg and ~1 mm/° of shift
    ax = (rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 1))
    n = math.sqrt(sum(a * a for a in ax))
    ang = math.radians(rng.uniform(0, cal_deg))
    err = Transform.from_pose(
        [rng.gauss(0, 1) * 0.001 * cal_deg for _ in range(3)] + [a / n * ang for a in ax]
    )
    return T, T.compose(err), boxes, truth, spec


def score(seed: int, sensor: Sensor, cal_deg: float, stride: int | None):
    rng = random.Random(seed)
    T, T_det, boxes, truth, spec = cell(rng, cal_deg)
    depth = render_depth(W, H, K, T, boxes, table_z=TABLE)
    depth = sense(depth, W, H, K, T, boxes, table_z=TABLE, sensor=sensor, seed=seed)
    t0 = time.perf_counter()
    scene = find_parts(W, H, depth, 0.001, K, T_det, spec=spec, stride=stride)
    dt = time.perf_counter() - t0
    to_det = T_det.compose(T.inverse())  # truth (base) → where the detector should put it
    rows = []
    used = set()
    for i in sorted(truth):
        b = boxes[i]
        c = to_det.apply((b.x, b.y, TABLE + b.height))
        hit = None
        for p in scene.parts:
            if id(p) not in used and math.dist(p.centre[:2], c[:2]) < b.width / 2:
                hit = p
                break
        if hit is not None:
            used.add(id(hit))
            ax_t = to_det.rotate((math.cos(b.theta), math.sin(b.theta), 0.0))
            d = (hit.theta - math.atan2(ax_t[1], ax_t[0])) % math.pi
            rows.append(
                (
                    "found",
                    math.dist(hit.centre[:2], c[:2]),
                    None if b.round or b.length < 1.2 * b.width else math.degrees(min(d, math.pi - d)),
                    hit.length_m - b.length,
                    hit.width_m - b.width,
                    hit.height_m - b.height,
                    None,
                )
            )
        else:
            near = min(scene.rejected, key=lambda p: math.dist(p.centre[:2], c[:2]), default=None)
            why = (
                near.why
                if near is not None and math.dist(near.centre[:2], c[:2]) < max(b.length, 0.03)
                else "nothing there"
            )
            if near is not None and why != "nothing there":
                got = mm(near.length_m, near.width_m, near.height_m)
                why += f" ({got} vs {mm(b.length, b.width, b.height)})"
            rows.append(("missed", None, None, None, None, None, why))
    false = [p for p in scene.parts if id(p) not in used]
    return rows, false, dt, spec, T


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-n", type=int, default=60, help="cells (default 60)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sensor", choices=sorted(SENSORS), default="real")
    ap.add_argument("--cal-deg", type=float, default=1.5, help="hand-eye rotation error up to this (deg)")
    ap.add_argument("--stride", type=int, default=None)
    ap.add_argument("--why", action="store_true", help="list every miss and false pick")
    a = ap.parse_args()
    found = missed = false_n = 0
    cerr, aerr, lerr, werr, herr, times, whys = [], [], [], [], [], [], {}
    for k in range(a.n):
        seed = a.seed * 100003 + k
        rows, false, dt, spec, T = score(seed, SENSORS[a.sensor], a.cal_deg, a.stride)
        times.append(dt)
        for kind, ce, ae, le, we, he, why in rows:
            if kind == "found":
                found += 1
                cerr.append(ce * 1000)
                lerr.append(le * 1000)
                werr.append(we * 1000)
                herr.append(abs(he) * 1000)
                if ae is not None:
                    aerr.append(ae)
            else:
                missed += 1
                key = why.split(" (")[0]
                whys[key] = whys.get(key, 0) + 1
                if a.why:
                    print(
                        f"  miss  seed {seed}  {spec.token()}  cam z {T.translation[2] - TABLE:.2f} m: {why}"
                    )
        false_n += len(false)
        if a.why:
            for p in false:
                print(
                    f"  FALSE seed {seed}  {spec.token()}  picked {mm(p.length_m, p.width_m, p.height_m)} mm"
                )

    def q(xs, f):
        return sorted(xs)[min(len(xs) - 1, int(f * len(xs)))] if xs else float("nan")

    total = found + missed
    print(
        f"{a.n} cells, sensor {a.sensor}, hand-eye ≤ {a.cal_deg}°: {found}/{total} found "
        f"({100 * found / max(1, total):.1f} %), {false_n} false picks"
    )
    print(f"  centre mm  median {q(cerr, 0.5):.1f}  p95 {q(cerr, 0.95):.1f}")
    print(f"  angle deg  median {q(aerr, 0.5):.1f}  p95 {q(aerr, 0.95):.1f}")
    print(
        f"  length mm  p5 / median / p95  {q(lerr, 0.05):+.1f} / {q(lerr, 0.5):+.1f} / {q(lerr, 0.95):+.1f}"
    )
    print(
        f"  width mm   p5 / median / p95  {q(werr, 0.05):+.1f} / {q(werr, 0.5):+.1f} / {q(werr, 0.95):+.1f}"
    )
    print(f"  height mm  |err| median {q(herr, 0.5):.1f}  p95 {q(herr, 0.95):.1f}")
    print(
        f"  detect     median {statistics.median(times) * 1000:.0f} ms"
        f"  max {max(times) * 1000:.0f} ms ({W}x{H})"
    )
    for why, n in sorted(whys.items(), key=lambda kv: -kv[1]):
        print(f"  missed {n:3d} × {why}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
