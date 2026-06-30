#!/usr/bin/env python3
"""Build Dance.urp — a multi-stage choreography for a real e-Series.

Stages: wake → bow → sweep → surge → wave → spiral → flourish → home.
Each stage runs at its own (a, v) so slow/dramatic and fast/snappy moves
contrast on the same arm.

Motion lives in a ``<Script type="File">`` helper called from a
``<Script type="Line">`` so the program loads + plays on the real robot
regardless of the active tool's TCP offset (native MoveJ/Waypoint nodes do
FK→IK through the active TCP and can stall on non-flange tools — see CLAUDE.md
→ "Build motion programs with script nodes").

All waypoints are orbits around the operator-set HOME pose (the spot the arm
was parked at by hand), so the entire dance stays inside the cell's known-safe
working envelope. Joints stay away from singularities (elbow nowhere near 0).
"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from urctl.urp_builder import UrpProgram  # noqa: E402

# Operator-parked home pose for this cell — the dance orbits this.
HOME = [-0.7427, -2.1034, -2.5358, -0.0718, 1.5714, 2.3782]

# Offsets we'll add to HOME, by joint index. Picked to stay reachable and
# clear of the elbow singularity (elbow stays in the [-2.9, -1.9] band).
#                       base    shldr   elbow   wrist1  wrist2  wrist3
READY        = [+0.00, +0.20, +0.40, -0.30, +0.00, +0.00]   # arm raised + extended
BOW          = [+0.00, -0.30, -0.20, +0.50, +0.00, +0.00]   # tuck/dip
SWEEP_L      = [+0.80, +0.10, +0.30, -0.20, +0.00, -0.40]
SWEEP_R      = [-0.80, +0.10, +0.30, -0.20, +0.00, +0.40]
SURGE_OUT    = [+0.00, +0.35, +0.60, -0.50, +0.00, +0.00]   # arm thrusts forward
SURGE_IN     = [+0.00, -0.25, -0.20, +0.30, +0.00, +0.00]
WAVE_A       = [+0.20, +0.10, +0.20, -0.40, +0.20, +0.80]
WAVE_B       = [+0.20, +0.10, +0.20, +0.40, -0.20, -0.80]
WAVE_C       = [-0.20, +0.10, +0.20, -0.40, +0.20, +0.80]
WAVE_D       = [-0.20, +0.10, +0.20, +0.40, -0.20, -0.80]
SPIRAL_HIGH  = [+0.00, +0.40, +0.40, -0.40, +0.00, +0.00]
FLOURISH_L   = [+0.50, +0.20, +0.30, -0.30, +0.00, -1.20]
FLOURISH_R   = [-0.50, +0.20, +0.30, -0.30, +0.00, +1.20]


def _waypoint(offset: list[float]) -> list[float]:
    return [round(h + d, 4) for h, d in zip(HOME, offset)]


def _movej(q: list[float], a: float, v: float, r: float | None = None) -> str:
    args = f"a={a}, v={v}"
    if r is not None:
        args += f", r={r}"
    return f"  movej({q}, {args})\n"


def build() -> UrpProgram:
    # Build the URScript body programmatically so the symbolic waypoint table
    # above stays the single source of truth — no hand-copied joint vectors.
    body = ["def dance():\n", '  textmsg("dance/stage=wake")\n']
    body.append(_movej(_waypoint(READY), a=0.8, v=0.6, r=0.05))

    body.append('  textmsg("dance/stage=bow")\n')
    body.append(_movej(_waypoint(BOW), a=0.8, v=0.5))
    body.append("  sleep(0.4)\n")
    body.append(_movej(_waypoint(READY), a=1.0, v=0.8, r=0.05))

    body.append('  textmsg("dance/stage=sweep")\n')
    for q, r in [(SWEEP_L, 0.08), (SWEEP_R, 0.08), (SWEEP_L, 0.08), (SWEEP_R, 0.08)]:
        body.append(_movej(_waypoint(q), a=1.2, v=1.0, r=r))
    body.append(_movej(_waypoint(READY), a=1.2, v=0.9, r=0.05))

    body.append('  textmsg("dance/stage=surge")\n')
    for _ in range(3):
        body.append(_movej(_waypoint(SURGE_OUT), a=1.6, v=1.3, r=0.04))
        body.append(_movej(_waypoint(SURGE_IN), a=1.6, v=1.3, r=0.04))
    body.append(_movej(_waypoint(READY), a=1.2, v=0.9, r=0.05))

    body.append('  textmsg("dance/stage=wave")\n')
    for q in (WAVE_A, WAVE_B, WAVE_C, WAVE_D, WAVE_A, WAVE_B):
        body.append(_movej(_waypoint(q), a=1.5, v=1.2, r=0.05))
    body.append(_movej(_waypoint(READY), a=1.2, v=0.9, r=0.05))

    body.append('  textmsg("dance/stage=spiral")\n')
    # Climb to SPIRAL_HIGH while wrist3 sweeps a full ±π — visible spin
    for k, dz in enumerate([-3.0, -1.5, 0.0, 1.5, 3.0, 1.5, 0.0, -1.5]):
        q = _waypoint(SPIRAL_HIGH)
        q[5] = round(q[5] + dz, 4)  # absolute wrist3 spin
        # last waypoint in this stage gets no blend (next stage's first move is a hard stop)
        r = None if k == 7 else 0.06
        body.append(_movej(q, a=1.4, v=1.4, r=r))

    body.append('  textmsg("dance/stage=flourish")\n')
    body.append(_movej(_waypoint(FLOURISH_L), a=1.6, v=1.3, r=0.06))
    body.append(_movej(_waypoint(FLOURISH_R), a=1.6, v=1.3, r=0.06))
    body.append(_movej(_waypoint(FLOURISH_L), a=1.6, v=1.3, r=0.06))

    body.append('  textmsg("dance/stage=home")\n')
    body.append(_movej(HOME, a=0.8, v=0.6))  # final waypoint — no r= (movej blend pitfall)
    body.append("  sleep(0.8)\n")
    body.append("  sync()\n")
    body.append("end\n")

    p = UrpProgram("Dance", installation="default", run_only_once=False)
    p.comment("Multi-stage dance: wake / bow / sweep / surge / wave / spiral / flourish / home")
    p.script_file("dance", "".join(body))
    p.script_line("dance()")
    return p


def main() -> None:
    out = Path(__file__).resolve().parent / "Dance.urp"
    build().save(out)
    print(f"wrote {out} ({out.stat().st_size} bytes gzipped)")


if __name__ == "__main__":
    main()
