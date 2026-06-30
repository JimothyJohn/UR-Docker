#!/usr/bin/env python3
"""Build Showcase.urp — a multi-stage choreography for a real e-Series.

Motion lives in a ``<Script type="File">`` helper called from a
``<Script type="Line">`` so the program loads + plays cleanly on a real robot
whose active TCP is not at the flange (native MoveJ/Waypoint nodes can fail
"Robot cannot reach the required pose" in that case — see CLAUDE.md).

``make regen-urps`` runs this. Standalone:  python programs/Showcase/build.py
"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from urctl.urp_builder import UrpProgram  # noqa: E402

SHOWCASE_DEF = """def showcase():
  v_j = 1.2
  a_j = 1.5
  b = 0.05

  home = [-0.7427, -2.1034, -2.5358, -0.0718, 1.5714, 2.3782]
  ready    = [0.0, -1.7, -1.5, -1.2, 1.5708, 0.0]
  look_l   = [0.6, -1.7, -1.5, -1.2, 1.5708, 0.0]
  look_r   = [-0.6, -1.7, -1.5, -1.2, 1.5708, 0.0]
  bow_low  = [0.0, -2.0, -1.0, -1.7, 1.5708, 0.0]
  high     = [0.0, -0.9, -1.8, -2.0, 1.5708, 0.0]
  swing_l  = [1.0, -1.4, -1.6, -1.6, 1.5708, 0.0]
  swing_r  = [-1.0, -1.4, -1.6, -1.6, 1.5708, 0.0]
  shim_a   = [0.3, -1.7, -1.5, -1.2, 1.9, 0.5]
  shim_b   = [-0.3, -1.7, -1.5, -1.2, 1.2, -0.5]
  shim_c   = [0.3, -1.7, -1.5, -1.2, 1.9, -0.5]
  shim_d   = [-0.3, -1.7, -1.5, -1.2, 1.2, 0.5]

  while True:
    textmsg("showcase/stage=wake")
    movej(ready, a=a_j, v=v_j*0.5)
    sleep(0.3)
    textmsg("showcase/stage=look")
    movej(look_l, a=a_j, v=v_j, r=b)
    movej(look_r, a=a_j, v=v_j, r=b)
    movej(ready, a=a_j, v=v_j)
    textmsg("showcase/stage=bow")
    movej(bow_low, a=a_j, v=v_j*0.5)
    sleep(0.3)
    movej(ready, a=a_j, v=v_j*0.5)
    textmsg("showcase/stage=swoop")
    movej(swing_l, a=a_j, v=v_j, r=b)
    movej(high, a=a_j, v=v_j, r=b)
    movej(swing_r, a=a_j, v=v_j, r=b)
    movej(high, a=a_j, v=v_j, r=b)
    movej(ready, a=a_j, v=v_j)
    textmsg("showcase/stage=shimmy")
    i = 0
    while i < 2:
      movej(shim_a, a=a_j, v=v_j, r=0.04)
      movej(shim_b, a=a_j, v=v_j, r=0.04)
      movej(shim_c, a=a_j, v=v_j, r=0.04)
      movej(shim_d, a=a_j, v=v_j, r=0.04)
      i = i + 1
    end
    movej(ready, a=a_j, v=v_j)
    textmsg("showcase/stage=pulse")
    k = 0
    while k < 3:
      movej(high, a=a_j, v=v_j*1.3, r=0.02)
      movej(ready, a=a_j, v=v_j*1.3, r=0.02)
      k = k + 1
    end
    textmsg("showcase/stage=home")
    movej(home, a=a_j, v=v_j*0.6)
    sleep(1.5)
    sync()
  end
end
"""


def build() -> UrpProgram:
    p = UrpProgram("Showcase", installation="default", run_only_once=False)
    p.comment("Showcase choreography: wake / look / bow / swoop / shimmy / pulse / home")
    p.script_file("showcase", SHOWCASE_DEF)
    p.script_line("showcase()")
    return p


def main() -> None:
    out = Path(__file__).resolve().parent / "Showcase.urp"
    build().save(out)
    print(f"wrote {out} ({out.stat().st_size} bytes gzipped)")


if __name__ == "__main__":
    main()
