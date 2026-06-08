---
name: ur-program-authoring
description: >-
  Author, convert, run, and save a URScript/PolyScope program for a UR e-Series
  robot in this repo — a choreography, pick-and-place, a motion routine, or any
  reusable .script/.urp under programs/. Use when asked to create/write/save a
  robot program or sequence, build a demo/dance/cycle, or convert between
  .script and .urp. Covers the programs/ conventions, the urp_convert workflow,
  installation pairing, how to run it (Primary vs Dashboard play), the URScript
  dialect traps (no random(), no nested defs, 2-arg textmsg, fixed arrays,
  pose_add), and the movej blend-radius pitfall that aborts randomized paths.
---

# Authoring a UR program

A "program" here is a `.script` (URScript) committed under `programs/<Name>/`,
converted to a PolyScope-loadable `.urp`, with a matching `.installation`. See
`programs/PickPlace`, `programs/AppleStack`, `programs/ElegantDance` as worked
references, and CLAUDE.md for the `.urp` schema. This skill is the procedure +
the dialect traps that cost real time. For naming/blend/Move-grouping
conventions, follow `docs/program-authoring-best-practices.md`.

## The shape every sample follows

```urscript
# Header: what it does, tunables, any vision/calibration assumptions, sim notes.
def MyProgram():
  textmsg("myprog/start")
  # ... motion + I/O, emitting "myprog/<checkpoint>" textmsg markers ...
  textmsg("myprog/done")
end

MyProgram()          # <- top-level call; without it nothing runs when played
```

- **Build stations relative to one reachable reference config**, not absolute
  literals: `movej(ref); p_ref = get_actual_tcp_pose()` then
  `pick = pose_add(p_ref, p[dx,dy,dz,0,0,0])`. `pose_add` offsets are in the
  **base frame**. This keeps the whole cycle inside the UR10 envelope on URSim
  regardless of cell layout. A good general-purpose `ref` is
  `[0.0, -1.2217, 1.5708, -1.9199, -1.5708, 0.0]` (front workspace, elbow folded).
- **Gripper** is modelled as standard digital output 0 (`set_standard_digital_out(0, True)`
  = closed). URSim has no physical gripper/objects, so a pick-and-place runs the
  full motion + I/O sequence but moves nothing real; on hardware, wire output 0.
- **Emit `textmsg("prefix/...", optional_value)` checkpoints.** They're how you
  verify a run end to end by scraping the Primary broadcast (see "Run it").
- A safe `home = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]` (candle) to start/end.

## Convert + pair the installation

```bash
python3 scripts/urp_convert.py to-urp programs/MyProgram/MyProgram.script \
    programs/MyProgram/MyProgram.urp --name MyProgram --installation MyProgram
cp programs/PickPlace/PickPlace.installation programs/MyProgram/MyProgram.installation
```

`load <name>` needs `<name>.urp` **and** a sibling `<name>.installation` in the
controller's `/programs` dir or you get `unknown failure`. Omit `--loop` (default)
so it runs once and reaches `STOPPED`; `--loop` makes it cycle forever.

## Run it (two ways)

**Over Primary (works in Local *or* Remote mode — best for dev/verification):**

```bash
urctl bring-up
urctl run-script --raw --capture --marker "myprog/" --collect-for 55 \
    < programs/MyProgram/MyProgram.script
```

`--raw` because the file defines its own `def` + call (don't let urctl wrap it
again). `--capture` holds the socket open for the whole run (reliable) and
returns the `myprog/` checkpoints — **confirm the final `myprog/done` appears**;
if the last marker you see is mid-sequence, the program aborted there (see
blend pitfall below).

**From PolyScope (Dashboard):** `docker cp` the `.urp` + `.installation` into
`/ursim/programs/`, then `urctl load MyProgram && urctl bring-up && urctl play`
(loading re-evaluates the installation and powers off; `play` needs Remote mode).

## URScript dialect traps (these bite)

- **No `random()`.** For randomized motion, inline a small LCG. Keep products
  inside double precision: modulus `m=16777216` (2^24), multiplier `16807`:
  `rng = rng*16807.0 + 1.0; rng = rng - floor(rng/m)*m; u = rng/m` (u in [0,1)).
  Seed from `get_actual_joint_positions()` to vary per run.
- **No nested `def`s.** Don't define helper functions inside another `def` —
  inline the logic. (This is also why running a self-contained program over
  Primary uses `--raw`, not urctl's auto-`def`-wrap.)
- **`textmsg` takes 2 args** (`textmsg(string, optional_value)`), not N. More
  args = compile error and the whole program silently won't run.
- **Math is available**: `sin cos tan sqrt pow floor ceil atan2 norm` etc. (all
  verified on URSim). No `format()`; `str_cat(a,b)` is binary — chain it.
- **Fixed-size arrays / pose literals.** Build a target as a fresh 6-element
  literal `[expr,expr,...]` each iteration; don't grow arrays.
- **`pose_add(a,b)`** = base-frame vector add of positions + rotation compose;
  with zero rotation in `b` it's a pure base-frame translation. Use `pose_trans`
  for tool-frame offsets.
- **`sync()` in tight compute loops.** A loop that does only arithmetic (no
  blocking motion call) can trip a non-real-time runtime error — keep motion
  primitives in the loop, or `sync()` each iteration.

## The movej blend-radius pitfall (cost a debugging cycle)

A non-zero blend radius `r` on `movej`/`movel` rounds corners for continuous,
flowing motion — but **if two consecutive waypoints are closer than `r`, the
controller aborts** the move ("blend radius too large for segment"), leaving the
robot stopped mid-program at a non-home pose (robot still RUNNING/NORMAL, so it
looks like a silent freeze; the marker capture stops before `*/done`).

For **randomized/generated** waypoints you can't rule this out by testing (the
seed changes every run), so either:
- **omit blends** (`r=0`) and get fluidity from fine waypoint spacing + high
  acceleration (what `ElegantDance` does), **or**
- keep a small `r` and **guarantee a minimum segment length** (skip waypoints
  closer than `r`).

For fixed, well-separated waypoints, modest blends are fine.

## Verify + save

- Run over Primary and confirm `*/done` is captured; `urctl state` shows the
  robot back home, `RUNNING`/`NORMAL`.
- Confirm the `.urp` loads: `urctl load MyProgram` → `STOPPED MyProgram.urp`.
- Add a load-guard integration test mirroring
  `tests/test_integration_ursim.py::test_bundled_apple_stack_urp_loads`, and a
  short `programs/MyProgram/README.md` (run instructions + assumptions).
- Commit the `.script`, `.urp`, `.installation`, `README.md` together. Don't bake
  real-robot kinematics into committed URPs — the converter's identity/checksum-
  off defaults work everywhere.
