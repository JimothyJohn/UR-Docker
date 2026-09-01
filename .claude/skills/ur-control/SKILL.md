---
name: ur-control
description: >-
  Drive the URSim simulator (or a real UR e-Series robot) end to end via the
  urctl toolkit — start the sim, cold-start to RUNNING, move (joints or the TCP
  in Cartesian/base-frame, including relative nudges/jogs), run URScript,
  load/play PolyScope programs, and read state. Use when the task is to operate,
  control, move, jog, nudge, swing, home, power, or run a program on a Universal
  Robots controller in this repo, or to debug why a robot/sim won't power on,
  load, play, or actually move. Covers the non-obvious sequencing (power → brake
  release, load → re-arm), reliable vs racy motion, protective-stop recovery, and
  the URSim-specific gotchas.
---

# Controlling a UR robot with urctl

This repo talks to a UR e-Series controller — URSim in Docker by default, or a
real robot at an IP — over its native network protocols. The reusable layer is
`urctl/` (a `Robot` facade + safety envelope + audit log), surfaced three ways
that all sit on the same `Robot`: the `urctl` CLI, the agent tool registry
(`urctl.tools`), and the `urctl-mcp` MCP server.

**Read `CLAUDE.md` for the protocol/format reference** (port map, Dashboard
cheat sheet, `.urp` schema, URScript dialect gotchas). This skill is the
*procedural* side: the exact sequences to get something done and how to verify
each step.

## First: pick the target and confirm it's reachable

Everything defaults to `localhost` (the URSim container) but is target-agnostic.
Set the target once and reuse it:

- CLI: `urctl --host <ip> <cmd>` or export `UR_HOST=<ip>`.
- Also honored: `UR_DASH_PORT` (29999), `UR_PRIMARY_PORT` (30001), `UR_TIMEOUT_S`.

Confirm the controller is actually up (not just the port open) before doing
anything else:

```bash
urctl state          # prints robot_mode / safety_mode / program_state as JSON
```

If this errors or shows `NO_CONTROLLER`, the controller hasn't booted — see
**Sim lifecycle** below. Every urctl command prints structured JSON and exits
non-zero when it reports `ok=false`; check the exit code in scripts.

## Sim lifecycle (URSim only)

```bash
make sim-up          # docker compose up -d  (VNC at http://localhost:6080/vnc.html)
make sim-down        # stop + remove the container
make sim-logs        # tail container logs
make sim-shell       # shell inside the container
```

URControl takes ~30s to boot after `sim-up`. Poll `urctl state` until
`robot_mode` is no longer `NO_CONTROLLER`. The compose healthcheck already
waits on this, so `docker compose ps` showing `healthy` is the green light.

If PolyScope shows **"No Controller"** that's the seccomp/ENOSYS issue — the
compose file already sets `seccomp:unconfined`; if you changed compose, restore
it. (Full symptom→cause table is in CLAUDE.md.)

## Cold start: power off → RUNNING

A freshly-booted controller is powered off with brakes engaged. One command
does the whole arming sequence (power on → wait → brake release → wait):

```bash
urctl bring-up                       # localhost sim
urctl --host 10.0.0.5 bring-up       # a real robot
# or, shell-only, no Python install:  make sim-poweron   (runs scripts/poweron.sh)
```

Verify: `urctl state` should report `robot_mode: RUNNING` and
`safety_mode: NORMAL`. URSim transitions fast — POWER_OFF→RUNNING can happen in
one tick, so don't assert you saw an intermediate `IDLE`.

**Local vs Remote control matters for `play`.** If Dashboard `play` returns
`Failed to execute`, PolyScope is in *Local* mode — flip it to *Remote* in the
pendant's top-right corner (over VNC). Powering/moving via URScript still works
in Local; only program playback needs Remote.

## Moving the robot

Two validated motion commands. Both run the target through the **safety
envelope** (bounds + RUNNING check) and the **audit log**, both wrap the URScript
in a `def`, and both **hold the Primary socket open until the move confirms** —
so they are *reliable* and return as soon as the move lands (not after a fixed
timeout). Prefer these over hand-rolled motion.

**Joint space** (`movej`), radians, order `[base, shoulder, elbow, w1, w2, w3]`:

```bash
urctl move-home                                       # candle pose [0,-π/2,0,-π/2,0,0]
urctl move-joints 0 -1.57 0 -1.57 0 0 --velocity 1.0 --acceleration 1.5
```

**Cartesian / linear** (`movel`) — `move-tcp`, pose `x y z rx ry rz` (m + rotvec):

```bash
urctl move-tcp 0 0.05 0 0 0 0 --relative              # nudge +50 mm along base +Y
urctl move-tcp -0.19 -0.30 0.40 0 3.14 0              # absolute base-frame pose
urctl move-tcp 0 0 0.10 0 0 0 --relative --velocity 0.1
```

`--relative` adds the pose to the **live TCP in the base frame** (exact wherever
the arm is) — this is the "nudge/jog" path.

**Direction conventions** (base frame, right-handed: X fwd, Y left, Z up):
"up/down" → ±Z; "left/right" → ±Y (viewpoint flips left/right — confirm if it
matters; in this repo's sessions "right" has meant **+Y**). A "nudge" is a small
relative `move-tcp`; pick a sane default (20–50 mm) and say what you used.

**Dry-run untrusted/generated poses first** — exact safety check + audit write,
sends nothing; fix `ok=false` violations rather than widening the envelope:

```bash
urctl --dry-run move-joints 99 0 0 0 0 0
urctl --dry-run move-tcp 0 5 0 0 0 0 --relative        # rejected: tcp_step (unit-error guard)
```

**Safety caps** (defaults in `urctl/safety.py`; tune via `SafetyEnvelope`):
joint speed ≤ **2.09 rad/s** (120°/s — the UR10 base-joint limit, so there's no
real "even faster" headroom past it without exceeding a hardware spec); TCP speed
≤ 1.0 m/s; absolute reach ≤ 1.3 m; a single **relative** TCP step > 1.0 m is
rejected as a likely unit error (inches/mm typed as metres).

Hand-guiding: `urctl freedrive on` / `urctl freedrive off`. Auditable log:
`urctl --audit-log run.jsonl move-joints ...`.

### Jogging far / "as far as you can"

Don't command one giant move to the limit — it faults or trips a protective stop
mid-move. **Step** in the direction (e.g. +Z by 0.1 m), re-read `urctl state`
each step, and stop when the value stops increasing or `safety_mode` leaves
`NORMAL`; shrink the step near the limit for precision. A protective stop leaves
the robot safe and put — recover and continue with:

```bash
urctl dashboard unlock protective stop && urctl dashboard close safety popup
```

"Swing" implies an **arc** → use `move-joints` (joint-space interpolation curves
the TCP) rather than a straight `move-tcp`.

## Running raw URScript (escape hatch — prefer move-tcp / move-joints)

Primary 30001 runs URScript immediately (execution; Dashboard is orchestration —
don't mix them). `run-script` **wraps the script in a `def` by default** so it
runs as one program — required because a bare top-level `movel`/`movej` is
otherwise split off / superseded and **silently never runs**. `--raw` opts out
(only for scripts that already define their own top-level functions, which can't
be nested inside another `def`).

```bash
urctl run-script 'popup("hi")'
urctl run-script --capture --marker "x/" --collect-for 5 'textmsg("x/done", 1)'
urctl run-script --raw --capture --collect-for 50 < programs/Foo/Foo.script   # run a whole program file
```

**Fire-and-forget motion via run-script is racy** — closing the Primary socket
right after sending can drop the program before the controller latches it. For
guaranteed motion use `move-tcp`/`move-joints` (they hold the socket open until
the move confirms) or `run-script --capture` (also held open). `--capture` reads
the broadcast for `--collect-for` seconds and returns lines containing `--marker`
— how you get `textmsg` output back.

URScript dialect has real teeth (no `random()`, no nested `def`s, 2-arg
`textmsg`, fixed-size arrays, `str_cat` is binary). For writing programs or
choreography, use the **ur-program-authoring** skill; for picking objects located
in a photo, the **ur-pick-from-image** skill.

## Loading and playing PolyScope programs

```bash
urctl load MotionDemo        # loads /programs/MotionDemo.urp
urctl play                   # start it
urctl pause / urctl stop
```

`load <name>` needs both `<name>.urp` **and** a sibling `<name>.installation`
in the controller's `/programs` dir, or you get `unknown failure`. Loading a
fresh installation makes PolyScope re-evaluate safety and **power the robot
off** — so the reliable sequence is:

```bash
urctl load MotionDemo && urctl bring-up && urctl play
```

(The standalone `scripts/e2e_drive.py` does exactly this load→re-arm→play dance
end to end; read it if you need a worked reference.) A program built with
`runOnlyOnce="false"` loops forever and never reaches `STOPPED` — regenerate
the `.urp` without `--loop` if you need it to terminate (see urp authoring).

## Authoring a new program

Writing a `.script`/`.urp` (choreography, pick-and-place, a routine)? That's its
own skill — **ur-program-authoring** — covering the converter workflow, the
URScript dialect traps, the movej blend-radius pitfall, and how to run/load what
you build. Use it instead of guessing.

## When something goes wrong

Tail the controller's own log — it's far more specific than Dashboard's
"unknown failure":

```bash
sudo docker exec ur-docker-ursim-1 tail -f /ursim/polyscope.log
```

The CLAUDE.md **Common gotchas** table maps symptom → cause → fix for the
recurring ones (No Controller, Modbus won't bind, play fails in Local mode,
load failures, never-stopping programs, post-load power-off). Check it before
guessing.

## One-place-to-add-a-capability rule

If the task is to add a *new* robot capability (not just use one): implement it
as a `Robot` method, then surface it as a `Tool` in `urctl/tools.py` (schema +
handler) and, if humans need it, a subcommand in `urctl/cli.py`. The CLI, tool
registry, and MCP server all share the one `Robot`, so they stay in lockstep.
Keep every mutating action routed through the safety envelope + audit log.

## Verifying your work

- `urctl state` after each phase — assert the expected `robot_mode` /
  `safety_mode` / `program_state`.
- `make test` (unit, no sim) and `make test-integration` (needs a running
  URSim) before declaring done.
- `make lint` (ruff + shellcheck). Don't `--no-verify` past pre-commit hooks —
  they catch URScript/URP schema regressions.
- Never push to `main` (blocked); use a feature branch + PR.
