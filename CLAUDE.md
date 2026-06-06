# CLAUDE.md

Working notes for AI assistants (and humans) on this repo. This is a
sandbox + tooling repo for **Universal Robots e-Series** control. Most of
what's interesting here lives in the network protocols and file formats UR
ships — not in the code itself — so this file captures the things that took
me real time to discover.

## What this repo is

A Docker-Compose'd PolyScope simulator (URSim 5.12.5, e-Series) plus a
small set of host-side tools that talk to it the way you'd talk to a real
controller. Goal: make experiments with URScript, the Dashboard API, and
PolyScope program files reproducible without needing a real robot.

```
docker-compose.yml          URSim container (UR10, ports 5900/6080/29999-30001)
urctl/                      Host-side control library + CLI + MCP server + agent tools
scripts/poweron.sh          Cold-start sequence over Dashboard (29999)
scripts/urp_convert.py      .script <-> .urp converter (PolyScope program files)
programs/InspectionBot/     Sample teach-by-prompt program (HelpfulBot-style)
tests/                      pytest suite (unit + URSim integration)
```

`urctl/` is the reusable, network-target-agnostic layer: `Robot` (facade over
the Dashboard + Primary clients), a `SafetyEnvelope` (pre-execution
validation), an `AuditLog` (structured JSON action log), a JSON-schema'd tool
registry (`urctl.tools`) for agent frameworks (OpenClaw/ROSClaw, MCP, plain
function-calling), an `urctl` CLI, and an `urctl-mcp` MCP server. The older
`scripts/` are kept as standalone shell/Python helpers; `urctl.urp` re-exports
the converter so `scripts/urp_convert.py` stays the single source of truth.

## Assistant skills (procedural how-tos)

Task-triggered skills live in `.claude/skills/` and encode the *procedures* that
go with this reference doc — use them, don't re-derive:

- **ur-control** — operate the robot/sim: power on, read state, move joints, jog
  the TCP (`move-tcp`, relative base-frame nudges), reliable-vs-racy motion,
  jogging to a limit + protective-stop recovery, load/play programs.
- **ur-program-authoring** — write/convert/run/save a `.script`/`.urp` program:
  conventions, URScript dialect traps, and the movej blend-radius pitfall.
- **ur-pick-from-image** — turn a photo of objects into a pick/stack program when
  there's no camera calibration (scale-from-object-size + anchor-the-cluster).

**Connection target is configurable, not hardcoded.** Nothing in `urctl/` (or
`poweron.sh` / `e2e_drive.py`) bakes in `localhost` — they default to it for
dev but read `UR_HOST` / `UR_DASH_PORT` / `UR_PRIMARY_PORT` / `UR_TIMEOUT_S`
(and `RobotConfig.from_env(host=...)` / `--host`) so the same code drives a
real robot at an IP. The only `localhost` references that *should* stay are the
docker-compose healthcheck and the CI startup probe — those run *inside the
container / on the runner*, where `localhost` is correct.

## Network surface of a UR controller

All ports are bound to localhost by docker-compose. On real hardware they
are unauthenticated; treat the robot's network like a backplane.

| Port  | Name              | Direction | What it's for                                                          |
| ----- | ----------------- | --------- | ---------------------------------------------------------------------- |
| 29999 | Dashboard server  | bi        | Line-oriented ASCII. The "press buttons in PolyScope" surface.         |
| 30001 | Primary Client    | bi        | 10 Hz state broadcast **and** accepts raw URScript that runs immediately. |
| 30002 | Secondary Client  | read      | Read-only mirror of Primary's broadcast for external monitors.         |
| 30003 | RT Interface      | read      | 125/500 Hz state — legacy hard-realtime consumers (pre-RTDE).          |
| 30004 | RTDE              | bi        | Subscription protocol: client picks fields + cadence. Modern drivers.  |
| 30020 | Interpreter mode  | bi        | URScript REPL that runs **inside an already-loaded program**.          |
| 502   | Modbus TCP        | bi        | Field I/O. Needs `NET_BIND_SERVICE` to bind (handled in compose).      |

The key distinction: **Dashboard is for orchestration** (load program,
power on, query state). **Primary 30001 is for execution** (write URScript,
it runs). Most beginner confusion stems from trying to play a program via
Primary or write URScript via Dashboard.

### Dashboard cheat sheet

```
power on          power off          brake release
robotmode         safetymode         programState         running
load <prog>.urp   play               stop                 pause
unlock protective stop                close safety popup
popup <text>      addToLog <text>
```

`load <name>.urp` requires the matching `<installation>.installation` file
in the same directory — PolyScope refuses to load a program whose
installation isn't found. Error text in that case is the cryptic
`Error while loading program: ...:unknown failure`; the real cause shows
up in `/ursim/polyscope.log` as `Failed to load installation: null.installation`.

### Talking to Dashboard from shell

```bash
printf 'robotmode\nquit\n' | nc -q1 localhost 29999
```

The server replies one line per command and only after it has fully
executed. Pipelining requires either `sleep` between writes or a tool that
waits for replies (the test suite wraps this in `dash()` and `wait_for()`).

## PolyScope X: the REST Robot-API (second platform)

The repo can also run **PolyScope X** (PolyScope 10) via the `ursim-px`
compose service (`make simx-up`, web UI on `localhost:8000`). PolyScope X is a
*different controller platform*, not a newer e-Series — and the remote surface
changed enough to matter:

- **No Dashboard server (29999).** Orchestration moved to a RESTful **Robot-API**
  served over HTTP at `http://<host>:<port>/universal-robots/robot-api`
  (self-documented at `…/robot-api/docs`, OpenAPI at `…/robot-api/openapi.json`).
- **Primary (30001) and RTDE (30004) still exist** and URScript execution is
  unchanged — but both interfaces are **off by default** and must be enabled
  once in the UI: hamburger → Settings → Security → Services → enable *Primary
  Client Interface* (+ RTDE). The `ursim-px` service publishes them on **offset
  host ports** (`31001`→30001, `31004`→30004) so it can coexist with the
  e-Series sim's 30001/30004.
- **Remote-mode gate is stricter.** *Every* mutating Robot-API call (power on,
  brake release, load, play) returns **HTTP 403** unless the robot is in
  **Remote** control mode, and there is **no REST endpoint to switch
  Local→Remote** — it's a toggle on the Safety screen in the UI. (On e-Series
  only Dashboard `play` needed Remote; Primary URScript did not.)

`urctl` drives PolyScope X with the **same commands** — select the platform with
`--platform polyscopex` (or `UR_PLATFORM=polyscopex`) and point the Robot-API at
the right port (`UR_ROBOT_API_PORT=8000` for the sim):

```bash
# read state (REST Robot-API) — works in Local mode
UR_PLATFORM=polyscopex UR_ROBOT_API_PORT=8000 urctl state
# or the Make shortcuts (set the env for you):
make simx-state
make simx-bring-up        # power on + brake release (needs Remote mode)
# motion rides Primary 31001 (needs the Primary interface enabled in the UI):
UR_PLATFORM=polyscopex UR_ROBOT_API_PORT=8000 UR_PRIMARY_PORT=31001 \
  urctl --platform polyscopex move-tcp 0 0.05 0 0 0 0 --relative
```

Implementation: `urctl/robotapi.py`'s `RobotAPIClient` is a **drop-in for**
`DashboardClient` (same method names + return-string conventions), injected by
`Robot.__init__` when `config.platform == "polyscopex"`. `PrimaryClient` is
reused unchanged. Verified Robot-API endpoints:

| Need | Robot-API call |
| ---- | -------------- |
| robot / safety mode | `GET /robotstate/v1/{robotmode,safetymode}` → `{mode,message}` |
| power / brake / unlock | `PUT /robotstate/v1/state` `{"action": POWER_ON\|POWER_OFF\|BRAKE_RELEASE\|UNLOCK_PROTECTIVE_STOP\|RESTART_SAFETY}` |
| program state | `GET /program/v1/state`; `PUT` `{"action": play\|pause\|resume\|stop}` |
| load program | `PUT /program/v1/loaded` `{"name": "<program>"}` (PolyScope X programs are `.urpx`, addressed by name) |
| control / op mode | `GET /system/v1/{controlmode,operationalmode}` → `LOCAL\|REMOTE` / `AUTOMATIC\|MANUAL` |

| Symptom | Cause | Fix |
| ------- | ----- | --- |
| Robot-API returns `403 Forbidden` on power/brake/load/play | Robot is in Local control mode | Switch to Remote on the Safety screen in the PolyScope X UI; there is no network way to do it |
| `move-tcp`/`run-script` can't reach Primary on `:31001` | Primary Client Interface not enabled, or container doesn't expose it | Enable it in Settings → Security → Services; if still unreachable see the port-exposure note in the PolyScope X service comments |

## PolyScope file formats

`.urp` and `.installation` are both **gzipped XML** — not the XStream Java
serialization that the format name might suggest. You can read them with
`zcat file | xmllint --format -`.

### Minimum schema for a `.urp` PolyScope will load

(This is what cost me the most time to figure out — it was reverse-engineered
by diffing against real saved URPs and reading polyscope.log error messages
after each failed `load`. Encoded in `scripts/urp_convert.py:script_to_urp`.)

```xml
<URProgram name="X" installation="Y" installationRelativePath="Y"
           directory="/programs" createdIn="5.12.5.10626004"
           lastSavedIn="5.12.5.10626004" robotSerialNumber="">
  <kinematics status="NOT_LINEARIZED" validChecksum="false">
    <deltaTheta value="0,0,0,0,0,0"/>
    <a value="0,0,0,0,0,0"/>
    <d value="0,0,0,0,0,0"/>
    <alpha value="0,0,0,0,0,0"/>
    <jointChecksum value="0,0,0,0,0,0"/>
  </kinematics>
  <children>
    <MainProgram runOnlyOnce="false" InitVariablesNode="false">
      <children>
        <Script type="File">
          <cachedContents>...URScript...</cachedContents>
          <file resolves-to="file">/programs/X.script</file>
        </Script>
      </children>
    </MainProgram>
  </children>
</URProgram>
```

Four things PolyScope demands, all easy to miss:

1. `installation=` + `installationRelativePath=` naming a sibling
   `<name>.installation` file in the same directory.
2. A `<kinematics>` block. Even with `validChecksum="false"` the block
   itself must exist or `ProgramRootNodeLoad` returns null.
3. A `<MainProgram>` wrapper between top-level `<children>` and the actual
   program nodes.
4. `<Script type="File">` carries `<cachedContents>` (inline text) *and*
   `<file resolves-to="file">` (source path). `type="Code"` is for inline
   "Script Code" nodes and doesn't carry the file pointer.

The `<kinematics>` values can be zero/identity for URSim; real hardware
prefers values matching the installed robot's actual calibration.

## URScript dialect notes

These are gotchas I hit writing `programs/InspectionBot/InspectionBot.script`:

- **No resizable arrays.** Pose lists are fixed-size literals. Use a
  separate counter variable for "active length".
- **Function-scoped locals.** `local x = ...` is hoisted to the enclosing
  function. Putting `local` inside a `while` loop works syntactically but
  doesn't create a per-iteration variable; prefer top-of-function declarations.
- **No block-scoped locals + global assignment.** Assigning to a name
  without `local` inside a function writes the global. To avoid clobbering,
  pick distinct names or declare `local` upfront.
- **`request_*_from_primary_client` blocks indefinitely** until a client
  on Primary 30001 (PolyScope's own UI counts) answers. Useful in wizards;
  terrible if you forget there's no UI session attached.
- **`freedrive_mode([axes], feature=...)`** without args defaults to all
  six axes free. The axes list is `[x, y, z, rx, ry, rz]` in the chosen
  feature frame.
- **`str_cat(a, b)`** takes exactly two args. Chain it for >2 strings.
  `to_str(x)` stringifies anything; no `format()`.
- **`sync()`** must be called every iteration of a tight loop in URScript
  or the controller throws a runtime error about non-real-time execution.

## Executing motion over Primary 30001 (the part that bit hard)

Getting a Cartesian move to actually run cost real time; here is the verified
mental model. `urctl.Robot.move_tcp` / `move_joints` encode all of it — prefer
them over hand-rolled URScript.

- **Wrap motion in a `def`.** URControl treats each newline-terminated
  top-level statement as its own program and kills the previous one. So a raw
  multi-line `target = ...` / `movel(target)` splits into two programs and the
  second runs with `target` undefined. Send motion as a single
  `def f(): … end\n f()` (what `urctl` does) — or, for a one-liner, fold it
  into one self-contained statement: `movel(pose_add(get_actual_tcp_pose(),
  p[0,0.05,0,0,0,0]), a=0.3, v=0.1)`.
- **Fire-and-forget is racy.** Writing motion and immediately closing the
  Primary socket (`transport.send`) sometimes lands and sometimes doesn't —
  the controller may not have latched the program before the FIN. Observed
  both outcomes for the *same* bare `movel`. The reliable pattern is to **hold
  the socket open until the move confirms**: run the body, `sync()`, then
  `textmsg("urctl/move/done=", get_actual_tcp_pose())`, and read the broadcast
  until that marker appears. `move_tcp`/`move_joints` do this and return as
  soon as the marker lands (see `transport.send_and_collect(stop_marker=...)`)
  — so they're both reliable *and* fast (don't block the full timeout).
- **Local vs Remote control does NOT gate Primary URScript.** Motion sent to
  30001 runs whether PolyScope is in Local or Remote mode (verified: a
  def-wrapped `movel` moved the robot with `is in remote control` → `false`).
  Only the **Dashboard `play` command** requires Remote mode. Don't add a
  remote-control precondition to motion — it would reject moves that work.
- **Relative moves are base-frame.** `move_tcp(..., relative=True)` adds the
  delta to the live TCP via `pose_add(get_actual_tcp_pose(), p[...])`, so the
  XYZ delta is in the **base** frame (use `pose_trans` if you ever want the
  tool frame). The safety envelope caps a single relative step (default 1.0 m)
  to catch unit mistakes (inches/mm entered as metres).

## Common gotchas (and the symptoms that lead you there)

| Symptom                                       | Cause                                                        | Fix                                                 |
| --------------------------------------------- | ------------------------------------------------------------ | --------------------------------------------------- |
| PolyScope shows "No Controller"               | URControl crashed at boot — usually `socket() ENOSYS`        | `security_opt: [seccomp:unconfined]` in compose     |
| Modbus refuses to start                       | Port 502 needs root                                          | `cap_add: [NET_BIND_SERVICE]` in compose            |
| Dashboard `play` returns `Failed to execute`  | PolyScope is in Local control mode                           | Toggle Local→Remote in PolyScope's top-right corner |
| `load` returns `unknown failure`              | URP missing `installation=` attr or `<kinematics>`           | Use `urp_convert.py` or inspect `polyscope.log`     |
| Robot state polling sees no IDLE              | URSim is fast — POWER_OFF→RUNNING in one tick                | Accept `IDLE\|RUNNING` from `wait_for`              |
| `.script` files invisible in Load Program     | Default file filter is `*.urp`                               | Flip the filter, or convert with `urp_convert.py`   |
| URP plays forever — never reaches STOPPED     | `runOnlyOnce="false"` (PolyScope's UI default for cycle programs) | Regenerate URP without `--loop` (`urp_convert.py` default is `true`) |
| Robot powers off after `load <name>.urp`      | Fresh `<name>.installation` triggers PolyScope's safety re-eval | `power on` + `brake release` after load (E2E driver handles this) |
| URScript `def foo(): … end; foo()` over Primary logs `Compile error: name 'foo' is not defined` | PolyScope auto-wraps inbound Primary URScript; the inner `def` is in a nested scope | Body executes anyway — the error is cosmetic. Use the wrap; raw multi-line statements break worse (each line becomes its own program). |
| `urctl run-script 'movel(...)'` returns `ok:true` but the robot doesn't move | Bare top-level motion is split into its own program and/or the Primary socket closes before the controller latches it (racy) | Use `urctl move-tcp` / `move-joints` (reliable, confirmed). `run-script` now def-wraps by default; `--raw` opts out. See "Executing motion over Primary". |

## Working in this repo

```bash
make sim-up          # start URSim
make sim-poweron     # power on + brake release (calls scripts/poweron.sh)
make test            # unit tests (pytest)
make test-integration # tests that need a running URSim
make lint            # ruff + shellcheck
make sim-down        # stop URSim
```

Host-side control (after `pip install -e .`), against the sim or a real robot:

```bash
urctl state                              # read state as JSON (localhost)
urctl --host 10.0.0.5 bring-up           # cold start a robot at an IP
urctl move-joints 0 -1.57 0 -1.57 0 0    # safety-validated movej
urctl move-tcp 0 0.05 0 0 0 0 --relative # safety-validated movel: +50mm base +Y
urctl --dry-run move-joints 99 0 0 0 0 0 # validate + audit, send nothing
urctl tools                              # dump the agent tool schemas (JSON)
urctl-mcp --host 10.0.0.5                # serve the same tools over MCP
```

When adding a new robot capability, add it in one place — a `Robot` method —
then surface it as a `Tool` in `urctl/tools.py` (schema + handler) and, if it
needs a human entry point, a subcommand in `urctl/cli.py`. The CLI, the agent
tool registry, and the MCP server all sit on the same `Robot`, so they stay in
lockstep. Keep mutating actions routed through the safety envelope + audit log.

For deeper development tasks:

- **Adding a new sample program**: drop a `.script` under
  `programs/<name>/`, run `urp_convert.py to-urp` to produce the `.urp`,
  copy URSim's `default.installation` as `<name>.installation`.
- **Changing the URP schema**: edit `script_to_urp` in
  `scripts/urp_convert.py`, then add a unit test in
  `tests/test_urp_convert.py` and run an integration test
  (`tests/test_integration_ursim.py::test_generated_urp_loads`) to confirm
  PolyScope still accepts it.
- **Debugging a load failure**: tail `polyscope.log` in the container —
  it spells out exactly what's wrong even when Dashboard returns "unknown
  failure":
  ```bash
  sudo docker exec ur-docker-ursim-1 tail -f /ursim/polyscope.log
  ```

## Things NOT to do

- Don't `--no-verify` past pre-commit hooks; they exist to catch URScript
  syntax + URP schema regressions early.
- Don't push directly to `main` — the environment blocks it. Use a feature
  branch + PR.
- Don't bake real-robot kinematics into URPs you commit. The defaults
  shipped by `urp_convert.py` (identity, checksum off) are intentional —
  they work everywhere; calibrated values are robot-specific.
- Don't `docker compose down` casually if you have unsaved work in
  PolyScope. URSim has no volume mount for `/ursim/programs/`; the
  container's program dir is wiped on recreate. Use `docker cp` to
  exfiltrate files first.
