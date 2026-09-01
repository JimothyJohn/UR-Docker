# The urctl harness — read everything, control safely, program beside the operator

This document is the architecture reference for the `urctl` harness: every
surface it reads, every action it can take, and how to build on top of it —
specifically the planned local GUI application and the MCP integration. It
assumes the protocol background in `CLAUDE.md` (ports table, URScript dialect,
real-robot divergences).

Everything below is **verified working** against both targets:

| Target | Platform | Notes |
| ------ | -------- | ----- |
| URSim 5.26.0 LTS container | e-Series sim | `localhost`, docker exec for files |
| UR10e `20225201277` @ 192.168.1.50 | real e-Series, URControl 82.0.34 G5 | SSH (key enrolled) for files |

## 1. The three read surfaces

The harness reads a robot at three depths. A GUI should use all three:

```
                 ┌───────────────────────────────────────────────┐
   once per      │  urctl snapshot        (network + filesystem) │  the CELL MODEL
   connect ────► │  Robot.get_state() + rtde_state(deep=True)    │
                 │  + SystemInspector.snapshot()                 │
                 ├───────────────────────────────────────────────┤
   1–10 Hz  ───► │  Robot.rtde_state(deep=True)   (port 30004)   │  DIAGNOSTICS
                 ├───────────────────────────────────────────────┤
   up to    ───► │  RtdeClient.receive() loop     (port 30004)   │  LIVE STREAM
   125 Hz        │  (joints, TCP, IO — for 3D view / jog UI)     │
                 └───────────────────────────────────────────────┘
```

### 1a. `urctl state` — orchestration state (Dashboard + RTDE)

Robot/safety/program/control mode plus live joints & TCP. The `control_mode`
field is the **Remote-mode gate signal**: on a real e-Series, Primary URScript
motion silently no-ops in `LOCAL` (see CLAUDE.md). A GUI must surface this.

### 1b. `urctl rtde-state --deep` — full telemetry (`Robot.rtde_state(deep=True)`)

One RTDE sample of the complete diagnostic recipe (`urctl.rtde.DEEP_OUTPUTS`),
decoded to names by `urctl.codes`:

- per-joint: position, target, velocity, **current, temperature, drive mode**
  (`joint_mode_names: ["RUNNING", ...]`)
- TCP: pose, speed, 6-axis **force/torque**, elbow position
- safety: `safety_mode_name`, `safety_flags` (each bit of
  `safety_status_bits` as a named boolean — `protective_stopped`, `fault`, …)
- power: robot voltage/current, main voltage
- tool connector: output voltage/current, temperature, analog in — a powered
  gripper shows up here (the real UR10e reads 24 V for its Hand-E)
- IO: digital in/out words, standard analog in/out
- speed: `speed_scaling` (live) vs `target_speed_fraction` (slider)

Fields a firmware doesn't offer are **dropped, not fatal** (tolerant recipe
negotiation; they're listed in `unavailable_fields`). The real UR10e lacks
`actual_joint_voltages`; everything else lands.

### 1c. `urctl snapshot` — the cell model (`ur_system_snapshot` tool)

Fuses the network reads with **filesystem introspection** over SSH (real
robot) or docker exec (URSim) — auto-inferred from the host, overridable with
`--ssh [user@]host` / `--container name`:

```bash
urctl --host 192.168.1.50 snapshot          # real robot, SSH inferred
urctl snapshot                              # URSim, docker exec inferred
urctl --host 192.168.1.50 snapshot --no-live  # controller on, arm powered off
```

Sections (each fails independently — a missing file annotates, never aborts):

| Section | What it answers |
| ------- | --------------- |
| `identity` | hostname, serial, model, URControl version, uptime |
| `joints` | per-joint serial + firmware, **replacement detection** (minority firmware), **kinematic-calibration mismatch** (the boot-log checksum error) + which joint ids, calibration date |
| `storage` | eMMC usage (df-accurate %), reclaimable log/flight-report sizes |
| `programs` | every `.urp` / `.script` / `.installation` with size + mtime |
| `installation` | parsed active installation — see below |
| `urcaps` | installed URCap jars |
| `recent` | last error/fault lines from the URControl log |

The `installation` section (`urctl.installation.parse_installation`) is the
program-authoring decision input:

- **`flange_tcp`** — `True` when the active TCP offset is all-zero. When
  `False` (a gripper — the real robot's `HandE_single`), **author motion as
  `script_file`/`script_line` nodes, not native MoveJ/Waypoint nodes** (the
  IK-through-active-TCP trap).
- `tcps` — every configured TCP with offset; `payload` — mass + CoG;
- `io_names` — the pins the integrator named (`Gripper` on DO0, `conveyor`
  on DO2, `Drop` on CI7 on the real cell) — use these as labels in a GUI and
  as vocabulary when building routines ("close the gripper" → DO0);
- `safety_limits` — Normal/Reduced headline limits (max TCP speed, force …);
- `safe_home` — the installation's safe-home joint pose.

### The high-rate stream (for a GUI's live view)

`Robot.rtde_state()` re-syncs per call (fresh sample, ~pair of round-trips) —
right for polling, wrong for streaming. A GUI's 3D/jog view should hold one
`RtdeClient` open and consume the broadcast:

```python
from urctl import RtdeClient, RobotConfig
from urctl.rtde import DEEP_OUTPUTS

client = RtdeClient(RobotConfig(host="192.168.1.50"),
                    outputs=DEEP_OUTPUTS, frequency=25.0, strict=False)
client.connect()
client._start()                      # begin the stream
while gui_running:
    sample = client.receive()        # blocks until the next frame
    update_view(sample)              # dict of decoded fields
client.close()
```

Pick `frequency` to match the view (10–25 Hz is plenty for visualization; the
controller supports up to 125 Hz on e-Series). One RTDE output recipe per
client; run reads and control on separate clients if they need different rates.

## 2. The control surfaces

All mutating actions flow through `SafetyEnvelope` (validation before send)
and `AuditLog` (structured JSON record after). Highlights, with the gotchas a
GUI must respect:

| Action | API | Gotcha it encodes |
| ------ | --- | ----------------- |
| cold start | `bring_up()` | power on → brake release → clears latched protective stop |
| joint move | `move_joints(q)` | def-wrapped, confirm-marker, landing check |
| Cartesian move | `move_tcp(pose, relative=)` | base-frame deltas; singularity → `protective_stop:true` in result |
| path | `move_trajectory(waypoints)` | one connection for N waypoints — **the only safe way to loop moves** (rapid reconnects wedge URControl) |
| jog | `move_tcp(..., relative=True)` | envelope caps step size |
| speed slider | `set_speed_override(f)` | RTDE input recipe, masked write |
| digital out | `set_digital_output(pin, v)` | RTDE masked write — doesn't disturb slider |
| freedrive | `freedrive(True/False)` | hand-guiding for teach |
| URScript | `run_script(s)` | def-wraps by default (`--raw` opts out) |
| program | `load_program(n)` / `play()` / `pause()` / `stop()` | `play` needs Remote mode |
| operator dialog | `popup(t)`, `confirm_on_pendant(t)` | Yes/No on the pendant, timeout cleans up its own dialog |

**Real-robot preconditions a GUI should check before motion buttons enable:**
`control_mode == "REMOTE"` (else motion silently fails) and
`safety_flags.protective_stopped == False` (else offer "Unlock" →
`bring_up()`).

## 3. Working as a programmer beside the operator

Three authoring layers, lowest to highest:

1. **`urctl.urp_builder.UrpProgram`** — programmatic native node trees
   (MoveJ/Waypoint, If, Loop, Set, Popup, script nodes). Use script-node
   motion when `snapshot` says `flange_tcp: false`.
2. **`urctl guided <name> --save p.urp [--freedrive] [--live ...]`** — the
   operator-in-the-loop builder: every step is proposed on the pendant
   (Yes/No), executed live, and recorded at the *achieved* pose. `--live-scp
   root@<ip>` grows the tree on the real pendant step by step.
3. **`urctl inspect <name>`** — the multi-point inspection app (freedrive to
   a spot, tap Yes to capture).

The cell model feeds this: named IO becomes routine vocabulary, the active
TCP decides node style, `safe_home` seeds the retreat pose, and existing
programs on the controller (`urctl programs`) are the operator's context.

## 4. The agent/MCP surface

`urctl-mcp` is **pure stdlib** — MCP's stdio transport is newline-delimited
JSON-RPC 2.0, spoken directly (`urctl/mcp_server.py`); no SDK install needed.
`urctl tools` prints all 20 tools as JSON-schema'd capabilities;
`urctl call <tool> --json '{...}'` dispatches one; `urctl-mcp` serves the
same registry over MCP (stdio). The three harness additions:

- `ur_rtde_state` (`deep` param) — full diagnostics
- `ur_system_snapshot` — the cell model (start here when meeting a robot)
- `ur_list_programs` — what the operator can load

Every tool call is schema-validated, safety-checked, and audited — the same
guarantees regardless of who's calling (CLI, GUI, Claude over MCP).

## 5. GUI application — `urctl gui` (the cockpit)

**Implemented**: `urctl/webapp.py` (stdlib HTTP server, loopback-only) +
`urctl/webui/index.html` (single-file cockpit UI). It sits **directly on the
`urctl` Python API** (same process, no extra daemon):

```bash
uv run urctl gui                          # URSim
uv run urctl --host 192.168.1.50 gui      # the real robot
uv run urctl-gui --host 192.168.1.50      # dedicated entry point
uv run urctl --dry-run gui                # every button validates + audits, sends nothing
```

Panels: live 3D arm view (UR10e forward kinematics on a canvas — drag to
orbit), Cartesian + per-joint jog with step presets, speed slider, named I/O
lamps (DO click-to-toggle), program inventory with load/play/pause/stop,
URScript + pendant-popup console, action history, and the cell model
(calibration status, active-TCP flange warning, disk health). Header chips +
banners surface the two real-robot gates: LOCAL control mode and a latched
protective stop (with an Unlock button → `ur_bring_up`).

Wire-level: `GET /api/stream` is Server-Sent Events pushing deep RTDE samples
at 12.5 Hz (each stream owns its own `RtdeClient`, so viewers never contend
with actions); `POST /api/action {tool, params}` dispatches through
`urctl.tools.call_tool` — the same schema validation, safety envelope, and
audit logging as the CLI and MCP. `/api/snapshot` caches the cell model per
session (`?refresh=1` re-reads).

```
┌────────────────────── laptop app ──────────────────────┐
│  UI (panels: state, jog, IO, programs, routine builder)│
│      │                     │                            │
│  Robot facade          RtdeClient stream    SystemInspector
│  (actions, audited)    (25 Hz live view)    (cell model, SSH)
└──────┼─────────────────────┼─────────────────────┼─────┘
       │ 29999 + 30001       │ 30004               │ 22 (ssh)
       └──────────────── UR controller ────────────┘
```

- **Writes**: every button is a tool call ⇒ automatically schema-validated,
  safety-checked and audit-logged; actions are serialized by a lock.
- The MCP server can run alongside the GUI against the same robot — RTDE
  inputs are claimed lazily and reads don't conflict; avoid two writers of
  the same RTDE input (speed slider) at once.
- **Security model**: binds 127.0.0.1 only, no auth — the same trust level as
  running `urctl` yourself. Don't bind it to a routable interface.

## 6. File map of the harness additions

```
urctl/rtde.py          DEEP_OUTPUTS recipe + tolerant (strict=False) negotiation
urctl/codes.py         mode/bit-word decoding tables (names, not magic numbers)
urctl/installation.py  .installation parser (TCPs, payload, IO names, limits)
urctl/sysinfo.py       SshRunner / DockerRunner / SystemInspector / runner_for
urctl/robot.py         rtde_state(deep=True)
urctl/tools.py         ur_system_snapshot, ur_list_programs, ur_rtde_state(deep)
urctl/cli.py           snapshot, programs, rtde-state --deep, gui
urctl/webapp.py        the cockpit's HTTP/SSE server (urctl gui / urctl-gui)
urctl/webui/index.html single-file cockpit UI (canvas FK arm view, jog, IO, programs)
tests/test_harness.py  25 unit tests (parsers, decoding, tolerant recipe, deep shaping)
tests/test_webapp.py   10 unit tests (routing, action dispatch, caching, error shaping)
```
