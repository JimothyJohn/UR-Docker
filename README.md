![ur](docs/ur-hero.png)

# UR-Docker

A sample container for Universal Robot testing and development.

## Prerequisites

* Linux - [Docker](https://docs.docker.com/engine/install/ubuntu/)
* Windows - [Docker Desktop](https://docs.docker.com/desktop/install/windows-install/)

## Usage (Linux)

Start the simulator:

```bash
git clone https://github.com/Olympus-Controls/UR-Docker
cd UR-Docker
sudo docker compose up -d --build
```

Then open your browser and connect to: [http://localhost:6080/vnc.html?host=localhost&port=6080](http://localhost:6080/vnc.html?host=localhost&port=6080)

NOTE: Do NOT connect at the IP address in the container output. You must use localhost.

![wrong address](docs/console-address.png)

To shut down the simulator:

```bash
sudo docker compose down
```

## Controlling a robot: the `urctl` toolkit

`urctl/` is a small, dependency-free Python package for driving a UR e-Series
controller over the network. It speaks the same backend protocols a real
controller exposes (Dashboard on 29999, Primary Client on 30001), so the
**same code drives this simulator and a physical robot** — the only thing that
changes is the connection target.

```bash
uv sync                     # create the .venv (installs `urctl` + `urctl-mcp`)
# prefix the commands below with `uv run`, or activate .venv first

urctl state                          # read robot state as JSON (localhost)
urctl bring-up                       # cold start -> RUNNING (power on + brakes)
urctl move-joints 0 -1.57 0 -1.57 0 0
urctl move-home --velocity 0.3
urctl load MotionDemo && urctl play
urctl popup "hello from urctl"
urctl --dry-run move-joints 99 0 0 0 0 0   # preview + validate, send nothing
```

As a library:

```python
from urctl import Robot, RobotConfig

robot = Robot(RobotConfig(host="10.0.0.5"))   # or RobotConfig.from_env()
robot.bring_up()
robot.move_joints([0, -1.5708, 0, -1.5708, 0, 0])
print(robot.get_state())
```

Every mutating call is checked against a configurable **safety envelope**
(joint range, speed/acceleration caps, required RUNNING state) *before* it
reaches the robot, and every call is written to a structured **audit log**.

### Connecting to a real robot (not just localhost)

Everything defaults to the local URSim container but is overridable, so the
toolkit works unchanged against a controller at an IP address. Point it with
`--host` or the `UR_HOST` environment variable:

```bash
urctl --host 10.0.0.5 state
UR_HOST=10.0.0.5 urctl bring-up
UR_HOST=10.0.0.5 ./scripts/poweron.sh        # the shell helper honors it too
make sim-poweron UR_HOST=10.0.0.5
```

| Variable | Default | Meaning |
| --- | --- | --- |
| `UR_HOST` | `localhost` | Controller host or IP |
| `UR_DASH_PORT` | `29999` | Dashboard port (e-Series) |
| `UR_PRIMARY_PORT` | `30001` | Primary Client port |
| `UR_PLATFORM` | `e-series` | Controller software: `e-series` (Dashboard) or `polyscopex` (REST Robot-API) |
| `UR_ROBOT_API_PORT` | `80` | PolyScope X Robot-API HTTP port (sim publishes it on `8000`) |
| `UR_TIMEOUT_S` | `10` | Socket timeout (seconds) |
| `UR_AUDIT_LOG` | _(unset)_ | If set, append a JSON-lines audit record per action |

> On real hardware these ports are **unauthenticated** — treat the robot's
> network like a backplane and keep it isolated. The safety envelope is a
> client-side guard, not a substitute for the controller's own configured
> safety limits.

### Driving the PolyScope X sim

The same `urctl` commands work against **PolyScope X** (PolyScope 10) — pick the
platform with `--platform polyscopex` (or `UR_PLATFORM=polyscopex`). PolyScope X
has no Dashboard server; orchestration goes through its REST **Robot-API**
(`http://<host>:<port>/universal-robots/robot-api`), while URScript still rides
the Primary client. See **CLAUDE.md → "PolyScope X: the REST Robot-API"** for the
full surface and the two gotchas (Remote-mode gate + enabling Primary in the UI).

```bash
make simx-up                                  # start the PolyScope X sim (web UI :8000)
make simx-state                               # read state via the Robot-API
make simx-bring-up                            # power on + brake release (needs Remote mode)
# equivalently, by hand:
UR_PLATFORM=polyscopex UR_ROBOT_API_PORT=8000 urctl state
```

## Agent / OpenClaw integration

`urctl` exposes its capabilities as a framework-neutral, JSON-schema'd tool
registry (`urctl.tools`) so an LLM agent can discover and call them. This is
the contract agentic robot executives like
[OpenClaw / ROSClaw](https://arxiv.org/abs/2603.26997) expect: capability
discovery, pre-execution validation, and structured audit logging.

```python
from urctl import Robot
from urctl.tools import get_tool_schemas, call_tool

robot = Robot()                       # or Robot(RobotConfig(host="10.0.0.5"))
tools = get_tool_schemas()            # hand these to the model's tool-calling API
result = call_tool(robot, "ur_move_joints",
                   {"joints": [0, -1.57, 0, -1.57, 0, 0]})
```

`get_tool_schemas()` returns `{name, description, input_schema}` dicts that
drop straight into Anthropic's `tools=` parameter (rename `input_schema` to
`parameters` for the OpenAI shape). `urctl tools` prints the same schemas for
inspection.

### MCP server

The same registry is exposed over the Model Context Protocol, so any MCP
client (Claude Desktop, Claude Code, or any MCP-capable agent) can drive the
robot:

```bash
uv sync --extra mcp                   # installs the optional `mcp` SDK
uv run urctl-mcp --host 10.0.0.5      # serve over stdio
```

Example Claude Desktop / Claude Code MCP config:

```json
{
  "mcpServers": {
    "ur": {
      "command": "urctl-mcp",
      "args": ["--host", "10.0.0.5"],
      "env": {"UR_AUDIT_LOG": "/tmp/ur-audit.jsonl"}
    }
  }
}
```

Add `--dry-run` (CLI/MCP) to validate and audit tool calls without sending
anything to the controller — useful for previewing what an agent would do.

## Perception: the `perception` package

A second, **parallel** toolkit (it does not import `urctl`): turn an RGB frame
into detected blobs annotated with monocular-estimated depth. Same shape as
`urctl` — env config, pluggable Protocol backends, a pipeline facade, an agent
tool registry, and a CLI. The core is dependency-free (pure-Python stub depth +
blob backends); real backends (Depth Anything V2, OpenCV, webcam capture) are
optional extras selected purely by config.

```bash
perceive synthetic                                 # no camera, no model weights
perceive image inputs/image.png                    # bundled apples-on-steel fixture
perceive capture                                   # one webcam frame → blobs
perceive --depth-backend depth_anything capture    # real monocular depth model
```

On the bundled `inputs/image.png` (three apples on a steel table) the
dependency-free stubs segment the produce from the metallic background by
*colorfulness*, then split touching same-colored objects with a distance-
transform watershed — returning all three apples as separate blobs with
per-blob depth, ignoring the steel and the overlay text.

A `Blob` carries `centroid_px`, `bbox`, `area_px`, `depth_m`, `mean_rgb` (2D +
depth). The 3D deprojection + camera→base transform that would feed
`urctl.Robot.move_tcp` is the planned integration seam — see
[`docs/perception.md`](docs/perception.md).

## Helper scripts

`scripts/poweron.sh` — brings the controller from a cold start through the
full PolyScope initialization sequence (`POWER_OFF` → `IDLE` → `RUNNING`) by
talking to the Dashboard server on `29999`, then sends a URScript `popup()`
directly to the Primary Client on `30001` to verify the backend is responsive.
Run after `docker compose up`:

```bash
./scripts/poweron.sh                  # localhost
UR_HOST=10.0.0.5 ./scripts/poweron.sh # real robot
```

`scripts/urp_convert.py` — converts between `.script` (raw URScript text) and
`.urp` (PolyScope program format, gzipped XML). Useful because PolyScope's
"Load Program" file browser filters to `*.urp` by default, so a plain
`.script` file is invisible unless you flip the filter.

```bash
# wrap a .script as a .urp so it shows up in PolyScope's default file list:
python3 scripts/urp_convert.py to-urp my_program.script my_program.urp

# extract URScript from a .urp for inspection or version control:
python3 scripts/urp_convert.py to-script my_program.urp my_program.script
```

The generated `.urp` references `default.installation` as its installation
file; override with `--installation <name>` to pair with a different one.

## Development

```bash
make install-dev        # pytest, ruff, etc.
make sim-up             # start URSim
make sim-poweron        # power on and release brakes
make test               # unit tests (no simulator needed)
make test-integration   # tests that talk to URSim
make lint               # ruff + shellcheck
```

`CLAUDE.md` documents the network surface, file formats, URScript gotchas,
and common diagnostic recipes — read it before diving deeper.

## Sample programs

`programs/InspectionBot/` — a guided multi-station quality-inspection
routine in the teach-by-prompt style of
[HelpfulBot](https://github.com/nickarmenta/HelpfulBot). The operator walks
through PolyScope popups to wire I/O and freedrive-teach a variable-length
tour of inspection waypoints; the cycle visits each station, asks pass/fail,
and pulses a verdict output. See the program's own README for details.

`programs/MotionDemo/` — a fully non-interactive movement program used by
`scripts/e2e_drive.py` to verify the full controller lifecycle. Visits four
joint-space poses, emits textmsg checkpoints, returns home.

`programs/PickPlace/` — a basic pick-and-place cycle: grasp a part at one
station (gripper on digital output 0), carry it over, release it at another,
return home. Stations are derived from a reachable reference pose so it stays
inside the UR10 envelope on any setup; swap in your taught poses on hardware.
Run it by streaming to the Primary interface:

```bash
urctl run-script --capture --marker pickplace/ --collect-for 35 \
  < programs/PickPlace/PickPlace.script
```

`programs/PickAndStack/` — picks up three items (the apples in the provided
`inputs/image.png`) and stacks them at one nearby location, **motions only**.
The gripper open/close steps are placeholders, commented out, so you wire your
gripper (or a URCap call) in later. Pick stations and the stack are derived at
runtime from one reachable reference pose (base-frame offsets) so it runs inside
the UR10 envelope on URSim; replace them with poses taught in your cell. Run it
by streaming to the Primary interface:

```bash
urctl run-script --capture --marker pickstack/ --collect-for 80 \
  < programs/PickAndStack/PickAndStack.script
```

> Editability note: this ships as a `.script` wrapped into a `.urp` (via
> `urp_convert.py`), so PolyScope loads it as a **single Script program node**
> whose code you open and edit on the Program tab. It is *not* decomposed into
> individual Move/Waypoint nodes — PolyScope stores those via a proprietary
> serializer (hand-authored inline `Script`/`Waypoint` nodes fail to
> deserialize on URSim 5.12.5), so a true per-node-editable tree has to be
> authored/saved from the teach pendant.

## End-to-end driver

`scripts/e2e_drive.py` proves the host can drive the robot from cold start
to completed program. Walks six phases against the documented backends:
smoke test via Primary 30001 → state read → motion + verification →
return home → Dashboard load → Dashboard play (with Primary fallback if
PolyScope is in Local control mode).

```bash
./scripts/poweron.sh      # bring controller to RUNNING
python3 scripts/e2e_drive.py
```

Output is one PASS/FAIL line per phase; `--json` emits machine-readable
records (used by `tests/test_integration_ursim.py::TestE2EDrive`).
