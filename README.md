# perceptronics

Drive a Universal Robots arm from your laptop. This repo gives you a PolyScope
simulator in Docker and `urctl`, a zero-dependency Python CLI, library and MCP
server. `urctl` talks to the simulator and a real e-Series controller the same
way: `--host` is the only difference.

## Quick start

```bash
git clone https://github.com/Olympus-Controls/UR-utils && cd UR-utils
docker compose up -d          # the simulator; pendant at http://localhost:6080/vnc.html
uv sync                       # or: pip install .
uv run urctl bring-up         # power on + release brakes
uv run urctl state            # robot state as JSON
```

The e-Series image is amd64-only. On Apple Silicon, run the PolyScope X sim
instead: `HOST_ARCH=arm64 make simx-up`.

## Use it

```bash
urctl move-joints 0 -1.57 0 -1.57 0 0      # joint move (radians)
urctl move-tcp 0 0.05 0 0 0 0 --relative   # nudge the tool +50 mm in base Y
urctl --host 10.0.0.5 state                # a real robot
urctl-mcp --host 10.0.0.5                  # the same commands as MCP tools for an agent
```

Every move is checked by a safety envelope before it is sent and is written to
an audit log. Add `--dry-run` to check without moving. On a real robot, set the
pendant to **Remote** first: in Local mode, motion silently does nothing.

## Camera (optional)

A RealSense D435 on the tool flange turns a click on the camera image into a
robot move:

```bash
uv run perceptronics --fake gui     # the cockpit on a synthetic scene, no camera
uv run perceptronics --cell ur3 gui # a real cell
```

## More

- [CLAUDE.md](CLAUDE.md): protocols, file formats, and every hard-won gotcha
- [docs/realsense.md](docs/realsense.md): the camera, hand-eye calibration, picking
- [docs/harness.md](docs/harness.md): how the CLI, MCP server and cockpit fit together
- [urcap/](urcap/README.md): the camera node for the PolyScope pendant

## Development

```bash
uv run pytest -m "not integration"   # unit tests
make test-integration                # against a running simulator
make lint
```
