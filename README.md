# perceptronics

A vision backend and a browser cockpit for robot picking. A depth camera on the
arm sees the parts; perceptronics measures them in 3-D, turns the one you want
into a pose in the robot's base frame, and hands that pose to the robot.

Perceptronics is meant to work with **any robot that can report where its tool
is and move to a pose it is given**. Universal Robots is the first one wired
up (e-Series and PolyScope X), through the `urctl` library in this repo and a
pendant plug-in (URCap). Other arms plug in at one seam: see
[Adding a robot](#adding-a-robot).

## The idea

```
 depth camera ──▶ vision backend ─────────────────────▶ cockpit (browser)
 (RealSense,      frames + flange pose, segment,          live picture, 3-D scan,
  synthetic)      measure, hand-eye, reach                click → target, pick
                       ▲          │
           flange pose │          │ target pose
                       │          ▼
                ┌──────────────────────────┐
                │ robot adapter            │   today: Universal Robots
                │ read pose · move to pose │   (urctl, URCap pick node)
                └──────────────────────────┘
```

The robot is a pose source and a pose sink. Everything between the two lives in
this repo and knows nothing about the vendor:

- **Pose in every frame.** The robot streams its flange pose and each camera
  frame carries the pose it was taken at, so a pixel becomes a point in the
  base frame even while the wrist moves.
- **Segment and measure.** Click a part, or describe it by size, and the
  backend finds it in colour + depth: its top face, centre, heading, and
  length × width × height.
- **Hand-eye.** Where the camera sits on the flange: a seed from the bracket
  design, refined by a touch-and-click or a mark-less calibration.
- **Plan.** Approach pose, grasp pose, and a reach check from the arm's own
  kinematics. The controller's IK has the final say before anything moves.
- **Hand it over.** The target goes out through the robot adapter as a
  safety-checked, audited move, or back to a program on the robot that asked
  for it.

The core is pure Python standard library. It runs from a checkout on Windows,
macOS and Linux, amd64 and arm64, with nothing to install. The production target
is a small arm64 computer next to the robot (a Pi 5 or a Jetson).

## Quick start

No camera and no robot needed. Python 3.10 or newer:

```bash
git clone https://github.com/JimothyJohn/perceptronics && cd perceptronics
python3 -m perceptronics gui --fake     # cockpit on a synthetic scene: http://localhost:7621
python3 -m perceptronics doctor         # pre-flight: camera, SDK, robot, hand-eye
```

With a simulated robot in the loop:

```bash
make simx-up                            # PolyScope X sim (HOST_ARCH=arm64 on Apple Silicon)
python3 -m perceptronics --cell sim gui # synthetic camera, simulated arm
```

Motion on PolyScope X needs Remote mode and the Primary interface switched on in
its UI once ([CLAUDE.md](CLAUDE.md#polyscope-x-the-rest-robot-api-second-platform)).
With the real camera and cell: `python3 -m perceptronics --cell ur3 gui`. A
*cell* (`perceptronics/cells/*.env`) names the robot's address, platform, tool
length and camera bracket in one word.

## Where the code lives

| Path | What it is |
| ---- | ---------- |
| `perceptronics/realsense.py`, `rgbd.py` | RealSense D4xx over librealsense's C API (ctypes, no `pyrealsense2`): aligned colour + depth, intrinsics, deprojection |
| `perceptronics/segment.py`, `volume.py`, `partspec.py` | Click-to-segment (region growing, or SAM with the `sam` extra) and part-by-size detection from depth alone |
| `perceptronics/handeye.py`, `calibrate.py`, `orbitcal.py` | Camera-to-flange transform: bracket seed, touch-and-click and orbit calibration |
| `perceptronics/posestream.py` | The live flange pose, stamped onto every frame |
| `perceptronics/pickplan.py`, `armfk.py`, `armik.py` | Approach and grasp planning; forward and inverse kinematics for reach |
| `perceptronics/webapp.py`, `webui/` | The cockpit: HTTP API and the single-page UI (point-cloud scan, live feed, pick) |
| `perceptronics/picknode.py` | Pick server: a program on the robot asks over a socket and gets a target pose back |
| `perceptronics/robotlink.py` | The cockpit's one door to the robot; every action goes through the tool registry |
| `perceptronics/synthscene.py`, `--fake` | Synthetic RGB-D scenes, so everything runs and is tested without hardware |
| `urctl/` | The Universal Robots adapter: library, CLI, MCP server, safety envelope, audit log |
| `urcap/` | Pendant plug-ins for PolyScope 5 and PolyScope X: the live picture and the 3D Pick program node |
| `hardware/`, `deploy/` | Camera bracket (parametric CAD) and the pick-computer deployment |

## The cockpit's API

The UI is one page over a plain HTTP API, so an agent, a script or another UI
can drive the same things. The main routes:

| Route | Does |
| ----- | ---- |
| `GET /api/rgbd`, `/api/color.png`, `/api/depth.png` | Frames, each with the flange pose it was taken at |
| `POST /api/segment`, `/api/objects`; `GET /api/pick/scene` | Segment a click; list every part with its top face; find parts by size |
| `POST /api/robot/locate` | Camera point → base-frame point, approach pose, reachable or not (no motion) |
| `POST /api/robot/move`, `/api/robot/pick` | Move to a pose; run a full approach or pick |
| `GET /api/robot/pose`, `/api/info`, `/api/doctor` | Live pose, stream health, pre-flight |
| `POST /api/snapshot` | Save the current views as PNGs an agent can read |

`python3 -m perceptronics.mcp_server` (wired in `.mcp.json`) serves the same
robot and camera actions as MCP tools.

## Adding a robot

The vision side asks a robot two things: *where is your flange* and *move to
this pose*. There are two ways to answer, and a new arm can use either:

1. **The host drives the robot.** Implement the `Controller` protocol in
   [`urctl/controller.py`](urctl/controller.py) (`get_flange_pose`,
   `move_tcp`, `move_tcp_path`, `stop`, …). The tool registry, the cockpit's
   `RobotLink`, the pick cycle and calibration call those method names, not
   UR ones. A live pose stream (`posestream.py`, RTDE on UR) is what lets every
   frame carry its pose.
2. **The robot asks.** A program on the robot opens a socket to the pick
   server, sends its flange pose and reads back a target pose
   (`perceptronics/picknode.py`: one ASCII line in, one list of numbers out).
   Any controller that can open a TCP socket and parse numbers can use it. The
   UR 3D Pick node is the reference client.

The reach check takes kinematics from a DH table (`armfk.DH`). An arm that isn't
in the table is left to its controller's own IK.

## Development

```bash
make install-dev          # .venv with the pinned dev tools (pytest, ruff), hash-checked
make test                 # unit tests: no camera, no robot
make test-integration     # against a running simulator
make lint
```

Run the unit tests in a clean shell: an exported `UR_CELL` or `PERCEPTRONICS_*`
changes the defaults they expect.

PRs go to `dev`. CI runs lint and the unit tests on Python 3.10, 3.12 and 3.14
(Linux, plus Windows, macOS and Linux arm64), and runs URCap changes against
every PolyScope 5 minor release.

## More

- [docs/realsense.md](docs/realsense.md): the camera, depth quality, hand-eye, sending a point to the robot
- [docs/pick-kit.md](docs/pick-kit.md): the pick kit for a PolyScope 5 cell
- [docs/harness.md](docs/harness.md): the `urctl` read and control surfaces
- [urcap/README.md](urcap/README.md): the pendant plug-ins
- [CLAUDE.md](CLAUDE.md): working notes, protocols, and the gotchas that cost real time
