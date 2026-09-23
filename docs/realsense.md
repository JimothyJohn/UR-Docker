# RealSense on the robot — capture, segment, measure

The `perception` package can read an Intel RealSense D4xx directly (color +
aligned depth), segment an object with a click, and measure it in metres — the
RealSenseTrainer workflow without the training: capture aligned RGB-D plus a
label and metadata into a dataset folder, but get the object's centroid, 3D
position, size and orientation *now*.

```
perception rs-info                       # SDK + attached cameras
perception rs-capture --out captures     # one RGB-D capture + nearest-object mask
perception gui                           # the RGB-D cockpit (browser, loopback)
perception gui --fake                    # same, synthetic scene, no camera needed
perception --segment-backend sam gui     # Segment Anything instead of region growing
```

## Where the compute lives

| Option | Status | Notes |
| --- | --- | --- |
| **Jetson Orin next to the robot** (target) | designed for; container in `Dockerfile.perception` | Camera on the tool flange (`hardware/d435-tool-bracket/`), USB back to the Jetson, this package runs as a service there (`compose --profile perception`). The Jetson also runs `urctl` against the controller over the network, so one box owns perception *and* motion. |
| **Laptop (this Mac)** | works today | `brew install librealsense`; the SDK needs **root to claim the USB interface on macOS** (libusb has to detach Apple's UVC driver): `sudo uv run perception gui`. |
| **On the UR controller itself** | not attempted | The CB is a Debian box with USB, so librealsense *could* be built there, but it shares the CPU with URControl's real-time loop, UR re-images it on update, and there is no supported way to ship a service with the robot. The URSim container cannot see USB at all (Docker Desktop on macOS has no USB passthrough). Deferred; the Jetson makes it unnecessary. |

## How it works

`perception/realsense.py` binds librealsense's **C API with ctypes** — no
`pyrealsense2`, which has no macOS wheel and is a source build on Jetson
anyway. The package stays zero-dependency; the only requirement is
`librealsense2.{dylib,so,dll}` (found via `$REALSENSE_LIB`, `find_library`, or
the usual prefixes). Enum ordinals are self-checked against the library's
`*_to_string` functions at load, and our Python deprojection is unit-tested
against `rs2_deproject_pixel_to_point` whenever the library is present.

`RealSenseCamera` streams Z16 depth + RGB8 color at one resolution and runs the
SDK's `align` block so depth is re-projected into the colour image: pixel
`(u, v)` names the same physical point in both, and the frame carries the
colour intrinsics. `RgbdFrame.point_at(u, v)` gives camera-frame metres.

## Depth quality (why the raw stream looks noisy, and what's on by default)

A D435 pixel straight off the sensor jitters by millimetres at half a metre
and by a centimetre or two at two metres — stereo error grows with the square
of distance — so a single-pixel readout flickers. Three things now run by
default to steady it, all through the same ctypes binding (no new deps):

| Knob | Default | Flag / env | What it does |
| --- | --- | --- | --- |
| **Depth resolution** | 848×480 | `--depth-res WxH`, `PERCEPTION_RS_DEPTH_WIDTH/_HEIGHT` | The D435's native stereo mode; the ASIC derives 640×480 by downscaling, so 848×480 is the accuracy-optimal choice (Intel's D400 tuning guide). **Colour follows the depth size** unless you set `--width/--height` (or `PERCEPTION_WIDTH/HEIGHT`): on this D435 (fw 5.12.7.100, macOS) 848×480 depth next to 640×480 colour returned **all-black colour frames** with every sensor option at factory and a lit room, while 640/640 and 848/848 were both fine. Keep the two sensors at the same size. |
| **Post-processing chain** | on | `--no-depth-filters`, `PERCEPTION_RS_FILTERS=0` | `DepthFilters`: depth→disparity → **spatial** (edge-preserving smoothing) → **temporal** (per-pixel EMA over frames, persistence "valid in 2 of the last 4") → disparity→depth, applied to the depth frame *before* alignment in Intel's recommended order. Hole filling and a min/max threshold exist on the dataclass but are off — hole filling invents depth, which is wrong for measuring. |
| **Sensor tuning** | `high_accuracy`, laser max | `--rs-preset NAME\|none`, `--laser-power MW\|max\|none`, `PERCEPTION_RS_PRESET`, `PERCEPTION_RS_LASER_POWER` | `DepthTuning`, written once **after the first frameset arrives** (writing them between pipeline start and the first frame stalled a freshly claimed D435 on macOS — no frame ever came; verified 2026-09-04): the *High Accuracy* visual preset raises the stereo confidence threshold (fewer pixels, far fewer wrong ones) and full projector power puts more texture on flat surfaces. Best effort — an unsupported or refused option is reported in `/api/info` → `camera.depth.tuning_applied`, never fatal. `none` leaves a sensor you tuned in realsense-viewer alone. |

The filter blocks are `rs2_processing_block`s fed whole framesets, exactly like
the `align` block, so each is one `rs2_process_frame` + queue wait per frame.
The temporal filter is what steadies a static scene; it lags on moving objects
by a few frames, which is the trade. `tests/test_realsense_hw.py::
test_filtered_depth_is_steadier_than_raw` measures per-pixel temporal noise in
a centre patch, raw vs default, on the attached camera
(`sudo uv run pytest -m realsense -q -s` prints both numbers).

What no filter fixes: anything under ~28 cm returns nothing; dark matte,
shiny, or transparent surfaces defeat stereo; direct sunlight washes out the
IR projector. 30–60 cm from the workpiece is the good zone for the bracket.

The cockpit (`perception/webapp.py` + `perception/webui/index.html`) is a
stdlib HTTP server, same shape as `urctl gui`. One thread pumps the camera;
the page long-polls `/api/rgbd` for a binary container (JSON header + colour
PNG + zlib'd `uint16` depth), inflates the depth with `DecompressionStream`
and colourises it in the browser. Hover measures; click posts `/api/segment`.

## Segmenting and the features you get

| Backend | Select | What it does |
| --- | --- | --- |
| `stub` (default) | — | Region growing from the click by colour similarity **and depth continuity** (a neighbour joins only if its depth is within 2 cm). Pure Python, ~10 ms. Also `nearest_object` — RealSenseTrainer's "closest thing" rule. |
| `sam` | `--segment-backend sam` / `PERCEPTION_SEGMENT_BACKEND=sam`, `uv sync --extra sam` | Segment Anything (`facebook/sam-vit-base` via transformers) with the click as the point prompt; CUDA on the Jetson, `mps` on a Mac. First run downloads ~375 MB. Not exercised in CI. |

`extract_features(mask, frame)` returns, per object: area, bbox, centroid,
mean colour, depth median/min/max, **3D centroid in the camera frame**, metric
extent (bbox at the object's depth), thickness, principal-axis orientation,
major/minor axis lengths, elongation, fill ratio, and a labelled *assumption*
of a grasp (top-down, close across the minor axis). Turning that into a base-
frame pick needs the hand-eye transform — the bracket spec gives a nominal
`T_flange_camera` seed, and `urctl` supplies the flange pose.

## Capture layout (RealSenseTrainer-compatible)

```
captures/<name>/
  color_00001.png   8-bit RGB
  depth_00001.png   16-bit grayscale, raw units × depth_scale_m = metres
  mask_00001.png    0/255 label (when a segment was taken)
  meta_00001.json   intrinsics, depth scale, timestamps, device, features
```

Numbering resumes from the highest existing index. `CaptureStore.load()` reads
a capture back into an `RgbdFrame`, so measurements can be re-run offline.

## Running it on the Jetson (container)

```bash
docker compose --profile perception build      # builds librealsense (RSUSB backend) from source
docker compose --profile perception up -d      # privileged for USB; cockpit on :7621, bound 0.0.0.0
```

Verified 2026-09-02: the image builds for `linux/arm64` (librealsense v2.58.4,
RSUSB backend, ~10 min on Apple Silicon), the SDK loads inside it, and
`perception rs-info` / `rs-capture --fake` run. USB itself was not exercised
(Docker Desktop on macOS cannot pass the camera through).

The container has no auth — it is a cell-network cockpit, like `urctl gui`.
Keep it off routable networks or put it behind the Jetson's firewall.

## Click or drag to segment

In the cockpit, a **click** sends a point prompt and a **press-drag-release**
sends a box prompt (`/api/segment` takes `{"x","y"}`, `{"box":[x0,y0,x1,y1]}`,
or both — a point inside a box disambiguates). Boxes are `x1`/`y1`-exclusive
pixel edges, corner order doesn't matter, and anything off-frame or empty is a
400 rather than a clamp. The last box is drawn dashed until you clear.

Backends (`--segment-backend`, `$PERCEPTION_SEGMENT_BACKEND`):

| backend | what it does | box prompt |
| --- | --- | --- |
| `stub` (default) | colour + depth region growing, pure Python | grows from the box centre (or the click) and clips to the box |
| `sam` | Segment Anything via `transformers` (`uv sync --extra sam`) | native SAM box prompt, single-mask decode |

SAM checkpoint (`--sam-model`, `$PERCEPTION_SAM_MODEL`, any `SamModel`-loadable
id): `facebook/sam-vit-base` by default; `Zigeng/SlimSAM-uniform-50` is the
light one (27 M params). Measured on this M-series Mac (`mps`, 640×480):

| | embed a new frame | decode a prompt on the same frame |
| --- | --- | --- |
| `sam-vit-base` | ~0.45 s | ~10–50 ms |
| `SlimSAM-uniform-50` | ~0.33 s | ~10–50 ms |

The image embedding is cached per frame, so re-clicking or re-boxing the same
frame costs only the decode — pause the stream (Space) to iterate on one frame.
On the Jetson the same backend runs on CUDA; NanoSAM (TensorRT) is a possible
later backend behind the same `Segmenter` seam but is not wired.

## Sending a point to the robot (cockpit → base frame → `movel`)

The Object panel has a **Robot** section. With a segment that has depth:

1. **Locate in base** — `POST /api/robot/locate`. The cockpit reads the live
   flange pose from the controller (`ur_flange_pose`: one Primary `textmsg`
   round-trip returning `get_actual_tcp_pose()`, `get_tcp_offset()` and the
   controller's own `pose_trans(tcp, pose_inv(offset))`; works in Local mode)
   and maps the segment's camera-frame point through the hand-eye transform:
   `p_base = T_base_flange · T_flange_depth · T_depth_color · p_color`. It
   shows every intermediate frame plus an **approach pose**: the TCP placed
   *standoff* metres short of the point along the camera's viewing ray, with
   the tool's current orientation. Nothing moves. The panel's **from** picks
   what the standoff is measured to: the active TCP, or the tool **flange**
   (`reference: "flange"` — the flange target is converted to the TCP pose
   `movel` takes through the live active-TCP offset). Use flange when the
   controller's active TCP is not the physical tool (the UR3e's 223 mm
   training TCP; the cell file says `PERCEPTION_APPROACH_REFERENCE=flange`,
   `PERCEPTION_STANDOFF_M=0.075` — "flange 75 mm above the part"). Locate also
   reports `reachable` against the arm's reach; the Move button stays disabled
   when it is false.
2. **Approach** (the test loop) — `POST /api/robot/approach_cycle`: over the
   segment at `clearance_m` (0.10), down to the standoff, hold `hold_s` (1 s),
   back up, back to where the picture was taken — **one** URScript program
   (`ur_move_tcp_path`) on one connection, every leg safety-validated first,
   refused when out of reach. With the `flange` reference every leg runs with
   the TCP forced to the flange (`set_tcp(p[0,…])`). Click the object (depth +
   colour region growing, no model), press Approach, watch the cycle, move the
   object, repeat. Also `cam_approach_cycle` over MCP.
3. **Move TCP to approach** — `POST /api/robot/move` with the pose you just
   saw. One absolute `movel` through `ur_move_tcp` — the same schema-validated,
   safety-enveloped, audited path as the `urctl` CLI and MCP server (refused
   when not RUNNING, over the speed caps, or outside reach). Slow by default
   (0.1 m/s). On a real e-Series this needs **Remote** control mode; locating
   doesn't.

**Hand-eye transform.** `perception/handeye.py` seeds `T_flange_depth` from
the bracket geometry (`hardware/d435-tool-bracket/README.md` §3,
`ARM_ANGLE_DEG = 90`, camera on the tool-I/O side: camera x = flange −X,
camera y = flange −Y, camera z = flange +Z, depth origin at (17.5, 66.5, 1.7) mm)
and takes
`T_depth_color` from the SDK's extrinsics at open (`rs2_get_extrinsics`,
~15 mm along x on a D435; identity on the synthetic camera). That is an
**uncalibrated seed** — a printed part won't hold ±1°, and 1° at 0.5 m is
~9 mm. Replace it with a hand-eye calibration via
`PERCEPTION_T_FLANGE_CAMERA="[x, y, z, rx, ry, rz]"` (the depth frame in the
flange frame, metres + UR rotation vector); the panel's *hand-eye* row says
which one is active. Pose arithmetic is `urctl/pose.py` (Rodrigues,
`pose_trans`/`pose_inv` with URScript semantics, pure stdlib) and is
cross-checked against the controller's own `pose_trans` on every locate
(the *host/controller Δ* row).

Flags / env: `--robot-host` (`$UR_HOST`, default localhost = URSim),
`--robot-dry-run` (validate + audit, send nothing; a stand-in flange pose
lets the whole flow run on `--fake`), `--no-robot` (no panel). The
standoff and its reference are per-click in the panel, defaulting to the
cell's `PERCEPTION_STANDOFF_M` / `PERCEPTION_APPROACH_REFERENCE` (0.10 m from
the TCP unless the cell file says otherwise).

**Verified 2026-09-04 against the PolyScope X simulator (10.13.0, native
arm64, Remote mode):** `ur_flange_pose` matched the controller's own
`pose_trans(tcp, pose_inv(offset))` exactly; the cockpit's Locate mapped the
synthetic camera point into the base frame and Move landed on the approach
pose to 0.1 mm (RTDE readback). Not yet run with the camera on the bracket or
on the physical UR10, and the sim's TCP offset is zero, so the offset half of
the arithmetic is covered by unit tests only.

What it does **not** do: pick the object (no gripper orientation from the
segment — the approach keeps the current tool rotation; the segment's
`grasp` yaw hint is there for the next step), avoid obstacles (a straight
`movel`; move above the scene first), or verify reach before you press Move
(the envelope's reach check is a sphere; PolyScope's IK is the final word,
see *Cartesian moves and singularities* in CLAUDE.md).

## The pilot's seat: cells, doctor, Pilot panel, and the keys for an agent

Everything below is the day-to-day loop for testing on a laptop at the cell
(the presentation-grade walkthrough is `docs/realsense-cell.html`).

**Cell profiles** (`perception/cells/{sim,ur3,ur20}.env`, `--cell NAME` on any
`perception` entry point or `UR_CELL=NAME`): one word selects the robot's
host/platform/ports and the bracket print (`PERCEPTION_BRACKET=eseries|ur20`,
which picks the hand-eye seed — `perception.handeye.BRACKET_SEEDS`). A variable
already set in the shell wins over the file. The real cells ship with `UR_HOST`
empty: fill in the controller IP once. `perception cells` prints them;
`eval "$(uv run perception cells --export ur20)"` puts the same variables in
your shell so plain `urctl` commands target the cell too.

**Doctor** (`perception --cell ur20 doctor [--stream] [--no-robot] [--json]`,
`make doctor CELL=ur20`, the **Doctor** button in the cockpit, or the
`cell_doctor` MCP tool): the pre-flight. SDK found + version, camera enumerated
(USB link), optional stream check (fps, black-colour and sparse-depth
symptoms), robot reachability per port with a **platform-mismatch** diagnosis
(cell says PolyScope X but only the Dashboard port answers, or vice versa),
robot/safety/control mode, telemetry, active TCP offset, hand-eye calibration
status and bracket-vs-robot-model mismatch. Every failure carries the fix in
words; the verdict is `READY`, `STATE ONLY (motion gated)` (Local mode /
protective stop: reads work, moves won't) or `NOT READY`.

**Pilot panel** (top of the cockpit's side column): live status chips
(robot/safety/control/program), **Bring up**, **STOP**, **Freedrive**,
**Doctor**, a base-frame **jog pad** (1–50 mm per press, each press one
safety-checked `movel`, capped at 5 cm / 0.35 rad per axis server-side), the
live TCP, and an **Events** log of everything the cockpit did (robot actions
with their outcome and safety verdicts, segments, captures, camera errors).
Endpoints: `POST /api/robot/{jog,bring_up,stop,freedrive}`, `GET /api/doctor`,
`GET /api/events?after=N`, `POST /api/snapshot`.

**The keys for an agent** (`perception-mcp --cell ur20`, wired in `.mcp.json`
for Claude Code): one MCP server serving the whole `urctl` robot registry plus
the camera family — `cam_info`, `cam_snapshot` (writes colour/depth/mask PNGs
an agent can *look at*), `cam_segment`, `cam_nearest`, `cam_clear`,
`cam_locate`, `cam_move_to_approach`, `cam_capture`, `cam_events`,
`cell_doctor`, `cell_jog`. The camera tools go **through the running cockpit**
(`PERCEPTION_COCKPIT_URL`, default `http://127.0.0.1:7621`) because one process
must own the USB camera; with no cockpit up they answer with an in-band error
and the command to start one, and the robot tools keep working. An unreachable
controller is likewise reported in-band (`robot unreachable at …`), not as a
server crash.

**Windows laptop** (today's brain): `scripts\setup-windows.ps1` installs uv
(winget `astral-sh.uv`), downloads and runs the Intel RealSense SDK 2.0
installer from the librealsense GitHub release (`RealSense.SDK-WIN10-<ver>.exe`
— the default install puts `realsense2.dll` under
`C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64\`, and the script
sets `REALSENSE_LIB` to it), runs `uv sync` and the doctor. Then
`scripts\cockpit.ps1 -Cell ur20` is the pilot's seat. No `sudo` story on
Windows: librealsense uses the native backend there. **Not yet run on the
laptop** as of 2026-09-12 — the first run is the verification.

## Hand-eye calibration (touch-and-click)

The bracket seed is a drawing, not a measurement. `perception/calibrate.py`
replaces it with a solve from the robot's own poses — no target to print, no
extra dependency. In the cockpit's Robot panel open **Calibrate hand-eye**
(or use the `cal_*` MCP tools):

1. **Record mark** — freedrive the tool tip onto a mark on the table; the live
   TCP position becomes the mark in the base frame. Back the tool off.
2. **Add view** ×4–6 — from different poses (rotate the wrist, change tilt and
   height: the *rotation diversity* between views is what makes the camera
   offset observable) click the mark in the colour view, then Add view. Each
   view stores the flange pose and the clicked point (5×5 median of valid
   depth, `GET /api/point`).
3. **Solve** — Levenberg–Marquardt on the six parameters of `T_flange_color`,
   seeded from the bracket, then composed with the SDK's depth→colour
   extrinsics into `T_flange_depth`. The table shows the pose, the Δ from the
   seed, RMS + per-view residuals, rotation diversity and warnings (diversity
   < 10°, RMS > 5 mm). Drop the worst view with × and re-solve.
4. **Apply + save** — uses it immediately and writes
   `captures/calibration/handeye_<cell>.json`; later starts load it
   (precedence: `PERCEPTION_T_FLANGE_CAMERA` env > that file
   (`PERCEPTION_HANDEYE_FILE` to relocate) > bracket seed). A solve with
   warnings is refused unless forced (the page asks).

Without a touch the mark is solved jointly (≥ 4 views) at some cost in
conditioning. The synthetic test (`tests/test_calibrate.py`) recovers a known
transform from noisy views to a few mm; the real check is Locate + Move
landing on the object after applying.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `failed to set power state` / `RS2_USB_STATUS_ACCESS` on macOS **without** `sudo` | libusb can't take the interfaces from Apple's UVC driver unprivileged | run under `sudo`; close any app holding the camera |
| the same **under `sudo`**, or the pipeline starts but `Frame didn't arrive within 5000` on the first read | **A race with macOS's own camera driver, lost.** Verified in the unified log on macOS 26 (Darwin 25): to claim the camera, libusb (as root) issues a USB reset (`AppleUSBHostPort::terminateDevice … reset API call`); the device re-enumerates, and within ~60 ms both our process and Apple's `UVCAssistant` (the userspace UVC extension) try to open each interface exclusively (`openGated: failed to open … already opened for exclusive access by pid 588, UVCAssistant`). Whoever wins each interface holds it: lose the depth interface → `RS2_USB_STATUS_ACCESS`; lose only the RGB one → a half-alive pipeline that never delivers a frameset. Reproduced once on 2026-09-03 after a burst of failed one-shot opens; after a re-plug, `scripts/rs_probe.sh` then streamed at 30 fps on both the first open **and** a clean re-open, so a healthy camera re-opens fine — it is the *failed* opens that cascade (each resets the device and re-runs the race). | Unplug and re-plug the camera to clear it, then confirm with `scripts/rs_probe.sh` (headless: runs the cockpit twice and prints frames/fps/last_error + who won each USB interface in the OS log; `--fake` self-tests without root). Don't retry failed opens in a tight loop — the cockpit backs off 1→30 s. Prefer one long-lived process (the cockpit, tunnelled over SSH with `ssh -L 7621:127.0.0.1:7621`) to back-to-back one-shot commands. macOS dev-box only: Linux/Jetson has no competing UVC daemon once the udev rules are installed. |
| USB 2 link (`usb 2.x` in `rs-info` / the header) | depth + colour at 640×480@30 exceeds the USB 2 budget, the SDK just stops delivering | the camera auto-caps at 15 fps on USB 2 (`--rs-fps` forces); move to a direct USB 3 port for 30 |
| same on Linux | udev rules missing | install librealsense's `99-realsense-libusb.rules`, re-plug |
| `librealsense2 not found` | SDK not installed / not on the search path | `brew install librealsense`, or set `REALSENSE_LIB` |
| stream stalls after a while | USB-C cable too long / hub | RealSense is picky: ≤ 2 m active-free cable, direct port |
| depth readout flickers by mm–cm on a static scene | raw stereo noise (grows with distance²); or the filters were turned off | leave the defaults on (see *Depth quality*); check `/api/info` → `camera.depth.filters` is non-empty and `tuning_applied` says `ok`; get the camera closer to the work |
| colour panel is black (mean RGB 0,0,0), depth fine, room lit, RGB options at factory (`rs-info --options`) | depth and colour streaming at **different resolutions** (seen with 848×480 depth + 640×480 colour on a D435, fw 5.12.7.100) | keep them equal — the default now does; if you pass `--width/--height`, pass a matching `--depth-res`. The hardware test asserts the colour frame isn't black. |
| first open of a process never delivers a frame, re-opens in the same process do | sensor option writes landed between pipeline start and the first frameset (fw 5.12.7.100, macOS libusb backend) | fixed — `DepthTuning` is applied on the first `read()`; if you add sensor writes, put them after a frameset has arrived (`tests/test_realsense.py::test_tuning_waits_for_the_first_frameset`) |
| `Couldn't resolve requests` / pipeline start fails right after this change | the depth sensor doesn't offer 848×480 (D405, some firmware) | `--depth-res 640x480` |
