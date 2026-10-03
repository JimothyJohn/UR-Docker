# RealSense on the robot — capture, segment, measure

The `perceptronics` package can read an Intel RealSense D4xx directly (color +
aligned depth), segment an object with a click, and measure it in metres: the
object's centroid, 3D position, size and orientation, now, for a robot to act on.

```
perceptronics rs-info                       # SDK + attached cameras
perceptronics gui                           # the RGB-D cockpit (browser, loopback)
perceptronics gui --fake                    # same, synthetic scene, no camera needed
perceptronics --segment-backend sam gui     # Segment Anything instead of region growing
```

## Where the compute lives

| Option | Status | Notes |
| --- | --- | --- |
| **Jetson Orin next to the robot** (target) | designed for; container in `Dockerfile.perceptronics` | Camera on the tool flange (`hardware/d435-tool-bracket/`), USB back to the Jetson, this package runs as a service there (`compose --profile perceptronics`). The Jetson also runs `urctl` against the controller over the network, so one box owns perception *and* motion. |
| **Laptop (this Mac)** | works with the lean open | `brew install librealsense`; the SDK needs **root to claim the USB interface on macOS** (libusb has to detach Apple's UVC driver): `sudo python3 -m perceptronics --cell ur3 gui --rs-lean`. The default open loses the claim race to Apple's UVC daemon (*Troubleshooting*); the **lean open** (`--rs-lean`) streamed on 2026-09-25 — the first open still errors once, the cockpit's back-off re-opens and the stream then holds. Launch from a local Terminal: the extra webcam views go through TCC, which silently denies SSH sessions. |
| **On the UR controller itself** | not attempted | The CB is a Debian box with USB, so librealsense *could* be built there, but it shares the CPU with URControl's real-time loop, UR re-images it on update, and there is no supported way to ship a service with the robot. The URSim container cannot see USB at all (Docker Desktop on macOS has no USB passthrough). Deferred; the Jetson makes it unnecessary. |

## How it works

`perceptronics/realsense.py` binds librealsense's **C API with ctypes** — no
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
| **Depth resolution** | 848×480 | `--depth-res WxH`, `PERCEPTRONICS_RS_DEPTH_WIDTH/_HEIGHT` | The D435's native stereo mode; the ASIC derives 640×480 by downscaling, so 848×480 is the accuracy-optimal choice (Intel's D400 tuning guide). **Colour follows the depth size** unless you set `--width/--height` (or `PERCEPTRONICS_WIDTH/HEIGHT`): on this D435 (fw 5.12.7.100, macOS) 848×480 depth next to 640×480 colour returned **all-black colour frames** with every sensor option at factory and a lit room, while 640/640 and 848/848 were both fine. Keep the two sensors at the same size. A request the camera can't serve (USB 2 has no 848×480 colour) is renegotiated at open to the fastest same-size pair it lists — see *Troubleshooting → USB 2 link*. |
| **Post-processing chain** | on | `--no-depth-filters`, `PERCEPTRONICS_RS_FILTERS=0` | `DepthFilters`: depth→disparity → **spatial** (edge-preserving smoothing) → **temporal** (per-pixel EMA over frames, persistence "valid in 2 of the last 4") → disparity→depth, applied to the depth frame *before* alignment in Intel's recommended order. Hole filling and a min/max threshold exist on the dataclass but are off — hole filling invents depth, which is wrong for measuring. |
| **Sensor tuning** | `high_density`, laser max | `--rs-preset NAME\|none`, `--laser-power MW\|max\|none`, `PERCEPTRONICS_RS_PRESET`, `PERCEPTRONICS_RS_LASER_POWER` | `DepthTuning`, written once **after the first frameset arrives** (writing them between pipeline start and the first frame stalled a freshly claimed D435 on macOS — no frame ever came; verified 2026-09-04): the *High Density* visual preset keeps low-confidence matches — chosen 2026-10-03 because *High Accuracy*'s higher confidence threshold left a dark, printed 30 mm-wide box top 57 % holes at 0.72 m and the detector missed the box (High Density: 92 % depth, 4 of 4 found; `tests/fixtures/d435`); full projector power puts more texture on flat surfaces. Best effort — an unsupported or refused option is reported in `/api/info` → `camera.depth.tuning_applied`, never fatal. `none` leaves a sensor you tuned in realsense-viewer alone. |
| **Lean open** | off | `--rs-lean`, `PERCEPTRONICS_RS_LEAN=1` | The fewest USB handle opens per start: no USB-type probe, no stream-mode enumeration (the configured mode is requested as is — pick a listed one), no preset / laser writes (`tuning_applied` says `skipped`), and `RS2_OPTION_GLOBAL_TIME_ENABLED` switched off on every sensor right after the pipeline starts (the SDK otherwise polls the hardware monitor for its clock fit while streaming; result per sensor in `/api/info` → `camera.global_time_off`). Built 2026-09-25 for the macOS claim race (*Troubleshooting*) and **verified the same day on the Mac Studio**: the first open still fails once (the race), the cockpit's back-off re-opens, and the lean stream then holds — the desktop Mac is a D435 host again with this flag.  |
| **Extra viewpoints** | none | `--view DEVICE` (repeatable), `PERCEPTRONICS_VIEWS=a,b`, `--view-res WxH`, `--view-fps N` | Plain webcams shown under the colour/depth pair (the **2×2 grid**; one view spans the row, a third is not laid out) and written into every snapshot (`POST /api/snapshot` → `views[].path`, so an agent reading the snapshot sees the robot from the room, not only from the wrist). `perceptronics/views.py`: **ffmpeg** (an external binary, like `ssh`/`docker` — zero Python deps) opens the device with the OS capture API and pipes MJPEG back; `GET /api/view/<i>?after=SEQ` long-polls one JPEG (`X-Seq`, `X-Fps`). macOS picks the device **by AVFoundation name** (`ffmpeg -f avfoundation -list_devices true -i ""`; indices shift when a camera is plugged in), Linux takes `/dev/videoN`, Windows the dshow name (both unverified); `lavfi:testsrc` is a camera-less source and `--fake` makes every view synthetic. Views measure nothing — no intrinsics, no hand-eye. Verified 2026-09-25 with two C920s (`perceptronics/cells/ur3.env`). |

The filter blocks are `rs2_processing_block`s fed whole framesets, exactly like
the `align` block, so each is one `rs2_process_frame` + queue wait per frame.
The temporal filter is what steadies a static scene; it lags on moving objects
by a few frames, which is the trade. `tests/test_realsense_hw.py::
test_filtered_depth_is_steadier_than_raw` measures per-pixel temporal noise in
a centre patch, raw vs default, on the attached camera
(`sudo python3 -m pytest -m realsense -q -s` prints both numbers).

What no filter fixes: anything under ~28 cm returns nothing; dark matte,
shiny, or transparent surfaces defeat stereo; direct sunlight washes out the
IR projector. 30–60 cm from the workpiece is the good zone for the bracket.

The cockpit (`perceptronics/webapp.py` + `perceptronics/webui/index.html`) is a
stdlib HTTP server (the only GUI since `urctl gui` was retired on 2026-09-26). One thread pumps the camera;
the page long-polls `/api/rgbd` for a binary container (JSON header + colour
PNG + zlib'd `uint16` depth), inflates the depth with `DecompressionStream`
and colourises it in the browser. Hover measures; click posts `/api/segment`.

## Segmenting and the features you get

| Backend | Select | What it does |
| --- | --- | --- |
| `stub` (default) | — | Region growing from the click by colour similarity **and depth continuity** (a neighbour joins only if its depth is within 2 cm). Pure Python, ~10 ms. Also `nearest_object` — RealSenseTrainer's "closest thing" rule. |
| `sam` | `--segment-backend sam` / `PERCEPTRONICS_SEGMENT_BACKEND=sam`, `python3 -m pip install -e ".[sam]"` | Segment Anything (`facebook/sam-vit-base` via transformers) with the click as the point prompt; CUDA on the Jetson, `mps` on a Mac. First run downloads ~375 MB. Not exercised in CI. |

`extract_features(mask, frame)` returns, per object: area, bbox, centroid,
mean colour, depth median/min/max, **3D centroid in the camera frame**, metric
extent (bbox at the object's depth), thickness, principal-axis orientation,
major/minor axis lengths, elongation, fill ratio, and a labelled *assumption*
of a grasp (top-down, close across the minor axis). Turning that into a base-
frame pick needs the hand-eye transform — the bracket spec gives a nominal
`T_flange_camera` seed, and `urctl` supplies the flange pose.

## Running it on the Jetson (container)

```bash
docker compose --profile perceptronics build      # builds librealsense (RSUSB backend) from source
docker compose --profile perceptronics up -d      # privileged for USB; cockpit on :7621, bound 0.0.0.0
```

Verified 2026-09-02: the image builds for `linux/arm64` (librealsense v2.58.4,
RSUSB backend, ~10 min on Apple Silicon), the SDK loads inside it, and
`perceptronics rs-info` / `rs-capture --fake` run. USB itself was not exercised
(Docker Desktop on macOS cannot pass the camera through).

The container has no auth — it is a cell-network cockpit.
Keep it off routable networks or put it behind the Jetson's firewall.

## Click or drag to segment

In the cockpit, a **click** sends a point prompt and a **press-drag-release**
sends a box prompt (`/api/segment` takes `{"x","y"}`, `{"box":[x0,y0,x1,y1]}`,
or both — a point inside a box disambiguates). Boxes are `x1`/`y1`-exclusive
pixel edges, corner order doesn't matter, and anything off-frame or empty is a
400 rather than a clamp. The last box is drawn dashed until you clear.

Backends (`--segment-backend`, `$PERCEPTRONICS_SEGMENT_BACKEND`):

| backend | what it does | box prompt |
| --- | --- | --- |
| `stub` (default) | colour + depth region growing, pure Python | grows from the box centre (or the click) and clips to the box |
| `sam` | Segment Anything via `transformers` (`python3 -m pip install -e ".[sam]"`) | native SAM box prompt, single-mask decode |

SAM checkpoint (`--sam-model`, `$PERCEPTRONICS_SAM_MODEL`, any `SamModel`-loadable
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
   training TCP; the cell file says `PERCEPTRONICS_APPROACH_REFERENCE=flange`,
   `PERCEPTRONICS_STANDOFF_M=0.075` — "flange 75 mm above the part"). Locate also
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

**Hand-eye transform.** `perceptronics/handeye.py` seeds `T_flange_depth` from
the bracket geometry (`hardware/d435-tool-bracket/README.md` §3,
`ARM_ANGLE_DEG = 90`, camera on the tool-I/O side: camera x = flange −X,
camera y = flange −Y, camera z = flange +Z, depth origin at (17.5, 66.5, 1.7) mm)
and takes
`T_depth_color` from the SDK's extrinsics at open (`rs2_get_extrinsics`,
~15 mm along x on a D435; identity on the synthetic camera). That is an
**uncalibrated seed** — a printed part won't hold ±1°, and 1° at 0.5 m is
~9 mm. Replace it with a hand-eye calibration via
`PERCEPTRONICS_T_FLANGE_CAMERA="[x, y, z, rx, ry, rz]"` (the depth frame in the
flange frame, metres + UR rotation vector); the panel's *hand-eye* row says
which one is active. Pose arithmetic is `urctl/pose.py` (Rodrigues,
`pose_trans`/`pose_inv` with URScript semantics, pure stdlib) and is
cross-checked against the controller's own `pose_trans` on every locate
(the *host/controller Δ* row).

Flags / env: `--robot-host` (`$UR_HOST`, default localhost = URSim),
`--robot-dry-run` (validate + audit, send nothing; a stand-in flange pose
lets the whole flow run on `--fake`), `--no-robot` (no panel). The
standoff and its reference are per-click in the panel, defaulting to the
cell's `PERCEPTRONICS_STANDOFF_M` / `PERCEPTRONICS_APPROACH_REFERENCE` (0.10 m from
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

**Cell profiles** (`perceptronics/cells/{sim,ur3,ur20}.env`, `--cell NAME` on any
`perceptronics` entry point or `UR_CELL=NAME`): one word selects the robot's
host/platform/ports and the bracket print (`PERCEPTRONICS_BRACKET=eseries|ur20`,
which picks the hand-eye seed — `perceptronics.handeye.BRACKET_SEEDS`). A variable
already set in the shell wins over the file. The real cells ship with `UR_HOST`
empty: fill in the controller IP once. `perceptronics cells` prints them;
`eval "$(python3 -m perceptronics cells --export ur20)"` puts the same variables in
your shell so plain `urctl` commands target the cell too.

**Doctor** (`perceptronics --cell ur20 doctor [--stream] [--no-robot] [--json]`,
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

**The keys for an agent** (`perceptronics-mcp --cell ur20`, wired in `.mcp.json`
for Claude Code): one MCP server serving the whole `urctl` robot registry plus
the camera family — `cam_info`, `cam_snapshot` (writes colour/depth/mask PNGs
an agent can *look at*), `cam_segment`, `cam_nearest`, `cam_clear`,
`cam_locate`, `cam_move_to_approach`, `cam_events`,
`cell_doctor`, `cell_jog`. The camera tools go **through the running cockpit**
(`PERCEPTRONICS_COCKPIT_URL`, default `http://127.0.0.1:7621`) because one process
must own the USB camera; with no cockpit up they answer with an in-band error
and the command to start one, and the robot tools keep working. An unreachable
controller is likewise reported in-band (`robot unreachable at …`), not as a
server crash.

**Windows laptop** (today's brain): `scripts\setup-windows.ps1` installs Python when none
is found (winget `Python.Python.3.13`), downloads and runs the Intel RealSense SDK 2.0
installer from the librealsense GitHub release (`RealSense.SDK-WIN10-<ver>.exe`
— the default install puts `realsense2.dll` under
`C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64\`, and the script
sets `REALSENSE_LIB` to it) and runs the doctor. Then
`scripts\cockpit.ps1 -Cell ur20` is the pilot's seat. No `sudo` story on
Windows: librealsense uses the native backend there. **Not yet run on the
laptop** as of 2026-09-12 — the first run is the verification.

## Hand-eye calibration (touch-and-click)

The bracket seed is a drawing, not a measurement. `perceptronics/calibrate.py`
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
   (precedence: `PERCEPTRONICS_T_FLANGE_CAMERA` env > that file
   (`PERCEPTRONICS_HANDEYE_FILE` to relocate) > bracket seed). A solve with
   warnings is refused unless forced (the page asks).

Without a touch the mark is solved jointly (≥ 4 views) at some cost in
conditioning. The synthetic test (`tests/test_calibrate.py`) recovers a known
transform from noisy views to a few mm; the real check is Locate + Move
landing on the object after applying.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `failed to set power state` / `RS2_USB_STATUS_ACCESS` on macOS **without** `sudo` | libusb can't take the interfaces from Apple's UVC driver unprivileged | run under `sudo`; close any app holding the camera |
| the same **under `sudo`**, or the pipeline starts but `Frame didn't arrive within 5000` on the first read | **A race with macOS's own camera driver, lost.** Verified in the unified log on macOS 26 (Darwin 25): to claim the camera, libusb (as root) issues a USB reset (`AppleUSBHostPort::terminateDevice … reset API call`); the device re-enumerates, and within ~60 ms both our process and Apple's `UVCAssistant` (the userspace UVC extension) try to open each interface exclusively (`openGated: failed to open … already opened for exclusive access by pid 588, UVCAssistant`). Whoever wins each interface holds it: lose the depth interface → `RS2_USB_STATUS_ACCESS`; lose only the RGB one → a half-alive pipeline that never delivers a frameset. Reproduced once on 2026-09-03 after a burst of failed one-shot opens; after a re-plug, `the cockpit's `--rs-lean` open` then streamed at 30 fps on both the first open **and** a clean re-open, so a healthy camera re-opens fine — it is the *failed* opens that cascade (each resets the device and re-runs the race). | Unplug and re-plug the camera to clear it, then confirm with `the cockpit's `--rs-lean` open` (headless: runs the cockpit twice and prints frames/fps/last_error + who won each USB interface in the OS log; `--fake` self-tests without root). Don't retry failed opens in a tight loop — the cockpit backs off 1→30 s. Prefer one long-lived process (the cockpit, tunnelled over SSH with `ssh -L 7621:127.0.0.1:7621`) to back-to-back one-shot commands. **Any app that watches for cameras joins the race** (2026-09-24, Mac Studio used as a desktop with Spotify, a browser and two webcams attached): the probe's winners list showed `winner=Spotify` on the device, after which every `set_xu` control transfer timed out and no frame came — the Chromium shell inside Spotify opens UVC devices whenever one enumerates, which libusb's claim makes happen on every open. The 2026-09-03 success was a headless box reached over SSH with nothing else running. Quit camera-aware apps (`osascript -e 'quit app "Spotify"'`) before opening the D435, and read the probe's *who won each USB interface* list — anything other than `python3.13` and `UVCAssistant` there is a contender to close. A device that disconnects during the open also **segfaults the process** (librealsense 2.58.4 → `libusb_control_transfer` → `darwin_submit_transfer` on the dead handle, crash reports in `~/Library/Logs/DiagnosticReports/python3.13-*.ips`); the cockpit's backoff can't survive that, so the fix is upstream of the open, not a retry. **2026-09-25, the controlled run: both webcams unplugged 30 min earlier, Spotify closed, the D435 alone on the bus at SuperSpeed — still no stream, so the webcams were never the variable.** The unified log shows the real shape of the race. libusb's root-only kernel-driver detach (`USBDeviceReEnumerate` with capture, in libusb since 1.0.25) **resets the camera on every handle open**; librealsense opens a handle per sensor and per interface; and `UVCAssistant` re-matches the freshly enumerated device within ~30 ms every single time — **43 resets in 40 s** for one cockpit start, python losing roughly one interface claim in six (`Depth@0`, `RGB@4`, `Y@2` all seen). Worse, the SDK opens *more* handles after streaming has started (hardware-monitor polling for global timestamps, the preset / laser writes), and each of those resets kills the stream already running: the first open delivered 2 frames, then `uvc streamer watchdog … endpoint 130` and `usb device disconnected`. None of this is reachable from the ctypes binding. **Verdict: the Mac Studio used as a desktop is not a host for the D435.** The verified hosts are the Windows laptop under WSL2 (2026-09-23) and Linux/Jetson. The one lever not yet tried is *fewer handle opens per start* (skip the mode enumeration, no preset / laser writes, `RS2_OPTION_GLOBAL_TIME_ENABLED` = 53 off) — built the same day as the **lean open** (`--rs-lean` / `PERCEPTRONICS_RS_LEAN=1`, table above): it removes every handle open that is ours, so what is left is the SDK's own;  Two more explanations checked and refuted on 2026-09-25: the software is identical to 09-03 (macOS 26.6.2 installed 08-23, librealsense 2.58.4 + libusb 1.0.30 on 09-02), and a desktop session was logged in on 09-03 as well (`last`: console Sep 2 19:03 → Sep 8), so "headless" was never the difference; a fresh-from-boot `UVCAssistant` also lost on 09-24 23:20 (the Mac had rebooted at 21:55). The daemon then **segfaulted twice** (`setupInterruptReads` on a device the reset had just destroyed, crash reports 23:20:50 and 23:21:09), so the storm breaks both sides. Why 09-03 streamed twice in a row is unexplained and outside log retention; with the observed loss rate it was at best an unlikely draw. Probe caveat: the old winners list was the last 15 lines of the last 5 minutes and dropped python's own losses, which is how two sessions read "python won every interface"; it now covers the probe's run, counts both directions and ends with the reset count. macOS dev-box only: Linux/Jetson has no competing UVC daemon once the udev rules are installed. |
| the camera enumerates as **USB 2** on a USB 3 port (`usb 2.1` in `rs-info`, the device sits under a `USB2 Hub@…` node in `ioreg -p IOUSB`) and later logs `usb device disconnected` mid-open | a marginal cable: the SuperSpeed pairs failed link training and the device fell back to USB 2, then dropped (2026-09-24, Mac Studio rear port) | swap the cable — with a good one the same port enumerated the D435 at SuperSpeed (`Device Speed = 3`, under the `USB3 Gen2 Hub`) |
| USB 2 link (`usb 2.x` in `rs-info` / the header) | the D435 offers a **shorter mode list** on USB 2 — no 848×480 colour at all, 848×480 depth only at 10/6 Hz (`sudo rs-enumerate-devices`, fw 5.12.7.100, 2026-09-24) — so the USB 3 default (848×480 both) is unresolvable and the pipeline refuses to start (`Couldn't resolve requests`); and 640×480@30 on both exceeds the USB 2 budget, the SDK just stops delivering | nothing to do: `open()` reads the camera's own profile list (`Api.stream_modes`) and `negotiate_mode` puts both streams at the fastest size they share — 640×480 @ 15 on this camera — and says so on stderr (`describe()["negotiated"]`). `--rs-fps` still forces the rate; a direct USB 3 port gives 848×480 @ 30 |
| same on Linux | udev rules missing | install librealsense's `99-realsense-libusb.rules`, re-plug |
| `librealsense2 not found` | SDK not installed / not on the search path | `brew install librealsense`, or set `REALSENSE_LIB` |
| stream stalls after a while | USB-C cable too long / hub | RealSense is picky: ≤ 2 m active-free cable, direct port |
| depth readout flickers by mm–cm on a static scene | raw stereo noise (grows with distance²); or the filters were turned off | leave the defaults on (see *Depth quality*); check `/api/info` → `camera.depth.filters` is non-empty and `tuning_applied` says `ok`; get the camera closer to the work |
| colour panel is black (mean RGB 0,0,0), depth fine, room lit, RGB options at factory (`rs-info --options`) | depth and colour streaming at **different resolutions** (seen with 848×480 depth + 640×480 colour on a D435, fw 5.12.7.100) | keep them equal — the default now does; if you pass `--width/--height`, pass a matching `--depth-res`. The hardware test asserts the colour frame isn't black. |
| every hover lands centimetres off although `PERCEPTRONICS_T_FLANGE_CAMERA` looks plausible | the hand-eye is a seed (bracket nominal, or an old solve rotated after the bracket was re-clocked); 2026-09-25 the rotated 09-23 solve was 5-10 cm wrong | re-solve; with nothing to touch, use the orbit calibration (section below) |
| no depth on the part under the fingers at hover height; `locate` under-estimates its top | the camera is inside the D435's ~0.2 m minimum range whenever the fingertips are within ~40 mm of the part | measure from >= 0.25 m, take the top face as the nearest 3-D plane (not the blob's median depth), then descend on that height |
| a straight descent puts a fingertip on the part's edge and protective-stops | the tool axis was 12 deg off vertical, so 180 mm of gripper puts the fingertips ~38 mm sideways of the flange | target the fingertips: flange = tip - R_flange*(0,0,L), L = flange-to-fingertip (Hand-E 157 mm + 6 mm adapter) |
| a view panel says `Input/output error` / `not permitted`, or ffmpeg opens nothing, while `ffmpeg -list_devices` lists the webcam | macOS TCC: camera access is granted per *launching* app; an SSH session (`sshd-session` is the responsible process) is denied **without a prompt** (`tccd: Policy disallows prompt … kTCCServiceCamera denied`, 2026-09-25) | launch the cockpit from a local Terminal/iTerm (allow it once under Privacy & Security → Camera); the RealSense is unaffected (libusb, not AVFoundation) |
| `PermissionError: captures/snapshots` from a non-root cockpit | an earlier `sudo` run created `captures/` as root | `sudo chown -R $USER captures` |
| first open of a process never delivers a frame, re-opens in the same process do | sensor option writes landed between pipeline start and the first frameset (fw 5.12.7.100, macOS libusb backend) | fixed — `DepthTuning` is applied on the first `read()`; if you add sensor writes, put them after a frameset has arrived (`tests/test_realsense.py::test_tuning_waits_for_the_first_frameset`) |
| `Couldn't resolve requests` / pipeline start fails | the requested mode isn't on the camera's list *and* no same-size pair could replace it (negotiation only steps in when both sensors share a size at or below the requested rate) | `sudo rs-enumerate-devices` to see what's offered; `--depth-res` / `--width/--height` / `--rs-fps` to a listed pair |


## Hand-eye without a mark (orbit calibration)

`CalibrationSession` solves the mark jointly when none was recorded
(`MIN_VIEWS_WITHOUT_MARK`). Used on 2026-09-25 with a Hand-E on the flange, so
nothing could touch a mark: a foam block's top-face centre is the mark, the
wrist orbits it, and every view is a click on that centre through
`POST /api/cal/view`. What made it converge:

- **Range diversity.** Views at one range leave the camera offset and the mark
  trading off: three seeds gave three answers with the same residual. Use
  three ranges (0.2 / 0.3 / 0.4 m) plus tilts of 15 deg and yaws of 30 deg
  about the mark, and lateral shifts.
- **The click is the top face's 3-D centroid** (plane fit, points within 5 mm),
  not the nearest white pixels, which bias toward the near edge under tilt.
- **Track the block by identity.** Predict the mark's pixel from the current
  solve and reject any candidate whose top-face point is more than 40 mm from
  it; a neighbouring block sneaking into the set is what a 60 mm residual means.
- **Trim.** Drop views above ~9 mm residual and re-solve; the UR3e solve went
  from 9.3 mm over 23 views to 4.4 mm over 16 with no warnings.

Then `POST /api/cal/apply` puts it in force and saves `handeye_<cell>.json`;
paste the `env_line` into the cell file.

**The command (2026-09-26, `perceptronics/orbitcal.py`):** centre one block under
the camera at about 0.3 m, then

```bash
python3 -m perceptronics --cell ur3 calibrate --dry-run   # find the mark, print the 39-view plan, move nothing
python3 -m perceptronics --cell ur3 calibrate             # orbit, click every view in, solve, trim; prints the env_line
python3 -m perceptronics --cell ur3 calibrate --apply     # …and put it in force + save handeye_<cell>.json
```

It drives the arm directly over Primary (`tcp=[0]*6`, 0.08 m/s; `--via-cockpit`
to go through the cockpit's API instead) and uses the running cockpit for vision
and for the calibration session itself (`/api/cal/*`), so the cockpit's Calibrate
panel shows the views arriving. Each range is planned from the *current* solve —
the seed, then the running solve once four views are in — so a seed centimetres
off still lands (fake cell: 70 mm / 6° seed → 1.4 mm). The block is found again at
every view by identity (nearest top-face centroid to the predicted mark, 120 mm
gate before the first solve, 40 mm after). A protective stop unlocks and moves
on; a view the envelope refuses or the arm can't reach is skipped and named; the
worst view is dropped while any residual exceeds `--trim-mm` (9). Ctrl-C stops
the program. `tests/test_orbitcal.py` runs it end to end on a fake cell (a
rendered block, a real `CalibrationSession` from a wrong seed) plus the fault
campaign; **not yet run on the UR3e** — the `scripts/pilot/orbit_cal*.py`
scratch scripts stay as the proven fallback until it is.

## Picking with the Hand-E

The Robotiq URCap daemon speaks on the controller's loopback only
(`127.0.0.1:63352`), so `urctl gripper` / `ur_gripper` / `POST /api/robot/gripper`
drive it from URScript over Primary. A pick that worked on 2026-09-25: measure
the block from >= 0.25 m, hover with the **fingertips** 40 mm above its top,
descend to the top level and check both webcam views for the block between the
fingers, descend 15 mm more, close (`OBJ` 2 = held, `POS` 114 = 27.6 mm opening
on a 27 mm block), lift. The webcams are the arbiter at every step; the wrist
camera is blind that close.
