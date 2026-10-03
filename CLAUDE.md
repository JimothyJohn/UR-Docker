# CLAUDE.md

Working notes for AI assistants (and humans) on **perceptronics**: the
distribution, the `perceptronics` package and CLI, the `PERCEPTRONICS_*`
variables (renamed from `perception` / `PERCEPTION_*` on 2026-09-28; the
tools warn about a stale `PERCEPTION_*`), the compose project, the docs and
the cockpit. `urctl`, the `UR_*` variables and the URCap bundle IDs keep
their names. There is no PyPI package. It started as a sandbox + tooling
repo for **Universal Robots e-Series** control. Most of what's interesting
here lives in the network protocols and file formats UR ships — not in the
code itself — so this file captures the things that took me real time to
discover.

**Hardware, addresses, what is connected today: `SETUP.md`.** The cells, the
computers, the simulators that run on this Mac, the USB stick — dated, with the
field-log entries that matter. Update it when the cell changes.

## What this repo is

A Docker-Compose'd PolyScope simulator (URSim 5.26.0 LTS, e-Series) plus a
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
`scripts/` are kept as standalone shell/Python helpers; `scripts/urp_convert.py`
is the single source of truth for the converter (vendored into the wheel as
`urctl/_urp_convert.py`).

**Deep introspection (the harness layer, see `docs/harness.md`):** the toolkit
can read *everything* about a controller, not just live state. `urctl
rtde-state --deep` pulls the full RTDE diagnostic recipe (per-joint
currents/temps/drive modes, TCP force, supply power, tool telemetry, speed
scaling — decoded to names by `urctl/codes.py`; unsupported fields drop
gracefully). `urctl snapshot` fuses that with filesystem introspection
(`urctl/sysinfo.py` — SSH on a real robot, docker exec on URSim, auto-inferred
from the host) into one cell model: joint serials/firmware with replacement
detection, kinematic-calibration mismatch, storage health, program inventory,
installed URCaps, and the parsed active installation
(`urctl/installation.py`) — whose `flange_tcp` flag is the decision input for
the MoveJ-node-vs-script-node authoring choice below. `urctl programs` lists
what the operator can load. All three are also agent tools
(`ur_rtde_state`/`ur_system_snapshot`/`ur_list_programs`) and therefore MCP
tools.

**Portability:** the runtime — `urctl`, the GUI, and `urctl-mcp` (rewritten
as stdlib JSON-RPC/stdio; the `mcp` extra is now an empty no-op) — has **zero
dependencies** and runs on Windows/macOS/Linux, amd64/arm64 (CI tests all
four). External binaries are only reached for by the *filesystem* layer and
checked with clear errors: `ssh` (real robot; ships with all three OSes) and
`docker` (URSim). CLI entry points reconfigure stdout with errors="replace" so
legacy Windows codepages never crash on unicode.

**RealSense RGB-D (`perceptronics rs-info` / `gui`, see
`docs/realsense.md`):** `perceptronics/realsense.py` binds librealsense's C API
with ctypes (no `pyrealsense2`; zero deps kept), streams colour + depth aligned
to colour (both sensors at the D435's native 848×480 — **mismatched sizes give
black colour frames** — through the SDK's spatial + temporal filter chain,
sensor on the High Accuracy preset at full laser; `docs/realsense.md` §Depth
quality; `--no-depth-filters` / `--rs-preset none` for raw), and the cockpit (`perceptronics/webapp.py` + `perceptronics/webui/`) does
hover-to-measure, click-to-segment (`perceptronics/segment.py`: colour+depth
region growing, or SAM via the `sam` extra), snapshots (`POST /api/snapshot`), and a **Robot** panel that sends the segment's point to
the arm: `ur_flange_pose` (new tool) + the bracket-nominal hand-eye seed
(`perceptronics/handeye.py`, override with `PERCEPTRONICS_T_FLANGE_CAMERA`) give a
base-frame point and an approach pose; **Move** is one `ur_move_tcp` through the
same tool registry (`perceptronics/robotlink.py`; `docs/realsense.md` §Sending a
point to the robot). `urctl/pose.py` is the stdlib pose math (`pose_trans` /
`pose_inv` semantics). **Extra viewpoints** (`--view DEVICE`, `PERCEPTRONICS_VIEWS` in the
cell): plain webcams under the colour/depth pair (2×2 grid) and in every snapshot,
via ffmpeg (`perceptronics/views.py`; macOS picks devices by AVFoundation name; launch
from a local Terminal — SSH sessions are denied camera access by TCC). **macOS needs `sudo`** to open the camera (libusb
must detach Apple's UVC driver — `failed to set power state` otherwise); Linux
needs the udev rules. `--fake` runs everything on a synthetic scene. The
target compute is a Jetson Orin next to the robot: `Dockerfile.perceptronics` +
`docker compose --profile perceptronics`. The camera mounts on the tool flange via
`hardware/d435-tool-bracket/` (parametric CadQuery, STL/STEP, spec in its
README; nominal `T_flange_camera` seed in §3).

**The pilot's seat (cells, doctor, Pilot panel, MCP keys; `docs/realsense.md`
§The pilot's seat, presentation in `docs/realsense-cell.html`):** a *cell*
(`perceptronics/cells/{sim,ur3,ur20}.env`, `--cell` / `UR_CELL`) is the one-word
selector for robot host/platform/ports + bracket print (`PERCEPTRONICS_BRACKET`
→ `handeye.BRACKET_SEEDS`; the UR20 print is clocked 45°, so its seed differs).
`perceptronics doctor` is the pre-flight (SDK, camera, per-port reachability with
platform-mismatch diagnosis, modes, TCP offset, hand-eye status; every failure
carries its fix; also `GET /api/doctor` and the `cell_doctor` tool). The cockpit
has a Pilot panel (bring-up/stop/freedrive, capped jog pad, events log) and
`POST /api/snapshot` writes PNGs an agent can read. **Hand-eye calibration**
is touch-and-click (`perceptronics/calibrate.py`; cockpit "Calibrate hand-eye",
`cal_*` tools): record the mark with the tool tip, 4–6 clicked views from
varied wrist poses, LM solve seeded from the bracket, Apply + save to
`captures/calibration/handeye_<cell>.json` (env > file > seed). **Mark-less:**
`perceptronics calibrate` orbits the block under the camera (3 ranges × 13 views,
found by identity, trimmed) and clicks every view into the cockpit's session;
`--apply` saves it (`perceptronics/orbitcal.py`, `docs/realsense.md` §Hand-eye
without a mark; unverified on hardware as of 2026-09-26). `perceptronics-mcp` (`.mcp.json`)
serves robot + `cam_*`/`cell_*` tools; the camera tools proxy the running
cockpit because one process owns the USB camera. Windows bring-up:
`scripts/setup-windows.ps1` + `scripts/cockpit.ps1`. **Keep
`docs/realsense-cell.html` current** — it is the demo/explainer and has a dated
field log; append to it when something is verified or changes.

**Pick cycle (`perceptronics pick-cycle`, `perceptronics/pickcycle.py`):** the
heuristic, model-free routine — survey white blocks from one or more overlook
poses (`--survey-pose x y z rx ry rz`, repeatable, merged by position; the camera
must be ≥ 0.25 m from the parts, the D435 has no depth closer than ~0.2 m), fit
each top face as the nearest 3-D plane, then per block: yaw the fingers across
the short side, hover with the **fingertips** 40 mm over the top, descend to the
edge and 15 mm further, close, lift an inch, set it back, release; `--drop` adds a
pass that lets each block go from 120 mm up. A close on nothing opens and moves
on; a protective stop unlocks, lifts and moves on. It is a client of the running
cockpit (HTTP, like the MCP tools) and falls back to `urctl gripper` when the
cockpit predates the gripper route. `--record DIR` saves the three feeds + the
captioned events for `scripts/pilot/assemble.py`. Hand-E as mounted: the fingers
travel along flange **Y**; a yaw about the tool Z (pointing down) is the negative
of the base-heading yaw — `grasp_yaw_deg` handles it and a composed-pose test
locks it. First runs 2026-09-25 on the UR3e + Hand-E.

**The table is flat and parallel to base XY (Nick, 2026-09-27).** Only its height
and the parts' heights vary. The UR3e stands on a ~12 in pedestal (parts ~0.27 m below
the base) and the Hand-E adds 163 mm. `pick-cycle` grasps straight down
(`grasp_rotation`), leans 12°/24° outward only when the controller's IK can't solve the
vertical path, and skips a block no lean solves.

**The live pose in every frame (`perceptronics/posestream.py`, 2026-09-27).** The cockpit
streams RTDE `actual_TCP_pose` + `tcp_offset` at 30 Hz; flange = tcp ∘ offset⁻¹ (identical
to the `get_flange_pose` script on the UR3e, no Primary program, works in Local). Each
`/api/rgbd` header carries `flange_pose`, `pose_age_s`, `flange_to_color_pose`; `GET
/api/robot/pose`; `POST /api/objects` lists every white block with its top-face points.
The page (`webui/index.html`) is a scan-first ship's-computer UI: the depth as a point
cloud in the base frame (drag/wheel/WASD/Space/click-to-centre), light columns over
objects, the feed's target box re-projected through the live pose; `/classic` is the
old page (calibration lives there). Needs a cockpit restart to pick up server changes;
the page itself is read from disk per request.

**A lost link is named on the page (2026-09-30, Nick: "obvious to the user", "hotswapped without a
reboot or refresh").** The cockpit page (`webui/index.html`, `health()`) raises one alarm strip under
the header — COCKPIT NOT ANSWERING, CAMERA STOPPED · PICTURE IS n S OLD, ROBOT NOT ANSWERING, PENDANT
SWITCHED TO LOCAL — each with the fix and the fact that it reconnects by itself; the feed and scan dim
while their picture is old. All four clear without a refresh (the loops keep retrying; the camera pump
re-opens with back-off; a restarted cockpit is noticed by `uptime_s` going backwards and the page's
counters reset). The one case that needs F5 says so: a restart that serves a *different* page. A robot
that never answered this session (practice mode) is lamps + log only, not an alarm. Server side,
`fps()` decays to 0 when frames stop and `/api/info` carries `frame_age_s` / `stalled` (the doctor's
`stream` check uses them) — before, a dropped D435 kept reading ~30 fps. `/classic` has no strip.
Checked headless (kill + restart the cockpit, stop + restart the pump); not on the cell.

**Pick from the cockpit (`POST /api/robot/pick`, `perceptronics/pickplan.py`, 2026-09-27).**
Nick's spec: the gripper comes in straight down the base Z axis, wrist 3 across the
object's short side, the jaws pre-opened to their full 50 mm stroke (a 1.2x opening caught a
block's edge), fingertips 25 mm over its top.
Two programs: a blended sweep from the picture pose (`PERCEPTRONICS_HOME_POSE`; FANCY adds a
swing and a wrist flourish) to a **close look** — the object on the colour camera's axis at
0.24 m, just outside the D435's blind zone — then the object is re-found there and the
final program (over → approach [→ grasp → lift with PICK]) is built from that measurement.
Every pose goes to the controller's IK first. The look is **tilted 10°** (`LOOK_TILT_DEG`; the camera looks out at the part from the base side) and the re-find casts the detector's pixels onto the top plane, so the tilt doesn't bias it. A pick can name a stored object instead of the clicked mask (`target: {centre, theta, major_m, minor_m}` — the page sends it for a selected card), and `survey: true` stops after the close look and returns the fresh measurement; in the page, right-click a card or a part → APPROACH / PICK / SURVEY. The Robotiq pre-open rides the sweep program (`move_tcp_path(gripper_first=POS)`: set before the first `movel`, not waited for). The page's LEVEL lamp is the fitted floor's
tilt in the base frame: the table is flat, so any tilt is calibration error. The arm's
linkage comes from RTDE `actual_q` through `perceptronics/armfk.py` (UR3e DH verified 0.84 mm
against the controller's flange; other models guarded by the same check).

**Hand-eye drift, 2026-09-27 evening:** the 09-25 solve placed blocks 2–5 cm off,
view-dependently (the tilted survey and a vertical look disagreed by 26–28 mm; both
grasps missed). Re-run `perceptronics calibrate` before trusting a pick again.

**Approach by the fingertips, always (Nick, 2026-09-27: "the tool offset is
critical").** The cockpit's default approach reference is `fingertip`: the gripper's
fingertips (`PERCEPTRONICS_TIP_M` along flange +Z — 0.163 m = Hand-E 157 mm + 6 mm adapter,
the same length pick-cycle uses) sit `PERCEPTRONICS_STANDOFF_M` short of the object along the
**tool axis**, and every cockpit move, approach cycle and controller-IK reach check runs
with the TCP set to those fingertips (`RobotLink.reference_tcp`; `RobotLink.move` applies
it when a client sends only a pose, an explicit `tcp` still wins). The controller's active
TCP is never trusted for this — on the UR3e it is a 220 mm training offset the Hand-E
doesn't match. `flange` / `tcp` references remain for a cell with no tool; `perceptronics
doctor`'s `approach` line names the tool length and whether the active TCP differs. A new
cell with a different tool must set `PERCEPTRONICS_TIP_M` (the `ur20.env` stub says so).
Cell files also note where the work surface is relative to the base (UR3e: parts ~0.27 m
below it) — that height, not the datasheet radius, is what decides reach.

**One GUI.** The RGB-D cockpit (`perceptronics gui`) is the only web UI; the
older robot-only `urctl gui` / `urctl-gui` panel was retired on 2026-09-26
(it lives in git history before that commit). Its Pilot panel covers the
same bring-up / jog / stop / freedrive buttons through the same tool registry.

**Demo view (`perceptronics gui --demo`, `/?demo=1`, header **Demo** button):** the
RGB-D cockpit reduced to the picture, four big buttons (Start robot → Find
object → Pick, STOP), one status light and one instruction line; **Developer
view** toggles back. Same page, same API — `body.demo` CSS hides the rest, the
buttons call the existing bring-up / nearest / approach-cycle / stop actions.

**PolyScope X URCap (`urcap/` — `README.md` to install, `DEVELOPING.md` to work on it):** an
Application Node that embeds the cockpit's colour feed in PolyScope X and turns a
click into a base-frame point + approach pose (`/api/segment` → `/api/robot/locate`)
with two Move buttons — PolyScope's IK + auto-move screen, or the cockpit over
Primary — plus, since 0.3.0 (2026-09-29), the **pick areas + reach** on that node and
the **Perceptronic Pick** / **After picture N** program nodes: the PolyScope 5 kit's
node for PolyScope X (`pickscript.js` = `PickScript.java`'s port, same pick-server
protocol 2, held to the Python parser by `tests/test_urcapx_pick.py`). PolyScope X draws
a program node's presenter inside its 48 px tree row, so the Pick row is one line and
the screen is a PolyScope custom dialog (`dialogService.openCustomDialog`); a code
generator answers `{type: "$$ScriptBuilder", script, currentIndent}` (children indent by
`currentIndent`; the after-children builder carries the negative). Verified in the 10.13
sim: toolbox → row → dialog → picture point from PolyScope's joints → the script
compiled and run by Play (NEXT → movej → FIND over the network to a cockpit).
**`urcap/psx_matrix.py`** runs the e2e on the newest PolyScope X releases from the
URCap's floor, 10.8 (`urcapx-matrix.yml`, weekly + on URCap changes, not required; all
eight 10.8–10.14 pass 2026-09-30 with fallbacks for what 10.8–10.11 lack,
`urcap/README.md` §Tested; 10.6/10.7 dropped — their sims don't start on the runners). A
`urcapx-v<version>` tag publishes `dist/perceptronic-<version>.urcapx` as a GitHub
Release (`release-urcapx.yml`, `urcapx.py release-check`). Plain JavaScript, no npm: `urcap/urcapx.py package|install|list|delete`
(gzipped tar, manifest first; multipart to the Robot-API's `urcaps/v1/urcaps/`);
`make urcap-package|urcap-install|urcap-cockpit`. **`urcap/dist/*.urcapx` is the
committed download** and a test holds it byte-equal to a fresh (reproducible)
build — after editing the URCap run `make urcap-package` and commit `dist/`. The cockpit must be started
with `--cors <PolyScope origin>` (`PERCEPTRONICS_CORS`) and serves `GET
/api/color.png` for it. The installer posts `urcapxFile` to the **urservice**
endpoint System Manager itself uses (`/universal-robots/urservice/api/v1/urcaps`;
201 / 409 / DELETE, no Remote mode needed) — the SDK's Robot-API path answers
403 unless the robot is in Remote mode, from inside the container too. Built
against SDK 6.5.65 (the 10.13 pairing) — the worker speaks threads.js's
protocol directly. Verified 2026-09-26 in the 10.13.0 sim (headless Chromium):
node loads, feed + hover + click-to-segment work against the fake cockpit;
locate/move still need a cockpit with a robot link. **Release tracking:** the URCap targets
*one* PolyScope X release — the newest minor's newest patch — pinned in
`urcap/target.json` (notes URL, sim image by digest, SDK tag + contribution-api /
threads / manifest-spec versions); `urcap/track.py check|update|compat` and
`urcap/e2e.py` (boot that sim, install, load + click the node headless; ~2 min) are
what `.github/workflows/urcap-track.yml` runs weekly → PR or `urcap-attention` issue.
10.14.0 verified 2026-09-27 (the 10.14 Services-toggle bug doesn't touch URCaps), then
**held at 10.13** the same day to match the local sim image (`"hold": "10.13"` in
`target.json`: the track re-pins that minor's newest patch, never a newer minor). **Move (PolyScope)** re-expresses the
cockpit's flange target in *PolyScope's own* active TCP (`getKinematicInfo` DH +
`convertJointPositionsToTcpPose` at zero joints) before `getInverseKinematics(pose,
qNear)`, which takes no TCP — handing it the cockpit's `approach_pose` (a pose under the
*cockpit controller's* TCP) made every click unreachable in the sim. That IK never
answers an unsolvable pose (the node gives it 8 s). Cockpit field shorthand (`:7621`,
`host:port`) is completed to a URL; a bare `:7621` used to be fetched relative to
PolyScope's own page and its 404 misread as an outdated cockpit.

**The URCaps are "Perceptronic" (renamed from "RealSense Pilot" 2026-09-29, Nick: "not this
weird RealSense Pilot thing"), owned by Nick personally, namespaced by his domain advin.io
(Nick, 2026-09-30; the 09-29 builds were `com.nickarmenta.perceptronic` / `nickarmenta/perceptronic`,
PolyScope X 0.3.0 was released under that id):** PolyScope 5 bundle
`io.advin.perceptronic` (Java package the same, `Bundle-Vendor: Nick Armenta`,
`Bundle-Copyright: Copyright (c) 2026 Nick Armenta`, jar
`urcap/dist/perceptronic-ps5-<ver>.urcap`, and beside it `urmagic_perceptronic.sh` — the
stick's auto-install file filled in for that jar, written by `make urcap5-package`, held equal by
a test), PolyScope X `advin/perceptronic` from 0.4.0
(`urcap/perceptronic/`, element `advin-perceptronic`). A different bundle identity is a
different URCap to PolyScope: the old one must be removed on a robot that had it, and its
saved node data (cockpit address, pick areas) and programs' RealSense Pick nodes don't carry
over. Node titles: **Perceptronic** (installation), **Perceptronic Pick** (program). The mark
is `urcap/perceptronic.svg` — a P with a lens in its bowl; `Logo.java` draws the same glyph
with Java2D (toolbar badge, screen headers, a 38 % watermark bottom-right of every live
picture) and a test holds its `SVG` constant byte-equal to the file and to the PolyScope X
icon copy. **The P button (0.6.0, `ToolbarService` / `ToolbarContribution`):** a
`SwingToolbarService` — the Robotiq-style header button — that drops the live picture down
over any screen (420 px tall, polls only while open, reads the Installation node's saved
address through `ApplicationAPI.getInstallationNode`). The toolbar API has been in the URCap
API since 1.7.0 (PolyScope 5.4; checked in the jars), so no compat guard; the matrix's Felix
check requires the toolbar service registered (`ps5_matrix.NODE_SERVICES`). The feed loop
is one class, `FeedPoller` (no UR API, tested against the real cockpit), shared by the
Installation node and the popup. Not yet seen on a pendant: the button's icon size (30 px)
and popup height are guesses to check on the UR3e.

**PolyScope 5 (e-Series) URCap (`urcap/perceptronic-ps5/`, `README.md`):** the same node
as a Java 8 Swing **Installation node**, same layout and buttons (Open cockpit dropped —
no browser on the pendant); Move (PolyScope) hands `RobotMovement.requestUserToMoveRobot`
the controller's `joint_target`. Built by `urcap/urcap5.py` (`make urcap5-sdk` reads the
URCap API bundles of any `ursim_e-series` tag straight from Docker Hub's registry into
`target/urcap5-sdk/<tag>/` — no Docker; not on Maven Central, never committed; `make
urcap5-package` → `urcap/dist/*.urcap`, reproducible, with a sources digest a test checks).
**Backwards compatible from PolyScope 5.4** (`bundle.properties` `compat.floor`): compiled
against 5.4's API (`api-1.7.0.jar`, hence `urcap.api.version=1.7.0` — 5.10+ refuse a pom
naming a newer API than their newest `PackageDependencyProvider`); newer API only in a
`compat.since.<ver>` class loaded by name behind a check (`TeachPosition2`:
`RobotPositionCallback2`, 5.8+; its package imported `resolution:=optional`). The script
adapts to the PolyScope it runs on: no `get_inverse_kin_has_solution` before 5.9.4, and
lists keep their first size up to 5.14 (never reassign a read to a different count). PolyScope 5's loader needs `URCapCompatibility-CB3`
/ `-eSeries` in the manifest and reads the API version from an embedded
`META-INF/maven/**/pom.xml` (`com.ur.urcap:api` dependency). Its **installer** (Settings →
URCaps → +) also demands `Bundle-Category: URCap` and the manifest among the first two zip
entries (`URCapsServiceImpl.isValidFile`); the VM's `install-urcap` copies the jar into the
bundle dir and skips those checks, so a VM load does not prove the pendant will install it
(missed once, 2026-09-27). Check with PolyScope's own `URCapFileValidationHelper` from the
image's `/ursim/GUI/bundle`. Installed and rendered on the UR3e pendant 2026-09-27 (0.2.0).

**Perceptronic Pick program node (same bundle, 0.4.0; `PickScript.java`, `perceptronics/picknode.py`):**
Program tab → URCaps → Perceptronic Pick. Its URScript runs in the operator's program (Local mode,
no Primary) and talks to a pick server over a plain socket (`:7622`; `FIND`/`LOOK`/`REFINE`, and
`LOG` lines that are never answered — a stray reply would be read as the next answer). No survey
position is needed: the first look is from where the arm is, the closer look from halfway toward
the block with it centred (never nearer than 0.25 m). Every stage is a `textmsg` **and** a `LOG`
line; a failed pick raises a blocking popup naming the reason. The cockpit serves the pick server
itself; a cockpit already running older code gets it from **`perceptronics pick-server`** — a sidecar
on `:7631` (point the node's cockpit URL there) that answers from the cockpit's frames, forwards
every other route, and writes the whole trace to `captures/pick-server.log` or, when that's
root-owned, `~/Library/Logs/perceptronics/` (also `GET /api/pick/log`). **Kill the sidecar before
relaunching the cockpit** (`pkill -f "pick-server --bind"`): both bind `:7622`, and the cockpit
just warns and runs without its pick server. The 0.3.0 script has not yet run on a controller.
**Part size (0.4.0, `perceptronics/partspec.py`):** the node's Length × Width [× Height] ± tolerance
(as the part lies; default ±25 %, never under ±5 mm) rides on FIND/REFINE as `part=60x40x30 tol=25`
and replaces `detect_blocks`' fixed foam-block gate (≤ 70 × 60 mm); height is measured against the
non-white depth in a ring around the blob (unseen → not checked). Status −7 = white things in view,
none that size; the teach screen draws the rejects grey with why. Still colour-segmented: the part
must read white. Verified on synthetic scenes only.

**The pick kit, 0.5.0 (`docs/pick-kit.md`, 2026-09-28).** The product line for a PolyScope 5
cell: `hardware/BOM.md` (sourced, dated prices), the bracket, `deploy/pi/` (a Pi-class arm64
camera computer on minimal Debian: librealsense built pinned, the cockpit as the
`perceptronics-cockpit` service on :7621/:7622, nftables; `scripts/deploy-pi.sh`, the
`deploy-pick-pc` skill), and the URCap 0.5.0. **Detection is by volume**
(`perceptronics/volume.py`, depth only, no colour): what stands the part's height above the
surface with a top face its length × width; the surface is a taught pick area (three
fingertip touches in the Installation node, nudged ≤ 15 mm to the live table) or the table
found live; each part's min-area rectangle gives the axes, the fingers close across the short
side. `order_parts` numbers parts in the **picture's** directions (front = bottom of the
picture); with the grip check on, `volume.clear_the_way` then reorders them so a part pinned
only by other parts (one in its finger zone) is picked after them instead of being turned
away, and the picture's order only breaks ties (Nick, 2026-10-02: "pick the parts that make
other parts easier to pick"). Reach = base outer radius + 150 mm .. rated reach − 150 mm (`Reach.for_model`).
**Pick server protocol 2** (`picknode.parse_options`): every request carries
`part= tol= order= grip= stroke= [reach=] [plane= area=] node= loc= locs= proto=2`, answers
are 16 numbers; `NEXT` serves the per-node queue (the next part already seen: no trip to the
picture point), `FIND` refills it. The Java `PickScript.tokens` writes the same line, and
`tests/test_urcap5_pick.py` reads the Java's tokens back with the Python parser — change
one, the test tells you about the other. The node's screens (`PickScreen`, `LocationsScreen`,
`LiveView`, `Diagrams`, `Ui`) are pure Swing: render them off-pendant with a harness (the
README's `screens/` are `urcap/preview5.py --screens urcap/perceptronic-ps5/screens`). The teach screen asks `GET /api/pick/scene?opts=<tokens>`.
**Every PolyScope 5 minor, 5.4-5.26** (the newest image of each; all 23 green, run
36521908487) runs every URCap change in CI (`urcap5-matrix.yml`, versions from
`ps5_matrix.py list`; `docker-compose.ps5-matrix.yml` is generated by `ps5_matrix.py
compose`, ports 20000 + 100 × minor + the usual port's last two digits; `check-tags` only
ever adds): `urcap5.py check` against the image's own API jars, Felix shell (port 6666 in
the container) proves the bundle Active — polyscope.log never says so — then
`urcap/pick5_e2e.py` runs the node's own script, two picks, the second from the queue, plus
a compile probe (Robotiq + popup inside `if False:`) and a socket-timeout probe. A `build`
job rebuilds the committed jar against 5.4's jars with JDK 21 and `urcap5.py compare`s it
(entries, non-class bytes, class members — JDK-independent; the committed jar is JDK 25's).
`check-tags` also compares each image's `VERSION` with `IMAGE_VERSIONS` (bare tags re-pushed).

**3D Pick (URCap 0.7.0 on PolyScope 5, 0.5.0 on PolyScope X; Nick's feedback 2026-09-30).** The program
node is **"3D Pick"** (service id `PerceptronicPick` / tag `advin-perceptronic-pick` kept): one move
sequence with **no children** that starts with the survey and ends with the gripper clamped on a
part — no lift, no "After picture N" node (`PickRoutine*` and `after-node.worker.js` are gone). What
supersedes the paragraphs above: **(1) nothing scrolls** — picture points are a fixed 2 × 6 grid of
numbered buttons with the selected one's actions under it, only the selected pick area is open, and
`test_everything_on_the_nodes_screen_fits_without_scrolling` lays each screen out and fails on a
clipped control or a scroll pane. **(2) Options = two tabs**, Part (Box / Cylinder, size, tolerance,
**Grip check**, off by default) and Approach (approach, grip depth, **Closer look**); speeds and the
gripper have no settings (`PickScript` constants: 60 %, Robotiq via its URCap socket; `digital` is the
simulator's stand-in). **(3) The picture carries only near misses** (`Scene.nearMisses`, the server's
`near` flag: a little off the size, or the right size and out of reach) and a **Picture / Depth**
toggle top right inside the frame (`GET /api/depth.png`, half-size, ramp = the frame's 2–98 %).
**(4) Reach is the arm's kinematics**: the node sends `arm=<model>` (never `reach=`), and
`perceptronics/armik.py` — closed-form UR IK over `armfk.DH`, every answer checked against the FK —
says whether approach + grip have a joint solution at lean 0 / 12 / 24°; only the base's keep-out
(radius + 150 mm) is kept; an arm not in the table is left to the controller. The Installation's reach
margins are gone; its map shows the rated reach, named. **(5) The closer look** puts the camera
0.30 m from the part with the part 12° off the optical axis on the side away from the gripper
(`picknode.look_pose`, `gripper_bearing`: on the UR3e the open fingertip is 9° off-axis, 0.15 m out —
a centred part was behind it); with no look pose the node measures again from where it is; with the
closer look off every part is measured from its picture point (no queue). **(6) No camera** → the
picture's place says what to check (cables, IP address, firewall; USB on a 503) and the exception goes
to the log (`Log.java` / the browser console). **(7) `shape=cyl`**: an upright cylinder by its
diameter, wrist not turned; `gripcheck=0` drops the stroke and finger-room checks. The two nodes write
the same URScript — `test_the_script_is_the_polyscope_5_nodes_line_for_line` (apart from `global x =`).
`preview5.py --screens DIR` re-renders the README's pictures. **Not run on a robot**; the PolyScope X
dialog was clicked through in the 10.13 sim (`urcap/e2e.py`: depth view, both option tabs fit).

**3D Pick drives no gripper (URCap 0.8.0 / PolyScope X 0.6.0; Nick 2026-10-01 — this corrects the
paragraph above where it says the node clamps).** "The node should not control the gripper
whatsoever because the user will open the gripper before the node and close it after": the script has
no Robotiq socket, no digital output, no held check — it ends with the fingertips at the grip
(`rs_pick_found = True` there) and `test_the_node_does_not_touch_the_gripper` holds both nodes to it.
With no gripper there is no stroke: the grip check is **on by default** and is the operator's number,
**Finger room** (Approach tab, default 20 mm) → `gripcheck=1 room=20`: that much clear space on each
of the two sides the fingers come down on, from the part's edge (`pickplan.clearance(room_m=)`), and
no "wider than the open gripper" test. **Grip across the long side** (Approach tab, boxes only) →
`across=long`: the wrist turns 90° and the room is checked on those sides. Cylinders stand on a flat
end (decided; a lying one rolls). **The picture: pickable parts green with their pick-order number,
near misses yellow with why** (still nothing for what is nothing like the part). The closer look
assumes fingers open 50 mm (`LOOK_STROKE_MM`) when it keeps the part clear of them.

**Settled 2026-10-01 (Nick):** green overlays for pickable parts and yellow for *marginal* ones (the
server's `near` flag) is the intended picture — nothing is drawn for what is nothing like the part;
the 3D Pick node's picture carries no watermark; the **UR7e is the UR5e's arm and the UR12e the
UR10e's** (`armfk.DH` aliases, so `armik` judges them). The UR30 and UR15 rows are UR's published table (2026-10-02); an arm not in `armfk.DH` is left to the controller.

**Three traps from the 3D Pick sessions.** (1) `urcap/pick5_e2e.py` compiles the test harness in
`tests/test_urcap5.py` (`HARNESS`): anything the harness starts to use must be in the source list
`generate()` copies (`PURE_JAVA` + `SCREEN_JAVA`) — a mismatch failed all 23 controller jobs in a
minute; `tests/test_pick5_e2e.py` now builds the e2e's scripts and replays its session. (2) The
committed packages in `urcap/dist/` conflict on a rebase whenever *anything packaged* changes on `dev`
— the LICENSE included — and a conflicting PR shows **no checks at all**: rebase, delete the
conflicted files, `make urcap-package urcap5-package`, push. (3) Playwright is not in `.venv` (uv is
gone from the repo): `uv run --with playwright==1.63.0 python urcap/e2e.py --image
universalrobots/ursim_polyscopex:10.13.0` still works where uv is installed, but leaves a stray
`uv.lock` — delete it before committing.

**Monocular scan** (`perceptronics scan`, `docs/mono-scan.md`) was removed on
2026-09-25 (branch refactor/prune-2026-09-25); it lives in git history before
that commit if the idea comes back.

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
| 22    | SSH (real robots) | bi        | Debian sshd on the controller. The **only** way to drop files in `/programs/` on a real e-Series (no Dashboard upload, no SMB/NFS by default). Not exposed by URSim — use `docker cp` instead. |

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
  only Dashboard `play` needed Remote; Primary URScript did not.) **Primary
  URScript is gated on Remote too on PolyScope X** (verified 2026-09-04 on the
  10.13.0 sim): in Local the port accepts and broadcasts state but scripts are
  silently ignored — `textmsg` never surfaces, `run_and_capture` returns `[]`,
  `move-*` returns `landed: null`. Once Remote, `textmsg`, confirmed moves and
  `get_flange_pose` all work over `:31001`.
- **UI passwords** (UR's published defaults, first use forces a change): admin
  `easybot` (gates Settings → Security → Services), operational mode `operator`
  (gates Manual/Automatic + Remote). **Stay on the 10.13.0 image** — see the
  comment in `docker-compose.yml`; on 10.14 the Services toggles never stick.
- **Runs natively on Apple Silicon** (arm64 image; `HOST_ARCH=arm64 make
  simx-up`), which makes it the sim to use on this Mac now that the e-Series
  image can't boot under Rosetta here (gotchas table).

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

### Authoring a *native node tree* (not a one-line script wrapper)

**Authoring conventions** (functional waypoint names, one Move node per motion
type, blend every non-stopping waypoint) live in
`docs/program-authoring-best-practices.md` — follow them when generating
programs.

`urp_convert`'s `script_to_urp` wraps a whole `.script` in a single
`<Script type="File">` node — PolyScope loads it, but the operator sees one
opaque "Script" line. When you want the program to look like real,
point-and-click UR nodes (MoveJ→Waypoint, If, Loop, Folder…) the `.urp` must
*be* that typed node tree. `urctl/urp_builder.py` (`UrpProgram` + `Waypoint`)
emits it. `.script → rich .urp` reconstruction is still impossible (URScript is
Turing-complete; the tree is gone once it's flat) — but **authoring** the tree
when *we* generate the program is straightforward.

Two findings, both verified by generating a `.urp` and watching `load` succeed
against the running sim (`programs/NodeTreeDemo/build.py` is a worked example;
`tests/test_urp_builder.py` + `TestUrpBuilderLoading` lock it in):

- **The kinematics envelope is load-critical for `<Script>` nodes.** A program
  with real (UR10e) `<kinematics>`/`jointChecksum="-1,…"` loads fine for
  Move/Waypoint nodes, but `ScriptNodeConversionStrategy` rejects the whole
  program ("not a valid file version"). The all-zero `NOT_LINEARIZED` envelope
  (identical to `urp_convert`'s) loads *every* node type, so the builder emits
  that unconditionally. Joint-space waypoints don't need calibration anyway.
- **Verified node set:** `Comment`, `Folder`, `MoveJ`/`MoveL` + joint-space
  `Waypoint`, `If` (raw expr or digital-in), `Loop` (counting), `Set` (digital
  out), `Popup`, and the hybrid `Script type="File"` (multi-line helper `def`s)
  + `Script type="Line"` (a call shown as one script line). `Wait` and
  `Script type="Code"` were tried and **rejected** by their conversion
  strategies — they're intentionally absent. Add a node type only after
  teaching it once in PolyScope (`load` the result over Dashboard to confirm)
  and diffing the save; do **not** ship guessed node XML.

The "load key functions and use them" pattern: put helper `def`s in a
`script_file(...)` node and call them from `script_line(...)` nodes, so the
visible flow reads as named operations (`close_gripper()`) instead of I/O
plumbing. `make regen-urps` runs any `programs/*/build.py` to refresh its `.urp`
(falling back to the `.script`→`.urp` converter for script-based samples).

### Guided build with on-pendant confirmation

`urctl/guided.py` (`GuidedSession`) builds a node-tree program **interactively**:
the operator issues a step, the robot raises a **Yes/No dialog on its own
pendant** describing what it's about to do, and on Yes the step *executes live
and is recorded* into the growing `.urp`. The console front end is
`urctl guided <name> --save <path>` (a REPL: `movej`, `movetcp [rel]`, `out`,
`comment`, `summary`, `save`, `quit`).

Two authoring conventions are baked in (see
`docs/program-authoring-best-practices.md`): every recorded step is preceded by a
**Comment node** built from its `; description` (so always pass one), and
waypoints are **named by function** (CamelCased description), reusing an earlier
name when a step returns to the same pose. Because each step gets its own
comment, guided moves are intentionally not grouped into shared Move nodes.
Passing **`--freedrive`** makes each move drop into freedrive after executing so
the operator can hand-guide the exact pose and tap Yes (No/Cancel skips)
(`Robot.reteach_in_freedrive`); the achieved pose is recorded and later relative
steps build off it.

**Multi-point inspection app** (`urctl inspect <name> --save <path> [--live]`,
built on `guided.run_inspection`): a pendant-led loop for "walk the robot to each
spot and snap a picture." It embeds a camera-trigger subprogram once
(`guided.CAMERA_TRIGGER_DEF` — a digital-out pulse placeholder; swap for the real
trigger via `--call` or by editing the def), then repeatedly drops into freedrive
so the operator hand-guides to a point and taps **Yes** to capture (records a
MoveJ waypoint + a `script_line` call to the subprogram) or **No/Cancel** to
finish. This is the canonical use of the Yes/No/Cancel return: Cancel ends the
loop. No camera calibration needed — points are joint-space teach poses.

The pendant gate is `Robot.confirm_on_pendant(prompt, timeout=…)`, built on
`request_boolean_from_primary_client` — which **PolyScope's own UI answers**
(the operator taps the choice on the teach pendant; see the dialect note below
on `request_*_from_primary_client`). Hard-won details, all verified against the
sim:

- **Confirm, then act — two round-trips, not one.** The confirm is separate from
  the motion so the move still flows through the `SafetyEnvelope`; inlining the
  action into the confirm script would bypass validation.
- **A timed-out confirm leaves a dialog up on the controller** that blocks the
  next step. `confirm_on_pendant` detects no-answer (`confirmed=None`) and
  issues `stop` + `close popup` to clear it. On Yes/No the request program ends
  itself, so cleanup only runs on timeout.
- **Only approved *and* executed steps are recorded.** A reject, a safety-refused
  move, or a move that doesn't confirm completion is tallied in `session.steps`
  but never added to the program.
- **Moves are recorded as joint-space waypoints at the achieved pose** (read back
  after the move), so a Cartesian step replays to the same physical spot without
  calibration — `move_tcp` becomes a `MoveL` over a joint `Waypoint`.

**Live tree growth (`--live`).** By default the node tree is built host-side and
only appears in PolyScope's Program tab once the `.urp` is loaded. To make the
tree grow *node-by-node as you confirm*, pass `urctl guided … --live`: after
every recorded step the program is saved, placed on the controller, and
reloaded. An e-Series controller **only refreshes its tree by loading a file**,
so there's no avoiding a brief reload blink per step. The env-specific bit —
getting the file into the controller's program dir — is a *placer*:
`docker_placer(container)` (`docker cp` into this repo's URSim; the default,
`--live-container`) or `local_dir_placer(dir)` (a host-reachable program dir, a
mounted real-robot share; `--live-program-dir`). `GuidedSession(on_record=…)` is
the generic post-record hook; `LiveReloader` is the supplied save→place→load
implementation (reload failures are swallowed + logged — the in-memory program
and the final save are unaffected).

The one thing that can't be CI-verified is a human actually tapping Yes/No on
the pendant; everything up to and after that (dialog raised, blocks, readback,
timeout cleanup, record-on-approve, live publish+reload) is covered by tests.

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
- **Absolute moves are reach-checked by the controller's own IK.** Before an
  absolute `move_tcp` / `move_tcp_path`, `Robot.inverse_kin` asks the controller
  (`get_inverse_kin_has_solution` + `get_inverse_kin`, one Primary program, same
  `tcp=` as the move; no motion, so it answers in Local on e-Series). Its verdict
  replaces the datasheet sphere both ways: a no is an `ik_reach` violation (nothing
  sent), a yes passes a pose the sphere would reject. The sphere — sized by
  `SafetyEnvelope.for_model` (UR3e 0.5 m, UR20 1.75 m, unknown → UR10 1.3 m;
  `UR_MAX_REACH_M` overrides) from the **base origin** — is only the fallback when
  the controller gives no answer (dry-run, Primary busy, PolyScope X in Local). On
  the UR3e cell (2026-09-27, parts 0.27 m below the base) it rejected reachable
  targets — flanges 0.505–0.533 m out that the controller solves — and a sphere
  cannot see an in-radius pose the arm can't reach either. `locate` reports `reachable`, `reach_check` (`controller_ik`/`sphere`) and
  the controller's `joint_target`; check `reachable` before offering a Move.
- **One program at a time.** A new submission on 30001 replaces whatever is
  running. `Robot.move_tcp_path` runs a whole multi-leg Cartesian path
  (over → down → dwell → up → back) as one program on one connection, and
  `PrimaryClient` refuses a concurrent submission (`PrimaryBusyError`) so a
  state poll or a Locate can't kill it. `tcp=` on `move_tcp`/`move_tcp_path`
  runs `set_tcp` in the same program (`[0]*6` = the flange).
- **Relative moves are base-frame.** `move_tcp(..., relative=True)` adds the
  delta to the live TCP via `pose_add(get_actual_tcp_pose(), p[...])`, so the
  XYZ delta is in the **base** frame (use `pose_trans` if you ever want the
  tool frame). The safety envelope caps a single relative step (default 1.0 m)
  to catch unit mistakes (inches/mm entered as metres).

## Cartesian moves and singularities (`movel` protective stops, C154A0)

A `movel` from a **near-singular pose protective-stops the robot** (controller
error `C154A0`), even for a tiny step — the inverse kinematics blows up joint
velocities at the singularity. Two poses bite constantly:

- **URSim's default startup pose** (TCP straight up at z≈1.48 m — fully
  extended).
- **`HOME_JOINTS` = `[0, -90, 0, -90, 0, 0]`** — elbow joint = 0 ⇒ the upper
  arm and forearm are colinear ⇒ **elbow singularity**. `move_home` lands here,
  so "home then nudge in Cartesian" is exactly the trap.

`movej` is immune (it interpolates in joint space), so the fix is to **`movej`
to a dexterous, elbow-bent pose before any `movel`**. The integration tests use
`READY_JOINTS = [0, -1.0, 1.2, -1.8, -1.5708, 0]` for this (see
`tests/test_integration_ursim.py::_running_robot`).

Diagnosing it: a waited `move_tcp`/`move_joints` that hits this returns
`ok=False, landed=null` and now also `result.protective_stop=true` (the facade
checks `safetymode` when a move fails to confirm). The raw reason is in the
container's URControl log, not `polyscope.log`:

```bash
sudo docker exec perceptronics-ursim-1 grep -i 'protective\|C154' /ursim/URControl.log | tail
```

**`robot_mode` stays `RUNNING` through a protective stop** — only `safety_mode`
flips to `PROTECTIVE_STOP`. So any "is the robot ready?" check that looks at
`robot_mode` alone is wrong; check `safety_mode` too, and `bring_up()` clears a
latched protective stop (it calls `unlock protective stop`). A test/helper that
gates recovery on `robot_mode` will let one protective stop cascade into every
later motion test.

## How fast can you actually drive it (control throughput)

Measured on the emulated sim, a single **confirmed** move (`move_tcp` /
`move_joints`, `wait=True`) has a fixed floor of **~1–1.5 s** regardless of the
move distance/velocity. It breaks down as:

- **~0.5 s** — the safety pre-check does one Dashboard `robotmode` round-trip
  (connect → send → read-until-close). This is per move.
- **~0.7 s** — the Primary broadcast round-trip: send the program, then wait for
  the `urctl/move/done=` `textmsg` to surface in the ~10 Hz state stream.
- plus the actual motion time.

Two hard limits when you try to go faster by firing moves back-to-back:

- **Rapid per-move reconnects wedge the controller.** Each `move_*` opens a
  fresh Primary (30001) socket. After **~10** rapid connect/run/disconnect
  cycles URControl's Primary interface chokes (`Socket::OperatorOverload` /
  `MultiTCPSender sendall failed` in `URControl.log`) and the robot can drop to
  **POWER_OFF**. `bring_up()` recovers it (Dashboard still works); the Primary
  broadcast itself only fully recovers on a container restart.
- **The safety envelope caps joint speed at ~2.09 rad/s** (`120°/s`,
  `DEFAULT_MAX_JOINT_SPEED`) and TCP speed at 1.0 m/s. A move above the cap is
  rejected up front (`ok=False` with a `velocity` violation, nothing sent) — it
  looks like an instant failure but it's the envelope doing its job. Keep test
  velocities ≤ ~2.0 rad/s.

**To move a lot, stream a trajectory, don't loop single moves.**
`Robot.move_trajectory(waypoints, ...)` (tool `ur_move_trajectory`) sends a whole
joint-space path as one `def` over a single connection: it pays the handshake
**once**, never reconnects mid-path, validates every waypoint up front, and can
`blend_radius`-smooth segments (dropped on the last waypoint — a blend on the
final `movej` errors; see the movej blend-radius pitfall in ur-program-authoring).
This is both faster and the only way that doesn't risk wedging the controller.

## Driving a real e-Series (vs URSim): which path to use

Three divergences cost real time on the physical UR10 at `192.168.1.50` —
none reproduce on URSim. The decision tree:

| You want to…                                            | URSim                                         | Real e-Series                                                                                   |
| ------------------------------------------------------- | --------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| Read state, send `popup`/`addToLog`/`load`/`stop`        | Always works                                  | Always works (Dashboard does not need Remote)                                                   |
| Move via Primary URScript (`move-joints`, `move-tcp`, `move_trajectory`, `run-script` with motion) | Works in Local — verified                     | **Requires Remote control mode.** In Local, `urctl move-*` and `ur_move_trajectory` silently return `ok:false`, `landed:null`, with **no** safety violation. `run-script` of a textmsg-only script still returns `ok:true` and executes, so it's the motion specifically that is gated. There is no network endpoint to switch Local→Remote — flip the indicator in the top-right corner of the pendant. |
| Play a loaded `.urp` via Dashboard `play`               | Requires Remote                               | Requires Remote                                                                                  |
| Drop a `.urp` (or `.installation`) in `/programs/`       | `docker cp` into the URSim container          | **SCP over SSH** (port 22). `/programs` is the PolyScope-visible path. `default.installation` typically already exists, so no installation upload is needed when `urp_builder`'s default matches. |
| Build a `.urp` of native Move/Waypoint nodes (`UrpProgram.movej(Waypoint(...))`) | Plays cleanly — `kinematicsFlags="4"` + `positionType="CartesianPose"` with the zero kinematics envelope satisfies URSim's IK | **Fails with "Robot cannot reach the required pose"** at runtime if the customer's active tool has a non-zero TCP offset. PolyScope's MoveJ does FK→IK through the **active TCP**, and a long gripper (the installations on this robot have offsets up to ~0.15 m on `tcpOffset`) puts the IK target outside reach for joint configurations that raw URScript `movej(joints)` accepts. URScript's `movej(joints,...)` does NOT do this round-trip — it just runs the joints. So a `.urp` whose motion comes from a `<Script>` node calling `movej` is the reliable path. Switching `kinematicsFlags` to `"6"` (what PolyScope itself emits) alone does **not** fix it. |

The first two divergences are gates (state polling and orchestration are
fine in Local; motion isn't). The third is structural: the same `.urp`
that loads and runs on URSim can stall on a real robot if the customer's
active tool has a non-zero TCP offset, even though the joint targets are
reachable. **Programs you author for a real-robot cell should emit motion
as `script_file`/`script_line` nodes**, not native `MoveJ`/`Waypoint`
nodes. Native nodes are still right for URSim, demos, and any cell whose
active TCP is at the flange — but you can't tell from the `.urp` alone
which side of that line you're on.

### File placement on a real robot

```bash
export SSHPASS='<password>'                       # do not pass -p (visible in ps)
sshpass -e scp programs/Dance/Dance.urp root@192.168.1.50:/programs/Dance.urp
python3 -m urctl --host 192.168.1.50 load Dance
python3 -m urctl --host 192.168.1.50 play
```

Verified: real e-Series ships Debian + sshd on `:22`, root login enabled,
`/programs/` writable. Pre-existing `/programs/default.installation` pairs
with `UrpProgram(..., installation="default")` (the builder default). Long
term, `ssh-copy-id root@<ip>` and drop `sshpass`. There is no Dashboard
upload command and no SMB/NFS exposed — SSH is the only file-placement
path.

For step-by-step live program authoring on a real robot, `urctl guided
--live` accepts **`--live-scp root@<ip>`** (in addition to the URSim
`--live-container` and mounted-share `--live-program-dir`). Each
approved step is saved → scp'd into `/programs/` → reloaded over
Dashboard, so the PolyScope tree grows node-by-node on the pendant. Set
`SSHPASS` once per shell or enroll a pubkey first — the placer does not
prompt for a password. The matching `<installation>.installation` is
expected to already exist on the controller (the placer does not push one
— a real-cell installation is configuration the operator owns, not
something to overwrite from the dev box).

### Build motion programs with script nodes (for real-robot cells)

```python
from urctl.urp_builder import UrpProgram

prog = UrpProgram("Dance", run_only_once=False)
prog.comment("Looping dance — raw movej() bypasses MoveJ-node IK validation")
prog.script_file("dance", """def dance():
  movej([-0.7427, -2.1034, -2.5358, -0.0718, 1.5714, 2.9782], a=1.2, v=1.0, r=0.03)
  ...
  movej(home, a=1.2, v=1.0)   # final waypoint — no r= (see movej blend pitfall)
  sync()
end
""")
prog.script_line("dance()")
prog.save("programs/Dance/Dance.urp")
```

The operator sees one "Script: dance helper" definition + one "Script:
dance()" call line in the Program tree — less editable than native
Move/Waypoint nodes, but it's the difference between a program that
runs and one that pops "cannot reach the required pose" mid-cycle.
`programs/Dance/Dance.urp` is the worked example.

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
| `move-tcp` returns `ok:false`, `landed:null`, `protective_stop:true`; every later move also fails | A `movel` tripped a protective stop (singularity / C154A0), and `robot_mode` stays `RUNNING` so naive "ready?" checks miss it | `movej` to a dexterous pose before `movel`; `bring_up()` (or `unlock protective stop`) to clear the latch. See "Cartesian moves and singularities". |
| RTDE / Secondary / RT reads refused on `:30004`/`:30002`/`:30003` | Those ports weren't published by docker-compose | The e-Series `ports:` list now publishes `30002-30004`, `30020`, `502`; recreate the container (`docker compose up -d`) after editing it |
| Moves start failing fast (`ok:false`, no `protective_stop`) and the robot ends `POWER_OFF` after a burst of moves | ~10+ rapid per-move Primary reconnects wedged URControl (`Socket::OperatorOverload`) | Don't loop single moves — use `move_trajectory` (one connection). `bring_up()` to recover. See "How fast can you actually drive it". |
| A move returns `ok:false` instantly with a `velocity`/`tcp_velocity` violation | Speed exceeds the safety envelope cap (joints ~2.09 rad/s, TCP 1.0 m/s) | Lower the velocity, or raise the cap on a custom `SafetyEnvelope` if you really mean it |
| **On a real e-Series**, `urctl move-joints` / `move-tcp` / `ur_move_trajectory` returns `ok:false`, `landed:null`, **no safety violation** and joints don't change. `urctl run-script 'textmsg(...)'` still returns `ok:true` from the same robot | Robot is in **Local** control mode. Real e-Series gates Primary URScript *motion* (not state, not textmsg, not Dashboard) on Remote. URSim doesn't, so this only bites on hardware. | Flip the top-right pendant indicator Local→Remote (no network endpoint exists). Add a precondition: read `urctl state`, refuse motion unless `control_mode == "REMOTE"`. See "Driving a real e-Series". |
| **On a real e-Series**, a `.urp` built with `UrpProgram.movej(Waypoint(...))` loads, plays the first few waypoints, then pops **"Robot cannot reach the required pose"** — yet `ur_move_trajectory` with the *same joint targets* runs clean | PolyScope's MoveJ node does FK→IK through the **active tool TCP** (`useActiveTCP="true"`, `positionType="CartesianPose"`). With a non-flange TCP (gripper, sensor — common, up to ~0.15 m on this robot), the FK pose can be outside reach even when the joints themselves are valid. Raw URScript `movej(joints)` skips the round-trip. Patching `kinematicsFlags="4"`→`"6"` (what PolyScope itself emits) does not fix it. | Don't use native Move/Waypoint nodes for real-robot cells with a non-flange TCP — emit motion via `prog.script_file("helpers", "def f(): movej(...) end")` + `prog.script_line("f()")`. See "Build motion programs with script nodes". |
| PolyScope GUI (noVNC on :6080) stuck on "Please wait…" | Cosmetic — the Java/AWT front-end is slow under emulation; the controller and all network interfaces (Dashboard/Primary/RTDE) are fine | Ignore it; the headless `urctl` path doesn't use the GUI. Confirm with `urctl state` (RUNNING/NORMAL). `docker compose restart ursim` only if you actually need the GUI |
| `docker ps` shows the container `(unhealthy)` but everything works | The healthcheck shelled out to `nc`, which **isn't installed in the URSim image**, so it always failed (not the controller) | Fixed: the healthcheck now uses `python3` (present in the image). Benign regardless — verify the controller with `urctl state` |
| `make sim-up` fails with `port 5900 … address already in use` on a Mac | macOS **Screen Sharing** serves VNC on 5900 (`nc localhost 5900` answers `RFB …`) | Use noVNC on 6080 instead; publish the container's 5900 elsewhere (`15900:5900`) or turn Screen Sharing off. Compose < 2.24 has no `!override` for `ports:`, so edit the mapping or `docker run` the service |
| URSim container is `Up` but 29999 refuses / resets and `docker logs` shows `Trace/breakpoint trap   Xvfb` | Docker Desktop is emulating amd64 with **Rosetta**; Xvfb crashes under it, PolyScope (which serves the Dashboard) never starts, and URControl stops listening within minutes. Seen 2026-09-04 on the Mac Studio; the `Exited (101)` containers from weeks earlier were the same | **Docker Desktop's emulators can't run the e-Series sim on the Mac Studio**: with Rosetta off (QEMU user-mode) Xvfb survives but URControl dies (TODO.md, 2026-09-04; re-confirmed 2026-09-27). The image is amd64-only (every tag). `scripts/ursim-e-vm.sh up` runs it in a full x86_64 QEMU VM instead: URControl, Dashboard, Primary/RTDE and the URCap loader work (8 min to Dashboard), but PolyScope's JVM crashes in JIT code there (SIGILL/SIGSEGV, `hs_err_pid*.log`) — fine for "does the URCap load", not for clicking through PolyScope. For that: an amd64 host, CI, or the real UR3e |
| Every cockpit click on the UR3e cell reads **OUT OF REACH** although the arm reaches the parts | The reach check was the datasheet radius (0.5 m) from the base **origin**; the parts sit 0.27 m below the base | Fixed 2026-09-27: `locate` and every absolute move ask the controller's IK (`reach_check: controller_ik`); the sphere is only the no-answer fallback. If you still see `reach_check: sphere`, Primary isn't answering (PolyScope X in Local, Primary disabled) |
| A cockpit Move with the Hand-E on drives the fingers into the part | The standoff was measured from the **flange** (`PERCEPTRONICS_APPROACH_REFERENCE=flange`, 75 mm) — the fingertips are 163 mm past it | Fixed 2026-09-27: approach by the fingertips (the default); check `perceptronics doctor`'s `approach` line shows the tool length you measured |
| The Perceptronic Pick node (or any FIND) seems **hung**: nothing moves, the pick-server log shows `FIND: no fresh camera frame` (status −4) over and over | The D435 dropped out of the cockpit: its frames freeze at one `seq` (`/api/info` now says `stalled: true`, `fps` 0 — before 2026-09-30 `fps` kept reading ~30), and `/api/info`'s `last_error` says `Frame didn't arrive within 5000` (the macOS USB-claim race). Each FIND waits for a frame newer than the request, gets none, answers −4, and a looping program asks again | Re-plug the camera; the lean open re-opens it by itself. The sidecar's log now says `cockpit frames stalled at seq N` with the camera's error, and 0.3.0 pops up the reason instead of looping silently |
| The cockpit runs the **old hand-eye** although `perceptronics/cells/ur3.env` has the new solve (`/api/info` → `robot.handeye.flange_to_depth_pose` ≠ the cell file's value; seen 2026-09-27) | `apply_cell` only fills keys the environment doesn't already have, so a `PERCEPTRONICS_T_FLANGE_CAMERA` already in the cockpit's environment wins. `handeye.source` reads `env:…` either way, so it can't tell you which one won | Launch with `sudo env -u PERCEPTRONICS_T_FLANGE_CAMERA python3 -m perceptronics --cell ur3 gui …`; after every launch compare `flange_to_depth_pose` in `/api/info` with the cell file |
| RealSense colour panel black, depth fine, RGB options at factory | depth and colour streaming at **different sizes** on the D435 | keep both at 848×480 (the default); `docs/realsense.md` §Depth quality |
| Cockpit shows nothing on a **USB 2** link; log says `Couldn't resolve requests` then `RS2_USB_STATUS_ACCESS` on every retry | USB 2 lists **no 848×480 colour** (and 848×480 depth only at 10/6 Hz), so the default pair can't start; each failed open re-runs the macOS UVC race | Fixed: `open()` enumerates the camera's profiles (`Api.stream_modes`) and `negotiate_mode` picks the fastest same-size pair it offers (640×480 @ 15 on the D435) — no flags needed; re-plug once to clear the race. `docs/realsense.md` §Troubleshooting |
| RealSense open fails on the Mac with `RS2_USB_STATUS_ACCESS` / `set_xu … timed out` / no frame, and the process **segfaults** after `usb device disconnected` | The Mac is a desktop now: libusb's claim re-enumerates the device and every camera-aware app (Spotify won it on 2026-09-24; browsers; Apple's UVCAssistant) races for it; a disconnect mid-open crashes librealsense 2.58.4 in libusb | Don't chase it. On this Mac every libusb handle open resets the camera (root-only kernel-driver detach = re-enumerate with capture) and Apple's `UVCAssistant` re-claims it each time: 43 resets in 40 s and a stream that dies after 2 frames, with nothing else on the bus (2026-09-25, webcams and Spotify gone). The Mac-as-desktop is not a D435 host; use the Windows laptop under WSL2 (verified 09-23) or the Jetson. the cockpit under `--rs-lean`. `docs/realsense.md` §Troubleshooting |
| RealSense first open of a process never delivers a frame, re-opens work | sensor options written between pipeline start and the first frameset | write them on the first `read()` (`DepthTuning` does); never at open |
| **On a real e-Series**, `move-tcp` / cockpit **Move** to a target the arm can't reach returns `ok:false`, `landed:null`, no violation — and the arm **stretches to a straight elbow** chasing it (UR3e, 2026-09-23: a 0.69 m target on a 0.5 m arm) | The envelope's reach cap used to be a hardcoded UR10 1.3 m, whatever the arm | Fixed: `SafetyEnvelope.for_model` sizes `max_reach` from `UR_ROBOT_MODEL` (the cell files) or the Dashboard's `get robot model` (probed once before the first absolute move); `MODEL_REACH_M` covers UR3/5/7e/10/12e/15/16e/20/30. `locate` now returns `reachable` and the cockpit's event says **OUT OF REACH** before you press Move. `UR_MAX_REACH_M` overrides (long TCP). Doctor line `robot.model` shows the cap and flags a cell/controller model mismatch. |
| A multi-leg move (`ur_move_tcp_path`, the cockpit **Approach** cycle) stops part-way with `ok:false`, no protective stop, robot parked mid-path | **Any new URScript on 30001 replaces the running program.** A concurrent state poll whose RTDE read hiccuped (legacy `textmsg` fallback), a Locate (`get_flange_pose` is a script), or a second Move kills the cycle silently. Seen once on the UR3e 2026-09-23 (4-leg cycle died after leg 2) | Fixed inside one process: `PrimaryClient` holds a non-blocking in-flight lock — a concurrent submission raises `PrimaryBusyError`, and `get_state` reports `primary_busy` with no joints instead of sending. Across *processes* (a CLI `run-script` while the cockpit drives) nothing can protect you — don't. |
| The cockpit's flange-referenced approach lands a constant ~35 mm off the object, even with the TCP forced to zero | The hand-measured `PERCEPTRONICS_T_FLANGE_CAMERA` (to the camera housing) was 21 mm off in X, 36 mm in Z and had the tilt sign inverted — the depth origin is the **left IR imager**, not the housing centre | Run the touch-and-click hand-eye (mark = flange centre on the part with `set_tcp(p[0,…])`, 3 clicked views from varied wrist poses); 2026-09-23 on the UR3e: RMS 1.7 mm, located point 3 mm from the mark afterwards. The solved pose is in `perceptronics/cells/ur3.env` + `captures/calibration/handeye_ur3.json`. |
| `ur_flange_pose` / Locate gives a flange pose that is wrong by tens of mm while `get_tcp_offset()` looks plausible | The controller reported an active-TCP offset (`0,-0.035,0.22,…`) that was **not** what its motion actually used; `pose_trans(tcp, inv(offset))` then puts the "flange" in the wrong place | Force the TCP yourself: `set_tcp(p[0,0,0,0,0,0])` (persists until the next `set_tcp`/installation load) — then TCP == flange and every read/move agrees. `move_tcp(..., tcp=[0]*6)` / `--tcp` puts it in the same program as the `movel`; the flange-referenced cycle does this on every leg. |
| Robotiq gripper: port 63352 is closed from the network although the URCap is installed | the URCap daemon binds the controller's loopback only | `urctl gripper status\|open\|close\|move --position N` / `ur_gripper` / `POST /api/robot/gripper` — one Primary program opens the socket from inside the controller (UR3e + Hand-E, 2026-09-25) |
| Cockpit **Freedrive** (or `urctl freedrive on`, `ur_freedrive`) reports ok but the real arm stays stiff; the reteach/inspect dialogs *do* hand-guide | Freedrive lives only while the program that called `freedrive_mode()` runs. A one-line script ends instantly → freedrive ends instantly (URSim is lenient, hardware isn't) | Fixed: `Robot.freedrive(True, hold_s=600)` sends a bounded `sleep` loop that keeps the program alive and confirms via `textmsg("urctl/freedrive=on")` (`ok:false` if the echo never comes); `off` sends `end_freedrive_mode()` as a new program, which also kills the hold; the hold releases itself when it expires (`--hold`, tool `hold_s`, max 1 h). |

## Working in this repo

**Required checks on `dev`: `gate` and `URCap5 gate`** (2026-09-29). `URCap5 gate`
(`urcap5-matrix.yml`) passes at once on a PR that doesn't touch the URCap's paths and only on a
green `build` + every PolyScope version on one that does — so auto-merge can't land a red
URCap and the matrix never blocks unrelated PRs. Before a new required job, make sure it runs
(and answers) on every PR, or it blocks the ones it skips.

**CI/CD** (`.github/`): every PR runs lint (ruff check + format, shellcheck),
unit tests on Python 3.10/3.12/3.14, and a packaging job that builds the wheel,
verifies it carries `perceptronics/webui/` + the vendored `_urp_convert.py`, and
smoke-installs it. Integration tests (URSim boot) run on pushes to main or on
PRs labeled **`run-integration`**. Dependabot maintains the pinned dev requirements (pip), action pins,
and simulator images (weekly/monthly, grouped); minor/patch non-simulator
bumps auto-merge once CI is green (`dependabot-auto-merge.yml` — needs
"Allow auto-merge" enabled in repo settings). CodeQL scans Python + JS weekly
and per PR. Tagging `v<version>` (matching `urctl.__version__`) builds, tests,
and attaches the sdist + wheel to a GitHub Release (no PyPI package). The PolyScope 5
URCap has its own tag line: `urcap5-v<Bundle-Version>` attaches the committed
`urcap/dist/*.urcap` + sha256 to a GitHub Release after `urcap5.py release-check`
proves it is the tagged sources' build (`release-urcap5.yml`; CI never rebuilds it —
the URCap API jars exist only in the URSim image).

**Before changing anything public-facing — links, owner or vendor names, the licence, a bundle
id — check Nick's recorded decisions.** They live in the assistant's project memory (the decisions
log and the dated answers), not in `TODO.md`, which holds only open questions and work. On
2026-09-30 every link on `docs/index.html` was repointed against a standing decision and had to be
reverted; when reality contradicts a decision, write the mismatch under `TODO.md`'s open questions
and let Nick decide (he did, the next day: `LICENSE` is Nick Armenta's, and quick starts and links
use `JimothyJohn/perceptronics` only).

**`perceptronics doctor` exits 1 when the verdict is NOT READY** — the right answer on a machine
with no robot, so a script or CI step must not treat that exit code as a crash: look for the
`verdict:` line (the Windows CI step does; `scripts/setup-windows.ps1` ends with its own `exit 0`).
`doctor | head; echo $?` reports `head`'s 0, which is how this was misread once.

Unit tests assume a clean shell: with `UR_CELL` (or `PERCEPTRONICS_*`) exported,
the handeye/webapp/cockpit tests pick up the cell's defaults and fail — run them
with `env -u UR_CELL python3 -m pytest …` or in a fresh shell.

```bash
make sim-up          # start URSim
make sim-poweron     # power on + brake release (calls scripts/poweron.sh)
make test            # unit tests (pytest)
make test-integration # tests that need a running URSim
make lint            # ruff + shellcheck
make sim-down        # stop URSim
```

**No uv (Nick, 2026-09-30: "completely remove UV from the equation since we don't have any
dependencies") — this repo is the exception to the global Python → uv default.** The core
(`urctl` + perceptronics core) is pure stdlib, so nothing is installed to run it: any Python
≥ 3.10 runs it from the checkout as `python3 -m urctl …` / `python3 -m perceptronics …` /
`python3 -m perceptronics.mcp_server` (`.mcp.json`). macOS's own `/usr/bin/python3` is 3.9 — use
Homebrew's; `scripts/cockpit*.sh` check and take `PYTHON=`. The console scripts (`urctl`,
`perceptronics`, …) exist only after `pip install .`. numpy/OpenCV/torch are optional extras
(`python3 -m pip install -e ".[vision]"`).

The only third-party packages are the **development tools** — pytest, hypothesis, ruff, build,
hatchling. `requirements-dev.in` lists them; `requirements-dev.txt` pins them and everything
they pull in, with hashes, for every supported Python and OS (`requirements-vision.txt` does
the same for the `vision` extra). CI installs with `pip install --require-hashes -r …` (the
lockfile rule, kept) and builds with `python -m build --no-isolation` so hatchling is the
pinned one. Dependabot's `pip` ecosystem updates the pins and their hashes; a bump that needs a
*new* transitive package fails the hash check in CI — add it to the `.txt` by hand (name,
version, `--hash` lines from PyPI). Windows: `scripts/setup-windows.ps1` installs Python with
winget when none is found (`scripts/_python.ps1` finds it, skipping the Microsoft Store stub);
the Windows CI leg runs `cockpit.ps1 -Doctor` and `Windows-Setup.cmd` under Windows PowerShell 5.1.

```bash
make install-dev                              # .venv + the pinned dev tools (pip, hash-checked)
make test                                     # unit tests (.venv/bin/python -m pytest)
python3 -m urctl state                        # run the CLI: no env needed
```

Host-side control (`urctl` below is `python3 -m urctl`, or the console script after
`pip install .`), against the sim or a real robot:

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

**The vendor-neutral seam (`urctl/controller.py`, 2026-09-26):** `Controller`
and `Gripper` are `typing.Protocol`s — `Robot` satisfies `Controller`
structurally (e-Series and PolyScope X are one class selected by
`RobotConfig.platform`), `Robot.gripper_device` is the `RobotiqUrcapGripper`
adapter. A Fanuc is a second class satisfying `Controller`; the tools, the
cockpit's `RobotLink`, `pick-cycle` and `calibrate` program against the
protocol's method names. **Tool names are canonical without a vendor prefix**
(`move_tcp`, `get_state`, `gripper`, …); the original `ur_*` names are aliases
(`urctl.tools.TOOL_ALIASES`) accepted by `call_tool`, both MCP servers and the
CLI, so existing agent configurations and the `ur_*` mentions in this file keep
working. `tools/list` shows canonical names; `get_tool_schemas(include_aliases=True)`
adds the aliases.

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
  sudo docker exec perceptronics-ursim-1 tail -f /ursim/polyscope.log
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
