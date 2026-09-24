# Monocular scan: one global-shutter camera + RTDE poses (design note)

*2026-09-24. Status: **parked** (Nick, later that day) after building everything that needs no print or purchase; verified on the synthetic rig (§9) and the synthetic cockpit, not yet on the UR3e. What is left is in `TODO.md` ("Monocular scan — backburner").*
*Target: the UR3e cell (`perception/cells/ur3.env`), camera on the existing tool bracket.*
*Compute: Nick's laptop (CPU only) now; an NVIDIA Thor at the cell later. Everything but future ML segmentation must run on CPU.*

Goal: replace the RealSense with a low-cost fixed-lens 2D camera and get 3D object
positions + heights from a short robot move, using the robot's own pose stream as the
second "eye". Poses come from RTDE (500 Hz on e-Series, `actual_TCP_pose`,
`actual_q`, `timestamp`, digital I/O bits); scale is metric because the baseline is
read off the robot. This is *not* COLMAP: nothing is solved for except the points.

## 1. The two forks (decide these first)

### 1a. Global vs rolling shutter

| | Rolling shutter (webcams, most CSI modules) | Global shutter (OV9281/OV9782, IMX296, AR0234) |
| --- | --- | --- |
| Static (stop-and-shoot) | fine | fine |
| Dynamic (capture during the move) | **unusable for metrology.** Rows are exposed at different times; a ~20 ms readout at 250 mm/s skews the frame 5 mm top-to-bottom, and the skew depends on motion direction, so it can't be calibrated out as a constant | one exposure instant per frame; motion blur = speed × exposure (0.25 mm at 250 mm/s and 1 ms) |
| Hardware trigger input | rare | common (that is the point of these modules) |
| Cost | $10–30 | $30–80 |

**Decision (Nick, 2026-09-24): global shutter with a trigger input.** It is the only thing that keeps the
dynamic path open, and it is also what a line laser needs (§5). Candidates: Arducam
OV9281 (1280×800 mono) / OV9782 (colour) USB or CSI, IMX296 CSI on the Jetson. Mono is
fine for everything in this note; colour only matters if colour segmentation stays.
Lock exposure, gain, white balance; fixed-focus lens (autofocus changes intrinsics
per frame). Pick the lens so the bin fills the frame at the scan height: 60–70° HFOV at
0.3–0.4 m on the UR3e.

### 1b. Static vs dynamic capture

| | Static: move, settle, shoot, N times | Dynamic: one continuous sweep, frames streamed |
| --- | --- | --- |
| Pose per frame | trivial: read once after the arm settles (`actual_TCP_speed` ≈ 0) | must be *synchronised* to the exposure instant (§2) |
| Shutter | either | global only |
| Scan time (150 mm baseline) | 3–5 stops × ~1.2 s floor (Dashboard pre-check + Primary round-trip) ≈ **4–6 s** | one `move_trajectory` or `movel`, ≈ **1–1.5 s** at 150–250 mm/s incl. accel |
| Frames | 3–5 | 50–100 at 60 fps (many baselines: robust, redundant) |
| Line laser (§5) | not possible (needs the sweep) | native |
| Risk | none new | sync error → depth error (§2 numbers) |

**Decision (Nick, 2026-09-24): dynamic, T2 trigger.** Nick wants one move, not several, and the laser path needs it.
Static stays as the degraded mode (any camera, no trigger wiring) and as the way to
bring the pipeline up before the trigger is wired.

## 2. Frame ↔ pose sync (why you *don't* have to stop and shoot)

The problem is not RTDE; it's the camera. RTDE gives a pose every 2 ms with the
controller's timestamp. A USB camera delivers a frame 1–3 frame-times after the
exposure with an unknown, drifting latency. Three tiers:

| Tier | How | Pose error at 100 mm/s | at 250 mm/s |
| --- | --- | --- | --- |
| T0 software | host receive time, latency calibrated once (§2.1) | ±1 frame jitter at 60 fps ≈ ±1.6 mm | ±4 mm |
| T1 host trigger | Jetson GPIO fires the camera; host stamps the edge; RTDE clock offset measured | ~2–4 ms ≈ 0.2–0.4 mm | 0.5–1 mm |
| **T2 robot-clock trigger** | the trigger edge is *visible in RTDE*: the camera is triggered from a UR **tool digital output** pulsed by a URScript thread in the same program that runs the sweep (**the decision**: the laptop has no GPIO, so the robot owns the trigger; a Jetson/Thor could alternatively trigger the camera and toggle a UR tool DI on the same edge). Frame *k* ↔ the *k*-th edge in `actual_digital_output_bits` / `actual_digital_input_bits`, pose = `actual_TCP_pose` at that sample (interpolate between the two neighbours) | ≤ 1 RTDE sample = 2 ms → **0.2 mm** | 0.5 mm |

T2 is the one to build. It has no clock-offset problem at all, the tool I/O is on the
same connector the camera already sits next to, and it costs a level shifter (UR tool
DO is 24 V/12 V, camera trigger is 3.3 V). Trigger from the Jetson side (camera + UR DI)
if you want the host to own the exposure schedule; trigger from the robot side (UR DO →
camera) if you want the sweep program self-contained. Either way `urctl rtde-state
--deep` already decodes the I/O bits; the recorder just has to run RTDE at full rate
during the sweep.

Depth error from pose error is roughly the pose error itself (the baseline is
150 mm, so a 0.2 mm baseline error is 0.13 % of depth). Angular error dominates at
range: 0.1° of orientation error at 0.35 m is 0.6 mm. RTDE joint angles are the
controller's own, so orientation is as good as the hand-eye (§3).

### 2.1 Latency calibration (T0 only)

Sweep past the ChArUco plate at constant speed, PnP every frame, fit the one scalar
`t_offset` that makes the PnP trajectory agree with the RTDE trajectory. One-parameter
line search, 20 s, and it doubles as a check that the hand-eye is right. Still do it
even with T2 wired: it verifies the wiring.

## 3. Calibration: the one-minute hand-aided setup

Everything below is per bracket print / per camera, once. Camera intrinsics are the
only thing that needs more than a minute and they are once per camera+lens, on the bench.

1. **Intrinsics** (bench, 5 min, once per camera+lens): 20–30 hand-held views of the
   plate, OpenCV ChArUco calibration. Store `K`, distortion, image size. Reject if
   reprojection RMS > 0.3 px.
2. **Plate → base** (30 s): probe on the flange (`hardware/charuco-board/`, TCP
   `[0,0,0.050]` on the bracket). In the cockpit, freedrive the tip into dimple **A**,
   click *Record A*; same for **B** and **C**. Three flange poses + the probe TCP give
   `T_base_plate` (origin A, +X toward B, +Y toward C) with no feature readback at all.
   This reuses `CalibrationSession.record_mark`. The pendant's Plane tool does the same
   geometry, but a taught feature is *not* visible to scripts sent over Primary (no
   installation preamble) and the UR3e's SSH is closed, so it can't be read back; keep
   it as the operator's cross-check, not the data path.
3. **Hand-eye** (30 s): operator freedrives (or a scripted 5-pose fan) so the camera sees
   the plate from 4–6 wrist orientations. Per view: PnP gives `T_cam_pattern`, RTDE
   gives `T_base_flange`, the feature + `T_plate_pattern` give `T_base_pattern`, so
   **every view is a closed-form solve** for `T_flange_camera`; average the solves and
   report the spread. No AX=XB, no Tsai, no minimum-motion condition. The existing
   `perception/calibrate.py` LM solver keeps its structure; the observation model changes
   from a depth-derived point to a PnP pose. Reject if the per-view spread > 1 mm / 0.2°.
4. **Table plane** (20 s): three more recorded touches on the table (or the bin
   floor) with the probe, same button. All heights are measured from this plane, which
   is what makes textureless parts tractable (§5).

Hand-eye error is the one that hurts: 1° at 0.35 m is 6 mm. Step 3's spread number is
the health metric; `perception doctor` should print it.

## 4. Runtime: the scan, cycle time, multiple objects

- **The scan is the approach.** Start the sweep from wherever the arm is, end it above
  the bin at the pick approach height. The 150 mm of baseline is spent on the way to
  the pick, so the scan's cost is mostly what the approach already cost.
- **Sweep**: `movel` 150 mm across the bin at 150–250 mm/s, 60 fps triggered
  (T2), RTDE logging at 500 Hz. Time ≈ 0.9 s at 250 mm/s, 1.4 s at 150 mm/s.
- **Compute** (laptop CPU): passive multi-view on 640×480 is 40–80 ms per rectified
  pair (SGBM), ~10 ms with CUDA later on the Thor; fuse 4–6 pairs by median ⇒ < 0.3 s after the
  last frame. Laser-line peak extraction is < 2 ms per frame and runs as frames arrive, so
  the profile cloud is complete a few ms after the sweep ends.
- **Cycle time**: **≈ 1.5–2 s from "scan" to "all objects located"** in the dynamic path,
  4–6 s in the static path.
- **Multiple objects**: one scan yields a height map (or profile cloud) over the whole
  swept field, so every object in it is located in the same pass. Segmentation of
  "things above the table plane" is a height threshold + connected components: no
  ML, no colour. Re-scan only when the scene changes: after each pick, take a single
  untriggered 2D frame from the approach pose and check the remaining blobs' outlines
  haven't moved (a 30 ms silhouette diff); re-scan when they have, or every N picks.
  With a line laser the swept strip is (line length) × 150 mm; a bigger bin needs a
  longer sweep or a second pass, whereas passive multi-view covers the whole camera FOV.

## 5. Textureless, reflective parts under bad lighting

This is the real constraint and it rules out the naive plan. Passive stereo / SfM
matches *texture*; machined and moulded parts have none, and specular highlights move
with viewpoint, so they actively lie. Three strategies, cheapest first. Plan on 1 + 2
together; 3 is the fallback.

1. **Model-based edges, not points.** The CAD (§6) gives each part's silhouette per
   resting pose. Detect edges in each frame, back-project through the known pose onto the
   table plane, and match outlines (chamfer / contour ICP) against the CAD silhouettes at
   the plane distance. This is exactly what the end-state "2D camera alone" needs
   anyway. Height comes from the CAD once the resting pose is identified; it is
   verified, not measured, by the multi-view edge parallax (edges *do* triangulate,
   surfaces don't). Robust to texture, sensitive to a busy background: a plain, matte,
   contrasting bin floor is a requirement, not a preference.
2. **Line laser on the bracket** (≈ $40: 650 nm 5 mW line module + 650 nm bandpass filter
   on the lens). The sweep the robot is already doing turns the camera into a laser-line
   profilometer: one profile per frame, depth by triangulation against the laser plane,
   sub-pixel peak ⇒ ~0.3 mm at 0.35 m with a 60 mm camera-laser baseline. Immune to
   texture; the bandpass + short exposure make it immune to ambient lighting. Reflective
   metal gives blooming and ghost lines: mitigate with two exposures per profile (HDR),
   lower laser power, and reject peaks that don't lie on the laser plane's epipolar
   line. It also gives you the per-part height directly, which strategy 1 can't for
   unknown or stacked parts. Costs a one-time laser-plane calibration (sweep the plate;
   the plane is fit from the line on the known board). *Needs global shutter + the
   dynamic path*, which is why §1 decided the way it did.
3. **Second camera** (true stereo, ~$60 more): only if 1+2 fail on parts that are both
   featureless and too shiny for the laser.

Strategy 1 is the pure-2D end state; strategy 2 is what makes the initial scan and
height measurement reliable on the parts described. Both share the camera, bracket,
trigger, sync, and calibration. Do not budget on SGBM working on these parts.

## 6. CAD ingestion (STL / STEP)

- **STL**: stdlib parse (binary + ASCII), convex hull, **stable resting poses** (hull
  faces whose support region contains the centroid's projection, ranked by face area),
  per resting pose: height above the table, footprint outline (mesh projected onto the
  plane), oriented bounding box, and the grasp point transformed into that pose.
- **STEP**: tessellate with OCP/CadQuery (`uv run --with cadquery`, the same optional
  dependency the hardware build uses; ~150 MB, offline) then the STL path. Not in the
  zero-dependency core.
- Output: `parts/<name>.json` = the per-pose silhouettes + heights the runtime matches
  against (§5.1). The operator's upload is the whole part library step.

## 7. Bill of materials (UR3e cell)

| Item | Approx. | Note |
| --- | --- | --- |
| Global-shutter camera, trigger input, fixed M12 lens (OV9281/OV9782 USB or IMX296 CSI) | $40–80 | §1a |
| 650 nm line laser module + 650 nm bandpass filter | $40 | §5.2 |
| Level shifter / opto for the trigger line (UR tool 24 V ↔ 3.3 V) | $5 | §2 T2 |
| Calibration plate + probe (printed) + 4 magnets | $5 | `hardware/charuco-board/` |
| Bracket revision: camera + laser at ~60 mm spacing, laser tilted ~30° toward the optical axis | print | `hardware/d435-tool-bracket/` derivative |

## 8. Fit with the repo (as built)

| Piece | Module | Command |
| --- | --- | --- |
| RTDE streaming + pose-by-host-time | `urctl/rtde.py` `stream()`, `perception/posestream.py` | — |
| 2D sources: RealSense colour, synthetic renderer | `perception/monocam.py` | `--fake` |
| Table plane (z, 3 points, env/file) | `perception/tableplane.py` | `perception touch --set table` |
| Probe touches (table 1/2/3, plate A/B/C) | `perception/touch.py` | `perception touch` |
| The sweep (frames + poses, save/replay) + fake rig | `perception/sweep.py` | `perception scan` |
| Locator: segment → back-project → parallax seed → box fit → library | `perception/locate2d.py`, `perception/boxfit.py` | `perception locate` |
| Part library from STL (OBB, resting poses); STEP via CadQuery | `perception/partlib.py`, `parts/` | `perception part-info`, `make-box` |
| ChArUco PnP, closed-form hand-eye, plate frame | `perception/charuco.py` | (hand-eye command: next) |
| Latency fit: ChArUco (needs the plate) or **`--objects`** (no plate: the box-fit IoU over the parts vs the latency) | `perception/latency.py` | `perception latency-fit [--objects]` |
| Table plane from the depth (no probe; RANSAC on one RGB-D frame + the flange pose) | `perception/tableplane.py` `plane_from_depth` | `perception table-from-depth`, cockpit **Table from depth**, MCP `cell_table_from_depth` |
| Depth cross-check of every located object (the RealSense grades the mono result) | `perception/depthcheck.py` | `depth_check` per object in `scan` / cockpit / MCP |
| Left-IR stream (global shutter) as the 2D camera — *unverified on hardware* | `perception/realsense.py` `infrared=True`, `RealSenseMono(stream="ir")` | `perception scan --stream ir` |
| Cockpit **Scan (mono)** panel: Table from depth → Scan → Approach an object | `perception/webapp.py` (`/api/scan`, `/api/scan/approach`, `/api/table/from_depth`) | `perception gui`, demo: `perception gui --fake-scan` |
| MCP tools over the cockpit | `perception/mcp_server.py` | `cam_scan`, `cam_scan_result`, `cam_scan_approach`, `cell_table_from_depth` |
| Sweep re-stamping offline (the recorder's trajectory is saved with the sweep) | `perception/sweep.py` `Sweep.restamp` | `latency-fit` uses it |

## 9. Verified on the synthetic rig (2026-09-24)

`perception scan --fake`: three 50×30×30 blocks (two flat, one standing), 150 mm
sweep at 0.15 m/s, camera 0.35 m up, 40 ms simulated latency, sensor noise.
Result: centres within 0.2 mm, yaw within 0.1°, heights within 1 mm, the
standing block matched to the right resting pose, IoU 0.99; ~2 s CPU for the
fit. With the latency left at 0 the blocks shift 5 mm along the sweep and IoU
drops to 0.89 — the fit number is the health metric. The parallax *seed* alone
under-reads height by ~40 % (side faces), which is why the model fit is not
optional. `tests/test_monoscan.py` locks all of this in.

**Frame stamps.** A frame's ``host_t`` is its *arrival* time unless the camera reports
an exposure instant (``RgbdFrame.extra["exposure_t"]``: the synthetic sweep camera does,
a triggered camera will). The cockpit's synthetic scene renders slowly (~100 ms a
frame), which is exactly how an unmodelled latency shows up: the moving frames form a
second cluster shifted along the sweep. That failure mode is now a test, and the
exposure stamp is the convention any future camera source should follow.

**Next on hardware (UR3e), no prints needed:** (1) `perception --cell ur3
table-from-depth` over a clear patch (or the cockpit's *Table from depth*), (2) a
slow sweep, `perception --cell ur3 scan --velocity 0.05 --parts parts/block_50x30x30.stl`,
and read `depth_check` per object, (3) `perception --cell ur3 latency-fit <sweep>
--objects --save`, then raise the speed, (4) `--stream ir` for the global-shutter
imager. The plate/probe steps and the purchases are in `TODO.md`.

## 10. Original repo-fit plan

Camera-specific code is one module (`perception/realsense.py`). Everything else
(`segment`, `handeye`, `pose`, `robotlink`, `cells`, cockpit, doctor, MCP tools) is
agnostic. New pieces, in build order:

1. `perception/monocam.py`: triggered capture (V4L2/UVC or CSI) + RTDE recorder →
   `(frame, T_base_flange, t)` stream; static mode first, T2 trigger second.
2. `perception/calib_charuco.py`: intrinsics + the closed-form hand-eye of §3 (+ the
   latency fit of §2.1); reads `hardware/charuco-board/out/board.json`.
3. `perception/mvs.py`: virtual-stereo depth from pose pairs (bring-up + textured parts).
4. `perception/laserline.py`: laser-plane calibration + per-frame profiles.
5. `perception/partlib.py`: STL/STEP → resting poses/silhouettes; `perception/outline.py`:
   edge matching against the library.
6. Doctor lines: intrinsics, hand-eye spread, plane feature, trigger edges seen in RTDE.
