# TODO

## Open questions for Nick

- 2026-09-02 — `Dockerfile.perception` pins librealsense at a tag verified to exist; the on-controller GPU target is **not decided** (UR AI Accelerator as a URCap-X container vs a separate Jetson). When it is: which JetPack / L4T base to pin for CUDA + SAM.
- 2026-09-12 — Controller IPs for the `ur3` (UR3e, PolyScope 5) and `ur20` (PolyScope X) cells — `perception/cells/*.env` ship with `UR_HOST` empty.
- 2026-09-12 — The Windows path (`scripts/setup-windows.ps1`, `scripts/cockpit.ps1`, `REALSENSE_LIB` at the SDK's default `bin\x64\realsense2.dll`) is written against the librealsense repo's own references but has NOT been run on a Windows box yet. First run on the work laptop is the verification; paste the doctor output (`uv run perception --cell ur20 doctor --stream --json`) if anything fails.

- 2026-09-25 — The Mac Studio lost the D435 claim race with nothing else on the bus (43 libusb capture-resets in 40 s, `docs/realsense.md` §Troubleshooting). Drop the Mac as a camera host (laptop/WSL2 + Jetson only), or spend one more flagged experiment on it: fewer handle opens per start — skip the stream-mode enumeration on darwin, `tuning=None` (no preset/laser HWM writes), `RS2_OPTION_GLOBAL_TIME_ENABLED` (53) off on both sensors before `pipeline_start` — then `sudo scripts/rs_probe.sh` after a re-plug. Needs your hands (re-plug + sudo); ~30 lines behind an env flag if you say go.

## Decisions (so they don't get re-asked)

- 2026-09-04 — Perception / send-to-robot work targets PolyScope X; the e-Series sim on the Mac Studio is abandoned (Xvfb dies under Rosetta, URControl under QEMU). URSim e-Series stays CI-only.
- 2026-09-12 — The UR3e in the test cell is **PolyScope 5 (e-Series)**; the UR20 is PolyScope X. Cells carry the platform, so both work; `UR_PLATFORM` keeps its `e-series` default (no global flip).
- 2026-09-12 — Bracket camera side: +Y, the tool-I/O side (Rev B); the UR20 print clocked 45° off its M8 socket.
- 2026-09-12 — Hand-eye calibration routine in the cockpit: **yes, build it** before the demo.
- 2026-09-12 — Repo push: fork `Olympus-Controls/UR-utils` and push the branch to the fork; PR from there.
- 2026-09-12 — D435 USB-C is on an **end face** of Nick's unit (not the back as in Intel's mesh); bracket clears it, spec §6 A1 updated.
- 2026-09-12 — SAM backend stays "wired, unverified" until a GPU box exists; the demo uses the stub segmenter.
- 2026-09-12 — Dependabot ignores `universalrobots/ursim_polyscopex` bumps (pinned 10.13.0; re-test new tags by hand).

## Monocular scan — backburner (2026-09-24, `docs/mono-scan.md`)

Built and verified on the synthetic rig; parked until there is time at the UR3e. Everything below needs hardware, a print, or a purchase — the software side is done up to the point marked.

**Needs the 3D prints (`hardware/charuco-board/`):**
- Print the ChArUco plate + the touch probe (flat seating face, no pilot); print the pattern PDF at 100 %, cut on the line, A corner at the notch.
- `perception --cell ur3 touch --set plate --label A|B|C` with the probe (TCP `[0,0,0.050]` on the bracket) → `T_base_plate`.
- `perception latency-fit <sweep over the plate> --save` — the ChArUco path (PnP per frame). Until then the print-free `--objects` path (fit the latency by silhouette IoU over the blocks) is the way.
- ChArUco hand-eye command (`perception/charuco.py` has the closed-form per-view solve + averaging; no CLI yet): needed for any camera that is *not* the RealSense — the current hand-eye came from the depth-based touch-and-click routine.
- Camera-intrinsics command for a non-RealSense camera (ChArUco calibration; the RealSense reports its own).

**Needs the UR3e (no print):**
- `perception --cell ur3 table-from-depth` (one RGB-D frame of the empty surface → table plane) — or three flange-centre touches with `touch --probe-tcp 0 0 0 0 0 0`.
- First sweep at 0.05 m/s (rolling-shutter colour): `perception --cell ur3 scan --parts parts/block_50x30x30.stl`; read `depth_check` per object (the RealSense depth grades the mono result).
- `perception latency-fit <that sweep> --objects --save`, then raise the sweep speed.
- Try `--stream ir` (left IR imager = global shutter, emitter off): built blind against the SDK's C API, **unverified on the camera**; verify the IR frame arrives and the hand-eye (depth frame) applies without extrinsics.
- Cockpit **Scan** panel + `cam_scan*` MCP tools on the real cell (verified on the synthetic cockpit only).

**Later / purchases:**
- Global-shutter camera with a trigger input (Arducam OV9281/OV9782 USB, or IMX296 CSI on the Thor) + a 24 V→3.3 V level shifter: the T2 trigger (UR tool DO → camera, edges read from RTDE's `actual_digital_output_bits`; `PoseRecorder.edges()` already extracts them).
- Line laser (650 nm module + bandpass filter) on the bracket for textureless/reflective parts; laser-plane calibration over the plate.
- Model-based edge matching for non-box parts (the box fit uses the OBB corners; use the CAD hull's silhouette).
- Stable-pose analysis from the convex hull (resting poses are OBB-based now: exact for boxes only).
- NVIDIA Thor: CUDA SGBM / SAM; `Dockerfile.perception` retarget.
