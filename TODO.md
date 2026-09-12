# TODO

## Open questions for Nick

- 2026-09-02 — D435 USB-C location: Intel's drawing doesn't dimension the connector face. The bracket leaves the camera's back and both ends free (5 mm above the plate), which covers both a rear and an end exit — please confirm on the unit which face it is (spec §6 A1) before the second print.
- 2026-09-02 — `Dockerfile.perception` pins librealsense at a tag verified to exist; the Jetson / on-controller GPU (JetPack / L4T) base image for CUDA + SAM is left as a follow-up — which JetPack version, and is the target the UR AI Accelerator (PolyScope X + Jetson AGX Orin, deployed as a URCap-X container)?
- 2026-09-02 — SAM backend (`--segment-backend sam`) is written against transformers' SamProcessor/SamModel API but was never run against weights (no torch here). Leave as "wired, unverified", or prove it on the Mac now that the camera path works?
- 2026-09-12 — Controller IPs for the `ur3` (UR3e, PolyScope 5) and `ur20` (PolyScope X) cells — `perception/cells/*.env` ship with `UR_HOST` empty.
- 2026-09-12 — The Windows path (`scripts/setup-windows.ps1`, `scripts/cockpit.ps1`, `REALSENSE_LIB` at the SDK's default `bin\x64\realsense2.dll`) is written against the librealsense repo's own references but has NOT been run on a Windows box yet. First run on the work laptop is the verification; paste the doctor output (`uv run perception --cell ur20 doctor --stream --json`) if anything fails.

## Decisions (so they don't get re-asked)

- 2026-09-04 — Perception / send-to-robot work targets PolyScope X; the e-Series sim on the Mac Studio is abandoned (Xvfb dies under Rosetta, URControl under QEMU). URSim e-Series stays CI-only.
- 2026-09-12 — The UR3e in the test cell is **PolyScope 5 (e-Series)**; the UR20 is PolyScope X. Cells carry the platform, so both work; `UR_PLATFORM` keeps its `e-series` default (no global flip).
- 2026-09-12 — Bracket camera side: +Y, the tool-I/O side (Rev B); the UR20 print clocked 45° off its M8 socket.
- 2026-09-12 — Hand-eye calibration routine in the cockpit: **yes, build it** before the demo.
- 2026-09-12 — Repo push: fork `Olympus-Controls/UR-utils` and push the branch to the fork; PR from there.
- 2026-09-12 — Dependabot ignores `universalrobots/ursim_polyscopex` bumps (pinned 10.13.0; re-test new tags by hand).
