# TODO

## Open questions for Nick

- 2026-09-02 — RealSense on macOS needs root to open the camera (libusb must detach Apple's UVC driver). The live hardware path (`sudo uv run pytest -m realsense`, `sudo uv run perception gui`) is unverified until you run it once with your password; everything else is tested on the synthetic camera + the real dylib. Result?
- 2026-09-02 — D435 USB-C location: Intel's drawing doesn't dimension the connector face. The bracket leaves the camera's back and both ends free (5 mm above the plate), which covers both a rear and an end exit — please confirm on the unit which face it is (spec §6 A1) before the second print.
- 2026-09-02 — Bracket clocking: default `ARM_ANGLE_DEG=0` puts the camera 90° from the dowel / tool-I/O connector. Do you want it opposite the connector (180°) so the USB and tool cables route on different sides?
- 2026-09-02 — On-controller deployment was not attempted (robot at 192.168.1.50 unreachable today; URSim can't see USB). With the Jetson as the target, do you still want a feasibility poke at the CB's Debian when the robot is back on the network?
- 2026-09-02 — `Dockerfile.perception` pins librealsense at a tag verified to exist; the Jetson (JetPack / L4T) base image for CUDA + SAM is left as a follow-up — which JetPack version will the Orin run?
- 2026-09-02 — SAM backend (`--segment-backend sam`) is written against transformers 5.16's SamProcessor/SamModel API but was not run against weights in this session (no torch installed here). OK to leave as "wired, unverified", or should I pull torch + sam-vit-base and prove it on the Mac?
