# TODO

## Open questions for Nick

- 2026-09-03 — macOS camera path: `sudo` works (hardware tests passed, 640×480@30 on USB 3), but re-opening the camera after a process released it loses a race against Apple's `UVCAssistant` (verified in the unified log; docs/realsense.md Troubleshooting). Pending your check: does a fresh re-plug + one open win *every* time, and does the cockpit then hold a stream indefinitely? If yes, "re-plug, then one long-lived process" is the documented Mac workflow and the Jetson is unaffected. If no, next candidate is `rs2_hardware_reset` + retry in `open()` (symbol present in 2.58.4, not yet wired).
- 2026-09-02 — D435 USB-C location: Intel's drawing doesn't dimension the connector face. The bracket leaves the camera's back and both ends free (5 mm above the plate), which covers both a rear and an end exit — please confirm on the unit which face it is (spec §6 A1) before the second print.
- 2026-09-02 — Bracket clocking: default `ARM_ANGLE_DEG=0` puts the camera 90° from the dowel / tool-I/O connector. Do you want it opposite the connector (180°) so the USB and tool cables route on different sides?
- 2026-09-02 — On-controller deployment was not attempted (robot at 192.168.1.50 unreachable today; URSim can't see USB). With the Jetson as the target, do you still want a feasibility poke at the CB's Debian when the robot is back on the network?
- 2026-09-02 — `Dockerfile.perception` pins librealsense at a tag verified to exist; the Jetson (JetPack / L4T) base image for CUDA + SAM is left as a follow-up — which JetPack version will the Orin run?
- 2026-09-02 — SAM backend (`--segment-backend sam`) is written against transformers 5.16's SamProcessor/SamModel API but was not run against weights in this session (no torch installed here). OK to leave as "wired, unverified", or should I pull torch + sam-vit-base and prove it on the Mac?
