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

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `failed to set power state` / `RS2_USB_STATUS_ACCESS` on macOS **without** `sudo` | libusb can't take the interfaces from Apple's UVC driver unprivileged | run under `sudo`; close any app holding the camera |
| the same **under `sudo`**, or the pipeline starts but `Frame didn't arrive within 5000` on the first read | **A race with macOS's own camera driver, lost.** Verified in the unified log on macOS 26 (Darwin 25): to claim the camera, libusb (as root) issues a USB reset (`AppleUSBHostPort::terminateDevice … reset API call`); the device re-enumerates, and within ~60 ms both our process and Apple's `UVCAssistant` (the userspace UVC extension) try to open each interface exclusively (`openGated: failed to open … already opened for exclusive access by pid 588, UVCAssistant`). Whoever wins each interface holds it: lose the depth interface → `RS2_USB_STATUS_ACCESS`; lose only the RGB one → a half-alive pipeline that never delivers a frameset. Reproduced once on 2026-09-03 after a burst of failed one-shot opens; after a re-plug, `scripts/rs_probe.sh` then streamed at 30 fps on both the first open **and** a clean re-open, so a healthy camera re-opens fine — it is the *failed* opens that cascade (each resets the device and re-runs the race). | Unplug and re-plug the camera to clear it, then confirm with `scripts/rs_probe.sh` (headless: runs the cockpit twice and prints frames/fps/last_error + who won each USB interface in the OS log; `--fake` self-tests without root). Don't retry failed opens in a tight loop — the cockpit backs off 1→30 s. Prefer one long-lived process (the cockpit, tunnelled over SSH with `ssh -L 7621:127.0.0.1:7621`) to back-to-back one-shot commands. macOS dev-box only: Linux/Jetson has no competing UVC daemon once the udev rules are installed. |
| USB 2 link (`usb 2.x` in `rs-info` / the header) | depth + colour at 640×480@30 exceeds the USB 2 budget, the SDK just stops delivering | the camera auto-caps at 15 fps on USB 2 (`--rs-fps` forces); move to a direct USB 3 port for 30 |
| same on Linux | udev rules missing | install librealsense's `99-realsense-libusb.rules`, re-plug |
| `librealsense2 not found` | SDK not installed / not on the search path | `brew install librealsense`, or set `REALSENSE_LIB` |
| stream stalls after a while | USB-C cable too long / hub | RealSense is picky: ≤ 2 m active-free cable, direct port |
