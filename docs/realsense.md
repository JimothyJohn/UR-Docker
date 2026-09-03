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

The container has no auth — it is a cell-network cockpit, like `urctl gui`.
Keep it off routable networks or put it behind the Jetson's firewall.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `failed to set power state` / `RS2_USB_STATUS_ACCESS` on macOS | libusb can't detach the built-in UVC driver | run under `sudo`; close any app holding the camera |
| same on Linux | udev rules missing | install librealsense's `99-realsense-libusb.rules`, re-plug |
| `librealsense2 not found` | SDK not installed / not on the search path | `brew install librealsense`, or set `REALSENSE_LIB` |
| `usb 2.x` chip in the header | camera on a USB 2 port/cable | depth at 640×480@30 needs USB 3; use the short USB-C 3.x cable |
| stream stalls after a while | USB-C cable too long / hub | RealSense is picky: ≤ 2 m active-free cable, direct port |
