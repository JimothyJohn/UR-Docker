# `perception/` — RGB → monocular depth → blob detection

A self-contained perception stack, **parallel to `urctl/`** (it does not import
it). Same design DNA as the control library: env-driven config, Protocol-based
pluggable backends, a facade, a plain-data agent tool registry, and a CLI.

## Pipeline

```
FrameSource ──▶ Frame ──▶ DepthEstimator ──▶ DepthMap ──┐
 (webcam)        (RGB)     (depth backend)    (metres)    ├──▶ BlobDetector ──▶ [Blob]
                                                          │     (blob backend)   2D + depth
                                       Frame ─────────────┘
```

A **`Blob`** is `centroid_px`, `bbox`, `area_px`, `depth_m`, `mean_rgb` — *2D +
depth*, no 3D deprojection yet (that's the integration step, below).

## Backends are pluggable; the default is dependency-free

| Stage | `stub` (default, pure Python) | Optional real backend |
| ----- | ----------------------------- | --------------------- |
| Depth | `StubDepthEstimator` — heuristic (vertical position + brightness), produces metres in `[near, far]` | `depth_anything` → Depth Anything V2 (`transformers` + `torch`) |
| Blobs | `StubBlobDetector` — chroma threshold + color-similarity region growing | `blob_cv` → `cv2.SimpleBlobDetector` |
| Source | `Frame.from_png` (dependency-free PNG decoder); tests also build frames in-memory | `DeviceSource` — webcam via OpenCV |

### How the stub segments

The stub finds **colorful objects on a neutral background** — the apples-on-steel
case this repo targets. Three steps:

1. **Chroma threshold** — a pixel is foreground when its *chroma* (`max(R,G,B) −
   min(R,G,B)`) exceeds `blob_min_chroma`. Neutral surfaces (brushed steel),
   white overlay text, and specular highlights are all near-zero chroma and
   ignored for free.
2. **Color-similarity region growing** — foreground pixels join a blob only when
   a neighbor's color is within `blob_link_tolerance` (RGB distance), so two
   *touching, differently colored* objects (a red and a green apple) split apart
   while a single shaded object stays whole.
3. **Instance split** (`blob_split_touching`, on by default) — two *same-colored*
   touching objects survive step 2 as one region. A lightweight distance-
   transform watershed finds one peak per object center and assigns each pixel to
   its nearest peak, so the two touching red apples separate into two instances.
   A single convex object has one peak and is left whole.

On `inputs/image.png` this yields **3 apples** (two red, one green); set
`blob_split_touching=False` (or `PERCEPTION_BLOB_SPLIT_TOUCHING=0`) to fall back
to **2 color regions** (the red pair merged). Occlusion and irregular shapes are
where a learned segmenter (the upgrade path) earns its keep.

The stub backends import nothing third-party, so the core runs (and is fully
tested) on a box with no numpy, no OpenCV, no GPU. Selecting a real backend is a
**config change** — output shape is identical either way.

```bash
pip install -e .                      # core only (stubs)
pip install -e .[perception]          # + numpy + OpenCV (webcam, blob_cv)
pip install -e .[perception-torch]    # + Depth Anything V2
```

## Usage

```bash
perceive synthetic                                   # no camera, no weights
perceive image inputs/image.png                      # the apples-on-steel fixture
perceive capture                                     # one webcam frame → blobs
perceive --depth-backend depth_anything capture      # real monocular depth
perceive tools                                       # agent tool schemas (JSON)
perceive call perceive_synthetic --json '{"width":320,"height":240}'
```

`perceive image inputs/image.png` on the bundled fixture (three apples on a
brushed-steel table) returns three blobs — the two (touching) red apples split
apart plus the green apple — each with its centroid, bbox, mean color, and
sampled depth, with the steel and the camera's overlay text correctly ignored.
This is the test fixture in `tests/test_perception.py`.

```python
from perception import PerceptionPipeline, PerceptionConfig
from perception.frame import synthetic_frame

pipe = PerceptionPipeline(PerceptionConfig.from_env())
print(pipe.process(synthetic_frame()).as_dict())     # JSON-ready
```

Config is read from `PERCEPTION_*` env vars (`PERCEPTION_WIDTH`,
`PERCEPTION_FPS`, `PERCEPTION_DEVICE`, `PERCEPTION_DEPTH_BACKEND`,
`PERCEPTION_BLOB_BACKEND`, …) or `--flags`, mirroring `RobotConfig`.

## Default geometry

640×480 @ 15 fps. Enough resolution for meaningful centroids; slow enough that
the pure-Python stubs keep up without a GPU. All overridable.

## Integration seam (intentionally not wired yet)

The blob output stops at `centroid_px + depth_m` on purpose. Turning a blob into
an arm pick needs two things this module deliberately leaves out:

1. **Camera intrinsics** — deproject `(u, v, depth_m)` → a 3D point in the
   *camera* frame (pinhole model).
2. **Hand–eye transform** — map the camera-frame point into the robot **base**
   frame.

A future bridge composes those, then hands the base-frame point to
`urctl.Robot.move_tcp`. Because `perception.tools` is shaped exactly like
`urctl.tools`, an agent that both perceives and moves just concatenates the two
registries — that's the planned merge point (see the PickAndStack apple sample
on the control side for the target scenario).
