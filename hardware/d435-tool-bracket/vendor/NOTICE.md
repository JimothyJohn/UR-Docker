# Vendored third-party geometry

## `d435_realsense-ros_12k.stl` — Intel RealSense D435 body

Decimated copy of Intel's own D435 visual mesh from the `realsense-ros` package,
used for the assembly renders and exported (placed in the flange frame) as
`out/d435_body_in_flange_frame.stl`. It is *not* a runtime dependency of anything.

| | |
| --- | --- |
| Source | `IntelRealSense/realsense-ros`, `realsense2_description/meshes/d435.dae` |
| Commit | `9189b0591fae677570d2dfd0ae8957fb6f12e635` (2021-01-18) |
| URL | https://raw.githubusercontent.com/IntelRealSense/realsense-ros/9189b0591fae677570d2dfd0ae8957fb6f12e635/realsense2_description/meshes/d435.dae |
| SHA-256 of the source | `42f3b66f47a1f8f425a2e4dc07c1d9c283183167d8441f520a15623d98f9bf78` |
| License | Apache License 2.0 — Copyright Intel Corporation (see the repository's `LICENSE`) |
| Processing | COLLADA → triangles (metres → mm), quadric decimation 231 186 → 12 000 triangles (`fast-simplification`, `agg=7`). Frame unchanged: x = camera-left, y = up, z = forward, origin on the front plate at mid-height, on the tripod hole's length position. |
| Regenerate | `uv run --with numpy --with fast-simplification python hardware/d435-tool-bracket/bracket.py --refresh-camera-mesh` |

Intel also publishes native CAD (SolidWorks `D435_Solid.SLDPRT` in the "D400
Depth Cameras" archive at https://dev.realsenseai.com/docs/cad-files/); there is
no STEP in that archive, which is why the mesh is used here.

Modifications to the mesh: decimation only. No other changes.
