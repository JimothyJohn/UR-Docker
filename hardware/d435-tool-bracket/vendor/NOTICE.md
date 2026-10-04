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
| Regenerate | `python3 hardware/d435-tool-bracket/bracket.py --refresh-camera-mesh` |

Intel also publishes native CAD (SolidWorks `D435_Solid.SLDPRT` in the "D400
Depth Cameras" archive at https://dev.realsenseai.com/docs/cad-files/); there is
no STEP in that archive, which is why the mesh is used here.

Modifications to the mesh: decimation only. No other changes.

## `uf850_link6_xarm_ros2.stl` — UFACTORY 850 wrist (link 6) with its tool flange

UFACTORY's own visual mesh of the 850's last link, used to check the `uf850` print
against the real wrist (`robot_clash` in `bracket.py`: no vertex of the wrist may be
inside the bracket) and to draw the 850 renders. It is *not* a runtime dependency of
anything.

| | |
| --- | --- |
| Source | `xArm-Developer/xarm_ros2`, `xarm_description/meshes/uf850/visual/link6.stl` |
| Commit | `62936f7ea1846a85f7350de2c4c18f39e6d19715` (2026-08-11) |
| URL | https://raw.githubusercontent.com/xArm-Developer/xarm_ros2/62936f7ea1846a85f7350de2c4c18f39e6d19715/xarm_description/meshes/uf850/visual/link6.stl |
| SHA-256 | `d031adf90d7254b358ea922f8dfe3308cebec0852ba056e122a34353cc8ea9e3` (this copy is byte-identical) |
| Frame | link6 = UFACTORY's flange frame: origin on the flange face, +Z out of it, metres. The Ø6 dowel is at −Y; the bracket's frame has it at +Y, so `bracket.py` turns the mesh 180° about Z (`UF850_ROBOT_FLANGE_DEG`) and scales to mm. |
| License | BSD 3-Clause, reproduced below |

Modifications to the mesh: none.

```
Copyright (c) 2018, UFACTORY Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
