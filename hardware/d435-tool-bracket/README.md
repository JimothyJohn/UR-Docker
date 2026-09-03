# D435 tool-flange bracket — specification / datasheet

**Part:** `d435_tool_bracket` · **Rev:** A (2026-09-02) · **Status:** designed, exported, *not yet printed or test-fitted*
**Files:** `bracket.py` (parametric source, CadQuery) · `out/d435_tool_bracket.stl` (print) · `out/d435_tool_bracket.step` (edit) · `out/d435_tool_bracket_assembly.step` (flange + bracket + camera + hardware) · `renders/*.png`

A one-piece 90° bracket that bolts an Intel RealSense D435 to the ISO 9409-1-50-4-M6 tool flange of a UR e-Series arm, camera looking along the flange +Z (away from the flange, the tool direction). It is a *sandwich* plate: the gripper's own ISO-50 pattern is re-presented on top, so the camera rides between the wrist and the end effector.

![ISO close-up](renders/iso_closeup.png)

| | |
| --- | --- |
| ![top](renders/top_xy.png) | ![side](renders/side_xz.png) |

## 1. Requirements (what this part must do)

| ID | Requirement | How it is met | Verified |
| --- | --- | --- | --- |
| R1 | Mount to a UR e-Series ISO 9409-1-50-4-M6 tool flange | Ø63 plate, 4× Ø6.6 on Ø50 PCD at 45°/135°/225°/315°, Ø31.3 spigot into the Ø31.5 H7 recess | dims from UR10e manual §8.7.5 |
| R2 | Camera optical axis parallel to flange +Z, looking away from the flange | camera bottom face on a wall parallel to Z; front face flush with the wall top so the wall never enters the FOV | by construction (`derived()`) |
| R3 | Simple 90° bracket, arm projecting ~25 mm past the flange so the camera screw is reachable | `OVERHANG = 25`: 25 mm radial gap between flange edge and wall inner face; the 1/4-20 head lives in that gap | render `side_xz.png` |
| R4 | Cable exit unobstructed | nothing touches the camera except its mounting face; back face and both ends are free (5 mm `BACK_CLEAR` above the plate) | see §6 assumption A1 |
| R5 | Centre/align on the flange so it is secure and repeatable | Ø31.3 spigot (centring) + radial Ø6.2 slot at the dowel position (clocking, UR-recommended slot) | needs test fit (spigot allowance) |
| R6 | Printable in PPA-CF today; features simple enough for mass manufacture | all features are Z-extrusions or radial holes, no undercuts except the coaxial spigot/recess; mouldable with a single draw + one side action | — |
| R7 | Keep the gripper mountable | ISO-50 pattern passes through; top recess Ø31.7 × 5 and dowel slot through, so a gripper spigot + pin seat normally | — |
| R8 | Others can iterate | every number is a named parameter in `bracket.py`; STL/STEP/renders regenerate from one command | this file |

## 2. Interfaces (the numbers that are not ours)

### 2.1 Robot: UR e-Series tool flange (UR10e User Manual SW 5.19, §8.7.5 "Securing Tool")

| Feature | Value | Note |
| --- | --- | --- |
| Standard | ISO 9409-1-50-4-M6 | identical on UR3e/5e/10e/16e |
| Flange face | Ø63 h8, proud of the Ø90 wrist by 6.50 | plate OD matches |
| Bolt circle | Ø50 ±0.1, 4× M6-6H, thread depth 8 | at 45° from the dowel, 4×90° |
| Dowel hole | Ø6 H7, depth 6.20 ±0.20, on the PCD at 12 o'clock | same side as the Lumberg tool-I/O connector |
| Centring recess | Ø31.50 H7 | depth not dimensioned; spigot kept to 4 mm |
| Bolt length | "do not use bolts that extend beyond 10 mm" into the flange | ⇒ ≤ 8 mm engagement (thread depth) |
| Pin locating | UR: "use a radially slotted hole for the positioning pin to avoid over-constraining" | done |

### 2.2 Camera: Intel RealSense D435 (D400 Series Datasheet 337029-017, Fig. 10-9)

| Feature | Value |
| --- | --- |
| Envelope | 90 × 25 × 25.05 mm |
| Tripod thread | 1/4-20 UNC at the bottom centre (45 from each end, centred across the depth) |
| M3 mounting points | 2×, 45 mm apart on the same centreline, **max insertion 3 mm, 0.4 Nm** |
| Left imager | 17.5 mm from the 1/4-20 centreline along the length; stereo baseline 50 mm |
| Depth FOV (HxV) | 87° × 58° (D435), RGB 69° × 42° |
| Mass | 72 g (Intel spec page) |
| Connector | USB-C, on the body — **face not dimensioned in the drawing** (see A1) |

## 3. Geometry (all mm, from `bracket.py` PARAMS → `derived()`)

Frame: origin = centre of the flange face, +Z = away from the flange, +Y = towards the dowel hole, +X = arm direction (`ARM_ANGLE_DEG = 0`; rotate in 90° steps to clock the camera against the tool-I/O cable — the dowel slot follows the *robot*, not the arm).

| Item | Value | Parameter(s) |
| --- | --- | --- |
| Plate | Ø63 × 8 | `FLANGE_OD`, `PLATE_T` |
| Bolt holes | 4× Ø6.6 thru, Ø50 PCD | `BOLT_HOLE_D`, `PCD`, `BOLT_ANGLES_DEG` |
| Dowel slot | 6.2 wide × 9 long, radial, thru, at +Y on the PCD | `DOWEL_SLOT_W/L`, `DOWEL_ANGLE_DEG` |
| Spigot (bottom) | Ø31.3 × 4, ID 24, 0.8 chamfer | `SPIGOT_OD/H/CHAMFER` |
| Recess (top) | Ø31.7 × 5 | `TOP_RECESS_D/H` |
| Centre hole | Ø24 thru (cables / air) | `CENTER_HOLE_D` |
| Arm slab | 60 wide (Y) × 8 thick, X = 15 … 62.5 | `ARM_W`, `OVERHANG`, `WALL_T` |
| Wall | X = 56.5 … 62.5, 60 wide, Z = 0 … **38.0** | `WALL_T`, `BACK_CLEAR`, `CAM_D` |
| Camera fastener line | Z = **25.5**, Y = 0 (Ø6.6) and Y = ±22.5 (Ø3.4) through the wall | `TRIPOD_HOLE_D`, `M3_HOLE_D`, `M3_SPACING` |
| Gussets | 2×, 4 thick at Y = ±28…±30, 18 along X, 22 up the wall | `GUSSET_T/L/H` |
| Camera envelope | X = 62.5 … 87.5, Y = ±45, Z = 13.0 … 38.0 | — |
| Radial extent | 87.5 from the flange axis | `radial_extent` |
| Bracket bbox / volume / mass | 94 × 63 × 42 mm · 49.1 cm³ · ≈ 59 g solid PPA-CF (infill lowers it) | `out/build_info.json` |

**Nominal camera pose (hand-eye seed).** Camera axes in flange axes: `x_cam = +Y`, `y_cam = −X` (image-down points at the mounting wall), `z_cam = +Z`. Depth origin (left imager) ≈ **(75.0, −17.5, 35.0) mm** in the flange frame for `ARM_ANGLE_DEG = 0` (lens plane taken 3 mm behind the front face, `LENS_SETBACK`). Use it to seed `T_flange_camera`; calibrate to finish — a printed part will not hold ±1°.

## 4. Hardware (BOM)

| Qty | Item | Spec / note |
| --- | --- | --- |
| 4 | M6 × 14 socket head cap screw, 12.9 | 6 mm engagement in the flange (thread depth 8, UR limit 10). **With a gripper on top, use the gripper's bolts instead, 8 mm longer than the gripper alone needs** — the bracket's own bolt heads would sit between the plate and the gripper. |
| 4 | M6 flat washer, DIN 125 | spreads head load on the plastic (see §5 torque) |
| 1 | Dowel pin Ø6 m6 × 20 | 6 into the flange hole, through the plate slot, 6 proud for the gripper's pin hole |
| 1 | 1/4-20 UNC × 1/2" (12.7) hex socket + washer | through the 6 mm wall, ≈ 5–6 mm into the camera. Intel's D435 drawing gives no max insertion for the 1/4-20; the D455 says 9 mm. Do not exceed 6 mm. A knurled thumbscrew here makes camera swaps tool-free. |
| 2 | M3 × 8 SHCS (optional) | anti-rotation; 2 mm into the camera (**max 3 mm**, 0.4 Nm per Intel) |
| — | Thread-locker (medium) on the M6 | plastic joints relax; re-torque after 24 h |

## 5. Material, printing, torque

- **Material:** PPA-CF (e.g. Bambu PPA-CF / Polymaker Fiberon PPA-CF). Dry the spool (≥ 8 h at 80–100 °C) — PPA prints wet look fine and are 30–50 % weaker. Hardened 0.4 nozzle, ~300–320 °C, bed 80–100 °C, enclosure, slow first layer. Anneal per the filament datasheet if the part will see > 80 °C (cell lights, motors). Alternative that prints anywhere: PA6-CF or PET-CF; then double `GUSSET_T`.
- **Orientation (recommended): camera face on the bed** (wall lying flat, arm and plate standing up). Everything is then supported — the spigot ring becomes a 4 mm horizontal protrusion with its 0.8 chamfer, the Ø24 bore is a horizontal hole (bridges fine at Ø24), fastener holes in the wall become vertical holes. Layers run parallel to the wall, so the wall-to-plate corner is loaded across layers only in the plate; the loads are tiny (a 72 g camera on an 87 mm lever ≈ 0.06 N·m) and the section is 60 × 8 mm.
- **Alternative orientation:** flange face down with supports under the plate (a Ø63 annulus at 4 mm above the bed). Better spigot, worse mating face — sand flat before use.
- **Walls/infill:** 5 perimeters, 40 % gyroid, 0.2 mm layers. The plate should be near-solid under the bolt heads; 5 perimeters + top/bottom 6 layers gets there.
- **Fit tuning:** print once, measure. `SPIGOT_OD` (31.3) should slip into the Ø31.5 H7 recess with light hand pressure; if it binds, `SPIGOT_OD=31.2`. `HOLE_PRINT_ALLOWANCE` adds to every hole if your printer undersizes holes (typical 0.1–0.2 with CF filaments).
- **Torque (plastic joint):** M6 with washer: **3 N·m** (≈ 2.4 kN clamp per bolt, ~30 MPa under the washer — comfortably inside PPA-CF's compressive range; a bare head at full 12.9 torque would creep). 1/4-20: 1.5 N·m. M3: 0.4 N·m (Intel).

## 6. Design considerations and assumptions

1. **A1 — USB-C location.** Intel's Fig. 10-9 does not dimension the connector face. The bracket touches only the camera's mounting face and leaves the back face and both ends open, with the back face 5 mm above the plate and radially outboard of the plate edge, so a straight plug on the back (exiting −Z) or on an end (exiting ±Y) both clear. If the plug turns out to point at the plate, raise `BACK_CLEAR`. **Check on the unit before printing a second one.**
2. **A2 — spigot depth.** UR's drawing does not state the recess depth; ISO 9409-1 recesses are ≥ 6 mm on the 50 flange. The 4 mm spigot leaves margin. If the plate rocks, the spigot is bottoming: shorten `SPIGOT_H`.
3. **Screw access (R3).** The 1/4-20 head sits in the 25 mm gap between the flange edge and the wall, 25.5 mm above the flange face. With the top of the plate free (no gripper yet) a driver comes straight in along −X. With a gripper mounted, use a thumbscrew or a ball-end key from above. **Sequence: bracket → camera → gripper.**
4. **Field of view.** The wall top is coplanar with the camera's front face, so no bracket geometry is in the 87° × 58° cone. The gripper will be: the camera looks along +Z from x ≈ 75 mm, y ≈ −17.5 mm — the gripper fingers appear at the bottom of the image at close range. That is normal eye-in-hand; mask them in perception if needed.
5. **Why a sandwich, not a side clamp.** Anything clamped around the wrist or gripper is a friction joint that walks under acceleration and ruins calibration. The ISO pattern is the only datum on the robot with a tolerance (Ø50 ±0.1, H7 recess, H7 pin) — use it.
6. **Why the slot is radial.** The flange's Ø31.5 H7 recess and the Ø6 H7 pin hole *both* define position; a round pin hole in the tool over-constrains and either binds or bends the pin. UR says slot it radially; the slot locates angle only.
7. **Stiffness.** The bracket is far stiffer than needed for a 72 g camera; the gussets exist for the *print* (they brace the wall–plate corner across layer lines) and for handling. If mass matters, `GUSSET_H` can drop to 12 and `ARM_W` to 52 (still covers the M3 pair).
8. **Mass manufacturing.** Every feature is a straight extrusion along Z or a straight hole along X. For injection moulding: single Z draw for the plate/wall/gussets, one side action for the three wall holes (or drill them). For CNC: 2 setups (top/bottom) + one side op. Add 1° draft on the wall and gussets for moulding (`bracket.py` has no draft on purpose — printed parts don't need it).
9. **PoE later.** The future PoE camera swap changes only §2.2; the wall parameters are the only ones that reference the camera.
10. **Safety.** The camera adds ~150 g at 87 mm and 38 mm from the flange; include the bracket + camera in the UR payload/CoG settings (`Installation → Payload`) or the joint torque model drifts and protective stops follow.

## 7. Verification checklist (fill in after the first print)

- [ ] Spigot seats in the Ø31.5 recess without rocking; note the measured spigot OD and the flange recess depth
- [ ] Dowel pin passes the slot; bracket cannot rotate on the flange
- [ ] 4× M6 × 14 enter freely and torque to 3 N·m; re-torque after 24 h
- [ ] 1/4-20 engages ≤ 6 mm; M3 ≤ 3 mm (check with a depth gauge before screwing in)
- [ ] USB-C plug seats with the cable straight; no contact with the plate (A1)
- [ ] `perception rs-info` sees the camera; `perception gui` shows the flange edge/gripper where §6.4 predicts
- [ ] Hand-eye calibration result vs. the nominal (75, −17.5, 35) mm / axis map in §3
- [ ] Payload updated on the pendant

## 8. Regenerating

```bash
# in a scratch venv (cadquery is a design-time tool, not a runtime dependency of this repo)
uv venv /tmp/cadenv --python 3.12 && uv pip install --python /tmp/cadenv/bin/python cadquery matplotlib
/tmp/cadenv/bin/python hardware/d435-tool-bracket/bracket.py                 # defaults
/tmp/cadenv/bin/python hardware/d435-tool-bracket/bracket.py SPIGOT_OD=31.2 ARM_ANGLE_DEG=180
```

Outputs land in `out/` (STL, STEP, assembly STEP, `build_info.json` with the derived numbers) and `renders/`. Sources for every external number are cited in `bracket.py`'s docstring.
