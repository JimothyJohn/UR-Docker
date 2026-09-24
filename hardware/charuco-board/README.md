# ChArUco calibration plate + touch probe

**Parts:** `charuco_plate` (180 × 180 × 6 mm) · `touch_probe` (ISO-50 spigot probe, tip 44 mm below its seating face) · `charuco_pattern.pdf` (print at 100 % / "actual size", cut on the black line, drop into the recess with the **A corner at the notch**) · **Rev:** A (2026-09-24) · **Status:** designed, exported, *not yet printed*
**Source:** `board.py` (parametric, CadQuery) · **Outputs:** `out/` · **Build:** `uv run --with cadquery --with matplotlib python hardware/charuco-board/board.py` (`KEY=value` overrides, see `P` in the script)

Purpose: the one-minute, hand-aided calibration for the monocular scan (`docs/mono-scan.md`). The plate carries a printed ChArUco pattern (what the camera sees) and three 90° conical touch dimples A, B, C (what the robot touches). Teaching a UR **Plane feature** by touching A, B, C in order with the probe makes the feature *be* the plate frame, so no math is needed to get the board into base coordinates.

| Item | Value |
| --- | --- |
| Pattern | 7 × 7 squares, 20 mm square, 15 mm marker, `DICT_4X4_50`, 140 × 140 mm (24 markers, 36 inner corners) |
| Recess | 141 × 141 × 0.6 mm, Ø10 finger notch at the A corner (orientation key) |
| Dimples (plate frame, mm) | A (0, 0) · B (160, 0) · C (0, 160); Ø5 mouth, 90° cone, 2.5 mm deep |
| Plate frame | origin A, +X toward B, +Y toward C, +Z out of the printed face == the taught Plane feature |
| Pattern frame (OpenCV) | origin at the print's top-left, x right, y down the page, z into the board; `T_plate_pattern` = translate (10, 150, 0) then 180° about X (`out/board.json`) |
| Magnets | 4 × underside pockets Ø10.4 × 2.2 for 10 × 2 discs (steel table); `MAGNET_D=0` to drop |
| Probe | Ø63 × 6 collar with a **flat seating face** and 4 × Ø6.6 on Ø50 PCD (the tool bolts locate it), cone to a point. No pilot (`PROBE_SPIGOT_H=0`; set it > 0 to add one) so it prints face-down, point-up, no overhangs |
| Probe TCP | `[0, 0, 0.050]` seated on the D435 bracket (6 mm plate), `[0, 0, 0.044]` on the bare flange |

Print the plate flat, printed face up; print the probe on its seating face, point up. No supports on either (the dimples are 90° cones, the probe's taper is ~9° off vertical). PLA is fine; the plate is a reference, not a load part. Check the print: the pattern must measure 140.0 mm across; the cut sheet must drop into the recess without bowing. `board.json` is what the calibration code reads.
