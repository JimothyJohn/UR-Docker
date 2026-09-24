"""ChArUco calibration board + tool-flange touch probe (parametric, CadQuery).

Build (same toolchain as hardware/d435-tool-bracket):

    uv run --with cadquery --with matplotlib python hardware/charuco-board/board.py
    uv run --with cadquery --with matplotlib python hardware/charuco-board/board.py SQUARE_MM=25 SQUARES=6

Outputs (hardware/charuco-board/out/):
    charuco_plate.{stl,step}      the plate: pattern recess + 3 touch dimples (A, B, C) + magnet pockets
    touch_probe.{stl,step}        ISO-50 spigot probe with a point at a known TCP (for the dimples)
    charuco_pattern.png / .pdf    the printed pattern, exact size (print the PDF at 100 % / "actual size")
    board.json                    every number the calibration code needs (frames, dimples, TCP, dict)
    render_*.png                  sanity renders (matplotlib)

Frames (all mm, right-handed):

  plate frame  == the UR *Plane feature* taught by touching dimple A, then B, then C
                  with the probe: origin at A, +X from A toward B, +Y toward C, +Z out of
                  the printed face.  So `T_base_plate` is read straight off the taught feature.
  pattern frame == OpenCV's CharucoBoard frame (origin at the pattern's top-left corner as
                  printed, x right, y DOWN the page, z INTO the board).  `T_plate_pattern`
                  in board.json converts.  The sheet goes in with the notch at dimple A.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "out"

# ---- pattern (OpenCV ChArUco) -------------------------------------------------
P = dict(
    SQUARES=7,  # squares per side
    SQUARE_MM=20.0,  # chessboard square
    MARKER_MM=15.0,  # ArUco marker inside a square
    DICT="DICT_4X4_50",  # 7x7 -> 24 markers; 4x4_50 has 50 ids
    # ---- plate ------------------------------------------------------------------
    PLATE_T=6.0,  # plate thickness
    MARGIN=20.0,  # plate edge beyond the pattern, each side
    RECESS_CLEAR=0.5,  # recess is pattern + 2*clear
    RECESS_DEPTH=0.6,  # paper/label sits below the face
    NOTCH_R=5.0,  # finger notch at the A corner of the recess (orientation key)
    DIMPLE_OFF=10.0,  # dimple centre outside the pattern corner, both axes
    DIMPLE_D=5.0,  # cone mouth diameter (90 deg cone -> depth = D/2)
    MAGNET_D=10.4,  # underside pockets for 10x2 disc magnets (0 = none)
    MAGNET_T=2.2,
    CHAMFER=1.0,
    # ---- probe ------------------------------------------------------------------
    PROBE_SPIGOT_OD=31.6,  # optional pilot into the bracket's O31.7 recess (used only if H > 0)
    PROBE_SPIGOT_H=0.0,  # 0: flat seating face; prints face-down, point-up, no overhangs
    PROBE_COLLAR_OD=63.0,  # ISO-50 face; 4x O6.6 on O50 PCD so it can be bolted with tool bolts
    PROBE_COLLAR_T=6.0,
    PROBE_TIP_FROM_FACE=44.0,  # point below the face it sits on (bracket top = robot flange + 6)
    BRACKET_T=6.0,  # d435 bracket plate thickness (probe sits on it)
)
for arg in sys.argv[1:]:
    k, v = arg.split("=", 1)
    P[k] = type(P[k])(v) if not isinstance(P[k], str) else v

pattern = P["SQUARES"] * P["SQUARE_MM"]
recess = pattern + 2 * P["RECESS_CLEAR"]
plate_w = pattern + 2 * P["MARGIN"]
# plate frame origin = dimple A; pattern corner (plate coords) is at (DIMPLE_OFF, DIMPLE_OFF)
d = P["DIMPLE_OFF"]
pat0 = (d, d)  # pattern lower-left in plate frame
dimples = {"A": (0.0, 0.0), "B": (pattern + 2 * d, 0.0), "C": (0.0, pattern + 2 * d)}
plate_lo = d - P["MARGIN"]
plate_c = (plate_lo + plate_w / 2, plate_lo + plate_w / 2)


def build_plate() -> cq.Workplane:
    t = P["PLATE_T"]
    wp = (
        cq.Workplane("XY")
        .center(*plate_c)
        .rect(plate_w, plate_w)
        .extrude(t)
        .edges("|Z")
        .fillet(4.0)
        .faces(">Z")
        .edges()
        .chamfer(P["CHAMFER"])
    )
    # pattern recess, centred on the pattern
    pc = (pat0[0] + pattern / 2, pat0[1] + pattern / 2)
    wp = (
        wp.faces(">Z")
        .workplane()
        .center(pc[0] - plate_c[0], pc[1] - plate_c[1])
        .rect(recess, recess)
        .cutBlind(-P["RECESS_DEPTH"])
    )
    # finger notch / orientation key at the A corner of the recess
    notch = (
        cq.Workplane("XY")
        .center(pat0[0] - P["RECESS_CLEAR"], pat0[1] - P["RECESS_CLEAR"])
        .circle(P["NOTCH_R"])
        .extrude(P["RECESS_DEPTH"])
        .translate((0, 0, t - P["RECESS_DEPTH"]))
    )
    wp = wp.cut(notch)
    # 90-degree conical touch dimples
    r = P["DIMPLE_D"] / 2
    for x, y in dimples.values():
        cone = cq.Solid.makeCone(r, 0.0, r, pnt=cq.Vector(x, y, t), dir=cq.Vector(0, 0, -1))
        wp = wp.cut(cq.Workplane("XY").add(cone))
    # underside magnet pockets, one per corner of the plate
    if P["MAGNET_D"] > 0:
        m = P["MARGIN"] / 2
        for sx in (-1, 1):
            for sy in (-1, 1):
                x = plate_c[0] + sx * (plate_w / 2 - m)
                y = plate_c[1] + sy * (plate_w / 2 - m)
                wp = wp.cut(cq.Workplane("XY").center(x, y).circle(P["MAGNET_D"] / 2).extrude(P["MAGNET_T"]))
    # engraved letters next to the dimples (best effort: needs a font on the build host)
    try:
        for name, (x, y) in dimples.items():
            ox = 7.0 if x < pattern / 2 else -7.0
            oy = -7.0 if y < pattern / 2 else 7.0
            txt = (
                cq.Workplane("XY")
                .workplane(offset=t)
                .center(x - ox, y + oy)
                .text(name, 6.0, -0.5, combine=False)
            )
            wp = wp.cut(txt)
    except Exception as e:  # noqa: BLE001
        print(f"note: letter engraving skipped ({e})")
    return wp


def build_probe() -> cq.Workplane:
    # z=0 is the face the probe sits on (the bracket's tool face / the flange face); the
    # tip goes -Z. Located by the 4x O6.6 bolt holes (ISO-50 pattern), not by a pilot, so
    # the seating face is flat and the part prints face-down, point-up, with no overhangs.
    collar = (
        cq.Workplane("XY")
        .circle(P["PROBE_COLLAR_OD"] / 2)
        .extrude(-P["PROBE_COLLAR_T"])
        .faces("<Z")
        .workplane()
        .polygon(4, 50.0)
        .vertices()
        .hole(6.6)
    )
    tip_len = P["PROBE_TIP_FROM_FACE"] - P["PROBE_COLLAR_T"]
    cone = cq.Solid.makeCone(
        6.0, 0.0, tip_len, pnt=cq.Vector(0, 0, -P["PROBE_COLLAR_T"]), dir=cq.Vector(0, 0, -1)
    )
    probe = collar.union(cq.Workplane("XY").add(cone))
    if P["PROBE_SPIGOT_H"] > 0:  # optional pilot (needs supports or a flipped print)
        spigot = cq.Workplane("XY").circle(P["PROBE_SPIGOT_OD"] / 2).extrude(P["PROBE_SPIGOT_H"])
        probe = probe.union(spigot.edges(">Z").chamfer(0.5))
    # bolt pattern 45 deg off the axes like the flange (holes at 45/135/225/315)
    return probe.rotate((0, 0, 0), (0, 0, 1), 45)


def pattern_images() -> dict:
    import cv2

    aruco = cv2.aruco
    dic = aruco.getPredefinedDictionary(getattr(aruco, P["DICT"]))
    n = P["SQUARES"]
    board = aruco.CharucoBoard((n, n), P["SQUARE_MM"] / 1000, P["MARKER_MM"] / 1000, dic)
    px_per_mm = 600 / 25.4
    side = int(round(pattern * px_per_mm))
    img = board.generateImage((side, side), marginSize=0, borderBits=1)
    # add a white border with an orientation legend so the sheet is cut/placed unambiguously
    margin = int(round(8 * px_per_mm))
    canvas = 255 * __import__("numpy").ones((side + 2 * margin, side + 2 * margin), dtype="uint8")
    canvas[margin : margin + side, margin : margin + side] = img
    cv2.rectangle(canvas, (margin, margin), (margin + side - 1, margin + side - 1), 0, 2)  # cut line
    f = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(canvas, "A (notch)  ->", (margin, canvas.shape[0] - margin // 3), f, 1.2, 0, 3)
    cv2.putText(
        canvas,
        f"charuco {n}x{n} sq {P['SQUARE_MM']:g}mm mk {P['MARKER_MM']:g}mm {P['DICT']}  print at 100%",
        (margin, margin // 2 + 10),
        f,
        0.9,
        0,
        2,
    )
    cv2.putText(canvas, "B", (margin + side - 40, canvas.shape[0] - margin // 3), f, 1.2, 0, 3)
    cv2.putText(canvas, "C", (margin // 4, margin + 40), f, 1.2, 0, 3)
    cv2.imwrite(str(OUT / "charuco_pattern.png"), canvas)
    # exact-size PDF (figure size in inches == physical size)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    w_in = canvas.shape[1] / px_per_mm / 25.4
    h_in = canvas.shape[0] / px_per_mm / 25.4
    fig = plt.figure(figsize=(w_in, h_in), dpi=600)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")
    ax.imshow(canvas, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
    fig.savefig(OUT / "charuco_pattern.pdf", dpi=600)
    plt.close(fig)
    return {"sheet_mm": [canvas.shape[1] / px_per_mm, canvas.shape[0] / px_per_mm], "cut_to_mm": pattern}


def render(shape: cq.Workplane, name: str, elev=35, azim=-60):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    verts, tris = shape.val().tessellate(0.3, 0.5)
    v = np.array([[p.x, p.y, p.z] for p in verts])
    t = np.array(tris)
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_trisurf(v[:, 0], v[:, 1], v[:, 2], triangles=t, color="#c8c8c8", edgecolor="#555", linewidth=0.15)
    lo, hi = v.min(0), v.max(0)
    c = (lo + hi) / 2
    r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(name)
    fig.savefig(OUT / f"render_{name}.png", dpi=110)
    plt.close(fig)


def main():
    OUT.mkdir(exist_ok=True)
    plate = build_plate()
    probe = build_probe()
    cq.exporters.export(plate, str(OUT / "charuco_plate.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(plate, str(OUT / "charuco_plate.step"))
    cq.exporters.export(probe, str(OUT / "touch_probe.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(probe, str(OUT / "touch_probe.step"))
    sheet = pattern_images()
    render(plate, "plate_top", elev=55, azim=-50)
    render(plate, "plate_bottom", elev=-50, azim=-50)
    render(probe, "probe", elev=20, azim=-60)
    tip_from_flange = P["PROBE_TIP_FROM_FACE"] + P["BRACKET_T"]
    spec = {
        "params": P,
        "pattern": {
            "squares": [P["SQUARES"], P["SQUARES"]],
            "square_m": P["SQUARE_MM"] / 1000,
            "marker_m": P["MARKER_MM"] / 1000,
            "dictionary": P["DICT"],
            "side_mm": pattern,
            **sheet,
        },
        "plate_frame": "origin = dimple A, +X toward B, +Y toward C, +Z out of the printed face "
        "(== a UR Plane feature taught A, B, C)",
        "dimples_plate_mm": dimples,
        "dimple_cone_depth_mm": P["DIMPLE_D"] / 2,
        "plate_size_mm": [plate_w, plate_w, P["PLATE_T"]],
        # OpenCV CharucoBoard frame: origin top-left of the print, x right, y down, z into the board.
        # Printed with the notch (A) at the bottom-left => pattern top-left sits at plate (d, d+pattern).
        "T_plate_pattern": {
            "t_mm": [pat0[0], pat0[1] + pattern, 0.0],
            "R": [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
            "note": "180 deg about X: pattern y (down the page) = plate -Y, "
            "pattern z (into board) = plate -Z",
        },
        "probe": {
            "tip_below_seating_face_mm": P["PROBE_TIP_FROM_FACE"],
            "tcp_on_bracket_m": [0, 0, tip_from_flange / 1000, 0, 0, 0],
            "tcp_on_bare_flange_m": [0, 0, P["PROBE_TIP_FROM_FACE"] / 1000, 0, 0, 0],
            "note": "flat seating face; locate with the 4x O6.6 on the ISO-50 pattern (the tool bolts) "
            "against the bracket's tool face or the bare flange",
        },
    }
    (OUT / "board.json").write_text(json.dumps(spec, indent=2))
    print(json.dumps({k: v for k, v in spec.items() if k != "params"}, indent=1))


if __name__ == "__main__":
    main()
