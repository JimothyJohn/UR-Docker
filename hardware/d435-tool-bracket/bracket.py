"""D435 tool-flange bracket — parametric CAD (CadQuery), STL/STEP export and
assembly renders. Everything dimensional lives in the PARAMS block; the spec
(README.md next to this file) is generated from the same numbers so the two
cannot drift.

    uv run --with cadquery --with matplotlib python hardware/d435-tool-bracket/bracket.py

Frame: origin at the centre of the robot's tool-flange face, +Z away from the
flange (the tool direction, and the camera's optical axis), +Y towards the
Ø6 H7 dowel hole (12 o'clock, same side as the UR tool-I/O connector), +X
completes the right-handed set. The camera arm lies along +X by default
(``ARM_ANGLE_DEG`` rotates it).

Sources (verified 2026-09-02):
  * UR10e User Manual (SW 5.19), §8.7.5 "Securing Tool": Ø63 h8 face, Ø50 ±0.1
    PCD, 4× M6-6H ▽8 at 45° from the dowel, Ø6 H7 ▽6.20 dowel hole at 12
    o'clock, Ø31.50 H7 centring recess, Ø90 wrist, flange proud of the wrist
    by 6.50; "do not use bolts that extend beyond 10 mm"; UR recommends a
    radially slotted hole for the positioning pin.
  * Intel RealSense D400 Series Datasheet 337029-017, Fig. 10-9 (D435/D435i):
    90 × 25 × 25.05 mm, 1/4-20 at the bottom centre, 2× M3 mounting points
    45 mm apart (max insertion 3 mm, 0.4 Nm), left imager 17.5 mm from the
    1/4-20 centreline, 50 mm stereo baseline.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

# ----------------------------------------------------------------------------
# PARAMS — the only place with numbers. Units: mm, degrees.
# ----------------------------------------------------------------------------
PARAMS = {
    # -- robot side: ISO 9409-1-50-4-M6 as built on UR e-Series ----------------
    "FLANGE_OD": 63.0,  # Ø63 h8 flange face; the plate matches it
    "PCD": 50.0,  # bolt circle
    "BOLT_HOLE_D": 6.6,  # M6 clearance (ISO 273 medium)
    "BOLT_ANGLES_DEG": (45.0, 135.0, 225.0, 315.0),  # 45° from the dowel at 90°
    "DOWEL_ANGLE_DEG": 90.0,  # Ø6 H7 hole sits at 12 o'clock on the PCD
    "DOWEL_SLOT_W": 6.2,  # Ø6 pin + 0.2 fit
    "DOWEL_SLOT_L": 9.0,  # radial slot (UR: slot radially to avoid over-constraint)
    "SPIGOT_OD": 31.3,  # into the Ø31.5 H7 recess; 0.2 print allowance (tune)
    "SPIGOT_H": 4.0,  # recess is deeper; 4 mm engagement leaves margin
    "SPIGOT_CHAMFER": 0.8,
    "TOP_RECESS_D": 31.7,  # the same Ø31.5 pattern re-presented to the gripper
    "TOP_RECESS_H": 5.0,
    "CENTER_HOLE_D": 24.0,  # cable / air pass-through
    "PLATE_T": 8.0,
    # -- the arm and the camera wall ----------------------------------------------
    "ARM_ANGLE_DEG": 0.0,  # 0 = camera on +X, i.e. 90° from the dowel/tool-I/O side
    "ARM_W": 60.0,  # tangential width (covers the M3 pair at ±22.5 with margin)
    "OVERHANG": 25.0,  # radial gap flange-edge → wall inner face (screw-head access)
    "WALL_T": 6.0,
    "BACK_CLEAR": 5.0,  # camera back face above the plate top (cable/plug clearance)
    "GUSSET_T": 4.0,
    "GUSSET_L": 18.0,  # along X on the plate
    "GUSSET_H": 22.0,  # up the wall, stays below the M3/tripod line
    # -- camera: Intel RealSense D435 -------------------------------------------------
    "CAM_L": 90.0,
    "CAM_H": 25.0,  # along the wall face (tangential) is the length; this is the height
    "CAM_D": 25.05,
    "TRIPOD_HOLE_D": 6.6,  # 1/4-20 UNC clearance
    "M3_HOLE_D": 3.4,
    "M3_SPACING": 45.0,
    "IMAGER_OFFSET": 17.5,  # 1/4-20 centreline → left imager, along the length
    "BASELINE": 50.0,
    "LENS_SETBACK": 3.0,  # optical centre behind the front face (estimate)
    # -- printing allowances --------------------------------------------------------
    "HOLE_PRINT_ALLOWANCE": 0.0,  # add to every hole diameter if your printer undersizes
}

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
RENDERS = HERE / "renders"


def derived(p: dict) -> dict:
    """Numbers that follow from PARAMS (also written into the spec)."""
    r_flange = p["FLANGE_OD"] / 2
    wall_in = r_flange + p["OVERHANG"]
    wall_out = wall_in + p["WALL_T"]
    wall_h = p["PLATE_T"] + p["BACK_CLEAR"] + p["CAM_D"]
    tripod_z = wall_h - p["CAM_D"] / 2  # 1/4-20 is centred across the camera depth
    cam_x0 = wall_out
    cam_x1 = wall_out + p["CAM_H"]
    lens_x = wall_out + p["CAM_H"] / 2  # lenses on the height centreline
    lens_z = wall_h - p["LENS_SETBACK"]
    # camera frame in flange coords: +z_cam = +Z, +y_cam (image down, toward the
    # camera's mounting face) = -X, hence +x_cam = +Y. Left imager is at -x_cam.
    depth_origin = (lens_x, -p["IMAGER_OFFSET"], lens_z)
    return {
        "wall_inner_x": wall_in,
        "wall_outer_x": wall_out,
        "wall_h": wall_h,
        "wall_top_z": wall_h,
        "tripod_z": tripod_z,
        "camera_x_range": (cam_x0, cam_x1),
        "camera_z_range": (wall_h - p["CAM_D"], wall_h),
        "camera_back_face_z": wall_h - p["CAM_D"],
        "radial_extent": cam_x1,
        "depth_origin_flange_mm": depth_origin,
        "R_flange_to_cam": [[0, 1, 0], [-1, 0, 0], [0, 0, 1]],  # columns = x_cam, y_cam, z_cam in flange axes
        "arm_angle_deg": p["ARM_ANGLE_DEG"],
    }


# ----------------------------------------------------------------------------
# geometry
# ----------------------------------------------------------------------------


def build_bracket(p: dict):
    import cadquery as cq

    d = derived(p)
    T = p["PLATE_T"]
    ha = p["HOLE_PRINT_ALLOWANCE"]

    # plate disc + arm slab (one extrusion each, unioned)
    plate = cq.Workplane("XY").circle(p["FLANGE_OD"] / 2).extrude(T)
    arm = (
        cq.Workplane("XY")
        .center((d["wall_outer_x"] + 15) / 2, 0)
        .rect(d["wall_outer_x"] - 15, p["ARM_W"])
        .extrude(T)
    )
    body = plate.union(arm)

    # the wall
    wall = (
        cq.Workplane("XY")
        .center((d["wall_inner_x"] + d["wall_outer_x"]) / 2, 0)
        .rect(p["WALL_T"], p["ARM_W"])
        .extrude(d["wall_h"])
    )
    body = body.union(wall)

    # gussets: right triangles in the XZ plane at both Y extremes
    gl, gh, gt = p["GUSSET_L"], p["GUSSET_H"], p["GUSSET_T"]
    x1 = d["wall_inner_x"]
    for y in (p["ARM_W"] / 2 - gt / 2, -(p["ARM_W"] / 2 - gt / 2)):
        g = (
            cq.Workplane("XZ", origin=(0, y + gt / 2, 0))
            .polyline([(x1 - gl, T), (x1, T), (x1, T + gh)])
            .close()
            .extrude(gt)
        )
        body = body.union(g)

    # bottom spigot ring (into the flange recess), chamfered lead-in
    spig = (
        cq.Workplane("XY", origin=(0, 0, -p["SPIGOT_H"]))
        .circle(p["SPIGOT_OD"] / 2)
        .circle(p["CENTER_HOLE_D"] / 2)
        .extrude(p["SPIGOT_H"])
    )
    try:
        spig = spig.faces("<Z").edges(cq.selectors.RadiusNthSelector(1)).chamfer(p["SPIGOT_CHAMFER"])
    except Exception:  # chamfer is cosmetic; never fail the build on it
        pass
    body = body.union(spig)

    # through features: centre hole, top recess, bolt holes, dowel slot
    body = body.cut(cq.Workplane("XY", origin=(0, 0, -20)).circle(p["CENTER_HOLE_D"] / 2).extrude(80))
    body = body.cut(
        cq.Workplane("XY", origin=(0, 0, T - p["TOP_RECESS_H"])).circle(p["TOP_RECESS_D"] / 2).extrude(20)
    )
    r = p["PCD"] / 2
    for a in p["BOLT_ANGLES_DEG"]:
        x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        body = body.cut(
            cq.Workplane("XY", origin=(x, y, -20)).circle((p["BOLT_HOLE_D"] + ha) / 2).extrude(80)
        )
    da = math.radians(p["DOWEL_ANGLE_DEG"])
    slot = (
        cq.Workplane("XY", origin=(0, 0, -20))
        .center(r * math.cos(da), r * math.sin(da))
        .slot2D(p["DOWEL_SLOT_L"], p["DOWEL_SLOT_W"] + ha, angle=p["DOWEL_ANGLE_DEG"])
        .extrude(80)
    )
    body = body.cut(slot)

    # camera fastener holes through the wall (axis = X)
    tz = d["tripod_z"]
    body = body.cut(
        cq.Workplane("YZ", origin=(d["wall_inner_x"] - 5, 0, 0))
        .center(0, tz)
        .circle((p["TRIPOD_HOLE_D"] + ha) / 2)
        .extrude(p["WALL_T"] + 10)
    )
    for y in (-p["M3_SPACING"] / 2, p["M3_SPACING"] / 2):
        body = body.cut(
            cq.Workplane("YZ", origin=(d["wall_inner_x"] - 5, 0, 0))
            .center(y, tz)
            .circle((p["M3_HOLE_D"] + ha) / 2)
            .extrude(p["WALL_T"] + 10)
        )

    if p["ARM_ANGLE_DEG"]:
        body = body.rotate((0, 0, 0), (0, 0, 1), p["ARM_ANGLE_DEG"])
    return body


def build_flange_standin(p: dict):
    """UR e-Series wrist-3 + tool flange, dimensions from the UR10e manual."""
    import cadquery as cq

    wrist = cq.Workplane("XY", origin=(0, 0, -6.5 - 48.75)).circle(45.0).extrude(48.75)
    face = cq.Workplane("XY", origin=(0, 0, -6.5)).circle(p["FLANGE_OD"] / 2).extrude(6.5)
    f = wrist.union(face)
    f = f.cut(cq.Workplane("XY", origin=(0, 0, -6.5)).circle(31.5 / 2).extrude(6.5))  # recess
    r = p["PCD"] / 2
    for a in p["BOLT_ANGLES_DEG"]:
        x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        f = f.cut(cq.Workplane("XY", origin=(x, y, -8)).circle(2.5).extrude(8))
    da = math.radians(p["DOWEL_ANGLE_DEG"])
    f = f.cut(cq.Workplane("XY", origin=(r * math.cos(da), r * math.sin(da), -6.2)).circle(3.0).extrude(6.2))
    return f


def build_camera_standin(p: dict):
    """D435 envelope with lens/projector bosses and a USB-C slot, placed on the wall."""
    import cadquery as cq

    d = derived(p)
    x0, x1 = d["camera_x_range"]
    z0, z1 = d["camera_z_range"]
    cam = (
        cq.Workplane("XY", origin=(x0, 0, z0))
        .center((x1 - x0) / 2, 0)
        .rect(x1 - x0, p["CAM_L"])
        .extrude(z1 - z0)
    )
    try:
        cam = cam.edges("|X").fillet(min(11.0, p["CAM_H"] / 2 - 1))
    except Exception:
        pass
    # front face: left imager, RGB, projector, right imager (D435 order along -x_cam→+x_cam)
    lens_x = (x0 + x1) / 2
    yl = -p["IMAGER_OFFSET"]
    for y, rad in ((yl, 3.5), (yl + 15, 2.5), (yl + 29, 2.0), (yl + p["BASELINE"], 3.5)):
        cam = cam.cut(cq.Workplane("XY", origin=(lens_x, y, z1 - 1.0)).circle(rad).extrude(2))
    # USB-C slot on the back face near one end (location not on the datasheet drawing — assumed)
    cam = cam.cut(
        cq.Workplane("XY", origin=(lens_x, -p["CAM_L"] / 2 + 12, z0 - 1)).rect(3.2, 9.0).extrude(3)
    )
    if p["ARM_ANGLE_DEG"]:
        cam = cam.rotate((0, 0, 0), (0, 0, 1), p["ARM_ANGLE_DEG"])
    return cam


def build_hardware(p: dict):
    """Dowel pin, 4× M6 SHCS, 1/4-20 screw — cosmetic stand-ins for the render."""
    import cadquery as cq

    d = derived(p)
    parts = {}
    r = p["PCD"] / 2
    da = math.radians(p["DOWEL_ANGLE_DEG"])
    parts["dowel"] = (
        cq.Workplane("XY", origin=(r * math.cos(da), r * math.sin(da), -6.0)).circle(3.0).extrude(20.0)
    )
    for i, a in enumerate(p["BOLT_ANGLES_DEG"]):
        x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        shank = cq.Workplane("XY", origin=(x, y, -6.0)).circle(3.0).extrude(6.0 + p["PLATE_T"])
        head = cq.Workplane("XY", origin=(x, y, p["PLATE_T"])).circle(5.0).extrude(6.0)
        parts[f"m6_{i}"] = shank.union(head)
    tz = d["tripod_z"]
    shank = (
        cq.Workplane("YZ", origin=(d["wall_inner_x"], 0, 0)).center(0, tz).circle(3.175).extrude(p["WALL_T"] + 5)
    )
    head = cq.Workplane("YZ", origin=(d["wall_inner_x"] - 6.0, 0, 0)).center(0, tz).circle(6.0).extrude(6.0)
    parts["tripod_screw"] = shank.union(head)
    if p["ARM_ANGLE_DEG"]:
        parts = {k: v.rotate((0, 0, 0), (0, 0, 1), p["ARM_ANGLE_DEG"]) for k, v in parts.items()}
    return parts


# ----------------------------------------------------------------------------
# export + render
# ----------------------------------------------------------------------------


def export(p: dict) -> dict:
    import cadquery as cq

    OUT.mkdir(parents=True, exist_ok=True)
    bracket = build_bracket(p)
    cq.exporters.export(bracket, str(OUT / "d435_tool_bracket.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(bracket, str(OUT / "d435_tool_bracket.step"))
    assy = cq.Assembly(name="d435_tool_bracket_assembly")
    assy.add(build_flange_standin(p), name="ur_tool_flange", color=cq.Color(0.55, 0.57, 0.6))
    assy.add(bracket, name="bracket", color=cq.Color(0.15, 0.15, 0.17))
    assy.add(build_camera_standin(p), name="d435", color=cq.Color(0.75, 0.75, 0.78))
    for k, v in build_hardware(p).items():
        assy.add(v, name=k, color=cq.Color(0.8, 0.7, 0.3))
    assy.save(str(OUT / "d435_tool_bracket_assembly.step"))
    vol = bracket.val().Volume()  # mm^3
    bb = bracket.val().BoundingBox()
    return {
        "volume_cm3": vol / 1000.0,
        "mass_g_ppa_cf": vol / 1000.0 * 1.20,  # ~1.2 g/cm³ for CF-filled PA/PPA at 100% — infill lowers it
        "bbox_mm": [round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
        "stl": str(OUT / "d435_tool_bracket.stl"),
        "step": str(OUT / "d435_tool_bracket.step"),
        "assembly_step": str(OUT / "d435_tool_bracket_assembly.step"),
    }


def _mesh(shape, tol=0.15):
    verts, tris = shape.val().tessellate(tol, 0.2)
    import numpy as np

    v = np.array([[q.x, q.y, q.z] for q in verts])
    return v, np.array(tris)


def render(p: dict) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    RENDERS.mkdir(parents=True, exist_ok=True)
    parts = [
        (build_flange_standin(p), (0.55, 0.57, 0.60), 0.35),
        (build_bracket(p), (0.16, 0.18, 0.22), 0.6),
        (build_camera_standin(p), (0.78, 0.79, 0.82), 0.25),
    ]
    for v in build_hardware(p).values():
        parts.append((v, (0.82, 0.68, 0.30), 0.3))
    light = np.array([0.4, -0.6, 0.7])
    light /= np.linalg.norm(light)

    def draw(ax, lim, center):
        for shape, base, spec in parts:
            v, t = _mesh(shape)
            tri = v[t]
            n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
            lam = np.clip(n @ light, 0, 1)
            shade = 0.45 + 0.55 * lam
            cols = np.clip(np.array(base)[None, :] * shade[:, None] + spec * lam[:, None] ** 8, 0, 1)
            pc = Poly3DCollection(tri, facecolors=cols, edgecolors="none")
            ax.add_collection3d(pc)
        cx, cy, cz = center
        ax.set_xlim(cx - lim, cx + lim)
        ax.set_ylim(cy - lim, cy + lim)
        ax.set_zlim(cz - lim, cz + lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_axis_off()

    d = derived(p)
    views = [
        ("iso_closeup", 28, -55, 62, (40, 0, 12)),
        ("iso_wide", 24, -35, 105, (30, 0, 0)),
        ("side_xz", 0, -90, 62, (45, 0, 12)),
        ("top_xy", 90, -90, 62, (35, 0, 12)),
        ("front_yz", 0, 0, 62, (45, 0, 12)),
    ]
    files = []
    for name, elev, azim, lim, center in views:
        fig = plt.figure(figsize=(9, 9), dpi=150)
        ax = fig.add_subplot(111, projection="3d")
        draw(ax, lim, center)
        ax.view_init(elev=elev, azim=azim)
        fig.patch.set_facecolor("#0D1319")
        ax.set_facecolor("#0D1319")
        title = {
            "iso_closeup": "D435 on the UR tool flange — ISO close-up",
            "iso_wide": "Assembly — wrist-3, bracket, D435",
            "side_xz": "Side (X–Z): camera looks along +Z",
            "top_xy": "Top (X–Y): dowel at +Y, arm on +X",
            "front_yz": "Front (Y–Z): from the camera's view direction",
        }[name]
        fig.text(0.02, 0.97, title, color="#D7E1EB", fontsize=13, family="monospace", va="top")
        fig.text(
            0.02,
            0.03,
            f"wall top z={d['wall_top_z']:.1f}  tripod z={d['tripod_z']:.1f}  radial extent {d['radial_extent']:.1f} mm",
            color="#7F91A3",
            fontsize=9,
            family="monospace",
        )
        out = RENDERS / f"{name}.png"
        fig.savefig(out, facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.1)
        plt.close(fig)
        files.append(str(out))
    return files


def main(argv: list[str]) -> int:
    p = dict(PARAMS)
    for arg in argv:
        if "=" in arg:
            k, v = arg.split("=", 1)
            if k not in p:
                sys.exit(f"unknown parameter {k}; see PARAMS in bracket.py")
            p[k] = type(p[k])(json.loads(v)) if not isinstance(p[k], tuple) else tuple(json.loads(v))
    info = {"params": p, "derived": derived(p)}
    info["export"] = export(p)
    info["renders"] = render(p)
    (OUT / "build_info.json").write_text(json.dumps(info, indent=2, default=list))
    print(json.dumps(info, indent=2, default=list))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
