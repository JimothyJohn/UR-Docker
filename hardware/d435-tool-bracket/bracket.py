"""D435 tool-flange adapter — parametric CAD (CadQuery), STL/STEP export and
assembly renders. Everything dimensional lives in the PARAMS block; the spec
(README.md next to this file) quotes the same numbers, so keep them in step.

    uv run --with cadquery --with matplotlib python hardware/d435-tool-bracket/bracket.py

Design (Rev B): a sandwich *adapter plate* that carries BOTH the ISO 9409-1-50
(UR3e/5e/10e/16e) and the ISO 9409-1-80 (UR20/UR30) bolt patterns as through
holes, so the tool's own bolts pass through it into whichever flange it is on.
The camera hangs off one edge and points *back along -Z* beside the wrist — its
front plate is flush with the plate's tool face, so nothing of the camera or
the bracket rises into the tool's volume. The 1/4-20 and the two M3 go through
the hanging wall from the wrist side, countersunk flush: fit the camera to the
adapter first, then bolt the adapter to the robot.

Frame: origin at the centre of the robot's tool-flange face, +Z away from the
flange (the tool direction, and the camera's optical axis), +Y towards the
dowel/pin hole (12 o'clock on both UR flanges, the tool-I/O connector side),
+X completes the right-handed set. The camera hangs on +Y by default — the
same side as the tool-I/O connector, so both cables leave together
(``ARM_ANGLE_DEG = 90``; the geometry is authored on +X and rotated).

Sources (verified 2026-09-04):
  * UR10e User Manual (SW 5.19), §8.7.5 "Securing Tool": Ø63 h8 face, Ø50 ±0.1
    PCD, 4× M6-6H ▽8 at 45° from the dowel, Ø6 H7 ▽6.20 dowel hole at 12
    o'clock, Ø31.50 H7 centring recess, Ø90 wrist, flange proud of the wrist
    by 6.50; "do not use bolts that extend beyond 10 mm"; UR recommends a
    radially slotted hole for the positioning pin.
  * UR20 User Manual (SW 5.21, 718-818-00), §8.11.3 "Securing Tool": Ø100 h8
    face, Ø80 ±0.1 PCD, 6× M8-6H ▽17.25 on 6×60° starting 30° from the pin,
    Ø8 H7 ▽8 ±0.2 pin hole at 12 o'clock, Ø50 H7 pilot, the Ø100 housing
    continues 56.50 behind the face; "do not use bolts that extend beyond
    17.25 mm". UR30 shares the flange (ISO 9409-1-80-6-M8). Tool-I/O socket at
    12 o'clock, centreline 17.60 behind the face (section A-A).
  * UR10e User Manual (SW 5.21, 711-039-00), §7.11.3: Lumberg RKMW 8-354
    tool-I/O socket at 12 o'clock on the Ø90 wrist, 35.65 behind the face.
  * Intel RealSense D400 Series Datasheet 337029-017, Fig. 10-9 (D435/D435i):
    90 × 25 × 25.05 mm, 1/4-20 on the bottom, 2× M3 45 mm apart (max
    insertion 3 mm, 0.4 Nm), 50 mm stereo baseline.
  * Intel's own D435 body geometry, ``realsense-ros`` (Apache-2.0),
    ``realsense2_description/meshes/d435.dae`` @ 9189b05 and
    ``urdf/_d435.urdf.xacro``: tripod boss 14.9 mm behind the front plate at
    the length centre; the depth origin (left imager) 17.5 mm to the camera's
    left of the tripod, at mid-height, 4.3 mm behind the front plate; M3 pair
    at ±22.5 mm on the bottom, 14.2 mm behind the front plate (measured on the
    mesh); USB-C on the BACK face at the camera's left end (x 36–45 mm from
    centre). The decimated mesh in ``vendor/`` drives the renders.
"""

from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path

# ----------------------------------------------------------------------------
# PARAMS — the only place with numbers. Units: mm, degrees.
# ----------------------------------------------------------------------------
PARAMS = {
    # -- plate ---------------------------------------------------------------------------
    "PLATE_OD": 100.0,  # matches the UR20/UR30 Ø100 h8 face; overhangs the e-Series Ø63 face
    "PLATE_T": 8.0,
    "PLATE_EDGE_CHAMFER": 1.0,
    "CENTER_HOLE_D": 24.0,  # cable / air pass-through
    # -- ISO 9409-1-50-4-M6 (UR3e/5e/10e/16e) --------------------------------------------
    "PCD50": 50.0,
    "BOLT50_HOLE_D": 6.6,  # M6 clearance (ISO 273 medium)
    "BOLT50_ANGLES_DEG": (45.0, 135.0, 225.0, 315.0),  # 45° from the dowel at 90°
    "DOWEL50_SLOT_W": 6.2,  # Ø6 H7 pin + 0.2
    "DOWEL50_SLOT_L": 9.0,  # radial slot (UR: slot radially to avoid over-constraint)
    "SPIGOT_OD": 31.3,  # into the Ø31.5 H7 recess; 0.2 print allowance (tune)
    "SPIGOT_H": 4.0,  # recess is deeper; 4 mm engagement leaves margin
    "SPIGOT_CHAMFER": 0.8,
    "TOP_RECESS_D": 31.7,  # the Ø31.5 pilot re-presented to an ISO-50 tool
    "TOP_RECESS_H": 5.0,
    # -- ISO 9409-1-80-6-M8 (UR20/UR30) --------------------------------------------------
    "PCD80": 80.0,
    "BOLT80_HOLE_D": 9.0,  # M8 clearance (ISO 273 medium)
    "BOLT80_ANGLES_DEG": (0.0, 60.0, 120.0, 180.0, 240.0, 300.0),  # 30° from the pin at 90°
    "DOWEL80_SLOT_W": 8.2,  # Ø8 H7 pin + 0.2
    "DOWEL80_SLOT_L": 11.0,
    "SPIGOT80_OD": 0.0,  # 0 = off (fits both robots). 49.8 adds a Ø50 H7 spigot: UR20-only print
    "SPIGOT80_H": 3.0,
    "DOWEL_ANGLE_DEG": 90.0,  # both pin holes sit at 12 o'clock on their PCD
    # -- tool-I/O connector (M8 socket at 12 o'clock on the wrist housing, behind the face) -----
    "TOOL_CONNECTOR_Z_ESERIES": -35.65,  # UR10e manual §7.11.3: Lumberg RKMW 8-354, 35.65 behind the face
    "TOOL_CONNECTOR_Z_UR20": -17.6,  # UR20 manual §8.11.3 section A-A: centreline 17.60 behind the face
    "TOOL_CONNECTOR_D": 10.0,  # socket face (render); the plug body is bigger:
    "TOOL_PLUG_D": 12.0,  # M8 8-pin cable plug body, used for the clearance check
    # -- the hanging camera wall -----------------------------------------------------------
    "ARM_ANGLE_DEG": 90.0,  # 90 = camera on +Y, the pin / tool-I/O connector side (cables leave together)
    "UR20_ARM_ANGLE_DEG": 45.0,  # UR20: its socket is only 17.6 behind the face, so clock the camera 45° off
    "ARM_W": 66.0,  # tangential width of the tab + wall (covers the M3 pair at ±22.5)
    "WRIST_R": 50.0,  # largest thing the wall must clear: the UR20's Ø100 housing (e-Series wrist is Ø90)
    "WALL_CLEAR": 3.0,  # radial gap housing → wall inner face (flat heads are flush, so this is all it needs)
    "WALL_T": 6.0,
    "WALL_BELOW_CAM": 2.0,  # wall continues this far past the camera's back face
    "WALL_CORNER_R": 6.0,  # rounded bottom corners of the wall / outer corners of the tab
    "CAM_PROUD": 0.0,  # camera front plate above the plate's tool face (0 = flush)
    # -- camera: Intel RealSense D435 ---------------------------------------------------------
    "CAM_L": 90.0,
    "CAM_H": 25.0,  # bottom→top (along the wall normal, +X)
    "CAM_D": 25.05,  # front plate → back face (along -Z)
    "TRIPOD_FROM_FRONT": 14.9,  # bottom 1/4-20 boss, behind the front plate (Intel URDF + mesh)
    "TRIPOD_HOLE_D": 6.6,  # 1/4-20 UNC clearance
    "TRIPOD_CSK_D": 12.7,  # 82° flat head (1/4" FHMS head ≈ Ø12.1)
    "TRIPOD_CSK_ANGLE": 82.0,
    "M3_FROM_FRONT": 14.2,  # measured on Intel's mesh (0.7 in front of the tripod line)
    "M3_HOLE_D": 3.4,
    "M3_CSK_D": 6.6,  # 90° flat head (M3 FHMS head Ø6.0)
    "M3_CSK_ANGLE": 90.0,
    "M3_SPACING": 45.0,
    "IMAGER_OFFSET": 17.5,  # tripod → left imager, along the length, to the camera's left
    "BASELINE": 50.0,
    "DEPTH_ORIGIN_FROM_FRONT": 4.3,  # zero-depth plane behind the front plate (4.2 glass + 0.1)
    # -- printing allowances ----------------------------------------------------------------
    "HOLE_PRINT_ALLOWANCE": 0.0,  # add to every hole diameter if your printer undersizes
}

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
RENDERS = HERE / "renders"
VENDOR = HERE / "vendor"
CAMERA_MESH = VENDOR / "d435_realsense-ros_12k.stl"
CAMERA_MESH_SOURCE = {
    "url": "https://raw.githubusercontent.com/IntelRealSense/realsense-ros/9189b0591fae677570d2dfd0ae8957fb6f12e635/realsense2_description/meshes/d435.dae",
    "sha256": "42f3b66f47a1f8f425a2e4dc07c1d9c283183167d8441f520a15623d98f9bf78",
    "license": "Apache-2.0 (IntelRealSense/realsense-ros)",
    "triangles": 12000,
}


def derived(p: dict) -> dict:
    """Numbers that follow from PARAMS (also written into the spec)."""
    wall_in = p["WRIST_R"] + p["WALL_CLEAR"]
    wall_out = wall_in + p["WALL_T"]
    front_z = p["PLATE_T"] + p["CAM_PROUD"]
    back_z = front_z - p["CAM_D"]
    wall_bottom_z = back_z - p["WALL_BELOW_CAM"]
    tripod_z = front_z - p["TRIPOD_FROM_FRONT"]
    m3_z = front_z - p["M3_FROM_FRONT"]
    cam_x0, cam_x1 = wall_out, wall_out + p["CAM_H"]
    # camera frame in flange coords before the arm rotation: +z_cam = +Z, +y_cam
    # (image down, toward the camera's bottom = the wall) = -X, hence +x_cam = +Y
    # and camera-left = -Y. ARM_ANGLE_DEG rotates the whole set about Z.
    a = math.radians(p["ARM_ANGLE_DEG"])
    ca, sa = math.cos(a), math.sin(a)

    def rot(x, y, z):  # the whole camera side rotates about Z by ARM_ANGLE_DEG
        return (round(ca * x - sa * y, 6), round(sa * x + ca * y, 6), z)

    depth_origin = rot(wall_out + p["CAM_H"] / 2, -p["IMAGER_OFFSET"], front_z - p["DEPTH_ORIGIN_FROM_FRONT"])
    x_cam, y_cam, z_cam = rot(0, 1, 0), rot(-1, 0, 0), (0, 0, 1)
    return {
        "wall_inner_x": wall_in,
        "wall_outer_x": wall_out,
        "wall_z_range": (wall_bottom_z, p["PLATE_T"]),
        "camera_front_z": front_z,
        "camera_back_z": back_z,
        "camera_x_range": (cam_x0, cam_x1),
        "camera_y_range": (-p["CAM_L"] / 2, p["CAM_L"] / 2),
        "camera_z_range": (back_z, front_z),
        "tripod_z": tripod_z,
        "m3_z": m3_z,
        "radial_extent": cam_x1,
        "lowest_z": wall_bottom_z,
        "usb_c": {
            "where": "camera back face (-Z), camera-left end, 36–45 mm from centre; exits along -Z",
            "flange_xyz_mm": rot(wall_out + p["CAM_H"] / 2, -40.5, back_z),
        },
        "tool_connector": {
            "eseries": _connector_check(
                p, p["ARM_ANGLE_DEG"], p["TOOL_CONNECTOR_Z_ESERIES"], wall_in, wall_bottom_z
            ),
            "ur20_at_this_angle": _connector_check(
                p, p["ARM_ANGLE_DEG"], p["TOOL_CONNECTOR_Z_UR20"], wall_in, wall_bottom_z
            ),
            "ur20_at_UR20_ARM_ANGLE_DEG": _connector_check(
                p, p["UR20_ARM_ANGLE_DEG"], p["TOOL_CONNECTOR_Z_UR20"], wall_in, wall_bottom_z
            ),
        },
        "depth_origin_flange_mm": depth_origin,
        "camera_axes_in_flange": {"x_cam": x_cam, "y_cam": y_cam, "z_cam": z_cam},
        "arm_angle_deg": p["ARM_ANGLE_DEG"],
    }


def _connector_check(p: dict, arm_deg: float, conn_z: float, wall_in: float, wall_bot: float) -> dict:
    """Does the hanging wall stay clear of the M8 tool-I/O plug at 12 o'clock?
    Angular: the wall spans ±atan(ARM_W/2 / wall_in) about the arm angle, the plug
    ±asin(TOOL_PLUG_D/2 / wall_in) about 12 o'clock. Axial: the plug body spans
    conn_z ± TOOL_PLUG_D/2; the wall spans wall_bot … PLATE_T."""
    half_wall = math.degrees(math.atan2(p["ARM_W"] / 2, wall_in))
    half_plug = math.degrees(math.asin(min(1.0, p["TOOL_PLUG_D"] / 2 / wall_in)))
    dang = abs((arm_deg - p["DOWEL_ANGLE_DEG"] + 180) % 360 - 180)
    angular_gap = round(dang - half_wall - half_plug, 1)
    axial_gap = round(wall_bot - (conn_z + p["TOOL_PLUG_D"] / 2), 2)  # wall bottom above the plug's top edge
    clear = angular_gap > 0 or axial_gap > 0
    return {"angular_gap_deg": angular_gap, "axial_gap_mm": axial_gap, "clear": clear}


# ----------------------------------------------------------------------------
# geometry
# ----------------------------------------------------------------------------


def _pcd_xy(r: float, angle_deg: float) -> tuple[float, float]:
    a = math.radians(angle_deg)
    return r * math.cos(a), r * math.sin(a)


def _rot_z(shape, deg: float):
    return shape.rotate((0, 0, 0), (0, 0, 1), deg) if deg else shape


def build_bracket(p: dict):
    import cadquery as cq

    d = derived(p)
    T = p["PLATE_T"]
    ha = p["HOLE_PRINT_ALLOWANCE"]
    r_plate = p["PLATE_OD"] / 2
    wall_in, wall_out = d["wall_inner_x"], d["wall_outer_x"]
    wall_bot = d["wall_z_range"][0]

    # plate disc
    plate = cq.Workplane("XY").circle(r_plate).extrude(T)
    try:
        plate = (
            plate.faces(">Z")
            .edges("%CIRCLE")
            .edges(cq.selectors.RadiusNthSelector(0, directionMax=True))
            .chamfer(p["PLATE_EDGE_CHAMFER"])
        )
    except Exception:
        pass

    # tab: from well inside the disc out to the wall's outer face, full plate thickness
    tab_x0 = r_plate - 20.0
    tab = cq.Workplane("XY").center((tab_x0 + wall_out) / 2, 0).rect(wall_out - tab_x0, p["ARM_W"]).extrude(T)
    try:
        tab = tab.edges("|Z").edges(">X").fillet(p["WALL_CORNER_R"])
    except Exception:
        pass

    # the hanging wall: from the underside of the tab down past the camera's back
    wall = (
        cq.Workplane("XY", origin=(0, 0, wall_bot))
        .center((wall_in + wall_out) / 2, 0)
        .rect(p["WALL_T"], p["ARM_W"])
        .extrude(T - wall_bot)
    )
    try:
        wall = wall.edges("|X").edges("<Z").fillet(p["WALL_CORNER_R"])
    except Exception:
        pass
    side = tab.union(wall)
    # camera fasteners through the wall (axis = X), countersunk on the wrist-side face
    side = side.cut(
        _yz_hole_tool(
            cq,
            wall_in,
            0,
            d["tripod_z"],
            p["TRIPOD_HOLE_D"] + ha,
            p["TRIPOD_CSK_D"],
            p["TRIPOD_CSK_ANGLE"],
            p["WALL_T"],
        )
    )
    for y in (-p["M3_SPACING"] / 2, p["M3_SPACING"] / 2):
        side = side.cut(
            _yz_hole_tool(
                cq, wall_in, y, d["m3_z"], p["M3_HOLE_D"] + ha, p["M3_CSK_D"], p["M3_CSK_ANGLE"], p["WALL_T"]
            )
        )

    # only the camera side clocks; the plate features stay with the robot
    body = plate.union(_rot_z(side, p["ARM_ANGLE_DEG"]))

    # bottom spigot ring(s) into the flange pilot, chamfered lead-in
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
    if p["SPIGOT80_OD"]:
        body = body.union(
            cq.Workplane("XY", origin=(0, 0, -p["SPIGOT80_H"]))
            .circle(p["SPIGOT80_OD"] / 2)
            .circle(p["SPIGOT80_OD"] / 2 - 4.0)
            .extrude(p["SPIGOT80_H"])
        )

    # through features: centre hole, top recess, both bolt patterns, both pin slots
    body = body.cut(cq.Workplane("XY", origin=(0, 0, -20)).circle(p["CENTER_HOLE_D"] / 2).extrude(80))
    body = body.cut(
        cq.Workplane("XY", origin=(0, 0, T - p["TOP_RECESS_H"])).circle(p["TOP_RECESS_D"] / 2).extrude(20)
    )
    for pcd, dia, angles in (
        (p["PCD50"], p["BOLT50_HOLE_D"], p["BOLT50_ANGLES_DEG"]),
        (p["PCD80"], p["BOLT80_HOLE_D"], p["BOLT80_ANGLES_DEG"]),
    ):
        for a in angles:
            x, y = _pcd_xy(pcd / 2, a)
            body = body.cut(cq.Workplane("XY", origin=(x, y, -20)).circle((dia + ha) / 2).extrude(80))
    for pcd, w, ln in (
        (p["PCD50"], p["DOWEL50_SLOT_W"], p["DOWEL50_SLOT_L"]),
        (p["PCD80"], p["DOWEL80_SLOT_W"], p["DOWEL80_SLOT_L"]),
    ):
        x, y = _pcd_xy(pcd / 2, p["DOWEL_ANGLE_DEG"])
        body = body.cut(
            cq.Workplane("XY", origin=(0, 0, -20))
            .center(x, y)
            .slot2D(ln, w + ha, angle=p["DOWEL_ANGLE_DEG"])
            .extrude(80)
        )

    return body


def _yz_hole_tool(
    cq, x_face: float, y: float, z: float, dia: float, csk_d: float, csk_angle: float, depth: float
):
    """A countersunk-hole *cutter* with its axis along +X: cone opening at x_face
    (the wrist side), cylinder continuing through the wall."""
    cone_h = (csk_d - dia) / 2 / math.tan(math.radians(csk_angle / 2))
    cyl = cq.Workplane("YZ", origin=(x_face - 1.0, 0, 0)).center(y, z).circle(dia / 2).extrude(depth + 2.0)
    cone = cq.Solid.makeCone(
        csk_d / 2,
        dia / 2,
        cone_h,
        pnt=cq.Vector(x_face, y, z),
        dir=cq.Vector(1, 0, 0),
    )
    # extend the mouth 1 mm outside the face so the cut is clean
    mouth = cq.Solid.makeCylinder(csk_d / 2, 1.0, pnt=cq.Vector(x_face - 1.0, y, z), dir=cq.Vector(1, 0, 0))
    return cyl.union(cq.Workplane().add(cone)).union(cq.Workplane().add(mouth))


def build_flange_standin(p: dict, robot: str = "eseries"):
    """Robot stand-in for the assembly/renders.
    eseries: UR e-Series wrist-3 + Ø63 tool flange (UR10e manual §8.7.5).
    ur20:    UR20/UR30 Ø100 output housing, 56.5 long, with the ISO-80 pattern (UR20 manual §8.11.3)."""
    import cadquery as cq

    if robot == "ur20":
        f = cq.Workplane("XY", origin=(0, 0, -56.5)).circle(50.0).extrude(56.5)
        f = f.cut(cq.Workplane("XY", origin=(0, 0, -30)).circle(25.0).extrude(30))  # Ø50 H7 pilot bore
        for a in p["BOLT80_ANGLES_DEG"]:
            x, y = _pcd_xy(p["PCD80"] / 2, a)
            f = f.cut(cq.Workplane("XY", origin=(x, y, -17.25)).circle(3.4).extrude(17.25))
        x, y = _pcd_xy(p["PCD80"] / 2, p["DOWEL_ANGLE_DEG"])
        f = f.cut(cq.Workplane("XY", origin=(x, y, -8.0)).circle(4.0).extrude(8.0))
        return f.union(_connector_standin(cq, p, 50.0, p["TOOL_CONNECTOR_Z_UR20"]))
    wrist = cq.Workplane("XY", origin=(0, 0, -6.5 - 48.75)).circle(45.0).extrude(48.75)
    face = cq.Workplane("XY", origin=(0, 0, -6.5)).circle(31.5).extrude(6.5)
    f = wrist.union(face)
    f = f.cut(cq.Workplane("XY", origin=(0, 0, -6.5)).circle(31.5 / 2).extrude(6.5))  # recess
    for a in p["BOLT50_ANGLES_DEG"]:
        x, y = _pcd_xy(p["PCD50"] / 2, a)
        f = f.cut(cq.Workplane("XY", origin=(x, y, -8)).circle(2.5).extrude(8))
    x, y = _pcd_xy(p["PCD50"] / 2, p["DOWEL_ANGLE_DEG"])
    f = f.cut(cq.Workplane("XY", origin=(x, y, -6.2)).circle(3.0).extrude(6.2))
    return f.union(_connector_standin(cq, p, 45.0, p["TOOL_CONNECTOR_Z_ESERIES"]))


def _connector_standin(cq, p: dict, housing_r: float, z: float):
    """The M8 tool-I/O socket: a Ø10 boss at 12 o'clock, 2 mm proud of the housing."""
    a = math.radians(p["DOWEL_ANGLE_DEG"])
    pnt = cq.Vector((housing_r - 1.0) * math.cos(a), (housing_r - 1.0) * math.sin(a), z)
    axis = cq.Vector(math.cos(a), math.sin(a), 0)
    return cq.Workplane().add(cq.Solid.makeCylinder(p["TOOL_CONNECTOR_D"] / 2, 3.0, pnt=pnt, dir=axis))


def camera_mesh_to_flange(p: dict, verts):
    """Map Intel's mesh frame (x = camera-left, y = up/away from the bottom, z =
    forward, origin on the front plate at the tripod's length position, mid-height)
    into the flange frame for this bracket. Returns an (N, 3) array."""
    import numpy as np

    d = derived(p)
    v = np.asarray(verts, dtype=float)
    out = np.empty_like(v)
    out[:, 0] = d["wall_outer_x"] + p["CAM_H"] / 2 + v[:, 1]
    out[:, 1] = -v[:, 0]
    out[:, 2] = d["camera_front_z"] + v[:, 2]
    a = math.radians(p["ARM_ANGLE_DEG"])
    if a:
        c, s = math.cos(a), math.sin(a)
        x, y = out[:, 0].copy(), out[:, 1].copy()
        out[:, 0], out[:, 1] = c * x - s * y, s * x + c * y
    return out


def load_camera_mesh(p: dict):
    """Intel's D435 body (vendored, decimated) as (verts, tris) in the flange frame."""
    import numpy as np

    raw = CAMERA_MESH.read_bytes()
    n = struct.unpack_from("<I", raw, 80)[0]
    rec = np.frombuffer(raw, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n, offset=84)
    verts = rec["v"].reshape(-1, 3)
    tris = np.arange(len(verts)).reshape(-1, 3)
    return camera_mesh_to_flange(p, verts), tris


def build_camera_standin(p: dict):
    """D435 envelope for the STEP assembly (the mesh can't go into STEP at a sane
    size): 90 × 25 × 25.05 with the rounded front/back edges, lens windows on the
    front plate, the USB-C recess on the back at the camera-left end."""
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
        cam = cam.edges("|Y").edges(">Z").fillet(8.0)
        cam = cam.edges("|Y").edges("<Z").fillet(4.0)
        cam = cam.edges("|Z").fillet(4.0)
    except Exception:
        pass
    xc = (x0 + x1) / 2
    left = -1.0  # camera-left is -Y
    for off, rad in (
        (p["IMAGER_OFFSET"], 5.0),
        (p["IMAGER_OFFSET"] - p["BASELINE"], 5.0),
        (p["IMAGER_OFFSET"] + 15.0, 4.0),
        (0.0, 5.0),
    ):
        cam = cam.cut(cq.Workplane("XY", origin=(xc, left * off, z1 - 0.5)).circle(rad).extrude(1))
    cam = cam.cut(cq.Workplane("XY", origin=(xc, left * 40.5, z0 - 0.5)).rect(3.2, 9.0).extrude(1.5))
    return _rot_z(cam, p["ARM_ANGLE_DEG"])


def build_hardware(p: dict, robot: str = "eseries"):
    """Fasteners as cosmetic stand-ins for the render: the tool-pattern bolts +
    dowel for the chosen robot, the countersunk 1/4-20 and M3 from the wrist side."""
    import cadquery as cq

    d = derived(p)
    parts = {}
    if robot == "ur20":
        pcd, angles, shank_r, head_r, head_h, pin_r, pin_len, pin_in = (
            p["PCD80"],
            p["BOLT80_ANGLES_DEG"],
            4.0,
            6.5,
            8.0,
            4.0,
            22.0,
            8.0,
        )
    else:
        pcd, angles, shank_r, head_r, head_h, pin_r, pin_len, pin_in = (
            p["PCD50"],
            p["BOLT50_ANGLES_DEG"],
            3.0,
            5.0,
            6.0,
            3.0,
            20.0,
            6.0,
        )
    x, y = _pcd_xy(pcd / 2, p["DOWEL_ANGLE_DEG"])
    parts["dowel"] = cq.Workplane("XY", origin=(x, y, -pin_in)).circle(pin_r).extrude(pin_len)
    for i, a in enumerate(angles):
        x, y = _pcd_xy(pcd / 2, a)
        shank = cq.Workplane("XY", origin=(x, y, -pin_in)).circle(shank_r).extrude(pin_in + p["PLATE_T"])
        head = cq.Workplane("XY", origin=(x, y, p["PLATE_T"])).circle(head_r).extrude(head_h)
        parts[f"bolt_{i}"] = shank.union(head)
    wall_in = d["wall_inner_x"]
    for name, y, z, dia, csk_d, ang, length in (
        (
            "tripod_screw",
            0.0,
            d["tripod_z"],
            6.35,
            p["TRIPOD_CSK_D"] - 0.6,
            p["TRIPOD_CSK_ANGLE"],
            p["WALL_T"] + 5.5,
        ),
        (
            "m3_a",
            -p["M3_SPACING"] / 2,
            d["m3_z"],
            3.0,
            p["M3_CSK_D"] - 0.6,
            p["M3_CSK_ANGLE"],
            p["WALL_T"] + 2.5,
        ),
        (
            "m3_b",
            p["M3_SPACING"] / 2,
            d["m3_z"],
            3.0,
            p["M3_CSK_D"] - 0.6,
            p["M3_CSK_ANGLE"],
            p["WALL_T"] + 2.5,
        ),
    ):
        cone_h = (csk_d - dia) / 2 / math.tan(math.radians(ang / 2))
        shank = cq.Workplane("YZ", origin=(wall_in, 0, 0)).center(y, z).circle(dia / 2).extrude(length)
        head = cq.Solid.makeCone(
            csk_d / 2, dia / 2, cone_h, pnt=cq.Vector(wall_in, y, z), dir=cq.Vector(1, 0, 0)
        )
        parts[name] = shank.union(cq.Workplane().add(head))
    cam_side = ("tripod_screw", "m3_a", "m3_b")
    return {k: (_rot_z(v, p["ARM_ANGLE_DEG"]) if k in cam_side else v) for k, v in parts.items()}


# ----------------------------------------------------------------------------
# export + render
# ----------------------------------------------------------------------------


def export(p: dict) -> dict:
    import cadquery as cq

    OUT.mkdir(parents=True, exist_ok=True)
    bracket = build_bracket(p)
    cq.exporters.export(bracket, str(OUT / "d435_tool_bracket.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(bracket, str(OUT / "d435_tool_bracket.step"))
    p_ur20 = _ur20_params(p)
    bracket_ur20 = build_bracket(p_ur20)
    cq.exporters.export(
        bracket_ur20, str(OUT / "d435_tool_bracket_ur20.stl"), tolerance=0.02, angularTolerance=0.1
    )
    for robot, pp, br in (("eseries", p, bracket), ("ur20", p_ur20, bracket_ur20)):
        assy = cq.Assembly(name=f"d435_tool_bracket_{robot}")
        assy.add(build_flange_standin(pp, robot), name=f"ur_flange_{robot}", color=cq.Color(0.55, 0.57, 0.6))
        assy.add(br, name="bracket", color=cq.Color(0.15, 0.15, 0.17))
        assy.add(build_camera_standin(pp), name="d435_envelope", color=cq.Color(0.75, 0.75, 0.78))
        for k, v in build_hardware(pp, robot).items():
            assy.add(v, name=k, color=cq.Color(0.8, 0.7, 0.3))
        assy.save(str(OUT / f"d435_tool_bracket_assembly_{robot}.step"))
    # Intel's body, placed in the flange frame, for anyone assembling in their own CAD
    verts, tris = load_camera_mesh(p)
    _write_stl(OUT / "d435_body_in_flange_frame.stl", verts, tris)
    vol = bracket.val().Volume()  # mm^3
    bb = bracket.val().BoundingBox()
    return {
        "volume_cm3": vol / 1000.0,
        "mass_g_ppa_cf": vol / 1000.0 * 1.20,  # ~1.2 g/cm³ for CF-filled PA/PPA at 100% — infill lowers it
        "bbox_mm": [round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
        "stl": str(OUT / "d435_tool_bracket.stl"),
        "stl_ur20_clocked": str(OUT / "d435_tool_bracket_ur20.stl"),
        "ur20_derived": derived(p_ur20),
        "step": str(OUT / "d435_tool_bracket.step"),
        "assembly_step": [str(OUT / f"d435_tool_bracket_assembly_{r}.step") for r in ("eseries", "ur20")],
        "camera_body_stl": str(OUT / "d435_body_in_flange_frame.stl"),
    }


def _ur20_params(p: dict) -> dict:
    """The same part clocked for a UR20/UR30 (its M8 socket is in the wall's path at 90°)."""
    return {**p, "ARM_ANGLE_DEG": p["UR20_ARM_ANGLE_DEG"]}


def _write_stl(path: Path, verts, tris) -> None:
    import numpy as np

    tri = np.asarray(verts)[np.asarray(tris)]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tri), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(path, "wb") as f:
        f.write(b"urctl d435 body, flange frame, mm".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tri)))
        f.write(rec.tobytes())


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
    d = derived(p)
    scenes, angles = {}, {}
    for robot, pp in (("eseries", p), ("ur20", _ur20_params(p))):
        parts = [
            (_mesh(build_flange_standin(pp, robot)), (0.55, 0.57, 0.60), 0.35),
            (_mesh(build_bracket(pp)), (0.16, 0.18, 0.22), 0.6),
            (load_camera_mesh(pp), (0.80, 0.81, 0.84), 0.30),
        ]
        for v in build_hardware(pp, robot).values():
            parts.append((_mesh(v), (0.82, 0.68, 0.30), 0.3))
        scenes[robot] = parts
        angles[robot] = pp["ARM_ANGLE_DEG"]
    angles["bracket"] = angles["eseries"]
    scenes["bracket"] = list(scenes["eseries"][1:3]) + [
        (_mesh(v), (0.82, 0.68, 0.30), 0.3)
        for k, v in build_hardware(p, "eseries").items()
        if k in ("tripod_screw", "m3_a", "m3_b")
    ]
    light = np.array([0.4, -0.6, 0.7])
    light /= np.linalg.norm(light)

    def draw(ax, parts, lim, center):
        for (v, t), base, spec in parts:
            tri = v[t]
            n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
            lam = np.clip(n @ light, 0, 1)
            shade = 0.45 + 0.55 * lam
            cols = np.clip(np.array(base)[None, :] * shade[:, None] + spec * lam[:, None] ** 8, 0, 1)
            ax.add_collection3d(Poly3DCollection(tri, facecolors=cols, edgecolors="none"))
        cx, cy, cz = center
        ax.set_xlim(cx - lim, cx + lim)
        ax.set_ylim(cy - lim, cy + lim)
        ax.set_zlim(cz - lim, cz + lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_axis_off()

    views = [
        (
            "iso_closeup",
            "eseries",
            28,
            -55,
            62,
            (35, 0, -5),
            "D435 on a UR e-Series flange — beside the wrist, on the tool-I/O side",
        ),
        ("iso_wide", "eseries", 24, -35, 105, (25, 0, -15), "e-Series assembly — wrist-3, adapter, D435"),
        ("iso_ur20", "ur20", 28, -55, 75, (35, 0, -10), "ur20_title"),
        (
            "side_xz",
            "eseries",
            0,
            -90,
            62,
            (40, 0, -5),
            "Side (X–Z): front plate flush with the tool face, looks +Z",
        ),
        (
            "top_xy",
            "eseries",
            90,
            -90,
            62,
            (35, 0, 0),
            "Top (X–Y): ISO-50 + ISO-80 patterns; pins, M8 socket and camera all on +Y",
        ),
        (
            "wrist_side",
            "bracket",
            -12,
            175,
            48,
            (55, 0, -6),
            "Wrist side, robot hidden: countersunk 1/4-20 + 2× M3 sit flush",
        ),
    ]
    files = []
    for name, robot, elev, azim, lim, center, title in views:
        if title == "ur20_title":
            title = (
                f"UR20/UR30: M8 socket only 17.6 behind the face — camera clocked {angles['ur20']:.0f}° off"
            )
        a = math.radians(angles[robot])
        cx, cy, cz = center  # views are authored for the camera on +X; follow the scene's arm angle
        center = (cx * math.cos(a) - cy * math.sin(a), cx * math.sin(a) + cy * math.cos(a), cz)
        azim += angles[robot]
        fig = plt.figure(figsize=(9, 9), dpi=150)
        ax = fig.add_subplot(111, projection="3d")
        draw(ax, scenes[robot], lim, center)
        ax.view_init(elev=elev, azim=azim)
        fig.patch.set_facecolor("#0D1319")
        ax.set_facecolor("#0D1319")
        fig.text(0.02, 0.97, title, color="#D7E1EB", fontsize=13, family="monospace", va="top")
        footer = (
            f"front plate z={d['camera_front_z']:.1f}  tripod z={d['tripod_z']:.1f}  "
            f"wall x={d['wall_inner_x']:.0f}…{d['wall_outer_x']:.0f}  "
            f"radial extent {d['radial_extent']:.1f}  lowest z={d['lowest_z']:.1f} mm"
        )
        fig.text(
            0.02,
            0.03,
            footer,
            color="#7F91A3",
            fontsize=9,
            family="monospace",
        )
        out = RENDERS / f"{name}.png"
        fig.savefig(out, facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.1)
        plt.close(fig)
        files.append(str(out))
    for stale in ("front_yz.png", "below_iso.png"):
        (RENDERS / stale).unlink(missing_ok=True)
    return files


def refresh_camera_mesh() -> None:
    """Re-derive vendor/ from Intel's DAE (needs network + `fast-simplification`)."""
    import hashlib
    import urllib.request
    import xml.etree.ElementTree as ET

    import fast_simplification as fs
    import numpy as np

    raw = urllib.request.urlopen(CAMERA_MESH_SOURCE["url"], timeout=120).read()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != CAMERA_MESH_SOURCE["sha256"]:
        sys.exit(f"d435.dae sha256 mismatch: {digest}")
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.fromstring(raw)
    verts, tris, off = [], [], 0
    for g in root.findall(".//c:library_geometries/c:geometry", ns):
        mesh = g.find("c:mesh", ns)
        srcs = {
            s.get("id"): np.array(s.find("c:float_array", ns).text.split(), float)
            for s in mesh.findall("c:source", ns)
        }
        pos_id = mesh.find("c:vertices", ns).find("c:input", ns).get("source")[1:]
        P = srcs[pos_id].reshape(-1, 3) * 1000.0  # metres → mm
        for tri in mesh.findall("c:triangles", ns):
            stride = len(tri.findall("c:input", ns))
            idx = np.array(tri.find("c:p", ns).text.split(), int).reshape(-1, 3, stride)[:, :, 0]
            tris.append(idx + off)
        verts.append(P)
        off += len(P)
    V, T = np.vstack(verts).astype(np.float32), np.vstack(tris).astype(np.int64)
    v2, t2 = fs.simplify(V, T, target_count=CAMERA_MESH_SOURCE["triangles"], agg=7)
    VENDOR.mkdir(parents=True, exist_ok=True)
    _write_stl(CAMERA_MESH, v2, t2)
    print(f"wrote {CAMERA_MESH} ({len(t2)} triangles) from {len(T)}")


def main(argv: list[str]) -> int:
    if "--refresh-camera-mesh" in argv:
        refresh_camera_mesh()
        return 0
    p = dict(PARAMS)
    for arg in argv:
        if "=" in arg:
            k, v = arg.split("=", 1)
            if k not in p:
                sys.exit(f"unknown parameter {k}; see PARAMS in bracket.py")
            p[k] = type(p[k])(json.loads(v)) if not isinstance(p[k], tuple) else tuple(json.loads(v))
    info = {"params": p, "derived": derived(p), "camera_mesh": CAMERA_MESH_SOURCE}
    info["export"] = export(p)
    info["renders"] = render(p)
    (OUT / "build_info.json").write_text(json.dumps(info, indent=2, default=list))
    print(json.dumps(info, indent=2, default=list))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
