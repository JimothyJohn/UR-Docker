"""D435 tool-flange adapter — parametric CAD (CadQuery), STL/STEP export and
assembly renders. Everything dimensional lives in the PARAMS block; the spec
(README.md next to this file) quotes the same numbers, so keep them in step.

    python3 hardware/d435-tool-bracket/bracket.py

Design (Rev C): a sandwich *adapter plate* per flange (VARIANTS) whose bolt
patterns are through holes, so the tool's own bolts pass through it into the
robot. The camera hangs off one edge, back along -Z beside the wrist — its
front plate at the plate's tool face, so nothing of the camera or the bracket
rises into the tool's volume — on a 3 mm wall drafted 5° so the optical axis
tips in toward the flange axis. A chamfered lip round the seat locates the
camera and one countersunk 1/4-20 from the wrist side holds it (Rev C dropped
the two M3); a C clip beside the USB end takes the cable back along the wrist.
Fit the camera to the adapter first, then bolt the adapter to the robot.

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
  * UFACTORY 850 User Manual V2.3.0, §1.2.2.5 "End-effector Installation"
    (p. 26): ISO 9409-1-50-4-M6, Ø63 h6 face, Ø31.50 H6 pilot, 4× M6 ▽10 on
    Ø50 at 45° plus 2× M6 ▽10 on Ø50 at 90° to the dowel, Ø6 H7 ▽5 dowel; the
    connectors (tool I/O, Ethernet M8) leave the housing beside the dowel.
  * UFACTORY's own 850 wrist geometry, ``xarm_ros2`` (BSD-3-Clause),
    ``xarm_description/meshes/uf850/visual/link6.stl`` @ 62936f7 (vendored,
    unmodified): the face is 6 proud of an Ø84 housing; the pilot recess is 6
    deep; the connector block reaches r 47.9, 11.5–24.5 behind the face, from
    67° to 162° round the axis in this frame (the dowel at 90°).
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
    "PLATE_OD": 96.0,  # smallest disc that clears the ISO-80 holes (Ø9 on Ø80 + 3 mm); UR20 face is Ø100
    "PATTERNS": ("iso50", "iso80"),  # bolt patterns + pin slots cut into the plate (base: both)
    "PLATE_T": 6.0,  # 8 → 6 saved ~20 % filament; the pin still has 6 in the flange, M6/M8 clamp fine
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
    "TOP_RECESS_H": 4.0,  # leaves a 2 mm floor in the 6 mm plate (the spigot ring sits under it)
    # -- ISO 9409-1-80-6-M8 (UR20/UR30) --------------------------------------------------
    "PCD80": 80.0,
    "BOLT80_HOLE_D": 9.0,  # M8 clearance (ISO 273 medium)
    "BOLT80_ANGLES_DEG": (0.0, 60.0, 120.0, 180.0, 240.0, 300.0),  # 30° from the pin at 90°
    "DOWEL80_SLOT_W": 8.2,  # Ø8 H7 pin + 0.2
    "DOWEL80_SLOT_L": 11.0,
    "SPIGOT80_OD": 0.0,  # 0 = off in the base design; the ur20 variant turns it on (see VARIANTS)
    "SPIGOT80_H": 3.0,
    "DOWEL_ANGLE_DEG": 90.0,  # both pin holes sit at 12 o'clock on their PCD
    "ROBOT_FLANGE_DEG": 0.0,  # this frame turned by this about Z = the robot's own flange frame (UR: same)
    # -- tool-I/O connector (M8 socket at 12 o'clock on the wrist housing, behind the face) -----
    "TOOL_CONNECTOR_Z_ESERIES": -35.65,  # UR10e manual §7.11.3: Lumberg RKMW 8-354, 35.65 behind the face
    "TOOL_CONNECTOR_Z_UR20": -17.6,  # UR20 manual §8.11.3 section A-A: centreline 17.60 behind the face
    "TOOL_CONNECTOR_D": 10.0,  # socket face (render); the plug body is bigger:
    "TOOL_PLUG_D": 12.0,  # M8 8-pin cable plug body, used for the clearance check
    # -- the hanging camera wall -----------------------------------------------------------
    "ARM_ANGLE_DEG": 90.0,  # 90 = camera on +Y, the pin / tool-I/O connector side (cables leave together)
    # -- variants: two independent prints, one per flange. Overrides applied on top of the base design.
    "ESERIES_PLATE_OD": 63.0,  # e-Series print: ISO-50 only, so the disc shrinks to the Ø63 face
    "ESERIES_PATTERNS": ("iso50",),
    "ESERIES_WRIST_R": 45.0,  # ...and the wall only has to clear the Ø90 wrist (UR20 housing is Ø100)
    # UR20: its socket is only 17.6 behind the face, so clock the camera off it. Rev C's 95-wide seat
    # (+ clip) needs ≥ 54° off the socket: 30° (60° off) leaves 6.4°; Rev B's 56-wide wall sat at 45°
    "UR20_ARM_ANGLE_DEG": 30.0,
    "UR20_SPIGOT_OD": 49.8,  # Ø50 H7 pilot engaged (a dual part can't: it would hold it off a Ø63 face)
    "UR20_TOP_RECESS_D": 50.2,  # the Ø50 pilot re-presented to an ISO-80 tool (e-Series variant keeps Ø31.7)
    # UFACTORY 850: the e-Series' ISO-50 face (Ø63, Ø31.5 pilot, 4× M6 on Ø50) plus two M6 at 3 and 9
    # o'clock, an Ø84 housing, and a connector block (tool I/O + Ethernet M8) from 67° to 162° round the
    # axis, 11.5-24.5 behind the face, r ≤ 47.9 — where the e-Series wall hangs, so the camera clocks off it
    "UF850_PLATE_OD": 63.0,
    "UF850_BOLT50_ANGLES_DEG": (0.0, 45.0, 135.0, 180.0, 225.0, 315.0),  # all six M6 pass a tool's bolts
    "UF850_WRIST_R": 42.0,  # Ø84 housing behind the face (UFACTORY's link6 mesh); the wall clears it by 3
    "UF850_ARM_ANGLE_DEG": 0.0,  # 3 o'clock: 18° clear of the connector block, opposite the button at ~156°
    #   and 90° from the six-axis F/T port on the face's rim at 270°
    "UF850_ROBOT_FLANGE_DEG": 180.0,  # UFACTORY's flange frame (link6) has the dowel at -Y: this frame + 180°
    "ARM_W": 56.0,  # tangential width of the tab that carries the wall (the wall is as wide as the seat)
    "WRIST_R": 50.0,  # largest thing the wall must clear: the UR20's Ø100 housing (e-Series wrist is Ø90)
    "WALL_CLEAR": 3.0,  # radial gap housing → wall inner face (the flat head is sunk: this is all it needs)
    "WALL_T": 3.0,  # Rev C: 6 → 3 (Nick). At the top; the draft thickens it downward (≈ 4.4 at the 1/4-20)
    "TAB_T": 3.0,  # Rev C: the tab beyond the disc; the disc keeps PLATE_T (pilot recess + spigot)
    "CAM_TILT_DEG": 5.0,  # Rev C "draft": the seat leans so the optical axis tips 5° toward the flange axis
    #   (pivot = the camera's outer front edge, so nothing rises above the tool face)
    "CAM_PROUD": 0.0,  # camera front plate above the plate's tool face (0 = flush)
    # -- the chamfered lip round the camera seat (locates the camera; one screw can't stop it turning) ----
    "LIP_H": 2.5,  # proud of the seat; the USB-C plug on the end face starts ≈ 9 mm up the camera, well clear
    "LIP_W": 2.0,
    "LIP_CLEAR": 0.3,  # per side, camera envelope → lip
    "LIP_CHAMFER": 1.0,  # 45° lead-in on the lip's inner top edge
    "LIP_OUTER_CHAMFER": 0.5,
    "LIP_CORNER_R": 4.0,  # outer corners of the lip and the wall's bottom corners; inner = this − LIP_W
    # -- USB cable clip: a snap-in C ring beside the camera's USB end, cable running back along the wrist ----
    "CABLE_SIDE": "right",  # seen from the front (the lens side): "right" = the camera's own left end, where
    #   Intel's mesh puts the USB-C; "left" mirrors it; "none" drops the clip
    "CABLE_D": 6.3,  # the BOM's cable (K4, Newnex high-flex USB 3): 6.3 OD per Newnex; ring bore +0.4
    "CABLE_CLIP_WALL": 2.0,
    "CABLE_CLIP_GAP": 5.0,  # snap opening, ≈ 0.8 × CABLE_D (PPA-CF is stiff, the jacket gives), facing out
    # -- camera: Intel RealSense D435 ---------------------------------------------------------
    "CAM_L": 90.0,
    "CAM_H": 25.0,  # bottom→top (along the wall normal, +X)
    "CAM_D": 25.05,  # front plate → back face (along -Z)
    "TRIPOD_FROM_FRONT": 14.9,  # bottom 1/4-20 boss, behind the front plate (Intel URDF + mesh)
    "TRIPOD_HOLE_D": 6.6,  # 1/4-20 UNC clearance
    "TRIPOD_CSK_D": 12.7,  # 82° flat head (1/4" FHMS head ≈ Ø12.1)
    "TRIPOD_CSK_ANGLE": 82.0,
    "TRIPOD_CSK_SINK": 0.6,  # extra depth: the wrist face is vertical, the screw tilted 5°, so a flush mouth
    #   would leave one side of the head 0.55 proud
    "TRIPOD_SCREW_L": 9.525,  # 1/4-20 × 3/8" flat head (overall length) → ≈ 5.6 mm into the camera
    "IMAGER_OFFSET": 17.5,  # tripod → left imager, along the length, to the camera's left
    "BASELINE": 50.0,
    "DEPTH_ORIGIN_FROM_FRONT": 4.3,  # zero-depth plane behind the front plate (4.2 glass + 0.1)
    # -- printing allowances ----------------------------------------------------------------
    "HOLE_PRINT_ALLOWANCE": 0.0,  # add to every hole diameter if your printer undersizes
}

VARIANTS = ("eseries", "ur20", "uf850")


def variant_params(p: dict, variant: str) -> dict:
    """Base PARAMS (both patterns, Ø96, wall clearing a Ø100 housing) plus the
    per-flange overrides: ``eseries`` drops the ISO-80 pattern, shrinks to the Ø63
    face and pulls the wall in to the Ø90 wrist; ``ur20`` clocks the camera off
    the M8 socket and engages the Ø50 pilot both ways; ``uf850`` is the e-Series
    disc with the 850's two extra M6, the wall pulled in to its Ø84 housing and
    clocked off its connector block."""
    if variant == "eseries":
        return {
            **p,
            "PLATE_OD": p["ESERIES_PLATE_OD"],
            "PATTERNS": p["ESERIES_PATTERNS"],
            "WRIST_R": p["ESERIES_WRIST_R"],
        }
    if variant == "ur20":
        return {
            **p,
            "ARM_ANGLE_DEG": p["UR20_ARM_ANGLE_DEG"],
            "SPIGOT80_OD": p["UR20_SPIGOT_OD"],
            "TOP_RECESS_D": p["UR20_TOP_RECESS_D"],
        }
    if variant == "uf850":
        return {
            **p,
            "PLATE_OD": p["UF850_PLATE_OD"],
            "PATTERNS": ("iso50",),
            "BOLT50_ANGLES_DEG": p["UF850_BOLT50_ANGLES_DEG"],
            "WRIST_R": p["UF850_WRIST_R"],
            "ARM_ANGLE_DEG": p["UF850_ARM_ANGLE_DEG"],
            "ROBOT_FLANGE_DEG": p["UF850_ROBOT_FLANGE_DEG"],
        }
    raise ValueError(f"unknown variant {variant!r}; one of {VARIANTS}")


HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
RENDERS = HERE / "renders"
VENDOR = HERE / "vendor"
CAMERA_MESH = VENDOR / "d435_realsense-ros_12k.stl"
ROBOT_MESHES = {"uf850": VENDOR / "uf850_link6_xarm_ros2.stl"}  # the real wrist, where a vendor publishes it
CAMERA_MESH_SOURCE = {
    "url": "https://raw.githubusercontent.com/IntelRealSense/realsense-ros/9189b0591fae677570d2dfd0ae8957fb6f12e635/realsense2_description/meshes/d435.dae",
    "sha256": "42f3b66f47a1f8f425a2e4dc07c1d9c283183167d8441f520a15623d98f9bf78",
    "license": "Apache-2.0 (IntelRealSense/realsense-ros)",
    "triangles": 12000,
}


def derived(p: dict) -> dict:
    """Numbers that follow from PARAMS (also written into the spec).

    The camera side is authored on +X (radial out), +Y tangential, untilted;
    ``tilt`` then leans it CAM_TILT_DEG about the camera's outer front edge (so
    the optical axis tips in toward the flange axis and nothing rises above the
    tool face), and ``rot`` clocks it about Z by ARM_ANGLE_DEG."""
    wall_in = p["WRIST_R"] + p["WALL_CLEAR"]
    seat_x = wall_in + p["WALL_T"]  # the camera's bottom plane before the tilt
    front_z = p["PLATE_T"] + p["CAM_PROUD"]
    back_z = front_z - p["CAM_D"]
    tripod_z_u = front_z - p["TRIPOD_FROM_FRONT"]
    t = math.radians(p["CAM_TILT_DEG"])
    ct, st = math.cos(t), math.sin(t)
    px, pz = seat_x + p["CAM_H"], front_z  # pivot: the camera's outer front edge

    def tilt(x, y, z):
        dx, dz = x - px, z - pz
        return (px + ct * dx - st * dz, y, pz + st * dx + ct * dz)

    a = math.radians(p["ARM_ANGLE_DEG"])
    ca, sa = math.cos(a), math.sin(a)

    def rot(x, y, z):  # the whole camera side rotates about Z by ARM_ANGLE_DEG
        return (round(ca * x - sa * y, 6), round(sa * x + ca * y, 6), round(z, 6))

    # seat outline (lip outer edge), camera coords before the tilt
    seat_half_l = p["CAM_L"] / 2 + p["LIP_CLEAR"] + p["LIP_W"]
    seat_z_lo = back_z - p["LIP_CLEAR"] - p["LIP_W"]
    # lowest point: the seat's bottom edge where it meets the (vertical) wrist-side face
    dz = seat_z_lo - pz
    dx = (wall_in - px + st * dz) / ct
    lowest_z = pz + st * dx + ct * dz
    cam_corners = [tilt(x, 0, z) for x in (seat_x, seat_x + p["CAM_H"]) for z in (back_z, front_z)]
    tripod = tilt(seat_x, 0, tripod_z_u)
    # 1/4-20 axis runs in along (-cos t, -sin t) from the seat to the wrist-side face x = wall_in
    tripod_wall = (tripod[0] - wall_in) / ct
    clip = _cable_clip_xy(p, wall_in, seat_half_l)
    # camera frame in flange coords before tilt + arm rotation: +z_cam = +Z, +y_cam (image down,
    # toward the camera's bottom = the wall) = -X, hence +x_cam = +Y and camera-left = -Y
    depth_origin = rot(
        *tilt(seat_x + p["CAM_H"] / 2, -p["IMAGER_OFFSET"], front_z - p["DEPTH_ORIGIN_FROM_FRONT"])
    )
    x_cam, y_cam, z_cam = rot(0, 1, 0), rot(-ct, 0, -st), rot(-st, 0, ct)
    rf = math.radians(p["ROBOT_FLANGE_DEG"])
    cr, sr = math.cos(rf), math.sin(rf)

    def to_robot(v):  # this frame → the robot's flange frame (what its controller reports poses in)
        return (round(cr * v[0] - sr * v[1], 6), round(sr * v[0] + cr * v[1], 6), round(v[2], 6))

    usb_side = -1.0 if p["CABLE_SIDE"] != "left" else 1.0
    return {
        "wall_inner_x": wall_in,
        "wall_outer_x": round(seat_x, 6),
        "seat_x_untilted": seat_x,
        "seat_half_length": seat_half_l,
        "seat_z_lo_untilted": seat_z_lo,
        "tilt_deg": p["CAM_TILT_DEG"],
        "tilt_pivot_xz": (px, pz),
        "wall_z_range": (round(lowest_z, 3), p["PLATE_T"]),
        "wall_t_at_tripod": round(tripod_wall, 3),
        "camera_front_z": front_z,
        "camera_back_z": back_z,
        "camera_x_range": (
            round(min(c[0] for c in cam_corners), 3),
            round(max(c[0] for c in cam_corners), 3),
        ),
        "camera_y_range": (-p["CAM_L"] / 2, p["CAM_L"] / 2),
        "camera_z_range": (
            round(min(c[2] for c in cam_corners), 3),
            round(max(c[2] for c in cam_corners), 3),
        ),
        "tripod_z": round(tripod[2], 3),
        "tripod_xyz_unclocked": tuple(round(v, 3) for v in tripod),
        "radial_extent": round(max(max(c[0] for c in cam_corners), clip["x_max"] if clip else 0.0), 3),
        "lowest_z": round(min(lowest_z, min(c[2] for c in cam_corners)), 3),
        "cable_clip": clip,
        "usb_c": {
            "where": "on an end face (Nick's unit, 2026-09-12): the camera-left end in Intel's mesh, "
            "i.e. the right-hand end seen from the front; the cable clip sits on CABLE_SIDE",
            "flange_xyz_mm": rot(
                *tilt(seat_x + p["CAM_H"] / 2, usb_side * p["CAM_L"] / 2, front_z - p["CAM_D"] / 2)
            ),
        },
        "tool_connector": {
            "eseries": _connector_check(
                p, p["ARM_ANGLE_DEG"], p["TOOL_CONNECTOR_Z_ESERIES"], wall_in, lowest_z, seat_half_l, clip
            ),
            "ur20_at_this_angle": _connector_check(
                p, p["ARM_ANGLE_DEG"], p["TOOL_CONNECTOR_Z_UR20"], wall_in, lowest_z, seat_half_l, clip
            ),
            "ur20_at_UR20_ARM_ANGLE_DEG": _connector_check(
                p, p["UR20_ARM_ANGLE_DEG"], p["TOOL_CONNECTOR_Z_UR20"], wall_in, lowest_z, seat_half_l, clip
            ),
        },
        "depth_origin_flange_mm": depth_origin,
        "camera_axes_in_flange": {"x_cam": x_cam, "y_cam": y_cam, "z_cam": z_cam},
        # the hand-eye seed: the same, in the robot's own flange frame (UR: identical)
        "robot_flange_deg": p["ROBOT_FLANGE_DEG"],
        "depth_origin_robot_flange_mm": to_robot(depth_origin),
        "camera_axes_in_robot_flange": {
            k: to_robot(v) for k, v in (("x_cam", x_cam), ("y_cam", y_cam), ("z_cam", z_cam))
        },
        "arm_angle_deg": p["ARM_ANGLE_DEG"],
    }


def _cable_clip_xy(p: dict, wall_in: float, seat_half_l: float) -> dict | None:
    """Where the cable ring sits, unclocked: just past the seat's end, its outer edge
    on the seat line, so the cable drops off the USB plug, in under the camera and
    back along the wrist — and the plug's overmold (camera mid-height, x ≳ seat + 7)
    stays clear. Past |y| = 46 the ring is outside any wrist radius."""
    side = p["CABLE_SIDE"]
    if side == "none":
        return None
    if side not in ("right", "left"):
        raise ValueError(f"CABLE_SIDE must be right, left or none, not {side!r}")
    r_in = (p["CABLE_D"] + 0.4) / 2
    r_out = r_in + p["CABLE_CLIP_WALL"]
    sgn = -1.0 if side == "right" else 1.0  # right, seen from the front = camera-left = -Y unclocked
    cy = sgn * (seat_half_l + r_out - 1.0)  # 1 mm into the wall's end
    cx = wall_in + p["WALL_T"] - r_out
    return {"x": cx, "y": cy, "r_in": r_in, "r_out": r_out, "x_max": cx + r_out, "y_extent": abs(cy) + r_out}


def _connector_check(
    p: dict,
    arm_deg: float,
    conn_z: float,
    wall_in: float,
    wall_bot: float,
    seat_half_l: float,
    clip: dict | None,
) -> dict:
    """Does the hanging wall stay clear of the M8 tool-I/O plug at 12 o'clock?
    Angular: the wall (with the cable clip, taken on both sides to be safe) spans
    ±atan(half width / wall_in) about the arm angle, the plug ±asin(TOOL_PLUG_D/2 /
    wall_in) about 12 o'clock. Axial: the plug body spans conn_z ± TOOL_PLUG_D/2;
    the wall spans wall_bot … PLATE_T."""
    half_w = max(p["ARM_W"] / 2, seat_half_l, clip["y_extent"] if clip else 0.0)
    half_wall = math.degrees(math.atan2(half_w, wall_in))
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


def _tilt(shape, p: dict):
    """Lean a camera-side shape (authored untilted) by CAM_TILT_DEG about the pivot
    in ``derived`` — the same map as ``derived``'s ``tilt``."""
    if not p["CAM_TILT_DEG"]:
        return shape
    px, pz = derived(p)["tilt_pivot_xz"]
    return shape.rotate((px, 0, pz), (px, 1, pz), -p["CAM_TILT_DEG"])


def build_bracket(p: dict):
    import cadquery as cq

    d = derived(p)
    T = p["PLATE_T"]
    ha = p["HOLE_PRINT_ALLOWANCE"]
    r_plate = p["PLATE_OD"] / 2
    wall_in, seat_x = d["wall_inner_x"], d["seat_x_untilted"]
    front_z, back_z = d["camera_front_z"], d["camera_back_z"]
    half_l, z_lo = d["seat_half_length"], d["seat_z_lo_untilted"]

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

    # --- the camera side, authored untilted in camera coordinates (seat = the plane x = seat_x) ---
    # the wall: a slab under the whole seat, made thick enough inward that once tilted and trimmed to the
    # vertical wrist-side face (x = wall_in) it is a wedge — WALL_T at the top, thickening downward
    slab_t = p["WALL_T"] + 8.0
    z_hi = front_z + 12.0  # past the tool face; trimmed flat at z = T after the tilt
    wall = (
        cq.Workplane("YZ", origin=(seat_x - slab_t, 0, 0))
        .center(0, (z_lo + z_hi) / 2)
        .rect(2 * half_l, z_hi - z_lo)
        .extrude(slab_t)
    )
    try:
        wall = wall.edges("|X").edges("<Z").fillet(p["LIP_CORNER_R"])
    except Exception:
        pass
    # the lip round the camera's footprint, chamfered on both top edges (inner = lead-in)
    zc = (front_z + back_z) / 2
    in_l, in_d = p["CAM_L"] + 2 * p["LIP_CLEAR"], p["CAM_D"] + 2 * p["LIP_CLEAR"]
    lip = (
        cq.Workplane("YZ", origin=(seat_x, 0, 0))
        .center(0, zc)
        .rect(in_l + 2 * p["LIP_W"], in_d + 2 * p["LIP_W"])
        .extrude(p["LIP_H"])
        .edges("|X")
        .fillet(p["LIP_CORNER_R"])
    )
    hole = (
        cq.Workplane("YZ", origin=(seat_x - 1.0, 0, 0))
        .center(0, zc)
        .rect(in_l, in_d)
        .extrude(p["LIP_H"] + 2.0)
        .edges("|X")
        .fillet(max(0.5, p["LIP_CORNER_R"] - p["LIP_W"]))
    )
    lip = lip.cut(hole)
    for which, size in (("outer", p["LIP_OUTER_CHAMFER"]), ("inner", p["LIP_CHAMFER"])):
        if not size:
            continue
        top = lip.faces(">X").val()
        edges = top.outerWire().Edges() if which == "outer" else top.innerWires()[0].Edges()
        lip = lip.newObject(edges).chamfer(size)
    side = wall.union(lip)
    # 1/4-20 through the wall along the seat normal, countersunk on the wrist side
    tripod_u = front_z - p["TRIPOD_FROM_FRONT"]
    mouth = seat_x - d["wall_t_at_tripod"] + p["TRIPOD_CSK_SINK"]
    side = side.cut(
        _yz_hole_tool(
            cq,
            mouth,
            0,
            tripod_u,
            p["TRIPOD_HOLE_D"] + ha,
            p["TRIPOD_CSK_D"],
            p["TRIPOD_CSK_ANGLE"],
            seat_x - mouth + 2.0,
            mouth_len=4.0,
        )
    )
    side = _tilt(side, p)
    # trim: nothing inside the wrist clearance, nothing above the tool face
    keep = cq.Workplane("XY", origin=(wall_in, -200, -100)).box(300, 400, 100 + T, centered=False)
    side = side.intersect(keep)

    # the tab: disc → wall, TAB_T thick, flush with the tool face
    tab_x0, tab_x1 = r_plate - 20.0, wall_in + 1.5
    tab = (
        cq.Workplane("XY", origin=(0, 0, T - p["TAB_T"]))
        .center((tab_x0 + tab_x1) / 2, 0)
        .rect(tab_x1 - tab_x0, p["ARM_W"])
        .extrude(p["TAB_T"])
    )
    side = side.union(tab)

    # the cable clip: a C ring standing on the tool face (prints as a plain vertical extrusion)
    clip = d["cable_clip"]
    if clip:
        zb = d["wall_z_range"][0]
        ring = (
            cq.Workplane("XY", origin=(0, 0, zb))
            .center(clip["x"], clip["y"])
            .circle(clip["r_out"])
            .circle(clip["r_in"])
            .extrude(T - zb)
        )
        gap = (
            cq.Workplane("XY", origin=(0, 0, zb - 1.0))
            .center(clip["x"] + clip["r_out"] / 2 + 0.5, clip["y"])
            .rect(clip["r_out"] + 1.0, p["CABLE_CLIP_GAP"])
            .extrude(T - zb + 2.0)
        )
        side = side.union(ring.cut(gap))

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
    patterns = {
        "iso50": (
            p["PCD50"],
            p["BOLT50_HOLE_D"],
            p["BOLT50_ANGLES_DEG"],
            p["DOWEL50_SLOT_W"],
            p["DOWEL50_SLOT_L"],
        ),
        "iso80": (
            p["PCD80"],
            p["BOLT80_HOLE_D"],
            p["BOLT80_ANGLES_DEG"],
            p["DOWEL80_SLOT_W"],
            p["DOWEL80_SLOT_L"],
        ),
    }
    for name in p["PATTERNS"]:
        pcd, dia, angles, w, ln = patterns[name]
        for a in angles:
            x, y = _pcd_xy(pcd / 2, a)
            body = body.cut(cq.Workplane("XY", origin=(x, y, -20)).circle((dia + ha) / 2).extrude(80))
        x, y = _pcd_xy(pcd / 2, p["DOWEL_ANGLE_DEG"])
        body = body.cut(
            cq.Workplane("XY", origin=(0, 0, -20))
            .center(x, y)
            .slot2D(ln, w + ha, angle=p["DOWEL_ANGLE_DEG"])
            .extrude(80)
        )

    return body


def _yz_hole_tool(
    cq,
    x_face: float,
    y: float,
    z: float,
    dia: float,
    csk_d: float,
    csk_angle: float,
    depth: float,
    mouth_len: float = 1.0,
):
    """A countersunk-hole *cutter* with its axis along +X: cone opening at x_face
    (the wrist side), cylinder continuing through the wall, and a Ø csk_d
    cylinder mouth_len outward of the face so the cut is clean."""
    cone_h = (csk_d - dia) / 2 / math.tan(math.radians(csk_angle / 2))
    cyl = cq.Workplane("YZ", origin=(x_face - 1.0, 0, 0)).center(y, z).circle(dia / 2).extrude(depth + 2.0)
    cone = cq.Solid.makeCone(
        csk_d / 2,
        dia / 2,
        cone_h,
        pnt=cq.Vector(x_face, y, z),
        dir=cq.Vector(1, 0, 0),
    )
    mouth = cq.Solid.makeCylinder(
        csk_d / 2, mouth_len, pnt=cq.Vector(x_face - mouth_len, y, z), dir=cq.Vector(1, 0, 0)
    )
    return cyl.union(cq.Workplane().add(cone)).union(cq.Workplane().add(mouth))


def build_flange_standin(p: dict, robot: str = "eseries"):
    """Robot stand-in for the assembly/renders.
    eseries: UR e-Series wrist-3 + Ø63 tool flange (UR10e manual §8.7.5).
    ur20:    UR20/UR30 Ø100 output housing, 56.5 long, with the ISO-80 pattern (UR20 manual §8.11.3)."""
    import cadquery as cq

    if robot == "uf850":
        return _uf850_standin(cq, p)
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


def _uf850_standin(cq, p: dict):
    """UFACTORY 850 wrist for the STEP assembly: Ø84 × 24 housing, Ø63 × 6 face, the
    Ø31.5 × 6 pilot, six M6 ▽10, the Ø6 ▽5 dowel, and the connector block as a box
    (r 38…47.9 over 67°…95°, 11.5…24.5 behind the face) — the envelope UFACTORY's
    mesh shows; the renders and the clash check use the mesh itself."""
    f = cq.Workplane("XY", origin=(0, 0, -30.0)).circle(p["UF850_WRIST_R"]).extrude(24.0)
    f = f.union(cq.Workplane("XY", origin=(0, 0, -6.0)).circle(31.5).extrude(6.0))
    f = f.cut(cq.Workplane("XY", origin=(0, 0, -6.0)).circle(31.5 / 2).extrude(6.0))
    for a in p["UF850_BOLT50_ANGLES_DEG"]:
        x, y = _pcd_xy(p["PCD50"] / 2, a)
        f = f.cut(cq.Workplane("XY", origin=(x, y, -10.0)).circle(2.5).extrude(10.0))
    x, y = _pcd_xy(p["PCD50"] / 2, p["DOWEL_ANGLE_DEG"])
    f = f.cut(cq.Workplane("XY", origin=(x, y, -5.0)).circle(3.0).extrude(5.0))
    block = (
        cq.Workplane("XY", origin=(0, 0, -24.5))
        .center(43.0, 0)
        .rect(10.0, 2 * 47.9 * math.sin(math.radians(14.0)))
        .extrude(13.0)
    )
    return f.union(_rot_z(block, 81.0))


def load_robot_mesh(p: dict, robot: str):
    """The vendor's wrist mesh (metres, its own flange frame) as (verts mm, tris) in
    this frame — turned by -ROBOT_FLANGE_DEG — or None when there is none."""
    import numpy as np

    path = ROBOT_MESHES.get(robot)
    if path is None:
        return None
    raw = path.read_bytes()
    n = struct.unpack_from("<I", raw, 80)[0]
    rec = np.frombuffer(raw, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n, offset=84)
    v = rec["v"].reshape(-1, 3).astype(float) * 1000.0
    a = -math.radians(p["ROBOT_FLANGE_DEG"])
    c, s = math.cos(a), math.sin(a)
    out = v.copy()
    out[:, 0], out[:, 1] = c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1]
    return out, np.arange(len(out)).reshape(-1, 3)


def robot_clash(p: dict, bracket, robot: str) -> dict | None:
    """The bracket against the vendor's wrist mesh: vertices of the wrist inside the
    bracket (with the wrist lowered 0.05 mm, since the plate sits *on* its face) —
    the build refuses anything but 0 — and the smallest gaps behind the face: the
    spigot in the pilot, and the hanging wall to the housing and connectors."""
    import cadquery as cq
    import numpy as np

    mesh = load_robot_mesh(p, robot)
    if mesh is None:
        return None
    verts = np.unique(np.round(mesh[0], 4), axis=0) - (0.0, 0.0, 0.05)
    solid = bracket.val()
    bb = solid.BoundingBox()
    near = verts[
        (verts[:, 0] > bb.xmin - 6)
        & (verts[:, 0] < bb.xmax + 6)
        & (verts[:, 1] > bb.ymin - 6)
        & (verts[:, 1] < bb.ymax + 6)
        & (verts[:, 2] < bb.zmax + 6)
    ]
    inside = int(sum(solid.isInside(cq.Vector(*q), 0.0) for q in near))
    behind = near[near[:, 2] < -0.5]
    r = np.hypot(behind[:, 0], behind[:, 1])

    def gap(pts):
        g = min((solid.distance(cq.Vertex.makeVertex(*q)) for q in pts), default=None)
        return None if g is None else round(g, 2)

    return {
        "wrist_vertices_inside": inside,
        "pilot_gap_mm": gap(behind[r < 20.0]),  # the spigot in the pilot recess: a locating fit, by design
        "wall_gap_mm": gap(behind[r > 33.0]),  # the hanging wall and clip to the housing and connectors
    }


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
    out[:, 0] = d["seat_x_untilted"] + p["CAM_H"] / 2 + v[:, 1]
    out[:, 1] = -v[:, 0]
    out[:, 2] = d["camera_front_z"] + v[:, 2]
    t = math.radians(p["CAM_TILT_DEG"])
    if t:  # derived()'s tilt about the camera's outer front edge
        (px, pz), c, s = d["tilt_pivot_xz"], math.cos(t), math.sin(t)
        dx, dz = out[:, 0] - px, out[:, 2] - pz
        out[:, 0], out[:, 2] = px + c * dx - s * dz, pz + s * dx + c * dz
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


def camera_clash(p: dict, bracket) -> int:
    """Vertices of Intel's D435 body inside the bracket, with the camera lifted
    0.05 mm off its seat (its bottom flat lies *on* the seat, which a point test
    counts as inside). The build refuses anything but 0."""
    import cadquery as cq
    import numpy as np

    raw = CAMERA_MESH.read_bytes()
    n = struct.unpack_from("<I", raw, 80)[0]
    rec = np.frombuffer(raw, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n, offset=84)
    local = np.unique(rec["v"].reshape(-1, 3).astype(float), axis=0) + (0.0, 0.05, 0.0)
    solid = bracket.val()
    return int(sum(solid.isInside(cq.Vector(*q), 0.0) for q in camera_mesh_to_flange(p, local)))


def build_camera_standin(p: dict):
    """D435 envelope for the STEP assembly (the mesh can't go into STEP at a sane
    size): 90 × 25 × 25.05 with the rounded front/back edges, lens windows on the
    front plate, the USB-C recess on the camera-left end face."""
    import cadquery as cq

    d = derived(p)
    x0 = d["seat_x_untilted"]
    x1 = x0 + p["CAM_H"]
    z0, z1 = d["camera_back_z"], d["camera_front_z"]
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
    end = left * p["CAM_L"] / 2
    cam = cam.cut(cq.Workplane("XZ", origin=(xc, end, (z0 + z1) / 2)).rect(9.0, 3.2).extrude(1.5, both=True))
    return _rot_z(_tilt(cam, p), p["ARM_ANGLE_DEG"])


def build_hardware(p: dict, robot: str = "eseries"):
    """Fasteners as cosmetic stand-ins for the render: the tool-pattern bolts +
    dowel for the chosen robot, the countersunk 1/4-20 from the wrist side."""
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
    # the 1/4-20 × 3/8" flat head, authored untilted along the seat normal like its hole
    dia, csk_d, ang = 6.35, p["TRIPOD_CSK_D"] - 0.6, p["TRIPOD_CSK_ANGLE"]
    z = d["camera_front_z"] - p["TRIPOD_FROM_FRONT"]
    x_head = d["seat_x_untilted"] - d["wall_t_at_tripod"] + p["TRIPOD_CSK_SINK"] + 0.3
    cone_h = (csk_d - dia) / 2 / math.tan(math.radians(ang / 2))
    shank = (
        cq.Workplane("YZ", origin=(x_head, 0, 0)).center(0, z).circle(dia / 2).extrude(p["TRIPOD_SCREW_L"])
    )
    head = cq.Solid.makeCone(csk_d / 2, dia / 2, cone_h, pnt=cq.Vector(x_head, 0, z), dir=cq.Vector(1, 0, 0))
    parts["tripod_screw"] = _rot_z(_tilt(shank.union(cq.Workplane().add(head)), p), p["ARM_ANGLE_DEG"])
    return parts


# ----------------------------------------------------------------------------
# export + render
# ----------------------------------------------------------------------------


def export(p: dict) -> dict:
    import cadquery as cq

    OUT.mkdir(parents=True, exist_ok=True)
    out = {"variants": {}}
    for variant in VARIANTS:
        pp = variant_params(p, variant)
        bracket = build_bracket(pp)
        stl = OUT / f"d435_tool_bracket_{variant}.stl"
        step = OUT / f"d435_tool_bracket_{variant}.step"
        cq.exporters.export(bracket, str(stl), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(bracket, str(step))
        assy = cq.Assembly(name=f"d435_tool_bracket_{variant}")
        robot_name = "uf850_wrist" if variant == "uf850" else f"ur_flange_{variant}"
        assy.add(build_flange_standin(pp, variant), name=robot_name, color=cq.Color(0.55, 0.57, 0.6))
        assy.add(bracket, name="bracket", color=cq.Color(0.15, 0.15, 0.17))
        assy.add(build_camera_standin(pp), name="d435_envelope", color=cq.Color(0.75, 0.75, 0.78))
        for k, v in build_hardware(pp, variant).items():
            assy.add(v, name=k, color=cq.Color(0.8, 0.7, 0.3))
        assy_path = OUT / f"d435_tool_bracket_assembly_{variant}.step"
        assy.save(str(assy_path))
        vol = bracket.val().Volume()  # mm^3
        clash = camera_clash(pp, bracket)
        if clash:
            sys.exit(f"{variant}: {clash} vertices of Intel's D435 body are inside the bracket")
        wrist = robot_clash(pp, bracket, variant)
        if wrist and wrist["wrist_vertices_inside"]:
            sys.exit(
                f"{variant}: {wrist['wrist_vertices_inside']} vertices of the robot's wrist are inside "
                "the bracket"
            )
        bb = bracket.val().BoundingBox()
        out["variants"][variant] = {
            "overrides": {k: v for k, v in pp.items() if p.get(k) != v},
            "derived": derived(pp),
            "volume_cm3": vol / 1000.0,
            "camera_clash_vertices": clash,
            "robot_clash": wrist,
            "mass_g_ppa_cf_solid": vol / 1000.0 * 1.25,  # Bambu PPA-CF spec 1.25 g/cm³, 100 % solid
            "bbox_mm": [round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
            "stl": str(stl),
            "step": str(step),
            "assembly_step": str(assy_path),
        }
    # Intel's body, placed in the flange frame (default clocking), for anyone assembling in their own CAD
    verts, tris = load_camera_mesh(p)
    _write_stl(OUT / "d435_body_in_flange_frame.stl", verts, tris)
    out["camera_body_stl"] = str(OUT / "d435_body_in_flange_frame.stl")
    return out


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
    scenes, angles, derived_by_scene = {}, {}, {}
    for robot in VARIANTS:
        pp = variant_params(p, robot)
        wrist = load_robot_mesh(pp, robot)  # the vendor's own wrist where there is one
        parts = [
            (
                wrist if wrist is not None else _mesh(build_flange_standin(pp, robot)),
                (0.55, 0.57, 0.60),
                0.35,
            ),
            (_mesh(build_bracket(pp)), (0.16, 0.18, 0.22), 0.6),
            (load_camera_mesh(pp), (0.80, 0.81, 0.84), 0.30),
        ]
        for v in build_hardware(pp, robot).values():
            parts.append((_mesh(v), (0.82, 0.68, 0.30), 0.3))
        scenes[robot] = parts
        angles[robot] = pp["ARM_ANGLE_DEG"]
        derived_by_scene[robot] = derived_by_scene[f"bracket_{robot}"] = derived(pp)
    for robot in VARIANTS:  # robot hidden: the print itself, the camera and its screw
        scenes[f"bracket_{robot}"] = list(scenes[robot][1:3]) + [
            (_mesh(v), (0.82, 0.68, 0.30), 0.3)
            for k, v in build_hardware(variant_params(p, robot), robot).items()
            if k == "tripod_screw"
        ]
        angles[f"bracket_{robot}"] = angles[robot]
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
            "iso_uf850",
            "uf850",
            22,
            40,
            72,
            (25, 15, -10),
            "UFACTORY 850: camera at 3 o'clock, clear of the tool-I/O + Ethernet block",
        ),
        (
            "side_xz",
            "eseries",
            0,
            -90,
            62,
            (40, 0, -5),
            "Side (X–Z): seat drafted 5°, the camera looks in toward the flange axis",
        ),
        (
            "top_xy",
            "eseries",
            90,
            -90,
            62,
            (35, 0, 0),
            "Top (X–Y), e-Series print: ISO-50 only; pin, M8 socket and camera all on +Y",
        ),
        (
            "underside_eseries",
            "bracket_eseries",
            -50,
            -125,
            60,
            (15, 0, -2),
            "e-Series print from below: Ø63 disc, Ø31.3 spigot into the Ø31.5 H7 recess",
        ),
        (
            "underside_ur20",
            "bracket_ur20",
            -50,
            -125,
            60,
            (15, 0, -2),
            "UR20 print from below: Ø49.8 spigot into the Ø50 H7 pilot, camera 45° off the M8",
        ),
        (
            "underside_uf850",
            "bracket_uf850",
            -50,
            -125,
            60,
            (15, 0, -2),
            "UFACTORY 850 print from below: six M6, Ø31.3 spigot into the Ø31.5 H6 pilot",
        ),
        (
            "wrist_side",
            "bracket_eseries",
            -12,
            175,
            48,
            (55, 0, -6),
            "Wrist side, robot hidden: one sunk 1/4-20, the cable clip on the USB end",
        ),
    ]
    files = []
    for name, robot, elev, azim, lim, center, title in views:
        if title == "ur20_title":
            title = (
                f"UR20/UR30: M8 socket only 17.6 behind the face — camera clocked {angles['ur20']:.0f}° off"
            )
        d = derived_by_scene[robot]
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
            if isinstance(p[k], str):
                p[k] = v
            else:
                p[k] = type(p[k])(json.loads(v)) if not isinstance(p[k], tuple) else tuple(json.loads(v))
    info = {"params": p, "variants": VARIANTS, "derived": derived(p), "camera_mesh": CAMERA_MESH_SOURCE}
    info["export"] = export(p)
    info["renders"] = render(p)
    (OUT / "build_info.json").write_text(json.dumps(info, indent=2, default=list))
    print(json.dumps(info, indent=2, default=list))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
