"""Step-wise pick of the calibration block. State in pick_state.json; every step snapshots.
pick.py plan            → block top centre (mark), top-face axes, yawed hover pose (no motion)
pick.py hover [yaw_deg] → move above the block, fingertips 40 mm over its top (yaw override in deg)
pick.py descend [mm]    → straight down so the pads sit `mm` below the top face (default 20)
pick.py grip / release  → Robotiq close / open through `urctl gripper` (separate process; cockpit idle)
pick.py lift [mm]       → straight up (default 80)
pick.py move dx dy      → translate in base (m) at the current height
"""
# ruff: noqa  — scratch script from the 2026-09-25 hardware session (scripts/pilot/README.md)

import os as _os

HERE = _os.path.dirname(_os.path.abspath(__file__))

import json
import math
import subprocess
import sys

src = open(HERE + "/orbit_cal4.py").read()
exec(src.split("track = Transform")[0])  # post/get/frame/white_blobs/top_face_click/K/Transform
STATE = HERE + "/pick_state.json"
FINGERTIP = 0.163  # Hand-E 157 mm + 6 mm bracket adapter, below the flange along tool z


def load():
    try:
        return json.load(open(STATE))
    except Exception:
        return {}


def save(st):
    json.dump(st, open(STATE, "w"), indent=1)


def snap(name):
    s = post("/api/snapshot", {"name": name})
    return s.get("color_png"), [v.get("path") for v in s.get("views", [])]


def flange_now():
    r = post("/api/robot/locate", {"standoff_m": 0.2, "reference": "flange", "point_m": [0, 0, 0.3]})
    return r["flange_pose"]


def move_to(pose, v=0.05):
    r = post("/api/robot/move", {"pose": pose, "velocity": v, "tcp": [0] * 6})
    print(
        "  move →",
        {k: r.get(k) for k in ("ok", "error", "protective_stop")},
        [round(x, 4) for x in r.get("landed") or []],
    )
    return r


cmd = sys.argv[1]
st = load()
if cmd == "plan":
    rob = get("/api/robot")["robot"]
    he = rob["handeye"]
    he["pose"] = he["flange_to_color_pose"]
    T_fc = Transform.from_pose(he["pose"])
    cal = get("/api/cal")
    mark = cal["result"]["mark_base"]
    if len(sys.argv) >= 4:
        mark = [float(sys.argv[2]), float(sys.argv[3]), mark[2]]  # plan X Y: target this block instead
    fl = flange_now()
    T_bf = Transform.from_pose(fl)
    # find the block at the predicted pixel and fit its top face for the axes
    p_cam = T_fc.inverse().apply(T_bf.inverse().apply(mark))
    pr = (K["ppx"] + K["fx"] * p_cam[0] / p_cam[2], K["ppy"] + K["fy"] * p_cam[1] / p_cam[2])
    hdr, w, h, ch, rgb, depth = frame()
    best = None
    for b in white_blobs(w, h, ch, rgb):
        if b["px"] < 400:
            continue
        tf = top_face_click(w, h, ch, rgb, depth, b["bbox"], hdr["depth_scale_m"])
        if tf is None:
            continue
        d3 = math.hypot(*(tf[2][i] - p_cam[i] for i in range(3)))
        if best is None or d3 < best[0]:
            best = (d3, b, tf)
    d3, b, (tx, ty, c3, nsel, nall) = best
    if d3 > 0.08:
        sys.exit(
            f"no block within 40 mm of the solved mark (nearest {d3 * 1000:.0f} mm at pixel {(b['cx'], b['cy'])}) — move so the mark is in view"
        )
    # top-face principal axes: PCA (2x2 on the in-plane spread) of the selected points, in camera coords
    x0, y0, x1, y1 = b["bbox"]
    pts = []
    for y in range(y0, y1):
        for x in range(x0, x1):
            i = (y * w + x) * ch
            r, g, bb = rgb[i], rgb[i + 1], rgb[i + 2]
            if min(r, g, bb) > 185 and max(r, g, bb) - min(r, g, bb) < 30:
                d = depth[2 * (y * w + x)] | (depth[2 * (y * w + x) + 1] << 8)
                if d:
                    z = d * hdr["depth_scale_m"]
                    p = ((x - K["ppx"]) * z / K["fx"], (y - K["ppy"]) * z / K["fy"], z)
                    if abs(p[2] - c3[2]) < 0.008:
                        pts.append(p)
    # axes in base (horizontal): rotate camera points to base then PCA in XY
    R = lambda p: T_bf.rotate(T_fc.rotate(p))
    base_pts = [R(p) for p in pts]
    cx = sum(p[0] for p in base_pts) / len(base_pts)
    cy = sum(p[1] for p in base_pts) / len(base_pts)
    sxx = sum((p[0] - cx) ** 2 for p in base_pts)
    syy = sum((p[1] - cy) ** 2 for p in base_pts)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in base_pts)
    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)  # major axis angle in the base XY plane
    lam1 = 0.5 * (sxx + syy) + 0.5 * math.hypot(sxx - syy, 2 * sxy)
    lam2 = 0.5 * (sxx + syy) - 0.5 * math.hypot(sxx - syy, 2 * sxy)
    n = len(base_pts)
    major = 2 * math.sqrt(3 * lam1 / n)
    minor = 2 * math.sqrt(3 * lam2 / n)  # uniform-rectangle extents
    mark_now = T_bf.apply(T_fc.apply(c3))
    print(
        f"mark (solved) {[round(v, 4) for v in mark]}  top-face centre now {[round(v, 4) for v in mark_now]}  d3 {d3 * 1000:.0f} mm"
    )
    print(
        f"top face: {n} pts, major {major * 1000:.0f} mm at {math.degrees(theta):.1f}° (base XY), minor {minor * 1000:.0f} mm — fingers must close across the minor axis"
    )
    # flange X axis heading in base now
    fx_b = T_bf.rotate((1, 0, 0))
    heading = math.degrees(math.atan2(fx_b[1], fx_b[0]))
    minor_dir = theta + math.pi / 2
    yaw = math.degrees(minor_dir) - heading
    yaw = (yaw + 90) % 180 - 90  # smallest turn that lines flange X up with the minor axis (either sense)
    print(
        f"flange X heading {heading:.1f}°, minor-axis heading {math.degrees(minor_dir):.1f}° → yaw the flange by {yaw:.1f}° about its Z (if the fingers open along flange X)"
    )
    st.update(
        {
            "mark": mark_now,
            "mark_solved": mark,
            "major_mm": major * 1000,
            "minor_mm": minor * 1000,
            "theta_deg": math.degrees(theta),
            "yaw_deg": yaw,
            "flange": fl,
            "handeye": he["pose"],
        }
    )
    save(st)
    print("hover would be at z", round(mark_now[2] + FINGERTIP + 0.04, 4))
elif cmd == "hover":
    yaw = float(sys.argv[2]) if len(sys.argv) > 2 else st["yaw_deg"]
    fl = st["flange"]
    T_bf = Transform.from_pose(fl)
    Rz = Transform.from_pose([0, 0, 0, 0, 0, math.radians(yaw)])
    Rn = Transform(T_bf.rotation, (0, 0, 0)).compose(Rz)  # yaw about the flange's own Z
    m = st["mark"]
    zax = Rn.rotate((0, 0, 1))  # the fingertips sit FINGERTIP down the (tilted) tool axis
    tip = [m[0], m[1], m[2] + 0.04]
    pose = [tip[i] - zax[i] * FINGERTIP for i in range(3)] + Rn.to_pose()[3:]
    print("hover pose", [round(v, 4) for v in pose], "yaw", yaw)
    move_to(pose, 0.06)
    st["hover"] = pose
    st["yaw_used"] = yaw
    save(st)
    print(snap("pick_hover"))
elif cmd == "descend":
    below = float(sys.argv[2]) / 1000 if len(sys.argv) > 2 else 0.02
    fl = flange_now()
    m = st["mark"]
    zax = Transform.from_pose(fl).rotate((0, 0, 1))
    tip = [m[0], m[1], m[2] - below]
    pose = [tip[i] - zax[i] * FINGERTIP for i in range(3)] + list(fl[3:])
    print("descend to", round(pose[2], 4))
    move_to(pose, 0.03)
    print(snap("pick_down"))
elif cmd in ("grip", "release"):
    out = subprocess.run(
        [sys.executable, "-m", "urctl", "gripper", "close" if cmd == "grip" else "open", "--force", "80"],
        capture_output=True,
        text=True,
        env={**__import__("os").environ, "UR_HOST": "192.168.3.3"},
    )
    j = json.loads(out.stdout)
    print(cmd, {k: j.get(k) for k in ("ok", "status", "object_detected", "opening_est_mm", "error")})
    print(snap(f"pick_{cmd}"))
elif cmd == "lift":
    up = float(sys.argv[2]) / 1000 if len(sys.argv) > 2 else 0.08
    fl = flange_now()
    pose = [fl[0], fl[1], fl[2] + up, *fl[3:]]
    move_to(pose, 0.05)
    print(snap("pick_lift"))
elif cmd == "move":
    dx, dy = float(sys.argv[2]), float(sys.argv[3])
    fl = flange_now()
    pose = [fl[0] + dx, fl[1] + dy, *fl[2:]]
    move_to(pose, 0.05)
    print(snap("pick_move"))
