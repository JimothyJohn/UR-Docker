# ruff: noqa  — scratch script from the 2026-09-25 hardware session (scripts/pilot/README.md)
import os as _os

HERE = _os.path.dirname(_os.path.abspath(__file__))
import json
import math
import sys

src = open(HERE + "/pick.py").read()
exec(src.split("cmd = sys.argv[1]")[0])
st = load()
rob = get("/api/robot")["robot"]
T_fc = Transform.from_pose(rob["handeye"]["flange_to_color_pose"])
fl = flange_now()
T_bf = Transform.from_pose(fl)
hdr, w, h, ch, rgb, depth = frame()
others = []
for b in white_blobs(w, h, ch, rgb):
    if b["px"] < 400:
        continue
    tf = top_face_click(w, h, ch, rgb, depth, b["bbox"], hdr["depth_scale_m"])
    if tf is None:
        continue
    p = T_bf.apply(T_fc.apply(tf[2]))
    others.append(p)
    print("white thing at base", [round(v, 3) for v in p], "px", (b["cx"], b["cy"]), b["px"])
pick = st["mark"]
print("pick point", [round(v, 3) for v in pick])
best = None
for k in range(12):
    a = 2 * math.pi * k / 12
    c = [pick[0] + 0.09 * math.cos(a), pick[1] + 0.09 * math.sin(a)]
    r = math.hypot(c[0], c[1])
    if r < 0.30 or r > 0.42:
        continue
    dmin = min([math.hypot(c[0] - o[0], c[1] - o[1]) for o in others] or [1.0])
    if best is None or dmin > best[0]:
        best = (dmin, c, math.degrees(a))
dmin, c, ang = best
print(f"place at {[round(v, 3) for v in c]} (bearing {ang:.0f}°, nearest other thing {dmin * 1000:.0f} mm)")
if "--go" in sys.argv:
    zax = T_bf.rotate((0, 0, 1))
    over = [c[0] - zax[0] * FINGERTIP, c[1] - zax[1] * FINGERTIP, fl[2], *fl[3:]]
    move_to(over, 0.06)
    print(snap("place_over"))
    down = [
        over[0],
        over[1],
        (pick[2] - 0.012) - zax[2] * FINGERTIP + 0.0,
        *fl[3:],
    ]  # same grasp height as the pick
    move_to(down, 0.03)
    import os
    import subprocess

    out = subprocess.run(
        [sys.executable, "-m", "urctl", "gripper", "open"],
        capture_output=True,
        text=True,
        env={**os.environ, "UR_HOST": "192.168.3.3"},
    )
    print("release", json.loads(out.stdout).get("status"))
    print(snap("place_release"))
    move_to([down[0], down[1], down[2] + 0.08, *fl[3:]], 0.05)
    print(snap("place_up"))
