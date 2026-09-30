"""Mark-less hand-eye calibration: orbit the wrist about a block's top-face centre,
click that centre in each view (top face = the nearest white pixels), solve."""
# ruff: noqa  — scratch script from the 2026-09-25 hardware session (scripts/pilot/README.md)

import os as _os

HERE = _os.path.dirname(_os.path.abspath(__file__))

import json
import math
import struct
import sys
import time
import urllib.error
import urllib.request
import zlib

from perceptronics.pngio import load_png
from urctl.pose import Transform, matrix_to_rotvec, rotvec_to_matrix

BASE = "http://127.0.0.1:7621"


def post(path, body):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"}
    )
    try:
        return json.load(urllib.request.urlopen(req, timeout=120))
    except urllib.error.HTTPError as e:
        return {"ok": False, "error": f"HTTP {e.code}: {e.read()[:300]}"}


def get_bytes(path):
    return urllib.request.urlopen(BASE + path, timeout=30).read()


def rgbd():
    b = get_bytes("/api/rgbd")
    assert b[:4] == b"RGBD"
    hl = struct.unpack(">I", b[4:8])[0]
    hdr = json.loads(b[8 : 8 + hl])
    i = 8 + hl
    pl = struct.unpack(">I", b[i : i + 4])[0]
    png = b[i + 4 : i + 4 + pl]
    i += 4 + pl
    dl = struct.unpack(">I", b[i : i + 4])[0]
    depth = zlib.decompress(b[i + 4 : i + 4 + dl])
    w, h, ch, rgb = load_png(png)  # from bytes? fall back to a temp file
    return hdr, w, h, rgb, depth


def load_png_bytes(png):
    p = HERE + "/_frame.png"
    open(p, "wb").write(png)
    return load_png(p)


def frame():
    b = get_bytes("/api/rgbd")
    hl = struct.unpack(">I", b[4:8])[0]
    hdr = json.loads(b[8 : 8 + hl])
    i = 8 + hl
    pl = struct.unpack(">I", b[i : i + 4])[0]
    png = b[i + 4 : i + 4 + pl]
    i += 4 + pl
    dl = struct.unpack(">I", b[i : i + 4])[0]
    depth = zlib.decompress(b[i + 4 : i + 4 + dl])
    w, h, ch, rgb = load_png_bytes(png)
    return hdr, w, h, ch, rgb, depth


def white_blobs(w, h, ch, rgb, step=4):
    cells = set()
    for y in range(0, h, step):
        for x in range(0, w, step):
            i = (y * w + x) * ch
            r, g, b = rgb[i], rgb[i + 1], rgb[i + 2]
            if min(r, g, b) > 185 and max(r, g, b) - min(r, g, b) < 30:
                cells.add((x // step, y // step))
    seen, out = set(), []
    for c in cells:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            cx, cy = stack.pop()
            comp.append((cx, cy))
            for n in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if n in cells and n not in seen:
                    seen.add(n)
                    stack.append(n)
        xs = [q[0] * step for q in comp]
        ys = [q[1] * step for q in comp]
        out.append(
            {
                "px": len(comp) * step * step,
                "cx": sum(xs) // len(xs),
                "cy": sum(ys) // len(ys),
                "bbox": (min(xs), min(ys), max(xs) + step, max(ys) + step),
            }
        )
    return out


def top_face_centre(w, h, ch, rgb, depth, bbox, scale):
    """centroid of the white pixels in bbox whose depth is within 12 mm of the nearest white pixel"""
    x0, y0, x1, y1 = bbox
    pts = []
    for y in range(y0, min(y1, h)):
        for x in range(x0, min(x1, w)):
            i = (y * w + x) * ch
            r, g, b = rgb[i], rgb[i + 1], rgb[i + 2]
            if min(r, g, b) > 185 and max(r, g, b) - min(r, g, b) < 30:
                d = depth[2 * (y * w + x)] | (depth[2 * (y * w + x) + 1] << 8)
                if d:
                    pts.append((x, y, d * scale))
    if not pts:
        return None
    zmin = min(p[2] for p in pts)
    top = [p for p in pts if p[2] <= zmin + 0.012]
    return (sum(p[0] for p in top) // len(top), sum(p[1] for p in top) // len(top), zmin, len(top), len(pts))


def find_block(prev_px):
    hdr, w, h, ch, rgb, depth = frame()
    blobs = [b for b in white_blobs(w, h, ch, rgb) if b["px"] > 1500 and b["cx"] > 200]
    if not blobs:
        return None, hdr
    b = min(blobs, key=lambda b: math.hypot(b["cx"] - prev_px[0], b["cy"] - prev_px[1]))
    tf = top_face_centre(w, h, ch, rgb, depth, b["bbox"], hdr["depth_scale_m"])
    return (b, tf), hdr


def orbit_pose(flange, mark, axis, deg):
    """rotate the flange pose about `axis` (base frame) through `mark`"""
    rv = [a * math.radians(deg) for a in axis]
    R = Transform(rotvec_to_matrix(rv), (0.0, 0.0, 0.0))
    T = Transform.from_pose(flange)
    rel = [T.translation[i] - mark[i] for i in range(3)]
    p = R.apply(rel)
    p = [p[i] + mark[i] for i in range(3)]
    Rn = R.compose(Transform(T.rotation, (0.0, 0.0, 0.0)))
    return [*p, *matrix_to_rotvec(Rn.rotation)]


home = [-0.2325, 0.2776, -0.040, -2.9309, 0.151, -0.1047]
mark = [-0.2135, 0.2299, -0.2593]
orbits = [
    ("home", None, 0),
    ("X+15", (1, 0, 0), 15),
    ("X-15", (1, 0, 0), -15),
    ("Y+15", (0, 1, 0), 15),
    ("Y-15", (0, 1, 0), -15),
    ("Z+30", (0, 0, 1), 30),
    ("Z-30", (0, 0, 1), -30),
]
heights = [-0.040, 0.060, 0.160]  # camera ~0.20 / 0.30 / 0.40 m from the block top
views = [(f"{name}@{z:+.2f}", axis, deg, z) for z in heights for (name, axis, deg) in orbits]
if len(sys.argv) > 1:
    views = [v for v in views if any(a in v[0] for a in sys.argv[1:])]
print("reset:", post("/api/cal/reset", {}).get("ok"))
px = (448, 105)
kept = 0
for name, axis, deg, z in views:
    base_pose = [home[0], home[1], z, *home[3:]]
    pose = base_pose if axis is None else orbit_pose(base_pose, mark, axis, deg)
    r = post("/api/robot/move", {"pose": pose, "velocity": 0.08, "tcp": [0] * 6})
    if not r.get("ok"):
        print(
            f"[{name}] move refused/failed: {r.get('error')} viol={(r.get('safety') or {}).get('violations')} pstop={r.get('protective_stop')}"
        )
        if r.get("protective_stop"):
            post("/api/robot/bring_up", {})
            time.sleep(1)
        continue
    time.sleep(0.7)
    found, hdr = find_block(px)
    if not found or found[1] is None:
        print(f"[{name}] block not found in view")
        continue
    b, (tx, ty, zmin, ntop, nall) = found
    x0, y0, x1, y1 = b["bbox"]
    W, H = hdr["width"], hdr["height"]
    px = (b["cx"], b["cy"])
    if x0 <= 8 or y0 <= 8 or x1 >= W - 8 or y1 >= H - 8:
        print(f"[{name}] block clipped at the frame edge (bbox {b['bbox']}) — skipped")
        continue
    v = post("/api/cal/view", {"x": tx, "y": ty})
    pc = (v.get("views") or [{}])[-1].get("point_cam") if v.get("ok") else None
    kept += 1 if v.get("ok") else 0
    print(
        f"[{name}] blob {b['px']}px@{px} top@({tx},{ty}) zmin={zmin:.3f} top/all={ntop}/{nall} → ok={v.get('ok')} cam={[round(c, 4) for c in pc] if pc else v.get('error')}"
    )
print("views kept:", kept)
res = post("/api/cal/solve", {})
print(
    "solve:",
    json.dumps(
        {
            k: res.get(k)
            for k in (
                "ok",
                "error",
                "rms_m",
                "views",
                "rotation_diversity_deg",
                "mark_base",
                "delta_from_seed_mm",
                "warnings",
                "iterations",
            )
        },
        default=str,
    ),
)
print("per-view residual mm:", [round(v * 1000, 1) for v in res.get("per_view_residual_m") or []])
print("env_line:", res.get("env_line"))
