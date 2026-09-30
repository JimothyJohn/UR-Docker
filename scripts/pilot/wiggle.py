"""One wiggle step: optional jog (base frame, m) → snapshot → find the white block in the
wrist image → segment it → locate → report its offset from the finger axis (flange frame)."""
# ruff: noqa  — scratch script from the 2026-09-25 hardware session (scripts/pilot/README.md)

import os as _os

HERE = _os.path.dirname(_os.path.abspath(__file__))

import json
import sys
import urllib.request

from perceptronics.pngio import load_png
from urctl.pose import Transform

BASE = "http://127.0.0.1:7621"


def post(path, body):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"}
    )
    try:
        return json.load(urllib.request.urlopen(req, timeout=90))
    except urllib.error.HTTPError as e:
        return {"ok": False, "error": f"HTTP {e.code}: {e.read()[:200]}"}


def white_blobs(png):
    w, h, ch, data = load_png(png)
    step = 4
    cells = set()
    for y in range(0, h, step):
        for x in range(0, w, step):
            i = (y * w + x) * ch
            r, g, b = data[i], data[i + 1], data[i + 2]
            if min(r, g, b) > 185 and max(r, g, b) - min(r, g, b) < 30:
                cells.add((x // step, y // step))
    seen, comps = set(), []
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
        comps.append(comp)
    out = []
    for comp in comps:
        xs = [c[0] * step for c in comp]
        ys = [c[1] * step for c in comp]
        out.append(
            {
                "px": len(comp) * step * step,
                "cx": sum(xs) // len(xs),
                "cy": sum(ys) // len(ys),
                "bbox": (min(xs), min(ys), max(xs), max(ys)),
            }
        )
    return sorted(out, key=lambda b: -b["px"])


name = sys.argv[1]
jog = [float(v) for v in sys.argv[2:5]] if len(sys.argv) >= 5 else None
if jog:
    r = post("/api/robot/jog", {"delta": [*jog, 0, 0, 0], "velocity": 0.03})
    print(
        "jog",
        [round(v * 1000, 1) for v in jog],
        "mm →",
        {k: r.get(k) for k in ("ok", "error")},
        "landed",
        [round(v, 4) for v in (r.get("landed") or [])],
    )
snap = post("/api/snapshot", {"name": name})
blobs = [
    b for b in white_blobs(snap["color_png"]) if b["cx"] > 300 and b["px"] > 2000
]  # skip the velcro straps at the left edge
print("white blobs (right of the rail):", [(b["px"], b["cx"], b["cy"]) for b in blobs[:4]])
if not blobs:
    sys.exit("no block in the wrist view")
b = blobs[0]
seg = post("/api/segment", {"x": b["cx"], "y": b["cy"]})
f = seg.get("features") or {}
d = f.get("depth") or {}
print(
    "segment @",
    (b["cx"], b["cy"]),
    "area",
    f.get("area_px"),
    "bbox",
    f.get("bbox"),
    "depth median",
    round(d.get("median_m", 0), 3),
    "valid",
    d.get("valid_px"),
    "extent mm",
    [round(v * 1000) for v in f.get("extent_m", [])],
)
loc = post("/api/robot/locate", {"standoff_m": 0.20, "reference": "flange"})
pf, pb, fl = loc["point_flange_m"], loc["point_base_m"], loc["flange_pose"]
T = Transform.from_pose(fl)
db = T.rotate((pf[0], pf[1], 0.0))
print(
    f"block in flange frame (mm): x={pf[0] * 1000:.1f} y={pf[1] * 1000:.1f} z={pf[2] * 1000:.1f}   base: {[round(v, 4) for v in pb]}   flange z {fl[2]:.4f}"
)
print(f"→ shift to centre it, base frame (mm): {[round(v * 1000, 1) for v in db]}")
print("views:", [v.get("path") for v in snap.get("views", [])])
