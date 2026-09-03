"""Capture store — RealSenseTrainer's dataset layout, one folder per object.

    <root>/<name>/
        color_00001.png   8-bit RGB (the color image)
        depth_00001.png   16-bit grayscale, raw depth units (× scale_m = metres)
        mask_00001.png    8-bit label (0/255) — present when a segment was taken
        meta_00001.json   intrinsics, depth scale, timestamps, device, features

Numbering resumes from the highest existing index, exactly like the original
``rst/main.py`` did, so a session can be stopped and continued. Names are
restricted to a safe character set: this store is driven from a browser page,
and ``../`` must never turn into a path.
"""

from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass
from pathlib import Path

from .rgbd import DepthImage, Intrinsics, RgbdFrame
from .segment import Mask

_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


def validate_name(name: str) -> str:
    """A capture-set name: 1–64 chars of ``[A-Za-z0-9_.-]``, no leading dot."""
    if not isinstance(name, str) or not _NAME_RE.match(name) or ".." in name:
        raise ValueError(
            f"invalid capture name {name!r}: use letters, digits, '_', '-', '.' (no leading dot)"
        )
    return name


@dataclass
class CaptureStore:
    root: Path

    def __post_init__(self) -> None:
        self.root = Path(self.root)

    def next_index(self, name: str) -> int:
        d = self.root / validate_name(name)
        best = 0
        if d.is_dir():
            for p in d.glob("color_*.png"):
                m = re.fullmatch(r"color_(\d{5})\.png", p.name)
                if m:
                    best = max(best, int(m.group(1)))
        return best + 1

    def save(
        self,
        name: str,
        rgbd: RgbdFrame,
        *,
        mask: Mask | None = None,
        features: dict | None = None,
        device: dict | None = None,
        extra: dict | None = None,
    ) -> dict:
        """Write one capture; returns the file paths + index."""
        from .pngio import encode_png

        name = validate_name(name)
        d = self.root / name
        d.mkdir(parents=True, exist_ok=True)
        idx = self.next_index(name)
        tag = f"{idx:05d}"
        color = rgbd.color if rgbd.color.channels == 3 else rgbd.color.to_rgb()
        paths = {
            "color": d / f"color_{tag}.png",
            "depth": d / f"depth_{tag}.png",
            "meta": d / f"meta_{tag}.json",
        }
        paths["color"].write_bytes(encode_png(color.width, color.height, 3, color.data))
        paths["depth"].write_bytes(rgbd.depth.to_png16())
        if mask is not None:
            paths["mask"] = d / f"mask_{tag}.png"
            paths["mask"].write_bytes(mask.to_png())
        meta = {
            "index": idx,
            "name": name,
            "saved_at": time.time(),
            "width": color.width,
            "height": color.height,
            "depth_scale_m": rgbd.depth.scale_m,
            "aligned": rgbd.aligned,
            "timestamp_ms": rgbd.timestamp_ms,
            "frame_number": rgbd.frame_number,
            "intrinsics": rgbd.intrinsics.as_dict(),
            "device": device or {},
            "features": features,
            "files": {k: p.name for k, p in paths.items()},
        }
        if extra:
            meta["extra"] = extra
        paths["meta"].write_text(json.dumps(meta, indent=2, default=str))
        return {"ok": True, "index": idx, "dir": str(d), "files": {k: str(p) for k, p in paths.items()}}

    def load(self, name: str, index: int) -> tuple[RgbdFrame, Mask | None, dict]:
        """Read a capture back as ``(RgbdFrame, mask_or_None, meta)``."""
        from .frame import Frame
        from .pngio import load_png

        d = self.root / validate_name(name)
        tag = f"{int(index):05d}"
        meta = json.loads((d / f"meta_{tag}.json").read_text())
        w, h, c, buf = load_png(str(d / f"color_{tag}.png"))
        color = Frame(width=w, height=h, data=buf, channels=c)
        depth = DepthImage.from_png16(d / f"depth_{tag}.png", scale_m=float(meta["depth_scale_m"]))
        rgbd = RgbdFrame(
            color=color,
            depth=depth,
            intrinsics=Intrinsics.from_dict(meta["intrinsics"]),
            timestamp_ms=float(meta.get("timestamp_ms", 0.0)),
            frame_number=int(meta.get("frame_number", 0)),
            aligned=bool(meta.get("aligned", True)),
        )
        mask = None
        mp = d / f"mask_{tag}.png"
        if mp.exists():
            mw, mh, _mc, mbuf = _load_gray8(mp)
            mask = Mask(mw, mh, bytes(1 if v else 0 for v in mbuf))
        return rgbd, mask, meta

    def list(self) -> list[dict]:
        out = []
        if not self.root.is_dir():
            return out
        for d in sorted(p for p in self.root.iterdir() if p.is_dir()):
            n = len(list(d.glob("color_*.png")))
            if n:
                out.append({"name": d.name, "count": n})
        return out


def _load_gray8(path: Path) -> tuple[int, int, int, bytes]:
    """Decode the 8-bit grayscale PNG we write for masks (color type 0)."""
    import struct
    import zlib

    data = path.read_bytes()
    width = height = 0
    idat = bytearray()
    i = 8
    while i < len(data):
        (length,) = struct.unpack(">I", data[i : i + 4])
        ctype = data[i + 4 : i + 8]
        body = data[i + 8 : i + 8 + length]
        if ctype == b"IHDR":
            width, height, bit_depth, color_type, _c, _f, _i = struct.unpack(">IIBBBBB", body)
            if (bit_depth, color_type) != (8, 0):
                raise ValueError(f"{path}: expected an 8-bit grayscale mask PNG")
        elif ctype == b"IDAT":
            idat += body
        elif ctype == b"IEND":
            break
        i += 12 + length
    raw = zlib.decompress(bytes(idat))
    out = bytearray()
    for y in range(height):
        if raw[y * (width + 1)] != 0:
            raise ValueError(f"{path}: unsupported PNG filter on row {y}")
        out += raw[y * (width + 1) + 1 : (y + 1) * (width + 1)]
    return width, height, 1, bytes(out)
