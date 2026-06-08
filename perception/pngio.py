"""Minimal, dependency-free PNG decoder.

Just enough to load a real test image into a :class:`~perception.frame.Frame`
without pulling in Pillow/numpy — keeping the package core (and its tests)
third-party-free, the same property ``urctl`` holds. Supports the common case
this repo needs: 8-bit, non-interlaced, color type 2 (RGB) or 6 (RGBA), which is
what camera captures and most tooling emit. Anything else raises a clear error
pointing at the optional extras.

PNG is zlib-compressed, per-scanline-filtered raw samples. Decompression is C
(``zlib``); only the unfiltering is Python, so cost scales with pixel count —
fine for a one-shot fixture load, after which callers downsample for the
pure-Python pipeline.
"""

from __future__ import annotations

import struct
import zlib

_SIG = b"\x89PNG\r\n\x1a\n"


def load_png(path: str) -> tuple[int, int, int, bytes]:
    """Decode ``path`` to ``(width, height, channels, rgb_or_rgba_bytes)``.

    Channels is 3 (RGB, color type 2) or 4 (RGBA, color type 6).
    """
    with open(path, "rb") as f:
        data = f.read()
    if data[:8] != _SIG:
        raise ValueError(f"{path}: not a PNG (bad signature)")

    width = height = bit_depth = color_type = interlace = 0
    idat = bytearray()
    i = 8
    while i < len(data):
        (length,) = struct.unpack(">I", data[i : i + 4])
        ctype = data[i + 4 : i + 8]
        body = data[i + 8 : i + 8 + length]
        if ctype == b"IHDR":
            width, height, bit_depth, color_type, _comp, _filt, interlace = struct.unpack(">IIBBBBB", body)
        elif ctype == b"IDAT":
            idat += body
        elif ctype == b"IEND":
            break
        i += 12 + length  # length + type + data + CRC

    if bit_depth != 8:
        raise ValueError(f"{path}: only 8-bit PNGs supported (got bit depth {bit_depth})")
    if interlace != 0:
        raise ValueError(f"{path}: interlaced PNGs are not supported")
    if color_type == 2:
        channels = 3
    elif color_type == 6:
        channels = 4
    else:
        raise ValueError(
            f"{path}: unsupported color type {color_type} (need 2=RGB or 6=RGBA); "
            "install Pillow via `.[perception]` and use a converter for others"
        )

    raw = zlib.decompress(bytes(idat))
    pixels = _unfilter(raw, width, height, channels)
    return width, height, channels, pixels


def _unfilter(raw: bytes, width: int, height: int, channels: int) -> bytes:
    """Reverse PNG scanline filters (None/Sub/Up/Average/Paeth) -> raw samples."""
    stride = width * channels
    out = bytearray(height * stride)
    prev = bytearray(stride)  # previous reconstructed scanline (zeros for row 0)
    pos = 0  # cursor into `raw`
    for y in range(height):
        ft = raw[pos]
        pos += 1
        line = bytearray(raw[pos : pos + stride])
        pos += stride
        bpp = channels  # bytes per pixel (8-bit)

        if ft == 0:  # None
            pass
        elif ft == 1:  # Sub
            for x in range(bpp, stride):
                line[x] = (line[x] + line[x - bpp]) & 0xFF
        elif ft == 2:  # Up
            for x in range(stride):
                line[x] = (line[x] + prev[x]) & 0xFF
        elif ft == 3:  # Average
            for x in range(stride):
                a = line[x - bpp] if x >= bpp else 0
                line[x] = (line[x] + ((a + prev[x]) >> 1)) & 0xFF
        elif ft == 4:  # Paeth
            for x in range(stride):
                a = line[x - bpp] if x >= bpp else 0
                b = prev[x]
                c = prev[x - bpp] if x >= bpp else 0
                p = a + b - c
                pa = p - a if p >= a else a - p
                pb = p - b if p >= b else b - p
                pc = p - c if p >= c else c - p
                if pa <= pb and pa <= pc:
                    pr = a
                elif pb <= pc:
                    pr = b
                else:
                    pr = c
                line[x] = (line[x] + pr) & 0xFF
        else:
            raise ValueError(f"unknown PNG filter type {ft}")

        out[y * stride : (y + 1) * stride] = line
        prev = line
    return bytes(out)
