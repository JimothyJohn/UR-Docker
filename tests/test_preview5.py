"""``urcap/preview5.py``: the URCap's screens on a desktop. Runs the real launcher headless
(``--snapshot``) and reads the picture it wrote — the simulated camera computer's parts on the
picture, and the NO CAMERA CONNECTED card when there is no camera computer."""

from __future__ import annotations

import shutil
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "urcap"))
import preview5  # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("javac") is None, reason="javac is not installed")


def pixels(png: Path) -> tuple[int, int, bytes]:
    """RGB of an 8-bit PNG (what ImageIO writes: no interlace), stdlib only."""
    data = png.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    i, idat, w = 8, b"", 0
    while i < len(data):
        n, kind = struct.unpack(">I4s", data[i : i + 8])
        body = data[i + 8 : i + 8 + n]
        if kind == b"IHDR":
            w, h, depth, color = struct.unpack(">IIBB", body[:10])
            assert depth == 8 and color in (2, 6)
            bpp = 3 if color == 2 else 4
        elif kind == b"IDAT":
            idat += body
        i += 12 + n
    raw, out, prev = zlib.decompress(idat), bytearray(), bytearray(w * bpp)
    stride = w * bpp
    for y in range(h):
        f, line = raw[y * (stride + 1)], bytearray(raw[y * (stride + 1) + 1 : (y + 1) * (stride + 1)])
        for x in range(stride):
            a = line[x - bpp] if x >= bpp else 0
            b, c = prev[x], prev[x - bpp] if x >= bpp else 0
            if f == 1:
                line[x] = (line[x] + a) & 255
            elif f == 2:
                line[x] = (line[x] + b) & 255
            elif f == 3:
                line[x] = (line[x] + (a + b) // 2) & 255
            elif f == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                line[x] = (line[x] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 255
        prev = line
        for x in range(w):
            out += line[x * bpp : x * bpp + 3]
    return w, h, bytes(out)


def count(img: tuple[int, int, bytes], test, box=None) -> int:
    w, h, rgb = img
    x0, y0, x1, y1 = box or (0, 0, w, h)
    return sum(
        1
        for y in range(y0, y1, 2)
        for x in range(x0, x1, 2)
        if test(*rgb[3 * (y * w + x) : 3 * (y * w + x) + 3])
    )


GREEN = lambda r, g, b: g > 180 and r < 200 and b < 200 and g - r > 30  # noqa: E731 — a pickable part's outline (Ui.PART)
AMBER = lambda r, g, b: r > 240 and 180 < g < 215 and b < 90  # noqa: E731 — a near miss's outline (Ui.JAW)


def test_the_preview_shows_parts_green_and_near_misses_yellow_and_says_it_is_simulated(tmp_path):
    out = tmp_path / "preview.png"
    assert preview5.main(["--snapshot", str(out)]) == 0
    img = pixels(out)
    assert img[:2] == (1280, 772)
    picture = (8, 60, 960, 760)
    # the seven parts it will pick are green (0.8.0); the three it won't are outlined in yellow
    assert count(img, GREEN, picture) > 300
    assert count(img, AMBER, picture) > 150
    # the simulated picture's NO CAMERA CONNECTED banner (red, 0xb4 0x23 0x23)
    assert count(img, lambda r, g, b: r == 0xB4 and g == 0x23 and b == 0x23, picture) > 2000


def test_the_depth_view_is_the_heatmap_with_the_same_outlines(tmp_path):
    out = tmp_path / "depth.png"
    assert preview5.main(["--snapshot", str(out), "--view", "depth"]) == 0
    img = pixels(out)
    picture = (8, 150, 960, 670)
    # no camera picture: no banner; the table and the parts' tops are two ends of the ramp
    assert count(img, lambda r, g, b: r == 0xB4 and g == 0x23 and b == 0x23, picture) == 0
    far = count(img, lambda r, g, b: r > 150 and g > 180 and b < 120, picture)  # the table: yellow-green
    near = count(img, lambda r, g, b: r < 90 and g < 60 and 40 < b < 120, picture)  # the tops: deep violet
    assert far > 20000 and near > 2000
    assert count(img, AMBER, picture) > 150


def test_without_a_camera_computer_the_picture_is_the_no_camera_card(tmp_path):
    out = tmp_path / "nocam.png"
    assert preview5.main(["--cockpit", "http://127.0.0.1:1", "--snapshot", str(out)]) == 0
    img = pixels(out)
    picture = (8, 60, 960, 760)
    # the card's red frame and no green part overlay anywhere
    assert count(img, lambda r, g, b: r > 190 and g < 90 and b < 90, picture) > 100
    assert count(img, GREEN, picture) == 0 and count(img, AMBER, picture) == 0
    # ... and it says what to check, not an exception: the card is taller than its two-line title
    card = [
        y
        for y in range(60, 760, 2)
        if count(img, lambda r, g, b: r > 190 and g < 90 and b < 90, (300, y, 400, y + 2))
    ]
    assert max(card) - min(card) > 120
