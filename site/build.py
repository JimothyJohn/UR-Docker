"""Assemble the product page (perceptronics.advin.io) into ``site/_build/``.

The page's facts come from the repo, never from the template: the two URCaps and their
versions are the committed ``urcap/dist/`` files (copied to ``downloads/`` with their
sha256 and size), the supported PolyScope ranges are the CI matrices' own lists
(``urcap/ps5_matrix.py``, ``urcap/psx_matrix.py``), and the screenshots are the rendered
pendant screens in ``urcap/perceptronic-ps5/screens/``. A ``{{NAME}}`` left in the output
is an error. Stdlib only: ``python3 site/build.py [--out DIR]``.

The printed documents — the one-page datasheet (``public/datasheet.html``) and the UR
Quickstart guide (``public/quickstart-ur.html``) — are printed to PDF by a local Chrome
(``python3 site/build.py --pdf``; CI has no browser, so each PDF is committed in
``site/print/`` next to the sha256 of the page it was printed from). A test holds them
together: change a page, or ship a new URCap, and it says to print it again.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

SITE = Path(__file__).resolve().parent
REPO = SITE.parent
PUBLIC = SITE / "public"
DIST = REPO / "urcap" / "dist"
SCREENS = REPO / "urcap" / "perceptronic-ps5" / "screens"
# the pendant screens the page shows (all 1000 x 560)
SCREEN_NAMES = ("pick-part.png", "pick-options.png", "installation-areas.png")
PRINT_DIR = SITE / "print"
# page -> the PDF it is printed to (committed in PRINT_DIR with "<page>.sha256" beside it)
PRINTS = {
    "datasheet.html": "perceptronics-datasheet.pdf",
    "quickstart-ur.html": "perceptronics-ur-quickstart.pdf",
}
SHEET_PDF = PRINTS["datasheet.html"]
QUICKSTART_PDF = PRINTS["quickstart-ur.html"]
# the datasheet's revision date: move it when its figures or wording change
SHEET_DATE = "2026-10-03"
# the UR Quickstart's revision date
QUICKSTART_DATE = "2026-10-03"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
PLACEHOLDER = re.compile(r"\{\{([A-Z0-9_]+)\}\}")


def _module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[path.stem] = mod
    spec.loader.exec_module(mod)
    return mod


def _one(pattern: str) -> tuple[Path, str]:
    """The single ``urcap/dist`` file matching ``pattern`` and the version in its name."""
    rx = re.compile(pattern)
    found = [(p, m.group(1)) for p in sorted(DIST.iterdir()) if (m := rx.fullmatch(p.name))]
    if len(found) != 1:
        raise SystemExit(f"expected exactly one urcap/dist file matching {pattern}, found {len(found)}")
    return found[0]


def _minor(release: str) -> tuple[int, int]:
    major, minor = release.split(".")[:2]
    return int(major), int(minor)


def _span(releases: list[str]) -> str:
    lo, hi = min(releases, key=_minor), max(releases, key=_minor)
    return f"{'.'.join(map(str, _minor(lo)))} to {'.'.join(map(str, _minor(hi)))}"


def _kb(path: Path) -> str:
    return f"{round(path.stat().st_size / 1000)} kB"


def facts() -> dict[str, str]:
    ps5, ps5_version = _one(r"perceptronic-ps5-(\d+\.\d+\.\d+)\.urcap")
    psx, psx_version = _one(r"perceptronic-(\d+\.\d+\.\d+)\.urcapx")
    ps5_matrix = _module(REPO / "urcap" / "ps5_matrix.py")
    psx_matrix = _module(REPO / "urcap" / "psx_matrix.py")
    return {
        "PS5_FILE": ps5.name,
        "PS5_VERSION": ps5_version,
        "PS5_SHA256": hashlib.sha256(ps5.read_bytes()).hexdigest(),
        "PS5_SIZE": _kb(ps5),
        "PS5_RANGE": _span(list(ps5_matrix.MATRIX)),
        "PSX_FILE": psx.name,
        "PSX_VERSION": psx_version,
        "PSX_SHA256": hashlib.sha256(psx.read_bytes()).hexdigest(),
        "PSX_SIZE": _kb(psx),
        "PSX_RANGE": _span(list(psx_matrix.RELEASES)),
        "SHEET_DATE": SHEET_DATE,
        "SHEET_PDF": SHEET_PDF,
        "QUICKSTART_DATE": QUICKSTART_DATE,
        "QUICKSTART_PDF": QUICKSTART_PDF,
    }


def render(text: str, values: dict[str, str]) -> str:
    def sub(m: re.Match[str]) -> str:
        if m.group(1) not in values:
            raise SystemExit(f"site template names an unknown value: {m.group(0)}")
        return values[m.group(1)]

    return PLACEHOLDER.sub(sub, text)


def build(out: Path) -> Path:
    values = facts()
    if out.exists():
        shutil.rmtree(out)
    (out / "downloads").mkdir(parents=True)
    (out / "screens").mkdir()
    for page in sorted(PUBLIC.glob("*.html")):
        (out / page.name).write_text(render(page.read_text(encoding="utf-8"), values), encoding="utf-8")
    for name in (values["PS5_FILE"], values["PSX_FILE"]):
        shutil.copyfile(DIST / name, out / "downloads" / name)
    for name in SCREEN_NAMES:
        shutil.copyfile(SCREENS / name, out / "screens" / name)
    for pdf in PRINTS.values():
        if (PRINT_DIR / pdf).is_file():
            shutil.copyfile(PRINT_DIR / pdf, out / "downloads" / pdf)
    return out


def stamp_path(page: str) -> Path:
    return PRINT_DIR / f"{page}.sha256"


def page_digest(out: Path, page: str) -> str:
    """sha256 of a built page, line endings normalised (Windows checkouts)."""
    text = (out / page).read_text(encoding="utf-8").replace("\r\n", "\n")
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def print_page(out: Path, page: str, chrome: str) -> Path:
    """Print one built page to ``site/print/`` with headless Chrome and stamp it."""
    if not Path(chrome).is_file():
        raise SystemExit(f"no Chrome at {chrome}: set CHROME to a Chrome or Chromium binary")
    PRINT_DIR.mkdir(exist_ok=True)
    pdf = PRINT_DIR / PRINTS[page]
    pdf.unlink(missing_ok=True)
    profile = out.parent / "_chrome"
    # headless Chrome does not always exit after printing: wait for the file, then stop it
    proc = subprocess.Popen(
        [
            chrome,
            "--headless=new",
            "--disable-gpu",
            "--no-first-run",
            f"--user-data-dir={profile}",
            "--no-pdf-header-footer",
            f"--print-to-pdf={pdf}",
            (out / page).as_uri(),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and proc.poll() is None:
            if pdf.is_file() and pdf.stat().st_size > 0:
                time.sleep(1.0)  # let it finish writing
                break
            time.sleep(0.2)
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        shutil.rmtree(profile, ignore_errors=True)
    if not pdf.is_file() or not pdf.read_bytes().startswith(b"%PDF"):
        raise SystemExit(f"Chrome did not write {pdf.name}")
    stamp_path(page).write_text(page_digest(out, page) + "\n", encoding="utf-8")
    shutil.copyfile(pdf, out / "downloads" / pdf.name)
    return pdf


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=SITE / "_build")
    ap.add_argument("--pdf", action="store_true", help="also print the PDFs (needs Chrome)")
    args = ap.parse_args(argv)
    out = build(args.out)
    if args.pdf:
        for page in PRINTS:
            print(print_page(out, page, os.environ.get("CHROME", CHROME)))
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
