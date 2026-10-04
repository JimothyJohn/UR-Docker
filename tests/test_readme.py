"""The root README's install steps name the files in ``urcap/dist/`` — exactly them."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_the_readme_names_the_committed_polyscope_5_urcap_and_its_magic_file():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    built = sorted(p.name for p in (ROOT / "urcap" / "dist").glob("perceptronic-ps5-*.urcap"))
    assert len(built) == 1, built
    named = set(re.findall(r"perceptronic-ps5-[0-9.]+\.urcap", readme))
    assert named == set(built), f"README.md names {sorted(named)}, urcap/dist has {built}"
    for name in (*built, "urmagic_perceptronic.sh"):
        assert (ROOT / "urcap" / "dist" / name).is_file()
        assert f"raw/main/urcap/dist/{name}" in readme, f"README.md has no download link for {name}"
