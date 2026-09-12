"""Cell profiles — *which robot, which bracket, which ports* as one name.

A cell is a ``KEY=VALUE`` file (``perception/cells/<name>.env``, or any path)
holding the environment the toolkit already reads (``UR_HOST``,
``UR_PLATFORM``, ``UR_*_PORT``, ``PERCEPTION_BRACKET``, ``PERCEPTION_FAKE``…).
Selecting one is ``--cell ur20`` on any ``perception`` entry point, or
``UR_CELL=ur20`` in the shell, so switching from the simulator to the UR3e to
the UR20 is a one-word change instead of five exports — and the doctor, the
cockpit, the MCP server and the CLI all agree on what "the UR20 cell" means.

Precedence is deliberate: **a variable already set in the process environment
wins over the cell file.** The file is a set of defaults for a place, not a
mandate, so ``UR_HOST=10.0.0.9 perception --cell ur20 doctor`` still points at
the host you named.

Shipped cells: ``sim`` (the PolyScope X container on this machine, synthetic
camera), ``ur3`` (UR3e, ``eseries`` bracket print), ``ur20`` (UR20, ``ur20``
print). The real cells ship with ``UR_HOST`` empty — fill in the controller's
IP once; ``perception doctor`` says so until you do.
"""

from __future__ import annotations

import os
from collections.abc import Mapping, MutableMapping
from pathlib import Path

ENV_CELL = "UR_CELL"
CELLS_DIR = Path(__file__).parent / "cells"
# Keys a cell may set. Anything else is refused so a typo (or a stray line in a
# pasted file) never silently becomes a no-op — and nothing outside the
# robot/perception namespace can be injected into the process environment.
ALLOWED_PREFIXES = ("UR_", "PERCEPTION_", "REALSENSE_")


def list_cells() -> list[str]:
    """Names of the shipped cell profiles (``sim``, ``ur3``, ``ur20``…)."""
    return sorted(p.stem for p in CELLS_DIR.glob("*.env"))


def cell_path(name_or_path: str) -> Path:
    """A shipped cell name, or a path to any ``.env``-style file."""
    text = (name_or_path or "").strip()
    if not text:
        raise ValueError("cell name is empty")
    direct = Path(text)
    if direct.suffix == ".env" or os.sep in text or "/" in text:
        if not direct.is_file():
            raise ValueError(f"cell file not found: {text}")
        return direct
    shipped = CELLS_DIR / f"{text.lower()}.env"
    if not shipped.is_file():
        raise ValueError(f"unknown cell {text!r}; one of {list_cells()} or a path to a .env file")
    return shipped


def parse_env_text(text: str) -> dict[str, str]:
    """``KEY=VALUE`` lines → dict. ``#`` comments and blank lines are skipped;
    values may be single- or double-quoted; no shell expansion of any kind."""
    out: dict[str, str] = {}
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].lstrip()
        if "=" not in line:
            raise ValueError(f"line {lineno}: expected KEY=VALUE, got {raw!r}")
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key.isidentifier() or not key.isupper():
            raise ValueError(f"line {lineno}: bad variable name {key!r}")
        if not key.startswith(ALLOWED_PREFIXES):
            allowed = ", ".join(ALLOWED_PREFIXES)
            raise ValueError(f"line {lineno}: {key} is not a cell variable (allowed prefixes: {allowed})")
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        elif " #" in value:
            value = value.split(" #", 1)[0].rstrip()
        out[key] = value
    return out


def load_cell(name_or_path: str) -> dict[str, str]:
    """Read a cell file → its variables (nothing applied)."""
    path = cell_path(name_or_path)
    return parse_env_text(path.read_text(encoding="utf-8"))


def apply_cell(
    name_or_path: str | None,
    env: MutableMapping[str, str] | None = None,
) -> dict:
    """Load ``name_or_path`` (or ``$UR_CELL`` when ``None``) into ``env`` without
    overriding anything already set. Returns what happened::

        {"cell": "ur20", "path": ".../ur20.env", "applied": {...}, "kept": {...}}

    ``applied`` are the variables the cell set; ``kept`` are the ones the
    environment already had (and their existing values). Empty values in the
    file are *not* applied (they are the "fill me in" placeholders) but are
    reported under ``missing`` so the doctor can say what to fill in.
    """
    env = os.environ if env is None else env
    name = name_or_path if name_or_path else env.get(ENV_CELL, "")
    if not name:
        return {"cell": None, "path": None, "applied": {}, "kept": {}, "missing": []}
    path = cell_path(name)
    values = parse_env_text(path.read_text(encoding="utf-8"))
    applied: dict[str, str] = {}
    kept: dict[str, str] = {}
    missing: list[str] = []
    for key, value in values.items():
        if key in env and env[key] != "":
            kept[key] = env[key]
        elif value == "":
            missing.append(key)
        else:
            env[key] = value
            applied[key] = value
    env[ENV_CELL] = path.stem
    return {"cell": path.stem, "path": str(path), "applied": applied, "kept": kept, "missing": missing}


def describe_cell(env: Mapping[str, str] | None = None) -> dict:
    """The cell-relevant slice of the environment, for ``/api/info`` and the
    doctor — never includes anything outside the allowed prefixes."""
    env = os.environ if env is None else env
    keys = (
        ENV_CELL,
        "UR_HOST",
        "UR_PLATFORM",
        "UR_ROBOT_API_PORT",
        "UR_PRIMARY_PORT",
        "UR_RTDE_PORT",
        "UR_DASH_PORT",
        "UR_ROBOT_MODEL",
        "PERCEPTION_BRACKET",
        "PERCEPTION_FAKE",
        "PERCEPTION_T_FLANGE_CAMERA",
        "PERCEPTION_SEGMENT_BACKEND",
        "REALSENSE_LIB",
    )
    return {k: env.get(k) for k in keys if env.get(k) not in (None, "")}
