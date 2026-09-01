"""Parse PolyScope ``.installation`` files — the robot's cell configuration.

An e-Series ``.installation`` is gzipped XML (like ``.urp``; see CLAUDE.md).
It is where the *cell*, as opposed to the program, is described: the active
TCP offset, the payload, safety limits, IO names, the safe-home position.
Reading it answers the questions that bite hardest when moving programs from
URSim to a real robot — above all **"is the active TCP at the flange?"**,
which decides whether native MoveJ/Waypoint nodes are safe to author or
whether motion must be emitted as raw-URScript script nodes (the IK-through-
active-TCP trap documented in CLAUDE.md).

The parser is deliberately tolerant: PolyScope versions move elements around,
so every field is optional and missing sections simply come back absent.

    from urctl.installation import parse_installation_file
    info = parse_installation_file("programs/Foo/Foo.installation")
    info["tcps"]       # [{"name": "TCP", "offset": [0,0,0,0,0,0], "active": True}]
    info["payload"]    # {"mass": 0.0, "center_of_gravity": [0,0,0]}
    info["flange_tcp"] # True when the ACTIVE tcp offset is all-zero
"""

from __future__ import annotations

import gzip
import re
import xml.etree.ElementTree as ET
from pathlib import Path

__all__ = ["parse_installation", "parse_installation_file"]


def _floats(text: str | None) -> list[float] | None:
    """Parse PolyScope's comma-separated float attribute format."""
    if text is None:
        return None
    try:
        return [float(part.strip()) for part in text.split(",") if part.strip() != ""]
    except ValueError:
        return None


def _names(text: str | None) -> list[str] | None:
    """IO-name attributes are comma-joined, blank for unnamed pins."""
    if text is None:
        return None
    return [part.strip() for part in text.split(",")]


# The SafetySettings element embeds an INI-style text blob rather than XML.
# We surface just the headline limits — the numbers an operator/GUI cares
# about — not the whole safety configuration (which is checksummed and owned
# by the safety system, never something this toolkit writes).
_SAFETY_KEYS = {
    "maxTcpSpeed": "max_tcp_speed",
    "maxToolSpeed": "max_tool_speed",
    "maxElbowSpeed": "max_elbow_speed",
    "maxPower": "max_power",
    "maxMomentum": "max_momentum",
    "maxForce": "max_force",
    "maxToolForce": "max_tool_force",
    "maxStoppingTime": "max_stopping_time",
    "maxStoppingDistance": "max_stopping_distance",
}


def _parse_safety_text(text: str) -> dict:
    """Pull the Normal/Reduced headline limits out of the SafetySettings blob.

    The blob is a sequence of ``[Section]`` headers with ``key = value`` lines;
    we track which section we are in and collect known keys from the
    ``SafetyLimits Normal Values`` / ``SafetyLimits Reduced Values`` sections.
    """
    limits: dict[str, dict] = {}
    section = None
    for line in text.splitlines():
        line = line.strip()
        m = re.match(r"\[SafetyLimits (Normal|Reduced) Values\]", line)
        if m:
            section = m.group(1).lower()
            limits.setdefault(section, {})
            continue
        if line.startswith("["):
            section = None
            continue
        if section and "=" in line:
            key, _, value = line.partition("=")
            key = key.strip()
            if key in _SAFETY_KEYS:
                try:
                    limits[section][_SAFETY_KEYS[key]] = float(value.strip())
                except ValueError:
                    pass
    return limits


def parse_installation(data: bytes) -> dict:
    """Parse raw ``.installation`` bytes (gzipped or already-decompressed XML).

    Returns a dict of the cell-relevant facts; every key except ``tcps`` may be
    absent/None when the file predates or omits that section.
    """
    if data[:2] == b"\x1f\x8b":
        data = gzip.decompress(data)
    root = ET.fromstring(data)

    out: dict = {"tcps": []}

    # --- version this installation was saved by ------------------------------
    version = root.find("Version")
    if version is not None:
        out["saved_by"] = {
            "software": version.get("projectName"),
            "version": ".".join(str(version.get(k)) for k in ("major", "minor", "bugfix") if version.get(k)),
            "build": version.get("buildNumber"),
        }

    # --- TCPs (the decisive field for real-robot program authoring) ----------
    tcp_settings = root.find("TCPSettings")
    active_name = tcp_settings.get("activePose") if tcp_settings is not None else None
    if tcp_settings is not None:
        for tcp in tcp_settings.iter("tcp"):
            offset = _floats(tcp.get("offset"))
            out["tcps"].append(
                {
                    "name": tcp.get("name"),
                    "offset": offset,
                    "active": tcp.get("name") == active_name,
                }
            )
    out["active_tcp"] = active_name
    active = next((t for t in out["tcps"] if t["active"]), None)
    if active and active["offset"] is not None:
        # All-zero active offset == tool frame at the flange: native
        # MoveJ/Waypoint nodes are safe. Any non-zero component means MoveJ-node
        # IK resolves through the offset and can reject reachable joint targets.
        out["flange_tcp"] = all(abs(v) < 1e-12 for v in active["offset"])
    else:
        out["flange_tcp"] = None

    # --- payload --------------------------------------------------------------
    payload = root.find(".//PayloadSettings/Payload")
    if payload is not None:
        out["payload"] = {
            "name": payload.get("name"),
            "mass": float(payload.get("mass", "nan")),
            "center_of_gravity": _floats(payload.get("centerOfGravity")),
            "default": payload.get("defaultPayload") == "true",
        }

    # --- mounting (absent on a default flat-mounted cell) ---------------------
    mounting = root.find(".//MountingSettings") or root.find(".//Mounting")
    if mounting is not None:
        out["mounting"] = dict(mounting.attrib)

    # --- safe home ------------------------------------------------------------
    safe_home = root.find("SafeHomeSettings")
    if safe_home is not None:
        out["safe_home"] = {
            "enabled": safe_home.get("enabled") == "true",
            "joints": _floats(safe_home.get("position")),
        }

    # --- IO names (only the pins someone bothered to name) --------------------
    ios = root.find("IOs")
    if ios is not None:
        io_names: dict[str, list] = {}
        for element, key in (
            ("DigitalInputNames", "digital_in"),
            ("DigitalOutputNames", "digital_out"),
            ("ToolDigitalInputNames", "tool_digital_in"),
            ("ToolDigitalOutputNames", "tool_digital_out"),
            ("AnalogInputNames", "analog_in"),
            ("AnalogOutputNames", "analog_out"),
            ("ConfigurableInputNames", "configurable_in"),
            ("ConfigurableOutputNames", "configurable_out"),
        ):
            node = ios.find(element)
            names = _names(node.get("value")) if node is not None else None
            if names and any(names):
                io_names[key] = [{"pin": i, "name": n} for i, n in enumerate(names) if n]
        if io_names:
            out["io_names"] = io_names

    # --- headline safety limits ----------------------------------------------
    safety = root.find("SafetySettings")
    if safety is not None and safety.text:
        limits = _parse_safety_text("".join(safety.itertext()))
        if limits:
            out["safety_limits"] = limits

    # --- default/auto-load program -------------------------------------------
    default_prog = root.find("DefaultProgramSettings")
    if default_prog is not None:
        out["default_program"] = dict(default_prog.attrib)

    return out


def parse_installation_file(path: str | Path) -> dict:
    """Parse a ``.installation`` file from disk."""
    return parse_installation(Path(path).read_bytes())
