#!/usr/bin/env python3
"""
urp_convert.py — Convert between PolyScope `.urp` (program) files and
URScript `.script` files for Universal Robots e-Series controllers.

WHY THIS EXISTS
---------------
PolyScope's "Load Program" file browser filters to *.urp by default, so a
plain .script file is invisible unless you flip the filter. This tool wraps
a .script into a one-node .urp so it appears in the default file list, and
goes the other way for inspection.

FORMAT
------
A .urp is a gzipped XML document. Top-level element is <URProgram> with
attributes describing the version it was authored in and the robot model.
Children live under <children> as program-tree nodes; the simplest node is
<Script type="Code"> whose <cachedContents> holds raw URScript text. (This
matches the sibling format used for `default.installation`, which is also
gzipped XML with PolyScope-flavored tags.)

USAGE
-----
    urp_convert.py to-urp     foo.script  [foo.urp]
    urp_convert.py to-script  foo.urp     [foo.script]

By default the output filename swaps the extension. Use `-` for stdin/stdout.

CAVEATS
-------
The .urp produced by `to-urp` is a single-Script-node wrapper. PolyScope
will load and run it, but features that need typed program nodes (Waypoint,
MoveJ with teach pendant calibration, etc.) cannot be reconstructed from a
flat .script — by the time URScript is generated, the structured tree is
gone. For that direction, .urp is the source of truth.
"""

from __future__ import annotations

import argparse
import gzip
import sys
from pathlib import Path
from xml.etree import ElementTree as ET
from xml.sax.saxutils import escape, unescape


def _attr(value: str) -> str:
    """Escape a value for inclusion as the content of an XML attribute.

    `xml.sax.saxutils.escape` covers `<`, `>`, `&` but not the quote
    characters that would let an attacker break out of an attribute
    delimiter. We always emit attributes inside double quotes, so escape
    that one too.
    """
    return escape(value, {'"': "&quot;"})


# PolyScope embeds version metadata in the .urp header. Values picked to match
# URSim 5.12.5 (image used by this repo's docker-compose); other 5.x versions
# accept these unchanged.
DEFAULT_CREATED_IN = "5.12.5.10626004"
DEFAULT_LAST_SAVED_IN = "5.12.5.10626004"


def script_to_urp(
    script_text: str,
    *,
    name: str,
    installation: str = "default",
    directory: str = "/programs",
    source_file: str | None = None,
    run_only_once: bool = True,
    created_in: str = DEFAULT_CREATED_IN,
    last_saved_in: str = DEFAULT_LAST_SAVED_IN,
) -> bytes:
    """Wrap URScript text in a minimal PolyScope .urp (returns gzipped XML).

    `run_only_once=True` matches PolyScope's "Run Once" checkbox: the
    program completes after one pass through MainProgram. With False
    (PolyScope's UI default for tend/cycle programs) the program loops
    forever — fine for HelpfulBot-style cycle programs, terrible for
    one-shot demos and CI fixtures.

    PolyScope's program loader is picky. The minimum schema it will accept
    (derived empirically by diffing against real saved .urp files and
    reading polyscope.log error messages):

      * `installation` + `installationRelativePath` attrs naming a sibling
        .installation file (without extension). PolyScope refuses to load
        any program without a matching installation file in the same dir.
      * A `<kinematics>` block. Even with `validChecksum="false"`, the
        block itself must be present, or `ProgramRootNodeLoad` returns
        null with the unhelpful "unknown failure" error.
      * A `<MainProgram>` wrapper inside top-level `<children>`. Program
        nodes (Script, Move, Waypoint…) live inside MainProgram's own
        `<children>`, not directly under URProgram.
      * `<Script type="File">` with both `<cachedContents>` (inline script
        text) and `<file resolves-to="file">` (path the script was loaded
        from). `type="Code"` is for the inline "Script Code" tree node
        and doesn't carry source file metadata.
    """
    body = escape(script_text)
    src = source_file or f"{directory}/{name}.script"

    xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
        f'<URProgram name="{_attr(name)}" installation="{_attr(installation)}" '
        f'installationRelativePath="{_attr(installation)}" '
        f'directory="{_attr(directory)}" '
        f'createdIn="{_attr(created_in)}" lastSavedIn="{_attr(last_saved_in)}" '
        f'robotSerialNumber="">\n'
        # Identity-ish kinematics with the checksum check disabled. URSim
        # accepts this; real hardware would prefer values matching the
        # installed robot's actual calibration.
        '  <kinematics status="NOT_LINEARIZED" validChecksum="false">\n'
        '    <deltaTheta value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>\n'
        '    <a value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>\n'
        '    <d value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>\n'
        '    <alpha value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>\n'
        '    <jointChecksum value="0, 0, 0, 0, 0, 0"/>\n'
        "  </kinematics>\n"
        "  <children>\n"
        f'    <MainProgram runOnlyOnce="{str(run_only_once).lower()}" InitVariablesNode="false">\n'
        "      <children>\n"
        '        <Script type="File">\n'
        f"          <cachedContents>{body}</cachedContents>\n"
        f'          <file resolves-to="file">{escape(src)}</file>\n'  # element text — escape() is sufficient
        "        </Script>\n"
        "      </children>\n"
        "    </MainProgram>\n"
        "  </children>\n"
        "</URProgram>\n"
    )
    return gzip.compress(xml.encode("utf-8"))


def urp_to_script(urp_bytes: bytes) -> str:
    """Extract URScript text from a .urp file.

    A .urp has two distinct regions:
      * URP-level header (`<kinematics>`, version attrs, etc.) — we ignore.
      * Program tree, rooted at `<MainProgram>` inside the URP's top-level
        `<children>`. Each program-tree node represents one row in the
        PolyScope program tree; the only one we can roundtrip to URScript
        is `<Script>` (its `<cachedContents>` holds the URScript text).
        Other node types (`<MoveJ>`, `<Waypoint>`, `<Set>`, …) describe a
        teach-pendant-authored structure that has no faithful URScript
        equivalent; we emit them as `# [unrepresentable: <NodeName>]`
        comments so the user sees what's missing rather than getting a
        silently truncated extract.
    """
    xml_bytes = gzip.decompress(urp_bytes)
    root = ET.fromstring(xml_bytes)

    out: list[str] = [
        f"# Extracted from URP: name={root.get('name','?')}, "
        f"lastSavedIn={root.get('lastSavedIn','?')}, "
        f"robotType={root.get('robotType','?')}"
    ]

    def walk_program_tree(node: ET.Element) -> None:
        """Recurse into program-tree children. Caller passes the element
        whose direct children are program-tree nodes (MainProgram, or a
        nested <children> wrapper)."""
        for child in node:
            if child.tag == "children":
                walk_program_tree(child)
            elif child.tag == "Script":
                contents = child.findtext("cachedContents")
                if contents is None:
                    contents = (child.text or "").strip()
                if contents:
                    out.append(unescape(contents))
            else:
                out.append(f"# [unrepresentable: <{child.tag}>]")
                # Recurse so nested Scripts inside e.g. <If>/<Loop> still
                # surface, even if we lose the surrounding control flow.
                walk_program_tree(child)

    # Find the program tree: URProgram/children/MainProgram. Fall back to a
    # wider search for variants that nest things differently.
    main_program = root.find("./children/MainProgram")
    if main_program is not None:
        walk_program_tree(main_program)
    else:
        # No MainProgram wrapper — older or hand-crafted URP. Walk from
        # whatever <children> the root has.
        for children in root.findall("./children"):
            walk_program_tree(children)

    return "\n".join(out) + "\n"


# ----- CLI plumbing ----------------------------------------------------------


def _read(path: str) -> bytes:
    if path == "-":
        return sys.stdin.buffer.read()
    return Path(path).read_bytes()


def _write(path: str, data: bytes) -> None:
    if path == "-":
        sys.stdout.buffer.write(data)
        return
    Path(path).write_bytes(data)


def _default_out(inp: str, new_ext: str) -> str:
    if inp == "-":
        return "-"
    p = Path(inp)
    return str(p.with_suffix(new_ext))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_to_urp = sub.add_parser("to-urp", help="Wrap a .script as a .urp")
    p_to_urp.add_argument("input", help="path to .script (or '-' for stdin)")
    p_to_urp.add_argument(
        "output", nargs="?", help="path to .urp (default: input with .urp suffix)"
    )
    p_to_urp.add_argument("--name", help="program name embedded in URP (default: input stem)")
    p_to_urp.add_argument(
        "--installation",
        default="default",
        help="sibling .installation file name without extension (default: 'default')",
    )
    p_to_urp.add_argument(
        "--directory",
        default="/programs",
        help="program directory path embedded in URP (default: '/programs')",
    )
    p_to_urp.add_argument(
        "--source-file",
        help="path of source .script as recorded in URP (default: <directory>/<name>.script)",
    )
    p_to_urp.add_argument(
        "--loop",
        dest="run_only_once",
        action="store_false",
        default=True,
        help="set runOnlyOnce=false so PolyScope restarts the program on completion "
        "(matches the un-ticked 'Run Once' checkbox in the UI)",
    )
    p_to_urp.add_argument("--created-in", default=DEFAULT_CREATED_IN)
    p_to_urp.add_argument("--last-saved-in", default=DEFAULT_LAST_SAVED_IN)

    p_to_script = sub.add_parser("to-script", help="Extract URScript from a .urp")
    p_to_script.add_argument("input", help="path to .urp (or '-' for stdin)")
    p_to_script.add_argument(
        "output", nargs="?", help="path to .script (default: input with .script suffix)"
    )

    args = ap.parse_args(argv)

    if args.cmd == "to-urp":
        out_path = args.output or _default_out(args.input, ".urp")
        name = args.name or (Path(args.input).stem if args.input != "-" else "program")
        script_text = _read(args.input).decode("utf-8")
        urp = script_to_urp(
            script_text,
            name=name,
            installation=args.installation,
            directory=args.directory,
            source_file=args.source_file,
            run_only_once=args.run_only_once,
            created_in=args.created_in,
            last_saved_in=args.last_saved_in,
        )
        _write(out_path, urp)
        if out_path != "-":
            print(f"wrote {out_path} ({len(urp)} bytes gzipped)", file=sys.stderr)
        return 0

    if args.cmd == "to-script":
        out_path = args.output or _default_out(args.input, ".script")
        script_text = urp_to_script(_read(args.input))
        _write(out_path, script_text.encode("utf-8"))
        if out_path != "-":
            print(f"wrote {out_path}", file=sys.stderr)
        return 0

    return 2


if __name__ == "__main__":
    sys.exit(main())
