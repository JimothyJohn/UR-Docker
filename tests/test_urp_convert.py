"""Unit tests for scripts/urp_convert.py.

These tests don't need URSim — they exercise the converter purely against
in-memory data and tmp files. Integration tests that prove PolyScope
actually accepts the generated URPs live in test_integration_ursim.py.
"""

from __future__ import annotations

import gzip
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

# Side-effect import: adds scripts/ to sys.path.
import _ursim  # noqa: F401
import pytest
import urp_convert as uc

REPO_ROOT = Path(__file__).resolve().parent.parent
CONVERTER = REPO_ROOT / "scripts" / "urp_convert.py"


# ---------------------------------------------------------------------------
# script_to_urp
# ---------------------------------------------------------------------------


def _parse_urp(urp_bytes: bytes) -> ET.Element:
    return ET.fromstring(gzip.decompress(urp_bytes))


class TestScriptToUrp:
    def test_output_is_gzipped(self):
        out = uc.script_to_urp("textmsg('hi')\n", name="t")
        assert out[:2] == b"\x1f\x8b", "missing gzip magic bytes"

    def test_decompresses_to_valid_xml(self):
        root = _parse_urp(uc.script_to_urp("textmsg('hi')\n", name="t"))
        assert root.tag == "URProgram"

    def test_required_top_level_attrs(self):
        # PolyScope's loader rejects URPs missing these — see CLAUDE.md.
        root = _parse_urp(uc.script_to_urp("x=1\n", name="prog", installation="myinst"))
        for attr in (
            "name",
            "installation",
            "installationRelativePath",
            "directory",
            "createdIn",
            "lastSavedIn",
        ):
            assert root.get(attr) is not None, f"missing required attr: {attr}"
        assert root.get("name") == "prog"
        assert root.get("installation") == "myinst"
        assert root.get("installationRelativePath") == "myinst"

    def test_kinematics_block_present(self):
        # The single most expensive lesson from reverse-engineering the
        # format: PolyScope refuses the URP if <kinematics> is missing,
        # even when validChecksum is false.
        root = _parse_urp(uc.script_to_urp("x=1\n", name="t"))
        kin = root.find("./kinematics")
        assert kin is not None
        for child in ("deltaTheta", "a", "d", "alpha", "jointChecksum"):
            assert kin.find(child) is not None, f"missing kinematics/{child}"

    def test_main_program_wrapper_present(self):
        root = _parse_urp(uc.script_to_urp("x=1\n", name="t"))
        assert root.find("./children/MainProgram") is not None
        assert root.find("./children/MainProgram/children/Script") is not None

    def test_run_only_once_defaults_true(self):
        # One-shot is the safe default; runOnlyOnce=false makes PolyScope
        # restart the program forever, which surprised the E2E driver the
        # first time. Encode the default into the schema.
        root = _parse_urp(uc.script_to_urp("x=1\n", name="t"))
        mp = root.find("./children/MainProgram")
        assert mp.get("runOnlyOnce") == "true"

    def test_run_only_once_can_be_disabled_for_cycle_programs(self):
        # HelpfulBot-style tend programs loop forever; allow opting out.
        root = _parse_urp(uc.script_to_urp("x=1\n", name="t", run_only_once=False))
        mp = root.find("./children/MainProgram")
        assert mp.get("runOnlyOnce") == "false"

    def test_script_node_carries_source_file_pointer(self):
        # type="File" must include <file resolves-to="file"> per the
        # PolyScope schema. type="Code" wouldn't carry this.
        root = _parse_urp(uc.script_to_urp("x=1\n", name="myprog"))
        s = root.find("./children/MainProgram/children/Script")
        assert s.get("type") == "File"
        f = s.find("./file")
        assert f is not None
        assert f.get("resolves-to") == "file"
        # Default source-file path follows directory/name.script.
        assert f.text == "/programs/myprog.script"

    def test_explicit_source_file_overrides_default(self):
        root = _parse_urp(uc.script_to_urp("x=1\n", name="myprog", source_file="/custom/path.script"))
        f = root.find("./children/MainProgram/children/Script/file")
        assert f.text == "/custom/path.script"

    def test_script_body_preserved(self):
        body = 'def t():\n  popup("hi")\nend\n'
        root = _parse_urp(uc.script_to_urp(body, name="t"))
        contents = root.findtext("./children/MainProgram/children/Script/cachedContents")
        assert contents == body

    @pytest.mark.parametrize(
        "special",
        [
            'set_var("a", "b & c")',  # ampersand
            'popup("<warning>")',  # angle brackets
            "x = 'apostrophes'",  # apostrophes
            'popup("quotes \\"here\\"")',  # escaped quotes
        ],
    )
    def test_special_chars_in_body_are_escaped(self, special):
        # The script body lands inside an XML element; bad escaping would
        # produce malformed XML that doesn't even parse.
        urp = uc.script_to_urp(special + "\n", name="t")
        root = _parse_urp(urp)  # would raise ParseError if escaping broke
        contents = root.findtext("./children/MainProgram/children/Script/cachedContents")
        assert special in contents

    def test_unicode_in_script_body(self):
        body = 'popup("résumé — naïve façade 中文")\n'
        root = _parse_urp(uc.script_to_urp(body, name="t"))
        contents = root.findtext("./children/MainProgram/children/Script/cachedContents")
        assert contents == body

    def test_name_is_xml_escaped(self):
        # If a hostile name with quote chars sneaks in we don't want it to
        # break out of the attribute.
        urp = uc.script_to_urp("x=1\n", name='evil"name')
        root = _parse_urp(urp)  # would fail if escaping was missing
        assert root.get("name") == 'evil"name'


# ---------------------------------------------------------------------------
# urp_to_script
# ---------------------------------------------------------------------------


class TestUrpToScript:
    def test_extracts_script_body(self):
        body = 'def t():\n  textmsg("hi")\nend\n'
        urp = uc.script_to_urp(body, name="t")
        out = uc.urp_to_script(urp)
        assert body.strip() in out

    def test_header_comment_carries_metadata(self):
        urp = uc.script_to_urp("x=1\n", name="probe")
        out = uc.urp_to_script(urp)
        first = out.splitlines()[0]
        assert first.startswith("# Extracted from URP:")
        assert "name=probe" in first

    def test_does_not_emit_polyscope_envelope_as_unrepresentable(self):
        # Regression test: an earlier version flagged <kinematics>,
        # <deltaTheta>, etc. as "unrepresentable" nodes. The walker should
        # only descend into the program tree under MainProgram.
        urp = uc.script_to_urp("x=1\n", name="t")
        out = uc.urp_to_script(urp)
        assert "unrepresentable" not in out

    def test_flags_unknown_program_nodes(self, synthetic_urp_with_unknown_node):
        # MoveJ / Waypoint are real PolyScope program nodes with no faithful
        # URScript equivalent; the extractor must surface their existence.
        out = uc.urp_to_script(synthetic_urp_with_unknown_node.read_bytes())
        assert "[unrepresentable: <MoveJ>]" in out
        assert "[unrepresentable: <Waypoint>]" in out

    def test_recurses_into_unknown_nodes_to_find_nested_scripts(self):
        # When a Script is nested inside MoveJ/If/Loop, we still want its
        # body in the output, even though the surrounding control flow is
        # lost. The synthetic URP has both a nested and a top-level Script.
        # (re-build the synthetic URP inline rather than re-import the fixture)
        from xml.sax.saxutils import escape

        # Build escaped payloads outside the f-strings: a backslash inside an
        # f-string replacement field is a syntax error before Python 3.12.
        inside = escape('textmsg("inside")')
        outside = escape('textmsg("outside")')
        xml = (
            '<URProgram name="t" installation="default" '
            'installationRelativePath="default" directory="/programs" '
            'createdIn="x" lastSavedIn="x" robotSerialNumber="">'
            "<children><MainProgram>"
            "<children>"
            "<MoveJ><children>"
            f'<Script type="Code"><cachedContents>{inside}</cachedContents></Script>'
            "</children></MoveJ>"
            f'<Script type="Code"><cachedContents>{outside}</cachedContents></Script>'
            "</children></MainProgram></children></URProgram>"
        )
        out = uc.urp_to_script(gzip.compress(xml.encode()))
        assert 'textmsg("inside")' in out
        assert 'textmsg("outside")' in out

    def test_handles_script_with_text_content_instead_of_cachedContents(self):
        # Older / hand-edited URPs sometimes inline the script as element
        # text rather than wrapping it in <cachedContents>.
        from xml.sax.saxutils import escape

        body = 'textmsg("old format")'
        xml = (
            '<URProgram name="t" installation="default" '
            'installationRelativePath="default" directory="/programs" '
            'createdIn="x" lastSavedIn="x" robotSerialNumber="">'
            "<children><MainProgram><children>"
            f'<Script type="Code">{escape(body)}</Script>'
            "</children></MainProgram></children></URProgram>"
        )
        out = uc.urp_to_script(gzip.compress(xml.encode()))
        assert body in out

    def test_handles_urp_without_main_program_wrapper(self):
        # Defensive: walk from <children> directly if no MainProgram exists.
        from xml.sax.saxutils import escape

        body = 'textmsg("flat")'
        xml = (
            '<URProgram name="t" installation="default" '
            'installationRelativePath="default" directory="/programs" '
            'createdIn="x" lastSavedIn="x" robotSerialNumber="">'
            "<children>"
            f'<Script type="Code"><cachedContents>{escape(body)}</cachedContents></Script>'
            "</children></URProgram>"
        )
        out = uc.urp_to_script(gzip.compress(xml.encode()))
        assert body in out


# ---------------------------------------------------------------------------
# Roundtrips
# ---------------------------------------------------------------------------


class TestRoundtrip:
    def test_script_urp_script_preserves_body(self):
        body = 'def demo():\n  popup("hi")\n  sleep(0.1)\nend\n'
        urp = uc.script_to_urp(body, name="demo")
        out = uc.urp_to_script(urp)
        assert body.strip() in out

    def test_urp_script_urp_loads_consistently(self):
        # Convert -> extract -> reconvert. The second URP need not be
        # byte-identical (timestamps, gzip compression metadata), but its
        # parsed program body must match.
        body = 'textmsg("idempotency check")\n'
        urp1 = uc.script_to_urp(body, name="x")
        script = uc.urp_to_script(urp1)
        urp2 = uc.script_to_urp(script, name="x")
        # Both URPs must have a script node and the extracted body must
        # round-trip.
        for u in (urp1, urp2):
            root = _parse_urp(u)
            assert root.find("./children/MainProgram/children/Script") is not None
        assert body.strip() in uc.urp_to_script(urp2)

    def test_inspection_bot_urp_roundtrips_cleanly(self, inspection_bot_urp):
        # The bundled program is the converter's flagship integration target.
        out = uc.urp_to_script(inspection_bot_urp.read_bytes())
        # Sanity-check several markers from the source script.
        assert "InspectionBot" in out
        assert "TeachWaypoints" in out
        assert "RunProgram()" in out
        # And no unrepresentable noise (it's pure Script content).
        assert "unrepresentable" not in out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


class TestCli:
    def _run(self, *args: str, stdin: bytes | None = None) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(CONVERTER), *args],
            input=stdin,
            capture_output=True,
            check=False,
        )

    def test_to_urp_writes_default_output_name(self, tmp_script: Path):
        r = self._run("to-urp", str(tmp_script))
        assert r.returncode == 0, r.stderr.decode()
        out = tmp_script.with_suffix(".urp")
        assert out.exists()
        # quick parse check
        ET.fromstring(gzip.decompress(out.read_bytes()))

    def test_to_urp_explicit_output(self, tmp_script: Path, tmp_path: Path):
        out = tmp_path / "custom.urp"
        r = self._run("to-urp", str(tmp_script), str(out))
        assert r.returncode == 0, r.stderr.decode()
        assert out.exists()

    def test_to_script_stdin_stdout(self, tmp_script: Path):
        urp = uc.script_to_urp(tmp_script.read_text(), name="demo")
        r = self._run("to-script", "-", "-", stdin=urp)
        assert r.returncode == 0, r.stderr.decode()
        assert b"popup" in r.stdout

    def test_to_urp_custom_installation_flag(self, tmp_script: Path, tmp_path: Path):
        out = tmp_path / "custom.urp"
        r = self._run("to-urp", str(tmp_script), str(out), "--installation", "MyRig")
        assert r.returncode == 0, r.stderr.decode()
        root = ET.fromstring(gzip.decompress(out.read_bytes()))
        assert root.get("installation") == "MyRig"
        assert root.get("installationRelativePath") == "MyRig"

    def test_missing_subcommand_errors(self):
        r = self._run()
        assert r.returncode != 0
        assert b"required" in r.stderr.lower() or b"usage" in r.stderr.lower()

    def test_to_urp_loop_flag_disables_run_only_once(self, tmp_script: Path, tmp_path: Path):
        out = tmp_path / "looping.urp"
        r = self._run("to-urp", str(tmp_script), str(out), "--loop")
        assert r.returncode == 0, r.stderr.decode()
        root = ET.fromstring(gzip.decompress(out.read_bytes()))
        assert root.find("./children/MainProgram").get("runOnlyOnce") == "false"
