"""Unit tests for urctl/urp_builder.py.

These run without URSim — they exercise the node-tree emitter against
in-memory XML. The PolyScope acceptance test (proving the controller actually
deserializes the generated tree) lives in
test_integration_ursim.py::TestUrpBuilderLoading.
"""

from __future__ import annotations

import gzip
from xml.etree import ElementTree as ET

import pytest

from urctl.urp_builder import UrpProgram, Waypoint


def _parse(prog: UrpProgram) -> ET.Element:
    return ET.fromstring(gzip.decompress(prog.to_bytes()))


def _main_children(root: ET.Element) -> list[ET.Element]:
    """Direct program-tree children under MainProgram/children."""
    mp = root.find("./children/MainProgram/children")
    assert mp is not None, "missing MainProgram/children"
    return list(mp)


# --------------------------------------------------------------------------- #
# Envelope: the parts PolyScope's loader requires (mirrors test_urp_convert).
# --------------------------------------------------------------------------- #


class TestEnvelope:
    def test_output_is_gzipped(self):
        assert UrpProgram("t").to_bytes()[:2] == b"\x1f\x8b"

    def test_root_is_urprogram(self):
        assert _parse(UrpProgram("t")).tag == "URProgram"

    def test_required_top_level_attrs(self):
        root = _parse(UrpProgram("prog", installation="myinst"))
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

    def test_kinematics_envelope_is_zeroed_not_linearized(self):
        # The empirically-verified envelope: a non-zero/UR10e block makes
        # ScriptNodeConversionStrategy reject the program. See module docstring.
        kin = _parse(UrpProgram("t")).find("./kinematics")
        assert kin is not None
        assert kin.get("status") == "NOT_LINEARIZED"
        assert kin.find("jointChecksum").get("value") == "0, 0, 0, 0, 0, 0"

    def test_main_program_wrapper_present(self):
        root = _parse(UrpProgram("t"))
        assert root.find("./children/MainProgram") is not None

    def test_run_only_once_toggles(self):
        once = _parse(UrpProgram("t", run_only_once=True))
        loop = _parse(UrpProgram("t", run_only_once=False))
        assert once.find("./children/MainProgram").get("runOnlyOnce") == "true"
        assert loop.find("./children/MainProgram").get("runOnlyOnce") == "false"


# --------------------------------------------------------------------------- #
# Leaf nodes
# --------------------------------------------------------------------------- #


class TestLeafNodes:
    def test_comment(self):
        p = UrpProgram("t")
        p.comment("hello world")
        (node,) = _main_children(_parse(p))
        assert node.tag == "Comment"
        assert node.get("comment") == "hello world"

    def test_movej_waypoint(self):
        p = UrpProgram("t")
        p.movej(Waypoint("Home", q=[0, -1.57, 0, -1.57, 0, 0]))
        (move,) = _main_children(_parse(p))
        assert move.tag == "Move"
        assert move.get("motionType") == "MoveJ"
        wp = move.find("./children/Waypoint")
        assert wp.get("name") == "Home"
        angles = wp.find("./position/JointAngles").get("angles")
        assert angles == "0.0, -1.57, 0.0, -1.57, 0.0, 0.0"

    def test_movel_motion_type(self):
        p = UrpProgram("t")
        p.movel(Waypoint("L", q=[0, 0, 0, 0, 0, 0]))
        (move,) = _main_children(_parse(p))
        assert move.get("motionType") == "MoveL"

    def test_move_multiple_waypoints_under_one_node(self):
        p = UrpProgram("t")
        p.movej(
            Waypoint("A", q=[0, 0, 0, 0, 0, 0]),
            Waypoint("B", q=[1, 1, 1, 1, 1, 1]),
        )
        (move,) = _main_children(_parse(p))
        names = [w.get("name") for w in move.findall("./children/Waypoint")]
        assert names == ["A", "B"]

    def test_move_requires_waypoint(self):
        p = UrpProgram("t")
        with pytest.raises(ValueError):
            p.movej()
        with pytest.raises(ValueError):
            p.movel()

    def test_waypoint_without_blend_emits_empty_motion_parameters(self):
        p = UrpProgram("t")
        p.movel(Waypoint("Stop", q=[0, 0, 0, 0, 0, 0]))
        mp = _parse(p).find(".//Waypoint/motionParameters")
        assert mp is not None and mp.get("blendRadius") is None

    def test_waypoint_blend_radius_becomes_motion_parameter_attribute(self):
        # Schema (blendRadius as a <motionParameters> attribute) matches the
        # controller's own WaypointNodeConversionStrategy reader/writer.
        p = UrpProgram("t")
        p.movel(Waypoint("Fly", q=[0, 0, 0, 0, 0, 0], blend_radius=0.05))
        mp = _parse(p).find(".//Waypoint/motionParameters")
        assert mp.get("blendRadius") == "0.05"

    def test_negative_blend_radius_rejected(self):
        with pytest.raises(ValueError):
            Waypoint("bad", q=[0, 0, 0, 0, 0, 0], blend_radius=-0.01)

    def test_group_merges_consecutive_same_type_moves(self):
        p = UrpProgram("t")
        p.movel(Waypoint("A", q=[0, 0, 0, 0, 0, 0]), group=True)
        p.movel(Waypoint("B", q=[1, 1, 1, 1, 1, 1]), group=True)
        (move,) = _main_children(_parse(p))  # one Move node
        names = [w.get("name") for w in move.findall("./children/Waypoint")]
        assert names == ["A", "B"]

    def test_group_splits_on_motion_type_or_speed_change(self):
        p = UrpProgram("t")
        p.movel(Waypoint("A", q=[0, 0, 0, 0, 0, 0]), group=True)
        p.movej(Waypoint("B", q=[1, 1, 1, 1, 1, 1]), group=True)  # different type
        p.movel(Waypoint("C", q=[2, 2, 2, 2, 2, 2]), group=True, speed=0.9)  # different speed
        kinds = [(m.get("motionType")) for m in _main_children(_parse(p))]
        assert kinds == ["MoveL", "MoveJ", "MoveL"]

    def test_group_false_keeps_moves_separate(self):
        p = UrpProgram("t")
        p.movel(Waypoint("A", q=[0, 0, 0, 0, 0, 0]))
        p.movel(Waypoint("B", q=[1, 1, 1, 1, 1, 1]))
        assert len(_main_children(_parse(p))) == 2

    def test_set_output(self):
        p = UrpProgram("t")
        p.set_output(2, True)
        (node,) = _main_children(_parse(p))
        assert node.tag == "Set"
        assert node.get("type") == "DigitalOutput"
        assert node.get("outputBit") == "2"
        assert node.get("outputValue") == "true"

    def test_popup(self):
        p = UrpProgram("t")
        p.popup("done", warning=True)
        (node,) = _main_children(_parse(p))
        assert node.tag == "Popup"
        assert node.get("warning") == "true"
        assert node.find("message").text == "done"

    def test_script_line(self):
        p = UrpProgram("t")
        p.script_line("retract()")
        (node,) = _main_children(_parse(p))
        assert node.tag == "Script"
        assert node.get("type") == "Line"
        assert node.find("./expression/ExpressionToken").get("token") == "retract()"

    def test_script_file_carries_multiline_helper(self):
        p = UrpProgram("t", directory="/programs")
        body = 'def helper():\n  textmsg("hi")\nend\n'
        p.script_file("helpers", body)
        (node,) = _main_children(_parse(p))
        assert node.tag == "Script"
        assert node.get("type") == "File"
        assert node.find("cachedContents").text == body
        assert node.find("file").text == "/programs/helpers.script"

    def test_invalid_joint_vector_length(self):
        with pytest.raises(ValueError):
            UrpProgram("t").movej(Waypoint("bad", q=[0, 0, 0]))


# --------------------------------------------------------------------------- #
# Containers / nesting
# --------------------------------------------------------------------------- #


class TestNesting:
    def test_folder_nests_children(self):
        p = UrpProgram("t")
        with p.folder("Setup"):
            p.set_output(0, True)
            p.comment("inside")
        (folder,) = _main_children(_parse(p))
        assert folder.tag == "Folder"
        assert folder.get("name") == "Setup"
        kids = [c.tag for c in folder.find("children")]
        assert kids == ["Set", "Comment"]

    def test_if_digital_in_expression(self):
        p = UrpProgram("t")
        with p.if_digital_in(1, True):
            p.popup("triggered")
        (ifnode,) = _main_children(_parse(p))
        assert ifnode.tag == "If"
        tok = ifnode.find("./expression/ExpressionToken").get("token")
        assert tok == "get_standard_digital_in(1) == True"
        assert ifnode.find("./children/Popup") is not None

    def test_if_expr_raw(self):
        p = UrpProgram("t")
        with p.if_expr("var_x > 5"):
            p.comment("c")
        (ifnode,) = _main_children(_parse(p))
        assert ifnode.find("./expression/ExpressionToken").get("token") == "var_x > 5"

    def test_loop_count(self):
        p = UrpProgram("t")
        with p.loop(4):
            p.movej(Waypoint("Tap", q=[0, 0, 0, 0, 0, 0]))
        (loop,) = _main_children(_parse(p))
        assert loop.tag == "Loop"
        assert loop.get("loopNum") == "4"
        assert loop.find("./children/Move") is not None

    def test_deep_nesting(self):
        p = UrpProgram("t")
        with p.loop(2):
            with p.if_digital_in(0):
                p.movej(Waypoint("W", q=[0, 0, 0, 0, 0, 0]))
        root = _parse(p)
        wp = root.find("./children/MainProgram/children/Loop/children/If/children/Move/children/Waypoint")
        assert wp is not None and wp.get("name") == "W"

    def test_unbalanced_nesting_detected(self):
        p = UrpProgram("t")
        # Manually enter a container context without exiting, then emit.
        cm = p.folder("Leaky")
        cm.__enter__()
        with pytest.raises(RuntimeError):
            p.to_xml()


# --------------------------------------------------------------------------- #
# Escaping & well-formedness
# --------------------------------------------------------------------------- #


class TestEscaping:
    def test_special_chars_in_attrs_and_text(self):
        p = UrpProgram('prog & "x"')
        p.comment('a < b & c > "d"')
        p.popup('tom & "jerry"')
        # Must still parse — escaping is correct.
        root = _parse(p)
        assert root.get("name") == 'prog & "x"'
        children = _main_children(root)
        assert children[0].get("comment") == 'a < b & c > "d"'
        assert children[1].find("message").text == 'tom & "jerry"'

    def test_full_program_is_well_formed(self):
        p = UrpProgram("Pick")
        p.comment("generated")
        with p.folder("Setup"):
            p.set_output(0, True)
        p.script_file("helpers", 'def f():\n  textmsg("x")\nend\n')
        p.movej(Waypoint("Approach", q=[-1.6, -1.72, -2.2, -0.8, 1.595, -0.03]))
        with p.if_digital_in(0, True):
            p.movel(Waypoint("Grasp", q=[-1.6, -1.9, -2.16, -0.68, 1.595, -0.03]))
        with p.loop(3):
            p.movej(Waypoint("Tap", q=[-1.6, -1.72, -2.2, -0.8, 1.595, -0.03]))
        p.script_line("f()")
        p.popup("done")
        # Throws if not well-formed.
        ET.fromstring(p.to_xml())

    def test_save_writes_file(self, tmp_path):
        out = tmp_path / "demo.urp"
        UrpProgram("demo").save(out)
        assert out.exists()
        assert out.read_bytes()[:2] == b"\x1f\x8b"
