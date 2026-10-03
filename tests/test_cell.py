"""Cell profiles: parsing, precedence (env wins), the shipped files, and the
injection surface (only UR_/PERCEPTRONICS_/REALSENSE_ keys, no expansion)."""

from __future__ import annotations

import pytest

from perceptronics.cell import (
    ENV_CELL,
    apply_cell,
    cell_path,
    describe_cell,
    legacy_variables,
    list_cells,
    load_cell,
    parse_env_text,
)


def test_shipped_cells_parse_and_cover_the_three_targets():
    names = list_cells()
    assert {"sim", "ur3", "ur20"} <= set(names)
    for n in names:
        vals = load_cell(n)
        assert vals["UR_PLATFORM"] in ("polyscopex", "e-series")
        assert vals["PERCEPTRONICS_BRACKET"] in ("eseries", "ur20")
    assert load_cell("sim")["PERCEPTRONICS_FAKE"] == "1" and load_cell("sim")["UR_HOST"] == "localhost"
    assert load_cell("ur20")["PERCEPTRONICS_BRACKET"] == "ur20" and load_cell("ur20")["UR_HOST"] == ""
    assert load_cell("ur3")["PERCEPTRONICS_BRACKET"] == "eseries"


def test_parse_env_text_shapes():
    text = """
    # comment
    UR_HOST=10.0.0.5   # trailing comment
    export UR_PLATFORM="polyscopex"
    PERCEPTRONICS_T_FLANGE_CAMERA='[0, 0, 0.1, 0, 0, 0]'
    UR_ROBOT_API_PORT=80
    """
    vals = parse_env_text(text)
    assert vals == {
        "UR_HOST": "10.0.0.5",
        "UR_PLATFORM": "polyscopex",
        "PERCEPTRONICS_T_FLANGE_CAMERA": "[0, 0, 0.1, 0, 0, 0]",
        "UR_ROBOT_API_PORT": "80",
    }


@pytest.mark.parametrize(
    "bad",
    [
        "PATH=/tmp",  # outside the allowed namespace
        "LD_PRELOAD=x.so",
        "ur_host=1",  # lower-case
        "UR_HOST",  # no '='
        "UR_HOST=$(rm -rf /)\nOTHER=1",  # second line refused (first is kept literal, no expansion)
    ],
)
def test_parse_env_text_refuses_foreign_keys(bad):
    with pytest.raises(ValueError):
        parse_env_text(bad)


def test_no_shell_expansion():
    assert parse_env_text("UR_HOST=$(hostname)")["UR_HOST"] == "$(hostname)"
    assert parse_env_text("UR_HOST=${X}")["UR_HOST"] == "${X}"


def test_apply_cell_env_wins_and_empty_values_are_reported_missing(tmp_path):
    f = tmp_path / "lab.env"
    f.write_text("UR_HOST=\nUR_PLATFORM=polyscopex\nPERCEPTRONICS_BRACKET=ur20\n")
    env = {"UR_PLATFORM": "e-series"}
    out = apply_cell(str(f), env)
    assert out["cell"] == "lab" and out["applied"] == {"PERCEPTRONICS_BRACKET": "ur20"}
    assert out["kept"] == {"UR_PLATFORM": "e-series"} and out["missing"] == ["UR_HOST"]
    assert (
        env["UR_PLATFORM"] == "e-series" and env["PERCEPTRONICS_BRACKET"] == "ur20" and "UR_HOST" not in env
    )
    assert env[ENV_CELL] == "lab"


def test_apply_cell_from_env_var_and_none():
    env = {ENV_CELL: "sim"}
    out = apply_cell(None, env)
    assert out["cell"] == "sim" and env["UR_ROBOT_API_PORT"] == "8000" and env["UR_PRIMARY_PORT"] == "31001"
    assert apply_cell(None, {})["cell"] is None


def test_legacy_perception_variables_are_named_not_silently_ignored(capsys):
    # The PERCEPTION_* names were renamed PERCEPTRONICS_* (2026-09-28). A stale
    # export must be called out, and one in a cell file refused with its new name.
    env = {
        "PERCEPTION_T_FLANGE_CAMERA": "0,0,0,0,0,0",
        "PERCEPTION_TIP_M": "0.2",
        "PERCEPTRONICS_TIP_M": "0.163",
    }
    assert legacy_variables(env) == ["PERCEPTION_TIP_M", "PERCEPTION_T_FLANGE_CAMERA"]
    apply_cell(None, env)
    err = capsys.readouterr().err
    assert "PERCEPTION_TIP_M" in err and "PERCEPTION_T_FLANGE_CAMERA" in err and "PERCEPTRONICS_" in err
    assert env["PERCEPTRONICS_TIP_M"] == "0.163"  # nothing is copied across
    with pytest.raises(ValueError, match="PERCEPTRONICS_BRACKET"):
        parse_env_text("PERCEPTION_BRACKET=ur20")
    apply_cell(None, {"PERCEPTRONICS_TIP_M": "0.163"})
    assert capsys.readouterr().err == ""


def test_unknown_cell_and_bad_path():
    with pytest.raises(ValueError, match="unknown cell"):
        cell_path("mars")
    with pytest.raises(ValueError, match="not found"):
        cell_path("/nonexistent/dir/x.env")
    with pytest.raises(ValueError):
        cell_path("")


def test_describe_cell_is_a_safe_slice():
    env = {"UR_HOST": "h", "UR_CELL": "ur3", "AWS_SECRET_ACCESS_KEY": "nope", "PERCEPTRONICS_BRACKET": ""}
    d = describe_cell(env)
    assert d == {"UR_CELL": "ur3", "UR_HOST": "h"}


def test_cli_cells_export_is_shell_safe(capsys):
    from perceptronics.cli import main

    assert main(["cells", "--export", "sim"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert "export UR_HOST=localhost" in out and "export UR_CELL=sim" in out
    assert all(line.startswith("export ") and "=" in line for line in out)
    assert main(["cells", "--export", "nope"]) == 2


# -- without_key: the pick PC moves the hand-eye out of its cell.env ------------------------------


def test_without_key_removes_every_line_of_it_and_returns_the_last_value():
    from perceptronics.cell import without_key

    text = (
        "# a comment naming PERCEPTRONICS_T_FLANGE_CAMERA=[stays]\n"
        "UR_HOST=192.168.3.3\n"
        "PERCEPTRONICS_T_FLANGE_CAMERA=[1,2,3,4,5,6]\n"
        "PERCEPTRONICS_TIP_M=0.163\n"
        'export PERCEPTRONICS_T_FLANGE_CAMERA="[0.1,0,0,0,0,0]"\n'
        "PERCEPTRONICS_T_FLANGE_CAMERA_X=keep\n"
    )
    rest, value = without_key(text, "PERCEPTRONICS_T_FLANGE_CAMERA")
    assert value == "[0.1,0,0,0,0,0]"  # the last one, unquoted the way the parser reads it
    assert rest == (
        "# a comment naming PERCEPTRONICS_T_FLANGE_CAMERA=[stays]\n"
        "UR_HOST=192.168.3.3\n"
        "PERCEPTRONICS_TIP_M=0.163\n"
        "PERCEPTRONICS_T_FLANGE_CAMERA_X=keep\n"
    )


def test_without_key_leaves_a_file_without_it_unchanged():
    from perceptronics.cell import without_key

    text = "UR_HOST=192.168.3.3\n# PERCEPTRONICS_T_FLANGE_CAMERA=x\n"
    assert without_key(text, "PERCEPTRONICS_T_FLANGE_CAMERA") == (text, None)
