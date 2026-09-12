"""Cell profiles: parsing, precedence (env wins), the shipped files, and the
injection surface (only UR_/PERCEPTION_/REALSENSE_ keys, no expansion)."""

from __future__ import annotations

import pytest

from perception.cell import (
    ENV_CELL,
    apply_cell,
    cell_path,
    describe_cell,
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
        assert vals["PERCEPTION_BRACKET"] in ("eseries", "ur20")
    assert load_cell("sim")["PERCEPTION_FAKE"] == "1" and load_cell("sim")["UR_HOST"] == "localhost"
    assert load_cell("ur20")["PERCEPTION_BRACKET"] == "ur20" and load_cell("ur20")["UR_HOST"] == ""
    assert load_cell("ur3")["PERCEPTION_BRACKET"] == "eseries"


def test_parse_env_text_shapes():
    text = """
    # comment
    UR_HOST=10.0.0.5   # trailing comment
    export UR_PLATFORM="polyscopex"
    PERCEPTION_T_FLANGE_CAMERA='[0, 0, 0.1, 0, 0, 0]'
    UR_ROBOT_API_PORT=80
    """
    vals = parse_env_text(text)
    assert vals == {
        "UR_HOST": "10.0.0.5",
        "UR_PLATFORM": "polyscopex",
        "PERCEPTION_T_FLANGE_CAMERA": "[0, 0, 0.1, 0, 0, 0]",
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
    f.write_text("UR_HOST=\nUR_PLATFORM=polyscopex\nPERCEPTION_BRACKET=ur20\n")
    env = {"UR_PLATFORM": "e-series"}
    out = apply_cell(str(f), env)
    assert out["cell"] == "lab" and out["applied"] == {"PERCEPTION_BRACKET": "ur20"}
    assert out["kept"] == {"UR_PLATFORM": "e-series"} and out["missing"] == ["UR_HOST"]
    assert env["UR_PLATFORM"] == "e-series" and env["PERCEPTION_BRACKET"] == "ur20" and "UR_HOST" not in env
    assert env[ENV_CELL] == "lab"


def test_apply_cell_from_env_var_and_none():
    env = {ENV_CELL: "sim"}
    out = apply_cell(None, env)
    assert out["cell"] == "sim" and env["UR_ROBOT_API_PORT"] == "8000" and env["UR_PRIMARY_PORT"] == "31001"
    assert apply_cell(None, {})["cell"] is None


def test_unknown_cell_and_bad_path():
    with pytest.raises(ValueError, match="unknown cell"):
        cell_path("mars")
    with pytest.raises(ValueError, match="not found"):
        cell_path("/nonexistent/dir/x.env")
    with pytest.raises(ValueError):
        cell_path("")


def test_describe_cell_is_a_safe_slice():
    env = {"UR_HOST": "h", "UR_CELL": "ur3", "AWS_SECRET_ACCESS_KEY": "nope", "PERCEPTION_BRACKET": ""}
    d = describe_cell(env)
    assert d == {"UR_CELL": "ur3", "UR_HOST": "h"}


def test_cli_cells_export_is_shell_safe(capsys):
    from perception.cli import main

    assert main(["cells", "--export", "sim"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert "export UR_HOST=localhost" in out and "export UR_CELL=sim" in out
    assert all(line.startswith("export ") and "=" in line for line in out)
    assert main(["cells", "--export", "nope"]) == 2
