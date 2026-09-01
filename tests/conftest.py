"""Shared pytest fixtures.

URSim-talking helpers live in tests/_ursim.py so they can be imported from
both this file and the test modules — pytest's conftest.py isn't itself a
regular importable module.
"""

from __future__ import annotations

import gzip
from pathlib import Path

import pytest
from _ursim import (
    PROGRAMS_DIR,
    URSIM_DASH_PORT,
    URSIM_HOST,
    docker_available,
    tcp_open,
    wait_for_dash,
)

# ----- URSim availability gates ----------------------------------------------


@pytest.fixture(scope="session")
def ursim_ready() -> str:
    """Skip tests if a powered-on URSim controller isn't reachable.

    Tests that depend on the simulator should request this fixture; it
    yields the host so the test can pass it through to subprocesses.
    """
    if not tcp_open(URSIM_HOST, URSIM_DASH_PORT):
        pytest.skip(f"URSim Dashboard not reachable at {URSIM_HOST}:{URSIM_DASH_PORT}")
    # Best-effort: wait up to 30 s for URControl to come up, in case the
    # container was started right before the test run.
    try:
        wait_for_dash(lambda r: "NO_CONTROLLER" not in r and "Robotmode:" in r, timeout=30.0)
    except TimeoutError:
        pytest.skip("URSim is up but URControl never came online")
    return URSIM_HOST


@pytest.fixture(scope="session")
def docker_ready() -> str:
    from _ursim import URSIM_CONTAINER

    if not docker_available():
        pytest.skip(f"docker / {URSIM_CONTAINER} not available")
    return URSIM_CONTAINER


# ----- sample data fixtures --------------------------------------------------


@pytest.fixture
def tmp_script(tmp_path: Path) -> Path:
    """A trivial .script file usable as input to the converter."""
    p = tmp_path / "demo.script"
    p.write_text(
        "def demo():\n"
        '  popup("hello from a unit test", title="demo")\n'
        '  textmsg("demo: joints=", get_actual_joint_positions())\n'
        "end\n"
    )
    return p


@pytest.fixture
def inspection_bot_urp() -> Path:
    p = PROGRAMS_DIR / "InspectionBot" / "InspectionBot.urp"
    if not p.exists():
        pytest.skip("InspectionBot.urp not present — regenerate with scripts/urp_convert.py")
    return p


@pytest.fixture
def synthetic_urp_with_unknown_node(tmp_path: Path) -> Path:
    """A URP with a non-Script node inside MainProgram, to exercise the
    'unrepresentable node' branch in `urp_to_script`."""
    xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
        '<URProgram name="synthetic" installation="default" '
        'installationRelativePath="default" directory="/programs" '
        'createdIn="5.12.5" lastSavedIn="5.12.5" robotSerialNumber="">\n'
        '  <kinematics status="NOT_LINEARIZED" validChecksum="false">\n'
        '    <deltaTheta value="0,0,0,0,0,0"/>\n'
        '    <a value="0,0,0,0,0,0"/>\n'
        '    <d value="0,0,0,0,0,0"/>\n'
        '    <alpha value="0,0,0,0,0,0"/>\n'
        '    <jointChecksum value="0,0,0,0,0,0"/>\n'
        "  </kinematics>\n"
        "  <children>\n"
        '    <MainProgram runOnlyOnce="false" InitVariablesNode="false">\n'
        "      <children>\n"
        '        <MoveJ acceleration="1.0" velocity="0.5">\n'
        "          <children>\n"
        '            <Waypoint name="wp1"/>\n'
        '            <Script type="Code">\n'
        '              <cachedContents>textmsg("nested in MoveJ")</cachedContents>\n'
        "            </Script>\n"
        "          </children>\n"
        "        </MoveJ>\n"
        '        <Script type="Code">\n'
        '          <cachedContents>textmsg("top level")</cachedContents>\n'
        "        </Script>\n"
        "      </children>\n"
        "    </MainProgram>\n"
        "  </children>\n"
        "</URProgram>\n"
    )
    p = tmp_path / "synthetic.urp"
    p.write_bytes(gzip.compress(xml.encode("utf-8")))
    return p
