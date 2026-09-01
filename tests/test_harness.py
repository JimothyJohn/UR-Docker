"""Tests for the deep-introspection harness layer: code decoding
(:mod:`urctl.codes`), installation parsing (:mod:`urctl.installation`),
system inspection parsers (:mod:`urctl.sysinfo`), the tolerant RTDE recipe,
and the deep ``rtde_state`` shaping."""

from __future__ import annotations

import gzip
import struct
from pathlib import Path

import pytest

from urctl import Robot, RobotConfig, codes
from urctl import rtde as urctl_rtde
from urctl.installation import parse_installation, parse_installation_file
from urctl.rtde import RtdeClient
from urctl.sysinfo import (
    DockerRunner,
    SshRunner,
    SystemInspector,
    parse_df,
    parse_joint_info,
    parse_program_listing,
    runner_for,
)

REPO = Path(__file__).resolve().parent.parent


# ----- codes -----------------------------------------------------------------


class TestCodes:
    def test_known_modes(self):
        assert codes.name_for(codes.ROBOT_MODES, 7) == "RUNNING"
        assert codes.name_for(codes.SAFETY_MODES, 3) == "PROTECTIVE_STOP"
        assert codes.name_for(codes.RUNTIME_STATES, 2) == "PLAYING"
        assert codes.name_for(codes.JOINT_MODES, 253) == "RUNNING"

    def test_unknown_code_is_visible_not_lost(self):
        assert codes.name_for(codes.ROBOT_MODES, 99) == "UNKNOWN(99)"

    def test_none_is_none(self):
        assert codes.name_for(codes.ROBOT_MODES, None) is None
        assert codes.decode_bits(codes.SAFETY_STATUS_BITS, None) is None

    def test_decode_bits_real_healthy_word(self):
        # 2049 is what a healthy e-Series reports: NORMAL + 3-pos-enabling bit.
        flags = codes.decode_bits(codes.SAFETY_STATUS_BITS, 2049)
        assert flags["normal_mode"] is True
        assert flags["three_position_enabling"] is True
        assert flags["protective_stopped"] is False
        assert flags["fault"] is False

    def test_decode_bits_reports_unnamed_high_bits(self):
        word = 1 | (1 << 20)
        flags = codes.decode_bits(codes.SAFETY_STATUS_BITS, word)
        assert flags["normal_mode"] is True
        assert flags["bit20"] is True


# ----- installation parsing --------------------------------------------------


def _make_installation(tcp_offset: str, *, active: str = "Gripper") -> bytes:
    xml = f"""<Installation directory="/programs" fileName="default">
      <Version projectName="URSoftware" major="5" minor="11" bugfix="10"/>
      <TCPSettings activePose="{active}">
        <availablePoses>
          <tcp id="a" name="Gripper" offset="{tcp_offset}"/>
          <tcp id="b" name="Flange" offset="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>
        </availablePoses>
      </TCPSettings>
      <SafetySettings>
[SafetyLimits Normal Values]
maxTcpSpeed = 1.5
maxForce = 150.0
[SafetyLimits Reduced Values]
maxTcpSpeed = 0.75
      </SafetySettings>
      <SafeHomeSettings enabled="true" position="0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0"/>
      <PayloadSettings>
        <Payload name="Gripper" mass="1.2" defaultPayload="false"
                 centerOfGravity="0.0, 0.0, 0.05"/>
      </PayloadSettings>
      <IOs>
        <DigitalOutputNames value="Gripper, , conveyor, , , , , "/>
        <DigitalInputNames value=", , , , , , , "/>
      </IOs>
    </Installation>"""
    return gzip.compress(xml.encode())


class TestInstallationParser:
    def test_repo_sample_is_flange_tcp(self):
        info = parse_installation_file(REPO / "programs" / "InspectionBot" / "InspectionBot.installation")
        assert info["active_tcp"] == "TCP"
        assert info["flange_tcp"] is True
        assert info["payload"]["mass"] == 0.0
        assert info["saved_by"]["version"] == "5.12.5"
        assert info["safety_limits"]["normal"]["max_tcp_speed"] == 1.5

    def test_non_flange_tcp_detected(self):
        # The real-robot trap: a gripper TCP offset makes native MoveJ nodes
        # unsafe to author. flange_tcp must come back False.
        info = parse_installation(_make_installation("0.0, 0.0, 0.15, 0.0, 0.0, 0.0"))
        assert info["active_tcp"] == "Gripper"
        assert info["flange_tcp"] is False
        offsets = {t["name"]: t["offset"] for t in info["tcps"]}
        assert offsets["Gripper"][2] == 0.15
        active = [t["name"] for t in info["tcps"] if t["active"]]
        assert active == ["Gripper"]

    def test_zero_offset_active_tcp_is_flange(self):
        info = parse_installation(_make_installation("0.0, 0.0, 0.0, 0.0, 0.0, 0.0"))
        assert info["flange_tcp"] is True

    def test_named_io_pins_extracted(self):
        info = parse_installation(_make_installation("0,0,0,0,0,0"))
        assert info["io_names"]["digital_out"] == [
            {"pin": 0, "name": "Gripper"},
            {"pin": 2, "name": "conveyor"},
        ]
        # All-blank name lists are omitted entirely.
        assert "digital_in" not in info["io_names"]

    def test_uncompressed_xml_accepted(self):
        raw = gzip.decompress(_make_installation("0,0,0,0,0,0"))
        info = parse_installation(raw)
        assert info["tcps"]

    def test_payload_and_safe_home(self):
        info = parse_installation(_make_installation("0,0,0.1,0,0,0"))
        assert info["payload"]["mass"] == 1.2
        assert info["payload"]["center_of_gravity"] == [0.0, 0.0, 0.05]
        assert info["safe_home"]["enabled"] is True
        assert len(info["safe_home"]["joints"]) == 6


# ----- sysinfo parsers -------------------------------------------------------


JOINT_INFO_FIXTURE = """\
[joint 0]
serial = 221110150442
selftest_version_uA = 33.0.7

[joint 1]
serial = 221110148242
selftest_version_uA = 33.0.7

[joint 2]
serial = 221120049734
selftest_version_uA = 33.0.7

[joint 3]
serial = 221110073522
selftest_version_uA = 33.0.7

[joint 4]
serial = 221100242623
selftest_version_uA = 33.0.7

[joint 5]
serial = 222070489722
selftest_version_uA = 37.1.17
"""


class TestSysinfoParsers:
    def test_joint_info_flags_replacement(self):
        joints = parse_joint_info(JOINT_INFO_FIXTURE)
        assert len(joints) == 6
        assert [j["joint"] for j in joints] == list(range(6))
        assert joints[5]["serial"] == "222070489722"
        assert joints[5]["replacement_suspected"] is True
        assert all(not j["replacement_suspected"] for j in joints[:5])

    def test_joint_info_uniform_arm_flags_nothing(self):
        uniform = JOINT_INFO_FIXTURE.replace("37.1.17", "33.0.7")
        joints = parse_joint_info(uniform)
        assert all(not j["replacement_suspected"] for j in joints)

    def test_parse_df_matches_df_use_percent(self):
        # Real UR10e figures: df itself reports 87% (used/(used+avail)), while
        # used/total would say 82% — the reserved-blocks difference.
        text = (
            "Filesystem     1K-blocks    Used Available Use% Mounted on\n"
            "/dev/mmcblk1p3   1744383 1428480    227832  87% /\n"
        )
        disk = parse_df(text)
        assert disk["filesystem"] == "/dev/mmcblk1p3"
        assert disk["used_percent"] == 86  # round(100*1428480/(1428480+227832))
        assert disk["available_mb"] == 222

    def test_parse_program_listing_sorts_and_kinds(self):
        text = (
            "1788000000 2926 /programs/zeta.urp\n"
            "1788000001 1234 /programs/Alpha.script\n"
            "1788000002 999 /programs/default.installation\n"
            "garbage line\n"
        )
        entries = parse_program_listing(text)
        assert [e["name"] for e in entries] == ["Alpha.script", "default.installation", "zeta.urp"]
        assert entries[2]["kind"] == "urp"
        assert entries[2]["size"] == 2926

    def test_runner_inference(self):
        assert isinstance(runner_for(RobotConfig(host="localhost")), DockerRunner)
        remote = runner_for(RobotConfig(host="192.168.1.50"))
        assert isinstance(remote, SshRunner)
        assert remote.host == "192.168.1.50"
        assert remote.user == "root"

    def test_runner_override_and_user_at_host(self):
        r = runner_for(RobotConfig(host="localhost"), access="ssh", target="ur@10.1.2.3")
        assert isinstance(r, SshRunner)
        assert (r.user, r.host) == ("ur", "10.1.2.3")
        d = runner_for(RobotConfig(host="10.0.0.5"), access="docker", target="my-ursim")
        assert isinstance(d, DockerRunner)
        assert d.container == "my-ursim"

    def test_runner_for_rejects_unknown_access(self):
        with pytest.raises(ValueError, match="unknown access"):
            runner_for(RobotConfig(), access="carrier-pigeon")


class FixtureRunner:
    """A runner whose 'controller' is a canned command->output mapping."""

    label = "fixture"
    paths = dict(
        program_dir="/programs",
        urcontrol_dir="/root/.urcontrol",
        urcontrol_log="/tmp/log/urcontrol/current",
        polyscope_log="/root/polyscope.log",
        log_history="/root/log_history.txt",
        urcaps_dir="/root/.urcaps",
        flightreports_dir="/root/flightreports",
    )

    def __init__(self, responses: dict[str, str]):
        self.responses = responses
        self.commands: list[str] = []

    def run(self, command: str) -> str:
        self.commands.append(command)
        for key, out in self.responses.items():
            if key in command:
                return out
        from urctl.sysinfo import RunnerError

        raise RunnerError(f"fixture has no response for {command!r}")


class TestSystemInspector:
    def test_joints_section_detects_mismatch(self):
        insp = SystemInspector(
            FixtureRunner(
                {
                    "joint_info.txt": JOINT_INFO_FIXTURE,
                    "kinematic calibration checksum": "1\n",
                    "checksum of the joint": "5\n",
                    "calibration.conf": "2022-04-08\n",
                }
            )
        )
        result = insp.joints()
        assert result["calibration_mismatch"] is True
        assert result["mismatched_joint_ids"] == [5]
        assert result["calibration_date"] == "2022-04-08"
        assert result["joints"][5]["replacement_suspected"] is True

    def test_sections_fail_independently(self):
        insp = SystemInspector(FixtureRunner({}))
        snap = insp.snapshot()
        # Every section is present and carries an error note instead of raising.
        for section in ("identity", "joints", "storage", "programs", "installation", "urcaps"):
            assert section in snap
        assert "error" in snap["identity"]
        assert "error" in snap["installation"]


# ----- tolerant RTDE recipe --------------------------------------------------


def _control_reply(ptype: int, body: bytes) -> bytes:
    return struct.pack(">HB", 3 + len(body), ptype) + body


class TestTolerantRecipe:
    def _client(self, reply_bodies: list[bytes], *, strict: bool) -> tuple[RtdeClient, object]:
        from tests.test_urctl import FakeSocket

        client = RtdeClient(RobotConfig(), outputs=["actual_q", "made_up_field", "timestamp"], strict=strict)
        data = b"".join(
            _control_reply(urctl_rtde._RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, b) for b in reply_bodies
        )
        client._sock = FakeSocket(data)
        return client, client._sock

    def test_strict_raises_on_not_found(self):
        client, _ = self._client([b"\x01" + b"VECTOR6D,NOT_FOUND,DOUBLE"], strict=True)
        with pytest.raises(urctl_rtde.RtdeError, match="not available"):
            client._setup_outputs()

    def test_tolerant_drops_and_reasks(self):
        client, sock = self._client(
            [
                b"\x01" + b"VECTOR6D,NOT_FOUND,DOUBLE",  # first ask: one miss
                b"\x02" + b"VECTOR6D,DOUBLE",  # re-ask without it
            ],
            strict=False,
        )
        client._setup_outputs()
        assert client.dropped_outputs == ["made_up_field"]
        assert client.outputs == ["actual_q", "timestamp"]
        assert [name for name, _ in client._out_recipe] == ["actual_q", "timestamp"]
        assert client._out_recipe_id == 2

    def test_tolerant_all_missing_still_raises(self):
        client, _ = self._client([b"\x01" + b"NOT_FOUND,NOT_FOUND,NOT_FOUND"], strict=False)
        with pytest.raises(urctl_rtde.RtdeError, match="no requested"):
            client._setup_outputs()


# ----- deep rtde_state shaping -----------------------------------------------


class DeepFakeRtde:
    dropped_outputs = ["actual_joint_voltages"]

    def read_outputs(self) -> dict:
        return {
            "actual_q": [0.0] * 6,
            "actual_qd": [0.0] * 6,
            "actual_TCP_pose": [0.4, -0.2, 0.5, 0, 3.14, 0],
            "actual_TCP_speed": [0.0] * 6,
            "actual_TCP_force": [0.0] * 6,
            "safety_status_bits": 2049,
            "runtime_state": 1,
            "robot_mode": 7,
            "timestamp": 99.0,
            "safety_mode": 1,
            "robot_status_bits": 1,
            "actual_current": [0.1] * 6,
            "joint_temperatures": [31.5] * 6,
            "joint_mode": [253] * 6,
            "target_q": [0.0] * 6,
            "speed_scaling": 1.0,
            "target_speed_fraction": 1.0,
            "actual_robot_voltage": 48.1,
            "actual_main_voltage": 48.0,
            "tool_output_voltage": 24,
            "tool_mode": 253,
            "actual_digital_input_bits": 0,
            "actual_digital_output_bits": 5,
        }

    def close(self) -> None:
        pass


class TestDeepRtdeState:
    def test_deep_mapping_and_decoding(self):
        robot = Robot(RobotConfig())
        robot._rtde = DeepFakeRtde()
        robot._rtde_deep = True  # pretend the cached client negotiated DEEP_OUTPUTS
        result = robot.rtde_state(deep=True)
        assert result["ok"]
        assert result["safety_mode_name"] == "NORMAL"
        assert result["robot_mode_name"] == "RUNNING"
        assert result["runtime_state_name"] == "STOPPED"
        assert result["joint_mode_names"] == ["RUNNING"] * 6
        assert result["safety_flags"]["normal_mode"] is True
        assert result["robot_status_flags"]["power_on"] is True
        assert result["tool_output_voltage"] == 24
        assert result["digital_outputs"] == 5
        assert result["unavailable_fields"] == ["actual_joint_voltages"]

    def test_deep_upgrade_replaces_default_client(self, monkeypatch):
        # A cached default-recipe client must be discarded when deep is requested.
        robot = Robot(RobotConfig())
        robot._rtde = DeepFakeRtde()
        robot._rtde_deep = False
        built = {}

        class FakeDeepClient(DeepFakeRtde):
            def __init__(self, config, outputs=None, strict=True):
                built["outputs"] = outputs
                built["strict"] = strict

        monkeypatch.setattr(urctl_rtde, "RtdeClient", FakeDeepClient)
        client = robot._rtde_client(deep=True)
        assert isinstance(client, FakeDeepClient)
        assert built["strict"] is False
        assert set(urctl_rtde.DEFAULT_OUTPUTS) <= set(built["outputs"])
        assert robot._rtde_deep is True
