"""The UFACTORY wire protocol (urctl/xarm.py), held to the vendor's own bytes.

The golden requests below were captured by running UFACTORY's
xArm-Python-SDK 1.18.5 command layer (``UxbusCmdTcp``) against a port that
records what it writes — the SDK is not a dependency, its output is. If an
encoder drifts from what the controller's own SDK sends, these fail."""

from __future__ import annotations

import math
import struct

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from urctl import xarm
from urctl.xarm import (
    FEEDBACK_REGISTER,
    REPORT_NORMAL_MIN,
    XArmError,
    decode_reply,
    decode_report,
    encode_request,
    fp32_le,
    parse_version,
    split_frames,
)

# name -> (transaction id, register, payload builder, the SDK's bytes)
GOLDEN = {
    "get_version": (1, xarm.REG_GET_VERSION, b"", "00010002000101"),
    "motion_en": (2, xarm.REG_MOTION_EN, bytes([8, 1]), "0002000200030b0801"),
    "set_mode": (3, xarm.REG_SET_MODE, bytes([0, 0]), "000300020003130000"),
    "set_state": (4, xarm.REG_SET_STATE, bytes([4]), "0004000200020c04"),
    "move_joint": (
        5,
        xarm.REG_MOVE_JOINT,
        fp32_le([0.1, -0.2, 0.3, 0, 0.5, 0, 0, 0.5, 0.8, 0]),
        "00050002002917cdcccc3dcdcc4cbe9a99993e000000000000003f00000000000000000000003fcdcc4c3f00000000",
    ),
    "move_jointb": (
        6,
        xarm.REG_MOVE_JOINTB,
        fp32_le([0.1, -0.2, 0.3, 0, 0.5, 0, 0, 0.5, 0.8, 20.0]),
        "00060002002918cdcccc3dcdcc4cbe9a99993e000000000000003f00000000000000000000003fcdcc4c3f0000a041",
    ),
    "move_line_common": (
        7,
        xarm.REG_MOVE_LINE,
        fp32_le([300, 0, 200, 3.14159, 0, 0, 100, 1000, 0, 5.0]) + bytes([0, 1, 0]),
        "00070002002c15000096430000000000004843d00f494000000000000000000000c84200007a44000000000000a040000100",
    ),
    "move_line_common_check": (
        8,
        xarm.REG_MOVE_LINE,
        fp32_le([300, 0, 200, 3.14159, 0, 0, 100, 1000, 0, -1]) + bytes([0, 1, 2]),
        "00080002002c15000096430000000000004843d00f494000000000000000000000c84200007a4400000000000080bf000102",
    ),
    "move_line_aa": (
        9,
        xarm.REG_MOVE_LINE_AA,
        fp32_le([300, 0, 200, 3.14159, 0, 0, 100, 1000, 0]) + bytes([0, 0]),
        "0009000200275c000096430000000000004843d00f494000000000000000000000c84200007a44000000000000",
    ),
    "set_tcp_offset": (
        10,
        xarm.REG_SET_TCP_OFFSET,
        fp32_le([0, 0, 163, 0, 0, 1.5708]),
        "000a00020019230000000000000000000023430000000000000000f90fc93f",
    ),
    "move_gohome": (13, xarm.REG_MOVE_HOME, fp32_le([0.5, 0.8, 0]), "000d0002000d190000003fcdcc4c3f00000000"),
    "sleep": (14, xarm.REG_SLEEP_INSTT, fp32_le([1.5]), "000e000200051a0000c03f"),
    "get_state": (15, xarm.REG_GET_STATE, b"", "000f000200010d"),
    "get_cmdnum": (16, xarm.REG_GET_CMDNUM, b"", "0010000200010e"),
    "get_err": (17, xarm.REG_GET_ERROR, b"", "0011000200010f"),
    "get_pose_aa": (18, xarm.REG_GET_TCP_POSE_AA, b"", "0012000200015b"),
    "get_joint": (19, xarm.REG_GET_JOINT_POS, b"", "0013000200012a"),
    "clean_err": (20, xarm.REG_CLEAN_ERR, b"", "00140002000110"),
}


@pytest.mark.parametrize("name", sorted(GOLDEN))
def test_requests_match_the_vendor_sdk_byte_for_byte(name):
    trans_id, register, payload, expected = GOLDEN[name]
    assert encode_request(trans_id, register, payload).hex() == expected


class _Capture:
    """A client whose request() records the frame instead of sending it."""

    def __init__(self):
        self.client = xarm.XArmClient("capture.invalid")
        self.frames: list[bytes] = []

        def request(register, payload=b"", *, timeout=None):
            tid = len(self.frames) + 1
            self.frames.append(encode_request(tid, register, payload))
            return xarm.Reply(tid, register, 0, bytes(64))

        self.client.request = request  # type: ignore[method-assign]


def test_the_typed_commands_build_the_sdks_payloads():
    """The client's typed methods, not just the encoder, produce the SDK's bytes."""
    cap = _Capture()
    c = cap.client
    v = parse_version("v2.5.0")
    c.get_version()
    c.motion_enable(True)
    c.set_mode(0, v)
    c.set_state(4)
    c.move_joints([0.1, -0.2, 0.3, 0, 0.5, 0], 0.5, 0.8)
    c.move_joints([0.1, -0.2, 0.3, 0, 0.5, 0], 0.5, 0.8, radius=20.0)
    c.move_line_aa([300, 0, 200, 3.14159, 0, 0], 100, 1000, radius=5.0)
    c.move_line_aa([300, 0, 200, 3.14159, 0, 0], 100, 1000, only_check=2)
    c.move_line_aa([300, 0, 200, 3.14159, 0, 0], 100, 1000, common=False)
    c.set_tcp_offset([0, 0, 163, 0, 0, 1.5708])
    c.set_controller_digital(3, True)
    c.set_controller_digital(10, False)
    c.move_home(0.5, 0.8)
    c.sleep(1.5)
    expected = [GOLDEN[n][3] for n in GOLDEN]
    # the golden table skips ids 11/12 (the two GPIO frames below)
    got = [f.hex() for f in cap.frames]
    assert got[:10] == expected[:10]
    assert got[10] == "000b00020003860808"  # cgpio_set_auxdigit(3, 1)
    assert got[11] == "000c000200058600000400"  # cgpio_set_auxdigit(10, 0)
    assert got[12:] == expected[10:12]


def test_old_firmware_set_mode_sends_no_detection_byte():
    cap = _Capture()
    cap.client.set_mode(2, parse_version("v1.9.0"))
    assert cap.frames[0].hex() == "000100020002" + "13" + "02"


def _reply(trans_id: int, register: int, status: int, data: bytes) -> bytes:
    return struct.pack(">HHHB", trans_id, 2, len(data) + 2, register) + bytes([status]) + data


def test_reply_status_bits_decode():
    r = decode_reply(_reply(7, 13, 0x40 | 0x10, b"\x02"))
    assert (r.trans_id, r.register, r.data) == (7, 13, b"\x02")
    assert r.has_error and not r.has_warning and not r.ready_to_move and not r.invalid
    assert decode_reply(_reply(7, 13, 0x08, b"")).invalid
    assert decode_reply(_reply(7, 13, 0x20, b"")).has_warning


def test_a_reply_with_the_wrong_protocol_or_length_is_refused():
    good = _reply(1, 13, 0, b"\x02")
    with pytest.raises(XArmError):
        decode_reply(good[:2] + b"\x00\x00" + good[4:])
    with pytest.raises(XArmError):
        decode_reply(good + b"\x00")
    with pytest.raises(XArmError):
        decode_reply(good[:7])


@given(st.lists(st.binary(max_size=40), max_size=6), st.data())
def test_frames_survive_any_tcp_chunking(payloads, data):
    """However the stream is cut into recv() chunks, the same frames come out."""
    frames = [_reply(i + 1, 13, 0, p) for i, p in enumerate(payloads)]
    stream = b"".join(frames)
    cuts = sorted(data.draw(st.lists(st.integers(0, len(stream)), max_size=8)))
    got, buf, last = [], b"", 0
    for cut in [*cuts, len(stream)]:
        buf += stream[last:cut]
        last = cut
        out, buf = split_frames(buf)
        got += out
    assert got == frames and buf == b""


@given(st.binary(max_size=300))
@settings(max_examples=300)
def test_decoders_only_ever_raise_xarm_error_on_garbage(blob):
    for decode in (decode_reply, decode_report):
        try:
            decode(blob)
        except XArmError:
            pass
    frames, rest = split_frames(blob)
    assert b"".join(frames) + rest == blob


def make_report(
    *,
    state=2,
    mode=0,
    cmd_num=0,
    joints=(0.0,) * 7,
    pose=(300.0, 0.0, 200.0, math.pi, 0.0, 0.0),
    error=0,
    warn=0,
    enable=0x3F,
    offset=(0.0,) * 6,
    size=REPORT_NORMAL_MIN,
) -> bytes:
    body = bytes([(mode << 4) | state]) + struct.pack(">H", cmd_num)
    body += fp32_le(list(joints)) + fp32_le(list(pose)) + fp32_le([0.0] * 7)
    body += bytes([0, enable, error, warn]) + fp32_le(list(offset)) + fp32_le([0.0] * 4)
    body += bytes([3, 3])
    body += bytes(size - 4 - len(body))
    return struct.pack(">I", size) + body


def test_report_fields_land_where_the_sdk_reads_them():
    r = decode_report(
        make_report(state=1, mode=2, cmd_num=3, joints=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0), error=22, warn=11)
    )
    assert (r.state, r.mode, r.cmd_num, r.error_code, r.warn_code) == (1, 2, 3, 22, 11)
    assert r.state_name == "MOVING" and r.mode_name == "JOINT_TEACHING"
    assert r.joints[:6] == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert r.pose_rpy_mm[:3] == pytest.approx([300, 0, 200])
    assert r.enable_bits == 0x3F and r.collision_sensitivity == 3


def test_a_desynchronised_report_is_refused():
    frame = bytearray(make_report())
    frame[131] = 9  # collision sensitivity out of 0..5
    with pytest.raises(XArmError):
        decode_report(bytes(frame))
    with pytest.raises(XArmError):
        decode_report(make_report()[:-1])


def test_the_233_byte_header_quirk_is_accepted():
    """Some firmware announces 233 and sends 245 (the SDK special-cases it)."""
    frame = bytearray(make_report(size=245))
    frame[0:4] = struct.pack(">I", 233)
    assert decode_report(bytes(frame)).state == 2


@pytest.mark.parametrize(
    ("raw", "number", "model"),
    [
        # The SDK's regex: "<axis>,<type>,<arm sn>,<box sn>,…v<maj>.<min>.<rev>"
        (b"6,12,XS1303XXXXXXXX,AC1303XXXXXXXX,v2.5.105\0junk", (2, 5, 105), "850"),
        ("6,6,XF1300,AC1300,v1.11.100", (1, 11, 100), "xArm6"),
        ("v1.9.0", (1, 9, 0), ""),
        ("nonsense", (0, 0, 0), ""),
    ],
)
def test_version_strings(raw, number, model):
    v = parse_version(raw)
    assert v.number == number and v.model == model


def test_a_feedback_frame_and_a_stale_reply_are_skipped(monkeypatch):
    """Byte 6 == 0xFF is motion feedback; an older transaction id is a late reply."""
    import socket

    a, b = socket.socketpair()
    client = xarm.XArmClient("pair.invalid", timeout=2.0)
    client._sock = a
    feedback = struct.pack(">HHHB", 0, 2, 3, FEEDBACK_REGISTER) + b"\x00\x00"
    stale = _reply(99, 13, 0, b"\x01")
    b.sendall(feedback + stale + _reply(1, 13, 0, b"\x02"))
    assert client.get_state() == 2
    b.recv(64)  # the request
    a.close()
    b.close()


def test_a_closed_connection_is_an_xarm_error_and_reconnects_next_time():
    import socket

    a, b = socket.socketpair()
    client = xarm.XArmClient("pair.invalid", timeout=1.0)
    client._sock = a
    b.close()
    with pytest.raises(XArmError):
        client.get_state()
    assert client._sock is None


def test_unreachable_controller_raises_xarm_error_not_a_raw_oserror():
    client = xarm.XArmClient("127.0.0.1", port=1, report_port=1, timeout=0.5)
    with pytest.raises(XArmError):
        client.get_state()
    with pytest.raises(XArmError):
        client.read_report(timeout=0.5)


def test_error_text():
    assert xarm.error_text(0) == ""
    assert "Self-Collision" in xarm.error_text(22)
    assert xarm.error_text(250) == "controller error 250"
    assert "cache is full" in xarm.warning_text(11)
