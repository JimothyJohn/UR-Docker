"""perceptronics.uvc: finding the camera terminal in a raw configuration descriptor,
the PERCEPTRONICS_VIEW_FOCUS grammar, and set_focus's never-raise contract."""

from __future__ import annotations

import pytest

from perceptronics import uvc
from perceptronics.pickcycle import PickCycle


def _iface(number: int, cls: int, sub: int) -> bytes:
    return bytes([9, 0x04, number, 0, 0, cls, sub, 0, 0])


def _input_terminal(tid: int, ttype: int) -> bytes:
    return bytes([18, 0x24, 0x02, tid, ttype & 0xFF, ttype >> 8]) + bytes(12)


CONFIG_HEADER = bytes([9, 0x02, 0, 0, 2, 1, 0, 0x80, 250])
IAD = bytes([8, 0x0B, 0, 2, 0x0E, 0x03, 0, 0])


def test_finds_the_camera_terminal_of_the_videocontrol_interface():
    raw = CONFIG_HEADER + IAD + _iface(0, 0x0E, 0x01) + _input_terminal(1, 0x0201) + _iface(1, 0x0E, 0x02)
    assert uvc.find_camera_terminal(raw) == uvc.CameraTerminal(interface=0, terminal_id=1)


def test_camera_terminal_on_a_later_interface_with_another_id():
    # an audio function first (its class-specific descriptors must not count), then video on 2
    audio = _iface(0, 0x01, 0x01) + _input_terminal(9, 0x0201)
    raw = CONFIG_HEADER + audio + _iface(2, 0x0E, 0x01) + _input_terminal(4, 0x0201)
    assert uvc.find_camera_terminal(raw) == uvc.CameraTerminal(interface=2, terminal_id=4)


def test_a_non_camera_input_terminal_is_not_the_camera():
    # ITT_MEDIA_TRANSPORT_INPUT (0x0202) on a VideoControl interface
    raw = CONFIG_HEADER + _iface(0, 0x0E, 0x01) + _input_terminal(1, 0x0202)
    assert uvc.find_camera_terminal(raw) is None


@pytest.mark.parametrize(
    "raw",
    [
        b"",
        b"\x00",
        b"\x00\x00\x00",  # zero-length descriptor: must not loop forever
        CONFIG_HEADER + _iface(0, 0x0E, 0x01) + _input_terminal(1, 0x0201)[:10],  # truncated
        bytes([255, 0x04]) + bytes(20),  # claims more than there is
    ],
)
def test_malformed_descriptors_give_none(raw):
    assert uvc.find_camera_terminal(raw) is None


def test_focus_spec_grammar():
    assert uvc.parse_focus_spec(None) is None
    assert uvc.parse_focus_spec("  ") is None
    assert uvc.parse_focus_spec("auto") is None
    assert uvc.parse_focus_spec("0") == 0
    assert uvc.parse_focus_spec("C920=0, C920e=auto,") == {"C920": 0, "C920e": None}
    with pytest.raises(ValueError):
        uvc.parse_focus_spec("C920=near")


def test_focus_for_picks_the_longest_matching_name():
    spec = {"C920": 30, "C920e": None}
    assert uvc.focus_for(spec, "Logi Webcam C920e") == (True, None)
    assert uvc.focus_for(spec, "HD Pro Webcam C920") == (True, 30)
    assert uvc.focus_for(spec, "FaceTime HD Camera") == (False, None)
    assert uvc.focus_for(0, "anything") == (True, 0)
    assert uvc.focus_for(None, "anything") == (False, None)


def test_set_focus_never_raises(monkeypatch):
    assert uvc.set_focus("lavfi:testsrc", 0)["ok"] is True

    def no_libusb():
        raise OSError("libusb-1.0 not found")

    monkeypatch.setattr(uvc, "_libusb", no_libusb)
    monkeypatch.setattr(uvc.sys, "platform", "darwin")
    out = uvc.set_focus("C920", 0)
    assert out["ok"] is False and "libusb" in out["error"]


def test_narrated_events_carry_plain_words_beside_the_engineers_line():
    lines = []
    cycle = PickCycle(dry_run=True, say_fn=lines.append)
    cycle.say("Found 3 block(s)", think="I see 3 blocks.")
    cycle.say("Done")
    assert lines[0]["text"] == "Found 3 block(s)" and lines[0]["think"] == "I see 3 blocks."
    assert "do" not in lines[0]
    assert set(lines[1]) == {"t", "text"}
