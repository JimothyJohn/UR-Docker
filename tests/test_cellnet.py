"""The cell network defaults and the pick PC's "another DHCP server here?" probe.

The probe guards a DHCP server: a wrong "nobody here" on a plant network hands out leases
against the plant's own server, so the parser is held to the wire format and fuzzed (the
bytes come off the network), and the probe runs against a real UDP server on loopback.
"""

from __future__ import annotations

import re
import socket
import struct
import threading
from pathlib import Path

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from perceptronics import cellnet

ROOT = Path(__file__).resolve().parents[1]
MAC = bytes.fromhex("02aabbccddee")


def offer(
    xid: int, *, op: int = 2, msg: int = cellnet.DHCPOFFER, server: str | None = "192.168.3.1"
) -> bytes:
    """A DHCP reply the way a server writes one."""
    zero = b"\0" * 4
    head = struct.pack(
        "!BBBBIHH4s4s4s4s16s64s128s",
        op, 1, 6, 0, xid, 0, 0x8000, zero, socket.inet_aton("192.168.3.50"),
        socket.inet_aton("10.9.9.9"), zero, MAC.ljust(16, b"\0"), b"", b"",
    )  # fmt: skip
    opts = bytes([53, 1, msg])
    if server:
        opts += bytes([54, 4]) + socket.inet_aton(server)
    return head + cellnet.MAGIC + opts + bytes([1, 4, 255, 255, 255, 0, 255])


# -- the packets -----------------------------------------------------------------------------


def test_discover_is_a_broadcast_bootp_request():
    pkt = cellnet.discover(0x12345678, MAC)
    assert len(pkt) >= 240 and pkt[236:240] == cellnet.MAGIC
    op, htype, hlen, _hops, xid, _secs, flags = struct.unpack_from("!BBBBIHH", pkt)
    assert (op, htype, hlen, xid, flags) == (1, 1, 6, 0x12345678, 0x8000)
    assert pkt[28:34] == MAC
    assert pkt[240:243] == bytes([53, 1, cellnet.DHCPDISCOVER]) and pkt.endswith(b"\xff")


def test_discover_refuses_a_mac_that_is_not_six_bytes():
    with pytest.raises(ValueError):
        cellnet.discover(1, b"\x02\x00")


def test_an_offer_names_its_server():
    assert cellnet.offer_server(offer(7), 7) == "192.168.3.1"


def test_an_offer_without_a_server_id_falls_back_to_siaddr():
    assert cellnet.offer_server(offer(7, server=None), 7) == "10.9.9.9"


@pytest.mark.parametrize(
    "packet",
    [
        offer(8),  # someone else's transaction
        offer(7, op=1),  # a request, not a reply (another client's DISCOVER on the wire)
        offer(7, msg=5),  # an ACK
        offer(7)[:240],  # no options
        offer(7)[:200],  # short
        offer(7).replace(cellnet.MAGIC, b"\0\0\0\0"),  # no magic cookie: plain BOOTP
        offer(7)[:244],  # option 54 cut off mid-value
    ],
    ids=["other-xid", "request", "ack", "no-options", "short", "no-cookie", "truncated"],
)
def test_anything_but_our_offer_is_not_an_offer(packet):
    assert cellnet.offer_server(packet, 7) is None


@settings(max_examples=400, deadline=None)
@given(st.binary(max_size=600), st.integers(min_value=0, max_value=2**32 - 1))
def test_the_parser_never_raises_on_network_bytes(data, xid):
    got = cellnet.offer_server(data, xid)
    assert got is None or isinstance(got, str)


@settings(max_examples=200, deadline=None)
@given(st.binary(max_size=80))
def test_trailing_garbage_after_valid_options_never_raises(tail):
    pkt = offer(7)[:-1] + tail  # drop the end option, append anything
    got = cellnet.offer_server(pkt, 7)
    assert got is None or isinstance(got, str)


# -- the probe, against a real UDP server on loopback -------------------------------------------


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class FakeServer:
    """Answers each DISCOVER with ``reply(xid)`` (None: stays silent)."""

    def __init__(self, reply):
        self.reply = reply
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.stop = threading.Event()
        self.port = self.sock.getsockname()[1]
        self.seen: list[bytes] = []
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self.stop.is_set():
            try:
                pkt, addr = self.sock.recvfrom(4096)
            except TimeoutError:
                continue
            self.seen.append(pkt)
            answer = self.reply(struct.unpack_from("!I", pkt, 4)[0])
            if answer is not None:
                self.sock.sendto(answer, addr)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join(5)
        self.sock.close()


def _probe(server: FakeServer, **kw) -> str | None:
    return cellnet.probe(
        None, target="127.0.0.1", server_port=server.port, client_port=_free_udp_port(), **kw
    )


def test_probe_hears_another_dhcp_server():
    with FakeServer(lambda xid: offer(xid, server="10.0.0.1")) as srv:
        assert _probe(srv, timeout_s=2) == "10.0.0.1"
    assert srv.seen and srv.seen[0][240:243] == bytes([53, 1, cellnet.DHCPDISCOVER])


def test_probe_on_a_quiet_port_finds_nobody_after_every_try():
    with FakeServer(lambda xid: None) as srv:
        assert _probe(srv, timeout_s=0.2, tries=2) is None
    assert len(srv.seen) == 2
    # each try is its own transaction, from a locally administered MAC (never a real NIC's)
    assert len({p[4:8] for p in srv.seen}) == 2 and all(p[28] & 0x02 for p in srv.seen)


def test_probe_ignores_replies_that_are_not_offers_to_it():
    with FakeServer(lambda xid: offer(xid ^ 1)) as srv:
        assert _probe(srv, timeout_s=0.3, tries=1) is None
    with FakeServer(lambda xid: offer(xid, msg=5)) as srv:
        assert _probe(srv, timeout_s=0.3, tries=1) is None


def test_cli_refuses_an_interface_name_that_is_not_one(capsys):
    assert cellnet.main(["probe", "eth0; reboot"]) == 2
    assert cellnet.main(["probe", "x" * 16]) == 2


def test_cli_prints_the_defaults(capsys):
    assert cellnet.main(["defaults"]) == 0
    out = capsys.readouterr().out
    assert "PICK_PC_ADDRESS=192.168.3.20" in out and "ROBOT_ADDRESS=192.168.3.3" in out


# -- one address, everywhere it is written ------------------------------------------------------


def _const(path: str, pattern: str) -> str:
    m = re.search(pattern, (ROOT / path).read_text(encoding="utf-8"))
    assert m, f"{path}: {pattern}"
    return m.group(1)


def test_the_urcaps_and_the_installer_agree_on_the_pick_pcs_address():
    pc, robot = cellnet.PICK_PC_ADDRESS, cellnet.ROBOT_ADDRESS
    java = "urcap/perceptronic-ps5/src/io/advin/perceptronic/Cockpit.java"
    js = "urcap/perceptronic/perceptronic-frontend/pickscript.js"
    assert _const(java, r'DEFAULT_HOST = "([\d.]+)"') == pc
    assert _const(java, r'ROBOT_DEFAULT_HOST = "([\d.]+)"') == robot
    assert _const(js, r'DEFAULT_COCKPIT_HOST = "([\d.]+)"') == pc
    assert _const(js, r'ROBOT_DEFAULT_HOST = "([\d.]+)"') == robot
    assert (
        _const("urcap/perceptronic/perceptronic-frontend/main.js", r'DEFAULT_COCKPIT_HOST = "([\d.]+)"') == pc
    )
    assert _const("deploy/pi/install.sh", r'cell_address="([\d.]+)/(?:\d+)"') == pc
    assert int(_const("deploy/pi/install.sh", r'cell_address="[\d.]+/(\d+)"')) == cellnet.CELL_PREFIX


def test_the_robot_address_is_the_shipped_cells_ur_host_and_on_the_pick_pcs_network():
    # the lease the pick PC hands out is where the cockpit looks for the robot
    from perceptronics.cell import load_cell

    assert load_cell("ur3")["UR_HOST"] == cellnet.ROBOT_ADDRESS
    net = cellnet.PICK_PC_ADDRESS.rsplit(".", 1)[0]
    assert cellnet.ROBOT_ADDRESS.rsplit(".", 1)[0] == net and cellnet.CELL_PREFIX == 24
    assert cellnet.ROBOT_ADDRESS != cellnet.PICK_PC_ADDRESS
