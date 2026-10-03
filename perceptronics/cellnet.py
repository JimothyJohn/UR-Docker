"""The pick PC's cell network: the address the robot and the URCap assume, and the
"is somebody else already serving DHCP here?" probe.

The defaults are chosen to work with a robot nobody has configured: UR tells operators to
pick **DHCP** on the robot's network screen (Settings → System → Network), so the pick PC
holds a fixed address on its cell port (:data:`PICK_PC_ADDRESS`) and answers DHCP there
with one lease, :data:`ROBOT_ADDRESS` — the address the shipped cell profiles give
``UR_HOST`` — so a robot on DHCP is where the cockpit looks for it, and the URCap's empty
Cockpit field means the pick PC (``urcap/perceptronic-ps5/.../Cockpit.java``,
``pickscript.js`` hold the same constant; a test keeps them equal).

A DHCP server on a network that already has one takes leases away from it, so the pick PC
only serves after :func:`probe` heard no other server on the port
(``perceptronics-cell-dhcp.service``'s ``ExecCondition=``, re-run whenever the port comes
up). ``python3 -m perceptronics.cellnet probe eth0`` exits 0 when the port is free to
serve, 1 when another server answered, 2 when the probe itself could not run.
"""

from __future__ import annotations

import argparse
import secrets
import socket
import struct
import sys
import time

PICK_PC_ADDRESS = "192.168.3.20"
CELL_PREFIX = 24
ROBOT_ADDRESS = "192.168.3.3"

MAGIC = b"\x63\x82\x53\x63"
BOOTREQUEST, BOOTREPLY = 1, 2
DHCPDISCOVER, DHCPOFFER = 1, 2
FLAG_BROADCAST = 0x8000
SERVER_PORT, CLIENT_PORT = 67, 68
_FIXED = struct.Struct("!BBBBIHH4s4s4s4s16s64s128s")  # the BOOTP header, 236 bytes


def discover(xid: int, mac: bytes) -> bytes:
    """A DHCPDISCOVER from ``mac`` asking for a broadcast reply (no address of our own)."""
    if len(mac) != 6:
        raise ValueError("mac must be 6 bytes")
    zero = b"\0" * 4
    head = _FIXED.pack(
        BOOTREQUEST, 1, 6, 0, xid, 0, FLAG_BROADCAST, zero, zero, zero, zero, mac.ljust(16, b"\0"), b"", b""
    )
    # 53 message type, 55 parameter request list (subnet, router), 255 end
    options = bytes([53, 1, DHCPDISCOVER, 55, 2, 1, 3, 255])
    return head + MAGIC + options


def offer_server(packet: bytes, xid: int) -> str | None:
    """The answering server's address if ``packet`` is a DHCPOFFER for ``xid``, else None.

    Anything else — short, another transaction, a request, no magic cookie, truncated or
    malformed options — is None, never an exception: the bytes come off the network.
    """
    if len(packet) < _FIXED.size + len(MAGIC):
        return None
    op, _htype, _hlen, _hops, got_xid, _secs, _flags, _ci, _yi, siaddr, _gi, _ch, _sn, _fi = (
        _FIXED.unpack_from(packet)
    )
    if op != BOOTREPLY or got_xid != xid or packet[_FIXED.size : _FIXED.size + 4] != MAGIC:
        return None
    msg_type = None
    server_id = None
    i = _FIXED.size + 4
    while i < len(packet):
        code = packet[i]
        if code == 255:
            break
        if code == 0:
            i += 1
            continue
        if i + 1 >= len(packet):
            return None
        size = packet[i + 1]
        value = packet[i + 2 : i + 2 + size]
        if len(value) != size:
            return None
        if code == 53 and size == 1:
            msg_type = value[0]
        elif code == 54 and size == 4:
            server_id = socket.inet_ntoa(value)
        i += 2 + size
    if msg_type != DHCPOFFER:
        return None
    return server_id or socket.inet_ntoa(siaddr)


def probe(
    iface: str | None,
    *,
    timeout_s: float = 3.0,
    tries: int = 2,
    target: str = "255.255.255.255",
    server_port: int = SERVER_PORT,
    client_port: int = CLIENT_PORT,
) -> str | None:
    """Ask for a lease on ``iface`` and return the first server that offers one, or None.

    Nothing is accepted: an offer is only listened for, so no lease is taken from that
    server. ``iface`` None skips the device binding (tests on loopback, with ``target`` and
    the ports pointed at a fake server); binding to a device needs root or CAP_NET_RAW.
    """
    mac = bytes([0x02]) + secrets.token_bytes(5)  # locally administered: never a real NIC's
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        if iface is not None:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface.encode() + b"\0")
        sock.bind(("" if iface is not None else "127.0.0.1", client_port))
        for _ in range(max(1, tries)):
            xid = secrets.randbits(32)
            sock.sendto(discover(xid, mac), (target, server_port))
            deadline = time.monotonic() + timeout_s
            while (left := deadline - time.monotonic()) > 0:
                sock.settimeout(left)
                try:
                    packet, _ = sock.recvfrom(4096)
                except TimeoutError:
                    break
                server = offer_server(packet, xid)
                if server is not None:
                    return server
        return None
    finally:
        sock.close()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="python3 -m perceptronics.cellnet", description=__doc__.split("\n\n")[0]
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser(
        "probe", help="exit 0: no other DHCP server on IFACE; 1: one answered; 2: could not probe"
    )
    p.add_argument("iface")
    p.add_argument("--timeout", type=float, default=3.0, help="seconds to wait per try (default 3)")
    sub.add_parser("defaults", help="print the cell addresses as KEY=VALUE")
    args = ap.parse_args(argv)
    if args.cmd == "defaults":
        print(f"PICK_PC_ADDRESS={PICK_PC_ADDRESS}\nCELL_PREFIX={CELL_PREFIX}\nROBOT_ADDRESS={ROBOT_ADDRESS}")
        return 0
    if not args.iface.replace("-", "").replace(".", "").replace("_", "").isalnum() or len(args.iface) > 15:
        print("cellnet: not an interface name", file=sys.stderr)
        return 2
    try:
        server = probe(args.iface, timeout_s=args.timeout)
    except OSError as exc:
        print(f"cellnet: could not probe {args.iface}: {exc.strerror or exc}", file=sys.stderr)
        return 2
    if server is not None:
        print(
            f"cellnet: {args.iface}: DHCP server {server} already answers here: not serving", file=sys.stderr
        )
        return 1
    print(f"cellnet: {args.iface}: no other DHCP server answered: serving the robot's lease")
    return 0


if __name__ == "__main__":
    sys.exit(main())
