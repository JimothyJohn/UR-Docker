"""Thin TCP helpers shared by the Dashboard and Primary clients.

Kept in one place so unit tests can monkeypatch a single seam
(``urctl.transport.request_until_close`` / ``send`` / ``send_and_collect``)
instead of mocking raw sockets in every test.
"""

from __future__ import annotations

import socket
import time


def request_until_close(host: str, port: int, payload: bytes, timeout: float = 10.0) -> bytes:
    """Send ``payload``, read until the peer closes (or times out), return bytes.

    This is the Dashboard pattern: the server replies one line per command and
    closes the socket when it receives ``quit``.
    """
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall(payload)
        s.settimeout(timeout)
        chunks: list[bytes] = []
        while True:
            try:
                chunk = s.recv(4096)
            except TimeoutError:
                break
            if not chunk:
                break
            chunks.append(chunk)
    return b"".join(chunks)


def send(host: str, port: int, payload: bytes, timeout: float = 5.0) -> None:
    """Fire-and-forget write — the Primary client pattern (broadcast socket)."""
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall(payload)


def send_and_collect(
    host: str,
    port: int,
    payload: bytes,
    *,
    collect_for: float = 2.0,
    timeout: float = 5.0,
    stop_marker: bytes | None = None,
) -> bytes:
    """Send ``payload`` then keep reading the broadcast for ``collect_for`` s.

    Used to harvest ``textmsg()`` output that the controller interleaves into
    the Primary state stream after running a snippet.

    ``collect_for`` is an upper bound, not a fixed wait: if ``stop_marker`` is
    given and shows up in the stream, reading stops as soon as it arrives (plus
    one trailing recv to capture the rest of that line). This is what keeps a
    confirmed move from blocking the full timeout after it has already landed.
    """
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall(payload)
        s.settimeout(0.5)
        buf = bytearray()
        end = time.monotonic() + collect_for
        while time.monotonic() < end:
            try:
                chunk = s.recv(8192)
            except TimeoutError:
                continue
            if not chunk:
                break
            buf.extend(chunk)
            if stop_marker and stop_marker in buf:
                # Marker seen — grab whatever's immediately available to finish
                # the line, then stop instead of draining the whole window. Use a
                # short timeout: the rest of the marker line is almost always in
                # the same packet, so blocking the full 0.5 s recv timeout here
                # added ~0.5 s of pure latency to every confirmed move.
                s.settimeout(0.1)
                try:
                    tail = s.recv(8192)
                    if tail:
                        buf.extend(tail)
                except TimeoutError:
                    pass
                break
    return bytes(buf)
