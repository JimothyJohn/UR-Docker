"""PrimaryClient — the execution surface (port 30001).

Anything written to the Primary Client is parsed as URScript and runs on the
controller immediately. The same socket also broadcasts a ~10 Hz binary state
stream into which the controller interleaves ``textmsg()`` output. We don't
decode the binary protocol; we scan for printable ASCII runs and pull out the
``textmsg`` lines we asked for. (For high-rate structured telemetry you'd use
RTDE on 30004 — out of scope for this control-oriented toolkit.)
"""

from __future__ import annotations

import re
import threading

from . import transport
from .config import RobotConfig

# Strings of >=4 printable ASCII chars — used to harvest textmsg output from
# the binary broadcast.
_ASCII_RUN = re.compile(rb"[\x20-\x7e]{4,}")

# A 6-element bracketed vector: bare int, decimal, or scientific notation.
_NUM = r"-?\d+(?:\.\d+)?(?:e[+-]?\d+)?"
_VEC6_RX = re.compile(rf"\[({_NUM}(?:,\s*{_NUM}){{5}})\]")


def _wrap(fn_name: str, body: str) -> str:
    """Wrap a URScript body in a single ``def`` so the controller treats the
    whole submission as one program.

    URControl treats each newline-terminated top-level statement as a separate
    program and kills the previous one when a new one starts — which would
    abort a multi-line motion before it runs. Wrapping keeps it on one program.
    The controller logs a cosmetic ``Compile error: name '<fn>' is not
    defined`` afterward (PolyScope auto-wraps inbound script, interfering with
    the outer call, not the inner body); the body still executes. See CLAUDE.md.
    """
    indented = "".join(f"  {line}\n" for line in body.strip().splitlines())
    return f"def {fn_name}():\n{indented}end\n{fn_name}()\n"


BUSY_MESSAGE = (
    "Primary channel busy: a program sent by this client is still running. "
    "Any new URScript would replace (kill) it — wait for it to finish or stop it first."
)


class PrimaryBusyError(OSError):
    """A second URScript submission was refused while one is still in flight.

    The controller runs one program at a time: a new submission on 30001
    silently kills the current one. That is exactly how a state poll's
    ``textmsg`` fallback, a Locate, or a second Move can cut an approach cycle
    short mid-path (seen 2026-09-23 on the UR3e: a 4-leg cycle stopped after
    leg 2 with no fault). So one client never lets that happen: the second
    caller gets this error instead, and ``Robot.get_state`` leaves joints
    unset rather than send anything.
    """


class PrimaryClient:
    def __init__(self, config: RobotConfig | None = None):
        self.config = config or RobotConfig.from_env()
        # Held for the whole send(+capture) of one submission. Non-blocking:
        # a concurrent caller is refused, never queued behind a running move.
        self._inflight = threading.Lock()

    @property
    def busy(self) -> bool:
        """True while a submission from this client is on the wire / being waited on."""
        if self._inflight.acquire(blocking=False):
            self._inflight.release()
            return False
        return True

    def send(self, urscript: str) -> None:
        """Stream a URScript snippet; the controller runs it immediately.

        Does not wait for a response — Primary is a broadcast channel.
        """
        if not self._inflight.acquire(blocking=False):
            raise PrimaryBusyError(BUSY_MESSAGE)
        try:
            transport.send(
                self.config.host,
                self.config.primary_port,
                urscript.encode("utf-8"),
                timeout=self.config.timeout,
            )
        finally:
            self._inflight.release()

    def send_and_capture(
        self,
        urscript: str,
        *,
        marker: str = "",
        collect_for: float = 2.0,
        stop_marker: str = "",
    ) -> list[str]:
        """Send URScript, then read the broadcast for up to ``collect_for`` seconds.

        Returns every printable ASCII run; if ``marker`` is given, only runs
        containing it. If ``stop_marker`` is given, reading stops as soon as it
        appears in the stream rather than waiting the whole window — used to
        return promptly once a move's ``done`` textmsg lands.
        """
        if not self._inflight.acquire(blocking=False):
            raise PrimaryBusyError(BUSY_MESSAGE)
        try:
            raw = transport.send_and_collect(
                self.config.host,
                self.config.primary_port,
                urscript.encode("utf-8"),
                collect_for=collect_for,
                timeout=self.config.timeout,
                stop_marker=stop_marker.encode("utf-8") if stop_marker else None,
            )
        finally:
            self._inflight.release()
        marker_b = marker.encode("utf-8")
        return [
            m.group(0).decode("ascii", errors="replace")
            for m in _ASCII_RUN.finditer(raw)
            if not marker_b or marker_b in m.group(0)
        ]

    def run(self, body: str, *, fn_name: str = "urctl_snippet") -> None:
        """Wrap ``body`` in a ``def`` and run it (no output captured)."""
        self.send(_wrap(fn_name, body))

    def run_and_capture(
        self,
        body: str,
        *,
        fn_name: str = "urctl_snippet",
        marker: str = "",
        collect_for: float = 2.0,
        stop_marker: str = "",
    ) -> list[str]:
        """Wrap ``body`` in a ``def``, run it, and capture marked textmsg output."""
        return self.send_and_capture(
            _wrap(fn_name, body),
            marker=marker,
            collect_for=collect_for,
            stop_marker=stop_marker,
        )

    # ----- state read --------------------------------------------------------

    def read_state(self, *, collect_for: float = 2.0) -> dict[str, list[float] | None]:
        """Read joint positions and TCP pose via a textmsg round-trip.

        Returns ``{"joints": [...]|None, "tcp": [...]|None}``. ``None`` means
        the value didn't surface in the capture window (try a longer
        ``collect_for`` on a slow link).
        """
        captured = self.run_and_capture(
            'textmsg("urctl/state/joints=", get_actual_joint_positions())\n'
            'textmsg("urctl/state/tcp=", get_actual_tcp_pose())\n',
            fn_name="urctl_read_state",
            marker="urctl/state",
            collect_for=collect_for,
            # tcp is the second (last) textmsg — once it's in, both are present.
            stop_marker="urctl/state/tcp=",
        )
        return {
            "joints": parse_vector(captured, "urctl/state/joints"),
            "tcp": parse_vector(captured, "urctl/state/tcp"),
        }


def parse_vector(captured: list[str], tag: str) -> list[float] | None:
    """Pull a 6-vector out of a ``tag=[...]`` textmsg line."""
    for line in captured:
        if tag in line:
            m = _VEC6_RX.search(line)
            if m:
                return [float(x) for x in m.group(1).split(",")]
    return None
