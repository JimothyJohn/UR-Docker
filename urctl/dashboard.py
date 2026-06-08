"""DashboardClient — the line-oriented orchestration surface (port 29999).

The Dashboard server is the network equivalent of tapping buttons in
PolyScope: power on, load a program, query state. It is line-oriented ASCII —
each command is one line, the server replies one line per command and only
after it has fully executed.

This client wraps that protocol with typed helpers for the commands you
actually reach for, plus :meth:`command` for anything not wrapped and
:meth:`wait_for` for polling state transitions.
"""

from __future__ import annotations

import time
from collections.abc import Callable

from . import transport
from .config import RobotConfig

# Framing lines the server emits around real replies; stripped so callers can
# assert on the actual command output.
_FRAMING_PREFIXES = ("Connected:", "Disconnected")


class DashboardClient:
    def __init__(self, config: RobotConfig | None = None):
        self.config = config or RobotConfig.from_env()

    # ----- raw protocol ------------------------------------------------------

    def command(self, *commands: str, timeout: float | None = None) -> str:
        """Send one or more Dashboard commands in a single session.

        Returns the joined server replies with framing lines stripped. A
        trailing ``quit`` is appended so the server closes the socket cleanly.
        """
        timeout = self.config.timeout if timeout is None else timeout
        payload = ("\n".join(commands) + "\nquit\n").encode("utf-8")
        raw = transport.request_until_close(
            self.config.host, self.config.dashboard_port, payload, timeout=timeout
        )
        text = raw.decode("utf-8", errors="replace")
        lines = [ln for ln in text.splitlines() if ln and not ln.startswith(_FRAMING_PREFIXES)]
        return "\n".join(lines)

    def wait_for(
        self,
        predicate: Callable[[str], bool],
        *,
        query: str = "robotmode",
        timeout: float = 60.0,
        interval: float = 1.0,
    ) -> str:
        """Poll ``query`` until ``predicate(reply)`` is truthy. Returns last reply."""
        end = time.monotonic() + timeout
        last = ""
        while time.monotonic() < end:
            try:
                last = self.command(query)
            except OSError:
                last = ""
            if predicate(last):
                return last
            time.sleep(interval)
        raise TimeoutError(f"timed out waiting on {query!r}; last reply: {last!r}")

    # ----- state queries -----------------------------------------------------

    def robot_mode(self) -> str:
        return self.command("robotmode")

    def safety_mode(self) -> str:
        return self.command("safetymode")

    def program_state(self) -> str:
        return self.command("programState")

    def is_running(self) -> bool:
        return "RUNNING" in self.robot_mode()

    def is_remote_control(self) -> bool:
        # Returns "true"/"false"; absent on CB-series or older firmware.
        return "true" in self.command("is in remote control").lower()

    def control_mode(self) -> str:
        """``"REMOTE"`` / ``"LOCAL"`` — the Dashboard analogue of the PolyScope X
        ``/system/v1/controlmode``. Lets :meth:`Robot.get_state` report the same
        ``control_mode`` field on both platforms."""
        return "REMOTE" if self.is_remote_control() else "LOCAL"

    # ----- power / brakes ----------------------------------------------------

    def power_on(self) -> str:
        return self.command("power on")

    def power_off(self) -> str:
        return self.command("power off")

    def brake_release(self) -> str:
        return self.command("brake release")

    # ----- safety ------------------------------------------------------------

    def unlock_protective_stop(self) -> str:
        return self.command("unlock protective stop")

    def close_safety_popup(self) -> str:
        return self.command("close safety popup")

    def close_popup(self) -> str:
        return self.command("close popup")

    # ----- programs ----------------------------------------------------------

    def load(self, program: str) -> str:
        """Load ``<program>.urp``. The matching ``.installation`` must sit beside it."""
        if not program.endswith(".urp"):
            program += ".urp"
        return self.command(f"load {program}")

    def play(self) -> str:
        return self.command("play")

    def stop(self) -> str:
        return self.command("stop")

    def pause(self) -> str:
        return self.command("pause")

    # ----- misc --------------------------------------------------------------

    def popup(self, text: str) -> str:
        return self.command(f"popup {text}")

    def add_to_log(self, text: str) -> str:
        return self.command(f"addToLog {text}")
