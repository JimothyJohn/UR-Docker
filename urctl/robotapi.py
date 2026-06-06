"""RobotAPIClient — the PolyScope X orchestration surface (REST "Robot-API").

PolyScope X (PolyScope 10) **removed the line-oriented Dashboard server** that
the e-Series exposes on port 29999. Its orchestration equivalent — power the
robot on, release brakes, load a program, play/stop it, query modes — lives
behind a RESTful HTTP "Robot-API" served from::

    http://<host>:<port>/universal-robots/robot-api

This client is a **drop-in replacement for** :class:`urctl.dashboard.DashboardClient`:
it exposes the same method names and the same return-string conventions, so the
:class:`urctl.robot.Robot` facade can sit on either one unchanged (it picks the
client by ``config.platform``). URScript *execution* is unchanged across
platforms — it still rides the Primary client on 30001 (see
:class:`urctl.primary.PrimaryClient`).

Endpoints (verified against the URSim PolyScope X Robot-API, v3.1.6):

    GET  /robotstate/v1/robotmode      -> {"mode": "POWER_OFF"|"IDLE"|"RUNNING"|..., "message"}
    GET  /robotstate/v1/safetymode     -> {"mode": "NORMAL"|"PROTECTIVE_STOP"|..., "message"}
    PUT  /robotstate/v1/state          <- {"action": "POWER_ON"|"POWER_OFF"|"BRAKE_RELEASE"|
                                                      "UNLOCK_PROTECTIVE_STOP"|"RESTART_SAFETY"}
    GET  /program/v1/state             -> {"state": "STOPPED"|"PLAYING"|"PAUSED", "message", "details"}
    PUT  /program/v1/state             <- {"action": "play"|"pause"|"resume"|"stop"}
    GET  /program/v1/loaded            -> loaded program name
    PUT  /program/v1/loaded            <- {"name": "<program>"}
    GET  /system/v1/controlmode        -> {"mode": "LOCAL"|"REMOTE"}
    GET  /system/v1/operationalmode    -> {"mode": "AUTOMATIC"|"MANUAL"}

**Remote-mode gate.** Every *mutating* call (power, brake, load, play, …) is
rejected with HTTP 403 unless the robot is in **Remote** control mode, and there
is no REST endpoint to switch Local->Remote — it is a toggle on the Safety
screen in the PolyScope X UI. We turn that 403 into a clear, actionable message
rather than a bare error.
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from collections.abc import Callable

from .config import RobotConfig


def _http_json(
    method: str, url: str, *, body: dict | None = None, timeout: float = 10.0
) -> tuple[int, dict | None]:
    """Perform one HTTP request and return ``(status_code, parsed_json_or_None)``.

    This is the single seam the client funnels through — the REST analogue of
    :mod:`urctl.transport`, so unit tests monkeypatch *this* function instead of
    mocking sockets. Uses only the stdlib (``urllib``) to keep the toolkit
    dependency-free.

    A non-2xx HTTP status is returned as data (with the parsed error body), not
    raised — mirroring the Dashboard client, which returns the server's error
    *text* rather than throwing. A genuine connection failure
    (:class:`urllib.error.URLError`, a subclass of :class:`OSError`) still
    propagates, exactly as a socket error would on the Dashboard path.
    """
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            status = resp.status
            raw = resp.read()
    except urllib.error.HTTPError as exc:  # 4xx/5xx carry a useful JSON body
        status = exc.code
        raw = exc.read()
    text = raw.decode("utf-8", errors="replace") if raw else ""
    parsed: dict | None = None
    if text:
        try:
            decoded = json.loads(text)
            parsed = decoded if isinstance(decoded, dict) else {"value": decoded}
        except json.JSONDecodeError:
            parsed = {"message": text}
    return status, parsed


# Map the friendly client method to the Robot-API RobotStateChange enum value.
_ROBOT_STATE_ACTIONS = {
    "power_on": "POWER_ON",
    "power_off": "POWER_OFF",
    "brake_release": "BRAKE_RELEASE",
    "unlock_protective_stop": "UNLOCK_PROTECTIVE_STOP",
    "restart_safety": "RESTART_SAFETY",
}


class RobotAPIClient:
    def __init__(self, config: RobotConfig | None = None):
        self.config = config or RobotConfig.from_env()

    # ----- raw protocol ------------------------------------------------------

    def _get(self, path: str) -> tuple[int, dict | None]:
        return _http_json(
            "GET", self.config.robot_api_url + path, timeout=self.config.timeout
        )

    def _put(self, path: str, payload: dict) -> tuple[int, dict | None]:
        return _http_json(
            "PUT", self.config.robot_api_url + path, body=payload, timeout=self.config.timeout
        )

    @staticmethod
    def _error_text(status: int, body: dict | None) -> str:
        """Render a non-2xx response as one human-readable line.

        Special-cases the 403 remote-mode gate into actionable guidance, since
        that is by far the most common reason a PolyScope X command is refused.
        """
        msg = ""
        if isinstance(body, dict):
            msg = body.get("message") or ""
            details = body.get("details")
            if details:
                msg = f"{msg} — {details}" if msg else details
        if status == 403:
            return (
                "Forbidden (403): PolyScope X refused this because the robot is not in "
                "Remote control mode. Switch to Remote on the Safety screen in the "
                "PolyScope X UI (localhost:8000), then retry."
                + (f" [robot said: {msg}]" if msg else "")
            )
        return f"Error ({status}): {msg}" if msg else f"Error ({status})"

    def _put_reply(self, path: str, payload: dict, success: str) -> str:
        """PUT and return ``success`` on 2xx, else a clear error line.

        The ``success`` string is chosen to contain the substring the
        :class:`~urctl.robot.Robot` facade looks for (e.g. ``"Loading program"``,
        ``"Starting program"``), so the facade's existing ok-detection works
        verbatim across both platforms.
        """
        status, body = self._put(path, payload)
        if 200 <= status < 300:
            return success
        return self._error_text(status, body)

    def _mode(self, path: str, key: str) -> str:
        status, body = self._get(path)
        if 200 <= status < 300 and isinstance(body, dict) and key in body:
            return str(body[key])
        return self._error_text(status, body)

    def command(self, *commands: str, timeout: float | None = None) -> str:
        """Not available on PolyScope X — there is no Dashboard server.

        Kept so the duck-typed surface matches :class:`DashboardClient`; returns
        a clear note (so :meth:`Robot.dashboard_command` reports it rather than
        pretending) instead of raising.
        """
        return (
            "Dashboard commands are not available on PolyScope X (no Dashboard server on "
            "29999). Use the urctl subcommands — they route through the Robot-API."
        )

    def wait_for(
        self,
        predicate: Callable[[str], bool],
        *,
        query: str = "robotmode",
        timeout: float = 60.0,
        interval: float = 1.0,
    ) -> str:
        """Poll ``query`` until ``predicate(reply)`` is truthy. Returns last reply.

        Signature-compatible with :meth:`DashboardClient.wait_for`; ``query`` is
        the Dashboard verb name, mapped here to the matching Robot-API getter.
        """
        getter = {
            "robotmode": self.robot_mode,
            "safetymode": self.safety_mode,
            "programState": self.program_state,
        }.get(query, self.robot_mode)
        end = time.monotonic() + timeout
        last = ""
        while time.monotonic() < end:
            try:
                last = getter()
            except OSError:  # URLError (connection refused) subclasses OSError
                last = ""
            if predicate(last):
                return last
            time.sleep(interval)
        raise TimeoutError(f"timed out waiting on {query!r}; last reply: {last!r}")

    # ----- state queries -----------------------------------------------------

    def robot_mode(self) -> str:
        return self._mode("/robotstate/v1/robotmode", "mode")

    def safety_mode(self) -> str:
        return self._mode("/robotstate/v1/safetymode", "mode")

    def program_state(self) -> str:
        return self._mode("/program/v1/state", "state")

    def control_mode(self) -> str:
        """``"LOCAL"`` or ``"REMOTE"`` — the Robot-API has no analogue on
        e-Series Dashboard beyond ``is in remote control``."""
        return self._mode("/system/v1/controlmode", "mode")

    def operational_mode(self) -> str:
        """``"AUTOMATIC"`` or ``"MANUAL"``."""
        return self._mode("/system/v1/operationalmode", "mode")

    def is_running(self) -> bool:
        return "RUNNING" in self.robot_mode()

    def is_remote_control(self) -> bool:
        return self.control_mode() == "REMOTE"

    # ----- power / brakes ----------------------------------------------------

    def power_on(self) -> str:
        return self._put_reply(
            "/robotstate/v1/state", {"action": _ROBOT_STATE_ACTIONS["power_on"]}, "Powering on"
        )

    def power_off(self) -> str:
        return self._put_reply(
            "/robotstate/v1/state", {"action": _ROBOT_STATE_ACTIONS["power_off"]}, "Powering off"
        )

    def brake_release(self) -> str:
        return self._put_reply(
            "/robotstate/v1/state",
            {"action": _ROBOT_STATE_ACTIONS["brake_release"]},
            "Brake releasing",
        )

    # ----- safety ------------------------------------------------------------

    def unlock_protective_stop(self) -> str:
        return self._put_reply(
            "/robotstate/v1/state",
            {"action": _ROBOT_STATE_ACTIONS["unlock_protective_stop"]},
            "Protective stop unlocked",
        )

    def close_safety_popup(self) -> str:
        """No Robot-API analogue (PolyScope X has no Dashboard popups). No-op so
        :meth:`Robot.bring_up`'s unconditional call is harmless."""
        return ""

    def close_popup(self) -> str:
        return ""

    # ----- programs ----------------------------------------------------------

    def load(self, program: str) -> str:
        """Load a program by name. PolyScope X programs are ``.urpx`` and are
        addressed by *name* (no ``.urp`` suffix, unlike the Dashboard ``load``)."""
        name = program[:-4] if program.endswith(".urp") else program
        return self._put_reply(
            "/program/v1/loaded", {"name": name}, f"Loading program: {name}"
        )

    def play(self) -> str:
        return self._put_reply("/program/v1/state", {"action": "play"}, "Starting program")

    def stop(self) -> str:
        return self._put_reply("/program/v1/state", {"action": "stop"}, "Stopped")

    def pause(self) -> str:
        return self._put_reply("/program/v1/state", {"action": "pause"}, "Pausing program")

    def resume(self) -> str:
        return self._put_reply("/program/v1/state", {"action": "resume"}, "Resuming program")

    # ----- misc --------------------------------------------------------------

    def popup(self, text: str) -> str:
        """No Robot-API popup endpoint. For an on-pendant message, send a
        URScript ``popup()`` via ``run-script`` instead."""
        return (
            "(popup is not available via the PolyScope X Robot-API; send a URScript "
            'popup("...") with `urctl run-script` instead)'
        )

    def add_to_log(self, text: str) -> str:
        return "(addToLog is not available via the PolyScope X Robot-API)"
