"""HTTP client for a running cockpit (``perception gui``).

Only one process can hold the camera, so everything else that wants frames —
the MCP server an agent drives, a script, the doctor — talks to the cockpit
that already owns it, over the same ``/api`` the browser uses. Stdlib only.

    c = CockpitClient()                      # http://127.0.0.1:7621
    c.info()["fps"]
    c.snapshot(dir="captures/snapshots")     # → {"color_png": ..., "depth_png": ...}
    c.segment(x=400, y=240)["features"]["point_m"]
    c.locate(standoff_m=0.1)["approach_pose"]
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from collections.abc import Sequence

DEFAULT_COCKPIT_URL = "http://127.0.0.1:7621"
ENV_COCKPIT_URL = "PERCEPTION_COCKPIT_URL"


class CockpitUnavailable(RuntimeError):
    """The cockpit is not running (or not where we were told)."""


class CockpitError(RuntimeError):
    """The cockpit answered with an error (bad request, robot refused, …)."""


class CockpitClient:
    def __init__(self, url: str | None = None, *, timeout: float = 30.0):
        self.url = (url or os.environ.get(ENV_COCKPIT_URL) or DEFAULT_COCKPIT_URL).rstrip("/")
        self.timeout = timeout

    # -- transport ---------------------------------------------------------------

    def _request(self, method: str, path: str, body: dict | None = None) -> dict:
        data = json.dumps(body or {}).encode() if method == "POST" else None
        req = urllib.request.Request(
            self.url + path, data=data, method=method, headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                payload = json.loads(r.read())
        except urllib.error.HTTPError as exc:
            try:
                payload = json.loads(exc.read())
            except Exception:
                payload = {"ok": False, "error": f"HTTP {exc.code}"}
            raise CockpitError(str(payload.get("error") or f"HTTP {exc.code}")) from None
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise CockpitUnavailable(
                f"no cockpit at {self.url} ({exc}). Start one: `perception --cell <cell> gui` "
                f"(or set {ENV_COCKPIT_URL})"
            ) from None
        return payload

    def get(self, path: str) -> dict:
        return self._request("GET", path)

    def post(self, path: str, body: dict | None = None) -> dict:
        return self._request("POST", path, body)

    # -- camera --------------------------------------------------------------------

    def info(self) -> dict:
        return self.get("/api/info")

    def snapshot(self, *, dir: str | None = None, name: str | None = None) -> dict:
        body: dict = {}
        if dir:
            body["dir"] = dir
        if name:
            body["name"] = name
        return self.post("/api/snapshot", body)

    def segment(
        self, *, x: int | None = None, y: int | None = None, box: Sequence[int] | None = None
    ) -> dict:
        body: dict = {}
        if x is not None and y is not None:
            body.update({"x": int(x), "y": int(y)})
        if box is not None:
            body["box"] = [int(v) for v in box]
        return self.post("/api/segment", body)

    def nearest(self, near_ratio: float = 1.2) -> dict:
        return self.post("/api/nearest", {"near_ratio": float(near_ratio)})

    def clear(self) -> dict:
        return self.post("/api/clear")

    def capture(self, name: str = "object", include_mask: bool = True) -> dict:
        return self.post("/api/capture", {"name": name, "include_mask": bool(include_mask)})

    def captures(self) -> dict:
        return self.get("/api/captures")

    def events(self, after: int = 0, limit: int = 200) -> dict:
        return self.get(f"/api/events?after={int(after)}&limit={int(limit)}")

    def doctor(self, *, robot: bool = True) -> dict:
        return self.get(f"/api/doctor?robot={'1' if robot else '0'}")

    # -- robot (through the cockpit's link) ------------------------------------------

    def robot_state(self) -> dict:
        return self.post("/api/robot/state")

    def locate(
        self,
        *,
        standoff_m: float | None = None,
        point_m: Sequence[float] | None = None,
        reference: str | None = None,
    ) -> dict:
        body: dict = {}
        if standoff_m is not None:
            body["standoff_m"] = float(standoff_m)
        if point_m is not None:
            body["point_m"] = [float(v) for v in point_m]
        if reference is not None:
            body["reference"] = str(reference)
        return self.post("/api/robot/locate", body)

    def approach_cycle(self, **kwargs) -> dict:
        body = {k: v for k, v in kwargs.items() if v is not None}
        return self.post("/api/robot/approach_cycle", body)

    def move(
        self, pose: Sequence[float], *, velocity: float | None = None, tcp: Sequence[float] | None = None
    ) -> dict:
        body: dict = {"pose": [float(v) for v in pose]}
        if tcp is not None:
            body["tcp"] = [float(v) for v in tcp]
        if velocity is not None:
            body["velocity"] = float(velocity)
        return self.post("/api/robot/move", body)

    def jog(self, delta: Sequence[float], *, velocity: float | None = None) -> dict:
        body: dict = {"delta": [float(v) for v in delta]}
        if velocity is not None:
            body["velocity"] = float(velocity)
        return self.post("/api/robot/jog", body)
