"""RtdeClient — Real-Time Data Exchange (port 30004), pure stdlib.

RTDE is the *modern* state surface of a UR controller. Where the Primary
client (30001) broadcasts a fixed ~10 Hz binary stream into which we have to
scrape ASCII ``textmsg`` output (see :mod:`urctl.primary`), RTDE is a
**subscription protocol**: the client names exactly the fields it wants, the
controller replies with their types, and from then on streams them as packed
binary at a chosen cadence (up to the 500 Hz control rate). Three things this
buys the toolkit that the textmsg hack cannot:

  * **Structured, typed reads** — joints, TCP pose *and* velocities, TCP
    force/torque, safety status, IO bits — decoded from binary, no regex.
  * **Works without a running program** — RTDE reflects the controller, not a
    URScript ``textmsg`` we injected, so state is readable while IDLE/STOPPED.
  * **Writes** — the same protocol accepts *input* recipes (speed slider,
    digital outputs), so it is a genuine remote-control surface, not just telemetry.

This module hand-rolls the wire protocol with ``socket`` + ``struct`` (no
external dependency), matching the zero-dependency style of
:mod:`urctl.transport`. Every frame is ``>HB`` — uint16 total size (including
the 3-byte header) then a uint8 package type — followed by a type-specific,
**big-endian** payload. The handshake is: negotiate protocol v2 -> setup the
output recipe -> setup the input recipe -> START; data then flows as ``U``
packages. See https://docs.universal-robots.com/.../rtde-guide.html.

Reads are kept *fresh* rather than buffered: each :meth:`read_outputs` pauses,
re-starts, takes one sample and pauses again, so an on-demand read never
returns a packet that has been sitting in the socket buffer for seconds.
"""

from __future__ import annotations

import socket
import struct

from .config import RobotConfig

# --- package type bytes (the uint8 after the size in every frame) ------------
_RTDE_REQUEST_PROTOCOL_VERSION = 86  # 'V'
_RTDE_TEXT_MESSAGE = 77  # 'M' — async controller message, skipped
_RTDE_DATA_PACKAGE = 85  # 'U' — a streamed (output) or written (input) sample
_RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # 'O'
_RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73  # 'I'
_RTDE_CONTROL_PACKAGE_START = 83  # 'S'
_RTDE_CONTROL_PACKAGE_PAUSE = 80  # 'P'

PROTOCOL_VERSION = 2

# RTDE field type string -> (big-endian struct format, element count). The
# controller tells us each field's type in the SETUP reply, so this table is
# the single source of truth for how wide a field is and how to (de)serialize
# it — no hardcoded byte offsets anywhere.
_RTDE_TYPES: dict[str, tuple[str, int]] = {
    "VECTOR6D": (">6d", 6),
    "VECTOR3D": (">3d", 3),
    "VECTOR6INT32": (">6i", 6),
    "VECTOR6UINT32": (">6I", 6),
    "DOUBLE": (">d", 1),
    "UINT64": (">Q", 1),
    "UINT32": (">I", 1),
    "INT32": (">i", 1),
    "BOOL": (">?", 1),
    "UINT8": (">B", 1),
}

# A small, control-oriented default output recipe. Only field *names* — the
# controller reports the types, so this list is robust to INT32-vs-UINT32 drift
# between firmware versions. Robot.rtde_state() maps these to friendly keys.
DEFAULT_OUTPUTS: list[str] = [
    "timestamp",
    "robot_mode",
    "safety_status_bits",
    "runtime_state",
    "actual_q",
    "actual_qd",
    "actual_TCP_pose",
    "actual_TCP_speed",
    "actual_TCP_force",
    "actual_digital_input_bits",
    "actual_digital_output_bits",
]
DEFAULT_OUTPUT_FREQUENCY = 125.0  # Hz — ample for on-demand state, cheap on the controller.

# The input recipe we register for writes. ``*_mask`` fields select which
# values in the same package actually apply, so one field can be written
# without disturbing the others (e.g. nudge the speed slider but leave IO alone).
INPUT_FIELDS: list[str] = [
    "speed_slider_mask",
    "speed_slider_fraction",
    "standard_digital_output_mask",
    "standard_digital_output",
]


class RtdeError(Exception):
    """Raised on an RTDE protocol failure (rejected handshake, NOT_FOUND field,
    recipe mismatch, or a peer that closed mid-frame)."""


class RtdeClient:
    """A stateful RTDE connection. Use as a context manager or let
    :class:`urctl.Robot` hold one lazily.

        with RtdeClient(RobotConfig(host="10.0.0.5")) as c:
            print(c.read_outputs()["actual_q"])
            c.set_speed_slider(0.3)
    """

    def __init__(
        self,
        config: RobotConfig | None = None,
        *,
        outputs: list[str] | None = None,
        frequency: float = DEFAULT_OUTPUT_FREQUENCY,
    ):
        self.config = config or RobotConfig.from_env()
        self.outputs = list(outputs) if outputs is not None else list(DEFAULT_OUTPUTS)
        self.frequency = frequency
        self._sock: socket.socket | None = None
        self._out_recipe: list[tuple[str, str]] = []  # (name, type) in order
        self._out_recipe_id: int = 0
        self._in_recipe: list[tuple[str, str]] = []
        self._in_recipe_id: int = 0
        self._started = False

    # ----- lifecycle ---------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        """Open the socket and run the full handshake (version, recipes)."""
        if self._sock is not None:
            return
        sock = socket.create_connection(
            (self.config.host, self.config.rtde_port), timeout=self.config.timeout
        )
        sock.settimeout(self.config.timeout)
        self._sock = sock
        self._in_recipe = []  # inputs are set up lazily, only when first written
        self._in_recipe_id = 0
        try:
            self._negotiate_version()
            self._setup_outputs()
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
                self._started = False

    def __enter__(self) -> RtdeClient:
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ----- framing (the seam unit tests patch) -------------------------------

    def _send(self, ptype: int, payload: bytes = b"") -> None:
        assert self._sock is not None
        size = 3 + len(payload)
        self._sock.sendall(struct.pack(">HB", size, ptype) + payload)

    def _recv_exactly(self, n: int) -> bytes:
        """Read exactly ``n`` bytes, looping over short reads. The single most
        important RTDE correctness detail: a frame can arrive split across
        several ``recv`` calls."""
        assert self._sock is not None
        chunks: list[bytes] = []
        remaining = n
        while remaining > 0:
            chunk = self._sock.recv(remaining)
            if not chunk:
                raise RtdeError("RTDE connection closed mid-frame")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _recv_package(self) -> tuple[int, bytes]:
        head = self._recv_exactly(3)
        size, ptype = struct.unpack(">HB", head)
        body = self._recv_exactly(size - 3)
        return ptype, body

    def _recv_control(self, expected: int) -> bytes:
        """Read packages until the expected control reply arrives, skipping
        async text messages and any stray data packages."""
        while True:
            ptype, body = self._recv_package()
            if ptype in (_RTDE_TEXT_MESSAGE, _RTDE_DATA_PACKAGE):
                continue
            if ptype == expected:
                return body
            raise RtdeError(f"expected RTDE package {expected}, got {ptype}")

    # ----- handshake ---------------------------------------------------------

    def _negotiate_version(self) -> None:
        self._send(_RTDE_REQUEST_PROTOCOL_VERSION, struct.pack(">H", PROTOCOL_VERSION))
        body = self._recv_control(_RTDE_REQUEST_PROTOCOL_VERSION)
        if not body or body[0] != 1:
            raise RtdeError(f"controller rejected RTDE protocol v{PROTOCOL_VERSION}")

    def _parse_recipe(self, body: bytes, names: list[str]) -> tuple[int, list[tuple[str, str]]]:
        """A SETUP reply: recipe_id (uint8) + comma-joined type names. Raises
        if any requested field came back NOT_FOUND or is an unknown type."""
        recipe_id = body[0]
        types = body[1:].decode("ascii", errors="replace").split(",")
        if len(types) != len(names):
            raise RtdeError(f"recipe reply had {len(types)} types for {len(names)} fields")
        missing = [n for n, t in zip(names, types, strict=False) if t == "NOT_FOUND"]
        if missing:
            raise RtdeError(f"RTDE fields not available on this controller: {missing}")
        in_use = [n for n, t in zip(names, types, strict=False) if t == "IN_USE"]
        if in_use:
            raise RtdeError(f"RTDE input(s) already controlled by another client: {in_use}")
        unknown = [t for t in types if t not in _RTDE_TYPES]
        if unknown:
            raise RtdeError(f"unsupported RTDE field type(s): {sorted(set(unknown))}")
        return recipe_id, list(zip(names, types, strict=False))

    def _setup_outputs(self) -> None:
        payload = struct.pack(">d", self.frequency) + ",".join(self.outputs).encode("ascii")
        self._send(_RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, payload)
        body = self._recv_control(_RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS)
        self._out_recipe_id, self._out_recipe = self._parse_recipe(body, self.outputs)

    def _ensure_inputs(self) -> None:
        """Register the input recipe on first write. Done lazily (not at connect)
        so a read-only client never *claims* the speed-slider/IO variables —
        which would lock other clients out (IN_USE) and put inputs under RTDE
        control needlessly."""
        if self._in_recipe:
            return
        self._send(_RTDE_CONTROL_PACKAGE_SETUP_INPUTS, ",".join(INPUT_FIELDS).encode("ascii"))
        body = self._recv_control(_RTDE_CONTROL_PACKAGE_SETUP_INPUTS)
        self._in_recipe_id, self._in_recipe = self._parse_recipe(body, INPUT_FIELDS)

    def _start(self) -> None:
        if self._started:
            return
        self._send(_RTDE_CONTROL_PACKAGE_START)
        body = self._recv_control(_RTDE_CONTROL_PACKAGE_START)
        if not body or body[0] != 1:
            raise RtdeError("controller refused RTDE START (variable already controlled?)")
        self._started = True

    def _pause(self) -> None:
        if not self._started:
            return
        self._send(_RTDE_CONTROL_PACKAGE_PAUSE)
        self._recv_control(_RTDE_CONTROL_PACKAGE_PAUSE)  # PAUSE always succeeds
        self._started = False

    # ----- read path ---------------------------------------------------------

    def receive(self) -> dict:
        """Block for the next data package and decode it by the output recipe."""
        while True:
            ptype, body = self._recv_package()
            if ptype == _RTDE_TEXT_MESSAGE:
                continue
            if ptype != _RTDE_DATA_PACKAGE:
                continue
            if body and body[0] != self._out_recipe_id:
                # A data package for some other recipe id — ignore.
                continue
            return self._decode(body[1:])

    def _decode(self, payload: bytes) -> dict:
        out: dict = {}
        off = 0
        for name, tstr in self._out_recipe:
            fmt, n = _RTDE_TYPES[tstr]
            width = struct.calcsize(fmt)
            vals = struct.unpack(fmt, payload[off : off + width])
            off += width
            out[name] = list(vals) if n > 1 else vals[0]
        return out

    def read_outputs(self) -> dict:
        """Connect if needed and return one *fresh* output sample.

        Re-syncs (pause -> start -> sample -> pause) so the returned packet is
        current rather than whatever has accumulated in the socket buffer since
        the last read.
        """
        if not self.connected:
            self.connect()
        self._pause()
        self._start()
        try:
            return self.receive()
        finally:
            self._pause()

    # ----- write path --------------------------------------------------------

    def send_input(self, values: dict) -> None:
        """Pack ``values`` per the input recipe and send them, then consume one
        output cycle so we know the controller advanced past applying them.
        Missing fields default to 0 (their masks should be 0 too)."""
        if not self.connected:
            self.connect()
        self._ensure_inputs()
        self._start()
        parts = [struct.pack(">B", self._in_recipe_id)]
        for name, tstr in self._in_recipe:
            fmt, n = _RTDE_TYPES[tstr]
            v = values.get(name, 0)
            parts.append(struct.pack(fmt, *v) if n > 1 else struct.pack(fmt, v))
        self._send(_RTDE_DATA_PACKAGE, b"".join(parts))
        self.receive()  # let one control cycle elapse so the write has taken effect
        self._pause()

    def set_speed_slider(self, fraction: float) -> None:
        """Set the global speed slider (0-1); leaves digital outputs untouched."""
        self.send_input(
            {
                "speed_slider_mask": 1,
                "speed_slider_fraction": float(fraction),
                "standard_digital_output_mask": 0,
                "standard_digital_output": 0,
            }
        )

    def set_standard_digital_output(self, pin: int, value: bool) -> None:
        """Set standard digital output ``pin`` (0-7) high/low; leaves the speed
        slider untouched."""
        bit = 1 << pin
        self.send_input(
            {
                "speed_slider_mask": 0,
                "speed_slider_fraction": 0.0,
                "standard_digital_output_mask": bit,
                "standard_digital_output": bit if value else 0,
            }
        )
