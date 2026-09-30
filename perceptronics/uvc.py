"""UVC camera controls for the views — today one thing: **lock the focus**.

A Logitech C920 left on continuous autofocus hunts whenever the scene changes
(an arm swinging through, a warm lamp behind it): every other second the
picture goes soft and snaps back, which ruins a timelapse (2026-09-27, the
``HD Pro Webcam C920`` room view). Autofocus off + a fixed focus fixes it, and
the camera keeps the setting until it loses power — ffmpeg reopening it does
not reset it (verified the same day: the cockpit restarted, both stayed locked).

How it is sent: a UVC ``SET_CUR`` class request on endpoint 0, addressed to the
device's *camera terminal* (``CT_FOCUS_AUTO_CONTROL`` = 0, then
``CT_FOCUS_ABSOLUTE_CONTROL``). On macOS the request goes through libusb
(ctypes, the Homebrew ``libusb-1.0.dylib`` librealsense already needs) and
**needs neither root nor claiming an interface** — the device stays with
Apple's UVC driver and keeps streaming (verified as the login user, not under
the cockpit's sudo; ``pick-cycle --record`` applies it from its own process).
Intel devices are never opened: a libusb open of the D435 on this Mac is what
re-enumerates it. On Linux, ``/dev/videoN`` views go
through ``v4l2-ctl`` (unverified). Windows: not implemented (the result says so).

The camera terminal's ID and the VideoControl interface number are read from
the device's own configuration descriptor, not assumed.

    python -m perceptronics.uvc                      # every UVC camera: autofocus + focus
    python -m perceptronics.uvc lock "C920" 0        # autofocus off, focus 0 (far)
    python -m perceptronics.uvc auto "C920"          # back to autofocus
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass

GET_CUR, SET_CUR = 0x81, 0x01
CT_FOCUS_ABSOLUTE_CONTROL, CT_FOCUS_AUTO_CONTROL = 0x06, 0x08
CC_VIDEO, SC_VIDEOCONTROL = 0x0E, 0x01
CS_INTERFACE, VC_INPUT_TERMINAL, ITT_CAMERA = 0x24, 0x02, 0x0201
INTEL_VID = 0x8086  # RealSense: the cockpit owns it (docs/realsense.md §Troubleshooting)
LIBUSB_CANDIDATES = (
    "/opt/homebrew/lib/libusb-1.0.dylib",
    "/usr/local/lib/libusb-1.0.dylib",
    "libusb-1.0.so.0",
)


@dataclass(frozen=True)
class CameraTerminal:
    interface: int  # the VideoControl interface number
    terminal_id: int  # bTerminalID of the ITT_CAMERA input terminal


def find_camera_terminal(config: bytes) -> CameraTerminal | None:
    """The camera terminal in a raw USB configuration descriptor: the first
    VideoControl interface, then the ``VC_INPUT_TERMINAL`` of type ``ITT_CAMERA``
    among its class-specific descriptors. None when there is none (not a UVC
    camera, or a truncated descriptor)."""
    i, vc_if = 0, None
    while i + 2 <= len(config):
        length, kind = config[i], config[i + 1]
        if length < 2 or i + length > len(config):
            return None
        d = config[i : i + length]
        if kind == 0x04 and length >= 9:  # INTERFACE
            vc_if = d[2] if (d[5], d[6]) == (CC_VIDEO, SC_VIDEOCONTROL) else None
        elif kind == CS_INTERFACE and vc_if is not None and length >= 8 and d[2] == VC_INPUT_TERMINAL:
            if d[4] | (d[5] << 8) == ITT_CAMERA:
                return CameraTerminal(vc_if, d[3])
        i += length
    return None


def parse_focus_spec(text: str | None) -> dict[str, int | None] | int | None:
    """``PERCEPTRONICS_VIEW_FOCUS`` → what to do with each view's focus.

    - empty → None: leave the cameras alone;
    - ``"0"`` → 0: lock every view at that focus;
    - ``"C920=0,C920e=auto"`` → per view (a case-insensitive substring of the
      view's name; ``auto`` = continuous autofocus back on)."""
    text = (text or "").strip()
    if not text:
        return None
    if "=" not in text:
        return None if text.lower() == "auto" else int(text)
    out: dict[str, int | None] = {}
    for part in text.split(","):
        if not part.strip():
            continue
        name, _, value = part.partition("=")
        value = value.strip().lower()
        out[name.strip()] = None if value == "auto" else int(value)
    return out


def focus_for(spec: dict[str, int | None] | int | None, view_name: str) -> tuple[bool, int | None]:
    """``(act, focus)`` for one view: ``act`` False = leave it alone; ``focus``
    None = autofocus on, else locked at that value. The longest matching key wins."""
    if spec is None:
        return False, None
    if isinstance(spec, int):
        return True, spec
    hits = [k for k in spec if k.lower() in view_name.lower()]
    if not hits:
        return False, None
    return True, spec[max(hits, key=len)]


# -- libusb (macOS; Linux works too with device permissions) ---------------------------


class _Desc(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8),
        ("bDescriptorType", ctypes.c_uint8),
        ("bcdUSB", ctypes.c_uint16),
        ("bDeviceClass", ctypes.c_uint8),
        ("bDeviceSubClass", ctypes.c_uint8),
        ("bDeviceProtocol", ctypes.c_uint8),
        ("bMaxPacketSize0", ctypes.c_uint8),
        ("idVendor", ctypes.c_uint16),
        ("idProduct", ctypes.c_uint16),
        ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8),
        ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8),
        ("bNumConfigurations", ctypes.c_uint8),
    ]


def _libusb():
    path = os.environ.get("LIBUSB_LIB")
    names = [path] if path else [*LIBUSB_CANDIDATES, ctypes.util.find_library("usb-1.0")]
    for name in names:
        if not name:
            continue
        try:
            lib = ctypes.CDLL(name)
        except OSError:
            continue
        vp, u8, u16 = ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint16
        lib.libusb_init.argtypes = [ctypes.POINTER(vp)]
        lib.libusb_exit.argtypes = [vp]
        lib.libusb_get_device_list.argtypes = [vp, ctypes.POINTER(ctypes.POINTER(vp))]
        lib.libusb_get_device_list.restype = ctypes.c_ssize_t
        lib.libusb_free_device_list.argtypes = [ctypes.POINTER(vp), ctypes.c_int]
        lib.libusb_get_device_descriptor.argtypes = [vp, ctypes.POINTER(_Desc)]
        lib.libusb_open.argtypes = [vp, ctypes.POINTER(vp)]
        lib.libusb_close.argtypes = [vp]
        lib.libusb_control_transfer.argtypes = [vp, u8, u8, u16, u16, ctypes.c_char_p, u16, ctypes.c_uint]
        lib.libusb_get_string_descriptor_ascii.argtypes = [vp, u8, ctypes.c_char_p, ctypes.c_int]
        lib.libusb_error_name.argtypes = [ctypes.c_int]
        lib.libusb_error_name.restype = ctypes.c_char_p
        return lib
    raise OSError("libusb-1.0 not found (brew install libusb; or set LIBUSB_LIB)")


class _Camera:
    def __init__(self, lib, handle, name: str, terminal: CameraTerminal):
        self.lib, self.handle, self.name, self.terminal = lib, handle, name, terminal

    def _xfer(self, request_type: int, request: int, selector: int, buf: bytes | ctypes.Array) -> int:
        t = self.terminal
        n = len(buf)
        r = self.lib.libusb_control_transfer(
            self.handle,
            request_type,
            request,
            selector << 8,
            (t.terminal_id << 8) | t.interface,
            buf,
            n,
            1000,
        )
        if r < 0:
            raise OSError(f"{self.name}: control transfer failed ({self.lib.libusb_error_name(r).decode()})")
        return r

    def get(self, selector: int, size: int) -> int:
        buf = ctypes.create_string_buffer(size)
        self._xfer(0xA1, GET_CUR, selector, buf)
        return int.from_bytes(buf.raw, "little")

    def set(self, selector: int, value: int, size: int) -> None:
        self._xfer(0x21, SET_CUR, selector, value.to_bytes(size, "little"))

    def status(self) -> dict:
        return {
            "name": self.name,
            "autofocus": bool(self.get(CT_FOCUS_AUTO_CONTROL, 1)),
            "focus": self.get(CT_FOCUS_ABSOLUTE_CONTROL, 2),
        }


def _each_camera(fn):
    """Call ``fn(camera)`` for every attached UVC camera with a camera terminal."""
    lib = _libusb()
    ctx = ctypes.c_void_p()
    if lib.libusb_init(ctypes.byref(ctx)) != 0:
        raise OSError("libusb_init failed")
    devs = ctypes.POINTER(ctypes.c_void_p)()
    try:
        n = lib.libusb_get_device_list(ctx, ctypes.byref(devs))
        for i in range(max(0, n)):
            desc = _Desc()
            if lib.libusb_get_device_descriptor(devs[i], ctypes.byref(desc)) != 0:
                continue
            if desc.bDeviceClass not in (0x00, 0xEF):  # per-interface or IAD composite: where UVC lives
                continue
            if desc.idVendor == INTEL_VID:
                continue  # never open the RealSense: on macOS every libusb open of it re-enumerates it
            handle = ctypes.c_void_p()
            if lib.libusb_open(devs[i], ctypes.byref(handle)) != 0:
                continue
            try:
                raw = ctypes.create_string_buffer(4096)
                got = lib.libusb_control_transfer(handle, 0x80, 0x06, 0x0200, 0, raw, 4096, 1000)
                terminal = find_camera_terminal(raw.raw[: max(0, got)])
                if terminal is None:
                    continue
                name = ctypes.create_string_buffer(128)
                lib.libusb_get_string_descriptor_ascii(handle, desc.iProduct, name, 128)
                fn(_Camera(lib, handle, name.value.decode(errors="replace"), terminal))
            finally:
                lib.libusb_close(handle)
        if n > 0:
            lib.libusb_free_device_list(devs, 1)
    finally:
        lib.libusb_exit(ctx)


def cameras() -> list[dict]:
    """Autofocus + focus of every attached UVC camera (libusb)."""
    out: list[dict] = []

    def one(cam: _Camera) -> None:
        try:
            out.append(cam.status())
        except OSError as exc:
            out.append({"name": cam.name, "error": str(exc)})

    _each_camera(one)
    return out


def set_focus(name: str, focus: int | None) -> dict:
    """Lock ``focus`` (autofocus off) on the camera(s) whose USB product name
    contains ``name`` — or ``focus=None``: autofocus back on. A ``/dev/videoN``
    name goes through ``v4l2-ctl`` instead. Never raises: the result carries
    ``ok`` and, on failure, ``error``."""
    if name.startswith("lavfi:"):
        return {"ok": True, "name": name, "skipped": "synthetic view"}
    if name.startswith("/dev/video"):
        return _v4l2_focus(name, focus)
    if sys.platform == "win32":
        return {"ok": False, "name": name, "error": "focus control is not implemented on Windows"}
    done: list[dict] = []

    def one(cam: _Camera) -> None:
        if name.lower() not in cam.name.lower():
            return
        try:
            if focus is None:
                cam.set(CT_FOCUS_AUTO_CONTROL, 1, 1)
            else:
                cam.set(CT_FOCUS_AUTO_CONTROL, 0, 1)
                cam.set(CT_FOCUS_ABSOLUTE_CONTROL, int(focus), 2)
            done.append(cam.status())
        except OSError as exc:
            done.append({"name": cam.name, "error": str(exc)})

    try:
        _each_camera(one)
    except OSError as exc:
        return {"ok": False, "name": name, "error": str(exc)}
    if not done:
        return {"ok": False, "name": name, "error": f"no UVC camera named like {name!r}"}
    errors = [d["error"] for d in done if "error" in d]
    return {"ok": not errors, "name": name, "cameras": done, **({"error": errors[0]} if errors else {})}


def _v4l2_focus(device: str, focus: int | None) -> dict:
    tool = shutil.which("v4l2-ctl")
    if tool is None:
        return {"ok": False, "name": device, "error": "v4l2-ctl not found (apt install v4l-utils)"}
    # the kernel renamed focus_auto → focus_automatic_continuous (5.x); try both
    for auto in ("focus_automatic_continuous", "focus_auto"):
        ctrls = f"{auto}=1" if focus is None else f"{auto}=0,focus_absolute={int(focus)}"
        proc = subprocess.run(
            [tool, "-d", device, "-c", ctrls],
            capture_output=True,
            text=True,
            stdin=subprocess.DEVNULL,
            check=False,
        )
        if proc.returncode == 0:
            return {"ok": True, "name": device, "focus": focus}
    return {"ok": False, "name": device, "error": (proc.stderr or proc.stdout).strip()[-200:]}


def main(argv: list[str] | None = None) -> int:
    import json

    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print(json.dumps(cameras(), indent=2))
        return 0
    if argv[0] == "lock" and len(argv) == 3:
        out = set_focus(argv[1], int(argv[2]))
    elif argv[0] == "auto" and len(argv) == 2:
        out = set_focus(argv[1], None)
    else:
        print(__doc__.strip().splitlines()[-3:], file=sys.stderr)
        return 2
    print(json.dumps(out, indent=2))
    return 0 if out.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
