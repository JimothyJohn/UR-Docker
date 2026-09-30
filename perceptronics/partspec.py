"""The part the operator is looking for, by its rough size — the detector's filter.

Without one the detector keeps anything block-sized (the foam blocks' hardcoded gate in
:func:`perceptronics.pickcycle.detect_blocks`). With one it keeps only candidates whose
top face measures ``length × width`` (either way round) and, when a height is given,
that stand ``height`` above the surface around them — each within ``tol`` (a fraction,
never tighter than :data:`MIN_SLACK_M`, the measurement's own noise at working range).

The dimensions are **as the part lies on the table**: length and width are its
footprint, height is how far its top is above the table. The table is flat and
parallel to the base XY plane, so the top face is the face the camera measures.

Text form (the node's request token, the teach screen's query): ``60x40`` or
``60x40x30`` in millimetres, ``tol`` in percent — ``part=60x40x30 tol=25``.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

MIN_DIM_MM, MAX_DIM_MM = 5.0, 500.0
MIN_TOL_PCT, MAX_TOL_PCT = 5.0, 100.0
DEFAULT_TOL_PCT = 25.0
MIN_SLACK_M = 0.005  # extents from a 4 px grid at 0.3-0.5 m, heights from D435 depth: ±5 mm is noise

_NUM = r"\d{1,4}(?:\.\d{1,3})?"
PART_RX = re.compile(rf"\bpart=({_NUM})[xX]({_NUM})(?:[xX]({_NUM}))?(?![\w.])")
TOL_RX = re.compile(rf"\btol=({_NUM})(?![\w.])")


@dataclass(frozen=True)
class PartSpec:
    length_m: float  # footprint, the longer side (normalised on construction)
    width_m: float
    height_m: float | None = None  # top above the surface; None: not checked
    tol: float = DEFAULT_TOL_PCT / 100.0

    def __post_init__(self) -> None:
        dims = [self.length_m, self.width_m] + ([self.height_m] if self.height_m is not None else [])
        if not all(isinstance(d, (int, float)) and math.isfinite(d) for d in dims + [self.tol]):
            raise ValueError("part dimensions must be finite numbers")
        if not all(MIN_DIM_MM / 1000 <= d <= MAX_DIM_MM / 1000 for d in dims):
            raise ValueError(f"part dimensions must be {MIN_DIM_MM:.0f}..{MAX_DIM_MM:.0f} mm")
        if not (MIN_TOL_PCT / 100 <= self.tol <= MAX_TOL_PCT / 100):
            raise ValueError(f"tolerance must be {MIN_TOL_PCT:.0f}..{MAX_TOL_PCT:.0f} %")
        if self.width_m > self.length_m:
            a, b = self.width_m, self.length_m
            object.__setattr__(self, "length_m", a)
            object.__setattr__(self, "width_m", b)

    @classmethod
    def from_mm(
        cls, length: float, width: float, height: float | None = None, tol_pct: float = DEFAULT_TOL_PCT
    ):
        return cls(length / 1000.0, width / 1000.0, None if not height else height / 1000.0, tol_pct / 100.0)

    def slack(self, dim_m: float) -> float:
        return max(self.tol * dim_m, MIN_SLACK_M)

    def why_not(self, major_m: float, minor_m: float, height_m: float | None) -> str | None:
        """Why a candidate with these measurements is not this part, or None when it is.
        An unmeasured height (no surface visible around it) is not held against it."""
        lo, hi = lambda d: d - self.slack(d), lambda d: d + self.slack(d)  # noqa: E731
        if major_m > hi(self.length_m):
            n = round(major_m / self.length_m)
            if n >= 2 and abs(major_m - n * self.length_m) <= self.slack(n * self.length_m):
                return f"{n} parts touching?"
            return "too long"
        if minor_m > hi(self.width_m):
            n = round(minor_m / self.width_m)
            if n >= 2 and abs(minor_m - n * self.width_m) <= self.slack(n * self.width_m):
                return f"{n} parts touching?"
            return "too wide"
        if major_m < lo(self.length_m):
            return "too short"
        if minor_m < lo(self.width_m):
            return "too narrow"
        if self.height_m is not None and height_m is not None:
            if height_m > hi(self.height_m):
                return "too tall"
            if height_m < lo(self.height_m):
                return "too flat"
        return None

    def token(self) -> str:
        """``part=60x40x30 tol=25`` — what the node sends and :func:`parse` reads back."""
        dims = [self.length_m, self.width_m] + ([self.height_m] if self.height_m is not None else [])
        return "part=" + "x".join(_num(d * 1000) for d in dims) + f" tol={_num(self.tol * 100)}"

    def as_dict(self) -> dict:
        return {
            "length_mm": round(self.length_m * 1000, 1),
            "width_mm": round(self.width_m * 1000, 1),
            "height_mm": None if self.height_m is None else round(self.height_m * 1000, 1),
            "tol_pct": round(self.tol * 100, 1),
        }


def _num(x: float) -> str:
    return f"{x:.1f}".rstrip("0").rstrip(".")


def _dims(text: str) -> tuple[float, float, float | None] | None:
    m = PART_RX.search(text)
    if not m:
        if re.search(r"\bpart=", text):
            raise ValueError("part must be LxW or LxWxH in mm, e.g. part=60x40x30")
        return None
    return float(m.group(1)), float(m.group(2)), float(m.group(3)) if m.group(3) else None


def parse(text: str) -> PartSpec | None:
    """The ``part=LxW[xH] [tol=T]`` token in ``text``, or None when there is none.
    ValueError when there is one and it is malformed or out of range."""
    dims = _dims(text)
    if dims is None:
        return None
    tol = DEFAULT_TOL_PCT
    if re.search(r"\btol=", text):
        t = TOL_RX.search(text)
        if not t:
            raise ValueError("tol must be a percentage, e.g. tol=25")
        tol = float(t.group(1))
    return PartSpec.from_mm(*dims, tol_pct=tol)


_DIMS_EXACT = re.compile(rf"({_NUM})[xX]({_NUM})(?:[xX]({_NUM}))?")
_TOL_EXACT = re.compile(_NUM)


def _exact(part: str, tol) -> PartSpec:
    """A bare ``60x40[x30]`` and a tolerance — the whole value, nothing around it."""
    m = _DIMS_EXACT.fullmatch(part.strip())
    if not m:
        raise ValueError('part must be LxW or LxWxH in mm, like "60x40x30"')
    dims = float(m.group(1)), float(m.group(2)), float(m.group(3)) if m.group(3) else None
    try:
        tol_pct = float(tol)
    except OverflowError:
        raise ValueError("tol must be a percentage, like 25") from None
    return PartSpec.from_mm(*dims, tol_pct=tol_pct)


def from_query(qs: dict) -> PartSpec | None:
    """``?part=60x40x30&tol=25`` (``urllib.parse.parse_qs`` shape) → PartSpec or None;
    ValueError when malformed."""
    part = (qs.get("part") or [""])[0]
    if not part:
        return None
    tol = (qs.get("tol") or [""])[0]
    if tol and not _TOL_EXACT.fullmatch(tol.strip()):
        raise ValueError("tol must be a percentage, like 25")
    return _exact(part, tol.strip() or DEFAULT_TOL_PCT)


def from_payload(payload: dict) -> PartSpec | None:
    """A JSON body's ``part`` (``"60x40x30"``) and ``tol`` (percent) → PartSpec or None;
    ValueError when malformed."""
    part = payload.get("part")
    if part in (None, ""):
        return None
    if not isinstance(part, str):
        raise ValueError('part must be a string like "60x40x30"')
    tol = payload.get("tol", DEFAULT_TOL_PCT)
    if isinstance(tol, bool) or not isinstance(tol, (int, float)):
        raise ValueError("tol must be a number (percent)")
    return _exact(part, tol)
