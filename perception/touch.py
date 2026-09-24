"""Probe touches — points the robot's tool tip is placed on, recorded in the base frame.

The one-minute calibration (`docs/mono-scan.md` §3) is three touches on the
table (→ :class:`~perception.tableplane.Plane`) and three on the plate's
dimples A/B/C (→ ``T_base_plate``, :func:`perception.charuco.plate_from_touches`).
Each touch reads the flange pose (``ur_flange_pose``) and applies the probe's
TCP (``[0, 0, 0.050]`` seated on the bracket — ``hardware/charuco-board/out/board.json``).
Saved to ``captures/calibration/touch_<cell>.json``. Pure stdlib.
"""

from __future__ import annotations

import json
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from urctl.pose import Transform, pose_trans

from .tableplane import Plane

SETS = ("table", "plate")
PLATE_LABELS = ("A", "B", "C")
TABLE_LABELS = ("1", "2", "3")
DEFAULT_PROBE_TCP = [0.0, 0.0, 0.050, 0.0, 0.0, 0.0]
ENV_TOUCH_FILE = "PERCEPTION_TOUCH_FILE"


def default_touch_path(env: Mapping[str, str] | None = None) -> str:
    env = os.environ if env is None else env
    explicit = env.get(ENV_TOUCH_FILE, "").strip()
    if explicit:
        return explicit
    cell = env.get("UR_CELL", "").strip()
    return os.path.join("captures/calibration", f"touch_{cell}.json" if cell else "touch.json")


@dataclass
class TouchSet:
    probe_tcp: list[float] = field(default_factory=lambda: list(DEFAULT_PROBE_TCP))
    table: dict[str, list[float]] = field(default_factory=dict)
    plate: dict[str, list[float]] = field(default_factory=dict)

    def record(self, which: str, label: str, flange_pose: Sequence[float]) -> list[float]:
        if which not in SETS:
            raise ValueError(f"set must be one of {SETS}")
        labels = PLATE_LABELS if which == "plate" else TABLE_LABELS
        label = label.upper()
        if label not in labels:
            raise ValueError(f"{which} labels are {labels}")
        tip = pose_trans([float(v) for v in flange_pose], self.probe_tcp)[:3]
        getattr(self, which)[label] = tip
        return tip

    def table_plane(self) -> Plane | None:
        if all(k in self.table for k in TABLE_LABELS):
            return Plane.from_points(*(self.table[k] for k in TABLE_LABELS))
        return None

    def plate_transform(self) -> Transform | None:
        if all(k in self.plate for k in PLATE_LABELS):
            from .charuco import plate_from_touches

            return plate_from_touches(*(self.plate[k] for k in PLATE_LABELS))
        return None

    def as_dict(self) -> dict:
        plane = self.table_plane()
        plate = self.plate_transform()
        return {
            "probe_tcp": self.probe_tcp,
            "table": self.table,
            "plate": self.plate,
            "table_plane": None if plane is None else plane.as_dict(),
            "plate_pose": None if plate is None else plate.to_pose(),
            "missing": {
                "table": [k for k in TABLE_LABELS if k not in self.table],
                "plate": [k for k in PLATE_LABELS if k not in self.plate],
            },
        }

    def save(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.as_dict(), indent=2))
        return p

    @classmethod
    def load(cls, path: str | Path) -> TouchSet:
        d = json.loads(Path(path).read_text())
        return cls(
            list(d.get("probe_tcp", DEFAULT_PROBE_TCP)), dict(d.get("table", {})), dict(d.get("plate", {}))
        )

    @classmethod
    def load_or_new(cls, path: str | Path) -> TouchSet:
        return cls.load(path) if Path(path).is_file() else cls()
