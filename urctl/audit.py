"""AuditLog — a structured, append-only record of every action taken.

ROSClaw-style executives expect "structured audit logging" and "provenance":
when an autonomous agent drives a robot you want a replayable record of what
was commanded, against which target, whether the safety envelope passed, and
what came back.

Each action is one :class:`AuditRecord`, kept in memory and (optionally)
appended as a JSON line to a file. JSON-lines is deliberate: it's trivially
greppable, streamable, and ingestible by log pipelines.

By default the log path comes from ``UR_AUDIT_LOG``; if unset, records are
kept in memory only. Construct explicitly to force a path::

    audit = AuditLog(path="/var/log/urctl-audit.jsonl")
"""

from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import asdict, dataclass, field


@dataclass
class AuditRecord:
    action: str
    args: dict
    host: str
    ok: bool
    ts: float  # epoch seconds
    result: dict = field(default_factory=dict)
    safety: dict | None = None  # SafetyVerdict.as_dict() when applicable
    dry_run: bool = False

    def to_json(self) -> str:
        return json.dumps(asdict(self), default=str, sort_keys=True)


class AuditLog:
    def __init__(self, path: str | None = None, *, capture: bool = True):
        """:param path: JSON-lines file to append to. Falls back to
        ``UR_AUDIT_LOG``; ``None`` means in-memory only.
        :param capture: keep records in ``self.records`` for later inspection.
        """
        self.path = path if path is not None else os.environ.get("UR_AUDIT_LOG")
        self.capture = capture
        self.records: list[AuditRecord] = []
        self._lock = threading.Lock()

    def record(
        self,
        action: str,
        *,
        host: str,
        args: dict,
        ok: bool,
        result: dict | None = None,
        safety: dict | None = None,
        dry_run: bool = False,
    ) -> AuditRecord:
        rec = AuditRecord(
            action=action,
            args=args,
            host=host,
            ok=ok,
            ts=time.time(),
            result=result or {},
            safety=safety,
            dry_run=dry_run,
        )
        with self._lock:
            if self.capture:
                self.records.append(rec)
            if self.path:
                with open(self.path, "a", encoding="utf-8") as fh:
                    fh.write(rec.to_json() + "\n")
        return rec
