"""Append-only session index for drift tracking."""

from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

from srb.interfaces.sim_to_real.validation.artifact_reference import INDEX_JSONL
from srb.interfaces.sim_to_real.validation.numeric import (
    NonFiniteValidationError,
    ensure_finite,
)
from srb.interfaces.sim_to_real.validation.session_workspace import (
    ValidationSessionIdentity,
)
from srb.utils.failures import IntegrityFailureError
from srb.utils.publication import (
    ArtifactPublication,
    PublicationCleanupError,
    PublicationError,
    PublicationTimeoutError,
)

_DRIFT_CONTENT_ERRORS = (
    json.JSONDecodeError,
    UnicodeDecodeError,
    NonFiniteValidationError,
)


class DriftStore:
    """Append-only ``<storage_root>/_index.jsonl`` store.

    Readers fail closed and writers refuse corrupt history. A diagnostic
    snapshot may be preserved, but the primary index is never repaired.
    """

    INDEX_NAME = INDEX_JSONL

    def __init__(self, storage_root: str | Path) -> None:
        self.root = Path(storage_root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / self.INDEX_NAME

    def record(
        self,
        *,
        git_sha: str,
        timestamp: str,
        task: str,
        algo: str,
        session_dir: str,
        metrics: dict[str, float],
        exit_code: int,
        identity: ValidationSessionIdentity | None = None,
    ) -> None:
        row = {
            "git_sha": git_sha,
            "timestamp": timestamp,
            "task": task,
            "algo": algo,
            "session_dir": session_dir,
            "metrics": dict(metrics),
            "exit_code": exit_code,
        }
        if identity is not None:
            row.update(identity.to_dict())
        ensure_finite(row, context="validation drift record")
        row_payload = (json.dumps(row, allow_nan=False) + "\n").encode("utf-8")
        publication = ArtifactPublication.file(self.path)
        current_payload: bytes | None = None
        try:
            # Reject corruption before stage creation can recover a companion
            # identity transaction. The locked read closes the publication race.
            self._read_snapshot(publication)
            # Lock validation and replacement to prevent lost appends. Discard
            # the stage if the primary is corrupt.
            with publication.stage() as stage:
                current_payload, _rows = self._read_snapshot(publication)
                stage.path.write_bytes((current_payload or b"") + row_payload)
                stage.publish()
        except IntegrityFailureError:
            raise
        except PublicationTimeoutError:
            raise
        except PublicationCleanupError:
            # A committed destination remains valid; do not call it corrupt or
            # back up bytes that were successfully committed.
            raise
        except _DRIFT_CONTENT_ERRORS + (PublicationError,) as exc:
            # Propagate operational writer errors after discarding the stage;
            # publication validation errors indicate bad evidence identity.
            if current_payload is not None:
                self._preserve_backup(current_payload)
            raise self._integrity_failure(exc) from exc

    def lookup(self, task: str, algo: str, *, last_k: int) -> list[dict[str, Any]]:
        try:
            self.path.lstat()
        except FileNotFoundError:
            return []
        except OSError as exc:
            raise self._integrity_failure(exc) from exc
        try:
            rows = self._read_all()
        except FileNotFoundError as exc:
            # The index existed at lstat; a missing snapshot is an evidence
            # race, not empty history.
            raise self._integrity_failure(exc) from exc
        matched = [r for r in rows if r.get("task") == task and r.get("algo") == algo]
        return matched[-last_k:]

    def _read_all(self) -> list[dict[str, Any]]:
        payload, rows = self._read_snapshot(ArtifactPublication.file(self.path))
        if payload is None:
            raise FileNotFoundError(f"validation drift index disappeared: {self.path}")
        return rows

    def _read_snapshot(
        self, publication: ArtifactPublication
    ) -> tuple[bytes | None, list[dict[str, Any]]]:
        raw_payload: bytes | None = None

        def parse(payload: bytes) -> list[dict[str, Any]]:
            nonlocal raw_payload
            raw_payload = payload
            return self._decode_payload(payload)

        try:
            rows = publication.read(parse)
        except PublicationTimeoutError:
            raise
        except _DRIFT_CONTENT_ERRORS + (PublicationError, OSError) as exc:
            # Fail closed on unreadable indexes; only an index missing at
            # lookup entry means empty history.
            if raw_payload is not None:
                self._preserve_backup(raw_payload)
            raise self._integrity_failure(exc) from exc
        if rows is None:
            return None, []
        assert raw_payload is not None
        return raw_payload, rows

    def _preserve_backup(self, payload: bytes) -> None:
        """Best-effort immutable preservation that cannot touch the primary."""

        backup = self.path.with_name(f"{self.path.name}.bak.{time.time_ns()}")
        expected_sha256 = hashlib.sha256(payload).hexdigest()
        try:
            ArtifactPublication.file(backup).publish(
                lambda staging: staging.write_bytes(payload),
                expected_sha256=expected_sha256,
                immutable_identity=True,
            )
        except Exception:
            # Backup is optional; preserve typed integrity failures and never
            # repair the primary.
            return

    def _integrity_failure(self, exc: Exception) -> IntegrityFailureError:
        return IntegrityFailureError(
            code="validation.drift_history.integrity",
            message=f"validation drift history is corrupt: {exc}",
            component="validation.drift",
            details={
                "path": str(self.path),
                "cause": type(exc).__name__,
            },
            fallback_allowed=False,
            evidence_valid=False,
        )

    def _decode_payload(self, payload: bytes) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for line in payload.decode("utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if not isinstance(row, dict):
                raise json.JSONDecodeError(
                    "drift index rows must be JSON objects", line, 0
                )
            ensure_finite(row, context=f"validation drift record {self.path}")
            rows.append(row)
        return rows


def z_score(
    current: dict[str, float],
    history: list[dict[str, Any]],
    metrics: list[str],
) -> dict[str, float]:
    """Per-metric z-scores for metrics shared by current and all history rows.

    Returns ``{}`` with fewer than two historical samples. For zero standard
    deviation, returns 0.0 on a match or the largest finite float otherwise.
    """
    ensure_finite(current, context="current validation metrics")
    ensure_finite(history, context="validation drift history")
    if len(history) < 2:
        return {}
    out: dict[str, float] = {}
    for key in metrics:
        if key not in current:
            continue
        vals = [h.get("metrics", {}).get(key) for h in history]
        if any(v is None for v in vals):
            continue
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        std = math.sqrt(var)
        delta = current[key] - mean
        if std == 0.0:
            out[key] = (
                0.0
                if delta == 0.0
                else math.copysign(float.fromhex("0x1.fffffffffffffp+1023"), delta)
            )
        else:
            out[key] = delta / std
    return out
