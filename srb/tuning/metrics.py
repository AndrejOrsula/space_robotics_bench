"""Read trial metrics JSONL and derive progress, rungs, and objective.

The reporter appends records on each flush. The driver tails them to compute
clamped progress, integer pruning rungs, and the chosen field's tail-mean.
Pure stdlib; no Optuna, torch, or Isaac.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class MetricRecord:
    env_steps: int
    episodes_done: int
    ep_return_mean: float
    ep_return_std: float
    ep_len_mean: float
    success_rate: float | None = None

    @classmethod
    def from_dict(cls, d: dict) -> "MetricRecord":
        return cls(
            env_steps=int(d["env_steps"]),
            episodes_done=int(d.get("episodes_done", 0)),
            ep_return_mean=float(d["ep_return_mean"]),
            ep_return_std=float(d.get("ep_return_std", 0.0)),
            ep_len_mean=float(d.get("ep_len_mean", 0.0)),
            success_rate=(
                None if d.get("success_rate") is None else float(d["success_rate"])
            ),
        )


def read_metrics(path: str | Path) -> list[MetricRecord]:
    """Parse JSONL metrics, skipping malformed or incomplete trailing lines."""
    p = Path(path)
    if not p.is_file():
        return []
    records: list[MetricRecord] = []
    text = p.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    for line in lines:
        if not line.endswith("\n"):
            # Final partial line — writer may still be flushing it.
            break
        stripped = line.strip()
        if not stripped:
            continue
        try:
            d = json.loads(stripped)
            records.append(MetricRecord.from_dict(d))
        except (json.JSONDecodeError, KeyError, ValueError, TypeError, OverflowError):
            continue
    return records


class MetricsTailer:
    """Incrementally parse appended complete lines during polling.

    Retains a byte offset, records, divergence flag, and incomplete tail.
    """

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._offset = 0
        self._tail = b""
        self.records: list[MetricRecord] = []
        self.diverged = False

    def poll(self) -> list[MetricRecord]:
        """Consume appended bytes; return ALL records read so far."""
        if not self._path.is_file():
            return self.records
        with open(self._path, "rb") as f:
            f.seek(self._offset)
            chunk = f.read()
            self._offset = f.tell()
        if not chunk:
            return self.records
        lines = (self._tail + chunk).split(b"\n")
        self._tail = lines.pop()  # b"" when the chunk ended on a newline
        for raw in lines:
            stripped = raw.strip()
            if not stripped:
                continue
            try:
                record = MetricRecord.from_dict(json.loads(stripped.decode("utf-8")))
            except (
                json.JSONDecodeError,
                UnicodeDecodeError,
                KeyError,
                ValueError,
                TypeError,
                OverflowError,
            ):
                continue
            self.records.append(record)
            if not math.isfinite(record.ep_return_mean):
                self.diverged = True
        return self.records


def progress(env_steps: int, target_env_steps: int) -> float:
    """Training progress in ``[0, 1]``. A non-positive target reads as complete."""
    if target_env_steps <= 0:
        return 1.0
    return max(0.0, min(1.0, env_steps / target_env_steps))


def rung_for_progress(p: float, n_rungs: int = 20) -> int:
    """Quantize progress to an integer rung in ``[0, n_rungs]``."""
    return max(0, min(n_rungs, int(math.floor(p * n_rungs))))


def _field(record: MetricRecord, field: str) -> float:
    value = getattr(record, field)
    return float("nan") if value is None else float(value)


def smoothed_return(
    records: list[MetricRecord], *, window: int = 5, field: str = "ep_return_mean"
) -> float:
    """Mean of *field* over the last ``window`` records (running smoothing)."""
    if not records:
        return float("nan")
    tail = records[-window:]
    vals = [_field(r, field) for r in tail]
    return sum(vals) / len(vals)


def tail_mean_objective(
    records: list[MetricRecord],
    target: int,
    *,
    tail: float = 0.75,
    field: str = "ep_return_mean",
) -> float:
    """Episode-weighted mean *field* where ``progress ≥ tail``; ``nan`` if invalid/empty."""
    vals: list[float] = []
    weights: list[int] = []
    previous_episodes = 0
    for r in records:
        # Each flush averages the episodes completed since the previous one.
        episodes = max(r.episodes_done - previous_episodes, 0)
        previous_episodes = r.episodes_done
        if progress(r.env_steps, target) >= tail:
            vals.append(_field(r, field))
            weights.append(episodes)
    if not vals or any(not math.isfinite(v) for v in vals):
        return float("nan")
    if sum(weights) == 0:  # Records without episode counts weigh equally.
        weights = [1] * len(vals)
    return sum(v * w for v, w in zip(vals, weights)) / sum(weights)


def is_diverged(records: list[MetricRecord]) -> bool:
    """True if any record's return is non-finite (NaN/Inf)."""
    return any(not math.isfinite(r.ep_return_mean) for r in records)


__all__ = (
    "MetricRecord",
    "MetricsTailer",
    "read_metrics",
    "progress",
    "rung_for_progress",
    "smoothed_return",
    "tail_mean_objective",
    "is_diverged",
)
