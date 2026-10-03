"""Export TensorBoard scalars to one ``metrics.jsonl`` per event directory.

Each scalar sample becomes a JSON-lines record::

    {"step": 1024, "wall_time": 1758000000.5, "tag": "rollout/ep_rew_mean", "value": 3.2}

Existing output is kept when no newer event file exists.
"""

from __future__ import annotations

import json
import math
import sys
from collections.abc import Iterator
from pathlib import Path

__all__ = ["METRICS_FILENAME", "export_tensorboard_scalars", "main"]

METRICS_FILENAME = "metrics.jsonl"
_EVENT_PREFIX = "events.out.tfevents"


def _event_dirs(root: Path) -> Iterator[Path]:
    seen: set[Path] = set()
    for event_file in sorted(root.rglob(f"{_EVENT_PREFIX}*")):
        parent = event_file.parent
        if parent not in seen:
            seen.add(parent)
            yield parent


def _needs_export(directory: Path) -> bool:
    target = directory / METRICS_FILENAME
    if not target.is_file():
        return True
    newest = max(p.stat().st_mtime for p in directory.glob(f"{_EVENT_PREFIX}*"))
    return newest > target.stat().st_mtime


def _export_dir(directory: Path) -> int:
    from tensorboard.backend.event_processing.event_accumulator import (
        EventAccumulator,
    )

    accumulator = EventAccumulator(str(directory), size_guidance={"scalars": 0})
    accumulator.Reload()
    records: list[tuple[int, float | None, str, float | None]] = []
    for tag in accumulator.Tags().get("scalars", ()):
        for event in accumulator.Scalars(tag):
            value = float(event.value)
            records.append(
                (
                    int(event.step),
                    float(event.wall_time) if math.isfinite(event.wall_time) else None,
                    tag,
                    value if math.isfinite(value) else None,
                )
            )
    records.sort(key=lambda r: (r[0], r[2], float("inf") if r[1] is None else r[1]))
    target = directory / METRICS_FILENAME
    tmp = target.with_name(f".{METRICS_FILENAME}.tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        for step, wall_time, tag, value in records:
            handle.write(
                json.dumps(
                    {"step": step, "wall_time": wall_time, "tag": tag, "value": value},
                    separators=(",", ":"),
                )
                + "\n"
            )
    tmp.replace(target)
    return len(records)


def export_tensorboard_scalars(root: Path | str) -> dict[Path, int]:
    """Write ``metrics.jsonl`` for every event-file directory under ``root``.

    Returns the record count per exported directory; directories whose
    ``metrics.jsonl`` is already newer than their event files are skipped.
    """
    root = Path(root)
    exported: dict[Path, int] = {}
    if not root.is_dir():
        return exported
    for directory in _event_dirs(root):
        if _needs_export(directory):
            exported[directory] = _export_dir(directory)
    return exported


def main(argv: list[str] | None = None) -> int:
    """``python -m srb.utils.metrics_jsonl <dir>...``: export and report."""
    args = sys.argv[1:] if argv is None else argv
    if not args:
        print(
            "usage: python -m srb.utils.metrics_jsonl <run-dir> [<run-dir> ...]",
            file=sys.stderr,
        )
        return 2
    for arg in args:
        for directory, count in export_tensorboard_scalars(arg).items():
            print(f"{directory / METRICS_FILENAME}: {count} record(s)")
    return 0


if __name__ == "__main__":  # pragma: no cover - thin CLI shim
    raise SystemExit(main())
