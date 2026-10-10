"""Shared terrain lifecycle report formatting for asset CLIs."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def format_terrain_lifecycle_report(
    lifecycle: Any,
    *,
    include_paths: bool,
    issue_remediation_label: str,
) -> tuple[str, ...]:
    """Return stable human-readable lifecycle lines without printing."""
    availability = lifecycle.availability
    lines: list[str] = [f"status: {availability.status.value}"]
    if include_paths:
        lines.append(f"raw_dem: {_path_state(availability.raw_dem_path)}")
        lines.append(
            f"expected_baked_usd: {_path_state(availability.expected_baked_path)}"
        )
        if availability.baked_path is not None:
            lines.append(f"baked_usd: {availability.baked_path}")
    if availability.issue is not None:
        lines.append(f"issue: {availability.issue.kind.value}")
        lines.append(f"detail: {availability.issue.summary}")
        lines.append(f"{issue_remediation_label}: {availability.issue.remediation}")
    for note in getattr(availability, "notes", ()):
        lines.append(f"note: {note}")
    if lifecycle.next_step is not None:
        lines.append(f"next: {lifecycle.next_step}")
    return tuple(lines)


def _path_state(path: Path | None) -> str:
    if path is None:
        return "None"
    state = "exists" if path.exists() else "missing"
    return f"{path} ({state})"
