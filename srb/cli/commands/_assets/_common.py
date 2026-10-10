"""Shared CLI helpers for the ``srb asset`` subcommands."""

from __future__ import annotations

import sys
from collections.abc import Callable, Mapping
from typing import Optional, TypeVar

from srb.cli.commands._assets._catalog import get_asset_kind
from srb.terrain.errors import TerrainError

T = TypeVar("T")
R = TypeVar("R")


def print_error(error: TerrainError, *, debug: bool = False) -> None:
    """Print a typed terrain error with its remediation hint to stderr."""
    name = type(error).__name__
    print(f"error: {name}: {error}", file=sys.stderr)
    remediation = getattr(error, "remediation", None)
    if remediation:
        print(f"  hint: {remediation}", file=sys.stderr)
    if debug:
        import traceback

        traceback.print_exc(file=sys.stderr)


def parse_terrain_center(center: Optional[str]) -> Optional[tuple[float, float]]:
    """Parse a terrain patch center from the shared ``--center x,y`` option."""
    if center is None:
        return None
    parts = center.split(",")
    if len(parts) != 2:
        raise SystemExit(f"--center must be x,y in meters, got {center!r}")
    try:
        return float(parts[0]), float(parts[1])
    except ValueError as exc:
        raise SystemExit(f"--center must be x,y in meters, got {center!r}") from exc


def filter_by_body(items: dict, body: Optional[str], body_attr: str = "body") -> dict:
    """Return items whose ``body_attr`` matches ``body`` (or all if None)."""
    if body is None:
        return items
    return {k: v for k, v in items.items() if getattr(v, body_attr, None) == body}


def print_item_error(label: str, error: BaseException, *, debug: bool = False) -> None:
    """Print one named asset operation failure (with its traceback on ``debug``)."""
    print(f"{label}: ERROR: {error}", file=sys.stderr)
    if isinstance(error, TerrainError) and error.remediation:
        print(f"{label}: hint: {error.remediation}", file=sys.stderr)
    if debug:
        import traceback

        traceback.print_exception(error, file=sys.stderr)


def run_asset_operation(
    *,
    command: str,
    kind: str,
    target: Optional[str],
    all: bool,
    items: Mapping[str, T],
    action: Callable[[str], R],
    format_success: Callable[[str, R, bool], str],
    target_label: str | None = None,
    debug: bool = False,
) -> int:
    """Run a shared all-or-one named asset operation loop."""
    if all:
        rc = 0
        for name in items:
            try:
                result = action(name)
                print(f"{name}: {format_success(name, result, True)}")
            except Exception as e:
                print_item_error(name, e, debug=debug)
                rc = 1
        return rc

    if not target:
        label = target_label or get_asset_kind(kind).target_label
        print(f"{command} {kind}: must specify <{label}> or --all", file=sys.stderr)
        raise SystemExit(2)

    try:
        result = action(target)
        print(f"{target}: {format_success(target, result, False)}")
        return 0
    except Exception as e:
        print_item_error(target, e, debug=debug)
        return 1
