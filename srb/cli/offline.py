"""Read-only helpers for CLI views backed by generated cache snapshots."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import cast

from srb.utils.failures import IntegrityFailureError
from srb.utils.path import (
    SRB_ACTION_MODES_CACHE_PATH,
    SRB_AGENT_CONFIG_KEYS_CACHE_PATH,
    SRB_AGENT_ENUM_VALUES_CACHE_PATH,
    SRB_CONFIG_KEYS_CACHE_PATH,
    SRB_ENUM_VALUES_CACHE_PATH,
    SRB_ENV_CACHE_PATH,
    SRB_GUI_CATALOG_CACHE_PATH,
    SRB_HARDWARE_INTERFACE_CACHE_PATH,
    SRB_OBJECT_CACHE_PATH,
    SRB_ROBOT_CACHE_PATH,
    SRB_SCENERY_CACHE_PATH,
    SRB_TASK_DEFAULTS_CACHE_PATH,
)
from srb.utils.publication import (
    ArtifactPublication,
    PublicationError,
    PublicationNotFoundError,
)

_CACHE_PATHS: Mapping[str, Path] = {
    "env": SRB_ENV_CACHE_PATH,
    "robot": SRB_ROBOT_CACHE_PATH,
    "scenery": SRB_SCENERY_CACHE_PATH,
    "object": SRB_OBJECT_CACHE_PATH,
    "hardware": SRB_HARDWARE_INTERFACE_CACHE_PATH,
    "config_keys": SRB_CONFIG_KEYS_CACHE_PATH,
    "enum_values": SRB_ENUM_VALUES_CACHE_PATH,
    "agent_config_keys": SRB_AGENT_CONFIG_KEYS_CACHE_PATH,
    "agent_enum_values": SRB_AGENT_ENUM_VALUES_CACHE_PATH,
    "action_modes": SRB_ACTION_MODES_CACHE_PATH,
    "task_defaults": SRB_TASK_DEFAULTS_CACHE_PATH,
    "gui_catalog": SRB_GUI_CATALOG_CACHE_PATH,
}
_DEFAULT_CACHE_PATHS = dict(_CACHE_PATHS)


def read_cache_payload(name: str) -> object | None:
    """Read one member from the current immutable cache generation.

    A path not present in the canonical table is an explicit compatibility
    override for legacy callers. Canonical reads never fall back to the
    mutable fixed files.
    """
    path = _CACHE_PATHS.get(name)
    if path is None:
        return None
    try:
        if path == _DEFAULT_CACHE_PATHS.get(name):
            from srb.utils.cache import open_offline_srb_cache_snapshot

            snapshot = open_offline_srb_cache_snapshot()
            raw = snapshot.read(name, lambda payload: payload)
        else:

            def parse_json(payload: bytes) -> object:
                value: object = json.loads(payload)
                if value is None:
                    raise ValueError("offline cache payload must not be null")
                return value

            raw = ArtifactPublication.file(path).read(parse_json)
    except PublicationNotFoundError:
        # Missing cache is an empty view; corrupt publication state fails below.
        return None
    except (
        OSError,
        UnicodeError,
        PublicationError,
        json.JSONDecodeError,
        TypeError,
        ValueError,
    ) as exc:
        raise IntegrityFailureError(
            code="cli.offline_cache.integrity",
            message=(
                f"Offline cache {name!r} at {path} failed publication validation: {exc}"
            ),
            component="srb.cli.offline",
            details={
                "cache": name,
                "path": str(path),
                "cause": str(exc),
                "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
            },
        ) from exc
    if raw is None:
        return None
    if isinstance(raw, dict) and "payload" in raw and "__srb_cache__" in raw:
        return cast(object, raw["payload"])
    return raw


def warn_if_offline_cache_incomplete() -> None:
    """Report absent, incomplete, or stale generated cache state to CLI views."""
    from srb.utils.cache import warn_if_offline_srb_cache_incomplete

    warn_if_offline_srb_cache_incomplete()


def flatten_cache_names(value: object) -> tuple[str, ...]:
    """Return deterministic leaf strings from a JSON cache payload."""
    result: list[str] = []

    def visit(item: object) -> None:
        if isinstance(item, str):
            result.append(item)
        elif isinstance(item, Mapping):
            for child in item.values():
                visit(child)
        elif isinstance(item, Sequence) and not isinstance(item, (str, bytes)):
            for child in item:
                visit(child)

    visit(value)
    return tuple(sorted(dict.fromkeys(result)))


__all__ = ["flatten_cache_names", "read_cache_payload"]
