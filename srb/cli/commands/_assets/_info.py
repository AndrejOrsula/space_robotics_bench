"""``srb asset info <kind> <target>``."""

from __future__ import annotations

import sys
from typing import Any

from srb.cli.commands._assets._catalog import get_asset_kind
from srb.cli.commands._assets._terrain_report import (
    format_terrain_lifecycle_report,
)


def _print_pydantic(obj: Any, indent: int = 0) -> None:
    """Pretty-print a pydantic model as `field: value`, recursing into nested models."""
    pad = "  " * indent
    for name in obj.model_fields:
        val = getattr(obj, name)
        if hasattr(val, "model_fields"):
            print(f"{pad}{name}:")
            _print_pydantic(val, indent + 1)
        elif (
            isinstance(val, dict)
            and val
            and hasattr(next(iter(val.values())), "model_fields")
        ):
            print(f"{pad}{name}:")
            for k, v in val.items():
                print(f"{pad}  {k}:")
                _print_pydantic(v, indent + 2)
        else:
            print(f"{pad}{name}: {val}")


def run(*, kind: str, target: str, **_unused) -> int:
    asset_kind = get_asset_kind(kind)
    if not asset_kind.supports_info:
        raise ValueError(f"info: unsupported asset kind: {kind!r}")
    return _INFO_HANDLERS[kind](target)


def _info_terrain(target: str) -> int:
    from srb.terrain.lifecycle import assess_terrain_asset
    from srb.terrain.manifest_loader import load_manifest

    sites = load_manifest().sites
    if target not in sites:
        _print_unknown_target("terrain", target)
        return 1
    _print_pydantic(sites[target])
    lifecycle = assess_terrain_asset(target)
    print("availability:")
    for line in format_terrain_lifecycle_report(
        lifecycle,
        include_paths=False,
        issue_remediation_label="remediation",
    ):
        print(f"  {line}")
    return 0


def _info_rocks(target: str) -> int:
    from srb.terrain.rocks.catalog_loader import load_rocks_catalog

    sets = load_rocks_catalog().sets
    if target not in sets:
        _print_unknown_target("rocks", target)
        return 1
    _print_pydantic(sets[target])
    return 0


def _info_hdri(target: str) -> int:
    from srb.terrain.hdri.catalog_loader import load_hdri_catalog

    domes = load_hdri_catalog().skydomes
    if target not in domes:
        _print_unknown_target("hdri", target)
        return 1
    _print_pydantic(domes[target])
    return 0


def _info_lights(target: str) -> int:
    from srb.terrain.lights.catalog_loader import load_lighting_catalog

    presets = load_lighting_catalog().presets
    if target not in presets:
        _print_unknown_target("lights", target)
        return 1
    _print_pydantic(presets[target])
    return 0


def _print_unknown_target(kind: str, target: str) -> None:
    asset_kind = get_asset_kind(kind)
    print(
        f"unknown {asset_kind.target_label}: {target!r}; "
        f"run `srb asset list {kind}` to see available targets",
        file=sys.stderr,
    )


_INFO_HANDLERS = {
    "terrain": _info_terrain,
    "rocks": _info_rocks,
    "hdri": _info_hdri,
    "lights": _info_lights,
}
