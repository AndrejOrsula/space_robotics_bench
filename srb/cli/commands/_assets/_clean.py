"""``srb asset clean <kind> [<target>] [--raw|--baked|--all]``."""

from __future__ import annotations

import sys
from typing import Optional

from srb.cli.commands._assets._catalog import find_asset_kind


def _clean_terrain(target: Optional[str], raw: bool, baked: bool, all_: bool) -> int:
    from srb.terrain.cache import clean_terrain_cache

    if not (raw or baked or all_):
        print("clean terrain: pass --raw, --baked, or --all", file=sys.stderr)
        return 2
    if target is not None and (raw or all_):
        print(
            "clean terrain: raw archives are content-addressed and shared; "
            "omit the site target with --raw or --all",
            file=sys.stderr,
        )
        return 2

    removed = clean_terrain_cache(
        raw=raw,
        baked=baked,
        all_artifacts=all_,
        target=target,
    )

    if not removed:
        print("clean terrain: nothing to remove")
    else:
        for r in removed:
            print(f"removed: {r}")
    return 0


def run(
    *,
    kind: str,
    target: Optional[str] = None,
    raw: bool = False,
    baked: bool = False,
    all: bool = False,
    **_unused,
) -> int:
    asset_kind = find_asset_kind(kind)
    if asset_kind is not None and asset_kind.supports_clean:
        return _CLEAN_HANDLERS[kind](target, raw, baked, all)
    print(f"clean: unsupported kind: {kind!r}; use 'terrain'", file=sys.stderr)
    return 2


_CLEAN_HANDLERS = {
    "terrain": _clean_terrain,
}
