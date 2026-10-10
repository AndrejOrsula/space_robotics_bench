"""``srb asset bake terrain --dem <path>``."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import cast

from srb.cli.commands._assets._catalog import find_asset_kind


def run(
    *,
    kind: str,
    dem: str,
    body: str,
    site_name: str,
    patch_size: float | None,
    lod: str,
    attribution: str,
    seed: int = 0,
    debug: bool = False,
    **_unused,
) -> int:
    asset_kind = find_asset_kind(kind)
    if asset_kind is None or not asset_kind.supports_bake:
        print(f"bake: unsupported kind: {kind!r}; use 'terrain'", file=sys.stderr)
        return 2

    dem_path = Path(dem)
    if not dem_path.is_file():
        print(f"bake: DEM not found: {dem_path}", file=sys.stderr)
        return 1

    from srb.terrain.api import bake_or_load
    from srb.terrain.spec import Body, Lod, TerrainSpec

    spec = TerrainSpec(
        body=cast(Body, body),
        dem_path=dem_path,
        site_name=site_name,
        attribution=attribution,
        patch_size_m=patch_size,
        lod=cast(Lod, lod),
        seed=seed,
    )
    try:
        baked = bake_or_load(spec, auto_fetch=True)
    except Exception as e:
        print(f"bake: ERROR: {e}", file=sys.stderr)
        if debug:
            import traceback

            traceback.print_exc(file=sys.stderr)
        return 1

    print(f"{site_name}: {baked}")
    return 0
