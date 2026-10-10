"""``srb asset prefetch-fixtures`` -- pre-bake every manifest site at lod=low."""

from __future__ import annotations

from typing import cast

from srb.cli.commands._assets._common import run_asset_operation


def run(**_unused) -> int:
    from srb.terrain.api import bake_or_load
    from srb.terrain.manifest_loader import load_manifest
    from srb.terrain.spec import Body, TerrainSpec

    sites = load_manifest().sites
    return run_asset_operation(
        command="prefetch-fixtures",
        kind="terrain",
        target=None,
        all=True,
        items=sites,
        action=lambda site_name: bake_or_load(
            TerrainSpec(
                body=cast(Body, sites[site_name].body),
                site=site_name,
                patch="default",
                lod="low",
            ),
            auto_fetch=True,
        ),
        format_success=lambda _name, path, _all: f"ok ({path})",
    )
