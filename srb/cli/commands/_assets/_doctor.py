"""``srb asset doctor <kind> [target]`` preflight diagnostics."""

from __future__ import annotations

import sys
from typing import Optional, cast

from srb.cli.commands._assets._catalog import find_asset_kind
from srb.cli.commands._assets._common import filter_by_body, parse_terrain_center
from srb.cli.commands._assets._terrain_report import (
    format_terrain_lifecycle_report,
)
from srb.terrain.spec import Lod


def _doctor_terrain_site(
    site: str,
    *,
    lod: str,
    patch: str,
    patch_size: float | None,
    center: str | None,
    no_ortho: bool,
    seed: int,
    deshade: bool = False,
) -> int:
    from srb.terrain.lifecycle import TerrainAssetAction, assess_terrain_asset
    from srb.terrain.manifest_loader import get_site

    entry = get_site(site)
    lifecycle = assess_terrain_asset(
        site,
        patch=patch,
        lod=cast(Lod, lod),
        seed=seed,
        patch_size_m=patch_size,
        patch_center_xy_m=parse_terrain_center(center),
        use_ortho=not no_ortho,
        deshade=deshade,
    )

    print(f"site: {site}")
    print(f"  body: {entry.body}")
    for line in format_terrain_lifecycle_report(
        lifecycle,
        include_paths=True,
        issue_remediation_label="hint",
    ):
        print(f"  {line}")
    if lifecycle.action is not TerrainAssetAction.READY:
        return 1
    return 0


def _doctor_terrain(
    *,
    target: Optional[str],
    body: Optional[str],
    lod: str = "medium",
    patch: str = "default",
    patch_size: float | None = None,
    center: str | None = None,
    no_ortho: bool = False,
    seed: int = 0,
    deshade: bool = False,
) -> int:
    from srb.terrain.errors import ManifestSiteUnknownError
    from srb.terrain.manifest_loader import load_manifest

    sites = load_manifest().sites
    if target is not None:
        if target not in sites:
            raise ManifestSiteUnknownError(target)
        if target not in filter_by_body(sites, body):
            print(
                f"doctor: site {target!r} is on {sites[target].body}, not {body}",
                file=sys.stderr,
            )
            return 1
        return _doctor_terrain_site(
            target,
            lod=lod,
            patch=patch,
            patch_size=patch_size,
            center=center,
            no_ortho=no_ortho,
            seed=seed,
            deshade=deshade,
        )

    rc = 0
    filtered_sites = filter_by_body(sites, body)
    for index, site in enumerate(sorted(filtered_sites)):
        if index:
            print()
        rc = max(
            rc,
            _doctor_terrain_site(
                site,
                lod=lod,
                patch=patch,
                patch_size=patch_size,
                center=center,
                no_ortho=no_ortho,
                seed=seed,
                deshade=deshade,
            ),
        )
    return rc


def run(*, kind: str, **kwargs) -> int:
    asset_kind = find_asset_kind(kind)
    if asset_kind is not None and asset_kind.supports_doctor:
        return _DOCTOR_HANDLERS[kind](**kwargs)
    print(f"doctor: unsupported asset kind: {kind}; use 'terrain'", file=sys.stderr)
    return 2


_DOCTOR_HANDLERS = {
    "terrain": _doctor_terrain,
}
