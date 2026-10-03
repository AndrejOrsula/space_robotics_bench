"""Runtime surface-source policy for procedural and real terrain assets.

This module is intentionally Isaac-free. Surface asset classes provide the
procedural builder, while asset composition supplies the real-terrain factory
only after source preflight allows it.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Callable, Literal, Sequence, TypeVar

from srb.terrain.availability import assess_terrain_site, assess_terrain_spec
from srb.terrain.catalog.resolve import RegionRequest
from srb.terrain.errors import TerrainError
from srb.terrain.log import get_logger
from srb.terrain.source_health import (
    TerrainSourceIssue,
    classify_terrain_source_exception,
    resolve_terrain_source,
    warn_terrain_fallback_once,
)
from srb.terrain.spec import Body, Lod, TerrainSpec

SurfaceSource = Literal["procedural", "real"]

_AUTO_FETCH_ENV = "SRB_TERRAIN_AUTO_FETCH"
_AssetCfgT = TypeVar("_AssetCfgT")
_TERRAIN_LOG = get_logger("runtime")


def terrain_auto_fetch_default() -> bool:
    """Honor ``SRB_TERRAIN_AUTO_FETCH=0`` to globally disable terrain fetches."""
    val = os.environ.get(_AUTO_FETCH_ENV)
    if val is None or not val.strip():
        return True
    return val.strip().lower() not in ("0", "false", "no", "off")


def validate_real_terrain_selection(
    *,
    body: str,
    lod: str,
    augment: str,
    site: str | None = None,
    dem_path: Path | None = None,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    size_m: float | None = None,
    patch_size_m: float | None = None,
    patch_center_xy_m: tuple[float, float] | None = None,
    num_locations: int = 1,
    locations: Sequence[tuple[float, float]] | None = None,
    location_spread_m: float | None = None,
) -> None:
    """Validate RealTerrain literal fields outside Isaac-bound asset modules."""
    if body not in ("moon", "mars", "earth"):
        raise ValueError(f"body must be 'moon', 'mars', or 'earth', got {body!r}")
    if lod not in ("low", "medium", "high"):
        raise ValueError(f"lod must be 'low', 'medium', or 'high', got {lod!r}")
    if augment not in ("none", "simforge", "database"):
        raise ValueError(
            f"augment must be 'none', 'simforge', or 'database', got {augment!r}"
        )

    # lat_deg/lon_deg/size_m are all-three-or-none, and mutually
    # exclusive with site/dem_path.
    region_fields = {"lat_deg": lat_deg, "lon_deg": lon_deg, "size_m": size_m}
    set_region_fields = [
        name for name, value in region_fields.items() if value is not None
    ]
    missing_region_fields = [
        name for name, value in region_fields.items() if value is None
    ]
    if locations is None and set_region_fields and missing_region_fields:
        raise ValueError(
            "RealTerrain region requests require all of lat_deg, lon_deg, "
            f"and size_m; missing {missing_region_fields}"
        )
    is_region_request = len(set_region_fields) == 3
    if is_region_request and site is not None:
        raise ValueError(
            "RealTerrain cannot combine a region request (lat_deg/lon_deg/"
            "size_m) with `site`; choose one selection mode"
        )
    if is_region_request and dem_path is not None:
        raise ValueError(
            "RealTerrain cannot combine a region request (lat_deg/lon_deg/"
            "size_m) with `dem_path`; choose one selection mode"
        )
    # patch_size_m/patch_center_xy_m are
    # silently ignored by the region bake path yet still enter the cache
    # key -- mirror TerrainSpec.__post_init__'s rejection here so RealTerrain
    # callers get a clean ValueError instead of a deep bake-time traceback.
    if is_region_request and (
        patch_size_m is not None or patch_center_xy_m is not None
    ):
        raise ValueError(
            "RealTerrain cannot combine a region request (lat_deg/lon_deg/"
            "size_m) with `patch_size_m`/`patch_center_xy_m`; the region's "
            "own `size_m` and resolved center already determine the bake "
            "footprint"
        )

    # Variety is region-form only.
    variety_requested = (
        num_locations != 1 or locations is not None or location_spread_m is not None
    )
    if variety_requested and (site is not None or dem_path is not None):
        offender = (
            "num_locations"
            if num_locations != 1
            else "locations"
            if locations is not None
            else "location_spread_m"
        )
        raise ValueError(
            f"RealTerrain `{offender}` (per-env location variety) applies "
            "only to region requests (lat_deg/lon_deg/size_m) or an explicit "
            "`locations` list; curated `site` / local `dem_path` terrains "
            "have a fixed footprint"
        )
    if locations is not None:
        if lat_deg is not None or lon_deg is not None:
            raise ValueError(
                "RealTerrain cannot combine `locations` with a lat_deg/"
                "lon_deg center; the list already provides the centers"
            )
        if size_m is None:
            raise ValueError(
                "RealTerrain `locations` requires `size_m` (shared patch "
                "footprint for every listed center)"
            )
        if patch_size_m is not None or patch_center_xy_m is not None:
            raise ValueError(
                "RealTerrain cannot combine `locations` with `patch_size_m`/"
                "`patch_center_xy_m`; each derived location's own `size_m` "
                "and resolved center already determine the bake footprint"
            )
    elif num_locations != 1 and not is_region_request:
        raise ValueError(
            "RealTerrain `num_locations` > 1 requires a region request "
            "(lat_deg/lon_deg/size_m) to derive centers from"
        )


def build_real_terrain_spec(
    *,
    body: Body,
    site: str | None,
    dem_path: Path | None,
    site_name: str | None,
    attribution: str | None,
    patch: str,
    patch_size_m: float | None,
    patch_center_xy_m: tuple[float, float] | None,
    lod: Lod,
    seed: int,
    use_ortho: bool,
    materials_version: int | None,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    size_m: float | None = None,
    deshade: bool = False,
    allow_synthetic: bool = False,
    augment_stamp: bool = False,
    augment_preset: str | None = None,
) -> TerrainSpec:
    """Build a real-terrain bake spec from runtime asset fields.

    A complete ``lat_deg``/``lon_deg``/``size_m`` triple becomes
    ``TerrainSpec.region``; selection validation rejects combining it with
    ``site`` or ``dem_path``.

    ``deshade``, ``augment_stamp``, and ``augment_preset`` pass through
    unchanged. Effective deshading is resolved after manifest lookup.
    ``allow_synthetic`` is copied to both the region request and spec so
    candidate resolution and cache keys use the same policy.
    """
    region: RegionRequest | None = None
    if lat_deg is not None and lon_deg is not None and size_m is not None:
        region = RegionRequest(
            body=body,
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            size_m=size_m,
            lod=lod,
            allow_synthetic=allow_synthetic,
        )
    return TerrainSpec(
        body=body,
        site=site,
        dem_path=dem_path,
        site_name=site_name,
        attribution=attribution,
        region=region,
        patch=patch,
        patch_size_m=patch_size_m,
        patch_center_xy_m=patch_center_xy_m,
        lod=lod,
        seed=seed,
        use_ortho=use_ortho,
        materials_version=materials_version,
        deshade=deshade,
        allow_synthetic=allow_synthetic,
        augment_stamp=augment_stamp,
        augment_preset=augment_preset,
    )


def resolve_real_terrain_baked_path(
    spec: TerrainSpec,
    *,
    auto_fetch: bool,
    preset_name: str | None,
    bake: Callable[..., Path],
    resolve_source: Callable[..., Any] = resolve_terrain_source,
) -> Path:
    """Resolve a real-terrain spec through runtime source policy."""
    return resolve_source(
        spec,
        auto_fetch=auto_fetch,
        preset_name=preset_name,
        bake=bake,
    ).require_path()


def build_real_terrain_asset_cfg(
    baked_path: Path,
    *,
    make_asset_cfg: Callable[..., _AssetCfgT],
    make_usd_spawn: Callable[..., Any],
    make_collision_props: Callable[..., Any],
) -> _AssetCfgT:
    """Build the primary Isaac asset cfg for a baked real-terrain USD."""
    return make_asset_cfg(
        prim_path="{ENV_REGEX_NS}/real_terrain",
        spawn=make_usd_spawn(
            usd_path=str(baked_path),
            collision_props=make_collision_props(),
        ),
    )


def build_real_terrain_runtime_asset(
    *,
    body: Body,
    site: str | None,
    dem_path: Path | None,
    site_name: str | None,
    attribution: str | None,
    patch: str,
    patch_size_m: float | None,
    patch_center_xy_m: tuple[float, float] | None,
    lod: Lod,
    seed: int,
    use_ortho: bool,
    materials_version: int | None,
    auto_fetch: bool,
    preset_name: str | None,
    bake: Callable[..., Path],
    make_asset_cfg: Callable[..., _AssetCfgT],
    make_usd_spawn: Callable[..., Any],
    make_collision_props: Callable[..., Any],
    resolve_source: Callable[..., Any] = resolve_terrain_source,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    size_m: float | None = None,
    deshade: bool = False,
    allow_synthetic: bool = False,
    augment_stamp: bool = False,
    augment_preset: str | None = None,
    num_locations: int = 1,
    locations: Sequence[tuple[float, float]] | None = None,
    location_spread_m: float | None = None,
    make_multi_asset_spawn: Callable[[list[Any]], Any] | None = None,
) -> _AssetCfgT:
    """Build a real-terrain runtime asset from fields and Layer-B adapters.

    Multiple locations are baked independently and assembled by
    ``make_multi_asset_spawn``. A single location uses the existing spec path.
    """
    if locations is not None or num_locations != 1:
        from srb.terrain.locations import derive_location_requests

        requests = derive_location_requests(
            body=body,
            lat_deg=lat_deg
            if lat_deg is not None
            else (locations or [(0.0, 0.0)])[0][0],
            lon_deg=lon_deg
            if lon_deg is not None
            else (locations or [(0.0, 0.0)])[0][1],
            size_m=size_m,  # validated non-None one level up
            num_locations=num_locations,
            location_spread_m=location_spread_m,
            seed=seed,
            lod=lod,
            locations=locations,
        )
    else:
        requests = None

    if requests is None or len(requests) == 1:
        # Single-location path: preserve existing behavior.
        single = requests[0] if requests else None
        spec = build_real_terrain_spec(
            body=body,
            site=site,
            dem_path=dem_path,
            site_name=site_name,
            attribution=attribution,
            patch=patch,
            patch_size_m=patch_size_m,
            patch_center_xy_m=patch_center_xy_m,
            lod=lod,
            seed=seed,
            use_ortho=use_ortho,
            materials_version=materials_version,
            lat_deg=single.lat_deg if single else lat_deg,
            lon_deg=single.lon_deg if single else lon_deg,
            size_m=single.size_m if single else size_m,
            deshade=deshade,
            allow_synthetic=allow_synthetic,
            augment_stamp=augment_stamp,
            augment_preset=augment_preset,
        )
        baked_path = resolve_real_terrain_baked_path(
            spec,
            auto_fetch=auto_fetch,
            preset_name=preset_name,
            bake=bake,
            resolve_source=resolve_source,
        )
        return build_real_terrain_asset_cfg(
            baked_path,
            make_asset_cfg=make_asset_cfg,
            make_usd_spawn=make_usd_spawn,
            make_collision_props=make_collision_props,
        )

    if make_multi_asset_spawn is None:
        raise ValueError(
            "num_locations > 1 (or a multi-entry `locations` list) requires "
            "a `make_multi_asset_spawn` adapter to build the variant spawner"
        )
    spawn_cfgs = []
    for i, request in enumerate(requests):
        _TERRAIN_LOG.info(
            "baking location %d/%d: %s", i + 1, len(requests), request.slug
        )
        spec = build_real_terrain_spec(
            body=body,
            site=None,
            dem_path=None,
            site_name=site_name,
            attribution=attribution,
            patch=patch,
            patch_size_m=None,
            patch_center_xy_m=None,
            lod=lod,
            seed=seed,
            use_ortho=use_ortho,
            materials_version=materials_version,
            lat_deg=request.lat_deg,
            lon_deg=request.lon_deg,
            size_m=request.size_m,
            deshade=deshade,
            allow_synthetic=allow_synthetic,
            augment_stamp=augment_stamp,
            augment_preset=augment_preset,
        )
        baked_path = resolve_real_terrain_baked_path(
            spec,
            auto_fetch=auto_fetch,
            preset_name=preset_name,
            bake=bake,
            resolve_source=resolve_source,
        )
        spawn_cfgs.append(
            make_usd_spawn(
                usd_path=str(baked_path),
                collision_props=make_collision_props(),
            )
        )
    return make_asset_cfg(
        prim_path="{ENV_REGEX_NS}/real_terrain",
        spawn=make_multi_asset_spawn(spawn_cfgs),
    )


def resolve_surface_asset_cfg(
    *,
    surface_name: str,
    body: Body,
    site: str | None = None,
    source: SurfaceSource | str,
    auto_fetch: bool | None = None,
    build_procedural_asset_cfg: Callable[[], _AssetCfgT],
    build_real_asset_cfg: Callable[..., _AssetCfgT] | None = None,
    assess_site: Callable[..., Any] = assess_terrain_site,
    warn: Callable[[str], None] | None = None,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    size_m: float | None = None,
) -> _AssetCfgT:
    """Return the selected asset cfg for a runtime planetary surface.

    Region requests require all of ``lat_deg``, ``lon_deg``, and ``size_m``
    and cannot include ``site``. The resolver preflights availability;
    ``TerrainError`` failures fall back to procedural terrain with a warn-once
    message labeled by the region cache slug.
    """

    if source not in ("procedural", "real"):
        raise ValueError(f"source must be 'procedural' or 'real', got {source!r}")

    region_fields = {"lat_deg": lat_deg, "lon_deg": lon_deg, "size_m": size_m}
    set_region_fields = [k for k, v in region_fields.items() if v is not None]
    is_region = len(set_region_fields) == 3
    if set_region_fields and not is_region:
        missing = [k for k, v in region_fields.items() if v is None]
        raise ValueError(
            "surface region requests require all of lat_deg, lon_deg, and "
            f"size_m; missing {missing}"
        )
    if is_region and site is not None:
        raise ValueError(
            "surface cannot combine a region request (lat_deg/lon_deg/size_m) "
            "with `site`; choose one selection mode"
        )
    if not is_region and site is None:
        raise ValueError(
            "surface needs `site` unless a full lat_deg/lon_deg/size_m "
            "region request is given"
        )

    if source == "procedural":
        return build_procedural_asset_cfg()

    if warn is None:
        warn = _TERRAIN_LOG.warning
    auto_fetch_given = auto_fetch is not None
    if auto_fetch is None:
        auto_fetch = terrain_auto_fetch_default()

    warn_site: str
    region_request: RegionRequest | None = None
    if is_region:
        region_request = RegionRequest(
            body=body, lat_deg=lat_deg, lon_deg=lon_deg, size_m=size_m
        )
        warn_site = region_request.slug
    else:
        assert site is not None  # exactly-one-of validation above
        warn_site = site

    try:
        if is_region:
            assert region_request is not None
            availability = assess_terrain_spec(
                TerrainSpec(body=body, region=region_request)
            )
            should_fallback = availability.issue is not None and (
                not availability.should_attempt_bake
                or (auto_fetch_given and not auto_fetch)
            )
            if should_fallback:
                _warn_real_terrain_fallback(
                    surface_name=surface_name,
                    body=body,
                    site=warn_site,
                    issue=availability.issue,
                    warn=warn,
                )
                return build_procedural_asset_cfg()
            return _call_real_asset_builder(
                build_real_asset_cfg=build_real_asset_cfg,
                body=body,
                site=None,
                auto_fetch=auto_fetch,
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                size_m=size_m,
            )
        availability = assess_site(site, body=body)
        should_fallback = availability.issue is not None and (
            not availability.should_attempt_bake
            or (
                auto_fetch_given
                and (not auto_fetch)
                and availability.status.value == "needs_download"
            )
        )
        if should_fallback:
            _warn_real_terrain_fallback(
                surface_name=surface_name,
                body=body,
                site=warn_site,
                issue=availability.issue,
                warn=warn,
            )
            return build_procedural_asset_cfg()
        return _call_real_asset_builder(
            build_real_asset_cfg=build_real_asset_cfg,
            body=body,
            site=site,
            auto_fetch=auto_fetch,
        )
    except TerrainError as exc:
        _warn_real_terrain_fallback(
            surface_name=surface_name,
            body=body,
            site=warn_site,
            issue=classify_terrain_source_exception(exc),
            warn=warn,
        )
        return build_procedural_asset_cfg()


def _call_real_asset_builder(
    *,
    build_real_asset_cfg: Callable[..., _AssetCfgT] | None,
    body: Body,
    site: str | None,
    auto_fetch: bool,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    size_m: float | None = None,
) -> _AssetCfgT:
    if build_real_asset_cfg is None:
        raise TypeError(
            "source='real' requires the caller to provide build_real_asset_cfg"
        )
    kwargs: dict[str, Any] = {
        "body": body,
        "site": site,
        "auto_fetch": auto_fetch,
    }
    if lat_deg is not None or lon_deg is not None or size_m is not None:
        kwargs.update(lat_deg=lat_deg, lon_deg=lon_deg, size_m=size_m)
    return build_real_asset_cfg(**kwargs)


def _warn_real_terrain_fallback(
    *,
    surface_name: str,
    body: Body,
    site: str,
    issue: TerrainSourceIssue,
    warn: Callable[[str], None],
) -> None:
    warn_terrain_fallback_once(
        surface_name=surface_name,
        body=body,
        site=site,
        issue=issue,
        warn=warn,
    )


__all__ = [
    "SurfaceSource",
    "terrain_auto_fetch_default",
    "validate_real_terrain_selection",
    "build_real_terrain_spec",
    "resolve_real_terrain_baked_path",
    "build_real_terrain_asset_cfg",
    "build_real_terrain_runtime_asset",
    "resolve_surface_asset_cfg",
]
