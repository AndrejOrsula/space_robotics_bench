"""``srb asset download <kind> <target> [--all] [opts]``."""

from __future__ import annotations

import sys
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Optional, TypeVar, cast

from srb.cli.commands._assets._catalog import get_asset_kind
from srb.cli.commands._assets._common import (
    parse_terrain_center,
    run_asset_operation,
)
from srb.terrain.errors import TerrainError

if TYPE_CHECKING:
    from srb.terrain.spec import TerrainSpec

T = TypeVar("T")
R = TypeVar("R")


def _site_body(site: str) -> str:
    from srb.terrain.manifest_loader import load_manifest

    sites = load_manifest().sites
    if site not in sites:
        # Let the caller report this as an operation failure; SystemExit is
        # reserved for argument guards.
        raise ValueError(f"unknown site: {site!r}")
    return sites[site].body


def run_asset_download(
    *,
    kind: str,
    target: Optional[str],
    all: bool,
    items: Mapping[str, T],
    bake: Callable[[str], R],
    format_success: Callable[[str, R, bool], str],
    target_label: str | None = None,
    debug: bool = False,
) -> int:
    """Run the shared ``srb asset download`` all-or-one loop."""
    return run_asset_operation(
        command="download",
        kind=kind,
        target=target,
        all=all,
        items=items,
        action=bake,
        format_success=format_success,
        target_label=target_label,
        debug=debug,
    )


def _download_one_terrain(
    site: str,
    *,
    patch: str,
    lod: str,
    patch_size: Optional[float],
    center: Optional[str],
    no_ortho: bool,
    seed: int,
    force: bool,
    deshade: bool = False,
) -> Path:
    from srb.terrain.lifecycle import assess_terrain_asset
    from srb.terrain.spec import Body, Lod

    body_for_site = _site_body(site)
    if deshade:
        # Reject v1 sites before fetching their DEM and orthomosaic; deshading
        # requires a fused v2 source.
        from srb.terrain.errors import DeshadeUnsupportedError
        from srb.terrain.manifest_loader import get_site

        if get_site(site).sources.dem is not None:
            raise DeshadeUnsupportedError(
                f"site {site!r} is a v1 single-DEM site; --deshade requires "
                "a fused (v2) site or a --lat/--lon/--size-m region"
            )
    parsed_center = parse_terrain_center(center)
    lifecycle = assess_terrain_asset(
        site,
        body=cast(Body, body_for_site),
        patch=patch,
        patch_size_m=patch_size,
        patch_center_xy_m=parsed_center,
        lod=cast(Lod, lod),
        seed=seed,
        use_ortho=not no_ortho,
        deshade=deshade,
    )
    cached = lifecycle.cached_path_for_download(force=force)
    if cached is not None:
        return cached
    if lifecycle.blocking_error is not None:
        raise lifecycle.blocking_error
    if not (force or lifecycle.should_attempt_bake):
        raise TerrainError(
            f"terrain {site!r} is not downloadable in state "
            f"{lifecycle.availability.status.value}",
            remediation=lifecycle.next_step or "Run `srb asset doctor terrain`.",
        )

    from srb.terrain.api import bake_or_load
    from srb.terrain.availability import _cache_paths as _terrain_cache_paths
    from srb.terrain.cache import remove_baked_cache
    from srb.terrain.materials.presets import get_default_preset

    spec = lifecycle.availability.spec

    if force:
        # Use the shared v1/v2-aware cache key so --force removes the same
        # entry that bake_or_load would otherwise reuse.
        paths = _terrain_cache_paths(
            spec, preset_name=get_default_preset(spec.body).name
        )
        remove_baked_cache(paths)

    return bake_or_load(spec, auto_fetch=True)


_REGION_FLAG_NAMES = ("--lat", "--lon", "--size-m")


def _validate_region_form(
    *,
    target: Optional[str],
    all: bool,
    lat: Optional[float],
    lon: Optional[float],
    size_m: Optional[float],
    patch_size: Optional[float],
    center: Optional[str],
    patch: str,
    no_ortho: bool,
    num_locations: int = 1,
    location_spread_m: Optional[float] = None,
) -> bool:
    """Validate region flags; return True when the complete region form is used.

    Coordinates and size are all required and cannot combine with a site or
    ``--all``. Region requests also reject curated-site options that do not
    apply: ``--patch-size``, ``--center``, ``--patch``, and ``--no-ortho``.
    Invalid combinations exit 2 with the offending flags named.
    """
    region_flags = dict(zip(_REGION_FLAG_NAMES, (lat, lon, size_m)))
    given = [name for name, value in region_flags.items() if value is not None]
    missing = [name for name, value in region_flags.items() if value is None]
    if given and missing:
        print(
            "download terrain: the region form requires all of "
            f"{', '.join(_REGION_FLAG_NAMES)}; missing {', '.join(missing)}",
            file=sys.stderr,
        )
        raise SystemExit(2)
    if not given:
        if num_locations != 1 or location_spread_m is not None:
            print(
                "download terrain: --num-locations/--location-spread-m "
                "require the --lat/--lon/--size-m region form",
                file=sys.stderr,
            )
            raise SystemExit(2)
        return False
    if target:
        print(
            f"download terrain: cannot combine the site target {target!r} "
            "with the --lat/--lon/--size-m region form; use one or the other",
            file=sys.stderr,
        )
        raise SystemExit(2)
    if all:
        print(
            "download terrain: --all cannot be combined with the "
            "--lat/--lon/--size-m region form",
            file=sys.stderr,
        )
        raise SystemExit(2)
    if patch_size is not None:
        print(
            "download terrain: --patch-size has no effect on a region "
            "download; the region's own --size-m defines the bake footprint",
            file=sys.stderr,
        )
        raise SystemExit(2)
    if center is not None:
        print(
            "download terrain: --center has no effect on a region download; "
            "the region's --lat/--lon define the bake center",
            file=sys.stderr,
        )
        raise SystemExit(2)
    if patch != "default":
        print(
            "download terrain: --patch has no effect on a region download; "
            "a region has no manifest patches -- its --lat/--lon/--size-m "
            "define the bake footprint",
            file=sys.stderr,
        )
        raise SystemExit(2)
    if no_ortho:
        print(
            "download terrain: --no-ortho has no effect on a region "
            "download; a region's color always comes from its resolved "
            "color stack",
            file=sys.stderr,
        )
        raise SystemExit(2)
    if num_locations < 1:
        print(
            f"download terrain: --num-locations must be >= 1, got {num_locations}",
            file=sys.stderr,
        )
        raise SystemExit(2)
    return True


def _bake_region_spec(spec: "TerrainSpec") -> Path:
    """Bake seam for tests of real request derivation and spec construction."""
    from srb.terrain.api import bake_or_load

    return bake_or_load(spec, auto_fetch=True)


def _download_region_terrain(
    *,
    lat: float,
    lon: float,
    size_m: float,
    body: Optional[str],
    lod: str,
    seed: int,
    deshade: bool,
    force: bool,
    num_locations: int = 1,
    location_spread_m: Optional[float] = None,
) -> int:
    """Bake region requests through the standard spec and acquisition path.

    Unresolvable regions print an explanation and return 1; the CLI has no
    procedural fallback. Other ``TerrainError`` instances use the standard
    ``AssetCommand.execute`` handler. Multiple locations run in derivation
    order and stop at the first unresolved region, leaving earlier results
    baked and printed.
    """
    from srb.terrain.errors import RegionUnresolvedError
    from srb.terrain.locations import derive_location_requests
    from srb.terrain.spec import Body, Lod
    from srb.terrain.surface_source import build_real_terrain_spec

    if force:
        # Match bake_or_load's region cache key, including stack hashes and
        # deshade, before removing the cached entry.
        from srb.terrain.availability import _cache_paths as _terrain_cache_paths
        from srb.terrain.cache import remove_baked_cache
        from srb.terrain.materials.presets import get_default_preset

    # One location preserves the requested center; larger counts fan out.
    requests = derive_location_requests(
        body=cast(Body, body or "moon"),
        lat_deg=lat,
        lon_deg=lon,
        size_m=size_m,
        num_locations=num_locations,
        location_spread_m=location_spread_m,
        seed=seed,
        lod=cast(Lod, lod),
    )

    for request in requests:
        spec = build_real_terrain_spec(
            body=cast(Body, body or "moon"),
            site=None,
            dem_path=None,
            site_name=None,
            attribution=None,
            patch="default",
            patch_size_m=None,
            patch_center_xy_m=None,
            lod=cast(Lod, lod),
            seed=seed,
            # Region validation rejects --no-ortho; the spec defaults to on.
            use_ortho=True,
            materials_version=None,
            lat_deg=request.lat_deg,
            lon_deg=request.lon_deg,
            size_m=request.size_m,
            deshade=deshade,
        )
        assert spec.region is not None  # the validated triplet built a region
        slug = spec.region.slug

        try:
            if force:
                paths = _terrain_cache_paths(
                    spec, preset_name=get_default_preset(spec.body).name
                )
                remove_baked_cache(paths)

            baked = _bake_region_spec(spec)
        except RegionUnresolvedError as e:
            # Report unresolved regions without a traceback or fallback;
            # include the slug to identify a failed location in a batch.
            print(f"{slug}: ERROR: {e}", file=sys.stderr)
            if e.remediation:
                print(f"{slug}: hint: {e.remediation}", file=sys.stderr)
            # Mention catalog dormancy only when the loaded catalog is empty;
            # other resolution failures must not blame the catalog.
            try:
                from srb.terrain.catalog.loader import (
                    catalog_path,
                    load_source_catalog,
                )

                region_body = spec.region.body
                catalog_is_empty = not load_source_catalog(region_body).layers
            except Exception:
                catalog_is_empty = False
            if catalog_is_empty:
                print(
                    f"{slug}: note: the source catalog for body "
                    f"{region_body!r} ({catalog_path(region_body)}) has no "
                    "terrain layers configured (`layers: []`), so every "
                    "region resolves empty",
                    file=sys.stderr,
                )
            return 1

        print(f"{slug}: {baked}")

    return 0


def _download_terrain(
    *,
    target: Optional[str],
    all: bool,
    body: Optional[str],
    lat: Optional[float] = None,
    lon: Optional[float] = None,
    size_m: Optional[float] = None,
    deshade: bool = False,
    num_locations: int = 1,
    location_spread_m: Optional[float] = None,
    **opts,
) -> int:
    is_region = _validate_region_form(
        target=target,
        all=all,
        lat=lat,
        lon=lon,
        size_m=size_m,
        patch_size=opts.get("patch_size"),
        center=opts.get("center"),
        patch=opts.get("patch") or "default",
        no_ortho=opts.get("no_ortho", False),
        num_locations=num_locations,
        location_spread_m=location_spread_m,
    )
    if is_region:
        assert lat is not None and lon is not None and size_m is not None
        return _download_region_terrain(
            lat=lat,
            lon=lon,
            size_m=size_m,
            body=body,
            lod=opts.get("lod", "medium"),
            seed=opts.get("seed") if opts.get("seed") is not None else 0,
            deshade=deshade,
            force=opts.get("force", False),
            num_locations=num_locations,
            location_spread_m=location_spread_m,
        )

    if not target and not all:
        print("download terrain: must specify <site> or --all", file=sys.stderr)
        raise SystemExit(2)

    # Remove parser-only flags before forwarding bake options.
    debug = opts.pop("debug", False)
    opts["deshade"] = deshade
    opts["patch"] = opts.get("patch") or "default"
    opts["seed"] = opts.get("seed") if opts.get("seed") is not None else 0

    if all:
        from srb.cli.commands._assets._common import filter_by_body
        from srb.terrain.manifest_loader import load_manifest

        sites = filter_by_body(load_manifest().sites, body)
        return run_asset_download(
            kind="terrain",
            target=None,
            all=True,
            items=sites,
            bake=lambda site_name: _download_one_terrain(site_name, **opts),
            format_success=lambda _name, path, _all: str(path),
            debug=debug,
        )

    # The guard above guarantees a target unless --all was selected.
    assert target is not None
    return run_asset_download(
        kind="terrain",
        target=target,
        all=False,
        items={target: object()},
        bake=lambda site_name: _download_one_terrain(site_name, **opts),
        format_success=lambda _name, path, _all: str(path),
        debug=debug,
    )


def _download_rocks(
    *, target: Optional[str], all: bool, body: Optional[str], **_unused
) -> int:
    from srb.cli.commands._assets._common import filter_by_body
    from srb.terrain.rocks.catalog_loader import load_rocks_catalog
    from srb.terrain.rocks.pipeline import bake_rock_set

    sets = filter_by_body(load_rocks_catalog().sets, body)
    return run_asset_download(
        kind="rocks",
        target=target,
        all=all,
        items=sets,
        bake=bake_rock_set,
        format_success=lambda _name, paths, batch: (
            f"{len(paths)} rocks baked"
            if batch
            else f"{len(paths)} rocks baked at {paths[0].parent.parent}"
        ),
        target_label="set",
        debug=_unused.get("debug", False),
    )


def _download_hdri(
    *, target: Optional[str], all: bool, body: Optional[str], **_unused
) -> int:
    from srb.cli.commands._assets._common import filter_by_body
    from srb.terrain.hdri.catalog_loader import load_hdri_catalog
    from srb.terrain.hdri.pipeline import bake_skydome

    domes = filter_by_body(load_hdri_catalog().skydomes, body)
    return run_asset_download(
        kind="hdri",
        target=target,
        all=all,
        items=domes,
        bake=bake_skydome,
        format_success=lambda _name, path, _all: str(path),
        target_label="name",
        debug=_unused.get("debug", False),
    )


def _download_lights(*, target: Optional[str], **_unused) -> int:
    from srb.terrain.lights.pipeline import bake_lighting

    if not target:
        print("download lights: must specify <preset>", file=sys.stderr)
        raise SystemExit(2)
    return run_asset_download(
        kind="lights",
        target=target,
        all=False,
        items={target: object()},
        bake=bake_lighting,
        format_success=lambda _name, path, _all: str(path),
        target_label="preset",
        debug=_unused.get("debug", False),
    )


def run(*, kind: str, **kwargs) -> int:
    asset_kind = get_asset_kind(kind)
    if not asset_kind.supports_download:
        raise ValueError(f"download: unsupported asset kind: {kind!r}")
    if kind != "terrain":
        # Reject terrain-only flags here; other handlers accept and ignore
        # unknown keyword arguments.
        num_locations = kwargs.get("num_locations", 1)
        terrain_only = {
            "--force": kwargs.get("force") or None,
            "--patch": (
                kwargs.get("patch") if kwargs.get("patch") is not None else None
            ),
            "--patch-size": kwargs.get("patch_size"),
            "--center": kwargs.get("center"),
            "--no-ortho": kwargs.get("no_ortho") or None,
            "--seed": kwargs.get("seed"),
            "--lod": kwargs.get("lod")
            if kwargs.get("lod", "medium") != "medium"
            else None,
            "--lat": kwargs.get("lat"),
            "--lon": kwargs.get("lon"),
            "--size-m": kwargs.get("size_m"),
            "--deshade": kwargs.get("deshade") or None,
            # Match `_validate_region_form`'s predicate -- an explicit
            # `--num-locations 1`
            # is a semantic no-op (indistinguishable from "not passed"), the
            # same benign imprecision already accepted for --deshade above.
            "--num-locations": num_locations if num_locations != 1 else None,
            "--location-spread-m": kwargs.get("location_spread_m"),
        }
        offenders = [flag for flag, value in terrain_only.items() if value is not None]
        if offenders:
            print(
                f"download {kind}: {', '.join(offenders)} "
                f"{'is' if len(offenders) == 1 else 'are'} only valid for "
                "kind 'terrain'",
                file=sys.stderr,
            )
            raise SystemExit(2)
    return _DOWNLOAD_HANDLERS[kind](**kwargs)


_DOWNLOAD_HANDLERS = {
    "terrain": _download_terrain,
    "rocks": _download_rocks,
    "hdri": _download_hdri,
    "lights": _download_lights,
}
