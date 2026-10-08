"""V2 terrain resolution and fusion-bake orchestration."""

from __future__ import annotations

import hashlib
import json
import logging
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Optional, TypedDict

from srb.terrain import cache as cache_mod
from srb.terrain.log import get_logger, stage_timer
from srb.terrain.manifest_loader import get_site
from srb.terrain.source_health import (
    TerrainFallbackWarning,
    classify_terrain_source_exception,
)
from srb.terrain.spec import TerrainSpec
from srb.utils.failures import (
    FALLBACK_ALLOWED_CATEGORIES,
    FailureError,
    HealthEvent,
)

if TYPE_CHECKING:
    from srb.terrain.catalog.resolve import CompositionPlan
    from srb.terrain.catalog_schema import LayerKind, SourceCatalog, SourceLayer
    from srb.terrain.fusion.engine import FusionResult, LayerLoader
    from srb.terrain.fusion.grid import PatchGrid
    from srb.terrain.ingest.dem import Dem, Resampling
    from srb.terrain.schema import SiteEntry, StackRef
    from srb.terrain.sources.acquisition import ProviderGetter
    from srb.terrain.sources.vsicurl import Opener
    from srb.utils.failures import FailureInfo

_log = get_logger("api")


@dataclass(frozen=True, slots=True)
class V2BakePlan:
    """Resolved composition and cache identity for a v2 bake."""

    composition_plan: "CompositionPlan"
    catalog: "SourceCatalog"
    cache_paths: cache_mod.CachePaths
    effective_deshade: bool
    dem_stack_sha256: str
    color_stack_sha256: str


def _require_string(value: object, *, source: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{source} returned {type(value).__name__}, expected str")
    return value


def _require_bool(value: object, *, source: str) -> bool:
    if not isinstance(value, bool):
        raise TypeError(f"{source} returned {type(value).__name__}, expected bool")
    return value


def _require_allowed_hosts(value: object, *, source: str) -> frozenset[str]:
    if not isinstance(value, frozenset) or not all(
        isinstance(host, str) for host in value
    ):
        raise TypeError(f"{source} returned an invalid host allowlist")
    return value


MAX_WINDOW_PIXELS = 4096
_LOADER_PAD_M = 200.0
_INGEST_RESAMPLING_BY_KIND: dict[LayerKind, Resampling] = {
    "dem": "bilinear",
    "color": "bilinear",
    "mask": "nearest",
    "quality": "nearest",
}


def _effective_deshade(
    spec: TerrainSpec,
    *,
    site_getter: Callable[[str], "SiteEntry"] = get_site,
) -> bool:
    """Return the effective flag: manifest OR request for sites, request-only for
    regions. Used by both cache identity and fusion.
    """
    if spec.site is not None:
        site_entry: SiteEntry = site_getter(spec.site)
        return _require_bool(
            site_entry.deshade, source="manifest deshade"
        ) or _require_bool(spec.deshade, source="request deshade")
    return _require_bool(spec.deshade, source="request deshade")


def _resolve_v2_region(
    spec: TerrainSpec,
) -> Optional[tuple[CompositionPlan, SourceCatalog]]:
    """Resolve a region and catalog; return None when not a region. Empty DEM plans
    raise with resolver warnings so no silent empty bake occurs. Consent here only
    reports required layers; acquisition enforces the fetch gate.
    """
    if spec.region is None:
        return None

    from srb.terrain.catalog.loader import load_source_catalog
    from srb.terrain.catalog.resolve import resolve_stacks
    from srb.terrain.errors import RegionUnresolvedError
    from srb.terrain.sources.acquisition import large_download_consented

    region = spec.region
    catalog: SourceCatalog = load_source_catalog(region.body)
    plan: CompositionPlan = resolve_stacks(
        region, catalog, consented=large_download_consented()
    )
    if plan.is_empty:
        reason = "no catalog DEM layer remains after region filters"
        raise RegionUnresolvedError(
            body=region.body,
            lat_deg=region.lat_deg,
            lon_deg=region.lon_deg,
            size_m=region.size_m,
            reason=reason,
            warnings=tuple(plan.warnings),
        )
    return plan, catalog


def _resolve_v2_site(
    spec: TerrainSpec,
    *,
    site_getter: Callable[[str], "SiteEntry"] = get_site,
) -> Optional[tuple[CompositionPlan, SourceCatalog]]:
    """Resolve declared-stack sites; return None for local DEMs and v1 single-source
    sites. Runs before cache lookup so stack identity is available. Unresolvable
    bases and manifest/catalog authoring errors propagate; v2 sites have no v1
    fallback.
    """
    if spec.dem_path is not None:
        return None  # user-supplied local DEM; never a v2 site.
    assert spec.site is not None  # TerrainSpec.__post_init__ guarantees this.

    site_entry: SiteEntry = site_getter(spec.site)
    if site_entry.sources.dem is not None:
        return None  # v1 curated site (single `sources.dem`).

    # Defense in depth: the manifest validator already forbids `dem_stack`
    # in v1 manifests. A different version here means validation was bypassed.
    from srb.terrain.manifest_loader import load_manifest

    manifest = load_manifest()
    if manifest.version != 2:
        raise ValueError(
            f"site {spec.site!r} declares sources.dem_stack under a "
            f"version={manifest.version!r} manifest; "
            "Manifest._validate_version_1_excludes_v2_fields should have "
            "rejected this at load time"
        )

    from srb.terrain.catalog.loader import load_source_catalog
    from srb.terrain.catalog.resolve import (
        POLAR_LAT_DEG,
        RegionRequest,
        resolve_declared_stacks,
    )
    from srb.terrain.errors import ManifestInvalidError
    from srb.terrain.ingest.planetary_crs import (
        canonical_crs_for,
        local_stereographic_crs,
        xy_to_latlon,
    )

    patch = site_entry.patches.get(spec.patch)
    if patch is None:
        raise ManifestInvalidError(f"site '{spec.site}' has no patch '{spec.patch}'")

    effective_patch_size_m = (
        spec.patch_size_m if spec.patch_size_m is not None else patch.size_m
    )

    def _nominal_crs(lat_deg: float, lon_deg: float) -> str:
        """Use the body-aware CRS ladder at the site nominal coordinates; a center_xy
        patch needs that known CRS to convert its own center.
        """
        if abs(lat_deg) >= POLAR_LAT_DEG:
            return _require_string(
                canonical_crs_for(spec.body, lat_deg, polar_lat_deg=POLAR_LAT_DEG),
                source="canonical planetary CRS",
            )
        return _require_string(
            local_stereographic_crs(spec.body, lat_deg, lon_deg),
            source="local stereographic CRS",
        )

    # `spec.patch_center_xy_m` overrides either manifest center field; the
    # schema guarantees that one manifest field is present.
    _nominal = _nominal_crs(site_entry.coords.lat, site_entry.coords.lon)
    if spec.patch_center_xy_m is not None:
        cx, cy = spec.patch_center_xy_m
        lat_deg, lon_deg = xy_to_latlon(cx, cy, _nominal)
    elif patch.center_latlon is not None:
        lat_deg, lon_deg = patch.center_latlon
    else:
        assert patch.center_xy_m is not None  # PatchEntry: exactly-one-of
        cx, cy = patch.center_xy_m
        lat_deg, lon_deg = xy_to_latlon(cx, cy, _nominal)

    request = RegionRequest(
        body=spec.body,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        size_m=effective_patch_size_m,
        lod=spec.lod,
        allow_synthetic=spec.allow_synthetic,
    )

    catalog: SourceCatalog = load_source_catalog(spec.body)
    dem_refs = site_entry.sources.dem_stack
    assert dem_refs is not None  # Sources: exactly-one-of dem/dem_stack
    color_refs = site_entry.sources.color_stack or ()

    plan: CompositionPlan = resolve_declared_stacks(
        request, catalog, dem_refs=dem_refs, color_refs=color_refs
    )
    _raise_if_declared_base_unresolvable(plan, dem_refs=dem_refs, site=spec.site)

    return plan, catalog


def _raise_if_declared_base_unresolvable(
    plan: CompositionPlan, *, dem_refs: Sequence[StackRef], site: str
) -> None:
    """Reject a missing declared base before cache-key creation. Distinguish a base
    dropped by max_gsd from a resolved base with zero coverage; inspect structure,
    not warning text.
    """
    from srb.terrain.errors import DeclaredBaseUnavailableError

    if plan.is_empty:
        base_ref = next((r for r in dem_refs if r.role == "base"), None)
        layer_id = base_ref.layer_id if base_ref is not None else "<unknown>"
        raise DeclaredBaseUnavailableError(
            layer_id,
            site=site,
            reason=(
                "no DEM base resolved -- dropped by the declared stack's "
                "max_gsd_m filter (see CompositionPlan.warnings)"
            ),
        )

    base = plan.dem_stack[0]
    if base.coverage_fraction <= 0.0:
        raise DeclaredBaseUnavailableError(
            base.layer_id,
            site=site,
            reason="the declared base has zero coverage of the requested patch",
        )


def _v2_stack_hashes(plan: CompositionPlan, catalog: SourceCatalog) -> tuple[str, str]:
    """Return canonical DEM and colour/auxiliary stack identities."""
    from srb.terrain.catalog.resolve import to_stack_refs
    from srb.terrain.schema import StackRef

    dem_refs, color_refs = to_stack_refs(plan)
    extra_color_refs = [
        StackRef(layer_id=c.layer_id, role=c.role, max_gsd_m=None)
        for c in (*plan.mask_layers, *plan.color_stack_psr)
    ]
    quality_refs = [
        StackRef(layer_id=c.layer_id, role=c.role, max_gsd_m=None)
        for c in plan.quality_layers
    ]
    dem_hash = cache_mod.stack_hash(dem_refs, catalog)
    geometry = json.dumps(
        [plan.crs, plan.center_xy_m, plan.request.size_m, plan.target_gsd_m],
        allow_nan=False,
        separators=(",", ":"),
    )
    dem_hash = hashlib.sha256(f"{dem_hash}:{geometry}".encode()).hexdigest()
    return (
        dem_hash,
        cache_mod.stack_hash([*color_refs, *extra_color_refs, *quality_refs], catalog),
    )


def _v2_cache_paths(
    spec: TerrainSpec,
    *,
    plan: CompositionPlan,
    catalog: SourceCatalog,
    materials_version: int,
    preset_name: Optional[str],
    deshade_resolver: Callable[[TerrainSpec], bool] | None = None,
    stack_hashes: tuple[str, str] | None = None,
) -> cache_mod.CachePaths:
    """Build the v2 key from the v1 key plus resolved stack identities. Preserve v1
    fields via CacheKey.from_spec/replace. Hash color, mask, PSR, and quality refs in
    fixed order inside the existing color hash; `stack_hash` is order-sensitive.
    Catalog loading is cached, so on-disk edits require cache clearing or a new
    process.
    """
    import dataclasses

    from srb.terrain.fusion import FUSION_VERSION

    v1_key = cache_mod.CacheKey.from_spec(
        spec, materials_version=materials_version, preset_name=preset_name
    )
    dem_stack_hash, color_stack_hash = (
        stack_hashes if stack_hashes is not None else _v2_stack_hashes(plan, catalog)
    )
    key = dataclasses.replace(
        v1_key,
        dem_stack_hash=dem_stack_hash,
        color_stack_hash=color_stack_hash,
        fusion_version=FUSION_VERSION,
        target_gsd_m=plan.target_gsd_m,
        # Include the effective manifest-or-request value: manifest-declared
        # deshading must affect the cache key even when the request omits it.
        deshade=(deshade_resolver or _effective_deshade)(spec),
    )
    return cache_mod.CachePaths(
        root=cache_mod.cache_root(),
        body=spec.body,
        site=spec.cache_site,
        key=key.hash12(),
    )


def resolve_v2_bake_plan(
    spec: TerrainSpec,
    *,
    materials_version: int,
    preset_name: Optional[str],
    site_getter: Callable[[str], "SiteEntry"] = get_site,
) -> V2BakePlan | None:
    """Resolve composition, effective deshade, stack hashes, and matching cache paths
    together. Return None for v1/local requests; site_getter remains a test seam.
    """
    resolved = (
        _resolve_v2_region(spec)
        if spec.region is not None
        else _resolve_v2_site(spec, site_getter=site_getter)
    )
    if resolved is None:
        return None

    composition_plan, catalog = resolved
    effective_deshade = _effective_deshade(spec, site_getter=site_getter)
    stack_hashes = _v2_stack_hashes(composition_plan, catalog)
    cache_paths = _v2_cache_paths(
        spec,
        plan=composition_plan,
        catalog=catalog,
        materials_version=materials_version,
        preset_name=preset_name,
        deshade_resolver=lambda _spec: effective_deshade,
        stack_hashes=stack_hashes,
    )
    return V2BakePlan(
        composition_plan=composition_plan,
        catalog=catalog,
        cache_paths=cache_paths,
        effective_deshade=effective_deshade,
        dem_stack_sha256=stack_hashes[0],
        color_stack_sha256=stack_hashes[1],
    )


@dataclass(frozen=True, slots=True)
class _WindowReadRequest:
    """Common typed request for local and remote readers."""

    layer_id: str
    center_xy_m: tuple[float, float]
    size_m: float
    target_gsd_m: float | None
    pad_m: float
    dst_crs: str
    resampling: Resampling
    declared_crs: str | None
    body: str


class _RemoteWindowKwargs(TypedDict):
    """Keyword payload shared by remote window readers."""

    layer_id: str
    content_sha256: str
    center_xy_m: tuple[float, float]
    size_m: float
    target_gsd_m: float | None
    pad_m: float
    allowed_hosts: frozenset[str]
    cache_root: Path | None
    opener: "Opener | None"
    dst_crs: str | None
    resampling: Resampling
    declared_crs: str | None
    body: str | None
    verified_local_artifact: Path | None


@dataclass(frozen=True, slots=True)
class _LayerLoader:
    """Layer loader plus diagnostics used to report fatal base failures."""

    load: "LayerLoader"
    last_errors: dict[str, BaseException]
    health_events: dict[str, HealthEvent] = field(default_factory=dict)

    def __call__(self, layer: "SourceLayer", grid: "PatchGrid") -> "Dem | None":
        return self.load(layer, grid)


def _normalize_color_dem(dem: "Dem", *, layer_id: str) -> "Dem":
    """Normalize monochrome color to HWC3; reject unsupported channel counts."""
    if dem.heights.ndim != 3:
        raise ValueError(
            f"color layer {layer_id!r} returned unsupported heights shape "
            f"{dem.heights.shape}; expected (H, W, 1) or (H, W, 3)"
        )
    channels = dem.heights.shape[2]
    if channels == 1:
        import dataclasses

        import numpy as np

        return dataclasses.replace(
            dem,
            heights=np.repeat(dem.heights, 3, axis=2),
        )
    if channels != 3:
        raise ValueError(
            f"color layer {layer_id!r} returned {channels} color channels; "
            "expected 1 or 3"
        )
    return dem


def _layer_read_failure_info(exc: BaseException, *, layer_id: str) -> "FailureInfo":
    """Classify one failed read. RasterioIOError from the remote window reader is
    otherwise untyped and would abort optional-layer fallback, so wrap only that
    availability error. Structural Rasterio errors and integrity failures must
    propagate.
    """
    import rasterio.errors

    from srb.utils.failures import FailureCategory, FailureInfo

    if isinstance(exc, FailureError):
        return exc.failure_info
    if isinstance(exc, rasterio.errors.RasterioIOError):
        return FailureInfo(
            category=FailureCategory.TRANSIENT_EXTERNAL_FAILURE,
            code="terrain.v2.layer_read_io",
            message=(
                f"windowed read of layer {layer_id!r} failed with a raster I/O "
                f"error: {exc}"
            ),
            component="terrain.v2.layer_loader",
            details={"layer_id": layer_id, "error_type": type(exc).__name__},
            retryable=True,
            fallback_allowed=True,
        )
    return classify_terrain_source_exception(exc).failure_info


def _native_window_gsd_m(
    *, native_gsd_m: float, window_size_m: float, max_window_pixels: int
) -> tuple[Optional[float], bool]:
    """Return native GSD unless it exceeds the per-side pixel cap; then return a coarser
    GSD and mark the read for warning.
    """
    native_pixels = window_size_m / native_gsd_m
    if native_pixels <= max_window_pixels:
        return None, False
    return window_size_m / max_window_pixels, True


def _whole_pixel_window_pad_m(
    *, size_m: float, native_gsd_m: float, configured_pad_m: float
) -> float:
    """Quantize padding only for coarse layers: plan_window can round away requested
    coverage when a native pixel exceeds twice the margin. Fine layers retain the
    configured padding.
    """
    if native_gsd_m <= 2.0 * configured_pad_m:
        return configured_pad_m

    requested_extent_m = size_m + 2.0 * configured_pad_m
    native_pixels = math.ceil(requested_extent_m / native_gsd_m)
    whole_pixel_pad_m = (native_pixels * native_gsd_m - size_m) / 2.0
    return max(configured_pad_m, whole_pixel_pad_m)


def _make_layer_loader(
    *,
    root: Optional[Path] = None,
    provider_getter: Optional["ProviderGetter"] = None,
    env: Optional[Mapping[str, str]] = None,
    opener: Optional["Opener"] = None,
    logger: logging.Logger | None = None,
    max_window_pixels: int = MAX_WINDOW_PIXELS,
    pad_m: float = _LOADER_PAD_M,
    resampling_by_kind: dict[LayerKind, Resampling] = _INGEST_RESAMPLING_BY_KIND,
) -> _LayerLoader:
    """Build the production loader through catalog planning and acquisition. Remote COGs
    are fetched and verified locally before window reads; injected openers are
    test-only. Preserve host allowlists, checksums, zip-member extraction, and
    multiband color reads. Read at native GSD unless the pixel cap requires
    decimation; output-grid GSD here would trigger a nearest-neighbor pass-through.
    Only explicitly fallback-allowed failures return None and record health; all
    others propagate. The caller turns a missing base into an error.
    """
    import dataclasses

    import rasterio.errors as rasterio_errors

    from srb.terrain.catalog_schema import url_basename
    from srb.terrain.errors import TerrainError
    from srb.terrain.ingest.dem import (
        _require_dem,
        load_dem_window,
        load_dem_window_multiband,
    )
    from srb.terrain.ingest.mask import decode_mask_dn
    from srb.terrain.sources.acquisition import (
        allowed_hosts_for_layer,
        fetch_catalog_layer,
        plan_catalog_layer,
    )
    from srb.terrain.sources.vsicurl import (
        read_window_remote,
        read_window_remote_multiband,
    )

    _active_log = logger or _log

    last_errors: dict[str, BaseException] = {}
    health_events: dict[str, HealthEvent] = {}

    def _load(layer: "SourceLayer", grid: "PatchGrid") -> Optional["Dem"]:
        effective_pad_m = _whole_pixel_window_pad_m(
            size_m=grid.size_m,
            native_gsd_m=layer.native_gsd_m,
            configured_pad_m=pad_m,
        )
        window_size_m = grid.size_m + 2.0 * effective_pad_m
        target_gsd_m, decimated = _native_window_gsd_m(
            native_gsd_m=layer.native_gsd_m,
            window_size_m=window_size_m,
            max_window_pixels=max_window_pixels,
        )
        if decimated:
            _active_log.warning(
                "v2 layer loader: %r native read (~%d px/side at %.3f m) "
                "exceeds the %d px cap; decimating to %.3f m/px via the "
                "source's overview pyramid (averaged, not nearest-neighbour)",
                layer.layer_id,
                round(window_size_m / layer.native_gsd_m),
                layer.native_gsd_m,
                max_window_pixels,
                target_gsd_m,
            )

        # Plan outside the fetch error handler: catalog/loader errors such as
        # ambiguous archives must not be treated as unavailable layers.
        layer_plan = plan_catalog_layer(layer, root=root)
        if layer.access == "zip_member" and len(layer_plan.member_paths) != 1:
            raise ValueError(
                f"{layer.layer_id}: zip_member layer declares "
                f"{len(layer_plan.member_paths)} extracted member(s) "
                f"({sorted(layer_plan.member_paths)}); the v2 loader "
                "only supports exactly one -- which raster member to "
                "read is otherwise ambiguous"
            )

        ingest_resampling: Resampling = resampling_by_kind[layer.kind]
        window_request = _WindowReadRequest(
            layer_id=layer.layer_id,
            center_xy_m=grid.center_xy_m,
            size_m=grid.size_m,
            target_gsd_m=target_gsd_m,
            pad_m=effective_pad_m,
            dst_crs=grid.crs,
            resampling=ingest_resampling,
            declared_crs=layer.crs,
            body=layer.body,
        )
        loaded_dem: object
        try:
            if layer.access == "cog_remote":
                url = layer.urls[0]
                # Acquire and verify through SRB before windowing locally;
                # GDAL /vsicurl cannot enforce host policy across redirects.
                # An injected opener keeps tests network-free.
                verified_local_artifact = (
                    None
                    if opener is not None
                    else fetch_catalog_layer(
                        layer_plan, provider_getter=provider_getter, env=env
                    )
                )
                remote_kwargs: _RemoteWindowKwargs = {
                    "layer_id": window_request.layer_id,
                    "content_sha256": layer.sha256[url_basename(url)],
                    "center_xy_m": window_request.center_xy_m,
                    "size_m": window_request.size_m,
                    "target_gsd_m": window_request.target_gsd_m,
                    "pad_m": window_request.pad_m,
                    "allowed_hosts": _require_allowed_hosts(
                        allowed_hosts_for_layer(layer, provider_getter=provider_getter),
                        source=f"allowed_hosts_for_layer({layer.layer_id!r})",
                    ),
                    "cache_root": root,
                    "opener": opener,
                    # Reproject only when CRS differs or pixels are non-square.
                    "dst_crs": window_request.dst_crs,
                    "resampling": window_request.resampling,
                    # Check declared CRS against the source header before
                    # reprojection. A null declaration skips this check.
                    "declared_crs": window_request.declared_crs,
                    "body": window_request.body,
                    "verified_local_artifact": verified_local_artifact,
                }
                if layer.kind == "color":
                    loaded_dem = read_window_remote_multiband(url, **remote_kwargs)
                else:
                    # Match local reads: out-of-footprint margins must be
                    # NaN, not edge-replicated, in both branches.
                    loaded_dem = read_window_remote(
                        url, pad_mode="nan", **remote_kwargs
                    )
            else:
                fetched_path: Path = fetch_catalog_layer(
                    layer_plan, provider_getter=provider_getter, env=env
                )
                if layer.access == "zip_member":
                    (fetched_path,) = layer_plan.member_paths.values()
                # Preserve multi-band color data; the single-band reader
                # drops all but red, breaking color fusion and gain matching.
                if layer.kind == "color":
                    loaded_dem = load_dem_window_multiband(
                        fetched_path,
                        center_xy_m=window_request.center_xy_m,
                        size_m=window_request.size_m,
                        target_gsd_m=window_request.target_gsd_m,
                        pad_m=window_request.pad_m,
                        dst_crs=window_request.dst_crs,
                        resampling=window_request.resampling,
                        # See the `cog_remote` branch's
                        # own comment above for the full rationale.
                        declared_crs=window_request.declared_crs,
                        layer_id=window_request.layer_id,
                        body=window_request.body,
                    )
                else:
                    # NaN padding preserves inset footprints; edge padding
                    # repeats pixels into the margin and expands the layer.
                    loaded_dem = load_dem_window(
                        fetched_path,
                        center_xy_m=window_request.center_xy_m,
                        size_m=window_request.size_m,
                        target_gsd_m=window_request.target_gsd_m,
                        pad_m=window_request.pad_m,
                        pad_mode="nan",
                        dst_crs=window_request.dst_crs,
                        resampling=window_request.resampling,
                        # See the `cog_remote` branch's
                        # own comment above for the full rationale.
                        declared_crs=window_request.declared_crs,
                        layer_id=window_request.layer_id,
                        body=window_request.body,
                    )
        except (FailureError, TerrainError, rasterio_errors.RasterioIOError) as exc:
            failure = _layer_read_failure_info(exc, layer_id=layer.layer_id)
            if (
                not failure.fallback_allowed
                or failure.category not in FALLBACK_ALLOWED_CATEGORIES
            ):
                raise
            health_event = HealthEvent.degraded_fallback(
                failure,
                component="terrain.v2.layer_loader",
                code="terrain.v2.layer_fallback",
                message=(
                    f"v2 {layer.kind} layer {layer.layer_id!r} was dropped "
                    "after an explicitly permitted source fallback"
                ),
                details={
                    "layer_id": layer.layer_id,
                    "kind": layer.kind,
                    "access": layer.access,
                },
            )
            _active_log.warning(
                "v2 layer loader: %r could not be loaded (%s: %s); "
                "dropping with degraded health %s",
                layer.layer_id,
                type(exc).__name__,
                exc,
                health_event.code,
            )
            last_errors[layer.layer_id] = exc
            health_events[layer.layer_id] = health_event
            return None

        dem = _require_dem(
            loaded_dem,
            source=f"layer {layer.layer_id!r}",
            expected_ndim=3 if layer.kind == "color" else 2,
        )
        if layer.kind == "color":
            # Apply one shape contract after either the local or remote
            # reader.  Remote `(H, W, 1)` products must not bypass the
            # monochrome-to-RGB expansion used by local reads.
            dem = _normalize_color_dem(dem, layer_id=layer.layer_id)

        # Normalize raw masks using the layer encoding. Fusion consumes
        # {0, 1, NaN}; absent encoding preserves raw values.
        if layer.kind == "mask" and layer.mask_encoding is not None:
            dem = dataclasses.replace(
                dem,
                heights=decode_mask_dn(dem.heights, layer.mask_encoding),
                nodata=float("nan"),
            )

        return dem

    return _LayerLoader(
        load=_load,
        last_errors=last_errors,
        health_events=health_events,
    )


def bake_v2_fused(
    spec: TerrainSpec,
    *,
    plan: CompositionPlan,
    catalog: SourceCatalog,
    ortho_path: Optional[Path],
    deshade: bool = False,
    loader_factory: Callable[[], "_LayerLoader"] | None = None,
    logger: logging.Logger | None = None,
) -> tuple[
    FusionResult,
    PatchGrid,
    str,
    float,
    tuple[float, float],
    Optional[Path],
]:
    """Fuse the full resolved stack and return data needed by the existing bake and v2
    artifact flow. Return `(fusion_result, grid, dem_url, patch_size, patch_center,
    ortho_path)` in that order. The result has a DEM or raises. Use plan
    center in plan CRS for georeferencing; fusion warps every layer onto that grid.
    Reject base CRS-header mismatch before warp: coverage checks cannot detect an
    in-bounds false-easting shift. Base availability uses its own warped finite
    coverage, not post-blend provenance, since a higher-priority inset may consume
    all blend weight. Pass the effective deshade flag through to fusion. Missing or
    mismatched bases raise because v2 has no v1 fallback.
    """
    import dataclasses

    _active_log = logger or _log
    from srb.terrain.catalog.loader import layers_by_id
    from srb.terrain.errors import (
        DeclaredBaseCrsMismatchError,
        DeclaredBaseUnavailableError,
        DeclaredLayerUnknownError,
    )
    from srb.terrain.fusion.engine import fuse
    from srb.terrain.fusion.grid import grid_from_plan

    by_id = layers_by_id(catalog)
    base_contribution = plan.dem_stack[0]
    try:
        base_layer = by_id[base_contribution.layer_id]
    except KeyError:
        # A caller-contract violation (plan resolved against a DIFFERENT
        # catalog than the one handed to this function), not a runtime
        # degradation -- mirrors `fusion.engine._lookup_layer`'s identical
        # handling of the identical condition.
        raise DeclaredLayerUnknownError(base_contribution.layer_id) from None

    grid = grid_from_plan(plan)
    load_layer = (loader_factory or _make_layer_loader)()
    with stage_timer(_active_log, "fusion"):
        fusion_result = fuse(plan, catalog, load_layer=load_layer, deshade=deshade)

    # Preserve resolve-time warnings in logs and metadata; fuse() starts
    # with an empty warning list.
    health_warnings = tuple(
        TerrainFallbackWarning(
            "fuse: "
            f"{event.details['kind']} layer {layer_id!r} was dropped; "
            "structured degraded health: "
            f"{json.dumps(event.to_mapping(), sort_keys=True, separators=(',', ':'))}",
            event,
        )
        for layer_id, event in load_layer.health_events.items()
    )
    fusion_result = dataclasses.replace(
        fusion_result,
        warnings=(*plan.warnings, *fusion_result.warnings, *health_warnings),
    )

    # Log fusion warnings here too, including fatal-base warnings.
    for warning in fusion_result.warnings:
        _active_log.warning("%s", warning)

    if fusion_result.dem is None:
        if fusion_result.base_crs_mismatch is not None:
            # The base's own raster
            # CRS header disagreed with `plan.crs`, caught in
            # `fusion.engine._fuse_dem` before any warp happened -- see
            # `DeclaredBaseCrsMismatchError`'s own docstring.
            dem_crs, plan_crs = fusion_result.base_crs_mismatch
            raise DeclaredBaseCrsMismatchError(
                base_layer.layer_id,
                site=spec.cache_site,
                dem_crs=dem_crs,
                plan_crs=plan_crs,
            )
        reason = load_layer.last_errors.get(base_layer.layer_id)
        raise DeclaredBaseUnavailableError(
            base_layer.layer_id,
            site=spec.cache_site,
            reason=(
                str(reason)
                if reason is not None
                # No `last_errors`
                # entry for the base means the loader succeeded --
                # `fusion.engine._fuse_dem`'s only OTHER fatal-base
                # reason once the CRS matched is a warped array with
                # zero finite coverage (measured there, not from
                # post-blend "won" pixels -- see this function's own
                # docstring).
                else (
                    "the DEM base loaded and matched the plan's CRS, but "
                    "its warped contribution to the fused patch is "
                    "entirely nodata (0% finite coverage after warp) -- "
                    "check the layer's declared footprint against its "
                    "raster header"
                )
            ),
        )

    effective_patch_size_m = plan.request.size_m
    effective_patch_center = plan.center_xy_m
    ortho_for_bake = ortho_path
    # Attribution overrides apply only to v1. V2 aggregates layer attribution;
    # warn if a caller supplies an override that cannot apply to a fused bake.
    if spec.attribution:
        _active_log.warning(
            "spec.attribution=%r is ignored on a v2 (fused) bake for site "
            "%r; ATTRIBUTION.txt and meta['attribution'] are always the "
            "aggregate of every contributing catalog layer's own "
            "license/attribution, never a caller override",
            spec.attribution,
            spec.cache_site,
        )
    dem_url_for_meta = base_layer.urls[0]

    return (
        fusion_result,
        grid,
        dem_url_for_meta,
        effective_patch_size_m,
        effective_patch_center,
        ortho_for_bake,
    )
