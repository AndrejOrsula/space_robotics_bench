"""Network-free preflight for terrain manifest and cache state."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path

from srb.terrain import cache as cache_mod
from srb.terrain import v2_bake as v2_bake_mod
from srb.terrain.dependencies import missing_bake_dependencies
from srb.terrain.errors import (
    ManifestInvalidError,
    MissingTerrainDependencyError,
    TerrainNotBakedError,
)
from srb.terrain.manifest_loader import get_site
from srb.terrain.materials import MATERIALS_VERSION
from srb.terrain.materials.presets import get_default_preset, get_preset
from srb.terrain.schema import SiteEntry
from srb.terrain.source_health import (
    TerrainSourceIssue,
    TerrainSourceIssueKind,
    classify_terrain_source_exception,
)
from srb.terrain.sources.acquisition import plan_curated_site_sources
from srb.terrain.spec import Body, Lod, TerrainSpec


class TerrainAvailabilityStatus(str, Enum):
    READY = "ready"
    NEEDS_DOWNLOAD = "needs_download"
    NEEDS_BAKE = "needs_bake"
    MISSING_DEPENDENCY = "missing_dependency"
    KNOWN_BAD_MANIFEST = "known_bad_manifest"


@dataclass(frozen=True, slots=True)
class TerrainAvailability:
    spec: TerrainSpec
    status: TerrainAvailabilityStatus
    baked_path: Path | None
    issue: TerrainSourceIssue | None
    raw_dem_path: Path | None = None
    expected_baked_path: Path | None = None
    notes: tuple[str, ...] = ()

    @property
    def should_attempt_bake(self) -> bool:
        return self.status in (
            TerrainAvailabilityStatus.NEEDS_DOWNLOAD,
            TerrainAvailabilityStatus.NEEDS_BAKE,
        )


def assess_terrain_site(
    site: str,
    *,
    body: Body | None = None,
    patch: str = "default",
    lod: Lod = "medium",
    seed: int = 0,
    patch_size_m: float | None = None,
    patch_center_xy_m: tuple[float, float] | None = None,
    use_ortho: bool = True,
    preset_name: str | None = None,
    deshade: bool = False,
) -> TerrainAvailability:
    """Preflight a curated site without fetching sources.

    ``deshade`` is passed into the spec so assessment uses the bake's cache key.
    """
    site_entry = get_site(site)
    if body is not None and site_entry.body != body:
        raise ValueError(
            f"terrain site {site!r} belongs to body {site_entry.body!r}, not {body!r}"
        )
    # Reject unsupported v1 requests before reporting a remediation command.
    if (deshade or site_entry.deshade) and site_entry.sources.dem is not None:
        from srb.terrain.errors import DeshadeUnsupportedError

        raise DeshadeUnsupportedError(
            f"site {site!r} is a v1 single-DEM site; --deshade requires a "
            "fused (v2) site or a --lat/--lon/--size-m region"
        )
    spec = TerrainSpec(
        body=site_entry.body,
        site=site,
        patch=patch,
        patch_size_m=patch_size_m,
        patch_center_xy_m=patch_center_xy_m,
        lod=lod,
        seed=seed,
        use_ortho=use_ortho,
        deshade=deshade,
    )
    availability = assess_terrain_spec(
        spec,
        site_entry=site_entry,
        preset_name=preset_name,
    )
    note = _ephemeris_kernel_note(site_entry)
    if note is not None:
        availability = replace(availability, notes=(*availability.notes, note))
    return availability


def _ephemeris_kernel_note(site_entry: SiteEntry) -> str | None:
    """A non-blocking note on SPICE kernel status for ephemeris-lit sites.

    Returns ``None`` for any non-ephemeris site. Kernel absence is NOT a hard
    failure (the runtime falls back to the body-default static preset); this
    only surfaces the status so ``srb asset list|doctor`` can report it. The
    probe never imports ``spiceypy`` and never downloads.
    """
    from srb.terrain.lights.ephemeris_bridge import (
        EPHEMERIS_LIGHTING,
        _body_default_preset_name,
        _kernels_present,
    )

    if getattr(site_entry, "lighting", None) != EPHEMERIS_LIGHTING:
        return None
    epoch = getattr(site_entry, "epoch", None) or "?"
    default_preset = _body_default_preset_name(site_entry.body)
    if _kernels_present():
        return f"lighting=ephemeris @ {epoch}; SPICE kernels present"
    return (
        f"lighting=ephemeris @ {epoch}; SPICE kernels missing -- runtime uses "
        f"static '{default_preset}' lighting fallback (run 'srb ephemeris download')"
    )


def _ready_baked_path(paths: cache_mod.CachePaths) -> Path | None:
    """Return one stable terrain member path from a validated snapshot."""
    generation = cache_mod.published_generation(paths)
    if generation is None:
        return None
    return paths.with_generation(generation).terrain_usd


def assess_terrain_spec(
    spec: TerrainSpec,
    *,
    site_entry: SiteEntry | None = None,
    preset_name: str | None = None,
) -> TerrainAvailability:
    """Preflight a terrain spec using only manifest and cache state."""
    if spec.dem_path is not None:
        paths = _cache_paths(spec, preset_name=preset_name)
        baked_path = _ready_baked_path(paths)
        if baked_path is not None:
            return TerrainAvailability(
                spec=spec,
                status=TerrainAvailabilityStatus.READY,
                baked_path=baked_path,
                issue=None,
                expected_baked_path=baked_path,
            )
        missing_dependencies = missing_bake_dependencies()
        if missing_dependencies:
            return TerrainAvailability(
                spec=spec,
                status=TerrainAvailabilityStatus.MISSING_DEPENDENCY,
                baked_path=None,
                issue=classify_terrain_source_exception(
                    MissingTerrainDependencyError(missing_dependencies)
                ),
                expected_baked_path=paths.terrain_usd,
            )
        return TerrainAvailability(
            spec=spec,
            status=TerrainAvailabilityStatus.NEEDS_BAKE,
            baked_path=None,
            issue=classify_terrain_source_exception(
                TerrainNotBakedError(
                    site=spec.cache_site,
                    patch=spec.patch,
                    lod=spec.lod,
                )
            ),
            expected_baked_path=paths.terrain_usd,
        )

    if spec.region is not None:
        # Regions have no manifest entry; use their resolved v2 cache key.
        return _assess_v2_region_spec(spec, preset_name=preset_name)

    if spec.site is None:
        raise ValueError("TerrainSpec needs either site or dem_path or region")
    if site_entry is None:
        site_entry = get_site(spec.site)

    if site_entry.sources.dem is None:
        # Declared-stack sites have no single-source DEM path or placeholder
        # hash; assess them using the resolved v2 cache identity.
        return _assess_v2_site_spec(
            spec, site_entry=site_entry, preset_name=preset_name
        )

    sources = plan_curated_site_sources(
        spec.site,
        site_entry,
        use_ortho=spec.use_ortho,
    )
    placeholder_labels = sources.placeholder_labels
    if placeholder_labels:
        label = ", ".join(placeholder_labels)
        issue = TerrainSourceIssue(
            kind=TerrainSourceIssueKind.MANIFEST_PLACEHOLDER,
            summary=f"terrain manifest has placeholder sha256 for {label}",
            remediation=(
                "Backfill the terrain manifest sha256 or mirror the source "
                "before selecting this real terrain site."
            ),
            recoverable=False,
            exception=ManifestInvalidError(
                f"terrain manifest has placeholder sha256 for {label}"
            ),
        )
        return TerrainAvailability(
            spec=spec,
            status=TerrainAvailabilityStatus.KNOWN_BAD_MANIFEST,
            baked_path=None,
            issue=issue,
            raw_dem_path=sources.raw_dem_path,
        )

    paths = _cache_paths(
        spec,
        site_entry=site_entry,
        preset_name=preset_name,
    )
    baked_path = _ready_baked_path(paths)
    if baked_path is not None:
        return TerrainAvailability(
            spec=spec,
            status=TerrainAvailabilityStatus.READY,
            baked_path=baked_path,
            issue=None,
            raw_dem_path=sources.raw_dem_path,
            expected_baked_path=baked_path,
        )

    missing_dependencies = missing_bake_dependencies()
    if missing_dependencies:
        return TerrainAvailability(
            spec=spec,
            status=TerrainAvailabilityStatus.MISSING_DEPENDENCY,
            baked_path=None,
            issue=classify_terrain_source_exception(
                MissingTerrainDependencyError(missing_dependencies)
            ),
            raw_dem_path=sources.raw_dem_path,
            expected_baked_path=paths.terrain_usd,
        )

    raw_dem_path = sources.raw_dem_path
    status = (
        TerrainAvailabilityStatus.NEEDS_BAKE
        if raw_dem_path.exists()
        else TerrainAvailabilityStatus.NEEDS_DOWNLOAD
    )
    return TerrainAvailability(
        spec=spec,
        status=status,
        baked_path=None,
        issue=classify_terrain_source_exception(
            TerrainNotBakedError(
                site=spec.cache_site,
                patch=spec.patch,
                lod=spec.lod,
            )
        ),
        raw_dem_path=raw_dem_path,
        expected_baked_path=paths.terrain_usd,
    )


def _assess_v2_site_spec(
    spec: TerrainSpec, *, site_entry: SiteEntry, preset_name: str | None
) -> TerrainAvailability:
    """Assess a declared-stack site using its v2 cache identity.

    Resolution is network-free. Stack and manifest errors propagate; this
    preflight does not model per-layer downloads or placeholder hashes.
    """
    paths = _cache_paths(spec, site_entry=site_entry, preset_name=preset_name)
    return _assess_v2_paths(spec, paths)


def _assess_v2_region_spec(
    spec: TerrainSpec, *, preset_name: str | None
) -> TerrainAvailability:
    """Assess an arbitrary region using its resolved v2 cache identity.

    Resolution is network-free. Unresolvable regions propagate their
    resolution error to the caller.
    """
    paths = _cache_paths(spec, preset_name=preset_name)
    return _assess_v2_paths(spec, paths)


def _assess_v2_paths(
    spec: TerrainSpec, paths: cache_mod.CachePaths
) -> TerrainAvailability:
    """Classify a v2 request after resolving its cache paths."""
    baked_path = _ready_baked_path(paths)
    if baked_path is not None:
        return TerrainAvailability(
            spec=spec,
            status=TerrainAvailabilityStatus.READY,
            baked_path=baked_path,
            issue=None,
            expected_baked_path=baked_path,
        )

    missing_dependencies = missing_bake_dependencies()
    if missing_dependencies:
        return TerrainAvailability(
            spec=spec,
            status=TerrainAvailabilityStatus.MISSING_DEPENDENCY,
            baked_path=None,
            issue=classify_terrain_source_exception(
                MissingTerrainDependencyError(missing_dependencies)
            ),
            expected_baked_path=paths.terrain_usd,
        )

    return TerrainAvailability(
        spec=spec,
        status=TerrainAvailabilityStatus.NEEDS_BAKE,
        baked_path=None,
        issue=classify_terrain_source_exception(
            TerrainNotBakedError(
                site=spec.cache_site,
                patch=spec.patch,
                lod=spec.lod,
            )
        ),
        expected_baked_path=paths.terrain_usd,
    )


def _cache_paths(
    spec: TerrainSpec,
    *,
    site_entry: SiteEntry | None = None,
    preset_name: str | None,
) -> cache_mod.CachePaths:
    materials_version = (
        spec.materials_version
        if spec.materials_version is not None
        else MATERIALS_VERSION
    )
    effective_preset = (
        get_preset(preset_name)
        if preset_name is not None
        else get_default_preset(spec.body)
    )
    if spec.region is not None:
        # Arbitrary-region requests use the same cohesive resolution/cache
        # interface as bake_or_load. `RegionUnresolvedError` propagates from
        # that interface; availability performs no network or raster I/O.
        v2_bake_plan = v2_bake_mod.resolve_v2_bake_plan(
            spec,
            materials_version=materials_version,
            preset_name=effective_preset.name,
        )
        assert v2_bake_plan is not None  # spec.region is not None
        return v2_bake_plan.cache_paths
    manifest_dem_sha: str | None = None
    if spec.dem_path is None:
        if site_entry is None:
            assert spec.site is not None
            site_entry = get_site(spec.site)
        if site_entry.sources.dem is None:
            # A declared-stack site has no v1 `sources.dem` to hash. Use the
            # same v2 resolution/cache-plan interface as bake_or_load so a
            # ready v2 artifact is assessed with its real cache identity.
            v2_bake_plan = v2_bake_mod.resolve_v2_bake_plan(
                spec,
                materials_version=materials_version,
                preset_name=effective_preset.name,
            )
            assert v2_bake_plan is not None  # just confirmed sources.dem is None
            return v2_bake_plan.cache_paths
        manifest_dem_sha = site_entry.sources.dem.sha256

    # Keep the v1 key's manifest/request deshade behavior without importing
    # the v2 implementation seam. `site_entry` is populated above for every
    # curated v1 site; local DEM specs use the request flag alone.
    effective_deshade = bool(
        spec.deshade or (site_entry.deshade if site_entry is not None else False)
    )

    paths = cache_mod.cache_paths(
        spec,
        materials_version=materials_version,
        preset_name=effective_preset.name,
        manifest_dem_sha256=manifest_dem_sha,
        deshade_override=effective_deshade,
        canonicalize_geometry=(
            spec.dem_path is None
            and site_entry is not None
            and site_entry.sources.dem is not None
        ),
    )
    if (
        site_entry is not None
        and site_entry.sources.dem is not None
        and spec.dem_path is None
    ):
        return cache_mod.with_named_single_source_identity(paths, spec, site_entry)
    return paths


__all__ = [
    "TerrainAvailability",
    "TerrainAvailabilityStatus",
    "assess_terrain_site",
    "assess_terrain_spec",
]
