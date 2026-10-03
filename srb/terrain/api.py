"""Bake or load terrain assets from local DEMs, curated sites, or regions."""

from __future__ import annotations

import datetime as dt
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Optional, TypedDict

from srb.terrain import cache as cache_mod
from srb.terrain import v2_bake as v2_bake_mod
from srb.terrain.errors import TerrainNotBakedError
from srb.terrain.fusion.provenance import (
    FusionMetadata,
    format_fusion_attribution,
    fusion_attributions,
    validate_fusion_metadata,
)
from srb.terrain.ingest.crop import crop_to_patch
from srb.terrain.ingest.dem import load_dem
from srb.terrain.ingest.ortho import load_ortho
from srb.terrain.log import get_logger, stage_timer
from srb.terrain.manifest_loader import get_site
from srb.terrain.materials import MATERIALS_VERSION
from srb.terrain.materials.baker import MeshSurface, bake_textures
from srb.terrain.materials.presets import get_default_preset, get_preset
from srb.terrain.materials.usd_writer import write_material_usda
from srb.terrain.mesh.builder import LOD_FACTORS, center_z_origin_m, heightmap_to_mesh
from srb.terrain.mesh.usd_writer import write_terrain_usd
from srb.terrain.schema import SiteEntry
from srb.terrain.sources.acquisition import fetch_curated_site_sources
from srb.terrain.spec import TerrainSpec

if TYPE_CHECKING:
    import trimesh

    from srb.terrain.augmentation.presets import AugmentPreset
    from srb.terrain.catalog.resolve import CompositionPlan
    from srb.terrain.catalog_schema import SourceCatalog
    from srb.terrain.fusion.engine import FusionResult
    from srb.terrain.fusion.grid import PatchGrid
    from srb.terrain.ingest.dem import Dem
    from srb.terrain.schema import SourceEntry, StackRef
    from srb.terrain.sources.acquisition import ProviderGetter
    from srb.terrain.sources.vsicurl import Opener

_log = get_logger("api")


class _GeorefMetadata(TypedDict):
    crs: str
    patch_center_xy_m: list[float]
    patch_center_latlon: list[float]
    north_convergence_deg: float
    grid_axes: str


class _AugmentationCounts(TypedDict):
    craters_requested: int
    craters_kept: int
    rocks_requested: int
    rocks_kept: int


class _AugmentationMetadata(TypedDict):
    file: str
    schema: str
    db_version: int
    counts: _AugmentationCounts


def _require_path(value: object, *, source: str) -> Path:

    if not isinstance(value, Path):
        raise TypeError(f"{source} returned {type(value).__name__}, expected Path")
    return value


def _require_string(value: object, *, source: str) -> str:

    if not isinstance(value, str):
        raise TypeError(f"{source} returned {type(value).__name__}, expected str")
    return value


def _require_float(value: object, *, source: str) -> float:

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{source} returned {type(value).__name__}, expected float")
    return float(value)


def _require_v1_dem_source(site_entry: SiteEntry, *, site: str | None) -> SourceEntry:

    source = site_entry.sources.dem
    if source is None:
        from srb.terrain.errors import ManifestInvalidError

        raise ManifestInvalidError(
            f"site {site!r} has no single-source DEM for the v1 bake path"
        )
    return source


def _require_string_mapping(value: object, *, source: str) -> Mapping[str, object]:

    if not isinstance(value, dict):
        raise TypeError(f"{source} returned {type(value).__name__}, expected dict")
    result: dict[str, object] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise TypeError(f"{source} returned a non-string metadata key")
        result[key] = item
    return result


def _metadata_value(mapping: Mapping[str, object], key: str, *, source: str) -> object:

    if key not in mapping:
        raise TypeError(f"{source} is missing required field {key!r}")
    return mapping[key]


def _require_int(value: object, *, source: str) -> int:

    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{source} returned {type(value).__name__}, expected int")
    return value


def _require_augmentation_counts(value: object, *, source: str) -> _AugmentationCounts:

    mapping = _require_string_mapping(value, source=source)
    return {
        "craters_requested": _require_int(
            _metadata_value(mapping, "craters_requested", source=source),
            source=f"{source}.craters_requested",
        ),
        "craters_kept": _require_int(
            _metadata_value(mapping, "craters_kept", source=source),
            source=f"{source}.craters_kept",
        ),
        "rocks_requested": _require_int(
            _metadata_value(mapping, "rocks_requested", source=source),
            source=f"{source}.rocks_requested",
        ),
        "rocks_kept": _require_int(
            _metadata_value(mapping, "rocks_kept", source=source),
            source=f"{source}.rocks_kept",
        ),
    }


def bake_or_load(
    spec: TerrainSpec,
    *,
    auto_fetch: bool = True,
    ortho_path: Optional[Path] = None,
    preset_name: Optional[str] = None,
) -> Path:
    """Return the cached bake, or create one if ``auto_fetch`` is true.

    Raises ``TerrainNotBakedError`` on a cache miss when ``auto_fetch`` is
    false. ``ortho_path`` supplies a pre-fetched ortho; ``preset_name``
    overrides the body's default preset.
    """
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

    # Resolve stack identities before cache lookup; v2 keys depend on them,
    # and resolution errors also apply on hits.
    v2_bake_plan = v2_bake_mod.resolve_v2_bake_plan(
        spec,
        materials_version=materials_version,
        preset_name=effective_preset.name,
        site_getter=get_site,
    )

    if v2_bake_plan is not None:
        paths = v2_bake_plan.cache_paths
        effective_deshade = v2_bake_plan.effective_deshade
    else:
        manifest_dem_sha: Optional[str] = None
        site_entry: SiteEntry | None = None
        if spec.dem_path is None and spec.site is not None:
            site_entry = get_site(spec.site)
            source_entry = _require_v1_dem_source(site_entry, site=spec.site)
            manifest_dem_sha = source_entry.sha256
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
            spec.dem_path is None
            and site_entry is not None
            and site_entry.sources.dem is not None
        ):
            paths = cache_mod.with_named_single_source_identity(
                paths,
                spec,
                site_entry,
                ortho_path=ortho_path,
            )

    generation = cache_mod.published_generation(paths)
    if generation is not None:
        return _require_path(
            paths.with_generation(generation).terrain_usd,
            source="cache generation terrain_usd",
        )

    if not auto_fetch:
        raise TerrainNotBakedError(site=spec.cache_site, patch=spec.patch, lod=spec.lod)

    _bake(
        spec=spec,
        paths=paths,
        ortho_path=ortho_path,
        preset_name=preset_name,
        materials_version=materials_version,
        v2_bake_plan=v2_bake_plan,
    )

    generation = cache_mod.published_generation(paths)
    return (
        _require_path(
            paths.with_generation(generation).terrain_usd,
            source="cache generation terrain_usd",
        )
        if generation is not None
        else _require_path(paths.terrain_usd, source="cache terrain_usd")
    )


MAX_WINDOW_PIXELS = v2_bake_mod.MAX_WINDOW_PIXELS
_LOADER_PAD_M = v2_bake_mod._LOADER_PAD_M
_INGEST_RESAMPLING_BY_KIND = v2_bake_mod._INGEST_RESAMPLING_BY_KIND


def _effective_deshade(spec: TerrainSpec) -> bool:
    return v2_bake_mod._effective_deshade(spec, site_getter=get_site)


# Keep these delegates for historical private-name compatibility;
# production code uses V2BakePlan.
def _resolve_v2_region(
    spec: TerrainSpec,
) -> Optional[tuple[CompositionPlan, SourceCatalog]]:
    return v2_bake_mod._resolve_v2_region(spec)


def _resolve_v2_site(
    spec: TerrainSpec,
) -> Optional[tuple[CompositionPlan, SourceCatalog]]:
    return v2_bake_mod._resolve_v2_site(spec, site_getter=get_site)


def _raise_if_declared_base_unresolvable(
    plan: CompositionPlan, *, dem_refs: Sequence[StackRef], site: str
) -> None:
    v2_bake_mod._raise_if_declared_base_unresolvable(plan, dem_refs=dem_refs, site=site)


def _v2_stack_hashes(plan: CompositionPlan, catalog: SourceCatalog) -> tuple[str, str]:
    return v2_bake_mod._v2_stack_hashes(plan, catalog)


def _v2_cache_paths(
    spec: TerrainSpec,
    *,
    plan: CompositionPlan,
    catalog: SourceCatalog,
    materials_version: int,
    preset_name: Optional[str],
) -> cache_mod.CachePaths:
    return v2_bake_mod._v2_cache_paths(
        spec,
        plan=plan,
        catalog=catalog,
        materials_version=materials_version,
        preset_name=preset_name,
        deshade_resolver=_effective_deshade,
    )


def _make_layer_loader(
    *,
    root: Optional[Path] = None,
    provider_getter: Optional["ProviderGetter"] = None,
    env: Optional[Mapping[str, str]] = None,
    opener: Optional["Opener"] = None,
) -> "v2_bake_mod._LayerLoader":
    return v2_bake_mod._make_layer_loader(
        root=root,
        provider_getter=provider_getter,
        env=env,
        opener=opener,
        logger=_log,
        max_window_pixels=MAX_WINDOW_PIXELS,
        pad_m=_LOADER_PAD_M,
        resampling_by_kind=_INGEST_RESAMPLING_BY_KIND,
    )


def _native_window_gsd_m(
    *, native_gsd_m: float, window_size_m: float, max_window_pixels: int
) -> tuple[Optional[float], bool]:
    return v2_bake_mod._native_window_gsd_m(
        native_gsd_m=native_gsd_m,
        window_size_m=window_size_m,
        max_window_pixels=max_window_pixels,
    )


def _bake_v2_fused(
    spec: TerrainSpec,
    *,
    plan: CompositionPlan,
    catalog: SourceCatalog,
    ortho_path: Optional[Path],
    deshade: bool = False,
) -> tuple[
    FusionResult,
    PatchGrid,
    str,
    float,
    tuple[float, float],
    Optional[Path],
]:
    return v2_bake_mod.bake_v2_fused(
        spec,
        plan=plan,
        catalog=catalog,
        ortho_path=ortho_path,
        deshade=deshade,
        loader_factory=_make_layer_loader,
        logger=_log,
    )


def _resolve_curated_site(
    spec: TerrainSpec,
) -> tuple[Path, Optional[Path], "SiteEntry"]:

    assert spec.site is not None
    site_entry = get_site(spec.site)
    dem_path, ortho_path = fetch_curated_site_sources(
        spec.site,
        site_entry,
        use_ortho=spec.use_ortho,
    )

    return dem_path, ortho_path, site_entry


GRID_AXES = "x=grid-east, y=grid-north (projected)"


def _compute_georef(
    dem_crs: Optional[str],
    patch_center_xy_m: Optional[tuple[float, float]],
    body: str,
) -> _GeorefMetadata | None:

    if not dem_crs or patch_center_xy_m is None:
        return None

    from srb.terrain.ingest import planetary_crs

    norm = planetary_crs.normalize_crs(dem_crs, body=body)
    if norm.matched_by in ("authority", "parameters") and norm.canonical_id:
        operative_crs = _require_string(
            norm.canonical_id, source="normalized planetary CRS"
        )
    else:
        operative_crs = dem_crs

    cx, cy = float(patch_center_xy_m[0]), float(patch_center_xy_m[1])
    lat, lon = planetary_crs.xy_to_latlon(cx, cy, operative_crs)
    convergence = planetary_crs.north_convergence_deg(lat, lon, operative_crs)

    return {
        "crs": operative_crs,
        "patch_center_xy_m": [cx, cy],
        "patch_center_latlon": [
            _require_float(lat, source="planetary CRS latitude"),
            _require_float(lon, source="planetary CRS longitude"),
        ],
        "north_convergence_deg": _require_float(
            convergence, source="planetary CRS convergence"
        ),
        "grid_axes": GRID_AXES,
    }


BYTES_VERIFIED_KEY: Literal["bytes_verified"] = "bytes_verified"
"""Key marking whether a fusion layer's bytes were verified during the bake."""


def _annotate_bytes_verified(
    fusion_meta: FusionMetadata, plan: "CompositionPlan"
) -> FusionMetadata:

    access_by_layer_id: dict[str, str] = {
        contribution.layer_id: contribution.access
        for stack in (
            plan.dem_stack,
            plan.color_stack,
            plan.color_stack_psr,
            plan.mask_layers,
            plan.quality_layers,
        )
        for contribution in stack
    }
    for records in (
        fusion_meta["dem_layers"],
        fusion_meta["color_layers"],
        fusion_meta["contributors"],
    ):
        for record in records:
            access = access_by_layer_id.get(record["layer_id"])
            record[BYTES_VERIFIED_KEY] = access is not None
    return fusion_meta


def _write_fusion_artifacts(
    paths: cache_mod.CachePaths,
    *,
    staging: Path,
    fusion_result: FusionResult,
    grid: PatchGrid,
    plan: "CompositionPlan",
) -> FusionMetadata:
    """Write fusion rasters and matching metadata into staging."""
    from srb.terrain.fusion import FUSION_VERSION
    from srb.terrain.fusion.provenance import (
        fusion_meta_block,
        write_provenance_tif,
        write_uncertainty_contract_tags,
        write_uncertainty_tif,
    )

    if fusion_result.provenance is None:
        raise AssertionError(
            "_write_fusion_artifacts: fusion_result.provenance is None but "
            "fusion_result.dem is not -- fuse()'s own contract is broken"
        )
    write_provenance_tif(staging / "provenance.tif", fusion_result.provenance, grid)

    uncertainty_tif_name = None
    if fusion_result.uncertainty is not None:
        write_uncertainty_tif(
            staging / "uncertainty.tif", fusion_result.uncertainty, grid
        )
        write_uncertainty_contract_tags(
            staging / "uncertainty.tif",
            source_layer_ids=tuple(
                contributor.layer_id
                for contributor in fusion_result.contributors
                if contributor.kind == "quality"
            ),
        )
        uncertainty_tif_name = "uncertainty.tif"

    fusion_metadata = validate_fusion_metadata(
        fusion_meta_block(
            grid=grid,
            dem_layers=fusion_result.dem_layers,
            color_layers=fusion_result.color_layers,
            fusion_version=FUSION_VERSION,
            provenance_tif_name="provenance.tif",
            uncertainty_tif_name=uncertainty_tif_name,
            warnings=fusion_result.warnings,
            contributors=fusion_result.contributors,
        ),
        source="fusion_meta_block",
    )
    return validate_fusion_metadata(
        _annotate_bytes_verified(fusion_metadata, plan),
        source="annotated fusion metadata",
    )


def _format_v2_attribution(metadata: FusionMetadata) -> str:

    return format_fusion_attribution(metadata)


def _effective_augment_preset(spec: TerrainSpec) -> "AugmentPreset":
    from srb.terrain.companion_scene import (
        resolve_effective_augment_preset,
        spec_augment_preset,
    )

    return resolve_effective_augment_preset(spec_augment_preset(spec), spec.body)


def _requires_augmentation_db(spec: TerrainSpec) -> bool:
    preset = _effective_augment_preset(spec)
    return preset.rock_density_per_m2 > 0 or preset.crater_density_per_m2 > 0


def _bake(
    *,
    spec: TerrainSpec,
    paths: cache_mod.CachePaths,
    ortho_path: Optional[Path],
    preset_name: Optional[str],
    materials_version: int,
    plan: Optional[CompositionPlan] = None,
    catalog: Optional[SourceCatalog] = None,
    v2_bake_plan: v2_bake_mod.V2BakePlan | None = None,
) -> None:
    if v2_bake_plan is not None:
        plan = v2_bake_plan.composition_plan
        catalog = v2_bake_plan.catalog
    _validate_deshade_request(spec)
    required_members: tuple[str, ...] = cache_mod.BAKE_REQUIRED_MEMBERS
    if plan is not None:
        required_members = (*required_members, "provenance.tif")
    if _requires_augmentation_db(spec):
        required_members = (*required_members, "augmentation.npz")
    cache_mod.publish_baked_generation(
        paths,
        lambda staging: _bake_into(
            spec=spec,
            paths=paths,
            staging=staging,
            ortho_path=ortho_path,
            preset_name=preset_name,
            materials_version=materials_version,
            plan=plan,
            catalog=catalog,
            v2_bake_plan=v2_bake_plan,
        ),
        required_members=required_members,
        reuse_if_ready=True,
    )


def _validate_deshade_request(spec: TerrainSpec) -> None:

    if spec.dem_path is not None:
        if spec.deshade:
            from srb.terrain.errors import DeshadeUnsupportedError

            raise DeshadeUnsupportedError(
                "deshade=True is not supported for a local dem_path bake "
                f"(site_name={spec.site_name!r}); the fusion engine that "
                "implements deshading only runs for a v2 declared-stack "
                "manifest ('dem_stack'/'color_stack', version: 2) or an "
                "arbitrary region request (lat_deg/lon_deg/size_m)"
            )
        return

    if spec.site is None:
        return
    site_entry = get_site(spec.site)
    if site_entry.sources.dem is None or not (site_entry.deshade or spec.deshade):
        return

    from srb.terrain.errors import DeshadeUnsupportedError

    raise DeshadeUnsupportedError(
        "deshade=True is not supported for the v1 (non-fused) bake "
        f"path (site={spec.site!r}); the fusion engine that implements "
        "deshading only runs for a v2 declared-stack manifest "
        "('dem_stack'/'color_stack', version: 2) or an arbitrary region "
        "request (lat_deg/lon_deg/size_m)"
    )


def _mesh_surface_px(mesh: trimesh.Trimesh, dem: Dem) -> MeshSurface:
    """Map the mesh top surface back onto the DEM pixel grid."""
    import numpy as np

    rows, cols = dem.heights.shape[:2]
    gsd = float(dem.gsd_m)
    v = np.asarray(mesh.vertices, dtype=np.float64)
    f = np.asarray(mesh.faces, dtype=np.int64)
    col = v[:, 0] / gsd + 0.5 * (cols - 1)
    row = 0.5 * (rows - 1) - v[:, 1] / gsd
    z = v[:, 2] + center_z_origin_m(dem.heights)
    a, b, c = v[f[:, 0], :2], v[f[:, 1], :2], v[f[:, 2], :2]
    area2 = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (
        c[:, 0] - a[:, 0]
    )
    return MeshSurface(vertices=np.column_stack([col, row, z]), faces=f[area2 > 0.0])


def _bake_into(
    *,
    spec: TerrainSpec,
    paths: cache_mod.CachePaths,
    staging: Path,
    ortho_path: Optional[Path],
    preset_name: Optional[str],
    materials_version: int,
    plan: Optional[CompositionPlan] = None,
    catalog: Optional[SourceCatalog] = None,
    v2_bake_plan: v2_bake_mod.V2BakePlan | None = None,
) -> None:
    if v2_bake_plan is not None:
        plan = v2_bake_plan.composition_plan
        catalog = v2_bake_plan.catalog
    dem = None
    fusion_result: Optional[FusionResult] = None
    fusion_grid: Optional[PatchGrid] = None
    effective_patch_size_m: float | None
    effective_patch_center: tuple[float, float] | None

    if plan is not None:
        assert catalog is not None
        (
            fusion_result,
            fusion_grid,
            dem_url_for_meta,
            effective_patch_size_m,
            effective_patch_center,
            ortho_for_bake,
        ) = _bake_v2_fused(
            spec,
            plan=plan,
            catalog=catalog,
            ortho_path=ortho_path,
            deshade=(
                v2_bake_plan.effective_deshade
                if v2_bake_plan is not None
                else _effective_deshade(spec)
            ),
        )
        dem = fusion_result.dem
        assert dem is not None
        # V2 ATTRIBUTION.txt aggregates catalog layers from fusion metadata.
        attribution_for_bake = None
    elif spec.dem_path is None:
        dem_path, resolved_ortho, site_entry = _resolve_curated_site(spec)
        if site_entry.deshade or spec.deshade:
            from srb.terrain.errors import DeshadeUnsupportedError

            raise DeshadeUnsupportedError(
                f"deshade=True is not supported for the v1 (non-fused) bake "
                f"path (site={spec.site!r}); the fusion engine that implements "
                "deshading only runs for a v2 declared-stack manifest site "
                "('dem_stack'/'color_stack', version: 2) or an arbitrary "
                "region request (lat_deg/lon_deg/size_m)"
            )

        patch = site_entry.patches.get(spec.patch)
        if patch is None:
            from srb.terrain.errors import ManifestInvalidError

            raise ManifestInvalidError(
                f"site '{spec.site}' has no patch '{spec.patch}'"
            )
        effective_patch_size_m = (
            spec.patch_size_m if spec.patch_size_m is not None else patch.size_m
        )
        effective_patch_center = (
            spec.patch_center_xy_m
            if spec.patch_center_xy_m is not None
            else patch.center_xy_m
        )
        ortho_for_bake = ortho_path if ortho_path is not None else resolved_ortho
        dem_source = _require_v1_dem_source(site_entry, site=spec.site)
        attribution_for_bake = (
            spec.attribution if spec.attribution else dem_source.attribution
        )
        dem_url_for_meta = str(dem_path)
    else:
        if spec.deshade:
            from srb.terrain.errors import DeshadeUnsupportedError

            raise DeshadeUnsupportedError(
                "deshade=True is not supported for a local dem_path bake "
                f"(site_name={spec.site_name!r}); the fusion engine that "
                "implements deshading only runs for a v2 declared-stack "
                "manifest site ('dem_stack'/'color_stack', version: 2) or an "
                "arbitrary region request (lat_deg/lon_deg/size_m)"
            )
        dem_path = spec.dem_path
        ortho_for_bake = ortho_path
        effective_patch_size_m = spec.patch_size_m
        effective_patch_center = spec.patch_center_xy_m
        attribution_for_bake = spec.attribution
        dem_url_for_meta = str(dem_path)

    if dem is None:
        with stage_timer(_log, "ingest"):
            dem = load_dem(dem_path)

    if (
        plan is None
        and effective_patch_size_m is not None
        and effective_patch_center is None
    ):
        # A bare patch size crops around the DEM center.
        a, _, c, _, e, f = dem.transform_a
        rows, cols = dem.heights.shape[:2]
        effective_patch_center = (c + 0.5 * cols * a, f + 0.5 * rows * e)

    # Fused DEM already matches the requested grid; recropping resamples it again.
    if (
        plan is None
        and effective_patch_size_m is not None
        and effective_patch_center is not None
    ):
        with stage_timer(_log, "crop"):
            dem = crop_to_patch(
                dem,
                center_xy_m=effective_patch_center,
                size_m=effective_patch_size_m,
                target_gsd_m=None,
            )

    # Repair nodata once after fusion; fusion uses NaN to mean no measurement.
    import dataclasses

    import numpy as _np

    _finite_mask = _np.isfinite(dem.heights)
    if not _finite_mask.all():
        _finite = dem.heights[_finite_mask]
        _fill = float(_np.median(_finite)) if _finite.size else 0.0
        _n_bad = int((~_finite_mask).sum())
        dem = dataclasses.replace(
            dem,
            heights=_np.where(_finite_mask, dem.heights, _fill).astype(
                _np.float32, copy=False
            ),
        )
        _log.warning(
            "DEM patch had %d non-finite (nodata) cells; filled with %.3f m "
            "(median of finite cells, or 0 if none) before baking/meshing",
            _n_bad,
            _fill,
        )

    _mesh_lod_factor = 1 if plan is not None else LOD_FACTORS[spec.lod]
    _mesh_h_arr_for_origin = (
        dem.heights[::_mesh_lod_factor, ::_mesh_lod_factor]
        if _mesh_lod_factor > 1
        else dem.heights
    )
    _mesh_z_origin_m = center_z_origin_m(_mesh_h_arr_for_origin)

    augmentation_db = None
    from srb.terrain.augmentation.database import generate_augmentation_db

    augment_preset = _effective_augment_preset(spec)
    if _requires_augmentation_db(spec):
        effective_augment_patch_size_m = effective_patch_size_m
        if effective_augment_patch_size_m is None and spec.dem_path is not None:
            # Use the largest centered square that fits both DEM axes;
            # sampling clamps out-of-bounds coordinates.
            _dem_width_m = float(dem.heights.shape[1]) * float(dem.gsd_m)
            _dem_height_m = float(dem.heights.shape[0]) * float(dem.gsd_m)
            effective_augment_patch_size_m = min(_dem_width_m, _dem_height_m)

        augmentation_db = generate_augmentation_db(
            slug=spec.cache_site,
            body=spec.body,
            seed=spec.seed,
            patch_size_m=_require_float(
                effective_augment_patch_size_m, source="augmentation patch size"
            ),
            gsd_m=dem.gsd_m,
            preset=augment_preset,
            dem=dem.heights,
            mesh_z_origin_m=_mesh_z_origin_m,
        )

    # Stamp before mesh and materials so every output uses the same DEM.
    if (
        spec.augment_stamp
        and augmentation_db is not None
        and augmentation_db.craters.size
    ):
        from srb.terrain.augmentation.database import sample_height_m
        from srb.terrain.augmentation.stamping import stamp_craters

        stamped_heights = stamp_craters(
            dem.heights, craters=augmentation_db.craters, gsd_m=dem.gsd_m
        )
        stamped_db = augmentation_db
        if augmentation_db.rocks.size:
            stamped_rocks = augmentation_db.rocks.copy()

            # Match the mesh's exact LOD stride and centered Z origin
            # for augmentation heights.
            stamped_for_origin = (
                stamped_heights[::_mesh_lod_factor, ::_mesh_lod_factor]
                if _mesh_lod_factor > 1
                else stamped_heights
            )
            stamped_mesh_z_origin_m = center_z_origin_m(stamped_for_origin)
            stamped_rocks[:, 2] = (
                sample_height_m(
                    stamped_heights,
                    dem.gsd_m,
                    augmentation_db.rocks[:, 0],
                    augmentation_db.rocks[:, 1],
                )
                - stamped_mesh_z_origin_m
            )
            stamped_db = dataclasses.replace(augmentation_db, rocks=stamped_rocks)

        dem = dataclasses.replace(dem, heights=stamped_heights)
        augmentation_db = stamped_db

    ortho = None
    if spec.use_ortho:
        if ortho_for_bake is not None:
            # Explicit ortho_path overrides manifest and fused color.

            with stage_timer(_log, "ortho"):
                ortho = load_ortho(ortho_for_bake, target=dem)
        elif plan is not None:
            assert fusion_result is not None
            ortho = fusion_result.color_rgb
        else:
            _log.warning(
                "spec.use_ortho=True but no ortho_path provided; "
                "falling back to procedural albedo"
            )

    preset = (
        get_preset(preset_name)
        if preset_name is not None
        else get_default_preset(spec.body)
    )

    face_budget: Optional[int] = None
    texture_cap_px: Optional[int] = None
    ao_max_px: Optional[int] = None
    if plan is not None:
        from srb.terrain.fusion.budgets import AO_MAX_PX, FACE_BUDGET, TEXTURE_CAP

        face_budget = FACE_BUDGET[spec.lod]
        texture_cap_px = TEXTURE_CAP[spec.lod]
        ao_max_px = AO_MAX_PX

    # Build mesh first; the normal map stores relief omitted by the mesh.
    with stage_timer(_log, "mesh"):
        mesh = heightmap_to_mesh(
            dem,
            lod=spec.lod,
            skirts=True,
            solid=True,
            center_xy=True,
            z_origin="center",
            face_budget=face_budget,
        )
    mesh_surface = None
    if face_budget is not None:
        _rows, _cols = dem.heights.shape[:2]
        if 2 * (_rows - 1) * (_cols - 1) > face_budget:
            mesh_surface = _mesh_surface_px(mesh, dem)

    textures_dir = staging / "textures"
    with stage_timer(_log, "materials"):
        textures = bake_textures(
            dem,
            ortho=ortho,
            preset=preset,
            dest=textures_dir,
            seed=spec.seed,
            texture_cap_px=texture_cap_px,
            ao_max_px=ao_max_px,
            mesh_stride_px=_mesh_lod_factor,
            mesh_surface=mesh_surface,
        )

    material_usda = staging / "material.usda"
    with stage_timer(_log, "material_usda"):
        write_material_usda(material_usda, textures=textures, material_name="Regolith")

    georef = _compute_georef(dem.crs, effective_patch_center, spec.body)

    fusion_meta: FusionMetadata | None = None
    if plan is not None:
        assert fusion_result is not None and fusion_grid is not None
        fusion_meta = _write_fusion_artifacts(
            paths,
            staging=staging,
            fusion_result=fusion_result,
            grid=fusion_grid,
            plan=plan,
        )

    terrain_usd = staging / "terrain.usd"
    with stage_timer(_log, "terrain_usd"):
        write_terrain_usd(
            terrain_usd,
            mesh=mesh,
            material_usda_relpath="material.usda",
            material_name="Regolith",
            body=spec.body,
            georef=dict(georef) if georef is not None else None,
            fusion=dict(fusion_meta) if fusion_meta is not None else None,
        )

    if fusion_meta is not None:
        (staging / "ATTRIBUTION.txt").write_text(_format_v2_attribution(fusion_meta))
        attribution_list = fusion_attributions(fusion_meta)
    else:
        (staging / "ATTRIBUTION.txt").write_text(
            (attribution_for_bake + "\n") if attribution_for_bake else ""
        )
        attribution_list = [attribution_for_bake] if attribution_for_bake else []

    augmentation_meta: _AugmentationMetadata | None = None
    if augmentation_db is not None:
        from srb.terrain.augmentation.database import (
            AUGMENT_DB_VERSION,
            SCHEMA_ID,
            save_augmentation_db,
        )

        save_augmentation_db(augmentation_db, staging / "augmentation.npz")
        augmentation_meta = {
            "file": "augmentation.npz",
            "schema": SCHEMA_ID,
            "db_version": AUGMENT_DB_VERSION,
            "counts": _require_augmentation_counts(
                augmentation_db.meta["counts"], source="augmentation_db.counts"
            ),
        }

    meta: dict[str, object] = {
        "site": spec.cache_site,
        "body": spec.body,
        "patch": spec.patch,
        "patch_size_m": effective_patch_size_m,
        "lod": spec.lod,
        "seed": spec.seed,
        "materials_version": materials_version,
        "preset": preset.name,
        "attribution": attribution_list,
        "dem_url": dem_url_for_meta,
        "mesh_frame": {
            "center_xy": True,
            "z_origin": "center",
        },
        "baked_at_iso": dt.datetime.now(dt.timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z"),
    }
    if georef is not None:
        meta["georef"] = georef
    if fusion_meta is not None:
        meta["fusion"] = fusion_meta
        if plan is None or catalog is None:
            raise AssertionError(
                "fusion metadata requires the CompositionPlan and SourceCatalog"
            )
        from srb.terrain.fusion import FUSION_VERSION

        if v2_bake_plan is not None:
            dem_stack_hash = v2_bake_plan.dem_stack_sha256
            color_stack_hash = v2_bake_plan.color_stack_sha256
            effective_deshade = v2_bake_plan.effective_deshade
        else:
            dem_stack_hash, color_stack_hash = _v2_stack_hashes(plan, catalog)
            effective_deshade = _effective_deshade(spec)
        meta["bake_identity"] = {
            "schema": cache_mod.BAKE_IDENTITY_SCHEMA,
            "cache_key": paths.key,
            "dem_stack_sha256": dem_stack_hash,
            "color_stack_sha256": color_stack_hash,
            "fusion_version": FUSION_VERSION,
            "target_gsd_m": plan.target_gsd_m,
            "deshade": effective_deshade,
        }
    if augmentation_meta is not None:
        meta["augmentation_db"] = augmentation_meta
    (staging / cache_mod.META_FILENAME).write_text(
        json.dumps(meta, indent=2, sort_keys=True)
    )
