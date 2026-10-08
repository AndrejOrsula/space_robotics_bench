"""Coordinate DEM, colour, quality fusion, and provenance."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
from math import ceil
from typing import Protocol, cast

import numpy as np

from srb.terrain.catalog.loader import layers_by_id
from srb.terrain.catalog.resolve import CompositionPlan, LayerContribution
from srb.terrain.catalog_schema import AlignmentRecord, SourceCatalog, SourceLayer
from srb.terrain.errors import DeclaredLayerUnknownError
from srb.terrain.fusion.blend import blend_len_px, detrend_inset, priority_blend
from srb.terrain.fusion.color import (
    composite_color,
    default_norm_len_m,
    deshade_hillshade_divide,
    lowpass_gain,
    psr_mask,
)
from srb.terrain.fusion.coregister import (
    MAX_SHIFT_PX,
    AppliedShift,
    apply_shift,
    coregister,
)
from srb.terrain.fusion.filters import sigma_px_for_length
from srb.terrain.fusion.grid import PatchGrid, grid_from_plan, warp_to_grid
from srb.terrain.fusion.provenance import (
    ContributorProvenance,
    LayerProvenance,
    _validate_layer_index_contract,
    layer_area_fraction,
    layer_upsampled,
    provenance_raster,
)
from srb.terrain.ingest.dem import Dem, Resampling
from srb.terrain.ingest.planetary_crs import normalize_crs
from srb.terrain.log import get_logger

_log = get_logger("fusion.engine")


class LayerLoader(Protocol):
    """Load one source layer for a fusion grid; return None when unavailable."""

    def __call__(self, layer: SourceLayer, grid: PatchGrid) -> Dem | None: ...


class QualityContractError(ValueError):
    """Base error for quality-layer admission."""


class UnknownQualitySemanticsError(QualityContractError):
    """A selected quality layer has unsupported semantics."""


class InvalidQualityRasterError(QualityContractError):
    """A quality raster violates the sigma-metre contract."""


class ColorAlignmentContractError(ValueError):
    """A color alignment cannot be applied safely."""


_CLAMPED_FRACTION_WARN_THRESHOLD = 0.01


@dataclass(frozen=True, slots=True)
class FusionResult:
    """Outputs and diagnostics; raster snapshots are read-only. Quality is one-sigma
    metres.
    """

    dem: Dem | None
    color_rgb: np.ndarray | None
    provenance: np.ndarray | None
    uncertainty: np.ndarray | None
    dem_layers: tuple[LayerProvenance, ...]
    color_layers: tuple[LayerProvenance, ...]
    warnings: tuple[str, ...]
    base_crs_mismatch: tuple[str, str] | None = None
    contributors: tuple[ContributorProvenance, ...] = ()

    def __post_init__(self) -> None:
        """Publish detached, read-only raster snapshots."""
        for key in ("color_rgb", "provenance", "uncertainty"):
            value = getattr(self, key)
            if isinstance(value, np.ndarray):
                snapshot = value.copy()
                snapshot.setflags(write=False)
                object.__setattr__(self, key, snapshot)


def _base_identity_shift() -> AppliedShift:
    """Return the identity shift for the base fusion frame."""
    return AppliedShift(
        dx_m=0.0,
        dy_m=0.0,
        dz_m=0.0,
        source="assumed_zero",
        residual_m=None,
        demoted=False,
        reason=None,
    )


def _dem_with_planar_shift(dem: Dem, record: AlignmentRecord) -> Dem:
    """Apply a published lateral correction to the source affine."""
    if record.dx_m == 0.0 and record.dy_m == 0.0:
        return dem
    a, b, c, d, e, f = dem.transform_a
    return replace(dem, transform_a=(a, b, c + record.dx_m, d, e, f + record.dy_m))


def _published_shift_halo_px(record: AlignmentRecord, *, gsd_m: float) -> int:
    """Return the source halo needed for a published shift."""
    if record.source == "assumed_zero":
        return 0
    displacement_m = max(abs(record.dx_m), abs(record.dy_m))
    return ceil(displacement_m / gsd_m) + 1 if displacement_m else 0


def _color_applied_shift(layer: SourceLayer) -> AppliedShift:
    """Validate and convert a catalog color alignment."""
    if layer.kind != "color":
        raise ColorAlignmentContractError(
            f"layer {layer.layer_id!r} is not a color layer"
        )

    record = layer.alignment
    if record.dz_m != 0.0:
        raise ColorAlignmentContractError(
            f"color layer {layer.layer_id!r} has nonzero dz_m={record.dz_m!r}"
        )
    if record.source == "assumed_zero":
        if record.dx_m != 0.0 or record.dy_m != 0.0:
            raise ColorAlignmentContractError(
                f"color layer {layer.layer_id!r} uses source='assumed_zero' "
                "with a nonzero lateral shift"
            )
    elif record.source != "published":
        raise ColorAlignmentContractError(
            f"color layer {layer.layer_id!r} uses DEM-only alignment source "
            f"{record.source!r}; color requires 'published' or 'assumed_zero'"
        )

    return AppliedShift(
        dx_m=record.dx_m,
        dy_m=record.dy_m,
        dz_m=0.0,
        source=record.source,
        residual_m=None,
        demoted=False,
        reason=None,
    )


def _auxiliary_contributor_provenance(
    contributions: Sequence[LayerContribution],
) -> tuple[ContributorProvenance, ...]:
    """Build provenance for loaded inputs without raster indices."""
    return tuple(
        ContributorProvenance(
            layer_id=contribution.layer_id,
            role=contribution.role,
            kind=contribution.kind,
            native_gsd_m=contribution.native_gsd_m,
            radiometry=contribution.radiometry,
            license=contribution.license,
            attribution=contribution.attribution,
        )
        for contribution in contributions
    )


def _luminance(rgb: np.ndarray) -> np.ndarray:
    """Return a 2-D luminance array from grayscale or multiband input."""
    if rgb.ndim not in (2, 3):
        raise ValueError(f"colour array must be HW or HWC, got shape={rgb.shape}")
    if rgb.ndim == 2:
        # A grayscale array already is luminance; reducing its last axis drops columns.
        return _require_ndarray(np.asarray(rgb, dtype=np.float64), source="luminance")
    luminance = _require_ndarray(np.mean(rgb, axis=-1), source="luminance")
    if luminance.shape != rgb.shape[:2]:
        raise ValueError(
            f"luminance returned shape={luminance.shape}, expected {rgb.shape[:2]}"
        )
    return luminance


def _require_ndarray(value: object, *, source: str) -> np.ndarray:
    """Validate and narrow an array-producing adapter result."""
    if not isinstance(value, np.ndarray):
        raise TypeError(f"{source} returned {type(value).__name__}, expected ndarray")
    return value


def _lookup_layer(by_id: dict[str, SourceLayer], layer_id: str) -> SourceLayer:
    """Look up a planned layer; raise DeclaredLayerUnknownError on catalog mismatch."""
    try:
        return by_id[layer_id]
    except KeyError:
        raise DeclaredLayerUnknownError(layer_id) from None


def _load_layer_cached(
    layer_id: str,
    *,
    by_id: dict[str, SourceLayer],
    grid: PatchGrid,
    load_layer: LayerLoader,
    cache: dict[str, Dem | None],
) -> Dem | None:
    """Load each catalog layer at most once per fuse call."""
    if layer_id not in cache:
        cache[layer_id] = load_layer(_lookup_layer(by_id, layer_id), grid)
    return cache[layer_id]


def _warp_stack(
    contributions: Sequence[LayerContribution],
    *,
    by_id: dict[str, SourceLayer],
    grid: PatchGrid,
    load_layer: LayerLoader,
    cache: dict[str, Dem | None],
    warnings: list[str],
    kind_label: str,
    resampling: Resampling = "bilinear",
) -> tuple[list[LayerContribution], list[np.ndarray]]:
    """Load and warp contributions in order, dropping unavailable layers with warnings."""
    kept: list[LayerContribution] = []
    arrays: list[np.ndarray] = []
    for contribution in contributions:
        layer = _lookup_layer(by_id, contribution.layer_id)
        request_halo_px = (
            _published_shift_halo_px(layer.alignment, gsd_m=grid.gsd_m)
            if layer.kind == "color"
            else 0
        )
        request_grid = (
            PatchGrid(
                crs=grid.crs,
                center_xy_m=grid.center_xy_m,
                size_m=grid.size_m + 2 * request_halo_px * grid.gsd_m,
                gsd_m=grid.gsd_m,
            )
            if request_halo_px
            else grid
        )
        dem = _load_layer_cached(
            contribution.layer_id,
            by_id=by_id,
            grid=request_grid,
            load_layer=load_layer,
            cache=cache,
        )
        if dem is None:
            msg = (
                f"fuse: {kind_label} layer {contribution.layer_id!r} could "
                "not be loaded; dropped"
            )
            warnings.append(msg)
            _log.warning(msg)
            continue
        kept.append(contribution)
        if layer.kind == "color" and layer.alignment.source == "published":
            dem = _dem_with_planar_shift(dem, layer.alignment)
        arrays.append(warp_to_grid(dem, grid, resampling=resampling))
    return kept, arrays


def _require_vertical_sigma_semantics(layer: SourceLayer) -> None:
    semantics = layer.quality_semantics
    if semantics is None or semantics.measure != "vertical_one_sigma_m":
        raise UnknownQualitySemanticsError(
            f"quality layer {layer.layer_id!r} must declare "
            "quality_semantics.measure='vertical_one_sigma_m'"
        )


def _validate_quality_plan(
    contributions: Sequence[LayerContribution], *, by_id: dict[str, SourceLayer]
) -> None:
    """Validate quality semantics before layer I/O."""
    for contribution in contributions:
        layer = _lookup_layer(by_id, contribution.layer_id)
        if contribution.kind != "quality" or layer.kind != "quality":
            raise QualityContractError(
                f"quality plan entry {contribution.layer_id!r} does not resolve "
                "to kind='quality'"
            )
        _require_vertical_sigma_semantics(layer)


def decode_quality_to_sigma_m(
    layer: SourceLayer, values: np.ndarray, *, grid: PatchGrid
) -> np.ndarray:
    """Validate a grid-shaped raster as vertical one-sigma uncertainty in metres."""
    _require_vertical_sigma_semantics(layer)
    sigma_m = np.asarray(values, dtype=np.float32)
    if sigma_m.ndim != 2 or sigma_m.shape != grid.shape:
        raise InvalidQualityRasterError(
            f"quality layer {layer.layer_id!r} must be a 2-D raster with "
            f"shape {grid.shape!r}; got {sigma_m.shape!r}"
        )
    if np.any(np.isinf(sigma_m)):
        raise InvalidQualityRasterError(
            f"quality layer {layer.layer_id!r} contains infinite uncertainty"
        )
    finite = np.isfinite(sigma_m)
    if np.any(sigma_m[finite] < 0.0):
        raise InvalidQualityRasterError(
            f"quality layer {layer.layer_id!r} contains negative uncertainty"
        )
    return cast(np.ndarray, sigma_m.copy())


def _fuse_quality(
    contributions: Sequence[LayerContribution],
    *,
    by_id: dict[str, SourceLayer],
    grid: PatchGrid,
    load_layer: LayerLoader,
    cache: dict[str, Dem | None],
    warnings: list[str],
) -> tuple[np.ndarray | None, tuple[LayerContribution, ...]]:
    """Priority-fill typed sigma rasters without statistical combination."""
    if not contributions:
        return None, ()

    uncertainty: np.ndarray | None = None
    kept: list[LayerContribution] = []
    for contribution in contributions:
        if uncertainty is not None and np.all(np.isfinite(uncertainty)):
            break
        loaded, arrays = _warp_stack(
            (contribution,),
            by_id=by_id,
            grid=grid,
            load_layer=load_layer,
            cache=cache,
            warnings=warnings,
            kind_label="quality",
            resampling="nearest",
        )
        if not arrays:
            continue

        sigma_m = decode_quality_to_sigma_m(
            _lookup_layer(by_id, contribution.layer_id), arrays[0], grid=grid
        )
        kept.extend(loaded)
        if uncertainty is None:
            uncertainty = sigma_m
            continue
        fill = ~np.isfinite(uncertainty) & np.isfinite(sigma_m)
        uncertainty[fill] = sigma_m[fill]
    return uncertainty, tuple(kept)


def _empty_result(
    warnings: list[str], *, base_crs_mismatch: tuple[str, str] | None = None
) -> FusionResult:
    """Build the fatal-base result."""
    return FusionResult(
        dem=None,
        color_rgb=None,
        provenance=None,
        uncertainty=None,
        dem_layers=(),
        color_layers=(),
        warnings=tuple(warnings),
        base_crs_mismatch=base_crs_mismatch,
    )


@dataclass(frozen=True, slots=True)
class _BaseFatal:
    """Internal fatal-base result, including an optional CRS mismatch."""

    crs_mismatch: tuple[str, str] | None


def _dem_layer_provenance(
    *,
    blend_order: Sequence[LayerContribution],
    shifts: Sequence[AppliedShift],
    raster: np.ndarray,
    grid: PatchGrid,
) -> tuple[LayerProvenance, ...]:
    """Build provenance in blend order with area fractions from the raster."""
    return tuple(
        LayerProvenance(
            layer_id=contribution.layer_id,
            role=contribution.role,
            kind=contribution.kind,
            index=index,
            area_fraction=layer_area_fraction(raster, index=index),
            native_gsd_m=contribution.native_gsd_m,
            upsampled=layer_upsampled(contribution.native_gsd_m, grid.gsd_m),
            radiometry=contribution.radiometry,
            alignment=shift,
            license=contribution.license,
            attribution=contribution.attribution,
        )
        for index, (contribution, shift) in enumerate(
            zip(blend_order, shifts, strict=True),
            start=1,
        )
    )


def _fuse_dem(
    plan: CompositionPlan,
    *,
    by_id: dict[str, SourceLayer],
    grid: PatchGrid,
    load_layer: LayerLoader,
    cache: dict[str, Dem | None],
    estimate_alignment: bool,
    blend_len_m: float | None,
    warnings: list[str],
) -> tuple[Dem, np.ndarray, tuple[LayerProvenance, ...]] | _BaseFatal:
    """Load, align, detrend, and blend the DEM stack; base failure is fatal."""
    base_contribution = plan.dem_stack[0]
    # Request a halo so lateral shifts cannot expose patch edges.
    halo_px = MAX_SHIFT_PX + 1
    for contribution in plan.dem_stack[1:]:
        layer = _lookup_layer(by_id, contribution.layer_id)
        halo_px = max(
            halo_px,
            _published_shift_halo_px(layer.alignment, gsd_m=grid.gsd_m),
        )
    work_grid = PatchGrid(
        crs=grid.crs,
        center_xy_m=grid.center_xy_m,
        size_m=grid.size_m + 2 * halo_px * grid.gsd_m,
        gsd_m=grid.gsd_m,
    )
    crop = (slice(halo_px, halo_px + grid.pixels),) * 2
    base_dem = _load_layer_cached(
        base_contribution.layer_id,
        by_id=by_id,
        grid=work_grid,
        load_layer=load_layer,
        cache=cache,
    )
    if base_dem is None:
        msg = (
            f"fuse: DEM base layer {base_contribution.layer_id!r} could not "
            "be loaded -- the DEM stack has no other source to fall back to"
        )
        warnings.append(msg)
        _log.warning(msg)
        return _BaseFatal(crs_mismatch=None)

    if base_dem.crs:
        base_crs_norm = normalize_crs(base_dem.crs, body=plan.request.body)
        plan_crs_norm = normalize_crs(plan.crs, body=plan.request.body)
        if base_crs_norm.canonical_id != plan_crs_norm.canonical_id:
            msg = (
                f"fuse: DEM base layer {base_contribution.layer_id!r} raster "
                f"CRS {base_dem.crs!r} does not match the resolved plan CRS "
                f"{plan.crs!r} (canonical id {base_crs_norm.canonical_id!r} "
                f"!= {plan_crs_norm.canonical_id!r}) -- the windowed read "
                "paths interpret center_xy_m in the raster's own CRS with "
                "no transform, so a header disagreement here means the "
                "read window itself is wrong and/or the reprojection below "
                "would displace or empty out its content"
            )
            warnings.append(msg)
            _log.warning(msg)
            return _BaseFatal(crs_mismatch=(str(base_dem.crs), str(plan.crs)))

    base_grid = warp_to_grid(base_dem, work_grid)

    # Measure base coverage before blending; higher-priority insets can
    # reduce its final provenance area to zero.
    base_patch = base_grid[crop]
    base_finite_fraction = (
        float(np.isfinite(base_patch).mean()) if base_patch.size else 0.0
    )
    if base_finite_fraction <= 0.0:
        msg = (
            f"fuse: DEM base layer {base_contribution.layer_id!r} loaded "
            "and matched the plan's CRS, but its warped contribution to "
            "the fused patch is entirely nodata (0% finite coverage after "
            "warp) -- check the layer's declared footprint against its "
            "raster header"
        )
        warnings.append(msg)
        _log.warning(msg)
        return _BaseFatal(crs_mismatch=None)

    warped_insets: list[tuple[LayerContribution, SourceLayer, np.ndarray]] = []
    for contribution in plan.dem_stack[1:]:
        layer = _lookup_layer(by_id, contribution.layer_id)
        dem = _load_layer_cached(
            contribution.layer_id,
            by_id=by_id,
            grid=work_grid,
            load_layer=load_layer,
            cache=cache,
        )
        if dem is None:
            msg = (
                f"fuse: DEM inset {contribution.layer_id!r} could not be "
                "loaded; dropped"
            )
            warnings.append(msg)
            _log.warning(msg)
            continue
        is_published = layer.alignment.source != "assumed_zero"
        aligned_dem = (
            _dem_with_planar_shift(dem, layer.alignment) if is_published else dem
        )
        warped_insets.append(
            (contribution, layer, warp_to_grid(aligned_dem, work_grid))
        )

    qc_blend_px, _ = blend_len_px(
        base_gsd_m=base_contribution.native_gsd_m,
        out_gsd_m=grid.gsd_m,
        blend_len_m=blend_len_m,
    )
    qc_trend_sigma_px = sigma_px_for_length(qc_blend_px * grid.gsd_m, gsd_m=grid.gsd_m)
    coregistered: list[tuple[LayerContribution, np.ndarray, AppliedShift]] = []
    for contribution, layer, warped in warped_insets:
        is_published = layer.alignment.source != "assumed_zero"
        check_record = (
            AlignmentRecord(source="published", dz_m=layer.alignment.dz_m)
            if is_published
            else layer.alignment
        )
        shifted, applied = coregister(
            base_grid[crop],
            warped[crop],
            gsd_m=grid.gsd_m,
            record=check_record,
            estimate=estimate_alignment,
            trend_sigma_px=qc_trend_sigma_px,
        )
        if not applied.demoted:
            shift_dx_px = 0.0 if is_published else applied.dx_m / grid.gsd_m
            shift_dy_px = 0.0 if is_published else -applied.dy_m / grid.gsd_m
            shifted = apply_shift(
                warped,
                dx_px=shift_dx_px,
                dy_px=shift_dy_px,
                dz_m=check_record.dz_m,
            )
        if is_published:
            applied = replace(
                applied,
                dx_m=layer.alignment.dx_m,
                dy_m=layer.alignment.dy_m,
            )
            if applied.demoted:
                original = _load_layer_cached(
                    contribution.layer_id,
                    by_id=by_id,
                    grid=grid,
                    load_layer=load_layer,
                    cache=cache,
                )
                assert original is not None
                shifted = warp_to_grid(original, work_grid)
        if applied.reason is not None:
            if applied.demoted:
                msg = (
                    f"fuse: DEM inset {contribution.layer_id!r} demoted "
                    f"during co-registration: {applied.reason}"
                )
            else:
                msg = (
                    f"fuse: DEM inset {contribution.layer_id!r} kept but "
                    f"not co-registered: {applied.reason}"
                )
            warnings.append(msg)
            _log.warning(msg)
        if applied.demoted:
            continue
        coregistered.append((contribution, shifted[crop], applied))

    inset_blend_px, blend_len_clamped = blend_len_px(
        base_gsd_m=base_contribution.native_gsd_m,
        out_gsd_m=grid.gsd_m,
        blend_len_m=blend_len_m,
    )
    # Warn only when an explicit blend_len_m was clamped; default-derived
    # clamps are expected.
    if blend_len_clamped and blend_len_m is not None:
        msg = (
            "fuse: DEM blend_len_m clamped to fusion.blend's [50, 200] m "
            f"range (requested blend_len_m={blend_len_m}, "
            f"base_gsd_m={base_contribution.native_gsd_m})"
        )
        warnings.append(msg)
        _log.warning(msg)
    effective_blend_len_m = inset_blend_px * grid.gsd_m
    detrend_sigma_px = sigma_px_for_length(effective_blend_len_m, gsd_m=grid.gsd_m)

    detrended: list[tuple[LayerContribution, np.ndarray, AppliedShift]] = [
        (
            contribution,
            detrend_inset(shifted, base_patch, sigma_px=detrend_sigma_px),
            applied,
        )
        for contribution, shifted, applied in coregistered
    ]

    # Blend highest-priority insets first; the base is last.
    inset_arrays = [arr for _c, arr, _a in detrended]
    layers_for_blend = [*inset_arrays, base_patch]
    blend_px_for_blend = [inset_blend_px] * len(inset_arrays) + [0.0]
    assert layers_for_blend[-1] is base_patch, (
        "priority_blend needs the base last; it consumes inputs front to back"
    )
    assert blend_px_for_blend[-1] == 0.0, (
        "the base must feather with blend_px=0 -- a nonzero base feather "
        "erodes a blend_px-wide NaN ring around every interior hole in the "
        "base (fusion.blend.priority_blend's own docstring, trap 2)"
    )
    fused, dem_prov_index = priority_blend(
        layers_for_blend, blend_px=blend_px_for_blend
    )
    dem_blend_order = [*(c for c, _a, _s in detrended), base_contribution]
    dem_shifts = [
        *(shift for _c, _a, shift in detrended),
        _base_identity_shift(),
    ]
    dem_provenance_raster = provenance_raster(dem_prov_index, len(dem_blend_order))
    dem_layers = _dem_layer_provenance(
        blend_order=dem_blend_order,
        shifts=dem_shifts,
        raster=dem_provenance_raster,
        grid=grid,
    )

    result_dem = Dem(
        heights=fused,
        gsd_m=grid.gsd_m,
        nodata=float("nan"),
        crs=grid.crs,
        transform_a=grid.transform_a,
    )
    return result_dem, dem_provenance_raster, dem_layers


def _normalize_color_stack(
    *,
    color_kept: list[LayerContribution],
    color_arrays: list[np.ndarray],
    grid: PatchGrid,
    warnings: list[str],
) -> None:
    """Match sunlit layers to the base anchor; mutate color arrays in place."""
    anchor_pos = next((i for i, c in enumerate(color_kept) if c.role == "base"), None)
    if anchor_pos is None:
        if color_kept:
            names = ", ".join(c.layer_id for c in color_kept)
            msg = (
                "fuse: colour stack has no role=='base' anchor; skipping "
                f"radiometric normalisation for: {names} (expected for an "
                "all-inset colour stack -- not necessarily a problem)"
            )
            warnings.append(msg)
            _log.warning(msg)
        return

    anchor_contribution = color_kept[anchor_pos]
    anchor_luma = _luminance(color_arrays[anchor_pos])
    norm_len_m = default_norm_len_m(
        anchor_contribution.native_gsd_m, patch_size_m=grid.size_m
    )
    norm_sigma_px = sigma_px_for_length(norm_len_m, gsd_m=grid.gsd_m)

    for i, contribution in enumerate(color_kept):
        if i == anchor_pos:
            continue
        detail_luma = _luminance(color_arrays[i])
        gain, clamped_fraction = lowpass_gain(
            detail_luma, anchor_luma, sigma_px=norm_sigma_px
        )
        # Ignore isolated edge clamps; warn when gain correction clamps
        # across more than 1% of layer support.
        if clamped_fraction > _CLAMPED_FRACTION_WARN_THRESHOLD:
            msg = (
                f"fuse: colour layer {contribution.layer_id!r} radiometric "
                f"gain clamped on {clamped_fraction:.1%} of its support"
            )
            warnings.append(msg)
            _log.warning(msg)
        color_arrays[i] = color_arrays[i] * (
            gain if color_arrays[i].ndim == 2 else gain[..., None]
        )


def _color_layer_provenance(
    *,
    by_id: dict[str, SourceLayer],
    psr_kept: list[LayerContribution],
    color_kept: list[LayerContribution],
    raw_prov_index: np.ndarray,
    grid: PatchGrid,
) -> tuple[LayerProvenance, ...]:
    """Build color provenance in composite order with remapped indices."""
    raw_order = [*psr_kept, *color_kept]
    seen_ids: set[str] = set()
    unique_order: list[LayerContribution] = []
    for c in raw_order:
        if c.layer_id not in seen_ids:
            seen_ids.add(c.layer_id)
            unique_order.append(c)

    sunlit_anchor_id = next((c.layer_id for c in color_kept if c.role == "base"), None)
    if sunlit_anchor_id is not None:
        anchor = next(c for c in color_kept if c.layer_id == sunlit_anchor_id)
        unique_order = [
            anchor if c.layer_id == sunlit_anchor_id else c for c in unique_order
        ]
    unique_order = [
        replace(c, role="inset")
        if c.role == "base" and c.layer_id != sunlit_anchor_id
        else c
        for c in unique_order
    ]

    blend_order = [
        *(c for c in unique_order if c.role != "base"),
        *(c for c in unique_order if c.role == "base"),
    ]
    id_to_blend_index = {c.layer_id: i for i, c in enumerate(blend_order)}
    remap_table = np.array(
        [id_to_blend_index[c.layer_id] for c in raw_order], dtype=np.int16
    )
    blend_prov_index = np.where(
        raw_prov_index >= 0,
        remap_table[np.clip(raw_prov_index, 0, None)],
        np.int16(-1),
    ).astype(np.int16)
    raster = provenance_raster(blend_prov_index, len(blend_order))

    return tuple(
        LayerProvenance(
            layer_id=c.layer_id,
            role=c.role,
            kind=c.kind,
            index=index,
            area_fraction=layer_area_fraction(raster, index=index),
            native_gsd_m=c.native_gsd_m,
            upsampled=layer_upsampled(c.native_gsd_m, grid.gsd_m),
            radiometry=c.radiometry,
            alignment=_color_applied_shift(_lookup_layer(by_id, c.layer_id)),
            license=c.license,
            attribution=c.attribution,
        )
        for index, c in enumerate(blend_order, start=1)
    )


def _normalize_color_layer_scales(
    *,
    kept: Sequence[LayerContribution],
    arrays: list[np.ndarray],
    cache: dict[str, Dem | None],
) -> None:
    """Convert integer color inputs to unit scale before compositing."""
    for i, contribution in enumerate(kept):
        dem = cache.get(contribution.layer_id)
        if dem is None:
            continue
        dtype = dem.heights.dtype
        if np.issubdtype(dtype, np.integer):
            arrays[i] = arrays[i].astype(np.float32) / float(np.iinfo(dtype).max)
        elif dtype == np.bool_:
            arrays[i] = arrays[i].astype(np.float32)


def _deshade_color_stack(
    *,
    color_kept: list[LayerContribution],
    color_arrays: list[np.ndarray],
    by_id: dict[str, SourceLayer],
    fused_dem: Dem,
    grid: PatchGrid,
    warnings: list[str],
) -> None:
    """Remove modeled illumination in place; warn when sun geometry is missing."""
    for i, contribution in enumerate(color_kept):
        layer = _lookup_layer(by_id, contribution.layer_id)
        if layer.sun_azimuth_deg is None or layer.sun_elevation_deg is None:
            msg = (
                f"fuse: deshade requested but colour layer "
                f"{contribution.layer_id!r} has no "
                "sun_azimuth_deg/sun_elevation_deg illumination metadata; "
                "left un-deshaded"
            )
            warnings.append(msg)
            _log.warning(msg)
            continue
        color_arrays[i] = deshade_hillshade_divide(
            color_arrays[i],
            fused_dem.heights,
            gsd_m=grid.gsd_m,
            sun_azimuth_deg=layer.sun_azimuth_deg,
            sun_elevation_deg=layer.sun_elevation_deg,
        )


def _fuse_color(
    plan: CompositionPlan,
    *,
    by_id: dict[str, SourceLayer],
    grid: PatchGrid,
    load_layer: LayerLoader,
    cache: dict[str, Dem | None],
    warnings: list[str],
    deshade: bool = False,
    fused_dem: Dem | None = None,
) -> tuple[np.ndarray | None, tuple[LayerProvenance, ...]]:
    """Warp, normalize, optionally deshade, and composite color layers."""
    for contribution in (*plan.color_stack, *plan.color_stack_psr):
        _color_applied_shift(_lookup_layer(by_id, contribution.layer_id))

    color_kept, color_arrays = _warp_stack(
        plan.color_stack,
        by_id=by_id,
        grid=grid,
        load_layer=load_layer,
        cache=cache,
        warnings=warnings,
        kind_label="colour",
    )
    if deshade and color_arrays:
        if fused_dem is None:
            msg = (
                "fuse: deshade requested but no fused DEM is available to "
                "model hillshade from; colour left un-deshaded"
            )
            warnings.append(msg)
            _log.warning(msg)
        else:
            _deshade_color_stack(
                color_kept=color_kept,
                color_arrays=color_arrays,
                by_id=by_id,
                fused_dem=fused_dem,
                grid=grid,
                warnings=warnings,
            )
    psr_kept, psr_arrays = _warp_stack(
        plan.color_stack_psr,
        by_id=by_id,
        grid=grid,
        load_layer=load_layer,
        cache=cache,
        warnings=warnings,
        kind_label="colour PSR",
    )
    _normalize_color_layer_scales(kept=color_kept, arrays=color_arrays, cache=cache)
    _normalize_color_layer_scales(kept=psr_kept, arrays=psr_arrays, cache=cache)
    mask_kept, mask_arrays = _warp_stack(
        plan.mask_layers,
        by_id=by_id,
        grid=grid,
        load_layer=load_layer,
        cache=cache,
        warnings=warnings,
        kind_label="mask",
        resampling="nearest",
    )
    del mask_kept

    if not color_arrays and not psr_arrays:
        if not plan.color_stack and not plan.color_stack_psr:
            msg = (
                "fuse: CompositionPlan.color_stack is empty; no colour to "
                "composite (expected for a DEM-only site -- not a "
                "degradation)"
            )
        else:
            msg = "fuse: every colour layer failed to load; no colour to composite"
        warnings.append(msg)
        _log.warning(msg)
        return None, ()

    _normalize_color_stack(
        color_kept=color_kept, color_arrays=color_arrays, grid=grid, warnings=warnings
    )

    psr_mask_arr = psr_mask(mask_arrays)
    rgb_u8, raw_prov_index = composite_color(
        color_arrays, psr=psr_mask_arr, psr_layers=psr_arrays, assume_scale="unit"
    )
    color_layers = _color_layer_provenance(
        by_id=by_id,
        psr_kept=psr_kept,
        color_kept=color_kept,
        raw_prov_index=raw_prov_index,
        grid=grid,
    )
    return rgb_u8, color_layers


def fuse(
    plan: CompositionPlan,
    catalog: SourceCatalog,
    *,
    load_layer: LayerLoader,
    estimate_alignment: bool = True,
    blend_len_m: float | None = None,
    deshade: bool = False,
) -> FusionResult:
    """Fuse a plan into DEM, color, quality, provenance, and diagnostics.

    A missing DEM base is fatal; failed insets and color layers are omitted. Quality
    rasters are vertical one-sigma metres and fill holes in resolver order.
    """
    warnings: list[str] = []
    by_id = layers_by_id(catalog)
    _validate_quality_plan(plan.quality_layers, by_id=by_id)
    grid = grid_from_plan(plan)
    load_cache: dict[str, Dem | None] = {}

    if not plan.dem_stack:
        msg = "fuse: CompositionPlan.dem_stack is empty -- no DEM base to fuse"
        warnings.append(msg)
        _log.warning(msg)
        return _empty_result(warnings)

    dem_outcome = _fuse_dem(
        plan,
        by_id=by_id,
        grid=grid,
        load_layer=load_layer,
        cache=load_cache,
        estimate_alignment=estimate_alignment,
        blend_len_m=blend_len_m,
        warnings=warnings,
    )
    if isinstance(dem_outcome, _BaseFatal):
        return _empty_result(warnings, base_crs_mismatch=dem_outcome.crs_mismatch)
    result_dem, dem_provenance_raster, dem_layers = dem_outcome

    uncertainty, quality_kept = _fuse_quality(
        plan.quality_layers,
        by_id=by_id,
        grid=grid,
        load_layer=load_layer,
        cache=load_cache,
        warnings=warnings,
    )

    # Deshading uses the final fused DEM, so DEM fusion must finish first.
    color_rgb, color_layers = _fuse_color(
        plan,
        by_id=by_id,
        grid=grid,
        load_layer=load_layer,
        cache=load_cache,
        warnings=warnings,
        deshade=deshade,
        fused_dem=result_dem,
    )
    loaded_masks = (
        tuple(
            contribution
            for contribution in plan.mask_layers
            if load_cache.get(contribution.layer_id) is not None
        )
        if color_rgb is not None
        else ()
    )

    # These checks must run under python -O, where assert statements disappear.
    _validate_layer_index_contract(dem_layers, field="dem_layers")
    _validate_layer_index_contract(color_layers, field="color_layers")

    return FusionResult(
        dem=result_dem,
        color_rgb=color_rgb,
        provenance=dem_provenance_raster,
        uncertainty=uncertainty,
        dem_layers=dem_layers,
        color_layers=color_layers,
        warnings=tuple(warnings),
        contributors=_auxiliary_contributor_provenance((*loaded_masks, *quality_kept)),
    )


__all__ = ["FusionResult", "LayerLoader", "fuse"]
