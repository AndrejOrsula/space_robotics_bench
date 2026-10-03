"""Resolve catalog layers into terrain stacks for a region request. Candidates sort by
priority, resolution, and ID; DEMs use a full-coverage base plus finer insets.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Literal

from srb.terrain.catalog.footprint import (
    Region,
    coverage_samples_inside,
    parse_footprint,
)
from srb.terrain.catalog.loader import layers_by_id
from srb.terrain.catalog_schema import (
    AccessMode,
    LayerKind,
    Radiometry,
    SourceCatalog,
    SourceLayer,
    url_basename,
)
from srb.terrain.errors import (
    DeclaredLayerDuplicateError,
    DeclaredLayerUnknownError,
    ReferenceOnlyLayerError,
    RegionInvalidError,
    RegionTooLargeError,
)
from srb.terrain.ingest.planetary_crs import (
    CANONICAL_CRS,
    canonical_crs_for,
    latlon_to_xy,
    local_stereographic_crs,
    normalize_crs,
)
from srb.terrain.schema import Body, Lod, StackRef
from srb.terrain.sources.acquisition import MAX_UNAPPROVED_SOURCE_BYTES

MAX_REGION_SIZE_M: float = 8000.0
POLAR_LAT_DEG: float = 60.0
FLOAT32_BYTES: int = 4

_NO_BASE_WARNING = "no gap-free DEM base covers the region"

_DECLARED_MAX_GSD_DROPPED_WARNING_FMT = (
    "declared layer {layer_id!r} native_gsd_m={native_gsd_m} exceeds "
    "max_gsd_m={max_gsd_m}; dropped"
)
DECLARED_NO_BASE_WARNING = (
    "declared DEM base was dropped by the max_gsd_m filter; no DEM base remains"
)
"""Public API: was `_DECLARED_NO_BASE_WARNING`. A caller
that needs to discriminate "no DEM base remains" from every other reason a
declared plan can carry warnings (the raise-on-`is_empty` contract
wants to say *why* in its own error message) needs this as an importable
name, not a private one forcing a substring match against resolver-internal
wording."""
_DECLARED_BASE_ZERO_COVERAGE_WARNING_FMT = (
    "declared DEM base {layer_id!r} has zero coverage of the requested "
    "region (coverage_fraction=0.0); dem_stack will carry no real data "
    "despite resolving non-empty"
)
_DECLARED_FRAME_MISMATCH_WARNING_FMT = (
    "declared layer {layer_id!r} has a polygon footprint authored in "
    "{authored_crs!r}, which is not the request's frame {crs!r}; its "
    "absolute metres are meaningless there, so it is dropped"
)
"""The `_build_sorted_candidates` path skips a non-comparable polygon
footprint; the declared-ref half must drop it too rather than fall through
to the SAMPLED-ZERO path (`inside, total = 0, 1`) and append the layer
anyway. Membership on a declared stack is coverage-independent by design,
so "coverage 0.0" does not drop it -- it would become a real
`LayerContribution`, and `_total_fetch_bytes` /
`_consent_required_layer_ids` / `fusion.engine`'s contribution loop are all
coverage-BLIND, so gigabytes would be fetched for a layer that can
contribute nothing, silently. A frame mismatch is provably
an authoring/frame error rather than a footprint miss -- the metres cannot
be compared at all, so there is no coverage question to be
coverage-independent about -- hence a dedicated warning and a drop, not the
zero-coverage warning."""
_DECLARED_BODY_MISMATCH_WARNING_FMT = (
    "declared layer {layer_id!r} belongs to body {layer_body!r}, not the "
    "request's body {body!r}; dropped"
)
"""`_build_sorted_candidates` filters
`layer.body != body` as its very first test; the declared-ref path checks it
here. A cross-body `layer_id` would otherwise be projected and coverage-sampled
in the WRONG body's frame -- `_polygon_footprint_is_comparable` reads
`CANONICAL_CRS[f"{layer.body}_south"]`, so the authoring frame would be built
from the other body's canonical entry. `_check_body_match` only compares
`catalog.body` to `request.body` and cannot see a per-LAYER mismatch, which is
exactly why `_build_sorted_candidates` carries the per-layer test too."""

_Candidate = tuple[SourceLayer, float, bool]


@dataclass(frozen=True, slots=True)
class RegionRequest:
    """A terrain request, validated and quantized before projection. Sizes are finite,
    positive, and at most 8 km; coordinates are rounded to 4 decimals.
    """

    body: Body
    lat_deg: float
    lon_deg: float
    size_m: float
    target_gsd_m: float | None = None
    lod: Lod = "medium"
    crs: str | None = None
    allow_synthetic: bool = False

    def __post_init__(self) -> None:
        for name in ("lat_deg", "lon_deg", "size_m"):
            value = getattr(self, name)
            try:
                finite = math.isfinite(float(value))
            except (TypeError, ValueError, OverflowError):
                finite = False
            if not finite:
                remediation = {
                    "lat_deg": "Pass --lat as a finite coordinate.",
                    "lon_deg": "Pass --lon as a finite coordinate.",
                    "size_m": "Pass --size as a finite positive number of meters.",
                }[name]
                raise RegionInvalidError(
                    self.size_m,
                    reason=f"{name} must be finite, got {value!r}",
                    remediation=remediation,
                )
        if self.size_m > MAX_REGION_SIZE_M:
            raise RegionTooLargeError(self.size_m, max_size_m=MAX_REGION_SIZE_M)
        if self.size_m <= 0:
            raise RegionInvalidError(self.size_m)
        if not (-90.0 <= self.lat_deg <= 90.0):
            raise RegionInvalidError(
                self.size_m,
                reason=f"lat_deg must be in [-90, 90], got {self.lat_deg}",
                remediation="Pass --lat in [-90, 90] degrees.",
            )
        if self.target_gsd_m is not None:
            try:
                valid_target_gsd = math.isfinite(float(self.target_gsd_m)) and (
                    self.target_gsd_m > 0
                )
            except (TypeError, ValueError, OverflowError):
                valid_target_gsd = False
            if not valid_target_gsd:
                raise RegionInvalidError(
                    self.size_m,
                    reason=(
                        "target_gsd_m must be finite and > 0, got "
                        f"{self.target_gsd_m!r}"
                    ),
                    remediation="Pass --target-gsd as a finite positive number of meters.",
                )

        # Canonicalize stored values so cache identity and resolution agree.
        lon = (self.lon_deg + 180.0) % 360.0 - 180.0
        lat = round(self.lat_deg, 4)
        lon = round(lon, 4)
        # Rounding can reintroduce +180 and signed-zero aliases.
        if lon >= 180.0:
            lon -= 360.0
        lat += 0.0
        lon += 0.0
        object.__setattr__(self, "lat_deg", lat)
        object.__setattr__(self, "lon_deg", lon)
        object.__setattr__(self, "size_m", float(round(self.size_m)))
        if self.size_m <= 0:
            raise RegionInvalidError(
                self.size_m,
                reason=(
                    "size_m must remain > 0 after whole-metre rounding; "
                    f"got {self.size_m}"
                ),
            )

    @property
    def resolved_crs(self) -> str:
        """Use the explicit CRS, otherwise the body’s polar CRS at high latitudes or a
        request-centered local stereographic CRS.
        """
        if self.crs is not None:
            return self.crs
        if self.is_polar:
            return canonical_crs_for(
                self.body, self.lat_deg, polar_lat_deg=POLAR_LAT_DEG
            )
        return local_stereographic_crs(self.body, self.lat_deg, self.lon_deg)

    @property
    def is_polar(self) -> bool:
        return abs(self.lat_deg) >= POLAR_LAT_DEG

    @property
    def slug(self) -> str:
        """Stable cache-site identity for the quantized body, coordinates, and size;
        changing field widths or rounding can orphan cache entries.
        """
        return (
            f"region_{self.body}_{self.lat_deg:+08.4f}_{self.lon_deg:+09.4f}"
            f"_{int(round(self.size_m))}m"
        )


@dataclass(frozen=True, slots=True)
class LayerContribution:
    """One catalog layer in a resolved stack."""

    layer_id: str
    role: Literal["base", "inset"]
    kind: LayerKind
    native_gsd_m: float
    priority_class: int
    coverage_fraction: float
    access: AccessMode
    est_fetch_bytes: int
    requires_consent: bool
    radiometry: Radiometry | None
    license: str
    attribution: str


@dataclass(frozen=True, slots=True)
class CompositionPlan:
    """Resolved terrain stacks and their fetch and consent metadata."""

    request: RegionRequest
    crs: str
    center_xy_m: tuple[float, float]
    target_gsd_m: float
    dem_stack: tuple[LayerContribution, ...]
    color_stack: tuple[LayerContribution, ...]
    color_stack_psr: tuple[LayerContribution, ...]
    mask_layers: tuple[LayerContribution, ...]
    quality_layers: tuple[LayerContribution, ...]
    total_fetch_bytes: int
    consented: bool
    consent_required_layer_ids: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def is_empty(self) -> bool:
        """Whether the resolved DEM stack is empty."""
        return len(self.dem_stack) == 0


def _sort_key(layer: SourceLayer) -> tuple[bool, int, float, str]:
    """Sort by synthetic status, priority, native GSD, then unique layer ID. Synthetic
    layers always follow real layers.
    """
    return (
        layer.radiometry == "synthetic",
        layer.priority_class,
        layer.native_gsd_m,
        layer.layer_id,
    )


def est_fetch_bytes(layer: SourceLayer, *, size_m: float, target_gsd_m: float) -> int:
    """Estimate bytes for one layer and region. Remote COG reads use max(native GSD,
    target GSD); downloads and ZIP members use full file size. Float32 sizing is
    conservative for 8-bit data. Upsampling and padding can make actual reads larger.
    """
    if layer.access == "cog_remote":
        step = max(layer.native_gsd_m, target_gsd_m)
        px = math.ceil(size_m / step)
        return min(layer.size_bytes, px * px * FLOAT32_BYTES)
    return layer.size_bytes


def _contribution(
    layer: SourceLayer,
    *,
    role: Literal["base", "inset"],
    coverage_fraction: float,
    size_m: float,
    target_gsd_m: float,
) -> LayerContribution:
    fetch_bytes = est_fetch_bytes(layer, size_m=size_m, target_gsd_m=target_gsd_m)
    return LayerContribution(
        layer_id=layer.layer_id,
        role=role,
        kind=layer.kind,
        native_gsd_m=layer.native_gsd_m,
        priority_class=layer.priority_class,
        coverage_fraction=coverage_fraction,
        access=layer.access,
        est_fetch_bytes=fetch_bytes,
        requires_consent=fetch_bytes > MAX_UNAPPROVED_SOURCE_BYTES,
        radiometry=layer.radiometry,
        license=layer.license,
        attribution=layer.attribution,
    )


def _check_body_match(catalog: SourceCatalog, request: RegionRequest) -> None:
    """Require the catalog and request to use the same body."""
    if catalog.body != request.body:
        raise ValueError(
            f"catalog body {catalog.body!r} does not match request body "
            f"{request.body!r}"
        )


_WKT_POLYGON_PREFIXES = ("POLYGON", "MULTIPOLYGON")


def _is_absolute_frame_footprint(spec: str) -> bool:
    """Whether a footprint uses absolute projected coordinates (WKT polygon)."""
    return spec.strip().upper().startswith(_WKT_POLYGON_PREFIXES)


def _polygon_footprint_is_comparable(crs: str, layer: SourceLayer) -> bool:
    """Check whether a polygon footprint and request use the same authored frame.
    Normalize equivalent CRS spellings; use strict CRS equality when either side only
    matched a body default.
    """
    authored = layer.crs or CANONICAL_CRS[f"{layer.body}_south"]
    if crs == authored:
        return True
    try:
        req_norm = normalize_crs(crs, body=layer.body)
        auth_norm = normalize_crs(authored, body=layer.body)
    except Exception:
        return False
    if req_norm.matched_by != "body_default" and auth_norm.matched_by != "body_default":
        return req_norm.canonical_id == auth_norm.canonical_id
    try:
        from rasterio.crs import CRS as _RioCRS

        return _RioCRS.from_user_input(crs) == _RioCRS.from_user_input(authored)
    except Exception:
        return False


def _build_sorted_candidates(
    catalog: SourceCatalog,
    *,
    body: Body,
    region: Region,
    crs: str,
    allow_synthetic: bool = False,
) -> list[_Candidate]:
    """Select same-body, comparable, non-reference layers with sampled coverage, then
    sort. Synthetic layers require opt-in; reference-only layers enter only through
    explicit declared refs.
    """
    candidates: list[_Candidate] = []
    for layer in catalog.layers:
        if layer.body != body:
            continue
        if layer.reference_only:
            continue
        if layer.radiometry == "synthetic" and not allow_synthetic:
            continue
        # A frame mismatch has no meaningful coverage value.
        if _is_absolute_frame_footprint(
            layer.footprint
        ) and not _polygon_footprint_is_comparable(crs, layer):
            continue
        footprint = parse_footprint(layer.footprint, crs=crs)
        inside, total = coverage_samples_inside(footprint, region)
        if inside <= 0:
            continue
        candidates.append((layer, inside / total, inside == total))

    candidates.sort(key=lambda c: _sort_key(c[0]))
    return candidates


def _select_candidates(
    candidates: list[_Candidate],
    *,
    kind: LayerKind,
    radiometry: Callable[[Radiometry | None], bool] | None = None,
) -> list[_Candidate]:
    """Filter the sorted candidates by kind and optional radiometry predicate."""
    selected = [c for c in candidates if c[0].kind == kind]
    if radiometry is not None:
        selected = [c for c in selected if radiometry(c[0].radiometry)]
    return selected


def _assign_roles(
    ordered: list[_Candidate], *, size_m: float, target_gsd_m: float
) -> tuple[LayerContribution, ...]:
    """Mark the first fully covering layer as base; all others are insets.

    This stack path preserves candidate order. Base-less non-DEM stacks are
    valid and do not trigger the DEM warning."""
    base_claimed = False
    contributions: list[LayerContribution] = []
    for layer, coverage, full in ordered:
        role: Literal["base", "inset"]
        if full and not base_claimed:
            role = "base"
            base_claimed = True
        else:
            role = "inset"
        contributions.append(
            _contribution(
                layer,
                role=role,
                coverage_fraction=coverage,
                size_m=size_m,
                target_gsd_m=target_gsd_m,
            )
        )
    return tuple(contributions)


def _dem_dominance_filter(
    dem_candidates: list[_Candidate],
) -> tuple[list[_Candidate], tuple[str, ...]]:
    """Choose the first full-coverage DEM as base; keep only finer partial-coverage
    insets. Without a base, return an empty stack and warning.
    """
    base_index = next(
        (i for i, (_layer, _cov, full) in enumerate(dem_candidates) if full), None
    )
    if base_index is None:
        return [], (_NO_BASE_WARNING,)

    base_layer, base_coverage, _base_full = dem_candidates[base_index]
    insets = [
        c
        for i, c in enumerate(dem_candidates)
        if i != base_index and not c[2] and c[0].native_gsd_m < base_layer.native_gsd_m
    ]
    ordered = [(base_layer, base_coverage, True), *insets]
    return ordered, ()


def _resolve_dem_stack(
    candidates: list[_Candidate], request: RegionRequest
) -> tuple[tuple[LayerContribution, ...], float, tuple[str, ...]]:
    """Resolve DEM candidates and derive target GSD from the dominance-filtered layers."""
    dem_selected = _select_candidates(candidates, kind="dem")
    dem_ordered, warnings = _dem_dominance_filter(dem_selected)

    target_gsd_m = _target_gsd_m(
        request, [layer.native_gsd_m for layer, _cov, _full in dem_ordered]
    )
    dem_stack = _assign_roles(
        dem_ordered, size_m=request.size_m, target_gsd_m=target_gsd_m
    )
    return dem_stack, target_gsd_m, warnings


def _resolve_kind_stack(
    candidates: list[_Candidate],
    *,
    kind: LayerKind,
    size_m: float,
    target_gsd_m: float,
    radiometry: Callable[[Radiometry | None], bool] | None = None,
) -> tuple[LayerContribution, ...]:
    """Resolve non-DEM stacks without dominance filtering."""
    selected = _select_candidates(candidates, kind=kind, radiometry=radiometry)
    return _assign_roles(selected, size_m=size_m, target_gsd_m=target_gsd_m)


def _is_sunlit_radiometry(radiometry: Radiometry | None) -> bool:
    """Include all colour radiometry except secondary illumination; untagged layers
    remain eligible.
    """
    return radiometry != "secondary_illumination"


def _is_psr_radiometry(radiometry: Radiometry | None) -> bool:
    """Include secondary-illumination and normalized-albedo layers in the PSR stack."""
    return radiometry in ("secondary_illumination", "normalized_albedo")


def _has_mask_candidate(candidates: list[_Candidate]) -> bool:
    return any(layer.kind == "mask" for layer, _cov, _full in candidates)


def _target_gsd_m(request: RegionRequest, dem_native_gsd_m: list[float]) -> float:
    if request.target_gsd_m is not None:
        return request.target_gsd_m
    if dem_native_gsd_m:
        return min(dem_native_gsd_m)
    return 0.0


def _fetch_dedup_key(layer: SourceLayer) -> tuple[str, str]:
    """Identify the fetch unit: a layer, or the pinned archive SHA-256 for ZIP members."""
    if layer.access == "zip_member":
        return ("zip_archive", layer.sha256[url_basename(layer.urls[0])])
    return ("layer", layer.layer_id)


def _total_fetch_bytes(
    *stacks: tuple[LayerContribution, ...], catalog: SourceCatalog
) -> int:
    """Sum each fetch unit once across stacks. ZIP members share one archive cost."""
    by_id = layers_by_id(catalog)
    by_fetch_unit: dict[tuple[str, str], int] = {}
    for stack in stacks:
        for contribution in stack:
            layer = by_id.get(contribution.layer_id)
            key = (
                _fetch_dedup_key(layer)
                if layer is not None
                else ("layer", contribution.layer_id)
            )
            by_fetch_unit[key] = contribution.est_fetch_bytes
    return sum(by_fetch_unit.values())


def shared_fetch_unit_layer_ids(
    plan: CompositionPlan, catalog: SourceCatalog
) -> frozenset[str]:
    """Return layer IDs sharing a fetch unit with another layer in the plan."""
    by_id = layers_by_id(catalog)
    members: dict[tuple[str, str], set[str]] = {}
    for stack in (
        plan.dem_stack,
        plan.color_stack,
        plan.color_stack_psr,
        plan.mask_layers,
        plan.quality_layers,
    ):
        for contribution in stack:
            layer = by_id.get(contribution.layer_id)
            key = (
                _fetch_dedup_key(layer)
                if layer is not None
                else ("layer", contribution.layer_id)
            )
            members.setdefault(key, set()).add(contribution.layer_id)
    return frozenset(
        layer_id
        for layer_ids in members.values()
        if len(layer_ids) > 1
        for layer_id in layer_ids
    )


def _consent_required_layer_ids(
    *stacks: tuple[LayerContribution, ...],
) -> tuple[str, ...]:
    """Return unique consent-required IDs in first-occurrence stack order."""
    seen: set[str] = set()
    ordered: list[str] = []
    for stack in stacks:
        for contribution in stack:
            if contribution.requires_consent and contribution.layer_id not in seen:
                seen.add(contribution.layer_id)
                ordered.append(contribution.layer_id)
    return tuple(ordered)


def _plan(
    request: RegionRequest,
    *,
    crs: str,
    center_xy_m: tuple[float, float],
    target_gsd_m: float,
    consented: bool,
    dem_stack: tuple[LayerContribution, ...] = (),
    color_stack: tuple[LayerContribution, ...] = (),
    color_stack_psr: tuple[LayerContribution, ...] = (),
    mask_layers: tuple[LayerContribution, ...] = (),
    quality_layers: tuple[LayerContribution, ...] = (),
    total_fetch_bytes: int = 0,
    consent_required_layer_ids: tuple[str, ...] = (),
    warnings: tuple[str, ...] = (),
) -> CompositionPlan:
    return CompositionPlan(
        request=request,
        crs=crs,
        center_xy_m=center_xy_m,
        target_gsd_m=target_gsd_m,
        dem_stack=dem_stack,
        color_stack=color_stack,
        color_stack_psr=color_stack_psr,
        mask_layers=mask_layers,
        quality_layers=quality_layers,
        total_fetch_bytes=total_fetch_bytes,
        consented=consented,
        consent_required_layer_ids=consent_required_layer_ids,
        warnings=warnings,
    )


def resolve_stacks(
    request: RegionRequest, catalog: SourceCatalog, *, consented: bool = False
) -> CompositionPlan:
    """Resolve all catalog candidates into DEM, colour, mask, and quality stacks. No
    coverage yields an empty stack and warning.
    """
    _check_body_match(catalog, request)

    crs = request.resolved_crs
    center_xy_m = latlon_to_xy(request.lat_deg, request.lon_deg, crs)
    region = Region(crs=crs, center_xy_m=center_xy_m, size_m=request.size_m)

    candidates = _build_sorted_candidates(
        catalog,
        body=request.body,
        region=region,
        crs=crs,
        allow_synthetic=request.allow_synthetic,
    )

    dem_stack, target_gsd_m, warnings = _resolve_dem_stack(candidates, request)

    color_stack = _resolve_kind_stack(
        candidates,
        kind="color",
        size_m=request.size_m,
        target_gsd_m=target_gsd_m,
        radiometry=_is_sunlit_radiometry,
    )

    if _has_mask_candidate(candidates):
        color_stack_psr = _resolve_kind_stack(
            candidates,
            kind="color",
            size_m=request.size_m,
            target_gsd_m=target_gsd_m,
            radiometry=_is_psr_radiometry,
        )
    else:
        color_stack_psr = ()

    mask_layers = _resolve_kind_stack(
        candidates, kind="mask", size_m=request.size_m, target_gsd_m=target_gsd_m
    )
    quality_layers = _resolve_kind_stack(
        candidates, kind="quality", size_m=request.size_m, target_gsd_m=target_gsd_m
    )

    total_fetch_bytes = _total_fetch_bytes(
        dem_stack,
        color_stack,
        color_stack_psr,
        mask_layers,
        quality_layers,
        catalog=catalog,
    )
    consent_required_layer_ids = (
        ()
        if consented
        else _consent_required_layer_ids(
            dem_stack, color_stack, color_stack_psr, mask_layers, quality_layers
        )
    )

    return _plan(
        request,
        crs=crs,
        center_xy_m=center_xy_m,
        target_gsd_m=target_gsd_m,
        dem_stack=dem_stack,
        color_stack=color_stack,
        color_stack_psr=color_stack_psr,
        mask_layers=mask_layers,
        quality_layers=quality_layers,
        total_fetch_bytes=total_fetch_bytes,
        consent_required_layer_ids=consent_required_layer_ids,
        warnings=warnings,
        consented=consented,
    )


_DeclaredCandidate = tuple[SourceLayer, StackRef, float]
"""(layer, ref, coverage_fraction) -- the declared-stack analog
of `_Candidate`, carrying the `StackRef` alongside the joined `SourceLayer` so
`role` can be read off it directly instead of derived from coverage."""


def _resolve_declared_refs(
    refs: Sequence[StackRef],
    by_id: dict[str, SourceLayer],
    *,
    body: Body,
    region: Region,
    crs: str,
    allow_reference_only: bool,
    expected_kind: Literal["dem", "color"],
) -> tuple[list[_DeclaredCandidate], list[str]]:
    """Join explicit refs to layers; reject unknown or duplicate IDs. Apply body, frame,
    and max-GSD filters; coverage does not control membership.
    """
    resolved: list[_DeclaredCandidate] = []
    warnings: list[str] = []
    seen_layer_ids: set[str] = set()
    for ref in refs:
        if ref.layer_id in seen_layer_ids:
            raise DeclaredLayerDuplicateError(ref.layer_id)
        seen_layer_ids.add(ref.layer_id)
        layer = by_id.get(ref.layer_id)
        if layer is None:
            raise DeclaredLayerUnknownError(ref.layer_id)
        if layer.kind != expected_kind:
            raise ValueError(
                f"declared {expected_kind} ref {ref.layer_id!r} resolves to kind={layer.kind!r}"
            )
        if layer.reference_only and not allow_reference_only:
            raise ReferenceOnlyLayerError(ref.layer_id)
        if layer.body != body:
            warnings.append(
                _DECLARED_BODY_MISMATCH_WARNING_FMT.format(
                    layer_id=ref.layer_id, layer_body=layer.body, body=body
                )
            )
            continue
        if ref.max_gsd_m is not None and layer.native_gsd_m > ref.max_gsd_m:
            warnings.append(
                _DECLARED_MAX_GSD_DROPPED_WARNING_FMT.format(
                    layer_id=ref.layer_id,
                    native_gsd_m=layer.native_gsd_m,
                    max_gsd_m=ref.max_gsd_m,
                )
            )
            continue
        # A frame mismatch has no meaningful coverage value.
        if _is_absolute_frame_footprint(
            layer.footprint
        ) and not _polygon_footprint_is_comparable(crs, layer):
            warnings.append(
                _DECLARED_FRAME_MISMATCH_WARNING_FMT.format(
                    layer_id=ref.layer_id,
                    authored_crs=layer.crs or CANONICAL_CRS[f"{layer.body}_south"],
                    crs=crs,
                )
            )
            continue
        footprint = parse_footprint(layer.footprint, crs=crs)
        inside, total = coverage_samples_inside(footprint, region)
        resolved.append((layer, ref, inside / total))
    return resolved, warnings


def _resolve_declared_dem_stack(
    dem_refs: Sequence[StackRef],
    by_id: dict[str, SourceLayer],
    *,
    region: Region,
    crs: str,
    request: RegionRequest,
    allow_reference_only: bool,
) -> tuple[tuple[LayerContribution, ...], float, list[str]]:
    """Resolve declared DEM refs, sort by the catalog key, then place the declared base
    first. A filtered base yields an empty stack and warning.
    """
    if sum(r.role == "base" for r in dem_refs) != 1:
        raise ValueError(
            "dem_refs must contain exactly one role='base' entry -- Sources "
            "already enforces this at the schema level (schema.py), so "
            f"reaching here with roles={[r.role for r in dem_refs]!r} means a "
            "caller bypassed that validator (defence in depth, not the primary "
            "gate)"
        )

    resolved, warnings = _resolve_declared_refs(
        dem_refs,
        by_id,
        body=request.body,
        region=region,
        crs=crs,
        allow_reference_only=allow_reference_only,
        expected_kind="dem",
    )
    resolved = sorted(resolved, key=lambda c: _sort_key(c[0]))

    base_index = next((i for i, c in enumerate(resolved) if c[1].role == "base"), None)
    if base_index is None:
        warnings.append(DECLARED_NO_BASE_WARNING)
        ordered: list[_DeclaredCandidate] = []
    else:
        base_entry = resolved[base_index]
        if base_entry[2] == 0.0:
            # Declared membership ignores coverage, so keep the base and warn.
            warnings.append(
                _DECLARED_BASE_ZERO_COVERAGE_WARNING_FMT.format(
                    layer_id=base_entry[0].layer_id
                )
            )
        rest = [c for i, c in enumerate(resolved) if i != base_index]
        ordered = [base_entry, *rest]

    target_gsd_m = _target_gsd_m(
        request, [layer.native_gsd_m for layer, _ref, _cov in ordered]
    )

    dem_stack = tuple(
        _contribution(
            layer,
            role=ref.role,
            coverage_fraction=coverage,
            size_m=request.size_m,
            target_gsd_m=target_gsd_m,
        )
        for layer, ref, coverage in ordered
    )
    return dem_stack, target_gsd_m, warnings


def _resolve_declared_color_stack(
    color_refs: Sequence[StackRef],
    by_id: dict[str, SourceLayer],
    *,
    body: Body,
    region: Region,
    crs: str,
    size_m: float,
    target_gsd_m: float,
    allow_reference_only: bool,
) -> tuple[tuple[LayerContribution, ...], list[str]]:
    """Resolve and sort declared colour refs; role comes from each ref and base is not
    forced first.
    """
    resolved, warnings = _resolve_declared_refs(
        color_refs,
        by_id,
        body=body,
        region=region,
        crs=crs,
        allow_reference_only=allow_reference_only,
        expected_kind="color",
    )
    resolved = sorted(resolved, key=lambda c: _sort_key(c[0]))
    color_stack = tuple(
        _contribution(
            layer,
            role=ref.role,
            coverage_fraction=coverage,
            size_m=size_m,
            target_gsd_m=target_gsd_m,
        )
        for layer, ref, coverage in resolved
    )
    return color_stack, warnings


def _resolve_declared_stacks(
    request: RegionRequest,
    catalog: SourceCatalog,
    *,
    dem_refs: Sequence[StackRef],
    color_refs: Sequence[StackRef] = (),
    consented: bool = False,
    allow_reference_only: bool,
) -> CompositionPlan:
    """Resolve manifest-declared DEM and colour layers. Roles come from refs; sorted
    priority controls order, with the DEM base first. No dominance filtering applies.
    Other stacks still come from footprint search over the catalog.
    """
    _check_body_match(catalog, request)

    crs = request.resolved_crs
    center_xy_m = latlon_to_xy(request.lat_deg, request.lon_deg, crs)
    region = Region(crs=crs, center_xy_m=center_xy_m, size_m=request.size_m)
    by_id = layers_by_id(catalog)

    dem_stack, target_gsd_m, dem_warnings = _resolve_declared_dem_stack(
        dem_refs,
        by_id,
        region=region,
        crs=crs,
        request=request,
        allow_reference_only=allow_reference_only,
    )
    color_stack, color_warnings = _resolve_declared_color_stack(
        color_refs,
        by_id,
        body=request.body,
        region=region,
        crs=crs,
        size_m=request.size_m,
        target_gsd_m=target_gsd_m,
        allow_reference_only=allow_reference_only,
    )

    candidates = _build_sorted_candidates(
        catalog,
        body=request.body,
        region=region,
        crs=crs,
        allow_synthetic=request.allow_synthetic,
    )

    if _has_mask_candidate(candidates):
        color_stack_psr = _resolve_kind_stack(
            candidates,
            kind="color",
            size_m=request.size_m,
            target_gsd_m=target_gsd_m,
            radiometry=_is_psr_radiometry,
        )
    else:
        color_stack_psr = ()

    mask_layers = _resolve_kind_stack(
        candidates, kind="mask", size_m=request.size_m, target_gsd_m=target_gsd_m
    )
    quality_layers = _resolve_kind_stack(
        candidates, kind="quality", size_m=request.size_m, target_gsd_m=target_gsd_m
    )

    total_fetch_bytes = _total_fetch_bytes(
        dem_stack,
        color_stack,
        color_stack_psr,
        mask_layers,
        quality_layers,
        catalog=catalog,
    )
    consent_required_layer_ids = (
        ()
        if consented
        else _consent_required_layer_ids(
            dem_stack, color_stack, color_stack_psr, mask_layers, quality_layers
        )
    )

    return _plan(
        request,
        crs=crs,
        center_xy_m=center_xy_m,
        target_gsd_m=target_gsd_m,
        dem_stack=dem_stack,
        color_stack=color_stack,
        color_stack_psr=color_stack_psr,
        mask_layers=mask_layers,
        quality_layers=quality_layers,
        total_fetch_bytes=total_fetch_bytes,
        consent_required_layer_ids=consent_required_layer_ids,
        warnings=(*dem_warnings, *color_warnings),
        consented=consented,
    )


def resolve_declared_stacks(
    request: RegionRequest,
    catalog: SourceCatalog,
    *,
    dem_refs: Sequence[StackRef],
    color_refs: Sequence[StackRef] = (),
    consented: bool = False,
) -> CompositionPlan:
    """Resolve production manifest stacks; reference-only layers fail closed."""
    return _resolve_declared_stacks(
        request,
        catalog,
        dem_refs=dem_refs,
        color_refs=color_refs,
        consented=consented,
        allow_reference_only=False,
    )


def resolve_declared_reference_stacks(
    request: RegionRequest,
    catalog: SourceCatalog,
    *,
    dem_refs: Sequence[StackRef],
    color_refs: Sequence[StackRef] = (),
    consented: bool = False,
) -> CompositionPlan:
    """Resolve explicit stacks with reference-only layers enabled for validation
    studies.
    """
    return _resolve_declared_stacks(
        request,
        catalog,
        dem_refs=dem_refs,
        color_refs=color_refs,
        consented=consented,
        allow_reference_only=True,
    )


def to_stack_refs(plan: CompositionPlan) -> tuple[list[StackRef], list[StackRef]]:
    """Return DEM and colour refs in plan order, preserving roles; max_gsd_m is unset."""
    dem_refs = [
        StackRef(layer_id=c.layer_id, role=c.role, max_gsd_m=None)
        for c in plan.dem_stack
    ]
    color_refs = [
        StackRef(layer_id=c.layer_id, role=c.role, max_gsd_m=None)
        for c in plan.color_stack
    ]
    return dem_refs, color_refs


__all__ = [
    "CANONICAL_CRS",
    "DECLARED_NO_BASE_WARNING",
    "FLOAT32_BYTES",
    "MAX_REGION_SIZE_M",
    "POLAR_LAT_DEG",
    "CompositionPlan",
    "LayerContribution",
    "RegionRequest",
    "est_fetch_bytes",
    "resolve_declared_stacks",
    "resolve_declared_reference_stacks",
    "resolve_stacks",
    "to_stack_refs",
]
