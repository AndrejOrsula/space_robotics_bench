"""Write fusion metadata and provenance/uncertainty rasters.

Blend indices are 0-based int16 (-1 means none); provenance rasters encode
them as uint8 (0 means none, 1..N index [*insets, base]). The base is last;
255 is reserved. Area fractions count winning pixels, so feathering can
reduce them below footprint fractions. Demoted shifts are retained. Metadata
is JSON-native; rasters use the PatchGrid georeference. Uncertainty is
cache-side and tagged. Warning formatting belongs to the engine.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, NoReturn, NotRequired, TypedDict, cast

import numpy as np
import rasterio
from rasterio.transform import Affine

from srb.terrain.catalog_schema import AlignmentSource, LayerKind, Radiometry
from srb.terrain.fusion.blend import _MAX_LAYERS
from srb.terrain.fusion.coregister import AppliedShift
from srb.terrain.fusion.grid import PatchGrid
from srb.utils.failures import IntegrityFailureError


class AppliedShiftMetadata(TypedDict):
    dx_m: float
    dy_m: float
    dz_m: float
    source: AlignmentSource
    residual_m: float | None
    demoted: bool
    reason: str | None


class AttributionMetadata(TypedDict):
    layer_id: str
    license: str
    attribution: str


class LayerProvenanceMetadata(AttributionMetadata):
    role: Literal["base", "inset"]
    kind: LayerKind
    index: int
    area_fraction: float
    native_gsd_m: float
    upsampled: bool
    radiometry: Radiometry | None
    alignment: AppliedShiftMetadata
    bytes_verified: NotRequired[bool]


class ContributorProvenanceMetadata(AttributionMetadata):
    role: Literal["base", "inset"]
    kind: LayerKind
    native_gsd_m: float
    radiometry: Radiometry | None
    bytes_verified: NotRequired[bool]


class FusionGridMetadata(TypedDict):
    crs: str
    center_xy_m: list[float]
    size_m: float
    gsd_m: float
    pixels: int


class FusionMetadata(TypedDict):
    fusion_version: int
    grid: FusionGridMetadata
    dem_layers: list[LayerProvenanceMetadata]
    color_layers: list[LayerProvenanceMetadata]
    contributors: list[ContributorProvenanceMetadata]
    provenance_tif: str
    uncertainty_tif: str | None
    warnings: list[str]


@dataclass(frozen=True, slots=True)
class LayerProvenance:
    """One layer record serialized in meta.json["fusion"]."""

    layer_id: str
    role: Literal["base", "inset"]
    kind: LayerKind
    index: int
    # 1-based provenance value: position in blend order `[*insets, base]`.
    # A pixel inside a layer's region must equal its index.
    area_fraction: float
    # Fraction of patch pixels won, measured from the provenance raster.
    native_gsd_m: float
    upsampled: bool  # native_gsd_m > the output grid's gsd_m
    radiometry: Radiometry | None
    alignment: AppliedShift
    license: str
    attribution: str


@dataclass(frozen=True, slots=True)
class ContributorProvenance:
    """A non-raster-indexed input that affects the fused output."""

    layer_id: str
    role: Literal["base", "inset"]
    kind: LayerKind
    native_gsd_m: float
    radiometry: Radiometry | None
    license: str
    attribution: str


_FUSION_METADATA_KEYS = frozenset(
    {
        "fusion_version",
        "grid",
        "dem_layers",
        "color_layers",
        "contributors",
        "provenance_tif",
        "uncertainty_tif",
        "warnings",
    }
)
_FUSION_GRID_KEYS = frozenset({"crs", "center_xy_m", "size_m", "gsd_m", "pixels"})
_LAYER_METADATA_KEYS = frozenset(
    {
        "layer_id",
        "role",
        "kind",
        "index",
        "area_fraction",
        "native_gsd_m",
        "upsampled",
        "radiometry",
        "alignment",
        "license",
        "attribution",
    }
)
_CONTRIBUTOR_METADATA_KEYS = frozenset(
    {
        "layer_id",
        "role",
        "kind",
        "native_gsd_m",
        "radiometry",
        "license",
        "attribution",
    }
)
_ALIGNMENT_METADATA_KEYS = frozenset(
    {"dx_m", "dy_m", "dz_m", "source", "residual_m", "demoted", "reason"}
)
_OPTIONAL_PROVENANCE_KEYS = frozenset({"bytes_verified"})
_ROLES = frozenset({"base", "inset"})
_LAYER_KINDS = frozenset({"dem", "color", "mask", "quality"})
_CONTRIBUTOR_KINDS = frozenset({"mask", "quality"})
_RADIOMETRIES = frozenset(
    {
        "normalized_albedo",
        "illumination_averaged",
        "single_illumination",
        "secondary_illumination",
        "synthetic",
    }
)
_ALIGNMENT_SOURCES = frozenset(
    {"published", "pc_align", "cross_correlation", "assumed_zero"}
)


def _fusion_metadata_error(source: str, problem: str) -> NoReturn:
    raise IntegrityFailureError(
        code="terrain.fusion.metadata_integrity",
        message=f"fusion metadata {source} is invalid: {problem}",
        component="terrain.fusion.provenance",
        details={"source": source},
        fallback_allowed=False,
        evidence_valid=False,
    )


def _metadata_object(value: object, *, source: str) -> dict[str, object]:
    if not isinstance(value, dict):
        _fusion_metadata_error(source, f"expected object, got {type(value).__name__}")
    result: dict[str, object] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            _fusion_metadata_error(source, "contains a non-string key")
        result[key] = item
    return result


def _metadata_exact_keys(
    mapping: dict[str, object],
    *,
    source: str,
    required: frozenset[str],
    optional: frozenset[str] = frozenset(),
) -> None:
    keys = frozenset(mapping)
    missing = sorted(required - keys)
    unknown = sorted(keys - required - optional)
    if missing or unknown:
        parts: list[str] = []
        if missing:
            parts.append(f"missing fields {missing}")
        if unknown:
            parts.append(f"unknown fields {unknown}")
        _fusion_metadata_error(source, "; ".join(parts))


def _metadata_value(mapping: dict[str, object], key: str, *, source: str) -> object:
    try:
        return mapping[key]
    except KeyError:
        _fusion_metadata_error(source, f"missing field {key!r}")


def _metadata_list(value: object, *, source: str) -> list[object]:
    if not isinstance(value, list):
        _fusion_metadata_error(source, f"expected list, got {type(value).__name__}")
    return list(value)


def _metadata_string(value: object, *, source: str, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        _fusion_metadata_error(source, f"expected string, got {type(value).__name__}")
    if not allow_empty and not value:
        _fusion_metadata_error(source, "must not be empty")
    return value


def _metadata_optional_string(value: object, *, source: str) -> str | None:
    if value is None:
        return None
    return _metadata_string(value, source=source, allow_empty=True)


def _metadata_choice(value: object, *, source: str, choices: frozenset[str]) -> str:
    item = _metadata_string(value, source=source)
    if item not in choices:
        _fusion_metadata_error(source, f"unsupported value {item!r}")
    return item


def _metadata_optional_choice(
    value: object, *, source: str, choices: frozenset[str]
) -> str | None:
    if value is None:
        return None
    return _metadata_choice(value, source=source, choices=choices)


def _metadata_bool(value: object, *, source: str) -> bool:
    if not isinstance(value, bool):
        _fusion_metadata_error(source, f"expected bool, got {type(value).__name__}")
    return value


def _metadata_int(value: object, *, source: str, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        _fusion_metadata_error(source, f"expected int, got {type(value).__name__}")
    if minimum is not None and value < minimum:
        _fusion_metadata_error(source, f"must be >= {minimum}, got {value}")
    return value


def _metadata_float(
    value: object,
    *,
    source: str,
    minimum: float | None = None,
    maximum: float | None = None,
    strictly_positive: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fusion_metadata_error(
            source, f"expected finite number, got {type(value).__name__}"
        )
    result = float(value)
    if not math.isfinite(result):
        _fusion_metadata_error(source, f"must be finite, got {result!r}")
    if strictly_positive and result <= 0.0:
        _fusion_metadata_error(source, f"must be > 0, got {result!r}")
    if minimum is not None and result < minimum:
        _fusion_metadata_error(source, f"must be >= {minimum}, got {result!r}")
    if maximum is not None and result > maximum:
        _fusion_metadata_error(source, f"must be <= {maximum}, got {result!r}")
    return result


def _metadata_member_name(value: object, *, source: str) -> str:
    name = _metadata_string(value, source=source)
    path = Path(name)
    if path.is_absolute() or path.name != name or name in {".", ".."}:
        _fusion_metadata_error(source, f"must be a generation-local filename: {name!r}")
    if path.suffix.lower() not in {".tif", ".tiff"}:
        _fusion_metadata_error(source, f"must name a TIFF member: {name!r}")
    return name


def _validate_applied_shift_metadata(
    value: object, *, source: str
) -> AppliedShiftMetadata:
    mapping = _metadata_object(value, source=source)
    _metadata_exact_keys(
        mapping,
        source=source,
        required=_ALIGNMENT_METADATA_KEYS,
    )
    residual_value = _metadata_value(mapping, "residual_m", source=source)
    residual = (
        None
        if residual_value is None
        else _metadata_float(
            residual_value,
            source=f"{source}.residual_m",
            minimum=0.0,
        )
    )
    return {
        "dx_m": _metadata_float(
            _metadata_value(mapping, "dx_m", source=source),
            source=f"{source}.dx_m",
        ),
        "dy_m": _metadata_float(
            _metadata_value(mapping, "dy_m", source=source),
            source=f"{source}.dy_m",
        ),
        "dz_m": _metadata_float(
            _metadata_value(mapping, "dz_m", source=source),
            source=f"{source}.dz_m",
        ),
        "source": cast(
            AlignmentSource,
            _metadata_choice(
                _metadata_value(mapping, "source", source=source),
                source=f"{source}.source",
                choices=_ALIGNMENT_SOURCES,
            ),
        ),
        "residual_m": residual,
        "demoted": _metadata_bool(
            _metadata_value(mapping, "demoted", source=source),
            source=f"{source}.demoted",
        ),
        "reason": _metadata_optional_string(
            _metadata_value(mapping, "reason", source=source),
            source=f"{source}.reason",
        ),
    }


def _validate_layer_metadata(
    value: object,
    *,
    source: str,
    expected_kind: Literal["dem", "color"],
    grid_gsd_m: float,
) -> LayerProvenanceMetadata:
    mapping = _metadata_object(value, source=source)
    _metadata_exact_keys(
        mapping,
        source=source,
        required=_LAYER_METADATA_KEYS,
        optional=_OPTIONAL_PROVENANCE_KEYS,
    )
    kind = _metadata_choice(
        _metadata_value(mapping, "kind", source=source),
        source=f"{source}.kind",
        choices=_LAYER_KINDS,
    )
    if kind != expected_kind:
        _fusion_metadata_error(
            f"{source}.kind", f"expected {expected_kind!r}, got {kind!r}"
        )
    native_gsd_m = _metadata_float(
        _metadata_value(mapping, "native_gsd_m", source=source),
        source=f"{source}.native_gsd_m",
        strictly_positive=True,
    )
    upsampled = _metadata_bool(
        _metadata_value(mapping, "upsampled", source=source),
        source=f"{source}.upsampled",
    )
    expected_upsampled = native_gsd_m > grid_gsd_m
    if upsampled is not expected_upsampled:
        _fusion_metadata_error(
            f"{source}.upsampled",
            f"got {upsampled}, expected {expected_upsampled} from native_gsd_m/grid.gsd_m",
        )
    radiometry = _metadata_optional_choice(
        _metadata_value(mapping, "radiometry", source=source),
        source=f"{source}.radiometry",
        choices=_RADIOMETRIES,
    )
    record: LayerProvenanceMetadata = {
        "layer_id": _metadata_string(
            _metadata_value(mapping, "layer_id", source=source),
            source=f"{source}.layer_id",
        ),
        "role": cast(
            Literal["base", "inset"],
            _metadata_choice(
                _metadata_value(mapping, "role", source=source),
                source=f"{source}.role",
                choices=_ROLES,
            ),
        ),
        "kind": cast(LayerKind, kind),
        "index": _metadata_int(
            _metadata_value(mapping, "index", source=source),
            source=f"{source}.index",
            minimum=1,
        ),
        "area_fraction": _metadata_float(
            _metadata_value(mapping, "area_fraction", source=source),
            source=f"{source}.area_fraction",
            minimum=0.0,
            maximum=1.0,
        ),
        "native_gsd_m": native_gsd_m,
        "upsampled": upsampled,
        "radiometry": cast(Radiometry | None, radiometry),
        "alignment": _validate_applied_shift_metadata(
            _metadata_value(mapping, "alignment", source=source),
            source=f"{source}.alignment",
        ),
        "license": _metadata_string(
            _metadata_value(mapping, "license", source=source),
            source=f"{source}.license",
        ),
        "attribution": _metadata_string(
            _metadata_value(mapping, "attribution", source=source),
            source=f"{source}.attribution",
        ),
    }
    if "bytes_verified" in mapping:
        record["bytes_verified"] = _metadata_bool(
            mapping["bytes_verified"], source=f"{source}.bytes_verified"
        )
    return record


def _validate_layer_metadata_list(
    value: object,
    *,
    source: str,
    expected_kind: Literal["dem", "color"],
    grid_gsd_m: float,
    require_base: bool,
) -> list[LayerProvenanceMetadata]:
    items = _metadata_list(value, source=source)
    if len(items) > _MAX_LAYERS:
        _fusion_metadata_error(
            source, f"contains {len(items)} layers; max is {_MAX_LAYERS}"
        )
    layers = [
        _validate_layer_metadata(
            item,
            source=f"{source}[{index}]",
            expected_kind=expected_kind,
            grid_gsd_m=grid_gsd_m,
        )
        for index, item in enumerate(items)
    ]
    layer_ids = [layer["layer_id"] for layer in layers]
    if len(set(layer_ids)) != len(layer_ids):
        _fusion_metadata_error(source, "contains duplicate layer_id values")
    bases = [layer for layer in layers if layer["role"] == "base"]
    if require_base and len(bases) != 1:
        _fusion_metadata_error(
            source, f"has {len(bases)} base layers, expected exactly 1"
        )
    if not require_base and len(bases) > 1:
        _fusion_metadata_error(
            source, f"has {len(bases)} base layers, expected at most 1"
        )
    n_layers = len(layers)
    if bases:
        (base,) = bases
        if base["index"] != n_layers:
            _fusion_metadata_error(
                source,
                f"base layer index is {base['index']}, expected {n_layers}",
            )
        expected_inset_indexes = list(range(1, n_layers))
    else:
        expected_inset_indexes = list(range(1, n_layers + 1))
    actual_inset_indexes = sorted(
        layer["index"] for layer in layers if layer["role"] == "inset"
    )
    if actual_inset_indexes != expected_inset_indexes:
        _fusion_metadata_error(
            source,
            f"inset indexes {actual_inset_indexes} are not {expected_inset_indexes}",
        )
    return layers


def _validate_contributor_metadata(
    value: object, *, source: str
) -> ContributorProvenanceMetadata:
    mapping = _metadata_object(value, source=source)
    _metadata_exact_keys(
        mapping,
        source=source,
        required=_CONTRIBUTOR_METADATA_KEYS,
        optional=_OPTIONAL_PROVENANCE_KEYS,
    )
    radiometry = _metadata_optional_choice(
        _metadata_value(mapping, "radiometry", source=source),
        source=f"{source}.radiometry",
        choices=_RADIOMETRIES,
    )
    record: ContributorProvenanceMetadata = {
        "layer_id": _metadata_string(
            _metadata_value(mapping, "layer_id", source=source),
            source=f"{source}.layer_id",
        ),
        "role": cast(
            Literal["base", "inset"],
            _metadata_choice(
                _metadata_value(mapping, "role", source=source),
                source=f"{source}.role",
                choices=_ROLES,
            ),
        ),
        "kind": cast(
            LayerKind,
            _metadata_choice(
                _metadata_value(mapping, "kind", source=source),
                source=f"{source}.kind",
                choices=_CONTRIBUTOR_KINDS,
            ),
        ),
        "native_gsd_m": _metadata_float(
            _metadata_value(mapping, "native_gsd_m", source=source),
            source=f"{source}.native_gsd_m",
            strictly_positive=True,
        ),
        "radiometry": cast(Radiometry | None, radiometry),
        "license": _metadata_string(
            _metadata_value(mapping, "license", source=source),
            source=f"{source}.license",
        ),
        "attribution": _metadata_string(
            _metadata_value(mapping, "attribution", source=source),
            source=f"{source}.attribution",
        ),
    }
    if "bytes_verified" in mapping:
        record["bytes_verified"] = _metadata_bool(
            mapping["bytes_verified"], source=f"{source}.bytes_verified"
        )
    return record


def _validate_contributor_metadata_list(
    value: object, *, source: str
) -> list[ContributorProvenanceMetadata]:
    items = _metadata_list(value, source=source)
    contributors = [
        _validate_contributor_metadata(item, source=f"{source}[{index}]")
        for index, item in enumerate(items)
    ]
    layer_ids = [record["layer_id"] for record in contributors]
    if len(set(layer_ids)) != len(layer_ids):
        _fusion_metadata_error(source, "contains duplicate layer_id values")
    return contributors


def validate_fusion_metadata(value: object, *, source: str) -> FusionMetadata:
    """Validate a persisted fusion block, rejecting unknown fields."""
    mapping = _metadata_object(value, source=source)
    _metadata_exact_keys(
        mapping,
        source=source,
        required=_FUSION_METADATA_KEYS,
    )
    grid_source = f"{source}.grid"
    grid_mapping = _metadata_object(
        _metadata_value(mapping, "grid", source=source), source=grid_source
    )
    _metadata_exact_keys(
        grid_mapping,
        source=grid_source,
        required=_FUSION_GRID_KEYS,
    )
    center_source = f"{grid_source}.center_xy_m"
    center_values = _metadata_list(
        _metadata_value(grid_mapping, "center_xy_m", source=grid_source),
        source=center_source,
    )
    if len(center_values) != 2:
        _fusion_metadata_error(center_source, "must contain exactly two values")
    size_m = _metadata_float(
        _metadata_value(grid_mapping, "size_m", source=grid_source),
        source=f"{grid_source}.size_m",
        strictly_positive=True,
    )
    gsd_m = _metadata_float(
        _metadata_value(grid_mapping, "gsd_m", source=grid_source),
        source=f"{grid_source}.gsd_m",
        strictly_positive=True,
    )
    pixels = _metadata_int(
        _metadata_value(grid_mapping, "pixels", source=grid_source),
        source=f"{grid_source}.pixels",
        minimum=1,
    )
    expected_pixels = int(round(size_m / gsd_m))
    if pixels != expected_pixels:
        _fusion_metadata_error(
            f"{grid_source}.pixels",
            f"got {pixels}, expected round(size_m / gsd_m) == {expected_pixels}",
        )
    grid: FusionGridMetadata = {
        "crs": _metadata_string(
            _metadata_value(grid_mapping, "crs", source=grid_source),
            source=f"{grid_source}.crs",
        ),
        "center_xy_m": [
            _metadata_float(center_values[0], source=f"{center_source}[0]"),
            _metadata_float(center_values[1], source=f"{center_source}[1]"),
        ],
        "size_m": size_m,
        "gsd_m": gsd_m,
        "pixels": pixels,
    }
    dem_layers = _validate_layer_metadata_list(
        _metadata_value(mapping, "dem_layers", source=source),
        source=f"{source}.dem_layers",
        expected_kind="dem",
        grid_gsd_m=gsd_m,
        require_base=True,
    )
    color_layers = _validate_layer_metadata_list(
        _metadata_value(mapping, "color_layers", source=source),
        source=f"{source}.color_layers",
        expected_kind="color",
        grid_gsd_m=gsd_m,
        require_base=False,
    )
    contributors = _validate_contributor_metadata_list(
        _metadata_value(mapping, "contributors", source=source),
        source=f"{source}.contributors",
    )
    warning_values = _metadata_list(
        _metadata_value(mapping, "warnings", source=source),
        source=f"{source}.warnings",
    )
    warnings = [
        _metadata_string(
            warning,
            source=f"{source}.warnings[{index}]",
            allow_empty=True,
        )
        for index, warning in enumerate(warning_values)
    ]
    uncertainty_value = _metadata_value(mapping, "uncertainty_tif", source=source)
    uncertainty_tif = (
        None
        if uncertainty_value is None
        else _metadata_member_name(
            uncertainty_value, source=f"{source}.uncertainty_tif"
        )
    )
    _validate_uncertainty_contributor_contract(
        uncertainty_tif=uncertainty_tif,
        contributors=contributors,
        source=source,
    )
    return {
        "fusion_version": _metadata_int(
            _metadata_value(mapping, "fusion_version", source=source),
            source=f"{source}.fusion_version",
            minimum=1,
        ),
        "grid": grid,
        "dem_layers": dem_layers,
        "color_layers": color_layers,
        "contributors": contributors,
        "provenance_tif": _metadata_member_name(
            _metadata_value(mapping, "provenance_tif", source=source),
            source=f"{source}.provenance_tif",
        ),
        "uncertainty_tif": uncertainty_tif,
        "warnings": warnings,
    }


def fusion_attributions(metadata: FusionMetadata) -> list[str]:
    return [
        record["attribution"]
        for records in (
            metadata["dem_layers"],
            metadata["color_layers"],
            metadata["contributors"],
        )
        for record in records
    ]


def format_fusion_attribution(
    metadata: FusionMetadata,
) -> str:
    records: list[AttributionMetadata] = [
        *metadata["dem_layers"],
        *metadata["color_layers"],
        *metadata["contributors"],
    ]
    return "\n".join(
        f"layer_id: {record['layer_id']}\n"
        f"license: {record['license']}\n"
        f"attribution: {record['attribution']}\n"
        for record in records
    )


def provenance_raster(prov_index: np.ndarray, n_layers: int) -> np.ndarray:
    """Encode blend indices as uint8: zero means none; 1..N are 1-based.

    Pass the original layer count, including layers that won no pixels. Reject counts
    above 254 and indices outside [-1, n_layers - 1] to prevent wrapping.
    """
    if n_layers > _MAX_LAYERS:
        raise ValueError(
            f"provenance_raster: n_layers={n_layers} exceeds the "
            f"{_MAX_LAYERS}-layer cap (the provenance raster is uint8, "
            "1-based 1..N indices)"
        )
    if prov_index.size:
        lo, hi = int(prov_index.min()), int(prov_index.max())
        if lo < -1 or hi > n_layers - 1:
            raise ValueError(
                f"provenance_raster: prov_index values must be in "
                f"[-1, {n_layers - 1}] for n_layers={n_layers}, got range "
                f"[{lo}, {hi}] -- this must be priority_blend's own 0-based "
                "prov_index for a `layers` sequence of exactly n_layers "
                "elements, not an arbitrary caller-built array"
            )
    shifted = prov_index.astype(np.int32) + 1
    return shifted.astype(np.uint8)


def layer_area_fraction(raster: np.ndarray, *, index: int) -> float:
    """Return the fraction of provenance pixels equal to index.

    Requires uint8 input so raw 0-based blend indices cannot shift the result.
    """
    if raster.dtype != np.uint8:
        raise ValueError(
            f"layer_area_fraction: raster.dtype={raster.dtype!r} must be "
            "uint8 (provenance_raster's output -- not priority_blend's raw "
            "int16 prov_index, which is 0-based and would misalign every "
            "index by one)"
        )
    return float(np.count_nonzero(raster == index)) / float(raster.size)


def layer_upsampled(native_gsd_m: float, grid_gsd_m: float) -> bool:
    return native_gsd_m > grid_gsd_m


def _write_single_band_tif(
    path: Path, array: np.ndarray, grid: PatchGrid, *, nodata: float | None
) -> None:
    """Write a compressed band on the grid; reject shape mismatches before opening
    the destination."""
    if array.shape != grid.shape:
        raise ValueError(
            f"{path.name}: array.shape={array.shape!r} != grid.shape={grid.shape!r}"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        height=grid.pixels,
        width=grid.pixels,
        count=1,
        dtype=array.dtype,
        crs=grid.crs,
        transform=Affine(*grid.transform_a),
        nodata=nodata,
        compress="deflate",
    ) as dst:
        dst.write(array, 1)


def write_provenance_tif(path: Path, raster: np.ndarray, grid: PatchGrid) -> None:
    """Write uint8 provenance; zero is data, so no nodata value is set."""
    if raster.dtype != np.uint8:
        raise ValueError(
            f"write_provenance_tif: raster.dtype={raster.dtype!r} must be "
            "uint8 (the provenance encoding -- see provenance_raster)"
        )
    _write_single_band_tif(path, raster, grid, nodata=None)


def write_uncertainty_tif(path: Path, sigma: np.ndarray, grid: PatchGrid) -> None:
    """Write cache-side float32 uncertainty with NaN nodata."""
    _write_single_band_tif(
        path, sigma.astype(np.float32, copy=False), grid, nodata=float("nan")
    )


def write_uncertainty_contract_tags(
    path: Path,
    *,
    source_layer_ids: Sequence[str],
) -> None:
    """Tag uncertainty units, representation, composition, and source IDs.

        Quality values use resolver-order first-finite selection; SRB does not
    estimate uncertainty.
    """
    from json import dumps as json_dumps

    from rasterio import open as open_raster

    layer_ids = [str(layer_id) for layer_id in source_layer_ids]
    if not layer_ids:
        raise ValueError(
            "uncertainty.tif requires at least one kind='quality' source layer"
        )
    if any(not layer_id for layer_id in layer_ids):
        raise ValueError("uncertainty.tif source layer ids must be non-empty")
    if len(set(layer_ids)) != len(layer_ids):
        raise ValueError("uncertainty.tif source layer ids must be unique")

    with open_raster(path, "r+") as dataset:
        dataset.update_tags(
            ns="SRB_UNCERTAINTY",
            measure="vertical_one_sigma_m",
            units="m",
            dtype="float32",
            nodata="NaN",
            composition="resolver_order_first_finite",
            srb_estimator="none",
            source_layer_ids=json_dumps(layer_ids, separators=(",", ":")),
        )


def _validate_uncertainty_contributor_contract(
    *,
    uncertainty_tif: str | None,
    contributors: Sequence[ContributorProvenanceMetadata],
    source: str,
) -> None:
    quality_layer_ids = [
        contributor["layer_id"]
        for contributor in contributors
        if contributor["kind"] == "quality"
    ]
    if uncertainty_tif is None and quality_layer_ids:
        _fusion_metadata_error(
            source, "kind='quality' contributors require uncertainty_tif"
        )
    if uncertainty_tif is not None and not quality_layer_ids:
        _fusion_metadata_error(
            source,
            "uncertainty_tif requires at least one kind='quality' contributor",
        )


def _applied_shift_dict(shift: AppliedShift) -> dict:
    return {
        "dx_m": float(shift.dx_m),
        "dy_m": float(shift.dy_m),
        "dz_m": float(shift.dz_m),
        "source": str(shift.source),
        "residual_m": (
            float(shift.residual_m) if shift.residual_m is not None else None
        ),
        "demoted": bool(shift.demoted),
        "reason": str(shift.reason) if shift.reason is not None else None,
    }


def _layer_provenance_dict(layer: LayerProvenance) -> dict:
    return {
        "layer_id": str(layer.layer_id),
        "role": str(layer.role),
        "kind": str(layer.kind),
        "index": int(layer.index),
        "area_fraction": float(layer.area_fraction),
        "native_gsd_m": float(layer.native_gsd_m),
        "upsampled": bool(layer.upsampled),
        "radiometry": str(layer.radiometry) if layer.radiometry is not None else None,
        "alignment": _applied_shift_dict(layer.alignment),
        "license": str(layer.license),
        "attribution": str(layer.attribution),
    }


def _contributor_provenance_dict(layer: ContributorProvenance) -> dict:
    return {
        "layer_id": str(layer.layer_id),
        "role": str(layer.role),
        "kind": str(layer.kind),
        "native_gsd_m": float(layer.native_gsd_m),
        "radiometry": str(layer.radiometry) if layer.radiometry is not None else None,
        "license": str(layer.license),
        "attribution": str(layer.attribution),
    }


def _validate_layer_index_contract(
    layers: Sequence[LayerProvenance], *, field: str
) -> None:
    """Check base-last and gap-free layer indexes without raster data.

    At most one base is allowed; its index is len(layers). Insets fill the
    remaining 1-based indexes. Empty lists are valid.
    """
    if not layers:
        return
    n = len(layers)
    bases = [layer for layer in layers if layer.role == "base"]
    if len(bases) > 1:
        raise ValueError(
            f"{field}: {len(bases)} layers have role=='base', expected at most 1"
        )
    if bases:
        (base,) = bases
        if base.index != n:
            raise ValueError(
                f"{field}: role=='base' layer {base.layer_id!r} has "
                f"index={base.index}, expected {n} (== len({field})) -- the "
                "gap-free base is LAST in blend order (this module's "
                "docstring, contract 2); a dem_stack/color_stack-order "
                "assignment mistake produces index==1 instead"
            )
        expected_inset_indexes = list(range(1, n))
    else:
        expected_inset_indexes = list(range(1, n + 1))
    inset_indexes = sorted(layer.index for layer in layers if layer.role == "inset")
    if inset_indexes != expected_inset_indexes:
        raise ValueError(
            f"{field}: inset layer indexes {inset_indexes} are not exactly "
            f"{expected_inset_indexes}"
        )


def fusion_meta_block(
    *,
    grid: PatchGrid,
    dem_layers: Sequence[LayerProvenance],
    color_layers: Sequence[LayerProvenance],
    fusion_version: int,
    provenance_tif_name: str | None,
    uncertainty_tif_name: str | None,
    warnings: Sequence[str],
    contributors: Sequence[ContributorProvenance] = (),
) -> dict:
    """Build the JSON-safe meta.json["fusion"] block.

    Preserve grid.crs (which may differ from top-level georef.crs) and input
    list order. Raster names are generation-local. Quality and uncertainty
    contributors must occur together. Layer indexes are validated.
    """
    _validate_layer_index_contract(dem_layers, field="dem_layers")
    _validate_layer_index_contract(color_layers, field="color_layers")
    contributor_records = [
        _contributor_provenance_dict(layer) for layer in contributors
    ]
    uncertainty_tif = (
        str(uncertainty_tif_name) if uncertainty_tif_name is not None else None
    )
    _validate_uncertainty_contributor_contract(
        uncertainty_tif=uncertainty_tif,
        contributors=contributor_records,
        source="fusion_meta_block",
    )
    return {
        "fusion_version": int(fusion_version),
        "grid": {
            "crs": str(grid.crs),
            "center_xy_m": [float(grid.center_xy_m[0]), float(grid.center_xy_m[1])],
            "size_m": float(grid.size_m),
            "gsd_m": float(grid.gsd_m),
            "pixels": int(grid.pixels),
        },
        "dem_layers": [_layer_provenance_dict(layer) for layer in dem_layers],
        "color_layers": [_layer_provenance_dict(layer) for layer in color_layers],
        "contributors": contributor_records,
        "provenance_tif": (
            str(provenance_tif_name) if provenance_tif_name is not None else None
        ),
        "uncertainty_tif": uncertainty_tif,
        "warnings": [str(w) for w in warnings],
    }
