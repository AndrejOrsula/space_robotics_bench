"""Import external illumination products into ``srb_illum/2``.

Supports lunarlab Zarr/NPZ and PGDA LPSR GeoTIFF sources. Heavy dependencies
load lazily. Imports reproject onto a north-up ``ProductGrid`` and correct
azimuth for grid convergence and elevation for source-to-target parallax.
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import zipfile
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, NoReturn, Optional

import numpy as np

from srb.core.ephemeris.products import (
    SCHEMA_ID,
    IlluminationProduct,
    ProductGrid,
    build_product_identity,
    save_product,
)
from srb.core.terrain_metadata import (
    TerrainMetadataWriter,
    resolve_terrain_metadata_writer,
)
from srb.utils.failures import (
    FailureError,
    IntegrityFailureError,
    InternalInvariantError,
    InvalidInputError,
    OptionalDependencyUnavailableError,
    TransientExternalFailureError,
)
from srb.utils.resource_budget import (
    MAX_ILLUMINATION_ARCHIVE_BYTES,
    MAX_ILLUMINATION_ARRAY_BYTES,
    MAX_ILLUMINATION_DECODED_BYTES,
    MAX_METADATA_AGGREGATE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    ByteBudget,
    checked_shape_nbytes,
    inspect_npz_archive,
    require_directory_budget,
    require_file_size,
)

_DEFAULT_SUN_ANGULAR_DIAMETER_DEG = 0.53
# Scale the default solar disk by each body's mean heliocentric distance.
_MEAN_HELIOCENTRIC_DISTANCE_AU = {"moon": 1.0, "mars": 1.523679, "earth": 1.0}
_BODY_RADIUS_M = {"moon": 1_737_400.0, "mars": 3_389_500.0, "earth": 6_371_000.0}

_EPOCH_ARRAY_KEYS = ("sunlit", "earth_los", "sun_fraction")
_AZEL_KEYS = ("sun_azel", "earth_azel")
_STATIC_KEYS = ("psr", "lit_fraction", "max_sun_el")
_ALLOCATION_BUDGET_KEY = "_allocation_budget"

# Raw epoch tier fields; optional and derived fields do not determine completeness.
_EPOCH_RAW_REQUIRED = ("sunlit", "sun_azel", "utcs")

_TT_UTC_OFFSET_VALID_FROM = "2017-01-01T00:00:00"
_IMPORTER_COMPONENT = "ephemeris.importers"


def _new_allocation_budget() -> ByteBudget:
    """Create the cumulative allocation budget shared by all import stages.

    Retained source arrays overlap later selections and destinations, so
    reservations remain charged after each stage returns.
    """
    return ByteBudget(
        "illumination importer allocations", MAX_ILLUMINATION_DECODED_BYTES
    )


def _allocation_budget(arrays: dict[str, Any]) -> ByteBudget | None:
    candidate = arrays.get(_ALLOCATION_BUDGET_KEY)
    return candidate if isinstance(candidate, ByteBudget) else None


def _reserve_retained_arrays(
    arrays: dict[str, Any], budget: ByteBudget, *, resource_prefix: str
) -> None:
    """Admit already-materialized arrays for callers without a reader budget."""
    for key, value in arrays.items():
        if key.startswith("_") or not isinstance(value, np.ndarray):
            continue
        budget.consume(
            checked_shape_nbytes(
                value.shape,
                value.dtype,
                resource=f"{resource_prefix} {key}",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
        )


def _nested_value_shape(value: object, *, resource: str) -> tuple[int, ...]:
    """Inspect a JSON-like value shape without materializing a NumPy array."""
    if isinstance(value, np.ndarray):
        return tuple(int(dimension) for dimension in value.shape)
    if isinstance(value, (list, tuple)):
        if not value:
            return (0,)
        child_shape = _nested_value_shape(value[0], resource=resource)
        if any(
            _nested_value_shape(child, resource=resource) != child_shape
            for child in value[1:]
        ):
            raise ValueError(f"{resource} has a ragged shape")
        return (len(value), *child_shape)
    return ()


def _nested_text_width(value: object) -> int:
    if isinstance(value, (list, tuple)):
        return max((_nested_text_width(child) for child in value), default=1)
    return max(1, len(str(value)))


def _value_shape_dtype(
    value: object, *, dtype: np.dtype[Any] | None = None, resource: str
) -> tuple[tuple[int, ...], np.dtype[Any]]:
    """Return shape/dtype for an array-like value without coercing it."""
    if isinstance(value, np.ndarray):
        return tuple(int(dimension) for dimension in value.shape), np.dtype(
            dtype if dtype is not None else value.dtype
        )
    shape = _nested_value_shape(value, resource=resource)
    if dtype is not None:
        return shape, np.dtype(dtype)
    # Unicode sizing bounds JSON-like inputs without materializing an array.
    return shape, np.dtype(f"<U{_nested_text_width(value)}")


def _reserve_value_array(
    budget: ByteBudget,
    value: object,
    *,
    dtype: np.dtype[Any] | None = None,
    resource: str,
) -> tuple[int, ...]:
    shape, materialized_dtype = _value_shape_dtype(
        value, dtype=dtype, resource=resource
    )
    budget.consume(
        checked_shape_nbytes(
            shape,
            materialized_dtype,
            resource=resource,
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
    )
    return shape


def _reserve_azel_correction(
    budget: ByteBudget, value: object, *, resource: str
) -> tuple[int, ...]:
    """Admit all arrays used by the azimuth/elevation correction expression."""
    shape, _source_dtype = _value_shape_dtype(value, resource=resource)
    if len(shape) != 2 or shape[1] != 2:
        raise ValueError(f"{resource} must have shape (T, 2), got {shape!r}")
    epochs = shape[0]
    vector = (epochs,)
    # Charge every intermediate to the cumulative operation budget.
    stages = (
        (shape, "input dtype conversion"),
        (vector, "azimuth wrap"),
        (vector, "azimuth radians"),
        (vector, "azimuth sine"),
        (vector, "azimuth sine scale"),
        (vector, "azimuth cosine"),
        (vector, "azimuth cosine scale"),
        (vector, "parallel displacement"),
        (vector, "parallel displacement radians"),
        (vector, "parallel displacement degrees"),
        (vector, "site elevation"),
        (shape, "corrected azimuth/elevation"),
    )
    for stage_shape, stage_name in stages:
        budget.consume(
            checked_shape_nbytes(
                stage_shape,
                np.dtype(np.float64),
                resource=f"{resource} {stage_name}",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
        )
    return shape


def _reserve_post_reprojection_materialization(
    budget: ByteBudget,
    arrays: dict[str, Any],
    reproj: dict[str, Any],
    *,
    has_epochs: bool,
) -> None:
    """Admit correction temporaries and product value-object snapshots."""
    for key in ("sun_azel", "earth_azel"):
        value = arrays.get(key)
        if value is None:
            continue
        shape = _reserve_azel_correction(
            budget,
            value,
            resource=f"illumination {key} correction",
        )
        if has_epochs:
            budget.consume(
                checked_shape_nbytes(
                    shape,
                    np.dtype(np.float64),
                    resource=f"illumination {key} product snapshot",
                    max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                )
            )
            # `save_product` takes another snapshot.
            budget.consume(
                checked_shape_nbytes(
                    shape,
                    np.dtype(np.float64),
                    resource=f"illumination {key} save_product snapshot",
                    max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                )
            )

    if has_epochs:
        times = arrays.get("utcs")
        if times is not None:
            times_shape = _reserve_value_array(
                budget,
                times,
                resource="illumination times_utc product materialization",
            )
            budget.consume(
                checked_shape_nbytes(
                    times_shape,
                    _value_shape_dtype(
                        times,
                        resource="illumination times_utc product snapshot",
                    )[1],
                    resource="illumination times_utc product snapshot",
                    max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                )
            )
            budget.consume(
                checked_shape_nbytes(
                    times_shape,
                    _value_shape_dtype(
                        times,
                        resource="illumination times_utc save_product snapshot",
                    )[1],
                    resource="illumination times_utc save_product snapshot",
                    max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                )
            )
        sun_ang_diam = arrays.get("sun_ang_diam")
        if sun_ang_diam is not None:
            angular_shape = _reserve_value_array(
                budget,
                sun_ang_diam,
                dtype=np.dtype(np.float64),
                resource="illumination sun angular diameter product materialization",
            )
            budget.consume(
                checked_shape_nbytes(
                    angular_shape,
                    np.dtype(np.float64),
                    resource="illumination sun angular diameter product snapshot",
                    max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                )
            )
            budget.consume(
                checked_shape_nbytes(
                    angular_shape,
                    np.dtype(np.float64),
                    resource="illumination sun angular diameter save_product snapshot",
                    max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                )
            )
        for key in ("sunlit", "sun_fraction", "earth_los"):
            value = reproj.get(key)
            if value is not None:
                _reserve_value_array(
                    budget,
                    value,
                    resource=f"illumination {key} product snapshot",
                )
                _reserve_value_array(
                    budget,
                    value,
                    resource=f"illumination {key} save_product snapshot",
                )

    for key in _STATIC_KEYS:
        value = reproj.get(key)
        if value is not None:
            _reserve_value_array(
                budget,
                value,
                resource=f"illumination {key} product snapshot",
            )
            _reserve_value_array(
                budget,
                value,
                resource=f"illumination {key} save_product snapshot",
            )


def _failure_details(
    *, source: object | None = None, cause: BaseException | None = None
) -> dict[str, object]:
    details: dict[str, object] = {}
    if source is not None:
        details["source"] = str(source)
    if cause is not None:
        details["cause_type"] = f"{type(cause).__module__}.{type(cause).__qualname__}"
    return details


def _raise_importer_invalid(
    message: str,
    *,
    code: str = "ephemeris.importers.invalid_input",
    source: object | None = None,
    cause: BaseException | None = None,
) -> NoReturn:
    error = InvalidInputError(
        code=code,
        message=message,
        component=_IMPORTER_COMPONENT,
        details=_failure_details(source=source, cause=cause),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


def _raise_importer_optional(
    message: str,
    *,
    code: str = "ephemeris.importers.optional_dependency",
    source: object | None = None,
    cause: BaseException | None = None,
) -> NoReturn:
    error = OptionalDependencyUnavailableError(
        code=code,
        message=message,
        component=_IMPORTER_COMPONENT,
        details=_failure_details(source=source, cause=cause),
        retryable=False,
        fallback_allowed=True,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


def _raise_importer_transient(
    message: str,
    *,
    code: str = "ephemeris.importers.io",
    source: object | None = None,
    cause: BaseException | None = None,
) -> NoReturn:
    error = TransientExternalFailureError(
        code=code,
        message=message,
        component=_IMPORTER_COMPONENT,
        details=_failure_details(source=source, cause=cause),
        retryable=True,
        fallback_allowed=True,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


def _raise_importer_integrity(
    message: str,
    *,
    code: str = "ephemeris.importers.integrity",
    source: object | None = None,
    cause: BaseException | None = None,
) -> NoReturn:
    error = IntegrityFailureError(
        code=code,
        message=message,
        component=_IMPORTER_COMPONENT,
        details=_failure_details(source=source, cause=cause),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


def _raise_importer_internal(
    message: str,
    *,
    code: str = "ephemeris.importers.internal",
    source: object | None = None,
    cause: BaseException | None = None,
) -> NoReturn:
    error = InternalInvariantError(
        code=code,
        message=message,
        component=_IMPORTER_COMPONENT,
        details=_failure_details(source=source, cause=cause),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


if TYPE_CHECKING:
    from srb.utils.publication import PublishedGeneration


def _require_valid_body(body: str, *, from_source: bool = False) -> None:
    """Reject unknown bodies before radius lookup or metadata propagation."""
    if not isinstance(body, str) or body not in _BODY_RADIUS_M:
        message = f"unknown body {body!r}; expected one of {sorted(_BODY_RADIUS_M)}"
        if from_source:
            _raise_importer_integrity(message, source=body)
        _raise_importer_invalid(message, code="ephemeris.importers.body", source=body)


def _require_post_2017(times_utc: "np.ndarray | list[str]") -> None:
    """Reject epochs before the fixed 69.184 s TT-UTC offset is valid."""

    def _text(value: object) -> str:
        if isinstance(value, (bytes, bytearray)):
            return value.decode("utf-8")
        return str(value)

    # Iterate source values directly to avoid an uncharged widened copy.
    try:
        earliest = min((_parse_iso(_text(value)) for value in times_utc), default=None)
    except (TypeError, ValueError, OverflowError) as exc:
        _raise_importer_integrity(
            "illumination source contains an invalid epoch timestamp",
            code="ephemeris.importers.epoch_integrity",
            cause=exc,
        )
    if earliest is None:
        return
    cutoff = _parse_iso(_TT_UTC_OFFSET_VALID_FROM)
    if earliest < cutoff:
        _raise_importer_integrity(
            f"illumination import carries epochs before "
            f"{_TT_UTC_OFFSET_VALID_FROM} (earliest: {earliest.isoformat()}); the "
            "fixed 69.184 s TT-UTC offset is invalid there — pre-2017 "
            "products need a leap-second ladder first",
            code="ephemeris.importers.epoch_integrity",
        )


def _convergence(lat_deg: float, lon_deg: float, crs: str) -> float:
    try:
        from srb.core.geodesy import north_convergence_deg
    except FailureError:
        raise
    except ImportError as exc:
        _raise_importer_optional(
            "computing ephemeris grid convergence requires the planetary CRS dependencies",
            code="ephemeris.importers.crs_dependency",
            source=crs,
            cause=exc,
        )

    try:
        return north_convergence_deg(lat_deg, lon_deg, crs)
    except FailureError:
        raise
    except (TypeError, ValueError) as exc:
        _raise_importer_integrity(
            f"source CRS {crs!r} cannot provide grid convergence: {exc}",
            code="ephemeris.importers.crs_integrity",
            source=crs,
            cause=exc,
        )


def _correct_azel(azel, *, gamma_deg, offset_xy_m, body_radius_m) -> np.ndarray:
    """Convert grid azimuth to true north and adjust elevation for parallax.

    Cells toward the sun azimuth have higher elevation under the source's curvature-ramp
    model.
    """
    azel = np.asarray(azel, dtype=np.float64)
    az_grid = azel[:, 0]
    el = azel[:, 1]
    az_true = np.mod(az_grid - gamma_deg, 360.0)
    az_rad = np.radians(az_grid)
    s_par = offset_xy_m[0] * np.sin(az_rad) + offset_xy_m[1] * np.cos(az_rad)
    el_site = el + np.degrees(s_par / body_radius_m)
    return np.stack([az_true, el_site], axis=1)


def _source_frame_correction_geometry(
    src_crs: str, src_transform, src_shape: tuple[int, int], dst_grid: ProductGrid
) -> tuple[float, float]:
    """Return target-minus-source offset in the source raster's grid frame.

    Parallax and convergence stay in the source frame when destination CRS differs.
    """
    import pyproj
    from rasterio.transform import Affine

    src_h, src_w = src_shape
    affine = Affine(*src_transform)
    src_x, src_y = affine * (src_w / 2.0, src_h / 2.0)
    dst_x, dst_y = dst_grid.patch_center_xy_m
    if str(src_crs) != str(dst_grid.crs):
        try:
            transformer = pyproj.Transformer.from_crs(
                dst_grid.crs, src_crs, always_xy=True
            )
            dst_x, dst_y = transformer.transform(dst_x, dst_y)
        except pyproj.exceptions.ProjError as exc:
            raise ValueError(
                f"cannot transform target grid into source CRS {src_crs!r}: {exc}"
            ) from exc
    return float(dst_x - src_x), float(dst_y - src_y)


def _source_reference_latlon(
    src_crs: str, src_transform, src_shape: tuple[int, int]
) -> tuple[float, float]:
    """Geographic latitude/longitude of the source raster reference centre."""
    import pyproj
    from rasterio.transform import Affine

    src_h, src_w = src_shape
    src_x, src_y = Affine(*src_transform) * (src_w / 2.0, src_h / 2.0)
    try:
        source_crs = pyproj.CRS.from_user_input(src_crs)
        geographic_crs = source_crs.geodetic_crs
        if geographic_crs is None:
            raise ValueError(f"source CRS {src_crs!r} has no geodetic CRS")
        lon, lat = pyproj.Transformer.from_crs(
            source_crs, geographic_crs, always_xy=True
        ).transform(src_x, src_y)
    except pyproj.exceptions.ProjError as exc:
        raise ValueError(
            f"cannot transform source reference centre from CRS {src_crs!r}: {exc}"
        ) from exc
    return float(lat), float(lon)


def _parse_iso(value: str) -> datetime:
    dt = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(timezone.utc)


def _reserve_npz_materialization(
    declarations: Mapping[str, Any],
) -> ByteBudget:
    """Reserve NPZ payloads and conversion copies before opening ``np.load``."""
    budget = _new_allocation_budget()
    conversion_dtypes = {
        "sunlit": np.dtype(bool),
        "earth_los": np.dtype(bool),
        "sun_fraction": np.dtype(np.float32),
        "sun_azel": np.dtype(np.float64),
        "earth_azel": np.dtype(np.float64),
    }
    for key, declaration in declarations.items():
        budget.consume(declaration.decoded_bytes)
        conversion_dtype = conversion_dtypes.get(key)
        if conversion_dtype is not None:
            converted_bytes = checked_shape_nbytes(
                declaration.shape,
                conversion_dtype,
                resource=f"lunarlab illumination NPZ {key} conversion",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            budget.consume(converted_bytes)

    # Legacy NPZ stores only a mask; reserve its derived float32 fraction too.
    if "sunlit" in declarations and "sun_fraction" not in declarations:
        derived_bytes = checked_shape_nbytes(
            declarations["sunlit"].shape,
            np.dtype(np.float32),
            resource="lunarlab illumination NPZ derived sun_fraction",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        budget.consume(derived_bytes)
    return budget


def _read_lunarlab_zarr(source_dir, *, epoch_range=None, epoch_stride: int = 1) -> dict:
    try:
        import zarr
    except FailureError:
        raise
    except ImportError as exc:
        _raise_importer_optional(
            "reading lunarlab spice_maps 'epochs.zarr' products requires the "
            "'zarr' package (and 'numcodecs'); install the srb[spice] extra "
            "or `pip install zarr numcodecs` to use import_lunarlab_zarr",
            source=source_dir,
            cause=exc,
        )

    try:
        require_directory_budget(
            source_dir,
            resource="lunarlab zarr product",
            max_bytes=MAX_ILLUMINATION_ARCHIVE_BYTES,
            max_file_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        metadata_budget = ByteBudget(
            "lunarlab zarr metadata allocations", MAX_METADATA_AGGREGATE_BYTES
        )
        for metadata_name in (".zattrs", ".zarray", "zarr.json"):
            for metadata_path in Path(source_dir).rglob(metadata_name):
                metadata_size = require_file_size(
                    metadata_path,
                    resource="lunarlab zarr metadata",
                    max_bytes=MAX_METADATA_ATTRIBUTE_BYTES,
                )
                metadata_budget.consume(metadata_size)
        grp = zarr.open_group(str(source_dir), mode="r")
        attrs = dict(grp.attrs)
        decoded_budget = _new_allocation_budget()

        def _metadata_shape(value: object) -> tuple[int, ...]:
            if isinstance(value, np.ndarray):
                return tuple(int(dimension) for dimension in value.shape)
            if isinstance(value, (list, tuple)):
                if not value:
                    return (0,)
                child_shape = _metadata_shape(value[0])
                if any(_metadata_shape(child) != child_shape for child in value[1:]):
                    raise ValueError("Zarr metadata array has a ragged shape")
                return (len(value), *child_shape)
            return ()

        def _reserve_metadata_array(
            key: str,
            value: object,
            dtype: np.dtype[Any],
        ) -> tuple[int, ...]:
            shape = _metadata_shape(value)
            amount = checked_shape_nbytes(
                shape,
                dtype,
                resource=f"lunarlab zarr metadata {key} conversion",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            metadata_budget.consume(amount)
            decoded_budget.consume(amount)
            return shape

        utcs_value = attrs["utcs"]
        utcs_shape = _metadata_shape(utcs_value)
        if utcs_shape:
            string_width = max(
                (len(str(value)) for value in utcs_value),
                default=1,
            )
        else:
            string_width = 1
        _reserve_metadata_array(
            "utcs",
            utcs_value,
            np.dtype(f"<U{max(1, string_width)}"),
        )
        for key in ("sun_azel", "earth_azel"):
            if key in attrs:
                _reserve_metadata_array(key, attrs[key], np.dtype(np.float64))
        if "sun_ang_radius_deg" in attrs:
            # Charge the float64 conversion and multiplication result.
            _reserve_metadata_array(
                "sun_ang_radius_deg source",
                attrs["sun_ang_radius_deg"],
                np.dtype(np.float64),
            )
            _reserve_metadata_array(
                "sun_ang_radius_deg conversion",
                attrs["sun_ang_radius_deg"],
                np.dtype(np.float64),
            )
        w = int(attrs["W"])
        if w <= 0:
            raise ValueError(f"invalid packed width W={w}")

        raw_utcs = np.asarray(utcs_value)
        selection_indices: np.ndarray | None = None
        selection_applied = False
        selection_identity = True
        if "sunlit" in grp:
            selection_indices = _epoch_selection_indices(
                raw_utcs,
                epoch_range,
                epoch_stride,
                budget=decoded_budget,
            )
            selection_applied = True
            selection_identity = selection_indices.size == len(raw_utcs)
            if selection_identity:
                identity_bytes = checked_shape_nbytes(
                    (len(raw_utcs),),
                    np.dtype(np.intp),
                    resource="lunarlab zarr epoch identity comparison",
                    max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                )
                decoded_budget.consume(identity_bytes)
                selection_identity = np.array_equal(
                    selection_indices, np.arange(len(raw_utcs))
                )
            if not selection_identity:
                selected_shape = (len(selection_indices), *utcs_shape[1:])
                selected_bytes = checked_shape_nbytes(
                    selected_shape,
                    np.dtype(raw_utcs.dtype),
                    resource="lunarlab zarr metadata utcs selection",
                    max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                )
                metadata_budget.consume(selected_bytes)
                decoded_budget.consume(selected_bytes)
                for key in ("sun_azel", "earth_azel"):
                    if key not in attrs:
                        continue
                    metadata_shape = _metadata_shape(attrs[key])
                    selected_bytes = checked_shape_nbytes(
                        (len(selection_indices), *metadata_shape[1:]),
                        np.dtype(np.float64),
                        resource=f"lunarlab zarr metadata {key} selection",
                        max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                    )
                    metadata_budget.consume(selected_bytes)
                    decoded_budget.consume(selected_bytes)
                if "sun_ang_radius_deg" in attrs:
                    selected_bytes = checked_shape_nbytes(
                        (len(selection_indices),),
                        np.dtype(np.float64),
                        resource="lunarlab zarr metadata sun_ang_diam selection",
                        max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                    )
                    metadata_budget.consume(selected_bytes)
                    decoded_budget.consume(selected_bytes)

        def _selected_shape(source_shape: tuple[int, ...]) -> tuple[int, ...]:
            if selection_indices is None or selection_identity:
                return source_shape
            if not source_shape or source_shape[0] != len(raw_utcs):
                raise ValueError(
                    f"epoch array shape {source_shape} does not match "
                    f"timestamp count {len(raw_utcs)}"
                )
            return (len(selection_indices), *source_shape[1:])

        def _read_selected(source_array: Any) -> np.ndarray:
            if selection_indices is None or selection_identity:
                return np.asarray(source_array[...])
            source_shape = tuple(int(value) for value in source_array.shape)
            if not source_shape or source_shape[0] != len(raw_utcs):
                raise ValueError(
                    f"epoch array shape {source_shape} does not match "
                    f"timestamp count {len(raw_utcs)}"
                )
            selection = (selection_indices, *([slice(None)] * (len(source_shape) - 1)))
            if hasattr(source_array, "oindex"):
                return np.asarray(source_array.oindex[selection])
            if hasattr(source_array, "get_orthogonal_selection"):
                return np.asarray(source_array.get_orthogonal_selection(selection))
            raise ValueError("Zarr reader has no bounded orthogonal selection API")

        def _reserve_chunk_workspace(
            source_array: Any,
            source_shape: tuple[int, ...],
            source_dtype: np.dtype[Any],
            key: str,
        ) -> None:
            # Orthogonal reads decode whole touched chunks or shards; budget that workspace.
            storage_shape = getattr(source_array, "shards", None)
            if storage_shape is None:
                storage_shape = getattr(source_array, "chunks", None)
            if storage_shape is None:
                raise ValueError(f"lunarlab zarr {key} has no bounded chunk shape")
            try:
                storage_shape = tuple(int(value) for value in storage_shape)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(
                    f"lunarlab zarr {key} has an invalid chunk shape {storage_shape!r}"
                ) from exc
            if len(storage_shape) != len(source_shape) or any(
                value <= 0 for value in storage_shape
            ):
                raise ValueError(
                    f"lunarlab zarr {key} has an invalid chunk shape {storage_shape!r}"
                )
            # Codec scratch may cover padded storage beyond the logical array edge.
            chunk_bytes = checked_shape_nbytes(
                storage_shape,
                source_dtype,
                resource=f"lunarlab zarr {key} decoded chunk workspace",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            storage_units = [
                (dimension + chunk - 1) // chunk
                for dimension, chunk in zip(source_shape, storage_shape)
            ]
            if selection_indices is not None and not selection_identity:
                first_axis_units = {
                    int(index) // storage_shape[0] for index in selection_indices
                }
                storage_units[0] = len(first_axis_units)
            touched_units = math.prod(storage_units)
            # Charge every touched chunk or shard; Zarr may retain several during gather.
            decoded_budget.consume(chunk_bytes * touched_units)

        unpack_plans: dict[str, tuple[Any, tuple[int, ...]]] = {}

        for key in ("sunlit", "earth_los"):
            if key not in grp:
                continue
            source_array = grp[key]
            full_source_shape = tuple(int(value) for value in source_array.shape)
            source_shape = _selected_shape(full_source_shape)
            source_dtype = np.dtype(source_array.dtype)
            _reserve_chunk_workspace(source_array, full_source_shape, source_dtype, key)
            packed_bytes = checked_shape_nbytes(
                source_shape,
                source_dtype,
                resource=f"lunarlab zarr {key} packed array",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            if not source_shape or source_shape[-1] * 8 < w:
                raise ValueError(f"invalid packed {key} shape {source_shape} for W={w}")
            unpacked_full_shape = (*source_shape[:-1], source_shape[-1] * 8)
            unpacked_full_bytes = checked_shape_nbytes(
                unpacked_full_shape,
                np.dtype(np.uint8),
                resource=f"lunarlab zarr {key} unpacked buffer",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            unpacked_shape = (*source_shape[:-1], w)
            unpacked_bytes = checked_shape_nbytes(
                unpacked_shape,
                np.dtype(bool),
                resource=f"lunarlab zarr {key} unpacked array",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            for amount in (packed_bytes, unpacked_full_bytes, unpacked_bytes):
                decoded_budget.consume(amount)
            unpack_plans[key] = (source_array, unpacked_shape)

        fraction_plan: tuple[Any, tuple[int, ...]] | None = None
        if "sun_fraction" in grp:
            source_array = grp["sun_fraction"]
            full_source_shape = tuple(int(value) for value in source_array.shape)
            source_shape = _selected_shape(full_source_shape)
            source_dtype = np.dtype(source_array.dtype)
            _reserve_chunk_workspace(
                source_array, full_source_shape, source_dtype, "sun_fraction"
            )
            source_bytes = checked_shape_nbytes(
                source_shape,
                source_dtype,
                resource="lunarlab zarr sun_fraction source array",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            decoded_bytes = checked_shape_nbytes(
                source_shape,
                np.dtype(np.float32),
                resource="lunarlab zarr sun_fraction decoded array",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            # Charge simultaneously live read, normalization, mask, and conversion arrays.
            decoded_budget.consume(source_bytes)
            decoded_budget.consume(decoded_bytes)
            decoded_budget.consume(decoded_bytes)
            decoded_budget.consume(decoded_bytes)
            decoded_budget.consume(decoded_bytes)
            decoded_budget.consume(decoded_bytes)
            fraction_plan = (source_array, source_shape)

        def _unpack(plan: tuple[Any, tuple[int, ...]] | None) -> np.ndarray | None:
            if plan is None:
                return None
            source_array, _unpacked_shape = plan
            packed = _read_selected(source_array)
            return np.unpackbits(packed, axis=-1)[..., :w].astype(bool)

        sunlit = _unpack(unpack_plans.get("sunlit"))
        earth_los = _unpack(unpack_plans.get("earth_los"))

        sun_fraction = None
        if fraction_plan is not None:
            source_array, _source_shape = fraction_plan
            q = np.asarray(_read_selected(source_array), dtype=np.float32)
            sun_fraction = np.where(q == 255, np.nan, q / 254.0).astype(np.float32)

        sun_ang_radius = attrs.get("sun_ang_radius_deg")
        sun_ang_diam = (
            None
            if sun_ang_radius is None
            else 2.0 * np.asarray(sun_ang_radius, dtype=np.float64)
        )

        # Validate epochs after selection in `_import_common`, not on raw input.
        utcs = raw_utcs
        if selection_indices is not None and not selection_identity:
            utcs = utcs[selection_indices]
        sun_azel = np.asarray(attrs["sun_azel"], dtype=np.float64)
        earth_azel = (
            np.asarray(attrs["earth_azel"], dtype=np.float64)
            if "earth_azel" in attrs
            else None
        )
        if selection_indices is not None and not selection_identity:
            sun_azel = sun_azel[selection_indices]
            if earth_azel is not None:
                earth_azel = earth_azel[selection_indices]
        if (
            sun_ang_diam is not None
            and selection_indices is not None
            and not selection_identity
        ):
            sun_ang_diam = sun_ang_diam[selection_indices]

        return {
            "sunlit": sunlit,
            "earth_los": earth_los,
            "sun_fraction": sun_fraction,
            "utcs": utcs,
            "sun_azel": sun_azel,
            "earth_azel": earth_azel,
            "sun_ang_diam": sun_ang_diam,
            "crs": str(attrs["crs"]),
            "transform": tuple(float(v) for v in attrs["transform"]),
            "shape": tuple(int(v) for v in attrs["shape"]),
            "window_key": str(attrs.get("window_key", Path(source_dir).name)),
            "warnings": [],
            "_epoch_selection_applied": selection_applied,
            _ALLOCATION_BUDGET_KEY: decoded_budget,
        }
    except FailureError:
        raise
    except OSError as exc:
        _raise_importer_transient(
            f"could not read lunarlab zarr product {source_dir}: {exc}",
            source=source_dir,
            cause=exc,
        )
    except (
        ValueError,
        KeyError,
        TypeError,
        IndexError,
        OverflowError,
        zipfile.BadZipFile,
    ) as exc:
        _raise_importer_integrity(
            f"lunarlab zarr product is corrupt or incomplete: {source_dir} ({exc})",
            source=source_dir,
            cause=exc,
        )


def _read_lunarlab_npz(source, *, body: str = "moon") -> dict:
    try:
        declarations = inspect_npz_archive(
            source,
            resource="lunarlab illumination NPZ",
            max_archive_bytes=MAX_ILLUMINATION_ARCHIVE_BYTES,
            max_decoded_bytes=MAX_ILLUMINATION_DECODED_BYTES,
            max_array_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        if "meta" in declarations and declarations["meta"].decoded_bytes > (
            MAX_METADATA_ATTRIBUTE_BYTES
        ):
            raise ValueError("lunarlab NPZ metadata exceeds its resource limit")
        budget = _reserve_npz_materialization(declarations)
        with np.load(source, allow_pickle=False) as data:
            sunlit = np.asarray(data["sunlit"]) if "sunlit" in data else None
            earth_los = np.asarray(data["earth_los"]) if "earth_los" in data else None
            for name, mask in (("sunlit", sunlit), ("earth_los", earth_los)):
                if mask is not None and (
                    mask.dtype.kind not in "biuf" or not np.isin(mask, (0, 1)).all()
                ):
                    raise ValueError(f"{name} mask values must be boolean or 0/1")
            sunlit = None if sunlit is None else sunlit.astype(bool)
            earth_los = None if earth_los is None else earth_los.astype(bool)
            sun_fraction = (
                np.asarray(data["sun_fraction"], dtype=np.float32)
                if "sun_fraction" in data
                else None
            )
            sun_azel = (
                np.asarray(data["sun_azel"], dtype=np.float64)
                if "sun_azel" in data
                else None
            )
            earth_azel = (
                np.asarray(data["earth_azel"], dtype=np.float64)
                if "earth_azel" in data
                else None
            )
            utcs = np.asarray(data["times_utc"]) if "times_utc" in data else None
            meta_raw = json.loads(str(data["meta"])) if "meta" in data else {}
    except FailureError:
        raise
    except OSError as exc:
        _raise_importer_transient(
            f"could not read lunarlab npz product {source}: {exc}",
            source=source,
            cause=exc,
        )
    except (
        ValueError,
        KeyError,
        TypeError,
        IndexError,
        OverflowError,
        UnicodeError,
        zipfile.BadZipFile,
    ) as exc:
        _raise_importer_integrity(
            f"lunarlab npz product is corrupt or incomplete: {source} ({exc})",
            source=source,
            cause=exc,
        )

    # `_import_common` validates the selected epoch window.

    if not isinstance(meta_raw, dict):
        _raise_importer_integrity(
            f"lunarlab npz metadata must be an object: {source}",
            source=source,
        )

    try:
        warnings: list[str] = []

        if sun_fraction is None and sunlit is not None:
            # Legacy `save_masks` has a boolean mask only; derive its 0/1 fraction.
            sun_fraction = sunlit.astype(np.float32)

        sun_ang_diam = None
        if "sun_ang_radius_deg" in meta_raw:
            radius_shape = _nested_value_shape(
                meta_raw["sun_ang_radius_deg"],
                resource="lunarlab illumination sun angular radius",
            )
            radius_bytes = checked_shape_nbytes(
                radius_shape,
                np.dtype(np.float64),
                resource="lunarlab illumination sun angular radius conversion",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            # Reserve the float64 conversion and multiplication result before allocation.
            budget.consume(radius_bytes)
            budget.consume(radius_bytes)
            sun_ang_diam = 2.0 * np.asarray(
                meta_raw["sun_ang_radius_deg"], dtype=np.float64
            )
        elif sun_azel is not None:
            angular_bytes = checked_shape_nbytes(
                (len(sun_azel),),
                np.dtype(np.float64),
                resource="lunarlab illumination default sun angular diameter",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            budget.consume(angular_bytes)
            default_diameter = (
                _DEFAULT_SUN_ANGULAR_DIAMETER_DEG / _MEAN_HELIOCENTRIC_DISTANCE_AU[body]
            )
            sun_ang_diam = np.full(len(sun_azel), default_diameter, dtype=np.float64)
            warnings.append(
                "legacy lunarlab npz carries no sun_ang_radius_deg; defaulted "
                f"sun_angular_diameter_deg to {default_diameter:.4f} deg (mean "
                f"{body} distance) for every epoch"
            )

        return {
            "sunlit": sunlit,
            "earth_los": earth_los,
            "sun_fraction": sun_fraction,
            "utcs": utcs,
            "sun_azel": sun_azel,
            "earth_azel": earth_azel,
            "sun_ang_diam": sun_ang_diam,
            "crs": str(meta_raw["crs"]),
            "transform": tuple(float(v) for v in meta_raw["transform"]),
            "shape": tuple(int(v) for v in meta_raw["shape"]),
            "window_key": None,
            "warnings": warnings,
            _ALLOCATION_BUDGET_KEY: budget,
        }
    except FailureError:
        raise
    except (ValueError, KeyError, TypeError, IndexError, OverflowError) as exc:
        _raise_importer_integrity(
            f"lunarlab npz metadata is corrupt or incomplete: {source} ({exc})",
            source=source,
            cause=exc,
        )


def _read_pgda_lpsr(source_tif) -> dict:
    try:
        import rasterio
    except FailureError:
        raise
    except ImportError as exc:
        _raise_importer_optional(
            "reading PGDA LPSR products requires the 'rasterio' package",
            source=source_tif,
            cause=exc,
        )

    try:
        require_file_size(
            source_tif,
            resource="PGDA LPSR source",
            max_bytes=MAX_ILLUMINATION_ARCHIVE_BYTES,
        )
        with rasterio.open(source_tif) as ds:
            source_dtype = np.dtype(ds.dtypes[0])
            if source_dtype not in (np.dtype(np.int16), np.dtype(np.uint8)):
                raise ValueError(
                    f"{source_tif}: unsupported PGDA LPSR dtype {source_dtype!r}; "
                    "expected int16 raw or uint8 canonical"
                )
            checked_shape_nbytes(
                (int(ds.height), int(ds.width)),
                source_dtype,
                resource="PGDA LPSR decoded raster",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            pgda_budget = _new_allocation_budget()
            source_bytes = checked_shape_nbytes(
                (int(ds.height), int(ds.width)),
                source_dtype,
                resource="PGDA LPSR source allocation",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            pgda_budget.consume(source_bytes)
            if source_dtype == np.dtype(np.int16):
                # Charge the separately live mask, `where` result, and uint8 conversion.
                for dtype in (np.dtype(bool), np.dtype(np.int64), np.dtype(np.uint8)):
                    pgda_budget.consume(
                        checked_shape_nbytes(
                            (int(ds.height), int(ds.width)),
                            dtype,
                            resource="PGDA LPSR conversion allocation",
                            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                        )
                    )
            else:
                for dtype in (np.dtype(bool), np.dtype(np.uint8)):
                    pgda_budget.consume(
                        checked_shape_nbytes(
                            (int(ds.height), int(ds.width)),
                            dtype,
                            resource="PGDA LPSR conversion allocation",
                            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
                        )
                    )
            arr = ds.read(1)
            transform = tuple(ds.transform)[:6]
            crs = ds.crs.to_string() if ds.crs is not None else None
            shape = (int(ds.height), int(ds.width))
    except FailureError:
        raise
    except OSError as exc:
        _raise_importer_transient(
            f"could not read PGDA LPSR product {source_tif}: {exc}",
            source=source_tif,
            cause=exc,
        )
    except (ValueError, KeyError, TypeError, IndexError, OverflowError) as exc:
        _raise_importer_integrity(
            f"PGDA LPSR product is corrupt or incomplete: {source_tif} ({exc})",
            source=source_tif,
            cause=exc,
        )

    try:
        if arr.dtype == np.int16:
            psr = np.where(arr == 20000, 1, 0).astype(np.uint8)
            psr[arr == -32768] = 255
        elif arr.dtype == np.uint8:
            if not np.isin(arr, (0, 1, 255)).all():
                raise ValueError(
                    f"{source_tif}: uint8 canonical PGDA LPSR must only contain "
                    "{0, 1, 255}"
                )
            psr = arr.copy()
        else:
            raise ValueError(
                f"{source_tif}: unsupported PGDA LPSR dtype {arr.dtype!r}; expected "
                "int16 raw (DN==20000 -> psr, DN==-32768 -> nodata) or uint8 "
                "canonical {0, 1, 255}"
            )
    except FailureError:
        raise
    except (ValueError, TypeError, IndexError, OverflowError) as exc:
        _raise_importer_integrity(
            f"PGDA LPSR product has invalid pixel data: {source_tif} ({exc})",
            source=source_tif,
            cause=exc,
        )

    return {
        "sunlit": None,
        "earth_los": None,
        "sun_fraction": None,
        "utcs": None,
        "sun_azel": None,
        "earth_azel": None,
        "sun_ang_diam": None,
        "psr": psr,
        "crs": str(crs),
        "transform": transform,
        "shape": shape,
        "window_key": Path(source_tif).stem,
        "warnings": [],
        _ALLOCATION_BUDGET_KEY: pgda_budget,
    }


def _epoch_selection_indices(
    utcs: object,
    epoch_range,
    epoch_stride: int,
    *,
    budget: ByteBudget | None = None,
) -> np.ndarray:
    utcs = np.asarray(utcs)
    if isinstance(epoch_stride, bool):
        _raise_importer_invalid(
            f"epoch_stride must be a positive integer, got {epoch_stride!r}",
            code="ephemeris.importers.epoch_stride",
        )
    try:
        stride = int(epoch_stride)
        numeric_stride = float(epoch_stride)
    except (TypeError, ValueError, OverflowError) as exc:
        _raise_importer_invalid(
            f"epoch_stride must be a positive integer, got {epoch_stride!r}",
            code="ephemeris.importers.epoch_stride",
            cause=exc,
        )
    if not math.isfinite(numeric_stride) or numeric_stride != stride or stride <= 0:
        _raise_importer_invalid(
            f"epoch_stride must be a positive integer, got {epoch_stride!r}",
            code="ephemeris.importers.epoch_stride",
        )
    if budget is not None:
        index_bytes = checked_shape_nbytes(
            (len(utcs),),
            np.dtype(np.intp),
            resource="illumination epoch selection index allocation",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        budget.consume(index_bytes)
    idx = np.arange(len(utcs))
    if epoch_range is not None:
        try:
            start_dt = _parse_iso(epoch_range[0])
            end_dt = _parse_iso(epoch_range[1])
        except (TypeError, ValueError, IndexError, OverflowError) as exc:
            _raise_importer_invalid(
                f"epoch_range must contain two ISO-8601 timestamps, got {epoch_range!r}",
                code="ephemeris.importers.epoch_range",
                cause=exc,
            )
        if budget is not None:
            timestamp_bytes = checked_shape_nbytes(
                (len(utcs),),
                np.dtype(np.object_),
                resource="illumination epoch selection timestamp allocation",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            mask_bytes = checked_shape_nbytes(
                (len(utcs),),
                np.dtype(bool),
                resource="illumination epoch selection mask allocation",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            filtered_index_bytes = checked_shape_nbytes(
                (len(utcs),),
                np.dtype(np.intp),
                resource="illumination epoch selection filtered index allocation",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            budget.consume(timestamp_bytes)
            budget.consume(mask_bytes)
            budget.consume(filtered_index_bytes)
        try:
            times = np.array([_parse_iso(str(u)) for u in utcs])
        except (TypeError, ValueError, OverflowError) as exc:
            _raise_importer_integrity(
                "illumination source contains an invalid epoch timestamp",
                code="ephemeris.importers.epoch_integrity",
                cause=exc,
            )
        mask = np.array([start_dt <= t <= end_dt for t in times])
        idx = idx[mask]
    idx = idx[::stride]

    if idx.size == 0:
        requested = (
            f"({epoch_range[0]}, {epoch_range[1]})"
            if epoch_range is not None
            else "<all>"
        )
        product_span = (
            f"({utcs[0]!s}, {utcs[-1]!s})" if len(utcs) else "<empty product>"
        )
        _raise_importer_invalid(
            f"epoch_range {requested} (stride={epoch_stride}) selects zero "
            f"epochs from product span {product_span}",
            code="ephemeris.importers.epoch_range",
        )
    return idx


def _apply_epoch_selection(arrays: dict, epoch_range, epoch_stride: int) -> dict:
    utcs = np.asarray(arrays["utcs"])
    selection_budget = _allocation_budget(arrays)
    if selection_budget is None:
        selection_budget = _new_allocation_budget()
        _reserve_retained_arrays(
            arrays,
            selection_budget,
            resource_prefix="illumination epoch source allocation",
        )
    idx = _epoch_selection_indices(
        utcs,
        epoch_range,
        epoch_stride,
        budget=selection_budget,
    )
    identity = idx.size == len(utcs)
    if identity:
        identity_bytes = checked_shape_nbytes(
            (len(utcs),),
            np.dtype(np.intp),
            resource="illumination epoch identity comparison",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        selection_budget.consume(identity_bytes)
        identity = np.array_equal(idx, np.arange(len(utcs)))
    if identity:
        return arrays

    selected_times_bytes = checked_shape_nbytes(
        (len(idx),),
        np.dtype(utcs.dtype),
        resource="illumination epoch selection times_utc",
        max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
    )
    selection_budget.consume(selected_times_bytes)
    for key in (*_EPOCH_ARRAY_KEYS, *_AZEL_KEYS):
        array = arrays.get(key)
        if array is None:
            continue
        selected_shape = (len(idx),) + tuple(array.shape[1:])
        selected_bytes = checked_shape_nbytes(
            selected_shape,
            np.dtype(array.dtype),
            resource=f"illumination epoch selection {key!r}",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        selection_budget.consume(selected_bytes)
    if arrays.get("sun_ang_diam") is not None:
        selected_bytes = checked_shape_nbytes(
            (len(idx),),
            np.dtype(np.asarray(arrays["sun_ang_diam"]).dtype),
            resource="illumination epoch selection sun_ang_diam",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        selection_budget.consume(selected_bytes)

    out = dict(arrays)
    out["utcs"] = utcs[idx]
    for key in (*_EPOCH_ARRAY_KEYS, *_AZEL_KEYS):
        if arrays.get(key) is not None:
            out[key] = arrays[key][idx]
    if arrays.get("sun_ang_diam") is not None:
        out["sun_ang_diam"] = np.asarray(arrays["sun_ang_diam"])[idx]
    return out


def _reproject_stack(
    arrays: dict, src_crs: str, src_transform, dst_grid: ProductGrid
) -> dict:
    import rasterio.warp
    from rasterio.enums import Resampling
    from rasterio.transform import Affine

    try:
        src_h, src_w = (int(value) for value in arrays["shape"])
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            f"source raster shape is invalid: {arrays.get('shape')!r}"
        ) from exc
    checked_shape_nbytes(
        (src_h, src_w),
        np.dtype(np.uint8),
        resource="illumination source raster shape",
        max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
    )
    src_aff = Affine(*src_transform)
    dst_h, dst_w = dst_grid.shape
    dst_aff = Affine(*dst_grid.transform)

    cx, cy = dst_grid.patch_center_xy_m
    half_w = dst_aff.a * dst_w / 2.0
    half_h = -dst_aff.e * dst_h / 2.0
    dst_bounds = (cx - half_w, cy - half_h, cx + half_w, cy + half_h)
    xs4 = [dst_bounds[0], dst_bounds[0], dst_bounds[2], dst_bounds[2]]
    ys4 = [dst_bounds[1], dst_bounds[3], dst_bounds[1], dst_bounds[3]]

    if str(src_crs) != str(dst_grid.crs):
        import pyproj

        transformer = pyproj.Transformer.from_crs(dst_grid.crs, src_crs, always_xy=True)
        xs, ys = transformer.transform(xs4, ys4)
    else:
        xs, ys = xs4, ys4

    inv = ~src_aff
    cols, rows = [], []
    for x, y in zip(xs, ys):
        col, row = inv * (x, y)
        cols.append(col)
        rows.append(row)

    margin = 2
    row_start = max(0, int(math.floor(min(rows))) - margin)
    row_stop = min(src_h, int(math.ceil(max(rows))) + margin)
    col_start = max(0, int(math.floor(min(cols))) - margin)
    col_stop = min(src_w, int(math.ceil(max(cols))) + margin)
    row_stop = max(row_stop, row_start + 1)
    col_stop = max(col_stop, col_start + 1)

    windowed_transform = src_aff * Affine.translation(col_start, row_start)

    def _crop(arr: np.ndarray | None) -> np.ndarray | None:
        return None if arr is None else arr[..., row_start:row_stop, col_start:col_stop]

    def _one(slab: np.ndarray, kind: str) -> np.ndarray:
        if 0 in slab.shape[-2:]:
            fill = False if kind == "bool" else 255 if kind == "uint8" else np.nan
            return np.full((dst_h, dst_w), fill, dtype=output_dtypes[kind])
        if kind == "bool":
            src = slab.astype(np.uint8)
            dst = np.zeros((dst_h, dst_w), dtype=np.uint8)
            rasterio.warp.reproject(
                source=src,
                destination=dst,
                src_transform=windowed_transform,
                src_crs=src_crs,
                dst_transform=dst_aff,
                dst_crs=dst_grid.crs,
                resampling=Resampling.nearest,
            )
            return dst.astype(bool)
        if kind == "uint8":
            src = slab.astype(np.uint8)
            dst = np.full((dst_h, dst_w), 255, dtype=np.uint8)
            rasterio.warp.reproject(
                source=src,
                destination=dst,
                src_transform=windowed_transform,
                src_crs=src_crs,
                dst_transform=dst_aff,
                dst_crs=dst_grid.crs,
                src_nodata=255,
                dst_nodata=255,
                init_dest_nodata=True,
                resampling=Resampling.nearest,
            )
            return dst
        src = slab.astype(np.float64)
        dst = np.full((dst_h, dst_w), np.nan, dtype=np.float64)
        rasterio.warp.reproject(
            source=src,
            destination=dst,
            src_transform=windowed_transform,
            src_crs=src_crs,
            dst_transform=dst_aff,
            dst_crs=dst_grid.crs,
            src_nodata=np.nan,
            dst_nodata=np.nan,
            init_dest_nodata=True,
            resampling=Resampling.bilinear,
        )
        return dst.astype(np.float32)

    budget = _allocation_budget(arrays)
    if budget is None:
        budget = _new_allocation_budget()
        _reserve_retained_arrays(
            arrays,
            budget,
            resource_prefix="illumination reprojection source allocation",
        )
    plans: dict[str, tuple[np.ndarray, str, tuple[int, ...]]] = {}
    crop_shape = (row_stop - row_start, col_stop - col_start)
    temp_dtypes = {
        "bool": (np.dtype(np.uint8), np.dtype(np.uint8), np.dtype(bool)),
        "uint8": (np.dtype(np.uint8), np.dtype(np.uint8), None),
        "float": (np.dtype(np.float64), np.dtype(np.float64), np.dtype(np.float32)),
    }
    output_dtypes = {
        "bool": np.dtype(bool),
        "uint8": np.dtype(np.uint8),
        "float": np.dtype(np.float32),
    }
    # Keep slab reservations cumulative; native backends may retain prior slabs.
    for key, kind in (
        ("sunlit", "bool"),
        ("earth_los", "bool"),
        ("sun_fraction", "float"),
        ("psr", "uint8"),
        ("lit_fraction", "float"),
        ("max_sun_el", "float"),
    ):
        arr = arrays.get(key)
        if arr is None:
            continue
        if arr.ndim not in (2, 3) or tuple(arr.shape[-2:]) != (src_h, src_w):
            raise ValueError(
                f"illumination source {key!r} shape {getattr(arr, 'shape', None)!r} "
                f"does not match source grid {(src_h, src_w)!r}"
            )
        slab_count = 1 if arr.ndim == 2 else int(arr.shape[0])
        output_shape = ((slab_count,) if arr.ndim == 3 else ()) + (dst_h, dst_w)
        output_bytes = checked_shape_nbytes(
            output_shape,
            output_dtypes[kind],
            resource=f"illumination reprojection output {key!r}",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        conversion_dtype, destination_dtype, final_dtype = temp_dtypes[kind]
        per_slab = checked_shape_nbytes(
            crop_shape,
            conversion_dtype,
            resource=f"illumination reprojection {key!r} conversion",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        per_slab += checked_shape_nbytes(
            (dst_h, dst_w),
            destination_dtype,
            resource=f"illumination reprojection {key!r} destination",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        if final_dtype is not None:
            per_slab += checked_shape_nbytes(
                (dst_h, dst_w),
                final_dtype,
                resource=f"illumination reprojection {key!r} conversion result",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
        budget.consume(output_bytes)
        budget.consume(per_slab * slab_count)
        plans[key] = (arr, kind, output_shape)

    def _stack(key: str, kind: str) -> np.ndarray | None:
        plan = plans.get(key)
        if plan is None:
            return None
        arr, _kind, output_shape = plan
        cropped = _crop(arr)
        assert cropped is not None
        result = np.empty(output_shape, dtype=output_dtypes[kind])
        if cropped.ndim == 2:
            result[...] = _one(cropped, kind)
        else:
            for index in range(cropped.shape[0]):
                result[index] = _one(cropped[index], kind)
        return result

    return {
        "sunlit": _stack("sunlit", "bool"),
        "earth_los": _stack("earth_los", "bool"),
        "sun_fraction": _stack("sun_fraction", "float"),
        "psr": _stack("psr", "uint8"),
        "lit_fraction": _stack("lit_fraction", "float"),
        "max_sun_el": _stack("max_sun_el", "float"),
        _ALLOCATION_BUDGET_KEY: budget,
    }


@dataclass(frozen=True)
class _BakedGeoref:
    """Validated baked-patch georeference, before target GSD is applied."""

    meta_path: Path
    crs: str
    center_xy_m: tuple[float, float]
    center_latlon: tuple[float, float]
    body: str
    patch_size_m: float


def _read_baked_georef(baked_dir) -> _BakedGeoref:
    """Read and validate baked-patch metadata without opening raster data."""
    try:
        baked_dir = Path(baked_dir)
    except (TypeError, ValueError) as exc:
        _raise_importer_invalid(
            f"baked_dir is invalid: {baked_dir!r}",
            code="ephemeris.importers.baked_dir",
            cause=exc,
        )
    generation = _metadata_generation(baked_dir)
    if generation is not None:
        baked_dir = generation.path
    from srb.utils.publication import (
        ArtifactPublication,
        PublicationError,
        PublicationTimeoutError,
    )

    meta_path = baked_dir / "meta.json"
    try:
        if generation is None:
            meta = ArtifactPublication.file(meta_path).read(json.loads)
            if meta is None:
                _raise_importer_invalid(
                    f"baked terrain metadata does not exist: {meta_path}",
                    code="ephemeris.importers.baked_metadata",
                    source=meta_path,
                )
        else:
            meta = generation.read_member("meta.json", json.loads)
    except FailureError:
        raise
    except PublicationTimeoutError as exc:
        _raise_importer_transient(
            f"could not read baked terrain metadata {meta_path}: {exc}",
            source=meta_path,
            cause=exc,
        )
    except OSError as exc:
        _raise_importer_transient(
            f"could not read baked terrain metadata {meta_path}: {exc}",
            source=meta_path,
            cause=exc,
        )
    except (PublicationError, ValueError, TypeError, KeyError) as exc:
        _raise_importer_integrity(
            f"baked terrain metadata is corrupt: {meta_path} ({exc})",
            code="ephemeris.importers.baked_metadata_integrity",
            source=meta_path,
            cause=exc,
        )

    if not isinstance(meta, dict):
        _raise_importer_integrity(
            f"baked terrain metadata must be an object: {meta_path}",
            code="ephemeris.importers.baked_metadata_integrity",
            source=meta_path,
        )
    try:
        georef = meta.get("georef")
        if not isinstance(georef, dict):
            raise ValueError("meta.json has no 'georef' block")

        crs = str(georef["crs"])
        cx, cy = (float(v) for v in georef["patch_center_xy_m"])
        lat, lon = (float(v) for v in georef["patch_center_latlon"])
        body = str(meta.get("body", "moon"))
        size_m = float(meta["patch_size_m"])
        if not math.isfinite(size_m) or size_m <= 0.0:
            raise ValueError(
                f"patch_size_m must be finite and positive, got {size_m!r}"
            )
    except FailureError:
        raise
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        _raise_importer_integrity(
            f"baked terrain georeference is corrupt: {meta_path} ({exc})",
            code="ephemeris.importers.baked_georef_integrity",
            source=meta_path,
            cause=exc,
        )
    return _BakedGeoref(
        meta_path=meta_path,
        crs=crs,
        center_xy_m=(cx, cy),
        center_latlon=(lat, lon),
        body=body,
        patch_size_m=size_m,
    )


def _baked_target_grid(georef: _BakedGeoref, gsd: float) -> ProductGrid:
    """Build the destination grid; `ProductGrid` validates its dimensions."""
    try:
        ratio = georef.patch_size_m / gsd
        if not math.isfinite(ratio) or ratio <= 0.0:
            raise ValueError(
                f"patch_size_m/gsd_m must be finite and positive, got {ratio!r}"
            )
        n = int(round(ratio))
        if n <= 0:
            raise ValueError(
                f"patch_size_m/gsd_m rounds to an empty target grid: {ratio!r}"
            )
        cx, cy = georef.center_xy_m
        a = gsd
        e = -gsd
        c = cx - a * n / 2.0
        f = cy - e * n / 2.0
        return ProductGrid(
            crs=georef.crs,
            transform=(a, 0.0, c, 0.0, e, f),
            shape=(n, n),
            patch_center_xy_m=(cx, cy),
        )
    except FailureError:
        raise
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        _raise_importer_integrity(
            f"baked terrain georeference is corrupt: {georef.meta_path} ({exc})",
            code="ephemeris.importers.baked_georef_integrity",
            source=georef.meta_path,
            cause=exc,
        )


def _metadata_generation(baked_dir: Path) -> "PublishedGeneration | None":
    """Resolve a cache root or generation to its published snapshot."""
    from srb.utils.publication import (
        ArtifactPublication,
        PublicationError,
        PublicationTimeoutError,
        PublishedGeneration,
    )

    expected_path: Path | None = None
    if baked_dir.parent.name == "generations":
        root = baked_dir.parent.parent
        expected_path = baked_dir.absolute()
    elif (baked_dir / "current.json").is_file():
        root = baked_dir
    else:
        return None
    try:
        opened = ArtifactPublication.generation(root).open(
            required_members=("meta.json",)
        )
    except FailureError:
        raise
    except PublicationTimeoutError as exc:
        _raise_importer_transient(
            f"could not read published terrain generation {baked_dir}: {exc}",
            source=baked_dir,
            cause=exc,
        )
    except OSError as exc:
        _raise_importer_transient(
            f"could not read published terrain generation {baked_dir}: {exc}",
            source=baked_dir,
            cause=exc,
        )
    except (PublicationError, ValueError, TypeError, KeyError) as exc:
        _raise_importer_integrity(
            f"published terrain generation is corrupt: {baked_dir} ({exc})",
            code="ephemeris.importers.baked_generation_integrity",
            source=baked_dir,
            cause=exc,
        )
    if not isinstance(opened, PublishedGeneration):
        _raise_importer_integrity(
            f"{baked_dir}: publication did not return a generation",
            code="ephemeris.importers.baked_generation_integrity",
            source=baked_dir,
        )
    if expected_path is not None and opened.path != expected_path:
        _raise_importer_integrity(
            f"{baked_dir}: terrain generation is no longer current",
            code="ephemeris.importers.baked_generation_integrity",
            source=baked_dir,
        )
    return opened


def _cache_key_dir(baked_dir: Path) -> Path:
    """Return the cache-key directory for a key or generation path."""
    if baked_dir.parent.name == "generations":
        return baked_dir.parent.parent
    return baked_dir


def _published_product_sha256(path: Path) -> str:
    """Return the digest of a publication-verified product snapshot."""
    from srb.utils.publication import (
        ArtifactPublication,
        PublicationError,
        PublicationTimeoutError,
    )

    try:
        digest = ArtifactPublication.file(path).read(
            lambda payload: hashlib.sha256(payload).hexdigest()
        )
    except FailureError:
        raise
    except PublicationTimeoutError as exc:
        _raise_importer_transient(
            f"could not read published illumination product {path}: {exc}",
            source=path,
            cause=exc,
        )
    except OSError as exc:
        _raise_importer_transient(
            f"could not read published illumination product {path}: {exc}",
            source=path,
            cause=exc,
        )
    except PublicationError as exc:
        _raise_importer_integrity(
            f"published illumination product failed integrity validation: {path} ({exc})",
            code="ephemeris.importers.publication_integrity",
            source=path,
            cause=exc,
        )
    if digest is None:
        _raise_importer_transient(
            f"published illumination product is missing: {path}",
            source=path,
        )
    return digest


def _record_in_terrain_meta(
    baked_dir,
    out_path,
    product: IlluminationProduct,
    source_label: str,
    *,
    static_keys_present: list[str],
    product_sha256: str | None = None,
    metadata_writer: TerrainMetadataWriter | None = None,
) -> None:
    baked_dir = Path(baked_dir)
    writer = (
        resolve_terrain_metadata_writer(baked_dir)
        if metadata_writer is None
        else metadata_writer
    )
    if product_sha256 is None:
        product_sha256 = _published_product_sha256(Path(out_path))
    block = {
        "file": Path(out_path).name,
        "schema": SCHEMA_ID,
        "sha256": product_sha256,
        "content_sha256": product_sha256,
        "publication": {
            "kind": "immutable-file",
            "sha256": product_sha256,
        },
        "source": source_label,
        "epochs": (
            {
                "count": product.epoch_count,
                "t0": str(product.times_utc[0]),
                "step_s": product.epoch_step_s,
            }
            if product.has_epochs
            else None
        ),
        "static": list(static_keys_present),
        "imported_at_iso": datetime.now(timezone.utc)
        .isoformat()
        .replace("+00:00", "Z"),
    }

    def update(meta: dict[str, Any]) -> None:
        meta["illumination"] = block

    try:
        writer(update)
    except FailureError:
        raise
    except OSError as exc:
        _raise_importer_transient(
            f"could not update baked terrain metadata for {baked_dir}: {exc}",
            source=baked_dir,
            cause=exc,
        )
    except (TypeError, ValueError, KeyError) as exc:
        _raise_importer_integrity(
            f"baked terrain metadata update is invalid for {baked_dir}: {exc}",
            code="ephemeris.importers.baked_metadata_integrity",
            source=baked_dir,
            cause=exc,
        )


def _requested_baked_gsd(baked_dir, gsd_m) -> tuple[Path, Optional[float]]:
    """Parse baked directory and optional finite positive target GSD."""
    try:
        baked_path = Path(baked_dir)
        requested = float(gsd_m) if gsd_m is not None else None
    except (TypeError, ValueError, OverflowError) as exc:
        _raise_importer_invalid(
            f"baked_dir/gsd_m input is invalid: baked_dir={baked_dir!r}, gsd_m={gsd_m!r}",
            code="ephemeris.importers.target",
            cause=exc,
        )
    if requested is not None and (not math.isfinite(requested) or requested <= 0.0):
        _raise_importer_invalid(
            f"gsd_m must be finite and positive, got {gsd_m!r}",
            code="ephemeris.importers.gsd",
        )
    return baked_path, requested


def _admit_target_before_decode(
    *,
    target: Optional[ProductGrid],
    baked_dir,
    gsd_m: Optional[float],
    site_latlon: Optional[tuple[float, float]],
    out_path,
    body: str,
) -> Optional[_BakedGeoref]:
    """Validate known target geometry before decoding the source.

    With no caller GSD, dimensions depend on decoded source resolution and are
    checked before reprojection allocation. Reuse the georeference to read
    ``meta.json`` only once.
    """
    if baked_dir is None:
        _resolve_target(
            target=target,
            baked_dir=None,
            gsd_m=gsd_m,
            site_latlon=site_latlon,
            out_path=out_path,
            default_out_name="illumination.npz",
            src_gsd=math.nan,
            body=body,
        )
        return None
    baked_path, requested_gsd = _requested_baked_gsd(baked_dir, gsd_m)
    georef = _read_baked_georef(baked_path)
    if requested_gsd is not None:
        _baked_target_grid(georef, requested_gsd)
    return georef


def _resolve_target(
    *,
    target: Optional[ProductGrid],
    baked_dir,
    gsd_m: Optional[float],
    site_latlon: Optional[tuple[float, float]],
    out_path,
    default_out_name: str,
    src_gsd: float,
    body: str,
    baked_georef: Optional[_BakedGeoref] = None,
):
    if baked_dir is not None:
        baked_dir, requested_gsd = _requested_baked_gsd(baked_dir, gsd_m)
        effective_gsd = requested_gsd if requested_gsd is not None else src_gsd
        if requested_gsd is None and (
            not math.isfinite(effective_gsd) or effective_gsd <= 0.0
        ):
            _raise_importer_integrity(
                f"source pixel size is invalid for baked target: {effective_gsd!r}",
                code="ephemeris.importers.source_integrity",
                source=baked_dir,
            )
        georef = (
            baked_georef if baked_georef is not None else _read_baked_georef(baked_dir)
        )
        grid = _baked_target_grid(georef, effective_gsd)
        patch_center_latlon, meta_body = georef.center_latlon, georef.body
        if meta_body != body:
            logging.getLogger("srb").warning(
                "%s: baked_dir meta.json body %r overrides caller-supplied "
                "body %r; the baked meta wins",
                baked_dir,
                meta_body,
                body,
            )
        body = meta_body
        key_dir = _cache_key_dir(baked_dir)
        if out_path is None:
            resolved_out = key_dir / default_out_name
        else:
            try:
                resolved_out = Path(out_path)
            except (TypeError, ValueError) as exc:
                _raise_importer_invalid(
                    f"out_path is invalid: {out_path!r}",
                    code="ephemeris.importers.output_path",
                    cause=exc,
                )
        return grid, patch_center_latlon, body, resolved_out, key_dir

    if site_latlon is None:
        _raise_importer_invalid(
            "`site_latlon` is required when `target` is given directly "
            "(without `baked_dir`, there is no meta.json to read it from)",
            code="ephemeris.importers.site",
        )
    if out_path is None:
        _raise_importer_invalid(
            "`out_path` is required when `target` is given directly "
            "(without `baked_dir`, there is no default location to write to)",
            code="ephemeris.importers.output_path",
        )
    try:
        resolved_site = tuple(float(v) for v in site_latlon)
        resolved_out = Path(out_path)
    except (TypeError, ValueError, OverflowError) as exc:
        _raise_importer_invalid(
            f"direct target site/out_path input is invalid: site_latlon={site_latlon!r}, out_path={out_path!r}",
            code="ephemeris.importers.target",
            cause=exc,
        )
    if len(resolved_site) != 2:
        _raise_importer_invalid(
            f"site_latlon must contain latitude and longitude, got {site_latlon!r}",
            code="ephemeris.importers.site",
        )
    latitude, longitude = resolved_site
    if (
        not math.isfinite(latitude)
        or not math.isfinite(longitude)
        or not -90.0 <= latitude <= 90.0
        or not -180.0 <= longitude <= 180.0
    ):
        _raise_importer_invalid(
            f"site_latlon must contain finite latitude/longitude in valid ranges, "
            f"got {site_latlon!r}",
            code="ephemeris.importers.site",
        )
    if not isinstance(target, ProductGrid):
        _raise_importer_invalid(
            f"target must be a ProductGrid, got {type(target).__name__}",
            code="ephemeris.importers.target",
        )
    return target, resolved_site, body, resolved_out, None


def _import_common(
    arrays: dict,
    *,
    target: Optional[ProductGrid],
    baked_dir,
    gsd_m: Optional[float],
    epoch_range,
    epoch_stride: int,
    out_path,
    site_latlon: Optional[tuple[float, float]],
    source_label_prefix: str,
    source_key: str,
    body: str = "moon",
    baked_georef: Optional[_BakedGeoref] = None,
) -> Path:
    src_crs = arrays["crs"]
    src_transform = arrays["transform"]
    src_h, src_w = arrays["shape"]
    src_gsd = abs(src_transform[0])

    source_label = f"{source_label_prefix}:{source_key}"

    epoch_present = [k for k in _EPOCH_RAW_REQUIRED if arrays.get(k) is not None]
    if epoch_present and len(epoch_present) != len(_EPOCH_RAW_REQUIRED):
        missing = [k for k in _EPOCH_RAW_REQUIRED if k not in epoch_present]
        _raise_importer_integrity(
            f"{source_label}: partial epoch tier -- present {epoch_present}, "
            f"missing {missing}; the epoch tier is all-or-none "
            f"({', '.join(_EPOCH_RAW_REQUIRED)})",
            code="ephemeris.importers.epoch_integrity",
            source=source_label,
        )

    grid, patch_center_latlon, body, resolved_out, resolved_baked_dir = _resolve_target(
        target=target,
        baked_dir=baked_dir,
        gsd_m=gsd_m,
        site_latlon=site_latlon,
        out_path=out_path,
        default_out_name="illumination.npz",
        src_gsd=src_gsd,
        body=body,
        baked_georef=baked_georef,
    )
    _require_valid_body(body, from_source=baked_dir is not None)

    has_epochs = arrays.get("sunlit") is not None
    if has_epochs and not arrays.get("_epoch_selection_applied", False):
        arrays = _apply_epoch_selection(arrays, epoch_range, epoch_stride)
    # Validate selected UTCs even without a sunlit mask; raw products may include
    # pre-2017 epochs outside the requested window.
    if arrays.get("utcs") is not None:
        _require_post_2017(arrays["utcs"])

    try:
        offset_xy_m = _source_frame_correction_geometry(
            src_crs, src_transform, (src_h, src_w), grid
        )
    except (TypeError, ValueError, OverflowError) as exc:
        _raise_importer_integrity(
            f"could not establish source-frame az/el correction geometry: {exc}",
            code="ephemeris.importers.crs_integrity",
            source=source_label,
            cause=exc,
        )
    body_radius_m = _BODY_RADIUS_M[body]

    needs_gamma = (
        arrays.get("sun_azel") is not None or arrays.get("earth_azel") is not None
    )
    if needs_gamma:
        try:
            source_reference_latlon = _source_reference_latlon(
                src_crs, src_transform, (src_h, src_w)
            )
        except (TypeError, ValueError, OverflowError) as exc:
            _raise_importer_integrity(
                f"could not establish source-frame reference location: {exc}",
                code="ephemeris.importers.crs_integrity",
                source=source_label,
                cause=exc,
            )
        gamma = _convergence(*source_reference_latlon, src_crs)
    else:
        gamma = 0.0

    reproj = _reproject_stack(arrays, src_crs, src_transform, grid)
    budget = _allocation_budget(reproj)
    if budget is None:
        budget = _allocation_budget(arrays)
    if budget is None:
        budget = _new_allocation_budget()
        _reserve_retained_arrays(
            arrays,
            budget,
            resource_prefix="illumination final source allocation",
        )
    _reserve_post_reprojection_materialization(
        budget,
        arrays,
        reproj,
        has_epochs=has_epochs,
    )

    sun_azel = (
        _correct_azel(
            arrays["sun_azel"],
            gamma_deg=gamma,
            offset_xy_m=offset_xy_m,
            body_radius_m=body_radius_m,
        )
        if arrays.get("sun_azel") is not None
        else None
    )
    earth_azel = (
        _correct_azel(
            arrays["earth_azel"],
            gamma_deg=gamma,
            offset_xy_m=offset_xy_m,
            body_radius_m=body_radius_m,
        )
        if arrays.get("earth_azel") is not None
        else None
    )

    kwargs: dict = {}
    if has_epochs:
        kwargs["sunlit"] = reproj["sunlit"]
        kwargs["sun_fraction"] = reproj["sun_fraction"]
        kwargs["times_utc"] = np.asarray(arrays["utcs"])
        kwargs["sun_azel"] = sun_azel
        kwargs["sun_angular_diameter_deg"] = np.asarray(
            arrays["sun_ang_diam"], dtype=np.float64
        )
        if reproj.get("earth_los") is not None:
            kwargs["earth_los"] = reproj["earth_los"]
        if earth_azel is not None:
            kwargs["earth_azel"] = earth_azel

    static_keys_present = [key for key in _STATIC_KEYS if reproj.get(key) is not None]
    for key in static_keys_present:
        kwargs[key] = reproj[key]

    meta: dict = {
        "body": body,
        "source": source_label,
        "identity": build_product_identity(
            body=body,
            site_latlon=patch_center_latlon,
            grid=grid,
            times_utc=kwargs.get("times_utc") if has_epochs else None,
        ),
    }
    warnings = list(arrays.get("warnings", ()))
    if warnings:
        meta["warnings"] = warnings

    product = IlluminationProduct(grid=grid, meta=meta, **kwargs)
    if resolved_baked_dir is not None:
        metadata_writer = resolve_terrain_metadata_writer(resolved_baked_dir)
        save_product(product, resolved_out)
        product_sha256 = _published_product_sha256(resolved_out)
        _record_in_terrain_meta(
            resolved_baked_dir,
            resolved_out,
            product,
            source_label,
            static_keys_present=static_keys_present,
            product_sha256=product_sha256,
            metadata_writer=metadata_writer,
        )
    else:
        save_product(product, resolved_out)

    return resolved_out


def _check_target_xor_baked_dir(target, baked_dir) -> None:
    if (target is None) == (baked_dir is None):
        _raise_importer_invalid(
            "exactly one of `target` or `baked_dir` must be given "
            f"(target={target!r}, baked_dir={baked_dir!r})",
            code="ephemeris.importers.target_selection",
        )


def _run_importer(source: object, operation: Callable[[], Path]) -> Path:
    """Map uncategorized public-importer failures to typed errors.

    Preserve failures already classified by readers and lower-level helpers.
    """
    try:
        return operation()
    except FailureError:
        raise
    except ImportError as exc:
        _raise_importer_optional(
            f"ephemeris importer dependency is unavailable for {source}: {exc}",
            source=source,
            cause=exc,
        )
    except OSError as exc:
        _raise_importer_transient(
            f"ephemeris importer I/O failed for {source}: {exc}",
            source=source,
            cause=exc,
        )
    except (
        ValueError,
        KeyError,
        TypeError,
        IndexError,
        OverflowError,
        UnicodeError,
        zipfile.BadZipFile,
    ) as exc:
        _raise_importer_integrity(
            f"ephemeris source or generated product is corrupt for {source}: {exc}",
            source=source,
            cause=exc,
        )
    except Exception as exc:  # noqa: BLE001 - final owned public boundary
        _raise_importer_internal(
            f"ephemeris importer failed internally for {source}: {exc}",
            source=source,
            cause=exc,
        )


def import_lunarlab_zarr(
    source_dir,
    *,
    target: Optional[ProductGrid] = None,
    baked_dir=None,
    gsd_m: Optional[float] = None,
    epoch_range: Optional[tuple[str, str]] = None,
    epoch_stride: int = 1,
    out_path=None,
    site_latlon: Optional[tuple[float, float]] = None,
    body: str = "moon",
) -> Path:
    _require_valid_body(body)
    _check_target_xor_baked_dir(target, baked_dir)
    try:
        source_dir = Path(source_dir)
    except (TypeError, ValueError) as exc:
        _raise_importer_invalid(
            f"source_dir is invalid: {source_dir!r}",
            code="ephemeris.importers.source_path",
            cause=exc,
        )
    baked_georef = _admit_target_before_decode(
        target=target,
        baked_dir=baked_dir,
        gsd_m=gsd_m,
        site_latlon=site_latlon,
        out_path=out_path,
        body=body,
    )
    return _run_importer(
        source_dir,
        lambda: _import_common(
            _read_lunarlab_zarr(
                source_dir, epoch_range=epoch_range, epoch_stride=epoch_stride
            ),
            target=target,
            baked_dir=baked_dir,
            gsd_m=gsd_m,
            epoch_range=epoch_range,
            epoch_stride=epoch_stride,
            out_path=out_path,
            body=body,
            baked_georef=baked_georef,
            site_latlon=site_latlon,
            source_label_prefix="lunarlab-zarr",
            source_key=Path(source_dir).name,
        ),
    )


def import_lunarlab_npz(
    source_npz,
    *,
    target: Optional[ProductGrid] = None,
    baked_dir=None,
    gsd_m: Optional[float] = None,
    epoch_range: Optional[tuple[str, str]] = None,
    epoch_stride: int = 1,
    out_path=None,
    site_latlon: Optional[tuple[float, float]] = None,
    body: str = "moon",
) -> Path:
    _require_valid_body(body)
    _check_target_xor_baked_dir(target, baked_dir)
    try:
        source_npz = Path(source_npz)
    except (TypeError, ValueError) as exc:
        _raise_importer_invalid(
            f"source_npz is invalid: {source_npz!r}",
            code="ephemeris.importers.source_path",
            cause=exc,
        )
    baked_georef = _admit_target_before_decode(
        target=target,
        baked_dir=baked_dir,
        gsd_m=gsd_m,
        site_latlon=site_latlon,
        out_path=out_path,
        body=body,
    )
    return _run_importer(
        source_npz,
        lambda: _import_common(
            _read_lunarlab_npz(source_npz, body=body),
            target=target,
            baked_dir=baked_dir,
            gsd_m=gsd_m,
            epoch_range=epoch_range,
            epoch_stride=epoch_stride,
            out_path=out_path,
            body=body,
            baked_georef=baked_georef,
            site_latlon=site_latlon,
            source_label_prefix="lunarlab-npz",
            source_key=source_npz.stem,
        ),
    )


def import_pgda_lpsr(
    source_tif,
    *,
    target: Optional[ProductGrid] = None,
    baked_dir=None,
    gsd_m: Optional[float] = None,
    out_path=None,
    site_latlon: Optional[tuple[float, float]] = None,
    body: str = "moon",
) -> Path:
    _require_valid_body(body)
    if body != "moon":
        _raise_importer_invalid(
            "PGDA LPSR products are supported for the Moon only",
            code="ephemeris.importers.pgda_body",
            source=body,
        )
    _check_target_xor_baked_dir(target, baked_dir)
    try:
        source_tif = Path(source_tif)
    except (TypeError, ValueError) as exc:
        _raise_importer_invalid(
            f"source_tif is invalid: {source_tif!r}",
            code="ephemeris.importers.source_path",
            cause=exc,
        )
    baked_georef = _admit_target_before_decode(
        target=target,
        baked_dir=baked_dir,
        gsd_m=gsd_m,
        site_latlon=site_latlon,
        out_path=out_path,
        body=body,
    )
    if baked_georef is not None and baked_georef.body != "moon":
        _raise_importer_invalid(
            "PGDA LPSR baked metadata must identify the Moon",
            code="ephemeris.importers.pgda_baked_body",
            source=baked_georef.meta_path,
        )
    return _run_importer(
        source_tif,
        lambda: _import_common(
            _read_pgda_lpsr(source_tif),
            target=target,
            baked_dir=baked_dir,
            gsd_m=gsd_m,
            epoch_range=None,
            epoch_stride=1,
            out_path=out_path,
            body=body,
            baked_georef=baked_georef,
            site_latlon=site_latlon,
            source_label_prefix="pgda-lpsr",
            source_key=Path(source_tif).stem,
        ),
    )
