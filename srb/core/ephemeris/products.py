"""Precomputed illumination products (schema ``srb_illum/2``).

A product is a single ``np.savez_compressed`` archive holding rasters on a
north-up projected grid centered on a terrain patch, plus a ``meta`` JSON
string. Two tiers, both optional but at least one present:

- epoch tier (all-or-none): ``sunlit`` bool ``(T,H,W)``, ``sun_fraction``
  float32 ``(T,H,W)`` with NaN holes, ``times_utc`` ``(T,)`` ISO strings,
  ``sun_azel`` ``(T,2)`` [true-north compass azimuth deg, patch-center
  elevation deg], ``sun_angular_diameter_deg`` ``(T,)``; optionally
  ``earth_los`` bool and ``earth_azel``.
- static tier (per-key optional): ``psr`` uint8 {0,1,255=nodata},
  ``lit_fraction`` float32 NaN holes, ``max_sun_el`` float32 NaN holes.

Isaac-free and import-light: numpy + stdlib only.
"""

from __future__ import annotations

import json
import operator
import zipfile
from bisect import bisect_left
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from types import MappingProxyType
from typing import (
    BinaryIO,
    NoReturn,
    Optional,
    SupportsFloat,
    SupportsIndex,
    TypeAlias,
    cast,
)

import numpy as np

from srb.utils.failures import (
    FailureError,
    IntegrityFailureError,
    InvalidInputError,
    TransientExternalFailureError,
)
from srb.utils.json_types import ImmutableValueMixin
from srb.utils.publication import (
    ArtifactPublication,
    PublicationCleanupError,
    PublicationError,
    PublicationIntegrityError,
    PublicationTimeoutError,
    sha256_file,
)
from srb.utils.resource_budget import (
    MAX_ILLUMINATION_ARCHIVE_BYTES,
    MAX_ILLUMINATION_ARRAY_BYTES,
    MAX_ILLUMINATION_DECODED_BYTES,
    MAX_IMAGE_PIXELS,
    MAX_METADATA_ATTRIBUTE_BYTES,
    ByteBudget,
    checked_shape_nbytes,
    inspect_npz_archive,
    require_bounded_int,
    require_file_size,
)

Metadata: TypeAlias = Mapping[str, object]

SCHEMA_ID = "srb_illum/2"
LEGACY_SCHEMA_ID = "srb_illum/1"
PRODUCT_SCHEMA_VERSION = 2
PRODUCT_IDENTITY_VERSION = 1
PSR_NODATA = 255

_SUPPORTED_BODIES = frozenset(("moon", "mars", "earth"))
_SITE_TOLERANCE_DEG = 1.0e-6
_SITE_ALT_TOLERANCE_M = 1.0e-3
_EPOCH_MIN_UTC = datetime(2017, 1, 1, tzinfo=timezone.utc)
_EPOCH_MAX_UTC = datetime(2149, 12, 31, 23, 59, 59, 999999, tzinfo=timezone.utc)

_EPOCH_GROUP = (
    "sunlit",
    "sun_fraction",
    "times_utc",
    "sun_azel",
    "sun_angular_diameter_deg",
)
_EPOCH_OPTIONAL = ("earth_los", "earth_azel")
_STATIC_KEYS = ("psr", "lit_fraction", "max_sun_el")
_PRODUCT_ARRAY_KEYS = (*_EPOCH_GROUP, *_EPOCH_OPTIONAL, *_STATIC_KEYS)

# TT-UTC for epochs >= 2017-01-01 (32.184 s + 37 leap seconds); TDB==TT to <2 ms.
_TT_MINUS_UTC_S = 69.184
_J2000_UTC = datetime(2000, 1, 1, 12, 0, 0, tzinfo=timezone.utc)

_PRODUCT_COMPONENT = "ephemeris.products"


class IlluminationProductError(ValueError):
    """Base error for invalid or incompatible illumination products."""


class IlluminationProductValidationError(IlluminationProductError):
    """The product bytes or metadata violate the product contract."""


class ProductIntegrityFailure(
    IntegrityFailureError, IlluminationProductValidationError
):
    """Typed product-integrity failure retaining the proven ValueError API."""


class ProductIdentityError(IlluminationProductValidationError):
    """The product identity is incompatible with a requested site."""


class ProductIdentityInputError(InvalidInputError, ProductIdentityError):
    """A query site does not match a valid product identity."""


class ProductSamplingIndexError(InvalidInputError, IndexError):
    """A sampling epoch index is invalid, with legacy IndexError compatibility."""


def _failure_details(
    *, path: Path | None = None, cause: BaseException | None = None
) -> dict[str, object]:
    details: dict[str, object] = {}
    if path is not None:
        details["path"] = str(path)
    if cause is not None:
        details["cause_type"] = f"{type(cause).__module__}.{type(cause).__qualname__}"
    return details


def _raise_invalid_product_input(
    message: str,
    *,
    code: str = "ephemeris.products.invalid_input",
    path: Path | None = None,
    cause: BaseException | None = None,
) -> NoReturn:
    error = InvalidInputError(
        code=code,
        message=message,
        component=_PRODUCT_COMPONENT,
        details=_failure_details(path=path, cause=cause),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


def _raise_product_identity_input(message: str) -> NoReturn:
    raise ProductIdentityInputError(
        code="ephemeris.products.identity_input",
        message=message,
        component=_PRODUCT_COMPONENT,
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )


def _raise_product_integrity(
    message: str,
    *,
    code: str = "ephemeris.products.integrity",
    path: Path | None = None,
    cause: BaseException | None = None,
) -> NoReturn:
    error = IntegrityFailureError(
        code=code,
        message=message,
        component=_PRODUCT_COMPONENT,
        details=_failure_details(path=path, cause=cause),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


def _raise_product_validation(cause: BaseException) -> NoReturn:
    error = ProductIntegrityFailure(
        code="ephemeris.products.validation",
        message=f"illumination product violates its contract: {cause}",
        component=_PRODUCT_COMPONENT,
        details=_failure_details(cause=cause),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    raise error from cause


def _raise_product_io(
    message: str,
    *,
    code: str = "ephemeris.products.io",
    path: Path | None = None,
    cause: BaseException | None = None,
    cleanup_error: PublicationCleanupError | None = None,
) -> NoReturn:
    error = TransientExternalFailureError(
        code=code,
        message=message,
        component=_PRODUCT_COMPONENT,
        details=_failure_details(path=path, cause=cause),
        retryable=True,
        fallback_allowed=True,
        evidence_valid=False,
    )
    if cleanup_error is not None:
        # Keep the domain-level transient classification while retaining the
        # publication owner and the exact destinations already committed.
        setattr(error, "owner", cleanup_error.owner)
        setattr(error, "cleanup_owner", cleanup_error.owner)
        setattr(error, "committed_paths", cleanup_error.committed_paths)
        setattr(error, "committed_result", cleanup_error.committed_result)
    if cause is None:
        raise error
    raise error from cause


def _metadata_dict(value: object, *, error: str) -> dict[str, object]:
    if not isinstance(value, Mapping):
        raise IlluminationProductValidationError(error)
    result: dict[str, object] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise IlluminationProductValidationError(
                "product metadata object keys must be strings"
            )
        result[key] = item
    return result


def _freeze_metadata(value: object) -> object:
    """Recursively detach JSON-shaped metadata from caller-owned containers."""
    if isinstance(value, Mapping):
        frozen: dict[str, object] = {}
        for key, item in value.items():
            # Preserve malformed keys so ``validate()`` remains the error seam.
            frozen[cast(str, key)] = _freeze_metadata(item)
        return MappingProxyType(frozen)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_metadata(item) for item in value)
    return value


def _thaw_metadata(value: object) -> object:
    """Return ordinary JSON containers for the public product serializer."""
    if isinstance(value, Mapping):
        return {cast(str, key): _thaw_metadata(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_thaw_metadata(item) for item in value]
    return value


def _snapshot_array(value: np.ndarray) -> np.ndarray:
    """Own an array snapshot and expose it through NumPy's read-only flag."""
    snapshot = value.copy()
    snapshot.setflags(write=False)
    return snapshot


def _parse_utc(epoch_utc: object) -> datetime:
    dt = datetime.fromisoformat(str(epoch_utc).replace("Z", "+00:00"))
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(timezone.utc)


def _finite_float(value: object, *, name: str) -> float:
    try:
        if not (
            isinstance(value, (str, bytes, bytearray))
            or isinstance(value, SupportsFloat)
            or isinstance(value, SupportsIndex)
        ):
            raise TypeError
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise IlluminationProductValidationError(
            f"{name} must be a finite number, got {value!r}"
        ) from exc
    if not np.isfinite(result):
        raise IlluminationProductValidationError(
            f"{name} must be finite, got {value!r}"
        )
    return result


def _validate_body(body: object, *, name: str = "body") -> str:
    body = str(body)
    if body not in _SUPPORTED_BODIES:
        raise IlluminationProductValidationError(
            f"{name} {body!r} not in {sorted(_SUPPORTED_BODIES)}"
        )
    return body


def _validate_site_center(
    site_latlon: Sequence[object] | None,
) -> tuple[float, float]:
    if site_latlon is None:
        raise IlluminationProductValidationError(
            "product identity requires a surface site center"
        )
    try:
        lat, lon = site_latlon
    except (TypeError, ValueError) as exc:
        raise IlluminationProductValidationError(
            "site center must contain latitude and longitude"
        ) from exc
    lat = _finite_float(lat, name="site center latitude")
    lon = _finite_float(lon, name="site center longitude")
    if not -90.0 <= lat <= 90.0:
        raise IlluminationProductValidationError(
            f"site center latitude {lat} outside [-90, 90]"
        )
    if not -180.0 <= lon <= 180.0:
        raise IlluminationProductValidationError(
            f"site center longitude {lon} outside [-180, 180]"
        )
    return lat, lon


def _parse_epoch_series(times_utc: Iterable[object] | None) -> list[datetime]:
    if times_utc is None:
        raise IlluminationProductValidationError("product has no epoch timestamps")
    try:
        values = list(times_utc)
    except TypeError as exc:
        raise IlluminationProductValidationError(
            "times_utc must be a one-dimensional sequence"
        ) from exc
    if not values:
        raise IlluminationProductValidationError("times_utc must not be empty")
    parsed: list[datetime] = []
    for value in values:
        try:
            dt = _parse_utc(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise IlluminationProductValidationError(
                f"epoch {value!r} must be a finite ISO-8601 UTC timestamp"
            ) from exc
        if not np.isfinite(dt.timestamp()):
            raise IlluminationProductValidationError(f"epoch {value!r} must be finite")
        if not (_EPOCH_MIN_UTC <= dt <= _EPOCH_MAX_UTC):
            raise IlluminationProductValidationError(
                f"epoch {value!r} outside supported range "
                f"[{_EPOCH_MIN_UTC.date()}, {_EPOCH_MAX_UTC.date()}]"
            )
        parsed.append(dt)
    if len(parsed) > 1:
        deltas = np.asarray(
            [(b - a).total_seconds() for a, b in zip(parsed, parsed[1:])],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(deltas)) or np.any(deltas <= 0.0):
            raise IlluminationProductValidationError(
                "times_utc must be finite and strictly increasing "
                "(descending and duplicate epochs are invalid)"
            )
    return parsed


def _cadence_for_times(parsed: list[datetime]) -> Metadata:
    if len(parsed) == 1:
        return {"kind": "single", "step_s": None, "tolerance_s": 3600.0}
    deltas = np.asarray(
        [(b - a).total_seconds() for a, b in zip(parsed, parsed[1:])],
        dtype=np.float64,
    )
    step = float(deltas[0])
    if np.allclose(deltas, step, rtol=0.0, atol=1.0e-6):
        return {
            "kind": "uniform",
            "step_s": step,
            "tolerance_s": step / 2.0,
        }
    return {
        "kind": "irregular",
        "step_s": None,
        "tolerance_s": float(np.min(deltas) / 2.0),
    }


def _physical_identity() -> Metadata:
    return {
        "model": "solar_disk_v1",
        "azimuth_unit": "deg",
        "elevation_unit": "deg",
        "angular_diameter_unit": "deg",
        "azimuth_range_deg": [0.0, 360.0],
        "elevation_range_deg": [-90.0, 90.0],
        "fraction_range": [0.0, 1.0],
        "solar_angular_diameter_range_deg": [0.0, 180.0],
    }


def build_product_identity(
    *,
    body: str,
    site_latlon: Sequence[object] | None,
    grid: ProductGrid,
    times_utc: Iterable[object] | None = None,
) -> Metadata:
    """Build the versioned identity metadata required by schema v2."""
    body = _validate_body(body)
    lat, lon = _validate_site_center(site_latlon)
    cadence: Metadata
    if times_utc is None:
        cadence = {"kind": "static", "step_s": None, "tolerance_s": None}
    else:
        cadence = _cadence_for_times(_parse_epoch_series(times_utc))
    return {
        "version": PRODUCT_IDENTITY_VERSION,
        "body": body,
        "site": {
            "center_lat_deg": lat,
            "center_lon_deg": lon,
            "alt_m": 0.0,
            "tolerance_deg": _SITE_TOLERANCE_DEG,
            "alt_tolerance_m": _SITE_ALT_TOLERANCE_M,
        },
        "frame": {
            "crs": str(grid.crs),
            "x_axis": "grid_east",
            "y_axis": "grid_north",
            "azimuth_convention": "true_north_compass_deg",
            "elevation_reference": "patch_center_deg",
        },
        "cadence": cadence,
        "physical": _physical_identity(),
    }


def _validate_numeric_range(
    array: np.ndarray,
    *,
    name: str,
    lower: float,
    upper: float,
    allow_nan: bool = False,
) -> None:
    try:
        values = np.asarray(array, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise IlluminationProductValidationError(
            f"{name} must contain numeric values"
        ) from exc
    finite = np.isfinite(values)
    if allow_nan:
        invalid = np.isinf(values)
    else:
        invalid = ~finite
    if np.any(invalid):
        raise IlluminationProductValidationError(f"{name} contains non-finite values")
    checked = values[finite]
    if checked.size and (np.any(checked < lower) or np.any(checked > upper)):
        raise IlluminationProductValidationError(
            f"{name} values must be within [{lower}, {upper}]"
        )


def _validate_cadence(
    identity: Metadata, parsed: list[datetime], *, has_epochs: bool
) -> None:
    cadence = _metadata_dict(
        identity.get("cadence"),
        error="product identity cadence must be an object",
    )
    kind = cadence.get("kind")
    if has_epochs:
        if kind not in ("single", "uniform", "irregular"):
            raise IlluminationProductValidationError(
                f"epoch product has unsupported cadence kind {kind!r}"
            )
        if kind == "single" and len(parsed) != 1:
            raise IlluminationProductValidationError(
                "single cadence requires exactly one epoch"
            )
        if kind != "single" and len(parsed) < 2:
            raise IlluminationProductValidationError(
                f"{kind} cadence requires at least two epochs"
            )
        tolerance = _finite_float(
            cadence.get("tolerance_s"), name="cadence tolerance_s"
        )
        if tolerance <= 0.0:
            raise IlluminationProductValidationError(
                "cadence tolerance_s must be positive"
            )
        if kind == "single":
            if tolerance != 3600.0:
                raise IlluminationProductValidationError(
                    "single cadence tolerance_s must be 3600"
                )
            if cadence.get("step_s") is not None:
                raise IlluminationProductValidationError(
                    "single cadence step_s must be null"
                )
            return
        deltas = np.asarray(
            [(b - a).total_seconds() for a, b in zip(parsed, parsed[1:])],
            dtype=np.float64,
        )
        if kind == "uniform":
            step = _finite_float(cadence.get("step_s"), name="cadence step_s")
            if step <= 0.0 or not np.allclose(deltas, step, rtol=0.0, atol=1.0e-6):
                raise IlluminationProductValidationError(
                    "uniform cadence step_s does not match epoch spacing"
                )
            if tolerance > step / 2.0 + 1.0e-6:
                raise IlluminationProductValidationError(
                    "uniform cadence tolerance_s is wider than half a step"
                )
        else:
            if cadence.get("step_s") is not None:
                step = _finite_float(cadence.get("step_s"), name="cadence step_s")
                if step <= 0.0:
                    raise IlluminationProductValidationError(
                        "irregular cadence step_s must be positive when present"
                    )
            if tolerance > float(np.min(deltas)) / 2.0 + 1.0e-6:
                raise IlluminationProductValidationError(
                    "irregular cadence tolerance_s is wider than half the nearest gap"
                )
    elif kind != "static":
        raise IlluminationProductValidationError(
            f"static product has unsupported cadence kind {kind!r}"
        )
    elif cadence.get("step_s") is not None or cadence.get("tolerance_s") is not None:
        raise IlluminationProductValidationError(
            "static cadence step_s and tolerance_s must be null"
        )


def _validate_product_identity(
    meta: object,
    grid: ProductGrid,
    parsed: list[datetime],
    *,
    has_epochs: bool,
) -> Metadata:
    meta = _metadata_dict(meta, error="product metadata must be an object")
    schema = meta.get("schema")
    if schema is not None and schema != SCHEMA_ID:
        raise IlluminationProductValidationError(
            f"unsupported schema {schema!r} (expected {SCHEMA_ID})"
        )
    if "product_version" not in meta:
        raise IlluminationProductValidationError(
            "product metadata requires product_version "
            f"(expected {PRODUCT_SCHEMA_VERSION})"
        )
    product_version = meta.get("product_version")
    if isinstance(product_version, bool) or product_version != PRODUCT_SCHEMA_VERSION:
        raise IlluminationProductValidationError(
            f"unsupported product_version {product_version!r} "
            f"(expected {PRODUCT_SCHEMA_VERSION})"
        )
    identity = _metadata_dict(
        meta.get("identity"),
        error="product metadata requires versioned identity",
    )
    version = identity.get("version")
    if isinstance(version, bool) or version != PRODUCT_IDENTITY_VERSION:
        raise IlluminationProductValidationError(
            f"unsupported product identity version {version!r} "
            f"(expected {PRODUCT_IDENTITY_VERSION})"
        )
    body = _validate_body(identity.get("body"), name="identity.body")
    if meta.get("body") is not None and meta.get("body") != body:
        raise ProductIdentityError(
            f"metadata body {meta.get('body')!r} does not match identity body {body!r}"
        )

    site = _metadata_dict(
        identity.get("site"),
        error="product identity site must be an object",
    )
    lat = _finite_float(site.get("center_lat_deg"), name="identity site latitude")
    lon = _finite_float(site.get("center_lon_deg"), name="identity site longitude")
    if not -90.0 <= lat <= 90.0:
        raise IlluminationProductValidationError(
            f"identity site latitude {lat} outside [-90, 90]"
        )
    if not -180.0 <= lon <= 180.0:
        raise IlluminationProductValidationError(
            f"identity site longitude {lon} outside [-180, 180]"
        )
    _finite_float(site.get("alt_m"), name="identity site alt_m")
    tolerance_deg = _finite_float(
        site.get("tolerance_deg"), name="identity site tolerance_deg"
    )
    alt_tolerance_m = _finite_float(
        site.get("alt_tolerance_m"), name="identity site alt_tolerance_m"
    )
    if tolerance_deg <= 0.0 or alt_tolerance_m <= 0.0:
        raise IlluminationProductValidationError(
            "identity site tolerances must be positive"
        )

    frame = _metadata_dict(
        identity.get("frame"),
        error="product identity frame must be an object",
    )
    if frame.get("crs") != grid.crs:
        raise ProductIdentityError(
            f"identity frame CRS {frame.get('crs')!r} does not match grid CRS {grid.crs!r}"
        )
    expected_frame = {
        "x_axis": "grid_east",
        "y_axis": "grid_north",
        "azimuth_convention": "true_north_compass_deg",
        "elevation_reference": "patch_center_deg",
    }
    for key, expected in expected_frame.items():
        if frame.get(key) != expected:
            raise IlluminationProductValidationError(
                f"identity frame {key} must be {expected!r}"
            )

    physical = _metadata_dict(
        identity.get("physical"),
        error="product identity physical model must be an object",
    )
    for key, physical_expected in _physical_identity().items():
        actual = physical.get(key)
        if isinstance(physical_expected, list):
            try:
                actual_array = np.asarray(actual, dtype=np.float64)
            except (TypeError, ValueError) as exc:
                raise IlluminationProductValidationError(
                    f"identity physical {key} does not match the schema"
                ) from exc
            if actual_array.shape != (len(physical_expected),) or not np.allclose(
                actual_array, physical_expected, rtol=0.0, atol=0.0
            ):
                raise IlluminationProductValidationError(
                    f"identity physical {key} does not match the schema"
                )
        elif actual != physical_expected:
            raise IlluminationProductValidationError(
                f"identity physical {key} must be {physical_expected!r}"
            )
    _validate_cadence(identity, parsed, has_epochs=has_epochs)
    return identity


def approx_et_seconds(epoch_utc: str) -> float:
    """Approximate TDB seconds past J2000; valid from 2017 within 2 ms."""
    return (_parse_utc(epoch_utc) - _J2000_UTC).total_seconds() + _TT_MINUS_UTC_S


@dataclass(frozen=True)
class ProductGrid:
    """North-up projected grid; GeoTIFF-style transform, pixel-center sampling."""

    crs: str
    transform: tuple[float, float, float, float, float, float]
    shape: tuple[int, int]
    patch_center_xy_m: tuple[float, float]

    def __post_init__(self) -> None:
        try:
            a, b, c, d, e, f = (float(v) for v in self.transform)
        except (TypeError, ValueError, OverflowError) as exc:
            raise IlluminationProductValidationError(
                f"ProductGrid transform must contain six finite numbers, got {self.transform}"
            ) from exc
        if not np.all(np.isfinite((a, b, c, d, e, f))):
            raise IlluminationProductValidationError(
                f"ProductGrid transform must be finite, got {self.transform}"
            )
        if b != 0.0 or d != 0.0 or a <= 0.0 or e >= 0.0:
            raise IlluminationProductValidationError(
                f"ProductGrid requires a north-up axis-aligned transform, got {self.transform}"
            )
        try:
            raw_shape = tuple(self.shape)
            if len(raw_shape) != 2:
                raise ValueError("must contain exactly two dimensions")
            shape = tuple(
                require_bounded_int(
                    value,
                    resource=f"ProductGrid shape[{index}]",
                    minimum=1,
                    maximum=MAX_IMAGE_PIXELS,
                )
                for index, value in enumerate(raw_shape)
            )
            # Bound float64 reprojection destinations before allocation.
            checked_shape_nbytes(
                shape,
                np.dtype(np.float64),
                resource="ProductGrid destination shape",
                max_elements=MAX_IMAGE_PIXELS,
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
        except (TypeError, ValueError, OverflowError) as exc:
            if isinstance(exc, IlluminationProductValidationError):
                raise
            raise IlluminationProductValidationError(
                f"ProductGrid shape is invalid or exceeds its resource limit: {self.shape}"
            ) from exc
        center = (float(self.patch_center_xy_m[0]), float(self.patch_center_xy_m[1]))
        if not np.all(np.isfinite(center)):
            raise IlluminationProductValidationError(
                f"ProductGrid patch center must be finite, got {self.patch_center_xy_m}"
            )
        crs = str(self.crs)
        if not crs or crs == "None":
            raise IlluminationProductValidationError("ProductGrid requires a CRS")
        object.__setattr__(self, "transform", (a, b, c, d, e, f))
        object.__setattr__(self, "shape", shape)
        object.__setattr__(self, "patch_center_xy_m", center)
        object.__setattr__(self, "crs", crs)

    @property
    def gsd_m(self) -> float:
        return float(self.transform[0])

    def local_xy_to_rowcol_f(
        self, x: object, y: object
    ) -> tuple[np.ndarray, np.ndarray]:
        """Map patch-center-relative east/north meters to float pixel coordinates."""
        a, _, c, _, e, f = self.transform
        wx = np.asarray(x, dtype=np.float64) + self.patch_center_xy_m[0]
        wy = np.asarray(y, dtype=np.float64) + self.patch_center_xy_m[1]
        col = (wx - c) / a - 0.5
        row = (wy - f) / e - 0.5
        return row, col

    def rowcol_to_local_xy(
        self, row: object, col: object
    ) -> tuple[np.ndarray, np.ndarray]:
        """Map float pixel coordinates to patch-center-relative east/north meters.

        Exact inverse of :meth:`local_xy_to_rowcol_f` for any float coordinates.
        """
        a, _, c, _, e, f = self.transform
        row_array: np.ndarray = np.asarray(row, dtype=np.float64)
        col_array: np.ndarray = np.asarray(col, dtype=np.float64)
        x = c + (col_array + 0.5) * a - self.patch_center_xy_m[0]
        y = f + (row_array + 0.5) * e - self.patch_center_xy_m[1]
        return x, y


@dataclass(frozen=True)
class IlluminationProduct(ImmutableValueMixin):
    grid: ProductGrid
    meta: Metadata = field(default_factory=dict)
    # epoch tier
    sunlit: Optional[np.ndarray] = None
    sun_fraction: Optional[np.ndarray] = None
    earth_los: Optional[np.ndarray] = None
    times_utc: Optional[np.ndarray] = None
    sun_azel: Optional[np.ndarray] = None
    earth_azel: Optional[np.ndarray] = None
    sun_angular_diameter_deg: Optional[np.ndarray] = None
    # static tier
    psr: Optional[np.ndarray] = None
    lit_fraction: Optional[np.ndarray] = None
    max_sun_el: Optional[np.ndarray] = None

    def __post_init__(self) -> None:
        """Publish detached, read-only snapshots at the value-object seam."""
        frozen_meta = _freeze_metadata(self.meta)
        if isinstance(frozen_meta, Mapping):
            object.__setattr__(self, "meta", cast(Metadata, frozen_meta))
        for key in _PRODUCT_ARRAY_KEYS:
            value = getattr(self, key)
            if isinstance(value, np.ndarray):
                object.__setattr__(self, key, _snapshot_array(value))

    @property
    def has_epochs(self) -> bool:
        return self.sunlit is not None

    @property
    def epoch_count(self) -> int:
        return 0 if self.times_utc is None else int(len(self.times_utc))

    @property
    def epoch_step_s(self) -> Optional[float]:
        if self.times_utc is None or len(self.times_utc) < 2:
            return None
        t0 = _parse_utc(str(self.times_utc[0]))
        t1 = _parse_utc(str(self.times_utc[1]))
        return (t1 - t0).total_seconds()

    def _validate_contract(self) -> None:
        h, w = self.grid.shape
        epoch_present = [k for k in _EPOCH_GROUP if getattr(self, k) is not None]
        if epoch_present and len(epoch_present) != len(_EPOCH_GROUP):
            missing = sorted(set(_EPOCH_GROUP) - set(epoch_present))
            raise IlluminationProductValidationError(
                f"epoch tier is all-or-none; missing {missing}"
            )
        static_present = [k for k in _STATIC_KEYS if getattr(self, k) is not None]
        if not epoch_present and not static_present:
            raise IlluminationProductValidationError(
                "product must carry at least one tier (epoch or static)"
            )
        parsed: list[datetime] = []
        if epoch_present:
            times = self.times_utc
            sunlit = self.sunlit
            sun_fraction = self.sun_fraction
            sun_azel = self.sun_azel
            sun_angular_diameter_deg = self.sun_angular_diameter_deg
            if (
                times is None
                or sunlit is None
                or sun_fraction is None
                or sun_azel is None
                or sun_angular_diameter_deg is None
            ):
                raise IlluminationProductValidationError(
                    "epoch tier is all-or-none; required raster is missing"
                )
            times_array = np.asarray(times)
            if times_array.ndim != 1:
                raise IlluminationProductValidationError(
                    f"times_utc must be one-dimensional, got shape {times_array.shape}"
                )
            parsed = _parse_epoch_series(times_array)
            t = len(times)
            for key, arr in (
                ("sunlit", sunlit),
                ("sun_fraction", sun_fraction),
                ("earth_los", self.earth_los),
            ):
                if arr is not None and arr.shape != (t, h, w):
                    raise IlluminationProductValidationError(
                        f"{key} shape {arr.shape} != (T,H,W)=({t},{h},{w})"
                    )
            for key, arr, cols in (
                ("sun_azel", sun_azel, 2),
                ("earth_azel", self.earth_azel, 2),
            ):
                if arr is not None and arr.shape != (t, cols):
                    raise IlluminationProductValidationError(
                        f"{key} shape {arr.shape} != ({t},{cols})"
                    )
            if sun_angular_diameter_deg.shape != (t,):
                raise IlluminationProductValidationError(
                    f"sun_angular_diameter_deg shape {sun_angular_diameter_deg.shape} != ({t},)"
                )
            for key in ("sunlit", "earth_los"):
                arr = getattr(self, key)
                if arr is not None and arr.dtype != np.dtype(bool):
                    raise IlluminationProductValidationError(
                        f"{key} must have boolean dtype, got {arr.dtype}"
                    )
            _validate_numeric_range(
                sun_fraction,
                name="sun_fraction",
                lower=0.0,
                upper=1.0,
                allow_nan=True,
            )
            _validate_numeric_range(
                sun_azel[:, 0],
                name="sun_azel azimuth",
                lower=0.0,
                upper=360.0,
            )
            _validate_numeric_range(
                sun_azel[:, 1],
                name="sun_azel elevation",
                lower=-90.0,
                upper=90.0,
            )
            _validate_numeric_range(
                sun_angular_diameter_deg,
                name="sun_angular_diameter_deg",
                lower=0.0,
                upper=180.0,
            )
            if np.any(np.asarray(sun_angular_diameter_deg) <= 0.0):
                raise IlluminationProductValidationError(
                    "sun_angular_diameter_deg must be positive"
                )
            if self.earth_azel is not None:
                _validate_numeric_range(
                    self.earth_azel[:, 0],
                    name="earth_azel azimuth",
                    lower=0.0,
                    upper=360.0,
                )
                _validate_numeric_range(
                    self.earth_azel[:, 1],
                    name="earth_azel elevation",
                    lower=-90.0,
                    upper=90.0,
                )
        for key in _STATIC_KEYS:
            arr = getattr(self, key)
            if arr is not None and arr.shape != (h, w):
                raise IlluminationProductValidationError(
                    f"{key} shape {arr.shape} != (H,W)=({h},{w})"
                )
        if self.psr is not None:
            if self.psr.dtype != np.dtype(np.uint8):
                raise IlluminationProductValidationError(
                    f"psr must have uint8 dtype, got {self.psr.dtype}"
                )
            if np.any(~np.isin(self.psr, (0, 1, PSR_NODATA))):
                raise IlluminationProductValidationError(
                    "psr values must be one of {0, 1, 255}"
                )
        if self.lit_fraction is not None:
            _validate_numeric_range(
                self.lit_fraction,
                name="lit_fraction",
                lower=0.0,
                upper=1.0,
                allow_nan=True,
            )
        if self.max_sun_el is not None:
            if self.max_sun_el.dtype.kind != "f":
                raise IlluminationProductValidationError(
                    f"max_sun_el must have a floating dtype, got {self.max_sun_el.dtype}"
                )
            _validate_numeric_range(
                self.max_sun_el,
                name="max_sun_el",
                lower=-90.0,
                upper=90.0,
                allow_nan=True,
            )
        _validate_product_identity(
            self.meta,
            self.grid,
            parsed,
            has_epochs=bool(epoch_present),
        )

    def validate(self) -> None:
        """Validate the product; malformed state raises an integrity failure.

        The failure also subclasses ``ValueError`` for caller compatibility.
        """
        try:
            self._validate_contract()
        except FailureError:
            raise
        except (
            AttributeError,
            KeyError,
            IndexError,
            OverflowError,
            TypeError,
            ValueError,
        ) as exc:
            _raise_product_validation(exc)

    def validate_site(self, site: object) -> None:
        """Reject a query site that is not the product's declared site."""
        self.validate()
        self._validate_site_snapshot(site)

    def _validate_site_snapshot(self, site: object) -> None:
        """Check a query against an already-validated immutable snapshot."""
        if not getattr(site, "is_surface", False):
            _raise_product_identity_input(
                "table illumination products require a surface query site"
            )
        identity = _metadata_dict(
            self.meta.get("identity"),
            error="product metadata requires versioned identity",
        )
        expected_body = _validate_body(identity.get("body"), name="identity.body")
        query_body = getattr(site, "body", None)
        if query_body != expected_body:
            _raise_product_identity_input(
                f"table product body {expected_body!r} cannot answer "
                f"query body {query_body!r}"
            )
        try:
            query_lat = _finite_float(
                getattr(site, "lat_deg", None), name="query site latitude"
            )
            query_lon = _finite_float(
                getattr(site, "lon_deg", None), name="query site longitude"
            )
        except FailureError:
            raise
        except (TypeError, ValueError) as exc:
            _raise_invalid_product_input(
                f"query site coordinates are invalid: {exc}",
                code="ephemeris.products.identity_input",
                cause=exc,
            )
        if not -90.0 <= query_lat <= 90.0:
            _raise_product_identity_input(
                f"query site latitude {query_lat} outside [-90, 90]"
            )
        if not -180.0 <= query_lon <= 180.0:
            _raise_product_identity_input(
                f"query site longitude {query_lon} outside [-180, 180]"
            )
        declared = _metadata_dict(
            identity.get("site"),
            error="product identity site must be an object",
        )
        declared_lat = _finite_float(
            declared["center_lat_deg"], name="identity site latitude"
        )
        declared_lon = _finite_float(
            declared["center_lon_deg"], name="identity site longitude"
        )
        declared_tolerance = _finite_float(
            declared["tolerance_deg"], name="identity site tolerance_deg"
        )
        lat_delta = abs(query_lat - declared_lat)
        lon_delta = abs((query_lon - declared_lon + 180.0) % 360.0 - 180.0)
        if lat_delta > declared_tolerance or lon_delta > declared_tolerance:
            _raise_product_identity_input(
                "table product site center does not match query site "
                f"(query=({query_lat}, {query_lon}), "
                f"product=({declared['center_lat_deg']}, "
                f"{declared['center_lon_deg']}), "
                f"tolerance={declared['tolerance_deg']} deg)"
            )
        try:
            query_alt = _finite_float(
                getattr(site, "alt_m", 0.0), name="query site alt_m"
            )
        except FailureError:
            raise
        except (TypeError, ValueError) as exc:
            _raise_invalid_product_input(
                f"query site altitude is invalid: {exc}",
                code="ephemeris.products.identity_input",
                cause=exc,
            )
        declared_alt = _finite_float(declared["alt_m"], name="identity site alt_m")
        declared_alt_tolerance = _finite_float(
            declared["alt_tolerance_m"], name="identity site alt_tolerance_m"
        )
        if abs(query_alt - declared_alt) > declared_alt_tolerance:
            _raise_product_identity_input(
                "table product site altitude does not match query site "
                f"(query={query_alt}, product={declared['alt_m']}, "
                f"tolerance={declared['alt_tolerance_m']} m)"
            )

    def epoch_index(
        self, epoch_utc: str, *, tolerance_s: Optional[float] = None
    ) -> int:
        self.validate()
        return self._epoch_index_snapshot(epoch_utc, tolerance_s=tolerance_s)

    def _epoch_index_snapshot(
        self,
        epoch_utc: str,
        *,
        tolerance_s: Optional[float] = None,
        parsed_times: Sequence[datetime] | None = None,
    ) -> int:
        """Resolve an epoch on a product snapshot validated by its loader."""
        times_utc = self.times_utc
        if times_utc is None:
            raise IlluminationProductValidationError("product has no epoch tier")
        try:
            target = _parse_epoch_series([epoch_utc])[0]
        except (TypeError, ValueError, OverflowError) as exc:
            _raise_invalid_product_input(
                f"query epoch {epoch_utc!r} is not a valid ISO-8601 timestamp",
                code="ephemeris.products.epoch_input",
                cause=exc,
            )
        if parsed_times is None:
            parsed_times = tuple(_parse_utc(str(t)) for t in times_utc)
        insertion = bisect_left(parsed_times, target)
        candidates = tuple(
            index
            for index in (insertion - 1, insertion)
            if 0 <= index < len(parsed_times)
        )
        idx = min(
            candidates,
            key=lambda index: abs((target - parsed_times[index]).total_seconds()),
        )
        offset_s = (target - parsed_times[idx]).total_seconds()
        if tolerance_s is None:
            identity = _metadata_dict(
                self.meta.get("identity"),
                error="product metadata requires versioned identity",
            )
            cadence = _metadata_dict(
                identity.get("cadence"),
                error="product identity cadence must be an object",
            )
            tolerance = _finite_float(
                cadence.get("tolerance_s"), name="cadence tolerance_s"
            )
        else:
            try:
                tolerance = _finite_float(tolerance_s, name="query tolerance_s")
            except FailureError:
                raise
            except (TypeError, ValueError) as exc:
                _raise_invalid_product_input(
                    f"query tolerance_s is invalid: {exc}",
                    code="ephemeris.products.epoch_input",
                    cause=exc,
                )
            if tolerance <= 0.0:
                _raise_invalid_product_input(
                    "query tolerance_s must be positive",
                    code="ephemeris.products.epoch_input",
                )
        if abs(offset_s) > tolerance:
            _raise_invalid_product_input(
                f"epoch {epoch_utc} is {abs(offset_s):.0f}s from the nearest "
                f"product epoch {times_utc[idx]} (tolerance {tolerance:.0f}s)",
                code="ephemeris.products.epoch_input",
            )
        return idx

    def _sample(self, arr: np.ndarray, x: object, y: object) -> np.ndarray:
        row_f, col_f = self.grid.local_xy_to_rowcol_f(x, y)
        h, w = arr.shape
        if arr.dtype.kind == "f":
            out = np.full(np.shape(row_f), np.nan, dtype=np.float64)
            r0 = np.floor(row_f).astype(int)
            c0 = np.floor(col_f).astype(int)
            inb = (row_f >= 0) & (col_f >= 0) & (row_f <= h - 1) & (col_f <= w - 1)
            if np.any(inb):
                # Degenerate axes (h==1 or w==1) have no second neighbor row/col
                # to interpolate against; fall back to nearest along that axis
                # (weight 0 on the missing neighbor) instead of relying on
                # negative-index wraparound from `np.clip(..., 0, -1)`.
                if h > 1:
                    r0c = np.clip(r0[inb], 0, h - 2)
                    r1c = r0c + 1
                    fr = row_f[inb] - r0c
                else:
                    r0c = np.zeros(np.count_nonzero(inb), dtype=int)
                    r1c = r0c
                    fr = np.zeros(np.count_nonzero(inb), dtype=np.float64)
                if w > 1:
                    c0c = np.clip(c0[inb], 0, w - 2)
                    c1c = c0c + 1
                    fc = col_f[inb] - c0c
                else:
                    c0c = np.zeros(np.count_nonzero(inb), dtype=int)
                    c1c = c0c
                    fc = np.zeros(np.count_nonzero(inb), dtype=np.float64)
                v00 = arr[r0c, c0c]
                v01 = arr[r0c, c1c]
                v10 = arr[r1c, c0c]
                v11 = arr[r1c, c1c]
                weights = (
                    (1 - fr) * (1 - fc),
                    (1 - fr) * fc,
                    fr * (1 - fc),
                    fr * fc,
                )
                values = (v00, v01, v10, v11)
                out[inb] = sum(
                    np.where(weight != 0, value * weight, 0.0)
                    for value, weight in zip(values, weights)
                )
            return out
        # Masks use nearest-neighbor sampling.
        r = np.rint(row_f).astype(int)
        c = np.rint(col_f).astype(int)
        inb = (r >= 0) & (c >= 0) & (r < h) & (c < w)
        if arr.dtype == np.uint8:
            out = np.full(np.shape(r), PSR_NODATA, dtype=np.uint8)
        else:
            out = np.zeros(np.shape(r), dtype=bool)
        if np.any(inb):
            out[inb] = arr[r[inb], c[inb]]
        return out

    def sample_static(self, key: str, x: object, y: object) -> np.ndarray:
        if key not in _STATIC_KEYS:
            _raise_invalid_product_input(
                f"unknown static key {key!r}; expected one of {_STATIC_KEYS}",
                code="ephemeris.products.sample_key",
            )
        arr = getattr(self, key)
        if arr is None:
            _raise_product_integrity(
                f"product carries no {key!r} raster",
                code="ephemeris.products.sample_missing_raster",
            )
        try:
            return self._sample(arr, x, y)
        except FailureError:
            raise
        except (IndexError, OverflowError, TypeError, ValueError) as exc:
            _raise_invalid_product_input(
                f"static sample coordinates are invalid: {exc}",
                code="ephemeris.products.sample_coordinates",
                cause=exc,
            )

    def sample_epoch(self, key: str, t_index: int, x: object, y: object) -> np.ndarray:
        if key not in ("sunlit", "sun_fraction", "earth_los"):
            _raise_invalid_product_input(
                f"unknown epoch key {key!r}",
                code="ephemeris.products.sample_key",
            )
        cube = getattr(self, key)
        if cube is None:
            _raise_product_integrity(
                f"product carries no {key!r} cube",
                code="ephemeris.products.sample_missing_raster",
            )
        if isinstance(t_index, bool):
            _raise_invalid_product_input(
                "t_index must be an integer",
                code="ephemeris.products.sample_index",
            )
        try:
            # ``operator.index`` rejects floats instead of truncating them.
            t_index = operator.index(t_index)
        except (OverflowError, TypeError, ValueError) as exc:
            _raise_invalid_product_input(
                f"t_index must be an integer: {exc}",
                code="ephemeris.products.sample_index",
                cause=exc,
            )
        if not (0 <= t_index < cube.shape[0]):
            raise ProductSamplingIndexError(
                code="ephemeris.products.sample_index",
                message=(
                    f"t_index {t_index} out of range for {key!r} with "
                    f"{cube.shape[0]} epochs"
                ),
                component=_PRODUCT_COMPONENT,
                details={"key": key, "t_index": t_index, "epoch_count": cube.shape[0]},
                retryable=False,
                fallback_allowed=False,
                evidence_valid=False,
            )
        try:
            return self._sample(cube[t_index], x, y)
        except FailureError:
            raise
        except (IndexError, OverflowError, TypeError, ValueError) as exc:
            _raise_invalid_product_input(
                f"epoch sample coordinates are invalid: {exc}",
                code="ephemeris.products.sample_coordinates",
                cause=exc,
            )


def save_product(product: IlluminationProduct, path: Path | str) -> Path:
    try:
        path = Path(path)
    except (TypeError, ValueError) as exc:
        _raise_invalid_product_input(
            f"product output path is invalid: {path!r}",
            code="ephemeris.products.output_path",
            cause=exc,
        )
    # Stamp the schema identity before validation: the reader is fail-closed
    # on a missing ``product_version``, and the saved product must
    # be exactly what the validator accepted.
    try:
        thawed_meta = _thaw_metadata(product.meta)
        if not isinstance(thawed_meta, dict):
            raise TypeError("product metadata must be an object")
        meta = thawed_meta
        meta["schema"] = SCHEMA_ID
        meta["product_version"] = PRODUCT_SCHEMA_VERSION
        product = replace(product, meta=meta)
    except (TypeError, ValueError) as exc:
        _raise_invalid_product_input(
            f"illumination product is invalid: {exc}",
            code="ephemeris.products.invalid_product",
            path=path,
            cause=exc,
        )
    try:
        product.validate()
    except FailureError:
        raise
    except (TypeError, ValueError) as exc:
        _raise_invalid_product_input(
            f"illumination product is invalid: {exc}",
            code="ephemeris.products.invalid_product",
            path=path,
            cause=exc,
        )
    try:
        identity = _metadata_dict(
            meta.get("identity"),
            error="product metadata requires versioned identity",
        )
        meta.setdefault("body", identity["body"])
        meta["grid"] = {
            "crs": product.grid.crs,
            "transform": list(product.grid.transform),
            "shape": list(product.grid.shape),
            "patch_center_xy_m": list(product.grid.patch_center_xy_m),
        }
        arrays: dict[str, np.ndarray] = {
            "meta": np.asarray(json.dumps(meta, sort_keys=True))
        }
        for key in (*_EPOCH_GROUP, *_EPOCH_OPTIONAL, *_STATIC_KEYS):
            arr = getattr(product, key)
            if arr is not None:
                arrays[key] = np.asarray(arr)
        decoded_budget = ByteBudget(
            "illumination product decoded arrays",
            MAX_ILLUMINATION_DECODED_BYTES,
        )
        for key, arr in arrays.items():
            decoded_bytes = checked_shape_nbytes(
                arr.shape,
                arr.dtype,
                resource=f"illumination product array {key!r}",
                max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            if key == "meta" and decoded_bytes > MAX_METADATA_ATTRIBUTE_BYTES:
                raise ValueError("illumination product metadata exceeds resource limit")
            decoded_budget.consume(decoded_bytes)
    except FailureError:
        raise
    except (TypeError, ValueError, OverflowError) as exc:
        _raise_invalid_product_input(
            f"illumination product cannot be serialized: {exc}",
            code="ephemeris.products.serialization",
            path=path,
            cause=exc,
        )
    publication = ArtifactPublication.file(path)

    try:
        with publication.stage() as stage:
            with stage.path.open("wb") as stream:
                # NumPy 1.x stores ``allow_pickle`` from **kwds as an archive
                # member, changing identity bytes; object arrays are rejected.
                # Cast satisfies NumPy's stub; the assertion guards key collisions.
                assert "allow_pickle" not in arrays
                savez_compressed = cast("Callable[..., None]", np.savez_compressed)
                savez_compressed(stream, **arrays)
            require_file_size(
                stage.path,
                resource="illumination product archive",
                max_bytes=MAX_ILLUMINATION_ARCHIVE_BYTES,
            )
            inspect_npz_archive(
                stage.path,
                resource="illumination product archive",
                max_archive_bytes=MAX_ILLUMINATION_ARCHIVE_BYTES,
                max_decoded_bytes=MAX_ILLUMINATION_DECODED_BYTES,
                max_array_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
            )
            expected_sha256 = sha256_file(stage.path)
            published = stage.publish(
                expected_sha256=expected_sha256,
                immutable_identity=True,
            )
    except FailureError:
        raise
    except PublicationIntegrityError as exc:
        _raise_product_integrity(
            f"illumination product publication failed integrity validation: {exc}",
            code="ephemeris.products.publication_integrity",
            path=path,
            cause=exc,
        )
    except PublicationTimeoutError as exc:
        _raise_product_io(
            f"could not publish illumination product {path}: {exc}",
            code="ephemeris.products.publication_io",
            path=path,
            cause=exc,
        )
    except OSError as exc:
        _raise_product_io(
            f"could not publish illumination product {path}: {exc}",
            code="ephemeris.products.publication_io",
            path=path,
            cause=exc,
        )
    except PublicationCleanupError as exc:
        _raise_product_io(
            f"illumination product publication cleanup is pending: {path} ({exc})",
            code="ephemeris.products.publication_io",
            path=path,
            cause=exc,
            cleanup_error=exc,
        )
    except (TypeError, ValueError, OverflowError) as exc:
        _raise_invalid_product_input(
            f"illumination product exceeds serialization limits: {exc}",
            code="ephemeris.products.serialization",
            path=path,
            cause=exc,
        )
    except PublicationError as exc:
        _raise_invalid_product_input(
            f"illumination product output path is unusable: {path} ({exc})",
            code="ephemeris.products.output_path",
            path=path,
            cause=exc,
        )
    if not isinstance(published, Path):
        raise TypeError(
            "file artifact publication returned a non-file publication result"
        )
    return published


def _json_array(value: object, *, name: str) -> tuple[object, ...]:
    if not isinstance(value, list):
        raise ValueError(f"{name} must be a JSON array")
    items: list[object] = []
    for item in value:
        items.append(item)
    return tuple(items)


def _json_float_tuple(value: object, *, name: str, size: int) -> tuple[float, ...]:
    items = _json_array(value, name=name)
    if len(items) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    return tuple(
        _finite_float(item, name=f"{name}[{index}]") for index, item in enumerate(items)
    )


def _json_transform(value: object) -> tuple[float, float, float, float, float, float]:
    values = _json_float_tuple(value, name="grid.transform", size=6)
    a, b, c, d, e, f = values
    return a, b, c, d, e, f


def _json_float_pair(value: object, *, name: str) -> tuple[float, float]:
    values = _json_float_tuple(value, name=name, size=2)
    first, second = values
    return first, second


def _json_int_pair(value: object, *, name: str) -> tuple[int, int]:
    items = _json_array(value, name=name)
    if len(items) != 2:
        raise ValueError(f"{name} must contain exactly 2 values")
    values: list[int] = []
    for index, item in enumerate(items):
        # JSON integers only; int() would truncate 3.5 or parse "3".
        if isinstance(item, bool) or not isinstance(item, int):
            raise ValueError(f"{name}[{index}] must be an integer")
        values.append(item)
    return values[0], values[1]


def _reserve_product_load_materialization(
    declarations: Mapping[str, object],
) -> None:
    """Reserve NPZ payload, dtype conversion, and value-object snapshots."""
    budget = ByteBudget(
        "illumination product load allocations", MAX_ILLUMINATION_DECODED_BYTES
    )
    for key, declaration in declarations.items():
        if key == "allow_pickle":
            continue
        decoded_bytes = int(getattr(declaration, "decoded_bytes"))
        shape = tuple(getattr(declaration, "shape"))
        dtype = np.dtype(getattr(declaration, "dtype"))
        budget.consume(decoded_bytes)
        # Masks are stored as bool; casting other dtypes would turn NaN or
        # arbitrary values into ``True`` before validation sees them.
        if key in ("sunlit", "earth_los") and dtype != np.dtype(bool):
            raise ValueError(f"{key} must have boolean dtype, got {dtype}")
        snapshot_bytes = checked_shape_nbytes(
            shape,
            dtype,
            resource=f"illumination product {key!r} snapshot",
            max_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        budget.consume(snapshot_bytes)


def _load_product_snapshot(source: BinaryIO, *, path: Path) -> IlluminationProduct:
    """Parse one already-verified descriptor without whole-file bytes."""

    try:
        declarations = inspect_npz_archive(
            source,
            resource="illumination product archive",
            max_archive_bytes=MAX_ILLUMINATION_ARCHIVE_BYTES,
            max_decoded_bytes=MAX_ILLUMINATION_DECODED_BYTES,
            max_array_bytes=MAX_ILLUMINATION_ARRAY_BYTES,
        )
        # Older NumPy 1.x archives may contain this unintended member.
        allowed_keys = {"meta", "allow_pickle", *_PRODUCT_ARRAY_KEYS}
        unexpected_keys = set(declarations) - allowed_keys
        if unexpected_keys:
            raise ValueError(f"unexpected product arrays: {sorted(unexpected_keys)}")
        if "meta" not in declarations:
            raise ValueError(f"{path}: not an srb_illum product (no meta key)")
        if declarations["meta"].decoded_bytes > MAX_METADATA_ATTRIBUTE_BYTES:
            raise ValueError("product metadata exceeds its resource limit")
        _reserve_product_load_materialization(declarations)

        with np.load(source, allow_pickle=False) as data:
            meta = _metadata_dict(
                json.loads(str(data["meta"])),
                error="product metadata must be an object",
            )
            if meta.get("schema") != SCHEMA_ID:
                if meta.get("schema") == LEGACY_SCHEMA_ID:
                    raise ValueError(
                        f"legacy schema {meta['schema']!r} requires explicit migration"
                    )
                raise ValueError(
                    f"{path}: unsupported schema {meta.get('schema')!r} "
                    f"(expected {SCHEMA_ID})"
                )
            g = _metadata_dict(meta["grid"], error="product grid must be an object")
            grid = ProductGrid(
                crs=str(g["crs"]),
                transform=_json_transform(g["transform"]),
                shape=_json_int_pair(g["shape"], name="grid.shape"),
                patch_center_xy_m=_json_float_pair(
                    g["patch_center_xy_m"], name="grid.patch_center_xy_m"
                ),
            )
            kwargs: dict[str, np.ndarray] = {}
            for key in _PRODUCT_ARRAY_KEYS:
                if key in data:
                    kwargs[key] = np.asarray(data[key])
        product = IlluminationProduct(grid=grid, meta=meta, **kwargs)
        product.validate()
        return product
    except FailureError:
        raise
    except OSError as exc:
        _raise_product_io(
            f"could not decode illumination product {path}: {exc}",
            code="ephemeris.products.parse_io",
            path=path,
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
        _raise_product_integrity(
            f"not a readable srb_illum product: {path} ({exc})",
            path=path,
            cause=exc,
        )


def load_product(path: Path | str) -> IlluminationProduct:
    try:
        path = Path(path)
    except (TypeError, ValueError) as exc:
        _raise_invalid_product_input(
            f"illumination product path is invalid: {path!r}",
            code="ephemeris.products.input_path",
            cause=exc,
        )

    try:
        with ArtifactPublication.file(path).open_snapshot(
            max_bytes=MAX_ILLUMINATION_ARCHIVE_BYTES
        ) as snapshot:
            if snapshot is None:
                _raise_invalid_product_input(
                    f"illumination product file does not exist: {path}",
                    code="ephemeris.products.not_found",
                    path=path,
                )
            product = _load_product_snapshot(snapshot.file, path=path)
    except FailureError:
        raise
    except PublicationTimeoutError as exc:
        _raise_product_io(
            f"could not read illumination product {path}: {exc}",
            code="ephemeris.products.io",
            path=path,
            cause=exc,
        )
    except OSError as exc:
        _raise_product_io(
            f"could not read illumination product {path}: {exc}",
            path=path,
            cause=exc,
        )
    except PublicationIntegrityError as exc:
        _raise_product_integrity(
            f"illumination product publication failed integrity validation: {exc}",
            code="ephemeris.products.publication_integrity",
            path=path,
            cause=exc,
        )
    except PublicationError as exc:
        if path.is_symlink() or (path.exists() and not path.is_file()):
            _raise_invalid_product_input(
                f"illumination product path is not a regular file: {path} ({exc})",
                code="ephemeris.products.input_path",
                path=path,
                cause=exc,
            )
        _raise_product_integrity(
            f"illumination product publication is invalid: {path} ({exc})",
            code="ephemeris.products.publication_integrity",
            path=path,
            cause=exc,
        )
    return product


def sunlit_epoch_windows(
    product: IlluminationProduct,
    *,
    min_lit_fraction: float,
    xy_local: Optional[tuple[float, float]] = None,
    min_duration_s: float = 0.0,
) -> tuple[tuple[str, str], ...]:
    """Return inclusive UTC windows for consecutive epochs meeting the threshold.

    Point mode compares solar-disk fraction; grid mode compares the fraction
    of valid cells marked sunlit. NaNs fail point mode and are excluded in grid mode.
    """
    if not isinstance(product, IlluminationProduct):
        _raise_invalid_product_input(
            "product must be an IlluminationProduct",
            code="ephemeris.products.window_product",
        )
    if isinstance(min_lit_fraction, bool):
        _raise_invalid_product_input(
            "min_lit_fraction must be a finite number in [0, 1]",
            code="ephemeris.products.window_threshold",
        )
    try:
        threshold = _finite_float(min_lit_fraction, name="min_lit_fraction")
    except (OverflowError, TypeError, ValueError) as exc:
        _raise_invalid_product_input(
            f"min_lit_fraction is invalid: {exc}",
            code="ephemeris.products.window_threshold",
            cause=exc,
        )
    if not 0.0 <= threshold <= 1.0:
        _raise_invalid_product_input(
            f"min_lit_fraction {threshold} outside [0, 1]",
            code="ephemeris.products.window_threshold",
        )
    if isinstance(min_duration_s, bool):
        _raise_invalid_product_input(
            "min_duration_s must be a finite non-negative number",
            code="ephemeris.products.window_duration",
        )
    try:
        duration = _finite_float(min_duration_s, name="min_duration_s")
    except (OverflowError, TypeError, ValueError) as exc:
        _raise_invalid_product_input(
            f"min_duration_s is invalid: {exc}",
            code="ephemeris.products.window_duration",
            cause=exc,
        )
    if duration < 0.0:
        _raise_invalid_product_input(
            "min_duration_s must be non-negative",
            code="ephemeris.products.window_duration",
        )
    point: tuple[float, float] | None = None
    if xy_local is not None:
        try:
            x_value, y_value = xy_local
            if isinstance(x_value, bool) or isinstance(y_value, bool):
                raise TypeError("point coordinates must be numeric")
            point = (
                _finite_float(x_value, name="xy_local.x"),
                _finite_float(y_value, name="xy_local.y"),
            )
        except (IndexError, OverflowError, TypeError, ValueError) as exc:
            _raise_invalid_product_input(
                f"xy_local is invalid: {exc}",
                code="ephemeris.products.window_point",
                cause=exc,
            )

    product.validate()
    if not product.has_epochs:
        _raise_product_integrity(
            "product has no epoch tier",
            code="ephemeris.products.window_missing_epochs",
        )
    sunlit = product.sunlit
    sun_fraction = product.sun_fraction
    times_utc = product.times_utc
    if sunlit is None or sun_fraction is None or times_utc is None:
        raise IlluminationProductValidationError(
            "epoch tier is all-or-none; required raster is missing"
        )
    t = product.epoch_count
    ok = np.zeros(t, dtype=bool)
    if point is not None:
        x = np.array([point[0]])
        y = np.array([point[1]])
        for i in range(t):
            v = product.sample_epoch("sun_fraction", i, x, y)[0]
            ok[i] = bool(np.isfinite(v) and v >= threshold)
    else:
        valid = np.isfinite(sun_fraction)
        for i in range(t):
            n_valid = int(valid[i].sum())
            if n_valid == 0:
                continue
            lit = int((sunlit[i] & valid[i]).sum())
            ok[i] = (lit / n_valid) >= threshold
    windows: list[tuple[str, str]] = []
    start = None
    for i in range(t):
        if ok[i] and start is None:
            start = i
        if start is not None and (not ok[i] or i == t - 1):
            end = i if ok[i] else i - 1
            t_start = _parse_utc(str(times_utc[start]))
            t_end = _parse_utc(str(times_utc[end]))
            if (t_end - t_start).total_seconds() >= duration:
                windows.append((str(times_utc[start]), str(times_utc[end])))
            start = None
    return tuple(windows)
