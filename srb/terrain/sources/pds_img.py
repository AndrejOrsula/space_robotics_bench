"""Convert PDS3 `.IMG` files to GeoTIFF.

Post-fetch adapter, not a registry provider. Tries GDAL, then lazily imports
optional `pdr` from `terrain-extras`; raises `MissingTerrainDependencyError`
if neither is available.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import NoReturn

import numpy as np
import rasterio
import rasterio.errors
from rasterio.transform import Affine

from srb.terrain.dependencies import missing_img_dependencies
from srb.terrain.errors import DemLoadError, MissingTerrainDependencyError
from srb.terrain.source_health import (
    make_terrain_source_health_event,
    publish_terrain_source_health,
)
from srb.utils.failures import (
    FailureCategory,
    FailureError,
    FailureInfo,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
    IntegrityFailureError,
    InternalInvariantError,
)
from srb.utils.publication import ArtifactPublication
from srb.utils.resource_budget import (
    ResourceLimitError,
    checked_shape_nbytes,
    require_file_size,
)

_PDS_DRIVER_NAMES = frozenset({"PDS", "PDS4"})
_PDS_LABEL_PROBE_BYTES = 64 * 1024
_UNSUPPORTED_DECODER_MARKERS = (
    "not recognized as a supported file format",
    "not recognized as a supported data format",
    "unsupported file format",
    "unsupported pds",
    "pds driver does not support",
)


def pdr_available() -> bool:
    """Is the optional `pdr` adapter importable?

    Uses `importlib.util.find_spec` (via `missing_img_dependencies`) --
    does not import `pdr`.
    """
    return not missing_img_dependencies()


def gdal_pds_driver_available() -> bool:
    """Does the linked GDAL expose the PDS/PDS4 driver?

    Enumerates registered drivers via `rasterio.Env().drivers()` -- no
    file needed.
    """
    with rasterio.Env() as env:
        registered = env.drivers()
    return not _PDS_DRIVER_NAMES.isdisjoint(registered)


def needs_img_conversion(path: Path) -> bool:
    """True when `path`'s suffix is `.img`/`.IMG` (the legacy PDS3 format)."""
    return path.suffix.lower() == ".img"


def img_to_geotiff(
    src: Path,
    dest: Path,
    *,
    band: int = 0,
    health: Callable[[HealthEvent], None] | None = None,
    on_health: Callable[[HealthEvent], None] | None = None,
) -> Path:
    """Convert a PDS3 `.IMG` file at `src` to a single-band float32 GeoTIFF
    at `dest`, readable by `srb.terrain.ingest.dem.load_dem` /
    `load_dem_window`.

    Tries GDAL first. It falls back to `pdr` only when a recognizable PDS label
    meets an unsupported-decoder error; missing, unreadable, or corrupt input
    remains an integrity failure. A `pdr` fallback emits a degraded
    `HealthEvent` to the source-health seam and optional ``health``/``on_health``
    observers.

    Raises `MissingTerrainDependencyError` if neither decoder is available.
    Publishes through an API-owned stage to avoid exposing partial output.
    """
    if health is not None and on_health is not None:
        raise TypeError("pass either health or on_health, not both")
    health_sink = health if health is not None else on_health

    if gdal_pds_driver_available():
        try:
            return _convert_via_gdal(src, dest, band=band)
        except FailureError as exc:
            _publish_failure_health(exc, src=src, health=health_sink)
            raise
        except DemLoadError as exc:
            _publish_terrain_error_health(exc, src=src, health=health_sink)
            raise
        except ResourceLimitError:
            raise
        except rasterio.errors.RasterioIOError as exc:
            if not _is_unsupported_local_decoder_error(src, exc):
                failure = _pds_integrity_failure(src, exc, stage="open")
                _publish_failure_health(failure, src=src, health=health_sink)
                raise failure from exc
            return _convert_with_pdr_substitute(
                src,
                dest,
                band=band,
                gdal_cause=exc,
                health=health_sink,
            )
        except Exception as exc:  # noqa: BLE001 - PDS decoder boundary
            failure = _pds_internal_failure(src, exc, stage="gdal")
            _publish_failure_health(failure, src=src, health=health_sink)
            raise failure from exc

    return _convert_with_pdr_substitute(
        src,
        dest,
        band=band,
        gdal_cause=None,
        health=health_sink,
    )


def _convert_via_gdal(src: Path, dest: Path, *, band: int) -> Path:
    # Keep the open failure at the caller's decoder-selection seam. Once a
    # dataset has opened, a RasterioIOError from reading pixels is a source
    # integrity failure, never evidence that another decoder should be tried.
    try:
        require_file_size(src, resource=f"PDS IMG source {src}")
    except ResourceLimitError:
        raise
    except (OSError, ValueError) as exc:
        raise _pds_integrity_failure(src, exc, stage="size") from exc
    with rasterio.open(src) as ds:
        if not (0 <= band < ds.count):
            raise DemLoadError(
                f"band {band} out of range for {src} with {ds.count} band(s)"
            )
        checked_shape_nbytes(
            (ds.height, ds.width),
            np.dtype(ds.dtypes[band]),
            resource=f"PDS IMG decoded band {src}",
        )
        try:
            array = ds.read(band + 1)
        except rasterio.errors.RasterioIOError as exc:
            raise _pds_integrity_failure(src, exc, stage="read") from exc
        crs = str(ds.crs) if ds.crs else None
        transform = ds.transform
        nodata = ds.nodatavals[band] if ds.nodatavals else None

    return _write_geotiff(array, dest, crs=crs, transform=transform, nodata=nodata)


def _convert_with_pdr_substitute(
    src: Path,
    dest: Path,
    *,
    band: int,
    gdal_cause: rasterio.errors.RasterioIOError | None,
    health: Callable[[HealthEvent], None] | None,
) -> Path:
    """Use pdr only as an explicit local decoder substitution."""
    if not pdr_available():
        _raise_missing_pdr(src, health=health)

    try:
        result = _convert_via_pdr(src, dest, band=band)
    except FailureError as exc:
        _publish_failure_health(exc, src=src, health=health)
        raise
    except DemLoadError as exc:
        _publish_terrain_error_health(exc, src=src, health=health)
        raise
    except ImportError as exc:
        # The availability probe and the lazy import are intentionally both
        # guarded: a package can disappear between them in a live process.
        _raise_missing_pdr(src, health=health, cause=exc)
    except ResourceLimitError:
        raise
    except Exception as exc:  # noqa: BLE001 - optional decoder boundary
        failure = (
            _pds_integrity_failure(src, exc, stage="pdr")
            if isinstance(exc, (OSError, ValueError, EOFError))
            else _pds_internal_failure(src, exc, stage="pdr")
        )
        _publish_failure_health(failure, src=src, health=health)
        raise failure from exc

    failure = FailureInfo(
        category=FailureCategory.UNSUPPORTED_CAPABILITY,
        code="terrain.pds_img.gdal_decoder_unsupported",
        message=(
            f"GDAL could not decode PDS IMG {src}; decoded with the local pdr "
            "substitute"
        ),
        component="terrain.sources.pds_img",
        details={
            "source": str(src),
            "primary_decoder": "gdal",
            "substitute_decoder": "pdr",
            "primary_error": None if gdal_cause is None else str(gdal_cause),
            "primary_error_type": (
                None if gdal_cause is None else type(gdal_cause).__name__
            ),
        },
        retryable=False,
        fallback_allowed=True,
        evidence_valid=False,
    )
    event = make_terrain_source_health_event(
        failure,
        component="terrain.sources.pds_img",
        code="terrain.pds_img.pdr_fallback",
        message=(
            f"PDS IMG decoded with local pdr substitute after GDAL decoder "
            f"unavailability: {src}"
        ),
        details={
            "source": str(src),
            "primary_decoder": "gdal",
            "substitute_decoder": "pdr",
            "fallback_applied": True,
        },
    )
    publish_terrain_source_health(event, health=health)
    return result


def _raise_missing_pdr(
    src: Path,
    *,
    health: Callable[[HealthEvent], None] | None,
    cause: ImportError | None = None,
) -> NoReturn:
    """Publish the unavailable-decoder failure before failing the conversion."""
    error = MissingTerrainDependencyError(("pdr",), extra="terrain-extras")
    from srb.terrain.source_health import classify_terrain_source_exception

    failure = classify_terrain_source_exception(error).failure_info
    details: dict[str, object] = {
        "source": str(src),
        "dependency": "pdr",
        "fallback_applied": False,
    }
    if cause is not None:
        details["cause_type"] = f"{type(cause).__module__}.{type(cause).__qualname__}"
    event = HealthEvent(
        status=HealthStatus.UNHEALTHY,
        severity=HealthSeverity.ERROR,
        component="terrain.sources.pds_img",
        code=failure.code,
        message=failure.message,
        details=details,
        failure=failure,
    )
    publish_terrain_source_health(event, health=health)
    if cause is None:
        raise error
    raise error from cause


def _is_unsupported_local_decoder_error(
    src: Path, exc: rasterio.errors.RasterioIOError
) -> bool:
    """Recognize an unsupported GDAL dialect, not a bad or unreadable file."""
    message = str(exc).lower()
    if not any(marker in message for marker in _UNSUPPORTED_DECODER_MARKERS):
        return False
    try:
        with src.open("rb") as handle:
            label = handle.read(_PDS_LABEL_PROBE_BYTES)
    except OSError:
        return False
    text = label.decode("ascii", errors="ignore").upper()
    return "PDS_VERSION_ID" in text and "^IMAGE" in text and "END" in text


def _pds_integrity_failure(
    src: Path, cause: BaseException, *, stage: str
) -> IntegrityFailureError:
    """Create a typed, source-causal failure for bad PDS bytes/I/O."""
    failure = FailureInfo(
        category=FailureCategory.INTEGRITY_FAILURE,
        code="terrain.pds_img.decode_failed",
        message=f"PDS IMG {stage} failed for {src}: {cause}",
        component="terrain.sources.pds_img",
        details={
            "source": str(src),
            "stage": stage,
            "cause_type": f"{type(cause).__module__}.{type(cause).__qualname__}",
            "cause_message": str(cause) or type(cause).__name__,
        },
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    return IntegrityFailureError(failure)


def _pds_internal_failure(
    src: Path, cause: BaseException, *, stage: str
) -> InternalInvariantError:
    """Create a typed failure for an unexpected adapter invariant."""
    failure = FailureInfo(
        category=FailureCategory.INTERNAL_INVARIANT,
        code="terrain.pds_img.decoder_invariant",
        message=f"PDS IMG {stage} adapter failed for {src}: {cause}",
        component="terrain.sources.pds_img",
        details={
            "source": str(src),
            "stage": stage,
            "cause_type": f"{type(cause).__module__}.{type(cause).__qualname__}",
            "cause_message": str(cause) or type(cause).__name__,
        },
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    return InternalInvariantError(failure)


def _publish_failure_health(
    exc: FailureError,
    *,
    src: Path,
    health: Callable[[HealthEvent], None] | None,
) -> None:
    event = make_terrain_source_health_event(
        exc.failure_info,
        component="terrain.sources.pds_img",
        code=exc.failure_info.code,
        message=exc.failure_info.message,
        details={"source": str(src), "fallback_applied": False},
    )
    publish_terrain_source_health(event, health=health)


def _publish_terrain_error_health(
    exc: DemLoadError,
    *,
    src: Path,
    health: Callable[[HealthEvent], None] | None,
) -> None:
    from srb.terrain.source_health import classify_terrain_source_exception

    failure = classify_terrain_source_exception(exc).failure_info
    event = make_terrain_source_health_event(
        failure,
        component="terrain.sources.pds_img",
        code=failure.code,
        message=failure.message,
        details={"source": str(src), "fallback_applied": False},
    )
    publish_terrain_source_health(event, health=health)


def _convert_via_pdr(src: Path, dest: Path, *, band: int) -> Path:
    import pdr  # optional dependency (terrain-extras); imported lazily.

    try:
        require_file_size(src, resource=f"PDS IMG source {src}")
    except ResourceLimitError:
        raise
    except (OSError, ValueError) as exc:
        raise _pds_integrity_failure(src, exc, stage="size") from exc
    data = pdr.read(str(src))
    keys = list(data.keys())
    if not keys:
        raise DemLoadError(f"pdr found no data objects in {src}")
    # Prefer the canonical PDS3 image-object name when present; otherwise
    # take whatever pdr identified first.
    name = "IMAGE" if "IMAGE" in keys else keys[0]
    value = data[name]
    declared_shape = getattr(value, "shape", None)
    declared_dtype = getattr(value, "dtype", None)
    if declared_shape is not None and declared_dtype is not None:
        checked_shape_nbytes(
            declared_shape,
            np.dtype(declared_dtype),
            resource=f"PDS IMG pdr object {src}",
        )
    array = np.asarray(value)
    checked_shape_nbytes(
        array.shape,
        array.dtype,
        resource=f"PDS IMG pdr decoded object {src}",
    )

    if array.ndim == 3:
        if not (0 <= band < array.shape[0]):
            raise DemLoadError(
                f"band {band} out of range for {src} with {array.shape[0]} "
                "band(s) (decoded via pdr)"
            )
        array = array[band]
    elif band != 0:
        raise DemLoadError(
            f"band {band} requested but {src} decoded as a single 2D array via pdr"
        )

    return _write_geotiff(array, dest, crs=None, transform=None, nodata=None)


def _write_geotiff(
    array: np.ndarray,
    dest: Path,
    *,
    crs: str | None,
    transform: Affine | None,
    nodata: float | None,
) -> Path:
    """Persist `array` as a single-band float32 GeoTIFF through the shared
    publication seam so a concurrent reader never observes a
    partially-written file."""
    if array.ndim != 2:
        raise DemLoadError(f"PDS IMG decoded array must be 2-D, got {array.shape}")
    checked_shape_nbytes(
        array.shape,
        array.dtype,
        resource="PDS IMG decoded output",
    )
    checked_shape_nbytes(
        array.shape,
        np.dtype(np.float32),
        resource="PDS IMG float32 output",
    )
    array = array.astype(np.float32, copy=False)

    def writer(staging: Path) -> None:
        with rasterio.open(
            staging,
            "w",
            driver="GTiff",
            height=array.shape[0],
            width=array.shape[1],
            count=1,
            dtype=np.float32,
            crs=crs,
            transform=transform if transform is not None else Affine.identity(),
            nodata=nodata,
        ) as dst:
            dst.write(array, 1)

    ArtifactPublication.file(dest).publish(writer)
    return dest
