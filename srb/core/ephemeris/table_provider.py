"""Build ephemeris states from srb_illum/2 products without SPICE.

An explicit set_table_product(path) overrides the environment variable.
"""

from __future__ import annotations

import math
import os
from datetime import datetime
from pathlib import Path
from typing import NoReturn, Optional

from srb.core.ephemeris import geometry
from srb.core.ephemeris.config import SiteSpec
from srb.core.ephemeris.products import (
    IlluminationProduct,
    _parse_utc,
    approx_et_seconds,
    load_product,
)
from srb.core.ephemeris.provider import (
    EphemerisProvider,
    register_provider,
    validate_query_bodies,
    validate_query_epoch,
)
from srb.core.ephemeris.state import BodyState, EphemerisState
from srb.utils.failures import (
    FailureError,
    IntegrityFailureError,
    InvalidInputError,
    TransientExternalFailureError,
    UnsupportedCapabilityError,
)

_AU_M = 149_597_870_700.0
_SUN_RADIUS_M = 695_700_000.0
_EARTH_RADIUS_M = 6_371_000.0
_EARTH_MEAN_DISTANCE_M = 3.844e8

_PRODUCT_PATH: Optional[Path] = None
# Cached snapshots come from load_product; public product values use validated
# methods on each call.
_CACHE: dict[tuple[str, int, int, int, int, int], IlluminationProduct] = {}
_PARSED_TIMES: dict[int, tuple[datetime, ...]] = {}
_TABLE_COMPONENT = "ephemeris.table_provider"


class _TableCapabilityError(UnsupportedCapabilityError, ValueError):
    """Surface-only table capability with the proven ValueError base."""


def _failure_details(
    *, path: Path | None = None, cause: BaseException | None = None, **items: object
) -> dict[str, object]:
    details = dict(items)
    if path is not None:
        details["path"] = str(path)
    if cause is not None:
        details["cause_type"] = f"{type(cause).__module__}.{type(cause).__qualname__}"
    return details


def _raise_invalid(
    message: str,
    *,
    code: str,
    path: Path | None = None,
    cause: BaseException | None = None,
    **details: object,
) -> NoReturn:
    error = InvalidInputError(
        code=code,
        message=message,
        component=_TABLE_COMPONENT,
        details=_failure_details(path=path, cause=cause, **details),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


def _raise_unsupported(
    message: str,
    *,
    code: str,
    path: Path | None = None,
    fallback_allowed: bool = False,
    **details: object,
) -> NoReturn:
    raise _TableCapabilityError(
        code=code,
        message=message,
        component=_TABLE_COMPONENT,
        details=_failure_details(path=path, **details),
        retryable=False,
        fallback_allowed=fallback_allowed,
        evidence_valid=False,
    )


def _raise_transient(
    message: str,
    *,
    code: str,
    path: Path,
    cause: BaseException,
    **details: object,
) -> NoReturn:
    error = TransientExternalFailureError(
        code=code,
        message=message,
        component=_TABLE_COMPONENT,
        details=_failure_details(path=path, cause=cause, **details),
        retryable=True,
        fallback_allowed=True,
        evidence_valid=False,
    )
    raise error from cause


def _raise_integrity(
    message: str,
    *,
    code: str,
    path: Path,
    cause: BaseException,
    **details: object,
) -> NoReturn:
    error = IntegrityFailureError(
        code=code,
        message=message,
        component=_TABLE_COMPONENT,
        details=_failure_details(path=path, cause=cause, **details),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )
    raise error from cause


def set_table_product(path) -> None:
    """Explicitly select the product file for the "table" provider."""
    global _PRODUCT_PATH
    if path is None:
        _PRODUCT_PATH = None
        return
    try:
        _PRODUCT_PATH = Path(path)
    except (TypeError, ValueError) as exc:
        _raise_invalid(
            f"table provider product path is invalid: {path!r}",
            code="ephemeris.table.config.product_path",
            cause=exc,
            configuration="set_table_product",
        )


def _resolve_product_path() -> Path:
    if _PRODUCT_PATH is not None:
        return _PRODUCT_PATH
    env = os.environ.get("SRB_EPHEMERIS_TABLE_PRODUCT")
    if env:
        try:
            return Path(env)
        except (TypeError, ValueError) as exc:
            _raise_invalid(
                "SRB_EPHEMERIS_TABLE_PRODUCT is not a valid product path",
                code="ephemeris.table.config.product_path",
                cause=exc,
                configuration="SRB_EPHEMERIS_TABLE_PRODUCT",
            )
    _raise_invalid(
        'the "table" ephemeris provider needs a product file: call '
        "srb.core.ephemeris.table_provider.set_table_product(path) or set "
        "the SRB_EPHEMERIS_TABLE_PRODUCT environment variable",
        code="ephemeris.table.config.product_path",
        configuration="set_table_product_or_environment",
    )


def _load(path: Path) -> IlluminationProduct:
    try:
        identity = path.stat()
    except OSError as exc:
        _raise_transient(
            f"could not stat table illumination product {path}: {exc}",
            code="ephemeris.table.path.stat",
            path=path,
            cause=exc,
            operation="stat",
        )
    except (TypeError, ValueError) as exc:
        _raise_invalid(
            f"table illumination product path cannot be stat'ed: {path}",
            code="ephemeris.table.path.stat",
            path=path,
            cause=exc,
            operation="stat",
        )
    key = (
        str(path),
        identity.st_dev,
        identity.st_ino,
        identity.st_size,
        identity.st_mtime_ns,
        identity.st_ctime_ns,
    )
    product = _CACHE.get(key)
    if product is None:
        try:
            product = load_product(path)
        except FailureError:
            raise
        except OSError as exc:
            _raise_transient(
                f"could not read table illumination product {path}: {exc}",
                code="ephemeris.table.product_io",
                path=path,
                cause=exc,
                operation="load",
            )
        except (IndexError, KeyError, TypeError, ValueError, OverflowError) as exc:
            _raise_integrity(
                f"table illumination product is invalid: {path} ({exc})",
                code="ephemeris.table.product_integrity",
                path=path,
                cause=exc,
                operation="load",
            )
        if not product.has_epochs:
            _raise_unsupported(
                f"{path}: product has no epoch tier; the table provider needs "
                "per-epoch sun az/el (static-only products serve spawn/reward "
                "consumers, not ephemeris queries)",
                code="ephemeris.table.capability.epoch_tier",
                path=path,
                required="epoch_tier",
            )
        _CACHE.clear()  # keep at most one product resident
        _PARSED_TIMES.clear()
        _CACHE[key] = product
        if product.times_utc is not None:
            _PARSED_TIMES[id(product)] = tuple(
                _parse_utc(str(value)) for value in product.times_utc
            )
    return product


def _validate_query_site(site: SiteSpec) -> None:
    if not isinstance(site, SiteSpec):
        _raise_invalid(
            "table provider query site must be a SiteSpec",
            code="ephemeris.table.query.site",
        )
    try:
        site.validate()
    except (TypeError, ValueError, OverflowError) as exc:
        _raise_invalid(
            f"table provider query site is invalid: {exc}",
            code="ephemeris.table.query.site",
            cause=exc,
        )
    if not site.is_surface:
        _raise_unsupported(
            'the "table" provider serves surface sites only (lat/lon set); '
            "orbit-domain ephemeris needs the spice provider",
            code="ephemeris.table.capability.surface_only",
            capability="surface_site_queries",
        )
    for name, value in (
        ("lat_deg", site.lat_deg),
        ("lon_deg", site.lon_deg),
        ("alt_m", site.alt_m),
        ("north_yaw_deg", site.north_yaw_deg),
    ):
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            _raise_invalid(
                f"table provider query site {name} is invalid",
                code="ephemeris.table.query.site",
                cause=exc,
                field=name,
            )
        if not math.isfinite(number):
            _raise_invalid(
                f"table provider query site {name} must be finite",
                code="ephemeris.table.query.site",
                field=name,
            )
    if not -180.0 <= float(site.lon_deg) <= 180.0:
        _raise_invalid(
            f"table provider query longitude {site.lon_deg} outside [-180, 180]",
            code="ephemeris.table.query.site",
            field="lon_deg",
        )


def _validate_table_body_availability(
    requested_bodies: tuple[str, ...],
    product: IlluminationProduct,
    path: Path,
) -> None:
    available = {"sun"}
    if product.earth_azel is not None:
        available.add("earth")
    missing = tuple(body for body in requested_bodies if body not in available)
    if missing:
        _raise_unsupported(
            "table illumination product cannot answer requested supported body "
            f"{missing[0]!r}; available body tiers: {tuple(sorted(available))}",
            code="ephemeris.table.query.body_unavailable",
            path=path,
            fallback_allowed=True,
            bodies=missing,
            available=tuple(sorted(available)),
        )


@register_provider("table")
class TableProvider(EphemerisProvider):
    def query(
        self,
        site: SiteSpec,
        epoch_utc: str,
        bodies: tuple[str, ...] = ("sun",),
        *,
        optional_bodies: tuple[str, ...] = (),
    ) -> EphemerisState:
        _validate_query_site(site)
        try:
            epoch_utc = validate_query_epoch(epoch_utc)
            requested_bodies = validate_query_bodies(bodies)
            optional_bodies = validate_query_bodies(optional_bodies)
            if set(requested_bodies) & set(optional_bodies):
                raise ValueError("optional bodies overlap required query bodies")
        except FailureError:
            raise
        except (TypeError, ValueError, OverflowError) as exc:
            _raise_invalid(
                f"table provider query input is invalid: {exc}",
                code="ephemeris.table.query.input",
                cause=exc,
            )
        path = _resolve_product_path()
        product = _load(path)
        _validate_table_body_availability(requested_bodies, product, path)
        product._validate_site_snapshot(site)
        idx = product._epoch_index_snapshot(
            epoch_utc,
            parsed_times=_PARSED_TIMES.get(id(product)),
        )
        try:
            out: dict[str, BodyState] = {}
            az, el = (float(v) for v in product.sun_azel[idx])
            ang = float(product.sun_angular_diameter_deg[idx])
            dist = _SUN_RADIUS_M / math.sin(math.radians(ang / 2.0))
            out["sun"] = BodyState(
                azimuth_deg=az,
                elevation_deg=el,
                direction_world=tuple(
                    geometry.direction_world(az, el, site.north_yaw_deg)
                ),
                distance_m=dist,
                angular_diameter_deg=ang,
            )
            if (
                "earth" in (*requested_bodies, *optional_bodies)
                and product.earth_azel is not None
            ):
                eaz, eel = (float(v) for v in product.earth_azel[idx])
                edist = _EARTH_MEAN_DISTANCE_M
                out["earth"] = BodyState(
                    azimuth_deg=eaz,
                    elevation_deg=eel,
                    direction_world=tuple(
                        geometry.direction_world(eaz, eel, site.north_yaw_deg)
                    ),
                    distance_m=edist,
                    angular_diameter_deg=2.0
                    * math.degrees(math.asin(_EARTH_RADIUS_M / edist)),
                )
        except FailureError:
            raise
        except (
            AttributeError,
            IndexError,
            KeyError,
            OverflowError,
            TypeError,
            ValueError,
        ) as exc:
            _raise_integrity(
                f"table provider query data is invalid for {path}: {exc}",
                code="ephemeris.table.query.integrity",
                path=path,
                cause=exc,
                operation="query",
            )
        return EphemerisState(
            epoch_utc=epoch_utc,
            epoch_et=approx_et_seconds(epoch_utc),
            site=site,
            bodies=out,
            irradiance_scale=(_AU_M / dist) ** 2,
            provenance={
                "provider": "table",
                "product": str(path),
                "source": str(product.meta.get("source", "")),
            },
        )
