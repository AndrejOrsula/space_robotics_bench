"""Ephemeris provider interface and registry."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable

from srb.core.ephemeris.config import (
    SUPPORTED_BODIES,
    SiteSpec,
    _parse_epoch,
    _validate_body_sequence,
)
from srb.core.ephemeris.state import EphemerisState
from srb.utils.failures import (
    FailureError,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
    InvalidInputError,
    OptionalDependencyUnavailableError,
    classify_exception,
)

_REGISTRY: dict[str, type["EphemerisProvider"]] = {}
_PROVIDER_COMPONENT = "ephemeris.provider"
HealthObserver = Callable[[HealthEvent], None]


def _query_invalid(
    message: str, *, code: str, details: dict[str, object] | None = None
) -> InvalidInputError:
    return InvalidInputError(
        code=code,
        message=message,
        component=_PROVIDER_COMPONENT,
        details=details or {},
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )


def validate_query_site(site: SiteSpec) -> SiteSpec:
    """Validate the common site contract before provider work or imports."""
    if not isinstance(site, SiteSpec):
        raise _query_invalid(
            "ephemeris query site must be a SiteSpec",
            code="ephemeris.provider.query.site",
            details={"site_type": type(site).__name__},
        )
    try:
        site.validate()
    except (TypeError, ValueError, OverflowError) as exc:
        raise _query_invalid(
            f"ephemeris query site is invalid: {exc}",
            code="ephemeris.provider.query.site",
            details={"cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}"},
        ) from exc
    return site


def validate_query_epoch(epoch_utc: str) -> str:
    """Validate the shared ISO epoch and de440s coverage contract."""
    if not isinstance(epoch_utc, str):
        raise _query_invalid(
            "ephemeris query epoch must be an ISO-8601 string",
            code="ephemeris.provider.query.epoch",
            details={"epoch_type": type(epoch_utc).__name__},
        )
    try:
        _parse_epoch(epoch_utc)
    except (TypeError, ValueError, OverflowError) as exc:
        raise _query_invalid(
            f"ephemeris query epoch is invalid: {exc}",
            code="ephemeris.provider.query.epoch",
            details={"cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}"},
        ) from exc
    return epoch_utc


def validate_query_bodies(bodies: object) -> tuple[str, ...]:
    """Validate the explicit body vocabulary without silently deduplicating."""
    try:
        return _validate_body_sequence(bodies, context="ephemeris query bodies")
    except (TypeError, ValueError, OverflowError) as exc:
        raise _query_invalid(
            f"ephemeris query bodies are invalid: {exc}; "
            f"supported bodies: {SUPPORTED_BODIES}",
            code="ephemeris.provider.query.bodies",
            details={
                "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
                "supported": SUPPORTED_BODIES,
            },
        ) from exc


def validate_query_inputs(
    site: SiteSpec, epoch_utc: str, bodies: object
) -> tuple[SiteSpec, str, tuple[str, ...]]:
    """Validate all common provider query inputs in deterministic order."""
    return (
        validate_query_site(site),
        validate_query_epoch(epoch_utc),
        validate_query_bodies(bodies),
    )


class EphemerisProvider(ABC):
    @abstractmethod
    def query(
        self,
        site: SiteSpec,
        epoch_utc: str,
        bodies: tuple[str, ...] = ("sun",),
        *,
        optional_bodies: tuple[str, ...] = (),
    ) -> EphemerisState: ...


def register_provider(
    name: str,
) -> Callable[[type[EphemerisProvider]], type[EphemerisProvider]]:
    def deco(cls: type[EphemerisProvider]) -> type[EphemerisProvider]:
        _REGISTRY[name] = cls
        return cls

    return deco


def make_provider(name: str) -> EphemerisProvider:
    if type(name) is not str or not name.strip():
        raise InvalidInputError(
            code="ephemeris.provider.invalid_name",
            message="provider name must be a non-empty string",
            component=_PROVIDER_COMPONENT,
            details={"provider": repr(name)},
            retryable=False,
            fallback_allowed=False,
            evidence_valid=False,
        )

    try:
        if name == "spice" and "spice" not in _REGISTRY:
            import srb.core.ephemeris.spice_provider  # noqa: F401  (self-registers)
        if name == "table" and "table" not in _REGISTRY:
            import srb.core.ephemeris.table_provider  # noqa: F401  (self-registers)
    except FailureError:
        raise
    except ImportError as exc:
        error = OptionalDependencyUnavailableError(
            code="ephemeris.provider.optional_dependency",
            message=f"provider {name!r} is unavailable: {exc}",
            component=_PROVIDER_COMPONENT,
            details={
                "provider": name,
                "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
            },
            retryable=False,
            fallback_allowed=True,
            evidence_valid=False,
        )
        raise error from exc

    provider_type = _REGISTRY.get(name)
    if provider_type is None:
        raise InvalidInputError(
            code="ephemeris.provider.unknown",
            message=(
                f"unknown ephemeris provider {name!r}; expected one of "
                f"{sorted(_REGISTRY)}"
            ),
            component=_PROVIDER_COMPONENT,
            details={"provider": name, "available": sorted(_REGISTRY)},
            retryable=False,
            fallback_allowed=False,
            evidence_valid=False,
        )
    return provider_type()


def _query_health_event(
    *,
    provider: str,
    site: SiteSpec,
    epoch_utc: str,
    bodies: tuple[str, ...],
    error: BaseException,
) -> HealthEvent:
    if isinstance(error, FailureError):
        failure = error.info
    else:
        failure = classify_exception(
            error,
            component=_PROVIDER_COMPONENT,
            details={
                "provider": str(provider),
                "site_body": str(getattr(site, "body", type(site).__name__)),
                "epoch_utc": str(epoch_utc),
                "bodies": repr(bodies),
            },
        )
    return HealthEvent(
        status=HealthStatus.UNHEALTHY,
        severity=HealthSeverity.ERROR,
        component=_PROVIDER_COMPONENT,
        code="ephemeris.provider.query_failure",
        message=f"ephemeris provider query failed: {failure.message}",
        details={
            "provider": str(provider),
            "site_body": str(getattr(site, "body", type(site).__name__)),
            "epoch_utc": str(epoch_utc),
            "bodies": repr(bodies),
            "failure_category": failure.category.value,
            "failure_code": failure.code,
        },
        failure=failure,
    )


def _notify_query_failure(
    observer: HealthObserver | None,
    *,
    provider: str,
    site: SiteSpec,
    epoch_utc: str,
    bodies: tuple[str, ...],
    error: BaseException,
) -> None:
    if observer is None:
        return
    event = _query_health_event(
        provider=provider,
        site=site,
        epoch_utc=epoch_utc,
        bodies=bodies,
        error=error,
    )
    try:
        observer(event)
    except Exception:
        # Health checks must not mask provider failures or imply query success.
        return


def query(
    site: SiteSpec,
    epoch_utc: str,
    bodies: tuple[str, ...] = ("sun",),
    provider: str = "spice",
    *,
    health_observer: HealthObserver | None = None,
    optional_bodies: tuple[str, ...] = (),
) -> EphemerisState:
    if health_observer is not None and not callable(health_observer):
        raise InvalidInputError(
            code="ephemeris.provider.invalid_health_observer",
            message="health_observer must be callable or None",
            component=_PROVIDER_COMPONENT,
            details={"observer_type": type(health_observer).__name__},
            retryable=False,
            fallback_allowed=False,
            evidence_valid=False,
        )
    try:
        # Validate common inputs before custom providers can bypass the contract.
        validated_site, validated_epoch, validated_bodies = validate_query_inputs(
            site, epoch_utc, bodies
        )
        validated_optional = validate_query_bodies(optional_bodies)
        overlap = set(validated_bodies) & set(validated_optional)
        if overlap:
            raise _query_invalid(
                "optional ephemeris bodies must not also be required",
                code="ephemeris.provider.query.optional_bodies",
                details={"overlap": tuple(sorted(overlap))},
            )
        backend = make_provider(provider)
        if validated_optional:
            return backend.query(
                validated_site,
                validated_epoch,
                validated_bodies,
                optional_bodies=validated_optional,
            )
        return backend.query(validated_site, validated_epoch, validated_bodies)
    except FailureError as exc:
        _notify_query_failure(
            health_observer,
            provider=provider,
            site=site,
            epoch_utc=epoch_utc,
            bodies=bodies,
            error=exc,
        )
        raise
    except Exception as exc:
        _notify_query_failure(
            health_observer,
            provider=provider,
            site=site,
            epoch_utc=epoch_utc,
            bodies=bodies,
            error=exc,
        )
        raise
