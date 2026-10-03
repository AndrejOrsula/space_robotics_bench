"""Classify terrain-source health and fallback diagnostics in pure Python."""

from __future__ import annotations

import logging
import re
from collections import OrderedDict
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from srb.terrain.errors import (
    ChecksumMismatchError,
    DemLoadError,
    FetchError,
    Http4xxError,
    Http5xxError,
    InvalidSourcePayloadError,
    LargeDownloadConsentRequiredError,
    MissingTerrainDependencyError,
    NetworkError,
    RegionUnresolvedError,
    SourceTooLargeError,
    TerrainError,
    TerrainNotBakedError,
    UnreadableFileError,
)
from srb.terrain.spec import TerrainSpec
from srb.utils.failures import (
    FALLBACK_ALLOWED_CATEGORIES,
    FailureCategory,
    FailureError,
    FailureInfo,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
)

_MAX_EMITTED_FALLBACK_WARNINGS = 256
_MAX_EMITTED_EPHEMERIS_LIGHTING_WARNINGS = 256


class _BoundedWarningKeys:
    """Deterministic LRU keys for process-local warning de-duplication."""

    def __init__(self, max_size: int) -> None:
        if max_size < 1:
            raise ValueError("max_size must be positive")
        self._max_size = max_size
        self._keys: OrderedDict[tuple[str, ...], None] = OrderedDict()

    def __contains__(self, key: object) -> bool:
        return key in self._keys

    def __len__(self) -> int:
        return len(self._keys)

    def add(self, key: tuple[str, ...]) -> None:
        if key in self._keys:
            self._keys.move_to_end(key)
            return
        if len(self._keys) >= self._max_size:
            self._keys.popitem(last=False)
        self._keys[key] = None

    def touch(self, key: tuple[str, ...]) -> None:
        if key in self._keys:
            self._keys.move_to_end(key)

    def discard(self, key: tuple[str, ...]) -> None:
        self._keys.pop(key, None)

    def clear(self) -> None:
        self._keys.clear()


_EMITTED_FALLBACK_WARNINGS = _BoundedWarningKeys(_MAX_EMITTED_FALLBACK_WARNINGS)
_EMITTED_EPHEMERIS_LIGHTING_WARNINGS = _BoundedWarningKeys(
    _MAX_EMITTED_EPHEMERIS_LIGHTING_WARNINGS
)
_LAST_TERRAIN_SOURCE_HEALTH_EVENT: HealthEvent | None = None
_LAST_EPHEMERIS_LIGHTING_HEALTH_EVENT: HealthEvent | None = None
_HEALTH_OBSERVER_MESSAGE_LIMIT = 256
_LOGGER = logging.getLogger(__name__)


class TerrainSourceIssueKind(str, Enum):
    MANIFEST_PLACEHOLDER = "manifest_placeholder"
    NOT_FOUND = "not_found"
    FORBIDDEN = "forbidden"
    HTTP_CLIENT_ERROR = "http_client_error"
    HTTP_SERVER_ERROR = "http_server_error"
    LARGE_DOWNLOAD_CONSENT_REQUIRED = "large_download_consent_required"
    SOURCE_TOO_LARGE = "source_too_large"
    NETWORK = "network"
    CHECKSUM_MISMATCH = "checksum_mismatch"
    INVALID_SOURCE_PAYLOAD = "invalid_source_payload"
    MISSING_DEPENDENCY = "missing_dependency"
    UNREADABLE_DEM = "unreadable_dem"
    NOT_BAKED = "not_baked"
    DEM_LOAD = "dem_load"
    TERRAIN_PIPELINE = "terrain_pipeline"
    UNKNOWN = "unknown"


@dataclass(frozen=True, slots=True, init=False)
class TerrainSourceIssue:
    """Wrap a canonical terrain failure while retaining the legacy API flag.

    Fallback policy uses ``failure.fallback_allowed``. The legacy
    ``recoverable`` flag cannot soften hard failures; not-baked remains allowed.
    """

    kind: TerrainSourceIssueKind
    summary: str
    remediation: str
    recoverable: bool
    exception: BaseException
    http_status: int | None = None
    failure: FailureInfo = field(init=False)

    def __init__(
        self,
        kind: TerrainSourceIssueKind,
        summary: str,
        remediation: str,
        recoverable: bool,
        exception: BaseException,
        http_status: int | None = None,
        failure: FailureInfo | None = None,
    ) -> None:
        if failure is None and isinstance(exception, FailureError):
            failure = exception.failure_info

        # Preserve typed failures; rebuilding loses their policy and provenance.
        typed_failure = (
            isinstance(exception, FailureError) and failure is exception.failure_info
        )
        if typed_failure:
            assert failure is not None
            canonical_recoverable = _failure_allows_fallback(failure)
            canonical_failure = failure
        else:
            canonical_recoverable = _canonical_recoverable(kind, recoverable)
            canonical_failure = _failure_info_for_issue(
                kind=kind,
                summary=summary,
                remediation=remediation,
                exception=exception,
                http_status=http_status,
                recoverable=canonical_recoverable,
            )
            if failure is not None and (
                failure.category is not canonical_failure.category
                or failure.fallback_allowed is not canonical_failure.fallback_allowed
                or failure.retryable is not canonical_failure.retryable
                or failure.evidence_valid is not canonical_failure.evidence_valid
            ):
                raise ValueError(
                    f"injected failure does not match the canonical failure policy "
                    f"for {kind.value}"
                )
        object.__setattr__(self, "kind", kind)
        object.__setattr__(self, "summary", summary)
        object.__setattr__(self, "remediation", remediation)
        object.__setattr__(self, "recoverable", canonical_recoverable)
        object.__setattr__(self, "exception", exception)
        object.__setattr__(self, "http_status", http_status)
        object.__setattr__(
            self,
            "failure",
            canonical_failure if failure is None else failure,
        )
        self.__post_init__()

    def __post_init__(self) -> None:
        if not isinstance(self.failure, FailureInfo):
            raise TypeError("failure must be FailureInfo")
        if self.kind is TerrainSourceIssueKind.UNKNOWN and not (
            isinstance(self.exception, FailureError)
            and self.failure is self.exception.failure_info
        ):
            # An explicitly supplied legacy ``recoverable`` value must not
            # turn an unclassified exception into a successful fallback.
            object.__setattr__(self, "recoverable", False)

    @property
    def failure_info(self) -> FailureInfo:
        """Compatibility spelling for callers that name the payload fully."""
        return self.failure

    @property
    def info(self) -> FailureInfo:
        """Short compatibility spelling for the canonical failure payload."""
        return self.failure_info


class TerrainFallbackWarning(str):
    """String-compatible warning carrying its canonical health event."""

    health_event: HealthEvent

    def __new__(cls, message: str, health_event: HealthEvent) -> TerrainFallbackWarning:  # noqa: PYI034
        value = str.__new__(cls, message)
        value.health_event = health_event
        return value


@dataclass(frozen=True, slots=True)
class TerrainSourceResolution:
    baked_path: Path | None
    issue: TerrainSourceIssue | None

    def require_path(self) -> Path:
        if self.baked_path is not None:
            return self.baked_path
        assert self.issue is not None
        raise self.issue.exception


_HTTP_STATUS_RE = re.compile(r"\bHTTP\s+(\d{3})\b")


def _canonical_recoverable(kind: TerrainSourceIssueKind, recoverable: bool) -> bool:
    """Keep the legacy flag from weakening a hard terrain issue kind."""
    hard_kinds = {
        TerrainSourceIssueKind.MANIFEST_PLACEHOLDER,
        TerrainSourceIssueKind.NOT_FOUND,
        TerrainSourceIssueKind.FORBIDDEN,
        TerrainSourceIssueKind.HTTP_CLIENT_ERROR,
        TerrainSourceIssueKind.LARGE_DOWNLOAD_CONSENT_REQUIRED,
        TerrainSourceIssueKind.SOURCE_TOO_LARGE,
        TerrainSourceIssueKind.CHECKSUM_MISMATCH,
        TerrainSourceIssueKind.INVALID_SOURCE_PAYLOAD,
        TerrainSourceIssueKind.UNREADABLE_DEM,
        TerrainSourceIssueKind.DEM_LOAD,
        TerrainSourceIssueKind.TERRAIN_PIPELINE,
        TerrainSourceIssueKind.UNKNOWN,
    }
    if kind in hard_kinds:
        return False
    if kind is TerrainSourceIssueKind.NOT_BAKED:
        return True
    return recoverable


def _failure_allows_fallback(failure: FailureInfo) -> bool:
    """Return the only policy that may authorize a fallback."""
    return failure.fallback_allowed and failure.category in FALLBACK_ALLOWED_CATEGORIES


def _failure_info_for_issue(
    *,
    kind: TerrainSourceIssueKind,
    summary: str,
    remediation: str,
    exception: BaseException,
    http_status: int | None,
    recoverable: bool,
) -> FailureInfo:
    """Build the caller-action contract for a legacy terrain issue."""
    integrity_kinds = {
        TerrainSourceIssueKind.MANIFEST_PLACEHOLDER,
        TerrainSourceIssueKind.CHECKSUM_MISMATCH,
        TerrainSourceIssueKind.INVALID_SOURCE_PAYLOAD,
        TerrainSourceIssueKind.UNREADABLE_DEM,
        TerrainSourceIssueKind.DEM_LOAD,
    }
    unsupported_kinds = {TerrainSourceIssueKind.NOT_BAKED}
    transient_kinds = {
        TerrainSourceIssueKind.NETWORK,
        TerrainSourceIssueKind.HTTP_SERVER_ERROR,
    }
    invalid_input_kinds = {
        TerrainSourceIssueKind.NOT_FOUND,
        TerrainSourceIssueKind.FORBIDDEN,
        TerrainSourceIssueKind.HTTP_CLIENT_ERROR,
        TerrainSourceIssueKind.LARGE_DOWNLOAD_CONSENT_REQUIRED,
        TerrainSourceIssueKind.SOURCE_TOO_LARGE,
    }
    optional_kinds = {TerrainSourceIssueKind.MISSING_DEPENDENCY}

    if kind in integrity_kinds:
        category = FailureCategory.INTEGRITY_FAILURE
        retryable = False
        fallback_allowed = False
    elif kind in unsupported_kinds:
        category = FailureCategory.UNSUPPORTED_CAPABILITY
        retryable = False
        fallback_allowed = True
    elif kind in transient_kinds:
        category = FailureCategory.TRANSIENT_EXTERNAL_FAILURE
        retryable = True
        fallback_allowed = recoverable
    elif kind in invalid_input_kinds:
        category = FailureCategory.INVALID_INPUT
        retryable = False
        fallback_allowed = False
    elif kind in optional_kinds:
        category = FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE
        retryable = False
        fallback_allowed = True
    else:
        category = FailureCategory.INTERNAL_INVARIANT
        retryable = False
        fallback_allowed = False

    details: dict[str, object] = {
        "kind": kind.value,
        "exception_type": (
            f"{type(exception).__module__}.{type(exception).__qualname__}"
        ),
        "remediation": remediation,
    }
    if http_status is not None:
        details["http_status"] = http_status
    return FailureInfo(
        category=category,
        code=f"terrain.source.{kind.value}",
        message=summary or type(exception).__name__,
        component="terrain.source",
        details=details,
        retryable=retryable,
        fallback_allowed=fallback_allowed,
        evidence_valid=False,
    )


def classify_terrain_source_exception(exc: BaseException) -> TerrainSourceIssue:
    """Classify terrain fetch/bake failures for runtime fallback policy."""
    if isinstance(exc, FailureError):
        failure = exc.failure_info
        return TerrainSourceIssue(
            # Keep the typed failure authoritative; legacy kinds are lossy.
            kind=TerrainSourceIssueKind.UNKNOWN,
            summary=failure.message,
            remediation=getattr(exc, "remediation", "") or _default_remediation(exc),
            recoverable=_failure_allows_fallback(failure),
            exception=exc,
            failure=failure,
        )
    summary = str(exc) or exc.__class__.__name__
    remediation = getattr(exc, "remediation", "") or _default_remediation(exc)

    if isinstance(exc, ChecksumMismatchError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.CHECKSUM_MISMATCH,
            summary=summary,
            remediation=(
                "Verify the terrain manifest sha256 or clear the raw cache and "
                "re-download the source."
            ),
            recoverable=False,
            exception=exc,
        )
    if isinstance(exc, InvalidSourcePayloadError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.INVALID_SOURCE_PAYLOAD,
            summary=summary,
            remediation=remediation,
            recoverable=False,
            exception=exc,
        )
    if isinstance(exc, TerrainNotBakedError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.NOT_BAKED,
            summary=summary,
            remediation=remediation,
            recoverable=True,
            exception=exc,
        )
    if isinstance(exc, RegionUnresolvedError):
        # Treat missing catalog coverage as an observable fallback condition.
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.NOT_BAKED,
            summary=summary,
            remediation=remediation,
            recoverable=True,
            exception=exc,
        )
    if isinstance(exc, MissingTerrainDependencyError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.MISSING_DEPENDENCY,
            summary=summary,
            remediation=remediation,
            recoverable=False,
            exception=exc,
        )
    if isinstance(exc, Http4xxError):
        status = _http_status(summary)
        if status == 403:
            kind = TerrainSourceIssueKind.FORBIDDEN
            remediation = "Check the upstream access policy or mirror the source."
        elif status == 404:
            kind = TerrainSourceIssueKind.NOT_FOUND
            remediation = "Update the terrain manifest URL or use a mirrored source."
        else:
            kind = TerrainSourceIssueKind.HTTP_CLIENT_ERROR
        return TerrainSourceIssue(
            kind=kind,
            summary=summary,
            remediation=remediation,
            recoverable=False,
            exception=exc,
            http_status=status,
        )
    if isinstance(exc, Http5xxError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.HTTP_SERVER_ERROR,
            summary=summary,
            remediation=remediation,
            recoverable=True,
            exception=exc,
            http_status=_http_status(summary),
        )
    if isinstance(exc, NetworkError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.NETWORK,
            summary=summary,
            remediation=remediation,
            recoverable=True,
            exception=exc,
        )
    if isinstance(exc, UnreadableFileError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.UNREADABLE_DEM,
            summary=summary,
            remediation="Inspect the source GeoTIFF and GDAL/rasterio support.",
            recoverable=False,
            exception=exc,
        )
    if isinstance(exc, DemLoadError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.DEM_LOAD,
            summary=summary,
            remediation=remediation,
            recoverable=False,
            exception=exc,
        )
    if isinstance(exc, LargeDownloadConsentRequiredError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.LARGE_DOWNLOAD_CONSENT_REQUIRED,
            summary=summary,
            remediation=remediation,
            recoverable=False,
            exception=exc,
        )
    if isinstance(exc, SourceTooLargeError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.SOURCE_TOO_LARGE,
            summary=summary,
            remediation=remediation,
            recoverable=False,
            exception=exc,
        )
    if isinstance(exc, FetchError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.NETWORK,
            summary=summary,
            remediation=remediation,
            recoverable=True,
            exception=exc,
        )
    if isinstance(exc, TerrainError):
        return TerrainSourceIssue(
            kind=TerrainSourceIssueKind.TERRAIN_PIPELINE,
            summary=summary,
            remediation=remediation,
            recoverable=False,
            exception=exc,
        )
    return TerrainSourceIssue(
        kind=TerrainSourceIssueKind.UNKNOWN,
        summary=summary,
        remediation="Inspect the terrain source and bake logs.",
        recoverable=False,
        exception=exc,
    )


def resolve_terrain_source(
    spec: TerrainSpec,
    *,
    auto_fetch: bool = True,
    preset_name: str | None = None,
    bake: Callable[..., Path],
) -> TerrainSourceResolution:
    """Run a terrain bake/load adapter and return a classified resolution."""
    try:
        return TerrainSourceResolution(
            baked_path=Path(bake(spec, auto_fetch=auto_fetch, preset_name=preset_name)),
            issue=None,
        )
    except Exception as exc:  # noqa: BLE001 - adapter boundary classifies unknown failures
        return TerrainSourceResolution(
            baked_path=None,
            issue=classify_terrain_source_exception(exc),
        )


def format_terrain_fallback_warning(
    *,
    surface_name: str,
    body: str,
    site: str,
    issue: TerrainSourceIssue,
) -> str:
    """Return the single runtime warning for real-to-procedural fallback."""
    return (
        f"{surface_name} real terrain source {body}/{site} unavailable "
        f"({issue.kind.value}: {issue.summary}); falling back to procedural "
        f"terrain. Remediation: {issue.remediation}"
    )


def warn_terrain_fallback_once(
    *,
    surface_name: str,
    body: str,
    site: str,
    issue: TerrainSourceIssue,
    warn: Callable[[str], None],
    health: Callable[[HealthEvent], None] | None = None,
    on_health: Callable[[HealthEvent], None] | None = None,
) -> bool:
    """Emit one allowed fallback warning and optional ``srb.health/1`` event.

    Return whether the warning was emitted. ``health`` and legacy ``on_health``
    are aliases; integrity and internal failures re-raise the original error.
    """
    if health is not None and on_health is not None:
        raise TypeError("pass either health or on_health, not both")
    health_sink = health if health is not None else on_health
    health_event = make_terrain_fallback_health_event(
        surface_name=surface_name,
        body=body,
        site=site,
        issue=issue,
    )
    key = (surface_name, body, site, issue.kind.value, issue.summary)
    if key in _EMITTED_FALLBACK_WARNINGS:
        touch = getattr(_EMITTED_FALLBACK_WARNINGS, "touch", None)
        if touch is not None:
            touch(key)
        return False
    _EMITTED_FALLBACK_WARNINGS.add(key)
    try:
        publish_terrain_source_health(health_event, health=health_sink)
        warn(
            TerrainFallbackWarning(
                format_terrain_fallback_warning(
                    surface_name=surface_name,
                    body=body,
                    site=site,
                    issue=issue,
                ),
                health_event,
            )
        )
    except BaseException:
        _EMITTED_FALLBACK_WARNINGS.discard(key)
        raise
    return True


def make_terrain_source_health_event(
    failure: FailureInfo,
    *,
    component: str | None = None,
    code: str | None = None,
    message: str | None = None,
    details: Mapping[str, object] | None = None,
) -> HealthEvent:
    """Build a health event; only typed, fallback-authorized failures degrade."""
    event_component = failure.component if component is None else component
    event_code = failure.code if code is None else code
    event_message = failure.message if message is None else message
    if _failure_allows_fallback(failure):
        return HealthEvent.degraded_fallback(
            failure,
            component=event_component,
            code=event_code,
            message=event_message,
            details=details,
        )

    return HealthEvent(
        status=HealthStatus.UNHEALTHY,
        severity=(
            HealthSeverity.CRITICAL
            if failure.category is FailureCategory.SAFETY_VIOLATION
            else HealthSeverity.ERROR
        ),
        component=event_component,
        code=event_code,
        message=event_message,
        details=details,
        failure=failure,
    )


def _notify_health_observer(
    event: HealthEvent, health_sink: Callable[[HealthEvent], None] | None
) -> None:
    if health_sink is None:
        return
    try:
        health_sink(event)
    except Exception as exc:  # noqa: BLE001 - observer is diagnostic only
        _log_health_observer_failure(event, exc)


def publish_terrain_source_health(
    event: HealthEvent,
    *,
    health: Callable[[HealthEvent], None] | None = None,
    on_health: Callable[[HealthEvent], None] | None = None,
) -> None:
    """Retain the latest source event and send it to an optional observer."""
    if health is not None and on_health is not None:
        raise TypeError("pass either health or on_health, not both")
    global _LAST_TERRAIN_SOURCE_HEALTH_EVENT
    _LAST_TERRAIN_SOURCE_HEALTH_EVENT = event
    health_sink = health if health is not None else on_health
    _notify_health_observer(event, health_sink)


def _log_health_observer_failure(event: HealthEvent, exc: Exception) -> None:
    """Log one bounded diagnostic without allowing logging to affect callers."""
    try:
        message = str(exc) or type(exc).__name__
    except Exception:  # pragma: no cover - pathological exception __str__
        message = type(exc).__name__
    if len(message) > _HEALTH_OBSERVER_MESSAGE_LIMIT:
        message = message[:_HEALTH_OBSERVER_MESSAGE_LIMIT] + "..."
    try:
        _LOGGER.warning(
            "health observer failed for %s (%s): %s",
            event.component,
            f"{type(exc).__module__}.{type(exc).__qualname__}",
            message,
        )
    except Exception:  # pragma: no cover - logging must not mask source state
        pass


def make_terrain_fallback_health_event(
    *,
    surface_name: str,
    body: str,
    site: str,
    issue: TerrainSourceIssue,
) -> HealthEvent:
    """Return the canonical event for an allowed procedural fallback."""
    if not issue.failure_info.fallback_allowed or issue.failure_info.category not in {
        FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
        FailureCategory.TRANSIENT_EXTERNAL_FAILURE,
        FailureCategory.UNSUPPORTED_CAPABILITY,
    }:
        raise issue.exception
    return HealthEvent.degraded_fallback(
        issue.failure_info,
        component=surface_name,
        code="terrain.procedural_fallback",
        message=(f"procedural terrain fallback for {body}/{site}: {issue.summary}"),
        details={
            "body": body,
            "site": site,
            "surface_name": surface_name,
            "issue_kind": issue.kind.value,
            "remediation": issue.remediation,
        },
    )


def format_ephemeris_lighting_fallback_warning(
    *,
    site: str,
    body: str,
    epoch: str,
    reason: str,
) -> str:
    """Return the single runtime warning for ephemeris-to-static lighting fallback."""
    from srb.terrain.lights.ephemeris_bridge import _body_default_preset_name

    default_preset = _body_default_preset_name(body)
    return (
        f"ephemeris lighting for {body}/{site} @ {epoch} unavailable ({reason}); "
        f"falling back to static '{default_preset}' lighting preset. "
        "Remediation: run 'srb ephemeris download' to enable ephemeris-driven "
        "lighting."
    )


def warn_ephemeris_lighting_fallback_once(
    *,
    site: str,
    body: str,
    epoch: str,
    reason: str,
    warn: Callable[[str], None],
    health: Callable[[HealthEvent], None] | None = None,
    on_health: Callable[[HealthEvent], None] | None = None,
) -> bool:
    """Emit a warning and degraded health for static-lighting fallback once."""
    if health is not None and on_health is not None:
        raise TypeError("pass either health or on_health, not both")
    health_sink = health if health is not None else on_health
    health_event = make_ephemeris_lighting_fallback_health_event(
        site=site,
        body=body,
        epoch=epoch,
        reason=reason,
    )
    global _LAST_EPHEMERIS_LIGHTING_HEALTH_EVENT
    _LAST_EPHEMERIS_LIGHTING_HEALTH_EVENT = health_event
    key = (site, body, epoch, reason)
    if key in _EMITTED_EPHEMERIS_LIGHTING_WARNINGS:
        touch = getattr(_EMITTED_EPHEMERIS_LIGHTING_WARNINGS, "touch", None)
        if touch is not None:
            touch(key)
        return False
    _EMITTED_EPHEMERIS_LIGHTING_WARNINGS.add(key)
    try:
        warn(
            format_ephemeris_lighting_fallback_warning(
                site=site,
                body=body,
                epoch=epoch,
                reason=reason,
            )
        )
        _notify_health_observer(health_event, health_sink)
    except BaseException:
        _EMITTED_EPHEMERIS_LIGHTING_WARNINGS.discard(key)
        raise
    return True


def make_ephemeris_lighting_fallback_health_event(
    *,
    site: str,
    body: str,
    epoch: str,
    reason: str,
) -> HealthEvent:
    """Return the typed degraded event for an ephemeris-to-static fallback."""
    failure = FailureInfo(
        category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
        code="terrain.lighting.ephemeris_kernels_missing",
        message=reason,
        component="terrain.lighting.ephemeris",
        details={
            "body": body,
            "site": site,
            "epoch": epoch,
            "reason": reason,
        },
        retryable=False,
        fallback_allowed=True,
        evidence_valid=False,
    )
    return HealthEvent.degraded_fallback(
        failure,
        component="terrain.lighting",
        code="terrain.lighting.ephemeris_fallback",
        message=(f"static lighting fallback for {body}/{site} @ {epoch}: {reason}"),
        details={
            "body": body,
            "site": site,
            "epoch": epoch,
            "mode": "static",
        },
    )


def get_last_ephemeris_lighting_health_event() -> HealthEvent | None:
    """Return the last ephemeris-lighting fallback event, if one occurred."""
    return _LAST_EPHEMERIS_LIGHTING_HEALTH_EVENT


def _http_status(message: str) -> int | None:
    match = _HTTP_STATUS_RE.search(message)
    return int(match.group(1)) if match else None


def _default_remediation(exc: BaseException) -> str:
    if isinstance(exc, TerrainError):
        return "Inspect the terrain source and retry the bake."
    return "Inspect the terrain source and runtime logs."
