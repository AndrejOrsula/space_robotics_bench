"""Import-light failure, cleanup, evidence, and runtime-health contracts."""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType
from typing import ClassVar, TypeAlias, TypeVar, cast


class FailureCategory(str, Enum):
    """The action a caller may take after a failure."""

    INVALID_INPUT = "invalid_input"
    UNSUPPORTED_CAPABILITY = "unsupported_capability"
    OPTIONAL_DEPENDENCY_UNAVAILABLE = "optional_dependency_unavailable"
    TRANSIENT_EXTERNAL_FAILURE = "transient_external_failure"
    INTEGRITY_FAILURE = "integrity_failure"
    SAFETY_VIOLATION = "safety_violation"
    INTERNAL_INVARIANT = "internal_invariant"


FALLBACK_ALLOWED_CATEGORIES: frozenset[FailureCategory] = frozenset(
    {
        FailureCategory.UNSUPPORTED_CAPABILITY,
        FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
        FailureCategory.TRANSIENT_EXTERNAL_FAILURE,
    }
)


CALLER_ACTION_CATEGORIES: tuple[FailureCategory, ...] = tuple(FailureCategory)


class HealthStatus(str, Enum):
    """Observable state of a component."""

    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"


class HealthSeverity(str, Enum):
    """Severity of a health observation."""

    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


HEALTH_EVENT_SCHEMA = "srb.health/1"

SnapshotValue: TypeAlias = (
    None
    | bool
    | int
    | float
    | str
    | tuple["SnapshotValue", ...]
    | Mapping[str, "SnapshotValue"]
)
SerializableValue: TypeAlias = (
    None
    | bool
    | int
    | float
    | str
    | list["SerializableValue"]
    | dict[str, "SerializableValue"]
)

_EnumType = TypeVar("_EnumType", bound=Enum)
_CleanupResult = TypeVar("_CleanupResult")


def attempt_cleanup(
    operation: Callable[[], _CleanupResult],
) -> tuple[_CleanupResult | None, BaseException | None]:
    """Run cleanup once and return either its result or its original error."""
    try:
        return operation(), None
    except BaseException as error:  # noqa: BLE001 - preserve cleanup authority
        return None, error


class KitBindingError(RuntimeError):
    """Raised when Kit startup finds incompatible Python package bindings."""


_REDACTED = "[REDACTED]"
_SENSITIVE_KEY_FORMS = frozenset(
    {
        "password",
        "passwd",
        "passphrase",
        "secret",
        "credential",
        "credentials",
        "token",
        "api_key",
        "access_key",
        "secret_key",
        "private_key",
        "signing_key",
        "api_token",
        "access_token",
        "refresh_token",
        "id_token",
        "auth_token",
        "oauth_token",
        "session_token",
        "authorization",
        "proxy_authorization",
        "cookie",
        "set_cookie",
    }
)
_SENSITIVE_KEY_TOKENS = frozenset(
    {
        "password",
        "passwd",
        "passphrase",
        "secret",
        "credential",
        "credentials",
        "authorization",
        "cookie",
    }
)


def _normalized_key_tokens(value: str) -> tuple[str, ...]:
    value = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", value)
    value = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", value)
    value = re.sub(r"[^A-Za-z0-9]+", "_", value)
    return tuple(token.lower() for token in value.strip("_").split("_") if token)


def _is_sensitive_key(value: object) -> bool:
    if not isinstance(value, str):
        return False
    tokens = _normalized_key_tokens(value)
    if not tokens:
        return False
    canonical = "_".join(tokens)
    if canonical in _SENSITIVE_KEY_FORMS:
        return True
    if any(token in _SENSITIVE_KEY_TOKENS for token in tokens):
        return True
    # Treat suffixes as credential names while preserving keys like token_count.
    if tokens[-1] == "token":
        return True
    for form in _SENSITIVE_KEY_FORMS:
        form_tokens = tuple(form.split("_"))
        if len(form_tokens) > 1 and any(
            tokens[index : index + len(form_tokens)] == form_tokens
            for index in range(len(tokens) - len(form_tokens) + 1)
        ):
            return True
    return False


_AUTH_CONTEXT_QUOTED_RE = re.compile(
    r"(?P<prefix>\b(?:proxy[-_. ]?authorization|authorization|auth)"
    r"[ \t]*[:=][ \t]*)(?P<scheme>Bearer|Basic)(?P<separator>[ \t]+)"
    r"(?P<quote>['\"])(?P<value>(?:\\.|[^'\r\n\"\\])*\\?)(?P=quote)",
    re.IGNORECASE,
)
_AUTH_CONTEXT_RE = re.compile(
    r"(?P<prefix>\b(?:proxy[-_. ]?authorization|authorization|auth)"
    r"[ \t]*[:=][ \t]*)(?P<scheme>Bearer|Basic)(?P<separator>[ \t]+)"
    r"(?P<value>[^\s,;]+)",
    re.IGNORECASE,
)
_AUTH_QUOTED_RE = re.compile(
    r"\b(?P<scheme>Bearer|Basic)(?P<separator>[ \t]+)"
    r"(?P<quote>['\"])(?P<value>(?:\\.|[^'\r\n\"\\])*\\?)(?P=quote)",
    re.IGNORECASE,
)
_AUTH_RE = re.compile(
    r"\b(?P<scheme>Bearer|Basic)(?P<separator>[ \t]+)" r"(?P<value>[^\s,;]+)",
    re.IGNORECASE,
)
_URL_USERINFO_RE = re.compile(
    r"(?P<scheme>[A-Za-z][A-Za-z0-9+.-]*://)(?P<userinfo>[^/?#\s@]+@)"
)
_URL_QUERY_PARAMETER_RE = re.compile(
    r"(?P<marker>[?&])(?P<key>[A-Za-z][A-Za-z0-9_.-]*)(?P<equals>=)"
    r"(?P<value>[^&#\s]*)"
)
_KEY_VALUE_QUOTED_RE = re.compile(
    r"(?<![A-Za-z0-9_.-])(?P<prefix>--)?"
    r"(?P<key_quote>['\"])?(?P<key>[A-Za-z][A-Za-z0-9_.-]*)"
    r"(?(key_quote)(?P=key_quote))(?P<separator>[ \t]*[:=][ \t]*)"
    r"(?P<quote>['\"])(?P<value>(?:\\.|[^'\r\n\"\\])*\\?)(?P=quote)",
    re.IGNORECASE,
)
_KEY_VALUE_RE = re.compile(
    r"(?<![A-Za-z0-9_.-])(?P<prefix>--)?"
    r"(?P<key>[A-Za-z][A-Za-z0-9_.-]*)(?P<separator>[ \t]*[:=][ \t]*)"
    r"(?!['\"])(?P<value>[^\s,;]+?)(?P<trailing>[.,!?)]?)(?=$|[\s,;])",
    re.IGNORECASE,
)
_CLI_VALUE_QUOTED_RE = re.compile(
    r"(?<![A-Za-z0-9_.-])(?P<prefix>--)"
    r"(?P<key>[A-Za-z][A-Za-z0-9_.-]*)(?P<separator>[ \t]+)"
    r"(?P<quote>['\"])(?P<value>(?:\\.|[^'\r\n\"\\])*\\?)(?P=quote)",
    re.IGNORECASE,
)
_CLI_VALUE_RE = re.compile(
    r"(?<![A-Za-z0-9_.-])(?P<prefix>--)"
    r"(?P<key>[A-Za-z][A-Za-z0-9_.-]*)(?P<separator>[ \t]+)"
    r"(?!['\"])(?P<value>[^\s,;]+?)(?P<trailing>[.,!?)]?)(?=$|[\s,;])",
    re.IGNORECASE,
)
_ORDINARY_AUTH_WORDS = frozenset(
    {
        "example",
        "authentication",
        "authorization",
        "header",
        "request",
        "scheme",
    }
)


def _redact_auth_match(match: re.Match[str]) -> str:
    groups = match.groupdict()
    prefix = groups.get("prefix") or ""
    scheme = match.group("scheme")
    separator = match.group("separator")
    quote = groups.get("quote") or ""
    return f"{prefix}{scheme}{separator}{quote}{_REDACTED}{quote}"


def _redact_standalone_auth_match(match: re.Match[str]) -> str:
    if match.group("value").casefold() in _ORDINARY_AUTH_WORDS:
        return match.group(0)
    return _redact_auth_match(match)


def _redact_url_userinfo(match: re.Match[str]) -> str:
    return f"{match.group('scheme')}{_REDACTED}@"


def _redact_url_query_parameter(match: re.Match[str]) -> str:
    if not _is_sensitive_key(match.group("key")) or not match.group("value"):
        return match.group(0)
    return f"{match.group('marker')}{match.group('key')}={_REDACTED}"


def _redact_key_value(match: re.Match[str]) -> str:
    if not _is_sensitive_key(match.group("key")):
        return match.group(0)
    value = match.group("value")
    if value.casefold() in {"bearer", "basic"}:
        return match.group(0)
    groups = match.groupdict()
    prefix = groups.get("prefix") or ""
    key_quote = groups.get("key_quote") or ""
    separator = match.group("separator")
    quote = groups.get("quote") or ""
    trailing = groups.get("trailing") or ""
    return (
        f"{prefix}{key_quote}{match.group('key')}{key_quote}"
        f"{separator}{quote}{_REDACTED}{quote}{trailing}"
    )


def _sanitize_text(value: str) -> str:
    sanitized = _AUTH_CONTEXT_QUOTED_RE.sub(_redact_auth_match, value)
    sanitized = _AUTH_CONTEXT_RE.sub(_redact_auth_match, sanitized)
    sanitized = _AUTH_QUOTED_RE.sub(_redact_standalone_auth_match, sanitized)
    sanitized = _AUTH_RE.sub(_redact_standalone_auth_match, sanitized)
    sanitized = _URL_USERINFO_RE.sub(_redact_url_userinfo, sanitized)
    sanitized = _URL_QUERY_PARAMETER_RE.sub(_redact_url_query_parameter, sanitized)
    sanitized = _KEY_VALUE_QUOTED_RE.sub(_redact_key_value, sanitized)
    sanitized = _KEY_VALUE_RE.sub(_redact_key_value, sanitized)
    sanitized = _CLI_VALUE_QUOTED_RE.sub(_redact_key_value, sanitized)
    return _CLI_VALUE_RE.sub(_redact_key_value, sanitized)


def _coerce_enum(
    value: object, enum_type: type[_EnumType], field_name: str
) -> _EnumType:
    if isinstance(value, enum_type):
        return value
    if not isinstance(value, str):
        raise TypeError(f"{field_name} must be a string enum value")
    try:
        return enum_type(value)
    except ValueError as exc:
        allowed = ", ".join(repr(member.value) for member in enum_type)
        raise ValueError(
            f"unknown {field_name} {value!r}; expected one of {allowed}"
        ) from exc


def _coerce_canonical_enum(
    value: object, enum_type: type[_EnumType], field_name: str
) -> _EnumType:
    """Coerce a direct-constructor value without accepting legacy aliases."""
    if isinstance(value, enum_type):
        return value
    if type(value) is not str:
        raise TypeError(f"{field_name} must be a canonical string enum value")
    try:
        return enum_type(value)
    except ValueError as exc:
        allowed = ", ".join(repr(member.value) for member in enum_type)
        raise ValueError(
            f"unknown {field_name} {value!r}; expected one of {allowed}"
        ) from exc


def _coerce_status(value: object) -> HealthStatus:
    aliases = {
        "ok": HealthStatus.HEALTHY,
        "pass": HealthStatus.HEALTHY,
        "warning": HealthStatus.DEGRADED,
        "failed": HealthStatus.UNHEALTHY,
        "failure": HealthStatus.UNHEALTHY,
        "error": HealthStatus.UNHEALTHY,
    }
    if isinstance(value, str):
        value = aliases.get(value, value)
    return _coerce_enum(value, HealthStatus, "status")


def _coerce_severity(value: object) -> HealthSeverity:
    aliases = {"warn": HealthSeverity.WARNING, "fatal": HealthSeverity.CRITICAL}
    if isinstance(value, str):
        value = aliases.get(value, value)
    return _coerce_enum(value, HealthSeverity, "severity")


def _require_text(value: object, field_name: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{field_name} must be a string")
    if not value.strip():
        raise ValueError(f"{field_name} must not be empty")
    return value


def _require_plain_text(value: object, field_name: str) -> str:
    if type(value) is not str:
        raise TypeError(f"{field_name} must be a string")
    if not value.strip():
        raise ValueError(f"{field_name} must not be empty")
    return value


def _require_bool(value: object, field_name: str) -> bool:
    if type(value) is not bool:
        raise TypeError(f"{field_name} must be a bool")
    return value


def _string_mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{name} must be a mapping")
    candidate = cast(Mapping[object, object], value)
    result: dict[str, object] = {}
    for key, item in candidate.items():
        if not isinstance(key, str):
            raise TypeError(f"{name} keys must be strings")
        result[str(key)] = item
    return result


def _freeze_value(value: object, *, path: str) -> SnapshotValue:
    if value is None:
        return value
    if type(value) is bool:
        return value
    if type(value) is int:
        return value
    if type(value) is str:
        return _sanitize_text(value)
    # Normalize scalar subclasses to prevent custom objects in snapshots.
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, int):
        return int(value)
    if isinstance(value, str):
        return _sanitize_text(str(value))
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{path} must be finite")
        return float(value)
    if isinstance(value, Mapping):
        candidate = cast(Mapping[object, object], value)
        frozen: dict[str, SnapshotValue] = {}
        for key, item in candidate.items():
            if not isinstance(key, str):
                raise TypeError(f"{path} keys must be strings")
            frozen[str(key)] = (
                _REDACTED
                if _is_sensitive_key(key)
                else _freeze_value(item, path=f"{path}.{key}")
            )
        return MappingProxyType(dict(sorted(frozen.items())))
    if isinstance(value, (list, tuple)):
        return tuple(
            _freeze_value(item, path=f"{path}[{index}]")
            for index, item in enumerate(value)
        )
    raise TypeError(
        f"{path} contains unsupported non-serializable value {type(value).__name__}"
    )


def _freeze_details(
    details: Mapping[str, object] | None,
) -> Mapping[str, SnapshotValue]:
    source: Mapping[str, object] = {} if details is None else details
    frozen = _freeze_value(source, path="details")
    if not isinstance(frozen, Mapping):  # pragma: no cover - _freeze_value contract
        raise TypeError("details must be a mapping")
    return frozen


def _thaw_value(value: SnapshotValue) -> SerializableValue:
    if isinstance(value, Mapping):
        return {key: _thaw_value(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw_value(item) for item in value]
    return value


def _reject_unknown_keys(
    mapping: Mapping[str, object], expected: frozenset[str], name: str
) -> None:
    unknown = sorted(set(mapping) - expected)
    if unknown:
        raise ValueError(f"{name} contains unknown field(s): {', '.join(unknown)}")


@dataclass(frozen=True, slots=True, init=False)
class FailureInfo:
    """Immutable, serializable description of one caller-action failure."""

    category: FailureCategory
    code: str
    message: str
    component: str
    details: Mapping[str, SnapshotValue]
    retryable: bool
    fallback_allowed: bool
    evidence_valid: bool

    def __init__(
        self,
        category: FailureCategory,
        code: str,
        message: str,
        component: str,
        details: Mapping[str, object] | None = None,
        retryable: bool = False,
        fallback_allowed: bool = False,
        evidence_valid: bool = False,
    ) -> None:
        object.__setattr__(self, "category", category)
        object.__setattr__(self, "code", code)
        object.__setattr__(self, "message", message)
        object.__setattr__(self, "component", component)
        object.__setattr__(self, "details", details if details is not None else {})
        object.__setattr__(self, "retryable", retryable)
        object.__setattr__(self, "fallback_allowed", fallback_allowed)
        object.__setattr__(self, "evidence_valid", evidence_valid)
        self.__post_init__()

    def __post_init__(self) -> None:
        category = _coerce_enum(self.category, FailureCategory, "category")
        object.__setattr__(self, "category", category)
        object.__setattr__(self, "code", _require_text(self.code, "code"))
        object.__setattr__(
            self,
            "message",
            _sanitize_text(_require_text(self.message, "message")),
        )
        object.__setattr__(
            self, "component", _require_text(self.component, "component")
        )
        object.__setattr__(
            self,
            "details",
            _freeze_details(cast(Mapping[str, object], self.details)),
        )
        retryable = _require_bool(self.retryable, "retryable")
        fallback_allowed = _require_bool(self.fallback_allowed, "fallback_allowed")
        evidence_valid = _require_bool(self.evidence_valid, "evidence_valid")
        if fallback_allowed and category not in FALLBACK_ALLOWED_CATEGORIES:
            raise ValueError(f"{category.value} cannot claim fallback_allowed")
        if (
            category
            in (
                FailureCategory.INTEGRITY_FAILURE,
                FailureCategory.SAFETY_VIOLATION,
            )
            and evidence_valid
        ):
            raise ValueError(f"{category.value} cannot claim evidence_valid")
        object.__setattr__(self, "retryable", retryable)
        object.__setattr__(self, "fallback_allowed", fallback_allowed)
        object.__setattr__(self, "evidence_valid", evidence_valid)

    def __copy__(self) -> FailureInfo:
        return self

    def __deepcopy__(self, memo: dict[int, object]) -> FailureInfo:
        """Return ``self``; frozen fields and nested mappings are immutable.

        ``MappingProxyType`` cannot be pickled, so ``configclass`` deep-copying
        this value during environment setup otherwise fails.
        """
        return self

    def to_mapping(self) -> dict[str, SerializableValue]:
        """Return a fresh deterministic JSON-shaped mapping."""
        return {
            "category": self.category.value,
            "code": self.code,
            "message": self.message,
            "component": self.component,
            "details": _thaw_value(self.details),
            "retryable": self.retryable,
            "fallback_allowed": self.fallback_allowed,
            "evidence_valid": self.evidence_valid,
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> FailureInfo:
        """Validate and rebuild a :class:`FailureInfo` from its mapping."""
        mapping = _string_mapping(value, "failure info")
        _reject_unknown_keys(
            mapping,
            frozenset(
                {
                    "category",
                    "code",
                    "message",
                    "component",
                    "details",
                    "retryable",
                    "fallback_allowed",
                    "evidence_valid",
                }
            ),
            "failure info",
        )
        required = (
            "category",
            "code",
            "message",
            "component",
            "details",
            "retryable",
            "fallback_allowed",
            "evidence_valid",
        )
        missing = [key for key in required if key not in mapping]
        if missing:
            raise ValueError(f"failure info missing field(s): {', '.join(missing)}")
        details = _string_mapping(mapping["details"], "failure info.details")
        return cls(
            category=_coerce_enum(mapping["category"], FailureCategory, "category"),
            code=_require_text(mapping["code"], "code"),
            message=_require_text(mapping["message"], "message"),
            component=_require_text(mapping["component"], "component"),
            details=details,
            retryable=_require_bool(mapping["retryable"], "retryable"),
            fallback_allowed=_require_bool(
                mapping["fallback_allowed"], "fallback_allowed"
            ),
            evidence_valid=_require_bool(mapping["evidence_valid"], "evidence_valid"),
        )


@dataclass(frozen=True, slots=True, init=False)
class HealthEvent:
    """Immutable structured health observation."""

    status: HealthStatus
    severity: HealthSeverity
    component: str
    code: str
    message: str
    details: Mapping[str, SnapshotValue]
    failure: FailureInfo | None
    schema: str

    def __init__(
        self,
        status: HealthStatus,
        severity: HealthSeverity,
        component: str,
        code: str,
        message: str,
        details: Mapping[str, object] | None = None,
        failure: FailureInfo | None = None,
        schema: str = HEALTH_EVENT_SCHEMA,
    ) -> None:
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "severity", severity)
        object.__setattr__(self, "component", component)
        object.__setattr__(self, "code", code)
        object.__setattr__(self, "message", message)
        object.__setattr__(self, "details", details if details is not None else {})
        object.__setattr__(self, "failure", failure)
        object.__setattr__(self, "schema", schema)
        self.__post_init__()

    def __post_init__(self) -> None:
        status = _coerce_canonical_enum(self.status, HealthStatus, "status")
        severity = _coerce_canonical_enum(self.severity, HealthSeverity, "severity")
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "severity", severity)
        object.__setattr__(
            self, "component", _require_text(self.component, "component")
        )
        object.__setattr__(self, "code", _require_text(self.code, "code"))
        object.__setattr__(
            self,
            "message",
            _sanitize_text(_require_text(self.message, "message")),
        )
        object.__setattr__(
            self,
            "details",
            _freeze_details(cast(Mapping[str, object], self.details)),
        )
        schema = _require_plain_text(self.schema, "schema")
        object.__setattr__(self, "schema", schema)
        if schema != HEALTH_EVENT_SCHEMA:
            raise ValueError(
                f"unsupported health schema {schema!r}; expected "
                f"{HEALTH_EVENT_SCHEMA!r}"
            )
        if self.failure is not None and not isinstance(self.failure, FailureInfo):
            raise TypeError("failure must be FailureInfo or None")
        if self.failure is not None and status is HealthStatus.HEALTHY:
            raise ValueError("a failure cannot be represented as healthy")
        if self.failure is not None and status is HealthStatus.DEGRADED:
            if (
                not self.failure.fallback_allowed
                or self.failure.category not in FALLBACK_ALLOWED_CATEGORIES
            ):
                raise ValueError(
                    "degraded health with failure requires "
                    "fallback_allowed=True and an allowed fallback category"
                )

    def __copy__(self) -> HealthEvent:
        return self

    def __deepcopy__(self, memo: dict[int, object]) -> HealthEvent:
        """Return ``self``; frozen fields and nested mappings are immutable.

        ``MappingProxyType`` cannot be pickled, so ``configclass`` deep-copying
        this value during environment setup otherwise fails.
        """
        return self

    def to_mapping(self) -> dict[str, SerializableValue]:
        """Return a fresh deterministic JSON-shaped mapping."""
        result: dict[str, SerializableValue] = {
            "schema": self.schema,
            "status": self.status.value,
            "severity": self.severity.value,
            "component": self.component,
            "code": self.code,
            "message": self.message,
            "details": _thaw_value(self.details),
        }
        if self.failure is not None:
            result["failure"] = self.failure.to_mapping()
        return result

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> HealthEvent:
        """Validate and rebuild a canonical health event mapping."""
        mapping = _string_mapping(value, "health event")
        _reject_unknown_keys(
            mapping,
            frozenset(
                {
                    "schema",
                    "schema_id",
                    "status",
                    "severity",
                    "component",
                    "code",
                    "message",
                    "details",
                    "failure",
                }
            ),
            "health event",
        )
        if "schema" not in mapping and "schema_id" not in mapping:
            raise ValueError("health event missing schema")
        if (
            "schema" in mapping
            and "schema_id" in mapping
            and mapping["schema"] != mapping["schema_id"]
        ):
            raise ValueError("health event schema and schema_id disagree")
        schema = mapping.get("schema", mapping.get("schema_id"))
        if schema != HEALTH_EVENT_SCHEMA:
            raise ValueError(
                f"unsupported health schema {schema!r}; expected "
                f"{HEALTH_EVENT_SCHEMA!r}"
            )
        required = ("status", "severity", "component", "code", "message", "details")
        missing = [key for key in required if key not in mapping]
        if missing:
            raise ValueError(f"health event missing field(s): {', '.join(missing)}")
        details = _string_mapping(mapping["details"], "health event.details")
        failure_value = mapping.get("failure")
        failure: FailureInfo | None
        if failure_value is None:
            failure = None
        else:
            failure = FailureInfo.from_mapping(
                _string_mapping(failure_value, "health event.failure")
            )
        return cls(
            status=_coerce_status(mapping["status"]),
            severity=_coerce_severity(mapping["severity"]),
            component=_require_text(mapping["component"], "component"),
            code=_require_text(mapping["code"], "code"),
            message=_require_text(mapping["message"], "message"),
            details=details,
            failure=failure,
            schema=_require_text(schema, "schema"),
        )

    @classmethod
    def degraded_fallback(
        cls,
        failure: FailureInfo,
        *,
        component: str | None = None,
        code: str = "optional_runtime_fallback",
        message: str | None = None,
        details: Mapping[str, object] | None = None,
    ) -> HealthEvent:
        """Record an explicitly permitted fallback as degraded health."""
        if not failure.fallback_allowed:
            raise ValueError("fallback requires failure.info.fallback_allowed=True")
        if failure.category not in FALLBACK_ALLOWED_CATEGORIES:
            raise ValueError(f"{failure.category.value} cannot use degraded fallback")
        fallback_details: dict[str, object] = {}
        if details is not None:
            fallback_details.update(details)
        fallback_details["fallback_applied"] = True
        return cls(
            status=HealthStatus.DEGRADED,
            severity=HealthSeverity.WARNING,
            component=failure.component if component is None else component,
            code=code,
            message=(
                f"fallback applied: {failure.message}" if message is None else message
            ),
            details=fallback_details,
            failure=failure,
        )


def make_degraded_health_event(
    failure: FailureInfo,
    *,
    component: str | None = None,
    code: str = "optional_runtime_fallback",
    message: str | None = None,
    details: Mapping[str, object] | None = None,
) -> HealthEvent:
    """Apply an explicit fallback and expose it as degraded health."""
    return HealthEvent.degraded_fallback(
        failure,
        component=component,
        code=code,
        message=message,
        details=details,
    )


def normalize_health_event(
    value: object, *, component: str | None = None
) -> HealthEvent | None:
    """Normalize canonical events or legacy hardware mappings.

    Retain legacy fields in ``details`` after JSON-shape and finite-value checks.
    """
    if value is None:
        return None
    if isinstance(value, HealthEvent):
        return value
    mapping = _string_mapping(value, "health event")
    if "schema" in mapping or "schema_id" in mapping:
        return HealthEvent.from_mapping(mapping)

    if "kind" in mapping:
        code = _require_text(mapping["kind"], "legacy health kind")
    elif "code" in mapping:
        code = _require_text(mapping["code"], "legacy health code")
    else:
        raise ValueError("legacy health mapping requires kind")

    if "component" in mapping:
        component_value = mapping["component"]
    elif "interface" in mapping:
        component_value = mapping["interface"]
    else:
        component_value = component
    if component_value is None:
        raise ValueError("legacy health mapping requires component or interface")
    component_name = _require_text(component_value, "component")

    if "message" in mapping:
        message = _require_text(mapping["message"], "message")
    else:
        message = f"legacy health event: {code}"
    status = _coerce_status(mapping.get("status", HealthStatus.DEGRADED))
    severity = _coerce_severity(mapping.get("severity", HealthSeverity.WARNING))
    failure_value = mapping.get("failure")
    if failure_value is None:
        failure = None
    elif isinstance(failure_value, FailureInfo):
        failure = failure_value
    else:
        failure = FailureInfo.from_mapping(
            _string_mapping(failure_value, "legacy health event.failure")
        )
    details = dict(mapping)
    details.pop("failure", None)
    return HealthEvent(
        status=status,
        severity=severity,
        component=component_name,
        code=code,
        message=message,
        details=details,
        failure=failure,
    )


def classify_exception(
    exc: BaseException,
    *,
    category: FailureCategory | None = None,
    code: str | None = None,
    component: str | None = None,
    context: str | Mapping[str, object] | None = None,
    details: Mapping[str, object] | None = None,
    retryable: bool = False,
    fallback_allowed: bool = False,
    evidence_valid: bool = False,
) -> FailureInfo:
    """Classify without inferring from built-in types.

    Typed failures round-trip. Others default to internal-invariant unless
    categorized; ``ValueError`` and ``ImportError`` never permit fallback.
    """
    if isinstance(exc, FailureError):
        return exc.info

    resolved_category = (
        FailureCategory.INTERNAL_INVARIANT
        if category is None
        else _coerce_enum(category, FailureCategory, "category")
    )
    resolved_component: str | None = component
    if resolved_component is None and isinstance(context, str):
        resolved_component = context
    if resolved_component is None:
        resolved_component = "unclassified"

    classified_details: dict[str, object] = {}
    if details is not None:
        classified_details.update(details)
    if context is not None:
        if "context" in classified_details:
            raise ValueError("details already contains context")
        classified_details["context"] = context
    classified_details.setdefault("exception_type", _exception_type_name(exc))

    message = str(exc) or type(exc).__name__
    resolved_code = code or (
        "internal_invariant.unclassified_exception"
        if category is None
        else f"{resolved_category.value}.unclassified"
    )
    return FailureInfo(
        category=resolved_category,
        code=resolved_code,
        message=message,
        component=resolved_component,
        details=classified_details,
        retryable=retryable,
        fallback_allowed=fallback_allowed,
        evidence_valid=evidence_valid,
    )


def _exception_type_name(exc: BaseException) -> str:
    return f"{type(exc).__module__}.{type(exc).__qualname__}"


class FailureError(Exception):
    """Base exception carrying a validated :class:`FailureInfo`."""

    category: ClassVar[FailureCategory] = FailureCategory.INTERNAL_INVARIANT
    default_code: ClassVar[str] = "internal_invariant.failure"
    info: FailureInfo

    def __init__(
        self,
        info: FailureInfo | str | None = None,
        *,
        code: str | None = None,
        message: str | None = None,
        component: str = "unclassified",
        details: Mapping[str, object] | None = None,
        retryable: bool = False,
        fallback_allowed: bool = False,
        evidence_valid: bool = False,
    ) -> None:
        if isinstance(info, FailureInfo):
            if (
                code is not None
                or message is not None
                or component != "unclassified"
                or details is not None
                or retryable
                or fallback_allowed
                or evidence_valid
            ):
                raise TypeError("pass either FailureInfo or constructor fields")
            failure_info = info
        else:
            if info is not None:
                if not isinstance(info, str):
                    raise TypeError("error message must be a string or FailureInfo")
                if message is not None:
                    raise TypeError("message supplied twice")
                message = info
            failure_info = FailureInfo(
                category=self.category,
                code=code or self.default_code,
                message=message or type(self).__name__,
                component=component,
                details=details,
                retryable=retryable,
                fallback_allowed=fallback_allowed,
                evidence_valid=evidence_valid,
            )
        if failure_info.category is not self.category:
            raise ValueError(
                f"{type(self).__name__} requires category {self.category.value!r}"
            )
        self.info = failure_info
        super().__init__(failure_info.message)

    @property
    def failure_info(self) -> FailureInfo:
        """Compatibility spelling for callers that name the payload fully."""
        return self.info


class InvalidInputError(FailureError, ValueError):
    category = FailureCategory.INVALID_INPUT
    default_code = "invalid_input.failure"


class UnsupportedCapabilityError(FailureError, RuntimeError):
    category = FailureCategory.UNSUPPORTED_CAPABILITY
    default_code = "unsupported_capability.failure"


class OptionalDependencyUnavailableError(FailureError, ImportError):
    category = FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE
    default_code = "optional_dependency_unavailable.failure"


class TransientExternalFailureError(FailureError, RuntimeError):
    category = FailureCategory.TRANSIENT_EXTERNAL_FAILURE
    default_code = "transient_external_failure.failure"


class IntegrityFailureError(FailureError, RuntimeError):
    category = FailureCategory.INTEGRITY_FAILURE
    default_code = "integrity_failure.failure"


class SafetyViolationError(FailureError, RuntimeError):
    category = FailureCategory.SAFETY_VIOLATION
    default_code = "safety_violation.failure"


class InternalInvariantError(FailureError, RuntimeError):
    category = FailureCategory.INTERNAL_INVARIANT
    default_code = "internal_invariant.failure"


__all__ = [
    "CALLER_ACTION_CATEGORIES",
    "FALLBACK_ALLOWED_CATEGORIES",
    "HEALTH_EVENT_SCHEMA",
    "FailureCategory",
    "FailureError",
    "FailureInfo",
    "HealthEvent",
    "HealthSeverity",
    "HealthStatus",
    "IntegrityFailureError",
    "InternalInvariantError",
    "InvalidInputError",
    "KitBindingError",
    "OptionalDependencyUnavailableError",
    "SafetyViolationError",
    "TransientExternalFailureError",
    "UnsupportedCapabilityError",
    "attempt_cleanup",
    "classify_exception",
    "make_degraded_health_event",
    "normalize_health_event",
]
