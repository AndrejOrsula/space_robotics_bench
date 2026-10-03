"""Strict validation-session JSONL records and evidence validation."""

from __future__ import annotations

import json
import re
from collections.abc import Iterator, Mapping
from math import isfinite
from pathlib import Path
from typing import IO, Any

from srb.utils.failures import IntegrityFailureError
from srb.utils.publication import ArtifactPublication, PublicationError
from srb.utils.validation_contract import (
    CRASH_TERM_REASONS,
    SESSION_ABORT_TERM_REASONS,
    VALID_TERM_REASONS,
)

EPISODES_JSONL = "episodes.jsonl"

JsonRecord = dict[str, Any]
ValidationEpisodeSteps = dict[int, list[JsonRecord]]
ValidationEpisodeEnds = dict[int, JsonRecord]

_STEP_REQUIRED_FIELDS = frozenset(
    {"ep", "step", "obs_summary", "act", "rew", "info_summary"}
)
_STEP_OPTIONAL_FIELDS = frozenset({"pre_obs_summary", "next_obs_summary"})
# Optional transition snapshots supplement the required step fields.
_EPISODE_END_FIELDS = frozenset({"event", "ep", "success", "term_reason", "ep_metrics"})
_EVENT_FIELDS = frozenset({"event", "payload"})
_VALID_TERM_REASONS = VALID_TERM_REASONS
_VALID_EVENT_KINDS = frozenset(
    {
        "step",
        "health",
        "topic_stale",
        "preflight_failure",
        "validation.health_poll_exception",
        "validation.health_check_exception",
        "validation.health_event_malformed",
        "hardware_event",
        "policy_nan",
        "disk_full",
        "external_abort",
        "timeout",
        *CRASH_TERM_REASONS,
        *SESSION_ABORT_TERM_REASONS,
    }
)
_HARDWARE_EVENT_KIND_RE = re.compile(
    r"[A-Za-z][A-Za-z0-9_]*(?:\.[A-Za-z][A-Za-z0-9_]*)+"
)


def validation_log_path(session_dir_or_path: str | Path) -> Path:
    """Return the validation JSONL path for a session directory or log path."""

    path = Path(session_dir_or_path)
    if path.name == EPISODES_JSONL or path.suffix == ".jsonl":
        return path
    return path / EPISODES_JSONL


def build_step_record(
    *,
    ep: int,
    step: int,
    obs_summary: Mapping[str, Any],
    act: Any,
    reward: float,
    info: Mapping[str, Any],
    pre_obs_summary: Mapping[str, Any] | None = None,
    next_obs_summary: Mapping[str, Any] | None = None,
) -> JsonRecord:
    """Build and validate one step record using the committed JSONL shape."""

    try:
        record = {
            "ep": _require_nonnegative_int(ep, "ep"),
            "step": _require_nonnegative_int(step, "step"),
            "obs_summary": _normalize_json_structure(obs_summary),
            "act": _normalize_json_structure(_action_to_json(act)),
            "rew": _require_finite_number(reward, "rew"),
            "info_summary": {
                key: _normalize_json_structure(
                    float(value) if isinstance(value, float) else value
                )
                for key, value in info.items()
                if isinstance(value, (int, float, bool, str))
            },
        }
        for name, value in (
            ("pre_obs_summary", pre_obs_summary),
            ("next_obs_summary", next_obs_summary),
        ):
            if value is not None:
                if not isinstance(value, Mapping):
                    raise TypeError(f"{name} must be an object")
                record[name] = _normalize_json_structure(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise _integrity_failure(
            f"invalid validation step record: {exc}", cause=exc
        ) from exc
    return validate_validation_record(record)


def build_episode_end_record(
    *,
    ep: int,
    success: bool,
    term_reason: str,
    ep_metrics: Mapping[str, float],
) -> JsonRecord:
    """Build and validate one episode-end record.

    ``success`` is deliberately checked before any conversion.  In
    particular, ``bool("false")`` is not an admissible evidence path.
    """

    try:
        record = {
            "event": "episode_end",
            "ep": _require_nonnegative_int(ep, "ep"),
            "success": _require_bool(success, "success"),
            "term_reason": _require_term_reason(term_reason),
            "ep_metrics": _coerce_metrics(ep_metrics, context="ep_metrics"),
        }
        if record["success"] != (record["term_reason"] == "success"):
            raise ValueError("success must agree with the episode term_reason")
    except (TypeError, ValueError, OverflowError) as exc:
        raise _integrity_failure(
            f"invalid validation episode-end record: {exc}", cause=exc
        ) from exc
    return validate_validation_record(record)


def build_event_record(kind: str, payload: Mapping[str, Any]) -> JsonRecord:
    """Build and validate one event record using the committed JSONL shape."""

    try:
        if type(kind) is not str or not kind.strip() or kind == "episode_end":
            raise ValueError(
                "validation event kind must be a non-empty non-episode string"
            )
        if not isinstance(payload, Mapping):
            raise TypeError("validation event payload must be a mapping")
        return validate_validation_record(
            {"event": kind, "payload": _normalize_json_structure(payload)}
        )
    except IntegrityFailureError:
        raise
    except (TypeError, ValueError, OverflowError) as exc:
        raise _integrity_failure(
            f"invalid validation event record: {exc}", cause=exc
        ) from exc


def write_validation_record(fh: IO[str], record: Mapping[str, Any]) -> None:
    """Append one strictly validated validation record to an open JSONL handle."""

    validated = validate_validation_record(record)
    fh.write(json.dumps(validated, allow_nan=False) + "\n")


def iter_validation_records(session_dir_or_path: str | Path) -> Iterator[JsonRecord]:
    """Yield schema-validated records; malformed evidence raises an integrity error."""

    episodes_path = validation_log_path(session_dir_or_path)
    if not episodes_path.exists():
        raise FileNotFoundError(f"validation telemetry not found: {episodes_path}")
    try:
        payload = ArtifactPublication.file(episodes_path).read(
            lambda raw: raw.decode("utf-8")
        )
    except (OSError, PublicationError, UnicodeDecodeError) as exc:
        raise _integrity_failure(
            f"could not read validation telemetry: {episodes_path}",
            cause=exc,
        ) from exc
    if payload is None:
        raise FileNotFoundError(f"validation telemetry not found: {episodes_path}")
    for line_number, line in enumerate(payload.splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            record = json.loads(stripped, object_pairs_hook=_unique_json_object)
        except (
            json.JSONDecodeError,
            UnicodeDecodeError,
            RecursionError,
            ValueError,
        ) as exc:
            raise _integrity_failure(
                f"invalid validation telemetry JSON at {episodes_path}:{line_number}",
                cause=exc,
            ) from exc
        try:
            yield validate_validation_record(
                record, source=f"{episodes_path}:{line_number}"
            )
        except IntegrityFailureError:
            raise
        except (TypeError, ValueError, OverflowError, RecursionError) as exc:
            raise _integrity_failure(
                f"invalid validation telemetry record at "
                f"{episodes_path}:{line_number}: {exc}",
                cause=exc,
            ) from exc


def validate_validation_record(
    record: object,
    *,
    source: str = "validation record",
) -> JsonRecord:
    """Validate one in-memory or decoded JSONL record."""

    if not isinstance(record, Mapping):
        raise _integrity_failure(f"{source} must be a JSON object")
    candidate = dict(record)
    try:
        for key, value in candidate.items():
            if type(key) is not str:
                raise _integrity_failure(f"{source} keys must be strings")
            _validate_json_value(value, context=f"{source}.{key}")
        if "event" not in candidate:
            _validate_step_record(candidate, source=source)
        else:
            event = candidate["event"]
            if type(event) is not str:
                raise _integrity_failure(f"{source}.event must be a string")
            if event == "episode_end":
                _validate_episode_end_record(candidate, source=source)
            else:
                _validate_event_record(candidate, source=source)
    except IntegrityFailureError:
        raise
    except (TypeError, ValueError, OverflowError) as exc:
        raise _integrity_failure(f"{source} is malformed: {exc}", cause=exc) from exc
    return candidate


def iter_episode_end_records(
    session_dir_or_path: str | Path,
) -> Iterator[JsonRecord]:
    """Yield unique, strictly validated episode-end records."""

    seen: set[int] = set()
    for record in iter_validation_records(session_dir_or_path):
        if record.get("event") != "episode_end":
            continue
        ep = record["ep"]
        if type(ep) is not int:
            raise TypeError("type(ep) is int is false")
        if ep in seen:
            raise _integrity_failure(f"duplicate episode-end record for ep={ep}")
        seen.add(ep)
        yield record


def read_validation_episode_records(
    session_dir_or_path: str | Path,
) -> tuple[ValidationEpisodeSteps, ValidationEpisodeEnds]:
    """Group strictly validated step and episode-end records by episode."""

    episodes: ValidationEpisodeSteps = {}
    episode_end: ValidationEpisodeEnds = {}
    for record in iter_validation_records(session_dir_or_path):
        if record.get("event") == "episode_end":
            ep = record["ep"]
            if type(ep) is not int:
                raise TypeError("type(ep) is int is false")
            if ep in episode_end:
                raise _integrity_failure(f"duplicate episode-end record for ep={ep}")
            episode_end[ep] = record
            continue
        if "event" in record:
            continue
        ep = record["ep"]
        step = record["step"]
        if not (type(ep) is int and type(step) is int):
            raise TypeError("type(ep) is int and type(step) is int is false")
        if ep in episode_end:
            raise _integrity_failure(
                f"step record follows episode-end record for ep={ep}"
            )
        if any(existing["step"] == step for existing in episodes.get(ep, [])):
            raise _integrity_failure(f"duplicate step record for ep={ep}, step={step}")
        episodes.setdefault(ep, []).append(record)
    return episodes, episode_end


def _validate_step_record(record: Mapping[str, Any], *, source: str) -> None:
    _require_fields(
        record,
        required=_STEP_REQUIRED_FIELDS,
        optional=_STEP_OPTIONAL_FIELDS,
        source=source,
    )
    _require_nonnegative_int(record["ep"], f"{source}.ep")
    _require_nonnegative_int(record["step"], f"{source}.step")
    if not isinstance(record["obs_summary"], Mapping):
        raise _integrity_failure(f"{source}.obs_summary must be an object")
    for field_name in ("pre_obs_summary", "next_obs_summary"):
        if field_name in record and not isinstance(record[field_name], Mapping):
            raise _integrity_failure(f"{source}.{field_name} must be an object")
    if not isinstance(record["info_summary"], Mapping):
        raise _integrity_failure(f"{source}.info_summary must be an object")
    if not isinstance(record["act"], list):
        raise _integrity_failure(f"{source}.act must be an array")
    _require_finite_number(record["rew"], f"{source}.rew")


def _validate_episode_end_record(record: Mapping[str, Any], *, source: str) -> None:
    _require_exact_fields(record, _EPISODE_END_FIELDS, source)
    _require_nonnegative_int(record["ep"], f"{source}.ep")
    _require_bool(record["success"], f"{source}.success")
    term_reason = _require_term_reason(record["term_reason"])
    if record["success"] != (term_reason == "success"):
        raise _integrity_failure(
            f"{source}.success disagrees with {source}.term_reason"
        )
    _coerce_metrics(record["ep_metrics"], context=f"{source}.ep_metrics")


def _validate_event_record(record: Mapping[str, Any], *, source: str) -> None:
    _require_exact_fields(record, _EVENT_FIELDS, source)
    kind = record["event"]
    if type(kind) is not str or not kind.strip():
        raise _integrity_failure(f"{source}.event must be a non-empty string")
    if kind not in _VALID_EVENT_KINDS and not _HARDWARE_EVENT_KIND_RE.fullmatch(kind):
        raise _integrity_failure(f"{source}.event is not a canonical event kind")
    if not isinstance(record["payload"], Mapping):
        raise _integrity_failure(f"{source}.payload must be an object")


def _require_exact_fields(
    record: Mapping[str, Any], expected: frozenset[str], source: str
) -> None:
    actual = set(record)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        detail = []
        if missing:
            detail.append(f"missing={missing}")
        if extra:
            detail.append(f"extra={extra}")
        raise _integrity_failure(f"{source} has invalid fields ({', '.join(detail)})")


def _require_fields(
    record: Mapping[str, Any],
    *,
    required: frozenset[str],
    optional: frozenset[str],
    source: str,
) -> None:
    actual = set(record)
    missing = sorted(required - actual)
    extra = sorted(actual - required - optional)
    if missing or extra:
        detail = []
        if missing:
            detail.append(f"missing={missing}")
        if extra:
            detail.append(f"extra={extra}")
        raise _integrity_failure(f"{source} has invalid fields ({', '.join(detail)})")


def _require_nonnegative_int(value: object, field_name: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")
    return value


def _require_bool(value: object, field_name: str) -> bool:
    if type(value) is not bool:
        raise TypeError(f"{field_name} must be a boolean")
    return value


def _require_term_reason(value: object) -> str:
    if type(value) is not str or value not in _VALID_TERM_REASONS:
        raise ValueError(f"unsupported validation term_reason: {value!r}")
    return value


def _require_finite_number(value: object, field_name: str) -> float:
    if type(value) is int:
        numeric = float(value)
    elif type(value) is float:
        numeric = value
    else:
        raise TypeError(f"{field_name} must be an int or float, not bool/string")
    if not isfinite(numeric):
        raise ValueError(f"{field_name} must be finite")
    return numeric


def _coerce_metrics(value: object, *, context: str) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{context} must be an object")
    metrics: dict[str, float] = {}
    for key, item in value.items():
        if type(key) is not str or not key:
            raise TypeError(f"{context} keys must be non-empty strings")
        metrics[key] = _require_finite_number(item, f"{context}.{key}")
    return metrics


def _validate_json_value(value: object, *, context: str) -> None:
    if value is None or type(value) in (bool, str):
        return
    if type(value) in (int, float):
        _require_finite_number(value, context)
        return
    if isinstance(value, Mapping):
        for key, item in value.items():
            if type(key) is not str:
                raise _integrity_failure(f"{context} object keys must be strings")
            _validate_json_value(item, context=f"{context}.{key}")
        return
    if isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _validate_json_value(item, context=f"{context}[{index}]")
        return
    raise _integrity_failure(
        f"{context} contains unsupported value type {type(value).__name__}"
    )


def _integrity_failure(
    message: str,
    *,
    cause: BaseException | None = None,
) -> IntegrityFailureError:
    details: dict[str, object] = {}
    if cause is not None:
        details["cause"] = type(cause).__name__
    error = IntegrityFailureError(
        code="validation.session_log.integrity",
        message=message,
        component="validation.session_log",
        details=details,
        evidence_valid=False,
    )
    if cause is not None:
        error.__cause__ = cause
    return error


def _action_to_json(act: Any) -> Any:
    if hasattr(act, "tolist"):
        return act.tolist()
    return act


def _normalize_json_structure(value: Any) -> Any:
    """Convert nested mapping containers and array-like values to JSON shapes."""
    if isinstance(value, Mapping):
        return {key: _normalize_json_structure(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_normalize_json_structure(item) for item in value]
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _normalize_json_structure(tolist())
    return value


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


__all__ = [
    "EPISODES_JSONL",
    "JsonRecord",
    "ValidationEpisodeEnds",
    "ValidationEpisodeSteps",
    "build_episode_end_record",
    "build_event_record",
    "build_step_record",
    "iter_episode_end_records",
    "iter_validation_records",
    "read_validation_episode_records",
    "validate_validation_record",
    "validation_log_path",
    "write_validation_record",
]
