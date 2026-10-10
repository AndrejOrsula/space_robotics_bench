"""Validation report-index loading and fail-closed rendering."""

from __future__ import annotations

import json
import re
from math import isfinite
from pathlib import Path
from typing import Any

from srb.interfaces.sim_to_real.validation.exit_codes import VALIDATION_EXIT_CODES
from srb.interfaces.sim_to_real.validation.report import write_index_html
from srb.utils.failures import IntegrityFailureError
from srb.utils.publication import ArtifactPublication, PublicationError

_REQUIRED_ROW_FIELDS = frozenset(
    {"timestamp", "task", "algo", "exit_code", "session_dir", "metrics"}
)
# The dashboard links ``session_dir`` in an ``href``; HTML escaping does not
# neutralize a ``javascript:`` (or any other) URL scheme.
_URL_SCHEME = re.compile(r"^[A-Za-z][A-Za-z0-9+.\-]*:")
_OPTIONAL_ROW_FIELDS = frozenset(
    {
        "git_sha",
        "session_id",
        "algorithm",
        "code_sha",
        "config_digest",
        "artifact_digest",
    }
)
_VALID_EXIT_CODES = frozenset(item.code for item in VALIDATION_EXIT_CODES)


def load_index_rows(index_path: str | Path) -> list[dict[str, Any]]:
    """Load index rows, failing closed on any malformed non-blank line."""

    path = Path(index_path)
    if not path.exists():
        return []
    if not path.is_file():
        raise _integrity_failure(f"validation report index is not a file: {path}")
    try:
        payload = ArtifactPublication.file(path).read(lambda raw: raw.decode("utf-8"))
    except (OSError, PublicationError, UnicodeDecodeError) as exc:
        raise _integrity_failure(
            f"could not read validation report index: {path}", cause=exc
        ) from exc
    if payload is None:
        raise _integrity_failure(f"validation report index disappeared: {path}")

    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(payload.splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            row = json.loads(stripped)
        except json.JSONDecodeError as exc:
            raise _integrity_failure(
                f"invalid validation report index JSON at {path}:{line_number}",
                cause=exc,
            ) from exc
        try:
            _validate_row(row, source=f"{path}:{line_number}")
        except IntegrityFailureError:
            raise
        rows.append(row)
    return rows


def write_validation_index(
    index_path: str | Path,
    out_path: str | Path,
    *,
    metric_keys: list[str],
) -> None:
    """Render the static index only after its source rows validate."""

    if any(type(key) is not str or not key for key in metric_keys):
        raise _integrity_failure("validation report metric keys must be strings")
    write_index_html(
        load_index_rows(index_path),
        Path(out_path),
        metric_keys=metric_keys,
    )


def _validate_row(value: object, *, source: str) -> None:
    if not isinstance(value, dict):
        raise _integrity_failure(f"{source} must be a JSON object")
    fields = set(value)
    missing = sorted(_REQUIRED_ROW_FIELDS - fields)
    unknown = sorted(fields - _REQUIRED_ROW_FIELDS - _OPTIONAL_ROW_FIELDS)
    if missing or unknown:
        detail = []
        if missing:
            detail.append(f"missing={missing}")
        if unknown:
            detail.append(f"unknown={unknown}")
        raise _integrity_failure(f"{source} has invalid fields ({', '.join(detail)})")
    for field in ("timestamp", "task", "algo", "session_dir"):
        if type(value[field]) is not str or not value[field].strip():
            raise _integrity_failure(f"{source}.{field} must be a non-empty string")
    # Browsers drop tab/newline and leading control/space characters before
    # parsing a scheme, so check the value the way they will read it.
    href = re.sub(r"[\t\n\r]", "", value["session_dir"]).lstrip(
        "".join(map(chr, range(0x21)))
    )
    if _URL_SCHEME.match(href):
        raise _integrity_failure(f"{source}.session_dir must be a filesystem path")
    exit_code = value["exit_code"]
    if type(exit_code) is not int or exit_code not in _VALID_EXIT_CODES:
        raise _integrity_failure(f"{source}.exit_code is invalid")
    metrics = value["metrics"]
    if not isinstance(metrics, dict):
        raise _integrity_failure(f"{source}.metrics must be an object")
    for key, item in metrics.items():
        if type(key) is not str or not key:
            raise _integrity_failure(f"{source}.metrics keys must be strings")
        if type(item) not in (int, float):
            raise _integrity_failure(
                f"{source}.metrics.{key} must be an exact int or float"
            )
        try:
            finite = isfinite(float(item))
        except (OverflowError, TypeError, ValueError) as exc:
            raise _integrity_failure(
                f"{source}.metrics.{key} is not finite", cause=exc
            ) from exc
        if not finite:
            raise _integrity_failure(f"{source}.metrics.{key} is non-finite")
    for field in _OPTIONAL_ROW_FIELDS:
        if field in value and (
            type(value[field]) is not str or not value[field].strip()
        ):
            raise _integrity_failure(f"{source}.{field} must be a non-empty string")


def _integrity_failure(
    message: str,
    *,
    cause: BaseException | None = None,
) -> IntegrityFailureError:
    details: dict[str, object] = {}
    if cause is not None:
        details["cause"] = type(cause).__name__
    return IntegrityFailureError(
        code="validation.report_index.integrity",
        message=message,
        component="validation.report_index",
        details=details,
        evidence_valid=False,
    )
