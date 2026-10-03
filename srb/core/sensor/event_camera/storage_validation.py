"""Canonical EVENT_HDF5 schema validation."""

from __future__ import annotations

import json
import math
import operator
from dataclasses import dataclass, field, fields, replace
from numbers import Integral, Real
from pathlib import Path
from typing import Any, NoReturn, SupportsIndex, cast

from srb.core.sensor.event_camera.storage import (
    _JSON_ENCODED_METADATA_FIELDS,
    EVENT_HDF5_COLUMNS,
    EVENT_HDF5_VERSION,
    EventEpisodeMetadata,
    EventHDF5WriterTelemetry,
    _check_event_full_read_budget,
    _iter_event_hdf5_handle,
    _read_event_hdf5_handle,
    _require_event_hdf5_file_size,
    _validate_hdf5_metadata_storage,
)
from srb.utils.failures import (
    FailureCategory,
    FailureError,
    FailureInfo,
    IntegrityFailureError,
    InternalInvariantError,
    InvalidInputError,
    OptionalDependencyUnavailableError,
    SafetyViolationError,
    TransientExternalFailureError,
    UnsupportedCapabilityError,
)
from srb.utils.publication import ArtifactPublication, PublicationError
from srb.utils.resource_budget import MAX_EVENT_BATCH_ROWS, MAX_EVENT_ROWS

_VALIDATION_COMPONENT = "event_camera.storage_validation"
_DEFAULT_VALIDATION_CHUNK_SIZE = 4096
# The canonical file has four event columns and no metadata members; allow
# additive members without walking an unbounded group before rejecting it.
_MAX_EVENT_GROUP_MEMBERS = 64
_UNCONTAINED_STORAGE_MARKER = "outside the snapshot"

# These attrs were added after the v1 metadata schema. An older file
# may omit them, but a file that includes them must still carry valid JSON.
_OPTIONAL_METADATA_ATTRS = frozenset(
    ("hot_pixel_mask", "hf_repo_ids", "hot_pixel_rate_hz")
)
# Telemetry is JSON text; an int field must not decode as a float.
_WRITER_TELEMETRY_TYPES: dict[str, type] = {
    item.name: int if item.type in ("int", int) else float
    for item in fields(EventHDF5WriterTelemetry)
}


def _register_hdf5_filters() -> None:
    """Best-effort hdf5plugin import so Blosc datasets decode in this process."""
    try:
        import hdf5plugin  # noqa: F401 - import registers the HDF5 filters
    except ImportError:
        pass


def _failure_details(
    path: object,
    *,
    issues: tuple[str, ...] = (),
    dependency: str | None = None,
    cause: BaseException | None = None,
) -> dict[str, object]:
    details: dict[str, object] = {"source": _path_text(path)}
    if issues:
        details["issues"] = list(issues[:3])
        details["issue_count"] = len(issues)
    if dependency is not None:
        details["dependency"] = dependency
    if cause is not None:
        details["cause_type"] = f"{type(cause).__module__}.{type(cause).__qualname__}"
    return details


def _path_text(path: object) -> str:
    try:
        return str(path)
    except Exception:  # noqa: BLE001 - diagnostics must not mask the boundary
        return f"<{type(path).__module__}.{type(path).__qualname__}>"


def _schema_integrity_failure(
    path: Path,
    issues: tuple[str, ...],
) -> FailureInfo:
    preview = "; ".join(issues[:3])
    extra = "" if len(issues) <= 3 else f"; +{len(issues) - 3} more"
    failure_kind = _failure_kind_for_issues(issues)
    code = (
        "event_hdf5.validation.integrity"
        if failure_kind == "schema_mismatch"
        else f"event_hdf5.validation.{failure_kind}"
    )
    details = _failure_details(path, issues=issues)
    details["failure_kind"] = failure_kind
    return FailureInfo(
        category=FailureCategory.INTEGRITY_FAILURE,
        code=code,
        message=(f"EVENT_HDF5 schema validation failed for {path}: {preview}{extra}"),
        component=_VALIDATION_COMPONENT,
        details=details,
    )


def _failure_kind_for_issues(issues: tuple[str, ...]) -> str:
    completion_attrs = (
        "event_count",
        "event_sha256",
        "compression",
        "dropped_batches",
    )
    if any(
        f"metadata missing required attr {attr}" in issue
        for issue in issues
        for attr in completion_attrs
    ):
        return "incomplete_publication"
    if any(_UNCONTAINED_STORAGE_MARKER in issue for issue in issues):
        return "uncontained_storage"
    if any("event_sha256" in issue for issue in issues):
        return "digest_metadata"
    if any(
        "does not match" in issue and ("event_count" in issue or "events/" in issue)
        for issue in issues
    ):
        return "count_mismatch"
    if any(
        marker in issue
        for issue in issues
        for marker in (
            "non-finite",
            "timestamps are not monotonically",
            "timestamps must be non-negative",
            "polarity values outside",
            "coordinates must be in",
        )
    ):
        return "invalid_event_data"
    return "schema_mismatch"


def _typed_integrity_failure(
    path: Path,
    *,
    kind: str,
    message: str,
    cause: BaseException | None = None,
) -> FailureInfo:
    details = _failure_details(path, cause=cause)
    details["failure_kind"] = kind
    return FailureInfo(
        category=FailureCategory.INTEGRITY_FAILURE,
        code=f"event_hdf5.validation.{kind}",
        message=message,
        component=_VALIDATION_COMPONENT,
        details=details,
    )


def _invalid_path_failure(
    path: object,
    message: str,
    *,
    cause: BaseException | None = None,
) -> FailureInfo:
    return FailureInfo(
        category=FailureCategory.INVALID_INPUT,
        code="event_hdf5.validation.invalid_path",
        message=message,
        component=_VALIDATION_COMPONENT,
        details=_failure_details(path, cause=cause),
    )


def _integrity_failure(
    path: Path,
    message: str,
    *,
    cause: BaseException | None = None,
) -> FailureInfo:
    return FailureInfo(
        category=FailureCategory.INTEGRITY_FAILURE,
        code="event_hdf5.validation.integrity",
        message=message,
        component=_VALIDATION_COMPONENT,
        details=_failure_details(path, cause=cause),
    )


def _optional_dependency_failure(
    path: Path,
    dependency: str,
    message: str,
    *,
    cause: BaseException | None = None,
) -> FailureInfo:
    return FailureInfo(
        category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
        code="event_hdf5.validation.optional_dependency",
        message=message,
        component=_VALIDATION_COMPONENT,
        details=_failure_details(path, dependency=dependency, cause=cause),
    )


def _internal_failure(
    path: Path,
    message: str,
    *,
    cause: BaseException | None = None,
) -> FailureInfo:
    return FailureInfo(
        category=FailureCategory.INTERNAL_INVARIANT,
        code="event_hdf5.validation.internal",
        message=message,
        component=_VALIDATION_COMPONENT,
        details=_failure_details(path, cause=cause),
    )


def _error_for_failure(failure: FailureInfo) -> FailureError:
    if failure.category is FailureCategory.INVALID_INPUT:
        return InvalidInputError(failure)
    if failure.category is FailureCategory.UNSUPPORTED_CAPABILITY:
        return UnsupportedCapabilityError(failure)
    if failure.category is FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE:
        return OptionalDependencyUnavailableError(failure)
    if failure.category is FailureCategory.TRANSIENT_EXTERNAL_FAILURE:
        return TransientExternalFailureError(failure)
    if failure.category is FailureCategory.INTEGRITY_FAILURE:
        return IntegrityFailureError(failure)
    if failure.category is FailureCategory.SAFETY_VIOLATION:
        return SafetyViolationError(failure)
    return InternalInvariantError(failure)


def _raise_failure(
    failure: FailureInfo,
    *,
    cause: BaseException | None = None,
) -> NoReturn:
    error = _error_for_failure(failure)
    if cause is None:
        raise error
    raise error from cause


@dataclass(frozen=True)
class EventHDF5ValidationReport:
    """Schema-validation result for one canonical EVENT_HDF5 file."""

    path: Path
    issues: tuple[str, ...] = ()
    event_count: int | None = None
    metadata_event_count: int | None = None
    metadata_event_sha256: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)
    legacy_compatible: bool = False
    failure_info: FailureInfo | None = None

    @property
    def valid(self) -> bool:
        return not self.issues

    @property
    def failure(self) -> FailureInfo | None:
        """Compatibility spelling for callers that use ``failure``."""
        return self.failure_info

    def raise_for_errors(self) -> None:
        if self.valid:
            return
        failure = self.failure_info or _schema_integrity_failure(self.path, self.issues)
        _raise_failure(failure)


def _invalid_path_report(
    path: object, cause: BaseException
) -> EventHDF5ValidationReport:
    message = f"EVENT_HDF5 path is invalid: {_path_text(path)} ({cause})"
    return EventHDF5ValidationReport(
        path=Path("<invalid-event-hdf5-path>"),
        issues=(message,),
        failure_info=_invalid_path_failure(path, message, cause=cause),
    )


def _invalid_chunk_report(
    path: Path | str,
    chunk_size: object,
    cause: BaseException | None = None,
) -> EventHDF5ValidationReport:
    chunk_size_text = repr(chunk_size)
    message = (
        "EVENT_HDF5 chunk_size must be a positive integer no greater than "
        f"{MAX_EVENT_BATCH_ROWS}; got {chunk_size_text}"
    )
    failure = FailureInfo(
        category=FailureCategory.INVALID_INPUT,
        code="event_hdf5.validation.invalid_chunk_bound",
        message=message,
        component=_VALIDATION_COMPONENT,
        details={
            **_failure_details(path, cause=cause),
            "chunk_size": chunk_size_text,
            "failure_kind": "invalid_chunk_bound",
        },
    )
    try:
        report_path = Path(path)
    except (TypeError, ValueError):
        report_path = Path("<invalid-event-hdf5-path>")
    return EventHDF5ValidationReport(
        path=report_path,
        issues=(message,),
        failure_info=failure,
    )


def _coerce_chunk_bound(value: object) -> int:
    if isinstance(value, bool):
        raise TypeError("bool is not a chunk bound")
    try:
        bound = operator.index(cast(SupportsIndex, value))
    except (TypeError, ValueError, OverflowError) as exc:
        raise TypeError("chunk_size must be a positive integer") from exc
    if bound <= 0 or bound > MAX_EVENT_BATCH_ROWS:
        raise ValueError(
            "chunk_size must be a positive integer no greater than "
            f"{MAX_EVENT_BATCH_ROWS}"
        )
    return bound


def _classify_stream_failure(path: Path, exc: FailureError) -> FailureInfo:
    failure = exc.failure_info
    message = failure.message
    if failure.code == "event_hdf5.validation.integrity" and message in (
        f"EVENT_HDF5 digest validation failed for {path}",
        "event_sha256 does not match event payload",
    ):
        return _typed_integrity_failure(
            path,
            kind="digest_mismatch",
            message=message,
            cause=exc,
        )
    if "event count" in message or "event_count" in message:
        return _typed_integrity_failure(
            path,
            kind="count_mismatch",
            message=message,
            cause=exc,
        )
    return failure


def validate_event_hdf5_schema(
    path: Path | str,
    *,
    chunk_size: int = _DEFAULT_VALIDATION_CHUNK_SIZE,
    allow_legacy_without_digest: bool = False,
) -> EventHDF5ValidationReport:
    """Validate one published EVENT_HDF5 file using bounded event chunks.

    Strict validation requires the final ``event_count`` and ``event_sha256``
    metadata written by the current publisher and consumes
    :func:`iter_event_hdf5` to verify that contract. Schema-v1 files created
    before digest metadata existed may opt into bounded legacy validation via
    ``allow_legacy_without_digest=True``; invalid digest metadata never falls
    back to that compatibility path.
    """
    try:
        hdf5_path = Path(path)
    except (TypeError, ValueError) as exc:
        return _invalid_path_report(path, exc)
    try:
        chunk_bound = _coerce_chunk_bound(chunk_size)
    except (TypeError, ValueError, OverflowError) as exc:
        return _invalid_chunk_report(path, chunk_size, exc)
    try:
        import h5py
        import numpy as np
    except ImportError as exc:
        dependency = getattr(exc, "name", None) or "h5py/numpy"
        return EventHDF5ValidationReport(
            path=hdf5_path,
            issues=(f"EVENT_HDF5 schema validation requires h5py and numpy: {exc}",),
            failure_info=_optional_dependency_failure(
                hdf5_path,
                dependency,
                f"EVENT_HDF5 schema validation requires h5py and numpy: {exc}",
                cause=exc,
            ),
        )
    _register_hdf5_filters()

    try:
        with ArtifactPublication.file(hdf5_path).open_snapshot() as snapshot:
            if snapshot is None:
                return EventHDF5ValidationReport(
                    path=hdf5_path,
                    issues=(
                        f"not a readable EVENT_HDF5 file: {hdf5_path} does not exist",
                    ),
                    failure_info=_invalid_path_failure(
                        hdf5_path,
                        f"not a readable EVENT_HDF5 file: {hdf5_path} does not exist",
                    ),
                )
            _require_event_hdf5_file_size(snapshot.file, path=hdf5_path)
            with h5py.File(snapshot.file, "r") as h5:
                report = _validate_event_hdf5_handle(
                    h5py=h5py,
                    np=np,
                    h5=h5,
                    path=hdf5_path,
                    chunk_size=chunk_bound,
                    allow_legacy_without_digest=allow_legacy_without_digest,
                )
                if not report.valid or report.legacy_compatible:
                    return report

                streamed_event_count = 0
                for chunk in _iter_event_hdf5_handle(
                    h5,
                    np,
                    path=hdf5_path,
                    chunk_size=chunk_bound,
                ):
                    chunk_lengths = {
                        name: int(values.shape[0]) for name, values in chunk.items()
                    }
                    if len(set(chunk_lengths.values())) != 1:
                        issues = (
                            f"streamed event chunk has unequal lengths {chunk_lengths}",
                        )
                        return replace(
                            report,
                            issues=issues,
                            failure_info=_schema_integrity_failure(hdf5_path, issues),
                        )
                    streamed_event_count += next(iter(chunk_lengths.values()))

                if report.metadata_event_count != streamed_event_count:
                    issues = (
                        "metadata event_count "
                        f"{report.metadata_event_count} does not match "
                        f"streamed event count {streamed_event_count}",
                    )
                    return replace(
                        report,
                        issues=issues,
                        event_count=streamed_event_count,
                        failure_info=_schema_integrity_failure(hdf5_path, issues),
                    )
                return replace(report, event_count=streamed_event_count)
    except FailureError as exc:
        failure = _classify_stream_failure(hdf5_path, exc)
        return EventHDF5ValidationReport(
            path=hdf5_path,
            issues=(failure.message,),
            failure_info=failure,
        )
    except PublicationError as exc:
        message = f"EVENT_HDF5 publication is incomplete for {hdf5_path}: {exc}"
        return EventHDF5ValidationReport(
            path=hdf5_path,
            issues=(message,),
            failure_info=_typed_integrity_failure(
                hdf5_path,
                kind="incomplete_publication",
                message=message,
                cause=exc,
            ),
        )
    except (OSError, ValueError) as exc:
        message = f"not a readable EVENT_HDF5 file: {exc}"
        return EventHDF5ValidationReport(
            path=hdf5_path,
            issues=(message,),
            failure_info=_integrity_failure(hdf5_path, message, cause=exc),
        )
    except Exception as exc:  # noqa: BLE001 - public report is non-throwing
        message = f"EVENT_HDF5 validation failed internally: {exc}"
        return EventHDF5ValidationReport(
            path=hdf5_path,
            issues=(message,),
            failure_info=_internal_failure(hdf5_path, message, cause=exc),
        )


def read_validated_event_hdf5(
    path: Path | str,
    *,
    chunk_size: int = _DEFAULT_VALIDATION_CHUNK_SIZE,
    allow_legacy_without_digest: bool = False,
) -> dict[str, Any]:
    """Stream-validate one publication, then materialize its event arrays."""
    try:
        hdf5_path = Path(path)
    except (TypeError, ValueError) as exc:
        report = _invalid_path_report(path, exc)
        assert report.failure_info is not None
        _raise_failure(report.failure_info, cause=exc)
    try:
        chunk_bound = _coerce_chunk_bound(chunk_size)
    except (TypeError, ValueError, OverflowError) as exc:
        report = _invalid_chunk_report(hdf5_path, chunk_size, exc)
        assert report.failure_info is not None
        _raise_failure(report.failure_info, cause=exc)
    try:
        import h5py
        import numpy as np
    except ImportError as exc:
        dependency = getattr(exc, "name", None) or "h5py/numpy"
        _raise_failure(
            _optional_dependency_failure(
                hdf5_path,
                dependency,
                f"EVENT_HDF5 reading requires h5py and numpy: {exc}",
                cause=exc,
            ),
            cause=exc,
        )
    _register_hdf5_filters()

    try:
        with ArtifactPublication.file(hdf5_path).open_snapshot() as snapshot:
            if snapshot is None:
                _raise_failure(
                    _invalid_path_failure(
                        hdf5_path,
                        f"EVENT_HDF5 schema validation failed for {hdf5_path}: "
                        "file does not exist",
                    )
                )
            _require_event_hdf5_file_size(snapshot.file, path=hdf5_path)
            with h5py.File(snapshot.file, "r") as h5:
                return _read_validated_event_hdf5_handle(
                    h5py=h5py,
                    np=np,
                    h5=h5,
                    path=hdf5_path,
                    chunk_size=chunk_bound,
                    allow_legacy_without_digest=allow_legacy_without_digest,
                )
    except FailureError:
        raise
    except (OSError, ValueError, PublicationError) as exc:
        _raise_failure(
            _integrity_failure(
                hdf5_path,
                f"not a readable EVENT_HDF5 file: {hdf5_path}: {exc}",
                cause=exc,
            ),
            cause=exc,
        )
    except Exception as exc:  # noqa: BLE001 - convert the owned public seam
        _raise_failure(
            _internal_failure(
                hdf5_path,
                f"EVENT_HDF5 reading failed internally: {exc}",
                cause=exc,
            ),
            cause=exc,
        )


def _require_event_mapping(value: object) -> dict[str, Any]:
    """Narrow the legacy HDF5 reader at its typed publication seam."""

    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise TypeError("EVENT_HDF5 reader must return a string-keyed mapping")
    return value


def _read_validated_event_hdf5_handle(
    *,
    h5py: Any,
    np: Any,
    h5: Any,
    path: Path,
    chunk_size: int = _DEFAULT_VALIDATION_CHUNK_SIZE,
    allow_legacy_without_digest: bool = False,
) -> dict[str, Any]:
    """Validate through bounded chunks before materializing an open handle."""

    # Nothing may be read before every member is known to live in this file.
    storage_issues = _uncontained_storage_issues(h5py, h5)
    if storage_issues:
        _raise_failure(_schema_integrity_failure(path, storage_issues))
    # Shape/dtype/decoded-byte checks must precede the bounded validation scan:
    # a full-read request over the materialization budget fails without even
    # allocating one validation chunk.
    _check_event_full_read_budget(h5, np, path=path)
    report = _validate_event_hdf5_handle(
        h5py=h5py,
        np=np,
        h5=h5,
        path=path,
        chunk_size=chunk_size,
        allow_legacy_without_digest=allow_legacy_without_digest,
    )
    report.raise_for_errors()
    # Recheck the already-open descriptor's structure before the explicit
    # materialization phase. The canonical digest was verified above through
    # the public bounded iterator; this pass never reads a whole event column.
    if not report.legacy_compatible:
        try:
            for _chunk in _iter_event_hdf5_handle(
                h5,
                np,
                path=path,
                chunk_size=chunk_size,
            ):
                pass
        except FailureError as exc:
            failure = _classify_stream_failure(path, exc)
            if failure is exc.failure_info:
                raise
            _raise_failure(failure, cause=exc)

    descriptor_report = _validate_event_hdf5_handle(
        h5py=h5py,
        np=np,
        h5=h5,
        path=path,
        chunk_size=chunk_size,
        allow_legacy_without_digest=allow_legacy_without_digest,
    )
    descriptor_report.raise_for_errors()
    return _require_event_mapping(_read_event_hdf5_handle(h5, np))


def _validate_event_hdf5_handle(
    *,
    h5py: Any,
    np: Any,
    h5: Any,
    path: Path,
    chunk_size: int = _DEFAULT_VALIDATION_CHUNK_SIZE,
    allow_legacy_without_digest: bool = False,
) -> EventHDF5ValidationReport:
    storage_issues = _uncontained_storage_issues(h5py, h5)
    if storage_issues:
        return EventHDF5ValidationReport(
            path=path,
            issues=storage_issues,
            failure_info=_schema_integrity_failure(path, storage_issues),
        )

    issues: list[str] = []
    event_count: int | None = None
    metadata_event_count: int | None = None
    metadata_event_sha256: str | None = None
    legacy_compatible = False

    events_group = h5.get("events")
    if not isinstance(events_group, h5py.Group):
        issues.append("missing required group events")
        events_group = None

    metadata_group = h5.get("metadata")
    if not isinstance(metadata_group, h5py.Group):
        issues.append("missing required group metadata")
        metadata_group = None

    if metadata_group is not None:
        _validate_hdf5_metadata_storage(metadata_group.attrs, path=path)

    resolution = (
        _metadata_resolution(
            attrs=metadata_group.attrs,
            np=np,
            issues=issues,
        )
        if metadata_group is not None
        else None
    )

    if events_group is not None:
        event_count = _validate_event_hdf5_event_datasets(
            h5py=h5py,
            np=np,
            events_group=events_group,
            columns=EVENT_HDF5_COLUMNS,
            issues=issues,
            chunk_size=chunk_size,
            resolution=resolution,
        )

    if metadata_group is not None:
        (
            metadata_event_count,
            metadata_event_sha256,
            legacy_compatible,
        ) = _validate_event_hdf5_metadata(
            attrs=metadata_group.attrs,
            expected_schema_version=EVENT_HDF5_VERSION,
            expected_event_count=event_count,
            provenance_fields=tuple(EventEpisodeMetadata.__dataclass_fields__),
            issues=issues,
            allow_legacy_without_digest=allow_legacy_without_digest,
        )

    metadata = _decode_metadata_attrs(metadata_group.attrs) if metadata_group else {}

    return EventHDF5ValidationReport(
        path=path,
        issues=tuple(issues),
        event_count=event_count,
        metadata_event_count=metadata_event_count,
        metadata_event_sha256=metadata_event_sha256,
        metadata=metadata,
        legacy_compatible=legacy_compatible and not issues,
        failure_info=(
            None if not issues else _schema_integrity_failure(path, tuple(issues))
        ),
    )


def _uncontained_storage_issues(h5py: Any, h5: Any) -> tuple[str, ...]:
    """Name every event or metadata member whose bytes live outside this file.

    The validated readers digest a published snapshot. An external link, HDF5
    external raw storage or a virtual dataset would make them read another
    local path instead -- one that can hang (a FIFO), leak an arbitrary
    readable file, or change between the digest pass and materialization.
    Links are inspected with ``getlink=True`` and datasets through their
    storage properties, so this check reads no event data.
    """

    issues: list[str] = []
    for group_name in ("events", "metadata"):
        link = h5.get(group_name, getlink=True)
        if link is None:
            continue  # reported as a missing group by the schema pass
        if not isinstance(link, h5py.HardLink):
            issues.append(
                f"{group_name} is a {type(link).__name__}, not a hard link; "
                f"its contents would be read {_UNCONTAINED_STORAGE_MARKER}"
            )
            continue
        group = h5[group_name]
        if not isinstance(group, h5py.Group):
            continue
        if len(group) > _MAX_EVENT_GROUP_MEMBERS:
            issues.append(
                f"{group_name} has {len(group)} members, more than "
                f"{_MAX_EVENT_GROUP_MEMBERS}; unchecked members could be read "
                f"{_UNCONTAINED_STORAGE_MARKER}"
            )
            continue
        for name in group:
            member = f"{group_name}/{name}"
            member_link = group.get(name, getlink=True)
            if not isinstance(member_link, h5py.HardLink):
                issues.append(
                    f"{member} is a {type(member_link).__name__}, not a hard "
                    f"link; it would be read {_UNCONTAINED_STORAGE_MARKER}"
                )
                continue
            value = group.get(name)
            if not isinstance(value, h5py.Dataset):
                continue
            if value.is_virtual:
                issues.append(
                    f"{member} is a virtual dataset; it would be read "
                    f"{_UNCONTAINED_STORAGE_MARKER}"
                )
            elif value.external:
                issues.append(
                    f"{member} uses external storage; it would be read "
                    f"{_UNCONTAINED_STORAGE_MARKER}"
                )
    return tuple(issues)


def _decode_metadata_attrs(attrs: Any) -> dict[str, Any]:
    """Copy bounded HDF5 metadata and decode the canonical JSON attrs."""

    metadata = dict(attrs)
    for key in _JSON_ENCODED_METADATA_FIELDS:
        if key not in metadata:
            continue
        raw = metadata[key]
        if isinstance(raw, (bytes, bytearray)):
            try:
                raw = raw.decode("utf-8")
            except UnicodeDecodeError:
                continue
        try:
            metadata[key] = json.loads(raw)
        except (TypeError, ValueError):
            # The schema issue already records malformed JSON. Keep the raw
            # value available for diagnostics without masking that issue.
            pass
    return metadata


def _metadata_resolution(
    *,
    attrs: Any,
    np: Any,
    issues: list[str],
) -> tuple[int, int] | None:
    if "resolution" not in attrs:
        return None
    try:
        raw = np.asarray(attrs["resolution"])
        if raw.shape != (2,):
            raise ValueError("resolution must have two values")
        bounds: list[int] = []
        for value in raw:
            bound = _strict_integer(value)
            if bound <= 0:
                raise ValueError("resolution values must be positive")
            bounds.append(bound)
    except (TypeError, ValueError, OverflowError):
        issues.append("metadata resolution must be a pair of positive integers")
        return None
    return bounds[0], bounds[1]


def _validate_event_hdf5_event_datasets(
    *,
    h5py: Any,
    np: Any,
    events_group: Any,
    columns: tuple[tuple[str, str], ...],
    issues: list[str],
    chunk_size: int,
    resolution: tuple[int, int] | None,
) -> int | None:
    event_count: int | None = None
    for name, dtype in columns:
        dataset = events_group.get(name)
        dataset_path = f"events/{name}"
        if not isinstance(dataset, h5py.Dataset):
            issues.append(f"missing required dataset {dataset_path}")
            continue

        if dataset.ndim != 1:
            issues.append(
                f"{dataset_path} must be 1-dimensional; got shape "
                f"{tuple(dataset.shape)}"
            )
            continue

        if np.dtype(dataset.dtype) != np.dtype(dtype):
            issues.append(
                f"{dataset_path} dtype {dataset.dtype} does not match "
                f"required dtype {np.dtype(dtype)}"
            )

        length = int(dataset.shape[0])
        if event_count is None:
            event_count = length
        elif length != event_count:
            issues.append(
                f"{dataset_path} length {length} does not match "
                f"events/t length {event_count}"
            )

        if length > MAX_EVENT_ROWS:
            issues.append(
                f"{dataset_path} length {length} exceeds event row limit "
                f"{MAX_EVENT_ROWS}"
            )
            continue

        # Value invariants are scanned in bounded slices. This legacy-capable
        # structural pass never materializes a complete event column.
        previous_timestamp: int | None = None
        for start in range(0, length, chunk_size):
            values = np.asarray(dataset[start : min(start + chunk_size, length)])
            try:
                finite = bool(np.all(np.isfinite(values)))
            except TypeError:
                finite = False
            if not finite:
                issues.append(f"{dataset_path} contains non-finite event data")
                break
            if name in {"x", "y"} and resolution is not None and values.size:
                bound = resolution[0] if name == "x" else resolution[1]
                if values.dtype.kind == "u":
                    outside = bool(np.any(values >= bound))
                else:
                    outside = bool(np.any(values < 0)) or bool(np.any(values >= bound))
                if outside:
                    issues.append(
                        f"{dataset_path} coordinates must be in [0, {bound}) "
                        "from metadata resolution"
                    )
                    break
            if name == "t" and values.size:
                timestamps = np.asarray(values, dtype=np.int64)
                if bool(np.any(timestamps < 0)):
                    issues.append(f"{dataset_path} timestamps must be non-negative")
                    break
                if (
                    previous_timestamp is not None
                    and int(timestamps[0]) < previous_timestamp
                ) or (
                    timestamps.size > 1 and not bool(np.all(np.diff(timestamps) >= 0))
                ):
                    issues.append(
                        f"{dataset_path} timestamps are not monotonically "
                        "non-decreasing"
                    )
                    break
                previous_timestamp = int(timestamps[-1])
            elif name == "p" and values.size:
                if not bool(np.all((values == -1) | (values == 1))):
                    issues.append(
                        f"{dataset_path} contains polarity values outside {{-1, +1}}"
                    )
                    break
    return event_count


def _strict_integer(value: object) -> int:
    value_type = type(value)
    if (
        isinstance(value, bool)
        or (
            value_type.__module__.split(".", 1)[0] == "numpy"
            and value_type.__name__ in {"bool", "bool_"}
        )
        or not isinstance(value, Integral)
    ):
        raise TypeError("metadata value must be an integer")
    return int(value)


def _valid_writer_telemetry(value: object) -> bool:
    if not isinstance(value, dict) or set(value) != set(_WRITER_TELEMETRY_TYPES):
        return False
    for name, expected in _WRITER_TELEMETRY_TYPES.items():
        item = value[name]
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            return False
        if expected is int and not isinstance(item, int):
            return False
        if not math.isfinite(item) or item < 0:
            return False
    return True


def _validate_event_hdf5_metadata(
    *,
    attrs: Any,
    expected_schema_version: int,
    expected_event_count: int | None,
    provenance_fields: tuple[str, ...],
    issues: list[str],
    allow_legacy_without_digest: bool,
) -> tuple[int | None, str | None, bool]:
    schema_version = _int_metadata_attr(attrs, "schema_version", issues)
    legacy_without_digest = (
        allow_legacy_without_digest
        and schema_version == expected_schema_version == 1
        and "event_sha256" not in attrs
    )
    required_provenance_fields = tuple(
        attr for attr in provenance_fields if attr not in _OPTIONAL_METADATA_ATTRS
    )
    required_attrs = required_provenance_fields + (
        "compression",
        "dropped_batches",
        "event_count",
    )
    if not legacy_without_digest:
        required_attrs += ("event_sha256",)
    for attr in required_attrs:
        if attr not in attrs:
            issues.append(f"metadata missing required attr {attr}")

    for attr in ("hot_pixel_mask", "hf_repo_ids", "writer_telemetry"):
        if attr not in attrs:
            continue
        raw = attrs[attr]
        if isinstance(raw, (bytes, bytearray)):
            try:
                raw = raw.decode("utf-8")
            except UnicodeDecodeError as exc:
                issues.append(f"metadata {attr} must contain valid UTF-8: {exc}")
                continue
        if not isinstance(raw, str):
            issues.append(f"metadata {attr} must be JSON text")
            continue
        try:
            value = json.loads(raw)
        except (TypeError, ValueError) as exc:
            issues.append(f"metadata {attr} must contain valid JSON: {exc}")
            continue
        if attr == "hot_pixel_mask":
            if not isinstance(value, list) or any(
                not isinstance(pair, list)
                or len(pair) != 2
                or any(
                    isinstance(coordinate, bool)
                    or not isinstance(coordinate, int)
                    or coordinate < 0
                    for coordinate in pair
                )
                for pair in value
            ):
                issues.append(
                    "metadata hot_pixel_mask must be a list of non-negative "
                    "[x, y] integer pairs"
                )
        elif attr == "writer_telemetry":
            if not _valid_writer_telemetry(value):
                issues.append(
                    "metadata writer_telemetry must be an object with exactly the "
                    "writer telemetry fields as non-negative finite numbers"
                )
        elif not isinstance(value, list) or any(
            not isinstance(repo_id, str) or not repo_id.strip() for repo_id in value
        ):
            issues.append("metadata hf_repo_ids must be a list of non-empty strings")

    if "hot_pixel_rate_hz" in attrs:
        raw_rate = attrs["hot_pixel_rate_hz"]
        if (
            isinstance(raw_rate, bool)
            or not isinstance(raw_rate, Real)
            or not math.isfinite(float(raw_rate))
            or float(raw_rate) < 0
        ):
            issues.append(
                "metadata hot_pixel_rate_hz must be a finite non-negative number"
            )

    if "reward_summary" in attrs:
        raw_reward_summary = attrs["reward_summary"]
        valid_utf8 = True
        if isinstance(raw_reward_summary, (bytes, bytearray)):
            try:
                raw_reward_summary = raw_reward_summary.decode("utf-8")
            except UnicodeDecodeError as exc:
                issues.append(
                    f"metadata reward_summary must contain valid UTF-8: {exc}"
                )
                valid_utf8 = False
        if valid_utf8 and not isinstance(raw_reward_summary, str):
            issues.append("metadata reward_summary must be JSON text")
        elif valid_utf8 and raw_reward_summary.strip():
            try:
                reward_summary = json.loads(raw_reward_summary)
            except (TypeError, ValueError) as exc:
                issues.append(f"metadata reward_summary must contain valid JSON: {exc}")
            else:
                if not isinstance(reward_summary, dict):
                    issues.append("metadata reward_summary must be a JSON object")

    if schema_version is not None and schema_version != expected_schema_version:
        issues.append(
            f"metadata schema_version {schema_version} does not match "
            f"expected {expected_schema_version}"
        )

    dropped_batches = _int_metadata_attr(attrs, "dropped_batches", issues)
    metadata_event_count = _int_metadata_attr(attrs, "event_count", issues)
    time_bins = _int_metadata_attr(attrs, "time_bins", issues)
    episode_id = _int_metadata_attr(attrs, "episode_id", issues)
    _int_metadata_attr(attrs, "seed", issues)
    success_flag = _int_metadata_attr(attrs, "success_flag", issues)
    if dropped_batches is not None and dropped_batches < 0:
        issues.append("metadata dropped_batches must be non-negative")
    if time_bins is not None and time_bins < 1:
        issues.append("metadata time_bins must be positive")
    if episode_id is not None and episode_id < 0:
        issues.append("metadata episode_id must be non-negative")
    if success_flag is not None and success_flag not in (0, 1):
        issues.append("metadata success_flag must be 0 or 1")
    _finite_float_metadata_attr(attrs, "sub_render_hz", issues)
    if metadata_event_count is not None and metadata_event_count < 0:
        issues.append("metadata event_count must be non-negative")
    if (
        metadata_event_count is not None
        and expected_event_count is not None
        and metadata_event_count != expected_event_count
    ):
        issues.append(
            f"metadata event_count {metadata_event_count} does not match "
            f"events/t length {expected_event_count}"
        )

    metadata_event_sha256: str | None = None
    if "event_sha256" in attrs:
        raw_digest = attrs["event_sha256"]
        if isinstance(raw_digest, (bytes, bytearray)):
            raw_digest = raw_digest.decode("ascii", errors="replace")
        if isinstance(raw_digest, str):
            normalized_digest = raw_digest.lower()
            if len(normalized_digest) == 64 and all(
                character in "0123456789abcdef" for character in normalized_digest
            ):
                metadata_event_sha256 = normalized_digest
            else:
                issues.append("metadata event_sha256 must be 64 hexadecimal chars")
        else:
            issues.append("metadata event_sha256 must be hexadecimal text")
    return metadata_event_count, metadata_event_sha256, legacy_without_digest


def _int_metadata_attr(
    attrs: Any,
    key: str,
    issues: list[str],
) -> int | None:
    if key not in attrs:
        return None
    try:
        return _strict_integer(attrs[key])
    except (TypeError, ValueError, OverflowError):
        issues.append(f"metadata {key} must be an integer; got {attrs[key]!r}")
        return None


def _finite_float_metadata_attr(
    attrs: Any,
    key: str,
    issues: list[str],
) -> float | None:
    if key not in attrs:
        return None
    value = attrs[key]
    if isinstance(value, bool) or not isinstance(value, Real):
        issues.append(f"metadata {key} must be a finite positive number")
        return None
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        issues.append(f"metadata {key} must be a finite positive number")
        return None
    return parsed


__all__ = [
    "EventHDF5ValidationReport",
    "read_validated_event_hdf5",
    "validate_event_hdf5_schema",
]
