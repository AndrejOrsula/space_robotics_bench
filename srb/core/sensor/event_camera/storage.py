"""Canonical event-camera HDF5 layout and writer contract.

Datasets: /events/{t:int64 ns, x:uint16, y:uint16, p:int8}; /metadata stores
EventEpisodeMetadata attrs. Writers validate into private staging files and publish on
close.
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import operator
import os
import queue
import struct
import threading
import time
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from types import TracebackType
from typing import TYPE_CHECKING, Any, Iterator
from uuid import uuid4

from srb.utils.extras import require_extra
from srb.utils.failures import (
    FALLBACK_ALLOWED_CATEGORIES,
    FailureCategory,
    FailureError,
    FailureInfo,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
    IntegrityFailureError,
    InternalInvariantError,
    InvalidInputError,
    OptionalDependencyUnavailableError,
    TransientExternalFailureError,
    make_degraded_health_event,
)
from srb.utils.publication import (
    ArtifactPublication,
    PublicationCleanupError,
    PublicationIntegrityError,
    PublicationTimeoutError,
)
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_EVENT_BATCH_ROWS,
    MAX_EVENT_HDF5_BYTES,
    MAX_EVENT_ROWS,
    MAX_METADATA_AGGREGATE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    MAX_METADATA_ATTRIBUTES,
    ByteBudget,
    ResourceLimitError,
    checked_shape_nbytes,
    require_file_size,
)

if TYPE_CHECKING:
    import h5py
    import numpy as np
    import torch

EVENT_HDF5_VERSION = 1
# Bump for breaking event dataset changes or metadata attribute removals,
# renames, or retypes; additive metadata attributes remain compatible.

_JSON_ENCODED_METADATA_FIELDS: frozenset[str] = frozenset(
    {
        "hot_pixel_mask",
        "hf_repo_ids",
        "writer_telemetry",
    }
)
# Omit optional fields set to None; absence represents "unknown".
_OPTIONAL_UNKNOWN_METADATA_FIELDS: frozenset[str] = frozenset({"hot_pixel_rate_hz"})
# These fields are JSON-encoded because their Python types do not round-trip
# through HDF5 attributes. Changes to this set require a schema version bump.

EVENT_HDF5_COLUMNS: tuple[tuple[str, str], ...] = (
    ("t", "<i8"),
    ("x", "<u2"),
    ("y", "<u2"),
    ("p", "i1"),
)

_INITIAL_CHUNK_ROWS = 4096

_EVENT_VALIDATION_CHUNK_ROWS = _INITIAL_CHUNK_ROWS

_DEFAULT_QUEUE_SIZE = 256

_DEFAULT_QUEUE_TIMEOUT_S = 5.0

_DEFAULT_CLOSE_TIMEOUT_S = 5.0

_MAX_HEALTH_EVENTS = 4
# Retain a bounded set of health observations.

_MAX_CAUSE_TEXT = 256

_LOG = logging.getLogger(__name__)
_EVENT_STORAGE_COMPONENT = "event_camera.storage"
_EVENT_DIGEST_METADATA_KEY = "event_sha256"
_EVENT_DIGEST_DOMAIN = b"srb.event_camera.event_digest.v1"


def _require_positive_int(value: object, *, name: str, path: Path) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise InvalidInputError(
            code=f"event_hdf5.writer.invalid_{name}",
            message=f"{name} must be a positive integer",
            component=_EVENT_STORAGE_COMPONENT,
            details={"source": str(path), name: value},
        )
    return value


def _require_limited_positive_int(
    value: object,
    *,
    name: str,
    path: Path,
    maximum: int,
) -> int:
    parsed = _require_positive_int(value, name=name, path=path)
    if parsed > maximum:
        raise InvalidInputError(
            code=f"event_hdf5.writer.invalid_{name}",
            message=f"{name} must be no greater than {maximum}",
            component=_EVENT_STORAGE_COMPONENT,
            details={"source": str(path), name: parsed, "maximum": maximum},
        )
    return parsed


def _require_event_hdf5_file_size(file: Any, *, path: Path) -> int:
    size = os.fstat(file.fileno()).st_size
    if size > MAX_EVENT_HDF5_BYTES:
        raise ResourceLimitError(
            "EVENT_HDF5 file",
            limit=MAX_EVENT_HDF5_BYTES,
            observed=size,
        )
    return size


def _attribute_storage_size(attribute: Any) -> int:
    try:
        return int(attribute.get_storage_size())
    except Exception:
        shape = getattr(attribute, "shape", None)
        if shape is not None and any(int(dim) == 0 for dim in shape):
            return 0
        raise


def _validate_hdf5_metadata_storage(attrs: Any, *, path: Path) -> None:
    names: list[str] = []
    for index, name in enumerate(attrs):
        if index >= MAX_METADATA_ATTRIBUTES:
            raise ResourceLimitError(
                "EVENT_HDF5 metadata attribute count",
                limit=MAX_METADATA_ATTRIBUTES,
                observed=index + 1,
            )
        names.append(str(name))

    aggregate = ByteBudget(
        "EVENT_HDF5 metadata storage",
        MAX_METADATA_AGGREGATE_BYTES,
    )
    for name in names:
        try:
            attribute = attrs.get_id(name)
            storage_size = _attribute_storage_size(attribute)
        except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
            raise ValueError(
                f"cannot inspect EVENT_HDF5 metadata attribute {name!r} in {path}"
            ) from exc
        if storage_size > MAX_METADATA_ATTRIBUTE_BYTES:
            raise ResourceLimitError(
                f"EVENT_HDF5 metadata attribute {name!r}",
                limit=MAX_METADATA_ATTRIBUTE_BYTES,
                observed=storage_size,
            )
        aggregate.consume(storage_size)


def _peek_event_rows(events: Any) -> int | None:
    shape = getattr(events, "shape", None)
    if shape is not None:
        try:
            if len(shape) >= 1:
                return operator.index(shape[0])
        except (IndexError, TypeError, ValueError):
            return None
    try:
        return len(events)
    except TypeError:
        return None


class EventHDF5BackpressureError(TransientExternalFailureError):
    """A bounded async queue could not accept a batch before its deadline."""


def _coerce_event_array(events: Any, np: Any) -> Any:
    try:
        import torch
    except ImportError:
        torch = None

    if torch is not None and isinstance(events, torch.Tensor):
        arr = events.detach().cpu().numpy()
    else:
        arr = np.asarray(events)

    if arr.ndim != 2 or arr.shape[1] != 4:
        raise InvalidInputError(
            code="event_hdf5.writer.invalid_events",
            message=f"events must be shape (N, 4); got {arr.shape}",
            component=_EVENT_STORAGE_COMPONENT,
            details={"shape": tuple(arr.shape)},
        )
    # Async callers may mutate their source immediately after append().  The
    # accepted batch is therefore a private, contiguous snapshot.
    return np.array(arr, copy=True, order="C")


def _invalid_event_data(
    message: str,
    *,
    path: Path,
    field: str | None = None,
) -> InvalidInputError:
    details: dict[str, object] = {"source": str(path)}
    if field is not None:
        details["field"] = field
    return InvalidInputError(
        code="event_hdf5.writer.invalid_event_data",
        message=message,
        component=_EVENT_STORAGE_COMPONENT,
        details=details,
    )


def _event_resolution(metadata: Any, *, path: Path) -> tuple[int, int]:
    resolution = getattr(metadata, "resolution", None)
    if not isinstance(resolution, (tuple, list)) or len(resolution) != 2:
        raise _invalid_event_data(
            "metadata resolution must be a pair of positive integers",
            path=path,
            field="resolution",
        )
    width_value, height_value = resolution
    try:
        width = operator.index(width_value)
        height = operator.index(height_value)
    except TypeError as exc:
        raise _invalid_event_data(
            "metadata resolution must be a pair of positive integers",
            path=path,
            field="resolution",
        ) from exc
    if width <= 0 or height <= 0:
        raise _invalid_event_data(
            "metadata resolution must be a pair of positive integers",
            path=path,
            field="resolution",
        )
    return width, height


def _validate_event_array(
    arr: Any,
    metadata: Any,
    previous_timestamp: int | None,
    np: Any,
    *,
    path: Path,
) -> int | None:
    dtype = arr.dtype
    if dtype.kind not in "iuf":
        raise _invalid_event_data(
            "events dtype must be a real numeric dtype that can be cast safely "
            "to the canonical event schema",
            path=path,
        )
    try:
        finite = bool(np.all(np.isfinite(arr)))
    except TypeError as exc:
        raise _invalid_event_data(
            "events must contain finite values safely castable to the canonical "
            "event schema",
            path=path,
        ) from exc
    if not finite:
        raise _invalid_event_data(
            "events must contain finite values safely castable to the canonical "
            "event schema",
            path=path,
        )

    if dtype.kind == "f":
        for column_index, (name, _dtype) in enumerate(EVENT_HDF5_COLUMNS):
            values = arr[:, column_index]
            if not bool(np.all(values == np.trunc(values))):
                raise _invalid_event_data(
                    f"events/{name} contains values that are not safely "
                    "castable to its canonical integer dtype",
                    path=path,
                    field=name,
                )
            if name == "t":
                # Above this boundary floating dtypes cannot represent every
                # integer timestamp, even when the stored float itself is
                # integral and casts back to the same float value.
                max_exact_integer = 1 << (int(np.finfo(dtype).nmant) + 1)
                if bool(np.any(np.abs(values) >= max_exact_integer)):
                    raise _invalid_event_data(
                        "events/t timestamps must be exactly representable as int64",
                        path=path,
                        field="t",
                    )

    t_values = arr[:, 0]
    x_values = arr[:, 1]
    y_values = arr[:, 2]
    p_values = arr[:, 3]
    if bool(np.any(t_values < 0)):
        raise _invalid_event_data(
            "events/t timestamps must be non-negative",
            path=path,
            field="t",
        )
    if bool(np.any(t_values >= (1 << 63))):
        raise _invalid_event_data(
            "events/t contains values that are not safely castable to int64",
            path=path,
            field="t",
        )
    if bool(np.any(x_values >= (1 << 16))) or bool(np.any(y_values >= (1 << 16))):
        raise _invalid_event_data(
            "events coordinates contain values that are not safely castable to uint16",
            path=path,
            field="x/y",
        )
    if bool(np.any(p_values < -128)) or bool(np.any(p_values > 127)):
        raise _invalid_event_data(
            "events/p contains values that are not safely castable to int8",
            path=path,
            field="p",
        )

    width, height = _event_resolution(metadata, path=path)
    if bool(np.any(x_values < 0)) or bool(np.any(x_values >= width)):
        raise _invalid_event_data(
            f"events/x coordinates must be in [0, {width})",
            path=path,
            field="x",
        )
    if bool(np.any(y_values < 0)) or bool(np.any(y_values >= height)):
        raise _invalid_event_data(
            f"events/y coordinates must be in [0, {height})",
            path=path,
            field="y",
        )
    if not bool(np.all((p_values == -1) | (p_values == 1))):
        raise _invalid_event_data(
            "events/p polarity values must be -1 or +1",
            path=path,
            field="p",
        )

    timestamps = arr[:, 0].astype(np.int64, copy=False)
    if timestamps.size > 1 and not bool(np.all(timestamps[1:] >= timestamps[:-1])):
        raise _invalid_event_data(
            "events/t timestamps must be monotonically non-decreasing",
            path=path,
            field="t",
        )
    first_timestamp = int(timestamps[0])
    if previous_timestamp is not None and first_timestamp < previous_timestamp:
        raise _invalid_event_data(
            "events/t timestamps must be monotonically non-decreasing across batches",
            path=path,
            field="t",
        )
    return int(timestamps[-1])


def _canonical_event_columns(arr: Any, np: Any) -> tuple[Any, ...]:
    return tuple(
        np.array(
            arr[:, column_index],
            dtype=np.dtype(dtype),
            copy=True,
            order="C",
        )
        for column_index, (_, dtype) in enumerate(EVENT_HDF5_COLUMNS)
    )


def _digest_frame(hasher: Any, payload: bytes) -> None:
    hasher.update(struct.pack("<Q", len(payload)))
    hasher.update(payload)


def _digest_shape_frame(shape: tuple[int, ...]) -> bytes:
    return struct.pack("<Q", len(shape)) + b"".join(
        struct.pack("<q", int(dimension)) for dimension in shape
    )


class _EventDigestAccumulator:
    def __init__(self) -> None:
        self.count = 0
        self._field_hashers = tuple(hashlib.sha256() for _ in EVENT_HDF5_COLUMNS)

    def update(self, arr: Any, np: Any) -> None:
        self._update_columns(_canonical_event_columns(arr, np))

    def update_columns(self, columns: dict[str, Any], np: Any) -> None:
        canonical = tuple(
            np.array(
                columns[name],
                dtype=np.dtype(dtype),
                copy=True,
                order="C",
            )
            for name, dtype in EVENT_HDF5_COLUMNS
        )
        self._update_columns(canonical)

    def _update_columns(self, columns: tuple[Any, ...]) -> None:
        row_count = int(columns[0].shape[0])
        if any(int(column.shape[0]) != row_count for column in columns[1:]):
            raise InternalInvariantError(
                code="event_hdf5.digest.column_lengths",
                message="event digest columns have unequal lengths",
                component=_EVENT_STORAGE_COMPONENT,
                details={"row_count": row_count},
            )
        for hasher, column in zip(self._field_hashers, columns):
            hasher.update(column.tobytes(order="C"))
        self.count += row_count

    def hexdigest(self, np: Any) -> str:
        digest = hashlib.sha256()
        _digest_frame(digest, _EVENT_DIGEST_DOMAIN)
        _digest_frame(digest, struct.pack("<Q", len(EVENT_HDF5_COLUMNS)))
        _digest_frame(digest, struct.pack("<Q", self.count))
        for (name, dtype), field_hasher in zip(EVENT_HDF5_COLUMNS, self._field_hashers):
            _digest_frame(digest, name.encode("ascii"))
            _digest_frame(digest, np.dtype(dtype).str.encode("ascii"))
            _digest_frame(digest, _digest_shape_frame((self.count,)))
            _digest_frame(digest, field_hasher.digest())
        return digest.hexdigest()


def event_digest(events: Any) -> str:
    """Hash fields in declared order using versioned, length-framed canonical
    little-endian bytes; input strides and endianness do not affect the digest.
    """
    try:
        import numpy as np
    except ImportError as exc:
        raise OptionalDependencyUnavailableError(
            code="event_hdf5.digest.numpy_unavailable",
            message="event_digest requires numpy",
            component=_EVENT_STORAGE_COMPONENT,
            details={"source": "event_digest"},
        ) from exc
    arr = _coerce_event_array(events, np)
    accumulator = _EventDigestAccumulator()
    accumulator.update(arr, np)
    return accumulator.hexdigest(np)


def _bounded_text(value: object) -> str:
    text = str(value) or type(value).__name__
    if len(text) <= _MAX_CAUSE_TEXT:
        return text
    return f"{text[: _MAX_CAUSE_TEXT - 3]}..."


def _exception_details(exc: BaseException) -> dict[str, object]:
    return {
        "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
        "cause_message": _bounded_text(exc),
    }


def _enable_swmr(h5: Any) -> None:
    h5.swmr_mode = True


def _swmr_fallback_failure(
    exc: BaseException,
    *,
    path: Path,
) -> FailureInfo | None:
    """Classify only typed failures and stable h5py causes; leave unrelated
    constructor failures unclassified."""

    if isinstance(exc, FailureError):
        failure = exc.failure_info
        if failure.category in FALLBACK_ALLOWED_CATEGORIES and failure.fallback_allowed:
            return failure
        return None

    if isinstance(exc, (AttributeError, NotImplementedError, RuntimeError)):
        category = FailureCategory.UNSUPPORTED_CAPABILITY
        code = "event_hdf5.swmr.unsupported"
        retryable = False
    elif isinstance(exc, ImportError):
        category = FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE
        code = "event_hdf5.swmr.optional_dependency"
        retryable = False
    elif isinstance(exc, (TimeoutError, OSError)):
        category = FailureCategory.TRANSIENT_EXTERNAL_FAILURE
        code = "event_hdf5.swmr.transient_failure"
        retryable = True
    else:
        return None

    return FailureInfo(
        category=category,
        code=code,
        message=(
            f"SWMR enablement failed for {path}; the writer will use "
            f"non-SWMR mode: {_bounded_text(exc)}"
        ),
        component=_EVENT_STORAGE_COMPONENT,
        details={"source": str(path), **_exception_details(exc)},
        retryable=retryable,
        fallback_allowed=True,
    )


def _writer_failure_error(
    exc: BaseException,
    *,
    path: Path,
    operation: str,
) -> FailureError:
    if isinstance(exc, FailureError):
        return exc

    if isinstance(exc, PublicationIntegrityError):
        error_type: type[FailureError] = IntegrityFailureError
        category = FailureCategory.INTEGRITY_FAILURE
        retryable = False
    elif isinstance(exc, (PublicationTimeoutError, TimeoutError)):
        error_type = TransientExternalFailureError
        category = FailureCategory.TRANSIENT_EXTERNAL_FAILURE
        retryable = True
    else:
        # An untyped h5py/filesystem exception has no evidence for a more
        # specific caller action.  The taxonomy's conservative default is
        # internal-invariant, and the original exception remains __cause__.
        error_type = InternalInvariantError
        category = FailureCategory.INTERNAL_INVARIANT
        retryable = False

    info = FailureInfo(
        category=category,
        code=f"event_hdf5.writer.{operation}.failure",
        message=(
            f"EVENT_HDF5 writer {operation} failed for {path}: {_bounded_text(exc)}"
        ),
        component=_EVENT_STORAGE_COMPONENT,
        details={
            "source": str(path),
            "operation": operation,
            **_exception_details(exc),
        },
        retryable=retryable,
    )
    error = error_type(info)
    # Background failures are raised later by close(); preserve the exact
    # original exception even though no ``raise ... from`` occurs there.
    error.__cause__ = exc
    error.__suppress_context__ = True
    return error


@dataclass
class EventEpisodeProvenance:
    """Canonical fields shared by HDF5 metadata and manifest rows."""

    sensor_preset: str = ""
    resolution: tuple[int, int] = (0, 0)
    sub_render_hz: float = 0.0
    backend_name: str = ""
    backend_version: str = ""
    backend_commit_sha: str = ""
    representation: str = ""
    time_bins: int = 0
    episode_id: int = 0
    policy_id: str = ""
    policy_checkpoint_hash: str = ""  # SHA256 of policy checkpoint
    task_id: str = ""
    seed: int = 0
    srb_git_sha: str = ""
    isaaclab_version: str = ""
    rgb_mp4_path: str = ""  # paired video file (relative to manifest dir)
    success_flag: int = 0  # 0 = failure / unknown, 1 = success
    reward_summary: str = ""  # JSON-encoded scalar summary
    # Additive fields are stored under `/metadata`. Empty
    # defaults keep round-trip of earlier HDF5 files valid.
    hot_pixel_mask: tuple[tuple[int, int], ...] = ()
    hf_repo_ids: tuple[str, ...] = ()
    # Effective hot-pixel injection rate.
    # ``None`` means unknown (a file written before the attr existed) and is
    # not written; new recordings always carry a float (0.0 = no injection).
    hot_pixel_rate_hz: float | None = None


@dataclass
class EventEpisodeMetadata(EventEpisodeProvenance):
    """Per-episode metadata stored under /metadata."""

    schema_version: int = field(default=EVENT_HDF5_VERSION)


@dataclass(frozen=True)
class EventHDF5WriterTelemetry:
    queue_capacity: int
    accepted_batch_count: int
    accepted_event_count: int
    written_batch_count: int
    written_event_count: int
    queued_high_water: int
    enqueue_wait_total_s: float
    enqueue_wait_max_s: float
    enqueue_timeout_count: int
    enqueue_rejection_count: int
    close_drain_elapsed_s: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _snapshot_event_metadata(
    metadata: EventEpisodeMetadata,
    *,
    path: Path,
) -> EventEpisodeMetadata:
    schema_version = getattr(metadata, "schema_version", None)
    if type(schema_version) is not int or schema_version != EVENT_HDF5_VERSION:
        raise InvalidInputError(
            code="event_hdf5.writer.invalid_schema_version",
            message=(
                "metadata schema_version must be the exact supported integer "
                f"{EVENT_HDF5_VERSION}"
            ),
            component=_EVENT_STORAGE_COMPONENT,
            details={
                "source": str(path),
                "schema_version": schema_version,
                "supported_schema_version": EVENT_HDF5_VERSION,
            },
        )
    values = _metadata_values(metadata)
    _event_resolution(metadata, path=path)
    _prepare_metadata_attrs(values, path=path)
    return EventEpisodeMetadata(**_snapshot_metadata_values(values))


def _metadata_values(metadata: EventEpisodeMetadata) -> dict[str, Any]:
    values = {
        item.name: getattr(metadata, item.name) for item in fields(EventEpisodeMetadata)
    }
    # Optional attrs whose ``None`` means "unknown" are omitted, not written.
    for name in _OPTIONAL_UNKNOWN_METADATA_FIELDS:
        if values.get(name) is None:
            values.pop(name, None)
    return values


def _bounded_utf8_size(value: str, *, resource: str, limit: int) -> int:
    size = 0
    for character in value:
        codepoint = ord(character)
        if 0xD800 <= codepoint <= 0xDFFF:
            raise ValueError(f"{resource} contains an unpaired UTF-16 surrogate")
        if codepoint <= 0x7F:
            size += 1
        elif codepoint <= 0x7FF:
            size += 2
        elif codepoint <= 0xFFFF:
            size += 3
        else:
            size += 4
        if size > limit:
            raise ResourceLimitError(resource, limit=limit, observed=size)
    return size


def _bounded_size_add(
    total: int,
    amount: int,
    *,
    resource: str,
    limit: int,
) -> int:
    observed = total + amount
    if observed > limit:
        raise ResourceLimitError(resource, limit=limit, observed=observed)
    return observed


def _json_string_size(value: str, *, resource: str, limit: int) -> int:
    size = 2  # JSON string delimiters.
    for character in value:
        codepoint = ord(character)
        if 0xD800 <= codepoint <= 0xDFFF:
            raise ValueError(f"{resource} contains an unpaired UTF-16 surrogate")
        if character in {'"', "\\"}:
            increment = 2
        elif codepoint in {8, 9, 10, 12, 13}:
            increment = 2
        elif codepoint < 0x20:
            increment = 6  # ``\\u00XX``.
        elif 0x20 <= codepoint <= 0x7E:
            increment = 1
        elif codepoint <= 0xFFFF:
            increment = 6  # ``\\uXXXX``.
        else:
            increment = 12  # Escaped UTF-16 surrogate pair.
        size = _bounded_size_add(size, increment, resource=resource, limit=limit)
    return size


def _json_float_text(value: float) -> str:
    if math.isnan(value):
        return "NaN"
    if math.isinf(value):
        return "-Infinity" if value < 0 else "Infinity"
    return repr(value)


def _json_key_text(value: object) -> str:
    if isinstance(value, str):
        return value
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return _json_float_text(value)
    raise TypeError(
        f"JSON object key of type {type(value).__name__} is not serializable"
    )


def _bounded_json_size(
    value: object,
    *,
    resource: str,
    limit: int,
    depth: int = 0,
) -> int:
    if depth > 128:
        raise ValueError(f"{resource} is nested deeper than 128 levels")
    if value is None:
        return 4
    if isinstance(value, bool):
        return 4 if value else 5
    if isinstance(value, str):
        return _json_string_size(value, resource=resource, limit=limit)
    if isinstance(value, int):
        return _bounded_utf8_size(
            str(value),
            resource=resource,
            limit=limit,
        )
    if isinstance(value, float):
        return _bounded_utf8_size(
            _json_float_text(value),
            resource=resource,
            limit=limit,
        )
    if isinstance(value, (tuple, list)):
        size = 2
        for index, item in enumerate(value):
            if index:
                size = _bounded_size_add(size, 1, resource=resource, limit=limit)
            size = _bounded_size_add(
                size,
                _bounded_json_size(
                    item,
                    resource=resource,
                    limit=limit - size,
                    depth=depth + 1,
                ),
                resource=resource,
                limit=limit,
            )
        return size
    if isinstance(value, dict):
        size = 2
        for index, (key, item) in enumerate(value.items()):
            if index:
                size = _bounded_size_add(size, 1, resource=resource, limit=limit)
            key_text = _json_key_text(key)
            size = _bounded_size_add(
                size,
                _json_string_size(
                    key_text,
                    resource=resource,
                    limit=limit - size,
                )
                + 1,
                resource=resource,
                limit=limit,
            )
            size = _bounded_size_add(
                size,
                _bounded_json_size(
                    item,
                    resource=resource,
                    limit=limit - size,
                    depth=depth + 1,
                ),
                resource=resource,
                limit=limit,
            )
        return size
    raise TypeError(
        f"{resource} contains a non-JSON value of type {type(value).__name__}"
    )


def _metadata_attr_size(
    name: str,
    value: object,
    *,
    path: Path,
) -> int:
    resource = f"EVENT_HDF5 metadata attribute {name!r}"
    if name in _JSON_ENCODED_METADATA_FIELDS:
        size = _bounded_json_size(
            value,
            resource=resource,
            limit=MAX_METADATA_ATTRIBUTE_BYTES,
        )
        if size > MAX_METADATA_ATTRIBUTE_BYTES:
            raise ResourceLimitError(
                resource,
                limit=MAX_METADATA_ATTRIBUTE_BYTES,
                observed=size,
            )
        return size
    if isinstance(value, str):
        return _bounded_utf8_size(
            value,
            resource=resource,
            limit=MAX_METADATA_ATTRIBUTE_BYTES,
        )
    if isinstance(value, (bytes, bytearray)):
        size = len(value)
        if size > MAX_METADATA_ATTRIBUTE_BYTES:
            raise ResourceLimitError(
                resource,
                limit=MAX_METADATA_ATTRIBUTE_BYTES,
                observed=size,
            )
        return size
    if name == "resolution" and isinstance(value, (tuple, list)):
        if len(value) != 2:
            raise InvalidInputError(
                code="event_hdf5.writer.invalid_resolution",
                message="metadata resolution must contain exactly two values",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(path), "resolution": value},
            )
        try:
            for bound in value:
                operator.index(bound)
        except TypeError as exc:
            raise InvalidInputError(
                code="event_hdf5.writer.invalid_resolution",
                message="metadata resolution must contain integer values",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(path), "resolution": value},
            ) from exc
        if 16 > MAX_METADATA_ATTRIBUTE_BYTES:
            raise ResourceLimitError(
                resource,
                limit=MAX_METADATA_ATTRIBUTE_BYTES,
                observed=16,
            )
        return 16
    if isinstance(value, (bool, int, float)):
        if 8 > MAX_METADATA_ATTRIBUTE_BYTES:
            raise ResourceLimitError(
                resource,
                limit=MAX_METADATA_ATTRIBUTE_BYTES,
                observed=8,
            )
        return 8
    dtype = getattr(value, "dtype", None)
    shape = getattr(value, "shape", None)
    if dtype is not None and shape is not None:
        try:
            import numpy as np
        except ImportError as exc:
            raise TypeError(
                f"{resource} requires a numpy array for array metadata"
            ) from exc
        if not isinstance(value, np.ndarray):
            raise TypeError(
                f"{resource} does not support custom array-like metadata values"
            )
        try:
            shape_tuple = tuple(operator.index(dimension) for dimension in shape)
            kind = str(dtype.kind)
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            raise TypeError(f"{resource} has an invalid array value") from exc
        if kind not in "biuf":
            raise TypeError(
                f"{resource} does not support object, string, or variable-width values"
            )
        return int(
            checked_shape_nbytes(
                shape_tuple,
                dtype,
                resource=resource,
                max_elements=MAX_ARRAY_ELEMENTS,
                max_bytes=MAX_METADATA_ATTRIBUTE_BYTES,
            )
        )
    raise TypeError(f"{resource} has unsupported value type {type(value).__name__}")


def _snapshot_metadata_values(values: dict[str, Any]) -> dict[str, Any]:
    try:
        import numpy as np
    except ImportError:
        np = None

    for key, value in tuple(values.items()):
        if key == "resolution" and isinstance(value, (tuple, list)):
            values[key] = tuple(value)
        elif key == "hot_pixel_mask" and isinstance(value, (tuple, list)):
            values[key] = tuple(
                tuple(pair) if isinstance(pair, (tuple, list)) else pair
                for pair in value
            )
        elif key == "hf_repo_ids" and isinstance(value, (tuple, list)):
            values[key] = tuple(value)
        elif np is not None and isinstance(value, np.ndarray):
            # _prepare_metadata_attrs has already checked the finite size and
            # dtype. NumPy's bounded copy therefore preserves the old
            # asdict/deepcopy isolation without an unbounded allocation.
            copied = value.copy()
            if copied is value:  # pragma: no cover - defensive array seam
                raise TypeError(f"metadata array {key!r} was not detached")
            values[key] = copied
    return values


def _encode_metadata_attr(name: str, value: object, *, size: int) -> Any:
    if name in _JSON_ENCODED_METADATA_FIELDS:
        encoded = json.dumps(value, separators=(",", ":"))
        actual_size = _bounded_utf8_size(
            encoded,
            resource=f"EVENT_HDF5 metadata attribute {name!r}",
            limit=MAX_METADATA_ATTRIBUTE_BYTES,
        )
        if actual_size != size:
            raise InternalInvariantError(
                code="event_hdf5.writer.metadata_size_mismatch",
                message=(
                    f"metadata size estimator disagrees for {name!r}: "
                    f"{size} != {actual_size}"
                ),
                component=_EVENT_STORAGE_COMPONENT,
                details={"field": name, "estimated": size, "actual": actual_size},
            )
        return encoded
    if name == "resolution" and isinstance(value, (tuple, list)):
        return list(value)
    return value


def _prepare_metadata_attrs(
    values: dict[str, object],
    *,
    path: Path,
) -> tuple[tuple[str, Any], ...]:
    if len(values) > MAX_METADATA_ATTRIBUTES:
        raise ResourceLimitError(
            "EVENT_HDF5 metadata attributes",
            limit=MAX_METADATA_ATTRIBUTES,
            observed=len(values),
        )
    aggregate = ByteBudget(
        "EVENT_HDF5 metadata aggregate",
        MAX_METADATA_AGGREGATE_BYTES,
    )
    measured: list[tuple[str, object, int]] = []
    for name, value in values.items():
        size = _metadata_attr_size(name, value, path=path)
        aggregate.consume(size)
        measured.append((name, value, size))
    return tuple(
        (name, _encode_metadata_attr(name, value, size=size))
        for name, value, size in measured
    )


def _metadata_limit_failure(path: Path, exc: ResourceLimitError) -> InvalidInputError:
    return InvalidInputError(
        code="event_hdf5.writer.metadata_limit",
        message=str(exc),
        component=_EVENT_STORAGE_COMPONENT,
        details={
            "source": str(path),
            "resource": exc.resource,
            "limit": exc.limit,
            "observed": exc.observed,
        },
    )


@dataclass(frozen=True)
class _CompressionResolution:
    compression: Any
    extra_kwargs: dict[str, Any]
    label: str
    health_event: HealthEvent | None = None


def _create_writer_staging_path(
    destination: Path,
    requested: Path | None,
) -> Path:
    """Reserve a unique staging file without holding the parent publication lease;
    acquire publication ownership only after closing HDF5."""

    destination_absolute = destination.absolute()
    if requested is not None:
        staging = requested
        if staging.absolute() == destination_absolute:
            raise InvalidInputError(
                code="event_hdf5.writer.invalid_staging_path",
                message="staging_path must differ from the publication path",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(destination)},
            )
        staging.parent.mkdir(parents=True, exist_ok=True)
        try:
            staging.touch(exist_ok=False)
        except FileExistsError as exc:
            raise InvalidInputError(
                code="event_hdf5.writer.staging_exists",
                message=f"staging_path already exists: {staging}",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(destination), "staging_path": str(staging)},
            ) from exc
        return staging

    destination.parent.mkdir(parents=True, exist_ok=True)
    while True:
        staging = destination.with_name(f".{destination.name}.{uuid4().hex}.stage")
        try:
            staging.touch(exist_ok=False)
        except FileExistsError:
            continue
        return staging


def _load_hdf5_dependencies(*, source: str) -> tuple[Any, Any]:
    try:
        import h5py
        import numpy as np
    except ImportError as exc:
        dependency = getattr(exc, "name", None) or "h5py/numpy"
        raise OptionalDependencyUnavailableError(
            code="event_hdf5.storage.optional_dependency",
            message=f"{source} requires h5py and numpy; missing {dependency}",
            component=_EVENT_STORAGE_COMPONENT,
            details={"source": source, "dependency": dependency},
        ) from exc
    # HDF5 only decodes Blosc datasets when the plugin filters are registered
    # in the reading process. hdf5plugin registers them on import, so readers
    # that never went through _resolve_compression (a fresh import or
    # validation process) must import it too; absence is fine for gzip files.
    try:
        import hdf5plugin  # noqa: F401 - import registers the HDF5 filters
    except ImportError:
        pass
    return h5py, np


def _resolve_compression(
    requested: str | None,
    opts: int | None,
) -> _CompressionResolution:
    if requested is None:
        return _CompressionResolution(None, {}, "none")

    if requested.startswith("blosc"):
        try:
            with require_extra("event-hdf5-blosc"):
                import hdf5plugin
        except ImportError as exc:
            fallback_label = "gzip:6"
            failure = FailureInfo(
                category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
                code="event_hdf5.compression.hdf5plugin_unavailable",
                message=(
                    "hdf5plugin is unavailable; Blosc:zstd is unsupported and "
                    "gzip:6 was selected"
                ),
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": "hdf5plugin",
                    "requested_codec": requested,
                    "fallback_codec": fallback_label,
                    **_exception_details(exc),
                },
                fallback_allowed=True,
            )
            health_event = make_degraded_health_event(
                failure,
                component=_EVENT_STORAGE_COMPONENT,
                code="event_hdf5.compression.fallback",
                message=(
                    "Blosc:zstd compression is unavailable; event HDF5 is using gzip:6"
                ),
                details={
                    "source": "hdf5plugin",
                    "requested_codec": requested,
                    "fallback_codec": fallback_label,
                    **_exception_details(exc),
                },
            )
            _LOG.warning(
                "hdf5plugin not installed; falling back to gzip-6 for event HDF5"
            )
            return _CompressionResolution(
                "gzip",
                {"compression_opts": 6},
                fallback_label,
                health_event,
            )
        clevel = opts if opts is not None else 3
        try:
            blosc = hdf5plugin.Blosc(cname="zstd", clevel=int(clevel))
        except Exception as exc:
            raise InternalInvariantError(
                code="event_hdf5.compression.plugin_invalid",
                message="hdf5plugin could not construct the requested Blosc codec",
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": "hdf5plugin",
                    "requested_codec": requested,
                },
            ) from exc
        return _CompressionResolution(
            None,
            dict(blosc),
            f"blosc:zstd:{clevel}",
        )

    if requested == "gzip":
        level = opts if opts is not None else 6
        return _CompressionResolution(
            "gzip", {"compression_opts": level}, f"gzip:{level}"
        )

    label = requested if opts is None else f"{requested}:{opts}"
    kw: dict[str, Any] = {}
    if opts is not None:
        kw["compression_opts"] = opts
    return _CompressionResolution(requested, kw, label)


class _EventWriterPublicationCleanupOwner:
    def __init__(
        self,
        writer: "EventHDF5Writer",
        failure: PublicationCleanupError,
    ) -> None:
        self._writer = writer
        self._failure = failure
        self._owner = failure.owner
        expected = writer.path
        expected_absolute = os.path.abspath(os.fspath(expected))
        self._committed_writer_path = any(
            os.path.abspath(os.fspath(path)) == expected_absolute
            for path in failure.committed_paths
        )

    @property
    def committed_paths(self) -> tuple[Path, ...]:
        return self._failure.committed_paths

    @property
    def committed_result(self) -> object | None:
        return self._failure.committed_result

    def retry_cleanup(self) -> None:
        self._owner.retry_cleanup()
        if self._committed_writer_path:
            self._writer._clear_publication_cleanup_failure(self._failure)


class _EventWriterStagingCleanupOwner:
    """Retry removal of a private stage after the published file committed."""

    def __init__(self, writer: "EventHDF5Writer", staging_path: Path) -> None:
        self._writer = writer
        self._staging_path = staging_path

    def retry_cleanup(self) -> None:
        try:
            self._staging_path.unlink()
        except FileNotFoundError:
            pass
        with self._writer._lock:
            if self._writer._staging_path == self._staging_path:
                self._writer._staging_path = None


class EventHDF5Writer:
    """Async event HDF5 writer; publish validated staging data on close.

    Use as a context manager. Queue saturation is bounded by queue_timeout; set
    publish_on_close=False for caller-controlled publication.
    """

    def __init__(
        self,
        path: Path | str,
        metadata: EventEpisodeMetadata,
        *,
        compression: str | None = "blosc:zstd",
        compression_opts: int | None = 3,
        async_writes: bool = True,
        queue_size: int = _DEFAULT_QUEUE_SIZE,
        swmr: bool = True,
        queue_timeout: float = _DEFAULT_QUEUE_TIMEOUT_S,
        close_timeout: float = _DEFAULT_CLOSE_TIMEOUT_S,
        max_batch_rows: int = _INITIAL_CHUNK_ROWS,
        max_event_rows: int = MAX_EVENT_ROWS,
        publish_on_close: bool = True,
        staging_path: Path | str | None = None,
    ) -> None:
        self._path = Path(path)
        self._publish_on_close = bool(publish_on_close)
        self._max_batch_rows = _require_limited_positive_int(
            max_batch_rows,
            name="max_batch_rows",
            path=self._path,
            maximum=MAX_EVENT_BATCH_ROWS,
        )
        self._max_event_rows = _require_limited_positive_int(
            max_event_rows,
            name="max_event_rows",
            path=self._path,
            maximum=MAX_EVENT_ROWS,
        )
        h5py, np = _load_hdf5_dependencies(source="EventHDF5Writer")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        try:
            queue_capacity = operator.index(queue_size)
        except TypeError as exc:
            raise InvalidInputError(
                code="event_hdf5.writer.invalid_queue_size",
                message="queue_size must be a positive integer",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(self._path), "queue_size": queue_size},
            ) from exc
        if queue_capacity <= 0:
            raise InvalidInputError(
                code="event_hdf5.writer.invalid_queue_size",
                message="queue_size must be a positive integer",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(self._path), "queue_size": queue_size},
            )
        self._queue_timeout = float(queue_timeout)
        if not math.isfinite(self._queue_timeout) or self._queue_timeout <= 0:
            raise InvalidInputError(
                code="event_hdf5.writer.invalid_queue_timeout",
                message="queue_timeout must be positive",
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": str(self._path),
                    "queue_timeout": self._queue_timeout,
                },
            )
        self._close_timeout = float(close_timeout)
        if not math.isfinite(self._close_timeout) or self._close_timeout <= 0:
            raise InvalidInputError(
                code="event_hdf5.writer.invalid_close_timeout",
                message="close_timeout must be positive",
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": str(self._path),
                    "close_timeout": self._close_timeout,
                },
            )

        self._staging_path: Path | None = _create_writer_staging_path(
            self._path,
            None if staging_path is None else Path(staging_path),
        )
        self._h5: h5py.File | None = None
        self._np = np
        self._health_events: tuple[HealthEvent, ...] = ()
        self._failure: FailureError | None = None
        self._publication_cleanup_failure: PublicationCleanupError | None = None
        self._state = "open"
        self._close_lock = threading.Lock()
        self._append_lock = threading.Lock()
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._inflight_appends = 0
        self._sentinel_enqueued = False

        libver = "latest" if swmr else None
        open_kwargs: dict[str, Any] = {"mode": "w"}
        if libver is not None:
            open_kwargs["libver"] = libver
        try:
            self._h5 = h5py.File(self._staging_path, **open_kwargs)

            compression_resolution = _resolve_compression(compression, compression_opts)
            self._metadata = _snapshot_event_metadata(metadata, path=self._path)
            self._codec_label = compression_resolution.label
            if compression_resolution.health_event is not None:
                self._append_health_event(compression_resolution.health_event)
            self._write_metadata()
            self._datasets = self._create_event_datasets(
                compression=compression_resolution.compression,
                extra_kwargs=compression_resolution.extra_kwargs,
            )

            # SWMR can only be enabled after dataset creation in h5py.
            if swmr:
                try:
                    _enable_swmr(self._h5)
                except FailureError as exc:
                    failure = _swmr_fallback_failure(exc, path=self._path)
                    if failure is None:
                        raise
                    self._apply_swmr_fallback(failure, cause=exc)
                except (
                    AttributeError,
                    ImportError,
                    NotImplementedError,
                    OSError,
                    RuntimeError,
                    TimeoutError,
                ) as exc:
                    failure = _swmr_fallback_failure(exc, path=self._path)
                    if failure is None:  # pragma: no cover - defensive seam guard
                        raise
                    self._apply_swmr_fallback(failure, cause=exc)
        except BaseException:
            self._discard_staging()
            raise

        self._row_count = 0
        self._async = bool(async_writes)
        self._queue_capacity = queue_capacity
        self._queue: queue.Queue[Any] | None = None
        self._thread: threading.Thread | None = None
        self._dropped_batches = 0
        self._accepted_batch_count = 0
        self._accepted_event_count = 0
        self._written_batch_count = 0
        self._written_event_count = 0
        self._queued_batches = 0
        self._queued_high_water = 0
        self._enqueue_wait_total_s = 0.0
        self._enqueue_wait_max_s = 0.0
        self._enqueue_timeout_count = 0
        self._enqueue_rejection_count = 0
        self._close_drain_elapsed_s = 0.0
        self._last_timestamp: int | None = None
        self._event_digest = _EventDigestAccumulator()
        self._sentinel: object = object()

        if self._async:
            self._queue = queue.Queue(maxsize=queue_capacity)
            self._thread = threading.Thread(
                target=self._drain_loop,
                name=f"EventHDF5Writer:{self._path.name}",
                daemon=True,
            )
            try:
                self._thread.start()
            except BaseException:
                self._discard_staging()
                raise

    @property
    def path(self) -> Path:
        return self._path

    @property
    def row_count(self) -> int:
        with self._lock:
            return self._row_count

    @property
    def max_batch_rows(self) -> int:
        return self._max_batch_rows

    @property
    def max_event_rows(self) -> int:
        return self._max_event_rows

    @property
    def dropped_batches(self) -> int:
        return self._dropped_batches

    @property
    def telemetry(self) -> EventHDF5WriterTelemetry:
        with self._lock:
            return self._telemetry_snapshot_locked()

    def _telemetry_snapshot_locked(self) -> EventHDF5WriterTelemetry:
        return EventHDF5WriterTelemetry(
            queue_capacity=self._queue_capacity,
            accepted_batch_count=self._accepted_batch_count,
            accepted_event_count=self._accepted_event_count,
            written_batch_count=self._written_batch_count,
            written_event_count=self._written_event_count,
            queued_high_water=self._queued_high_water,
            enqueue_wait_total_s=self._enqueue_wait_total_s,
            enqueue_wait_max_s=self._enqueue_wait_max_s,
            enqueue_timeout_count=self._enqueue_timeout_count,
            enqueue_rejection_count=self._enqueue_rejection_count,
            close_drain_elapsed_s=self._close_drain_elapsed_s,
        )

    @property
    def codec(self) -> str:
        return self._codec_label

    @property
    def health_events(self) -> tuple[HealthEvent, ...]:
        return self._health_events

    @property
    def health_status(self) -> HealthStatus:
        if any(event.status is HealthStatus.UNHEALTHY for event in self._health_events):
            return HealthStatus.UNHEALTHY
        if any(event.status is HealthStatus.DEGRADED for event in self._health_events):
            return HealthStatus.DEGRADED
        return HealthStatus.HEALTHY

    def _append_health_event(self, event: HealthEvent) -> None:
        if len(self._health_events) < _MAX_HEALTH_EVENTS:
            self._health_events = (*self._health_events, event)

    def _apply_swmr_fallback(
        self,
        failure: FailureInfo,
        *,
        cause: BaseException,
    ) -> None:
        event = make_degraded_health_event(
            failure,
            component=_EVENT_STORAGE_COMPONENT,
            code="event_hdf5.swmr.fallback",
            message=(
                "SWMR enablement failed; event HDF5 writer is using non-SWMR mode"
            ),
            details={
                "source": str(self._path),
                "requested_mode": "swmr",
                "selected_mode": "non-swmr",
                **_exception_details(
                    cause.__cause__
                    if isinstance(cause, FailureError) and cause.__cause__ is not None
                    else cause
                ),
            },
        )
        self._append_health_event(event)
        _LOG.warning(
            "SWMR unavailable for %s; using non-SWMR mode (%s): %s",
            self._path,
            failure.category.value,
            _bounded_text(cause),
        )

    def append(self, events: "torch.Tensor | np.ndarray") -> None:
        """Validate synchronously; defer HDF5 writes to the background thread when
        async_writes is enabled.

        Append an (N, 4) batch. Nothing is returned because async batches may
        still be queued; read :attr:`row_count` for the persisted rows.
        """
        # Serialize producer ordering without coupling producers to the close
        # lifecycle mutex. A blocked queue insertion therefore cannot prevent
        # close() from marking the writer closing and waking that producer.
        with self._append_lock:
            self._begin_append()
            try:
                input_rows = _peek_event_rows(events)
                if input_rows is not None and input_rows > self._max_batch_rows:
                    raise InvalidInputError(
                        code="event_hdf5.writer.batch_too_large",
                        message=(
                            f"events batch has {input_rows} rows, exceeding "
                            f"max_batch_rows {self._max_batch_rows}"
                        ),
                        component=_EVENT_STORAGE_COMPONENT,
                        details={
                            "source": str(self._path),
                            "rows": input_rows,
                            "max_batch_rows": self._max_batch_rows,
                        },
                    )
                if input_rows is not None:
                    self._require_total_capacity(input_rows)
                arr = _coerce_event_array(events, self._np)
                if int(arr.shape[0]) > self._max_batch_rows:
                    raise InvalidInputError(
                        code="event_hdf5.writer.batch_too_large",
                        message=(
                            f"events batch has {int(arr.shape[0])} rows, exceeding "
                            f"max_batch_rows {self._max_batch_rows}"
                        ),
                        component=_EVENT_STORAGE_COMPONENT,
                        details={
                            "source": str(self._path),
                            "rows": int(arr.shape[0]),
                            "max_batch_rows": self._max_batch_rows,
                        },
                    )
                self._require_total_capacity(int(arr.shape[0]))
                if arr.shape[0] == 0:
                    return

                with self._lock:
                    previous_timestamp = self._last_timestamp
                try:
                    last_timestamp = _validate_event_array(
                        arr,
                        self._metadata,
                        previous_timestamp,
                        self._np,
                        path=self._path,
                    )
                except InvalidInputError as exc:
                    failure = self._record_failure(
                        exc,
                        operation="semantic_validation",
                    )
                    if failure is exc:
                        raise
                    raise failure from exc
                assert last_timestamp is not None

                if self._async:
                    assert self._queue is not None
                    self._enqueue_with_backpressure(arr)
                    with self._lock:
                        self._last_timestamp = last_timestamp
                else:
                    try:
                        self._write_batch(arr)
                    except Exception as exc:  # noqa: BLE001 - writer boundary
                        failure = self._record_failure(exc, operation="sync_write")
                        if failure is exc:
                            raise
                        raise failure from exc
                    with self._lock:
                        self._accepted_batch_count += 1
                        self._accepted_event_count += int(arr.shape[0])
                        self._last_timestamp = last_timestamp
            finally:
                self._end_append()

    def _require_total_capacity(self, incoming_rows: int) -> None:
        if incoming_rows < 0:
            raise InvalidInputError(
                code="event_hdf5.writer.invalid_events",
                message="event row count must be non-negative",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(self._path), "rows": incoming_rows},
            )
        with self._lock:
            accepted_rows = self._accepted_event_count
        observed = accepted_rows + incoming_rows
        if observed > self._max_event_rows:
            raise InvalidInputError(
                code="event_hdf5.writer.event_limit",
                message=(
                    f"event row total {observed} exceeds max_event_rows "
                    f"{self._max_event_rows}"
                ),
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": str(self._path),
                    "accepted_rows": accepted_rows,
                    "incoming_rows": incoming_rows,
                    "max_event_rows": self._max_event_rows,
                },
            )

    def _begin_append(self) -> None:
        with self._condition:
            failure = self._failure
            state = self._state
            if failure is not None:
                raise failure
            if state != "open":
                raise InternalInvariantError(
                    code="event_hdf5.writer.closed",
                    message="EventHDF5Writer is closed",
                    component=_EVENT_STORAGE_COMPONENT,
                    details={"source": str(self._path), "state": state},
                )
            self._inflight_appends += 1

    def _end_append(self) -> None:
        with self._condition:
            if self._inflight_appends <= 0:
                raise InternalInvariantError(
                    code="event_hdf5.writer.inflight_appends",
                    message="EventHDF5Writer append lifecycle underflow",
                    component=_EVENT_STORAGE_COMPONENT,
                    details={"source": str(self._path)},
                )
            self._inflight_appends -= 1
            self._condition.notify_all()

    def finalize(self, metadata: EventEpisodeMetadata | None = None) -> None:
        """Replace provisional metadata before final count, digest, and publication;
        with publish_on_close=False, leave the closed stage for publish()."""
        self._close(metadata)

    def close(self) -> None:
        self._close(None)

    def publish(self) -> Path:
        with self._close_lock:
            with self._lock:
                state = self._state
                failure = self._failure
                publication_cleanup_failure = self._publication_cleanup_failure
            if publication_cleanup_failure is not None:
                raise publication_cleanup_failure
            if failure is not None:
                raise failure
            if state != "closed":
                raise InternalInvariantError(
                    code="event_hdf5.writer.publish_before_close",
                    message="EventHDF5Writer must be closed before publish",
                    component=_EVENT_STORAGE_COMPONENT,
                    details={"source": str(self._path), "state": state},
                )
            try:
                return self._publish_staged()
            except PublicationCleanupError as exc:
                # Publication has its own identity-bound cleanup owner.  Keep
                # that typed operational failure intact; the writer only owns
                # its external source stage, which is safe to discard here.
                self._discard_staging()
                self._remember_publication_cleanup_failure(exc)
                raise
            except Exception as exc:  # noqa: BLE001 - publication boundary
                failure = self._record_failure(exc, operation="publication")
                self._discard_staging()
                if failure is exc:
                    raise
                raise failure from exc

    def _close(self, final_metadata: EventEpisodeMetadata | None) -> None:
        with self._close_lock:
            close_started = time.monotonic()
            deadline = close_started + self._close_timeout
            with self._condition:
                if self._state == "closed":
                    publication_cleanup_failure = self._publication_cleanup_failure
                    if publication_cleanup_failure is not None:
                        raise publication_cleanup_failure
                    failure = self._failure
                    if failure is None:
                        return
                    raise failure
                else:
                    self._state = "closing"
                    self._condition.notify_all()

            try:
                self._wait_for_inflight_appends(deadline)
                self._stop_worker(deadline=deadline)
            except FailureError:
                self._record_close_drain_elapsed(close_started)
                # A timed-out worker may still own the HDF5 handle.  Its
                # eventual exit performs staging cleanup; never close that
                # handle concurrently from this thread.
                with self._lock:
                    inflight_appends = self._inflight_appends
                if inflight_appends == 0 and (
                    self._thread is None or not self._thread.is_alive()
                ):
                    self._discard_staging()
                    self._mark_closed()
                raise
            self._record_close_drain_elapsed(close_started)
            failure = self._failure
            if failure is not None:
                self._discard_staging()
                self._mark_closed()
                raise failure

            if time.monotonic() >= deadline:
                timeout = TimeoutError(
                    "EventHDF5Writer close deadline expired before finalization"
                )
                failure = self._record_failure(
                    timeout,
                    operation="close_timeout",
                )
                self._discard_staging()
                self._mark_closed()
                raise failure

            try:
                self._finalize_to_staging(final_metadata)
                if self._publish_on_close:
                    self._publish_staged()
            except PublicationCleanupError as exc:
                self._discard_staging()
                self._mark_closed()
                self._remember_publication_cleanup_failure(exc)
                raise
            except Exception as exc:  # noqa: BLE001 - finalization boundary
                failure = self._record_failure(exc, operation="finalization")
                self._discard_staging()
                self._mark_closed()
                if failure is exc:
                    raise
                raise failure from exc
            else:
                self._mark_closed()

    def _wait_for_inflight_appends(self, deadline: float) -> None:
        timed_out = False
        with self._condition:
            while self._inflight_appends > 0:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    timed_out = True
                    break
                self._condition.wait(timeout=remaining)
        if timed_out:
            timeout = TimeoutError(
                "EventHDF5Writer in-flight appenders did not settle before "
                "the close deadline"
            )
            raise self._record_failure(
                timeout,
                operation="append_drain_timeout",
            )

    def _record_close_drain_elapsed(self, started: float) -> None:
        elapsed = max(0.0, time.monotonic() - started)
        with self._lock:
            self._close_drain_elapsed_s = elapsed

    def abort(self, cause: BaseException | None = None) -> None:
        del cause
        self._abort_after_body_error()

    def __enter__(self) -> "EventHDF5Writer":
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        if exc_type is None:
            self.close()
        else:
            self._abort_after_body_error()

    def _enqueue_with_backpressure(self, arr: "np.ndarray") -> None:
        assert self._queue is not None
        started = time.monotonic()
        deadline = started + self._queue_timeout
        with self._condition:
            while True:
                failure = self._failure
                state = self._state
                if failure is not None or state != "open":
                    self._record_enqueue_wait_and_rejection_locked(
                        time.monotonic() - started,
                        timeout=False,
                    )
                    if failure is not None:
                        raise failure
                    raise InternalInvariantError(
                        code="event_hdf5.writer.closed",
                        message="EventHDF5Writer is closed",
                        component=_EVENT_STORAGE_COMPONENT,
                        details={"source": str(self._path), "state": state},
                    )

                try:
                    self._queue.put_nowait(arr)
                except queue.Full:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        timeout = TimeoutError(
                            f"EventHDF5Writer queue remained full for "
                            f"{self._queue_timeout:.3f}s"
                        )
                        self._record_enqueue_wait_and_rejection_locked(
                            time.monotonic() - started,
                            timeout=True,
                        )
                        failure = self._backpressure_failure(timeout)
                        # No `from` here on purpose: `_record_failure_locked`
                        # returns an exception already chained to the
                        # synthesized `timeout`, which is the cause callers are
                        # told to read. Adding `from` would replace that with
                        # the raw `queue.Full`.
                        raise self._record_failure_locked(
                            timeout,
                            operation="backpressure",
                            failure=failure,
                        )
                    self._condition.wait(timeout=min(0.05, remaining))
                    continue

                waited = time.monotonic() - started
                self._enqueue_wait_total_s += waited
                self._enqueue_wait_max_s = max(self._enqueue_wait_max_s, waited)
                self._accepted_batch_count += 1
                self._accepted_event_count += int(arr.shape[0])
                self._queued_batches += 1
                self._queued_high_water = max(
                    self._queued_high_water,
                    self._queued_batches,
                )
                self._condition.notify_all()
                return

    def _record_enqueue_wait_and_rejection_locked(
        self,
        waited: float,
        *,
        timeout: bool,
    ) -> None:
        self._enqueue_wait_total_s += waited
        self._enqueue_wait_max_s = max(self._enqueue_wait_max_s, waited)
        self._enqueue_rejection_count += 1
        if timeout:
            self._enqueue_timeout_count += 1

    def _record_dequeued_batch_locked(self) -> None:
        if self._queued_batches <= 0:
            raise InternalInvariantError(
                code="event_hdf5.writer.queue_occupancy",
                message="EventHDF5Writer queue occupancy underflow",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(self._path)},
            )
        self._queued_batches -= 1

    def _drain_loop(self) -> None:
        assert self._queue is not None
        while True:
            with self._condition:
                while True:
                    if self._failure is not None:
                        item = self._sentinel
                        break
                    try:
                        item = self._queue.get_nowait()
                    except queue.Empty:
                        self._condition.wait(timeout=0.05)
                        continue
                    if item is not self._sentinel:
                        self._record_dequeued_batch_locked()
                    self._condition.notify_all()
                    break
            if item is self._sentinel:
                if self._failure is not None:
                    self._discard_staging()
                return
            if self._failure is not None:
                self._discard_staging()
                return
            try:
                self._write_batch(item)
            except Exception as exc:  # noqa: BLE001 - worker boundary
                self._record_failure(exc, operation="background_write")
                _LOG.exception(
                    "EventHDF5Writer background write failed for %s",
                    self._path.name,
                )
                self._discard_staging()
                return

    def _write_batch(self, arr: "np.ndarray") -> None:
        n = int(arr.shape[0])
        if n == 0:
            return
        assert self._h5 is not None
        with self._lock:
            start = self._row_count
            new_end = start + n
            if new_end > self._max_event_rows:
                raise InternalInvariantError(
                    code="event_hdf5.writer.event_limit",
                    message=(
                        f"write would exceed max_event_rows {self._max_event_rows}"
                    ),
                    component=_EVENT_STORAGE_COMPONENT,
                    details={
                        "source": str(self._path),
                        "row_count": start,
                        "incoming_rows": n,
                        "max_event_rows": self._max_event_rows,
                    },
                )
            for col_idx, (name, dtype) in enumerate(EVENT_HDF5_COLUMNS):
                ds = self._datasets[name]
                self._write_column(
                    ds,
                    start=start,
                    end=new_end,
                    values=arr[:, col_idx].astype(dtype, copy=False),
                )
            self._event_digest.update(arr, self._np)
            self._row_count = new_end
            self._written_batch_count += 1
            self._written_event_count = new_end

    @staticmethod
    def _write_column(
        ds: "h5py.Dataset",
        *,
        start: int,
        end: int,
        values: "np.ndarray",
    ) -> None:
        ds.resize((end,))
        ds[start:end] = values
        ds.flush()

    def _record_failure(
        self,
        exc: BaseException,
        *,
        operation: str,
        failure: FailureError | None = None,
    ) -> FailureError:
        if failure is None:
            failure = _writer_failure_error(
                exc,
                path=self._path,
                operation=operation,
            )
        with self._condition:
            return self._record_failure_locked(
                exc,
                operation=operation,
                failure=failure,
            )

    def _record_failure_locked(
        self,
        exc: BaseException,
        *,
        operation: str,
        failure: FailureError,
    ) -> FailureError:
        if self._failure is not None:
            return self._failure
        self._failure = failure
        cause = (
            exc.__cause__
            if isinstance(exc, FailureError) and exc.__cause__ is not None
            else exc
        )
        event = HealthEvent(
            status=HealthStatus.UNHEALTHY,
            severity=HealthSeverity.ERROR,
            component=_EVENT_STORAGE_COMPONENT,
            code="event_hdf5.writer.failure",
            message=failure.failure_info.message,
            details={
                "source": str(self._path),
                "operation": operation,
                "failure_category": failure.failure_info.category.value,
                **_exception_details(cause),
            },
            failure=failure.failure_info,
        )
        self._append_health_event(event)
        self._condition.notify_all()
        return failure

    def _backpressure_failure(self, cause: TimeoutError) -> FailureError:
        assert self._queue is not None
        info = FailureInfo(
            category=FailureCategory.TRANSIENT_EXTERNAL_FAILURE,
            code="event_hdf5.writer.backpressure",
            message=(
                f"EVENT_HDF5 writer queue remained full for {self._path}: "
                f"{self._queue_timeout:.3f}s deadline exceeded"
            ),
            component=_EVENT_STORAGE_COMPONENT,
            details={
                "source": str(self._path),
                "queue_size": self._queue.maxsize,
                "queue_timeout": self._queue_timeout,
                **_exception_details(cause),
            },
            retryable=True,
        )
        failure = EventHDF5BackpressureError(info)
        failure.__cause__ = cause
        failure.__suppress_context__ = True
        return failure

    def _stop_worker(self, *, deadline: float | None = None) -> None:
        if not self._async or self._queue is None or self._thread is None:
            return

        stop_deadline = (
            time.monotonic() + self._close_timeout if deadline is None else deadline
        )
        while self._thread.is_alive():
            with self._condition:
                if not self._sentinel_enqueued and self._inflight_appends == 0:
                    try:
                        self._queue.put_nowait(self._sentinel)
                    except queue.Full:
                        if self._failure is not None:
                            try:
                                item = self._queue.get_nowait()
                            except queue.Empty:
                                pass
                            else:
                                if item is not self._sentinel:
                                    self._record_dequeued_batch_locked()
                                self._condition.notify_all()
                    else:
                        self._sentinel_enqueued = True
                        self._condition.notify_all()

            remaining = stop_deadline - time.monotonic()
            if remaining <= 0:
                timeout = TimeoutError(
                    "EventHDF5Writer worker did not stop before the close deadline"
                )
                failure = self._record_failure(timeout, operation="close_timeout")
                with self._condition:
                    if not self._sentinel_enqueued and self._inflight_appends == 0:
                        try:
                            self._queue.put_nowait(self._sentinel)
                            self._sentinel_enqueued = True
                            self._condition.notify_all()
                        except queue.Full:
                            pass
                raise failure
            self._thread.join(timeout=min(0.05, remaining))

    def _finalize_to_staging(
        self,
        final_metadata: EventEpisodeMetadata | None,
    ) -> None:
        assert self._h5 is not None
        with self._lock:
            if final_metadata is not None:
                try:
                    self._metadata = _snapshot_event_metadata(
                        final_metadata,
                        path=self._path,
                    )
                except ResourceLimitError as exc:
                    raise _metadata_limit_failure(self._path, exc) from exc
            event_count = self._validated_event_count()
            if self._accepted_event_count != event_count:
                raise InternalInvariantError(
                    code="event_hdf5.writer.accepted_count",
                    message=(
                        "accepted event count does not match written count: "
                        f"{self._accepted_event_count} != {event_count}"
                    ),
                    component=_EVENT_STORAGE_COMPONENT,
                    details={
                        "source": str(self._path),
                        "accepted_event_count": self._accepted_event_count,
                        "event_count": event_count,
                    },
                )
            if self._event_digest.count != event_count:
                raise InternalInvariantError(
                    code="event_hdf5.writer.digest_count",
                    message=(
                        "event digest count does not match written count: "
                        f"{self._event_digest.count} != {event_count}"
                    ),
                    component=_EVENT_STORAGE_COMPONENT,
                    details={
                        "source": str(self._path),
                        "digest_count": self._event_digest.count,
                        "event_count": event_count,
                    },
                )
            event_sha256 = self._event_digest.hexdigest(self._np)
            staged_event_sha256 = self._validate_staged_events(event_count)
            if staged_event_sha256 != event_sha256:
                raise IntegrityFailureError(
                    code="event_hdf5.writer.staged_digest",
                    message=(
                        "staged event digest does not match accepted-input digest"
                    ),
                    component=_EVENT_STORAGE_COMPONENT,
                    details={
                        "source": str(self._path),
                        "accepted_event_sha256": event_sha256,
                        "staged_event_sha256": staged_event_sha256,
                    },
                )
            final_attrs = _metadata_values(self._metadata)
            final_attrs.update(
                {
                    "compression": self._codec_label,
                    "event_count": event_count,
                    _EVENT_DIGEST_METADATA_KEY: event_sha256,
                    "dropped_batches": self._dropped_batches,
                    "writer_telemetry": self._telemetry_snapshot_locked().to_dict(),
                }
            )
            try:
                prepared = _prepare_metadata_attrs(final_attrs, path=self._path)
            except ResourceLimitError as exc:
                raise _metadata_limit_failure(self._path, exc) from exc
            metadata = self._h5["/metadata"].attrs
            # Optional values omitted from final metadata must remove their
            # provisional values. ``finalize(None)`` keeps the initial snapshot.
            for key in _OPTIONAL_UNKNOWN_METADATA_FIELDS:
                if key not in final_attrs and key in metadata:
                    del metadata[key]
            for key, value in prepared:
                metadata[key] = value
            _validate_hdf5_metadata_storage(metadata, path=self._path)
            self._h5.flush()
            self._h5.close()
            self._h5 = None
            assert self._staging_path is not None
            try:
                require_file_size(
                    self._staging_path,
                    resource="EVENT_HDF5 staging file",
                    max_bytes=MAX_EVENT_HDF5_BYTES,
                )
            except (ResourceLimitError, ValueError) as exc:
                raise InvalidInputError(
                    code="event_hdf5.writer.file_limit",
                    message=str(exc),
                    component=_EVENT_STORAGE_COMPONENT,
                    details={
                        "source": str(self._path),
                        "max_bytes": MAX_EVENT_HDF5_BYTES,
                    },
                ) from exc

    def _publish_staged(self) -> Path:
        with self._lock:
            staging_path = self._staging_path
        if staging_path is None:
            return self._path

        published = ArtifactPublication.file(self._path).publish_staged(staging_path)
        try:
            staging_path.unlink()
        except FileNotFoundError:
            pass
        except OSError as exc:
            raise PublicationCleanupError(
                "event HDF5 committed but its staging file cleanup is pending",
                owner=_EventWriterStagingCleanupOwner(self, staging_path),
                committed_paths=(published,),
                committed_result=published,
                errors=(exc,),
            ) from exc
        with self._lock:
            if self._staging_path == staging_path:
                self._staging_path = None
        return published

    def _validated_event_count(self) -> int:
        lengths = {name: int(ds.shape[0]) for name, ds in self._datasets.items()}
        if len(set(lengths.values())) != 1:
            raise InternalInvariantError(
                code="event_hdf5.writer.column_lengths",
                message=f"event columns have unequal lengths: {lengths}",
                component=_EVENT_STORAGE_COMPONENT,
                details={"source": str(self._path), "lengths": lengths},
            )
        event_count = next(iter(lengths.values()))
        if event_count > self._max_event_rows:
            raise InternalInvariantError(
                code="event_hdf5.writer.event_limit",
                message=(
                    f"event count {event_count} exceeds max_event_rows "
                    f"{self._max_event_rows}"
                ),
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": str(self._path),
                    "event_count": event_count,
                    "max_event_rows": self._max_event_rows,
                },
            )
        if event_count != self._row_count:
            raise InternalInvariantError(
                code="event_hdf5.writer.row_count",
                message=(
                    "event column length does not match row_count: "
                    f"{event_count} != {self._row_count}"
                ),
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": str(self._path),
                    "event_count": event_count,
                    "row_count": self._row_count,
                },
            )
        if event_count != self._written_event_count:
            raise InternalInvariantError(
                code="event_hdf5.writer.written_count",
                message=(
                    "event column length does not match written_event_count: "
                    f"{event_count} != {self._written_event_count}"
                ),
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": str(self._path),
                    "event_count": event_count,
                    "written_event_count": self._written_event_count,
                },
            )
        return event_count

    def _validate_staged_events(self, event_count: int) -> str:
        staged_digest = _EventDigestAccumulator()
        previous_timestamp: int | None = None
        for start in range(0, event_count, _EVENT_VALIDATION_CHUNK_ROWS):
            end = min(start + _EVENT_VALIDATION_CHUNK_ROWS, event_count)
            chunk = self._np.empty((end - start, len(EVENT_HDF5_COLUMNS)), dtype="<i8")
            for column_index, (name, _dtype) in enumerate(EVENT_HDF5_COLUMNS):
                chunk[:, column_index] = self._datasets[name][start:end]
            previous_timestamp = _validate_event_array(
                chunk,
                self._metadata,
                previous_timestamp,
                self._np,
                path=self._path,
            )
            staged_digest.update(chunk, self._np)
        if staged_digest.count != event_count:
            raise InternalInvariantError(
                code="event_hdf5.writer.staged_digest_count",
                message=(
                    "staged event digest count does not match written count: "
                    f"{staged_digest.count} != {event_count}"
                ),
                component=_EVENT_STORAGE_COMPONENT,
                details={
                    "source": str(self._path),
                    "staged_digest_count": staged_digest.count,
                    "event_count": event_count,
                },
            )
        return staged_digest.hexdigest(self._np)

    def _abort_after_body_error(self) -> None:
        with self._close_lock:
            deadline = time.monotonic() + self._close_timeout
            with self._condition:
                if self._state == "closed":
                    has_staging = self._staging_path is not None
                else:
                    has_staging = False
                    self._state = "closing"
                    self._condition.notify_all()
            if has_staging:
                self._discard_staging()
            if self._state == "closed":
                return
            try:
                self._wait_for_inflight_appends(deadline)
                if self._queue is not None:
                    with self._condition:
                        while True:
                            try:
                                item = self._queue.get_nowait()
                            except queue.Empty:
                                break
                            if item is not self._sentinel:
                                self._record_dequeued_batch_locked()
                            else:
                                self._sentinel_enqueued = False
                        self._condition.notify_all()
                self._stop_worker(deadline=deadline)
            except FailureError:
                _LOG.exception(
                    "EventHDF5Writer worker did not stop while aborting %s",
                    self._path.name,
                )
                return
            self._discard_staging()
            self._mark_closed()

    def _mark_closed(self) -> None:
        with self._condition:
            self._state = "closed"
            self._condition.notify_all()

    def _remember_publication_cleanup_failure(
        self, failure: PublicationCleanupError
    ) -> None:
        owner = _EventWriterPublicationCleanupOwner(self, failure)
        failure.owner = owner
        failure.cleanup_owner = owner
        with self._lock:
            self._publication_cleanup_failure = failure

    def _clear_publication_cleanup_failure(
        self, failure: PublicationCleanupError
    ) -> None:
        with self._lock:
            if self._publication_cleanup_failure is failure:
                self._publication_cleanup_failure = None

    def _discard_staging(self) -> None:
        h5 = self._h5
        self._h5 = None
        if h5 is not None:
            try:
                h5.close()
            except (OSError, RuntimeError, ValueError) as exc:
                self._record_failure(exc, operation="cleanup")
                _LOG.exception("Failed to close staging HDF5 for %s", self._path.name)
        staging_path = self._staging_path
        if staging_path is not None:
            try:
                staging_path.unlink()
            except FileNotFoundError:
                self._staging_path = None
            except OSError as exc:
                self._record_failure(exc, operation="cleanup")
                _LOG.exception("Failed to remove staging HDF5 for %s", self._path.name)
            else:
                self._staging_path = None

    def _write_metadata(self) -> None:
        assert self._h5 is not None
        group = self._h5.create_group("metadata")
        self._write_metadata_attrs(
            group,
            self._metadata,
            path=self._path,
            extra={"compression": self._codec_label},
        )

    @staticmethod
    def _write_metadata_attrs(
        group: Any,
        metadata: EventEpisodeMetadata,
        *,
        path: Path,
        extra: dict[str, object] | None = None,
    ) -> None:
        values = _metadata_values(metadata)
        if extra:
            values.update(extra)
        prepared = _prepare_metadata_attrs(values, path=path)
        for key, value in prepared:
            group.attrs[key] = value

    def _create_event_datasets(
        self,
        *,
        compression: Any,
        extra_kwargs: dict[str, Any],
    ) -> dict[str, "h5py.Dataset"]:
        assert self._h5 is not None
        events_group = self._h5.create_group("events")
        datasets: dict[str, h5py.Dataset] = {}
        for name, dtype in EVENT_HDF5_COLUMNS:
            kw = dict(extra_kwargs)
            if compression is not None:
                kw["compression"] = compression
            ds = events_group.create_dataset(
                name,
                shape=(0,),
                maxshape=(None,),
                dtype=dtype,
                chunks=(_INITIAL_CHUNK_ROWS,),
                **kw,
            )
            datasets[name] = ds
        return datasets


def _check_event_full_read_budget(
    h5: Any,
    np: Any,
    *,
    path: Path,
    strict_schema: bool = False,
) -> int:
    try:
        events_group = h5["events"]
    except (KeyError, TypeError, ValueError):
        # The schema validator owns missing-group diagnostics.
        if strict_schema:
            raise ValueError(f"EVENT_HDF5 events group is missing for {path}")
        return 0

    aggregate = ByteBudget(
        f"EVENT_HDF5 full-read decoded event columns for {path}",
        MAX_DECODED_AGGREGATE_BYTES,
    )
    event_count: int | None = None
    for name, dtype in EVENT_HDF5_COLUMNS:
        try:
            dataset = events_group[name]
            shape = tuple(dataset.shape)
            dataset_dtype = np.dtype(dataset.dtype)
        except (AttributeError, KeyError, TypeError, ValueError):
            # Defer malformed schema errors to the validator so callers keep
            # its typed failure classification and diagnostics.
            if strict_schema:
                raise ValueError(
                    f"EVENT_HDF5 events/{name} is missing or unreadable for {path}"
                )
            return 0
        if len(shape) != 1 or dataset_dtype != np.dtype(dtype):
            if strict_schema:
                raise ValueError(
                    f"EVENT_HDF5 events/{name} has invalid shape or dtype for {path}"
                )
            return 0
        length = operator.index(shape[0])
        if event_count is None:
            event_count = length
        elif length != event_count:
            if strict_schema:
                raise ValueError(
                    f"EVENT_HDF5 event columns have mismatched lengths for {path}"
                )
            return 0
        if length > MAX_EVENT_ROWS:
            # The schema validator emits the established row-limit failure;
            # the direct compatibility helper checks it immediately below.
            return length
        column_bytes = checked_shape_nbytes(
            shape,
            dataset_dtype,
            resource=f"EVENT_HDF5 full-read events/{name} decoded bytes for {path}",
            max_elements=min(MAX_ARRAY_ELEMENTS, MAX_EVENT_ROWS),
            max_bytes=MAX_ARRAY_BYTES,
            max_rank=1,
        )
        aggregate.consume(column_bytes)
    return 0 if event_count is None else event_count


def _read_event_hdf5_handle(h5: Any, np: Any) -> dict[str, Any]:
    metadata_group = h5["metadata"]
    _validate_hdf5_metadata_storage(metadata_group.attrs, path=Path("<open-hdf5>"))
    event_count = _check_event_full_read_budget(
        h5,
        np,
        path=Path("<open-hdf5>"),
        strict_schema=True,
    )
    if event_count > MAX_EVENT_ROWS:
        raise ResourceLimitError(
            "EVENT_HDF5 event rows",
            limit=MAX_EVENT_ROWS,
            observed=event_count,
        )
    metadata = dict(metadata_group.attrs)
    for key in _JSON_ENCODED_METADATA_FIELDS:
        if key not in metadata:
            continue
        raw = metadata[key]
        if isinstance(raw, (bytes, bytearray)):
            raw = raw.decode("utf-8")
        try:
            metadata[key] = json.loads(raw)
        except (TypeError, ValueError):
            # Leave raw if it isn't valid JSON — better than silently dropping
            # the attr.
            pass
    return {
        "t": np.asarray(h5["events/t"][:]),
        "x": np.asarray(h5["events/x"][:]),
        "y": np.asarray(h5["events/y"][:]),
        "p": np.asarray(h5["events/p"][:]),
        "metadata": metadata,
    }


def _raise_iterator_integrity(
    path: Path,
    message: str,
    *,
    cause: BaseException | None = None,
) -> None:
    from srb.core.sensor.event_camera.storage_validation import (
        _integrity_failure,
        _raise_failure,
    )

    _raise_failure(
        _integrity_failure(path, message, cause=cause),
        cause=cause,
    )


def _validate_iterator_schema(
    h5: Any,
    np: Any,
    *,
    path: Path,
) -> tuple[dict[str, Any], int, str]:
    try:
        events_group = h5["events"]
        metadata_group = h5["metadata"]
        dataset_names = set(events_group.keys())
    except (KeyError, TypeError, ValueError) as exc:
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: missing groups",
            cause=exc,
        )

    expected_names = {name for name, _ in EVENT_HDF5_COLUMNS}
    if dataset_names != expected_names or not hasattr(metadata_group, "keys"):
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: invalid groups",
        )

    datasets: dict[str, Any] = {}
    lengths: dict[str, int] = {}
    for name, dtype in EVENT_HDF5_COLUMNS:
        try:
            dataset = events_group[name]
            dataset_dtype = np.dtype(dataset.dtype)
            dataset_ndim = int(dataset.ndim)
            dataset_length = int(dataset.shape[0])
        except (AttributeError, KeyError, IndexError, TypeError, ValueError) as exc:
            _raise_iterator_integrity(
                path,
                f"EVENT_HDF5 schema validation failed for {path}: "
                f"invalid events/{name} dataset",
                cause=exc,
            )
        if dataset_ndim != 1 or dataset_dtype != np.dtype(dtype):
            _raise_iterator_integrity(
                path,
                f"EVENT_HDF5 schema validation failed for {path}: "
                f"invalid events/{name} dtype or shape",
            )
        if dataset_length > MAX_EVENT_ROWS:
            _raise_iterator_integrity(
                path,
                f"EVENT_HDF5 schema validation failed for {path}: "
                f"events/{name} length {dataset_length} exceeds "
                f"the limit {MAX_EVENT_ROWS}",
            )
        datasets[name] = dataset
        lengths[name] = dataset_length

    if len(set(lengths.values())) != 1:
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: "
            f"unequal event lengths {lengths}",
        )

    try:
        _validate_hdf5_metadata_storage(metadata_group.attrs, path=path)
    except (ResourceLimitError, ValueError) as exc:
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 metadata exceeds resource limits for {path}: {exc}",
            cause=exc,
        )
    attrs = dict(metadata_group.attrs)
    try:
        raw_schema_version = attrs["schema_version"]
        raw_event_count = attrs["event_count"]
    except KeyError as exc:
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: missing final metadata",
            cause=exc,
        )
    integer_types = (int, np.integer)
    boolean_types = (bool, np.bool_)
    if isinstance(raw_schema_version, boolean_types) or not isinstance(
        raw_schema_version,
        integer_types,
    ):
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: "
            "schema_version must be an integer scalar",
        )
    if isinstance(raw_event_count, boolean_types) or not isinstance(
        raw_event_count,
        integer_types,
    ):
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: "
            "event_count must be an integer scalar",
        )
    schema_version = int(raw_schema_version)
    event_count = int(raw_event_count)
    if schema_version != EVENT_HDF5_VERSION or event_count < 0:
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: "
            "invalid schema version or event count",
        )
    if event_count != next(iter(lengths.values())):
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: "
            "event count does not match datasets",
        )

    expected_digest = attrs.get(_EVENT_DIGEST_METADATA_KEY)
    if isinstance(expected_digest, (bytes, bytearray)):
        expected_digest = expected_digest.decode("ascii", errors="replace")
    expected_digest = str(expected_digest)
    if len(expected_digest) != hashlib.sha256().digest_size * 2 or any(
        character not in "0123456789abcdefABCDEF" for character in expected_digest
    ):
        _raise_iterator_integrity(
            path,
            f"EVENT_HDF5 schema validation failed for {path}: invalid event digest",
        )
    return datasets, event_count, expected_digest.lower()


def iter_event_hdf5(
    path: Path | str,
    chunk_size: int,
):
    """Yield chunks of at most `chunk_size` rows from one published snapshot.

    The whole-file digest is verified only when iteration reaches the end.
    """
    try:
        if isinstance(chunk_size, bool):
            raise TypeError
        chunk_bound = operator.index(chunk_size)
    except TypeError as exc:
        raise InvalidInputError(
            code="event_hdf5.reader.invalid_chunk_size",
            message="chunk_size must be a positive integer",
            component=_EVENT_STORAGE_COMPONENT,
            details={"chunk_size": chunk_size},
        ) from exc
    if chunk_bound <= 0 or chunk_bound > MAX_EVENT_BATCH_ROWS:
        raise InvalidInputError(
            code="event_hdf5.reader.invalid_chunk_size",
            message=(
                "chunk_size must be a positive integer no greater than "
                f"{MAX_EVENT_BATCH_ROWS}"
            ),
            component=_EVENT_STORAGE_COMPONENT,
            details={
                "chunk_size": chunk_size,
                "maximum": MAX_EVENT_BATCH_ROWS,
            },
        )

    h5py, np = _load_hdf5_dependencies(source="iter_event_hdf5")
    source_path = Path(path)
    from srb.core.sensor.event_camera.storage_validation import (
        _internal_failure,
        _invalid_path_failure,
        _raise_failure,
        _schema_integrity_failure,
        _uncontained_storage_issues,
    )

    with ArtifactPublication.file(source_path).open_snapshot() as snapshot:
        if snapshot is None:
            _raise_failure(
                _invalid_path_failure(
                    source_path,
                    f"EVENT_HDF5 schema validation failed for {path}: "
                    "file does not exist",
                )
            )
        try:
            _require_event_hdf5_file_size(snapshot.file, path=source_path)
            with h5py.File(snapshot.file, "r") as h5:
                # Nothing is read before every member is known to live in
                # this snapshot.
                storage_issues = _uncontained_storage_issues(h5py, h5)
                if storage_issues:
                    _raise_failure(
                        _schema_integrity_failure(source_path, storage_issues)
                    )
                datasets, event_count, expected_digest = _validate_iterator_schema(
                    h5,
                    np,
                    path=source_path,
                )
                accumulator = _EventDigestAccumulator()
                for start in range(0, event_count, chunk_bound):
                    end = min(start + chunk_bound, event_count)
                    chunk = {
                        name: np.asarray(dataset[start:end])
                        for name, dataset in datasets.items()
                    }
                    accumulator.update_columns(chunk, np)
                    yield chunk
                if accumulator.hexdigest(np) != expected_digest:
                    _raise_iterator_integrity(
                        source_path,
                        f"EVENT_HDF5 digest validation failed for {path}",
                    )
        except FailureError:
            raise
        except (OSError, ValueError) as exc:
            _raise_iterator_integrity(
                source_path,
                f"not a readable EVENT_HDF5 file: {path}: {exc}",
                cause=exc,
            )
        except Exception as exc:  # noqa: BLE001 - public reader boundary
            _raise_failure(
                _internal_failure(
                    source_path,
                    f"EVENT_HDF5 iteration failed internally: {exc}",
                    cause=exc,
                ),
                cause=exc,
            )


def read_event_hdf5(path: Path | str) -> dict[str, Any]:
    """Read a canonical event HDF5 file into numpy arrays.

    Equivalent to ``read_validated_event_hdf5`` with its strict defaults.
    """
    from srb.core.sensor.event_camera.storage_validation import (
        read_validated_event_hdf5,
    )

    return read_validated_event_hdf5(path)


__all__ = [
    "EVENT_HDF5_COLUMNS",
    "EVENT_HDF5_VERSION",
    "EventHDF5BackpressureError",
    "EventEpisodeMetadata",
    "EventEpisodeProvenance",
    "EventHDF5WriterTelemetry",
    "EventHDF5Writer",
    "event_digest",
    "iter_event_hdf5",
    "read_event_hdf5",
]


def _iter_event_hdf5_handle(
    h5: Any,
    np: Any,
    *,
    path: Path,
    chunk_size: int,
) -> Iterator[dict[str, Any]]:
    datasets, event_count, expected_digest = _validate_iterator_schema(
        h5, np, path=path
    )
    accumulator = _EventDigestAccumulator()
    for start in range(0, event_count, chunk_size):
        stop = min(start + chunk_size, event_count)
        chunk = {
            name: np.asarray(datasets[name][start:stop])
            for name, _dtype in EVENT_HDF5_COLUMNS
        }
        accumulator.update_columns(chunk, np)
        yield chunk
    if accumulator.hexdigest(np) != expected_digest:
        _raise_iterator_integrity(
            path,
            "event_sha256 does not match event payload",
        )
