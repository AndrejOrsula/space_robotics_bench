"""Read named HDF5 text metadata in a bounded child process.

The child isolates h5py's native imports, enforces finite resource limits, and
uses private binary frames without pickling provider objects.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import stat
import struct
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from srb.utils.failures import attempt_cleanup
from srb.utils.process_supervisor import (
    OwnedMonitoredProcess,
    ProcessCleanupOwner,
    ProcessKind,
    ProcessOwnershipError,
    ProcessPlatformError,
    ProcessResult,
    ProcessSpawnError,
    ProcessStartupCleanupError,
    ProcessStatus,
)
from srb.utils.publication import OwnedTemporaryDirectory, PublicationError
from srb.utils.resource_budget import (
    GIB,
    MAX_EXTERNAL_FILE_BYTES,
    MAX_METADATA_AGGREGATE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    require_bounded_int,
)

_ALLOWED_NAMES: Final[frozenset[str]] = frozenset(
    {"env_name", "env_kwargs", "env_args", "action_schema", "model_file"}
)
_DEFAULT_TIMEOUT_S: Final[float] = 120.0
_DEFAULT_ADDRESS_SPACE_BYTES: Final[int] = 2 * GIB
_MAX_REQUEST_BYTES: Final[int] = 64 * 1024
_MAX_OBJECT_PATH_CHARS: Final[int] = 4096
_MAX_OBJECT_PATH_COMPONENTS: Final[int] = 32
_MAX_ERROR_BYTES: Final[int] = 4096
_RESPONSE_OVERHEAD_BYTES: Final[int] = 1 << 20
_RESPONSE_MAGIC: Final[bytes] = b"SRBH5M01"
_HASH_RESPONSE_MAGIC: Final[bytes] = b"SRBH5H01"
_HASH_RESPONSE_LIMIT: Final[int] = 4096
_RESPONSE_PREFIX = struct.Struct(">8sB")
_HASH_SUCCESS = struct.Struct(">8sB32s")
_SUCCESS_COUNT = struct.Struct(">I")
_SUCCESS_RECORD = struct.Struct(">H B Q")
_ERROR_HEADER = struct.Struct(">B H Q Q")

_STATUS_SUCCESS: Final[int] = 0
_STATUS_ERROR: Final[int] = 1
_VALUE_TEXT: Final[int] = 1
_VALUE_BYTES: Final[int] = 2

_ERROR_VALUE_LIMIT: Final[int] = 2
_ERROR_DIGEST: Final[int] = 3
_ERROR_PLATFORM: Final[int] = 4
_ERROR_SOURCE: Final[int] = 5
_ERROR_LINK: Final[int] = 6
_ERROR_AGGREGATE_LIMIT: Final[int] = 7
_ERROR_NATIVE_LIMIT: Final[int] = 8
_ERROR_RESPONSE_LIMIT: Final[int] = 9
_ERROR_SOURCE_LIMIT: Final[int] = 10


class Hdf5MetadataError(ValueError):
    """The source, worker, response, or metadata violates the decoder contract."""


class Hdf5MetadataLimitError(Hdf5MetadataError):
    """Decoded metadata exceeds one of the caller's finite limits."""

    def __init__(self, resource: str, *, limit: int, observed: int) -> None:
        self.resource = resource
        self.limit = limit
        self.observed = observed
        Hdf5MetadataError.__init__(
            self,
            f"{resource} exceeds metadata limit: observed {observed}, limit {limit}",
        )


class Hdf5MetadataDigestError(Hdf5MetadataError):
    """The source changed or does not match its expected SHA-256 digest."""


class Hdf5MetadataPlatformError(Hdf5MetadataError):
    """The native child-limit contract is unavailable on this platform."""


class Hdf5MetadataTimeoutError(Hdf5MetadataError):
    """The bounded metadata worker exceeded its deadline."""


class Hdf5MetadataWorkerError(Hdf5MetadataError):
    """The owned worker exited without a valid bounded response."""


class Hdf5MetadataCleanupError(RuntimeError):
    """Worker or scratch cleanup remains pending and can be retried."""

    def __init__(
        self,
        message: str,
        *,
        owner: ProcessCleanupOwner | None,
        scratch_path: Path,
        result: ProcessResult | None = None,
        scratch_owner: OwnedTemporaryDirectory | None = None,
    ) -> None:
        self.owner = owner
        self.cleanup_owner = owner
        self.scratch_path = scratch_path
        self.result = result
        self.scratch_owner = scratch_owner
        self._scratch_binding_error: OSError | PublicationError | None = None
        if self.scratch_owner is None:
            try:
                self.scratch_owner = OwnedTemporaryDirectory.bind(scratch_path)
            except (OSError, PublicationError) as error:
                # Do not adopt a replacement path on retry.
                self._scratch_binding_error = error
        self._callbacks: list[Callable[[], object]] = []
        super().__init__(message)

    def retain_cleanup(self, *callbacks: Callable[[], object]) -> None:
        """Retain dependent owners until worker and scratch cleanup finish."""

        self._callbacks.extend(callbacks)

    def retry_cleanup(self) -> ProcessResult | None:
        """Retry process cleanup, then remove private scratch ownership."""

        result = self.result
        if self.owner is not None:
            cleanup_result, cleanup_error = attempt_cleanup(self.owner.cleanup)
            if cleanup_error is not None:
                self.result = self.owner.result
                raise self from cleanup_error
            if cleanup_result is None:
                self.result = self.owner.result
                raise self from RuntimeError(
                    "HDF5 metadata worker cleanup returned no result"
                )
            result = cleanup_result
            self.result = result
            if not result.cleanup_complete:
                raise self from RuntimeError(
                    "HDF5 metadata worker cleanup remains incomplete"
                )
        if self.scratch_owner is not None:
            _, cleanup_error = attempt_cleanup(self.scratch_owner.cleanup)
            if cleanup_error is not None:
                raise self from cleanup_error
        elif self.scratch_path.exists() or self.scratch_path.is_symlink():
            raise self from self._scratch_binding_error
        while self._callbacks:
            _, cleanup_error = attempt_cleanup(self._callbacks[0])
            if cleanup_error is not None:
                raise self from cleanup_error
            self._callbacks.pop(0)
        return result


@dataclass(frozen=True, slots=True)
class Hdf5MetadataLimits:
    """Finite decoder limits; every value may only lower the repository default."""

    max_value_bytes: int = MAX_METADATA_ATTRIBUTE_BYTES
    max_aggregate_bytes: int = MAX_METADATA_AGGREGATE_BYTES
    max_source_bytes: int = MAX_EXTERNAL_FILE_BYTES
    address_space_bytes: int = _DEFAULT_ADDRESS_SPACE_BYTES
    timeout_s: float = _DEFAULT_TIMEOUT_S

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "max_value_bytes",
            require_bounded_int(
                self.max_value_bytes,
                resource="HDF5 metadata value bytes",
                minimum=1,
                maximum=MAX_METADATA_ATTRIBUTE_BYTES,
            ),
        )
        object.__setattr__(
            self,
            "max_aggregate_bytes",
            require_bounded_int(
                self.max_aggregate_bytes,
                resource="HDF5 metadata aggregate bytes",
                minimum=1,
                maximum=MAX_METADATA_AGGREGATE_BYTES,
            ),
        )
        object.__setattr__(
            self,
            "max_source_bytes",
            require_bounded_int(
                self.max_source_bytes,
                resource="HDF5 metadata source bytes",
                minimum=1,
                maximum=MAX_EXTERNAL_FILE_BYTES,
            ),
        )
        object.__setattr__(
            self,
            "address_space_bytes",
            require_bounded_int(
                self.address_space_bytes,
                resource="HDF5 metadata address space bytes",
                minimum=1,
                maximum=_DEFAULT_ADDRESS_SPACE_BYTES,
            ),
        )
        if isinstance(self.timeout_s, bool):
            raise TypeError("HDF5 metadata timeout must be finite")
        try:
            timeout_s = float(self.timeout_s)
        except (OverflowError, TypeError, ValueError) as error:
            raise TypeError("HDF5 metadata timeout must be finite") from error
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("HDF5 metadata timeout must be positive and finite")
        if timeout_s > _DEFAULT_TIMEOUT_S:
            raise ValueError(
                f"HDF5 metadata timeout cannot exceed {_DEFAULT_TIMEOUT_S:g} seconds"
            )
        object.__setattr__(self, "timeout_s", timeout_s)


def _validate_names(names: tuple[str, ...]) -> tuple[str, ...]:
    if not isinstance(names, tuple):
        raise TypeError("names must be a tuple of strings")
    if len(names) > len(_ALLOWED_NAMES):
        raise ValueError("names contains too many metadata names")
    checked: list[str] = []
    for name in names:
        if not isinstance(name, str):
            raise TypeError("metadata names must be strings")
        if name not in _ALLOWED_NAMES:
            raise ValueError(f"unknown metadata name {name!r}")
        if name in checked:
            raise ValueError(f"duplicate metadata name {name!r}")
        checked.append(name)
    return tuple(checked)


def _validate_object_path(object_path: str) -> str:
    if not isinstance(object_path, str):
        raise TypeError("object_path must be a string")
    if len(object_path) > _MAX_OBJECT_PATH_CHARS:
        raise ValueError("object_path exceeds its bounded request length")
    if not object_path or "\x00" in object_path:
        raise ValueError("object_path must be a non-empty HDF5 path")
    if object_path == "/":
        raise ValueError("object_path must name an HDF5 object")
    components = object_path.split("/")
    if components[0] == "":
        components = components[1:]
    if len(components) > _MAX_OBJECT_PATH_COMPONENTS:
        raise ValueError("object_path exceeds its bounded component depth")
    if not components or any(component in {"", ".", ".."} for component in components):
        raise ValueError("object_path must contain ordinary HDF5 components")
    return object_path


def _validate_digest(expected_sha256: str | None) -> str | None:
    if expected_sha256 is None:
        return None
    if not isinstance(expected_sha256, str):
        raise TypeError("expected_sha256 must be a hexadecimal string or None")
    if len(expected_sha256) != hashlib.sha256().digest_size * 2:
        raise ValueError("expected_sha256 must contain 64 hexadecimal characters")
    if any(character not in "0123456789abcdefABCDEF" for character in expected_sha256):
        raise ValueError("expected_sha256 must contain hexadecimal characters")
    return expected_sha256.lower()


def _validate_source(
    source_path: Path,
    *,
    max_source_bytes: int = MAX_EXTERNAL_FILE_BYTES,
) -> Path:
    if not isinstance(source_path, Path):
        raise TypeError("source_path must be a pathlib.Path")
    if "\x00" in str(source_path):
        raise ValueError("source_path contains NUL")
    absolute_path = source_path.absolute()
    try:
        source_stat = os.lstat(absolute_path)
    except OSError as error:
        raise Hdf5MetadataError("HDF5 metadata source is unavailable") from error
    if stat.S_ISLNK(source_stat.st_mode):
        raise Hdf5MetadataError("HDF5 metadata source symlink is not allowed")
    if not stat.S_ISREG(source_stat.st_mode):
        raise Hdf5MetadataError("HDF5 metadata source must be a regular file")
    if source_stat.st_size > max_source_bytes:
        raise Hdf5MetadataLimitError(
            "HDF5 metadata source bytes",
            limit=max_source_bytes,
            observed=source_stat.st_size,
        )
    current = Path(absolute_path.anchor or os.curdir)
    for component in absolute_path.parts[1:-1]:
        current /= component
        try:
            parent_stat = os.lstat(current)
        except OSError as error:
            raise Hdf5MetadataError(
                "HDF5 metadata source parent is unavailable"
            ) from error
        if stat.S_ISLNK(parent_stat.st_mode) or not stat.S_ISDIR(parent_stat.st_mode):
            raise Hdf5MetadataError("HDF5 metadata source parent is not a directory")
    return absolute_path


def _write_private(path: Path, payload: bytes) -> None:
    if len(payload) > _MAX_REQUEST_BYTES:
        raise Hdf5MetadataError("HDF5 metadata request exceeds its bound")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_CLOEXEC", 0)
    descriptor = os.open(path, flags, 0o600)
    try:
        view = memoryview(payload)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise OSError("short private metadata request write")
            view = view[written:]
    finally:
        os.close(descriptor)


def _remove_scratch(
    scratch_path: Path,
    *,
    owner: ProcessCleanupOwner | None,
    scratch_owner: OwnedTemporaryDirectory | None = None,
    result: ProcessResult | None = None,
) -> None:
    if scratch_owner is None:
        if scratch_path.exists() or scratch_path.is_symlink():
            raise PublicationError("HDF5 metadata scratch ownership is unproven")
        return
    _, cleanup_error = attempt_cleanup(scratch_owner.cleanup)
    if cleanup_error is not None:
        raise Hdf5MetadataCleanupError(
            "HDF5 metadata scratch cleanup failed",
            owner=owner,
            scratch_path=scratch_path,
            result=result if result is not None else _owner_result(owner),
            scratch_owner=scratch_owner,
        ) from cleanup_error


def _owner_result(owner: ProcessCleanupOwner | None) -> ProcessResult | None:
    if owner is None:
        return None
    return owner.result


def _read_private_response(path: Path, *, max_bytes: int) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NONBLOCK", 0)
    if not hasattr(os, "O_NOFOLLOW"):
        raise Hdf5MetadataPlatformError("HDF5 metadata no-follow reads are unavailable")
    descriptor = os.open(path, flags | os.O_NOFOLLOW)
    try:
        source_stat = os.fstat(descriptor)
        if not stat.S_ISREG(source_stat.st_mode):
            raise Hdf5MetadataWorkerError("metadata worker response is not regular")
        if source_stat.st_size > max_bytes:
            raise Hdf5MetadataLimitError(
                "HDF5 metadata response bytes",
                limit=max_bytes,
                observed=source_stat.st_size,
            )
        chunks: list[bytes] = []
        remaining = max_bytes + 1
        while remaining > 0:
            chunk = os.read(descriptor, min(1 << 20, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        if len(payload) > max_bytes:
            raise Hdf5MetadataLimitError(
                "HDF5 metadata response bytes",
                limit=max_bytes,
                observed=len(payload),
            )
        return payload
    finally:
        os.close(descriptor)


def _decode_response(
    payload: bytes,
    *,
    requested_names: tuple[str, ...],
    limits: Hdf5MetadataLimits,
) -> dict[str, str | bytes]:
    prefix_size = _RESPONSE_PREFIX.size
    if len(payload) < prefix_size:
        raise Hdf5MetadataWorkerError("metadata worker response is truncated")
    magic, status = _RESPONSE_PREFIX.unpack_from(payload)
    if magic != _RESPONSE_MAGIC:
        raise Hdf5MetadataWorkerError("metadata worker response has invalid framing")
    offset = prefix_size
    if status == _STATUS_ERROR:
        if len(payload) < offset + _ERROR_HEADER.size:
            raise Hdf5MetadataWorkerError("metadata worker error response is truncated")
        code, message_size, limit, observed = _ERROR_HEADER.unpack_from(payload, offset)
        offset += _ERROR_HEADER.size
        if message_size > _MAX_ERROR_BYTES or len(payload) != offset + message_size:
            raise Hdf5MetadataWorkerError("metadata worker error response is invalid")
        message = payload[offset : offset + message_size].decode("utf-8", "replace")
        if code == _ERROR_VALUE_LIMIT:
            if limit != limits.max_value_bytes or observed <= limit or limit <= 0:
                raise Hdf5MetadataWorkerError("metadata worker limit frame is invalid")
            raise Hdf5MetadataLimitError(
                "HDF5 metadata value bytes",
                limit=limit,
                observed=observed,
            ) from None
        if code == _ERROR_AGGREGATE_LIMIT:
            if limit != limits.max_aggregate_bytes or observed <= limit or limit <= 0:
                raise Hdf5MetadataWorkerError("metadata worker limit frame is invalid")
            raise Hdf5MetadataLimitError(
                "HDF5 metadata aggregate bytes",
                limit=limit,
                observed=observed,
            ) from None
        if code == _ERROR_RESPONSE_LIMIT:
            response_limit = limits.max_aggregate_bytes + _RESPONSE_OVERHEAD_BYTES
            if limit != response_limit or observed <= limit or limit <= 0:
                raise Hdf5MetadataWorkerError("metadata worker limit frame is invalid")
            raise Hdf5MetadataLimitError(
                "HDF5 metadata response bytes",
                limit=limit,
                observed=observed,
            ) from None
        if code == _ERROR_SOURCE_LIMIT:
            if limit != limits.max_source_bytes or observed <= limit or limit <= 0:
                raise Hdf5MetadataWorkerError("metadata worker source frame is invalid")
            raise Hdf5MetadataLimitError(
                "HDF5 metadata source bytes",
                limit=limit,
                observed=observed,
            ) from None
        if code == _ERROR_NATIVE_LIMIT:
            if limit != 0 or observed != 0:
                raise Hdf5MetadataWorkerError("metadata worker native frame is invalid")
            raise Hdf5MetadataWorkerError(
                message or "metadata worker exhausted its native memory limit"
            )
        if limit != 0 or observed != 0:
            raise Hdf5MetadataWorkerError("metadata worker error frame is invalid")
        if code == _ERROR_DIGEST:
            raise Hdf5MetadataDigestError(message or "HDF5 metadata digest mismatch")
        if code == _ERROR_PLATFORM:
            raise Hdf5MetadataPlatformError(
                message or "HDF5 metadata native limits unavailable"
            )
        raise Hdf5MetadataError(message or "HDF5 metadata worker rejected the source")
    if status != _STATUS_SUCCESS:
        raise Hdf5MetadataWorkerError("metadata worker response has invalid status")
    if len(payload) < offset + _SUCCESS_COUNT.size:
        raise Hdf5MetadataWorkerError("metadata worker success response is truncated")
    (count,) = _SUCCESS_COUNT.unpack_from(payload, offset)
    offset += _SUCCESS_COUNT.size
    if count > len(requested_names):
        raise Hdf5MetadataWorkerError("metadata worker returned too many attributes")
    requested = set(requested_names)
    values: dict[str, str | bytes] = {}
    aggregate = 0
    for _ in range(count):
        if len(payload) < offset + _SUCCESS_RECORD.size:
            raise Hdf5MetadataWorkerError(
                "metadata worker attribute frame is truncated"
            )
        name_size, value_kind, value_size = _SUCCESS_RECORD.unpack_from(payload, offset)
        offset += _SUCCESS_RECORD.size
        if name_size == 0 or name_size > 128:
            raise Hdf5MetadataWorkerError("metadata worker returned an invalid name")
        if len(payload) < offset + name_size:
            raise Hdf5MetadataWorkerError("metadata worker name frame is truncated")
        try:
            name = payload[offset : offset + name_size].decode("ascii")
        except UnicodeDecodeError as error:
            raise Hdf5MetadataWorkerError(
                "metadata worker returned a non-ASCII name"
            ) from error
        offset += name_size
        if name not in requested or name in values:
            raise Hdf5MetadataWorkerError("metadata worker returned an unexpected name")
        if value_size > limits.max_value_bytes:
            raise Hdf5MetadataLimitError(
                "HDF5 metadata value bytes",
                limit=limits.max_value_bytes,
                observed=value_size,
            )
        aggregate += value_size
        if aggregate > limits.max_aggregate_bytes:
            raise Hdf5MetadataLimitError(
                "HDF5 metadata aggregate bytes",
                limit=limits.max_aggregate_bytes,
                observed=aggregate,
            )
        if len(payload) < offset + value_size:
            raise Hdf5MetadataWorkerError("metadata worker value frame is truncated")
        value_payload = payload[offset : offset + value_size]
        offset += value_size
        if value_kind == _VALUE_TEXT:
            values[name] = value_payload.decode("utf-8", "surrogateescape")
        elif value_kind == _VALUE_BYTES:
            values[name] = bytes(value_payload)
        else:
            raise Hdf5MetadataWorkerError(
                "metadata worker returned an invalid value kind"
            )
    if offset != len(payload):
        raise Hdf5MetadataWorkerError("metadata worker response has trailing bytes")
    return values


def _decode_hash_response(
    payload: bytes,
    *,
    limits: Hdf5MetadataLimits,
) -> str:
    prefix_size = _RESPONSE_PREFIX.size
    if len(payload) < prefix_size:
        raise Hdf5MetadataWorkerError("metadata hash worker response is truncated")
    magic, status = _RESPONSE_PREFIX.unpack_from(payload)
    if magic != _HASH_RESPONSE_MAGIC:
        raise Hdf5MetadataWorkerError(
            "metadata hash worker response has invalid framing"
        )
    offset = prefix_size
    if status == _STATUS_ERROR:
        if len(payload) < offset + _ERROR_HEADER.size:
            raise Hdf5MetadataWorkerError(
                "metadata hash worker error response is truncated"
            )
        code, message_size, limit, observed = _ERROR_HEADER.unpack_from(payload, offset)
        offset += _ERROR_HEADER.size
        if message_size > _MAX_ERROR_BYTES or len(payload) != offset + message_size:
            raise Hdf5MetadataWorkerError(
                "metadata hash worker error response is invalid"
            )
        message = payload[offset : offset + message_size].decode("utf-8", "replace")
        if code == _ERROR_SOURCE_LIMIT:
            if limit != limits.max_source_bytes or observed <= limit or limit <= 0:
                raise Hdf5MetadataWorkerError(
                    "metadata hash worker source frame is invalid"
                )
            raise Hdf5MetadataLimitError(
                "HDF5 metadata source bytes",
                limit=limit,
                observed=observed,
            ) from None
        if code == _ERROR_NATIVE_LIMIT:
            if limit != 0 or observed != 0:
                raise Hdf5MetadataWorkerError(
                    "metadata hash worker native frame is invalid"
                )
            raise Hdf5MetadataWorkerError(
                message or "metadata hash worker exhausted its native memory limit"
            )
        if limit != 0 or observed != 0:
            raise Hdf5MetadataWorkerError("metadata hash worker error frame is invalid")
        if code == _ERROR_PLATFORM:
            raise Hdf5MetadataPlatformError(
                message or "metadata hash worker native limits unavailable"
            )
        raise Hdf5MetadataError(message or "metadata hash worker rejected the source")
    if status != _STATUS_SUCCESS or len(payload) != _HASH_SUCCESS.size:
        raise Hdf5MetadataWorkerError(
            "metadata hash worker success response is invalid"
        )
    response_magic, response_status, digest = _HASH_SUCCESS.unpack(payload)
    if response_magic != _HASH_RESPONSE_MAGIC or response_status != _STATUS_SUCCESS:
        raise Hdf5MetadataWorkerError("metadata hash worker success framing is invalid")
    digest_text = bytes(digest).hex()
    if len(digest_text) != hashlib.sha256().digest_size * 2:
        raise Hdf5MetadataWorkerError("metadata hash worker returned an invalid digest")
    return digest_text


def _worker_command(
    source_path: Path,
    request_path: Path,
    response_path: Path,
    limits: Hdf5MetadataLimits,
    *,
    mode: str,
) -> tuple[str, ...]:
    worker_path = Path(__file__).with_name("_hdf5_metadata_worker.py")
    if mode == "hash":
        response_limit = _HASH_RESPONSE_LIMIT
    else:
        response_limit = limits.max_aggregate_bytes + _RESPONSE_OVERHEAD_BYTES
    return (
        sys.executable,
        "-I",
        "-B",
        str(worker_path),
        "--mode",
        mode,
        "--source",
        str(source_path),
        "--request",
        str(request_path),
        "--response",
        str(response_path),
        "--max-value-bytes",
        str(limits.max_value_bytes),
        "--max-aggregate-bytes",
        str(limits.max_aggregate_bytes),
        "--max-source-bytes",
        str(limits.max_source_bytes),
        "--address-space-bytes",
        str(limits.address_space_bytes),
        "--response-limit-bytes",
        str(response_limit),
    )


def _remaining_timeout(deadline_at: float) -> float:
    remaining = deadline_at - time.monotonic()
    if remaining <= 0:
        raise Hdf5MetadataTimeoutError("HDF5 metadata operation deadline expired")
    return remaining


def _wait_worker(
    owner: OwnedMonitoredProcess,
    *,
    scratch_path: Path,
    scratch_owner: OwnedTemporaryDirectory | None = None,
) -> ProcessResult:
    result, wait_error = attempt_cleanup(owner.wait)
    if wait_error is not None:
        cleanup_result = owner.result
        if cleanup_result is None or not cleanup_result.cleanup_complete:
            raise Hdf5MetadataCleanupError(
                "HDF5 metadata worker cleanup failed",
                owner=owner,
                scratch_path=scratch_path,
                result=cleanup_result,
                scratch_owner=scratch_owner,
            ) from wait_error
        raise wait_error.with_traceback(wait_error.__traceback__)
    if result is None:
        raise Hdf5MetadataWorkerError("metadata worker returned no result")
    return result


def _run_worker(
    source_path: Path,
    request_path: Path,
    response_path: Path,
    limits: Hdf5MetadataLimits,
    *,
    deadline_at: float,
    scratch_path: Path,
    scratch_owner: OwnedTemporaryDirectory | None = None,
    mode: str,
) -> OwnedMonitoredProcess:
    environment = os.environ.copy()
    for variable in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        environment[variable] = "1"
    try:
        owned = OwnedMonitoredProcess.spawn(
            _worker_command(
                source_path,
                request_path,
                response_path,
                limits,
                mode=mode,
            ),
            kind=ProcessKind.FINITE,
            deadline_s=_remaining_timeout(deadline_at),
            capture_output=False,
            cwd=Path.cwd(),
            env=environment,
        )
    except ProcessPlatformError as error:
        raise Hdf5MetadataPlatformError(
            "HDF5 metadata native process limits are unavailable"
        ) from error
    except ProcessStartupCleanupError as error:
        raise Hdf5MetadataCleanupError(
            "HDF5 metadata worker startup cleanup remains incomplete",
            owner=error.cleanup_owner,
            scratch_path=scratch_path,
            result=error.result,
            scratch_owner=scratch_owner,
        ) from error
    except (ProcessOwnershipError, ProcessSpawnError) as error:
        raise Hdf5MetadataWorkerError("metadata worker could not be started") from error
    result = _wait_worker(
        owned,
        scratch_path=scratch_path,
        scratch_owner=scratch_owner,
    )
    if not result.cleanup_complete:
        raise Hdf5MetadataCleanupError(
            "HDF5 metadata worker cleanup remains incomplete",
            owner=owned,
            scratch_path=scratch_path,
            result=result,
            scratch_owner=scratch_owner,
        )
    if result.status == ProcessStatus.TIMED_OUT:
        raise Hdf5MetadataTimeoutError("HDF5 metadata worker deadline expired")
    if result.returncode != 0:
        raise Hdf5MetadataWorkerError("metadata worker exited without a valid response")
    return owned


def read_hdf5_text_attributes(
    source_path: Path,
    *,
    names: tuple[str, ...],
    object_path: str = "data",
    expected_sha256: str | None = None,
    limits: Hdf5MetadataLimits | None = None,
) -> dict[str, str | bytes]:
    """Read named scalar string attributes from a regular HDF5 source.

    Without a digest, one bounded worker admits the source and another hashes
    and reads it. Missing attributes are omitted; errors return no partial
    result.
    """

    operation_started = time.monotonic()
    checked_names = _validate_names(names)
    checked_object_path = _validate_object_path(object_path)
    checked_digest = _validate_digest(expected_sha256)
    checked_limits = Hdf5MetadataLimits() if limits is None else limits
    if not isinstance(checked_limits, Hdf5MetadataLimits):
        raise TypeError("limits must be Hdf5MetadataLimits or None")
    if not sys.platform.startswith("linux"):
        raise Hdf5MetadataPlatformError("HDF5 metadata native limits require Linux")
    checked_source = _validate_source(
        source_path,
        max_source_bytes=checked_limits.max_source_bytes,
    )
    deadline_at = operation_started + checked_limits.timeout_s
    scratch_owner = OwnedTemporaryDirectory.create(prefix="srb-hdf5-metadata-")
    scratch_path = scratch_owner.path
    worker_owner: ProcessCleanupOwner | None = None
    cleanup_deferred = False
    try:
        if checked_digest is None:
            hash_request = json.dumps(
                {"mode": "hash"},
                ensure_ascii=True,
                separators=(",", ":"),
            ).encode("ascii")
            hash_request_path = scratch_path / "hash-request.json"
            hash_response_path = scratch_path / "hash-response.bin"
            _write_private(hash_request_path, hash_request)
            worker_owner = _run_worker(
                checked_source,
                hash_request_path,
                hash_response_path,
                checked_limits,
                deadline_at=deadline_at,
                scratch_path=scratch_path,
                scratch_owner=scratch_owner,
                mode="hash",
            )
            _remaining_timeout(deadline_at)
            try:
                hash_response = _read_private_response(
                    hash_response_path,
                    max_bytes=_HASH_RESPONSE_LIMIT,
                )
            except FileNotFoundError as error:
                raise Hdf5MetadataWorkerError(
                    "metadata hash worker produced no response"
                ) from error
            checked_digest = _decode_hash_response(
                hash_response,
                limits=checked_limits,
            )
            _remaining_timeout(deadline_at)

        request = json.dumps(
            {
                "mode": "read",
                "names": checked_names,
                "object_path": checked_object_path,
                "expected_sha256": checked_digest,
            },
            ensure_ascii=True,
            separators=(",", ":"),
        ).encode("ascii")
        response_limit = checked_limits.max_aggregate_bytes + _RESPONSE_OVERHEAD_BYTES
        request_path = scratch_path / "request.json"
        response_path = scratch_path / "response.bin"
        _write_private(request_path, request)
        worker_owner = _run_worker(
            checked_source,
            request_path,
            response_path,
            checked_limits,
            deadline_at=deadline_at,
            scratch_path=scratch_path,
            scratch_owner=scratch_owner,
            mode="read",
        )
        _remaining_timeout(deadline_at)
        try:
            response = _read_private_response(response_path, max_bytes=response_limit)
        except FileNotFoundError as error:
            raise Hdf5MetadataWorkerError(
                "metadata worker produced no response"
            ) from error
        _remaining_timeout(deadline_at)
        result = _decode_response(
            response,
            requested_names=checked_names,
            limits=checked_limits,
        )
        _remaining_timeout(deadline_at)
        return result
    except Hdf5MetadataCleanupError:
        cleanup_deferred = True
        raise
    finally:
        if not cleanup_deferred:
            _remove_scratch(
                scratch_path,
                owner=worker_owner,
                scratch_owner=scratch_owner,
                result=_owner_result(worker_owner),
            )


__all__ = [
    "Hdf5MetadataCleanupError",
    "Hdf5MetadataDigestError",
    "Hdf5MetadataError",
    "Hdf5MetadataLimitError",
    "Hdf5MetadataLimits",
    "Hdf5MetadataPlatformError",
    "Hdf5MetadataTimeoutError",
    "Hdf5MetadataWorkerError",
    "read_hdf5_text_attributes",
]
