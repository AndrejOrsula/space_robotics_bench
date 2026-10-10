"""Copy HDF5 merges in a bounded native process.

The caller owns source snapshots and the initialized publication stage. If
cleanup is incomplete, it must retain them with the cleanup owner. Output is
publishable only after this call returns.
"""

from __future__ import annotations

import math
import os
import sys
import time
from base64 import b64decode, b64encode
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

from srb.utils.failures import attempt_cleanup
from srb.utils.json_types import JsonDecodeBudget, bounded_json_dumps
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
from srb.utils.publication import (
    ArtifactPublication,
    OwnedTemporaryDirectory,
    PublicationError,
    sha256_file,
)
from srb.utils.resource_budget import (
    GIB,
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_DIRECTORY_ENTRIES,
    MAX_EXTERNAL_FILE_BYTES,
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_METADATA_AGGREGATE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    MAX_METADATA_ATTRIBUTES,
    ByteBudget,
    ResourceLimitError,
    require_bounded_int,
    require_dataset_file_size,
)

_MAX_REQUEST_BYTES = MAX_METADATA_ATTRIBUTE_BYTES
_MAX_RESPONSE_BYTES = 8192


class Hdf5NativeError(ValueError):
    """Native merge admission, execution or verification failed."""


class Hdf5NativeTimeoutError(Hdf5NativeError):
    """The native worker exceeded its finite deadline."""


class Hdf5NativeCleanupError(RuntimeError):
    """Retain the worker and its resources until cleanup can be confirmed."""

    def __init__(
        self,
        message: str,
        *,
        owner: ProcessCleanupOwner | None,
        scratch: Path,
        scratch_owner: OwnedTemporaryDirectory | None = None,
    ) -> None:
        super().__init__(message)
        self.owner = owner
        self.scratch = scratch
        self.scratch_owner = scratch_owner
        self._scratch_binding_error: OSError | PublicationError | None = None
        if self.scratch_owner is None:
            try:
                self.scratch_owner = OwnedTemporaryDirectory.bind(scratch)
            except (OSError, PublicationError) as error:
                # Do not adopt a replacement path on retry.
                self._scratch_binding_error = error
        self._callbacks: list[Callable[[], object]] = []
        self.spawn_ownership_error: ProcessOwnershipError | None = None

    def retain_cleanup(self, *callbacks: Callable[[], object]) -> None:
        """Keep bound resource owners alive; release them only after reaping."""
        self._callbacks.extend(callbacks)

    def retry_cleanup(self) -> ProcessResult | None:
        result = None
        completed = False
        try:
            if self.spawn_ownership_error is not None:
                raise RuntimeError("native HDF5 spawn identity remains unproven")
            if self.owner is not None:
                result = self.owner.cleanup()
                if not result.cleanup_complete:
                    raise RuntimeError("native HDF5 process cleanup incomplete")
            while self._callbacks:
                self._callbacks[0]()
                self._callbacks.pop(0)
            if self.scratch_owner is not None:
                self.scratch_owner.cleanup()
            elif self.scratch.exists() or self.scratch.is_symlink():
                raise RuntimeError(
                    "native HDF5 scratch ownership is unproven"
                ) from self._scratch_binding_error
            completed = True
        finally:
            if not completed:
                # Retain all owners and callbacks, including the failed one.
                raise self from sys.exc_info()[1]
        return result


@dataclass(frozen=True, slots=True)
class Hdf5NativeLimits:
    """Lowering-only native merge allowances, including import/startup time."""

    address_space_bytes: int = 2 * GIB
    max_output_bytes: int = MAX_EXTERNAL_FILE_BYTES
    max_source_bytes: int = MAX_EXTERNAL_FILE_BYTES
    max_array_bytes: int = MAX_ARRAY_BYTES
    max_array_elements: int = MAX_ARRAY_ELEMENTS
    max_decoded_bytes: int = MAX_DECODED_AGGREGATE_BYTES
    max_entries: int = MAX_DIRECTORY_ENTRIES
    max_metadata_bytes: int = MAX_METADATA_AGGREGATE_BYTES
    max_attribute_bytes: int = MAX_METADATA_ATTRIBUTE_BYTES
    max_attributes: int = MAX_METADATA_ATTRIBUTES
    timeout_s: float = 120.0

    def __post_init__(self) -> None:
        ceilings = {
            "address_space_bytes": 2 * GIB,
            "max_output_bytes": MAX_EXTERNAL_FILE_BYTES,
            "max_source_bytes": MAX_EXTERNAL_FILE_BYTES,
            "max_array_bytes": MAX_ARRAY_BYTES,
            "max_array_elements": MAX_ARRAY_ELEMENTS,
            "max_decoded_bytes": MAX_DECODED_AGGREGATE_BYTES,
            "max_entries": MAX_DIRECTORY_ENTRIES,
            "max_metadata_bytes": MAX_METADATA_AGGREGATE_BYTES,
            "max_attribute_bytes": MAX_METADATA_ATTRIBUTE_BYTES,
            "max_attributes": MAX_METADATA_ATTRIBUTES,
        }
        for name, ceiling in ceilings.items():
            object.__setattr__(
                self,
                name,
                require_bounded_int(
                    getattr(self, name),
                    resource=f"native HDF5 {name}",
                    minimum=1,
                    maximum=ceiling,
                ),
            )
        if isinstance(self.timeout_s, bool):
            raise TypeError("native HDF5 timeout must be finite")
        try:
            timeout = float(self.timeout_s)
        except (OverflowError, TypeError, ValueError) as error:
            raise TypeError("native HDF5 timeout must be finite") from error
        if not math.isfinite(timeout) or not 0 < timeout <= 120:
            raise ValueError("native HDF5 timeout must be positive and at most 120s")
        object.__setattr__(self, "timeout_s", timeout)


@dataclass(frozen=True, slots=True)
class Hdf5NativeResult:
    demo_count: int
    total_samples: int
    output_sha256: str
    output_bytes: int
    parameter_payload: bytes | None = None
    output_path: Path | None = None


@dataclass(frozen=True, slots=True)
class Hdf5InspectionResult:
    """Bounded physical and semantic inspection returned by the native child."""

    complete: bool
    errors: tuple[str, ...]
    demo_count: int | None
    total_samples: int | None
    action_dim: int | None
    train_count: int | None
    valid_count: int | None
    obs_keys: frozenset[str]
    metadata: Mapping[str, str | bytes] | None
    #: Canonical digest of the ``data`` group ``demo_io_contract`` attribute
    #: (deep inspection only; ``None`` when absent).
    demo_io_contract_sha256: str | None = None


def _member(path: Path, digest: str) -> list[str]:
    if not isinstance(path, Path):
        raise TypeError("native HDF5 members require pathlib.Path")
    name = str(path.absolute())
    if len(name) > 4096 or "\x00" in name:
        raise Hdf5NativeError("native HDF5 member path exceeds its bound")
    if (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(c not in "0123456789abcdef" for c in digest)
    ):
        raise Hdf5NativeError("native HDF5 member requires lowercase SHA-256")
    return [name, digest]


def _optional_member(path: Path, digest: str | None) -> list[str | None]:
    if not isinstance(path, Path):
        raise TypeError("native HDF5 members require pathlib.Path")
    name = str(path.absolute())
    if len(name) > 4096 or "\x00" in name:
        raise Hdf5NativeError("native HDF5 member path exceeds its bound")
    if digest is not None:
        _member(path, digest)
    return [name, digest]


def _owned_directory_binding(
    owner: OwnedTemporaryDirectory,
) -> tuple[Path, tuple[int, int]]:
    """Return the owner's captured directory identity.

    Use the identity bound at creation instead of ``Path.stat``, which could
    adopt a replacement. The child verifies the binding before output.
    """

    try:
        identity = getattr(owner, "_identity")
        path = Path(os.path.abspath(os.fspath(owner.path)))
    except (AttributeError, OSError, TypeError, ValueError) as error:
        raise Hdf5NativeError(
            "native HDF5 merge output directory ownership is unavailable"
        ) from error
    if (
        not isinstance(identity, tuple)
        or len(identity) != 2
        or any(type(value) is not int or value < 0 for value in identity)
        or len(str(path)) > 4096
        or "\x00" in str(path)
    ):
        raise Hdf5NativeError("native HDF5 merge output directory identity is invalid")
    return path, (identity[0], identity[1])


def _worker_command(
    request: Path,
    response: Path,
    limits: Hdf5NativeLimits,
    *,
    worker_script: Path | None = None,
) -> tuple[str, ...]:
    return (
        sys.executable,
        "-I",
        "-B",
        str(
            worker_script
            if worker_script is not None
            else Path(__file__).with_name("_hdf5_native_worker.py")
        ),
        str(request),
        str(response),
        str(limits.address_space_bytes),
        str(limits.max_output_bytes),
    )


def _response_limit(limits: Hdf5NativeLimits) -> int:
    return min(
        MAX_IN_MEMORY_FILE_BYTES,
        max(_MAX_RESPONSE_BYTES, limits.max_metadata_bytes + (1 << 20)),
    )


def _run_native_payload(
    payload: bytes,
    *,
    limits: Hdf5NativeLimits,
    response_limit: int,
    worker_script: Path | None = None,
) -> dict[str, object]:
    """Run one native request and own its process/scratch cleanup seam."""

    started = time.monotonic()
    scratch_owner = OwnedTemporaryDirectory.create(prefix="srb-hdf5-native-")
    scratch = scratch_owner.path
    owner: OwnedMonitoredProcess | None = None
    cleanup_retained = False
    try:
        request = scratch / "request.json"
        response = scratch / "response.json"
        with request.open("xb") as stream:
            stream.write(payload)
        remaining = started + limits.timeout_s - time.monotonic()
        if remaining <= 0:
            raise Hdf5NativeTimeoutError("native HDF5 request deadline expired")
        environment = os.environ.copy()
        for name in (
            "OMP_NUM_THREADS",
            "OPENBLAS_NUM_THREADS",
            "MKL_NUM_THREADS",
            "NUMEXPR_NUM_THREADS",
        ):
            environment[name] = "1"
        try:
            command = _worker_command(
                request, response, limits, worker_script=worker_script
            )
            owner = OwnedMonitoredProcess.spawn(
                command,
                kind=ProcessKind.FINITE,
                deadline_s=remaining,
                capture_output=False,
                env=environment,
            )
        except ProcessStartupCleanupError as error:
            raise Hdf5NativeCleanupError(
                "native HDF5 startup cleanup remains incomplete",
                owner=error.cleanup_owner,
                scratch=scratch,
                scratch_owner=scratch_owner,
            ) from error
        except (
            ProcessOwnershipError,
            ProcessPlatformError,
            ProcessSpawnError,
        ) as error:
            raise Hdf5NativeError("native HDF5 worker could not be started") from error
        result = None
        wait_completed = False
        try:
            result = owner.wait()
            wait_completed = True
        finally:
            if not wait_completed and (
                owner.result is None or not owner.result.cleanup_complete
            ):
                raise Hdf5NativeCleanupError(
                    "native HDF5 worker cleanup failed",
                    owner=owner,
                    scratch=scratch,
                    scratch_owner=scratch_owner,
                ) from sys.exc_info()[1]
        if result is None or not result.cleanup_complete:
            raise Hdf5NativeCleanupError(
                "native HDF5 worker cleanup incomplete",
                owner=owner,
                scratch=scratch,
                scratch_owner=scratch_owner,
            )
        if result.status == ProcessStatus.TIMED_OUT:
            raise Hdf5NativeTimeoutError("native HDF5 worker deadline expired")
        if result.returncode != 0:
            raise Hdf5NativeError(
                "native HDF5 worker failed within its resource limits"
            )
        try:
            raw = ArtifactPublication.file(response).read(
                lambda value: value,
                max_bytes=response_limit,
            )
        except (PublicationError, OSError) as error:
            raise Hdf5NativeError("native HDF5 response cannot be admitted") from error
        if raw is None:
            raise Hdf5NativeError("native HDF5 worker produced no response")
        value = JsonDecodeBudget(
            max_nodes=1_000_000,
            max_depth=64,
            max_total_nodes=16_000_000,
        ).loads(
            raw,
            context="native HDF5 response",
            max_bytes=response_limit,
        )
        if not isinstance(value, dict) or type(value.get("version")) is not int:
            raise Hdf5NativeError("native HDF5 worker response is invalid")
        return value
    except Hdf5NativeCleanupError:
        cleanup_retained = True
        raise
    finally:
        if not cleanup_retained:
            _, cleanup_error = attempt_cleanup(scratch_owner.cleanup)
            if cleanup_error is not None:
                raise Hdf5NativeCleanupError(
                    "native HDF5 scratch cleanup failed",
                    owner=owner,
                    scratch=scratch,
                    scratch_owner=scratch_owner,
                ) from cleanup_error


def _raise_worker_error(value: Mapping[str, object]) -> None:
    if set(value) == {"version", "error"} and isinstance(value.get("error"), str):
        raise Hdf5NativeError(str(value["error"])[:4096])
    if value.get("kind") == "resource_limit":
        resource = value.get("resource")
        limit = value.get("limit")
        observed = value.get("observed")
        if (
            isinstance(resource, str)
            and len(resource) <= 256
            and type(limit) is int
            and type(observed) is int
            and 0 < limit <= MAX_DECODED_AGGREGATE_BYTES
            and observed > limit
        ):
            raise ResourceLimitError(resource, limit=limit, observed=observed)
    message = value.get("message")
    if isinstance(message, str) and 0 < len(message) <= 4096:
        raise Hdf5NativeError(message)
    raise Hdf5NativeError(
        "native HDF5 worker rejected input or exhausted a resource limit"
    )


def _parse_result(
    value: Mapping[str, object],
    *,
    version: int,
    limits: Hdf5NativeLimits,
    output_path: Path,
    normalization: dict[str, int] | None,
) -> Hdf5NativeResult:
    if type(value.get("version")) is not int or value["version"] != version:
        _raise_worker_error(value)
    if set(value) == {"version", "error"}:
        _raise_worker_error(value)
    expected_fields = {
        "version",
        "demo_count",
        "total_samples",
        "output_sha256",
        "output_bytes",
        "parameters",
    }
    if set(value) != expected_fields:
        raise Hdf5NativeError("native HDF5 worker response fields are invalid")
    demos = require_bounded_int(
        value["demo_count"],
        resource="native HDF5 demo count",
        maximum=limits.max_entries,
    )
    samples = require_bounded_int(
        value["total_samples"],
        resource="native HDF5 samples",
        maximum=limits.max_entries * limits.max_array_elements,
    )
    size = require_bounded_int(
        value["output_bytes"],
        resource="native HDF5 output bytes",
        maximum=limits.max_output_bytes,
    )
    digest = value["output_sha256"]
    if not isinstance(digest, str):
        raise Hdf5NativeError("native HDF5 output digest is invalid")
    _member(output_path, digest)
    output_budget = ByteBudget("native HDF5 output bytes", limits.max_output_bytes)
    if (
        sha256_file(output_path, byte_budget=output_budget) != digest
        or output_budget.consumed != size
    ):
        raise Hdf5NativeError("native HDF5 output identity changed")
    parameters = value["parameters"]
    if parameters is not None and (
        normalization is None
        or not isinstance(parameters, str)
        or len(parameters) > 4096
    ):
        raise Hdf5NativeError("native HDF5 parameter response is invalid")
    parameter_payload = None if parameters is None else parameters.encode("ascii")
    return Hdf5NativeResult(demos, samples, digest, size, parameter_payload)


def _require_train_split(train_split: object) -> None:
    """Reject a split the native worker would turn into a wrong mask."""

    if isinstance(train_split, bool) or not isinstance(train_split, (int, float)):
        raise TypeError("native HDF5 train_split must be a finite number")
    if not math.isfinite(train_split) or not 0 <= train_split <= 1:
        raise ValueError(
            "native HDF5 train_split must be a finite number within [0, 1]"
        )


def _run_native_transform(
    sources: Sequence[tuple[Path, str]],
    output_path: Path,
    *,
    train_split: float,
    limits: Hdf5NativeLimits | None = None,
    normalization: dict[str, int] | None = None,
    worker_script: Path | None = None,
) -> Hdf5NativeResult:
    """Copy admitted demo graphs through the shared native process owner."""

    limits = Hdf5NativeLimits() if limits is None else limits
    if not isinstance(limits, Hdf5NativeLimits):
        raise TypeError("limits must be Hdf5NativeLimits")
    _require_train_split(train_split)
    if not sys.platform.startswith("linux") or not hasattr(os, "O_NOFOLLOW"):
        raise Hdf5NativeError("native HDF5 resource limits require Linux")
    require_bounded_int(
        len(sources), resource="native HDF5 sources", maximum=limits.max_entries
    )
    request_budget = ByteBudget("native HDF5 request bytes", _MAX_REQUEST_BYTES)
    members = []
    for path, digest in sources:
        member = _member(path, digest)
        request_budget.consume(6 * len(member[0]) + 128)
        members.append(member)
    output = _member(
        output_path,
        sha256_file(
            output_path,
            byte_budget=ByteBudget("native HDF5 output bytes", limits.max_output_bytes),
        ),
    )
    payload = bounded_json_dumps(
        {
            "version": 1,
            "sources": members,
            "output": output,
            "train_split": train_split,
            "limits": asdict(limits),
            "normalization": normalization,
        },
        context="native HDF5 request",
        max_bytes=_MAX_REQUEST_BYTES,
        max_nodes=1_000_000,
        max_depth=8,
        allow_nan=False,
    ).encode("ascii")
    value = _run_native_payload(
        payload,
        limits=limits,
        response_limit=_MAX_RESPONSE_BYTES,
        worker_script=worker_script,
    )
    return _parse_result(
        value,
        version=1,
        limits=limits,
        output_path=output_path,
        normalization=normalization,
    )


def _encode_metadata_wire(
    metadata: Mapping[str, str | bytes] | None,
    *,
    limits: Hdf5NativeLimits,
) -> list[list[str]] | None:
    if metadata is None:
        return None
    if not isinstance(metadata, Mapping):
        raise TypeError("native HDF5 metadata must be a mapping or None")
    require_bounded_int(
        len(metadata),
        resource="native HDF5 metadata attributes",
        maximum=limits.max_attributes,
    )
    budget = ByteBudget("native HDF5 request bytes", _MAX_REQUEST_BYTES)
    wire: list[list[str]] = []
    for key in sorted(metadata):
        if not isinstance(key, str) or not key or len(key) > 128:
            raise Hdf5NativeError("native HDF5 metadata name is invalid")
        value = metadata[key]
        if not isinstance(value, (str, bytes)):
            raise TypeError("native HDF5 metadata values must be text or bytes")
        value_bytes = (
            value.encode("utf-8", "surrogateescape")
            if isinstance(value, str)
            else bytes(value)
        )
        if len(value_bytes) > limits.max_attribute_bytes:
            raise ResourceLimitError(
                f"native HDF5 metadata attribute {key!r}",
                limit=limits.max_attribute_bytes,
                observed=len(value_bytes),
            )
        if isinstance(value, str):
            wire_value = value
            kind = "text"
        else:
            wire_value = b64encode(value_bytes).decode("ascii")
            kind = "bytes"
        budget.consume(len(key) + len(wire_value) + 64)
        wire.append([key, kind, wire_value])
    return wire


def _decode_metadata_wire(
    value: object,
    *,
    limits: Hdf5NativeLimits,
) -> dict[str, str | bytes] | None:
    if value is None:
        return None
    if not isinstance(value, list):
        raise Hdf5NativeError("native HDF5 metadata response is invalid")
    require_bounded_int(
        len(value),
        resource="native HDF5 metadata attributes",
        maximum=limits.max_attributes,
    )
    result: dict[str, str | bytes] = {}
    budget = ByteBudget("native HDF5 metadata response", limits.max_metadata_bytes)
    for item in value:
        if not isinstance(item, list) or len(item) != 3:
            raise Hdf5NativeError("native HDF5 metadata response is invalid")
        key, kind, encoded = item
        if (
            not isinstance(key, str)
            or not key
            or len(key) > 128
            or key in result
            or not isinstance(kind, str)
            or not isinstance(encoded, str)
        ):
            raise Hdf5NativeError("native HDF5 metadata response is invalid")
        if kind == "text":
            decoded: str | bytes = encoded
            value_bytes = encoded.encode("utf-8", "surrogateescape")
        elif kind == "bytes":
            try:
                raw = b64decode(encoded.encode("ascii"), validate=True)
            except (ValueError, UnicodeError) as error:
                raise Hdf5NativeError(
                    "native HDF5 metadata response is invalid"
                ) from error
            decoded = raw
            value_bytes = raw
        else:
            raise Hdf5NativeError("native HDF5 metadata response is invalid")
        if len(value_bytes) > limits.max_attribute_bytes:
            raise ResourceLimitError(
                f"native HDF5 metadata attribute {key!r}",
                limit=limits.max_attribute_bytes,
                observed=len(value_bytes),
            )
        budget.consume(len(value_bytes))
        result[key] = decoded
    return result


def inspect_robomimic_hdf5(
    path: Path,
    *,
    mode: Literal["count", "deep"],
    expected_sha256: str | None = None,
    limits: Hdf5NativeLimits | None = None,
    require_demo_io_contract: bool = False,
) -> Hdf5InspectionResult:
    """Inspect one Robomimic file in the bounded native HDF5 child."""

    limits = Hdf5NativeLimits() if limits is None else limits
    if not isinstance(limits, Hdf5NativeLimits):
        raise TypeError("limits must be Hdf5NativeLimits")
    if mode not in {"count", "deep"}:
        raise ValueError("native HDF5 inspection mode is invalid")
    if expected_sha256 is not None:
        _member(path, expected_sha256)
    if not sys.platform.startswith("linux") or not hasattr(os, "O_NOFOLLOW"):
        raise Hdf5NativeError("native HDF5 resource limits require Linux")
    require_dataset_file_size(
        path,
        resource="native HDF5 inspection source",
        max_bytes=limits.max_source_bytes,
    )
    payload = bounded_json_dumps(
        {
            "version": 2,
            "operation": "inspect",
            "source": _optional_member(path, expected_sha256),
            "mode": mode,
            "require_demo_io_contract": require_demo_io_contract,
            "limits": asdict(limits),
        },
        context="native HDF5 inspection request",
        max_bytes=_MAX_REQUEST_BYTES,
        max_nodes=1_000_000,
        max_depth=8,
        allow_nan=False,
    ).encode("ascii")
    value = _run_native_payload(
        payload,
        limits=limits,
        response_limit=_response_limit(limits),
    )
    if type(value.get("version")) is not int or value["version"] != 2:
        _raise_worker_error(value)
    if set(value) == {"version", "error"}:
        _raise_worker_error(value)
    if value.get("kind") in {"worker", "resource_limit"}:
        _raise_worker_error(value)
    expected_fields = {
        "version",
        "operation",
        "complete",
        "errors",
        "demo_count",
        "total_samples",
        "action_dim",
        "train_count",
        "valid_count",
        "obs_keys",
        "metadata",
        "demo_io_contract_sha256",
    }
    if set(value) != expected_fields or value.get("operation") != "inspect":
        raise Hdf5NativeError("native HDF5 inspection response fields are invalid")
    complete = value["complete"]
    if type(complete) is not bool:
        raise Hdf5NativeError("native HDF5 inspection completion flag is invalid")
    raw_errors = value["errors"]
    if not isinstance(raw_errors, list):
        raise Hdf5NativeError("native HDF5 inspection errors are invalid")
    require_bounded_int(
        len(raw_errors),
        resource="native HDF5 inspection errors",
        maximum=limits.max_entries,
    )
    errors: list[str] = []
    for error in raw_errors:
        if not isinstance(error, str) or len(error) > 4096:
            raise Hdf5NativeError("native HDF5 inspection error is invalid")
        errors.append(error)

    def nullable_int(name: str, maximum: int) -> int | None:
        raw = value[name]
        if raw is None:
            return None
        return require_bounded_int(raw, resource=f"native HDF5 {name}", maximum=maximum)

    demo_count = nullable_int("demo_count", limits.max_entries)
    total_samples = nullable_int(
        "total_samples", limits.max_entries * limits.max_array_elements
    )
    action_dim = nullable_int("action_dim", limits.max_array_elements)
    train_count = nullable_int("train_count", limits.max_entries)
    valid_count = nullable_int("valid_count", limits.max_entries)
    raw_obs_keys = value["obs_keys"]
    if not isinstance(raw_obs_keys, list):
        raise Hdf5NativeError("native HDF5 inspection observation keys are invalid")
    require_bounded_int(
        len(raw_obs_keys),
        resource="native HDF5 inspection observation keys",
        maximum=limits.max_entries,
    )
    obs_keys: set[str] = set()
    for key in raw_obs_keys:
        if not isinstance(key, str) or not key or len(key) > 4096:
            raise Hdf5NativeError("native HDF5 inspection observation key is invalid")
        obs_keys.add(key)
    if len(obs_keys) != len(raw_obs_keys):
        raise Hdf5NativeError("native HDF5 inspection observation keys are duplicated")
    metadata = _decode_metadata_wire(value["metadata"], limits=limits)
    contract_sha256 = value["demo_io_contract_sha256"]
    if contract_sha256 is not None and (
        not isinstance(contract_sha256, str)
        or len(contract_sha256) != 64
        or any(char not in "0123456789abcdef" for char in contract_sha256)
    ):
        raise Hdf5NativeError("native HDF5 inspection contract digest is invalid")
    if not complete and (
        contract_sha256 is not None
        or demo_count is not None
        or total_samples is not None
        or action_dim is not None
        or train_count is not None
        or valid_count is not None
        or obs_keys
        or metadata is not None
    ):
        raise Hdf5NativeError(
            "native HDF5 incomplete inspection contains partial statistics"
        )
    return Hdf5InspectionResult(
        complete=complete,
        errors=tuple(errors),
        demo_count=demo_count,
        total_samples=total_samples,
        action_dim=action_dim,
        train_count=train_count,
        valid_count=valid_count,
        obs_keys=frozenset(obs_keys),
        metadata=metadata,
        demo_io_contract_sha256=contract_sha256,
    )


def merge_private_hdf5(
    sources: Sequence[tuple[Path, str]],
    output_dir: OwnedTemporaryDirectory,
    *,
    train_split: float,
    metadata: Mapping[str, str | bytes] | None = None,
    limits: Hdf5NativeLimits | None = None,
) -> Hdf5NativeResult:
    """Validate and create a complete merge in a caller-owned private directory."""

    limits = Hdf5NativeLimits() if limits is None else limits
    if not isinstance(limits, Hdf5NativeLimits):
        raise TypeError("limits must be Hdf5NativeLimits")
    if not isinstance(output_dir, OwnedTemporaryDirectory):
        raise TypeError("native HDF5 merge requires an OwnedTemporaryDirectory")
    _require_train_split(train_split)
    if not sys.platform.startswith("linux") or not hasattr(os, "O_NOFOLLOW"):
        raise Hdf5NativeError("native HDF5 resource limits require Linux")
    require_bounded_int(
        len(sources), resource="native HDF5 sources", maximum=limits.max_entries
    )
    request_budget = ByteBudget("native HDF5 request bytes", _MAX_REQUEST_BYTES)
    members: list[list[str]] = []
    for path, digest in sources:
        member = _member(path, digest)
        request_budget.consume(6 * len(member[0]) + 128)
        members.append(member)
    directory, directory_identity = _owned_directory_binding(output_dir)
    metadata_wire = _encode_metadata_wire(metadata, limits=limits)
    request_budget.consume(256 + len(str(directory)))
    payload = bounded_json_dumps(
        {
            "version": 2,
            "operation": "merge_private",
            "sources": members,
            "output_dir": [str(directory), *directory_identity],
            "output_name": "merged.hdf5",
            "train_split": float(train_split),
            "metadata": metadata_wire,
            "limits": asdict(limits),
        },
        context="native HDF5 merge request",
        max_bytes=_MAX_REQUEST_BYTES,
        max_nodes=1_000_000,
        max_depth=8,
        allow_nan=False,
    ).encode("ascii")
    value = _run_native_payload(
        payload,
        limits=limits,
        response_limit=_response_limit(limits),
    )
    if type(value.get("version")) is not int or value["version"] != 2:
        _raise_worker_error(value)
    if set(value) == {"version", "error"}:
        _raise_worker_error(value)
    if value.get("kind") in {"worker", "resource_limit"}:
        _raise_worker_error(value)
    expected_fields = {
        "version",
        "operation",
        "demo_count",
        "total_samples",
        "output_path",
        "output_sha256",
        "output_bytes",
    }
    if set(value) != expected_fields or value.get("operation") != "merge_private":
        raise Hdf5NativeError("native HDF5 merge response fields are invalid")
    output_path = directory / "merged.hdf5"
    if value["output_path"] != str(output_path):
        raise Hdf5NativeError("native HDF5 merge output path is invalid")
    demos = require_bounded_int(
        value["demo_count"],
        resource="native HDF5 demo count",
        maximum=limits.max_entries,
    )
    samples = require_bounded_int(
        value["total_samples"],
        resource="native HDF5 samples",
        maximum=limits.max_entries * limits.max_array_elements,
    )
    size = require_bounded_int(
        value["output_bytes"],
        resource="native HDF5 output bytes",
        maximum=limits.max_output_bytes,
    )
    digest = value["output_sha256"]
    if not isinstance(digest, str):
        raise Hdf5NativeError("native HDF5 merge output digest is invalid")
    _member(output_path, digest)
    budget = ByteBudget("native HDF5 output bytes", limits.max_output_bytes)
    if (
        sha256_file(output_path, byte_budget=budget) != digest
        or budget.consumed != size
    ):
        raise Hdf5NativeError("native HDF5 merge output identity changed")
    return Hdf5NativeResult(demos, samples, digest, size, None, output_path)


def copy_merged_hdf5(
    sources: Sequence[tuple[Path, str]],
    output_path: Path,
    *,
    train_split: float,
    limits: Hdf5NativeLimits | None = None,
) -> Hdf5NativeResult:
    """Copy demo graphs into an initialized private stage using native limits."""
    return _run_native_transform(
        sources, output_path, train_split=train_split, limits=limits
    )


def normalize_materialized_hdf5(
    path: Path,
    *,
    max_decoded_bytes: int,
    max_batch_bytes: int,
    worker_script: Path,
    limits: Hdf5NativeLimits | None = None,
) -> bytes | None:
    """Transform one owned materialization; return parameter text after verification."""
    result = _run_native_transform(
        (),
        path,
        train_split=0.0,
        limits=limits,
        worker_script=worker_script,
        normalization={
            "max_decoded_bytes": max_decoded_bytes,
            "max_batch_bytes": max_batch_bytes,
        },
    )
    return result.parameter_payload


__all__ = [
    "Hdf5NativeCleanupError",
    "Hdf5NativeError",
    "Hdf5InspectionResult",
    "Hdf5NativeLimits",
    "Hdf5NativeResult",
    "Hdf5NativeTimeoutError",
    "copy_merged_hdf5",
    "inspect_robomimic_hdf5",
    "merge_private_hdf5",
    "normalize_materialized_hdf5",
]
