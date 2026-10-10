"""Isolated native HDF5 child; install resource limits before providers."""

from __future__ import annotations

import errno
import hashlib
import json
import math
import os
import stat
import sys
from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO, cast

NativeNormalizer = Callable[[BinaryIO, dict[str, int], int, int], bytes | None]

_GIB = 1 << 30
_MAX_ERROR_BYTES = 4096
_CEILINGS = {
    "address_space_bytes": 2 * _GIB,
    "max_output_bytes": 16 * _GIB,
    "max_source_bytes": 16 * _GIB,
    "max_array_bytes": 2 * _GIB,
    "max_array_elements": 268_435_456,
    "max_decoded_bytes": 16 * _GIB,
    "max_entries": 100_000,
    "max_metadata_bytes": 128 * (1 << 20),
    "max_attribute_bytes": 64 * (1 << 20),
    "max_attributes": 256,
}
_HDF5_INTEGER_ATTRIBUTE_BYTES = 8
_HDF5_COUNT_ATTRIBUTES = ("num_demos", "total_samples", "num_train", "num_valid")


@dataclass(frozen=True, slots=True)
class _PreparedMergeMetadata:
    """Environment attributes admitted before the private output exists."""

    env_name: str | bytes
    env_kwargs: str | bytes
    env_args: str


class _BoundedDiagnostics(list[str]):
    """Retain ordered diagnostics only while the report allowance permits."""

    def __init__(self, budget: Any, *, max_entries: int, resource: str) -> None:
        super().__init__()
        self._budget = budget
        self._max_entries = max_entries
        self._resource = resource

    def append(self, value: str) -> None:
        from srb.utils.resource_budget import ResourceLimitError

        if not isinstance(value, str):
            raise TypeError(f"{self._resource} must contain text")
        encoded_size = len(value.encode("utf-8", "surrogateescape"))
        if encoded_size > _MAX_ERROR_BYTES:
            raise ResourceLimitError(
                f"{self._resource} diagnostic bytes",
                limit=_MAX_ERROR_BYTES,
                observed=encoded_size,
            )
        observed_entries = len(self) + 1
        if observed_entries > self._max_entries:
            raise ResourceLimitError(
                f"{self._resource} entries",
                limit=self._max_entries,
                observed=observed_entries,
            )
        self._budget.consume(encoded_size + 128)
        super().append(value)

    def extend(self, values: Any) -> None:
        for value in values:
            self.append(value)


class _BoundedObservationKeys(set[str]):
    """Retain the complete observation-key union under report limits."""

    def __init__(self, budget: Any, *, max_entries: int, resource: str) -> None:
        super().__init__()
        self._budget = budget
        self._max_entries = max_entries
        self._resource = resource

    def add(self, value: str) -> None:
        from srb.utils.resource_budget import ResourceLimitError

        if value in self:
            return
        observed_entries = len(self) + 1
        if observed_entries > self._max_entries:
            raise ResourceLimitError(
                f"{self._resource} entries",
                limit=self._max_entries,
                observed=observed_entries,
            )
        self._budget.consume(len(value.encode("utf-8", "surrogateescape")) + 128)
        super().add(value)


def _integer(value: object, ceiling: int, *, minimum: int = 1) -> int:
    if type(value) is not int or not minimum <= value <= ceiling:
        raise ValueError("native HDF5 integer limit is invalid")
    return value


def _apply_limits(address_space: int, output_bytes: int) -> None:
    if not sys.platform.startswith("linux"):
        raise ValueError("native HDF5 limits require Linux")
    import resource

    for resource_id, limit in (
        (resource.RLIMIT_AS, _integer(address_space, 2 * _GIB)),
        (resource.RLIMIT_FSIZE, _integer(output_bytes, 16 * _GIB)),
    ):
        soft, hard = resource.getrlimit(resource_id)
        current = [limit]
        current.extend(
            value for value in (soft, hard) if value != resource.RLIM_INFINITY
        )
        target = min(current)
        resource.setrlimit(resource_id, (target, target))


def _trusted_root() -> Path:
    """Return the installed package root, never a caller-controlled path."""

    candidate = Path(__file__).resolve()
    for root in (candidate.parents[2], candidate.parents[3]):
        if (root / "srb" / "datasets" / "robomimic.py").is_file():
            return root
    raise RuntimeError("native HDF5 installed worker root is unavailable")


def _trusted_import(module: str) -> Any:
    import importlib

    root = _trusted_root()
    root_text = str(root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)
    return importlib.import_module(module)


def _require_train_split(split: object) -> float:
    """Enforce the ``[0, 1]`` split accepted by Robomimic writers."""

    if (
        isinstance(split, bool)
        or not isinstance(split, (int, float))
        or not math.isfinite(split)
        or not 0 <= split <= 1
    ):
        raise ValueError(
            "native HDF5 train_split must be a finite number within [0, 1]"
        )
    return float(split)


_SYMLINK_COMPONENT_MESSAGE = (
    "native HDF5 path contains a symlinked component; symbolic links are "
    "refused, so pass the resolved path (for example Path.resolve())"
)


def _open_component(part: str, flags: int, *, dir_fd: int) -> int:
    """Open one component without following symlinks."""

    try:
        return os.open(part, flags | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=dir_fd)
    except OSError as error:
        if error.errno not in {errno.ELOOP, errno.ENOTDIR}:
            raise
        try:
            is_link = stat.S_ISLNK(
                os.stat(part, dir_fd=dir_fd, follow_symlinks=False).st_mode
            )
        except OSError:
            raise error from None
        if is_link:
            raise ValueError(_SYMLINK_COMPONENT_MESSAGE) from error
        raise


@contextmanager
def _regular(path: str, *, writable: bool = False) -> Iterator[BinaryIO]:
    """Bind every path component through no-follow directory descriptors."""
    if not isinstance(path, str) or not path.startswith("/") or len(path) > 4096:
        raise ValueError("native HDF5 member path is invalid")
    parts = path.split("/")[1:]
    if not parts or any(part in {"", ".", ".."} or "\x00" in part for part in parts):
        raise ValueError("native HDF5 member path is invalid")
    directory = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in parts[:-1]:
            child = _open_component(
                part, os.O_RDONLY | os.O_DIRECTORY, dir_fd=directory
            )
            os.close(directory)
            directory = child
        flags = os.O_RDWR if writable else os.O_RDONLY
        descriptor = _open_component(parts[-1], flags | os.O_NONBLOCK, dir_fd=directory)
    finally:
        os.close(directory)
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise ValueError("native HDF5 member is not a regular file")
        stream = os.fdopen(descriptor, "r+b" if writable else "rb")
    except (OSError, ValueError):
        os.close(descriptor)
        raise
    with stream:
        yield stream


def _hash(stream: BinaryIO, ceiling: int) -> tuple[str, int]:
    size = os.fstat(stream.fileno()).st_size
    if not 0 <= size <= ceiling:
        raise ValueError("native HDF5 file exceeds its byte limit")
    stream.seek(0)
    digest = hashlib.sha256()
    count = 0
    while chunk := stream.read(min(1 << 20, ceiling + 1 - count)):
        count += len(chunk)
        if count > ceiling:
            raise ValueError("native HDF5 file exceeds its byte limit")
        digest.update(chunk)
    if count != size:
        raise ValueError("native HDF5 file changed during hashing")
    stream.seek(0)
    return digest.hexdigest(), size


def _member(value: object) -> tuple[str, str]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError("native HDF5 member declaration is invalid")
    path, digest = value
    if (
        not isinstance(path, str)
        or not isinstance(digest, str)
        or len(digest) != 64
        or any(c not in "0123456789abcdef" for c in digest)
    ):
        raise ValueError("native HDF5 member digest is invalid")
    return path, digest


def _optional_member(value: object) -> tuple[str, str | None]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError("native HDF5 member declaration is invalid")
    path, digest = value
    if not isinstance(path, str) or not path.startswith("/"):
        raise ValueError("native HDF5 member path is invalid")
    if digest is not None and (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(c not in "0123456789abcdef" for c in digest)
    ):
        raise ValueError("native HDF5 member digest is invalid")
    return path, digest


def _unchanged_path(path: str, stream: BinaryIO) -> None:
    original = os.fstat(stream.fileno())
    with _regular(path) as current:
        now = os.fstat(current.fileno())
        if (original.st_dev, original.st_ino) != (now.st_dev, now.st_ino):
            raise ValueError("native HDF5 member identity changed")


@contextmanager
def _directory(path: str, *, expected: tuple[int, int] | None = None) -> Iterator[int]:
    """Open and bind a directory through no-follow descriptors."""

    if not isinstance(path, str) or not path.startswith("/") or len(path) > 4096:
        raise ValueError("native HDF5 directory path is invalid")
    parts = path.split("/")[1:]
    if not parts or any(part in {"", ".", ".."} or "\x00" in part for part in parts):
        raise ValueError("native HDF5 directory path is invalid")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for part in parts:
            child = _open_component(
                part, os.O_RDONLY | os.O_DIRECTORY, dir_fd=descriptor
            )
            os.close(descriptor)
            descriptor = child
        info = os.fstat(descriptor)
        if not stat.S_ISDIR(info.st_mode):
            raise ValueError("native HDF5 output directory is not a directory")
        identity = (int(info.st_dev), int(info.st_ino))
        if expected is not None and identity != expected:
            raise ValueError("native HDF5 output directory identity changed")
        yield descriptor
    finally:
        os.close(descriptor)


def _shape_bytes(
    shape: Any,
    itemsize: int,
    limits: dict[str, int],
    *,
    resource: str,
) -> int:
    from srb.utils.resource_budget import ResourceLimitError

    if shape is None:
        return 0
    if len(shape) > 32:
        raise ValueError(f"{resource} rank exceeds its limit")
    count = 1
    for size in shape:
        # Admit every extent even if another dimension is zero.
        if type(size) is not int or size < 0:
            raise ValueError(f"{resource} shape is invalid")
        if size > limits["max_array_elements"]:
            raise ResourceLimitError(
                f"{resource} elements",
                limit=limits["max_array_elements"],
                observed=size,
            )
    for size in shape:
        count *= size
        if count > limits["max_array_elements"]:
            raise ResourceLimitError(
                f"{resource} elements",
                limit=limits["max_array_elements"],
                observed=count,
            )
    size = count * itemsize
    if size > limits["max_array_bytes"]:
        raise ResourceLimitError(
            f"{resource} bytes",
            limit=limits["max_array_bytes"],
            observed=size,
        )
    return size


def _walk(root: Any, h5py: Any, limits: dict[str, int]) -> Iterator[Any]:
    """Inspect hard links before following them, bounding unique object traversal."""
    pending = [(root, 0)]
    seen = set()
    entries = 1
    while pending:
        current, depth = pending.pop()
        if isinstance(current, h5py.Group) and depth > 32:
            raise ValueError("native HDF5 storage depth exceeds its limit")
        identity = int(h5py.h5o.get_info(current.id).addr)
        if identity in seen:
            continue
        seen.add(identity)
        if isinstance(current, h5py.Group):
            for key in current:
                entries += 1
                if entries > limits["max_entries"]:
                    raise ValueError("native HDF5 storage entries exceed their limit")
                if not isinstance(current.get(key, getlink=True), h5py.HardLink):
                    raise ValueError(
                        "native HDF5 symbolic or external link is not admitted"
                    )
                pending.append((current[key], depth + 1))
        elif isinstance(current, h5py.Dataset):
            if current.is_virtual or current.external:
                raise ValueError("native HDF5 external storage is not admitted")
        elif not isinstance(current, h5py.Datatype):
            raise ValueError("native HDF5 object is not supported")
        yield current


def _hdf5_scalar_vlen_text(attribute: Any, h5py: Any) -> bool:
    """Recognize ordinary scalar VLEN text while leaving NULL shapes alone."""

    shape = getattr(attribute, "shape", None)
    if shape is None:
        return False
    try:
        if tuple(shape):
            return False
    except (TypeError, ValueError):
        return False
    dtype = getattr(attribute, "dtype", None)
    return getattr(dtype, "kind", None) == "O" and h5py.check_dtype(vlen=dtype) in (
        str,
        bytes,
    )


def _copied_object_ids(
    roots: tuple[Any, ...], h5py: Any, limits: dict[str, int]
) -> set[int]:
    identities: set[int] = set()
    for root in roots:
        for value in _walk(root, h5py, limits):
            identities.add(int(h5py.h5o.get_info(value.id).addr))
    return identities


def _admit_file(
    handle: Any,
    h5py: Any,
    limits: dict[str, int],
    *,
    copied_roots: tuple[Any, ...] = (),
    metadata_budget: Any | None = None,
) -> None:
    from srb.utils.resource_budget import ByteBudget

    copied_ids = _copied_object_ids(copied_roots, h5py, limits)
    budget = (
        metadata_budget
        if metadata_budget is not None
        else ByteBudget("native HDF5 metadata bytes", limits["max_metadata_bytes"])
    )
    attribute_count = 0
    for value in _walk(handle, h5py, limits):
        for name in value.attrs:
            attribute_count += 1
            if attribute_count > limits["max_attributes"]:
                raise ValueError("native HDF5 attribute count exceeds its limit")
            attribute = value.attrs.get_id(name)
            try:
                copied_text = int(
                    h5py.h5o.get_info(value.id).addr
                ) in copied_ids and _hdf5_scalar_vlen_text(attribute, h5py)
                if copied_text:
                    # Scalar text is charged by decoded wire bytes in the
                    # shared native reader below, not by its descriptor.
                    continue
                size = (
                    0
                    if attribute.shape is not None and 0 in attribute.shape
                    else int(attribute.get_storage_size())
                )
                if not 0 <= size <= limits["max_attribute_bytes"]:
                    raise ValueError("native HDF5 attribute bytes exceed their limit")
                budget.consume(size)
            finally:
                attribute.close()


def _admit_copied_scalar_metadata(
    roots: tuple[Any, ...],
    h5py: Any,
    limits: dict[str, int],
    metadata_budget: Any,
) -> None:
    """Charge copied scalar VLEN text through the bounded native decoder."""

    from srb.utils.resource_budget import ResourceLimitError

    seen: set[int] = set()
    for root in roots:
        for value in _walk(root, h5py, limits):
            identity = int(h5py.h5o.get_info(value.id).addr)
            if identity in seen:
                continue
            seen.add(identity)
            for name in value.attrs:
                attribute = value.attrs.get_id(name)
                try:
                    if not _hdf5_scalar_vlen_text(attribute, h5py):
                        continue
                finally:
                    attribute.close()
                remaining = metadata_budget.limit - metadata_budget.consumed
                decoded = _native_text_reader(
                    value,
                    (str(name),),
                    max(1, remaining),
                    h5py=h5py,
                    limits=limits,
                )
                if name not in decoded:
                    raise ValueError(
                        f"native HDF5 copied metadata attribute {name!r} is missing"
                    )
                payload = decoded[name]
                size = (
                    len(payload.encode("utf-8", "surrogateescape"))
                    if isinstance(payload, str)
                    else len(payload)
                )
                if size > limits["max_attribute_bytes"]:
                    raise ResourceLimitError(
                        f"native HDF5 metadata attribute {name!r}",
                        limit=limits["max_attribute_bytes"],
                        observed=size,
                    )
                metadata_budget.consume(size)


def _admit_copy(
    demo: Any, h5py: Any, limits: dict[str, int], totals: list[int]
) -> None:
    from srb.utils.resource_budget import ResourceLimitError

    for value in _walk(demo, h5py, limits):
        totals[1] += 1
        if totals[1] > limits["max_entries"]:
            raise ResourceLimitError(
                "native HDF5 copied entries",
                limit=limits["max_entries"],
                observed=totals[1],
            )
        if not isinstance(value, h5py.Dataset):
            continue
        totals[0] += _shape_bytes(
            value.shape,
            value.dtype.itemsize,
            limits,
            resource=f"{value.name} copied array",
        )
        if totals[0] > limits["max_decoded_bytes"]:
            raise ResourceLimitError(
                "native HDF5 copied decoded bytes",
                limit=limits["max_decoded_bytes"],
                observed=totals[0],
            )
        if value.chunks is not None:
            _shape_bytes(
                value.chunks,
                value.dtype.itemsize,
                limits,
                resource=f"{value.name} copied chunk",
            )


def _merge(request: dict[str, Any], limits: dict[str, int]) -> dict[str, object]:
    import h5py
    import numpy as np

    if (
        set(request)
        != {"version", "sources", "output", "train_split", "limits", "normalization"}
        or request["version"] != 1
    ):
        raise ValueError("native HDF5 request fields are invalid")
    sources = request["sources"]
    if not isinstance(sources, list) or len(sources) > limits["max_entries"]:
        raise ValueError("native HDF5 source count exceeds its limit")
    split = _require_train_split(request["train_split"])
    robomimic = _trusted_import("srb.datasets.robomimic")
    output_path, expected_output = _member(request["output"])
    demos = samples = 0
    totals = [0, 0]
    wire_bytes = 0
    from srb.utils.resource_budget import ByteBudget

    metadata_budget = ByteBudget(
        "native HDF5 merge metadata bytes", limits["max_metadata_bytes"]
    )
    with _regular(output_path, writable=True) as output:
        if _hash(output, limits["max_output_bytes"])[0] != expected_output:
            raise ValueError("native HDF5 output digest mismatch")
        with h5py.File(output, "r+") as target:
            if (
                set(target) != {"data"}
                or not isinstance(target["data"], h5py.Group)
                or len(target["data"]) != 0
            ):
                raise ValueError(
                    "native HDF5 output stage must be initialized and empty"
                )
            _admit_file(target, h5py, limits)
            for member in sources:
                path, digest = _member(member)
                with _regular(path) as source:
                    actual, size = _hash(source, limits["max_source_bytes"])
                    if actual != digest:
                        raise ValueError("native HDF5 source digest mismatch")
                    wire_bytes += size
                    if wire_bytes > limits["max_source_bytes"]:
                        raise ValueError(
                            "native HDF5 source aggregate bytes exceed their limit"
                        )
                    with h5py.File(source, "r") as origin:
                        # Validate every source link and storage object before
                        # selecting or following the copied demo roots.
                        for _ in _walk(origin, h5py, limits):
                            pass
                        demo_keys = (
                            [
                                key
                                for key in sorted(origin["data"])
                                if key.startswith("demo_")
                            ]
                            if "data" in origin
                            else []
                        )
                        demo_roots = (
                            tuple(origin["data"][key] for key in demo_keys)
                            if demo_keys
                            else ()
                        )
                        _admit_file(
                            origin,
                            h5py,
                            limits,
                            copied_roots=demo_roots,
                            metadata_budget=metadata_budget,
                        )
                        _admit_copied_scalar_metadata(
                            demo_roots,
                            h5py,
                            limits,
                            metadata_budget,
                        )
                        for key in demo_keys:
                            demo = origin["data"][key]
                            _admit_copy(demo, h5py, limits, totals)
                            shape = demo["actions"].shape
                            if shape is None or not shape:
                                raise ValueError(
                                    "native HDF5 actions require a leading dimension"
                                )
                            origin.copy(demo, target["data"], name=f"demo_{demos}")
                            samples += shape[0]
                            demos += 1
                    if _hash(source, limits["max_source_bytes"])[0] != digest:
                        raise ValueError("native HDF5 source changed during copy")
                    _unchanged_path(path, source)
            keys = [f"demo_{index}" for index in range(demos)]
            n_train = robomimic._robomimic_train_count(demos, split)
            train, valid = keys[:n_train], keys[n_train:]
            target.create_group("mask")
            target["mask/train"] = np.array(train, dtype="S")
            target["mask/valid"] = np.array(valid, dtype="S")
            for name, value in (
                ("num_demos", demos),
                ("total_samples", samples),
                ("num_train", len(train)),
                ("num_valid", len(valid)),
            ):
                target["data"].attrs[name] = value
        output.flush()
        os.fsync(output.fileno())
        digest, size = _hash(output, limits["max_output_bytes"])
        _unchanged_path(output_path, output)
    return {
        "version": 1,
        "demo_count": demos,
        "total_samples": samples,
        "output_sha256": digest,
        "output_bytes": size,
        "parameters": None,
    }


def _declared_limits(value: object) -> dict[str, int]:
    if not isinstance(value, dict) or set(value) != {*_CEILINGS, "timeout_s"}:
        raise ValueError("native HDF5 limits are invalid")
    limits = {
        name: _integer(value[name], ceiling) for name, ceiling in _CEILINGS.items()
    }
    timeout = value["timeout_s"]
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise ValueError("native HDF5 timeout is invalid")
    if not math.isfinite(timeout) or not 0 < timeout <= 120:
        raise ValueError("native HDF5 timeout is invalid")
    limits["timeout_s"] = int(timeout)
    return limits


def _native_text_reader(
    data: Any,
    names: tuple[str, ...],
    remaining: int,
    *,
    h5py: Any,
    limits: dict[str, int],
) -> dict[str, str | bytes]:
    """Decode selected attributes directly through the metadata worker seam."""

    metadata_worker = _trusted_import("srb.datasets._hdf5_metadata_worker")
    worker_failure = getattr(metadata_worker, "_WorkerFailure", None)
    if not isinstance(worker_failure, type):
        raise RuntimeError("native HDF5 metadata worker failure type is unavailable")
    worker_failure_type = cast(type[BaseException], worker_failure)
    values: dict[str, str | bytes] = {}
    for name in names:
        # A zero remaining operation allowance still admits a zero-byte
        # scalar. Keep the configured native decoder limit positive, then let
        # the caller charge the decoded wire cost against the zero remainder.
        value_limit = min(limits["max_attribute_bytes"], max(1, remaining))
        try:
            value = metadata_worker._attribute_value(
                data.attrs,
                name,
                h5py=h5py,
                max_value_bytes=value_limit,
            )
        except worker_failure_type as error:
            # The metadata helper's private frame type remains inside this
            # process. Translate only its bounded resource codes into the
            # public resource error; provider details never cross the seam.
            code = getattr(error, "code", None)
            if code in {2, 7}:
                from srb.utils.resource_budget import ResourceLimitError

                raise ResourceLimitError(
                    f"native HDF5 metadata attribute {name!r}",
                    limit=value_limit,
                    observed=value_limit + 1,
                ) from None
            detail = str(error).strip()[:_MAX_ERROR_BYTES]
            raise ValueError(
                detail or "native HDF5 metadata attribute is invalid"
            ) from None
        except (
            AttributeError,
            IndexError,
            KeyError,
            OSError,
            OverflowError,
            RuntimeError,
            TypeError,
            ValueError,
        ):
            raise
        if value is None:
            continue
        value_kind, payload = value
        if value_kind == 1:
            values[name] = payload.decode("utf-8", "surrogateescape")
        elif value_kind == 2:
            values[name] = bytes(payload)
        else:
            raise ValueError("native HDF5 metadata value kind is invalid")
    return values


def _metadata_request_values(
    value: object, limits: dict[str, int]
) -> dict[str, str | bytes] | None:
    import base64

    if value is None:
        return None
    if not isinstance(value, list) or len(value) > limits["max_attributes"]:
        raise ValueError("native HDF5 metadata request is invalid")
    values: dict[str, str | bytes] = {}
    aggregate = 0
    for item in value:
        if (
            not isinstance(item, list)
            or len(item) != 3
            or not isinstance(item[0], str)
            or not item[0]
            or len(item[0]) > 128
            or item[0] in values
            or item[1] not in {"text", "bytes"}
            or not isinstance(item[2], str)
        ):
            raise ValueError("native HDF5 metadata request is invalid")
        name, kind, encoded = item
        if kind == "text":
            decoded: str | bytes = encoded
            size = len(encoded.encode("utf-8", "surrogateescape"))
        else:
            try:
                decoded = base64.b64decode(encoded.encode("ascii"), validate=True)
            except ValueError:
                raise ValueError("native HDF5 metadata request is invalid") from None
            size = len(decoded)
        if size > limits["max_attribute_bytes"]:
            raise ValueError("native HDF5 metadata request attribute is too large")
        aggregate += size
        if aggregate > limits["max_metadata_bytes"]:
            raise ValueError("native HDF5 metadata request exceeds its byte limit")
        values[name] = decoded
    return values


def _prepare_merge_metadata(
    metadata: dict[str, str | bytes] | None,
    *,
    robomimic: Any,
    limits: dict[str, int],
    max_env_args_bytes: int | None = None,
) -> _PreparedMergeMetadata | None:
    """Validate merge environment metadata before opening the output file."""

    if metadata is None:
        return None
    env_name, env_name_text, env_kwargs_raw, kwargs_payload = _merge_metadata_fields(
        metadata
    )
    if max_env_args_bytes is None:
        max_env_args_bytes = limits["max_attribute_bytes"]
    if max_env_args_bytes <= 0:
        from srb.utils.resource_budget import ResourceLimitError

        raise ResourceLimitError(
            "native HDF5 merge output metadata bytes",
            limit=limits["max_metadata_bytes"],
            observed=limits["max_metadata_bytes"] + 1,
        )
    from srb.utils.json_types import JsonDecodeBudget

    env_kwargs = JsonDecodeBudget(
        max_nodes=1_000_000,
        max_depth=64,
        max_total_nodes=16_000_000,
    ).loads(
        kwargs_payload,
        context="native HDF5 env_kwargs",
        max_bytes=limits["max_attribute_bytes"],
    )
    if not isinstance(env_kwargs, dict):
        raise ValueError("native HDF5 env_kwargs must contain a JSON object")
    env_args = robomimic.robomimic_env_args(
        env_name_text,
        env_kwargs,
        max_bytes=min(limits["max_attribute_bytes"], max_env_args_bytes),
    )
    return _PreparedMergeMetadata(env_name, env_kwargs_raw, env_args)


def _merge_metadata_fields(
    metadata: dict[str, str | bytes],
) -> tuple[str | bytes, str, str | bytes, str]:
    env_name = metadata.get("env_name", "unknown")
    env_kwargs_raw = metadata.get("env_kwargs", "{}")
    if not isinstance(env_name, (str, bytes)) or not isinstance(
        env_kwargs_raw, (str, bytes)
    ):
        raise ValueError("native HDF5 merge metadata has invalid environment values")
    env_name_text = (
        env_name.decode("utf-8", "strict") if isinstance(env_name, bytes) else env_name
    )
    kwargs_payload = (
        env_kwargs_raw.decode("utf-8", "strict")
        if isinstance(env_kwargs_raw, bytes)
        else env_kwargs_raw
    )
    return env_name, env_name_text, env_kwargs_raw, kwargs_payload


def _metadata_value_size(value: str | bytes) -> int:
    if isinstance(value, str):
        return len(value.encode("utf-8", "surrogateescape"))
    return len(value)


def _prepare_merge_output_static_bytes(
    metadata: dict[str, str | bytes] | None,
    *,
    limits: dict[str, int],
    contract: bytes | None = None,
    action_schema: str | bytes | None = None,
) -> int:
    """Reserve fixed output values before generating the bounded env_args text."""

    attribute_count = (
        len(_HDF5_COUNT_ATTRIBUTES)
        + (3 if metadata is not None else 0)
        + (contract is not None)
        + (action_schema is not None)
    )
    from srb.utils.resource_budget import ResourceLimitError

    if attribute_count > limits["max_attributes"]:
        raise ResourceLimitError(
            "native HDF5 merge output metadata attribute count",
            limit=limits["max_attributes"],
            observed=attribute_count,
        )
    static_values: list[tuple[str, int]] = [
        (name, _HDF5_INTEGER_ATTRIBUTE_BYTES) for name in _HDF5_COUNT_ATTRIBUTES
    ]
    if metadata is not None:
        env_name, _env_name_text, env_kwargs_raw, _kwargs_payload = (
            _merge_metadata_fields(metadata)
        )
        static_values.extend(
            (
                ("env_name", _metadata_value_size(env_name)),
                ("env_kwargs", _metadata_value_size(env_kwargs_raw)),
            )
        )
    if contract is not None:
        static_values.append(("demo_io_contract", len(contract)))
    if action_schema is not None:
        static_values.append(("action_schema", _metadata_value_size(action_schema)))
    static_total = 0
    for name, size in static_values:
        if size > limits["max_attribute_bytes"]:
            raise ResourceLimitError(
                f"native HDF5 merge output metadata attribute {name!r}",
                limit=limits["max_attribute_bytes"],
                observed=size,
            )
        static_total += size
    if static_total > limits["max_metadata_bytes"]:
        raise ResourceLimitError(
            "native HDF5 merge output metadata bytes",
            limit=limits["max_metadata_bytes"],
            observed=static_total,
        )
    return static_total


def _prepare_merge_output_attributes(
    metadata: _PreparedMergeMetadata | None,
    *,
    demos: int,
    samples: int,
    num_train: int,
    num_valid: int,
    limits: dict[str, int],
    contract: bytes | None = None,
    action_schema: str | bytes | None = None,
) -> dict[str, int | str | bytes]:
    """Admit every root attribute before opening the private output file."""

    values: list[tuple[str, int | str | bytes]] = []
    if contract is not None:
        values.append(("demo_io_contract", contract))
    if action_schema is not None:
        values.append(("action_schema", action_schema))
    if metadata is not None:
        values.extend(
            (
                ("env_name", metadata.env_name),
                ("env_kwargs", metadata.env_kwargs),
                ("env_args", metadata.env_args),
            )
        )
    values.extend(
        (
            ("num_demos", demos),
            ("total_samples", samples),
            ("num_train", num_train),
            ("num_valid", num_valid),
        )
    )
    if len(values) > limits["max_attributes"]:
        from srb.utils.resource_budget import ResourceLimitError

        raise ResourceLimitError(
            "native HDF5 merge output metadata attribute count",
            limit=limits["max_attributes"],
            observed=len(values),
        )
    from srb.utils.resource_budget import ByteBudget, ResourceLimitError

    budget = ByteBudget(
        "native HDF5 merge output metadata bytes", limits["max_metadata_bytes"]
    )
    admitted: dict[str, int | str | bytes] = {}
    for name, value in values:
        if isinstance(value, int):
            size = _HDF5_INTEGER_ATTRIBUTE_BYTES
        else:
            size = _metadata_value_size(value)
        if size > limits["max_attribute_bytes"]:
            raise ResourceLimitError(
                f"native HDF5 merge output metadata attribute {name!r}",
                limit=limits["max_attribute_bytes"],
                observed=size,
            )
        try:
            budget.consume(size)
        except ResourceLimitError:
            raise ResourceLimitError(
                "native HDF5 merge output metadata bytes",
                limit=limits["max_metadata_bytes"],
                observed=budget.consumed + size,
            ) from None
        admitted[name] = value
    return admitted


def _write_merge_metadata(
    data: Any,
    metadata: _PreparedMergeMetadata | None,
    *,
    np: Any,
) -> None:
    if metadata is None:
        return
    data.attrs["env_name"] = (
        np.bytes_(metadata.env_name)
        if isinstance(metadata.env_name, bytes)
        else metadata.env_name
    )
    data.attrs["env_kwargs"] = (
        np.bytes_(metadata.env_kwargs)
        if isinstance(metadata.env_kwargs, bytes)
        else metadata.env_kwargs
    )
    data.attrs["env_args"] = metadata.env_args


def _native_attribute_storage_size(attribute: Any) -> int:
    """Return encoded HDF5 attribute storage without importing sensor code."""

    try:
        return int(attribute.get_storage_size())
    except (
        AttributeError,
        KeyError,
        OSError,
        RuntimeError,
        TypeError,
        ValueError,
    ):
        shape = getattr(attribute, "shape", None)
        if shape is not None and any(int(dim) == 0 for dim in shape):
            return 0
        raise


def _preflight_metadata_storage(
    handle: Any,
    *,
    source_path: Path,
    h5py: Any,
    limits: dict[str, int],
    metadata_budget: Any,
    copied_roots: tuple[Any, ...] = (),
) -> dict[str, str | bytes]:
    """Use the shared Robomimic admission pass with static child seams."""

    from functools import partial

    aggregate_limit = limits["max_metadata_bytes"] - metadata_budget.consumed
    if aggregate_limit < 0:
        from srb.utils.resource_budget import ResourceLimitError

        raise ResourceLimitError(
            metadata_budget.resource,
            limit=metadata_budget.limit,
            observed=metadata_budget.consumed + 1,
        )
    robomimic = _trusted_import("srb.datasets.robomimic")
    reader = partial(_native_text_reader, h5py=h5py, limits=limits)
    return cast(
        dict[str, str | bytes],
        robomimic._preflight_hdf5_metadata_storage(
            handle,
            path=source_path,
            # The public helper keeps configured ceilings positive; the
            # operation budget carries the valid zero remaining state.
            max_aggregate_bytes=max(1, aggregate_limit),
            metadata_budget=metadata_budget,
            max_attribute_bytes=limits["max_attribute_bytes"],
            max_attributes=limits["max_attributes"],
            text_reader=reader,
            attribute_storage_size=_native_attribute_storage_size,
            copied_roots=copied_roots,
        ),
    )


def _inspection_report(
    request: dict[str, Any],
    limits: dict[str, int],
) -> dict[str, object]:
    import h5py

    robomimic = _trusted_import("srb.datasets.robomimic")
    from srb.utils.resource_budget import ByteBudget, ResourceLimitError

    source_path, expected = _optional_member(request["source"])
    mode = request["mode"]
    require_contract = request["require_demo_io_contract"]
    if mode not in {"count", "deep"} or type(require_contract) is not bool:
        raise ValueError("native HDF5 inspection request fields are invalid")
    with _regular(source_path) as source:
        before_digest, _source_size = _hash(source, limits["max_source_bytes"])
        if expected is not None and before_digest != expected:
            raise ValueError("native HDF5 source digest mismatch")
        with h5py.File(source, "r") as handle:
            metadata_budget = ByteBudget(
                "native HDF5 metadata bytes", limits["max_metadata_bytes"]
            )
            report_budget = ByteBudget(
                "native HDF5 inspection report bytes",
                min(256 * (1 << 20), limits["max_metadata_bytes"] + (1 << 20)),
            )
            errors: _BoundedDiagnostics = _BoundedDiagnostics(
                report_budget,
                max_entries=limits["max_entries"],
                resource="native HDF5 inspection diagnostics",
            )

            def incomplete_report() -> dict[str, object]:
                after_digest = _hash(source, limits["max_source_bytes"])[0]
                if after_digest != before_digest:
                    raise ValueError("native HDF5 source changed during inspection")
                _unchanged_path(source_path, source)
                return {
                    "version": 2,
                    "operation": "inspect",
                    "complete": False,
                    "errors": errors,
                    "demo_count": None,
                    "total_samples": None,
                    "action_dim": None,
                    "train_count": None,
                    "valid_count": None,
                    "obs_keys": [],
                    "metadata": None,
                    "demo_io_contract_sha256": None,
                }

            if not robomimic._preflight_hdf5_storage(
                handle,
                errors=errors,
                max_entries=limits["max_entries"],
                max_depth=32,
            ):
                return incomplete_report()
            data_group = handle.get("data")
            try:
                robomimic._preflight_robomimic_primitive_metadata(handle)
            except (
                AttributeError,
                IndexError,
                KeyError,
                OSError,
                OverflowError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as error:
                detail = str(error).strip()
                errors.append(
                    detail or "native HDF5 primitive metadata inspection failed"
                )
                return incomplete_report()
            try:
                metadata = _preflight_metadata_storage(
                    handle,
                    source_path=Path(source_path),
                    h5py=h5py,
                    limits=limits,
                    metadata_budget=metadata_budget,
                )
            except ResourceLimitError:
                raise
            except (
                AttributeError,
                IndexError,
                KeyError,
                OSError,
                OverflowError,
                RuntimeError,
                TypeError,
                ValueError,
            ):
                errors.append("native HDF5 metadata inspection failed")
                return incomplete_report()
            if data_group is None or not hasattr(data_group, "keys"):
                errors.append("robomimic HDF5 missing required group 'data'")
                return incomplete_report()
            decoded_budget = ByteBudget(
                "native HDF5 decoded bytes", limits["max_decoded_bytes"]
            )
            stats = robomimic._RobomimicInspectionStats(
                obs_keys=_BoundedObservationKeys(
                    report_budget,
                    max_entries=limits["max_entries"],
                    resource="native HDF5 inspection observation keys",
                )
            )
            try:
                if mode == "count":
                    demos, samples, action_dim, num_train, num_valid = (
                        robomimic._inspect_robomimic_counts(
                            h5=handle,
                            errors=errors,
                            decoded_budget=decoded_budget,
                            max_batch_bytes=min(
                                limits["max_array_bytes"], 256 * (1 << 20)
                            ),
                            max_entries=limits["max_entries"],
                            max_array_bytes=limits["max_array_bytes"],
                            max_array_elements=limits["max_array_elements"],
                            max_rows=100_000_000,
                            stats=stats,
                        )
                    )
                else:
                    demos, samples = robomimic._validate_robomimic_demos(
                        h5=handle,
                        errors=errors,
                        decoded_budget=decoded_budget,
                        max_batch_bytes=min(limits["max_array_bytes"], 256 * (1 << 20)),
                        require_demo_io_contract=require_contract,
                        max_entries=limits["max_entries"],
                        max_array_bytes=limits["max_array_bytes"],
                        max_array_elements=limits["max_array_elements"],
                        max_rows=100_000_000,
                        stats=stats,
                    )
                    num_train, num_valid = robomimic._validate_robomimic_masks(
                        h5=handle,
                        demo_keys={str(key) for key in data_group.keys()},
                        errors=errors,
                        decoded_budget=decoded_budget,
                        max_batch_bytes=min(limits["max_array_bytes"], 256 * (1 << 20)),
                        max_array_bytes=limits["max_array_bytes"],
                        max_array_elements=limits["max_array_elements"],
                        max_rows=100_000_000,
                    )
                    data_counts = robomimic._read_hdf5_count_attributes(
                        data_group.attrs
                    )
                    robomimic._validate_metadata_count(
                        metadata=data_counts,
                        key="num_demos",
                        actual=demos,
                        errors=errors,
                    )
                    robomimic._validate_metadata_count(
                        metadata=data_counts,
                        key="total_samples",
                        actual=samples,
                        errors=errors,
                    )
                    if num_train is not None:
                        robomimic._validate_metadata_count(
                            metadata=data_counts,
                            key="num_train",
                            actual=num_train,
                            errors=errors,
                        )
                    if num_valid is not None:
                        robomimic._validate_metadata_count(
                            metadata=data_counts,
                            key="num_valid",
                            actual=num_valid,
                            errors=errors,
                        )
                    action_dim = stats.action_dim
            except ResourceLimitError:
                raise
            except (
                AttributeError,
                IndexError,
                KeyError,
                OSError,
                OverflowError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as error:
                detail = str(error).strip()
                errors.append(
                    "native HDF5 semantic inspection failed"
                    if not detail
                    else f"native HDF5 semantic inspection failed: {detail}"
                )
                return incomplete_report()
            before_digest_after = _hash(source, limits["max_source_bytes"])[0]
            if before_digest_after != before_digest:
                raise ValueError("native HDF5 source changed during inspection")
            _unchanged_path(source_path, source)
            return {
                "version": 2,
                "operation": "inspect",
                "complete": True,
                "errors": errors,
                "demo_count": demos,
                "total_samples": samples,
                "action_dim": stats.action_dim if mode == "deep" else action_dim,
                "train_count": num_train,
                "valid_count": num_valid,
                "obs_keys": sorted(stats.obs_keys),
                "metadata": _metadata_wire_from_values(
                    metadata,
                    limits,
                    report_budget=report_budget,
                ),
                "demo_io_contract_sha256": stats.demo_io_contract_sha256,
            }


def _metadata_wire_from_values(
    values: Mapping[str, str | bytes],
    limits: dict[str, int],
    *,
    report_budget: Any | None = None,
) -> list[list[str]]:
    import base64

    result: list[list[str]] = []
    for name in sorted(values):
        value = values[name]
        if isinstance(value, str):
            encoded = value
            kind = "text"
        elif isinstance(value, bytes):
            encoded = base64.b64encode(value).decode("ascii")
            kind = "bytes"
        else:
            raise ValueError("native HDF5 metadata value is invalid")
        if (
            len(
                value.encode("utf-8", "surrogateescape")
                if isinstance(value, str)
                else value
            )
            > limits["max_attribute_bytes"]
        ):
            raise ValueError("native HDF5 metadata value exceeds its byte limit")
        if report_budget is not None:
            report_budget.consume(len(name) + len(kind) + len(encoded) + 64)
        result.append([name, kind, encoded])
    return result


def _validate_merge_source(
    path: str,
    expected: str,
    *,
    limits: dict[str, int],
    h5py: Any,
    robomimic: Any,
    decoded_budget: Any,
    metadata_budget: Any,
    copy_totals: list[int],
    report_budget: Any,
) -> tuple[
    list[str], dict[str, str | bytes] | None, int, int, tuple | None, bytes | None
]:
    layout: tuple | None = None
    contract: bytes | None = None
    with _regular(path) as source:
        before_digest, _source_size = _hash(source, limits["max_source_bytes"])
        if before_digest != expected:
            raise ValueError("native HDF5 source digest mismatch")
        with h5py.File(source, "r") as handle:
            errors = _BoundedDiagnostics(
                report_budget,
                max_entries=limits["max_entries"],
                resource="native HDF5 merge diagnostics",
            )
            if not robomimic._preflight_hdf5_storage(
                handle,
                errors=errors,
                max_entries=limits["max_entries"],
                max_depth=32,
            ):
                raise ValueError("native HDF5 merge source storage is invalid")
            data = handle.get("data")
            metadata: dict[str, str | bytes] | None = None
            if data is None:
                robomimic._preflight_robomimic_primitive_metadata(handle)
                _preflight_metadata_storage(
                    handle,
                    source_path=Path(path),
                    h5py=h5py,
                    limits=limits,
                    metadata_budget=metadata_budget,
                )
                selected: list[str] = []
                demos = samples = 0
            elif not hasattr(data, "keys"):
                raise ValueError("native HDF5 merge source 'data' must be a group")
            else:
                robomimic._preflight_robomimic_primitive_metadata(handle)
                # Numeric order keeps demo_10 after demo_2 when renumbering.
                selected = sorted(
                    (str(key) for key in data.keys() if str(key).startswith("demo_")),
                    key=lambda key: (
                        (0, int(key[5:]), key) if key[5:].isdigit() else (1, 0, key)
                    ),
                )
                metadata = _preflight_metadata_storage(
                    handle,
                    source_path=Path(path),
                    h5py=h5py,
                    limits=limits,
                    metadata_budget=metadata_budget,
                    copied_roots=tuple(data[key] for key in selected),
                )
                if selected:
                    stats = _trusted_import(
                        "srb.datasets.robomimic"
                    )._RobomimicInspectionStats()
                    demos, samples = robomimic._validate_robomimic_demos(
                        h5=handle,
                        demo_keys=selected,
                        errors=errors,
                        decoded_budget=decoded_budget,
                        max_batch_bytes=min(limits["max_array_bytes"], 256 * (1 << 20)),
                        require_demo_io_contract=False,
                        max_entries=limits["max_entries"],
                        max_array_bytes=limits["max_array_bytes"],
                        max_array_elements=limits["max_array_elements"],
                        max_rows=100_000_000,
                        stats=stats,
                    )
                    for key in selected:
                        _admit_copy(data[key], h5py, limits, copy_totals)
                    layout = (stats.action_dim, frozenset(stats.obs_keys))
                else:
                    demos = samples = 0
                if robomimic._read_hdf5_demo_io_contract_attr(data.attrs) is not None:
                    contract = bytes(data.attrs["demo_io_contract"])
                data_counts = robomimic._read_hdf5_count_attributes(data.attrs)
                robomimic._validate_metadata_count(
                    metadata=data_counts, key="num_demos", actual=demos, errors=errors
                )
                robomimic._validate_metadata_count(
                    metadata=data_counts,
                    key="total_samples",
                    actual=samples,
                    errors=errors,
                )
                if errors:
                    raise ValueError(
                        "native HDF5 merge source semantic validation failed: "
                        + "; ".join(errors)[:4000]
                    )
        if _hash(source, limits["max_source_bytes"])[0] != expected:
            raise ValueError("native HDF5 source changed during validation")
        _unchanged_path(path, source)
    return selected, metadata, demos, samples, layout, contract


def _merge_private(
    request: dict[str, Any], limits: dict[str, int]
) -> dict[str, object]:
    import h5py
    import numpy as np

    from srb.utils.resource_budget import ByteBudget

    sources = request["sources"]
    output_dir = request["output_dir"]
    output_name = request["output_name"]
    split = request["train_split"]
    if (
        not isinstance(sources, list)
        or len(sources) > limits["max_entries"]
        or not isinstance(output_dir, list)
        or len(output_dir) != 3
        or not isinstance(output_dir[0], str)
        or type(output_dir[1]) is not int
        or type(output_dir[2]) is not int
        or output_name != "merged.hdf5"
    ):
        raise ValueError("native HDF5 merge request fields are invalid")
    split = _require_train_split(split)
    directory_path = output_dir[0]
    expected_directory = (output_dir[1], output_dir[2])
    metadata = _metadata_request_values(request["metadata"], limits)
    robomimic = _trusted_import("srb.datasets.robomimic")
    decoded_budget = ByteBudget(
        "native HDF5 merge decoded bytes", limits["max_decoded_bytes"]
    )
    metadata_budget = ByteBudget(
        "native HDF5 merge metadata bytes", limits["max_metadata_bytes"]
    )
    report_budget = ByteBudget(
        "native HDF5 merge report bytes", limits["max_metadata_bytes"]
    )
    selected_sources: list[tuple[str, list[str]]] = []
    first_metadata: dict[str, str | bytes] | None = None
    first_layout: tuple | None = None
    contracts: set[bytes | None] = set()
    wire_bytes = 0
    copy_totals = [0, 0]
    planned_demos = 0
    planned_samples = 0
    for member in sources:
        path, expected = _member(member)
        with _regular(path) as source:
            _actual, size = _hash(source, limits["max_source_bytes"])
        wire_bytes += size
        if wire_bytes > limits["max_source_bytes"]:
            raise ValueError("native HDF5 source aggregate bytes exceed their limit")
        demo_keys, values, source_demos, source_samples, layout, source_contract = (
            _validate_merge_source(
                path,
                expected,
                limits=limits,
                h5py=h5py,
                robomimic=robomimic,
                decoded_budget=decoded_budget,
                metadata_budget=metadata_budget,
                copy_totals=copy_totals,
                report_budget=report_budget,
            )
        )
        selected_sources.append((path, demo_keys))
        planned_demos += source_demos
        planned_samples += source_samples
        contracts.add(source_contract)
        if len(contracts) > 1:
            raise ValueError("native HDF5 merge sources differ in demo_io_contract")
        if layout is not None:
            if first_layout is None:
                first_layout = layout
            elif layout != first_layout:
                raise ValueError(
                    "native HDF5 merge sources differ in action dimension "
                    "or observation keys"
                )
        if values is not None:
            if first_metadata is None:
                first_metadata = values
            elif (
                _merge_metadata_fields(values)[1]
                != _merge_metadata_fields(first_metadata)[1]
            ):
                raise ValueError("native HDF5 merge sources differ in env_name")
            elif values.get("action_schema") != first_metadata.get("action_schema"):
                raise ValueError("native HDF5 merge sources differ in action_schema")
    effective_metadata = first_metadata if first_metadata is not None else metadata
    contract = next(iter(contracts), None)
    action_schema = (
        effective_metadata.get("action_schema")
        if effective_metadata is not None
        else None
    )
    static_output_bytes = _prepare_merge_output_static_bytes(
        effective_metadata,
        limits=limits,
        contract=contract,
        action_schema=action_schema,
    )
    prepared_metadata = _prepare_merge_metadata(
        effective_metadata,
        robomimic=robomimic,
        limits=limits,
        max_env_args_bytes=limits["max_metadata_bytes"] - static_output_bytes,
    )
    planned_train = robomimic._robomimic_train_count(planned_demos, split)
    planned_valid = planned_demos - planned_train
    output_attributes = _prepare_merge_output_attributes(
        prepared_metadata,
        demos=planned_demos,
        samples=planned_samples,
        num_train=planned_train,
        num_valid=planned_valid,
        limits=limits,
        contract=contract,
        action_schema=action_schema,
    )
    demos = 0
    samples = 0
    output_path = str(Path(directory_path) / output_name)
    with _directory(directory_path, expected=expected_directory) as directory_fd:
        descriptor = os.open(
            output_name,
            os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
            dir_fd=directory_fd,
        )
        with os.fdopen(descriptor, "r+b") as output:
            with h5py.File(output, "w") as target:
                data = target.create_group("data")
                _write_merge_metadata(
                    data,
                    prepared_metadata,
                    np=np,
                )
                for name in _HDF5_COUNT_ATTRIBUTES:
                    data.attrs[name] = output_attributes[name]
                if contract is not None:
                    data.attrs["demo_io_contract"] = np.bytes_(contract)
                if action_schema is not None:
                    data.attrs["action_schema"] = (
                        np.bytes_(action_schema)
                        if isinstance(action_schema, bytes)
                        else action_schema
                    )
                for path, demo_keys in selected_sources:
                    with _regular(path) as source:
                        before_digest, _ = _hash(source, limits["max_source_bytes"])
                        expected = next(
                            digest
                            for member in sources
                            if member[0] == path
                            for digest in [member[1]]
                        )
                        if before_digest != expected:
                            raise ValueError("native HDF5 source changed before copy")
                        with h5py.File(source, "r") as origin:
                            origin_data = origin.get("data")
                            if origin_data is not None:
                                for key in demo_keys:
                                    demo = origin_data[key]
                                    shape = demo["actions"].shape
                                    if not shape:
                                        raise ValueError(
                                            "native HDF5 actions require a leading dimension"
                                        )
                                    origin.copy(demo, data, name=f"demo_{demos}")
                                    demos += 1
                                    samples += int(shape[0])
                        if _hash(source, limits["max_source_bytes"])[0] != expected:
                            raise ValueError("native HDF5 source changed during copy")
                        _unchanged_path(path, source)
                keys = [f"demo_{index}" for index in range(demos)]
                target.create_group("mask")
                target["mask/train"] = np.array(keys[:planned_train], dtype="S")
                target["mask/valid"] = np.array(keys[planned_train:], dtype="S")
            output.flush()
            os.fsync(output.fileno())
            digest, size = _hash(output, limits["max_output_bytes"])
            _unchanged_path(output_path, output)
    return {
        "version": 2,
        "operation": "merge_private",
        "demo_count": demos,
        "total_samples": samples,
        "output_path": output_path,
        "output_sha256": digest,
        "output_bytes": size,
    }


def _normalize(
    request: dict[str, Any], limits: dict[str, int], normalizer: NativeNormalizer
) -> dict[str, object]:
    """Load the installed/source transform only after native limits are active."""
    options = request["normalization"]
    if (
        not isinstance(options, dict)
        or set(options) != {"max_decoded_bytes", "max_batch_bytes"}
        or request["sources"]
    ):
        raise ValueError("native HDF5 normalization options are invalid")
    decoded = _integer(options["max_decoded_bytes"], 16 * _GIB)
    batch = _integer(options["max_batch_bytes"], 2 * _GIB)
    path, expected = _member(request["output"])
    with _regular(path, writable=True) as output:
        if _hash(output, limits["max_source_bytes"])[0] != expected:
            raise ValueError("native HDF5 normalization source digest mismatch")
        try:
            parameters = normalizer(
                output,
                limits,
                min(decoded, limits["max_decoded_bytes"]),
                min(batch, limits["max_array_bytes"]),
            )
        except ValueError as error:
            # Domain diagnostics are rendered only inside this limited worker.
            return {"version": 1, "error": str(error)[:4096]}
        output.flush()
        os.fsync(output.fileno())
        digest, size = _hash(output, limits["max_output_bytes"])
        _unchanged_path(path, output)
    return {
        "version": 1,
        "demo_count": 0,
        "total_samples": 0,
        "output_sha256": digest,
        "output_bytes": size,
        "parameters": None if parameters is None else parameters.decode("ascii"),
    }


def _respond(path: str, value: dict[str, object]) -> None:
    payload = json.dumps(value, ensure_ascii=True, allow_nan=False).encode("ascii")
    if len(payload) > 8192:
        raise ValueError("native HDF5 response exceeds its limit")
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600
    )
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)


def _respond_bounded(path: str, value: dict[str, object], *, max_bytes: int) -> None:
    bounded_json_dumps = _trusted_import("srb.utils.json_types").bounded_json_dumps

    payload = bounded_json_dumps(
        value,
        context="native HDF5 response",
        max_bytes=max_bytes,
        max_nodes=1_000_000,
        max_depth=64,
        allow_nan=False,
    ).encode("ascii")
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)


def _error_value(error: BaseException, *, version: int) -> dict[str, object]:
    if version == 1:
        return {
            "version": 1,
            "error": "native HDF5 worker rejected input or exhausted a resource limit",
        }
    if error.__class__.__name__ == "ResourceLimitError":
        resource = getattr(error, "resource", "native HDF5 resource")
        limit = getattr(error, "limit", 0)
        observed = getattr(error, "observed", 0)
        if (
            not isinstance(resource, str)
            or len(resource) > 256
            or type(limit) is not int
            or type(observed) is not int
            or limit <= 0
            or observed <= limit
        ):
            return {
                "version": version,
                "kind": "worker",
                "message": "native HDF5 worker rejected input or exhausted a resource limit",
            }
        return {
            "version": version,
            "kind": "resource_limit",
            "resource": resource,
            "limit": limit,
            "observed": observed,
        }
    message = "native HDF5 worker rejected input or exhausted a resource limit"
    if isinstance(error, ValueError):
        detail = str(error).strip()
        if detail:
            message = detail[:_MAX_ERROR_BYTES]
    return {
        "version": version,
        "kind": "worker",
        "message": message,
    }


def main(*, normalizer: NativeNormalizer | None = None) -> int:
    if len(sys.argv) != 5:
        return 2
    request_path, response_path, address_arg, output_arg = sys.argv[1:]
    try:
        address, output = int(address_arg), int(output_arg)
        _apply_limits(address, output)
        JsonDecodeBudget = _trusted_import("srb.utils.json_types").JsonDecodeBudget

        with _regular(request_path) as stream:
            if os.fstat(stream.fileno()).st_size > 64 * (1 << 20):
                raise ValueError("native HDF5 request exceeds its limit")
            payload = stream.read(64 * (1 << 20) + 1)
        if len(payload) > 64 * (1 << 20):
            raise ValueError("native HDF5 request exceeds its limit")
        request = JsonDecodeBudget(
            max_nodes=1_000_000,
            max_depth=64,
            max_total_nodes=16_000_000,
        ).loads(
            payload,
            context="native HDF5 request",
            max_bytes=64 * (1 << 20),
        )
        del payload
        if not isinstance(request, dict) or not isinstance(request.get("limits"), dict):
            raise ValueError("native HDF5 request is invalid")
        version = request.get("version")
        if type(version) is not int or version not in {1, 2}:
            raise ValueError("native HDF5 request version is invalid")
        declared = request["limits"]
        limits = _declared_limits(declared)
        if (
            address != limits["address_space_bytes"]
            or output != limits["max_output_bytes"]
        ):
            raise ValueError("native HDF5 command limits differ from request")
        if version == 1:
            if set(request) != {
                "version",
                "sources",
                "output",
                "train_split",
                "limits",
                "normalization",
            }:
                raise ValueError("native HDF5 request fields are invalid")
            if request.get("normalization") is None:
                result = _merge(request, limits)
            elif normalizer is None:
                raise ValueError("native HDF5 worker has no normalization adapter")
            else:
                result = _normalize(request, limits, normalizer)
            _respond(response_path, result)
            return 0
        operation = request.get("operation")
        if operation == "inspect":
            if set(request) != {
                "version",
                "operation",
                "source",
                "mode",
                "require_demo_io_contract",
                "limits",
            }:
                raise ValueError("native HDF5 inspection request fields are invalid")
            result = _inspection_report(request, limits)
            _respond_bounded(
                response_path,
                result,
                max_bytes=min(
                    256 * (1 << 20), limits["max_metadata_bytes"] + (1 << 20)
                ),
            )
            return 0
        if operation == "merge_private":
            if set(request) != {
                "version",
                "operation",
                "sources",
                "output_dir",
                "output_name",
                "train_split",
                "metadata",
                "limits",
            }:
                raise ValueError("native HDF5 merge request fields are invalid")
            result = _merge_private(request, limits)
            _respond_bounded(
                response_path,
                result,
                max_bytes=min(
                    256 * (1 << 20), limits["max_metadata_bytes"] + (1 << 20)
                ),
            )
            return 0
        raise ValueError("native HDF5 operation is invalid")
    except (
        ImportError,
        KeyError,
        MemoryError,
        OSError,
        OverflowError,
        RecursionError,
        RuntimeError,
        TypeError,
        ValueError,
    ) as caught_error:
        # Never render provider exceptions or payloads into the parent protocol.
        try:
            response_version = (
                request.get("version")
                if isinstance(locals().get("request"), dict)
                and type(request.get("version")) is int
                and request.get("version") in {1, 2}
                else 1
            )
            if response_version == 2 and isinstance(locals().get("limits"), dict):
                _respond_bounded(
                    response_path,
                    _error_value(caught_error, version=2),
                    max_bytes=min(
                        256 * (1 << 20), limits["max_metadata_bytes"] + (1 << 20)
                    ),
                )
            else:
                _respond(response_path, _error_value(caught_error, version=1))
        except (MemoryError, OSError, ValueError, TypeError):
            return 1
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
