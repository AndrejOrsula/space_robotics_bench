"""Isolated worker for :mod:`srb.datasets.hdf5_metadata`.

Install native limits before importing h5py.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import resource
import stat
import struct
import sys
from pathlib import Path
from typing import Final

_ALLOWED_NAMES: Final[frozenset[str]] = frozenset(
    {"env_name", "env_kwargs", "env_args", "action_schema", "model_file"}
)
_MAX_REQUEST_BYTES: Final[int] = 64 * 1024
_MAX_OBJECT_PATH_CHARS: Final[int] = 4096
_MAX_OBJECT_PATH_COMPONENTS: Final[int] = 32
_MAX_ERROR_BYTES: Final[int] = 4096
_MAX_VALUE_BYTES: Final[int] = 64 * (1 << 20)
_MAX_AGGREGATE_BYTES: Final[int] = 128 * (1 << 20)
_MAX_EXTERNAL_FILE_BYTES: Final[int] = 16 * (1 << 30)
_MAX_ADDRESS_SPACE_BYTES: Final[int] = 2 * (1 << 30)
_RESPONSE_MAGIC: Final[bytes] = b"SRBH5M01"
_HASH_RESPONSE_MAGIC: Final[bytes] = b"SRBH5H01"
_RESPONSE_PREFIX = struct.Struct(">8sB")
_HASH_SUCCESS = struct.Struct(">8sB32s")
_SUCCESS_COUNT = struct.Struct(">I")
_SUCCESS_RECORD = struct.Struct(">H B Q")
_ERROR_HEADER = struct.Struct(">B H Q Q")

_STATUS_SUCCESS: Final[int] = 0
_STATUS_ERROR: Final[int] = 1
_VALUE_TEXT: Final[int] = 1
_VALUE_BYTES: Final[int] = 2

_ERROR_GENERIC: Final[int] = 1
_ERROR_VALUE_LIMIT: Final[int] = 2
_ERROR_DIGEST: Final[int] = 3
_ERROR_PLATFORM: Final[int] = 4
_ERROR_SOURCE: Final[int] = 5
_ERROR_LINK: Final[int] = 6
_ERROR_AGGREGATE_LIMIT: Final[int] = 7
_ERROR_NATIVE_LIMIT: Final[int] = 8
_ERROR_RESPONSE_LIMIT: Final[int] = 9
_ERROR_SOURCE_LIMIT: Final[int] = 10


class _WorkerFailure(Exception):
    def __init__(
        self,
        code: int,
        message: str,
        *,
        limit: int = 0,
        observed: int = 0,
    ) -> None:
        self.code = code
        self.message = message[:_MAX_ERROR_BYTES]
        self.limit = limit
        self.observed = observed
        super().__init__(self.message)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--mode", choices=("read", "hash"), default="read")
    parser.add_argument("--source", required=True)
    parser.add_argument("--request", required=True)
    parser.add_argument("--response", required=True)
    parser.add_argument("--max-value-bytes", required=True, type=int)
    parser.add_argument("--max-aggregate-bytes", required=True, type=int)
    parser.add_argument("--max-source-bytes", required=True, type=int)
    parser.add_argument("--address-space-bytes", required=True, type=int)
    parser.add_argument("--response-limit-bytes", required=True, type=int)
    return parser.parse_args()


def _read_request(path: Path) -> dict[str, object]:
    if not hasattr(os, "O_NOFOLLOW"):
        raise _WorkerFailure(
            _ERROR_PLATFORM,
            "HDF5 metadata no-follow reads are unavailable",
        )
    flags = (
        os.O_RDONLY
        | os.O_NOFOLLOW
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise _WorkerFailure(
            _ERROR_SOURCE, "metadata request is unavailable"
        ) from error
    try:
        source_stat = os.fstat(descriptor)
        if (
            not stat.S_ISREG(source_stat.st_mode)
            or source_stat.st_size > _MAX_REQUEST_BYTES
        ):
            raise _WorkerFailure(_ERROR_SOURCE, "metadata request is invalid")
        payload = os.read(descriptor, _MAX_REQUEST_BYTES + 1)
        if len(payload) > _MAX_REQUEST_BYTES:
            raise _WorkerFailure(_ERROR_SOURCE, "metadata request is too large")
    finally:
        os.close(descriptor)
    try:
        request = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise _WorkerFailure(
            _ERROR_GENERIC, "metadata request is not valid JSON"
        ) from error
    if not isinstance(request, dict):
        raise _WorkerFailure(_ERROR_GENERIC, "metadata request is not an object")
    return request


def _request_fields(
    request: dict[str, object],
) -> tuple[tuple[str, ...], str, str]:
    raw_names = request.get("names")
    object_path = request.get("object_path")
    expected_sha256 = request.get("expected_sha256")
    if not isinstance(raw_names, list) or not isinstance(object_path, str):
        raise _WorkerFailure(_ERROR_GENERIC, "metadata request fields are invalid")
    if len(raw_names) > len(_ALLOWED_NAMES):
        raise _WorkerFailure(_ERROR_GENERIC, "metadata request names are invalid")
    names: list[str] = []
    for name in raw_names:
        if not isinstance(name, str) or name not in _ALLOWED_NAMES or name in names:
            raise _WorkerFailure(_ERROR_GENERIC, "metadata request names are invalid")
        names.append(name)
    if len(object_path) > _MAX_OBJECT_PATH_CHARS:
        raise _WorkerFailure(_ERROR_GENERIC, "metadata object path is invalid")
    if not object_path or "\x00" in object_path or object_path == "/":
        raise _WorkerFailure(_ERROR_GENERIC, "metadata object path is invalid")
    components = object_path.split("/")
    if components[0] == "":
        components = components[1:]
    if (
        not components
        or len(components) > _MAX_OBJECT_PATH_COMPONENTS
        or any(component in {"", ".", ".."} for component in components)
    ):
        raise _WorkerFailure(_ERROR_GENERIC, "metadata object path is invalid")
    if not isinstance(expected_sha256, str) or len(expected_sha256) != 64:
        raise _WorkerFailure(_ERROR_GENERIC, "metadata digest is invalid")
    if any(character not in "0123456789abcdefABCDEF" for character in expected_sha256):
        raise _WorkerFailure(_ERROR_GENERIC, "metadata digest is invalid")
    expected_sha256 = expected_sha256.lower()
    return tuple(names), object_path, expected_sha256


def _lower_limit(limit_name: int, requested: int) -> None:
    try:
        soft, hard = resource.getrlimit(limit_name)
        target = requested
        if hard != resource.RLIM_INFINITY:
            target = min(target, hard)
        if soft != resource.RLIM_INFINITY:
            target = min(target, soft)
        resource.setrlimit(limit_name, (target, hard))
    except (OSError, ValueError) as error:
        raise _WorkerFailure(
            _ERROR_PLATFORM,
            "metadata native limits could not be installed",
        ) from error


def _apply_native_limits(address_space_bytes: int, response_limit_bytes: int) -> None:
    if not sys.platform.startswith("linux"):
        raise _WorkerFailure(
            _ERROR_PLATFORM,
            "metadata native limits require Linux",
        )
    if not hasattr(resource, "RLIMIT_AS") or not hasattr(resource, "RLIMIT_FSIZE"):
        raise _WorkerFailure(
            _ERROR_PLATFORM,
            "metadata native limits are unavailable",
        )
    if not 1 <= address_space_bytes <= _MAX_ADDRESS_SPACE_BYTES:
        raise _WorkerFailure(_ERROR_GENERIC, "metadata address-space limit is invalid")
    if not 1 <= response_limit_bytes <= _MAX_AGGREGATE_BYTES + (1 << 20):
        raise _WorkerFailure(_ERROR_GENERIC, "metadata response limit is invalid")
    for variable in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[variable] = "1"
    _lower_limit(resource.RLIMIT_AS, address_space_bytes)
    _lower_limit(resource.RLIMIT_FSIZE, response_limit_bytes)


def _hash_stream(
    stream: object,
    *,
    descriptor: int,
    max_source_bytes: int,
) -> str:
    seek = getattr(stream, "seek")
    read = getattr(stream, "read")
    seek(0)
    digest = hashlib.sha256()
    observed = 0
    while True:
        remaining = max_source_bytes - observed
        chunk = read(min(1 << 20, remaining + 1))
        observed += len(chunk)
        if observed > max_source_bytes:
            raise _WorkerFailure(
                _ERROR_SOURCE_LIMIT,
                "metadata source exceeds its byte limit",
                limit=max_source_bytes,
                observed=observed,
            )
        if not chunk:
            break
        digest.update(chunk)
    seek(0)
    final_stat = os.fstat(descriptor)
    if final_stat.st_size > max_source_bytes:
        raise _WorkerFailure(
            _ERROR_SOURCE_LIMIT,
            "metadata source exceeds its byte limit",
            limit=max_source_bytes,
            observed=final_stat.st_size,
        )
    return digest.hexdigest()


def _hash_source(source_path: Path, *, max_source_bytes: int) -> bytes:
    if not hasattr(os, "O_NOFOLLOW"):
        raise _WorkerFailure(
            _ERROR_PLATFORM, "metadata source no-follow is unavailable"
        )
    flags = (
        os.O_RDONLY
        | os.O_NOFOLLOW
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        descriptor = os.open(source_path, flags)
    except OSError as error:
        raise _WorkerFailure(
            _ERROR_SOURCE, "metadata source could not be opened"
        ) from error
    try:
        source_stat = os.fstat(descriptor)
        if not stat.S_ISREG(source_stat.st_mode):
            raise _WorkerFailure(_ERROR_SOURCE, "metadata source is not a regular file")
        if source_stat.st_size > max_source_bytes:
            raise _WorkerFailure(
                _ERROR_SOURCE_LIMIT,
                "metadata source exceeds its byte limit",
                limit=max_source_bytes,
                observed=source_stat.st_size,
            )
        digest = hashlib.sha256()
        observed = 0
        while True:
            remaining = max_source_bytes - observed
            chunk = os.read(descriptor, min(1 << 20, remaining + 1))
            observed += len(chunk)
            if observed > max_source_bytes:
                raise _WorkerFailure(
                    _ERROR_SOURCE_LIMIT,
                    "metadata source exceeds its byte limit",
                    limit=max_source_bytes,
                    observed=observed,
                )
            if not chunk:
                break
            digest.update(chunk)
        final_stat = os.fstat(descriptor)
        if final_stat.st_size > max_source_bytes:
            raise _WorkerFailure(
                _ERROR_SOURCE_LIMIT,
                "metadata source exceeds its byte limit",
                limit=max_source_bytes,
                observed=final_stat.st_size,
            )
        return digest.digest()
    finally:
        os.close(descriptor)


def _resolve_object(handle: object, object_path: str, h5py: object) -> object:
    current = handle
    components = object_path.split("/")
    if components[0] == "":
        components = components[1:]
    hard_link = getattr(h5py, "HardLink")
    group_type = getattr(h5py, "Group")
    for component in components:
        if not isinstance(current, group_type):
            raise _WorkerFailure(
                _ERROR_GENERIC,
                "metadata object path traverses a non-group HDF5 object",
            )
        get = getattr(current, "get")
        try:
            link = get(component, getlink=True)
        except (KeyError, OSError, TypeError, ValueError) as error:
            raise _WorkerFailure(
                _ERROR_GENERIC,
                "metadata object path could not be traversed",
            ) from error
        if not isinstance(link, hard_link):
            raise _WorkerFailure(
                _ERROR_LINK,
                "metadata object path contains an HDF5 link",
            )
        try:
            current = current[component]  # type: ignore[index]
        except (KeyError, OSError, TypeError, ValueError) as error:
            raise _WorkerFailure(
                _ERROR_GENERIC,
                "metadata object path could not be traversed",
            ) from error
    return current


def _attribute_value(
    attrs: object,
    name: str,
    *,
    h5py: object,
    max_value_bytes: int,
) -> tuple[int, bytes] | None:
    contains = getattr(attrs, "__contains__")
    if not contains(name):
        return None
    get_id = getattr(attrs, "get_id")
    attribute_id = get_id(name)
    try:
        shape = tuple(getattr(attribute_id, "shape"))
        if shape:
            raise _WorkerFailure(
                _ERROR_GENERIC, "metadata attribute must be scalar text"
            )
        dtype = getattr(attribute_id, "dtype")
        kind = getattr(dtype, "kind")
        check_vlen = getattr(h5py, "check_dtype")
        vlen_type = check_vlen(vlen=dtype)
        if kind == "S":
            declared_size = int(getattr(dtype, "itemsize"))
            if declared_size > max_value_bytes:
                raise _WorkerFailure(
                    _ERROR_VALUE_LIMIT,
                    "metadata value exceeds its byte limit",
                    limit=max_value_bytes,
                    observed=declared_size,
                )
        elif kind != "O" or vlen_type not in (str, bytes):
            raise _WorkerFailure(
                _ERROR_GENERIC, "metadata attribute must be scalar text"
            )
        value = attrs[name]  # type: ignore[index]
        if isinstance(value, str):
            payload = value.encode("utf-8", "surrogateescape")
            value_kind = _VALUE_TEXT
        elif isinstance(value, bytes):
            payload = bytes(value)
            value_kind = _VALUE_BYTES
        else:
            raise _WorkerFailure(
                _ERROR_GENERIC, "metadata attribute must be scalar text"
            )
        if len(payload) > max_value_bytes:
            raise _WorkerFailure(
                _ERROR_VALUE_LIMIT,
                "metadata value exceeds its byte limit",
                limit=max_value_bytes,
                observed=len(payload),
            )
        return value_kind, payload
    finally:
        attribute_id.close()


def _read_values(
    source_path: Path,
    *,
    names: tuple[str, ...],
    object_path: str,
    expected_sha256: str,
    max_value_bytes: int,
    max_aggregate_bytes: int,
    max_source_bytes: int,
) -> list[tuple[str, int, bytes]]:
    if not hasattr(os, "O_NOFOLLOW"):
        raise _WorkerFailure(
            _ERROR_PLATFORM, "metadata source no-follow is unavailable"
        )
    flags = (
        os.O_RDONLY
        | os.O_NOFOLLOW
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        descriptor = os.open(source_path, flags)
    except OSError as error:
        raise _WorkerFailure(
            _ERROR_SOURCE, "metadata source could not be opened"
        ) from error
    try:
        before_stat = os.fstat(descriptor)
        if not stat.S_ISREG(before_stat.st_mode):
            raise _WorkerFailure(_ERROR_SOURCE, "metadata source is not a regular file")
        if before_stat.st_size > max_source_bytes:
            raise _WorkerFailure(
                _ERROR_SOURCE_LIMIT,
                "metadata source exceeds its byte limit",
                limit=max_source_bytes,
                observed=before_stat.st_size,
            )
        stream = os.fdopen(descriptor, "rb", closefd=False)
        try:
            before_digest = _hash_stream(
                stream,
                descriptor=descriptor,
                max_source_bytes=max_source_bytes,
            )
            if before_digest != expected_sha256:
                raise _WorkerFailure(_ERROR_DIGEST, "metadata source digest mismatch")
            import h5py

            with h5py.File(stream, "r") as handle:
                target = _resolve_object(handle, object_path, h5py)
                attrs = getattr(target, "attrs")
                values: list[tuple[str, int, bytes]] = []
                aggregate = 0
                for name in names:
                    value = _attribute_value(
                        attrs,
                        name,
                        h5py=h5py,
                        max_value_bytes=max_value_bytes,
                    )
                    if value is None:
                        continue
                    value_kind, payload = value
                    aggregate += len(payload)
                    if aggregate > max_aggregate_bytes:
                        raise _WorkerFailure(
                            _ERROR_AGGREGATE_LIMIT,
                            "metadata aggregate exceeds its byte limit",
                            limit=max_aggregate_bytes,
                            observed=aggregate,
                        )
                    values.append((name, value_kind, payload))
            after_digest = _hash_stream(
                stream,
                descriptor=descriptor,
                max_source_bytes=max_source_bytes,
            )
            after_stat = os.fstat(descriptor)
        finally:
            stream.close()
        if (
            before_stat.st_dev != after_stat.st_dev
            or before_stat.st_ino != after_stat.st_ino
            or before_digest != after_digest
        ):
            raise _WorkerFailure(_ERROR_DIGEST, "metadata source changed while reading")
        return values
    finally:
        os.close(descriptor)


def _success_payload(
    values: list[tuple[str, int, bytes]], response_limit_bytes: int
) -> bytes:
    frame = bytearray(_RESPONSE_PREFIX.pack(_RESPONSE_MAGIC, _STATUS_SUCCESS))
    frame.extend(_SUCCESS_COUNT.pack(len(values)))
    for name, value_kind, value in values:
        encoded_name = name.encode("ascii")
        frame.extend(_SUCCESS_RECORD.pack(len(encoded_name), value_kind, len(value)))
        frame.extend(encoded_name)
        frame.extend(value)
        if len(frame) > response_limit_bytes:
            raise _WorkerFailure(
                _ERROR_RESPONSE_LIMIT,
                "metadata response exceeds its byte limit",
                limit=response_limit_bytes,
                observed=len(frame),
            )
    return bytes(frame)


def _hash_success_payload(digest: bytes) -> bytes:
    if len(digest) != hashlib.sha256().digest_size:
        raise _WorkerFailure(_ERROR_GENERIC, "metadata hash is invalid")
    return _HASH_SUCCESS.pack(_HASH_RESPONSE_MAGIC, _STATUS_SUCCESS, digest)


def _error_payload(
    code: int,
    message: str,
    *,
    magic: bytes = _RESPONSE_MAGIC,
    limit: int = 0,
    observed: int = 0,
) -> bytes:
    encoded_message = message.encode("utf-8", "replace")[:_MAX_ERROR_BYTES]
    return (
        _RESPONSE_PREFIX.pack(magic, _STATUS_ERROR)
        + _ERROR_HEADER.pack(code, len(encoded_message), limit, observed)
        + encoded_message
    )


def _write_response(path: Path, payload: bytes) -> None:
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_TRUNC
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(path, flags, 0o600)
    try:
        view = memoryview(payload)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise OSError("short metadata response write")
            view = view[written:]
    finally:
        os.close(descriptor)


def _safe_error(
    response_path: Path,
    code: int,
    message: str,
    *,
    magic: bytes = _RESPONSE_MAGIC,
    limit: int = 0,
    observed: int = 0,
) -> int:
    try:
        _write_response(
            response_path,
            _error_payload(
                code,
                message,
                magic=magic,
                limit=limit,
                observed=observed,
            ),
        )
    except OSError:
        return 2
    return 0


def main() -> int:
    arguments = _arguments()
    response_magic = (
        _HASH_RESPONSE_MAGIC if arguments.mode == "hash" else _RESPONSE_MAGIC
    )
    try:
        if not 1 <= arguments.max_value_bytes <= _MAX_VALUE_BYTES:
            raise _WorkerFailure(_ERROR_GENERIC, "metadata value limit is invalid")
        if not 1 <= arguments.max_aggregate_bytes <= _MAX_AGGREGATE_BYTES:
            raise _WorkerFailure(_ERROR_GENERIC, "metadata aggregate limit is invalid")
        if not 1 <= arguments.max_source_bytes <= _MAX_EXTERNAL_FILE_BYTES:
            raise _WorkerFailure(_ERROR_GENERIC, "metadata source limit is invalid")
        request = _read_request(Path(arguments.request))
        request_mode = request.get("mode", "read")
        if request_mode != arguments.mode:
            raise _WorkerFailure(_ERROR_GENERIC, "metadata worker mode is invalid")
        _apply_native_limits(
            arguments.address_space_bytes, arguments.response_limit_bytes
        )
        if arguments.mode == "hash":
            if set(request) != {"mode"}:
                raise _WorkerFailure(_ERROR_GENERIC, "metadata hash request is invalid")
            response = _hash_success_payload(
                _hash_source(
                    Path(arguments.source),
                    max_source_bytes=arguments.max_source_bytes,
                )
            )
        else:
            names, object_path, expected_sha256 = _request_fields(request)
            values = _read_values(
                Path(arguments.source),
                names=names,
                object_path=object_path,
                expected_sha256=expected_sha256,
                max_value_bytes=arguments.max_value_bytes,
                max_aggregate_bytes=arguments.max_aggregate_bytes,
                max_source_bytes=arguments.max_source_bytes,
            )
            response = _success_payload(values, arguments.response_limit_bytes)
        _write_response(Path(arguments.response), response)
        return 0
    except _WorkerFailure as error:
        return _safe_error(
            Path(arguments.response),
            error.code,
            error.message,
            magic=response_magic,
            limit=error.limit,
            observed=error.observed,
        )
    except MemoryError:
        return _safe_error(
            Path(arguments.response),
            _ERROR_NATIVE_LIMIT,
            "metadata worker exhausted its native memory limit",
            magic=response_magic,
        )
    except (
        ImportError,
        KeyError,
        OSError,
        OverflowError,
        RuntimeError,
        TypeError,
        UnicodeError,
        ValueError,
    ) as error:
        del error
        return _safe_error(
            Path(arguments.response),
            _ERROR_GENERIC,
            "metadata worker rejected the HDF5 source",
            magic=response_magic,
        )


if __name__ == "__main__":
    raise SystemExit(main())
