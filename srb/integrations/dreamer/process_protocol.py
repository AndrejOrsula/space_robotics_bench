"""Versioned numeric wire protocol for Dreamer worker processes.

Frames pair bounded canonical JSON metadata with contiguous ndarray bytes;
transport stays independent of Isaac and learner state and never uses pickle.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Iterable, Mapping, TypeAlias

import numpy as np

from srb.utils.json_types import ImmutableValueMixin

PROTOCOL_NAME = "srb.dreamer.process"
PROTOCOL_VERSION = 4
FRAME_MAGIC = b"SRBDR004"
FRAME_PREFIX = struct.Struct("!8sII")

MAX_HEADER_BYTES = 64 * 1024
MAX_PAYLOAD_BYTES = 64 * 1024 * 1024
MAX_ARRAYS = 32
MAX_RANK = 8
MAX_DIMENSION = 1_000_000
MAX_ELEMENTS = 16_000_000
MAX_INFO_ITEMS = 32
MAX_INFO_KEY_BYTES = 64
MAX_ERROR_TEXT_BYTES = 512
MAX_FRAME_BYTES = FRAME_PREFIX.size + MAX_HEADER_BYTES + MAX_PAYLOAD_BYTES

_SCHEMA_HASH_KEYS = frozenset({"action", "info", "observation"})
ZERO_SCHEMA_HASH = "0" * 64
ZERO_SCHEMA_HASHES = {key: ZERO_SCHEMA_HASH for key in _SCHEMA_HASH_KEYS}

# Restrict info to numeric scalars; it cannot carry arbitrary Python objects.
INFO_ALLOWLIST = frozenset(
    {
        "action_norm",
        "cost",
        "env_id",
        "episode_id",
        "is_first",
        "metric",
        "reward",
        "seed",
        "status_code",
        "step_index",
        "success",
        "terminated",
        "timeout",
        "truncated",
        "value",
        "worker",
        "worker_id",
    }
)

# Keep this flat HELLO field separate from demo metadata and closed to arbitrary
# task/config objects.
LANDSCAPING_CONTRACT_KEYS = frozenset(
    {
        "contract_id",
        "dreamer_sha",
        "io_schema_fingerprint",
        "mapper_profile_sha256",
        "normalization_policy",
        "projection_layout_fingerprint",
        "srb_sha",
        "target_manifest_sha256",
        "task_id",
        "target_identity_kind",
        "target_generator_version",
        "target_profile_sha256",
        "target_reset_layout_sha256",
    }
)
_LANDSCAPING_CONTRACT_NULLABLE_KEYS = frozenset(
    {
        "dreamer_sha",
        "io_schema_fingerprint",
        "mapper_profile_sha256",
        "projection_layout_fingerprint",
        "srb_sha",
        "target_manifest_sha256",
    }
)
_LANDSCAPING_CONTRACT_SHA256_KEYS = frozenset(
    {
        "io_schema_fingerprint",
        "mapper_profile_sha256",
        "projection_layout_fingerprint",
        "target_manifest_sha256",
        "target_profile_sha256",
        "target_reset_layout_sha256",
    }
)
_LANDSCAPING_CONTRACT_GIT_SHA_KEYS = frozenset({"dreamer_sha", "srb_sha"})
_GENERAL_MISSION_FAMILY_FIELDS = frozenset(
    {
        "target_identity_kind",
        "target_generator_version",
        "target_profile_sha256",
        "target_reset_layout_sha256",
    }
)
_LANDSCAPING_CONTRACT_REQUIRED_KEYS = (
    LANDSCAPING_CONTRACT_KEYS - _GENERAL_MISSION_FAMILY_FIELDS
)
MAX_LANDSCAPING_CONTRACT_BYTES = 4096


class DreamerProcessProtocolError(ValueError):
    """Raised when a protocol frame is not exactly valid."""


# Short alias used by callers that want to distinguish codec failures from
# transport/barrier failures without depending on the longer class name.
ProtocolCodecError = DreamerProcessProtocolError


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise DreamerProcessProtocolError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise DreamerProcessProtocolError(f"non-finite JSON constant {value!r}")


def _canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    try:
        return json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
    except (TypeError, ValueError, UnicodeEncodeError) as exc:
        raise DreamerProcessProtocolError("metadata is not canonical JSON") from exc


def _decode_json_metadata(data: bytes) -> dict[str, Any]:
    if len(data) > MAX_HEADER_BYTES:
        raise DreamerProcessProtocolError("metadata header exceeds the size limit")
    try:
        value = json.loads(
            data.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_json_constant,
        )
    except DreamerProcessProtocolError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
        raise DreamerProcessProtocolError("metadata is not valid JSON") from exc
    if not isinstance(value, dict):
        raise DreamerProcessProtocolError("metadata root must be a JSON object")
    if _canonical_json_bytes(value) != data:
        raise DreamerProcessProtocolError("metadata is not canonical JSON")
    return value


def _require_int(value: Any, *, name: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise DreamerProcessProtocolError(
            f"{name} must be an integer in [{minimum}, {maximum}]"
        )
    return value


def _require_real(value: Any, *, name: str) -> float:
    if type(value) not in (int, float) or isinstance(value, bool):
        raise DreamerProcessProtocolError(f"{name} must be a real number")
    result = float(value)
    if not math.isfinite(result):
        raise DreamerProcessProtocolError(f"{name} must be finite")
    return result


def _require_string(value: Any, *, name: str, max_bytes: int) -> str:
    if not isinstance(value, str):
        raise DreamerProcessProtocolError(f"{name} must be a string")
    if len(value.encode("utf-8")) > max_bytes:
        raise DreamerProcessProtocolError(f"{name} exceeds the size limit")
    return value


def _require_hex_digest(
    value: Any,
    *,
    name: str,
    length: int,
) -> str:
    text = _require_string(value, name=name, max_bytes=length)
    if len(text) != length or any(char not in "0123456789abcdef" for char in text):
        raise DreamerProcessProtocolError(
            f"{name} must be a lowercase {length * 4}-bit hex digest"
        )
    return text


def _validate_landscaping_contract(value: Any) -> dict[str, str | None]:
    """Validate the bounded, flat live landscaping metadata block."""

    if not isinstance(value, Mapping):
        raise DreamerProcessProtocolError("landscaping_contract must be a JSON object")
    keys = set(value)
    if any(type(key) is not str for key in keys):
        raise DreamerProcessProtocolError("landscaping_contract keys must be strings")
    unknown = keys - LANDSCAPING_CONTRACT_KEYS
    if unknown:
        raise DreamerProcessProtocolError(
            f"landscaping_contract has unknown keys: {sorted(unknown)}"
        )
    missing = _LANDSCAPING_CONTRACT_REQUIRED_KEYS - keys
    if missing:
        raise DreamerProcessProtocolError(
            f"landscaping_contract is missing required keys: {sorted(missing)}"
        )
    family_fields = keys & _GENERAL_MISSION_FAMILY_FIELDS
    task_id = value.get("task_id")
    is_general_task = isinstance(task_id, str) and task_id.rsplit("/", 1)[-1] == (
        "terrain_landscaping"
    )
    if family_fields and family_fields != _GENERAL_MISSION_FAMILY_FIELDS:
        raise DreamerProcessProtocolError(
            "landscaping_contract general mission family fields must be "
            "present together"
        )
    if is_general_task and not family_fields:
        raise DreamerProcessProtocolError(
            "general landscaping_contract is missing mission family fields"
        )
    if family_fields and not is_general_task:
        raise DreamerProcessProtocolError(
            "static landscaping_contract cannot declare general mission family fields"
        )
    if (
        family_fields
        and value.get("target_identity_kind") != "general_mission_family_v1"
    ):
        raise DreamerProcessProtocolError(
            "landscaping_contract has an unknown target identity family"
        )
    result: dict[str, str | None] = {}
    for key, item in value.items():
        field_name = f"landscaping_contract.{key}"
        if item is None and key in _LANDSCAPING_CONTRACT_NULLABLE_KEYS:
            result[key] = None
        elif key in _LANDSCAPING_CONTRACT_SHA256_KEYS:
            result[key] = _require_hex_digest(item, name=field_name, length=64)
        elif key in _LANDSCAPING_CONTRACT_GIT_SHA_KEYS:
            result[key] = _require_hex_digest(item, name=field_name, length=40)
        else:
            text = _require_string(item, name=field_name, max_bytes=1024)
            if not text:
                raise DreamerProcessProtocolError(f"{field_name} must not be empty")
            result[key] = text
    encoded = _canonical_json_bytes(result)
    if len(encoded) > MAX_LANDSCAPING_CONTRACT_BYTES:
        raise DreamerProcessProtocolError("landscaping_contract exceeds the size limit")
    return result


def validate_landscaping_contract(value: Mapping[str, Any]) -> dict[str, str | None]:
    """Validate and copy a caller-provided landscaping metadata block."""

    return _validate_landscaping_contract(value)


def _validate_pid(value: Any) -> None:
    if value is not None:
        _require_int(value, name="pid", minimum=1, maximum=2**63 - 1)


def _validate_schema_hashes(value: Any) -> dict[str, str]:
    if not isinstance(value, dict) or set(value) != _SCHEMA_HASH_KEYS:
        raise DreamerProcessProtocolError(
            "schema_hashes must contain exactly action, info, and observation"
        )
    result: dict[str, str] = {}
    for key, item in value.items():
        text = _require_string(item, name=f"schema_hashes.{key}", max_bytes=64)
        if len(text) != 64 or any(char not in "0123456789abcdef" for char in text):
            raise DreamerProcessProtocolError(
                f"schema_hashes.{key} must be a lowercase SHA-256 hex digest"
            )
        result[key] = text
    return result


def validate_schema_hashes(value: Mapping[str, str]) -> dict[str, str]:
    """Validate and copy a caller-provided schema-hash mapping."""

    return _validate_schema_hashes(dict(value))


def _validate_shape(value: Any) -> tuple[int, ...]:
    if not isinstance(value, (list, tuple)) or len(value) > MAX_RANK:
        raise DreamerProcessProtocolError("shape must be a bounded dimension list")
    result: list[int] = []
    elements = 1
    for index, dimension in enumerate(value):
        dimension = _require_int(
            dimension,
            name=f"shape[{index}]",
            minimum=0,
            maximum=MAX_DIMENSION,
        )
        result.append(dimension)
        elements *= dimension
        if elements > MAX_ELEMENTS:
            raise DreamerProcessProtocolError("array has too many elements")
    return tuple(result)


def _numeric_dtype(value: Any) -> str:
    try:
        dtype = np.dtype(value)
    except (TypeError, ValueError) as exc:
        raise DreamerProcessProtocolError("dtype is not valid") from exc
    if dtype.hasobject or dtype.fields is not None or dtype.subdtype is not None:
        raise DreamerProcessProtocolError("object and structured dtypes are forbidden")
    if dtype.kind not in "biuf":
        raise DreamerProcessProtocolError(
            f"dtype kind {dtype.kind!r} is not an allowed numeric kind"
        )
    return dtype.str


@dataclass(frozen=True, slots=True)
class ArraySchema:
    """Canonical numeric ndarray shape and dtype, without payload bytes."""

    dtype: str
    shape: tuple[int, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "dtype", _numeric_dtype(self.dtype))
        object.__setattr__(self, "shape", _validate_shape(self.shape))

    @classmethod
    def from_array(cls, value: np.ndarray) -> "ArraySchema":
        if not isinstance(value, np.ndarray):
            raise DreamerProcessProtocolError("array value must be an ndarray")
        return cls(value.dtype.str, tuple(value.shape))

    @classmethod
    def from_metadata(cls, value: Any) -> "ArraySchema":
        if not isinstance(value, dict) or set(value) != {"dtype", "shape"}:
            raise DreamerProcessProtocolError(
                "array schema must contain exactly dtype and shape"
            )
        return cls(value["dtype"], tuple(value["shape"]))

    @property
    def nbytes(self) -> int:
        elements = 1
        for dimension in self.shape:
            elements *= dimension
        size = elements * np.dtype(self.dtype).itemsize
        if size > MAX_PAYLOAD_BYTES:
            raise DreamerProcessProtocolError("array exceeds the payload size limit")
        return size

    def to_metadata(self) -> dict[str, Any]:
        return {"dtype": self.dtype, "shape": list(self.shape)}


def schema_hash(schema: ArraySchema) -> str:
    """Return the stable hash used for one numeric array schema."""

    return hashlib.sha256(_canonical_json_bytes(schema.to_metadata())).hexdigest()


def schema_hashes_for(
    action: str,
    observation: str,
    *,
    info_keys: Iterable[str] | None = None,
) -> dict[str, str]:
    """Build protocol hashes from advertised action/observation projections."""

    keys = INFO_ALLOWLIST if info_keys is None else frozenset(info_keys)
    if not keys <= INFO_ALLOWLIST:
        raise DreamerProcessProtocolError("info schema contains an unknown key")
    for name, value in (("action", action), ("observation", observation)):
        if (
            type(value) is not str
            or len(value) != 64
            or any(char not in "0123456789abcdef" for char in value)
        ):
            raise DreamerProcessProtocolError(
                f"{name} projection hash must be a lowercase SHA-256 digest"
            )
    info_meta = {"keys": sorted(keys)}
    return {
        "action": action,
        "info": hashlib.sha256(_canonical_json_bytes(info_meta)).hexdigest(),
        "observation": observation,
    }


def _validate_flags(value: Any, *, expected: set[str]) -> dict[str, bool]:
    if not isinstance(value, dict) or set(value) != expected:
        raise DreamerProcessProtocolError(
            f"flags must contain exactly {sorted(expected)}"
        )
    result: dict[str, bool] = {}
    for key, item in value.items():
        if type(item) is not bool:
            raise DreamerProcessProtocolError(f"flags.{key} must be boolean")
        result[key] = item
    return result


def _validate_info(value: Any) -> dict[str, bool | int | float]:
    if not isinstance(value, dict) or len(value) > MAX_INFO_ITEMS:
        raise DreamerProcessProtocolError("info must be a bounded JSON object")
    result: dict[str, bool | int | float] = {}
    for key, item in value.items():
        _require_string(key, name="info key", max_bytes=MAX_INFO_KEY_BYTES)
        if key not in INFO_ALLOWLIST:
            raise DreamerProcessProtocolError(f"info key {key!r} is not allowlisted")
        if type(item) is bool:
            result[key] = item
        elif type(item) is int:
            _require_int(item, name=f"info.{key}", minimum=-(2**63), maximum=2**63 - 1)
            result[key] = item
        elif type(item) is float and math.isfinite(item):
            result[key] = item
        else:
            raise DreamerProcessProtocolError(f"info.{key} must be finite numeric data")
    return result


def _validate_array_name(value: Any) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 64
        or not value.isascii()
        or any(
            char
            not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-"
            for char in value
        )
    ):
        raise DreamerProcessProtocolError("array name is not valid")
    return value


def _validate_array_descriptors(
    value: Any,
    *,
    expected_names: set[str],
    payload_len: int | None,
) -> None:
    if not isinstance(value, dict) or len(value) > MAX_ARRAYS:
        raise DreamerProcessProtocolError("arrays must be a bounded JSON object")
    names = {_validate_array_name(name) for name in value}
    if names != expected_names:
        raise DreamerProcessProtocolError(
            f"arrays must contain exactly {sorted(expected_names)}"
        )

    entries: list[tuple[int, int, ArraySchema, dict[str, Any]]] = []
    for name, descriptor in value.items():
        if not isinstance(descriptor, dict) or set(descriptor) != {
            "dtype",
            "nbytes",
            "offset",
            "shape",
        }:
            raise DreamerProcessProtocolError(
                f"array descriptor {name!r} has unknown or missing fields"
            )
        schema = ArraySchema.from_metadata(
            {"dtype": descriptor["dtype"], "shape": descriptor["shape"]}
        )
        offset = _require_int(
            descriptor["offset"],
            name=f"arrays.{name}.offset",
            minimum=0,
            maximum=MAX_PAYLOAD_BYTES,
        )
        nbytes = _require_int(
            descriptor["nbytes"],
            name=f"arrays.{name}.nbytes",
            minimum=0,
            maximum=MAX_PAYLOAD_BYTES,
        )
        if nbytes != schema.nbytes:
            raise DreamerProcessProtocolError(
                f"array descriptor {name!r} has an incorrect byte count"
            )
        entries.append((offset, nbytes, schema, descriptor))

    cursor = 0
    for offset, nbytes, _schema, _descriptor in sorted(
        entries, key=lambda item: item[0]
    ):
        if offset < cursor:
            raise DreamerProcessProtocolError("array payload descriptors overlap")
        if offset > cursor:
            raise DreamerProcessProtocolError("array payload descriptors have a gap")
        cursor += nbytes
        if cursor > MAX_PAYLOAD_BYTES:
            raise DreamerProcessProtocolError("array payload exceeds the size limit")
    if payload_len is not None:
        if payload_len > MAX_PAYLOAD_BYTES:
            raise DreamerProcessProtocolError("array payload exceeds the size limit")
        if cursor != payload_len:
            raise DreamerProcessProtocolError(
                "array payload has trailing bytes or uncovered descriptors"
            )


def _expected_fields(direction: str, kind: str) -> tuple[set[str], set[str]]:
    common = {
        "arrays",
        "direction",
        "episode_id",
        "flags",
        "kind",
        "pid",
        "schema_hashes",
        "seed",
        "sequence",
        "step_index",
        "version",
        "worker_id",
    }
    if direction == "request" and kind in {"HELLO", "RESET", "STEP", "CLOSE"}:
        return common, set()
    if direction == "response" and kind in {"HELLO", "CLOSED"}:
        return common | ({"projections"} if kind == "HELLO" else set()), (
            {"demo_projection", "landscaping_contract"} if kind == "HELLO" else set()
        )
    if direction == "response" and kind == "TRANSITION":
        return common | {"info", "reward", "step_delta"}, set()
    if direction == "response" and kind == "ERROR":
        return common | {"error"}, set()
    raise DreamerProcessProtocolError(
        f"unknown protocol direction/kind {direction!r}/{kind!r}"
    )


def _validate_wire_metadata(meta: dict[str, Any], *, payload_len: int) -> None:
    if meta.get("version") != PROTOCOL_VERSION:
        raise DreamerProcessProtocolError("unknown protocol version")
    direction = meta.get("direction")
    kind = meta.get("kind")
    if not isinstance(direction, str) or direction not in {"request", "response"}:
        raise DreamerProcessProtocolError("invalid protocol direction or kind")
    if not isinstance(kind, str):
        raise DreamerProcessProtocolError("invalid protocol direction or kind")
    required, optional = _expected_fields(direction, kind)
    if not set(meta) >= required or bool(set(meta) - required - optional):
        unknown = sorted(set(meta) - required - optional)
        missing = sorted(required - set(meta))
        raise DreamerProcessProtocolError(
            f"message schema mismatch; unknown={unknown}, missing={missing}"
        )

    _require_int(meta["worker_id"], name="worker_id", minimum=0, maximum=2**31 - 1)
    _validate_pid(meta["pid"])
    _require_int(meta["seed"], name="seed", minimum=-(2**63), maximum=2**63 - 1)
    _require_int(meta["sequence"], name="sequence", minimum=0, maximum=2**63 - 1)
    _require_int(meta["episode_id"], name="episode_id", minimum=-1, maximum=2**63 - 1)
    _require_int(meta["step_index"], name="step_index", minimum=0, maximum=2**63 - 1)
    _validate_schema_hashes(meta["schema_hashes"])

    flag_keys = {
        ("request", "HELLO"): {"initial"},
        ("request", "RESET"): {"reset"},
        ("request", "STEP"): {"reset"},
        ("request", "CLOSE"): {"graceful"},
        ("response", "HELLO"): {"ready"},
        ("response", "TRANSITION"): {"is_first", "terminated", "truncated"},
        ("response", "CLOSED"): {"clean"},
        ("response", "ERROR"): {"fatal"},
    }[(direction, kind)]
    _validate_flags(meta["flags"], expected=flag_keys)

    carries_arrays = (direction, kind) in {
        ("request", "STEP"),
        ("response", "TRANSITION"),
    }
    arrays = meta["arrays"]
    if carries_arrays and (not isinstance(arrays, dict) or not arrays):
        raise DreamerProcessProtocolError(
            f"{kind} must carry at least one numeric array"
        )
    expected_arrays = (
        set(arrays) if carries_arrays and isinstance(arrays, dict) else set()
    )
    _validate_array_descriptors(
        arrays, expected_names=expected_arrays, payload_len=payload_len
    )

    if kind == "HELLO" and direction == "response":
        projections = meta["projections"]
        if type(projections) is not dict or set(projections) != {
            "action",
            "observation",
        }:
            raise DreamerProcessProtocolError(
                "HELLO projections must contain exactly action and observation"
            )
        for name, value in projections.items():
            if type(value) is not dict:
                raise DreamerProcessProtocolError(
                    f"HELLO {name} projection metadata must be an object"
                )
        if "demo_projection" in meta and type(meta["demo_projection"]) is not dict:
            raise DreamerProcessProtocolError(
                "HELLO demo_projection metadata must be an object"
            )
        if "landscaping_contract" in meta:
            _validate_landscaping_contract(meta["landscaping_contract"])
    elif kind == "TRANSITION" and direction == "response":
        _require_real(meta["reward"], name="reward")
        _require_int(meta["step_delta"], name="step_delta", minimum=0, maximum=1)
        _validate_info(meta["info"])
        if meta["episode_id"] < 0 or meta["step_index"] < 0:
            raise DreamerProcessProtocolError(
                "TRANSITION episode_id and step_index must be known"
            )
    elif kind == "ERROR" and direction == "response":
        error = meta["error"]
        if not isinstance(error, dict) or set(error) != {"code", "message"}:
            raise DreamerProcessProtocolError(
                "ERROR must contain exactly code and message"
            )
        _require_string(error["code"], name="error.code", max_bytes=64)
        _require_string(
            error["message"], name="error.message", max_bytes=MAX_ERROR_TEXT_BYTES
        )


def _encode_arrays(value: Mapping[str, np.ndarray]) -> tuple[dict[str, Any], bytes]:
    if not isinstance(value, Mapping):
        raise DreamerProcessProtocolError("arrays must be a mapping")
    if len(value) > MAX_ARRAYS:
        raise DreamerProcessProtocolError("too many arrays")
    descriptors: dict[str, Any] = {}
    payload_parts: list[bytes] = []
    offset = 0
    for name in sorted(value):
        _validate_array_name(name)
        array = value[name]
        if not isinstance(array, np.ndarray):
            raise DreamerProcessProtocolError(f"array {name!r} must be a NumPy ndarray")
        schema = ArraySchema.from_array(array)
        # Enforce the cumulative limit before copying an oversized array.
        if offset + array.nbytes > MAX_PAYLOAD_BYTES:
            raise DreamerProcessProtocolError("array payload exceeds the size limit")
        contiguous = np.ascontiguousarray(array)
        raw = contiguous.tobytes(order="C")
        descriptors[name] = {
            "dtype": schema.dtype,
            "nbytes": len(raw),
            "offset": offset,
            "shape": list(schema.shape),
        }
        payload_parts.append(raw)
        offset += len(raw)
    return descriptors, b"".join(payload_parts)


def _decode_arrays(value: dict[str, Any], payload: bytes) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for name, descriptor in value.items():
        schema = ArraySchema.from_metadata(
            {"dtype": descriptor["dtype"], "shape": descriptor["shape"]}
        )
        offset = descriptor["offset"]
        nbytes = descriptor["nbytes"]
        view = memoryview(payload)[offset : offset + nbytes]
        try:
            array = np.frombuffer(view, dtype=np.dtype(schema.dtype), count=-1)
            array = array.reshape(schema.shape).copy()
        except (TypeError, ValueError) as exc:
            raise DreamerProcessProtocolError(
                f"array {name!r} payload cannot be decoded"
            ) from exc
        result[name] = array
    return result


def _copy_wire_arrays(
    value: Mapping[str, np.ndarray], *, name: str
) -> Mapping[str, np.ndarray]:
    if not isinstance(value, Mapping) or not 1 <= len(value) <= MAX_ARRAYS:
        raise DreamerProcessProtocolError(
            f"{name} must be a non-empty bounded array mapping"
        )
    result: dict[str, np.ndarray] = {}
    for wire_name, array in value.items():
        _validate_array_name(wire_name)
        if not isinstance(array, np.ndarray):
            raise DreamerProcessProtocolError(
                f"array {wire_name!r} must be a NumPy ndarray"
            )
        ArraySchema.from_array(array)
        copied = np.array(array, copy=True, order="C")
        copied.setflags(write=False)
        result[wire_name] = copied
    return MappingProxyType(result)


def _copy_projection_metadata(value: Mapping[str, Any], *, name: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise DreamerProcessProtocolError(f"{name} projection must be a JSON object")
    encoded = _canonical_json_bytes(value)
    decoded = json.loads(encoded.decode("ascii"))
    if decoded != value:
        raise DreamerProcessProtocolError(
            f"{name} projection must use exact JSON-compatible field types"
        )
    return decoded


@dataclass(frozen=True, slots=True)
class HelloRequest:
    worker_id: int
    seed: int
    schema_hashes: Mapping[str, str]
    pid: int | None = None
    sequence: int = 0
    episode_id: int = -1
    step_index: int = 0
    flags: Mapping[str, bool] = field(default_factory=lambda: {"initial": True})


@dataclass(frozen=True, slots=True)
class ResetRequest:
    worker_id: int
    seed: int
    schema_hashes: Mapping[str, str]
    pid: int | None = None
    sequence: int = 1
    episode_id: int = -1
    step_index: int = 0
    flags: Mapping[str, bool] = field(default_factory=lambda: {"reset": True})


@dataclass(frozen=True, slots=True)
class StepRequest:
    worker_id: int
    seed: int
    schema_hashes: Mapping[str, str]
    action: Mapping[str, np.ndarray]
    pid: int | None = None
    sequence: int = 1
    episode_id: int = 0
    step_index: int = 0
    flags: Mapping[str, bool] = field(default_factory=lambda: {"reset": False})

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "action", _copy_wire_arrays(self.action, name="STEP action")
        )


@dataclass(frozen=True, slots=True)
class CloseRequest:
    worker_id: int
    seed: int
    schema_hashes: Mapping[str, str]
    pid: int | None = None
    sequence: int = 1
    episode_id: int = -1
    step_index: int = 0
    flags: Mapping[str, bool] = field(default_factory=lambda: {"graceful": True})


@dataclass(frozen=True, slots=True)
class HelloResponse(ImmutableValueMixin):
    worker_id: int
    seed: int
    schema_hashes: Mapping[str, str]
    action_projection: Mapping[str, Any]
    observation_projection: Mapping[str, Any]
    pid: int | None = None
    sequence: int = 0
    episode_id: int = -1
    step_index: int = 0
    flags: Mapping[str, bool] = field(default_factory=lambda: {"ready": True})
    # Append optional metadata to preserve legacy positional arguments.
    demo_projection: Mapping[str, Any] | None = None
    landscaping_contract: Mapping[str, Any] | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "action_projection",
            _copy_projection_metadata(self.action_projection, name="action"),
        )
        object.__setattr__(
            self,
            "observation_projection",
            _copy_projection_metadata(self.observation_projection, name="observation"),
        )
        if self.demo_projection is not None:
            object.__setattr__(
                self,
                "demo_projection",
                _copy_projection_metadata(self.demo_projection, name="demo"),
            )
        if self.landscaping_contract is not None:
            object.__setattr__(
                self,
                "landscaping_contract",
                MappingProxyType(
                    _validate_landscaping_contract(self.landscaping_contract)
                ),
            )


@dataclass(frozen=True, slots=True)
class TransitionResponse(ImmutableValueMixin):
    worker_id: int
    seed: int
    schema_hashes: Mapping[str, str]
    observation: Mapping[str, np.ndarray]
    reward: float
    terminated: bool
    truncated: bool
    is_first: bool
    episode_id: int
    step_index: int
    step_delta: int
    pid: int | None = None
    sequence: int = 1
    info: Mapping[str, bool | int | float] = field(default_factory=dict)
    # Derived from the fields above; an explicit value must agree with them.
    flags: Mapping[str, bool] | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "observation",
            _copy_wire_arrays(self.observation, name="TRANSITION observation"),
        )
        # The wire carries termination only in ``flags``, so a disagreeing
        # duplicate would silently change on round-trip.
        flags = {
            "is_first": self.is_first,
            "terminated": self.terminated,
            "truncated": self.truncated,
        }
        if self.flags is not None and dict(self.flags) != flags:
            raise DreamerProcessProtocolError(
                "TRANSITION flags disagree with its termination fields"
            )
        object.__setattr__(self, "flags", flags)


@dataclass(frozen=True, slots=True)
class ClosedResponse:
    worker_id: int
    seed: int
    schema_hashes: Mapping[str, str]
    pid: int | None = None
    sequence: int = 1
    episode_id: int = -1
    step_index: int = 0
    flags: Mapping[str, bool] = field(default_factory=lambda: {"clean": True})


@dataclass(frozen=True, slots=True)
class ErrorResponse:
    worker_id: int
    seed: int
    schema_hashes: Mapping[str, str]
    code: str
    message: str
    pid: int | None = None
    sequence: int = 1
    episode_id: int = -1
    step_index: int = 0
    flags: Mapping[str, bool] = field(default_factory=lambda: {"fatal": True})


ProtocolMessage: TypeAlias = (
    HelloRequest
    | ResetRequest
    | StepRequest
    | CloseRequest
    | HelloResponse
    | TransitionResponse
    | ClosedResponse
    | ErrorResponse
)


def _common_meta(
    *,
    direction: str,
    kind: str,
    worker_id: int,
    pid: int | None,
    seed: int,
    sequence: int,
    episode_id: int,
    step_index: int,
    flags: Mapping[str, bool],
    schema_hashes: Mapping[str, str],
    arrays: dict[str, Any],
) -> dict[str, Any]:
    return {
        "arrays": arrays,
        "direction": direction,
        "episode_id": episode_id,
        "flags": dict(flags),
        "kind": kind,
        "pid": pid,
        "schema_hashes": dict(schema_hashes),
        "seed": seed,
        "sequence": sequence,
        "step_index": step_index,
        "version": PROTOCOL_VERSION,
        "worker_id": worker_id,
    }


def _message_to_wire(message: ProtocolMessage) -> tuple[dict[str, Any], bytes]:
    if isinstance(message, HelloRequest):
        return (
            _common_meta(
                direction="request",
                kind="HELLO",
                worker_id=message.worker_id,
                pid=message.pid,
                seed=message.seed,
                sequence=message.sequence,
                episode_id=message.episode_id,
                step_index=message.step_index,
                flags=message.flags,
                schema_hashes=message.schema_hashes,
                arrays={},
            ),
            b"",
        )
    if isinstance(message, ResetRequest):
        return (
            _common_meta(
                direction="request",
                kind="RESET",
                worker_id=message.worker_id,
                pid=message.pid,
                seed=message.seed,
                sequence=message.sequence,
                episode_id=message.episode_id,
                step_index=message.step_index,
                flags=message.flags,
                schema_hashes=message.schema_hashes,
                arrays={},
            ),
            b"",
        )
    if isinstance(message, StepRequest):
        arrays, payload = _encode_arrays(message.action)
        return (
            _common_meta(
                direction="request",
                kind="STEP",
                worker_id=message.worker_id,
                pid=message.pid,
                seed=message.seed,
                sequence=message.sequence,
                episode_id=message.episode_id,
                step_index=message.step_index,
                flags=message.flags,
                schema_hashes=message.schema_hashes,
                arrays=arrays,
            ),
            payload,
        )
    if isinstance(message, CloseRequest):
        return (
            _common_meta(
                direction="request",
                kind="CLOSE",
                worker_id=message.worker_id,
                pid=message.pid,
                seed=message.seed,
                sequence=message.sequence,
                episode_id=message.episode_id,
                step_index=message.step_index,
                flags=message.flags,
                schema_hashes=message.schema_hashes,
                arrays={},
            ),
            b"",
        )
    if isinstance(message, HelloResponse):
        meta = _common_meta(
            direction="response",
            kind="HELLO",
            worker_id=message.worker_id,
            pid=message.pid,
            seed=message.seed,
            sequence=message.sequence,
            episode_id=message.episode_id,
            step_index=message.step_index,
            flags=message.flags,
            schema_hashes=message.schema_hashes,
            arrays={},
        )
        meta["projections"] = {
            "action": dict(message.action_projection),
            "observation": dict(message.observation_projection),
        }
        if message.demo_projection is not None:
            meta["demo_projection"] = dict(message.demo_projection)
        if message.landscaping_contract is not None:
            meta["landscaping_contract"] = dict(message.landscaping_contract)
        return meta, b""
    if isinstance(message, TransitionResponse):
        arrays, payload = _encode_arrays(message.observation)
        meta = _common_meta(
            direction="response",
            kind="TRANSITION",
            worker_id=message.worker_id,
            pid=message.pid,
            seed=message.seed,
            sequence=message.sequence,
            episode_id=message.episode_id,
            step_index=message.step_index,
            flags=message.flags or {},
            schema_hashes=message.schema_hashes,
            arrays=arrays,
        )
        meta["info"] = dict(message.info)
        meta["reward"] = message.reward
        meta["step_delta"] = message.step_delta
        return meta, payload
    if isinstance(message, ClosedResponse):
        return (
            _common_meta(
                direction="response",
                kind="CLOSED",
                worker_id=message.worker_id,
                pid=message.pid,
                seed=message.seed,
                sequence=message.sequence,
                episode_id=message.episode_id,
                step_index=message.step_index,
                flags=message.flags,
                schema_hashes=message.schema_hashes,
                arrays={},
            ),
            b"",
        )
    if isinstance(message, ErrorResponse):
        meta = _common_meta(
            direction="response",
            kind="ERROR",
            worker_id=message.worker_id,
            pid=message.pid,
            seed=message.seed,
            sequence=message.sequence,
            episode_id=message.episode_id,
            step_index=message.step_index,
            flags=message.flags,
            schema_hashes=message.schema_hashes,
            arrays={},
        )
        meta["error"] = {"code": message.code, "message": message.message}
        return meta, b""
    raise DreamerProcessProtocolError(
        f"unsupported protocol message {type(message).__name__}"
    )


def encode_message(message: ProtocolMessage) -> bytes:
    """Encode one typed message into one ``send_bytes`` frame."""

    meta, payload = _message_to_wire(message)
    _validate_wire_metadata(meta, payload_len=len(payload))
    header = _canonical_json_bytes(meta)
    if len(header) > MAX_HEADER_BYTES:
        raise DreamerProcessProtocolError("metadata header exceeds the size limit")
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise DreamerProcessProtocolError("array payload exceeds the size limit")
    return FRAME_PREFIX.pack(FRAME_MAGIC, len(header), len(payload)) + header + payload


def _wire_to_message(
    meta: dict[str, Any], arrays: dict[str, np.ndarray]
) -> ProtocolMessage:
    common = {
        "worker_id": meta["worker_id"],
        "seed": meta["seed"],
        "schema_hashes": dict(meta["schema_hashes"]),
        "pid": meta["pid"],
        "sequence": meta["sequence"],
        "episode_id": meta["episode_id"],
        "step_index": meta["step_index"],
        "flags": dict(meta["flags"]),
    }
    direction = meta["direction"]
    kind = meta["kind"]
    if direction == "request" and kind == "HELLO":
        return HelloRequest(**common)
    if direction == "request" and kind == "RESET":
        return ResetRequest(**common)
    if direction == "request" and kind == "STEP":
        return StepRequest(action=arrays, **common)
    if direction == "request" and kind == "CLOSE":
        return CloseRequest(**common)
    if direction == "response" and kind == "HELLO":
        projections = meta["projections"]
        return HelloResponse(
            action_projection=projections["action"],
            observation_projection=projections["observation"],
            demo_projection=meta.get("demo_projection"),
            landscaping_contract=meta.get("landscaping_contract"),
            **common,
        )
    if direction == "response" and kind == "TRANSITION":
        flags = meta["flags"]
        return TransitionResponse(
            observation=arrays,
            reward=float(meta["reward"]),
            terminated=flags["terminated"],
            truncated=flags["truncated"],
            is_first=flags["is_first"],
            step_delta=meta["step_delta"],
            info=dict(meta["info"]),
            **common,
        )
    if direction == "response" and kind == "CLOSED":
        return ClosedResponse(**common)
    if direction == "response" and kind == "ERROR":
        error = meta["error"]
        return ErrorResponse(code=error["code"], message=error["message"], **common)
    raise DreamerProcessProtocolError("message schema dispatch failed")


def decode_message(frame: bytes | bytearray | memoryview) -> ProtocolMessage:
    """Decode one complete frame and copy every ndarray out of its buffer."""

    raw = bytes(frame)
    if len(raw) < FRAME_PREFIX.size:
        raise DreamerProcessProtocolError("frame is shorter than its prefix")
    if len(raw) > MAX_FRAME_BYTES:
        raise DreamerProcessProtocolError("frame exceeds the size limit")
    magic, header_len, payload_len = FRAME_PREFIX.unpack_from(raw)
    if magic != FRAME_MAGIC:
        raise DreamerProcessProtocolError("unknown frame magic")
    if header_len > MAX_HEADER_BYTES:
        raise DreamerProcessProtocolError("metadata header exceeds the size limit")
    if payload_len > MAX_PAYLOAD_BYTES:
        raise DreamerProcessProtocolError("array payload exceeds the size limit")
    expected_len = FRAME_PREFIX.size + header_len + payload_len
    if len(raw) != expected_len:
        raise DreamerProcessProtocolError("frame has truncated or trailing bytes")
    header_start = FRAME_PREFIX.size
    header_end = header_start + header_len
    meta = _decode_json_metadata(raw[header_start:header_end])
    _validate_wire_metadata(meta, payload_len=payload_len)
    arrays = _decode_arrays(meta["arrays"], raw[header_end:])
    return _wire_to_message(meta, arrays)


def send_message(connection: Any, message: ProtocolMessage) -> None:
    """Send exactly one encoded frame through a multiprocessing connection."""

    connection.send_bytes(encode_message(message))


def recv_message(connection: Any) -> ProtocolMessage:
    """Receive exactly one encoded frame without enabling pickle fallback."""

    try:
        frame = connection.recv_bytes(MAX_FRAME_BYTES)
    except OSError as exc:
        raise DreamerProcessProtocolError(
            "connection refused a bounded protocol frame"
        ) from exc
    return decode_message(frame)


__all__ = [
    "ArraySchema",
    "CloseRequest",
    "ClosedResponse",
    "DreamerProcessProtocolError",
    "ErrorResponse",
    "FRAME_MAGIC",
    "FRAME_PREFIX",
    "HelloRequest",
    "HelloResponse",
    "INFO_ALLOWLIST",
    "LANDSCAPING_CONTRACT_KEYS",
    "MAX_LANDSCAPING_CONTRACT_BYTES",
    "MAX_FRAME_BYTES",
    "MAX_HEADER_BYTES",
    "MAX_PAYLOAD_BYTES",
    "ProtocolCodecError",
    "ProtocolMessage",
    "PROTOCOL_NAME",
    "PROTOCOL_VERSION",
    "ResetRequest",
    "StepRequest",
    "TransitionResponse",
    "ZERO_SCHEMA_HASH",
    "ZERO_SCHEMA_HASHES",
    "decode_message",
    "encode_message",
    "recv_message",
    "schema_hash",
    "schema_hashes_for",
    "send_message",
    "validate_landscaping_contract",
    "validate_schema_hashes",
]
