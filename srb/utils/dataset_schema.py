"""Pure dataset schema enums shared across data and experiment contracts."""

from __future__ import annotations

import math
from collections.abc import Mapping
from enum import Enum
from numbers import Real
from typing import cast

from srb.utils.json_types import (
    FrozenJSONValue,
    JSONValue,
    thaw_json_value,
    validate_json_object,
)

DEMO_IO_CONTRACT_VERSION = "srb.demo_io.v1"


class _StrEnum(str, Enum):
    def __str__(self) -> str:
        return cast(str, self.value)


class DatasetSplit(_StrEnum):
    """Public SRB split names."""

    HUMAN = "human"
    POLICY = "policy"
    RANDOM = "random"
    ZERO = "zero"
    REAL = "real"


class DatasetController(_StrEnum):
    """Public labels for how a dataset was generated."""

    HUMAN = "human"
    POLICY = "policy"
    RANDOM = "random"
    ZERO = "zero"
    REAL = "real"


class StorageFormat(_StrEnum):
    """Known physical dataset layouts."""

    SRB_JSONL = "srb_jsonl"
    ROBOMIMIC_HDF5 = "robomimic_hdf5"
    LEROBOT = "lerobot"
    EVENT_HDF5 = "event_hdf5"


def validate_demo_io_contract(
    value: object,
    *,
    context: str = "metadata demo_io_contract",
    expected_task: str | None = None,
    required: bool = False,
) -> dict[str, JSONValue] | None:
    """Validate the producer-to-replay I/O contract.

    Legacy metadata may omit it; strict training can set ``required=True``.
    Unknown fields remain available for versioned additions.
    """

    if value is None:
        if required:
            raise ValueError(f"{context} is required for strict replay")
        return None
    try:
        # Thaw immutable containers; validation still rejects non-JSON input.
        contract: dict[str, JSONValue] = validate_json_object(
            thaw_json_value(cast(JSONValue | FrozenJSONValue, value)), context=context
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(str(exc)) from exc

    version = contract.get("version")
    if version != DEMO_IO_CONTRACT_VERSION:
        raise ValueError(
            f"{context}.version must be {DEMO_IO_CONTRACT_VERSION!r}, got {version!r}"
        )

    if expected_task is not None and "task" in contract:
        task = contract["task"]
        if not isinstance(task, str) or not task:
            raise ValueError(f"{context}.task must be a non-empty string")
        if task != expected_task:
            raise ValueError(
                f"{context}.task={task!r} does not match manifest task={expected_task!r}"
            )

    observation = _contract_object(contract, "observation", context=context)
    _validate_observation_contract(observation, context=f"{context}.observation")

    action = _contract_object(contract, "action", context=context)
    _validate_action_contract(action, context=f"{context}.action")

    return contract


def _contract_object(
    contract: Mapping[str, JSONValue],
    key: str,
    *,
    context: str,
) -> dict[str, JSONValue]:
    value = contract.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{context}.{key} must be an object")
    # Copy so callers cannot retain the producer's mutable mapping.
    return dict(value)


def _nonempty_string(value: object, *, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{context} must be a non-empty string")
    return value


def _string_list(
    value: object, *, context: str, allow_empty: bool = False
) -> list[str]:
    if not isinstance(value, list) or (not allow_empty and not value):
        raise ValueError(f"{context} must be a list of strings")
    if any(not isinstance(item, str) or not item for item in value):
        raise ValueError(f"{context} must be a list of non-empty strings")
    return cast(list[str], value)


def _positive_shape(value: object, *, context: str) -> int:
    # A scalar Box has shape (), represented by [] with one element.
    if not isinstance(value, list):
        raise ValueError(f"{context} must be a list of positive integers")
    result = 1
    for item in value:
        if not isinstance(item, int) or isinstance(item, bool) or item <= 0:
            raise ValueError(f"{context} must be a list of positive integers")
        result *= item
    return result


def _finite_number_list(value: object, *, context: str) -> list[float]:
    if not isinstance(value, list):
        raise ValueError(f"{context} must be a list of finite numbers")
    if any(
        isinstance(item, bool)
        or not isinstance(item, Real)
        or not math.isfinite(float(item))
        for item in value
    ):
        raise ValueError(f"{context} must be a list of finite numbers")
    return [float(item) for item in value]


def _validate_bounds(value: Mapping[str, JSONValue], *, dim: int, context: str) -> None:
    bounds: dict[str, list[float]] = {}
    for name in ("low", "high"):
        if name in value:
            bounds[name] = _finite_number_list(value[name], context=f"{context}.{name}")
            if len(bounds[name]) != dim:
                raise ValueError(f"{context}.{name} length must equal dimension {dim}")
    if bounds and len(bounds) != 2:
        raise ValueError(f"{context} bounds require both low and high")
    if bounds and any(
        low > high for low, high in zip(bounds["low"], bounds["high"], strict=True)
    ):
        raise ValueError(f"{context} bounds must satisfy low <= high")


def _validate_observation_contract(
    observation: Mapping[str, JSONValue],
    *,
    context: str,
) -> None:
    _nonempty_string(observation.get("wrapper"), context=f"{context}.wrapper")
    _nonempty_string(observation.get("obs_key"), context=f"{context}.obs_key")
    _nonempty_string(observation.get("projection"), context=f"{context}.projection")
    _nonempty_string(
        observation.get("root_observation_key"),
        context=f"{context}.root_observation_key",
    )
    root = observation.get("root")
    if not isinstance(root, bool):
        raise ValueError(f"{context}.root must be a boolean")
    keys = _string_list(observation.get("keys"), context=f"{context}.keys")
    if len(keys) != len(set(keys)):
        raise ValueError(f"{context}.keys must not contain duplicates")
    _string_list(
        observation.get("projection_path"),
        context=f"{context}.projection_path",
        allow_empty=True,
    )
    if type(observation.get("selection_present")) is not bool:
        raise ValueError(f"{context}.selection_present must be a boolean")
    terms = observation.get("terms")
    if not isinstance(terms, list) or not terms:
        raise ValueError(f"{context}.terms must be a non-empty list")
    for index, raw_term in enumerate(terms):
        term_context = f"{context}.terms[{index}]"
        if not isinstance(raw_term, Mapping):
            raise ValueError(f"{term_context} must be an object")
        _nonempty_string(raw_term.get("key"), context=f"{term_context}.key")
        _nonempty_string(
            raw_term.get("source_key"),
            context=f"{term_context}.source_key",
        )
        _nonempty_string(
            raw_term.get("output_key"),
            context=f"{term_context}.output_key",
        )
        shape_dim = _positive_shape(
            raw_term.get("shape"),
            context=f"{term_context}.shape",
        )
        dtype = raw_term.get("dtype")
        _nonempty_string(dtype, context=f"{term_context}.dtype")
        vector = raw_term.get("vector")
        if not isinstance(vector, bool):
            raise ValueError(f"{term_context}.vector must be a boolean")
        vector_slice = raw_term.get("vector_slice")
        if vector_slice is not None:
            if (
                not isinstance(vector_slice, list)
                or len(vector_slice) != 2
                or any(
                    not isinstance(item, int) or isinstance(item, bool) or item < 0
                    for item in vector_slice
                )
            ):
                raise ValueError(
                    f"{term_context}.vector_slice must span the declared shape"
                )
            start, end = cast(list[int], vector_slice)
            if end - start != shape_dim:
                raise ValueError(
                    f"{term_context}.vector_slice must span the declared shape"
                )
        shape = cast(list[int], raw_term.get("shape"))
        if "channels" in raw_term:
            _validate_visual_channels(
                raw_term["channels"],
                width=shape[-1] if shape else 1,
                context=f"{term_context}.channels",
            )
        if "depth_range_m" not in raw_term and any(
            isinstance(channel, Mapping)
            and channel.get("encoding") == "depth_linear_u8"
            for channel in cast(list[object], raw_term.get("channels") or [])
        ):
            raise ValueError(
                f"{term_context}.depth_range_m is required for depth_linear_u8 channels"
            )
        if "depth_range_m" in raw_term:
            depth_range = _finite_number_list(
                raw_term["depth_range_m"], context=f"{term_context}.depth_range_m"
            )
            if len(depth_range) != 2 or not depth_range[0] < depth_range[1]:
                raise ValueError(
                    f"{term_context}.depth_range_m must be [near, far] with near < far"
                )
        if "components" in raw_term:
            _validate_vector_components(
                raw_term["components"],
                width=shape_dim,
                context=f"{term_context}.components",
            )


VISUAL_CHANNEL_ENCODINGS = frozenset(
    (
        "rgb_u8",
        "depth_linear_u8",
        "normals_unit",
        "segmentation_rgba_u8",
        "segmentation_id",
    )
)


def _validate_contiguous_slices(
    value: object, *, width: int, context: str
) -> list[Mapping[str, JSONValue]]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{context} must be a non-empty list")
    expected_start = 0
    entries: list[Mapping[str, JSONValue]] = []
    for index, raw_entry in enumerate(value):
        entry_context = f"{context}[{index}]"
        if not isinstance(raw_entry, Mapping):
            raise ValueError(f"{entry_context} must be an object")
        start = raw_entry.get("start")
        end = raw_entry.get("end")
        if (
            not isinstance(start, int)
            or isinstance(start, bool)
            or not isinstance(end, int)
            or isinstance(end, bool)
            or start != expected_start
            or end <= start
        ):
            raise ValueError(f"{context} must tile the term without gaps")
        expected_start = end
        entries.append(raw_entry)
    if expected_start != width:
        raise ValueError(f"{context} must tile the term width {width}")
    return entries


def _validate_visual_channels(value: object, *, width: int, context: str) -> None:
    for index, entry in enumerate(
        _validate_contiguous_slices(value, width=width, context=context)
    ):
        _nonempty_string(
            entry.get("data_type"), context=f"{context}[{index}].data_type"
        )
        if entry.get("encoding") not in VISUAL_CHANNEL_ENCODINGS:
            raise ValueError(
                f"{context}[{index}].encoding must be one of "
                f"{sorted(VISUAL_CHANNEL_ENCODINGS)}"
            )


def _validate_vector_components(value: object, *, width: int, context: str) -> None:
    for index, entry in enumerate(
        _validate_contiguous_slices(value, width=width, context=context)
    ):
        entry_context = f"{context}[{index}]"
        _nonempty_string(entry.get("key"), context=f"{entry_context}.key")
        _nonempty_string(entry.get("dtype"), context=f"{entry_context}.dtype")
        size = _positive_shape(entry.get("shape"), context=f"{entry_context}.shape")
        if size != cast(int, entry["end"]) - cast(int, entry["start"]):
            raise ValueError(f"{entry_context} slice must span its shape")


def _validate_action_contract(
    action: Mapping[str, JSONValue],
    *,
    context: str,
) -> None:
    _nonempty_string(action.get("source"), context=f"{context}.source")
    schema = action.get("schema")
    if not isinstance(schema, list) or not schema:
        raise ValueError(f"{context}.schema must be a non-empty list")
    expected_start = 0
    for index, raw_term in enumerate(schema):
        term_context = f"{context}.schema[{index}]"
        if not isinstance(raw_term, Mapping):
            raise ValueError(f"{term_context} must be an object")
        _nonempty_string(raw_term.get("name"), context=f"{term_context}.name")
        _string_list(
            raw_term.get("path"), context=f"{term_context}.path", allow_empty=True
        )
        dim = raw_term.get("dim")
        if not isinstance(dim, int) or isinstance(dim, bool) or dim <= 0:
            raise ValueError(f"{term_context}.dim must be a positive integer")
        shape_dim = _positive_shape(
            raw_term.get("shape"), context=f"{term_context}.shape"
        )
        if shape_dim != dim:
            raise ValueError(
                f"{term_context}.shape product {shape_dim} does not match dim {dim}"
            )
        start = raw_term.get("start")
        end = raw_term.get("end")
        if (
            not isinstance(start, int)
            or isinstance(start, bool)
            or not isinstance(end, int)
            or isinstance(end, bool)
            or start != expected_start
            or end - start != dim
        ):
            raise ValueError(
                f"{term_context} must provide contiguous start/end slices from zero"
            )
        expected_start = end
        dtype = raw_term.get("dtype")
        if dtype is not None:
            _nonempty_string(dtype, context=f"{term_context}.dtype")
        _validate_bounds(raw_term, dim=dim, context=term_context)
        if "scale" in raw_term:
            scale = raw_term["scale"]
            if (
                isinstance(scale, bool)
                or not isinstance(scale, Real)
                or not math.isfinite(float(scale))
            ):
                raise ValueError(f"{term_context}.scale must be a finite number")

    action_dim = action.get("action_dim")
    if action_dim is not None and (
        not isinstance(action_dim, int)
        or isinstance(action_dim, bool)
        or action_dim != expected_start
    ):
        raise ValueError(
            f"{context}.action_dim must equal schema dimension {expected_start}"
        )
    _validate_bounds(action, dim=expected_start, context=context)
    normalized = action.get("normalized")
    if normalized is not None and not isinstance(normalized, bool):
        raise ValueError(f"{context}.normalized must be a boolean or null")
    scales = action.get("scales")
    if scales is not None:
        if not isinstance(scales, Mapping):
            raise ValueError(f"{context}.scales must be an object")
        for key, value in scales.items():
            if not isinstance(key, str) or not key:
                raise ValueError(f"{context}.scales keys must be non-empty strings")
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(float(value))
            ):
                raise ValueError(f"{context}.scales values must be finite numbers")


__all__ = [
    "DEMO_IO_CONTRACT_VERSION",
    "DatasetController",
    "DatasetSplit",
    "StorageFormat",
    "validate_demo_io_contract",
]
