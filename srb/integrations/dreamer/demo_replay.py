"""Bounded canonical demonstration rows for the Dreamer replay seam."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any

import numpy as np

from srb.datasets.metadata_validation import validate_demo_io_contract
from srb.datasets.transitions import (
    MAX_DEMO_TRANSITIONS,
    build_demo_source_binding,
    open_demo_transition_source,
)
from srb.utils.artifact_identity import slugify_underscore
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_IN_MEMORY_FILE_BYTES,
    ByteBudget,
    ResourceLimitError,
    checked_shape_nbytes,
    require_bounded_int,
)

DEMO_REPLAY_BINDING_VERSION = "srb.dreamer.demo_replay.v1"
_CALLBACK_METADATA_BYTES = 3 * np.dtype(np.int64).itemsize
_CONTROL_KEYS = ("reward", "is_first", "is_last", "is_terminal")
_FORBIDDEN_ROW_KEYS = frozenset(("latent", "reset", "stepid"))
_MAX_BINDING_BYTES = 1 * (1 << 20)
_MAX_BINDING_DEPTH = 32
_MAX_BINDING_ITEMS = 10_000
_MIN_BINDING_INT = -(2**63)
_MAX_BINDING_INT = 2**63 - 1


class DreamerDemoReplayError(ValueError):
    """A canonical demonstration cannot be admitted to Dreamer replay."""


def _readonly_array(value: np.ndarray[Any, Any]) -> np.ndarray[Any, Any]:
    """Copy one native value into storage whose write flag cannot be enabled."""

    return np.frombuffer(value.tobytes(order="C"), dtype=value.dtype).reshape(
        value.shape
    )


class _BindingJSONBudget:
    """Bound a binding before recursively copying or serializing it."""

    __slots__ = ("encoded_size", "items")

    def __init__(self) -> None:
        self.encoded_size = 0
        self.items = 0

    @property
    def remaining_items(self) -> int:
        return _MAX_BINDING_ITEMS - self.items

    @property
    def remaining_bytes(self) -> int:
        return _MAX_BINDING_BYTES - self.encoded_size

    def visit(self, depth: int, *, label: str) -> None:
        if depth > _MAX_BINDING_DEPTH:
            raise DreamerDemoReplayError(
                f"{label} exceeds maximum depth {_MAX_BINDING_DEPTH}"
            )
        self.items += 1
        if self.items > _MAX_BINDING_ITEMS:
            raise DreamerDemoReplayError(
                f"{label} exceeds maximum item count {_MAX_BINDING_ITEMS}"
            )

    def visit_key(self, *, label: str) -> None:
        self.items += 1
        if self.items > _MAX_BINDING_ITEMS:
            raise DreamerDemoReplayError(
                f"{label} exceeds maximum item count {_MAX_BINDING_ITEMS}"
            )

    def add_bytes(self, amount: int, *, label: str) -> None:
        self.encoded_size += amount
        if self.encoded_size > _MAX_BINDING_BYTES:
            raise ResourceLimitError(
                label,
                limit=_MAX_BINDING_BYTES,
                observed=self.encoded_size,
            )


def _binding_json_string_size(
    value: str,
    budget: _BindingJSONBudget,
    *,
    label: str,
) -> int:
    remaining = budget.remaining_bytes
    if len(value) + 2 > remaining:
        raise ResourceLimitError(
            label,
            limit=_MAX_BINDING_BYTES,
            observed=_MAX_BINDING_BYTES + 1,
        )
    size = 2
    for character in value:
        codepoint = ord(character)
        if codepoint < 0x20:
            size += 2 if codepoint in {8, 9, 10, 12, 13} else 6
        elif codepoint in {0x22, 0x5C}:
            size += 2
        elif codepoint < 0x7F:
            size += 1
        elif codepoint <= 0xFFFF:
            size += 6
        else:
            size += 12
        if size > remaining:
            raise ResourceLimitError(
                label,
                limit=_MAX_BINDING_BYTES,
                observed=_MAX_BINDING_BYTES + 1,
            )
    return size


def _strict_json(
    value: Any,
    *,
    label: str,
    depth: int = 0,
    budget: _BindingJSONBudget | None = None,
    allow_tuple: bool = False,
) -> Any:
    """Copy bounded JSON values while rejecting tuples and NumPy values."""

    if budget is None:  # pragma: no cover - only the canonical root uses this
        budget = _BindingJSONBudget()
    budget.visit(depth, label=label)
    value_type = type(value)
    if value is None:
        budget.add_bytes(4, label=label)
        return None
    if value_type is bool:
        budget.add_bytes(4 if value else 5, label=label)
        return value
    if value_type is int:
        if not _MIN_BINDING_INT <= value <= _MAX_BINDING_INT:
            raise DreamerDemoReplayError(f"{label} contains an out-of-range integer")
        budget.add_bytes(len(str(value)), label=label)
        return value
    if value_type is float:
        if not math.isfinite(value):
            raise DreamerDemoReplayError(f"{label} must contain finite numbers")
        encoded = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        budget.add_bytes(len(encoded), label=label)
        return value
    if value_type is str:
        budget.add_bytes(
            _binding_json_string_size(value, budget, label=label),
            label=label,
        )
        return value
    if value_type is list or value_type is tuple:
        if value_type is tuple and not allow_tuple:
            raise DreamerDemoReplayError(
                f"{label} contains a tuple or NumPy value; use JSON lists/scalars"
            )
        if len(value) > budget.remaining_items:
            raise DreamerDemoReplayError(
                f"{label} exceeds maximum item count {_MAX_BINDING_ITEMS}"
            )
        budget.add_bytes(1, label=label)
        result: list[Any] = []
        for index, item in enumerate(value):
            if index:
                budget.add_bytes(1, label=label)
            result.append(
                _strict_json(
                    item,
                    label=f"{label}[{index}]",
                    depth=depth + 1,
                    budget=budget,
                    allow_tuple=allow_tuple,
                )
            )
        budget.add_bytes(1, label=label)
        return result
    if isinstance(value, Mapping):
        budget.add_bytes(1, label=label)
        entry_count = len(value)
        if entry_count < 0 or entry_count > budget.remaining_items // 2:
            raise DreamerDemoReplayError(
                f"{label} exceeds maximum item count {_MAX_BINDING_ITEMS}"
            )
        result: dict[str, Any] = {}
        iterator = iter(value)
        for index in range(entry_count):
            if budget.items + 2 > _MAX_BINDING_ITEMS:
                raise DreamerDemoReplayError(
                    f"{label} exceeds maximum item count {_MAX_BINDING_ITEMS}"
                )
            try:
                key = next(iterator)
            except StopIteration as exc:
                raise DreamerDemoReplayError(
                    f"{label} mapping changed during traversal"
                ) from exc
            if type(key) is not str:
                raise DreamerDemoReplayError(f"{label} keys must be strings")
            if index:
                budget.add_bytes(1, label=label)
            budget.visit_key(label=label)
            budget.add_bytes(
                _binding_json_string_size(key, budget, label=label),
                label=label,
            )
            budget.add_bytes(1, label=label)
            result[key] = _strict_json(
                value[key],
                label=f"{label}.{key}",
                depth=depth + 1,
                budget=budget,
                allow_tuple=allow_tuple,
            )
        if len(result) != entry_count:
            raise DreamerDemoReplayError(f"{label} mapping changed during traversal")
        budget.add_bytes(1, label=label)
        return result
    if isinstance(value, (np.ndarray, np.generic)):
        raise DreamerDemoReplayError(
            f"{label} contains a tuple or NumPy value; use JSON lists/scalars"
        )
    raise DreamerDemoReplayError(
        f"{label} contains unsupported value {value_type.__name__}"
    )


def _canonical_json(
    value: Mapping[str, Any], *, label: str
) -> tuple[dict[str, Any], str]:
    budget = _BindingJSONBudget()
    copied = _strict_json(value, label=label, budget=budget)
    if not isinstance(copied, dict):  # pragma: no cover - mapping invariant
        raise DreamerDemoReplayError(f"{label} must be a JSON object")
    encoded = json.dumps(
        copied,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )
    size = len(encoded.encode("utf-8"))
    if size != budget.encoded_size:  # pragma: no cover - accounting invariant
        raise DreamerDemoReplayError(f"{label} JSON size accounting mismatch")
    return copied, encoded


def _config_lookup(config: Any, path: str) -> Any:
    """Read dotted mapping/attribute paths without coercing their values."""

    if isinstance(config, Mapping) and path in config:
        return config[path]
    current = config
    for component in path.split("."):
        if isinstance(current, Mapping):
            if component not in current:
                raise KeyError(path)
            current = current[component]
        else:
            try:
                current = getattr(current, component)
            except AttributeError as exc:
                raise KeyError(path) from exc
    return current


def _config_value(config: Any, *paths: str, default: Any = None) -> Any:
    for path in paths:
        try:
            return _config_lookup(config, path)
        except KeyError:
            continue
    return default


_OPERATIONAL_CONFIG_KEYS = frozenset(
    {
        "run.from_checkpoint",
        "run.steps",
        "run.log_every",
        "run.report_every",
        "run.save_every",
        "run.usage",
        "logdir",
    }
)


def _config_json_value(value: Any, *, label: str) -> Any:
    """Normalize config containers to JSON lists without coercing scalars."""

    return _strict_json(
        value,
        label=label,
        budget=_BindingJSONBudget(),
        allow_tuple=True,
    )


def _effective_config_binding(config: Any) -> dict[str, Any]:
    """Return all effective learner settings except operational run controls."""

    def is_operational(key: str) -> bool:
        return (
            key in _OPERATIONAL_CONFIG_KEYS
            or key.startswith("logger.")
            or key.startswith("run.log_")
            or key.startswith("run.report_")
            or key.startswith("run.save_")
            or key.startswith("run.usage.")
        )

    flat = getattr(config, "flat", None)
    if isinstance(flat, Mapping):
        raw_items = flat.items()
        traversal_budget = _BindingJSONBudget()
    else:
        flattened: dict[str, Any] = {}
        traversal_budget = _BindingJSONBudget()

        def visit(value: Any, prefix: str, *, depth: int) -> None:
            traversal_budget.visit(depth, label=prefix or "config")
            if depth > _MAX_BINDING_DEPTH:
                raise DreamerDemoReplayError(
                    "effective Dreamer config exceeds the binding depth limit"
                )
            if isinstance(value, Mapping):
                iterator = value.items()
            elif hasattr(value, "__dict__"):
                iterator = vars(value).items()
            else:
                if not prefix:
                    raise DreamerDemoReplayError(
                        "effective Dreamer config must expose mapping or public attributes"
                    )
                if not is_operational(prefix):
                    flattened[prefix] = _config_json_value(value, label=prefix)
                return
            for key, item in iterator:
                if type(key) is not str or not key or key.startswith("_"):
                    if type(key) is str and key.startswith("_"):
                        continue
                    raise DreamerDemoReplayError(
                        "effective Dreamer config keys must be strings"
                    )
                child = f"{prefix}.{key}" if prefix else key
                traversal_budget.visit_key(label=child)
                traversal_budget.add_bytes(
                    _binding_json_string_size(key, traversal_budget, label=child),
                    label=child,
                )
                if is_operational(child):
                    continue
                if isinstance(item, Mapping) or hasattr(item, "__dict__"):
                    visit(item, child, depth=depth + 1)
                else:
                    flattened[child] = _config_json_value(item, label=child)

        visit(config, "", depth=0)
        raw_items = flattened.items()

    result: dict[str, Any] = {}
    for key, value in raw_items:
        if type(key) is not str or not key:
            raise DreamerDemoReplayError(
                "effective Dreamer config keys must be strings"
            )
        if is_operational(key):
            continue
        traversal_budget.visit_key(label=f"config.{key}")
        traversal_budget.add_bytes(
            _binding_json_string_size(key, traversal_budget, label=f"config.{key}"),
            label=f"config.{key}",
        )
        result[key] = _config_json_value(value, label=f"config.{key}")
    return result


def _bounded_config_int(
    value: Any,
    *,
    resource: str,
    minimum: int = 1,
    maximum: int = MAX_ARRAY_ELEMENTS,
) -> int:
    # Dreamer's YAML defaults use values such as ``5e6``.  Accept only an
    # exactly integral finite float at this configuration boundary.
    if isinstance(value, (float, np.floating)):
        if not math.isfinite(float(value)) or float(value) != math.trunc(float(value)):
            raise TypeError(f"{resource} must be an integer")
        value = int(value)
    return require_bounded_int(
        value,
        resource=resource,
        minimum=minimum,
        maximum=maximum,
    )


def _checked_mul(
    left: int,
    right: int,
    *,
    resource: str,
    maximum: int = MAX_ARRAY_ELEMENTS,
) -> int:
    if left < 0 or right < 0:
        raise ValueError(f"{resource} factors must be non-negative")
    result = left * right
    return require_bounded_int(
        result,
        resource=resource,
        minimum=0,
        maximum=maximum,
    )


def _checked_add(
    left: int,
    right: int,
    *,
    resource: str,
    maximum: int = MAX_ARRAY_ELEMENTS,
) -> int:
    if left < 0 or right < 0:
        raise ValueError(f"{resource} terms must be non-negative")
    result = left + right
    return require_bounded_int(
        result,
        resource=resource,
        minimum=0,
        maximum=maximum,
    )


def _space_shape_dtype(
    space: Any, *, resource: str
) -> tuple[tuple[int, ...], np.dtype[Any]]:
    try:
        shape = tuple(space.shape)
        dtype = np.dtype(space.dtype)
    except (AttributeError, TypeError, ValueError) as exc:
        raise DreamerDemoReplayError(f"{resource} is not a native array space") from exc
    checked_shape_nbytes(
        shape,
        dtype,
        resource=resource,
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
        max_rank=8,
    )
    if dtype.kind not in "biuf":
        raise DreamerDemoReplayError(
            f"{resource} uses unsupported native dtype {dtype}"
        )
    return shape, dtype


def _space_mapping(env: Any, name: str) -> dict[str, Any]:
    value = getattr(env, name, None)
    if not isinstance(value, Mapping) or not value:
        raise DreamerDemoReplayError(
            f"Dreamer demo env {name} must be a non-empty mapping"
        )
    if len(value) > _MAX_BINDING_ITEMS:
        raise DreamerDemoReplayError(
            f"Dreamer demo env {name} has too many native fields"
        )
    result: dict[str, Any] = {}
    for key, space in value.items():
        if type(key) is not str or not key:
            raise DreamerDemoReplayError(
                f"Dreamer demo {name} keys must be non-empty strings"
            )
        if key in result:
            raise DreamerDemoReplayError(
                f"Dreamer demo {name} contains duplicate key {key!r}"
            )
        result[key] = space
    return result


def _native_leaf(
    value: Any,
    space: Any,
    *,
    resource: str,
) -> np.ndarray[Any, Any]:
    shape, dtype = _space_shape_dtype(space, resource=resource)
    try:
        raw = np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise DreamerDemoReplayError(f"{resource} is not an array") from exc
    if raw.dtype.kind not in "biuf":
        raise DreamerDemoReplayError(f"{resource} is not real numeric data")
    if raw.shape != shape:
        raise DreamerDemoReplayError(
            f"{resource} shape must be {shape}, got {raw.shape}"
        )
    if raw.dtype.kind == "f" and not np.all(np.isfinite(raw)):
        raise DreamerDemoReplayError(f"{resource} contains non-finite values")

    if dtype.kind == "b":
        if raw.dtype.kind != "b":
            raise DreamerDemoReplayError(f"{resource} must have bool dtype")
        converted = np.asarray(raw, dtype=dtype)
    elif dtype.kind == "f":
        try:
            converted = np.asarray(raw, dtype=dtype)
        except (TypeError, ValueError, OverflowError) as exc:
            raise DreamerDemoReplayError(
                f"{resource} cannot be cast to {dtype}"
            ) from exc
        if not np.all(np.isfinite(converted)):
            raise DreamerDemoReplayError(
                f"{resource} overflows finite native dtype {dtype}"
            )
    else:
        if raw.dtype.kind not in "iu" or raw.dtype != dtype:
            raise DreamerDemoReplayError(f"{resource} must have dtype {dtype}")
        converted = raw

    low = getattr(space, "low", None)
    high = getattr(space, "high", None)
    if low is not None or high is not None:
        try:
            low_array = np.asarray(low) if low is not None else None
            high_array = np.asarray(high) if high is not None else None
            if low_array is not None and np.any(
                np.isfinite(low_array) & (converted < low_array)
            ):
                raise DreamerDemoReplayError(f"{resource} is below native bounds")
            if high_array is not None and np.any(
                np.isfinite(high_array) & (converted > high_array)
            ):
                raise DreamerDemoReplayError(f"{resource} is above native bounds")
        except DreamerDemoReplayError:
            raise
        except (TypeError, ValueError):
            raise DreamerDemoReplayError(
                f"{resource} native bounds are malformed"
            ) from None
    return _readonly_array(np.asarray(converted, dtype=dtype, order="C"))


def _native_mapping(
    values: Any,
    spaces: Mapping[str, Any],
    *,
    resource: str,
) -> Mapping[str, np.ndarray[Any, Any]]:
    if not isinstance(values, Mapping):
        raise DreamerDemoReplayError(f"{resource} must be a mapping")
    if set(values) != set(spaces):
        raise DreamerDemoReplayError(
            f"{resource} keys differ: expected {tuple(spaces)}, got {tuple(values)}"
        )
    result = {
        key: _native_leaf(values[key], space, resource=f"{resource}.{key}")
        for key, space in spaces.items()
    }
    return MappingProxyType(result)


def _charge_mapping(
    mapping: Mapping[str, np.ndarray[Any, Any]],
    budget: ByteBudget,
    *,
    resource: str,
) -> int:
    total = 0
    for key, value in mapping.items():
        nbytes = checked_shape_nbytes(
            value.shape,
            value.dtype,
            resource=f"{resource}.{key}",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
            max_rank=8,
        )
        budget.consume(nbytes)
        total += nbytes
    return total


def _build_native_row(
    observation: Mapping[str, np.ndarray[Any, Any]],
    action: Mapping[str, np.ndarray[Any, Any]],
    controls: Mapping[str, Any],
    control_spaces: Mapping[str, Any],
    *,
    staging_budget: ByteBudget,
    resource: str,
) -> Mapping[str, Any]:
    frozen_controls = _native_mapping(
        controls,
        control_spaces,
        resource=f"{resource} controls",
    )
    row_data: dict[str, Any] = {}
    for mapping in (observation, action, frozen_controls):
        if set(row_data) & set(mapping):
            raise DreamerDemoReplayError("Dreamer demo native row fields collide")
        row_data.update(mapping)
    if set(row_data) & _FORBIDDEN_ROW_KEYS or any(
        key.startswith("srb_") or key.startswith("log/") for key in row_data
    ):
        raise DreamerDemoReplayError(
            "Dreamer demo native row contains a forbidden field"
        )
    frozen_row = MappingProxyType(row_data)
    _charge_mapping(
        observation,
        staging_budget,
        resource=f"{resource} observation",
    )
    _charge_mapping(
        action,
        staging_budget,
        resource=f"{resource} action",
    )
    _charge_mapping(
        frozen_controls,
        staging_budget,
        resource=f"{resource} controls",
    )
    return frozen_row


def _mapping_equal(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    if set(left) != set(right):
        return False
    for key in left:
        lvalue = np.asarray(left[key])
        rvalue = np.asarray(right[key])
        if lvalue.shape != rvalue.shape or lvalue.dtype != rvalue.dtype:
            return False
        if not np.array_equal(lvalue, rvalue):
            return False
    return True


def _find_learner_projection(env: Any) -> Any:
    current = env
    seen: set[int] = set()
    for _ in range(16):
        if id(current) in seen:
            break
        seen.add(id(current))
        for name in ("_learner_projection", "learner_projection"):
            try:
                projection = getattr(current, name, None)
            except (AttributeError, ValueError):
                projection = None
            if projection is not None:
                return projection
        try:
            child = getattr(current, "env", None)
        except (AttributeError, ValueError):
            child = None
        if child is None or child is current:
            break
        current = child
    raise DreamerDemoReplayError(
        "Dreamer demo env does not expose its live learner projection"
    )


def _find_demo_projection(env: Any, *, contract: Mapping[str, Any]) -> Any:
    """Use an adapter already negotiated by the final env when available."""

    from srb.integrations.dreamer.demo_projection import DreamerDemoProjection

    current = env
    seen: set[int] = set()
    for _ in range(16):
        if id(current) in seen:
            break
        seen.add(id(current))
        try:
            candidate = getattr(current, "demo_projection", None)
        except (AttributeError, ValueError):
            candidate = None
        if candidate is not None:
            if all(
                callable(getattr(candidate, name, None))
                for name in (
                    "project_observation",
                    "project_action",
                )
            ):
                return candidate
            if isinstance(candidate, Mapping):
                return DreamerDemoProjection.from_metadata(candidate)
            raise DreamerDemoReplayError(
                "Dreamer demo env exposes an invalid live projection adapter"
            )
        try:
            child = getattr(current, "env", None)
        except (AttributeError, ValueError):
            child = None
        if child is None or child is current:
            break
        current = child

    learner_projection = _find_learner_projection(env)
    return DreamerDemoProjection.from_live(
        env,
        learner_projection=learner_projection,
        contract=contract,
    )


def _space_descriptors(spaces: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for key, space in spaces.items():
        shape, dtype = _space_shape_dtype(space, resource=f"native space {key!r}")
        descriptor: dict[str, Any] = {
            "shape": list(shape),
            "dtype": str(dtype),
            "nbytes": checked_shape_nbytes(
                shape,
                dtype,
                resource=f"native space {key!r}",
                max_elements=MAX_ARRAY_ELEMENTS,
                max_bytes=MAX_ARRAY_BYTES,
                max_rank=8,
            ),
        }
        low = getattr(space, "low", None)
        high = getattr(space, "high", None)
        if low is not None and high is not None:
            low_array = np.asarray(low)
            high_array = np.asarray(high)
            if low_array.shape != shape or high_array.shape != shape:
                raise DreamerDemoReplayError(
                    f"native space {key!r} bounds do not match its shape"
                )
            if np.all(np.isfinite(low_array)) and np.all(np.isfinite(high_array)):
                descriptor["low"] = low_array.tolist()
                descriptor["high"] = high_array.tolist()
        result[key] = descriptor
    return result


def _validate_action_contract(contract: Mapping[str, Any]) -> int:
    action = contract.get("action")
    if not isinstance(action, Mapping):
        raise DreamerDemoReplayError("canonical demo contract has no action mapping")
    if action.get("normalized") is not True:
        raise DreamerDemoReplayError(
            "Dreamer demonstrations require normalized action coordinates"
        )
    action_dim = action.get("action_dim")
    if type(action_dim) is not int or not 1 <= action_dim <= MAX_ARRAY_ELEMENTS:
        raise DreamerDemoReplayError("canonical demo action_dim is invalid")
    low = action.get("low")
    high = action.get("high")
    try:
        low_array = np.asarray(low, dtype=np.float64)
        high_array = np.asarray(high, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise DreamerDemoReplayError(
            "canonical demo action bounds are not numeric"
        ) from exc
    if (
        low_array.shape != (action_dim,)
        or high_array.shape != (action_dim,)
        or not np.all(np.isfinite(low_array))
        or not np.all(np.isfinite(high_array))
        or not np.array_equal(low_array, -np.ones(action_dim))
        or not np.array_equal(high_array, np.ones(action_dim))
    ):
        raise DreamerDemoReplayError(
            "Dreamer demonstrations require exact finite [-1, 1] action bounds"
        )
    schema = action.get("schema")
    if not isinstance(schema, (list, tuple)) or not schema:
        raise DreamerDemoReplayError("canonical demo action schema is missing")
    for index, term in enumerate(schema):
        if not isinstance(term, Mapping):
            raise DreamerDemoReplayError(
                f"canonical demo action term {index} is invalid"
            )
        dtype = term.get("dtype")
        if type(dtype) is not str or np.dtype(dtype).kind != "f":
            raise DreamerDemoReplayError(
                "Dreamer demonstrations require floating action terms"
            )
    return action_dim


def _validate_action_value(
    action: Any, *, action_dim: int, index: int
) -> np.ndarray[Any, Any]:
    if not isinstance(action, np.ndarray) or action.dtype != np.dtype(np.float32):
        raise DreamerDemoReplayError(
            f"canonical demo transition {index} action must be float32"
        )
    if action.shape != (action_dim,):
        raise DreamerDemoReplayError(
            f"canonical demo transition {index} action shape must be {(action_dim,)}"
        )
    if not np.all(np.isfinite(action)):
        raise DreamerDemoReplayError(
            f"canonical demo transition {index} action contains non-finite values"
        )
    if np.any(action < -1.0) or np.any(action > 1.0):
        raise DreamerDemoReplayError(
            f"canonical demo transition {index} action is outside [-1, 1]"
        )
    return action


def _group_episodes(transitions: Sequence[Any]) -> list[list[Any]]:
    episodes: list[list[Any]] = []
    current: list[Any] = []
    current_identity: tuple[int, str] | None = None
    for transition in transitions:
        identity = (transition.episode_index, transition.episode_key)
        if current_identity != identity:
            if current:
                episodes.append(current)
            current = []
            current_identity = identity
        current.append(transition)
    if current:
        episodes.append(current)
    return episodes


@dataclass(frozen=True, slots=True)
class DreamerDemoReplayPayload:
    """Fully staged, immutable native Dreamer replay rows."""

    rows: tuple[Mapping[str, Any], ...]
    dataset_fingerprint: str
    dataset_path: str
    transition_count: int
    capacity: int
    chunksize: int
    replay_length: int
    worker_count: int
    batch_size: int
    batch_length: int
    replay_context: int
    consec_train: int
    replicas: int
    row_bytes: int
    staged_bytes: int
    admission_bytes: int
    _binding_json: str = field(repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.rows, tuple):
            raise TypeError("Dreamer demo payload rows must be a tuple")
        if not isinstance(self.dataset_path, str) or not self.dataset_path:
            raise TypeError("Dreamer demo payload dataset_path must be a string")
        if (
            type(self.dataset_fingerprint) is not str
            or len(self.dataset_fingerprint) != 64
            or any(char not in "0123456789abcdef" for char in self.dataset_fingerprint)
        ):
            raise ValueError("Dreamer demo payload fingerprint is invalid")
        for key in (
            "transition_count",
            "capacity",
            "chunksize",
            "replay_length",
            "worker_count",
            "batch_size",
            "batch_length",
            "replay_context",
            "consec_train",
            "replicas",
            "row_bytes",
            "staged_bytes",
            "admission_bytes",
        ):
            value = getattr(self, key)
            if type(value) is not int or value < 0:
                raise TypeError(f"Dreamer demo payload {key} must be a Python integer")
        if self.transition_count <= 0 or len(self.rows) <= 0:
            raise ValueError("Dreamer demo payload cannot be empty")
        if type(self._binding_json) is not str:
            raise TypeError("Dreamer demo payload binding must be canonical JSON")
        try:
            parsed = json.loads(self._binding_json)
        except (
            TypeError,
            json.JSONDecodeError,
        ) as exc:  # pragma: no cover - builder invariant
            raise ValueError("Dreamer demo payload binding is invalid JSON") from exc
        if not isinstance(parsed, dict):  # pragma: no cover - builder invariant
            raise ValueError("Dreamer demo payload binding must be a JSON object")

    @property
    def native_row_count(self) -> int:
        return len(self.rows)

    @property
    def binding(self) -> dict[str, Any]:
        """Return a detached JSON object for checkpoint/metadata consumers."""

        return json.loads(self._binding_json)


def build_demo_replay_payload(
    dataset_path: str | Path,
    wrapped_env: Any,
    max_transitions: int,
    *,
    config: Any,
    task_id: str,
) -> DreamerDemoReplayPayload:
    """Read a complete bounded source and stage ``L + 1`` rows per episode."""

    if type(task_id) is not str or not task_id:
        raise DreamerDemoReplayError("Dreamer demo task_id must be a non-empty string")
    try:
        requested_task = slugify_underscore(task_id)
    except (TypeError, ValueError) as exc:
        raise DreamerDemoReplayError("Dreamer demo task_id is not canonical") from exc
    limit = require_bounded_int(
        max_transitions,
        resource="Dreamer demo transitions",
        minimum=1,
        maximum=MAX_DEMO_TRANSITIONS,
    )

    batch_size = _bounded_config_int(
        _config_value(config, "batch_size"), resource="Dreamer demo batch_size"
    )
    batch_length = _bounded_config_int(
        _config_value(config, "batch_length"), resource="Dreamer demo batch_length"
    )
    replay_context = _bounded_config_int(
        _config_value(config, "replay_context"),
        resource="Dreamer demo replay_context",
    )
    consec_train = _bounded_config_int(
        _config_value(config, "consec_train"), resource="Dreamer demo consec_train"
    )
    capacity = _bounded_config_int(
        _config_value(config, "replay.size", "replay_size"),
        resource="Dreamer demo capacity",
    )
    chunksize = _bounded_config_int(
        _config_value(config, "replay.chunksize", "replay_chunksize"),
        resource="Dreamer demo chunksize",
    )
    replicas = _bounded_config_int(
        _config_value(config, "replicas", default=1), resource="Dreamer demo replicas"
    )
    if replicas != 1:
        raise DreamerDemoReplayError(
            "Dreamer demonstration replay requires replicas=1 for a compatible transaction"
        )
    configured_envs = _config_value(config, "run.envs", "envs")
    if configured_envs is None:
        configured_envs = getattr(wrapped_env, "num_envs", None)
    if configured_envs is None:
        try:
            configured_envs = len(wrapped_env)
        except (TypeError, AttributeError):
            configured_envs = 1
    environment_count = _bounded_config_int(
        configured_envs, resource="Dreamer demo environment count"
    )
    worker_count = _checked_add(
        environment_count,
        1,
        resource="Dreamer demo worker count",
        maximum=MAX_ARRAY_ELEMENTS,
    )
    replay_length = _checked_mul(
        consec_train,
        batch_length,
        resource="Dreamer demo sequence length",
    )
    replay_length = _checked_add(
        replay_length,
        replay_context,
        resource="Dreamer demo sequence length",
        maximum=MAX_ARRAY_ELEMENTS,
    )
    window_rows = _checked_mul(
        batch_size,
        replay_length,
        resource="Dreamer demo replay window rows",
    )
    if capacity < window_rows:
        raise DreamerDemoReplayError(
            f"Dreamer demo replay capacity {capacity} is smaller than one batch window {window_rows}"
        )

    obs_spaces = _space_mapping(wrapped_env, "obs_space")
    act_spaces = _space_mapping(wrapped_env, "act_space")
    if set(obs_spaces) & set(act_spaces):
        raise DreamerDemoReplayError("Dreamer demo observation/action spaces collide")
    if not {"reward", "is_first", "is_last", "is_terminal"}.issubset(obs_spaces):
        raise DreamerDemoReplayError(
            "Dreamer demo env is missing one or more required control observation spaces"
        )
    control_spaces = {key: obs_spaces[key] for key in _CONTROL_KEYS}
    for key, space in control_spaces.items():
        shape, dtype = _space_shape_dtype(
            space, resource=f"native control space {key!r}"
        )
        expected_kind = "f" if key == "reward" else "b"
        if shape != () or dtype.kind != expected_kind:
            raise DreamerDemoReplayError(
                f"native control space {key!r} must be scalar {expected_kind} data"
            )
    live_obs_spaces = {
        key: space
        for key, space in obs_spaces.items()
        if not key.startswith("log/") and key not in _CONTROL_KEYS
    }
    live_act_spaces = {
        key: space for key, space in act_spaces.items() if key != "reset"
    }
    if not live_obs_spaces or not live_act_spaces:
        raise DreamerDemoReplayError(
            "Dreamer demo env has no learner observation/action leaves"
        )
    for key, space in live_act_spaces.items():
        shape, dtype = _space_shape_dtype(
            space, resource=f"native action space {key!r}"
        )
        if bool(getattr(space, "discrete", False)) or dtype.kind != "f":
            raise DreamerDemoReplayError(
                f"Dreamer demonstrations do not support discrete/non-floating action {key!r}"
            )
        low = getattr(space, "low", None)
        high = getattr(space, "high", None)
        if (
            low is None
            or high is None
            or not np.all(np.isfinite(low))
            or not np.all(np.isfinite(high))
        ):
            raise DreamerDemoReplayError(
                f"native action space {key!r} must have finite bounds"
            )
        try:
            low_array = np.asarray(low, dtype=np.float64)
            high_array = np.asarray(high, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise DreamerDemoReplayError(
                f"native action space {key!r} bounds are malformed"
            ) from exc
        if (
            low_array.shape != shape
            or high_array.shape != shape
            or not np.array_equal(low_array, np.full(shape, -1.0))
            or not np.array_equal(high_array, np.full(shape, 1.0))
        ):
            raise DreamerDemoReplayError(
                f"native action space {key!r} must use exact normalized [-1, 1] bounds"
            )

    native_obs_schema = _space_descriptors(live_obs_spaces)
    native_act_schema = _space_descriptors(live_act_spaces)
    native_control_schema = _space_descriptors(
        {key: obs_spaces[key] for key in _CONTROL_KEYS}
    )
    row_bytes = _CALLBACK_METADATA_BYTES + sum(
        descriptor["nbytes"]
        for schema in (native_obs_schema, native_act_schema, native_control_schema)
        for descriptor in schema.values()
    )
    row_bytes = require_bounded_int(
        row_bytes,
        resource="Dreamer demo native row bytes",
        minimum=1,
        maximum=MAX_IN_MEMORY_FILE_BYTES,
    )
    per_worker_overhead = _checked_add(
        replay_length - 1,
        _checked_mul(
            2,
            chunksize,
            resource="Dreamer demo chunk overhead",
            maximum=MAX_ARRAY_ELEMENTS,
        ),
        resource="Dreamer demo worker storage overhead",
        maximum=MAX_ARRAY_ELEMENTS,
    )
    storage_rows = _checked_add(
        capacity,
        _checked_mul(
            worker_count,
            per_worker_overhead,
            resource="Dreamer demo worker storage rows",
            maximum=MAX_ARRAY_ELEMENTS,
        ),
        resource="Dreamer demo native storage rows",
        maximum=MAX_ARRAY_ELEMENTS,
    )
    admission_bytes = _checked_mul(
        storage_rows,
        row_bytes,
        resource="Dreamer demo configured replay bytes",
        maximum=MAX_IN_MEMORY_FILE_BYTES,
    )

    source_path = Path(dataset_path).expanduser().resolve()
    try:
        with open_demo_transition_source(
            source_path,
            max_transitions=limit,
        ) as source:
            transitions = source.transitions
            if not transitions or len(transitions) > limit:
                raise DreamerDemoReplayError(
                    "canonical demo source exceeds the requested transition bound"
                )
            identity = source.manifest.identity
            if identity.task != requested_task:
                raise DreamerDemoReplayError(
                    f"canonical demo task {identity.task!r} does not match task_id {task_id!r}"
                )
            contract_value = validate_demo_io_contract(
                source.demo_io_contract,
                required=True,
                expected_task=identity.task,
            )
            if contract_value is None:  # pragma: no cover - required=True invariant
                raise DreamerDemoReplayError(
                    "canonical demo source requires a demo_io_contract"
                )
            action_dim = _validate_action_contract(contract_value)

            source_binding = build_demo_source_binding(
                source,
                contract=contract_value,
            )
            projection = _find_demo_projection(
                wrapped_env,
                contract=contract_value,
            )
            projection.validate_final_spaces(wrapped_env)
            if projection.contract != source_binding["demo_io_contract"]:
                raise DreamerDemoReplayError(
                    "source demo_io_contract differs from the live Dreamer projection"
                )

            episodes = _group_episodes(transitions)
            native_row_count = 0
            for episode in episodes:
                native_row_count = _checked_add(
                    native_row_count,
                    _checked_add(
                        len(episode),
                        1,
                        resource="Dreamer demo episode row count",
                        maximum=MAX_ARRAY_ELEMENTS,
                    ),
                    resource="Dreamer demo native row count",
                    maximum=MAX_ARRAY_ELEMENTS,
                )
            if native_row_count <= 0:
                raise DreamerDemoReplayError("canonical demo source has no episodes")
            replay_window_count = native_row_count - replay_length + 1
            if replay_window_count < 1:
                raise DreamerDemoReplayError(
                    "Dreamer demo source is shorter than one native replay sequence"
                )
            if replay_window_count > capacity:
                raise DreamerDemoReplayError(
                    f"Dreamer demo replay capacity {capacity} cannot hold "
                    f"{replay_window_count} native replay windows"
                )
            staged_bytes = _checked_mul(
                native_row_count,
                row_bytes,
                resource="Dreamer demo staged native row bytes",
                maximum=MAX_IN_MEMORY_FILE_BYTES,
            )
            effective_config = _effective_config_binding(config)
            staging_budget = ByteBudget(
                "Dreamer demo staged rows",
                MAX_IN_MEMORY_FILE_BYTES,
            )
            rows: list[Mapping[str, Any]] = []
            for episode in episodes:
                previous_next: Mapping[str, np.ndarray[Any, Any]] | None = None
                previous_transition: Any | None = None
                for index, transition in enumerate(episode):
                    try:
                        observation = projection.project_observation(transition.obs)
                        next_observation = projection.project_observation(
                            transition.next_obs
                        )
                    except (TypeError, ValueError) as exc:
                        raise DreamerDemoReplayError(
                            f"canonical episode {transition.episode_key!r} observation projection failed"
                        ) from exc
                    observation = _native_mapping(
                        observation,
                        live_obs_spaces,
                        resource=f"demo episode {transition.episode_key!r} observation {index}",
                    )
                    next_observation = _native_mapping(
                        next_observation,
                        live_obs_spaces,
                        resource=f"demo episode {transition.episode_key!r} next observation {index}",
                    )
                    if previous_next is not None and not _mapping_equal(
                        previous_next, observation
                    ):
                        raise DreamerDemoReplayError(
                            f"canonical episode {transition.episode_key!r} next_obs/obs adjacency mismatch at row {index}"
                        )
                    canonical_action = _validate_action_value(
                        transition.action,
                        action_dim=action_dim,
                        index=transition.step_index,
                    )
                    try:
                        native_action = projection.project_action(canonical_action)
                    except (TypeError, ValueError) as exc:
                        raise DreamerDemoReplayError(
                            f"canonical episode {transition.episode_key!r} action projection failed"
                        ) from exc
                    native_action = _native_mapping(
                        native_action,
                        live_act_spaces,
                        resource=f"demo episode {transition.episode_key!r} action {index}",
                    )
                    controls = {
                        "reward": (
                            0.0
                            if previous_transition is None
                            else previous_transition.reward
                        ),
                        "is_first": previous_transition is None,
                        "is_last": (
                            False
                            if previous_transition is None
                            else previous_transition.terminated
                            or previous_transition.truncated
                        ),
                        "is_terminal": (
                            False
                            if previous_transition is None
                            else previous_transition.terminated
                        ),
                    }
                    rows.append(
                        _build_native_row(
                            observation,
                            native_action,
                            controls,
                            control_spaces,
                            staging_budget=staging_budget,
                            resource=(
                                f"demo episode {transition.episode_key!r} row {index}"
                            ),
                        )
                    )
                    previous_next = next_observation
                    previous_transition = transition
                if previous_next is None or previous_transition is None:
                    raise DreamerDemoReplayError("canonical demo episode is empty")
                try:
                    zero_action = projection.project_action(
                        np.zeros(action_dim, dtype=np.float32)
                    )
                except (TypeError, ValueError) as exc:
                    raise DreamerDemoReplayError(
                        "Dreamer zero action sentinel cannot be projected"
                    ) from exc
                zero_action = _native_mapping(
                    zero_action,
                    live_act_spaces,
                    resource=f"demo episode {episode[0].episode_key!r} zero action",
                )
                rows.append(
                    _build_native_row(
                        previous_next,
                        zero_action,
                        {
                            "reward": previous_transition.reward,
                            "is_first": False,
                            "is_last": previous_transition.terminated
                            or previous_transition.truncated,
                            "is_terminal": previous_transition.terminated,
                        },
                        control_spaces,
                        staging_budget=staging_budget,
                        resource=(
                            f"demo episode {episode[0].episode_key!r} terminal row"
                        ),
                    )
                )

            if len(rows) != native_row_count:  # pragma: no cover - builder invariant
                raise DreamerDemoReplayError(
                    "Dreamer demo projected row count differs from the admitted count"
                )

            projection_metadata = projection.metadata
            binding: dict[str, Any] = {
                "version": DEMO_REPLAY_BINDING_VERSION,
                "dataset": source_binding,
                "requested_task_id": task_id,
                "canonical_task_id": requested_task,
                "dataset_path": str(source_path),
                "dataset_fingerprint": source.content_fingerprint,
                "transition_count": len(transitions),
                "native_row_count": native_row_count,
                "replay_window_count": replay_window_count,
                "demo_projection": projection_metadata,
                "config": effective_config,
                "sequence": {
                    "batch_size": batch_size,
                    "batch_length": batch_length,
                    "replay_context": replay_context,
                    "consec_train": consec_train,
                    "length": replay_length,
                    "capacity": capacity,
                    "chunksize": chunksize,
                    "replicas": replicas,
                    "environment_count": environment_count,
                    "worker_count": worker_count,
                },
                "native_row_schema": {
                    "observation": native_obs_schema,
                    "action": native_act_schema,
                    "controls": native_control_schema,
                    "callback_metadata_bytes": _CALLBACK_METADATA_BYTES,
                    "row_bytes": row_bytes,
                },
                "replay": {
                    "exclude_log_prefix": "log/",
                    "exclusion_owner": "embodied.core.replay.Replay.add",
                    "storage_rows": storage_rows,
                    "admission_bytes": admission_bytes,
                },
            }
            optional_replay = {
                key: _config_value(config, f"replay.{key}")
                for key in ("online", "save", "save_wait")
            }
            optional_replay = {
                key: value
                for key, value in optional_replay.items()
                if value is not None
            }
            if optional_replay:
                binding["replay"].update(optional_replay)
            _normalized_binding, binding_json = _canonical_json(
                binding,
                label="Dreamer demo replay binding",
            )
            return DreamerDemoReplayPayload(
                rows=tuple(rows),
                dataset_fingerprint=source.content_fingerprint,
                dataset_path=str(source_path),
                transition_count=len(transitions),
                capacity=capacity,
                chunksize=chunksize,
                replay_length=replay_length,
                worker_count=worker_count,
                batch_size=batch_size,
                batch_length=batch_length,
                replay_context=replay_context,
                consec_train=consec_train,
                replicas=replicas,
                row_bytes=row_bytes,
                staged_bytes=staged_bytes,
                admission_bytes=admission_bytes,
                _binding_json=binding_json,
            )
    except DreamerDemoReplayError:
        raise
    except (KeyError, TypeError, ValueError, OverflowError, OSError) as exc:
        raise DreamerDemoReplayError(str(exc)) from exc


__all__ = [
    "DEMO_REPLAY_BINDING_VERSION",
    "DreamerDemoReplayError",
    "DreamerDemoReplayPayload",
    "build_demo_replay_payload",
]
