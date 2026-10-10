# SPDX-License-Identifier: Apache-2.0 OR MIT
"""Bounded admission and owned row buffering for robomimic demonstrations."""

from __future__ import annotations

import numbers
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Literal, cast

import numpy as np

from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_DIRECTORY_ENTRIES,
    MAX_EVENT_ROWS,
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_METADATA_AGGREGATE_BYTES,
    MAX_PUBLICATION_DIRECTORY_DEPTH,
    ResourceLimitError,
    checked_shape_nbytes,
    require_bounded_int,
)

_MAX_HDF5_RANK = 8
_MAX_BATCH_ROWS = 1_024
_METADATA_NODE_BYTES = 32
_METADATA_KEY_BYTES = 32
_METADATA_SEQUENCE_BYTES = 16
_SUPPORTED_DTYPE_KINDS = frozenset("biuf")


@dataclass(frozen=True, slots=True)
class _FrozenActionLeaf:
    shape: tuple[int, ...]
    dtype: np.dtype[Any] | None

    def __deepcopy__(self, memo: dict[int, object]) -> _FrozenActionLeaf:
        del memo
        return self


@dataclass(frozen=True, slots=True)
class _FrozenActionContainer:
    spaces: Mapping[str, _FrozenActionNode] | tuple[_FrozenActionNode, ...]

    def __deepcopy__(self, memo: dict[int, object]) -> _FrozenActionContainer:
        del memo
        return self


_FrozenActionNode = _FrozenActionLeaf | _FrozenActionContainer


@dataclass(slots=True)
class _ActionSpaceBudget:
    nodes: int = 0
    metadata_bytes: int = 0


@dataclass(frozen=True, slots=True)
class _LeafPlan:
    source: object
    shape: tuple[int, ...]
    dtype: np.dtype[Any]
    nbytes: int
    kind: Literal["array", "torch", "sequence", "scalar"]
    temporary_bytes: int = 0


@dataclass(frozen=True, slots=True)
class _MappingPlan:
    children: tuple[tuple[str, _NodePlan], ...]
    metadata_bytes: int


_NodePlan = _LeafPlan | _MappingPlan


@dataclass(frozen=True, slots=True)
class _LeafSchema:
    shape: tuple[int, ...]
    dtype: np.dtype[Any]


@dataclass(frozen=True, slots=True)
class _MappingSchema:
    children: tuple[tuple[str, _NodeSchema], ...]


_NodeSchema = _LeafSchema | _MappingSchema


@dataclass(frozen=True, slots=True)
class _ActionSummary:
    elements: int
    dtype: np.dtype[Any]
    metadata_bytes: int
    temporary_bytes: int = 0
    snapshot: object | None = None
    already_flat: bool = False


@dataclass(frozen=True, slots=True)
class _Row:
    obs: dict[str, object]
    next_obs: dict[str, object]
    action: np.ndarray[Any, Any]
    reward: np.ndarray[Any, Any]
    done: bool
    terminated: bool
    truncated: bool


def _summary_snapshot(summary: _ActionSummary) -> object:
    snapshot = summary.snapshot
    if snapshot is None:
        raise RuntimeError("action declaration has no snapshot")
    return snapshot


def _is_torch_tensor(value: object) -> bool:
    """Identify a Torch tensor without importing Torch at module import time."""

    try:
        import torch
    except ImportError:
        return False
    return isinstance(value, torch.Tensor)


def _torch_numpy_dtype(value: object) -> np.dtype[Any]:
    """Map a Torch dtype without moving or materializing its tensor."""

    dtype = getattr(value, "dtype", None)
    name = str(dtype)
    mapped = {
        "torch.bool": np.dtype(np.bool_),
        "torch.uint8": np.dtype(np.uint8),
        "torch.uint16": np.dtype(np.uint16),
        "torch.uint32": np.dtype(np.uint32),
        "torch.uint64": np.dtype(np.uint64),
        "torch.int8": np.dtype(np.int8),
        "torch.int16": np.dtype(np.int16),
        "torch.int32": np.dtype(np.int32),
        "torch.int64": np.dtype(np.int64),
        "torch.float16": np.dtype(np.float16),
        "torch.float32": np.dtype(np.float32),
        "torch.float64": np.dtype(np.float64),
        "torch.complex32": np.dtype(np.complex64),
        "torch.complex64": np.dtype(np.complex64),
        "torch.complex128": np.dtype(np.complex128),
    }
    try:
        return mapped[name]
    except KeyError as exc:
        raise TypeError(f"unsupported Torch dtype {dtype!r}") from exc


def _normalise_shape(shape: object, *, resource: str) -> tuple[int, ...]:
    try:
        iterator = iter(cast(Any, shape))
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{resource} shape must be a sequence") from exc
    dimensions: list[object] = []
    for index, dimension in enumerate(iterator):
        if index >= _MAX_HDF5_RANK:
            raise ResourceLimitError(
                f"{resource} rank", limit=_MAX_HDF5_RANK, observed=index + 1
            )
        dimensions.append(dimension)
    normalised: list[int] = []
    for index, dimension in enumerate(dimensions):
        normalised.append(
            require_bounded_int(
                dimension,
                resource=f"{resource} shape[{index}]",
                minimum=0,
                maximum=MAX_ARRAY_ELEMENTS,
            )
        )
    return tuple(normalised)


def _validate_dtype(dtype: np.dtype[Any], *, resource: str) -> None:
    if (
        dtype.hasobject
        or dtype.fields is not None
        or dtype.kind not in _SUPPORTED_DTYPE_KINDS
    ):
        raise TypeError(f"{resource} has unsupported dtype {dtype}")


def _inspect_declared_array(
    value: object,
    *,
    resource: str,
    kind: Literal["array", "torch", "scalar"],
) -> _LeafPlan:
    if kind == "torch":
        shape = _normalise_shape(getattr(value, "shape", ()), resource=resource)
        dtype = _torch_numpy_dtype(value)
    elif isinstance(value, np.ndarray):
        shape = _normalise_shape(value.shape, resource=resource)
        dtype = np.dtype(value.dtype)
    elif isinstance(value, np.generic):
        shape = ()
        dtype = np.dtype(value.dtype)
    else:
        shape = ()
        dtype = _scalar_dtype(value, resource=resource)

    _validate_dtype(dtype, resource=resource)
    nbytes = checked_shape_nbytes(
        shape,
        dtype,
        resource=resource,
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
        max_rank=_MAX_HDF5_RANK,
    )
    temporary_bytes = 0
    if kind == "torch":
        device = getattr(value, "device", None)
        if getattr(device, "type", "cpu") != "cpu":
            temporary_bytes = nbytes
    return _LeafPlan(value, shape, dtype, nbytes, kind, temporary_bytes)


def _scalar_dtype(value: object, *, resource: str) -> np.dtype[Any]:
    """Inspect a scalar's NumPy dtype without invoking an array conversion."""

    if isinstance(value, bool):
        return np.dtype(np.bool_)
    if isinstance(value, int):
        limits = np.iinfo(np.int64)
        if value < int(limits.min) or value > int(limits.max):
            raise ResourceLimitError(
                f"{resource} integer", limit=int(limits.max), observed=abs(value)
            )
        return np.dtype(np.int64)
    if isinstance(value, float):
        return np.dtype(np.float64)
    if isinstance(value, complex):
        return np.dtype(np.complex128)
    try:
        return np.dtype(type(value))
    except TypeError as exc:
        raise TypeError(f"{resource} is not a supported scalar") from exc


def _inspect_sequence(
    value: Sequence[Any],
    *,
    resource: str,
    active: frozenset[int],
) -> _LeafPlan:
    identity = id(value)
    if identity in active:
        raise ValueError(f"{resource} contains a cyclic sequence")
    active = active | {identity}
    try:
        try:
            declared_length = require_bounded_int(
                len(value),
                resource=f"{resource} length",
                minimum=0,
                maximum=MAX_DIRECTORY_ENTRIES,
            )
        except ResourceLimitError:
            raise
        except (TypeError, ValueError) as exc:
            raise TypeError(f"{resource} length is not bounded") from exc

        child_shape: tuple[int, ...] | None = None
        child_dtype: np.dtype[Any] | None = None
        child_plans: list[_LeafPlan] = []
        observed = 0
        for child in value:
            observed += 1
            if observed > declared_length:
                raise ValueError(
                    f"{resource} sequence length changed during inspection: "
                    f"declared {declared_length}, observed {observed}"
                )
            if observed > MAX_ARRAY_ELEMENTS:
                raise ResourceLimitError(
                    f"{resource} elements",
                    limit=MAX_ARRAY_ELEMENTS,
                    observed=observed,
                )
            if isinstance(child, Mapping):
                raise TypeError(f"{resource} sequence contains a mapping leaf")
            child_plan = _inspect_value(
                child,
                resource=f"{resource}[{observed - 1}]",
                active=active,
            )
            child_plans.append(child_plan)
            if child_shape is None:
                child_shape = child_plan.shape
                child_dtype = child_plan.dtype
            elif child_plan.shape != child_shape:
                raise ValueError(
                    f"{resource} sequence has inconsistent shapes: "
                    f"{child_shape} and {child_plan.shape}"
                )
            else:
                assert child_dtype is not None
                child_dtype = np.dtype(np.result_type(child_dtype, child_plan.dtype))

        if observed != declared_length:
            raise ValueError(
                f"{resource} sequence length changed during inspection: "
                f"declared {declared_length}, observed {observed}"
            )
        if child_shape is None:
            child_shape = ()
            child_dtype = np.dtype(np.float64)
        assert child_dtype is not None
        shape = (declared_length, *child_shape)
        _validate_dtype(child_dtype, resource=resource)
        nbytes = checked_shape_nbytes(
            shape,
            child_dtype,
            resource=resource,
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
            max_rank=_MAX_HDF5_RANK,
        )
        temporary_bytes = sum(child_plan.temporary_bytes for child_plan in child_plans)
        return _LeafPlan(
            tuple(child_plans),
            shape,
            child_dtype,
            nbytes,
            "sequence",
            temporary_bytes,
        )
    finally:
        # ``active`` is immutable; no caller-visible state needs rollback.
        del active


def _inspect_value(
    value: object,
    *,
    resource: str,
    active: frozenset[int] = frozenset(),
) -> _LeafPlan:
    if _is_torch_tensor(value):
        return _inspect_declared_array(value, resource=resource, kind="torch")
    if isinstance(value, np.ndarray):
        return _inspect_declared_array(value, resource=resource, kind="array")
    if isinstance(value, np.generic):
        return _inspect_declared_array(value, resource=resource, kind="scalar")
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return _inspect_sequence(value, resource=resource, active=active)
    if isinstance(value, (bool, numbers.Number)):
        return _inspect_declared_array(value, resource=resource, kind="scalar")
    raise TypeError(f"{resource} has unsupported value type {type(value).__name__}")


def _validate_hdf5_key(key: object, *, resource: str) -> str:
    if not isinstance(key, str) or not key:
        raise ValueError(f"{resource} mapping keys must be non-empty strings")
    if key.startswith("/") or "/" in key or "\x00" in key or key in {".", ".."}:
        raise ValueError(
            f"{resource} mapping key must not be absolute, contain '/', NUL, "
            "or dot path components"
        )
    try:
        key.encode("utf-8")
    except UnicodeEncodeError as exc:
        raise ValueError(f"{resource} mapping key is not valid UTF-8") from exc
    return key


def _metadata_key_bytes(key: str) -> int:
    return _METADATA_KEY_BYTES + len(key.encode("utf-8"))


def _metadata_action_key_bytes(key: object) -> int:
    if isinstance(key, str):
        return _metadata_key_bytes(key)
    return _METADATA_KEY_BYTES + 16


def _bounded_action_space_mapping(
    value: Mapping[object, object], *, resource: str
) -> tuple[tuple[str, object], ...]:
    """Snapshot a mapping space through a bounded, single traversal."""

    try:
        declared_length = require_bounded_int(
            len(value),
            resource=f"{resource} mapping entries",
            minimum=0,
            maximum=MAX_DIRECTORY_ENTRIES,
        )
    except ResourceLimitError:
        raise
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{resource} mapping length is not bounded") from exc

    children: list[tuple[str, object]] = []
    seen: set[str] = set()
    for raw_key in value:
        if len(children) >= declared_length:
            raise ValueError(f"{resource} mapping length changed during inspection")
        if len(children) >= MAX_DIRECTORY_ENTRIES:
            raise ResourceLimitError(
                f"{resource} mapping entries",
                limit=MAX_DIRECTORY_ENTRIES,
                observed=len(children) + 1,
            )
        if not isinstance(raw_key, str):
            raise ValueError(f"{resource} mapping keys must be strings")
        if raw_key in seen:
            raise ValueError(f"{resource} mapping contains duplicate key {raw_key!r}")
        seen.add(raw_key)
        try:
            child = value[raw_key]
        except (KeyError, TypeError) as exc:
            raise ValueError(
                f"{resource} mapping changed while reading key {raw_key!r}"
            ) from exc
        children.append((raw_key, child))
    if len(children) != declared_length:
        raise ValueError(f"{resource} mapping length changed during inspection")
    return tuple(children)


def _bounded_action_space_sequence(
    value: Sequence[object], *, resource: str
) -> tuple[object, ...]:
    """Snapshot a tuple/list space through bounded indexed reads."""

    try:
        declared_length = require_bounded_int(
            len(value),
            resource=f"{resource} sequence elements",
            minimum=0,
            maximum=MAX_DIRECTORY_ENTRIES,
        )
    except ResourceLimitError:
        raise
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{resource} sequence length is not bounded") from exc
    children: list[object] = []
    for index in range(declared_length):
        try:
            children.append(value[index])
        except (IndexError, KeyError, TypeError) as exc:
            raise ValueError(
                f"{resource} sequence length changed at index {index}"
            ) from exc
    return tuple(children)


def _freeze_action_space(
    value: object,
    *,
    resource: str = "action space",
    depth: int = 0,
    budget: _ActionSpaceBudget | None = None,
    metadata_limit: int = MAX_METADATA_AGGREGATE_BYTES,
) -> tuple[_FrozenActionNode, int]:
    """Capture bounded action shape/order metadata without copying bounds."""

    from srb.core.env.runtime.action_layout import classify_action_space

    if budget is None:
        budget = _ActionSpaceBudget()
    if depth > MAX_PUBLICATION_DIRECTORY_DEPTH:
        raise ResourceLimitError(
            f"{resource} depth",
            limit=MAX_PUBLICATION_DIRECTORY_DEPTH,
            observed=depth,
        )
    budget.nodes += 1
    if budget.nodes > MAX_DIRECTORY_ENTRIES:
        raise ResourceLimitError(
            f"{resource} nodes",
            limit=MAX_DIRECTORY_ENTRIES,
            observed=budget.nodes,
        )

    kind = classify_action_space(value)
    node_metadata = (
        _METADATA_NODE_BYTES
        if kind == "mapping"
        else _METADATA_SEQUENCE_BYTES
        if kind == "sequence"
        else 0
    )
    budget.metadata_bytes += node_metadata
    if budget.metadata_bytes > metadata_limit:
        raise ResourceLimitError(
            f"{resource} metadata",
            limit=metadata_limit,
            observed=budget.metadata_bytes,
        )

    if kind == "mapping":
        if isinstance(value, Mapping):
            source = value
        else:
            children = getattr(value, "spaces", None)
            if not isinstance(children, Mapping):
                raise TypeError(f"{resource} mapping space has no mapping children")
            source = children
        mapping_children = _bounded_action_space_mapping(source, resource=resource)
        owned: dict[str, _FrozenActionNode] = {}
        for key, child in mapping_children:
            key_bytes = _metadata_key_bytes(key)
            budget.metadata_bytes += key_bytes
            if budget.metadata_bytes > metadata_limit:
                raise ResourceLimitError(
                    f"{resource} metadata",
                    limit=metadata_limit,
                    observed=budget.metadata_bytes,
                )
            frozen_child, _child_metadata = _freeze_action_space(
                child,
                resource=f"{resource}/{key}",
                depth=depth + 1,
                budget=budget,
                metadata_limit=metadata_limit,
            )
            owned[key] = frozen_child
        return _FrozenActionContainer(MappingProxyType(owned)), budget.metadata_bytes

    if kind == "sequence":
        sequence_source = getattr(value, "spaces", None)
        if not isinstance(sequence_source, (tuple, list)):
            raise TypeError(f"{resource} sequence space has no tuple/list children")
        declared_children = _bounded_action_space_sequence(
            sequence_source, resource=resource
        )
        owned_sequence: list[_FrozenActionNode] = []
        for index, child in enumerate(declared_children):
            frozen_child, _child_metadata = _freeze_action_space(
                child,
                resource=f"{resource}[{index}]",
                depth=depth + 1,
                budget=budget,
                metadata_limit=metadata_limit,
            )
            owned_sequence.append(frozen_child)
        return _FrozenActionContainer(tuple(owned_sequence)), budget.metadata_bytes

    shape_value = getattr(value, "shape", ())
    if shape_value is None:
        shape_value = ()
    shape = _normalise_shape(shape_value, resource=f"{resource} shape")
    dtype_value = getattr(value, "dtype", None)
    dtype: np.dtype[Any] | None
    if dtype_value is None:
        dtype = None
        dtype_for_bounds = np.dtype(np.float32)
    else:
        try:
            dtype = np.dtype(dtype_value)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"{resource} has unsupported dtype {dtype_value!r}"
            ) from exc
        dtype_for_bounds = dtype
    checked_shape_nbytes(
        shape,
        dtype_for_bounds,
        resource=f"{resource} shape",
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
        max_rank=_MAX_HDF5_RANK,
    )
    return _FrozenActionLeaf(shape, dtype), budget.metadata_bytes


def snapshot_action_space(
    value: object,
    *,
    max_metadata_bytes: int = MAX_METADATA_AGGREGATE_BYTES,
) -> tuple[object, int]:
    """Return a bounded immutable action declaration and its metadata bytes.

    The declaration retains only recursive shape, dtype and insertion order.
    Space bounds and other mutable Gym state are deliberately not copied.
    """

    metadata_limit = require_bounded_int(
        max_metadata_bytes,
        resource="action space metadata limit",
        minimum=0,
        maximum=MAX_METADATA_AGGREGATE_BYTES,
    )
    frozen, metadata_bytes = _freeze_action_space(
        value,
        metadata_limit=metadata_limit,
    )
    return frozen, metadata_bytes


def _bounded_observation_keys(value: object) -> tuple[str, ...]:
    """Copy an observation-key iterable only after bounding its traversal."""

    if isinstance(value, (str, bytes, bytearray)):
        raise TypeError("obs_keys must be a sequence of strings")

    try:
        declared_length = len(cast(Any, value))
    except (AttributeError, TypeError):
        declared_length = None
    else:
        declared_length = require_bounded_int(
            declared_length,
            resource="observation keys",
            minimum=0,
            maximum=MAX_DIRECTORY_ENTRIES,
        )

    keys: list[str] = []
    try:
        iterator = iter(cast(Any, value))
    except TypeError as exc:
        raise TypeError("obs_keys must be a sequence of strings") from exc
    for raw_key in iterator:
        if len(keys) >= MAX_DIRECTORY_ENTRIES:
            raise ResourceLimitError(
                "observation keys",
                limit=MAX_DIRECTORY_ENTRIES,
                observed=len(keys) + 1,
            )
        key = _validate_hdf5_key(raw_key, resource="observation")
        keys.append(key)
    if declared_length is not None and len(keys) != declared_length:
        raise ValueError(
            "observation keys length changed during inspection: "
            f"declared {declared_length}, observed {len(keys)}"
        )
    if len(set(keys)) != len(keys):
        raise ValueError("obs_keys must be unique")
    return tuple(keys)


def _inspect_observation_node(
    value: object,
    *,
    resource: str,
    active: frozenset[int],
    depth: int = 0,
    metadata_limit: int = MAX_METADATA_AGGREGATE_BYTES,
) -> tuple[_NodePlan, int]:
    if not isinstance(value, Mapping):
        plan = _inspect_value(value, resource=resource, active=active)
        return plan, 0

    if depth > MAX_PUBLICATION_DIRECTORY_DEPTH:
        raise ResourceLimitError(
            f"{resource} mapping depth",
            limit=MAX_PUBLICATION_DIRECTORY_DEPTH,
            observed=depth,
        )

    identity = id(value)
    if identity in active:
        raise ValueError(f"{resource} contains a cyclic mapping")
    active = active | {identity}
    try:
        try:
            entry_count = require_bounded_int(
                len(value),
                resource=f"{resource} mapping entries",
                minimum=1,
                maximum=MAX_DIRECTORY_ENTRIES,
            )
        except TypeError as exc:
            raise TypeError(f"{resource} mapping length is not bounded") from exc
        children: list[tuple[str, _NodePlan]] = []
        seen_keys: set[str] = set()
        metadata_bytes = _METADATA_NODE_BYTES
        if metadata_bytes > metadata_limit:
            raise ResourceLimitError(
                f"{resource} metadata",
                limit=metadata_limit,
                observed=metadata_bytes,
            )
        observed = 0
        for raw_key in value:
            observed += 1
            if observed > entry_count:
                raise ValueError(
                    f"{resource} mapping length changed during inspection: "
                    f"declared {entry_count}, observed {observed}"
                )
            if observed > MAX_DIRECTORY_ENTRIES:
                raise ResourceLimitError(
                    f"{resource} mapping entries",
                    limit=MAX_DIRECTORY_ENTRIES,
                    observed=observed,
                )
            key = _validate_hdf5_key(raw_key, resource=resource)
            if key in seen_keys:
                raise ValueError(f"{resource} mapping contains duplicate key {key!r}")
            seen_keys.add(key)
            metadata_bytes += _metadata_key_bytes(key)
            if metadata_bytes > metadata_limit:
                raise ResourceLimitError(
                    f"{resource} metadata",
                    limit=metadata_limit,
                    observed=metadata_bytes,
                )
            child, child_metadata = _inspect_observation_node(
                value[raw_key],
                resource=f"{resource}/{key}",
                active=active,
                depth=depth + 1,
                metadata_limit=metadata_limit - metadata_bytes,
            )
            metadata_bytes += child_metadata
            if metadata_bytes > metadata_limit:
                raise ResourceLimitError(
                    f"{resource} metadata",
                    limit=metadata_limit,
                    observed=metadata_bytes,
                )
            children.append((key, child))
        if observed != entry_count:
            raise ValueError(
                f"{resource} mapping length changed during inspection: "
                f"declared {entry_count}, observed {observed}"
            )
        if not children:
            raise ValueError(f"{resource} must not be an empty mapping")
        return _MappingPlan(tuple(children), metadata_bytes), metadata_bytes
    finally:
        del active


def _plan_temporary_bytes(plan: _NodePlan) -> int:
    if isinstance(plan, _MappingPlan):
        return sum(_plan_temporary_bytes(child) for _key, child in plan.children)
    return plan.temporary_bytes


def _action_leaf_temporary_bytes(plan: _LeafPlan) -> int:
    """Estimate transient storage used while canonical action flattening runs."""

    temporary_bytes = plan.temporary_bytes
    if plan.kind in {"scalar", "sequence"}:
        # Scalar and sequence leaves materialize before canonical action concatenation.
        return temporary_bytes + plan.nbytes
    if plan.kind == "array" and isinstance(plan.source, np.ndarray):
        if not bool(plan.source.flags.c_contiguous):
            return temporary_bytes + plan.nbytes
    if plan.kind == "torch":
        is_contiguous = getattr(plan.source, "is_contiguous", None)
        if callable(is_contiguous):
            try:
                if not bool(is_contiguous()):
                    temporary_bytes += plan.nbytes
            except (RuntimeError, TypeError):
                # A failed contiguity query is handled conservatively during conversion.
                temporary_bytes += plan.nbytes
    return temporary_bytes


def _inspect_action_node(
    value: object,
    space: object,
    *,
    resource: str,
    active: frozenset[int],
    depth: int = 0,
    metadata_limit: int = MAX_METADATA_AGGREGATE_BYTES,
) -> _ActionSummary:
    from srb.core.env.runtime.action_layout import (
        classify_action_space,
        iter_action_space_children,
    )

    kind = classify_action_space(space)
    children = tuple(iter_action_space_children(space))
    if kind in {"mapping", "sequence"} and depth > MAX_PUBLICATION_DIRECTORY_DEPTH:
        raise ResourceLimitError(
            f"{resource} action depth",
            limit=MAX_PUBLICATION_DIRECTORY_DEPTH,
            observed=depth,
        )
    structural_metadata = (
        _METADATA_NODE_BYTES
        if kind == "mapping"
        else _METADATA_SEQUENCE_BYTES
        if kind == "sequence"
        else 0
    )
    if structural_metadata > metadata_limit:
        raise ResourceLimitError(
            f"{resource} metadata",
            limit=metadata_limit,
            observed=structural_metadata,
        )
    if kind == "mapping":
        if not isinstance(value, Mapping):
            raise TypeError(f"{resource} must be a mapping")
        identity = id(value)
        if identity in active:
            raise ValueError(f"{resource} contains a cyclic mapping")
        active = active | {identity}
        try:
            try:
                actual_count = require_bounded_int(
                    len(value),
                    resource=f"{resource} mapping entries",
                    minimum=0,
                    maximum=MAX_DIRECTORY_ENTRIES,
                )
            except ResourceLimitError:
                raise
            except (TypeError, ValueError) as exc:
                raise TypeError(f"{resource} mapping length is not bounded") from exc
            expected_keys = {key for key, _child_space in children}
            actual_keys: list[object] = []
            actual_values: dict[object, object] = {}
            metadata_bytes = _METADATA_NODE_BYTES
            for key in value:
                actual_keys.append(key)
                if len(actual_keys) > actual_count:
                    raise ValueError(
                        f"{resource} mapping length changed during inspection"
                    )
                metadata_bytes += _metadata_action_key_bytes(key)
                if metadata_bytes > metadata_limit:
                    raise ResourceLimitError(
                        f"{resource} metadata",
                        limit=metadata_limit,
                        observed=metadata_bytes,
                    )
                if len(actual_keys) > MAX_DIRECTORY_ENTRIES:
                    raise ResourceLimitError(
                        f"{resource} mapping entries",
                        limit=MAX_DIRECTORY_ENTRIES,
                        observed=len(actual_keys),
                    )
                actual_values[key] = value[key]
            if len(actual_keys) != actual_count:
                raise ValueError(f"{resource} mapping length changed during inspection")
            actual_key_set = set(actual_keys)
            if actual_key_set != expected_keys:
                missing = sorted(str(key) for key in expected_keys - actual_key_set)
                extra = sorted(str(key) for key in actual_key_set - expected_keys)
                raise ValueError(
                    f"{resource} keys do not match action space: "
                    f"missing={missing}, extra={extra}"
                )
            elements = 0
            dtype: np.dtype[Any] | None = None
            temporary_bytes = 0
            mapping_snapshots: dict[object, object] = {}
            for key, child_space in children:
                child = _inspect_action_node(
                    actual_values[key],
                    child_space,
                    resource=f"{resource}/{key}",
                    active=active,
                    depth=depth + 1,
                    metadata_limit=metadata_limit - metadata_bytes,
                )
                elements += child.elements
                dtype = (
                    child.dtype
                    if dtype is None
                    else np.dtype(np.result_type(dtype, child.dtype))
                )
                metadata_bytes += child.metadata_bytes
                temporary_bytes += child.temporary_bytes
                mapping_snapshots[key] = _summary_snapshot(child)
                if metadata_bytes > metadata_limit:
                    raise ResourceLimitError(
                        f"{resource} metadata",
                        limit=metadata_limit,
                        observed=metadata_bytes,
                    )
            if dtype is None:
                dtype = np.dtype(np.float32)
            return _ActionSummary(
                elements,
                dtype,
                metadata_bytes,
                temporary_bytes,
                mapping_snapshots,
            )
        finally:
            del active

    if kind == "sequence":
        if not isinstance(value, (tuple, list)):
            raise TypeError(f"{resource} must be a tuple or list")
        identity = id(value)
        if identity in active:
            raise ValueError(f"{resource} contains a cyclic sequence")
        active = active | {identity}
        try:
            return _inspect_action_sequence(
                value,
                children,
                resource=resource,
                active=active,
                depth=depth + 1,
                metadata_limit=metadata_limit,
            )
        finally:
            del active

    plan = _inspect_value(value, resource=resource, active=active)
    expected_shape = _normalise_shape(
        getattr(space, "shape", ()), resource=f"{resource} action space"
    )
    if plan.shape != expected_shape:
        raise ValueError(f"{resource} shape must be {expected_shape}, got {plan.shape}")
    elements = plan.nbytes // int(plan.dtype.itemsize)
    return _ActionSummary(
        elements,
        plan.dtype,
        0,
        _action_leaf_temporary_bytes(plan),
        plan,
    )


def _inspect_action_sequence(
    value: tuple[Any, ...] | list[Any],
    children: tuple[tuple[object, object], ...],
    *,
    resource: str,
    active: frozenset[int],
    depth: int,
    metadata_limit: int,
) -> _ActionSummary:
    if len(value) != len(children):
        raise ValueError(
            f"{resource} has {len(value)} values, expected {len(children)}"
        )
    elements = 0
    action_dtype: np.dtype[Any] | None = None
    temporary_bytes = 0
    metadata_bytes = _METADATA_SEQUENCE_BYTES
    if metadata_bytes > metadata_limit:
        raise ResourceLimitError(
            f"{resource} metadata",
            limit=metadata_limit,
            observed=metadata_bytes,
        )
    snapshots: list[object] = []
    values = tuple(value[index] for index, _child_space in enumerate(children))
    for index, (_key, child_space) in enumerate(children):
        child = _inspect_action_node(
            values[index],
            child_space,
            resource=f"{resource}[{index}]",
            active=active,
            depth=depth,
            metadata_limit=metadata_limit - metadata_bytes,
        )
        elements += child.elements
        action_dtype = (
            child.dtype
            if action_dtype is None
            else np.dtype(np.result_type(action_dtype, child.dtype))
        )
        metadata_bytes += child.metadata_bytes
        temporary_bytes += child.temporary_bytes
        snapshots.append(_summary_snapshot(child))
        if metadata_bytes > metadata_limit:
            raise ResourceLimitError(
                f"{resource} metadata",
                limit=metadata_limit,
                observed=metadata_bytes,
            )
    if action_dtype is None:
        action_dtype = np.dtype(np.float32)
    return _ActionSummary(
        elements,
        action_dtype,
        metadata_bytes,
        temporary_bytes,
        tuple(snapshots),
    )


def _inspect_untyped_action(
    value: object,
    *,
    resource: str,
    active: frozenset[int] = frozenset(),
    depth: int = 0,
    metadata_limit: int = MAX_METADATA_AGGREGATE_BYTES,
) -> _ActionSummary:
    if (
        isinstance(value, (Mapping, tuple, list))
        and depth > MAX_PUBLICATION_DIRECTORY_DEPTH
    ):
        raise ResourceLimitError(
            f"{resource} action depth",
            limit=MAX_PUBLICATION_DIRECTORY_DEPTH,
            observed=depth,
        )
    if isinstance(value, Mapping):
        identity = id(value)
        if identity in active:
            raise ValueError(f"{resource} contains a cyclic mapping")
        active = active | {identity}
        try:
            try:
                count = require_bounded_int(
                    len(value),
                    resource=f"{resource} mapping entries",
                    minimum=0,
                    maximum=MAX_DIRECTORY_ENTRIES,
                )
            except ResourceLimitError:
                raise
            except (TypeError, ValueError) as exc:
                raise TypeError(f"{resource} mapping length is not bounded") from exc
            elements = 0
            action_dtype: np.dtype[Any] | None = None
            temporary_bytes = 0
            metadata_bytes = _METADATA_NODE_BYTES
            if metadata_bytes > metadata_limit:
                raise ResourceLimitError(
                    f"{resource} metadata",
                    limit=metadata_limit,
                    observed=metadata_bytes,
                )
            observed = 0
            snapshots: dict[object, object] = {}
            for key in value:
                observed += 1
                if observed > count:
                    raise ValueError(
                        f"{resource} mapping length changed during inspection"
                    )
                if observed > MAX_DIRECTORY_ENTRIES:
                    raise ResourceLimitError(
                        f"{resource} mapping entries",
                        limit=MAX_DIRECTORY_ENTRIES,
                        observed=observed,
                    )
                metadata_bytes += _metadata_action_key_bytes(key)
                if metadata_bytes > metadata_limit:
                    raise ResourceLimitError(
                        f"{resource} metadata",
                        limit=metadata_limit,
                        observed=metadata_bytes,
                    )
                child = _inspect_untyped_action(
                    value[key],
                    resource=f"{resource}/{key}",
                    active=active,
                    depth=depth + 1,
                    metadata_limit=metadata_limit - metadata_bytes,
                )
                elements += child.elements
                action_dtype = (
                    child.dtype
                    if action_dtype is None
                    else np.dtype(np.result_type(action_dtype, child.dtype))
                )
                metadata_bytes += child.metadata_bytes
                temporary_bytes += child.temporary_bytes
                snapshots[key] = _summary_snapshot(child)
                if metadata_bytes > metadata_limit:
                    raise ResourceLimitError(
                        f"{resource} metadata",
                        limit=metadata_limit,
                        observed=metadata_bytes,
                    )
            if observed != count:
                raise ValueError(f"{resource} mapping length changed during inspection")
            if action_dtype is None:
                action_dtype = np.dtype(np.float32)
            return _ActionSummary(
                elements,
                action_dtype,
                metadata_bytes,
                temporary_bytes,
                snapshots,
            )
        finally:
            del active
    if isinstance(value, (tuple, list)):
        identity = id(value)
        if identity in active:
            raise ValueError(f"{resource} contains a cyclic sequence")
        active = active | {identity}
        try:
            try:
                declared_length = require_bounded_int(
                    len(value),
                    resource=f"{resource} sequence elements",
                    minimum=0,
                    maximum=MAX_DIRECTORY_ENTRIES,
                )
            except ResourceLimitError:
                raise
            except (TypeError, ValueError) as exc:
                raise TypeError(f"{resource} sequence length is not bounded") from exc
            elements = 0
            dtype: np.dtype[Any] | None = None
            temporary_bytes = 0
            metadata_bytes = _METADATA_SEQUENCE_BYTES
            if metadata_bytes > metadata_limit:
                raise ResourceLimitError(
                    f"{resource} metadata",
                    limit=metadata_limit,
                    observed=metadata_bytes,
                )
            observed = 0
            sequence_snapshots: list[object] = []
            for index, child_value in enumerate(value):
                observed += 1
                if observed > declared_length:
                    raise ValueError(
                        f"{resource} sequence length changed during inspection: "
                        f"declared {declared_length}, observed {observed}"
                    )
                if observed > MAX_DIRECTORY_ENTRIES:
                    raise ResourceLimitError(
                        f"{resource} sequence elements",
                        limit=MAX_DIRECTORY_ENTRIES,
                        observed=observed,
                    )
                child = _inspect_untyped_action(
                    child_value,
                    resource=f"{resource}[{index}]",
                    active=active,
                    depth=depth + 1,
                    metadata_limit=metadata_limit - metadata_bytes,
                )
                elements += child.elements
                dtype = (
                    child.dtype
                    if dtype is None
                    else np.dtype(np.result_type(dtype, child.dtype))
                )
                metadata_bytes += child.metadata_bytes
                temporary_bytes += child.temporary_bytes
                sequence_snapshots.append(_summary_snapshot(child))
                if metadata_bytes > metadata_limit:
                    raise ResourceLimitError(
                        f"{resource} metadata",
                        limit=metadata_limit,
                        observed=metadata_bytes,
                    )
            if observed != declared_length:
                raise ValueError(
                    f"{resource} sequence length changed during inspection: "
                    f"declared {declared_length}, observed {observed}"
                )
            if dtype is None:
                dtype = np.dtype(np.float32)
            return _ActionSummary(
                elements,
                dtype,
                metadata_bytes,
                temporary_bytes,
                tuple(sequence_snapshots),
            )
        finally:
            del active

    plan = _inspect_value(value, resource=resource, active=active)
    elements = plan.nbytes // int(plan.dtype.itemsize)
    return _ActionSummary(
        elements,
        plan.dtype,
        0,
        _action_leaf_temporary_bytes(plan),
        plan,
    )


def _schema_from_plan(plan: _NodePlan) -> _NodeSchema:
    if isinstance(plan, _MappingPlan):
        return _MappingSchema(
            tuple((key, _schema_from_plan(child)) for key, child in plan.children)
        )
    return _LeafSchema(plan.shape, plan.dtype)


def _merge_schema(
    existing: _NodeSchema | None,
    plan: _NodePlan,
    *,
    resource: str,
) -> _NodeSchema:
    if existing is None:
        return _schema_from_plan(plan)
    if isinstance(existing, _MappingSchema) != isinstance(plan, _MappingPlan):
        raise ValueError(f"{resource} mapping structure changes between rows")
    if isinstance(existing, _MappingSchema):
        assert isinstance(plan, _MappingPlan)
        existing_keys = {key for key, _child in existing.children}
        plan_keys = {key for key, _child in plan.children}
        if existing_keys != plan_keys:
            raise ValueError(f"{resource} mapping structure changes between rows")
        plans = dict(plan.children)
        return _MappingSchema(
            tuple(
                (
                    key,
                    _merge_schema(
                        child,
                        plans[key],
                        resource=f"{resource}/{key}",
                    ),
                )
                for key, child in existing.children
            )
        )
    assert isinstance(plan, _LeafPlan)
    if existing.shape != plan.shape:
        raise ValueError(
            f"{resource} shape changes between rows: {existing.shape} and {plan.shape}"
        )
    return _LeafSchema(
        existing.shape, np.dtype(np.result_type(existing.dtype, plan.dtype))
    )


def _declared_numpy_source(plan: _LeafPlan, *, resource: str) -> np.ndarray[Any, Any]:
    source = plan.source
    if not isinstance(source, np.ndarray):
        raise TypeError(f"{resource} declaration is no longer a NumPy array")
    source_shape = tuple(source.shape)
    source_dtype = np.dtype(source.dtype)
    if source_shape != plan.shape or source_dtype != plan.dtype:
        raise ValueError(
            f"{resource} changed during admission: expected shape/dtype "
            f"{plan.shape}/{plan.dtype}, got {source_shape}/{source_dtype}"
        )
    # Strip subclasses before copying so ``__array__`` cannot alter shape or dtype.
    return source.view(np.ndarray)


def _declared_torch_source(plan: _LeafPlan, *, resource: str) -> np.ndarray[Any, Any]:
    source = cast(Any, plan.source)
    source_shape = _normalise_shape(getattr(source, "shape", ()), resource=resource)
    source_dtype = _torch_numpy_dtype(source)
    if source_shape != plan.shape or source_dtype != plan.dtype:
        raise ValueError(
            f"{resource} changed during admission: expected shape/dtype "
            f"{plan.shape}/{plan.dtype}, got {source_shape}/{source_dtype}"
        )
    value = source.detach().cpu().numpy()
    if not isinstance(value, np.ndarray):
        raise TypeError(f"{resource} conversion did not produce an array")
    value_shape = tuple(value.shape)
    value_dtype = np.dtype(value.dtype)
    if value_shape != plan.shape or value_dtype != plan.dtype:
        raise ValueError(
            f"{resource} changed during admission: expected shape/dtype "
            f"{plan.shape}/{plan.dtype}, got {value_shape}/{value_dtype}"
        )
    return value.view(np.ndarray)


def _copy_plan_into(
    plan: _LeafPlan,
    target: np.ndarray[Any, Any],
    *,
    resource: str,
) -> None:
    if tuple(target.shape) != plan.shape:
        raise ValueError(
            f"{resource} target shape changed: expected {plan.shape}, "
            f"got {target.shape}"
        )
    if plan.kind == "sequence":
        children = plan.source
        if not isinstance(children, tuple) or len(children) != plan.shape[0]:
            raise ValueError(f"{resource} sequence snapshot is invalid")
        for index, child in enumerate(children):
            if not isinstance(child, _LeafPlan):
                raise ValueError(f"{resource} sequence snapshot is invalid")
            _copy_plan_into(
                child,
                target[index : index + 1].reshape(child.shape),
                resource=f"{resource}[{index}]",
            )
        return
    if plan.kind == "array":
        source = _declared_numpy_source(plan, resource=resource)
        np.copyto(target, source, casting="unsafe")
        return
    if plan.kind == "torch":
        source = _declared_torch_source(plan, resource=resource)
        np.copyto(target, source, casting="unsafe")
        return
    try:
        target[...] = plan.source
    except (OverflowError, TypeError, ValueError, RuntimeError) as exc:
        raise ValueError(f"{resource} scalar could not be copied") from exc


def _materialize_leaf(
    plan: _LeafPlan,
    *,
    resource: str,
    force_copy: bool = True,
) -> np.ndarray[Any, Any]:
    try:
        if plan.kind == "sequence":
            array = np.empty(plan.shape, dtype=plan.dtype)
            _copy_plan_into(plan, array, resource=resource)
        elif plan.kind == "array":
            source = _declared_numpy_source(plan, resource=resource)
            if not force_copy and bool(source.flags.c_contiguous):
                array = source
            else:
                array = np.empty(plan.shape, dtype=plan.dtype)
                np.copyto(array, source, casting="unsafe")
        elif plan.kind == "torch":
            source = _declared_torch_source(plan, resource=resource)
            if not force_copy and bool(source.flags.c_contiguous):
                array = source
            else:
                array = np.empty(plan.shape, dtype=plan.dtype)
                np.copyto(array, source, casting="unsafe")
        else:
            array = np.empty(plan.shape, dtype=plan.dtype)
            _copy_plan_into(plan, array, resource=resource)
    except (OverflowError, TypeError, ValueError, RuntimeError) as exc:
        raise ValueError(f"{resource} could not be copied") from exc
    if tuple(array.shape) != plan.shape or np.dtype(array.dtype) != plan.dtype:
        raise ValueError(
            f"{resource} changed during admission: expected shape/dtype "
            f"{plan.shape}/{plan.dtype}, got {array.shape}/{array.dtype}"
        )
    return array


def _own_array(
    value: object,
    *,
    dtype: np.dtype[Any],
    force_copy: bool = False,
) -> np.ndarray[Any, Any]:
    """Return a C-contiguous owned array, copying only when required."""

    if isinstance(value, np.ndarray):
        if (
            not force_copy
            and np.dtype(value.dtype) == dtype
            and bool(value.flags.c_contiguous)
            and bool(value.flags.owndata)
        ):
            return value
        return np.array(value, dtype=dtype, copy=True, order="C")
    return np.array(value, dtype=dtype, copy=True, order="C")


def _materialize_node(
    plan: _NodePlan, *, resource: str, force_copy: bool = True
) -> object:
    if isinstance(plan, _MappingPlan):
        return {
            key: _materialize_node(
                child,
                resource=f"{resource}/{key}",
                force_copy=force_copy,
            )
            for key, child in plan.children
        }
    return _materialize_leaf(plan, resource=resource, force_copy=force_copy)


def _materialize_action_snapshot(
    snapshot: object, *, resource: str, force_copy: bool = False
) -> object:
    if isinstance(snapshot, _LeafPlan):
        return _materialize_leaf(snapshot, resource=resource, force_copy=force_copy)
    if isinstance(snapshot, dict):
        return {
            key: _materialize_action_snapshot(
                child,
                resource=f"{resource}/{key}",
                force_copy=force_copy,
            )
            for key, child in snapshot.items()
        }
    if isinstance(snapshot, tuple):
        return tuple(
            _materialize_action_snapshot(
                child,
                resource=f"{resource}[{index}]",
                force_copy=force_copy,
            )
            for index, child in enumerate(snapshot)
        )
    raise ValueError(f"{resource} action snapshot is invalid")


def _iter_schema_leaves(
    schema: _NodeSchema,
    *,
    resource: str,
) -> Sequence[tuple[str, _LeafSchema]]:
    leaves: list[tuple[str, _LeafSchema]] = []
    if isinstance(schema, _LeafSchema):
        leaves.append((resource, schema))
    else:
        for key, child in schema.children:
            leaves.extend(_iter_schema_leaves(child, resource=f"{resource}/{key}"))
    return leaves


def _schema_decoded_bytes(
    schema: _NodeSchema,
    *,
    rows: int,
    resource: str,
) -> int:
    total = 0
    for path, leaf in _iter_schema_leaves(schema, resource=resource):
        total += checked_shape_nbytes(
            (rows, *leaf.shape),
            leaf.dtype,
            resource=path,
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
            max_rank=_MAX_HDF5_RANK,
        )
    return total


def _schema_leaf_specs(
    schema: _NodeSchema,
    *,
    resource: str,
) -> tuple[tuple[str, tuple[int, ...], np.dtype[Any]], ...]:
    """Return declared leaf shapes/dtypes for one write column tree."""

    return tuple(
        (path, leaf.shape, leaf.dtype)
        for path, leaf in _iter_schema_leaves(schema, resource=resource)
    )


def _lookup_node(mapping: object, key: str, *, resource: str) -> object:
    if not isinstance(mapping, Mapping):
        raise ValueError(f"{resource} is not a mapping in an admitted row")
    try:
        return mapping[key]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"{resource} is missing an admitted mapping key") from exc


def _merge_top_schema(
    existing: dict[str, _NodeSchema] | None,
    plans: Mapping[str, _NodePlan],
    *,
    obs_keys: tuple[str, ...],
    resource: str,
) -> dict[str, _NodeSchema]:
    if existing is None:
        return {key: _schema_from_plan(plans[key]) for key in obs_keys}
    return {
        key: _merge_schema(existing[key], plans[key], resource=f"{resource}/{key}")
        for key in obs_keys
    }


def _row_observation_metadata_bytes(obs_keys: tuple[str, ...]) -> int:
    per_mapping = _METADATA_NODE_BYTES + sum(
        _metadata_key_bytes(key) for key in obs_keys
    )
    return 2 * per_mapping


class TrajectoryBuffer:
    """Admit owned trajectory rows and emit one bounded HDF5 demo group."""

    def __init__(
        self,
        *,
        obs_keys: Sequence[str],
        action_dim: int,
        action_space: Any | None = None,
        max_episode_steps: int = 100_000,
        max_decoded_bytes: int = MAX_DECODED_AGGREGATE_BYTES,
        max_batch_bytes: int = MAX_IN_MEMORY_FILE_BYTES,
        max_metadata_bytes: int = MAX_METADATA_AGGREGATE_BYTES,
    ) -> None:
        keys = _bounded_observation_keys(obs_keys)
        if not keys:
            raise ValueError(
                "Robomimic trajectories require at least one observation key"
            )
        self.obs_keys = keys
        self.action_dim = require_bounded_int(
            action_dim,
            resource="action_dim",
            minimum=0,
            maximum=MAX_ARRAY_ELEMENTS,
        )
        self.max_episode_steps = require_bounded_int(
            max_episode_steps,
            resource="max_episode_steps",
            minimum=1,
            maximum=MAX_EVENT_ROWS,
        )
        self.max_decoded_bytes = require_bounded_int(
            max_decoded_bytes,
            resource="max_decoded_bytes",
            minimum=0,
            maximum=MAX_DECODED_AGGREGATE_BYTES,
        )
        self.max_batch_bytes = require_bounded_int(
            max_batch_bytes,
            resource="max_batch_bytes",
            minimum=0,
            maximum=MAX_IN_MEMORY_FILE_BYTES,
        )
        self.max_metadata_bytes = require_bounded_int(
            max_metadata_bytes,
            resource="max_metadata_bytes",
            minimum=0,
            maximum=MAX_METADATA_AGGREGATE_BYTES,
        )
        self.action_space = action_space
        self._action_layout: Any | None = None
        self._owned_action_space: Any | None = None
        action_space_metadata = 0
        if action_space is not None:
            from srb.core.env.runtime.action_layout import ActionLayout

            self._owned_action_space, action_space_metadata = snapshot_action_space(
                action_space,
                max_metadata_bytes=self.max_metadata_bytes,
            )
            self.action_space = self._owned_action_space
            self._action_layout = ActionLayout.from_structured_space(
                self._owned_action_space
            )
            if self._action_layout.flat_dim != self.action_dim:
                raise ValueError(
                    f"action space has dimension {self._action_layout.flat_dim}, "
                    f"recorder action_dim is {self.action_dim}"
                )

        metadata_bytes = _METADATA_NODE_BYTES
        for key in keys:
            metadata_bytes += _metadata_key_bytes(key)
        if action_space is not None:
            metadata_bytes += action_space_metadata
        if metadata_bytes > self.max_metadata_bytes:
            raise ResourceLimitError(
                "trajectory metadata",
                limit=self.max_metadata_bytes,
                observed=metadata_bytes,
            )
        self._metadata_bytes = metadata_bytes
        self._rows: list[_Row] = []
        self._obs_schema: dict[str, _NodeSchema] | None = None
        self._next_obs_schema: dict[str, _NodeSchema] | None = None
        self._action_schema: _LeafSchema | None = None
        self._decoded_nbytes = 0

    @property
    def num_steps(self) -> int:
        """Number of admitted rows retained by this buffer."""

        return len(self._rows)

    @property
    def decoded_nbytes(self) -> int:
        """Decoded bytes in the final promoted arrays for all public columns."""

        return self._decoded_nbytes

    @property
    def metadata_nbytes(self) -> int:
        """Retained mapping and key bookkeeping bytes admitted by this buffer."""

        return self._metadata_bytes

    @property
    def write_temporary_nbytes(self) -> int:
        """Maximum NumPy batch allocation needed by the current write plan."""

        if not self._rows:
            return 0
        if (
            self._obs_schema is None
            or self._next_obs_schema is None
            or self._action_schema is None
        ):
            raise RuntimeError("trajectory schemas are unavailable")
        _batch_rows, largest_batch = self._batch_plan(
            self._obs_schema,
            self._next_obs_schema,
            self._action_schema,
            rows=len(self._rows),
        )
        return largest_batch

    def append(
        self,
        obs: Mapping[str, object],
        action: object,
        reward: object,
        done: object,
        next_obs: Mapping[str, object],
        *,
        terminated: object | None = None,
        truncated: object | None = None,
    ) -> None:
        """Inspect, admit and snapshot one complete transition row."""

        next_step = len(self._rows) + 1
        if next_step > self.max_episode_steps:
            raise ResourceLimitError(
                "max_episode_steps",
                limit=self.max_episode_steps,
                observed=next_step,
            )
        if not isinstance(obs, Mapping) or not isinstance(next_obs, Mapping):
            raise TypeError("obs and next_obs must be mappings")
        missing_obs = [key for key in self.obs_keys if key not in obs]
        missing_next_obs = [key for key in self.obs_keys if key not in next_obs]
        if missing_obs or missing_next_obs:
            raise ValueError(
                "missing required observation keys: "
                f"obs={missing_obs}, next_obs={missing_next_obs}"
            )

        flag_temporary_bytes = 0
        for name, value in (
            ("done", done),
            ("terminated", terminated),
            ("truncated", truncated),
        ):
            if value is None:
                continue
            flag_plan = _inspect_value(value, resource=name)
            if flag_plan.shape != ():
                raise ValueError(f"{name} must be a scalar, got {flag_plan.shape}")
            flag_temporary_bytes += flag_plan.temporary_bytes
        resolved_done = self._coerce_flag(done, resource="done")
        resolved_terminated = (
            resolved_done
            if terminated is None
            else self._coerce_flag(terminated, resource="terminated")
        )
        resolved_truncated = (
            False
            if truncated is None
            else self._coerce_flag(truncated, resource="truncated")
        )
        if resolved_done != (resolved_terminated or resolved_truncated):
            raise ValueError(
                "inconsistent termination flags: done="
                f"{resolved_done}, terminated={resolved_terminated}, "
                f"truncated={resolved_truncated}; done must equal "
                "(terminated or truncated)"
            )

        obs_plans: dict[str, _NodePlan] = {}
        next_obs_plans: dict[str, _NodePlan] = {}
        # Every admitted row retains its own top-level mappings and keys.  The
        # base observation keys are charged once; row structures accumulate.
        metadata_bytes = self._metadata_bytes + _row_observation_metadata_bytes(
            self.obs_keys
        )
        if metadata_bytes > self.max_metadata_bytes:
            raise ResourceLimitError(
                "trajectory metadata",
                limit=self.max_metadata_bytes,
                observed=metadata_bytes,
            )
        for key in self.obs_keys:
            plan, plan_metadata = _inspect_observation_node(
                obs[key],
                resource=f"obs/{key}",
                active=frozenset(),
                metadata_limit=self.max_metadata_bytes - metadata_bytes,
            )
            obs_plans[key] = plan
            metadata_bytes += plan_metadata
            next_plan, next_metadata = _inspect_observation_node(
                next_obs[key],
                resource=f"next_obs/{key}",
                active=frozenset(),
                metadata_limit=self.max_metadata_bytes - metadata_bytes,
            )
            next_obs_plans[key] = next_plan
            metadata_bytes += next_metadata

        action_summary = self._inspect_action(
            action,
            metadata_limit=self.max_metadata_bytes - metadata_bytes,
        )
        metadata_bytes += action_summary.metadata_bytes
        if metadata_bytes > self.max_metadata_bytes:
            raise ResourceLimitError(
                "trajectory metadata",
                limit=self.max_metadata_bytes,
                observed=metadata_bytes,
            )
        if action_summary.elements != self.action_dim:
            raise ValueError(
                f"action has {action_summary.elements} values but recorder "
                f"action_dim is {self.action_dim}"
            )

        reward_plan = _inspect_value(reward, resource="reward")
        if reward_plan.shape != ():
            raise ValueError(f"reward must be a scalar, got {reward_plan.shape}")
        if reward_plan.dtype.kind not in "biuf":
            raise TypeError(f"reward has unsupported dtype {reward_plan.dtype}")
        obs_schema = _merge_top_schema(
            self._obs_schema,
            obs_plans,
            obs_keys=self.obs_keys,
            resource="obs",
        )
        next_obs_schema = _merge_top_schema(
            self._next_obs_schema,
            next_obs_plans,
            obs_keys=self.obs_keys,
            resource="next_obs",
        )
        action_schema = _LeafSchema(
            (self.action_dim,),
            action_summary.dtype
            if self._action_schema is None
            else np.dtype(
                np.result_type(self._action_schema.dtype, action_summary.dtype)
            ),
        )
        decoded_nbytes = self._decoded_bytes(
            obs_schema,
            next_obs_schema,
            action_schema,
            rows=next_step,
        )
        if decoded_nbytes > self.max_decoded_bytes:
            raise ResourceLimitError(
                "trajectory decoded bytes",
                limit=self.max_decoded_bytes,
                observed=decoded_nbytes,
            )
        _batch_rows, write_temporary_bytes = self._batch_plan(
            obs_schema,
            next_obs_schema,
            action_schema,
            rows=next_step,
        )
        conversion_temporary_bytes = sum(
            _plan_temporary_bytes(plan) for plan in obs_plans.values()
        )
        conversion_temporary_bytes += sum(
            _plan_temporary_bytes(plan) for plan in next_obs_plans.values()
        )
        conversion_temporary_bytes += action_summary.temporary_bytes
        conversion_temporary_bytes += reward_plan.temporary_bytes + flag_temporary_bytes
        peak_decoded_bytes = (
            decoded_nbytes + conversion_temporary_bytes + write_temporary_bytes
        )
        if peak_decoded_bytes > self.max_decoded_bytes:
            raise ResourceLimitError(
                "trajectory decoded bytes",
                limit=self.max_decoded_bytes,
                observed=peak_decoded_bytes,
            )

        # Conversion happens only after every declaration, promoted schema and
        # aggregate ceiling has passed.  Any conversion failure leaves all
        # existing schemas, rows and counters untouched.
        owned_obs = {
            key: _materialize_node(plan, resource=f"obs/{key}")
            for key, plan in obs_plans.items()
        }
        owned_next_obs = {
            key: _materialize_node(plan, resource=f"next_obs/{key}")
            for key, plan in next_obs_plans.items()
        }
        owned_action = self._materialize_action(action_summary)
        if np.dtype(owned_action.dtype) != action_summary.dtype:
            raise ValueError(
                "action dtype changed during admission: "
                f"expected {action_summary.dtype}, got {owned_action.dtype}"
            )
        owned_reward = np.asarray(float(cast(Any, reward)), dtype=np.float64)
        row = _Row(
            obs=owned_obs,
            next_obs=owned_next_obs,
            action=owned_action,
            reward=owned_reward,
            done=resolved_done,
            terminated=resolved_terminated,
            truncated=resolved_truncated,
        )

        self._rows.append(row)
        self._obs_schema = obs_schema
        self._next_obs_schema = next_obs_schema
        self._action_schema = action_schema
        self._decoded_nbytes = decoded_nbytes
        self._metadata_bytes = metadata_bytes

    @staticmethod
    def _coerce_flag(value: object, *, resource: str) -> bool:
        try:
            return bool(value)
        except (TypeError, ValueError, RuntimeError) as exc:
            raise ValueError(f"{resource} must be a scalar boolean-like value") from exc

    def _inspect_action(
        self,
        action: object,
        *,
        metadata_limit: int = MAX_METADATA_AGGREGATE_BYTES,
    ) -> _ActionSummary:
        if self._action_layout is None:
            return _inspect_untyped_action(
                action,
                resource="action",
                metadata_limit=metadata_limit,
            )

        if self._action_layout.kind != "leaf" and (
            _is_torch_tensor(action) or isinstance(action, np.ndarray)
        ):
            plan = _inspect_value(action, resource="action")
            if plan.shape != (self.action_dim,):
                raise ValueError(
                    f"action shape must be {(self.action_dim,)}, got {plan.shape}"
                )
            return _ActionSummary(
                plan.nbytes // int(plan.dtype.itemsize),
                plan.dtype,
                0,
                _action_leaf_temporary_bytes(plan),
                plan,
                True,
            )
        return _inspect_action_node(
            action,
            self._owned_action_space,
            resource="action",
            active=frozenset(),
            metadata_limit=metadata_limit,
        )

    def _materialize_action(
        self, action_summary: _ActionSummary
    ) -> np.ndarray[Any, Any]:
        from srb.core.env.runtime.action_space import flatten_action

        snapshot = _summary_snapshot(action_summary)
        try:
            if action_summary.already_flat:
                if not isinstance(snapshot, _LeafPlan):
                    raise ValueError("flat action snapshot is invalid")
                array = _materialize_leaf(
                    snapshot,
                    resource="action",
                    force_copy=True,
                )
            else:
                owned = _materialize_action_snapshot(
                    snapshot,
                    resource="action",
                )
                if self._action_layout is None:
                    flattened = flatten_action(owned, None)
                else:
                    flattened = self._action_layout.flatten(owned)
                array = _own_array(flattened, dtype=np.dtype(flattened.dtype))
        except (TypeError, ValueError, RuntimeError) as exc:
            raise ValueError("action could not be flattened") from exc
        if tuple(array.shape) != (self.action_dim,):
            raise ValueError(
                f"flattened action shape must be {(self.action_dim,)}, got {array.shape}"
            )
        _validate_dtype(np.dtype(array.dtype), resource="action")
        return array

    def _decoded_bytes(
        self,
        obs_schema: Mapping[str, _NodeSchema],
        next_obs_schema: Mapping[str, _NodeSchema],
        action_schema: _LeafSchema,
        *,
        rows: int,
    ) -> int:
        total = 0
        for key in self.obs_keys:
            total += _schema_decoded_bytes(
                obs_schema[key], rows=rows, resource=f"obs/{key}"
            )
            total += _schema_decoded_bytes(
                next_obs_schema[key], rows=rows, resource=f"next_obs/{key}"
            )
        total += _schema_decoded_bytes(action_schema, rows=rows, resource="actions")
        for name, dtype in (
            ("rewards", np.dtype(np.float64)),
            ("dones", np.dtype(np.bool_)),
            ("terminated", np.dtype(np.bool_)),
            ("truncated", np.dtype(np.bool_)),
        ):
            total += checked_shape_nbytes(
                (rows,),
                dtype,
                resource=name,
                max_elements=MAX_ARRAY_ELEMENTS,
                max_bytes=MAX_ARRAY_BYTES,
                max_rank=_MAX_HDF5_RANK,
            )
        return total

    def _batch_plan(
        self,
        obs_schema: Mapping[str, _NodeSchema],
        next_obs_schema: Mapping[str, _NodeSchema],
        action_schema: _LeafSchema,
        *,
        rows: int,
    ) -> tuple[int, int]:
        """Plan one-column-at-a-time writes before touching the HDF5 group."""

        specs: list[tuple[str, tuple[int, ...], np.dtype[Any]]] = []
        for key in self.obs_keys:
            specs.extend(_schema_leaf_specs(obs_schema[key], resource=f"obs/{key}"))
            specs.extend(
                _schema_leaf_specs(next_obs_schema[key], resource=f"next_obs/{key}")
            )
        specs.append(("actions", action_schema.shape, action_schema.dtype))
        specs.extend(
            (
                name,
                (),
                dtype,
            )
            for name, dtype in (
                ("rewards", np.dtype(np.float64)),
                ("dones", np.dtype(np.bool_)),
                ("terminated", np.dtype(np.bool_)),
                ("truncated", np.dtype(np.bool_)),
            )
        )

        one_row_bytes: list[tuple[str, int]] = []
        for resource, shape, dtype in specs:
            one_row_bytes.append(
                (
                    resource,
                    checked_shape_nbytes(
                        (1, *shape),
                        dtype,
                        resource=resource,
                        max_elements=MAX_ARRAY_ELEMENTS,
                        max_bytes=MAX_ARRAY_BYTES,
                        max_rank=_MAX_HDF5_RANK,
                    ),
                )
            )
        largest_row = max((size for _resource, size in one_row_bytes), default=0)
        batch_rows = min(rows, _MAX_BATCH_ROWS)
        if largest_row > 0:
            batch_rows = min(
                batch_rows,
                max(1, self.max_batch_bytes // largest_row),
            )
        batch_rows = max(1, batch_rows)

        largest_batch = 0
        for resource, shape, dtype in specs:
            size = checked_shape_nbytes(
                (batch_rows, *shape),
                dtype,
                resource=resource,
                max_elements=MAX_ARRAY_ELEMENTS,
                max_bytes=MAX_ARRAY_BYTES,
                max_rank=_MAX_HDF5_RANK,
            )
            if size > largest_batch:
                largest_batch = size
        if largest_batch > self.max_batch_bytes:
            raise ResourceLimitError(
                "trajectory batch bytes",
                limit=self.max_batch_bytes,
                observed=largest_batch,
            )
        return batch_rows, largest_batch

    def _batch_rows(self) -> int:
        if not self._rows:
            return 0
        if (
            self._obs_schema is None
            or self._next_obs_schema is None
            or self._action_schema is None
        ):
            raise RuntimeError("trajectory schemas are unavailable")
        batch_rows, _largest_batch = self._batch_plan(
            self._obs_schema,
            self._next_obs_schema,
            self._action_schema,
            rows=len(self._rows),
        )
        return batch_rows

    def write(self, demo_group: Any) -> None:
        """Write admitted rows as bounded batches into ``demo_group``."""

        if not self._rows:
            raise ValueError("cannot write an empty trajectory")
        if (
            self._obs_schema is None
            or self._next_obs_schema is None
            or self._action_schema is None
        ):
            raise RuntimeError("trajectory schemas are unavailable")
        _batch_rows, _largest_batch = self._batch_plan(
            self._obs_schema,
            self._next_obs_schema,
            self._action_schema,
            rows=len(self._rows),
        )

        obs_group = demo_group.create_group("obs")
        for key in self.obs_keys:
            self._write_node(
                obs_group,
                key,
                self._obs_schema[key],
                side="obs",
            )
        next_obs_group = demo_group.create_group("next_obs")
        for key in self.obs_keys:
            self._write_node(
                next_obs_group,
                key,
                self._next_obs_schema[key],
                side="next_obs",
            )
        self._write_leaf_column(demo_group, "actions", self._action_schema, "action")
        self._write_primitive_column(
            demo_group, "rewards", np.dtype(np.float64), lambda row: row.reward
        )
        self._write_primitive_column(
            demo_group,
            "dones",
            np.dtype(np.bool_),
            lambda row: np.asarray(row.done, dtype=np.bool_),
        )
        self._write_primitive_column(
            demo_group,
            "terminated",
            np.dtype(np.bool_),
            lambda row: np.asarray(row.terminated, dtype=np.bool_),
        )
        self._write_primitive_column(
            demo_group,
            "truncated",
            np.dtype(np.bool_),
            lambda row: np.asarray(row.truncated, dtype=np.bool_),
        )

    def _write_node(
        self,
        parent: Any,
        key: str,
        schema: _NodeSchema,
        *,
        side: Literal["obs", "next_obs"],
        path: tuple[str, ...] | None = None,
    ) -> None:
        current_path = (key,) if path is None else (*path, key)
        if isinstance(schema, _MappingSchema):
            group = parent.create_group(key)
            for child_key, child_schema in schema.children:
                self._write_node(
                    group,
                    child_key,
                    child_schema,
                    side=side,
                    path=current_path,
                )
            return
        resource = f"{side}/{'/'.join(current_path)}"
        dataset = parent.create_dataset(
            key,
            shape=(len(self._rows), *schema.shape),
            dtype=schema.dtype,
        )
        self._fill_dataset(
            dataset,
            schema,
            lambda row: self._row_observation_value(row, side=side, path=current_path),
            resource=resource,
        )

    @staticmethod
    def _row_observation_value(
        row: _Row,
        *,
        side: Literal["obs", "next_obs"],
        path: tuple[str, ...],
    ) -> object:
        value: object = row.obs if side == "obs" else row.next_obs
        for key in path:
            value = _lookup_node(value, key, resource=f"{side}/{key}")
        return value

    def _write_leaf_column(
        self,
        parent: Any,
        name: str,
        schema: _LeafSchema,
        side: Literal["action"],
    ) -> None:
        dataset = parent.create_dataset(
            name,
            shape=(len(self._rows), *schema.shape),
            dtype=schema.dtype,
        )
        self._fill_dataset(
            dataset,
            schema,
            lambda row: row.action,
            resource=side,
        )

    def _write_primitive_column(
        self,
        parent: Any,
        name: str,
        dtype: np.dtype[Any],
        value_getter: Any,
    ) -> None:
        schema = _LeafSchema((), dtype)
        dataset = parent.create_dataset(
            name,
            shape=(len(self._rows),),
            dtype=dtype,
        )
        self._fill_dataset(dataset, schema, value_getter, resource=name)

    def _fill_dataset(
        self,
        dataset: Any,
        schema: _LeafSchema,
        value_getter: Any,
        *,
        resource: str,
    ) -> None:
        batch_rows = self._batch_rows()
        if batch_rows <= 0:
            raise RuntimeError("trajectory batch size is unavailable")
        for start in range(0, len(self._rows), batch_rows):
            stop = min(start + batch_rows, len(self._rows))
            count = stop - start
            checked_shape_nbytes(
                (count, *schema.shape),
                schema.dtype,
                resource=resource,
                max_elements=MAX_ARRAY_ELEMENTS,
                max_bytes=self.max_batch_bytes,
                max_rank=_MAX_HDF5_RANK,
            )
            batch = np.empty((count, *schema.shape), dtype=schema.dtype)
            for offset, row in enumerate(self._rows[start:stop]):
                value = value_getter(row)
                try:
                    batch[offset] = value
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"{resource} row cannot be written") from exc
            dataset[start:stop] = batch
            del batch


__all__ = ["TrajectoryBuffer", "snapshot_action_space"]
