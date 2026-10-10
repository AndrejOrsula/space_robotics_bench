"""Validate bounded canonical transitions before demonstration replay.

Every row needs an explicit next observation and Gymnasium termination flags.
Validate the complete input before mutating a framework-owned replay buffer;
this module has no learning-framework dependency.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, cast

import numpy as np

from srb.datasets.jsonl import (
    MAX_JSONL_LINE_BYTES,
    MAX_JSONL_NESTING,
    MAX_JSONL_NODES,
    srb_jsonl_episode_paths,
    validate_srb_jsonl_row,
)
from srb.datasets.manifest import (
    DatasetManifest,
    DatasetSnapshot,
    open_dataset_snapshot,
)
from srb.datasets.metadata_validation import (
    DEMO_IO_CONTRACT_VERSION,
    validate_demo_io_contract,
)
from srb.datasets.schema import StorageFormat
from srb.utils.dataset_manifest import demo_content_fingerprint
from srb.utils.json_types import JsonDecodeBudget
from srb.utils.publication import ArtifactPublication, PublicationError
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_JSONL_ROW_NODES,
    MAX_METADATA_AGGREGATE_BYTES,
    ByteBudget,
    ResourceLimitError,
    checked_shape_nbytes,
    require_bounded_int,
)

# Keep the row cap within the array-element ceiling so every accepted count can
# be represented by bounded transition values.
MAX_DEMO_TRANSITIONS = min(MAX_ARRAY_ELEMENTS, 100_000_000)
MAX_DEMO_SOURCE_BINDING_BYTES = 1 * (1 << 20)

# Callers admit real numeric dtypes before iterating scalar values.
_RealScalar = int | float | np.bool_ | np.integer[Any] | np.floating[Any]

# Fixed charges bound retained plan objects, containers, and references while
# declaration bookkeeping builds the ownership plan. Payloads use the array
# budget. Charges also cover shape metadata and source snapshots held alongside
# plans; slotted plans alone occupy 48–80 bytes on CPython 3.12.
_DECLARATION_PLAN_BYTES = 128
_DECLARATION_TUPLE_BYTES = 64
_DECLARATION_REFERENCE_BYTES = 16
_DECLARATION_ENTRY_BYTES = 128
_DECLARATION_SHAPE_BYTES = 64
_DECLARATION_SHAPE_REFERENCE_BYTES = 16


@dataclass(frozen=True, slots=True)
class _NumericPlan:
    shape: tuple[int, ...]
    dtype: np.dtype[Any]
    nbytes: int
    source: object
    children: tuple[_NumericPlan, ...] = ()
    dtype_known: bool = True


@dataclass(frozen=True, slots=True)
class _MappingPlan:
    entries: tuple[tuple[str, object], ...]


@dataclass(frozen=True, slots=True)
class _ActionPlan:
    source: _NumericPlan
    target_nbytes: int


class _DeclarationBudget:
    """Bound the retained shape plan before adding child entries."""

    __slots__ = ("metadata_bytes", "nodes")

    def __init__(self) -> None:
        self.nodes = 0
        self.metadata_bytes = 0

    @staticmethod
    def _diagnostic(value: object) -> str:
        return _bounded_context(value)

    def retain(self, amount: int, *, context: object) -> None:
        """Charge retained plan metadata before growing an owned container."""

        if amount < 0:
            raise ValueError("declaration metadata charge must be non-negative")
        observed = self.metadata_bytes + amount
        if observed > MAX_METADATA_AGGREGATE_BYTES:
            raise ResourceLimitError(
                f"{self._diagnostic(context)} metadata",
                limit=MAX_METADATA_AGGREGATE_BYTES,
                observed=observed,
            )
        self.metadata_bytes = observed

    def visit(self, *, context: str, depth: int) -> None:
        if depth > MAX_JSONL_NESTING:
            raise ResourceLimitError(
                f"{self._diagnostic(context)} JSON nesting",
                limit=MAX_JSONL_NESTING,
                observed=depth,
            )
        self.nodes += 1
        if self.nodes > MAX_JSONL_NODES:
            raise ResourceLimitError(
                f"{self._diagnostic(context)} JSON nodes",
                limit=MAX_JSONL_NODES,
                observed=self.nodes,
            )

    def retain_key(self, key: str, *, context: str) -> None:
        key_overhead = 64
        remaining = MAX_METADATA_AGGREGATE_BYTES - self.metadata_bytes - key_overhead
        key_bytes = _utf8_length(key, limit=max(0, remaining))
        observed = self.metadata_bytes + key_overhead + key_bytes
        if observed > MAX_METADATA_AGGREGATE_BYTES:
            raise ResourceLimitError(
                f"{self._diagnostic(context)} metadata",
                limit=MAX_METADATA_AGGREGATE_BYTES,
                observed=observed,
            )
        self.metadata_bytes = observed


def _bounded_context(value: object) -> str:
    text = value if isinstance(value, str) else str(value)
    if len(text) <= 256:
        return text
    return text[:240] + "...<truncated>"


def _path_context(context: object, suffix: object) -> str:
    prefix = _bounded_context(context)
    clipped_suffix = _bounded_context(suffix)
    return _bounded_context(prefix + clipped_suffix)


def _key_context(context: object, key: str) -> str:
    """Build a bounded mapping path without copying a complete caller key."""

    return _path_context(context, "." + _bounded_context(key))


def _bounded_key_text(key: str) -> str:
    return repr(_bounded_context(key))


def _utf8_length(value: str, *, limit: int | None = None) -> int:
    """Measure UTF-8 bytes without allocating an encoded copy."""

    if value.isascii():
        length = len(value)
        if limit is not None and length > limit:
            return limit + 1
        return length
    encoded_length = 0
    for character in value:
        codepoint = ord(character)
        if codepoint <= 0x7F:
            encoded_length += 1
        elif codepoint <= 0x7FF:
            encoded_length += 2
        elif codepoint <= 0xFFFF:
            encoded_length += 3
        else:
            encoded_length += 4
        if limit is not None and encoded_length > limit:
            return encoded_length
    return encoded_length


def _numeric_error(context: object) -> ValueError:
    return ValueError(
        f"{_bounded_context(context)} must be a rectangular numeric array"
    )


def _require_finite_scalar(value: _RealScalar, *, context: object) -> None:
    try:
        finite = bool(np.isfinite(value))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            f"{_bounded_context(context)} must contain only finite numbers"
        ) from exc
    if not finite:
        raise ValueError(
            f"{_bounded_context(context)} must contain only finite numbers"
        )


def _require_float32_scalar(value: _RealScalar, *, context: object) -> None:
    try:
        with np.errstate(over="ignore", invalid="ignore"):
            converted = np.float32(value)
        finite = bool(np.isfinite(converted))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            f"{_bounded_context(context)} must be representable as float32"
        ) from exc
    if not finite:
        raise ValueError(f"{_bounded_context(context)} is outside finite float32 range")


def _require_finite_array(value: np.ndarray[Any, Any], *, context: object) -> None:
    if not np.issubdtype(value.dtype, np.floating):
        return
    for item in cast(Iterator[_RealScalar], value.flat):
        _require_finite_scalar(item, context=context)


def _validate_numeric_dtype(
    dtype: np.dtype[Any], *, context: object, allow_bool: bool = False
) -> None:
    if dtype.kind not in ("iuf" if not allow_bool else "biuf"):
        if dtype.kind == "c":
            raise ValueError(
                f"{_bounded_context(context)} must contain real numeric values"
            )
        raise _numeric_error(context)


def _python_scalar_dtype(
    value: object, *, context: object, allow_bool: bool = False
) -> np.dtype[Any]:
    if isinstance(value, (bool, np.bool_)):
        if allow_bool:
            return np.dtype(np.bool_)
        raise ValueError(f"{_bounded_context(context)} must be a finite numeric array")
    if isinstance(value, int) and not isinstance(value, bool):
        if -(1 << 63) <= value <= (1 << 63) - 1:
            return np.dtype(np.int64)
        if 0 <= value <= (1 << 64) - 1:
            return np.dtype(np.uint64)
        raise _numeric_error(context)
    if isinstance(value, float):
        _require_finite_scalar(value, context=context)
        return np.dtype(np.float64)
    if isinstance(value, np.integer):
        return np.dtype(value.dtype)
    if isinstance(value, np.floating):
        _require_finite_scalar(value, context=context)
        return np.dtype(value.dtype)
    raise ValueError(f"{_bounded_context(context)} must be a finite numeric array")


def _numeric_plan(
    shape: tuple[int, ...],
    dtype: np.dtype[Any],
    *,
    context: object,
    budget: _DeclarationBudget,
    allow_bool: bool = False,
    source: object,
    children: tuple[_NumericPlan, ...] = (),
    dtype_known: bool = True,
) -> _NumericPlan:
    _validate_numeric_dtype(dtype, context=context, allow_bool=allow_bool)
    try:
        nbytes = checked_shape_nbytes(
            shape,
            dtype,
            resource=_bounded_context(context),
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
            max_rank=MAX_JSONL_NESTING,
        )
    except ResourceLimitError:
        raise
    except (TypeError, ValueError) as exc:
        raise ValueError(str(exc)) from exc
    budget.retain(
        _DECLARATION_PLAN_BYTES
        + _DECLARATION_SHAPE_BYTES
        + _DECLARATION_SHAPE_REFERENCE_BYTES * len(shape)
        + _DECLARATION_REFERENCE_BYTES,
        context=context,
    )
    return _NumericPlan(
        shape=shape,
        dtype=dtype,
        nbytes=nbytes,
        source=source,
        children=children,
        dtype_known=dtype_known,
    )


def _declare_numeric(
    value: object,
    *,
    context: str,
    budget: _DeclarationBudget,
    depth: int,
    active: set[int],
    require_float32: bool = False,
    allow_bool: bool = False,
) -> _NumericPlan:
    if isinstance(value, np.ndarray):
        budget.visit(context=context, depth=depth)
        dtype = np.dtype(value.dtype)
        _validate_numeric_dtype(dtype, context=context, allow_bool=allow_bool)
        plan = _numeric_plan(
            tuple(int(dim) for dim in value.shape),
            dtype,
            context=context,
            budget=budget,
            allow_bool=allow_bool,
            source=value,
        )
        _require_finite_array(value, context=context)
        if require_float32:
            for item in cast(Iterator[_RealScalar], value.flat):
                _require_float32_scalar(item, context=context)
        return plan

    if isinstance(value, Mapping):
        raise _numeric_error(context)

    if isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{_bounded_context(context)} must be a finite numeric array")

    if isinstance(value, (bool, np.bool_, int, np.integer, float, np.floating)):
        budget.visit(context=context, depth=depth)
        dtype = _python_scalar_dtype(
            value,
            context=context,
            allow_bool=allow_bool,
        )
        if require_float32:
            _require_float32_scalar(value, context=context)
        return _numeric_plan(
            (),
            dtype,
            context=context,
            budget=budget,
            allow_bool=allow_bool,
            source=value,
        )

    if not isinstance(value, Sequence):
        raise _numeric_error(context)

    identity = id(value)
    if identity in active:
        raise ValueError(f"{_bounded_context(context)} contains a cyclic numeric value")
    active.add(identity)
    try:
        budget.visit(context=context, depth=depth)
        try:
            length = len(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise _numeric_error(context) from exc
        if length > MAX_ARRAY_ELEMENTS:
            raise ResourceLimitError(
                f"{_bounded_context(context)} elements",
                limit=MAX_ARRAY_ELEMENTS,
                observed=length,
            )
        if length == 0:
            budget.retain(_DECLARATION_TUPLE_BYTES, context=context)
            return _numeric_plan(
                (0,),
                np.dtype(np.float64),
                context=context,
                budget=budget,
                allow_bool=allow_bool,
                source=(),
                dtype_known=False,
            )

        try:
            first_context = _path_context(context, "[0]")
            first = _declare_numeric(
                value[0],
                context=first_context,
                budget=budget,
                depth=depth + 1,
                active=active,
                require_float32=require_float32,
                allow_bool=True,
            )
        except (IndexError, KeyError, TypeError) as exc:
            raise _numeric_error(context) from exc
        child_shape = first.shape
        dtype = first.dtype
        dtype_known = first.dtype_known
        budget.retain(_DECLARATION_ENTRY_BYTES, context=context)
        children: list[_NumericPlan] = [first]
        for index in range(1, length):
            try:
                child_context = _path_context(context, f"[{index}]")
                child = _declare_numeric(
                    value[index],
                    context=child_context,
                    budget=budget,
                    depth=depth + 1,
                    active=active,
                    require_float32=require_float32,
                    allow_bool=True,
                )
            except (IndexError, KeyError, TypeError) as exc:
                raise _numeric_error(context) from exc
            if child.shape != child_shape:
                raise _numeric_error(context)
            if child.dtype_known:
                if not dtype_known:
                    dtype = child.dtype
                    dtype_known = True
                else:
                    try:
                        dtype = np.result_type(dtype, child.dtype)
                    except (TypeError, ValueError) as exc:
                        raise _numeric_error(context) from exc
            _validate_numeric_dtype(
                np.dtype(dtype),
                context=context,
                allow_bool=True,
            )
            budget.retain(_DECLARATION_ENTRY_BYTES, context=context)
            children.append(child)
        final_dtype = np.dtype(dtype)
        budget.retain(
            _DECLARATION_TUPLE_BYTES * 2
            + _DECLARATION_REFERENCE_BYTES * len(children) * 2,
            context=context,
        )
        source_snapshot = tuple(child.source for child in children)
        child_snapshot = tuple(children)
        return _numeric_plan(
            (length, *child_shape),
            final_dtype,
            context=context,
            budget=budget,
            allow_bool=allow_bool,
            source=source_snapshot,
            children=child_snapshot,
            dtype_known=dtype_known,
        )
    finally:
        active.remove(identity)


def _declare_tree(
    value: object,
    *,
    context: str,
    budget: _DeclarationBudget,
    depth: int,
    active: set[int],
) -> _MappingPlan:
    if not isinstance(value, Mapping):
        raise TypeError(f"{_bounded_context(context)} must be a non-empty mapping")
    identity = id(value)
    if identity in active:
        raise ValueError(f"{_bounded_context(context)} contains a cyclic mapping")
    active.add(identity)
    try:
        budget.visit(context=context, depth=depth)
        try:
            entry_count = require_bounded_int(
                len(value),
                resource=f"{_bounded_context(context)} mapping entries",
                minimum=0,
                maximum=MAX_JSONL_NODES,
            )
        except TypeError as exc:
            raise TypeError(
                f"{_bounded_context(context)} mapping length is not bounded"
            ) from exc
        if entry_count == 0:
            raise TypeError(f"{_bounded_context(context)} must be a non-empty mapping")
        entries: list[tuple[str, object]] = []
        seen: set[str] = set()
        observed = 0
        for key, item in value.items():
            observed += 1
            if observed > entry_count:
                raise ValueError(
                    f"{_bounded_context(context)} mapping length changed during "
                    f"inspection: declared {entry_count}, observed {observed}"
                )
            if not isinstance(key, str):
                raise ValueError(f"{_bounded_context(context)} keys must be strings")
            if key in seen:
                raise ValueError(
                    f"{_bounded_context(context)} contains duplicate key "
                    f"{_bounded_key_text(key)}"
                )
            child_context = _key_context(context, key)
            budget.visit(context=f"{child_context} key", depth=depth + 1)
            budget.retain_key(key, context=child_context)
            budget.retain(_DECLARATION_ENTRY_BYTES, context=child_context)
            seen.add(key)
            if isinstance(item, Mapping):
                child: object = _declare_tree(
                    item,
                    context=child_context,
                    budget=budget,
                    depth=depth + 1,
                    active=active,
                )
            else:
                child = _declare_numeric(
                    item,
                    context=child_context,
                    budget=budget,
                    depth=depth + 1,
                    active=active,
                    allow_bool=True,
                )
            entries.append((key, child))
        if observed != entry_count:
            raise ValueError(
                f"{_bounded_context(context)} mapping length changed during "
                f"inspection: declared {entry_count}, observed {observed}"
            )
        budget.retain(
            _DECLARATION_PLAN_BYTES
            + _DECLARATION_TUPLE_BYTES
            + _DECLARATION_REFERENCE_BYTES * len(entries) * 2,
            context=context,
        )
        return _MappingPlan(tuple(entries))
    finally:
        active.remove(identity)


def _plan_shape(plan: object) -> object:
    if isinstance(plan, _NumericPlan):
        return plan.shape
    if isinstance(plan, _MappingPlan):
        return tuple((key, _plan_shape(child)) for key, child in sorted(plan.entries))
    raise TypeError(f"unexpected declaration {type(plan).__name__}")


def _array_is_immutable(value: np.ndarray[Any, Any]) -> bool:
    if value.flags.writeable:
        return False
    current: object = value
    seen: set[int] = set()
    while id(current) not in seen:
        seen.add(id(current))
        if isinstance(current, np.ndarray):
            current = current.base
            if current is None:
                return False
            continue
        if isinstance(current, memoryview):
            if isinstance(current.obj, bytes):
                return True
            current = current.obj
            continue
        return isinstance(current, bytes)
    return False


def _readonly_array(value: np.ndarray[Any, Any]) -> np.ndarray[Any, Any]:
    """Detach an array from parser-owned storage and make it immutable."""

    if _array_is_immutable(value):
        # The view owns its shape/dtype/stride metadata; its immutable backing
        # stays shared, but caller metadata changes cannot alter the view.
        result: np.ndarray[Any, Any] = value.view(np.ndarray)
        result.flags.writeable = False
        return result
    # A bytes-backed array cannot have its write flag re-enabled by a caller.
    backing = value.tobytes(order="C")
    owned_result: np.ndarray[Any, Any] = np.frombuffer(
        backing, dtype=value.dtype
    ).reshape(value.shape)
    return owned_result


def _numeric_array_usage(plan: _NumericPlan) -> tuple[int, int]:
    """Return retained bytes and transient bytes for one numeric plan."""

    retained = plan.nbytes
    if isinstance(plan.source, np.ndarray) and _array_is_immutable(plan.source):
        peak = retained
    else:
        peak = retained * 2
    return retained, peak - retained


def _mapping_array_usage(plan: _MappingPlan) -> tuple[int, int]:
    retained = 0
    transient = 0
    for _key, child in plan.entries:
        if isinstance(child, _MappingPlan):
            child_retained, child_transient = _mapping_array_usage(child)
        else:
            child_retained, child_transient = _numeric_array_usage(
                cast(_NumericPlan, child)
            )
        retained += child_retained
        transient = max(transient, child_transient)
    return retained, transient


def _action_array_usage(plan: _ActionPlan) -> tuple[int, int]:
    retained = plan.target_nbytes
    source = plan.source
    if isinstance(source.source, np.ndarray):
        peak = retained if _array_is_immutable(source.source) else retained * 2
    else:
        peak = source.nbytes + retained * 2
    return retained, peak - retained


def _reserve_array_usage(
    arrays: ByteBudget,
    *,
    retained_bytes: int,
    peak_bytes: int,
) -> None:
    """Reserve one complete retained value before any array allocation."""

    observed = arrays.consumed + peak_bytes
    if observed > arrays.limit:
        raise ResourceLimitError(
            arrays.resource,
            limit=arrays.limit,
            observed=observed,
        )
    arrays.consume(retained_bytes)


def _reserve_transition_arrays(
    arrays: ByteBudget,
    *,
    action: _ActionPlan,
    obs: _MappingPlan,
    next_obs: _MappingPlan,
) -> None:
    action_retained, action_transient = _action_array_usage(action)
    obs_retained, obs_transient = _mapping_array_usage(obs)
    next_retained, next_transient = _mapping_array_usage(next_obs)
    retained = action_retained + obs_retained + next_retained
    transient = max(action_transient, obs_transient, next_transient)
    _reserve_array_usage(
        arrays,
        retained_bytes=retained,
        peak_bytes=retained + transient,
    )


def _validate_declared_numeric(
    plan: _NumericPlan,
    *,
    context: str,
    require_float32: bool = False,
) -> None:
    """Recheck captured array leaves without traversing caller containers."""

    source = plan.source
    if isinstance(source, np.ndarray):
        if (
            tuple(int(dim) for dim in source.shape) != plan.shape
            or np.dtype(source.dtype) != plan.dtype
        ):
            raise _numeric_error(context)
        _require_finite_array(source, context=context)
        if require_float32:
            for item in cast(Iterator[_RealScalar], source.flat):
                _require_float32_scalar(item, context=context)
        return
    if plan.children:
        if not isinstance(source, tuple) or len(source) != len(plan.children):
            raise _numeric_error(context)
        for index, child in enumerate(plan.children):
            _validate_declared_numeric(
                child,
                context=_path_context(context, f"[{index}]"),
                require_float32=require_float32,
            )


def _materialize_numeric(
    value: object,
    plan: _NumericPlan,
    *,
    context: str,
    arrays: ByteBudget,
    admitted: bool = False,
) -> np.ndarray[Any, Any]:
    del value
    if not admitted:
        retained, transient = _numeric_array_usage(plan)
        _reserve_array_usage(
            arrays,
            retained_bytes=retained,
            peak_bytes=retained + transient,
        )
    _validate_declared_numeric(plan, context=context)
    source = plan.source
    if isinstance(source, np.ndarray):
        if (
            tuple(int(dim) for dim in source.shape) != plan.shape
            or np.dtype(source.dtype) != plan.dtype
        ):
            raise _numeric_error(context)
        return _readonly_array(source)
    try:
        array = np.asarray(source)
    except (TypeError, ValueError) as exc:
        raise _numeric_error(context) from exc
    if (
        tuple(int(dim) for dim in array.shape) != plan.shape
        or np.dtype(array.dtype) != plan.dtype
    ):
        raise _numeric_error(context)
    _require_finite_array(array, context=context)
    return _readonly_array(array)


def _materialize_tree(
    value: object,
    plan: _MappingPlan,
    *,
    context: str,
    arrays: ByteBudget,
    admitted: bool = False,
) -> Mapping[str, object]:
    del value
    if not admitted:
        retained, transient = _mapping_array_usage(plan)
        _reserve_array_usage(
            arrays,
            retained_bytes=retained,
            peak_bytes=retained + transient,
        )
    frozen: dict[str, object] = {}
    for key, child_plan in plan.entries:
        child_context = _key_context(context, key)
        if isinstance(child_plan, _MappingPlan):
            frozen[key] = _materialize_tree(
                None,
                child_plan,
                context=child_context,
                arrays=arrays,
                admitted=True,
            )
        else:
            frozen[key] = _materialize_numeric(
                None,
                cast(_NumericPlan, child_plan),
                context=child_context,
                arrays=arrays,
                admitted=True,
            )
    return MappingProxyType(frozen)


def _same_tree_shape(left: object, right: object, *, context: str) -> None:
    """Require identical mapping/array structure for obs and next_obs."""

    if isinstance(left, Mapping) or isinstance(right, Mapping):
        if not isinstance(left, Mapping) or not isinstance(right, Mapping):
            raise ValueError(f"{_bounded_context(context)} mapping structure differs")
        if set(left) != set(right):
            raise ValueError(f"{_bounded_context(context)} mapping keys differ")
        for key in left:
            _same_tree_shape(
                left[key],
                right[key],
                context=_key_context(context, key),
            )
        return

    if isinstance(left, np.ndarray) and isinstance(right, np.ndarray):
        if left.shape != right.shape:
            raise ValueError(
                f"{_bounded_context(context)} shape differs: "
                f"left={left.shape} right={right.shape}"
            )
        return
    raise ValueError(
        f"{_bounded_context(context)} must contain matching numeric array leaves"
    )


def _tree_shape(value: object) -> object:
    """Return an immutable shape-only signature for one numeric tree."""

    if isinstance(value, Mapping):
        return tuple((key, _tree_shape(value[key])) for key in sorted(value))
    if isinstance(value, np.ndarray):
        return tuple(int(dim) for dim in value.shape)
    raise TypeError(f"unexpected numeric tree leaf {type(value).__name__}")


def _declare_action(
    value: object,
    *,
    context: str,
    budget: _DeclarationBudget,
    reject_writable: bool,
) -> _ActionPlan:
    if isinstance(value, np.ndarray):
        if value.dtype != np.float32:
            raise TypeError(f"{_bounded_context(context)} must be a float32 array")
        if value.ndim != 1 or value.size == 0:
            raise ValueError(
                f"{_bounded_context(context)} must be a non-empty flat array"
            )
        if reject_writable and value.flags.writeable:
            raise ValueError(f"{_bounded_context(context)} must be read-only")

    source = _declare_numeric(
        value,
        context=context,
        budget=budget,
        depth=0,
        active=set(),
        require_float32=True,
    )
    if len(source.shape) != 1:
        raise ValueError(
            f"{_bounded_context(context)} must be a flat one-dimensional array"
        )
    if source.shape[0] == 0:
        raise ValueError(f"{_bounded_context(context)} must be non-empty")
    try:
        target_nbytes = checked_shape_nbytes(
            source.shape,
            np.dtype(np.float32),
            resource=_bounded_context(context),
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
            max_rank=1,
        )
    except ResourceLimitError:
        raise
    except (TypeError, ValueError) as exc:
        raise ValueError(str(exc)) from exc
    budget.retain(
        _DECLARATION_PLAN_BYTES + _DECLARATION_REFERENCE_BYTES,
        context=context,
    )
    return _ActionPlan(source=source, target_nbytes=target_nbytes)


def _materialize_action(
    value: object,
    plan: _ActionPlan,
    *,
    context: str,
    arrays: ByteBudget,
    admitted: bool = False,
) -> np.ndarray[Any, np.dtype[np.float32]]:
    source = plan.source
    del value
    if not admitted:
        retained, transient = _action_array_usage(plan)
        _reserve_array_usage(
            arrays,
            retained_bytes=retained,
            peak_bytes=retained + transient,
        )
    _validate_declared_numeric(source, context=context, require_float32=True)
    captured = source.source
    if isinstance(captured, np.ndarray):
        if (
            captured.dtype != np.float32
            or captured.ndim != 1
            or captured.size == 0
            or tuple(int(dim) for dim in captured.shape) != source.shape
        ):
            raise ValueError(
                f"{_bounded_context(context)} must be a non-empty flat float32 array"
            )
        return cast(
            np.ndarray[Any, np.dtype[np.float32]],
            _readonly_array(captured),
        )

    # Keep the raw NumPy conversion and the immutable backing copy in the
    # peak declaration. Neither conversion is allowed to happen first.
    try:
        raw = np.asarray(captured)
        if (
            tuple(int(dim) for dim in raw.shape) != source.shape
            or np.dtype(raw.dtype) != source.dtype
        ):
            raise _numeric_error(context)
        flat = np.asarray(raw, dtype=np.float32).reshape(-1)
    except (TypeError, ValueError, OverflowError) as exc:
        if isinstance(exc, ValueError) and str(exc).endswith(
            "must be a rectangular numeric array"
        ):
            raise
        raise ValueError(f"{context} must be representable as float32") from exc
    _require_finite_array(flat, context=context)
    return cast(np.ndarray[Any, np.dtype[np.float32]], _readonly_array(flat))


def _require_finite_reward(value: object, *, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int, np.number)):
        raise ValueError(f"{context} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{context} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{context} must be finite")
    return result


def _validate_transition_fields(
    *,
    obs: object,
    action: object,
    reward: object,
    next_obs: object,
    terminated: object,
    truncated: object,
    episode_index: object,
    step_index: object,
    episode_key: object,
    require_immutable: bool,
) -> None:
    if not isinstance(obs, Mapping) or not obs:
        raise TypeError("DemoTransition obs must be a non-empty mapping")
    if not isinstance(next_obs, Mapping) or not next_obs:
        raise TypeError("DemoTransition next_obs must be a non-empty mapping")
    if not isinstance(action, np.ndarray) or action.dtype != np.float32:
        raise TypeError("DemoTransition action must be a float32 array")
    if action.ndim != 1 or action.size == 0:
        raise ValueError("DemoTransition action must be a non-empty flat array")
    if action.flags.writeable:
        raise ValueError("DemoTransition action must be read-only")
    if require_immutable and not _array_is_immutable(action):
        raise ValueError("DemoTransition action must own immutable backing")
    if not isinstance(terminated, bool) or not isinstance(truncated, bool):
        raise TypeError("DemoTransition termination flags must be booleans")
    if not isinstance(episode_index, int) or isinstance(episode_index, bool):
        raise TypeError("DemoTransition episode_index must be an integer")
    if not isinstance(step_index, int) or isinstance(step_index, bool):
        raise TypeError("DemoTransition step_index must be an integer")
    if episode_index < 0 or step_index < 0:
        raise ValueError("DemoTransition identity indices must be non-negative")
    if not isinstance(episode_key, str) or not episode_key:
        raise TypeError("DemoTransition episode_key must be a non-empty string")
    if not math.isfinite(float(cast(Any, reward))):
        raise ValueError("DemoTransition reward must be finite")
    _same_tree_shape(obs, next_obs, context="DemoTransition obs/next_obs")


@dataclass(frozen=True, slots=True)
class DemoTransition:
    """One immutable framework-neutral demonstration transition."""

    obs: Mapping[str, object]
    action: np.ndarray[Any, np.dtype[np.float32]]
    reward: float
    next_obs: Mapping[str, object]
    terminated: bool
    truncated: bool
    episode_index: int
    step_index: int
    episode_key: str

    def __post_init__(self) -> None:
        # Keep the historical writable-action rejection before touching caller
        # observations. A read-only view is still copied below unless its base
        # is immutable bytes.
        if not isinstance(self.action, np.ndarray) or self.action.dtype != np.float32:
            raise TypeError("DemoTransition action must be a float32 array")
        if self.action.ndim != 1 or self.action.size == 0:
            raise ValueError("DemoTransition action must be a non-empty flat array")
        if self.action.flags.writeable:
            raise ValueError("DemoTransition action must be read-only")
        if not isinstance(self.obs, Mapping):
            raise TypeError("DemoTransition obs must be a non-empty mapping")
        if not isinstance(self.next_obs, Mapping):
            raise TypeError("DemoTransition next_obs must be a non-empty mapping")
        if not isinstance(self.terminated, bool) or not isinstance(
            self.truncated, bool
        ):
            raise TypeError("DemoTransition termination flags must be booleans")
        if not isinstance(self.episode_index, int) or isinstance(
            self.episode_index, bool
        ):
            raise TypeError("DemoTransition episode_index must be an integer")
        if not isinstance(self.step_index, int) or isinstance(self.step_index, bool):
            raise TypeError("DemoTransition step_index must be an integer")
        if self.episode_index < 0 or self.step_index < 0:
            raise ValueError("DemoTransition identity indices must be non-negative")
        if not isinstance(self.episode_key, str) or not self.episode_key:
            raise TypeError("DemoTransition episode_key must be a non-empty string")
        reward = _require_finite_reward(self.reward, context="DemoTransition reward")

        arrays = ByteBudget(
            "DemoTransition decoded array bytes", MAX_DECODED_AGGREGATE_BYTES
        )
        declarations = _DeclarationBudget()
        action_plan = _declare_action(
            self.action,
            context="DemoTransition action",
            budget=declarations,
            reject_writable=True,
        )
        obs_plan = _declare_tree(
            self.obs,
            context="DemoTransition obs",
            budget=declarations,
            depth=0,
            active=set(),
        )
        next_obs_plan = _declare_tree(
            self.next_obs,
            context="DemoTransition next_obs",
            budget=declarations,
            depth=0,
            active=set(),
        )
        if _plan_shape(obs_plan) != _plan_shape(next_obs_plan):
            raise ValueError("DemoTransition obs/next_obs mapping structure differs")
        _reserve_transition_arrays(
            arrays,
            action=action_plan,
            obs=obs_plan,
            next_obs=next_obs_plan,
        )

        action = _materialize_action(
            self.action,
            action_plan,
            context="DemoTransition action",
            arrays=arrays,
            admitted=True,
        )
        obs = _materialize_tree(
            self.obs,
            obs_plan,
            context="DemoTransition obs",
            arrays=arrays,
            admitted=True,
        )
        next_obs = _materialize_tree(
            self.next_obs,
            next_obs_plan,
            context="DemoTransition next_obs",
            arrays=arrays,
            admitted=True,
        )
        object.__setattr__(self, "obs", obs)
        object.__setattr__(self, "action", action)
        object.__setattr__(self, "reward", reward)
        object.__setattr__(self, "next_obs", next_obs)
        _validate_transition_fields(
            obs=obs,
            action=action,
            reward=reward,
            next_obs=next_obs,
            terminated=self.terminated,
            truncated=self.truncated,
            episode_index=self.episode_index,
            step_index=self.step_index,
            episode_key=self.episode_key,
            require_immutable=True,
        )

    @classmethod
    def _from_owned(
        cls,
        *,
        obs: Mapping[str, object],
        action: np.ndarray[Any, np.dtype[np.float32]],
        reward: float,
        next_obs: Mapping[str, object],
        terminated: bool,
        truncated: bool,
        episode_index: int,
        step_index: int,
        episode_key: str,
    ) -> DemoTransition:
        _validate_transition_fields(
            obs=obs,
            action=action,
            reward=reward,
            next_obs=next_obs,
            terminated=terminated,
            truncated=truncated,
            episode_index=episode_index,
            step_index=step_index,
            episode_key=episode_key,
            require_immutable=True,
        )
        result = object.__new__(cls)
        object.__setattr__(result, "obs", obs)
        object.__setattr__(result, "action", action)
        object.__setattr__(result, "reward", reward)
        object.__setattr__(result, "next_obs", next_obs)
        object.__setattr__(result, "terminated", terminated)
        object.__setattr__(result, "truncated", truncated)
        object.__setattr__(result, "episode_index", episode_index)
        object.__setattr__(result, "step_index", step_index)
        object.__setattr__(result, "episode_key", episode_key)
        return result

    @property
    def done(self) -> bool:
        """Return the flag derived from termination and truncation."""
        return self.terminated or self.truncated


@dataclass(frozen=True, slots=True)
class ValidatedDemoSource:
    """Validated transitions bound to one manifest/content generation."""

    manifest: DatasetManifest
    transitions: tuple[DemoTransition, ...]
    content_fingerprint: str
    manifest_fingerprint: str
    member_fingerprints: Mapping[str, str]

    def __post_init__(self) -> None:
        if not self.transitions:
            raise ValueError("validated demo source contains no transitions")
        if (
            not isinstance(self.content_fingerprint, str)
            or len(self.content_fingerprint) != 64
            or any(c not in "0123456789abcdef" for c in self.content_fingerprint)
        ):
            raise ValueError("validated demo source content fingerprint is invalid")
        if (
            not isinstance(self.manifest_fingerprint, str)
            or len(self.manifest_fingerprint) != 64
            or any(c not in "0123456789abcdef" for c in self.manifest_fingerprint)
        ):
            raise ValueError("validated demo source manifest fingerprint is invalid")
        if not isinstance(self.member_fingerprints, Mapping):
            raise TypeError(
                "validated demo source member fingerprints must be a mapping"
            )
        object.__setattr__(
            self,
            "member_fingerprints",
            MappingProxyType(dict(self.member_fingerprints)),
        )

    @property
    def num_transitions(self) -> int:
        return len(self.transitions)

    @property
    def content_binding(self) -> Mapping[str, object]:
        """Return the immutable manifest/member binding used for this source."""
        return MappingProxyType(
            {
                "manifest_sha256": self.manifest_fingerprint,
                "members": self.member_fingerprints,
                "content_sha256": self.content_fingerprint,
            }
        )

    @property
    def demo_io_contract(self) -> Mapping[str, object] | None:
        """Return the producer contract, when the artifact records one."""
        value = self.manifest.metadata.get("demo_io_contract")
        return value if isinstance(value, Mapping) else None


def build_demo_source_binding(
    source: Any,
    *,
    contract: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the bounded identity for one validated demonstration source."""

    content_binding = getattr(source, "content_binding", None)
    if not isinstance(content_binding, Mapping):
        raise ValueError("canonical demo source requires a complete content binding")
    expected_content = {
        "manifest_sha256": getattr(source, "manifest_fingerprint", None),
        "members": getattr(source, "member_fingerprints", None),
        "content_sha256": getattr(source, "content_fingerprint", None),
    }
    if (
        set(content_binding) != set(expected_content)
        or content_binding.get("manifest_sha256") != expected_content["manifest_sha256"]
        or content_binding.get("content_sha256") != expected_content["content_sha256"]
        or content_binding.get("members") != expected_content["members"]
    ):
        raise ValueError("canonical demo source content binding is inconsistent")
    members = content_binding.get("members")
    if not isinstance(members, Mapping) or not members:
        raise ValueError("canonical demo source content binding has no members")

    manifest = getattr(source, "manifest", None)
    identity = getattr(manifest, "identity", None)
    metadata = getattr(manifest, "metadata", None)
    if manifest is None or identity is None or not isinstance(metadata, Mapping):
        raise ValueError("canonical demo source manifest binding is missing")
    revision = metadata.get("revision")
    if revision is not None and not isinstance(revision, str):
        raise ValueError("canonical demo manifest revision must be a string")
    manifest_binding = {
        "identity": {
            "task": identity.task,
            "embodiment": identity.embodiment,
            "action_mode": identity.action_mode,
        },
        "hf_repo_id": manifest.hf_repo_id,
        "revision": revision,
        "split": manifest.split.value,
        "storage_format": manifest.storage_format.value,
        "artifact_hash": manifest.artifact_hash,
    }
    return _normalise_source_binding(
        {
            "content_binding": content_binding,
            "manifest": manifest_binding,
            "demo_io_contract": contract,
            "dataset_fingerprint": source.content_fingerprint,
            "manifest_fingerprint": source.manifest_fingerprint,
            "transition_count": len(source.transitions),
        }
    )


def _normalise_source_binding(value: Any) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError("canonical demo binding must be a mapping")
    try:
        payload = json.loads(
            json.dumps(value, sort_keys=True, default=_binding_json_default)
        )
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("canonical demo binding is not JSON-compatible") from exc
    if not isinstance(payload, dict):
        raise ValueError("canonical demo binding must be a JSON object")
    try:
        encoded = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
            default=_binding_json_default,
        ).encode("utf-8")
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("canonical demo binding is not JSON-compatible") from exc
    if len(encoded) > MAX_DEMO_SOURCE_BINDING_BYTES:
        raise ValueError("canonical demo binding exceeds metadata budget")
    return payload


def _binding_json_default(value: Any) -> Any:
    if isinstance(value, Mapping):
        return dict(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"value {type(value).__name__} is not JSON serializable")


def _read_stable_manifest(
    snapshot: DatasetSnapshot,
) -> tuple[DatasetManifest, str]:
    """Read and hash the exact manifest bytes associated with *snapshot*."""
    manifest_path = snapshot.manifest_path
    if manifest_path is None:
        return snapshot.manifest, snapshot.manifest.artifact_hash
    _member_name, expected = snapshot.member_identity(manifest_path)
    with ArtifactPublication.file(manifest_path).open_snapshot(
        expected_sha256=expected,
    ) as stable:
        if stable is None:
            raise FileNotFoundError(manifest_path)
        payload = stable.file.read(MAX_IN_MEMORY_FILE_BYTES + 1)
        if len(payload) > MAX_IN_MEMORY_FILE_BYTES:
            raise ResourceLimitError(
                "demo manifest bytes",
                limit=MAX_IN_MEMORY_FILE_BYTES,
                observed=len(payload),
            )
        try:
            raw_manifest = JsonDecodeBudget().loads(
                payload, context="demo dataset manifest"
            )
            parsed = DatasetManifest.from_dict(raw_manifest)
        except (TypeError, ValueError, RecursionError) as exc:
            raise ValueError(f"invalid demo dataset manifest: {exc}") from exc
        if parsed != snapshot.manifest:
            raise PublicationError(
                "dataset manifest changed between snapshot resolution and strict read"
            )
        return parsed, stable.sha256


def _parse_stable_jsonl_file(
    *,
    path: Path,
    snapshot: DatasetSnapshot,
    line_budget: ByteBudget,
    array_budget: ByteBudget,
    max_transitions: int,
    transition_offset: int,
    json_budget: JsonDecodeBudget,
) -> tuple[list[tuple[DemoTransition, str]], str]:
    """Admit typed rows incrementally from one descriptor-backed member."""
    _member_name, expected = snapshot.member_identity(path)
    rows: list[tuple[DemoTransition, str]] = []
    digest = hashlib.sha256()
    with ArtifactPublication.file(path).open_snapshot(
        expected_sha256=expected,
    ) as stable:
        if stable is None:
            raise FileNotFoundError(path)
        stream = stable.file
        line_number = 0
        while True:
            raw_line = stream.readline(MAX_JSONL_LINE_BYTES + 1)
            if not raw_line:
                break
            digest.update(raw_line)
            line_budget.consume(len(raw_line))
            if len(raw_line) > MAX_JSONL_LINE_BYTES:
                raise ResourceLimitError(
                    "demo JSONL line",
                    limit=MAX_JSONL_LINE_BYTES,
                    observed=len(raw_line),
                )
            line_number += 1
            try:
                line = raw_line.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise ValueError(f"{path}:{line_number} is not valid UTF-8") from exc
            stripped = line.strip()
            if not stripped:
                continue
            context = f"{path}:{line_number}"
            if transition_offset + len(rows) >= max_transitions:
                raise ResourceLimitError(
                    "demo transitions",
                    limit=max_transitions,
                    observed=transition_offset + len(rows) + 1,
                )
            try:
                raw_row = json_budget.loads(
                    stripped, context=context, max_bytes=MAX_JSONL_LINE_BYTES
                )
                if not isinstance(raw_row, Mapping):
                    raise ValueError(f"{context} must be a JSON object")
                if "next_obs" not in raw_row or raw_row["next_obs"] is None:
                    raise ValueError(f"{context} missing required explicit next_obs")
                for flag in ("terminated", "truncated"):
                    if flag not in raw_row:
                        raise ValueError(
                            f"{context} requires explicit terminated and truncated flags"
                        )
                    if type(raw_row[flag]) is not bool:
                        raise ValueError(
                            f"{context} requires explicit terminated and truncated "
                            "boolean flags"
                        )
                row = validate_srb_jsonl_row(raw_row, context=context)
            except (json.JSONDecodeError, RecursionError, TypeError, ValueError) as exc:
                raise ValueError(str(exc)) from exc
            rows.append(
                (
                    _build_transition(row, context=context, array_budget=array_budget),
                    context,
                )
            )
    actual = digest.hexdigest()
    if expected is not None and actual != expected:
        raise PublicationError(
            f"dataset member changed while strict demo source was read: {path}"
        )
    return rows, actual


def _build_transition(
    row: Mapping[str, Any],
    *,
    context: str,
    array_budget: ByteBudget,
) -> DemoTransition:
    if "next_obs" not in row:
        raise ValueError(f"{context} missing required explicit next_obs")
    if "terminated" not in row or "truncated" not in row:
        raise ValueError(f"{context} requires explicit terminated and truncated flags")
    terminated = row["terminated"]
    truncated = row["truncated"]
    if type(terminated) is not bool or type(truncated) is not bool:
        raise ValueError(f"{context} termination flags must be boolean")
    done = row.get("done")
    if type(done) is not bool or done != (terminated or truncated):
        raise ValueError(f"{context} done must equal (terminated or truncated)")
    episode_index = row.get("episode_index")
    step_index = row.get("step_index")
    if (
        not isinstance(episode_index, int)
        or isinstance(episode_index, bool)
        or episode_index < 0
    ):
        raise ValueError(f"{context} episode_index must be a non-negative integer")
    if (
        not isinstance(step_index, int)
        or isinstance(step_index, bool)
        or step_index < 0
    ):
        raise ValueError(f"{context} step_index must be a non-negative integer")
    raw_key = row.get("episode_key")
    if raw_key is None:
        episode_key = f"episode_{episode_index:06d}"
    elif isinstance(raw_key, str) and raw_key:
        episode_key = raw_key
    else:
        raise ValueError(f"{context} episode_key must be a non-empty string")
    reward = _require_finite_reward(row.get("reward"), context=f"{context}.reward")

    # Admit every declaration before allocating any retained NumPy value.
    declarations = _DeclarationBudget()
    action_plan = _declare_action(
        row.get("action"),
        context=f"{context}.action",
        budget=declarations,
        reject_writable=False,
    )
    obs_plan = _declare_tree(
        row.get("obs"),
        context=f"{context}.obs",
        budget=declarations,
        depth=0,
        active=set(),
    )
    next_obs_plan = _declare_tree(
        row["next_obs"],
        context=f"{context}.next_obs",
        budget=declarations,
        depth=0,
        active=set(),
    )
    if _plan_shape(obs_plan) != _plan_shape(next_obs_plan):
        raise ValueError(f"{context} obs/next_obs mapping structure differs")
    _reserve_transition_arrays(
        array_budget,
        action=action_plan,
        obs=obs_plan,
        next_obs=next_obs_plan,
    )

    action = _materialize_action(
        row.get("action"),
        action_plan,
        context=f"{context}.action",
        arrays=array_budget,
        admitted=True,
    )
    obs = _materialize_tree(
        row.get("obs"),
        obs_plan,
        context=f"{context}.obs",
        arrays=array_budget,
        admitted=True,
    )
    next_obs = _materialize_tree(
        row["next_obs"],
        next_obs_plan,
        context=f"{context}.next_obs",
        arrays=array_budget,
        admitted=True,
    )
    return DemoTransition._from_owned(
        obs=obs,
        action=action,
        reward=reward,
        next_obs=next_obs,
        terminated=terminated,
        truncated=truncated,
        episode_index=episode_index,
        step_index=step_index,
        episode_key=episode_key,
    )


def _validate_episode_order(
    transitions: list[DemoTransition],
    *,
    contexts: list[str],
    member_paths: list[Path],
) -> None:
    if not transitions:
        raise ValueError("canonical demonstration dataset contains no transitions")
    closed: set[int] = set()
    seen_keys: dict[str, int] = {}
    current_episode: int | None = None
    current_key: str | None = None
    previous_step: int | None = None
    episode_members: dict[int, Path] = {}
    episode_done = False
    expected_action_dim = int(transitions[0].action.size)
    expected_obs_shape = _tree_shape(transitions[0].obs)
    expected_next_shape = _tree_shape(transitions[0].next_obs)

    for transition, context, member in zip(
        transitions,
        contexts,
        member_paths,
        strict=True,
    ):
        episode = transition.episode_index
        key = transition.episode_key
        if transition.action.size != expected_action_dim:
            raise ValueError(
                f"{context} action dimension {transition.action.size} does not "
                f"match first dimension {expected_action_dim}"
            )
        if _tree_shape(transition.obs) != expected_obs_shape:
            raise ValueError(f"{context} observation shape differs from the first row")
        if _tree_shape(transition.next_obs) != expected_next_shape:
            raise ValueError(
                f"{context} next observation shape differs from the first row"
            )

        if key in seen_keys and seen_keys[key] != episode:
            raise ValueError(
                f"{context} episode_key {key!r} identifies multiple episode indices"
            )
        seen_keys.setdefault(key, episode)

        if current_episode is None:
            current_episode = episode
            current_key = key
            previous_step = None
            episode_done = False
        elif episode != current_episode:
            if not episode_done:
                raise ValueError(
                    f"{context} episode {current_episode} is incomplete before "
                    f"episode {episode} starts"
                )
            if episode < current_episode:
                raise ValueError(
                    f"{context} episode indices are out of order: "
                    f"{episode} follows {current_episode}"
                )
            closed.add(current_episode)
            if episode in closed:
                raise ValueError(
                    f"{context} episode {episode} is interleaved after completion"
                )
            current_episode = episode
            current_key = key
            previous_step = None
            episode_done = False
        elif episode in closed:
            raise ValueError(f"{context} episode {episode} is interleaved")

        if key != current_key:
            raise ValueError(f"{context} episode_key changed within one episode")
        first_member = episode_members.setdefault(episode, member)
        if member != first_member:
            # A split episode is ambiguous to native replay consumers.  Exported
            # canonical datasets place exactly one logical episode in each file.
            raise ValueError(
                f"{context} episode {episode} is split across JSONL members"
            )
        expected_step = 0 if previous_step is None else previous_step + 1
        if transition.step_index != expected_step:
            raise ValueError(
                f"{context} step_index must be contiguous from zero; expected "
                f"{expected_step}, got {transition.step_index}"
            )
        if episode_done:
            raise ValueError(f"{context} contains a row after its terminal row")
        previous_step = transition.step_index
        episode_done = transition.done

    if not episode_done:
        raise ValueError(
            f"episode {current_episode} is incomplete: final row must have done=True"
        )


@contextmanager
def open_demo_transition_source(
    path: str | Path,
    *,
    max_transitions: int,
    json_budget: JsonDecodeBudget | None = None,
) -> Iterator[ValidatedDemoSource]:
    """Open and validate one bounded canonical JSONL demonstration source.

    The complete selected source is admitted before yielding.  ``max_transitions``
    is an upper bound, never a prefix-selection request: a source with one more
    row is rejected instead of silently truncated. Retained rows also share a
    structural JSON budget across all members, independent of the row cap.
    """
    limit = require_bounded_int(
        max_transitions,
        resource="demo transitions",
        minimum=1,
        maximum=MAX_DEMO_TRANSITIONS,
    )
    if json_budget is not None and not isinstance(json_budget, JsonDecodeBudget):
        raise TypeError("json_budget must be a JsonDecodeBudget or None")
    decoder = (
        JsonDecodeBudget(max_nodes=MAX_JSONL_ROW_NODES)
        if json_budget is None
        else json_budget
    )

    # Snapshot admission owns bounded traversal of the selected dataset members.
    # Scanning a sidecar's entire parent would admit unrelated sibling trees.
    with open_dataset_snapshot(path) as snapshot:
        if snapshot.external_reference:
            raise PublicationError(
                "canonical dataset reference escapes its snapshot root"
            )
        manifest, manifest_fingerprint = _read_stable_manifest(snapshot)
        validate_demo_io_contract(
            manifest.metadata.get("demo_io_contract"),
            expected_task=manifest.identity.task,
        )
        if manifest.storage_format is not StorageFormat.SRB_JSONL:
            raise ValueError(
                "strict demo transition source requires srb_jsonl; export HDF5 "
                "to canonical JSONL before replay prefill"
            )
        data_path = snapshot.dataset_path
        episode_paths = srb_jsonl_episode_paths(data_path)
        if not episode_paths:
            raise ValueError(
                f"canonical dataset has no JSONL episode files: {data_path}"
            )

        line_budget = ByteBudget(
            "demo JSONL aggregate bytes", MAX_DECODED_AGGREGATE_BYTES
        )
        array_budget = ByteBudget(
            "demo decoded array bytes", MAX_DECODED_AGGREGATE_BYTES
        )
        transitions: list[DemoTransition] = []
        contexts: list[str] = []
        members: list[Path] = []
        member_fingerprints: dict[str, str] = {}
        for episode_path in episode_paths:
            rows, member_hash = _parse_stable_jsonl_file(
                path=episode_path,
                snapshot=snapshot,
                line_budget=line_budget,
                array_budget=array_budget,
                max_transitions=limit,
                transition_offset=len(transitions),
                json_budget=decoder,
            )
            if not rows:
                raise ValueError(f"canonical JSONL member is empty: {episode_path}")
            member_name, _expected_digest = snapshot.member_identity(episode_path)
            member_fingerprints[member_name] = member_hash
            for transition, context in rows:
                transitions.append(transition)
                contexts.append(context)
                members.append(episode_path)
        _validate_episode_order(transitions, contexts=contexts, member_paths=members)

        # Rows come from owned member copies, and their complete byte digests
        # bind the content handed to the native adapter. Keep those copies alive
        # until the yielded source closes, including native replay prefill.
        content_fingerprint = demo_content_fingerprint(
            manifest_fingerprint=manifest_fingerprint,
            member_fingerprints=member_fingerprints,
        )
        yield ValidatedDemoSource(
            manifest=manifest,
            transitions=tuple(transitions),
            content_fingerprint=content_fingerprint,
            manifest_fingerprint=manifest_fingerprint,
            member_fingerprints=member_fingerprints,
        )


__all__ = [
    "DEMO_IO_CONTRACT_VERSION",
    "MAX_DEMO_TRANSITIONS",
    "MAX_DEMO_SOURCE_BINDING_BYTES",
    "DemoTransition",
    "ValidatedDemoSource",
    "build_demo_source_binding",
    "open_demo_transition_source",
]
