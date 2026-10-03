"""Dependency-free JSON value types, validation, and bounded encoding."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import fields, is_dataclass
from types import MappingProxyType
from typing import Any, TypeAlias, TypeGuard, TypeVar

from srb.utils.resource_budget import (
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_JSON_AGGREGATE_NODES,
    MAX_JSON_DEPTH,
    MAX_JSON_NODES,
    MAX_JSONL_ROW_NODES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    ResourceLimitError,
    require_bounded_int,
)


class JsonDecodeBudget:
    """Count JSON structure before decoding it.

    Counters belong to one operation. Streaming callers can create one per row;
    retained metadata can share one. Limits measure input structure, not memory.
    """

    def __init__(
        self,
        *,
        max_nodes: int = MAX_JSON_NODES,
        max_depth: int = MAX_JSON_DEPTH,
        max_total_nodes: int = MAX_JSON_AGGREGATE_NODES,
    ) -> None:
        self._max_nodes = require_bounded_int(
            max_nodes, resource="JSON node limit", maximum=MAX_JSONL_ROW_NODES
        )
        self._max_depth = require_bounded_int(
            max_depth, resource="JSON depth limit", maximum=MAX_JSON_DEPTH
        )
        self._max_total_nodes = require_bounded_int(
            max_total_nodes,
            resource="JSON aggregate node limit",
            maximum=MAX_JSON_AGGREGATE_NODES,
        )
        self._consumed_nodes = 0

    @property
    def consumed_nodes(self) -> int:
        """Lexical nodes scanned so far, including rejected documents."""
        return self._consumed_nodes

    def loads(
        self,
        payload: str | bytes | bytearray,
        *,
        context: str = "JSON",
        max_bytes: int = MAX_IN_MEMORY_FILE_BYTES,
    ) -> Any:
        """Admit and decode one document; syntax remains delegated to ``json.loads``.

        Counts include containers, keys, scalars, and duplicate object entries.
        Byte inputs may be copied to immutable text before the constant-space scan.
        """
        byte_limit = require_bounded_int(
            max_bytes,
            resource="JSON byte limit",
            maximum=MAX_IN_MEMORY_FILE_BYTES,
        )
        if not isinstance(payload, (str, bytes, bytearray)):
            raise TypeError("JSON input must be str, bytes or bytearray")
        if len(payload) > byte_limit:
            raise ResourceLimitError(
                f"{context} JSON bytes", limit=byte_limit, observed=len(payload)
            )
        if isinstance(payload, bytearray):
            # Snapshot the bounded input so caller mutation cannot race decoding.
            payload = bytes(memoryview(payload)[: byte_limit + 1])
        if len(payload) > byte_limit:
            raise ResourceLimitError(
                f"{context} JSON bytes", limit=byte_limit, observed=len(payload)
            )
        if isinstance(payload, str):
            # Count UTF-8 bytes without allocating an encoded copy.
            wire_bytes = 0
            for char in payload:
                point = ord(char)
                wire_bytes += (
                    1
                    if point < 0x80
                    else 2
                    if point < 0x800
                    else 3
                    if point < 0x10000
                    else 4
                )
                if wire_bytes > byte_limit:
                    raise ResourceLimitError(
                        f"{context} JSON bytes", limit=byte_limit, observed=wire_bytes
                    )
            text = payload
        else:
            # Match json.loads encoding detection and avoid later input mutation.
            text = payload.decode(json.detect_encoding(payload), "surrogatepass")
        nodes = 0
        in_string = False
        escaped = False
        in_atom = False
        depth = 0
        for char in text:
            if in_string:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    in_string = False
                continue
            if char in " \t\r\n,:]}":
                if char in "]}":
                    depth = max(0, depth - 1)
                in_atom = False
                continue
            node_depth = depth
            if char == '"':
                in_string = True
                in_atom = False
            elif char in "[{":
                depth += 1
                in_atom = False
            elif in_atom:
                continue
            else:
                in_atom = True
            nodes += 1
            self._consumed_nodes += 1
            if node_depth > self._max_depth:
                raise ResourceLimitError(
                    f"{context} JSON nesting",
                    limit=self._max_depth,
                    observed=node_depth,
                )
            if nodes > self._max_nodes:
                raise ResourceLimitError(
                    f"{context} JSON nodes", limit=self._max_nodes, observed=nodes
                )
            if self._consumed_nodes > self._max_total_nodes:
                raise ResourceLimitError(
                    f"{context} JSON aggregate nodes",
                    limit=self._max_total_nodes,
                    observed=self._consumed_nodes,
                )
        return json.loads(text)


_DECIMAL_LOG10_2_LOWER_NUMERATOR = 301_029_995_663
_DECIMAL_LOG10_2_DENOMINATOR = 1_000_000_000_000


class _JSONEncodeAdmission:
    """Measure and copy one JSON graph before invoking the JSON encoder."""

    def __init__(
        self,
        *,
        context: str,
        max_bytes: int,
        max_nodes: int,
        max_depth: int,
        indent: int | None,
        allow_nan: bool,
    ) -> None:
        self._context = context
        self._max_bytes = max_bytes
        self._max_nodes = max_nodes
        self._max_depth = max_depth
        self._indent = indent
        self._allow_nan = allow_nan
        self._bytes = 0
        self._nodes = 0
        self._active: set[int] = set()

    def _resource(self, kind: str) -> str:
        return f"{self._context} JSON {kind}"

    def _add_bytes(self, amount: int) -> None:
        observed = self._bytes + amount
        if observed > self._max_bytes:
            raise ResourceLimitError(
                self._resource("bytes"),
                limit=self._max_bytes,
                observed=observed,
            )
        self._bytes = observed

    def _measure_open_indent(self, depth: int) -> None:
        if self._indent is not None:
            self._add_bytes(1 + self._indent * (depth + 1))

    def _measure_close_indent(self, depth: int) -> None:
        if self._indent is not None:
            self._add_bytes(1 + self._indent * depth)

    def _measure_item_separator(self, depth: int) -> None:
        if self._indent is None:
            self._add_bytes(2)
        else:
            self._add_bytes(2 + self._indent * (depth + 1))

    def _add_node(self, depth: int) -> None:
        if depth > self._max_depth:
            raise ResourceLimitError(
                self._resource("nesting"),
                limit=self._max_depth,
                observed=depth,
            )
        observed = self._nodes + 1
        if observed > self._max_nodes:
            raise ResourceLimitError(
                self._resource("nodes"),
                limit=self._max_nodes,
                observed=observed,
            )
        self._nodes = observed

    def _measure_string(self, value: str) -> None:
        self._add_bytes(2)  # JSON string delimiters.
        for character in str.__iter__(value):
            codepoint = ord(character)
            if character in {'"', "\\"}:
                increment = 2
            elif codepoint in {8, 9, 10, 12, 13}:
                increment = 2
            elif codepoint < 0x20:
                increment = 6
            elif codepoint < 0x7F:
                increment = 1
            elif codepoint <= 0xFFFF:
                increment = 6
            else:
                increment = 12
            self._add_bytes(increment)

    @staticmethod
    def _integer_decimal_lower_bound(value: int) -> int:
        """Return a safe lower bound for the encoded decimal digit count."""

        bits = int.bit_length(value)
        if bits == 0:
            digits = 1
        else:
            # This lower bound prevents an oversized decimal conversion.
            digits = (
                (bits - 1) * _DECIMAL_LOG10_2_LOWER_NUMERATOR
            ) // _DECIMAL_LOG10_2_DENOMINATOR + 1
        if int.__lt__(value, 0):
            digits += 1
        return digits

    def _integer_text(self, value: int) -> str:
        lower_bound = self._integer_decimal_lower_bound(value)
        remaining = self._max_bytes - self._bytes
        if lower_bound > remaining:
            raise ResourceLimitError(
                self._resource("bytes"),
                limit=self._max_bytes,
                observed=self._bytes + lower_bound,
            )
        # Match JSONEncoder's conversion while respecting the byte budget.
        text = int.__repr__(value)
        if len(text) > remaining:
            raise ResourceLimitError(
                self._resource("bytes"),
                limit=self._max_bytes,
                observed=self._bytes + len(text),
            )
        return text

    def _float_text(self, value: float) -> str:
        if float.__eq__(value, value) is False:
            text = "NaN"
        elif float.__eq__(value, float("inf")) is True:
            text = "Infinity"
        elif float.__eq__(value, float("-inf")) is True:
            text = "-Infinity"
        else:
            text = float.__repr__(value)
        if self._allow_nan or text not in {"NaN", "Infinity", "-Infinity"}:
            return text
        raise ValueError(
            f"Out of range float values are not JSON compliant: {float.__repr__(value)}"
        )

    def _measure_key(self, key: object) -> None:
        if isinstance(key, str):
            key_text = key
        elif key is None:
            key_text = "null"
        elif key is True:
            key_text = "true"
        elif key is False:
            key_text = "false"
        elif isinstance(key, int):
            key_text = self._integer_text(key)
        elif isinstance(key, float):
            key_text = self._float_text(key)
        else:
            raise TypeError(
                "keys must be str, int, float, bool or None, "
                f"not {key.__class__.__name__}"
            )
        self._measure_string(key_text)
        self._add_bytes(2)  # JSON key/value separator.

    def _walk(self, value: object, depth: int) -> object:
        if isinstance(value, str):
            self._add_node(depth)
            self._measure_string(value)
            return value
        if value is None:
            self._add_node(depth)
            self._add_bytes(4)
            return None
        if value is True:
            self._add_node(depth)
            self._add_bytes(4)
            return True
        if value is False:
            self._add_node(depth)
            self._add_bytes(5)
            return False
        if isinstance(value, int):
            self._add_node(depth)
            text = self._integer_text(value)
            self._add_bytes(len(text))
            return value
        if isinstance(value, float):
            self._add_node(depth)
            self._add_bytes(len(self._float_text(value)))
            return value
        if isinstance(value, (list, tuple)):
            is_empty = not value
            identifier = id(value)
            if identifier in self._active:
                raise ValueError("Circular reference detected")
            self._add_node(depth)
            self._active.add(identifier)
            try:
                if is_empty:
                    self._add_bytes(2)
                    if isinstance(value, tuple):
                        return ()
                    return []
                self._add_bytes(1)
                self._measure_open_indent(depth)
                array_copy: list[object] = []
                for index, item in enumerate(value):
                    if index:
                        self._measure_item_separator(depth)
                    array_copy.append(self._walk(item, depth + 1))
                self._measure_close_indent(depth)
                self._add_bytes(1)
                if isinstance(value, tuple):
                    return tuple(array_copy)
                return array_copy
            finally:
                self._active.remove(identifier)
        if isinstance(value, Mapping):
            is_empty = not value
            identifier = id(value)
            if identifier in self._active:
                raise ValueError("Circular reference detected")
            self._add_node(depth)
            self._active.add(identifier)
            try:
                if is_empty:
                    self._add_bytes(2)
                    return {}
                self._add_bytes(1)
                self._measure_open_indent(depth)
                object_copy: dict[object, object] = {}
                for index, (key, item) in enumerate(value.items()):
                    if index:
                        self._measure_item_separator(depth)
                    self._add_node(depth + 1)
                    self._measure_key(key)
                    object_copy[key] = self._walk(item, depth + 1)
                self._measure_close_indent(depth)
                self._add_bytes(1)
                return object_copy
            finally:
                self._active.remove(identifier)
        raise TypeError(
            f"{self._context} contains a non-JSON value of type {type(value).__name__}"
        )

    def snapshot(self, value: object) -> object:
        return self._walk(value, 0)


def bounded_json_dumps(
    value: object,
    *,
    context: str = "JSON",
    max_bytes: int = MAX_METADATA_ATTRIBUTE_BYTES,
    max_nodes: int = MAX_JSON_NODES,
    max_depth: int = MAX_JSON_DEPTH,
    sort_keys: bool = False,
    indent: int | None = None,
    allow_nan: bool = True,
) -> str:
    """Validate size and structure, snapshot, then encode one JSON value.

    Formatting follows ``json.dumps`` with ASCII escaping. Mapping inputs are
    copied before encoding, including immutable mapping views.
    """

    byte_limit = require_bounded_int(
        max_bytes,
        resource="JSON byte limit",
        maximum=MAX_IN_MEMORY_FILE_BYTES,
    )
    node_limit = require_bounded_int(
        max_nodes,
        resource="JSON node limit",
        maximum=MAX_JSON_NODES,
    )
    depth_limit = require_bounded_int(
        max_depth,
        resource="JSON depth limit",
        maximum=MAX_JSON_DEPTH,
    )
    indent_value = (
        None
        if indent is None
        else require_bounded_int(indent, resource="JSON indent", maximum=64)
    )
    admission = _JSONEncodeAdmission(
        context=context,
        max_bytes=byte_limit,
        max_nodes=node_limit,
        max_depth=depth_limit,
        indent=indent_value,
        allow_nan=allow_nan,
    )
    snapshot = admission.snapshot(value)
    return json.dumps(
        snapshot,
        ensure_ascii=True,
        separators=(", ", ": ") if indent_value is None else (",", ": "),
        sort_keys=sort_keys,
        indent=indent_value,
        allow_nan=allow_nan,
    )


JSONScalar: TypeAlias = str | int | float | bool | None
JSONValue: TypeAlias = JSONScalar | list["JSONValue"] | dict[str, "JSONValue"]
JSONObject: TypeAlias = dict[str, JSONValue]


class _FrozenJSONArray(tuple["FrozenJSONValue", ...]):
    """Immutable tuple array that retains list-comparison compatibility."""

    def __eq__(self, other: object) -> bool:
        if isinstance(other, (list, tuple)):
            return tuple(self) == tuple(other)
        return False

    def __ne__(self, other: object) -> bool:
        return not self == other


FrozenJSONArray: TypeAlias = tuple["FrozenJSONValue", ...]
FrozenJSONObject: TypeAlias = Mapping[str, "FrozenJSONValue"]
FrozenJSONValue: TypeAlias = JSONScalar | FrozenJSONArray | FrozenJSONObject


def is_json_value(value: object) -> TypeGuard[JSONValue]:
    """Return whether *value* is already in the recursive JSON vocabulary."""

    return _is_json_value(value, set())


def _is_json_value(value: object, active: set[int]) -> bool:
    if value is None or isinstance(value, (str, bool, int)):
        return True
    if isinstance(value, float):
        return math.isfinite(value)
    if not isinstance(value, (list, Mapping)):
        return False
    # Containers on the current path; a repeat is a cycle JSON cannot encode.
    if id(value) in active:
        return False
    active.add(id(value))
    try:
        if isinstance(value, list):
            return all(_is_json_value(item, active) for item in value)
        return all(
            isinstance(key, str) and _is_json_value(item, active)
            for key, item in value.items()
        )
    finally:
        active.discard(id(value))


def validate_json_value(value: object, *, context: str = "JSON value") -> JSONValue:
    """Copy a raw value into the JSON vocabulary, rejecting non-JSON objects."""

    return _validate_json_value(value, context, set())


def validate_json_object(
    value: object,
    *,
    context: str = "JSON object",
) -> JSONObject:
    """Validate and copy a raw mapping into a string-keyed JSON object."""

    if not isinstance(value, Mapping):
        raise TypeError(f"{context} must be an object, got {type(value).__name__}")
    return _validate_json_object(value, context, set())


def _validate_json_value(value: object, context: str, active: set[int]) -> JSONValue:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{context} must contain only finite numbers")
        return value
    if isinstance(value, list):
        _enter_container(value, context, active)
        try:
            return [
                _validate_json_value(item, f"{context}[{index}]", active)
                for index, item in enumerate(value)
            ]
        finally:
            active.discard(id(value))
    if isinstance(value, Mapping):
        return _validate_json_object(value, context, active)
    raise TypeError(f"{context} must be a JSON value, got {type(value).__name__}")


def _validate_json_object(
    value: Mapping[object, object], context: str, active: set[int]
) -> JSONObject:
    _enter_container(value, context, active)
    try:
        result: JSONObject = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError(
                    f"{context} keys must be strings, got {type(key).__name__}"
                )
            result[key] = _validate_json_value(item, f"{context}.{key}", active)
        return result
    finally:
        active.discard(id(value))


def _enter_container(value: object, context: str, active: set[int]) -> None:
    if id(value) in active:
        raise ValueError(f"{context} must not contain a reference cycle")
    active.add(id(value))


def freeze_json_value(value: JSONValue) -> FrozenJSONValue:
    """Recursively freeze a validated value into proxies and tuples."""

    if isinstance(value, dict):
        return MappingProxyType(
            {key: freeze_json_value(item) for key, item in value.items()}
        )
    if isinstance(value, list):
        return _FrozenJSONArray(freeze_json_value(item) for item in value)
    return value


def thaw_json_value(value: JSONValue | FrozenJSONValue) -> JSONValue:
    """Return a fresh mutable JSON copy of a frozen or regular value."""

    if isinstance(value, Mapping):
        return {key: thaw_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [thaw_json_value(item) for item in value]
    return value


# Frozen dataclasses use this mixin to support deepcopy and pickle despite
# MappingProxyType fields. Pickle restores init fields through the constructor;
# non-init fields are rejected because the constructor cannot restore them.


_T = TypeVar("_T", bound="ImmutableValueMixin")


def thaw_mapping_proxies(value: object) -> object:
    """Replace nested mapping proxies with dictionaries in supported containers."""

    if isinstance(value, MappingProxyType):
        return {key: thaw_mapping_proxies(item) for key, item in value.items()}
    if type(value) is dict:
        return {key: thaw_mapping_proxies(item) for key, item in value.items()}
    if type(value) is tuple:
        return tuple(thaw_mapping_proxies(item) for item in value)
    if type(value) is list:
        return [thaw_mapping_proxies(item) for item in value]
    return value


def _rebuild_immutable_value(
    cls: type[_T],
    init_kwargs: tuple[tuple[str, object], ...],
    proxy_fields: tuple[str, ...],
) -> _T:
    instance = cls(**dict(init_kwargs))
    # Restore proxy fields if the constructor did not freeze them.
    for name in proxy_fields:
        current = getattr(instance, name)
        if isinstance(current, Mapping) and not isinstance(current, MappingProxyType):
            object.__setattr__(instance, name, MappingProxyType(dict(current)))
    return instance


class ImmutableValueMixin:
    """Copy/pickle hooks for transitively immutable frozen dataclasses."""

    __slots__ = ()

    def __copy__(self: _T) -> _T:
        return self

    def __deepcopy__(self: _T, memo: dict[int, object]) -> _T:
        memo[id(self)] = self
        return self

    def __reduce__(self) -> tuple[Any, ...]:
        cls = type(self)
        if not is_dataclass(self):
            raise TypeError(
                f"{cls.__name__} mixes in ImmutableValueMixin but is not a dataclass"
            )
        non_init = tuple(f.name for f in fields(self) if not f.init)
        if non_init:
            raise TypeError(
                f"{cls.__name__} cannot be pickled: non-init fields {non_init} "
                "would not be restored by the constructor"
            )
        init_kwargs: list[tuple[str, object]] = []
        proxy_fields: list[str] = []
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, MappingProxyType):
                proxy_fields.append(f.name)
            init_kwargs.append((f.name, thaw_mapping_proxies(value)))
        return (
            _rebuild_immutable_value,
            (cls, tuple(init_kwargs), tuple(proxy_fields)),
        )


__all__ = [
    "ImmutableValueMixin",
    "JsonDecodeBudget",
    "bounded_json_dumps",
    "JSONObject",
    "JSONScalar",
    "JSONValue",
    "FrozenJSONArray",
    "FrozenJSONObject",
    "FrozenJSONValue",
    "freeze_json_value",
    "is_json_value",
    "thaw_json_value",
    "thaw_mapping_proxies",
    "validate_json_object",
    "validate_json_value",
]
