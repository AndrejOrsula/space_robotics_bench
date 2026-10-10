"""Canonical numeric Gymnasium projections for Dreamer process messages."""

from __future__ import annotations

import base64
import copy
import hashlib
import json
from collections import OrderedDict
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, NoReturn

import numpy as np
from gymnasium import spaces

from .process_protocol import MAX_ARRAYS, ArraySchema

__all__ = ["DreamerProcessProjection", "DreamerProcessProjectionError"]


# Keep local limits to import only the protocol types used by the wire contract.
_MAX_HEADER_BYTES = 64 * 1024
_MAX_PAYLOAD_BYTES = 64 * 1024 * 1024
_MAX_RANK = 8
_MAX_DIMENSION = 1_000_000
_MAX_ELEMENTS = 16_000_000
_MAX_DEPTH = 16
_MAX_TREE_NODES = MAX_ARRAYS * (_MAX_DEPTH + 1) * 2
_MAX_KEY_BYTES = 4096
_MAX_BOUND_RAW_BYTES = (_MAX_HEADER_BYTES // 4) * 3
_MAX_DISCRETE_N = 2**63 - 1
_REAL_DTYPE_KINDS = frozenset("biuf")

_TOP_LEVEL_FIELDS = frozenset({"version", "tree", "leaves"})
_LEAF_FIELDS = frozenset({"ref", "name", "path", "schema", "space"})
_ARRAY_FIELDS = frozenset({"dtype", "shape"})
_BOUND_FIELDS = frozenset({"dtype", "shape", "nbytes", "data"})
_BROADCAST_BOUND_FIELDS = _BOUND_FIELDS | {"broadcast"}
_BOX_FIELDS = frozenset({"kind", "dtype", "shape", "low", "high"})
_DISCRETE_FIELDS = frozenset({"kind", "n", "start", "dtype", "shape"})

_PathSegment = tuple[str, str | int]
_Path = tuple[_PathSegment, ...]


class DreamerProcessProjectionError(ValueError):
    """Raised when a Gym tree, projection metadata, or wire value is invalid."""


@dataclass(frozen=True, slots=True)
class _ProjectionLeaf:
    name: str
    path: _Path
    schema: ArraySchema
    kind: str
    n: int | None = None
    start: int | None = None
    low: np.ndarray | None = None
    high: np.ndarray | None = None


def _fail(message: str, cause: BaseException | None = None) -> NoReturn:
    if cause is None:
        raise DreamerProcessProjectionError(message)
    raise DreamerProcessProjectionError(message) from cause


def _exact_dict(value: Any, fields: frozenset[str], *, label: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != fields:
        _fail(f"{label} must contain exactly {sorted(fields)!r}")
    return value


def _strict_int(value: Any, *, label: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        _fail(f"{label} must be an integer in [{minimum}, {maximum}]")
    return value


def _strict_text(
    value: Any,
    *,
    label: str,
    max_bytes: int,
    allow_empty: bool = False,
) -> str:
    if type(value) is not str:
        _fail(f"{label} must be a string")
    if not allow_empty and not value:
        _fail(f"{label} must not be empty")
    try:
        encoded = value.encode("utf-8")
    except UnicodeEncodeError as exc:
        _fail(f"{label} is not valid Unicode", exc)
    if len(encoded) > max_bytes:
        _fail(f"{label} exceeds {max_bytes} bytes")
    if any(ord(character) < 32 or ord(character) == 127 for character in value):
        _fail(f"{label} contains a control character")
    return value


def _dtype_parts(value: Any, *, label: str) -> tuple[np.dtype[Any], str]:
    if not isinstance(value, (str, np.dtype)) and not isinstance(value, type):
        _fail(f"{label} must be a NumPy dtype")
    try:
        dtype = np.dtype(value)
    except (TypeError, ValueError) as exc:
        _fail(f"{label} is not a valid NumPy dtype", exc)
    if dtype.kind not in _REAL_DTYPE_KINDS:
        _fail(f"{label} must be numeric or boolean, not {dtype!r}")
    canonical = dtype.str
    return dtype, canonical


def _strict_metadata_dtype(value: Any, *, label: str) -> tuple[np.dtype[Any], str]:
    if type(value) is not str:
        _fail(f"{label} must be a canonical dtype string")
    dtype, canonical = _dtype_parts(value, label=label)
    if value != canonical:
        _fail(f"{label} must use canonical dtype spelling {canonical!r}")
    return dtype, canonical


def _strict_shape(value: Any, *, label: str) -> tuple[int, ...]:
    if type(value) is not list:
        _fail(f"{label} must be a list of dimensions")
    if len(value) > _MAX_RANK:
        _fail(f"{label} exceeds rank {_MAX_RANK}")
    shape: list[int] = []
    elements = 1
    for index, dimension in enumerate(value):
        dimension = _strict_int(
            dimension,
            label=f"{label}[{index}]",
            minimum=0,
            maximum=_MAX_DIMENSION,
        )
        shape.append(dimension)
        if dimension != 0:
            if elements > _MAX_ELEMENTS // dimension:
                _fail(f"{label} exceeds {_MAX_ELEMENTS} elements")
            elements *= dimension
    return tuple(shape)


def _schema_nbytes(schema: ArraySchema, *, label: str) -> int:
    try:
        nbytes = schema.nbytes
    except (TypeError, ValueError, OverflowError) as exc:
        _fail(f"{label} has an invalid array size", exc)
    if type(nbytes) is not int or not 0 <= nbytes <= _MAX_PAYLOAD_BYTES:
        _fail(f"{label} exceeds the payload size limit")
    return nbytes


def _schema_from_array(value: np.ndarray, *, label: str) -> ArraySchema:
    try:
        schema = ArraySchema.from_array(value)
    except (TypeError, ValueError, OverflowError) as exc:
        _fail(f"{label} is not an allowed numeric ndarray", exc)
    if not isinstance(schema, ArraySchema):  # pragma: no cover - protocol invariant
        _fail(f"{label} did not produce an ArraySchema")
    _dtype_parts(schema.dtype, label=f"{label}.dtype")
    if schema.shape != tuple(value.shape):
        _fail(f"{label} has an invalid shape")
    _schema_nbytes(schema, label=label)
    return schema


def _schema_from_metadata(value: Any, *, label: str) -> ArraySchema:
    descriptor = _exact_dict(value, _ARRAY_FIELDS, label=label)
    _dtype, canonical = _strict_metadata_dtype(
        descriptor["dtype"], label=f"{label}.dtype"
    )
    shape = _strict_shape(descriptor["shape"], label=f"{label}.shape")
    try:
        schema = ArraySchema.from_metadata({"dtype": canonical, "shape": list(shape)})
    except (TypeError, ValueError, OverflowError) as exc:
        _fail(f"{label} is not a valid ArraySchema", exc)
    if schema.dtype != canonical or schema.shape != shape:
        _fail(f"{label} is not canonical")
    _schema_nbytes(schema, label=label)
    return schema


def _canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    try:
        encoded = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
    except (TypeError, ValueError, UnicodeEncodeError) as exc:
        _fail("projection metadata is not canonical JSON", exc)
    if len(encoded) > _MAX_HEADER_BYTES:
        _fail("projection metadata exceeds the protocol header limit")
    return encoded


def _path_metadata(path: _Path) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for kind, value in path:
        if kind == "dict":
            result.append({"kind": "dict", "key": value})
        else:
            result.append({"kind": "tuple", "index": value})
    return result


def _public_path(path: _Path) -> tuple[str | int, ...]:
    return tuple(value for _kind, value in path)


def _validate_key(value: Any, *, label: str) -> str:
    return _strict_text(value, label=label, max_bytes=_MAX_KEY_BYTES, allow_empty=True)


def _readonly_c_copy(value: np.ndarray, *, label: str) -> np.ndarray:
    try:
        result = np.array(value, copy=True, order="C")
    except (TypeError, ValueError) as exc:
        _fail(f"{label} could not be copied as a contiguous ndarray", exc)
    result.setflags(write=False)
    return result


def _check_bounds(low: np.ndarray, high: np.ndarray, *, label: str) -> None:
    if low.dtype.kind == "f":
        try:
            if bool(np.isnan(low).any()) or bool(np.isnan(high).any()):
                _fail(f"{label} bounds must not contain NaN")
        except (TypeError, ValueError) as exc:
            _fail(f"{label} bounds could not be checked", exc)
    try:
        if bool(np.greater(low, high).any()):
            _fail(f"{label} has reversed bounds")
    except (TypeError, ValueError) as exc:
        _fail(f"{label} bounds could not be ordered", exc)


def _base64_length(nbytes: int) -> int:
    return 4 * ((nbytes + 2) // 3)


def _uniform_scalar_bytes(value: np.ndarray) -> bytes | None:
    """Return one exact scalar encoding when every C-order element matches."""

    if value.size == 0:
        return None
    itemsize = value.dtype.itemsize
    scalar: bytes | None = None
    iterator = np.nditer(
        value,
        flags=["buffered", "external_loop"],
        op_flags=["readonly"],
        order="C",
        buffersize=65_536,
    )
    for chunk in iterator:
        bounded_chunk = np.array(chunk, copy=True, order="C")
        byte_rows = bounded_chunk.view(np.uint8).reshape(-1, itemsize)
        chunk_scalar = byte_rows[0].tobytes()
        if scalar is None:
            scalar = chunk_scalar
        if scalar != chunk_scalar or not bool(np.all(byte_rows == byte_rows[0])):
            return None
    return scalar


def _bound_descriptor(
    value: np.ndarray, schema: ArraySchema, *, label: str
) -> dict[str, Any]:
    nbytes = _schema_nbytes(schema, label=label)
    try:
        scalar = _uniform_scalar_bytes(value)
        if scalar is not None:
            return {
                "broadcast": True,
                "data": base64.b64encode(scalar).decode("ascii"),
                "dtype": schema.dtype,
                "nbytes": len(scalar),
                "shape": list(schema.shape),
            }
        if nbytes > _MAX_BOUND_RAW_BYTES:
            _fail(f"{label} is too large for bounded metadata")
        encoded_length = _base64_length(nbytes)
        if encoded_length > _MAX_HEADER_BYTES:
            _fail(f"{label} exceeds the protocol header limit")
        raw = value.tobytes(order="C")
        data = base64.b64encode(raw).decode("ascii")
    except DreamerProcessProjectionError:
        raise
    except (TypeError, ValueError, UnicodeError) as exc:
        _fail(f"{label} could not be encoded canonically", exc)
    if len(raw) != nbytes or len(data) != encoded_length:
        _fail(f"{label} has an incorrect encoded byte count")
    return {
        "data": data,
        "dtype": schema.dtype,
        "nbytes": nbytes,
        "shape": list(schema.shape),
    }


def _decode_bound(
    value: Any,
    schema: ArraySchema,
    *,
    label: str,
    aggregate: list[int],
) -> np.ndarray:
    if type(value) is not dict:
        _fail(f"{label} must contain exactly {sorted(_BOUND_FIELDS)!r}")
    broadcast = "broadcast" in value
    descriptor = _exact_dict(
        value,
        _BROADCAST_BOUND_FIELDS if broadcast else _BOUND_FIELDS,
        label=label,
    )
    if broadcast and descriptor["broadcast"] is not True:
        _fail(f"{label}.broadcast must be true")
    bound_schema = _schema_from_metadata(
        {"dtype": descriptor["dtype"], "shape": descriptor["shape"]},
        label=f"{label}.schema",
    )
    if bound_schema != schema:
        _fail(f"{label} schema does not match its Box leaf")
    nbytes = _strict_int(
        descriptor["nbytes"],
        label=f"{label}.nbytes",
        minimum=0,
        maximum=_MAX_PAYLOAD_BYTES,
    )
    expected_nbytes = (
        np.dtype(schema.dtype).itemsize
        if broadcast
        else _schema_nbytes(schema, label=label)
    )
    if nbytes != expected_nbytes:
        _fail(f"{label}.nbytes does not match its schema")
    if nbytes > _MAX_BOUND_RAW_BYTES:
        _fail(f"{label} is too large for bounded metadata")
    if type(descriptor["data"]) is not str:
        _fail(f"{label}.data must be a base64 string")
    try:
        encoded = descriptor["data"].encode("ascii")
    except UnicodeEncodeError as exc:
        _fail(f"{label}.data must contain ASCII base64", exc)
    if len(encoded) > _MAX_HEADER_BYTES or len(encoded) != _base64_length(nbytes):
        _fail(f"{label}.data has a non-canonical encoded length")
    if aggregate[0] + nbytes > _MAX_PAYLOAD_BYTES:
        _fail("decoded Box bounds exceed the payload size limit")
    try:
        raw = base64.b64decode(encoded, validate=True)
    except (ValueError, TypeError) as exc:
        _fail(f"{label}.data is not valid base64", exc)
    if len(raw) != nbytes or base64.b64encode(raw) != encoded:
        _fail(f"{label}.data is not canonical base64")
    aggregate[0] += nbytes
    try:
        array = np.frombuffer(raw, dtype=np.dtype(schema.dtype), count=-1)
        if broadcast:
            array = np.broadcast_to(array.reshape(()), schema.shape)
        else:
            array = array.reshape(schema.shape).copy(order="C")
    except (TypeError, ValueError) as exc:
        _fail(f"{label} cannot be decoded using its schema", exc)
    array.setflags(write=False)
    return array


def _space_box_leaf(
    space: spaces.Box[Any], *, name: str, path: _Path, aggregate: list[int]
) -> _ProjectionLeaf:
    dtype, canonical_dtype = _dtype_parts(space.dtype, label="Box.dtype")
    low = np.asarray(space.low)
    high = np.asarray(space.high)
    low_schema = _schema_from_array(low, label="Box.low")
    high_schema = _schema_from_array(high, label="Box.high")
    expected_shape = tuple(space.shape)
    if low_schema.shape != expected_shape or high_schema.shape != expected_shape:
        _fail("Box bounds do not match the declared shape")
    if low_schema.dtype != canonical_dtype or high_schema.dtype != canonical_dtype:
        _fail("Box bounds do not match the declared dtype")
    if dtype.kind not in _REAL_DTYPE_KINDS:
        _fail("Box dtype must be numeric or boolean")
    _check_bounds(low, high, label="Box")
    schema = low_schema
    nbytes = _schema_nbytes(schema, label="Box")
    if aggregate[0] + 2 * nbytes > _MAX_PAYLOAD_BYTES:
        _fail("decoded Box bounds exceed the payload size limit")
    aggregate[0] += 2 * nbytes
    return _ProjectionLeaf(
        name=name,
        path=path,
        schema=schema,
        kind="box",
        low=_readonly_c_copy(low, label="Box.low"),
        high=_readonly_c_copy(high, label="Box.high"),
    )


def _integer_value(value: Any, *, label: str, minimum: int, maximum: int) -> int:
    if isinstance(value, (bool, np.bool_)):
        _fail(f"{label} must not be boolean")
    if isinstance(value, np.integer):
        value = int(value)
    return _strict_int(value, label=label, minimum=minimum, maximum=maximum)


def _space_discrete_leaf(
    space: spaces.Discrete[Any], *, name: str, path: _Path
) -> _ProjectionLeaf:
    n = _integer_value(space.n, label="Discrete.n", minimum=1, maximum=_MAX_DISCRETE_N)
    start = _integer_value(
        space.start,
        label="Discrete.start",
        minimum=-_MAX_DISCRETE_N,
        maximum=_MAX_DISCRETE_N,
    )
    if start != 0:
        _fail("Discrete.start must be zero")
    dtype, canonical_dtype = _dtype_parts(space.dtype, label="Discrete.dtype")
    if dtype.kind not in "iu" or dtype.kind == "b":
        _fail("Discrete.dtype must be an integer dtype")
    if dtype != np.dtype(np.int64) or canonical_dtype != np.dtype(np.int64).str:
        _fail("Discrete.dtype must be canonical int64")
    if tuple(space.shape) != ():
        _fail("Discrete.shape must be scalar")
    try:
        scalar = np.asarray(np.int64(0), dtype=dtype)
    except (TypeError, ValueError) as exc:
        _fail("Discrete scalar schema could not be built", exc)
    schema = _schema_from_array(scalar, label="Discrete")
    if schema.dtype != canonical_dtype or schema.shape != ():
        _fail("Discrete schema is not canonical")
    _schema_nbytes(schema, label="Discrete")
    return _ProjectionLeaf(
        name=name,
        path=path,
        schema=schema,
        kind="discrete",
        n=n,
        start=start,
    )


def _leaf_name(index: int) -> str:
    return f"leaf_{index:04d}"


def _build_tree(
    space: Any,
    *,
    path: _Path,
    depth: int,
    leaves: list[_ProjectionLeaf],
    aggregate: list[int],
    nodes: list[int],
) -> dict[str, Any]:
    nodes[0] += 1
    if nodes[0] > _MAX_TREE_NODES:
        _fail("Gym tree exceeds the node limit")
    if depth > _MAX_DEPTH:
        _fail(f"Gym tree exceeds maximum depth {_MAX_DEPTH}")
    if isinstance(space, spaces.Box):
        if len(leaves) >= MAX_ARRAYS:
            _fail(f"Gym tree exceeds the {MAX_ARRAYS}-leaf limit")
        name = _leaf_name(len(leaves))
        leaves.append(_space_box_leaf(space, name=name, path=path, aggregate=aggregate))
        return {"kind": "leaf", "ref": len(leaves) - 1}
    if isinstance(space, spaces.Discrete):
        if len(leaves) >= MAX_ARRAYS:
            _fail(f"Gym tree exceeds the {MAX_ARRAYS}-leaf limit")
        name = _leaf_name(len(leaves))
        leaves.append(_space_discrete_leaf(space, name=name, path=path))
        return {"kind": "leaf", "ref": len(leaves) - 1}
    if isinstance(space, spaces.Dict):
        if not isinstance(space.spaces, Mapping) or not space.spaces:
            _fail("Gym Dict trees must not be empty")
        children: list[dict[str, Any]] = []
        seen: set[str] = set()
        for key, child_space in space.spaces.items():
            key = _validate_key(key, label="Gym Dict key")
            if key in seen:
                _fail("Gym Dict keys must be unique")
            seen.add(key)
            children.append(
                {
                    "key": key,
                    "node": _build_tree(
                        child_space,
                        path=path + (("dict", key),),
                        depth=depth + 1,
                        leaves=leaves,
                        aggregate=aggregate,
                        nodes=nodes,
                    ),
                }
            )
        return {"kind": "dict", "children": children}
    if isinstance(space, spaces.Tuple):
        if not space.spaces:
            _fail("Gym Tuple trees must not be empty")
        children = []
        for index, child_space in enumerate(space.spaces):
            children.append(
                {
                    "index": index,
                    "node": _build_tree(
                        child_space,
                        path=path + (("tuple", index),),
                        depth=depth + 1,
                        leaves=leaves,
                        aggregate=aggregate,
                        nodes=nodes,
                    ),
                }
            )
        return {"kind": "tuple", "children": children}
    _fail(
        "unsupported Gym space at "
        f"{_public_path(path)!r}; expected Box, Discrete, Dict, or Tuple"
    )


def _leaf_space_metadata(leaf: _ProjectionLeaf) -> dict[str, Any]:
    schema = leaf.schema
    if leaf.kind == "box":
        if leaf.low is None or leaf.high is None:
            _fail("internal Box leaf is missing bounds")
        return {
            "dtype": schema.dtype,
            "high": _bound_descriptor(leaf.high, schema, label=f"{leaf.name}.high"),
            "kind": "box",
            "low": _bound_descriptor(leaf.low, schema, label=f"{leaf.name}.low"),
            "shape": list(schema.shape),
        }
    if leaf.kind == "discrete":
        if leaf.n is None or leaf.start is None:
            _fail("internal Discrete leaf is missing cardinality")
        return {
            "dtype": schema.dtype,
            "kind": "discrete",
            "n": leaf.n,
            "shape": list(schema.shape),
            "start": leaf.start,
        }
    _fail(f"unknown internal leaf kind {leaf.kind!r}")


def _metadata_for(
    tree: dict[str, Any], leaves: tuple[_ProjectionLeaf, ...]
) -> tuple[dict[str, Any], bytes]:
    metadata = {
        "leaves": [
            {
                "name": leaf.name,
                "path": _path_metadata(leaf.path),
                "ref": index,
                "schema": {
                    "dtype": leaf.schema.dtype,
                    "shape": list(leaf.schema.shape),
                },
                "space": _leaf_space_metadata(leaf),
            }
            for index, leaf in enumerate(leaves)
        ],
        "tree": tree,
        "version": 2,
    }
    encoded = _canonical_json_bytes(metadata)
    return metadata, encoded


def _parse_tree(value: Any) -> tuple[dict[str, Any], dict[int, _Path]]:
    references: dict[int, _Path] = {}
    nodes = [0]

    def visit(node: Any, path: _Path, depth: int) -> dict[str, Any]:
        nodes[0] += 1
        if nodes[0] > _MAX_TREE_NODES:
            _fail("projection tree exceeds the node limit")
        if depth > _MAX_DEPTH:
            _fail(f"projection tree exceeds maximum depth {_MAX_DEPTH}")
        if type(node) is not dict or type(node.get("kind")) is not str:
            _fail("projection tree node must have a string kind")
        kind = node["kind"]
        if kind == "leaf":
            _exact_dict(node, frozenset({"kind", "ref"}), label="leaf node")
            ref = _strict_int(
                node["ref"],
                label="leaf.ref",
                minimum=0,
                maximum=MAX_ARRAYS - 1,
            )
            if ref in references:
                _fail(f"projection leaf ref {ref} is duplicated")
            references[ref] = path
            return {"kind": "leaf", "ref": ref}
        if kind == "dict":
            _exact_dict(node, frozenset({"kind", "children"}), label="Dict node")
            children = node["children"]
            if type(children) is not list or not children:
                _fail("projection Dict children must be a non-empty list")
            if len(children) > MAX_ARRAYS:
                _fail("projection Dict has too many children")
            result: list[dict[str, Any]] = []
            seen: set[str] = set()
            for child in children:
                _exact_dict(child, frozenset({"key", "node"}), label="Dict child")
                key = _validate_key(child["key"], label="Dict child key")
                if key in seen:
                    _fail("projection Dict keys must be unique")
                seen.add(key)
                result.append(
                    {
                        "key": key,
                        "node": visit(
                            child["node"],
                            path + (("dict", key),),
                            depth + 1,
                        ),
                    }
                )
            return {"kind": "dict", "children": result}
        if kind == "tuple":
            _exact_dict(node, frozenset({"kind", "children"}), label="Tuple node")
            children = node["children"]
            if type(children) is not list or not children:
                _fail("projection Tuple children must be a non-empty list")
            if len(children) > MAX_ARRAYS:
                _fail("projection Tuple has too many children")
            result = []
            for expected_index, child in enumerate(children):
                _exact_dict(child, frozenset({"index", "node"}), label="Tuple child")
                index = _strict_int(
                    child["index"],
                    label="Tuple child index",
                    minimum=0,
                    maximum=MAX_ARRAYS - 1,
                )
                if index != expected_index:
                    _fail("Tuple child indices must be canonical and ordered")
                result.append(
                    {
                        "index": index,
                        "node": visit(
                            child["node"],
                            path + (("tuple", index),),
                            depth + 1,
                        ),
                    }
                )
            return {"kind": "tuple", "children": result}
        _fail(f"unsupported projection tree kind {kind!r}")

    tree = visit(value, (), 0)
    if not references:
        _fail("projection tree must contain at least one leaf")
    expected_refs = set(range(len(references)))
    if set(references) != expected_refs:
        _fail("projection leaf refs must be a contiguous canonical sequence")
    return tree, references


def _parse_path(value: Any, *, expected: _Path, label: str) -> _Path:
    if type(value) is not list:
        _fail(f"{label} must be a list")
    result: list[_PathSegment] = []
    for index, segment in enumerate(value):
        if type(segment) is not dict or type(segment.get("kind")) is not str:
            _fail(f"{label}[{index}] must be a typed path segment")
        kind = segment["kind"]
        if kind == "dict":
            _exact_dict(segment, frozenset({"kind", "key"}), label=f"{label}[{index}]")
            key = _validate_key(segment["key"], label=f"{label}[{index}].key")
            result.append(("dict", key))
        elif kind == "tuple":
            _exact_dict(
                segment,
                frozenset({"kind", "index"}),
                label=f"{label}[{index}]",
            )
            path_index = _strict_int(
                segment["index"],
                label=f"{label}[{index}].index",
                minimum=0,
                maximum=MAX_ARRAYS - 1,
            )
            result.append(("tuple", path_index))
        else:
            _fail(f"{label}[{index}] has an unsupported segment kind")
    parsed = tuple(result)
    if parsed != expected:
        _fail(f"{label} does not match the projection tree")
    return parsed


def _parse_space(
    value: Any,
    schema: ArraySchema,
    *,
    label: str,
    aggregate: list[int],
) -> _ProjectionLeaf:
    if type(value) is not dict or type(value.get("kind")) is not str:
        _fail(f"{label} must be a space descriptor")
    kind = value["kind"]
    if kind == "box":
        descriptor = _exact_dict(value, _BOX_FIELDS, label=label)
        _dtype, canonical_dtype = _strict_metadata_dtype(
            descriptor["dtype"], label=f"{label}.dtype"
        )
        shape = _strict_shape(descriptor["shape"], label=f"{label}.shape")
        if canonical_dtype != schema.dtype or shape != schema.shape:
            _fail(f"{label} does not match its leaf schema")
        low = _decode_bound(
            descriptor["low"], schema, label=f"{label}.low", aggregate=aggregate
        )
        high = _decode_bound(
            descriptor["high"], schema, label=f"{label}.high", aggregate=aggregate
        )
        _check_bounds(low, high, label=label)
        return _ProjectionLeaf(
            name="",
            path=(),
            schema=schema,
            kind="box",
            low=low,
            high=high,
        )
    if kind == "discrete":
        descriptor = _exact_dict(value, _DISCRETE_FIELDS, label=label)
        n = _strict_int(
            descriptor["n"],
            label=f"{label}.n",
            minimum=1,
            maximum=_MAX_DISCRETE_N,
        )
        start = _strict_int(
            descriptor["start"],
            label=f"{label}.start",
            minimum=-_MAX_DISCRETE_N,
            maximum=_MAX_DISCRETE_N,
        )
        if start != 0:
            _fail(f"{label}.start must be zero")
        _dtype, canonical_dtype = _strict_metadata_dtype(
            descriptor["dtype"], label=f"{label}.dtype"
        )
        shape = _strict_shape(descriptor["shape"], label=f"{label}.shape")
        if canonical_dtype != np.dtype(np.int64).str:
            _fail(f"{label}.dtype must be canonical int64")
        if canonical_dtype != schema.dtype or shape != () or schema.shape != ():
            _fail(f"{label} does not match a scalar Discrete schema")
        return _ProjectionLeaf(
            name="",
            path=(),
            schema=schema,
            kind="discrete",
            n=n,
            start=start,
        )
    _fail(f"{label} has an unsupported space kind {kind!r}")


def _parse_metadata(
    metadata: dict[str, Any],
) -> tuple[dict[str, Any], tuple[_ProjectionLeaf, ...], bytes]:
    if type(metadata) is not dict:
        _fail("projection metadata must be a JSON object")
    if set(metadata) != _TOP_LEVEL_FIELDS:
        _fail("projection metadata has unknown or missing top-level fields")
    if type(metadata["version"]) is not int or metadata["version"] != 2:
        _fail("only projection metadata version 2 is supported")
    input_encoded = _canonical_json_bytes(metadata)
    tree, references = _parse_tree(metadata["tree"])
    raw_leaves = metadata["leaves"]
    if type(raw_leaves) is not list or not raw_leaves:
        _fail("projection leaves must be a non-empty list")
    if len(raw_leaves) > MAX_ARRAYS or len(raw_leaves) != len(references):
        _fail("projection leaves exceed the bounded canonical count")
    leaves: list[_ProjectionLeaf] = []
    aggregate = [0]
    for index, raw_leaf in enumerate(raw_leaves):
        descriptor = _exact_dict(raw_leaf, _LEAF_FIELDS, label=f"leaves[{index}]")
        ref = _strict_int(
            descriptor["ref"],
            label=f"leaves[{index}].ref",
            minimum=0,
            maximum=MAX_ARRAYS - 1,
        )
        if ref != index:
            _fail("projection leaf refs must match canonical leaf numbering")
        name = descriptor["name"]
        if type(name) is not str or name != _leaf_name(index):
            _fail(f"leaves[{index}].name is not a canonical safe wire name")
        path = _parse_path(
            descriptor["path"],
            expected=references[index],
            label=f"leaves[{index}].path",
        )
        schema = _schema_from_metadata(
            descriptor["schema"], label=f"leaves[{index}].schema"
        )
        leaf = _parse_space(
            descriptor["space"],
            schema,
            label=f"leaves[{index}].space",
            aggregate=aggregate,
        )
        leaves.append(
            _ProjectionLeaf(
                name=name,
                path=path,
                schema=schema,
                kind=leaf.kind,
                n=leaf.n,
                start=leaf.start,
                low=leaf.low,
                high=leaf.high,
            )
        )
    wire_bytes = 0
    for leaf in leaves:
        wire_bytes += _schema_nbytes(leaf.schema, label=f"{leaf.name}.schema")
        if wire_bytes > _MAX_PAYLOAD_BYTES:
            _fail("projection wire arrays exceed the payload size limit")
    leaf_tuple = tuple(leaves)
    canonical, _encoded = _metadata_for(tree, leaf_tuple)
    if canonical != metadata:
        legacy_uniform = copy.deepcopy(metadata)
        for index, leaf in enumerate(leaf_tuple):
            if leaf.kind != "box":
                continue
            raw_space = legacy_uniform["leaves"][index]["space"]
            for name, bound in (("low", leaf.low), ("high", leaf.high)):
                if bound is None:
                    _fail("projection metadata is missing a Box bound")
                descriptor = raw_space[name]
                compact = _bound_descriptor(
                    bound, leaf.schema, label=f"leaves[{index}].space.{name}"
                )
                if "broadcast" not in descriptor and "broadcast" in compact:
                    raw_space[name] = compact
        if legacy_uniform != canonical:
            _fail("projection metadata is not canonical")
    return tree, leaf_tuple, input_encoded


def _host_array(value: Any, *, label: str) -> np.ndarray:
    candidate = value
    if not isinstance(candidate, np.ndarray):
        detach = getattr(candidate, "detach", None)
        if callable(detach):
            try:
                candidate = detach()
            except (AttributeError, RuntimeError, TypeError) as exc:
                _fail(f"{label} could not be detached to CPU", exc)
        cpu = getattr(candidate, "cpu", None)
        if callable(cpu):
            try:
                candidate = cpu()
            except (AttributeError, RuntimeError, TypeError) as exc:
                _fail(f"{label} could not be moved to CPU", exc)
        numpy_method = getattr(candidate, "numpy", None)
        if callable(numpy_method):
            try:
                candidate = numpy_method()
            except (AttributeError, RuntimeError, TypeError) as exc:
                _fail(f"{label} could not be converted to NumPy", exc)
    try:
        array = np.asarray(candidate)
    except (TypeError, ValueError) as exc:
        _fail(f"{label} is not NumPy-compatible", exc)
    if not isinstance(array, np.ndarray):  # pragma: no cover - NumPy invariant
        _fail(f"{label} is not an ndarray")
    return array


def _copy_wire_array(value: Any, schema: ArraySchema, *, label: str) -> np.ndarray:
    if not isinstance(value, np.ndarray):
        _fail(f"{label} must be a NumPy ndarray")
    actual = _schema_from_array(value, label=label)
    if actual != schema:
        _fail(
            f"{label} has dtype/shape {actual.dtype} {actual.shape}, "
            f"expected {schema.dtype} {schema.shape}"
        )
    return np.array(value, copy=True, order="C")


class DreamerProcessProjection:
    """Project one bounded Gymnasium space tree onto named wire arrays."""

    __slots__ = (
        "_tree",
        "_leaves",
        "_metadata",
        "_metadata_bytes",
        "_schema_hash",
    )

    def __init__(self, space: Any) -> None:
        leaves: list[_ProjectionLeaf] = []
        aggregate = [0]
        tree = _build_tree(
            space,
            path=(),
            depth=0,
            leaves=leaves,
            aggregate=aggregate,
            nodes=[0],
        )
        if not leaves:
            _fail("projection must contain at least one leaf")
        if len(leaves) > MAX_ARRAYS:
            _fail(f"projection exceeds the {MAX_ARRAYS}-leaf limit")
        wire_bytes = 0
        for leaf in leaves:
            wire_bytes += _schema_nbytes(leaf.schema, label=f"{leaf.name}.schema")
            if wire_bytes > _MAX_PAYLOAD_BYTES:
                _fail("projection wire arrays exceed the payload size limit")
        leaf_tuple = tuple(leaves)
        metadata, metadata_bytes = _metadata_for(tree, leaf_tuple)
        self._install(tree, leaf_tuple, metadata, metadata_bytes)

    @classmethod
    def from_metadata(cls, metadata: Any) -> "DreamerProcessProjection":
        """Reconstruct a projection only from strict canonical v2 metadata."""

        if type(metadata) is not dict:
            _fail("projection metadata must be a JSON object")
        tree, leaves, metadata_bytes = _parse_metadata(metadata)
        canonical = copy.deepcopy(metadata)
        result = cls.__new__(cls)
        result._install(tree, leaves, canonical, metadata_bytes)
        return result

    def _install(
        self,
        tree: dict[str, Any],
        leaves: tuple[_ProjectionLeaf, ...],
        metadata: dict[str, Any],
        metadata_bytes: bytes,
    ) -> None:
        self._tree = copy.deepcopy(tree)
        self._leaves = tuple(leaves)
        self._metadata = copy.deepcopy(metadata)
        self._metadata_bytes = bytes(metadata_bytes)
        self._schema_hash = hashlib.sha256(self._metadata_bytes).hexdigest()

    @property
    def metadata(self) -> dict[str, Any]:
        """Return a detached copy of the complete canonical v2 metadata."""

        return copy.deepcopy(self._metadata)

    @property
    def metadata_json(self) -> str:
        """Return canonical ASCII JSON for the complete projection schema."""

        return self._metadata_bytes.decode("ascii")

    @property
    def schema_hash(self) -> str:
        """Return the SHA-256 digest of the canonical projection metadata."""

        return self._schema_hash

    @property
    def io_schema_fingerprint(self) -> str:
        """Compatibility alias for callers that use fingerprint terminology."""

        return self._schema_hash

    @property
    def wire_names(self) -> tuple[str, ...]:
        return tuple(leaf.name for leaf in self._leaves)

    @property
    def names(self) -> tuple[str, ...]:
        return self.wire_names

    @property
    def paths(self) -> tuple[tuple[str | int, ...], ...]:
        return tuple(_public_path(leaf.path) for leaf in self._leaves)

    @property
    def wire_schema(self) -> dict[str, ArraySchema]:
        return {leaf.name: leaf.schema for leaf in self._leaves}

    @property
    def wire_schemas(self) -> tuple[ArraySchema, ...]:
        return tuple(leaf.schema for leaf in self._leaves)

    @property
    def schemas(self) -> tuple[ArraySchema, ...]:
        return self.wire_schemas

    @property
    def schema(self) -> dict[str, ArraySchema]:
        return self.wire_schema

    @property
    def space(self) -> Any:
        return self.to_space()

    def to_space(self) -> Any:
        """Return a fresh Gymnasium tree with exact leaf semantics."""

        def reconstruct(node: dict[str, Any]) -> Any:
            kind = node["kind"]
            if kind == "leaf":
                leaf = self._leaves[node["ref"]]
                if leaf.kind == "box":
                    if leaf.low is None or leaf.high is None:
                        _fail("internal Box leaf is missing bounds")
                    return spaces.Box(
                        low=np.array(leaf.low, copy=True, order="C"),
                        high=np.array(leaf.high, copy=True, order="C"),
                        dtype=np.dtype(leaf.schema.dtype),
                    )
                return spaces.Discrete(leaf.n, start=leaf.start)
            if kind == "dict":
                return spaces.Dict(
                    OrderedDict(
                        (
                            child["key"],
                            reconstruct(child["node"]),
                        )
                        for child in node["children"]
                    )
                )
            if kind == "tuple":
                return spaces.Tuple(
                    tuple(reconstruct(child["node"]) for child in node["children"])
                )
            _fail(f"unknown internal tree kind {kind!r}")

        return reconstruct(copy.deepcopy(self._tree))

    def pack(self, value: Any) -> dict[str, np.ndarray]:
        """Validate a Gym tree and copy its leaves into ordered wire arrays."""

        result: dict[str, np.ndarray] = {}

        def visit(node: dict[str, Any], current: Any, path: _Path) -> None:
            kind = node["kind"]
            if kind == "leaf":
                leaf = self._leaves[node["ref"]]
                array = _host_array(current, label=f"value at {_public_path(path)!r}")
                actual = _schema_from_array(
                    array, label=f"value at {_public_path(path)!r}"
                )
                if actual != leaf.schema:
                    _fail(
                        f"value at {_public_path(path)!r} has dtype/shape "
                        f"{actual.dtype} {actual.shape}, expected "
                        f"{leaf.schema.dtype} {leaf.schema.shape}"
                    )
                result[leaf.name] = np.array(array, copy=True, order="C")
                return
            if kind == "dict":
                if not isinstance(current, Mapping):
                    _fail(f"value at {_public_path(path)!r} must be a mapping")
                expected = {child["key"] for child in node["children"]}
                actual_keys = tuple(current.keys())
                if any(type(key) is not str for key in actual_keys):
                    _fail(f"value at {_public_path(path)!r} has non-string keys")
                if set(actual_keys) != expected:
                    _fail(f"value at {_public_path(path)!r} has the wrong keys")
                for child in node["children"]:
                    key = child["key"]
                    visit(
                        child["node"],
                        current[key],
                        path + (("dict", key),),
                    )
                return
            if kind == "tuple":
                if type(current) is not tuple or len(current) != len(node["children"]):
                    _fail(f"value at {_public_path(path)!r} must match the Tuple tree")
                for child in node["children"]:
                    index = child["index"]
                    visit(
                        child["node"],
                        current[index],
                        path + (("tuple", index),),
                    )
                return
            _fail(f"unknown internal tree kind {kind!r}")

        visit(self._tree, value, ())
        if set(result) != set(self.wire_names) or len(result) != len(self._leaves):
            _fail("packed wire names do not match the projection")
        return result

    def unpack(self, value: Any) -> Any:
        """Validate wire arrays and reconstruct a fresh Gym tree."""

        if not isinstance(value, Mapping):
            _fail("wire value must be an array mapping")
        keys = tuple(value.keys())
        if any(type(key) is not str for key in keys):
            _fail("wire array names must be strings")
        expected_names = set(self.wire_names)
        if set(keys) != expected_names or len(keys) != len(expected_names):
            _fail("wire arrays have the wrong names")
        arrays: dict[str, np.ndarray] = {}
        for leaf in self._leaves:
            arrays[leaf.name] = _copy_wire_array(
                value[leaf.name], leaf.schema, label=f"wire array {leaf.name!r}"
            )

        def visit(node: dict[str, Any]) -> Any:
            kind = node["kind"]
            if kind == "leaf":
                leaf = self._leaves[node["ref"]]
                array = arrays[leaf.name]
                if leaf.kind == "discrete":
                    return np.asarray(array, dtype=np.dtype(leaf.schema.dtype))[()]
                return np.array(array, copy=True, order="C")
            if kind == "dict":
                return {
                    child["key"]: visit(child["node"]) for child in node["children"]
                }
            if kind == "tuple":
                return tuple(visit(child["node"]) for child in node["children"])
            _fail(f"unknown internal tree kind {kind!r}")

        return visit(self._tree)
