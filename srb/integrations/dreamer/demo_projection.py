"""Project canonical demonstrations onto live Dreamer spaces.

Shared by simulator and process adapters; the process projection loads lazily
so importing this module does not import optional ML or simulator providers.
"""

from __future__ import annotations

import copy
import hashlib
import json
from collections import OrderedDict
from collections.abc import Mapping, Sequence
from typing import Any, NoReturn

import gymnasium
import numpy as np

from srb.utils.dataset_schema import (
    DEMO_IO_CONTRACT_VERSION,
    validate_demo_io_contract,
)

DEMO_PROJECTION_VERSION = "srb.dreamer.demo_projection.v1"
MAX_DEMO_PROJECTION_BYTES = 64 * 1024
MAX_ACTION_DIM = 1_000_000
MAX_TERMS = 32
MAX_MAP_ENTRIES = 128
_MAX_JSON_DEPTH = 32
_MAX_JSON_ITEMS = 4096
_MAX_JSON_STRING_BYTES = MAX_DEMO_PROJECTION_BYTES


class DreamerDemoProjectionError(ValueError):
    """A canonical demonstration cannot be projected onto the live learner."""


def _fail(message: str, cause: BaseException | None = None) -> NoReturn:
    if cause is None:
        raise DreamerDemoProjectionError(message)
    raise DreamerDemoProjectionError(message) from cause


def _json_string_size(value: str, *, label: str) -> int:
    """Count compact ensure-ascii JSON bytes without encoding the string."""

    if type(value) is not str:
        _fail(f"{label} must be a JSON string")
    if len(value) > _MAX_JSON_STRING_BYTES:
        _fail(f"{label} exceeds the bounded metadata limit")
    size = 2  # JSON string quotes.
    for character in value:
        codepoint = ord(character)
        if character in {'"', "\\"}:
            size += 2
        elif codepoint < 0x20:
            size += 2 if character in "\b\f\n\r\t" else 6
        elif codepoint < 0x7F:
            size += 1
        elif codepoint <= 0xFFFF:
            size += 6
        else:
            size += 12  # ensure_ascii emits a surrogate pair.
        if size > _MAX_JSON_STRING_BYTES:
            _fail(f"{label} exceeds the bounded metadata limit")
    return size


def _json_budget_add(budget: list[int], amount: int, *, label: str) -> None:
    if amount < 0 or budget[0] > MAX_DEMO_PROJECTION_BYTES - amount:
        _fail(f"{label} exceeds the bounded metadata limit")
    budget[0] += amount


def _preflight_json(value: Any, *, label: str) -> None:
    """Preflight size and nesting before canonical JSON encoding."""

    budget = [0]
    stack: list[tuple[Any, int, str | None, bool]] = [(value, 0, None, False)]
    item_count = 0
    while stack:
        current, depth, key, closing = stack.pop()
        if closing:
            _json_budget_add(budget, 1, label=label)
            continue
        if depth > _MAX_JSON_DEPTH:
            _fail(f"{label} exceeds the metadata nesting limit")
        if key is not None:
            _json_budget_add(
                budget, _json_string_size(key, label=f"{label} key"), label=label
            )
            _json_budget_add(budget, 1, label=label)  # colon
        if isinstance(current, Mapping):
            try:
                size = len(current)
            except (TypeError, OverflowError) as exc:
                _fail(f"{label} mapping size is unavailable", exc)
            if size > _MAX_JSON_ITEMS:
                _fail(f"{label} has too many metadata items")
            _json_budget_add(budget, 1, label=label)
            stack.append((None, depth, None, True))
            entries: list[tuple[Any, Any]] = []
            try:
                for entry in current.items():
                    if len(entries) >= _MAX_JSON_ITEMS:
                        _fail(f"{label} has too many metadata items")
                    entries.append(entry)
            except DreamerDemoProjectionError:
                raise
            except (AttributeError, TypeError, ValueError, RuntimeError) as exc:
                _fail(f"{label} mapping items are unavailable", exc)
            for index in range(len(entries) - 1, -1, -1):
                entry_key, entry_value = entries[index]
                if type(entry_key) is not str:
                    _fail(f"{label} keys must be strings")
                item_count += 1
                if item_count > _MAX_JSON_ITEMS:
                    _fail(f"{label} has too many metadata items")
                if index < len(entries) - 1:
                    _json_budget_add(budget, 1, label=label)  # comma
                stack.append((entry_value, depth + 1, entry_key, False))
            continue
        if isinstance(current, (list, tuple)):
            if len(current) > _MAX_JSON_ITEMS:
                _fail(f"{label} has too many metadata items")
            _json_budget_add(budget, 1, label=label)
            stack.append((None, depth, None, True))
            for index in range(len(current) - 1, -1, -1):
                item_count += 1
                if item_count > _MAX_JSON_ITEMS:
                    _fail(f"{label} has too many metadata items")
                if index < len(current) - 1:
                    _json_budget_add(budget, 1, label=label)  # comma
                stack.append((current[index], depth + 1, None, False))
            continue
        if type(current) is str:
            _json_budget_add(
                budget, _json_string_size(current, label=label), label=label
            )
            continue
        if current is None:
            _json_budget_add(budget, 4, label=label)
            continue
        if type(current) is bool:
            _json_budget_add(budget, 4 if current else 5, label=label)
            continue
        if type(current) is int:
            # Reject enormous integers from the bit length before converting
            # them to a potentially enormous decimal string.
            if current.bit_length() > _MAX_JSON_STRING_BYTES * 4:
                _fail(f"{label} integer exceeds the bounded metadata limit")
            try:
                integer_size = len(str(current))
            except (OverflowError, ValueError) as exc:
                _fail(f"{label} integer is too large", exc)
            _json_budget_add(budget, integer_size, label=label)
            continue
        if type(current) is float:
            if not np.isfinite(current):
                _fail(f"{label} contains a non-finite number")
            _json_budget_add(budget, len(repr(current)), label=label)
            continue
        _fail(f"{label} contains a value that is not canonical JSON")


def _json_copy(value: Any, *, label: str) -> Any:
    _preflight_json(value, label=label)
    try:
        encoded = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
        if len(encoded) > MAX_DEMO_PROJECTION_BYTES:
            _fail(f"{label} exceeds the bounded metadata limit")
        return json.loads(encoded.decode("ascii"))
    except DreamerDemoProjectionError:
        raise
    except (RecursionError, TypeError, ValueError, UnicodeEncodeError) as exc:
        _fail(f"{label} must be canonical JSON", exc)


def _canonical_bytes(value: Mapping[str, Any], *, label: str) -> bytes:
    _preflight_json(value, label=label)
    try:
        encoded = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
    except (RecursionError, TypeError, ValueError, UnicodeEncodeError) as exc:
        _fail(f"{label} must be canonical JSON", exc)
    if len(encoded) > MAX_DEMO_PROJECTION_BYTES:
        _fail(f"{label} exceeds the bounded metadata limit")
    return encoded


def _strict_text(value: Any, *, label: str, allow_empty: bool = False) -> str:
    if type(value) is not str or (not allow_empty and not value):
        _fail(f"{label} must be a string")
    try:
        if len(value.encode("utf-8")) > 4096:
            _fail(f"{label} exceeds the size limit")
    except UnicodeEncodeError as exc:
        _fail(f"{label} is not valid Unicode", exc)
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        _fail(f"{label} contains a control character")
    return value


def _strict_int(
    value: Any, *, label: str, minimum: int = 0, maximum: int = 2**31 - 1
) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        _fail(f"{label} must be an integer in [{minimum}, {maximum}]")
    return value


def _dtype(value: Any, *, label: str) -> np.dtype[Any]:
    if type(value) is not str:
        _fail(f"{label} must be a dtype string")
    try:
        result = np.dtype(value)
    except (TypeError, ValueError) as exc:
        _fail(f"{label} is not a NumPy dtype", exc)
    if result.kind not in "biuf":
        _fail(f"{label} must be real numeric or boolean")
    return result


def _shape(value: Any, *, label: str) -> tuple[int, ...]:
    if type(value) is not list or len(value) > 8:
        _fail(f"{label} must be a bounded shape list")
    result: list[int] = []
    elements = 1
    for index, item in enumerate(value):
        dimension = _strict_int(
            item,
            label=f"{label}[{index}]",
            minimum=0,
            maximum=1_000_000,
        )
        if dimension and elements > 16_000_000 // dimension:
            _fail(f"{label} exceeds the element limit")
        elements *= dimension
        result.append(dimension)
    return tuple(result)


def _path(value: Any, *, label: str) -> tuple[str | int, ...]:
    if type(value) is not list or len(value) > 16:
        _fail(f"{label} must be a bounded path list")
    result: list[str | int] = []
    for index, item in enumerate(value):
        if type(item) is str:
            result.append(
                _strict_text(item, label=f"{label}[{index}]", allow_empty=True)
            )
        elif (
            type(item) is int and not isinstance(item, bool) and 0 <= item <= 1_000_000
        ):
            result.append(item)
        else:
            _fail(f"{label}[{index}] must be a string or non-negative integer")
    return tuple(result)


def _space_projection(space: Any) -> Any:
    """Construct the wire projection lazily to keep parent imports light."""

    try:
        from .process_projection import DreamerProcessProjection

        return DreamerProcessProjection(space)
    except (TypeError, ValueError) as exc:
        _fail(
            f"live Dreamer space cannot be represented by the process projection: {exc}",
            exc,
        )


def _projection_from_metadata(value: Any, *, label: str) -> Any:
    try:
        from .process_projection import DreamerProcessProjection

        return DreamerProcessProjection.from_metadata(value)
    except (TypeError, ValueError) as exc:
        _fail(f"{label} projection metadata is invalid", exc)


def _space_leaves(projection: Any) -> tuple[dict[str, Any], ...]:
    try:
        space = projection.to_space()
        names = tuple(projection.wire_names)
        paths = tuple(projection.paths)
        schemas = projection.wire_schema
    except (AttributeError, TypeError, ValueError) as exc:
        _fail("projection does not expose its canonical wire leaves", exc)

    def walk(node: Any, current_path: tuple[str | int, ...]) -> tuple[Any, ...]:
        if isinstance(node, gymnasium.spaces.Dict):
            return tuple(
                child
                for key, value in node.spaces.items()
                for child in walk(value, (*current_path, key))
            )
        if isinstance(node, gymnasium.spaces.Tuple):
            return tuple(
                child
                for index, value in enumerate(node.spaces)
                for child in walk(value, (*current_path, index))
            )
        return (node,)

    leaves = walk(space, ())
    if len(leaves) != len(names) or len(names) != len(paths):
        _fail("projection wire leaves and paths are inconsistent")
    result: list[dict[str, Any]] = []
    for name, path_value, leaf in zip(names, paths, leaves, strict=True):
        if type(name) is not str or type(path_value) is not tuple:
            _fail("projection wire leaves are not canonical")
        try:
            schema = schemas[name]
            dtype = np.dtype(schema.dtype)
            shape = tuple(schema.shape)
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            _fail(f"projection leaf {name!r} is malformed", exc)
        result.append(
            {
                "name": name,
                "path": tuple(path_value),
                "space": leaf,
                "dtype": dtype,
                "shape": shape,
                "size": int(np.prod(shape, dtype=np.int64)) if shape else 1,
            }
        )
    return tuple(result)


def _path_key(path_value: Sequence[str | int]) -> str:
    return "/".join(str(item) for item in path_value)


def _get_path(value: Any, path_value: Sequence[str | int], *, label: str) -> Any:
    current = value
    for component in path_value:
        if isinstance(component, str):
            if not isinstance(current, Mapping) or component not in current:
                _fail(f"{label} is missing observation path {tuple(path_value)!r}")
            current = current[component]
        elif type(component) is int:
            if not isinstance(current, Sequence) or isinstance(
                current, (str, bytes, bytearray)
            ):
                _fail(f"{label} cannot descend into tuple path {tuple(path_value)!r}")
            if component >= len(current):
                _fail(f"{label} is missing observation path {tuple(path_value)!r}")
            current = current[component]
        else:  # pragma: no cover - paths are validated at construction
            _fail(f"{label} has an invalid path component")
    return current


def _set_path(
    value: Any, path_value: Sequence[str | int], item: Any, *, label: str
) -> None:
    if not path_value:
        _fail(f"{label} cannot assign an empty path")
    current = value
    for component in path_value[:-1]:
        if isinstance(component, str):
            if not isinstance(current, dict) or component not in current:
                _fail(f"{label} is missing path prefix {tuple(path_value)!r}")
            current = current[component]
        else:
            if not isinstance(current, list) or component >= len(current):
                _fail(f"{label} has an invalid tuple path")
            current = current[component]
    last = path_value[-1]
    if isinstance(last, str):
        if not isinstance(current, dict):
            _fail(f"{label} does not end in a mapping")
        current[last] = item
    elif isinstance(current, list) and last < len(current):
        current[last] = item
    else:
        _fail(f"{label} has an invalid final path")


def _copy_space(value: Any) -> Any:
    try:
        return copy.deepcopy(value)
    except (AttributeError, TypeError, ValueError, RecursionError) as exc:
        _fail("live Dreamer space could not be copied", exc)


def _read_contract(env: Any, *, obs_key: str) -> Mapping[str, Any]:
    # Native learner adapters expose an owned .env chain. Stop at the first
    # Gymnasium view so collection and training retain the same space transforms.
    source = env
    seen: set[int] = set()
    for _ in range(16):
        if id(source) in seen:
            _fail("live canonical contract wrapper chain contains a cycle")
        seen.add(id(source))
        try:
            getter = object.__getattribute__(source, "get_demo_io_contract")
        except AttributeError:
            getter = None
        if callable(getter):
            try:
                contract = getter()
            except (AttributeError, TypeError, ValueError, RuntimeError) as exc:
                _fail("live demo I/O contract could not be read", exc)
            if not isinstance(contract, Mapping):
                _fail("live demo I/O contract must be a mapping")
            return contract
        if isinstance(source, gymnasium.Env):
            break
        try:
            source = object.__getattribute__(source, "env")
        except AttributeError:
            _fail("live canonical contract requires a Gymnasium environment")
    else:
        _fail("live canonical contract wrapper chain is too deep")

    # Use the same outer Gymnasium view and selection as the canonical collector.
    # Construction reads introspection and spaces; it does not reset or step.
    try:
        from srb.wrappers.robomimic import RobomimicEnvWrapper

        wrapper = RobomimicEnvWrapper(source, obs_key=obs_key)
        contract = wrapper.get_demo_io_contract()
    except (AttributeError, TypeError, ValueError, RuntimeError, ImportError) as exc:
        _fail(
            "live environment does not expose a canonical Robomimic demo I/O contract",
            exc,
        )
    if not isinstance(contract, Mapping):
        _fail("live demo I/O contract must be a mapping")
    return contract


def _validate_contract(
    value: Any, *, label: str = "demo_io_contract"
) -> dict[str, Any]:
    _preflight_json(value, label=label)
    try:
        contract = validate_demo_io_contract(value, context=label, required=True)
    except (TypeError, ValueError) as exc:
        _fail(f"{label} is invalid", exc)
    if contract is None:  # pragma: no cover - required=True invariant
        _fail(f"{label} is required")
    copied = _json_copy(contract, label=label)
    if copied.get("version") != DEMO_IO_CONTRACT_VERSION:
        _fail(f"{label}.version must be {DEMO_IO_CONTRACT_VERSION!r}")
    return copied


def _contract_terms(contract: Mapping[str, Any]) -> tuple[dict[str, Any], ...]:
    observation = contract["observation"]
    terms = observation["terms"]
    if not isinstance(terms, list) or not 1 <= len(terms) <= MAX_TERMS:
        _fail("demo observation terms are outside the bounded limit")
    result: list[dict[str, Any]] = []
    for index, raw in enumerate(terms):
        if not isinstance(raw, Mapping):
            _fail(f"demo observation term {index} is not an object")
        shape_value = raw.get("shape")
        shape = _shape(shape_value, label=f"demo observation term {index}.shape")
        dtype = _dtype(raw.get("dtype"), label=f"demo observation term {index}.dtype")
        emitted = _path(
            raw.get("emitted_path"), label=f"demo observation term {index}.emitted_path"
        )
        if not emitted:
            _fail(f"demo observation term {index} has an empty emitted path")
        result.append(
            {
                "raw": dict(raw),
                "shape": shape,
                "dtype": dtype,
                "path": emitted,
                "source_key": _strict_text(
                    raw.get("source_key"),
                    label=f"demo observation term {index}.source_key",
                ),
                "output_key": _strict_text(
                    raw.get("output_key"),
                    label=f"demo observation term {index}.output_key",
                ),
            }
        )
    return tuple(result)


def _action_terms(contract: Mapping[str, Any]) -> tuple[dict[str, Any], ...]:
    action = contract["action"]
    raw_terms = action["schema"]
    if not isinstance(raw_terms, list) or not 1 <= len(raw_terms) <= MAX_TERMS:
        _fail("demo action terms are outside the bounded limit")
    result: list[dict[str, Any]] = []
    expected_start = 0
    for index, raw in enumerate(raw_terms):
        if not isinstance(raw, Mapping):
            _fail(f"demo action term {index} is not an object")
        start = _strict_int(
            raw.get("start"),
            label=f"demo action term {index}.start",
            maximum=MAX_ACTION_DIM,
        )
        end = _strict_int(
            raw.get("end"),
            label=f"demo action term {index}.end",
            maximum=MAX_ACTION_DIM,
        )
        dim = _strict_int(
            raw.get("dim"),
            label=f"demo action term {index}.dim",
            minimum=1,
            maximum=MAX_ACTION_DIM,
        )
        shape = _shape(raw.get("shape"), label=f"demo action term {index}.shape")
        shape_size = int(np.prod(shape, dtype=np.int64)) if shape else 1
        if start != expected_start or end - start != dim or shape_size != dim:
            _fail(f"demo action term {index} does not provide a contiguous flat slice")
        expected_start = end
        dtype_value = raw.get("dtype")
        dtype = (
            None
            if dtype_value is None
            else _dtype(dtype_value, label=f"demo action term {index}.dtype")
        )
        if dtype is None or dtype.kind != "f":
            _fail("Dreamer demonstrations require continuous floating action terms")
        path_value = _path(raw.get("path"), label=f"demo action term {index}.path")
        result.append(
            {
                "raw": dict(raw),
                "start": start,
                "end": end,
                "dim": dim,
                "shape": shape,
                "dtype": dtype,
                "path": path_value,
                "name": _strict_text(
                    raw.get("name"), label=f"demo action term {index}.name"
                ),
            }
        )
    action_dim = action.get("action_dim", expected_start)
    if type(action_dim) is not int or action_dim != expected_start:
        _fail("demo action dimension does not match its schema")
    if action.get("normalized") is not True:
        _fail("Dreamer demonstrations require normalized action coordinates")
    low = action.get("low")
    high = action.get("high")
    if (
        not isinstance(low, list)
        or not isinstance(high, list)
        or len(low) != expected_start
        or len(high) != expected_start
    ):
        _fail("normalized demonstrations must declare finite action bounds")
    try:
        low_array = np.asarray(low, dtype=np.float64)
        high_array = np.asarray(high, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        _fail("demo action bounds are not numeric", exc)
    if (
        low_array.shape != (expected_start,)
        or high_array.shape != (expected_start,)
        or not np.all(np.isfinite(low_array))
        or not np.all(np.isfinite(high_array))
        or not np.array_equal(low_array, -np.ones(expected_start))
        or not np.array_equal(high_array, np.ones(expected_start))
    ):
        _fail("Dreamer demonstrations require exact [-1, 1] source bounds")
    return tuple(result)


def _candidate_paths(
    path_value: Sequence[str | int], *, name: str, source_root: bool = False
) -> set[tuple[str | int, ...]]:
    result = {tuple(path_value)}
    if name:
        result.add((name,))
    if source_root:
        result.add(())
    return result


def _build_observation_map(
    contract: Mapping[str, Any],
    final_projection: Any,
    *,
    numeric_projection: Any | None = None,
) -> list[dict[str, Any]]:
    terms = _contract_terms(contract)
    leaves = _space_leaves(final_projection)
    if numeric_projection is not None:
        return _build_numeric_observation_map(contract, leaves, numeric_projection)
    result: list[dict[str, Any]] = []
    for term in terms:
        candidates = _candidate_paths(
            term["path"],
            name=term["output_key"],
        )
        candidates.add((term["source_key"],))
        matching = [
            leaf
            for leaf in leaves
            if leaf["path"] in candidates
            or _path_key(leaf["path"]) in {_path_key(item) for item in candidates}
        ]
        if (
            not matching
            and contract["observation"].get("root") is True
            and len(leaves) == 1
        ):
            matching = [leaves[0]]
        if len(matching) != 1:
            _fail(
                f"demo observation term {term['source_key']!r} does not match one live learner leaf"
            )
        leaf = matching[0]
        native_shape = leaf["shape"]
        native_size = leaf["size"]
        source_size = (
            int(np.prod(term["shape"], dtype=np.int64)) if term["shape"] else 1
        )
        native_slice: list[int] | None = None
        if native_shape != term["shape"]:
            vector_slice = term["raw"].get("vector_slice")
            if vector_slice is not None:
                if (
                    type(vector_slice) is not list
                    or len(vector_slice) != 2
                    or any(type(item) is not int for item in vector_slice)
                    or vector_slice[1] - vector_slice[0] != source_size
                    or vector_slice[0] < 0
                    or vector_slice[1] > native_size
                ):
                    _fail(
                        f"demo observation term {term['source_key']!r} has an invalid vector slice"
                    )
                native_slice = [int(vector_slice[0]), int(vector_slice[1])]
            elif source_size <= native_size:
                _fail(
                    f"demo observation term {term['source_key']!r} shape differs from its live learner leaf "
                    "without an explicit vector slice"
                )
            else:
                _fail(
                    f"demo observation term {term['source_key']!r} shape differs from its live learner leaf"
                )
        result.append(
            {
                "source_path": list(term["path"]),
                "source_key": term["source_key"],
                "output_key": term["output_key"],
                "source_shape": list(term["shape"]),
                "source_dtype": str(term["dtype"]),
                "source_slice": None,
                "native_path": list(leaf["path"]),
                "native_key": leaf["name"],
                "native_shape": list(native_shape),
                "native_dtype": str(leaf["dtype"]),
                "native_slice": native_slice,
            }
        )
    return result


def _build_numeric_observation_map(
    contract: Mapping[str, Any],
    final_leaves: Sequence[Mapping[str, Any]],
    numeric_projection: Any,
) -> list[dict[str, Any]]:
    """Map packed canonical vector slices to their learner contract leaves."""

    terms = _contract_terms(contract)
    try:
        numeric_leaves = tuple(numeric_projection.leaves)
    except (AttributeError, TypeError) as exc:
        _fail("live numeric observation projection does not expose its leaves", exc)
    if not 1 <= len(numeric_leaves) <= MAX_MAP_ENTRIES:
        _fail("numeric observation projection has an unsupported leaf count")

    result: list[dict[str, Any]] = []
    matched_terms: set[int] = set()
    for index, numeric_leaf in enumerate(numeric_leaves):
        try:
            numeric_path = str(numeric_leaf.path)
            numeric_source = str(numeric_leaf.source_key)
            source_slice = numeric_leaf.source_slice
            output_slice = tuple(int(item) for item in numeric_leaf.output_slice)
            numeric_shape = tuple(int(item) for item in numeric_leaf.shape)
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            _fail(f"numeric observation leaf {index} is malformed", exc)
        if len(output_slice) != 2 or output_slice[0] < 0 or output_slice[1] < 0:
            _fail(f"numeric observation leaf {index} has an invalid output slice")
        if len(numeric_shape) > 8:
            _fail(f"numeric observation leaf {index} has an invalid shape")

        candidates = [
            (term_index, term)
            for term_index, term in enumerate(terms)
            if numeric_path
            in {
                _path_key(term["path"]),
                term["source_key"],
                term["output_key"],
            }
            or numeric_source
            in {
                term["source_key"],
                term["output_key"],
            }
        ]
        if len(candidates) != 1:
            _fail(f"numeric observation leaf {index} does not match one canonical term")
        term_index, term = candidates[0]
        matched_terms.add(term_index)
        source_size = (
            int(np.prod(term["shape"], dtype=np.int64)) if term["shape"] else 1
        )
        source_slice_value: list[int] | None = None
        if source_slice is not None:
            try:
                source_slice_value = [int(source_slice[0]), int(source_slice[1])]
            except (TypeError, ValueError, IndexError) as exc:
                _fail(
                    f"numeric observation leaf {index} source slice is malformed",
                    exc,
                )
            if (
                len(source_slice) != 2
                or source_slice_value[0] < 0
                or source_slice_value[1] > source_size
                or source_slice_value[1] - source_slice_value[0]
                != int(np.prod(numeric_shape, dtype=np.int64))
            ):
                _fail(
                    f"numeric observation term {term['source_key']!r} source slice has the wrong width"
                )
        elif numeric_shape != term["shape"]:
            _fail(
                f"numeric observation term {term['source_key']!r} shape/slice drifted"
            )

        candidates = [
            leaf
            for leaf in final_leaves
            if leaf["name"] == numeric_source
            or leaf["name"] == numeric_path
            or leaf["path"] in {(numeric_source,), (numeric_path,)}
        ]
        if not candidates and len(final_leaves) == 1:
            candidates = [final_leaves[0]]
        if len(candidates) != 1:
            _fail(
                f"numeric observation term {term['source_key']!r} has no final learner leaf"
            )
        final_leaf = candidates[0]
        native_size = final_leaf["size"]
        if (
            output_slice[1] <= output_slice[0]
            or output_slice[1] > native_size
            or output_slice[1] - output_slice[0]
            != int(np.prod(numeric_shape, dtype=np.int64))
        ):
            _fail(
                f"numeric observation term {term['source_key']!r} exceeds its final leaf"
            )
        result.append(
            {
                "source_path": list(term["path"]),
                "source_key": term["source_key"],
                "output_key": term["output_key"],
                "source_shape": list(term["shape"]),
                "source_dtype": str(term["dtype"]),
                "source_slice": source_slice_value,
                "native_path": list(final_leaf["path"]),
                "native_key": final_leaf["name"],
                "native_shape": list(final_leaf["shape"]),
                "native_dtype": str(final_leaf["dtype"]),
                "native_slice": [output_slice[0], output_slice[1]],
            }
        )
    if matched_terms != set(range(len(terms))):
        _fail("numeric observation projection omits a canonical observation term")
    return result


def _build_action_map(
    contract: Mapping[str, Any], final_projection: Any
) -> list[dict[str, Any]]:
    terms = _action_terms(contract)
    leaves = _space_leaves(final_projection)
    result: list[dict[str, Any]] = []
    used: set[str] = set()
    for term in terms:
        candidates = _candidate_paths(
            term["path"],
            name=term["name"],
            source_root=not term["path"],
        )
        matching = [
            leaf
            for leaf in leaves
            if leaf["path"] in candidates
            or _path_key(leaf["path"]) in {_path_key(item) for item in candidates}
        ]
        if len(matching) != 1:
            _fail(
                f"demo action term {term['name']!r} does not match one live learner leaf"
            )
        leaf = matching[0]
        if leaf["name"] in used:
            _fail(f"demo action terms collide at live learner leaf {leaf['name']!r}")
        used.add(leaf["name"])
        if leaf["dtype"].kind != "f" or isinstance(
            leaf["space"], gymnasium.spaces.Discrete
        ):
            _fail("Dreamer demonstrations do not support discrete live actions")
        if leaf["shape"] != term["shape"]:
            _fail(
                f"demo action term {term['name']!r} shape differs from its live learner leaf"
            )
        try:
            native_low = np.asarray(leaf["space"].low).reshape(-1)
            native_high = np.asarray(leaf["space"].high).reshape(-1)
            source_low = np.asarray(contract["action"]["low"])[
                term["start"] : term["end"]
            ]
            source_high = np.asarray(contract["action"]["high"])[
                term["start"] : term["end"]
            ]
        except (AttributeError, TypeError, ValueError) as exc:
            _fail(
                f"demo action term {term['name']!r} has unavailable live bounds",
                exc,
            )
        if not np.array_equal(native_low, source_low) or not np.array_equal(
            native_high, source_high
        ):
            _fail(
                f"demo action term {term['name']!r} bounds differ from its live learner leaf"
            )
        result.append(
            {
                "source_path": list(term["path"]),
                "source_name": term["name"],
                "source_start": term["start"],
                "source_end": term["end"],
                "source_shape": list(term["shape"]),
                "source_dtype": str(term["dtype"]),
                "native_path": list(leaf["path"]),
                "native_key": leaf["name"],
                "native_shape": list(leaf["shape"]),
                "native_dtype": str(leaf["dtype"]),
                "scale": contract["action"].get("scales", {}).get(term["name"]),
            }
        )
    if len(used) != len(leaves):
        _fail("live learner action has leaves absent from the canonical action schema")
    return result


def _descriptor_without_fingerprint(value: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: copy.deepcopy(item) for key, item in value.items() if key != "fingerprint"
    }


def _validate_descriptor_shape(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        _fail("demo_projection must be a JSON object")
    descriptor = _json_copy(value, label="demo_projection")
    expected = {
        "version",
        "contract",
        "source",
        "final",
        "observation_map",
        "action_map",
        "fingerprint",
    }
    if set(descriptor) != expected:
        _fail(f"demo_projection fields must be exactly {sorted(expected)!r}")
    if descriptor["version"] != DEMO_PROJECTION_VERSION:
        _fail(f"demo_projection.version must be {DEMO_PROJECTION_VERSION!r}")
    contract = _validate_contract(
        descriptor["contract"], label="demo_projection.contract"
    )
    descriptor["contract"] = contract
    for section in ("source", "final"):
        raw_section = descriptor[section]
        if not isinstance(raw_section, Mapping) or set(raw_section) != {
            "action",
            "observation",
        }:
            _fail(
                f"demo_projection.{section} must contain action and observation projections"
            )
        for name in ("action", "observation"):
            projection = _projection_from_metadata(
                raw_section[name], label=f"demo_projection.{section}.{name}"
            )
            raw_section[name] = projection.metadata
    for field_name in ("observation_map", "action_map"):
        entries = descriptor[field_name]
        if not isinstance(entries, list) or not 1 <= len(entries) <= MAX_MAP_ENTRIES:
            _fail(f"demo_projection.{field_name} is outside the bounded limit")
        for index, entry in enumerate(entries):
            if not isinstance(entry, Mapping):
                _fail(f"demo_projection.{field_name}[{index}] is not an object")
    fingerprint = descriptor["fingerprint"]
    if (
        type(fingerprint) is not str
        or len(fingerprint) != 64
        or any(char not in "0123456789abcdef" for char in fingerprint)
    ):
        _fail("demo_projection.fingerprint must be a lowercase SHA-256 digest")
    expected_fingerprint = hashlib.sha256(
        _canonical_bytes(
            _descriptor_without_fingerprint(descriptor), label="demo_projection"
        )
    ).hexdigest()
    if fingerprint != expected_fingerprint:
        _fail("demo_projection.fingerprint does not match its descriptor")
    return descriptor


def validate_demo_projection_descriptor(
    value: Any,
    *,
    action_projection: Any | None = None,
    observation_projection: Any | None = None,
) -> dict[str, Any]:
    """Validate and detach a bounded descriptor against negotiated spaces."""

    descriptor = _validate_descriptor_shape(value)
    final_action = _projection_from_metadata(
        descriptor["final"]["action"], label="demo final action"
    )
    final_observation = _projection_from_metadata(
        descriptor["final"]["observation"], label="demo final observation"
    )
    _validate_maps(
        descriptor["contract"],
        final_observation,
        final_action,
        descriptor["observation_map"],
        descriptor["action_map"],
    )
    if action_projection is not None:
        try:
            actual = action_projection.metadata
        except AttributeError as exc:
            _fail("negotiated action projection metadata is unavailable", exc)
        if descriptor["final"]["action"] != actual:
            _fail(
                "demo projection final action disagrees with the negotiated action projection"
            )
    if observation_projection is not None:
        try:
            actual = observation_projection.metadata
        except AttributeError as exc:
            _fail("negotiated observation projection metadata is unavailable", exc)
        if descriptor["final"]["observation"] != actual:
            _fail(
                "demo projection final observation disagrees with the negotiated observation projection"
            )
    return descriptor


class DreamerDemoProjection:
    """Immutable one-row bridge from canonical demos to native Dreamer IO."""

    __slots__ = (
        "_action_map",
        "_contract",
        "_descriptor",
        "_final_action_projection",
        "_final_observation_projection",
        "_learner_projection",
        "_observation_map",
        "_source_action_projection",
        "_source_observation_projection",
    )

    def __init__(
        self,
        *,
        contract: Mapping[str, Any],
        descriptor: Mapping[str, Any],
        source_action_projection: Any,
        source_observation_projection: Any,
        final_action_projection: Any,
        final_observation_projection: Any,
        observation_map: Sequence[Mapping[str, Any]],
        action_map: Sequence[Mapping[str, Any]],
        learner_projection: Any | None,
    ) -> None:
        object.__setattr__(
            self, "_contract", _json_copy(contract, label="demo contract")
        )
        object.__setattr__(self, "_descriptor", _validate_descriptor_shape(descriptor))
        object.__setattr__(self, "_source_action_projection", source_action_projection)
        object.__setattr__(
            self, "_source_observation_projection", source_observation_projection
        )
        object.__setattr__(self, "_final_action_projection", final_action_projection)
        object.__setattr__(
            self, "_final_observation_projection", final_observation_projection
        )
        object.__setattr__(
            self,
            "_observation_map",
            tuple(
                _json_copy(dict(item), label="observation map")
                for item in observation_map
            ),
        )
        object.__setattr__(
            self,
            "_action_map",
            tuple(_json_copy(dict(item), label="action map") for item in action_map),
        )
        object.__setattr__(self, "_learner_projection", learner_projection)

    @classmethod
    def from_live(
        cls,
        env: Any,
        *,
        learner_projection: Any,
        contract: Mapping[str, Any] | None = None,
        observation_key: str | None = None,
    ) -> "DreamerDemoProjection":
        """Build from a live one-environment SRB view without stepping it."""

        if learner_projection is None:
            _fail("live Dreamer demo projection requires learner_projection")
        try:
            num_envs = learner_projection.num_envs
        except AttributeError as exc:
            _fail("live learner projection does not expose num_envs", exc)
        if type(num_envs) is not int or num_envs != 1:
            _fail("Dreamer demonstrations require exactly one live environment")
        obs_key = observation_key or "policy"
        live_contract = _validate_contract(_read_contract(env, obs_key=obs_key))
        selected_contract = (
            live_contract
            if contract is None
            else _validate_contract(contract, label="source demo_io_contract")
        )
        if _canonical_bytes(
            live_contract, label="live demo_io_contract"
        ) != _canonical_bytes(selected_contract, label="source demo_io_contract"):
            _fail("source demo_io_contract differs from the live canonical contract")

        try:
            source_observation_space = learner_projection.raw_observation_space
            source_action_space = learner_projection.raw_action_space
            final_observation_space = learner_projection.observation_space
            final_action_space = learner_projection.action_space
        except AttributeError as exc:
            _fail("live learner projection does not expose raw and final spaces", exc)
        source_action_projection = _space_projection(_copy_space(source_action_space))
        source_observation_projection = _space_projection(
            _copy_space(source_observation_space)
        )
        final_action_projection = _space_projection(_copy_space(final_action_space))
        final_observation_projection = _space_projection(
            _copy_space(final_observation_space)
        )
        observation_map = _build_observation_map(
            selected_contract,
            final_observation_projection,
            numeric_projection=getattr(
                learner_projection, "observation_projection", None
            ),
        )
        action_map = _build_action_map(selected_contract, final_action_projection)
        descriptor = _make_descriptor(
            selected_contract,
            source_action_projection,
            source_observation_projection,
            final_action_projection,
            final_observation_projection,
            observation_map,
            action_map,
        )
        return cls(
            contract=selected_contract,
            descriptor=descriptor,
            source_action_projection=source_action_projection,
            source_observation_projection=source_observation_projection,
            final_action_projection=final_action_projection,
            final_observation_projection=final_observation_projection,
            observation_map=observation_map,
            action_map=action_map,
            learner_projection=learner_projection,
        )

    @classmethod
    def from_metadata(cls, value: Any) -> "DreamerDemoProjection":
        """Reconstruct the process-parent adapter from bounded wire metadata."""

        descriptor = _validate_descriptor_shape(value)
        source_action_projection = _projection_from_metadata(
            descriptor["source"]["action"], label="demo source action"
        )
        source_observation_projection = _projection_from_metadata(
            descriptor["source"]["observation"], label="demo source observation"
        )
        final_action_projection = _projection_from_metadata(
            descriptor["final"]["action"], label="demo final action"
        )
        final_observation_projection = _projection_from_metadata(
            descriptor["final"]["observation"], label="demo final observation"
        )
        # Validate the map against the negotiated descriptor before exposing a
        # callable adapter.  This catches a tampered metadata row at startup.
        _validate_maps(
            descriptor["contract"],
            final_observation_projection,
            final_action_projection,
            descriptor["observation_map"],
            descriptor["action_map"],
        )
        return cls(
            contract=descriptor["contract"],
            descriptor=descriptor,
            source_action_projection=source_action_projection,
            source_observation_projection=source_observation_projection,
            final_action_projection=final_action_projection,
            final_observation_projection=final_observation_projection,
            observation_map=descriptor["observation_map"],
            action_map=descriptor["action_map"],
            learner_projection=None,
        )

    @property
    def metadata(self) -> dict[str, Any]:
        return copy.deepcopy(self._descriptor)

    @property
    def descriptor(self) -> dict[str, Any]:
        return self.metadata

    @property
    def contract(self) -> dict[str, Any]:
        return copy.deepcopy(self._contract)

    @property
    def demo_io_contract(self) -> dict[str, Any]:
        return self.contract

    @property
    def fingerprint(self) -> str:
        return str(self._descriptor["fingerprint"])

    @property
    def source_action_space(self) -> Any:
        return self._source_action_projection.to_space()

    @property
    def source_action_projection(self) -> Any:
        return self._source_action_projection

    @property
    def source_observation_space(self) -> Any:
        return self._source_observation_projection.to_space()

    @property
    def source_observation_projection(self) -> Any:
        return self._source_observation_projection

    @property
    def action_space(self) -> Any:
        return self._final_action_projection.to_space()

    @property
    def final_action_projection(self) -> Any:
        return self._final_action_projection

    @property
    def observation_space(self) -> Any:
        return self._final_observation_projection.to_space()

    @property
    def final_observation_projection(self) -> Any:
        return self._final_observation_projection

    @property
    def learner_projection(self) -> Any | None:
        return self._learner_projection

    @property
    def observation_map(self) -> tuple[dict[str, Any], ...]:
        return tuple(copy.deepcopy(item) for item in self._observation_map)

    @property
    def action_map(self) -> tuple[dict[str, Any], ...]:
        return tuple(copy.deepcopy(item) for item in self._action_map)

    def project_observation(self, canonical_row: Any) -> OrderedDict[str, np.ndarray]:
        """Project one canonical transition observation to an unbatched row."""

        normalized = self._canonical_observation(canonical_row)
        if self._learner_projection is not None:
            observation = normalized
            if self._contract["observation"]["root"]:
                root_key = self._contract["observation"]["root_observation_key"]
                observation = self._prepare_learner_observation(normalized[root_key])
            elif self._contract["observation"].get("selection_present"):
                observation = self._prepare_learner_observation(normalized)
                observation = {self._contract["observation"]["obs_key"]: observation}
            else:
                observation = self._prepare_learner_observation(observation)
            try:
                projected = self._learner_projection.project_observation_row(
                    observation
                )
            except (TypeError, ValueError) as exc:
                _fail(
                    "canonical observation disagrees with the live learner projection",
                    exc,
                )
            return OrderedDict(
                (str(key), np.array(value, copy=True, order="C"))
                for key, value in projected.items()
            )

        wire = self._project_observation_without_learner(normalized)
        try:
            projected = self._final_observation_projection.unpack(wire)
        except (TypeError, ValueError) as exc:
            _fail(
                "canonical observation does not match the negotiated learner projection",
                exc,
            )
        if not isinstance(projected, Mapping):
            _fail("final Dreamer observation must be a mapping")
        return OrderedDict(
            (str(key), np.array(value, copy=True, order="C"))
            for key, value in projected.items()
        )

    def _prepare_learner_observation(self, observation: Any) -> Any:
        """Adapt canonical per-term rows to the live raw learner leaf shapes."""

        source_leaves = _space_leaves(self._source_observation_projection)
        by_name = {leaf["name"]: leaf for leaf in source_leaves}
        by_path = {_path_key(leaf["path"]): leaf for leaf in source_leaves}
        result = copy.deepcopy(observation)
        used_source_names: set[str] = set()
        for entry in self._observation_map:
            entry_path = tuple(entry["source_path"])
            if not isinstance(result, Mapping) and entry_path == (
                self._contract["observation"]["root_observation_key"],
            ):
                entry_path = ()
            candidates = (
                entry_path,
                (str(entry["source_key"]),),
                (str(entry["output_key"]),),
            )
            source_leaf = next(
                (
                    by_path.get(_path_key(candidate))
                    for candidate in candidates
                    if _path_key(candidate) in by_path
                ),
                None,
            )
            if source_leaf is None:
                source_leaf = by_name.get(str(entry["native_key"]))
            if source_leaf is None:
                _fail(
                    f"raw learner observation leaf for {entry['source_key']!r} is missing"
                )
            used_source_names.add(source_leaf["name"])
            value = np.asarray(
                _get_path(result, entry_path, label="canonical observation")
                if entry_path
                else result
            )
            target_shape = tuple(source_leaf["shape"])
            if value.shape != target_shape:
                if target_shape == (1, *value.shape):
                    value = np.expand_dims(value, axis=0)
                else:
                    _fail(
                        f"canonical observation {entry['source_key']!r} cannot match "
                        f"raw learner shape {target_shape}"
                    )
            value = np.array(value, dtype=source_leaf["dtype"], copy=True, order="C")
            if entry_path:
                _set_path(result, entry_path, value, label="learner observation")
            else:
                result = value
        if used_source_names != set(by_name):
            _fail(
                "live raw observation leaves do not match the canonical observation map"
            )
        return result

    def project_action(self, flat_float32: Any) -> OrderedDict[str, np.ndarray]:
        """Project one normalized flat float32 action to a native row mapping."""

        if not isinstance(flat_float32, np.ndarray):
            _fail("canonical action must be a NumPy ndarray")
        if flat_float32.dtype != np.dtype(np.float32):
            _fail(f"canonical action dtype must be float32, got {flat_float32.dtype}")
        action_dim = int(self._contract["action"]["action_dim"])
        if flat_float32.shape != (action_dim,):
            _fail(
                f"canonical action shape must be {(action_dim,)}, got {flat_float32.shape}"
            )
        if not np.all(np.isfinite(flat_float32)):
            _fail("canonical action contains non-finite values")
        if np.any(flat_float32 < -1.0) or np.any(flat_float32 > 1.0):
            _fail("canonical action is outside its normalized [-1, 1] source domain")
        wire: dict[str, np.ndarray] = {}
        leaves = _space_leaves(self._final_action_projection)
        leaves_by_name = {leaf["name"]: leaf for leaf in leaves}
        for entry in self._action_map:
            name = entry["native_key"]
            leaf = leaves_by_name.get(name)
            if leaf is None:
                _fail(f"final action leaf {name!r} is missing")
            start = int(entry["source_start"])
            end = int(entry["source_end"])
            source_shape = tuple(int(item) for item in entry["source_shape"])
            value = flat_float32[start:end].reshape(source_shape)
            native_dtype = np.dtype(entry["native_dtype"])
            try:
                converted = np.asarray(value, dtype=native_dtype)
            except (TypeError, ValueError, OverflowError) as exc:
                _fail(
                    f"canonical action term {entry['source_name']!r} cannot use native dtype",
                    exc,
                )
            if converted.shape != tuple(entry["native_shape"]):
                _fail(
                    f"canonical action term {entry['source_name']!r} has a native shape mismatch"
                )
            wire[name] = np.array(converted, copy=True, order="C")
        try:
            projected = self._final_action_projection.unpack(wire)
        except (TypeError, ValueError) as exc:
            _fail(
                "canonical action does not match the negotiated learner projection", exc
            )
        if not isinstance(projected, Mapping):
            _fail("final Dreamer action must be a mapping")
        return OrderedDict(
            (str(key), np.array(value, copy=True, order="C"))
            for key, value in projected.items()
        )

    def validate_final_spaces(
        self,
        final_env: Any | None = None,
        *,
        observation_space: Any | None = None,
        action_space: Any | None = None,
        obs_space: Any | None = None,
        act_space: Any | None = None,
    ) -> None:
        """Reject final NormalizeAction/UnifyDtypes space drift."""

        if final_env is not None:
            observation_space = getattr(final_env, "obs_space", observation_space)
            action_space = getattr(final_env, "act_space", action_space)
        observation_space = (
            observation_space if observation_space is not None else obs_space
        )
        action_space = action_space if action_space is not None else act_space
        if observation_space is None or action_space is None:
            _fail("final Dreamer spaces are required for demo projection binding")
        _validate_final_space(
            observation_space, self._final_observation_projection, name="observation"
        )
        _validate_final_space(
            action_space,
            self._final_action_projection,
            name="action",
            ignore_keys={"reset"},
        )

    def _canonical_observation(self, canonical_row: Any) -> dict[str, Any]:
        if not isinstance(canonical_row, Mapping):
            _fail("canonical observation must be a mapping")
        contract_obs = self._contract["observation"]
        expected_keys = tuple(contract_obs["keys"])
        if set(canonical_row) != set(expected_keys):
            _fail(f"canonical observation keys must be {expected_keys!r}")
        normalized = copy.deepcopy(dict(canonical_row))
        for entry in self._observation_map:
            path_value = tuple(entry["source_path"])
            value = _get_path(canonical_row, path_value, label="canonical observation")
            array = np.asarray(value)
            expected_shape = tuple(int(item) for item in entry["source_shape"])
            if array.shape == (1, *expected_shape):
                array = np.array(array[0], copy=True)
            elif array.shape != expected_shape:
                _fail(
                    f"canonical observation {entry['source_key']!r} shape must be "
                    f"{expected_shape} or {(1, *expected_shape)}, got {array.shape}"
                )
            expected_dtype = np.dtype(entry["source_dtype"])
            if array.dtype != expected_dtype:
                if expected_dtype.kind in "iu" and (
                    array.dtype.kind not in "iu"
                    or not np.all(np.equal(array, np.floor(array)))
                ):
                    _fail(
                        f"canonical observation {entry['source_key']!r} dtype drift cannot be cast safely"
                    )
                if expected_dtype.kind in "iu":
                    limits = np.iinfo(expected_dtype)
                    if np.any(array < limits.min) or np.any(array > limits.max):
                        _fail(
                            f"canonical observation {entry['source_key']!r} values exceed {expected_dtype} bounds"
                        )
                elif expected_dtype.kind == "b" and array.dtype.kind != "b":
                    _fail(
                        f"canonical observation {entry['source_key']!r} requires boolean values"
                    )
                try:
                    array = np.asarray(array, dtype=expected_dtype)
                except (TypeError, ValueError, OverflowError) as exc:
                    _fail(
                        f"canonical observation {entry['source_key']!r} dtype is invalid",
                        exc,
                    )
            if array.dtype.kind in "f" and not np.all(np.isfinite(array)):
                _fail(
                    f"canonical observation {entry['source_key']!r} contains non-finite values"
                )
            _set_path(
                normalized,
                path_value,
                np.array(array, copy=True, order="C"),
                label="canonical observation",
            )
        return normalized

    def _project_observation_without_learner(
        self, normalized: Mapping[str, Any]
    ) -> dict[str, np.ndarray]:
        leaves = _space_leaves(self._final_observation_projection)
        groups: dict[str, list[dict[str, Any]]] = {}
        for entry in self._observation_map:
            groups.setdefault(entry["native_key"], []).append(entry)
        wire: dict[str, np.ndarray] = {}
        for leaf in leaves:
            entries = groups.get(leaf["name"], [])
            if not entries:
                _fail(
                    f"canonical observation has no value for live leaf {leaf['name']!r}"
                )
            target = np.zeros(leaf["shape"], dtype=leaf["dtype"])
            for entry in entries:
                value = np.asarray(
                    _get_path(
                        normalized, entry["source_path"], label="canonical observation"
                    ),
                    dtype=leaf["dtype"],
                )
                source_slice = entry.get("source_slice")
                if source_slice is not None:
                    value = value.reshape(-1)[
                        int(source_slice[0]) : int(source_slice[1])
                    ]
                native_slice = entry.get("native_slice")
                if native_slice is None:
                    if value.shape != leaf["shape"]:
                        _fail(
                            f"canonical observation leaf {entry['source_key']!r} has a native shape mismatch"
                        )
                    target[...] = value
                else:
                    start, end = (int(native_slice[0]), int(native_slice[1]))
                    target.reshape(-1)[start:end] = value.reshape(-1)
            wire[leaf["name"]] = target
        return wire

    def __setattr__(self, name: str, value: Any) -> None:
        if hasattr(self, "_contract"):
            raise AttributeError("DreamerDemoProjection is immutable")
        object.__setattr__(self, name, value)


def _make_descriptor(
    contract: Mapping[str, Any],
    source_action_projection: Any,
    source_observation_projection: Any,
    final_action_projection: Any,
    final_observation_projection: Any,
    observation_map: Sequence[Mapping[str, Any]],
    action_map: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    _validate_maps(
        contract,
        final_observation_projection,
        final_action_projection,
        observation_map,
        action_map,
    )
    descriptor: dict[str, Any] = {
        "version": DEMO_PROJECTION_VERSION,
        "contract": _json_copy(contract, label="demo contract"),
        "source": {
            "action": source_action_projection.metadata,
            "observation": source_observation_projection.metadata,
        },
        "final": {
            "action": final_action_projection.metadata,
            "observation": final_observation_projection.metadata,
        },
        "observation_map": _json_copy(list(observation_map), label="observation map"),
        "action_map": _json_copy(list(action_map), label="action map"),
    }
    descriptor["fingerprint"] = hashlib.sha256(
        _canonical_bytes(descriptor, label="demo_projection")
    ).hexdigest()
    _validate_descriptor_shape(descriptor)
    return descriptor


def _validate_maps(
    contract: Mapping[str, Any],
    final_observation_projection: Any,
    final_action_projection: Any,
    observation_map: Sequence[Mapping[str, Any]],
    action_map: Sequence[Mapping[str, Any]],
) -> None:
    terms = _contract_terms(contract)
    observation_leaves = _space_leaves(final_observation_projection)
    action_leaves = _space_leaves(final_action_projection)
    if not 1 <= len(observation_map) <= MAX_MAP_ENTRIES:
        _fail("demo projection observation map does not cover its contract")
    if len(action_map) != len(_action_terms(contract)):
        _fail("demo projection action map does not cover its contract")

    observation_by_name = {leaf["name"]: leaf for leaf in observation_leaves}
    observation_seen: set[str] = set()
    observation_ranges: dict[str, list[tuple[int, int]]] = {}
    matched_terms: set[int] = set()
    for index, entry in enumerate(observation_map):
        term_candidates = [
            (term_index, term)
            for term_index, term in enumerate(terms)
            if entry.get("source_path") == list(term["path"])
            and entry.get("source_key") == term["source_key"]
            and entry.get("output_key") == term["output_key"]
        ]
        if len(term_candidates) != 1:
            _fail(
                f"demo projection observation_map[{index}] does not match one canonical term"
            )
        term_index, term = term_candidates[0]
        matched_terms.add(term_index)
        _validate_observation_map_entry(
            term,
            entry,
            observation_by_name,
            label=f"demo projection observation_map[{index}]",
        )
        native_key = str(entry["native_key"])
        observation_seen.add(native_key)
        native_slice = entry["native_slice"]
        native_size = observation_by_name[native_key]["size"]
        interval = (
            (0, native_size)
            if native_slice is None
            else (int(native_slice[0]), int(native_slice[1]))
        )
        observation_ranges.setdefault(native_key, []).append(interval)
    if observation_seen != {leaf["name"] for leaf in observation_leaves}:
        _fail("demo projection observation map omits a final learner leaf")
    for native_key, ranges in observation_ranges.items():
        cursor = 0
        for start, end in sorted(ranges):
            if start < cursor:
                _fail(
                    f"demo projection observation map overlaps final leaf {native_key!r}"
                )
            if start > cursor:
                _fail(
                    f"demo projection observation map leaves a hole in {native_key!r}"
                )
            cursor = end
        if cursor != observation_by_name[native_key]["size"]:
            _fail(f"demo projection observation map leaves a hole in {native_key!r}")
    if matched_terms != set(range(len(terms))):
        _fail("demo projection observation map omits a canonical observation term")

    action_terms = _action_terms(contract)
    action_by_name = {leaf["name"]: leaf for leaf in action_leaves}
    action_seen: set[str] = set()
    for index, (term, entry) in enumerate(zip(action_terms, action_map, strict=True)):
        _validate_action_map_entry(
            term,
            entry,
            action_by_name,
            expected_scale=contract["action"].get("scales", {}).get(term["name"]),
            label=f"demo projection action_map[{index}]",
        )
        native_key = str(entry["native_key"])
        if native_key in action_seen:
            _fail("demo projection action map routes multiple terms to one action leaf")
        action_seen.add(native_key)
    if action_seen != {leaf["name"] for leaf in action_leaves}:
        _fail("demo projection action map omits a final learner leaf")


def _validate_observation_map_entry(
    term: Mapping[str, Any],
    entry: Mapping[str, Any],
    leaves: Mapping[str, Mapping[str, Any]],
    *,
    label: str,
) -> None:
    required = {
        "source_path",
        "source_key",
        "output_key",
        "source_shape",
        "source_dtype",
        "source_slice",
        "native_path",
        "native_key",
        "native_shape",
        "native_dtype",
        "native_slice",
    }
    if not required <= set(entry):
        _fail(f"{label} is missing required fields")
    if list(entry["source_path"]) != list(term["path"]):
        _fail(f"{label} source path disagrees with its contract term")
    if (
        entry["source_key"] != term["source_key"]
        or entry["output_key"] != term["output_key"]
    ):
        _fail(f"{label} source names disagree with its contract term")
    if entry["source_shape"] != list(term["shape"]) or entry["source_dtype"] != str(
        term["dtype"]
    ):
        _fail(f"{label} source schema disagrees with its contract term")
    native_key = entry["native_key"]
    if type(native_key) is not str or native_key not in leaves:
        _fail(f"{label} names an unknown final observation leaf")
    leaf = leaves[native_key]
    if list(entry["native_path"]) != list(leaf["path"]):
        _fail(f"{label} native path disagrees with the final observation projection")
    if entry["native_shape"] != list(leaf["shape"]) or entry["native_dtype"] != str(
        leaf["dtype"]
    ):
        _fail(f"{label} native schema disagrees with the final observation projection")
    native_slice = entry["native_slice"]
    source_size = int(np.prod(term["shape"], dtype=np.int64)) if term["shape"] else 1
    source_slice = entry["source_slice"]
    source_width = source_size
    if source_slice is not None:
        if (
            type(source_slice) is not list
            or len(source_slice) != 2
            or any(type(item) is not int or item < 0 for item in source_slice)
            or source_slice[1] - source_slice[0] <= 0
            or source_slice[1] > source_size
        ):
            _fail(f"{label} source slice is invalid")
        source_width = source_slice[1] - source_slice[0]
    if native_slice is not None:
        if (
            type(native_slice) is not list
            or len(native_slice) != 2
            or any(type(item) is not int or item < 0 for item in native_slice)
            or native_slice[1] - native_slice[0] != source_width
            or native_slice[1] > leaf["size"]
        ):
            _fail(f"{label} native slice is invalid")
    elif source_slice is not None or tuple(term["shape"]) != tuple(leaf["shape"]):
        _fail(f"{label} omits the slice needed for its shape conversion")


def _validate_action_map_entry(
    term: Mapping[str, Any],
    entry: Mapping[str, Any],
    leaves: Mapping[str, Mapping[str, Any]],
    *,
    expected_scale: Any,
    label: str,
) -> None:
    required = {
        "source_path",
        "source_name",
        "source_start",
        "source_end",
        "source_shape",
        "source_dtype",
        "native_path",
        "native_key",
        "native_shape",
        "native_dtype",
        "scale",
    }
    if not required <= set(entry):
        _fail(f"{label} is missing required fields")
    if (
        list(entry["source_path"]) != list(term["path"])
        or entry["source_name"] != term["name"]
        or entry["source_start"] != term["start"]
        or entry["source_end"] != term["end"]
        or entry["source_shape"] != list(term["shape"])
        or entry["source_dtype"] != str(term["dtype"])
    ):
        _fail(f"{label} source schema disagrees with its contract term")
    if entry["scale"] != expected_scale:
        _fail(f"{label} action scale disagrees with its contract term")
    native_key = entry["native_key"]
    if type(native_key) is not str or native_key not in leaves:
        _fail(f"{label} names an unknown final action leaf")
    leaf = leaves[native_key]
    if list(entry["native_path"]) != list(leaf["path"]):
        _fail(f"{label} native path disagrees with the final action projection")
    if entry["native_shape"] != list(leaf["shape"]) or entry["native_dtype"] != str(
        leaf["dtype"]
    ):
        _fail(f"{label} native schema disagrees with the final action projection")
    if leaf["dtype"].kind != "f":
        _fail(f"{label} routes a discrete action leaf")


def _validate_final_space(
    value: Any, projection: Any, *, name: str, ignore_keys: set[str] | None = None
) -> None:
    ignore = set(ignore_keys or ())
    if isinstance(value, gymnasium.Space):
        actual = _space_projection(value)
        if actual.metadata != projection.metadata:
            _fail(f"final {name} space differs from the negotiated projection")
        return
    if not isinstance(value, Mapping):
        _fail(f"final {name} space must be a Gymnasium space or mapping")
    leaves = _space_leaves(projection)
    expected: dict[str, dict[str, Any]] = {}
    for leaf in leaves:
        candidates = {leaf["name"], _path_key(leaf["path"])}
        if leaf["path"]:
            candidates.add(str(leaf["path"][-1]))
        for candidate in candidates:
            expected[candidate] = leaf
    seen: set[str] = set()
    for key, space in value.items():
        if key in ignore:
            continue
        if key not in expected:
            if name == "observation" and key in {
                "reward",
                "is_first",
                "is_last",
                "is_terminal",
            }:
                continue
            _fail(f"final {name} space has unexpected key {key!r}")
        leaf = expected[key]
        seen.add(leaf["name"])
        try:
            actual_dtype = np.dtype(space.dtype)
            actual_shape = tuple(space.shape)
        except (AttributeError, TypeError, ValueError) as exc:
            _fail(f"final {name} space key {key!r} is malformed", exc)
        if actual_dtype != leaf["dtype"] or actual_shape != leaf["shape"]:
            _fail(
                f"final {name} space key {key!r} dtype/shape differs from the negotiated projection"
            )
        if isinstance(leaf["space"], gymnasium.spaces.Discrete):
            if not bool(getattr(space, "discrete", False)):
                _fail(f"final {name} space key {key!r} changed discrete semantics")
            continue
        low = getattr(space, "low", None)
        high = getattr(space, "high", None)
        if low is None or high is None:
            _fail(f"final {name} space key {key!r} has no finite bounds")
        actual_low = np.asarray(low)
        actual_high = np.asarray(high)
        expected_low = np.asarray(leaf["space"].low)
        expected_high = np.asarray(leaf["space"].high)
        normalized_action = (
            name == "action"
            and np.all(np.isfinite(expected_low))
            and np.all(np.isfinite(expected_high))
            and np.array_equal(actual_low, -np.ones_like(actual_low, dtype=np.float64))
            and np.array_equal(actual_high, np.ones_like(actual_high, dtype=np.float64))
        )
        if not normalized_action and (
            not np.array_equal(actual_low, expected_low)
            or not np.array_equal(actual_high, expected_high)
        ):
            _fail(
                f"final {name} space key {key!r} bounds differ from the negotiated projection"
            )
    if seen != {leaf["name"] for leaf in leaves}:
        _fail(f"final {name} space is missing negotiated leaves")


__all__ = [
    "DEMO_IO_CONTRACT_VERSION",
    "DEMO_PROJECTION_VERSION",
    "DreamerDemoProjection",
    "DreamerDemoProjectionError",
    "MAX_DEMO_PROJECTION_BYTES",
    "validate_demo_projection_descriptor",
]
