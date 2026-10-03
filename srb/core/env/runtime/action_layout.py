from __future__ import annotations

import copy
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Literal, TypeAlias, cast

import gymnasium
import numpy as np
import torch
from gymnasium.vector.utils import batch_space

from .validation import validate_non_negative_int, validate_positive_int

_ACTION_KEY_ALIASES: Mapping[str, tuple[str, ...]] = {
    "cmd_vel": ("wheeled_drive", "velocity_command"),
    "wheeled_drive": ("cmd_vel", "velocity_command"),
    "velocity_command": ("cmd_vel", "wheeled_drive"),
    "delta_twist": ("cartesian_delta_pose",),
    "cartesian_delta_pose": ("delta_twist",),
}

SpaceKind = Literal["mapping", "sequence", "leaf"]
SpacePath = tuple[str | int, ...]


def validate_finite_action(value: Any, *, path: SpacePath = ()) -> None:
    """Reject non-finite action leaves after ingress conversion, recursively."""

    if isinstance(value, torch.Tensor):
        if value.device.type == "meta":
            raise TypeError(
                "finite action validation requires executable tensor data; "
                "meta tensors are unsupported"
            )
        try:
            finite = torch.isfinite(value)
        except RuntimeError as exc:
            raise TypeError(
                f"action '{_path_name(path)}' must contain real numeric values"
            ) from exc
        if not bool(finite.all().item()):
            raise ValueError(f"action '{_path_name(path)}' contains non-finite values")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            validate_finite_action(child, path=(*path, str(key)))
        return
    if isinstance(value, (tuple, list)):
        for index, child in enumerate(value):
            validate_finite_action(child, path=(*path, index))
        return

    try:
        array = np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"action '{_path_name(path)}' is not a numeric array-like value"
        ) from exc
    dtype = np.dtype(array.dtype)
    _reject_non_numeric_dtype(dtype, path)
    try:
        finite = np.isfinite(array)
    except TypeError as exc:
        raise TypeError(
            f"action '{_path_name(path)}' must contain real numeric values"
        ) from exc
    if not bool(np.all(finite)):
        raise ValueError(f"action '{_path_name(path)}' contains non-finite values")


def _copy_space(space: Any) -> Any:
    """Deep-copy a supported Gym-style space at an authority seam."""

    return copy.deepcopy(space)


def classify_action_space(action_space: Any) -> SpaceKind:
    """Classify a Gym-style space for dispatch; recursion belongs to the layout."""
    if isinstance(action_space, Mapping):
        return "mapping"
    children = getattr(action_space, "spaces", None)
    if isinstance(children, Mapping):
        return "mapping"
    if isinstance(children, (list, tuple)):
        return "sequence"
    return "leaf"


def iter_action_space_children(action_space: Any) -> tuple[tuple[Any, Any], ...]:
    """Return direct action-space children in declaration order."""
    if isinstance(action_space, Mapping):
        return tuple(action_space.items())
    children = getattr(action_space, "spaces", None)
    if isinstance(children, Mapping):
        return tuple(children.items())
    if isinstance(children, (list, tuple)):
        return tuple(enumerate(children))
    return ()


def _space_shape(action_space: Any) -> tuple[int, ...]:
    shape = getattr(action_space, "shape", None)
    if shape is None:
        return ()
    try:
        normalized = tuple(int(dim) for dim in tuple(shape))
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"action space shape must be a sequence of integers, got {shape!r}"
        ) from exc
    if any(dim < 0 for dim in normalized):
        raise ValueError(f"action space shape cannot contain negatives: {shape!r}")
    return normalized


def _space_dtype(action_space: Any) -> np.dtype | None:
    dtype = getattr(action_space, "dtype", None)
    if dtype is None:
        return None
    try:
        return cast(np.dtype[Any], np.dtype(dtype))
    except TypeError as exc:
        raise TypeError(f"action space has unsupported dtype {dtype!r}") from exc


def _value_to_numpy(value: Any, *, path: SpacePath) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        array = value.detach().cpu().numpy()
        if not isinstance(array, np.ndarray):
            raise TypeError("torch action conversion did not produce a numpy array")
        return array
    if isinstance(value, np.ndarray):
        return value
    try:
        array = np.asarray(value)
        if not isinstance(array, np.ndarray):
            raise TypeError("action conversion did not produce a numpy array")
        return array
    except (TypeError, ValueError) as exc:
        name = "/".join(str(part) for part in path) or "action"
        raise TypeError(f"action '{name}' is not array-like") from exc


def _value_shape_dtype(
    value: Any, *, path: SpacePath
) -> tuple[tuple[int, ...], np.dtype | None]:
    """Return shape and NumPy dtype without copying Torch tensors to the host."""
    if isinstance(value, torch.Tensor):
        return tuple(int(dim) for dim in value.shape), None
    array = _value_to_numpy(value, path=path)
    return tuple(int(dim) for dim in array.shape), np.dtype(array.dtype)


def _reject_non_numeric_dtype(dtype: np.dtype | None, path: SpacePath) -> None:
    if dtype is None:
        return
    if np.issubdtype(dtype, np.complexfloating):
        raise TypeError(
            f"action '{_path_name(path)}' dtype {dtype} must contain real numeric values"
        )
    if not np.issubdtype(dtype, np.number) and dtype != np.bool_:
        raise TypeError(f"action '{_path_name(path)}' dtype {dtype} is not numeric")


def _validate_integral_domain(array: np.ndarray, path: SpacePath) -> None:
    """Reject values that cannot represent an integer action exactly."""

    name = _path_name(path)
    if array.dtype == np.bool_:
        raise ValueError(f"action '{name}' must contain integer values, not bool")
    if np.issubdtype(array.dtype, np.integer):
        return
    if np.issubdtype(array.dtype, np.floating):
        if not np.all(np.equal(array, np.floor(array))):
            raise ValueError(f"action '{name}' must contain integral values")
        return
    raise TypeError(f"action '{name}' must contain real numeric values")


def _validate_strict_leaf_domain(value: Any, space: Any, path: SpacePath) -> None:
    """Validate a bounded leaf without first narrowing it to float32."""

    array = _value_to_numpy(value, path=path)
    dtype = np.dtype(array.dtype)
    _reject_non_numeric_dtype(dtype, path)
    if np.issubdtype(dtype, np.complexfloating):
        raise TypeError(f"action '{_path_name(path)}' must contain real numeric values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"action '{_path_name(path)}' contains non-finite values")

    if isinstance(space, gymnasium.spaces.Discrete):
        _validate_integral_domain(array, path)
        start = int(space.start)
        stop = start + int(space.n)
        if np.any(array < start) or np.any(array >= stop):
            raise ValueError(
                f"action '{_path_name(path)}' is outside Discrete range "
                f"[{start}, {stop})"
            )
        return

    if isinstance(space, gymnasium.spaces.MultiDiscrete):
        _validate_integral_domain(array, path)
        multi_start = np.asarray(space.start)
        multi_stop = multi_start + np.asarray(space.nvec)
        try:
            below = np.less(array, multi_start)
            above = np.greater_equal(array, multi_stop)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"action '{_path_name(path)}' MultiDiscrete bounds do not "
                f"broadcast to shape {tuple(array.shape)}"
            ) from exc
        if np.any(below) or np.any(above):
            raise ValueError(
                f"action '{_path_name(path)}' is outside MultiDiscrete bounds"
            )
        return

    if isinstance(space, gymnasium.spaces.MultiBinary):
        if dtype != np.bool_:
            _validate_integral_domain(array, path)
        if np.any(array < 0) or np.any(array > 1):
            raise ValueError(
                f"action '{_path_name(path)}' is outside MultiBinary bounds [0, 1]"
            )
        return

    low = getattr(space, "low", None)
    high = getattr(space, "high", None)
    if low is None or high is None:
        return
    try:
        below = np.less(array, np.asarray(low))
        above = np.greater(array, np.asarray(high))
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"action '{_path_name(path)}' bounds do not broadcast to shape "
            f"{tuple(array.shape)}"
        ) from exc
    if np.any(below):
        raise ValueError(
            f"action '{_path_name(path)}' contains a value below its low bound"
        )
    if np.any(above):
        raise ValueError(
            f"action '{_path_name(path)}' contains a value above its high bound"
        )


@dataclass(frozen=True)
class GenericActionTerm:
    """One leaf in a recursively structured action space."""

    path: SpacePath
    shape: tuple[int, ...]
    dtype: np.dtype | None
    _space: Any
    action_slice: slice

    @property
    def size(self) -> int:
        return _action_size(self.shape)

    @property
    def space(self) -> Any:
        return _copy_space(self._space)


@dataclass(frozen=True)
class GenericActionLayout:
    """Handle arbitrary Gym spaces recursively for compatibility APIs."""

    _space: Any
    terms: tuple[GenericActionTerm, ...]
    kind: SpaceKind
    flat_dtype: np.dtype | None

    @classmethod
    def from_space(cls, action_space: Any) -> "GenericActionLayout":
        return cls._from_owned_space(_copy_space(action_space))

    @classmethod
    def _from_owned_space(cls, action_space: Any) -> "GenericActionLayout":
        terms: list[GenericActionTerm] = []
        offset = 0

        def walk(space: Any, path: SpacePath) -> None:
            nonlocal offset
            kind = classify_action_space(space)
            children = iter_action_space_children(space)
            if kind != "leaf":
                for key, child in children:
                    walk(child, (*path, key if isinstance(key, int) else str(key)))
                return

            shape = _space_shape(space)
            size = _action_size(shape)
            terms.append(
                GenericActionTerm(
                    path=path,
                    shape=shape,
                    dtype=_space_dtype(space),
                    _space=space,
                    action_slice=slice(offset, offset + size),
                )
            )
            offset += size

        walk(action_space, ())
        # Slash-joined paths key the flat mapping APIs; reject spaces whose keys
        # make two leaves share one path (e.g. "a/b" next to {"a": {"b": ...}}).
        seen_paths: set[str] = set()
        for term in terms:
            joined = "/".join(str(part) for part in term.path)
            if joined in seen_paths:
                raise ValueError(f"action space has ambiguous leaf path '{joined}'")
            seen_paths.add(joined)
        dtypes = [term.dtype for term in terms if term.dtype is not None]
        flat_dtype = np.dtype(np.result_type(*dtypes)) if dtypes else None
        return cls(
            _space=action_space,
            terms=tuple(terms),
            kind=classify_action_space(action_space),
            flat_dtype=flat_dtype,
        )

    @property
    def space(self) -> Any:
        return _copy_space(self._space)

    @property
    def flat_dim(self) -> int:
        return sum(term.size for term in self.terms)

    @property
    def flat_shape(self) -> tuple[int, ...]:
        return (self.flat_dim,)

    @property
    def paths(self) -> tuple[SpacePath, ...]:
        return tuple(term.path for term in self.terms)

    def schema(self) -> list[dict[str, Any]]:
        """Return ordered flat-vector metadata without rebuilding the layout."""
        schema: list[dict[str, Any]] = []
        for term in self.terms:
            path = [str(part) for part in term.path]
            start = term.action_slice.start
            end = term.action_slice.stop
            if start is None or end is None:
                raise ValueError(f"Action layout term has incomplete slice: {term}")
            schema.append(
                {
                    "name": "/".join(path) if path else "action",
                    "path": path,
                    "shape": list(term.shape) or [1],
                    "dim": term.size,
                    "start": start,
                    "end": end,
                    "dtype": str(term.dtype) if term.dtype is not None else None,
                }
            )
        return schema

    def space_children(
        self, action_space: Any | None = None
    ) -> tuple[tuple[Any, Any], ...]:
        return iter_action_space_children(
            self._space if action_space is None else action_space
        )

    def validate_mapping_keys(
        self, action: Mapping[Any, Any], spaces: Mapping[Any, Any] | None = None
    ) -> None:
        expected = dict(
            self.space_children(spaces) if spaces is not None else self.space_children()
        )
        actual_keys = set(action)
        expected_keys = set(expected)
        if actual_keys != expected_keys:
            missing = sorted(str(key) for key in expected_keys - actual_keys)
            extra = sorted(str(key) for key in actual_keys - expected_keys)
            details: list[str] = []
            if missing:
                details.append(f"missing={missing}")
            if extra:
                details.append(f"extra={extra}")
            raise ValueError(
                "mapping action keys do not match action space: " + ", ".join(details)
            )

    def flatten(self, action: Any) -> np.ndarray:
        """Validate and flatten a detailed or already-flat single action."""
        if self.kind == "leaf":
            return self._flatten_leaf(action, self.terms[0])
        if isinstance(action, (np.ndarray, torch.Tensor)):
            flat = _value_to_numpy(action, path=())
            self._validate_flat(flat, allow_numeric_cast=True)
            _reject_non_numeric_dtype(np.dtype(flat.dtype), ())
            if self.flat_dtype is not None:
                return flat.astype(self.flat_dtype, copy=True)
            return flat.copy()

        parts: list[np.ndarray] = []
        self._flatten_node(action, self._space, (), parts)
        if not parts:
            return np.zeros((0,), dtype=np.float32)
        for term, part in zip(self.terms, parts, strict=True):
            _reject_non_numeric_dtype(np.dtype(part.dtype), term.path)
        if self.flat_dtype is not None:
            parts = [part.astype(self.flat_dtype, copy=False) for part in parts]
        return _concatenate_arrays(parts)

    def structure(self, flat_action: Any) -> Any:
        """Validate a flat vector and rebuild the detailed action structure."""
        flat = _value_to_numpy(flat_action, path=())
        self._validate_flat(flat)
        return self._structure_node(flat, self._space, ())

    def flatten_untyped(self, action: Any) -> np.ndarray:
        """Flatten without schema validation for the legacy untyped API."""
        parts: list[np.ndarray] = []

        def walk(value: Any) -> None:
            if isinstance(value, Mapping):
                for child in value.values():
                    walk(child)
                return
            if isinstance(value, (tuple, list)) and not isinstance(value, np.ndarray):
                for child in value:
                    walk(child)
                return
            array = _value_to_numpy(value, path=())
            if array.ndim == 0:
                array = array.reshape(1)
            parts.append(array.reshape(-1))

        walk(action)
        if not parts:
            return np.zeros((0,), dtype=np.float32)
        return _concatenate_arrays(parts)

    def coerce_untyped(self, action: Any, *, device: Any = None) -> Any:
        """Coerce a value without a declared space schema."""
        if isinstance(action, Mapping):
            return {
                key: GenericActionLayout.from_space(None).coerce_untyped(
                    value, device=device
                )
                for key, value in action.items()
            }
        if isinstance(action, (tuple, list)) and not isinstance(action, np.ndarray):
            return type(action)(
                GenericActionLayout.from_space(None).coerce_untyped(
                    value, device=device
                )
                for value in action
            )
        return _coerce_action_tensor(action, device=device)

    def coerce(
        self,
        action: Any,
        *,
        device: Any = None,
        num_envs: int | None = None,
    ) -> Any:
        """Validate an action and return float32 Torch leaves on ``device``."""
        if num_envs is not None:
            return self.coerce_batched(action, num_envs=num_envs, device=device)
        if self.kind == "leaf":
            return self._coerce_leaf(action, self.terms[0], device=device)
        if isinstance(action, (np.ndarray, torch.Tensor)):
            shape, dtype = _value_shape_dtype(action, path=())
            self._validate_flat_signature(shape, dtype, allow_numeric_cast=True)
            return self._coerce_structure_from_flat(action, device=device)
        return self._coerce_node(action, self._space, (), device=device)

    def coerce_batched(
        self,
        action: Any,
        *,
        num_envs: int,
        device: Any = None,
        strict_domain: bool = False,
    ) -> Any:
        """Validate a batched action and return Torch leaves on ``device``."""
        num_envs = validate_positive_int(num_envs, name="num_envs")
        if strict_domain:
            self._validate_strict_domain_batched(action, num_envs=num_envs)
        if isinstance(action, (np.ndarray, torch.Tensor)):
            if self.kind == "leaf":
                return self._coerce_batched_leaf(
                    action, self.terms[0], num_envs, device
                )
            shape, _dtype = _value_shape_dtype(action, path=())
            expected_shape = (num_envs, self.flat_dim)
            single_action = shape == (self.flat_dim,) and num_envs == 1
            if single_action:
                shape = expected_shape
            if shape != expected_shape:
                raise ValueError(
                    f"batched flat action shape must be {expected_shape}, got {shape}"
                )
            flat_tensor = _coerce_action_tensor(action, device=device)
            if single_action:
                flat_tensor = flat_tensor.reshape(expected_shape)
            return self._coerce_torch_batched_node(
                flat_tensor, self._space, num_envs, ()
            )
        return self._coerce_batched_node(action, self._space, num_envs, (), device)

    def _validate_strict_domain_batched(self, action: Any, *, num_envs: int) -> None:
        if isinstance(action, (np.ndarray, torch.Tensor)):
            if self.kind == "leaf":
                term = self.terms[0]
                shape, _dtype = _value_shape_dtype(action, path=term.path)
                if shape == (num_envs, *term.shape) or (
                    num_envs == 1 and shape == term.shape
                ):
                    _validate_strict_leaf_domain(action, term._space, term.path)
                return

            shape, _dtype = _value_shape_dtype(action, path=())
            expected_shape = (num_envs, self.flat_dim)
            single_action = shape == (self.flat_dim,) and num_envs == 1
            if shape != expected_shape and not single_action:
                return
            flat = _value_to_numpy(action, path=()).reshape(expected_shape)
            for term in self.terms:
                start = term.action_slice.start
                stop = term.action_slice.stop
                if start is None or stop is None:
                    raise ValueError(f"Action layout term has incomplete slice: {term}")
                leaf = flat[:, start:stop].reshape((num_envs, *term.shape))
                _validate_strict_leaf_domain(leaf, term._space, term.path)
            return

        self._validate_strict_domain_node(
            action, self._space, num_envs=num_envs, path=()
        )

    def _validate_strict_domain_node(
        self, action: Any, space: Any, *, num_envs: int, path: SpacePath
    ) -> None:
        kind = classify_action_space(space)
        children = iter_action_space_children(space)
        if kind == "mapping":
            if not isinstance(action, Mapping):
                return
            self.validate_mapping_keys(action, dict(children))
            for key, child in children:
                child_path = (*path, key if isinstance(key, int) else str(key))
                self._validate_strict_domain_node(
                    action[key], child, num_envs=num_envs, path=child_path
                )
            return
        if kind == "sequence":
            if not isinstance(action, (tuple, list)):
                return
            if len(action) != len(children):
                return
            for (key, child), value in zip(children, action, strict=True):
                child_path = (*path, key if isinstance(key, int) else str(key))
                self._validate_strict_domain_node(
                    value, child, num_envs=num_envs, path=child_path
                )
            return

        shape, _dtype = _value_shape_dtype(action, path=path)
        leaf_shape = _space_shape(space)
        if shape == (num_envs, *leaf_shape) or (num_envs == 1 and shape == leaf_shape):
            _validate_strict_leaf_domain(action, space, path)

    def policy_action(
        self, action: Any, *, device: Any = None, num_envs: int = 1
    ) -> Any:
        """Validate per-env policy output against the single-env space.

        Accept ``(num_envs, *shape)``, or ``shape`` when ``num_envs == 1``.
        """
        if self.kind != "leaf" and isinstance(action, Mapping):
            self.validate_mapping_keys(action)
            mapping_children = dict(self.space_children())
            return {
                key: GenericActionLayout._from_owned_space(
                    mapping_children[key]
                ).policy_action(action[key], device=device, num_envs=num_envs)
                for key in mapping_children
            }
        if self.kind == "sequence" and isinstance(action, (tuple, list)):
            sequence_children = self.space_children()
            if len(action) != len(sequence_children):
                raise ValueError(
                    f"tuple action has {len(action)} values but action space needs "
                    f"{len(sequence_children)}"
                )
            return tuple(
                GenericActionLayout._from_owned_space(space).policy_action(
                    action[index], device=device, num_envs=num_envs
                )
                for index, (_key, space) in enumerate(sequence_children)
            )

        term = self.terms[0]
        shape, _dtype = _value_shape_dtype(action, path=term.path)
        target_shape = term.shape
        batched_shape = (num_envs, *target_shape)
        if shape == target_shape and num_envs == 1:
            if isinstance(action, torch.Tensor):
                value: Any = action.reshape(batched_shape)
            else:
                value = np.asarray(action).reshape(batched_shape)
        elif shape == batched_shape:
            value = action
        else:
            expected = (
                f"{target_shape} or {batched_shape}"
                if num_envs == 1
                else str(batched_shape)
            )
            raise ValueError(
                f"action '{_path_name(term.path)}' shape must be {expected}, "
                f"got {shape}"
            )
        if device is not None:
            return torch.as_tensor(value, device=device)
        return value

    def policy_action_untyped(self, action: Any, *, device: Any = None) -> Any:
        """Preserve historical policy coercion when no space is available."""
        if isinstance(action, Mapping):
            return {
                key: self.policy_action_untyped(value, device=device)
                for key, value in action.items()
            }
        if isinstance(action, (tuple, list)) and not isinstance(action, np.ndarray):
            return type(action)(
                self.policy_action_untyped(value, device=device) for value in action
            )
        return torch.as_tensor(action, device=device) if device is not None else action

    def unflatten_path_mapping(self, value: Mapping[str, Any]) -> Any:
        """Rebuild a detailed structure from slash-delimited leaf paths."""
        expected = {"/".join(str(part) for part in term.path) for term in self.terms}
        actual = set(value)
        if actual != expected:
            missing = sorted(expected - actual)
            extra = sorted(actual - expected)
            details: list[str] = []
            if missing:
                details.append(f"missing={missing}")
            if extra:
                details.append(f"extra={extra}")
            raise ValueError(
                "action paths do not match action space: " + ", ".join(details)
            )

        def build(space: Any, path: SpacePath) -> Any:
            kind = classify_action_space(space)
            children = iter_action_space_children(space)
            if kind == "mapping":
                return {key: build(child, (*path, str(key))) for key, child in children}
            if kind == "sequence":
                return tuple(
                    build(child, (*path, index))
                    for index, (_key, child) in enumerate(children)
                )
            return value["/".join(str(part) for part in path)]

        return build(self._space, ())

    def flat_space_mapping(self) -> dict[str, Any]:
        return {
            "/".join(str(part) for part in term.path): term.space for term in self.terms
        }

    def _flatten_node(
        self,
        value: Any,
        space: Any,
        path: SpacePath,
        parts: list[np.ndarray],
    ) -> None:
        kind = classify_action_space(space)
        children = iter_action_space_children(space)
        if kind == "mapping":
            if not isinstance(value, Mapping):
                raise TypeError(
                    f"action at '{_path_name(path)}' must be a mapping, "
                    f"got {type(value).__name__}"
                )
            self.validate_mapping_keys(value, dict(children))
            for key, child in children:
                self._flatten_node(value[key], child, (*path, str(key)), parts)
            return
        if kind == "sequence":
            if not isinstance(value, (tuple, list)):
                raise TypeError(
                    f"action at '{_path_name(path)}' must be a tuple/list, "
                    f"got {type(value).__name__}"
                )
            if len(value) != len(children):
                raise ValueError(
                    f"action at '{_path_name(path)}' has {len(value)} values, "
                    f"expected {len(children)}"
                )
            for index, (_key, child) in enumerate(children):
                self._flatten_node(value[index], child, (*path, index), parts)
            return
        term = next(term for term in self.terms if term.path == path)
        parts.append(self._flatten_leaf(value, term))

    def _flatten_leaf(self, value: Any, term: GenericActionTerm) -> np.ndarray:
        array = _value_to_numpy(value, path=term.path)
        _reject_non_numeric_dtype(np.dtype(array.dtype), term.path)
        if tuple(array.shape) != term.shape:
            raise ValueError(
                f"action '{_path_name(term.path)}' shape must be {term.shape}, "
                f"got {tuple(array.shape)}"
            )
        return array.reshape(-1)

    def _validate_flat(
        self, flat: np.ndarray, *, allow_numeric_cast: bool = False
    ) -> None:
        self._validate_flat_signature(
            tuple(int(dim) for dim in flat.shape),
            np.dtype(flat.dtype),
            allow_numeric_cast=allow_numeric_cast,
        )

    def _validate_flat_signature(
        self,
        shape: tuple[int, ...],
        dtype: np.dtype | None,
        *,
        allow_numeric_cast: bool = False,
    ) -> None:
        if len(shape) != 1 or shape != self.flat_shape:
            raise ValueError(
                f"flat action shape must be {self.flat_shape}, got {shape}"
            )
        if (
            self.flat_dtype is not None
            and dtype is not None
            and dtype != self.flat_dtype
            and not allow_numeric_cast
        ):
            raise TypeError(f"flat action dtype must be {self.flat_dtype}, got {dtype}")

    def _structure_node(
        self,
        flat: np.ndarray,
        space: Any,
        path: SpacePath,
    ) -> Any:
        kind = classify_action_space(space)
        children = iter_action_space_children(space)
        if kind == "mapping":
            return {
                key: self._structure_node(flat, child_space, (*path, str(key)))
                for key, child_space in children
            }
        if kind == "sequence":
            return tuple(
                self._structure_node(flat, child_space, (*path, index))
                for index, (_key, child_space) in enumerate(children)
            )
        term = next(term for term in self.terms if term.path == path)
        piece = flat[term.action_slice]
        return piece.reshape(term.shape)

    def _coerce_structure_from_flat(self, flat_action: Any, *, device: Any) -> Any:
        flat = _coerce_action_tensor(flat_action, device=device)
        if flat.ndim != 1 or tuple(flat.shape) != self.flat_shape:
            raise ValueError(
                f"flat action shape must be {self.flat_shape}, got {tuple(flat.shape)}"
            )
        return self._coerce_torch_node(flat, self._space, ())

    def _coerce_node(
        self, value: Any, space: Any, path: SpacePath, *, device: Any
    ) -> Any:
        kind = classify_action_space(space)
        children = iter_action_space_children(space)
        if kind == "mapping":
            if not isinstance(value, Mapping):
                raise TypeError(
                    f"action at '{_path_name(path)}' must be a mapping, "
                    f"got {type(value).__name__}"
                )
            self.validate_mapping_keys(value, dict(children))
            return {
                key: self._coerce_node(
                    value[key], child_space, (*path, str(key)), device=device
                )
                for key, child_space in children
            }
        if kind == "sequence":
            if not isinstance(value, (tuple, list)):
                raise TypeError(
                    f"action at '{_path_name(path)}' must be a tuple/list, "
                    f"got {type(value).__name__}"
                )
            if len(value) != len(children):
                raise ValueError(
                    f"action at '{_path_name(path)}' has {len(value)} values, "
                    f"expected {len(children)}"
                )
            return tuple(
                self._coerce_node(value[index], child, (*path, index), device=device)
                for index, (_key, child) in enumerate(children)
            )
        return self._coerce_leaf(
            value, next(term for term in self.terms if term.path == path), device=device
        )

    def _coerce_batched_node(
        self,
        value: Any,
        space: Any,
        num_envs: int,
        path: SpacePath,
        device: Any,
    ) -> Any:
        kind = classify_action_space(space)
        children = iter_action_space_children(space)
        if kind == "mapping":
            if not isinstance(value, Mapping):
                raise TypeError(
                    f"action at '{_path_name(path)}' must be a mapping, "
                    f"got {type(value).__name__}"
                )
            self.validate_mapping_keys(value, dict(children))
            return {
                key: self._coerce_batched_node(
                    value[key], child, num_envs, (*path, str(key)), device
                )
                for key, child in children
            }
        if kind == "sequence":
            if not isinstance(value, (tuple, list)):
                raise TypeError(
                    f"action at '{_path_name(path)}' must be a tuple/list, "
                    f"got {type(value).__name__}"
                )
            if len(value) != len(children):
                raise ValueError(
                    f"action at '{_path_name(path)}' has {len(value)} values, "
                    f"expected {len(children)}"
                )
            return tuple(
                self._coerce_batched_node(
                    value[index], child, num_envs, (*path, index), device
                )
                for index, (_key, child) in enumerate(children)
            )
        return self._coerce_batched_leaf(
            value,
            next(term for term in self.terms if term.path == path),
            num_envs,
            device,
        )

    def _coerce_torch_batched_node(
        self,
        flat: torch.Tensor,
        space: Any,
        num_envs: int,
        path: SpacePath,
    ) -> Any:
        kind = classify_action_space(space)
        children = iter_action_space_children(space)
        if kind == "mapping":
            return {
                key: self._coerce_torch_batched_node(
                    flat, child, num_envs, (*path, str(key))
                )
                for key, child in children
            }
        if kind == "sequence":
            return tuple(
                self._coerce_torch_batched_node(flat, child, num_envs, (*path, index))
                for index, (_key, child) in enumerate(children)
            )
        term = next(term for term in self.terms if term.path == path)
        return flat[:, term.action_slice].reshape((num_envs, *term.shape))

    def _coerce_torch_node(
        self,
        flat: torch.Tensor,
        space: Any,
        path: SpacePath,
    ) -> Any:
        kind = classify_action_space(space)
        children = iter_action_space_children(space)
        if kind == "mapping":
            return {
                key: self._coerce_torch_node(flat, child_space, (*path, str(key)))
                for key, child_space in children
            }
        if kind == "sequence":
            return tuple(
                self._coerce_torch_node(flat, child_space, (*path, index))
                for index, (_key, child_space) in enumerate(children)
            )
        term = next(term for term in self.terms if term.path == path)
        piece = flat[term.action_slice]
        return piece.reshape(term.shape)

    @staticmethod
    def _coerce_leaf(
        value: Any, term: GenericActionTerm, *, device: Any
    ) -> torch.Tensor:
        shape, dtype = _value_shape_dtype(value, path=term.path)
        expected = term.shape
        if shape != expected and shape != (1, *expected):
            raise ValueError(
                f"action '{_path_name(term.path)}' shape must be {expected} or "
                f"{(1, *expected)}, got {shape}"
            )
        _reject_non_numeric_dtype(dtype, term.path)
        return _coerce_action_tensor(value, device=device, name=_path_name(term.path))

    @staticmethod
    def _coerce_batched_leaf(
        value: Any,
        term: GenericActionTerm,
        num_envs: int,
        device: Any,
    ) -> torch.Tensor:
        shape, dtype = _value_shape_dtype(value, path=term.path)
        expected = term.shape
        batched_shape = (num_envs, *expected)
        _reject_non_numeric_dtype(dtype, term.path)
        if shape == expected and num_envs == 1:
            tensor = _coerce_action_tensor(
                value, device=device, name=_path_name(term.path)
            )
            return tensor.reshape(batched_shape)
        if shape != batched_shape:
            raise ValueError(
                f"action '{_path_name(term.path)}' shape must be {expected} or "
                f"{batched_shape}, got {shape}"
            )
        return _coerce_action_tensor(value, device=device, name=_path_name(term.path))


def _path_name(path: SpacePath) -> str:
    return "/".join(str(part) for part in path) or "action"


def _concatenate_arrays(parts: list[np.ndarray]) -> np.ndarray:
    result = np.concatenate(parts, axis=0)
    if not isinstance(result, np.ndarray):
        raise TypeError("action concatenation did not produce a numpy array")
    return result


def _empty_array(dtype: Any) -> np.ndarray:
    result = np.zeros((0,), dtype=dtype)
    if not isinstance(result, np.ndarray):
        raise TypeError("action allocation did not produce a numpy array")
    return result


def _copy_torch_array(value: torch.Tensor) -> np.ndarray:
    array = value.detach().cpu().numpy()
    if not isinstance(array, np.ndarray):
        raise TypeError("torch action conversion did not produce a numpy array")
    return array.copy()


def _action_size(shape: tuple[int, ...]) -> int:
    return int(np.prod(shape, dtype=np.int64))


def _split_action_key(key: str) -> tuple[str, str]:
    if "/" not in key:
        return "", key
    prefix, base = key.rsplit("/", 1)
    return prefix, base


def _scale_suffixes(scale: Mapping[str, float], action_key: str) -> dict[str, float]:
    suffixes: dict[str, float] = {}
    for key, value in scale.items():
        if key.startswith(f"{action_key}_"):
            suffixes[key.removeprefix(f"{action_key}_")] = value
    _, base_key = _split_action_key(action_key)
    if base_key != action_key:
        for key, value in scale.items():
            if key.startswith(f"{base_key}_"):
                suffixes.setdefault(key.removeprefix(f"{base_key}_"), value)
    return suffixes


@dataclass(frozen=True)
class ActionTerm:
    key: str
    action_slice: slice
    shape: tuple[int, ...]
    dtype: np.dtype
    _space: gymnasium.spaces.Box

    @property
    def size(self) -> int:
        return _action_size(self.shape)

    @property
    def space(self) -> gymnasium.spaces.Box:
        return cast(gymnasium.spaces.Box, _copy_space(self._space))


def _concatenate_term_spaces(
    terms: Sequence[ActionTerm],
    *,
    total_dim: int,
) -> gymnasium.spaces.Box:
    """Flat action space retaining each term's bounds and dtype.

    A normalized ``[-1, 1]`` ``float32`` box would misreport limits to policies
    and samplers.
    """
    if not terms:
        return gymnasium.spaces.Box(
            low=-1.0, high=1.0, shape=(total_dim,), dtype=np.float32
        )
    dtype = np.result_type(*(term.dtype for term in terms))
    if not np.issubdtype(dtype, np.floating):
        dtype = np.dtype(np.float32)
    low = np.empty((total_dim,), dtype=dtype)
    high = np.empty((total_dim,), dtype=dtype)
    for term in terms:
        space = term.space
        low[term.action_slice] = np.asarray(space.low, dtype=dtype).reshape(-1)
        high[term.action_slice] = np.asarray(space.high, dtype=dtype).reshape(-1)
    # ``Box`` annotates ``dtype`` as a scalar type even though it normalizes
    # whatever it is handed through ``np.dtype``; pass the scalar type so the
    # strict type checking sees the contract the runtime already honours.
    return gymnasium.spaces.Box(low=low, high=high, dtype=dtype.type)


@dataclass(frozen=True)
class ActionLayout:
    terms: tuple[ActionTerm, ...]
    _single_space: gymnasium.spaces.Box
    _detailed_space: gymnasium.spaces.Dict
    _scale: Mapping[str, float]

    @classmethod
    def from_space(
        cls,
        action_space: Any,
        *,
        source_layout: object | None = None,
    ) -> ActionLayoutLike:
        """Build the adapter backend for the effective action space.

        Rank-one vectors and named flat terms use the runtime layout; nested
        spaces use the recursive backend.
        """
        if (
            isinstance(source_layout, ActionLayout)
            and isinstance(action_space, gymnasium.spaces.Box)
            and len(action_space.shape) == 1
            and tuple(source_layout.single_space.shape) == tuple(action_space.shape)
        ):
            return source_layout
        if (
            isinstance(action_space, gymnasium.spaces.Box)
            and len(action_space.shape) == 1
        ):
            return cls.from_box(action_space)
        if isinstance(action_space, gymnasium.spaces.Dict) and all(
            isinstance(space, gymnasium.spaces.Box)
            for space in action_space.spaces.values()
        ):
            return cls.from_spaces(action_space)
        return cls.from_structured_space(action_space)

    @staticmethod
    def from_structured_space(action_space: Any) -> StructuredActionLayout:
        """Build the structure-preserving backend through the canonical facade."""
        return GenericActionLayout.from_space(action_space)

    @classmethod
    def empty(cls) -> ActionLayout:
        return cls.from_action_dim(0)

    @classmethod
    def from_box(cls, action_space: gymnasium.spaces.Box) -> ActionLayout:
        """Flatten one rank-1 ``Box`` while preserving bounds and dtype.

        Only an absent space falls back to :meth:`from_action_dim`'s normalized
        ``float32`` default.
        """
        if not isinstance(action_space, gymnasium.spaces.Box):
            raise TypeError("from_box requires a gymnasium Box action space")
        if len(action_space.shape) != 1:
            raise ValueError(
                "from_box requires a rank-1 Box action space, "
                f"got shape {tuple(action_space.shape)}"
            )
        single_space = gymnasium.spaces.Box(
            low=np.array(action_space.low, copy=True),
            high=np.array(action_space.high, copy=True),
            shape=tuple(action_space.shape),
            # ``Space.dtype`` is optional and widens to ``void`` in the stubs.
            # A ``Box`` always carries a concrete numeric dtype, and the
            # isinstance guard above already rejected everything that is not
            # one, so narrow it here rather than re-validating at runtime.
            dtype=cast(
                "type[np.floating[Any]] | type[np.integer[Any]]",
                np.dtype(action_space.dtype).type,
            ),
        )
        return cls(
            terms=(),
            _single_space=single_space,
            _detailed_space=gymnasium.spaces.Dict({}),
            _scale={},
        )

    @classmethod
    def from_action_dim(cls, action_dim: int) -> ActionLayout:
        action_dim = validate_non_negative_int(action_dim, name="action_dim")
        single_space = gymnasium.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(action_dim,),
            dtype=np.float32,
        )
        return cls(
            terms=(),
            _single_space=single_space,
            _detailed_space=gymnasium.spaces.Dict({}),
            _scale={},
        )

    @classmethod
    def from_action_manager(cls, action_manager: object | None) -> ActionLayout:
        if action_manager is None:
            return cls.empty()
        iter_terms = getattr(action_manager, "iter_terms", None)
        if not callable(iter_terms):
            return cls.from_action_dim(
                validate_non_negative_int(
                    getattr(action_manager, "total_action_dim", 0),
                    name="action_dim",
                )
            )

        action_space_dict: dict[str, gymnasium.Space[Any]] = {}
        ordering: dict[str, slice] = {}
        scale: dict[str, float] = {}
        action_index = 0
        for action_key, action_term in iter_terms():
            action_dim = validate_non_negative_int(
                getattr(action_term, "action_dim", 0),
                name="action_dim",
            )
            action_space_dict[action_key] = gymnasium.spaces.Box(
                low=-1.0,
                high=1.0,
                shape=(action_dim,),
                dtype=np.float32,
            )
            ordering[action_key] = slice(action_index, action_index + action_dim)
            action_index += action_dim
            scale.update(_extract_scale(action_key, getattr(action_term, "cfg", None)))

        return cls.from_spaces(
            gymnasium.spaces.Dict(action_space_dict),
            ordering=ordering,
            scale=scale,
        )

    @classmethod
    def from_spaces(
        cls,
        detailed_space: gymnasium.spaces.Dict,
        *,
        ordering: Mapping[str, slice] | None = None,
        scale: Mapping[str, float] | None = None,
        single_space: gymnasium.Space[Any] | None = None,
    ) -> ActionLayout:
        if not isinstance(detailed_space, gymnasium.spaces.Dict):
            raise TypeError(
                f"Action layout requires a Dict space, got {detailed_space}"
            )
        detailed_space = cast(
            gymnasium.spaces.Dict,
            _copy_space(detailed_space),
        )

        inferred_ordering: dict[str, slice] = {}
        action_index = 0
        for action_key, action_space in detailed_space.spaces.items():
            if not isinstance(action_space, gymnasium.spaces.Box):
                raise TypeError(
                    f"Action '{action_key}' must use a Box space, got {action_space}"
                )
            action_size = _action_size(tuple(action_space.shape))
            inferred_ordering[action_key] = slice(
                action_index,
                action_index + action_size,
            )
            action_index += action_size

        ordering = dict(inferred_ordering if ordering is None else ordering)
        if set(ordering) != set(detailed_space.spaces):
            raise ValueError(
                f"Action ordering keys {set(ordering)} do not match detailed "
                f"space keys {set(detailed_space.spaces)}"
            )
        for action_key, action_slice in ordering.items():
            if not isinstance(action_slice, slice):
                raise TypeError(
                    f"Action '{action_key}' ordering must use a slice, "
                    f"got {type(action_slice).__name__}"
                )

        terms: list[ActionTerm] = []
        for action_key, action_slice in sorted(
            ordering.items(),
            key=lambda item: (item[1].start or 0, item[0]),
        ):
            action_space = detailed_space.spaces[action_key]
            if not isinstance(action_space, gymnasium.spaces.Box):
                raise TypeError(
                    f"Action '{action_key}' must use a Box space, got {action_space}"
                )
            if action_slice.step not in (None, 1):
                raise ValueError(
                    f"Action '{action_key}' ordering slice must have step 1, "
                    f"got {action_slice}"
                )
            if action_slice.start is None or action_slice.stop is None:
                raise ValueError(
                    f"Action '{action_key}' ordering slice needs explicit start "
                    f"and stop, got {action_slice}"
                )
            if action_slice.start < 0 or action_slice.stop < action_slice.start:
                raise ValueError(
                    f"Action '{action_key}' ordering slice is invalid: {action_slice}"
                )
            shape = tuple(action_space.shape)
            expected_stop = action_slice.start + _action_size(shape)
            if action_slice.stop != expected_stop:
                raise ValueError(
                    f"Action '{action_key}' slice {action_slice} does not match "
                    f"space shape {shape}"
                )
            terms.append(
                ActionTerm(
                    key=action_key,
                    action_slice=action_slice,
                    shape=shape,
                    dtype=np.dtype(action_space.dtype),
                    _space=action_space,
                )
            )

        # The slices (sorted by start above) must tile [0, total_dim) exactly:
        # contiguous, gap-free, non-overlapping. from_action_manager is
        # contiguous by construction, but a hand-authored ``ordering`` (e.g. a
        # RealEnv ACTION_ORDERING ClassVar) could overlap or leave a gap and
        # silently mis-slice actions in structure() with no diagnostic.
        cursor = 0
        for term in terms:
            start = term.action_slice.start
            stop = term.action_slice.stop
            if start is None or stop is None:
                raise ValueError(
                    f"Action '{term.key}' ordering slice must be concrete, "
                    f"got {term.action_slice}"
                )
            if start != cursor:
                raise ValueError(
                    f"Action ordering is not a contiguous, non-overlapping tiling: "
                    f"term '{term.key}' starts at {start}, expected {cursor}. "
                    f"Slices must cover [0, total_dim) with no gaps or overlaps."
                )
            cursor = stop

        total_dim = max(
            (
                term.action_slice.stop
                for term in terms
                if term.action_slice.stop is not None
            ),
            default=0,
        )
        if single_space is None:
            single_space = _concatenate_term_spaces(terms, total_dim=total_dim)
        if not isinstance(single_space, gymnasium.spaces.Box):
            raise TypeError(f"Single action space must be a Box, got {single_space}")
        single_space = cast(gymnasium.spaces.Box, _copy_space(single_space))
        if terms and tuple(single_space.shape) != (total_dim,):
            raise ValueError(
                f"Single action space shape must be {(total_dim,)}, "
                f"got {tuple(single_space.shape)}"
            )

        return cls(
            terms=tuple(terms),
            _single_space=single_space,
            _detailed_space=detailed_space,
            _scale=dict(scale or {}),
        )

    @property
    def single_space(self) -> gymnasium.spaces.Box:
        return cast(gymnasium.spaces.Box, _copy_space(self._single_space))

    @property
    def detailed_space(self) -> gymnasium.spaces.Dict:
        return cast(gymnasium.spaces.Dict, _copy_space(self._detailed_space))

    @property
    def ordering(self) -> dict[str, slice]:
        return {term.key: term.action_slice for term in self.terms}

    @property
    def scale(self) -> dict[str, float]:
        return dict(self._scale)

    def build_spaces(
        self,
        *,
        num_envs: int,
    ) -> tuple[gymnasium.spaces.Box, gymnasium.Space[Any]]:
        num_envs = validate_positive_int(num_envs, name="num_envs")
        single_space = cast(gymnasium.spaces.Box, _copy_space(self._single_space))
        batched_space = batch_space(
            cast(gymnasium.spaces.Box, _copy_space(self._single_space)),
            n=num_envs,
        )
        return single_space, batched_space

    def coerce(
        self,
        action: Any,
        *,
        num_envs: int,
        device: Any,
    ) -> torch.Tensor:
        """Validate a flat vector or named term mapping and return a batched tensor.

        Convert once here; the result stays on the runtime device for action
        processing.
        """
        num_envs = validate_positive_int(num_envs, name="num_envs")
        if isinstance(action, Mapping):
            expected_keys = set(self.ordering)
            actual_keys = set(action)
            if actual_keys != expected_keys:
                missing = sorted(expected_keys - actual_keys)
                extra = sorted(actual_keys - expected_keys)
                details: list[str] = []
                if missing:
                    details.append(f"missing={missing}")
                if extra:
                    details.append(f"extra={extra}")
                raise ValueError(
                    "Action keys do not match expected keys: " + ", ".join(details)
                )

            terms: list[torch.Tensor] = []
            for term in self.terms:
                value = _coerce_action_tensor(
                    action[term.key], device=device, name=term.key
                )
                value = _coerce_batched_term(
                    value,
                    expected_shape=term.shape,
                    num_envs=num_envs,
                    key=term.key,
                )
                terms.append(value.reshape(num_envs, -1))
            if not terms:
                return torch.zeros((num_envs, 0), dtype=torch.float32, device=device)
            return torch.cat(terms, dim=-1)

        value = _coerce_action_tensor(action, device=device)
        single_shape = tuple(self._single_space.shape)
        batched_shape = (num_envs, *single_shape)
        if tuple(value.shape) == single_shape and num_envs == 1:
            return value.reshape(batched_shape)
        if tuple(value.shape) != batched_shape:
            raise ValueError(
                f"Action shape {tuple(value.shape)} does not match expected "
                f"single shape {single_shape} or batched shape {batched_shape}"
            )
        return value

    def source_spaces(self) -> tuple[tuple[str, gymnasium.spaces.Box], ...]:
        return tuple((term.key, term.space) for term in self.terms)

    def structure(
        self,
        action: np.ndarray | torch.Tensor | Mapping[str, np.ndarray | torch.Tensor],
    ) -> dict[str, np.ndarray]:
        if isinstance(action, Mapping):
            if any(isinstance(value, torch.Tensor) for value in action.values()):
                action = {
                    action_key: value.detach().cpu().numpy()
                    if isinstance(value, torch.Tensor)
                    else value
                    for action_key, value in action.items()
                }
            expected_keys = set(self.ordering)
            actual_keys = set(action)
            if actual_keys != expected_keys:
                missing = sorted(expected_keys - actual_keys)
                extra = sorted(actual_keys - expected_keys)
                details: list[str] = []
                if missing:
                    details.append(f"missing={missing}")
                if extra:
                    details.append(f"extra={extra}")
                raise ValueError(
                    "Action keys do not match expected keys: " + ", ".join(details)
                )
            structured: dict[str, np.ndarray] = {}
            for term in self.terms:
                value = action[term.key]
                if not isinstance(value, np.ndarray):
                    raise TypeError(
                        f"Action '{term.key}' must be a numpy array or torch tensor, "
                        f"got {type(value).__name__}"
                    )
                if tuple(value.shape) != term.shape:
                    raise ValueError(
                        f"Action '{term.key}' shape must be {term.shape}, "
                        f"got {tuple(value.shape)}"
                    )
                if np.dtype(value.dtype) != term.dtype:
                    raise TypeError(
                        f"Action '{term.key}' dtype must be {term.dtype}, "
                        f"got {value.dtype}"
                    )
                structured[term.key] = value
        else:
            if isinstance(action, torch.Tensor):
                action = action.detach().cpu().numpy()
            if not isinstance(action, np.ndarray):
                raise TypeError(
                    "Action must be a numpy array, torch tensor, or mapping, "
                    f"got {type(action).__name__}"
                )
            if tuple(action.shape) != tuple(self._single_space.shape):
                raise ValueError(
                    f"Action shape {tuple(action.shape)} does not match expected "
                    f"shape {tuple(self._single_space.shape)}"
                )
            if np.dtype(action.dtype) != np.dtype(self._single_space.dtype):
                raise TypeError(
                    f"Action dtype must be {self._single_space.dtype}, got {action.dtype}"
                )
            structured = {
                term.key: action[term.action_slice].reshape(term.shape)
                for term in self.terms
            }

        return structured

    def flatten(
        self,
        action: Mapping[str, np.ndarray | torch.Tensor] | np.ndarray | torch.Tensor,
    ) -> np.ndarray:
        """Validate a detailed action and return the canonical flat vector."""
        if isinstance(action, Mapping):
            structured = self.structure(action)
            if not self.terms:
                return _empty_array(self._single_space.dtype)
            return _concatenate_arrays(
                [structured[term.key].reshape(-1) for term in self.terms]
            )
        self.structure(action)
        if isinstance(action, torch.Tensor):
            return _copy_torch_array(action)
        return action.copy()

    def match_keys(self, action_key: str) -> tuple[str, ...]:
        prefix, base_key = _split_action_key(action_key)
        candidates = [action_key]
        if base_key != action_key:
            candidates.append(base_key)
        for alias in _ACTION_KEY_ALIASES.get(base_key, ()):
            if prefix:
                candidates.append(f"{prefix}/{alias}")
            candidates.append(alias)
        return tuple(dict.fromkeys(candidates))

    def scale_for_hardware(
        self,
        mapping: Sequence[tuple[str, str]],
    ) -> dict[str, float]:
        scale = dict(self._scale)
        for action_key, hardware_key in mapping:
            scale[hardware_key] = scale.get(
                action_key,
                scale.get(_split_action_key(action_key)[1], 1.0),
            )
            for suffix, value in _scale_suffixes(scale, action_key).items():
                scale[f"{hardware_key}_{suffix}"] = value
        return scale


ActionLayoutLike: TypeAlias = ActionLayout | GenericActionLayout
StructuredActionLayout: TypeAlias = GenericActionLayout


def _coerce_action_tensor(
    value: Any, *, device: Any, name: str = "action"
) -> torch.Tensor:
    # Every float32 action cast in this module goes through here. The cast
    # keeps only the real part of a complex value (Torch and NumPy warn at
    # most), so reject complex input first, as the strict path does.
    if isinstance(value, torch.Tensor):
        if value.is_complex():
            raise TypeError(f"action '{name}' must contain real numeric values")
        return value.to(device=device, dtype=torch.float32)
    if isinstance(value, (np.ndarray, np.generic)) and np.iscomplexobj(value):
        raise TypeError(f"action '{name}' must contain real numeric values")
    return torch.as_tensor(value, device=device, dtype=torch.float32)


def _coerce_batched_term(
    value: torch.Tensor,
    *,
    expected_shape: tuple[int, ...],
    num_envs: int,
    key: str,
) -> torch.Tensor:
    batched_shape = (num_envs, *expected_shape)
    if tuple(value.shape) == expected_shape and num_envs == 1:
        return value.reshape(batched_shape)
    if tuple(value.shape) != batched_shape:
        raise ValueError(
            f"Action '{key}' shape {tuple(value.shape)} does not match "
            f"expected single shape {expected_shape} or batched shape "
            f"{batched_shape}"
        )
    return value


def _extract_scale(action_key: str, action_term_cfg: Any | None) -> dict[str, float]:
    if action_term_cfg is None:
        return {}
    scale: dict[str, float] = {}
    if hasattr(action_term_cfg, "scale"):
        scale[action_key] = action_term_cfg.scale
    if hasattr(action_term_cfg, "scale_linear") and hasattr(
        action_term_cfg, "scale_angular"
    ):
        scale[f"{action_key}_linear"] = action_term_cfg.scale_linear
        scale[f"{action_key}_angular"] = action_term_cfg.scale_angular
    if hasattr(action_term_cfg, "position_scale") and hasattr(
        action_term_cfg, "orientation_scale"
    ):
        scale[f"{action_key}_position"] = action_term_cfg.position_scale
        scale[f"{action_key}_orientation"] = action_term_cfg.orientation_scale
    return scale


__all__ = [
    "ActionLayout",
    "ActionLayoutLike",
    "ActionTerm",
    "GenericActionLayout",
    "GenericActionTerm",
    "StructuredActionLayout",
    "classify_action_space",
    "iter_action_space_children",
    "validate_finite_action",
]
