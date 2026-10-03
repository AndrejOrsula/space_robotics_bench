from __future__ import annotations

import copy
import functools
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, TypeAlias, cast

import gymnasium
import numpy as np
import torch
from gymnasium.vector.utils import batch_space

from srb._typing import ObservationGroup
from srb._typing.step_return import StepReturn
from srb.core.spaces.keys import ROOT_OBSERVATION_KEY
from srb.utils.json_types import ImmutableValueMixin

from .validation import validate_positive_int

_TORCH_TO_NUMPY = {
    torch.float32: np.float32,
    torch.float64: np.float64,
    torch.int8: np.int8,
    torch.uint8: np.uint8,
    torch.int16: np.int16,
    torch.uint16: np.uint16,
    torch.int32: np.int32,
    torch.uint32: np.uint32,
    torch.int64: np.int64,
    torch.uint64: np.uint64,
    torch.bool: np.bool_,
}


def _copy_space(space: Any) -> Any:
    """Deep-copy a supported Gym observation space at an authority seam."""

    return copy.deepcopy(space)


def _immutable_array(value: np.ndarray) -> np.ndarray:
    """Return an array backed by immutable bytes for private descriptors."""

    contiguous = np.ascontiguousarray(value)
    return cast(
        np.ndarray,
        np.frombuffer(contiguous.tobytes(), dtype=contiguous.dtype).reshape(
            contiguous.shape
        ),
    )


def _obs_key(category: str, key: str) -> str:
    return f"{category}__{key}"


_FLOAT_DTYPES = {torch.float16, torch.bfloat16, torch.float32, torch.float64}
# torch cannot cleanly promote these unsigned types with signed/float dtypes,
# so a mixed vector containing them is widened to float64.
_UNPROMOTABLE_UINT_DTYPES = {torch.uint32, torch.uint64}
# Integer dtypes whose range exceeds float32's 24-bit mantissa: mixing them with
# a float term and letting torch promote to float32 would silently round them.
_FLOAT32_UNSAFE_INT_DTYPES = {torch.int32, torch.int64}


def resolve_vector_dtype(vector_terms: list[torch.Tensor]) -> torch.dtype:
    """Promote vector terms without losing integer precision.

    For mixed terms, widen to ``float64`` when unsigned 32/64-bit types cannot
    promote or ``float32`` would round an ``int32``/``int64`` term. Otherwise
    use native promotion; narrow integers remain exact in ``float32``.
    """
    dtypes = {term.dtype for term in vector_terms}
    if len(dtypes) <= 1:
        return next(iter(dtypes), torch.float32)
    if any(dtype in _UNPROMOTABLE_UINT_DTYPES for dtype in dtypes):
        return torch.float64
    try:
        promoted = functools.reduce(torch.promote_types, dtypes)
    except TypeError:
        return torch.float64
    if (
        promoted in _FLOAT_DTYPES
        and promoted != torch.float64
        and any(dtype in _FLOAT32_UNSAFE_INT_DTYPES for dtype in dtypes)
    ):
        return torch.float64
    return promoted


def _cat_vector_terms(vector_terms: list[torch.Tensor]) -> torch.Tensor:
    target_dtype = resolve_vector_dtype(vector_terms)
    return torch.cat(
        [term.to(dtype=target_dtype) for term in vector_terms],
        dim=-1,
    )


def _box_bounds(dtype: np.dtype) -> tuple[float | int | bool, float | int | bool]:
    if np.issubdtype(dtype, np.bool_):
        return 0, 1
    if np.issubdtype(dtype, np.integer):
        info = np.iinfo(dtype)
        return info.min, info.max
    if np.issubdtype(dtype, np.floating):
        return -np.inf, np.inf
    raise TypeError(f"Unsupported numpy dtype {dtype}")


def _gym_box_dtype(
    dtype: np.dtype[Any],
) -> type[np.floating[Any]] | type[np.integer[Any]]:
    return cast(type[np.floating[Any]] | type[np.integer[Any]], dtype.type)


def box_for_dtype_shape(
    *,
    key: str,
    dtype: torch.dtype | np.dtype,
    shape: tuple[int, ...],
) -> gymnasium.spaces.Box:
    """Box space for a tensor key. Single source of truth for dtype mapping."""
    if isinstance(dtype, torch.dtype):
        numpy_type = _TORCH_TO_NUMPY.get(dtype)
        if numpy_type is None:
            raise TypeError(f"Observation '{key}' has unsupported dtype {dtype}")
        numpy_dtype = np.dtype(numpy_type)
    else:
        numpy_dtype = np.dtype(dtype)
    low, high = _box_bounds(numpy_dtype)
    return gymnasium.spaces.Box(
        low=low,
        high=high,
        shape=shape,
        dtype=_gym_box_dtype(numpy_dtype),
    )


@dataclass(frozen=True)
class ObservationTerm:
    category: str
    key: str
    output_key: str
    shape: tuple[int, ...]
    dtype: torch.dtype
    vector: bool
    #: Packed ``vector`` slice; ``None`` for non-vector terms.
    vector_slice: tuple[int, int] | None = None


ObservationExtraTerm = tuple[str, torch.dtype | np.dtype[Any], tuple[int, ...]]


@dataclass(frozen=True)
class NumpyObservationTerm:
    category: str
    key: str
    source_key: str
    output_key: str
    shape: tuple[int, ...]
    dtype: np.dtype
    vector: bool
    _space: gymnasium.spaces.Box
    #: Packed vector slice; ``None`` for non-vector terms.
    vector_slice: tuple[int, int] | None = None

    @property
    def space(self) -> gymnasium.spaces.Box:
        return cast(gymnasium.spaces.Box, _copy_space(self._space))


@dataclass(frozen=True)
class ObservationLayout(ImmutableValueMixin):
    num_envs: int
    terms: tuple[ObservationTerm, ...]
    vector_shape: tuple[int, ...] | None
    vector_dtype: torch.dtype | None

    @staticmethod
    def from_spaces(
        spaces: gymnasium.spaces.Dict,
        *,
        output_space: gymnasium.Space[Any] | None = None,
        exclude_categories: Sequence[str] = (),
    ) -> ObservationSpaceLayout:
        """Build the external-space backend through the canonical facade."""
        return NumpyObservationLayout.from_spaces(
            spaces,
            output_space=output_space,
            exclude_categories=exclude_categories,
        )

    @classmethod
    def from_step_return(
        cls,
        step_return: StepReturn,
        *,
        num_envs: int,
    ) -> ObservationLayout:
        return cls.from_observations(step_return.observation, num_envs=num_envs)

    @classmethod
    def from_observations(
        cls,
        observations: Mapping[str, Mapping[str, torch.Tensor]],
        *,
        num_envs: int | None = None,
        validate_dtypes: bool = True,
    ) -> ObservationLayout:
        if not isinstance(observations, Mapping):
            raise TypeError(
                "observations must be a mapping of categories to mappings, "
                f"got {type(observations).__name__}"
            )
        if num_envs is not None:
            num_envs = validate_positive_int(num_envs, name="num_envs")

        terms: list[ObservationTerm] = []
        vector_terms: list[torch.Tensor] = []
        vector_width = 0

        for category, group in observations.items():
            if not isinstance(group, Mapping):
                raise TypeError(
                    f"Observation category '{category}' must be a mapping, "
                    f"got {type(group).__name__}"
                )
            for key in sorted(group):
                value = group[key]
                if not isinstance(value, torch.Tensor):
                    raise TypeError(
                        f"Observation '{_obs_key(category, key)}' must be a torch "
                        f"tensor, got {type(value).__name__}"
                    )
                output_key = _obs_key(category, key)
                if value.ndim == 0:
                    raise ValueError(
                        f"Observation '{output_key}' must be at least 1-D, got scalar"
                    )
                if num_envs is None:
                    num_envs = int(value.shape[0])
                elif value.shape[0] != num_envs:
                    raise ValueError(
                        f"Observation '{output_key}' first dimension must be "
                        f"{num_envs}, got {tuple(value.shape)}"
                    )
                if validate_dtypes and value.dtype not in _TORCH_TO_NUMPY:
                    raise TypeError(
                        f"Observation '{output_key}' has unsupported dtype "
                        f"{value.dtype}"
                    )

                # Critic-only terms must not shift actor-visible vector inputs.
                vector = (
                    value.ndim <= 2 and category != ObservationGroup.STATE_PRIVILEGED
                )
                vector_slice: tuple[int, int] | None = None
                if vector:
                    flattened = value.reshape(value.shape[0], -1)
                    vector_terms.append(flattened)
                    vector_slice = (
                        vector_width,
                        vector_width + int(flattened.shape[1]),
                    )
                    vector_width += int(flattened.shape[1])
                terms.append(
                    ObservationTerm(
                        category=category,
                        key=key,
                        output_key=output_key,
                        shape=tuple(value.shape[1:]),
                        dtype=value.dtype,
                        vector=vector,
                        vector_slice=vector_slice,
                    )
                )

        if num_envs is None:
            raise ValueError("Observation layout requires at least one observation")

        vector_shape: tuple[int, ...] | None = None
        vector_dtype: torch.dtype | None = None
        if vector_terms:
            vector_shape = (vector_width,)
            vector_dtype = resolve_vector_dtype(vector_terms)
            if validate_dtypes and vector_dtype not in _TORCH_TO_NUMPY:
                raise TypeError(
                    f"Observation 'vector' has unsupported dtype {vector_dtype}"
                )

        return cls(
            num_envs=num_envs,
            terms=tuple(terms),
            vector_shape=vector_shape,
            vector_dtype=vector_dtype,
        )

    def flatten(
        self,
        observations: Mapping[str, Mapping[str, torch.Tensor]],
    ) -> dict[str, torch.Tensor]:
        self._validate_schema(observations)

        flat: dict[str, torch.Tensor] = {}
        vector_terms: list[torch.Tensor] = []
        for term in self.terms:
            value = observations[term.category][term.key]
            if term.vector:
                vector_terms.append(value.reshape(value.shape[0], -1))
            else:
                flat[term.output_key] = value.clone()

        if vector_terms:
            flat["vector"] = _cat_vector_terms(vector_terms)
        return flat

    def build_spaces(
        self,
        *,
        extra_terms: Sequence[ObservationExtraTerm] = (),
    ) -> tuple[gymnasium.spaces.Dict, gymnasium.Space[Any]]:
        spaces: dict[str, gymnasium.Space[Any]] = {}

        for term in self.terms:
            if term.vector:
                continue
            spaces[term.output_key] = self._box_for(
                key=term.output_key,
                dtype=term.dtype,
                shape=term.shape,
            )

        if self.vector_shape is not None:
            if self.vector_dtype is None:
                raise ValueError("Observation vector shape has no vector dtype")
            spaces["vector"] = self._box_for(
                key="vector",
                dtype=self.vector_dtype,
                shape=self.vector_shape,
            )

        for output_key, dtype, shape in extra_terms:
            if output_key in spaces:
                raise ValueError(
                    f"Observation space term '{output_key}' is already declared"
                )
            spaces[output_key] = box_for_dtype_shape(
                key=output_key,
                dtype=dtype,
                shape=tuple(shape),
            )

        single = gymnasium.spaces.Dict(spaces)
        return single, batch_space(single, n=self.num_envs)

    def _validate_schema(
        self,
        observations: Mapping[str, Mapping[str, torch.Tensor]],
    ) -> None:
        if not isinstance(observations, Mapping):
            raise TypeError(
                f"observations must be a mapping, got {type(observations).__name__}"
            )
        for category, group in observations.items():
            if not isinstance(group, Mapping):
                raise TypeError(
                    f"Observation category '{category}' must be a mapping, "
                    f"got {type(group).__name__}"
                )
        expected = {(term.category, term.key): term for term in self.terms}
        actual = {
            (category, key)
            for category, group in observations.items()
            for key in group.keys()
        }

        missing = sorted(set(expected) - actual)
        if missing:
            category, key = missing[0]
            raise ValueError(f"Observation schema missing '{_obs_key(category, key)}'")

        unexpected = sorted(actual - set(expected))
        if unexpected:
            category, key = unexpected[0]
            raise ValueError(
                f"Observation schema has unexpected '{_obs_key(category, key)}'"
            )

        for term in self.terms:
            value = observations[term.category][term.key]
            if not isinstance(value, torch.Tensor):
                raise TypeError(
                    f"Observation '{term.output_key}' must be a torch tensor, "
                    f"got {type(value).__name__}"
                )
            if value.ndim == 0:
                raise ValueError(
                    f"Observation '{term.output_key}' must be at least 1-D, got scalar"
                )
            if value.shape[0] != self.num_envs or tuple(value.shape[1:]) != term.shape:
                raise ValueError(
                    f"Observation '{term.output_key}' shape must be "
                    f"{(self.num_envs, *term.shape)}, got {tuple(value.shape)}"
                )
            if value.dtype != term.dtype:
                raise TypeError(
                    f"Observation '{term.output_key}' dtype must be {term.dtype}, "
                    f"got {value.dtype}"
                )

    def structure(
        self,
        flat: Mapping[str, torch.Tensor],
    ) -> dict[str, dict[str, torch.Tensor]]:
        """Validate flat outputs and rebuild the detailed Torch observation."""
        if not isinstance(flat, Mapping):
            raise TypeError(
                f"flat observations must be a mapping, got {type(flat).__name__}"
            )
        expected = {
            term.output_key if not term.vector else "vector" for term in self.terms
        }
        actual = set(flat)
        if actual != expected:
            missing = sorted(expected - actual)
            extra = sorted(actual - expected)
            details: list[str] = []
            if missing:
                details.append(f"missing={missing}")
            if extra:
                details.append(f"extra={extra}")
            raise ValueError(
                "flat observation keys do not match layout: " + ", ".join(details)
            )

        detailed: dict[str, dict[str, torch.Tensor]] = {}
        for term in self.terms:
            output_key = "vector" if term.vector else term.output_key
            value = flat[output_key]
            if not isinstance(value, torch.Tensor):
                raise TypeError(
                    f"Observation '{output_key}' must be a torch tensor, "
                    f"got {type(value).__name__}"
                )
            expected_shape = (self.num_envs, *term.shape)
            if term.vector:
                if term.vector_slice is None:
                    raise ValueError(
                        f"Observation '{term.output_key}' has no vector slice"
                    )
                start, stop = term.vector_slice
                expected_shape = (
                    (self.num_envs, self.vector_shape[0])
                    if self.vector_shape
                    else (self.num_envs,)
                )
                if tuple(value.shape) != expected_shape:
                    raise ValueError(
                        f"Observation '{output_key}' shape must be {expected_shape}, "
                        f"got {tuple(value.shape)}"
                    )
                value = value[:, start:stop].reshape((self.num_envs, *term.shape))
                expected_dtype = self.vector_dtype
            else:
                if tuple(value.shape) != expected_shape:
                    raise ValueError(
                        f"Observation '{output_key}' shape must be {expected_shape}, "
                        f"got {tuple(value.shape)}"
                    )
                expected_dtype = term.dtype
            if expected_dtype is None or value.dtype != expected_dtype:
                raise TypeError(
                    f"Observation '{output_key}' dtype must be {expected_dtype}, "
                    f"got {value.dtype}"
                )
            detailed.setdefault(term.category, {})[term.key] = value
        return detailed

    _box_for = staticmethod(box_for_dtype_shape)


@dataclass(frozen=True)
class NumpyObservationLayout(ImmutableValueMixin):
    terms: tuple[NumpyObservationTerm, ...]
    _vector_shapes: Mapping[str, tuple[int, ...]]
    _vector_dtypes: Mapping[str, np.dtype]
    _vector_bounds: Mapping[str, tuple[np.ndarray, np.ndarray]]

    @classmethod
    def from_spaces(
        cls,
        spaces: gymnasium.spaces.Dict,
        *,
        output_space: gymnasium.Space[Any] | None = None,
        exclude_categories: Sequence[str] = (),
    ) -> NumpyObservationLayout:
        if not isinstance(spaces, gymnasium.spaces.Dict):
            raise TypeError(
                f"Numpy observation layout requires a Dict space, got {spaces}"
            )
        spaces = cast(gymnasium.spaces.Dict, _copy_space(spaces))

        excluded_categories = set(exclude_categories)
        term_specs: list[tuple[str, str, str, gymnasium.spaces.Box, bool]] = []
        vector_widths: dict[str, int] = {}
        has_raw_categories = any(
            isinstance(space, gymnasium.spaces.Dict) for space in spaces.spaces.values()
        )
        for category, space in spaces.spaces.items():
            if has_raw_categories and _is_flat_output_artifact(category, space):
                continue
            if category in excluded_categories:
                continue
            if isinstance(space, gymnasium.spaces.Dict):
                for key, term_space in sorted(space.spaces.items()):
                    if not isinstance(term_space, gymnasium.spaces.Box):
                        raise TypeError(
                            f"Observation '{_obs_key(category, key)}' must use "
                            f"a Box space, got {term_space}"
                        )
                    source_key = f"{category}/{key}"
                    vector = len(term_space.shape) <= 1
                    if vector:
                        vector_widths[category] = vector_widths.get(category, 0) + int(
                            np.prod(term_space.shape, dtype=np.int64)
                        )
                    term_specs.append((category, key, source_key, term_space, vector))
            elif isinstance(space, gymnasium.spaces.Box):
                term_specs.append(("", category, category, space, False))
            else:
                raise TypeError(
                    f"Observation '{category}' expected Box or nested Dict, got {space}"
                )

        category_vector_outputs = _category_vector_outputs(
            output_space=output_space,
            vector_widths=vector_widths,
        )

        terms: list[NumpyObservationTerm] = []
        vector_parts: dict[str, list[np.dtype]] = {}
        vector_lows: dict[str, list[np.ndarray]] = {}
        vector_highs: dict[str, list[np.ndarray]] = {}
        vector_offsets: dict[str, int] = {}
        for category, key, source_key, space, vector in term_specs:
            dtype = np.dtype(space.dtype)
            vector_slice: tuple[int, int] | None = None
            if vector:
                output_key = category_vector_outputs.get(category, "vector")
                vector_parts.setdefault(output_key, []).append(dtype)
                vector_lows.setdefault(output_key, []).append(
                    np.asarray(space.low, dtype=dtype).reshape(-1)
                )
                vector_highs.setdefault(output_key, []).append(
                    np.asarray(space.high, dtype=dtype).reshape(-1)
                )
                width = int(np.prod(space.shape, dtype=np.int64)) if space.shape else 1
                start = vector_offsets.get(output_key, 0)
                vector_slice = (start, start + width)
                vector_offsets[output_key] = start + width
            else:
                output_key = key if not category else _obs_key(category, key)
            terms.append(
                NumpyObservationTerm(
                    category=category,
                    key=key,
                    source_key=source_key,
                    output_key=output_key,
                    shape=tuple(space.shape),
                    dtype=dtype,
                    vector=vector,
                    _space=space,
                    vector_slice=vector_slice,
                )
            )

        vector_shapes: dict[str, tuple[int, ...]] = {}
        vector_dtypes: dict[str, np.dtype] = {}
        vector_bounds: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for output_key, dtypes in vector_parts.items():
            dtype = np.result_type(*dtypes)
            low = np.concatenate(vector_lows[output_key], axis=0).astype(
                dtype,
                copy=False,
            )
            high = np.concatenate(vector_highs[output_key], axis=0).astype(
                dtype,
                copy=False,
            )
            vector_shapes[output_key] = (int(low.shape[0]),)
            vector_dtypes[output_key] = np.dtype(dtype)
            vector_bounds[output_key] = (low, high)

        immutable_bounds = {
            key: (_immutable_array(low), _immutable_array(high))
            for key, (low, high) in vector_bounds.items()
        }
        return cls(
            terms=tuple(terms),
            _vector_shapes=MappingProxyType(dict(vector_shapes)),
            _vector_dtypes=MappingProxyType(dict(vector_dtypes)),
            _vector_bounds=MappingProxyType(immutable_bounds),
        )

    @property
    def vector_shapes(self) -> dict[str, tuple[int, ...]]:
        return dict(self._vector_shapes)

    @property
    def vector_dtypes(self) -> dict[str, np.dtype]:
        return dict(self._vector_dtypes)

    @property
    def vector_bounds(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        return {
            key: (low.copy(), high.copy())
            for key, (low, high) in self._vector_bounds.items()
        }

    def source_spaces(self) -> tuple[tuple[str, gymnasium.spaces.Box], ...]:
        return tuple((term.source_key, term.space) for term in self.terms)

    def flat_space_mapping(self) -> dict[str, gymnasium.spaces.Box]:
        """Return detailed leaf spaces keyed by their slash-delimited paths."""
        return {term.source_key: term.space for term in self.terms}

    def merged_space(self) -> gymnasium.spaces.Box:
        """Build one bounded Box by concatenating every declared leaf."""
        if not self.terms:
            return gymnasium.spaces.Box(
                low=np.zeros((0,), dtype=np.float32),
                high=np.zeros((0,), dtype=np.float32),
                shape=(0,),
                dtype=np.float32,
            )
        dtypes = [term.dtype for term in self.terms]
        dtype = np.dtype(np.result_type(*dtypes))
        low = np.concatenate(
            [
                np.asarray(term._space.low, dtype=dtype).reshape(-1)
                for term in self.terms
            ]
        )
        high = np.concatenate(
            [
                np.asarray(term._space.high, dtype=dtype).reshape(-1)
                for term in self.terms
            ]
        )
        return gymnasium.spaces.Box(
            low=low,
            high=high,
            dtype=_gym_box_dtype(dtype),
        )

    def flatten_paths(
        self,
        observations: Mapping[str, object],
        *,
        batched: bool | None = None,
    ) -> dict[str, object]:
        """Validate and expose detailed leaves without packing vector terms."""
        self._validate_detailed_schema(observations, validate_values=False)
        inferred_batched = batched
        batch_size: int | None = None
        for term in self.terms:
            value = self._get_any_value(observations, term)
            shape = tuple(getattr(value, "shape", ()))
            if inferred_batched is None:
                inferred_batched = len(shape) == len(term.shape) + 1
            if inferred_batched:
                if len(shape) != len(term.shape) + 1 or shape[1:] != term.shape:
                    raise ValueError(
                        f"Observation '{term.source_key}' shape must be "
                        f"(batch, {term.shape}), got {shape}"
                    )
                current_batch = int(shape[0])
                if batch_size is None:
                    batch_size = current_batch
                elif current_batch != batch_size:
                    raise ValueError(
                        f"Observation '{term.source_key}' batch size must be "
                        f"{batch_size}, got {current_batch}"
                    )
            elif shape != term.shape:
                raise ValueError(
                    f"Observation '{term.source_key}' shape must be {term.shape}, "
                    f"got {shape}"
                )
            if isinstance(value, np.ndarray):
                if np.dtype(value.dtype) != term.dtype:
                    raise TypeError(
                        f"Observation '{term.source_key}' dtype must be {term.dtype}, "
                        f"got {value.dtype}"
                    )
            elif isinstance(value, torch.Tensor):
                expected_dtype = _torch_dtype_for_numpy(term.dtype)
                if value.dtype != expected_dtype:
                    raise TypeError(
                        f"Observation '{term.source_key}' dtype must be "
                        f"{expected_dtype}, got {value.dtype}"
                    )
            else:
                raise TypeError(
                    f"Observation '{term.source_key}' must be a numpy array or "
                    f"torch tensor, got {type(value).__name__}"
                )
        return {
            term.source_key: self._get_any_value(observations, term)
            for term in self.terms
        }

    def to_numpy(
        self,
        observations: Mapping[str, object],
        *,
        batched: bool | None = None,
        squeeze_single_batch: bool = False,
    ) -> dict[str, dict[str, np.ndarray] | np.ndarray]:
        """Convert observations to NumPy, preserving categories and layout spaces."""
        self._validate_detailed_schema(observations, validate_values=False)
        converted: dict[str, dict[str, np.ndarray] | np.ndarray] = {}
        for term in self.terms:
            value = self._get_any_value(observations, term)
            if isinstance(value, torch.Tensor):
                array = value.detach().cpu().numpy()
            elif isinstance(value, np.ndarray):
                array = value
            else:
                array = np.asarray(value)

            if term.category:
                group = converted.setdefault(term.category, {})
                if not isinstance(group, dict):
                    raise ValueError(
                        f"Observation category '{term.category}' collides with a leaf"
                    )
                group[term.key] = array
            else:
                converted[term.key] = array

        # Validate the converted leaves while they still carry the runtime's
        # batch axis, then apply the rollout-only single-env squeeze.
        self.flatten_paths(converted, batched=batched)
        if squeeze_single_batch:
            for term in self.terms:
                converted_value = self._get_any_value(converted, term)
                if not isinstance(converted_value, np.ndarray):
                    raise TypeError(
                        f"Observation '{term.source_key}' must be a numpy array"
                    )
                if (
                    converted_value.ndim == len(term.shape) + 1
                    and converted_value.shape[0] == 1
                ):
                    squeezed = converted_value.squeeze(0)
                    if term.category:
                        group = converted[term.category]
                        if not isinstance(group, dict):
                            raise ValueError(
                                f"Observation category '{term.category}' "
                                "collides with a leaf"
                            )
                        group[term.key] = squeezed
                    else:
                        converted[term.key] = squeezed
        return converted

    def flatten_torch(
        self,
        observations: Mapping[str, object],
        *,
        batched: bool | None = None,
    ) -> torch.Tensor:
        """Pack detailed observations while keeping data on the Torch device."""
        self._validate_detailed_schema(observations, validate_values=False)
        values: list[torch.Tensor] = []
        inferred_batched: bool | None = batched
        batch_size: int | None = None
        for term in self.terms:
            value = self._get_any_value(observations, term)
            if not isinstance(value, torch.Tensor):
                raise TypeError(
                    f"Observation '{term.source_key}' must be a torch tensor, "
                    f"got {type(value).__name__}"
                )
            expected_dtype = _torch_dtype_for_numpy(term.dtype)
            if value.dtype != expected_dtype:
                raise TypeError(
                    f"Observation '{term.source_key}' dtype must be {expected_dtype}, "
                    f"got {value.dtype}"
                )
            if inferred_batched is None:
                inferred_batched = value.ndim != len(term.shape)
            if inferred_batched:
                if (
                    value.ndim != len(term.shape) + 1
                    or tuple(value.shape[1:]) != term.shape
                ):
                    raise ValueError(
                        f"Observation '{term.source_key}' shape must be "
                        f"(batch, {term.shape}), got {tuple(value.shape)}"
                    )
                current_batch = int(value.shape[0])
                if batch_size is None:
                    batch_size = current_batch
                elif current_batch != batch_size:
                    raise ValueError(
                        f"Observation '{term.source_key}' batch size must be "
                        f"{batch_size}, got {current_batch}"
                    )
                values.append(value.reshape(current_batch, -1))
            else:
                if tuple(value.shape) != term.shape:
                    raise ValueError(
                        f"Observation '{term.source_key}' shape must be {term.shape}, "
                        f"got {tuple(value.shape)}"
                    )
                values.append(value.reshape(-1))
        if not values:
            # Keep empty outputs on the input device for later concatenation.
            def _tensors(node: object) -> Iterator[torch.Tensor]:
                if isinstance(node, torch.Tensor):
                    yield node
                elif isinstance(node, Mapping):
                    for child in node.values():
                        yield from _tensors(child)

            device = next((tensor.device for tensor in _tensors(observations)), None)
            return torch.zeros((0,), dtype=torch.float32, device=device)
        # Match ``merged_space``; Torch would round int32/int64 mixed with float32.
        target_dtype = _torch_dtype_for_numpy(
            np.dtype(np.result_type(*[term.dtype for term in self.terms]))
        )
        values = [value.to(dtype=target_dtype) for value in values]
        return torch.cat(values, dim=-1)

    def structure(
        self,
        flat: Mapping[str, np.ndarray],
    ) -> dict[str, dict[str, np.ndarray] | np.ndarray]:
        """Validate flat NumPy outputs and rebuild the detailed structure."""
        if not isinstance(flat, Mapping):
            raise TypeError(
                f"flat observations must be a mapping, got {type(flat).__name__}"
            )
        expected = {term.output_key for term in self.terms if not term.vector} | set(
            self._vector_shapes
        )
        actual = set(flat)
        if actual != expected:
            missing = sorted(expected - actual)
            extra = sorted(actual - expected)
            details: list[str] = []
            if missing:
                details.append(f"missing={missing}")
            if extra:
                details.append(f"extra={extra}")
            raise ValueError(
                "flat observation keys do not match layout: " + ", ".join(details)
            )

        detailed: dict[str, dict[str, np.ndarray] | np.ndarray] = {}
        for term in self.terms:
            output_key = term.output_key
            value = flat[output_key]
            if not isinstance(value, np.ndarray):
                raise TypeError(
                    f"Observation '{output_key}' must be a numpy array, "
                    f"got {type(value).__name__}"
                )
            if term.vector:
                if term.vector_slice is None:
                    raise ValueError(f"Observation '{output_key}' has no vector slice")
                expected_shape = self._vector_shapes[output_key]
                if tuple(value.shape) != expected_shape:
                    raise ValueError(
                        f"Observation '{output_key}' shape must be {expected_shape}, "
                        f"got {tuple(value.shape)}"
                    )
                if np.dtype(value.dtype) != self._vector_dtypes[output_key]:
                    raise TypeError(
                        f"Observation '{output_key}' dtype must be "
                        f"{self._vector_dtypes[output_key]}, got {value.dtype}"
                    )
                start, stop = term.vector_slice
                value = value[start:stop].reshape(term.shape)
            else:
                if tuple(value.shape) != term.shape:
                    raise ValueError(
                        f"Observation '{output_key}' shape must be {term.shape}, "
                        f"got {tuple(value.shape)}"
                    )
                if np.dtype(value.dtype) != term.dtype:
                    raise TypeError(
                        f"Observation '{output_key}' dtype must be {term.dtype}, "
                        f"got {value.dtype}"
                    )
            if term.category:
                group = detailed.setdefault(term.category, {})
                if not isinstance(group, dict):
                    raise ValueError(
                        f"Observation category '{term.category}' collides with a leaf"
                    )
                group[term.key] = value
            else:
                detailed[term.key] = value
        return detailed

    def flatten(
        self,
        observations: Mapping[str, object],
    ) -> dict[str, np.ndarray]:
        self._validate_schema(observations)

        flat: dict[str, np.ndarray] = {}
        vector_terms: dict[str, list[np.ndarray]] = {}
        for term in self.terms:
            value = self._get_value(observations, term)
            if term.vector:
                vector_terms.setdefault(term.output_key, []).append(value.reshape(-1))
            else:
                flat[term.output_key] = value

        for output_key, values in vector_terms.items():
            flat[output_key] = np.concatenate(values, axis=0).astype(
                self._vector_dtypes[output_key],
                copy=False,
            )
        return flat

    def build_spaces(self) -> gymnasium.spaces.Dict:
        spaces: dict[str, gymnasium.Space[Any]] = {}

        for term in self.terms:
            if term.vector:
                continue
            spaces[term.output_key] = term.space

        for output_key, shape in self._vector_shapes.items():
            low, high = self._vector_bounds[output_key]
            spaces[output_key] = gymnasium.spaces.Box(
                low=low.copy(),
                high=high.copy(),
                shape=shape,
                dtype=_gym_box_dtype(self._vector_dtypes[output_key]),
            )

        return gymnasium.spaces.Dict(spaces)

    def _validate_detailed_schema(
        self,
        observations: Mapping[str, object],
        *,
        validate_values: bool = True,
    ) -> None:
        if not isinstance(observations, Mapping):
            raise TypeError(
                f"observations must be a mapping, got {type(observations).__name__}"
            )
        expected_groups = {term.category for term in self.terms if term.category}
        expected_leaves = {term.key for term in self.terms if not term.category}
        actual_groups = {
            key for key, value in observations.items() if isinstance(value, Mapping)
        }
        actual_leaves = set(observations) - actual_groups
        if actual_groups != expected_groups or actual_leaves != expected_leaves:
            missing = sorted((expected_groups | expected_leaves) - set(observations))
            extra = sorted(set(observations) - (expected_groups | expected_leaves))
            details: list[str] = []
            if missing:
                details.append(f"missing={missing}")
            if extra:
                details.append(f"extra={extra}")
            raise ValueError(
                "observation schema does not match layout: " + ", ".join(details)
            )
        for category in expected_groups:
            group = observations[category]
            if not isinstance(group, Mapping):
                raise TypeError(
                    f"Observation category '{category}' must be a mapping, "
                    f"got {type(group).__name__}"
                )
            expected_keys = {
                term.key for term in self.terms if term.category == category
            }
            actual_keys = set(group)
            if actual_keys != expected_keys:
                missing = sorted(expected_keys - actual_keys)
                extra = sorted(actual_keys - expected_keys)
                category_details: list[str] = []
                if missing:
                    category_details.append(f"missing={missing}")
                if extra:
                    category_details.append(f"extra={extra}")
                raise ValueError(
                    f"observation category '{category}' does not match layout: "
                    + ", ".join(category_details)
                )
        if not validate_values:
            return
        for term in self.terms:
            value = self._get_any_value(observations, term)
            if isinstance(value, np.ndarray):
                value_shape = tuple(value.shape)
                value_dtype = np.dtype(value.dtype)
            elif isinstance(value, torch.Tensor):
                value_shape = tuple(value.shape)
                actual_torch_dtype = value.dtype
                expected_torch_dtype = _torch_dtype_for_numpy(term.dtype)
                if actual_torch_dtype != expected_torch_dtype:
                    raise TypeError(
                        f"Observation '{term.source_key}' dtype must be "
                        f"{expected_torch_dtype}, got {actual_torch_dtype}"
                    )
                value_dtype = term.dtype
            else:
                raise TypeError(
                    f"Observation '{term.source_key}' must be a numpy array or "
                    f"torch tensor, got {type(value).__name__}"
                )
            if value_shape != term.shape:
                raise ValueError(
                    f"Observation '{term.source_key}' shape must be {term.shape}, "
                    f"got {value_shape}"
                )
            if value_dtype != term.dtype:
                raise TypeError(
                    f"Observation '{term.source_key}' dtype must be {term.dtype}, "
                    f"got {value_dtype}"
                )

    @staticmethod
    def _get_any_value(
        observations: Mapping[str, object],
        term: NumpyObservationTerm,
    ) -> object:
        if term.category:
            group = observations.get(term.category)
            if not isinstance(group, Mapping) or term.key not in group:
                raise ValueError(f"Observation schema missing '{term.source_key}'")
            return group[term.key]
        if term.key not in observations:
            raise ValueError(f"Observation schema missing '{term.source_key}'")
        return observations[term.key]

    def _validate_schema(self, observations: Mapping[str, object]) -> None:
        self._validate_detailed_schema(observations)
        for term in self.terms:
            value = self._get_value(observations, term)
            if tuple(value.shape) != term.shape:
                raise ValueError(
                    f"Observation '{term.source_key}' shape must be {term.shape}, "
                    f"got {tuple(value.shape)}"
                )
            if np.dtype(value.dtype) != term.dtype:
                raise TypeError(
                    f"Observation '{term.source_key}' dtype must be {term.dtype}, "
                    f"got {value.dtype}"
                )

    @staticmethod
    def _get_value(
        observations: Mapping[str, object],
        term: NumpyObservationTerm,
    ) -> np.ndarray:
        if term.category:
            group = observations.get(term.category)
            if not isinstance(group, Mapping) or term.key not in group:
                raise ValueError(f"Observation schema missing '{term.source_key}'")
            value = group[term.key]
        else:
            if term.key not in observations:
                raise ValueError(f"Observation schema missing '{term.source_key}'")
            value = observations[term.key]
        if not isinstance(value, np.ndarray):
            raise TypeError(
                f"Observation '{term.source_key}' must be a numpy array, "
                f"got {type(value).__name__}"
            )
        return value


ObservationLayoutLike: TypeAlias = ObservationLayout | NumpyObservationLayout
ObservationSpaceLayout: TypeAlias = NumpyObservationLayout


def _is_flat_output_artifact(key: str, space: gymnasium.Space[Any]) -> bool:
    return isinstance(space, gymnasium.spaces.Box) and (key == "vector" or "__" in key)


def _torch_dtype_for_numpy(dtype: np.dtype) -> torch.dtype:
    try:
        return torch.from_numpy(np.empty((), dtype=np.dtype(dtype))).dtype
    except (TypeError, ValueError) as exc:
        raise TypeError(f"numpy dtype {dtype} cannot be represented by Torch") from exc


def _category_vector_outputs(
    *,
    output_space: gymnasium.Space[Any] | None,
    vector_widths: Mapping[str, int],
) -> dict[str, str]:
    if not isinstance(output_space, gymnasium.spaces.Dict):
        return {}
    if "vector" in output_space.spaces:
        return {}

    outputs: dict[str, str] = {}
    for category, width in vector_widths.items():
        space = output_space.spaces.get(category)
        if isinstance(space, gymnasium.spaces.Box) and tuple(space.shape) == (width,):
            outputs[category] = category
    return outputs


@dataclass(frozen=True)
class ProjectionLeaf:
    """One contract leaf routed into the flat vector."""

    path: str
    source_key: str
    source_slice: tuple[int, int] | None
    shape: tuple[int, ...]
    output_slice: tuple[int, int]

    @property
    def size(self) -> int:
        return int(np.prod(self.shape, dtype=np.int64)) if self.shape else 1


@dataclass(frozen=True)
class NumericObservationProjection:
    """Project assembled observations to float32 in contract or space key order.

    NumPy and Torch share leaf slices and bounds; fingerprints bind projections
    to their IO schema and layout.
    """

    #: ``None`` for a projection built by :meth:`from_observation_space`, which
    #: has no frozen IO contract behind it (only the space's own key order).
    contract_id: str | None
    io_schema_fingerprint: str | None
    leaves: tuple[ProjectionLeaf, ...]
    output_space: gymnasium.spaces.Box
    layout_fingerprint: str

    @classmethod
    def from_contract(
        cls,
        contract: Any,
        detailed_space: gymnasium.spaces.Dict,
        assembled_space: gymnasium.spaces.Dict,
        *,
        allow_unbounded_assembled_bounds: bool = False,
    ) -> "NumericObservationProjection":
        """Build from the contract and assembled space, validating leaves and bounds.

        Only SRB simulator layouts may opt in to fully unbounded bounds; they
        carry no packed-order information. Keep the opt-in false when physical
        bounds must be preserved.
        """
        if not isinstance(detailed_space, gymnasium.spaces.Dict):
            raise TypeError(
                "contract projection requires a detailed Dict space, "
                f"got {detailed_space}"
            )
        if not isinstance(assembled_space, gymnasium.spaces.Dict):
            raise TypeError(
                "contract projection requires an assembled Dict space, "
                f"got {assembled_space}"
            )

        contract_leaves = tuple(contract.observation_leaves)
        contract_paths = [leaf.key for leaf in contract_leaves]
        if len(set(contract_paths)) != len(contract_paths):
            raise ValueError(
                "landscaping contract declares duplicate observation leaves"
            )

        # Detailed space must contain exactly the actor-visible contract
        # leaves (privileged groups are explicitly excluded from the actor
        # projection and may not leak in here).
        detailed_paths: dict[str, gymnasium.spaces.Box] = {}
        for category, group in detailed_space.spaces.items():
            if category == ObservationGroup.STATE_PRIVILEGED:
                continue
            if not isinstance(group, gymnasium.spaces.Dict):
                raise TypeError(
                    f"detailed observation group '{category}' must be a Dict "
                    f"space, got {group}"
                )
            for key, space in group.spaces.items():
                if not isinstance(space, gymnasium.spaces.Box):
                    raise TypeError(
                        f"detailed observation '{category}/{key}' must be a "
                        f"Box space, got {space}"
                    )
                detailed_paths[f"{category}/{key}"] = space

        missing = [path for path in contract_paths if path not in detailed_paths]
        if missing:
            raise ValueError(
                f"detailed observation space is missing contract leaves: {missing}"
            )
        unexpected = sorted(set(detailed_paths) - set(contract_paths))
        if unexpected:
            raise ValueError(
                "detailed observation space has unexpected actor-visible "
                f"leaves: {unexpected}"
            )

        # Route contract paths through the existing layout machinery so the
        # assembled packed-vector slices come from one source of truth.
        layout = NumpyObservationLayout.from_spaces(
            detailed_space,
            output_space=assembled_space,
        )
        terms_by_path: dict[str, NumpyObservationTerm] = {}
        for term in layout.terms:
            if term.category == ObservationGroup.STATE_PRIVILEGED:
                continue
            if term.source_key in terms_by_path:
                raise ValueError(
                    f"duplicate observation leaf '{term.source_key}' in layout"
                )
            terms_by_path[term.source_key] = term

        expected_outputs = layout.build_spaces()
        actual_outputs = {
            key: space
            for key, space in assembled_space.spaces.items()
            if key in expected_outputs.spaces
        }
        # Explicitly excluded privileged groups may remain outside the actor
        # projection; anything else actor-visible is fatal.
        privileged_prefix = f"{ObservationGroup.STATE_PRIVILEGED}__"
        assembled_extra = sorted(
            key
            for key in set(assembled_space.spaces) - set(expected_outputs.spaces)
            if not str(key).startswith(privileged_prefix)
        )
        if assembled_extra:
            raise ValueError(
                "assembled observation space has unexpected actor-visible "
                f"outputs: {assembled_extra}"
            )
        for key, expected in expected_outputs.spaces.items():
            actual = actual_outputs.get(key)
            if actual is None:
                raise ValueError(f"assembled observation space is missing '{key}'")
            if not isinstance(actual, gymnasium.spaces.Box):
                raise TypeError(
                    f"assembled observation '{key}' must be a Box space, got {actual}"
                )
            if not isinstance(expected, gymnasium.spaces.Box):
                raise TypeError(
                    f"expected observation '{key}' must be a Box space, got {expected}"
                )
            if tuple(actual.shape) != tuple(expected.shape) or np.dtype(
                actual.dtype
            ) != np.dtype(expected.dtype):
                raise ValueError(
                    f"assembled observation '{key}' must be "
                    f"{tuple(expected.shape)} {np.dtype(expected.dtype)}, got "
                    f"{tuple(actual.shape)} {np.dtype(actual.dtype)}"
                )
            # Bound equality detects packed-order drift. Fully unbounded simulator
            # bounds carry no order information and are allowed only by opt-in.
            if not _projection_bounds_compatible(
                actual,
                expected,
                allow_unbounded=allow_unbounded_assembled_bounds,
            ):
                raise ValueError(
                    f"assembled observation '{key}' physical bounds do not "
                    "match the contract layout (possible packed-vector order "
                    "drift)"
                )

        leaves: list[ProjectionLeaf] = []
        offset = 0
        for contract_leaf in contract_leaves:
            path = contract_leaf.key
            matching_term = terms_by_path.get(path)
            if matching_term is None:
                raise ValueError(f"observation layout is missing '{path}'")
            expected_shape = tuple(int(dim) for dim in contract_leaf.shape)
            if tuple(matching_term.shape) != expected_shape:
                raise ValueError(
                    f"observation '{path}' shape must be {expected_shape}, "
                    f"got {tuple(matching_term.shape)}"
                )
            if np.dtype(matching_term.dtype) != np.dtype(np.float32):
                raise TypeError(
                    f"observation '{path}' dtype must be float32, got "
                    f"{matching_term.dtype}"
                )
            size = int(np.prod(expected_shape, dtype=np.int64))
            leaves.append(
                ProjectionLeaf(
                    path=path,
                    source_key=matching_term.output_key,
                    source_slice=matching_term.vector_slice,
                    shape=expected_shape,
                    output_slice=(offset, offset + size),
                )
            )
            offset += size

        flat_size = int(contract.flat_observation_size)
        if offset != flat_size:
            raise ValueError(
                f"contract projection covers {offset} values, expected {flat_size}"
            )

        low, high = contract.flattened_bounds()
        output_space = gymnasium.spaces.Box(
            low=np.asarray(low, dtype=np.float32).copy(),
            high=np.asarray(high, dtype=np.float32).copy(),
            shape=(flat_size,),
            dtype=np.float32,
        )

        io_schema_fingerprint = str(contract.fingerprint())
        layout_fingerprint = _projection_layout_fingerprint(
            contract_id=str(contract.contract_id),
            io_schema_fingerprint=io_schema_fingerprint,
            leaves=leaves,
            flat_size=flat_size,
        )
        return cls(
            contract_id=str(contract.contract_id),
            io_schema_fingerprint=io_schema_fingerprint,
            leaves=tuple(leaves),
            output_space=output_space,
            layout_fingerprint=layout_fingerprint,
        )

    @classmethod
    def from_observation_space(
        cls,
        assembled_space: gymnasium.spaces.Dict,
    ) -> "NumericObservationProjection":
        """Project each Box leaf whole, in space key order, without a contract.

        The output preserves physical bounds; pass a space with privileged
        leaves already excluded. Contract metadata remains ``None``.
        """
        if not isinstance(assembled_space, gymnasium.spaces.Dict):
            raise TypeError(
                "observation-space projection requires a Dict space, "
                f"got {assembled_space}"
            )
        if not assembled_space.spaces:
            raise ValueError("observation space contains no leaves to project")

        leaves: list[ProjectionLeaf] = []
        lows: list[np.ndarray] = []
        highs: list[np.ndarray] = []
        offset = 0
        for key, space in assembled_space.spaces.items():
            if not isinstance(space, gymnasium.spaces.Box):
                raise TypeError(f"observation '{key}' must be a Box space, got {space}")
            name = str(key)
            shape = tuple(int(dim) for dim in space.shape)
            size = int(np.prod(shape, dtype=np.int64)) if shape else 1
            leaves.append(
                ProjectionLeaf(
                    path=name,
                    source_key=name,
                    source_slice=None,
                    shape=shape,
                    output_slice=(offset, offset + size),
                )
            )
            lows.append(np.asarray(space.low, dtype=np.float32).reshape(-1))
            highs.append(np.asarray(space.high, dtype=np.float32).reshape(-1))
            offset += size

        output_space = gymnasium.spaces.Box(
            low=np.concatenate(lows, axis=0),
            high=np.concatenate(highs, axis=0),
            shape=(offset,),
            dtype=np.float32,
        )
        return cls(
            contract_id=None,
            io_schema_fingerprint=None,
            leaves=tuple(leaves),
            output_space=output_space,
            layout_fingerprint=_projection_layout_fingerprint(
                contract_id=None,
                io_schema_fingerprint=None,
                leaves=leaves,
                flat_size=offset,
            ),
        )

    @property
    def output_size(self) -> int:
        return int(self.output_space.shape[0])

    @property
    def source_keys(self) -> tuple[str, ...]:
        seen: dict[str, None] = {}
        for leaf in self.leaves:
            seen.setdefault(leaf.source_key, None)
        return tuple(seen)

    def verify_fingerprint(self, expected: str) -> None:
        """Fail loudly when the IO-schema fingerprint drifted from *expected*."""
        if str(expected) != self.io_schema_fingerprint:
            raise ValueError(
                "landscaping IO-schema fingerprint mismatch: projection has "
                f"{self.io_schema_fingerprint}, expected {expected}"
            )

    def project_numpy(self, observation: Mapping[str, Any]) -> np.ndarray:
        """Project an assembled NumPy observation to the flat contract vector."""
        sources = self._validated_sources(
            observation,
            is_backend=lambda value: isinstance(value, np.ndarray),
            backend_name="numpy array",
        )
        batch_shape = self._batch_shape(
            {key: tuple(value.shape) for key, value in sources.items()}
        )
        out: np.ndarray = np.empty((*batch_shape, self.output_size), dtype=np.float32)
        # Check per leaf for precise attribution; NumPy checks need no sync.
        for leaf in self.leaves:
            value = sources[leaf.source_key]
            if leaf.source_slice is not None:
                value = value[..., leaf.source_slice[0] : leaf.source_slice[1]]
            flat = np.asarray(value, dtype=np.float32).reshape(
                (*batch_shape, leaf.size)
            )
            if not np.isfinite(flat).all():
                raise ValueError(
                    f"observation '{leaf.path}' contains non-finite values"
                )
            out[..., leaf.output_slice[0] : leaf.output_slice[1]] = flat
        return out

    def project_torch(self, observation: Mapping[str, Any]) -> torch.Tensor:
        """Project an assembled Torch observation, preserving backend/device."""
        sources = self._validated_sources(
            observation,
            is_backend=lambda value: isinstance(value, torch.Tensor),
            backend_name="torch tensor",
        )
        batch_shape = self._batch_shape(
            {key: tuple(value.shape) for key, value in sources.items()}
        )
        device = next(iter(sources.values())).device
        out = torch.empty(
            (*batch_shape, self.output_size), dtype=torch.float32, device=device
        )
        for leaf in self.leaves:
            value = sources[leaf.source_key]
            if value.device != device:
                raise ValueError(
                    f"observation '{leaf.path}' is on device {value.device}, "
                    f"expected {device}"
                )
            if leaf.source_slice is not None:
                value = value[..., leaf.source_slice[0] : leaf.source_slice[1]]
            flat = value.to(dtype=torch.float32).reshape((*batch_shape, leaf.size))
            out[..., leaf.output_slice[0] : leaf.output_slice[1]] = flat
        # Check once to avoid per-leaf device synchronization; rescan only on failure.
        if not torch.isfinite(out).all():
            raise ValueError(
                f"observation '{self._first_non_finite_leaf(out)}' contains "
                "non-finite values"
            )
        return out

    def _first_non_finite_leaf(self, out: torch.Tensor) -> str:
        """Contract path of the first leaf holding a non-finite value."""
        for leaf in self.leaves:
            block = out[..., leaf.output_slice[0] : leaf.output_slice[1]]
            if not torch.isfinite(block).all():
                return leaf.path
        return "<unknown>"  # pragma: no cover - only reached if `out` changed

    def _validated_sources(
        self,
        observation: Mapping[str, Any],
        *,
        is_backend: Any,
        backend_name: str,
    ) -> dict[str, Any]:
        if not isinstance(observation, Mapping):
            raise TypeError(
                f"contract projection requires a Mapping observation, "
                f"got {type(observation).__name__}"
            )
        expected = set(self.source_keys)
        missing = sorted(expected - set(observation))
        if missing:
            raise ValueError(f"observation is missing assembled keys: {missing}")
        # Allow critic-only groups alongside actor-visible projection inputs.
        privileged_prefix = f"{ObservationGroup.STATE_PRIVILEGED}__"
        unexpected = sorted(
            key
            for key in set(observation) - expected
            if not str(key).startswith(privileged_prefix)
        )
        if unexpected:
            raise ValueError(
                f"observation has unexpected actor-visible keys: {unexpected}"
            )
        sources: dict[str, Any] = {}
        for key in self.source_keys:
            value = observation[key]
            if not is_backend(value):
                raise TypeError(
                    f"observation '{key}' must be a {backend_name}, "
                    f"got {type(value).__name__}"
                )
            if not _is_float_coercible_dtype(value.dtype):
                raise TypeError(
                    f"observation '{key}' dtype {value.dtype} is not coercible "
                    "to float32"
                )
            sources[key] = value
        return sources

    def _batch_shape(
        self, source_shapes: Mapping[str, tuple[int, ...]]
    ) -> tuple[int, ...]:
        expected_trailing = self._expected_trailing_shapes()
        batch_shape: tuple[int, ...] | None = None
        for key, shape in source_shapes.items():
            trailing = expected_trailing[key]
            if not trailing:
                # Scalar leaf: every dimension present is batch.
                batch_shape = tuple(shape) if batch_shape is None else batch_shape
                if tuple(shape) != batch_shape:
                    raise ValueError(
                        f"observation '{key}' batch shape {tuple(shape)} does "
                        f"not match {batch_shape}"
                    )
                continue
            if len(shape) < len(trailing) or tuple(shape[-len(trailing) :]) != (
                trailing
            ):
                raise ValueError(
                    f"observation '{key}' shape must end with {trailing}, got {shape}"
                )
            leading = tuple(shape[: len(shape) - len(trailing)])
            if batch_shape is None:
                batch_shape = leading
            elif leading != batch_shape:
                raise ValueError(
                    f"observation '{key}' batch shape {leading} does not match "
                    f"{batch_shape}"
                )
        if batch_shape is None:
            raise ValueError("observation projection requires at least one source leaf")
        return batch_shape

    def _expected_trailing_shapes(self) -> dict[str, tuple[int, ...]]:
        trailing: dict[str, tuple[int, ...]] = {}
        for leaf in self.leaves:
            if leaf.source_slice is None:
                trailing[leaf.source_key] = leaf.shape
            else:
                width = max(
                    stop
                    for other in self.leaves
                    if other.source_key == leaf.source_key
                    and other.source_slice is not None
                    for stop in (other.source_slice[1],)
                )
                trailing[leaf.source_key] = (width,)
        return trailing


def _projection_bounds_compatible(
    actual: gymnasium.spaces.Box,
    expected: gymnasium.spaces.Box,
    *,
    allow_unbounded: bool,
) -> bool:
    actual_low = np.asarray(actual.low, dtype=np.float32)
    actual_high = np.asarray(actual.high, dtype=np.float32)
    if np.array_equal(actual_low, np.asarray(expected.low, dtype=np.float32)) and (
        np.array_equal(actual_high, np.asarray(expected.high, dtype=np.float32))
    ):
        return True
    if not allow_unbounded:
        return False
    # Fully unbounded (dtype-extrema) runtime bounds carry no information;
    # anything partially finite that disagrees is treated as order drift.
    return not np.isfinite(actual_low).any() and not np.isfinite(actual_high).any()


def _is_float_coercible_dtype(dtype: Any) -> bool:
    if isinstance(dtype, torch.dtype):
        return dtype in _FLOAT_DTYPES
    try:
        return bool(np.issubdtype(np.dtype(dtype), np.floating))
    except TypeError:
        return False


def _projection_layout_fingerprint(
    *,
    contract_id: str | None,
    io_schema_fingerprint: str | None,
    leaves: Sequence[ProjectionLeaf],
    flat_size: int,
) -> str:
    import hashlib
    import json

    payload = {
        "contract_id": contract_id,
        "io_schema_fingerprint": io_schema_fingerprint,
        "dtype": "float32",
        "flat_size": flat_size,
        "leaves": [
            {
                "path": leaf.path,
                "source_key": leaf.source_key,
                "source_slice": list(leaf.source_slice)
                if leaf.source_slice is not None
                else None,
                "shape": list(leaf.shape),
                "output_slice": list(leaf.output_slice),
            }
            for leaf in leaves
        ],
    }
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def contract_detailed_dict_space(contract: Any) -> gymnasium.spaces.Dict:
    """Group contract leaf spaces, preserving contract category order.

    Tuple sequences preserve insertion order. Fully unbounded simulator bounds
    carry no information for checking packed order.
    """
    grouped: dict[str, list[tuple[str, gymnasium.spaces.Box]]] = {}
    for path, space in contract.to_gym_observation_spaces():
        category, _, key = str(path).partition("/")
        grouped.setdefault(category, []).append((key, space))
    return gymnasium.spaces.Dict(
        [
            (category, gymnasium.spaces.Dict(items))
            for category, items in grouped.items()
        ]
    )


def flatten_observations(
    observations: Mapping[str, Mapping[str, torch.Tensor]],
) -> dict[str, torch.Tensor]:
    if not any(observations.values()):
        return {}
    return ObservationLayout.from_observations(
        observations,
        validate_dtypes=False,
    ).flatten(observations)


def build_observation_space_layout(
    observation_space: gymnasium.Space[Any],
) -> tuple[ObservationSpaceLayout, bool]:
    """Build a layout and flag whether the input was a bare root space.

    Non-Dict spaces are nested under ``ROOT_OBSERVATION_KEY`` for frameworks
    requiring Dict observations; callers use the flag to unwrap outputs.
    """
    observation_is_root = not isinstance(observation_space, gymnasium.spaces.Dict)
    detailed_space = (
        gymnasium.spaces.Dict(
            [(ROOT_OBSERVATION_KEY, copy.deepcopy(observation_space))]
        )
        if observation_is_root
        else observation_space
    )
    return (
        ObservationLayout.from_spaces(cast(gymnasium.spaces.Dict, detailed_space)),
        observation_is_root,
    )


__all__ = [
    "NumericObservationProjection",
    "NumpyObservationLayout",
    "NumpyObservationTerm",
    "ObservationLayout",
    "ObservationLayoutLike",
    "ObservationSpaceLayout",
    "ObservationTerm",
    "ProjectionLeaf",
    "box_for_dtype_shape",
    "build_observation_space_layout",
    "contract_detailed_dict_space",
    "flatten_observations",
]
