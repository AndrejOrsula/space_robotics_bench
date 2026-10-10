"""Project SRB observations and actions at the Dreamer learner boundary.

Observation output is CPU NumPy; action coercion returns the Torch structure
expected by SRB. This module does not depend on Embodied, JAX, Isaac, or the
Dreamer process transport.
"""

from __future__ import annotations

import hashlib
import json
import time
from collections import OrderedDict
from collections.abc import Mapping
from copy import deepcopy
from types import MappingProxyType
from typing import Any

import gymnasium
import numpy as np
import torch

from srb.core.env.runtime.action_layout import ActionLayout, GenericActionLayout
from srb.core.env.runtime.observation_layout import (
    NumericObservationProjection,
    NumpyObservationLayout,
    ObservationLayout,
)

STATE_KEY = "state"
DEFAULT_ACTION_KEY = "action"


class DreamerLearnerProjectionError(ValueError):
    """Raised when the learner projection contract cannot be established."""


class DreamerLearnerProjection:
    """Immutable, batch-aware learner projection.

    Batch methods require exactly ``num_envs`` rows. Single-env observation
    projection accepts a row or one-row batch without squeezing image axes;
    action coercion returns structured Torch data. Row pack/unpack helpers use
    NumPy only.
    """

    __slots__ = (
        "_action_key",
        "_action_layout",
        "_action_space",
        "_action_spaces",
        "_device",
        "_io_schema_fingerprint",
        "_is_action_dict",
        "_is_observation_dict",
        "_layout_fingerprint",
        "_mode",
        "_num_envs",
        "_observation_key",
        "_observation_layout",
        "_observation_projection",
        "_observation_space",
        "_observation_spaces",
        "_raw_action_space",
        "_raw_observation_space",
        "_raw_observation_spaces",
        "_sealed",
    )

    def __init__(
        self,
        env: Any | None = None,
        *,
        single_observation_space: gymnasium.Space[Any] | None = None,
        single_action_space: gymnasium.Space[Any] | None = None,
        num_envs: int | None = None,
        device: Any = None,
        obs_key: str | None = None,
        act_key: str = DEFAULT_ACTION_KEY,
        mode: str | None = None,
        metadata: Mapping[str, Any] | None = None,
        introspection: Mapping[str, Any] | Any | None = None,
        unwrapped: Any | None = None,
        allow_unbounded_assembled_bounds: bool | None = None,
    ) -> None:
        """Build from an SRB-like env or explicit spaces and metadata.

        The explicit form supports child processes. Pass either ``env`` or
        single spaces; ``num_envs`` is fixed for all batch methods.
        """

        if env is not None and (
            single_observation_space is not None or single_action_space is not None
        ):
            raise DreamerLearnerProjectionError(
                "provide either env or explicit single observation/action spaces"
            )

        merged_metadata = _merge_metadata(metadata, introspection)
        source = env if env is not None else introspection

        if env is not None:
            if unwrapped is None:
                unwrapped = getattr(env, "unwrapped", env)
            if single_observation_space is None:
                single_observation_space = _first_present(
                    env, "single_observation_space", "observation_space"
                )
            if single_action_space is None:
                single_action_space = _first_present(
                    env, "single_action_space", "action_space"
                )
            if num_envs is None:
                num_envs = merged_metadata.get("num_envs", getattr(env, "num_envs", 1))
            if device is None:
                device = merged_metadata.get("device", getattr(env, "device", None))

        if unwrapped is None and source is not None and not isinstance(source, Mapping):
            unwrapped = source

        if mode is None:
            mode = merged_metadata.get("mode", merged_metadata.get("observation_mode"))
        if mode is None and env is not None:
            mode = getattr(env, "observation_mode", getattr(env, "obs_mode", None))

        if obs_key is None:
            obs_key = merged_metadata.get(
                "obs_key", merged_metadata.get("observation_key")
            )
        if obs_key is None and env is not None:
            obs_key = getattr(env, "obs_key", getattr(env, "_obs_key", None))

        if act_key == DEFAULT_ACTION_KEY:
            act_key = str(
                merged_metadata.get(
                    "act_key", merged_metadata.get("action_key", act_key)
                )
            )
            if env is not None:
                act_key = str(
                    getattr(env, "act_key", getattr(env, "_act_key", act_key))
                )

        if num_envs is None:
            num_envs = merged_metadata.get("num_envs", 1)
        if obs_key is None:
            obs_key = _default_observation_key(single_observation_space, mode)

        try:
            built = _build_projection(
                single_observation_space=single_observation_space,
                single_action_space=single_action_space,
                num_envs=num_envs,
                device=device,
                observation_key=obs_key,
                action_key=act_key,
                mode=mode,
                unwrapped=unwrapped,
                metadata=merged_metadata,
                allow_unbounded_assembled_bounds=allow_unbounded_assembled_bounds,
            )
        except DreamerLearnerProjectionError:
            raise
        except Exception as exc:
            raise DreamerLearnerProjectionError(
                f"invalid Dreamer learner projection: {exc}"
            ) from exc

        for name, value in built.items():
            object.__setattr__(self, name, value)
        object.__setattr__(self, "_sealed", True)

    @classmethod
    def from_spaces(
        cls,
        observation_space: gymnasium.Space[Any] | None = None,
        action_space: gymnasium.Space[Any] | None = None,
        *,
        single_observation_space: gymnasium.Space[Any] | None = None,
        single_action_space: gymnasium.Space[Any] | None = None,
        **kwargs: Any,
    ) -> "DreamerLearnerProjection":
        """Construct from explicit single spaces and minimal metadata."""

        if observation_space is not None and single_observation_space is not None:
            raise DreamerLearnerProjectionError(
                "observation_space and single_observation_space are aliases; "
                "provide only one"
            )
        if action_space is not None and single_action_space is not None:
            raise DreamerLearnerProjectionError(
                "action_space and single_action_space are aliases; provide only one"
            )
        return cls(
            single_observation_space=(
                observation_space
                if observation_space is not None
                else single_observation_space
            ),
            single_action_space=(
                action_space if action_space is not None else single_action_space
            ),
            **kwargs,
        )

    @property
    def num_envs(self) -> int:
        return self._num_envs

    @property
    def device(self) -> Any:
        return self._device

    @property
    def observation_key(self) -> str:
        return self._observation_key

    @property
    def action_key(self) -> str:
        return self._action_key

    @property
    def mode(self) -> str | None:
        return self._mode

    @property
    def is_action_dict(self) -> bool:
        return self._is_action_dict

    @property
    def observation_space(self) -> gymnasium.Space[Any]:
        """Fresh root Gymnasium space for DreamerProcessProjection."""

        return deepcopy(self._observation_space)

    @property
    def action_space(self) -> gymnasium.Space[Any]:
        """Fresh root Gymnasium space for DreamerProcessProjection."""

        return deepcopy(self._action_space)

    @property
    def observation_spaces(self) -> dict[str, gymnasium.Space[Any]]:
        return _copy_space_mapping(self._observation_spaces)

    @property
    def action_spaces(self) -> dict[str, gymnasium.Space[Any]]:
        return _copy_space_mapping(self._action_spaces)

    @property
    def obs_space(self) -> dict[str, gymnasium.Space[Any]]:
        """Ordered mapping equivalent to the wrapper's learner data fields."""

        return self.observation_spaces

    @property
    def act_space(self) -> dict[str, gymnasium.Space[Any]]:
        """Ordered mapping equivalent to the wrapper, without ``reset``."""

        return self.action_spaces

    @property
    def raw_observation_space(self) -> gymnasium.Space[Any]:
        return deepcopy(self._raw_observation_space)

    @property
    def raw_action_space(self) -> gymnasium.Space[Any]:
        return deepcopy(self._raw_action_space)

    @property
    def layout_fingerprint(self) -> str:
        return self._layout_fingerprint

    @property
    def io_schema_fingerprint(self) -> str | None:
        return self._io_schema_fingerprint

    @property
    def observation_projection(self) -> NumericObservationProjection | None:
        """Defensive copy for contract inspection; layouts remain private."""

        if self._observation_projection is None:
            return None
        return deepcopy(self._observation_projection)

    def project_observation(self, raw: Any) -> OrderedDict[str, np.ndarray]:
        """Preserve the one-env row-or-batch compatibility entry point."""

        if self._num_envs != 1:
            return self.project_observation_batch(raw)
        try:
            flat = self._flatten_raw_observation(raw)
            row_flags: list[bool] = []
            batch_flags: list[bool] = []
            for key, value in flat.items():
                shape = tuple(getattr(value, "shape", ()))
                expected, _dtype = _space_shape_dtype(self._raw_observation_spaces[key])
                row_flags.append(shape == expected)
                batch_flags.append(shape == (1, *expected))
            if all(batch_flags):
                return self.project_observation_batch(raw)
            if all(row_flags):
                return self.pack_observation_row(self.project_observation_row(raw))
            raise ValueError(
                "one-env observation leaves must be consistently unbatched "
                "or have one leading batch row"
            )
        except DreamerLearnerProjectionError:
            raise
        except Exception as exc:
            raise DreamerLearnerProjectionError(
                f"invalid Dreamer observation: {exc}"
            ) from exc

    def project_observation_batch(self, raw: Any) -> OrderedDict[str, np.ndarray]:
        """Project exactly ``num_envs`` raw rows to CPU NumPy learner data."""

        projected, _readback_s, _projection_s = (
            self.project_observation_batch_with_timings(raw)
        )
        return projected

    def project_observation_batch_with_timings(
        self, raw: Any
    ) -> tuple[OrderedDict[str, np.ndarray], float, float]:
        """Project a batch and report host-readback and projection durations."""

        readback_start_s = time.perf_counter()
        try:
            flat = self._flatten_raw_observation(raw, batched=True)
            packed = OrderedDict(
                (
                    key,
                    _batch_leaf(
                        value,
                        self._raw_observation_spaces[key],
                        num_envs=self._num_envs,
                        name=f"observation '{key}'",
                    ),
                )
                for key, value in flat.items()
            )
            projection_start_s = time.perf_counter()
            projected = self._project_flat_observation_batch(packed)
            end_s = time.perf_counter()
            return (
                projected,
                projection_start_s - readback_start_s,
                end_s - projection_start_s,
            )
        except DreamerLearnerProjectionError:
            raise
        except Exception as exc:
            raise DreamerLearnerProjectionError(
                f"invalid Dreamer observation batch: {exc}"
            ) from exc

    def project_observation_row(self, raw: Any) -> OrderedDict[str, np.ndarray]:
        """Project one child-process row through the batch authority."""

        if self._num_envs != 1:
            raise DreamerLearnerProjectionError(
                "row projection requires a projection configured with num_envs=1"
            )
        try:
            self._flatten_raw_observation(raw, batched=False)
            batched_raw = _add_batch_axis(raw, name="raw observation")
            return self.unpack_observation_row(
                self.project_observation_batch(batched_raw)
            )
        except DreamerLearnerProjectionError:
            raise
        except Exception as exc:
            raise DreamerLearnerProjectionError(
                f"invalid Dreamer observation row: {exc}"
            ) from exc

    def pack_observation_row(
        self, row: Mapping[str, Any]
    ) -> OrderedDict[str, np.ndarray]:
        """Pack an already learner-facing observation row for a worker."""

        return _pack_mapping_row(
            row,
            self._observation_spaces,
            name="learner observation",
        )

    def unpack_observation_row(
        self, batched: Mapping[str, Any]
    ) -> OrderedDict[str, np.ndarray]:
        """Unpack exactly one leading row axis, preserving all other axes."""

        return _unpack_mapping_row(
            batched,
            self._observation_spaces,
            name="learner observation",
        )

    def coerce_action(
        self, learner_action: Mapping[str, Any], *, device: Any = None
    ) -> Any:
        """Preserve the existing action entry point through batch coercion."""

        return self.coerce_action_batch(learner_action, device=device)

    def coerce_action_row(
        self, learner_action: Mapping[str, Any], *, device: Any = None
    ) -> Any:
        """Coerce one child-process action through the batch authority."""

        if self._num_envs != 1:
            raise DreamerLearnerProjectionError(
                "row action coercion requires a projection configured with num_envs=1"
            )
        return self.coerce_action_batch(learner_action, device=device)

    def coerce_action_batch(
        self, learner_action: Mapping[str, Any], *, device: Any = None
    ) -> Any:
        """Rebuild and coerce exactly ``num_envs`` learner action rows."""

        try:
            if not isinstance(learner_action, Mapping):
                raise TypeError("learner action must be a mapping of flat learner keys")
            _validate_mapping_keys(
                learner_action,
                self._action_spaces,
                name="learner action",
            )
            if self._is_action_dict:
                structured = self._action_layout.unflatten_path_mapping(learner_action)
            else:
                structured = learner_action[self._action_key]
            return self._action_layout.coerce_batched(
                structured,
                num_envs=self._num_envs,
                device=self._device if device is None else device,
                strict_domain=True,
            )
        except DreamerLearnerProjectionError:
            raise
        except Exception as exc:
            raise DreamerLearnerProjectionError(
                f"invalid Dreamer action: {exc}"
            ) from exc

    def __setattr__(self, name: str, value: Any) -> None:
        if getattr(self, "_sealed", False):
            raise AttributeError("DreamerLearnerProjection is immutable")
        object.__setattr__(self, name, value)

    def __delattr__(self, name: str) -> None:
        raise AttributeError("DreamerLearnerProjection is immutable")

    def _flatten_raw_observation(
        self, raw: Any, *, batched: bool | None = None
    ) -> Mapping[str, Any]:
        if self._observation_layout is not None:
            if not isinstance(raw, Mapping):
                raise TypeError("dict observation space requires a mapping observation")
            flattened = self._observation_layout.flatten_paths(raw, batched=batched)
            _validate_mapping_keys(
                flattened,
                self._raw_observation_spaces,
                name="raw observation",
            )
            return flattened

        value = raw
        if isinstance(raw, Mapping):
            _validate_mapping_keys(
                raw, {self._observation_key: None}, name="raw observation"
            )
            value = raw[self._observation_key]
        return OrderedDict(((self._observation_key, value),))

    def _project_flat_observation_batch(
        self, packed: OrderedDict[str, np.ndarray]
    ) -> OrderedDict[str, np.ndarray]:
        if self._observation_projection is not None:
            state = self._observation_projection.project_numpy(packed)
            return OrderedDict(((STATE_KEY, np.asarray(state).copy()),))
        if self._mode == "state":
            return OrderedDict(
                (
                    key,
                    np.asarray(value)
                    .reshape((self._num_envs, *self._observation_spaces[key].shape))
                    .copy(),
                )
                for key, value in packed.items()
            )
        return packed


def _build_projection(
    *,
    single_observation_space: gymnasium.Space[Any] | None,
    single_action_space: gymnasium.Space[Any] | None,
    num_envs: Any,
    device: Any,
    observation_key: Any,
    action_key: Any,
    mode: Any,
    unwrapped: Any,
    metadata: Mapping[str, Any],
    allow_unbounded_assembled_bounds: bool | None,
) -> dict[str, Any]:
    if isinstance(num_envs, bool) or not isinstance(num_envs, (int, np.integer)):
        raise TypeError(f"num_envs must be an integer, got {num_envs!r}")
    if int(num_envs) < 1:
        raise ValueError(
            f"Dreamer learner projection requires num_envs>=1, got {num_envs}"
        )
    if not isinstance(observation_key, str) or not observation_key:
        raise TypeError("observation key must be a non-empty string")
    if not isinstance(action_key, str) or not action_key:
        raise TypeError("action key must be a non-empty string")
    if mode is not None and mode not in {"state", "visual"}:
        raise ValueError(f"unsupported Dreamer observation mode {mode!r}")
    if single_observation_space is None or single_action_space is None:
        raise TypeError("single observation and action spaces are required")
    if not isinstance(single_observation_space, gymnasium.Space):
        raise TypeError(
            "single observation space must be a Gymnasium space, "
            f"got {type(single_observation_space).__name__}"
        )
    if not isinstance(single_action_space, gymnasium.Space):
        raise TypeError(
            "single action space must be a Gymnasium space, "
            f"got {type(single_action_space).__name__}"
        )

    raw_observation_space = deepcopy(single_observation_space)
    raw_action_space = deepcopy(single_action_space)

    observation_layout: NumpyObservationLayout | None
    if isinstance(raw_observation_space, gymnasium.spaces.Dict):
        observation_layout = ObservationLayout.from_spaces(raw_observation_space)
        raw_observation_spaces = OrderedDict(
            (str(key), deepcopy(space))
            for key, space in observation_layout.flat_space_mapping().items()
        )
        if not raw_observation_spaces:
            raise ValueError("observation Dict has no learner leaves")
    elif isinstance(
        raw_observation_space, (gymnasium.spaces.Box, gymnasium.spaces.Discrete)
    ):
        observation_layout = None
        raw_observation_spaces = OrderedDict(
            ((observation_key, deepcopy(raw_observation_space)),)
        )
    else:
        raise TypeError(
            "unsupported observation space; expected Box, Discrete, or Dict of Boxes"
        )

    is_action_dict = isinstance(raw_action_space, gymnasium.spaces.Dict)
    action_layout = ActionLayout.from_structured_space(raw_action_space)
    if not isinstance(action_layout, GenericActionLayout):
        raise TypeError("structured action layout did not use GenericActionLayout")
    if is_action_dict:
        action_spaces = OrderedDict(
            (str(key), deepcopy(space))
            for key, space in action_layout.flat_space_mapping().items()
        )
        if not action_spaces:
            raise ValueError("action Dict has no learner leaves")
    else:
        action_spaces = OrderedDict(((action_key, deepcopy(raw_action_space)),))

    observation_projection = _resolve_landscaping_projection(
        unwrapped,
        raw_observation_space,
        metadata=metadata,
        allow_unbounded_assembled_bounds=allow_unbounded_assembled_bounds,
    )
    if observation_projection is not None:
        learner_observation_spaces = OrderedDict(
            ((STATE_KEY, deepcopy(observation_projection.output_space)),)
        )
        layout_fingerprint = observation_projection.layout_fingerprint
        io_schema_fingerprint = observation_projection.io_schema_fingerprint
    elif mode == "state":
        learner_observation_spaces = OrderedDict(
            (key, _state_vector_space(space))
            for key, space in raw_observation_spaces.items()
        )
        layout_fingerprint = _mapping_fingerprint(
            learner_observation_spaces, action_spaces
        )
        io_schema_fingerprint = None
    else:
        learner_observation_spaces = OrderedDict(
            (key, deepcopy(space)) for key, space in raw_observation_spaces.items()
        )
        layout_fingerprint = _mapping_fingerprint(
            learner_observation_spaces, action_spaces
        )
        io_schema_fingerprint = None

    return {
        "_action_key": action_key,
        "_action_layout": action_layout,
        "_action_space": deepcopy(_dict_space(action_spaces)),
        "_action_spaces": MappingProxyType(dict(action_spaces)),
        "_device": device,
        "_io_schema_fingerprint": io_schema_fingerprint,
        "_is_action_dict": is_action_dict,
        "_is_observation_dict": observation_layout is not None,
        "_layout_fingerprint": layout_fingerprint,
        "_mode": mode,
        "_num_envs": int(num_envs),
        "_observation_key": observation_key,
        "_observation_layout": observation_layout,
        "_observation_projection": observation_projection,
        "_observation_space": deepcopy(_dict_space(learner_observation_spaces)),
        "_observation_spaces": MappingProxyType(dict(learner_observation_spaces)),
        "_raw_action_space": raw_action_space,
        "_raw_observation_space": raw_observation_space,
        "_raw_observation_spaces": MappingProxyType(dict(raw_observation_spaces)),
    }


def _resolve_landscaping_projection(
    unwrapped: Any,
    assembled_space: Any,
    *,
    metadata: Mapping[str, Any],
    allow_unbounded_assembled_bounds: bool | None,
) -> NumericObservationProjection | None:
    """Copy the wrapper's contract-driven landscaping selection rules."""

    metadata_fn = getattr(unwrapped, "landscaping_contract_metadata", None)
    declared_fingerprint = getattr(unwrapped, "IO_SCHEMA_FINGERPRINT", None)
    explicit_metadata = metadata.get("landscaping_contract_metadata")

    if not callable(metadata_fn) and not declared_fingerprint:
        if callable(explicit_metadata):
            metadata_fn = explicit_metadata
        elif isinstance(explicit_metadata, Mapping):

            def metadata_provider() -> Mapping[str, Any]:
                return explicit_metadata

            metadata_fn = metadata_provider
        elif metadata.get("io_schema_fingerprint"):
            explicit_fingerprint = str(metadata["io_schema_fingerprint"])
            declared_fingerprint = explicit_fingerprint

    if not callable(metadata_fn) and not declared_fingerprint:
        return None

    from srb.core.env.runtime.observation_layout import contract_detailed_dict_space
    from srb.terrain.landscaping_contract import LANDSCAPING_IO_CONTRACT

    contract = LANDSCAPING_IO_CONTRACT
    if callable(metadata_fn):
        published = str(metadata_fn().get("io_schema_fingerprint"))
        allow_unbounded = True
    else:
        published = str(declared_fingerprint)
        allow_unbounded = False
        if published != contract.fingerprint():
            return None
        if allow_unbounded_assembled_bounds is not None:
            allow_unbounded = bool(allow_unbounded_assembled_bounds)
        elif metadata.get("allow_unbounded_assembled_bounds") is not None:
            allow_unbounded = bool(metadata["allow_unbounded_assembled_bounds"])

    projection = NumericObservationProjection.from_contract(
        contract,
        contract_detailed_dict_space(contract),
        assembled_space,
        allow_unbounded_assembled_bounds=allow_unbounded,
    )
    projection.verify_fingerprint(published)
    return projection


def _first_present(obj: Any, *names: str) -> Any:
    for name in names:
        value = getattr(obj, name, None)
        if value is not None:
            return value
    raise DreamerLearnerProjectionError(
        f"SRB-like environment has none of {', '.join(names)}"
    )


def _merge_metadata(
    metadata: Mapping[str, Any] | None,
    introspection: Mapping[str, Any] | Any | None,
) -> dict[str, Any]:
    merged: dict[str, Any] = {}
    if metadata is not None:
        if not isinstance(metadata, Mapping):
            raise DreamerLearnerProjectionError("metadata must be a mapping")
        merged.update(metadata)
    if isinstance(introspection, Mapping):
        merged.update(introspection)
    elif introspection is not None:
        for name in (
            "num_envs",
            "device",
            "mode",
            "observation_mode",
            "obs_key",
            "observation_key",
            "act_key",
            "action_key",
            "io_schema_fingerprint",
            "allow_unbounded_assembled_bounds",
        ):
            value = getattr(introspection, name, None)
            if value is not None:
                merged[name] = value
    return merged


def _default_observation_key(space: Any, mode: Any) -> str:
    if mode == "visual":
        return "image"
    if mode == "state":
        return "state"
    if isinstance(space, gymnasium.spaces.Box) and len(space.shape) >= 2:
        return "image"
    return "state"


def _dict_space(spaces: Mapping[str, gymnasium.Space[Any]]) -> gymnasium.spaces.Dict:
    if not spaces:
        raise ValueError("learner space mapping must not be empty")
    return gymnasium.spaces.Dict(
        OrderedDict((str(key), deepcopy(space)) for key, space in spaces.items())
    )


def _copy_space_mapping(
    spaces: Mapping[str, gymnasium.Space[Any]],
) -> dict[str, gymnasium.Space[Any]]:
    return OrderedDict((key, deepcopy(space)) for key, space in spaces.items())


def _validate_mapping_keys(
    value: Mapping[str, Any],
    expected: Mapping[str, Any],
    *,
    name: str,
) -> None:
    actual_keys = set(value)
    expected_keys = set(expected)
    if actual_keys != expected_keys:
        missing = sorted(str(key) for key in expected_keys - actual_keys)
        extra = sorted(str(key) for key in actual_keys - expected_keys)
        details: list[str] = []
        if missing:
            details.append(f"missing={missing}")
        if extra:
            details.append(f"extra={extra}")
        raise ValueError(f"{name} keys do not match: {', '.join(details)}")


def _host_array(value: Any, *, name: str) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if isinstance(value, torch.Tensor):
        try:
            array = value.detach().cpu().numpy()
        except (RuntimeError, TypeError) as exc:
            raise TypeError(f"{name} tensor is not host-readable") from exc
        if not isinstance(array, np.ndarray):
            raise TypeError(f"{name} did not convert to a NumPy array")
        return array
    raise TypeError(f"{name} must be a NumPy array or Torch tensor")


def _space_shape_dtype(space: gymnasium.Space[Any]) -> tuple[tuple[int, ...], np.dtype]:
    if not hasattr(space, "shape") or not hasattr(space, "dtype"):
        raise TypeError(f"space {space} has no shape/dtype contract")
    return tuple(int(dim) for dim in space.shape), np.dtype(space.dtype)


def _state_vector_space(space: gymnasium.Space[Any]) -> gymnasium.Space[Any]:
    """Expose state Boxes as vectors so Dreamer never classifies them as images."""

    if not isinstance(space, gymnasium.spaces.Box):
        return deepcopy(space)
    return gymnasium.spaces.Box(
        low=np.asarray(space.low).reshape(-1),
        high=np.asarray(space.high).reshape(-1),
        dtype=space.dtype,
    )


def _pack_leaf(value: Any, space: gymnasium.Space[Any], *, name: str) -> np.ndarray:
    array = _host_array(value, name=name)
    expected_shape, expected_dtype = _space_shape_dtype(space)
    if np.dtype(array.dtype) != expected_dtype:
        raise TypeError(f"{name} dtype must be {expected_dtype}, got {array.dtype}")
    batched_shape = (1, *expected_shape)
    if tuple(array.shape) == expected_shape:
        return array.reshape(batched_shape).copy()
    if tuple(array.shape) == batched_shape:
        return array.copy()
    raise ValueError(
        f"{name} shape must be {expected_shape} or {batched_shape}, "
        f"got {tuple(array.shape)}"
    )


def _batch_leaf(
    value: Any,
    space: gymnasium.Space[Any],
    *,
    num_envs: int,
    name: str,
) -> np.ndarray:
    array = _host_array(value, name=name)
    expected_shape, expected_dtype = _space_shape_dtype(space)
    if np.dtype(array.dtype) != expected_dtype:
        raise TypeError(f"{name} dtype must be {expected_dtype}, got {array.dtype}")
    expected_batched_shape = (num_envs, *expected_shape)
    if tuple(array.shape) != expected_batched_shape:
        raise ValueError(
            f"{name} shape must be {expected_batched_shape}, got {tuple(array.shape)}"
        )
    return array.copy()


def _add_batch_axis(value: Any, *, name: str) -> Any:
    if isinstance(value, Mapping):
        return {
            key: _add_batch_axis(child, name=f"{name}.{key}")
            for key, child in value.items()
        }
    if isinstance(value, np.ndarray):
        return np.expand_dims(value, axis=0)
    if isinstance(value, torch.Tensor):
        return value.unsqueeze(0)
    raise TypeError(
        f"{name} must contain only mappings, NumPy arrays, or Torch tensors"
    )


def _pack_mapping_row(
    row: Mapping[str, Any],
    spaces: Mapping[str, gymnasium.Space[Any]],
    *,
    name: str,
) -> OrderedDict[str, np.ndarray]:
    if not isinstance(row, Mapping):
        raise TypeError(f"{name} must be a mapping")
    _validate_mapping_keys(row, spaces, name=name)
    return OrderedDict(
        (
            key,
            _pack_leaf(value, spaces[key], name=f"{name} '{key}'"),
        )
        for key, value in row.items()
    )


def _unpack_mapping_row(
    batched: Mapping[str, Any],
    spaces: Mapping[str, gymnasium.Space[Any]],
    *,
    name: str,
) -> OrderedDict[str, np.ndarray]:
    if not isinstance(batched, Mapping):
        raise TypeError(f"{name} must be a mapping")
    _validate_mapping_keys(batched, spaces, name=name)
    result: OrderedDict[str, np.ndarray] = OrderedDict()
    for key, value in batched.items():
        array = _host_array(value, name=f"{name} '{key}'")
        expected_shape, expected_dtype = _space_shape_dtype(spaces[key])
        if np.dtype(array.dtype) != expected_dtype:
            raise TypeError(
                f"{name} '{key}' dtype must be {expected_dtype}, got {array.dtype}"
            )
        expected_batched_shape = (1, *expected_shape)
        if tuple(array.shape) != expected_batched_shape:
            raise ValueError(
                f"{name} '{key}' shape must be {expected_batched_shape}, "
                f"got {tuple(array.shape)}"
            )
        result[key] = array[0].copy()
    return result


def _mapping_fingerprint(
    observations: Mapping[str, gymnasium.Space[Any]],
    actions: Mapping[str, gymnasium.Space[Any]],
) -> str:
    payload = {
        "observations": [
            [key, _space_signature(space)] for key, space in observations.items()
        ],
        "actions": [[key, _space_signature(space)] for key, space in actions.items()],
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _space_signature(space: gymnasium.Space[Any]) -> Any:
    if isinstance(space, gymnasium.spaces.Box):
        return {
            "type": "Box",
            "dtype": np.dtype(space.dtype).str,
            "shape": list(space.shape),
            "low": np.asarray(space.low).tolist(),
            "high": np.asarray(space.high).tolist(),
        }
    if isinstance(space, gymnasium.spaces.Discrete):
        return {
            "type": "Discrete",
            "n": int(space.n),
            "start": int(space.start),
            "dtype": np.dtype(space.dtype).str,
        }
    if isinstance(space, gymnasium.spaces.MultiDiscrete):
        return {
            "type": "MultiDiscrete",
            "nvec": np.asarray(space.nvec).tolist(),
            "start": np.asarray(space.start).tolist(),
            "dtype": np.dtype(space.dtype).str,
            "shape": list(space.shape),
        }
    if isinstance(space, gymnasium.spaces.Dict):
        return {
            "type": "Dict",
            "spaces": [
                [str(key), _space_signature(child)]
                for key, child in space.spaces.items()
            ],
        }
    if isinstance(space, gymnasium.spaces.Tuple):
        return {
            "type": "Tuple",
            "spaces": [_space_signature(child) for child in space.spaces],
        }
    return {
        "type": type(space).__name__,
        "dtype": np.dtype(space.dtype).str if hasattr(space, "dtype") else None,
        "shape": list(space.shape) if hasattr(space, "shape") else None,
    }
