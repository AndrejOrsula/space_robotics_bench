# SPDX-License-Identifier: Apache-2.0 OR MIT
"""Environment wrapper for robomimic compatibility."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, cast

import gymnasium
import numpy as np
import torch

from srb.core.env.introspect import EnvIntrospector
from srb.core.env.runtime import (
    ActionLayout,
    ObservationLayout,
    ObservationSpaceLayout,
)
from srb.core.spaces import ROOT_OBSERVATION_KEY, coerce_gym_step_outputs
from srb.utils.dataset_schema import DEMO_IO_CONTRACT_VERSION
from srb.utils.failures import UnsupportedCapabilityError

__all__ = ["RobomimicEnvWrapper"]

# Re-export the shared observation key under this module's private name.
_ROOT_OBSERVATION_KEY = ROOT_OBSERVATION_KEY


def _require_single_environment(num_envs: int) -> None:
    if num_envs == 1:
        return
    raise UnsupportedCapabilityError(
        code="integration.robomimic.vector_env.unsupported",
        message=(
            "Robomimic integration supports exactly one environment; "
            f"got num_envs={num_envs}"
        ),
        component="integration.robomimic",
        details={"num_envs": num_envs},
        fallback_allowed=False,
        evidence_valid=False,
    )


if TYPE_CHECKING:
    from typing import Sequence

    from srb._typing import AnyEnv


class RobomimicEnvWrapper(gymnasium.Wrapper):
    """Adapt SRB observations and actions to robomimic's rollout interface."""

    def __init__(
        self,
        env: "AnyEnv",
        obs_key: str = "policy",
    ):
        """Initialize the wrapper for the selected observation key."""
        view = EnvIntrospector.of(env)
        self._env_view = view
        _require_single_environment(view.num_envs)
        super().__init__(env)
        declared_vector = getattr(env, "is_vector_env", None)
        if declared_vector is None:
            getter = getattr(env, "get_wrapper_attr", None)
            if callable(getter):
                try:
                    declared_vector = getter("is_vector_env")
                except AttributeError:
                    pass
        self._is_vector_env = declared_vector is not False
        self._obs_key = obs_key
        # Use the effective single-environment spaces.
        self._obs_space = view.single_observation_space  # type: ignore[attr-defined]
        self._action_space = view.single_action_space  # type: ignore[attr-defined]
        self._action_layout = ActionLayout.from_structured_space(self._action_space)
        self._observation_layout, self._observation_is_root = (
            _derive_observation_layout(self._obs_space, obs_key=self._obs_key)
        )
        # Cache runtime metadata.
        self._num_envs = view.num_envs
        self._device = view.device

    @property
    def num_envs(self) -> int:
        return self._num_envs

    @property
    def device(self) -> torch.device:
        return self._device

    def seed(self, seed: int | None = None) -> Sequence[int | None]:
        # Gymnasium's ``None`` means choose a seed; SRB's seed API uses -1 for it.
        return [self.unwrapped.seed(-1 if seed is None else seed)] * self._num_envs  # type: ignore[return-value]

    def reset(self, **kwargs) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        obs_dict, info = self.env.reset(**kwargs)
        return self._process_obs(obs_dict), info

    @property
    def last_final_observation(self) -> dict[str, Any] | None:
        """Project the ended episode through the same layout as live rows."""
        getter = getattr(self.env, "get_wrapper_attr", None)

        def source_attribute(name: str) -> Any:
            if callable(getter):
                try:
                    return getter(name)
                except AttributeError:
                    return None
            return getattr(self.env, name, None)

        terminal = source_attribute("last_final_observation")
        if terminal is None:
            return None
        ids = source_attribute("last_final_observation_ids")
        if isinstance(ids, torch.Tensor):
            ids = ids.detach().cpu().numpy()
        if ids is None or np.asarray(ids).reshape(-1).tolist() != [0]:
            raise RuntimeError(
                "robomimic terminal observation must identify environment 0"
            )
        return self._process_obs(terminal)

    def step(
        self, action: Any
    ) -> tuple[
        dict[str, np.ndarray], np.ndarray, np.ndarray, np.ndarray, dict[str, Any]
    ]:
        # Coerce flat actions with an env axis; structured layouts batch leaves.
        if not self._is_vector_env:
            action = self._action_layout.coerce(action, device=self._device)
        else:
            action = self._action_layout.coerce(
                action,
                device=self._device,
                num_envs=self._num_envs,
            )

        # Step the environment.
        obs_dict, reward, terminated, truncated, info = self.env.step(action)

        # Convert observations and step outputs.
        obs = self._process_obs(obs_dict)

        reward, terminated, truncated = coerce_gym_step_outputs(
            reward, terminated, truncated, output="numpy"
        )
        if not self._is_vector_env:
            reward, terminated, truncated = (
                _ensure_single_batch_output(value)
                for value in (reward, terminated, truncated)
            )

        return obs, reward, terminated, truncated, info

    def _process_obs(self, obs_dict: dict[str, Any]) -> dict[str, Any]:
        """Convert observations to NumPy and preserve the one-env axis."""
        if isinstance(obs_dict, dict) and self._obs_key in obs_dict:
            obs = obs_dict[self._obs_key]
        else:
            obs = obs_dict

        observation = _convert_observation_to_numpy(
            self._observation_layout,
            observation_is_root=self._observation_is_root,
            observation=obs,
            squeeze_single_batch=False,
            batched=self._is_vector_env,
        )
        return observation if self._is_vector_env else _batch_observation(observation)

    def get_obs_keys(self) -> Sequence[str]:
        """Return the keys emitted by the observation projection."""
        if self._observation_is_root:
            # Preserve robomimic's historical root key.
            return ["obs"]

        # Use emitted tree keys, not packed vector names.
        keys: list[str] = []
        for term in self._observation_layout.terms:
            key = term.category or term.key
            if key not in keys:
                keys.append(key)
        return keys or ["obs"]

    def get_action_dim(self) -> int:
        return self._action_layout.flat_dim

    def get_action_space(self) -> Any:
        return self._action_space

    def get_action_schema(self) -> list[dict[str, Any]]:
        return self._action_layout.schema()

    def get_demo_io_contract(self) -> dict[str, Any]:
        """Describe the observation projection and effective action layout."""
        contract: dict[str, Any] = {
            "version": DEMO_IO_CONTRACT_VERSION,
            "wrapper": "srb.wrappers.robomimic.RobomimicEnvWrapper",
            "observation": _demo_observation_contract(self),
            "action": _demo_action_contract(self),
        }
        env_wrappers = _demo_env_wrappers(self.env)
        if env_wrappers:
            contract["env_wrappers"] = env_wrappers
        return contract


def _demo_env_wrappers(env: Any) -> dict[str, Any]:
    """Return configurations of wrappers that shape recorded data."""
    from srb.wrappers.action_smoothing import ActionSmoothingWrapper
    from srb.wrappers.observation_stacking import ObservationStackingWrapper

    wrappers: dict[str, Any] = {}
    seen: set[int] = set()
    current = env
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        if isinstance(current, ActionSmoothingWrapper):
            record = _wrapper_cfg_record(current.cfg)
            # An unset frequency is inferred from the env's agent rate; record
            # the rate the filter actually ran at.
            record["sample_frequency"] = float(current.sample_rate_hz)
            wrappers.setdefault("action_smoothing", record)
        elif isinstance(current, ObservationStackingWrapper):
            wrappers.setdefault(
                "observation_stacking", _wrapper_cfg_record(current.cfg)
            )
        current = getattr(current, "env", None)
    return wrappers


def _wrapper_cfg_record(cfg: Any) -> dict[str, Any]:
    # The configuration enums carry ``auto()`` values; record their names.
    from enum import Enum

    return {
        key: value.name.lower() if isinstance(value, Enum) else value
        for key, value in cfg.model_dump().items()
    }


def _derive_observation_layout(
    observation_space: gymnasium.Space[Any],
    *,
    obs_key: str,
) -> tuple[ObservationSpaceLayout, bool]:
    selected_space: gymnasium.Space[Any] = observation_space
    if (
        isinstance(observation_space, gymnasium.spaces.Dict)
        and obs_key in observation_space.spaces
    ):
        selected_space = observation_space.spaces[obs_key]

    is_root = not isinstance(selected_space, gymnasium.spaces.Dict)
    detailed_space = (
        gymnasium.spaces.Dict([(_ROOT_OBSERVATION_KEY, selected_space)])
        if is_root
        else selected_space
    )
    return (
        ObservationLayout.from_spaces(cast(gymnasium.spaces.Dict, detailed_space)),
        is_root,
    )


def _convert_observation_to_numpy(
    layout: ObservationSpaceLayout,
    *,
    observation_is_root: bool,
    observation: object,
    squeeze_single_batch: bool,
    batched: bool = True,
) -> dict[str, Any]:
    if observation_is_root:
        detailed: Mapping[str, object] = {_ROOT_OBSERVATION_KEY: observation}
    elif isinstance(observation, Mapping):
        detailed = observation
    else:
        raise TypeError(
            "Robomimic expected a mapping observation for its Dict layout, got "
            f"{type(observation).__name__}"
        )

    converted = layout.to_numpy(
        detailed,
        batched=batched,
        squeeze_single_batch=squeeze_single_batch,
    )
    if not observation_is_root:
        return converted
    root = converted[_ROOT_OBSERVATION_KEY]
    if not isinstance(root, np.ndarray):
        raise TypeError("Robomimic root observation did not convert to a numpy array")
    return {"obs": root}


def _batch_observation(observation: dict[str, Any]) -> dict[str, Any]:
    """Add robomimic's retained one-environment axis to observations."""
    return {
        key: (
            _batch_observation(value)
            if isinstance(value, dict)
            else np.expand_dims(value, 0)
        )
        for key, value in observation.items()
    }


def _ensure_single_batch_output(value: Any) -> np.ndarray:
    """Give scalar step outputs the one-row batch shape used by vector envs."""
    array = np.asarray(value)
    if array.ndim == 0:
        return array.reshape(1)
    if array.shape[0] != 1:
        raise ValueError(
            "scalar robomimic environment returned a step output with leading "
            f"dimension {array.shape[0]}, expected 1"
        )
    return array


def _demo_observation_contract(wrapper: RobomimicEnvWrapper) -> dict[str, Any]:
    selected = (
        isinstance(wrapper._obs_space, gymnasium.spaces.Dict)
        and wrapper._obs_key in wrapper._obs_space.spaces
    )
    keys = list(wrapper.get_obs_keys())
    runtime_descriptors = _runtime_observation_descriptors(wrapper.unwrapped)
    terms: list[dict[str, Any]] = []
    for term in wrapper._observation_layout.terms:
        output_key = "obs" if wrapper._observation_is_root else term.output_key
        term_key = "obs" if wrapper._observation_is_root else term.key
        source_key = "obs" if wrapper._observation_is_root else term.source_key
        emitted_path = (
            ["obs"]
            if wrapper._observation_is_root
            else [term.category, term.key]
            if term.category
            else [term.key]
        )
        entry: dict[str, Any] = {
            "key": term_key,
            "source_key": source_key,
            "output_key": output_key,
            "shape": [int(dim) for dim in term.shape],
            "dtype": str(term.dtype),
            "vector": bool(term.vector),
            "emitted_path": emitted_path,
            # The category is the stable fallback for task-owned observation
            # terms. Runtime sensor descriptors below refine visual and event
            # terms with the physical source and channel names.
            "modality": term.category,
        }
        descriptor = runtime_descriptors.get(term.key) or runtime_descriptors.get(
            term.source_key
        )
        if descriptor is not None:
            entry.update(_descriptor_for_shape(descriptor, entry["shape"]))
        if term.vector_slice is not None:
            entry["vector_slice"] = [
                int(term.vector_slice[0]),
                int(term.vector_slice[1]),
            ]
        terms.append(entry)
    return {
        "wrapper": "srb.wrappers.robomimic.RobomimicEnvWrapper",
        "obs_key": wrapper._obs_key,
        "projection": "select_obs_key_then_observation_layout_to_numpy",
        # Selection is conditional at runtime: ``_process_obs`` selects the
        # named key when present and otherwise passes the root mapping on to
        # the layout.  The path is still declared even when a legacy space
        # omits that key (some SRB envs expose policy only in step returns).
        "projection_path": [wrapper._obs_key],
        "selection": "if_present_else_root",
        "selection_present": selected,
        "root_observation_key": "obs",
        "root": bool(wrapper._observation_is_root),
        "keys": keys,
        "terms": terms,
    }


def _runtime_observation_descriptors(unwrapped: Any) -> dict[str, dict[str, Any]]:
    """Describe sensor-backed observation leaves when metadata is available."""

    assembly = getattr(unwrapped, "_observation_assembly", None)
    if assembly is None:
        return {}

    descriptors: dict[str, dict[str, Any]] = {}
    for visual in getattr(assembly, "visual_terms", ()):
        descriptor: dict[str, Any] = {
            "modality": "visual",
            "sensor_name": str(getattr(visual, "sensor_name", "")),
            "data_types": [str(value) for value in getattr(visual, "data_types", ())],
        }
        # Name each data type's channel slice and its encoding.
        channels = getattr(visual, "channels", ())
        if channels:
            descriptor["channels"] = [
                {
                    "data_type": str(channel.data_type),
                    "start": int(channel.start),
                    "end": int(channel.end),
                    "encoding": str(channel.encoding),
                }
                for channel in channels
            ]
        depth_range = getattr(visual, "depth_range", None)
        if depth_range is not None:
            descriptor["depth_range_m"] = [float(depth_range[0]), float(depth_range[1])]
        labels = getattr(visual, "segmentation_labels", None)
        if labels is not None:
            descriptor["segmentation_labels"] = {
                str(data_type): {str(key): str(value) for key, value in entries.items()}
                for data_type, entries in labels.items()
            }
            descriptor["segmentation_labels_scope"] = "first_observation"
            descriptor["segmentation_labels_truncated"] = bool(
                getattr(visual, "segmentation_labels_truncated", False)
            )
        shape = getattr(visual, "shape", None)
        if shape is not None:
            descriptor[_DESCRIBED_SHAPE] = [int(dim) for dim in shape]
        output_key = str(getattr(visual, "output_key", ""))
        if output_key:
            descriptors[output_key] = descriptor

    components = _vector_components(getattr(assembly, "layout", None))
    if components:
        descriptors["vector"] = {
            "components": components,
            _DESCRIBED_SHAPE: [max(component["end"] for component in components)],
        }

    for event in getattr(assembly, "event_terms", ()):
        output_key = str(getattr(event, "output_key", ""))
        sensor_name = str(getattr(event, "sensor_name", ""))
        if output_key:
            descriptors[output_key] = {
                "modality": "event_camera",
                "sensor_name": sensor_name,
            }
    return descriptors


# Single-step geometry a runtime descriptor was derived from. It is not
# written to the contract; it only gates the geometry fields below.
_DESCRIBED_SHAPE = "_described_shape"
# Fields that index into a term's tensor and are only valid for the exact
# single-step geometry the environment assembled.
_GEOMETRY_FIELDS = (
    "channels",
    "depth_range_m",
    "segmentation_labels",
    "segmentation_labels_scope",
    "segmentation_labels_truncated",
    "components",
)


def _descriptor_for_shape(
    descriptor: dict[str, Any], shape: list[int]
) -> dict[str, Any]:
    """Drop geometry fields when a wrapper changes the emitted shape."""
    described = descriptor.get(_DESCRIBED_SHAPE)
    result = {
        key: value for key, value in descriptor.items() if key != _DESCRIBED_SHAPE
    }
    if described is not None and list(described) != list(shape):
        for key in _GEOMETRY_FIELDS:
            result.pop(key, None)
    return result


def _vector_components(layout: Any) -> list[dict[str, Any]]:
    """Name the leaves the environment concatenated into ``vector``."""
    components: list[dict[str, Any]] = []
    for term in getattr(layout, "terms", ()):
        vector_slice = getattr(term, "vector_slice", None)
        if not getattr(term, "vector", False) or vector_slice is None:
            continue
        if int(vector_slice[1]) <= int(vector_slice[0]):
            # A zero-width leaf (e.g. no joints) occupies no columns.
            continue
        components.append(
            {
                "category": str(getattr(term, "category", "")),
                "key": str(term.key),
                "shape": [int(dim) for dim in term.shape],
                "dtype": str(term.dtype).removeprefix("torch."),
                "start": int(vector_slice[0]),
                "end": int(vector_slice[1]),
            }
        )
    return components


def _demo_action_contract(wrapper: RobomimicEnvWrapper) -> dict[str, Any]:
    source_layout = _runtime_action_layout(wrapper)
    source_terms = (
        getattr(source_layout, "terms", None) if source_layout is not None else None
    )
    if isinstance(source_terms, tuple) and source_terms:
        scales = _json_scales(getattr(source_layout, "scale", {}))
        schema = _runtime_action_schema(source_layout, scales=scales)
        source = "runtime_action_layout"
    else:
        schema = _effective_action_schema(wrapper)
        source = "effective_single_action_space"
        scales = {}

    effective_space = wrapper.get_action_space()
    action_dim = int(wrapper.get_action_dim())
    flat_low, flat_high = _finite_space_bounds(effective_space, action_dim)
    if flat_low is None or flat_high is None:
        normalized: bool | None = None
    else:
        normalized = bool(
            np.array_equal(np.asarray(flat_low), np.full(action_dim, -1.0))
            and np.array_equal(np.asarray(flat_high), np.full(action_dim, 1.0))
        )

    contract: dict[str, Any] = {
        "source": source,
        "schema": schema,
        "action_dim": action_dim,
        "normalized": normalized,
    }
    if flat_low is not None and flat_high is not None:
        contract["low"] = flat_low
        contract["high"] = flat_high
    if scales:
        contract["scales"] = scales
    return contract


def _runtime_action_layout(wrapper: RobomimicEnvWrapper) -> Any | None:
    for owner in (
        getattr(wrapper, "env", None),
        getattr(getattr(wrapper, "env", None), "unwrapped", None),
    ):
        layout = getattr(owner, "_action_layout", None)
        terms = getattr(layout, "terms", None)
        if layout is not None and isinstance(terms, tuple) and terms:
            if all(hasattr(term, "key") for term in terms):
                return layout
    return None


def _runtime_action_schema(
    layout: Any,
    *,
    scales: Mapping[str, float] | None = None,
) -> list[dict[str, Any]]:
    declared_scales = (
        _json_scales(getattr(layout, "scale", {})) if scales is None else dict(scales)
    )
    schema: list[dict[str, Any]] = []
    for term in layout.terms:
        action_slice = term.action_slice
        start = action_slice.start
        end = action_slice.stop
        if start is None or end is None:
            raise ValueError(f"runtime action term {term.key!r} has an open slice")
        entry: dict[str, Any] = {
            "name": str(term.key),
            "path": [str(term.key)],
            "shape": [int(dim) for dim in term.shape],
            "dim": int(term.size),
            "start": int(start),
            "end": int(end),
            "dtype": str(term.dtype),
        }
        low, high = _finite_space_bounds(term.space, int(term.size))
        if low is not None and high is not None:
            entry["low"] = low
            entry["high"] = high
        if term.key in declared_scales:
            entry["scale"] = declared_scales[term.key]
        schema.append(entry)
    return schema


def _effective_action_schema(wrapper: RobomimicEnvWrapper) -> list[dict[str, Any]]:
    raw_schema = wrapper._action_layout.schema()
    schema: list[dict[str, Any]] = []
    for term, raw_entry in zip(wrapper._action_layout.terms, raw_schema, strict=True):
        entry = dict(raw_entry)
        # Keep a scalar term's true ``shape=[]`` in the additive contract.
        # The legacy action schema uses ``[1]`` for its flat dimension and is
        # intentionally left untouched.
        entry["shape"] = [int(dim) for dim in term.shape]
        low, high = _finite_space_bounds(term.space, int(term.size))
        if low is not None and high is not None:
            entry["low"] = low
            entry["high"] = high
        schema.append(entry)
    return schema


def _finite_space_bounds(
    space: Any, dim: int
) -> tuple[list[float] | None, list[float] | None]:
    try:
        low = np.asarray(space.low).reshape(-1)
        high = np.asarray(space.high).reshape(-1)
    except (AttributeError, TypeError, ValueError):
        return None, None
    if low.size != dim or high.size != dim:
        return None, None
    if not np.issubdtype(low.dtype, np.number) or not np.issubdtype(
        high.dtype, np.number
    ):
        return None, None
    if not np.all(np.isfinite(low)) or not np.all(np.isfinite(high)):
        return None, None
    return [float(value) for value in low], [float(value) for value in high]


def _json_scales(value: Any) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise ValueError("runtime action scales must be a mapping")
    scales: dict[str, float] = {}
    for key, raw_value in value.items():
        if not isinstance(key, str) or not key:
            raise ValueError("runtime action scale keys must be non-empty strings")
        if isinstance(raw_value, bool) or not isinstance(
            raw_value, (int, float, np.integer, np.floating)
        ):
            raise ValueError(
                f"runtime action scale {key!r} must be a finite real number"
            )
        numeric = float(raw_value)
        if not np.isfinite(numeric):
            raise ValueError(
                f"runtime action scale {key!r} must be a finite real number"
            )
        scales[key] = numeric
    return scales
