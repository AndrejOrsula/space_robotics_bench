from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from copy import deepcopy
from typing import Any

import gymnasium
import numpy as np
import torch
from skrl.envs.wrappers.torch.isaaclab_envs import IsaacLabWrapper

from srb._typing import ObservationGroup
from srb.core.env.introspect import EnvIntrospector
from srb.core.env.runtime import (
    ActionLayout,
    ObservationLayout,
    ObservationSpaceLayout,
)
from srb.core.spaces import ROOT_OBSERVATION_KEY

# Re-export the canonical value from `srb/core/spaces/keys.py`.
_ROOT_OBSERVATION_KEY = ROOT_OBSERVATION_KEY

_PRIVILEGED_GROUP = str(ObservationGroup.STATE_PRIVILEGED)
_PRIVILEGED_PREFIX = f"{_PRIVILEGED_GROUP}__"


def _is_privileged_key(key: str) -> bool:
    """Whether a top-level observation key carries critic-only signals."""

    return key == _PRIVILEGED_GROUP or key.startswith(_PRIVILEGED_PREFIX)


class SkrlEnvWrapper(IsaacLabWrapper):
    """Torch-native SKRL seam backed exclusively by SRB runtime layouts."""

    def __init__(
        self,
        env: Any,
        obs_keys: Sequence[str] = (),
        state_keys: Sequence[str] | None = None,
        is_running: Callable[[], bool] | None = None,
    ) -> None:
        self._env_view = EnvIntrospector.of(env)
        self._is_running = is_running
        super().__init__(env)
        self._obs_keys = tuple(obs_keys)
        source_observation_space = self._env_view.single_observation_space
        # Critic-only keys that the default actor projection withholds; they
        # are routed into the state stream when no other critic source exists.
        self._privileged_keys: tuple[str, ...] = ()
        if (
            not self._obs_keys
            and isinstance(source_observation_space, gymnasium.spaces.Dict)
            and "policy" in source_observation_space.spaces
        ):
            # `critic` and `state_privileged` are value-only groups. Keep them
            # out of the default actor stream; `obs_keys` can select otherwise.
            self._effective_obs_keys: tuple[str, ...] = ("policy",)
        elif not self._obs_keys and isinstance(
            source_observation_space, gymnasium.spaces.Dict
        ):
            # Assembled observations expose privileged leaves as separate
            # `state_privileged__*` keys, which the default actor stream omits.
            keys = tuple(str(key) for key in source_observation_space.spaces)
            self._privileged_keys = tuple(
                key for key in keys if _is_privileged_key(key)
            )
            if self._privileged_keys:
                actor_keys = tuple(
                    key for key in keys if key not in self._privileged_keys
                )
                if not actor_keys:
                    raise ValueError(
                        "SKRL observation space has no actor-visible keys: "
                        f"every key is privileged {list(self._privileged_keys)}"
                    )
                self._effective_obs_keys = actor_keys
            else:
                self._effective_obs_keys = self._obs_keys
        else:
            self._effective_obs_keys = self._obs_keys
        self._state_keys = None if state_keys is None else tuple(state_keys)
        # The key selection behind the state stream when it is a projection
        # of the observation mapping rather than ``critic``/``env.state()``.
        self._effective_state_keys = self._state_keys
        self._states: torch.Tensor | None = None
        self._state_from_critic = False
        self._state_from_env = False

        self._action_layout = self._derive_action_layout()
        self._single_action_space, self._batched_action_space = (
            self._action_layout.build_spaces(num_envs=self._env_view.num_envs)
        )
        self._clip_actions_min = torch.as_tensor(
            self._single_action_space.low,
            device=self._env_view.device,
            dtype=torch.float32,
        )
        self._clip_actions_max = torch.as_tensor(
            self._single_action_space.high,
            device=self._env_view.device,
            dtype=torch.float32,
        )

        observation_input = self._select_observation_space(self._effective_obs_keys)
        (
            self._observation_layout,
            self._observation_is_root,
            self._single_observation_space,
        ) = self._derive_observation_transform(observation_input)

        if self._state_keys is not None:
            state_input = self._select_observation_space(self._state_keys)
        else:
            single_observation_space = self._env_view.single_observation_space
            declared_state_space = getattr(self._unwrapped, "state_space", None)
            if (
                isinstance(single_observation_space, gymnasium.spaces.Dict)
                and "critic" in single_observation_space.spaces
            ):
                state_input = deepcopy(single_observation_space.spaces["critic"])
                self._state_from_critic = True
            elif isinstance(declared_state_space, gymnasium.Space) and callable(
                getattr(self._env, "state", None)
            ):
                # Done transitions need the critic's terminal snapshot, which
                # only the env can supply; fail here, not at the first done.
                if not self._exposes_attr("last_final_state"):
                    raise ValueError(
                        "SKRL env.state() critic stream requires the env to "
                        "expose last_final_state for terminal transitions"
                    )
                state_input = self._single_state_space(declared_state_space)
                self._state_from_env = True
            elif self._privileged_keys:
                # Keep actor and withheld privileged keys in source-space order.
                self._effective_state_keys = tuple(
                    str(key) for key in single_observation_space.spaces
                )
                state_input = self._select_observation_space(self._effective_state_keys)
            else:
                # SKRL 2.1 requires a concrete state space; use observations
                # when no separate critic or `env.state()` stream exists.
                state_input = deepcopy(observation_input)
        if state_input is None:
            self._state_layout = None
            self._state_is_root = False
            self._state_space = None
        else:
            (
                self._state_layout,
                self._state_is_root,
                self._state_space,
            ) = self._derive_observation_transform(state_input)

    @property
    def action_space(self) -> gymnasium.spaces.Box:
        return self._single_action_space

    @property
    def single_action_space(self) -> gymnasium.spaces.Box:
        return self._single_action_space

    @property
    def observation_space(self) -> gymnasium.spaces.Box:
        return self._single_observation_space

    @property
    def single_observation_space(self) -> gymnasium.spaces.Box:
        return self._single_observation_space

    @property
    def state_space(self) -> gymnasium.spaces.Box | None:
        return self._state_space

    def step(
        self,
        actions: object,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, Any]:
        action_tensor = self._action_layout.coerce(
            actions,
            num_envs=self._env_view.num_envs,
            device=self._env_view.device,
        )
        action_tensor = torch.clamp(
            action_tensor,
            min=self._clip_actions_min,
            max=self._clip_actions_max,
        ).detach()
        observations, reward, terminated, truncated, self._info = self._env.step(
            action_tensor
        )
        # SKRL trainers never poll the app; a Ctrl-C raised inside a Kit
        # callback can be swallowed, so surface it here.
        if self._is_running is not None and not self._is_running():
            raise KeyboardInterrupt
        self._observations = self._flatten_observations(observations)
        self._states = self._flatten_states(observations)
        self._info = self._attach_terminal_observation(
            self._info, terminated, truncated
        )
        return (
            self._observations,
            reward.view(-1, 1),
            terminated.view(-1, 1),
            truncated.view(-1, 1),
            self._info,
        )

    def reset(self) -> tuple[torch.Tensor, Any]:
        if self._reset_once:
            observations, self._info = self._env.reset(seed=self._seed)
            self._observations = self._flatten_observations(observations)
            self._states = self._flatten_states(observations)
            self._reset_once = False
            self._seed = None
        if not isinstance(self._observations, torch.Tensor):
            raise RuntimeError("SKRL reset did not initialize observations")
        return self._observations, self._info

    def state(self) -> torch.Tensor | None:
        """Return the separately projected critic/state stream."""

        return self._states

    def observation_from_value(self, observations: object) -> torch.Tensor:
        """Project a caller-supplied observation through the training layout."""
        selected = self.__extract_observations(observations)
        detailed = self._batch_observation_value(selected)
        return self._observation_layout.flatten_torch(detailed, batched=True)

    def state_from_value(self, observations: object) -> torch.Tensor:
        """Project a historical state without reading the live environment.

        A separately declared ``env.state()`` stream is available for live
        interaction only. Historical replay rows carry observations, not a
        callable environment snapshot, so callers must fail closed rather
        than accidentally pairing them with the current state.
        """

        if self._state_from_env:
            raise RuntimeError(
                "SKRL historical state projection requires an explicit state "
                "stream; env.state() is live-only"
            )
        if self._effective_state_keys is None and not self._state_from_critic:
            # Reuse actor projection to unwrap root rows such as `{"obs": ...}`.
            return self.observation_from_value(observations)
        if self._state_keys is None and self._privileged_keys:
            # Accept canonical rows like the actor path; both use this mapping.
            selected = self.__extract_observations(
                observations, self._effective_state_keys
            )
            detailed = self._batch_observation_value(
                selected,
                layout=self._state_layout,
                is_root=self._state_is_root,
            )
            return self._state_layout.flatten_torch(detailed, batched=True)
        return self._flatten_states(observations)

    def demo_observation_binding(self) -> dict[str, object]:
        """Describe the live observation projection for replay provenance."""
        from srb.wrappers.robomimic import (
            _descriptor_for_shape,
            _runtime_observation_descriptors,
        )

        # Match the collector's contract, including runtime sensor descriptors.
        descriptors = _runtime_observation_descriptors(self._unwrapped)
        schema: list[dict[str, object]] = []
        for term in self._observation_layout.terms:
            descriptor = descriptors.get(term.key) or descriptors.get(term.source_key)
            if self._observation_is_root:
                # Preserve Robomimic's stable root `obs` key.
                schema.append(
                    {
                        "key": "obs",
                        "source_key": "obs",
                        "output_key": "obs",
                        "modality": term.category,
                        "shape": list(term.shape),
                        "dtype": str(np.dtype(term.dtype)),
                        "vector": bool(term.vector),
                        "emitted_path": ["obs"],
                    }
                )
                if descriptor is not None:
                    schema[-1].update(
                        _descriptor_for_shape(descriptor, list(term.shape))
                    )
                continue
            emitted_path = (
                [str(term.category), str(term.key)]
                if term.category
                else [str(term.key)]
            )
            schema.append(
                {
                    "key": str(term.key),
                    "source_key": str(term.source_key),
                    "output_key": str(term.output_key),
                    "modality": term.category,
                    "shape": list(term.shape),
                    "dtype": str(np.dtype(term.dtype)),
                    "vector": bool(term.vector),
                    "emitted_path": emitted_path,
                }
            )
            if descriptor is not None:
                schema[-1].update(_descriptor_for_shape(descriptor, list(term.shape)))
            if term.vector_slice is not None:
                schema[-1]["vector_slice"] = [
                    int(term.vector_slice[0]),
                    int(term.vector_slice[1]),
                ]
        source_space = self._env_view.single_observation_space
        selection_present = bool(
            isinstance(source_space, gymnasium.spaces.Dict)
            and "policy" in source_space.spaces
        )
        keys: list[str]
        if self._observation_is_root:
            keys = ["obs"]
        else:
            keys = []
            for term in self._observation_layout.terms:
                key = str(term.category) if term.category else str(term.key)
                if key not in keys:
                    keys.append(key)
        return {
            "wrapper": "srb.wrappers.robomimic.RobomimicEnvWrapper",
            "obs_key": "policy",
            "projection": "select_obs_key_then_observation_layout_to_numpy",
            "projection_path": ["policy"],
            "selection": "if_present_else_root",
            "selection_present": selection_present,
            "root_observation_key": "obs",
            "root": bool(self._observation_is_root),
            "keys": keys,
            "terms": schema,
        }

    def _attach_terminal_observation(
        self,
        info: Any,
        terminated: object,
        truncated: object,
    ) -> Any:
        """Publish the live SRB terminal snapshot in SKRL's flat layout."""

        terminal = self._wrapper_attr("last_final_observation")
        if terminal is None:
            return info

        device = self._observations.device
        done = (
            torch.as_tensor(terminated, device=device, dtype=torch.bool)
            .reshape(-1)
            .clone()
        )
        done |= torch.as_tensor(truncated, device=device, dtype=torch.bool).reshape(-1)
        ids = torch.as_tensor(
            self._wrapper_attr("last_final_observation_ids"),
            device=device,
            dtype=torch.long,
        ).reshape(-1)
        mask = torch.as_tensor(
            self._wrapper_attr("last_final_observation_mask"),
            device=device,
            dtype=torch.bool,
        ).reshape(-1)
        expected_ids = done.nonzero(as_tuple=False).flatten()
        if mask.shape != done.shape or not torch.equal(mask, done):
            raise RuntimeError(
                "SKRL terminal observation mask does not match the step done mask"
            )
        if not torch.equal(ids, expected_ids):
            raise RuntimeError(
                "SKRL terminal observation ids do not match the step done mask"
            )

        transformed = self._flatten_observations(terminal)
        if (
            transformed.ndim != self._observations.ndim
            or transformed.shape[0] != ids.numel()
        ):
            raise RuntimeError(
                "SKRL terminal observation shape does not match the live snapshot"
            )
        full = self._observations.clone()
        full.index_copy_(0, ids, transformed)
        result = dict(info) if isinstance(info, Mapping) else {}
        result["final_observation"] = full
        result["final_observation_ids"] = ids
        result["final_observation_mask"] = mask
        terminal_state = self._wrapper_attr("last_final_state")
        if terminal_state is None and not self._state_from_env:
            terminal_state = terminal
        if terminal_state is not None and self._states is not None:
            transformed_state = self._flatten_states(
                terminal_state,
                explicit_state=True,
            )
            if (
                transformed_state.ndim != self._states.ndim
                or transformed_state.shape[0] != ids.numel()
            ):
                raise RuntimeError(
                    "SKRL terminal state shape does not match the live state"
                )
            full_state = self._states.clone()
            full_state.index_copy_(0, ids, transformed_state)
            result["final_state"] = full_state
            result["final_state_ids"] = ids
            result["final_state_mask"] = mask
        return result

    def _exposes_attr(self, name: str) -> bool:
        getter = getattr(self._env, "get_wrapper_attr", None)
        if callable(getter):
            try:
                getter(name)
                return True
            except AttributeError:
                pass
        return hasattr(self._env, name) or hasattr(self._unwrapped, name)

    def _wrapper_attr(self, name: str) -> Any:
        getter = getattr(self._env, "get_wrapper_attr", None)
        if callable(getter):
            try:
                return getter(name)
            except AttributeError:
                pass
        value = getattr(self._env, name, None)
        if value is not None:
            return value
        return getattr(self._unwrapped, name, None)

    def _derive_action_layout(self) -> ActionLayout:
        source_space = deepcopy(self._env_view.single_action_space)
        source_layout = getattr(self._unwrapped, "_action_layout", None)

        if isinstance(source_layout, ActionLayout):
            source_shape = tuple(source_space.shape)
            layout_shape = tuple(source_layout.single_space.shape)
            if source_shape != layout_shape:
                raise ValueError(
                    "effective action space and ActionLayout disagree: "
                    f"{source_shape} != {layout_shape}"
                )
            normalized = gymnasium.spaces.Box(
                low=-1.0,
                high=1.0,
                shape=source_shape,
                dtype=np.float32,
            )
            return ActionLayout.from_spaces(
                deepcopy(source_layout.detailed_space),
                ordering=source_layout.ordering,
                scale=source_layout.scale,
                single_space=normalized,
            )

        if isinstance(source_space, gymnasium.spaces.Box):
            if len(source_space.shape) != 1:
                raise ValueError(
                    "SKRL requires a one-dimensional SRB action vector, got "
                    f"{tuple(source_space.shape)}"
                )
            return ActionLayout.from_action_dim(int(source_space.shape[0]))
        if isinstance(source_space, gymnasium.spaces.Dict):
            return ActionLayout.from_spaces(source_space)
        raise TypeError(
            f"SKRL requires a Box or named Dict action space, got {source_space}"
        )

    def _select_observation_space(
        self,
        keys: Sequence[str],
    ) -> gymnasium.Space[Any]:
        source = self._env_view.single_observation_space
        if not keys:
            return deepcopy(source)
        if not isinstance(source, gymnasium.spaces.Dict):
            raise TypeError("observation key selection requires a Dict space")
        missing = [key for key in keys if key not in source.spaces]
        if missing:
            raise KeyError(f"observation space is missing keys: {missing}")
        return gymnasium.spaces.Dict(
            [(key, deepcopy(source.spaces[key])) for key in keys]
        )

    def _single_state_space(
        self, declared: gymnasium.Space[Any]
    ) -> gymnasium.Space[Any]:
        """Normalize a declared batched state space to SKRL's single space."""

        # Reuse the single observation space when it exactly matches the
        # declared batched state space after batching.
        if isinstance(self._env_view.single_observation_space, gymnasium.spaces.Dict):
            try:
                candidate = gymnasium.vector.utils.batch_space(
                    self._env_view.single_observation_space,
                    self._env_view.num_envs,
                )
            except (TypeError, ValueError):
                candidate = None
            if candidate is not None and declared == candidate:
                return deepcopy(self._env_view.single_observation_space)
        return self._unbatch_state_space(declared)

    def _unbatch_state_space(self, space: gymnasium.Space[Any]) -> gymnasium.Space[Any]:
        """Unbatch common Gym spaces while preserving dtype and bounds."""

        if isinstance(space, gymnasium.spaces.Box):
            shape = tuple(int(dim) for dim in space.shape)
            # Preserve a vector Box when its shape could also mean batched scalars.
            if len(shape) > 1 and shape[0] == self._env_view.num_envs:
                return gymnasium.spaces.Box(
                    low=np.asarray(space.low)[0],
                    high=np.asarray(space.high)[0],
                    dtype=space.dtype,
                )
            return deepcopy(space)
        if isinstance(space, gymnasium.spaces.Dict):
            return gymnasium.spaces.Dict(
                [
                    (key, self._unbatch_state_space(child))
                    for key, child in space.spaces.items()
                ]
            )
        if isinstance(space, gymnasium.spaces.Tuple):
            return gymnasium.spaces.Tuple(
                [self._unbatch_state_space(child) for child in space.spaces]
            )
        return deepcopy(space)

    @staticmethod
    def _derive_observation_transform(
        source: gymnasium.Space[Any],
    ) -> tuple[ObservationSpaceLayout, bool, gymnasium.spaces.Box]:
        is_root = not isinstance(source, gymnasium.spaces.Dict)
        detailed = (
            gymnasium.spaces.Dict([(_ROOT_OBSERVATION_KEY, source)])
            if is_root
            else source
        )
        layout = ObservationLayout.from_spaces(detailed)
        return layout, is_root, layout.merged_space()

    def _flatten_observations(self, observations: object) -> torch.Tensor:
        selected = self.__extract_observations(observations)
        detailed: Mapping[str, object]
        if self._observation_is_root:
            detailed = {_ROOT_OBSERVATION_KEY: selected}
        elif isinstance(selected, Mapping):
            detailed = selected
        else:
            raise TypeError(
                "SKRL expected a mapping observation for its Dict layout, got "
                f"{type(selected).__name__}"
            )
        return self._observation_layout.flatten_torch(detailed, batched=True)

    def _flatten_states(
        self,
        observations: object,
        *,
        explicit_state: bool = False,
    ) -> torch.Tensor | None:
        """Project the critic/state stream without aliasing policy inputs."""

        if self._state_layout is None:
            return None
        selected = self._extract_states(observations, explicit_state=explicit_state)
        if self._state_is_root:
            detailed: Mapping[str, object] = {_ROOT_OBSERVATION_KEY: selected}
        elif isinstance(selected, Mapping):
            detailed = selected
        else:
            raise TypeError(
                "SKRL expected a mapping state observation for its Dict layout, "
                f"got {type(selected).__name__}"
            )
        return self._state_layout.flatten_torch(detailed, batched=True)

    def _extract_states(
        self,
        observations: object,
        *,
        explicit_state: bool = False,
    ) -> object:
        if self._effective_state_keys is not None:
            return self.__extract_observations(observations, self._effective_state_keys)
        if self._state_from_critic:
            if explicit_state and (
                not isinstance(observations, Mapping) or "critic" not in observations
            ):
                return observations
            if not isinstance(observations, Mapping) or "critic" not in observations:
                raise KeyError("SKRL state layout requires the critic observation")
            return observations["critic"]
        if self._state_from_env and not explicit_state:
            state_getter = getattr(self._env, "state", None)
            if not callable(state_getter):
                raise RuntimeError("SKRL declared state stream has no state() source")
            state = state_getter()
            if state is None:
                raise RuntimeError("SKRL state() returned no live state")
            return state
        return observations

    def _batch_observation_value(
        self,
        observations: object,
        *,
        layout: ObservationSpaceLayout | None = None,
        is_root: bool | None = None,
    ) -> Mapping[str, object]:
        """Coerce one canonical row into a live layout's batch shape."""

        if layout is None:
            layout = self._observation_layout
            is_root = self._observation_is_root
        terms = tuple(layout.terms)
        if is_root:
            selected = observations
            if isinstance(selected, Mapping):
                if _ROOT_OBSERVATION_KEY in selected:
                    if len(selected) != 1:
                        raise ValueError(
                            "root observation contains unexpected projection keys"
                        )
                    selected = selected[_ROOT_OBSERVATION_KEY]
                elif len(selected) == 1:
                    selected = next(iter(selected.values()))
                else:
                    raise ValueError("root observation must contain one array leaf")
            if len(terms) != 1 or terms[0].category:
                raise ValueError("root observation layout has unexpected terms")
            return {terms[0].key: self._batch_observation_leaf(selected, terms[0])}

        if not isinstance(observations, Mapping):
            raise TypeError(
                "SKRL expected a mapping observation for its Dict layout, got "
                f"{type(observations).__name__}"
            )
        expected_groups = {term.category for term in terms if term.category}
        expected_leaves = {term.key for term in terms if not term.category}
        actual_groups = {
            key for key, value in observations.items() if isinstance(value, Mapping)
        }
        actual_leaves = set(observations) - actual_groups
        if actual_groups != expected_groups or actual_leaves != expected_leaves:
            raise ValueError("canonical observation keys do not match live layout")

        detailed: dict[str, object] = {}
        for category in expected_groups:
            group = observations[category]
            if not isinstance(group, Mapping):
                raise TypeError(f"observation category {category!r} must be a mapping")
            expected_keys = {term.key for term in terms if term.category == category}
            if set(group) != expected_keys:
                raise ValueError(
                    f"canonical observation category {category!r} does not match live layout"
                )
            detailed[category] = {
                term.key: self._batch_observation_leaf(group[term.key], term)
                for term in terms
                if term.category == category
            }
        for term in terms:
            if not term.category:
                detailed[term.key] = self._batch_observation_leaf(
                    observations[term.key], term
                )
        return detailed

    @staticmethod
    def _batch_observation_leaf(value: object, term: Any) -> torch.Tensor:
        if isinstance(value, np.ndarray) and not value.flags.writeable:
            value = np.array(value, copy=True)
        try:
            tensor = torch.as_tensor(value)
        except (TypeError, ValueError, RuntimeError) as exc:
            raise TypeError(
                f"observation {term.source_key!r} is not tensor-convertible"
            ) from exc
        expected_shape = tuple(int(dim) for dim in term.shape)
        shape = tuple(int(dim) for dim in tensor.shape)
        if shape == expected_shape:
            tensor = tensor.unsqueeze(0)
        elif len(shape) == len(expected_shape) + 1 and shape[1:] == expected_shape:
            if shape[0] <= 0:
                raise ValueError(f"observation {term.source_key!r} has an empty batch")
        else:
            raise ValueError(
                f"observation {term.source_key!r} shape {shape} does not match "
                f"{expected_shape} or (batch, {expected_shape})"
            )
        expected_dtype = torch.from_numpy(
            np.empty((), dtype=np.dtype(term.dtype))
        ).dtype
        if tensor.dtype != expected_dtype:
            tensor = tensor.to(dtype=expected_dtype)
        if tensor.dtype.is_floating_point and not torch.isfinite(tensor).all():
            raise ValueError(
                f"observation {term.source_key!r} contains non-finite values"
            )
        return tensor

    def __extract_observations(
        self,
        observations: object,
        keys: Sequence[str] | None = None,
    ) -> object:
        selected_keys = self._effective_obs_keys if keys is None else tuple(keys)
        if not selected_keys:
            return observations
        if not isinstance(observations, Mapping):
            raise TypeError(
                "observation key selection requires a mapping value, got "
                f"{type(observations).__name__}"
            )
        missing = [key for key in selected_keys if key not in observations]
        if missing:
            raise KeyError(f"observation value is missing keys: {missing}")
        return {key: observations[key] for key in selected_keys}
