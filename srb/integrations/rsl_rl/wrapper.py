from collections.abc import Callable, Mapping
from copy import deepcopy
from typing import Any

import gymnasium
import numpy as np
import torch
from gymnasium.vector.utils import batch_space
from rsl_rl.env import VecEnv
from tensordict import TensorDict

from srb.core.env.introspect import EnvIntrospector
from srb.core.env.runtime import (
    ActionLayout,
    ActionLayoutLike,
    build_observation_space_layout,
)
from srb.core.spaces import ROOT_OBSERVATION_KEY
from srb.utils.failures import UnsupportedCapabilityError

# Keep the private name used below; the value is owned by srb/core/spaces/keys.py.
_ROOT_OBSERVATION_KEY = ROOT_OBSERVATION_KEY


def _build_action_layout(
    env: Any,
    action_space: gymnasium.Space[Any],
) -> ActionLayoutLike:
    source_layout = getattr(getattr(env, "unwrapped", env), "_action_layout", None)
    return ActionLayout.from_space(
        action_space,
        source_layout=source_layout,
    )


def _require_flat_action_space(
    action_space: gymnasium.Space[Any],
) -> gymnasium.spaces.Box:
    if isinstance(action_space, gymnasium.spaces.Box) and len(action_space.shape) == 1:
        return action_space
    raw_shape = getattr(action_space, "shape", None)
    shape = None if raw_shape is None else tuple(raw_shape)
    raise UnsupportedCapabilityError(
        code="integration.rsl_rl.action_space.non_flat",
        message=(
            "RSL-RL requires a rank-one Box action space; "
            f"got {type(action_space).__name__} with shape {shape}"
        ),
        component="integration.rsl_rl",
        details={"space_type": type(action_space).__name__, "shape": shape},
        fallback_allowed=False,
        evidence_valid=False,
    )


def _zero_observation(space: gymnasium.Space[Any], device: torch.device) -> Any:
    """Zero-valued batched observation with the structure of ``space``."""
    if isinstance(space, gymnasium.spaces.Dict):
        return {
            key: _zero_observation(subspace, device)
            for key, subspace in space.spaces.items()
        }
    if isinstance(space, gymnasium.spaces.Tuple):
        return tuple(_zero_observation(subspace, device) for subspace in space.spaces)
    shape = getattr(space, "shape", None)
    if shape is None:
        raise TypeError(
            f"RSL-RL cannot build a template observation for {type(space).__name__}"
        )
    return torch.as_tensor(np.zeros(shape, dtype=space.dtype), device=device)


def _space_shape_signature(space: gymnasium.Space[Any]) -> tuple[Any, ...]:
    if isinstance(space, gymnasium.spaces.Dict):
        return (
            "Dict",
            tuple(
                (key, _space_shape_signature(child))
                for key, child in space.spaces.items()
            ),
        )
    if isinstance(space, gymnasium.spaces.Tuple):
        return ("Tuple", tuple(_space_shape_signature(child) for child in space.spaces))
    shape = getattr(space, "shape", None)
    return (type(space), None if shape is None else tuple(shape))


def _add_observation_batch(observations: Any) -> Any:
    if isinstance(observations, Mapping):
        return {
            key: _add_observation_batch(value) for key, value in observations.items()
        }
    if isinstance(observations, tuple):
        return tuple(_add_observation_batch(value) for value in observations)
    if isinstance(observations, torch.Tensor):
        return observations.unsqueeze(0)
    if isinstance(observations, np.ndarray):
        return np.expand_dims(observations, axis=0)
    raise TypeError(
        f"RSL-RL observation leaf must be a numpy array or torch tensor, got "
        f"{type(observations).__name__}"
    )


class RslRlVecEnvWrapper(VecEnv):
    """Wraps a Space Robotics Bench gymnasium environment for the rsl_rl library.

    This wrapper must be the last wrapper in the chain as it does not follow
    the :class:`gymnasium.Wrapper` interface.
    """

    def __init__(
        self,
        env: gymnasium.Env,
        clip_actions: float | None = None,
        *,
        reset_on_init: bool = True,
        is_running: Callable[[], bool] | None = None,
    ) -> None:
        self.env = env
        self.clip_actions = clip_actions
        self._is_running = is_running
        view = EnvIntrospector.of(env)
        self._env_view = view
        flat_action_space = _require_flat_action_space(view.single_action_space)

        # Runtime state comes from the base env; spaces reflect the full wrapper
        # chain. Local copies keep clipping from changing other wrappers.
        self.num_envs = view.num_envs
        self.device = view.device
        self.max_episode_length = self.unwrapped.max_episode_length
        self._action_layout = _build_action_layout(env, view.single_action_space)
        self._observation_layout, self._observation_is_root = (
            build_observation_space_layout(view.single_observation_space)
        )
        self._observation_is_unbatched = self.num_envs == 1 and _space_shape_signature(
            view.observation_space
        ) == _space_shape_signature(view.single_observation_space)
        self.single_observation_space = deepcopy(view.single_observation_space)
        self.observation_space = (
            batch_space(self.single_observation_space, self.num_envs)
            if self._observation_is_unbatched
            else deepcopy(view.observation_space)
        )

        self.num_actions = int(flat_action_space.shape[0])

        if self.clip_actions is not None:
            if self.clip_actions < 0:
                raise ValueError(
                    f"clip_actions must be non-negative, got {self.clip_actions}"
                )
            self.single_action_space = gymnasium.spaces.Box(
                low=-self.clip_actions,
                high=self.clip_actions,
                shape=(self.num_actions,),
                dtype=np.float32,
            )
            if not isinstance(self.single_action_space, gymnasium.spaces.Box):
                raise TypeError("RSL clipped action space must be a Box")
            self.action_space = batch_space(self.single_action_space, self.num_envs)
            self._clip_low = torch.as_tensor(
                self.single_action_space.low, device=self.device, dtype=torch.float32
            )
            self._clip_high = torch.as_tensor(
                self.single_action_space.high, device=self.device, dtype=torch.float32
            )
        else:
            self.single_action_space = deepcopy(view.single_action_space)
            self.action_space = deepcopy(view.action_space)
            self._clip_low = None
            self._clip_high = None

        # The runner skips reset. Policy loading disables this for live sessions.
        self._cached_obs: Any = None
        if reset_on_init:
            obs_dict, _ = self.env.reset()
            # Cache the wrapper-chain value so runner sizing includes stacked
            # observations; reading from unwrapped would bypass transformations.
            self._cached_obs = self._validate_observations(obs_dict)

    @property
    def unwrapped(self) -> Any:
        """Returns the base environment."""
        return self.env.unwrapped

    @property
    def cfg(self) -> object:
        """Returns the configuration class instance of the environment."""
        return self.unwrapped.cfg

    @property
    def episode_length_buf(self) -> torch.Tensor:
        """The episode length buffer."""
        return self.unwrapped.episode_length_buf

    @episode_length_buf.setter
    def episode_length_buf(self, value: torch.Tensor) -> None:
        """Set the episode length buffer (needed for random initialization in rsl_rl)."""
        self.unwrapped.episode_length_buf = value

    def get_observations(self) -> TensorDict:
        """Returns the current observations of the environment."""
        if self._cached_obs is not None:
            return TensorDict(self._cached_obs, batch_size=[self.num_envs])
        # Policy loading may wrap a live env before reset/step. Build a zero
        # template from the effective space without invoking base-env hooks.
        # Unbatched single envs validate (and then batch) unbatched values.
        space = (
            self.single_observation_space
            if self._observation_is_unbatched
            else self.observation_space
        )
        return TensorDict(
            self._validate_observations(_zero_observation(space, self.device)),
            batch_size=[self.num_envs],
        )

    def observations_from_value(self, observations: Any) -> TensorDict:
        """Validate caller-supplied observations for inference.

        Public policy adapters must consume the observation passed by their
        caller. They must not read this wrapper's environment-local cache.
        """
        return TensorDict(
            self._validate_observations(observations),
            batch_size=[self.num_envs],
        )

    def reset(self) -> tuple[TensorDict, dict]:
        """Reset the environment."""
        obs_dict, extras = self.env.reset()
        self._cached_obs = self._validate_observations(obs_dict)
        return TensorDict(self._cached_obs, batch_size=[self.num_envs]), extras

    def step(
        self, actions: torch.Tensor
    ) -> tuple[TensorDict, torch.Tensor, torch.Tensor, dict]:
        """Apply actions and return (obs, rewards, dones, extras)."""
        actions = self._action_layout.coerce(
            actions,
            num_envs=self.num_envs,
            device=self.device,
        )
        if not isinstance(actions, torch.Tensor):
            raise TypeError("RSL-RL requires a flat Torch action tensor")
        if self._clip_low is not None and self._clip_high is not None:
            actions = torch.clamp(actions, min=self._clip_low, max=self._clip_high)

        obs_dict, rew, terminated, truncated, extras = self.env.step(actions)
        # RSL-RL never polls the app; a Ctrl-C raised inside a Kit callback
        # can be swallowed, so surface it here.
        if self._is_running is not None and not self._is_running():
            raise KeyboardInterrupt
        self._cached_obs = self._validate_observations(obs_dict)

        # Combine termination and truncation for RSL-RL.
        dones = (terminated | truncated).to(dtype=torch.long)

        # Preserve timeout flags for infinite-horizon tasks.
        if not self.unwrapped.cfg.is_finite_horizon:
            extras["time_outs"] = truncated

        return (
            TensorDict(self._cached_obs, batch_size=[self.num_envs]),
            rew,
            dones,
            extras,
        )

    def _validate_observations(self, observations: Any) -> Any:
        if self._observation_is_root:
            detailed: Mapping[str, object] = {_ROOT_OBSERVATION_KEY: observations}
        elif isinstance(observations, Mapping):
            detailed = observations
        else:
            raise TypeError("Dict observation space received a non-mapping value")
        if self._observation_is_unbatched:
            self._observation_layout.flatten_paths(detailed, batched=False)
            batched = _add_observation_batch(observations)
            return (
                {_ROOT_OBSERVATION_KEY: batched}
                if self._observation_is_root
                else batched
            )
        self._observation_layout.flatten_paths(detailed, batched=True)
        return detailed

    def close(self) -> None:
        """Close the environment."""
        self.env.close()
