from collections.abc import Mapping, Sequence
from copy import deepcopy
from typing import TYPE_CHECKING, Any

import gymnasium
import numpy as np
import torch
from stable_baselines3.common.env_util import is_wrapped
from stable_baselines3.common.vec_env.base_vec_env import (
    VecEnv,
    VecEnvIndices,
    VecEnvObs,
    VecEnvStepReturn,
)

from srb.core.env.introspect import EnvIntrospector
from srb.core.env.runtime import (
    ActionLayout,
    ActionLayoutLike,
    build_observation_space_layout,
)
from srb.core.spaces import (
    ROOT_OBSERVATION_KEY,
    autobound_action_space,
    coerce_gym_step_outputs,
    is_box,
)
from srb.wrappers._utils import get_wrapper_attr as _get_wrapper_attr

if TYPE_CHECKING:
    from srb._typing import AnyEnv


# Keep the private alias; the canonical value lives in ``spaces/keys.py``.
_ROOT_OBSERVATION_KEY = ROOT_OBSERVATION_KEY


def _build_action_layout(
    env: Any,
    action_space: gymnasium.Space[Any],
) -> ActionLayoutLike:
    """Use the runtime layout when it matches the effective adapter space."""
    source_layout = getattr(getattr(env, "unwrapped", env), "_action_layout", None)
    return ActionLayout.from_space(
        action_space,
        source_layout=source_layout,
    )


def _observation_value(
    observations: Mapping[str, Any],
    category: str,
    key: str,
) -> Any:
    if category:
        group = observations.get(category)
        if not isinstance(group, Mapping):
            raise TypeError(f"observation category '{category}' is not a mapping")
        return group[key]
    return observations[key]


def _add_single_batch_axis(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _add_single_batch_axis(child) for key, child in value.items()}
    if isinstance(value, tuple):
        return tuple(_add_single_batch_axis(child) for child in value)
    if isinstance(value, list):
        return [_add_single_batch_axis(child) for child in value]
    if isinstance(value, np.ndarray):
        return value.reshape((1, *value.shape))
    return value


def _index_observation(value: Any, index: int) -> Any:
    if isinstance(value, Mapping):
        return {key: _index_observation(child, index) for key, child in value.items()}
    if isinstance(value, tuple):
        return tuple(_index_observation(child, index) for child in value)
    if isinstance(value, list):
        return [_index_observation(child, index) for child in value]
    return value[index]


def _index_success_value(value: Any, index: int) -> Any:
    """Select one explicit vector success value without scalar broadcasting."""
    if isinstance(value, torch.Tensor):
        if value.ndim == 0:
            return None
        selected = value[index].detach().cpu()
        return selected.item() if selected.numel() == 1 else selected.numpy()
    if isinstance(value, np.ndarray):
        if value.ndim == 0:
            return None
        selected = value[index]
        return selected.item() if np.ndim(selected) == 0 else selected
    if isinstance(value, (list, tuple)):
        if index >= len(value):
            return None
        return value[index]
    return None


class Sb3EnvWrapper(VecEnv):
    def __init__(self, env: "AnyEnv", ignore_extras: bool = True):
        self.env = env
        view = EnvIntrospector.of(env)
        self._env_view = view
        self.num_envs = view.num_envs
        self.sim_device = view.device
        self.render_mode = self.unwrapped.render_mode

        self._action_layout = _build_action_layout(env, view.single_action_space)
        self._observation_layout, self._observation_is_root = (
            build_observation_space_layout(view.single_observation_space)
        )

        if ignore_extras:
            self._default_infos = [{"episode": None} for _ in range(self.num_envs)]
            self._process_extras = self._process_extras_ignore

        # Get Gym spaces; SB3 requires bounded continuous actions.
        observation_space = view.single_observation_space  # type: ignore[misc]
        action_space = view.single_action_space  # type: ignore[misc]
        # Bound continuous actions to SB3’s normalized Box contract.
        if is_box(action_space):
            action_space = autobound_action_space(action_space, low=-1.0, high=1.0)

        VecEnv.__init__(self, self.num_envs, observation_space, action_space)
        # Track episodic return and length for monitor info.
        self._ep_rew_buf = torch.zeros(self.num_envs, device=self.sim_device)
        self._ep_len_buf = torch.zeros(self.num_envs, device=self.sim_device)

    def __str__(self):
        return f"<{type(self).__name__}{self.env}>"

    def __repr__(self):
        return str(self)

    @classmethod
    def class_name(cls) -> str:
        return cls.__name__

    @property
    def unwrapped(self) -> "AnyEnv":
        return self.env.unwrapped  # type: ignore[return-value]

    def get_episode_rewards(self) -> Sequence[float]:
        return self._ep_rew_buf.cpu().tolist()

    def get_episode_lengths(self) -> Sequence[int]:
        return self._ep_len_buf.cpu().tolist()

    def seed(self, seed: int | None = None) -> Sequence[int | None]:
        # SrbEnv uses a negative seed to request a newly drawn seed.
        seed_value = -1 if seed is None else seed
        return [self.unwrapped.seed(seed_value)] * self.unwrapped.num_envs  # type: ignore[return-value]

    def reset(self) -> VecEnvObs:
        obs_dict, _ = self.env.reset()
        self._ep_rew_buf.zero_()
        self._ep_len_buf.zero_()
        return self._process_obs(obs_dict)  # type: ignore[return-value]

    def observation_from_value(self, observations: Any) -> VecEnvObs:
        """Convert an SRB observation without changing live environment state."""
        return self._process_obs(observations)  # type: ignore[return-value]

    def step_async(self, actions):
        # Coerce actions to the environment's batched tensor layout.
        self._async_actions = self._action_layout.coerce(
            actions,
            num_envs=self.num_envs,
            device=self.sim_device,
        )

    def _process_obs(self, obs: Any) -> VecEnvObs:
        if self._observation_is_root:
            if isinstance(obs, Mapping):
                raise TypeError("root observation space received a mapping")
            detailed: Mapping[str, Any] = {_ROOT_OBSERVATION_KEY: obs}
        elif isinstance(obs, Mapping):
            detailed = obs
        else:
            raise TypeError("Dict observation space received a non-mapping value")

        if self._observation_layout.terms:
            term = self._observation_layout.terms[0]
            value = _observation_value(detailed, term.category, term.key)
            value_shape = tuple(getattr(value, "shape", ()))
            is_batched = len(value_shape) == len(term.shape) + 1
        else:
            is_batched = self.num_envs != 1
        # Convert observations to NumPy at the SB3 boundary.
        converted = self._observation_layout.to_numpy(
            detailed,
            batched=True if self.num_envs != 1 else is_batched,
        )
        if self.num_envs == 1 and not is_batched:
            converted = _add_single_batch_axis(converted)

        if self._observation_is_root:
            return converted[_ROOT_OBSERVATION_KEY]  # type: ignore[return-value]
        return converted  # type: ignore[return-value]

    def step_wait(self) -> VecEnvStepReturn:
        obs_dict, reward, terminated, truncated, extras = self.env.step(
            self._async_actions  # type: ignore[misc]
        )
        # Track undiscounted episode return and length.
        self._ep_rew_buf += reward
        self._ep_len_buf += 1

        obs = self._process_obs(obs_dict)  # type: ignore[misc]
        reward, terminated, truncated = coerce_gym_step_outputs(
            reward, terminated, truncated, output="numpy"
        )

        dones = terminated | truncated
        reset_ids = np.argwhere(dones > 0).flatten()
        infos = self._process_extras(
            obs,  # type: ignore[misc]
            terminated,  # type: ignore[misc]
            truncated,  # type: ignore[misc]
            extras,
            reset_ids,  # type: ignore[misc]
        )

        self._ep_rew_buf[reset_ids] = 0
        self._ep_len_buf[reset_ids] = 0

        return obs, reward, dones, infos  # type: ignore[return-value]

    def _terminal_observations(
        self, obs: VecEnvObs, reset_ids: np.ndarray
    ) -> dict[int, Any]:
        if reset_ids.size == 0:
            return {}
        terminal = _get_wrapper_attr(self.env, "last_final_observation")
        if terminal is None:
            if bool(_get_wrapper_attr(self.env, "is_srb_env", False)):
                raise RuntimeError(
                    "SRB autoreset step has no last_final_observation snapshot"
                )
            # Keep compatibility with VecEnvs lacking a terminal accessor.
            return {
                int(env_id): _index_observation(obs, int(env_id))
                for env_id in reset_ids.tolist()
            }

        terminal_ids = _get_wrapper_attr(self.env, "last_final_observation_ids")
        if terminal_ids is None:
            mask = _get_wrapper_attr(self.env, "last_final_observation_mask")
            if mask is None:
                raise RuntimeError(
                    "terminal observation snapshot has no environment ids"
                )
            if isinstance(mask, torch.Tensor):
                terminal_ids = mask.detach().cpu().nonzero(as_tuple=False).flatten()
            else:
                terminal_ids = np.flatnonzero(np.asarray(mask, dtype=bool))
        if isinstance(terminal_ids, torch.Tensor):
            terminal_ids = terminal_ids.detach().cpu().numpy()
        terminal_ids = np.asarray(terminal_ids, dtype=np.int64).reshape(-1)
        expected_ids = np.asarray(reset_ids, dtype=np.int64).reshape(-1)
        if terminal_ids.tolist() != expected_ids.tolist():
            raise RuntimeError(
                "terminal observation ids do not match the step done mask"
            )

        transformed = self._process_obs(terminal)
        return {
            int(env_id): _index_observation(transformed, offset)
            for offset, env_id in enumerate(terminal_ids.tolist())
        }

    def close(self):
        self.env.close()

    def get_attr(self, attr_name: str, indices: VecEnvIndices = None):
        resolved_indices = list(self._get_indices(indices))
        attr_val = getattr(self.env, attr_name)
        if not isinstance(attr_val, torch.Tensor):
            return [attr_val] * len(resolved_indices)
        else:
            return attr_val[resolved_indices].detach().cpu().numpy()

    def set_attr(
        self, attr_name: str, value: Any, indices: Sequence[int] | None = None
    ):
        raise NotImplementedError("Setting attributes is not supported.")

    def env_method(
        self,
        method_name: str,
        *method_args,
        indices: Sequence[int] | None = None,
        **method_kwargs,
    ):
        if method_name == "render":
            # Gymnasium does not support changing render mode at runtime
            return self.env.render()
        else:
            # One vectorized env serves every index; `indices` is not forwarded.
            env_method = getattr(self.env, method_name)
            return env_method(*method_args, **method_kwargs)

    def env_is_wrapped(self, wrapper_class, indices=None):
        # Sub-environments are slices of one vectorized env sharing its wrappers.
        return [is_wrapped(self.env, wrapper_class)] * len(self._get_indices(indices))

    def get_images(self):
        raise NotImplementedError("Getting images is not supported.")

    def _process_extras(
        self,
        obs: Any,
        terminated: np.ndarray,
        truncated: np.ndarray,
        extras: dict[str, Any],
        reset_ids: np.ndarray,
    ) -> Sequence[Mapping[str, Any]]:
        terminal_observations = self._terminal_observations(obs, reset_ids)
        reset_set = set(reset_ids.tolist())
        # Create empty list of dictionaries to fill
        infos: list[dict[str, Any]] = [
            dict.fromkeys(extras.keys()) for _ in range(self.num_envs)
        ]
        # Fill-in information for each sub-environment
        # Note: This loop becomes slow when number of environments is large.
        for idx in range(self.num_envs):
            # Fill-in episode monitoring info
            if idx in reset_set:
                infos[idx]["episode"] = {}
                infos[idx]["episode"]["r"] = float(self._ep_rew_buf[idx])
                infos[idx]["episode"]["l"] = float(self._ep_len_buf[idx])
            else:
                infos[idx]["episode"] = None
            # Fill-in bootstrap information
            # Mark only time-limit truncations for SB3 bootstrapping.
            infos[idx]["TimeLimit.truncated"] = truncated[idx] and not terminated[idx]
            # Fill-in information from extras
            for key, value in extras.items():
                # 1. remap extra episodes information safely
                # 2. for others just store their values
                if key == "log":
                    # Only log this data for episodes that are terminated
                    if infos[idx]["episode"] is not None:
                        for sub_key, sub_value in value.items():
                            infos[idx]["episode"][sub_key] = sub_value
                else:
                    infos[idx][key] = value[idx]
            # Add information about terminal observation separately
            if idx in reset_set:
                infos[idx]["terminal_observation"] = terminal_observations[idx]
            else:
                infos[idx]["terminal_observation"] = None
        # Return list of dictionaries
        return infos

    def _process_extras_ignore(
        self,
        obs: np.ndarray,
        terminated: np.ndarray,
        truncated: np.ndarray,
        extras: dict[str, Any],
        reset_ids: np.ndarray,
    ) -> Sequence[Mapping[str, Any]]:
        infos = deepcopy(self._default_infos)

        terminal_observations = self._terminal_observations(obs, reset_ids)
        for idx in reset_ids:
            # Fill-in episode monitoring info
            infos[idx]["episode"] = {
                "r": float(self._ep_rew_buf[idx]),
                "l": float(self._ep_len_buf[idx]),
            }

            # Forward task success for evaluation; ignore other extras and scalars.
            success = extras.get("success")
            if success is not None:
                indexed_success = _index_success_value(success, int(idx))
                if indexed_success is not None:
                    infos[idx]["success"] = indexed_success
                    # SB3's standard key feeds its rollout/success_rate.
                    infos[idx]["is_success"] = indexed_success

            # Fill-in bootstrap information
            infos[idx]["TimeLimit.truncated"] = truncated[idx] and not terminated[idx]

            infos[idx]["terminal_observation"] = terminal_observations[int(idx)]

        return infos
