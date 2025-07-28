import functools
from typing import TYPE_CHECKING, Any, Dict, List, Mapping

import embodied
import gymnasium
import numpy
import torch
from elements import Space

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class EmbodiedEnvWrapper(embodied.Env):
    def __init__(
        self,
        env: "AnyEnv",
        obs_key: str = "image",
        act_key: str = "action",
    ):
        self.env = env
        self._obs_key = obs_key
        self._act_key = act_key

        # Extract spaces
        self._obs_space = self.unwrapped.single_observation_space  # type: ignore
        self._action_space = self.unwrapped.single_action_space  # type: ignore
        self._is_obs_dict = isinstance(self._obs_space, gymnasium.spaces.Dict)
        self._is_act_dict = isinstance(self._action_space, gymnasium.spaces.Dict)

        # Extract useful information
        self._num_envs = self.unwrapped.num_envs
        self._device = self.unwrapped.device

        # Init buffers
        self._done = numpy.ones(self._num_envs, dtype=bool)
        self._info = [None for _ in range(self._num_envs)]

    def __len__(self) -> int:
        return self._num_envs

    def __str__(self) -> str:
        return f"<{type(self).__name__}{self.env}>"

    def __repr__(self) -> str:
        return str(self)

    @classmethod
    def class_name(cls) -> str:
        return cls.__name__

    @property
    def unwrapped(self) -> "AnyEnv":
        return self.env.unwrapped  # type: ignore

    @property
    def info(self) -> Mapping[str, Any]:
        return self._info  # type: ignore

    @functools.cached_property
    def obs_space(self):
        if self._is_obs_dict:
            spaces = self._flatten(self.unwrapped.single_observation_space.spaces)  # type: ignore
        else:
            spaces = {self._obs_key: self.unwrapped.single_observation_space}  # type: ignore
        spaces = {k: self._convert(v) for k, v in spaces.items()}
        return {
            **spaces,
            "reward": Space(numpy.float32),
            "is_first": Space(bool),
            "is_last": Space(bool),
            "is_terminal": Space(bool),
        }

    @functools.cached_property
    def act_space(self):
        if self._is_act_dict:
            spaces = self._flatten(self._action_space.spaces)  # type: ignore
        else:
            spaces = {self._act_key: self._action_space}
        spaces = {k: self._convert(v) for k, v in spaces.items()}
        spaces["reset"] = Space(bool)
        return spaces

    def seed(self, seed: int | None = None) -> List[int | None]:
        return [self.unwrapped.seed(seed)] * self.unwrapped.num_envs  # type: ignore

    def reset(
        self,
    ) -> Mapping[str, numpy.ndarray]:
        obs, self._info = self.env.reset()
        self._done = numpy.zeros(self._num_envs, dtype=bool)
        return self._obs(
            obs=obs,  # type: ignore
            reward=numpy.zeros(self._num_envs, dtype=numpy.float32),
            is_first=numpy.ones(self._num_envs, dtype=bool),
            is_last=numpy.zeros(self._num_envs, dtype=bool),
            is_terminal=numpy.zeros(self._num_envs, dtype=bool),
        )

    def step(
        self, action: Mapping[str, numpy.ndarray | torch.Tensor]
    ) -> Mapping[str, numpy.ndarray]:
        if action["reset"].all() or self._done.all():
            return self.reset()

        if self._is_act_dict:
            act = self._unflatten(action)  # type: ignore
        else:
            act = action[self._act_key]  # type: ignore
            if not isinstance(act, torch.Tensor):
                act = torch.from_numpy(act).to(device=self._device)
            else:
                act = act.to(device=self._device)

        obs, reward, terminated, truncated, self._info = self.env.step(act)  # type: ignore

        if isinstance(reward, torch.Tensor):
            reward = reward.detach().cpu()
        elif not isinstance(reward, numpy.ndarray):
            reward = numpy.array((reward,), dtype=numpy.float32)

        if isinstance(terminated, torch.Tensor):
            terminated = terminated.detach().cpu()
        elif not isinstance(terminated, numpy.ndarray):
            terminated = numpy.array((terminated,), dtype=bool)

        if isinstance(truncated, torch.Tensor):
            truncated = truncated.detach().cpu()
        elif not isinstance(truncated, numpy.ndarray):
            truncated = numpy.array((truncated,), dtype=bool)

        self._done = terminated | truncated

        return self._obs(
            obs=obs,  # type: ignore
            reward=reward,
            is_first=numpy.zeros(self._num_envs, dtype=bool),
            is_last=self._done,
            is_terminal=terminated,
        )

    def render(self) -> numpy.ndarray:
        image = self.env.render()
        assert image is not None
        return image

    def close(self):
        try:
            self.env.close()
        except Exception:
            pass

    def get_attr(self, attr_name, indices=None) -> Any:
        if indices is None:
            indices = slice(None)
            num_indices = self._num_envs
        else:
            num_indices = len(indices)
        attr_val = getattr(self.env, attr_name)
        if not isinstance(attr_val, torch.Tensor):
            return [attr_val] * num_indices
        else:
            return attr_val[indices].detach().cpu()

    def set_attr(self, attr_name, value, indices=None):
        raise NotImplementedError("Setting attributes is not supported.")

    def env_method(
        self, method_name: str, *method_args, indices=None, **method_kwargs
    ) -> Any:
        if method_name == "render":
            return self.env.render()
        else:
            env_method = getattr(self.env, method_name)
            return env_method(*method_args, indices=indices, **method_kwargs)

    def _obs(
        self,
        obs: numpy.ndarray | torch.Tensor | dict[str, numpy.ndarray | torch.Tensor],
        reward: numpy.ndarray | torch.Tensor,
        is_first: numpy.ndarray | torch.Tensor,
        is_last: numpy.ndarray | torch.Tensor,
        is_terminal: numpy.ndarray | torch.Tensor,
    ) -> Mapping[str, numpy.ndarray | torch.Tensor]:
        if not self._is_obs_dict:
            obs = {self._obs_key: obs}  # type: ignore
        obs = self._flatten(obs)

        _first_obs = next(iter(obs.values()))
        if isinstance(_first_obs, torch.Tensor):
            obs = {k: v.detach().cpu() for k, v in obs.items()}  # type: ignore

        if self._num_envs == 1 and _first_obs.ndim == 1:
            obs = {k: v.reshape((1, *v.shape)) for k, v in obs.items()}

        obs.update(
            reward=reward,
            is_first=is_first,
            is_last=is_last,
            is_terminal=is_terminal,
        )
        return obs

    def _flatten(self, nest, prefix=None) -> Dict[str, Any]:
        result = {}
        for key, value in nest.items():
            key = prefix + "/" + key if prefix else key
            if isinstance(value, gymnasium.spaces.Dict):
                value = value.spaces
            if isinstance(value, Dict):
                result.update(self._flatten(value, key))
            else:
                result[key] = value
        return result

    def _unflatten(self, flat) -> Dict[str, Any]:
        result = {}
        for key, value in flat.items():
            parts = key.split("/")
            node = result
            for part in parts[:-1]:
                if part not in node:
                    node[part] = {}
                node = node[part]
            node[parts[-1]] = value
        return result

    def _convert(self, space) -> Space:
        if hasattr(space, "n"):
            return Space(numpy.int32, (), 0, space.n)
        return Space(space.dtype, space.shape, space.low, space.high)
