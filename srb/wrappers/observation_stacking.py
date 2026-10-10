from collections.abc import Mapping
from copy import deepcopy
from typing import TYPE_CHECKING, Any, Union

import gymnasium
import numpy as np
import torch
from gymnasium import spaces
from gymnasium.vector import AutoresetMode

from srb.core.env.introspect import runtime_num_envs
from srb.core.env.wrapper_cfg import ObservationStackingCfg
from srb.core.spaces import is_box, is_dict, is_tuple, map_boxes
from srb.utils import logging
from srb.wrappers._utils import get_wrapper_attr as _get_wrapper_attr

if TYPE_CHECKING:
    from srb._typing import AnyEnv


_AUTORESET_SAME_STEP = "SameStep"
_AUTORESET_NEXT_STEP = "NextStep"
_AUTORESET_DISABLED = "Disabled"
_AUTORESET_MODES = frozenset(
    {_AUTORESET_SAME_STEP, _AUTORESET_NEXT_STEP, _AUTORESET_DISABLED}
)


class ObservationStackingWrapper(gymnasium.Wrapper):
    """Stack recent observations while preserving their structure and device."""

    def __init__(self, env: "AnyEnv", cfg: ObservationStackingCfg):
        super().__init__(env)
        self.cfg = cfg

        # Circular history represented by a list.
        self.frames: list[Any] = []

        # Retain the transformed terminal snapshot for this step.
        self._last_final_observation: Any = None
        self._last_final_observation_ids = torch.empty(0, dtype=torch.long)
        self._last_final_observation_mask: torch.Tensor | None = None
        self._pending_reset: torch.Tensor | None = None

        is_srb_env = bool(_get_wrapper_attr(env, "is_srb_env", False))
        source_metadata = _get_wrapper_attr(env, "metadata", {})
        source_mode = (
            source_metadata.get("autoreset_mode")
            if isinstance(source_metadata, Mapping)
            else None
        )
        if hasattr(source_mode, "value"):
            source_mode = source_mode.value
        if source_mode is None:
            source_mode = _AUTORESET_SAME_STEP if is_srb_env else _AUTORESET_NEXT_STEP
        if source_mode not in _AUTORESET_MODES:
            raise ValueError(
                "unsupported autoreset_mode "
                f"{source_mode!r}; expected one of {sorted(_AUTORESET_MODES)}"
            )
        self._autoreset_mode = source_mode
        self._metadata = (
            dict(source_metadata) if isinstance(source_metadata, Mapping) else {}
        )
        self._metadata["autoreset_mode"] = AutoresetMode(source_mode)

        # Detect vector environments.
        self._num_envs = runtime_num_envs(env)
        self.is_vector_env = self._num_envs > 0

        # Read effective single and batched spaces separately.
        single_source = _get_wrapper_attr(env, "single_observation_space")
        main_source = getattr(env, "observation_space", None)
        if main_source is None:
            raise AttributeError(
                "wrapped environment does not expose observation_space"
            )
        if single_source is None:
            if self.is_vector_env:
                raise AttributeError(
                    "vector environment does not expose single_observation_space"
                )
            single_source = main_source

        # Copy source spaces before transforming their Box bounds.
        self.single_observation_space = self._transform_space(
            deepcopy(single_source), stack_axis=0
        )

        # Keep the batch axis intact for vector environments.
        self.main_stack_axis = 1 if self.is_vector_env else 0
        self.observation_space = self._transform_space(
            deepcopy(main_source), stack_axis=self.main_stack_axis
        )

        action_space = getattr(env, "action_space", None)
        single_action_space = _get_wrapper_attr(env, "single_action_space")
        if single_action_space is None:
            if self.is_vector_env and action_space is not None:
                raise AttributeError(
                    "vector environment does not expose single_action_space"
                )
            single_action_space = action_space
        if single_action_space is not None:
            self.single_action_space = deepcopy(single_action_space)
            self.action_space = (
                deepcopy(action_space)
                if action_space is not None
                else deepcopy(self.single_action_space)
            )

    def _transform_space(
        self, space: gymnasium.Space, stack_axis: int
    ) -> gymnasium.Space:
        """
        Recursively modifies the observation space metadata.
        """

        def stack_box(space: spaces.Box) -> spaces.Box:
            # Expand bounds along the stacking axis.
            low_source = np.array(space.low, copy=True)
            high_source = np.array(space.high, copy=True)
            if stack_axis >= low_source.ndim:
                low = np.stack([low_source] * self.cfg.n, axis=stack_axis)
                high = np.stack([high_source] * self.cfg.n, axis=stack_axis)
            else:
                low = np.concatenate([low_source] * self.cfg.n, axis=stack_axis)
                high = np.concatenate([high_source] * self.cfg.n, axis=stack_axis)
            return spaces.Box(low=low, high=high, dtype=space.dtype)

        # Non-Box spaces retain their latest-frame fallback.
        return map_boxes(space, stack_box)

    def _clone_obs(self, obs: Any) -> Any:
        """
        Deep clone to prevent simulator memory buffer overwrite issues.
        Critical for PhysX/IsaacLab environments where tensors view shared memory.
        """
        if isinstance(obs, torch.Tensor):
            return obs.clone()
        elif isinstance(obs, np.ndarray):
            return obs.copy()
        elif isinstance(obs, dict):
            return {k: self._clone_obs(v) for k, v in obs.items()}
        elif isinstance(obs, tuple):
            return tuple(self._clone_obs(v) for v in obs)
        return obs

    def _get_stacked_obs(
        self, frames: list[Any], space: gymnasium.Space
    ) -> Union[np.ndarray, torch.Tensor, dict[str, Any], tuple[Any, ...]]:
        """
        Concatenates frames along the feature dimension.
        """
        # Recurse through composite spaces.
        if is_dict(space):
            return {
                key: self._get_stacked_obs(
                    [f[key] for f in frames],
                    subspace,
                )
                for key, subspace in space.spaces.items()
            }
        elif is_tuple(space):
            return tuple(
                self._get_stacked_obs(
                    [f[i] for f in frames],
                    subspace,
                )
                for i, subspace in enumerate(space.spaces)
            )

        # Stack Box values.
        elif is_box(space):
            # Type check is sufficient for the recursive base case.
            first = frames[0]
            if isinstance(first, torch.Tensor):
                if self.main_stack_axis >= first.ndim:
                    return torch.stack(frames, dim=self.main_stack_axis)
                return torch.cat(frames, dim=self.main_stack_axis)
            elif isinstance(first, np.ndarray):
                if self.main_stack_axis >= first.ndim:
                    return np.stack(frames, axis=self.main_stack_axis)
                return np.concatenate(frames, axis=self.main_stack_axis)
            elif np.isscalar(first):
                return np.stack(frames, axis=self.main_stack_axis)
            else:
                return frames[-1]

        # Discrete and text values use the latest frame.
        else:
            return frames[-1]

    def _done_mask(self, terminated: Any, truncated: Any) -> "torch.Tensor | None":
        """Build a (num_envs,) bool mask of sub-envs that finished this step."""

        def _as_bool(x: Any) -> "torch.Tensor | None":
            if isinstance(x, torch.Tensor):
                return x.reshape(-1).bool()
            if isinstance(x, np.ndarray):
                return torch.as_tensor(x).reshape(-1).bool()
            return None

        mask: torch.Tensor | None = None
        for part in (_as_bool(terminated), _as_bool(truncated)):
            if part is None:
                continue
            mask = part if mask is None else (mask | part)
        return mask

    def _select_reset(self, prev: Any, new: Any, mask: "torch.Tensor") -> Any:
        """Per-env: take *new*'s rows where *mask*, else keep *prev*'s rows."""
        if isinstance(prev, dict):
            return {k: self._select_reset(prev[k], new[k], mask) for k in prev}
        if isinstance(prev, tuple):
            return tuple(
                self._select_reset(p, n, mask) for p, n in zip(prev, new, strict=True)
            )
        if isinstance(prev, torch.Tensor):
            m = mask.to(prev.device).view(prev.shape[0], *([1] * (prev.ndim - 1)))
            return torch.where(m, new, prev)
        if isinstance(prev, np.ndarray):
            m = mask.cpu().numpy().reshape(prev.shape[0], *([1] * (prev.ndim - 1)))
            return np.where(m, new, prev)
        return prev

    def _index_rows(self, value: Any, ids: torch.Tensor) -> Any:
        if isinstance(value, dict):
            return {key: self._index_rows(child, ids) for key, child in value.items()}
        if isinstance(value, tuple):
            return tuple(self._index_rows(child, ids) for child in value)
        if isinstance(value, list):
            return [self._index_rows(child, ids) for child in value]
        if isinstance(value, torch.Tensor):
            return value.index_select(0, ids.to(device=value.device))
        if isinstance(value, np.ndarray):
            return value[ids.detach().cpu().numpy()]
        return value

    def _terminal_state(
        self, obs: Any, done_ids: torch.Tensor
    ) -> tuple[Any, torch.Tensor]:
        source = self.env
        terminal = _get_wrapper_attr(source, "last_final_observation")
        source_ids = _get_wrapper_attr(source, "last_final_observation_ids")
        if terminal is None:
            if self._autoreset_mode == _AUTORESET_SAME_STEP:
                raise RuntimeError(
                    "SRB autoreset step has no last_final_observation snapshot"
                )
            # Generic vector wrappers fall back to the returned done row.
            terminal = self._index_rows(obs, done_ids)
            source_ids = done_ids
        if source_ids is None:
            source_ids = done_ids
        ids = torch.as_tensor(source_ids, dtype=torch.long).reshape(-1)
        if not torch.equal(ids.cpu(), done_ids.cpu()):
            raise RuntimeError(
                "terminal observation ids do not match the step done mask"
            )
        return self._clone_obs(terminal), ids

    @property
    def autoreset_mode(self) -> str:
        return self._autoreset_mode

    def _stack_terminal_observation(self, terminal: Any, ids: torch.Tensor) -> Any:
        if self.cfg.n <= 1:
            return self._clone_obs(terminal)
        history = [self._index_rows(frame, ids) for frame in self.frames[1:]]
        return self._get_stacked_obs(history + [terminal], self.observation_space)

    @property
    def last_final_observation(self) -> Any:
        return self._clone_obs(self._last_final_observation)

    @property
    def last_final_observation_ids(self) -> torch.Tensor:
        return self._last_final_observation_ids.clone()

    @property
    def last_final_observation_mask(self) -> torch.Tensor:
        if self._last_final_observation_mask is None:
            return torch.zeros(self._num_envs, dtype=torch.bool)
        return self._last_final_observation_mask.clone()

    def reset(
        self, *, seed: int | None = None, options: dict | None = None
    ) -> tuple[Any, dict]:
        obs, info = self.env.reset(seed=seed, options=options)

        # Clone PhysX-backed buffers.
        obs = self._clone_obs(obs)

        # Seed history with the cloned initial frame.
        self.frames = [obs] * self.cfg.n
        self._last_final_observation = None
        self._last_final_observation_ids = torch.empty(0, dtype=torch.long)
        self._pending_reset = None
        self._last_final_observation_mask = torch.zeros(
            self._num_envs, dtype=torch.bool
        )

        return self._get_stacked_obs(self.frames, self.observation_space), info

    def step(self, action: Any) -> tuple[Any, float, bool, bool, dict]:
        obs, reward, terminated, truncated, info = self.env.step(action)

        # Clone PhysX-backed buffers.
        obs = self._clone_obs(obs)

        done_mask = self._done_mask(terminated, truncated)
        done_ids = (
            done_mask.nonzero(as_tuple=False).flatten()
            if done_mask is not None
            else torch.empty(0, dtype=torch.long)
        )

        if done_ids.numel() > 0:
            terminal, terminal_ids = self._terminal_state(obs, done_ids)
            self._last_final_observation = self._stack_terminal_observation(
                terminal, terminal_ids
            )
            self._last_final_observation_ids = terminal_ids.clone()
            self._last_final_observation_mask = done_mask.clone()
        else:
            self._last_final_observation = None
            self._last_final_observation_ids = torch.empty(0, dtype=torch.long)
            self._last_final_observation_mask = (
                done_mask.clone()
                if done_mask is not None
                else torch.zeros(self._num_envs, dtype=torch.bool)
            )

        # Rotate history.
        self.frames.pop(0)
        self.frames.append(obs)

        if self.is_vector_env and done_mask is not None:
            if done_ids.numel() > 0 and self._autoreset_mode == _AUTORESET_SAME_STEP:
                # Same-step autoreset returns new episode rows; reseed history.
                self.frames = [
                    self._select_reset(frame, obs, done_mask) for frame in self.frames
                ]
                self._pending_reset = None
            elif self._autoreset_mode == _AUTORESET_NEXT_STEP:
                # Next-step autoreset needs reseeding after its reset observation.
                if self._pending_reset is not None and bool(self._pending_reset.any()):
                    self.frames = [
                        self._select_reset(frame, obs, self._pending_reset)
                        for frame in self.frames[:-1]
                    ] + [self.frames[-1]]
                self._pending_reset = done_mask.clone()
            else:
                # Disabled autoreset waits for an explicit reset.
                self._pending_reset = None

        stacked_obs = self._get_stacked_obs(self.frames, self.observation_space)

        return stacked_obs, reward, terminated, truncated, info

    @classmethod
    def wrap(
        cls,
        env: "AnyEnv",
        cfg: ObservationStackingCfg,
    ) -> "AnyEnv":
        if not cfg.enable:
            logging.debug("Observation stacking is disabled.")
            return env

        if cfg.n <= 1:
            logging.debug(f"Observation stacking disabled (num_stack={cfg.n}).")
            return env

        logging.info(f"Observation stacking enabled: {cfg}")
        return cls(env, cfg)


__all__ = ("ObservationStackingCfg", "ObservationStackingWrapper")
