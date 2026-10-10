from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any

import gymnasium
import numpy as np
import torch

from srb.core.env.introspect import EnvIntrospector
from srb.integrations.tdmpc2.observation import (
    STATE_KEY,
    Tdmpc2StateTransform,
    resolve_landscaping_projection,
)

_MISSING = object()

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class SrbTdmpc2Wrapper:
    """Adapt vectorized SRB environments to TD-MPC2's wrapper interface.

    The shared state transform defines spaces and runtime observations.
    Truncations close replay sequences while preserving bootstrap semantics;
    final observations retain pre-reset states for autoreset rows.
    """

    def __init__(self, env: "AnyEnv"):
        self.env = env
        view = EnvIntrospector.of(env)
        self._env_view = view

        # Spaces (single-env view, used by TD-MPC2 for config)
        self._raw_observation_space: gymnasium.Space = view.single_observation_space
        self.action_space: gymnasium.Space = view.single_action_space

        # Episode length
        self.max_episode_steps: int = self.unwrapped.max_episode_length  # type: ignore[attr-defined]

        # Vectorisation info
        self._num_envs = view.num_envs
        self._device = view.device
        self._is_obs_dict = view.is_obs_dict

        # Share one transform across spaces, observations, and saved metadata.
        # Unsupported leaves fail before model or replay allocation.
        self._state_transform = Tdmpc2StateTransform.from_observation_space(
            self._raw_observation_space,
            contract_projection=resolve_landscaping_projection(
                self.unwrapped, self._raw_observation_space
            ),
        )
        self.observation_space: gymnasium.Space = self._state_transform.state_space

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def unwrapped(self) -> "AnyEnv":
        return self.env.unwrapped  # type: ignore[return-value]

    @property
    def num_envs(self) -> int:
        return self._num_envs

    @property
    def device(self) -> torch.device:
        return self._device

    @property
    def raw_observation_space(self) -> gymnasium.Space:
        """Underlying assembled SRB observation space (pre-transform)."""
        return self._raw_observation_space

    @property
    def state_transform(self) -> Tdmpc2StateTransform:
        return self._state_transform

    @property
    def obs_shape(self) -> dict[str, tuple[int, ...]]:
        """``cfg.obs_shape`` for upstream, derived from the one transform."""
        return self._state_transform.obs_shape

    # ------------------------------------------------------------------
    # Core API
    # ------------------------------------------------------------------

    def reset(self, **kwargs) -> torch.Tensor:
        obs, _info = self.env.reset(**kwargs)
        return self._process_obs(obs)

    def step(
        self, action: torch.Tensor
    ) -> tuple[
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        dict[str, torch.Tensor],
    ]:
        action = action.to(self._device)
        obs, reward, terminated, truncated, info = self.env.step(action)

        terminated_t = self._batch_vector(terminated, "terminated", dtype=torch.bool)
        truncated_t = self._batch_vector(truncated, "truncated", dtype=torch.bool)
        done = terminated_t | truncated_t
        next_state = self._process_obs(obs)

        # Termination is not evidence of success; preserve an omitted signal.
        if info is None:
            raw_info: Mapping[str, Any] = {}
        elif isinstance(info, Mapping):
            raw_info = info
        else:
            raise TypeError(
                f"TD-MPC2 environment info must be a mapping, got {type(info).__name__}"
            )
        processed_info: dict[str, torch.Tensor] = {
            # Only MDP terminals set this flag; truncations still bootstrap.
            "terminated": terminated_t.float(),
            "truncated": truncated_t.float(),
        }
        if "success" in raw_info:
            processed_info["success"] = self._batch_vector(
                raw_info["success"], "success", dtype=torch.float32
            )

        # Keep pre-reset snapshots for replay while returning post-reset states.
        if bool(done.any()):
            processed_info.update(self._final_observation_info(next_state, done))

        return (
            next_state,
            self._batch_vector(reward, "reward", dtype=torch.float32),
            done,
            processed_info,
        )

    def _batch_vector(
        self,
        value: Any,
        name: str,
        *,
        dtype: torch.dtype,
    ) -> torch.Tensor:
        """Convert one scalar/vector step field without broadcasting rows."""
        tensor = value if isinstance(value, torch.Tensor) else torch.as_tensor(value)
        if tensor.numel() != self._num_envs:
            raise ValueError(
                f"TD-MPC2 {name} must contain exactly {self._num_envs} "
                f"environment values, got shape {tuple(tensor.shape)}"
            )
        return tensor.reshape(self._num_envs).to(device=self._device, dtype=dtype)

    def _env_attr(self, name: str) -> Any:
        """Read an SRB autoreset attribute through wrapper seams."""
        # Read through the outer wrapper first so snapshot shape matches obs.
        getter = self._explicit_attr(self.env, "get_wrapper_attr")
        if callable(getter):
            try:
                return getter(name)
            except AttributeError:
                pass
        value = self._explicit_attr(self.env, name)
        if value is not _MISSING:
            return value
        unwrapped = getattr(self.env, "unwrapped", None)
        value = self._explicit_attr(unwrapped, name)
        return None if value is _MISSING else value

    @staticmethod
    def _explicit_attr(owner: Any, name: str) -> Any:
        """Read a declared attribute, ignoring dynamic mock placeholders."""
        if owner is None:
            return _MISSING
        value = getattr(owner, name, _MISSING)
        if value is _MISSING:
            return _MISSING
        instance_attrs = getattr(owner, "__dict__", None)
        if isinstance(instance_attrs, Mapping) and name in instance_attrs:
            return value
        class_attr = getattr(type(owner), name, _MISSING)
        if class_attr is not _MISSING and hasattr(class_attr, "__get__"):
            return value
        return _MISSING

    def _final_observation_info(
        self, next_state: torch.Tensor, done: torch.Tensor
    ) -> dict[str, torch.Tensor]:
        """Validate and densify the pre-reset snapshot."""
        final_observation = self._env_attr("last_final_observation")
        if final_observation is None:
            raise RuntimeError(
                "TD-MPC2 SRB autoreset step has no last_final_observation "
                "snapshot for ended environments"
            )

        final_ids_raw = self._env_attr("last_final_observation_ids")
        final_mask_raw = self._env_attr("last_final_observation_mask")
        if final_ids_raw is None or final_mask_raw is None:
            raise RuntimeError(
                "TD-MPC2 SRB final observation snapshot must provide "
                "last_final_observation_ids and last_final_observation_mask"
            )

        final_ids_tensor = (
            final_ids_raw
            if isinstance(final_ids_raw, torch.Tensor)
            else torch.as_tensor(final_ids_raw)
        )
        if final_ids_tensor.numel() == 0:
            raise RuntimeError(
                "TD-MPC2 SRB final observation snapshot has no ended row ids"
            )
        if final_ids_tensor.dtype.is_floating_point:
            if not bool(torch.isfinite(final_ids_tensor).all()):
                raise RuntimeError(
                    "TD-MPC2 SRB final observation ids contain non-finite values"
                )
            if not bool(
                torch.equal(
                    final_ids_tensor,
                    final_ids_tensor.to(dtype=torch.long).to(
                        device=final_ids_tensor.device
                    ),
                )
            ):
                raise RuntimeError("TD-MPC2 SRB final observation ids must be integers")
        final_ids = final_ids_tensor.to(device=self._device, dtype=torch.long).reshape(
            -1
        )
        expected_ids = done.nonzero(as_tuple=False).flatten()
        if not torch.equal(final_ids, expected_ids):
            raise RuntimeError(
                "TD-MPC2 SRB final observation ids do not match the step done mask"
            )

        final_mask = self._batch_vector(
            final_mask_raw, "final_observation_mask", dtype=torch.bool
        )
        if not torch.equal(final_mask, done):
            raise RuntimeError(
                "TD-MPC2 SRB final observation mask does not match the step done mask"
            )

        final_state = self._process_obs(final_observation)
        if final_state.ndim == 1:
            final_state = final_state.reshape(1, -1)
        expected_shape = (int(final_ids.numel()), int(next_state.shape[-1]))
        if tuple(final_state.shape) != expected_shape:
            raise RuntimeError(
                "TD-MPC2 SRB final observation shape does not match its row ids: "
                f"got {tuple(final_state.shape)}, expected {expected_shape}"
            )

        dense_final = next_state.clone()
        dense_final.index_copy_(0, final_ids, final_state)
        return {
            "final_observation": dense_final,
            "final_observation_ids": final_ids,
            "final_observation_mask": final_mask,
        }

    def rand_act(self) -> torch.Tensor:
        """Return random actions as ``(num_envs, action_dim)`` tensor on device."""
        low = torch.as_tensor(
            self.action_space.low, dtype=torch.float32, device=self._device
        )
        high = torch.as_tensor(
            self.action_space.high, dtype=torch.float32, device=self._device
        )
        return low + (high - low) * torch.rand(
            self._num_envs, *self.action_space.shape, device=self._device
        )

    def render(self) -> np.ndarray | None:
        return self.env.render()

    def close(self) -> None:
        self.env.close()

    # ------------------------------------------------------------------
    # Observation helpers
    # ------------------------------------------------------------------

    def _process_obs(self, obs: Any) -> torch.Tensor:
        """Project assembled observations into a ``(B, state_size)`` tensor."""
        state = self._state_transform.transform(obs, device=self._device)
        if state.ndim == 1:
            state = state.reshape(1, -1)
        return state

    def state_from_observation(self, obs: Any) -> torch.Tensor:
        """Public seam for the policy/eval path (same transform as training)."""
        return self._process_obs(obs)


__all__ = ["STATE_KEY", "SrbTdmpc2Wrapper"]
