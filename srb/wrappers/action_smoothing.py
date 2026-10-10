from typing import TYPE_CHECKING

import numpy as np
import torch
from gymnasium import ActionWrapper

from srb.core.env.introspect import runtime_num_envs
from srb.core.env.wrapper_cfg import (
    ActionSmoothingCfg,
    FillMethod,
    SmoothingMethod,
)
from srb.core.spaces import is_box
from srb.utils import logging
from srb.wrappers._utils import get_wrapper_attr as _get_wrapper_attr

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class ActionSmoothingWrapper(ActionWrapper):
    """Smooth actions on their input device; NumPy inputs return as NumPy."""

    def __init__(self, env: "AnyEnv", cfg: ActionSmoothingCfg):
        super().__init__(env)
        if not is_box(env.action_space):
            raise TypeError("ActionSmoothingWrapper only supports Box action spaces.")

        # Preserve effective spaces for downstream vector integrations.
        single_observation_space = _get_wrapper_attr(env, "single_observation_space")
        if single_observation_space is not None:
            self.single_observation_space = single_observation_space
        single_action_space = _get_wrapper_attr(env, "single_action_space")
        if single_action_space is not None:
            self.single_action_space = single_action_space

        self.cfg = cfg
        self._num_envs = runtime_num_envs(env)

        # Resolve the sample rate.
        if cfg.sample_frequency is not None:
            self.sample_rate_hz = cfg.sample_frequency
        elif hasattr(env.unwrapped, "cfg") and hasattr(env.unwrapped.cfg, "agent_rate"):
            self.sample_rate_hz = 1.0 / env.unwrapped.cfg.agent_rate
        elif hasattr(env.unwrapped, "ACTION_RATE"):
            self.sample_rate_hz = 1.0 / env.unwrapped.ACTION_RATE  # type: ignore[attr-defined]
        else:
            raise ValueError(
                "Cannot infer the sample rate; set `sample_frequency` or expose `ACTION_RATE` "
                "or `cfg.agent_rate` on the environment."
            )

        # Derive filter parameters from ``smoothing_duration``.
        match self.cfg.method:
            case SmoothingMethod.BUTTERWORTH:
                from scipy.signal import butter, lfilter_zi

                self.cutoff_frequency_hz: float = 1.0 / cfg.smoothing_duration
                if self.cutoff_frequency_hz >= self.sample_rate_hz / 2.0:
                    raise ValueError(
                        f"Derived cutoff frequency ({self.cutoff_frequency_hz:.2f} Hz) meets or exceeds "
                        f"Nyquist limit ({self.sample_rate_hz / 2:.2f} Hz). "
                        f"Increase `smoothing_duration` (current: {self.cfg.smoothing_duration}s)."
                    )
                b, a = butter(
                    4, self.cutoff_frequency_hz, fs=self.sample_rate_hz, btype="low"
                )
                self._butter_b = np.asarray(b, dtype=np.float64)
                self._butter_a = np.asarray(a, dtype=np.float64)
                # Scale unit-step steady-state conditions by the initial action.
                self._butter_zi_unit = np.asarray(lfilter_zi(b, a), dtype=np.float64)
                self._butter_state: torch.Tensor | None = None

            case SmoothingMethod.MOVING_AVERAGE | SmoothingMethod.SAVGOL:
                self.history_len = int(
                    round(cfg.smoothing_duration * self.sample_rate_hz)
                )
                if self.history_len <= 1:
                    raise ValueError(
                        f"Invalid history_len: {self.history_len}. Increase `smoothing_duration` (current: {self.cfg.smoothing_duration}s)."
                    )

                if self.cfg.method == SmoothingMethod.SAVGOL:
                    from scipy.signal import savgol_coeffs

                    if self.history_len % 2 == 0:
                        self.history_len += 1
                    if self.history_len <= self.cfg.poly_order:
                        raise ValueError(
                            f"Invalid history_len: {self.history_len} for poly_order: {self.cfg.poly_order}. "
                            f"Increase `smoothing_duration` (current: {self.cfg.smoothing_duration}s)."
                        )
                    self._savgol_delay = self.history_len // 2
                    # Order centre-window coefficients oldest to newest.
                    self._savgol_coeffs = np.asarray(
                        savgol_coeffs(
                            self.history_len,
                            self.cfg.poly_order,
                            pos=self._savgol_delay,
                            use="dot",
                        ),
                        dtype=np.float64,
                    )

                self.action_buffer: torch.Tensor
                # Per-env count of written history slots.
                self.filled_steps: torch.Tensor
                self.buffer_idx: int

        self.is_initialized: bool = False
        # Defer per-env history resets until the next action after autoreset.
        self._pending_reset: torch.Tensor | None = None

    # ------------------------------------------------------------------ helpers
    def _as_tensor(self, action: np.ndarray | torch.Tensor) -> torch.Tensor:
        if isinstance(action, torch.Tensor):
            return action.detach()
        return torch.as_tensor(np.asarray(action))

    def _coefficients(self, like: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        if (
            getattr(self, "_butter_b_t", None) is None
            or self._butter_b_t.device != like.device
        ):
            self._butter_b_t = torch.as_tensor(
                self._butter_b, dtype=torch.float64, device=like.device
            )
            self._butter_a_t = torch.as_tensor(
                self._butter_a, dtype=torch.float64, device=like.device
            )
            self._butter_zi_t = torch.as_tensor(
                self._butter_zi_unit, dtype=torch.float64, device=like.device
            )
        return self._butter_b_t, self._butter_a_t

    def _initialize_state(self, first_action: np.ndarray | torch.Tensor):
        action = self._as_tensor(first_action)
        self._is_batched_action = action.ndim >= 2 or (
            self._num_envs > 0
            and action.ndim >= 1
            and action.shape[0] == self._num_envs
        )

        match self.cfg.method:
            case SmoothingMethod.BUTTERWORTH:
                self._coefficients(action)
                flat = action.reshape(-1).to(torch.float64)
                # State shape: (n_elems, order).
                self._butter_state = flat[:, None] * self._butter_zi_t[None, :]

            case SmoothingMethod.MOVING_AVERAGE | SmoothingMethod.SAVGOL:
                seed = (
                    torch.zeros_like(action)
                    if self.cfg.fill_on_reset == FillMethod.ZERO
                    else action
                )
                self.action_buffer = (
                    seed.unsqueeze(0).expand(self.history_len, *action.shape).clone()
                )
                self.filled_steps = torch.full(
                    action.shape[:1] if self._is_batched_action else (),
                    0
                    if self.cfg.fill_on_reset == FillMethod.ZERO
                    else self.history_len,
                    dtype=torch.long,
                    device=action.device,
                )
                self.buffer_idx = 0

    @staticmethod
    def _per_env(value: torch.Tensor, like: torch.Tensor) -> torch.Tensor:
        """Broadcast a per-env (or scalar) value over the action's trailing axes."""
        return value.reshape(value.shape + (1,) * (like.ndim - value.ndim))

    def action(self, action: np.ndarray | torch.Tensor) -> np.ndarray | torch.Tensor:
        if not self.is_initialized:
            self._initialize_state(action)
            self.is_initialized = True
        else:
            # Seed finished envs with their new episode's first action.
            self._apply_episode_resets(action)

        tensor = self._as_tensor(action)
        smoothed = self._apply_smoothing(tensor)
        if isinstance(action, torch.Tensor):
            return smoothed.to(dtype=action.dtype)
        return smoothed.cpu().numpy().astype(self.action_space.dtype)

    def _apply_smoothing(self, action: torch.Tensor) -> torch.Tensor:
        match self.cfg.method:
            case SmoothingMethod.BUTTERWORTH:
                b, a = self._coefficients(action)
                assert self._butter_state is not None
                original_shape = action.shape
                x = action.reshape(-1).to(torch.float64)
                z = self._butter_state
                # Apply one direct-form II transposed sample to all elements.
                y = b[0] * x + z[:, 0]
                order = z.shape[1]
                new_z = torch.empty_like(z)
                for i in range(order - 1):
                    new_z[:, i] = b[i + 1] * x + z[:, i + 1] - a[i + 1] * y
                new_z[:, order - 1] = b[order] * x - a[order] * y
                self._butter_state = new_z
                return y.reshape(original_shape).to(action.dtype)

            case SmoothingMethod.MOVING_AVERAGE:
                self.action_buffer[self.buffer_idx] = action
                self.buffer_idx = (self.buffer_idx + 1) % self.history_len
                self.filled_steps = (self.filled_steps + 1).clamp(max=self.history_len)
                # Divide by written slots, or by the full window for FIRST_ACTION.
                count = self._per_env(self.filled_steps, action)
                return self.action_buffer.sum(dim=0) / count.to(
                    self.action_buffer.dtype
                )

            case SmoothingMethod.SAVGOL:
                self.action_buffer[self.buffer_idx] = action
                self.buffer_idx = (self.buffer_idx + 1) % self.history_len
                self.filled_steps = (self.filled_steps + 1).clamp(max=self.history_len)
                # Pass through until each env has a full window.
                ready = self._per_env(self.filled_steps >= self.history_len, action)
                order = (
                    torch.arange(self.history_len, device=action.device)
                    + self.buffer_idx
                ) % self.history_len
                window = self.action_buffer[order].to(torch.float64)
                coeffs = torch.as_tensor(
                    self._savgol_coeffs, dtype=torch.float64, device=action.device
                )
                shape = (self.history_len,) + (1,) * (window.ndim - 1)
                filtered = (window * coeffs.reshape(shape)).sum(dim=0).to(action.dtype)
                return torch.where(ready, filtered, action)

        raise RuntimeError("Unreachable")

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        # Clear finished env histories on the next action, using its first sample.
        self._pending_reset = self._done_mask(terminated, truncated)
        return obs, reward, terminated, truncated, info

    def _done_mask(self, terminated, truncated) -> torch.Tensor | None:
        def _as_bool(x) -> torch.Tensor | None:
            if isinstance(x, torch.Tensor):
                return x.detach().reshape(-1).to(torch.bool)
            if isinstance(x, np.ndarray):
                return torch.as_tensor(x.reshape(-1).astype(bool))
            return None

        mask: torch.Tensor | None = None
        for part in (_as_bool(terminated), _as_bool(truncated)):
            if part is None:
                continue
            mask = part if mask is None else (mask | part.to(mask.device))
        return mask

    def _apply_episode_resets(self, action: np.ndarray | torch.Tensor) -> None:
        mask = self._pending_reset
        self._pending_reset = None
        if mask is None or not bool(mask.any()):
            return
        a = self._as_tensor(action)
        # Per-env reset requires a batched action with a matching env axis.
        if not self._is_batched_action or a.ndim < 1 or mask.shape[0] != a.shape[0]:
            return
        idx = torch.nonzero(mask.to(a.device)).reshape(-1)
        fill_zero = self.cfg.fill_on_reset == FillMethod.ZERO
        match self.cfg.method:
            case SmoothingMethod.BUTTERWORTH:
                assert self._butter_state is not None
                per_env = a.reshape(a.shape[0], -1).to(torch.float64)
                width = per_env.shape[1]
                seeds = (
                    torch.zeros_like(per_env[idx]) if fill_zero else per_env[idx]
                )  # (n_reset, width)
                element_rows = (
                    idx[:, None] * width + torch.arange(width, device=a.device)[None, :]
                ).reshape(-1)
                self._butter_state[element_rows] = (
                    seeds.reshape(-1)[:, None] * self._butter_zi_t[None, :]
                )
            case SmoothingMethod.MOVING_AVERAGE | SmoothingMethod.SAVGOL:
                # Replace each finished env's history; ZERO fill restarts its count.
                fill = torch.zeros_like(a[idx]) if fill_zero else a[idx]
                self.action_buffer[:, idx] = fill.to(self.action_buffer.dtype)
                if fill_zero:
                    self.filled_steps[idx.to(self.filled_steps.device)] = 0

    def reset(self, *args, **kwargs) -> tuple[np.ndarray, dict]:
        self.is_initialized = False
        self._pending_reset = None
        return super().reset(*args, **kwargs)

    @classmethod
    def wrap(
        cls,
        env: "AnyEnv",
        cfg: ActionSmoothingCfg,
    ) -> "AnyEnv":
        if not cfg.enable:
            logging.debug("Action smoothing is disabled.")
            return env

        logging.info(f"Action smoothing enabled: {cfg}")
        return cls(env, cfg)


__all__ = (
    "ActionSmoothingCfg",
    "ActionSmoothingWrapper",
    "FillMethod",
    "SmoothingMethod",
)
