from enum import Enum, auto
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple, Union

import numpy as np
import torch
from gymnasium import ActionWrapper
from gymnasium.spaces import Box
from scipy.signal import butter, lfilter, lfilter_zi, savgol_filter

from srb.utils import logging

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class SmoothingMethod(Enum):
    SAVGOL = auto()
    MOVING_AVERAGE = auto()
    BUTTERWORTH = auto()


class FillMethod(Enum):
    ZERO = auto()
    FIRST_ACTION = auto()


class ActionSmoothingWrapper(ActionWrapper):
    def __init__(
        self,
        env: "AnyEnv",
        method: SmoothingMethod,
        history_len: Optional[int] = None,
        history_len_moving_average: int = 5,
        history_len_savgol: int = 9,
        poly_order: int = 3,
        cutoff_frequency_hz: Optional[float] = None,
        sample_rate_hz: Optional[float] = None,
    ):
        super().__init__(env)
        if not isinstance(env.action_space, Box):
            raise TypeError("ActionSmoothingWrapper only supports Box action spaces.")

        self.method = method
        self.history_len = history_len or (
            history_len_savgol
            if method == SmoothingMethod.SAVGOL
            else history_len_moving_average
        )
        self.poly_order = poly_order
        if sample_rate_hz is not None:
            self.sample_rate_hz = sample_rate_hz
        elif hasattr(env, "cfg") and hasattr(env.cfg, "agent_rate"):
            self.sample_rate_hz = 1.0 / env.cfg.agent_rate
        elif hasattr(env, "ACTION_RATE"):
            self.sample_rate_hz = 1.0 / env.ACTION_RATE  # type: ignore
        else:
            raise ValueError(
                "Sample rate must be provided or the environment must define ACTION_RATE or cfg.agent_rate."
            )
        self.cutoff_frequency_hz = cutoff_frequency_hz or (self.sample_rate_hz / 10.0)
        self._validate_params()

        self.is_initialized: bool = False
        self.action_buffer: np.ndarray
        self.buffer_idx: int
        self.filled_steps: int
        self.is_batched: bool
        self._fill_on_reset: FillMethod = FillMethod.ZERO
        self._butter_b: np.ndarray
        self._butter_a: np.ndarray
        self._butter_zi_list: List[np.ndarray] = []

        self.action_shape = self.action_space.shape
        self._savgol_delay = self.history_len // 2
        self._roll_indices = np.arange(self.history_len)

    def _validate_params(self):
        if self.method == SmoothingMethod.BUTTERWORTH:
            if self.sample_rate_hz is None or self.cutoff_frequency_hz is None:
                raise ValueError(
                    "`sample_rate_hz` and `cutoff_frequency_hz` are required for BUTTERWORTH."
                )
            if self.cutoff_frequency_hz >= self.sample_rate_hz / 2:
                raise ValueError(
                    "`cutoff_frequency_hz` must be less than half the `sample_rate_hz` (Nyquist limit)."
                )
            self._butter_b, self._butter_a = butter(  # type: ignore
                4, self.cutoff_frequency_hz, fs=self.sample_rate_hz, btype="low"
            )
        elif self.method == SmoothingMethod.SAVGOL:
            if self.poly_order is None or self.poly_order >= self.history_len:
                raise ValueError(
                    "`poly_order` must be less than `history_len` for SAVGOL."
                )
            if self.history_len % 2 == 0:
                raise ValueError("For SAVGOL, `history_len` must be an odd integer.")

    def _initialize_state(self, first_action: np.ndarray):
        self.is_batched = first_action.ndim > len(self.action_shape)  # type: ignore
        buffer_shape = (self.history_len,) + first_action.shape

        fill_val = (
            np.zeros_like(first_action)
            if self._fill_on_reset == FillMethod.ZERO
            else first_action
        )
        self.filled_steps = (
            0 if self._fill_on_reset == FillMethod.ZERO else self.history_len
        )
        self.action_buffer = np.broadcast_to(fill_val, buffer_shape).copy()

        if self.method == SmoothingMethod.BUTTERWORTH:
            action_flat = first_action.flatten()
            self._butter_zi_list = []
            for i in range(len(action_flat)):
                zi = lfilter_zi(self._butter_b, self._butter_a) * action_flat[i]
                self._butter_zi_list.append(zi)

        self.buffer_idx = 0
        self.is_initialized = True

    def action(
        self, action: Union[np.ndarray, torch.Tensor]
    ) -> Union[np.ndarray, torch.Tensor]:
        is_torch = isinstance(action, torch.Tensor)
        original_device = action.device if is_torch else None
        numpy_action = action.cpu().numpy() if is_torch else action

        if not self.is_initialized:
            self._initialize_state(numpy_action)

        smoothed_action = self._apply_smoothing(numpy_action)

        if is_torch:
            return torch.from_numpy(smoothed_action).to(device=original_device)
        else:
            return smoothed_action.astype(self.action_space.dtype)

    def _apply_smoothing(self, action: np.ndarray) -> np.ndarray:
        if self.method == SmoothingMethod.BUTTERWORTH:
            original_shape = action.shape
            action_flat = action.flatten()
            smoothed_flat = np.zeros_like(action_flat)
            for i in range(len(action_flat)):
                smoothed_flat[i], self._butter_zi_list[i] = lfilter(
                    self._butter_b,
                    self._butter_a,
                    [action_flat[i]],
                    zi=self._butter_zi_list[i],
                )
            return smoothed_flat.reshape(original_shape)

        self.action_buffer[self.buffer_idx] = action
        self.buffer_idx = (self.buffer_idx + 1) % self.history_len
        if self.filled_steps < self.history_len:
            self.filled_steps += 1
            return action

        indices = (self.buffer_idx + self._roll_indices) % self.history_len
        ordered_buffer = self.action_buffer[indices]

        if self.method == SmoothingMethod.MOVING_AVERAGE:
            return np.mean(ordered_buffer, axis=0)
        if self.method == SmoothingMethod.SAVGOL:
            return savgol_filter(
                ordered_buffer, self.history_len, self.poly_order, axis=0
            )[self._savgol_delay]

        raise ValueError(f"Unknown/unsupported smoothing method: {self.method}")

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
        fill_value: FillMethod = FillMethod.FIRST_ACTION,
    ) -> Tuple[np.ndarray, dict]:
        obs, info = super().reset(seed=seed, options=options)
        self._fill_on_reset = fill_value
        self.is_initialized = False
        return obs, info


def maybe_wrap_action_smoothing(
    env: "AnyEnv", smoothing_cfg: Dict[str, Any]
) -> "AnyEnv":
    if not smoothing_cfg or not smoothing_cfg.get("enabled", False):
        logging.debug("Action smoothing is disabled.")
        return env

    params = smoothing_cfg.copy()
    params.pop("enabled", None)

    method_enum = SmoothingMethod[params.pop("method").upper()]

    logging.info(
        f"Action smoothing is enabled. Method: {method_enum.name}, Config: {params}"
    )

    return ActionSmoothingWrapper(env, method=method_enum, **params)  # type: ignore
