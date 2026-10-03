"""Wheeled-drive actions with rate and smoothing filters.

Filter chain (no jerk limiter):
    raw_action → RateLimiter → CriticallyDampedFilter → WheeledDriveAction

``emitted_action`` exposes the post-filter tensor to observations and rewards.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

import torch
from isaaclab.managers.action_manager import ActionTerm

from srb.core.action.term.mobile.wheeled_drive import (
    WheeledDriveAction,
    WheeledDriveActionCfg,
)
from srb.core.action_filters import CriticallyDampedFilter, RateLimiter
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv


# Define the action class before its config so ``class_type`` captures it.


class FilteredWheeledDriveAction(WheeledDriveAction):
    """WheeledDriveAction that applies a 2-stage filter chain before the controller.

    Instantiates RateLimiter + CriticallyDampedFilter in __init__ and applies
    them inside process_actions before delegating to super().

    There is no JerkLimiter for drive; only rate + smooth filter.

    The post-filter tensor is exposed via emitted_action for:
      - Task observation: prev_action
      - Reward terms: penalty_action_rate
    """

    cfg: "FilteredWheeledDriveActionCfg"
    ACTION_DIM = 2

    def __init__(self, cfg: "FilteredWheeledDriveActionCfg", env: "AnyEnv"):
        super().__init__(cfg, env)

        device = env.device
        dt = env.step_dt

        if cfg.rate_limit_override is not None:
            rate_limits = torch.tensor(
                cfg.rate_limit_override, dtype=torch.float32, device=device
            )
        else:
            rate_limits = torch.tensor(
                [cfg.accel_lin_max * dt, cfg.accel_ang_max * dt],
                dtype=torch.float32,
                device=device,
            )

        self._rate = RateLimiter(
            (env.num_envs, self.ACTION_DIM),
            max_rate_per_step=rate_limits,
            device=device,
        )
        self._smooth_filter = CriticallyDampedFilter(
            (env.num_envs, self.ACTION_DIM),
            tau_s=cfg.filter_tau_s,
            dt_s=dt,
            device=device,
        )
        self._emitted = torch.zeros((env.num_envs, self.ACTION_DIM), device=device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        if env_ids is None:
            zeros = torch.zeros_like(self._smooth_filter.pos)
        else:
            zeros = torch.zeros_like(self._smooth_filter.pos[env_ids])
        self._rate.reset(env_ids, zeros)
        self._smooth_filter.reset(env_ids, zeros)
        # Prevent stale emitted actions from reaching post-reset observations.
        if env_ids is None:
            self._emitted.zero_()
        else:
            self._emitted[env_ids] = 0.0

    def process_actions(self, actions: torch.Tensor) -> None:
        scaled = self._scale_to_limits(actions)
        x = self._rate.step(scaled)
        x = self._smooth_filter.step(x)
        # ``x`` is already in physical units; avoid applying the parent's scale
        # again. Retain its optional hard velocity clamps below.
        self._raw_actions = actions
        self._processed_actions = x.clone()
        if self.cfg.max_linear_velocity is not None:
            self._processed_actions[:, 0].clamp_(
                -self.cfg.max_linear_velocity, self.cfg.max_linear_velocity
            )
        if self.cfg.max_angular_velocity is not None:
            self._processed_actions[:, 1].clamp_(
                -self.cfg.max_angular_velocity, self.cfg.max_angular_velocity
            )
        # Keep observations and rewards aligned with the clamped wheel command.
        self._emitted = self._processed_actions.clone()

    def _scale_to_limits(self, raw: torch.Tensor) -> torch.Tensor:
        """Map normalized [-1, 1] policy output to physical units and clamp."""
        limits = torch.tensor(
            [self.cfg.v_lin_max, self.cfg.v_ang_max],
            dtype=raw.dtype,
            device=raw.device,
        )
        return raw.clamp(-1.0, 1.0) * limits

    @property
    def emitted_action(self) -> torch.Tensor:
        """Post-filter action tensor of shape (num_envs, 2).

        Consumed by:
          - Task observation: prev_action
          - Reward terms: penalty_action_rate
        """
        return self._emitted


# Define the config after the action so ``class_type`` captures the right class.


@configclass
class FilteredWheeledDriveActionCfg(WheeledDriveActionCfg):
    """WheeledDriveActionCfg extended with sim-to-real pre-filter parameters.

    All fields from WheeledDriveActionCfg are inherited unchanged.
    The added filter fields configure the RateLimiter and CriticallyDampedFilter.
    """

    class_type: type[ActionTerm] = FilteredWheeledDriveAction

    # --- Velocity limits (physical-space clamping) ---
    v_lin_max: float = 1.0
    """Maximum linear velocity magnitude [m/s] applied by scale-to-limits."""
    v_ang_max: float = 1.0
    """Maximum angular velocity magnitude [rad/s] applied by scale-to-limits."""

    # --- Acceleration limits (derive rate_limit_per_step if not overridden) ---
    accel_lin_max: float = 2.0
    """Maximum linear acceleration [m/s²] used to derive rate_limit_per_step[0]."""
    accel_ang_max: float = 2.0
    """Maximum angular acceleration [rad/s²] used to derive rate_limit_per_step[1]."""

    # --- Filter parameters ---
    filter_tau_s: float = 0.1
    """Time constant for the critically-damped 2nd-order pre-filter [s]."""

    # --- Optional per-channel override ---
    rate_limit_override: list[float] | None = None
    """Max absolute change per step for [v_lin, v_ang]. None → [accel_lin_max*dt, accel_ang_max*dt]."""
