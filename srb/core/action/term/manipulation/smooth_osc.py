"""OSC action term with rate, critically damped, and jerk filters.

The ActionGroup wrapper is in ``srb/core/action/group/manipulation/smooth_osc.py``.
``emitted_action`` exposes the filtered command to observations and rewards.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

import torch
from isaaclab.managers.action_manager import ActionTerm

from srb.core.action.term.manipulation.osc import (
    OperationalSpaceControllerAction,
    OperationalSpaceControllerActionCfg,
)
from srb.core.action_filters import CriticallyDampedFilter, JerkLimiter, RateLimiter
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv


# ---------------------------------------------------------------------------
# Define the action first: configclass captures ``class_type`` at decoration
# time, so a later assignment would leave its default factory set to ``None``.
# ---------------------------------------------------------------------------


class SmoothOperationalSpaceControllerAction(OperationalSpaceControllerAction):
    """Apply rate, critically damped, and jerk filters before OSC control."""

    cfg: "SmoothOperationalSpaceControllerActionCfg"

    def __init__(self, cfg: "SmoothOperationalSpaceControllerActionCfg", env: "AnyEnv"):
        super().__init__(cfg, env)

        # Pose channels are scaled by _scale_to_limits; neutralize OSC's second
        # pose scaling. Stiffness/wrench channels remain scaled by the base term.
        self._position_scale = torch.ones_like(self._position_scale)
        self._orientation_scale = torch.ones_like(self._orientation_scale)

        action_dim = self.action_dim
        device = env.device

        max_rate = (
            cfg.rate_limit_per_step if cfg.rate_limit_per_step is not None else 1.0
        )

        self._rate = RateLimiter(
            (env.num_envs, action_dim),
            max_rate_per_step=max_rate,
            device=device,
        )
        self._smooth_filter = CriticallyDampedFilter(
            (env.num_envs, action_dim),
            tau_s=cfg.pre_filter_tau_s,
            dt_s=env.step_dt,
            device=device,
        )
        self._jerk = JerkLimiter(
            (env.num_envs, action_dim),
            max_jerk_per_step=cfg.jerk_limit_per_step,
            dt_s=env.step_dt,
            device=device,
        )
        self._emitted = torch.zeros((env.num_envs, action_dim), device=device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        if env_ids is None:
            zeros = torch.zeros_like(self._smooth_filter.pos)
        else:
            zeros = torch.zeros_like(self._smooth_filter.pos[env_ids])
        self._rate.reset(env_ids, zeros)
        self._smooth_filter.reset(env_ids, zeros)
        self._jerk.reset(env_ids, zeros, None)
        # Prevent stale prev_action observations and action-rate penalties.
        if env_ids is None:
            self._emitted.zero_()
        else:
            self._emitted[env_ids] = 0.0

    def process_actions(self, actions: torch.Tensor) -> None:
        # Apply physical-space clamp before filters.
        actions = self._scale_to_limits(actions)
        x = self._rate.step(actions)
        x = self._smooth_filter.step(x)
        x = self._jerk.step(x)
        self._emitted = x
        super().process_actions(x)

    def _scale_to_limits(self, raw: torch.Tensor) -> torch.Tensor:
        """Map normalized [-1, 1] policy output to physical units and clamp."""
        dim = raw.shape[-1]
        limits = torch.empty(dim, device=raw.device, dtype=raw.dtype)
        limits[:3] = self.cfg.delta_pos_limit_m
        limits[3:6] = self.cfg.delta_rot_limit_rad
        if dim > 6:
            # Stiffness channels: pass through as unit scale.
            limits[6:] = 1.0
        return raw.clamp(-1.0, 1.0) * limits

    @property
    def emitted_action(self) -> torch.Tensor:
        """Post-filter action tensor of shape (num_envs, action_dim).

        Consumed by:
          - Task observation: prev_action
          - Reward terms: penalty_action_rate, penalty_jerk
        """
        return self._emitted


# ---------------------------------------------------------------------------
# Define the config after the action so configclass captures its class directly.
# ---------------------------------------------------------------------------


@configclass
class SmoothOperationalSpaceControllerActionCfg(OperationalSpaceControllerActionCfg):
    """OSC config with pre-filter and physical-space clamp parameters."""

    class_type: type[ActionTerm] = SmoothOperationalSpaceControllerAction

    # --- Pre-filter parameters ---
    pre_filter_tau_s: float = 0.08
    """Time constant for the critically-damped 2nd-order pre-filter [s]."""
    rate_limit_per_step: float | None = None
    """Max absolute change per step for the RateLimiter. None → 1.0 default."""
    jerk_limit_per_step: float = 100.0
    """Max jerk budget per step for the JerkLimiter."""

    # --- Physical-space clamping ---
    delta_pos_limit_m: float = 0.05
    """Symmetric clamp on each Cartesian-position delta channel [m]."""
    delta_rot_limit_rad: float = 0.1
    """Symmetric clamp on each axis-angle delta channel [rad]."""
