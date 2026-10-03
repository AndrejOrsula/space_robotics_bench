"""Joint-position actions with rate, smoothing, and jerk filters.

Filter chain:
    raw_action → RateLimiter → CriticallyDampedFilter → JerkLimiter
               → JointPositionAction

``emitted_action`` exposes the post-filter tensor to observations and rewards.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

import torch
from isaaclab.envs.mdp.actions.actions_cfg import (
    JointPositionActionCfg as _JointPositionActionCfg,
)
from isaaclab.envs.mdp.actions.actions_cfg import (
    RelativeJointPositionActionCfg as _RelativeJointPositionActionCfg,
)
from isaaclab.envs.mdp.actions.joint_actions import (
    JointPositionAction as _JointPositionAction,
)
from isaaclab.envs.mdp.actions.joint_actions import (
    RelativeJointPositionAction as _RelativeJointPositionAction,
)
from isaaclab.managers.action_manager import ActionTerm

from srb.core.action_filters import CriticallyDampedFilter, JerkLimiter, RateLimiter
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv


# Define action classes before their configs so ``class_type`` captures the
# correct class in its default.


class FilteredJointPositionAction(_JointPositionAction):
    """JointPositionAction that applies a 3-stage filter chain before writing targets.

    Instantiates RateLimiter + CriticallyDampedFilter + JerkLimiter in __init__
    and applies them inside process_actions before delegating to super().

    The post-filter tensor is exposed via emitted_action for:
      - Task observation: prev_action
      - Reward terms: penalty_action_rate, penalty_jerk
    """

    cfg: "FilteredJointPositionActionCfg"

    def __init__(self, cfg: "FilteredJointPositionActionCfg", env: "AnyEnv"):
        super().__init__(cfg, env)

        action_dim = self.action_dim
        device = env.device

        self._rate = RateLimiter(
            (env.num_envs, action_dim),
            max_rate_per_step=cfg.rate_limit_per_step,
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
            # Prevent the first post-reset observation from seeing stale action.
            self._emitted.zero_()
        else:
            zeros = torch.zeros_like(self._smooth_filter.pos[env_ids])
            self._emitted[env_ids] = 0.0
        self._rate.reset(env_ids, zeros)
        self._smooth_filter.reset(env_ids, zeros)
        self._jerk.reset(env_ids, zeros, None)

    def process_actions(self, actions: torch.Tensor) -> None:
        x = self._rate.step(actions)
        x = self._smooth_filter.step(x)
        x = self._jerk.step(x)
        self._emitted = x
        super().process_actions(x)

    @property
    def emitted_action(self) -> torch.Tensor:
        """Post-filter action tensor of shape (num_envs, num_joints).

        Consumed by:
          - Task observation: prev_action
          - Reward terms: penalty_action_rate, penalty_jerk
        """
        return self._emitted


class FilteredRelativeJointPositionAction(_RelativeJointPositionAction):
    """RelativeJointPositionAction with the same 3-stage filter chain."""

    cfg: "FilteredRelativeJointPositionActionCfg"

    def __init__(self, cfg: "FilteredRelativeJointPositionActionCfg", env: "AnyEnv"):
        super().__init__(cfg, env)

        action_dim = self.action_dim
        device = env.device

        self._rate = RateLimiter(
            (env.num_envs, action_dim),
            max_rate_per_step=cfg.rate_limit_per_step,
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
            # Prevent the first post-reset observation from seeing stale action.
            self._emitted.zero_()
        else:
            zeros = torch.zeros_like(self._smooth_filter.pos[env_ids])
            self._emitted[env_ids] = 0.0
        self._rate.reset(env_ids, zeros)
        self._smooth_filter.reset(env_ids, zeros)
        self._jerk.reset(env_ids, zeros, None)

    def process_actions(self, actions: torch.Tensor) -> None:
        x = self._rate.step(actions)
        x = self._smooth_filter.step(x)
        x = self._jerk.step(x)
        self._emitted = x
        super().process_actions(x)

    @property
    def emitted_action(self) -> torch.Tensor:
        return self._emitted


# Config classes follow their actions so ``class_type`` captures the right class.


@configclass
class FilteredJointPositionActionCfg(_JointPositionActionCfg):
    """JointPositionActionCfg extended with sim-to-real pre-filter parameters.

    All fields from JointPositionActionCfg are inherited unchanged.
    The added filter fields configure the RateLimiter, CriticallyDampedFilter,
    and JerkLimiter chain.
    """

    class_type: type[ActionTerm] = FilteredJointPositionAction

    # --- Filter parameters ---
    pre_filter_tau_s: float = 0.1
    """Time constant for the critically-damped 2nd-order pre-filter [s]."""
    rate_limit_per_step: float = 0.1
    """Max absolute change per step from the RateLimiter, in the units of the
    incoming action (per step). The filter chain runs before the inherited
    ``scale``/``offset``, so this equals rad/step only when ``scale`` is 1."""
    jerk_limit_per_step: float = 50.0
    """Max jerk budget per step for the JerkLimiter, in incoming-action units
    (before ``scale``)."""


@configclass
class FilteredRelativeJointPositionActionCfg(_RelativeJointPositionActionCfg):
    """RelativeJointPositionActionCfg extended with pre-filter parameters."""

    class_type: type[ActionTerm] = FilteredRelativeJointPositionAction

    pre_filter_tau_s: float = 0.1
    rate_limit_per_step: float = 0.1
    jerk_limit_per_step: float = 50.0
