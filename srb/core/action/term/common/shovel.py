"""One-command shovel lift with optional measured-state bucket curl slaving."""

from __future__ import annotations

from math import radians
from typing import TYPE_CHECKING, Any, Sequence

import torch
from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg

from srb.core.scene_introspection import get_scene_articulation
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def coupled_curl_target(
    lift_position_m: torch.Tensor,
    *,
    lift_range_m: tuple[float, float],
    curl_range_rad: tuple[float, float],
) -> torch.Tensor:
    """Map measured lift position to the deterministic curl position target."""
    lift_lo, lift_hi = lift_range_m
    curl_closed, curl_open = curl_range_rad
    fraction = ((lift_position_m - lift_lo) / (lift_hi - lift_lo)).clamp(0.0, 1.0)
    return curl_open + fraction * (curl_closed - curl_open)


class CoupledShovelVelocityAction(ActionTerm):
    """Drive one lift velocity and optionally slave bucket curl to measured lift.

    With ``cfg.curl_joint_name`` set to ``None`` the term drives the lift joint
    alone (the Raph hardware blade); otherwise it also writes curl
    position/velocity targets derived from measured lift state.
    """

    cfg: "CoupledShovelVelocityActionCfg"
    ACTION_DIM = 1

    def __init__(self, cfg: "CoupledShovelVelocityActionCfg", env: "AnyEnv"):
        super().__init__(cfg, env)
        shape = (env.num_envs, self.ACTION_DIM)
        self._raw_actions = torch.zeros(shape, dtype=torch.float32, device=env.device)
        self._processed_actions = torch.zeros_like(self._raw_actions)
        self._curl_target = torch.zeros_like(self._raw_actions)
        self._joint_ids: tuple[list[int], list[int] | None] | None = None

    @property
    def action_dim(self) -> int:
        return self.ACTION_DIM

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    @property
    def emitted_action(self) -> torch.Tensor:
        return self._processed_actions

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        if env_ids is None:
            self._raw_actions.zero_()
            self._processed_actions.zero_()
            self._curl_target.zero_()
        else:
            self._raw_actions[env_ids] = 0.0
            self._processed_actions[env_ids] = 0.0
            self._curl_target[env_ids] = 0.0

    def process_actions(self, actions: torch.Tensor) -> None:
        self._raw_actions = actions
        self._processed_actions = (
            actions.clamp(-1.0, 1.0) * self.cfg.lift_velocity_max_m_s
        )

    def _resolve_joint_ids(
        self, articulation: Any
    ) -> tuple[list[int], list[int] | None]:
        # Resolve once; a regex joint lookup per physics substep is wasted work.
        if self._joint_ids is not None:
            return self._joint_ids
        lift_ids, _ = articulation.find_joints(self.cfg.lift_joint_name)
        if len(lift_ids) != 1:
            raise ValueError(
                "CoupledShovelVelocityAction expected exactly one lift joint named "
                f"'{self.cfg.lift_joint_name}', found {lift_ids}"
            )
        curl_ids = None
        if self.cfg.curl_joint_name is not None:
            curl_ids, _ = articulation.find_joints(self.cfg.curl_joint_name)
            if len(curl_ids) != 1:
                raise ValueError(
                    "CoupledShovelVelocityAction expected exactly one curl joint "
                    f"named '{self.cfg.curl_joint_name}', found {curl_ids}"
                )
        self._joint_ids = (lift_ids, curl_ids)
        return self._joint_ids

    def apply_actions(self) -> None:
        articulation = get_scene_articulation(self._env.scene, self.cfg.asset_name)
        lift_ids, curl_ids = self._resolve_joint_ids(articulation)
        if curl_ids is None:
            articulation.set_joint_velocity_target(
                self._processed_actions, joint_ids=lift_ids
            )
            return

        joint_pos = getattr(
            articulation.data.joint_pos, "torch", articulation.data.joint_pos
        )
        joint_vel = getattr(
            articulation.data.joint_vel, "torch", articulation.data.joint_vel
        )
        measured_lift = joint_pos[:, lift_ids]
        measured_lift_velocity = joint_vel[:, lift_ids]
        curl_range_rad = self.cfg.curl_range_rad
        if curl_range_rad is None:
            raise ValueError(
                "CoupledShovelVelocityAction curl_joint_name requires curl_range_rad"
            )
        self._curl_target = coupled_curl_target(
            measured_lift,
            lift_range_m=self.cfg.lift_range_m,
            curl_range_rad=curl_range_rad,
        )
        lift_span = self.cfg.lift_range_m[1] - self.cfg.lift_range_m[0]
        curl_span = curl_range_rad[0] - curl_range_rad[1]
        curl_velocity_target = measured_lift_velocity * (curl_span / lift_span)

        articulation.set_joint_velocity_target(
            self._processed_actions, joint_ids=lift_ids
        )
        articulation.set_joint_position_target(self._curl_target, joint_ids=curl_ids)
        articulation.set_joint_velocity_target(curl_velocity_target, joint_ids=curl_ids)


@configclass
class CoupledShovelVelocityActionCfg(ActionTermCfg):
    """Configuration for one public lift command and optional curl slaving.

    ``curl_joint_name`` and ``curl_range_rad`` are both ``None`` for a
    lift-only payload, or both set for measured-state curl slaving.
    """

    class_type: type[ActionTerm] = CoupledShovelVelocityAction
    lift_joint_name: str = "shovel_joint"
    curl_joint_name: str | None = "shovel_curl_joint"
    lift_range_m: tuple[float, float] = (0.0, 0.2)
    curl_range_rad: tuple[float, float] | None = (-radians(70.0), 0.0)
    lift_velocity_max_m_s: float = 0.04

    @property
    def scale(self) -> float:
        """Magnitude the normalized lift command is multiplied by.

        ``process_actions`` computes ``clamp(action, -1, 1) *
        lift_velocity_max_m_s``, so this *is* the term's action scale. It is
        exposed under the name every scale consumer already reads (the runtime
        action layout, and through it the sim-to-real contract check) because
        this term replaced a ``JointVelocityActionCfg`` whose ``scale`` field
        carried the same number.
        """
        return self.lift_velocity_max_m_s

    def __post_init__(self) -> None:
        lift_lo, lift_hi = self.lift_range_m
        if lift_lo >= lift_hi:
            raise ValueError("lift_range_m must be strictly increasing")
        if (self.curl_joint_name is None) != (self.curl_range_rad is None):
            raise ValueError(
                "curl_joint_name and curl_range_rad must both be set or both be None"
            )
        if self.curl_range_rad is not None:
            curl_closed, curl_open = self.curl_range_rad
            if curl_closed >= curl_open:
                raise ValueError(
                    "curl_range_rad must be (closed, open) with closed < open"
                )
        if self.lift_velocity_max_m_s <= 0.0:
            raise ValueError("lift_velocity_max_m_s must be positive")
        if not self.asset_name:
            self.asset_name = "payload"


__all__ = [
    "CoupledShovelVelocityAction",
    "CoupledShovelVelocityActionCfg",
    "coupled_curl_target",
]
