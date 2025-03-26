from typing import TYPE_CHECKING, Sequence, Tuple, Type

import torch
from pydantic import BaseModel

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils.cfg import configclass
from srb.utils.math import deg_to_rad

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation, RigidObject


class ThrustAction(ActionTerm):
    cfg: "ThrustActionCfg"
    _asset: "Articulation | RigidObject"

    def __init__(self, cfg: "ThrustActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore
        )

        self._num_thrusters = len(cfg.thrusters)
        thruster_offset = []
        thruster_direction = []
        thruster_power = []
        thruster_gimbal_limits = []
        for thruster_cfg in cfg.thrusters:
            thruster_offset.append(thruster_cfg.offset)
            direction_norm = (
                thruster_cfg.direction[0] ** 2
                + thruster_cfg.direction[1] ** 2
                + thruster_cfg.direction[2] ** 2
            ) ** 0.5
            assert direction_norm > 0, (
                "Thruster direction must have a non-zero magnitude"
            )
            direction = (
                thruster_cfg.direction[0] / direction_norm,
                thruster_cfg.direction[1] / direction_norm,
                thruster_cfg.direction[2] / direction_norm,
            )
            thruster_direction.append(direction)
            thruster_gimbal_limits.append(thruster_cfg.gimbal_limits)
            thruster_power.append(thruster_cfg.power)
        self._thruster_offset = torch.tensor(thruster_offset, device=env.device)
        self._thruster_direction = torch.tensor(thruster_direction, device=env.device)
        self._thruster_power = torch.tensor(thruster_power, device=env.device)
        self._thruster_gimbal_limits = torch.tensor(
            thruster_gimbal_limits, device=env.device
        )
        self._num_thrusters_with_gimbal = len(
            [limits for limits in thruster_gimbal_limits if limits is not None]
        )

        self._remaining_fuel = cfg.fuel_capacity * torch.ones(
            env.num_envs, device=env.device
        )

        self._action_indices_thrust = torch.arange(
            self._num_thrusters, device=env.device
        )
        action_indices_gimbal: Sequence[Tuple[int, int]] = []
        for thruster_idx, limits in enumerate(thruster_gimbal_limits):
            if limits is None:
                action_indices_gimbal.append((-1, -1))
            else:
                start_idx = self._num_thrusters + 2 * thruster_idx
                action_indices_gimbal.append((start_idx, start_idx + 1))
        self._action_indices_gimbal = torch.tensor(
            action_indices_gimbal, device=env.device
        )

    @property
    def action_dim(self) -> int:
        return self._num_thrusters + 2 * self._num_thrusters_with_gimbal

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    @property
    def remaining_fuel(self) -> torch.Tensor:
        return self._remaining_fuel

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = actions.clone()
        self._processed_actions[:, self._action_indices_thrust] = torch.clamp(
            self._processed_actions[:, self._action_indices_thrust], 0.0, 1.0
        )

    def apply_actions(self):
        combined_forces = torch.zeros(self.num_envs, 3, device=self.device)
        combined_torques = torch.zeros(self.num_envs, 3, device=self.device)
        combined_positions = torch.zeros(self.num_envs, 3, device=self.device)

        # TODO: Compute the resulting forces and torques from the thrusters
        # and apply them to the physics engine

        self._asset.root_physx_view.apply_forces_and_torques_at_position(
            force_data=combined_forces,
            torque_data=combined_torques,
            position_data=combined_positions,
            indices=self._asset._ALL_INDICES,
            is_global=False,
        )

    def reset_idx(self, env_ids):
        self._remaining_fuel[env_ids] = self.cfg.fuel_capacity


class ThrusterCfg(BaseModel):
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    direction: Tuple[float, float, float] = (0.0, 0.0, -1.0)
    gimbal_limits: Tuple[float, float] | None = None  # (x limit, y limit | symmetrical)
    power: float = 1.0


@configclass
class ThrustActionCfg(ActionTermCfg):
    class_type: Type = ThrustAction

    scale: float = 1.0

    thrusters: Sequence[ThrusterCfg] = (
        ThrusterCfg(
            offset=(0.0, 0.0, 0.0),
            direction=(0.0, 0.0, -1.0),
            power=1.0,
            gimbal_limits=(deg_to_rad(30.0), deg_to_rad(30.0)),
        ),
    )

    fuel_capacity: float = 10.0
    fuel_density: float = 1000.0
