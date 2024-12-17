from collections.abc import Sequence
from dataclasses import MISSING

import torch
from omni.isaac.lab.managers import ActionTerm, ActionTermCfg
from omni.isaac.lab.utils import configclass

from space_robotics_bench.core.assets import Articulation
from space_robotics_bench.core.envs import BaseEnv


class SpacecraftAction(ActionTerm):
    cfg: "SpacecraftActionCfg"
    _asset: Articulation

    def __init__(self, cfg: "SpacecraftActionCfg", env: BaseEnv):
        super().__init__(cfg, env)

    @property
    def action_dim(self) -> int:
        return 6

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = self.raw_actions * self.cfg.scale

    def apply_actions(self):
        current_velocity = self._asset._data.body_vel_w[:, 0].squeeze(1)

        applied_velocities = current_velocity + self.processed_actions
        self._asset.write_root_velocity_to_sim(applied_velocities)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        pass


@configclass
class SpacecraftActionCfg(ActionTermCfg):
    class_type: ActionTerm = SpacecraftAction
    scale: float = 1.0


@configclass
class SpacecraftActionGroupCfg:
    flight: SpacecraftActionCfg = MISSING
