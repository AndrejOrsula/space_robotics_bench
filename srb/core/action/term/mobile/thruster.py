from typing import TYPE_CHECKING, Tuple, Type

import torch

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils.cfg import configclass
from srb.utils.math import deg_to_rad

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation, RigidObject


# TODO[mid]: Implement ThrusterAction


class ThrusterAction(ActionTerm):
    cfg: "ThrusterActionCfg"
    _asset: "Articulation | RigidObject"

    def __init__(self, cfg: "ThrusterActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore
        )

    @property
    def action_dim(self) -> int:
        if self.cfg.gimbal:
            return 1 + 2
        else:
            return 1

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = self._raw_actions
        self._processed_actions[:, 0] *= self.cfg.scale

    def apply_actions(self):
        thrust_vector = self.cfg.nominal_thrust_vector

        pass


@configclass
class ThrusterActionCfg(ActionTermCfg):
    class_type: Type = ThrusterAction

    scale: float = 1.0

    nominal_thrust_vector: Tuple[float, float, float] = (0.0, 0.0, -1.0)

    gimbal: bool = True
    gimbal_limits: Tuple[Tuple[float, float], Tuple[float, float]] = (
        (deg_to_rad(-30.0), deg_to_rad(30.0)),
        (deg_to_rad(-30.0), deg_to_rad(30.0)),
    )

    fuel_capacity: float = 10.0
    fuel_density: float = 1000.0
    fuel_consumption_rate: float = 0.1
