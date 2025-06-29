from functools import cached_property
from typing import Dict

import gymnasium
import numpy

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging


class DummyInterfaceCfg(HardwareInterfaceCfg):
    pass


class DummyInterface(HardwareInterface):
    cfg: DummyInterfaceCfg

    def __init__(self, cfg: DummyInterfaceCfg = DummyInterfaceCfg()):
        super().__init__(cfg)

        self.step_counter: int = 0
        self.should_reset: bool = False

    def start(self, **kwargs):
        super().start(**kwargs)

    def close(self):
        super().close()

    def sync(self):
        super().sync()

        self.step_counter += 1
        if self.should_reset:
            self.should_reset = False
            self.step_counter = 0

        self.obs = {
            "proprio/imu_ang_vel": numpy.array(
                (1.1, 1.2, 1.3),
                dtype=numpy.float32,
            ),
            "proprio/imu_lin_acc": numpy.array(
                (2.1, 2.2, 2.3),
                dtype=numpy.float32,
            ),
            "state/tf_pos_robot_to_target": numpy.array(
                (3.1, 3.2),
                dtype=numpy.float32,
            ),
            "state/vel_ang_robot": numpy.array(
                (4.1, 4.2, 4.3),
                dtype=numpy.float32,
            ),
            "state/vel_lin_robot": numpy.array(
                (5.1, 5.2, 5.3),
                dtype=numpy.float32,
            ),
        }
        self.rew = numpy.random.random()
        self.term = numpy.random.random() < (0.001 * self.step_counter)

    def reset(self):
        super().reset()
        self.should_reset = True

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "robot/wheeled_drive": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(2,), dtype=numpy.float32
                )
            }
        )

    @cached_property
    def action_scale_linear(self) -> float:
        return self._action_scale.get(
            "robot/wheeled_drive_linear"
        ) or self._action_scale.get("robot/wheeled_drive", 1.0)

    @cached_property
    def action_scale_angular(self) -> float:
        return self._action_scale.get(
            "robot/wheeled_drive_angular"
        ) or self._action_scale.get("robot/wheeled_drive", 1.0)

    def apply_action(self, action: Dict[str, numpy.ndarray]):
        assert "robot/wheeled_drive" in action.keys() and action[
            "robot/wheeled_drive"
        ].shape == (2,)

        act_linear = self.action_scale_linear * action["robot/wheeled_drive"][0]
        act_angular = self.action_scale_angular * action["robot/wheeled_drive"][1]

        logging.debug(
            f'[{self.name}] Applying action: {{"act_linear": {act_linear}, "act_angular": {act_angular}}}'
        )

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        return self.obs

    @property
    def reward(self) -> float:
        return self.rew

    @property
    def termination(self) -> bool:
        return self.term
