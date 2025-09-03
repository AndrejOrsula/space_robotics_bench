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
            "state/tf_pos2d_robot_to_target": numpy.array(
                (1.0, 9.0),
                dtype=numpy.float32,
            ),
            "state/tf_rot2dtrigyaw_robot_to_target": numpy.array(
                (0.2, 0.8),
                dtype=numpy.float32,
            ),
            "proprio/fk_pos_end_effector": numpy.random.rand(3).astype(numpy.float32),
            "proprio/fk_rot6d_end_effector": numpy.random.rand(6).astype(numpy.float32),
            "proprio_dyn/joint_pos_robot_normalized": numpy.random.rand(7).astype(
                numpy.float32
            ),
            "image_wrist": (numpy.random.rand(128, 128, 1) * 255).astype(numpy.uint8),
            "image_base": (numpy.random.rand(128, 128, 1) * 255).astype(numpy.uint8),
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
                ),
                "robot/operational_space_controller": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(18,), dtype=numpy.float32
                ),
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
        logging.debug(f"[{self.name}] Applying action: {action}")

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        return self.obs

    @property
    def reward(self) -> float:
        return self.rew

    @property
    def termination(self) -> bool:
        return self.term
