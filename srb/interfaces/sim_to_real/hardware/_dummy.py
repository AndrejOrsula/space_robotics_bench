from functools import cached_property

import gymnasium
import numpy as np

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.utils import logging


class DummyInterfaceCfg(HardwareInterfaceCfg):
    pass


class DummyInterface(HardwareInterface):
    cfg: DummyInterfaceCfg
    SUPPORTS_REWARD = True
    SUPPORTS_TERMINATION = True

    def __init__(self, cfg: DummyInterfaceCfg | None = None):
        if cfg is None:
            cfg = DummyInterfaceCfg()
        super().__init__(cfg)

        self.step_counter: int = 0
        self.should_reset: bool = False
        self.obs = {
            "state/tf_pos2d_robot_to_target": np.zeros(2, dtype=np.float32),
            "state/tf_rot2dtrigyaw_robot_to_target": np.zeros(2, dtype=np.float32),
            "proprio/fk_pos_end_effector": np.zeros(3, dtype=np.float32),
            "proprio/fk_rot6d_end_effector": np.zeros(6, dtype=np.float32),
            "proprio_dyn/joint_pos_robot_normalized": np.zeros(7, dtype=np.float32),
            "image_wrist": np.zeros((128, 128, 1), dtype=np.uint8),
            "image_base": np.zeros((128, 128, 1), dtype=np.uint8),
        }
        self.rew = 0.0
        self.term = False

    def sync(self):
        super().sync()

        self.step_counter += 1
        if self.should_reset:
            self.should_reset = False
            self.step_counter = 0

        self.obs = {
            "state/tf_pos2d_robot_to_target": np.array(
                (1.0, 9.0),
                dtype=np.float32,
            ),
            "state/tf_rot2dtrigyaw_robot_to_target": np.array(
                (0.2, 0.8),
                dtype=np.float32,
            ),
            "proprio/fk_pos_end_effector": np.random.rand(3).astype(np.float32),
            "proprio/fk_rot6d_end_effector": np.random.rand(6).astype(np.float32),
            "proprio_dyn/joint_pos_robot_normalized": np.random.rand(7).astype(
                np.float32
            ),
            "image_wrist": (np.random.rand(128, 128, 1) * 255).astype(np.uint8),
            "image_base": (np.random.rand(128, 128, 1) * 255).astype(np.uint8),
        }
        self.rew = np.random.random()
        self.term = np.random.random() < (0.001 * self.step_counter)

    def reset(self):
        super().reset()
        self.should_reset = True

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "robot/cmd_vel": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(2,), dtype=np.float32
                ),
                "robot/operational_space_controller": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(18,), dtype=np.float32
                ),
            }
        )

    @cached_property
    def action_scale_linear(self) -> float:
        return self._action_scale.get("robot/cmd_vel_linear") or self._action_scale.get(
            "robot/cmd_vel", 1.0
        )

    @cached_property
    def action_scale_angular(self) -> float:
        return self._action_scale.get(
            "robot/cmd_vel_angular"
        ) or self._action_scale.get("robot/cmd_vel", 1.0)

    def apply_action(self, action: dict[str, np.ndarray]):
        logging.debug(f"[{self.name}] Applying action: {action}")

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(getattr(self, "obs", {}))

    @property
    def reward(self) -> float:
        return self.rew

    @property
    def termination(self) -> bool:
        return self.term
