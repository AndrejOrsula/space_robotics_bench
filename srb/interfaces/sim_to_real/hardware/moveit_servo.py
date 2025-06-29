from functools import cached_property
from typing import Dict, Sequence

import gymnasium
import numpy

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)


class MoveitServoCfg(HardwareInterfaceCfg):
    frame_id: str = "end_effector"


class MoveitServo(HardwareInterface):
    cfg: MoveitServoCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()

    def __init__(self, cfg: MoveitServoCfg = MoveitServoCfg()):
        super().__init__(cfg)

    def start(self, **kwargs):
        super().start(**kwargs)
        from pymoveit2 import MoveIt2Servo

        self.moveit_servo = MoveIt2Servo(
            self.ros_node,
            frame_id=self.cfg.frame_id,
            linear_speed=self.action_scale_linear,
            angular_speed=self.action_scale_angular,
        )
        self.moveit_servo.servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))

    def close(self):
        super().close()

        self.moveit_servo.disable()

    def reset(self):
        super().reset()

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "robot/delta_twist": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(6,), dtype=numpy.float32
                )
            }
        )

    @cached_property
    def action_scale_linear(self) -> float:
        return self._action_scale.get(
            "robot/delta_twist_linear"
        ) or self._action_scale.get("robot/delta_twist", 1.0)

    @cached_property
    def action_scale_angular(self) -> float:
        return self._action_scale.get(
            "robot/delta_twist_angular"
        ) or self._action_scale.get("robot/delta_twist", 1.0)

    def apply_action(self, action: Dict[str, numpy.ndarray]):
        assert "robot/delta_twist" in action.keys() and action[
            "robot/delta_twist"
        ].shape == (6,)

        act = action["robot/delta_twist"]
        self.moveit_servo.servo(linear=act[0:3], angular=act[3:6])
