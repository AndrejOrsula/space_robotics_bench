from typing import Dict, Sequence

import gymnasium
import numpy

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)


class MoveitGripperCfg(HardwareInterfaceCfg):
    gripper_joint_names: Sequence[str] = ()
    open_gripper_joint_positions: Sequence[float] = ()
    closed_gripper_joint_positions: Sequence[float] = ()
    gripper_group_name: str = "gripper"


class MoveitGripper(HardwareInterface):
    cfg: MoveitGripperCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()

    def __init__(self, cfg: MoveitGripperCfg = MoveitGripperCfg()):
        super().__init__(cfg)

    def start(self, **kwargs):
        super().start(**kwargs)
        from pymoveit2 import MoveIt2Gripper

        self.moveit_gripper = MoveIt2Gripper(
            self.ros_node,
            gripper_joint_names=self.cfg.gripper_joint_names,
            open_gripper_joint_positions=self.cfg.open_gripper_joint_positions,
            closed_gripper_joint_positions=self.cfg.closed_gripper_joint_positions,
            gripper_group_name=self.cfg.gripper_group_name,
        )
        self.moveit_gripper.open()

    def close(self):
        super().close()

        self.moveit_gripper.disable()

    def reset(self):
        super().reset()

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "end_effector/joint_pos": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(1,), dtype=numpy.float32
                )
            }
        )

    def apply_action(self, action: Dict[str, numpy.ndarray]):
        assert "end_effector/joint_pos" in action.keys() and action[
            "end_effector/joint_pos"
        ].shape == (1,)

        if action["end_effector/joint_pos"] > 0.0:
            self.moveit_gripper.open()
        else:
            self.moveit_gripper.close()
