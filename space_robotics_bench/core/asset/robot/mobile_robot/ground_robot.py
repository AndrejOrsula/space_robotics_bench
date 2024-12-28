from __future__ import annotations

from typing import Sequence, Type

from space_robotics_bench.core.actions import (
    LocomotionJointSpaceActionCfg,
    WheeledRoverActionGroupCfg,
)
from space_robotics_bench.core.asset import Frame
from space_robotics_bench.core.asset.robot.mobile_robot.mobile_robot import MobileRobot
from space_robotics_bench.core.asset.robot.mobile_robot.mobile_robot_type import (
    MobileRobotType,
)


class GroundRobot(MobileRobot, mobile_robot_entrypoint=MobileRobotType.GROUND):
    @classmethod
    def mobile_robot_registry(cls) -> Sequence[Type[GroundRobot]]:
        return super().mobile_robot_registry().get(MobileRobotType.GROUND, [])  # type: ignore


class WheeledRobot(GroundRobot, mobile_robot_metaclass=True):
    ## Actions
    action_cfg: WheeledRoverActionGroupCfg

    ## Frames
    frame_camera_front: Frame

    ## Joints
    regex_drive_joints: str
    regex_steer_joints: str


class LeggedRobot(GroundRobot, mobile_robot_metaclass=True):
    ## Actions
    action_cfg: LocomotionJointSpaceActionCfg