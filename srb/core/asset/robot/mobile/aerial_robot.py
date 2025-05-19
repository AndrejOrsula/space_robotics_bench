from __future__ import annotations

from typing import Sequence, Type

from srb.core.asset.common import Frame
from srb.core.asset.robot.mobile.mobile_robot import MobileRobot, MobileRobotRegistry
from srb.core.asset.robot.mobile.mobile_robot_type import MobileRobotType


class AerialRobot(MobileRobot, mobile_robot_entrypoint=MobileRobotType.AERIAL):
    ## Frames
    frame_onboard_camera: Frame

    @classmethod
    def mobile_robot_registry(cls) -> Sequence[Type[AerialRobot]]:
        return MobileRobotRegistry.registry.get(MobileRobotType.AERIAL, [])  # type: ignore


class Multicopter(
    AerialRobot, mobile_robot_metaclass=True, arbitrary_types_allowed=True
):
    pass
