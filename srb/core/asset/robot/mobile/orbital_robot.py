from __future__ import annotations

from typing import Sequence, Type

from srb.core.asset.robot.mobile.mobile_robot import MobileRobot, MobileRobotRegistry
from srb.core.asset.robot.mobile.mobile_robot_type import MobileRobotType


class OrbitalRobot(
    MobileRobot,
    mobile_robot_entrypoint=MobileRobotType.ORBITAL,
    arbitrary_types_allowed=True,
):
    @classmethod
    def mobile_robot_registry(cls) -> Sequence[Type[OrbitalRobot]]:
        return MobileRobotRegistry.registry.get(MobileRobotType.ORBITAL, [])  # type: ignore


class Lander(OrbitalRobot, mobile_robot_metaclass=True):
    pass
