from __future__ import annotations

from typing import Sequence

from srb.core.asset.robot.mobile.mobile_robot import MobileRobot, MobileRobotRegistry
from srb.core.asset.robot.mobile.mobile_robot_type import MobileRobotType


class AquaticRobot(MobileRobot, mobile_robot_entrypoint=MobileRobotType.AQUATIC):
    @classmethod
    def mobile_robot_registry(cls) -> Sequence[type[AquaticRobot]]:
        return MobileRobotRegistry.registry.get(MobileRobotType.AQUATIC, [])  # type: ignore[return-value]


class SurfaceVehicle(
    AquaticRobot, mobile_robot_metaclass=True, arbitrary_types_allowed=True
):
    pass
