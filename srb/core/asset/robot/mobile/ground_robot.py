from __future__ import annotations

from typing import Sequence

from srb.core.asset import ArticulationCfg
from srb.core.asset.robot.mobile.mobile_robot import MobileRobot, MobileRobotRegistry
from srb.core.asset.robot.mobile.mobile_robot_type import MobileRobotType


class GroundRobot(MobileRobot, mobile_robot_entrypoint=MobileRobotType.GROUND):
    ## Model
    asset_cfg: ArticulationCfg

    @classmethod
    def mobile_robot_registry(cls) -> Sequence[type[GroundRobot]]:
        return MobileRobotRegistry.registry.get(MobileRobotType.GROUND, [])  # type: ignore[return-value]


class WheeledRobot(GroundRobot, mobile_robot_metaclass=True):
    ## Links
    regex_wheels_links: str


class LeggedRobot(GroundRobot, mobile_robot_metaclass=True):
    ## Links
    regex_feet_links: str
