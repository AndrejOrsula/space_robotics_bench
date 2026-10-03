from __future__ import annotations

from typing import Sequence

from srb.core.asset.robot.mobile.ground_robot import GroundRobot
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator import (
    MobileManipulatorRegistry,
)
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator_type import (
    MobileManipulatorType,
)

from ._metaclass import CombinedMobileManipulator


class GroundManipulator(
    CombinedMobileManipulator,
    GroundRobot,
    mobile_manipulator_entrypoint=MobileManipulatorType.GROUND,
):
    ## Model
    mobile_base: GroundRobot | None = None

    @classmethod
    def mobile_manipulator_registry(cls) -> Sequence[type[GroundManipulator]]:
        return MobileManipulatorRegistry.registry.get(MobileManipulatorType.GROUND, [])  # type: ignore[return-value]
