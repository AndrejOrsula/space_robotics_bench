from __future__ import annotations

from typing import Sequence

from srb.core.asset.robot.mobile.aerial_robot import AerialRobot
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator import (
    MobileManipulatorRegistry,
)
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator_type import (
    MobileManipulatorType,
)

from ._metaclass import CombinedMobileManipulator


class AerialManipulator(
    CombinedMobileManipulator,
    AerialRobot,
    mobile_manipulator_entrypoint=MobileManipulatorType.AERIAL,
):
    ## Model
    mobile_base: AerialRobot | None = None

    @classmethod
    def mobile_manipulator_registry(cls) -> Sequence[type[AerialManipulator]]:
        return MobileManipulatorRegistry.registry.get(MobileManipulatorType.AERIAL, [])  # type: ignore[return-value]
