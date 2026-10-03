from __future__ import annotations

from typing import Sequence

from srb.core.asset.robot.mobile.orbital_robot import OrbitalRobot
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator import (
    MobileManipulatorRegistry,
)
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator_type import (
    MobileManipulatorType,
)

from ._metaclass import CombinedMobileManipulator


class OrbitalManipulator(
    CombinedMobileManipulator,
    OrbitalRobot,
    mobile_manipulator_entrypoint=MobileManipulatorType.ORBITAL,
):
    ## Model
    mobile_base: OrbitalRobot | None = None

    @classmethod
    def mobile_manipulator_registry(cls) -> Sequence[type[OrbitalManipulator]]:
        return MobileManipulatorRegistry.registry.get(MobileManipulatorType.ORBITAL, [])  # type: ignore[return-value]
