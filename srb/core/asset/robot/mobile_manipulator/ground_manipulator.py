from __future__ import annotations

from typing import Sequence, Type

from srb.core.asset.robot.mobile_manipulator.mobile_manipulator import MobileManipulator
from srb.core.asset.robot.mobile_manipulator.mobile_manipulator_type import (
    MobileManipulatorType,
)


class GroundManipulator(
    MobileManipulator, mobile_manipulator_entrypoint=MobileManipulatorType.GROUND
):
    @classmethod
    def mobile_manipulator_registry(cls) -> Sequence[Type[GroundManipulator]]:
        return (
            super().mobile_manipulator_registry().get(MobileManipulatorType.GROUND, [])
        )  # type: ignore