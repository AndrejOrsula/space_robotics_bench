from __future__ import annotations

from typing import Sequence

from srb.core.asset.robot.mobile_manipulation.mobile_manipulator import (
    MobileManipulator,
    MobileManipulatorRegistry,
)
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator_type import (
    MobileManipulatorType,
)


class Humanoid(
    MobileManipulator, mobile_manipulator_entrypoint=MobileManipulatorType.HUMANOID
):
    ## Links
    regex_feet_links: str

    @classmethod
    def mobile_manipulator_registry(cls) -> Sequence[type[Humanoid]]:
        return MobileManipulatorRegistry.registry.get(
            MobileManipulatorType.HUMANOID, []
        )  # type: ignore[misc]
