from __future__ import annotations

from typing import Sequence

from srb.core.asset.robot.manipulation.manipulator import (
    Manipulator,
    ManipulatorRegistry,
)
from srb.core.asset.robot.manipulation.manipulator_type import ManipulatorType


class MultiManipulator(Manipulator, manipulator_entrypoint=ManipulatorType.MULTI):
    @classmethod
    def manipulator_registry(cls) -> Sequence[type[MultiManipulator]]:
        return ManipulatorRegistry.registry.get(ManipulatorType.MULTI, [])  # type: ignore[return-value]
