from __future__ import annotations

from functools import cached_property
from typing import ClassVar, Dict, Iterable, List, Sequence, Tuple, Type

from srb.core.asset import ArticulationCfg, RigidObjectCfg
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator_type import (
    MobileManipulatorType,
)
from srb.core.asset.robot.robot import Robot, RobotRegistry
from srb.core.asset.robot.robot_type import RobotType


class MobileManipulator(Robot, robot_entrypoint=RobotType.MOBILE_MANIPULATOR):
    ## Model
    asset_cfg: RigidObjectCfg | ArticulationCfg

    def __init_subclass__(
        cls,
        mobile_manipulator_entrypoint: MobileManipulatorType | None = None,
        mobile_manipulator_metaclass: bool = False,
        robot_metaclass: bool = False,
        **kwargs,
    ):
        super().__init_subclass__(
            robot_metaclass=(
                robot_metaclass
                or mobile_manipulator_entrypoint is not None
                or mobile_manipulator_metaclass
            ),
            **kwargs,
        )
        if mobile_manipulator_entrypoint is not None:
            assert isinstance(mobile_manipulator_entrypoint, MobileManipulatorType), (
                f"Class '{cls.__name__}' is marked as a mobile manipulator entrypoint, but '{mobile_manipulator_entrypoint}' is not a valid {MobileManipulatorType}"
            )
            assert (
                mobile_manipulator_entrypoint
                not in MobileManipulatorRegistry.base_types.keys()
            ), (
                f"Class '{cls.__name__}' is marked as '{mobile_manipulator_entrypoint}' mobile manipulator entrypoint, but it was already marked by '{MobileManipulatorRegistry.base_types[mobile_manipulator_entrypoint].__name__}'"
            )
            MobileManipulatorRegistry.base_types[mobile_manipulator_entrypoint] = cls
        elif mobile_manipulator_metaclass:
            MobileManipulatorRegistry.meta_types.append(cls)
        elif not robot_metaclass:
            for (
                mobile_manipulator_type,
                base,
            ) in MobileManipulatorRegistry.base_types.items():
                if issubclass(cls, base):
                    if (
                        mobile_manipulator_type
                        not in MobileManipulatorRegistry.registry.keys()
                    ):
                        MobileManipulatorRegistry.registry[mobile_manipulator_type] = []
                    else:
                        assert cls.name() not in (
                            mobile_manipulator.name()
                            for mobile_manipulator in MobileManipulatorRegistry.registry[
                                mobile_manipulator_type
                            ]
                        ), (
                            f"Cannot register multiple mobile manipulators with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{next(mobile_manipulator for mobile_manipulator in MobileManipulatorRegistry.registry[mobile_manipulator_type] if cls.name() == mobile_manipulator.name()).__module__}:{cls.__name__}'"
                        )
                    MobileManipulatorRegistry.registry[mobile_manipulator_type].append(
                        cls
                    )
                    break

    @cached_property
    def mobile_manipulator_type(self) -> MobileManipulatorType:
        for (
            mobile_manipulator_type,
            base,
        ) in MobileManipulatorRegistry.base_types.items():
            if isinstance(self, base):
                return mobile_manipulator_type
        raise ValueError(
            f"Class '{self.__class__.__name__}' has unknown mobile manipulator type"
        )

    @classmethod
    def mobile_manipulator_registry(cls) -> Sequence[Type[MobileManipulator]]:
        return list(MobileManipulatorRegistry.values_inner())

    @classmethod
    def robot_registry(cls) -> Sequence[Type[MobileManipulator]]:
        return RobotRegistry.registry.get(RobotType.MOBILE_MANIPULATOR, [])  # type: ignore


class MobileManipulatorRegistry:
    registry: ClassVar[Dict[MobileManipulatorType, List[Type[MobileManipulator]]]] = {}
    base_types: ClassVar[Dict[MobileManipulatorType, Type[MobileManipulator]]] = {}
    meta_types: ClassVar[List[Type[MobileManipulator]]] = []

    @classmethod
    def keys(cls) -> Iterable[MobileManipulatorType]:
        return cls.registry.keys()

    @classmethod
    def items(
        cls,
    ) -> Iterable[Tuple[MobileManipulatorType, Sequence[Type[MobileManipulator]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[Type[MobileManipulator]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[Type[MobileManipulator]]:
        return (
            mobile_manipulator
            for mobile_manipulators in cls.registry.values()
            for mobile_manipulator in mobile_manipulators
        )

    @classmethod
    def n_robots(cls) -> int:
        return sum(
            len(mobile_manipulators) for mobile_manipulators in cls.registry.values()
        )

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {
            mobile_manipulator.__module__
            for mobile_manipulators in cls.registry.values()
            for mobile_manipulator in mobile_manipulators
        }

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> Type[MobileManipulator] | None:
        for mobile_manipulator in cls.values_inner():
            if mobile_manipulator.name() == name:
                return mobile_manipulator
        return None
