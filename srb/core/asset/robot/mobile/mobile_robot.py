from __future__ import annotations

from functools import cached_property
from typing import ClassVar, Dict, Iterable, List, Sequence, Tuple, Type

from srb.core.asset.common import Frame
from srb.core.asset.object.payload import Payload
from srb.core.asset.robot.mobile.mobile_robot_type import MobileRobotType
from srb.core.asset.robot.robot import Robot, RobotRegistry
from srb.core.asset.robot.robot_type import RobotType


class MobileRobot(Robot, robot_entrypoint=RobotType.MOBILE_ROBOT):
    ## Model
    payload: Payload | None = None

    ## Frames
    frame_imu: Frame | None = None
    frame_payload_mount: Frame
    frame_manipulator_mount: Frame

    def __init_subclass__(
        cls,
        mobile_robot_entrypoint: MobileRobotType | None = None,
        mobile_robot_metaclass: bool = False,
        robot_metaclass: bool = False,
        **kwargs,
    ):
        super().__init_subclass__(
            robot_metaclass=(
                robot_metaclass
                or mobile_robot_entrypoint is not None
                or mobile_robot_metaclass
            ),
            **kwargs,
        )
        if mobile_robot_entrypoint is not None:
            assert isinstance(mobile_robot_entrypoint, MobileRobotType), (
                f"Class '{cls.__name__}' is marked as a mobile robot entrypoint, but '{mobile_robot_entrypoint}' is not a valid {MobileRobotType}"
            )
            assert (
                mobile_robot_entrypoint not in MobileRobotRegistry.base_types.keys()
            ), (
                f"Class '{cls.__name__}' is marked as '{mobile_robot_entrypoint}' mobile robot entrypoint, but it was already marked by '{MobileRobotRegistry.base_types[mobile_robot_entrypoint].__name__}'"
            )
            MobileRobotRegistry.base_types[mobile_robot_entrypoint] = cls
        elif mobile_robot_metaclass:
            MobileRobotRegistry.meta_types.append(cls)
        elif not robot_metaclass:
            for mobile_robot_type, base in MobileRobotRegistry.base_types.items():
                if issubclass(cls, base):
                    if mobile_robot_type not in MobileRobotRegistry.registry.keys():
                        MobileRobotRegistry.registry[mobile_robot_type] = []
                    else:
                        assert cls.name() not in (
                            mobile_robot.name()
                            for mobile_robot in MobileRobotRegistry.registry[
                                mobile_robot_type
                            ]
                        ), (
                            f"Cannot register multiple mobile robots with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{next(mobile_robot for mobile_robot in MobileRobotRegistry.registry[mobile_robot_type] if cls.name() == mobile_robot.name()).__module__}:{cls.__name__}'"
                        )
                    MobileRobotRegistry.registry[mobile_robot_type].append(cls)
                    break

    @cached_property
    def mobile_robot_type(self) -> MobileRobotType:
        for mobile_robot_type, base in MobileRobotRegistry.base_types.items():
            if isinstance(self, base):
                return mobile_robot_type
        raise ValueError(
            f"Class '{self.__class__.__name__}' has unknown mobile robot type"
        )

    @classmethod
    def mobile_robot_registry(cls) -> Sequence[Type[MobileRobot]]:
        return list(MobileRobotRegistry.values_inner())

    @classmethod
    def robot_registry(cls) -> Sequence[Type[MobileRobot]]:
        return RobotRegistry.registry.get(RobotType.MOBILE_ROBOT, [])  # type: ignore


class MobileRobotRegistry:
    registry: ClassVar[Dict[MobileRobotType, List[Type[MobileRobot]]]] = {}
    base_types: ClassVar[Dict[MobileRobotType, Type[MobileRobot]]] = {}
    meta_types: ClassVar[List[Type[MobileRobot]]] = []

    @classmethod
    def keys(cls) -> Iterable[MobileRobotType]:
        return cls.registry.keys()

    @classmethod
    def items(cls) -> Iterable[Tuple[MobileRobotType, Sequence[Type[MobileRobot]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[Type[MobileRobot]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[Type[MobileRobot]]:
        return (
            mobile_robot
            for mobile_robots in cls.registry.values()
            for mobile_robot in mobile_robots
        )

    @classmethod
    def n_robots(cls) -> int:
        return sum(len(mobile_robots) for mobile_robots in cls.registry.values())

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {
            mobile_robot.__module__
            for mobile_robots in cls.registry.values()
            for mobile_robot in mobile_robots
        }

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> Type[MobileRobot] | None:
        for mobile_robot in cls.values_inner():
            if mobile_robot.name() == name:
                return mobile_robot
        return None
