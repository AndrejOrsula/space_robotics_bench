from __future__ import annotations

from functools import cached_property
from typing import (
    Any,
    ClassVar,
    Dict,
    Iterable,
    List,
    Mapping,
    Sequence,
    Set,
    Tuple,
    Type,
)

from srb.core.asset import ArticulationCfg, RigidObjectCfg
from srb.core.asset.asset import Asset
from srb.core.asset.asset_type import AssetType
from srb.core.asset.common import Frame
from srb.core.asset.robot.robot_type import RobotType
from srb.utils import convert_to_snake_case


class Robot(Asset, asset_entrypoint=AssetType.ROBOT):
    INPUT_BLOCKLIST: ClassVar[Set] = {"asset_cfg", "action_cfg", "frame_base"}

    asset_cfg: ArticulationCfg | RigidObjectCfg
    action_cfg: Any
    frame_base: Frame

    def __init_subclass__(
        cls,
        robot_entrypoint: RobotType | None = None,
        robot_metaclass: bool = False,
        asset_metaclass: bool = False,
        **kwargs,
    ):
        super().__init_subclass__(
            asset_metaclass=(
                asset_metaclass or robot_entrypoint is not None or robot_metaclass
            ),
            **kwargs,
        )
        if robot_entrypoint is not None:
            assert isinstance(
                robot_entrypoint, RobotType
            ), f"Class '{cls.__name__}' is marked as a robot entrypoint, but '{robot_entrypoint}' is not a valid {RobotType}"
            assert (
                robot_entrypoint not in RobotRegistry.base_types.keys()
            ), f"Class '{cls.__name__}' is marked as '{robot_entrypoint}' robot entrypoint, but it was already marked by '{RobotRegistry.base_types[robot_entrypoint].__name__}'"
            RobotRegistry.base_types[robot_entrypoint] = cls
        elif robot_metaclass:
            RobotRegistry.meta_types.append(cls)
        else:
            for robot_type, base in RobotRegistry.base_types.items():
                if issubclass(cls, base):
                    if robot_type not in RobotRegistry.registry.keys():
                        RobotRegistry.registry[robot_type] = []
                    else:
                        assert (
                            convert_to_snake_case(cls.__name__)
                            not in (
                                convert_to_snake_case(robot.__name__)
                                for robot in RobotRegistry.registry[robot_type]
                            )
                        ), f"Cannot register multiple robots with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{next(robot for robot in RobotRegistry.registry[robot_type] if convert_to_snake_case(cls.__name__) == convert_to_snake_case(robot.__name__)).__module__}:{cls.__name__}'"
                    RobotRegistry.registry[robot_type].append(cls)

    @cached_property
    def robot_type(self) -> RobotType:
        for robot_type, base in RobotRegistry.base_types.items():
            if isinstance(self, base):
                return robot_type
        raise ValueError(f"Class '{self.__class__.__name__}' has unknown robot type")

    @classmethod
    def robot_registry(cls) -> Mapping[RobotType, Sequence[Type[Robot]]]:
        return RobotRegistry.registry

    @classmethod
    def asset_registry(cls) -> Sequence[Type[Robot]]:
        return super().asset_registry().get(AssetType.ROBOT, [])  # type: ignore


class RobotRegistry:
    registry: ClassVar[Dict[RobotType, List[Type[Robot]]]] = {}
    base_types: ClassVar[Dict[RobotType, Type[Robot]]] = {}
    meta_types: ClassVar[List[Type[Robot]]] = []

    @classmethod
    def keys(cls) -> Iterable[RobotType]:
        return cls.registry.keys()

    @classmethod
    def items(cls) -> Iterable[Tuple[RobotType, Sequence[Type[Robot]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[Type[Robot]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[Type[Robot]]:
        return {robot for robots in cls.registry.values() for robot in robots}

    @classmethod
    def n_robots(cls) -> int:
        return sum(len(robots) for robots in cls.registry.values())

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {
            robot.__module__ for robots in cls.registry.values() for robot in robots
        }

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}