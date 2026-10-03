from __future__ import annotations

from functools import cached_property
from typing import ClassVar, Iterable, Sequence

from srb.core.action import ActionGroup
from srb.core.asset import ArticulationCfg, AssetRegistry, RigidObjectCfg
from srb.core.asset._action_mode_utils import _make_action_group_property
from srb.core.asset.asset import Asset
from srb.core.asset.asset_type import AssetType
from srb.core.asset.common import Frame
from srb.core.asset.robot.robot_type import RobotType
from srb.core.registry_seam import register_asset_registry
from srb.core.sensor import SensorBaseCfg


class Robot(Asset, asset_entrypoint=AssetType.ROBOT):
    ## Model
    asset_cfg: RigidObjectCfg | ArticulationCfg

    ## Action modes — dict of available control groups keyed by lexical name.
    ## Class-level (ClassVar), not a Pydantic model field — contains
    ## ActionTermCfg callables that are not OmegaConf-serialisable.
    ## Insertion order matters: first entry is the default.
    action_modes: ClassVar[dict[str, ActionGroup]]

    ## Selected mode key. None → use first entry of action_modes.
    action_mode: str | None = None

    ## Resolved action group (property — never stored).
    action_group = _make_action_group_property()

    ## Sensors
    sensor_cfgs: dict[str, SensorBaseCfg] = {}

    ## Frames
    frame_base: Frame

    def model_post_init(self, __context):
        super().model_post_init(__context)
        self.configure_sensors()

    def configure_sensors(self):
        pass

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
            if not isinstance(robot_entrypoint, RobotType):
                raise TypeError(
                    f"Class '{cls.__name__}' is marked as a robot entrypoint, but '{robot_entrypoint}' is not a valid {RobotType}"
                )
            if robot_entrypoint in RobotRegistry.base_types:
                raise ValueError(
                    f"Class '{cls.__name__}' is marked as '{robot_entrypoint}' robot entrypoint, but it was already marked by '{RobotRegistry.base_types[robot_entrypoint].__name__}'"
                )
            RobotRegistry.base_types[robot_entrypoint] = cls
        elif robot_metaclass:
            RobotRegistry.meta_types.append(cls)
        elif not asset_metaclass:
            # Note: Sorted here is a simple workaround to prioritize "mobile_manipulator" over "mobile_robot"
            for robot_type, base in sorted(RobotRegistry.base_types.items()):
                if issubclass(cls, base):
                    if robot_type not in RobotRegistry.registry:
                        RobotRegistry.registry[robot_type] = []
                    else:
                        if cls.name() in (
                            robot.name() for robot in RobotRegistry.registry[robot_type]
                        ):
                            raise ValueError(
                                f"Cannot register multiple robots with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{next(robot for robot in RobotRegistry.registry[robot_type] if cls.name() == robot.name()).__module__}:{cls.__name__}'"
                            )
                    RobotRegistry.registry[robot_type].append(cls)
                    break

    @cached_property
    def robot_type(self) -> RobotType:
        for robot_type, base in sorted(RobotRegistry.base_types.items()):
            if isinstance(self, base):
                return robot_type
        raise ValueError(f"Class '{self.__class__.__name__}' has unknown robot type")

    @classmethod
    def robot_registry(cls) -> Sequence[type[Robot]]:
        return list(RobotRegistry.values_inner())

    @classmethod
    def asset_registry(cls) -> Sequence[type[Robot]]:
        return AssetRegistry.registry.get(AssetType.ROBOT, [])  # type: ignore[return-value]


class RobotRegistry:
    registry: ClassVar[dict[RobotType, list[type[Robot]]]] = {}
    base_types: ClassVar[dict[RobotType, type[Robot]]] = {}
    meta_types: ClassVar[list[type[Robot]]] = []

    @classmethod
    def keys(cls) -> Iterable[RobotType]:
        return cls.registry.keys()

    @classmethod
    def items(cls) -> Iterable[tuple[RobotType, Sequence[type[Robot]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[type[Robot]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[type[Robot]]:
        return (robot for robots in cls.registry.values() for robot in robots)

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

    @classmethod
    def get_by_name(cls, name: str) -> type[Robot] | None:
        for robot in cls.values_inner():
            if robot.name() == name:
                return robot
        return None


def _first_joint_mode(action_modes: dict[str, "ActionGroup"]) -> str | None:
    """Return the first ``joint_`` mode key, or ``None`` if absent."""
    for key in action_modes:
        if key.startswith("joint_"):
            return key
    return None


# Append-only and populated by `__init_subclass__`; declared as part of the
# supported snapshot/restore seam in `srb.core.registry_seam`.
register_asset_registry(RobotRegistry)
