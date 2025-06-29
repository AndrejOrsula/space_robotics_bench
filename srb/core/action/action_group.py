from __future__ import annotations

from functools import cache
from typing import ClassVar, Iterable, List, Sequence, Type

import torch

from srb.utils.cfg import configclass
from srb.utils.str import convert_to_snake_case


@configclass
class ActionGroup:
    @classmethod
    @cache
    def name(cls) -> str:
        return convert_to_snake_case(cls.__name__).removesuffix("_action_group")

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        raise NotImplementedError()

    def __init_subclass__(cls, action_group_metaclass: bool = False, **kwargs):
        super().__init_subclass__(**kwargs)
        if action_group_metaclass:
            return
        assert cls.name() not in (
            action_group.name() for action_group in ActionGroupRegistry.registry
        ), (
            f"Cannot register multiple action groups with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{next(robot for robot in ActionGroupRegistry.registry if cls.name() == robot.name()).__module__}:{cls.__name__}'"
        )
        ActionGroupRegistry.registry.append(cls)

    @classmethod
    def action_group_registry(cls) -> Sequence[Type[ActionGroup]]:
        return ActionGroupRegistry.registry


class ActionGroupRegistry:
    registry: ClassVar[List[Type[ActionGroup]]] = []

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {action_group.__module__ for action_group in cls.registry}

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> Type[ActionGroup] | None:
        for action_group in cls.registry:
            if action_group.name() == name:
                return action_group
        return None
