from __future__ import annotations

import copy
from collections.abc import Callable, Mapping
from functools import cache
from types import MethodType
from typing import ClassVar, Iterable

import torch

from srb.core.command_mapping import (
    CommandMapping,
    apply_command_mapping,
    concat_command_mappings,
    rename_destinations,
)
from srb.core.registry_seam import register_registry_container
from srb.utils.cfg import configclass
from srb.utils.str import convert_to_snake_case


def _renamed_command_mapping_getter(
    action_group: ActionGroup, names: Mapping[str, str]
) -> Callable[[], CommandMapping | None]:
    declared_mapper = action_group.map_cmd_to_action

    def _get_mapping() -> CommandMapping | None:
        current_mapper = action_group.map_cmd_to_action
        declared_self = getattr(declared_mapper, "__self__", None)
        declared_function = getattr(declared_mapper, "__func__", None)
        if declared_self is not None and declared_function is not None:
            if (
                getattr(current_mapper, "__self__", None) is not declared_self
                or getattr(current_mapper, "__func__", None) is not declared_function
            ):
                return None
        elif current_mapper is not declared_mapper:
            return None

        mapping = action_group.get_command_mapping()
        if mapping is None:
            return None
        return rename_destinations(mapping, names)

    return _get_mapping


def _compose_command_mapping_getters(
    getters: Iterable[Callable[[], CommandMapping | None]],
) -> CommandMapping | None:
    mappings = [getter() for getter in getters]
    if not mappings or any(mapping is None for mapping in mappings):
        return None
    return concat_command_mappings(*mappings)


def _snapshot_action_group_leaf(action_group: ActionGroup) -> ActionGroup:
    """Shallow-copy a declared group before it becomes an aggregate in place."""
    snapshot = copy.copy(action_group)
    if snapshot is action_group or type(snapshot) is not type(action_group):
        raise TypeError("copy.copy must return a distinct instance of the same type")
    for name, value in vars(snapshot).items():
        if isinstance(value, MethodType) and value.__self__ is action_group:
            setattr(snapshot, name, MethodType(value.__func__, snapshot))
    return snapshot


@configclass
class ActionGroup:
    # Keyed by `cls`; every action group is already pinned by
    # `ActionGroupRegistry`.
    @classmethod
    @cache
    def name(cls) -> str:
        return convert_to_snake_case(cls.__name__).removesuffix("_action_group")

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        raise NotImplementedError()

    def get_command_mapping(self) -> CommandMapping | None:
        """Return explicit direct-command semantics for this action group.

        The default deliberately carries no meaning. Action dimensions and
        scales are not enough to derive a command mapping.
        """
        if hasattr(self, "__srb_command_mapping_getters"):
            return self._get_composed_command_mapping()
        return None

    def _declared_command_mapping(
        self, mapping: CommandMapping
    ) -> CommandMapping | None:
        """Drop inherited declarations when a subclass replaces the mapper."""
        if hasattr(self, "__srb_command_mapping_getters"):
            return self._get_composed_command_mapping()
        instance_members = vars(self)
        if "map_cmd_to_action" in instance_members:
            return mapping if "get_command_mapping" in instance_members else None

        mapper_owner = next(
            (cls for cls in type(self).__mro__ if "map_cmd_to_action" in cls.__dict__),
            ActionGroup,
        )
        declaration_owner = next(
            (
                cls
                for cls in type(self).__mro__
                if "get_command_mapping" in cls.__dict__
            ),
            ActionGroup,
        )
        mro = type(self).__mro__
        if mapper_owner is ActionGroup:
            return None
        if mro.index(mapper_owner) < mro.index(declaration_owner):
            return None
        return mapping

    def _apply_declared_command_mapping(
        self, twist: torch.Tensor, event: bool
    ) -> torch.Tensor:
        mapping = self.get_command_mapping()
        if mapping is None:
            raise NotImplementedError()
        return apply_command_mapping(mapping, twist, event)

    def _get_composed_command_mapping(self) -> CommandMapping | None:
        try:
            expected_mapper = object.__getattribute__(
                self, "__srb_composed_command_mapper"
            )
            current_mapper = object.__getattribute__(self, "map_cmd_to_action")
        except AttributeError:
            return None
        if current_mapper is not expected_mapper:
            return None
        getters = object.__getattribute__(self, "__srb_command_mapping_getters")
        if not callable(getters):
            return None
        return _compose_command_mapping_getters(getters())

    def __init_subclass__(cls, action_group_metaclass: bool = False, **kwargs):
        super().__init_subclass__(**kwargs)
        if action_group_metaclass:
            return
        existing = next(
            (ag for ag in ActionGroupRegistry.registry if ag.name() == cls.name()),
            None,
        )
        if existing is not None:
            raise ValueError(
                f"Cannot register multiple action groups with an identical name: "
                f"'{cls.__module__}:{cls.__name__}' collides with the "
                f"already-registered '{existing.__module__}:{existing.__name__}' "
                f"(both resolve to name '{cls.name()}')"
            )
        ActionGroupRegistry.registry.append(cls)


class ActionGroupRegistry:
    registry: ClassVar[list[type[ActionGroup]]] = []

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {action_group.__module__ for action_group in cls.registry}

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> type[ActionGroup] | None:
        for action_group in cls.registry:
            if action_group.name() == name:
                return action_group
        return None


# Append-only and populated by `__init_subclass__`; declared as part of the
# supported snapshot/restore seam in `srb.core.registry_seam`.
register_registry_container(ActionGroupRegistry, "registry")
