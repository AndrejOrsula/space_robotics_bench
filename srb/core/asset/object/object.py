from __future__ import annotations

from functools import cached_property
from typing import ClassVar, Dict, Iterable, List, Sequence, Tuple, Type

from srb.core.asset.asset import Asset, AssetRegistry
from srb.core.asset.asset_type import AssetType
from srb.core.asset.object.object_type import ObjectType


class Object(Asset, asset_entrypoint=AssetType.OBJECT):
    def __init_subclass__(
        cls,
        object_entrypoint: ObjectType | None = None,
        object_metaclass: bool = False,
        asset_metaclass: bool = False,
        **kwargs,
    ):
        super().__init_subclass__(
            asset_metaclass=(
                asset_metaclass or object_entrypoint is not None or object_metaclass
            ),
            **kwargs,
        )
        if object_entrypoint is not None:
            assert isinstance(object_entrypoint, ObjectType), (
                f"Class '{cls.__name__}' is marked as an object entrypoint, but '{object_entrypoint}' is not a valid {ObjectType}"
            )
            assert object_entrypoint not in ObjectRegistry.base_types.keys(), (
                f"Class '{cls.__name__}' is marked as '{object_entrypoint}' object entrypoint, but it was already marked by '{ObjectRegistry.base_types[object_entrypoint].__name__}'"
            )
            ObjectRegistry.base_types[object_entrypoint] = cls
        elif object_metaclass:
            ObjectRegistry.meta_types.append(cls)
        elif not asset_metaclass:
            object_type = next(
                (
                    object_type
                    for object_type, base in ObjectRegistry.base_types.items()
                    if issubclass(cls, base)
                ),
                ObjectType.COMMON,
            )
            if object_type not in ObjectRegistry.registry.keys():
                ObjectRegistry.registry[object_type] = []
            else:
                assert cls.name() not in (
                    object.name() for object in ObjectRegistry.registry[object_type]
                ), (
                    f"Cannot register multiple sceneries with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{next(object for object in ObjectRegistry.registry[object_type] if cls.name() == object.name()).__module__}:{cls.__name__}'"
                )
            ObjectRegistry.registry[object_type].append(cls)

    @cached_property
    def object_type(self) -> ObjectType:
        for object_type, base in ObjectRegistry.base_types.items():
            if isinstance(self, base):
                return object_type
        return ObjectType.COMMON

    @classmethod
    def object_registry(cls) -> Sequence[Type[Object]]:
        return list(ObjectRegistry.values_inner())

    @classmethod
    def asset_registry(cls) -> Sequence[Type[Object]]:
        return AssetRegistry.registry.get(AssetType.OBJECT, [])  # type: ignore


class ObjectRegistry:
    registry: ClassVar[Dict[ObjectType, List[Type[Object]]]] = {}
    base_types: ClassVar[Dict[ObjectType, Type[Object]]] = {}
    meta_types: ClassVar[List[Type[Object]]] = []

    @classmethod
    def keys(cls) -> Iterable[ObjectType]:
        return cls.registry.keys()

    @classmethod
    def items(cls) -> Iterable[Tuple[ObjectType, Sequence[Type[Object]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[Type[Object]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[Type[Object]]:
        return (object for sceneries in cls.registry.values() for object in sceneries)

    @classmethod
    def n_sceneries(cls) -> int:
        return sum(len(sceneries) for sceneries in cls.registry.values())

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {
            object.__module__
            for sceneries in cls.registry.values()
            for object in sceneries
        }

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> Type[Object] | None:
        for object in cls.values_inner():
            if object.name() == name:
                return object
        return None
