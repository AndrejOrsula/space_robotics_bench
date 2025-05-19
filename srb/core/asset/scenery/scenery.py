from __future__ import annotations

from functools import cached_property
from typing import ClassVar, Dict, Iterable, List, Sequence, Tuple, Type

from pydantic import PositiveFloat

from srb.core.asset import AssetBaseCfg
from srb.core.asset.asset import Asset, AssetRegistry
from srb.core.asset.asset_type import AssetType
from srb.core.asset.scenery.scenery_type import SceneryType


class Scenery(Asset, asset_entrypoint=AssetType.SCENERY):
    ## Model
    asset_cfg: AssetBaseCfg

    ## Attributes forwarded to the spawner
    SPAWNER_ATTRIBUTES: ClassVar[Sequence[str]] = ("density", "flat_area_size")
    density: PositiveFloat | None = None
    flat_area_size: PositiveFloat | None = None

    def __init_subclass__(
        cls,
        scenery_entrypoint: SceneryType | None = None,
        scenery_metaclass: bool = False,
        asset_metaclass: bool = False,
        **kwargs,
    ):
        super().__init_subclass__(
            asset_metaclass=(
                asset_metaclass or scenery_entrypoint is not None or scenery_metaclass
            ),
            **kwargs,
        )
        if scenery_entrypoint is not None:
            assert isinstance(scenery_entrypoint, SceneryType), (
                f"Class '{cls.__name__}' is marked as a scenery entrypoint, but '{scenery_entrypoint}' is not a valid {SceneryType}"
            )
            assert scenery_entrypoint not in SceneryRegistry.base_types.keys(), (
                f"Class '{cls.__name__}' is marked as '{scenery_entrypoint}' scenery entrypoint, but it was already marked by '{SceneryRegistry.base_types[scenery_entrypoint].__name__}'"
            )
            SceneryRegistry.base_types[scenery_entrypoint] = cls
        elif scenery_metaclass:
            SceneryRegistry.meta_types.append(cls)
        elif not asset_metaclass:
            for scenery_type, base in SceneryRegistry.base_types.items():
                if issubclass(cls, base):
                    if scenery_type not in SceneryRegistry.registry.keys():
                        SceneryRegistry.registry[scenery_type] = []
                    else:
                        assert cls.name() not in (
                            scenery.name()
                            for scenery in SceneryRegistry.registry[scenery_type]
                        ), (
                            f"Cannot register multiple sceneries with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{next(scenery for scenery in SceneryRegistry.registry[scenery_type] if cls.name() == scenery.name()).__module__}:{cls.__name__}'"
                        )
                    SceneryRegistry.registry[scenery_type].append(cls)
                    break

    @cached_property
    def scenery_type(self) -> SceneryType:
        for scenery_type, base in SceneryRegistry.base_types.items():
            if isinstance(self, base):
                return scenery_type
        raise ValueError(f"Class '{self.__class__.__name__}' has unknown scenery type")

    @classmethod
    def scenery_registry(cls) -> Sequence[Type[Scenery]]:
        return list(SceneryRegistry.values_inner())

    @classmethod
    def asset_registry(cls) -> Sequence[Type[Scenery]]:
        return AssetRegistry.registry.get(AssetType.SCENERY, [])  # type: ignore


class SceneryRegistry:
    registry: ClassVar[Dict[SceneryType, List[Type[Scenery]]]] = {}
    base_types: ClassVar[Dict[SceneryType, Type[Scenery]]] = {}
    meta_types: ClassVar[List[Type[Scenery]]] = []

    @classmethod
    def keys(cls) -> Iterable[SceneryType]:
        return cls.registry.keys()

    @classmethod
    def items(cls) -> Iterable[Tuple[SceneryType, Sequence[Type[Scenery]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[Type[Scenery]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[Type[Scenery]]:
        return (scenery for sceneries in cls.registry.values() for scenery in sceneries)

    @classmethod
    def n_sceneries(cls) -> int:
        return sum(len(sceneries) for sceneries in cls.registry.values())

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {
            scenery.__module__
            for sceneries in cls.registry.values()
            for scenery in sceneries
        }

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> Type[Scenery] | None:
        for scenery in cls.values_inner():
            if scenery.name() == name:
                return scenery
        return None
