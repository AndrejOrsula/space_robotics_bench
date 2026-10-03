from __future__ import annotations

from functools import cached_property
from typing import ClassVar, Iterable, Self, Sequence

from pydantic import PositiveFloat

from srb.core.asset import AssetBaseCfg, TerrainImporterCfg
from srb.core.asset.asset import Asset, AssetRegistry
from srb.core.asset.asset_type import AssetType
from srb.core.asset.scenery.scenery_type import SceneryType
from srb.core.registry_seam import register_asset_registry


class Scenery(Asset, asset_entrypoint=AssetType.SCENERY):
    ## Model
    asset_cfg: AssetBaseCfg | TerrainImporterCfg

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
        # NOTE: ``scenery_metaclass`` excludes a scenery from the *catalog*
        # (``SceneryRegistry.meta_types`` below) while keeping it directly
        # instantiable — e.g. ``RealTerrain``, which bakes/downloads terrain in
        # ``model_post_init`` and must not be auto-enumerated by docgen/thumbnail
        # passes, yet is the documented entry point users construct by hand.
        # It must therefore NOT imply ``asset_metaclass`` (which would route the
        # class into ``AssetRegistry.meta_types`` and make ``Asset.__new__``
        # reject instantiation). Only ``asset_metaclass`` and base entrypoints
        # are genuinely abstract.
        super().__init_subclass__(
            asset_metaclass=(asset_metaclass or scenery_entrypoint is not None),
            **kwargs,
        )
        if scenery_entrypoint is not None:
            if not isinstance(scenery_entrypoint, SceneryType):
                raise TypeError(
                    f"Class '{cls.__name__}' is marked as a scenery entrypoint, but '{scenery_entrypoint}' is not a valid {SceneryType}"
                )
            if scenery_entrypoint in SceneryRegistry.base_types:
                raise ValueError(
                    f"Class '{cls.__name__}' is marked as '{scenery_entrypoint}' scenery entrypoint, but it was already marked by '{SceneryRegistry.base_types[scenery_entrypoint].__name__}'"
                )
            SceneryRegistry.base_types[scenery_entrypoint] = cls
        elif scenery_metaclass:
            SceneryRegistry.meta_types.append(cls)
        elif not asset_metaclass:
            for scenery_type, base in SceneryRegistry.base_types.items():
                if issubclass(cls, base):
                    if scenery_type not in SceneryRegistry.registry:
                        SceneryRegistry.registry[scenery_type] = []
                    else:
                        if cls.name() in (
                            scenery.name()
                            for scenery in SceneryRegistry.registry[scenery_type]
                        ):
                            raise ValueError(
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
    def scenery_registry(cls) -> Sequence[type[Self]]:
        """Registered sceneries of this class's category (all for ``Scenery``)."""
        for scenery_type, base in SceneryRegistry.base_types.items():
            if issubclass(cls, base):
                return list(SceneryRegistry.registry.get(scenery_type, []))  # type: ignore[arg-type]
        return list(SceneryRegistry.values_inner())  # type: ignore[arg-type]

    @classmethod
    def asset_registry(cls) -> Sequence[type[Scenery]]:
        return AssetRegistry.registry.get(AssetType.SCENERY, [])  # type: ignore[return-value]


class SceneryRegistry:
    registry: ClassVar[dict[SceneryType, list[type[Scenery]]]] = {}
    base_types: ClassVar[dict[SceneryType, type[Scenery]]] = {}
    meta_types: ClassVar[list[type[Scenery]]] = []

    @classmethod
    def keys(cls) -> Iterable[SceneryType]:
        return cls.registry.keys()

    @classmethod
    def items(cls) -> Iterable[tuple[SceneryType, Sequence[type[Scenery]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[type[Scenery]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[type[Scenery]]:
        return (scenery for sceneries in cls.registry.values() for scenery in sceneries)

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
    def get_by_name(cls, name: str) -> type[Scenery] | None:
        for scenery in cls.values_inner():
            if scenery.name() == name:
                return scenery
        return None


# Append-only and populated by `__init_subclass__`; declared as part of the
# supported snapshot/restore seam in `srb.core.registry_seam`.
register_asset_registry(SceneryRegistry)
