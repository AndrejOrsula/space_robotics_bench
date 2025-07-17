from __future__ import annotations

import functools
import operator
import types
from functools import cache, cached_property
from itertools import chain
from typing import (
    TYPE_CHECKING,
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

from pydantic import BaseModel, PositiveFloat
from simforge import BlGeometry, BlModel, BlShader, TexResConfig

from srb.core.asset import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from srb.core.asset.asset_type import AssetType
from srb.core.asset.asset_variant import AssetVariant
from srb.core.domain import Domain
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    ShapeCfg,
    SimforgeAssetCfg,
    SpawnerCfg,
)
from srb.utils import logging
from srb.utils.str import convert_to_snake_case

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


class Asset(BaseModel):
    ## Scenario
    DOMAINS: ClassVar[Sequence[Domain]] = ()

    ## Model
    asset_cfg: AssetBaseCfg | RigidObjectCfg | ArticulationCfg

    ## Attributes forwarded to the spawner
    SPAWNER_ATTRIBUTES: ClassVar[Sequence[str]] = (
        "size",
        "scale",
        "texture_resolution",
    )
    scale: Tuple[PositiveFloat, PositiveFloat, PositiveFloat] | None = None
    texture_resolution: TexResConfig | None = None

    def __new__(cls, *args, **kwargs):
        if cls in (
            Asset,
            *AssetRegistry.base_types.keys(),
            *AssetRegistry.meta_types,
        ):
            raise TypeError(f"Cannot instantiate abstract class {cls.__name__}")
        return super().__new__(cls)

    @classmethod
    @cache
    def name(cls) -> str:
        return convert_to_snake_case(cls.__name__)

    @property
    def size(self) -> Tuple[PositiveFloat, PositiveFloat] | None:
        if self.scale is not None:
            return (self.scale[0], self.scale[1])
        else:
            return None

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        """
        This method allows for additional scene setup that is specific to the asset.
        It is called after the asset has been added to the scene, but before the simulation starts.
        The intended use for this method is to create additional scene elements or modify existing ones.
        """
        pass

    def __init_subclass__(
        cls,
        asset_entrypoint: AssetType | None = None,
        asset_metaclass: bool = False,
        **kwargs,
    ):
        super().__init_subclass__(**kwargs)
        if asset_entrypoint is not None:
            assert isinstance(asset_entrypoint, AssetType), (
                f"Class '{cls.__name__}' is marked as an asset entrypoint, but '{asset_entrypoint}' is not a valid {AssetType}"
            )
            assert asset_entrypoint not in AssetRegistry.base_types.keys(), (
                f"Class '{cls.__name__}' is marked as '{asset_entrypoint}' asset entrypoint, but it was already marked by '{AssetRegistry.base_types[asset_entrypoint].__name__}'"
            )
            AssetRegistry.base_types[asset_entrypoint] = cls
        elif asset_metaclass:
            AssetRegistry.meta_types.append(cls)
        else:
            for asset_type, base in AssetRegistry.base_types.items():
                if issubclass(cls, base):
                    if asset_type not in AssetRegistry.registry.keys():
                        AssetRegistry.registry[asset_type] = []
                    else:
                        assert cls.name() not in (
                            asset.name() for asset in AssetRegistry.registry[asset_type]
                        ), (
                            f"Cannot register multiple assets with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{next(asset for asset in AssetRegistry.registry[asset_type] if cls.name() == asset.name()).__module__}:{cls.__name__}'"
                        )
                    AssetRegistry.registry[asset_type].append(cls)
                    break

    @cached_property
    def asset_type(self) -> AssetType:
        for asset_type, base in AssetRegistry.base_types.items():
            if isinstance(self, base):
                return asset_type
        raise ValueError(f"Class '{self.__class__.__name__}' has unknown asset type")

    @classmethod
    def asset_registry(cls) -> Sequence[Type[Asset]]:
        return list(AssetRegistry.values_inner())

    @cached_property
    def asset_variant(self) -> AssetVariant | None:
        if not isinstance(self.asset_cfg, AssetBaseCfg) or self.asset_cfg.spawn is None:
            return None

        def __get_variant(spawner: SpawnerCfg) -> AssetVariant:
            if isinstance(spawner, SimforgeAssetCfg):
                return AssetVariant.PROCEDURAL
            elif isinstance(spawner, ShapeCfg):
                return AssetVariant.PRIMITIVE
            else:
                return AssetVariant.DATASET

        if isinstance(self.asset_cfg.spawn, MultiAssetSpawnerCfg):
            assert len(self.asset_cfg.spawn.assets_cfg) > 0
            variants = set(
                __get_variant(spawner) for spawner in self.asset_cfg.spawn.assets_cfg
            )
            if AssetVariant.PROCEDURAL in variants:
                return AssetVariant.PROCEDURAL
            elif AssetVariant.DATASET in variants:
                return AssetVariant.DATASET
            else:
                return AssetVariant.PRIMITIVE
        else:
            return __get_variant(self.asset_cfg.spawn)

    def is_variant(self, variant: AssetVariant) -> bool:
        return variant is self.asset_variant

    @property
    def _model_values(self) -> Mapping[str, Any]:
        return {
            k: v
            for k, v in {
                k: getattr(self, k)
                for k in chain(
                    self.__class__.model_fields.keys(),
                    self.model_computed_fields.keys(),
                )
            }.items()
            if v is not None
        }

    def model_post_init(self, __context):
        super().model_post_init(__context)

        ## Forward specified attributes to the spawner
        if isinstance(self.asset_cfg, AssetBaseCfg):
            assert self.asset_cfg.spawn is not None
            Asset.__set_spawner_attributes(
                self.asset_cfg.spawn, self._spawner_attributes
            )

    @property
    def _spawner_attributes(self) -> Mapping[str, Any]:
        return {
            k: v
            for k, v in self._model_values.items()
            if k in self._get_combined_spawner_attribute_keys()
        }

    def _get_combined_spawner_attribute_keys(self) -> Set[str]:
        all_attributes = set()
        for cls in self.__class__.__mro__:
            if hasattr(cls, "SPAWNER_ATTRIBUTES"):
                all_attributes.update(cls.SPAWNER_ATTRIBUTES)
        return all_attributes

    @staticmethod
    def __set_spawner_attributes(
        spawner: SpawnerCfg, spawner_attributes: Mapping[str, Any]
    ):
        if len(spawner_attributes) == 0:
            return

        if isinstance(spawner, SimforgeAssetCfg):
            for asset in spawner.assets:
                match asset:
                    case _geo if isinstance(asset, BlGeometry):
                        for k, v in spawner_attributes.items():
                            _set = False
                            for op in asset.ops:
                                if hasattr(op, k) and isinstance(
                                    getattr(op, k), type(v)
                                ):
                                    setattr(op, k, v)
                                    _set = True
                                    logging.debug(
                                        f'Updated input "{k}" to "{v}" for {BlGeometry.__name__} operation "{op.__class__.__name__}" of "{asset.__class__.__name__}"'
                                    )
                            if not _set:
                                logging.trace(
                                    f'Input "{k}" of type "{type(k)}" not updated for "{asset.__class__.__name__}"'
                                )
                    case _model if isinstance(asset, BlModel):
                        for k, v in spawner_attributes.items():
                            _set = False
                            for op in asset.geo.ops:
                                if hasattr(op, k) and isinstance(
                                    getattr(op, k), type(v)
                                ):
                                    setattr(op, k, v)
                                    _set = True
                                    logging.debug(
                                        f'Updated input "{k}" to "{v}" for {BlGeometry.__name__} operation "{op.__class__.__name__}" of "{asset.__class__.__name__}/{asset.geo.__class__.__name__}"'
                                    )
                            if (
                                asset.mat is not None
                                and hasattr(asset.mat.shader, k)
                                and isinstance(getattr(asset.mat.shader, k), type(v))
                            ):
                                setattr(asset.mat.shader, k, v)
                                _set = True
                                logging.debug(
                                    f'Updated input "{k}" to "{v}" for "{asset.mat.shader.__class__.__name__}" {BlShader.__name__} of "{asset.__class__.__name__}/{asset.mat.__class__.__name__}"'
                                )
                            if hasattr(asset, k) and isinstance(
                                getattr(asset, k), type(v)
                            ):
                                setattr(asset, k, v)
                                _set = True
                                logging.debug(
                                    f'Updated input "{k}" to "{v}" for "{asset.__class__.__name__}"'
                                )
                            if not _set:
                                logging.trace(
                                    f'Input "{k}" of type "{type(k)}" not updated for "{asset.__class__.__name__}"'
                                )
                    case _:
                        logging.warning(
                            f"SimForge asset of type '{type(asset)}' is not supported for input updates"
                        )
        elif isinstance(spawner, MultiAssetSpawnerCfg):
            for subspawner in spawner.assets_cfg:
                Asset.__set_spawner_attributes(subspawner, spawner_attributes)
            for k, v in spawner_attributes.items():
                _set = False
                if hasattr(spawner, k) and isinstance(getattr(spawner, k), type(v)):
                    setattr(spawner, k, v)
                if not _set:
                    logging.trace(
                        f'Input "{k}" of type "{type(k)}" not updated for "{spawner.__class__.__name__}"'
                    )
        else:
            for k, v in spawner_attributes.items():
                _set = False
                if hasattr(spawner, k) and isinstance(getattr(spawner, k), type(v)):
                    setattr(spawner, k, v)
                    _set = True
                    logging.debug(
                        f'Updated input "{k}" to "{v}" for "{spawner.__class__.__name__}"'
                    )
                if not _set:
                    logging.trace(
                        f'Input "{k}" of type "{type(k)}" not updated for "{spawner.__class__.__name__}"'
                    )

    def as_asset_base_cfg(
        self, disable_articulation: bool = False, disable_rigid_body: bool = False
    ) -> AssetBaseCfg:
        if isinstance(self.asset_cfg, (RigidObjectCfg, ArticulationCfg)):
            asset_cfg = AssetBaseCfg(
                prim_path=self.asset_cfg.prim_path,
                spawn=self.asset_cfg.spawn,
                init_state=self.asset_cfg.init_state,
            )

            def remove_props(attr: Any):
                for props in (
                    "articulation_props",
                    "deformable_props",
                    "fixed_tendons_props",
                    "joint_drive_props",
                    "mass_props",
                    "rigid_props",
                ):
                    if hasattr(attr, props):
                        setattr(attr, props, None)
                if hasattr(attr, "activate_contact_sensors"):
                    attr.activate_contact_sensors = False  # type: ignore
                if disable_articulation:
                    setattr(
                        attr,
                        "articulation_props",
                        ArticulationRootPropertiesCfg(articulation_enabled=False),
                    )
                if disable_rigid_body:
                    setattr(
                        attr,
                        "rigid_props",
                        RigidBodyPropertiesCfg(rigid_body_enabled=False),
                    )

            remove_props(asset_cfg.spawn)
            if isinstance(asset_cfg.spawn, MultiAssetSpawnerCfg):
                for subspawner in asset_cfg.spawn.assets_cfg:
                    remove_props(subspawner)

            new_annotation: Any | None = None
            if isinstance(self.__annotations__["asset_cfg"], types.UnionType):
                for typ in self.__annotations__["asset_cfg"].__args__:
                    if issubclass(AssetBaseCfg, typ):
                        break
                else:
                    new_annotation = functools.reduce(
                        operator.or_,
                        tuple(
                            chain(
                                (AssetBaseCfg,),
                                self.__annotations__["asset_cfg"].__args__,
                            ),
                        ),
                    )
            elif not isinstance(self.__annotations__["asset_cfg"], AssetBaseCfg):
                new_annotation = functools.reduce(
                    operator.or_, (AssetBaseCfg, self.__annotations__["asset_cfg"])
                )
            if new_annotation is not None:
                self.__annotations__["asset_cfg"] = new_annotation
                self.model_fields["asset_cfg"].annotation = new_annotation
                self.model_rebuild(force=True)

            return asset_cfg
        elif isinstance(self.asset_cfg, AssetBaseCfg):
            return self.asset_cfg.copy()  # type: ignore
        else:
            raise TypeError(
                f"Cannot convert asset of type '{type(self.asset_cfg)}' to {AssetBaseCfg.__name__}"
            )

    def as_rigid_object_cfg(self, disable_articulation: bool = False) -> RigidObjectCfg:
        if isinstance(self.asset_cfg, ArticulationCfg):
            asset_cfg = RigidObjectCfg(
                prim_path=self.asset_cfg.prim_path,
                spawn=self.asset_cfg.spawn,
                init_state=self.asset_cfg.init_state,  # type: ignore
            )

            def remove_props(attr: Any):
                for props in (
                    "articulation_props",
                    "fixed_tendons_props",
                    "joint_drive_props",
                ):
                    if hasattr(attr, props):
                        setattr(attr, props, None)
                if disable_articulation:
                    setattr(
                        attr,
                        "articulation_props",
                        ArticulationRootPropertiesCfg(articulation_enabled=False),
                    )

            remove_props(asset_cfg.spawn)
            if isinstance(asset_cfg.spawn, MultiAssetSpawnerCfg):
                for subspawner in asset_cfg.spawn.assets_cfg:
                    remove_props(subspawner)

            new_annotation: Any | None = None
            if isinstance(self.__annotations__["asset_cfg"], types.UnionType):
                for typ in self.__annotations__["asset_cfg"].__args__:
                    if issubclass(RigidObjectCfg, typ):
                        break
                else:
                    new_annotation = functools.reduce(
                        operator.or_,
                        tuple(
                            chain(
                                (RigidObjectCfg,),
                                self.__annotations__["asset_cfg"].__args__,
                            ),
                        ),
                    )
            elif not isinstance(self.__annotations__["asset_cfg"], RigidObjectCfg):
                new_annotation = functools.reduce(
                    operator.or_, (RigidObjectCfg, self.__annotations__["asset_cfg"])
                )
            if new_annotation is not None:
                self.__annotations__["asset_cfg"] = new_annotation
                self.model_fields["asset_cfg"].annotation = new_annotation
                self.model_rebuild(force=True)

            return asset_cfg
        elif isinstance(self.asset_cfg, RigidObjectCfg):
            return self.asset_cfg.copy()  # type: ignore
        else:
            raise TypeError(
                f"Cannot convert asset of type '{type(self.asset_cfg)}' to {RigidObjectCfg.__name__}"
            )

    def as_articulation_cfg(self) -> ArticulationCfg:
        if isinstance(self.asset_cfg, ArticulationCfg):
            return self.asset_cfg.copy()  # type: ignore
        else:
            raise TypeError(
                f"Cannot convert asset of type '{type(self.asset_cfg)}' to {ArticulationCfg.__name__}"
            )


class AssetRegistry:
    registry: ClassVar[Dict[AssetType, List[Type[Asset]]]] = {}
    base_types: ClassVar[Dict[AssetType, Type[Asset]]] = {}
    meta_types: ClassVar[List[Type[Asset]]] = []

    @classmethod
    def keys(cls) -> Iterable[AssetType]:
        return cls.registry.keys()

    @classmethod
    def items(cls) -> Iterable[Tuple[AssetType, Sequence[Type[Asset]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[Type[Asset]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[Type[Asset]]:
        return (asset for assets in cls.registry.values() for asset in assets)

    @classmethod
    def n_assets(cls) -> int:
        return sum(len(assets) for assets in cls.registry.values())

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {
            asset.__module__ for assets in cls.registry.values() for asset in assets
        }

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> Type[Asset] | None:
        for asset in cls.values_inner():
            if asset.name() == name:
                return asset
        return None
