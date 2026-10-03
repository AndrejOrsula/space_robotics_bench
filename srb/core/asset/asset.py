from __future__ import annotations

import copy
from functools import cache, cached_property
from itertools import chain
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Iterable,
    Mapping,
    Sequence,
    Set,
)

from pydantic import BaseModel, PositiveFloat
from simforge import BlGeometry, BlModel, BlShader, TexResConfig

from srb.core.asset import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from srb.core.asset.asset_type import AssetType
from srb.core.asset.asset_variant import AssetVariant
from srb.core.asset.common import ThumbnailCameraPose
from srb.core.domain import Domain
from srb.core.registry_seam import register_asset_registry
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    ShapeCfg,
    SimforgeAssetCfg,
    SpawnerCfg,
    TerrainImporterCfg,
)
from srb.utils import logging
from srb.utils.str import convert_to_snake_case

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg

    # Proper type visible to static analysis (mypy / pyright).
    _AssetCfgType = AssetBaseCfg | RigidObjectCfg | ArticulationCfg
else:
    # At runtime Pydantic sees ``Any``, so it never tries to build schemas for
    # deeply-nested Isaac Lab ``@configclass`` types whose forward references
    # (hidden behind ``TYPE_CHECKING`` guards) cannot be resolved.
    _AssetCfgType = Any


class Asset(BaseModel):
    ## Scenario
    DOMAINS: ClassVar[Sequence[Domain]] = ()
    ASSET_VARIANT: ClassVar[AssetVariant | None] = None

    ## Model
    # NOTE: ``_AssetCfgType`` resolves to ``Any`` at runtime, preventing
    # Pydantic from walking the deeply-nested Isaac Lab ``@configclass`` type
    # tree.  Static analysers see the full union type instead.
    asset_cfg: _AssetCfgType

    ## Attributes forwarded to the spawner
    SPAWNER_ATTRIBUTES: ClassVar[Sequence[str]] = (
        "size",
        "scale",
        "texture_resolution",
    )
    scale: tuple[PositiveFloat, PositiveFloat, PositiveFloat] | None = None
    texture_resolution: TexResConfig | None = None

    ## Thumbnail
    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose | None] = None

    def __new__(cls, *args, **kwargs):
        if cls in (
            Asset,
            *AssetRegistry.base_types.values(),
            *AssetRegistry.meta_types,
        ):
            raise TypeError(f"Cannot instantiate abstract class {cls.__name__}")
        return super().__new__(cls)

    # Keyed by `cls`. Every asset class is already held for the process
    # lifetime by `AssetRegistry`, so this cache pins nothing the registry does
    # not already pin.
    @classmethod
    @cache
    def name(cls) -> str:
        return convert_to_snake_case(cls.__name__)

    @property
    def size(self) -> tuple[PositiveFloat, PositiveFloat] | None:
        if self.scale is not None:
            return (self.scale[0], self.scale[1])
        return None

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        """Add asset-specific scene elements or modify them before simulation starts."""
        pass

    def __init_subclass__(
        cls,
        asset_entrypoint: AssetType | None = None,
        asset_metaclass: bool = False,
        **kwargs,
    ):
        # Rewrite every non-ClassVar model-field annotation to ``Any`` before
        # Pydantic processes the model (triggered by ``super().__init_subclass__``).
        # This prevents Pydantic from recursing into deeply-nested Isaac Lab
        # ``@configclass`` / ``@dataclass`` type trees whose forward references
        # (hidden behind ``TYPE_CHECKING`` guards) cannot be resolved at runtime.
        # The real types are preserved for static analysis because type-checkers
        # evaluate annotations lazily and never execute this code path.
        #
        # The original annotations are preserved in ``__srb_annotations__`` so
        # that the Hydra asset resolver can still dispatch on the real field
        # types (e.g. ``Manipulator | _MISSING_TYPE``) at runtime.
        _own = cls.__dict__.get("__annotations__", {})
        _preserved: dict[str, Any] = {}
        for field_name, annotation in list(_own.items()):
            # Keep ClassVar annotations — they are not Pydantic model fields.
            if isinstance(annotation, str):
                if "ClassVar" in annotation:
                    continue
            elif getattr(annotation, "__origin__", None) is ClassVar:
                continue
            _preserved[field_name] = annotation
            _own[field_name] = Any
        if _preserved:
            setattr(cls, "__srb_annotations__", _preserved)

        super().__init_subclass__(**kwargs)
        if asset_entrypoint is not None:
            if not isinstance(asset_entrypoint, AssetType):
                raise TypeError(
                    f"Class '{cls.__name__}' is marked as an asset entrypoint, but '{asset_entrypoint}' is not a valid {AssetType}"
                )
            if asset_entrypoint in AssetRegistry.base_types:
                raise ValueError(
                    f"Class '{cls.__name__}' is marked as '{asset_entrypoint}' asset entrypoint, but it was already marked by '{AssetRegistry.base_types[asset_entrypoint].__name__}'"
                )
            AssetRegistry.base_types[asset_entrypoint] = cls
        elif asset_metaclass:
            AssetRegistry.meta_types.append(cls)
        else:
            for asset_type, base in AssetRegistry.base_types.items():
                if issubclass(cls, base):
                    if asset_type not in AssetRegistry.registry:
                        AssetRegistry.registry[asset_type] = []
                    else:
                        existing = next(
                            (
                                asset
                                for asset in AssetRegistry.registry[asset_type]
                                if asset.name() == cls.name()
                            ),
                            None,
                        )
                        if existing is not None:
                            raise ValueError(
                                f"Cannot register multiple assets with an identical name: '{cls.__module__}:{cls.__name__}' already exists as '{existing.__module__}:{existing.__name__}'"
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
    def asset_registry(cls) -> Sequence[type[Asset]]:
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
            if len(self.asset_cfg.spawn.assets_cfg) <= 0:
                raise ValueError(
                    "MultiAssetSpawnerCfg.assets_cfg is empty. "
                    "Add at least one asset configuration."
                )
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
                                    f'Input "{k}" of type "{type(v)}" not updated for "{asset.__class__.__name__}"'
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
                                    f'Input "{k}" of type "{type(v)}" not updated for "{asset.__class__.__name__}"'
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
                    _set = True
                if not _set:
                    logging.trace(
                        f'Input "{k}" of type "{type(v)}" not updated for "{spawner.__class__.__name__}"'
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
                        f'Input "{k}" of type "{type(v)}" not updated for "{spawner.__class__.__name__}"'
                    )

    def as_asset_base_cfg(
        self,
        disable_articulation: bool = True,
        disable_rigid_body: bool = True,
        remove_rigid_props: bool = False,
        disable_collision: bool = False,
    ) -> AssetBaseCfg:
        if isinstance(self.asset_cfg, (RigidObjectCfg, ArticulationCfg)):
            # Copy so stripping props below leaves this asset's own spawner intact.
            asset_cfg = AssetBaseCfg(
                prim_path=self.asset_cfg.prim_path,
                spawn=copy.deepcopy(self.asset_cfg.spawn),
                init_state=copy.deepcopy(self.asset_cfg.init_state),
            )

            def remove_props(attr: Any):
                for props in (
                    "deformable_props",
                    "fixed_tendons_props",
                    "joint_drive_props",
                    "mass_props",
                ):
                    if hasattr(attr, props):
                        setattr(attr, props, None)
                if hasattr(attr, "activate_contact_sensors"):
                    attr.activate_contact_sensors = False  # type: ignore[misc]
                if (
                    disable_articulation
                    and getattr(attr, "articulation_props", None) is not None
                ):
                    setattr(
                        attr,
                        "articulation_props",
                        ArticulationRootPropertiesCfg(articulation_enabled=False),
                    )
                if remove_rigid_props and hasattr(attr, "rigid_props"):
                    setattr(attr, "rigid_props", None)
                elif disable_rigid_body and hasattr(attr, "rigid_props"):
                    setattr(
                        attr,
                        "rigid_props",
                        RigidBodyPropertiesCfg(
                            kinematic_enabled=True,
                        ),
                    )
                if disable_collision:
                    if hasattr(attr, "collision_props"):
                        attr.collision_props = CollisionPropertiesCfg(
                            collision_enabled=False
                        )
                    if hasattr(attr, "mesh_collision_props"):
                        attr.mesh_collision_props = None

            remove_props(asset_cfg.spawn)
            if isinstance(asset_cfg.spawn, MultiAssetSpawnerCfg):
                for subspawner in asset_cfg.spawn.assets_cfg:
                    remove_props(subspawner)

            return asset_cfg
        elif isinstance(self.asset_cfg, (AssetBaseCfg, TerrainImporterCfg)):
            return self.asset_cfg.copy()  # type: ignore[attr-defined]
        else:
            raise TypeError(
                f"Cannot convert asset of type '{type(self.asset_cfg)}' to {AssetBaseCfg.__name__}"
            )


class AssetRegistry:
    registry: ClassVar[dict[AssetType, list[type[Asset]]]] = {}
    base_types: ClassVar[dict[AssetType, type[Asset]]] = {}
    meta_types: ClassVar[list[type[Asset]]] = []

    @classmethod
    def keys(cls) -> Iterable[AssetType]:
        return cls.registry.keys()

    @classmethod
    def items(cls) -> Iterable[tuple[AssetType, Sequence[type[Asset]]]]:
        return cls.registry.items()

    @classmethod
    def values(cls) -> Iterable[Iterable[type[Asset]]]:
        return cls.registry.values()

    @classmethod
    def values_inner(cls) -> Iterable[type[Asset]]:
        return (asset for assets in cls.registry.values() for asset in assets)

    @classmethod
    def registered_modules(cls) -> Iterable[str]:
        return {
            asset.__module__ for assets in cls.registry.values() for asset in assets
        }

    @classmethod
    def registered_packages(cls) -> Iterable[str]:
        return {module.split(".", maxsplit=1)[0] for module in cls.registered_modules()}

    @classmethod
    def get_by_name(cls, name: str) -> type[Asset] | None:
        for asset in cls.values_inner():
            if asset.name() == name:
                return asset
        return None


# Append-only and populated by `__init_subclass__`; declared as part of the
# supported snapshot/restore seam in `srb.core.registry_seam`.
register_asset_registry(AssetRegistry)
