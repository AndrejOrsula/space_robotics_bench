from typing import TYPE_CHECKING

from isaaclab.sim.spawners.wrappers import MultiAssetSpawnerCfg
from pxr import Usd

from srb.core.sim import CapsuleCfg, ConeCfg, CuboidCfg, CylinderCfg, SphereCfg
from srb.core.sim.spawners.multi_asset_compat import (
    spawn_multi_asset_compat as spawn_multi_asset,
)
from srb.utils.config_introspection import iter_config_items

if TYPE_CHECKING:
    from .cfg import MultiShapeSpawnerCfg

DEFAULT_SHAPES = ("cuboid", "sphere", "cylinder", "capsule", "cone")

IGNORED_SPAWN_ATTRIBUTES = (
    "func",
    "shapes",
    "scale",
    "radius",
    "height",
    "axis",
    "random_choice",
)


def _shape_spawner_kwargs(cfg: "MultiShapeSpawnerCfg") -> dict[str, object]:
    return {
        attr_name: attr_value
        for attr_name, attr_value in iter_config_items(cfg)
        if attr_name not in IGNORED_SPAWN_ATTRIBUTES
    }


def multi_shape_asset_cfgs(cfg: "MultiShapeSpawnerCfg") -> list[object]:
    """Build the concrete shape spawner configs represented by ``cfg``."""
    shape_cfg_kwargs = _shape_spawner_kwargs(cfg)
    shapes = cfg.shapes or DEFAULT_SHAPES

    assets_cfg = []
    if "cuboid" in shapes:
        assets_cfg.append(CuboidCfg(size=cfg.scale, **shape_cfg_kwargs))
    if "sphere" in shapes:
        assets_cfg.append(
            SphereCfg(radius=cfg.radius or cfg.scale[0], **shape_cfg_kwargs)
        )
    if "cylinder" in shapes:
        assets_cfg.append(
            CylinderCfg(
                radius=cfg.radius or cfg.scale[0],
                height=cfg.height or cfg.scale[1],
                axis=cfg.axis,
                **shape_cfg_kwargs,
            )
        )
    if "capsule" in shapes:
        assets_cfg.append(
            CapsuleCfg(
                radius=cfg.radius or cfg.scale[0],
                height=cfg.height or cfg.scale[1],
                axis=cfg.axis,
                **shape_cfg_kwargs,
            )
        )
    if "cone" in shapes:
        assets_cfg.append(
            ConeCfg(
                radius=cfg.radius or cfg.scale[0],
                height=cfg.height or cfg.scale[1],
                axis=cfg.axis,
                **shape_cfg_kwargs,
            )
        )
    return assets_cfg


def spawn_multi_shape(
    prim_path: str,
    cfg: "MultiShapeSpawnerCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    # Create and spawn multi-asset configuration
    return spawn_multi_asset(
        prim_path=prim_path,
        cfg=MultiAssetSpawnerCfg(
            assets_cfg=multi_shape_asset_cfgs(cfg),
            random_choice=cfg.random_choice,
            activate_contact_sensors=getattr(cfg, "activate_contact_sensors", False),
        ),
        translation=translation,
        orientation=orientation,
    )
