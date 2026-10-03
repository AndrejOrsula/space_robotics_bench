from typing import Sequence

from srb.core.asset import ModularElement, RigidObjectCfg
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    PreviewSurfaceCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


def _lego_block_spawn_cfg(
    filename: str, color: tuple[float, float, float]
) -> UsdFileCfg:
    return UsdFileCfg(
        usd_path=SRB_ASSETS_DIR_SRB_OBJECT.joinpath("lego", filename).as_posix(),
        collision_props=CollisionPropertiesCfg(),
        mesh_collision_props=MeshCollisionPropertiesCfg(
            mesh_approximation="sdf",
            sdf_resolution=128,
        ),
        rigid_props=RigidBodyPropertiesCfg(),
        mass_props=MassPropertiesCfg(density=800.0),
        visual_material=PreviewSurfaceCfg(diffuse_color=color),
    )


class LegoBlock_1x1(ModularElement):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lego_block_1x1",
        spawn=_lego_block_spawn_cfg("block_1x1.usdc", (0.8, 0.2, 0.6)),
    )

    pegs: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0096),)
    holes: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0),)
    holes_entrance_offset: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0016),)


class LegoBlock_1x2(ModularElement):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lego_block_1x2",
        spawn=_lego_block_spawn_cfg("block_1x2.usdc", (0.2, 0.6, 0.6)),
    )

    pegs: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0096),)
    holes: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0),)
    holes_entrance_offset: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0016),)


class LegoBlock_1x3(ModularElement):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lego_block_1x3",
        spawn=_lego_block_spawn_cfg("block_1x3.usdc", (0.8, 0.2, 0.2)),
    )

    pegs: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0096),)
    holes: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0),)
    holes_entrance_offset: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0016),)


class LegoBlock_1x4(ModularElement):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lego_block_1x4",
        spawn=_lego_block_spawn_cfg("block_1x4.usdc", (0.6, 0.2, 0.2)),
    )

    pegs: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0096),)
    holes: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0),)
    holes_entrance_offset: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0016),)


class LegoBlock_2x2(ModularElement):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lego_block_2x2",
        spawn=_lego_block_spawn_cfg("block_2x2.usdc", (0.4, 0.4, 0.2)),
    )

    pegs: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0096),)
    holes: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0),)
    holes_entrance_offset: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0016),)


class LegoBlock_2x3(ModularElement):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lego_block_2x3",
        spawn=_lego_block_spawn_cfg("block_2x3.usdc", (0.2, 0.6, 0.2)),
    )

    pegs: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0096),)
    holes: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0),)
    holes_entrance_offset: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0016),)


class LegoBlock_2x4(ModularElement):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lego_block_2x4",
        spawn=_lego_block_spawn_cfg("block_2x4.usdc", (0.2, 0.8, 0.2)),
    )

    pegs: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0096),)
    holes: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0),)
    holes_entrance_offset: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0016),)


class RandomLegoBlock(ModularElement):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lego_block",
        spawn=MultiAssetSpawnerCfg(
            assets_cfg=[  # type: ignore[union-attr]
                LegoBlock_1x1().asset_cfg.spawn,
                LegoBlock_1x2().asset_cfg.spawn,
                LegoBlock_1x3().asset_cfg.spawn,
                LegoBlock_1x4().asset_cfg.spawn,
                LegoBlock_2x2().asset_cfg.spawn,
                LegoBlock_2x3().asset_cfg.spawn,
                LegoBlock_2x4().asset_cfg.spawn,
            ],
        ),
    )

    # Every shipped block shares the same stud geometry, so any sampled variant
    # assembles with these offsets.
    pegs: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0096),)
    holes: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0),)
    holes_entrance_offset: Sequence[tuple[float, float, float]] = ((0.0, 0.0, 0.0016),)
