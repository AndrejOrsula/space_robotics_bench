from typing import ClassVar

from srb.core.asset import AssetBaseCfg, Frame, Object, RigidObjectCfg, Transform
from srb.core.asset.common.thumbnail import ThumbnailCameraPose
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class BoltM8(Object):
    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose] = ThumbnailCameraPose(
        eye=(0.0, -0.08, 0.05),
        lookat=(0.0, 0.0, 0.02),
    )

    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/peg",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("bolt_and_nut")
                .joinpath("bolt_hex_m8.usdz")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="sdf",
                sdf_resolution=512,
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.1),
        ),
    )

    ## Frames
    frame_driver_slot: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.01905)))


class BoltM8Tight(Object):
    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose] = ThumbnailCameraPose(
        eye=(0.0, -0.08, 0.05),
        lookat=(0.0, 0.0, 0.02),
    )

    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/peg",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("bolt_and_nut")
                .joinpath("bolt_hex_m8_tight.usdz")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="sdf",
                sdf_resolution=512,
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.1),
        ),
    )

    ## Frames
    frame_driver_slot: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.01905)))


class NutM8(Object):
    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose] = ThumbnailCameraPose(
        eye=(0.0, -0.05, 0.025),
        lookat=(0.0, 0.0, 0.003),
    )

    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/hole",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("bolt_and_nut")
                .joinpath("nut_hex_m8.usdz")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
        ),
    )


class NutM8Tight(Object):
    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose] = ThumbnailCameraPose(
        eye=(0.0, -0.05, 0.025),
        lookat=(0.0, 0.0, 0.003),
    )

    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/hole",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("bolt_and_nut")
                .joinpath("nut_hex_m8_tight.usdz")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
        ),
    )
