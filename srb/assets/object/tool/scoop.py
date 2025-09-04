import simforge_foundry

from srb.core.asset import Frame, RigidObjectCfg, Tool, Transform
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    SimforgeAssetCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT

SRB_ASSETS_DIR_SRB_OBJECT_SCOOP = SRB_ASSETS_DIR_SRB_OBJECT.joinpath("scoop")


class Scoop(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.ScoopRandom()],
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.001),
        ),
    )
    asset_cfg.spawn.assets[0].geo.ops[0].mount_radius = 0.0375  # type: ignore

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="scoop_random")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.1)))


class ScoopRectangular(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT_SCOOP.joinpath(
                    "scoop_rectangular.usdz"
                ).as_posix()
            ),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.001),
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="scoop")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.1)))


class ScoopSpherical(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT_SCOOP.joinpath(
                    "scoop_spherical.usdz"
                ).as_posix()
            ),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.001),
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="scoop")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.1)))


class ScoopTriangular(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT_SCOOP.joinpath(
                    "scoop_triangular.usdz"
                ).as_posix()
            ),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.001),
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="scoop")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.1)))


class ScoopCustom1(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT_SCOOP.joinpath(
                    "scoop_custom1.usdz"
                ).as_posix()
            ),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.001),
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="scoop")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.1)))


class ScoopCustom2(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT_SCOOP.joinpath(
                    "scoop_custom2.usdz"
                ).as_posix()
            ),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.001),
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="scoop")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.1)))


class ScoopCustom3(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT_SCOOP.joinpath(
                    "scoop_custom3.usdz"
                ).as_posix()
            ),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.001),
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="scoop")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.1)))


class RandomScoop(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=MultiAssetSpawnerCfg(
            assets_cfg=[
                UsdFileCfg(
                    usd_path=(
                        SRB_ASSETS_DIR_SRB_OBJECT_SCOOP.joinpath(
                            scoop_filename
                        ).as_posix()
                    ),
                    activate_contact_sensors=True,
                    collision_props=CollisionPropertiesCfg(),
                    mesh_collision_props=MeshCollisionPropertiesCfg(
                        mesh_approximation="sdf"
                    ),
                    rigid_props=RigidBodyPropertiesCfg(),
                    mass_props=MassPropertiesCfg(mass=0.001),
                )
                for scoop_filename in (
                    "scoop_rectangular.usdz",
                    "scoop_spherical.usdz",
                    "scoop_triangular.usdz",
                    # "scoop_custom1.usdz",
                    "scoop_custom2.usdz",
                    "scoop_custom3.usdz",
                )
            ],
        ),
    )
    ## Frames
    frame_mount: Frame = Frame(prim_relpath="scoop")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.1)))
