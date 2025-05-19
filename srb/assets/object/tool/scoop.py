from srb.core.asset import Frame, RigidObjectCfg, Tool, Transform
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class ScoopRectangular(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/scoop",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("scoop_rectangular.usdz").as_posix()
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
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("scoop_spherical.usdz").as_posix()
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
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("scoop_triangular.usdz").as_posix()
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
