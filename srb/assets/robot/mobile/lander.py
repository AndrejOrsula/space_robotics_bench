from srb.core.action import (
    ActionGroup,
    BodyAccelerationActionCfg,
    BodyAccelerationActionGroup,
)
from srb.core.asset import Frame, Lander, RigidObjectCfg, Transform
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class ApolloLander(Lander):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lander",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("lander")
            .joinpath("apollo.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = BodyAccelerationActionGroup(
        BodyAccelerationActionCfg(asset_name="robot", scale=0.05)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )


class PeregrineLander(Lander):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lander",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("lander")
            .joinpath("peregrine.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = BodyAccelerationActionGroup(
        BodyAccelerationActionCfg(asset_name="robot", scale=0.05)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )


class VikramLander(Lander):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/lander",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("lander")
            .joinpath("vikram.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = BodyAccelerationActionGroup(
        BodyAccelerationActionCfg(asset_name="robot", scale=0.05)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )


class RandomLander(Lander):
    asset_cfg: RigidObjectCfg = ApolloLander().asset_cfg.copy()  # type: ignore
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal"
    asset_cfg.spawn = MultiAssetSpawnerCfg(
        random_choice=False,
        assets_cfg=(
            ApolloLander().asset_cfg.spawn,  # type: ignore
            PeregrineLander().asset_cfg.spawn,  # type: ignore
            VikramLander().asset_cfg.spawn,  # type: ignore
        ),
        activate_contact_sensors=True,
    )

    ## Actions
    actions: ActionGroup = BodyAccelerationActionGroup(
        BodyAccelerationActionCfg(asset_name="robot", scale=0.05)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
