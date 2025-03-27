from srb.core.action import ActionGroup, ThrustActionCfg, ThrustActionGroup, ThrusterCfg
from srb.core.asset import Frame, Lander, RigidObjectCfg, Transform
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad, rpy_to_quat
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
            mass_props=MassPropertiesCfg(mass=4280.0),  # Dry mass
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                # Propulsion
                ThrusterCfg(
                    offset=(0.0, 0.0, 0.4142),
                    power=45100.0,
                ),
                # Attitude control
                ThrusterCfg(
                    offset=(1.74533, 1.82747, 4.9586),
                    direction=(0.0, 0.0, 1.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(1.76876, 2.0929, 4.64178),
                    direction=(0.0, 1.0, 0.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(2.00093, 1.85826, 4.64866),
                    direction=(1.0, 0.0, 0.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(1.75239, 1.83069, 4.32215),
                    direction=(0.0, 0.0, -1.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(1.72701, -1.74776, 4.95902),
                    direction=(0.0, 0.0, 1.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(2.02006, -1.72255, 4.65853),
                    direction=(1.0, 0.0, 0.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(1.80823, -2.01288, 4.65936),
                    direction=(0.0, -1.0, 0.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(1.72914, -1.74409, 4.3051),
                    direction=(0.0, 0.0, -1.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(-1.9284, -1.65115, 5.06328),
                    direction=(0.0, 0.0, 1.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(-1.9069, -1.91935, 4.5559),
                    direction=(0.0, -1.0, 0.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(-2.11252, -1.49807, 4.55269),
                    direction=(-1.0, 0.0, 0.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(-1.9284, -1.66499, 4.10111),
                    direction=(0.0, 0.0, -1.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(-1.93109, 1.70619, 5.05008),
                    direction=(0.0, 0.0, 1.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(-2.12702, 1.52566, 4.5559),
                    direction=(-1.0, 0.0, 0.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(-1.91524, 1.95915, 4.60795),
                    direction=(0.0, 1.0, 0.0),
                    power=445.0,
                ),
                ThrusterCfg(
                    offset=(-1.92403, 1.70298, 4.10203),
                    direction=(0.0, 0.0, -1.0),
                    power=445.0,
                ),
            ),
            # 20 seconds of full propulsion thrust, starting at 50% capacity
            fuel_capacity=0.5 * 10920.0,
            fuel_consumption_rate=(0.5 * 10920.0 / 45100.0) / 20.0,
        )
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
            mass_props=MassPropertiesCfg(mass=825.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                # Propulsion
                ThrusterCfg(
                    offset=(0.0, 0.0, 0.4448),
                    power=667.0,
                ),
                ThrusterCfg(
                    offset=(-0.21766, -0.21766, 0.4448),
                    power=667.0,
                ),
                ThrusterCfg(
                    offset=(-0.21766, 0.21766, 0.4448),
                    power=667.0,
                ),
                ThrusterCfg(
                    offset=(0.21766, -0.21766, 0.4448),
                    power=667.0,
                ),
                ThrusterCfg(
                    offset=(0.21766, 0.21766, 0.4448),
                    power=667.0,
                ),
                # Attitude control
                ThrusterCfg(
                    offset=(0.0038, -0.5947, 0.569),
                    direction=(0.0, 0.0, -1.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(-0.09, -0.647, 0.652),
                    direction=(-0.7, -1.0, 0.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(0.0942, -0.651, 0.6524),
                    direction=(0.7, -1.0, 0.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(0.0038, 0.5947, 0.569),
                    direction=(0.0, 0.0, -1.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(-0.09, 0.647, 0.652),
                    direction=(-0.7, 1.0, 0.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(0.0942, 0.651, 0.6524),
                    direction=(0.7, 1.0, 0.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(-1.05112, -0.00187, 1.7737),
                    direction=(0.0, 0.0, 1.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(-1.1074, 0.088521, 1.69),
                    direction=(-1.0, 0.7, 0.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(-1.1034, -0.095625, 1.69067),
                    direction=(-1.0, -0.7, 0.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(1.05112, -0.00187, 1.7737),
                    direction=(0.0, 0.0, 1.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(1.1074, 0.088521, 1.69),
                    direction=(1.0, 0.7, 0.0),
                    power=45.0,
                ),
                ThrusterCfg(
                    offset=(1.1034, -0.095625, 1.69067),
                    direction=(1.0, -0.7, 0.0),
                    power=45.0,
                ),
            ),
            fuel_capacity=0.5 * 450.0,
            fuel_consumption_rate=(0.5 * 450.0 / (5 * 667.0)) / 20.0,
        )
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
            mass_props=MassPropertiesCfg(mass=626.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                # Propulsion
                ThrusterCfg(
                    offset=(0.0, 0.0, 0.532305),
                    power=800.0,
                ),
                ThrusterCfg(
                    offset=(0.012183, -0.705708, 0.519355),
                    power=800.0,
                ),
                ThrusterCfg(
                    offset=(0.729607, 0.066033, 0.519355),
                    power=800.0,
                ),
                ThrusterCfg(
                    offset=(0.012183, 0.708475, 0.519355),
                    power=800.0,
                ),
                ThrusterCfg(
                    offset=(-0.738229, 0.066033, 0.519355),
                    power=800.0,
                ),
                # Attitude control
                ThrusterCfg(
                    offset=(-0.799375, 0.428285, 0.764497),
                    direction=(-1.0, 0.0, -0.84),
                    power=58.0,
                ),
                ThrusterCfg(
                    offset=(-0.807868, -0.364819, 0.760913),
                    direction=(-1.0, 0.0, -0.84),
                    power=58.0,
                ),
                ThrusterCfg(
                    offset=(-0.431845, -0.787548, 0.764497),
                    direction=(0.0, -1.0, -0.84),
                    power=58.0,
                ),
                ThrusterCfg(
                    offset=(0.418427, -0.776689, 0.760913),
                    direction=(0.0, -1.0, -0.84),
                    power=58.0,
                ),
                ThrusterCfg(
                    offset=(0.814197, -0.373962, 0.761142),
                    direction=(1.0, 0.0, -0.84),
                    power=58.0,
                ),
                ThrusterCfg(
                    offset=(0.806071, 0.419149, 0.760913),
                    direction=(1.0, 0.0, -0.84),
                    power=58.0,
                ),
                ThrusterCfg(
                    offset=(0.421697, 0.746937, 0.764498),
                    direction=(0.0, 1.0, -0.84),
                    power=58.0,
                ),
                ThrusterCfg(
                    offset=(-0.395689, 0.735653, 0.760913),
                    direction=(0.0, 1.0, -0.84),
                    power=58.0,
                ),
            ),
            fuel_capacity=0.5 * 845.0,
            fuel_consumption_rate=(0.5 * 845.0 / (5 * 800.0)) / 20.0,
        )
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
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                ThrusterCfg(
                    offset=(0.0, 0.0, 0.45),
                    power=50000.0,
                    gimbal_limits=(deg_to_rad(30.0), deg_to_rad(30.0)),
                ),
            ),
            fuel_capacity=5000.0,
            fuel_consumption_rate=(5000.0 / 50000.0) / 20.0,
        )
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
