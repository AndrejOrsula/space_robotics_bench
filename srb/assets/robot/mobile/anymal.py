from dataclasses import MISSING

from srb.core.action import (
    ActionGroup,
    JointPositionActionCfg,
    JointPositionActionGroup,
)
from srb.core.actuator import ActuatorNetLSTMCfg
from srb.core.asset import ArticulationCfg, Frame, LeggedRobot, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.nucleus import ISAACLAB_NUCLEUS_DIR


class AnymalC(LeggedRobot):
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path=MISSING,  # type: ignore
        spawn=UsdFileCfg(
            usd_path=MISSING,  # type: ignore
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.6),
            joint_pos={
                ".*HAA": 0.0,
                ".*F_HFE": 0.4,
                ".*H_HFE": -0.4,
                ".*F_KFE": -0.8,
                ".*H_KFE": 0.8,
            },
        ),
        actuators={  # type: ignore
            # "legs": DCMotorCfg(
            #     joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
            #     saturation_effort=120.0,
            #     effort_limit=80.0,
            #     velocity_limit=7.5,
            #     stiffness={".*": 40.0},
            #     damping={".*": 5.0},
            # ),
            "legs": ActuatorNetLSTMCfg(
                joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
                network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt",
                saturation_effort=120.0,
                effort_limit=80.0,
                velocity_limit=7.5,
            ),
        },
    )
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal_c"
    asset_cfg.spawn.usd_path = (  # type: ignore
        f"{ISAACLAB_NUCLEUS_DIR}/Robots/ANYbotics/ANYmal-C/anymal_c.usd"
    )

    ## Actions
    action_cfg: ActionGroup = JointPositionActionGroup(
        JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(-0.075, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.275, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="base/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*FOOT"


class AnymalD(LeggedRobot):
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path=MISSING,  # type: ignore
        spawn=UsdFileCfg(
            usd_path=MISSING,  # type: ignore
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.6),
            joint_pos={
                ".*HAA": 0.0,
                ".*F_HFE": 0.4,
                ".*H_HFE": -0.4,
                ".*F_KFE": -0.8,
                ".*H_KFE": 0.8,
            },
        ),
        actuators={  # type: ignore
            # "legs": DCMotorCfg(
            #     joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
            #     saturation_effort=120.0,
            #     effort_limit=80.0,
            #     velocity_limit=7.5,
            #     stiffness={".*": 40.0},
            #     damping={".*": 5.0},
            # ),
            "legs": ActuatorNetLSTMCfg(
                joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
                network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt",
                saturation_effort=120.0,
                effort_limit=80.0,
                velocity_limit=7.5,
            ),
        },
    )
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal_d"
    asset_cfg.spawn.usd_path = (  # type: ignore
        f"{ISAACLAB_NUCLEUS_DIR}/Robots/ANYbotics/ANYmal-D/anymal_d.usd"
    )

    ## Actions
    action_cfg: ActionGroup = JointPositionActionGroup(
        JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(-0.075, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.275, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="base/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*FOOT"


class RandomAnymalQuadruped(LeggedRobot):
    asset_cfg: ArticulationCfg = AnymalD().asset_cfg.copy()  # type: ignore
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal"
    asset_cfg.spawn = MultiAssetSpawnerCfg(
        random_choice=False,
        assets_cfg=(
            AnymalC().asset_cfg.spawn,  # type: ignore
            AnymalD().asset_cfg.spawn,  # type: ignore
        ),
        activate_contact_sensors=True,
    )

    ## Actions
    action_cfg: ActionGroup = JointPositionActionGroup(
        JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(-0.075, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.275, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="base/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*FOOT"
