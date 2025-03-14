from srb.core.action import (
    ActionGroup,
    JointPositionActionCfg,
    JointPositionActionGroup,
)
from srb.core.actuator import ActuatorNetMLPCfg, DCMotorCfg
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


class UnitreeA1(LeggedRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/unitree_a1",
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/A1/a1.usd",
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
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.42),
            joint_pos={
                ".*L_hip_joint": 0.1,
                ".*R_hip_joint": -0.1,
                "F[L,R]_thigh_joint": 0.8,
                "R[L,R]_thigh_joint": 1.0,
                ".*_calf_joint": -1.5,
            },
            joint_vel={".*": 0.0},
        ),
        actuators={
            "base_legs": DCMotorCfg(
                joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
                effort_limit=33.5,
                saturation_effort=33.5,
                velocity_limit=21.0,
                stiffness=25.0,
                damping=0.5,
                friction=0.0,
            ),
        },
    )

    ## Actions
    action_cfg: ActionGroup = JointPositionActionGroup(
        JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="trunk")
    frame_payload_mount: Frame = Frame(
        prim_relpath="trunk",
        offset=Transform(
            pos=(-0.1, 0.0, 0.06),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="trunk",
        offset=Transform(
            pos=(0.15, 0.0, 0.06),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="trunk/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*foot"


class UnitreeGo1(LeggedRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/unitree_go1",
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go1/go1.usd",
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
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.4),
            joint_pos={
                ".*L_hip_joint": 0.1,
                ".*R_hip_joint": -0.1,
                "F[L,R]_thigh_joint": 0.8,
                "R[L,R]_thigh_joint": 1.0,
                ".*_calf_joint": -1.5,
            },
            joint_vel={".*": 0.0},
        ),
        actuators={
            "base_legs": ActuatorNetMLPCfg(
                joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
                network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/Unitree/unitree_go1.pt",
                pos_scale=-1.0,
                vel_scale=1.0,
                torque_scale=1.0,
                input_order="pos_vel",
                input_idx=[0, 1, 2],
                effort_limit=23.7,
                velocity_limit=30.0,
                saturation_effort=23.7,
            ),
        },
    )

    ## Actions
    action_cfg: ActionGroup = JointPositionActionGroup(
        JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="trunk")
    frame_payload_mount: Frame = Frame(
        prim_relpath="trunk",
        offset=Transform(
            pos=(-0.1, 0.0, 0.06),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="trunk",
        offset=Transform(
            pos=(0.15, 0.0, 0.06),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="trunk/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*foot"


class UnitreeGo2(LeggedRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/unitree_go2",
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go2/go2.usd",
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
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.42),
            joint_pos={
                ".*L_hip_joint": 0.1,
                ".*R_hip_joint": -0.1,
                "F[L,R]_thigh_joint": 0.8,
                "R[L,R]_thigh_joint": 1.0,
                ".*_calf_joint": -1.5,
            },
            joint_vel={".*": 0.0},
        ),
        actuators={
            "base_legs": DCMotorCfg(
                joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
                effort_limit=33.5,
                saturation_effort=33.5,
                velocity_limit=21.0,
                stiffness=25.0,
                damping=0.5,
                friction=0.0,
            ),
        },
    )

    ## Actions
    action_cfg: ActionGroup = JointPositionActionGroup(
        JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="trunk")
    frame_payload_mount: Frame = Frame(
        prim_relpath="trunk",
        offset=Transform(
            pos=(-0.1, 0.0, 0.06),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="trunk",
        offset=Transform(
            pos=(0.15, 0.0, 0.06),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="trunk/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*foot"


class RandomUnitreeQuadruped(LeggedRobot):
    asset_cfg: ArticulationCfg = UnitreeGo2().asset_cfg.copy()  # type: ignore
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal"
    asset_cfg.spawn = MultiAssetSpawnerCfg(
        random_choice=False,
        assets_cfg=(
            UnitreeA1().asset_cfg.spawn,  # type: ignore
            UnitreeGo1().asset_cfg.spawn,  # type: ignore
            UnitreeGo2().asset_cfg.spawn,  # type: ignore
        ),
        activate_contact_sensors=True,
    )

    ## Actions
    action_cfg: ActionGroup = JointPositionActionGroup(
        JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="trunk")
    frame_payload_mount: Frame = Frame(
        prim_relpath="trunk",
        offset=Transform(
            pos=(-0.1, 0.0, 0.06),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="trunk",
        offset=Transform(
            pos=(0.15, 0.0, 0.06),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="trunk/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*foot"
