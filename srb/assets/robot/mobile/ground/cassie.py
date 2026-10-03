from typing import ClassVar

from srb.core.action import (
    ActionGroup,
    JointPositionBoundedActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, LeggedRobot, Transform
from srb.core.sensor import (
    CameraCfg,
    ContactSensorCfg,
    ImuCfg,
    PinholeCameraCfg,
    SensorBaseCfg,
)
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class Cassie(LeggedRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/cassie",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("legged")
            .joinpath("cassie.usdz")
            .as_posix(),
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
            pos=(0.0, 0.0, 0.9),
            joint_pos={
                "hip_abduction_left": 0.1,
                "hip_rotation_left": 0.0,
                "hip_flexion_left": 1.0,
                "thigh_joint_left": -1.8,
                "ankle_joint_left": 1.57,
                "toe_joint_left": -1.57,
                "hip_abduction_right": -0.1,
                "hip_rotation_right": 0.0,
                "hip_flexion_right": 1.0,
                "thigh_joint_right": -1.8,
                "ankle_joint_right": 1.57,
                "toe_joint_right": -1.57,
            },
            joint_vel={".*": 0.0},
        ),
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=["hip_.*", "thigh_.*", "ankle_.*"],
                effort_limit_sim=200.0,
                velocity_limit_sim=10.0,
                stiffness={
                    "hip_abduction.*": 100.0,
                    "hip_rotation.*": 100.0,
                    "hip_flexion.*": 200.0,
                    "thigh_joint.*": 200.0,
                    "ankle_joint.*": 200.0,
                },
                damping={
                    "hip_abduction.*": 3.0,
                    "hip_rotation.*": 3.0,
                    "hip_flexion.*": 6.0,
                    "thigh_joint.*": 6.0,
                    "ankle_joint.*": 6.0,
                },
            ),
            "toes": ImplicitActuatorCfg(
                joint_names_expr=["toe_.*"],
                effort_limit_sim=20.0,
                velocity_limit_sim=10.0,
                stiffness={
                    "toe_joint.*": 20.0,
                },
                damping={
                    "toe_joint.*": 1.0,
                },
            ),
        },
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/cassie/pelvis",
        ),
        "contacts_feet": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/cassie/(left|right)_toe",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/cassie/pelvis/camera_front",
            # : was a copy of the Mars Rover mast camera
            # (2 m above the pelvis, looking sideways). Front-most `pelvis`
            # visual-mesh vertex on the centreline, (0.165, 0.0, 0.0358) m in
            # the pelvis frame of assets/srb_assets/robot/legged/cassie.usdz.
            # Optical axis along the link's +x, image up = +z (ROS optical frame).
            offset=CameraCfg.OffsetCfg(
                pos=(0.165, 0.0, 0.0358),
                rot=rpy_to_quat(-90.0, 0.0, -90.0),
                convention="ros",
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "joint_pos": JointPositionBoundedActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="pelvis")
    frame_payload_mount: Frame = Frame(
        prim_relpath="pelvis",
        offset=Transform(
            pos=(-0.125, 0.0, 0.1),
            rot=rpy_to_quat(0.0, -30.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="pelvis",
        offset=Transform(
            pos=(0.1, 0.0, 0.125),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    ## Links
    regex_feet_links: str = "(left|right)_toe"
