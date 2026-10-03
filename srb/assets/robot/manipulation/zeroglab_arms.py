from typing import ClassVar

from srb.core.action import (  # noqa: F401
    ActionGroup,
    DifferentialIKControllerCfg,
    DifferentialInverseKinematicsActionCfg,
    InverseKinematicsActionGroup,
    JointEffortActionCfg,
    JointEffortActionGroup,
    JointPositionRelativeActionGroup,
    OperationalSpaceControlActionGroup,
    OperationalSpaceControllerActionCfg,
    OperationalSpaceControllerCfg,
    RelativeJointPositionActionCfg,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import (
    ArticulationCfg,
    Frame,
    MultiManipulator,
)
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.cfg import configclass
from srb.utils.math import radians
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


@configclass
class DualArmOSCActionGroup(ActionGroup):
    arm_ceiling: OperationalSpaceControllerActionCfg = (
        OperationalSpaceControllerActionCfg(
            asset_name="arm_ceiling",
            joint_names=[
                "rail_joint",
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            body_name="wrist_3_link",
            controller_cfg=OperationalSpaceControllerCfg(
                target_types=["pose_rel"],
                impedance_mode="variable_kp",
                motion_stiffness_limits_task=(10.0, 250.0),
                motion_damping_ratio_task=1.0,
                nullspace_control="none",
                inertial_dynamics_decoupling=True,
            ),
            nullspace_joint_pos_target="none",
            position_scale=0.1,
            orientation_scale=0.1,
            stiffness_scale=120.0,
            body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
        )
    )
    arm_wall: OperationalSpaceControllerActionCfg = OperationalSpaceControllerActionCfg(
        asset_name="arm_wall",
        joint_names=[
            "rail_joint",
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
        body_name="wrist_3_link",
        controller_cfg=OperationalSpaceControllerCfg(
            target_types=["pose_rel"],
            impedance_mode="variable_kp",
            motion_stiffness_limits_task=(10.0, 250.0),
            motion_damping_ratio_task=1.0,
            nullspace_control="none",
            inertial_dynamics_decoupling=True,
        ),
        nullspace_joint_pos_target="none",
        position_scale=0.1,
        orientation_scale=0.1,
        stiffness_scale=120.0,
        body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    )


class ZeroglabArms(MultiManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/zeroglab_arms",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("manipulator")
            .joinpath("zerog_arms.usdc")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "rail_joint": 2.0,
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": radians(-90.0),
                "elbow_joint": radians(90.0),
                "wrist_1_joint": radians(-90.0),
                "wrist_2_joint": radians(-90.0),
                "wrist_3_joint": radians(-90.0),
            },
        ),
        actuators={
            "rail": ImplicitActuatorCfg(
                joint_names_expr=["rail_joint"],
                velocity_limit_sim=0.2,
                effort_limit_sim=400.0,
                stiffness=3000.0,
                damping=2500.0,
            ),
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*shoulder.*", ".*elbow.*", ".*wrist.*"],
                velocity_limit_sim=100.0,
                effort_limit_sim=87.0,
                stiffness=800.0,
                damping=40.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "dual_arm_osc": DualArmOSCActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="_virtual_world")
