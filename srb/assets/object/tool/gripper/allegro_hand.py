from typing import ClassVar

from srb.core.action import ActionGroup, JointPositionBoundedActionGroup
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ActiveTool, ArticulationCfg, Frame, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.nucleus import ISAAC_NUCLEUS_DIR


class AllegroHand(ActiveTool):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/allegro_hand",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/WonikRobotics/AllegroHand/allegro_hand_instanceable.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False,
                enable_gyroscopic_forces=False,
                angular_damping=0.01,
                max_linear_velocity=1000.0,
                max_angular_velocity=3665,
                max_depenetration_velocity=1000.0,
                max_contact_impulse=1e32,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.5),
            rot=(0.283045, 0.683330, -0.621782, 0.257551),  # xyzw
            joint_pos={"^(?!thumb_joint_0).*": 0.0, "thumb_joint_0": 0.28},
        ),
        actuators={
            "fingers": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit_sim=0.5,
                velocity_limit_sim=100.0,
                stiffness=3.0,
                damping=0.1,
                friction=0.01,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "joint_pos": JointPositionBoundedActionGroup(),
    }

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="allegro_mount")
    frame_tool_centre_point: Frame = Frame(
        prim_relpath="allegro_mount", offset=Transform(pos=(0.0, 0.0, 0.15))
    )
