from typing import ClassVar

from srb.core.action import (
    ActionGroup,
    CoupledShovelVelocityActionCfg,
    ShovelJointVelocityActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import (
    ActivePayload,
    ArticulationCfg,
    Frame,
    Transform,
)
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    MassCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class RaphShovel(ActivePayload):
    """Raph blade with a 0.0–0.2 m lift joint and no curl.

    ``CURL_*`` remains ``None`` to declare its lift-only payload semantics.
    """

    JOINT_NAME: ClassVar[str] = "shovel_joint"
    CURL_JOINT_NAME: ClassVar[str | None] = None
    JOINT_POSITION_RANGE_M: ClassVar[tuple[float, float]] = (0.0, 0.2)
    JOINT_VELOCITY_MAX_M_S: ClassVar[float] = 0.04
    JOINT_STOWED_POSITION_M: ClassVar[float] = JOINT_POSITION_RANGE_M[1]
    CURL_POSITION_RANGE_RAD: ClassVar[tuple[float, float] | None] = None

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/raph_shovel",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_OBJECT.joinpath("raph_shovel.usdc").as_posix(),
            activate_contact_sensors=True,
            # Preserve the asset-authored convex-hull blade collider.
            collision_props=None,
            # The collider-free ``base`` lacks mass; author 0.25 kg so PhysX
            # derives its inertia instead of guessing from a negative mass.
            mass_props={"/base": [MassCfg(mass=0.25)]},
            mass_props_create_if_missing=True,
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        actuators={
            "driver": ImplicitActuatorCfg(
                joint_names_expr=[JOINT_NAME],
                velocity_limit_sim=JOINT_VELOCITY_MAX_M_S,
                effort_limit_sim=100.0,
                stiffness=0.0,
                damping=500.0,
            ),
        },
        # Start raised: the lower limit places the blade through the settled
        # bed, which would trigger depenetration before the first action.
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={JOINT_NAME: JOINT_STOWED_POSITION_M},
        ),
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "joint_vel": ShovelJointVelocityActionGroup(
            CoupledShovelVelocityActionCfg(
                asset_name="payload",
                lift_joint_name=JOINT_NAME,
                curl_joint_name=CURL_JOINT_NAME,
                lift_range_m=JOINT_POSITION_RANGE_M,
                curl_range_rad=CURL_POSITION_RANGE_RAD,
                lift_velocity_max_m_s=JOINT_VELOCITY_MAX_M_S,
            )
        ),
    }

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="base")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.0)))
