from typing import ClassVar

import torch

from srb.core.action import (
    ActionGroup,
    JointEffortActionCfg,
    ThrustActionCfg,
    ThrusterCfg,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import (
    ArticulationCfg,
    Frame,
    OrbitalRobot,
    Transform,
)
from srb.core.command_mapping import CommandDestination, CommandMapping, Matrix
from srb.core.sensor import (
    CameraCfg,
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
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT

# Planar allocation (vx, vy, wz) for one-way thrusters; force opposes `direction`.
# A full 6-DoF pseudoinverse cancels once thrust is clamped to [0, 1].
_THRUST_ALLOCATION = (
    (0.0, -1.0, 0.0, 0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
    (-1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0, 0.0, 0.0, -1.0),
    (0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
    (0.0, -1.0, 0.0, 0.0, 0.0, -1.0),
    (1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0, 0.0, 0.0, -1.0),
)
_REACTION_WHEEL_ALLOCATION_ROW = (0.000, 0.000, 0.000, 0.000, 0.000, 0.525)
_THRUST_AND_REACTION_COMMAND_MAPPING = CommandMapping(
    operation=Matrix(values=(*_THRUST_ALLOCATION, _REACTION_WHEEL_ALLOCATION_ROW)),
    destinations=(
        CommandDestination(name="thrust", start=0, stop=8),
        CommandDestination(name="reaction_wheel_torque", start=8, stop=9),
    ),
)
_THRUST_ONLY_COMMAND_MAPPING = CommandMapping(
    operation=Matrix(values=_THRUST_ALLOCATION),
    destinations=(CommandDestination(name="thrust", start=0, stop=8),),
)


@configclass
class ThrustAndReactionWheelActionGroup(ActionGroup):
    thrust: ThrustActionCfg = ThrustActionCfg(
        asset_name="robot",
        thrusters=(
            # 1
            ThrusterCfg(
                offset=(-0.23758788, 0.02269813, 0.295),
                direction=(0.0, 1.41421356, 0.0),
                power=1.0,
                link_name="base_link",
            ),
            # 2
            ThrusterCfg(
                offset=(-0.23758788, -0.02269813, 0.305),
                direction=(0.0, -1.41421356, 0.0),
                power=1.0,
                link_name="base_link",
            ),
            # 3
            ThrusterCfg(
                offset=(0.02269813, 0.23758788, 0.295),
                direction=(1.41421356, 0.0, 0.0),
                power=1.0,
                link_name="base_link",
            ),
            # 4
            ThrusterCfg(
                offset=(-0.02269813, 0.23758788, 0.305),
                direction=(-1.41421356, 0.0, 0.0),
                power=1.0,
                link_name="base_link",
            ),
            # 5
            ThrusterCfg(
                offset=(0.23758788, -0.02269813, 0.305),
                direction=(0.0, -1.41421356, 0.0),
                power=1.0,
                link_name="base_link",
            ),
            # 6
            ThrusterCfg(
                offset=(0.23758788, 0.02269813, 0.295),
                direction=(0.0, 1.41421356, 0.0),
                power=1.0,
                link_name="base_link",
            ),
            # 7
            ThrusterCfg(
                offset=(-0.02269813, -0.23758788, 0.305),
                direction=(-1.41421356, 0.0, 0.0),
                power=1.0,
                link_name="base_link",
            ),
            # 8
            ThrusterCfg(
                offset=(0.02269813, -0.23758788, 0.295),
                direction=(1.41421356, 0.0, 0.0),
                power=1.0,
                link_name="base_link",
            ),
        ),
        fuel_capacity=0.01,
        fuel_consumption_rate=0.0,
    )
    reaction_wheel_torque: JointEffortActionCfg = JointEffortActionCfg(
        asset_name="robot",
        joint_names=[
            "rw_revolute_joint",
        ],
        scale=0.05,
    )

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_THRUST_AND_REACTION_COMMAND_MAPPING)


@configclass
class ThrustOnlyActionGroup(ActionGroup):
    thrust: ThrustActionCfg = ThrustAndReactionWheelActionGroup().thrust

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_THRUST_ONLY_COMMAND_MAPPING)


class FloatingPlatform(OrbitalRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/cubo",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("floating_platform.usdc")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.02, rest_offset=0.005
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                rigid_body_enabled=True,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=100.0,
                enable_gyroscopic_forces=True,
                # Set both explicitly: vacuum has no drag, and PhysX defaults
                # angular damping to 0.05.
                linear_damping=0.0,
                angular_damping=0.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
            semantic_tags=[("class", "spacecraft")],
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            # 0.1 m clearance above the platform collision surface; see
            # measurements/m2_vacuum_coast.py.
            pos=(0.0, 0.0, 0.1),
            joint_pos={
                ".*": 0.0,
            },
        ),
        actuators={
            "reaction_wheel": ImplicitActuatorCfg(
                joint_names_expr=["rw_revolute_joint"],
                effort_limit_sim=1.0,
                velocity_limit_sim=100.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "thrust_only": ThrustOnlyActionGroup(),
        "thrust_and_reaction": ThrustAndReactionWheelActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base_link",
        offset=Transform(
            pos=(0.0, 0.0, 0.70515),
            rot=rpy_to_quat(0.0, 180.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base_link",
        offset=Transform(
            rot=rpy_to_quat(0.0, 0.0, 45.0),
        ),
    )
    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/cubo/base_link",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/cubo/base_link/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.2, -0.2, 0.70515),
                rot=rpy_to_quat(0.0, 0.0, 45.0),
                # ROS convention would aim this deck camera at the ceiling.
                convention="world",
            ),
            spawn=PinholeCameraCfg(),
        ),
    }
