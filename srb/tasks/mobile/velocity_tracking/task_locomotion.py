from dataclasses import MISSING

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import AssetVariant, Humanoid, LeggedRobot
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import (
    randomize_command,
    reset_joints_by_scale,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import LinearRewardTerm, ThresholdRewardTerm
from srb.core.reward.locomotion import (
    compute_foot_slip_metric,
    compute_locomotion_safety_termination,
    compute_quadruped_trot_metric,
    compute_stance_air_time_metric,
    find_quadruped_trot_pairs,
    resolve_feet_contact_indices,
)
from srb.core.scene_introspection import get_scene_sensor
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, normalize_joint_pos, rotmat_to_rot6d

from .task import EventCfg, SceneCfg, Task, TaskCfg
from .task import RewardCfg as VelTrackingRewardCfg

##############
### Config ###
##############


class LocomotionRewardCfg(VelTrackingRewardCfg):
    """Velocity-tracking rewards extended with legged-locomotion terms."""

    ## Locomotion-specific penalties
    penalty_joint_torque: LinearRewardTerm = LinearRewardTerm(
        weight=-0.000025, limit=4.0
    )
    """Penalises joint torque with a very small weight for legged robots."""

    penalty_joint_acceleration: LinearRewardTerm = LinearRewardTerm(
        weight=-0.00000025, limit=2.0
    )
    """Penalises joint acceleration to smooth gait."""

    penalty_undesired_robot_contacts: ThresholdRewardTerm = ThresholdRewardTerm(
        weight=-2.0, threshold=1.0, operator=">"
    )
    """Penalises non-foot contacts."""

    penalty_undesired_lin_vel_z: LinearRewardTerm = LinearRewardTerm(
        weight=-0.5, limit=4.0
    )
    """Penalises vertical velocity."""

    penalty_undesired_ang_vel_xy: LinearRewardTerm = LinearRewardTerm(
        weight=-0.1, limit=4.0
    )
    """Penalises roll and pitch angular velocity."""

    penalty_gravity_rotation_alignment: LinearRewardTerm = LinearRewardTerm(
        weight=-2.0, limit=8.0
    )
    """Penalises deviation from upright orientation."""

    ## Locomotion-specific rewards
    reward_feet_air_time: LinearRewardTerm = LinearRewardTerm(weight=0.75, limit=2.0)
    """Rewards stable stance at rest and bounded stance/air phases in motion."""

    reward_quadruped_trot: LinearRewardTerm = LinearRewardTerm(weight=1.0, limit=1.0)
    """Rewards diagonal-pair timing when foot names resolve."""

    penalty_foot_slip: LinearRewardTerm = LinearRewardTerm(weight=-0.5, limit=2.0)
    """Penalises loaded foot slip."""

    ## Shaping parameters
    mode_time_feet_air_time: float = 0.3
    """Target stance/air phase duration used by foot timing rewards."""

    threshold_feet_air_time: float = 0.1
    """Minimum command/body speed to activate moving gait rewards."""

    threshold_foot_contact_force: float = 1.0
    """Minimum foot contact force used for slip and body-contact decisions."""

    std_quadruped_trot: float = 0.1
    """Exponential kernel width for diagonal-pair gait timing."""

    max_err_quadruped_trot: float = 0.2
    """Maximum timing error considered by the quadruped trot metric."""

    terminate_on_undesired_contacts: bool = True
    """Terminate when non-foot bodies contact terrain."""

    terminate_on_fall: bool = True
    """Terminate when projected gravity indicates a large body tilt."""

    max_projected_gravity_xy: float = 0.85
    """Projected-gravity xy norm above which the robot is considered fallen."""


@configclass
class LocomotionSceneCfg(SceneCfg):
    contacts_robot: ContactSensorCfg = ContactSensorCfg(
        prim_path=MISSING,  # type: ignore[arg-type]
        update_period=0.0,
        history_length=3,
        track_air_time=True,
    )


@configclass
class LocomotionEventCfg(EventCfg):
    command = SrbEventTermCfg(
        func=randomize_command,
        mode="interval",
        interval_range_s=(2.0, 6.0),
        category=RandomizationCategory.TASK,
        params={
            "env_attr_name": "_command",
            "magnitude": 0.5,
        },
    )

    randomize_robot_joints: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_joints_by_scale,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )
    # See `docs/src/misc/modelling_envelope.md` for why disturbances are omitted.


@configclass
class LocomotionTaskCfg(TaskCfg):
    ## Assets
    robot: LeggedRobot | Humanoid | AssetVariant = assets.Spot()
    _robot: LeggedRobot = MISSING  # type: ignore[assignment]
    ## Scene
    scene: LocomotionSceneCfg = LocomotionSceneCfg()

    ## Events
    events: LocomotionEventCfg = LocomotionEventCfg()

    ## Reward
    reward: LocomotionRewardCfg = LocomotionRewardCfg()

    ## Time
    # 200 Hz is required by the timestep-convergence sweep and matches the
    # waypoint-navigation locomotion task.
    env_rate: float = 1.0 / 200.0

    def __post_init__(self):
        super().__post_init__()

        # Sensor: Robot contacts
        self.scene.contacts_robot.prim_path = f"{self.scene.robot.prim_path}/.*"


############
### Task ###
############


class _LocomotionTaskMixin:
    cfg: LocomotionTaskCfg

    def extract_step_return(self) -> StepReturn:
        if self.cfg.command_vis or self.cfg.debug_vis:
            self._update_visualization_markers()

        rew = self.cfg.reward
        num_envs = self.num_envs
        device = self.device

        ## Actions
        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        ## Root state
        tf_quat_robot = self._robot.data.root_quat_w
        vel_lin_robot = self._robot.data.root_lin_vel_b
        vel_ang_robot = self._robot.data.root_ang_vel_b
        projected_gravity_robot = _to_torch(self._robot.data.projected_gravity_b)
        body_vel_w_robot = _to_torch(self._robot.data.body_vel_w)

        ## Joints
        joint_pos_robot = self._robot.data.joint_pos
        joint_acc_robot = self._robot.data.joint_acc
        joint_applied_torque_robot = self._robot.data.applied_torque

        ## Contacts
        contact_forces_robot = _to_torch(self._contacts_robot.data.net_forces_w)  # type: ignore[union-attr]
        contact_time_fallback = torch.zeros(
            contact_forces_robot.shape[:2],
            device=device,
            dtype=contact_forces_robot.dtype,
        )
        contact_current_air_time = _get_contact_time_series(
            self._contacts_robot.data,
            "current_air_time",
            "last_air_time",
            fallback=contact_time_fallback,
        )
        contact_current_contact_time = _get_contact_time_series(
            self._contacts_robot.data,
            "current_contact_time",
            "last_contact_time",
            fallback=contact_time_fallback,
        )

        ## IMU
        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        ## Command
        command = self._command

        ############
        ## States ##
        ############
        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        joint_pos_robot_normalized = normalize_joint_pos(
            joint_pos_robot, self._robot.data.soft_joint_pos_limits
        )

        contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

        #####################
        ## Reward  metrics ##
        #####################
        _action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)
        _joint_torque_sq = torch.sum(torch.square(joint_applied_torque_robot), dim=1)
        _joint_acc_sq = torch.sum(torch.square(joint_acc_robot), dim=1)
        _undesired_contact_force = torch.max(
            torch.norm(
                contact_forces_robot[:, self._undesired_contact_sensor_indices, :],
                dim=-1,
            ),
            dim=1,
        )[0]
        _cmd_lin_vel_metric = torch.exp(
            -torch.sum(torch.square(command[:, :2] - vel_lin_robot[:, :2]), dim=1)
            / rew.exp_std_cmd_lin_vel_xy
        )
        _cmd_ang_vel_metric = torch.exp(
            -torch.square(command[:, 2] - vel_ang_robot[:, 2])
            / rew.exp_std_cmd_ang_vel_z
        )
        _feet_air_time_metric = compute_stance_air_time_metric(
            current_air_time=contact_current_air_time[:, self._feet_sensor_indices],
            current_contact_time=contact_current_contact_time[
                :, self._feet_sensor_indices
            ],
            command_xy=command[:, :2],
            body_xy_vel=vel_lin_robot[:, :2],
            mode_time=rew.mode_time_feet_air_time,
            velocity_threshold=rew.threshold_feet_air_time,
        )
        _quadruped_trot_metric = compute_quadruped_trot_metric(
            current_air_time=contact_current_air_time[:, self._feet_sensor_indices],
            current_contact_time=contact_current_contact_time[
                :, self._feet_sensor_indices
            ],
            synced_feet_pair_ids=self._quadruped_trot_pair_ids,
            command_xy=command[:, :2],
            body_xy_vel=vel_lin_robot[:, :2],
            velocity_threshold=rew.threshold_feet_air_time,
            std=rew.std_quadruped_trot,
            max_err=rew.max_err_quadruped_trot,
        )
        _foot_slip_metric = compute_foot_slip_metric(
            contact_forces_w=contact_forces_robot[:, self._feet_sensor_indices, :],
            foot_lin_vel_w=body_vel_w_robot[:, self._feet_indices, :3],
            contact_threshold=rew.threshold_foot_contact_force,
        )
        _undesired_lin_vel_z = torch.square(vel_lin_robot[:, 2])
        _undesired_ang_vel_xy = torch.sum(torch.square(vel_ang_robot[:, :2]), dim=-1)
        _gravity_alignment = torch.sum(
            torch.square(projected_gravity_robot[:, :2]), dim=1
        ) + torch.square(projected_gravity_robot[:, 2] + 1.0)

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {
            # Penalty: Action rate
            "penalty_action_rate": rew.penalty_action_rate(_action_rate),
            # Penalty: Joint torque
            "penalty_joint_torque": rew.penalty_joint_torque(_joint_torque_sq),
            # Penalty: Joint acceleration
            "penalty_joint_acceleration": rew.penalty_joint_acceleration(_joint_acc_sq),
            # Penalty: Undesired robot contacts
            "penalty_undesired_robot_contacts": rew.penalty_undesired_robot_contacts(
                _undesired_contact_force
            ),
            # Reward: Command tracking (linear)
            "reward_cmd_lin_vel_xy": rew.reward_cmd_lin_vel_xy(_cmd_lin_vel_metric),
            # Reward: Command tracking (angular)
            "reward_cmd_ang_vel_z": rew.reward_cmd_ang_vel_z(_cmd_ang_vel_metric),
            # Reward: Feet air time
            "reward_feet_air_time": rew.reward_feet_air_time(_feet_air_time_metric),
            "reward_quadruped_trot": rew.reward_quadruped_trot(_quadruped_trot_metric),
            "penalty_foot_slip": rew.penalty_foot_slip(_foot_slip_metric),
            # Penalty: Minimize non-command motion (linear)
            "penalty_undesired_lin_vel_z": rew.penalty_undesired_lin_vel_z(
                _undesired_lin_vel_z
            ),
            # Penalty: Minimize non-command motion (angular)
            "penalty_undesired_ang_vel_xy": rew.penalty_undesired_ang_vel_xy(
                _undesired_ang_vel_xy
            ),
            # Penalty: Minimize rotation with the gravity direction
            "penalty_gravity_rotation_alignment": rew.penalty_gravity_rotation_alignment(
                _gravity_alignment
            ),
        }

        ##################
        ## Terminations ##
        ##################
        termination = compute_locomotion_safety_termination(
            undesired_contact_force=_undesired_contact_force,
            projected_gravity_b=projected_gravity_robot,
            contact_force_threshold=rew.penalty_undesired_robot_contacts.threshold,
            max_projected_gravity_xy=rew.max_projected_gravity_xy,
            terminate_on_undesired_contacts=rew.terminate_on_undesired_contacts,
            terminate_on_fall=rew.terminate_on_fall,
        )
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                    "projected_gravity_robot": projected_gravity_robot,
                },
                ObservationGroup.STATE_DYN: {
                    "contact_forces_robot": contact_forces_robot,
                },
                ObservationGroup.PROPRIO: {
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                },
                ObservationGroup.PROPRIO_DYN: {
                    "joint_pos_robot_normalized": joint_pos_robot_normalized,
                    "joint_acc_robot": joint_acc_robot,
                    "joint_applied_torque_robot": joint_applied_torque_robot,
                },
                ObservationGroup.COMMAND: {
                    "cmd_vel": command,
                },
            },
            reward,
            termination,
            truncation,
        )


def _to_torch(value: torch.Tensor) -> torch.Tensor:
    return value.torch if hasattr(value, "torch") else value


def _get_contact_time_series(
    contact_data: object,
    primary: str,
    fallback_attr: str,
    fallback: torch.Tensor | None = None,
) -> torch.Tensor:
    if hasattr(contact_data, primary):
        return _to_torch(getattr(contact_data, primary))  # type: ignore[arg-type]
    if hasattr(contact_data, fallback_attr):
        return _to_torch(getattr(contact_data, fallback_attr))  # type: ignore[arg-type]
    if fallback is not None:
        return fallback

    raise AttributeError(
        f"Contact sensor data is missing both '{primary}' and '{fallback_attr}'."
    )


class LocomotionTask(_LocomotionTaskMixin, Task):
    """Legged velocity tracking on the owned SRB lifecycle."""

    cfg: LocomotionTaskCfg
    # The registered entry point declares its runtime explicitly.
    _srb_runtime_mode = "isaac"

    def __init__(self, cfg: LocomotionTaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._contacts_robot: ContactSensor = get_scene_sensor(
            self.scene, "contacts_robot"
        )

        # Sensor and articulation bodies use separate index orders.
        feet = resolve_feet_contact_indices(
            self._robot, self._contacts_robot, self.cfg._robot.regex_feet_links
        )
        self._feet_indices = list(feet.feet_body)
        self._feet_sensor_indices = list(feet.feet_sensor)
        self._undesired_contact_sensor_indices = list(feet.undesired_sensor)
        self._quadruped_trot_pair_ids = find_quadruped_trot_pairs(feet.foot_names)
