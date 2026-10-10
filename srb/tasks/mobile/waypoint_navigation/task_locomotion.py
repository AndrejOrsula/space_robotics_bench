from dataclasses import MISSING

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import AssetVariant, Humanoid, LeggedRobot
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import (
    reset_joints_by_scale,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import LinearRewardTerm, ThresholdRewardTerm
from srb.core.reward.locomotion import (
    compute_locomotion_safety_termination,
    resolve_feet_contact_indices,
)
from srb.core.scene_introspection import get_scene_sensor
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    normalize_joint_pos,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)

from .task import EventCfg, SceneCfg, Task, TaskCfg
from .task import RewardCfg as WaypointNavRewardCfg

##############
### Config ###
##############


class LocomotionRewardCfg(WaypointNavRewardCfg):
    """Waypoint rewards plus legged-robot penalties and feet air-time reward."""

    ## Locomotion-specific penalties
    penalty_joint_torque: LinearRewardTerm = LinearRewardTerm(
        weight=-0.000025, limit=4.0
    )
    """Small torque penalty for legged robots."""

    penalty_joint_acceleration: LinearRewardTerm = LinearRewardTerm(
        weight=-0.00000025, limit=2.0
    )
    """Reduces joint acceleration to smooth gait."""

    penalty_undesired_robot_contacts: ThresholdRewardTerm = ThresholdRewardTerm(
        weight=-2.0, threshold=1.0, operator=">"
    )
    """Penalizes contact on non-foot bodies."""

    penalty_undesired_lin_vel_z: LinearRewardTerm = LinearRewardTerm(
        weight=-0.5, limit=4.0
    )
    """Penalizes vertical motion for smoother locomotion."""

    penalty_undesired_ang_vel_xy: LinearRewardTerm = LinearRewardTerm(
        weight=-0.1, limit=4.0
    )
    """Penalizes roll and pitch motion for stability."""

    penalty_gravity_rotation_alignment: LinearRewardTerm = LinearRewardTerm(
        weight=-2.0, limit=8.0
    )
    """Penalizes tilt measured by projected gravity."""

    ## Locomotion-specific rewards
    reward_feet_air_time: LinearRewardTerm = LinearRewardTerm(weight=0.5, limit=4.0)
    """Rewards foot swing time to encourage gait rhythm."""

    ## Shaping parameters
    threshold_feet_air_time: float = 0.1
    """Minimum 2D distance to target to activate feet air time reward."""

    terminate_on_undesired_contacts: bool = True
    """Terminate when a non-foot body exceeds the contact-force threshold."""

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
    # External disturbance randomization is intentionally absent; see
    # `docs/src/misc/modelling_envelope.md` for the modeling rationale.


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
        ## Visualize target
        self._target_marker.visualize(self._goal[:, :3], self._goal[:, 3:])

        rew = self.cfg.reward
        num_envs = self.num_envs
        device = self.device

        ## Actions
        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        ## Root state
        _robot_pose = self._robot.data.root_link_pose_w
        tf_pos_robot = _robot_pose[:, 0:3]
        tf_quat_robot = _robot_pose[:, 3:7]
        vel_lin_robot = self._robot.data.root_lin_vel_b
        vel_ang_robot = self._robot.data.root_ang_vel_b
        projected_gravity_robot = _to_torch(self._robot.data.projected_gravity_b)

        ## Target
        tf_pos_target = self._goal[:, 0:3]
        tf_quat_target = self._goal[:, 3:7]

        ## Joints
        joint_pos_robot = self._robot.data.joint_pos
        joint_acc_robot = self._robot.data.joint_acc
        joint_applied_torque_robot = self._robot.data.applied_torque

        ## Contacts
        contact_forces_robot = self._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
        contact_robot = self._contacts_robot.compute_first_contact(self.step_dt)
        contact_last_air_time = self._contacts_robot.data.last_air_time  # type: ignore[union-attr]

        ## IMU
        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        ############
        ## States ##
        ############
        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        # Robot -> Target
        tf_pos_robot_to_target, _tf_quat_robot_to_target = subtract_frame_transforms(
            t01=tf_pos_robot, q01=tf_quat_robot, t02=tf_pos_target, q02=tf_quat_target
        )
        tf_rotmat_robot_to_target = matrix_from_quat(_tf_quat_robot_to_target)

        # Derived states
        tf_pos2d_robot_to_target = tf_pos_robot_to_target[:, :2]
        dist2d_robot_to_target = torch.norm(tf_pos2d_robot_to_target, dim=-1)
        angle_robot_to_target_pos = torch.atan2(
            tf_pos_robot_to_target[..., 1], tf_pos_robot_to_target[..., 0]
        )
        yaw_robot_to_target = torch.atan2(
            tf_rotmat_robot_to_target[..., 1, 0], tf_rotmat_robot_to_target[..., 0, 0]
        )

        # Observation noise inherited from the waypoint task: episode bias plus
        # per-step draws on the relative position and yaw; rewards stay noise-free.
        tf_pos2d_robot_to_target_with_noise = (
            tf_pos2d_robot_to_target
            + self._episodic_noise_tf_pos2d
            + torch.normal(
                mean=0.0,
                std=self.cfg.observation_step_noise_pos_std_m,
                size=(num_envs, 2),
                device=device,
            )
        )
        yaw_noise = self._episodic_noise_tf_yaw + torch.normal(
            mean=0.0,
            std=self.cfg.observation_step_noise_yaw_std_rad,
            size=(num_envs,),
            device=device,
        )
        sin_noise, cos_noise = torch.sin(yaw_noise), torch.cos(yaw_noise)
        zeros, ones = torch.zeros_like(yaw_noise), torch.ones_like(yaw_noise)
        rotmat_yaw_noise = torch.stack(
            (
                torch.stack((cos_noise, -sin_noise, zeros), dim=-1),
                torch.stack((sin_noise, cos_noise, zeros), dim=-1),
                torch.stack((zeros, zeros, ones), dim=-1),
            ),
            dim=-2,
        )
        tf_rot6d_robot_to_target_with_noise = rotmat_to_rot6d(
            rotmat_yaw_noise @ tf_rotmat_robot_to_target
        )

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
        position_tracking_metric = torch.square(dist2d_robot_to_target)
        _point_towards = 1.0 - torch.tanh(
            torch.abs(angle_robot_to_target_pos) / rew.tanh_std_point_towards_target
        )
        _position_tracking_precision = 1.0 - torch.tanh(
            dist2d_robot_to_target / rew.tanh_std_position_tracking_precision
        )
        _orientation_tracking_precision = _position_tracking_precision * (
            1.0
            - torch.tanh(
                torch.abs(yaw_robot_to_target) / rew.tanh_std_orientation_tracking
            )
        )
        action_rate_at_target_metric = _orientation_tracking_precision * (
            1.0 - torch.tanh(_action_rate / rew.tanh_std_action_rate_at_target)
        )
        _feet_air_time_metric = (
            dist2d_robot_to_target > rew.threshold_feet_air_time
        ) * torch.sum(
            (contact_last_air_time[:, self._feet_sensor_indices] - 0.5).clamp(min=0.0)
            * contact_robot[:, self._feet_sensor_indices],
            dim=1,
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
            # Penalty: Position tracking | Robot <--> Target
            "penalty_position_tracking": rew.penalty_position_tracking(
                position_tracking_metric
            ),
            # Reward: Point towards target | Robot <--> Target
            "reward_point_towards_target": rew.reward_point_towards_target(
                _point_towards
            ),
            # Reward: Position tracking | Robot <--> Target (precision)
            "reward_position_tracking_precision": rew.reward_position_tracking_precision(
                _position_tracking_precision
            ),
            # Reward: Target orientation tracking once position is reached | Robot <--> Target
            "reward_orientation_tracking": rew.reward_orientation_tracking(
                _orientation_tracking_precision
            ),
            # Reward: Action rate at target
            "reward_action_rate_at_target": rew.reward_action_rate_at_target(
                action_rate_at_target_metric
            ),
            # Reward: Feet air time
            "reward_feet_air_time": rew.reward_feet_air_time(_feet_air_time_metric),
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
                    "tf_pos2d_robot_to_target": tf_pos2d_robot_to_target_with_noise,
                    "tf_rot6d_robot_to_target": tf_rot6d_robot_to_target_with_noise,
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
            },
            reward,
            termination,
            truncation,
        )


class LocomotionTask(_LocomotionTaskMixin, Task):
    cfg: LocomotionTaskCfg

    def __init__(self, cfg: LocomotionTaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        ## Get scene assets
        self._contacts_robot: ContactSensor = get_scene_sensor(
            self.scene, "contacts_robot"
        )

        ## Cache metrics
        # Articulation and contact-sensor body orders differ; contact arrays
        # are indexed by the sensor's own lookup.
        feet = resolve_feet_contact_indices(
            self._robot, self._contacts_robot, self.cfg._robot.regex_feet_links
        )
        self._feet_sensor_indices = list(feet.feet_sensor)
        self._undesired_contact_sensor_indices = list(feet.undesired_sensor)


def _to_torch(value: torch.Tensor) -> torch.Tensor:
    return value.torch if hasattr(value, "torch") else value
