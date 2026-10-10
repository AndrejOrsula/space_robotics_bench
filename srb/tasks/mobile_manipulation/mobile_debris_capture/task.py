from dataclasses import MISSING

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    Articulation,
    AssetVariant,
    Object,
    OrbitalManipulator,
    RigidObject,
    RigidObjectCfg,
)
from srb.core.env import (
    OrbitalManipulationEnv,
    OrbitalManipulationEnvCfg,
    OrbitalManipulationEventCfg,
    OrbitalManipulationSceneCfg,
    ViewerCfg,
)
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    BaseRewardCfg,
    GaussianRewardTerm,
    LinearRewardTerm,
    SoftThresholdRewardTerm,
)
from srb.core.scene_introspection import get_scene_rigid_object
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import SimforgeAssetCfg
from srb.tasks.manipulation.debris_capture.asset import select_debris
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    normalize_joint_pos,
    radians,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)


class RewardCfg(BaseRewardCfg):
    """Combine orbital mobility and manipulation reward terms.

    Shape approach and velocity matching before rewarding grasp contact.
    Angular velocity matching is weighted twice as strongly as linear matching.
    """

    penalty_action_rate: LinearRewardTerm = LinearRewardTerm(weight=-0.1, limit=4.0)
    """Discourages rapid changes in commanded actions."""

    penalty_fuel_consumption: LinearRewardTerm = LinearRewardTerm(
        weight=-2.0, limit=8.0
    )
    """Penalises fuel usage — moderate weight; manoeuvring is expected."""

    penalty_angular_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=-0.05, limit=5.0
    )
    """Penalises base rotation to support stable manipulation."""

    penalty_joint_torque: LinearRewardTerm = LinearRewardTerm(
        weight=-0.000025, limit=4.0
    )
    """Penalises joint torque, with a small weight for orbital motion."""

    penalty_joint_acceleration: LinearRewardTerm = LinearRewardTerm(
        weight=-0.0005, limit=4.0
    )
    """Penalises large joint accelerations."""

    reward_distance_end_effector_to_obj: GaussianRewardTerm = GaussianRewardTerm(
        weight=16.0, scale=0.2
    )
    """Gaussian proximity reward with a 0.2 m scale."""

    reward_grasp: SoftThresholdRewardTerm = SoftThresholdRewardTerm(
        weight=32.0, threshold=5.0, operator=">", sharpness=1.0
    )
    """Soft reward for grasp contact near or above 5 N."""

    reward_minimize_rel_lin_vel: GaussianRewardTerm = GaussianRewardTerm(
        weight=12.0, scale=0.01
    )
    """Gaussian reward for low relative linear velocity (0.01 m/s scale)."""

    reward_minimize_rel_ang_vel: GaussianRewardTerm = GaussianRewardTerm(
        weight=24.0, scale=0.03
    )
    """Gaussian reward for low relative angular velocity (0.03 rad/s scale)."""

    penalty_debris_too_far: LinearRewardTerm = LinearRewardTerm(
        weight=-16.0, limit=16.0
    )
    """Penalises debris beyond end-effector range without terminating."""


@configclass
class SceneCfg(OrbitalManipulationSceneCfg):
    env_spacing: float = 8.0
    debris: RigidObjectCfg = MISSING  # type: ignore[assignment]


@configclass
class EventCfg(OrbitalManipulationEventCfg):
    randomize_obj_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("debris"),
            "pose_range": {
                "x": (2.0, 4.0),
                "y": (-1.0, 1.0),
                "z": (-1.0, 1.0),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-2.0, -0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-radians(10.0), radians(10.0)),
                "pitch": (-radians(10.0), radians(10.0)),
                "yaw": (-radians(10.0), radians(10.0)),
            },
        },
    )


@configclass
class TaskCfg(OrbitalManipulationEnvCfg):
    """Configure free-flight debris capture within 60-second episodes."""

    ## Assets
    robot: OrbitalManipulator | AssetVariant = assets.GenericOrbitalManipulator(
        mobile_base=assets.VenusExpress(), manipulator=assets.Franka()
    )
    debris: Object | AssetVariant = AssetVariant.PROCEDURAL

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 60.0
    is_finite_horizon: bool = True

    ## Penalty thresholds
    distance_debris_too_far: float = 25.0
    """Distance threshold for the too-far debris penalty; does not terminate the episode."""

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(8.0, -8.0, 4.0), lookat=(2.0, -2.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()

        self.scene.debris = select_debris(
            self,  # type: ignore[misc]
            prim_path="{ENV_REGEX_NS}/debris",
            init_state=RigidObjectCfg.InitialStateCfg(pos=(5.0, 0.0, 0.0)),
            activate_contact_sensors=True,
        )

        if isinstance(self.scene.debris.spawn, SimforgeAssetCfg):
            self.scene.debris.spawn.seed = self.seed + self.scene.num_envs

        # Reapply the debris filter after scene setup rebuilds this sensor;
        # otherwise force_matrix_w is unset and the grasp reward stays zero.
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                self.scene.debris.prim_path
            ]

        self._update_procedural_assets()


class Task(SrbEnv, OrbitalManipulationEnv):
    """Capture drifting debris with a free-flying robot and manipulator arm."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._obj: RigidObject = get_scene_rigid_object(self.scene, "debris")
        # ``ManipulationEnv`` already binds the optional ``_contacts_end_effector``.

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward
        num_envs = self.num_envs

        thrust_action_term = self._thrust_action_term
        if thrust_action_term is not None:
            remaining_fuel = (
                thrust_action_term.remaining_fuel / thrust_action_term.cfg.fuel_capacity
            ).unsqueeze(-1)
        else:
            remaining_fuel = torch.ones(
                (num_envs, 1), dtype=torch.float32, device=self.device
            )

        tf_pos_robot = self._robot.data.root_pos_w
        tf_quat_robot = self._robot.data.root_quat_w
        vel_lin_robot = self._robot.data.root_lin_vel_w
        vel_ang_robot = self._robot.data.root_ang_vel_w

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        tf_pos_obj = self._obj.data.root_pos_w
        tf_quat_obj = self._obj.data.root_quat_w
        vel_obj = self._obj.data.root_vel_w
        tf_pos_robot_to_obj = tf_pos_obj - tf_pos_robot
        vel_lin_robot_to_obj = vel_obj[:, :3] - vel_lin_robot
        vel_ang_robot_to_obj = vel_obj[:, 3:] - vel_ang_robot

        joint_pos_robot = self._manipulator.data.joint_pos
        joint_pos_robot_normalized = normalize_joint_pos(
            joint_pos_robot, self._manipulator.data.soft_joint_pos_limits
        )

        joint_pos_end_effector = (
            self._end_effector.data.joint_pos
            if isinstance(self._end_effector, Articulation)
            else None
        )
        joint_pos_end_effector_normalized = (
            normalize_joint_pos(
                joint_pos_end_effector,
                self._end_effector.data.soft_joint_pos_limits,
            )
            if joint_pos_end_effector is not None
            and isinstance(self._end_effector, Articulation)
            else (
                joint_pos_end_effector
                if joint_pos_end_effector is not None
                else torch.empty((num_envs, 0), dtype=torch.float32, device=self.device)
            )
        )

        joint_acc_robot = self._manipulator.data.joint_acc
        joint_applied_torque_robot = self._manipulator.data.applied_torque

        fk_pos_end_effector = self._tf_end_effector.data.target_pos_source[:, 0, :]
        fk_quat_end_effector = self._tf_end_effector.data.target_quat_source[:, 0, :]
        fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
        fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

        tf_pos_end_effector = self._tf_end_effector.data.target_pos_w[:, 0, :]
        tf_quat_end_effector = self._tf_end_effector.data.target_quat_w[:, 0, :]

        tf_pos_ee_to_obj, tf_quat_ee_to_obj = subtract_frame_transforms(
            t01=tf_pos_end_effector,
            q01=tf_quat_end_effector,
            t02=tf_pos_obj,
            q02=tf_quat_obj,
        )
        tf_rotmat_ee_to_obj = matrix_from_quat(tf_quat_ee_to_obj)
        tf_rot6d_ee_to_obj = rotmat_to_rot6d(tf_rotmat_ee_to_obj)

        contact_forces_robot = self._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
        contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

        contact_forces_end_effector = (
            self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None
        )
        contact_forces_mean_end_effector = (
            contact_forces_end_effector.mean(dim=1)
            if contact_forces_end_effector is not None
            else torch.empty((num_envs, 0), dtype=torch.float32, device=self.device)
        )
        contact_forces_end_effector_obs = (
            contact_forces_end_effector
            if contact_forces_end_effector is not None
            else torch.empty((num_envs, 0), dtype=torch.float32, device=self.device)
        )

        contact_force_matrix_end_effector = (
            self._contacts_end_effector.data.force_matrix_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None
        )

        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        reward: dict[str, torch.Tensor] = {}

        _action_rate = torch.mean(
            torch.square(self.action_manager.action - self.action_manager.prev_action),
            dim=1,
        )
        reward["penalty_action_rate"] = rew.penalty_action_rate(_action_rate)

        _fuel_used = torch.square(1.0 - remaining_fuel.squeeze(-1))
        reward["penalty_fuel_consumption"] = rew.penalty_fuel_consumption(_fuel_used)

        _ang_vel_sq = torch.sum(torch.square(vel_ang_robot), dim=1)
        reward["penalty_angular_velocity"] = rew.penalty_angular_velocity(_ang_vel_sq)

        _torque_sq = torch.sum(torch.square(joint_applied_torque_robot), dim=1)
        reward["penalty_joint_torque"] = rew.penalty_joint_torque(_torque_sq)

        _acc_sq = torch.sum(torch.square(joint_acc_robot), dim=1)
        reward["penalty_joint_acceleration"] = rew.penalty_joint_acceleration(_acc_sq)

        _dist_ee_to_obj = torch.norm(tf_pos_ee_to_obj, dim=-1)
        reward["reward_distance_end_effector_to_obj"] = (
            rew.reward_distance_end_effector_to_obj(_dist_ee_to_obj)
        )

        if contact_force_matrix_end_effector is not None:
            _grasp_metric = torch.mean(
                torch.max(
                    torch.norm(contact_force_matrix_end_effector, dim=-1), dim=-1
                )[0],
                dim=1,
            )
            reward["reward_grasp"] = rew.reward_grasp(_grasp_metric)
        else:
            reward["reward_grasp"] = torch.zeros(
                num_envs, dtype=torch.float32, device=self.device
            )

        _rel_lin_vel = torch.norm(vel_lin_robot_to_obj, dim=-1)
        reward["reward_minimize_rel_lin_vel"] = rew.reward_minimize_rel_lin_vel(
            _rel_lin_vel
        )

        _rel_ang_vel = torch.norm(vel_ang_robot_to_obj, dim=-1)
        reward["reward_minimize_rel_ang_vel"] = rew.reward_minimize_rel_ang_vel(
            _rel_ang_vel
        )

        debris_too_far_metric = (
            _dist_ee_to_obj > self.cfg.distance_debris_too_far
        ).float()
        reward["penalty_debris_too_far"] = rew.penalty_debris_too_far(
            debris_too_far_metric
        )

        # Distance penalty replaces early termination.
        termination = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                    "tf_pos_robot_to_obj": tf_pos_robot_to_obj,
                    "tf_pos_end_effector_to_obj": tf_pos_ee_to_obj,
                    "tf_rot6d_end_effector_to_obj": tf_rot6d_ee_to_obj,
                    "vel_lin_obj": vel_obj[:, :3],
                    "vel_ang_obj": vel_obj[:, 3:],
                    "vel_lin_robot_to_obj": vel_lin_robot_to_obj,
                    "vel_ang_robot_to_obj": vel_ang_robot_to_obj,
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
                },
                ObservationGroup.STATE_DYN: {
                    "contact_forces_robot": contact_forces_robot,
                    "contact_forces_end_effector": contact_forces_end_effector_obs,
                },
                ObservationGroup.PROPRIO: {
                    "fk_pos_end_effector": fk_pos_end_effector,
                    "fk_rot6d_end_effector": fk_rot6d_end_effector,
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                    "remaining_fuel": remaining_fuel,
                },
                ObservationGroup.PROPRIO_DYN: {
                    "joint_pos_robot_normalized": joint_pos_robot_normalized,
                    "joint_pos_end_effector_normalized": joint_pos_end_effector_normalized,
                    "joint_acc_robot": joint_acc_robot,
                    "joint_applied_torque_robot": joint_applied_torque_robot,
                },
            },
            reward,
            termination,
            truncation,
        )
