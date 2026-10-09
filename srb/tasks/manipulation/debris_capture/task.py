from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    AssetVariant,
    ExtravehicularScenery,
    Object,
    OrbitalRobot,
    Pedestal,
    RigidObject,
    RigidObjectCfg,
)
from srb.core.domain import Domain
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationRewardCfg,
    ManipulationSceneCfg,
)
from srb.core.env.manipulation._step_state import (
    manipulator_observation_tail,
    manipulator_step_state,
)
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    GaussianRewardTerm,
    HyperbolicRewardTerm,
    LinearRewardTerm,
    SoftThresholdRewardTerm,
)
from srb.core.scene_introspection import get_scene_rigid_object
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    radians,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)

from .asset import select_debris


def _point_velocity_w(
    body_vel_w: torch.Tensor,
    body_pos_w: torch.Tensor,
    point_pos_w: torch.Tensor,
    body_index: int,
) -> torch.Tensor:
    """Return the world-frame velocity at a point fixed to a rigid body."""
    body_twist_w = body_vel_w[:, body_index, :]
    body_origin_w = body_pos_w[:, body_index, :]
    return body_twist_w[:, :3] + torch.cross(
        body_twist_w[:, 3:6], point_pos_w - body_origin_w, dim=-1
    )


##############
### Config ###
##############


class RewardCfg(ManipulationRewardCfg):
    """Approach, velocity, speed, and grasp rewards plus shared penalties."""

    reward_distance_ee_to_obj: GaussianRewardTerm = GaussianRewardTerm(
        weight=16.0, scale=0.08
    )
    """Rewards end-effector proximity to the debris (Gaussian, 0.08 m scale)."""

    reward_grasp: SoftThresholdRewardTerm = SoftThresholdRewardTerm(
        weight=16.0, threshold=5.0, operator=">", sharpness=1.0
    )
    """Soft-threshold grasp reward around 5 N contact force."""

    penalty_debris_velocity_lin: LinearRewardTerm = LinearRewardTerm(
        weight=-3.0, limit=20.0
    )
    """Penalizes debris linear speed."""

    penalty_debris_velocity_ang: LinearRewardTerm = LinearRewardTerm(
        weight=-1.0, limit=10.0
    )
    """Penalizes debris angular speed."""

    reward_approach_velocity_alignment: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=4.0, scale=0.1
    )
    """Rewards end-effector velocity aligned toward the debris."""

    penalty_approach_speed: LinearRewardTerm = LinearRewardTerm(weight=-2.0, limit=4.0)
    """Penalizes approach speed above 0.3 m/s within 0.5 m."""

    ## Shaping parameters
    threshold_approach_speed: float = 0.3
    proximity_approach_speed: float = 0.5

    penalty_debris_too_far: LinearRewardTerm = LinearRewardTerm(weight=-8.0, limit=8.0)
    """Penalizes debris drifting beyond the configured distance."""


@configclass
class SceneCfg(ManipulationSceneCfg):
    debris: RigidObjectCfg = MISSING  # type: ignore[assignment]


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_obj_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("debris"),
            "pose_range": {
                "x": (-0.25, 0.25),
                "y": (-0.25, 0.25),
                "z": (-0.25, 0.25),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.2 - 0.05, -0.2 + 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.05),
                "roll": (-radians(10.0), radians(10.0)),
                "pitch": (-radians(10.0), radians(10.0)),
                "yaw": (-radians(10.0), radians(10.0)),
            },
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    """Configuration for the finite-horizon orbital debris capture task."""

    ## Scenario
    domain: Domain = Domain.ORBIT

    ## Assets
    scenery: ExtravehicularScenery | AssetVariant | None = None
    pedestal: Pedestal | OrbitalRobot | None = assets.VenusExpress()
    debris: Object | AssetVariant = AssetVariant.DATASET

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 10.0
    is_finite_horizon: bool = True

    ## Penalty thresholds
    distance_debris_too_far: float = 10.0
    """Distance threshold for the too-far debris penalty; does not terminate the episode."""

    def __post_init__(self):
        super().__post_init__()

        # Scene: Debris
        self.scene.debris = select_debris(
            self,
            prim_path="{ENV_REGEX_NS}/debris",
            init_state=RigidObjectCfg.InitialStateCfg(pos=(1.5, 0.0, 0.5)),
            activate_contact_sensors=True,
        )

        # Sensor: End-effector contacts
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                self.scene.debris.prim_path
            ]

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()


############
### Task ###
############


class _TaskMixin:
    """Orbital debris capture with a fixed-base manipulator.

    Observations cover end-effector pose and velocity, joints, debris-relative
    pose, and contacts. Actions are joint position targets; rewards shape
    approach, alignment, and grasping.
    """

    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._obj: RigidObject = get_scene_rigid_object(self.scene, "debris")
        ee_frame_path = self.cfg.scene.tf_end_effector.target_frames[0].prim_path
        ee_body_name = str(ee_frame_path).rstrip("/").rsplit("/", 1)[-1]
        if ee_body_name not in self._robot.body_names:
            raise ValueError(
                f"end-effector frame body {ee_body_name!r} is not an articulation "
                f"body of the robot ({list(self._robot.body_names)})"
            )
        self._ee_body_index = self._robot.body_names.index(ee_body_name)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward

        ################
        ## Raw states ##
        ################
        state = manipulator_step_state(self)
        num_envs, device, dtype = state.num_envs, state.device, state.dtype

        tf_pos_obj = self._obj.data.root_pos_w
        tf_quat_obj = self._obj.data.root_quat_w

        tf_pos_ee_to_obj, tf_quat_ee_to_obj = subtract_frame_transforms(
            t01=state.tf_pos_ee, q01=state.tf_quat_ee, t02=tf_pos_obj, q02=tf_quat_obj
        )
        tf_rotmat_ee_to_obj = matrix_from_quat(tf_quat_ee_to_obj)
        tf_rot6d_ee_to_obj = rotmat_to_rot6d(tf_rotmat_ee_to_obj)

        vel_lin_obj = self._obj.data.root_lin_vel_w
        vel_ang_obj = self._obj.data.root_ang_vel_w

        contact_forces_robot = self._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
        contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

        has_ee_contacts = isinstance(self._contacts_end_effector, ContactSensor)
        if has_ee_contacts:
            contact_forces_ee = self._contacts_end_effector.data.net_forces_w
            contact_forces_mean_ee = contact_forces_ee.mean(dim=1)
            contact_force_matrix_ee = self._contacts_end_effector.data.force_matrix_w
        else:
            contact_forces_ee = torch.empty((num_envs, 0), dtype=dtype, device=device)
            contact_forces_mean_ee = torch.empty(
                (num_envs, 0), dtype=dtype, device=device
            )
            contact_force_matrix_ee = None

        #####################
        ## Reward  metrics ##
        #####################
        action_rate_metric = torch.mean(
            torch.square(state.act_current - state.act_previous), dim=1
        )
        joint_torque_metric = torch.sum(
            torch.square(state.joint_applied_torque_robot), dim=1
        )
        joint_acc_metric = torch.sum(torch.square(state.joint_acc_robot), dim=1)
        undesired_contacts_metric = torch.max(
            torch.norm(contact_forces_robot, dim=-1), dim=1
        )[0]
        distance_ee_to_obj = torch.norm(tf_pos_ee_to_obj, dim=-1)

        # The configured EE point may be offset; pair link-origin velocity and
        # position so angular motion contributes to its world velocity.
        _ee_vel = _point_velocity_w(
            self._robot.data.body_link_vel_w,
            self._robot.data.body_pos_w,
            state.tf_pos_ee,
            self._ee_body_index,
        )
        _dir_ee_to_obj_world = (tf_pos_obj - state.tf_pos_ee) / (
            distance_ee_to_obj.unsqueeze(-1) + 1e-8
        )
        _ee_speed = torch.norm(_ee_vel, dim=-1)
        _cosine_similarity = torch.sum(_dir_ee_to_obj_world * _ee_vel, dim=-1) / (
            _ee_speed + 1e-8
        )
        approach_vel_alignment_error = 1.0 - torch.clamp(
            _cosine_similarity, min=0.0, max=1.0
        )
        approach_speed_metric = (_ee_speed - rew.threshold_approach_speed).clamp(
            min=0.0
        ) * (distance_ee_to_obj < rew.proximity_approach_speed).float()

        grasp_metric: torch.Tensor
        if contact_force_matrix_ee is not None:
            grasp_metric = torch.mean(
                torch.max(torch.norm(contact_force_matrix_ee, dim=-1), dim=-1)[0],
                dim=1,
            )
        else:
            grasp_metric = torch.zeros(num_envs, dtype=dtype, device=device)

        debris_vel_lin_metric = torch.norm(vel_lin_obj, dim=-1)
        debris_vel_ang_metric = torch.norm(vel_ang_obj, dim=-1)

        #############
        ## Rewards ##
        #############
        debris_too_far_metric = (
            distance_ee_to_obj > self.cfg.distance_debris_too_far
        ).float()

        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "penalty_joint_torque": rew.penalty_joint_torque(joint_torque_metric),
            "penalty_joint_acceleration": rew.penalty_joint_acceleration(
                joint_acc_metric
            ),
            "penalty_undesired_robot_contacts": rew.penalty_undesired_robot_contacts(
                undesired_contacts_metric
            ),
            "reward_distance_ee_to_obj": rew.reward_distance_ee_to_obj(
                distance_ee_to_obj
            ),
            "reward_grasp": rew.reward_grasp(grasp_metric),
            "penalty_debris_velocity_lin": rew.penalty_debris_velocity_lin(
                debris_vel_lin_metric
            ),
            "penalty_debris_velocity_ang": rew.penalty_debris_velocity_ang(
                debris_vel_ang_metric
            ),
            "reward_approach_velocity_alignment": rew.reward_approach_velocity_alignment(
                approach_vel_alignment_error
            ),
            "penalty_approach_speed": rew.penalty_approach_speed(approach_speed_metric),
            "penalty_debris_too_far": rew.penalty_debris_too_far(debris_too_far_metric),
        }

        ##################
        ## Terminations ##
        ##################
        # Drifting debris is penalized; episodes end by truncation.
        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "contact_forces_mean_end_effector": contact_forces_mean_ee,
                    "tf_pos_end_effector_to_obj": tf_pos_ee_to_obj,
                    "tf_rot6d_end_effector_to_obj": tf_rot6d_ee_to_obj,
                    "vel_lin_obj": vel_lin_obj,
                    "vel_ang_obj": vel_ang_obj,
                },
                **manipulator_observation_tail(
                    state, contact_forces_robot, contact_forces_ee
                ),
            },
            reward,
            termination,
            truncation,
        )


class Task(_TaskMixin, SrbEnv, ManipulationEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg, runtime_mode=runtime_mode, **kwargs)
