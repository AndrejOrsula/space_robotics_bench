from dataclasses import MISSING
from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    AssetVariant,
    Object,
    RigidObject,
    RigidObjectCfg,
    Terrain,
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
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    GaussianRewardTerm,
    GaussianTrackingRewardTerm,
    GaussianZoneTrackingRewardTerm,
    LinearRewardTerm,
    SoftThresholdRewardTerm,
)
from srb.core.scene_introspection import get_scene_rigid_object
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import PreviewSurfaceCfg, SphereCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)

from .asset import select_sample
from .event_camera_wiring import build_wrist_event_camera_cfg

##############
### Config ###
##############


class RewardCfg(ManipulationRewardCfg):
    """Approach, grasp, lift, and deposit rewards with a recoverable fall penalty."""

    reward_top_down_orientation: GaussianTrackingRewardTerm = (
        GaussianTrackingRewardTerm(weight=1.0, target=1.0, scale=0.25)
    )
    """Top-down end-effector orientation with a wide alignment plateau."""

    reward_distance_ee_to_obj: GaussianRewardTerm = GaussianRewardTerm(
        weight=4.0, scale=0.15
    )
    """About 61% of peak reward at 15 cm (0.15 m scale)."""

    reward_grasp: SoftThresholdRewardTerm = SoftThresholdRewardTerm(
        weight=6.0, threshold=5.0, operator=">", sharpness=1.0
    )
    """Smooth reward around the 5 N contact threshold."""

    reward_lift: GaussianZoneTrackingRewardTerm = GaussianZoneTrackingRewardTerm(
        weight=12.0, target=0.5, tolerance=0.25, scale=0.15
    )
    """Rewards a 50 cm lift with 25 cm tolerance."""

    reward_distance_obj_to_target: GaussianRewardTerm = GaussianRewardTerm(
        weight=32.0, scale=0.15
    )
    """Largest positive weight; rewards bringing the sample to target."""

    penalty_obj_fallen: LinearRewardTerm = LinearRewardTerm(weight=-16.0, limit=16.0)
    """Penalizes falls without ending the episode."""


@configclass
class SceneCfg(ManipulationSceneCfg):
    sample: RigidObjectCfg = MISSING  # type: ignore[assignment]


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_object_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("sample"),
            "pose_range": MISSING,
            "velocity_range": {},
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    """Pick up a sample and deposit it at a marked target."""

    ## Scenario
    domain: Domain = Domain.MARS

    ## Assets
    scenery: Terrain | AssetVariant | None = AssetVariant.PROCEDURAL
    sample: Object | AssetVariant = AssetVariant.DATASET

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 7.5
    is_finite_horizon: bool = True

    ## Penalty thresholds
    obj_floor_z: float = -0.5
    """World-frame Z threshold for the fall penalty; no termination."""

    ## Target
    tf_pos_target: tuple[float, float, float] = (0.5, 0.0, 0.75)
    tf_quat_target: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": SphereCfg(
                radius=0.02,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        super().__post_init__()

        sample = select_sample(
            self,
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.0)),
            activate_contact_sensors=True,
        )
        self.scene.sample = sample.asset_cfg

        self.events.randomize_object_state.params["pose_range"] = (
            sample.state_randomizer.params["pose_range"]
        )

        # Sensor: End-effector contacts
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                self.scene.sample.prim_path
            ]

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()

        # Mounted on the manipulator flange; spawned with `events` sensors.
        self.env_sensor_cfgs["event_camera_wrist"] = build_wrist_event_camera_cfg(
            f"{{ENV_REGEX_NS}}/robot/{self._robot.frame_flange.prim_relpath}",
            preset_name=self.event_camera_preset,
            backend=self.event_camera_backend,
            sub_render_hz=self.event_camera_sub_render_hz,
            resolution=self.event_camera_resolution,
        )


############
### Task ###
############


class _TaskMixin:
    """Pick up and deposit a sample using shaped approach, grasp, and lift rewards."""

    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._obj: RigidObject = get_scene_rigid_object(self.scene, "sample")
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        ## Initialize buffers
        self._tf_pos_obj_initial = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        self._tf_pos_target = self.scene.env_origins + torch.tensor(
            self.cfg.tf_pos_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

        ## Visualize target
        self._target_marker.visualize(self._tf_pos_target, self._tf_quat_target)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._tf_pos_obj_initial[env_ids] = self._obj.data.root_pos_w[env_ids]

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward

        ################
        ## Raw states ##
        ################
        state = manipulator_step_state(self)
        num_envs, device, dtype = state.num_envs, state.device, state.dtype

        # Transforms (world frame)
        tf_pos_obj = self._obj.data.root_pos_w
        tf_quat_obj = self._obj.data.root_quat_w

        tf_pos_ee_to_obj, tf_quat_ee_to_obj = subtract_frame_transforms(
            t01=state.tf_pos_ee, q01=state.tf_quat_ee, t02=tf_pos_obj, q02=tf_quat_obj
        )
        tf_rotmat_ee_to_obj = matrix_from_quat(tf_quat_ee_to_obj)
        tf_rot6d_ee_to_obj = rotmat_to_rot6d(tf_rotmat_ee_to_obj)

        # Object -> Target
        tf_pos_obj_to_target, tf_quat_obj_to_target = subtract_frame_transforms(
            t01=tf_pos_obj,
            q01=tf_quat_obj,
            t02=self._tf_pos_target,
            q02=self._tf_quat_target,
        )
        tf_rotmat_obj_to_target = matrix_from_quat(tf_quat_obj_to_target)
        tf_rot6d_obj_to_target = rotmat_to_rot6d(tf_rotmat_obj_to_target)

        # Contacts
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

        # Top-down orientation: dot product of EE Z-axis with world down
        down_vec = (
            torch.tensor((0.0, 0.0, -1.0), device=device)
            .unsqueeze(0)
            .expand(num_envs, 3)
        )
        ee_rotmat_w = matrix_from_quat(state.tf_quat_ee)
        top_down_alignment = torch.sum(ee_rotmat_w[:, :, 2] * down_vec, dim=1)

        distance_ee_to_obj = torch.norm(tf_pos_ee_to_obj, dim=-1)

        grasp_metric: torch.Tensor
        if contact_force_matrix_ee is not None:
            grasp_metric = torch.mean(
                torch.max(torch.norm(contact_force_matrix_ee, dim=-1), dim=-1)[0],
                dim=1,
            )
        else:
            grasp_metric = torch.zeros(num_envs, dtype=dtype, device=device)

        # Lift height above initial position
        lift_height = tf_pos_obj[:, 2] - self._tf_pos_obj_initial[:, 2]

        distance_obj_to_target = torch.norm(tf_pos_obj_to_target, dim=-1)

        #############
        ## Rewards ##
        #############
        # Object fallen metric: 1.0 when below floor threshold, 0.0 otherwise
        obj_fallen_metric = (tf_pos_obj[:, 2] < self.cfg.obj_floor_z).float()

        ## Rewards ##
        reward: dict[str, torch.Tensor] = {
            # Penalty: Action rate
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            # Penalty: Joint torque
            "penalty_joint_torque": rew.penalty_joint_torque(joint_torque_metric),
            # Penalty: Joint acceleration
            "penalty_joint_acceleration": rew.penalty_joint_acceleration(
                joint_acc_metric
            ),
            # Penalty: Undesired robot contacts
            "penalty_undesired_robot_contacts": rew.penalty_undesired_robot_contacts(
                undesired_contacts_metric
            ),
            # Reward: End-effector top-down orientation
            "reward_top_down_orientation": rew.reward_top_down_orientation(
                top_down_alignment
            ),
            # Reward: Distance | End-effector <--> Object
            "reward_distance_ee_to_obj": rew.reward_distance_ee_to_obj(
                distance_ee_to_obj
            ),
            # Reward: Grasp object
            "reward_grasp": rew.reward_grasp(grasp_metric),
            # Reward: Lift object
            "reward_lift": rew.reward_lift(lift_height),
            # Reward: Distance | Object <--> Target
            "reward_distance_obj_to_target": rew.reward_distance_obj_to_target(
                distance_obj_to_target
            ),
            "penalty_obj_fallen": rew.penalty_obj_fallen(obj_fallen_metric),
        }

        ##################
        ## Terminations ##
        ##################
        # A fall is penalized but does not end the episode, allowing recovery.
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
                    "tf_pos_obj_to_target": tf_pos_obj_to_target,
                    "tf_rot6d_obj_to_target": tf_rot6d_obj_to_target,
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
