from dataclasses import MISSING
from typing import Sequence

import torch

from srb._typing import StepReturn
from srb.core.asset import (
    Articulation,
    RigidObjectCfg,
    RigidObjectCollection,
    RigidObjectCollectionCfg,
)
from srb.core.env import ManipulationEnv
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.marker import VisualizationMarkers
from srb.core.mdp import reset_collection_root_state_uniform_poisson_disk_2d
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    rotmat_to_rot6d,
    scale_transform,
    subtract_frame_transforms,
)

from .asset import select_sample
from .task import EventCfg, SceneCfg, TaskCfg

##############
### Config ###
##############


@configclass
class MultiSceneCfg(SceneCfg):
    sample: None = None
    samples: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore
    )


@configclass
class MultiEventCfg(EventCfg):
    randomize_object_state: EventTermCfg = EventTermCfg(
        func=reset_collection_root_state_uniform_poisson_disk_2d,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("samples"),
            "pose_range": MISSING,
            "velocity_range": MISSING,
            "radius": 0.1,
        },
    )


@configclass
class MultiTaskCfg(TaskCfg):
    ## Scene
    scene: MultiSceneCfg = MultiSceneCfg()
    num_samples: int = 8

    ## Events
    events: MultiEventCfg = MultiEventCfg()

    ## Time
    episode_length_s: float = MISSING  # type: ignore
    _base_episode_length_s: float = 7.5

    def __post_init__(self):
        super().__post_init__()

        # Time
        self.episode_length_s = self.num_samples * self._base_episode_length_s

        # Scene: Samples
        self.scene.sample = None
        samples = [
            select_sample(
                self,
                seed=self.seed + (i * self.scene.num_envs),
                prim_path=f"{{ENV_REGEX_NS}}/sample{i}",
                asset_cfg=SceneEntityCfg(f"sample{i}"),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.55, 0.0, 0.0)),
                activate_contact_sensors=True,
            )
            for i in range(self.num_samples)
        ]
        self.scene.samples.rigid_objects = {
            f"sample{i}": cfg.asset_cfg for i, cfg in enumerate(samples)
        }

        # Sensor: End-effector contacts
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                f"{{ENV_REGEX_NS}}/sample{i}" for i in range(self.num_samples)
            ]

        # Events: Randomize object state
        self.events.randomize_object_state.params["pose_range"] = samples[
            0
        ].state_randomizer.params["pose_range"]
        self.events.randomize_object_state.params["velocity_range"] = samples[
            0
        ].state_randomizer.params["velocity_range"]

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()


############
### Task ###
############


class MultiTask(ManipulationEnv):
    cfg: MultiTaskCfg

    def __init__(self, cfg: MultiTaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._objs: RigidObjectCollection = self.scene["samples"]
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        ## Initialize buffers
        self._tf_pos_objs_initial = torch.zeros(
            (self.num_envs, self.cfg.num_samples, 3),
            dtype=torch.float32,
            device=self.device,
        )
        self._tf_pos_target = self.scene.env_origins.unsqueeze(1) + torch.tensor(
            self.cfg.tf_pos_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, self.cfg.num_samples, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, self.cfg.num_samples, 1)

        ## Visualize target
        self._target_marker.visualize(
            self._tf_pos_target[:, 0, :], self._tf_quat_target[:, 0, :]
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._tf_pos_objs_initial[env_ids] = self._objs.data.object_com_pos_w[env_ids]

    def extract_step_return(self) -> StepReturn:
        return _compute_step_return(
            ## Time
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            truncate_episodes=self.cfg.truncate_episodes,
            ## Actions
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            ## States
            # Joints
            joint_pos_robot=self._robot.data.joint_pos,
            joint_pos_limits_robot=(
                self._robot.data.soft_joint_pos_limits
                if torch.all(torch.isfinite(self._robot.data.soft_joint_pos_limits))
                else None
            ),
            joint_pos_end_effector=self._end_effector.data.joint_pos
            if isinstance(self._end_effector, Articulation)
            else None,
            joint_pos_limits_end_effector=(
                self._end_effector.data.soft_joint_pos_limits
                if isinstance(self._end_effector, Articulation)
                and torch.all(
                    torch.isfinite(self._end_effector.data.soft_joint_pos_limits)
                )
                else None
            ),
            # Kinematics
            fk_pos_end_effector=self._tf_end_effector.data.target_pos_source[:, 0, :],
            fk_quat_end_effector=self._tf_end_effector.data.target_quat_source[:, 0, :],
            # Transforms (world frame)
            tf_pos_end_effector=self._tf_end_effector.data.target_pos_w[:, 0, :],
            tf_quat_end_effector=self._tf_end_effector.data.target_quat_w[:, 0, :],
            tf_pos_objs_initial=self._tf_pos_objs_initial,
            tf_pos_objs=self._objs.data.object_com_pos_w,
            tf_quat_objs=self._objs.data.object_com_quat_w,
            tf_pos_target=self._tf_pos_target,
            tf_quat_target=self._tf_quat_target,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            contact_force_matrix_end_effector=self._contacts_end_effector.data.force_matrix_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
        )


@torch.jit.script
def _compute_step_return(
    *,
    ## Time
    episode_length: torch.Tensor,
    max_episode_length: int,
    truncate_episodes: bool,
    ## Actions
    act_current: torch.Tensor,
    act_previous: torch.Tensor,
    ## States
    # Joints
    joint_pos_robot: torch.Tensor,
    joint_pos_limits_robot: torch.Tensor | None,
    joint_pos_end_effector: torch.Tensor | None,
    joint_pos_limits_end_effector: torch.Tensor | None,
    # Kinematics
    fk_pos_end_effector: torch.Tensor,
    fk_quat_end_effector: torch.Tensor,
    # Transforms (world frame)
    tf_pos_end_effector: torch.Tensor,
    tf_quat_end_effector: torch.Tensor,
    tf_pos_objs_initial: torch.Tensor,
    tf_pos_objs: torch.Tensor,
    tf_quat_objs: torch.Tensor,
    tf_pos_target: torch.Tensor,
    tf_quat_target: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    contact_force_matrix_end_effector: torch.Tensor | None,
) -> StepReturn:
    num_envs = episode_length.size(0)
    num_objs = tf_pos_objs.size(1)
    dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
    ## Joints
    # Robot joints
    joint_pos_robot_normalized = (
        scale_transform(
            joint_pos_robot,
            joint_pos_limits_robot[:, :, 0],
            joint_pos_limits_robot[:, :, 1],
        )
        if joint_pos_limits_robot is not None
        else joint_pos_robot
    )
    # End-effector joints
    joint_pos_end_effector_normalized = (
        scale_transform(
            joint_pos_end_effector,
            joint_pos_limits_end_effector[:, :, 0],
            joint_pos_limits_end_effector[:, :, 1],
        )
        if joint_pos_end_effector is not None
        and joint_pos_limits_end_effector is not None
        else (
            joint_pos_end_effector
            if joint_pos_end_effector is not None
            else torch.empty((num_envs, 0), dtype=dtype, device=device)
        )
    )

    ## Kinematics
    fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
    fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

    ## Transforms (world frame)
    # End-effector -> Object
    tf_pos_end_effector_to_objs, tf_quat_end_effector_to_objs = (
        subtract_frame_transforms(
            t01=tf_pos_end_effector.unsqueeze(1).repeat(1, num_objs, 1),
            q01=tf_quat_end_effector.unsqueeze(1).repeat(1, num_objs, 1),
            t02=tf_pos_objs,
            q02=tf_quat_objs,
        )
    )
    tf_rotmat_end_effector_to_objs = matrix_from_quat(tf_quat_end_effector_to_objs)
    tf_rot6d_end_effector_to_objs = rotmat_to_rot6d(tf_rotmat_end_effector_to_objs)
    # Object -> Target
    tf_pos_objs_to_target, tf_quat_objs_to_target = subtract_frame_transforms(
        t01=tf_pos_objs,
        q01=tf_quat_objs,
        t02=tf_pos_target,
        q02=tf_quat_target,
    )
    tf_rotmat_objs_to_target = matrix_from_quat(tf_quat_objs_to_target)
    tf_rot6d_objs_to_target = rotmat_to_rot6d(tf_rotmat_objs_to_target)

    ## Contacts
    contact_forces_mean_robot = contact_forces_robot.mean(dim=1)
    contact_forces_mean_end_effector = (
        contact_forces_end_effector.mean(dim=1)
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=dtype, device=device)
    )
    contact_forces_end_effector = (
        contact_forces_end_effector
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=dtype, device=device)
    )

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    # Penalty: Undesired robot contacts
    WEIGHT_UNDESIRED_ROBOT_CONTACTS = -0.1
    THRESHOLD_UNDESIRED_ROBOT_CONTACTS = 10.0
    penalty_undesired_robot_contacts = WEIGHT_UNDESIRED_ROBOT_CONTACTS * (
        torch.max(torch.norm(contact_forces_robot, dim=-1), dim=1)[0]
        > THRESHOLD_UNDESIRED_ROBOT_CONTACTS
    )

    # Reward: Distance | End-effector <--> Objects
    WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJS = 1.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJS = 0.25
    reward_distance_end_effector_to_objs = WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJS * (
        1.0
        - torch.tanh(
            torch.norm(tf_pos_end_effector_to_objs, dim=-1)
            / TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJS
        )
    ).sum(dim=1)

    # Reward: Grasp object
    WEIGHT_GRASP = 4.0
    THRESHOLD_GRASP = 5.0
    reward_grasp = (
        WEIGHT_GRASP
        * (
            torch.mean(
                torch.max(
                    torch.norm(contact_force_matrix_end_effector, dim=-1), dim=-1
                )[0],
                dim=1,
            )
            > THRESHOLD_GRASP
        )
        if contact_force_matrix_end_effector is not None
        else torch.zeros(num_envs, dtype=dtype, device=device)
    )

    # Reward: Lift object
    WEIGHT_LIFT = 16.0
    HEIGHT_OFFSET_LIFT = 0.5
    HEIGHT_SPAN_LIFT = 0.25
    TANH_STD_HEIGHT_LIFT = 0.1
    reward_lift = WEIGHT_LIFT * (
        1.0
        - torch.tanh(
            (
                torch.abs(
                    tf_pos_objs[:, :, 2]
                    - tf_pos_objs_initial[:, :, 2]
                    - HEIGHT_OFFSET_LIFT
                )
                - HEIGHT_SPAN_LIFT
            ).clamp(min=0.0)
            / TANH_STD_HEIGHT_LIFT
        )
    ).sum(dim=1)

    # Reward: Distance | Object <--> Target
    WEIGHT_DISTANCE_OBJS_TO_TARGET = 32.0
    TANH_STD_DISTANCE_OBJS_TO_TARGET = 0.333
    reward_distance_objs_to_target = WEIGHT_DISTANCE_OBJS_TO_TARGET * (
        1.0
        - torch.tanh(
            torch.norm(tf_pos_objs_to_target, dim=-1) / TANH_STD_DISTANCE_OBJS_TO_TARGET
        )
    ).sum(dim=1)

    ##################
    ## Terminations ##
    ##################
    # No termination condition
    termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
    # Truncation
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=device)
    )

    return StepReturn(
        {
            "state": {
                "contact_forces_mean_robot": contact_forces_mean_robot,
                "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
                "tf_pos_end_effector_to_objs": tf_pos_end_effector_to_objs,
                "tf_rot6d_end_effector_to_objs": tf_rot6d_end_effector_to_objs,
                "tf_pos_objs_to_target": tf_pos_objs_to_target,
                "tf_rot6d_objs_to_target": tf_rot6d_objs_to_target,
            },
            "state_dyn": {
                "contact_forces_robot": contact_forces_robot,
                "contact_forces_end_effector": contact_forces_end_effector,
            },
            "proprio": {
                "fk_pos_end_effector": fk_pos_end_effector,
                "fk_rot6d_end_effector": fk_rot6d_end_effector,
            },
            "proprio_dyn": {
                "joint_pos_robot_normalized": joint_pos_robot_normalized,
                "joint_pos_end_effector_normalized": joint_pos_end_effector_normalized,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_undesired_robot_contacts": penalty_undesired_robot_contacts,
            "reward_distance_end_effector_to_objs": reward_distance_end_effector_to_objs,
            "reward_grasp": reward_grasp,
            "reward_lift": reward_lift,
            "reward_distance_objs_to_target": reward_distance_objs_to_target,
        },
        termination,
        truncation,
    )
