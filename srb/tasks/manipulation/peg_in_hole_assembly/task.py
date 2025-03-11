from dataclasses import MISSING
from typing import Sequence

import torch

from srb._typing import StepReturn
from srb.core.asset import (
    Articulation,
    AssetVariant,
    Object,
    RigidObject,
    RigidObjectCfg,
)
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    rotmat_to_rot6d,
    scale_transform,
    subtract_frame_transforms,
)

from .asset import select_peg_in_hole_assembly

##############
### Config ###
##############


@configclass
class SceneCfg(ManipulationSceneCfg):
    peg: RigidObjectCfg = MISSING  # type: ignore
    hole: RigidObjectCfg = MISSING  # type: ignore


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_object_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("peg"),
            "pose_range": {
                "x": (-0.25 - 0.025, -0.25 + 0.0125),
                "y": (-0.05, 0.05),
                "roll": (torch.pi / 2, torch.pi / 2),
                "yaw": (
                    torch.pi / 2 - torch.pi / 16,
                    torch.pi / 2 + torch.pi / 16,
                ),
            },
            "velocity_range": {},
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    ## Assets
    peg: Object | AssetVariant | None = AssetVariant.DATASET

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 10.0
    is_finite_horizon: bool = True

    def __post_init__(self):
        super().__post_init__()

        # Task setup: Peg-in-hole assembly
        self.peg_in_hole_assembly = select_peg_in_hole_assembly(
            self,
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.02)),
            peg_kwargs={
                "activate_contact_sensors": True,
            },
        )
        # Scene: Peg
        self.scene.peg = self.peg_in_hole_assembly.peg.asset_cfg
        # Scene: Hole
        self.scene.hole = self.peg_in_hole_assembly.hole.asset_cfg

        # Sensor: End-effector contacts
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                self.scene.peg.prim_path
            ]

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()


############
### Task ###
############


class Task(ManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._obj: RigidObject = self.scene["peg"]
        self._target: RigidObject = self.scene["hole"]

        ## Initialize buffers
        self._tf_pos_obj_initial = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        self._offset_pos_peg_ends = torch.tensor(
            self.cfg.peg_in_hole_assembly.peg.offset_pos_ends,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._symmetry_peg = torch.tensor(
            self.cfg.peg_in_hole_assembly.peg.symmetry,
            dtype=torch.int32,
            device=self.device,
        ).repeat(self.num_envs)
        self._offset_pos_hole_bottom = torch.tensor(
            self.cfg.peg_in_hole_assembly.hole.offset_pos_bottom,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._offset_pos_hole_entrance = torch.tensor(
            self.cfg.peg_in_hole_assembly.hole.offset_pos_entrance,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._tf_pos_obj_initial[env_ids] = self._obj.data.root_pos_w[env_ids]

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
            tf_pos_obj_initial=self._tf_pos_obj_initial,
            tf_pos_obj=self._obj.data.root_pos_w,
            tf_quat_obj=self._obj.data.root_quat_w,
            tf_pos_target=self._target.data.root_pos_w,
            tf_quat_target=self._target.data.root_quat_w,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            contact_force_matrix_end_effector=self._contacts_end_effector.data.force_matrix_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            ## Peg-in-hole descriptors
            offset_pos_peg_ends=self._offset_pos_peg_ends,
            symmetry_peg=self._symmetry_peg,
            offset_pos_hole_bottom=self._offset_pos_hole_bottom,
            offset_pos_hole_entrance=self._offset_pos_hole_entrance,
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
    tf_pos_obj_initial: torch.Tensor,
    tf_pos_obj: torch.Tensor,
    tf_quat_obj: torch.Tensor,
    tf_pos_target: torch.Tensor,
    tf_quat_target: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    contact_force_matrix_end_effector: torch.Tensor | None,
    ## Peg-in-hole descriptors
    offset_pos_peg_ends: torch.Tensor,
    symmetry_peg: torch.Tensor,
    offset_pos_hole_bottom: torch.Tensor,
    offset_pos_hole_entrance: torch.Tensor,
) -> StepReturn:
    num_envs = episode_length.size(0)
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
    tf_pos_end_effector_to_obj, tf_quat_end_effector_to_obj = subtract_frame_transforms(
        t01=tf_pos_end_effector,
        q01=tf_quat_end_effector,
        t02=tf_pos_obj,
        q02=tf_quat_obj,
    )
    tf_rotmat_end_effector_to_obj = matrix_from_quat(tf_quat_end_effector_to_obj)
    tf_rot6d_end_effector_to_obj = rotmat_to_rot6d(tf_rotmat_end_effector_to_obj)

    # Peg -> Peg ends
    _tf_pos_peg_end0, _ = combine_frame_transforms(
        t01=tf_pos_obj,
        q01=tf_quat_obj,
        t12=offset_pos_peg_ends[:, 0],
    )
    _tf_pos_peg_end1, _ = combine_frame_transforms(
        t01=tf_pos_obj,
        q01=tf_quat_obj,
        t12=offset_pos_peg_ends[:, 1],
    )
    tf_pos_peg_ends = torch.stack([_tf_pos_peg_end0, _tf_pos_peg_end1], dim=1)

    # Hole -> Hole entrance | bottom
    tf_pos_hole_entrance, _ = combine_frame_transforms(
        t01=tf_pos_target,
        q01=tf_quat_target,
        t12=offset_pos_hole_entrance,
    )
    tf_pos_hole_bottom, _ = combine_frame_transforms(
        t01=tf_pos_target,
        q01=tf_quat_target,
        t12=offset_pos_hole_bottom,
    )

    # Peg ends -> Hole entrance
    tf_pos_peg_end0_to_hole_entrance, tf_quat_peg_to_hole_entrance = (
        subtract_frame_transforms(
            t01=tf_pos_peg_ends[:, 0],
            q01=tf_quat_obj,
            t02=tf_pos_hole_entrance,
            q02=tf_quat_target,
        )
    )
    tf_pos_peg_end1_to_hole_entrance, _ = subtract_frame_transforms(
        t01=tf_pos_peg_ends[:, 1],
        q01=tf_quat_obj,
        t02=tf_pos_hole_entrance,
    )
    tf_pos_peg_ends_to_hole_entrance = torch.stack(
        [tf_pos_peg_end0_to_hole_entrance, tf_pos_peg_end1_to_hole_entrance], dim=1
    )
    tf_rotmat_peg_to_hole = matrix_from_quat(tf_quat_peg_to_hole_entrance)
    tf_rot6d_peg_to_hole = rotmat_to_rot6d(tf_rotmat_peg_to_hole)

    # Peg ends -> Hole bottom
    tf_pos_peg_end0_to_hole_bottom, _ = subtract_frame_transforms(
        t01=tf_pos_peg_ends[:, 0],
        q01=tf_quat_obj,
        t02=tf_pos_hole_bottom,
    )
    tf_pos_peg_end1_to_hole_bottom, _ = subtract_frame_transforms(
        t01=tf_pos_peg_ends[:, 1],
        q01=tf_quat_obj,
        t02=tf_pos_hole_bottom,
    )
    tf_pos_peg_ends_to_hole_bottom = torch.stack(
        [tf_pos_peg_end0_to_hole_bottom, tf_pos_peg_end1_to_hole_bottom], dim=1
    )

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

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ = 1.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ = 0.25
    reward_distance_end_effector_to_obj = WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ * (
        1.0
        - torch.tanh(
            torch.norm(tf_pos_end_effector_to_obj, dim=-1)
            / TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ
        )
    )

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
                    tf_pos_obj[:, 2] - tf_pos_obj_initial[:, 2] - HEIGHT_OFFSET_LIFT
                )
                - HEIGHT_SPAN_LIFT
            ).clamp(min=0.0)
            / TANH_STD_HEIGHT_LIFT
        )
    )

    # Reward: Alignment | Peg -> Hole | Primary Z axis
    WEIGHT_ALIGN_PEG_TO_HOLE_PRIMARY = 8.0
    TANH_STD_ALIGN_PEG_TO_HOLE_PRIMARY = 0.5
    _peg_to_hole_primary_axis_similarity = torch.abs(tf_rotmat_peg_to_hole[:, 2, 2])
    reward_align_peg_to_hole_primary = WEIGHT_ALIGN_PEG_TO_HOLE_PRIMARY * (
        1.0
        - torch.tanh(
            (1.0 - _peg_to_hole_primary_axis_similarity)
            / TANH_STD_ALIGN_PEG_TO_HOLE_PRIMARY
        )
    )

    # Reward: Alignment | Peg -> Hole | Secondary XY axes (affected by primary via power)
    WEIGHT_ALIGN_PEG_TO_HOLE_SECONDARY = 4.0
    TANH_STD_ALIGN_PEG_TO_HOLE_SECONDARY = 0.2
    _peg_to_hole_yaw = torch.atan2(
        tf_rotmat_peg_to_hole[:, 0, 1], tf_rotmat_peg_to_hole[:, 0, 0]
    )
    _symmetry_step = 2 * torch.pi / symmetry_peg
    _peg_to_hole_yaw_symmetric_directional = _peg_to_hole_yaw % _symmetry_step
    # Note: Lines above might result in NaN/inf when `peg_rot_symmetry_n=0` (infinite circular symmetry)
    #       However, the following `torch.where()` will handle this case
    _peg_to_hole_yaw_symmetric_normalized = torch.where(
        symmetry_peg <= 0,
        0.0,
        torch.min(
            _peg_to_hole_yaw_symmetric_directional,
            _symmetry_step - _peg_to_hole_yaw_symmetric_directional,
        )
        / (_symmetry_step / 2.0),
    )
    reward_align_peg_to_hole_secondary = WEIGHT_ALIGN_PEG_TO_HOLE_SECONDARY * (
        1.0
        - torch.tanh(
            _peg_to_hole_yaw_symmetric_normalized.pow(
                _peg_to_hole_primary_axis_similarity
            )
            / TANH_STD_ALIGN_PEG_TO_HOLE_SECONDARY
        )
    )

    # Reward: Distance | Peg -> Hole entrance
    WEIGHT_DISTANCE_PEG_TO_HOLE_ENTRANCE = 16.0
    TANH_STD_DISTANCE_PEG_TO_HOLE_ENTRANCE = 0.05
    reward_distance_peg_to_hole_entrance = WEIGHT_DISTANCE_PEG_TO_HOLE_ENTRANCE * (
        1.0
        - torch.tanh(
            torch.min(torch.norm(tf_pos_peg_ends_to_hole_entrance, dim=-1), dim=1)[0]
            / TANH_STD_DISTANCE_PEG_TO_HOLE_ENTRANCE
        )
    )

    # Reward: Distance | Peg -> Hole bottom
    WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM = 128.0
    TANH_STD_DISTANCE_PEG_TO_HOLE_BOTTOM = 0.005
    reward_distance_peg_to_hole_bottom = WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM * (
        1.0
        - torch.tanh(
            torch.min(torch.norm(tf_pos_peg_ends_to_hole_bottom, dim=-1), dim=1)[0]
            / TANH_STD_DISTANCE_PEG_TO_HOLE_BOTTOM
        )
    )

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
                "tf_pos_end_effector_to_obj": tf_pos_end_effector_to_obj,
                "tf_rot6d_end_effector_to_obj": tf_rot6d_end_effector_to_obj,
                "tf_pos_peg_end0_to_hole_entrance": tf_pos_peg_end0_to_hole_entrance,
                "tf_pos_peg_end1_to_hole_entrance": tf_pos_peg_end1_to_hole_entrance,
                "tf_pos_peg_end0_to_hole_bottom": tf_pos_peg_end0_to_hole_bottom,
                "tf_pos_peg_end1_to_hole_bottom": tf_pos_peg_end1_to_hole_bottom,
                "tf_rot6d_peg_to_hole": tf_rot6d_peg_to_hole,
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
            "reward_distance_end_effector_to_obj": reward_distance_end_effector_to_obj,
            "reward_grasp": reward_grasp,
            "reward_lift": reward_lift,
            "reward_align_peg_to_hole_primary": reward_align_peg_to_hole_primary,
            "reward_align_peg_to_hole_secondary": reward_align_peg_to_hole_secondary,
            "reward_distance_peg_to_hole_entrance": reward_distance_peg_to_hole_entrance,
            "reward_distance_peg_to_hole_bottom": reward_distance_peg_to_hole_bottom,
        },
        termination,
        truncation,
    )
