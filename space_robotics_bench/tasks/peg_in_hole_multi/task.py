from dataclasses import MISSING as DELAYED_CFG
from typing import Dict, List, Sequence, Tuple

import torch
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.lab.managers import EventTermCfg, SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor, ContactSensorCfg
from omni.isaac.lab.utils import configclass

import space_robotics_bench.utils.math as math_utils
import space_robotics_bench.utils.sampling as spacing_utils
from space_robotics_bench.core.assets import RigidObject, RigidObjectCfg
from space_robotics_bench.envs import (
    BaseManipulationEnv,
    BaseManipulationEnvCfg,
    BaseManipulationEnvEventCfg,
    mdp,
)

from ..peg_in_hole.task import peg_and_hole_cfg

##############
### Config ###
##############


@configclass
class TaskCfg(BaseManipulationEnvCfg):
    ## Environment
    episode_length_s: float = DELAYED_CFG
    num_problems_per_env: int = 6
    problem_spacing = 0.15

    ## Task
    is_finite_horizon: bool = False

    ## Events
    @configclass
    class EventCfg(BaseManipulationEnvEventCfg):
        pass

    events = EventCfg()

    def __post_init__(self):
        super().__post_init__()

        ## Environment
        self.episode_length_s = self.num_problems_per_env * 10.0

        ## Scene
        (num_rows, num_cols), (grid_spacing_pos, grid_spacing_rot) = (
            spacing_utils.compute_grid_spacing(
                num_instances=self.num_problems_per_env,
                spacing=self.problem_spacing,
                global_pos_offset=(0.5, 0.0, 0.1),
            )
        )
        self.problem_cfgs = [
            peg_and_hole_cfg(
                self.env_cfg,
                prim_path_peg=f"{{ENV_REGEX_NS}}/peg{i}",
                prim_path_hole=f"{{ENV_REGEX_NS}}/hole{i}",
                asset_cfg_peg=SceneEntityCfg(f"object{i}"),
                num_assets=self.scene.num_envs,
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=grid_spacing_pos[i],
                    rot=grid_spacing_rot[i],
                ),
                procgen_seed_offset=i * self.scene.num_envs,
                spawn_kwargs_peg={
                    "activate_contact_sensors": True,
                },
            )
            for i in range(self.num_problems_per_env)
        ]
        for i, problem_cfg in enumerate(self.problem_cfgs):
            setattr(
                self.scene,
                f"object{i}",
                problem_cfg[0].asset_cfg.replace(
                    init_state=RigidObjectCfg.InitialStateCfg(
                        pos=(0.5, 0.0, 0.13),
                    )
                ),
            )
            setattr(
                self.scene,
                f"target{i}",
                problem_cfg[1].asset_cfg,
            )

        ## Sensors
        self.scene.contacts_robot_hand_obj = ContactSensorCfg(
            prim_path=f"{self.scene.robot.prim_path}/{self.robot_cfg.regex_links_hand}",
            update_period=0.0,
            # Note: This causes error 'Filter pattern did not match the correct number of entries'
            #       However, it seems to function properly anyway...
            filter_prim_paths_expr=[
                asset.prim_path
                for asset in [
                    getattr(self.scene, f"object{i}")
                    for i in range(self.num_problems_per_env)
                ]
            ],
        )

        ## Events
        self.events.reset_rand_object_state_multi = EventTermCfg(
            func=mdp.reset_root_state_uniform_poisson_disk_2d,
            mode="reset",
            params={
                "asset_cfgs": [
                    SceneEntityCfg(f"object{i}")
                    for i in range(self.num_problems_per_env)
                ],
                "pose_range": {
                    "x": (
                        -0.5 * (num_rows - 0.5) * self.problem_spacing,
                        0.5 * (num_rows - 0.5) * self.problem_spacing,
                    ),
                    "y": (
                        -0.5 * (num_cols - 0.5) * self.problem_spacing,
                        0.5 * (num_cols - 0.5) * self.problem_spacing,
                    ),
                    "roll": (torch.pi / 2, torch.pi / 2),
                    "pitch": (-torch.pi, torch.pi),
                    "yaw": (-torch.pi, torch.pi),
                },
                "velocity_range": {},
                "radius": 0.1,
            },
        )


############
### Task ###
############


class Task(BaseManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        # Get handles to scene assets
        self._contacts_robot_hand_obj: ContactSensor = self.scene[
            "contacts_robot_hand_obj"
        ]
        self._objects: List[RigidObject] = [
            self.scene[f"object{i}"] for i in range(self.cfg.num_problems_per_env)
        ]
        self._targets: List[XFormPrimView] = [
            self.scene[f"target{i}"] for i in range(self.cfg.num_problems_per_env)
        ]

        ## Pre-compute metrics used in hot loops
        self._robot_arm_joint_indices, _ = self._robot.find_joints(
            self.cfg.robot_cfg.regex_joints_arm
        )
        self._robot_hand_joint_indices, _ = self._robot.find_joints(
            self.cfg.robot_cfg.regex_joints_hand
        )
        self._max_episode_length = self.max_episode_length
        self._obj_com_offset = torch.stack(
            [
                self._objects[i].data._root_physx_view.get_coms().to(self.device)
                for i in range(self.cfg.num_problems_per_env)
            ],
            dim=1,
        )

        ## Initialize buffers
        self._initial_obj_height_w = torch.zeros(
            (self.num_envs, self.cfg.num_problems_per_env),
            dtype=torch.float32,
            device=self.device,
        )
        self._peg_offset_pos_ends = torch.tensor(
            [
                self.cfg.problem_cfgs[i][0].offset_pos_ends
                for i in range(self.cfg.num_problems_per_env)
            ],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1, 1)

        self._peg_rot_symmetry_n = torch.tensor(
            [
                self.cfg.problem_cfgs[i][0].rot_symmetry_n
                for i in range(self.cfg.num_problems_per_env)
            ],
            dtype=torch.int32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._hole_offset_pos_bottom = torch.tensor(
            [
                self.cfg.problem_cfgs[i][1].offset_pos_bottom
                for i in range(self.cfg.num_problems_per_env)
            ],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._hole_offset_pos_entrance = torch.tensor(
            [
                self.cfg.problem_cfgs[i][1].offset_pos_entrance
                for i in range(self.cfg.num_problems_per_env)
            ],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)

        ## Initialize the intermediate state
        self._update_intermediate_state()

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        # Update the initial height of the objects
        self._initial_obj_height_w[env_ids] = torch.stack(
            [
                self._objects[i].data.root_pos_w[env_ids, 2]
                for i in range(self.cfg.num_problems_per_env)
            ],
            dim=1,
        )

    def _get_dones(self) -> Tuple[torch.Tensor, torch.Tensor]:
        # Note: This assumes that `_get_dones()` is called before `_get_rewards()` and `_get_observations()` in `step()`
        self._update_intermediate_state()

        if not self.cfg.enable_truncation:
            self._truncations = torch.zeros_like(self._truncations)

        return self._terminations, self._truncations

    def _get_rewards(self) -> torch.Tensor:
        return self._rewards

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return _construct_observations(
            remaining_time=self._remaining_time,
            robot_joint_pos_arm=self._robot_joint_pos_arm,
            robot_joint_pos_hand=self._robot_joint_pos_hand,
            robot_ee_pos_wrt_base=self._robot_ee_pos_wrt_base,
            robot_ee_rotmat_wrt_base=self._robot_ee_rotmat_wrt_base,
            robot_hand_wrench=self._robot_hand_wrench,
            obj_com_pos_wrt_robot_ee=self._obj_com_pos_wrt_robot_ee,
            obj_com_rotmat_wrt_robot_ee=self._obj_com_rotmat_wrt_robot_ee,
            hole_entrance_pos_wrt_peg_ends=self._hole_entrance_pos_wrt_peg_ends,
            hole_bottom_pos_wrt_peg_ends=self._hole_bottom_pos_wrt_peg_ends,
            hole_rotmat_wrt_peg=self._hole_rotmat_wrt_peg,
        )

    ########################
    ### Helper Functions ###
    ########################

    def _update_intermediate_state(self):
        ## Extract intermediate states
        self._robot_ee_pos_wrt_base = self._tf_robot_ee.data.target_pos_source[:, 0, :]
        self._robot_hand_wrench = (
            self._robot.root_physx_view.get_link_incoming_joint_force()[
                :, self._robot_hand_joint_indices
            ]
        )

        ## Compute other intermediate states
        target_pos_w = torch.stack(
            [
                self._targets[i].get_world_poses()[0]
                for i in range(self.cfg.num_problems_per_env)
            ],
            dim=1,
        )
        target_quat_w = torch.stack(
            [
                self._targets[i].get_world_poses()[1]
                for i in range(self.cfg.num_problems_per_env)
            ],
            dim=1,
        )
        (
            self._remaining_time,
            self._robot_joint_pos_arm,
            self._robot_joint_pos_hand,
            self._robot_ee_rotmat_wrt_base,
            self._obj_com_pos_wrt_robot_ee,
            self._obj_com_rotmat_wrt_robot_ee,
            self._hole_entrance_pos_wrt_peg_ends,
            self._hole_bottom_pos_wrt_peg_ends,
            self._hole_rotmat_wrt_peg,
            self._rewards,
            self._terminations,
            self._truncations,
        ) = _compute_intermediate_state(
            current_action=self.action_manager.action,
            previous_action=self.action_manager.prev_action,
            episode_length_buf=self.episode_length_buf,
            max_episode_length=self._max_episode_length,
            robot_arm_joint_indices=self._robot_arm_joint_indices,
            robot_hand_joint_indices=self._robot_hand_joint_indices,
            joint_pos=self._robot.data.joint_pos,
            soft_joint_pos_limits=self._robot.data.soft_joint_pos_limits,
            robot_ee_pos_w=self._tf_robot_ee.data.target_pos_w[:, 0, :],
            robot_ee_quat_w=self._tf_robot_ee.data.target_quat_w[:, 0, :],
            robot_ee_quat_wrt_base=self._tf_robot_ee.data.target_quat_source[:, 0, :],
            robot_arm_contact_net_forces=self._contacts_robot.data.net_forces_w,
            robot_hand_obj_contact_force_matrix=self._contacts_robot_hand_obj.data.force_matrix_w,
            obj_pos_w=torch.stack(
                [
                    self._objects[i].data.root_pos_w
                    for i in range(self.cfg.num_problems_per_env)
                ],
                dim=1,
            ),
            obj_quat_w=torch.stack(
                [
                    self._objects[i].data.root_quat_w
                    for i in range(self.cfg.num_problems_per_env)
                ],
                dim=1,
            ),
            obj_com_offset=self._obj_com_offset,
            target_pos_w=target_pos_w,
            target_quat_w=target_quat_w,
            initial_obj_height_w=self._initial_obj_height_w,
            peg_offset_pos_ends=self._peg_offset_pos_ends,
            peg_rot_symmetry_n=self._peg_rot_symmetry_n,
            hole_offset_pos_bottom=self._hole_offset_pos_bottom,
            hole_offset_pos_entrance=self._hole_offset_pos_entrance,
        )


#############################
### TorchScript functions ###
#############################


@torch.jit.script
def _compute_intermediate_state(
    *,
    current_action: torch.Tensor,
    previous_action: torch.Tensor,
    episode_length_buf: torch.Tensor,
    max_episode_length: int,
    robot_arm_joint_indices: List[int],
    robot_hand_joint_indices: List[int],
    joint_pos: torch.Tensor,
    soft_joint_pos_limits: torch.Tensor,
    robot_ee_pos_w: torch.Tensor,
    robot_ee_quat_w: torch.Tensor,
    robot_ee_quat_wrt_base: torch.Tensor,
    robot_arm_contact_net_forces: torch.Tensor,
    robot_hand_obj_contact_force_matrix: torch.Tensor,
    obj_pos_w: torch.Tensor,
    obj_quat_w: torch.Tensor,
    obj_com_offset: torch.Tensor,
    target_pos_w: torch.Tensor,
    target_quat_w: torch.Tensor,
    initial_obj_height_w: torch.Tensor,
    peg_offset_pos_ends: torch.Tensor,
    peg_rot_symmetry_n: torch.Tensor,
    hole_offset_pos_bottom: torch.Tensor,
    hole_offset_pos_entrance: torch.Tensor,
) -> Tuple[
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
]:
    ## Intermediate states
    # Time
    remaining_time = 1 - (episode_length_buf / max_episode_length).unsqueeze(-1)

    # Robot joint positions
    joint_pos_normalized = math_utils.scale_transform(
        joint_pos,
        soft_joint_pos_limits[:, :, 0],
        soft_joint_pos_limits[:, :, 1],
    )
    robot_joint_pos_arm, robot_joint_pos_hand = (
        joint_pos_normalized[:, robot_arm_joint_indices],
        joint_pos_normalized[:, robot_hand_joint_indices],
    )

    # End-effector pose (position and '6D' rotation)
    robot_ee_rotmat_wrt_base = math_utils.matrix_from_quat(robot_ee_quat_wrt_base)

    # Transformation | Object origin -> Object CoM
    obj_com_pos_w, obj_com_quat_w = math_utils.combine_frame_transforms(
        t01=obj_pos_w,
        q01=obj_quat_w,
        t12=obj_com_offset[:, :, :3],
        q12=obj_com_offset[:, :, 3:],
    )

    # Transformation | Object origin -> Peg ends
    _peg_end0_pos_w, _ = math_utils.combine_frame_transforms(
        t01=obj_pos_w,
        q01=obj_quat_w,
        t12=peg_offset_pos_ends[:, :, 0],
    )
    _peg_end1_pos_w, _ = math_utils.combine_frame_transforms(
        t01=obj_pos_w,
        q01=obj_quat_w,
        t12=peg_offset_pos_ends[:, :, 1],
    )
    peg_ends_pos_w = torch.stack([_peg_end0_pos_w, _peg_end1_pos_w], dim=1)

    # Transformation | Target origin -> Hole entrance
    hole_entrance_pos_w, _ = math_utils.combine_frame_transforms(
        t01=target_pos_w,
        q01=target_quat_w,
        t12=hole_offset_pos_entrance,
    )

    # Transformation | Target origin -> Hole bottom
    hole_bottom_pos_w, _ = math_utils.combine_frame_transforms(
        t01=target_pos_w,
        q01=target_quat_w,
        t12=hole_offset_pos_bottom,
    )

    # Transformation | End-effector -> Object CoM
    obj_com_pos_wrt_robot_ee, obj_com_quat_wrt_robot_ee = (
        math_utils.subtract_frame_transforms(
            t01=robot_ee_pos_w.unsqueeze(1).repeat(1, obj_pos_w.shape[1], 1),
            q01=robot_ee_quat_w.unsqueeze(1).repeat(1, obj_pos_w.shape[1], 1),
            t02=obj_com_pos_w,
            q02=obj_com_quat_w,
        )
    )
    obj_com_rotmat_wrt_robot_ee = math_utils.matrix_from_quat(obj_com_quat_wrt_robot_ee)

    # Transformation | Peg ends -> Hole entrance
    _hole_entrance_pos_wrt_peg_end0, hole_quat_wrt_peg = (
        math_utils.subtract_frame_transforms(
            t01=peg_ends_pos_w[:, 0],
            q01=obj_quat_w,
            t02=hole_entrance_pos_w,
            q02=target_quat_w,
        )
    )
    _hole_entrance_pos_wrt_peg_end1, _ = math_utils.subtract_frame_transforms(
        t01=peg_ends_pos_w[:, 1],
        q01=obj_quat_w,
        t02=hole_entrance_pos_w,
    )
    hole_entrance_pos_wrt_peg_ends = torch.stack(
        [_hole_entrance_pos_wrt_peg_end0, _hole_entrance_pos_wrt_peg_end1], dim=1
    )
    hole_rotmat_wrt_peg = math_utils.matrix_from_quat(hole_quat_wrt_peg)

    # Transformation | Peg ends -> Hole bottom
    _hole_bottom_pos_wrt_peg_end0, _ = math_utils.subtract_frame_transforms(
        t01=peg_ends_pos_w[:, 0],
        q01=obj_quat_w,
        t02=hole_bottom_pos_w,
    )
    _hole_bottom_pos_wrt_peg_end1, _ = math_utils.subtract_frame_transforms(
        t01=peg_ends_pos_w[:, 1],
        q01=obj_quat_w,
        t02=hole_bottom_pos_w,
    )
    hole_bottom_pos_wrt_peg_ends = torch.stack(
        [_hole_bottom_pos_wrt_peg_end0, _hole_bottom_pos_wrt_peg_end1], dim=1
    )

    ## Rewards
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(current_action - previous_action), dim=1
    )

    # Penalty: Undesired robot arm contacts
    WEIGHT_UNDERSIRED_ROBOT_ARM_CONTACTS = -0.1
    THRESHOLD_UNDERSIRED_ROBOT_ARM_CONTACTS = 10.0
    penalty_undersired_robot_arm_contacts = WEIGHT_UNDERSIRED_ROBOT_ARM_CONTACTS * (
        torch.max(torch.norm(robot_arm_contact_net_forces, dim=-1), dim=1)[0]
        > THRESHOLD_UNDERSIRED_ROBOT_ARM_CONTACTS
    )

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_EE_TO_OBJ = 1.0
    TANH_STD_DISTANCE_EE_TO_OBJ = 0.25
    reward_distance_ee_to_obj = WEIGHT_DISTANCE_EE_TO_OBJ * (
        1.0
        - torch.tanh(
            torch.norm(obj_com_pos_wrt_robot_ee, dim=-1) / TANH_STD_DISTANCE_EE_TO_OBJ
        )
    ).sum(dim=-1)

    # Reward: Object grasped
    WEIGHT_OBJ_GRASPED = 4.0
    THRESHOLD_OBJ_GRASPED = 5.0
    reward_obj_grasped = WEIGHT_OBJ_GRASPED * (
        torch.mean(
            torch.max(torch.norm(robot_hand_obj_contact_force_matrix, dim=-1), dim=-1)[
                0
            ],
            dim=1,
        )
        > THRESHOLD_OBJ_GRASPED
    )

    # Reward: Object lifted
    WEIGHT_OBJ_LIFTED = 8.0
    HEIGHT_OFFSET_OBJ_LIFTED = 0.3
    HEIGHT_SPAN_OBJ_LIFTED = 0.25
    TAHN_STD_HEIGHT_OBJ_LIFTED = 0.05
    obj_target_height_offset = (
        torch.abs(
            obj_com_pos_w[:, :, 2] - initial_obj_height_w - HEIGHT_OFFSET_OBJ_LIFTED
        )
        - HEIGHT_SPAN_OBJ_LIFTED
    ).clamp(min=0.0)
    reward_obj_lifted = WEIGHT_OBJ_LIFTED * (
        1.0 - torch.tanh(obj_target_height_offset / TAHN_STD_HEIGHT_OBJ_LIFTED)
    ).sum(dim=-1)

    # Reward: Alignment | Peg -> Hole | Primary Z axis
    WEIGHT_ALIGN_PEG_TO_HOLE_PRIMARY = 8.0
    TANH_STD_ALIGN_PEG_TO_HOLE_PRIMARY = 0.5
    _peg_to_hole_primary_axis_similarity = torch.abs(hole_rotmat_wrt_peg[:, :, 2, 2])
    reward_align_peg_to_hole_primary = WEIGHT_ALIGN_PEG_TO_HOLE_PRIMARY * (
        1.0
        - torch.tanh(
            (1.0 - _peg_to_hole_primary_axis_similarity)
            / TANH_STD_ALIGN_PEG_TO_HOLE_PRIMARY
        )
    ).sum(dim=-1)

    # Reward: Alignment | Peg -> Hole | Secondary XY axes (affected by primary via power)
    WEIGHT_ALIGN_PEG_TO_HOLE_SECONDARY = 4.0
    TANH_STD_ALIGN_PEG_TO_HOLE_SECONDARY = 0.2
    _peg_to_hole_yaw = torch.atan2(
        hole_rotmat_wrt_peg[:, :, 0, 1], hole_rotmat_wrt_peg[:, :, 0, 0]
    )
    _symmetry_step = 2 * torch.pi / peg_rot_symmetry_n
    _peg_to_hole_yaw_symmetric_directional = _peg_to_hole_yaw % _symmetry_step
    # Note: Lines above might result in NaN/inf when `peg_rot_symmetry_n=0` (infinite circular symmetry)
    #       However, the following `torch.where()` will handle this case
    _peg_to_hole_yaw_symmetric_normalized = torch.where(
        peg_rot_symmetry_n <= 0,
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
    ).sum(dim=-1)

    # Reward: Distance | Peg -> Hole entrance
    WEIGHT_DISTANCE_PEG_TO_HOLE_ENTRANCE = 16.0
    TANH_STD_DISTANCE_PEG_TO_HOLE_ENTRANCE = 0.025
    reward_distance_peg_to_hole_entrance = WEIGHT_DISTANCE_PEG_TO_HOLE_ENTRANCE * (
        1.0
        - torch.tanh(
            torch.min(torch.norm(hole_entrance_pos_wrt_peg_ends, dim=-1), dim=1)[0]
            / TANH_STD_DISTANCE_PEG_TO_HOLE_ENTRANCE
        )
    ).sum(dim=-1)

    # Reward: Distance | Peg -> Hole bottom
    WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM = 128.0
    TANH_STD_DISTANCE_PEG_TO_HOLE_BOTTOM = 0.005
    reward_distance_peg_to_hole_bottom = WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM * (
        1.0
        - torch.tanh(
            torch.min(torch.norm(hole_bottom_pos_wrt_peg_ends, dim=-1), dim=1)[0]
            / TANH_STD_DISTANCE_PEG_TO_HOLE_BOTTOM
        )
    ).sum(dim=-1)

    # Total reward
    rewards = torch.sum(
        torch.stack(
            [
                penalty_action_rate,
                penalty_undersired_robot_arm_contacts,
                reward_distance_ee_to_obj,
                reward_obj_grasped,
                reward_obj_lifted,
                reward_align_peg_to_hole_primary,
                reward_align_peg_to_hole_secondary,
                reward_distance_peg_to_hole_entrance,
                reward_distance_peg_to_hole_bottom,
            ],
            dim=-1,
        ),
        dim=-1,
    )

    ## Termination and truncation
    truncations = episode_length_buf > (max_episode_length - 1)
    terminations = torch.zeros_like(truncations)

    # print(
    #     f"""
    #     penalty |                   action_rate: {float(penalty_action_rate[0])}
    #     penalty | undersired_robot_arm_contacts: {float(penalty_undersired_robot_arm_contacts[0])}
    #     reward  |            distance_ee_to_obj: {float(reward_distance_ee_to_obj[0])}
    #     reward  |                   obj_grasped: {float(reward_obj_grasped[0])}
    #     reward  |                    obj_lifted: {float(reward_obj_lifted[0])}
    #     reward  |     align_peg_to_hole_primary: {float(reward_align_peg_to_hole_primary[0])}
    #     reward  |   align_peg_to_hole_secondary: {float(reward_align_peg_to_hole_secondary[0])}
    #     reward  | distance_peg_to_hole_entrance: {float(reward_distance_peg_to_hole_entrance[0])}
    #     reward  |   distance_peg_to_hole_bottom: {float(reward_distance_peg_to_hole_bottom[0])}
    #                                       total: {float(rewards[0])}
    #   """
    # )

    return (
        remaining_time,
        robot_joint_pos_arm,
        robot_joint_pos_hand,
        robot_ee_rotmat_wrt_base,
        obj_com_pos_wrt_robot_ee,
        obj_com_rotmat_wrt_robot_ee,
        hole_entrance_pos_wrt_peg_ends,
        hole_bottom_pos_wrt_peg_ends,
        hole_rotmat_wrt_peg,
        rewards,
        terminations,
        truncations,
    )


@torch.jit.script
def _construct_observations(
    *,
    remaining_time: torch.Tensor,
    robot_joint_pos_arm: torch.Tensor,
    robot_joint_pos_hand: torch.Tensor,
    robot_ee_pos_wrt_base: torch.Tensor,
    robot_ee_rotmat_wrt_base: torch.Tensor,
    robot_hand_wrench: torch.Tensor,
    obj_com_pos_wrt_robot_ee: torch.Tensor,
    obj_com_rotmat_wrt_robot_ee: torch.Tensor,
    hole_entrance_pos_wrt_peg_ends: torch.Tensor,
    hole_bottom_pos_wrt_peg_ends: torch.Tensor,
    hole_rotmat_wrt_peg: torch.Tensor,
) -> Dict[str, torch.Tensor]:
    """
    Note: The `robot_hand_wrench` is considered as state (robot without force-torque sensors)
    """

    num_envs = remaining_time.size(0)

    # Robot joint positions
    robot_joint_pos_hand_mean = robot_joint_pos_hand.mean(dim=-1, keepdim=True)

    # End-effector pose (position and '6D' rotation)
    robot_ee_rot6d = math_utils.rotmat_to_rot6d(robot_ee_rotmat_wrt_base)

    # Wrench
    robot_hand_wrench_full = robot_hand_wrench.view(num_envs, -1)
    robot_hand_wrench_mean = robot_hand_wrench.mean(dim=1)

    # Transformation | End-effector -> Object CoM
    obj_com_rot6d_wrt_robot_ee = math_utils.rotmat_to_rot6d(obj_com_rotmat_wrt_robot_ee)

    # Transformation | Object -> Target
    hole_6d_wrt_peg = math_utils.rotmat_to_rot6d(hole_rotmat_wrt_peg)

    return {
        "state": torch.cat(
            [
                obj_com_pos_wrt_robot_ee.view(num_envs, -1),
                obj_com_rot6d_wrt_robot_ee.view(num_envs, -1),
                hole_entrance_pos_wrt_peg_ends.view(num_envs, -1),
                hole_bottom_pos_wrt_peg_ends.view(num_envs, -1),
                hole_6d_wrt_peg.view(num_envs, -1),
                robot_hand_wrench_mean.view(num_envs, -1),
            ],
            dim=-1,
        ),
        "state_dyn": torch.cat([robot_hand_wrench_full], dim=-1),
        "proprio": torch.cat(
            [
                remaining_time,
                robot_ee_pos_wrt_base,
                robot_ee_rot6d,
                robot_joint_pos_hand_mean,
            ],
            dim=-1,
        ),
        "proprio_dyn": torch.cat([robot_joint_pos_arm, robot_joint_pos_hand], dim=-1),
    }
