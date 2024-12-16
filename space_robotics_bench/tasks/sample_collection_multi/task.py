from dataclasses import MISSING as DELAYED_CFG
from typing import Dict, List, Sequence, Tuple

import torch
from omni.isaac.lab.managers import EventTermCfg, SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor, ContactSensorCfg
from omni.isaac.lab.utils import configclass

import space_robotics_bench.core.envs as env_utils
import space_robotics_bench.core.sim as sim_utils
import space_robotics_bench.utils.math as math_utils
from space_robotics_bench.core.assets import RigidObject, RigidObjectCfg
from space_robotics_bench.core.markers import (
    VisualizationMarkers,
    VisualizationMarkersCfg,
)
from space_robotics_bench.core.sim.spawners import SphereCfg
from space_robotics_bench.envs import (
    BaseManipulationEnv,
    BaseManipulationEnvCfg,
    BaseManipulationEnvEventCfg,
    mdp,
)

from ..sample_collection.task import sample_cfg

##############
### Config ###
##############


@configclass
class TaskCfg(BaseManipulationEnvCfg):
    ## Environment
    episode_length_s: float = DELAYED_CFG
    num_problems_per_env: int = 8

    ## Task
    is_finite_horizon: bool = False

    ## Goal
    target_pos = (-0.6, 0.0, 0.0)
    target_quat = (1.0, 0.0, 0.0, 0.0)
    target_marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": SphereCfg(
                visible=False,
                radius=0.025,
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(1.0, 0.0, 0.0)
                ),
            )
        },
    )

    ## Events
    @configclass
    class EventCfg(BaseManipulationEnvEventCfg):
        pass

    events = EventCfg()

    def __post_init__(self):
        super().__post_init__()

        ## Environment
        self.episode_length_s = self.num_problems_per_env * 7.5

        ## Scene
        self.object_cfgs = [
            sample_cfg(
                self.env_cfg,
                prim_path=f"{{ENV_REGEX_NS}}/sample{i}",
                asset_cfg=SceneEntityCfg(f"object{i}"),
                num_assets=self.scene.num_envs,
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.55, 0.0, 0.0)),
                procgen_seed_offset=i * self.scene.num_envs,
                spawn_kwargs={
                    "activate_contact_sensors": True,
                },
            )
            for i in range(self.num_problems_per_env)
        ]
        for i, object_cfg in enumerate(self.object_cfgs):
            setattr(self.scene, f"object{i}", object_cfg.asset_cfg)
        if self.vehicle_cfg:
            self.target_pos = self.vehicle_cfg.frame_cargo_bay.offset.translation

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
                "pose_range": self.object_cfgs[0].state_randomizer.params["pose_range"],
                "velocity_range": self.object_cfgs[0].state_randomizer.params[
                    "velocity_range"
                ],
                "radius": (
                    0.225
                    if self.env_cfg.assets.object.variant
                    == env_utils.AssetVariant.DATASET
                    else 0.06
                ),
            },
        )


############
### Task ###
############


class Task(BaseManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get handles to scene assets
        self._contacts_robot_hand_obj: ContactSensor = self.scene[
            "contacts_robot_hand_obj"
        ]
        self._objects: List[RigidObject] = [
            self.scene[f"object{i}"] for i in range(self.cfg.num_problems_per_env)
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
        self._target_pos_w = torch.tensor(
            self.cfg.target_pos, dtype=torch.float32, device=self.device
        ).repeat(
            self.num_envs, self.cfg.num_problems_per_env, 1
        ) + self.scene.env_origins.unsqueeze(1)
        self._target_quat_w = torch.tensor(
            self.cfg.target_quat, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, self.cfg.num_problems_per_env, 1)

        ## Create visualization markers
        self._target_marker = VisualizationMarkers(self.cfg.target_marker_cfg)
        self._target_marker.visualize(
            self._target_pos_w[:, 0, :], self._target_quat_w[:, 0, :]
        )

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
            target_pos_wrt_obj_com=self._target_pos_wrt_obj_com,
            target_rotmat_wrt_obj_com=self._target_rotmat_wrt_obj_com,
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
        (
            self._remaining_time,
            self._robot_joint_pos_arm,
            self._robot_joint_pos_hand,
            self._robot_ee_rotmat_wrt_base,
            self._obj_com_pos_wrt_robot_ee,
            self._obj_com_rotmat_wrt_robot_ee,
            self._target_pos_wrt_obj_com,
            self._target_rotmat_wrt_obj_com,
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
            target_pos_w=self._target_pos_w,
            target_quat_w=self._target_quat_w,
            initial_obj_height_w=self._initial_obj_height_w,
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

    # End-effector '6D' rotation
    robot_ee_rotmat_wrt_base = math_utils.matrix_from_quat(robot_ee_quat_wrt_base)

    # Transformation | Object origin -> Object CoM
    obj_com_pos_w, obj_com_quat_w = math_utils.combine_frame_transforms(
        t01=obj_pos_w,
        q01=obj_quat_w,
        t12=obj_com_offset[:, :, :3],
        q12=obj_com_offset[:, :, 3:],
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

    # Transformation | Object CoM -> Target
    target_pos_wrt_obj_com, target_quat_wrt_obj_com = (
        math_utils.subtract_frame_transforms(
            t01=obj_com_pos_w,
            q01=obj_com_quat_w,
            t02=target_pos_w,
            q02=target_quat_w,
        )
    )
    target_rotmat_wrt_obj_com = math_utils.matrix_from_quat(target_quat_wrt_obj_com)

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
    HEIGHT_OFFSET_OBJ_LIFTED = 0.5
    HEIGHT_SPAN_OBJ_LIFTED = 0.25
    TAHN_STD_HEIGHT_OBJ_LIFTED = 0.1
    obj_target_height_offset = (
        torch.abs(
            obj_com_pos_w[:, :, 2] - initial_obj_height_w - HEIGHT_OFFSET_OBJ_LIFTED
        )
        - HEIGHT_SPAN_OBJ_LIFTED
    ).clamp(min=0.0)
    reward_obj_lifted = WEIGHT_OBJ_LIFTED * (
        1.0 - torch.tanh(obj_target_height_offset / TAHN_STD_HEIGHT_OBJ_LIFTED)
    ).sum(dim=-1)

    # Reward: Distance | Object <--> Target
    WEIGHT_DISTANCE_OBJ_TO_TARGET = 64.0
    TANH_STD_DISTANCE_OBJ_TO_TARGET = 0.333
    reward_distance_obj_to_target = WEIGHT_DISTANCE_OBJ_TO_TARGET * (
        1.0
        - torch.tanh(
            torch.norm(target_pos_wrt_obj_com, dim=-1) / TANH_STD_DISTANCE_OBJ_TO_TARGET
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
                reward_distance_obj_to_target,
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
    #     reward  |        distance_obj_to_target: {float(reward_distance_obj_to_target[0])}
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
        target_pos_wrt_obj_com,
        target_rotmat_wrt_obj_com,
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
    target_pos_wrt_obj_com: torch.Tensor,
    target_rotmat_wrt_obj_com: torch.Tensor,
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

    # Transformation | Object CoM -> Target
    target_rot6d_wrt_obj_com = math_utils.rotmat_to_rot6d(target_rotmat_wrt_obj_com)

    return {
        "state": torch.cat(
            [
                obj_com_pos_wrt_robot_ee.view(num_envs, -1),
                obj_com_rot6d_wrt_robot_ee.view(num_envs, -1),
                target_pos_wrt_obj_com.view(num_envs, -1),
                target_rot6d_wrt_obj_com.view(num_envs, -1),
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
