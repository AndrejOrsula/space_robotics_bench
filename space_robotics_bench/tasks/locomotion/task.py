from typing import Dict, List, Sequence, Tuple

import torch
from omni.isaac.lab.utils import configclass

import space_robotics_bench.utils.math as math_utils
from space_robotics_bench.envs import (
    BaseLocomotionEnv,
    BaseLocomotionEnvCfg,
    BaseLocomotionEnvEventCfg,
)

##############
### Config ###
##############


@configclass
class TaskCfg(BaseLocomotionEnvCfg):
    ## Environment
    episode_length_s: float = 30.0

    ## Task
    is_finite_horizon: bool = False

    ## Events
    @configclass
    class EventCfg(BaseLocomotionEnvEventCfg):
        pass

    events = EventCfg()

    def __post_init__(self):
        super().__post_init__()


############
### Task ###
############


class Task(BaseLocomotionEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Pre-compute metrics used in hot loops
        self._robot_feet_indices, _ = self._robot.find_bodies(".*FOOT")
        self._robot_undesired_contact_body_indices, _ = self._robot.find_bodies(
            ["base", ".*THIGH"]
        )
        self._max_episode_length = self.max_episode_length

        ## Initialize buffers
        self._command = torch.zeros(self.num_envs, 3, device=self.device)

        ## Initialize the intermediate state
        self._update_intermediate_state()

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        self._command[env_ids] = math_utils.sample_uniform(
            -1.0, 1.0, (len(env_ids), 3), device=self.device
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
            robot_joint_pos=self._robot_joint_pos,
            robot_root_rotmat_w=self._robot_root_rotmat_w,
            robot_feet_incoming_force=self._robot_feet_incoming_force,
            heightmap=self._heightmap,
            command=self._command,
        )

    ########################
    ### Helper Functions ###
    ########################

    def _update_intermediate_state(self):
        ## Extract intermediate states
        self._robot_joint_pos = self._robot.data.joint_pos
        self._robot_feet_incoming_force = (
            self._robot.root_physx_view.get_link_incoming_joint_force()[
                :, self._robot_feet_indices
            ]
        )

        ## Compute other intermediate states
        (
            self._remaining_time,
            self._robot_root_rotmat_w,
            self._heightmap,
            self._rewards,
            self._terminations,
            self._truncations,
        ) = _compute_intermediate_state(
            current_action=self.action_manager.action,
            previous_action=self.action_manager.prev_action,
            episode_length_buf=self.episode_length_buf,
            max_episode_length=self._max_episode_length,
            joint_acc=self._robot.data.joint_acc,
            joint_applied_torque=self._robot.data.applied_torque,
            root_quat_w=self._robot.data.root_quat_w,
            root_lin_vel=self._robot.data.root_lin_vel_b,
            root_ang_vel=self._robot.data.root_ang_vel_b,
            root_projected_gravity=self._robot.data.projected_gravity_b,
            robot_feet_indices=self._robot_feet_indices,
            robot_undesired_contact_body_indices=self._robot_undesired_contact_body_indices,
            contact_net_forces=self._contacts_robot.data.net_forces_w,
            first_contact=self._contacts_robot.compute_first_contact(self.step_dt),
            last_air_time=self._contacts_robot.data.last_air_time,
            height_scanner_pos_w=self._height_scanner.data.pos_w,
            height_scanner_ray_hits_w=self._height_scanner.data.ray_hits_w,
            command=self._command,
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
    joint_acc: torch.Tensor,
    joint_applied_torque: torch.Tensor,
    root_quat_w: torch.Tensor,
    root_lin_vel: torch.Tensor,
    root_ang_vel: torch.Tensor,
    root_projected_gravity: torch.Tensor,
    robot_feet_indices: List[int],
    robot_undesired_contact_body_indices: List[int],
    contact_net_forces: torch.Tensor,
    first_contact: torch.Tensor,
    last_air_time: torch.Tensor,
    height_scanner_pos_w: torch.Tensor,
    height_scanner_ray_hits_w: torch.Tensor,
    command: torch.Tensor,
) -> Tuple[
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

    # Robot '6D' rotation
    robot_root_rotmat_w = math_utils.matrix_from_quat(root_quat_w)

    # Height scanner
    heightmap = (
        height_scanner_pos_w[:, 2].unsqueeze(1)
        - height_scanner_ray_hits_w[..., 2]
        - 0.5
    ).clip(-1.0, 1.0)

    ## Rewards
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.01
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(current_action - previous_action), dim=1
    )

    # Penalty: Undesired contacts
    WEIGHT_UNDERSIRED_CONTACTS = -1.0
    THRESHOLD_UNDERSIRED_CONTACTS = 1.0
    penalty_undersired_contacts = WEIGHT_UNDERSIRED_CONTACTS * (
        torch.max(
            torch.norm(
                contact_net_forces[:, robot_undesired_contact_body_indices, :],
                dim=-1,
            ),
            dim=1,
        )[0]
        > THRESHOLD_UNDERSIRED_CONTACTS
    )

    # Reward: Command tracking (linear)
    WEIGHT_CMD_LIN_VEL_XY = 1.0
    EXP_STD_CMD_LIN_VEL_XY = 0.25
    reward_cmd_lin_vel_xy = WEIGHT_CMD_LIN_VEL_XY * torch.exp(
        -torch.sum(torch.square(command[:, :2] - root_lin_vel[:, :2]), dim=1)
        / EXP_STD_CMD_LIN_VEL_XY
    )

    # Reward: Command tracking (angular)
    WEIGHT_CMD_ANG_VEL_Z = 0.5
    EXP_STD_CMD_ANG_VEL_Z = 0.25
    reward_cmd_ang_vel_z = WEIGHT_CMD_ANG_VEL_Z * torch.exp(
        -torch.square(command[:, 2] - root_ang_vel[:, 2]) / EXP_STD_CMD_ANG_VEL_Z
    )

    # Reward: Feet air time
    WEIGHT_FEET_AIR_TIME = 0.5
    THRESHOLD_FEET_AIR_TIME = 0.1
    reward_feet_air_time = (
        WEIGHT_FEET_AIR_TIME
        * (torch.norm(command[:, :2], dim=1) > THRESHOLD_FEET_AIR_TIME)
        * torch.sum(
            (last_air_time[:, robot_feet_indices] - 0.5)
            * first_contact[:, robot_feet_indices],
            dim=1,
        )
    )

    # Penalty: Minimize non-command motion (linear)
    WEIGHT_UNDESIRED_LIN_VEL_Z = -2.0
    penalty_undesired_lin_vel_z = WEIGHT_UNDESIRED_LIN_VEL_Z * torch.square(
        root_lin_vel[:, 2]
    )

    # Penalty: Minimize non-command motion (angular)
    WEIGHT_UNDESIRED_ANG_VEL_XY = -0.05
    penalty_undesired_ang_vel_xy = WEIGHT_UNDESIRED_ANG_VEL_XY * torch.sum(
        torch.square(root_ang_vel[:, :2]), dim=-1
    )

    # Penalty: Joint torque
    WEIGHT_JOINT_TORQUE = -0.000025
    penalty_joint_torque = WEIGHT_JOINT_TORQUE * torch.sum(
        torch.square(joint_applied_torque), dim=1
    )

    # Penalty: Joint acceleration
    WEIGHT_JOINT_ACCELERATION = -0.00000025
    penalty_joint_acceleration = WEIGHT_JOINT_ACCELERATION * torch.sum(
        torch.square(joint_acc), dim=1
    )

    # Penalty: Minimize rotation with the gravity direction
    WEIGHT_GRAVITY_ROTATION_ALIGNMENT = -5.0
    penalty_gravity_rotation_alignment = WEIGHT_GRAVITY_ROTATION_ALIGNMENT * torch.sum(
        torch.square(root_projected_gravity[:, :2]), dim=1
    )

    # Total reward
    rewards = torch.sum(
        torch.stack(
            [
                penalty_action_rate,
                penalty_undersired_contacts,
                reward_cmd_lin_vel_xy,
                reward_cmd_ang_vel_z,
                reward_feet_air_time,
                penalty_undesired_lin_vel_z,
                penalty_undesired_ang_vel_xy,
                penalty_joint_torque,
                penalty_joint_acceleration,
                penalty_gravity_rotation_alignment,
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
    #     penalty |                action_rate: {float(penalty_action_rate[0])}
    #     penalty |        undersired_contacts: {float(penalty_undersired_contacts[0])}
    #     reward  |             cmd_lin_vel_xy: {float(reward_cmd_lin_vel_xy[0])}
    #     reward  |              cmd_ang_vel_z: {float(reward_cmd_ang_vel_z[0])}
    #     reward  |              feet_air_time: {float(reward_feet_air_time[0])}
    #     penalty |        undesired_lin_vel_z: {float(penalty_undesired_lin_vel_z[0])}
    #     penalty |       undesired_ang_vel_xy: {float(penalty_undesired_ang_vel_xy[0])}
    #     penalty |               joint_torque: {float(penalty_joint_torque[0])}
    #     penalty |         joint_acceleration: {float(penalty_joint_acceleration[0])}
    #     penalty | gravity_rotation_alignment: {float(penalty_gravity_rotation_alignment[0])}
    #   """
    # )

    return (
        remaining_time,
        robot_root_rotmat_w,
        heightmap,
        rewards,
        terminations,
        truncations,
    )


@torch.jit.script
def _construct_observations(
    *,
    remaining_time: torch.Tensor,
    robot_joint_pos: torch.Tensor,
    robot_root_rotmat_w: torch.Tensor,
    robot_feet_incoming_force: torch.Tensor,
    heightmap: torch.Tensor,
    command: torch.Tensor,
) -> Dict[str, torch.Tensor]:
    """
    Note: The `robot_hand_wrench` is considered as state (robot without force-torque sensors)
    """

    num_envs = remaining_time.size(0)

    # End-effector pose (position and '6D' rotation)
    robot_root_rot6d_w = math_utils.rotmat_to_rot6d(robot_root_rotmat_w)

    # Wrench
    robot_feet_incoming_force_full = robot_feet_incoming_force.view(num_envs, -1)

    return {
        "state": torch.cat(
            [
                heightmap,
            ],
            dim=-1,
        ),
        "state_dyn": torch.cat([robot_feet_incoming_force_full], dim=-1),
        "proprio": torch.cat(
            [
                remaining_time,
                robot_root_rot6d_w,
            ],
            dim=-1,
        ),
        "proprio_dyn": torch.cat([robot_joint_pos], dim=-1),
        "command": command,
    }
