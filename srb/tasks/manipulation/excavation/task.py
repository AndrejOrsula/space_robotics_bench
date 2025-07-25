from dataclasses import MISSING
from typing import Sequence

import torch

import srb.core.sim.spawners.particles.utils as particle_utils
from srb import assets
from srb._typing import StepReturn
from srb.core.asset import (
    Articulation,
    AssetBase,
    AssetBaseCfg,
    AssetVariant,
    Manipulator,
)
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.sensor import ContactSensor
from srb.core.sim import PyramidParticlesSpawnerCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d, scale_transform

##############
### Config ###
##############


@configclass
class SceneCfg(ManipulationSceneCfg):
    regolith = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/regolith",
        spawn=PyramidParticlesSpawnerCfg(
            ratio=MISSING,  # type: ignore
            particle_size=MISSING,  # type: ignore
            dim_x=MISSING,  # type: ignore
            dim_y=MISSING,  # type: ignore
            dim_z=MISSING,  # type: ignore
            velocity=((-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.0)),
            fluid=False,
            density=1500.0,
            friction=0.85,
            cohesion=0.65,
        ),
    )


@configclass
class EventCfg(ManipulationEventCfg):
    pass


@configclass
class TaskCfg(ManipulationEnvCfg):
    ## Assets
    robot: Manipulator | AssetVariant = assets.Franka(
        end_effector=assets.ScoopRectangular()
    )

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    env_rate: float = 1.0 / 200.0
    episode_length_s: float = 20.0
    is_finite_horizon: bool = True

    ## Particles
    particles: bool = True
    particles_ratio: float = 0.8
    particles_size: float = 0.01
    particles_update_interval: float = 2.0

    ## Assemblies (dynamic joints)
    assemble_rigid_end_effector: bool = False

    def __post_init__(self):
        super().__post_init__()
        assert self.particles, "Particles must be enabled for this task"

        # Disable default particles
        self.scene.particles = None  # type: ignore
        self._settle_down_particles()

        # Scene: Regolith
        assert self.spacing is not None
        _regolith_dim = round(self.spacing / self.particles_size)
        self.scene.regolith.spawn.ratio = self.particles_ratio  # type: ignore
        self.scene.regolith.spawn.particle_size = self.particles_size  # type: ignore
        self.scene.regolith.spawn.dim_x = round(0.225 * _regolith_dim)  # type: ignore
        self.scene.regolith.spawn.dim_y = round(0.35 * _regolith_dim)  # type: ignore
        self.scene.regolith.spawn.dim_z = round(0.075 * _regolith_dim)  # type: ignore
        self.scene.regolith.init_state.pos = (
            0.15 * self.spacing,
            0.0,
            0.05 * self.spacing,
        )

        # Async particle updates
        self._particle_update_interval_n_steps: int = round(
            self.particles_update_interval / self.agent_rate
        )


############
### Task ###
############


class Task(ManipulationEnv):
    cfg: TaskCfg

    # Note: Initial position of settled particles is coming from the `settle_and_reset_particles` event
    INITIAL_PARTICLE_POS_IDENT: str = "__particles_regolith_initial_pos"
    INITIAL_PARTICLE_VEL_IDENT: str = "__particles_regolith_initial_vel"

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._regolith: AssetBase = self.scene["regolith"]

        ## Initialize buffers
        self._particle_update_counter: int = self.cfg._particle_update_interval_n_steps
        self._initial_particle_mean_pos: torch.Tensor = torch.zeros(
            self.num_envs, 3, dtype=torch.float32, device=self.device
        )
        self._cached_particles_pos = torch.zeros(
            self.num_envs,
            1,
            3,
            dtype=torch.float32,
            device=self.device,
        )
        self._cached_particles_vel = torch.zeros(
            self.num_envs,
            1,
            3,
            dtype=torch.float32,
            device=self.device,
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        ## Reset particles
        if not hasattr(self, self.INITIAL_PARTICLE_POS_IDENT):
            super()._reset_idx(env_ids)

            # Determine the mean position of the settled particles
            initial_particle_pos = getattr(self, self.INITIAL_PARTICLE_POS_IDENT)
            self._initial_particle_mean_pos[:, :2] = self.scene.env_origins[
                :, :2
            ] + torch.mean(initial_particle_pos[:, :, :2], dim=1)
            self._initial_particle_mean_pos[:, 2] = torch.quantile(
                initial_particle_pos[:, :, 2], q=0.95
            )

            # Initialize the particle cache
            self._particle_update_counter = self.cfg._particle_update_interval_n_steps
            self._cached_particles_pos = initial_particle_pos.clone()
            self._cached_particles_vel = getattr(
                self, self.INITIAL_PARTICLE_VEL_IDENT
            ).clone()
        else:
            self._particle_update_counter = self.cfg._particle_update_interval_n_steps
            self._cached_particles_pos[env_ids] = getattr(
                self, self.INITIAL_PARTICLE_POS_IDENT
            )[env_ids]
            self._cached_particles_vel[env_ids] = getattr(
                self, self.INITIAL_PARTICLE_VEL_IDENT
            )[env_ids]

        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        # Update particle cache if interval is reached
        self._particle_update_counter -= 1
        if self._particle_update_counter == 0:
            self._cached_particles_pos = particle_utils.get_particles_pos_w(
                self, self._regolith
            )
            self._cached_particles_vel = particle_utils.get_particles_vel_w(
                self, self._regolith
            )
            self._particle_update_counter = self.cfg._particle_update_interval_n_steps

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
            joint_acc_robot=self._robot.data.joint_acc,
            joint_applied_torque_robot=self._robot.data.applied_torque,
            # Kinematics
            fk_pos_end_effector=self._tf_end_effector.data.target_pos_source[:, 0, :],
            fk_quat_end_effector=self._tf_end_effector.data.target_quat_source[:, 0, :],
            # Transforms (world frame)
            tf_pos_end_effector=self._tf_end_effector.data.target_pos_w[:, 0, :],
            tf_quat_end_effector=self._tf_end_effector.data.target_quat_w[:, 0, :],
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            # Particles
            particles_pos=self._cached_particles_pos,
            particles_vel=self._cached_particles_vel,
            particles_initial_mean_pos=self._initial_particle_mean_pos,
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
    joint_acc_robot: torch.Tensor,
    joint_applied_torque_robot: torch.Tensor,
    # Kinematics
    fk_pos_end_effector: torch.Tensor,
    fk_quat_end_effector: torch.Tensor,
    # Transforms (world frame)
    tf_pos_end_effector: torch.Tensor,
    tf_quat_end_effector: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    # Particles
    particles_pos: torch.Tensor,
    particles_vel: torch.Tensor,
    particles_initial_mean_pos: torch.Tensor,
) -> StepReturn:
    num_envs = episode_length.size(0)
    num_particles = particles_pos.size(1)
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
    # End-effector -> Initial particles
    tf_pos_end_effector_to_initial_particles = (
        tf_pos_end_effector - particles_initial_mean_pos
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

    ## Particles
    particles_vel_norm = torch.linalg.norm(particles_vel, dim=-1)

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    # Penalty: Joint torque
    WEIGHT_JOINT_TORQUE = -0.000025
    MAX_JOINT_TORQUE_PENALTY = -4.0
    penalty_joint_torque = torch.clamp_min(
        WEIGHT_JOINT_TORQUE
        * torch.sum(torch.square(joint_applied_torque_robot), dim=1),
        min=MAX_JOINT_TORQUE_PENALTY,
    )

    # Penalty: Joint acceleration
    WEIGHT_JOINT_ACCELERATION = -0.0005
    MAX_JOINT_ACCELERATION_PENALTY = -4.0
    penalty_joint_acceleration = torch.clamp_min(
        WEIGHT_JOINT_ACCELERATION * torch.sum(torch.square(joint_acc_robot), dim=1),
        min=MAX_JOINT_ACCELERATION_PENALTY,
    )

    # Penalty: Undesired robot contacts
    WEIGHT_UNDESIRED_ROBOT_CONTACTS = -1.0
    THRESHOLD_UNDESIRED_ROBOT_CONTACTS = 10.0
    penalty_undesired_robot_contacts = WEIGHT_UNDESIRED_ROBOT_CONTACTS * (
        torch.max(torch.norm(contact_forces_robot, dim=-1), dim=1)[0]
        > THRESHOLD_UNDESIRED_ROBOT_CONTACTS
    )

    # Reward: End-effector top-down orientation
    WEIGHT_TOP_DOWN_ORIENTATION = 1.0
    TANH_STD_TOP_DOWN_ORIENTATION = 0.15
    top_down_alignment = torch.sum(
        fk_rotmat_end_effector[:, :, 2]
        * torch.tensor((0.0, 0.0, -1.0), device=device)
        .unsqueeze(0)
        .expand(num_envs, 3),
        dim=1,
    )
    reward_top_down_orientation = WEIGHT_TOP_DOWN_ORIENTATION * (
        1.0 - torch.tanh((1.0 - top_down_alignment) / TANH_STD_TOP_DOWN_ORIENTATION)
    )

    # Reward: Distance end-effector to particles
    WEIGHT_DISTANCE_END_EFFECTOR_TO_INITIAL_PARTICLES = 16.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_INITIAL_PARTICLES = 0.2
    reward_distance_end_effector_to_initial_particles = (
        WEIGHT_DISTANCE_END_EFFECTOR_TO_INITIAL_PARTICLES
        * (
            1.0
            - torch.tanh(
                torch.norm(tf_pos_end_effector_to_initial_particles, dim=-1)
                / TANH_STD_DISTANCE_END_EFFECTOR_TO_INITIAL_PARTICLES
            )
        )
    )

    # Penalty: Particle velocity
    WEIGHT_SPLASHING_PENALTY = -8192.0
    MAX_SPLASHING_PENALTY = -32768.0
    penalty_particle_velocity = torch.clamp_min(
        WEIGHT_SPLASHING_PENALTY
        * (torch.quantile(torch.square(particles_vel_norm), q=0.8, dim=1)),
        min=MAX_SPLASHING_PENALTY,
    )

    # Reward: Number of lifted particles
    WEIGHT_PARTICLE_LIFT = 16384.0
    HEIGHT_OFFSET_PARTICLE_LIFT = 0.5
    HEIGHT_SPAN_PARTICLE_LIFT = 0.35
    TANH_STD_HEIGHT_PARTICLE_LIFT = 0.025
    reward_particle_lift = (
        WEIGHT_PARTICLE_LIFT
        * torch.sum(
            1.0
            - torch.tanh(
                (
                    torch.abs(
                        particles_pos[:, :, 2]
                        - particles_initial_mean_pos[:, 2].unsqueeze(1)
                        - HEIGHT_OFFSET_PARTICLE_LIFT
                    )
                    - HEIGHT_SPAN_PARTICLE_LIFT
                ).clamp(min=0.0)
                / TANH_STD_HEIGHT_PARTICLE_LIFT
            ),
            dim=1,
        )
        / num_particles
    )

    # Reward: Stabilization of excavated particles
    WEIGHT_STABILIZATION_REWARD = 65536.0
    TANH_STD_STABILIZATION_VELOCITY = 0.01
    reward_particle_stabilization = (
        WEIGHT_STABILIZATION_REWARD
        * torch.sum(
            (
                torch.abs(
                    particles_pos[:, :, 2]
                    - particles_initial_mean_pos[:, 2].unsqueeze(1)
                    - HEIGHT_OFFSET_PARTICLE_LIFT
                )
                < HEIGHT_SPAN_PARTICLE_LIFT
            ).float()
            * (1.0 - torch.tanh(particles_vel_norm / TANH_STD_STABILIZATION_VELOCITY)),
            dim=1,
        )
        / num_particles
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
                "tf_pos_end_effector_to_initial_particles": tf_pos_end_effector_to_initial_particles,
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
                "joint_acc_robot": joint_acc_robot,
                "joint_applied_torque_robot": joint_applied_torque_robot,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_joint_torque": penalty_joint_torque,
            "penalty_joint_acceleration": penalty_joint_acceleration,
            "penalty_undesired_robot_contacts": penalty_undesired_robot_contacts,
            "reward_top_down_orientation": reward_top_down_orientation,
            "reward_distance_end_effector_to_initial_particles": reward_distance_end_effector_to_initial_particles,
            "penalty_particle_velocity": penalty_particle_velocity,
            "reward_particle_lift": reward_particle_lift,
            "reward_particle_stabilization": reward_particle_stabilization,
        },
        termination,
        truncation,
    )
