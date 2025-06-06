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
    GroundManipulator,
)
from srb.core.env import (
    GroundManipulationEnvCfg,
    ManipulationEnv,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.sensor import ContactSensor
from srb.core.sim import PyramidParticlesSpawnerCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d, scale_transform


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
class TaskCfg(GroundManipulationEnvCfg):
    ## Assets
    robot: GroundManipulator | AssetVariant = assets.GenericGroundManipulator(
        mobile_base=assets.Spot(),
        manipulator=assets.Franka(end_effector=assets.ScoopRectangular()),
    )
    robot.asset_cfg.init_state.pos = (0.0, 0.0, 1.0)  # type: ignore

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    episode_length_s: float = 20.0
    is_finite_horizon: bool = True

    ## Particles
    scatter_particles: bool = False
    particles_ratio: float = 0.25
    particles_size: float = 0.02
    particles_settle_max_steps: int = 50
    particles_settle_step_time: float = 1.0
    particles_settle_extra_time: float = 10.0
    particles_settle_vel_threshold: float = 0.01

    def __post_init__(self):
        super().__post_init__()

        # Scene: Regolith
        assert self.spacing is not None
        _regolith_dim = round(self.spacing / self.particles_size)
        self.scene.regolith.spawn.ratio = self.particles_ratio  # type: ignore
        self.scene.regolith.spawn.particle_size = self.particles_size  # type: ignore
        self.scene.regolith.spawn.dim_x = _regolith_dim  # type: ignore
        self.scene.regolith.spawn.dim_y = _regolith_dim  # type: ignore
        self.scene.regolith.spawn.dim_z = round(0.2 * _regolith_dim)  # type: ignore
        self.scene.regolith.init_state.pos = (
            0.0,
            0.0,
            0.05 * self.spacing,
        )


############
### Task ###
############


class Task(ManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._regolith: AssetBase = self.scene["regolith"]

        ## Initialize buffers
        self._initial_particle_positions: torch.Tensor | None = None
        self._initial_particle_combined_height: torch.Tensor = torch.zeros(
            self.num_envs, 1, dtype=torch.float32, device=self.device
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        ## Let the particles settle on the first reset, then remember their positions for future resets
        if self._initial_particle_positions is None:
            for _ in range(self.cfg.particles_settle_max_steps):
                for _ in range(
                    round(self.cfg.particles_settle_step_time / self.step_dt)
                ):
                    self.sim.step(render=False)
                if (
                    torch.median(
                        torch.linalg.norm(
                            particle_utils.get_particles_vel_w(self, self._regolith),
                            dim=-1,
                        )
                    )
                    < self.cfg.particles_settle_vel_threshold
                ):
                    for _ in range(
                        round(self.cfg.particles_settle_extra_time / self.step_dt)
                    ):
                        self.sim.step(render=False)
                    break

            # Extract statistics about the initial state of the particles
            self._initial_particle_positions = particle_utils.get_particles_pos_w(
                self, self._regolith
            )
            self._initial_particle_velocities = torch.zeros_like(
                self._initial_particle_positions
            )
            self._initial_particle_combined_height = torch.quantile(
                self._initial_particle_positions[:, :, 2], q=0.95, dim=1
            ).unsqueeze(1)

        else:
            particle_utils.set_particles_pos_w(
                self, self._regolith, self._initial_particle_positions, env_ids=env_ids
            )
            particle_utils.set_particles_vel_w(
                self, self._regolith, self._initial_particle_velocities, env_ids=env_ids
            )

        super()._reset_idx(env_ids)

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
            # Root
            vel_lin_robot=self._robot.data.root_lin_vel_b,
            vel_ang_robot=self._robot.data.root_ang_vel_b,
            # Joints
            joint_pos_robot=self._manipulator.data.joint_pos,
            joint_pos_limits_robot=(
                self._manipulator.data.soft_joint_pos_limits
                if torch.all(
                    torch.isfinite(self._manipulator.data.soft_joint_pos_limits)
                )
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
            joint_acc_robot=self._manipulator.data.joint_acc,
            joint_applied_torque_robot=self._manipulator.data.applied_torque,
            # Kinematics
            fk_pos_end_effector=self._tf_end_effector.data.target_pos_source[:, 0, :],
            fk_quat_end_effector=self._tf_end_effector.data.target_quat_source[:, 0, :],
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            # IMU
            imu_lin_acc=self._imu_robot.data.lin_acc_b,
            imu_ang_vel=self._imu_robot.data.ang_vel_b,
            # Particles
            particles_pos=particle_utils.get_particles_pos_w(self, self._regolith),
            particles_vel=particle_utils.get_particles_vel_w(self, self._regolith),
            particles_initial_combined_height=self._initial_particle_combined_height,
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
    # Root
    vel_lin_robot: torch.Tensor,
    vel_ang_robot: torch.Tensor,
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
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    # IMU
    imu_lin_acc: torch.Tensor,
    imu_ang_vel: torch.Tensor,
    # Particles
    particles_pos: torch.Tensor,
    particles_vel: torch.Tensor,
    particles_initial_combined_height: torch.Tensor,
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

    # Penalty: Angular velocity
    WEIGHT_ANGULAR_VELOCITY = -0.25
    MAX_ANGULAR_VELOCITY_PENALTY = -5.0
    penalty_angular_velocity = torch.clamp_min(
        WEIGHT_ANGULAR_VELOCITY * torch.sum(torch.square(vel_ang_robot[:, :2]), dim=1),
        min=MAX_ANGULAR_VELOCITY_PENALTY,
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

    # Penalty: Particle velocity
    WEIGHT_SPLASHING_PENALTY = -512.0
    penalty_particle_velocity = WEIGHT_SPLASHING_PENALTY * (
        torch.sum(torch.square(particles_vel_norm), dim=1) / num_particles
    )

    # Reward: Number of lifted particles
    WEIGHT_PARTICLE_LIFT = 2048.0
    HEIGHT_OFFSET_PARTICLE_LIFT = 0.2
    HEIGHT_SPAN_PARTICLE_LIFT = 0.1
    TANH_STD_HEIGHT_PARTICLE_LIFT = 0.025
    reward_particle_lift = (
        WEIGHT_PARTICLE_LIFT
        * torch.sum(
            1.0
            - torch.tanh(
                (
                    torch.abs(
                        particles_pos[:, :, 2]
                        - particles_initial_combined_height
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
    WEIGHT_STABILIZATION_REWARD = 8192.0
    THRESHOLD_STABILIZATION_POSITION = 0.05
    TANH_STD_STABILIZATION_VELOCITY = 0.025
    reward_particle_stabilization = (
        WEIGHT_STABILIZATION_REWARD
        * torch.sum(
            (
                torch.abs(
                    particles_pos[:, :, 2]
                    - particles_initial_combined_height
                    - HEIGHT_OFFSET_PARTICLE_LIFT
                )
                < THRESHOLD_STABILIZATION_POSITION
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
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
            },
            "state_dyn": {
                "contact_forces_robot": contact_forces_robot,
                "contact_forces_end_effector": contact_forces_end_effector,
            },
            "proprio": {
                "fk_pos_end_effector": fk_pos_end_effector,
                "fk_rot6d_end_effector": fk_rot6d_end_effector,
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
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
            "penalty_angular_velocity": penalty_angular_velocity,
            "penalty_joint_torque": penalty_joint_torque,
            "penalty_joint_acceleration": penalty_joint_acceleration,
            "penalty_undesired_robot_contacts": penalty_undesired_robot_contacts,
            "penalty_particle_velocity": penalty_particle_velocity,
            "reward_particle_lift": reward_particle_lift,
            "reward_particle_stabilization": reward_particle_stabilization,
        },
        termination,
        truncation,
    )
