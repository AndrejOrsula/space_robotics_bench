import math

import torch
from omni.isaac.lab.envs import ViewerCfg
from omni.isaac.lab.managers import EventTermCfg, SceneEntityCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.utils import configclass

import space_robotics_bench.core.sim as sim_utils
from space_robotics_bench import assets
from space_robotics_bench.core import mdp
from space_robotics_bench.core.envs import BaseEnvCfg
from space_robotics_bench.core.sim import SimulationCfg


@configclass
class BaseSpacecraftRoboticsEnvEventCfg:
    ## Default scene reset
    reset_all = EventTermCfg(func=mdp.reset_scene_to_default, mode="reset")

    ## Light
    reset_rand_light_rot = EventTermCfg(
        func=mdp.reset_xform_orientation_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("light"),
            "orientation_distribution_params": {
                "roll": (-20.0 * torch.pi / 180.0,) * 2,
                "pitch": (50.0 * torch.pi / 180.0,) * 2,
            },
        },
    )

    ## Robot
    reset_rand_robot_state = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-0.2, 0.2),
                "y": (-0.2, 0.2),
                "z": (-0.2, 0.2),
                "roll": (
                    -torch.pi,
                    torch.pi,
                ),
                "pitch": (
                    -torch.pi,
                    torch.pi,
                ),
                "yaw": (
                    -torch.pi,
                    torch.pi,
                ),
            },
            "velocity_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (-0.1, 0.1),
                "roll": (-0.7854, 0.7854),
                "pitch": (-0.7854, 0.7854),
                "yaw": (-0.7854, 0.7854),
            },
        },
    )


@configclass
class BaseSpacecraftRoboticsEnvCfg(BaseEnvCfg):
    ## Environment
    episode_length_s: float = 50.0
    env_rate: float = 1.0 / 50.0

    ## Agent
    agent_rate: float = 1.0 / 50.0

    ## Simulation
    sim = SimulationCfg(
        disable_contact_processing=True,
        physx=sim_utils.PhysxCfg(
            enable_ccd=False,
            enable_stabilization=False,
            bounce_threshold_velocity=0.0,
            friction_correlation_distance=0.02,
            min_velocity_iteration_count=1,
            # GPU settings
            gpu_temp_buffer_capacity=2 ** (24 - 5),
            gpu_max_rigid_contact_count=2 ** (22 - 4),
            gpu_max_rigid_patch_count=2 ** (13 - 1),
            gpu_heap_capacity=2 ** (26 - 6),
            gpu_found_lost_pairs_capacity=2 ** (18 - 2),
            gpu_found_lost_aggregate_pairs_capacity=2 ** (10 - 1),
            gpu_total_aggregate_pairs_capacity=2 ** (10 - 1),
            gpu_max_soft_body_contacts=2 ** (20 - 3),
            gpu_max_particle_contacts=2 ** (20 - 3),
            gpu_collision_stack_size=2 ** (26 - 4),
            gpu_max_num_partitions=8,
        ),
        render=sim_utils.RenderCfg(
            enable_reflections=True,
        ),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
        ),
    )

    ## Viewer
    viewer = ViewerCfg(
        lookat=(0.0, 0.0, 0.0),
        eye=(-2.0, 2.0, 2.0),
        origin_type="env",
        env_index=0,
    )

    ## Scene
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=1.5, replicate_physics=False)

    ## Events
    events = BaseSpacecraftRoboticsEnvEventCfg()

    def __post_init__(self):
        super().__post_init__()

        ## Simulation
        self.decimation = int(self.agent_rate / self.env_rate)
        self.sim.dt = self.env_rate
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -self.env_cfg.scenario.gravity_magnitude)
        # Increase GPU settings based on the number of environments
        gpu_capacity_factor = self.scene.num_envs
        self.sim.physx.gpu_heap_capacity *= gpu_capacity_factor
        self.sim.physx.gpu_collision_stack_size *= gpu_capacity_factor
        self.sim.physx.gpu_temp_buffer_capacity *= gpu_capacity_factor
        self.sim.physx.gpu_max_rigid_contact_count *= gpu_capacity_factor
        self.sim.physx.gpu_max_rigid_patch_count *= gpu_capacity_factor
        self.sim.physx.gpu_found_lost_pairs_capacity *= gpu_capacity_factor
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity *= gpu_capacity_factor
        self.sim.physx.gpu_total_aggregate_pairs_capacity *= gpu_capacity_factor
        self.sim.physx.gpu_max_soft_body_contacts *= gpu_capacity_factor
        self.sim.physx.gpu_max_particle_contacts *= gpu_capacity_factor
        self.sim.physx.gpu_max_num_partitions = min(
            2 ** math.floor(1.0 + math.pow(self.scene.num_envs, 0.2)), 32
        )

        ## Scene
        self.scene.light = assets.sunlight_from_env_cfg(self.env_cfg)
        self.scene.sky = assets.sky_from_env_cfg(self.env_cfg)
        self.robot_cfg = assets.spacecraft_robot_from_env_cfg(self.env_cfg)
        self.scene.robot = self.robot_cfg.asset_cfg

        ## Actions
        self.actions = self.robot_cfg.action_cfg
