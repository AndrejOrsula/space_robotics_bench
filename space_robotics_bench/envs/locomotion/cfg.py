import math

import torch
from omni.isaac.lab.envs import ViewerCfg
from omni.isaac.lab.managers import EventTermCfg, SceneEntityCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from omni.isaac.lab.utils import configclass

import space_robotics_bench.core.sim as sim_utils
from space_robotics_bench import assets
from space_robotics_bench.core import mdp
from space_robotics_bench.core.envs import BaseEnvCfg
from space_robotics_bench.core.sim import SimulationCfg


@configclass
class BaseLocomotionEnvEventCfg:
    ## Default scene reset
    reset_all = EventTermCfg(func=mdp.reset_scene_to_default, mode="reset")

    ## Light
    reset_rand_light_rot = EventTermCfg(
        func=mdp.reset_xform_orientation_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("light"),
            "orientation_distribution_params": {
                "roll": (
                    -75.0 * torch.pi / 180.0,
                    75.0 * torch.pi / 180.0,
                ),
                "pitch": (
                    0.0,
                    75.0 * torch.pi / 180.0,
                ),
            },
        },
    )

    ## Robot
    reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )
    reset_robot_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )
    push_robot = EventTermCfg(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)},
        },
    )


@configclass
class BaseLocomotionEnvCfg(BaseEnvCfg):
    ## Environment
    episode_length_s: float = 30.0
    env_rate: float = 1.0 / 200.0

    ## Agent
    agent_rate: float = 1.0 / 50.0

    ## Simulation
    sim = SimulationCfg(
        disable_contact_processing=True,
        physx=sim_utils.PhysxCfg(
            enable_ccd=True,
            enable_stabilization=True,
            bounce_threshold_velocity=0.0,
            friction_correlation_distance=0.005,
            min_velocity_iteration_count=2,
            # GPU settings
            gpu_temp_buffer_capacity=2 ** (24 - 2),
            gpu_max_rigid_contact_count=2 ** (22 - 1),
            gpu_max_rigid_patch_count=2 ** (13 - 0),
            gpu_heap_capacity=2 ** (26 - 3),
            gpu_found_lost_pairs_capacity=2 ** (18 - 1),
            gpu_found_lost_aggregate_pairs_capacity=2 ** (10 - 0 + 1),
            gpu_total_aggregate_pairs_capacity=2 ** (10 - 0 + 1),
            gpu_max_soft_body_contacts=2 ** (20 - 1),
            gpu_max_particle_contacts=2 ** (20 - 1),
            gpu_collision_stack_size=2 ** (26 - 3),
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
        lookat=(0.0, 0.0, 0.25),
        eye=(2.0, 0.0, 1.75),
        origin_type="env",
        env_index=0,
    )

    ## Scene
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=0.0, replicate_physics=False)

    ## Events
    events = BaseLocomotionEnvEventCfg()

    def __post_init__(self):
        super().__post_init__()

        ## Simulation
        self.decimation = int(self.agent_rate / self.env_rate)
        self.sim.dt = self.env_rate
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -self.env_cfg.scenario.gravity_magnitude)
        # Increase GPU settings based on the number of environments
        gpu_capacity_factor = math.pow(self.scene.num_envs, 0.25)
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
            2 ** math.floor(1.0 + gpu_capacity_factor), 32
        )

        ## Scene
        self.scene.light = assets.sunlight_from_env_cfg(self.env_cfg)
        self.scene.sky = assets.sky_from_env_cfg(self.env_cfg)
        self.robot_cfg = assets.legged_robot_from_env_cfg(self.env_cfg)
        self.scene.robot = self.robot_cfg.asset_cfg
        self.scene.terrain = assets.terrain_from_env_cfg(
            self.env_cfg,
            num_assets=1,
            size=(48.0,) * 2,
            prim_path="/World/terrain",
            procgen_kwargs={
                "density": 0.1,
                "flat_area_size": 2.0,
                "texture_resolution": 4096,
            },
        )

        ## Actions
        self.actions = self.robot_cfg.action_cfg

        ## Sensors
        self.scene.contacts_robot = ContactSensorCfg(
            prim_path=f"{self.scene.robot.prim_path}/.*",
            update_period=0.0,
            history_length=3,
            track_air_time=True,
        )
        self.scene.height_scanner = RayCasterCfg(
            prim_path=f"{self.scene.robot.prim_path}/base",
            offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 10.0)),
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.5, 1.0]),
            debug_vis=False,
            mesh_prim_paths=[self.scene.terrain.prim_path],
        )
