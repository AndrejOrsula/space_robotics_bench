import math

import torch
from omni.isaac.lab.envs import ViewerCfg
from omni.isaac.lab.managers import EventTermCfg, SceneEntityCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import ContactSensorCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import (
    FrameTransformerCfg,
    OffsetCfg,
)
from omni.isaac.lab.utils import configclass

import space_robotics_bench.core.envs as env_utils
import space_robotics_bench.core.sim as sim_utils
from space_robotics_bench import assets
from space_robotics_bench.core import mdp
from space_robotics_bench.core.envs import BaseEnvCfg
from space_robotics_bench.core.markers import FRAME_MARKER_SMALL_CFG
from space_robotics_bench.core.sim import SimulationCfg


@configclass
class BaseManipulationEnvEventCfg:
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
    reset_rand_robot_state = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (-torch.pi / 32, torch.pi / 32),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class BaseManipulationEnvCfg(BaseEnvCfg):
    ## Environment
    episode_length_s: float = 50.0
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
            gpu_found_lost_aggregate_pairs_capacity=2 ** (10 - 0),
            gpu_total_aggregate_pairs_capacity=2 ** (10 - 0),
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
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=9.0, replicate_physics=False)

    ## Events
    events = BaseManipulationEnvEventCfg()

    def __post_init__(self):
        super().__post_init__()

        ## Simulation
        self.decimation = int(self.agent_rate / self.env_rate)
        self.sim.dt = self.env_rate
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -self.env_cfg.scenario.gravity_magnitude)
        # Increase GPU settings based on the number of environments
        gpu_capacity_factor = math.pow(self.scene.num_envs, 0.2)
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
        if self.env_cfg.scenario == env_utils.Scenario.ORBIT:
            self.scene.env_spacing = 42.0
        self.scene.light = assets.sunlight_from_env_cfg(self.env_cfg)
        self.scene.sky = assets.sky_from_env_cfg(self.env_cfg)
        self.robot_cfg = assets.manipulator_from_env_cfg(self.env_cfg)
        self.scene.robot = self.robot_cfg.asset_cfg
        self.vehicle_cfg = assets.vehicle_from_env_cfg(self.env_cfg)
        self.scene.terrain = assets.terrain_from_env_cfg(
            self.env_cfg,
            num_assets=self.scene.num_envs,
            size=(self.scene.env_spacing - 1,) * 2,
            procgen_kwargs={
                "density": 0.04,
                "flat_area_size": 2.0,
                "texture_resolution": 2048,
            },
        )
        if self.vehicle_cfg:
            # Add vehicle to scene
            self.scene.vehicle = self.vehicle_cfg.asset_cfg
            self.scene.vehicle.init_state.pos = (
                self.vehicle_cfg.frame_manipulator_base.offset.translation
            )

            # Update the robot based on the vehicle
            self.scene.robot.init_state.pos = (
                self.vehicle_cfg.frame_manipulator_base.offset.translation
            )
            self.scene.robot.init_state.rot = (
                self.vehicle_cfg.frame_manipulator_base.offset.rotation
            )

        ## Actions
        self.actions = self.robot_cfg.action_cfg

        ## Sensors
        self.scene.tf_robot_ee = FrameTransformerCfg(
            prim_path=f"{self.scene.robot.prim_path}/{self.robot_cfg.frame_base.prim_relpath}",
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    name="robot_ee",
                    prim_path=f"{self.scene.robot.prim_path}/{self.robot_cfg.frame_ee.prim_relpath}",
                    offset=OffsetCfg(
                        pos=self.robot_cfg.frame_ee.offset.translation,
                        rot=self.robot_cfg.frame_ee.offset.rotation,
                    ),
                ),
            ],
            visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(
                prim_path="/Visuals/robot_ee"
            ),
        )
        self.scene.contacts_robot = ContactSensorCfg(
            prim_path=f"{self.scene.robot.prim_path}/.*",
            update_period=0.0,
        )

        ## Events
        self.events.reset_rand_robot_state.params[
            "asset_cfg"
        ].joint_names = self.robot_cfg.regex_joints_arm
        if self.env_cfg.scenario == env_utils.Scenario.ORBIT:
            # Fix the orientation of the light such that it fits with the orbital HDR
            self.events.reset_rand_light_rot.params[
                "orientation_distribution_params"
            ] = {
                "roll": (-20.0 * torch.pi / 180.0,) * 2,
                "pitch": (50.0 * torch.pi / 180.0,) * 2,
            }
