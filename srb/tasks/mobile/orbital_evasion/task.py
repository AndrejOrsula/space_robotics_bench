from dataclasses import MISSING

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import RigidObjectCollection, RigidObjectCollectionCfg
from srb.core.env import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
    ViewerCfg,
)
from srb.core.env.mobile import OrbitalMobileRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import reset_collection_root_state_uniform_poisson_disk_3d
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    HyperbolicBarrierRewardTerm,
    HyperbolicRewardTerm,
    LinearRewardTerm,
)
from srb.core.scene_introspection import get_scene_rigid_object_collection
from srb.core.sim import PreviewSurfaceCfg, SphereCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, radians, rotmat_to_rot6d

from .asset import select_obstacle

##############
### Config ###
##############

_OBSTACLE_SCALE_REL_STD = 0.2
"""Per-axis standard deviation as a fraction of nominal asteroid size."""


def obstacle_bounding_radius(obstacle_scale: float) -> float:
    """Return asteroid half-edge plus two size standard deviations, in metres."""
    return 0.5 * obstacle_scale * (1.0 + 2.0 * _OBSTACLE_SCALE_REL_STD)


class RewardCfg(OrbitalMobileRewardCfg):
    """Balance obstacle clearance and target approach; inherit action and fuel
    penalties.
    """

    penalty_angular_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=-0.25, limit=5.0
    )
    """Penalise angular velocity during evasive manoeuvres."""

    reward_obstacle_clearance: HyperbolicBarrierRewardTerm = (
        HyperbolicBarrierRewardTerm(weight=4.0, scale=2.0, threshold=3.0, direction="<")
    )
    """Reward clearance from the nearest obstacle's bounding sphere."""

    penalty_distance_robot_to_target: LinearRewardTerm = LinearRewardTerm(
        weight=-16.0 / 50.0, limit=32.0
    )
    """Penalise distance to the target while encouraging approach."""

    penalty_out_of_bounds: LinearRewardTerm = LinearRewardTerm(weight=-16.0, limit=16.0)
    """Penalise exceeding ``max_target_distance`` without terminating."""

    reward_target_approach: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=8.0, scale=5.0
    )
    """Reward proximity to the target with hyperbolic shaping."""


@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing: float = 25.0

    ## Assets
    objs: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore[arg-type]
    )


@configclass
class EventCfg(OrbitalEventCfg):
    randomize_object_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_collection_root_state_uniform_poisson_disk_3d,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("objs"),
            # Obstacle centres stay at least about 7 m from spawn and target.
            "pose_range": {
                "x": (-10.0, 10.0),
                "y": (-10.0, 10.0),
                "z": (-43.0, -7.0),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (-0.1, 0.1),
                "roll": (-radians(10.0), radians(10.0)),
                "pitch": (-radians(10.0), radians(10.0)),
                "yaw": (-radians(10.0), radians(10.0)),
            },
            "radius": 7.0,
        },
    )


@configclass
class TaskCfg(OrbitalEnvCfg):
    """Configuration for target navigation among orbital debris."""

    ## Scene
    scene: SceneCfg = SceneCfg()
    num_obstacles: int = 16
    obstacle_scale: float = 5.0
    """Nominal edge length of each asteroid (m); sets the spawned size and the
    collision keep-out radius (:func:`obstacle_bounding_radius`)."""

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 30.0
    is_finite_horizon: bool = False

    ## Termination threshold
    termination_min_obstacle_distance: float = 0.0
    """Terminate when COM clearance falls below this value; zero means entry."""

    ## Penalty threshold
    max_target_distance: float = 100.0
    """Distance threshold for the non-terminal out-of-bounds penalty."""

    ## Sensors
    camera_clipping_range: tuple[float, float] | None = (0.01, 150.0)
    """Onboard camera depth range in metres."""

    viewer: ViewerCfg = ViewerCfg(
        eye=(3.0, -3.0, 1.5),
        lookat=(0.0, 0.0, -0.5),
        origin_type="env",
    )

    ## Target
    tf_pos_target: tuple[float, float, float] = (0.0, 0.0, -50.0)
    tf_quat_target: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": SphereCfg(
                radius=0.25,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        super().__post_init__()

        if type(self.num_obstacles) is not int or self.num_obstacles < 1:
            raise ValueError(
                f"num_obstacles must be an integer >= 1, got {self.num_obstacles!r}"
            )

        # Scene: Asteroids
        self.scene.objs.rigid_objects = {
            f"obstacle{i}": select_obstacle(
                self,
                prim_path=f"{{ENV_REGEX_NS}}/obstacle{i}",
                seed=self.seed + (i * self.scene.num_envs),
                scale=(self.obstacle_scale,) * 3,
                activate_contact_sensors=True,
            )
            for i in range(self.num_obstacles)
        }

        self._update_procedural_assets()


############
### Task ###
############


class Task(SrbEnv, OrbitalEnv):
    """Navigate to a target while avoiding debris and conserving fuel."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._objs: RigidObjectCollection = get_scene_rigid_object_collection(
            self.scene, "objs"
        )
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )
        self._obstacle_bounding_radius = obstacle_bounding_radius(
            self.cfg.obstacle_scale
        )

        ## Initialize buffers
        self._tf_pos_target = self.scene.env_origins + torch.tensor(
            self.cfg.tf_pos_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

        ## Visualize target
        self._target_marker.visualize(self._tf_pos_target, self._tf_quat_target)

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward
        num_envs = self.num_envs
        num_objs = self._objs.data.object_com_pos_w.size(1)

        thrust_action_term = self._thrust_action_term
        if thrust_action_term is not None:
            remaining_fuel = (
                thrust_action_term.remaining_fuel / thrust_action_term.cfg.fuel_capacity
            ).unsqueeze(-1)
        else:
            remaining_fuel = torch.ones(
                (num_envs, 1), dtype=torch.float32, device=self.device
            )

        ############
        ## States ##
        ############
        tf_pos_robot, tf_quat_robot, vel_lin_robot, vel_ang_robot = (
            self._robot_base_state()
        )
        tf_pos_objs = self._objs.data.object_com_pos_w

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        # Robot -> Objects (vector from robot to each object, consistent with
        # the tf_pos_robot_to_target convention used below)
        pos_robot_to_objs = tf_pos_objs - tf_pos_robot.unsqueeze(1).repeat(
            1, num_objs, 1
        )
        tf_pos_robot_to_nearest_obj = pos_robot_to_objs[
            torch.arange(pos_robot_to_objs.size(0), device=self.device),
            torch.argmin(torch.norm(pos_robot_to_objs, dim=-1), dim=1),
        ]

        # Robot -> Target
        tf_pos_robot_to_target = self._tf_pos_target - tf_pos_robot

        # IMU
        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {}

        _action_rate = torch.mean(
            torch.square(self.action_manager.action - self.action_manager.prev_action),
            dim=1,
        )
        reward["penalty_action_rate"] = rew.penalty_action_rate(_action_rate)

        _fuel_used = torch.square(1.0 - remaining_fuel.squeeze(-1))
        reward["penalty_fuel_consumption"] = rew.penalty_fuel_consumption(_fuel_used)

        _ang_vel_sq = torch.sum(torch.square(vel_ang_robot), dim=1)
        reward["penalty_angular_velocity"] = rew.penalty_angular_velocity(_ang_vel_sq)

        # Clearance is measured from the nearest obstacle's bounding sphere;
        # every obstacle shares one radius, so the COM-nearest is also the
        # clearance-nearest.
        _clearance_nearest_obj = (
            torch.norm(tf_pos_robot_to_nearest_obj, dim=-1)
            - self._obstacle_bounding_radius
        )
        reward["reward_obstacle_clearance"] = rew.reward_obstacle_clearance(
            _clearance_nearest_obj
        )

        _dist_to_target = torch.norm(tf_pos_robot_to_target, dim=-1)
        reward["penalty_distance_robot_to_target"] = (
            rew.penalty_distance_robot_to_target(_dist_to_target)
        )

        # Out-of-bounds distance is penalized without ending the episode.
        out_of_bounds_metric = (_dist_to_target > self.cfg.max_target_distance).float()
        reward["penalty_out_of_bounds"] = rew.penalty_out_of_bounds(
            out_of_bounds_metric
        )

        reward["reward_target_approach"] = rew.reward_target_approach(_dist_to_target)

        termination = (
            _clearance_nearest_obj < self.cfg.termination_min_obstacle_distance
        )
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=self.device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                    "tf_pos_robot_to_target": tf_pos_robot_to_target,
                    "tf_pos_robot_to_objs": pos_robot_to_objs.flatten(1),
                },
                ObservationGroup.PROPRIO: {
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                    "remaining_fuel": remaining_fuel,
                },
            },
            reward,
            termination,
            truncation,
        )
