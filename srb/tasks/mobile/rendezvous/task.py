import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import RigidObject, RigidObjectCfg
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
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import GaussianRewardTerm, LinearRewardTerm
from srb.core.scene_introspection import get_scene_rigid_object
from srb.core.sim import ArrowCfg, PreviewSurfaceCfg, SimforgeAssetCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    radians,
    rotmat_to_rot6d,
    rpy_to_quat,
    subtract_frame_transforms,
)


class RewardCfg(OrbitalMobileRewardCfg):
    """Docking precision rewards and penalties."""

    penalty_fuel_consumption: LinearRewardTerm = LinearRewardTerm(
        weight=-8.0, limit=8.0
    )
    """Penalize fuel use during approach."""

    penalty_angular_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=-0.05, limit=5.0
    )
    """Penalize angular velocity."""

    penalty_distance_robot_to_target: LinearRewardTerm = LinearRewardTerm(
        weight=-16.0, limit=128.0
    )
    """Penalize distance to the target."""

    penalty_too_far: LinearRewardTerm = LinearRewardTerm(weight=-16.0, limit=16.0)
    """Penalize exceeding the target-distance limit without termination."""

    reward_precision_position: GaussianRewardTerm = GaussianRewardTerm(
        weight=128.0, scale=0.025
    )
    """Reward positional alignment with the target."""

    reward_precision_orientation: LinearRewardTerm = LinearRewardTerm(
        weight=128.0, limit=128.0
    )
    """Reward orientation alignment near the target."""

    # Shaping parameter, not a reward term.
    tanh_std_precision_orientation: float = 0.05
    """Orientation-error shaping width."""


@configclass
class SceneCfg(OrbitalSceneCfg):
    target: RigidObjectCfg = assets.Cubesat().asset_cfg
    target.prim_path = "{ENV_REGEX_NS}/target"


@configclass
class EventCfg(OrbitalEventCfg):
    randomize_target_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("target"),
            "pose_range": {
                "x": (0.5, 2.0),
                "y": (-2.0, 2.0),
                "z": (-2.0, 2.0),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.05),
                "roll": (-radians(10.0), radians(10.0)),
                "pitch": (-radians(10.0), radians(10.0)),
                "yaw": (-radians(10.0), radians(10.0)),
            },
        },
    )


@configclass
class TaskCfg(OrbitalEnvCfg):
    """Configuration for orbital rendezvous and docking."""

    scene: SceneCfg = SceneCfg()

    events: EventCfg = EventCfg()

    reward: RewardCfg = RewardCfg()

    episode_length_s: float = 25.0
    is_finite_horizon: bool = False

    # Frame the nearby target and chaser from the sunlit side.
    viewer: ViewerCfg = ViewerCfg(
        eye=(-3.5, 4.5, 2.0), lookat=(1.0, 0.0, 0.0), origin_type="env"
    )

    max_target_distance: float = 50.0
    """Distance threshold for the non-terminal too-far penalty."""

    camera_clipping_range: tuple[float, float] | None = (0.01, 50.0)
    """Onboard camera depth range in metres."""

    target_offset_pos: tuple[float, float, float] = (0.0, 0.0, 0.5)
    target_offset_quat: tuple[float, float, float, float] = rpy_to_quat(0.0, 90.0, 0.0)
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": ArrowCfg(
                tail_radius=0.01,
                tail_length=0.05,
                head_radius=0.02,
                head_length=0.025,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        super().__post_init__()

        # Scene: Target
        if isinstance(self.scene.target.spawn, SimforgeAssetCfg):
            self.scene.target.spawn.seed = self.seed + self.scene.num_envs


class Task(SrbEnv, OrbitalEnv):
    """Approach a target and align for orbital docking."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._target: RigidObject = get_scene_rigid_object(self.scene, "target")
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        self._target_offset_pos = torch.tensor(
            self.cfg.target_offset_pos, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)
        self._target_offset_quat = torch.tensor(
            self.cfg.target_offset_quat, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward
        num_envs = self.num_envs

        tf_pos_target, tf_quat_target = combine_frame_transforms(
            self._target.data.root_pos_w,
            self._target.data.root_quat_w,
            self._target_offset_pos,
            self._target_offset_quat,
        )

        self._target_marker.visualize(tf_pos_target, tf_quat_target)

        thrust_action_term = self._thrust_action_term
        if thrust_action_term is not None:
            remaining_fuel = (
                thrust_action_term.remaining_fuel / thrust_action_term.cfg.fuel_capacity
            ).unsqueeze(-1)
        else:
            remaining_fuel = torch.ones(
                (num_envs, 1), dtype=torch.float32, device=self.device
            )

        tf_pos_robot, tf_quat_robot, vel_lin_robot, vel_ang_robot = (
            self._robot_base_state()
        )
        vel_lin_target = self._target.data.root_lin_vel_b
        vel_ang_target = self._target.data.root_ang_vel_b

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        tf_pos_robot_to_target, tf_quat_robot_to_target = subtract_frame_transforms(
            t01=tf_pos_robot,
            q01=tf_quat_robot,
            t02=tf_pos_target,
            q02=tf_quat_target,
        )
        tf_rotmat_robot_to_target = matrix_from_quat(tf_quat_robot_to_target)
        tf_rot6d_robot_to_target = rotmat_to_rot6d(tf_rotmat_robot_to_target)

        distance_robot_to_target = torch.norm(tf_pos_robot_to_target, dim=-1)

        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        reward: dict[str, torch.Tensor] = {}

        # Penalty: Action rate
        _action_rate = torch.mean(
            torch.square(self.action_manager.action - self.action_manager.prev_action),
            dim=1,
        )
        reward["penalty_action_rate"] = rew.penalty_action_rate(_action_rate)

        # Penalty: Fuel consumption
        _fuel_used = torch.square(1.0 - remaining_fuel.squeeze(-1))
        reward["penalty_fuel_consumption"] = rew.penalty_fuel_consumption(_fuel_used)

        # Penalty: Angular velocity
        _ang_vel_sq = torch.sum(torch.square(vel_ang_robot), dim=1)
        reward["penalty_angular_velocity"] = rew.penalty_angular_velocity(_ang_vel_sq)

        # Penalty: Distance to target (squared)
        _dist_sq = torch.square(distance_robot_to_target)
        # Penalty: Distance | Robot <--> Target
        reward["penalty_distance_robot_to_target"] = (
            rew.penalty_distance_robot_to_target(_dist_sq)
        )

        # Penalty: Too far from target (replaces hard termination)
        too_far_metric = (
            distance_robot_to_target > self.cfg.max_target_distance
        ).float()
        reward["penalty_too_far"] = rew.penalty_too_far(too_far_metric)

        # Reward: Precision position (Gaussian shaping on distance)
        precision_pos_metric = distance_robot_to_target
        reward["reward_precision_position"] = rew.reward_precision_position(
            precision_pos_metric
        )

        # Reward: Precision orientation (gated by position proximity)
        _rot_error = torch.norm(
            torch.norm(
                tf_rotmat_robot_to_target
                - torch.eye(3, device=self.device).unsqueeze(0),
                dim=-1,
            ),
            dim=-1,
        )
        _position_precision = 1.0 - torch.tanh(
            distance_robot_to_target / rew.reward_precision_position.scale
        )
        precision_orient_metric = _position_precision * (
            1.0 - torch.tanh(_rot_error / rew.tanh_std_precision_orientation)
        )
        reward["reward_precision_orientation"] = rew.reward_precision_orientation(
            precision_orient_metric
        )

        # Distance is penalized without early termination.
        termination = torch.zeros(num_envs, dtype=torch.bool, device=self.device)
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
                    "vel_lin_target": vel_lin_target,
                    "vel_ang_target": vel_ang_target,
                    "tf_pos_robot_to_target": tf_pos_robot_to_target,
                    "tf_rot6d_robot_to_target": tf_rot6d_robot_to_target,
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
