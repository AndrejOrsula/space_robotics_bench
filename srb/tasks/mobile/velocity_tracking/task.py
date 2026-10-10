from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.action import FilteredWheeledDriveActionCfg, WheeledDriveActionCfg
from srb.core.env import GroundEnv, GroundEnvCfg, GroundEventCfg, GroundSceneCfg
from srb.core.env.mobile import GroundMobileRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.core.marker import ARROW_CFG, VisualizationMarkers
from srb.core.mdp import randomize_command
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import LinearRewardTerm
from srb.core.sim import PreviewSurfaceCfg
from srb.tasks.mobile.event_camera_wiring import build_front_event_camera_cfg
from srb.utils.cfg import configclass, iter_config_items_by_type
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d

##############
### Config ###
##############


class RewardCfg(GroundMobileRewardCfg):
    """Track planar and yaw velocity; penalize off-axis motion.

    Inherits ``penalty_action_rate`` from :class:`GroundMobileRewardCfg`.
    """

    ## Command tracking
    reward_cmd_lin_vel_xy: LinearRewardTerm = LinearRewardTerm(weight=4.0, limit=4.0)
    """Rewards tracking commanded planar linear velocity."""

    reward_cmd_ang_vel_z: LinearRewardTerm = LinearRewardTerm(weight=2.0, limit=2.0)
    """Rewards tracking commanded yaw angular velocity."""

    ## Non-command motion penalties
    penalty_undesired_lin_vel_z: LinearRewardTerm = LinearRewardTerm(
        weight=-0.5, limit=4.0
    )
    """Penalizes non-commanded vertical motion."""

    penalty_undesired_ang_vel_xy: LinearRewardTerm = LinearRewardTerm(
        weight=-0.1, limit=4.0
    )
    """Penalizes non-commanded roll and pitch motion."""

    ## Shaping parameters
    exp_std_cmd_lin_vel_xy: float = 0.5
    exp_std_cmd_ang_vel_z: float = 0.25


@configclass
class SceneCfg(GroundSceneCfg):
    pass


@configclass
class EventCfg(GroundEventCfg):
    command = SrbEventTermCfg(
        func=randomize_command,
        mode="interval",
        interval_range_s=(0.5, 5.0),
        category=RandomizationCategory.TASK,
        params={
            "env_attr_name": "_command",
            # "magnitude": 1.0,
        },
    )


@configclass
class TaskCfg(GroundEnvCfg):
    """Configuration for tracking sampled ground-robot velocity commands."""

    ## Scene
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 20.0
    is_finite_horizon: bool = False

    ## Visualization
    command_vis: bool = True

    def __post_init__(self):
        super().__post_init__()

        # Keep wheeled commands within the drive limits; such drives cannot move laterally.
        drive = next(
            (
                term
                for _, term in iter_config_items_by_type(
                    self._robot.action_group, WheeledDriveActionCfg
                )
            ),
            None,
        )
        if drive is not None:
            if isinstance(drive, FilteredWheeledDriveActionCfg):
                limits = (drive.v_lin_max, drive.v_ang_max)
            else:
                limits = (drive.scale_linear, drive.scale_angular)
            lin, ang = (
                abs(limit) if cap is None else min(abs(limit), cap)
                for limit, cap in zip(
                    limits, (drive.max_linear_velocity, drive.max_angular_velocity)
                )
            )
            self.events.command.params["magnitude"] = (lin, 0.0, ang)

        # BaseEnvCfg materializes this config when it adds active sensors.
        self.env_sensor_cfgs["event_camera_front"] = build_front_event_camera_cfg(
            f"{{ENV_REGEX_NS}}/robot/{self._robot.frame_base.prim_relpath}",
            preset_name=self.event_camera_preset,
            backend=self.event_camera_backend,
            sub_render_hz=self.event_camera_sub_render_hz,
            resolution=self.event_camera_resolution,
        )


############
### Task ###
############


class _VelocityTrackingTaskMixin:
    """Shared command, observation, reward, and visualization logic."""

    cfg: TaskCfg

    def _initialize_velocity_tracking(self) -> None:
        ## Initialize buffers
        self._command = torch.zeros(self.num_envs, 3, device=self.device)

        ## Visualization
        if self.cfg.command_vis or self.cfg.debug_vis:
            self._setup_visualization_markers()

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        # The interval event does not fire on reset, so resample to avoid
        # carrying the previous episode's command into the new one.
        randomize_command(self, env_ids, **self.cfg.events.command.params)

    def _setup_visualization_markers(self):
        ## Linear velocity
        cfg = ARROW_CFG.copy().replace(  # type: ignore[union-attr]
            prim_path="/Visuals/command/target_linvel"
        )
        cfg.markers["arrow"].tail_radius = 0.01
        cfg.markers["arrow"].tail_length = 0.5
        cfg.markers["arrow"].head_radius = 0.02
        cfg.markers["arrow"].head_length = 0.1
        cfg.markers["arrow"].visual_material = PreviewSurfaceCfg(
            emissive_color=(0.0, 1.0, 0.0)
        )
        self._marker_target_linvel = VisualizationMarkers(cfg)
        cfg = ARROW_CFG.copy().replace(  # type: ignore[union-attr]
            prim_path="/Visuals/command/robot_linvel"
        )
        cfg.markers["arrow"].tail_radius = 0.01
        cfg.markers["arrow"].tail_length = 0.5
        cfg.markers["arrow"].head_radius = 0.02
        cfg.markers["arrow"].head_length = 0.1
        cfg.markers["arrow"].visual_material = PreviewSurfaceCfg(
            emissive_color=(0.2, 0.8, 0.2)
        )
        self._marker_robot_linvel = VisualizationMarkers(cfg)

        ## Angular velocity
        cfg = ARROW_CFG.copy().replace(  # type: ignore[union-attr]
            prim_path="/Visuals/command/target_angvel"
        )
        cfg.markers["arrow"].tail_length = 0.0
        cfg.markers["arrow"].tail_radius = 0.0
        cfg.markers["arrow"].head_radius = 0.025
        cfg.markers["arrow"].head_length = 0.15
        cfg.markers["arrow"].visual_material = PreviewSurfaceCfg(
            emissive_color=(0.2, 0.2, 0.8)
        )
        self._marker_target_angvel = VisualizationMarkers(cfg)
        cfg = ARROW_CFG.copy().replace(  # type: ignore[union-attr]
            prim_path="/Visuals/command/robot_angvel"
        )
        cfg.markers["arrow"].tail_length = 0.0
        cfg.markers["arrow"].tail_radius = 0.0
        cfg.markers["arrow"].head_radius = 0.025
        cfg.markers["arrow"].head_length = 0.15
        cfg.markers["arrow"].visual_material = PreviewSurfaceCfg(
            emissive_color=(0.2, 0.2, 0.8)
        )
        self._marker_robot_angvel = VisualizationMarkers(cfg)

    def _update_visualization_markers(self):
        MARKER_OFFSET_Z_LINVEL = 0.2
        MARKER_OFFSET_Z_ANGVEL = 0.175

        ## Common
        robot_pos_w = self._robot.data.root_link_pos_w
        marker_pos = torch.zeros(
            (self.cfg.scene.num_envs, 3), dtype=torch.float32, device=self.device
        )
        marker_orientation = torch.zeros(
            (self.cfg.scene.num_envs, 4), dtype=torch.float32, device=self.device
        )
        marker_scale = torch.ones(
            (self.cfg.scene.num_envs, 3), dtype=torch.float32, device=self.device
        )
        marker_pos[:, :2] = robot_pos_w[:, :2]

        ## Target linear velocity
        marker_pos[:, 2] = robot_pos_w[:, 2] + MARKER_OFFSET_Z_LINVEL
        marker_heading = self._robot.data.heading_w + torch.atan2(
            self._command[:, 1], self._command[:, 0]
        )
        marker_orientation[:, 2] = torch.sin(marker_heading * 0.5)
        marker_orientation[:, 3] = torch.cos(marker_heading * 0.5)
        marker_scale[:, 0] = torch.norm(
            torch.stack(
                (self._command[:, 0], self._command[:, 1]),
                dim=-1,
            ),
            dim=-1,
        )
        self._marker_target_linvel.visualize(
            marker_pos, marker_orientation, marker_scale
        )

        ## Robot linear velocity
        marker_heading = self._robot.data.heading_w + torch.atan2(
            self._robot.data.root_lin_vel_b[:, 1],
            self._robot.data.root_lin_vel_b[:, 0],
        )
        marker_orientation[:, 2] = torch.sin(marker_heading * 0.5)
        marker_orientation[:, 3] = torch.cos(marker_heading * 0.5)
        marker_scale[:, 0] = torch.norm(self._robot.data.root_lin_vel_b[:, :2], dim=-1)
        self._marker_robot_linvel.visualize(
            marker_pos, marker_orientation, marker_scale
        )

        ## Target angular velocity
        normalization_fac = torch.where(
            self._command[:, 2] != 0.0,
            (torch.pi / 2.0) / self._command[:, 2].abs(),
            torch.ones_like(self._command[:, 2]),
        ).clamp(max=1.0)
        marker_pos[:, 2] = robot_pos_w[:, 2] + MARKER_OFFSET_Z_ANGVEL
        marker_heading = (
            self._robot.data.heading_w + normalization_fac * self._command[:, 2]
        )
        marker_orientation[:, 2] = torch.sin(marker_heading * 0.5)
        marker_orientation[:, 3] = torch.cos(marker_heading * 0.5)
        marker_scale[:, 0] = 1.0
        self._marker_target_angvel.visualize(
            marker_pos, marker_orientation, marker_scale
        )

        ## Robot angular velocity
        marker_heading = (
            self._robot.data.heading_w
            + normalization_fac * self._robot.data.root_ang_vel_w[:, -1]
        )
        marker_orientation[:, 2] = torch.sin(marker_heading * 0.5)
        marker_orientation[:, 3] = torch.cos(marker_heading * 0.5)
        marker_scale[:, 0] = 1.0
        self._marker_robot_angvel.visualize(
            marker_pos, marker_orientation, marker_scale
        )

    def extract_step_return(self) -> StepReturn:
        if self.cfg.command_vis or self.cfg.debug_vis:
            self._update_visualization_markers()

        num_envs = self.num_envs
        device = self.device

        rew = self.cfg.reward

        ################
        ## Raw states ##
        ################
        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        # Root
        tf_quat_robot = self._robot.data.root_quat_w
        vel_lin_robot = self._robot.data.root_lin_vel_b
        vel_ang_robot = self._robot.data.root_ang_vel_b

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        # IMU
        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        #####################
        ## Reward  metrics ##
        #####################
        # Penalty: Action rate
        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        # Command tracking (exp-shaped, 0 to 1)
        # Reward: Command tracking (linear)
        cmd_lin_vel_xy_metric = torch.exp(
            -torch.sum(torch.square(self._command[:, :2] - vel_lin_robot[:, :2]), dim=1)
            / rew.exp_std_cmd_lin_vel_xy
        )
        # Reward: Command tracking (angular)
        cmd_ang_vel_z_metric = torch.exp(
            -torch.square(self._command[:, 2] - vel_ang_robot[:, 2])
            / rew.exp_std_cmd_ang_vel_z
        )
        # Non-command motion
        # Penalty: Minimize non-command motion (linear)
        undesired_lin_vel_z_metric = torch.square(vel_lin_robot[:, 2])
        # Penalty: Minimize non-command motion (angular)
        undesired_ang_vel_xy_metric = torch.sum(
            torch.square(vel_ang_robot[:, :2]), dim=-1
        )

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "reward_cmd_lin_vel_xy": rew.reward_cmd_lin_vel_xy(cmd_lin_vel_xy_metric),
            "reward_cmd_ang_vel_z": rew.reward_cmd_ang_vel_z(cmd_ang_vel_z_metric),
            "penalty_undesired_lin_vel_z": rew.penalty_undesired_lin_vel_z(
                undesired_lin_vel_z_metric
            ),
            "penalty_undesired_ang_vel_xy": rew.penalty_undesired_ang_vel_xy(
                undesired_ang_vel_xy_metric
            ),
        }

        ##################
        ## Terminations ##
        ##################
        # No termination condition
        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                },
                ObservationGroup.PROPRIO: {
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                },
                ObservationGroup.COMMAND: {
                    "cmd_vel": self._command,
                },
            },
            reward,
            termination,
            truncation,
        )


class Task(_VelocityTrackingTaskMixin, SrbEnv, GroundEnv):
    """Default velocity-tracking task on the owned SRB lifecycle."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)
        self._initialize_velocity_tracking()
