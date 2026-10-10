from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import AssetVariant, OrbitalRobot, Terrain
from srb.core.domain import Domain
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
from srb.core.reward import HyperbolicRewardTerm, LinearRewardTerm
from srb.core.scene_introspection import get_scene_sensor
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import PreviewSurfaceCfg, SphereCfg
from srb.tasks.mobile.event_camera_wiring import build_onboard_event_camera_cfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, radians, rotmat_to_rot6d


class RewardCfg(OrbitalMobileRewardCfg):
    """Reward terms for staged lunar descent and landing."""

    ## Shaping penalties
    penalty_angular_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=-0.25, limit=5.0
    )
    """Penalises high angular velocity during approach."""

    penalty_gravity_rotation_alignment: LinearRewardTerm = LinearRewardTerm(
        weight=-1.0, limit=4.0
    )
    """Penalises tilt away from the gravity vector."""

    penalty_horizontal_linear_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=-0.1, limit=4.0
    )
    """Penalises lateral drift to encourage controlled descent."""

    ## Approach reward
    reward_waypoint_progress: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=4.0, scale=4.0
    )
    """Rewards progress toward the current mission waypoint."""

    reward_vertical_linear_velocity: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=8.0, scale=1.0
    )
    """Rewards controlled descent and reduced vertical error at the waypoint."""

    ## Landing rewards (conditional on touchdown in phase 2)
    reward_landing_angle: LinearRewardTerm = LinearRewardTerm(weight=8.0, limit=8.0)
    """Rewards upright orientation at moment of touchdown."""

    reward_landing_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=16.0, limit=16.0
    )
    """Rewards gentle touchdown speed."""

    reward_landing_fuel: LinearRewardTerm = LinearRewardTerm(weight=4.0, limit=4.0)
    """Rewards remaining fuel fraction at touchdown."""

    ## Crash penalty
    penalty_crash: LinearRewardTerm = LinearRewardTerm(weight=-32.0, limit=32.0)
    """Strongly discourages hard landings and destabilization."""

    ## Below-target penalty (replaces hard termination)
    penalty_below_target: LinearRewardTerm = LinearRewardTerm(weight=-16.0, limit=16.0)
    """Penalises dropping far below the target corridor."""


@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing: float = 256.0

    ## Sensors
    contacts_robot: ContactSensorCfg = ContactSensorCfg(
        prim_path=MISSING,  # type: ignore[arg-type]
        track_air_time=True,
    )


@configclass
class EventCfg(OrbitalEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-6.0, 6.0),
                "y": (-6.0, 6.0),
                "z": (120.0, 220.0),
                "roll": (-radians(25.0), radians(25.0)),
                "pitch": (-radians(25.0), radians(25.0)),
                "yaw": (-torch.pi, torch.pi),
            },
            # Full-tank braking nets ~1.0 m/s^2 at the 25 deg roll+pitch corner
            # (~1.64 upright) with ~71 m/s of delta-v, so -12 m/s stops within
            # ~68 m of the 106 m above the deploy waypoint.
            "velocity_range": {
                "x": (-5.0, 5.0),
                "y": (-5.0, 5.0),
                "z": (-12.0, -4.0),
                "roll": (-radians(15.0), radians(15.0)),
                "pitch": (-radians(15.0), radians(15.0)),
                "yaw": (-radians(15.0), radians(15.0)),
            },
        },
    )


@configclass
class TaskCfg(OrbitalEnvCfg):
    """Configuration for the staged lunar descent task."""

    ## Scenario
    domain: Domain = Domain.MOON

    ## Assets
    robot: OrbitalRobot | AssetVariant = assets.PeregrineLander()
    scenery: Terrain | AssetVariant | None = AssetVariant.PROCEDURAL

    ## Scene
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 70.0
    is_finite_horizon: bool = True

    ## Mission design
    mission_deploy_altitude: float = 14.0
    mission_cruise_altitude: float = 1.8
    mission_pad_altitude: float = 0.0
    mission_deploy_xy_jitter: float = 2.0
    mission_pad_xy_jitter: float = 0.9
    mission_deploy_xy_tolerance: float = 1.8
    mission_pad_xy_tolerance: float = 1.1
    mission_altitude_tolerance: float = 1.2
    mission_sites: tuple[tuple[float, float, float], ...] = (
        (0.0, 0.0, 0.0),
        (7.0, 4.5, 0.0),
        (-7.0, -4.5, 0.0),
        (6.0, -5.0, 0.0),
        (-6.0, 5.0, 0.0),
    )

    ## Target and visual
    viewer: ViewerCfg = ViewerCfg(
        # Follows the lander, which spawns up to 220 m above the terrain.
        eye=(8.0, -8.0, 4.0),
        lookat=(0.0, 0.0, -1.0),
        origin_type="asset_root",
        asset_name="robot",
    )

    tf_pos_target: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tf_quat_target: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/mission_target",
        markers={
            "target": SphereCfg(
                radius=0.4,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.8, 0.2)),
            )
        },
    )

    ## Sensors
    camera_clipping_range: tuple[float, float] | None = (0.01, 400.0)
    """Onboard camera depth range in metres."""
    command_vis: bool = True

    def __post_init__(self):
        super().__post_init__()

        # Sensor: Robot contacts
        self.scene.contacts_robot.prim_path = f"{self.scene.robot.prim_path}/.*"

        # Sensor: Event camera on the lander base
        self.env_sensor_cfgs["event_camera_onboard"] = build_onboard_event_camera_cfg(
            f"{{ENV_REGEX_NS}}/robot/{self._robot.frame_base.prim_relpath}",
            preset_name=self.event_camera_preset,
            backend=self.event_camera_backend,
            sub_render_hz=self.event_camera_sub_render_hz,
            resolution=self.event_camera_resolution,
        )


class Task(SrbEnv, OrbitalEnv):
    """Navigate a deployment corridor, approach a pad, and land."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._contacts_robot: ContactSensor = get_scene_sensor(
            self.scene, "contacts_robot"
        )
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )
        self._altimeter = getattr(self, "_altimeter", None)
        if self._altimeter is None and hasattr(self.scene, "sensors"):
            self._altimeter = self.scene.sensors.get("altimeter")

        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

        self._mission_phase = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._mission_waypoint = torch.zeros((self.num_envs, 3), device=self.device)
        self._landing_site = torch.zeros((self.num_envs, 3), device=self.device)

        site_offsets = torch.tensor(
            self.cfg.mission_sites, dtype=torch.float32, device=self.device
        )
        if (
            site_offsets.ndim != 2
            or site_offsets.shape[0] == 0
            or site_offsets.shape[1] != 3
        ):
            raise ValueError(
                f"mission_sites must be a non-empty sequence of (x, y, z), got {self.cfg.mission_sites!r}"
            )
        self._mission_sites = site_offsets

        self._sample_landing_sites(torch.arange(self.num_envs, device=self.device))
        self._set_mission_waypoint(torch.arange(self.num_envs, device=self.device))

        # Preserve the waypoint used by reward and observation extraction.
        self._mission_waypoint_pre = self._mission_waypoint.clone()

    def _to_tensor_ids(self, env_ids: Sequence[int] | torch.Tensor) -> torch.Tensor:
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(dtype=torch.long, device=self.device)

        return torch.tensor(env_ids, dtype=torch.long, device=self.device)

    def _sample_landing_sites(self, env_ids: Sequence[int] | torch.Tensor) -> None:
        env_ids_t = self._to_tensor_ids(env_ids)
        if env_ids_t.numel() == 0:
            return

        env_origins = (
            self.scene.env_origins[env_ids_t].to(torch.float32).to(self.device)
        )

        num_sites = self._mission_sites.shape[0]
        if num_sites == 1:
            site_offsets = self._mission_sites.repeat(env_ids_t.numel(), 1)
        else:
            site_indices = torch.randint(
                low=0,
                high=num_sites,
                size=(env_ids_t.numel(),),
                device=self.device,
            )
            site_offsets = self._mission_sites[site_indices]

        landing_jitter_xy = (
            (torch.rand((env_ids_t.numel(), 2), device=self.device) - 0.5)
            * 2.0
            * self.cfg.mission_pad_xy_jitter
        )
        self._landing_site[env_ids_t, :2] = (
            env_origins[:, :2] + site_offsets[:, :2] + landing_jitter_xy
        )
        self._landing_site[env_ids_t, 2] = (
            env_origins[:, 2] + site_offsets[:, 2] + self.cfg.mission_pad_altitude
        )

        if self.cfg.command_vis:
            # Visualization rewrites all markers, so pass every environment.
            self._target_marker.visualize(self._landing_site, self._tf_quat_target)

    def _set_mission_waypoint(self, env_ids: Sequence[int] | torch.Tensor) -> None:
        env_ids_t = self._to_tensor_ids(env_ids)
        if env_ids_t.numel() == 0:
            return

        env_origins = self.scene.env_origins[env_ids_t]
        phases = self._mission_phase[env_ids_t]
        phase0 = phases == 0
        phase1 = phases == 1
        phase2 = phases == 2

        if phase0.any():
            ids = env_ids_t[phase0]
            deploy_xy_jitter = (
                (torch.rand((ids.numel(), 2), device=self.device) - 0.5)
                * 2.0
                * self.cfg.mission_deploy_xy_jitter
            )
            self._mission_waypoint[ids, :2] = env_origins[phase0, :2] + deploy_xy_jitter
            self._mission_waypoint[ids, 2] = (
                env_origins[phase0, 2] + self.cfg.mission_deploy_altitude
            )

        if phase1.any():
            ids = env_ids_t[phase1]
            self._mission_waypoint[ids, :2] = self._landing_site[ids, :2]
            self._mission_waypoint[ids, 2] = (
                self._landing_site[ids, 2] + self.cfg.mission_cruise_altitude
            )

        if phase2.any():
            ids = env_ids_t[phase2]
            self._mission_waypoint[ids, :2] = self._landing_site[ids, :2]
            self._mission_waypoint[ids, 2] = self._landing_site[ids, 2]

    def _advance_mission_phase(self, env_ids: Sequence[int] | torch.Tensor) -> None:
        env_ids_t = self._to_tensor_ids(env_ids)
        if env_ids_t.numel() == 0:
            return

        self._mission_phase[env_ids_t] = torch.minimum(
            self._mission_phase[env_ids_t] + 1,
            torch.tensor(2, device=self.device),
        )
        self._set_mission_waypoint(env_ids_t)

    def _compute_clearance(self, tf_pos_robot: torch.Tensor) -> torch.Tensor:
        altimeter = self._altimeter
        fallback_clearance = tf_pos_robot[:, 2] - self.scene.env_origins[:, 2]
        if altimeter is None:
            return fallback_clearance

        altimeter_heights = getattr(altimeter.data, "heights", None)
        if altimeter_heights is None:
            return fallback_clearance

        if altimeter_heights.ndim >= 2 and altimeter_heights.shape[1] > 0:
            # Use the nearest cell to support multi-cell altimeters.
            abs_h = torch.abs(altimeter_heights.reshape(self.num_envs, -1))
            valid = getattr(altimeter.data, "valid_mask", None)
            if valid is not None:
                # Ignore max-range sentinels; use altitude if all cells are invalid.
                vm = valid.reshape(self.num_envs, -1).bool()
                abs_h = torch.where(vm, abs_h, torch.full_like(abs_h, float("inf")))
                return torch.where(vm.any(dim=1), abs_h.amin(dim=1), fallback_clearance)
            return abs_h.amin(dim=1)

        return fallback_clearance

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        if len(env_ids) == 0:
            return

        self._mission_phase[env_ids] = 0
        self._sample_landing_sites(env_ids)
        self._set_mission_waypoint(env_ids)
        # Reset rows must publish their new waypoint after re-extraction.
        self._mission_waypoint_pre[env_ids] = self._mission_waypoint[env_ids]

    def advance_step_state(self) -> None:
        """Advance phases once per step; extraction scores the saved waypoint."""

        self._mission_waypoint_pre.copy_(self._mission_waypoint)

        tf_pos_robot = self._robot.data.root_pos_w
        tf_pos_to_waypoint = self._mission_waypoint - tf_pos_robot
        dist_to_waypoint_xy = torch.norm(tf_pos_to_waypoint[:, :2], dim=1)
        altitude_error = torch.abs((tf_pos_to_waypoint[:, 2]))
        clearance = self._compute_clearance(tf_pos_robot)

        phase0 = self._mission_phase == 0
        phase1 = self._mission_phase == 1
        # Combine full-size masks to support environments in mixed phases.
        mission_reached = (
            phase0
            & (dist_to_waypoint_xy < self.cfg.mission_deploy_xy_tolerance)
            & (altitude_error < self.cfg.mission_altitude_tolerance)
            & (clearance > self.cfg.mission_pad_altitude)
        ) | (
            phase1
            & (dist_to_waypoint_xy < self.cfg.mission_pad_xy_tolerance)
            & (altitude_error < self.cfg.mission_altitude_tolerance)
        )
        self._advance_mission_phase(mission_reached.nonzero(as_tuple=False).squeeze(-1))

    def extract_step_return(self) -> StepReturn:
        num_envs = self.num_envs
        device = self.device

        rew = self.cfg.reward

        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        tf_pos_robot = self._robot.data.root_pos_w
        tf_quat_robot = self._robot.data.root_quat_w
        vel_lin_robot = self._robot.data.root_lin_vel_b
        vel_ang_robot = self._robot.data.root_ang_vel_b
        vel_lin_robot_w = self._robot.data.root_lin_vel_w
        projected_gravity_robot = self._robot.data.projected_gravity_b

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        # Score and publish the pre-advance waypoint for the completed step.
        tf_pos_to_waypoint = self._mission_waypoint_pre - tf_pos_robot
        dist_to_waypoint_3d = torch.norm(tf_pos_to_waypoint, dim=1)
        # Publish a robot-relative landing site to avoid leaking environment origins.
        tf_pos_to_landing_site = self._landing_site - tf_pos_robot

        contact_forces_robot = self._contacts_robot.data.net_forces_w
        contact_robot = self._contacts_robot.compute_first_contact(self.step_dt)

        MAX_TOUCHDOWN_LINEAR_VELOCITY = 2.0
        MAX_TOUCHDOWN_ANGLE = 0.174533  # 10 degrees
        touchdown = contact_robot.any(dim=1)
        crash = touchdown & (
            (torch.norm(vel_lin_robot, dim=1) > MAX_TOUCHDOWN_LINEAR_VELOCITY)
            | (
                torch.acos(torch.clamp(-projected_gravity_robot[:, 2], -1.0, 1.0))
                > MAX_TOUCHDOWN_ANGLE
            )
        )
        landed = touchdown | (torch.norm(contact_forces_robot.mean(dim=1), dim=1) > 1.0)

        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        thrust_action_term = self._thrust_action_term
        if thrust_action_term is not None:
            remaining_fuel = (
                thrust_action_term.remaining_fuel / thrust_action_term.cfg.fuel_capacity
            ).unsqueeze(-1)
        else:
            remaining_fuel = torch.ones(
                (num_envs, 1), dtype=torch.float32, device=device
            )

        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        fuel_consumption_metric = torch.square(1.0 - remaining_fuel.squeeze(-1))
        angular_velocity_metric = torch.sum(torch.square(vel_ang_robot), dim=1)
        gravity_alignment_metric = torch.sum(
            torch.square(projected_gravity_robot[:, :2]), dim=1
        ) + torch.square(projected_gravity_robot[:, 2] + 1.0)
        horizontal_velocity_metric = torch.sum(torch.abs(vel_lin_robot_w[:, :2]), dim=1)
        vertical_distance_to_target = torch.clamp_min(-tf_pos_to_waypoint[:, 2], 0.001)
        vertical_velocity_metric = torch.clamp_min(
            torch.square(vel_lin_robot_w[:, 2]) - vertical_distance_to_target, 0.0
        )

        landing_phase = (self._mission_phase == 2).to(dtype=torch.float32)
        landing_angle_metric = (
            landing_phase
            * landed.float()
            * (
                1.0
                - torch.tanh(
                    torch.acos(torch.clamp(-projected_gravity_robot[:, 2], -1.0, 1.0))
                )
            )
        )
        landing_velocity_metric = (
            landing_phase
            * landed.float()
            * (
                1.0
                - torch.tanh(
                    (
                        torch.norm(vel_lin_robot, dim=-1)
                        + torch.norm(vel_ang_robot, dim=-1)
                    )
                    / 0.01
                )
            )
        )
        landing_fuel_metric = (
            landing_phase * landed.float() * remaining_fuel.squeeze(-1)
        )

        reward = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "penalty_fuel_consumption": rew.penalty_fuel_consumption(
                fuel_consumption_metric
            ),
            "penalty_angular_velocity": rew.penalty_angular_velocity(
                angular_velocity_metric
            ),
            "penalty_gravity_rotation_alignment": rew.penalty_gravity_rotation_alignment(
                gravity_alignment_metric
            ),
            "penalty_horizontal_linear_velocity": rew.penalty_horizontal_linear_velocity(
                horizontal_velocity_metric
            ),
            "reward_waypoint_progress": rew.reward_waypoint_progress(
                dist_to_waypoint_3d
            ),
            "reward_vertical_linear_velocity": rew.reward_vertical_linear_velocity(
                vertical_velocity_metric
            ),
            "reward_landing_angle": rew.reward_landing_angle(landing_angle_metric),
            "reward_landing_velocity": rew.reward_landing_velocity(
                landing_velocity_metric
            ),
            "reward_landing_fuel": rew.reward_landing_fuel(landing_fuel_metric),
            "penalty_crash": rew.penalty_crash(crash.float()),
            "penalty_below_target": rew.penalty_below_target(
                (tf_pos_to_waypoint[:, 2] > 10.0).float()
            ),
        }

        mission_success = (self._mission_phase == 2) & landed
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )
        termination = crash | mission_success

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                    "projected_gravity_robot": projected_gravity_robot,
                    "tf_pos_robot_to_target": self._mission_waypoint - tf_pos_robot,
                    "mission_phase": self._mission_phase.to(
                        dtype=torch.float32
                    ).unsqueeze(-1),
                    "landing_site": tf_pos_to_landing_site,
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
