from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.domain import Domain
from srb.core.env import AerialEnv, AerialEnvCfg, AerialEventCfg, AerialSceneCfg
from srb.core.env.mobile import AerialMobileRewardCfg
from srb.core.env.runtime import episode_outcome_info
from srb.core.env.srb_env import SrbEnv
from srb.core.marker import CYLINDER_CFG, VisualizationMarkers
from srb.core.reward import (
    HyperbolicBarrierRewardTerm,
    HyperbolicRewardTerm,
    HyperbolicTrackingRewardTerm,
    LinearRewardTerm,
    OffsetLinearRewardTerm,
    ThresholdRewardTerm,
)
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d

##############
### Config ###
##############


class RewardCfg(AerialMobileRewardCfg):
    """Waypoint, altitude, attitude, and safety rewards across mission phases.

    Phase progress outweighs other per-step terms. Reaching the final pad
    reports success without ending the episode.
    """

    reward_waypoint_approach: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=12.0, scale=4.0
    )
    """Smooth shaping reward that grows as the vehicle closes on the phase target."""

    reward_altitude_tracking: HyperbolicTrackingRewardTerm = (
        HyperbolicTrackingRewardTerm(weight=6.0, target=0.0, scale=1.0)
    )
    """Tracks altitude error to keep phase-specific clearance close to target."""

    reward_waypoint_reached: ThresholdRewardTerm = ThresholdRewardTerm(
        weight=20.0, threshold=0.7, operator="<"
    )
    """Rewards every step within the threshold of the current waypoint in XY.

    Altitude is not checked, and the term keeps paying while the vehicle holds
    over a waypoint, including the final landing pad.
    """

    penalty_tilt_angle: OffsetLinearRewardTerm = OffsetLinearRewardTerm(
        weight=-2.0, limit=0.6, threshold=0.35, direction=">"
    )
    """Penalises excessive tilt angles that destabilize controlled thrusting."""

    reward_terrain_clearance: HyperbolicBarrierRewardTerm = HyperbolicBarrierRewardTerm(
        weight=6.0, scale=0.8, threshold=0.8, direction="<"
    )
    """Protects against terrain impact by rewarding clearance above the barrier."""

    penalty_energy: LinearRewardTerm = LinearRewardTerm(weight=-0.6, limit=4.0)
    """Discourages saturated thrust commands and unnecessary activity."""

    reward_mission_progress: LinearRewardTerm = LinearRewardTerm(
        weight=60.0, limit=120.0
    )
    """Per-step bonus of ``weight`` per completed mission phase (0, 60, 120).
    The weight exceeds the per-step span of every other term together
    (44 best, -8.6 worst), so a later phase always outscores an earlier one."""


@configclass
class SceneCfg(AerialSceneCfg):
    pass


@configclass
class EventCfg(AerialEventCfg):
    pass


@configclass
class TaskCfg(AerialEnvCfg):
    """Configuration for the default Mars takeoff-and-landing mission."""

    ## Scenario
    domain: Domain = Domain.MARS
    scene: SceneCfg = SceneCfg()
    events: EventCfg = EventCfg()
    reward: RewardCfg = RewardCfg()

    ## Mission profile
    episode_length_s: float = 90.0
    is_finite_horizon: bool = True
    command_vis: bool = True
    env_rate: float = 1.0 / 50.0
    agent_rate: float = 1.0 / 25.0
    waypoint_xy_threshold: float = 0.75
    waypoint_altitude_threshold: float = 0.30
    mission_cruise_altitude: float = 5.5
    mission_pad_altitude: float = 1.0
    mission_ascent_xy_jitter: float = 0.75
    mission_ascent_altitude_jitter: float = 0.5
    mission_transfer_xy_radius: float = 8.0
    mission_pad_xy_jitter: float = 1.25

    ## Optional landing-only mode (starts directly at phase 2 / terminal landing descent)
    landing_only: bool = False

    ## Safety
    termination_min_clearance: float = 0.08
    termination_max_tilt: float = 1.2


############
### Task ###
############


class Task(SrbEnv, AerialEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._mission_phase = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._mission_waypoint = torch.zeros((self.num_envs, 3), device=self.device)
        # Latched after meeting the phase-2 landing criterion; cleared on reset.
        self._mission_landed = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._altimeter = getattr(self, "_altimeter", None)
        if self._altimeter is None and hasattr(self.scene, "sensors"):
            self._altimeter = self.scene.sensors.get("altimeter")
        # `_update_visualization_markers` draws under `command_vis or
        # debug_vis`, so the marker must exist under the same condition.
        if self.cfg.command_vis or self.cfg.debug_vis:
            self._setup_visualization_markers()

        if self.cfg.landing_only:
            self._mission_phase[:] = 2
        self._set_waypoint_for_phase(torch.arange(self.num_envs, device=self.device))

        # Keep extraction pure: score the finished step against this pre-advance
        # snapshot while observations use the current mission state.
        self._mission_phase_pre = self._mission_phase.clone()
        self._mission_waypoint_pre = self._mission_waypoint.clone()

    def _setup_visualization_markers(self):
        cfg = CYLINDER_CFG.copy()
        cfg.prim_path = "/Visuals/mission/target_waypoint"
        cfg.markers["cylinder"].radius = 0.45
        cfg.markers["cylinder"].height = 0.1
        self._target_marker: VisualizationMarkers = VisualizationMarkers(cfg)

    def _update_visualization_markers(self):
        if not self.cfg.command_vis and not self.cfg.debug_vis:
            return

        marker_pos = torch.zeros(
            (self.cfg.scene.num_envs, 3), dtype=torch.float32, device=self.device
        )
        marker_orient = torch.zeros(
            (self.cfg.scene.num_envs, 4), dtype=torch.float32, device=self.device
        )
        marker_orient[:, 3] = 1.0  # xyzw identity (markers are xyzw in this fork)
        marker_scale = torch.ones(
            (self.cfg.scene.num_envs, 3), dtype=torch.float32, device=self.device
        )
        marker_pos[:] = self._mission_waypoint
        self._target_marker.visualize(marker_pos, marker_orient, marker_scale)

    def _to_tensor_ids(self, env_ids: Sequence[int] | torch.Tensor) -> torch.Tensor:
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(dtype=torch.long, device=self.device)

        if len(env_ids) == 0:
            return torch.empty((0,), dtype=torch.long, device=self.device)

        return torch.tensor(env_ids, dtype=torch.long, device=self.device)

    def _set_waypoint_for_phase(self, env_ids: Sequence[int] | torch.Tensor) -> None:
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
            jitter_xy = (torch.rand((ids.numel(), 2), device=self.device) - 0.5) * 2.0
            jitter_xy *= self.cfg.mission_ascent_xy_jitter
            self._mission_waypoint[ids, :2] = env_origins[phase0, :2] + jitter_xy
            self._mission_waypoint[ids, 2] = (
                env_origins[phase0, 2]
                + self.cfg.mission_cruise_altitude
                + torch.rand(ids.numel(), device=self.device)
                * self.cfg.mission_ascent_altitude_jitter
            )

        if phase1.any():
            ids = env_ids_t[phase1]
            transfer_xy = (
                torch.rand((ids.numel(), 2), device=self.device) * 2.0 - 1.0
            ) * self.cfg.mission_transfer_xy_radius
            self._mission_waypoint[ids, :2] = env_origins[phase1, :2] + transfer_xy
            self._mission_waypoint[ids, 2] = (
                env_origins[phase1, 2] + self.cfg.mission_cruise_altitude
            )

        if phase2.any():
            ids = env_ids_t[phase2]
            pad_xy = (
                torch.rand((ids.numel(), 2), device=self.device) * 2.0 - 1.0
            ) * self.cfg.mission_pad_xy_jitter
            self._mission_waypoint[ids, :2] = env_origins[phase2, :2] + pad_xy
            self._mission_waypoint[ids, 2] = (
                env_origins[phase2, 2] + self.cfg.mission_pad_altitude
            )

    def _advance_mission_phase(self, env_ids: Sequence[int] | torch.Tensor) -> None:
        env_ids_t = self._to_tensor_ids(env_ids)
        if env_ids_t.numel() == 0:
            return

        current = self._mission_phase[env_ids_t]
        next_phase = torch.minimum(current + 1, torch.tensor(2, device=self.device))
        self._mission_phase[env_ids_t] = next_phase
        self._set_waypoint_for_phase(env_ids_t)

    def _compute_clearance(self, tf_pos_robot: torch.Tensor) -> torch.Tensor:
        altimeter = self._altimeter
        fallback_clearance = tf_pos_robot[:, 2] - self.scene.env_origins[:, 2]
        if altimeter is None:
            return fallback_clearance

        altimeter_heights = getattr(altimeter.data, "heights", None)
        if altimeter_heights is None:
            return fallback_clearance

        if altimeter_heights.ndim >= 2 and altimeter_heights.shape[1] > 0:
            # Reduce multi-cell altimeters to nearest clearance per environment.
            abs_h = torch.abs(altimeter_heights.reshape(self.num_envs, -1))
            valid = getattr(altimeter.data, "valid_mask", None)
            if valid is not None:
                # Ignore max-range sentinels; fall back to altitude if none are valid.
                vm = valid.reshape(self.num_envs, -1).bool()
                abs_h = torch.where(vm, abs_h, torch.full_like(abs_h, float("inf")))
                return torch.where(vm.any(dim=1), abs_h.amin(dim=1), fallback_clearance)
            return abs_h.amin(dim=1)

        return fallback_clearance

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        if len(env_ids) > 0:
            self._mission_phase[env_ids] = 2 if self.cfg.landing_only else 0
            self._mission_landed[env_ids] = False
            self._set_waypoint_for_phase(env_ids)
            # Reset rows need a fresh snapshot before post-reset extraction.
            self._mission_phase_pre[env_ids] = self._mission_phase[env_ids]
            self._mission_waypoint_pre[env_ids] = self._mission_waypoint[env_ids]
            # Step advancement is skipped on reset, so refresh the marker here.
            self._update_visualization_markers()

    def advance_step_state(self) -> None:
        """Advance the mission phase exactly once per step.

        Post-reset observation extraction covers every environment. Keeping
        this mutation here prevents a non-reset vehicle from advancing twice
        on the same step. Reward and termination use the pre-advance state in
        ``_mission_*_pre``; observations publish the current waypoint.
        """

        self._mission_phase_pre.copy_(self._mission_phase)
        self._mission_waypoint_pre.copy_(self._mission_waypoint)

        tf_pos_robot = self._robot.data.root_pos_w
        clearance = self._compute_clearance(tf_pos_robot)
        dist_to_waypoint_xy = torch.norm(
            (self._mission_waypoint - tf_pos_robot)[:, :2], dim=1
        )
        waypoint_altitude = self._mission_waypoint[:, 2] - self.scene.env_origins[:, 2]
        alt_error = torch.abs(clearance - waypoint_altitude)

        at_waypoint = (dist_to_waypoint_xy < self.cfg.waypoint_xy_threshold) & (
            alt_error < self.cfg.waypoint_altitude_threshold
        )
        # Phase 2 is the final phase: reaching the pad latches the episode
        # outcome (reported as success by ``extract_step_return``) but neither
        # advances the phase nor terminates the episode.
        self._mission_landed |= at_waypoint & (self._mission_phase == 2)
        # Phases 0 and 1 transition.
        mission_reached = at_waypoint & (
            (self._mission_phase == 0) | (self._mission_phase == 1)
        )
        self._advance_mission_phase(mission_reached.nonzero(as_tuple=False).squeeze(-1))

        # DirectEnv requires marker updates here; extraction can run again after
        # reset. The live-buffer marker then matches the published waypoint.
        self._update_visualization_markers()

    def extract_step_return(self) -> StepReturn:
        num_envs = self.num_envs
        device = self.device
        rew = self.cfg.reward

        ################
        ## Raw states ##
        ################
        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        tf_pos_robot = self._robot.data.root_pos_w
        tf_quat_robot = self._robot.data.root_quat_w
        vel_lin_robot = self._robot.data.root_lin_vel_b
        vel_ang_robot = self._robot.data.root_ang_vel_b

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        clearance = self._compute_clearance(tf_pos_robot)
        # Score the completed step from pre-advance state; observations use the
        # updated mission state.
        tf_pos_to_waypoint = self._mission_waypoint_pre - tf_pos_robot
        dist_to_waypoint_xy = torch.norm(tf_pos_to_waypoint[:, :2], dim=1)
        dist_to_waypoint_3d = torch.norm(tf_pos_to_waypoint, dim=1)
        waypoint_altitude = (
            self._mission_waypoint_pre[:, 2] - self.scene.env_origins[:, 2]
        )
        alt_error = torch.abs(clearance - waypoint_altitude)

        local_up = tf_rotmat_robot[:, :, 2]
        tilt_angle = torch.acos(local_up[:, 2].clamp(-1.0, 1.0))

        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        energy_metric = torch.sum(torch.square(act_current), dim=1)

        landing_reached = (
            (self._mission_phase_pre == 2)
            & (dist_to_waypoint_xy < self.cfg.waypoint_xy_threshold)
            & (alt_error < self.cfg.waypoint_altitude_threshold)
        )

        # Use the updated waypoint so relative and absolute observations agree
        # on phase-transition steps.
        rel_waypoint_obs = self._mission_waypoint - tf_pos_robot

        reward = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "reward_waypoint_approach": rew.reward_waypoint_approach(
                dist_to_waypoint_3d
            ),
            "reward_altitude_tracking": rew.reward_altitude_tracking(alt_error),
            "reward_waypoint_reached": rew.reward_waypoint_reached(dist_to_waypoint_xy),
            "penalty_tilt_angle": rew.penalty_tilt_angle(tilt_angle),
            "reward_terrain_clearance": rew.reward_terrain_clearance(clearance),
            "penalty_energy": rew.penalty_energy(energy_metric),
            "reward_mission_progress": rew.reward_mission_progress(
                self._mission_phase_pre.to(dtype=torch.float32)
            ),
        }

        # Only crashes terminate; landing remains a latched episode outcome.
        crashed = (clearance < self.cfg.termination_min_clearance) | (
            tilt_angle > self.cfg.termination_max_tilt
        )
        mission_success = self._mission_landed & ~crashed
        termination = crashed
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    # Env-local, so replicas at different origins observe alike.
                    "tf_pos_robot": tf_pos_robot - self.scene.env_origins,
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                    "rel_waypoint": rel_waypoint_obs,
                    "mission_phase": self._mission_phase.to(
                        dtype=torch.float32
                    ).unsqueeze(-1),
                },
                ObservationGroup.PROPRIO: {
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                    "clearance": clearance.unsqueeze(-1),
                    "mission_waypoint": self._mission_waypoint - self.scene.env_origins,
                },
            },
            reward,
            termination,
            truncation,
            info=episode_outcome_info(
                success=mission_success,
                metrics={
                    "crashed": crashed.float(),
                    "on_pad": landing_reached.float(),
                    "mission_phase": self._mission_phase_pre.float(),
                },
            ),
        )
