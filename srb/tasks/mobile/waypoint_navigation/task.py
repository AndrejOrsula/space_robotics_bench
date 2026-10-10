import math
from dataclasses import MISSING
from numbers import Real
from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.env import GroundEnv, GroundEnvCfg, GroundEventCfg, GroundSceneCfg
from srb.core.env.mobile import GroundMobileRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import offset_pose_natural, reset_pose_natural
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import LinearRewardTerm
from srb.core.sim import PreviewSurfaceCfg
from srb.core.sim.spawners.shapes.extras.cfg import PinnedArrowCfg
from srb.tasks.mobile.event_camera_wiring import build_front_event_camera_cfg
from srb.utils import logging
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, subtract_frame_transforms

from .synthetic_profile import WAYPOINT_SYNTHETIC_PROFILE


class RewardCfg(GroundMobileRewardCfg):
    """Waypoint rewards with staged heading, position, orientation, and
    action-rate terms. Inherits ``penalty_action_rate`` from
    :class:`GroundMobileRewardCfg`.
    """

    penalty_position_tracking: LinearRewardTerm = LinearRewardTerm(
        weight=-1.0, limit=4.0
    )
    """Penalises squared distance to target waypoint.  Always active;
    provides coarse gradient toward the waypoint."""

    reward_point_towards_target: LinearRewardTerm = LinearRewardTerm(
        weight=1.0, limit=1.0
    )
    """Rewards heading alignment toward the target position.
    Tanh std ~45 deg (0.7854 rad): ~0.24 of full reward at 45 deg heading error."""

    reward_position_tracking_precision: LinearRewardTerm = LinearRewardTerm(
        weight=4.0, limit=4.0
    )
    """Rewards precise position tracking near the waypoint.
    Tanh std 5 cm: ~0.24 of full reward at 5 cm offset."""

    reward_orientation_tracking: LinearRewardTerm = LinearRewardTerm(
        weight=8.0, limit=8.0
    )
    """Rewards yaw orientation tracking, gated on position precision.
    Tanh std ~15 deg (0.2618 rad) — activates only once position is close."""

    reward_action_rate_at_target: LinearRewardTerm = LinearRewardTerm(
        weight=16.0, limit=16.0
    )
    """Rewards smooth actions while at the target pose.  Reduced from 32
    to prevent dominating orientation tracking and discouraging movement."""

    tanh_std_point_towards_target: float = 0.7854
    tanh_std_position_tracking_precision: float = 0.05
    tanh_std_orientation_tracking: float = 0.2618
    tanh_std_action_rate_at_target: float = 0.1


@configclass
class SceneCfg(GroundSceneCfg):
    pass


@configclass
class EventCfg(GroundEventCfg):
    target_pose_evolution: SrbEventTermCfg = SrbEventTermCfg(
        func=offset_pose_natural,
        mode="interval",
        interval_range_s=(0.05, 0.05),
        is_global_time=True,
        category=RandomizationCategory.TASK,
        params={
            "env_attr_name": "_goal",
            "pos_axes": ("x", "y"),
            "pos_step_range": (0.005, 0.01),
            "pos_smoothness": 0.99,
            "pos_step_smoothness": 0.8,
            "pos_bounds": {
                "x": MISSING,
                "y": MISSING,
            },
            "orient_yaw_only": True,
            "orient_smoothness": 0.8,
        },
    )


@configclass
class TaskCfg(GroundEnvCfg):
    """Config for sequential ground waypoint navigation and heading control."""

    scene: SceneCfg = SceneCfg()
    stack: bool = True

    events: EventCfg = EventCfg()

    reward: RewardCfg = RewardCfg()

    episode_length_s: float = 60.0
    is_finite_horizon: bool = False

    target_pos_range_ratio: float = 0.9
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": PinnedArrowCfg(
                pin_radius=0.01,
                pin_length=2.0,
                tail_radius=0.01,
                tail_length=0.2,
                head_radius=0.04,
                head_length=0.08,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    action_delay_steps: int | tuple[int, int] = (0, 3)
    observation_delay_steps: int | tuple[int, int] = (0, 1)

    observation_episode_noise_pos_std_m: float = (
        WAYPOINT_SYNTHETIC_PROFILE.episode_position_noise_std_m
    )
    observation_episode_noise_yaw_std_rad: float = (
        WAYPOINT_SYNTHETIC_PROFILE.episode_yaw_noise_std_rad
    )
    observation_step_noise_pos_std_m: float = (
        WAYPOINT_SYNTHETIC_PROFILE.step_position_noise_std_m
    )
    observation_step_noise_yaw_std_rad: float = (
        WAYPOINT_SYNTHETIC_PROFILE.step_yaw_noise_std_rad
    )

    sim_to_real_profile: str = "none"
    sim_to_real_profile_provenance: str = "unprofiled"

    def __post_init__(self):
        self._apply_sim_to_real_profile()
        super().__post_init__()

        # Event: Waypoint target
        if (
            "hardcoded"
            not in self.events.target_pose_evolution.params["pos_bounds"].keys()  # type: ignore[attr-defined]
        ):
            assert self.spacing is not None
            for dim in ("x", "y"):
                self.events.target_pose_evolution.params["pos_bounds"][dim] = (  # type: ignore[attr-defined]
                    -0.5 * self.target_pos_range_ratio * self.spacing,
                    0.5 * self.target_pos_range_ratio * self.spacing,
                )

        # Mounted on the robot base; BaseEnvCfg spawns it only when active.
        self.env_sensor_cfgs["event_camera_front"] = build_front_event_camera_cfg(
            f"{{ENV_REGEX_NS}}/robot/{self._robot.frame_base.prim_relpath}",
            preset_name=self.event_camera_preset,
            backend=self.event_camera_backend,
            sub_render_hz=self.event_camera_sub_render_hz,
            resolution=self.event_camera_resolution,
        )

    def _apply_sim_to_real_profile(self) -> None:
        """Apply synthetic profile values before delay validation."""

        for field_name in (
            "observation_episode_noise_pos_std_m",
            "observation_episode_noise_yaw_std_rad",
            "observation_step_noise_pos_std_m",
            "observation_step_noise_yaw_std_rad",
        ):
            value = getattr(self, field_name)
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(float(value))
                or value < 0.0
            ):
                raise ValueError(f"{field_name} must be finite and non-negative")
            setattr(self, field_name, float(value))

        if self.sim_to_real_profile == "none":
            return
        if self.sim_to_real_profile != WAYPOINT_SYNTHETIC_PROFILE.name:
            raise ValueError(
                "unknown waypoint-navigation sim_to_real_profile: "
                f"{self.sim_to_real_profile!r}; expected 'none' or "
                f"{WAYPOINT_SYNTHETIC_PROFILE.name!r}"
            )

        profile = WAYPOINT_SYNTHETIC_PROFILE
        if (
            isinstance(self.agent_rate, bool)
            or not isinstance(self.agent_rate, Real)
            or not math.isclose(
                float(self.agent_rate),
                profile.agent_step_s,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
        ):
            raise ValueError(
                f"{profile.name} requires agent_rate={profile.agent_step_s} s; "
                f"got {self.agent_rate!r}"
            )
        if self.seed is None:
            self.seed = profile.default_seed
        self.sim_to_real_profile_provenance = profile.provenance
        pinned = {
            "action_delay_steps": profile.action_delay_steps,
            "observation_delay_steps": profile.observation_delay_steps,
            "action_delay_on_step_change_freq": profile.delay_change_freq_s,
            "observation_delay_on_step_change_freq": profile.delay_change_freq_s,
            "action_delay_on_step_change_prob": profile.delay_change_prob,
            "observation_delay_on_step_change_prob": profile.delay_change_prob,
            "observation_episode_noise_pos_std_m": profile.episode_position_noise_std_m,
            "observation_episode_noise_yaw_std_rad": profile.episode_yaw_noise_std_rad,
            "observation_step_noise_pos_std_m": profile.step_position_noise_std_m,
            "observation_step_noise_yaw_std_rad": profile.step_yaw_noise_std_rad,
        }
        fields = type(self).__dataclass_fields__
        for name, value in pinned.items():
            # Overrides that match neither the task default nor the profile conflict.
            field = fields[name]
            default = (
                field.default_factory()  # type: ignore[misc]
                if field.default is MISSING
                else field.default
            )
            current = getattr(self, name)
            if _as_tuple(current) not in (_as_tuple(default), _as_tuple(value)):
                raise ValueError(
                    f"{profile.name} fixes {name}; remove the env.{name} override"
                )
            setattr(self, name, value)

        if getattr(self, "action_noise_model", None) is not None:
            raise ValueError(
                f"{profile.name} fixes action_noise_model; remove the custom "
                "action_noise_model override"
            )
        from isaaclab.utils.noise import GaussianNoiseCfg, NoiseModelCfg

        self.action_noise_model = NoiseModelCfg(
            noise_cfg=GaussianNoiseCfg(
                mean=0.0,
                std=profile.action_noise_std_normalized,
            )
        )


def _as_tuple(value: object) -> object:
    return tuple(value) if isinstance(value, (list, tuple)) else value


class _WaypointTaskMixin:
    cfg: TaskCfg

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        self._goal[env_ids, 0:3] = self.scene.env_origins[env_ids]
        self._goal[env_ids, 3:7] = torch.tensor(
            [0.0, 0.0, 0.0, 1.0],
            device=self.device,  # xyzw identity
        )
        # Clear the natural-movement drift/orientation state so the moving goal
        # starts fresh from this reset pose rather than leaking the previous
        # episode's drift direction/speed and stale yaw.
        reset_pose_natural(self, env_ids, "_goal")

        num_reset_envs = len(env_ids)
        self._episodic_noise_tf_pos2d[env_ids] = torch.normal(
            mean=0.0,
            std=self.cfg.observation_episode_noise_pos_std_m,
            size=(num_reset_envs, 2),
            device=self.device,
        )
        self._episodic_noise_tf_yaw[env_ids] = torch.normal(
            mean=0.0,
            std=self.cfg.observation_episode_noise_yaw_std_rad,
            size=(num_reset_envs,),
            device=self.device,
        )

        # Constructed once per env lifetime, as in ``task_orbital``:
        # TransformListener registers a /tf subscription on the shared ros_node
        # that is never torn down, so rebuilding it on every reset would leak
        # one subscription per episode.
        if hasattr(self.unwrapped, "ros_node") and not hasattr(self, "tf_buffer"):
            from tf2_ros import Buffer, TransformListener

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer,
                self.unwrapped.ros_node,  # type: ignore[misc]
            )
            logging.info("TF listener initialized.")
            self.cfg.events.target_pose_evolution.interval_range_s = (1.0e9, 1.0e9)
            self.event_manager.set_term_cfg(
                "target_pose_evolution", self.cfg.events.target_pose_evolution
            )

    def advance_step_state(self) -> None:
        if hasattr(self, "tf_buffer"):
            from rclpy.time import Duration, Time

            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    "world",
                    "target",
                    Time(),
                    Duration(
                        seconds=0,
                        nanoseconds=200000000,
                    ),
                )
                logging.debug(f"Got transform from 'world' to 'target': {tf_stamped}")
                # The TF target is env-local: offset it by each env origin so
                # every env (distinct origins under `env.stack=false`) gets the
                # target in its own cell rather than env_0's world position.
                translation = tf_stamped.transform.translation
                self._goal[:, 0:3] = self.scene.env_origins + torch.tensor(
                    (translation.x, translation.y, translation.z),
                    dtype=self._goal.dtype,
                    device=self._goal.device,
                )
                self._goal[:, 3] = tf_stamped.transform.rotation.x
                self._goal[:, 4] = tf_stamped.transform.rotation.y
                self._goal[:, 5] = tf_stamped.transform.rotation.z
                self._goal[:, 6] = tf_stamped.transform.rotation.w
            except Exception as e:
                logging.warning(
                    f"Failed to get transform from 'world' to 'target': {e}"
                )

        self._target_marker.visualize(self._goal[:, 0:3], self._goal[:, 3:7])

    def extract_step_return(self) -> StepReturn:
        _robot_pose = self._robot.data.root_link_pose_w

        num_envs = self.num_envs
        device = self.device

        rew = self.cfg.reward

        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        tf_pos_robot = _robot_pose[:, 0:3]
        tf_quat_robot = _robot_pose[:, 3:7]

        tf_pos_robot_to_target, _tf_quat_robot_to_target = subtract_frame_transforms(
            t01=tf_pos_robot,
            q01=tf_quat_robot,
            t02=self._goal[:, 0:3],
            q02=self._goal[:, 3:7],
        )
        tf_rotmat_robot_to_target = matrix_from_quat(_tf_quat_robot_to_target)

        tf_pos2d_robot_to_target = tf_pos_robot_to_target[:, :2]
        dist2d_robot_to_target = torch.norm(tf_pos2d_robot_to_target, dim=-1)
        angle_robot_to_target_pos = torch.atan2(
            tf_pos_robot_to_target[..., 1], tf_pos_robot_to_target[..., 0]
        )
        yaw_robot_to_target = torch.atan2(
            tf_rotmat_robot_to_target[..., 1, 0],
            tf_rotmat_robot_to_target[..., 0, 0],
        )

        tf_pos2d_robot_to_target_with_noise = (
            tf_pos2d_robot_to_target
            + self._episodic_noise_tf_pos2d
            + torch.normal(
                mean=0.0,
                std=self.cfg.observation_step_noise_pos_std_m,
                size=(num_envs, 2),
                device=device,
            )
        )
        yaw_robot_to_target_with_noise = (
            yaw_robot_to_target
            + self._episodic_noise_tf_yaw
            + torch.normal(
                mean=0.0,
                std=self.cfg.observation_step_noise_yaw_std_rad,
                size=(num_envs,),
                device=device,
            )
        )
        tf_rot2dtrigyaw_robot_to_target_with_noise = torch.stack(
            (
                torch.sin(yaw_robot_to_target_with_noise),
                torch.cos(yaw_robot_to_target_with_noise),
            ),
            dim=-1,
        )

        _action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)

        position_tracking_metric = torch.square(dist2d_robot_to_target)
        point_towards_metric = 1.0 - torch.tanh(
            torch.abs(angle_robot_to_target_pos) / rew.tanh_std_point_towards_target
        )
        _position_tracking_precision = 1.0 - torch.tanh(
            dist2d_robot_to_target / rew.tanh_std_position_tracking_precision
        )
        position_precision_metric = _position_tracking_precision
        _orientation_tracking_precision = _position_tracking_precision * (
            1.0
            - torch.tanh(
                torch.abs(yaw_robot_to_target) / rew.tanh_std_orientation_tracking
            )
        )
        orientation_tracking_metric = _orientation_tracking_precision
        action_rate_at_target_metric = _orientation_tracking_precision * (
            1.0 - torch.tanh(_action_rate / rew.tanh_std_action_rate_at_target)
        )

        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(_action_rate),
            "penalty_position_tracking": rew.penalty_position_tracking(
                position_tracking_metric
            ),
            "reward_point_towards_target": rew.reward_point_towards_target(
                point_towards_metric
            ),
            "reward_position_tracking_precision": rew.reward_position_tracking_precision(
                position_precision_metric
            ),
            "reward_orientation_tracking": rew.reward_orientation_tracking(
                orientation_tracking_metric
            ),
            "reward_action_rate_at_target": rew.reward_action_rate_at_target(
                action_rate_at_target_metric
            ),
        }

        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_pos2d_robot_to_target": tf_pos2d_robot_to_target_with_noise,
                    "tf_rot2dtrigyaw_robot_to_target": tf_rot2dtrigyaw_robot_to_target_with_noise,
                },
            },
            reward,
            termination,
            truncation,
        )


class Task(_WaypointTaskMixin, SrbEnv, GroundEnv):
    """Navigate to sequential waypoints with heading alignment."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg, runtime_mode=runtime_mode, **kwargs)

        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        self._goal = torch.zeros(self.num_envs, 7, device=self.device)
        self._goal[:, 0:3] = self.scene.env_origins
        self._goal[:, 6] = 1.0  # xyzw identity (w at index 6)
        self._episodic_noise_tf_pos2d = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        self._episodic_noise_tf_yaw = torch.zeros(self.num_envs, device=self.device)
