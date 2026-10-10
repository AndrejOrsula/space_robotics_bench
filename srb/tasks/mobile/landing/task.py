from dataclasses import MISSING

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
from srb.core.mdp import (
    reset_root_state_uniform,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import HyperbolicRewardTerm, LinearRewardTerm
from srb.core.scene_introspection import get_scene_sensor
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import PreviewSurfaceCfg, SphereCfg
from srb.tasks.mobile.event_camera_wiring import build_onboard_event_camera_cfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, radians, rotmat_to_rot6d

##############
### Config ###
##############


class RewardCfg(OrbitalMobileRewardCfg):
    """Approach shaping and contact-gated landing rewards.

    Inherits action-rate and fuel-consumption terms.
    """

    ## Shaping penalties
    penalty_angular_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=-0.25, limit=5.0
    )
    """Penalises high angular velocity during approach."""

    penalty_gravity_rotation_alignment: LinearRewardTerm = LinearRewardTerm(
        weight=-1.0, limit=4.0
    )
    """Penalises tilt away from the gravity vector — keeps lander upright."""

    penalty_horizontal_linear_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=-0.1, limit=4.0
    )
    """Penalises lateral drift to encourage vertical descent profile."""

    ## Approach reward
    reward_vertical_linear_velocity: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=8.0, scale=1.0
    )
    """Rewards descent inside the ``v_z**2 <= height above target`` envelope;
    decays with the squared-speed excess beyond it."""

    ## Landing rewards (every step while in ground contact)
    reward_landing_angle: LinearRewardTerm = LinearRewardTerm(weight=8.0, limit=8.0)
    """Rewards upright orientation while landed."""

    reward_landing_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=16.0, limit=16.0
    )
    """Rewards resting while landed — 0.01 std means sub-cm/s + sub-0.01-rad/s
    combined velocity for full reward."""

    reward_landing_fuel: LinearRewardTerm = LinearRewardTerm(weight=4.0, limit=4.0)
    """Rewards remaining fuel fraction while landed — encourages efficiency."""

    ## Crash penalty
    penalty_crash: LinearRewardTerm = LinearRewardTerm(weight=-32.0, limit=32.0)
    """Crash penalty -- 2x the largest positive term; strongly discourages
    hard landings.  Reduced from -64 to allow more exploration."""

    ## Below-target penalty (replaces hard termination)
    penalty_below_target: LinearRewardTerm = LinearRewardTerm(weight=-16.0, limit=16.0)
    """Penalises falling far below the target altitude instead of terminating.
    Allows the agent to learn recovery from overshoot."""


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
                "x": (-50.0, 50.0),
                "y": (-50.0, 50.0),
                "z": (100.0, 200.0),
                "roll": (-radians(30.0), radians(30.0)),
                "pitch": (-radians(30.0), radians(30.0)),
                "yaw": (-torch.pi, torch.pi),
            },
            # Full-tank braking nets ~0.82 m/s^2 at the 30 deg roll+pitch corner,
            # so -12 m/s stops within ~88 m of the 100 m minimum spawn height.
            "velocity_range": {
                "x": (-10.0, 10.0),
                "y": (-10.0, 10.0),
                "z": (-12.0, -4.0),
                "roll": (-radians(15.0), radians(15.0)),
                "pitch": (-radians(15.0), radians(15.0)),
                "yaw": (-radians(15.0), radians(15.0)),
            },
        },
    )
    # External disturbance randomization is intentionally absent; see the
    # modelling envelope documentation.


@configclass
class TaskCfg(OrbitalEnvCfg):
    """Configuration for fuel-limited descent and planetary landing."""

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
    episode_length_s: float = 40.0
    is_finite_horizon: bool = True
    env_rate: float = 1.0 / 50.0
    agent_rate: float = 1.0 / 25.0

    viewer: ViewerCfg = ViewerCfg(
        # Follows the lander, which spawns up to 200 m above the terrain.
        eye=(8.0, -8.0, 4.0),
        lookat=(0.0, 0.0, -1.0),
        origin_type="asset_root",
        asset_name="robot",
    )

    ## Target
    tf_pos_target: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tf_quat_target: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": SphereCfg(
                radius=0.5,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    ## Sensors
    camera_clipping_range: tuple[float, float] | None = (0.01, 400.0)
    """Camera depth range (m); 400 m covers spawn terrain at image edges."""

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


############
### Task ###
############


class Task(SrbEnv, OrbitalEnv):
    """Powered descent and soft landing on a planetary surface."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        ## Get scene assets
        self._contacts_robot: ContactSensor = get_scene_sensor(
            self.scene, "contacts_robot"
        )
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
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
        num_envs = self.num_envs
        device = self.device
        dtype = torch.float32

        rew = self.cfg.reward

        ################
        ## Raw states ##
        ################
        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        # Root
        tf_pos_robot = self._robot.data.root_pos_w
        tf_quat_robot = self._robot.data.root_quat_w
        vel_lin_robot = self._robot.data.root_lin_vel_b
        vel_ang_robot = self._robot.data.root_ang_vel_b
        vel_lin_robot_w = self._robot.data.root_lin_vel_w
        projected_gravity_robot = self._robot.data.projected_gravity_b

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        # Transforms (world frame)
        tf_pos_robot_to_target = self._tf_pos_target - tf_pos_robot
        vertical_distance_to_target = torch.clamp_min(
            -tf_pos_robot_to_target[:, 2], 0.001
        )

        # Contacts
        contact_forces_robot = self._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
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

        # IMU
        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        # Fuel
        ## Get remaining fuel (if applicable)
        thrust_action_term = self._thrust_action_term
        if thrust_action_term is not None:
            remaining_fuel = (
                thrust_action_term.remaining_fuel / thrust_action_term.cfg.fuel_capacity
            ).unsqueeze(-1)
        else:
            remaining_fuel = torch.ones((num_envs, 1), dtype=dtype, device=device)

        #####################
        ## Reward  metrics ##
        #####################
        # Penalty: Action rate
        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        # Penalty: Fuel consumption
        fuel_consumption_metric = torch.square(1.0 - remaining_fuel.squeeze(-1))
        # Penalty: Angular velocity
        angular_velocity_metric = torch.sum(torch.square(vel_ang_robot), dim=1)
        # Penalty: Alignment with gravity
        gravity_alignment_metric = torch.sum(
            torch.square(projected_gravity_robot[:, :2]), dim=1
        ) + torch.square(projected_gravity_robot[:, 2] + 1.0)
        # Penalty: Horizontal linear velocity
        horizontal_velocity_metric = torch.sum(torch.abs(vel_lin_robot_w[:, :2]), dim=1)
        # Reward: Vertical linear velocity
        vertical_velocity_metric = torch.clamp_min(
            torch.square(vel_lin_robot_w[:, 2]) - vertical_distance_to_target, 0.0
        )

        # Conditional landing metrics (pre-shaped, passed to LinearRewardTerm)
        # Reward: Alignment with gravity (precision landing)
        landing_angle_metric = landed.float() * (
            1.0
            - torch.tanh(
                torch.acos(torch.clamp(-projected_gravity_robot[:, 2], -1.0, 1.0))
            )
        )
        # Reward: Velocity (precision landing)
        landing_velocity_metric = landed.float() * (
            1.0
            - torch.tanh(
                (torch.norm(vel_lin_robot, dim=-1) + torch.norm(vel_ang_robot, dim=-1))
                / 0.01
            )
        )
        # Reward: Remaining fuel (upon landing)
        landing_fuel_metric = landed.float() * remaining_fuel.squeeze(-1)

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {
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
            "reward_vertical_linear_velocity": rew.reward_vertical_linear_velocity(
                vertical_velocity_metric
            ),
            "reward_landing_angle": rew.reward_landing_angle(landing_angle_metric),
            "reward_landing_velocity": rew.reward_landing_velocity(
                landing_velocity_metric
            ),
            "reward_landing_fuel": rew.reward_landing_fuel(landing_fuel_metric),
            # Penalty: Crash
            "penalty_crash": rew.penalty_crash(crash.float()),
            "penalty_below_target": rew.penalty_below_target(
                (tf_pos_robot_to_target[:, 2] > 10.0).float()
            ),
        }

        ##################
        ## Terminations ##
        ##################
        # Termination: Crash (physical destruction -- keep as hard termination)
        termination = crash
        # Below-target is a penalty, not a termination
        # Truncation
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
                    "projected_gravity_robot": projected_gravity_robot,
                    "tf_pos_robot_to_target": tf_pos_robot_to_target,
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
