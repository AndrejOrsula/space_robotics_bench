import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    Articulation,
    AssetVariant,
    GroundManipulator,
    Robot,
    Subterrane,
)
from srb.core.domain import Domain
from srb.core.env import (
    GroundManipulationEnv,
    GroundManipulationEnvCfg,
    GroundManipulationEventCfg,
    GroundManipulationSceneCfg,
    ViewerCfg,
)
from srb.core.env.mobile_manipulation import GroundManipulationRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.sensor import ContactSensor
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    normalize_joint_pos,
    radians,
    rotmat_to_rot6d,
)

##############
### Config ###
##############


class RewardCfg(GroundManipulationRewardCfg):
    """Reward terms for the Lunalab ground mobile manipulation task."""

    pass


@configclass
class SceneCfg(GroundManipulationSceneCfg):
    pass


@configclass
class EventCfg(GroundManipulationEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-2.0, 2.0),
                # Clear of the fixed extra robots at y >= 0.
                "y": (-3.5, -1.0),
                "z": (0.5, 1.0),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.5),
                "roll": (-radians(5.0), radians(5.0)),
                "pitch": (-radians(5.0), radians(5.0)),
                "yaw": (-radians(15.0), radians(15.0)),
            },
        },
    )


@configclass
class TaskCfg(GroundManipulationEnvCfg):
    ## Scenario
    domain: Domain = Domain.MOON

    ## Assets
    robot: GroundManipulator | Robot | AssetVariant = assets.GenericGroundManipulator(
        mobile_base=assets.Husky(),
        manipulator=assets.KinovaGen3n7(end_effector=assets.RobotiqHandE()),
    )
    scenery: Subterrane | AssetVariant | None = assets.Lunalab()

    ## Scene
    scene: SceneCfg = SceneCfg()
    # Share one world for this physical facility (env_spacing=0). SimForge's
    # multi-asset MoonSurface has no source prim to clone per environment.
    stack: bool = True

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    env_rate: float = 1.0 / 50.0
    agent_rate: float = 1.0 / 25.0
    episode_length_s: float = 100000.0
    is_finite_horizon: bool = False
    truncate_episodes: bool = False

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(-4.0, -5.0, 1.5), lookat=(0.0, -1.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        ## Extra robots
        self.extra_robots["leo_rover_1"] = assets.LeoRover()
        self.extra_robots["leo_rover_2"] = assets.LeoRover()
        self.extra_robots["cadre"] = assets.Cadre()

        # Set distinct initial positions.
        self.extra_robots["leo_rover_1"].asset_cfg.init_state.pos = (1.5, 0.0, 0.5)
        self.extra_robots["leo_rover_2"].asset_cfg.init_state.pos = (1.5, 1.5, 0.5)
        self.extra_robots["cadre"].asset_cfg.init_state.pos = (0.0, 1.5, 0.5)

        super().__post_init__()

        # Extra robots inherit the primary Husky's broad reset range. Disable
        # those cloned events to preserve their fixed positions on every reset.
        for extra_name in ("leo_rover_1", "leo_rover_2", "cadre"):
            event_name = f"randomize_robot_state_{extra_name}"
            if hasattr(self.events, event_name):
                setattr(self.events, event_name, None)


############
### Task ###
############


class Task(SrbEnv, GroundManipulationEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

    def extract_step_return(self) -> StepReturn:
        num_envs = self.num_envs
        device = self.device

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

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        # IMU
        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        # Joints
        joint_pos_robot = self._manipulator.data.joint_pos
        joint_pos_robot_normalized = normalize_joint_pos(
            joint_pos_robot, self._manipulator.data.soft_joint_pos_limits
        )

        joint_pos_end_effector = (
            self._end_effector.data.joint_pos
            if isinstance(self._end_effector, Articulation)
            else None
        )
        joint_pos_end_effector_normalized = (
            normalize_joint_pos(
                joint_pos_end_effector,
                self._end_effector.data.soft_joint_pos_limits,
            )
            if joint_pos_end_effector is not None
            and isinstance(self._end_effector, Articulation)
            else (
                joint_pos_end_effector
                if joint_pos_end_effector is not None
                else torch.empty((num_envs, 0), dtype=torch.float32, device=device)
            )
        )

        # Kinematics
        fk_pos_end_effector = self._tf_end_effector.data.target_pos_source[:, 0, :]
        fk_quat_end_effector = self._tf_end_effector.data.target_quat_source[:, 0, :]
        fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
        fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

        # Contacts
        contact_forces_robot = self._contacts_robot.data.net_forces_w
        contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

        contact_forces_end_effector = (
            self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None
        )
        contact_forces_mean_end_effector = (
            contact_forces_end_effector.mean(dim=1)
            if contact_forces_end_effector is not None
            else torch.empty((num_envs, 0), dtype=torch.float32, device=device)
        )
        contact_forces_end_effector_obs = (
            contact_forces_end_effector
            if contact_forces_end_effector is not None
            else torch.empty((num_envs, 0), dtype=torch.float32, device=device)
        )

        #####################
        ## Reward  metrics ##
        #####################
        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        max_contact = torch.max(torch.norm(contact_forces_robot, dim=-1), dim=1)[0]

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "penalty_undesired_robot_contacts": rew.penalty_undesired_robot_contacts(
                max_contact
            ),
        }

        ##################
        ## Terminations ##
        ##################
        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_pos_robot": tf_pos_robot,
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
                },
                ObservationGroup.STATE_DYN: {
                    "contact_forces_robot": contact_forces_robot,
                    "contact_forces_end_effector": contact_forces_end_effector_obs,
                    "fk_pos_end_effector": fk_pos_end_effector,
                    "fk_rot6d_end_effector": fk_rot6d_end_effector,
                },
                ObservationGroup.PROPRIO: {
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                },
                ObservationGroup.PROPRIO_DYN: {
                    "joint_pos_robot_normalized": joint_pos_robot_normalized,
                    "joint_pos_end_effector_normalized": joint_pos_end_effector_normalized,
                },
            },
            reward,
            termination,
            truncation,
        )
