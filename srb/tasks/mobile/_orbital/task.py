import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.env import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
    ViewerCfg,
)
from srb.core.env.mobile import OrbitalMobileRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d

##############
### Config ###
##############


class RewardCfg(OrbitalMobileRewardCfg):
    """Reward terms for the base orbital mobile task."""

    pass


@configclass
class SceneCfg(OrbitalSceneCfg):
    pass


@configclass
class EventCfg(OrbitalEventCfg):
    pass


@configclass
class TaskCfg(OrbitalEnvCfg):
    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 10.0
    is_finite_horizon: bool = False

    ## Viewer
    # The orbital base viewer stands 17 m off for large spacecraft and wide
    # scenes; this template ships a single CubeSat-class robot, which at that
    # standoff renders as a few dark pixels against space. This eye sits on
    # the sunlit side with Earth's limb behind the robot.
    viewer: ViewerCfg = ViewerCfg(
        eye=(-1.6, 1.6, 1.0),
        lookat=(0.0, 0.0, 0.0),
        origin_type="env",
    )


############
### Task ###
############


class Task(SrbEnv, OrbitalEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

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
        tf_pos_robot, tf_quat_robot, vel_lin_robot, vel_ang_robot = (
            self._robot_base_state()
        )

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

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
        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        fuel_consumption_metric = torch.square(1.0 - remaining_fuel.squeeze(-1))

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {
            # Penalty: Action rate
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            # Penalty: Fuel consumption
            "penalty_fuel_consumption": rew.penalty_fuel_consumption(
                fuel_consumption_metric
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
                    # Env-local, so replicas at different origins observe alike.
                    "tf_pos_robot": tf_pos_robot - self.scene.env_origins,
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
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
