import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.env import AerialEnv, AerialEnvCfg, AerialEventCfg, AerialSceneCfg
from srb.core.env.mobile import AerialMobileRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d

##############
### Config ###
##############


class RewardCfg(AerialMobileRewardCfg):
    """Reward terms for the base aerial mobile task."""

    pass


@configclass
class SceneCfg(AerialSceneCfg):
    pass


@configclass
class EventCfg(AerialEventCfg):
    pass


@configclass
class TaskCfg(AerialEnvCfg):
    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 10.0
    is_finite_horizon: bool = False


############
### Task ###
############


class Task(SrbEnv, AerialEnv):
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

        #####################
        ## Reward  metrics ##
        #####################
        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {
            # Penalty: Action rate
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
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
                },
            },
            reward,
            termination,
            truncation,
        )
