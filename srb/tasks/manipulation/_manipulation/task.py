import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.env.manipulation import ManipulationRewardCfg
from srb.core.env.manipulation._step_state import manipulator_step_state
from srb.core.env.srb_env import SrbEnv
from srb.core.sensor import ContactSensor
from srb.utils.cfg import configclass

##############
### Config ###
##############


class RewardCfg(ManipulationRewardCfg):
    """Reward terms for the base manipulation task."""

    pass


@configclass
class SceneCfg(ManipulationSceneCfg):
    pass


@configclass
class EventCfg(ManipulationEventCfg):
    pass


@configclass
class TaskCfg(ManipulationEnvCfg):
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


class Task(SrbEnv, ManipulationEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg, runtime_mode=runtime_mode, **kwargs)

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward

        ################
        ## Raw states ##
        ################
        state = manipulator_step_state(self)
        num_envs, device, dtype = state.num_envs, state.device, state.dtype

        # Contacts
        contact_forces_robot = self._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
        contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

        has_ee_contacts = isinstance(self._contacts_end_effector, ContactSensor)
        if has_ee_contacts:
            contact_forces_ee = self._contacts_end_effector.data.net_forces_w
            contact_forces_mean_ee = contact_forces_ee.mean(dim=1)
        else:
            contact_forces_ee = torch.empty((num_envs, 0), dtype=dtype, device=device)
            contact_forces_mean_ee = torch.empty(
                (num_envs, 0), dtype=dtype, device=device
            )

        #####################
        ## Reward  metrics ##
        #####################
        action_rate_metric = torch.mean(
            torch.square(state.act_current - state.act_previous), dim=1
        )
        joint_torque_metric = torch.sum(
            torch.square(state.joint_applied_torque_robot), dim=1
        )
        joint_acc_metric = torch.sum(torch.square(state.joint_acc_robot), dim=1)
        undesired_contacts_metric = torch.max(
            torch.norm(contact_forces_robot, dim=-1), dim=1
        )[0]

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {
            # Penalty: Action rate
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            # Penalty: Joint torque
            "penalty_joint_torque": rew.penalty_joint_torque(joint_torque_metric),
            # Penalty: Joint acceleration
            "penalty_joint_acceleration": rew.penalty_joint_acceleration(
                joint_acc_metric
            ),
            # Penalty: Undesired robot contacts
            "penalty_undesired_robot_contacts": rew.penalty_undesired_robot_contacts(
                undesired_contacts_metric
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
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "contact_forces_mean_end_effector": contact_forces_mean_ee,
                },
                ObservationGroup.STATE_DYN: {
                    "contact_forces_robot": contact_forces_robot,
                    "contact_forces_end_effector": contact_forces_ee,
                },
                ObservationGroup.PROPRIO: {
                    "fk_pos_end_effector": state.fk_pos_ee,
                    "fk_rot6d_end_effector": state.fk_rot6d_ee,
                },
                ObservationGroup.PROPRIO_DYN: {
                    "joint_pos_robot_normalized": state.joint_pos_robot_normalized,
                    "joint_pos_end_effector_normalized": state.joint_pos_ee_normalized,
                    "joint_acc_robot": state.joint_acc_robot,
                    "joint_applied_torque_robot": state.joint_applied_torque_robot,
                },
            },
            reward,
            termination,
            truncation,
        )
