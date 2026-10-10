from srb._typing import StepReturn
from srb.core.env import (
    GroundManipulationEnv,
    GroundManipulationEnvCfg,
    GroundManipulationEventCfg,
    GroundManipulationSceneCfg,
)
from srb.core.env.mobile_manipulation import GroundManipulationRewardCfg
from srb.core.env.mobile_manipulation._template_task import (
    template_step_return,
)
from srb.core.env.srb_env import SrbEnv
from srb.utils.cfg import configclass

##############
### Config ###
##############


class RewardCfg(GroundManipulationRewardCfg):
    """Reward terms for the ground mobile manipulation base task."""

    pass


@configclass
class SceneCfg(GroundManipulationSceneCfg):
    pass


@configclass
class EventCfg(GroundManipulationEventCfg):
    pass


@configclass
class TaskCfg(GroundManipulationEnvCfg):
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


class Task(SrbEnv, GroundManipulationEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

    def extract_step_return(self) -> StepReturn:
        # Shared by the three mobile-manipulation templates.
        return template_step_return(self)
