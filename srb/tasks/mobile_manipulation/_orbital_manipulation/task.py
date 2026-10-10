from srb._typing import StepReturn
from srb.core.env import (
    OrbitalManipulationEnv,
    OrbitalManipulationEnvCfg,
    OrbitalManipulationEventCfg,
    OrbitalManipulationSceneCfg,
)
from srb.core.env.mobile_manipulation import OrbitalManipulationRewardCfg
from srb.core.env.mobile_manipulation._template_task import (
    template_step_return,
)
from srb.core.env.srb_env import SrbEnv
from srb.utils.cfg import configclass

##############
### Config ###
##############


class RewardCfg(OrbitalManipulationRewardCfg):
    """Reward terms for the orbital mobile manipulation base task."""

    pass


@configclass
class SceneCfg(OrbitalManipulationSceneCfg):
    pass


@configclass
class EventCfg(OrbitalManipulationEventCfg):
    pass


@configclass
class TaskCfg(OrbitalManipulationEnvCfg):
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


class Task(SrbEnv, OrbitalManipulationEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

    def extract_step_return(self) -> StepReturn:
        # Shared by the three mobile-manipulation templates.
        return template_step_return(self)
