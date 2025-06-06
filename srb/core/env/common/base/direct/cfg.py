import gymnasium
from isaaclab.envs import DirectRLEnvCfg as __DirectRLEnvCfg

from srb.utils.cfg import configclass

from ..env_cfg import BaseEnvCfg


@configclass
class DirectEnvCfg(BaseEnvCfg, __DirectRLEnvCfg):
    # Disable UI window by default
    ui_window_class_type: type | None = None

    # Temporarily set action_space, observation_space, and state_space (overridden by the implementation)
    action_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,))
    observation_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,))
    state_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,))

    def __post_init__(self):
        BaseEnvCfg.__post_init__(self)
