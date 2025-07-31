from typing import Tuple

import gymnasium
from isaaclab.envs import DirectRLEnvCfg as __DirectRLEnvCfg

from srb.utils.cfg import configclass

from ..env_cfg import BaseEnvCfg


@configclass
class DirectEnvCfg(BaseEnvCfg, __DirectRLEnvCfg):
    # Disable UI window by default
    ui_window_class_type: type | None = None

    # Temporarily set action_space, observation_space, and state_space (overridden by the implementation)
    # TODO[low]: Clean-up DirectEnv patch
    action_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,))
    observation_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,))
    state_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,))

    # Action/observation delay
    action_delay_steps: int | Tuple[int, int] = 0
    action_delay_on_step_change_freq: float = 1.0
    action_delay_on_step_change_prob: float = 0.01
    observation_delay_steps: int | Tuple[int, int] = 0
    observation_delay_on_step_change_freq: float = 1.0
    observation_delay_on_step_change_prob: float = 0.01

    def __post_init__(self):
        BaseEnvCfg.__post_init__(self)
