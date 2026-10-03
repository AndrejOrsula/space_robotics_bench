"""Configuration dataclass for :class:`~srb.core.env.DirectEnv`.

Extends Isaac Lab's ``DirectRLEnvCfg`` with SRB-specific defaults for
environment decimation, episode length, and Gymnasium space construction.
"""

import gymnasium
import numpy as np
from isaaclab.envs import DirectRLEnvCfg as __DirectRLEnvCfg

from srb.core.env.runtime.delay_config import validate_delay_steps
from srb.utils.cfg import configclass

from ..env_cfg import BaseEnvCfg


@configclass
class DirectEnvCfg(BaseEnvCfg, __DirectRLEnvCfg):
    # Disable UI window by default
    ui_window_class_type: type | None = None

    # Temporarily set action_space, observation_space, and state_space (overridden by the implementation)
    action_space = gymnasium.spaces.Box(
        low=-1.0, high=1.0, shape=(1,), dtype=np.float32
    )
    observation_space = gymnasium.spaces.Box(
        low=-1.0, high=1.0, shape=(1,), dtype=np.float32
    )
    state_space = gymnasium.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

    # Action/observation delay
    action_delay_steps: int | tuple[int, int] | list[int] = 0
    action_delay_on_step_change_freq: float = 1.0
    action_delay_on_step_change_prob: float = 0.01
    observation_delay_steps: int | tuple[int, int] | list[int] = 0
    observation_delay_on_step_change_freq: float = 1.0
    observation_delay_on_step_change_prob: float = 0.01

    _validate_delay_steps = staticmethod(validate_delay_steps)

    def __post_init__(self):
        BaseEnvCfg.__post_init__(self)
        self._validate_delay_steps("action_delay_steps", self.action_delay_steps)
        self._validate_delay_steps(
            "observation_delay_steps", self.observation_delay_steps
        )
