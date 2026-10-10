from typing import Any, Sequence

import gymnasium
import numpy as np

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.utils import logging


class TemplateInterfaceCfg(HardwareInterfaceCfg):
    name: str = ""


class TemplateInterface(HardwareInterface):
    cfg: TemplateInterfaceCfg
    SUPPORTS_REWARD = True
    SUPPORTS_TERMINATION = True
    CUSTOM_ALIASES: Sequence[Sequence[str]] = (
        ("robot/act1", "robot/cmd_vel"),
        ("obs_catA/obsAb", "state/tf_pos_robot"),
        ("obs_catB/obsB1", "state/tf_quat_robot"),
    )

    def __init__(self, cfg: TemplateInterfaceCfg | None = None):
        if cfg is None:
            cfg = TemplateInterfaceCfg()
        super().__init__(cfg)

        self.step_counter: int = 0
        self.should_reset: bool = False

    def _safe_stop(self):
        # Adapters must send a physical stop before latching the pause.
        super()._safe_stop()

    def sync(self):
        super().sync()

        self.extras = {}
        self.step_counter += 1
        if self.should_reset:
            self.should_reset = False
            self.step_counter = 0
            self.extras["ep_len"] = self.step_counter

        self.obs = {
            "obs_catA/obsA1": np.array(
                (1.1, 1.2),
                dtype=np.float32,
            ),
            "obs_catA/obsA2": np.array(
                (2.1, 2.2, 2.3),
                dtype=np.float32,
            ),
            "obs_catB/obsB1": np.array(
                (3.1, 3.2, 3.3, 3.4),
                dtype=np.float32,
            ),
        }
        self.rew = np.random.random()
        self.term = np.random.random() < (0.001 * self.step_counter)

    def reset(self):
        super().reset()
        self.should_reset = True

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "robot/act1": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(2,), dtype=np.float32
                )
            }
        )

    def apply_action(self, action: dict[str, np.ndarray]):
        if "robot/act1" not in action or action["robot/act1"].shape != (2,):
            raise ValueError("robot/act1 must be a shape-(2,) action")
        act = self.action_scale["robot/act1"] * action["robot/act1"]
        logging.debug(f'[{self.name}] Applying action: {{"robot/act1": {act}}}')

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(getattr(self, "obs", {}))

    @property
    def reward(self) -> float:
        return self.rew

    @property
    def termination(self) -> bool:
        return self.term

    @property
    def info(self) -> dict[str, Any]:
        return self.extras
