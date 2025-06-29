from typing import Any, Dict, Sequence

import gymnasium
import numpy

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging


class TemplateInterfaceCfg(HardwareInterfaceCfg):
    name: str = ""


class TemplateInterface(HardwareInterface):
    cfg: TemplateInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = (
        ("robot/act1", "robot/cmd_vel"),
        ("obs_catA/obsAb", "state/tf_pos_robot"),
        ("obs_catB/obsB1", "state/tf_quat_robot"),
    )

    def __init__(self, cfg: TemplateInterfaceCfg = TemplateInterfaceCfg()):
        super().__init__(cfg)

        self.step_counter: int = 0
        self.should_reset: bool = False

    def start(self, **kwargs):
        super().start(**kwargs)

    def close(self):
        super().close()

    def sync(self):
        super().sync()

        self.extras = {}
        self.step_counter += 1
        if self.should_reset:
            self.should_reset = False
            self.step_counter = 0
            self.extras["ep_len"] = self.step_counter

        self.obs = {
            "obs_catA/obsA1": numpy.array(
                (1.1, 1.2),
                dtype=numpy.float32,
            ),
            "obs_catA/obsA2": numpy.array(
                (2.1, 2.2, 2.3),
                dtype=numpy.float32,
            ),
            "obs_catB/obsB1": numpy.array(
                (3.1, 3.2, 3.3, 3.4),
                dtype=numpy.float32,
            ),
        }
        self.rew = numpy.random.random()
        self.term = numpy.random.random() < (0.001 * self.step_counter)

    def reset(self):
        super().reset()
        self.should_reset = True

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "robot/act1": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(2,), dtype=numpy.float32
                )
            }
        )

    def apply_action(self, action: Dict[str, numpy.ndarray]):
        assert "robot/act1" in action.keys() and action["robot/act1"].shape == (2,)
        act = self.action_scale["robot/act1"] * action["robot/act1"]
        logging.debug(f'[{self.name}] Applying action: {{"robot/act1": {act}}}')

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        return self.obs

    @property
    def reward(self) -> float:
        return self.rew

    @property
    def termination(self) -> bool:
        return self.term

    @property
    def info(self) -> Dict[str, Any]:
        return self.extras
