from typing import Dict

import torch

from srb.core.env import GroundEnvVisualExtCfg, VisualExt
from srb.utils.cfg import configclass

from .task import Task, TaskCfg
from .task_locomotion import LocomotionTask, LocomotionTaskCfg


@configclass
class VisualTaskCfg(GroundEnvVisualExtCfg, TaskCfg):
    def __post_init__(self):
        TaskCfg.__post_init__(self)
        GroundEnvVisualExtCfg.wrap(self, env_cfg=self)  # type: ignore


class VisualTask(VisualExt, Task):
    cfg: VisualTaskCfg

    def __init__(self, cfg: VisualTaskCfg, **kwargs):
        Task.__init__(self, cfg, **kwargs)
        VisualExt.__init__(self, cfg, **kwargs)

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {
            **Task._get_observations(self),
            **VisualExt._get_observations(self),
        }


@configclass
class VisualLocomotionTaskCfg(GroundEnvVisualExtCfg, LocomotionTaskCfg):
    def __post_init__(self):
        LocomotionTaskCfg.__post_init__(self)
        GroundEnvVisualExtCfg.wrap(self, env_cfg=self)  # type: ignore


class VisualLocomotionTask(VisualExt, LocomotionTask):
    cfg: VisualLocomotionTaskCfg

    def __init__(self, cfg: VisualLocomotionTaskCfg, **kwargs):
        LocomotionTask.__init__(self, cfg, **kwargs)
        VisualExt.__init__(self, cfg, **kwargs)

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {
            **LocomotionTask._get_observations(self),
            **VisualExt._get_observations(self),
        }
