from space_robotics_bench.core.assets import Articulation
from space_robotics_bench.core.envs import BaseEnv

from .cfg import BaseSpacecraftRoboticsEnvCfg


class BaseSpacecraftRoboticsEnv(BaseEnv):
    cfg: BaseSpacecraftRoboticsEnvCfg

    def __init__(self, cfg: BaseSpacecraftRoboticsEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get handles to scene assets
        self._robot: Articulation = self.scene["robot"]
