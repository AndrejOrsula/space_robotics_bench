from omni.isaac.lab.sensors import ContactSensor, RayCaster

from space_robotics_bench.core.assets import Articulation
from space_robotics_bench.core.envs import BaseEnv

from .cfg import BaseLocomotionEnvCfg


class BaseLocomotionEnv(BaseEnv):
    cfg: BaseLocomotionEnvCfg

    def __init__(self, cfg: BaseLocomotionEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get handles to scene assets
        self._robot: Articulation = self.scene["robot"]
        self._contacts_robot: ContactSensor = self.scene["contacts_robot"]
        self._height_scanner: RayCaster = self.scene["height_scanner"]
