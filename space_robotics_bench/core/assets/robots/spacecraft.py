from space_robotics_bench.core.actions import SpacecraftActionGroupCfg

from . import RobotCfg


class SpacecraftCfg(RobotCfg):
    class Config:
        arbitrary_types_allowed = True  # Due to SpacecraftActionGroupCfg

    ## Actions
    action_cfg: SpacecraftActionGroupCfg
