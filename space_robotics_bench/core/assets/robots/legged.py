from space_robotics_bench.core.actions import LocomotionJointSpaceActionCfg

from . import RobotCfg


class LeggedRobotCfg(RobotCfg):
    ## Actions
    action_cfg: LocomotionJointSpaceActionCfg
