from dataclasses import MISSING

from omni.isaac.lab.utils import configclass

from space_robotics_bench.core.actions import JointPositionActionCfg


@configclass
class LocomotionJointSpaceActionCfg:
    joint_pos: JointPositionActionCfg = MISSING
