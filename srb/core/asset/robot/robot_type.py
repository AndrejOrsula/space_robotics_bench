from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class RobotType(LowercaseNameEnum):
    MANIPULATOR = auto()
    MOBILE_MANIPULATOR = auto()
    MOBILE_ROBOT = auto()
