from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class MobileRobotType(LowercaseNameEnum):
    AERIAL = auto()
    AQUATIC = auto()
    GROUND = auto()
    ORBITAL = auto()
