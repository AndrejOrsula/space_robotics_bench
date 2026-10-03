from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class MobileManipulatorType(LowercaseNameEnum):
    HUMANOID = auto()
    GROUND = auto()
    AERIAL = auto()
    ORBITAL = auto()
