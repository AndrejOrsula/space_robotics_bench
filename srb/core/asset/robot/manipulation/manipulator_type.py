from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class ManipulatorType(LowercaseNameEnum):
    SERIAL = auto()
    MULTI = auto()
