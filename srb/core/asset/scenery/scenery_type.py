from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class SceneryType(LowercaseNameEnum):
    EXTRAVEHICULAR = auto()
    INTRAVEHICULAR = auto()
    TERRAIN = auto()
    SUBTERRANE = auto()
