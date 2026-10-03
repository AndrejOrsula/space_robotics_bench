from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class AssetType(LowercaseNameEnum):
    OBJECT = auto()
    ROBOT = auto()
    SCENERY = auto()
