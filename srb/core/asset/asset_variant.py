from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class AssetVariant(LowercaseNameEnum):
    PRIMITIVE = auto()
    DATASET = auto()
    PROCEDURAL = auto()
