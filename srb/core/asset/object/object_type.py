from enum import auto

from srb.core.algorithms import LowercaseNameEnum


class ObjectType(LowercaseNameEnum):
    COMMON = auto()
    LIGHT = auto()
    MODULAR_ELEMENT = auto()
    PAYLOAD = auto()
    PEDESTAL = auto()
    TOOL = auto()
