from __future__ import annotations

from enum import Enum


class EnvCapability(str, Enum):
    SRB_NATIVE = "srb-native"
    ISAACLAB_NATIVE = "isaaclab-native"
    GYMNASIUM_NATIVE = "gymnasium-native"


__all__ = ["EnvCapability"]
