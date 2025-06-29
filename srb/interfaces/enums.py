from enum import Enum, auto
from typing import Type

from typing_extensions import Self


class InterfaceType(str, Enum):
    GUI = auto()
    ROS = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )

    def implementer(self) -> Type:
        match self:
            case InterfaceType.GUI:
                from .interface.gui import GuiInterface

                return GuiInterface
            case InterfaceType.ROS:
                from .interface.ros import RosInterface

                return RosInterface
            case _:
                raise NotImplementedError(f"Interface type {self} not implemented")


class TeleopDeviceType(str, Enum):
    KEYBOARD = auto()
    ROS = auto()
    GAMEPAD = auto()
    SPACEMOUSE = auto()
    HAPTIC = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )

    def implementer(self) -> Type:
        match self:
            case TeleopDeviceType.KEYBOARD:
                from .teleop.keyboard_omni import OmniKeyboardTeleopInterface

                return OmniKeyboardTeleopInterface
            case TeleopDeviceType.ROS | TeleopDeviceType.GAMEPAD:
                from .teleop.ros import ROSTeleopInterface

                return ROSTeleopInterface
            case TeleopDeviceType.SPACEMOUSE:
                from .teleop.spacemouse import SpacemouseTeleopInterface

                return SpacemouseTeleopInterface
            case TeleopDeviceType.HAPTIC:
                from .teleop.haptic import HapticROSTeleopInterface

                return HapticROSTeleopInterface
            case _:
                raise NotImplementedError(f"Teleop device {self} not implemented")
