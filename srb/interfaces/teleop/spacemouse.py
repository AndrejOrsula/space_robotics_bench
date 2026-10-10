from srb.interfaces.teleop import DeviceBase
from srb.interfaces.teleop._spacemouse_impl import SpacemouseImplementation


class SpacemouseTeleopInterface(SpacemouseImplementation, DeviceBase):
    """Isaac device facade; command behavior lives in the base-free owner."""
