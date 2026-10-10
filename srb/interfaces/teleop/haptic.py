try:
    import rclpy  # noqa: F401
except ImportError:
    from srb.utils.ros import enable_ros2_bridge

    enable_ros2_bridge()

from isaaclab.devices import DeviceBase

from srb.interfaces.teleop._haptic_impl import HapticROSTeleopInterfaceImplementation


class HapticROSTeleopInterface(HapticROSTeleopInterfaceImplementation, DeviceBase):
    """Simulator compatibility facade retaining DeviceBase identity."""
