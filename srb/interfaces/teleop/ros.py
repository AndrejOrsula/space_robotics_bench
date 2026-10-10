try:
    import rclpy  # noqa: F401
except ImportError:
    from srb.utils.ros import enable_ros2_bridge

    enable_ros2_bridge()

from isaaclab.devices import DeviceBase
from rclpy.node import Node

from srb.interfaces.teleop._ros_impl import ROSTeleopInterfaceImplementation


class ROSTeleopInterface(ROSTeleopInterfaceImplementation, DeviceBase, Node):
    """Simulator compatibility facade retaining DeviceBase and Node identity."""
