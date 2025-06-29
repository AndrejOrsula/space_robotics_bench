from typing import TYPE_CHECKING, Dict

import numpy

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging

if TYPE_CHECKING:
    from sensor_msgs.msg import Imu


class RosImuCfg(HardwareInterfaceCfg):
    topic_name: str = "imu/data"


class RosImu(HardwareInterface):
    cfg: RosImuCfg

    def __init__(self, cfg: RosImuCfg = RosImuCfg()):
        super().__init__(cfg)

        self.obs_lin_acc = numpy.zeros(3, dtype=numpy.float32)
        self.obs_ang_vel = numpy.zeros(3, dtype=numpy.float32)

    def start(self, **kwargs):
        super().start(**kwargs)
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import Imu

        self.sub = self.ros_node.create_subscription(
            Imu,
            self.cfg.topic_name,
            self._imu_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        logging.info(f"[{self.name}] Subscribing to topic: {self.cfg.topic_name}")

    def close(self):
        super().close()
        self.sub.destroy()

    def reset(self):
        super().reset()
        self.obs_lin_acc = numpy.zeros(3, dtype=numpy.float32)
        self.obs_ang_vel = numpy.zeros(3, dtype=numpy.float32)

    def _imu_callback(self, msg: "Imu"):
        self.obs_lin_acc = numpy.array(
            (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
            dtype=numpy.float32,
        )
        self.obs_ang_vel = numpy.array(
            (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ),
            dtype=numpy.float32,
        )

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        return {
            "proprio/imu_ang_vel": self.obs_ang_vel,
            "proprio/imu_lin_acc": self.obs_lin_acc,
        }
