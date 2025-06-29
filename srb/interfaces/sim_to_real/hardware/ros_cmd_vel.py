from functools import cached_property
from typing import Dict, Sequence

import gymnasium
import numpy

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging


class RosCmdVelInterfaceCfg(HardwareInterfaceCfg):
    topic: str = "cmd_vel"


class RosCmdVelInterface(HardwareInterface):
    cfg: RosCmdVelInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()

    def __init__(self, cfg: RosCmdVelInterfaceCfg = RosCmdVelInterfaceCfg()):
        super().__init__(cfg)
        from geometry_msgs.msg import Twist

        self.msg = Twist()

    def start(self, **kwargs):
        super().start(**kwargs)
        from geometry_msgs.msg import Twist
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        self.pub = self.ros_node.create_publisher(
            Twist,
            self.cfg.topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        logging.info(f"[{self.name}] Creating publisher under topic: {self.cfg.topic}")

    def close(self):
        super().close()

        self.pub.destroy()

    def reset(self):
        super().reset()

        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "robot/wheeled_drive": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(2,), dtype=numpy.float32
                )
            }
        )

    @cached_property
    def action_scale_linear(self) -> float:
        return self._action_scale.get(
            "robot/wheeled_drive_linear"
        ) or self._action_scale.get("robot/wheeled_drive", 1.0)

    @cached_property
    def action_scale_angular(self) -> float:
        return self._action_scale.get(
            "robot/wheeled_drive_angular"
        ) or self._action_scale.get("robot/wheeled_drive", 1.0)

    def apply_action(self, action: Dict[str, numpy.ndarray]):
        assert "robot/wheeled_drive" in action.keys() and action[
            "robot/wheeled_drive"
        ].shape == (2,)

        self.msg.linear.x = self.action_scale_linear * action["robot/wheeled_drive"][0]
        self.msg.angular.z = (
            self.action_scale_angular * action["robot/wheeled_drive"][1]
        )
        self.pub.publish(self.msg)
