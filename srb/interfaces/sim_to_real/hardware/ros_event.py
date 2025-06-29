from typing import TYPE_CHECKING

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging

if TYPE_CHECKING:
    from std_msgs.msg import Float32
    from std_srvs.srv import Trigger


class RosEventCfg(HardwareInterfaceCfg):
    termination_service: str = "termination"
    reward_topic: str = "reward"


class RosEvent(HardwareInterface):
    cfg: RosEventCfg

    def __init__(self, cfg: RosEventCfg = RosEventCfg()):
        super().__init__(cfg)

        self._terminated = False
        self._reward = 0.0

    def start(self, **kwargs):
        super().start(**kwargs)
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import Float32
        from std_srvs.srv import Trigger

        # Create service for termination
        self.termination_srv = self.ros_node.create_service(
            Trigger, self.cfg.termination_service, self._termination_srv_callback
        )
        logging.info(
            f"[{self.name}] Created termination service: {self.cfg.termination_service}"
        )

        # Create subscription for reward
        self.reward_sub = self.ros_node.create_subscription(
            Float32,
            self.cfg.reward_topic,
            self._reward_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        logging.info(
            f"[{self.name}] Subscribing to reward topic: {self.cfg.reward_topic}"
        )

    def close(self):
        super().close()
        self.termination_srv.destroy()
        self.reward_sub.destroy()

    def reset(self):
        super().reset()
        self._terminated = False
        self._reward = 0.0

    @property
    def termination(self) -> bool:
        value = self._terminated
        self._terminated = False
        return value

    @property
    def reward(self) -> float:
        value = self._reward
        self._reward = 0.0
        return value

    def _termination_srv_callback(
        self, request: "Trigger.Request", response: "Trigger.Response"
    ):
        self._terminated = True
        response.success = True
        logging.info(f"[{self.name}] Termination service called")
        return response

    def _reward_callback(self, msg: "Float32"):
        self._reward = msg.data
        logging.debug(f"[{self.name}] Reward signal received: {self._reward}")
