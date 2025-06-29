from typing import TYPE_CHECKING

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging

if TYPE_CHECKING:
    from std_msgs.msg import Float32
    from std_srvs.srv import Trigger


class RosMwCfg(HardwareInterfaceCfg):
    termination_service: str = "termination"
    reward_topic: str = "reward"
    pause_service: str = "pause"
    resume_service: str = "resume"


class RosMw(HardwareInterface):
    cfg: RosMwCfg

    def __init__(self, cfg: RosMwCfg = RosMwCfg()):
        super().__init__(cfg)

        self._reward = 0.0
        self._terminated = False
        self._pause_signal = False
        self._resume_signal = False

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

        # Create service for termination
        self.termination_srv = self.ros_node.create_service(
            Trigger, self.cfg.termination_service, self._termination_srv_callback
        )
        logging.info(
            f"[{self.name}] Created termination service: {self.cfg.termination_service}"
        )

        # Create service for pause/resume
        self.pause_srv = self.ros_node.create_service(
            Trigger, self.cfg.pause_service, self._pause_srv_callback
        )
        logging.info(f"[{self.name}] Created pause service: {self.cfg.pause_service}")

        # Create service for resume
        self.resume_srv = self.ros_node.create_service(
            Trigger, self.cfg.resume_service, self._resume_srv_callback
        )
        logging.info(f"[{self.name}] Created resume service: {self.cfg.resume_service}")

    def close(self):
        super().close()
        self.reward_sub.destroy()
        self.termination_srv.destroy()
        self.pause_srv.destroy()
        self.resume_srv.destroy()

    def reset(self):
        super().reset()
        self._reward = 0.0
        self._terminated = False
        self._pause_signal = False
        self._resume_signal = False

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

    @property
    def pause_signal(self) -> bool:
        value = self._pause_signal
        self._pause_signal = False
        return value

    @property
    def resume_signal(self) -> bool:
        value = self._resume_signal
        self._resume_signal = False
        return value

    def _termination_srv_callback(
        self, request: "Trigger.Request", response: "Trigger.Response"
    ):
        self._terminated = True
        response.success = True
        logging.info(f"[{self.name}] Termination service called")
        return response

    def _pause_srv_callback(
        self, request: "Trigger.Request", response: "Trigger.Response"
    ):
        self._pause_signal = True
        response.success = True
        logging.info(f"[{self.name}] Pause service called")
        return response

    def _resume_srv_callback(
        self, request: "Trigger.Request", response: "Trigger.Response"
    ):
        self._resume_signal = True
        response.success = True
        logging.info(f"[{self.name}] Resume service called")
        return response

    def _reward_callback(self, msg: "Float32"):
        self._reward = msg.data
        logging.debug(f"[{self.name}] Reward signal received: {self._reward}")
