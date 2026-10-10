import math
from threading import Lock
from typing import TYPE_CHECKING, Any

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging
from srb.utils.failures import HealthEvent, HealthSeverity, HealthStatus

if TYPE_CHECKING:
    from std_msgs.msg import Float32
    from std_srvs.srv import Trigger


class RosMwInterfaceCfg(HardwareInterfaceCfg):
    termination_service: str = "termination"
    reward_topic: str = "reward"
    pause_service: str = "pause"
    resume_service: str = "resume"


class RosMwInterface(HardwareInterface):
    cfg: RosMwInterfaceCfg
    SUPPORTS_REWARD = True
    SUPPORTS_TERMINATION = True
    SUPPORTS_PAUSE = True
    SUPPORTS_RESUME = True

    def __init__(self, cfg: RosMwInterfaceCfg | None = None):
        if cfg is None:
            cfg = RosMwInterfaceCfg()
        super().__init__(cfg)

        # Callbacks run on executor threads. The lock protects all signal
        # state; pause remains latched until resume, and callback order wins.
        self._signal_lock = Lock()
        self._reward = 0.0
        self._terminated = False
        self._pause_signal = False
        self._resume_signal = False
        self._reward_rejected: dict[str, Any] | None = None
        self.reward_sub: Any | None = None
        self.termination_srv: Any | None = None
        self.pause_srv: Any | None = None
        self.resume_srv: Any | None = None

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

        # Reward subscription.
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

        # Termination service.
        self.termination_srv = self.ros_node.create_service(
            Trigger, self.cfg.termination_service, self._termination_srv_callback
        )
        logging.info(
            f"[{self.name}] Created termination service: {self.cfg.termination_service}"
        )

        # Pause service.
        self.pause_srv = self.ros_node.create_service(
            Trigger, self.cfg.pause_service, self._pause_srv_callback
        )
        logging.info(f"[{self.name}] Created pause service: {self.cfg.pause_service}")

        # Resume service.
        self.resume_srv = self.ros_node.create_service(
            Trigger, self.cfg.resume_service, self._resume_srv_callback
        )
        logging.info(f"[{self.name}] Created resume service: {self.cfg.resume_service}")

    def close(self):
        super().close()
        # Retain failed handles so a later close() can retry cleanup.
        errors: list[Exception] = []
        for attr in ("reward_sub", "termination_srv", "pause_srv", "resume_srv"):
            endpoint = getattr(self, attr, None)
            if endpoint is None:
                continue
            try:
                endpoint.destroy()
            except (AttributeError, OSError, RuntimeError, TypeError) as exc:
                errors.append(exc)
            else:
                setattr(self, attr, None)
        if len(errors) == 1:
            raise errors[0]
        if errors:
            raise ExceptionGroup(f"[{self.name}] endpoint cleanup failed", errors)

    def reset(self):
        super().reset()
        with self._signal_lock:
            self._reward = 0.0
            self._terminated = False
            self._reward_rejected = None
            # Pause and pending resume survive reset; only resume clears pause.

    def health(self) -> HealthEvent | None:
        with self._signal_lock:
            rejected = (
                None if self._reward_rejected is None else dict(self._reward_rejected)
            )
        if rejected is None:
            return None
        return HealthEvent(
            status=HealthStatus.UNHEALTHY,
            severity=HealthSeverity.ERROR,
            component=self.name,
            code=f"{self.name}.reward_non_finite",
            message=(
                "a non-finite reward sample was rejected; the episode reward "
                "cannot be trusted until reset"
            ),
            details={"state": "reward_non_finite", **rejected},
        )

    @property
    def termination(self) -> bool:
        with self._signal_lock:
            value = self._terminated
            self._terminated = False
        return value

    @property
    def reward(self) -> float:
        with self._signal_lock:
            value = self._reward
            self._reward = 0.0
        return value

    @property
    def pause_signal(self) -> bool:
        """Whether an operator pause is latched (reading does not clear it)."""
        with self._signal_lock:
            return self._pause_signal

    @property
    def resume_signal(self) -> bool:
        with self._signal_lock:
            value = self._resume_signal
            self._resume_signal = False
        return value

    def _termination_srv_callback(
        self, request: "Trigger.Request", response: "Trigger.Response"
    ):
        with self._signal_lock:
            self._terminated = True
        response.success = True
        logging.info(f"[{self.name}] Termination service called")
        return response

    def _pause_srv_callback(
        self, request: "Trigger.Request", response: "Trigger.Response"
    ):
        with self._signal_lock:
            self._pause_signal = True
            self._resume_signal = False
        response.success = True
        logging.info(f"[{self.name}] Pause service called")
        return response

    def _resume_srv_callback(
        self, request: "Trigger.Request", response: "Trigger.Response"
    ):
        with self._signal_lock:
            self._pause_signal = False
            self._resume_signal = True
        response.success = True
        logging.info(f"[{self.name}] Resume service called")
        return response

    def _reward_callback(self, msg: "Float32"):
        try:
            value = float(msg.data)
        except (TypeError, ValueError):
            value = math.nan
        with self._signal_lock:
            if not math.isfinite(value):
                # Reject the sample and latch unhealthy state; dispatch stops.
                rejected = self._reward_rejected or {"rejected_samples": 0}
                self._reward_rejected = {
                    "rejected_samples": int(rejected["rejected_samples"]) + 1,
                    "last_value": repr(msg.data),
                }
            else:
                self._reward = value
        if math.isfinite(value):
            logging.debug(f"[{self.name}] Reward signal received: {value}")
        else:
            logging.warning(
                f"[{self.name}] Rejected non-finite reward sample: {msg.data!r}"
            )
