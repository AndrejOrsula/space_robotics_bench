import threading
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

import gymnasium
import numpy as np
from pydantic import Field

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.utils import logging
from srb.utils.failures import (
    FailureCategory,
    FailureInfo,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
)

if TYPE_CHECKING:
    from sensor_msgs.msg import Imu


class _ImuSampleError(ValueError):
    """Validation failure retained as typed health evidence by the callback."""

    def __init__(
        self,
        *,
        category: FailureCategory,
        code: str,
        message: str,
        field_name: str,
        value_type: str,
    ) -> None:
        super().__init__(message)
        self.category = category
        self.code = code
        self.message = message
        self.field_name = field_name
        self.value_type = value_type


class RosImuInterfaceCfg(HardwareInterfaceCfg):
    topic_name: str = "imu/data"
    max_sample_age_s: float = Field(default=0.5, gt=0.0, allow_inf_nan=False)


class RosImuInterface(HardwareInterface):
    cfg: RosImuInterfaceCfg

    def __init__(
        self,
        cfg: RosImuInterfaceCfg | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ):
        if cfg is None:
            cfg = RosImuInterfaceCfg()
        super().__init__(cfg)
        self._clock = clock
        self._state_lock = threading.Lock()

        self.obs_lin_acc = np.zeros(3, dtype=np.float32)
        self.obs_ang_vel = np.zeros(3, dtype=np.float32)
        self._message_received = False
        self._last_sample_s: float | None = None
        self._last_failure: FailureInfo | None = None
        self.sub: Any | None = None

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
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        logging.info(f"[{self.name}] Subscribing to topic: {self.cfg.topic_name}")

    def close(self):
        super().close()
        # ``start`` may have failed before the subscription existed.
        sub = self.sub
        if sub is not None:
            sub.destroy()
            self.sub = None

    def reset(self):
        super().reset()
        with self._state_lock:
            self.obs_lin_acc = np.zeros(3, dtype=np.float32)
            self.obs_ang_vel = np.zeros(3, dtype=np.float32)
            self._message_received = False
            self._last_sample_s = None
            self._last_failure = None

    def _imu_callback(self, msg: "Imu"):
        with self._state_lock:
            self._process_imu_message(msg)

    def _process_imu_message(self, msg: "Imu"):
        try:
            lin_acc = self._validated_vector(
                getattr(msg, "linear_acceleration", None),
                field_name="linear_acceleration",
            )
            ang_vel = self._validated_vector(
                getattr(msg, "angular_velocity", None),
                field_name="angular_velocity",
            )
        except _ImuSampleError as exc:
            self._last_failure = FailureInfo(
                category=exc.category,
                code=exc.code,
                message=exc.message,
                component=self.name,
                details={
                    "topic": self.cfg.topic_name,
                    "field": exc.field_name,
                    "value_type": exc.value_type,
                },
                retryable=False,
                fallback_allowed=False,
                evidence_valid=False,
            )
            logging.error("[%s] rejected IMU sample: %s", self.name, exc.message)
            return
        except Exception as exc:  # noqa: BLE001 - ROS callback boundary
            self._last_failure = FailureInfo(
                category=FailureCategory.INTERNAL_INVARIANT,
                code="ros.imu.callback_invariant",
                message=f"IMU callback validation failed: {exc}",
                component=self.name,
                details={
                    "topic": self.cfg.topic_name,
                    "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
                    "cause_message": str(exc) or type(exc).__name__,
                },
                retryable=False,
                fallback_allowed=False,
                evidence_valid=False,
            )
            logging.error("[%s] rejected IMU sample: %s", self.name, exc)
            return

        self.obs_lin_acc = lin_acc
        self.obs_ang_vel = ang_vel
        self._message_received = True
        self._last_sample_s = self._clock()
        # Recovery is explicit: only a later valid sample clears the latest
        # sample failure, and it replaces both observation vectors atomically.
        self._last_failure = None

    @staticmethod
    def _validated_vector(value: object, *, field_name: str) -> np.ndarray:
        components = []
        for axis in ("x", "y", "z"):
            try:
                component = getattr(value, axis)
            except AttributeError as exc:
                raise _ImuSampleError(
                    category=FailureCategory.INVALID_INPUT,
                    code="ros.imu.sample_malformed",
                    message=f"{field_name} is missing component {axis}",
                    field_name=field_name,
                    value_type=type(value).__name__,
                ) from exc
            if isinstance(component, (bool, np.bool_)):
                raise _ImuSampleError(
                    category=FailureCategory.INVALID_INPUT,
                    code="ros.imu.sample_malformed",
                    message=f"{field_name}.{axis} must be numeric, not bool",
                    field_name=f"{field_name}.{axis}",
                    value_type=type(component).__name__,
                )
            try:
                scalar = np.asarray(component)
            except Exception as exc:  # pragma: no cover - hostile adapter value
                raise _ImuSampleError(
                    category=FailureCategory.INVALID_INPUT,
                    code="ros.imu.sample_malformed",
                    message=f"{field_name}.{axis} is not scalar numeric data",
                    field_name=f"{field_name}.{axis}",
                    value_type=type(component).__name__,
                ) from exc
            if scalar.ndim != 0 or scalar.dtype.kind not in "iuf":
                raise _ImuSampleError(
                    category=FailureCategory.INVALID_INPUT,
                    code="ros.imu.sample_malformed",
                    message=f"{field_name}.{axis} is not scalar numeric data",
                    field_name=f"{field_name}.{axis}",
                    value_type=type(component).__name__,
                )
            try:
                number = float(scalar)
            except (OverflowError, TypeError, ValueError) as exc:
                raise _ImuSampleError(
                    category=FailureCategory.INVALID_INPUT,
                    code="ros.imu.sample_malformed",
                    message=f"{field_name}.{axis} cannot be converted to float",
                    field_name=f"{field_name}.{axis}",
                    value_type=type(component).__name__,
                ) from exc
            if not np.isfinite(number):
                raise _ImuSampleError(
                    category=FailureCategory.INTEGRITY_FAILURE,
                    code="ros.imu.sample_nonfinite",
                    message=f"{field_name}.{axis} is non-finite",
                    field_name=f"{field_name}.{axis}",
                    value_type=type(component).__name__,
                )
            components.append(number)

        result = np.asarray(components, dtype=np.float32)
        if not np.all(np.isfinite(result)):
            raise _ImuSampleError(
                category=FailureCategory.INTEGRITY_FAILURE,
                code="ros.imu.sample_nonfinite",
                message=f"{field_name} overflows float32",
                field_name=field_name,
                value_type="float32",
            )
        return result

    def health(self) -> HealthEvent:
        """Report readiness from a real IMU message, including zero-valued data."""
        with self._state_lock:
            failure = self._last_failure
            last_sample_s = self._last_sample_s
            message_received = self._message_received
        age_s = (
            None if last_sample_s is None else max(0.0, self._clock() - last_sample_s)
        )
        if failure is not None:
            state = "failure"
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            code = failure.code
            message = f"latest IMU sample was rejected: {failure.message}"
        elif age_s is not None and age_s > self.cfg.max_sample_age_s:
            state = "stale"
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            code = f"{self.name}.{state}"
            message = "latest IMU sample is stale"
        else:
            state = "healthy" if message_received else "pre_message"
            status = (
                HealthStatus.HEALTHY if message_received else HealthStatus.UNHEALTHY
            )
            severity = HealthSeverity.INFO if message_received else HealthSeverity.ERROR
            code = f"{self.name}.{state}"
            message = (
                "IMU message received"
                if message_received
                else "waiting for the first IMU message"
            )
        return HealthEvent(
            status=status,
            severity=severity,
            component=self.name,
            code=code,
            message=message,
            details={
                "state": state,
                "topic": self.cfg.topic_name,
                "required_messages": 1,
                "received_messages": 1 if message_received else 0,
                "ready": status is HealthStatus.HEALTHY,
                "age_ms": None if age_s is None else age_s * 1000.0,
                "max_sample_age_ms": self.cfg.max_sample_age_s * 1000.0,
            },
            failure=failure,
        )

    @property
    def observation(self) -> dict[str, np.ndarray]:
        with self._state_lock:
            return {
                "proprio/imu_ang_vel": self.obs_ang_vel.copy(),
                "proprio/imu_lin_acc": self.obs_lin_acc.copy(),
            }

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self.observation)
