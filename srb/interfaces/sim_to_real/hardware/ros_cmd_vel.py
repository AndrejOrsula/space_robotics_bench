from collections.abc import Sequence
from math import isfinite

import gymnasium
import numpy as np
import torch

from srb.core.action_filters import CriticallyDampedFilter, RateLimiter
from srb.interfaces.sim_to_real.core.hardware import (
    HardwareDispatchNotReadyError,
    HardwareInterface,
    HardwareInterfaceCfg,
    PreparedHardwareAction,
)
from srb.utils import logging
from srb.utils.failures import SafetyViolationError


class _Vector3Buffer:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _TwistBuffer:
    """ROS-free command buffer used until ``start()`` creates a Twist."""

    def __init__(self) -> None:
        self.linear = _Vector3Buffer()
        self.angular = _Vector3Buffer()


class RosCmdVelInterfaceCfg(HardwareInterfaceCfg):
    topic: str = "cmd_vel"
    filter_enabled: bool = False
    """Apply the filtered-drive sim-to-real chain before publication."""
    v_lin_max: float = 1.0
    """Normalized linear action scale in physical metres per second."""
    v_ang_max: float = 1.0
    """Normalized angular action scale in physical radians per second."""
    accel_lin_max: float = 2.0
    """Linear acceleration bound used by the per-step rate limiter."""
    accel_ang_max: float = 2.0
    """Angular acceleration bound used by the per-step rate limiter."""
    filter_tau_s: float = 0.1
    """Critically damped filter time constant in seconds."""
    max_linear_velocity: float | None = None
    """Optional linear velocity clamp [m/s], applied last to every published
    command, with or without filtering (post-filter bound)."""
    max_angular_velocity: float | None = None
    """Optional angular velocity clamp [rad/s], applied last to every published
    command, with or without filtering (post-filter bound)."""


class RosCmdVelInterface(HardwareInterface):
    cfg: RosCmdVelInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()

    def __init__(self, cfg: RosCmdVelInterfaceCfg | None = None):
        cfg = cfg if cfg is not None else RosCmdVelInterfaceCfg()
        super().__init__(cfg)

        self.msg = _TwistBuffer()
        self._rate_limiter: RateLimiter | None = None
        self._smooth_filter: CriticallyDampedFilter | None = None
        self._emitted = np.zeros((2,), dtype=np.float32)
        self._lifecycle_operation: str | None = None
        self._safe_stop_in_progress = False
        self._allow_close_stop = False
        self._allow_internal_pause = False

    def start(self, **kwargs):
        self._reject_lifecycle_reentry("start")
        super().start(**kwargs)
        if self.cfg.filter_enabled:
            self._build_filters(float(self._action_rate))
        from geometry_msgs.msg import Twist
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        self.msg = Twist()
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

    def _reject_lifecycle_reentry(self, operation: str) -> None:
        if self._lifecycle_operation is not None:
            raise HardwareDispatchNotReadyError(
                self.name,
                f"{operation} re-entered during {self._lifecycle_operation}",
            )

    def _begin_lifecycle(self, operation: str) -> None:
        self._reject_lifecycle_reentry(operation)
        self._lifecycle_operation = operation

    def resume(self):
        self._reject_lifecycle_reentry("resume")
        return super().resume()

    def safe_stop(self) -> None:
        if self._safe_stop_in_progress:
            raise HardwareDispatchNotReadyError(
                self.name, "safe stop re-entered during transport stop"
            )
        if self._lifecycle_operation is not None and not self._allow_close_stop:
            raise HardwareDispatchNotReadyError(
                self.name,
                f"safe stop re-entered during {self._lifecycle_operation}",
            )
        self._safe_stop_in_progress = True
        try:
            super().safe_stop()
        finally:
            self._safe_stop_in_progress = False

    def _safe_stop(self) -> None:
        owns_operation = self._lifecycle_operation is None
        if owns_operation:
            self._lifecycle_operation = "safe_stop"
        self._allow_internal_pause = True
        try:
            super()._safe_stop()
        finally:
            self._allow_internal_pause = False
            if owns_operation:
                self._lifecycle_operation = None

    def close(self):
        # Invalidate staged commands first. Keep ``super().close`` last so a
        # failed stop/destroy remains retryable.
        self._begin_lifecycle("close")
        try:
            self._action_revision += 1
            pub = getattr(self, "pub", None)
            if not self.safe_stop_issued and (
                pub is not None or self._safe_stop_requested
            ):
                self._allow_close_stop = True
                try:
                    self.safe_stop()
                finally:
                    self._allow_close_stop = False
            else:
                self._reset_command_state()

            if pub is not None:
                try:
                    pub.destroy()
                except Exception as exc:  # noqa: BLE001 - retain for bounded retry
                    logging.warning(f"[{self.name}] Failed to destroy publisher: {exc}")
                    raise
                else:
                    self.pub = None
            super().close()
        finally:
            self._allow_close_stop = False
            self._lifecycle_operation = None

    def reset(self):
        self._begin_lifecycle("reset")
        try:
            super().reset()
            self._reset_command_state()
            self._publish_stop(required=self.is_started)
        finally:
            self._lifecycle_operation = None

    def pause(self):
        internal = self._allow_internal_pause
        if internal:
            self._allow_internal_pause = False
        else:
            self._begin_lifecycle("pause")
        try:
            super().pause()
            self._reset_command_state()
            self._publish_stop(required=self.is_started)
        finally:
            if not internal:
                self._lifecycle_operation = None

    def _reset_command_state(self) -> None:
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0
        self._emitted.fill(0.0)
        if self._rate_limiter is not None:
            zeros = torch.zeros((2,), dtype=torch.float32)
            self._rate_limiter.reset(None, zeros)
        if self._smooth_filter is not None:
            zeros = torch.zeros((2,), dtype=torch.float32)
            self._smooth_filter.reset(None, zeros)

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        action_key = f"{self._action_namespace(self.cfg.topic)}/wheeled_drive"
        return gymnasium.spaces.Dict(
            {
                action_key: gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(2,), dtype=np.float32
                )
            }
        )

    @staticmethod
    def _action_namespace(topic: str = "") -> str:
        cleaned = topic.strip("/")
        if not cleaned or "/" not in cleaned:
            return "robot"
        return cleaned.rsplit("/", 1)[0]

    @property
    def action_scale_linear(self) -> float:
        robot_key = f"{self._action_namespace(self.cfg.topic)}/wheeled_drive"
        return self._action_scale.get(
            f"{robot_key}_linear",
            self._action_scale.get(robot_key, 1.0),
        )

    @property
    def action_scale_angular(self) -> float:
        robot_key = f"{self._action_namespace(self.cfg.topic)}/wheeled_drive"
        return self._action_scale.get(
            f"{robot_key}_angular",
            self._action_scale.get(robot_key, 1.0),
        )

    @staticmethod
    def _copy_rate_limiter(rate_limiter: RateLimiter) -> RateLimiter:
        preview = RateLimiter(
            rate_limiter.shape,
            max_rate_per_step=rate_limiter.max_rate.clone(),
            device=rate_limiter.device,
        )
        preview.pos = rate_limiter.pos.clone()
        return preview

    @staticmethod
    def _copy_smooth_filter(
        smooth_filter: CriticallyDampedFilter,
    ) -> CriticallyDampedFilter:
        preview = CriticallyDampedFilter(
            smooth_filter.shape,
            tau_s=smooth_filter.tau_s,
            dt_s=smooth_filter.dt_s,
            device=smooth_filter.device,
        )
        preview.pos = smooth_filter.pos.clone()
        preview.vel = smooth_filter.vel.clone()
        return preview

    @staticmethod
    def _message_payload(message: object) -> np.ndarray:
        try:
            values = np.asarray(
                [
                    message.linear.x,
                    message.linear.y,
                    message.linear.z,
                    message.angular.x,
                    message.angular.y,
                    message.angular.z,
                ]
            )
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(
                "ROS cmd_vel payload is not a real numeric message"
            ) from exc
        if values.dtype.kind not in "iuf":
            raise ValueError("ROS cmd_vel payload is not a real numeric message")
        if not np.all(np.isfinite(values)):
            raise ValueError("ROS cmd_vel payload contains non-finite values")
        with np.errstate(over="ignore", invalid="ignore"):
            represented = values.astype(np.float32, copy=True)
        if not np.all(np.isfinite(represented)):
            raise ValueError("ROS cmd_vel payload is not finite in float32")
        return represented

    def _candidate_message(self, linear: float, angular: float) -> object:
        try:
            candidate = type(self.msg)()
            candidate.linear.x = self.msg.linear.x
            candidate.linear.y = self.msg.linear.y
            candidate.linear.z = self.msg.linear.z
            candidate.angular.x = self.msg.angular.x
            candidate.angular.y = self.msg.angular.y
            candidate.angular.z = self.msg.angular.z
            candidate.linear.x = linear
            candidate.angular.z = angular
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError("ROS cmd_vel payload cannot be represented") from exc
        self._message_payload(candidate)
        return candidate

    def prepare_action(self, action: dict[str, np.ndarray]) -> PreparedHardwareAction:
        """Prepare a complete cmd_vel payload without changing live state."""
        self._reject_lifecycle_reentry("prepare_action")
        super().prepare_action(action)
        self.validate_dispatch_ready()
        robot_key = f"{self._action_namespace(self.cfg.topic)}/wheeled_drive"
        if robot_key not in action:
            raise ValueError(f"[{self.name}] Missing action key '{robot_key}'")
        supplied = np.asarray(action[robot_key])
        if supplied.shape != (2,):
            raise ValueError(
                f"[{self.name}] Action shape mismatch: expected (2,), "
                f"got {supplied.shape}"
            )
        if supplied.dtype.kind not in "iuf":
            raise ValueError(f"[{self.name}] Action must contain real numeric values")
        with np.errstate(over="ignore", invalid="ignore"):
            raw = np.asarray(supplied, dtype=np.float32)
        if not np.all(np.isfinite(raw)):
            raise ValueError(f"[{self.name}] Action contains non-finite values")
        caps = self._validated_velocity_caps()

        next_rate_limiter: RateLimiter | None = self._rate_limiter
        next_smooth_filter: CriticallyDampedFilter | None = self._smooth_filter
        if self.cfg.filter_enabled:
            if self._rate_limiter is None or self._smooth_filter is None:
                raise RuntimeError(
                    f"[{self.name}] prepare_action called before start()"
                )
            next_rate_limiter = self._copy_rate_limiter(self._rate_limiter)
            next_smooth_filter = self._copy_smooth_filter(self._smooth_filter)
            limits = torch.tensor(
                [self.cfg.v_lin_max, self.cfg.v_ang_max],
                dtype=torch.float32,
            )
            target = torch.from_numpy(raw).clamp(-1.0, 1.0) * limits
            emitted = next_smooth_filter.step(next_rate_limiter.step(target))
            if not isinstance(emitted, torch.Tensor) or emitted.shape != (2,):
                raise ValueError(f"[{self.name}] Filter emitted an invalid shape")
            if not torch.is_floating_point(emitted) or not bool(
                torch.isfinite(emitted).all()
            ):
                raise ValueError(f"[{self.name}] Filter emitted non-finite data")
            next_emitted = (
                emitted.detach()
                .cpu()
                .numpy()
                .astype(
                    np.float32,
                    copy=True,
                )
            )
        else:
            # Match WheeledDriveAction: clamp normalized input, then scale.
            clamped = np.clip(raw, -1.0, 1.0)
            try:
                scaled = np.asarray(
                    [
                        self.action_scale_linear * clamped[0],
                        self.action_scale_angular * clamped[1],
                    ],
                    dtype=np.float64,
                )
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(
                    f"[{self.name}] Command is not real numeric data"
                ) from exc
            with np.errstate(over="ignore", invalid="ignore"):
                next_emitted = scaled.astype(np.float32, copy=True)

        if next_emitted.shape != (2,) or not np.all(np.isfinite(next_emitted)):
            raise ValueError(f"[{self.name}] Command produced a non-finite command")
        # Apply configured caps last on both paths; values within them pass
        # unchanged, within float32 precision.
        for index, cap in enumerate(caps):
            if cap is not None:
                next_emitted[index] = np.clip(next_emitted[index], -cap, cap)
        linear = float(next_emitted[0])
        angular = float(next_emitted[1])
        candidate_msg = self._candidate_message(linear, angular)
        pub = getattr(self, "pub", None)
        if pub is None:
            raise SafetyViolationError(
                code=f"{self.name}.command_transport_unavailable",
                message="ROS cmd_vel publisher is unavailable",
                component=self.name,
            )

        def send() -> None:
            current_pub = getattr(self, "pub", None)
            if current_pub is None:
                raise SafetyViolationError(
                    code=f"{self.name}.command_transport_unavailable",
                    message="ROS cmd_vel publisher is unavailable",
                    component=self.name,
                )
            try:
                current_pub.publish(candidate_msg)
            except Exception as exc:  # noqa: BLE001 - command delivery is safety-critical
                raise SafetyViolationError(
                    code=f"{self.name}.command_send_failed",
                    message=f"ROS cmd_vel publish failed: {exc}",
                    component=self.name,
                    details={"exception_type": type(exc).__name__},
                ) from exc

        committed_emitted = np.array(next_emitted, dtype=np.float32, copy=True)

        def commit() -> None:
            self._rate_limiter = next_rate_limiter
            self._smooth_filter = next_smooth_filter
            self._emitted = committed_emitted.copy()
            self.msg = candidate_msg

        return PreparedHardwareAction(self, send=send, commit=commit)

    def apply_action(self, action: dict[str, np.ndarray]):
        prepared = self.prepare_action(action)
        self.commit_prepared_action(action, prepared)

    @property
    def emitted_action(self) -> dict[str, np.ndarray]:
        robot_key = f"{self._action_namespace(self.cfg.topic)}/wheeled_drive"
        return {robot_key: self._emitted.copy()}

    def _build_filters(self, dt_s: float) -> None:
        if not isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError(
                f"[{self.name}] action_rate (period in seconds) must be "
                f"finite and positive, got {dt_s}"
            )
        velocity_limits = (self.cfg.v_lin_max, self.cfg.v_ang_max)
        if any(not isfinite(value) or value <= 0.0 for value in velocity_limits):
            raise ValueError(
                f"[{self.name}] velocity limits must be finite and positive, "
                f"got {velocity_limits}"
            )
        acceleration_limits = (self.cfg.accel_lin_max, self.cfg.accel_ang_max)
        if any(not isfinite(value) or value <= 0.0 for value in acceleration_limits):
            raise ValueError(
                f"[{self.name}] acceleration limits must be finite and positive, "
                f"got {acceleration_limits}"
            )
        self._validated_velocity_caps()
        self._rate_limiter = RateLimiter(
            (2,),
            max_rate_per_step=torch.tensor(
                [
                    self.cfg.accel_lin_max * dt_s,
                    self.cfg.accel_ang_max * dt_s,
                ],
                dtype=torch.float32,
            ),
            device="cpu",
        )
        self._smooth_filter = CriticallyDampedFilter(
            (2,),
            tau_s=self.cfg.filter_tau_s,
            dt_s=dt_s,
            device="cpu",
        )

    def _validated_velocity_caps(self) -> tuple[np.float32 | None, np.float32 | None]:
        """Return the float32 caps, rounded down so a capped value never
        exceeds the configured limit once published."""
        caps: list[np.float32 | None] = []
        for name, value in (
            ("max_linear_velocity", self.cfg.max_linear_velocity),
            ("max_angular_velocity", self.cfg.max_angular_velocity),
        ):
            if value is None:
                caps.append(None)
                continue
            if not isfinite(value) or value < 0.0:
                raise ValueError(
                    f"[{self.name}] {name} must be finite and non-negative, got {value}"
                )
            with np.errstate(over="ignore", invalid="ignore"):
                cap = np.float32(value)
            if not np.isfinite(cap):
                raise ValueError(
                    f"[{self.name}] {name} must be representable as finite "
                    f"float32, got {value}"
                )
            if float(cap) > float(value):
                cap = np.nextafter(cap, np.float32(0.0))
            caps.append(cap)
        return caps[0], caps[1]

    def _publish_stop(self, *, required: bool = False) -> None:
        pub = getattr(self, "pub", None)
        if pub is None:
            if required:
                raise SafetyViolationError(
                    code=f"{self.name}.safe_stop_transport_unavailable",
                    message="cannot safe-stop: ROS cmd_vel publisher is unavailable",
                    component=self.name,
                )
            return
        try:
            pub.publish(self.msg)
        except Exception as exc:  # noqa: BLE001
            raise SafetyViolationError(
                code=f"{self.name}.safe_stop_failed",
                message=f"ROS cmd_vel safe stop failed: {exc}",
                component=self.name,
                details={"exception_type": type(exc).__name__},
            ) from exc
