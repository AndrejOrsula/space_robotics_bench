"""Kinova Kortex joint-velocity commands and joint feedback.

Streams filtered, speed-limited joint targets; reports normalized position
and velocity without FK. Deploy filtering is not sim-equivalent for fabrics
policies. Kortex SDK imports are deferred."""

from __future__ import annotations

import copy
import math
import os
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Sequence

import gymnasium
import numpy as np
import torch
from pydantic import Field

from srb.core.action_filters import (
    CompositeFilter,
    CriticallyDampedFilter,
    JerkLimiter,
    RateLimiter,
    filter_chain,
)
from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    PreparedHardwareAction,
    observation_spaces_from_values,
)
from srb.interfaces.sim_to_real.core.lifecycle import raise_cleanup_errors
from srb.utils import logging
from srb.utils.failures import (
    HealthEvent,
    HealthSeverity,
    HealthStatus,
    SafetyViolationError,
)

from ._kortex_math import normalize_joint_positions as _normalize_joint_positions
from ._kortex_math import wrap_to_pi as _wrap_to_pi

if TYPE_CHECKING:
    pass


# Kinova Gen3 7-DoF limits, radians. Continuous joints use [-pi, pi].
_KINOVA_GEN3N7_JOINT_LIMITS_LOWER: tuple[float, ...] = (
    -math.pi,  # joint_1 (continuous)
    -2.41,  # joint_2
    -math.pi,  # joint_3 (continuous)
    -2.66,  # joint_4
    -math.pi,  # joint_5 (continuous)
    -2.23,  # joint_6
    -math.pi,  # joint_7 (continuous)
)
_KINOVA_GEN3N7_JOINT_LIMITS_UPPER: tuple[float, ...] = (
    math.pi,
    2.41,
    math.pi,
    2.66,
    math.pi,
    2.23,
    math.pi,
)

# Table 41 caps admittance/force at 50 deg/s; this interface uses Table 40.
_KINOVA_GEN3N7_JOINT_VEL_LIMIT_RAD_S: tuple[float, ...] = (
    math.radians(79.64),
    math.radians(79.64),
    math.radians(79.64),
    math.radians(79.64),
    math.radians(69.91),
    math.radians(69.91),
    math.radians(69.91),
)

_FEEDBACK_MAX_AGE_S = 0.5


def _float32_toward_zero(values: np.ndarray) -> np.ndarray:
    """Round finite values to float32 without increasing any magnitude.

    A clamp bound rounded to nearest can land ~1 ulp above the configured
    limit; stepping those entries one ulp towards zero keeps the published
    command within the limit.
    """
    exact = np.asarray(values, dtype=np.float64)
    rounded = exact.astype(np.float32)
    above = np.abs(rounded.astype(np.float64)) > np.abs(exact)
    return np.where(above, np.nextafter(rounded, np.float32(0.0)), rounded).astype(
        np.float32
    )


class RosKortexJointVelInterfaceCfg(HardwareInterfaceCfg):
    """Configuration for the Kortex joint-velocity hardware interface."""

    name: str = "ros_kortex_joint_vel"

    kortex_ip: str = "192.168.1.10"
    kortex_port: int = Field(default=10000, ge=1, le=65535)
    kortex_username: str = "admin"
    kortex_password: str | None = None
    # Reject credentials in config; read the password from SRB_KORTEX_PASSWORD.
    kortex_session_timeout_ms: int = 60000
    kortex_connection_inactivity_timeout_ms: int = 60000
    kortex_connect_timeout_ms: int = 2000

    pre_filter_tau_s: float = 0.1
    rate_limit_per_step: float = 1.0
    jerk_limit_per_step: float = 50.0

    joint_limits_lower: Sequence[float] = _KINOVA_GEN3N7_JOINT_LIMITS_LOWER
    joint_limits_upper: Sequence[float] = _KINOVA_GEN3N7_JOINT_LIMITS_UPPER

    joint_vel_limit_rad_s: Sequence[float] = _KINOVA_GEN3N7_JOINT_VEL_LIMIT_RAD_S

    num_joints: int = 7


class RosKortexJointVelInterface(HardwareInterface):
    """Stream normalized joint-velocity actions and expose joint feedback.

    ``robot/joint_velocity`` is normalized and scaled by per-joint rad/s
    limits before Kortex deg/s transmission. Feedback exposes
    ``proprio/joint_pos_normalized`` and ``proprio/joint_vel``; no FK.
    """

    cfg: RosKortexJointVelInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()

    ACTION_KEY: str = "robot/joint_velocity"

    def __init__(
        self,
        cfg: RosKortexJointVelInterfaceCfg | None = None,
        *,
        clock: Callable[[], float] | None = None,
    ):
        if cfg is None:
            cfg = RosKortexJointVelInterfaceCfg()
        super().__init__(cfg)
        self._clock = time.monotonic if clock is None else clock

        self.obs: dict[str, np.ndarray] = self._empty_obs()
        self._emitted_action = np.zeros((int(self.cfg.num_joints),), dtype=np.float32)

        self._rate_limiter: RateLimiter | None = None
        self._smooth_filter: CriticallyDampedFilter | None = None
        self._jerk_limiter: JerkLimiter | None = None
        self._filter_chain: CompositeFilter | None = None
        self._dt_s: float | None = None

        self._vel_limit_rad_s: torch.Tensor | None = None

        self._kortex_transport: Any = None
        self._kortex_router: Any = None
        self._kortex_session: Any = None
        self._kortex_base: Any = None
        self._kortex_base_cyclic: Any = None

        self._joint_speeds_cmd: Any = None
        self._startup_cleanup_pending: bool = False

        self._last_feedback_s: float | None = None
        self._last_feedback_error: str | None = None
        self._feedback_clock_error: str | None = None
        self._feedback_clock_recovery_pending = False

    def start(self, **kwargs):
        super().start(**kwargs)
        try:
            self._start_kortex()
        except BaseException as startup_error:
            self._handle_start_failure(startup_error)
            raise

    def _start_kortex(self) -> None:
        self._last_feedback_s = None
        self._last_feedback_error = None
        self._feedback_clock_error = None
        self._feedback_clock_recovery_pending = False
        self._validate_joint_position_limits()
        if self.cfg.kortex_password is not None:
            raise ValueError(
                f"[{self.name}] Kortex password must not be set in the "
                "configuration (it would be persisted with run metadata); "
                "export SRB_KORTEX_PASSWORD instead"
            )
        password = os.environ.get("SRB_KORTEX_PASSWORD")
        if not password or password == "admin":
            raise ValueError(
                f"[{self.name}] Kortex password must come from the "
                "SRB_KORTEX_PASSWORD environment variable and must not be the "
                "factory default"
            )

        # Here action_rate is the agent step period in seconds.
        if not math.isfinite(self._action_rate) or self._action_rate <= 0.0:
            raise ValueError(
                f"[{self.name}] action_rate (period in seconds) must be "
                f"finite and positive, got {self._action_rate}"
            )
        self._dt_s = float(self._action_rate)

        self._build_filters(self._dt_s)

        from kortex_api.autogen.client_stubs.BaseClientRpc import (  # type: ignore[import-not-found]
            BaseClient,
        )
        from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import (  # type: ignore[import-not-found]
            BaseCyclicClient,
        )
        from kortex_api.autogen.client_stubs.SessionClientRpc import (  # type: ignore[import-not-found]
            SessionClient,
        )
        from kortex_api.autogen.messages import (  # type: ignore[import-not-found]
            Base_pb2,
            Session_pb2,
        )
        from kortex_api.RouterClient import (  # type: ignore[import-not-found]
            RouterClient,
            RouterClientSendOptions,
        )
        from kortex_api.TCPTransport import (
            TCPTransport,  # type: ignore[import-not-found]
        )

        # Build before connecting so protobuf failure cannot strand a session.
        joint_speeds_cmd = Base_pb2.JointSpeeds()
        for i in range(int(self.cfg.num_joints)):
            js = joint_speeds_cmd.joint_speeds.add()
            js.joint_identifier = i
            js.value = 0.0
            js.duration = 0
        self._joint_speeds_cmd = joint_speeds_cmd

        transport = TCPTransport()
        self._kortex_transport = transport
        transport.connect(self.cfg.kortex_ip, self.cfg.kortex_port)
        logging.info(
            f"[{self.name}] Connected TCP transport to "
            f"{self.cfg.kortex_ip}:{self.cfg.kortex_port}"
        )

        def _error_cb(kException):  # noqa: N803 - Kortex naming
            logging.error(f"[{self.name}] Kortex callback error: {kException}")

        router = RouterClient(transport, _error_cb)
        self._kortex_router = router

        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = self.cfg.kortex_username
        session_info.password = password
        session_info.session_inactivity_timeout = self.cfg.kortex_session_timeout_ms
        session_info.connection_inactivity_timeout = (
            self.cfg.kortex_connection_inactivity_timeout_ms
        )

        session = SessionClient(router)
        self._kortex_session = session
        session.CreateSession(session_info)
        logging.info(f"[{self.name}] Kortex session opened")

        send_options = RouterClientSendOptions()
        send_options.timeout_ms = 1000  # ms

        base_client = BaseClient(router, send_options)
        self._kortex_base = base_client
        base_cyclic_client = BaseCyclicClient(router, send_options)
        self._kortex_base_cyclic = base_cyclic_client

    def _validate_joint_position_limits(self) -> None:
        n = int(self.cfg.num_joints)
        try:
            lower = np.asarray(self.cfg.joint_limits_lower, dtype=np.float64)
            upper = np.asarray(self.cfg.joint_limits_upper, dtype=np.float64)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"[{self.name}] joint position limits must be numeric"
            ) from exc
        if lower.shape != (n,) or upper.shape != (n,):
            raise ValueError(
                f"[{self.name}] joint position limits must have shape ({n},), "
                f"got {lower.shape} and {upper.shape}"
            )
        if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
            raise ValueError(f"[{self.name}] joint position limits must be finite")
        if np.any(upper <= lower):
            raise ValueError(
                f"[{self.name}] each upper joint position limit must exceed its lower limit"
            )

    def close(self) -> None:
        # Invalidate staged commands before stop or cleanup; failed cleanup is retryable.
        self._action_revision += 1
        if (
            self.is_started
            and not self.safe_stop_issued
            and not self._startup_cleanup_pending
        ):
            self.safe_stop()

        cleanup_errors = self._close_kortex_resources()
        raise_cleanup_errors(cleanup_errors, context="Kortex joint cleanup failed")
        self._startup_cleanup_pending = False
        super().close()

    def _handle_start_failure(self, startup_error: BaseException) -> None:
        cleanup_errors = self._close_kortex_resources()
        if cleanup_errors:
            # No policy command was sent; retain handles for direct cleanup retry.
            self._startup_cleanup_pending = True
            self._safe_stop_requested = True
            self._is_started = False
            for cleanup_error in cleanup_errors:
                startup_error.add_note(
                    "Kortex joint startup cleanup failed: "
                    f"{type(cleanup_error).__name__}: {cleanup_error}"
                )
            return
        self._startup_cleanup_pending = False
        super().close()

    def _close_kortex_resources(self) -> list[Exception]:
        # Retain dependent handles on failure so cleanup can be retried.
        errors: list[Exception] = []
        if self._kortex_session is not None:
            try:
                self._kortex_session.CloseSession()
            except Exception as exc:  # noqa: BLE001 - retain for retry
                errors.append(exc)
                return errors
            else:
                self._kortex_session = None

        if self._kortex_router is not None:
            try:
                self._kortex_router.SetActivationStatus(False)
            except Exception as exc:  # noqa: BLE001 - retain for retry
                errors.append(exc)
                return errors
            else:
                self._kortex_router = None

        if self._kortex_transport is not None:
            try:
                self._kortex_transport.disconnect()
            except Exception as exc:  # noqa: BLE001 - retain for retry
                errors.append(exc)
                return errors
            else:
                self._kortex_transport = None

        if not errors:
            self._kortex_base = None
            self._kortex_base_cyclic = None
            self._joint_speeds_cmd = None
        return errors

    def reset(self):
        super().reset()

        if self._filter_chain is not None:
            zeros = torch.zeros((int(self.cfg.num_joints),), dtype=torch.float32)
            self._filter_chain.reset(None, zeros)

        self.obs = self._empty_obs()
        self._last_feedback_s = None
        self._last_feedback_error = None
        self._feedback_clock_error = None
        self._feedback_clock_recovery_pending = False

        self._send_zero_joint_speeds(required=False)
        self._emitted_action.fill(0.0)

    def pause(self) -> None:
        """Pause means an immediate physical zero command, not retained motion."""

        super().pause()
        if self._filter_chain is not None:
            self._filter_chain.reset(
                None,
                torch.zeros((int(self.cfg.num_joints),), dtype=torch.float32),
            )
        self._send_zero_joint_speeds(required=self.is_started)
        self._emitted_action.fill(0.0)

    def sync(self):
        super().sync()
        if self._kortex_base_cyclic is None:
            # Preserve the last accepted frame; admission stays unhealthy.
            self._last_feedback_error = "Kortex feedback transport is unavailable"
            return

        try:
            feedback = self._kortex_base_cyclic.RefreshFeedback()
        except Exception as e:  # noqa: BLE001 - transient network issues
            logging.warning(f"[{self.name}] RefreshFeedback failed: {e}")
            self._last_feedback_error = f"RefreshFeedback failed: {e}"
            return

        # Publish only after the whole frame and receipt time validate.
        next_obs = self._empty_obs()

        n = int(self.cfg.num_joints)
        joint_pos_rad = np.zeros(n, dtype=np.float32)
        joint_vel_rad = np.zeros(n, dtype=np.float32)
        try:
            for i in range(n):
                act = feedback.actuators[i]
                pos_deg = float(act.position)
                vel_deg_s = float(act.velocity)
                if not math.isfinite(pos_deg) or not math.isfinite(vel_deg_s):
                    raise ValueError(
                        f"actuator {i} feedback contains a non-finite value"
                    )
                pos_rad = math.radians(pos_deg)
                vel_rad = math.radians(vel_deg_s)
                with np.errstate(over="ignore", invalid="ignore"):
                    joint_pos_rad[i] = _wrap_to_pi(pos_rad)
                    joint_vel_rad[i] = vel_rad
        except (
            IndexError,
            AttributeError,
            TypeError,
            ValueError,
            OverflowError,
        ) as e:
            logging.warning(f"[{self.name}] Joint feedback decode failed: {e}")
            self._last_feedback_error = f"joint feedback decode failed: {e}"
            return

        next_obs["proprio/joint_pos_normalized"] = _normalize_joint_positions(
            joint_pos_rad,
            np.asarray(self.cfg.joint_limits_lower, dtype=np.float32),
            np.asarray(self.cfg.joint_limits_upper, dtype=np.float32),
        )
        next_obs["proprio/joint_vel"] = joint_vel_rad
        if not all(np.all(np.isfinite(value)) for value in next_obs.values()):
            logging.warning(
                f"[{self.name}] decoded Kortex feedback contains a non-finite "
                "float32 observation"
            )
            self._last_feedback_error = (
                "decoded Kortex feedback contains a non-finite float32 observation"
            )
            return

        feedback_stamp, clock_error = self._feedback_clock_sample()
        if clock_error is not None:
            logging.warning(f"[{self.name}] {clock_error}")
            self._feedback_clock_error = clock_error
            self._feedback_clock_recovery_pending = True
            self._last_feedback_error = None
            return

        self.obs = next_obs
        self._last_feedback_s = feedback_stamp
        self._last_feedback_error = None
        self._feedback_clock_error = None
        self._feedback_clock_recovery_pending = False

    def health(self) -> HealthEvent | None:
        state, age_s = self._feedback_state()
        healthy = state == "healthy"
        messages = {
            "unavailable": "Kortex feedback transport is unavailable",
            "pre_message": "waiting for the first Kortex feedback sample",
            "failure": self._last_feedback_error or "Kortex feedback failed",
            "clock_invalid": self._feedback_clock_error
            or "Kortex feedback clock is invalid",
            "stale": "Kortex feedback is stale",
            "healthy": "Kortex feedback is current",
        }
        return HealthEvent(
            status=HealthStatus.HEALTHY if healthy else HealthStatus.UNHEALTHY,
            severity=HealthSeverity.INFO if healthy else HealthSeverity.ERROR,
            component=self.name,
            code=f"{self.name}.{state}",
            message=messages[state],
            details={
                "state": state,
                "ready": healthy,
                "age_ms": None if age_s is None else age_s * 1000.0,
                "last_feedback_error": self._last_feedback_error
                or self._feedback_clock_error,
            },
        )

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                self.ACTION_KEY: gymnasium.spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(int(self.cfg.num_joints),),
                    dtype=np.float32,
                )
            }
        )

    def apply_action(self, action: dict[str, np.ndarray]):
        """Prepare and commit one joint command through the owned seam."""
        prepared = self.prepare_action(action)
        self.commit_prepared_action(action, prepared)

    def prepare_action(self, action: dict[str, np.ndarray]) -> PreparedHardwareAction:
        """Build a complete joint transport attempt without side effects.

        The stateful deploy filters advance on a private copy.  Only the
        prepared token's commit callback installs that state and the emitted
        rad/s report after the synchronous SDK call returns normally.
        """
        if self.ACTION_KEY not in action:
            raise ValueError(f"[{self.name}] Missing action key '{self.ACTION_KEY}'")
        try:
            raw = np.asarray(action[self.ACTION_KEY])
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"[{self.name}] Action is not an array: {exc}") from exc
        n = int(self.cfg.num_joints)
        if raw.shape != (n,):
            raise ValueError(
                f"[{self.name}] Action shape mismatch: expected ({n},), got {raw.shape}"
            )
        if raw.dtype.kind not in "fiu":
            raise ValueError(
                f"[{self.name}] Action must have a real numeric dtype, got {raw.dtype}"
            )
        with np.errstate(over="ignore", invalid="ignore"):
            raw_float32 = raw.astype(np.float32)
        if not np.all(np.isfinite(raw_float32)):
            raise ValueError(f"[{self.name}] Action contains non-finite values")
        super().prepare_action(action)

        if (
            self._filter_chain is None
            or self._dt_s is None
            or self._vel_limit_rad_s is None
        ):
            raise RuntimeError(f"[{self.name}] apply_action called before start()")

        self._require_current_feedback()
        # Validate transport before an aggregate can commit any route.
        self._validate_joint_command_transport()

        extra_scale = float(self._action_scale.get(self.ACTION_KEY, 1.0))
        if not math.isfinite(extra_scale):
            raise ValueError(f"[{self.name}] Action scale is non-finite")
        raw_t = torch.as_tensor(raw_float32, dtype=torch.float32, device="cpu")
        scaled = raw_t * extra_scale
        if not bool(torch.isfinite(scaled).all()):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="scaled Kortex joint action is non-finite",
                component=self.name,
            )
        scaled = scaled.clamp(-1.0, 1.0)

        # Stage filter state so failed preparation or send cannot advance it.
        preview = self._copy_filter_chain()
        filtered = preview.step(scaled)
        filtered_np = self._validated_filter_output(filtered)

        vel_limit_raw = self._vel_limit_rad_s.detach().cpu().numpy()
        if vel_limit_raw.shape != (n,) or vel_limit_raw.dtype.kind not in "fiu":
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex joint velocity limits have invalid shape or dtype",
                component=self.name,
            )
        with np.errstate(over="ignore", invalid="ignore"):
            vel_limit_np = vel_limit_raw.astype(np.float32)
        if not np.all(np.isfinite(vel_limit_raw)) or not np.all(
            np.isfinite(vel_limit_np)
        ):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex joint velocity limits are non-finite",
                component=self.name,
            )
        if np.any(vel_limit_np < 0.0):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex joint velocity limits must be non-negative",
                component=self.name,
            )
        with np.errstate(over="ignore", invalid="ignore"):
            vel_rad_s = filtered_np * vel_limit_np
        report = self._validated_report(vel_rad_s)
        with np.errstate(over="ignore", invalid="ignore"):
            self._validate_joint_speeds_payload(np.degrees(report))
        # Round bounds toward zero so float32 conversion cannot exceed the limit.
        vel_limit_f64 = vel_limit_raw.astype(np.float64)
        bound_rad_s = _float32_toward_zero(vel_limit_f64)
        report = np.clip(report, -bound_rad_s, bound_rad_s)
        with np.errstate(over="ignore", invalid="ignore"):
            vel_deg_s = np.degrees(report)
        bound_deg_s = _float32_toward_zero(np.degrees(vel_limit_f64))
        vel_deg_s = np.clip(vel_deg_s, -bound_deg_s, bound_deg_s)
        transport_values = self._validate_joint_speeds_payload(vel_deg_s)

        report = report.astype(np.float32, copy=True)
        transport_values = transport_values.astype(np.float32, copy=True)

        def send() -> None:
            # Feedback can expire during aggregate preparation.
            self._require_current_feedback()
            self._send_prevalidated_joint_speeds(transport_values)

        def commit() -> None:
            self._install_filter_state(preview)
            self._emitted_action[:] = report

        return PreparedHardwareAction(self, send=send, commit=commit)

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs

    @property
    def emitted_action(self) -> dict[str, np.ndarray]:
        """Return the last submitted joint command in radians per second."""
        return {self.ACTION_KEY: self._emitted_action.copy()}

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self._empty_obs())

    def _feedback_state(self) -> tuple[str, float | None]:
        if self._kortex_base_cyclic is None:
            return "unavailable", None
        if self._feedback_clock_recovery_pending:
            return "clock_invalid", None
        if self._last_feedback_error is not None:
            return "failure", None
        if self._last_feedback_s is None:
            return "pre_message", None
        try:
            now_s = float(self._clock())
        except (
            AttributeError,
            OSError,
            OverflowError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as exc:
            self._feedback_clock_error = (
                f"Kortex feedback clock read failed: {type(exc).__name__}: {exc}"
            )
            self._feedback_clock_recovery_pending = True
            return "clock_invalid", None
        if not math.isfinite(now_s):
            self._feedback_clock_error = (
                "Kortex feedback clock returned a non-finite time"
            )
            self._feedback_clock_recovery_pending = True
            return "clock_invalid", None
        if now_s < self._last_feedback_s:
            self._feedback_clock_error = (
                "Kortex feedback clock moved backwards from "
                f"{self._last_feedback_s:.9f} to {now_s:.9f}"
            )
            self._feedback_clock_recovery_pending = True
            return "clock_invalid", None
        age_s = now_s - self._last_feedback_s
        if age_s > _FEEDBACK_MAX_AGE_S:
            return "stale", age_s
        return "healthy", age_s

    def _feedback_clock_sample(self) -> tuple[float | None, str | None]:
        """Read a finite, non-rolled-back receipt time for feedback."""
        try:
            stamp = float(self._clock())
        except (
            AttributeError,
            OSError,
            OverflowError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as exc:
            return None, (
                f"Kortex feedback clock read failed: {type(exc).__name__}: {exc}"
            )
        if not math.isfinite(stamp):
            return None, "Kortex feedback clock returned a non-finite time"
        if self._last_feedback_s is not None and stamp < self._last_feedback_s:
            return None, (
                "Kortex feedback clock moved backwards from "
                f"{self._last_feedback_s:.9f} to {stamp:.9f}"
            )
        return stamp, None

    def _require_current_feedback(self) -> None:
        """Reject a command unless a decoded feedback sample is current."""
        feedback_state, feedback_age_s = self._feedback_state()
        if feedback_state != "healthy":
            raise SafetyViolationError(
                code=f"{self.name}.feedback_{feedback_state}",
                message=(
                    "refusing to send Kortex joint velocities without current "
                    "successfully decoded feedback"
                ),
                component=self.name,
                details={
                    "state": feedback_state,
                    "age_ms": (
                        None if feedback_age_s is None else feedback_age_s * 1000.0
                    ),
                    "last_feedback_error": self._last_feedback_error
                    or self._feedback_clock_error,
                },
            )

    @staticmethod
    def _clone_filter(value: object) -> object:
        """Clone filter attributes while detaching mutable tensor state."""
        cloned = copy.copy(value)
        for name, state in vars(value).items():
            if isinstance(state, torch.Tensor):
                setattr(cloned, name, state.detach().clone())
            else:
                setattr(cloned, name, copy.deepcopy(state))
        return cloned

    def _copy_filter_chain(self) -> CompositeFilter:
        """Return a private copy of the complete live filter chain."""
        if self._filter_chain is None:
            raise RuntimeError(f"[{self.name}] joint filters are not initialized")
        chain = copy.copy(self._filter_chain)
        children = getattr(self._filter_chain, "filters", None)
        if children is not None:
            chain.filters = [self._clone_filter(child) for child in children]
        return chain

    @staticmethod
    def _copy_filter_attributes(current: object, staged: object) -> None:
        """Install staged child state without replacing live child objects."""
        with torch.no_grad():
            for name, state in vars(staged).items():
                if isinstance(state, torch.Tensor):
                    existing = getattr(current, name, None)
                    if (
                        isinstance(existing, torch.Tensor)
                        and existing.shape == state.shape
                        and existing.dtype == state.dtype
                        and existing.device == state.device
                    ):
                        existing.copy_(state)
                    else:
                        setattr(current, name, state.detach().clone())
                else:
                    setattr(current, name, copy.deepcopy(state))

    def _install_filter_state(self, staged: CompositeFilter) -> None:
        """Commit staged state while retaining all live child references."""
        if self._filter_chain is None:
            raise RuntimeError(f"[{self.name}] joint filters are not initialized")
        current_children = getattr(self._filter_chain, "filters", None)
        staged_children = getattr(staged, "filters", None)
        if current_children is None or staged_children is None:
            self._copy_filter_attributes(self._filter_chain, staged)
        else:
            if len(current_children) != len(staged_children):
                raise RuntimeError(
                    f"[{self.name}] joint filter chain changed during commit"
                )
            for current, next_state in zip(
                current_children, staged_children, strict=True
            ):
                self._copy_filter_attributes(current, next_state)

            # Keep named aliases on live children, including jerk counters.
            if len(current_children) >= 3:
                self._rate_limiter = current_children[0]
                self._smooth_filter = current_children[1]
                self._jerk_limiter = current_children[2]

    def _validated_filter_output(self, filtered: object) -> np.ndarray:
        """Validate a complete post-filter output in float32 representation."""
        try:
            if isinstance(filtered, torch.Tensor):
                if filtered.is_complex():
                    raise TypeError("complex filter output")
                values = filtered.detach().cpu().numpy()
            else:
                values = np.asarray(filtered)
        except (TypeError, ValueError, OverflowError) as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=f"Kortex filtered joint command is not numeric: {exc}",
                component=self.name,
            ) from exc
        n = int(self.cfg.num_joints)
        if values.shape != (n,):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    f"Kortex filtered joint command shape mismatch: expected ({n},), "
                    f"got {values.shape}"
                ),
                component=self.name,
            )
        if values.dtype.kind not in "fiu":
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    "Kortex filtered joint command must have a real numeric dtype, "
                    f"got {values.dtype}"
                ),
                component=self.name,
            )
        with np.errstate(over="ignore", invalid="ignore"):
            values_float64 = values.astype(np.float64, copy=False)
            values_float32 = values_float64.astype(np.float32)
        if not np.all(np.isfinite(values_float64)) or not np.all(
            np.isfinite(values_float32)
        ):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex filtered joint command is non-finite",
                component=self.name,
            )
        return values_float32.astype(np.float32, copy=True)

    def _validated_report(self, report: np.ndarray) -> np.ndarray:
        """Validate the emitted rad/s report before any live-state change."""
        n = int(self.cfg.num_joints)
        values = np.asarray(report)
        if values.shape != (n,) or values.dtype.kind not in "fiu":
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex joint rad/s report has invalid shape or dtype",
                component=self.name,
            )
        with np.errstate(over="ignore", invalid="ignore"):
            values_float64 = values.astype(np.float64, copy=False)
            values_float32 = values_float64.astype(np.float32)
        if not np.all(np.isfinite(values_float64)) or not np.all(
            np.isfinite(values_float32)
        ):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex joint rad/s report is non-finite",
                component=self.name,
            )
        return values_float32.astype(np.float32, copy=True)

    def _build_filters(self, dt_s: float) -> None:
        """Build the deploy-side rate, smoothing, and jerk filter chain."""
        dim = (int(self.cfg.num_joints),)
        device = "cpu"
        self._rate_limiter = RateLimiter(
            dim,
            max_rate_per_step=self.cfg.rate_limit_per_step,
            device=device,
        )
        self._smooth_filter = CriticallyDampedFilter(
            dim,
            tau_s=self.cfg.pre_filter_tau_s,
            dt_s=dt_s,
            device=device,
        )
        self._jerk_limiter = JerkLimiter(
            dim,
            max_jerk_per_step=self.cfg.jerk_limit_per_step,
            dt_s=dt_s,
            device=device,
        )
        self._filter_chain = filter_chain(
            [self._rate_limiter, self._smooth_filter, self._jerk_limiter]
        )

        self._vel_limit_rad_s = torch.as_tensor(
            list(self.cfg.joint_vel_limit_rad_s),
            dtype=torch.float64,
            device=device,
        )
        if self._vel_limit_rad_s.shape != dim:
            raise ValueError(
                f"[{self.name}] joint_vel_limit_rad_s shape mismatch: "
                f"expected {dim}, got {tuple(self._vel_limit_rad_s.shape)}"
            )
        if not bool(torch.isfinite(self._vel_limit_rad_s).all()) or bool(
            (self._vel_limit_rad_s < 0.0).any()
        ):
            raise ValueError(
                f"[{self.name}] joint_vel_limit_rad_s must be finite and "
                f"non-negative, got {tuple(self.cfg.joint_vel_limit_rad_s)}"
            )

    def _zero_joint_speeds(self) -> None:
        """Mutate the prebuilt joint-speeds command to all-zero."""
        if self._joint_speeds_cmd is None:
            return
        for js in self._joint_speeds_cmd.joint_speeds:
            js.value = 0.0
            js.duration = 0

    def _send_zero_joint_speeds(self, *, required: bool) -> None:
        if self._kortex_base is None or self._joint_speeds_cmd is None:
            if required:
                raise SafetyViolationError(
                    code=f"{self.name}.safe_stop_transport_unavailable",
                    message="cannot safe-stop: Kortex joint transport is unavailable",
                    component=self.name,
                )
            return
        self._zero_joint_speeds()
        try:
            self._kortex_base.SendJointSpeedsCommand(self._joint_speeds_cmd)
        except Exception as exc:  # noqa: BLE001 - a failed stop stays retryable
            raise SafetyViolationError(
                code=f"{self.name}.safe_stop_failed",
                message=f"Kortex joint safe stop failed: {exc}",
                component=self.name,
                details={"exception_type": type(exc).__name__},
            ) from exc

    def _validate_joint_command_transport(self) -> Callable[[Any], Any]:
        """Validate the reusable command and SDK callable without mutation."""
        if self._kortex_base is None or self._joint_speeds_cmd is None:
            raise SafetyViolationError(
                code=f"{self.name}.command_transport_unavailable",
                message="Kortex joint command transport is unavailable",
                component=self.name,
            )
        try:
            send = getattr(self._kortex_base, "SendJointSpeedsCommand")
        except AttributeError as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_transport_unavailable",
                message="Kortex joint command transport has no SendJointSpeedsCommand",
                component=self.name,
            ) from exc
        if not callable(send):
            raise SafetyViolationError(
                code=f"{self.name}.command_transport_unavailable",
                message="Kortex joint command transport is not callable",
                component=self.name,
            )
        try:
            command_slots = getattr(self._joint_speeds_cmd, "joint_speeds")
            slot_count = len(command_slots)
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=f"Kortex joint command has invalid slots: {exc}",
                component=self.name,
            ) from exc
        n = int(self.cfg.num_joints)
        if slot_count != n:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    f"Kortex joint command slot mismatch: expected {n}, "
                    f"got {slot_count}"
                ),
                component=self.name,
            )
        missing: list[str] = []
        for index in range(n):
            try:
                slot = command_slots[index]
            except (
                AttributeError,
                IndexError,
                TypeError,
                ValueError,
                OverflowError,
            ) as exc:
                missing.append(f"{index} ({type(exc).__name__})")
                continue
            slot_missing: list[str] = []
            for field in ("joint_identifier", "value", "duration"):
                try:
                    getattr(slot, field)
                except (AttributeError, TypeError, ValueError):
                    slot_missing.append(field)
            if slot_missing:
                missing.append(f"{index} ({', '.join(slot_missing)})")
        if missing:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    "Kortex joint command slots are missing required fields: "
                    + ", ".join(missing)
                ),
                component=self.name,
            )
        return send

    def _validate_joint_speeds_payload(self, vel_deg_s: np.ndarray) -> np.ndarray:
        """Purely validate and narrow a complete Kortex float32 payload."""
        n = int(self.cfg.num_joints)
        try:
            payload = np.asarray(vel_deg_s)
        except (TypeError, ValueError, OverflowError) as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=f"Kortex joint speeds are not numeric: {exc}",
                component=self.name,
            ) from exc
        if payload.shape != (n,):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    f"Kortex joint speeds shape mismatch: expected ({n},), "
                    f"got {payload.shape}"
                ),
                component=self.name,
            )
        if payload.dtype.kind not in "fiu":
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    "Kortex joint speeds must have a real numeric dtype, "
                    f"got {payload.dtype}"
                ),
                component=self.name,
            )

        # Validate protobuf float32 representability before mutating the message.
        with np.errstate(over="ignore", invalid="ignore"):
            payload_float64 = payload.astype(np.float64, copy=False)
        if not np.all(np.isfinite(payload_float64)):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex joint speeds contain non-finite degrees/s values",
                component=self.name,
            )
        with np.errstate(over="ignore", invalid="ignore"):
            transport_values = payload_float64.astype(np.float32)
        if not np.all(np.isfinite(transport_values)):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    "Kortex joint speeds are not finite in the transport "
                    "float32 representation"
                ),
                component=self.name,
            )
        return transport_values.astype(np.float32, copy=True)

    def _send_prevalidated_joint_speeds(self, transport_values: np.ndarray) -> None:
        """Send an already-narrowed payload without another transform."""
        send = self._validate_joint_command_transport()
        n = int(self.cfg.num_joints)
        command_slots = self._joint_speeds_cmd.joint_speeds
        for i in range(n):
            js = command_slots[i]
            js.joint_identifier = i
            js.value = float(transport_values[i])
            js.duration = 0  # continuous streaming
        try:
            send(self._joint_speeds_cmd)
        except Exception as e:  # noqa: BLE001 - transient network issues
            raise SafetyViolationError(
                code=f"{self.name}.command_send_failed",
                message=f"SendJointSpeedsCommand failed: {e}",
                component=self.name,
                details={"exception_type": type(e).__name__},
            ) from e

    def _empty_obs(self) -> dict[str, np.ndarray]:
        n = int(self.cfg.num_joints)
        return {
            "proprio/joint_pos_normalized": np.zeros(n, dtype=np.float32),
            "proprio/joint_vel": np.zeros(n, dtype=np.float32),
        }
