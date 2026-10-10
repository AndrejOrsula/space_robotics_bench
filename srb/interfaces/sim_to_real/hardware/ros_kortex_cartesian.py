"""Kinova Kortex Cartesian-admittance commands and joint/FK feedback.

Filters Cartesian deltas with the sim OSC chain, clamps them, and sends base-frame
twists through ``BaseClient.SendTwistCommand``. Feedback comes from
``BaseCyclicClient.RefreshFeedback``; joint state is normalized, and tool pose
comes from the arm's onboard FK. Kortex SDK imports are deferred to runtime so
the module remains importable without the optional SDK."""

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
    attempt_cleanup,
)

from ._kortex_math import normalize_joint_positions as _normalize_joint_positions
from ._kortex_math import wrap_to_pi as _wrap_to_pi

if TYPE_CHECKING:
    pass


# Continuous joints use [-pi, pi] for normalization.
_KINOVA_GEN3N7_JOINT_LIMITS_LOWER: tuple[float, ...] = (
    -math.pi,
    -2.41,  # joint_2
    -math.pi,
    -2.66,  # joint_4
    -math.pi,
    -2.23,  # joint_6
    -math.pi,
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

_FEEDBACK_MAX_AGE_S = 0.5


class RosKortexCartesianInterfaceCfg(HardwareInterfaceCfg):
    """Configuration for the Kortex Cartesian hardware interface."""

    name: str = "ros_kortex_cartesian"

    kortex_ip: str = "192.168.1.10"
    """IP address of the Kinova robot's Kortex API endpoint."""
    kortex_port: int = Field(default=10000, ge=1, le=65535)
    """TCP port for the Kortex API (default 10000)."""
    kortex_username: str = "admin"
    """Kortex API username (default 'admin')."""
    kortex_password: str | None = None
    """Deprecated and rejected: the password is read only from the
    ``SRB_KORTEX_PASSWORD`` environment variable. Configuration values are
    serialised into run metadata and logs, so ``start()`` refuses a non-None
    value here instead of using it."""
    kortex_session_timeout_ms: int = 60000
    """Session inactivity timeout in milliseconds."""
    kortex_connect_timeout_ms: int = 2000
    """Initial TCP connect timeout in milliseconds."""

    pre_filter_tau_s: float = 0.08
    """Time constant for the critically-damped 2nd-order pre-filter [s]."""
    rate_limit_per_step: float = 1.0
    """Max absolute change per step for the RateLimiter."""
    jerk_limit_per_step: float = 100.0
    """Max jerk budget per step for the JerkLimiter."""

    delta_pos_limit_m: float = 0.05
    """Symmetric clamp on each Cartesian-position delta channel [m]."""
    delta_rot_limit_rad: float = 0.1
    """Symmetric clamp on each axis-angle delta channel [rad]."""

    joint_limits_lower: Sequence[float] = _KINOVA_GEN3N7_JOINT_LIMITS_LOWER
    """Per-joint lower limits [rad] used to map to [-1, 1] in obs."""
    joint_limits_upper: Sequence[float] = _KINOVA_GEN3N7_JOINT_LIMITS_UPPER
    """Per-joint upper limits [rad] used to map to [-1, 1] in obs."""
    num_joints: int = 7
    """Number of actuated joints exposed in the joint-state observation."""

    contact_force_filter_tau_s: float = 0.05
    """Time constant [s] for the low-pass on ``contact_force_scoop``."""


class RosKortexCartesianInterface(HardwareInterface):
    """Kortex Cartesian-admittance command + joint/FK observation interface.

    Advertises one action ``robot/cartesian_delta_pose`` of shape ``(6,)`` in
    normalised ``[-1, 1]`` range. Each ``apply_action`` call:
      1. scales the raw delta to physical units (m / rad),
      2. runs the RateLimiter -> CriticallyDampedFilter -> JerkLimiter chain,
      3. applies the position/orientation factors, clamps the per-step delta
         to ``delta_pos_limit_m`` / ``delta_rot_limit_rad`` again and converts
         it to a twist (divide by ``dt_s``),
      4. sends it to the arm via ``base.SendTwistCommand``.

    Each ``sync`` call refreshes cyclic feedback and populates:

      - ``proprio/joint_pos_normalized``     : (num_joints,) float32 in [-1, 1]
      - ``proprio/joint_vel``                : (num_joints,) float32 in rad/s
      - ``proprio/fk_pos_end_effector_base`` : (3,) float32 in metres
      - ``proprio/fk_rot6d_end_effector_base``: (6,) float32 (first 2 cols
        of the tool rotation matrix expressed in the arm base frame).
      - ``proprio_dyn/contact_force_scoop``  : (3,) float32 in Newtons, a
        low-pass-filtered estimate of the EE external wrench read from
        Kortex's onboard ``tool_external_wrench_force_{x,y,z}``.
    """

    cfg: RosKortexCartesianInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()

    ACTION_KEY: str = "robot/cartesian_delta_pose"
    ACTION_DIM: int = 6

    def __init__(
        self,
        cfg: RosKortexCartesianInterfaceCfg | None = None,
        *,
        clock: Callable[[], float] | None = None,
    ):
        if cfg is None:
            cfg = RosKortexCartesianInterfaceCfg()
        super().__init__(cfg)
        self._clock = time.monotonic if clock is None else clock

        self.obs: dict[str, np.ndarray] = self._empty_obs()

        self._rate_limiter: RateLimiter | None = None
        self._smooth_filter: CriticallyDampedFilter | None = None
        self._jerk_limiter: JerkLimiter | None = None
        self._filter_chain: CompositeFilter | None = None
        self._contact_force_filter: CriticallyDampedFilter | None = None
        self._dt_s: float | None = None

        self._kortex_transport: Any = None
        self._kortex_router: Any = None
        self._kortex_session: Any = None
        self._kortex_base: Any = None
        self._kortex_base_cyclic: Any = None

        self._twist_command: Any = None
        self._prepared_transport_payload: np.ndarray | None = None
        self._startup_cleanup_pending: bool = False

        # Reported command is the last submitted per-step SI delta, not measured motion.
        self._emitted_action = np.zeros(self.ACTION_DIM, dtype=np.float32)

        self._last_feedback_stamp: float | None = None
        self._last_feedback_error: str | None = None
        self._feedback_clock_error: str | None = None
        self._feedback_clock_recovery_pending = False
        self._contact_force_filter_recovery_pending = False
        self._contact_force_filter_reset_error: str | None = None

    def start(self, **kwargs):
        # Invalidate staged commands even if the base start guard rejects entry.
        self._action_revision += 1
        self._emitted_action.fill(0.0)
        super().start(**kwargs)
        try:
            self._start_kortex()
        except BaseException as startup_error:
            self._handle_start_failure(startup_error)
            raise

    def _start_kortex(self) -> None:
        self._last_feedback_stamp = None
        self._last_feedback_error = None
        self._feedback_clock_error = None
        self._feedback_clock_recovery_pending = False
        self._contact_force_filter_recovery_pending = False
        self._contact_force_filter_reset_error = None
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

        # action_rate is the agent step period in seconds.
        if not math.isfinite(self._action_rate) or self._action_rate <= 0.0:
            raise ValueError(
                f"[{self.name}] action_rate (period in seconds) must be "
                f"finite and positive, got {self._action_rate}"
            )
        self._dt_s = float(self._action_rate)

        # One-element limits would broadcast silently across every joint.
        lower = np.asarray(self.cfg.joint_limits_lower, dtype=np.float64)
        upper = np.asarray(self.cfg.joint_limits_upper, dtype=np.float64)
        expected = (int(self.cfg.num_joints),)
        if (
            lower.shape != expected
            or upper.shape != expected
            or not np.all(np.isfinite(lower))
            or not np.all(np.isfinite(upper))
            or np.any(lower >= upper)
        ):
            raise ValueError(
                f"[{self.name}] joint_limits_lower/upper must be finite, have "
                f"num_joints={expected[0]} entries each, and satisfy lower < upper"
            )

        self._build_filters(self._dt_s)

        # Deferred imports keep the optional Kortex SDK out of module import.
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

        # Build before connecting so protobuf failure cannot leak a live session.
        twist_command = Base_pb2.TwistCommand()
        twist_command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        twist_command.duration = 0  # continuous streaming
        self._twist_command = twist_command

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
        session_info.connection_inactivity_timeout = self.cfg.kortex_connect_timeout_ms

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

    def close(self) -> None:
        # Invalidate staged commands before stop or cleanup callbacks.
        self._action_revision += 1
        self._prepared_transport_payload = None
        # A failed stop must leave handles available for a fail-closed retry.
        if (
            self.is_started
            and not self.safe_stop_issued
            and not self._startup_cleanup_pending
        ):
            self.safe_stop()

        cleanup_errors = self._close_kortex_resources()
        raise_cleanup_errors(cleanup_errors, context="Kortex Cartesian cleanup failed")
        self._startup_cleanup_pending = False
        super().close()
        self._emitted_action.fill(0.0)

    def _handle_start_failure(self, startup_error: BaseException) -> None:
        cleanup_errors = self._close_kortex_resources()
        if cleanup_errors:
            # Startup dispatched no command; retain the start barrier and retry cleanup.
            self._startup_cleanup_pending = True
            self._safe_stop_requested = True
            self._is_started = False
            for cleanup_error in cleanup_errors:
                startup_error.add_note(
                    "Kortex Cartesian startup cleanup failed: "
                    f"{type(cleanup_error).__name__}: {cleanup_error}"
                )
            return
        # No command was sent, so a released partial startup needs no safe-stop claim.
        self._startup_cleanup_pending = False
        super().close()

    def _close_kortex_resources(self) -> list[Exception]:
        # Retain dependent handles after failure so cleanup can be retried.
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
            self._twist_command = None
        return errors

    def health(self) -> HealthEvent:
        """Report the same decoded-feedback state used by the command gate."""

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

    def reset(self):
        super().reset()
        self._prepared_transport_payload = None

        if self._filter_chain is not None:
            zeros = torch.zeros((self.ACTION_DIM,), dtype=torch.float32)
            self._filter_chain.reset(None, zeros)

        if self._contact_force_filter is not None:
            self._contact_force_filter.reset(
                None, torch.zeros((3,), dtype=torch.float32)
            )
        self._contact_force_filter_recovery_pending = False
        self._contact_force_filter_reset_error = None

        self.obs = self._empty_obs()
        self._last_feedback_stamp = None
        self._last_feedback_error = None
        self._feedback_clock_error = None
        self._feedback_clock_recovery_pending = False

        self._send_zero_twist(required=False)
        self._emitted_action.fill(0.0)

    def pause(self) -> None:
        """Pause means an immediate physical zero command, not retained motion."""

        super().pause()
        self._prepared_transport_payload = None
        if self._filter_chain is not None:
            self._filter_chain.reset(
                None, torch.zeros((self.ACTION_DIM,), dtype=torch.float32)
            )
        self._send_zero_twist(required=self.is_started)
        self._emitted_action.fill(0.0)

    def sync(self):
        super().sync()
        if self._kortex_base_cyclic is None:
            # Preserve the last accepted frame; the action gate requires current feedback.
            self._last_feedback_error = "Kortex feedback transport is unavailable"
            return

        try:
            feedback = self._kortex_base_cyclic.RefreshFeedback()
        except Exception as e:  # noqa: BLE001 - transient network issues
            logging.warning(f"[{self.name}] RefreshFeedback failed: {e}")
            self._last_feedback_error = f"RefreshFeedback failed: {e}"
            return

        # Publish only a complete frame; malformed feedback must not overwrite accepted state.
        next_obs = self._empty_obs()

        n = int(self.cfg.num_joints)
        joint_pos_rad = np.zeros(n, dtype=np.float32)
        joint_vel_rad = np.zeros(n, dtype=np.float32)
        try:
            for i in range(n):
                act = feedback.actuators[i]
                position = float(act.position)
                velocity = float(act.velocity)
                if not math.isfinite(position) or not math.isfinite(velocity):
                    raise ValueError(
                        f"actuator {i} feedback contains a non-finite value"
                    )
                pos_rad = math.radians(position)
                vel_rad = math.radians(velocity)
                with np.errstate(over="ignore", invalid="ignore"):
                    joint_pos_rad[i] = _wrap_to_pi(pos_rad)
                    joint_vel_rad[i] = vel_rad
        except (IndexError, AttributeError, TypeError, ValueError, OverflowError) as e:
            logging.warning(f"[{self.name}] Joint feedback decode failed: {e}")
            self._last_feedback_error = f"joint feedback decode failed: {e}"
            return

        next_obs["proprio/joint_pos_normalized"] = _normalize_joint_positions(
            joint_pos_rad,
            np.asarray(self.cfg.joint_limits_lower, dtype=np.float32),
            np.asarray(self.cfg.joint_limits_upper, dtype=np.float32),
        )
        next_obs["proprio/joint_vel"] = joint_vel_rad

        # Kortex tool pose uses metres and Euler angles in degrees.
        try:
            base_fb = feedback.base
            tool_pos_m = np.array(
                (
                    float(base_fb.tool_pose_x),
                    float(base_fb.tool_pose_y),
                    float(base_fb.tool_pose_z),
                ),
                dtype=np.float32,
            )
            tool_rpy_rad = np.array(
                (
                    math.radians(float(base_fb.tool_pose_theta_x)),
                    math.radians(float(base_fb.tool_pose_theta_y)),
                    math.radians(float(base_fb.tool_pose_theta_z)),
                ),
                dtype=np.float32,
            )
            if not np.all(np.isfinite(tool_pos_m)) or not np.all(
                np.isfinite(tool_rpy_rad)
            ):
                raise ValueError("tool pose contains a non-finite value")
        except (AttributeError, TypeError, ValueError, OverflowError) as e:
            logging.warning(f"[{self.name}] Tool-pose feedback decode failed: {e}")
            self._last_feedback_error = f"tool-pose feedback decode failed: {e}"
            return

        rotmat = _rpy_xyz_to_rotmat(tool_rpy_rad[0], tool_rpy_rad[1], tool_rpy_rad[2])
        next_obs["proprio/fk_pos_end_effector_base"] = tool_pos_m
        next_obs["proprio/fk_rot6d_end_effector_base"] = _rotmat_to_rot6d(rotmat)

        # Require the onboard wrench: zero substitution would fabricate physical state.
        try:
            raw_force = np.array(
                (
                    float(base_fb.tool_external_wrench_force_x),
                    float(base_fb.tool_external_wrench_force_y),
                    float(base_fb.tool_external_wrench_force_z),
                ),
                dtype=np.float32,
            )
            if not np.all(np.isfinite(raw_force)):
                raise ValueError("tool wrench contains a non-finite value")
        except (AttributeError, TypeError, ValueError, OverflowError) as e:
            logging.warning(
                f"[{self.name}] Kortex feedback has no "
                f"tool_external_wrench_force_{{x,y,z}} fields "
                f"or returned invalid values: {e}"
            )
            self._last_feedback_error = f"tool-wrench feedback decode failed: {e}"
            return

        feedback_stamp, clock_error = self._feedback_clock_sample()
        if clock_error is not None:
            logging.warning(f"[{self.name}] {clock_error}")
            self._feedback_clock_error = clock_error
            self._feedback_clock_recovery_pending = True
            self._last_feedback_error = None
            return

        if not self._prepare_contact_force_filter_for_sample():
            return

        if self._contact_force_filter is not None:
            try:
                filtered_force = self._contact_force_filter.step(
                    torch.as_tensor(raw_force, dtype=torch.float32, device="cpu")
                )
                next_obs["proprio_dyn/contact_force_scoop"] = (
                    filtered_force.detach().cpu().numpy().astype(np.float32)
                )
            except Exception as exc:  # noqa: BLE001 - quarantine failed filter
                self._record_contact_filter_failure(
                    f"contact-force filter failed: {exc}"
                )
                return
        else:
            next_obs["proprio_dyn/contact_force_scoop"] = raw_force

        if not all(np.all(np.isfinite(value)) for value in next_obs.values()):
            self._record_contact_filter_failure(
                "decoded Kortex feedback contains a non-finite float32 observation"
            )
            return

        self.obs = next_obs
        self._last_feedback_stamp = feedback_stamp
        self._last_feedback_error = None
        self._feedback_clock_error = None
        self._feedback_clock_recovery_pending = False
        self._contact_force_filter_recovery_pending = False
        self._contact_force_filter_reset_error = None

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                self.ACTION_KEY: gymnasium.spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(self.ACTION_DIM,),
                    dtype=np.float32,
                )
            }
        )

    def apply_action(self, action: dict[str, np.ndarray]) -> None:
        """Prepare and commit one transformed Cartesian command."""

        self.commit_prepared_action(action, self.prepare_action(action))

    def prepare_action(self, action: dict[str, np.ndarray]) -> PreparedHardwareAction:
        """Validate and stage a complete Cartesian command.

        Preparation advances only a deep copy of the action filters and builds
        the final float32 Kortex payload without touching the reusable SDK
        message. The token's send callback installs neither filter state nor
        the emitted report; those changes happen only after a normal RPC return.
        """

        if self.ACTION_KEY not in action:
            raise ValueError(f"[{self.name}] Missing action key '{self.ACTION_KEY}'")
        raw_value = action[self.ACTION_KEY]
        try:
            raw = np.asarray(raw_value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"[{self.name}] Action is not a real numeric array"
            ) from exc
        if raw.shape != (self.ACTION_DIM,):
            raise ValueError(
                f"[{self.name}] Action shape mismatch: "
                f"expected ({self.ACTION_DIM},), got {raw.shape}"
            )
        if raw.dtype.kind not in "iuf":
            raise ValueError(f"[{self.name}] Action must contain real numeric values")
        if not np.all(np.isfinite(raw)):
            raise ValueError(f"[{self.name}] Action contains non-finite values")
        with np.errstate(over="ignore", invalid="ignore"):
            raw_float32 = raw.astype(np.float32, copy=False)
        if not np.all(np.isfinite(raw_float32)):
            raise ValueError(f"[{self.name}] Action contains non-finite values")
        super().prepare_action(action)

        if self._filter_chain is None or self._dt_s is None:
            raise RuntimeError(f"[{self.name}] prepare_action called before start()")
        dt_s = float(self._dt_s)
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian action period must be finite and positive",
                component=self.name,
            )

        self._require_feedback_ready()
        # Preflight before aggregate commit; validation does not mutate the reusable message.
        self._validate_command_transport()
        action_scale = getattr(self, "_action_scale", {})
        extra_scale = self._validated_scale(action_scale, self.ACTION_KEY, 1.0)
        pos_scale = self._validated_scale(
            action_scale,
            f"{self.ACTION_KEY}_position",
            1.0,
        )
        rot_scale = self._validated_scale(
            action_scale,
            f"{self.ACTION_KEY}_orientation",
            1.0,
        )

        # Preparation advances a private copy; live filter state changes after RPC success.
        staged_chain = self._copy_action_filter_chain()
        raw_t = torch.as_tensor(raw_float32, dtype=torch.float32, device="cpu")
        raw_t = raw_t * extra_scale
        scaled = self._scale_to_limits(raw_t)
        if not bool(torch.isfinite(scaled).all()):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian scaled action is non-finite",
                component=self.name,
            )
        filtered = staged_chain.step(scaled)
        if (
            not isinstance(filtered, torch.Tensor)
            or filtered.shape != (self.ACTION_DIM,)
            or not filtered.is_floating_point()
            or not bool(torch.isfinite(filtered).all())
        ):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=("Kortex Cartesian filter returned an invalid final command"),
                component=self.name,
            )
        factors = torch.tensor(
            (pos_scale, pos_scale, pos_scale, rot_scale, rot_scale, rot_scale),
            dtype=filtered.dtype,
            device=filtered.device,
        )
        report_t = filtered * factors
        if not bool(torch.isfinite(report_t).all()):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian filtered report is non-finite",
                component=self.name,
            )
        # Reapply per-axis limits after filters and scales; float32 bounds round down
        # so reported deltas cannot exceed configured limits.
        delta_limits_np = self._delta_limits()
        delta_limits = torch.as_tensor(
            delta_limits_np, dtype=report_t.dtype, device=report_t.device
        )
        report_t = torch.maximum(torch.minimum(report_t, delta_limits), -delta_limits)
        try:
            report = report_t.detach().cpu().numpy().astype(np.float32, copy=True)
        except (TypeError, ValueError, OverflowError) as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=f"Kortex Cartesian report is not float32-representable: {exc}",
                component=self.name,
            ) from exc
        if report.shape != (self.ACTION_DIM,) or not np.all(np.isfinite(report)):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian filtered report is non-finite",
                component=self.name,
            )
        # Recheck after narrowing to float32.
        report = np.clip(report, -delta_limits_np, delta_limits_np)

        # Preserve float32 period division; Kortex angular rates use degrees/s.
        with np.errstate(over="ignore", invalid="ignore"):
            dt_float32 = np.float32(dt_s)
        if not np.isfinite(dt_float32) or dt_float32 <= 0.0:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian action period is not float32-representable",
                component=self.name,
            )
        twist_si = report / dt_float32
        transport_payload = self._prepare_transport_payload(twist_si)
        transport_payload = self._validate_transport_payload(transport_payload)

        def send() -> None:
            # Recheck feedback before RPC; preparation may make it stale.
            # Keep the _send_twist override seam for prepared payloads.
            self._require_feedback_ready()
            self._validate_command_transport()
            self._prepared_transport_payload = transport_payload
            try:
                self._send_twist(twist_si)
            finally:
                self._prepared_transport_payload = None

        def commit() -> None:
            self._commit_action_filter_state(staged_chain)
            self._emitted_action[...] = report

        return PreparedHardwareAction(self, send=send, commit=commit)

    def _require_feedback_ready(self) -> None:
        feedback_state, feedback_age_s = self._feedback_state()
        if feedback_state != "healthy":
            raise SafetyViolationError(
                code=f"{self.name}.feedback_{feedback_state}",
                message=(
                    "refusing to send Kortex Cartesian twist without current "
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
    def _validated_scale(scales: dict[str, float], key: str, default: float) -> float:
        value = scales[key] if key in scales else default
        try:
            value_float = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"action scale {key!r} is not numeric") from exc
        if not math.isfinite(value_float):
            raise ValueError(f"action scale {key!r} is not finite")
        return value_float

    @staticmethod
    def _clone_action_filter(value: object) -> object:
        """Clone one action filter while detaching every mutable tensor."""

        cloned = copy.copy(value)
        for name, state in vars(value).items():
            if isinstance(state, torch.Tensor):
                setattr(cloned, name, state.detach().clone())
            else:
                setattr(cloned, name, copy.deepcopy(state))
        return cloned

    def _copy_action_filter_chain(self) -> CompositeFilter:
        """Return a private chain copy without copying adapter or SDK state."""

        if self._filter_chain is None:
            raise RuntimeError(f"[{self.name}] action filters are not initialized")
        chain = copy.copy(self._filter_chain)
        children = getattr(self._filter_chain, "filters", None)
        if children is not None:
            chain.filters = [self._clone_action_filter(child) for child in children]
        return chain

    @staticmethod
    def _copy_filter_attributes(current: object, staged: object) -> None:
        """Install staged state while retaining live tensor/object references."""

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

    def _commit_action_filter_state(self, staged_chain: CompositeFilter) -> None:
        """Commit staged children without replacing the live chain."""

        live_chain = self._filter_chain
        current_children = getattr(live_chain, "filters", None)
        staged_children = getattr(staged_chain, "filters", None)
        if live_chain is None or current_children is None or staged_children is None:
            raise RuntimeError(
                f"[{self.name}] action filter chain changed during command"
            )
        if len(current_children) != len(staged_children):
            raise RuntimeError(
                f"[{self.name}] action filter chain changed during command"
            )
        for current, staged in zip(current_children, staged_children, strict=True):
            self._copy_filter_attributes(current, staged)
        if len(current_children) >= 3:
            self._rate_limiter = current_children[0]
            self._smooth_filter = current_children[1]
            self._jerk_limiter = current_children[2]

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs

    @property
    def emitted_action(self) -> dict[str, np.ndarray]:
        """Return the last successfully submitted SI per-step Cartesian delta."""

        return {self.ACTION_KEY: self._emitted_action.copy()}

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self._empty_obs())

    def _feedback_state(self) -> tuple[str, float | None]:
        if self._kortex_base_cyclic is None:
            return "unavailable", None
        if self._feedback_clock_recovery_pending:
            return "clock_invalid", None
        if (
            self._last_feedback_error is not None
            or self._contact_force_filter_recovery_pending
        ):
            return "failure", None
        if self._last_feedback_stamp is None:
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
        if now_s < self._last_feedback_stamp:
            self._feedback_clock_error = (
                "Kortex feedback clock moved backwards from "
                f"{self._last_feedback_stamp:.9f} to {now_s:.9f}"
            )
            self._feedback_clock_recovery_pending = True
            return "clock_invalid", None
        age_s = now_s - self._last_feedback_stamp
        if age_s > _FEEDBACK_MAX_AGE_S:
            return "stale", age_s
        return "healthy", age_s

    def _feedback_clock_sample(self) -> tuple[float | None, str | None]:
        """Read a finite receipt time that does not precede the prior sample."""

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
        if self._last_feedback_stamp is not None and stamp < self._last_feedback_stamp:
            return None, (
                "Kortex feedback clock moved backwards from "
                f"{self._last_feedback_stamp:.9f} to {stamp:.9f}"
            )
        return stamp, None

    def _prepare_contact_force_filter_for_sample(self) -> bool:
        """Reset a quarantined contact filter before accepting a new sample."""

        if self._contact_force_filter is None:
            self._contact_force_filter_recovery_pending = False
            self._contact_force_filter_reset_error = None
            return True
        if not self._contact_force_filter_recovery_pending:
            return True
        _, error = attempt_cleanup(
            lambda: self._contact_force_filter.reset(
                None, torch.zeros((3,), dtype=torch.float32)
            )
        )
        if error is not None:
            self._contact_force_filter_reset_error = (
                f"contact-force filter reset failed: {error}"
            )
            self._last_feedback_error = self._contact_force_filter_reset_error
            if not isinstance(error, Exception):
                raise error
            return False
        self._contact_force_filter_reset_error = None
        return True

    def _record_contact_filter_failure(self, message: str) -> None:
        """Quarantine the contact filter and retain the last accepted frame."""

        self._contact_force_filter_recovery_pending = (
            self._contact_force_filter is not None
        )
        self._contact_force_filter_reset_error = None
        if self._contact_force_filter is not None:
            _, error = attempt_cleanup(
                lambda: self._contact_force_filter.reset(
                    None, torch.zeros((3,), dtype=torch.float32)
                )
            )
            if error is not None:
                self._contact_force_filter_reset_error = (
                    f"contact-force filter reset failed: {error}"
                )
                if not isinstance(error, Exception):
                    self._last_feedback_error = (
                        f"{message}; {self._contact_force_filter_reset_error}"
                    )
                    raise error
        if self._contact_force_filter_reset_error is not None:
            message = f"{message}; {self._contact_force_filter_reset_error}"
        self._last_feedback_error = message

    def _build_filters(self, dt_s: float) -> None:
        """Build filters; the action chain must match simulation bit-for-bit."""
        dim = (self.ACTION_DIM,)
        device = "cpu"
        for name in ("delta_pos_limit_m", "delta_rot_limit_rad"):
            value = float(getattr(self.cfg, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"[{self.name}] {name} must be finite and non-negative, got {value}"
                )
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

        # Force observation smoothing is separate from the action filter chain.
        self._contact_force_filter = CriticallyDampedFilter(
            (3,),
            tau_s=self.cfg.contact_force_filter_tau_s,
            dt_s=dt_s,
            device=device,
        )
        self._contact_force_filter_recovery_pending = False
        self._contact_force_filter_reset_error = None

    def _delta_limits(self) -> np.ndarray:
        """Return per-step position/rotation bounds rounded down to float32."""
        pos = float(self.cfg.delta_pos_limit_m)
        rot = float(self.cfg.delta_rot_limit_rad)
        if not (math.isfinite(pos) and math.isfinite(rot) and pos >= 0 and rot >= 0):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    "Kortex Cartesian delta_pos_limit_m/delta_rot_limit_rad must be "
                    "finite and non-negative"
                ),
                component=self.name,
            )
        exact = np.array((pos, pos, pos, rot, rot, rot), dtype=np.float64)
        with np.errstate(over="ignore", invalid="ignore"):
            rounded = exact.astype(np.float32)
        above = rounded.astype(np.float64) > exact
        return np.where(above, np.nextafter(rounded, np.float32(0.0)), rounded).astype(
            np.float32
        )

    def _scale_to_limits(self, raw: torch.Tensor) -> torch.Tensor:
        """Map normalized policy output to physical units and clamp."""
        dim = raw.shape[-1]
        limits = torch.empty(dim, device=raw.device, dtype=raw.dtype)
        limits[:3] = self.cfg.delta_pos_limit_m
        limits[3:6] = self.cfg.delta_rot_limit_rad
        if dim > 6:  # Defensive — this interface only advertises dim=6.
            limits[6:] = 1.0
        return raw.clamp(-1.0, 1.0) * limits

    def _zero_twist(self) -> None:
        """Mutate the prebuilt twist command to all-zero."""
        if self._twist_command is None:
            return
        tw = self._twist_command.twist
        tw.linear_x = 0.0
        tw.linear_y = 0.0
        tw.linear_z = 0.0
        tw.angular_x = 0.0
        tw.angular_y = 0.0
        tw.angular_z = 0.0

    def _send_zero_twist(self, *, required: bool) -> None:
        if self._kortex_base is None or self._twist_command is None:
            if required:
                raise SafetyViolationError(
                    code=f"{self.name}.safe_stop_transport_unavailable",
                    message="cannot safe-stop: Kortex Cartesian transport is unavailable",
                    component=self.name,
                )
            return
        self._zero_twist()
        try:
            self._kortex_base.SendTwistCommand(self._twist_command)
        except Exception as exc:  # noqa: BLE001 - a failed stop stays retryable
            raise SafetyViolationError(
                code=f"{self.name}.safe_stop_failed",
                message=f"Kortex Cartesian safe stop failed: {exc}",
                component=self.name,
                details={"exception_type": type(exc).__name__},
            ) from exc

    def _validate_command_transport(self) -> tuple[Any, Any, Any]:
        """Validate SDK command readiness without mutating the command."""

        base = self._kortex_base
        command = self._twist_command
        if base is None or command is None:
            raise SafetyViolationError(
                code=f"{self.name}.command_transport_unavailable",
                message="Kortex Cartesian command transport is unavailable",
                component=self.name,
            )
        try:
            send_command = getattr(base, "SendTwistCommand")
        except (AttributeError, TypeError, ValueError) as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_transport_unavailable",
                message="Kortex Cartesian command transport has no SendTwistCommand",
                component=self.name,
            ) from exc
        if not callable(send_command):
            raise SafetyViolationError(
                code=f"{self.name}.command_transport_unavailable",
                message="Kortex Cartesian SendTwistCommand is not callable",
                component=self.name,
            )
        try:
            twist = getattr(command, "twist")
        except (AttributeError, TypeError, ValueError) as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian command has no twist message",
                component=self.name,
            ) from exc
        missing: list[str] = []
        for field in (
            "linear_x",
            "linear_y",
            "linear_z",
            "angular_x",
            "angular_y",
            "angular_z",
        ):
            try:
                getattr(twist, field)
            except (AttributeError, TypeError, ValueError):
                missing.append(field)
        if missing:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    "Kortex Cartesian twist message is missing fields: "
                    + ", ".join(missing)
                ),
                component=self.name,
            )
        return base, command, twist

    def _prepare_transport_payload(self, twist: np.ndarray) -> np.ndarray:
        """Build the final finite float32 Kortex payload without side effects."""

        try:
            twist_values = np.asarray(twist)
        except (TypeError, ValueError, OverflowError) as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=f"Kortex Cartesian twist is not numeric: {exc}",
                component=self.name,
            ) from exc
        if twist_values.shape != (self.ACTION_DIM,):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    "Kortex Cartesian twist shape mismatch: "
                    f"expected ({self.ACTION_DIM},), got {twist_values.shape}"
                ),
                component=self.name,
            )
        if twist_values.dtype.kind not in "iuf":
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian twist must contain real numeric values",
                component=self.name,
            )
        if not np.all(np.isfinite(twist_values)):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian twist contains non-finite SI values",
                component=self.name,
            )
        with np.errstate(over="ignore", invalid="ignore"):
            twist_si = twist_values.astype(np.float64, copy=False)
        if not np.all(np.isfinite(twist_si)):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian twist is not finite in float64",
                component=self.name,
            )

        # Convert and narrow all fields before the reusable message is touched.
        with np.errstate(over="ignore", invalid="ignore"):
            transport_values = np.empty(self.ACTION_DIM, dtype=np.float64)
            transport_values[:3] = twist_si[:3]
            transport_values[3:] = np.degrees(twist_si[3:])
            transport_values = transport_values.astype(np.float32)
        if not np.all(np.isfinite(transport_values)):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=(
                    "Kortex Cartesian twist is non-finite after transport "
                    "unit conversion"
                ),
                component=self.name,
            )
        return transport_values

    def _validate_transport_payload(self, payload: Any) -> np.ndarray:
        """Validate the exact float32 wire vector before message mutation."""

        try:
            values = np.asarray(payload)
        except (TypeError, ValueError, OverflowError) as exc:
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message=f"Kortex Cartesian payload is not numeric: {exc}",
                component=self.name,
            ) from exc
        if (
            values.shape != (self.ACTION_DIM,)
            or values.dtype != np.dtype(np.float32)
            or not np.all(np.isfinite(values))
        ):
            raise SafetyViolationError(
                code=f"{self.name}.command_invalid",
                message="Kortex Cartesian payload is not finite float32 data",
                component=self.name,
            )
        return values

    def _send_twist(self, twist: np.ndarray) -> None:
        """Write SI twist values into Kortex fields and send the command.

        A prepared command supplies ``_prepared_transport_payload`` during its
        commit callback. In that path this method sends the already validated
        payload exactly once, without another unit conversion.
        """
        self._validate_command_transport()
        transport_values = self._prepared_transport_payload
        if transport_values is None:
            transport_values = self._prepare_transport_payload(twist)
        transport_values = self._validate_transport_payload(transport_values)
        self._send_transport_payload(transport_values)

    def _send_transport_payload(self, transport_values: np.ndarray) -> None:
        """Write an already validated float32 payload and perform one RPC."""

        transport_values = self._validate_transport_payload(transport_values)
        base, command, tw = self._validate_command_transport()
        tw.linear_x = float(transport_values[0])
        tw.linear_y = float(transport_values[1])
        tw.linear_z = float(transport_values[2])
        tw.angular_x = float(transport_values[3])
        tw.angular_y = float(transport_values[4])
        tw.angular_z = float(transport_values[5])
        try:
            base.SendTwistCommand(command)
        except Exception as e:  # noqa: BLE001 - transient network issues
            raise SafetyViolationError(
                code=f"{self.name}.command_send_failed",
                message=f"SendTwistCommand failed: {e}",
                component=self.name,
                details={"exception_type": type(e).__name__},
            ) from e

    def _empty_obs(self) -> dict[str, np.ndarray]:
        n = int(self.cfg.num_joints)
        return {
            "proprio/joint_pos_normalized": np.zeros(n, dtype=np.float32),
            "proprio/joint_vel": np.zeros(n, dtype=np.float32),
            "proprio/fk_pos_end_effector_base": np.zeros(3, dtype=np.float32),
            # Row-major rot-6D identity = ``np.eye(3)[:, :2].flatten()`` =
            # [1,0,0, 1,0,0], matching ``_rotmat_to_rot6d``.
            "proprio/fk_rot6d_end_effector_base": np.array(
                (1.0, 0.0, 0.0, 1.0, 0.0, 0.0), dtype=np.float32
            ),
            # Low-pass-filtered external tool-wrench force.
            "proprio_dyn/contact_force_scoop": np.zeros(3, dtype=np.float32),
        }


def _rpy_xyz_to_rotmat(rx: float, ry: float, rz: float) -> np.ndarray:
    """Convert Kortex base-frame XYZ Euler angles (radians) to a rotation matrix.

    Uses extrinsic XYZ: ``Rz(rz) @ Ry(ry) @ Rx(rx)``.
    """
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    # R = Rz(rz) @ Ry(ry) @ Rx(rx) — extrinsic XYZ (equiv. intrinsic ZYX).
    r00 = cy * cz
    r01 = sx * sy * cz - cx * sz
    r02 = cx * sy * cz + sx * sz
    r10 = cy * sz
    r11 = sx * sy * sz + cx * cz
    r12 = cx * sy * sz - sx * cz
    r20 = -sy
    r21 = sx * cy
    r22 = cx * cy
    return np.array(
        (
            (r00, r01, r02),
            (r10, r11, r12),
            (r20, r21, r22),
        ),
        dtype=np.float32,
    )


def _rotmat_to_rot6d(rotmat: np.ndarray) -> np.ndarray:
    """Flatten the first two rotation-matrix columns into the SRB rot-6D form."""
    return rotmat[:, :2].reshape(-1).astype(np.float32)
