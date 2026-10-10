"""ROS action subscriptions, callback pumping, and action buffering."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from numbers import Real
from typing import Any

import gymnasium
import torch

from srb.interfaces._ros_node import (
    RosCleanupScope,
    RosEndpointCleanupError,
    attempt_ros_cleanup,
    destroy_ros_endpoint,
)
from srb.interfaces.ros_namespace import RosNamespace

ActionExtractor = Callable[[Any], Sequence[float]]
ActionUpdateKey = tuple[Any, ...]

DEFAULT_COMMAND_TIMEOUT_S = 0.5


def validate_command_timeout(value: object) -> float | None:
    """Validate the ROS ingress freshness timeout."""

    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(
            f"command_timeout_s must be a positive finite number or None, got {value!r}"
        )
    try:
        timeout = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError(
            f"command_timeout_s must be a positive finite number or None, got {value!r}"
        ) from error
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError(
            f"command_timeout_s must be a positive finite number or None, got {value!r}"
        )
    return timeout


def default_action_qos_profile() -> Any:
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _ros_action_messages() -> tuple[type[Any], type[Any], type[Any]]:
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool as BoolMsg
    from std_msgs.msg import Float64MultiArray

    return Twist, BoolMsg, Float64MultiArray


def _validate_action_term(action_term: Any) -> None:
    try:
        from srb.core.action import ActionTerm, ActionTermCfg
    except Exception:
        return

    if not isinstance(action_term, ActionTerm):
        raise TypeError(
            f"ROS action term must be ActionTerm, got {type(action_term).__name__}"
        )
    if not isinstance(action_term.cfg, ActionTermCfg):
        raise TypeError(
            "ROS action term cfg must be ActionTermCfg, got "
            f"{type(action_term.cfg).__name__}"
        )


def _action_term_name_matches(action_term: Any, *class_names: str) -> bool:
    return any(cls.__name__ in class_names for cls in type(action_term).mro())


def _action_term_matches(action_term: Any, *class_names: str) -> bool:
    try:
        from srb.core import action as action_module
    except Exception:
        return _action_term_name_matches(action_term, *class_names)

    action_classes = tuple(
        action_cls
        for class_name in class_names
        if isinstance((action_cls := getattr(action_module, class_name, None)), type)
    )
    if not action_classes:
        return _action_term_name_matches(action_term, *class_names)
    return isinstance(action_term, action_classes)


def _supports_direct_action_command(actions: Any) -> bool:
    if actions is None:
        return False
    from srb.core.action.command import supports_direct_action_command

    return supports_direct_action_command(actions)


class RosActionIO:
    def __init__(
        self,
        *,
        env: Any,
        node: Any,
        namespace: RosNamespace,
        action_pump: Any | None = None,
        owns_action_pump: bool | None = None,
        qos_profile: Any | None = None,
        command_timeout_s: float | None = DEFAULT_COMMAND_TIMEOUT_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if action_pump is None and owns_action_pump is False:
            raise ValueError("borrowed action pump ownership requires an injected pump")
        self._command_timeout_s = validate_command_timeout(command_timeout_s)
        self._clock = clock
        self._last_clock_s: float | None = None
        self._rejection_warnings: set[str] = set()
        self._env = env
        self._node = node
        self._num_envs = int(env.num_envs)
        self._namespace = namespace
        self._qos_profile = (
            default_action_qos_profile() if qos_profile is None else qos_profile
        )
        self._action_subscription_count = 0
        self._subscriptions_configured = False
        self._lifecycle_lock = threading.RLock()
        self._close_in_progress = False
        self._close_requested = False
        self._closed = False
        self._action_pump_released = False
        self._owned_endpoints: list[tuple[Any, str, bool]] = []

        space = self._env.action_space
        if (
            not isinstance(space, gymnasium.spaces.Box)
            or len(space.shape) != 2
            or space.shape[0] != self._num_envs
            or self._num_envs <= 0
            or space.shape[1] <= 0
        ):
            raise ValueError("ROS action ingress requires a batched Box action space")
        self._action_low = torch.tensor(space.low, dtype=torch.float32)
        self._action_high = torch.tensor(space.high, dtype=torch.float32)
        # Float32 rounding must not widen a Box declared at higher precision.
        self._action_low = torch.where(
            self._action_low.double() < torch.tensor(space.low, dtype=torch.float64),
            torch.nextafter(
                self._action_low, torch.full_like(self._action_low, math.inf)
            ),
            self._action_low,
        )
        self._action_high = torch.where(
            self._action_high.double() > torch.tensor(space.high, dtype=torch.float64),
            torch.nextafter(
                self._action_high, torch.full_like(self._action_high, -math.inf)
            ),
            self._action_high,
        )
        if (
            torch.isnan(self._action_low).any()
            or torch.isnan(self._action_high).any()
            or (self._action_low > 0).any()
            or (self._action_high < 0).any()
        ):
            raise ValueError("ROS action bounds must admit a finite zero-action hold")
        self._actions = torch.zeros(
            self._env.action_space.shape, dtype=torch.float32, device=self._env.device
        )
        # CPU mirror written by ROS callbacks; copied to device on the sim thread.
        self._actions_cpu = torch.zeros(
            self._env.action_space.shape, dtype=torch.float32
        )
        self._action_condition = threading.Condition()
        self._action_sequence = 0
        self._applied_action_sequence = 0
        # Expand broadcasts per environment so selective reset preserves others.
        self._pending_action_updates: dict[ActionUpdateKey, tuple[int, float, Any]] = {}
        self._action_stamps = torch.full(space.shape, math.nan, dtype=torch.float64)
        self._action_ids = torch.zeros(space.shape, dtype=torch.int64)
        self._mapped_action_mask = torch.zeros(space.shape, dtype=torch.bool)
        self._actions_cmd_vel = torch.zeros((self._num_envs, 6), dtype=torch.float32)
        self._actions_event = torch.zeros((self._num_envs, 1), dtype=torch.float32)
        self._cmd_vel_stamps = torch.full(
            (self._num_envs,), math.nan, dtype=torch.float64
        )
        self._event_stamps = self._cmd_vel_stamps.clone()
        self._cmd_vel_ids = torch.zeros(self._num_envs, dtype=torch.int64)
        self._event_ids = self._cmd_vel_ids.clone()
        self._reset_generations = [0] * self._num_envs
        self._refresh_owner: int | None = None
        self._consumer_owner: int | None = None
        self._pump_owner: int | None = None

        # Validation and allocation must succeed before a live ROS owner exists.
        created_action_pump = action_pump is None
        if created_action_pump:
            from srb.interfaces._ros_node import acquire_ros_pump

            action_pump = acquire_ros_pump(self._node, suffix="action")
        if action_pump is None:
            raise RuntimeError("ROS action callback pump is unavailable")
        self._action_pump = action_pump
        self._owns_action_pump = (
            created_action_pump if owns_action_pump is None else owns_action_pump
        )

    def close(self) -> None:
        """Detach subscriptions and release the owned callback pump.

        Failed resources stay registered for retry; successful resources are
        never destroyed twice. Closing gates queued callbacks before any
        endpoint is destroyed.
        """
        with self._lifecycle_lock:
            if self._closed:
                return
            if self._close_in_progress:
                raise RosEndpointCleanupError(
                    "ROS action cleanup is already in progress; retry close"
                )
            self._close_in_progress = True
            self._close_requested = True
        try:
            if not self._action_condition.acquire(blocking=False):
                raise RosEndpointCleanupError(
                    "ROS action cleanup is waiting for an active command operation; retry close",
                    cleanup_owner=self,
                )
            try:
                self.clear_actions()
                self._action_condition.notify_all()
                if (
                    self._refresh_owner == threading.get_ident()
                    or self._consumer_owner == threading.get_ident()
                ):
                    raise RosEndpointCleanupError(
                        "ROS action close requested during mapping or consumption; retry after the action operation returns",
                        cleanup_owner=self,
                    )
            finally:
                self._action_condition.release()
            errors: list[BaseException] = []
            if not self._action_pump.lock.acquire(blocking=False):
                raise RosEndpointCleanupError(
                    "ROS action cleanup is waiting for an active callback; retry close"
                )
            try:
                for index, (endpoint, kind, complete) in enumerate(
                    self._owned_endpoints
                ):
                    if complete:
                        continue
                    error = attempt_ros_cleanup(
                        lambda: destroy_ros_endpoint(
                            self._action_pump.node,
                            endpoint,
                            kind=kind,
                        )
                    )
                    if error is not None:
                        errors.append(error)
                    else:
                        self._owned_endpoints[index] = (endpoint, kind, True)
            finally:
                self._action_pump.lock.release()

            release = getattr(self._action_pump, "release", None)
            if (
                self._owns_action_pump
                and callable(release)
                and not self._action_pump_released
                and not errors
            ):
                error = attempt_ros_cleanup(release)
                if error is not None:
                    errors.append(error)
                else:
                    self._action_pump_released = True

            if errors:
                raise RosEndpointCleanupError(
                    "ROS action endpoint cleanup incomplete",
                    errors=tuple(errors),
                )
            self._closed = True
        finally:
            with self._lifecycle_lock:
                self._close_in_progress = False

    @property
    def subscriptions_configured(self) -> bool:
        return self._subscriptions_configured

    @property
    def action(self) -> torch.Tensor:
        action, _ = self.action_with_sequence()
        return action

    @property
    def action_sequence(self) -> int:
        with self._action_condition:
            return self._applied_action_sequence

    def action_with_sequence(self, *, block: bool = False) -> tuple[torch.Tensor, int]:
        self._reject_reentrant_consumption()
        action_poll_budget = max(1, self._action_subscription_count)
        self._spin_action_callbacks(
            max_callbacks=action_poll_budget,
            timeout_sec=0.0,
        )
        if not self._action_condition.acquire(blocking=block):
            # Without the guard, freshness is unknown; never reuse unchecked motion.
            return torch.zeros_like(self._actions), self._applied_action_sequence
        try:
            self._refresh_actions_locked()
            return self._actions, self._applied_action_sequence
        finally:
            self._action_condition.release()

    @contextmanager
    def action_transaction(
        self, *, block: bool = False
    ) -> Iterator[tuple[torch.Tensor, int]]:
        """Keep a borrowed action valid through synchronous consumer submission.

        CUDA work submitted by the consumer completes before buffer reuse.
        The caller must not retain the alias for dispatch after this context.
        """
        self._reject_reentrant_consumption()
        self._spin_action_callbacks(
            max_callbacks=max(1, self._action_subscription_count), timeout_sec=0.0
        )
        if not self._action_condition.acquire(blocking=block):
            yield torch.zeros_like(self._actions), self._applied_action_sequence
            return
        try:
            self._refresh_actions_locked()
            self._consumer_owner = threading.get_ident()
            succeeded = False
            with RosCleanupScope(
                self,
                "ROS action consumption completion incomplete",
                always=True,
                cleanup=lambda: self._finish_action_consumption(succeeded=succeeded),
            ):
                yield self._actions, self._applied_action_sequence
                succeeded = True
        finally:
            self._consumer_owner = None
            self._action_condition.release()

    def _reject_reentrant_consumption(self) -> None:
        if threading.get_ident() in (
            self._consumer_owner,
            self._refresh_owner,
            self._pump_owner,
        ):
            raise RuntimeError("ROS action consumption cannot be re-entered")

    def _finish_action_consumption(self, *, succeeded: bool) -> None:
        # The cleanup scope may retry this callback after its first attempt.
        with self._action_condition:
            if not succeeded:
                self._close_requested = True
            error = attempt_ros_cleanup(self._synchronize_actions)
            if error is not None:
                self._close_requested = True
                raise error
            if self._close_requested:
                self.clear_actions()

    def _synchronize_actions(self) -> None:
        if self._actions.device.type == "cuda":
            error = attempt_ros_cleanup(
                lambda: torch.cuda.synchronize(self._actions.device)
            )
            if error is not None:
                self._close_requested = True
                raise RosEndpointCleanupError(
                    "ROS action device completion incomplete; retry close",
                    errors=(error,),
                    cleanup_owner=self,
                ) from error

    def _refresh_actions_locked(self) -> None:
        if self._refresh_owner is not None or self._consumer_owner is not None:
            raise RuntimeError("ROS action publication cannot be re-entered")
        self._refresh_owner = threading.get_ident()
        try:
            self._refresh_actions_transaction_locked()
        except BaseException as error:
            # Clear published motion before propagating mapper or transport errors.
            cleanup_error = attempt_ros_cleanup(self.clear_actions)
            if cleanup_error is not None:
                error.add_note(f"ROS action clearing also failed: {cleanup_error}")
            raise
        finally:
            self._refresh_owner = None

    def _refresh_actions_transaction_locked(self) -> None:
        if self._close_requested:
            self.clear_actions()
            return
        now = self._read_clock_locked()
        if now is None:
            return
        self._expire_actions_locked(now, remap=True)
        self._apply_pending_action_updates_locked(now)
        now = self._read_clock_locked()
        if now is None or self._close_requested:
            if self._close_requested:
                self.clear_actions()
            return
        self._expire_actions_locked(now, remap=True)
        # Mapping may re-enter lifecycle code; this final check must not renew input.
        now = self._read_clock_locked()
        if now is None or self._close_requested:
            if self._close_requested:
                self.clear_actions()
            return
        self._expire_actions_locked(now, remap=False)
        self._publish_actions_locked()
        if self._close_requested:
            self.clear_actions()
            return
        self._applied_action_sequence = max(
            self._applied_action_sequence, int(self._action_ids.max())
        )

    def _publish_actions_locked(self) -> None:
        # The blocking copy finishes under the guard; reset cannot publish a stale
        # detached CPU snapshot after clearing actions.
        self._actions.copy_(self._actions_cpu, non_blocking=False)

    def _read_clock_locked(self) -> float | None:
        try:
            value = self._clock()
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError("invalid monotonic clock")
            now = float(value)
            if not math.isfinite(now) or (
                self._last_clock_s is not None and now < self._last_clock_s
            ):
                raise ValueError("invalid monotonic clock")
        except (ArithmeticError, TypeError, ValueError, RuntimeError):
            self.clear_actions()
            self._warn_rejected(
                "clock", "Invalid ROS command clock; holding zero action"
            )
            return None
        except BaseException:
            self.clear_actions()
            raise
        self._last_clock_s = now
        return now

    def _fresh(self, stamps: torch.Tensor, now: float) -> torch.Tensor:
        valid = torch.isfinite(stamps) & (stamps <= now)
        if self._command_timeout_s is not None:
            valid &= now - stamps <= self._command_timeout_s
        return valid

    def wait_for_action(self, sequence: int, timeout: float | None = None) -> bool:
        self._reject_reentrant_consumption()
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            if self._close_requested:
                return False
            with self._action_condition:
                self._refresh_actions_locked()
                if self._close_requested:
                    return False
                if (self._action_ids > sequence).any():
                    return True

            spin_timeout = 0.05
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                spin_timeout = min(spin_timeout, remaining)

            action_poll_budget = max(1, self._action_subscription_count)
            self._spin_action_callbacks(
                max_callbacks=action_poll_budget,
                timeout_sec=spin_timeout,
            )

    def clear_actions(self, env_mask: torch.Tensor | None = None) -> None:
        with self._action_condition:
            if env_mask is None:
                env_mask = torch.ones(self._num_envs, dtype=torch.bool)
            if env_mask.dtype != torch.bool or env_mask.shape != (self._num_envs,):
                raise ValueError(
                    "ROS action reset mask must be one boolean per environment"
                )
            self._synchronize_actions()
            mask = env_mask.cpu()
            self._actions_cpu[mask] = 0.0
            self._action_stamps[mask] = math.nan
            self._action_ids[mask] = 0
            self._mapped_action_mask[mask] = False
            self._clear_mapped_action_state(mask)
            env_ids = set(mask.nonzero(as_tuple=False).flatten().tolist())
            for env_id in env_ids:
                self._reset_generations[env_id] += 1
            self._pending_action_updates = {
                key: update
                for key, update in self._pending_action_updates.items()
                if key[1] not in env_ids
            }
            # Do not publish other rows that a reentrant mapper may have staged.
            self._actions[mask.to(device=self._actions.device)] = 0.0
            self._synchronize_actions()
            self._action_condition.notify_all()

    def setup_subscriptions(self) -> None:
        with self._lifecycle_lock:
            if self._close_requested:
                return
            self._setup_subscriptions()

    def _setup_subscriptions(self) -> None:
        if self._subscriptions_configured:
            # Re-entry duplicates subscriptions and sequence increments.
            return

        Twist, BoolMsg, Float64MultiArray = _ros_action_messages()

        self._action_subscription_count = 0
        self._sub_action_term_all: dict[str, Any] = {}
        self._sub_action_term: dict[str, Sequence[Any]] = {}
        action_offset = 0
        for action_key, action_term in self._env.action_manager.iter_terms():
            _validate_action_term(action_term)
            topic_base = (
                f"{action_term.cfg.asset_name}/{action_key.rsplit('__', 1)[-1]}"
            )
            msg_type, msg_extractor = self._map_action_term(action_term)
            action_dim = action_term.action_dim
            if (
                msg_type is not Float64MultiArray
                and len(msg_extractor(msg_type())) != action_dim
            ):
                raise ValueError(
                    f"Action term '{action_key}' maps to {msg_type.__name__} with "
                    f"{len(msg_extractor(msg_type()))} extracted values, but the "
                    f"term expects action_dim={action_dim}"
                )

            envs_topic = self._namespace.envs_topic(topic_base)
            if envs_topic is not None:
                self._sub_action_term_all[action_key] = (
                    self._create_action_subscription(
                        msg_type=msg_type,
                        topic=envs_topic,
                        callback=self._create_cb_action_any(
                            dim=action_dim,
                            key=action_key,
                            extractor=msg_extractor,
                            offset=action_offset,
                        ),
                    )
                )
            self._sub_action_term[action_key] = tuple(
                self._create_action_subscription(
                    msg_type=msg_type,
                    topic=self._namespace.env_topic(i, topic_base),
                    callback=self._create_cb_action_any(
                        key=action_key,
                        extractor=msg_extractor,
                        dim=action_dim,
                        offset=action_offset,
                        env_id=i,
                    ),
                )
                for i in range(self._num_envs)
            )

            action_offset += action_dim

        if _supports_direct_action_command(getattr(self._env.cfg, "actions", None)):
            self._actions_cmd_vel = torch.zeros(
                (self._num_envs, 6), dtype=torch.float32
            )
            self._actions_event = torch.zeros((self._num_envs, 1), dtype=torch.float32)
            envs_cmd_vel_topic = self._namespace.envs_topic("action/cmd_vel")
            if envs_cmd_vel_topic is not None:
                self._sub_action_cmd_vel_all = self._create_action_subscription(
                    Twist,
                    envs_cmd_vel_topic,
                    callback=self._create_cb_action_cmd_vel(),
                )
            self._sub_action_cmd_vel = tuple(
                self._create_action_subscription(
                    Twist,
                    self._namespace.env_topic(i, "action/cmd_vel"),
                    callback=self._create_cb_action_cmd_vel(env_id=i),
                )
                for i in range(self._num_envs)
            )
            envs_event_topic = self._namespace.envs_topic("action/event")
            if envs_event_topic is not None:
                self._sub_action_event_all = self._create_action_subscription(
                    BoolMsg,
                    envs_event_topic,
                    callback=self._create_cb_action_event(),
                )
            self._sub_action_event = tuple(
                self._create_action_subscription(
                    BoolMsg,
                    self._namespace.env_topic(i, "action/event"),
                    callback=self._create_cb_action_event(env_id=i),
                )
                for i in range(self._num_envs)
            )

        action_dim = self._actions.shape[-1]
        envs_raw_topic = self._namespace.envs_topic("action/raw")
        if envs_raw_topic is not None:
            self._sub_action_raw_all = self._create_action_subscription(
                Float64MultiArray,
                envs_raw_topic,
                callback=self._create_cb_action_raw(
                    action_dim=action_dim,
                ),
            )
        self._sub_action_raw = tuple(
            self._create_action_subscription(
                Float64MultiArray,
                self._namespace.env_topic(i, "action/raw"),
                callback=self._create_cb_action_raw(
                    action_dim=action_dim,
                    env_id=i,
                ),
            )
            for i in range(self._num_envs)
        )
        self._subscriptions_configured = True

    def _spin_action_callbacks(
        self, *, max_callbacks: int, timeout_sec: float | None
    ) -> int:
        if max_callbacks <= 0 or self._close_requested:
            return 0
        if not self._action_pump.lock.acquire(blocking=False):
            return 0
        callbacks = 0
        self._pump_owner = threading.get_ident()
        try:
            for i in range(max_callbacks):
                if not self._action_condition.acquire(blocking=False):
                    break
                try:
                    if self._close_requested:
                        break
                    sequence_before = self._action_sequence
                    # Callbacks re-enter this condition; release would let another
                    # reader take the guard and block this nonblocking pump.
                    self._action_pump.executor.spin_once(
                        timeout_sec=timeout_sec if i == 0 else 0.0
                    )
                    sequence_after = self._action_sequence
                finally:
                    self._action_condition.release()
                if sequence_after == sequence_before:
                    break
                callbacks += 1
        finally:
            self._pump_owner = None
            self._action_pump.lock.release()
        return callbacks

    def _queue_action_update(self, key: ActionUpdateKey, payload: Any) -> None:
        if self._close_requested:
            return
        with self._action_condition:
            if self._close_requested:
                return
            kind, env_id = key[:2]
            if kind == "event":
                if not isinstance(payload, bool):
                    self._warn_rejected(
                        "payload", "Rejected malformed ROS action payload"
                    )
                    return
            else:
                expected_dim = (
                    key[3] - key[2]
                    if kind == "term"
                    else 6
                    if kind == "cmd_vel"
                    else self._actions_cpu.shape[1]
                )
                try:
                    values = tuple(float(value) for value in payload)
                    if len(values) != expected_dim or not all(
                        math.isfinite(value) for value in values
                    ):
                        raise ValueError("invalid numeric payload")
                    payload = torch.tensor(values, dtype=torch.float32)
                    if not torch.isfinite(payload).all():
                        raise ValueError("float32 overflow")
                except (ArithmeticError, TypeError, ValueError, RuntimeError):
                    self._warn_rejected(
                        "payload", "Rejected malformed or non-finite ROS action payload"
                    )
                    return
            now = self._read_clock_locked()
            if now is None:
                return
            self._action_sequence += 1
            env_ids = range(self._num_envs) if env_id is None else (env_id,)
            for target in env_ids:
                self._pending_action_updates[(kind, target, *key[2:])] = (
                    self._action_sequence,
                    now,
                    payload,
                )
            self._action_condition.notify_all()

    def _warn_rejected(self, category: str, message: str) -> None:
        # Use owner-defined categories and bound repeated malformed-message logs.
        if category not in self._rejection_warnings:
            self._rejection_warnings.add(category)
            self._node.get_logger().warn(message)

    def _clear_mapped_action_state(self, env_mask: torch.Tensor | None = None) -> None:
        mask = slice(None) if env_mask is None else env_mask.cpu()
        self._actions_cmd_vel[mask] = 0.0
        self._actions_event[mask] = 0.0
        self._cmd_vel_stamps[mask] = math.nan
        self._event_stamps[mask] = math.nan
        self._cmd_vel_ids[mask] = 0
        self._event_ids[mask] = 0

    def _bounded_action(self, value: Any, env_id: int, section: slice) -> torch.Tensor:
        # Validate before and after mapping; publish only float32 values.
        tensor = torch.as_tensor(value, device="cpu")
        expected = self._actions_cpu[env_id, section].shape
        if (
            tensor.shape != expected
            or tensor.is_complex()
            or not torch.isfinite(tensor).all()
        ):
            raise ValueError("invalid mapped action")
        action = tensor.to(dtype=torch.float32)
        if not torch.isfinite(action).all():
            raise ValueError("mapped action overflows float32")
        return torch.clamp(
            action,
            min=self._action_low[env_id, section],
            max=self._action_high[env_id, section],
        )

    def _map_command(
        self, cmd_vel: torch.Tensor, event: bool, env_id: int
    ) -> torch.Tensor:
        # The mapper receives detached input storage, never mutable cache state.
        return self._bounded_action(
            self._env.cfg.actions.map_cmd_to_action(cmd_vel.clone(), event),
            env_id,
            slice(None),
        )

    def _expire_actions_locked(self, now: float, *, remap: bool) -> None:
        expired = ~self._fresh(self._action_stamps, now) & ~self._mapped_action_mask
        self._actions_cpu[expired] = 0.0
        self._action_stamps[expired] = math.nan
        self._action_ids[expired] = 0
        expired_cmd = torch.isfinite(self._cmd_vel_stamps) & ~self._fresh(
            self._cmd_vel_stamps, now
        )
        expired_event = torch.isfinite(self._event_stamps) & ~self._fresh(
            self._event_stamps, now
        )
        self._actions_cmd_vel[expired_cmd] = 0.0
        self._cmd_vel_stamps[expired_cmd] = math.nan
        self._cmd_vel_ids[expired_cmd] = 0
        self._actions_event[expired_event] = 0.0
        self._event_stamps[expired_event] = math.nan
        self._event_ids[expired_event] = 0
        for env_id in (
            (expired_cmd | expired_event).nonzero(as_tuple=False).flatten().tolist()
        ):
            mask = self._mapped_action_mask[env_id].clone()
            if not mask.any():
                continue
            generation = self._reset_generations[env_id]
            stamp = max(
                (
                    stamp
                    for stamp in (
                        float(self._cmd_vel_stamps[env_id]),
                        float(self._event_stamps[env_id]),
                    )
                    if math.isfinite(stamp)
                ),
                default=math.nan,
            )
            action = None
            if remap and math.isfinite(stamp):
                try:
                    action = self._map_command(
                        self._actions_cmd_vel[env_id],
                        bool(self._actions_event[env_id]),
                        env_id,
                    )
                except (
                    ArithmeticError,
                    AttributeError,
                    TypeError,
                    ValueError,
                    RuntimeError,
                ):
                    self._warn_rejected(
                        "mapping", "Rejected invalid ROS command mapping"
                    )
            if self._close_requested or generation != self._reset_generations[env_id]:
                continue
            if action is None:
                self._actions_cpu[env_id, mask] = 0.0
                self._action_stamps[env_id, mask] = math.nan
                self._action_ids[env_id, mask] = 0
                self._mapped_action_mask[env_id, mask] = False
            else:
                self._actions_cpu[env_id, mask] = action[mask]
                self._action_stamps[env_id, mask] = stamp
                self._action_ids[env_id, mask] = max(
                    int(self._cmd_vel_ids[env_id]), int(self._event_ids[env_id])
                )

    def _apply_pending_action_updates_locked(self, now: float) -> None:
        if not self._pending_action_updates:
            return

        pending = sorted(
            self._pending_action_updates.items(), key=lambda item: item[1][0]
        )
        self._pending_action_updates.clear()
        generations = self._reset_generations.copy()
        for key, (seq, stamp, payload) in pending:
            kind, env_id = key[:2]
            if (
                self._close_requested
                or generations[env_id] != self._reset_generations[env_id]
                or stamp > now
                or (
                    self._command_timeout_s is not None
                    and now - stamp > self._command_timeout_s
                )
            ):
                continue
            section = slice(key[2], key[3]) if kind == "term" else slice(None)
            try:
                if kind in ("raw", "term"):
                    action = self._bounded_action(payload, env_id, section)
                elif kind in ("cmd_vel", "event"):
                    command = (
                        payload if kind == "cmd_vel" else self._actions_cmd_vel[env_id]
                    )
                    event = (
                        payload
                        if kind == "event"
                        else bool(self._actions_event[env_id])
                    )
                    action = self._map_command(command, event, env_id)
                else:
                    raise ValueError("unknown action channel")
            except (
                ArithmeticError,
                AttributeError,
                TypeError,
                ValueError,
                RuntimeError,
            ):
                self._warn_rejected("mapping", "Rejected invalid ROS command mapping")
                continue
            if (
                self._close_requested
                or generations[env_id] != self._reset_generations[env_id]
            ):
                continue
            if kind == "cmd_vel":
                self._actions_cmd_vel[env_id] = payload
                self._cmd_vel_stamps[env_id] = stamp
                self._cmd_vel_ids[env_id] = seq
            elif kind == "event":
                self._actions_event[env_id] = float(payload)
                self._event_stamps[env_id] = stamp
                self._event_ids[env_id] = seq
            if kind in ("cmd_vel", "event"):
                # Raw/term slices keep their receipt and ownership until expiry/reset;
                # mapped traffic updates only mapped or unclaimed cells.
                section = self._mapped_action_mask[env_id] | ~torch.isfinite(
                    self._action_stamps[env_id]
                )
                action = action[section]
            self._actions_cpu[env_id, section] = action
            self._action_stamps[env_id, section] = stamp
            self._action_ids[env_id, section] = seq
            self._mapped_action_mask[env_id, section] = kind in ("cmd_vel", "event")

    def _create_action_subscription(
        self,
        msg_type: type[Any],
        topic: str,
        callback: Callable[[Any], Any],
    ) -> Any:
        with self._lifecycle_lock:
            if self._close_requested:
                raise RuntimeError("ROS action IO is closing")
            subscription = self._action_pump.node.create_subscription(
                msg_type=msg_type,
                topic=topic,
                callback=callback,
                qos_profile=self._qos_profile,
            )
            self._owned_endpoints.append((subscription, "subscription", False))
            self._action_subscription_count += 1
            return subscription

    @staticmethod
    def _map_action_term(action_term: Any) -> tuple[type[Any], ActionExtractor]:
        Twist, BoolMsg, Float64MultiArray = _ros_action_messages()
        if _action_term_matches(action_term, "DifferentialInverseKinematicsAction"):
            if action_term.action_dim == 3:
                return Twist, lambda msg: [msg.linear.x, msg.linear.y, msg.linear.z]
            return Twist, lambda msg: [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ]

        if _action_term_matches(action_term, "BodyAccelerationAction"):
            return Twist, lambda msg: [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ]

        if _action_term_matches(
            action_term,
            "NonHolonomicAction",
            "WheeledDriveAction",
        ):
            return Twist, lambda msg: [msg.linear.x, msg.angular.z]

        if _action_term_matches(action_term, "MulticopterBodyAccelerationAction"):
            return Twist, lambda msg: [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.z,
            ]

        if _action_term_matches(
            action_term,
            "BinaryJointPositionAction",
            "BinaryJointVelocityAction",
        ):
            return BoolMsg, lambda msg: [-1.0 if msg.data else 1.0]

        return Float64MultiArray, lambda msg: list(msg.data)

    def _create_cb_action_any(
        self,
        key: str,
        extractor: ActionExtractor,
        dim: int,
        offset: int,
        env_id: int | None = None,
    ) -> Callable[[Any], None]:
        start_idx = offset
        end_idx = offset + dim

        def _proto_cb(msg: Any) -> None:
            with self._action_condition:
                if self._close_requested:
                    return
                try:
                    data = tuple(extractor(msg))
                except (AttributeError, ArithmeticError, TypeError, ValueError):
                    self._warn_rejected(
                        "payload", "Rejected malformed ROS action payload"
                    )
                    return
                if len(data) != dim:
                    self._warn_rejected(
                        "dimension",
                        f"Received action term '{key}' with incorrect dimension "
                        f"{len(data)} (expected {dim})",
                    )
                    return
                self._queue_action_update(
                    ("term", env_id, start_idx, end_idx),
                    data,
                )

        return _proto_cb

    def _create_cb_action_cmd_vel(
        self, env_id: int | None = None
    ) -> Callable[[Any], None]:
        def _proto_cb(msg: Any) -> None:
            with self._action_condition:
                if self._close_requested:
                    return
                try:
                    data = (
                        msg.linear.x,
                        msg.linear.y,
                        msg.linear.z,
                        msg.angular.x,
                        msg.angular.y,
                        msg.angular.z,
                    )
                except AttributeError:
                    self._warn_rejected(
                        "payload", "Rejected malformed ROS action payload"
                    )
                    return
                self._queue_action_update(("cmd_vel", env_id), data)

        return _proto_cb

    def _create_cb_action_event(
        self, env_id: int | None = None
    ) -> Callable[[Any], None]:
        def _proto_cb(msg: Any) -> None:
            with self._action_condition:
                if self._close_requested:
                    return
                self._queue_action_update(("event", env_id), getattr(msg, "data", None))

        return _proto_cb

    def _create_cb_action_raw(
        self, action_dim: int, env_id: int | None = None
    ) -> Callable[[Any], None]:
        def _proto_cb(msg: Any) -> None:
            with self._action_condition:
                if self._close_requested:
                    return
                try:
                    data = tuple(msg.data)
                except (AttributeError, TypeError, ValueError):
                    self._warn_rejected(
                        "payload", "Rejected malformed ROS action payload"
                    )
                    return
                if len(data) == action_dim:
                    self._queue_action_update(("raw", env_id), data)
                else:
                    self._warn_rejected(
                        "dimension",
                        "Received raw action with incorrect dimension "
                        f"{len(data)} (expected {action_dim})",
                    )

        return _proto_cb


__all__ = [
    "DEFAULT_COMMAND_TIMEOUT_S",
    "RosActionIO",
    "default_action_qos_profile",
    "validate_command_timeout",
]
