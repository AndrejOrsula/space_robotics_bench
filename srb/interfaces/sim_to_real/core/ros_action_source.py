"""Background ROS source for flat RealEnv policy actions."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from numbers import Real
from typing import Any

import numpy as np
from gymnasium.spaces import Box

from srb.interfaces._ros_node import (
    RosCleanupError,
    RosCleanupScope,
    acquire_ros_node_runtime,
    attempt_ros_cleanup,
    destroy_ros_endpoint,
)
from srb.interfaces.ros_action_io import (
    DEFAULT_COMMAND_TIMEOUT_S,
    default_action_qos_profile,
    validate_command_timeout,
)
from srb.interfaces.ros_namespace import RosNamespace
from srb.interfaces.sim_to_real.core.task_evaluator import OperatorAbort
from srb.utils.failures import attempt_cleanup

DEFAULT_STARTUP_TIMEOUT_S = 5.0


@dataclass(frozen=True, slots=True)
class RawActionSample:
    """One validated immutable action received by this source."""

    _source: object
    action: np.ndarray
    sequence: int
    received_monotonic: float
    deadline: float


class RawPolicyActionSource:
    """Own a background ROS subscription for flat RealEnv actions."""

    def __init__(
        self,
        *,
        env: Any,
        session: Any,
        single_action_space: Any,
        ros_action_timeout: float | None = None,
        startup_timeout: float = DEFAULT_STARTUP_TIMEOUT_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._clock = clock
        self._condition = threading.Condition()
        self._closing = False
        self._closed = False
        self._close_in_progress = False
        self._subscription: Any | None = None
        self._runtime: Any | None = None
        self._failure: BaseException | None = None
        self._latest: RawActionSample | None = None
        self._sequence = 0
        self._last_clock_s: float | None = None
        self._owner_token = object()
        self._timeout = validate_command_timeout(
            DEFAULT_COMMAND_TIMEOUT_S
            if ros_action_timeout is None
            else ros_action_timeout
        )
        if isinstance(startup_timeout, bool) or not isinstance(
            startup_timeout, (int, float)
        ):
            raise ValueError("startup_timeout must be a positive finite number")
        startup_timeout = float(startup_timeout)
        if not math.isfinite(startup_timeout) or startup_timeout <= 0:
            raise ValueError("startup_timeout must be a positive finite number")
        self._startup_timeout = (
            self._timeout if ros_action_timeout is not None else startup_timeout
        )

        space = single_action_space
        if not isinstance(space, Box) or len(space.shape) != 1 or not space.shape[0]:
            raise ValueError("RealEnv raw ROS ingress requires a flat Box action space")
        self._shape = tuple(space.shape)
        self._dtype = np.dtype(space.dtype)
        self._low = np.array(space.low, copy=True)
        self._high = np.array(space.high, copy=True)
        if (
            not np.all(np.isfinite(self._low) | np.isinf(self._low))
            or not np.all(np.isfinite(self._high) | np.isinf(self._high))
            or np.any(self._low > self._high)
        ):
            raise ValueError("RealEnv action bounds are invalid")

        # Register the cleanup owner before acquiring runtime or subscription.
        session.adopt_command_source(
            self,
            abort=lambda: env.abort_session("ROS raw action source stopped"),
        )
        with RosCleanupScope(
            self,
            "RealEnv raw ROS action source construction cleanup incomplete",
            cleanup=self.close,
        ):
            from std_msgs.msg import Float64MultiArray

            namespace = env.ros_node.get_namespace()
            self._runtime, error = attempt_cleanup(
                lambda: acquire_ros_node_runtime(
                    None,
                    name="real_raw_action",
                    namespace=namespace,
                    spin=True,
                )
            )
            if error is not None:
                partial_owner = getattr(error, "cleanup_owner", None)
                if callable(getattr(partial_owner, "close", None)):
                    self._runtime = partial_owner
                raise error
            # Relative topic names resolve under the acquired node's inherited
            # namespace. Keep the single-environment topic shape without
            # replacing the parent's namespace with the global root.
            namespace = RosNamespace.for_env_count(1)
            self._subscription = self._runtime.node.create_subscription(
                msg_type=Float64MultiArray,
                topic=namespace.env_topic(0, "action/raw"),
                callback=self._on_message,
                qos_profile=default_action_qos_profile(),
            )

    def _validated_action(self, values: Any) -> np.ndarray:
        try:
            raw = np.asarray(values)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("ROS action payload is not a numeric vector") from error
        if raw.shape != self._shape or raw.dtype.kind not in "biuf":
            raise ValueError(
                f"ROS action payload must have shape {self._shape} and real values"
            )
        if not np.all(np.isfinite(raw)):
            raise ValueError("ROS action payload contains non-finite values")
        if np.any(raw < self._low) or np.any(raw > self._high):
            raise ValueError("ROS action payload exceeds the declared action bounds")
        # Casting would silently truncate fractional or non-0/1 values.
        if self._dtype.kind == "b" and not np.all((raw == 0) | (raw == 1)):
            raise ValueError("ROS action payload is not exactly boolean (0 or 1)")
        if self._dtype.kind in "iu" and not np.all(raw == np.trunc(raw)):
            raise ValueError("ROS action payload is not exactly integral")
        with np.errstate(over="ignore", invalid="ignore"):
            cast = np.asarray(raw, dtype=self._dtype)
        if not np.all(np.isfinite(cast)):
            raise ValueError("ROS action payload is non-finite after dtype conversion")
        if np.any(cast < self._low) or np.any(cast > self._high):
            raise ValueError("ROS action payload exceeds bounds after dtype conversion")
        cast = np.frombuffer(
            np.ascontiguousarray(cast).tobytes(), dtype=self._dtype
        ).reshape(self._shape)
        return cast

    def _on_message(self, message: Any) -> None:
        with self._condition:
            if self._closing or self._closed:
                return
            received = self._read_clock_locked()
            if received is None:
                self._condition.notify_all()
                return
            try:
                action = self._validated_action(message.data)
            except (AttributeError, TypeError, ValueError, OverflowError) as error:
                self._failure = OperatorAbort(f"invalid ROS policy action: {error}")
                self._condition.notify_all()
                return
            assert self._timeout is not None
            deadline = received + self._timeout
            if not math.isfinite(deadline):
                self._latch_clock_failure(
                    ValueError("ROS action sample deadline is non-finite")
                )
                self._condition.notify_all()
                return
            self._sequence += 1
            self._latest = RawActionSample(
                self._owner_token,
                action,
                self._sequence,
                received,
                deadline,
            )
            self._condition.notify_all()

    def _latch_clock_failure(self, error: BaseException) -> None:
        failure = OperatorAbort(f"invalid ROS policy action clock: {error}")
        failure.__cause__ = error
        self._failure = failure

    def _read_clock_locked(self) -> float | None:
        """Read and validate monotonic time under the source lock."""

        def read() -> float:
            value = self._clock()
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError("clock must return a real number")
            now = float(value)
            if not math.isfinite(now) or (
                self._last_clock_s is not None and now < self._last_clock_s
            ):
                raise ValueError("clock is non-finite or moved backwards")
            return now

        now, error = attempt_cleanup(read)
        if error is not None:
            if not isinstance(error, Exception):
                raise error
            self._latch_clock_failure(error)
            return None
        self._last_clock_s = now
        return now

    def next_sample(
        self,
        *,
        after_sequence: int = 0,
        timeout: float | None = None,
        prior_deadline: float | None = None,
    ) -> RawActionSample:
        """Wait for a valid sample within an absolute monotonic time budget."""
        duration = self._startup_timeout if timeout is None else timeout
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("sample wait timeout must be positive and finite")
        with self._condition:
            if self._runtime is not None:
                self._runtime.raise_if_failed()
            started = self._read_clock_locked()
            if started is None:
                assert self._failure is not None
                raise self._failure
            local_deadline = started + duration
            if not math.isfinite(local_deadline):
                self._latch_clock_failure(
                    ValueError("ROS action wait deadline is non-finite")
                )
                assert self._failure is not None
                raise self._failure
            deadline = (
                min(local_deadline, prior_deadline)
                if prior_deadline is not None
                else local_deadline
            )
            latest_at_entry = self._latest
            newer_sample_at_entry = (
                latest_at_entry is not None
                and latest_at_entry.sequence > after_sequence
            )
            while True:
                if self._closed or self._closing:
                    raise OperatorAbort("ROS policy action source is closed")
                if self._failure is not None:
                    raise self._failure
                if self._runtime is not None:
                    self._runtime.raise_if_failed()
                now = self._read_clock_locked()
                if now is None:
                    assert self._failure is not None
                    raise self._failure
                sample = self._latest
                if sample is not None and sample.sequence > after_sequence:
                    if (
                        prior_deadline is not None
                        and sample.received_monotonic > prior_deadline
                    ):
                        raise OperatorAbort(
                            "previous ROS policy action expired before fresh input"
                        )
                    if (
                        not newer_sample_at_entry
                        and sample.received_monotonic > local_deadline
                    ):
                        raise OperatorAbort(
                            "timed out waiting for a fresh ROS policy action"
                        )
                    if now > sample.deadline:
                        raise OperatorAbort("ROS policy action sample expired")
                    return sample
                if prior_deadline is not None and now > prior_deadline:
                    raise OperatorAbort(
                        "previous ROS policy action expired before fresh input"
                    )
                if now > deadline:
                    raise OperatorAbort(
                        "timed out waiting for a fresh ROS policy action"
                    )
                self._condition.wait(timeout=min(0.1, max(0.001, deadline - now)))

    def latest_sample(self) -> RawActionSample:
        with self._condition:
            if self._closed or self._closing:
                raise OperatorAbort("ROS policy action source is closed")
            if self._failure is not None:
                raise self._failure
            if self._runtime is not None:
                self._runtime.raise_if_failed()
            sample = self._latest
            if sample is None:
                raise OperatorAbort("no valid ROS policy action has been received")
            now = self._read_clock_locked()
            if now is None:
                assert self._failure is not None
                raise self._failure
            if now > sample.deadline:
                raise OperatorAbort("ROS policy action sample expired")
            return sample

    def check_dispatch(self, sample: RawActionSample) -> None:
        """Reject foreign, invalidated or expired dispatch selections."""
        with self._condition:
            if sample._source is not self._owner_token:
                raise OperatorAbort(
                    "ROS policy action sample belongs to another source"
                )
            if self._closed or self._closing:
                raise OperatorAbort("ROS policy action source is closed")
            if self._failure is not None:
                raise self._failure
            now = self._read_clock_locked()
            if now is None:
                assert self._failure is not None
                raise self._failure
            if now > sample.deadline:
                raise OperatorAbort("ROS policy action sample expired before dispatch")

    def close(self) -> None:
        """Stop callbacks and retry failed endpoint or runtime cleanup."""
        with self._condition:
            if self._closed:
                return
            if self._close_in_progress:
                raise RuntimeError("ROS policy action source cleanup is in progress")
            self._closing = True
            self._close_in_progress = True
            self._condition.notify_all()
        errors: list[BaseException] = []
        try:
            if self._subscription is not None and self._runtime is not None:
                error = attempt_ros_cleanup(
                    lambda: destroy_ros_endpoint(
                        self._runtime.node, self._subscription, kind="subscription"
                    )
                )
                if error is None:
                    self._subscription = None
                else:
                    errors.append(error)
            if self._runtime is not None and self._subscription is None:
                error = attempt_ros_cleanup(self._runtime.close)
                if error is None:
                    self._runtime = None
                else:
                    errors.append(error)
            if errors:
                raise RosCleanupError(
                    "ROS policy action source cleanup incomplete",
                    errors=tuple(errors),
                    cleanup_owner=self,
                ) from errors[0]
            with self._condition:
                self._closed = True
        finally:
            with self._condition:
                self._close_in_progress = False


__all__ = ("RawActionSample", "RawPolicyActionSource")
