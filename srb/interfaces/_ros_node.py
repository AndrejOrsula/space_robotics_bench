"""Acquire, borrow, spin, and release ROS nodes with explicit ownership.

Provides background :class:`RosNodeRuntime` and synchronously pumped
:class:`RosPumpHandle` lifecycles. ROS imports stay lazy; process-global
``rclpy.shutdown()`` remains the top-level caller's responsibility.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from contextlib import AbstractContextManager
from dataclasses import dataclass, field
from types import TracebackType
from typing import TYPE_CHECKING, Any, Literal

from srb.utils.failures import attempt_cleanup

if TYPE_CHECKING:
    from rclpy.node import Node


class RosCleanupError(RuntimeError):
    """Cleanup failed; underlying errors and retryable ownership are retained."""

    def __init__(
        self,
        message: str,
        *,
        errors: tuple[BaseException, ...] = (),
        cleanup_owner: Any | None = None,
    ) -> None:
        self.errors = errors
        self.cleanup_owner = cleanup_owner
        detail = "; ".join(f"{type(error).__name__}: {error}" for error in errors)
        super().__init__(f"{message}: {detail}" if detail else message)


class RosExecutorCleanupError(RosCleanupError):
    """An owned executor or callback thread failed to stop."""


def attempt_ros_cleanup(operation: Callable[[], object]) -> BaseException | None:
    """Run one teardown operation and return its failure for the owner to retain."""
    try:
        operation()
    except BaseException as error:
        return error
    return None


class RosCleanupScope(AbstractContextManager["RosCleanupScope"]):
    """Preserve construction errors and make incomplete cleanup retryable.

    ``always=True`` cleans successful exits; callbacks retry through ``close()``.
    """

    def __init__(
        self,
        owner: Any,
        message: str,
        *,
        always: bool = False,
        cleanup: Callable[[], object] | None = None,
    ) -> None:
        self._message = message
        self._always = always
        self._cleanup = cleanup if cleanup is not None else lambda: owner.close()
        self._retry_owner = self if cleanup is not None else owner
        self._finished = False
        self._cleanup_lock = threading.Lock()

    def __enter__(self) -> RosCleanupScope:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> Literal[False]:
        if exc_value is not None or self._always:
            self._finish(exc_value)
        return False

    def close(self) -> None:
        """Retry the complete cleanup callback; successful cleanup is final."""
        self._finish(None)

    def _finish(self, initiating_error: BaseException | None) -> None:
        if not self._cleanup_lock.acquire(blocking=False):
            raise RosCleanupError(
                "ROS cleanup already in progress",
                errors=() if initiating_error is None else (initiating_error,),
                cleanup_owner=self._retry_owner,
            ) from initiating_error
        try:
            if self._finished:
                return
            cleanup_error = attempt_ros_cleanup(self._cleanup)
            if cleanup_error is not None:
                errors: tuple[BaseException, ...] = (cleanup_error,)
                if initiating_error is not None:
                    errors = (initiating_error, cleanup_error)
                raise RosCleanupError(
                    self._message,
                    errors=errors,
                    cleanup_owner=self._retry_owner,
                ) from (
                    initiating_error if initiating_error is not None else cleanup_error
                )
            self._finished = True
        finally:
            self._cleanup_lock.release()


class RosNodeCleanupError(RosCleanupError):
    """An owned ROS node could not be destroyed and remains retryable."""


class RosEndpointCleanupError(RosCleanupError):
    """One or more ROS publishers, subscriptions, or services remain owned."""


def _executor_shutdown(executor: Any, timeout: float | None) -> Any:
    """Shut down using Jazzy's bounded ``timeout_sec`` API."""
    return executor.shutdown(timeout_sec=timeout)


def _executor_work_complete(executor: Any, timeout: float | None) -> bool:
    """Wait for active callbacks; Jazzy shutdown may skip this tracker wait."""

    tracker = getattr(executor, "_work_tracker", None)
    wait = getattr(tracker, "wait", None)
    if not callable(wait):
        return True
    result = wait(timeout)
    return result is not False


def _cancel_executor_tasks(executor: Any) -> None:
    """Cancel queued rclpy tasks after the executor stops accepting work."""

    for task in tuple(getattr(executor, "_futures", ())):
        if getattr(task, "done", lambda: True)():
            continue
        if getattr(task, "executing", lambda: False)():
            continue
        cancel = getattr(task, "cancel", None)
        if callable(cancel):
            cancel()


def _executor_workers_complete(executor: Any, timeout: float | None) -> bool:
    """Stop and join workers before node destruction; Jazzy leaves its pool alive."""

    pool = getattr(executor, "_executor", None)
    if pool is None:
        return True

    shutdown = getattr(pool, "shutdown", None)
    if not callable(shutdown):
        return False

    # Keep shutdown non-blocking; task cancellation protects the node during drain.
    if not getattr(pool, "_shutdown", False):
        try:
            shutdown(wait=False, cancel_futures=True)
        except TypeError:
            shutdown(wait=False)
    _cancel_executor_tasks(executor)

    threads = tuple(getattr(pool, "_threads", ()))
    if not threads:
        return True

    deadline = None if timeout is None or timeout < 0.0 else time.monotonic() + timeout
    current = threading.current_thread()
    for thread in threads:
        if thread is current:
            return False
        remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
        thread.join(timeout=remaining)
        if thread.is_alive():
            return False
    return True


def _remaining_timeout(timeout: float | None, started: float) -> float | None:
    if timeout is None or timeout < 0.0:
        return None
    return max(0.0, timeout - (time.monotonic() - started))


def destroy_ros_endpoint(node: Any, endpoint: Any, *, kind: str) -> None:
    """Destroy an endpoint through its node; unwrap TF broadcaster publishers."""

    if endpoint is None:
        return

    if kind == "broadcaster":
        for attribute in ("pub_tf", "publisher", "_publisher", "_pub"):
            publisher = getattr(endpoint, attribute, None)
            if publisher is None:
                continue
            destroy_publisher = getattr(node, "destroy_publisher", None)
            if callable(destroy_publisher):
                result = destroy_publisher(publisher)
                if result is False:
                    raise RosEndpointCleanupError(
                        "ROS node destroy_publisher returned false"
                    )
                return

    destroy_on_node = getattr(node, f"destroy_{kind}", None)
    if callable(destroy_on_node):
        result = destroy_on_node(endpoint)
        if result is False:
            raise RosEndpointCleanupError(f"ROS node destroy_{kind} returned false")
        return

    destroy = getattr(endpoint, "destroy", None)
    if callable(destroy):
        result = destroy()
        if result is False:
            raise RosEndpointCleanupError(f"ROS {kind} destroy returned false")
        return

    raise RosEndpointCleanupError(f"ROS {kind} has no supported destruction operation")


@dataclass
class RosNodeHandle:
    """Reference to an ``rclpy.node.Node`` with its ownership state."""

    node: "Node"
    owns: bool
    _released: bool = field(default=False, init=False, repr=False)

    @property
    def released(self) -> bool:
        return self._released

    def release(self) -> None:
        if not self.owns or self._released:
            return
        try:
            result = self.node.destroy_node()
            if result is False:
                raise RuntimeError("destroy_node returned false")
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
            raise RosNodeCleanupError(
                "owned ROS node destruction failed",
                errors=(exc,),
            ) from exc
        self._released = True


@dataclass
class RosNodeRuntime:
    """Owned-or-borrowed ROS node plus optional executor thread."""

    node_handle: RosNodeHandle
    executor: Any | None = None
    thread: threading.Thread | None = None
    _closed: bool = False
    _close_requested: bool = False
    _executor_shutdown: bool = False
    _executor_pool_shutdown: bool = False
    _thread_stopped: bool = False
    _spin_once_loop: bool = False
    _spin_stop: threading.Event | None = field(
        default=None,
        init=False,
        repr=False,
    )
    _close_lock: threading.RLock = field(
        default_factory=threading.RLock,
        init=False,
        repr=False,
    )
    # Keep failure recording independent while close joins the thread.
    _spin_failure: BaseException | None = field(
        default=None,
        init=False,
        repr=False,
    )
    _spin_failure_reported: bool = field(default=False, init=False, repr=False)
    _spin_failure_lock: threading.Lock = field(
        default_factory=threading.Lock,
        init=False,
        repr=False,
    )

    @property
    def node(self) -> "Node":
        return self.node_handle.node

    def _record_spin_failure(self, error: BaseException) -> None:
        with self._spin_failure_lock:
            if self._spin_failure is None:
                self._spin_failure = error

    def _spin_failure_error(self) -> RosExecutorCleanupError | None:
        with self._spin_failure_lock:
            failure = self._spin_failure
        if failure is None:
            return None
        error = RosExecutorCleanupError(
            "ROS executor spin thread failed; owned node callbacks are no "
            "longer serviced",
            errors=(failure,),
        )
        error.__cause__ = failure
        return error

    def raise_if_failed(self) -> None:
        """Raise a recorded spin failure, chained to its original error.

        Call before trusting callback-fed state; borrowed or unspun runtimes have none.
        """
        error = self._spin_failure_error()
        if error is not None:
            raise error

    @property
    def owns(self) -> bool:
        return self.node_handle.owns

    def start_spin(
        self,
        *,
        num_threads: int = 2,
        thread_name: str | None = None,
    ) -> None:
        """Spin an owned node in the background; borrowed nodes remain caller-spun.

        Repeated calls are idempotent.
        """
        with self._close_lock:
            if self._closed or self._close_requested or not self.owns:
                return
            if self.executor is not None:
                return

            from rclpy.executors import MultiThreadedExecutor

            self.executor = MultiThreadedExecutor(num_threads=num_threads)
            self.executor.add_node(self.node)
            self._spin_stop = threading.Event()
            self._spin_once_loop = callable(getattr(self.executor, "spin_once", None))
            self.thread = threading.Thread(
                target=self._spin_owned_executor,
                name=thread_name,
                daemon=True,
            )
            self.thread.start()

    def _spin_owned_executor(self) -> None:
        """Stop callback intake before executor teardown.

        Jazzy may destroy its guard before workers finish, so ``close`` drains them
        first.
        """

        executor = self.executor
        if executor is None:
            return
        spin_once = getattr(executor, "spin_once", None)
        if not callable(spin_once):
            # Lightweight test doubles and older wrappers may only expose the
            # public spin method. Their shutdown remains the stop mechanism.
            # Any error ends callback service, so every type is recorded;
            # ``attempt_cleanup`` is the shared capture-everything seam.
            _, exc = attempt_cleanup(executor.spin)
            if exc is not None and not self._close_requested:
                self._record_spin_failure(exc)
            return

        stop = self._spin_stop
        while stop is None or not stop.is_set():
            _, exc = attempt_cleanup(lambda: spin_once(timeout_sec=0.1))
            if exc is not None:
                # An error after close requested the stop is teardown noise.
                # Any other error, of any type, ends the loop and is recorded,
                # so callback loss is reported by ``raise_if_failed`` and
                # ``close`` rather than only reaching ``threading.excepthook``.
                if stop is None or not stop.is_set():
                    self._record_spin_failure(exc)
                return

    def close(self, *, join_timeout: float | None = 1.0) -> None:
        """Stop owned callbacks before releasing the owned node.

        Failed cleanup remains owned for retry. Borrowed nodes and process-global
        ``rclpy`` state are never released here.
        """
        with self._close_lock:
            if self._closed:
                return
            self._close_requested = True
            errors: list[BaseException] = []
            close_started = time.monotonic()

            if self.executor is None:
                self._executor_shutdown = True
                self._executor_pool_shutdown = True
            elif not self._spin_once_loop and not self._executor_shutdown:
                try:
                    result = _executor_shutdown(self.executor, join_timeout)
                    if result is False:
                        raise RuntimeError("executor shutdown returned false")
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(
                        RosExecutorCleanupError(
                            "ROS executor shutdown failed",
                            errors=(exc,),
                        )
                    )
                else:
                    self._executor_shutdown = True

            if self._spin_once_loop and not self._thread_stopped:
                if self._spin_stop is not None:
                    self._spin_stop.set()
                wake = getattr(self.executor, "wake", None)
                if callable(wake):
                    try:
                        wake()
                    except (
                        AttributeError,
                        OSError,
                        RuntimeError,
                        TypeError,
                        ValueError,
                    ):
                        # The join below is authoritative. An already-stopped
                        # or externally-invalid guard needs no second report;
                        # a live thread remains owned and prevents release.
                        pass

            if self.thread is None:
                self._thread_stopped = True
            elif not self.thread.is_alive():
                self._thread_stopped = True
            elif self.thread is threading.current_thread():
                errors.append(
                    RosExecutorCleanupError(
                        "ROS executor close called from its callback thread; "
                        "callback must return before node release"
                    )
                )
            elif not self._thread_stopped:
                self.thread.join(
                    timeout=_remaining_timeout(join_timeout, close_started)
                )
                if self.thread.is_alive():
                    errors.append(
                        RosExecutorCleanupError(
                            f"ROS executor thread {self.thread.name!r} was still "
                            f"running {join_timeout}s after shutdown"
                        )
                    )
                else:
                    self._thread_stopped = True

            worker_pool_complete = True
            if self._thread_stopped and self.executor is not None:
                remaining = _remaining_timeout(join_timeout, close_started)
                try:
                    worker_pool_complete = _executor_workers_complete(
                        self.executor,
                        remaining,
                    )
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(
                        RosExecutorCleanupError(
                            "ROS executor worker pool shutdown failed",
                            errors=(exc,),
                        )
                    )
                    worker_pool_complete = False
                else:
                    if worker_pool_complete:
                        self._executor_pool_shutdown = True
                    else:
                        errors.append(
                            RosExecutorCleanupError(
                                "ROS executor worker pool remained active"
                            )
                        )

            work_complete = True
            if worker_pool_complete and self.executor is not None:
                remaining = _remaining_timeout(join_timeout, close_started)
                try:
                    work_complete = _executor_work_complete(self.executor, remaining)
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(
                        RosExecutorCleanupError(
                            "ROS executor work tracker failed",
                            errors=(exc,),
                        )
                    )
                    work_complete = False
                if not work_complete:
                    errors.append(
                        RosExecutorCleanupError(
                            "ROS executor callbacks remained in flight"
                        )
                    )

            # Destroy the guard only after callbacks stop and workers quiesce.
            if (
                self._spin_once_loop
                and not self._executor_shutdown
                and self._thread_stopped
                and worker_pool_complete
                and work_complete
            ):
                try:
                    result = _executor_shutdown(
                        self.executor,
                        _remaining_timeout(join_timeout, close_started),
                    )
                    if result is False:
                        raise RuntimeError("executor shutdown returned false")
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(
                        RosExecutorCleanupError(
                            "ROS executor shutdown failed",
                            errors=(exc,),
                        )
                    )
                else:
                    self._executor_shutdown = True

            # Keep ownership if shutdown failed, even if the thread exited.
            if (
                not errors
                and self._executor_shutdown
                and self._executor_pool_shutdown
                and self._thread_stopped
            ):
                try:
                    self.node_handle.release()
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(exc)

            if not errors and (not self.node_handle.owns or self.node_handle.released):
                self._closed = True
                # Report a spontaneous spin-thread failure once after cleanup.
                spin_failure = self._spin_failure_error()
                if spin_failure is not None and not self._spin_failure_reported:
                    self._spin_failure_reported = True
                    raise spin_failure
                return

            if errors:
                if len(errors) == 1:
                    raise errors[0]
                raise RosCleanupError(
                    "ROS runtime cleanup incomplete",
                    errors=tuple(errors),
                )
            raise RosCleanupError("ROS runtime cleanup incomplete")


def acquire_ros_node(
    node: "Node | None",
    *,
    name: str = "sim",
    namespace: str = "srb",
) -> RosNodeHandle:
    """Borrow ``node`` or initialize ``rclpy`` and create an owned node."""
    if node is not None:
        return RosNodeHandle(node=node, owns=False)

    import rclpy
    from rclpy.node import Node as _Node

    if not rclpy.ok():
        rclpy.init()
    return RosNodeHandle(
        node=_Node(name, namespace=namespace, start_parameter_services=False),
        owns=True,
    )


def acquire_ros_node_runtime(
    node: "Node | None",
    *,
    name: str = "sim",
    namespace: str = "srb",
    spin: bool = False,
    executor_threads: int = 2,
    thread_name: str | None = None,
) -> RosNodeRuntime:
    """Return a ROS node runtime and optionally spin owned nodes."""
    runtime = RosNodeRuntime(acquire_ros_node(node, name=name, namespace=namespace))
    with RosCleanupScope(runtime, "ROS runtime construction cleanup incomplete"):
        if spin:
            runtime.start_spin(num_threads=executor_threads, thread_name=thread_name)
    return runtime


@dataclass
class RosPumpHandle:
    """Owned child node whose callbacks are synchronously pumped via ``spin_once``.

    ``lock`` lets callers serialize custom spin-budget loops; ``try_spin_once``
    provides a non-blocking one-shot pump.
    """

    node: "Node"
    executor: Any
    lock: threading.Lock
    _closed: bool = False
    _close_requested: bool = False
    _executor_shutdown: bool = False
    _node_released: bool = False

    def try_spin_once(self, *, timeout_sec: float | None = None) -> bool:
        """Non-blocking lock attempt; return whether the spin happened."""
        if not self.lock.acquire(blocking=False):
            return False
        try:
            if self._closed or self._close_requested:
                return False
            self.executor.spin_once(timeout_sec=timeout_sec)
            return True
        finally:
            self.lock.release()

    def release(self) -> None:
        # Report incomplete cleanup rather than deadlock or destroy under a callback.
        if not self.lock.acquire(blocking=False):
            raise RosExecutorCleanupError(
                "ROS pump close is waiting for an active callback"
            )
        try:
            if self._closed:
                return
            self._close_requested = True
            errors: list[BaseException] = []
            if not self._executor_shutdown:
                try:
                    result = _executor_shutdown(self.executor, 0.0)
                    if result is False:
                        raise RuntimeError("executor shutdown returned false")
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(
                        RosExecutorCleanupError(
                            "ROS pump executor shutdown failed",
                            errors=(exc,),
                        )
                    )
                else:
                    self._executor_shutdown = True
            if self._executor_shutdown and not self._node_released:
                try:
                    result = self.node.destroy_node()
                    if result is False:
                        raise RuntimeError("destroy_node returned false")
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(
                        RosNodeCleanupError(
                            "owned ROS pump node destruction failed",
                            errors=(exc,),
                        )
                    )
                else:
                    self._node_released = True
            if not errors and self._executor_shutdown and self._node_released:
                self._closed = True
                return
            if len(errors) == 1:
                raise errors[0]
            raise RosCleanupError(
                "ROS pump cleanup incomplete",
                errors=tuple(errors),
            )
        finally:
            self.lock.release()


def acquire_ros_pump(
    parent_node: "Node",
    *,
    suffix: str = "action",
) -> RosPumpHandle:
    """Create an owned child node in the parent's namespace for on-demand pumping."""
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node as _Node

    child = _Node(
        f"{parent_node.get_name()}_{suffix}",
        namespace=parent_node.get_namespace(),
        start_parameter_services=False,
    )
    owner = RosNodeRuntime(RosNodeHandle(node=child, owns=True))
    with RosCleanupScope(owner, "ROS pump construction cleanup incomplete"):
        executor = SingleThreadedExecutor()
        owner.executor = executor
        executor.add_node(child)
        return RosPumpHandle(node=child, executor=executor, lock=threading.Lock())


__all__ = [
    "RosCleanupError",
    "RosCleanupScope",
    "attempt_ros_cleanup",
    "RosExecutorCleanupError",
    "RosNodeCleanupError",
    "RosEndpointCleanupError",
    "RosNodeHandle",
    "RosNodeRuntime",
    "RosPumpHandle",
    "acquire_ros_node",
    "acquire_ros_node_runtime",
    "acquire_ros_pump",
    "destroy_ros_endpoint",
]
