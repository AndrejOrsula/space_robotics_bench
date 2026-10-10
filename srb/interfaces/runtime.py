from __future__ import annotations

import threading
from collections.abc import Callable, Iterable
from typing import Any, TypeVar

from gymnasium import Wrapper

from srb.interfaces._ros_node import (
    RosCleanupError,
    RosCleanupScope,
    attempt_ros_cleanup,
)

InterfaceT = TypeVar("InterfaceT")
InterfaceFactory = Callable[..., Any]

_ENV_INTERFACES_ATTR = "_srb_interfaces"


class EnvInterfaceWrapper(Wrapper[Any, Any, Any, Any]):
    """Own environment interfaces and runtime, retaining failed cleanup for retry."""

    def __init__(
        self,
        env: Any,
        *,
        interface_factories: Iterable[InterfaceFactory],
        node: Any,
        ros_runtime: Any,
    ) -> None:
        self._lifecycle_lock = threading.RLock()
        self._close_requested = False
        self._closed = False
        self._cleanup_in_progress = False
        self._step_in_progress = False
        self._base_initialized = False
        self._environment_closed = False
        self._runtime_closed = False
        self._interfaces_detached = False
        self._owned_interfaces: list[Any] = []
        self._closed_interface_indices: set[int] = set()
        self._node = node
        self._ros_runtime = ros_runtime

        def construct_interface(factory: InterfaceFactory) -> Any:
            try:
                return factory(env=env, node=node)
            except RosCleanupError as error:
                self._adopt_cleanup_owner(error)
                raise

        with RosCleanupScope(
            self,
            "environment interface construction rollback",
        ):
            super().__init__(env)
            self._base_initialized = True
            set_env_interfaces(self, ())

            for factory in interface_factories:
                interface = construct_interface(factory)
                self._owned_interfaces.append(interface)
                set_env_interfaces(self, tuple(self._owned_interfaces))

    @staticmethod
    def _closable(owner: Any) -> bool:
        return callable(getattr(owner, "close", None))

    def _adopt_cleanup_owner(self, error: BaseException) -> None:
        owner = getattr(error, "cleanup_owner", None)
        if (
            owner is None
            or owner is self
            or owner is self._ros_runtime
            or owner is getattr(self, "env", None)
            or not self._closable(owner)
        ):
            return
        if any(interface is owner for interface in self._owned_interfaces):
            return
        self._owned_interfaces.append(owner)
        set_env_interfaces(self, tuple(self._owned_interfaces))

    def _adopt_attached_interfaces(self, interfaces: tuple[Any, ...]) -> None:
        """Adopt interfaces added through the shared runtime storage seam."""

        lifecycle_lock = self._lifecycle_lock
        if not lifecycle_lock.acquire(blocking=False):
            raise RuntimeError("environment interface operation already in progress")
        try:
            known = {id(interface) for interface in self._owned_interfaces}
            new_interfaces: list[Any] = []
            for interface in interfaces:
                if id(interface) in known:
                    continue
                known.add(id(interface))
                new_interfaces.append(interface)
            if interfaces and (self._close_requested or self._closed):
                raise RuntimeError(
                    "cannot attach an interface to a closed environment wrapper"
                )
            if new_interfaces and self._step_in_progress:
                raise RuntimeError(
                    "cannot attach an interface during an environment step"
                )
            self._owned_interfaces.extend(new_interfaces)
            setattr(self, _ENV_INTERFACES_ATTR, interfaces)
        finally:
            lifecycle_lock.release()

    def _detach_attached_interfaces(self) -> None:
        """Detach all published interfaces as one lifecycle-guarded change."""

        lifecycle_lock = self._lifecycle_lock
        if not lifecycle_lock.acquire(blocking=False):
            raise RuntimeError("environment interface operation already in progress")
        try:
            setattr(self, _ENV_INTERFACES_ATTR, ())
        finally:
            lifecycle_lock.release()

    def _append_attached_interface(self, interface: Any) -> None:
        """Append one interface while reading and publishing under one lock."""

        lifecycle_lock = self._lifecycle_lock
        if not lifecycle_lock.acquire(blocking=False):
            raise RuntimeError("environment interface operation already in progress")
        try:
            self._adopt_attached_interfaces((*iter_env_interfaces(self), interface))
        finally:
            lifecycle_lock.release()

    def _remove_attached_interface(self, interface: Any) -> tuple[Any, ...]:
        """Remove one interface while reading and publishing under one lock."""

        lifecycle_lock = self._lifecycle_lock
        if not lifecycle_lock.acquire(blocking=False):
            raise RuntimeError("environment interface operation already in progress")
        try:
            remaining = tuple(
                attached
                for attached in iter_env_interfaces(self)
                if attached is not interface
            )
            self._adopt_attached_interfaces(remaining)
            return remaining
        finally:
            lifecycle_lock.release()

    def _require_open(self) -> None:
        if self._close_requested or self._closed:
            raise RuntimeError("environment interface wrapper is closed")

    def _close_interfaces(self) -> None:
        errors: list[BaseException] = []
        for index, interface in enumerate(self._owned_interfaces):
            if index in self._closed_interface_indices:
                continue
            close = getattr(interface, "close", None)
            if not callable(close):
                self._closed_interface_indices.add(index)
                continue
            cleanup_error = attempt_ros_cleanup(close)
            if cleanup_error is None:
                self._closed_interface_indices.add(index)
            else:
                errors.append(cleanup_error)

        if errors:
            raise RosCleanupError(
                "environment interface cleanup incomplete",
                errors=tuple(errors),
                cleanup_owner=self,
            )
        if not self._interfaces_detached:
            self._detach_attached_interfaces()
            self._interfaces_detached = True

    def close(self) -> None:
        """Close interfaces, environment, and runtime with retryable phases."""

        lifecycle_lock = self._lifecycle_lock
        if not lifecycle_lock.acquire(blocking=False):
            raise RuntimeError("environment interface cleanup already in progress")
        if self._cleanup_in_progress or self._step_in_progress:
            lifecycle_lock.release()
            raise RuntimeError("environment interface cleanup already in progress")
        if self._closed:
            lifecycle_lock.release()
            return
        self._cleanup_in_progress = True
        self._close_requested = True
        lifecycle_lock.release()
        try:
            self._close_interfaces()

            if not self._environment_closed:
                if not self._base_initialized:
                    self._environment_closed = True
                else:
                    cleanup_error = attempt_ros_cleanup(super().close)
                    if cleanup_error is not None:
                        raise RosCleanupError(
                            "environment cleanup incomplete",
                            errors=(cleanup_error,),
                            cleanup_owner=self,
                        ) from cleanup_error
                    self._environment_closed = True

            if not self._runtime_closed:
                if self._ros_runtime is None:
                    self._runtime_closed = True
                else:
                    cleanup_error = attempt_ros_cleanup(
                        lambda: self._ros_runtime.close()
                    )
                    if cleanup_error is not None:
                        raise RosCleanupError(
                            "environment ROS runtime cleanup incomplete",
                            errors=(cleanup_error,),
                            cleanup_owner=self,
                        ) from cleanup_error
                    self._runtime_closed = True

            self._closed = True
        finally:
            lifecycle_lock.acquire()
            self._cleanup_in_progress = False
            lifecycle_lock.release()

    def __del__(self) -> None:
        attempt_ros_cleanup(self.close)

    def __enter__(self) -> EnvInterfaceWrapper:
        self._require_open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: Any,
    ) -> bool:
        cleanup_scope = RosCleanupScope(
            self,
            "environment interface cleanup incomplete",
            always=True,
        )
        return cleanup_scope.__exit__(exc_type, exc_value, traceback)

    def step(self, action: Any) -> tuple[Any, ...]:
        with self._lifecycle_lock:
            self._require_open()
            self._step_in_progress = True
            try:
                step_return = super().step(action)
                self._require_open()
                for interface in iter_env_interfaces(self):
                    self._require_open()
                    interface.update(*step_return, action=action)
                return step_return
            finally:
                self._step_in_progress = False


def iter_env_interfaces(env: Any) -> tuple[Any, ...]:
    """Return SRB interfaces attached to an environment wrapper."""

    interfaces = getattr(env, _ENV_INTERFACES_ATTR, ())
    if interfaces is None:
        return ()
    if isinstance(interfaces, tuple):
        return interfaces
    if isinstance(interfaces, Iterable):
        return tuple(interfaces)
    raise TypeError(
        f"Expected env.{_ENV_INTERFACES_ATTR} to be an iterable of interfaces, "
        f"got {type(interfaces).__name__}"
    )


def has_env_interfaces(env: Any) -> bool:
    """Return whether an environment wrapper owns SRB interface storage."""

    return hasattr(env, _ENV_INTERFACES_ATTR)


def set_env_interfaces(env: Any, interfaces: Iterable[Any]) -> tuple[Any, ...]:
    """Attach SRB interfaces to an environment wrapper and return them."""

    interfaces_tuple = tuple(interfaces)
    if isinstance(env, EnvInterfaceWrapper):
        env._adopt_attached_interfaces(interfaces_tuple)
        return interfaces_tuple
    setattr(env, _ENV_INTERFACES_ATTR, interfaces_tuple)
    return interfaces_tuple


def append_env_interface(env: Any, interface: Any) -> Any:
    """Append one SRB interface to an environment wrapper."""

    if isinstance(env, EnvInterfaceWrapper):
        env._append_attached_interface(interface)
        return interface
    set_env_interfaces(env, (*iter_env_interfaces(env), interface))
    return interface


def remove_env_interface(env: Any, interface: Any) -> tuple[Any, ...]:
    """Detach one interface while retaining every other attachment."""

    if isinstance(env, EnvInterfaceWrapper):
        return env._remove_attached_interface(interface)

    remaining = tuple(
        attached for attached in iter_env_interfaces(env) if attached is not interface
    )
    if remaining:
        set_env_interfaces(env, remaining)
    else:
        try:
            delattr(env, _ENV_INTERFACES_ATTR)
        except AttributeError:
            pass
    return remaining


def find_env_interface(env: Any, interface_type: type[InterfaceT]) -> InterfaceT | None:
    """Return the first attached interface matching ``interface_type``."""

    for interface in iter_env_interfaces(env):
        if isinstance(interface, interface_type):
            return interface
    return None


def find_env_interface_node(env: Any) -> Any | None:
    """Return the shared ROS-like node used by an attached interface, if present."""

    for interface in iter_env_interfaces(env):
        node = getattr(interface, "_node", None)
        if node is not None:
            return node
    return None
