"""Process and ROS-context ownership for one real-agent CLI invocation."""

from __future__ import annotations

import signal
import sys
import threading
from collections.abc import Callable
from typing import Any

from srb.interfaces._ros_node import RosCleanupError
from srb.utils.failures import attempt_cleanup


class RealAgentSession:
    """Order real environment cleanup before owned ROS context shutdown."""

    def __init__(self, *, defer_activation: bool = False) -> None:
        self._context: Any | None = None
        self._owns_context = False
        self._env: Any | None = None
        self._command_source: Any | None = None
        self._command_source_abort: Callable[[], None] | None = None
        self._command_source_abort_done = False
        self._command_source_close_done = False
        self._defer_activation = defer_activation
        self._activated = False
        self._handlers: dict[int, Any] = {}
        self._python_handlers_owned = False
        self._handler_protection_started = False
        self._first_signal: int | None = None
        self._stopping = False
        self._cleaning = False
        self._entered = False
        self._closed = False
        self._context_shutdown_attempted = False

    @property
    def running(self) -> bool:
        """Whether the workflow may start or continue another loop iteration."""
        return not self._stopping

    def __enter__(self) -> RealAgentSession:
        self._require_main_thread()
        if self._entered:
            raise RuntimeError("real-agent session cannot be entered twice")
        self._entered = True
        if self._defer_activation:
            if self._deferred_handlers_are_safe():
                _, install_error = attempt_cleanup(self._install_python_handlers)
                if install_error is not None:
                    self._restore_python_handlers()
                    raise install_error
            return self
        _, enter_error = attempt_cleanup(self.activate)
        if enter_error is not None:
            cleanup_error = None
            if self._owns_context and self._context is not None and self._context.ok():
                _, cleanup_error = attempt_cleanup(
                    lambda: self.close(initiating_error=enter_error)
                )
            self._restore_python_handlers()
            if cleanup_error is not None:
                raise cleanup_error
            raise enter_error
        return self

    def activate(self) -> None:
        """Acquire ROS process resources after an entered deferred scope."""
        self._require_main_thread()
        self._require_entered()
        if self._activated:
            return
        self._validate_native_handlers()
        _, install_error = attempt_cleanup(self._install_python_handlers)
        if install_error is not None:
            if not self._python_handlers_owned:
                self._restore_python_handlers()
            raise install_error
        self._acquire_context()
        self._activated = True

    def adopt_cleanup_owner(self, owner: Any) -> None:
        """Adopt one aggregate owner for resources created in this lifetime."""
        self._require_main_thread()
        self._require_entered()
        self._set_cleanup_owner(
            owner,
            conflict_message="real-agent session adopted multiple cleanup owners",
        )

    def adopt_command_source(self, owner: Any, *, abort: Callable[[], None]) -> None:
        """Adopt the one ROS policy source beneath the environment owner."""
        self._require_main_thread()
        self._require_entered()
        if self._stopping or self._closed or self._cleaning:
            raise RuntimeError("real-agent session cannot adopt a late command source")
        if owner is None:
            raise TypeError("real-agent command source cannot be None")
        if owner is self or owner is self._env:
            raise ValueError("real-agent command source must be a separate owner")
        if not callable(getattr(owner, "close", None)):
            raise TypeError("real-agent command source must provide close()")
        if not callable(abort):
            raise TypeError("real-agent command source abort must be callable")
        if self._command_source is not None and self._command_source is not owner:
            raise RuntimeError("real-agent session adopted multiple command sources")
        if self._command_source is owner:
            raise RuntimeError("real-agent session adopted the command source twice")
        self._command_source = owner
        self._command_source_abort = abort

    def _set_cleanup_owner(
        self, owner: Any, *, conflict_message: str, replace: bool = False
    ) -> None:
        if owner is None:
            raise TypeError("real-agent cleanup owner cannot be None")
        if owner is self:
            raise ValueError("real-agent session cannot own itself")
        if not callable(getattr(owner, "close", None)):
            raise TypeError("real-agent cleanup owner must provide close()")
        if not replace and self._env is not None and self._env is not owner:
            raise RuntimeError(conflict_message)
        self._env = owner

    def construct_env(self, factory: Callable[[Callable[[Any], None]], Any]) -> Any:
        """Adopt the base env during construction, then retain its final wrapper."""
        self._require_entered()

        def adopt(owner: Any) -> None:
            self._set_cleanup_owner(
                owner,
                conflict_message="real-agent factory adopted multiple environments",
            )

        env, error = attempt_cleanup(lambda: factory(adopt))
        if error is not None:
            owner = getattr(error, "cleanup_owner", None)
            if owner is not None:
                self._set_cleanup_owner(
                    owner,
                    conflict_message="real-agent factory adopted multiple environments",
                    replace=True,
                )
            raise error
        self._set_cleanup_owner(
            env,
            conflict_message="real-agent factory adopted multiple environments",
            replace=True,
        )
        return env

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self._require_main_thread()
        del exc_type, traceback
        self._stopping = True
        _, cleanup_error = attempt_cleanup(
            lambda: self.close(initiating_error=exc_value)
        )
        self._restore_python_handlers()
        if cleanup_error is not None:
            raise cleanup_error
        if exc_value is None and self._first_signal is not None:
            self._raise_signal(self._first_signal)
        return False

    def close(self, *, initiating_error: BaseException | None = None) -> None:
        """Retry incomplete cleanup while preserving env-before-context order."""
        self._require_main_thread()
        if self._cleaning:
            errors = () if initiating_error is None else (initiating_error,)
            raise RosCleanupError(
                "real-agent cleanup is already in progress",
                errors=errors,
                cleanup_owner=self,
            ) from initiating_error
        self._stopping = True
        installed_for_retry = not self._python_handlers_owned
        if installed_for_retry:
            if self._activated:
                self._validate_native_handlers()
                may_install = True
            else:
                may_install = self._defer_activation or self._handler_protection_started
                if may_install:
                    may_install = self._deferred_handlers_are_safe()
            if may_install:
                _, install_error = attempt_cleanup(self._install_python_handlers)
                if install_error is not None:
                    self._restore_python_handlers()
                    raise install_error
        self._cleaning = True
        failures: list[BaseException] = []
        try:
            if self._command_source is not None:
                if not self._command_source_abort_done:
                    callback = self._command_source_abort
                    if callback is not None:
                        _, error = attempt_cleanup(callback)
                        if error is None:
                            self._command_source_abort_done = True
                        else:
                            failures.append(error)
                if not self._command_source_close_done:
                    _, error = attempt_cleanup(self._command_source.close)
                    if error is None:
                        self._command_source_close_done = True
                    else:
                        failures.append(error)

            source_released = (
                self._command_source is None or self._command_source_close_done
            )
            # Only an unclosed source can still deliver callbacks. RealEnv.close()
            # retries a failed abort through sink cleanup; keep that failure.
            if self._env is not None and source_released:
                env = self._env
                _, error = attempt_cleanup(env.close)
                if error is not None:
                    failures.append(error)
                else:
                    self._env = None
                    self._command_source_abort_done = True

            if (
                self._env is None
                and source_released
                and self._owns_context
                and not self._context_shutdown_attempted
            ):
                context = self._context
                if context is not None:
                    if not context.ok():
                        self._context_shutdown_attempted = True
                        self._reset_default_context_if_owned()
                    else:
                        _, error = attempt_cleanup(
                            lambda: self._shutdown_context(context)
                        )
                        if error is not None:
                            failures.append(error)
                            # Shutdown callbacks may fail after context invalidation.
                            if not context.ok():
                                self._context_shutdown_attempted = True
                                self._reset_default_context_if_owned()
                        else:
                            self._context_shutdown_attempted = True
                            self._reset_default_context_if_owned()
            if not failures:
                self._closed = True
        finally:
            self._cleaning = False
            if installed_for_retry:
                self._restore_python_handlers()

        if failures:
            errors = (
                [initiating_error] if initiating_error is not None else []
            ) + failures
            raise RosCleanupError(
                "real-agent process cleanup incomplete",
                errors=tuple(errors),
                cleanup_owner=self,
            ) from (initiating_error or failures[0])

    def _acquire_context(self) -> None:
        import rclpy
        from rclpy.signals import SignalHandlerOptions

        context = rclpy.get_default_context()
        if context.ok():
            self._context = context
            return

        self._context = context
        self._owns_context = True
        _, init_error = attempt_cleanup(
            lambda: rclpy.init(
                context=context,
                signal_handler_options=SignalHandlerOptions.NO,
            )
        )
        if init_error is not None:
            if not context.ok():
                self._reset_default_context_if_owned()
                self._context = None
                self._owns_context = False
            raise init_error

    @staticmethod
    def _validate_native_handlers() -> None:
        from rclpy.signals import (
            SignalHandlerOptions,
            get_current_signal_handlers_options,
        )

        if get_current_signal_handlers_options() != SignalHandlerOptions.NO:
            raise RuntimeError(
                "real-agent CLI requires rclpy signal handlers disabled "
                "(SignalHandlerOptions.NO)"
            )

    def _install_python_handlers(self) -> None:
        self._require_main_thread()
        for signum in (signal.SIGINT, signal.SIGTERM):
            if signum not in self._handlers:
                self._handlers[signum] = signal.getsignal(signum)
            current = signal.getsignal(signum)
            if (
                getattr(current, "__self__", None) is not self
                or getattr(current, "__func__", None)
                is not self._handle_signal.__func__
            ):
                signal.signal(signum, self._handle_signal)
        self._python_handlers_owned = True
        self._handler_protection_started = True

    def _restore_python_handlers(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            return
        for signum, handler in self._handlers.items():
            signal.signal(signum, handler)
        self._handlers.clear()
        self._python_handlers_owned = False

    @staticmethod
    def _deferred_handlers_are_safe() -> bool:
        if "rclpy.signals" not in sys.modules:
            return True

        def probe_loaded_module() -> bool:
            signals_module = sys.modules["rclpy.signals"]
            options = signals_module.get_current_signal_handlers_options()
            no_handlers = signals_module.SignalHandlerOptions.NO
            return bool(options == no_handlers)

        safe, error = attempt_cleanup(probe_loaded_module)
        if error is None:
            return safe is True
        if isinstance(error, Exception):
            return False
        raise error

    def _handle_signal(self, signum: int, _frame: Any) -> None:
        already_stopping = self._stopping
        first_delivery = self._first_signal is None
        if self._first_signal is None:
            self._first_signal = signum
        self._stopping = True
        if first_delivery and not already_stopping:
            self._raise_signal(signum)

    @staticmethod
    def _raise_signal(signum: int) -> None:
        if signum == signal.SIGINT:
            raise KeyboardInterrupt
        raise SystemExit(128 + signum)

    def _reset_default_context_if_owned(self) -> None:
        if self._context is None:
            return
        from rclpy import utilities

        if utilities.get_default_context() is self._context:
            utilities.get_default_context(shutting_down=True)

    @staticmethod
    def _shutdown_context(context: Any) -> None:
        import rclpy

        rclpy.shutdown(context=context, uninstall_handlers=False)

    def _require_entered(self) -> None:
        if not self._entered or self._closed or self._stopping:
            raise RuntimeError("real-agent session is not active")

    @staticmethod
    def _require_main_thread() -> None:
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("real-agent process ownership requires the main thread")


__all__ = ("RealAgentSession",)
