"""Construct, update, and close the ROS runtime's child modules."""

from __future__ import annotations

import threading
from collections.abc import Callable
from contextlib import AbstractContextManager
from typing import Any

import torch

from srb.interfaces._ros_node import (
    RosCleanupError,
    RosCleanupScope,
    attempt_ros_cleanup,
)
from srb.interfaces.ros_action_io import (
    DEFAULT_COMMAND_TIMEOUT_S,
    RosActionIO,
    validate_command_timeout,
)
from srb.interfaces.ros_namespace import RosNamespace
from srb.interfaces.ros_reset_command import RosResetCommand
from srb.interfaces.ros_sensor_publishing import RosSensorPublisher
from srb.interfaces.ros_step_output import RosStepOutput
from srb.interfaces.ros_transform_publishing import RosTransformPublisher


class RosRuntimeModules:
    _CHILD_ATTRIBUTES = (
        "_action_io",
        "_reset_command",
        "_step_output",
        "_transform_publisher",
        "_sensor_publisher",
    )

    def __init__(
        self,
        *,
        env: Any,
        node: Any,
        namespace: RosNamespace,
        qos_profile: Any,
        qos_profile_tf: Any,
        command_timeout_s: float | None = DEFAULT_COMMAND_TIMEOUT_S,
    ) -> None:
        command_timeout_s = validate_command_timeout(command_timeout_s)
        self._lifecycle_lock = threading.RLock()
        self._close_requested = False
        self._close_in_progress = False
        self._closed = False
        self._closed_children: set[str] = set()
        # Initialize slots first so rollback can close partially built children.
        self._action_io: Any | None = None
        self._reset_command: Any | None = None
        self._step_output: Any | None = None
        self._transform_publisher: Any | None = None
        self._sensor_publisher: Any | None = None

        with RosCleanupScope(
            self,
            "ROS runtime module construction rollback incomplete",
        ):
            self._construct_child(
                "_action_io",
                lambda: RosActionIO(
                    env=env,
                    node=node,
                    namespace=namespace,
                    command_timeout_s=command_timeout_s,
                ),
            )
            action_io = self._action_io
            if action_io is None:
                raise RuntimeError("ROS action module was not constructed")

            self._construct_child(
                "_step_output",
                lambda: RosStepOutput(
                    env=env,
                    node=node,
                    namespace=namespace,
                    qos_profile=qos_profile,
                    clear_actions=action_io.clear_actions,
                    action_subscriptions_configured=(
                        lambda: action_io.subscriptions_configured
                    ),
                ),
            )
            self._construct_child(
                "_reset_command",
                lambda: RosResetCommand(
                    env=env,
                    node=node,
                    namespace=namespace,
                    clear_actions=action_io.clear_actions,
                ),
            )
            self._construct_child(
                "_transform_publisher",
                lambda: RosTransformPublisher(
                    env=env,
                    node=node,
                    namespace=namespace,
                    qos_profile=qos_profile_tf,
                ),
            )
            self._construct_child(
                "_sensor_publisher",
                lambda: RosSensorPublisher(
                    env=env,
                    node=node,
                    namespace=namespace,
                    qos_profile=qos_profile,
                ),
            )

    def _construct_child(self, attribute: str, factory: Callable[[], Any]) -> None:
        try:
            child = factory()
        except RosCleanupError as error:
            owner = error.cleanup_owner
            if (
                owner is not None
                and owner is not self
                and callable(getattr(owner, "close", None))
            ):
                setattr(self, attribute, owner)
            raise
        setattr(self, attribute, child)

    def _require_open(self) -> None:
        if self._close_requested:
            raise RuntimeError("ROS runtime modules are closed")

    def _require_action_io(self) -> Any:
        action_io = self._action_io
        if action_io is None:
            raise RuntimeError("ROS action module is unavailable")
        return action_io

    def close(self) -> None:
        with self._lifecycle_lock:
            if self._closed:
                return
            if self._close_in_progress:
                raise RosCleanupError(
                    "ROS runtime module cleanup is already in progress; retry close",
                    cleanup_owner=self,
                )
            self._close_requested = True
            self._close_in_progress = True
        try:
            errors: list[BaseException] = []
            # Stop ingress before other children; each retains failed cleanup.
            for attribute in self._CHILD_ATTRIBUTES:
                if attribute in self._closed_children:
                    continue
                module = getattr(self, attribute)
                if module is None:
                    self._closed_children.add(attribute)
                    continue
                close = getattr(module, "close", None)
                if not callable(close):
                    errors.append(
                        RosCleanupError(
                            f"ROS runtime child {attribute} has no public close",
                            cleanup_owner=module,
                        )
                    )
                    if attribute == "_action_io":
                        break
                    continue
                cleanup_error = attempt_ros_cleanup(close)
                if cleanup_error is None:
                    self._closed_children.add(attribute)
                else:
                    errors.append(cleanup_error)
                    if attribute == "_action_io":
                        # env.step may hold the ingress guard; retain dependent children.
                        break
            if errors:
                raise RosCleanupError(
                    "ROS runtime module cleanup incomplete",
                    errors=tuple(errors),
                    cleanup_owner=self,
                )
            self._closed = True
        finally:
            with self._lifecycle_lock:
                self._close_in_progress = False

    @property
    def action(self) -> torch.Tensor:
        self._require_open()
        return self._require_action_io().action

    @property
    def action_sequence(self) -> int:
        self._require_open()
        return self._require_action_io().action_sequence

    def action_with_sequence(self, *, block: bool = False) -> tuple[torch.Tensor, int]:
        self._require_open()
        return self._require_action_io().action_with_sequence(block=block)

    def action_transaction(
        self, *, block: bool = False
    ) -> AbstractContextManager[tuple[torch.Tensor, int]]:
        self._require_open()
        return self._require_action_io().action_transaction(block=block)

    def wait_for_action(self, sequence: int, timeout: float | None = None) -> bool:
        self._require_open()
        return self._require_action_io().wait_for_action(sequence, timeout=timeout)

    def clear_actions(self, env_mask: torch.Tensor | None = None) -> None:
        self._require_open()
        self._require_action_io().clear_actions(env_mask)

    def update(
        self,
        *,
        reward: torch.Tensor,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        info: dict[str, Any],
        action: torch.Tensor | None = None,
    ) -> None:
        with self._lifecycle_lock:
            self._require_open()
            time_msg = self._step_output.publish(
                reward=reward,
                terminated=terminated,
                truncated=truncated,
                info=info,
                action=action,
            )
            self._transform_publisher.publish(time_msg)
            self._sensor_publisher.publish(time_msg)
            self._reset_command.execute_pending()

    def setup_action_sub(self) -> None:
        with self._lifecycle_lock:
            if self._close_requested:
                return
            self._require_action_io()
            self._action_io.setup_subscriptions()


__all__ = ["RosRuntimeModules"]
