try:
    import rclpy  # noqa: F401
except ImportError:
    from srb.utils.ros import enable_ros2_bridge

    enable_ros2_bridge()

import threading
from contextlib import AbstractContextManager
from typing import TYPE_CHECKING, Any

import torch
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from srb.interfaces._ros_node import (
    RosCleanupError,
    RosCleanupScope,
    acquire_ros_node_runtime,
    attempt_ros_cleanup,
)
from srb.interfaces.ros_action_io import (
    DEFAULT_COMMAND_TIMEOUT_S,
    validate_command_timeout,
)
from srb.interfaces.ros_environment import require_ros_environment
from srb.interfaces.ros_namespace import RosNamespace
from srb.interfaces.ros_runtime_modules import RosRuntimeModules

from .base import InterfaceBase

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class RosInterface(InterfaceBase):
    __QOS_PROFILE = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    __QOS_PROFILE_TF = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(
        self,
        env: "AnyEnv",
        node: Node | None = None,
        *args,
        omit_single_env_ns: bool = True,
        command_timeout_s: float | None = DEFAULT_COMMAND_TIMEOUT_S,
        **kwargs,
    ):
        # Initialize state first so constructor failures can use close().
        self._lifecycle_lock = threading.RLock()
        self._close_requested = False
        self._close_in_progress = False
        self._closed = False
        self._modules_closed = False
        self._runtime_closed = False
        self._env: "AnyEnv" = None  # type: ignore[assignment]
        self._num_envs = 0
        self._namespace: RosNamespace | None = None
        self._node: Node | None = None
        self._ros_runtime: Any | None = None
        self._modules: Any | None = None
        command_timeout_s = validate_command_timeout(command_timeout_s)

        with RosCleanupScope(
            self,
            "ROS interface construction rollback incomplete",
        ):
            self._env = env.unwrapped  # type: ignore[misc]
            self._num_envs = self._env.num_envs

            # Validate the ROS graph before endpoint creation. Injected nodes
            # are configured by their caller.
            if node is None:
                require_ros_environment()

            ## Topic/frame namespace
            self._namespace = RosNamespace.for_env_count(
                self._num_envs,
                omit_single_env_ns=omit_single_env_ns,
            )

            ## Initialize node
            try:
                self._ros_runtime = acquire_ros_node_runtime(node, spin=True)
            except RosCleanupError as error:
                owner = error.cleanup_owner
                if owner is not None and self._closable(owner):
                    self._ros_runtime = owner
                raise
            self._node = self._ros_runtime.node
            try:
                self._modules = RosRuntimeModules(
                    env=self._env,
                    node=self._node,
                    namespace=self._namespace,
                    qos_profile=self.__QOS_PROFILE,
                    qos_profile_tf=self.__QOS_PROFILE_TF,
                    command_timeout_s=command_timeout_s,
                )
            except RosCleanupError as error:
                owner = error.cleanup_owner
                if owner is not None and self._closable(owner):
                    self._modules = owner
                raise

    @staticmethod
    def _closable(owner: Any) -> bool:
        return callable(getattr(owner, "close", None))

    def _require_open_modules(self) -> Any:
        if self._close_requested:
            raise RuntimeError("ROS interface is closed")
        modules = self._modules
        if modules is None:
            raise RuntimeError("ROS interface modules are unavailable")
        return modules

    def __del__(self):
        attempt_ros_cleanup(self.close)

    def close(self) -> None:
        with self._lifecycle_lock:
            if self._closed:
                return
            if self._close_in_progress:
                raise RosCleanupError(
                    "ROS interface cleanup is already in progress; retry close",
                    cleanup_owner=self,
                )
            self._close_requested = True
            self._close_in_progress = True
        try:
            if not self._modules_closed:
                if self._modules is None:
                    self._modules_closed = True
                else:
                    cleanup_error = attempt_ros_cleanup(self._modules.close)
                    if cleanup_error is not None:
                        raise RosCleanupError(
                            "ROS interface module cleanup incomplete",
                            errors=(cleanup_error,),
                            cleanup_owner=self,
                        ) from cleanup_error
                    self._modules_closed = True

            if not self._runtime_closed:
                if self._ros_runtime is None:
                    self._runtime_closed = True
                else:
                    cleanup_error = attempt_ros_cleanup(self._ros_runtime.close)
                    if cleanup_error is not None:
                        raise RosCleanupError(
                            "ROS interface runtime cleanup incomplete",
                            errors=(cleanup_error,),
                            cleanup_owner=self,
                        ) from cleanup_error
                    self._runtime_closed = True

            self._closed = True
        finally:
            with self._lifecycle_lock:
                self._close_in_progress = False

    @property
    def action(self) -> torch.Tensor:
        return self._require_open_modules().action

    @property
    def action_sequence(self) -> int:
        return self._require_open_modules().action_sequence

    def action_with_sequence(self, *, block: bool = False) -> tuple[torch.Tensor, int]:
        return self._require_open_modules().action_with_sequence(block=block)

    def action_transaction(
        self, *, block: bool = False
    ) -> AbstractContextManager[tuple[torch.Tensor, int]]:
        return self._require_open_modules().action_transaction(block=block)

    def wait_for_action(self, sequence: int, timeout: float | None = None) -> bool:
        return self._require_open_modules().wait_for_action(sequence, timeout=timeout)

    def clear_actions(self, env_mask: torch.Tensor | None = None) -> None:
        self._require_open_modules().clear_actions(env_mask)

    def update(
        self,
        observation: dict[str, torch.Tensor],
        reward: torch.Tensor,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        info: dict[str, Any],
        action: torch.Tensor | None = None,
        *args,
        **kwargs,
    ):
        modules = self._require_open_modules()
        # Surface executor failures to the agent loop.
        raise_if_failed = getattr(
            getattr(self, "_ros_runtime", None), "raise_if_failed", None
        )
        if callable(raise_if_failed):
            raise_if_failed()
        modules.update(
            reward=reward,
            terminated=terminated,
            truncated=truncated,
            info=info,
            action=action,
        )

    ## Action ##

    def setup_action_sub(self):
        if self._close_requested:
            return
        self._require_open_modules()
        self._modules.setup_action_sub()
