"""Queue ROS reset requests and execute them on the simulation thread."""

from __future__ import annotations

import threading
from collections.abc import Callable, Sequence
from typing import Any

import torch

from srb.interfaces._ros_node import (
    RosCleanupScope,
    RosEndpointCleanupError,
    destroy_ros_endpoint,
)
from srb.interfaces.ros_namespace import RosNamespace


def _empty_service_type() -> Any:
    from std_srvs.srv import Empty as EmptySrv

    return EmptySrv


def _simulation_manager() -> Any:
    from srb.core.manager import SimulationManager

    return SimulationManager


class RosResetCommand:
    def __init__(
        self,
        *,
        env: Any,
        node: Any,
        namespace: RosNamespace,
        clear_actions: Callable[[torch.Tensor | None], None],
    ) -> None:
        self._env = env
        self._node = node
        self._num_envs = int(env.num_envs)
        self._namespace = namespace
        self._clear_actions = clear_actions
        self._pending_lock = threading.Lock()
        self._lifecycle_lock = threading.RLock()
        self._reset_all_pending = False
        self._pending_env_ids: set[int] = set()
        self._close_requested = False
        self._closed = False
        self._owned_endpoints: list[tuple[Any, str, bool]] = []

        self._srv_reset_all = None
        self._srv_reset: tuple[Any, ...] = ()
        empty_srv = _empty_service_type()
        envs_reset_topic = self._namespace.envs_topic("reset")
        with RosCleanupScope(
            self,
            "ROS reset command construction cleanup incomplete",
        ):
            if envs_reset_topic is not None:
                self._srv_reset_all = self._create_service(
                    empty_srv,
                    envs_reset_topic,
                    self._create_reset_callback(),
                )
            self._srv_reset = tuple(
                self._create_service(
                    empty_srv,
                    self._namespace.env_topic(i, "reset"),
                    self._create_reset_callback(i),
                )
                for i in range(self._num_envs)
            )

    def execute_pending(self) -> None:
        if self._close_requested:
            return
        with self._pending_lock:
            reset_all = self._reset_all_pending
            env_ids = tuple(sorted(self._pending_env_ids))
            self._reset_all_pending = False
            self._pending_env_ids.clear()

        if self._close_requested:
            return
        if reset_all:
            self._reset_all()
            return

        if not env_ids:
            return

        self._reset_envs(env_ids)

    def _create_reset_callback(self, env_id: int | None = None) -> Callable:
        def _callback(request: Any, response: Any) -> Any:
            self._request_reset(env_id)
            return response

        return _callback

    def _request_reset(self, env_id: int | None = None) -> None:
        if self._close_requested:
            return
        with self._pending_lock:
            if self._close_requested:
                return
            if env_id is None:
                self._reset_all_pending = True
                self._pending_env_ids.clear()
                return

            if not self._reset_all_pending:
                self._pending_env_ids.add(env_id)

    def _reset_all(self) -> None:
        if self._close_requested:
            return
        self._clear_actions(None)
        self._env.reset()

    def _reset_envs(self, env_ids: Sequence[int]) -> None:
        if self._close_requested:
            return
        ids = torch.tensor(tuple(env_ids), dtype=torch.int, device=self._env.device)
        env_mask = torch.zeros(
            self._num_envs, dtype=torch.bool, device=self._env.device
        )
        env_mask[ids.long()] = True
        self._clear_actions(env_mask)
        self._env._reset_idx(ids)  # type: ignore[misc]
        self._env.scene.write_data_to_sim()
        self._env.sim.forward()
        self._render_after_partial_reset()
        lifecycle = getattr(self._env, "_owned_lifecycle", None)
        adapter = getattr(lifecycle, "_adapter", None)
        prime_recorders = getattr(adapter, "prime_recorders", None)
        if callable(prime_recorders):
            prime_recorders(ids)
        self._reseed_observation_delay(ids)

    def _reseed_observation_delay(self, env_ids: torch.Tensor) -> None:
        # _reset_idx seeds delay history with the old observation. Defer reseeding
        # until the next normal step; extracting out of band would advance counters
        # for every environment. The runtime ignores this when delay is disabled.
        delay_runtime = getattr(self._env, "_delay_runtime", None)
        mark_reset_reseed = getattr(delay_runtime, "mark_reset_reseed", None)
        if callable(mark_reset_reseed):
            mark_reset_reseed(env_ids)

    def _render_after_partial_reset(self) -> None:
        if not self._env.sim.has_rtx_sensors():
            return

        if self._env.cfg.rerender_on_reset:
            self._env.sim.render()
        if self._env.cfg.wait_for_textures:
            from srb.core.env.runtime.lifecycle import wait_for_assets_loaded

            wait_for_assets_loaded(
                _simulation_manager().assets_loading,
                self._env.sim.render,
            )

    def _create_service(
        self,
        service_type: Any,
        topic: str,
        callback: Callable,
    ) -> Any:
        with self._lifecycle_lock:
            if self._close_requested:
                raise RuntimeError("ROS reset command is closing")
            service = self._node.create_service(service_type, topic, callback)
            self._owned_endpoints.append((service, "service", False))
            return service

    def close(self) -> None:
        """Detach every reset service created by this module.

        A failed service destruction stays in the registry for a later retry;
        successful services are never destroyed twice. Borrowed nodes remain
        untouched.
        """
        with self._lifecycle_lock:
            if self._closed:
                return
            self._close_requested = True
            errors: list[BaseException] = []
            for index, (endpoint, kind, complete) in enumerate(self._owned_endpoints):
                if complete:
                    continue
                try:
                    destroy_ros_endpoint(self._node, endpoint, kind=kind)
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(exc)
                else:
                    self._owned_endpoints[index] = (endpoint, kind, True)
            if errors:
                raise RosEndpointCleanupError(
                    "ROS reset endpoint cleanup incomplete",
                    errors=tuple(errors),
                )
            self._closed = True


__all__ = ["RosResetCommand"]
