"""CLI adapter for ROS-driven agents."""

from __future__ import annotations

import math
import sys
import time
from typing import TYPE_CHECKING

from srb.interfaces._ros_node import RosCleanupScope
from srb.interfaces.runtime import (
    EnvInterfaceWrapper,
    append_env_interface,
    find_env_interface,
    find_env_interface_node,
    has_env_interfaces,
    remove_env_interface,
)

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv


def ros_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    ros_sync: bool = False,
    ros_action_timeout: float | None = None,
    ros_unthrottled: bool = False,
    **kwargs,
) -> None:
    """Run an agent controlled by ROS action subscriptions."""
    real_env = getattr(env, "unwrapped", env)
    real_agent_session = kwargs.get("real_agent_session")
    if real_agent_session is not None:
        from srb.interfaces.sim_to_real import RealEnv

        if not isinstance(real_env, RealEnv):
            raise RuntimeError(
                "real_agent_session was provided for a non-RealEnv ROS agent"
            )
        _real_env_ros_agent(
            env=env,
            real_env=real_env,
            sim_app=sim_app,
            real_agent_session=real_agent_session,
            ros_sync=ros_sync,
            ros_action_timeout=ros_action_timeout,
        )
        return

    # A constructed RealEnv necessarily loaded this implementation module.
    # Inspect it without importing the real runtime on the simulator path.
    real_env_module = sys.modules.get("srb.interfaces.sim_to_real.core.env")
    real_env_type = getattr(real_env_module, "RealEnv", None)
    if real_env_type is not None and isinstance(real_env, real_env_type):
        raise RuntimeError(
            "ROS control of RealEnv requires an active RealAgentSession owner"
        )

    from srb.interfaces.ros_action_io import validate_command_timeout

    ros_action_timeout = validate_command_timeout(ros_action_timeout)

    import torch

    from srb.interfaces.interface.ros import RosInterface
    from srb.utils import logging

    if hasattr(env.unwrapped.cfg, "truncate_episodes"):  # type: ignore[misc]
        env.unwrapped.cfg.truncate_episodes = False  # type: ignore[attr-defined]
    ros_interface = find_env_interface(env, RosInterface)
    ros_node = find_env_interface_node(env)
    should_update_interface = False
    owns_ros_interface = False
    created_ros_interface = False
    registered_reset_hook = None
    if ros_interface is None:
        interface_wrapper_updates = has_env_interfaces(env)
        created_interface = RosInterface(env=env, node=ros_node)
        created_ros_interface = True
        with RosCleanupScope(
            created_interface,
            "ROS agent interface attachment cleanup incomplete",
        ):
            append_env_interface(env, created_interface)
        ros_interface = created_interface
        owns_ros_interface = not isinstance(env, EnvInterfaceWrapper)
        should_update_interface = not interface_wrapper_updates

    interface_closed = not owns_ros_interface
    interface_detached = not owns_ros_interface
    reset_hook_detached = False

    def _cleanup_ros_agent() -> None:
        nonlocal interface_closed, interface_detached, reset_hook_detached
        if owns_ros_interface and not interface_closed:
            close = getattr(ros_interface, "close", None)
            if callable(close):
                close()
            interface_closed = True
        if owns_ros_interface and not interface_detached:
            remove_env_interface(env, ros_interface)
            interface_detached = True
        if not reset_hook_detached:
            if registered_reset_hook is None:
                reset_hook_detached = True
            else:
                reset_hooks = getattr(env.unwrapped, "_kb_reset_hooks", None)
                if reset_hooks is not None:
                    reset_hooks[:] = [
                        hook
                        for hook in reset_hooks
                        if hook is not registered_reset_hook
                    ]
                reset_hook_detached = True

    cleanup_scope = RosCleanupScope(
        ros_interface,
        "ROS agent interface cleanup incomplete",
        always=True,
        cleanup=_cleanup_ros_agent,
    )
    with cleanup_scope:
        if created_ros_interface:
            env.unwrapped.cfg.extras = True  # type: ignore[attr-defined]
        ros_interface.setup_action_sub()

        if hasattr(env.unwrapped, "_kb_reset_hooks"):
            registered_reset_hook = ros_interface.clear_actions
            env.unwrapped._kb_reset_hooks.append(registered_reset_hook)

        ros_step_period = _resolve_ros_step_period(
            env,
            ros_sync=ros_sync,
            ros_unthrottled=ros_unthrottled,
        )
        next_ros_step_time = time.monotonic()

        last_action_sequence = ros_interface.action_sequence
        sync_ready = False
        with torch.inference_mode():
            while sim_app.is_running():
                if ros_sync and sync_ready:
                    got_fresh_action = ros_interface.wait_for_action(
                        last_action_sequence,
                        timeout=ros_action_timeout,
                    )
                    if not got_fresh_action:
                        ros_interface.clear_actions()
                        logging.warning(
                            "Timed out waiting for a fresh ROS action; cleared the "
                            "cached command and applying the zero-action deadman."
                        )
                with ros_interface.action_transaction(
                    block=ros_sync and sync_ready
                ) as (action, last_action_sequence):
                    step_result = env.step(action)  # type: ignore[misc]
                observation, reward, terminated, truncated, info = step_result
                logging.trace(
                    f"action: {action}\n"
                    f"observation: {observation}\n"
                    f"reward: {reward}\n"
                    f"terminated: {terminated}\n"
                    f"truncated: {truncated}\n"
                    f"info: {info}\n"
                )

                if should_update_interface:
                    ros_interface.update(
                        observation,  # type: ignore[misc]
                        reward,
                        terminated,
                        truncated,
                        info,
                    )
                sync_ready = True
                if ros_step_period is not None:
                    next_ros_step_time += ros_step_period
                    sleep_duration = next_ros_step_time - time.monotonic()
                    if sleep_duration > 0.0:
                        time.sleep(sleep_duration)
                    else:
                        next_ros_step_time = time.monotonic()


def _resolve_ros_step_period(
    env: "AnyEnv", *, ros_sync: bool, ros_unthrottled: bool
) -> float | None:
    if ros_sync or ros_unthrottled:
        return None
    cfg = getattr(getattr(env, "unwrapped", env), "cfg", None)
    step_period = getattr(cfg, "agent_rate", None)
    if step_period is None:
        return None
    step_period = float(step_period)
    # Pacing is disabled only by ``--ros_sync`` or ``--ros_unthrottled``; a
    # zero, negative or NaN period is rejected rather than unthrottling.
    if not math.isfinite(step_period) or step_period <= 0.0:
        raise ValueError(
            "env.agent_rate (the ROS agent step period in seconds) must be "
            f"finite and positive, got {step_period!r}; use --ros_unthrottled "
            "to disable wall-clock pacing"
        )
    return step_period


def _real_env_ros_agent(
    *,
    env,
    real_env,
    sim_app,
    real_agent_session,
    ros_sync: bool,
    ros_action_timeout: float | None,
) -> None:
    """Run RealEnv from validated scalar raw-action samples."""
    if real_agent_session is None:
        raise RuntimeError(
            "ROS control of RealEnv requires an active RealAgentSession owner"
        )

    from srb.core.env.introspect import EnvIntrospector
    from srb.interfaces.sim_to_real.core.ros_action_source import (
        RawPolicyActionSource,
    )
    from srb.utils import logging

    source = RawPolicyActionSource(
        env=real_env,
        session=real_agent_session,
        single_action_space=EnvIntrospector.of(env).single_action_space,
        ros_action_timeout=ros_action_timeout,
    )
    sample = None
    last_sequence = 0
    stepped = False
    while sim_app.is_running():
        if sample is None:
            sample = source.next_sample()
            last_sequence = sample.sequence
        elif ros_sync and stepped:
            sample = source.next_sample(
                after_sequence=last_sequence,
                prior_deadline=sample.deadline,
            )
        elif not ros_sync:
            sample = source.latest_sample()

        with real_env.command_dispatch_precondition(
            lambda selected=sample: source.check_dispatch(selected)
        ):
            step_result = env.step(sample.action)
        observation, reward, terminated, truncated, info = step_result
        logging.trace(
            f"action: {sample.action}\n"
            f"observation: {observation}\n"
            f"reward: {reward}\n"
            f"terminated: {terminated}\n"
            f"truncated: {truncated}\n"
            f"info: {info}\n"
        )
        last_sequence = sample.sequence
        stepped = True


__all__ = (
    "ros_agent",
    "_resolve_ros_step_period",
    "_real_env_ros_agent",
)
