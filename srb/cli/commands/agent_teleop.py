"""CLI adapter for ``srb agent teleop`` workflows."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Mapping, Sequence

import numpy as np

from srb.cli.commands import agent_collection as _collection_cli
from srb.cli.commands import agent_policy as _policy_cli
from srb.experiments.policies import infer_algo_from_model
from srb.interfaces.runtime import find_env_interface_node

if TYPE_CHECKING:
    from pathlib import Path

    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv
    from srb.interfaces.teleop import CombinedTeleopInterface


should_reset = False
_resolve_teleop_devices = _collection_cli._resolve_teleop_devices
_resolve_model_artifact_policy_inputs = _policy_cli.resolve_model_artifact_policy_inputs


def _disable_command_events_for_teleop(
    event_manager: object,
    recognized_cmd_keys: Sequence[str],
) -> tuple[tuple[str, object, object, object], ...]:
    """Remove command events and report the manager's live terms afterward."""

    from srb.core.randomization import (
        live_event_terms,
        remove_event_terms_by_name_fragments,
    )

    remove_event_terms_by_name_fragments(event_manager, recognized_cmd_keys)
    return tuple(
        (
            name,
            getattr(term, "mode", None),
            getattr(term, "category", None),
            getattr(term, "interval_range_s", None),
        )
        for name, term in live_event_terms(event_manager)
    )


def teleop_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    headless: bool,
    teleop_device: Sequence[str] | None,
    pos_sensitivity: float,
    rot_sensitivity: float,
    algo: str | None = None,
    model: "Path | str | None" = None,
    recognized_cmd_keys: Sequence[str] = (
        "cmd",
        "command",
        "goal",
        "target",
    ),
    addititive_cmd_keys: Sequence[str] = (
        "goal",
        "target",
    ),
    **kwargs,
) -> None:
    if _collection_cli._is_real_env(env):
        if algo is not None or model is not None:
            raise ValueError(
                "Native direct teleoperation does not support --algo or --model"
            )
        _teleop_agent_real(
            env=env,
            sim_app=sim_app,
            teleop_device=teleop_device,
            pos_sensitivity=pos_sensitivity,
            rot_sensitivity=rot_sensitivity,
            invert_controls=kwargs.pop("invert_controls", True),
            real_agent_session=kwargs.get("real_agent_session"),
        )
        return

    from srb.core.action import ActionTermCfg
    from srb.core.action.command import env_supports_direct_action_command
    from srb.interfaces import TeleopDeviceType
    from srb.interfaces.teleop import CombinedTeleopInterface
    from srb.utils.config_introspection import iter_config_items_by_type

    # Keep the operator's reference (checkpoint or model artifact) for the
    # policy rollout; resolving an artifact here only supplies its algorithm.
    policy_model = model
    algo, model = _resolve_model_artifact_policy_inputs(algo=algo, model=model)

    # Infer algo from model if model provided without algo
    if algo is None and model is not None:
        algo = infer_algo_from_model(model)
        from srb.utils import logging

        logging.info(f"Inferred algorithm '{algo}' from model path: {model}")

    teleop_device = _resolve_teleop_devices(env, teleop_device)
    if headless and TeleopDeviceType.KEYBOARD in teleop_device:
        raise ValueError(
            "Teleoperation with the keyboard is only supported in GUI mode. "
            'Rerun without "--headless" or choose a non-keyboard "--teleop_device".'
        )
    if hasattr(env.unwrapped.cfg, "truncate_episodes"):  # type: ignore[misc]
        env.unwrapped.cfg.truncate_episodes = False  # type: ignore[attr-defined]
    ros_node = find_env_interface_node(env)

    teleop_interface = CombinedTeleopInterface(
        devices=teleop_device,  # type: ignore[misc]
        node=ros_node,
        pos_sensitivity=pos_sensitivity,
        rot_sensitivity=rot_sensitivity,
        actions=env.unwrapped.cfg.actions,  # type: ignore[attr-defined]
    )

    def cb_reset() -> None:
        global should_reset
        should_reset = True

    global should_reset
    should_reset = False
    teleop_interface.add_callback("L", cb_reset)

    teleop_interface.reset()
    print(teleop_interface)
    env.reset()

    env_supports_direct_teleop = env_supports_direct_action_command(env)

    env_supports_teleop_via_policy = (
        next(
            (
                key
                for key in (
                    *recognized_cmd_keys,
                    *map(lambda x: "_" + x, recognized_cmd_keys),
                    *map(lambda x: "__" + x, recognized_cmd_keys),
                )
                if hasattr(env.unwrapped, key)
            ),
            None,
        )
        is not None
    )
    if algo and env_supports_teleop_via_policy:
        _teleop_agent_via_policy(
            env=env,
            sim_app=sim_app,
            teleop_interface=teleop_interface,
            algo=algo,
            model=policy_model,
            recognized_cmd_keys=recognized_cmd_keys,
            addititive_cmd_keys=addititive_cmd_keys,
            **kwargs,
        )
    elif env_supports_direct_teleop:
        _teleop_agent_direct(
            env=env,
            sim_app=sim_app,
            teleop_interface=teleop_interface,
            **kwargs,
        )
    elif env_supports_teleop_via_policy:
        raise ValueError(
            f'Environment "{env}" can only be teleoperated via policy. Please provide a policy via "--algo" and an optional "--model" argument.'
        )
    else:
        action_terms = {
            action_key: action_term.__class__.__name__
            for action_key, action_term in iter_config_items_by_type(
                env.unwrapped.cfg.actions,  # type: ignore[attr-defined]
                ActionTermCfg,
            )
        }
        raise ValueError(
            f"Direct teleoperation is not supported for an action group with terms: {action_terms}"
        )


def _teleop_agent_real(
    *,
    env: Any,
    sim_app: Any,
    teleop_device: Sequence[str] | None,
    pos_sensitivity: float,
    rot_sensitivity: float,
    invert_controls: bool,
    real_agent_session: Any,
) -> None:
    """Run scalar RealEnv direct teleoperation through its declared mapping."""
    if real_agent_session is None:
        raise RuntimeError(
            "Direct teleoperation of RealEnv requires a RealAgentSession owner"
        )

    from srb.cli.commands.real_agent_teleop import prepare_real_teleop
    from srb.core.command_mapping import apply_command_mapping
    from srb.utils import logging

    real_env = getattr(env, "unwrapped", env)
    interface = prepare_real_teleop(
        real_env,
        session=real_agent_session,
        teleop_device=teleop_device,
        pos_sensitivity=pos_sensitivity,
        rot_sensitivity=rot_sensitivity,
    )
    mapping = real_env.command_mapping
    if mapping is None:
        raise ValueError("RealEnv does not declare direct command mapping metadata")

    reset_requested = False

    def request_reset() -> None:
        nonlocal reset_requested
        reset_requested = True

    if "L" in interface.callback_keys:
        interface.add_callback("L", request_reset)
    interface.reset()
    print(interface)
    real_env.reset()
    episode_done = False

    while sim_app.is_running():
        if reset_requested or episode_done:
            reset_requested = False
            interface.reset()
            real_env.reset()
            episode_done = False

        command, event = interface.advance()
        if invert_controls and mapping.task_family == "manipulation":
            command[:2] *= -1.0

        import torch

        mapped = apply_command_mapping(
            mapping,
            torch.as_tensor(command, dtype=torch.float32),
            bool(event),
        )
        action = (
            mapped.detach()
            .cpu()
            .numpy()
            .astype(real_env.single_action_space.dtype, copy=False)
        )
        observation, reward, terminated, truncated, info = env.step(action)
        logging.trace(
            f"action: {action}\n"
            f"observation: {observation}\n"
            f"reward: {reward}\n"
            f"terminated: {terminated}\n"
            f"truncated: {truncated}\n"
            f"info: {info}\n"
        )
        episode_done = bool(terminated) or bool(truncated)


def _teleop_agent_direct(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    teleop_interface: "CombinedTeleopInterface",
    invert_controls: bool,
    **kwargs,
) -> None:
    import torch

    from srb.core.action.command import map_direct_action_command
    from srb.core.env import ManipulationEnv
    from srb.utils import logging

    if invert_controls:
        invert_controls = isinstance(env.unwrapped, ManipulationEnv)

    with torch.inference_mode():
        while sim_app.is_running():
            twist, event = teleop_interface.advance()
            # Invert only for manipulation environments.
            if invert_controls:
                twist[:2] *= -1.0
            action = map_direct_action_command(env, twist, event).repeat(
                env.unwrapped.num_envs,  # type: ignore[misc]
                1,
            )

            observation, reward, terminated, truncated, info = env.step(action)
            logging.trace(
                f"action: {action}\n"
                f"observation: {observation}\n"
                f"reward: {reward}\n"
                f"terminated: {terminated}\n"
                f"truncated: {truncated}\n"
                f"info: {info}\n"
            )

            # Provide force feedback for teleop devices.
            if teleop_interface.ft_feedback_interfaces:
                if isinstance(env.unwrapped, ManipulationEnv):
                    if env.unwrapped._contacts_end_effector is not None:
                        contacts_end_effector = env.unwrapped._contacts_end_effector
                        contact_forces = (
                            contacts_end_effector.data.net_forces_w  # type: ignore[misc]
                        )[0].mean(dim=0)
                        contact_ft = torch.cat(
                            [
                                contact_forces,
                                torch.zeros(
                                    3,
                                    device=contact_forces.device,
                                    dtype=contact_forces.dtype,
                                ),
                            ]
                        )
                        ft_feedback: torch.Tensor = (
                            torch.tensor([0.33, 0.33, 0.33, 0.0, 0.0, 0.0])
                            * contact_ft.cpu()
                        )
                        teleop_interface.set_ft_feedback(ft_feedback)

            global should_reset
            if should_reset:
                should_reset = False
                teleop_interface.reset()
                observation, info = env.reset()


def _teleop_agent_via_policy(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    teleop_interface: "CombinedTeleopInterface",
    invert_controls: bool,
    recognized_cmd_keys: Sequence[str],
    addititive_cmd_keys: Sequence[str],
    algo: str | None = None,
    model: "Path | str | None" = None,
    **kwargs,
) -> None:
    import torch
    from gymnasium.core import ActType, SupportsFloat, Wrapper, WrapperObsType

    from srb.utils import logging
    from srb.utils.math import quat_mul, rpy_to_quat

    # As in direct teleoperation, inversion applies only to manipulation.
    if invert_controls:
        from srb.core.env import ManipulationEnv

        invert_controls = isinstance(env.unwrapped, ManipulationEnv)

    # Disable command events and report the post-removal live manager view.
    event_manager = getattr(env.unwrapped, "event_manager", None)
    if event_manager is not None:
        logging.debug(
            "teleop live randomization terms after command removal: %s",
            _disable_command_events_for_teleop(
                event_manager,
                recognized_cmd_keys,
            ),
        )

    class TeleopWrapper(Wrapper):
        def __init__(self, env, *args, **kwargs) -> None:
            super().__init__(env, *args, **kwargs)

            self._internal_cmd_attr_name = next(
                (
                    key
                    for key in (
                        *recognized_cmd_keys,
                        *map(lambda x: "_" + x, recognized_cmd_keys),
                        *map(lambda x: "__" + x, recognized_cmd_keys),
                    )
                    if hasattr(self.env.unwrapped, key)
                ),
                None,
            )
            if not self._internal_cmd_attr_name:
                raise ValueError(
                    "Unable to find the internal command attribute of the environment"
                )
            internal_cmd_attr_shape = getattr(
                self.env.unwrapped, self._internal_cmd_attr_name
            ).shape
            self._is_internal_cmd_attr_per_env = len(internal_cmd_attr_shape) == 2 and (
                internal_cmd_attr_shape[0] == self.env.unwrapped.cfg.scene.num_envs  # type: ignore[attr-defined]
            )
            self._is_cmd_additive = any(
                key in self._internal_cmd_attr_name for key in addititive_cmd_keys
            )

        def step(
            self,
            action: ActType,  # type: ignore[misc]
        ) -> tuple[WrapperObsType, SupportsFloat, bool, bool, Mapping[str, Any]]:  # type: ignore[misc]
            if not sim_app.is_running():
                exit(0)

            twist, event = teleop_interface.advance()
            if invert_controls:
                twist[:2] *= -1.0

            cmd_len = getattr(env.unwrapped, self._internal_cmd_attr_name).shape[-1]  # type: ignore[misc]
            match cmd_len:
                case _ if cmd_len < 7:
                    cmd = torch.from_numpy(twist[:cmd_len]).to(
                        device=env.unwrapped.device,  # type: ignore[misc]
                        dtype=torch.float32,
                    )
                    setattr(
                        env.unwrapped,
                        self._internal_cmd_attr_name,  # type: ignore[misc]
                        (
                            cmd.repeat(
                                self.env.unwrapped.cfg.scene.num_envs,  # type: ignore[attr-defined]
                                1,
                            )
                            if self._is_internal_cmd_attr_per_env
                            else cmd
                        )
                        if not self._is_cmd_additive
                        else (
                            getattr(env.unwrapped, self._internal_cmd_attr_name)  # type: ignore[misc]
                            + cmd
                        ),
                    )
                case 7:
                    if self._is_cmd_additive:
                        cmd = getattr(env.unwrapped, self._internal_cmd_attr_name)  # type: ignore[misc]
                        cmd[..., 0:3] += torch.as_tensor(
                            twist[0:3], device=cmd.device, dtype=cmd.dtype
                        )
                        cmd[..., 3:7] = quat_mul(
                            torch.tensor(
                                # twist[3:6] are rotation deltas in RADIANS
                                # (rot_sensitivity scales them by ~pi); pass
                                # deg=False so they are not misread as degrees.
                                rpy_to_quat(*twist[3:6], deg=False),
                                device=env.unwrapped.device,  # type: ignore[misc]
                                dtype=torch.float32,
                            ).repeat(
                                self.env.unwrapped.cfg.scene.num_envs,  # type: ignore[attr-defined]
                                1,
                            ),
                            cmd[..., 3:7],
                        )
                        setattr(env.unwrapped, self._internal_cmd_attr_name, cmd)  # type: ignore[misc]
                    else:
                        cmd = torch.concat(
                            (
                                torch.from_numpy(twist).to(
                                    device=env.unwrapped.device,  # type: ignore[misc]
                                    dtype=torch.float32,
                                ),
                                torch.Tensor((-1.0 if event else 1.0,)).to(
                                    device=env.unwrapped.device,  # type: ignore[misc]
                                ),
                            )
                        )
                        setattr(
                            env.unwrapped,
                            self._internal_cmd_attr_name,  # type: ignore[misc]
                            (
                                cmd.repeat(
                                    self.env.unwrapped.cfg.scene.num_envs,  # type: ignore[attr-defined]
                                    1,
                                )
                                if self._is_internal_cmd_attr_per_env
                                else cmd
                            ),
                        )
                case _:
                    raise ValueError(
                        f"Unsupported command length for teleoperation: {cmd_len}"
                    )

            observation, reward, terminated, truncated, info = super().step(action)
            logging.trace(
                f"action: {action}\n"
                f"observation: {observation}\n"
                f"reward: {reward}\n"
                f"terminated: {terminated}\n"
                f"truncated: {truncated}\n"
                f"info: {info}\n"
            )

            global should_reset
            if should_reset:
                should_reset = False
                teleop_interface.reset()
                observation, info = env.reset()
                # A manual reset starts a new episode. Report the boundary to
                # eval_agent so recurrent policies reset their hidden state.
                num_envs = env.unwrapped.num_envs  # type: ignore[misc]
                terminated = np.ones(num_envs, dtype=np.bool_)
                truncated = np.zeros(num_envs, dtype=np.bool_)

            return (
                observation,  # type: ignore[misc]
                reward,
                terminated,
                truncated,
                info,
            )

    from srb.cli.commands.agent_eval import eval_agent

    env = TeleopWrapper(env)  # type: ignore[misc]
    # ``model`` is the operator's ``--model`` (checkpoint or artifact); ``None``
    # keeps checkpoint discovery in the resolved policy run directory.
    eval_agent(env=env, sim_app=sim_app, algo=algo, model=model, **kwargs)


__all__ = (
    "teleop_agent",
    "_teleop_agent_direct",
    "_teleop_agent_via_policy",
)
