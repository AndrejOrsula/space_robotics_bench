#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
import argparse
import os
import shutil
import sys
from enum import Enum, auto
from importlib.util import find_spec
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Literal, Mapping, Sequence, Tuple

from typing_extensions import Self

from srb.interfaces import InterfaceType, TeleopDeviceType
from srb.utils.cache import (
    read_offline_srb_env_cache,
    read_offline_srb_hardware_interface_cache,
    update_offline_srb_cache,
)
from srb.utils.path import SRB_APPS_DIR, SRB_DIR, SRB_LOGS_DIR

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv
    from srb.interfaces.teleop import CombinedTeleopInterface


def main():
    def impl(
        subcommand: Literal[
            "agent", "real_agent", "list", "ls", "gui", "repl", "python", "docs", "test"
        ],
        **kwargs,
    ):
        if not find_spec("omni"):
            raise ImportError(
                "The Space Robotics Bench requires an environment with NVIDIA Omniverse and Isaac Sim installed."
            )

        match subcommand:
            case "agent":
                run_agent(**kwargs)
            case "real_agent":
                run_real_agent(**kwargs)
            case "ls" | "list":
                list_registered(**kwargs)
            case "gui":
                launch_gui(**kwargs)
            case "repl" | "python":
                enter_repl(**kwargs)
            case "docs":
                serve_docs(**kwargs)
            case "test":
                run_tests(**kwargs)

    impl(**vars(parse_cli_args()))


### Agent ###
def run_agent(
    agent_subcommand: Literal[
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
        "learn",
    ],
    **kwargs,
):
    # Run the implementation
    def agent_impl(**kwargs):
        match agent_subcommand:
            case "learn":
                raise NotImplementedError()
            case _:
                run_agent_with_env(agent_subcommand=agent_subcommand, **kwargs)

    agent_impl(**kwargs)


def run_agent_with_env(
    agent_subcommand: Literal[
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
    ],
    env_id: str,
    logdir_path: str,
    interface: Sequence[str],
    video_enable: bool,
    perf_enable: bool,
    perf_output: str,
    perf_duration: float,
    headless: bool,
    hide_ui: bool,
    forwarded_args: Sequence[str] = (),
    **kwargs,
):
    from srb.core.app import AppLauncher

    # Preprocess kwargs
    kwargs["enable_cameras"] = video_enable or env_id.endswith("_visual")
    kwargs["experience"] = SRB_APPS_DIR.joinpath(
        f"srb.{'headless.' if headless else ''}{'rendering.' if kwargs['enable_cameras'] else ''}{'xr.' if kwargs['xr'] else ''}kit"
    )

    # Launch Isaac Sim
    launcher = AppLauncher(headless=headless, **kwargs)

    # Update the offline registry cache
    update_offline_srb_cache()

    from omni.physx import acquire_physx_interface

    from srb.interfaces.teleop import EventOmniKeyboardTeleopInterface
    from srb.utils.cfg import last_logdir, new_logdir
    from srb.utils.hydra.sim import hydra_task_config
    from srb.utils.isaacsim import hide_isaacsim_ui

    # Post-launch configuration
    acquire_physx_interface().overwrite_gpu_setting(1)
    if hide_ui:
        hide_isaacsim_ui()

    # Get the log directory based on the workflow
    workflow = kwargs.get("algo") or agent_subcommand
    logdir_root = Path(logdir_path).resolve()
    if model := kwargs.get("model"):
        model = Path(model).resolve()
        assert model.exists(), f"Model path does not exist: {model}"
        logdir = model.parent
        while not (
            logdir.parent.name == workflow
            and (logdir.parent.parent.name == env_id.rsplit("/", 1)[-1])
        ):
            _new_parent = logdir.parent
            if logdir == _new_parent:
                logdir = new_logdir(env_id=env_id, workflow=workflow, root=logdir_root)
                model_symlink_path = logdir.joinpath(model.name)
                model_symlink_path.parent.mkdir(parents=True, exist_ok=True)
                os.symlink(model, model_symlink_path)
                model = model_symlink_path
                break
            logdir = _new_parent
        kwargs["model"] = model
    elif (agent_subcommand == "train" and kwargs["continue_training"]) or (
        agent_subcommand in ("eval", "teleop") and kwargs["algo"]
    ):
        logdir = last_logdir(env_id=env_id, workflow=workflow, root=logdir_root)
    else:
        logdir = new_logdir(env_id=env_id, workflow=workflow, root=logdir_root)

    # Update Hydra output directory
    if not any(arg.startswith("hydra.run.dir=") for arg in forwarded_args):
        sys.argv.extend([f"hydra.run.dir={logdir.as_posix()}"])
    maybe_config_path = Path(logdir).joinpath(".hydra", "config.yaml").resolve()

    @hydra_task_config(
        task_name=env_id,
        agent_cfg_entry_point=f"{kwargs['algo']}_cfg" if kwargs.get("algo") else None,
        config_path=maybe_config_path.as_posix()
        if maybe_config_path.exists()
        else None,
    )
    def hydra_main(env_cfg: Dict[str, Any], agent_cfg: Dict[str, Any] | None = None):
        import gymnasium

        # Create the environment and initialize it
        env = gymnasium.make(
            id=env_id, cfg=env_cfg, render_mode="rgb_array" if video_enable else None
        )
        env.reset()

        # Add wrapper for video recording
        if video_enable:
            from datetime import datetime

            env = gymnasium.wrappers.RecordVideo(
                env,
                video_folder=logdir.joinpath("videos")
                .joinpath(datetime.now().strftime("%Y%m%d_%H%M%S"))
                .as_posix(),
                name_prefix=env_id.rsplit("/", 1)[-1],
                disable_logger=True,
                episode_trigger=lambda _: True,
            )

        # Add wrapper for performance tests
        if perf_enable:
            env = __wrap_env_in_performance_test(
                env=env,  # type: ignore
                sim_app=launcher.app,
                perf_output=perf_output,
                perf_duration=perf_duration,
            )

        # Add keyboard callbacks
        if not headless and agent_subcommand not in [
            "teleop",
            "collect",
            "train",
        ]:
            _cb_keyboard = EventOmniKeyboardTeleopInterface({"L": env.reset})

        # Create interfaces
        if interface:
            import threading

            from gymnasium.core import (
                SupportsFloat,
                Wrapper,
                WrapperActType,
                WrapperObsType,
            )

            from srb.utils.ros import enable_ros2_bridge

            enable_ros2_bridge()

            import rclpy
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node

            rclpy.init()
            ros_node = Node("sim", namespace="srb", start_parameter_services=False)  # type: ignore
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(ros_node)
            thread = threading.Thread(target=executor.spin)
            thread.daemon = True
            thread.start()

            class InterfaceWrapper(Wrapper):
                def __init__(self, env, *args, **kwargs):
                    super().__init__(env, *args, **kwargs)
                    self._srb_interfaces = tuple(
                        iface.implementer()(env=env, node=ros_node)  # type: ignore
                        for iface in set(map(InterfaceType.from_str, interface))
                    )

                def step(
                    self,
                    action: WrapperActType,  # type: ignore
                ) -> tuple[WrapperObsType, SupportsFloat, bool, bool, dict[str, Any]]:  # type: ignore
                    step_return = super().step(action)
                    for interface in self._srb_interfaces:
                        interface.update(*step_return, action=action)
                    return step_return

            env = InterfaceWrapper(env)  # type: ignore
            env.unwrapped.ros_node = ros_node
            env.unwrapped.cfg.extras = True  # type: ignore

        # Run the implementation
        def agent_impl(**kwargs):
            kwargs.update(
                {
                    "env_id": env_id,
                    "agent_cfg": agent_cfg,
                    "env_cfg": env_cfg,
                }
            )

            match agent_subcommand:
                case "zero":
                    zero_agent(**kwargs)
                case "rand":
                    random_agent(**kwargs)
                case "teleop":
                    teleop_agent(headless=headless, **kwargs)
                case "ros":
                    ros_agent(**kwargs)
                case "train":
                    train_agent(**kwargs)
                case "eval":
                    eval_agent(**kwargs)
                case "collect":
                    raise NotImplementedError()

        agent_impl(env=env, sim_app=launcher.app, logdir=logdir, **kwargs)

        # Close the environment
        env.close()

    hydra_main()  # type: ignore

    # Shutdown Isaac Sim
    launcher.app.close()


def random_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    **kwargs,
):
    import torch

    from srb.utils import logging

    with torch.inference_mode():
        while sim_app.is_running():
            action = torch.from_numpy(env.action_space.sample()).to(
                device=env.unwrapped.device  # type: ignore
            )
            observation, reward, terminated, truncated, info = env.step(action)  # type: ignore
            logging.trace(
                f"action: {action}\n"
                f"observation: {observation}\n"
                f"reward: {reward}\n"
                f"terminated: {terminated}\n"
                f"truncated: {truncated}\n"
                f"info: {info}\n"
            )


def zero_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    **kwargs,
):
    import torch

    from srb.utils import logging

    action = torch.zeros(env.action_space.shape, device=env.unwrapped.device)  # type: ignore

    with torch.inference_mode():
        while sim_app.is_running():
            observation, reward, terminated, truncated, info = env.step(action)
            logging.trace(
                f"action: {action}\n"
                f"observation: {observation}\n"
                f"reward: {reward}\n"
                f"terminated: {terminated}\n"
                f"truncated: {truncated}\n"
                f"info: {info}\n"
            )


def teleop_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    headless: bool,
    teleop_device: Sequence[str],
    pos_sensitivity: float,
    rot_sensitivity: float,
    algo: str,
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
):
    import torch

    from srb.core.action import ActionGroup, ActionTermCfg
    from srb.interfaces.teleop import CombinedTeleopInterface

    teleop_device = list(set(map(TeleopDeviceType.from_str, teleop_device)))  # type: ignore

    ## Ensure that a feasible teleoperation device is selected
    if (
        headless
        and len(teleop_device) == 1
        and TeleopDeviceType.KEYBOARD in teleop_device
    ):
        raise ValueError(
            'Teleoperation with the keyboard is only supported in GUI mode. Consider disabling the "--headless" mode or using a different "--teleop_device".'
        )

    ## Disable truncation
    if hasattr(env.unwrapped.cfg, "truncate_episodes"):  # type: ignore
        env.unwrapped.cfg.truncate_episodes = False  # type: ignore

    ## Try to get ROS node from interfaces
    ros_node = None
    if hasattr(env, "_srb_interfaces"):
        for iface in env._srb_interfaces:  # type: ignore
            if hasattr(iface, "_node"):
                ros_node = iface._node  # type: ignore
                break

    ## Create teleop interface
    teleop_interface = CombinedTeleopInterface(
        devices=teleop_device,  # type: ignore
        node=ros_node,
        pos_sensitivity=pos_sensitivity,
        rot_sensitivity=rot_sensitivity,
        actions=env.unwrapped.cfg.actions,  # type: ignore
    )

    ## Set up reset callback
    def cb_reset():
        global should_reset
        should_reset = True

    global should_reset
    should_reset = False
    teleop_interface.add_callback("L", cb_reset)

    ## Initialize the teleop interface via reset
    teleop_interface.reset()
    print(teleop_interface)

    ## Initialize the environment
    env.reset()

    ## Determine if the environment supports direct teleoperation
    if hasattr(
        env.unwrapped.cfg,  # type: ignore
        "actions",
    ) and isinstance(
        env.unwrapped.cfg.actions,  # type: ignore
        ActionGroup,
    ):
        try:
            env.unwrapped.cfg.actions.map_cmd_to_action(torch.zeros(6), False)  # type: ignore
            env_supports_direct_teleop = True
        except NotImplementedError:
            env_supports_direct_teleop = False
    else:
        env_supports_direct_teleop = False

    ## Dispatch the appropriate implementation
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
            for action_key, action_term in env.unwrapped.cfg.actions.__dict__.items()  # type: ignore
            if isinstance(action_term, ActionTermCfg)
        }
        raise ValueError(
            f"Direct teleoperation is not supported for an action group with terms: {action_terms}"
        )


def _teleop_agent_direct(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    teleop_interface: "CombinedTeleopInterface",
    invert_controls: bool,
    ft_feedback_use_contacts: bool = True,
    **kwargs,
):
    import torch

    from srb.core.env import ManipulationEnv
    from srb.utils import logging

    # Invert only for manipulation environments
    if invert_controls:
        invert_controls = isinstance(env.unwrapped, ManipulationEnv)
    if teleop_interface.ft_feedback_interfaces:
        ft_feedback_use_contacts = ft_feedback_use_contacts

    ## Run the environment
    with torch.inference_mode():
        while sim_app.is_running():
            ## Get actions from the teleoperation interface and process them
            twist, event = teleop_interface.advance()
            if invert_controls:
                twist[:2] *= -1.0
            action = env.unwrapped.cfg.actions.map_cmd_to_action(  # type: ignore
                torch.from_numpy(twist).to(
                    device=env.unwrapped.device,  # type: ignore
                    dtype=torch.float32,
                ),
                event,
            ).repeat(
                env.unwrapped.num_envs,  # type: ignore
                1,
            )

            # Step the environment
            observation, reward, terminated, truncated, info = env.step(action)
            logging.trace(
                f"action: {action}\n"
                f"observation: {observation}\n"
                f"reward: {reward}\n"
                f"terminated: {terminated}\n"
                f"truncated: {truncated}\n"
                f"info: {info}\n"
            )

            # Provide force feedback for teleop devices
            if teleop_interface.ft_feedback_interfaces:
                if isinstance(env.unwrapped, ManipulationEnv):
                    if (
                        not ft_feedback_use_contacts
                        and env.unwrapped._end_effector is not None
                    ):
                        end_effector = env.unwrapped._end_effector
                        try:
                            incoming_ft = (
                                end_effector.root_physx_view.get_link_incoming_joint_force()  # type: ignore
                            )[0].mean(dim=0)
                            ft_feedback: torch.Tensor = (
                                torch.tensor([0.33, 0.33, 0.33, 0.0, 0.0, 0.0])
                                * incoming_ft.cpu()
                            )
                            teleop_interface.set_ft_feedback(ft_feedback)
                        except Exception:
                            ft_feedback_use_contacts = True
                    if (
                        ft_feedback_use_contacts
                        and env.unwrapped._contacts_end_effector is not None
                    ):
                        contacts_end_effector = env.unwrapped._contacts_end_effector
                        contact_forces = (
                            contacts_end_effector.data.net_forces_w  # type: ignore
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
                        ft_feedback = (
                            torch.tensor([0.33, 0.33, 0.33, 0.0, 0.0, 0.0])
                            * contact_ft.cpu()
                        )
                        teleop_interface.set_ft_feedback(ft_feedback)

            ## Process reset request
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
    **kwargs,
):
    import torch
    from gymnasium.core import ActType, SupportsFloat, Wrapper, WrapperObsType

    from srb.utils import logging
    from srb.utils.math import quat_mul, rpy_to_quat

    ## Try disabling event for the command
    if hasattr(env.unwrapped, "event_manager"):
        events_to_remove: Sequence[Tuple[str, str]] = []
        for (
            category,
            event_names,
        ) in env.unwrapped.event_manager._mode_term_names.items():  # type: ignore
            for event_name in event_names:
                for recognized_cmd_key in recognized_cmd_keys:
                    if recognized_cmd_key in event_name:
                        events_to_remove.append(
                            (
                                category,
                                event_names.index(event_name),
                            )
                        )
                        break
        for category, event_id in reversed(events_to_remove):
            env.unwrapped.event_manager._mode_term_names[category].pop(  # type: ignore
                event_id
            )
            env.unwrapped.event_manager._mode_term_cfgs[category].pop(  # type: ignore
                event_id
            )

    class InjectTeleopWrapper(Wrapper):
        def __init__(self, env, *args, **kwargs):
            super().__init__(env, *args, **kwargs)

            ## Find the internal command attribute of the environment
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
                internal_cmd_attr_shape[0] == self.env.unwrapped.cfg.scene.num_envs  # type: ignore
            )
            self._is_cmd_additive = any(
                key in self._internal_cmd_attr_name for key in addititive_cmd_keys
            )

        def step(
            self,
            action: ActType,  # type: ignore
        ) -> Tuple[WrapperObsType, SupportsFloat, bool, bool, Mapping[str, Any]]:  # type: ignore
            ## Exit if the simulation is not running
            if not sim_app.is_running():
                exit()

            ## Get actions from the teleoperation interface and process them
            twist, event = teleop_interface.advance()
            if invert_controls:
                twist[:2] *= -1.0

            ## Update internal command
            cmd_len = getattr(env.unwrapped, self._internal_cmd_attr_name).shape[-1]  # type: ignore
            match cmd_len:
                case _ if cmd_len < 7:
                    cmd = torch.from_numpy(twist[:cmd_len]).to(
                        device=env.unwrapped.device,  # type: ignore
                        dtype=torch.float32,
                    )
                    setattr(
                        env.unwrapped,
                        self._internal_cmd_attr_name,  # type: ignore
                        (
                            cmd.repeat(
                                self.env.unwrapped.cfg.scene.num_envs,  # type: ignore
                                1,
                            )
                            if self._is_internal_cmd_attr_per_env
                            else cmd
                        )
                        if not self._is_cmd_additive
                        else (
                            getattr(env.unwrapped, self._internal_cmd_attr_name)  # type: ignore
                            + cmd
                        ),
                    )
                case 7:
                    if self._is_cmd_additive:
                        cmd = getattr(env.unwrapped, self._internal_cmd_attr_name)  # type: ignore
                        cmd[..., 0:3] += twist[0:3]
                        cmd[..., 3:7] = quat_mul(
                            torch.tensor(
                                rpy_to_quat(*twist[3:6]),
                                device=env.unwrapped.device,  # type: ignore
                                dtype=torch.float32,
                            ).repeat(
                                self.env.unwrapped.cfg.scene.num_envs,  # type: ignore
                                1,
                            ),
                            cmd[..., 3:7],
                        )
                        setattr(env.unwrapped, self._internal_cmd_attr_name, cmd)  # type: ignore
                    else:
                        cmd = torch.concat(
                            (
                                torch.from_numpy(twist).to(
                                    device=env.unwrapped.device,  # type: ignore
                                    dtype=torch.float32,
                                ),
                                torch.Tensor((-1.0 if event else 1.0,)).to(
                                    device=env.unwrapped.device,  # type: ignore
                                ),
                            )
                        )
                        setattr(
                            env.unwrapped,
                            self._internal_cmd_attr_name,  # type: ignore
                            (
                                cmd.repeat(
                                    self.env.unwrapped.cfg.scene.num_envs,  # type: ignore
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

            ## Step the environment
            observation, reward, terminated, truncated, info = super().step(action)
            logging.trace(
                f"action: {action}\n"
                f"observation: {observation}\n"
                f"reward: {reward}\n"
                f"terminated: {terminated}\n"
                f"truncated: {truncated}\n"
                f"info: {info}\n"
            )

            ## Process reset request
            global should_reset
            if should_reset:
                should_reset = False
                teleop_interface.reset()
                observation, info = env.reset()

            return (
                observation,  # type: ignore
                reward,
                terminated,
                truncated,
                info,
            )

    # Wrap the environment with the teleoperation interface
    env = InjectTeleopWrapper(env)  # type: ignore

    ## Evaluate the agent with the wrapped environment
    eval_agent(env=env, sim_app=sim_app, **kwargs)


def ros_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    **kwargs,
):
    import torch

    from srb.interfaces.interface.ros import RosInterface
    from srb.utils import logging

    # Disable truncation
    if hasattr(env.unwrapped.cfg, "truncate_episodes"):  # type: ignore
        env.unwrapped.cfg.truncate_episodes = False  # type: ignore

    ## Get or create ROS interface
    ros_interface = None
    ros_node = None
    if hasattr(env, "_srb_interfaces"):
        for iface in env._srb_interfaces:  # type: ignore
            if hasattr(iface, "_node"):
                ros_node = iface._node  # type: ignore
            if isinstance(iface, RosInterface):
                ros_interface = iface
                break
    should_update_interface = False
    if ros_interface is None:
        ros_interface = RosInterface(env=env, node=ros_node)
        env.unwrapped.cfg.extras = True  # type: ignore
        if hasattr(env, "_srb_interfaces"):
            env._srb_interfaces = (  # type: ignore
                *env._srb_interfaces,  # type: ignore
                ros_interface,
            )
        else:
            should_update_interface = True

    # Set up ROS interfaces for actions
    ros_interface.setup_action_sub()

    ## Run the environment with ROS interface
    with torch.inference_mode():
        while sim_app.is_running():
            action = ros_interface.action
            observation, reward, terminated, truncated, info = env.step(action)  # type: ignore
            logging.trace(
                f"action: {action}\n"
                f"observation: {observation}\n"
                f"reward: {reward}\n"
                f"terminated: {terminated}\n"
                f"truncated: {truncated}\n"
                f"info: {info}\n"
            )

            # Update interface
            if should_update_interface:
                ros_interface.update(
                    observation,  # type: ignore
                    reward,
                    terminated,
                    truncated,
                    info,
                )


def train_agent(algo: str, **kwargs):
    WORKFLOW: str = "train"

    match algo:
        case "dreamer":
            from srb.integrations.dreamer import main as dreamer

            dreamer.run(workflow=WORKFLOW, **kwargs)
        case _skrl if algo.startswith("skrl"):
            from srb.integrations.skrl import main as skrl

            skrl.run(workflow=WORKFLOW, **kwargs)
        case _sb3 if algo.startswith("sb3"):
            from srb.integrations.sb3 import main as sb3

            sb3.run(workflow=WORKFLOW, algo=algo.removeprefix("sb3_"), **kwargs)
        case _sbx if algo.startswith("sbx"):
            from srb.integrations.sbx import main as sbx

            sbx.run(workflow=WORKFLOW, algo=algo.removeprefix("sbx_"), **kwargs)


def eval_agent(algo: str, **kwargs):
    WORKFLOW: str = "eval"

    match algo:
        case "dreamer":
            from srb.integrations.dreamer import main as dreamer

            dreamer.run(workflow=WORKFLOW, **kwargs)
        case _skrl if algo.startswith("skrl"):
            from srb.integrations.skrl import main as skrl

            skrl.run(workflow=WORKFLOW, **kwargs)
        case _sb3 if algo.startswith("sb3"):
            from srb.integrations.sb3 import main as sb3

            sb3.run(workflow=WORKFLOW, algo=algo.removeprefix("sb3_"), **kwargs)
        case _sbx if algo.startswith("sbx"):
            from srb.integrations.sbx import main as sbx

            sbx.run(workflow=WORKFLOW, algo=algo.removeprefix("sbx_"), **kwargs)


### Sim-to-Real ###
def run_real_agent(
    real_agent_subcommand: Literal[
        "gen",
        "sim2real_gen",
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
    ],
    **kwargs,
):
    # Run the implementation
    def real_agent_impl(**kwargs):
        match real_agent_subcommand:
            case "gen" | "sim2real_gen":
                generate_real_agent(**kwargs)
            case _:
                run_real_agent_with_env(
                    real_agent_subcommand=real_agent_subcommand, **kwargs
                )

    real_agent_impl(**kwargs)


def run_real_agent_with_env(
    real_agent_subcommand: Literal[
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
    ],
    env_id: str,
    hardware: Sequence[str],
    logdir_path: str,
    forwarded_args: Sequence[str] = (),
    **kwargs,
):
    import gymnasium

    from srb.interfaces.sim_to_real import RealEnv
    from srb.interfaces.sim_to_real import env as srb_real_env  # noqa: F401
    from srb.interfaces.sim_to_real import hardware as srb_real_hw  # noqa: F401
    from srb.utils import logging
    from srb.utils.cfg import last_logdir, new_logdir
    from srb.utils.hydra.real import hydra_task_config
    from srb.utils.registry import get_srb_tasks

    # Ensure the environment is registered
    if env_id not in get_srb_tasks("srb_real"):
        env_name = env_id.rsplit("/", 1)[-1]
        logging.critical(
            f"The RealEnv for {env_name} is not registered. Generating it now..."
        )
        _generate_real_agent_subprocess(
            env_id=env_name,
            forwarded_args=(*forwarded_args, "--hardware", *hardware)
            if hardware
            else forwarded_args,
        )
        logging.critical(
            f"Generated the missing RealEnv for {env_name}. Please re-run your desired workflow."
        )
        exit(0)

    # Get the log directory based on the workflow
    workflow = kwargs.get("algo") or real_agent_subcommand
    logdir_root = Path(logdir_path).resolve()
    if model := kwargs.get("model"):
        model = Path(model).resolve()
        assert model.exists(), f"Model path does not exist: {model}"
        logdir = model.parent
        while not (
            logdir.parent.name == workflow
            and (logdir.parent.parent.name == env_id.rsplit("/", 1)[-1])
        ):
            _new_parent = logdir.parent
            if logdir == _new_parent:
                logdir = new_logdir(
                    env_id=env_id,
                    workflow=workflow,
                    root=logdir_root,
                    namespace="srb_real",
                )
                model_symlink_path = logdir.joinpath(model.name)
                model_symlink_path.parent.mkdir(parents=True, exist_ok=True)
                os.symlink(model, model_symlink_path)
                model = model_symlink_path
                break
            logdir = _new_parent
        kwargs["model"] = model
    elif (real_agent_subcommand == "train" and kwargs["continue_training"]) or (
        real_agent_subcommand in ("eval", "teleop") and kwargs["algo"]
    ):
        logdir = last_logdir(
            env_id=env_id, workflow=workflow, root=logdir_root, namespace="srb_real"
        )
    else:
        logdir = new_logdir(
            env_id=env_id, workflow=workflow, root=logdir_root, namespace="srb_real"
        )

    # Update Hydra output directory
    if not any(arg.startswith("hydra.run.dir=") for arg in forwarded_args):
        sys.argv.extend([f"hydra.run.dir={logdir.as_posix()}"])
    maybe_config_path = Path(logdir).joinpath(".hydra", "config.yaml").resolve()

    @hydra_task_config(
        task_name=env_id,
        agent_cfg_entry_point=f"{kwargs['algo']}_cfg" if kwargs.get("algo") else None,
        config_path=maybe_config_path.as_posix()
        if maybe_config_path.exists()
        else None,
    )
    def hydra_main(agent_cfg: Dict[str, Any] | None = None):
        # Create the environment and initialize it
        env: RealEnv = gymnasium.make(id=env_id, hardware=hardware)  # type: ignore
        env.reset()

        # Run the implementation
        def agent_impl(**kwargs):
            kwargs.update(
                {
                    "env_id": env_id,
                    "agent_cfg": agent_cfg,
                    "env_cfg": None,
                }
            )

            match real_agent_subcommand:
                case "zero":
                    zero_agent(**kwargs)
                case "rand":
                    random_agent(**kwargs)
                case "teleop":
                    teleop_agent(**kwargs)
                case "ros":
                    ros_agent(**kwargs)
                case "train":
                    train_agent(**kwargs)
                case "eval":
                    eval_agent(**kwargs)
                case "collect":
                    raise NotImplementedError()

        class FakeSimulationApp:
            def is_running(self):
                return True

        agent_impl(
            env=env,
            sim_app=FakeSimulationApp(),
            logdir=logdir,
            headless=False,
            **kwargs,
        )

        # Close the environment
        env.close()

    hydra_main()  # type: ignore


def generate_real_agent(
    env_id: str,
    hardware: Sequence[str] = (),
    forwarded_args: Sequence[str] = (),
    **kwargs,
):
    if env_id.rsplit("/", 1)[-1] == "ALL":
        _generate_real_agent_subprocess(
            env_id="ALL",
            forwarded_args=(*forwarded_args, "--hardware", *hardware)
            if hardware
            else forwarded_args,
        )
        return

    from srb.core.app import AppLauncher

    # Launch Isaac Sim
    enable_cameras = env_id.endswith("_visual")
    launcher = AppLauncher(
        headless=True,
        enable_cameras=env_id.endswith("_visual"),
        experience=SRB_APPS_DIR.joinpath(
            f"srb.headless.{'rendering.' if enable_cameras else ''}kit"
        ),
    )

    # Update the offline registry cache
    update_offline_srb_cache()

    from omni.physx import acquire_physx_interface

    from srb.utils.hydra.sim import hydra_task_config

    # Post-launch configuration
    acquire_physx_interface().overwrite_gpu_setting(1)

    # Disable Hydra output
    if not any(arg.startswith("hydra.output_subdir=") for arg in forwarded_args):
        sys.argv.extend(["hydra.output_subdir=null"])

    @hydra_task_config(
        task_name=env_id,
        agent_cfg_entry_point=None,
    )
    def hydra_main(env_cfg: Dict[str, Any], agent_cfg: Dict[str, Any] | None = None):
        import gymnasium

        from srb.interfaces.sim_to_real import env as srb_real_env
        from srb.interfaces.sim_to_real.core.generator import RealEnvGenerator

        # Create the environment and initialize it
        env_spec = gymnasium.spec(env_id)
        env = gymnasium.make(id=env_id, cfg=env_cfg, render_mode=None)
        env.reset()

        # Generate RealEnv classes
        RealEnvGenerator().generate_offline(
            env=env,  # type: ignore
            env_spec=env_spec,
            output=Path(srb_real_env.__file__).parent.joinpath(
                f"{env_id.removeprefix('srb/')}.py"
            ),
            hardware=hardware,
        )

        # Close the environment
        env.close()

    hydra_main()  # type: ignore

    # Shutdown Isaac Sim
    launcher.app.close()


def _generate_real_agent_subprocess(env_id: str, forwarded_args: Sequence[str] = ()):
    import subprocess

    from srb.utils import logging
    from srb.utils.isaacsim import get_isaacsim_python

    # Get all registered environments
    if env_id == "ALL":
        env_list = read_offline_srb_env_cache()
        if not env_list:
            logging.warning(
                "No environments found in cache. Please run an environment first to populate the cache."
            )
            return

        # Filter out templates
        env_list = [
            env for env in env_list if not env.rsplit("/", 1)[-1].startswith("_")
        ]
        logging.info(
            f"Generating sim-to-real setup for {len(env_list)} environments..."
        )
        env_list.sort()
    else:
        env_list = (env_id,)

    successful_envs = []
    failed_envs = []
    for i, env in enumerate(env_list, 1):
        logging.info(f"[{i}/{len(env_list)}] Processing environment: {env}")
        cmd = [
            get_isaacsim_python(),
            "-m",
            "srb",
            "real_agent",
            "gen",
            "--env",
            env,
            *forwarded_args,
        ]
        try:
            _result = subprocess.run(cmd, check=True, timeout=300)
            successful_envs.append(env)
            logging.info(f"✓ Successfully generated sim-to-real setup for {env}")
        except subprocess.TimeoutExpired:
            failed_envs.append((env, "Timeout after 5 minutes"))
            logging.error(f"✗ Timeout generating sim-to-real setup for {env}")
        except subprocess.CalledProcessError as e:
            failed_envs.append((env, f"Process failed with code {e.returncode}"))
            logging.error(
                f"✗ Failed to generate sim-to-real setup for {env}: {e.stderr}"
            )
        except Exception as e:
            failed_envs.append((env, str(e)))
            logging.error(f"✗ Exception while processing {env}: {e}")

    # Summary report
    logging.info("\n=== Generation Summary ===")
    logging.info(
        f"Successfully processed: {len(successful_envs)}/{len(env_list)} environments"
    )
    if successful_envs:
        logging.info(f"Successful environments: {', '.join(successful_envs)}")
    if failed_envs:
        logging.warning("Failed environments:")
        for env, reason in failed_envs:
            logging.warning(f"  - {env}: {reason}")


### List ###
def list_registered(
    category: str | Sequence[str], show_hidden: bool, forwarded_args: Sequence[str] = ()
):
    from srb.core.app import AppLauncher

    if not find_spec("rich"):
        raise ImportError('The "rich" package is required to list registered entities')

    # Launch Isaac Sim
    launcher = AppLauncher(
        headless=True, experience=SRB_APPS_DIR.joinpath("srb.barebones.kit")
    )

    # Update the offline registry cache
    update_offline_srb_cache()

    import importlib
    import inspect
    from dataclasses import _MISSING_TYPE
    from os import path

    from rich import print
    from rich.table import Table

    # Standardize category
    category = (  # type: ignore
        {EntityToList.from_str(category)}
        if isinstance(category, str)
        else set(map(EntityToList.from_str, category))
    )
    if EntityToList.ALL in category:
        category = {  # type: ignore
            EntityToList.ACTION,
            EntityToList.ASSET,
            EntityToList.ENV,
        }
    if EntityToList.ASSET in category:
        category.remove(EntityToList.ASSET)  # type: ignore
        category.add(EntityToList.SCENERY)  # type: ignore
        category.add(EntityToList.OBJECT)  # type: ignore
        category.add(EntityToList.ROBOT)  # type: ignore

    if EntityToList.ENV in category:
        from srb import tasks as srb_tasks

    # Print table for assets
    if (
        EntityToList.SCENERY in category
        or EntityToList.OBJECT in category
        or EntityToList.ROBOT in category
    ):
        from srb.core.asset import AssetType

        table = Table(title="Assets")
        table.add_column("#", justify="right", style="cyan", no_wrap=True)
        table.add_column("Name", justify="left", style="blue", no_wrap=True)
        table.add_column("Type", justify="left", style="magenta", no_wrap=True)
        table.add_column("Subtype", justify="left", style="red", no_wrap=True)
        table.add_column("Parent Class", justify="left", style="green", no_wrap=True)
        table.add_column("Asset Config", justify="left", style="yellow")
        table.add_column("Path", justify="left", style="white")
        i = 0
        if EntityToList.SCENERY in category:
            from srb.assets import scenery as srb_sceneries
            from srb.core.asset import SceneryRegistry

            asset_type = AssetType.SCENERY
            for asset_subtype, asset_classes in SceneryRegistry.items():
                for j, asset_class in enumerate(asset_classes):
                    i += 1
                    asset_name = asset_class.name()
                    parent_class = asset_class.__bases__[0]
                    asset_cfg_class = asset_class().asset_cfg.__class__  # type: ignore
                    asset_module_path = Path(
                        inspect.getabsfile(
                            importlib.import_module(asset_class.__module__)
                        )
                    )
                    try:
                        asset_module_relpath = asset_module_path.relative_to(
                            Path(inspect.getabsfile(srb_sceneries)).parent
                        )
                    except ValueError:
                        asset_module_relpath = path.join("EXT", asset_module_path.name)
                    table.add_row(
                        str(i),
                        f"[link=vscode://file/{inspect.getabsfile(asset_class)}:{inspect.getsourcelines(asset_class)[1]}]{asset_name}[/link]",
                        str(asset_type),
                        str(asset_subtype),
                        f"[link=vscode://file/{inspect.getabsfile(parent_class)}:{inspect.getsourcelines(parent_class)[1]}]{parent_class.__name__}[/link]",
                        f"[link=vscode://file/{inspect.getabsfile(asset_cfg_class)}:{inspect.getsourcelines(asset_cfg_class)[1]}]{asset_cfg_class.__name__}[/link]",
                        f"[link=vscode://file/{asset_module_path}]{asset_module_relpath}[/link]",
                        end_section=(j + 1) == len(asset_classes),
                    )
        if EntityToList.OBJECT in category:
            from srb.assets import object as srb_objects
            from srb.core.asset import ObjectRegistry

            asset_type = AssetType.OBJECT
            for asset_subtype, asset_classes in ObjectRegistry.items():
                for j, asset_class in enumerate(asset_classes):
                    i += 1
                    asset_name = asset_class.name()
                    parent_class = asset_class.__bases__[0]
                    asset_cfg_class = asset_class().asset_cfg.__class__  # type: ignore
                    asset_module_path = Path(
                        inspect.getabsfile(
                            importlib.import_module(asset_class.__module__)
                        )
                    )
                    try:
                        asset_module_relpath = asset_module_path.relative_to(
                            Path(inspect.getabsfile(srb_objects)).parent
                        )
                    except ValueError:
                        asset_module_relpath = path.join("EXT", asset_module_path.name)
                    table.add_row(
                        str(i),
                        f"[link=vscode://file/{inspect.getabsfile(asset_class)}:{inspect.getsourcelines(asset_class)[1]}]{asset_name}[/link]",
                        str(asset_type),
                        str(asset_subtype),
                        f"[link=vscode://file/{inspect.getabsfile(parent_class)}:{inspect.getsourcelines(parent_class)[1]}]{parent_class.__name__}[/link]",
                        f"[link=vscode://file/{inspect.getabsfile(asset_cfg_class)}:{inspect.getsourcelines(asset_cfg_class)[1]}]{asset_cfg_class.__name__}[/link]",
                        f"[link=vscode://file/{asset_module_path}]{asset_module_relpath}[/link]",
                        end_section=(j + 1) == len(asset_classes),
                    )
        if EntityToList.ROBOT in category:
            from srb.assets import robot as srb_robots
            from srb.core.asset import RobotRegistry

            asset_type = AssetType.ROBOT
            for asset_subtype, asset_classes in RobotRegistry.items():
                for j, asset_class in enumerate(asset_classes):
                    i += 1
                    asset_name = asset_class.name()
                    parent_class = asset_class.__bases__[0]
                    asset_cfg_class = asset_class().asset_cfg.__class__  # type: ignore
                    asset_module_path = Path(
                        inspect.getabsfile(
                            importlib.import_module(asset_class.__module__)
                        )
                    )
                    try:
                        asset_module_relpath = asset_module_path.relative_to(
                            Path(inspect.getabsfile(srb_robots)).parent
                        )
                    except ValueError:
                        asset_module_relpath = path.join("EXT", asset_module_path.name)
                    table.add_row(
                        str(i),
                        f"[link=vscode://file/{inspect.getabsfile(asset_class)}:{inspect.getsourcelines(asset_class)[1]}]{asset_name}[/link]",
                        str(asset_type),
                        str(asset_subtype),
                        f"[link=vscode://file/{inspect.getabsfile(parent_class)}:{inspect.getsourcelines(parent_class)[1]}]{parent_class.__name__}[/link]",
                        f"[link=vscode://file/{inspect.getabsfile(asset_cfg_class)}:{inspect.getsourcelines(asset_cfg_class)[1]}]{asset_cfg_class.__name__}[/link]"
                        if asset_cfg_class is not _MISSING_TYPE
                        else "<DYNAMIC>",
                        f"[link=vscode://file/{asset_module_path}]{asset_module_relpath}[/link]",
                        end_section=(j + 1) == len(asset_classes),
                    )
        print(table)

    # Print table for action groups
    if EntityToList.ACTION in category:
        from srb.core.action import ActionGroupRegistry
        from srb.core.action import group as srb_action_groups

        table = Table(title="Action Groups")
        table.add_column("#", justify="right", style="cyan", no_wrap=True)
        table.add_column("Name", justify="left", style="blue", no_wrap=True)
        table.add_column("Path", justify="left", style="white")

        for i, action_group_class in enumerate(ActionGroupRegistry.registry, 1):
            action_group_name = action_group_class.name()
            action_group_path = Path(
                inspect.getabsfile(
                    importlib.import_module(action_group_class.__module__)
                )
            )
            try:
                action_group_relpath = action_group_path.relative_to(
                    Path(inspect.getabsfile(srb_action_groups)).parent
                )
            except ValueError:
                action_group_relpath = path.join("EXT", action_group_path.name)
            table.add_row(
                str(i),
                f"[link=vscode://file/{inspect.getabsfile(action_group_class)}:{inspect.getsourcelines(action_group_class)[1]}]{action_group_name}[/link]",
                f"[link=vscode://file/{action_group_path}]{action_group_relpath}[/link]",
            )
        print(table)

    # Print table for hardware interfaces
    if EntityToList.ACTION in category:
        from srb.interfaces.sim_to_real import hardware as srb_hardware_interfaces
        from srb.interfaces.sim_to_real.core.hardware import HardwareInterfaceRegistry

        table = Table(title="Hardware Interfaces")
        table.add_column("#", justify="right", style="cyan", no_wrap=True)
        table.add_column("Name", justify="left", style="blue", no_wrap=True)
        table.add_column("Path", justify="left", style="white")

        for i, hardware_interface_class in enumerate(
            HardwareInterfaceRegistry.registry, 1
        ):
            hardware_interface_name = hardware_interface_class.class_name()
            hardware_interface_path = Path(
                inspect.getabsfile(
                    importlib.import_module(hardware_interface_class.__module__)
                )
            )
            try:
                hardware_interface_relpath = hardware_interface_path.relative_to(
                    Path(inspect.getabsfile(srb_hardware_interfaces)).parent
                )
            except ValueError:
                hardware_interface_relpath = path.join(
                    "EXT", hardware_interface_path.name
                )
            table.add_row(
                str(i),
                f"[link=vscode://file/{inspect.getabsfile(hardware_interface_class)}:{inspect.getsourcelines(hardware_interface_class)[1]}]{hardware_interface_name}[/link]",
                f"[link=vscode://file/{hardware_interface_path}]{hardware_interface_relpath}[/link]",
            )
        print(table)

    # Print table for environments
    if EntityToList.ENV in category:
        import gymnasium

        from srb.utils.registry import get_srb_tasks

        table = Table(title="Environments")
        table.add_column("#", justify="right", style="cyan", no_wrap=True)
        table.add_column("ID", justify="left", style="blue", no_wrap=True)
        table.add_column("Entrypoint", justify="left", style="green")
        table.add_column("Config", justify="left", style="yellow")
        table.add_column("Path", justify="left", style="white")
        i = 0
        for task_id in get_srb_tasks():
            if not show_hidden and task_id.endswith("_visual"):
                continue
            i += 1
            env = gymnasium.registry[task_id]
            entrypoint_str = env.entry_point
            entrypoint_module, entrypoint_class = str(entrypoint_str).split(":")
            env_module_path = Path(
                inspect.getabsfile(
                    importlib.import_module(entrypoint_module.rsplit(".", 1)[0])
                )
            )
            try:
                env_module_relpath = env_module_path.parent.relative_to(
                    Path(inspect.getabsfile(srb_tasks)).parent
                )
            except ValueError:
                env_module_relpath = path.join("EXT", env_module_path.name)
            entrypoint_module = importlib.import_module(entrypoint_module)
            entrypoint_class = getattr(entrypoint_module, entrypoint_class)
            entrypoint_parent = entrypoint_class.__bases__[0]
            cfg_class = env.kwargs["task_cfg"]
            cfg_parent = cfg_class.__bases__[0]
            table.add_row(
                str(i),
                task_id.rsplit("/", 1)[-1]
                + (" <template>" if task_id.rsplit("/", 1)[-1].startswith("_") else ""),
                f"[link=vscode://file/{inspect.getabsfile(entrypoint_class)}:{inspect.getsourcelines(entrypoint_class)[1]}]{entrypoint_class.__name__}[/link]([red][link=vscode://file/{inspect.getabsfile(entrypoint_parent)}:{inspect.getsourcelines(entrypoint_parent)[1]}]{entrypoint_parent.__name__}[/link][/red])",
                f"[link=vscode://file/{inspect.getabsfile(cfg_class)}:{inspect.getsourcelines(cfg_class)[1]}]{cfg_class.__name__}[/link]([magenta][link=vscode://file/{inspect.getabsfile(cfg_parent)}:{inspect.getsourcelines(cfg_parent)[1]}]{cfg_parent.__name__}[/link][/magenta])",
                f"[link=vscode://file/{env_module_path}]{env_module_relpath}[/link]",
            )
        print(table)

    # Shutdown Isaac Sim
    launcher.app.close()


### GUI ###
def launch_gui(forwarded_args: Sequence[str] = ()):
    import string
    import subprocess

    from srb.utils import logging

    cmd = (
        "cargo",
        "run",
        "--manifest-path",
        SRB_DIR.joinpath("Cargo.toml").as_posix(),
        "--package",
        "srb_gui",
        "--bin",
        "gui",
        *forwarded_args,
    )
    logging.info(
        "Launching GUI with the following command: "
        + " ".join(
            (f'"{arg}"' if any(c in string.whitespace for c in arg) else arg)
            for arg in cmd
        )
    )

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        logging.critical("Launching GUI failed due to the exception above")
        exit(e.returncode)


### REPL ###
def enter_repl(
    headless: bool, hide_ui: bool, forwarded_args: Sequence[str] = (), **kwargs
):
    from srb.core.app import AppLauncher

    if not find_spec("ptpython"):
        raise ImportError('The "ptpython" package is required to enter REPL')

    # Preprocess kwargs
    kwargs["enable_cameras"] = True
    kwargs["experience"] = SRB_APPS_DIR.joinpath(
        f"srb.{'headless.' if headless else ''}rendering.kit"
    )

    # Launch Isaac Sim
    launcher = AppLauncher(headless=headless, **kwargs)

    # Update the offline registry cache
    update_offline_srb_cache()

    import ptpython

    import srb  # noqa: F401
    from srb.utils import logging  # noqa: F401
    from srb.utils.isaacsim import hide_isaacsim_ui

    # Post-launch configuration
    if hide_ui:
        hide_isaacsim_ui()

    # Enter REPL
    ptpython.repl.embed(globals(), locals(), title="Space Robotics Bench")

    # Shutdown Isaac Sim
    launcher.app.close()


### Docs ###
def serve_docs(forwarded_args: Sequence[str] = ()):
    import string
    import subprocess

    from srb.utils import logging

    if not shutil.which("mdbook"):
        raise FileNotFoundError('The "mdbook" tool is required to serve the docs')

    cmd = (
        "mdbook",
        "serve",
        SRB_DIR.joinpath("docs").as_posix(),
        "--open",
        *forwarded_args,
    )
    logging.info(
        "Serving the docs with the following command: "
        + " ".join(
            (f'"{arg}"' if any(c in string.whitespace for c in arg) else arg)
            for arg in cmd
        )
    )

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        logging.critical("Serving the docs failed due to the exception above")
        exit(e.returncode)


### Test ###
def run_tests(language: Sequence[str], forwarded_args: Sequence[str] = ()):
    import string
    import subprocess

    from srb.utils import logging
    from srb.utils.isaacsim import get_isaacsim_python

    if not find_spec("pytest"):
        raise ImportError('The "pytest" package is required to run tests')

    language = list(set(map(Lang.from_str, language)))  # type: ignore

    for lang in language:
        match lang:
            case Lang.PYTHON:
                cmd = (
                    get_isaacsim_python(),
                    "-m",
                    "pytest",
                    SRB_DIR.as_posix(),
                    *forwarded_args,
                )
            case Lang.RUST:
                cmd = (
                    "cargo",
                    "test",
                    "--manifest-path",
                    SRB_DIR.joinpath("Cargo.toml").as_posix(),
                    *forwarded_args,
                )
        logging.info(
            f"Running {str(lang)} tests with the following command: "
            + " ".join(
                (f'"{arg}"' if any(c in string.whitespace for c in arg) else arg)
                for arg in cmd
            )
        )

        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            logging.critical(
                f"Running {str(lang)} tests failed due to the exception above"
            )
            exit(e.returncode)


### Misc ###
def __wrap_env_in_performance_test(
    env: "AnyEnv", sim_app: "SimulationApp", perf_output: str, perf_duration: float
):
    import sys
    import time
    from collections import deque

    import torch
    from gymnasium.core import ActType, Env, ObsType, Wrapper

    class PerformanceTestWrapper(Wrapper):
        def __init__(
            self,
            env: Env[ObsType, ActType],
            output: str,
            duration: float,
            min_report_interval_fraction: float = 0.1,
            max_buffer_size: int = 1000000,
        ):
            super().__init__(env)
            self.__perf_output = output
            self.__perf_duration = duration
            self.__perf_min_report_interval = duration * min_report_interval_fraction

            _env: "AnyEnv" = env.unwrapped  # type: ignore
            self.__num_envs = _env.num_envs
            self.__agent_rate = _env.cfg.agent_rate

            self._perf_num_steps = 0
            self._perf_num_episodes = 0
            self._perf_total_time = 0.0
            self._perf_step_timings = deque(maxlen=max_buffer_size)
            self._perf_start_time = time.perf_counter()
            self._perf_last_report_time = self._perf_start_time

            if self.__perf_output != "STDOUT":
                parent_dir = os.path.dirname(self.__perf_output)
                if not os.path.exists(parent_dir):
                    os.makedirs(parent_dir, exist_ok=True)

        def step(self, action):
            # Step timing
            t_start = time.perf_counter()
            obs, reward, terminated, truncated, info = super().step(action)
            t_end = time.perf_counter()
            step_timing = t_end - t_start
            self._perf_step_timings.append(step_timing)
            self._perf_num_steps += self.__num_envs
            self._perf_total_time += step_timing

            # Episode end detection
            done: torch.Tensor = terminated | truncated  # type: ignore
            num_done = torch.sum(done).item()
            self._perf_num_episodes += num_done
            episode_ended = num_done > 0.5 * self.__num_envs

            if episode_ended or (
                t_end - self._perf_last_report_time > self.__perf_min_report_interval
            ):
                self.__perf_report(episode_end=episode_ended)
                self._perf_last_report_time = t_end

            # Check for duration limit
            if self._perf_total_time >= self.__perf_duration:
                self.__perf_report(final=True)
                print("The performance test has finished (duration limit reached).")
                env.close()
                sim_app.close()
                sys.exit(0)

            return obs, reward, terminated, truncated, info

        def __perf_report(self, episode_end: bool = False, final: bool = False):
            steps = self._perf_num_steps
            episodes = self._perf_num_episodes
            total_time = self._perf_total_time
            timings = torch.tensor(list(self._perf_step_timings))
            steps_per_sec = steps / total_time if total_time > 0 else 0
            mean_step_time = torch.mean(timings).item() if timings.numel() > 0 else 0
            median_step_time = (
                torch.median(timings).item() if timings.numel() > 0 else 0
            )
            min_step_time = torch.min(timings).item() if timings.numel() > 0 else 0
            max_step_time = torch.max(timings).item() if timings.numel() > 0 else 0

            def torch_percentiles(t, percentiles):
                if t.numel() == 0:
                    return [0 for _ in percentiles]
                t_sorted, _ = torch.sort(t)
                n = t.numel()
                result = []
                for p in percentiles:
                    k = (n - 1) * (p / 100)
                    f = torch.floor(torch.tensor(k)).long()
                    c = torch.ceil(torch.tensor(k)).long()
                    if f == c:
                        val = t_sorted[f]
                    else:
                        d0 = t_sorted[f] * (c.float() - k)
                        d1 = t_sorted[c] * (k - f.float())
                        val = d0 + d1
                    result.append(val.item())
                return result

            step_time_percentiles = torch_percentiles(timings, [10, 20, 80, 90])
            if not hasattr(self, "_perf_episode_lengths"):
                self._perf_episode_lengths = []
            if episode_end and self._perf_num_episodes > 0:
                last_episode_len = self._perf_num_steps / self._perf_num_episodes
                self._perf_episode_lengths.append(last_episode_len)
            episode_lengths = torch.tensor(self._perf_episode_lengths)
            mean_episode_len = (
                torch.mean(episode_lengths).item() if episode_lengths.numel() > 0 else 0
            )
            median_episode_len = (
                torch.median(episode_lengths).item()
                if episode_lengths.numel() > 0
                else 0
            )
            min_episode_len = (
                torch.min(episode_lengths).item() if episode_lengths.numel() > 0 else 0
            )
            max_episode_len = (
                torch.max(episode_lengths).item() if episode_lengths.numel() > 0 else 0
            )
            episode_len_percentiles = torch_percentiles(
                episode_lengths, [10, 20, 80, 90]
            )
            report = (
                f"\nPerformance Report{' (final)' if final else ''}:\n"
                f"    Elapsed time (s)       : {total_time:.2f}\n"
                f"    Total steps (#)        : {steps}\n"
                f"    Total episodes (#)     : {episodes}\n"
                f"    Steps per second (#/s) : {steps_per_sec:.2f}\n"
                f"    Step time (ms)         : min={min_step_time * 1000:.3f}, p10={step_time_percentiles[0] * 1000:.3f}, p20={step_time_percentiles[1] * 1000:.3f}, mean={mean_step_time * 1000:.3f}, median={median_step_time * 1000:.3f}, p80={step_time_percentiles[2] * 1000:.3f}, p90={step_time_percentiles[3] * 1000:.3f}, max={max_step_time * 1000:.3f}\n"
                f"    Episode length (steps) : min={min_episode_len:.1f}, p10={episode_len_percentiles[0]:.1f}, p20={episode_len_percentiles[1]:.1f}, mean={mean_episode_len:.1f}, median={median_episode_len:.1f}, p80={episode_len_percentiles[2]:.1f}, p90={episode_len_percentiles[3]:.1f}, max={max_episode_len:.1f}\n"
                f"    Episode length (s)     : min={self.__agent_rate * min_episode_len:.1f}, p10={self.__agent_rate * episode_len_percentiles[0]:.1f}, p20={self.__agent_rate * episode_len_percentiles[1]:.1f}, mean={self.__agent_rate * mean_episode_len:.1f}, median={self.__agent_rate * median_episode_len:.1f}, p80={self.__agent_rate * episode_len_percentiles[2]:.1f}, p90={self.__agent_rate * episode_len_percentiles[3]:.1f}, max={self.__agent_rate * max_episode_len:.1f}\n"
            )
            if self.__perf_output == "STDOUT":
                print(report)
            else:
                with open(self.__perf_output, "a") as f:
                    f.write(report + "\n")

    return PerformanceTestWrapper(env, output=perf_output, duration=perf_duration)


### CLI ###
def parse_cli_args() -> argparse.Namespace:
    env_choices = read_offline_srb_env_cache()
    interface_choices = sorted(map(str, InterfaceType))
    teleop_device_choices = sorted(map(str, TeleopDeviceType))
    algo_choices = sorted(map(str, SupportedAlgo))
    hardware_choices = read_offline_srb_hardware_interface_cache()

    parser = argparse.ArgumentParser(
        description="Space Robotics Bench",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    subparsers = parser.add_subparsers(
        title="Subcommands",
        dest="subcommand",
        required=True,
    )

    ## Agent subcommand
    agent_parser = subparsers.add_parser(
        "agent",
        help="Agent subcommands",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    agent_subparsers = agent_parser.add_subparsers(
        title="Agent subcommands",
        dest="agent_subcommand",
        required=True,
    )
    zero_agent_parser = agent_subparsers.add_parser(
        "zero",
        help="Agent with zero-valued actions",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    rand_agent_parser = agent_subparsers.add_parser(
        "rand",
        help="Agent with random actions",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    teleop_agent_parser = agent_subparsers.add_parser(
        "teleop",
        help="Teleoperate agent",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ros_agent_parser = agent_subparsers.add_parser(
        "ros",
        help="Agent with actions from ROS 2 | Space ROS",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    train_agent_parser = agent_subparsers.add_parser(
        "train",
        help="Train agent",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    eval_agent_parser = agent_subparsers.add_parser(
        "eval",
        help="Evaluate agent",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    collect_agent_parser = agent_subparsers.add_parser(
        "collect",
        help="Collect demonstrations with agent",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    learn_agent_parser = agent_subparsers.add_parser(
        "learn",
        help="Learn from demonstrations",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    agent_parsers_with_env = (
        zero_agent_parser,
        rand_agent_parser,
        teleop_agent_parser,
        ros_agent_parser,
        train_agent_parser,
        eval_agent_parser,
        collect_agent_parser,
        learn_agent_parser,
    )

    ## Sim-to-Real subcommand
    real_agent_parser = subparsers.add_parser(
        "real_agent",
        help="Sim-to-real subcommands",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    real_agent_subparsers = real_agent_parser.add_subparsers(
        title="Sim-to-real subcommands",
        dest="real_agent_subcommand",
        required=True,
    )
    real_env_gen_parsers = []
    for i, alias in enumerate(("gen", "sim2real_gen")):
        real_env_gen_parser = real_agent_subparsers.add_parser(
            alias,
            help=("Alias: gen" if i > 0 else "Generate sim-to-real setup"),
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )
        real_env_gen_parsers.append(real_env_gen_parser)
    zero_real_agent_parser = real_agent_subparsers.add_parser(
        "zero",
        help="Real agent with zero-valued actions",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    rand_real_agent_parser = real_agent_subparsers.add_parser(
        "rand",
        help="Real agent with random actions",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    teleop_real_agent_parser = real_agent_subparsers.add_parser(
        "teleop",
        help="Teleoperate real agent",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ros_real_agent_parser = real_agent_subparsers.add_parser(
        "ros",
        help="Real agent with actions from ROS 2 | Space ROS",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    train_real_agent_parser = real_agent_subparsers.add_parser(
        "train",
        help="Train real agent",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    eval_real_agent_parser = real_agent_subparsers.add_parser(
        "eval",
        help="Evaluate real agent",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    collect_real_agent_parser = real_agent_subparsers.add_parser(
        "collect",
        help="Collect demonstrations with real agent",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    real_agent_parsers_with_env = (
        zero_real_agent_parser,
        rand_real_agent_parser,
        teleop_real_agent_parser,
        ros_real_agent_parser,
        train_real_agent_parser,
        eval_real_agent_parser,
        collect_real_agent_parser,
    )

    ## List subcommand
    for i, alias in enumerate(("list", "ls")):
        list_parser = subparsers.add_parser(
            alias,
            help=("Alias: list" if i > 0 else "List registered assets and environments")
            + ("" if find_spec("rich") else ' (MISSING: "rich" Python package)'),
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )
        list_parser.add_argument(
            "category",
            help="Filter of categories to list",
            nargs="*",
            type=str,
            choices=sorted(map(str, EntityToList)),
            default=str(EntityToList.ALL),
        )
        list_parser.add_argument(
            "-a",
            "--show_hidden",
            help='Show hidden entities ("*_visual" environments are hidden by default)',
            action="store_true",
            default=False,
        )

    ## GUI subcommand
    _gui_parser = subparsers.add_parser(
        "gui",
        help="Launch GUI",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    ## REPL subcommand
    repl_parsers = []
    for i, alias in enumerate(("repl", "python")):
        repl_parser = subparsers.add_parser(
            alias,
            help=("Alias: repl" if i > 0 else "Enter Python REPL")
            + (
                "" if find_spec("ptpython") else ' (MISSING: "ptpython" Python package)'
            ),
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )
        repl_parsers.append(repl_parser)

    ## Docs subcommand
    _docs_parser = subparsers.add_parser(
        "docs",
        help="Serve documentation"
        + ("" if shutil.which("mdbook") else ' (MISSING: "mdbook" tool)'),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    ## Test subcommand
    test_parser = subparsers.add_parser(
        "test",
        help="Run tests"
        + ("" if find_spec("pytest") else ' (MISSING: "pytest" Python package)'),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    test_parser.add_argument(
        "-l",
        "--language",
        "--lang",
        help="Languages to test",
        nargs="+",
        type=str,
        choices=set(map(str, Lang)),
        default=[str(Lang.PYTHON)],
    )

    ## Simulation launcher args
    for _parser in (
        *agent_parsers_with_env,
        *repl_parsers,
    ):
        launcher_group = _parser.add_argument_group("Launcher")
        launcher_group.add_argument(
            "--headless",
            help="Run the simulation without display output",
            action="store_true",
            default=False,
        )
        launcher_group.add_argument(
            "--hide_ui",
            help="Disable most of the Isaac Sim UI and set it to fullscreen",
            action="store_true",
            default=False,
        )
        launcher_group.add_argument(
            "--livestream",
            help="Force enable livestreaming. Mapping corresponds to that for the `LIVESTREAM` environment variable (0: Disabled, 1: Native, 2: WebRTC)",
            type=int,
            choices={0, 1, 2},
            default=-1,
        )
        launcher_group.add_argument(
            "--rendering_mode",
            help=(
                f"Configure the rendering parameters with one of the predefined presets in the {SRB_APPS_DIR.joinpath('rendering_modes').resolve().as_posix()} directory. "
            ),
            type=str,
            action=ExplicitAction,
            choices={"performance", "balanced", "quality", "xr"},
        )
        launcher_group.add_argument(
            "--xr",
            help="Enable XR mode for VR/AR applications.",
            action="store_true",
            default=False,
        )
        launcher_group.add_argument(
            "--kit_args",
            help="CLI args for the Omniverse Kit as a string separated by a space delimiter (e.g., '--ext-folder=/path/to/ext1 --ext-folder=/path/to/ext2')",
            type=str,
            default="",
        )

    ## Environment args
    for _parser in (
        *agent_parsers_with_env,
        *real_agent_parsers_with_env,
        *real_env_gen_parsers,
    ):
        environment_group = _parser.add_argument_group("Environment")
        environment_group.add_argument(
            "-e",
            "--env",
            "--task",
            dest="env_id",
            help="Name of the environment to select",
            type=str,
            action=AutoRealNamespaceTaskAction
            if _parser in real_agent_parsers_with_env
            else AutoNamespaceTaskAction,
            choices=(
                env_choices
                if _parser not in real_env_gen_parsers
                else (("ALL", *env_choices) if env_choices else ())
            ),
            required=True,
        )

    ## Environment args (extras)
    for _parser in agent_parsers_with_env:
        interfaces_group = _parser.add_argument_group("Interface")
        interfaces_group.add_argument(
            "--interface",
            help="Sequence of interfaces to enable",
            type=str,
            nargs="*",
            choices=interface_choices,
            default=[],
        )

        video_recording_group = _parser.add_argument_group("Video Recording")
        video_recording_group.add_argument(
            "--video",
            dest="video_enable",
            help="Record videos",
            action="store_true",
            default=False,
        )

        performance_group = _parser.add_argument_group("Performance")
        performance_group.add_argument(
            "--perf",
            dest="perf_enable",
            help="Test the performance of the environment",
            action="store_true",
            default=False,
        )
        performance_group.add_argument(
            "--perf_output",
            help='Path to the output of the performance test (special keys: "STDOUT")',
            type=str,
            default="STDOUT",
        )
        performance_group.add_argument(
            "--perf_duration",
            help="Maximum duration of the performance test in seconds (0: No limit)",
            type=float,
            default=150.0,
        )

    ## Hardware args
    for _parser in (
        *real_agent_parsers_with_env,
        *real_env_gen_parsers,
    ):
        hardware_group = _parser.add_argument_group("Hardware")
        hardware_group.add_argument(
            "--hardware",
            "--hw",
            help="Sequence of hardware interfaces to use",
            type=str,
            nargs="*",
            choices=hardware_choices,
            default=[],
        )

    ## Logging
    for _parser in (
        *agent_parsers_with_env,
        *real_agent_parsers_with_env,
    ):
        logging_group = _parser.add_argument_group("Logging")
        logging_group.add_argument(
            "--logdir",
            "--logs",
            dest="logdir_path",
            help="Path to the root directory for storing logs",
            type=str,
            default=SRB_LOGS_DIR,
        )

    ## Teleop args
    for _parser in (
        teleop_agent_parser,
        collect_agent_parser,
        teleop_real_agent_parser,
        collect_real_agent_parser,
    ):
        teleop_group = _parser.add_argument_group("Teleop")
        teleop_group.add_argument(
            "--teleop_device",
            help="Device for interacting with environment",
            type=str,
            nargs="+",
            choices=teleop_device_choices,
            default=[str(TeleopDeviceType.KEYBOARD)],
        )
        teleop_group.add_argument(
            "--pos_sensitivity",
            help="Sensitivity factor for translation",
            type=float,
            default=1.0,
        )
        teleop_group.add_argument(
            "--rot_sensitivity",
            help="Sensitivity factor for rotation",
            type=float,
            default=3.1415927,
        )
        teleop_group.add_argument(
            "--invert_controls",
            "--invert",
            help="Flag to invert the controls for translation in manipulation environments",
            action="store_true",
            default=True,
        )

    ## Algorithm args
    for _parser in (
        train_agent_parser,
        eval_agent_parser,
        teleop_agent_parser,
        collect_agent_parser,
        learn_agent_parser,
        train_real_agent_parser,
        eval_real_agent_parser,
        teleop_real_agent_parser,
        collect_real_agent_parser,
    ):
        algorithm_group = _parser.add_argument_group(
            "Teleop Policy"
            if _parser
            in (
                teleop_agent_parser,
                collect_agent_parser,
                teleop_real_agent_parser,
                collect_real_agent_parser,
            )
            else "Algorithm"
        )
        algorithm_group.add_argument(
            "--algo",
            help="Name of the algorithm",
            type=str,
            choices=algo_choices,
            required=_parser
            not in (
                teleop_agent_parser,
                collect_agent_parser,
                teleop_real_agent_parser,
                collect_real_agent_parser,
            ),
        )
        if _parser not in (
            train_agent_parser,
            learn_agent_parser,
            train_real_agent_parser,
        ):
            algorithm_group.add_argument(
                "--model",
                type=str,
                help="Path to the model checkpoint",
            )

    ## Train args
    for _parser in (
        train_agent_parser,
        train_real_agent_parser,
    ):
        train_group = _parser.add_argument_group("Train")
        mutex_group = train_group.add_mutually_exclusive_group()
        mutex_group.add_argument(
            "--continue_training",
            "--continue",
            help="Continue training the model from the last checkpoint",
            action="store_true",
            default=False,
        )
        mutex_group.add_argument(
            "--model",
            help="Continue training the model from the specified checkpoint",
            type=str,
        )

    # Trigger argcomplete
    if find_spec("argcomplete"):
        import argcomplete

        argcomplete.autocomplete(parser)

    # Enable rich traceback (delayed after argcomplete to maintain snappy completion)
    from srb.utils.tracing import with_rich

    with_rich()

    # Allow separation of arguments meant for other purposes
    if "--" in sys.argv:
        forwarded_args = sys.argv[(sys.argv.index("--") + 1) :]
        sys.argv = sys.argv[: sys.argv.index("--")]
    else:
        forwarded_args = []

    # Parse arguments
    args, other_args = parser.parse_known_args()

    # Add forwarded arguments
    args.forwarded_args = forwarded_args

    # Detect any unsupported arguments
    unsupported_args = [
        arg for arg in other_args if arg.startswith("-") or "=" not in arg
    ]
    if unsupported_args:
        import string

        raise ValueError(
            f"Unsupported CLI argument{'s' if len(unsupported_args) > 1 else ''}: "
            + ", ".join(
                f'"{arg}"' if any(c in string.whitespace for c in arg) else arg
                for arg in unsupported_args
            )
            + (
                (
                    '\nUse "--" to separate arguments meant for spawned processes: '
                    + " ".join(
                        f'"{arg}"' if any(c in string.whitespace for c in arg) else arg
                        for arg in sys.argv
                        if arg not in unsupported_args and arg != "--"
                    )
                    + " -- "
                    + " ".join(
                        f'"{arg}"' if any(c in string.whitespace for c in arg) else arg
                        for arg in unsupported_args
                    )
                )
                if args.subcommand in ("gui", "docs", "test")
                else ""
            )
        )

    # Forward other arguments to hydra
    sys.argv = [sys.argv[0], *other_args]

    return args


class AutoNamespaceTaskAction(argparse.Action):
    NAMESPACE: str = "srb"

    def __call__(
        self,
        parser: argparse.ArgumentParser,
        namespace: argparse.Namespace,
        values: str,
        option_string: str | None = None,
    ):
        if "/" not in values:
            values = f"{self.NAMESPACE}/{values}"
        setattr(namespace, self.dest, values)


class AutoRealNamespaceTaskAction(AutoNamespaceTaskAction):
    NAMESPACE: str = "srb_real"


class ExplicitAction(argparse.Action):
    def __call__(
        self,
        parser: argparse.ArgumentParser,
        namespace: argparse.Namespace,
        values: str,
        option_string: str | None = None,
    ):
        setattr(namespace, self.dest, values)
        setattr(namespace, f"{self.dest}_explicit", True)


class EntityToList(str, Enum):
    ALL = auto()
    ACTION = auto()
    ASSET = auto()
    ENV = auto()
    SCENERY = auto()
    OBJECT = auto()
    ROBOT = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class SupportedAlgo(str, Enum):
    # DreamerV3
    DREAMER = auto()

    # Stable-Baselines3
    SB3_A2C = auto()
    SB3_ARS = auto()
    SB3_CROSSQ = auto()
    SB3_DDPG = auto()
    SB3_PPO = auto()
    SB3_PPO_LSTM = auto()
    SB3_SAC = auto()
    SB3_TD3 = auto()
    SB3_TQC = auto()
    SB3_TRPO = auto()

    # SBX
    SBX_CROSSQ = auto()
    SBX_DDPG = auto()
    SBX_PPO = auto()
    SBX_SAC = auto()
    SBX_TD3 = auto()
    SBX_TQC = auto()

    # skrl
    SKRL_A2C = auto()
    SKRL_AMP = auto()
    SKRL_CEM = auto()
    SKRL_DDPG = auto()
    SKRL_PPO = auto()
    SKRL_PPO_RNN = auto()
    SKRL_RPO = auto()
    SKRL_SAC = auto()
    SKRL_TD3 = auto()
    SKRL_TRPO = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class Lang(str, Enum):
    PYTHON = auto()
    RUST = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


if __name__ == "__main__":
    main()
