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
from srb.utils.cache import read_env_list_cache, update_env_list_cache
from srb.utils.path import SRB_APPS_DIR, SRB_DIR, SRB_LOGS_DIR

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv
    from srb.interfaces.teleop import CombinedTeleopInterface


def main():
    def impl(
        subcommand: Literal["agent", "ls", "gui", "repl", "docs", "test"], **kwargs
    ):
        if not find_spec("omni"):
            raise ImportError(
                "The Space Robotics Bench requires an environment with NVIDIA Omniverse and Isaac Sim installed."
            )

        match subcommand:
            case "agent":
                if kwargs["agent_subcommand"] == "learn":
                    raise NotImplementedError()
                else:
                    run_agent_with_env(**kwargs)
            case "ls":
                list_registered(**kwargs)
            case "gui":
                launch_gui(**kwargs)
            case "repl":
                enter_repl(**kwargs)
            case "docs":
                serve_docs(**kwargs)
            case "test":
                run_tests(**kwargs)

    impl(**vars(parse_cli_args()))


### Agent ###
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
    video_enable: bool,
    video_length: int,
    video_interval: int,
    hide_ui: bool,
    interface: Sequence[str] = (),
    forwarded_args: Sequence[str] = (),
    **kwargs,
):
    from srb.core.app import AppLauncher

    # Preprocess kwargs
    kwargs["enable_cameras"] = video_enable or env_id.endswith("_visual")
    kwargs["experience"] = SRB_APPS_DIR.joinpath(
        f"srb.{'headless.' if kwargs['headless'] else ''}{'rendering.' if video_enable or kwargs['enable_cameras'] else ''}kit"
    )

    # Launch Isaac Sim
    launcher = AppLauncher(launcher_args=kwargs)

    # Update the offline environment registry
    update_env_list_cache()

    from omni.physx import acquire_physx_interface

    from srb.interfaces.teleop import EventKeyboardTeleopInterface
    from srb.utils import logging
    from srb.utils.cfg import hydra_task_config, last_logdir, new_logdir
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
            and (logdir.parent.parent.name == env_id.removeprefix("srb/"))
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

    @hydra_task_config(
        task_name=env_id,
        agent_cfg_entry_point=f"{kwargs['algo']}_cfg" if kwargs.get("algo") else None,
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
            video_kwargs = {
                "video_folder": logdir.joinpath("videos"),
                "step_trigger": lambda step: step % video_interval == 0,
                "video_length": video_length,
                "disable_logger": True,
            }
            logging.info(f"Recording videos: {video_kwargs}")
            env = gymnasium.wrappers.RecordVideo(env, **video_kwargs)

        # Add keyboard callbacks
        if not kwargs["headless"] and agent_subcommand not in [
            "teleop",
            "collect",
            "train",
        ]:
            _cb_keyboard = EventKeyboardTeleopInterface({"L": env.reset})

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
                    teleop_agent(**kwargs)
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
    **kwargs,
):
    import gymnasium
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

    # TODO[low]: Add force feedback for haptic teleoperation

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
    if env_supports_direct_teleop:
        _teleop_agent_direct(
            env=env,
            sim_app=sim_app,
            teleop_interface=teleop_interface,
            **kwargs,
        )
    elif (
        isinstance(env.observation_space, gymnasium.spaces.Dict)
        and "command" in env.observation_space.spaces.keys()
    ):
        if algo:
            _teleop_agent_via_policy(
                env=env,
                sim_app=sim_app,
                teleop_interface=teleop_interface,
                algo=algo,
                **kwargs,
            )
        else:
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
    **kwargs,
):
    import torch

    from srb.utils import logging

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
    recognized_cmd_keys: Sequence[str] = ("command", "_command", "cmd", "_cmd"),
    **kwargs,
):
    import torch
    from gymnasium.core import (
        ActType,
        ObservationWrapper,
        ObsType,
        SupportsFloat,
        WrapperObsType,
    )

    from srb.utils import logging

    ## Try disabling event for the command
    if hasattr(env.unwrapped, "event_manager"):
        event_cmd: Tuple[str, int] | None = None
        for (
            category,
            event_names,
        ) in env.unwrapped.event_manager._mode_term_names.items():  # type: ignore
            for recognized_cmd_key in recognized_cmd_keys:
                if recognized_cmd_key in event_names:
                    event_cmd = (category, event_names.index(recognized_cmd_key))
                    break
            else:
                continue
            break
        if event_cmd:
            env.unwrapped.event_manager._mode_term_names[event_cmd[0]].pop(event_cmd[1])  # type: ignore
            env.unwrapped.event_manager._mode_term_cfgs[event_cmd[0]].pop(event_cmd[1])  # type: ignore

    class InjectTeleopWrapper(ObservationWrapper):
        def __init__(self, env, *args, **kwargs):
            super().__init__(env, *args, **kwargs)
            self._obs_cmd_key = next(
                (
                    key
                    for key in recognized_cmd_keys
                    if key in self.observation_space.spaces.keys()  # type: ignore
                ),
                None,
            )
            if not self._obs_cmd_key:
                raise ValueError(
                    f"Unable to find the command key in the observation space: {recognized_cmd_keys}"
                )

            self._internal_cmd_attr_name = next(
                (
                    key
                    for key in recognized_cmd_keys
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

        def observation(self, observation: ObsType) -> WrapperObsType:  # type: ignore
            ## Get actions from the teleoperation interface and process them
            twist, event = teleop_interface.advance()
            if invert_controls:
                twist[:2] *= -1.0

            cmd_len = observation[self._obs_cmd_key].shape[-1]  # type: ignore

            ## Map teleoperation actions to commands
            match cmd_len:
                case _ if cmd_len < 7:
                    cmd = torch.from_numpy(twist[:cmd_len]).to(
                        device=env.unwrapped.device,  # type: ignore
                        dtype=torch.float32,
                    )
                    observation[self._obs_cmd_key][:] = cmd  # type: ignore
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
                case 7:
                    cmd = torch.concat(
                        (
                            torch.from_numpy(twist).to(
                                device=env.unwrapped.device,  # type: ignore
                                dtype=torch.float32,
                            ),
                            torch.Tensor((-1.0 if event else 1.0,)).to(
                                device=twist.device  # type: ignore
                            ),
                        )
                    )
                    observation[self._obs_cmd_key][:] = cmd  # type: ignore
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

            return observation  # type: ignore

        def step(
            self,
            action: ActType,  # type: ignore
        ) -> Tuple[WrapperObsType, SupportsFloat, bool, bool, Mapping[str, Any]]:  # type: ignore
            # Exit if the simulation is not running
            if not sim_app.is_running():
                exit()

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

            sb3.run(workflow=WORKFLOW, algo=algo.strip("sb3_"), **kwargs)
        case _sbx if algo.startswith("sbx"):
            from srb.integrations.sbx import main as sbx

            sbx.run(workflow=WORKFLOW, algo=algo.strip("sbx_"), **kwargs)


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

            sb3.run(workflow=WORKFLOW, algo=algo.strip("sb3_"), **kwargs)
        case _sbx if algo.startswith("sbx"):
            from srb.integrations.sbx import main as sbx

            sbx.run(workflow=WORKFLOW, algo=algo.strip("sbx_"), **kwargs)


### List ###
def list_registered(
    category: str | Sequence[str], show_hidden: bool, forwarded_args: Sequence[str] = ()
):
    from srb.core.app import AppLauncher

    if not find_spec("rich"):
        raise ImportError(
            'The "rich" package is required to list registered entities of the Space Robotics Bench'
        )

    # Launch Isaac Sim
    launcher = AppLauncher(
        headless=True, experience=SRB_APPS_DIR.joinpath("srb.barebones.kit")
    )

    # Update the offline environment registry
    update_env_list_cache()

    import importlib
    import inspect
    from dataclasses import _MISSING_TYPE
    from os import path

    from rich import print
    from rich.table import Table

    from srb.utils.str import convert_to_snake_case

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

        table = Table(title="Assets of the Space Robotics Bench")
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
                    asset_name = convert_to_snake_case(asset_class.__name__)
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
                    asset_name = convert_to_snake_case(asset_class.__name__)
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
                    asset_name = convert_to_snake_case(asset_class.__name__)
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
        from srb.core.action import ActionGroupRegistry, canonicalize_action_group_name
        from srb.core.action import group as srb_action_groups

        table = Table(title="Action Groups of the Space Robotics Bench")
        table.add_column("#", justify="right", style="cyan", no_wrap=True)
        table.add_column("Name", justify="left", style="blue", no_wrap=True)
        table.add_column("Path", justify="left", style="white")

        for i, action_group_class in enumerate(ActionGroupRegistry.registry, 1):
            action_group_name = canonicalize_action_group_name(
                action_group_class.__name__
            )
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

    # Print table for environments
    if EntityToList.ENV in category:
        import gymnasium

        from srb.utils.registry import get_srb_tasks

        table = Table(title="Environments of the Space Robotics Bench")
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
                task_id.removeprefix("srb/")
                + (
                    " <template>"
                    if task_id.removeprefix("srb/").startswith("_")
                    else ""
                ),
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
        "Launching GUI of the Space Robotics Bench with the following command: "
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
def enter_repl(hide_ui: bool, forwarded_args: Sequence[str] = (), **kwargs):
    from srb.core.app import AppLauncher

    if not find_spec("ptpython"):
        raise ImportError(
            'The "ptpython" package is required to enter REPL of the Space Robotics Bench'
        )

    # Preprocess kwargs
    kwargs["enable_cameras"] = True
    kwargs["experience"] = SRB_APPS_DIR.joinpath(
        f"srb.{'headless.' if kwargs['headless'] else ''}rendering.kit"
    )

    # Launch Isaac Sim
    launcher = AppLauncher(launcher_args=kwargs)

    import ptpython

    import srb  # noqa: F401
    from srb.utils import logging  # noqa: F401
    from srb.utils.isaacsim import hide_isaacsim_ui

    # Update the offline environment registry
    update_env_list_cache()

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
        raise FileNotFoundError(
            'The "mdbook" tool is required to serve the docs of the Space Robotics Bench'
        )

    cmd = (
        "mdbook",
        "serve",
        SRB_DIR.joinpath("docs").as_posix(),
        "--open",
        *forwarded_args,
    )
    logging.info(
        "Serving the docs of the Space Robotics Bench with the following command: "
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
        raise ImportError(
            'The "pytest" package is required to run tests of the Space Robotics Bench'
        )

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
            f"Running {str(lang)} tests of the Space Robotics Bench with the following command: "
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


### CLI ###
def parse_cli_args() -> argparse.Namespace:
    """
    Parse command-line arguments for this script.
    """

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
        help="Collect demonstrations",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    learn_agent_parser = agent_subparsers.add_parser(
        "learn",
        help="Learn from demonstrations",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    ## List subcommand
    list_parser = subparsers.add_parser(
        "ls",
        help="List registered assets and environments"
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
    repl_parser = subparsers.add_parser(
        "repl",
        help="Enter REPL"
        + ("" if find_spec("ptpython") else ' (MISSING: "ptpython" Python package)'),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

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

    ## Launcher args
    for _agent_parser in (
        zero_agent_parser,
        rand_agent_parser,
        teleop_agent_parser,
        ros_agent_parser,
        train_agent_parser,
        eval_agent_parser,
        collect_agent_parser,
        learn_agent_parser,
        repl_parser,
    ):
        launcher_group = _agent_parser.add_argument_group("Launcher")
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
            "--kit_args",
            help="CLI args for the Omniverse Kit as a string separated by a space delimiter (e.g., '--ext-folder=/path/to/ext1 --ext-folder=/path/to/ext2')",
            type=str,
            default="",
        )

    ## Environment args
    _env_choices = read_env_list_cache()
    _interface_choices = sorted(map(str, InterfaceType))
    for _agent_parser in (
        zero_agent_parser,
        rand_agent_parser,
        teleop_agent_parser,
        ros_agent_parser,
        train_agent_parser,
        eval_agent_parser,
        collect_agent_parser,
    ):
        environment_group = _agent_parser.add_argument_group("Environment")
        environment_group.add_argument(
            "-e",
            "--env",
            "--task",
            dest="env_id",
            help="Name of the environment to select",
            type=str,
            action=AutoNamespaceTaskAction,
            choices=_env_choices,
            required=True,
        )

        logging_group = _agent_parser.add_argument_group("Logging")
        logging_group.add_argument(
            "--logdir",
            "--logs",
            dest="logdir_path",
            help="Path to the root directory for storing logs",
            type=str,
            default=SRB_LOGS_DIR,
        )

        video_recording_group = _agent_parser.add_argument_group("Video Recording")
        video_recording_group.add_argument(
            "--video",
            dest="video_enable",
            help="Record videos",
            action="store_true",
            default=False,
        )
        video_recording_group.add_argument(
            "--video_length",
            help="Length of the recorded video (in steps)",
            type=int,
            default=1000,
        )
        video_recording_group.add_argument(
            "--video_interval",
            help="Interval between video recordings (in steps)",
            type=int,
            default=10000,
        )

        interfaces_group = _agent_parser.add_argument_group("Interface")
        interfaces_group.add_argument(
            "--interface",
            help="Sequence of interfaces to enable",
            type=str,
            nargs="*",
            choices=_interface_choices,
            default=[],
        )

    ## Teleop args
    _teleop_device_choices = sorted(map(str, TeleopDeviceType))
    for _agent_parser in (teleop_agent_parser, collect_agent_parser):
        teleop_group = _agent_parser.add_argument_group("Teleop")
        teleop_group.add_argument(
            "--teleop_device",
            help="Device for interacting with environment",
            type=str,
            nargs="+",
            choices=_teleop_device_choices,
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
            help="Flag to invert the controls for translation",
            action="store_true",
            default=False,
        )

    ## Algorithm args
    _algo_choices = sorted(map(str, SupportedAlgo))
    for _agent_parser in (
        train_agent_parser,
        eval_agent_parser,
        teleop_agent_parser,
        collect_agent_parser,
        learn_agent_parser,
    ):
        algorithm_group = _agent_parser.add_argument_group(
            "Teleop Policy"
            if _agent_parser in (teleop_agent_parser, collect_agent_parser)
            else "Algorithm"
        )
        algorithm_group.add_argument(
            "--algo",
            help="Name of the algorithm",
            type=str,
            choices=_algo_choices,
            required=_agent_parser not in (teleop_agent_parser, collect_agent_parser),
        )
        if _agent_parser != train_agent_parser:
            algorithm_group.add_argument(
                "--model",
                type=str,
                help="Path to the model checkpoint",
            )

    ## Train args
    train_group = train_agent_parser.add_argument_group("Train")
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
    def __call__(
        self,
        parser: argparse.ArgumentParser,
        namespace: argparse.Namespace,
        values: str,
        option_string: str | None = None,
    ):
        if "/" not in values:
            DEFAULT_TASK_NAMESPACE: str = "srb"
            values = f"{DEFAULT_TASK_NAMESPACE}/{values}"
        setattr(namespace, self.dest, values)


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
    # Dreamer
    DREAMER = auto()
    # SB3
    SB3_A2C = auto()
    SB3_DDPG = auto()
    SB3_DQN = auto()
    SB3_PPO = auto()
    SB3_SAC = auto()
    SB3_TD3 = auto()
    # SB3 Contrib
    SB3_ARS = auto()
    SB3_CROSSQ = auto()
    SB3_QRDQN = auto()
    SB3_TQC = auto()
    SB3_TRPO = auto()
    SB3_PPO_LSTM = auto()
    # SBX
    SBX_DDPG = auto()
    SBX_DQN = auto()
    SBX_PPO = auto()
    SBX_SAC = auto()
    SBX_TD3 = auto()
    SBX_TQC = auto()
    SBX_CrossQ = auto()
    # SKRL
    SKRL_A2C = auto()
    SKRL_AMP = auto()
    SKRL_CEM = auto()
    SKRL_DDPG = auto()
    SKRL_DDQN = auto()
    SKRL_DQN = auto()
    SKRL_PPO = auto()
    SKRL_PPO_RNN = auto()
    SKRL_RPO = auto()
    SKRL_SAC = auto()
    SKRL_TD3 = auto()
    SKRL_TRPO = auto()
    SKRL_IPPO = auto()
    SKRL_MAPPO = auto()

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
