#!/root/isaac-sim/python.sh
"""
Teleoperation script for running tasks with human-in-the-loop control

Examples:
    ros2 run space_robotics_bench teleop.py
    ros2 run space_robotics_bench teleop.py --task sample_collection --num_envs 4 --teleop_device spacemouse
"""

from _cli_utils import add_default_cli_args, argparse, launch_app, shutdown_app
from omni.isaac.lab.app import AppLauncher


def main(launcher: AppLauncher, args: argparse.Namespace):
    ## Note: Importing modules here due to delayed Omniverse Kit extension loading
    import threading
    from os import path

    import gymnasium
    import numpy as np
    import torch
    from omni.isaac.kit import SimulationApp
    from omni.isaac.lab.utils.dict import print_dict
    from rclpy.executors import MultiThreadedExecutor

    import space_robotics_bench  # Noqa: F401
    from space_robotics_bench.core import mdp
    from space_robotics_bench.core.actions import (
        ManipulatorTaskSpaceActionCfg,
        MultiCopterActionGroupCfg,
        WheeledRoverActionGroupCfg,
    )
    from space_robotics_bench.core.interfaces import ROS2, GuiInterface
    from space_robotics_bench.core.managers import SceneEntityCfg
    from space_robotics_bench.core.teleop_devices import CombinedInterface
    from space_robotics_bench.utils.parsing import create_logdir_path, parse_task_cfg
    from space_robotics_bench.utils.ros import enable_ros2_bridge

    enable_ros2_bridge()
    import rclpy
    from rclpy.node import Node

    if args.headless and "keyboard" in args.teleop_device:
        raise ValueError("Native teleoperation is only supported in GUI mode.")

    ## Extract simulation app
    sim_app: SimulationApp = launcher.app

    # Parse configuration
    task_cfg = parse_task_cfg(
        task_name=args.task,
        device=args.device,
        num_envs=args.num_envs,
        use_fabric=not args.disable_fabric,
    )
    # Disable truncation
    if hasattr(task_cfg, "enable_truncation"):
        task_cfg.enable_truncation = False

    ## Create the environment
    env = gymnasium.make(
        id=args.task, cfg=task_cfg, render_mode="rgb_array" if args.video else None
    )

    # ROS 2 node
    try:
        rclpy.init(args=None)
    except Exception as _:
        pass
    ros_node = Node("srb")

    ## Teleop interface
    teleop_interface = CombinedInterface(
        devices=args.teleop_device,
        pos_sensitivity=args.pos_sensitivity,
        rot_sensitivity=args.rot_sensitivity,
        action_cfg=env.unwrapped.cfg.actions,
        node=ros_node,
    )

    def cb_reset():
        global should_reset
        should_reset = True

    global should_reset
    should_reset = False
    teleop_interface.add_callback("L", cb_reset)

    teleop_interface.reset()
    print(teleop_interface)

    ## ROS 2 interface
    if args.ros2_integration:
        ros2_interface = ROS2(env, node=ros_node)

    ## GUI interface
    if args.gui_integration:
        gui_interface = GuiInterface(env, node=ros_node)

    ## Initialize the environment
    observation, info = env.reset()

    ## Add wrapper for video recording (if enabled)
    if args.video:
        logdir = create_logdir_path("srb", args.task)
        video_kwargs = {
            "video_folder": path.join(logdir, "videos"),
            "step_trigger": lambda step: step % args.video_interval == 0,
            "video_length": args.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gymnasium.wrappers.RecordVideo(env, **video_kwargs)

    def process_actions(twist: np.ndarray, gripper_cmd: bool) -> torch.Tensor:
        twist = torch.tensor(
            twist, dtype=torch.float32, device=env.unwrapped.device
        ).repeat(env.unwrapped.num_envs, 1)
        if isinstance(env.unwrapped.cfg.actions, ManipulatorTaskSpaceActionCfg):
            if not args.disable_control_scheme_inversion:
                twist[:, :2] *= -1.0
            gripper_action = torch.zeros(twist.shape[0], 1, device=twist.device)
            gripper_action[:] = -1.0 if gripper_cmd else 1.0
            return torch.concat([twist, gripper_action], dim=1)
        elif isinstance(env.unwrapped.cfg.actions, MultiCopterActionGroupCfg):
            return torch.concat(
                [
                    twist[:, :3],
                    twist[:, 5].unsqueeze(1),
                ],
                dim=1,
            )

        elif isinstance(env.unwrapped.cfg.actions, WheeledRoverActionGroupCfg):
            return twist[:, :2]

    # ROS 2 executor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros_node)
    thread = threading.Thread(target=executor.spin)
    thread.daemon = True
    thread.start()

    ## Run the environment
    with torch.inference_mode():
        while sim_app.is_running():
            # Get actions from the teleoperation interface
            actions = process_actions(*teleop_interface.advance())

            # Step the environment
            observation, reward, terminated, truncated, info = env.step(actions)

            # Note: Each environment is automatically reset (independently) when terminated or truncated

            # Provide force feedback for teleop devices
            if isinstance(env.unwrapped.cfg.actions, ManipulatorTaskSpaceActionCfg):
                FT_FEEDBACK_SCALE = torch.tensor([0.16, 0.16, 0.16, 0.0, 0.0, 0.0])
                ft_feedback_asset_cfg = SceneEntityCfg(
                    "robot",
                    body_names=task_cfg.robot_cfg.regex_links_hand,
                )
                ft_feedback_asset_cfg.resolve(env.unwrapped.scene)
                ft_feedback = (
                    FT_FEEDBACK_SCALE
                    * mdp.body_incoming_wrench_mean(
                        env=env.unwrapped,
                        asset_cfg=ft_feedback_asset_cfg,
                    )[0, ...].cpu()
                )
                teleop_interface.set_ft_feedback(ft_feedback)

            ## ROS 2 interface
            if args.ros2_integration:
                ros2_interface.publish(observation, reward, terminated, truncated, info)
                ros2_interface.update()

            ## GUI interface
            if args.gui_integration:
                gui_interface.update()

            if should_reset:
                should_reset = False
                teleop_interface.reset()
                observation, info = env.unwrapped.reset()

    ## Close the environment
    env.close()


### Helper functions ###
def parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    algorithm_group = parser.add_argument_group(
        "teleop",
        description="Arguments for teleoperation",
    )
    algorithm_group.add_argument(
        "--teleop_device",
        type=str,
        nargs="+",
        default=["keyboard"],
        help="Device for interacting with environment",
    )
    algorithm_group.add_argument(
        "--pos_sensitivity",
        type=float,
        default=10.0,
        help="Sensitivity factor for translation.",
    )
    algorithm_group.add_argument(
        "--rot_sensitivity",
        type=float,
        default=40.0,
        help="Sensitivity factor for rotation.",
    )
    algorithm_group.add_argument(
        "--disable_control_scheme_inversion",
        action="store_true",
        default=False,
        help="Flag to disable inverting the control scheme due to view for manipulation-based tasks.",
    )
    algorithm_group.add_argument(
        "--ros2_integration",
        action="store_true",
        default=False,
        help="Flag to enable ROS 2 interface for subscribing to per-env actions and publishing per-env observations",
    )
    algorithm_group.add_argument(
        "--gui_integration",
        action="store_true",
        default=False,
        help="Flag to enable GUI integration",
    )
    add_default_cli_args(parser)
    return parser.parse_args()


if __name__ == "__main__":
    # Parse arguments
    args = parse_cli_args()

    # Launch the app
    launcher = launch_app(args)

    # Run the main function
    main(launcher=launcher, args=args)

    # Shutdown the app
    shutdown_app(launcher)
