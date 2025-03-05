# Integration with ROS 2

> Take a look at [ROS 2 Workflow](../../instructions/workflows/ros2.md) if you are directly interested in instructions with concrete examples.

## Motivation

ROS has become the de facto standard for developing robotic systems across various environments, including outer space, with the advent of ROS 2. The Space Robotics Bench integrates seamlessly with the ROS 2 ecosystem, facilitating the exposure of relevant simulation data to ROS nodes. This integration aims to accelerate the iterative development and testing of space robotic systems.

## Approach

Isaac Sim’s computational graph is primarily offloaded to the system’s dedicated NVIDIA GPU, which presents challenges in directly exposing all internal states to the ROS middleware without compromising performance. Instead, the package focuses on exposing the inputs and outputs of each registered Gymnasium environment alongside a fixed global mapping configuration to maintain modularity and flexibility within the simulation architecture.

## Workflow

The Space Robotics Bench provides a `ros2.py` script that spawns a ROS node to interface with the environments. Subscribers, publishers, and services are dynamically created based on the selected environment and global mapping configuration. When running multiple environment instances in parallel, the script automatically assigns different namespaces to inputs and outputs, preventing conflicts. The script also includes additional functionalities, such as simulation reset capabilities.
