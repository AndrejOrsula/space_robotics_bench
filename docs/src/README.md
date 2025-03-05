![](./_images/srb_multi_env.jpg)

The **Space Robotics Bench** aims to be a comprehensive collection of environments and tasks for robotics research in the challenging domain of space. The benchmark covers a wide range of applications and scenarios while providing a unified framework for experimenting with new tasks. Although the primary focus is on the application of robot learning techniques, the benchmark is designed to be flexible and extensible to accommodate a variety of research directions.

<div class="warning">
This documentation is currently incomplete. Inactive pages found in the navigation panel indicate what topics will be covered prior to the first release. Please let us know by opening an issue if something is missing or about a specific topic that you are interested in having documented first. Thank you! :)
</div>

## Key Features

### On-Demand Procedural Generation with [Blender](https://blender.org)

Blender is used to generate procedural assets across a wide range of scenarios to provide environments that are representative of the diversity in space. By doing so, this benchmark emphasizes the need for generalization and adapatibility of robots in space due to their safety-critical nature.

### Highly-Parallelized Simulation with [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)

By leveraging the hardware-acceleration capabilities of NVIDIA Isaac Sim, all environments support parallel simulation instances, significantly accelerating workflows such as parameter tuning, verification, synthetic data generation, and online learning. The uniqueness of each procedurally generated instance also contributes towards the diversity that robots experience alongside the included domain randomization. Furthermore, compliance with [Isaac Lab](https://isaac-sim.github.io/IsaacLab) enhances compatibility with a wide array of pre-configured robots and sensors.

### Compatibility with [Gymnasium API](https://gymnasium.farama.org)

All tasks are registered with the standardized Gymnasium API, ensuring seamless integration with a broad ecosystem of libraries and tools. This enables developers to leverage popular reinforcement learning and imitation learning algorithms while also simplifying the evaluation and comparison of various solutions across diverse scenarios, giving rise to potential collaboration efforts.

### Integration with [ROS 2](https://ros.org) & [Space ROS](https://space.ros.org)

The benchmark can also be installed as a ROS 2 package to bring interoperability to its wide ecosystem, including aspects of Space ROS. This integration provides access to a rich set of tools and libraries that accelerate the development and deployment of robotic systems. At the same time, ROS developers get access to a set of reproducible space environments for evaluating their systems and algorithms while benefiting from the procedural variety and parallel instances via namespaced middleware communication.

### Agnostic Interfaces

The interfaces of the benchmark are designed with abstraction layers to ensure flexibility for various applications and systems. By adjusting configuration and changing procedural pipelines, a single task definition can be reused across different robots and domains of space. Moreover, all assets are decoupled from the benchmark into a separate [`srb_assets` repository](https://github.com/AndrejOrsula/srb_assets), enabling their straightforward integration with external frameworks and projects.

## Architecture
