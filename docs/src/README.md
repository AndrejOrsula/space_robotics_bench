![](./_images/srb_multi_env.jpg)

The **Space Robotics Bench** is a comprehensive collection of environments and tasks for robotics research in space. It provides a unified framework for experimenting with new tasks across various applications and scenarios, with a primary focus on robot learning techniques while remaining flexible for diverse research directions.

## Key Features

### On-Demand Procedural Generation with [Blender](https://blender.org)

Blender generates procedural assets across diverse scenarios, emphasizing the need for robotic generalization and adaptability in safety-critical space applications.

### Highly-Parallelized Simulation with [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)

NVIDIA Isaac Sim enables hardware-accelerated parallel simulation instances, accelerating parameter tuning, verification, data generation, and online learning workflows. Compliance with [Isaac Lab](https://isaac-sim.github.io/IsaacLab) enhances compatibility with pre-configured robots and sensors.

### Compatibility with [Gymnasium API](https://gymnasium.farama.org)

All tasks use the standardized Gymnasium API, ensuring seamless integration with a broad ecosystem of libraries and tools for reinforcement learning and imitation learning algorithms.

### Integration with [ROS 2](https://ros.org) & [Space ROS](https://space.ros.org)

The benchmark can be installed as a ROS 2 package, providing interoperability with the ROS ecosystem and Space ROS. This gives ROS developers access to reproducible space environments with procedural variety.

### Agnostic Interfaces

The benchmark's interfaces include abstraction layers to ensure flexibility across different robots and space domains. Assets are decoupled in a separate [`srb_assets` repository](https://github.com/AndrejOrsula/srb_assets) for integration with external frameworks.

## Architecture

The Space Robotics Bench follows a modular architecture:

1. **Core System**: Central API providing environment registration, configuration management, and interface to Isaac Sim.

1. **Environment Framework**: Modular environments built on Gymnasium with procedural generation support.

1. **Agent Interface**: Multiple agent types supported (zero, random, teleoperation, ROS, learning-based), with flexible integration options.

1. **Workflow Management**: Streamlined workflows for training, evaluation, data collection, and teleoperation with standardized logging.

1. **Extension System**: Interfaces for various external systems (ROS 2, teleoperation devices) and learning frameworks.

All components interact through standardized APIs, allowing users to mix and match capabilities while maintaining a consistent experience across different use cases.
