![](./_images/srb_multi_env.jpg)

**Space Robotics Bench** is a comprehensive collection of environments and tasks for robotics research in the challenging domain of space. It provides a unified framework for developing and validating autonomous systems under diverse extraterrestrial scenarios. At the same time, its design is flexible and extensible to accommodate a variety of development workflows and research directions beyond Earth.

## Key Features

### Highly Parallelized Simulation via [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)

NVIDIA Isaac Sim enables hardware-accelerated parallel simulation instances, accelerating parameter tuning, verification, data generation, and online learning workflows. Compliance with [Isaac Lab](https://isaac-sim.github.io/IsaacLab) enhances compatibility with pre-configured robots and sensors.

### On-Demand Procedural Generation with [SimForge](https://github.com/AndrejOrsula/simforge)

SimForge leverages Blendet and other generators to create procedural assets across diverse scenarios, emphasizing the need for robotic generalization and adaptability in safety-critical space applications.

### Extensive Domain Randomization

The benchmark supports extensive randomization of simulation parameters, sensor noise, lighting conditions, and other environmental factors to enhance robustness and generalization of learning algorithms.

### Compatibility with [Gymnasium API](https://gymnasium.farama.org)

All tasks use the standardized Gymnasium API, ensuring seamless integration with a broad ecosystem of libraries and tools for reinforcement learning and imitation learning algorithms.

### Seamless Interface with [ROS 2](https://ros.org)

The benchmark can be installed as a ROS 2 package, providing interoperability with the ROS ecosystem and Space ROS. This gives ROS developers access to reproducible space environments with procedural variety.

### Abstract Architecture

The benchmark's agnostic interfaces include abstraction layers to ensure flexibility across different robots and space domains. Assets are decoupled in a separate [`srb_assets` repository](https://github.com/AndrejOrsula/srb_assets) for integration with external frameworks.
