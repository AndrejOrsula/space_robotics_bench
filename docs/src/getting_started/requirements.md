# System Requirements

<div class="warning">
This project requires a dedicated NVIDIA GPU with RT Cores (RTX series). Isaac Sim does not support GPUs from other vendors (AMD, Intel) or older NVIDIA GPUs without RT Cores.
</div>

## Hardware Requirements

The hardware requirements for running Space Robotics Bench are inherited from the [Isaac Sim requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html). While it is possible to run the simulation on lower-spec systems, performance will be significantly reduced.

| Component    | Minimum Requirement                   |
| ------------ | ------------------------------------- |
| Architecture | `x86_64`                              |
| CPU          | Any silicon-rich rock will do         |
| RAM          | 16 GB                                 |
| GPU          | NVIDIA GPU with RT Cores (RTX series) |
| VRAM         | 4 GB                                  |
| Disk Space   | 30 GB                                 |
| Network      | 12 GB (for pulling Docker images)     |

## Software Requirements

The following software requirements are essential for running the simulation:

| Component     | Requirement                                               |
| ------------- | --------------------------------------------------------- |
| OS            | Linux-based distribution (Ubuntu 22.04/24.04 recommended) |
| NVIDIA Driver | 535.183.01+ (officially tested version)                   |

### Docker-specific Requirements

When using the Docker setup (recommended for most users):

| Component                | Requirement                                | Notes                                  |
| ------------------------ | ------------------------------------------ | -------------------------------------- |
| Docker Engine            | 20.10.0+ (24.0.0+ recommended)             | Required for container management      |
| NVIDIA Container Toolkit | Compatible with your Docker and GPU driver | Enables GPU access inside containers   |
| Display Server           | X11 (Wayland is supported via XWayland)    | Required for GUI application rendering |
