# System Requirements

## Hardware Requirements

The hardware requirements for Space Robotics Bench are inherited from the [Isaac Sim requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html). With careful tuning, it is possible to run the included environments on lower-spec systems. However, the performance of some workflows might be limited. The bare minimum requirements are listed below:

| Component  | Requirement |
| ---------- | :---------: |
| CPU        |   x86_64    |
| GPU        | NVIDIA RTX  |
| RAM        |    16 GB    |
| VRAM       |    4 GB     |
| Disk Space |    32 GB    |

<div class="warning">

This project requires a dedicated NVIDIA GPU with RT Cores (RTX series). Isaac Sim does not support GPUs from other vendors (AMD, Intel) or older NVIDIA GPUs without RT Cores.

</div>

## Software Requirements

A Linux-based OS with an appropriate NVIDIA driver is required to use Space Robotics Bench. Other operating systems might be functional, but they are not officially supported. Please let us know if you confirm functionality on other non-listed systems.

| Component                |  Requirement   |
| ------------------------ | :------------: |
| OS (Native Installation) |  Ubuntu 22.04  |
| OS (Docker Installation) | Linux with X11 |
| NVIDIA Driver            |  >=535,\<560   |

### NVIDIA Driver

> Official instructions: [Driver Installation Guide — Choose an Installation Method](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/index.html#choose-an-installation-method)

Install NVIDIA driver by following the official instructions above or through the package manager of your distribution. Although [Isaac Sim specifies `535.129.03` as the recommended version](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/requirements.html#driver-requirements), newer drivers should also be compatible.

## ... continue with [Installation](./install.md)
