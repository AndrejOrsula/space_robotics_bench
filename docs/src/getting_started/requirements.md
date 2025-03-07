# System Requirements

## Hardware Requirements

The hardware requirements for using the Space Robotics Bench are inherited from the [Isaac Sim requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html). With careful tuning, it is possible to run the included environments on lower-spec systems. However, the performance of some workflows might be limited. The bare minimum requirements are listed below:

| Component  | Requirement |
| ---------- | :---------: |
| CPU        |  `x86_64`   |
| GPU        | NVIDIA RTX  |
| RAM        |    8 GB     |
| VRAM       |    4 GB     |
| Disk Space |    32 GB    |

<div class="warning">
This project requires a dedicated NVIDIA GPU with RT Cores (RTX series). Isaac Sim does not support GPUs from other vendors (AMD, Intel) or older NVIDIA GPUs without RT Cores.
</div>

## Software Requirements

Linux-based OS with an appropriate NVIDIA driver is required to use the Space Robotics Bench. Other OS might be functional, but they are not officially supported. Please let us know if you confirmed functionality on other non-listed systems.

| Component                |  Requirement   |
| ------------------------ | :------------: |
| OS (Native Installation) |  Ubuntu 22.04  |
| OS (Docker Installation) | Linux with X11 |
| NVIDIA Driver            |   >=535,\<560   |
