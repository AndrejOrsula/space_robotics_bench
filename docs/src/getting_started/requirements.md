# System Requirements

## Hardware Requirements

SRB targets Isaac Sim 6.1.0 (`v6.1.0`) and Isaac Lab 3.0.0 (`release/3.0.0`). Their commits are `7c206f75bdadd9e05fc457f19863ca4c3f0cb693` and `fdc650c88dfd86aa158b9a1760b6537fef360555`. Docker x86_64 is tested. Native, Apptainer/HPC, and arm64 are untested. The tables show SRB's minimum requirements and NVIDIA's tested baseline.

### SRB declared minimum

| Component | Requirement |
| ---------- | :----------------------------: |
| CPU | x86_64 Linux |
| GPU | NVIDIA GPU with RT Cores for Isaac Sim workflows |
| RAM | 16 GB (32 GB for simulator; estimates) |
| VRAM | About 4 GB for GPU-accelerated workflows that do not launch Isaac Sim; about 12 GB for SRB simulator workflows (estimates). Isaac Sim's tested baseline is 16 GB. |
| Disk Space | 50 GB SSD, plus source, assets, and caches |

<div class="warning">

Isaac Sim needs a dedicated NVIDIA GPU with RT Cores; AMD, Intel, and older NVIDIA GPUs without RT Cores are unsupported. Host-only commands and tests need no GPU.

</div>

SRB supports `linux/amd64` with NVIDIA RTX. Linux `arm64`/`aarch64` installers are untested and unsupported.

### NVIDIA Isaac Sim 6.1 tested baseline

NVIDIA's [Isaac Sim 6.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html)
specify this x86_64 baseline:

| Component | NVIDIA tested baseline |
| ------------------------ | :----------------: |
| OS | Ubuntu 22.04/24.04 |
| CPU | 4 cores (Intel Core i7 7th generation or AMD Ryzen 5 minimum) |
| RAM | 32 GB |
| GPU | GeForce RTX 4080 minimum |
| VRAM | 16 GB |
| Storage | 50 GB SSD |
| Linux driver | 595.58.03 |

Training and large sensor scenes need more RAM and VRAM. GPUs without RT Cores, including A100/H100, are unsupported. NVIDIA aarch64 support is limited to DGX Spark (DGX OS 7, driver 580.159.03); SRB does not support it.

## Software Requirements

SRB requires Linux and an NVIDIA driver appropriate for the GPU. Other operating systems are unsupported.

| Component | Requirement |
| ------------------------ | :----------------: |
| OS (Native Installation) | Ubuntu 22.04/24.04, x86_64 |
| OS (Docker Installation) | Linux (`X11` required only for GUI workflows) |
| NVIDIA Driver | Appropriate for the GPU; see NVIDIA's tested Isaac Sim 6.1 baseline above |
| Python | 3.12.\* |
| Isaac Sim | Source `VERSION` 6.1.0-rc.26, built from commit `7c206f75bdadd9e05fc457f19863ca4c3f0cb693` |
| Isaac Lab | 3.0.0 (`release/3.0.0`) at `fdc650c88dfd86aa158b9a1760b6537fef360555` |
| ROS 2 | Jazzy, installed and sourced separately when ROS workflows are used |

SRB builds Isaac Sim from source and installs Isaac Lab in that Python environment. Docker also supports NVIDIA's NGC Isaac Sim image, with different paths, UID, and cache ownership; SRB workflows use the source image.

### NVIDIA Driver

Install the NVIDIA driver using [NVIDIA's official guide](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/index.html#choose-an-installation-method) or your distribution's package manager.

Next: [Installation](install.md)
