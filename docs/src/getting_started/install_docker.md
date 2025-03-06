# Installation: Docker

The Docker-based installation is recommended for most users, providing a pre-configured environment with all dependencies included.

## Advantages of Docker Installation

- **Simplified Setup**: No need to manually install Isaac Sim or resolve dependency conflicts
- **Consistency**: Works the same across different Linux distributions
- **Isolation**: Keeps the Space Robotics Bench environment separate from your system
- **Easy Updates**: Simple commands to pull the latest version

## Prerequisites

Ensure your system meets the [system requirements](./requirements.md), specifically:

- NVIDIA GPU with RT Cores (RTX series)
- Compatible NVIDIA drivers (535.183.01+ recommended)
- X11 display server (or Wayland with XWayland)

### Docker-specific Requirements

When using the Docker setup (recommended for most users):

| Component                | Requirement                                | Notes                                  |
| ------------------------ | ------------------------------------------ | -------------------------------------- |
| Docker Engine            | 20.10.0+ (24.0.0+ recommended)             | Required for container management      |
| NVIDIA Container Toolkit | Compatible with your Docker and GPU driver | Enables GPU access inside containers   |
| Display Server           | X11 (Wayland is supported via XWayland)    | Required for GUI application rendering |

## Installation Steps

### 1. Install Docker Engine

```bash
# Install Docker using the convenience script
curl -fsSL https://get.docker.com | sh

# Start Docker and enable auto-start
sudo systemctl enable --now docker

# Add your user to the docker group (eliminates need for sudo)
sudo usermod -aG docker $USER
newgrp docker
```

### 2. Install NVIDIA Container Toolkit

```bash
# Add NVIDIA Container Toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify installation
docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi
```

### 3. Clone the Repository

```bash
# Clone with all submodules
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
cd space_robotics_bench
```

### 4. Run Space Robotics Bench

```bash
# Launch an interactive shell in the container
.docker/run.bash
```

The first run will automatically download the Docker image from Docker Hub.

## Working with Docker

### Running Specific Commands

```bash
# Run a specific script
.docker/run.bash scripts/teleop.py --env perseverance

# Run with environment variables
.docker/run.bash -e SRB_DETAIL=1.0 scripts/teleop.py --env perseverance
```

### Custom Image Building (Optional)

```bash
# Build a custom Docker image locally
.docker/build.bash
```

### Development Workflow

The Docker setup is optimized for development:

```bash
# By default, your local repository is mounted inside the container
.docker/run.bash

# Make changes in your local editor
# The changes are immediately available inside the container
# No rebuild or restart necessary!

# For deployment without source mounting:
WITH_DEV_VOLUME=false .docker/run.bash
```

### Managing Running Containers

```bash
# Connect to an already running container
.docker/join.bash
```
