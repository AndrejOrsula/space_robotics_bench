# Installation: Native

This guide covers installing SRB natively on your system without containerization. Although it simplifies development, it requires more manual setup and decreases reproducibility.

## Prerequisites

Ensure your system meets the [system requirements](./requirements.md).

## Installation Steps

### 1. Clone the Repository

First, clone the SRB repository with all submodules:

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
```

### 2. Install NVIDIA Isaac Sim 4.5

> Official instructions: [Isaac Sim — Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html)

Install Isaac Sim either by following the official instructions above or using the provided convenience script:

```bash
space_robotics_bench/scripts/install_isaacsim.bash "$HOME/isaac-sim"
```

#### `ISAAC_SIM_PYTHON`

It is highly recommended to make `ISAAC_SIM_PYTHON` point to the Python entrypoint of Isaac Sim in your shell configuration (script above will prompt you to do so):

##### A. `bash`

```bash
echo "export ISAAC_SIM_PYTHON='$HOME/isaac-sim/python.sh'" >> ~/.bashrc
source ~/.bashrc
```

##### B. `fish`

```sh
set -Ux ISAAC_SIM_PYTHON '$HOME/isaac-sim/python.sh'
```

### 3. Install NVIDIA Isaac Lab 2.0

> Official instructions: [Isaac Lab — Installation](https://isaac-sim.github.io/IsaacLab/v2.0.2/source/setup/installation/binaries_installation.html#installing-isaac-lab)

Install Isaac Lab either by following the official instructions above or using the provided convenience script:

```bash
# Follow the instructions at:
# https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_lab/index.html
```

### 4. Create and Activate a Virtual Environment (Recommended)

```bash
# Create a virtual environment
python -m venv .venv

# Activate it
source .venv/bin/activate  # On Linux/macOS
```

### 5. Install Space Robotics Bench

```bash
pip install -e .[all]
```

## 6. Verify Installation

Confirm your installation is working:

```bash
# Run a simple demo
python scripts/teleop.py --env perseverance
```

## Troubleshooting Common Issues

If you encounter problems:

- **Isaac Sim doesn't launch**: Ensure NVIDIA drivers are compatible and properly installed
- **Import errors**: Check Python dependencies with `pip list` and verify paths
- **Missing assets**: Verify that submodules were cloned with `git submodule status`
- **GPU not detected**: Run `nvidia-smi` to verify GPU accessibility

For more detailed troubleshooting, refer to the [Troubleshooting Guide](../misc/troubleshooting.md).
