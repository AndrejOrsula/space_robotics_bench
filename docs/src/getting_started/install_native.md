# Installation: Native

This guide covers installing Space Robotics Bench directly on your system without containerization. While this approach offers maximum performance, it requires more manual setup than the Docker method.

## Why Choose Native Installation?

- **Performance**: Direct hardware access without container overhead
- **Integration**: Seamless integration with other locally installed tools
- **Development**: Easier debugging and profiling
- **Customization**: More control over the environment

## Prerequisites

Ensure your system meets the [system requirements](./requirements.md) and has:

- NVIDIA GPU with RT Cores (RTX series)
- Compatible NVIDIA drivers
- Python 3.8+ with pip and venv

## Installation Steps

### 1. Install NVIDIA Isaac Sim 4.5

```bash
# Option A: Use our convenience script
./scripts/install_isaac_sim.bash

# Option B: Manual installation via NVIDIA Omniverse Launcher
# Download from: https://www.nvidia.com/en-us/omniverse/download/
```

### 2. Install NVIDIA Isaac Lab 2.0

Isaac Lab provides robotics-specific extensions for Isaac Sim:

```bash
# Follow the instructions at:
# https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_lab/index.html
```

### 3. Clone the Repository

```bash
# Clone with all submodules (necessary for simulation assets)
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
cd space_robotics_bench
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
