# Installation: Native

This guide covers installing SRB natively on your system without containerization. Although it simplifies development, it requires more manual setup and decreases reproducibility.

## 1. Clone the Repository

First, clone the SRB repository with all submodules:

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
```

## 2. Install NVIDIA Isaac Sim 4.5

> Official instructions: [Isaac Sim — Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html)

Install Isaac Sim either by following the official instructions above or using the provided convenience script:

```bash
./space_robotics_bench/scripts/install_isaacsim.bash "$HOME/isaac-sim"
```

### `ISAAC_SIM_PYTHON`

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

## 3. Install NVIDIA Isaac Lab 2.0

> Official instructions: [Isaac Lab — Installation](https://isaac-sim.github.io/IsaacLab/v2.0.2/source/setup/installation/binaries_installation.html#installing-isaac-lab)

Install Isaac Lab either by following the official instructions above or using the provided convenience script:

```bash
./space_robotics_bench/scripts/install_isaaclab.bash "$HOME/isaaclab"
```

## 4. Install the Space Robotics Bench

Install the `srb` package in editable mode:

```bash
"$ISAAC_SIM_PYTHON" -m pip install --editable ./space_robotics_bench[all]
```

> **Note**: The `all` extra installs optional dependencies to support all workflows and improve usability. Feel free to check [`pyproject.toml`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/pyproject.toml) and adjust the extras to your needs.

## 5. Verify Installation

### Isaac Sim

Confirm that you can launch Isaac Sim:

```bash
"$HOME/isaac-sim/isaac-sim.sh"
```

### Isaac Lab

Confirm that Isaac Lab is installed:

```bash
"$ISAAC_SIM_PYTHON" -m pip show isaaclab
```

### Space Robotics Bench

Verify that the `srb` command is available:

```bash
"$ISAAC_SIM_PYTHON" -m srb --help
```

## ... continue with [Basic Usage](./basic_usage.md)
