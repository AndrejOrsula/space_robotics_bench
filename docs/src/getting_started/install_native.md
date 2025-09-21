# Installation — Native

This guide covers installing SRB natively on your system without containerization. Although this approach simplifies development, it requires more manual setup and decreases reproducibility.

## 1. Clone the Repository

First, clone the SRB repository with all submodules:

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
```

## 2. Install NVIDIA Isaac Sim 5.0

> Official instructions: [Isaac Sim — Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_workstation.html)

Install Isaac Sim either by following the official instructions above or using the provided convenience [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/install_isaacsim.bash):

```bash
./space_robotics_bench/scripts/install_isaacsim.bash "$HOME/isaac-sim"
```

### `ISAAC_SIM_PYTHON`

It is highly recommended to make `ISAAC_SIM_PYTHON` point to the Python entrypoint of Isaac Sim in your shell configuration (the script above will prompt you to do so):

##### A. `bash`

```bash
echo "export ISAAC_SIM_PYTHON='$HOME/isaac-sim/python.sh'" >> ~/.bashrc
source ~/.bashrc
```

##### B. `zsh`

```sh
echo "export ISAAC_SIM_PYTHON='$HOME/isaac-sim/python.sh'" >> ~/.zshrc
source ~/.zshrc
```

##### C. `fish`

```sh
set -Ux ISAAC_SIM_PYTHON "$HOME/isaac-sim/python.sh"
```

## 3. Install NVIDIA Isaac Lab 2.2

> Official instructions: [Isaac Lab — Installation](https://isaac-sim.github.io/IsaacLab/v2.2.1/source/setup/installation/binaries_installation.html#installing-isaac-lab)

Install Isaac Lab either by following the official instructions above or using the provided convenience [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/install_isaaclab.bash):

```bash
./space_robotics_bench/scripts/install_isaaclab.bash "$HOME/isaaclab"
```

## 4. Install Blender 4.3 with SimForge

> Official instructions: [Blender — Install from blender.org](https://docs.blender.org/manual/en/4.3/getting_started/installing/linux.html#install-from-blender-org)

1. Install Blender by following the official instructions for downloading and extracting its archive.
1. Ensure that the `blender` executable is accessible from your system's `PATH`.
1. Install SimForge with its assets within the Blender Python environment.

As an example, the commands below will install Blender in your home directory and create a symbolic link to `$HOME/.local/bin/blender` (assuming it is in your `PATH`):

```bash
export BLENDER_VERSION="4.3.2"
export BLENDER_VERSION_SHORT=$(echo $BLENDER_VERSION | sed 's/\.[^.]*$//')
mkdir -p $HOME/blender
curl -fsSL "https://download.blender.org/release/Blender$BLENDER_VERSION_SHORT/blender-$BLENDER_VERSION-linux-x64.tar.xz" | tar xJ -C $HOME/blender --strip-components=1
ln -sf $HOME/blender/blender $HOME/.local/bin/blender
"$HOME/blender/$BLENDER_VERSION_SHORT/python/bin/python3.11" -m pip install simforge[assets]
```

<div class="warning">

Avoid installing Blender through Snap, as it would prevent the integration of the required Python dependencies within its environment.

</div>

## 5. Install the Space Robotics Bench

Install the `srb` package in editable mode:

```bash
"$ISAAC_SIM_PYTHON" -m pip install --editable ./space_robotics_bench[all]
```

> **Note**: The `all` extra installs optional dependencies to support all workflows and improve usability. Feel free to check [`pyproject.toml`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/pyproject.toml) and adjust the extras to your needs.

### Setup CLI

Make the `srb` CLI command available in your shell through the provided [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/setup_cli.bash):

```bash
./space_robotics_bench/scripts/setup_cli.bash
```

> **Note:** In case the script fails, the `srb` CLI is still accessible via `"$ISAAC_SIM_PYTHON" -m srb`.

## 6. Verify Installation

After the installation, verify that everything works as expected. If you encounter any issues, please refer to the [Troubleshooting](../misc/troubleshooting.md) guide.

### Isaac Sim

Confirm that you can launch Isaac Sim:

```bash
"$HOME/isaac-sim/isaac-sim.sh"
```

> Note: The first launch might take a while because Isaac Sim needs to compile shaders and prepare the environment.

### Isaac Lab

Confirm that Isaac Lab is installed:

```bash
"$ISAAC_SIM_PYTHON" -m pip show isaaclab
```

### Space Robotics Bench

Verify that the entrypoint script of SRB is available in the Python environment of Isaac Sim:

```bash
"$ISAAC_SIM_PYTHON" -m srb --help
```

Verify that the `srb` command is available:

```bash
srb --help
```

Verify that argument completion works:

```bash
srb <TAB> <TAB>
```

## ... continue with [Basic Usage](./basic_usage.md)

______________________________________________________________________

## Extras

### Development

To improve your development experience, consider [configuring your IDE (guide)](../development/ide.md).
