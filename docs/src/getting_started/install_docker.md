# Installation — Docker (Recommended)

Using SRB inside Docker is recommended for most users, as it provides an isolated, reproducible environment that is fully pre-configured.

## 1. Clone the Repository

First, clone the SRB repository with all submodules:

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
```

## 2. Install Docker Engine & NVIDIA Container Toolkit

> 1. Official instructions: [Install Docker Engine](https://docs.docker.com/engine/install)
> 1. Official instructions: [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall)
> 1. Official instructions: [Installing the NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

Install Docker Engine and NVIDIA Container Toolkit either by following the official instructions above or using the provided convenience [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/host/install_docker.bash):

```bash
./space_robotics_bench/.docker/host/install_docker.bash
```

## 3. Run

Now, you can run the Docker container with the provided [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/run.bash). The first run will automatically pull the latest image from [Docker Hub](https://hub.docker.com/r/andrejorsula/space_robotics_bench/tags):

```bash
./space_robotics_bench/.docker/run.bash
```

## 4. Verify Installation

Once you enter the Docker container, verify that everything works as expected. If you encounter any issues, please refer to the [Troubleshooting](../misc/troubleshooting.md) guide.

### Isaac Sim

Confirm that you can launch Isaac Sim:

```bash
"$HOME/isaac-sim/isaac-sim.sh"
```

> Note: The first launch might take a while because Isaac Sim needs to compile shaders and prepare the environment.

### Space Robotics Bench

Verify that the `srb` command is available:

```bash
srb --help
```

## ... continue with [Basic Usage](basic_usage.md)

______________________________________________________________________

## Extras

### Build a New Docker Image

If you want to build a custom Docker image, you can use the provided [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/build.bash):

```bash
./space_robotics_bench/.docker/build.bash
```

### Join a Running Container

To join a running container from another terminal, use the provided [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/join.bash):

```bash
./space_robotics_bench/.docker/join.bash
```

### Development

The repository workspace is automatically mounted inside the Docker container, so you can edit the code either on the host or inside the container, and the changes will be persistently reflected in both environments.

To improve your development experience, you can open the project as a [Dev Container (guide)](../development/devcontainer.md).
