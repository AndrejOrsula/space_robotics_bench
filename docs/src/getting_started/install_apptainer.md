# Installation — Apptainer/Singularity

You can use SRB on HPC clusters or systems where Docker is not available by leveraging Apptainer. Apptainer is a containerization tool similar to Docker but designed for environments with stricter security policies or without Docker support.

## Setup (Local)

### 1. Clone the Repository

First, clone the SRB repository with all submodules:

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
```

### 2. Install Docker Engine & NVIDIA Container Toolkit

> 1. Official instructions: [Install Docker Engine](https://docs.docker.com/engine/install)
> 1. Official instructions: [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall)
> 1. Official instructions: [Installing the NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

Install Docker Engine and NVIDIA Container Toolkit either by following the official instructions above or using the provided convenience [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/host/install_docker.bash):

```bash
./space_robotics_bench/.docker/host/install_docker.bash
```

### 3. Install Apptainer

> Official instructions: [Installing Apptainer](https://apptainer.org/docs/admin/main/installation.html)

Install Apptainer by following the official instructions above. For instance, you can use these commands on Ubuntu:

```bash
sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer
```

### 4. Build the Apptainer Image

Now you can build the Apptainer image with the provided [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/hpc/build.bash):

```bash
./space_robotics_bench/.docker/hpc/build.bash
```

This will create an Apptainer image `space_robotics_bench.sif` inside the `./space_robotics_bench/.docker/hpc/images/` directory.

## Deployment (HPC Cluster)

### 1. Transfer the Repository (with the Apptainer Image)

Transfer the local SRB repository to the HPC cluster using your preferred method (e.g., `scp` or `rsync`):

#### A. `scp`

```bash
scp -r ./space_robotics_bench user@hpc-cluster:/path/to/space_robotics_bench
```

#### B. `rsync`

```bash
rsync -avr ./space_robotics_bench user@hpc-cluster:/path/to/space_robotics_bench
```

### 2. SSH into the HPC Cluster

SSH into the HPC cluster:

```bash
ssh user@hpc-cluster
```

### 3. Run the Apptainer Image

Now you can run the Apptainer image with the provided [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/hpc/run.bash) inside an interactive session (you might need make some adjustments based on your HPC environment):

```bash
# HPC interactive session
/path/to/space_robotics_bench/.docker/hpc/run.bash bash
```

### 4. Verify Installation

Once you enter the Apptainer container, verify that everything works as expected. If you encounter any issues, please refer to the [Troubleshooting](../misc/troubleshooting.md) guide.

#### Isaac Sim

Confirm that you can launch Isaac Sim:

```bash
# Inside Apptainer container
"$HOME/isaac-sim/isaac-sim.sh"
```

> Note: The first launch might take a while because Isaac Sim needs to compile shaders and prepare the environment.

#### Space Robotics Bench

Verify that the `srb` command is available:

```bash
# Inside Apptainer container
srb --help
```

## ... continue with [Basic Usage](basic_usage.md)

______________________________________________________________________

## Extras

### Schedule a SLURM Job

For long-running tasks or automated workflows, you can schedule a SLURM job that will automatically run the Apptainer image. It is highly recommended that you adjust the [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/hpc/submit.bash) to your needs before submitting the job:

```bash
# HPC login node
/path/to/space_robotics_bench/.docker/hpc/submit.bash [CMD]
```
