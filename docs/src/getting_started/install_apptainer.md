# Installation: Apptainer (Singularity)

This guide explains how to use Space Robotics Bench with Apptainer (formerly Singularity), primarily intended for HPC environments where Docker isn't available.

## When to Use Apptainer

- **HPC Clusters**: Most high-performance computing clusters don't allow Docker but support Apptainer
- **Security Requirements**: Environments with stricter security policies may prefer Apptainer
- **SLURM Integration**: When running jobs through workload managers like SLURM

## Prerequisites

- Access to a system with Apptainer installed (common on HPC clusters)
- NVIDIA GPU with RT Cores
- Appropriate NVIDIA drivers

## Installation Steps

### 1. Install Apptainer (if needed)

If Apptainer isn't already available on your system:

```bash
# For Ubuntu/Debian systems
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer
```

For other distributions or non-admin installations, refer to the [official Apptainer documentation](https://apptainer.org/docs/admin/main/installation.html).

### 2. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
cd space_robotics_bench
```

### 3. Workflow on HPC Systems

Space Robotics Bench includes specialized scripts for HPC environments under `.docker/hpc/`:

#### Building the Container

```bash
# Convert the Docker image to an Apptainer SIF file
.docker/hpc/build.bash
```

#### Interactive Usage

```bash
# Run the container interactively
.docker/hpc/run.bash

# Run a specific script
.docker/hpc/run.bash scripts/teleop.py --env perseverance
```

#### SLURM Batch Jobs

For non-interactive batch processing on SLURM clusters:

```bash
# Submit a basic job
.docker/hpc/submit.bash scripts/train.py --env titan

# Submit with custom SLURM parameters
.docker/hpc/submit.bash scripts/train.py --env titan \
  --time=8:00:00 \
  --partition=gpu \
  --gpus=1 \
  --mem=16G \
  --cpus-per-task=4
```

## HPC-Specific Considerations

### Storage Management

HPC environments often have different storage locations:

```bash
# Use specific temporary directory (often needed for large files)
.docker/hpc/run.bash --tmp=$SCRATCH/tmp scripts/teleop.py --env perseverance

# Use specific output directory
.docker/hpc/run.bash --output=$SCRATCH/outputs scripts/train.py --env titan
```

### Binding Directories

You may need to mount additional directories:

```bash
# Bind additional directories
.docker/hpc/run.bash --bind=/path/on/host:/path/in/container scripts/teleop.py
```

### Performance Optimization

For best performance on HPC systems:

- Request sufficient resources in SLURM scripts
- Use local scratch storage for temporary files
- Consider using MPI for distributed training when available
- Set appropriate environment variables for GPU optimization
