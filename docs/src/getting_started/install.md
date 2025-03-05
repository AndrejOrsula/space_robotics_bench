# Installation

Before proceeding, ensure your system meets the [system requirements](./requirements.md).

## Installation Methods

Space Robotics Bench offers three installation approaches:

- **[Docker](./install_docker.md)** (**Recommended**):

  - ✅ Easiest setup process
  - ✅ Consistent environment across systems
  - ✅ No dependency conflicts
  - ✅ Includes all required components

- **[Native Installation](./install_native.md)**:

  - ✅ Maximum performance
  - ✅ Full system integration
  - ❗ More complex setup
  - ❗ Requires manual installation of Isaac Sim

- **[Apptainer/Singularity](./install_apptainer.md)**:

  - ✅ Works on HPC clusters
  - ✅ Compatible with SLURM workflows
  - ❗ Requires additional setup steps

## Quick Testing with Docker

Try Space Robotics Bench immediately without repository cloning:

```bash
# Start an interactive shell with the latest version
bash -c "$(curl -fsSL https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/main/.docker/run.bash)"
```

```bash
# Launch directly into the Perseverance rover demo
bash -c "$(curl -fsSL https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/main/.docker/run.bash)" -- \
  latest scripts/teleop.py --env perseverance
```

These commands will automatically:

1. Pull the pre-built Docker image
1. Configure GPU access and display forwarding
1. Start the container with appropriate environment variables
1. Run your specified command or launch an interactive shell

For ongoing development, we recommend cloning the repository as described in the [Docker installation](./install_docker.md) guide.
