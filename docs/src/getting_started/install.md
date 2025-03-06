# Installation

Before proceeding, ensure your system meets the [system requirements](./requirements.md).

## Methods

Space Robotics Bench supports three installation methods that can be used interchangeable.

**A. [Native](./install_native.md)**
- ✅ Full system integration
- ✅ Best development experience
- ❗ Complex manual setup
- ❗ No CLI argument completion

**B. [Docker — Recommended](./install_docker.md)**
- ✅ Simple installation & deployment
- ✅ Reproducible & faster to debug
- ✅ User-friendly interface (via custom scripts)
- ✅ Okay-ish development experience (via Dev Containers)
- ❗ Requires privileged access (no HPC)

**C. [Apptainer/Singularity](./install_apptainer.md)**
- ✅ Deployable to HPC clusters
- ❗ Uff...

## Quickstart (Temporary Setup)

<div class="warning">
Quickstart requires Debian/Ubuntu
</div>

If you are unsure about using Space Robotics Bench but you consider giving it a try, you can get started with the *quickstart* approach! This temporary leverages pre-built Docker images and a helper script to automatically configure



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
