# Installation

Before proceeding, ensure your system meets the [system requirements](./requirements.md).

## Installation Methods

The Space Robotics Bench supports three installation methods that can be used interchangeably.

**A. [Native](./install_native.md)**

- ✅ Full system integration
- ✅ Smooth development experience
- ⚠️ No CLI argument completion
- ❗ Complex setup

**B. [Docker — Recommended](./install_docker.md)**

- ✅ Simple installation & deployment
- ✅ Reproducible & easy to update
- ✅ User-friendly interface (via custom scripts)
- ⚠️ Okay-ish development experience (via Dev Containers)
- ❗ Requires privileged access (no HPC)

**C. [Apptainer/Singularity](./install_apptainer.md)**

- ✅ Deployable to HPC clusters
- ❗ Uff...

## Temporary Setup (Quick Start)

Alternatively, you can quickly experiment with the Space Robotics Bench via a temporary setup using a one-liner command. This command executes a script that pulls a pre-built Docker image and runs it in a pre-configured container. Note that the container is ephemeral and internal data is not persisted between sessions.

<div class="warning">
Consider inspecting the <a href="https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/run.bash" target="_blank"><code>.docker/run.bash</code> script</a> before executing it with <code>curl</code> or <code>wget</code>.
</div>

**A. Using [curl](https://curl.se)** (same as `wget`)

```bash
WITH_DEV_VOLUME=false WITH_HISTORY=false bash -c "$(curl -fsSL https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/run.bash)"
```

**B. Using [wget](https://www.gnu.org/software/wget)** (same as `curl`)

```bash
WITH_DEV_VOLUME=false WITH_HISTORY=false bash -c "$(wget -qO - https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/run.bash)"
```
