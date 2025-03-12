# Installation

Before proceeding, ensure your system meets the [system requirements](./requirements.md).

## Installation Methods

SRB supports three installation methods, each with different trade-offs:

### A. [Native](./install_native.md)

- ✅ Full system integration
- ✅ Smooth development experience
- ❗ Complex setup process
- ❗ Potential dependency conflicts

### B. [Docker — Recommended](./install_docker.md)

- ✅ Simple installation & deployment
- ✅ Reproducible environment & easy to update
- ⚠️ Moderate development experience (via Dev Containers)
- ❗ Requires privileged access (not suitable for HPC)

### C. [Apptainer/Singularity](./install_apptainer.md)

- ✅ Deployable to HPC clusters
- ❗ Uff...

## Alternative — Temporary Setup (Quickstart)

For quick experimentation with SRB, you can use a temporary setup that downloads a pre-built Docker image and runs it in a pre-configured container. A single [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/run.bash) accomplishes everything, which you can call directly via [`curl`](https://curl.se) or [`wget`](https://www.gnu.org/software/wget) (consider inspecting the script before executing it):

#### A. `curl`

```bash
ENSURE_DOCKER=true WITH_DEV_VOLUME=false WITH_HISTORY=false bash -c "$(curl -fsSL https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/run.bash)" --
```

#### B. `wget`

```bash
ENSURE_DOCKER=true WITH_DEV_VOLUME=false WITH_HISTORY=false bash -c "$(wget -qO - https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/run.bash)" --
```

<div class="warning">

The Docker container created by this setup is ephemeral (`WITH_DEV_VOLUME=false`), and data is not persisted between sessions. **Any changes made inside the container will be lost when the container is removed.**

</div>

### Cleanup of Temporary Setup

If you do not wish to continue using SRB, you can remove the Docker container and its associated image by executing these commands:

```bash
docker rm -f space_robotics_bench
docker rmi andrejorsula/space_robotics_bench:latest
```
