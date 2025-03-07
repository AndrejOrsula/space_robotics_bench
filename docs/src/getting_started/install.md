# Installation

Before proceeding, ensure your system meets the [system requirements](./requirements.md).

## Installation Methods

SRB supports three installation methods, each with different trade-offs:

#### A. [Native](./install_native.md)

- ✅ Full system integration
- ✅ Smooth development experience
- ⚠️ No CLI argument completion
- ❗ Complex setup
- ❗ Potential conflicts

#### B. [Docker — Recommended](./install_docker.md)

- ✅ Simple installation & deployment
- ✅ Reproducible & easy to update
- ✅ User-friendly interface (via custom scripts)
- ⚠️ Moderate development experience (via Dev Containers)
- ❗ Requires privileged access (no HPC)

#### C. [Apptainer/Singularity](./install_apptainer.md)

- ✅ Deployable to HPC clusters
- ❗ Uff...

## Temporary Setup (Quickstart)

For quick experimentation with SRB, you can use a temporary setup that downloads a pre-built Docker image and runs it in a pre-configured container. Everything is accomplished by a single script that you can directly call via [`curl`](https://curl.se) or [`wget`](https://www.gnu.org/software/wget):

<div class="warning">
Consider inspecting the <a href="https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/run.bash" target="_blank"><code>.docker/run.bash</code> script</a> first before executing it.
</div>

#### A. `curl`

```bash
WITH_DEV_VOLUME=false WITH_HISTORY=false bash -c "$(curl -fsSL https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/run.bash)" --
```

#### B. `wget`

```bash
WITH_DEV_VOLUME=false WITH_HISTORY=false bash -c "$(wget -qO - https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/run.bash)" --
```

<div class="warning">
The Docker container created by this setup is ephemeral, and data is not persisted between sessions. <strong>Any changes made inside the container will be lost when the container is removed.</strong>
</div>

## Cleanup of Temporary Setup

If you do not wish to continue using SRB, you can remove the Docker container and its associated image by executing the following commands:

```bash
docker rm -f space_robotics_bench
docker rmi andrejorsula/space_robotics_bench:latest
```
