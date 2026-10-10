# Installation — Apptainer/Singularity

Build the `.sif` from the Docker image, which pins Isaac Sim and Lab ([requirements](requirements.md)), then transfer it to the cluster.

## Setup (Local)

<div class="warning">

Apptainer installs are untested.

</div>

### 1. Clone the repository

Clone from the repository root:

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
cd space_robotics_bench
```

### 2. Install Docker Engine and NVIDIA Container Toolkit

Install [Docker Engine](https://docs.docker.com/engine/install), follow its [Linux post-install steps](https://docs.docker.com/engine/install/linux-postinstall), and install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). Or run the [pinned helper](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/host/install_docker.bash):

```bash
./.docker/host/install_docker.bash
```

The helper supports Ubuntu 22.04/24.04 amd64/arm64, verifies key fingerprints, and pins Docker Engine/CLI 29.8.0 and optional NVIDIA Container Toolkit 1.20.0. Docker's guide covers version overrides and non-interactive group behavior.

### 3. Install Apptainer

Follow [Apptainer installation instructions](https://apptainer.org/docs/admin/main/installation.html) or install on Ubuntu:

```bash
sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer
```

### 4. Build the Apptainer Image

Run the [build script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/hpc/build.bash):

```bash
./.docker/hpc/build.bash 6.1.0
```

This builds a tagged Docker image and converts it to `.docker/hpc/images/space_robotics_bench.sif`.

### 5. Record image details

Before transfer, record its source digest and architecture, plus SIF metadata and checksum:

```bash
docker image inspect --format '{{.Id}} {{.Architecture}}' space_robotics_bench:6.1.0 # prefix '<user>/' if `docker info` lists a Username
apptainer inspect .docker/hpc/images/space_robotics_bench.sif
sha256sum .docker/hpc/images/space_robotics_bench.sif
```

A `docker info` username prefixes the image tag: `<username>/space_robotics_bench:6.1.0`.

## Deployment (HPC Cluster)

### 1. Transfer repository and image

Choose one method:

```bash
scp -r /path/to/space_robotics_bench user@hpc-cluster:/path/to/space_robotics_bench
```

```bash
rsync -avr /path/to/space_robotics_bench user@hpc-cluster:/path/to/space_robotics_bench
```

### 2. Connect to the cluster

```bash
ssh user@hpc-cluster
```

### 3. Run the image

On a `module` node, adjust [run.bash](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/hpc/run.bash). The script loads `tools/Singularity` and calls `singularity`. Because `singularity exec` skips the Docker entrypoint, the script sources `/entrypoint.bash` (ROS Jazzy).

```bash
/path/to/space_robotics_bench/.docker/hpc/run.bash bash
```

| Setting | Behavior |
| --- | --- |
| Mount / caches | Source image: checkout at `/root/ws`; caches under `${SCRATCH:-$HOME}/volumes/space_robotics_bench/`. NGC image: checkout at `/workspace` (override with `SRB_NGC_WORKSPACE`); caches under `${SCRATCH:-$HOME}/volumes/space_robotics_bench/ngc/isaac-sim-6.1/<arch>` (override with `SRB_NGC_CACHE_DIR`). |
| `WITH_GPU` | Defaults `true` (`--nv`); `false` omits passthrough but keeps Isaac Sim in the image. |
| Command | Arguments follow `run.bash`; `bash` opens an interactive shell. |

For NGC images:

| Setting | Behavior |
| --- | --- |
| `IMAGE_PATH` | Required for NGC; set it when `.sif` is outside `.docker/hpc/images/`. |
| Provenance | Checks the recorded SIF checksum on each run; large files may take longer. |
| `SRB_NGC_CACHE_OWNERSHIP` | `user` (default) or `skip`; the invoking user runs Singularity and owns caches. |
| Paths, digest, and RTX | Sets `ISAACSIM_PATH=/isaac-sim`; requires an NGC SIF via `IMAGE_PATH` and checks its recorded digest and checksum. The RTX check matches Docker NGC preflight. |

Use the target architecture's `<arch digest>` for both NGC commands. From the repository root:

```bash
SRB_DOCKER_PROFILE=ngc IMAGE_PATH=X.sif ./.docker/hpc/run.bash bash
```

```bash
apptainer build X.sif docker://nvcr.io/nvidia/isaac-sim@sha256:<arch digest>
ref='nvcr.io/nvidia/isaac-sim@sha256:<arch digest>'
printf 'source=%s\nsif_sha256=%s\n' "$ref" "$(sha256sum X.sif | cut -d' ' -f1)" > X.sif.ngc-source
```

```bash
/path/to/space_robotics_bench/.docker/hpc/run.bash bash -lc '
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
printf "%s\n" "$ROS_DISTRO" "$AMENT_PREFIX_PATH" "$LD_LIBRARY_PATH" "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID"
srb agent ros --headless -e peg_in_hole
'
```

HPC GUI and WebRTC sessions are untested.

### 4. Verify Installation

In the container, see [Troubleshooting](../misc/troubleshooting.md).

#### Isaac Sim

```bash
nvidia-smi
"${ISAACSIM_PATH:-$HOME/isaac-sim}/isaac-sim.sh"
```

Initial launch may take longer because Isaac Sim compiles shaders and prepares the environment.

#### Space Robotics Bench

Check the SRB cache and interpreter:

```bash
srb --help
srb cache update
"${ISAACSIM_PATH:-$HOME/isaac-sim}/python.sh" -c "import isaacsim; print(isaacsim.__file__)"
"${ISAACSIM_PATH:-$HOME/isaac-sim}/python.sh" -c "import srb; print(srb.__file__)"
srb --help
```

`isaacsim` and `srb` paths must share an installation; the last command runs the `srb` found on `PATH`. SRB has no `--version` command.

## Extras

### Schedule a SLURM Job

Adjust the [cluster SLURM script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/hpc/submit.bash):

```bash
/path/to/space_robotics_bench/.docker/hpc/submit.bash [CMD]
```

It writes under `$HOME/jobs/` by default and submits with `sbatch`. Review the partition, constraint, memory, wall time, and GPU settings.

Next: [Basic Usage](basic_usage.md)
