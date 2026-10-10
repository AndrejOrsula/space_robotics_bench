# Installation — Docker (Recommended)

Docker isolates SRB. Its source image builds the Isaac Sim and Isaac Lab versions in [system requirements](requirements.md). Headless runtime and task catalog are tested; GUI and WebRTC are untested.

## 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
cd space_robotics_bench
```

## 2. Install Docker Engine & NVIDIA Container Toolkit

Follow the official [Docker Engine](https://docs.docker.com/engine/install), [Linux post-install](https://docs.docker.com/engine/install/linux-postinstall), and [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) instructions. Interactive use needs a working host NVIDIA driver (`nvidia-smi`). GUI forwarding also needs `xauth` (`sudo apt-get install -y xauth`) and an X11 session with `DISPLAY` set. Run the pinned [installer](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/host/install_docker.bash):

```bash
./.docker/host/install_docker.bash
```

| Installer | Default / limits |
| --- | --- |
| Host | Ubuntu 22.04/24.04; amd64/arm64. Verifies Docker (and, when installing the toolkit, NVIDIA) repository key fingerprints; refuses unavailable versions. |
| Docker | Engine/CLI 29.8.0; containerd.io 2.3.6; Buildx 0.37.1; Compose 5.6.0; rootless extras 29.8.0. Override with `SRB_DOCKER_CE_VERSION`, `SRB_DOCKER_CLI_VERSION`, `SRB_DOCKER_CONTAINERD_VERSION`, `SRB_DOCKER_BUILDX_VERSION`, `SRB_DOCKER_COMPOSE_VERSION`, `SRB_DOCKER_ROOTLESS_EXTRAS_VERSION`. |
| NVIDIA Container Toolkit | 1.20.0 when a working GPU is detected. Override with `SRB_NVIDIA_TOOLKIT_VERSION`; set `SRB_INSTALL_NVIDIA_TOOLKIT=false` on CPU-only hosts. |
| Docker group | Non-interactive calls add membership only with `SRB_ADD_DOCKER_GROUP=true`. |

Start a new login session after adding the user to the `docker` group for passwordless Docker. Until then, SRB helpers use `sudo docker ...` as needed.

## 3. Run

Build the image and start the [runner](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/run.bash):

```bash
SRB_IMAGE_NAME=space_robotics_bench ./.docker/build.bash 6.1.0
export SRB_DOCKER_IMAGE='space_robotics_bench:6.1.0'
./.docker/run.bash
```

Local tags resolve to immutable image IDs; missing tags fail without remote fallback. Default `safe` is interactive, isolates networking/IPC, drops capabilities, and has no host devices or mounts. Use it for lightweight commands without Isaac Sim:

```bash
DOCKER_TTY=false ./.docker/run.bash srb --help
```

For remote releases, provide the complete registry digest; mutable tags are rejected:

```bash
SRB_DOCKER_IMAGE='<registry>/<image>@sha256:<64-hex-digest>' ./.docker/run.bash srb --help
```

Keep the image reference consistent across `run.bash`, `dev.bash`, and any join/HPC wrapper used together.

Isaac Sim requires GPU access and host networking for ROS 2 discovery and livestreaming. When an NVIDIA GPU is detected or force-enabled, `simulator` exposes all host-visible NVIDIA GPUs; the runner has no per-GPU selector. Select `simulator`:

```bash
SRB_DOCKER_PROFILE=simulator ./.docker/run.bash
```

Unprivileged `simulator` enables `no-new-privileges`, NVIDIA runtime when an NVIDIA GPU is detected or force-enabled, `/dev/dri` when available on the host, host networking/IPC, writable simulator caches, GUI forwarding/history, and writable `/root/ws`; it neither uses `--privileged` nor mounts full `/dev`. Disable host networking or use private IPC with explicit shared memory (default 8g):

```bash
# Bridge networking (no ROS 2 multicast discovery with host nodes)
SRB_DOCKER_PROFILE=simulator WITH_HOST_NETWORK=false ./.docker/run.bash

# Private IPC namespace with an explicit /dev/shm size (default 8g)
SRB_DOCKER_PROFILE=simulator WITH_HOST_IPC=false SRB_DOCKER_SHM_SIZE=16g ./.docker/run.bash
```

`trusted-simulator` enables `--privileged`, mounts all `/dev`, and prints a security warning. Use only with a locally trusted image or authenticated digest for hardware passthrough (cameras, serial adapters, joysticks):

```bash
SRB_DOCKER_PROFILE=trusted-simulator ./.docker/run.bash
```

Useful variants:

```bash
# Headless simulator session
SRB_DOCKER_PROFILE=simulator WITH_GUI=false ./.docker/run.bash

# Without GPU and GUI, for commands that do not launch Isaac Sim
SRB_DOCKER_PROFILE=simulator WITH_GPU=false WITH_GUI=false ./.docker/run.bash

# Simulator without writable source or shell history
SRB_DOCKER_PROFILE=simulator WITH_DEV_VOLUME=false WITH_HISTORY=false ./.docker/run.bash

# Force the NVIDIA runtime when GPU detection fails (diagnostics only)
SRB_DOCKER_PROFILE=simulator WITH_GPU_FORCE_NVIDIA=true ./.docker/run.bash

# Non-interactive command execution, for CI and scripts
DOCKER_TTY=false ./.docker/run.bash bash -lc "srb --help"
```

Pass variables with `-e`; bare `-e NAME` forwards secrets. Exported caller variables use a private temporary env file hidden from Docker's command line and `ps`. Invalid entries exit 2: empty names, whitespace or leading `#` in names, invalid bare-name identifiers, or newline values. Errors expose only entry position and length. Docker applies `-e` before the entrypoint adds `PYTHONPATH`, `LD_LIBRARY_PATH`, `PATH`, and ROS entries; override them inside the command.

While the container runs, join from another terminal:

```bash
./.docker/join.bash
```

Both scripts accept a trailing command:

```bash
SRB_DOCKER_PROFILE=simulator ./.docker/run.bash bash
./.docker/join.bash 0 fish
```

## 4. Verify Installation

Verify inside the container; see [Troubleshooting](../misc/troubleshooting.md) for help.

### Isaac Sim

If using default `safe`, restart from the host with `SRB_DOCKER_PROFILE=simulator ./.docker/run.bash`, check GPU visibility, then launch Isaac Sim:

```bash
nvidia-smi
```

```bash
"$HOME/isaac-sim/isaac-sim.sh"
```

The first launch may take longer during shader compilation and environment setup.

### Space Robotics Bench

```bash
srb --help
srb cache update
```

`srb cache update` launches Isaac Sim and builds the offline task catalog.

### GUI Forwarding

```bash
echo "$DISPLAY"
test -n "$XAUTHORITY" && ls -l "$XAUTHORITY"
```

## Extras

### Build a New Docker Image

Build a custom image with the [script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/build.bash):

```bash
./.docker/build.bash
```

Build `RUN` steps use bridge networking and daemon DNS by default.

**Symptom:** `Could not resolve host: github.com` or `uv` reports `operation timed out` during a build.

**Cause:** VPN-only, split-horizon, or loopback stub DNS (such as `systemd-resolved` at `127.0.0.53`) can block bridge DNS.

**Fix:** Auto mode probes bridge DNS and uses `--network host` with a warning only when lookup completes without a matching host; other failures stop. Set `SRB_BUILD_NETWORK` to any `docker build --network` value:

```bash
SRB_BUILD_NETWORK=default ./.docker/build.bash  # keep the bridge, fail loudly
SRB_BUILD_NETWORK=host ./.docker/build.bash     # skip the probe, always host
```

Forwarded `--network` takes precedence. For a durable fix, set reachable `"dns"` in `/etc/docker/daemon.json` and restart Docker.

Isaac Lab 3.0 pins runtime deps in root `pyproject.toml`; extensions declare none. Builds install extensions editable, then root deps via `.docker/helpers/isaaclab_root_requirements.py` (`requirements`/`overrides`). Documented omissions: Newton viewer (SRB is PhysX-only; PhysX articulations import `newton`); Isaac Lab's `transformers` pin (SRB lock selects another via `robomimic`); `rsl-rl-lib` (already locked). `RSL_RL_DEV=true` replaces the locked library with an editable checkout at `RSL_RL_COMMIT_SHA` (off by default).

### Image Dependencies

Dockerfile exports committed `uv.lock` with `--frozen --all-extras --group image-runtime --group vcs-sources`, then installs the editable project with `--no-deps`. `image-runtime` locks shared simulator/framework deps; `vcs-sources` pins Git robomimic, v2e, and e2vid, which PyPI metadata rejects. Neither is a published SRB extra. Registry artifacts retain lock hashes; Git sources use full commit refs. By default, `ROBOMIMIC_DEV=true` omits robomimic for the patched checkout; pinned checkouts are explicit build inputs. Arm64 omits unavailable `usd-core` and uses bundled USD.

The robomimic checkout lazily patches `e10526b9a40c78b41f1e37e60041dc0ec0a5f60f`; builds verify `9e88eb3288079c6014f912e8d805c76dcd40157a`. Low-dimensional dataset imports/learning need no CLIP weights; language observations need the model and tokenizer.

After changing Isaac Sim or Isaac Lab pins, rebuild and remove old-pin caches with [`srb clean`](../development/clean_cache.md).

### Optional NVIDIA NGC Isaac Sim Image

The `ngc` profile runs NVIDIA's official Isaac Sim 6.1 image (`nvcr.io/nvidia/isaac-sim`), pinned to an architecture-specific digest:

```bash
SRB_DOCKER_PROFILE=ngc SRB_NGC_MODE=headless ./.docker/run.bash
```

Modes are `SRB_NGC_MODE=headless` (default), `webrtc`, and `gui`, with `ngc-headless`, `ngc-webrtc`, and `ngc-gui` shorthand profiles. The container runs as UID `1234:1234`; Isaac Sim is at `/isaac-sim`, and the checkout is at `/workspace`. `ACCEPT_EULA=Y` accepts NVIDIA's license unless overridden. Writable caches use `SRB_NGC_CACHE_DIR` (default `~/.cache/space_robotics_bench/ngc/isaac-sim-6.1/<arch>`). WebRTC needs host networking; GUI mode needs a real display. SRB tasks, ROS 2, and Rust are untested.

NGC always requires an RTX-class GPU; other profiles opt in with `SRB_REQUIRE_RTX=true`. Preflight checks NVIDIA hardware ray tracing with `vulkaninfo` (`VK_KHR_ray_tracing_pipeline`, `VK_KHR_acceleration_structure`), not product names: L40S, A40, GB10 qualify; GTX/software devices do not. Errors distinguish missing/failed `vulkaninfo`, no device, or no ray-tracing device. GUI merges X credentials to user-owned `${TMPDIR:-/tmp}/srb-xauth-<project>-<user>/Xauthority`.

### Join a Running Container

Use the [join script](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/join.bash):

```bash
./.docker/join.bash
```

For multiple containers, choose a listed index. Set `SRB_CONTAINER_NAME` as for `run.bash`; only `<name>` and `<name><digits>` appear, excluding NGC. Set `DOCKER_TTY=false` in CI/scripts to avoid `docker exec --tty` failing with `the input device is not a TTY`.

```bash
DOCKER_TTY=false ./.docker/join.bash 0 bash -lc "srb --help"
```

### Development

Simulator, trusted-simulator, and development profiles mount the workspace, so edits persist on the host; `safe` does not. For development, open a [Dev Container](../development/devcontainer.md).

Use `./.docker/dev.bash` for development variables or bind mounts. It wraps `run.bash` with unprivileged `simulator`; `SRB_DOCKER_PROFILE=trusted-simulator` adds `--privileged` and `/dev` (security warning). `LOG_LEVEL` defaults to `info`; set `LOG_LEVEL=debug ./.docker/dev.bash ...` for verbose Isaac/SimForge startup logs.

The wrapper shell places mounted `/root/ws` before the ROS overlay on `PYTHONPATH`, so `srb` subprocesses find checkout modules without rebuilding. On exit, created checkout files (such as `.cache/`, `logs/`, `outputs/`) return to the host user. `SRB_DOCKER_DEV_WRAPPER=false` runs commands as given.

Rust caches persist across containers. Host Cargo registry, Git, and target caches default to `~/.cache/space_robotics_bench/docker`. Container builds use `CARGO_TARGET_DIR=/root/.cache/srb/cargo-target`, preserving image artifacts in `/root/.cargo/target`. Override the host cache with `SRB_DOCKER_CACHE_DIR=/path/to/cache ./.docker/dev.bash ...` or the container target with `SRB_CARGO_TARGET_DIR=/path/in/container ./.docker/dev.bash ...`; do not set the latter to `/root/.cargo/target`.

Next: [Basic Usage](basic_usage.md)
