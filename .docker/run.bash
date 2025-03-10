#!/usr/bin/env bash
### Run a Docker container
### Usage: run.bash [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE] [TAG] [CMD]
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"

## If the current user is not in the docker group, all docker commands will be run as root
if ! grep -qi /etc/group -e "docker.*${USER}"; then
    echo "[INFO] The current user '${USER}' is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO="sudo"
fi

## Config
# Name of the Docker image to run if an image with locally-defined name does not exist
DOCKERHUB_IMAGE_NAME="${DOCKERHUB_IMAGE_NAME:-"andrejorsula/space_robotics_bench"}"
# Options for running the container
DOCKER_RUN_OPTS="${DOCKER_RUN_OPTS:-
    --interactive
    --tty
    --rm
    --network host
    --ipc host
    --privileged
}"
# Flag to enable GPU
WITH_GPU="${WITH_GPU:-true}"
# Flag to enable GUI (X11)
WITH_GUI="${WITH_GUI:-true}"
# Flag to enable mounting the command history as a volume
WITH_HISTORY="${WITH_HISTORY:-true}"
# Flag to enable mounting the source code as a volume
WITH_DEV_VOLUME="${WITH_DEV_VOLUME:-true}"
# Flag to install Docker if it is not available
ENSURE_DOCKER="${ENSURE_DOCKER:-false}"
# Volumes to mount inside the container
DOCKER_VOLUMES=(
    ## Common
    # Time
    "/etc/localtime:/etc/localtime:ro"
    "/etc/timezone:/etc/timezone:ro"
    # Hot-pluggable IO devices
    "/dev:/dev"
    ## Isaac Sim
    # Data
    "${HOME}/.nvidia-omniverse/data/isaac-sim:/root/isaac-sim/kit/data:rw"
    # Cache
    "${HOME}/.cache/isaac-sim:/root/isaac-sim/kit/cache:rw"
    "${HOME}/.cache/nvidia/GLCache:/root/.cache/nvidia/GLCache:rw"
    "${HOME}/.cache/ov:/root/.cache/ov:rw"
    "${HOME}/.nv/ComputeCache:/root/.nv/ComputeCache:rw"
    # Logs
    "${HOME}/.nvidia-omniverse/logs:/root/.nvidia-omniverse/logs:rw"
    "${HOME}/.nvidia-omniverse/logs/isaac-sim:/root/isaac-sim/kit/logs:rw"
    ## SimForge
    # Cache
    "${HOME}/.cache/simforge:/root/.cache/simforge:rw"
)
# Environment variables to set inside the container
DOCKER_ENVIRON=(
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-"0"}"
    ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-"1"}"
    RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-"rmw_cyclonedds_cpp"}"
)

if [[ "${WITH_HISTORY,,}" = true ]]; then
    DOCKER_VOLUMES+=(
        "${SCRIPT_DIR}/.history:/.history:rw"
    )
    DOCKER_ENVIRON+=(
        HISTFILE="/.history/.bash_history"
        PYTHON_HISTORY="/.history/.python_history"
    )
fi
if [[ "${WITH_DEV_VOLUME,,}" = true ]]; then
    DOCKER_VOLUMES+=(
        "${REPOSITORY_DIR}:/root/ws:rw"
    )
fi

## DDS config
if [[ "${RMW_IMPLEMENTATION:-"rmw_cyclonedds_cpp"}" = "rmw_cyclonedds_cpp" ]]; then
    if [ -n "${CYCLONEDDS_URI}" ]; then
        if [[ "${CYCLONEDDS_URI}" =~ ^file://.* ]]; then
            DOCKER_VOLUMES+=("${CYCLONEDDS_URI//file:\/\//}:/root/.ros/cyclonedds.xml:ro")
            DOCKER_ENVIRON+=("CYCLONEDDS_URI=file:///root/.ros/cyclonedds.xml")
        else
            DOCKER_ENVIRON+=("CYCLONEDDS_URI=${CYCLONEDDS_URI}")
        fi
    elif [[ "${ROS_LOCALHOST_ONLY}" = "0" ]]; then
        DOCKER_ENVIRON+=("CYCLONEDDS_URI=<CycloneDDS><Domain><General><RedundantNetworking>true</RedundantNetworking><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>")
    fi
elif [[ "${RMW_IMPLEMENTATION}" = "rmw_fastrtps_cpp" ]]; then
    if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then
        DOCKER_VOLUMES+=("${FASTRTPS_DEFAULT_PROFILES_FILE}:/root/.ros/fastrtps.xml:ro")
        DOCKER_ENVIRON+=("FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastrtps.xml")
    fi
fi

## Ensure that Docker Engine with NVIDIA Container Toolkit are available
if [[ "${ENSURE_DOCKER,,}" = true ]]; then
    if ! command -v docker >/dev/null 2>&1 || ! command -v nvidia-container-toolkit >/dev/null 2>&1; then
        if ! command -v curl >/dev/null 2>&1; then
            if command -v apt-get >/dev/null 2>&1; then
                sudo apt-get install -y curl
            elif command -v dnf >/dev/null 2>&1; then
                sudo dnf install -y curl
            elif command -v yum >/dev/null 2>&1; then
                sudo yum install -y curl
            fi
        fi
        bash -c "$(curl -fsSL https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/host/install_docker.bash)"
    fi
fi

## Determine the name of the image to run
DOCKERHUB_USER="$(${WITH_SUDO} docker info 2>/dev/null | sed '/Username:/!d;s/.* //')"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"
IMAGE_NAME="${DOCKERHUB_USER:+${DOCKERHUB_USER}/}${PROJECT_NAME,,}"
if [[ -z "$(${WITH_SUDO} docker images -q "${IMAGE_NAME}" 2>/dev/null)" ]] && [[ -n "$(curl -fsSL "https://registry.hub.docker.com/v2/repositories/${DOCKERHUB_IMAGE_NAME}" 2>/dev/null)" ]]; then
    IMAGE_NAME="${DOCKERHUB_IMAGE_NAME}"
fi

## Generate a unique container name
CONTAINER_NAME="${IMAGE_NAME##*/}"
CONTAINER_NAME="${CONTAINER_NAME//[^a-zA-Z0-9]/_}"
ALL_CONTAINER_NAMES=$(${WITH_SUDO} docker container list --all --format "{{.Names}}")
if echo "${ALL_CONTAINER_NAMES}" | grep -qi "${CONTAINER_NAME}"; then
    ID=1
    while echo "${ALL_CONTAINER_NAMES}" | grep -qi "${CONTAINER_NAME}${ID}"; do
        ID=$((ID + 1))
    done
    CONTAINER_NAME="${CONTAINER_NAME}${ID}"
fi
DOCKER_RUN_OPTS+=" --name ${CONTAINER_NAME}"

## Parse volumes and environment variables
while getopts ":v:e:" opt; do
    case "${opt}" in
        v) DOCKER_VOLUMES+=("${OPTARG}") ;;
        e) DOCKER_ENVIRON+=("${OPTARG}") ;;
        *)
            echo >&2 "Usage: ${0} [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE] [TAG] [CMD]"
            exit 2
            ;;
    esac
done
shift "$((OPTIND - 1))"

## Parse TAG and forward CMD arguments
if [ "${#}" -gt "0" ]; then
    if [[ $(${WITH_SUDO} docker images --format "{{.Tag}}" "${IMAGE_NAME}") =~ (^|[[:space:]])${1}($|[[:space:]]) || $(curl -fsSL "https://registry.hub.docker.com/v2/repositories/${IMAGE_NAME}/tags" 2>/dev/null | grep -Poe '(?<=(\"name\":\")).*?(?=\")') =~ (^|[[:space:]])${1}($|[[:space:]]) ]]; then
        IMAGE_NAME+=":${1}"
        CMD=${*:2}
    else
        CMD=${*:1}
    fi
fi

## GPU
if [[ "${WITH_GPU,,}" = true ]]; then
    check_nvidia_gpu() {
        if [[ -n "${WITH_GPU_FORCE_NVIDIA}" ]]; then
            if [[ "${WITH_GPU_FORCE_NVIDIA,,}" = true ]]; then
                echo "[INFO] NVIDIA GPU is force-enabled via 'WITH_GPU_FORCE_NVIDIA=true'."
                return 0 # NVIDIA GPU is force-enabled
            else
                echo "[INFO] NVIDIA GPU is force-disabled via 'WITH_GPU_FORCE_NVIDIA=false'."
                return 1 # NVIDIA GPU is force-disabled
            fi
        elif ! lshw -C display 2>/dev/null | grep -qi "vendor.*nvidia"; then
            return 1 # NVIDIA GPU is not present
        elif ! command -v nvidia-smi >/dev/null 2>&1; then
            echo >&2 -e "\033[1;33m[WARNING] NVIDIA GPU is detected, but its functionality cannot be verified. This container will not be able to use the GPU. Please install nvidia-utils on the host system or force-enable NVIDIA GPU via 'WITH_GPU_FORCE_NVIDIA=true'.\033[0m"
            return 1 # NVIDIA GPU is present but nvidia-utils not installed
        elif ! nvidia-smi -L &>/dev/null; then
            echo >&2 -e "\033[1;33m[WARNING] NVIDIA GPU is detected, but it does not seem to be working properly. This container will not be able to use the GPU. Please ensure the NVIDIA drivers are properly installed on the host system.\033[0m"
            return 1 # NVIDIA GPU is present but is not working properly
        else
            return 0 # NVIDIA GPU is present and appears to be working
        fi
    }
    if check_nvidia_gpu; then
        # Enable GPU either via NVIDIA Container Toolkit or NVIDIA Docker (depending on Docker version)
        DOCKER_VERSION="$(${WITH_SUDO} docker version --format '{{.Server.Version}}')"
        MIN_VERSION_FOR_TOOLKIT="19.3"
        if [ "$(printf '%s\n' "${MIN_VERSION_FOR_TOOLKIT}" "${DOCKER_VERSION}" | sort -V | head -n1)" = "$MIN_VERSION_FOR_TOOLKIT" ]; then
            DOCKER_RUN_OPTS+=" --gpus all"
        else
            DOCKER_RUN_OPTS+=" --runtime nvidia"
        fi
        DOCKER_ENVIRON+=(
            NVIDIA_VISIBLE_DEVICES="all"
            NVIDIA_DRIVER_CAPABILITIES="all"
        )
    fi
    if [[ -e /dev/dri ]]; then
        DOCKER_RUN_OPTS+=" --device=/dev/dri:/dev/dri"
        if [[ $(getent group video) ]]; then
            DOCKER_RUN_OPTS+=" --group-add video"
        fi
    fi
fi

## GUI
if [[ "${WITH_GUI,,}" = true ]]; then
    # To enable GUI, make sure processes in the container can connect to the x server
    XAUTH="${TMPDIR:-"/tmp"}/xauth_docker_${PROJECT_NAME}"
    touch "${XAUTH}"
    chmod a+r "${XAUTH}"
    XAUTH_LIST=$(xauth nlist "${DISPLAY}")
    if [ -n "${XAUTH_LIST}" ]; then
        echo "${XAUTH_LIST}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge -
    fi
    # GUI-enabling volumes
    DOCKER_VOLUMES+=(
        "${XAUTH}:${XAUTH}"
        "/tmp/.X11-unix:/tmp/.X11-unix"
        "/dev/input:/dev/input"
    )
    # GUI-enabling environment variables
    DOCKER_ENVIRON+=(
        DISPLAY="${DISPLAY}"
        XAUTHORITY="${XAUTH}"
    )
fi

## Run the container
DOCKER_RUN_CMD=(
    "${WITH_SUDO}" docker run
    "${DOCKER_RUN_OPTS}"
    "${DOCKER_VOLUMES[@]/#/"--volume "}"
    "${DOCKER_ENVIRON[@]/#/"--env "}"
    "${IMAGE_NAME}"
    "${CMD}"
)
echo -e "\033[1;90m[TRACE] ${DOCKER_RUN_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${DOCKER_RUN_CMD[*]}
