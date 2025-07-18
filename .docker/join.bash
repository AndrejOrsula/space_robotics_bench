#!/usr/bin/env bash
### Join a running Docker container
### Usage: join.bash [ID] [CMD]
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"

## If the current user is not in the docker group, all docker commands will be run as root
WITH_SUDO=()
if ! grep -qi /etc/group -e "docker.*${USER}"; then
    echo "[INFO] The current user '${USER}' is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO=("sudo")
fi

## Config
# Name of the Docker image to join if an image with locally-defined name does not exist
DOCKERHUB_IMAGE_NAME="${DOCKERHUB_IMAGE_NAME:-"andrejorsula/space_robotics_bench"}"
# Options for executing a command inside the container
DOCKER_EXEC_OPTS=(
    --interactive
    --tty
)
# Default command to execute inside the container
DEFAULT_CMD=("bash")

## Parse ID and CMD
ID=""
if [[ $# -gt 0 ]] && [[ "${1}" =~ ^[0-9]+$ ]]; then
    ID="${1}"
    shift # Consume the ID argument
fi

CMD=("$@")
if [ ${#CMD[@]} -eq 0 ]; then
    CMD=("${DEFAULT_CMD[@]}")
fi

## Determine the name of the container to join
DOCKERHUB_USER="$("${WITH_SUDO[@]}" docker info 2>/dev/null | sed '/Username:/!d;s/.* //')"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"
IMAGE_NAME="${DOCKERHUB_USER:+${DOCKERHUB_USER}/}${PROJECT_NAME,,}"
if [[ -z "$("${WITH_SUDO[@]}" docker images -q "${IMAGE_NAME}" 2>/dev/null)" ]] && [[ -n "$(curl -fsSL "https://registry.hub.docker.com/v2/repositories/${DOCKERHUB_IMAGE_NAME}" 2>/dev/null)" ]]; then
    IMAGE_NAME="${DOCKERHUB_IMAGE_NAME}"
fi
CONTAINER_NAME_BASE="${IMAGE_NAME##*/}"
CONTAINER_NAME_BASE="${CONTAINER_NAME_BASE//[^a-zA-Z0-9]/_}"

## Verify/select the appropriate container to join
RELEVANT_CONTAINERS=$("${WITH_SUDO[@]}" docker container list --all --format "{{.Names}}" | grep "^${CONTAINER_NAME_BASE}" || :)
RELEVANT_CONTAINERS_COUNT=$(echo "${RELEVANT_CONTAINERS}" | wc -w)

if [ "${RELEVANT_CONTAINERS_COUNT}" -eq "0" ]; then
    echo >&2 -e "\033[1;31m[ERROR] No containers with the name '${CONTAINER_NAME_BASE}' found. Run the container first.\033[0m"
    exit 1
fi

if [ -n "${ID}" ]; then
    # If an ID is provided, construct the target name
    TARGET_CONTAINER_NAME="${CONTAINER_NAME_BASE}"
    if [ "${ID}" -gt "0" ]; then
        TARGET_CONTAINER_NAME="${CONTAINER_NAME_BASE}${ID}"
    fi
elif [ "${RELEVANT_CONTAINERS_COUNT}" -eq 1 ]; then
    # If no ID is provided and there's only one container, use it
    TARGET_CONTAINER_NAME="${RELEVANT_CONTAINERS}"
else
    # If no ID is provided and there are multiple containers, it's an error
    echo >&2 -e "\033[1;31m[ERROR] Multiple containers found. Specify the ID of the container to join.\033[0m"
    echo >&2 "Usage: ${0} [ID] [CMD]"
    echo "${RELEVANT_CONTAINERS}" | sort --version-sort | while read -r container; do
        id=$(echo "${container}" | grep -oE '[0-9]+$' || echo "0")
        echo >&2 -e " ${container}\t(ID=${id})"
    done
    exit 2
fi

# Verify that the selected container actually exists
if echo "${RELEVANT_CONTAINERS}" | grep -qxw "${TARGET_CONTAINER_NAME}"; then
    CONTAINER_NAME="${TARGET_CONTAINER_NAME}"
else
    echo >&2 -e "\033[1;31m[ERROR] Container '${TARGET_CONTAINER_NAME}' does not exist.\033[0m"
    exit 1
fi

## Execute command inside the container
DOCKER_EXEC_CMD=(
    "${WITH_SUDO[@]}"
    docker exec
    "${DOCKER_EXEC_OPTS[@]}"
    "${CONTAINER_NAME}"
    "${CMD[@]}"
)
printf "\033[1;90m[TRACE] "
printf "%q " "${DOCKER_EXEC_CMD[@]}"
printf "\033[0m\n"
exec "${DOCKER_EXEC_CMD[@]}"
