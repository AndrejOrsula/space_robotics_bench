#!/usr/bin/env bash
### Run a Docker container with additional development volumes mounted
### Usage: dev.bash [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE] [TAG] [CMD]
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"
WS_DIR="$(dirname "${REPOSITORY_DIR}")"
export WITH_DEV_VOLUME="${WITH_DEV_VOLUME:-true}"

## Config
# Development volumes to mount inside the container
DOCKER_DEV_VOLUMES=(
    "${WS_DIR}/isaaclab:/root/isaaclab:rw"
    # "${WS_DIR}/dreamerv3:/root/dreamerv3:rw"
)
# Development environment variables to set inside the container
DOCKER_DEV_ENVIRON=(
    SRB_WITH_TRACEBACK="${SRB_WITH_TRACEBACK:-true}"
)

## Run the container with development volumes
DOCKER_DEV_CMD=(
    "${SCRIPT_DIR}/run.bash"
    "${DOCKER_DEV_VOLUMES[@]/#/"-v "}"
    "${DOCKER_DEV_ENVIRON[@]/#/"-e "}"
    "${*:1}"
)
echo -e "\033[1;90m[TRACE] ${DOCKER_DEV_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${DOCKER_DEV_CMD[*]}
