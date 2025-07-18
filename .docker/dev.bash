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
    # "${WS_DIR}/isaaclab:/root/isaaclab:rw"
    # "${WS_DIR}/simforge:/root/simforge:rw"
    # "${WS_DIR}/simforge_foundry:/root/simforge_foundry:rw"
    # "${WS_DIR}/oxidasim:/root/oxidasim:rw"
    # "${WS_DIR}/dreamerv3:/root/dreamerv3:rw"
    # "${WS_DIR}/skrl:/root/skrl:rw"
)
# Development environment variables to set inside the container
DOCKER_DEV_ENVIRON=(
    LOG_LEVEL="${LOG_LEVEL:-debug}"
    DEBUG_VIS="${DEBUG_VIS:-true}"
    HYDRA_FULL_ERROR="${HYDRA_FULL_ERROR:-1}"
    RICH_TRACEBACK="${RICH_TRACEBACK:-true}"
    # RICH_TRACEBACK_LOCALS="${RICH_TRACEBACK_LOCALS:-true}"
    LOGFIRE_ENABLE="${LOGFIRE_ENABLE:-true}"
    # LOGFIRE_SEND_TO_LOGFIRE="${LOGFIRE_SEND_TO_LOGFIRE:-true}"
)

## Verify that all development volumes exist
for volume in "${DOCKER_DEV_VOLUMES[@]}"; do
    host_dir="${volume%%:*}"
    target_dir="${volume#*:}"
    target_dir="${target_dir%%:*}"
    if [[ ! -d "${host_dir}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] The source directory '${host_dir}' to be mounted as a volume at '${target_dir}' does not exist.\033[0m"
        exit 1
    fi
done

## Run the container with development volumes
DEV_ARGS=()
for vol in "${DOCKER_DEV_VOLUMES[@]}"; do
    DEV_ARGS+=("-v" "${vol}")
done
for env_var in "${DOCKER_DEV_ENVIRON[@]}"; do
    DEV_ARGS+=("-e" "${env_var}")
done
DOCKER_DEV_CMD=(
    "${SCRIPT_DIR}/run.bash"
    "${DEV_ARGS[@]}"
    "$@"
)
printf "\033[1;90m[TRACE] "
printf "%q " "${DOCKER_DEV_CMD[@]}"
printf "\033[0m\n"
exec "${DOCKER_DEV_CMD[@]}"
