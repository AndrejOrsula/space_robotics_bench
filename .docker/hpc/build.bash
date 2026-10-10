#!/usr/bin/env bash
### Build a Singularity image from the Docker image
### Usage: build.bash [TAG] [BUILD_ARGS...]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
source "${SCRIPT_DIR}/run.bash"
DOT_DOCKER_DIR="$(dirname "${SCRIPT_DIR}")"
REPOSITORY_DIR="$(dirname "${DOT_DOCKER_DIR}")"
IMAGES_DIR="${SCRIPT_DIR}/images"
DOCKER_BUILD_SCRIPT="${DOT_DOCKER_DIR}/build.bash"
export APPTAINER_TMPDIR="${APPTAINER_TMPDIR:-"${HOME}/.apptainer/tmp"}"

USER="${USER:-$(id -un)}"

WITH_SUDO=()
if [[ " $(id -nG "${USER}" 2>/dev/null) " != *" docker "* ]]; then
    echo "[INFO] The current user '${USER}' is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO=(sudo)
fi

DOCKERHUB_USER="$("${WITH_SUDO[@]}" docker info 2>/dev/null | sed '/Username:/!d;s/.* //')"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"
IMAGE_NAME="${SRB_IMAGE_NAME:-${DOCKERHUB_USER:+${DOCKERHUB_USER}/}${PROJECT_NAME,,}}"
OUTPUT_PATH="${IMAGES_DIR}/${PROJECT_NAME}.sif"

if [ "${#}" -gt "0" ]; then
    if [[ "${1}" != "-"* ]]; then
        TAG="${1}"
        shift
    fi
fi
BUILD_ARGS=("$@")
TAG="${TAG:-"latest"}"
IMAGE_NAME+=":${TAG}"

mkdir -p "${APPTAINER_TMPDIR}" "${IMAGES_DIR}"

run_owned_session "${DOCKER_BUILD_SCRIPT}" "${TAG}" "${BUILD_ARGS[@]}"

APPTAINER_BUILD_CMD=(
    apptainer build
    "${OUTPUT_PATH}"
    "docker-daemon:${IMAGE_NAME}"
)
printf "\033[1;90m[TRACE] "
printf "%q " "${APPTAINER_BUILD_CMD[@]}"
printf "\033[0m\n"
run_owned_session "${APPTAINER_BUILD_CMD[@]}"
