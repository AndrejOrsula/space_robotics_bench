#!/usr/bin/env bash
### Extract Isaac Sim from a Docker image
### Usage: extract_isaacsim.bash [destination_directory]
### Env: SRC_IMAGE=local image, or NAME@sha256:DIGEST to pull a remote one
set -e

SRC_IMAGE="${SRC_IMAGE:-"andrejorsula/space_robotics_bench:latest"}"
SRC_PATH="${SRC_PATH:-"/root/isaac-sim"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/isaac-sim"}"}"
DEST_DIR="${DEST_DIR%/}"
if [[ -L "${DEST_DIR}" ]]; then
    DEST_LINK="${DEST_DIR}"
    DEST_DIR="$(readlink -f -- "${DEST_LINK}" || :)"
    if [[ -z "${DEST_DIR}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Cannot resolve the destination symlink '${DEST_LINK}'\033[0m"
        exit 1
    fi
fi
if [[ -z "${DEST_DIR}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] The destination must not be the filesystem root\033[0m"
    exit 1
fi
CONTAINER_NAME="isaac_sim_extract_$(date +%s)_$$"

echo "[INFO] Source image: ${SRC_IMAGE}"
echo "[INFO] Source path: ${SRC_PATH}"
echo "[INFO] Destination path: ${DEST_DIR}"

if [[ -e "${DEST_DIR}" && ! -d "${DEST_DIR}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Destination '${DEST_DIR}' exists and is not a directory\033[0m"
    exit 1
fi
if [[ -d "${DEST_DIR}" && -n "$(ls -A -- "${DEST_DIR}")" ]] &&
[[ ! -f "${DEST_DIR}/python.sh" || ! -d "${DEST_DIR}/kit" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Destination '${DEST_DIR}' is neither empty nor an Isaac Sim install (python.sh, kit/); refusing to replace it\033[0m"
    exit 1
fi
if [[ -d "${DEST_DIR}" ]]; then
    echo -en "\033[1;33m[WARNING] Destination directory already exists, overwrite? [y/N]\033[0m "
    read -r
    if [[ ! "${REPLY}" =~ ^[Yy]$ ]]; then
        echo "[INFO] Exiting"
        exit 0
    fi
fi

if ! docker info >/dev/null 2>&1; then
    echo >&2 -e "\033[1;31m[ERROR] Docker is not running or not accessible\033[0m"
    exit 1
fi

if [[ -z "$(docker image ls -q "${SRC_IMAGE}")" ]]; then
    if [[ ! "${SRC_IMAGE}" =~ @sha256:[0-9a-f]{64}$ ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Image '${SRC_IMAGE}' is not available locally; set SRC_IMAGE to a local image or pin it as NAME@sha256:DIGEST\033[0m"
        exit 2
    fi
    echo "[INFO] Pulling the Docker image"
    docker pull "${SRC_IMAGE}" &>/dev/null
else
    echo "[INFO] Docker image is already available locally"
fi

STAGE_DIR=""
cleanup() {
    echo "[INFO] Stopping container"
    docker stop "${CONTAINER_NAME}" &>/dev/null || true
    if [[ -n "${STAGE_DIR}" ]]; then
        if [[ -e "${STAGE_DIR}/previous" ]]; then
            echo "[INFO] Restoring the previous Isaac Sim install"
            rm -rf -- "${DEST_DIR}"
            if ! mv -- "${STAGE_DIR}/previous" "${DEST_DIR}"; then
                echo >&2 -e "\033[1;31m[ERROR] Could not restore the previous Isaac Sim install from ${STAGE_DIR}/previous\033[0m"
                return 1
            fi
        fi
        rm -rf -- "${STAGE_DIR}"
    fi
}
trap cleanup EXIT  # Ensure cleanup runs on script exit

echo "[INFO] Starting a temporary Docker container: ${CONTAINER_NAME}"
docker run --name "${CONTAINER_NAME}" --rm --entrypoint bash "${SRC_IMAGE}" -c "sleep infinity" &

echo "[INFO] Checking if container is ready"
for i in {1..10}; do
    sleep 0.5
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        if docker exec "${CONTAINER_NAME}" ls "${SRC_PATH}" &>/dev/null; then
            echo "[INFO] Container is ready"
            break
        else
            echo >&2 -e "\033[1;31m[ERROR] SRC_PATH='${SRC_PATH}' is absent from image '${SRC_IMAGE}'; set SRC_PATH to the Isaac Sim install path inside that image.\033[0m"
            exit 1
        fi
    fi
    if [[ "${i}" -eq 10 ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Container did not start in time\033[0m"
        exit 1
    fi
done

mkdir -p "$(dirname "${DEST_DIR}")"

STAGE_DIR="$(mktemp -d "${DEST_DIR}.extract.XXXXXX")"
echo "[INFO] Copying ${CONTAINER_NAME}:${SRC_PATH} to ${DEST_DIR}"
docker cp -L "${CONTAINER_NAME}:${SRC_PATH}" "${STAGE_DIR}/tree"
if [[ ! -d "${STAGE_DIR}/tree" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Copy of '${SRC_PATH}' did not produce a directory\033[0m"
    exit 1
fi
if [[ -e "${DEST_DIR}" ]]; then
    mv -- "${DEST_DIR}" "${STAGE_DIR}/previous"
fi
if ! mv -- "${STAGE_DIR}/tree" "${DEST_DIR}"; then
    if [[ -e "${STAGE_DIR}/previous" ]]; then
        mv -- "${STAGE_DIR}/previous" "${DEST_DIR}"
    fi
    echo >&2 -e "\033[1;31m[ERROR] Could not move the extracted tree into ${DEST_DIR}\033[0m"
    exit 1
fi
echo "[INFO] Updating pip in extracted environment"
if [[ -f "${DEST_DIR}/python.sh" ]]; then
    "${DEST_DIR}/python.sh" -m pip install --upgrade pip
else
    echo >&2 -e "\033[1;33m[WARNING] python.sh not found in ${DEST_DIR}, skipping pip upgrade\033[0m"
fi

rm -rf -- "${STAGE_DIR}"
STAGE_DIR=""

echo "[INFO] Isaac Sim has been extracted to ${DEST_DIR}"
echo "[INFO] Recommended environment variable setup:"
echo "[INFO]   echo \"export ISAACSIM_PYTHON='${DEST_DIR}/python.sh'\" >> ~/.bashrc"
echo "[INFO]   set -Ux ISAACSIM_PYTHON '${DEST_DIR}/python.sh' # Fish"
