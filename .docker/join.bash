#!/usr/bin/env bash
### Join a running Docker container
### Usage: join.bash [ID] [CMD]
### Env: DOCKER_TTY=false to skip TTY allocation for non-interactive callers
### Env: SRB_DOCKER_PROFILE=ngc to join the immutable NGC 6.1 lane
### Env: SRB_NGC_PLATFORM=linux/amd64|linux/arm64 (ngc profile)
### Env: SRB_CONTAINER_NAME=NAME to join containers started with the same setting
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"

USER="${USER:-$(id -un)}"

WITH_SUDO=()
if [[ " $(id -nG "${USER}" 2>/dev/null) " != *" docker "* ]]; then
    echo "[INFO] The current user '${USER}' is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO=("sudo")
fi

DOCKERHUB_IMAGE_NAME="${DOCKERHUB_IMAGE_NAME:-"andrejorsula/space_robotics_bench"}"
SRB_DOCKER_PROFILE="${SRB_DOCKER_PROFILE:-safe}"
SRB_NGC_MODE="${SRB_NGC_MODE:-headless}"
SRB_NGC_ENABLED=false
case "${SRB_DOCKER_PROFILE}" in
    ngc-headless)
        SRB_NGC_MODE=headless
        SRB_NGC_ENABLED=true
        SRB_DOCKER_PROFILE=ngc
        ;;
    ngc-webrtc)
        SRB_NGC_MODE=webrtc
        SRB_NGC_ENABLED=true
        SRB_DOCKER_PROFILE=ngc
        ;;
    ngc-gui)
        SRB_NGC_MODE=gui
        SRB_NGC_ENABLED=true
        SRB_DOCKER_PROFILE=ngc
        ;;
    ngc)
        SRB_NGC_ENABLED=true
        ;;
esac
if [[ "${SRB_DOCKER_NGC:-false}" = true ]]; then
    SRB_NGC_ENABLED=true
    SRB_DOCKER_PROFILE=ngc
fi
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    case "${SRB_NGC_MODE,,}" in
        headless|webrtc|gui) ;;
        *)
            echo >&2 "[ERROR] Unsupported SRB_NGC_MODE '${SRB_NGC_MODE}'. Expected headless, webrtc or gui."
            exit 2
            ;;
    esac
fi
DOCKER_EXEC_OPTS=(
    --interactive
)
DOCKER_TTY="${DOCKER_TTY:-true}"
if [[ "${DOCKER_TTY,,}" = true ]]; then
    DOCKER_EXEC_OPTS+=(--tty)
fi
DEFAULT_CMD=("bash")

ID=""
if [[ $# -gt 0 ]] && [[ "${1}" =~ ^[0-9]+$ ]]; then
    ID="${1}"
    shift
fi

CMD=("$@")
if [ ${#CMD[@]} -eq 0 ]; then
    CMD=("${DEFAULT_CMD[@]}")
fi

DOCKERHUB_USER="$("${WITH_SUDO[@]}" docker info 2>/dev/null | sed '/Username:/!d;s/.* //')"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    NGC_AMD64_DIGEST="sha256:0c16dd67d09a70ea474f2c809a13c4d09bd23c738184cf1bf28af487ecd39080"
    NGC_ARM64_DIGEST="sha256:dcf1a6e81c56f644ab7ac69c4251291c51ecb234aa0792d12d0d9c7e4e4e7cf0"
    NGC_IMAGE_REPOSITORY="nvcr.io/nvidia/isaac-sim"
    NGC_PLATFORM_INPUT="${SRB_NGC_PLATFORM:-${SRB_DOCKER_PLATFORM:-${DOCKER_DEFAULT_PLATFORM:-}}}"
    if [[ -z "${NGC_PLATFORM_INPUT}" ]]; then
        NGC_PLATFORM_INPUT="$(uname -m)"
    fi
    case "${NGC_PLATFORM_INPUT,,}" in
        linux/amd64|amd64|x86_64)
            NGC_ARCH=amd64
            NGC_PLATFORM=linux/amd64
            NGC_IMAGE_DIGEST="${NGC_AMD64_DIGEST}"
            ;;
        linux/arm64|arm64|aarch64)
            NGC_ARCH=arm64
            NGC_PLATFORM=linux/arm64
            NGC_IMAGE_DIGEST="${NGC_ARM64_DIGEST}"
            ;;
        *)
            echo >&2 "[ERROR] Unsupported NGC architecture/platform '${NGC_PLATFORM_INPUT}'. Expected linux/amd64 or linux/arm64."
            exit 2
            ;;
    esac
    IMAGE_NAME="${SRB_NGC_IMAGE:-${SRB_DOCKER_IMAGE:-${NGC_IMAGE_REPOSITORY}@${NGC_IMAGE_DIGEST}}}"
    EXPECTED_NGC_IMAGE="${NGC_IMAGE_REPOSITORY}@${NGC_IMAGE_DIGEST}"
    if [[ "${IMAGE_NAME}" != "${EXPECTED_NGC_IMAGE}" ]]; then
        echo >&2 "[ERROR] NGC join requires the architecture-specific Isaac Sim 6.1 digest ${EXPECTED_NGC_IMAGE}."
        exit 2
    fi
else
    IMAGE_NAME="${SRB_IMAGE_NAME:-${DOCKERHUB_USER:+${DOCKERHUB_USER}/}${PROJECT_NAME,,}}"
    if [[ -z "$("${WITH_SUDO[@]}" docker images -q "${IMAGE_NAME}" 2>/dev/null)" ]]; then
        echo >&2 "[INFO] Local source image '${IMAGE_NAME}' was not found; join uses only existing containers."
    fi
fi
CONTAINER_NAME_BASE="${SRB_CONTAINER_NAME:-${PROJECT_NAME//[^a-zA-Z0-9]/_}}"
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    CONTAINER_NAME_BASE+="_ngc"
fi

RELEVANT_CONTAINER_LIST=()
while IFS= read -r container; do
    [[ "${container}" == "${CONTAINER_NAME_BASE}"* ]] || continue
    [[ "${container#"${CONTAINER_NAME_BASE}"}" =~ ^[0-9]*$ ]] || continue
    RELEVANT_CONTAINER_LIST+=("${container}")
done < <("${WITH_SUDO[@]}" docker container list --all --format "{{.Names}}")
RELEVANT_CONTAINERS=""
if [[ "${#RELEVANT_CONTAINER_LIST[@]}" -gt 0 ]]; then
    RELEVANT_CONTAINERS="$(printf '%s\n' "${RELEVANT_CONTAINER_LIST[@]}")"
fi
RELEVANT_CONTAINERS_COUNT=$(echo "${RELEVANT_CONTAINERS}" | wc -w)

if [ "${RELEVANT_CONTAINERS_COUNT}" -eq "0" ]; then
    echo >&2 -e "\033[1;31m[ERROR] No containers with the name '${CONTAINER_NAME_BASE}' found. Run the container first.\033[0m"
    exit 1
fi

if [ -n "${ID}" ]; then
    TARGET_CONTAINER_NAME="${CONTAINER_NAME_BASE}"
    if [ "${ID}" -gt "0" ]; then
        TARGET_CONTAINER_NAME="${CONTAINER_NAME_BASE}${ID}"
    fi
elif [ "${RELEVANT_CONTAINERS_COUNT}" -eq 1 ]; then
    TARGET_CONTAINER_NAME="${RELEVANT_CONTAINERS}"
else
    echo >&2 -e "\033[1;31m[ERROR] Multiple containers found. Specify the ID of the container to join.\033[0m"
    echo >&2 "Usage: ${0} [ID] [CMD]"
    echo "${RELEVANT_CONTAINERS}" | sort --version-sort | while read -r container; do
        id=$(echo "${container}" | grep -oE '[0-9]+$' || echo "0")
        echo >&2 -e " ${container}\t(ID=${id})"
    done
    exit 2
fi

if grep -qxF -- "${TARGET_CONTAINER_NAME}" <<<"${RELEVANT_CONTAINERS}"; then
    CONTAINER_NAME="${TARGET_CONTAINER_NAME}"
else
    echo >&2 -e "\033[1;31m[ERROR] Container '${TARGET_CONTAINER_NAME}' does not exist.\033[0m"
    exit 1
fi

if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    CONTAINER_STATE="$("${WITH_SUDO[@]}" docker inspect --format '{{.State.Status}}' "${CONTAINER_NAME}" 2>/dev/null || :)"
    if [[ "${CONTAINER_STATE}" != running ]]; then
        echo >&2 "[ERROR] NGC container '${CONTAINER_NAME}' is not running (state: ${CONTAINER_STATE:-unknown})."
        exit 1
    fi
    CONTAINER_IMAGE_LABEL="$("${WITH_SUDO[@]}" docker inspect --format '{{index .Config.Labels "com.srb.image"}}' "${CONTAINER_NAME}" 2>/dev/null || :)"
    if [[ "${CONTAINER_IMAGE_LABEL}" != "${IMAGE_NAME}" ]]; then
        echo >&2 "[ERROR] NGC container '${CONTAINER_NAME}' has stale image identity; expected ${IMAGE_NAME}."
        exit 2
    fi
    CONTAINER_PROFILE_LABEL="$("${WITH_SUDO[@]}" docker inspect --format '{{index .Config.Labels "com.srb.profile"}}' "${CONTAINER_NAME}" 2>/dev/null || :)"
    if [[ "${CONTAINER_PROFILE_LABEL}" != ngc ]]; then
        echo >&2 "[ERROR] NGC container '${CONTAINER_NAME}' is missing the ngc profile label."
        exit 2
    fi
    CONTAINER_ARCH_LABEL="$("${WITH_SUDO[@]}" docker inspect --format '{{index .Config.Labels "com.srb.ngc.arch"}}' "${CONTAINER_NAME}" 2>/dev/null || :)"
    if [[ "${CONTAINER_ARCH_LABEL}" != "${NGC_ARCH}" ]]; then
        echo >&2 "[ERROR] NGC container '${CONTAINER_NAME}' has architecture '${CONTAINER_ARCH_LABEL:-unknown}'; expected ${NGC_ARCH} (${NGC_PLATFORM})."
        exit 2
    fi
    CONTAINER_USER="$("${WITH_SUDO[@]}" docker inspect --format '{{.Config.User}}' "${CONTAINER_NAME}" 2>/dev/null || :)"
    if [[ "${CONTAINER_USER}" != 1234:1234 ]]; then
        echo >&2 "[ERROR] NGC container '${CONTAINER_NAME}' is not running as UID 1234:1234."
        exit 2
    fi
    DOCKER_EXEC_OPTS+=(--user 1234:1234)
fi

DOCKER_EXEC_CMD=(
    "${WITH_SUDO[@]}"
    docker exec
    "${DOCKER_EXEC_OPTS[@]}"
    "${CONTAINER_NAME}"
    "${CMD[@]}"
)
printf "\033[1;90m[TRACE] docker exec profile=%q container=%q command_args=%d\033[0m\n" \
    "${SRB_DOCKER_PROFILE}" "${CONTAINER_NAME}" "${#CMD[@]}"
exec "${DOCKER_EXEC_CMD[@]}"
