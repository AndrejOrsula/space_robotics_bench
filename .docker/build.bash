#!/usr/bin/env bash
### Build the Docker image
### Usage: build.bash [TAG] [BUILD_ARGS...]
### Env: SRB_BUILD_NETWORK=auto|default|host|none to select the build network
set -Eeu

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

USER="${USER:-$(id -un)}"

WITH_SUDO=()
if [[ " $(id -nG "${USER}" 2>/dev/null) " != *" docker "* ]]; then
    echo "[INFO] The current user '${USER}' is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO=("sudo")
fi

PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"
IMAGE_TAG=""

if [ "${#}" -gt "0" ]; then
    if [[ "${1}" != "-"* ]]; then
        IMAGE_TAG="${1}"
        shift
    fi
fi
BUILD_ARGS=("$@")

DOCKERHUB_USER="$("${WITH_SUDO[@]}" docker info 2>/dev/null | sed '/Username:/!d;s/.* //')"
IMAGE_NAME="${SRB_IMAGE_NAME:-${DOCKERHUB_USER:+${DOCKERHUB_USER}/}${PROJECT_NAME,,}}"
if [[ -n "${IMAGE_TAG}" ]]; then
    IMAGE_NAME+=":${IMAGE_TAG}"
fi

ASSETS_DIR="${REPOSITORY_DIR}/assets/srb_assets"
GRAPHICS_DIR="${REPOSITORY_DIR}/docs/src/_graphics"

bootstrap_repo() {
    local dir="$1"
    local label="$2"
    local source_env_var="$3"
    local repo_id_env_var="$4"
    local repo_type_env_var="$5"
    local repo_revision_env_var="$6"
    local update_args=()
    local source="${!source_env_var:-}"
    local hf_repo_id="${!repo_id_env_var:-}"
    local hf_repo_type="${!repo_type_env_var:-}"
    local hf_repo_revision="${!repo_revision_env_var:-}"
    if [[ -n "${hf_repo_id}" && -z "${hf_repo_type}" ]]; then
        hf_repo_type="dataset"
    fi

    if [ -d "${dir}" ] && [ -n "$(find "${dir}" -mindepth 1 -maxdepth 1 ! -name '.git' -print -quit)" ]; then
        return
    fi

    echo "[INFO] Initializing ${label}"
    if [[ -n "${source}" ]]; then
        if [[ "${source}" != "hf" && "${source}" != "git" ]]; then
            echo >&2 -e "\033[1;31m[ERROR] ${source_env_var} must be 'git' or 'hf'\033[0m"
            exit 1
        fi
        update_args+=(--source "${source}")
    elif [[ -n "${hf_repo_id}" ]]; then
        update_args+=(--source "hf")
    elif [[ -n "${hf_repo_type:-}" || -n "${hf_repo_revision:-}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] ${label} HF type/revision is set but no repo id is configured\033[0m"
        echo >&2 "Set ${repo_id_env_var} when using HF source."
        exit 1
    fi

    if [[ -n "${hf_repo_id}" ]]; then
        update_args+=(--repo-id "${hf_repo_id}")
    fi
    if [[ -n "${hf_repo_type:-}" ]]; then
        update_args+=(--repo-type "${hf_repo_type}")
    fi
    if [[ -n "${hf_repo_revision:-}" ]]; then
        update_args+=(--revision "${hf_repo_revision}")
    fi
    if [[ "${#update_args[@]}" -eq 0 ]]; then
        update_args+=(--source git)
    fi

    update_args+=(--path "${dir}")

    echo "[INFO] Initializing ${label} with: ${update_args[*]}"
    if ! "${REPOSITORY_DIR}/assets/scripts/update.bash" "${update_args[@]}"; then
        echo >&2 -e "\033[1;31m[ERROR] Failed to initialize ${label}.\033[0m"
        exit 1
    fi
}

SRB_ASSETS_HF_REPO_ID="${SRB_ASSETS_HF_REPO_ID:-${HF_SRB_ASSETS_REPO_ID:-}}"
SRB_ASSETS_HF_REPO_TYPE="${SRB_ASSETS_HF_REPO_TYPE:-${HF_SRB_ASSETS_REPO_TYPE:-}}"
SRB_ASSETS_HF_REPO_REVISION="${SRB_ASSETS_HF_REPO_REVISION:-${HF_SRB_ASSETS_REPO_REVISION:-}}"
bootstrap_repo \
    "${ASSETS_DIR}" \
    "assets/srb_assets" \
    "SRB_ASSETS_SOURCE" \
    "SRB_ASSETS_HF_REPO_ID" \
    "SRB_ASSETS_HF_REPO_TYPE" \
    "SRB_ASSETS_HF_REPO_REVISION"

SRB_GRAPHICS_HF_REPO_ID="${SRB_GRAPHICS_HF_REPO_ID:-${HF_SRB_GRAPHICS_REPO_ID:-}}"
SRB_GRAPHICS_HF_REPO_TYPE="${SRB_GRAPHICS_HF_REPO_TYPE:-${HF_SRB_GRAPHICS_REPO_TYPE:-}}"
SRB_GRAPHICS_HF_REPO_REVISION="${SRB_GRAPHICS_HF_REPO_REVISION:-${HF_SRB_GRAPHICS_REPO_REVISION:-}}"
bootstrap_repo \
    "${GRAPHICS_DIR}" \
    "docs/src/_graphics" \
    "SRB_GRAPHICS_SOURCE" \
    "SRB_GRAPHICS_HF_REPO_ID" \
    "SRB_GRAPHICS_HF_REPO_TYPE" \
    "SRB_GRAPHICS_HF_REPO_REVISION"

DNS_PROBE_HOST="pypi.org"

probe_build_dns() {
    local base_name base_tag
    base_name="$(sed -n 's/^ARG BASE_IMAGE_NAME="\(.*\)"$/\1/p' "${REPOSITORY_DIR}/Dockerfile" | head -n 1)"
    base_tag="$(sed -n 's/^ARG BASE_IMAGE_TAG="\(.*\)"$/\1/p' "${REPOSITORY_DIR}/Dockerfile" | head -n 1)"
    if [[ -z "${base_name}" || -z "${base_tag}" ]]; then
        return 0
    fi
    timeout 120 "${WITH_SUDO[@]}" docker run --rm --network bridge \
        --dns-option timeout:2 --dns-option attempts:1 \
        "${base_name}:${base_tag}" \
        getent hosts "${DNS_PROBE_HOST}" &>/dev/null
}

SRB_BUILD_NETWORK="${SRB_BUILD_NETWORK:-auto}"
NETWORK_ARGS=()
if [[ "${#BUILD_ARGS[@]}" -gt 0 ]] && printf '%s\n' "${BUILD_ARGS[@]}" | grep -qE '^--network(=|$)'; then
    echo "[INFO] Build network selected by an explicit --network build argument"
elif [[ "${SRB_BUILD_NETWORK}" != "auto" ]]; then
    NETWORK_ARGS=(--network "${SRB_BUILD_NETWORK}")
else
    DNS_PROBE_STATUS=0
    probe_build_dns || DNS_PROBE_STATUS=$?
    case "${DNS_PROBE_STATUS}" in
        0) ;;
        2)
            echo >&2 -e "\033[1;33m[WARN] The default Docker bridge network cannot resolve '${DNS_PROBE_HOST}'.\033[0m"
            echo >&2 "Building with --network host so the build reaches the host's resolvers. This exposes"
            echo >&2 "the host network to every RUN step; set SRB_BUILD_NETWORK=default to opt out, or"
            echo >&2 "configure the daemon's DNS (\"dns\" in /etc/docker/daemon.json) to fix the bridge."
            NETWORK_ARGS=(--network host)
            ;;
        *)
            fail "Docker DNS probe did not complete a supported lookup (exit ${DNS_PROBE_STATUS}); refusing to build while probe completion is uncertain"
            ;;
    esac
fi

DOCKER_BUILD_CMD=(
    "${WITH_SUDO[@]}"
    docker build
    --file "${REPOSITORY_DIR}/Dockerfile"
    --tag "${IMAGE_NAME}"
    "${NETWORK_ARGS[@]}"
    "${BUILD_ARGS[@]}"
    "${REPOSITORY_DIR}"
)
printf "\033[1;90m[TRACE] "
printf "%q " "${DOCKER_BUILD_CMD[@]}"
printf "\033[0m\n"
"${DOCKER_BUILD_CMD[@]}"
