#!/usr/bin/env bash
### Install pinned Docker Engine/CLI and, when requested, NVIDIA Container Toolkit.
### Usage: install_docker.bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
    echo "[ERROR] /etc/os-release is required; refusing an unknown host." >&2
    exit 64
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 && "${VERSION_ID:-}" != 24.04 ]]; then
    echo "[ERROR] supported hosts are Ubuntu 22.04 (jammy) and 24.04 (noble); found ${ID:-unknown} ${VERSION_ID:-unknown}." >&2
    exit 64
fi
if [[ "${VERSION_CODENAME:-}" != jammy && "${VERSION_CODENAME:-}" != noble ]]; then
    echo "[ERROR] Ubuntu codename must be jammy or noble; found ${VERSION_CODENAME:-unknown}." >&2
    exit 64
fi
if ! command -v dpkg >/dev/null 2>&1 || ! command -v apt-get >/dev/null 2>&1; then
    echo "[ERROR] this installer requires apt-get and dpkg." >&2
    exit 64
fi
DPKG_ARCH="$(dpkg --print-architecture)"
if [[ "${DPKG_ARCH}" != amd64 && "${DPKG_ARCH}" != arm64 ]]; then
    echo "[ERROR] supported architectures are amd64 and arm64; found ${DPKG_ARCH}." >&2
    exit 64
fi

if (( EUID == 0 )); then
    ROOT_CMD=()
else
    command -v sudo >/dev/null 2>&1 || {
        echo "[ERROR] sudo is required when this script is not run as root." >&2
        exit 64
    }
    ROOT_CMD=(sudo)
fi

run_root() {
    "${ROOT_CMD[@]}" "$@"
}

DOCKER_KEY_URL="https://download.docker.com/linux/ubuntu/gpg"
DOCKER_KEY_FINGERPRINT="9DC858229FC7DD38854AE2D88D81803C0EBFCD88"
NVIDIA_KEY_URL="https://nvidia.github.io/libnvidia-container/gpgkey"
NVIDIA_KEY_FINGERPRINT="C95B321B61E88C1809C4F759DDCAE044F796ECB0"
DOCKER_CE_VERSION="${SRB_DOCKER_CE_VERSION:-5:29.8.0-1~ubuntu.${VERSION_ID}~${VERSION_CODENAME}}"
DOCKER_CLI_VERSION="${SRB_DOCKER_CLI_VERSION:-${DOCKER_CE_VERSION}}"
DOCKER_CONTAINERD_VERSION="${SRB_DOCKER_CONTAINERD_VERSION:-2.3.6-1~ubuntu.${VERSION_ID}~${VERSION_CODENAME}}"
DOCKER_BUILDX_VERSION="${SRB_DOCKER_BUILDX_VERSION:-0.37.1-1~ubuntu.${VERSION_ID}~${VERSION_CODENAME}}"
DOCKER_COMPOSE_VERSION="${SRB_DOCKER_COMPOSE_VERSION:-5.6.0-1~ubuntu.${VERSION_ID}~${VERSION_CODENAME}}"
DOCKER_ROOTLESS_EXTRAS_VERSION="${SRB_DOCKER_ROOTLESS_EXTRAS_VERSION:-${DOCKER_CE_VERSION}}"
NVIDIA_TOOLKIT_VERSION="${SRB_NVIDIA_TOOLKIT_VERSION:-1.20.0-1}"

fetch_verified_key() {
    local url="$1"
    local expected="$2"
    local output="$3"
    local temporary
    temporary="$(mktemp)"
    trap 'rm -f "${temporary}"' RETURN
    curl --fail --silent --show-error --location --proto '=https' --tlsv1.2 "${url}" -o "${temporary}"
    local primary_fingerprints
    primary_fingerprints="$(
        gpg --show-keys --with-colons "${temporary}" 2>/dev/null |
            awk -F: '
                $1 == "pub" { in_primary=1; next }
                $1 == "sub" { in_primary=0; next }
                in_primary && $1 == "fpr" { print toupper($10) }
            '
    )"
    local -a primary_fingerprint_list=()
    if [[ -n "${primary_fingerprints}" ]]; then
        mapfile -t primary_fingerprint_list <<<"${primary_fingerprints}"
    fi
    if (( ${#primary_fingerprint_list[@]} != 1 )); then
        echo "[ERROR] key bundle has ${#primary_fingerprint_list[@]} primary key fingerprints for ${url}; expected exactly one (${expected}), found ${primary_fingerprints:-none}." >&2
        exit 65
    fi
    if [[ "${primary_fingerprint_list[0]}" != "${expected}" ]]; then
        echo "[ERROR] key fingerprint mismatch for ${url}: expected ${expected}, found ${primary_fingerprint_list[0]}." >&2
        exit 65
    fi
    run_root install -d -m 0755 /etc/apt/keyrings
    run_root gpg --dearmor --yes -o "${output}" "${temporary}"
    run_root chmod 0644 "${output}"
    rm -f "${temporary}"
    trap - RETURN
}

run_root apt-get update
run_root env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ca-certificates curl gnupg

fetch_verified_key "${DOCKER_KEY_URL}" "${DOCKER_KEY_FINGERPRINT}" /etc/apt/keyrings/docker.gpg
printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu %s stable\n' \
    "${DPKG_ARCH}" "${VERSION_CODENAME}" |
run_root tee /etc/apt/sources.list.d/docker.list >/dev/null

run_root apt-get update
DOCKER_PACKAGES=(
    "docker-ce=${DOCKER_CE_VERSION}"
    "docker-ce-cli=${DOCKER_CLI_VERSION}"
    "containerd.io=${DOCKER_CONTAINERD_VERSION}"
    "docker-buildx-plugin=${DOCKER_BUILDX_VERSION}"
    "docker-compose-plugin=${DOCKER_COMPOSE_VERSION}"
    "docker-ce-rootless-extras=${DOCKER_ROOTLESS_EXTRAS_VERSION}"
)
for package_pin in "${DOCKER_PACKAGES[@]}"; do
    if ! apt-cache policy "${package_pin%%=*}" | grep -F "${package_pin#*=}" >/dev/null; then
        echo "[ERROR] ${package_pin%%=*} version ${package_pin#*=} is unavailable for Ubuntu ${VERSION_CODENAME}/${DPKG_ARCH}." >&2
        exit 66
    fi
done
run_root env DEBIAN_FRONTEND=noninteractive apt-get install -y "${DOCKER_PACKAGES[@]}"
for package_pin in "${DOCKER_PACKAGES[@]}"; do
    if [[ "$(dpkg-query -W -f='${Status} ${Version}' "${package_pin%%=*}" 2>/dev/null)" != "install ok installed ${package_pin#*=}" ]]; then
        echo "[ERROR] ${package_pin%%=*} is not installed at the pinned version ${package_pin#*=}." >&2
        exit 1
    fi
done
run_root systemctl enable --now docker

check_nvidia_gpu() {
    command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1
}

INSTALL_NVIDIA="${SRB_INSTALL_NVIDIA_TOOLKIT:-auto}"
case "${INSTALL_NVIDIA,,}" in
    auto)
        if check_nvidia_gpu; then
            INSTALL_NVIDIA=true
        else
            INSTALL_NVIDIA=false
        fi
        ;;
    true|false)
        ;;
    *)
        echo "[ERROR] SRB_INSTALL_NVIDIA_TOOLKIT must be auto, true, or false." >&2
        exit 64
        ;;
esac

if [[ "${INSTALL_NVIDIA,,}" = true ]]; then
    fetch_verified_key "${NVIDIA_KEY_URL}" "${NVIDIA_KEY_FINGERPRINT}" /etc/apt/keyrings/nvidia-container-toolkit.gpg
    printf 'deb [arch=%s signed-by=/etc/apt/keyrings/nvidia-container-toolkit.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/%s /\n' \
        "${DPKG_ARCH}" "${DPKG_ARCH}" |
    run_root tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
    run_root apt-get update
    if ! apt-cache policy nvidia-container-toolkit | grep -F "${NVIDIA_TOOLKIT_VERSION}" >/dev/null; then
        echo "[ERROR] NVIDIA Container Toolkit version ${NVIDIA_TOOLKIT_VERSION} is unavailable for ${DPKG_ARCH}." >&2
        exit 66
    fi
    run_root env DEBIAN_FRONTEND=noninteractive apt-get install -y \
        "nvidia-container-toolkit=${NVIDIA_TOOLKIT_VERSION}"
    [[ "$(dpkg-query -W -f='${Status} ${Version}' nvidia-container-toolkit 2>/dev/null)" == "install ok installed ${NVIDIA_TOOLKIT_VERSION}" ]]
    run_root nvidia-ctk runtime configure --runtime=docker
    run_root systemctl restart docker
else
    echo "[INFO] NVIDIA Container Toolkit installation skipped (SRB_INSTALL_NVIDIA_TOOLKIT=${INSTALL_NVIDIA})."
fi

TARGET_USER="${SUDO_USER:-${USER:-$(id -un)}}"
if [[ "${TARGET_USER}" != root ]] && ! id -nG "${TARGET_USER}" 2>/dev/null | tr ' ' '\n' | grep -qx docker; then
    ADD_DOCKER_GROUP="${SRB_ADD_DOCKER_GROUP:-false}"
    if [[ "${ADD_DOCKER_GROUP,,}" = ask ]]; then
        if [[ -t 0 && -t 1 ]]; then
            read -r -p "Add ${TARGET_USER} to the docker group? [y/N] " ADD_DOCKER_GROUP
        else
            ADD_DOCKER_GROUP=false
        fi
    fi
    case "${ADD_DOCKER_GROUP,,}" in
        true|y|yes)
            run_root groupadd --force docker
            run_root usermod --append --groups docker "${TARGET_USER}"
            echo "[INFO] added ${TARGET_USER} to the docker group; start a new login session."
            ;;
        false|n|no|"")
            echo "[INFO] did not change ${TARGET_USER}'s group membership."
            ;;
        *)
            echo "[ERROR] SRB_ADD_DOCKER_GROUP must be true, false, or ask." >&2
            exit 64
            ;;
    esac
else
    echo "[INFO] ${TARGET_USER} already has Docker access or is root."
fi
