#!/usr/bin/env bash
### Usage: install_isaacsim.bash [destination_directory]
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OWNED_DIR_TOOL="${SCRIPT_DIR}/owned_directory.py"
INSTALL_OWNER="srb.installer.isaacsim"
SRC_REPOSITORY="${SRC_REPOSITORY:-"https://github.com/isaac-sim/IsaacSim.git"}"
SRC_REPOSITORY_DISPLAY="$(printf '%s' "${SRC_REPOSITORY}" | sed -E 's#^([A-Za-z][A-Za-z0-9+.-]*://)[^/@]+@#\1***@#')"
SRC_BRANCH="${SRC_BRANCH:-"release/6.1"}"
SRC_COMMIT_SHA="${SRC_COMMIT_SHA:-"7c206f75bdadd9e05fc457f19863ca4c3f0cb693"}"
EXPECTED_SOURCE_VERSION="${ISAACSIM_SOURCE_VERSION:-"6.1.0-rc.26"}"
DEST_DIR="${1:-"${DEST_DIR:-"${HOME:-/tmp}/isaac-sim"}"}"
DEST_DIR="$(realpath -m -- "${DEST_DIR}")"
DEST_PARENT="$(dirname -- "${DEST_DIR}")"
SRC_DIR=""
STATE_DIR=""
OMNI_HUB_WORK=""
DEST_MARKED=false
INSTALL_PUBLISHED=false
COMPILER_MUTATED=false
VULKAN_MUTATED=false
HOST_STATE_SNAPSHOTTED=false

SRC_ALLOW_UNPINNED="${SRC_ALLOW_UNPINNED:-false}"
if [[ "${SRC_ALLOW_UNPINNED,,}" == "true" ]]; then
    echo >&2 -e "\033[1;33m[WARNING] SRC_ALLOW_UNPINNED=true: installing the '${SRC_BRANCH}' tip of ${SRC_REPOSITORY_DISPLAY} without verifying any commit. Unverified upstream code will be executed.\033[0m"
elif [[ ! "${SRC_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]]; then
    echo >&2 -e "\033[1;31m[ERROR] SRC_COMMIT_SHA must be a full 40-character lowercase hex commit (got '${SRC_COMMIT_SHA}'). Set SRC_ALLOW_UNPINNED=true to install an unverified branch tip instead.\033[0m"
    exit 1
fi

ACCEPT_EULA="${ACCEPT_EULA:-${ISAACSIM_ACCEPT_EULA:-}}"
case "${ACCEPT_EULA^^}" in
    Y|YES|TRUE|1) ;;
    *)
        echo >&2 -e "\033[1;31m[ERROR] Isaac Sim EULA consent is required. Re-run with ACCEPT_EULA=Y after reviewing NVIDIA's license.\033[0m"
        exit 64
        ;;
esac

echo "[INFO] Source repository: ${SRC_REPOSITORY_DISPLAY}"
echo "[INFO] Source branch: ${SRC_BRANCH}"
echo "[INFO] Source commit: ${SRC_COMMIT_SHA}"
echo "[INFO] Expected source VERSION: ${EXPECTED_SOURCE_VERSION}"
echo "[INFO] Destination path: ${DEST_DIR}"

VULKAN_PATHS=(
    "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    "/usr/share/glvnd/egl_vendor.d/50_mesa.json"
    "/etc/vulkan/icd.d/nvidia_icd.json"
    "/etc/vulkan/implicit_layer.d/nvidia_layers.json"
)

_state_key() {
    case "$1" in
        "/usr/share/glvnd/egl_vendor.d/10_nvidia.json") printf '%s' egl_nvidia ;;
        "/usr/share/glvnd/egl_vendor.d/50_mesa.json") printf '%s' egl_mesa ;;
        "/etc/vulkan/icd.d/nvidia_icd.json") printf '%s' icd_nvidia ;;
        "/etc/vulkan/implicit_layer.d/nvidia_layers.json") printf '%s' layers_nvidia ;;
        *) return 1 ;;
    esac
}

_snapshot_vulkan_file() {
    local path="$1"
    local key
    key="$(_state_key "${path}")"
    if [[ -e "${path}" || -L "${path}" ]]; then
        printf '%s\n' present > "${STATE_DIR}/vulkan/${key}.state"
        sudo cp -a -- "${path}" "${STATE_DIR}/vulkan/${key}.value"
    else
        printf '%s\n' absent > "${STATE_DIR}/vulkan/${key}.state"
    fi
}

_restore_vulkan_file() {
    local path="$1"
    local key
    key="$(_state_key "${path}")"
    local state_file="${STATE_DIR}/vulkan/${key}.state"
    [[ -f "${state_file}" ]] || return 0
    if [[ "$(<"${state_file}")" == present ]]; then
        sudo mkdir -p -- "$(dirname -- "${path}")"
        sudo rm -f -- "${path}"
        sudo cp -a -- "${STATE_DIR}/vulkan/${key}.value" "${path}"
    else
        sudo rm -f -- "${path}"
    fi
}

_alternative_value() {
    awk -F': ' '$1 == "Value" { print $2; exit }' <<< "$1"
}

_alternative_status() {
    awk -F': ' '$1 == "Status" { print $2; exit }' <<< "$1"
}

_restore_alternative() {
    local name="$1"
    local state="$2"
    local value
    if [[ -z "${state}" ]]; then
        sudo update-alternatives --remove "${name}" "/usr/bin/${name}-11" 2>/dev/null || true
        return 0
    fi
    value="$(_alternative_value "${state}")"
    if [[ "$(_alternative_status "${state}")" == auto ]]; then
        sudo update-alternatives --auto "${name}"
    elif [[ -n "${value}" && -e "${value}" ]]; then
        sudo update-alternatives --set "${name}" "${value}"
    else
        echo >&2 "[WARNING] Could not restore ${name} alternative target '${value}'"
        return 1
    fi
}

restore_host_state() {
    local failed=0
    set +e
    if [[ "${COMPILER_MUTATED}" == true && "${HOST_STATE_SNAPSHOTTED}" == true ]]; then
        _restore_alternative gcc "${GCC_ALTERNATIVE_STATE}" || failed=1
        _restore_alternative g++ "${GXX_ALTERNATIVE_STATE}" || failed=1
    fi
    if [[ "${VULKAN_MUTATED}" == true && "${HOST_STATE_SNAPSHOTTED}" == true && "${INSTALL_PUBLISHED}" != true ]]; then
        for path in "${VULKAN_PATHS[@]}"; do
            _restore_vulkan_file "${path}" || failed=1
        done
    fi
    set -e
    return "${failed}"
}

cleanup() {
    local exit_status="${1:-0}"
    local cleanup_failed=0
    set +e

    if [[ "${DEST_MARKED}" == true && "${INSTALL_PUBLISHED}" != true ]]; then
        python3 "${OWNED_DIR_TOOL}" remove --path "${DEST_DIR}" --owner "${INSTALL_OWNER}" || cleanup_failed=1
    fi
    if [[ -n "${SRC_DIR}" && -d "${SRC_DIR}" ]]; then
        rm -rf -- "${SRC_DIR}"
    fi
    if ! restore_host_state; then
        cleanup_failed=1
    fi
    if [[ -n "${STATE_DIR}" && -d "${STATE_DIR}" ]]; then
        rm -rf -- "${STATE_DIR}"
    fi
    if [[ -n "${OMNI_HUB_WORK}" && -d "${OMNI_HUB_WORK}" ]]; then
        rm -rf -- "${OMNI_HUB_WORK}"
    fi
    set -e

    if (( exit_status == 0 && cleanup_failed != 0 )); then
        exit_status=1
    fi
    return "${exit_status}"
}

trap 'exit_status=$?; cleanup "${exit_status}"; exit "${exit_status}"' EXIT

for cmd in git git-lfs cmake make curl unzip sha256sum realpath mktemp cp mv find sed python3 sudo update-alternatives; do
    if ! command -v "${cmd}" &>/dev/null; then
        echo >&2 -e "\033[1;31m[ERROR] Required command '${cmd}' is not installed.\033[0m"
        exit 1
    fi
done
for compiler in gcc-11 g++-11; do
    if ! command -v "${compiler}" &>/dev/null; then
        echo >&2 -e "\033[1;31m[ERROR] Required compiler '${compiler}' is not installed. Install it with: sudo apt-get install gcc-11 g++-11\033[0m"
        exit 1
    fi
done

if [[ -e "${DEST_DIR}" || -L "${DEST_DIR}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Destination already exists: ${DEST_DIR}. Choose a new side-by-side/disposable path; existing installations are never overwritten.\033[0m"
    exit 73
fi
mkdir -p -- "${DEST_PARENT}"
if ! mkdir -- "${DEST_DIR}"; then
    echo >&2 -e "\033[1;31m[ERROR] Destination appeared during preflight: ${DEST_DIR}\033[0m"
    exit 73
fi
if ! python3 "${OWNED_DIR_TOOL}" mark-created --path "${DEST_DIR}" --owner "${INSTALL_OWNER}"; then
    if ! rmdir -- "${DEST_DIR}"; then
        echo >&2 -e "\033[1;33m[WARNING] Could not remove unmarked destination directory: ${DEST_DIR}\033[0m"
    fi
    echo >&2 -e "\033[1;31m[ERROR] Could not claim the disposable destination safely: ${DEST_DIR}\033[0m"
    exit 73
fi
DEST_MARKED=true

SRC_DIR="$(mktemp -d -t isaac-sim-src.XXXXXXXX)"
echo "[INFO] Cloning repository to ${SRC_DIR}"
git clone --branch "${SRC_BRANCH}" "${SRC_REPOSITORY}" "${SRC_DIR}"
if [[ "${SRC_ALLOW_UNPINNED,,}" != "true" ]]; then
    echo "[INFO] Resetting to commit ${SRC_COMMIT_SHA}"
    git -C "${SRC_DIR}" reset --hard "${SRC_COMMIT_SHA}"
fi
SOURCE_HEAD="$(git -C "${SRC_DIR}" rev-parse HEAD)"
if [[ "${SRC_ALLOW_UNPINNED,,}" != "true" ]]; then
    if [[ "${SOURCE_HEAD}" != "${SRC_COMMIT_SHA}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Isaac Sim HEAD ${SOURCE_HEAD} does not match pinned commit ${SRC_COMMIT_SHA}\033[0m"
        exit 1
    fi
fi
if [[ ! -f "${SRC_DIR}/VERSION" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Isaac Sim source VERSION is missing from ${SRC_DIR}\033[0m"
    exit 1
fi
SOURCE_VERSION="$(tr -d '\r\n' < "${SRC_DIR}/VERSION")"
if [[ "${SOURCE_VERSION}" != "${EXPECTED_SOURCE_VERSION}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Isaac Sim source VERSION ${SOURCE_VERSION} does not match expected ${EXPECTED_SOURCE_VERSION}\033[0m"
    exit 1
fi

echo "[INFO] Pulling LFS objects"
git -C "${SRC_DIR}" lfs install
git -C "${SRC_DIR}" lfs pull

STATE_DIR="$(mktemp -d -t srb-isaacsim-state.XXXXXXXX)"
mkdir -p -- "${STATE_DIR}/vulkan"
GCC_ALTERNATIVE_STATE="$(sudo update-alternatives --query gcc 2>/dev/null || true)"
GXX_ALTERNATIVE_STATE="$(sudo update-alternatives --query g++ 2>/dev/null || true)"
for path in "${VULKAN_PATHS[@]}"; do
    _snapshot_vulkan_file "${path}"
done
HOST_STATE_SNAPSHOTTED=true

echo "[INFO] Setting up gcc-11/g++-11 as compiler alternatives"
COMPILER_MUTATED=true
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
sudo update-alternatives --set gcc /usr/bin/gcc-11
sudo update-alternatives --set g++ /usr/bin/g++-11

echo "[INFO] Building Isaac Sim 6.1 (this may take a while)"
touch "${SRC_DIR}/.eula_accepted"
"${SRC_DIR}/build.sh" --release-only --no-licensing

ARCH="$(uname -m)"
RELEASE_DIR="${SRC_DIR}/_build/linux-${ARCH}/release"
if [[ ! -d "${RELEASE_DIR}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Build output not found at ${RELEASE_DIR}\033[0m"
    exit 1
fi
for required in python.sh isaac-sim.sh VERSION; do
    if [[ ! -e "${RELEASE_DIR}/${required}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Build output is missing ${required}: ${RELEASE_DIR}\033[0m"
        exit 1
    fi
done
if [[ ! -x "${RELEASE_DIR}/python.sh" || ! -x "${RELEASE_DIR}/isaac-sim.sh" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Build launchers are not executable in ${RELEASE_DIR}\033[0m"
    exit 1
fi
RELEASE_VERSION="$(tr -d '\r\n' < "${RELEASE_DIR}/VERSION")"
if [[ -z "${RELEASE_VERSION}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Build output VERSION is empty: ${RELEASE_DIR}/VERSION\033[0m"
    exit 1
fi
echo "[INFO] Build output VERSION: ${RELEASE_VERSION}"

echo "[INFO] Copying release artifacts to ${DEST_DIR}"
cp -Lr -- "${RELEASE_DIR}/." "${DEST_DIR}/"
INSTALLED_VERSION="$(tr -d '\r\n' < "${DEST_DIR}/VERSION")"
if [[ "${INSTALLED_VERSION}" != "${RELEASE_VERSION}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Installed Isaac Sim VERSION ${INSTALLED_VERSION} differs from build output ${RELEASE_VERSION}\033[0m"
    exit 1
fi
ISAACSIM_PATH="${DEST_DIR}"
ISAACSIM_PYTHON_EXE="${DEST_DIR}/python.sh"
ISAACSIM_PYTHON="${ISAACSIM_PYTHON_EXE}"

echo "[INFO] Setting up Vulkan/EGL configuration"
if [[ ! -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
    VULKAN_MUTATED=true
    sudo mkdir -p /usr/share/glvnd/egl_vendor.d
    sudo tee /usr/share/glvnd/egl_vendor.d/10_nvidia.json > /dev/null << 'EJSON'
{
"file_format_version": "1.0.0",
"ICD": {
"library_path": "libEGL_nvidia.so.0"
}
}
EJSON
fi
if [[ ! -f /usr/share/glvnd/egl_vendor.d/50_mesa.json ]]; then
    VULKAN_MUTATED=true
    sudo mkdir -p /usr/share/glvnd/egl_vendor.d
    sudo tee /usr/share/glvnd/egl_vendor.d/50_mesa.json > /dev/null << 'EJSON'
{
"file_format_version": "1.0.0",
"ICD": {
"library_path": "libEGL_mesa.so.0"
}
}
EJSON
fi

VK_ICD_PATH=""
for candidate in /etc/vulkan/icd.d/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json; do
    if [[ -f "${candidate}" ]]; then
        VK_ICD_PATH="${candidate}"
        break
    fi
done
if [[ -z "${VK_ICD_PATH}" ]]; then
    echo "[INFO] NVIDIA Vulkan ICD not found, creating one"
    VULKAN_MUTATED=true
    sudo mkdir -p /etc/vulkan/icd.d
    sudo tee /etc/vulkan/icd.d/nvidia_icd.json > /dev/null << 'EJSON'
{
"file_format_version": "1.0.0",
"ICD": {
"library_path": "libGLX_nvidia.so.0",
"api_version": "1.3.194"
}
}
EJSON
    VK_ICD_PATH="/etc/vulkan/icd.d/nvidia_icd.json"
else
    echo "[INFO] Using existing Vulkan ICD at ${VK_ICD_PATH}"
fi
if [[ ! -f /etc/vulkan/implicit_layer.d/nvidia_layers.json ]]; then
    VULKAN_MUTATED=true
    sudo mkdir -p /etc/vulkan/implicit_layer.d
    sudo tee /etc/vulkan/implicit_layer.d/nvidia_layers.json > /dev/null << 'EJSON'
{
"file_format_version": "1.0.0",
"layer": {
"name": "VK_LAYER_NV_optimus",
"type": "INSTANCE",
"library_path": "libGLX_nvidia.so.0",
"api_version": "1.3.194",
"implementation_version": "1",
"description": "NVIDIA Optimus layer",
"functions": {
"vkGetInstanceProcAddr": "vk_optimusGetInstanceProcAddr",
"vkGetDeviceProcAddr": "vk_optimusGetDeviceProcAddr"
},
"enable_environment": {
"__NV_PRIME_RENDER_OFFLOAD": "1"
},
"disable_environment": {
"DISABLE_LAYER_NV_OPTIMUS_1": ""
}
}
}
EJSON
fi

echo "[INFO] Updating pip in the bundled Isaac Python environment"
"${ISAACSIM_PYTHON_EXE}" -m ensurepip
"${ISAACSIM_PYTHON_EXE}" -m pip install --upgrade pip

OMNI_HUB_WORK="$(mktemp -d -t srb-omni-hub-XXXXXXXXXX)"
OMNI_HUB_ARCHIVE="${OMNI_HUB_WORK}/omni_hub.zip"
OMNI_HUB_EXTRACT="${OMNI_HUB_WORK}/extract"
OMNI_HUB_SHA256_X86_64="${OMNI_HUB_SHA256_X86_64:-"1deb6c8d3e3eb76bac6be33d76edfdaaeae3e766241c71fe41fef4389f093c36"}"
OMNI_HUB_SHA256_AARCH64="${OMNI_HUB_SHA256_AARCH64:-"755e215e6c3b71740c43756bbdc15eb52322e76f8f7729739e1d441ed37d0e95"}"
if [[ "${ARCH}" == x86_64 ]]; then
    HUB_ARCH=x86_64
    OMNI_HUB_SHA256="${OMNI_HUB_SHA256_X86_64}"
elif [[ "${ARCH}" == aarch64 ]]; then
    HUB_ARCH=aarch64
    OMNI_HUB_SHA256="${OMNI_HUB_SHA256_AARCH64}"
else
    echo >&2 -e "\033[1;31m[ERROR] Omniverse Hub is unavailable for architecture '${ARCH}'; refusing to continue.\033[0m"
    exit 65
fi
if [[ ! "${OMNI_HUB_SHA256}" =~ ^[0-9a-f]{64}$ ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Omniverse Hub SHA-256 is not a full lowercase digest for ${HUB_ARCH}.\033[0m"
    exit 65
fi
OMNI_HUB_URL="https://api.ngc.nvidia.com/v2/resources/nvidia/omniverse/hub_workstation_cache/versions/2.0.0/files/omni_hub.linux-${HUB_ARCH}@2.0.0+4a4ab4a3.zip"
echo "[INFO] Downloading Omniverse Hub from ${OMNI_HUB_URL}"
if ! curl --proto "=https" --tlsv1.2 -sSfL "${OMNI_HUB_URL}" -o "${OMNI_HUB_ARCHIVE}"; then
    echo >&2 -e "\033[1;31m[ERROR] Failed to download Omniverse Hub\033[0m"
    exit 65
fi
echo "[INFO] Verifying Omniverse Hub archive digest"
if ! echo "${OMNI_HUB_SHA256}  ${OMNI_HUB_ARCHIVE}" | sha256sum --check --strict - >/dev/null; then
    echo >&2 -e "\033[1;31m[ERROR] Omniverse Hub archive failed its sha256 check; refusing to extract or execute it.\033[0m"
    exit 65
fi
mkdir -p -- "${OMNI_HUB_EXTRACT}"
if ! unzip -q "${OMNI_HUB_ARCHIVE}" -d "${OMNI_HUB_EXTRACT}"; then
    echo >&2 -e "\033[1;31m[ERROR] Failed to extract the verified Omniverse Hub archive\033[0m"
    exit 65
fi
if [[ ! -x "${OMNI_HUB_EXTRACT}/scripts/install.sh" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Verified Omniverse Hub archive has no executable scripts/install.sh\033[0m"
    exit 65
fi
echo "[INFO] Running verified Omniverse Hub installation script"
if ! "${OMNI_HUB_EXTRACT}/scripts/install.sh"; then
    echo >&2 -e "\033[1;31m[ERROR] Omniverse Hub installation failed\033[0m"
    exit 65
fi
rm -rf -- "${OMNI_HUB_WORK}"
OMNI_HUB_WORK=""

ISAACSIM_CACHE_ROOT="${ISAACSIM_CACHE_ROOT:-"${HOME:-/tmp}/.cache/srb/isaacsim-6.1.0-isaaclab-3.0.0"}"
ISAACSIM_ASSET_ROOT="${ISAACSIM_ASSET_ROOT:-"${DEST_DIR}/assets"}"
ISAACSIM_ASSET_ROOT_PROFILE="${ISAACSIM_ASSET_ROOT_PROFILE:-"isaac-sim-6.1"}"
ISAACSIM_CACHE="${ISAACSIM_CACHE:-"${ISAACSIM_CACHE_ROOT}"}"
OMNI_KIT_CACHE_ROOT="${OMNI_KIT_CACHE_ROOT:-"${ISAACSIM_CACHE}/kit"}"
OMNI_KIT_DATA_ROOT="${OMNI_KIT_DATA_ROOT:-"${ISAACSIM_CACHE}/data"}"
OMNI_KIT_LOG_ROOT="${OMNI_KIT_LOG_ROOT:-"${ISAACSIM_CACHE}/logs"}"
OMNI_KIT_CONFIG="${OMNI_KIT_CONFIG:-"${ISAACSIM_CACHE}/config"}"

echo "[INFO] Isaac Sim ${RELEASE_VERSION} has been installed to ${DEST_DIR}"
echo "[INFO] Verified source HEAD: ${SOURCE_HEAD}"
echo "[INFO] Verified source VERSION: ${SOURCE_VERSION}"
echo "[INFO] Bundled Python aliases and native paths:"
echo "[INFO]   export ISAACSIM_PATH='${ISAACSIM_PATH}'"
echo "[INFO]   export ISAACSIM_PYTHON_EXE='${ISAACSIM_PYTHON_EXE}'"
echo "[INFO]   export ISAACSIM_PYTHON='${ISAACSIM_PYTHON}'"
echo "[INFO]   export ISAACSIM_ASSET_ROOT='${ISAACSIM_ASSET_ROOT}'"
echo "[INFO]   export ISAACSIM_ASSET_ROOT_PROFILE='${ISAACSIM_ASSET_ROOT_PROFILE}'"
echo "[INFO]   export ISAACSIM_CACHE_ROOT='${ISAACSIM_CACHE_ROOT}'"
echo "[INFO]   export ISAACSIM_CACHE='${ISAACSIM_CACHE}'"
echo "[INFO]   export OMNI_KIT_CACHE_ROOT='${OMNI_KIT_CACHE_ROOT}'"
echo "[INFO]   export OMNI_KIT_DATA_ROOT='${OMNI_KIT_DATA_ROOT}'"
echo "[INFO]   export OMNI_KIT_LOG_ROOT='${OMNI_KIT_LOG_ROOT}'"
echo "[INFO]   export OMNI_KIT_CONFIG='${OMNI_KIT_CONFIG}'"
echo "[INFO]   export ISAAC_PATH='${DEST_DIR}'"
echo "[INFO]   export CARB_APP_PATH='${DEST_DIR}/kit'"
echo "[INFO]   export EXP_PATH='${DEST_DIR}/apps'"
echo "[INFO]   export LD_PRELOAD='${DEST_DIR}/kit/libcarb.so'"
echo "[INFO]   export VK_DRIVER_FILES='${VK_ICD_PATH}'"
echo "[INFO]   export OMNI_KIT_ALLOW_ROOT='1'"

INSTALL_PUBLISHED=true
