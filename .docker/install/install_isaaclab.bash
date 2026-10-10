#!/usr/bin/env bash
### Usage: install_isaaclab.bash [destination_directory]
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OWNED_DIR_TOOL="${SCRIPT_DIR}/owned_directory.py"
INSTALL_OWNER="srb.installer.isaaclab"
SRC_REPOSITORY="${SRC_REPOSITORY:-"https://github.com/isaac-sim/IsaacLab.git"}"
SRC_REPOSITORY_DISPLAY="$(printf '%s' "${SRC_REPOSITORY}" | sed -E 's#^([A-Za-z][A-Za-z0-9+.-]*://)[^/@]+@#\1***@#')"
SRC_BRANCH="${SRC_BRANCH:-"release/3.0.0"}"
SRC_COMMIT_SHA="${SRC_COMMIT_SHA:-"fdc650c88dfd86aa158b9a1760b6537fef360555"}"
EXPECTED_LAB_VERSION="${ISAACLAB_VERSION:-"3.0.0"}"
DEST_DIR="${1:-"${DEST_DIR:-"${HOME:-/tmp}/isaaclab"}"}"
DEST_DIR="$(realpath -m -- "${DEST_DIR}")"
DEST_PARENT="$(dirname -- "${DEST_DIR}")"
ISAACSIM_PATH="${ISAACSIM_PATH:-}"
ISAACSIM_PYTHON_EXE="${ISAACSIM_PYTHON_EXE:-${ISAACSIM_PYTHON:-}}"
SOURCE_STAGE=""
EXTENSION_LIST=""
DEST_MARKED=false
INSTALL_PUBLISHED=false
INTERPRETER_MUTATED=false

SRC_ALLOW_UNPINNED="${SRC_ALLOW_UNPINNED:-false}"
if [[ "${SRC_ALLOW_UNPINNED,,}" == "true" ]]; then
    echo >&2 -e "\033[1;33m[WARNING] SRC_ALLOW_UNPINNED=true: installing the '${SRC_BRANCH}' tip of ${SRC_REPOSITORY_DISPLAY} without verifying any commit. Unverified upstream code will be executed.\033[0m"
elif [[ ! "${SRC_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]]; then
    echo >&2 -e "\033[1;31m[ERROR] SRC_COMMIT_SHA must be a full 40-character lowercase hex commit (got '${SRC_COMMIT_SHA}'). Set SRC_ALLOW_UNPINNED=true to install an unverified branch tip instead.\033[0m"
    exit 1
fi

if [[ -z "${ISAACSIM_PYTHON_EXE}" && -n "${ISAACSIM_PATH}" ]]; then
    ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
fi
if [[ -z "${ISAACSIM_PYTHON_EXE}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAACSIM_PYTHON_EXE or ISAACSIM_PYTHON must point to the bundled Isaac Sim python.sh.\033[0m"
    exit 1
fi
if [[ ! -x "${ISAACSIM_PYTHON_EXE}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAACSIM_PYTHON_EXE is not an executable bundled Isaac Sim interpreter: ${ISAACSIM_PYTHON_EXE}\033[0m"
    exit 1
fi
if [[ -n "${ISAACSIM_PYTHON:-}" && "$(realpath -e -- "${ISAACSIM_PYTHON}")" != "$(realpath -e -- "${ISAACSIM_PYTHON_EXE}")" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAACSIM_PYTHON and ISAACSIM_PYTHON_EXE must identify the same bundled python.sh.\033[0m"
    exit 1
fi
ISAACSIM_PYTHON="${ISAACSIM_PYTHON_EXE}"
if [[ -z "${ISAACSIM_PATH}" ]]; then
    ISAACSIM_PATH="$(dirname -- "${ISAACSIM_PYTHON_EXE}")"
fi
ISAACSIM_PATH="$(realpath -e -- "${ISAACSIM_PATH}")"
if [[ ! -x "${ISAACSIM_PATH}/python.sh" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAACSIM_PATH must contain an executable python.sh.\033[0m"
    exit 1
fi
if [[ "$(realpath -e -- "${ISAACSIM_PYTHON_EXE}")" != "$(realpath -e -- "${ISAACSIM_PATH}/python.sh")" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAACSIM_PYTHON_EXE must point to ISAACSIM_PATH/python.sh.\033[0m"
    exit 1
fi
if [[ ! -f "${ISAACSIM_PATH}/VERSION" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Isaac Sim VERSION is missing from ISAACSIM_PATH: ${ISAACSIM_PATH}\033[0m"
    exit 1
fi
SIM_VERSION="$(tr -d '\r\n' < "${ISAACSIM_PATH}/VERSION")"
if [[ -z "${SIM_VERSION}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Isaac Sim VERSION is empty: ${ISAACSIM_PATH}/VERSION\033[0m"
    exit 1
fi

echo "[INFO] Source repository: ${SRC_REPOSITORY_DISPLAY}"
echo "[INFO] Source branch: ${SRC_BRANCH}"
echo "[INFO] Source commit: ${SRC_COMMIT_SHA}"
echo "[INFO] Expected Isaac Lab VERSION: ${EXPECTED_LAB_VERSION}"
echo "[INFO] Isaac Sim VERSION: ${SIM_VERSION}"
echo "[INFO] ISAACSIM_PATH: ${ISAACSIM_PATH}"
echo "[INFO] ISAACSIM_PYTHON_EXE: ${ISAACSIM_PYTHON_EXE}"
echo "[INFO] ISAACSIM_PYTHON: ${ISAACSIM_PYTHON}"
echo "[INFO] Destination path: ${DEST_DIR}"

cleanup() {
    local exit_status="${1:-0}"
    local cleanup_failed=0
    set +e
    if [[ "${DEST_MARKED}" == true && "${INSTALL_PUBLISHED}" != true ]]; then
        if [[ "${INTERPRETER_MUTATED}" == true ]]; then
            echo >&2 -e "\033[1;33m[WARNING] Installation failed after the Python interpreter was modified. Keeping ${DEST_DIR} because editable installs may point into it. Rerun this installer with a new destination, which reinstalls every extension from there, then remove the kept checkout by hand.\033[0m"
        else
            python3 "${OWNED_DIR_TOOL}" remove --path "${DEST_DIR}" --owner "${INSTALL_OWNER}" || cleanup_failed=1
        fi
    fi
    if [[ -n "${SOURCE_STAGE}" && -d "${SOURCE_STAGE}" ]]; then
        rm -rf -- "${SOURCE_STAGE}"
    fi
    if [[ -n "${EXTENSION_LIST}" ]]; then
        rm -f -- "${EXTENSION_LIST}"
    fi
    set -e
    if (( exit_status == 0 && cleanup_failed != 0 )); then
        exit_status=1
    fi
    return "${exit_status}"
}
trap 'exit_status=$?; cleanup "${exit_status}"; exit "${exit_status}"' EXIT

for cmd in git realpath mktemp mv mkdir find sed tr python3; do
    if ! command -v "${cmd}" &>/dev/null; then
        echo >&2 -e "\033[1;31m[ERROR] Required command '${cmd}' is not installed.\033[0m"
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

SOURCE_STAGE="$(mktemp -d -t isaac-lab-src.XXXXXXXX)"
echo "[INFO] Cloning repository to ${SOURCE_STAGE}"
git clone --branch "${SRC_BRANCH}" "${SRC_REPOSITORY}" "${SOURCE_STAGE}"
if [[ "${SRC_ALLOW_UNPINNED,,}" != "true" ]]; then
    echo "[INFO] Resetting to commit ${SRC_COMMIT_SHA}"
    git -C "${SOURCE_STAGE}" reset --hard "${SRC_COMMIT_SHA}"
fi
SOURCE_HEAD="$(git -C "${SOURCE_STAGE}" rev-parse HEAD)"
if [[ "${SRC_ALLOW_UNPINNED,,}" != "true" ]]; then
    if [[ "${SOURCE_HEAD}" != "${SRC_COMMIT_SHA}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Isaac Lab HEAD ${SOURCE_HEAD} does not match pinned commit ${SRC_COMMIT_SHA}\033[0m"
        exit 1
    fi
fi
if [[ ! -f "${SOURCE_STAGE}/VERSION" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Isaac Lab source VERSION is missing from ${SOURCE_STAGE}\033[0m"
    exit 1
fi
SOURCE_VERSION="$(tr -d '\r\n' < "${SOURCE_STAGE}/VERSION")"
if [[ "${SOURCE_VERSION}" != "${EXPECTED_LAB_VERSION}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Isaac Lab source VERSION ${SOURCE_VERSION} does not match expected ${EXPECTED_LAB_VERSION}\033[0m"
    exit 1
fi

echo "[INFO] Publishing verified Isaac Lab checkout to ${DEST_DIR}"
shopt -s dotglob nullglob
SOURCE_ENTRIES=("${SOURCE_STAGE}"/*)
if (( ${#SOURCE_ENTRIES[@]} == 0 )); then
    shopt -u dotglob nullglob
    echo >&2 -e "\033[1;31m[ERROR] Verified Isaac Lab checkout is empty: ${SOURCE_STAGE}\033[0m"
    exit 1
fi
mv -- "${SOURCE_ENTRIES[@]}" "${DEST_DIR}/"
shopt -u dotglob nullglob
SOURCE_STAGE=""

ISAACSIM_ASSET_ROOT="${ISAACSIM_ASSET_ROOT:-"${ISAACSIM_PATH}/assets"}"
ISAACSIM_ASSET_ROOT_PROFILE="${ISAACSIM_ASSET_ROOT_PROFILE:-"isaac-sim-6.1"}"
ISAACSIM_CACHE_ROOT="${ISAACSIM_CACHE_ROOT:-"${HOME:-/tmp}/.cache/srb/isaacsim-6.1.0-isaaclab-3.0.0"}"
ISAACSIM_CACHE="${ISAACSIM_CACHE:-"${ISAACSIM_CACHE_ROOT}"}"
OMNI_KIT_CACHE_ROOT="${OMNI_KIT_CACHE_ROOT:-"${ISAACSIM_CACHE}/kit"}"
OMNI_KIT_DATA_ROOT="${OMNI_KIT_DATA_ROOT:-"${ISAACSIM_CACHE}/data"}"
OMNI_KIT_LOG_ROOT="${OMNI_KIT_LOG_ROOT:-"${ISAACSIM_CACHE}/logs"}"
OMNI_KIT_CONFIG="${OMNI_KIT_CONFIG:-"${ISAACSIM_CACHE}/config"}"
export ISAACSIM_PATH ISAACSIM_PYTHON_EXE ISAACSIM_PYTHON
export ISAACSIM_ASSET_ROOT ISAACSIM_ASSET_ROOT_PROFILE
export ISAACSIM_CACHE_ROOT ISAACSIM_CACHE OMNI_KIT_CACHE_ROOT OMNI_KIT_DATA_ROOT OMNI_KIT_LOG_ROOT OMNI_KIT_CONFIG
export SRB_ISAACLAB_SOURCE="${DEST_DIR}/source/isaaclab"

ln -sfn -- "${ISAACSIM_PATH}" "${DEST_DIR}/_isaac_sim"

INTERPRETER_MUTATED=true
echo "[INFO] Updating pip in the bundled Isaac Sim environment"
"${ISAACSIM_PYTHON_EXE}" -m ensurepip
"${ISAACSIM_PYTHON_EXE}" -m pip install --upgrade pip

echo "[INFO] Installing pip-only dependencies needed by Isaac Lab editable installs"
"${ISAACSIM_PYTHON_EXE}" -m pip install "lazy_loader>=0.4"

echo "[INFO] Masking NVIDIA-private deps in isaaclab and isaaclab_teleop pyproject.toml"
for masked in "isaaclab:omniverseclient" "isaaclab_teleop:isaacteleop"; do
    ext_name="${masked%%:*}"
    pkg_name="${masked##*:}"
    pyproject="${DEST_DIR}/source/${ext_name}/pyproject.toml"
    if [[ -f "${pyproject}" ]]; then
        sed -i -E "/^[[:space:]]*\"${pkg_name}([[:space:]]|\[|~|=|>|<|,|\").*$/d" "${pyproject}"
    fi
done

EXTENSION_LIST="$(mktemp)"
find -L "${DEST_DIR}/source" -mindepth 1 -maxdepth 1 -type d -print0 > "${EXTENSION_LIST}"
INSTALLED_EXTENSIONS=0
while IFS= read -r -d '' extension; do
    if [[ -f "${extension}/pyproject.toml" || -f "${extension}/setup.py" ]]; then
        echo "[INFO] Installing ${extension}"
        "${ISAACSIM_PYTHON_EXE}" -m pip install --editable "${extension}"
        INSTALLED_EXTENSIONS=$((INSTALLED_EXTENSIONS + 1))
    fi
done < "${EXTENSION_LIST}"
rm -f -- "${EXTENSION_LIST}"
EXTENSION_LIST=""
if [[ "${INSTALLED_EXTENSIONS}" -eq 0 ]]; then
    echo >&2 -e "\033[1;31m[ERROR] No Isaac Lab extension was found under ${DEST_DIR}/source\033[0m"
    exit 1
fi

echo "[INFO] Fixing numpy version and installing compatibility packages"
"${ISAACSIM_PYTHON_EXE}" -m pip install "numpy>=2.1" array_api_compat

INSTALLED_HEAD="$(git -C "${DEST_DIR}" rev-parse HEAD)"
if [[ "${SRC_ALLOW_UNPINNED,,}" != "true" ]]; then
    if [[ "${INSTALLED_HEAD}" != "${SRC_COMMIT_SHA}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Installed Isaac Lab HEAD ${INSTALLED_HEAD} does not match pinned commit ${SRC_COMMIT_SHA}\033[0m"
        exit 1
    fi
fi
INSTALLED_VERSION="$(tr -d '\r\n' < "${DEST_DIR}/VERSION")"
if [[ "${INSTALLED_VERSION}" != "${EXPECTED_LAB_VERSION}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] Installed Isaac Lab VERSION ${INSTALLED_VERSION} does not match expected ${EXPECTED_LAB_VERSION}\033[0m"
    exit 1
fi

echo "[INFO] Isaac Lab ${INSTALLED_VERSION} has been installed as editable packages from ${DEST_DIR}/source"
echo "[INFO] Verified source HEAD: ${INSTALLED_HEAD}"
echo "[INFO] Isaac Sim path/version: ${ISAACSIM_PATH} (${SIM_VERSION})"
echo "[INFO]   export ISAACSIM_PATH='${ISAACSIM_PATH}'"
echo "[INFO]   export ISAACSIM_PYTHON_EXE='${ISAACSIM_PYTHON_EXE}'"
echo "[INFO]   export ISAACSIM_PYTHON='${ISAACSIM_PYTHON}'"
echo "[INFO]   export SRB_ISAACLAB_SOURCE='${SRB_ISAACLAB_SOURCE}'"
echo "[INFO]   export ISAACSIM_ASSET_ROOT='${ISAACSIM_ASSET_ROOT}'"
echo "[INFO]   export ISAACSIM_ASSET_ROOT_PROFILE='${ISAACSIM_ASSET_ROOT_PROFILE}'"
echo "[INFO]   export ISAACSIM_CACHE_ROOT='${ISAACSIM_CACHE_ROOT}'"
echo "[INFO]   export ISAACSIM_CACHE='${ISAACSIM_CACHE}'"
echo "[INFO]   export OMNI_KIT_CACHE_ROOT='${OMNI_KIT_CACHE_ROOT}'"
echo "[INFO]   export OMNI_KIT_DATA_ROOT='${OMNI_KIT_DATA_ROOT}'"
echo "[INFO]   export OMNI_KIT_LOG_ROOT='${OMNI_KIT_LOG_ROOT}'"
echo "[INFO]   export OMNI_KIT_CONFIG='${OMNI_KIT_CONFIG}'"

INSTALL_PUBLISHED=true
