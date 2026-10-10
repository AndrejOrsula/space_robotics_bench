#!/usr/bin/env bash
### Usage: install_rsl_rl.bash [destination_directory]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OWNED_DIR_TOOL="${SCRIPT_DIR}/owned_directory.py"
INSTALL_OWNER="srb.installer.rsl-rl"
SRC_REPOSITORY="${SRC_REPOSITORY:-"https://github.com/leggedrobotics/rsl_rl.git"}"
SRC_REPOSITORY_DISPLAY="$(printf '%s' "${SRC_REPOSITORY}" | sed -E 's#^([A-Za-z][A-Za-z0-9+.-]*://)[^/@]+@#\1***@#')"
SRC_BRANCH="${SRC_BRANCH:-"main"}"
SRC_COMMIT_SHA="${SRC_COMMIT_SHA:-"c7beb6f02f71abfbbd9cd52d37bbe8377837b406"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/rsl_rl"}"}"
DEST_DIR="$(realpath -m -- "${DEST_DIR}")"
DEST_MARKED=false
INSTALL_PUBLISHED=false
INTERPRETER_MUTATED=false
SOURCE_STAGE=""

cleanup() {
    local exit_status="${1:-0}"
    local cleanup_failed=0
    set +e
    if [[ "${DEST_MARKED}" == true && "${INSTALL_PUBLISHED}" != true ]]; then
        if [[ "${INTERPRETER_MUTATED}" == true ]]; then
            echo >&2 -e "\033[1;33m[WARNING] Installation failed after the Python interpreter was modified. Keeping ${DEST_DIR} because editable installs may point into it. Rerun this installer and confirm the overwrite to reinstall from a fresh checkout.\033[0m"
        else
            python3 "${OWNED_DIR_TOOL}" remove --path "${DEST_DIR}" --owner "${INSTALL_OWNER}" || cleanup_failed=1
        fi
    fi
    if [[ -n "${SOURCE_STAGE}" && -d "${SOURCE_STAGE}" ]]; then
        rm -rf -- "${SOURCE_STAGE}" || cleanup_failed=1
    fi
    set -e
    if (( exit_status == 0 && cleanup_failed != 0 )); then
        exit_status=1
    fi
    return "${exit_status}"
}
trap 'exit_status=$?; cleanup "${exit_status}"; exit "${exit_status}"' EXIT

echo "[INFO] Source repository: ${SRC_REPOSITORY_DISPLAY}"
echo "[INFO] Source branch: ${SRC_BRANCH}"
echo "[INFO] Source commit: ${SRC_COMMIT_SHA}"
echo "[INFO] Destination path: ${DEST_DIR}"

SRC_ALLOW_UNPINNED="${SRC_ALLOW_UNPINNED:-false}"
if [[ "${SRC_ALLOW_UNPINNED,,}" == "true" ]]; then
    echo >&2 -e "\033[1;33m[WARNING] SRC_ALLOW_UNPINNED=true: installing the '${SRC_BRANCH}' tip of ${SRC_REPOSITORY_DISPLAY} without verifying any commit. Unverified upstream code will be executed.\033[0m"
elif [[ ! "${SRC_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]]; then
    echo >&2 -e "\033[1;31m[ERROR] SRC_COMMIT_SHA must be a full 40-character lowercase hex commit (got '${SRC_COMMIT_SHA}'). Set SRC_ALLOW_UNPINNED=true to install an unverified branch tip instead.\033[0m"
    exit 1
fi

if [[ -z "${ISAACSIM_PYTHON}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAACSIM_PYTHON is not set. Please set it to the path of the Python executable in the Isaac Sim environment.\033[0m"
    exit 1
fi
if [[ ! -x "${ISAACSIM_PYTHON}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAACSIM_PYTHON is not a valid Python executable. Please set it to the path of the Python executable in the Isaac Sim environment.\033[0m"
    exit 1
fi

if [[ -d "${DEST_DIR}" ]]; then
    echo -en "\033[1;33m[WARNING] Destination directory already exists, overwrite? [y/N]\033[0m "
    read -r
    if [[ ! "${REPLY}" =~ ^[Yy]$ ]]; then
        echo "[INFO] Exiting"
        exit 0
    fi
    python3 "${OWNED_DIR_TOOL}" remove --path "${DEST_DIR}" --owner "${INSTALL_OWNER}"
fi

DEST_PARENT="$(dirname -- "${DEST_DIR}")"
mkdir -p -- "${DEST_PARENT}"
mkdir -- "${DEST_DIR}"
if ! python3 "${OWNED_DIR_TOOL}" mark-created --path "${DEST_DIR}" --owner "${INSTALL_OWNER}"; then
    if ! rmdir -- "${DEST_DIR}"; then
        echo >&2 -e "\033[1;33m[WARNING] Could not remove unmarked destination directory: ${DEST_DIR}\033[0m"
    fi
    echo >&2 -e "\033[1;31m[ERROR] Could not claim the disposable destination safely: ${DEST_DIR}\033[0m"
    exit 73
fi
DEST_MARKED=true

SOURCE_STAGE="$(mktemp -d -- "${DEST_PARENT}/.rsl_rl-clone.XXXXXXXX")"
echo "[INFO] Cloning repository to ${DEST_DIR}"
git clone --branch "${SRC_BRANCH}" "${SRC_REPOSITORY}" "${SOURCE_STAGE}/checkout"
shopt -s dotglob nullglob
mv -- "${SOURCE_STAGE}/checkout/"* "${DEST_DIR}/"
shopt -u dotglob nullglob
rm -rf -- "${SOURCE_STAGE}"
SOURCE_STAGE=""
printf '%s\n' "/.srb-owned-directory.json" >> "${DEST_DIR}/.git/info/exclude"
if [[ "${SRC_ALLOW_UNPINNED,,}" != "true" ]]; then
    echo "[INFO] Resetting to commit ${SRC_COMMIT_SHA}"
    git -C "${DEST_DIR}" reset --hard "${SRC_COMMIT_SHA}"
    SOURCE_HEAD="$(git -C "${DEST_DIR}" rev-parse HEAD)"
    if [[ "${SOURCE_HEAD}" != "${SRC_COMMIT_SHA}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] rsl_rl HEAD ${SOURCE_HEAD} does not match pinned commit ${SRC_COMMIT_SHA}\033[0m"
        exit 1
    fi
fi
INTERPRETER_MUTATED=true
echo "[INFO] Installing rsl_rl in editable mode"
"${ISAACSIM_PYTHON}" -m pip install --editable "${DEST_DIR}"

echo "[INFO] rsl_rl has been installed as an editable package from ${DEST_DIR}"
INSTALL_PUBLISHED=true
