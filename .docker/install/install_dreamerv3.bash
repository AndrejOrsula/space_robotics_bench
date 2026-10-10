#!/usr/bin/env bash
### Usage: install_dreamerv3.bash [destination_directory]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OWNED_DIR_TOOL="${SCRIPT_DIR}/owned_directory.py"
INSTALL_OWNER="srb.installer.dreamerv3"
readonly _DREAMERV3_CANONICAL_COMMIT_SHA="4049794d4135e41c691f18da38a9af7541b01553"
SRC_REPOSITORY="${SRC_REPOSITORY:-"https://github.com/AndrejOrsula/dreamerv3.git"}"
SRC_REPOSITORY_DISPLAY="$(printf '%s' "${SRC_REPOSITORY}" | sed -E 's#^([A-Za-z][A-Za-z0-9+.-]*://)[^/@]+@#\1***@#')"
SRC_BRANCH="${SRC_BRANCH:-"main"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/dreamerv3"}"}"
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
echo "[INFO] Source commit: ${_DREAMERV3_CANONICAL_COMMIT_SHA}"
echo "[INFO] Destination path: ${DEST_DIR}"

if [[ -z "${ISAACSIM_PYTHON:-}" ]]; then
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

SOURCE_STAGE="$(mktemp -d -- "${DEST_PARENT}/.dreamerv3-clone.XXXXXXXX")"
echo "[INFO] Cloning repository to ${DEST_DIR}"
git clone --branch "${SRC_BRANCH}" "${SRC_REPOSITORY}" "${SOURCE_STAGE}/checkout"
shopt -s dotglob nullglob
mv -- "${SOURCE_STAGE}/checkout/"* "${DEST_DIR}/"
shopt -u dotglob nullglob
rm -rf -- "${SOURCE_STAGE}"
SOURCE_STAGE=""
printf '%s\n' "/.srb-owned-directory.json" >> "${DEST_DIR}/.git/info/exclude"

echo "[INFO] Fetching canonical commit ${_DREAMERV3_CANONICAL_COMMIT_SHA}"
git -C "${DEST_DIR}" fetch --no-tags "${SRC_REPOSITORY}" "${_DREAMERV3_CANONICAL_COMMIT_SHA}"
echo "[INFO] Resetting to commit ${_DREAMERV3_CANONICAL_COMMIT_SHA}"
git -C "${DEST_DIR}" reset --hard "${_DREAMERV3_CANONICAL_COMMIT_SHA}"

SOURCE_HEAD="$(git -C "${DEST_DIR}" rev-parse HEAD)"
if [[ "${SOURCE_HEAD}" != "${_DREAMERV3_CANONICAL_COMMIT_SHA}" ]]; then
    printf '%s\n' "[ERROR] DreamerV3 HEAD does not match canonical commit" >&2
    exit 1
fi
INTERPRETER_MUTATED=true
echo "[INFO] Installing DreamerV3 in editable mode"
"${ISAACSIM_PYTHON}" -m pip install --editable "${DEST_DIR}"

echo "[INFO] DreamerV3 has been installed as an editable package from ${DEST_DIR}"
INSTALL_PUBLISHED=true
