#!/usr/bin/env bash
### Usage: install_robomimic.bash [destination_directory]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OWNED_DIR_TOOL="${SCRIPT_DIR}/owned_directory.py"
INSTALL_OWNER="srb.installer.robomimic"
SRC_REPOSITORY="${SRC_REPOSITORY:-"https://github.com/ARISE-Initiative/robomimic.git"}"
SRC_REPOSITORY_DISPLAY="$(printf '%s' "${SRC_REPOSITORY}" | sed -E 's#^([A-Za-z][A-Za-z0-9+.-]*://)[^/@]+@#\1***@#')"
SRC_BRANCH="${SRC_BRANCH:-"master"}"
SRC_COMMIT_SHA="${SRC_COMMIT_SHA:-"e10526b9a40c78b41f1e37e60041dc0ec0a5f60f"}"
PATCH_FILE="${REPO_ROOT}/srb/integrations/robomimic/patches/0001-lazy-language-provider.patch"
SRC_PATCHED_COMMIT_SHA="${SRC_PATCHED_COMMIT_SHA:-"9e88eb3288079c6014f912e8d805c76dcd40157a"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/robomimic"}"}"
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
elif [[ ! "${SRC_PATCHED_COMMIT_SHA}" =~ ^[0-9a-f]{40}$ ]]; then
    echo >&2 -e "\033[1;31m[ERROR] SRC_PATCHED_COMMIT_SHA must be a full 40-character lowercase hex commit (got '${SRC_PATCHED_COMMIT_SHA}').\033[0m"
    exit 1
fi
if [[ ! -f "${PATCH_FILE}" || -L "${PATCH_FILE}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] robomimic patch is not a regular file: ${PATCH_FILE}\033[0m"
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

SOURCE_STAGE="$(mktemp -d -- "${DEST_PARENT}/.robomimic-clone.XXXXXXXX")"
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
        echo >&2 -e "\033[1;31m[ERROR] robomimic HEAD ${SOURCE_HEAD} does not match pinned commit ${SRC_COMMIT_SHA}\033[0m"
        exit 1
    fi
fi
echo "[INFO] Applying ${PATCH_FILE#"${REPO_ROOT}/"}"
GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_NOSYSTEM=1 \
    GIT_COMMITTER_NAME="Space Robotics Bench" \
    GIT_COMMITTER_EMAIL="srb@localhost" \
    git -C "${DEST_DIR}" -c commit.gpgsign=false \
    am -C0 --committer-date-is-author-date "${PATCH_FILE}"
if [[ "${SRC_ALLOW_UNPINNED,,}" != "true" ]]; then
    PATCHED_HEAD="$(git -C "${DEST_DIR}" rev-parse HEAD)"
    if [[ "${PATCHED_HEAD}" != "${SRC_PATCHED_COMMIT_SHA}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] patched robomimic HEAD ${PATCHED_HEAD} does not match pinned patched commit ${SRC_PATCHED_COMMIT_SHA}\033[0m"
        exit 1
    fi
fi
INTERPRETER_MUTATED=true
echo "[INFO] Installing robomimic in editable mode"
"${ISAACSIM_PYTHON}" -m pip install --editable "${DEST_DIR}"

echo "[INFO] robomimic has been installed as an editable package from ${DEST_DIR}"
INSTALL_PUBLISHED=true
