#!/usr/bin/env bash
### Usage: install_blender.bash [destination_directory]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OWNED_DIR_TOOL="${SCRIPT_DIR}/owned_directory.py"
INSTALL_OWNER="srb.installer.blender"
BLENDER_VERSION="${BLENDER_VERSION:-"4.5.8"}"
BLENDER_VERSION_SHORT="${BLENDER_VERSION%.*}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/blender"}"}"
DEST_DIR="$(realpath -m -- "${DEST_DIR}")"
BLENDER_PYTHON="${DEST_DIR}/${BLENDER_VERSION_SHORT}/python/bin/python3.11"
BLENDER_SHA256_AMD64="${BLENDER_SHA256_AMD64:-"8cc3997ca2148a43187ca625f150b41bd3ef7c2991988725a34b46cbf25ba82f"}"
BLENDER_SHA256_ARM64="${BLENDER_SHA256_ARM64:-""}"
SIMFORGE_VERSION="${SIMFORGE_VERSION:-"0.4.0"}"
SIMFORGE_FOUNDRY_VERSION="${SIMFORGE_FOUNDRY_VERSION:-"0.3.1"}"
SIMFORGE_PATCH_SCRIPT="${SCRIPT_DIR}/patch_simforge_isaaclab.bash"
for pin_name in SIMFORGE_VERSION SIMFORGE_FOUNDRY_VERSION; do
    if [[ ! "${!pin_name}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo >&2 -e "\033[1;31m[ERROR] ${pin_name} must be an exact X.Y.Z release (got '${!pin_name}').\033[0m"
        exit 64
    fi
done
WORK_DIR="$(mktemp -d -t srb-blender-XXXXXXXXXX)"
ARCHIVE_PATH="${WORK_DIR}/blender-${BLENDER_VERSION}.tar.xz"
DEST_MARKED=false
INSTALL_COMPLETE=false

echo "[INFO] Blender version: ${BLENDER_VERSION}"
echo "[INFO] Destination path: ${DEST_DIR}"

cleanup() {
    local exit_status=$?
    trap - EXIT
    set +e
    if [[ "${exit_status}" -ne 0 && "${DEST_MARKED}" == true && "${INSTALL_COMPLETE}" != true ]]; then
        if ! python3 "${OWNED_DIR_TOOL}" remove --path "${DEST_DIR}" --owner "${INSTALL_OWNER}"; then
            echo >&2 -e "\033[1;33m[WARNING] Installation failed and the owned destination could not be removed: ${DEST_DIR}\033[0m"
        fi
    fi
    echo "[INFO] Cleaning up temporary files"
    rm -rf -- "${WORK_DIR}"
}
trap cleanup EXIT

if [[ -d "${DEST_DIR}" ]]; then
    echo -en "\033[1;33m[WARNING] Destination directory already exists, overwrite? [y/N]\033[0m "
    read -r
    if [[ ! "${REPLY}" =~ ^[Yy]$ ]]; then
        echo "[INFO] Exiting"
        exit 0
    fi
    python3 "${OWNED_DIR_TOOL}" remove --path "${DEST_DIR}" --owner "${INSTALL_OWNER}"
fi

ARCH="$(uname -m)"
if [[ "${ARCH}" == "x86_64" ]]; then
    BLENDER_ARCH="linux-x64"
    BLENDER_SHA256="${BLENDER_SHA256_AMD64}"
elif [[ "${ARCH}" == "aarch64" ]]; then
    BLENDER_ARCH="linux-aarch64"
    BLENDER_SHA256="${BLENDER_SHA256_ARM64}"
else
    echo >&2 -e "\033[1;31m[ERROR] Unsupported architecture: ${ARCH}\033[0m"
    exit 1
fi

SRC_URL="https://download.blender.org/release/Blender${BLENDER_VERSION_SHORT}/blender-${BLENDER_VERSION}-${BLENDER_ARCH}.tar.xz"
echo "[INFO] Downloading Blender from ${SRC_URL}"
if [[ -z "${BLENDER_SHA256}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] No Blender ${BLENDER_VERSION} digest is pinned for ${ARCH}; set BLENDER_SHA256_ARM64.\033[0m"
    exit 64
fi
curl --proto "=https" --tlsv1.2 -sSfL "${SRC_URL}" -o "${ARCHIVE_PATH}"

echo "[INFO] Verifying archive digest"
echo "${BLENDER_SHA256}  ${ARCHIVE_PATH}" | sha256sum --check --strict -

echo "[INFO] Extracting archive to ${DEST_DIR}"
mkdir -p -- "$(dirname -- "${DEST_DIR}")"
if ! mkdir -- "${DEST_DIR}"; then
    echo >&2 -e "\033[1;31m[ERROR] Destination appeared during installation: ${DEST_DIR}\033[0m"
    exit 73
fi
if ! python3 "${OWNED_DIR_TOOL}" mark-created --path "${DEST_DIR}" --owner "${INSTALL_OWNER}"; then
    if ! rmdir -- "${DEST_DIR}" 2>/dev/null; then
        echo >&2 -e "\033[1;33m[WARNING] Could not remove the unmarked destination directory: ${DEST_DIR}\033[0m"
    fi
    echo >&2 -e "\033[1;31m[ERROR] Failed to mark the Blender destination as owned: ${DEST_DIR}\033[0m"
    exit 1
fi
DEST_MARKED=true
tar xf "${ARCHIVE_PATH}" -C "${DEST_DIR}" --strip-components=1

echo "[INFO] Installing SimForge into Blender's Python environment"
if [[ -f "${BLENDER_PYTHON}" ]]; then
    "${BLENDER_PYTHON}" -m ensurepip
    "${BLENDER_PYTHON}" -m pip install "simforge[dev]==${SIMFORGE_VERSION}" "simforge-foundry==${SIMFORGE_FOUNDRY_VERSION}"
    INSTALLED_SIMFORGE="$("${BLENDER_PYTHON}" -c 'from importlib.metadata import version; print(version("simforge"))')"
    INSTALLED_FOUNDRY="$("${BLENDER_PYTHON}" -c 'from importlib.metadata import version; print(version("simforge-foundry"))')"
    if [[ "${INSTALLED_SIMFORGE}" != "${SIMFORGE_VERSION}" || "${INSTALLED_FOUNDRY}" != "${SIMFORGE_FOUNDRY_VERSION}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] Blender's Python installed simforge ${INSTALLED_SIMFORGE} and simforge-foundry ${INSTALLED_FOUNDRY}; expected ${SIMFORGE_VERSION} and ${SIMFORGE_FOUNDRY_VERSION}\033[0m"
        exit 1
    fi
    SIMFORGE_BLENDER_SITE="$("${BLENDER_PYTHON}" -c 'import importlib.util, pathlib; print(pathlib.Path(importlib.util.find_spec("simforge").origin).parent)')"
    echo "[INFO] Applying the IsaacLab compatibility patch to ${SIMFORGE_BLENDER_SITE}"
    bash "${SIMFORGE_PATCH_SCRIPT}" "${SIMFORGE_BLENDER_SITE}"
else
    echo >&2 -e "\033[1;33m[WARNING] Blender Python not found at ${BLENDER_PYTHON}, skipping SimForge installation\033[0m"
fi

SYMLINK_DIR="$HOME/.local/bin"
mkdir -p "${SYMLINK_DIR}"
ln -sf "${DEST_DIR}/blender" "${SYMLINK_DIR}/blender"
echo "[INFO] Created symlink: ${SYMLINK_DIR}/blender -> ${DEST_DIR}/blender"

echo "[INFO] Blender ${BLENDER_VERSION} has been installed to ${DEST_DIR}"
echo "[INFO] Ensure that '${SYMLINK_DIR}' is in your PATH"
echo ""
echo -e "\033[1;33m[WARNING] Avoid installing Blender through Snap, as it would prevent the integration of the required Python dependencies within its environment.\033[0m"
INSTALL_COMPLETE=true
