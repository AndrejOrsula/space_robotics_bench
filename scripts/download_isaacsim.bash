#!/usr/bin/env bash
### Download Isaac Sim
### Usage: download_isaacsim.bash [destination_directory]
set -e

## Config
SRC_URL="${SRC_URL:-"https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.5.0-rc.36%2Brelease.19112.f59b3005.gl.linux-x86_64.release.zip"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/isaac-sim"}"}"
ARCHIVE_PATH="/tmp/isaac-sim-$(date +%s).zip"

echo "[INFO] Source URL: ${SRC_URL}"
echo "[INFO] Destination path: ${DEST_DIR}"

# Check if the destination directory already exists and prompt for overwrite
if [[ -d "${DEST_DIR}" ]]; then
    echo -en "\033[1;33m[WARNING] Destination directory already exists, overwrite? [y/N]\033[0m "
    read -r
    if [[ ! "${REPLY}" =~ ^[Yy]$ ]]; then
        echo "[INFO] Exiting"
        exit 0
    fi
fi

# Function to clean up the downloaded archive
cleanup() {
    echo "[INFO] Removing the downloaded archive at ${ARCHIVE_PATH}"
    rm -f "${ARCHIVE_PATH}"
}
trap cleanup EXIT  # Ensure cleanup runs on script exit

# Download the archive
echo "[INFO] Downloading archive to ${ARCHIVE_PATH}"
curl -SL "${SRC_URL}" -o "${ARCHIVE_PATH}"

# Extract the archive
echo "[INFO] Extracting archive to ${DEST_DIR}"
unzip -q "${ARCHIVE_PATH}" -d "${DEST_DIR}"

# Run post_install.sh in the extracted environment
echo "[INFO] Running post install script in extracted environment"
if [[ -f "${DEST_DIR}/post_install.sh" ]]; then
    "${DEST_DIR}/post_install.sh"
else
    echo >&2 -e "\033[1;33m[WARNING] post_install.sh not found in ${DEST_DIR}, skipping post-installation\033[0m"
fi

# Update pip in the extracted environment
echo "[INFO] Updating pip in extracted environment"
if [[ -f "${DEST_DIR}/python.sh" ]]; then
    "${DEST_DIR}/python.sh" -m pip install --upgrade pip
else
    echo >&2 -e "\033[1;33m[WARNING] python.sh not found in ${DEST_DIR}, skipping pip upgrade\033[0m"
fi

# Print environment setup instructions
echo "[INFO] Isaac Sim has been downloaded to ${DEST_DIR}"
echo "[INFO] Recommended environment variable setup:"
echo "[INFO]   echo \"export ISAAC_SIM_PYTHON='${DEST_DIR}/python.sh'\" >> ~/.bashrc # Bash"
echo "[INFO]   set -Ux ISAAC_SIM_PYTHON '${DEST_DIR}/python.sh' # Fish"
