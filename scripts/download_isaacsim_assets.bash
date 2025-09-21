#!/usr/bin/env bash
### Download Isaac Sim assets
### Usage: download_isaacsim_assets.bash [destination_directory]
set -e

## Config
SRC_URLS=(
    "https://download.isaacsim.omniverse.nvidia.com/isaac-sim-assets-complete-5.0.0.zip.001"
    "https://download.isaacsim.omniverse.nvidia.com/isaac-sim-assets-complete-5.0.0.zip.002"
    "https://download.isaacsim.omniverse.nvidia.com/isaac-sim-assets-complete-5.0.0.zip.003"
)
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/isaac-sim-assets"}"}"
ARCHIVE_PATH="/tmp/isaac-sim-assets-$(date +%s).zip"

for i in "${!SRC_URLS[@]}"; do
    echo "[INFO] Source URL $(($i + 1)): ${SRC_URLS[i]}"
done
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

for i in "${!SRC_URLS[@]}"; do
    # Download the archive
    echo "[INFO] Downloading archive to ${ARCHIVE_PATH}"
    curl -SL "${SRC_URLS[i]}" -o "${ARCHIVE_PATH}"

    # Extract the archive
    echo "[INFO] Extracting archive to ${DEST_DIR}"
    unzip -qo "${ARCHIVE_PATH}" -d "${DEST_DIR}"
done

# Print the final message
echo "[INFO] Isaac Sim assets have been downloaded to ${DEST_DIR}"
