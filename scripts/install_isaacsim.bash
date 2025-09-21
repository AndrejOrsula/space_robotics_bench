#!/usr/bin/env bash
### Install Isaac Sim
### Usage: install_isaacsim.bash [destination_directory]
set -e

## Config
SRC_URL="${SRC_URL:-"https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.0.0-linux-x86_64.zip"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/isaac-sim"}"}"
ARCHIVE_PATH="/tmp/isaac-sim-$(date +%s).zip"
OMNI_HUB_ARCHIVE="/tmp/omni_hub-$(date +%s).zip"
OMNI_HUB_EXTRACT="/tmp/omni_hub-$(date +%s)"

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
    rm -rf "${DEST_DIR}"
fi

# Function to clean up the downloaded archives
cleanup() {
    echo "[INFO] Cleaning up temporary files"
    rm -f "${ARCHIVE_PATH}"
    rm -f "${OMNI_HUB_ARCHIVE}"
    rm -rf "${OMNI_HUB_EXTRACT}"
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

# Install Omniverse Hub
echo "[INFO] Downloading Omniverse Hub"
if curl --proto "=https" --tlsv1.2 -sSfL "https://api.ngc.nvidia.com/v2/resources/nvidia/omniverse/hub_workstation_cache/versions/1.0.0/files/omni_hub.linux-x86_64@1.0.0+8e3e0971.zip" -o "${OMNI_HUB_ARCHIVE}"; then
    echo "[INFO] Extracting Omniverse Hub"
    mkdir -p "${OMNI_HUB_EXTRACT}"
    if unzip -q "${OMNI_HUB_ARCHIVE}" -d "${OMNI_HUB_EXTRACT}"; then
        echo "[INFO] Running Omniverse Hub installation script"
        if [[ -f "${OMNI_HUB_EXTRACT}/scripts/install.sh" ]]; then
            "${OMNI_HUB_EXTRACT}/scripts/install.sh"
            echo "[INFO] Omniverse Hub installation completed"
        else
            echo >&2 -e "\033[1;31m[ERROR] Omniverse Hub installation script not found\033[0m"
        fi
    else
        echo >&2 -e "\033[1;31m[ERROR] Failed to extract Omniverse Hub archive\033[0m"
    fi
else
    echo >&2 -e "\033[1;31m[ERROR] Failed to download Omniverse Hub\033[0m"
fi

# Print environment setup instructions
echo "[INFO] Isaac Sim has been installed to ${DEST_DIR}"
echo "[INFO] Recommended environment variable setup:"
echo "[INFO]   echo \"export ISAAC_SIM_PYTHON='${DEST_DIR}/python.sh'\" >> ~/.bashrc # Bash"
echo "[INFO]   echo \"export ISAAC_SIM_PYTHON='${DEST_DIR}/python.sh'\" >> ~/.zshrc # Zsh"
echo "[INFO]   set -Ux ISAAC_SIM_PYTHON '${DEST_DIR}/python.sh' # Fish"
