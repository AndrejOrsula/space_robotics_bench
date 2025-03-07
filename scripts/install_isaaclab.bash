#!/usr/bin/env bash
### Install Isaac Lab
### Usage: install_isaaclab.bash [destination_directory]
set -e

## Config
SRC_REPOSITORY="${SRC_REPOSITORY:-"https://github.com/isaac-sim/IsaacLab.git"}"
SRC_BRANCH="${SRC_BRANCH:-"v2.0.2"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/isaaclab"}"}"

echo "[INFO] Source repository: ${SRC_REPOSITORY}"
echo "[INFO] Source branch/tag: ${SRC_BRANCH}"
echo "[INFO] Destination path: ${DEST_DIR}"

# Verify that ISAAC_SIM_PYTHON is set and points to a valid Python executable
if [[ -z "${ISAAC_SIM_PYTHON}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAAC_SIM_PYTHON is not set. Please set it to the path of the Python executable in the Isaac Sim environment.\033[0m"
    exit 1
fi
if [[ ! -x "${ISAAC_SIM_PYTHON}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] ISAAC_SIM_PYTHON is not a valid Python executable. Please set it to the path of the Python executable in the Isaac Sim environment.\033[0m"
    exit 1
fi
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-"$(dirname "${ISAAC_SIM_PYTHON}")"}"

echo "[INFO] ISAAC_SIM_PATH: ${ISAAC_SIM_PATH}"
echo "[INFO] ISAAC_SIM_PYTHON: ${ISAAC_SIM_PYTHON}"

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

# Clone the repository
echo "[INFO] Cloning repository to ${DEST_DIR}"
git clone --branch "${SRC_BRANCH}" "${SRC_REPOSITORY}" "${DEST_DIR}"

# Create symbolic link to Isaac Sim
ln -sf "${ISAAC_SIM_PATH}" "${DEST_DIR}/_isaac_sim"

# Update pip
echo "[INFO] Updating pip in extracted environment"
"${ISAAC_SIM_PYTHON}" -m pip install --upgrade pip

# Install all extensions in editable mode
# shellcheck disable=SC2044
for extension in $(find -L "${DEST_DIR}/source" -mindepth 1 -maxdepth 1 -type d); do
    if [ -f "${extension}/pyproject.toml" ] || [ -f "${extension}/setup.py" ]; then
        echo "[INFO] Installing ${extension}"
        "${ISAAC_SIM_PYTHON}" -m pip install --editable "${extension}"
    fi
done

# Print final message
echo "[INFO] Isaac Lab extensions have been installed as editable packages from ${DEST_DIR}/source"
