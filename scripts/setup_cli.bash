#!/usr/bin/env bash
### Install CLI tools and configure shell completions
### Usage: setup_cli.bash [destination_directory] [shell1 shell2 ...]
set -e

## Config
DEST_DIR="${DEST_DIR:-"$HOME/.local/bin"}"
SPECIFIED_SHELLS=()
EXECUTABLES=(
    "simforge"
    "srb"
    "space_robotics_bench"
)

## Process arguments
for arg in "$@"; do
    # If it looks like a directory path, use it as DEST_DIR
    if [[ "$arg" == /* || "$arg" == ./* || "$arg" == ~/* || "$arg" == ../* ]]; then
        DEST_DIR="$arg"
        # Otherwise treat it as a shell name
    elif [[ "$arg" == "bash" || "$arg" == "fish" || "$arg" == "zsh" ]]; then
        SPECIFIED_SHELLS+=("$arg")
    else
        echo >&2 -e "\033[1;31m[ERROR]\033[0m Unrecognized argument: $arg"
        echo >&2 "Usage: $0 [destination_directory] [bash|fish|zsh]"
        exit 1
    fi
done

## Check if a shell is requested
is_shell_requested() {
    local shell="$1"
    # If no shells were specified, configure all available shells
    [[ ${#SPECIFIED_SHELLS[@]} -eq 0 ]] && return 0
    # Otherwise, check if the shell was specified
    for s in "${SPECIFIED_SHELLS[@]}"; do
        [[ "$s" == "$shell" ]] && return 0
    done
    return 1
}

echo "[INFO] Executables: [$(IFS=", "; echo "${EXECUTABLES[*]}")]"
echo "[INFO] Destination path: ${DEST_DIR}"
if [[ ${#SPECIFIED_SHELLS[@]} -gt 0 ]]; then
    echo "[INFO] Configuring shells: [$(IFS=", "; echo "${SPECIFIED_SHELLS[*]}")]"
else
    echo "[INFO] Configuring all detected shells"
fi

if [[ -z "${ISAAC_SIM_PYTHON}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR]\033[0m ISAAC_SIM_PYTHON is not set. Please set it to the path of the Python executable in the Isaac Sim environment."
    exit 1
fi
if [[ ! -x "${ISAAC_SIM_PYTHON}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR]\033[0m ISAAC_SIM_PYTHON is not a valid Python executable. Please set it to the path of the Python executable in the Isaac Sim environment."
    exit 1
fi

ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-"$(dirname "${ISAAC_SIM_PYTHON}")"}"
ISAAC_SIM_BIN_PATH="${ISAAC_SIM_PATH}/kit/python/bin"

echo "[INFO] ISAAC_SIM_PATH: ${ISAAC_SIM_PATH}"
echo "[INFO] ISAAC_SIM_PYTHON: ${ISAAC_SIM_PYTHON}"
echo -e "\033[1;90m[TRACE]\033[0m ISAAC_SIM_BIN_PATH: ${ISAAC_SIM_BIN_PATH}"

# Ensure destination directory exists
if [[ ! -d "${DEST_DIR}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR]\033[0m Destination directory ${DEST_DIR} does not exist. Please create it first."
    exit 1
fi

# Make sure the destination directory is in PATH
if [[ ! ":$PATH:" == *":${DEST_DIR}:"* ]]; then
    echo >&2 -e "\033[1;31m[ERROR]\033[0m ${DEST_DIR} is not in PATH. Add it to your PATH or specify a different destination directory via \`setup_cli.bash [destination_directory]\`."
    exit 1
fi

echo "[INFO] Installing executables..."

WITH_SUDO=""
if [[ ! -w "${DEST_DIR}" ]]; then
    echo "[INFO] Destination directory requires elevated permissions. Using sudo for file operations."
    WITH_SUDO="sudo"
    sudo -v
fi
for exe in "${EXECUTABLES[@]}"; do
    exe_path="${ISAAC_SIM_BIN_PATH}/${exe}"

    if [[ -f "${exe_path}" ]]; then
        echo -e "\033[1;90m[TRACE]\033[0m Updating shebang of ${exe_path}"
        sed -i "1s|.*|#!${ISAAC_SIM_PYTHON}|" "${exe_path}"

        echo "[INFO] Copying ${exe} to ${DEST_DIR}"
        ${WITH_SUDO} cp -f "${exe_path}" "${DEST_DIR}/"
        ${WITH_SUDO} chmod +x "${DEST_DIR}/${exe}"
    else
        echo >&2 -e "\033[1;33m[WARNING]\033[0m Executable ${exe} not found at ${exe_path}."
    fi
done

echo "[INFO] Configuring argcomplete..."

# Find register-python-argcomplete in Isaac Sim
REGISTER_ARGCOMPLETE=""
if [[ -f "${ISAAC_SIM_BIN_PATH}/register-python-argcomplete" ]]; then
    REGISTER_ARGCOMPLETE="${ISAAC_SIM_BIN_PATH}/register-python-argcomplete"
    echo -e "\033[1;90m[TRACE]\033[0m Found register-python-argcomplete at ${REGISTER_ARGCOMPLETE}"
fi

if [[ -z "${REGISTER_ARGCOMPLETE}" ]]; then
    echo >&2 -e "\033[1;33m[WARNING]\033[0m register-python-argcomplete not found in Isaac Sim, looking in system PATH"

    if command -v register-python-argcomplete &>/dev/null; then
        REGISTER_ARGCOMPLETE="$(command -v register-python-argcomplete)"
        echo -e "\033[1;90m[TRACE]\033[0m Using system register-python-argcomplete at ${REGISTER_ARGCOMPLETE}"
    fi
fi

if [[ -z "${REGISTER_ARGCOMPLETE}" ]]; then
    echo >&2 -e "\033[1;33m[WARNING]\033[0m register-python-argcomplete not found, skipping argcomplete setup."
else
    # Setup Fish completions
    if command -v fish &>/dev/null && is_shell_requested "fish"; then
        FISH_COMPLETION_DIR="${HOME}/.config/fish/completions"
        mkdir -p "${FISH_COMPLETION_DIR}"
        echo "[INFO] Setting up Fish completions in ${FISH_COMPLETION_DIR}"

        for exe in "${EXECUTABLES[@]}"; do
            "${REGISTER_ARGCOMPLETE}" --shell fish "${exe}" > "${FISH_COMPLETION_DIR}/${exe}.fish"
            echo -e "\033[1;90m[TRACE]\033[0m Created Fish completion for ${exe}"
        done
        echo "[INFO] Fish completions installed"
    fi

    # Setup Bash completions
    if command -v bash &>/dev/null && is_shell_requested "bash"; then
        BASH_COMPLETION_DIR="${HOME}/.bash_completion.d"
        mkdir -p "${BASH_COMPLETION_DIR}"
        echo "[INFO] Setting up Bash completions in ${BASH_COMPLETION_DIR}"

        for exe in "${EXECUTABLES[@]}"; do
            "${REGISTER_ARGCOMPLETE}" "${exe}" > "${BASH_COMPLETION_DIR}/${exe}"
            echo -e "\033[1;90m[TRACE]\033[0m Created Bash completion for ${exe}"
        done

        # Add source command to bashrc if not already there
        if ! grep -q "bash_completion.d" "${HOME}/.bashrc" 2>/dev/null; then
            echo "[INFO] Adding Bash completion source to ~/.bashrc"
            cat >> "${HOME}/.bashrc" << 'EOF'

# Source bash completions
if [ -d "$HOME/.bash_completion.d" ]; then
  for f in "$HOME/.bash_completion.d"/*; do
    [ -f "$f" ] && source "$f"
  done
fi
EOF
        else
            echo -e "\033[1;90m[TRACE]\033[0m Bash completion source already in ~/.bashrc"
        fi
        echo "[INFO] Bash completions installed"
    fi

    # Setup Zsh completions
    if command -v zsh &>/dev/null && is_shell_requested "zsh"; then
        ZSH_COMPLETION_DIR="${HOME}/.zsh/completion"
        mkdir -p "${ZSH_COMPLETION_DIR}"
        echo "[INFO] Setting up Zsh completions in ${ZSH_COMPLETION_DIR}"

        for exe in "${EXECUTABLES[@]}"; do
            "${REGISTER_ARGCOMPLETE}" "${exe}" > "${ZSH_COMPLETION_DIR}/_${exe}"
            echo -e "\033[1;90m[TRACE]\033[0m Created Zsh completion for ${exe}"
        done

        # Add fpath and compinit to zshrc if not already there
        if ! grep -q "fpath=.*${ZSH_COMPLETION_DIR}" "${HOME}/.zshrc" 2>/dev/null; then
            echo "[INFO] Adding Zsh completion configuration to ~/.zshrc"
            cat >> "${HOME}/.zshrc" << EOF

# Add custom completions directory
fpath=(${ZSH_COMPLETION_DIR} \$fpath)
autoload -U compinit && compinit
EOF
        else
            echo -e "\033[1;90m[TRACE]\033[0m Zsh completion configuration already in ~/.zshrc"
        fi
        echo "[INFO] Zsh completions installed"
    fi
fi

echo "[INFO] Installation completed successfully"
