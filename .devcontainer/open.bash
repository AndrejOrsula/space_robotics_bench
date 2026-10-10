#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"

if [[ -n "$1" ]]; then
    WORKSPACE_FOLDER="$1"
else
    WORKSPACE_FOLDER="$(grep -Po '"workspaceFolder":.*?[^\\]",' "${SCRIPT_DIR}/devcontainer.json" | cut -d'"' -f4 || true)"
    if [[ -z "${WORKSPACE_FOLDER}" ]]; then
        WORKSPACE_FOLDER="/"
    fi
fi

CODE_REMOTE_CMD=(
    code --remote
    "dev-container+$(printf "%s" "${REPOSITORY_DIR}" | xxd -p | tr -d "[:space:]")"
    "${WORKSPACE_FOLDER}"
)
echo -e "\033[1;90m${CODE_REMOTE_CMD[*]}\033[0m" | xargs
exec "${CODE_REMOTE_CMD[@]}"
