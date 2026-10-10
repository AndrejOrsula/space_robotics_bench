#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
exec bash "${SCRIPT_DIR}/../../srb/cli/setup_cli.bash" "$@"
