#!/usr/bin/env bash
### Update asset bootstrap for SRB development
### Usage: update.bash [--source git|hf] [--repo-id ID] [--repo-type model|dataset|space] [--revision REF] [--path DIR]
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "$(dirname "${SCRIPT_DIR}")")"
DEFAULT_ASSET_PATH="${REPOSITORY_DIR}/assets/srb_assets"

usage() {
    cat <<'EOF'
Usage:
  update.bash [options]

Options:
  --source git|hf         Select asset update source (default: git)
  --repo-id <id>          Hugging Face repo id for --source hf (required)
  --repo-type model|dataset|space
                         Hugging Face repo type for --source hf (default: dataset)
  --revision <ref>        Optional HF revision/tag/commit for --source hf
  --path <dir>            Override destination directory for assets, which must be
                         inside the repository (default: assets/srb_assets)
  -h, --help             Show this help message

Notes:
  By default this script updates the git submodule at the destination directory.
  Use --source hf when cloning from a Hugging Face mirror instead.
EOF
}

normalize_path() {
    local path="$1"
    if [[ "${path}" == /* ]]; then
        readlink -f "${path}"
    else
        readlink -f "${REPOSITORY_DIR}/${path}"
    fi
}

to_git_submodule_path() {
    local path="$1"
    if [[ "${path}" == "${REPOSITORY_DIR}/"* ]]; then
        printf '%s\n' "${path#"${REPOSITORY_DIR}"/}"
    else
        printf '%s\n' "${path}"
    fi
}

require_path_inside_repository() {
    local source="$1"
    # The repository root itself is rejected: the hf source replaces --path wholesale.
    if [[ "${ASSET_PATH}" != "${REPOSITORY_DIR}/"* ]]; then
        echo "[ERROR] --path must be beneath the repository root for ${source} source"
        echo "        path: ${ASSET_PATH}"
        echo "        repo: ${REPOSITORY_DIR}"
        exit 2
    fi
}

ASSET_SOURCE="${SRB_ASSETS_SOURCE:-git}"
SOURCE_EXPLICIT=0
HF_REPO_ID="${SRB_ASSETS_HF_REPO_ID:-${HF_SRB_ASSETS_REPO_ID:-}}"
HF_REPO_TYPE="${SRB_ASSETS_HF_REPO_TYPE:-${HF_SRB_ASSETS_REPO_TYPE:-dataset}}"
HF_REVISION="${SRB_ASSETS_HF_REPO_REVISION:-${SRB_ASSETS_HF_REVISION:-${HF_SRB_ASSETS_REPO_REVISION:-}}}"
ASSET_PATH="${DEFAULT_ASSET_PATH}"

require_value() {
    local flag="$1"
    local value="$2"
    if [[ -z "${value:-}" || "${value}" == --* ]]; then
        echo "[ERROR] ${flag} requires a value"
        usage
        exit 2
    fi
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --source)
            require_value "$1" "${2:-}"
            ASSET_SOURCE="$2"
            SOURCE_EXPLICIT=1
            shift 2
            ;;
        --repo-id)
            require_value "$1" "${2:-}"
            HF_REPO_ID="$2"
            shift 2
            ;;
        --repo-type)
            require_value "$1" "${2:-}"
            HF_REPO_TYPE="$2"
            shift 2
            ;;
        --revision)
            require_value "$1" "${2:-}"
            HF_REVISION="$2"
            shift 2
            ;;
        --path)
            require_value "$1" "${2:-}"
            ASSET_PATH="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $1"
            usage
            exit 2
            ;;
    esac
done

ASSET_PATH="$(normalize_path "${ASSET_PATH}")"

## Validate args
if [[ "${ASSET_SOURCE}" != "git" && "${ASSET_SOURCE}" != "hf" ]]; then
    echo "[ERROR] --source must be one of: git, hf"
    exit 2
fi

if [[ "${SOURCE_EXPLICIT}" -eq 0 && "${HF_REPO_ID}" != "" && "${ASSET_SOURCE}" == "git" ]]; then
    ASSET_SOURCE="hf"
fi

if [[ "${ASSET_SOURCE}" == "hf" ]]; then
    if [[ -z "${HF_REPO_ID}" ]]; then
        echo "[ERROR] --repo-id is required for --source hf"
        exit 2
    fi
    if [[ "${HF_REPO_TYPE}" != "model" && "${HF_REPO_TYPE}" != "dataset" && "${HF_REPO_TYPE}" != "space" ]]; then
        echo "[ERROR] --repo-type must be model, dataset, or space"
        exit 2
    fi
fi

if [[ "${ASSET_SOURCE}" == "git" ]]; then
    require_path_inside_repository "git"

    ASSET_SUBMODULE_PATH="$(to_git_submodule_path "${ASSET_PATH}")"

    ## Update git submodules
    GIT_SUBMODULE_UPDATE_CMD=(
        git
        -C "${REPOSITORY_DIR}"
        submodule
        update
        --init
        --remote
        --recursive
        "${ASSET_SUBMODULE_PATH}"
    )
    echo -e "\033[1;90m[TRACE] ${GIT_SUBMODULE_UPDATE_CMD[*]}\033[0m" | xargs
    # shellcheck disable=SC2048
    exec "${GIT_SUBMODULE_UPDATE_CMD[@]}"
fi

if [[ "${ASSET_SOURCE}" != "hf" ]]; then
    echo "[ERROR] unsupported asset source: ${ASSET_SOURCE}"
    exit 2
fi

## The download below rmtree's and renames paths derived from the destination,
## so it gets the same containment check as the git branch.
require_path_inside_repository "hf"


## Download from Hugging Face
HF_DOWNLOAD_CMD=(
    python3
    -c
    'import os, shutil; from pathlib import Path; from huggingface_hub import snapshot_download
repo_id = os.environ["HF_ASSET_REPO_ID"]
repo_type = os.environ["HF_ASSET_REPO_TYPE"]
local_dir = Path(os.environ["HF_ASSET_PATH"])
revision = os.environ.get("HF_ASSET_REVISION") or None
tmp_dir = local_dir.with_name(f"{local_dir.name}.tmp.hf")
if tmp_dir.exists():
    shutil.rmtree(tmp_dir)
# Download into a sibling temp dir first so a failed download never destroys an
# existing destination and never leaves a partial temp dir behind.
try:
    snapshot_download(
        repo_id=repo_id,
        repo_type=repo_type,
        local_dir=str(tmp_dir),
        revision=revision,
    )
except BaseException:
    if tmp_dir.exists():
        shutil.rmtree(tmp_dir)
    raise
# Download succeeded: swap the freshly downloaded tree into place. Move the old
# destination aside (atomic rename) instead of deleting it up front, so a failure
# during the swap can restore it -- a plain rmtree-then-rename leaves the
# destination destroyed if rmtree raises partway or the process is interrupted.
bak_dir = local_dir.with_name(f"{local_dir.name}.bak.hf")
if bak_dir.exists():
    shutil.rmtree(bak_dir)
had_old = local_dir.exists()
if had_old:
    local_dir.rename(bak_dir)
try:
    tmp_dir.rename(local_dir)
except BaseException:
    if had_old and not local_dir.exists():
        bak_dir.rename(local_dir)
    raise
if had_old:
    shutil.rmtree(bak_dir)
print(f"[INFO] Downloaded {repo_id} ({repo_type}) into {local_dir}")'
)

export HF_ASSET_REPO_ID="${HF_REPO_ID}"
export HF_ASSET_REPO_TYPE="${HF_REPO_TYPE}"
export HF_ASSET_REVISION="${HF_REVISION}"
export HF_ASSET_PATH="${ASSET_PATH}"

if ! python3 -c "import importlib.util; raise SystemExit(1) if importlib.util.find_spec('huggingface_hub') is None else SystemExit(0)"; then
    echo "[ERROR] huggingface_hub is not installed."
    echo "Install it (for example: python3 -m pip install huggingface_hub)."
    exit 1
fi

echo -e "\033[1;90m[TRACE] huggingface snapshot_download --repo-id ${HF_REPO_ID} --repo-type ${HF_REPO_TYPE} --path ${ASSET_PATH}\033[0m"
"${HF_DOWNLOAD_CMD[@]}"
