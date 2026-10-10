#!/usr/bin/env bash
### Build the Docker image
### Usage: build.bash [TAG] [BUILD_ARGS...]
### Env: SRB_BUILD_NETWORK=auto|default|host|none to select the build network
### Strict source identity: SRB_SOURCE_PROVENANCE_REQUIRED=1 plus the source
### manifest, snapshot UUID, tree digest, and explicit helper path.
set -Eeu

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
SOURCE_REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"
BUILD_CONTEXT_DIR="${SOURCE_REPOSITORY_DIR}"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

STRICT_CONTEXT_PARENT=""
STRICT_CONTEXT_DIR=""
STRICT_PROVENANCE=0
PROVENANCE_HELPER=""
PROVENANCE_MANIFEST=""
SOURCE_SNAPSHOT_ID=""
SOURCE_TREE_DIGEST=""
IID_FILE=""
IID_FILE_OWNED=0
IID_OUTPUT_FILE=""
IID_OUTPUT_STAGE=""

cleanup_build_state() {
    if [[ -n "${STRICT_CONTEXT_PARENT}" && -d "${STRICT_CONTEXT_PARENT}" ]]; then
        find -P "${STRICT_CONTEXT_PARENT}" -type d -exec chmod u+rwx -- {} +
        rm -rf -- "${STRICT_CONTEXT_PARENT}"
    fi
    if [[ "${IID_FILE_OWNED}" -eq 1 && -n "${IID_FILE}" ]]; then
        rm -f -- "${IID_FILE}"
    fi
    if [[ -n "${IID_OUTPUT_STAGE}" && -d "${IID_OUTPUT_STAGE}" ]]; then
        rm -rf -- "${IID_OUTPUT_STAGE}"
    fi
}
trap cleanup_build_state EXIT

USER="${USER:-$(id -un)}"

WITH_SUDO=()
if [[ " $(id -nG "${USER}" 2>/dev/null) " != *" docker "* ]]; then
    echo "[INFO] The current user '${USER}' is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO=("sudo")
fi

PROJECT_NAME="$(basename "${SOURCE_REPOSITORY_DIR}")"
IMAGE_TAG=""

if [ "${#}" -gt "0" ]; then
    if [[ "${1}" != "-"* ]]; then
        IMAGE_TAG="${1}"
        shift
    fi
fi
BUILD_ARGS=("$@")

SOURCE_LABEL_VERSION="org.space-robotics-bench.source.provenance-version"
SOURCE_LABEL_SNAPSHOT="org.space-robotics-bench.source.snapshot-id"
SOURCE_LABEL_TREE="org.space-robotics-bench.source.tree-digest"

parse_iidfile_argument() {
    local index argument next
    for ((index = 0; index < ${#BUILD_ARGS[@]}; index++)); do
        argument="${BUILD_ARGS[index]}"
        case "${argument}" in
            --iidfile)
                ((index + 1 < ${#BUILD_ARGS[@]})) || fail '--iidfile needs a path'
                next="${BUILD_ARGS[index + 1]}"
                [[ -n "${next}" ]] || fail '--iidfile needs a non-empty path'
                [[ -z "${IID_FILE}" ]] || fail 'multiple --iidfile arguments are not supported'
                IID_FILE="${next}"
                index=$((index + 1))
                ;;
            --iidfile=*)
                next="${argument#--iidfile=}"
                [[ -n "${next}" ]] || fail '--iidfile needs a non-empty path'
                [[ -z "${IID_FILE}" ]] || fail 'multiple --iidfile arguments are not supported'
                IID_FILE="${next}"
                ;;
        esac
    done
}

prepare_strict_iid_output() {
    local index argument
    local -a retained_args=()
    IID_OUTPUT_FILE="${IID_FILE}"
    if [[ -n "${IID_OUTPUT_FILE}" ]]; then
        [[ ! -e "${IID_OUTPUT_FILE}" && ! -L "${IID_OUTPUT_FILE}" ]] ||
        fail "strict IID output must not already exist: ${IID_OUTPUT_FILE}"
        [[ -d "$(dirname -- "${IID_OUTPUT_FILE}")" ]] ||
        fail "strict IID output parent directory is missing: ${IID_OUTPUT_FILE}"
    fi
    for ((index = 0; index < ${#BUILD_ARGS[@]}; index++)); do
        argument="${BUILD_ARGS[index]}"
        case "${argument}" in
            --iidfile) index=$((index + 1)) ;;
            --iidfile=*) ;;
            *) retained_args+=("${argument}") ;;
        esac
    done
    BUILD_ARGS=("${retained_args[@]}")
    IID_FILE=""
}

reject_source_label_override() {
    local index argument next key
    for ((index = 0; index < ${#BUILD_ARGS[@]}; index++)); do
        argument="${BUILD_ARGS[index]}"
        case "${argument}" in
            --label)
                ((index + 1 < ${#BUILD_ARGS[@]})) || fail '--label needs a key=value argument'
                next="${BUILD_ARGS[index + 1]}"
                index=$((index + 1))
                ;;
            --label=*)
                next="${argument#--label=}"
                ;;
            *)
                continue
                ;;
        esac
        key="${next%%=*}"
        case "${key}" in
            "${SOURCE_LABEL_VERSION}"|"${SOURCE_LABEL_SNAPSHOT}"|"${SOURCE_LABEL_TREE}")
                fail "strict source label is caller-controlled: ${key}"
                ;;
        esac
    done
}

reject_strict_context_override() {
    local index argument
    for ((index = 0; index < ${#BUILD_ARGS[@]}; index++)); do
        argument="${BUILD_ARGS[index]}"
        case "${argument}" in
            --file)
                ((index + 1 < ${#BUILD_ARGS[@]})) ||
                fail '--file needs a path'
                fail 'strict source provenance does not permit --file'
                ;;
            --file=*)
                fail 'strict source provenance does not permit --file'
                ;;
            -f)
                ((index + 1 < ${#BUILD_ARGS[@]})) ||
                fail '-f needs a path'
                fail 'strict source provenance does not permit -f'
                ;;
            -f?*)
                fail 'strict source provenance does not permit -f'
                ;;
            --build-context)
                ((index + 1 < ${#BUILD_ARGS[@]})) ||
                fail '--build-context needs a name=path value'
                fail 'strict source provenance does not permit --build-context'
                ;;
            --build-context=*)
                fail 'strict source provenance does not permit --build-context'
                ;;
        esac
    done
}

verify_provenance() {
    local root="$1"
    local result
    if ! result="$(bash "${PROVENANCE_HELPER}" verify --root "${root}" --manifest "${PROVENANCE_MANIFEST}" --expected-snapshot-id "${SOURCE_SNAPSHOT_ID}" --expected-tree-digest "${SOURCE_TREE_DIGEST}" --json 2>&1)"; then
        printf '[ERROR] source provenance verification failed for %s\n%s\n' "${root}" "${result}" >&2
        return 1
    fi
}

compare_context_manifest() {
    local context_manifest="${BUILD_CONTEXT_DIR}/.source-provenance.json"
    if ! cmp -s -- "${PROVENANCE_MANIFEST}" "${context_manifest}"; then
        fail 'strict source context manifest bytes do not match transferred manifest'
    fi
}

require_populated_directory() {
    local directory="$1"
    local label="$2"
    if [[ ! -d "${directory}" || -L "${directory}" ]]; then
        fail "strict source provenance requires ${label} directory: ${directory}"
    fi
    if [[ -z "$(find "${directory}" -mindepth 1 -maxdepth 1 ! -name '.git' -print -quit)" ]]; then
        fail "strict source provenance requires non-empty ${label}: ${directory}"
    fi
}

prepare_strict_context() {
    local exclude
    local -a snapshot_args
    require_populated_directory "${SOURCE_REPOSITORY_DIR}/assets/srb_assets" 'assets/srb_assets'
    require_populated_directory "${SOURCE_REPOSITORY_DIR}/docs/src/_graphics" 'docs/src/_graphics'

    [[ -f "${PROVENANCE_MANIFEST}" ]] || fail "source provenance manifest is missing: ${PROVENANCE_MANIFEST}"
    verify_provenance "${SOURCE_REPOSITORY_DIR}" || exit 1

    command -v python3 >/dev/null 2>&1 || fail 'strict source provenance requires python3'
    STRICT_CONTEXT_PARENT="$(mktemp -d "${TMPDIR:-/tmp}/srb-build-context.XXXXXX")"
    STRICT_CONTEXT_DIR="${STRICT_CONTEXT_PARENT}/source"
    snapshot_args=(snapshot --source "${SOURCE_REPOSITORY_DIR}" --output "${STRICT_CONTEXT_DIR}")
    while IFS= read -r -d '' exclude; do
        snapshot_args+=(--exclude "${exclude}")
        done < <(
        python3 - "${PROVENANCE_MANIFEST}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    manifest = json.load(stream)
for exclusion in manifest["exclusions"]:
    sys.stdout.write(exclusion)
    sys.stdout.write("\0")
PY
    )
    bash "${PROVENANCE_HELPER}" "${snapshot_args[@]}" >/dev/null
    rm -f -- "${STRICT_CONTEXT_DIR}/.source-provenance.json"
    cp -- "${PROVENANCE_MANIFEST}" "${STRICT_CONTEXT_DIR}/.source-provenance.json"
    verify_provenance "${STRICT_CONTEXT_DIR}" || exit 1
    BUILD_CONTEXT_DIR="${STRICT_CONTEXT_DIR}"
    compare_context_manifest
    require_populated_directory "${STRICT_CONTEXT_DIR}/assets/srb_assets" 'context assets/srb_assets'
    require_populated_directory "${STRICT_CONTEXT_DIR}/docs/src/_graphics" 'context docs/src/_graphics'
}

parse_iidfile_argument

case "${SRB_SOURCE_PROVENANCE_REQUIRED:-0}" in
    0|"") ;;
    1)
        STRICT_PROVENANCE=1
        SOURCE_SNAPSHOT_ID="${SRB_SOURCE_SNAPSHOT_ID:-}"
        SOURCE_TREE_DIGEST="${SRB_SOURCE_TREE_DIGEST:-}"
        PROVENANCE_HELPER="${SRB_SOURCE_PROVENANCE_HELPER:-}"
        PROVENANCE_MANIFEST="${SRB_SOURCE_PROVENANCE_MANIFEST:-${SOURCE_REPOSITORY_DIR}/.source-provenance.json}"
        [[ "${SOURCE_SNAPSHOT_ID}" =~ ^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$ ]] ||
        fail 'SRB_SOURCE_SNAPSHOT_ID must be a lowercase canonical UUID'
        [[ "${SOURCE_TREE_DIGEST}" =~ ^[0-9a-f]{64}$ ]] ||
        fail 'SRB_SOURCE_TREE_DIGEST must be a lowercase SHA-256 digest'
        [[ "${PROVENANCE_HELPER}" = /* && -f "${PROVENANCE_HELPER}" ]] ||
        fail 'SRB_SOURCE_PROVENANCE_HELPER must name an explicit helper file'
        [[ "${PROVENANCE_MANIFEST}" = /* && -f "${PROVENANCE_MANIFEST}" ]] ||
        fail 'SRB_SOURCE_PROVENANCE_MANIFEST must name an absolute manifest file'
        reject_strict_context_override
        reject_source_label_override
        prepare_strict_iid_output
        prepare_strict_context
        IID_FILE="${STRICT_CONTEXT_PARENT}/image.iid"
        IID_FILE_OWNED=1
        ;;
    *)
        fail 'SRB_SOURCE_PROVENANCE_REQUIRED must be 0 or 1'
        ;;
esac

DOCKERHUB_USER="$("${WITH_SUDO[@]}" docker info 2>/dev/null | sed '/Username:/!d;s/.* //')"
IMAGE_NAME="${SRB_IMAGE_NAME:-${DOCKERHUB_USER:+${DOCKERHUB_USER}/}${PROJECT_NAME,,}}"
if [[ -n "${IMAGE_TAG}" ]]; then
    IMAGE_NAME+=":${IMAGE_TAG}"
fi

ASSETS_DIR="${BUILD_CONTEXT_DIR}/assets/srb_assets"
GRAPHICS_DIR="${BUILD_CONTEXT_DIR}/docs/src/_graphics"

bootstrap_repo() {
    local dir="$1"
    local label="$2"
    local source_env_var="$3"
    local repo_id_env_var="$4"
    local repo_type_env_var="$5"
    local repo_revision_env_var="$6"
    local update_args=()
    local source="${!source_env_var:-}"
    local hf_repo_id="${!repo_id_env_var:-}"
    local hf_repo_type="${!repo_type_env_var:-}"
    local hf_repo_revision="${!repo_revision_env_var:-}"
    if [[ -n "${hf_repo_id}" && -z "${hf_repo_type}" ]]; then
        hf_repo_type="dataset"
    fi

    if [ -d "${dir}" ] && [ -n "$(find "${dir}" -mindepth 1 -maxdepth 1 ! -name '.git' -print -quit)" ]; then
        return
    fi

    echo "[INFO] Initializing ${label}"
    if [[ -n "${source}" ]]; then
        if [[ "${source}" != "hf" && "${source}" != "git" ]]; then
            echo >&2 -e "\033[1;31m[ERROR] ${source_env_var} must be 'git' or 'hf'\033[0m"
            exit 1
        fi
        update_args+=(--source "${source}")
    elif [[ -n "${hf_repo_id}" ]]; then
        update_args+=(--source "hf")
    elif [[ -n "${hf_repo_type:-}" || -n "${hf_repo_revision:-}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] ${label} HF type/revision is set but no repo id is configured\033[0m"
        echo >&2 "Set ${repo_id_env_var} when using HF source."
        exit 1
    fi

    if [[ -n "${hf_repo_id}" ]]; then
        update_args+=(--repo-id "${hf_repo_id}")
    fi
    if [[ -n "${hf_repo_type:-}" ]]; then
        update_args+=(--repo-type "${hf_repo_type}")
    fi
    if [[ -n "${hf_repo_revision:-}" ]]; then
        update_args+=(--revision "${hf_repo_revision}")
    fi
    if [[ "${#update_args[@]}" -eq 0 ]]; then
        update_args+=(--source git)
    fi

    update_args+=(--path "${dir}")

    echo "[INFO] Initializing ${label} with: ${update_args[*]}"
    if ! "${BUILD_CONTEXT_DIR}/assets/scripts/update.bash" "${update_args[@]}"; then
        echo >&2 -e "\033[1;31m[ERROR] Failed to initialize ${label}.\033[0m"
        exit 1
    fi
}

SRB_ASSETS_HF_REPO_ID="${SRB_ASSETS_HF_REPO_ID:-${HF_SRB_ASSETS_REPO_ID:-}}"
SRB_ASSETS_HF_REPO_TYPE="${SRB_ASSETS_HF_REPO_TYPE:-${HF_SRB_ASSETS_REPO_TYPE:-}}"
SRB_ASSETS_HF_REPO_REVISION="${SRB_ASSETS_HF_REPO_REVISION:-${HF_SRB_ASSETS_REPO_REVISION:-}}"
if [[ "${STRICT_PROVENANCE}" -eq 0 ]]; then
    bootstrap_repo \
        "${ASSETS_DIR}" \
        "assets/srb_assets" \
        "SRB_ASSETS_SOURCE" \
        "SRB_ASSETS_HF_REPO_ID" \
        "SRB_ASSETS_HF_REPO_TYPE" \
        "SRB_ASSETS_HF_REPO_REVISION"

    SRB_GRAPHICS_HF_REPO_ID="${SRB_GRAPHICS_HF_REPO_ID:-${HF_SRB_GRAPHICS_REPO_ID:-}}"
    SRB_GRAPHICS_HF_REPO_TYPE="${SRB_GRAPHICS_HF_REPO_TYPE:-${HF_SRB_GRAPHICS_REPO_TYPE:-}}"
    SRB_GRAPHICS_HF_REPO_REVISION="${SRB_GRAPHICS_HF_REPO_REVISION:-${HF_SRB_GRAPHICS_REPO_REVISION:-}}"
    bootstrap_repo \
        "${GRAPHICS_DIR}" \
        "docs/src/_graphics" \
        "SRB_GRAPHICS_SOURCE" \
        "SRB_GRAPHICS_HF_REPO_ID" \
        "SRB_GRAPHICS_HF_REPO_TYPE" \
        "SRB_GRAPHICS_HF_REPO_REVISION"
fi

DNS_PROBE_HOST="pypi.org"

probe_build_dns() {
    local base_name base_tag
    base_name="$(sed -n 's/^ARG BASE_IMAGE_NAME="\(.*\)"$/\1/p' "${BUILD_CONTEXT_DIR}/Dockerfile" | head -n 1)"
    base_tag="$(sed -n 's/^ARG BASE_IMAGE_TAG="\(.*\)"$/\1/p' "${BUILD_CONTEXT_DIR}/Dockerfile" | head -n 1)"
    if [[ -z "${base_name}" || -z "${base_tag}" ]]; then
        return 0
    fi
    timeout 120 "${WITH_SUDO[@]}" docker run --rm --network bridge \
        --dns-option timeout:2 --dns-option attempts:1 \
        "${base_name}:${base_tag}" \
        getent hosts "${DNS_PROBE_HOST}" &>/dev/null
}

SRB_BUILD_NETWORK="${SRB_BUILD_NETWORK:-auto}"
NETWORK_ARGS=()
if [[ "${#BUILD_ARGS[@]}" -gt 0 ]] && printf '%s\n' "${BUILD_ARGS[@]}" | grep -qE '^--network(=|$)'; then
    echo "[INFO] Build network selected by an explicit --network build argument"
elif [[ "${SRB_BUILD_NETWORK}" != "auto" ]]; then
    NETWORK_ARGS=(--network "${SRB_BUILD_NETWORK}")
else
    DNS_PROBE_STATUS=0
    probe_build_dns || DNS_PROBE_STATUS=$?
    case "${DNS_PROBE_STATUS}" in
        0) ;;
        2)
            echo >&2 -e "\033[1;33m[WARN] The default Docker bridge network cannot resolve '${DNS_PROBE_HOST}'.\033[0m"
            echo >&2 "Building with --network host so the build reaches the host's resolvers. This exposes"
            echo >&2 "the host network to every RUN step; set SRB_BUILD_NETWORK=default to opt out, or"
            echo >&2 "configure the daemon's DNS (\"dns\" in /etc/docker/daemon.json) to fix the bridge."
            NETWORK_ARGS=(--network host)
            ;;
        *)
            fail "Docker DNS probe did not complete a supported lookup (exit ${DNS_PROBE_STATUS}); refusing to build while probe completion is uncertain"
            ;;
    esac
fi

if [[ "${STRICT_PROVENANCE}" -eq 1 ]]; then
    if [[ "${IID_FILE_OWNED}" -eq 1 ]]; then
        BUILD_ARGS+=(--iidfile "${IID_FILE}")
    fi
    BUILD_ARGS+=(
        --label "${SOURCE_LABEL_VERSION}=1"
        --label "${SOURCE_LABEL_SNAPSHOT}=${SOURCE_SNAPSHOT_ID}"
        --label "${SOURCE_LABEL_TREE}=${SOURCE_TREE_DIGEST}"
    )
fi
DOCKER_BUILD_CMD=(
    "${WITH_SUDO[@]}"
    docker build
    --file "${BUILD_CONTEXT_DIR}/Dockerfile"
    --tag "${IMAGE_NAME}"
    "${NETWORK_ARGS[@]}"
    "${BUILD_ARGS[@]}"
    "${BUILD_CONTEXT_DIR}"
)
printf "\033[1;90m[TRACE] "
printf "%q " "${DOCKER_BUILD_CMD[@]}"
printf "\033[0m\n"
"${DOCKER_BUILD_CMD[@]}"

if [[ "${STRICT_PROVENANCE}" -eq 1 ]]; then
    IID="$(<"${IID_FILE}")"
    [[ "${IID}" =~ ^sha256:[0-9a-f]{64}$ ]] ||
    fail "Docker wrote malformed image IID: ${IID_FILE}"
    INSPECT_RECORD="$(
        "${WITH_SUDO[@]}" docker image inspect --format \
            '{{.Id}}{{"\t"}}{{index .Config.Labels "org.space-robotics-bench.source.provenance-version"}}{{"\t"}}{{index .Config.Labels "org.space-robotics-bench.source.snapshot-id"}}{{"\t"}}{{index .Config.Labels "org.space-robotics-bench.source.tree-digest"}}' \
            "${IID}"
    )" || fail "Docker could not inspect fresh IID: ${IID}"
    EXPECTED_RECORD="${IID}"$'\t'"1"$'\t'"${SOURCE_SNAPSHOT_ID}"$'\t'"${SOURCE_TREE_DIGEST}"
    [[ "${INSPECT_RECORD}" == "${EXPECTED_RECORD}" ]] ||
    fail "fresh IID labels do not prove requested source identity"
    verify_provenance "${SOURCE_REPOSITORY_DIR}" || exit 1
    verify_provenance "${BUILD_CONTEXT_DIR}" || exit 1
    compare_context_manifest
    if [[ -n "${IID_OUTPUT_FILE}" ]]; then
        IID_OUTPUT_STAGE="$(mktemp -d -- "$(dirname -- "${IID_OUTPUT_FILE}")/.srb-build-iid.XXXXXX")"
        printf '%s\n' "${IID}" >"${IID_OUTPUT_STAGE}/image.iid"
        ln --no-target-directory -- "${IID_OUTPUT_STAGE}/image.iid" "${IID_OUTPUT_FILE}" ||
        fail "cannot publish strict IID without replacement: ${IID_OUTPUT_FILE}"
    fi
    printf 'result: schema=1 tag=%s iid=%s source=snapshot source-provenance=verified source-snapshot-id=%s source-tree-digest=%s id=%s\n' \
        "${IMAGE_NAME}" "${IID}" "${SOURCE_SNAPSHOT_ID}" "${SOURCE_TREE_DIGEST}" "${IID}"
fi
