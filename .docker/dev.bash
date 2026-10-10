#!/usr/bin/env bash
### Run the container with development volumes; arguments as for run.bash
### Env: SRB_DOCKER_CACHE_DIR, SRB_CARGO_TARGET_DIR, SRB_DOCKER_PROFILE (default: simulator)
### Env: SRB_DOCKER_DEV_WRAPPER=false runs the command as given (no file-ownership hand-back)
### Secrets: pass `-e NAME`, not `-e NAME=VALUE`; values never reach the docker argv
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"
export WITH_DEV_VOLUME="${WITH_DEV_VOLUME:-true}"
export SRB_DOCKER_PROFILE="${SRB_DOCKER_PROFILE:-simulator}"
SRB_NGC_ENABLED=false
case "${SRB_DOCKER_PROFILE}" in
    ngc|ngc-headless|ngc-webrtc|ngc-gui) SRB_NGC_ENABLED=true ;;
esac
if [[ "${SRB_DOCKER_NGC:-false}" = true ]]; then
    SRB_NGC_ENABLED=true
    export SRB_DOCKER_PROFILE=ngc
fi
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    export SRB_NGC_WORKSPACE="${SRB_NGC_WORKSPACE:-/workspace}"
fi
if [[ "${SRB_DOCKER_PROFILE}" = ngc-headless ]]; then
    export SRB_NGC_MODE=headless
elif [[ "${SRB_DOCKER_PROFILE}" = ngc-webrtc ]]; then
    export SRB_NGC_MODE=webrtc
elif [[ "${SRB_DOCKER_PROFILE}" = ngc-gui ]]; then
    export SRB_NGC_MODE=gui
fi

if [[ "${SRB_DOCKER_PROFILE}" = trusted-simulator ]]; then
    echo >&2 "[SECURITY WARNING] .docker/dev.bash was asked for the trusted-simulator profile. The container receives --privileged, all of /dev, and a writable workspace."
fi

SRB_DOCKER_CACHE_DIR="${SRB_DOCKER_CACHE_DIR:-"${HOME}/.cache/space_robotics_bench/docker"}"
SRB_CARGO_TARGET_DIR="${SRB_CARGO_TARGET_DIR:-"/root/.cache/srb/cargo-target"}"
SRB_CARGO_TARGET_DIR="${SRB_CARGO_TARGET_DIR%/}"
if [[ "${SRB_CARGO_TARGET_DIR}" = "/root/.cargo/target" ]]; then
    echo >&2 -e "\033[1;31m[ERROR] SRB_CARGO_TARGET_DIR must not be /root/.cargo/target because that path contains Docker image build artifacts. Use /root/.cache/srb/cargo-target or another container path.\033[0m"
    exit 2
fi
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    SRB_NGC_CARGO_CACHE_DIR="${SRB_NGC_CARGO_CACHE_DIR:-${SRB_NGC_CACHE_DIR:-${HOME}/.cache/space_robotics_bench/ngc}/cargo}"
    SRB_NGC_CARGO_ROOT="${SRB_NGC_CARGO_ROOT:-/tmp/srb-ngc-cargo}"
    SRB_CARGO_TARGET_DIR="${SRB_NGC_CARGO_TARGET_DIR:-${SRB_NGC_CARGO_ROOT}/target}"
    DOCKER_DEV_CACHE_VOLUMES=(
        "${SRB_NGC_CARGO_CACHE_DIR}/registry:${SRB_NGC_CARGO_ROOT}/registry:rw"
        "${SRB_NGC_CARGO_CACHE_DIR}/git:${SRB_NGC_CARGO_ROOT}/git:rw"
        "${SRB_NGC_CARGO_CACHE_DIR}/target:${SRB_CARGO_TARGET_DIR}:rw"
    )
else
    DOCKER_DEV_CACHE_VOLUMES=(
        "${SRB_DOCKER_CACHE_DIR}/cargo/registry:/root/.cargo/registry:rw"
        "${SRB_DOCKER_CACHE_DIR}/cargo/git:/root/.cargo/git:rw"
        "${SRB_DOCKER_CACHE_DIR}/cargo/target:${SRB_CARGO_TARGET_DIR}:rw"
    )
fi
DOCKER_DEV_VOLUMES=(
    "${DOCKER_DEV_CACHE_VOLUMES[@]}"
)
if command -v git >/dev/null 2>&1 && git -C "${REPOSITORY_DIR}" rev-parse --git-dir >/dev/null 2>&1; then
    GIT_DIR="$(git -C "${REPOSITORY_DIR}" rev-parse --git-dir)"
    GIT_COMMON_DIR="$(git -C "${REPOSITORY_DIR}" rev-parse --git-common-dir)"
    [[ "${GIT_DIR}" = /* ]] || GIT_DIR="${REPOSITORY_DIR}/${GIT_DIR}"
    [[ "${GIT_COMMON_DIR}" = /* ]] || GIT_COMMON_DIR="${REPOSITORY_DIR}/${GIT_COMMON_DIR}"
    GIT_DIR="$(cd "${GIT_DIR}" && pwd -P)"
    GIT_COMMON_DIR="$(cd "${GIT_COMMON_DIR}" && pwd -P)"
    if [[ "${GIT_DIR}" != "${GIT_COMMON_DIR}" ]]; then
        DOCKER_DEV_VOLUMES+=("${GIT_COMMON_DIR}:${GIT_COMMON_DIR}:rw")
    fi
fi
DOCKER_DEV_ENVIRON=(
    LOG_LEVEL="${LOG_LEVEL:-info}"
    DEBUG_VIS="${DEBUG_VIS:-true}"
    HYDRA_FULL_ERROR="${HYDRA_FULL_ERROR:-1}"
    RICH_TRACEBACK="${RICH_TRACEBACK:-true}"
    RICH_TRACEBACK_LOCALS="${RICH_TRACEBACK_LOCALS:-false}"
    LOGFIRE_ENABLE="${LOGFIRE_ENABLE:-true}"
    LOGFIRE_SEND_TO_LOGFIRE="${LOGFIRE_SEND_TO_LOGFIRE:-false}"
    SF_BAKER="${SF_BAKER:-0}"
    CARGO_TARGET_DIR="${SRB_CARGO_TARGET_DIR}"
    GIT_CONFIG_COUNT=1
    GIT_CONFIG_KEY_0=safe.directory
    GIT_CONFIG_VALUE_0="${SRB_NGC_WORKSPACE:-/root/ws}"
)
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    DOCKER_DEV_ENVIRON+=(
        SRB_NGC_WORKSPACE="${SRB_NGC_WORKSPACE:-/workspace}"
        SRB_NGC_CARGO_ROOT="${SRB_NGC_CARGO_ROOT}"
    )
fi

for volume in "${DOCKER_DEV_CACHE_VOLUMES[@]}"; do
    host_dir="${volume%%:*}"
    mkdir -p "${host_dir}"
done

for volume in "${DOCKER_DEV_VOLUMES[@]}"; do
    host_dir="${volume%%:*}"
    target_dir="${volume#*:}"
    target_dir="${target_dir%%:*}"
    if [[ ! -d "${host_dir}" ]]; then
        echo >&2 -e "\033[1;31m[ERROR] The source directory '${host_dir}' to be mounted as a volume at '${target_dir}' does not exist.\033[0m"
        exit 1
    fi
done

DEV_ARGS=()
for vol in "${DOCKER_DEV_VOLUMES[@]}"; do
    DEV_ARGS+=("-v" "${vol}")
done
for env_var in "${DOCKER_DEV_ENVIRON[@]}"; do
    DEV_ARGS+=("-e" "${env_var}")
done
while getopts ":v:e:" opt; do
    case "${opt}" in
        v) DEV_ARGS+=("-v" "${OPTARG}") ;;
        e) DEV_ARGS+=("-e" "${OPTARG}") ;;
        *)
            echo >&2 "Usage: ${0} [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE] [TAG] [CMD]"
            exit 2
            ;;
    esac
done
shift "$((OPTIND - 1))"
SRB_DOCKER_DEV_WRAPPER="${SRB_DOCKER_DEV_WRAPPER:-true}"
if [[ "${SRB_NGC_ENABLED}" = true && "${SRB_DOCKER_DEV_WRAPPER,,}" = true ]]; then
    echo >&2 "[INFO] NGC profile disables the root ownership hand-back wrapper; vendor paths run as UID 1234."
    SRB_DOCKER_DEV_WRAPPER=false
fi
DEV_CMD=("$@")
if [[ "${SRB_DOCKER_DEV_WRAPPER,,}" = true && "${WITH_DEV_VOLUME,,}" = true ]]; then
    # Supervise the child to forward signals and restore mounted-file ownership.
    read -r -d '' DEV_SHELL_WRAPPER <<'WRAPPER' || :
export PYTHONPATH="/root/ws${PYTHONPATH:+:${PYTHONPATH}}"
srb_dev_restore_ownership() {
    if [[ -z "${SRB_HOST_UID:-}" || -z "${SRB_HOST_GID:-}" ]]; then
        return 0
    fi
    local target
    for target in /root/ws ${SRB_HOST_OWNED_PATHS:-}; do
        [[ -e "${target}" ]] || continue
        chown -R --from=0:0 "${SRB_HOST_UID}:${SRB_HOST_GID}" "${target}" 2>/dev/null || :
    done
}
# PID 1 defers traps during a foreground child: run the workload in the background
# and forward SIGTERM so `docker stop` reaches it. SIGKILL skips the hand-back.
srb_dev_child=""
srb_dev_signal=""
srb_dev_forward() {
    srb_dev_signal="$1"
    if [[ -n "${srb_dev_child}" ]]; then
        kill -s "$1" "${srb_dev_child}" 2>/dev/null || :
    fi
}
trap 'srb_dev_restore_ownership' EXIT
trap 'srb_dev_forward TERM' TERM
trap 'if [[ -t 0 ]]; then :; else srb_dev_forward INT; fi' INT
if [[ $# -eq 0 ]]; then
    if [[ -n "${DEFAULT_SHELL:-}" ]] && command -v "${DEFAULT_SHELL}" >/dev/null 2>&1; then
        set -- "${DEFAULT_SHELL}"
    else
        set -- bash
    fi
fi
(
    trap - INT QUIT
    exec "$@"
) 0<&0 &
srb_dev_child=$!
if [[ "${srb_dev_signal}" = TERM ]]; then
    kill -s TERM "${srb_dev_child}" 2>/dev/null || :
fi
while :; do
    srb_dev_status=0
    wait "${srb_dev_child}" || srb_dev_status=$?
    kill -0 "${srb_dev_child}" 2>/dev/null || break
done
srb_dev_restore_ownership
case "${srb_dev_signal}" in
    TERM) exit 143 ;;
    INT) exit 130 ;;
esac
exit "${srb_dev_status}"
WRAPPER
    SRB_HOST_OWNED_PATHS=""
    for vol in "${DOCKER_DEV_CACHE_VOLUMES[@]}"; do
        container_path="${vol#*:}"
        container_path="${container_path%%:*}"
        SRB_HOST_OWNED_PATHS+="${SRB_HOST_OWNED_PATHS:+ }${container_path}"
    done
    DEV_ARGS+=("-e" "SRB_HOST_UID=$(id -u)" "-e" "SRB_HOST_GID=$(id -g)")
    DEV_ARGS+=("-e" "SRB_HOST_OWNED_PATHS=${SRB_HOST_OWNED_PATHS}")
    DEV_CMD=(bash -c "${DEV_SHELL_WRAPPER}" srb-dev-shell "$@")
fi
DOCKER_DEV_CMD=(
    "${SCRIPT_DIR}/run.bash"
    "${DEV_ARGS[@]}"
    "${DEV_CMD[@]}"
)
# Trace counts only; environment values and arguments may contain secrets.
printf "\033[1;90m[TRACE] %q dev_volumes=%d dev_env=%d args=%d\033[0m\n" \
    "${SCRIPT_DIR}/run.bash" "${#DOCKER_DEV_VOLUMES[@]}" \
    "${#DOCKER_DEV_ENVIRON[@]}" "$#"
exec "${DOCKER_DEV_CMD[@]}"
