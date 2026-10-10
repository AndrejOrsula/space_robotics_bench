#!/usr/bin/env bash
### Run a Docker container
### Usage: run.bash [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE] [TAG] [CMD]
### Env: SRB_DOCKER_PROFILE=safe|simulator|trusted-simulator|ngc
### Env: SRB_NGC_MODE=headless|webrtc|gui (ngc profile; default: headless)
### Env: SRB_NGC_CACHE_DIR=/host/cache/root (ngc profile)
### Env: SRB_NGC_READINESS_CHECK=true to poll the vendor readiness log
### Env: SRB_REQUIRE_GPU/VULKAN/RTX/DISPLAY/XVFB=true for fail-closed preflights
set -e

# Snapshot before assigning anything: `-e NAME` forwards the caller's value.
declare -A CALLER_EXPORTED_ENV=()
while IFS= read -r caller_exported_name; do
    CALLER_EXPORTED_ENV["${caller_exported_name}"]="${!caller_exported_name}"
done < <(compgen -e)
unset caller_exported_name

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"

USER="${USER:-$(id -un)}"

WITH_SUDO=()
if [[ " $(id -nG "${USER}" 2>/dev/null) " != *" docker "* ]]; then
    echo "[INFO] The current user '${USER}' is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO=("sudo")
fi

SRB_DOCKER_PROFILE="${SRB_DOCKER_PROFILE:-safe}"
NGC_AMD64_DIGEST="sha256:0c16dd67d09a70ea474f2c809a13c4d09bd23c738184cf1bf28af487ecd39080"
NGC_ARM64_DIGEST="sha256:dcf1a6e81c56f644ab7ac69c4251291c51ecb234aa0792d12d0d9c7e4e4e7cf0"
NGC_IMAGE_REPOSITORY="nvcr.io/nvidia/isaac-sim"
SRB_NGC_MODE="${SRB_NGC_MODE:-headless}"
SRB_NGC_ENABLED=false
case "${SRB_DOCKER_PROFILE}" in
    ngc-headless)
        SRB_NGC_MODE=headless
        SRB_NGC_ENABLED=true
        SRB_DOCKER_PROFILE=ngc
        ;;
    ngc-webrtc)
        SRB_NGC_MODE=webrtc
        SRB_NGC_ENABLED=true
        SRB_DOCKER_PROFILE=ngc
        ;;
    ngc-gui)
        SRB_NGC_MODE=gui
        SRB_NGC_ENABLED=true
        SRB_DOCKER_PROFILE=ngc
        ;;
    ngc)
        SRB_NGC_ENABLED=true
        ;;
esac
if [[ "${SRB_DOCKER_NGC:-false}" == true ]]; then
    SRB_NGC_ENABLED=true
    if [[ "${SRB_DOCKER_PROFILE}" != ngc ]]; then
        SRB_DOCKER_PROFILE=ngc
    fi
fi
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    case "${SRB_NGC_MODE,,}" in
        headless|webrtc|gui) SRB_NGC_MODE="${SRB_NGC_MODE,,}" ;;
        *)
            echo >&2 "[ERROR] Unsupported SRB_NGC_MODE '${SRB_NGC_MODE}'. Expected headless, webrtc or gui."
            exit 2
            ;;
    esac
fi
WITH_HOST_NETWORK="${WITH_HOST_NETWORK:-true}"
WITH_HOST_IPC="${WITH_HOST_IPC:-true}"
SRB_DOCKER_SHM_SIZE="${SRB_DOCKER_SHM_SIZE:-8g}"
case "${SRB_DOCKER_PROFILE}" in
    safe)
        DOCKER_RUN_OPTS=(
            --interactive
            --rm
            --network none
            --ipc private
            --cap-drop ALL
            --security-opt no-new-privileges
        )
        _PROFILE_WITH_GPU=false
        _PROFILE_WITH_GUI=false
        _PROFILE_WITH_HISTORY=false
        _PROFILE_WITH_DEV_VOLUME=false
        ;;
    simulator)
        DOCKER_RUN_OPTS=(
            --interactive
            --rm
            --security-opt no-new-privileges
        )
        _PROFILE_WITH_GPU=true
        _PROFILE_WITH_GUI=true
        _PROFILE_WITH_HISTORY=true
        _PROFILE_WITH_DEV_VOLUME=true
        ;;
    trusted-simulator)
        echo >&2 "[SECURITY WARNING] The trusted-simulator profile grants the container --privileged, all of /dev, host networking, host IPC, caches, GUI, and optional writable source access. Prefer SRB_DOCKER_PROFILE=simulator unless hardware passthrough is required. Use only a locally trusted image or an authenticated digest."
        DOCKER_RUN_OPTS=(
            --interactive
            --rm
            --privileged
        )
        _PROFILE_WITH_GPU=true
        _PROFILE_WITH_GUI=true
        _PROFILE_WITH_HISTORY=true
        _PROFILE_WITH_DEV_VOLUME=true
        ;;
    ngc)
        DOCKER_RUN_OPTS=(
            --interactive
            --rm
            --security-opt no-new-privileges
            --user 1234:1234
        )
        _PROFILE_WITH_GPU=true
        if [[ "${SRB_NGC_MODE}" = gui ]]; then
            _PROFILE_WITH_GUI=true
        else
            _PROFILE_WITH_GUI=false
        fi
        _PROFILE_WITH_HISTORY=false
        _PROFILE_WITH_DEV_VOLUME=true
        ;;
    *)
        echo >&2 "[ERROR] Unsupported SRB_DOCKER_PROFILE '${SRB_DOCKER_PROFILE}'. Expected 'safe', 'simulator', 'trusted-simulator' or 'ngc'."
        exit 2
        ;;
esac
if [[ "${SRB_DOCKER_PROFILE}" != safe ]]; then
    if [[ "${WITH_HOST_NETWORK,,}" = true ]]; then
        DOCKER_RUN_OPTS+=(--network host)
    else
        DOCKER_RUN_OPTS+=(--network bridge)
    fi
    if [[ "${WITH_HOST_IPC,,}" = true ]]; then
        DOCKER_RUN_OPTS+=(--ipc host)
    else
        DOCKER_RUN_OPTS+=(--ipc private --shm-size "${SRB_DOCKER_SHM_SIZE}")
    fi
fi
DOCKER_TTY="${DOCKER_TTY:-true}"
if [[ "${DOCKER_TTY,,}" = true ]]; then
    DOCKER_RUN_OPTS+=(--tty)
fi
WITH_GPU="${WITH_GPU:-${_PROFILE_WITH_GPU}}"
WITH_GUI="${WITH_GUI:-${_PROFILE_WITH_GUI}}"
WITH_HISTORY="${WITH_HISTORY:-${_PROFILE_WITH_HISTORY}}"
WITH_DEV_VOLUME="${WITH_DEV_VOLUME:-${_PROFILE_WITH_DEV_VOLUME}}"
ENSURE_DOCKER="${ENSURE_DOCKER:-false}"
SRB_REQUIRE_GPU="${SRB_REQUIRE_GPU:-false}"
SRB_REQUIRE_VULKAN="${SRB_REQUIRE_VULKAN:-false}"
SRB_REQUIRE_RTX="${SRB_REQUIRE_RTX:-false}"
SRB_REQUIRE_DISPLAY="${SRB_REQUIRE_DISPLAY:-false}"
SRB_REQUIRE_XVFB="${SRB_REQUIRE_XVFB:-false}"
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    SRB_REQUIRE_GPU=true
    SRB_REQUIRE_VULKAN=true
    SRB_REQUIRE_RTX=true
    if [[ "${SRB_NGC_MODE}" = gui ]]; then
        SRB_REQUIRE_DISPLAY=true
    elif [[ "${WITH_GUI,,}" = true ]]; then
        echo >&2 "[ERROR] NGC ${SRB_NGC_MODE} mode is display-free; use SRB_NGC_MODE=gui for a real-display lane."
        exit 2
    fi
fi
DOCKER_VOLUMES=()
DOCKER_ENVIRON=(
    DEFAULT_SHELL="${DEFAULT_SHELL:-"fish"}"
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-"0"}"
)
if [[ "${SRB_DOCKER_PROFILE}" != safe && "${SRB_NGC_ENABLED}" != true ]]; then
    DOCKER_VOLUMES=(
        "/etc/localtime:/etc/localtime:ro"
        "/etc/timezone:/etc/timezone:ro"
        "${HOME}/.nvidia-omniverse/data/isaac-sim:/root/isaac-sim/kit/data:rw"
        "${HOME}/.cache/isaac-sim:/root/isaac-sim/kit/cache:rw"
        "${HOME}/.cache/nvidia/GLCache:/root/.cache/nvidia/GLCache:rw"
        "${HOME}/.cache/ov:/root/.cache/ov:rw"
        "${HOME}/.nv/ComputeCache:/root/.nv/ComputeCache:rw"
        "${HOME}/.nvidia-omniverse/logs:/root/.nvidia-omniverse/logs:rw"
        "${HOME}/.nvidia-omniverse/logs/isaac-sim:/root/isaac-sim/kit/logs:rw"
        "${HOME}/.cache/simforge:/root/.cache/simforge:rw"
    )
fi
NGC_ARCH=""
NGC_PLATFORM=""
NGC_CACHE_ROOT=""
NGC_CACHE_VOLUMES=()
NGC_CACHE_HOST_PATHS=()
NGC_WORKSPACE="${SRB_NGC_WORKSPACE:-/workspace}"
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    NGC_PLATFORM_INPUT="${SRB_NGC_PLATFORM:-${SRB_DOCKER_PLATFORM:-${DOCKER_DEFAULT_PLATFORM:-${TARGETPLATFORM:-}}}}"
    if [[ -z "${NGC_PLATFORM_INPUT}" ]]; then
        NGC_PLATFORM_INPUT="$(uname -m)"
    fi
    case "${NGC_PLATFORM_INPUT,,}" in
        linux/amd64|amd64|x86_64)
            NGC_ARCH=amd64
            NGC_PLATFORM=linux/amd64
            NGC_IMAGE_DIGEST="${NGC_AMD64_DIGEST}"
            ;;
        linux/arm64|arm64|aarch64)
            NGC_ARCH=arm64
            NGC_PLATFORM=linux/arm64
            NGC_IMAGE_DIGEST="${NGC_ARM64_DIGEST}"
            ;;
        *)
            echo >&2 "[ERROR] Unsupported NGC architecture/platform '${NGC_PLATFORM_INPUT}'. Expected linux/amd64 or linux/arm64."
            exit 2
            ;;
    esac
    NGC_CACHE_ROOT="${SRB_NGC_CACHE_DIR:-${HOME}/.cache/space_robotics_bench/ngc/isaac-sim-6.1/${NGC_ARCH}}"
    if [[ -z "${NGC_CACHE_ROOT}" || "${NGC_CACHE_ROOT}" != /* ]]; then
        echo >&2 "[ERROR] SRB_NGC_CACHE_DIR must be an absolute path to a dedicated cache directory."
        exit 2
    fi
    # `chown -R 1234:1234` runs over its subdirectories; admit only a dedicated directory.
    NGC_CACHE_ROOT="$(readlink -m -- "${NGC_CACHE_ROOT}")"
    for protected_dir in / "${HOME:-}" "${REPOSITORY_DIR}"; do
        [[ -n "${protected_dir}" ]] || continue
        protected_dir="$(readlink -m -- "${protected_dir}")"
        if [[ "${NGC_CACHE_ROOT}" = "${protected_dir}" || "${protected_dir}" = "${NGC_CACHE_ROOT%/}"/* ]]; then
            echo >&2 "[ERROR] SRB_NGC_CACHE_DIR must name a dedicated cache directory; '${NGC_CACHE_ROOT}' is '${protected_dir}' or contains it."
            exit 2
        fi
    done
    NGC_CACHE_VOLUMES=(
        "${NGC_CACHE_ROOT}/cache:/isaac-sim/.cache:rw"
        "${NGC_CACHE_ROOT}/compute:/isaac-sim/.nv/ComputeCache:rw"
        "${NGC_CACHE_ROOT}/kit:/isaac-sim/kit/cache:rw"
        "${NGC_CACHE_ROOT}/logs:/isaac-sim/.nvidia-omniverse/logs:rw"
        "${NGC_CACHE_ROOT}/config:/isaac-sim/.nvidia-omniverse/config:rw"
        "${NGC_CACHE_ROOT}/data:/isaac-sim/.local/share/ov/data:rw"
        "${NGC_CACHE_ROOT}/pkg:/isaac-sim/.local/share/ov/pkg:rw"
        "${NGC_CACHE_ROOT}/hub:/var/cache/hub:rw"
    )
    for volume in "${NGC_CACHE_VOLUMES[@]}"; do
        NGC_CACHE_HOST_PATHS+=("${volume%%:*}")
    done
    DOCKER_VOLUMES+=("${NGC_CACHE_VOLUMES[@]}")
    DOCKER_ENVIRON+=(
        ACCEPT_EULA="${ACCEPT_EULA:-Y}"
        PRIVACY_CONSENT="${PRIVACY_CONSENT:-Y}"
        ISAACSIM_PATH=/isaac-sim
        ISAACSIM_PYTHON=/isaac-sim/python.sh
        ISAACSIM_PYTHON_EXE=/isaac-sim/python.sh
        SRB_NGC_ARCH="${NGC_ARCH}"
        SRB_NGC_PLATFORM="${NGC_PLATFORM}"
    )
    if [[ "${SRB_NGC_MODE}" = webrtc ]]; then
        if [[ "${WITH_HOST_NETWORK,,}" != true ]]; then
            echo >&2 "[ERROR] SRB_NGC_MODE=webrtc requires WITH_HOST_NETWORK=true for the vendor streaming ports."
            exit 2
        fi
        DOCKER_ENVIRON+=(
            ISAACSIM_HOST="${ISAACSIM_HOST:-127.0.0.1}"
            ISAACSIM_WEBRTC_TCP_PORT="${ISAACSIM_WEBRTC_TCP_PORT:-49100}"
            ISAACSIM_WEBRTC_UDP_PORT="${ISAACSIM_WEBRTC_UDP_PORT:-47998}"
        )
    fi
    if [[ "${WITH_DEV_VOLUME,,}" = true ]]; then
        case "${SRB_NGC_WORKSPACE_MODE:-rw}" in
            ro|rw) ;;
            *)
                echo >&2 "[ERROR] SRB_NGC_WORKSPACE_MODE must be ro or rw."
                exit 2
                ;;
        esac
        DOCKER_VOLUMES+=("${REPOSITORY_DIR}:${NGC_WORKSPACE}:${SRB_NGC_WORKSPACE_MODE}")
        DOCKER_ENVIRON+=(
            SRB_NGC_WORKSPACE="${NGC_WORKSPACE}"
            PYTHONPATH="${NGC_WORKSPACE}${PYTHONPATH:+:${PYTHONPATH}}"
        )
    fi
fi

if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    SRB_NGC_CACHE_OWNERSHIP="${SRB_NGC_CACHE_OWNERSHIP:-chown}"
    case "${SRB_NGC_CACHE_OWNERSHIP,,}" in
        chown|skip) ;;
        *)
            echo >&2 "[ERROR] SRB_NGC_CACHE_OWNERSHIP must be chown or skip."
            exit 2
            ;;
    esac
    for host_dir in "${NGC_CACHE_HOST_PATHS[@]}"; do
        mkdir -p "${host_dir}"
    done
    if [[ "${SRB_NGC_CACHE_OWNERSHIP,,}" = chown ]]; then
        # Only the mounted subdirectories, never other content of the root.
        if ! "${WITH_SUDO[@]}" chown -R 1234:1234 "${NGC_CACHE_HOST_PATHS[@]}"; then
            echo >&2 "[ERROR] Could not provision ${NGC_CACHE_ROOT} for NGC UID 1234. Pre-own it or set SRB_NGC_CACHE_OWNERSHIP=skip after an ACL/write probe."
            exit 2
        fi
    fi
fi
if [[ "${SRB_DOCKER_PROFILE}" = trusted-simulator ]]; then
    DOCKER_VOLUMES+=("/dev:/dev")
fi

if [[ "${WITH_HISTORY,,}" = true ]]; then
    DOCKER_VOLUMES+=(
        "${SCRIPT_DIR}/.history:/history:rw"
        "${SCRIPT_DIR}/.history/fish:/root/.local/share/fish:rw"
    )
    DOCKER_ENVIRON+=(
        HISTFILE="/history/bash_history"
        HISTCONTROL="ignoredups:erasedups"
        PROMPT_COMMAND="history -a; history -c; history -r"
        PYTHON_HISTORY="/history/python_history"
    )
fi
if [[ "${WITH_DEV_VOLUME,,}" = true && "${SRB_NGC_ENABLED}" != true ]]; then
    DOCKER_VOLUMES+=(
        "${REPOSITORY_DIR}:/root/ws:rw"
    )
fi

if [[ "${SRB_DOCKER_PROFILE}" != safe && "${SRB_NGC_ENABLED}" != true && -f /usr/share/nvidia/nvoptix.bin ]]; then
    DOCKER_VOLUMES+=(
        "/usr/share/nvidia/nvoptix.bin:/usr/share/nvidia/nvoptix.bin:ro"
    )
fi

if [[ -n "${RMW_IMPLEMENTATION}" ]]; then
    DOCKER_ENVIRON+=("RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}")
    if [[ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]]; then
        if [ -n "${CYCLONEDDS_URI}" ]; then
            if [[ "${CYCLONEDDS_URI}" =~ ^file://.* ]]; then
                DOCKER_VOLUMES+=("${CYCLONEDDS_URI//file:\/\//}:/root/.ros/cyclonedds.xml:ro")
                DOCKER_ENVIRON+=("CYCLONEDDS_URI=file:///root/.ros/cyclonedds.xml")
            else
                DOCKER_ENVIRON+=("CYCLONEDDS_URI=${CYCLONEDDS_URI}")
            fi
        else
            DOCKER_ENVIRON+=("CYCLONEDDS_URI=<CycloneDDS><Domain><General><RedundantNetworking>true</RedundantNetworking><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>")
        fi
    elif [[ "${RMW_IMPLEMENTATION}" = "rmw_fastrtps_cpp" ]]; then
        if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then
            DOCKER_VOLUMES+=("${FASTRTPS_DEFAULT_PROFILES_FILE}:/root/.ros/fastrtps.xml:ro")
            DOCKER_ENVIRON+=("FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastrtps.xml")
        fi
    fi
fi

if [[ "${ENSURE_DOCKER,,}" = true ]]; then
    if ! command -v docker >/dev/null 2>&1 || ! command -v nvidia-container-toolkit >/dev/null 2>&1; then
        if ! command -v curl >/dev/null 2>&1; then
            if command -v apt-get >/dev/null 2>&1; then
                sudo apt-get install -y curl
            elif command -v dnf >/dev/null 2>&1; then
                sudo dnf install -y curl
            elif command -v yum >/dev/null 2>&1; then
                sudo yum install -y curl
            fi
        fi
        bash "${SCRIPT_DIR}/host/install_docker.bash"
    fi
fi

DOCKERHUB_USER="$("${WITH_SUDO[@]}" docker info 2>/dev/null | sed '/Username:/!d;s/.* //')"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"
IMAGE_REFERENCE="${SRB_DOCKER_IMAGE:-}"
EXPLICIT_IMAGE=false
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    IMAGE_REFERENCE="${SRB_NGC_IMAGE:-${IMAGE_REFERENCE:-${NGC_IMAGE_REPOSITORY}@${NGC_IMAGE_DIGEST}}}"
    EXPECTED_NGC_IMAGE="${NGC_IMAGE_REPOSITORY}@${NGC_IMAGE_DIGEST}"
    if [[ "${IMAGE_REFERENCE}" != "${EXPECTED_NGC_IMAGE}" ]]; then
        echo >&2 "[ERROR] NGC image must be the architecture-specific Isaac Sim 6.1 digest ${EXPECTED_NGC_IMAGE}; tags and other digests are rejected."
        exit 2
    fi
    EXPLICIT_IMAGE=true
elif [[ -n "${IMAGE_REFERENCE}" ]]; then
    EXPLICIT_IMAGE=true
else
    LOCAL_IMAGE_CANDIDATES=(
        "${DOCKERHUB_USER:+${DOCKERHUB_USER}/}${PROJECT_NAME,,}"
        "${PROJECT_NAME,,}"
    )
    for candidate in "${LOCAL_IMAGE_CANDIDATES[@]}"; do
        if "${WITH_SUDO[@]}" docker image inspect "${candidate}" >/dev/null 2>&1; then
            IMAGE_REFERENCE="${candidate}"
            break
        fi
    done
    if [[ -z "${IMAGE_REFERENCE}" ]]; then
        echo >&2 "[ERROR] No local SRB image exists. Build one with ./.docker/build.bash or set SRB_DOCKER_IMAGE to an image pinned as NAME@sha256:DIGEST."
        exit 2
    fi
fi

# Suffix a number only if the exact name is taken; join.bash derives the same names.
CONTAINER_NAME="${SRB_CONTAINER_NAME:-${PROJECT_NAME//[^a-zA-Z0-9]/_}}"
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    CONTAINER_NAME+="_ngc"
fi
ALL_CONTAINER_NAMES=$("${WITH_SUDO[@]}" docker container list --all --format "{{.Names}}")
container_name_taken() {
    grep -qixF -- "${1}" <<<"${ALL_CONTAINER_NAMES}"
}
if container_name_taken "${CONTAINER_NAME}"; then
    ID=1
    while container_name_taken "${CONTAINER_NAME}${ID}"; do
        ID=$((ID + 1))
    done
    CONTAINER_NAME="${CONTAINER_NAME}${ID}"
fi
DOCKER_RUN_OPTS+=("--name" "${CONTAINER_NAME}")

CALLER_ENVIRON=()
while getopts ":v:e:" opt; do
    case "${opt}" in
        v) DOCKER_VOLUMES+=("${OPTARG}") ;;
        e) CALLER_ENVIRON+=("${OPTARG}") ;;
        *)
            echo >&2 "Usage: ${0} [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE|-e ENV] [TAG] [CMD]"
            exit 2
            ;;
    esac
done
shift "$((OPTIND - 1))"

declare -A CALLER_ENV_INDEX=()
CALLER_ENV_LINES=()
entry_position=0
for entry in "${CALLER_ENVIRON[@]}"; do
    entry_position=$((entry_position + 1))
    entry_label="-e entry #${entry_position} (${#entry} characters)"
    if [[ "${entry}" == *$'\n'* ]]; then
        echo >&2 "[ERROR] ${entry_label}: a newline cannot be forwarded through an env-file."
        exit 2
    fi
    name="${entry%%=*}"
    if [[ "${entry}" != *=* ]]; then
        if [[ ! "${name}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
            echo >&2 "[ERROR] ${entry_label}: expected NAME=VALUE, or NAME of an exported variable."
            exit 2
        fi
        if [[ -z "${CALLER_EXPORTED_ENV[${name}]+set}" ]]; then
            echo >&2 "[WARNING] ${entry_label}: not exported by the calling environment; not forwarded."
            continue
        fi
        entry="${name}=${CALLER_EXPORTED_ENV[${name}]}"
    fi
    if [[ -z "${name}" || "${name}" == *[[:space:]]* || "${name}" == \#* ]]; then
        echo >&2 "[ERROR] ${entry_label}: the name before '=' must be non-empty, contain no whitespace and not start with '#'."
        exit 2
    fi
    if [[ -n "${CALLER_ENV_INDEX[${name}]+set}" ]]; then
        CALLER_ENV_LINES["${CALLER_ENV_INDEX[${name}]}"]="${entry}"
    else
        CALLER_ENV_INDEX[${name}]="${#CALLER_ENV_LINES[@]}"
        CALLER_ENV_LINES+=("${entry}")
    fi
done
if [[ "${#CALLER_ENV_LINES[@]}" -gt 0 ]]; then
    SCRIPT_ENVIRON=()
    for entry in "${DOCKER_ENVIRON[@]}"; do
        [[ -n "${CALLER_ENV_INDEX[${entry%%=*}]+set}" ]] || SCRIPT_ENVIRON+=("${entry}")
    done
    DOCKER_ENVIRON=("${SCRIPT_ENVIRON[@]}")
fi

if [[ "${EXPLICIT_IMAGE}" = false && "${#}" -gt "0" ]]; then
    if "${WITH_SUDO[@]}" docker images --format "{{.Tag}}" "${IMAGE_REFERENCE}" | grep -Fx -- "${1}" >/dev/null; then
        IMAGE_REFERENCE+=":${1}"
        shift
    fi
fi
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    IMAGE_NAME="${IMAGE_REFERENCE}"
elif "${WITH_SUDO[@]}" docker image inspect "${IMAGE_REFERENCE}" >/dev/null 2>&1; then
    IMAGE_NAME="$("${WITH_SUDO[@]}" docker image inspect --format '{{.Id}}' "${IMAGE_REFERENCE}")"
elif [[ "${IMAGE_REFERENCE}" =~ @sha256:[0-9a-f]{64}$ ]]; then
    IMAGE_NAME="${IMAGE_REFERENCE}"
else
    echo >&2 "[ERROR] SRB_DOCKER_IMAGE must be local or pinned by sha256 digest: ${IMAGE_REFERENCE}"
    exit 2
fi
if [[ "${SRB_NGC_ENABLED}" = true ]]; then
    DOCKER_RUN_OPTS+=(--platform "${NGC_PLATFORM}")
    DOCKER_RUN_OPTS+=(--label "com.srb.profile=ngc")
    DOCKER_RUN_OPTS+=(--label "com.srb.image=${IMAGE_NAME}")
    DOCKER_RUN_OPTS+=(--label "com.srb.ngc.arch=${NGC_ARCH}")
else
    DOCKER_RUN_OPTS+=(--label "com.srb.profile=${SRB_DOCKER_PROFILE}")
    DOCKER_RUN_OPTS+=(--label "com.srb.image=${IMAGE_NAME}")
fi
DOCKER_RUN_OPTS+=(--env "SRB_IMAGE_ID=${IMAGE_NAME}" --env "SRB_HOST=${SRB_HOST:-$(hostname)}")
CMD=("$@")
NGC_DEFAULT_HEADLESS=false
if [[ "${SRB_NGC_ENABLED}" = true && "${#CMD[@]}" -eq 0 ]]; then
    if [[ "${SRB_NGC_MODE}" = headless || "${SRB_NGC_MODE}" = webrtc ]]; then
        DOCKER_RUN_OPTS+=(--entrypoint /isaac-sim/runheadless.sh)
        CMD=(-v)
        NGC_DEFAULT_HEADLESS=true
    else
        CMD=(bash)
    fi
fi
if [[ "${NGC_DEFAULT_HEADLESS}" != true && "${#CMD[@]}" -gt 0 && "${#DOCKER_ENVIRON[@]}" -gt 0 ]]; then
    CMD=(env "${DOCKER_ENVIRON[@]}" "${CMD[@]}")
fi

if [[ "${WITH_GPU,,}" = true ]]; then
    if [[ "${SRB_REQUIRE_GPU,,}" = true && -n "${WITH_GPU_FORCE_NVIDIA:-}" ]]; then
        echo >&2 "[ERROR] Required GPU preflight rejects WITH_GPU_FORCE_NVIDIA; host capability must be verified by nvidia-smi."
        exit 2
    fi
    check_nvidia_gpu() {
        if [[ -n "${WITH_GPU_FORCE_NVIDIA:-}" ]]; then
            if [[ "${WITH_GPU_FORCE_NVIDIA,,}" = true ]]; then
                echo "[INFO] NVIDIA GPU is force-enabled via 'WITH_GPU_FORCE_NVIDIA=true'."
                return 0 # NVIDIA GPU is force-enabled
            else
                echo "[INFO] NVIDIA GPU is force-disabled via 'WITH_GPU_FORCE_NVIDIA=false'."
                return 1 # NVIDIA GPU is force-disabled
            fi
        elif ! lshw -C display 2>/dev/null | grep -qi "vendor.*nvidia"; then
            return 1 # NVIDIA GPU is not present
        elif ! command -v nvidia-smi >/dev/null 2>&1; then
            echo >&2 -e "\033[1;33m[WARNING] NVIDIA GPU is detected, but its functionality cannot be verified. This container will not be able to use the GPU. Please install nvidia-utils on the host system or force-enable NVIDIA GPU via 'WITH_GPU_FORCE_NVIDIA=true'.\033[0m"
            return 1 # NVIDIA GPU is present but nvidia-utils not installed
        elif ! nvidia-smi -L &>/dev/null; then
            echo >&2 -e "\033[1;33m[WARNING] NVIDIA GPU is detected, but it does not seem to be working properly. This container will not be able to use the GPU. Please ensure the NVIDIA drivers are properly installed on the host system.\033[0m"
            return 1 # NVIDIA GPU is present but is not working properly
        else
            return 0 # NVIDIA GPU is present and appears to be working
        fi
    }
    if check_nvidia_gpu; then
        DOCKER_VERSION="$("${WITH_SUDO[@]}" docker version --format '{{.Server.Version}}')"
        MIN_VERSION_FOR_TOOLKIT="19.3"
        if [ "$(printf '%s\n' "${MIN_VERSION_FOR_TOOLKIT}" "${DOCKER_VERSION}" | sort -V | head -n1)" = "$MIN_VERSION_FOR_TOOLKIT" ]; then
            DOCKER_RUN_OPTS+=("--gpus" "all")
        else
            DOCKER_RUN_OPTS+=("--runtime" "nvidia")
        fi
        DOCKER_ENVIRON+=(
            NVIDIA_VISIBLE_DEVICES="all"
            NVIDIA_DRIVER_CAPABILITIES="all"
        )
    elif [[ "${SRB_REQUIRE_GPU,,}" = true ]]; then
        echo >&2 "[ERROR] Required NVIDIA GPU preflight failed; refusing to launch without a verified host GPU."
        exit 2
    fi
    if [[ -e /dev/dri ]]; then
        DOCKER_RUN_OPTS+=("--device=/dev/dri:/dev/dri")
        if [[ $(getent group video) ]]; then
            DOCKER_RUN_OPTS+=("--group-add" "video")
        fi
    fi
elif [[ "${SRB_REQUIRE_GPU,,}" = true ]]; then
    echo >&2 "[ERROR] Required GPU preflight needs WITH_GPU=true."
    exit 2
fi

if [[ "${SRB_REQUIRE_VULKAN,,}" = true ]]; then
    if ! command -v vulkaninfo >/dev/null 2>&1; then
        echo >&2 "[ERROR] Required Vulkan preflight needs vulkaninfo on the host."
        exit 2
    fi
    if ! vulkaninfo --summary >/dev/null 2>&1; then
        echo >&2 "[ERROR] Required Vulkan preflight failed; refusing to launch."
        exit 2
    fi
fi
# First NVIDIA device with hardware ray tracing; software and GTX devices do not
# count. Keep in step with .docker/hpc/run.bash.
srb_vulkan_rt_device() {
    awk '
        function flush() {
            if (in_dev && nvidia && accel && rtp && !gtx && found == "") found = name
        }
        /^GPU[0-9]+:?[ \t]*$/ {
            flush(); in_dev = 1; nvidia = 0; accel = 0; rtp = 0; gtx = 0; name = ""
            next
        }
        !in_dev { next }
        /^[ \t]*vendorID[ \t]*=/ { if ($0 ~ /0x10[dD][eE]/ || $0 ~ /=[ \t]*4318/) nvidia = 1 }
        /^[ \t]*deviceName[ \t]*=/ {
            name = $0; sub(/^[ \t]*deviceName[ \t]*=[ \t]*/, "", name)
            if (name ~ /GTX/) gtx = 1
        }
        /^[ \t]*VK_KHR_acceleration_structure[ \t]*:/ { accel = 1 }
        /^[ \t]*VK_KHR_ray_tracing_pipeline[ \t]*:/ { rtp = 1 }
        END { flush(); if (found == "") exit 1; print found }
    '
}
if [[ "${SRB_REQUIRE_RTX,,}" = true ]]; then
    if ! command -v vulkaninfo >/dev/null 2>&1; then
        echo >&2 "[ERROR] Required RTX preflight could not read the Vulkan report: vulkaninfo is not installed."
        exit 2
    fi
    VULKAN_REPORT_STATUS=0
    VULKAN_REPORT="$(vulkaninfo 2>/dev/null)" || VULKAN_REPORT_STATUS=$?
    if ! RTX_DEVICE="$(srb_vulkan_rt_device <<<"${VULKAN_REPORT}")"; then
        if grep -Eq '^GPU[0-9]+:?[[:space:]]*$' <<<"${VULKAN_REPORT}"; then
            echo >&2 "[ERROR] Required RTX preflight found no NVIDIA Vulkan device with hardware ray tracing (VK_KHR_ray_tracing_pipeline + VK_KHR_acceleration_structure)."
        elif [[ "${VULKAN_REPORT_STATUS}" -ne 0 ]]; then
            echo >&2 "[ERROR] Required RTX preflight could not read the Vulkan report: vulkaninfo failed (exit ${VULKAN_REPORT_STATUS})."
        else
            echo >&2 "[ERROR] Required RTX preflight could not read the Vulkan report: vulkaninfo listed no Vulkan device."
        fi
        exit 2
    fi
    echo "[INFO] RTX preflight: hardware ray tracing on '${RTX_DEVICE}'."
fi

if [[ "${WITH_GUI,,}" = true ]]; then
    if [[ "${SRB_NGC_ENABLED}" = true && "${SRB_NGC_MODE}" = gui && "${DISPLAY:-}" = :99* && "${SRB_NGC_ALLOW_XVFB:-false}" != true ]]; then
        echo >&2 "[ERROR] NGC GUI requires a real display; source-image Xvfb :99 is a separate lane."
        exit 2
    fi
    if [[ -n "${DISPLAY:-}" ]]; then
        if [[ "${SRB_REQUIRE_XVFB,,}" = true ]]; then
            if [[ "${DISPLAY}" != :99 && "${DISPLAY}" != :99.* ]]; then
                echo >&2 "[ERROR] Required Xvfb preflight needs DISPLAY=:99."
                exit 2
            fi
            if [[ ! -e /tmp/.X11-unix/X99 ]]; then
                echo >&2 "[ERROR] Required Xvfb preflight could not find /tmp/.X11-unix/X99."
                exit 2
            fi
        fi
        if ! command -v xauth >/dev/null 2>&1; then
            echo >&2 "[ERROR] GUI preflight needs xauth to provision Xauthority."
            exit 2
        fi
        # The private directory protects the host while the bound cookie stays readable.
        XAUTH_DIR="${TMPDIR:-"/tmp"}/srb-xauth-${PROJECT_NAME}-${USER}"
        mkdir -m 700 -- "${XAUTH_DIR}" 2>/dev/null || :
        if [[ -L "${XAUTH_DIR}" || ! -d "${XAUTH_DIR}" || ! -O "${XAUTH_DIR}" ]]; then
            echo >&2 "[ERROR] Xauthority directory '${XAUTH_DIR}' is not a directory owned by '${USER}'; remove it and retry."
            exit 2
        fi
        chmod 700 "${XAUTH_DIR}"
        XAUTH="${XAUTH_DIR}/Xauthority"
        touch "${XAUTH}"
        if ! XAUTH_LIST=$(xauth nlist "${DISPLAY}" 2>/dev/null); then
            if [[ "${SRB_REQUIRE_DISPLAY,,}" = true ]]; then
                echo >&2 "[ERROR] Required display preflight could not read Xauthority for ${DISPLAY}."
                exit 2
            fi
            XAUTH_LIST=""
        fi
        if [ -n "${XAUTH_LIST}" ]; then
            if ! echo "${XAUTH_LIST}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge -; then
                echo >&2 "[ERROR] Could not create the container Xauthority file."
                exit 2
            fi
        fi
        chmod a+r "${XAUTH}"
        if [[ "${SRB_REQUIRE_DISPLAY,,}" = true && ! -r "${XAUTH}" ]]; then
            echo >&2 "[ERROR] Required display preflight could not create Xauthority."
            exit 2
        fi
        DOCKER_VOLUMES+=(
            "${XAUTH}:${XAUTH}"
            "/tmp/.X11-unix:/tmp/.X11-unix"
            "/dev/input:/dev/input"
        )
        DOCKER_ENVIRON+=(
            DISPLAY="${DISPLAY}"
            XAUTHORITY="${XAUTH}"
        )
    else
        if [[ "${SRB_REQUIRE_DISPLAY,,}" = true ]]; then
            echo >&2 "[ERROR] Required display preflight needs DISPLAY and an accessible Xauthority."
            exit 2
        fi
        echo >&2 -e "\033[1;33m[WARNING] DISPLAY variable is not set. GUI will not be enabled in the container.\033[0m"
    fi
fi
if [[ "${SRB_REQUIRE_DISPLAY,,}" = true && "${WITH_GUI,,}" != true ]]; then
    echo >&2 "[ERROR] Required display preflight needs WITH_GUI=true."
    exit 2
fi

NGC_READINESS_CHECK="${SRB_NGC_READINESS_CHECK:-false}"
NGC_READINESS_PROBE=false
if [[ "${SRB_NGC_ENABLED}" = true && "${NGC_DEFAULT_HEADLESS}" = true && "${NGC_READINESS_CHECK,,}" = true ]]; then
    NGC_READINESS_PROBE=true
    NGC_READY_TIMEOUT_S="${SRB_NGC_READY_TIMEOUT_S:-120}"
    if [[ ! "${NGC_READY_TIMEOUT_S}" =~ ^[1-9][0-9]*$ ]]; then
        echo >&2 "[ERROR] SRB_NGC_READY_TIMEOUT_S must be a positive integer."
        exit 2
    fi
    NGC_STATE_DIR="$(mktemp -d)"
    NGC_CIDFILE="${NGC_STATE_DIR}/cid"
    NGC_CONTAINER_ID=""
    NGC_KEEP_CONTAINER=false
    # Remove the probe container so it releases GPU memory and streaming ports.
    ngc_remove_container() {
        local container_filter remaining_containers
        if [[ "${NGC_CONTAINER_ID}" =~ ^[0-9a-f]{64}$ ]]; then
            container_filter="id=${NGC_CONTAINER_ID}"
        else
            container_filter="name=^/${NGC_CONTAINER_ID}$"
        fi
        "${WITH_SUDO[@]}" docker rm --force "${NGC_CONTAINER_ID}" >/dev/null 2>&1 || :
        if ! remaining_containers="$("${WITH_SUDO[@]}" docker container ls --all --no-trunc --filter "${container_filter}" --format '{{.ID}}' 2>/dev/null)"; then
            echo >&2 "[ERROR] Could not verify removal of NGC container ${NGC_CONTAINER_ID} (${CONTAINER_NAME}); Docker container listing failed."
            return 1
        fi
        if [[ -n "${remaining_containers}" ]]; then
            echo >&2 "[ERROR] Could not remove NGC container ${NGC_CONTAINER_ID} (${CONTAINER_NAME}); it still holds the GPU and host ports. Remove it with: docker rm --force ${NGC_CONTAINER_ID}"
            return 1
        fi
    }
    ngc_probe_exit_cleanup() {
        local exit_status=$? cid_from_file
        trap - EXIT INT TERM HUP
        if [[ "${NGC_KEEP_CONTAINER}" != true ]]; then
            cid_from_file="$(cat "${NGC_CIDFILE}" 2>/dev/null || :)"
            if [[ "${cid_from_file}" =~ ^[0-9a-f]{64}$ ]]; then
                NGC_CONTAINER_ID="${cid_from_file}"
            elif [[ "${NGC_DOCKER_STARTED:-false}" = true ]]; then
                NGC_CONTAINER_ID="${CONTAINER_NAME}"
            else
                NGC_CONTAINER_ID=""
            fi
            if [[ -n "${NGC_CONTAINER_ID}" ]] && ! ngc_remove_container; then
                (( exit_status != 0 )) || exit_status=2
            fi
        fi
        if ! rm -rf -- "${NGC_STATE_DIR}"; then
            echo >&2 "[ERROR] Could not remove NGC probe state directory ${NGC_STATE_DIR}."
            (( exit_status != 0 )) || exit_status=2
        fi
        exit "${exit_status}"
    }
    trap 'ngc_probe_exit_cleanup' EXIT
    trap 'exit 129' HUP
    trap 'exit 130' INT
    trap 'exit 143' TERM
    NGC_DOCKER_RUN_OPTS=()
    for option in "${DOCKER_RUN_OPTS[@]}"; do
        [[ "${option}" = --rm ]] || NGC_DOCKER_RUN_OPTS+=("${option}")
    done
    NGC_DOCKER_RUN_OPTS+=(--detach --cidfile "${NGC_CIDFILE}")
    DOCKER_RUN_OPTS=("${NGC_DOCKER_RUN_OPTS[@]}")
fi

VOL_ARGS=()
for vol in "${DOCKER_VOLUMES[@]}"; do
    VOL_ARGS+=("--volume" "${vol}")
done
ENV_ARGS=()
for env_var in "${DOCKER_ENVIRON[@]}"; do
    ENV_ARGS+=("--env" "${env_var}")
done
# Keep caller secrets out of argv and disk; Docker reads the private FIFO once.
ENV_FILE_FIFO=""
ENV_FILE_REAPER_PID=""
if [[ "${#CALLER_ENV_LINES[@]}" -gt 0 ]]; then
    ENV_FILE_BASE="${XDG_RUNTIME_DIR:-}"
    if [[ -z "${ENV_FILE_BASE}" || ! -d "${ENV_FILE_BASE}" || ! -w "${ENV_FILE_BASE}" ]]; then
        ENV_FILE_BASE="${TMPDIR:-/tmp}"
    fi
    ENV_FILE_DIR="$(mktemp -d "${ENV_FILE_BASE%/}/srb-docker-env.XXXXXXXX")"
    ENV_FILE_FIFO="${ENV_FILE_DIR}/env"
    mkfifo -m 600 "${ENV_FILE_FIFO}"
    ENV_FILE_OWNER_PID=$$
    (
        (printf '%s\n' "${CALLER_ENV_LINES[@]}" >"${ENV_FILE_FIFO}") &
        writer_pid=$!
        while kill -0 "${writer_pid}" 2>/dev/null; do
            if ! kill -0 "${ENV_FILE_OWNER_PID}" 2>/dev/null; then
                kill "${writer_pid}" 2>/dev/null || :
                break
            fi
            sleep 0.1
        done
        wait "${writer_pid}" 2>/dev/null || :
        rm -rf -- "${ENV_FILE_DIR}"
    ) </dev/null >/dev/null 2>&1 &
    ENV_FILE_REAPER_PID=$!
    ENV_ARGS+=(--env-file "${ENV_FILE_FIFO}")
fi
release_env_file() {
    [[ -n "${ENV_FILE_REAPER_PID}" ]] || return 0
    if [[ -p "${ENV_FILE_FIFO}" ]]; then
        { true <>"${ENV_FILE_FIFO}"; } 2>/dev/null || :
    fi
    wait "${ENV_FILE_REAPER_PID}" 2>/dev/null || :
    ENV_FILE_REAPER_PID=""
}
DOCKER_RUN_CMD=(
    "${WITH_SUDO[@]}"
    docker run
    --init
    "${DOCKER_RUN_OPTS[@]}"
    "${VOL_ARGS[@]}"
    "${ENV_ARGS[@]}"
    "${IMAGE_NAME}"
    "${CMD[@]}"
)
# Trace counts only; environment values and command arguments may contain secrets.
printf "\033[1;90m[TRACE] docker run profile=%q image=%q volumes=%d env=%d env_file=%d command_args=%d\033[0m\n" \
    "${SRB_DOCKER_PROFILE}" "${IMAGE_NAME}" "${#DOCKER_VOLUMES[@]}" \
    "${#DOCKER_ENVIRON[@]}" "${#CALLER_ENV_LINES[@]}" "${#CMD[@]}"
if [[ "${NGC_READINESS_PROBE}" = true ]]; then
    if ! "${DOCKER_RUN_CMD[@]}" >/dev/null; then
        release_env_file
        echo >&2 "[ERROR] NGC headless launcher failed to start."
        exit 2
    fi
    release_env_file
    NGC_DOCKER_STARTED=true
    NGC_CONTAINER_ID="$(cat "${NGC_CIDFILE}" 2>/dev/null || :)"
    if [[ ! "${NGC_CONTAINER_ID}" =~ ^[0-9a-f]{64}$ ]]; then
        NGC_CONTAINER_ID="${CONTAINER_NAME}"
    fi
    NGC_READY=false
    NGC_READY_DEADLINE=$((SECONDS + NGC_READY_TIMEOUT_S))
    while (( SECONDS < NGC_READY_DEADLINE )); do
        NGC_LOG="$("${WITH_SUDO[@]}" docker logs "${NGC_CONTAINER_ID}" 2>&1 || :)"
        if grep -Fq "${SRB_NGC_READY_PATTERN:-Isaac Sim Full Streaming App is loaded}" <<<"${NGC_LOG}"; then
            NGC_READY=true
            break
        fi
        NGC_STATE="$("${WITH_SUDO[@]}" docker inspect --format '{{.State.Status}}' "${NGC_CONTAINER_ID}" 2>/dev/null || :)"
        if [[ "${NGC_STATE}" = exited || "${NGC_STATE}" = dead ]]; then
            break
        fi
        sleep 1
    done
    if [[ "${NGC_READY}" != true ]]; then
        echo >&2 "[ERROR] NGC readiness line was not observed within ${NGC_READY_TIMEOUT_S}s."
        "${WITH_SUDO[@]}" docker stop --time "${SRB_NGC_STOP_TIMEOUT_S:-20}" "${NGC_CONTAINER_ID}" >/dev/null 2>&1 || :
        ngc_remove_container || :
        exit 2
    fi
    echo "[INFO] NGC readiness: ${SRB_NGC_READY_PATTERN:-Isaac Sim Full Streaming App is loaded}"
    if [[ "${SRB_NGC_STOP_AFTER_READY:-false}" = true ]]; then
        if ! "${WITH_SUDO[@]}" docker stop --time "${SRB_NGC_STOP_TIMEOUT_S:-20}" "${NGC_CONTAINER_ID}" >/dev/null; then
            echo >&2 "[ERROR] NGC container did not stop gracefully."
            ngc_remove_container || :
            exit 2
        fi
        NGC_EXIT_CODE="$("${WITH_SUDO[@]}" docker inspect --format '{{.State.ExitCode}}' "${NGC_CONTAINER_ID}" 2>/dev/null || :)"
        if [[ "${NGC_EXIT_CODE}" != 0 ]]; then
            echo >&2 "[ERROR] NGC container exit code after graceful stop was '${NGC_EXIT_CODE}', expected 0."
            ngc_remove_container || :
            exit 2
        fi
        if ! ngc_remove_container; then
            exit 2
        fi
        echo "[INFO] NGC headless launcher stopped cleanly (exit ${NGC_EXIT_CODE})."
    else
        echo "[INFO] NGC headless launcher is ready in ${CONTAINER_NAME}; join it with ./.docker/join.bash."
        NGC_KEEP_CONTAINER=true
    fi
    exit 0
fi
exec "${DOCKER_RUN_CMD[@]}"
