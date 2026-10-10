#!/usr/bin/env bash
### Submit a job using Slurm
### Usage: submit.bash [CMD]
set -euo pipefail


SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
DOT_DOCKER_DIR="$(dirname "${SCRIPT_DIR}")"
REPOSITORY_DIR="$(dirname "${DOT_DOCKER_DIR}")"
IMAGES_DIR="${SCRIPT_DIR}/images"
PROJECT_NAME="$(basename "${REPOSITORY_DIR}")"

IMAGE_PATH="${IMAGE_PATH:-"${IMAGES_DIR}/${PROJECT_NAME}.sif"}"
JOBS_DIR="${JOBS_DIR:-"${HOME}/jobs"}"

if [ ! -f "${IMAGE_PATH}" ]; then
    echo >&2 -e "\033[1;31m[ERROR] Singularity image '${IMAGE_PATH}' is missing; set IMAGE_PATH to an existing SIF image and retry.\033[0m"
    exit 1
fi

if [ "${#}" -gt "0" ]; then
    CMD=("$@")
else
    echo >&2 -e "\033[1;31m[ERROR] No command provided.\033[0m"
    exit 1
fi

if [ ! -d "${JOBS_DIR}" ]; then
    mkdir -p "${JOBS_DIR}"
    echo -e "[INFO] Created directory ${JOBS_DIR}"
fi

JOB_PATH="$(mktemp "${JOBS_DIR%/}/job_XXXXXXXX.bash")"
JOB_NAME="${JOB_PATH##*/}"
JOB_NAME="${JOB_NAME%.bash}"
RUN_SCRIPT_PATH="${SCRIPT_DIR}/run.bash"

cleanup_submission() {
    rm -f -- "${JOB_PATH}"
}
trap cleanup_submission EXIT

ARGV_BASE64="$(printf '%s\0' "${CMD[@]}" | base64 | tr -d '\n')"
{
    printf '%s\n' '#!/bin/bash -l'
    printf '#SBATCH --job-name=%s\n' "${JOB_NAME}"
    cat <<'EOT'
#SBATCH --time=1-12:00:00
#SBATCH --qos=normal
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=168GB
#SBATCH --partition=gpu
#SBATCH --constraint volta32
#SBATCH --gpus=1
#SBATCH --gpus-per-node=1
#SBATCH --gpus-per-task=1
#SBATCH --signal=B:SIGTERM@60

set -euo pipefail
EOT
    printf 'RUN_SCRIPT=%q\n' "${RUN_SCRIPT_PATH}"
    printf 'SUBMITTED_JOB_FILE=%q\n' "${JOB_PATH}"
    cat <<'EOT'
cleanup_job() {
    rm -f -- "${SUBMITTED_JOB_FILE}"
}
trap cleanup_job EXIT

CMD=()
while IFS= read -r -d '' arg; do
    CMD+=("${arg}")
done < <(
    base64 --decode <<'SRB_ARGV_BASE64'
EOT
    printf '%s\n' "${ARGV_BASE64}"
    cat <<'EOT'
SRB_ARGV_BASE64
)

source "${RUN_SCRIPT}"
run_container "${CMD[@]}"
EOT
} >"${JOB_PATH}"
chmod 0700 "${JOB_PATH}"

SBATCH_CMD=(
    sbatch "${JOB_PATH}"
)
printf "\033[1;90m[TRACE] "
printf "%q " "${SBATCH_CMD[@]}"
printf "\033[0m\n"
if setsid --wait "${SBATCH_CMD[@]}"; then
    trap - EXIT
else
    status=$?
    exit "${status}"
fi
