#!/usr/bin/env bash
### Note: These assets are for Isaac Sim 6.1.
### Usage: download_isaacsim_assets.bash [destination_directory]
set -Eeuo pipefail

ISAACSIM_ASSETS_VERSION="${ISAACSIM_ASSETS_VERSION:-"6.1.0"}"
SRC_URLS=(
    "https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-${ISAACSIM_ASSETS_VERSION}.001.zip"
    "https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-${ISAACSIM_ASSETS_VERSION}.002.zip"
    "https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-${ISAACSIM_ASSETS_VERSION}.003.zip"
    "https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-${ISAACSIM_ASSETS_VERSION}.004.zip"
    "https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-${ISAACSIM_ASSETS_VERSION}.005.zip"
)
DEST_DIR="${1:-"${DEST_DIR:-"${HOME:-/tmp}/isaac-sim-assets"}"}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OWNED_DIR_TOOL="${SCRIPT_DIR}/owned_directory.py"
INSTALL_OWNER="srb.installer.isaacsim-assets"
ARCHIVE_KEYS=()
for source_url in "${SRC_URLS[@]}"; do
    ARCHIVE_KEYS+=("$(basename -- "${source_url}")")
done
AGGREGATE_TEMP_PREFIX=".srb-isaacsim-assets-aggregate-"
PART_TEMP_PREFIX=".srb-isaacsim-assets-part-"

preflight_refuse() {
    echo >&2 -e "\033[1;31m[ERROR] $*\033[0m"
    exit 1
}

if [[ ! "${ISAACSIM_ASSETS_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    preflight_refuse "ISAACSIM_ASSETS_VERSION must be numeric X.Y.Z: ${ISAACSIM_ASSETS_VERSION}"
fi
ASSET_VERSION_DIRECTORY="${ISAACSIM_ASSETS_VERSION%.*}"

assert_no_symlink_components() {
    local raw_path="$1" absolute_path component prefix="/"
    if [[ "${raw_path}" == /* ]]; then
        absolute_path="${raw_path}"
    else
        absolute_path="${PWD}/${raw_path}"
    fi
    local -a components=()
    IFS='/' read -r -a components <<<"${absolute_path}"
    for component in "${components[@]}"; do
        case "${component}" in
            ""|.) continue ;;
            ..)
                prefix="$(dirname -- "${prefix}")"
                continue
                ;;
        esac
        if [[ "${prefix}" == "/" ]]; then
            prefix="/${component}"
        else
            prefix="${prefix}/${component}"
        fi
        if [[ -L "${prefix}" ]]; then
            preflight_refuse "Destination path contains a symlink: ${prefix}"
        fi
    done
}

assert_no_symlink_components "${DEST_DIR}"
if [[ -e "${DEST_DIR}" || -L "${DEST_DIR}" ]]; then
    preflight_refuse "Destination already exists; choose a fresh side-by-side asset path: ${DEST_DIR}"
fi
DEST_CANONICAL="$(realpath -m -- "${DEST_DIR}")"
if [[ "${DEST_CANONICAL}" == "/" ]]; then
    preflight_refuse "Refusing filesystem-root asset destination: ${DEST_DIR}"
fi
for protected_path in "${HOME:-}" "${SCRIPT_DIR}/../.." "${PWD}"; do
    [[ -n "${protected_path}" ]] || continue
    protected_path="$(realpath -m -- "${protected_path}")"
    if [[ "${DEST_CANONICAL}" == "${protected_path}" || "${protected_path}" == "${DEST_CANONICAL}"/* ]]; then
        preflight_refuse "Destination is a protected root or ancestor: ${DEST_DIR}"
    fi
done
DEST_DIR="${DEST_CANONICAL}"

owned_operation() {
    python3 "${OWNED_DIR_TOOL}" "$@"
}

refuse() {
    echo >&2 -e "\033[1;31m[ERROR] $*\033[0m"
    echo >&2 -e "\033[1;31m[ERROR] Refusing to extract or publish. Nothing was written to ${DEST_DIR}\033[0m"
    exit 1
}

write_resume_state() {
    python3 - "${RESUME_STATE}" "${DEST_DIR}" "${ISAACSIM_ASSETS_VERSION}" <<'PY'
import json
import os
import sys

path, destination, version = sys.argv[1:]
payload = {
    "destination": destination,
    "schema": "srb-isaacsim-assets-resume/1",
    "version": version,
}
flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
try:
    descriptor = os.open(path, flags, 0o600)
except OSError as error:
    print(f"could not create resume state: {error}", file=sys.stderr)
    raise SystemExit(1)
with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
    json.dump(payload, stream, sort_keys=True, separators=(",", ":"))
    stream.write("\n")
    stream.flush()
    os.fsync(stream.fileno())
PY
}

validate_resume_state() {
    python3 - "${RESUME_STATE}" "${DEST_DIR}" "${ISAACSIM_ASSETS_VERSION}" <<'PY'
import json
import os
import sys

path, destination, version = sys.argv[1:]
if os.path.islink(path) or not os.path.isfile(path):
    print("resume state must be a regular file", file=sys.stderr)
    raise SystemExit(1)
try:
    with open(path, encoding="utf-8") as stream:
        payload = json.load(stream)
except (OSError, UnicodeError, json.JSONDecodeError) as error:
    print(f"resume state is invalid: {error}", file=sys.stderr)
    raise SystemExit(1)
if (
    type(payload) is not dict
    or set(payload) != {"destination", "schema", "version"}
    or payload["schema"] != "srb-isaacsim-assets-resume/1"
    or payload["destination"] != destination
    or payload["version"] != version
):
    print("resume state destination/version binding mismatch", file=sys.stderr)
    raise SystemExit(1)
PY
}

validate_resume_scope() {
    local entry name allowed key
    while IFS= read -r -d '' entry; do
        name="${entry##*/}"
        allowed=0
        case "${name}" in
            .srb-owned-directory.json|.srb-isaacsim-assets-resume.json|staging|"${ARCHIVE_PATH##*/}"|"${AGGREGATE_TEMP_PREFIX}"*|"${PART_TEMP_PREFIX}"*)
                allowed=1
                ;;
        esac
        for key in "${ARCHIVE_KEYS[@]}"; do
            if [[ "${name}" == "${key}" ]]; then
                allowed=1
                break
            fi
        done
        if ((allowed != 1)); then
            echo "unexpected entry in resumable work directory: ${name}" >&2
            return 1
        fi
    done < <(find -- "${WORK_DIR}" -mindepth 1 -maxdepth 1 -print0)
}

PUBLISHER_MD5=(
    "9d5520d6b179b1ee10c78b37528d65cb"
    "8e7be70261ab6c226077fa8e458f90c9"
    "b13723342f072c49a5584f7b821cd019"
    "11b76b9cc15d546236fc9fc4cc50153c"
    "562fe8ec4e7a23b3fd2a2c5c5c00fd5b"
)
if [[ "${ISAACSIM_ASSETS_VERSION}" != "6.1.0" ]]; then
    PUBLISHER_MD5=("" "" "" "" "")
fi
ISAACSIM_ASSETS_MD5=(
    "${ISAACSIM_ASSETS_MD5_1:-${PUBLISHER_MD5[0]}}"
    "${ISAACSIM_ASSETS_MD5_2:-${PUBLISHER_MD5[1]}}"
    "${ISAACSIM_ASSETS_MD5_3:-${PUBLISHER_MD5[2]}}"
    "${ISAACSIM_ASSETS_MD5_4:-${PUBLISHER_MD5[3]}}"
    "${ISAACSIM_ASSETS_MD5_5:-${PUBLISHER_MD5[4]}}"
)
ISAACSIM_ASSETS_SHA256=(
    "${ISAACSIM_ASSETS_SHA256_1:-}"
    "${ISAACSIM_ASSETS_SHA256_2:-}"
    "${ISAACSIM_ASSETS_SHA256_3:-}"
    "${ISAACSIM_ASSETS_SHA256_4:-}"
    "${ISAACSIM_ASSETS_SHA256_5:-}"
)
PIN_FILE="${ISAACSIM_ASSETS_PIN_FILE:-"${SCRIPT_DIR}/../../docs/data/isaacsim_assets_pins.sha256"}"

for i in "${!SRC_URLS[@]}"; do
    echo "[INFO] Source URL $((i + 1)): ${SRC_URLS[i]}"
done
echo "[INFO] Destination path: ${DEST_DIR}"
echo "[INFO] Digest pin file: ${PIN_FILE}"

DEST_PARENT="$(dirname -- "${DEST_DIR}")"
if [[ ! -d "${DEST_PARENT}" ]]; then
    preflight_refuse "Destination parent does not exist: ${DEST_PARENT}"
fi
RESUME_DIR="${ISAACSIM_ASSETS_RESUME_DIR:-}"
RESUMING=0
if [[ -n "${RESUME_DIR}" ]]; then
    [[ "${RESUME_DIR}" == /* ]] ||
    preflight_refuse "ISAACSIM_ASSETS_RESUME_DIR must be an absolute path"
    WORK_DIR="${RESUME_DIR}"
    RESUMING=1
else
    WORK_DIR="$(mktemp -d "${DEST_PARENT}/.srb-isaacsim-assets-XXXXXXXXXX")"
fi
ARCHIVE_PATH="${WORK_DIR}/isaac-sim-assets-complete-${ISAACSIM_ASSETS_VERSION}.zip"
STAGING_DIR="${WORK_DIR}/staging"
STAGED_ROOT="${STAGING_DIR}/isaac-sim-assets-${ISAACSIM_ASSETS_VERSION}"
PUBLISH_ROOT="${STAGED_ROOT}/Assets/Isaac/${ASSET_VERSION_DIRECTORY}"
RESUME_STATE="${WORK_DIR}/.srb-isaacsim-assets-resume.json"
PART_PATHS=()
TOFU_KEYS=()
TOFU_DIGESTS=()

if ((RESUMING == 1)); then
    if ! owned_operation validate --path "${WORK_DIR}" --owner "${INSTALL_OWNER}"; then
        preflight_refuse "Resume directory is not a validated marker-owned stage: ${WORK_DIR}"
    fi
    if ! validate_resume_state; then
        preflight_refuse "Resume directory destination/version binding is invalid: ${WORK_DIR}"
    fi
    if ! validate_resume_scope; then
        preflight_refuse "Resume directory contains entries outside the installer scope: ${WORK_DIR}"
    fi
else
    if ! owned_operation mark-created --path "${WORK_DIR}" --owner "${INSTALL_OWNER}"; then
        preflight_refuse "Could not mark fresh installer work directory as owned: ${WORK_DIR}"
    fi
fi

cleanup() {
    local status=$?
    trap - EXIT
    if ((status == 0)); then
        if ! owned_operation remove --path "${WORK_DIR}" --owner "${INSTALL_OWNER}"; then
            echo >&2 "[ERROR] Could not remove validated installer work directory: ${WORK_DIR}"
            status=1
        fi
    else
        echo >&2 "[ERROR] Work directory retained at ${WORK_DIR}; inspect it and retry the installer"
    fi
    exit "${status}"
}
trap cleanup EXIT

if ((RESUMING == 0)); then
    if ! write_resume_state; then
        refuse "Could not write the fresh work-directory destination/version binding"
    fi
fi

is_sha256() {
    [[ "${#1}" -eq 64 && "$1" =~ ^[0-9a-f]+$ ]]
}

is_md5() {
    [[ "${#1}" -eq 32 && "$1" =~ ^[0-9a-f]+$ ]]
}

pin_lookup() {
    [[ -f "${PIN_FILE}" ]] || return 0
    awk -v key="$1" '
        /^[[:space:]]*#/ { next }
        NF == 0 { next }
        {
            for (field = 1; field <= NF; field++) {
                if ($field == key) {
                    if (NF == 2 && field == 2 && length($1) == 64 && $1 ~ /^[0-9a-f]+$/)
                        print $1
                    else
                        print "malformed"
                    exit
                }
            }
        }
    ' "${PIN_FILE}"
}

pin_record() {
    local key="$1" digest="$2" pin_dir tmp
    pin_dir="$(dirname -- "${PIN_FILE}")" || return 1
    mkdir -p -- "${pin_dir}" || return 1
    tmp="$(mktemp -- "${PIN_FILE}.XXXXXXXX")" || return 1
    if [[ -s "${PIN_FILE}" ]]; then
        cat -- "${PIN_FILE}" >"${tmp}" || return 1
        # Keep the appended pin on its own line.
        [[ -z "$(tail -c 1 -- "${tmp}")" ]] || printf '\n' >>"${tmp}" || return 1
    else
        printf '%s\n' \
            "# Trust-on-first-use SHA-256 pins for the Isaac Sim asset archives." \
            "# Written by .docker/install/download_isaacsim_assets.bash; commit it so the" \
            "# pins are reviewable." \
            "#" \
            "# Format: <sha256>  <versioned archive file name>" \
            "#" \
            "# NVIDIA publishes MD5 for these fragments, but not a SHA-256" \
            "# provenance record. MD5 is retained as the publisher check; a local" \
            "# pin freezes the bytes for future runs. It does NOT authenticate this" \
            "# first download. Delete a line only if you intend to re-establish" \
            "# trust from scratch." \
            >"${tmp}" || return 1
    fi
    printf '%s  %s\n' "${digest}" "${key}" >>"${tmp}" || return 1
    chmod 0644 -- "${tmp}" || return 1
    mv -f -- "${tmp}" "${PIN_FILE}" || return 1
    if [[ -n "${SUDO_UID:-}" && -n "${SUDO_GID:-}" ]]; then
        if ! chown "${SUDO_UID}:${SUDO_GID}" -- "${PIN_FILE}" 2>/dev/null; then
            printf >&2 '[ERROR] Could not restore ownership of pin file %s to %s:%s. Run without sudo or make the file writable by that user, then retry.\n' \
                "${PIN_FILE}" "${SUDO_UID}" "${SUDO_GID}"
            return 1
        fi
    fi
    return 0
}

for i in "${!SRC_URLS[@]}"; do
    ARCHIVE_KEY="$(basename -- "${SRC_URLS[i]}")"
    PART_PATH="${WORK_DIR}/${ARCHIVE_KEY}"
    PART_PATHS+=("${PART_PATH}")

    if [[ -e "${PART_PATH}" || -L "${PART_PATH}" ]]; then
        if [[ ! -f "${PART_PATH}" || -L "${PART_PATH}" ]]; then
            refuse "Resume part is not a regular non-symlink file: ${PART_PATH}"
        fi
        echo "[INFO] Reusing resumable archive part $((i + 1)) at ${PART_PATH}"
    else
        PART_TEMP_PATH="$(mktemp -- "${WORK_DIR}/${PART_TEMP_PREFIX}XXXXXXXXXX")" ||
        refuse "Could not create a private temporary file for ${ARCHIVE_KEY}"
        echo "[INFO] Downloading archive part $((i + 1)) to ${PART_TEMP_PATH}"
        if ! curl --proto "=https" --tlsv1.2 -sSfL "${SRC_URLS[i]}" -o "${PART_TEMP_PATH}"; then
            refuse "Could not download ${ARCHIVE_KEY}"
        fi
        if ! mv --no-clobber --no-target-directory -- "${PART_TEMP_PATH}" "${PART_PATH}"; then
            refuse "Could not publish downloaded part without replacing a resumable file: ${ARCHIVE_KEY}"
        fi
    fi

    echo "[INFO] Hashing ${ARCHIVE_KEY}"
    ACTUAL_SHA256="$(sha256sum -- "${PART_PATH}" | cut -d' ' -f1)" ||
    refuse "Could not compute a SHA-256 for ${PART_PATH}"
    is_sha256 "${ACTUAL_SHA256}" ||
    refuse "Could not compute a SHA-256 for ${PART_PATH}"

    ACTUAL_MD5="$(md5sum -- "${PART_PATH}" | cut -d' ' -f1)" ||
    refuse "Could not compute the publisher MD5 for ${PART_PATH}"
    is_md5 "${ACTUAL_MD5}" ||
    refuse "Could not compute the publisher MD5 for ${PART_PATH}"
    EXPECTED_MD5="${ISAACSIM_ASSETS_MD5[i]}"
    is_md5 "${EXPECTED_MD5}" ||
    refuse "ISAACSIM_ASSETS_MD5_$((i + 1)) is not a lowercase MD5: ${EXPECTED_MD5}"
    if [[ "${ACTUAL_MD5}" != "${EXPECTED_MD5}" ]]; then
        if [[ "${ISAACSIM_ASSETS_VERSION}" == "6.1.0" ]]; then
            refuse "$({
                printf '%s\n' \
                    "Publisher MD5 mismatch for ${ARCHIVE_KEY}:" \
                    "  expected ${EXPECTED_MD5}" \
                    "  actual   ${ACTUAL_MD5}"
            })"
        fi
        refuse "$({
            printf '%s\n' \
                "MD5 mismatch for ${ARCHIVE_KEY}. Supply the publisher checksum for ${ISAACSIM_ASSETS_VERSION}:" \
                "  expected ${EXPECTED_MD5}" \
                "  actual   ${ACTUAL_MD5}"
        })"
    fi
    echo "[INFO] Publisher MD5 verified: ${ACTUAL_MD5}"

    SUPPLIED_SHA256="${ISAACSIM_ASSETS_SHA256[i]}"
    RECORDED_SHA256="$(pin_lookup "${ARCHIVE_KEY}")"
    if [[ -n "${RECORDED_SHA256}" ]] && ! is_sha256 "${RECORDED_SHA256}"; then
        refuse "$({
            printf '%s\n' \
                "The pin record for ${ARCHIVE_KEY} in ${PIN_FILE} is malformed." \
                "Repair it, or delete the line to re-establish trust deliberately."
        })"
    fi

    if [[ -n "${SUPPLIED_SHA256}" ]]; then
        is_sha256 "${SUPPLIED_SHA256}" ||
        refuse "ISAACSIM_ASSETS_SHA256_$((i + 1)) is not a lowercase SHA-256: ${SUPPLIED_SHA256}"
        if [[ -n "${RECORDED_SHA256}" && "${RECORDED_SHA256}" != "${SUPPLIED_SHA256}" ]]; then
            refuse "$({
                printf '%s\n' \
                    "Digest conflict for ${ARCHIVE_KEY}:" \
                    "  ISAACSIM_ASSETS_SHA256_$((i + 1)) = ${SUPPLIED_SHA256}" \
                    "  recorded pin                     = ${RECORDED_SHA256} (${PIN_FILE})" \
                    "Resolve it deliberately: correct the variable, or delete the pin line to re-trust."
            })"
        fi
        EXPECTED_SHA256="${SUPPLIED_SHA256}"
        EXPECTED_SOURCE="ISAACSIM_ASSETS_SHA256_$((i + 1))"
    else
        EXPECTED_SHA256="${RECORDED_SHA256}"
        EXPECTED_SOURCE="recorded pin in ${PIN_FILE}"
    fi

    if [[ -n "${EXPECTED_SHA256}" ]]; then
        echo "[INFO] Verifying ${ARCHIVE_KEY} against ${EXPECTED_SOURCE}"
        if [[ "${ACTUAL_SHA256}" != "${EXPECTED_SHA256}" ]]; then
            refuse "$({
                printf '%s\n' \
                    "SHA-256 mismatch for ${ARCHIVE_KEY}:" \
                    "  expected ${EXPECTED_SHA256} (${EXPECTED_SOURCE})" \
                    "  actual   ${ACTUAL_SHA256}"
            })"
        fi
        echo "[INFO] Digest verified: ${ACTUAL_SHA256}"
    else
        echo -e "\033[1;33m[TOFU] No SHA-256 pin for ${ARCHIVE_KEY}; recording this download after aggregate validation.\033[0m"
        echo -e "\033[1;33m[TOFU] This freezes the bytes for future runs. It does NOT authenticate this first download.\033[0m"
        TOFU_KEYS+=("${ARCHIVE_KEY}")
        TOFU_DIGESTS+=("${ACTUAL_SHA256}")
    fi
done

echo "[INFO] Concatenating ${#PART_PATHS[@]} ordered archive parts"
if [[ -e "${ARCHIVE_PATH}" || -L "${ARCHIVE_PATH}" ]]; then
    if [[ ! -f "${ARCHIVE_PATH}" || -L "${ARCHIVE_PATH}" ]]; then
        refuse "Aggregate archive path is not a regular non-symlink file: ${ARCHIVE_PATH}"
    fi
fi
AGGREGATE_TEMP_PATH="$(mktemp -- "${WORK_DIR}/${AGGREGATE_TEMP_PREFIX}XXXXXXXXXX")" ||
refuse "Could not create a private aggregate archive temporary file"
if ! cat -- "${PART_PATHS[@]}" >"${AGGREGATE_TEMP_PATH}"; then
    refuse "Could not concatenate the ordered archive parts"
fi

echo "[INFO] Validating aggregate archive ${AGGREGATE_TEMP_PATH}"
if ! unzip -tq "${AGGREGATE_TEMP_PATH}" >/dev/null; then
    refuse "Aggregate archive validation failed; the ordered parts may be truncated or corrupt"
fi
if ! mv -f --no-target-directory -- "${AGGREGATE_TEMP_PATH}" "${ARCHIVE_PATH}"; then
    refuse "Could not publish the validated aggregate archive without following a link"
fi

if [[ -e "${STAGING_DIR}" || -L "${STAGING_DIR}" ]]; then
    if [[ ! -d "${STAGING_DIR}" || -L "${STAGING_DIR}" ]]; then
        refuse "Staging path is not a regular directory: ${STAGING_DIR}"
    fi
    if ! owned_operation validate --path "${STAGING_DIR}" --owner "${INSTALL_OWNER}"; then
        refuse "Existing staging path is not marker-owned by this installer: ${STAGING_DIR}"
    fi
    if ! owned_operation remove --path "${STAGING_DIR}" --owner "${INSTALL_OWNER}"; then
        refuse "Could not clear validated resumable staging path: ${STAGING_DIR}"
    fi
fi
mkdir -p -- "${STAGING_DIR}" || refuse "Could not create extraction staging directory ${STAGING_DIR}"
if ! owned_operation mark-created --path "${STAGING_DIR}" --owner "${INSTALL_OWNER}"; then
    refuse "Could not mark extraction staging directory as owned: ${STAGING_DIR}"
fi
mkdir -p -- "${STAGED_ROOT}" || refuse "Could not create extraction staging root ${STAGED_ROOT}"
echo "[INFO] Extracting aggregate archive to ${STAGED_ROOT}"
if ! unzip -q "${ARCHIVE_PATH}" -d "${STAGED_ROOT}"; then
    refuse "Could not extract the validated aggregate archive"
fi
for required_dir in \
    "${STAGED_ROOT}/Assets" \
    "${STAGED_ROOT}/Assets/Isaac" \
    "${PUBLISH_ROOT}" \
    "${PUBLISH_ROOT}/Isaac" \
    "${PUBLISH_ROOT}/NVIDIA"; do
    if [[ ! -d "${required_dir}" || -L "${required_dir}" ]]; then
        refuse "Extracted archive is missing a required real asset directory: ${required_dir}"
    fi
done

for i in "${!TOFU_KEYS[@]}"; do
    if ! pin_record "${TOFU_KEYS[i]}" "${TOFU_DIGESTS[i]}"; then
        refuse "$({
            printf '%s\n' \
                "Could not record the digest pin at ${PIN_FILE}." \
                "Extraction would leave the archive permanently unverifiable, so it is refused." \
                "Point ISAACSIM_ASSETS_PIN_FILE at a writable path and re-run."
        })"
    fi
    if [[ "$(pin_lookup "${TOFU_KEYS[i]}")" != "${TOFU_DIGESTS[i]}" ]]; then
        refuse "Digest pin for ${TOFU_KEYS[i]} did not persist to ${PIN_FILE}"
    fi
    echo "[TOFU] Pinned ${TOFU_DIGESTS[i]}  ${TOFU_KEYS[i]} (commit ${PIN_FILE})"
done

if [[ -e "${DEST_DIR}" || -L "${DEST_DIR}" ]]; then
    refuse "Destination appeared during bootstrap; refusing no-clobber publication: ${DEST_DIR}"
fi
STAGED_ID="$(stat -c '%d:%i' -- "${PUBLISH_ROOT}")" ||
refuse "Could not identify the staged asset root before publication"
if ! mv --no-clobber --no-target-directory -- "${PUBLISH_ROOT}" "${DEST_DIR}"; then
    refuse "Could not publish the staged asset root without replacing a destination"
fi
if [[ ! -d "${DEST_DIR}" || -L "${DEST_DIR}" ]]; then
    refuse "Destination publication did not create a real directory: ${DEST_DIR}"
fi
PUBLISHED_ID="$(stat -c '%d:%i' -- "${DEST_DIR}")" ||
refuse "Could not identify the published asset root"
if [[ "${PUBLISHED_ID}" != "${STAGED_ID}" ]]; then
    refuse "Destination changed during no-clobber publication: ${DEST_DIR}"
fi

echo "[INFO] Isaac Sim assets have been downloaded to ${DEST_DIR}"
