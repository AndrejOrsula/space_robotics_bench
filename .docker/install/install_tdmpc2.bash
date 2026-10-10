#!/usr/bin/env bash
### Usage: install_tdmpc2.bash [destination_directory]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OWNED_DIR_TOOL="${SCRIPT_DIR}/owned_directory.py"
INSTALL_OWNER="srb.installer.tdmpc2"
PIN_MANIFEST="${REPO_ROOT}/srb/integrations/tdmpc2/pin.toml"
SRC_REPOSITORY="${SRC_REPOSITORY:-"https://github.com/nicklashansen/tdmpc2.git"}"
SRC_REPOSITORY_DISPLAY="$(printf '%s' "${SRC_REPOSITORY}" | sed -E 's#^([A-Za-z][A-Za-z0-9+.-]*://)[^/@]+@#\1***@#')"
SRC_BRANCH="${SRC_BRANCH:-"main"}"
DEST_DIR="${1:-"${DEST_DIR:-"$HOME/tdmpc2"}"}"
DEST_DIR="$(realpath -m -- "${DEST_DIR}")"
DEST_MARKED=false
INSTALL_PUBLISHED=false
SOURCE_STAGE=""
PATCH_COMMITTER_NAME="SRB Maintainers"
PATCH_COMMITTER_EMAIL="srb-maintainers@localhost"

cleanup() {
local exit_status="${1:-0}"
local cleanup_failed=0
set +e
if [[ "${DEST_MARKED}" == true && "${INSTALL_PUBLISHED}" != true ]]; then
    python3 "${OWNED_DIR_TOOL}" remove --path "${DEST_DIR}" --owner "${INSTALL_OWNER}" || cleanup_failed=1
fi
if [[ -n "${SOURCE_STAGE}" && -d "${SOURCE_STAGE}" ]]; then
    rm -rf -- "${SOURCE_STAGE}" || cleanup_failed=1
fi
set -e
if (( exit_status == 0 && cleanup_failed != 0 )); then
    exit_status=1
fi
return "${exit_status}"
}
trap 'exit_status=$?; cleanup "${exit_status}"; exit "${exit_status}"' EXIT

if ! MANIFEST_OUTPUT="$({
python3 - "${PIN_MANIFEST}" <<'PY'
import json
import re
import sys
from pathlib import Path, PurePosixPath, PureWindowsPath

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10 installer hosts
    tomllib = None

manifest_path = Path(sys.argv[1])
expected_keys = {
    "schema_version",
    "base_sha",
    "backport_sha",
    "qualified_sha",
    "rejected_dependency_pin_sha",
    "patch_path",
}
sha_pattern = re.compile(r"[0-9a-f]{40}")


def load_manifest(path):
    if tomllib is not None:
        with path.open("rb") as stream:
            return tomllib.load(stream)

    # Python 3.10 lacks tomllib. Parse only the manifest's JSON-compatible
    # scalar assignments; do not evaluate shell or Python code.
    assignment_pattern = re.compile(
        r"(?P<key>[a-z][a-z0-9_]*)\s*=\s*(?P<value>.+)"
    )
    manifest = {}
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        match = assignment_pattern.fullmatch(stripped)
        if match is None:
            raise ValueError(f"unsupported manifest syntax on line {line_number}")
        key = match.group("key")
        if key in manifest:
            raise ValueError(f"duplicate manifest key {key!r}")
        try:
            manifest[key] = json.loads(match.group("value"))
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"invalid manifest scalar on line {line_number}"
            ) from exc
    return manifest


try:
    manifest = load_manifest(manifest_path)
except (OSError, UnicodeError, ValueError) as exc:
    print(f"invalid TD-MPC2 pin manifest: {exc}", file=sys.stderr)
    raise SystemExit(1) from exc

if type(manifest) is not dict or set(manifest) != expected_keys:
    print("invalid TD-MPC2 pin manifest schema or fields", file=sys.stderr)
    raise SystemExit(1)
if type(manifest["schema_version"]) is not int or manifest["schema_version"] != 1:
    print("pin manifest schema_version must be integer 1", file=sys.stderr)
    raise SystemExit(1)
for field in (
    "base_sha",
    "backport_sha",
    "qualified_sha",
    "rejected_dependency_pin_sha",
):
    value = manifest[field]
    if type(value) is not str or sha_pattern.fullmatch(value) is None:
        print(f"pin manifest {field} must be a lowercase 40-hex SHA", file=sys.stderr)
        raise SystemExit(1)

patch_path = manifest["patch_path"]
if type(patch_path) is not str or not patch_path:
    print("pin manifest patch_path must be a non-empty string", file=sys.stderr)
    raise SystemExit(1)
if (
    patch_path.startswith(("/", "\\"))
    or "\\" in patch_path
    or "\x00" in patch_path
    or any(ord(char) < 0x20 or ord(char) == 0x7F for char in patch_path)
):
    print("pin manifest patch_path is not a safe repository-relative path", file=sys.stderr)
    raise SystemExit(1)
if any(part in {"", ".", ".."} for part in patch_path.split("/")):
    print("pin manifest patch_path contains an unsafe path component", file=sys.stderr)
    raise SystemExit(1)
windows_path = PureWindowsPath(patch_path)
if PurePosixPath(patch_path).is_absolute() or windows_path.drive or windows_path.root:
    print("pin manifest patch_path must be repository-relative", file=sys.stderr)
    raise SystemExit(1)

print(manifest["schema_version"])
print(manifest["base_sha"])
print(manifest["backport_sha"])
print(manifest["qualified_sha"])
print(manifest["rejected_dependency_pin_sha"])
print(patch_path)
PY
})"; then
echo "[ERROR] Could not parse ${PIN_MANIFEST}" >&2
exit 1
fi
mapfile -t PIN_VALUES <<< "${MANIFEST_OUTPUT}"
if [[ "${#PIN_VALUES[@]}" -ne 6 ]]; then
echo "[ERROR] Pin manifest parser returned an invalid record" >&2
exit 1
fi
BASE_SHA="${PIN_VALUES[1]}"
QUALIFIED_SHA="${PIN_VALUES[3]}"
PATCH_PATH="${PIN_VALUES[5]}"

PATCH_FILE="${REPO_ROOT}/${PATCH_PATH}"
if [[ ! -f "${PATCH_FILE}" || -L "${PATCH_FILE}" ]]; then
echo "[ERROR] Manifest patch is not a regular non-symlink file: ${PATCH_FILE}" >&2
exit 1
fi
PATCH_HEADER=""
if ! IFS= read -r PATCH_HEADER < "${PATCH_FILE}"; then
echo "[ERROR] Manifest patch has no From header: ${PATCH_FILE}" >&2
exit 1
fi
if [[ ! "${PATCH_HEADER}" =~ ^From[[:space:]]([0-9a-f]{40})[[:space:]] ]]; then
echo "[ERROR] Manifest patch has an invalid From header: ${PATCH_FILE}" >&2
exit 1
fi
if [[ "${BASH_REMATCH[1]}" != "${QUALIFIED_SHA}" ]]; then
echo "[ERROR] Manifest patch From SHA does not match qualified_sha" >&2
exit 1
fi

abort_git_am() {
if [[ -d "${DEST_DIR}/.git" ]]; then
    git -C "${DEST_DIR}" am --abort >/dev/null 2>&1 || true
fi
}

abort_after_patch_failure() {
local status="$1"
local message="$2"
echo "${message}" >&2
abort_git_am
exit "${status}"
}

echo "[INFO] Source repository: ${SRC_REPOSITORY_DISPLAY}"
echo "[INFO] Source branch: ${SRC_BRANCH}"
echo "[INFO] Source commit: ${BASE_SHA}"
echo "[INFO] Destination path: ${DEST_DIR}"

if [[ -d "${DEST_DIR}" ]]; then
echo -en "\033[1;33m[WARNING] Destination directory already exists, overwrite? [y/N]\033[0m "
read -r
if [[ ! "${REPLY}" =~ ^[Yy]$ ]]; then
    echo "[INFO] Exiting"
    exit 0
fi
python3 "${OWNED_DIR_TOOL}" remove --path "${DEST_DIR}" --owner "${INSTALL_OWNER}"
fi

DEST_PARENT="$(dirname -- "${DEST_DIR}")"
mkdir -p -- "${DEST_PARENT}"
mkdir -- "${DEST_DIR}"
if python3 "${OWNED_DIR_TOOL}" mark-created --path "${DEST_DIR}" --owner "${INSTALL_OWNER}"; then
DEST_MARKED=true
else
status=$?
if ! rmdir -- "${DEST_DIR}"; then
    echo "[WARNING] Could not remove empty unmarked destination: ${DEST_DIR}" >&2
fi
echo "[ERROR] Could not mark owned TD-MPC2 destination: ${DEST_DIR}" >&2
exit "${status}"
fi

SOURCE_STAGE="$(mktemp -d -- "${DEST_PARENT}/.tdmpc2-clone.XXXXXXXX")"
echo "[INFO] Cloning repository to ${DEST_DIR}"
git clone --branch "${SRC_BRANCH}" "${SRC_REPOSITORY}" "${SOURCE_STAGE}/checkout"
shopt -s dotglob nullglob
mv -- "${SOURCE_STAGE}/checkout/"* "${DEST_DIR}/"
shopt -u dotglob nullglob
rm -rf -- "${SOURCE_STAGE}"
SOURCE_STAGE=""
printf '%s\n' "/.srb-owned-directory.json" >> "${DEST_DIR}/.git/info/exclude"
echo "[INFO] Resetting to commit ${BASE_SHA}"
if git -C "${DEST_DIR}" reset --hard "${BASE_SHA}"; then
:
else
status=$?
echo "[ERROR] Could not reset TD-MPC2 checkout to ${BASE_SHA}" >&2
exit "${status}"
fi
if BASE_HEAD="$(git -C "${DEST_DIR}" rev-parse HEAD)"; then
:
else
status=$?
abort_after_patch_failure "${status}" "[ERROR] Could not read TD-MPC2 base HEAD"
fi
if [[ "${BASE_HEAD}" != "${BASE_SHA}" ]]; then
abort_after_patch_failure 1 \
    "[ERROR] Checkout HEAD ${BASE_HEAD} does not match manifest base_sha ${BASE_SHA}"
fi

echo "[INFO] Applying qualified TD-MPC2 source correction $(basename "${PATCH_FILE}")"
if GIT_COMMITTER_NAME="${PATCH_COMMITTER_NAME}" GIT_COMMITTER_EMAIL="${PATCH_COMMITTER_EMAIL}" git -C "${DEST_DIR}" -c commit.gpgsign=false am --ignore-whitespace --committer-date-is-author-date "${PATCH_FILE}"; then
:
else
status=$?
abort_after_patch_failure "${status}" "[ERROR] Could not apply manifest patch"
fi

if FINAL_HEAD="$(git -C "${DEST_DIR}" rev-parse HEAD)"; then
:
else
status=$?
abort_after_patch_failure "${status}" "[ERROR] Could not read final TD-MPC2 HEAD"
fi
echo "[INFO] Pinned TD-MPC2 revision: ${FINAL_HEAD}"
if [[ "${FINAL_HEAD}" != "${QUALIFIED_SHA}" ]]; then
abort_after_patch_failure 1 \
    "[ERROR] Final HEAD ${FINAL_HEAD} does not match manifest qualified_sha ${QUALIFIED_SHA}"
fi
echo "[INFO] TD-MPC2 source has been cloned to ${DEST_DIR}"
echo "[INFO] Note: TD-MPC2 is not pip-installed. Its Python dependencies (tensordict, torchrl, etc.) are installed via pyproject.toml."
echo "[INFO] You must set the TDMPC2_PATH environment variable to use TD-MPC2:"
echo "[INFO]"
echo "[INFO]   # Bash (~/.bashrc)"
echo "[INFO]   export TDMPC2_PATH='${DEST_DIR}/tdmpc2'"
echo "[INFO]"
echo "[INFO]   # Zsh (~/.zshrc)"
echo "[INFO]   export TDMPC2_PATH='${DEST_DIR}/tdmpc2'"
echo "[INFO]"
echo "[INFO]   # Fish"
echo "[INFO]   set -Ux TDMPC2_PATH '${DEST_DIR}/tdmpc2'"
INSTALL_PUBLISHED=true
