#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 SIMFORGE_PACKAGE_ROOT" >&2
    exit 64
fi

root="$1"
if [[ ! -d "${root}" || -L "${root}" ]]; then
    echo "SimForge package root must be a real directory: ${root}" >&2
    exit 66
fi
if find "${root}" -type l -print -quit | grep -q .; then
    echo "SimForge package root must not contain symlinks: ${root}" >&2
    exit 66
fi

old_pattern='^[[:space:]]*from isaaclab\.utils import configclass[[:space:]]*$'
new_pattern='^[[:space:]]*from isaaclab\.utils\.configclass import configclass[[:space:]]*$'
old_count="$(find "${root}" -type f -name '*.py' -exec grep -l -E "${old_pattern}" {} + | wc -l || true)"
new_count="$(find "${root}" -type f -name '*.py' -exec grep -l -E "${new_pattern}" {} + | wc -l || true)"

if ((old_count == 0 && new_count == 0)); then
    echo "SimForge configclass import contract not found under: ${root}" >&2
    exit 65
fi

find "${root}" -type f -name '*.py' -exec \
    sed -i 's/^\([[:space:]]*\)from isaaclab\.utils import configclass[[:space:]]*$/\1from isaaclab.utils.configclass import configclass/' {} +

if find "${root}" -type f -name '*.py' -exec grep -l -E "${old_pattern}" {} + | grep -q .; then
    echo "SimForge configclass patch left stale imports under: ${root}" >&2
    exit 1
fi
if ! find "${root}" -type f -name '*.py' -exec grep -l -E "${new_pattern}" {} + | grep -q .; then
    echo "SimForge configclass patch produced no accepted imports under: ${root}" >&2
    exit 1
fi

stage_old_pattern='^[[:space:]]*(from isaacsim\.core\.utils\.stage import get_current_stage|import isaacsim\.core\.utils\.stage as stage_utils)[[:space:]]*$'
stage_old_count="$(find "${root}" -type f -name '*.py' -exec grep -l -E "${stage_old_pattern}" {} + | wc -l || true)"
find "${root}" -type f -name '*.py' -exec \
    sed -i \
    -e 's/^\([[:space:]]*\)from isaacsim\.core\.utils\.stage import get_current_stage[[:space:]]*$/\1from isaaclab.sim.utils import get_current_stage/' \
    -e 's/^\([[:space:]]*\)import isaacsim\.core\.utils\.stage as stage_utils[[:space:]]*$/\1import isaaclab.sim.utils as stage_utils/' \
    {} +

if find "${root}" -type f -name '*.py' -exec grep -l -E '^[[:space:]]*(from|import) isaacsim\.core' {} + | grep -q .; then
    echo "SimForge still imports isaacsim.core (absent from Isaac Sim 6.1) under: ${root}" >&2
    find "${root}" -type f -name '*.py' -exec grep -n -E '^[[:space:]]*(from|import) isaacsim\.core' {} + >&2
    exit 1
fi
if ((stage_old_count > 0)) && ! find "${root}" -type f -name '*.py' -exec grep -l -E '^[[:space:]]*(from isaaclab\.sim\.utils import get_current_stage|import isaaclab\.sim\.utils as stage_utils)[[:space:]]*$' {} + | grep -q .; then
    echo "SimForge stage patch produced no accepted imports under: ${root}" >&2
    exit 1
fi
