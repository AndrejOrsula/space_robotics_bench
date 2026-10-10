#!/usr/bin/env python3
"""Print dependencies declared by Isaac Lab 3.0's root pyproject.

Its ``source/`` extensions declare no dependencies. SRB installs them editable
into its locked environment, so it installs root dependencies separately
(including ``warp`` and ``prettytable``).

Usage after installing the extensions::

    python .docker/helpers/isaaclab_root_requirements.py requirements <root pyproject>
    python .docker/helpers/isaaclab_root_requirements.py overrides <root pyproject>

``requirements`` prints root requirements for ``uv pip install -r``;
``overrides`` prints uv overrides and never-matching masks for NVIDIA-internal
packages, for ``uv pip install --overrides``.

Requirements omit editable ``isaaclab*`` workspace members, internal
``omniverseclient``/``isaacteleop``, and the viewer-only ``open3d``,
``imgui-bundle``, and ``PyOpenGL-accelerate`` packages. Keep the ``newton``
runtime included: PhysX articulation imports it at module load. Omit
``transformers`` (Isaac Lab 5.x conflicts with SRB's locked 4.41.2) and
``rsl-rl-lib`` (its 5.5.1 version is already in SRB's lock).

Git branches and tags require reviewed pins in ``PINNED_GIT_REFS``; Newton's
``release-1.6`` dependency is pinned there to avoid resolving a moving branch.
"""

from __future__ import annotations

import re
import sys
import tomllib
from pathlib import Path

EXCLUDED_PREFIXES: tuple[str, ...] = (
    "isaaclab",
    "omniverseclient",
    "isaacteleop",
    "open3d",
    "imgui-bundle",
    "pyopengl-accelerate",
    "transformers",
    "rsl-rl-lib",
)

MASKED: tuple[str, ...] = ("omniverseclient", "isaacteleop")

PINNED_GIT_REFS: dict[str, str] = {
    "https://github.com/newton-physics/newton.git@release-1.6": (
        "a1361df6072fc30f0fb57d48124da6f97e68cf9d"
    ),
}

_GIT_REFERENCE = re.compile(r"git\+(?P<url>[^\s@;#]+)@(?P<ref>[^\s;#]+)")
_FULL_SHA = re.compile(r"[0-9a-f]{40}")


def _name(requirement: str) -> str:
    return (
        re.split(r"[\s;<>=!\[~]", requirement.strip(), 1)[0].lower().replace("_", "-")
    )


def pin_git_reference(requirement: str) -> str:
    """Rewrite a git requirement's branch or tag to its reviewed commit."""
    match = _GIT_REFERENCE.search(requirement)
    if match is None or _FULL_SHA.fullmatch(match["ref"]):
        return requirement
    key = f"{match['url']}@{match['ref']}"
    if key not in PINNED_GIT_REFS:
        raise ValueError(
            f"{requirement!r} names the git reference {match['ref']!r}, which has no "
            "reviewed commit in PINNED_GIT_REFS; review the upstream ref and pin "
            "its full commit SHA"
        )
    start, end = match.span("ref")
    return requirement[:start] + PINNED_GIT_REFS[key] + requirement[end:]


def requirements(pyproject: dict) -> list[str]:
    return [
        pin_git_reference(dep.strip())
        for dep in pyproject["project"]["dependencies"]
        if not _name(dep).startswith(EXCLUDED_PREFIXES)
    ]


def overrides(pyproject: dict) -> list[str]:
    masked = [f'{name} ; python_version < "0"' for name in MASKED]
    return masked + [
        pin_git_reference(dep)
        for dep in pyproject.get("tool", {})
        .get("uv", {})
        .get("override-dependencies", [])
    ]


def main(argv: list[str]) -> int:
    if len(argv) != 3 or argv[1] not in ("requirements", "overrides"):
        print(__doc__, file=sys.stderr)
        return 2
    with Path(argv[2]).open("rb") as stream:
        pyproject = tomllib.load(stream)
    try:
        lines = (
            requirements(pyproject)
            if argv[1] == "requirements"
            else overrides(pyproject)
        )
    except ValueError as error:
        print(f"{argv[2]}: {error}", file=sys.stderr)
        return 1
    if not lines:
        print(f"{argv[2]}: no {argv[1]} found", file=sys.stderr)
        return 1
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
