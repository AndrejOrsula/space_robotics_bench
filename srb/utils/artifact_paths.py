"""Strict cross-platform names and contained paths for generated artifacts."""

from __future__ import annotations

import re
from pathlib import Path, PurePosixPath

_SEGMENT_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}\Z")
_WINDOWS_RESERVED = frozenset(
    {
        "CON",
        "PRN",
        "AUX",
        "NUL",
        *(f"COM{index}" for index in range(1, 10)),
        *(f"LPT{index}" for index in range(1, 10)),
    }
)


def validate_artifact_segment(value: object, *, field: str) -> str:
    """Return one portable filename segment or reject it."""

    if not isinstance(value, str):
        raise TypeError(f"{field} must be a string")
    if not _SEGMENT_RE.fullmatch(value):
        raise ValueError(f"{field} must match {_SEGMENT_RE.pattern!r}; got {value!r}")
    if value in {".", ".."} or value.rstrip(" .") != value:
        raise ValueError(f"{field} is not a safe artifact name: {value!r}")
    if value.split(".", 1)[0].upper() in _WINDOWS_RESERVED:
        raise ValueError(f"{field} uses a reserved platform name: {value!r}")
    return value


def validate_artifact_relative_path(value: object, *, field: str) -> PurePosixPath:
    """Return a normalized, relative POSIX artifact member path."""

    if not isinstance(value, str):
        raise TypeError(f"{field} must be a string")
    if not value or len(value) > 1024 or "\\" in value:
        raise ValueError(f"{field} must be a non-empty portable relative path")
    relative = PurePosixPath(value)
    if relative.is_absolute() or relative.as_posix() != value:
        raise ValueError(f"{field} must be a normalized relative POSIX path")
    if not relative.parts:
        raise ValueError(f"{field} must name an artifact member")
    for index, part in enumerate(relative.parts):
        validate_artifact_segment(part, field=f"{field} segment {index}")
    return relative


def resolve_artifact_member(root: Path | str, value: object, *, field: str) -> Path:
    """Resolve one member while rejecting traversal and symlink escape."""

    relative = validate_artifact_relative_path(value, field=field)
    resolved_root = Path(root).resolve()
    candidate = resolved_root.joinpath(*relative.parts).resolve(strict=False)
    try:
        candidate.relative_to(resolved_root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes artifact root {resolved_root}") from exc
    return candidate


__all__ = [
    "resolve_artifact_member",
    "validate_artifact_relative_path",
    "validate_artifact_segment",
]
