"""Best-effort lookup of the SRB source revision for event provenance."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import NoReturn

from srb.utils.json_types import JsonDecodeBudget
from srb.utils.process_supervisor import (
    ProcessKind,
    ProcessPlatformError,
    ProcessSpawnError,
    ProcessStatus,
    run_owned,
)
from srb.utils.resource_budget import require_file_size

MAX_SOURCE_MANIFEST_BYTES = 64 * 1024 * 1024
_GIT_OBJECT_RE = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})\Z")


def _reject_constant(value: str) -> NoReturn:
    raise ValueError(f"invalid JSON constant: {value}")


def _unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _manifest_revision(path: Path) -> str:
    try:
        size = require_file_size(
            path,
            resource="source provenance manifest",
            max_bytes=MAX_SOURCE_MANIFEST_BYTES,
        )
        with path.open("rb") as stream:
            raw = stream.read(MAX_SOURCE_MANIFEST_BYTES + 1)
    except (OSError, ValueError):
        return ""
    if len(raw) != size or len(raw) > MAX_SOURCE_MANIFEST_BYTES:
        return ""

    try:
        JsonDecodeBudget().loads(
            raw,
            context="source provenance manifest",
            max_bytes=MAX_SOURCE_MANIFEST_BYTES,
        )
        manifest = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_reject_constant,
        )
        canonical = (
            json.dumps(
                manifest,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")
    except (UnicodeError, json.JSONDecodeError, RecursionError, TypeError, ValueError):
        return ""
    if raw != canonical:
        return ""
    if not isinstance(manifest, dict):
        return ""
    if (
        manifest.get("schema") != "srb.source-provenance"
        or type(manifest.get("schema_version")) is not int
        or manifest.get("schema_version") != 1
    ):
        return ""
    producer = manifest.get("producer")
    if not isinstance(producer, dict) or (
        producer.get("id") != "srb-source-provenance"
        or type(producer.get("version")) is not int
        or producer.get("version") != 1
    ):
        return ""
    source = manifest.get("source")
    if not isinstance(source, dict) or source.get("worktree") != "clean":
        return ""
    root_commit = source.get("root_commit")
    root_tree = source.get("root_tree")
    if not isinstance(root_commit, str) or not _GIT_OBJECT_RE.fullmatch(root_commit):
        return ""
    if not isinstance(root_tree, str) or not _GIT_OBJECT_RE.fullmatch(root_tree):
        return ""
    return root_commit


def detect_srb_git_sha(project_root: Path) -> str:
    """Resolve this project root's Git revision or adjacent build manifest.

    A root Git marker takes precedence, including when Git is unavailable or
    cannot read the repository. Manifest fallback is limited to the exact
    project-root-adjacent source manifest; no ancestor or ambient source is
    inferred.
    """

    root = project_root.resolve()
    git_marker = root / ".git"
    if git_marker.exists():
        try:
            result = run_owned(
                [
                    "git",
                    "-c",
                    f"safe.directory={root}",
                    "-C",
                    str(root),
                    "rev-parse",
                    "HEAD",
                ],
                kind=ProcessKind.FINITE,
                timeout=2.0,
                capture_output=True,
            )
        except (ProcessPlatformError, ProcessSpawnError):
            return ""
        if (
            result.status is ProcessStatus.EXITED
            and result.returncode == 0
            and result.cleanup_complete
        ):
            revision = (result.stdout or "").strip()
            return revision if _GIT_OBJECT_RE.fullmatch(revision) else ""
        return ""
    return _manifest_revision(root / ".source-provenance.json")


__all__ = ["detect_srb_git_sha"]
