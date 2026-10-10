"""Validation session identity, paths, and spec snapshot persistence."""

from __future__ import annotations

import datetime
import hashlib
import json
import logging
import re
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from srb.interfaces.sim_to_real.validation.artifact_reference import SPEC_YAML
from srb.utils.process_supervisor import (
    ProcessKind,
    ProcessSpawnError,
    ProcessStatus,
    run_owned,
)
from srb.utils.publication import ArtifactPublication, sha256_file
from srb.utils.validation_spec import ValidationSpec

logger = logging.getLogger(__name__)

_SAFE_COMPONENT_RE = re.compile(
    r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}\Z",
    re.ASCII,
)
_WORKSPACE_PATH_ERROR = "unsafe validation workspace path"


class ValidationWorkspaceError(RuntimeError):
    """Raised when a validation workspace path is unsafe."""


def _workspace_path_error() -> ValidationWorkspaceError:
    return ValidationWorkspaceError(_WORKSPACE_PATH_ERROR)


def _absolute_path(path: Path) -> Path:
    if path.is_absolute():
        return path
    return Path.cwd() / path


def _component_contained(container: Path, candidate: Path) -> bool:
    try:
        candidate.relative_to(container)
    except ValueError:
        return False
    return True


def _check_existing_components(path: Path) -> None:
    """Reject symlinks and non-directories in an existing path prefix."""

    absolute = _absolute_path(path)
    current = Path(absolute.anchor or ".")
    try:
        for component in absolute.parts[1:]:
            current /= component
            if current.is_symlink():
                raise _workspace_path_error()
            if current.exists() and not current.is_dir():
                raise _workspace_path_error()
    except (OSError, RuntimeError):
        raise _workspace_path_error() from None


def _resolve_storage_root(storage_root: str | Path) -> Path:
    try:
        resolved = Path(storage_root).expanduser().resolve(strict=False)
    except (OSError, RuntimeError, TypeError, ValueError):
        raise _workspace_path_error() from None
    _check_existing_components(resolved)
    return resolved


def _validate_candidate(root: Path, candidate: Path) -> Path:
    absolute = _absolute_path(candidate)
    if not _component_contained(root, absolute):
        raise _workspace_path_error()
    _check_existing_components(absolute)
    try:
        resolved = absolute.resolve(strict=False)
    except (OSError, RuntimeError):
        raise _workspace_path_error() from None
    if not _component_contained(root, resolved):
        raise _workspace_path_error()
    return resolved


def _safe_component(value: object) -> bool:
    return isinstance(value, str) and _SAFE_COMPONENT_RE.fullmatch(value) is not None


def _validate_workspace_paths(
    storage_root: str | Path,
    task: object,
    session_dir: str | Path,
) -> tuple[Path, Path, Path, Path]:
    """Validate and resolve the root, task, session, and staging paths."""

    if not _safe_component(task):
        raise _workspace_path_error()
    try:
        session_candidate = _absolute_path(Path(session_dir))
    except (TypeError, ValueError):
        raise _workspace_path_error() from None
    if not _safe_component(session_candidate.name):
        raise _workspace_path_error()

    root = _resolve_storage_root(storage_root)
    task_candidate = root / task
    staging_candidate = session_candidate / ".staging"

    if not _component_contained(root, task_candidate):
        raise _workspace_path_error()
    if not _component_contained(task_candidate, session_candidate):
        raise _workspace_path_error()

    task_path = _validate_candidate(root, task_candidate)
    session_path = _validate_candidate(root, session_candidate)
    staging_path = _validate_candidate(root, staging_candidate)

    if task_path == session_path or not _component_contained(task_path, session_path):
        raise _workspace_path_error()
    if not _component_contained(session_path, staging_path):
        raise _workspace_path_error()
    return root, task_path, session_path, staging_path


@dataclass(frozen=True)
class ValidationSessionIdentity:
    """Immutable identity attached to one validation attempt."""

    session_id: str
    task: str
    algorithm: str
    code_sha: str
    config_digest: str
    artifact_digest: str

    def to_dict(self) -> dict[str, str]:
        return {
            "session_id": self.session_id,
            "task": self.task,
            "algorithm": self.algorithm,
            "code_sha": self.code_sha,
            "config_digest": self.config_digest,
            "artifact_digest": self.artifact_digest,
        }


@dataclass(frozen=True)
class ValidationSessionWorkspace:
    """Resolved local workspace for one validation session."""

    session_id: str
    task: str
    algorithm: str
    code_sha: str
    config_digest: str
    artifact_digest: str
    git_sha: str
    timestamp: str
    storage_root: Path
    session_dir: Path

    @property
    def identity(self) -> ValidationSessionIdentity:
        return ValidationSessionIdentity(
            session_id=self.session_id,
            task=self.task,
            algorithm=self.algorithm,
            code_sha=self.code_sha,
            config_digest=self.config_digest,
            artifact_digest=self.artifact_digest,
        )

    @property
    def staging_dir(self) -> Path:
        """Private run directory, removed only after successful finalization."""

        return self.session_dir / ".staging"


def current_git_sha() -> str:
    """Return the current short git SHA, or ``unknown`` outside a checkout."""

    try:
        result = run_owned(
            ["git", "rev-parse", "--short", "HEAD"],
            kind=ProcessKind.FINITE,
            timeout=10.0,
            capture_output=True,
        )
        if (
            result.status is ProcessStatus.EXITED
            and result.returncode == 0
            and result.cleanup_complete
        ):
            return (result.stdout or "").strip() or "unknown"
    except (ProcessSpawnError, OSError):
        return "unknown"
    return "unknown"


def current_code_sha() -> str:
    """Return the full source SHA, or ``unknown`` outside a checkout."""

    try:
        result = run_owned(
            ["git", "rev-parse", "HEAD"],
            kind=ProcessKind.FINITE,
            timeout=10.0,
            capture_output=True,
        )
        if (
            result.status is ProcessStatus.EXITED
            and result.returncode == 0
            and result.cleanup_complete
        ):
            return (result.stdout or "").strip() or "unknown"
    except (ProcessSpawnError, OSError):
        return "unknown"
    return "unknown"


def _canonical_digest(value: Any) -> str:
    payload = json.dumps(
        value,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def validation_config_digest(spec: ValidationSpec) -> str:
    """Hash the canonical, resolved validation configuration."""

    return _canonical_digest(spec.model_dump(mode="json"))


def validation_artifact_digest(checkpoint: str | Path) -> str:
    """Hash checkpoint bytes or a deterministic directory manifest.

    Missing paths get an explicit digest; runtime preflight rejects unusable
    checkpoints. Including the path keeps unresolved sessions distinct.
    """

    path = Path(checkpoint)
    if path.is_file():
        return sha256_file(path)
    if path.is_dir():
        members: list[dict[str, str | int]] = []
        for member in sorted(path.rglob("*")):
            relative = member.relative_to(path).as_posix()
            if member.is_symlink():
                members.append(
                    {
                        "path": relative,
                        "kind": "symlink",
                        "target": member.readlink().as_posix(),
                    }
                )
            elif member.is_file():
                members.append(
                    {
                        "path": relative,
                        "kind": "file",
                        "size_bytes": member.stat().st_size,
                        "sha256": sha256_file(member),
                    }
                )
            elif not member.is_dir():
                members.append({"path": relative, "kind": "other"})
        return _canonical_digest(
            {"schema": "srb-validation-artifact/1", "members": members}
        )
    return _canonical_digest(
        {
            "schema": "srb-validation-artifact/1",
            "status": "missing",
            "path": str(path.resolve()),
        }
    )


def utc_session_timestamp(now: datetime.datetime | None = None) -> str:
    """Format a UTC timestamp for use in validation session directory names."""

    if now is None:
        now = datetime.datetime.now(datetime.timezone.utc)
    elif now.tzinfo is None:
        now = now.replace(tzinfo=datetime.timezone.utc)
    else:
        now = now.astimezone(datetime.timezone.utc)
    return now.strftime("%Y-%m-%dT%H-%M-%SZ")


def build_session_workspace(
    spec: ValidationSpec,
    *,
    git_sha: str | None = None,
    timestamp: str | None = None,
    code_sha: str | None = None,
) -> ValidationSessionWorkspace:
    """Resolve a unique, content-identified workspace for ``spec``."""

    sha = current_git_sha() if git_sha is None else git_sha
    source_sha = (
        current_code_sha()
        if code_sha is None and git_sha is None
        else (sha if code_sha is None else code_sha)
    )
    stamp = utc_session_timestamp() if timestamp is None else timestamp
    session_id = str(uuid.uuid4())
    session_component = f"{session_id}_{stamp}"
    if not _safe_component(session_component):
        raise _workspace_path_error()
    storage_root, _task_dir, session_dir, _staging_dir = _validate_workspace_paths(
        spec.storage_root,
        spec.task,
        _resolve_storage_root(spec.storage_root) / spec.task / session_component,
    )
    return ValidationSessionWorkspace(
        session_id=session_id,
        task=spec.task,
        algorithm=spec.algo,
        code_sha=source_sha,
        config_digest=validation_config_digest(spec),
        artifact_digest=validation_artifact_digest(spec.checkpoint),
        git_sha=sha,
        timestamp=stamp,
        storage_root=storage_root,
        session_dir=session_dir,
    )


def write_spec_snapshot(spec: ValidationSpec, session_dir: str | Path) -> None:
    """Persist ``spec.yaml`` for rebuild-summary; log and continue on failure."""

    _root, _task_dir, safe_session_dir, _staging_dir = _validate_workspace_paths(
        spec.storage_root,
        spec.task,
        session_dir,
    )
    try:
        payload = yaml.safe_dump(spec.model_dump(mode="json"), sort_keys=False)
        _root, _task_dir, safe_session_dir, _staging_dir = _validate_workspace_paths(
            spec.storage_root,
            spec.task,
            safe_session_dir,
        )
        ArtifactPublication.file(safe_session_dir / SPEC_YAML).publish_bytes(
            payload.encode("utf-8")
        )
    except Exception as exc:  # noqa: BLE001
        logger.warning("failed to persist spec.yaml: %s", exc)
