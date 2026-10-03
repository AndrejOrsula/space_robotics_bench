"""Build EventRecorder provenance from sim-agent CLI inputs.

Missing values use empty strings so episode rows stay well-formed.
"""

from __future__ import annotations

import hashlib
import os
import stat
from enum import Enum
from pathlib import Path

from srb.utils.digest import sha256_file

#: Versioned domain prefix for directory-checkpoint manifest hashes.
_DIRECTORY_MANIFEST_DOMAIN = b"srb.policy-checkpoint-directory.v1\0"
CHECKPOINT_FREE_POLICY_IDS = frozenset(
    ("random", "human", "zero", "rand", "teleop", "ros", "synthetic_fixture")
)


def _policy_label(value: str | Enum | None) -> str:
    """Return a stable label from a string or string-valued enum."""
    if isinstance(value, Enum):
        value = value.value
    return value if isinstance(value, str) else ""


def is_checkpoint_free_policy(policy_id: str | None) -> bool:
    """Whether this exact policy identity has no checkpoint by definition."""
    return isinstance(policy_id, str) and policy_id in CHECKPOINT_FREE_POLICY_IDS


def _manifest_record(*fields: str) -> bytes:
    """Encode length-prefixed fields; POSIX paths may contain newlines."""

    encoded = [field.encode("utf-8", "surrogateescape") for field in fields]
    return b"".join(b"%d:%s" % (len(item), item) for item in encoded) + b"\n"


def _warn_unhashed(path: Path, reason: str) -> None:
    from srb.utils import logging

    logging.warning(
        f"Policy checkpoint hash could not be established for {path}: {reason}; "
        "episode provenance will record an empty policy_checkpoint_hash"
    )


def _directory_manifest_sha256(root: Path) -> str | None:
    """Hash sorted path and content records without following symlinks.

    Return ``None`` for empty trees or unsupported members.
    """

    manifest = hashlib.sha256(_DIRECTORY_MANIFEST_DOMAIN)
    members = 0

    def _raise(error: OSError) -> None:
        raise error

    for current, directories, files in os.walk(
        root, topdown=True, followlinks=False, onerror=_raise
    ):
        directories.sort()
        base = Path(current)
        for name in sorted((*directories, *files)):
            member = base / name
            relative = member.relative_to(root).as_posix()
            mode = os.lstat(member).st_mode
            if stat.S_ISLNK(mode):
                record = _manifest_record("symlink", relative, os.readlink(member))
            elif stat.S_ISDIR(mode):
                record = _manifest_record("dir", relative)
            elif stat.S_ISREG(mode):
                record = _manifest_record(
                    "file",
                    relative,
                    str(os.lstat(member).st_size),
                    sha256_file(member),
                )
            else:
                return None
            manifest.update(record)
            members += 1
    return manifest.hexdigest() if members else None


def compute_checkpoint_hash(model_path: Path | str | None) -> str:
    """Hash a checkpoint file or deterministic directory manifest, or return ``""``.

    Files stream in 1 MiB chunks; directory records include sorted relative
    paths and content. Resolve a symlinked root. Missing, unreadable, empty, or
    unsupported paths log a warning and return ``""``; recording continues.
    """
    if model_path is None:
        return ""
    path = Path(model_path)
    try:
        if path.is_dir():
            root = path.resolve(strict=True)
            digest = _directory_manifest_sha256(root)
            if digest is None:
                _warn_unhashed(
                    path,
                    "directory has no hashable members or contains unsupported members",
                )
                return ""
            return digest
        if path.is_file():
            return sha256_file(path)
    except (OSError, RuntimeError, ValueError) as exc:
        _warn_unhashed(path, f"{type(exc).__name__}: {exc}")
        return ""
    _warn_unhashed(path, "path is neither a regular file nor a directory")
    return ""


def derive_policy_id(
    *,
    agent_subcommand: str | None = None,
    collector: str | Enum | None = None,
    algo: str | None = None,
    model_path: Path | str | None = None,
) -> str:
    """Build a stable policy ID from CLI inputs.

    Prefer a checkpoint-free collector, then ``<algo>:<run-dir>:<checkpoint>``
    when model and algorithm are known, then the algorithm, agent subcommand,
    or ``""``. The checkpoint ID omits the full path.
    """
    collector_id = _policy_label(collector)
    if is_checkpoint_free_policy(collector_id):
        return collector_id
    if model_path is not None and algo:
        path = Path(model_path)
        run_dir = path.parent.name or ""
        stem = path.stem or ""
        parts = [algo]
        if run_dir:
            parts.append(run_dir)
        if stem:
            parts.append(stem)
        return ":".join(parts)
    if algo:
        return str(algo)
    if agent_subcommand:
        return str(agent_subcommand)
    return ""


def detect_runtime_provenance() -> tuple[str, str]:
    """Return best-effort ``(srb_git_sha, isaaclab_version)``; unavailable is ``""``."""
    from srb.utils.source_revision import detect_srb_git_sha

    repo_root = Path(__file__).resolve().parents[2]
    git_sha = detect_srb_git_sha(repo_root)

    isaaclab_version = ""
    try:
        import isaaclab  # type: ignore[import-not-found]

        isaaclab_version = str(getattr(isaaclab, "__version__", "") or "")
    except ImportError:
        try:
            from importlib.metadata import PackageNotFoundError, version

            isaaclab_version = version("isaaclab")
        except (PackageNotFoundError, Exception):  # noqa: BLE001
            isaaclab_version = ""

    return git_sha, isaaclab_version


def build_event_provenance(
    *,
    env_id: str | None = None,
    agent_subcommand: str | None = None,
    collector: str | Enum | None = None,
    algo: str | None = None,
    model_path: Path | str | None = None,
) -> dict[str, str]:
    """Build EventRecorder fields; callers may override precomputed values."""
    git_sha, isaaclab_version = detect_runtime_provenance()
    collector_id = _policy_label(collector)
    policy_id = derive_policy_id(
        agent_subcommand=agent_subcommand,
        collector=collector,
        algo=algo,
        model_path=model_path,
    )
    return {
        "policy_id": policy_id,
        "policy_checkpoint_hash": (
            ""
            if is_checkpoint_free_policy(collector_id)
            else compute_checkpoint_hash(model_path)
        ),
        "task_id": str(env_id or ""),
        "srb_git_sha": git_sha,
        "isaaclab_version": isaaclab_version,
    }


__all__ = [
    "CHECKPOINT_FREE_POLICY_IDS",
    "build_event_provenance",
    "compute_checkpoint_hash",
    "derive_policy_id",
    "detect_runtime_provenance",
    "is_checkpoint_free_policy",
]
