"""Shared, framework-neutral Hugging Face Hub transport primitives."""

from __future__ import annotations

import importlib.util
import os
import secrets
import shutil
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
from typing import Any, Generic, TypeVar

from srb.utils import artifact_cards as _artifact_cards
from srb.utils.publication import MaterializedDirectorySnapshot, default_new_file_mode
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES,
    MAX_PUBLICATION_DIRECTORY_BYTES,
    MAX_PUBLICATION_DIRECTORY_DEPTH,
    MAX_PUBLICATION_DIRECTORY_ENTRIES,
    ByteBudget,
    ResourceLimitError,
    require_bounded_int,
    require_directory_budget,
)

ManifestT = TypeVar("ManifestT")

format_huggingface_card_title = _artifact_cards.format_card_title
format_huggingface_metadata_value = _artifact_cards.format_metadata_value


@dataclass(frozen=True)
class HuggingFaceHubUploadPlan(Generic[ManifestT]):
    """Resolved files and repository target for a Hugging Face upload."""

    repo_id: str
    repo_type: str
    local_dir: Path
    files: list[str]
    manifest: ManifestT
    commit_message: str | None = None
    generated_files: list[str] = field(default_factory=list)
    upload_dir: Path | None = field(default=None, compare=False)
    _snapshot: MaterializedDirectorySnapshot | None = field(
        default=None,
        repr=False,
        compare=False,
    )

    def __post_init__(self) -> None:
        if self.upload_dir is None:
            object.__setattr__(self, "upload_dir", self.local_dir)

    def close(self) -> None:
        if self._snapshot is not None:
            self._snapshot.close()

    def __enter__(self) -> HuggingFaceHubUploadPlan[ManifestT]:
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()

    def to_dict(self, *, dry_run: bool) -> dict[str, Any]:
        payload = {
            "dry_run": dry_run,
            "repo_id": self.repo_id,
            "repo_type": self.repo_type,
            "local_dir": str(self.local_dir),
            "files": self.files,
        }
        if self.commit_message is not None:
            payload["commit_message"] = self.commit_message
        if self.generated_files:
            payload["generated_files"] = list(self.generated_files)
        return payload


@dataclass(frozen=True)
class HuggingFaceHubCardResult(Generic[ManifestT]):
    """Result of writing a Hugging Face README.md card."""

    path: Path
    written: bool
    repo_id: str
    manifest: ManifestT

    def to_dict(self) -> dict[str, Any]:
        return {
            "card_path": str(self.path),
            "written": self.written,
            "repo_id": self.repo_id,
        }


def list_huggingface_upload_files(local_dir: str | Path) -> list[str]:
    """Return deterministic repository-relative upload file paths."""
    root = Path(local_dir)
    if not root.is_dir():
        raise ValueError(f"Hugging Face upload root is not a directory: {root}")
    files = []
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"Hugging Face upload tree contains a symlink: {path}")
        if path.is_file():
            files.append(str(path.relative_to(root)))
    return sorted(files)


def generated_huggingface_card_files(
    files: Sequence[str],
    *,
    card_filename: str = "README.md",
) -> list[str]:
    """Return card files a dry-run push would generate before upload."""
    return [] if card_filename in files else [card_filename]


def raise_for_blocking_huggingface_publication_warnings(
    warnings: Sequence[str],
    *,
    blocking_prefixes: Sequence[str],
    artifact_label: str,
) -> None:
    """Raise when validation warnings block Hub publication."""
    blocking = [
        warning
        for warning in warnings
        if any(warning.startswith(prefix) for prefix in blocking_prefixes)
    ]
    if blocking:
        raise ValueError(f"{artifact_label} is not publishable: " + "; ".join(blocking))


def require_huggingface_hub(artifact_label: str) -> None:
    """Raise a user-facing error when the optional Hub dependency is missing."""
    if importlib.util.find_spec("huggingface_hub") is None:
        raise RuntimeError(
            f"{artifact_label} upload requires huggingface_hub. "
            "Install huggingface_hub or use --dry-run to inspect the upload plan."
        )


def upload_huggingface_folder(
    *,
    repo_id: str,
    repo_type: str,
    local_dir: str | Path,
    private: bool,
    commit_message: str,
    mirror: bool = False,
) -> None:
    """Create a Hub repository and upload a local folder.

    ``mirror=True`` deletes remote files absent locally; otherwise uploads only
    add or overwrite files.
    """
    from huggingface_hub import HfApi

    # Fail closed before any Hub call: upload_folder would follow symlinks.
    list_huggingface_upload_files(local_dir)
    api = HfApi()
    api.create_repo(
        repo_id=repo_id,
        repo_type=repo_type,
        private=private,
        exist_ok=True,
    )
    api.upload_folder(
        repo_id=repo_id,
        repo_type=repo_type,
        folder_path=str(local_dir),
        commit_message=commit_message,
        delete_patterns="*" if mirror else None,
    )


def resolve_huggingface_revision(
    *,
    repo_id: str,
    repo_type: str,
    revision: str | None = None,
) -> str:
    """Resolve a branch or tag to its immutable commit SHA."""
    from huggingface_hub import HfApi

    info = HfApi().repo_info(repo_id, repo_type=repo_type, revision=revision)
    sha = getattr(info, "sha", None)
    if not isinstance(sha, str) or len(sha) != 40:
        raise RuntimeError(
            f"Hugging Face Hub did not report a commit SHA for {repo_id}@{revision or 'default'}"
        )
    return sha


_SNAPSHOT_BLOCK_BYTES = 1 << 20


@dataclass(frozen=True)
class _SnapshotFile:
    """One repository file admitted by the metadata preflight."""

    parts: tuple[str, ...]
    size: int


def _repository_path_parts(path: object) -> tuple[str, ...]:
    """Return the components of one relative, normalized repository path."""
    if not isinstance(path, str) or not path or "\\" in path or "\x00" in path:
        raise ValueError(f"unsafe Hugging Face repository path: {path!r}")
    parts = tuple(path.split("/"))
    if PurePosixPath(path).is_absolute() or any(
        part in ("", ".", "..") for part in parts
    ):
        raise ValueError(f"unsafe Hugging Face repository path: {path!r}")
    return parts


def _inventory_huggingface_snapshot(
    *,
    repo_id: str,
    repo_type: str,
    sha: str,
    max_entries: int,
    max_bytes: int,
    max_file_bytes: int,
    max_depth: int,
) -> list[_SnapshotFile]:
    """Validate streamed tree metadata and declared size limits before transfer."""
    from huggingface_hub import HfApi
    from huggingface_hub.hf_api import RepoFile, RepoFolder

    listing = HfApi().list_repo_tree(
        repo_id, recursive=True, expand=False, revision=sha, repo_type=repo_type
    )
    files: list[_SnapshotFile] = []
    seen: set[tuple[str, ...]] = set()
    aggregate = ByteBudget("Hugging Face snapshot declared bytes", max_bytes)
    for entries, entry in enumerate(listing, start=1):
        if entries > max_entries:
            raise ResourceLimitError(
                "Hugging Face snapshot entries", limit=max_entries, observed=entries
            )
        parts = _repository_path_parts(getattr(entry, "path", None))
        if parts in seen:
            raise ValueError(
                f"Hugging Face repository path listed more than once: {'/'.join(parts)!r}"
            )
        seen.add(parts)
        if isinstance(entry, RepoFolder):
            depth = len(parts)
        elif isinstance(entry, RepoFile):
            depth = len(parts) - 1
        else:
            raise TypeError(
                f"unexpected Hugging Face tree entry {type(entry).__name__} "
                f"for {'/'.join(parts)!r}"
            )
        if depth > max_depth:
            raise ResourceLimitError(
                f"Hugging Face snapshot depth of {'/'.join(parts)!r}",
                limit=max_depth,
                observed=depth,
            )
        if isinstance(entry, RepoFolder):
            continue
        size = entry.size
        if isinstance(size, bool) or not isinstance(size, int) or size < 0:
            raise ValueError(
                f"Hugging Face file {'/'.join(parts)!r} has no valid size "
                f"metadata: {size!r}"
            )
        if size > max_file_bytes:
            raise ResourceLimitError(
                f"Hugging Face file {'/'.join(parts)!r}",
                limit=max_file_bytes,
                observed=size,
            )
        aggregate.consume(size)
        files.append(_SnapshotFile(parts=parts, size=size))
    return files


def _transfer_huggingface_file(
    *,
    repo_id: str,
    repo_type: str,
    sha: str,
    member: _SnapshotFile,
    destination: Path,
) -> None:
    """Stream one file, rejecting the block that crosses its declared size."""
    import requests
    from huggingface_hub import constants, hf_hub_url
    from huggingface_hub.utils import build_hf_headers

    name = "/".join(member.parts)
    url = hf_hub_url(repo_id, name, repo_type=repo_type, revision=sha)
    response = requests.get(
        url,
        headers=build_hf_headers(),
        stream=True,
        timeout=constants.HF_HUB_DOWNLOAD_TIMEOUT,
    )
    try:
        response.raise_for_status()
        received = 0
        with open(
            destination,
            "xb",
            opener=lambda path, flags: os.open(path, flags, 0o600),
        ) as output:
            os.fchmod(output.fileno(), 0o600)
            for chunk in response.iter_content(chunk_size=_SNAPSHOT_BLOCK_BYTES):
                if not chunk:
                    continue
                received += len(chunk)
                if received > member.size:
                    raise ResourceLimitError(
                        f"Hugging Face file {name!r} downloaded bytes",
                        limit=member.size,
                        observed=received,
                    )
                output.write(chunk)
        if received != member.size:
            raise OSError(
                f"Hugging Face file {name!r} is truncated: declared "
                f"{member.size} bytes, received {received}"
            )
    finally:
        response.close()


def _require_fresh_snapshot_destination(destination: Path) -> None:
    """Refuse an output that already holds anything, or is not a directory."""
    if not os.path.lexists(destination):
        return
    if destination.is_symlink() or not destination.is_dir():
        raise FileExistsError(
            f"Hugging Face download output exists and is not a directory: {destination}"
        )
    with os.scandir(destination) as children:
        if next(children, None) is not None:
            raise FileExistsError(
                f"Hugging Face download output is not empty: {destination}. "
                "Choose a new output directory or remove the previous download; "
                "a snapshot is never merged into existing files."
            )


def download_huggingface_snapshot(
    *,
    repo_id: str,
    repo_type: str,
    local_dir: str | Path,
    revision: str | None = None,
    max_entries: int = MAX_PUBLICATION_DIRECTORY_ENTRIES,
    max_bytes: int = MAX_PUBLICATION_DIRECTORY_BYTES,
    max_file_bytes: int = MAX_EXTERNAL_FILE_BYTES,
    max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
) -> tuple[Path, str]:
    """Download a revision into an absent or empty directory.

    Resolve mutable revision names to a commit SHA. Validate tree and byte
    limits, stage into a private directory, then rename only when complete.
    """
    destination = Path(local_dir)
    _require_fresh_snapshot_destination(destination)
    limits = {
        "max_entries": require_bounded_int(
            max_entries,
            resource="Hugging Face snapshot entry limit",
            minimum=1,
            maximum=MAX_PUBLICATION_DIRECTORY_ENTRIES,
        ),
        "max_bytes": require_bounded_int(
            max_bytes,
            resource="Hugging Face snapshot byte limit",
            minimum=1,
            maximum=MAX_PUBLICATION_DIRECTORY_BYTES,
        ),
        "max_file_bytes": require_bounded_int(
            max_file_bytes,
            resource="Hugging Face snapshot file limit",
            minimum=1,
            maximum=MAX_EXTERNAL_FILE_BYTES,
        ),
        "max_depth": require_bounded_int(
            max_depth,
            resource="Hugging Face snapshot depth limit",
            minimum=1,
            maximum=MAX_PUBLICATION_DIRECTORY_DEPTH,
        ),
    }
    sha = resolve_huggingface_revision(
        repo_id=repo_id, repo_type=repo_type, revision=revision
    )
    members = _inventory_huggingface_snapshot(
        repo_id=repo_id, repo_type=repo_type, sha=sha, **limits
    )

    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = destination.parent / f".{destination.name}.hub-stage-{secrets.token_hex(8)}"
    stage.mkdir(mode=0o700)
    published = False
    try:
        os.chmod(stage, 0o700)
        for member in members:
            target = stage.joinpath(*member.parts)
            parent = stage
            for part in member.parts[:-1]:
                parent = parent / part
                parent.mkdir(mode=0o700, exist_ok=True)
                os.chmod(parent, 0o700)
            _transfer_huggingface_file(
                repo_id=repo_id,
                repo_type=repo_type,
                sha=sha,
                member=member,
                destination=target,
            )
        require_directory_budget(
            stage, resource="Hugging Face snapshot stage", **limits
        )
        published_file_mode = default_new_file_mode()
        for member in members:
            os.chmod(
                stage.joinpath(*member.parts),
                published_file_mode,
            )
        # Do not merge files written to the destination during the download.
        os.rename(stage, destination)
        published = True
    finally:
        # Propagate cleanup failures with the original exception as context.
        if not published:
            shutil.rmtree(stage)
    return destination, sha


@dataclass(frozen=True)
class HuggingFaceHubDownloadPlan:
    """Resolved repository target and local destination for a Hub download."""

    repo_id: str
    repo_type: str
    local_dir: Path
    revision: str | None = None

    def to_dict(self, *, dry_run: bool) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "dry_run": dry_run,
            "repo_id": self.repo_id,
            "repo_type": self.repo_type,
            "local_dir": str(self.local_dir),
        }
        if self.revision is not None:
            payload["revision"] = self.revision
        return payload


__all__ = [
    "HuggingFaceHubCardResult",
    "HuggingFaceHubDownloadPlan",
    "HuggingFaceHubUploadPlan",
    "download_huggingface_snapshot",
    "format_huggingface_card_title",
    "format_huggingface_metadata_value",
    "generated_huggingface_card_files",
    "list_huggingface_upload_files",
    "raise_for_blocking_huggingface_publication_warnings",
    "require_huggingface_hub",
    "resolve_huggingface_revision",
    "upload_huggingface_folder",
]
