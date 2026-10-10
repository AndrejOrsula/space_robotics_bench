"""Canonical SRB dataset manifest helpers."""

from __future__ import annotations

import hashlib
import json
import os
import sys
from collections.abc import Mapping, Sequence
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import TypedDict, TypeGuard, cast

from srb.datasets.formats import get_dataset_format_adapter
from srb.datasets.schema import DatasetController, DatasetSplit, StorageFormat
from srb.utils.artifact_identity import ArtifactIdentity, slugify_underscore
from srb.utils.dataset_manifest import (
    DEFAULT_MANIFEST_RELPATH,
    MANIFEST_VERSION,
    DatasetManifest,
    DatasetManifestDict,
    _dataset_publication_marked,
    _open_dataset_publication_for_path,
    dataset_root_from_manifest_path,
    find_dataset_manifest,
    read_dataset_manifest,
)
from srb.utils.dataset_manifest import (
    resolve_dataset_path as _resolve_relative_dataset_path,
)
from srb.utils.failures import attempt_cleanup
from srb.utils.json_types import (
    FrozenJSONObject,
    ImmutableValueMixin,
    JsonDecodeBudget,
    JSONValue,
    bounded_json_dumps,
    thaw_json_value,
    validate_json_object,
)
from srb.utils.publication import (
    ArtifactPublication,
    MaterializedDirectorySnapshot,
    PublicationError,
    PublicationResourceLimitError,
    PublishedDirectory,
    directory_member_hashes,
    materialize_file_snapshot,
    sha256_file,
)
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    MAX_PUBLICATION_DIRECTORY_BYTES,
    MAX_PUBLICATION_DIRECTORY_ENTRIES,
    ByteBudget,
    ResourceLimitError,
    require_bounded_int,
)

__all__ = [
    "DEFAULT_MANIFEST_RELPATH",
    "MANIFEST_VERSION",
    "DatasetManifest",
    "DatasetManifestDict",
    "DatasetSnapshot",
    "DatasetValidationReport",
    "build_dataset_manifest",
    "dataset_manifest_bytes",
    "dataset_root_from_manifest_path",
    "find_dataset_manifest",
    "open_dataset_snapshot",
    "read_dataset_manifest",
    "resolve_dataset_path",
    "validate_dataset_manifest_output_path",
    "validate_dataset_layout",
    "write_dataset_manifest",
]


class DatasetValidationReportDict(TypedDict):
    """Serialized dataset validation result fields."""

    valid: bool
    errors: list[str]
    warnings: list[str]


@dataclass(frozen=True, slots=True)
class DatasetValidationReport(ImmutableValueMixin):
    """Result of checking manifest consistency and local files."""

    valid: bool
    errors: Sequence[str]
    warnings: Sequence[str] = field(default_factory=tuple)
    manifest: DatasetManifest | None = None
    member_hashes: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if type(self.valid) is not bool:
            raise TypeError("dataset validation report valid must be a boolean")
        if self.manifest is not None and type(self.manifest) is not DatasetManifest:
            raise TypeError(
                "dataset validation report manifest must be a DatasetManifest or null"
            )
        object.__setattr__(self, "errors", _freeze_messages(self.errors, "errors"))
        object.__setattr__(
            self,
            "warnings",
            _freeze_messages(self.warnings, "warnings"),
        )
        if not isinstance(self.member_hashes, Mapping):
            raise TypeError("dataset validation report member_hashes must be a mapping")
        if not all(
            type(member) is str
            and type(digest) is str
            and len(digest) == 64
            and all(character in "0123456789abcdef" for character in digest)
            for member, digest in self.member_hashes.items()
        ):
            raise ValueError("dataset validation report member_hashes are invalid")
        object.__setattr__(
            self,
            "member_hashes",
            MappingProxyType(dict(self.member_hashes)),
        )

    def to_dict(self) -> DatasetValidationReportDict:
        return {
            "valid": self.valid,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
        }


@dataclass
class DatasetSnapshot:
    """Verified dataset input, owned paths, and their snapshot lifetime."""

    manifest_path: Path | None
    manifest: DatasetManifest
    dataset_path: Path
    member_hashes: Mapping[str, str] = field(default_factory=dict)
    source_manifest_path: Path | None = None
    source_dataset_path: Path | None = None
    _member_identities: Mapping[Path, tuple[str, str]] = field(
        default_factory=dict, repr=False
    )
    _materialized: MaterializedDirectorySnapshot | None = field(
        default=None,
        repr=False,
    )
    _manifest_materialized: MaterializedDirectorySnapshot | None = field(
        default=None, repr=False
    )
    _closed: bool = field(default=False, init=False, repr=False)
    _cleanup_deferred: bool = field(default=False, init=False, repr=False)

    @property
    def external_reference(self) -> bool:
        """Whether the original data reference escapes its manifest root."""
        if self.source_manifest_path is None or self.source_dataset_path is None:
            return False
        source_root = _absolute_path(
            dataset_root_from_manifest_path(self.source_manifest_path)
        )
        return not _absolute_path(self.source_dataset_path).is_relative_to(source_root)

    def member_identity(self, path: Path) -> tuple[str, str]:
        """Return the source provenance name and digest for one owned member."""
        if self._closed:
            raise PublicationError("dataset snapshot is closed")
        identity = self._member_identities.get(_absolute_path(path))
        if identity is None:
            raise PublicationError(
                f"dataset member is not in the verified snapshot: {path}"
            )
        return identity

    def __enter__(self) -> "DatasetSnapshot":
        if self._closed:
            raise PublicationError("dataset snapshot is closed")
        if self._cleanup_deferred:
            raise PublicationError("dataset snapshot cleanup is pending")
        if self._materialized is not None:
            self._materialized.__enter__()
        if self._manifest_materialized is not None:
            self._manifest_materialized.__enter__()
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: object | None,
    ) -> None:
        if _exc is not None:
            from srb.datasets.hdf5_metadata import Hdf5MetadataCleanupError
            from srb.datasets.hdf5_native import Hdf5NativeCleanupError

            if isinstance(_exc, (Hdf5MetadataCleanupError, Hdf5NativeCleanupError)):
                if not self._cleanup_deferred:
                    self.defer_finalizer_cleanup()
                    _exc.retain_cleanup(self.close)
                return
        self.close()

    def defer_finalizer_cleanup(self) -> None:
        """Preserve temporary paths until an explicit owner retries cleanup."""
        if self._materialized is not None:
            self._materialized.defer_finalizer_cleanup()
        if self._manifest_materialized is not None:
            self._manifest_materialized.defer_finalizer_cleanup()
        self._cleanup_deferred = True

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._materialized is not None:
                self._materialized.close()
        finally:
            if self._manifest_materialized is not None:
                self._manifest_materialized.close()
        self._closed = True
        self._cleanup_deferred = False

    def __del__(self) -> None:  # pragma: no cover - interpreter cleanup
        if getattr(self, "_cleanup_deferred", False):
            return
        try:
            self.close()
        except Exception:
            pass


class _FrozenMessages(tuple[str, ...]):
    """Tuple-backed messages that retain list-comparison compatibility."""

    def __eq__(self, other: object) -> bool:
        if isinstance(other, (list, tuple)):
            return tuple(self) == tuple(other)
        return False

    def __ne__(self, other: object) -> bool:
        return not self == other


def _freeze_messages(values: Sequence[str], field_name: str) -> _FrozenMessages:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
        raise TypeError(f"dataset validation report {field_name} must be a sequence")
    messages = tuple(values)
    if not all(type(message) is str for message in messages):
        raise TypeError(f"dataset validation report {field_name} must contain strings")
    return _FrozenMessages(messages)


def build_dataset_manifest(
    *,
    identity: ArtifactIdentity,
    dataset_path: str | Path,
    split: DatasetSplit | str,
    controller: str,
    storage_format: StorageFormat | str,
    metadata: Mapping[str, object] | None = None,
    hf_org: str = "space_robotics_bench",
) -> DatasetManifest:
    """Create a normalized dataset manifest."""
    return DatasetManifest(
        identity=identity,
        dataset_path=str(Path(dataset_path)),
        split=DatasetSplit(str(split)),
        controller=controller,
        storage_format=StorageFormat(str(storage_format)),
        hf_repo_id=identity.dataset_repo_id(hf_org),
        metadata=validate_json_object(
            metadata if metadata is not None else {},
            context="dataset manifest metadata",
        ),
    )


def validate_dataset_manifest_output_path(output_path: str | Path) -> Path:
    """Validate that a manifest destination is outside published state."""
    path = Path(output_path)
    if _dataset_publication_marked(path):
        raise PublicationError(
            "cannot mutate a manifest inside a published dataset directory: "
            f"{dataset_root_from_manifest_path(path)}"
        )
    return path


def write_dataset_manifest(
    manifest: DatasetManifest,
    output_path: str | Path,
) -> Path:
    """Write a manifest as stable, human-readable JSON."""
    path = validate_dataset_manifest_output_path(output_path)
    payload = dataset_manifest_bytes(manifest)
    ArtifactPublication.file(path).publish_bytes(payload)
    return path


def _write_staged_dataset_manifest(
    manifest: DatasetManifest,
    output_path: str | Path,
    *,
    stage_root: str | Path,
) -> Path:
    """Write into a caller-owned private stage after validating its path."""
    path = Path(output_path)
    root = _absolute_path(Path(stage_root))
    staged_path = _absolute_path(path)
    _validate_staged_manifest_path(root, staged_path)
    _ensure_staged_manifest_parent(root, staged_path.parent)
    _validate_staged_manifest_path(root, staged_path)
    payload = dataset_manifest_bytes(manifest)

    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(staged_path, flags, 0o644)
    except OSError as exc:
        raise PublicationError(
            f"staged dataset manifest is not a safe regular file: {staged_path}"
        ) from exc
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
    return path


def dataset_manifest_bytes(
    manifest: DatasetManifest,
    *,
    max_bytes: int = MAX_METADATA_ATTRIBUTE_BYTES,
) -> bytes:
    """Serialize bounded canonical JSON before thawing frozen metadata."""
    byte_limit = require_bounded_int(
        max_bytes,
        resource="dataset manifest byte limit",
        minimum=1,
        maximum=MAX_METADATA_ATTRIBUTE_BYTES,
    )
    text = bounded_json_dumps(
        _dataset_manifest_structural_view(manifest),
        context="dataset manifest",
        max_bytes=byte_limit - 1,
        indent=2,
        allow_nan=False,
        sort_keys=True,
    )
    payload = (text + "\n").encode("utf-8")
    if len(payload) > byte_limit:
        raise ResourceLimitError(
            "dataset manifest JSON bytes",
            limit=byte_limit,
            observed=len(payload),
        )
    return payload


def _dataset_manifest_structural_view(
    manifest: DatasetManifest,
) -> dict[str, object]:
    """Build the canonical field view without thawing frozen metadata."""
    return {
        "format_version": manifest.format_version,
        "identity": manifest.identity.to_dict(),
        "dataset_path": manifest.dataset_path,
        "split": manifest.split.value,
        "controller": manifest.controller,
        "storage_format": manifest.storage_format.value,
        "hf_repo_id": manifest.hf_repo_id,
        "metadata": manifest.metadata,
    }


def _absolute_path(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path)))


def resolve_dataset_path(
    dataset_path: str | Path,
    manifest_path: str | Path | None,
) -> Path:
    """Resolve a manifest dataset path against its dataset root.

    Legacy manifests stored absolute paths, which break once a run directory is
    moved or copied out of its container. Such a path that no longer exists
    falls back to its shortest trailing components (e.g. ``data/demos.hdf5``)
    that exist under the manifest's dataset root.
    """
    path = Path(dataset_path)
    if path.is_absolute() and manifest_path is not None and not os.path.exists(path):
        root = dataset_root_from_manifest_path(manifest_path)
        for count in range(1, min(len(path.parts) - 1, 3) + 1):
            relocated = root.joinpath(*path.parts[-count:])
            if os.path.exists(relocated):
                return relocated
    return _resolve_relative_dataset_path(path, manifest_path)


def _validate_staged_manifest_path(root: Path, path: Path) -> None:
    if path == root or not path.is_relative_to(root):
        raise PublicationError(
            f"staged dataset manifest must be inside its private stage: {path}"
        )
    _validate_staged_manifest_root(root)

    relative = path.relative_to(root)
    current = root
    for component in relative.parts[:-1]:
        current /= component
        if current.is_symlink():
            raise PublicationError(
                f"staged dataset manifest path contains symlink: {current}"
            )
        if current.exists() and not current.is_dir():
            raise PublicationError(
                f"staged dataset manifest parent is not a directory: {current}"
            )
    if path.is_symlink():
        raise PublicationError(f"staged dataset manifest is a symlink: {path}")
    if path.exists() and not path.is_file():
        raise PublicationError(f"staged dataset manifest is not a regular file: {path}")


def _validate_staged_manifest_root(root: Path) -> None:
    if root.is_symlink():
        raise PublicationError(f"dataset publication stage is a symlink: {root}")
    if not root.is_dir():
        raise PublicationError(f"dataset publication stage is not a directory: {root}")

    current = Path(root.anchor)
    for component in root.parts[1:]:
        current /= component
        if current.is_symlink():
            raise PublicationError(
                f"dataset publication stage parent contains symlink: {current}"
            )
        if not current.is_dir():
            raise PublicationError(
                f"dataset publication stage parent is not a directory: {current}"
            )


def _ensure_staged_manifest_parent(root: Path, parent: Path) -> None:
    try:
        relative = parent.relative_to(root)
    except ValueError as exc:
        raise PublicationError(
            f"staged dataset manifest parent escapes its private stage: {parent}"
        ) from exc

    current = root
    for component in relative.parts:
        child = current / component
        if child.is_symlink():
            raise PublicationError(
                f"staged dataset manifest path contains symlink: {child}"
            )
        if child.exists():
            if not child.is_dir():
                raise PublicationError(
                    f"staged dataset manifest parent is not a directory: {child}"
                )
        else:
            child.mkdir()
        current = child


def _lowering_source_limit(max_source_bytes: int | None) -> int | None:
    if max_source_bytes is None:
        return None
    checked = require_bounded_int(
        max_source_bytes,
        resource="dataset snapshot source bytes",
        minimum=1,
        maximum=MAX_EXTERNAL_FILE_BYTES,
    )
    if checked >= MAX_EXTERNAL_FILE_BYTES:
        return None
    return checked


def _source_copy_limit(
    source_limit: int | None,
    *,
    manifest_bytes: int,
    manifest_path: Path,
) -> int | None:
    if source_limit is None:
        return None
    remaining = MAX_PUBLICATION_DIRECTORY_BYTES - manifest_bytes
    if remaining < 1:
        raise PublicationResourceLimitError(
            manifest_path,
            limit=MAX_PUBLICATION_DIRECTORY_BYTES,
            observed=manifest_bytes,
        )
    return min(source_limit, remaining)


def open_dataset_snapshot(
    manifest_or_path: DatasetManifest | str | Path,
    *,
    max_source_bytes: int | None = None,
    json_budget: JsonDecodeBudget | None = None,
) -> DatasetSnapshot:
    """Resolve a manifest and bind consumers to a verified dataset snapshot.

    ``max_source_bytes`` can only lower the external-file ceiling; manifest
    bytes count separately within the publication ceiling. Legacy inputs retain
    best-effort snapshot semantics with bounded inventory and reads.
    """

    source_limit = _lowering_source_limit(max_source_bytes)
    if json_budget is None:
        decoder = JsonDecodeBudget()
    elif isinstance(json_budget, JsonDecodeBudget):
        decoder = json_budget
    else:
        raise TypeError("json_budget must be a JsonDecodeBudget or None")

    if isinstance(manifest_or_path, DatasetManifest):
        return _open_manifest_value_snapshot(
            manifest_or_path,
            max_source_bytes=source_limit,
        )

    manifest_path = find_dataset_manifest(manifest_or_path)
    published = _open_dataset_publication_for_path(manifest_path)
    if published is None:
        return _open_legacy_dataset_snapshot(
            manifest_path,
            max_source_bytes=source_limit,
            json_budget=decoder,
        )

    try:
        relative_manifest = (
            manifest_path.resolve().relative_to(published.path.resolve()).as_posix()
        )
    except ValueError as exc:
        raise PublicationError(
            f"dataset manifest is outside its published root: {manifest_path}"
        ) from exc
    if relative_manifest not in published.member_hashes:
        raise PublicationError(
            f"published dataset manifest is not a manifested member: {manifest_path}"
        )
    if source_limit is not None:
        return _open_published_dataset_snapshot(
            published,
            manifest_path=manifest_path,
            relative_manifest=relative_manifest,
            max_source_bytes=source_limit,
            json_budget=decoder,
        )

    materialized = published.materialize_snapshot()
    ownership_transferred = False
    try:
        snapshot_manifest_path = materialized.path / relative_manifest
        manifest = DatasetManifest.from_dict(
            decoder.loads(
                materialized.read_member_bytes(relative_manifest).decode("utf-8")
            )
        )
        dataset_path = _resolve_published_dataset_path(
            manifest=manifest,
            manifest_relative_path=Path(relative_manifest),
            materialized=materialized,
        )
        snapshot = DatasetSnapshot(
            manifest_path=snapshot_manifest_path,
            manifest=manifest,
            dataset_path=dataset_path,
            member_hashes=published.member_hashes,
            source_manifest_path=manifest_path,
            source_dataset_path=resolve_dataset_path(
                manifest.dataset_path, manifest_path
            ),
            _member_identities=MappingProxyType(
                {
                    materialized.path / member: (member, digest)
                    for member, digest in published.member_hashes.items()
                }
            ),
            _materialized=materialized,
        )
        ownership_transferred = True
        return snapshot
    finally:
        if not ownership_transferred:
            startup_error = sys.exc_info()[1]
            _, cleanup_error = attempt_cleanup(materialized.close)
            if cleanup_error is not None:
                _, annotation_error = attempt_cleanup(
                    lambda: setattr(cleanup_error, "cleanup_owner", materialized)
                )
                if annotation_error is not None and not isinstance(
                    annotation_error, Exception
                ):
                    raise annotation_error
                if startup_error is not None:
                    raise cleanup_error from startup_error
                raise cleanup_error


def _published_dataset_member_path(
    *,
    manifest: DatasetManifest,
    relative_manifest: str,
) -> str:
    raw_path = Path(manifest.dataset_path)
    if raw_path.is_absolute():
        raise PublicationError(
            "published dataset_path must be relative to the published dataset root"
        )
    dataset_root = Path(relative_manifest).parent
    if dataset_root.name == "meta":
        dataset_root = dataset_root.parent
    candidate = Path(os.path.normpath(os.fspath(dataset_root / raw_path)))
    candidate_text = candidate.as_posix()
    if candidate_text == ".." or candidate_text.startswith("../"):
        raise PublicationError(
            "published dataset_path escapes its materialized snapshot"
        )
    return candidate_text


def _open_published_dataset_snapshot(
    published: PublishedDirectory,
    *,
    manifest_path: Path,
    relative_manifest: str,
    max_source_bytes: int,
    json_budget: JsonDecodeBudget,
) -> DatasetSnapshot:
    """Materialize a published dataset with a lower source-byte allowance."""

    manifest_payload = published.read_member_bytes(relative_manifest)
    try:
        manifest = DatasetManifest.from_dict(
            json_budget.loads(manifest_payload.decode("utf-8"))
        )
    except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
        raise PublicationError(
            f"published dataset manifest is invalid: {manifest_path}"
        ) from exc
    relative_dataset = _published_dataset_member_path(
        manifest=manifest,
        relative_manifest=relative_manifest,
    )
    dataset = Path(relative_dataset)
    prefix = relative_dataset.rstrip("/")
    if manifest.storage_format == StorageFormat.LEROBOT and dataset.name == "data":
        # The LeRobot reader consumes metadata siblings from the format root
        # as well as the ``data`` tree.  Match ``_dataset_snapshot_source``
        # used by local and legacy snapshots when the source is lowered.
        source_root = dataset.parent.as_posix()
        source_prefix = "" if source_root in ("", ".") else source_root + "/"
        data_members = [
            member
            for member in published.member_hashes
            if member != relative_manifest
            and (not source_prefix or member.startswith(source_prefix))
        ]
    elif relative_dataset == ".":
        data_members = [
            member for member in published.member_hashes if member != relative_manifest
        ]
    elif relative_dataset in published.member_hashes:
        data_members = [relative_dataset]
    else:
        data_members = [
            member
            for member in published.member_hashes
            if member.startswith(prefix + "/")
        ]
    if not data_members:
        raise PublicationError(
            f"published dataset_path has no manifested members: {relative_dataset}"
        )

    source_copy_limit = _source_copy_limit(
        max_source_bytes,
        manifest_bytes=len(manifest_payload),
        manifest_path=manifest_path,
    )
    assert source_copy_limit is not None
    data_files = {
        member: (published.path / member, published.member_hashes[member])
        for member in data_members
    }
    manifest_parts = len(Path(relative_manifest).parts)
    data_entry_limit = MAX_PUBLICATION_DIRECTORY_ENTRIES - manifest_parts
    if data_entry_limit < 1:
        raise PublicationResourceLimitError(
            manifest_path,
            limit=MAX_PUBLICATION_DIRECTORY_ENTRIES,
            observed=MAX_PUBLICATION_DIRECTORY_ENTRIES + 1,
        )
    with ExitStack() as cleanup:
        materialized = cleanup.enter_context(
            materialize_file_snapshot(
                data_files,
                max_bytes=source_copy_limit,
                max_entries=data_entry_limit,
                byte_budget=ByteBudget(
                    "published dataset source copy bytes", source_copy_limit
                ),
            )
        )
        manifest_materialized = cleanup.enter_context(
            materialize_file_snapshot(
                {
                    relative_manifest: (
                        published.path / relative_manifest,
                        published.member_hashes[relative_manifest],
                    )
                },
                max_bytes=len(manifest_payload),
                max_entries=manifest_parts,
                byte_budget=ByteBudget(
                    "published dataset manifest copy bytes", len(manifest_payload)
                ),
            )
        )
        dataset_path = _resolve_published_dataset_path(
            manifest=manifest,
            manifest_relative_path=Path(relative_manifest),
            materialized=materialized,
        )
        snapshot_manifest_path = manifest_materialized.path / relative_manifest
        identities = {
            materialized.path / member: (member, published.member_hashes[member])
            for member in data_members
        }
        identities[snapshot_manifest_path] = (
            relative_manifest,
            published.member_hashes[relative_manifest],
        )
        snapshot = DatasetSnapshot(
            manifest_path=snapshot_manifest_path,
            manifest=manifest,
            dataset_path=dataset_path,
            member_hashes=published.member_hashes,
            source_manifest_path=manifest_path,
            source_dataset_path=resolve_dataset_path(
                manifest.dataset_path, manifest_path
            ),
            _member_identities=MappingProxyType(identities),
            _materialized=materialized,
            _manifest_materialized=manifest_materialized,
        )
        cleanup.pop_all()
        return snapshot


def _dataset_snapshot_source(manifest: DatasetManifest, dataset_path: Path) -> Path:
    # LeRobot's data alias also requires sibling metadata in the format root.
    if (
        manifest.storage_format == StorageFormat.LEROBOT
        and dataset_path.name == "data"
        and dataset_path.parent.is_dir()
    ):
        return dataset_path.parent
    return dataset_path


def _open_manifest_value_snapshot(
    manifest: DatasetManifest,
    *,
    max_source_bytes: int | None = None,
) -> DatasetSnapshot:
    dataset_path = resolve_dataset_path(manifest.dataset_path, None)
    source = _dataset_snapshot_source(manifest, dataset_path)
    data_is_directory = source.is_dir()
    data_root = source if data_is_directory else source.parent
    source_limit = _lowering_source_limit(max_source_bytes)
    source_hash_budget = (
        ByteBudget("manifest value source hash bytes", source_limit)
        if source_limit is not None
        else None
    )
    if data_is_directory:
        hashes = directory_member_hashes(source, byte_budget=source_hash_budget)
    elif source.exists() or source.is_symlink():
        hashes = {source.name: sha256_file(source, byte_budget=source_hash_budget)}
    else:
        hashes = {}
    source_copy_budget = (
        ByteBudget("manifest value source copy bytes", source_limit)
        if source_limit is not None
        else None
    )
    with ExitStack() as cleanup:
        materialized = cleanup.enter_context(
            materialize_file_snapshot(
                {
                    member: (data_root / member, digest)
                    for member, digest in hashes.items()
                },
                max_bytes=source_limit,
                byte_budget=source_copy_budget,
            )
        )
        snapshot = DatasetSnapshot(
            manifest_path=None,
            manifest=manifest,
            dataset_path=materialized.path / dataset_path.relative_to(data_root),
            member_hashes=hashes,
            source_dataset_path=dataset_path,
            _member_identities=MappingProxyType(
                {
                    materialized.path / member: (member, digest)
                    for member, digest in hashes.items()
                }
            ),
            _materialized=materialized,
        )
        cleanup.pop_all()
        return snapshot


def _open_legacy_dataset_snapshot(
    manifest_path: Path,
    *,
    max_source_bytes: int | None = None,
    json_budget: JsonDecodeBudget | None = None,
) -> DatasetSnapshot:
    payload = ArtifactPublication.file(manifest_path).read(lambda raw: raw)
    if payload is None:
        raise FileNotFoundError(manifest_path)
    if json_budget is None:
        decoder = JsonDecodeBudget()
    elif isinstance(json_budget, JsonDecodeBudget):
        decoder = json_budget
    else:
        raise TypeError("json_budget must be a JsonDecodeBudget or None")
    manifest = DatasetManifest.from_dict(decoder.loads(payload))
    dataset_path = resolve_dataset_path(manifest.dataset_path, manifest_path)
    source = _dataset_snapshot_source(manifest, dataset_path)
    data_is_directory = source.is_dir()
    source_limit = _lowering_source_limit(max_source_bytes)
    source_copy_limit = _source_copy_limit(
        source_limit,
        manifest_bytes=len(payload),
        manifest_path=manifest_path,
    )
    source_hash_budget = (
        ByteBudget("legacy dataset source hash bytes", source_copy_limit)
        if source_copy_limit is not None
        else None
    )
    member_hashes = _dataset_member_hashes(
        manifest_path=manifest_path,
        dataset_path=source,
        source_byte_budget=source_hash_budget,
    )
    root = _absolute_path(dataset_root_from_manifest_path(manifest_path))
    relative_manifest = _absolute_path(manifest_path).relative_to(root).as_posix()
    # Resolution and the copied manifest must describe the same admitted bytes.
    # A second inventory read must not silently replace this expected digest.
    manifest_digest = hashlib.sha256(payload).hexdigest()
    member_hashes[relative_manifest] = manifest_digest
    source_data = _absolute_path(source)
    source_manifest = _absolute_path(manifest_path)
    data_root = source_data if data_is_directory else source_data.parent
    files: dict[str, tuple[Path, str]] = {}
    identities: dict[str, tuple[str, str]] = {}
    copied_manifest: str | None = None
    separate_manifest = source_limit is not None
    for member, digest in member_hashes.items():
        source = _absolute_path(root / member)
        if data_is_directory:
            if not source.is_relative_to(data_root):
                continue
        elif source != source_data:
            continue
        if separate_manifest and source == source_manifest:
            continue
        destination = source.relative_to(data_root).as_posix()
        if source == source_manifest:
            copied_manifest = destination
            digest = manifest_digest
            member_hashes[member] = digest
        files[destination] = (source, digest)
        identities[destination] = (member, digest)
    budget = (
        ByteBudget("legacy dataset copy bytes", MAX_PUBLICATION_DIRECTORY_BYTES)
        if source_copy_limit is None
        else ByteBudget("legacy dataset source copy bytes", source_copy_limit)
    )
    with ExitStack() as cleanup:
        materialized = cleanup.enter_context(
            materialize_file_snapshot(
                files,
                max_bytes=(
                    MAX_PUBLICATION_DIRECTORY_BYTES
                    if source_copy_limit is None
                    else source_copy_limit
                ),
                max_entries=(
                    MAX_PUBLICATION_DIRECTORY_ENTRIES - 1
                    if separate_manifest
                    else MAX_PUBLICATION_DIRECTORY_ENTRIES
                    if copied_manifest is not None
                    else MAX_PUBLICATION_DIRECTORY_ENTRIES - 1
                ),
                byte_budget=budget,
            )
        )
        manifest_materialized = None
        owned_identities = {
            materialized.path / member: identity
            for member, identity in identities.items()
        }
        if separate_manifest:
            manifest_bytes = len(payload)
            manifest_materialized = cleanup.enter_context(
                materialize_file_snapshot(
                    {manifest_path.name: (source_manifest, manifest_digest)},
                    max_bytes=manifest_bytes,
                    max_entries=1,
                    byte_budget=ByteBudget(
                        "legacy dataset manifest copy bytes", manifest_bytes
                    ),
                )
            )
            owned_manifest_path = manifest_materialized.path / manifest_path.name
        elif copied_manifest is None:
            # Separate backing storage keeps reader-owned metadata out of the
            # data tree without charging a synthetic directory level to data.
            manifest_materialized = cleanup.enter_context(
                materialize_file_snapshot(
                    {manifest_path.name: (source_manifest, manifest_digest)},
                    max_bytes=MAX_PUBLICATION_DIRECTORY_BYTES,
                    max_entries=1,
                    byte_budget=budget,
                )
            )
            owned_manifest_path = manifest_materialized.path / manifest_path.name
        else:
            owned_manifest_path = materialized.path / copied_manifest
        owned_identities[owned_manifest_path] = (relative_manifest, manifest_digest)
        snapshot = DatasetSnapshot(
            manifest_path=owned_manifest_path,
            manifest=manifest,
            dataset_path=(
                materialized.path / _absolute_path(dataset_path).relative_to(data_root)
            ),
            member_hashes=member_hashes,
            source_manifest_path=manifest_path,
            source_dataset_path=dataset_path,
            _member_identities=MappingProxyType(owned_identities),
            _materialized=materialized,
            _manifest_materialized=manifest_materialized,
        )
        cleanup.pop_all()
        return snapshot


def _resolve_published_dataset_path(
    *,
    manifest: DatasetManifest,
    manifest_relative_path: Path,
    materialized: MaterializedDirectorySnapshot,
) -> Path:
    """Resolve one published dataset path inside its verified snapshot only."""

    raw_path = Path(manifest.dataset_path)
    if raw_path.is_absolute():
        raise PublicationError(
            "published dataset_path must be relative to the published dataset root"
        )

    dataset_root = manifest_relative_path.parent
    if dataset_root.name == "meta":
        dataset_root = dataset_root.parent
    materialized_root = materialized.path.resolve()
    candidate = (materialized.path / dataset_root / raw_path).resolve()
    try:
        relative_dataset = candidate.relative_to(materialized_root).as_posix()
    except ValueError as exc:
        raise PublicationError(
            "published dataset_path escapes its materialized snapshot"
        ) from exc

    members = materialized.member_hashes
    if candidate.is_file():
        if relative_dataset not in members:
            raise PublicationError(
                f"published dataset_path is not a manifested member: {relative_dataset}"
            )
    elif candidate.is_dir():
        prefix = relative_dataset.rstrip("/")
        if not any(
            member == prefix or member.startswith(prefix + "/") for member in members
        ):
            raise PublicationError(
                f"published dataset_path has no manifested members: {relative_dataset}"
            )
    else:
        raise PublicationError(
            "published dataset_path does not resolve to a manifested file or "
            f"directory: {relative_dataset}"
        )
    return candidate


def _validate_dataset_snapshot(
    snapshot: DatasetSnapshot,
    *,
    deep: bool,
    strict: bool,
) -> DatasetValidationReport:
    """Validate one already-resolved dataset snapshot."""

    manifest = snapshot.manifest
    errors: list[str] = []
    warnings: list[str] = []
    member_hashes: Mapping[str, str] = snapshot.member_hashes

    _validate_manifest_contract(manifest, errors=errors, warnings=warnings)

    dataset_path = snapshot.dataset_path
    if not dataset_path.exists():
        errors.append(
            f"dataset_path does not exist: {snapshot.source_dataset_path or dataset_path}"
        )
    else:
        if deep:
            _validate_dataset_contents(
                manifest=manifest,
                dataset_path=dataset_path,
                errors=errors,
                warnings=warnings,
            )
        else:
            _validate_dataset_counts(
                manifest=manifest,
                dataset_path=dataset_path,
                errors=errors,
                warnings=warnings,
            )

    identity = manifest.identity
    canonical_repo_id = identity.dataset_repo_id()
    # Earlier collections prefixed the full task slug, so ``srb/<task>`` gave
    # ``srb_srb_<task>``; that id was canonical when written and names the repo.
    legacy_repo_id = (
        f"{canonical_repo_id.split('/', 1)[0]}/srb_{identity.task}_"
        f"{identity.embodiment}_{identity.action_mode}"
    )
    if manifest.hf_repo_id not in (canonical_repo_id, legacy_repo_id):
        warnings.append(
            "hf_repo_id does not match canonical identity-derived dataset repo "
            f"{canonical_repo_id!r}"
        )

    if strict:
        errors.extend(f"warning treated as error: {warning}" for warning in warnings)

    return DatasetValidationReport(
        valid=not errors,
        errors=errors,
        warnings=warnings,
        manifest=manifest,
        member_hashes=member_hashes,
    )


def validate_dataset_layout(
    manifest_or_path: DatasetManifest | str | Path,
    *,
    deep: bool = False,
    strict: bool = False,
) -> DatasetValidationReport:
    """Validate local manifest fields and referenced data files.

    Incomplete native metadata cleanup propagates its explicit retry owner;
    reducing that failure to a report would discard cleanup authority.
    """
    from srb.datasets.hdf5_metadata import Hdf5MetadataCleanupError
    from srb.datasets.hdf5_native import Hdf5NativeCleanupError

    try:
        with open_dataset_snapshot(manifest_or_path) as snapshot:
            return _validate_dataset_snapshot(snapshot, deep=deep, strict=strict)
    except (Hdf5MetadataCleanupError, Hdf5NativeCleanupError):
        raise
    except Exception as exc:
        return DatasetValidationReport(valid=False, errors=[str(exc)])


def _validate_manifest_contract(
    manifest: DatasetManifest,
    *,
    errors: list[str],
    warnings: list[str],
) -> None:
    metadata = thaw_json_value(cast(FrozenJSONObject, manifest.metadata))
    if not isinstance(metadata, dict):
        raise TypeError("dataset manifest metadata must be a JSON object")

    controller = str(manifest.controller).strip()
    allowed_controllers = sorted(controller.value for controller in DatasetController)
    if not controller:
        errors.append("controller must not be empty")
    elif controller not in allowed_controllers:
        errors.append("controller must be one of: " + ", ".join(allowed_controllers))
    elif manifest.split != DatasetSplit.REAL and controller != manifest.split.value:
        errors.append(
            f"split {manifest.split.value} must use controller "
            f"{manifest.split.value}; use split real for real validation data "
            "with a different controller"
        )

    for metadata_key in ("task", "embodiment", "action_mode"):
        _validate_identity_metadata_field(
            metadata=metadata,
            metadata_key=metadata_key,
            expected=str(getattr(manifest.identity, metadata_key)),
            errors=errors,
        )
    _validate_action_schema_metadata(metadata, errors=errors)

    if (
        manifest.split == DatasetSplit.POLICY
        and manifest.identity.algo is None
        and not any(
            key in metadata
            for key in ("algo", "checkpoint", "checkpoint_path", "model_repo_id")
        )
    ):
        warnings.append(
            "policy dataset does not identify the generating algorithm or checkpoint"
        )


def _validate_action_schema_metadata(
    metadata: Mapping[str, JSONValue],
    *,
    errors: list[str],
) -> None:
    schema = metadata.get("action_schema")
    if schema is None:
        return
    if not isinstance(schema, list) or not schema:
        errors.append("metadata action_schema must be a non-empty list")
        return

    next_start = 0
    total_dim = 0
    for index, component in enumerate(schema):
        label = f"metadata action_schema[{index}]"
        if not isinstance(component, dict):
            errors.append(f"{label} must be an object")
            continue

        name = component.get("name")
        if not isinstance(name, str) or not name:
            errors.append(f"{label}.name must be a non-empty string")

        path = component.get("path")
        if not isinstance(path, list):
            errors.append(f"{label}.path must be a list")

        dim = component.get("dim")
        start = component.get("start")
        end = component.get("end")
        if not _is_positive_int_value(dim):
            errors.append(f"{label}.dim must be a positive integer")
            continue
        start_value: int | None = None
        if not _is_int_value(start):
            errors.append(f"{label}.start must be an integer")
        else:
            start_value = start
        if start_value is not None and start_value != next_start:
            errors.append(f"{label}.start={start} does not match expected {next_start}")

        end_value: int | None = None
        if not _is_int_value(end):
            errors.append(f"{label}.end must be an integer")
        else:
            end_value = end
        if (
            start_value is not None
            and end_value is not None
            and end_value - start_value != dim
        ):
            errors.append(f"{label}.end-start must match dim {dim}")

        shape = component.get("shape")
        shape_dim = _metadata_shape_dim(shape)
        if shape_dim is None:
            errors.append(f"{label}.shape must be a list of positive integers")
        elif shape_dim != dim:
            errors.append(f"{label}.shape product {shape_dim} does not match dim {dim}")

        if end_value is not None:
            next_start = end_value
        else:
            next_start += dim
        total_dim += dim

    action_dim = metadata.get("action_dim")
    if action_dim is None:
        return
    if not _is_int_value(action_dim):
        errors.append("metadata action_dim must be an integer")
    elif total_dim != int(action_dim):
        errors.append(
            f"metadata action_schema dim sum {total_dim} does not match "
            f"metadata action_dim {action_dim}"
        )


def _validate_identity_metadata_field(
    *,
    metadata: Mapping[str, JSONValue],
    metadata_key: str,
    expected: str,
    errors: list[str],
) -> None:
    if metadata_key not in metadata:
        return
    raw_value = metadata[metadata_key]
    if not isinstance(raw_value, str):
        errors.append(f"metadata {metadata_key} must be a string")
        return
    try:
        normalized = slugify_underscore(raw_value)
    except ValueError:
        errors.append(f"metadata {metadata_key} must not be empty")
        return
    if normalized != expected:
        errors.append(
            f"metadata {metadata_key}={raw_value} does not match "
            f"identity {metadata_key}={expected}"
        )


def _validate_dataset_contents(
    *,
    manifest: DatasetManifest,
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
) -> None:
    get_dataset_format_adapter(manifest.storage_format).validate_contents(
        manifest=manifest,
        dataset_path=dataset_path,
        errors=errors,
        warnings=warnings,
    )


def _dataset_member_hashes(
    *,
    manifest_path: Path,
    dataset_path: Path,
    source_byte_budget: ByteBudget | None = None,
) -> dict[str, str]:
    """Bound legacy inventory work without claiming immutable consumption."""

    root = dataset_root_from_manifest_path(manifest_path)
    external_reference = not _absolute_path(dataset_path).is_relative_to(
        _absolute_path(root)
    )
    if manifest_path.name.endswith(".srb.json") or external_reference:
        budget = ByteBudget("legacy dataset bytes", MAX_PUBLICATION_DIRECTORY_BYTES)
        source_budget = source_byte_budget or budget
        if dataset_path.is_dir():
            manifest_is_embedded = _absolute_path(manifest_path).is_relative_to(
                _absolute_path(dataset_path)
            )
            hashes = {}
            if not manifest_is_embedded:
                hashes[manifest_path.relative_to(root).as_posix()] = sha256_file(
                    manifest_path, byte_budget=budget
                )
            target_prefix = Path(os.path.relpath(dataset_path, root))
            hashes.update(
                {
                    (target_prefix / name).as_posix(): digest
                    for name, digest in directory_member_hashes(
                        dataset_path,
                        max_entries=(
                            MAX_PUBLICATION_DIRECTORY_ENTRIES
                            if manifest_is_embedded
                            else MAX_PUBLICATION_DIRECTORY_ENTRIES - 1
                        ),
                        byte_budget=source_budget,
                    ).items()
                }
            )
            return hashes
        paths = [manifest_path]
        if dataset_path.exists() or dataset_path.is_symlink():
            paths.append(dataset_path)
        hashes = {
            Path(os.path.relpath(manifest_path, root)).as_posix(): sha256_file(
                manifest_path,
                byte_budget=budget,
            )
        }
        if len(paths) > 1:
            hashes[Path(os.path.relpath(dataset_path, root)).as_posix()] = sha256_file(
                dataset_path,
                byte_budget=source_budget,
            )
        return hashes
    if not root.is_dir():
        return {}
    return directory_member_hashes(root, byte_budget=source_byte_budget)


def _validate_dataset_counts(
    *,
    manifest: DatasetManifest,
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
) -> None:
    get_dataset_format_adapter(manifest.storage_format).validate_counts(
        manifest=manifest,
        dataset_path=dataset_path,
        errors=errors,
        warnings=warnings,
    )


def _is_int_value(value: object) -> TypeGuard[int]:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_positive_int_value(value: object) -> TypeGuard[int]:
    return _is_int_value(value) and value > 0


def _metadata_shape_dim(value: JSONValue | None) -> int | None:
    if not isinstance(value, list) or not value:
        return None
    dim = 1
    for item in value:
        if not _is_positive_int_value(item):
            return None
        dim *= item
    return dim
