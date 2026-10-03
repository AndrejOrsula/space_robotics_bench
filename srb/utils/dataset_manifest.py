"""Pure SRB dataset manifest contract and publication-aware readers."""

from __future__ import annotations

import hashlib
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import TypedDict, cast

from srb.utils.artifact_identity import (
    ArtifactIdentity,
    ArtifactIdentityDict,
    stable_hash,
)
from srb.utils.dataset_schema import DatasetSplit, StorageFormat
from srb.utils.json_types import (
    FrozenJSONObject,
    JsonDecodeBudget,
    JSONObject,
    JSONValue,
    freeze_json_value,
    thaw_json_value,
    validate_json_object,
)
from srb.utils.publication import (
    ArtifactPublication,
    PublicationError,
    PublishedDirectory,
)

MANIFEST_VERSION = "0.1"
DEFAULT_MANIFEST_RELPATH = Path("meta") / "dataset_manifest.json"
_DIRECTORY_PUBLICATION_MANIFEST = ".srb-directory-publication.json"
_MANIFEST_KEYS = frozenset(
    {
        "format_version",
        "identity",
        "dataset_path",
        "split",
        "controller",
        "storage_format",
        "hf_repo_id",
        "metadata",
    }
)


def demo_content_fingerprint(
    *,
    manifest_fingerprint: str,
    member_fingerprints: Mapping[str, str],
) -> str:
    """Bind validated demo member names and exact manifest/member digests."""
    digest = hashlib.sha256()
    digest.update(b"srb-demo-transition-source/1\0")
    digest.update(manifest_fingerprint.encode("ascii"))
    digest.update(b"\0")
    for member, member_hash in sorted(member_fingerprints.items()):
        digest.update(member.encode("utf-8"))
        digest.update(b"\0")
        digest.update(member_hash.encode("ascii"))
        digest.update(b"\0")
    return digest.hexdigest()


class DatasetManifestDict(TypedDict):
    """Serialized canonical dataset manifest fields."""

    format_version: str
    identity: ArtifactIdentityDict
    dataset_path: str
    split: str
    controller: str
    storage_format: str
    hf_repo_id: str
    metadata: JSONObject


@dataclass(frozen=True, slots=True)
class DatasetManifest:
    """Portable metadata for one SRB dataset artifact."""

    identity: ArtifactIdentity
    dataset_path: str
    split: DatasetSplit
    controller: str
    storage_format: StorageFormat
    hf_repo_id: str
    format_version: str = MANIFEST_VERSION
    metadata: Mapping[str, JSONValue] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if type(self.identity) is not ArtifactIdentity:
            raise TypeError("dataset manifest identity must be an ArtifactIdentity")
        _require_nonempty_string(self.dataset_path, field_name="dataset_path")
        _require_nonempty_string(self.controller, field_name="controller")
        _require_nonempty_string(self.hf_repo_id, field_name="hf_repo_id")
        _require_nonempty_string(self.format_version, field_name="format_version")
        if self.format_version != MANIFEST_VERSION:
            raise ValueError(
                f"unsupported dataset manifest format_version "
                f"{self.format_version!r}; expected {MANIFEST_VERSION!r}"
            )
        if type(self.split) is not DatasetSplit:
            raise TypeError("dataset manifest split must be a DatasetSplit")
        if type(self.storage_format) is not StorageFormat:
            raise TypeError("dataset manifest storage_format must be a StorageFormat")

        raw_metadata: object = self.metadata
        if isinstance(self.metadata, MappingProxyType):
            raw_metadata = thaw_json_value(cast(FrozenJSONObject, self.metadata))
        metadata = validate_json_object(
            raw_metadata,
            context="dataset manifest metadata",
        )
        frozen_metadata = freeze_json_value(metadata)
        if not isinstance(frozen_metadata, Mapping):
            raise TypeError("dataset manifest metadata must be a JSON object")
        object.__setattr__(self, "metadata", frozen_metadata)

    @property
    def artifact_hash(self) -> str:
        digest: str = stable_hash(self.to_dict())
        return digest

    def to_dict(self) -> DatasetManifestDict:
        metadata = thaw_json_value(cast(FrozenJSONObject, self.metadata))
        if not isinstance(metadata, dict):
            raise TypeError("dataset manifest metadata must be a JSON object")
        payload: DatasetManifestDict = {
            "format_version": self.format_version,
            "identity": self.identity.to_dict(),
            "dataset_path": self.dataset_path,
            "split": self.split.value,
            "controller": self.controller,
            "storage_format": self.storage_format.value,
            "hf_repo_id": self.hf_repo_id,
            "metadata": metadata,
        }
        return payload

    @classmethod
    def from_dict(cls, data: Mapping[str, object]) -> "DatasetManifest":
        """Parse a manifest with explicit legacy-default compatibility.

        Canonical manifests contain every field in :class:`DatasetManifestDict`.
        ``metadata`` may be omitted and is normalized to an empty object.
        ``format_version`` is REQUIRED: a manifest that never carried one
        predates versioning and must not be silently read as current. Unknown
        fields and every other omission are rejected.
        """
        payload = validate_json_object(data, context="dataset manifest")
        unknown_fields = set(payload) - _MANIFEST_KEYS
        if unknown_fields:
            names = ", ".join(sorted(unknown_fields))
            raise ValueError(f"dataset manifest has unknown field(s): {names}")

        # Require the version: defaulting would misread legacy manifests as current.
        if "format_version" not in payload:
            raise ValueError(
                "dataset manifest has no 'format_version'; it predates manifest "
                f"versioning and cannot be read as {MANIFEST_VERSION!r}"
            )
        format_version = _required_string(
            payload["format_version"],
            field_name="format_version",
        )
        # Like JSONL rows, a newer manifest must not be read with older rules.
        if format_version != MANIFEST_VERSION:
            raise ValueError(
                f"unsupported dataset manifest format_version {format_version!r}; "
                f"expected {MANIFEST_VERSION!r}"
            )
        raw_identity = _required_object(payload, "identity")
        dataset_path = _required_string(
            _required_value(payload, "dataset_path"),
            field_name="dataset_path",
        )
        split = DatasetSplit(
            _required_string(_required_value(payload, "split"), field_name="split")
        )
        controller = _required_string(
            _required_value(payload, "controller"),
            field_name="controller",
        )
        storage_format = StorageFormat(
            _required_string(
                _required_value(payload, "storage_format"),
                field_name="storage_format",
            )
        )
        hf_repo_id = _required_string(
            _required_value(payload, "hf_repo_id"),
            field_name="hf_repo_id",
        )
        raw_metadata = payload.get("metadata", {})
        metadata = _as_object(raw_metadata, field_name="metadata")
        return cls(
            format_version=format_version,
            identity=ArtifactIdentity.from_dict(raw_identity),
            dataset_path=dataset_path,
            split=split,
            controller=controller,
            storage_format=storage_format,
            hf_repo_id=hf_repo_id,
            metadata=metadata,
        )


def _required_value(payload: JSONObject, key: str) -> JSONValue:
    if key not in payload:
        raise ValueError(f"dataset manifest is missing required field {key!r}")
    return payload[key]


def _require_nonempty_string(value: object, *, field_name: str) -> None:
    if type(value) is not str:
        raise TypeError(
            f"dataset manifest field {field_name!r} must be a string, "
            f"got {type(value).__name__}"
        )
    if not value:
        raise ValueError(f"dataset manifest field {field_name!r} must not be empty")


def _required_string(value: JSONValue, *, field_name: str) -> str:
    if not isinstance(value, str):
        raise TypeError(
            f"dataset manifest field {field_name!r} must be a string, "
            f"got {type(value).__name__}"
        )
    return value


def _required_object(payload: JSONObject, key: str) -> JSONObject:
    value = _required_value(payload, key)
    if not isinstance(value, dict):
        raise TypeError(
            f"dataset manifest field {key!r} must be an object, "
            f"got {type(value).__name__}"
        )
    return value


def _as_object(value: JSONValue, *, field_name: str) -> JSONObject:
    if not isinstance(value, dict):
        raise TypeError(
            f"dataset manifest field {field_name!r} must be an object, "
            f"got {type(value).__name__}"
        )
    return value


def read_dataset_manifest(
    path: str | Path,
    *,
    json_budget: JsonDecodeBudget | None = None,
) -> DatasetManifest:
    """Read a dataset manifest JSON file."""
    if json_budget is None:
        decoder = JsonDecodeBudget()
    elif isinstance(json_budget, JsonDecodeBudget):
        decoder = json_budget
    else:
        raise TypeError("json_budget must be a JsonDecodeBudget or None")
    manifest_path = Path(path)
    publication = _open_dataset_publication_for_path(manifest_path)
    payload: bytes | None
    if publication is not None:
        root = dataset_root_from_manifest_path(manifest_path)
        try:
            relative = manifest_path.resolve().relative_to(root.resolve()).as_posix()
        except ValueError as exc:
            raise PublicationError(
                f"dataset manifest escapes its published root: {manifest_path}"
            ) from exc
        if relative not in publication.member_hashes:
            raise PublicationError(
                f"published dataset manifest is not a manifested member: {manifest_path}"
            )
        payload = publication.read_member_bytes(relative)
    else:
        payload = ArtifactPublication.file(manifest_path).read(lambda raw: raw)
        if payload is None:
            raise FileNotFoundError(manifest_path)
    if payload is None:
        raise FileNotFoundError(manifest_path)
    return DatasetManifest.from_dict(decoder.loads(payload))


def find_dataset_manifest(path: str | Path) -> Path:
    """Resolve a dataset path, manifest path, directory, or HDF5 sidecar."""
    candidate = Path(path)
    if candidate.is_dir():
        _open_dataset_publication_for_path(candidate)
        for relpath in (DEFAULT_MANIFEST_RELPATH, Path("dataset_manifest.json")):
            manifest_path = candidate / relpath
            if manifest_path.exists():
                return manifest_path
    if candidate.exists() and candidate.suffix == ".json":
        _open_dataset_publication_for_path(candidate)
        return candidate
    sidecar_path = candidate.with_suffix(".srb.json")
    if sidecar_path.exists():
        _open_dataset_publication_for_path(sidecar_path)
        return sidecar_path
    for root in _candidate_manifest_roots(candidate):
        _open_dataset_publication_for_path(root)
        for relpath in (DEFAULT_MANIFEST_RELPATH, Path("dataset_manifest.json")):
            manifest_path = root / relpath
            if manifest_path.exists():
                return manifest_path
    raise FileNotFoundError(f"dataset manifest not found for: {candidate}")


def _dataset_publication_marked(path: str | Path) -> bool:
    """Return whether a directory has publication state requiring validation."""

    return _publication_root_for_path(Path(path)) is not None


def _publication_root_for_path(path: Path) -> Path | None:
    """Find the nearest marked publication root for one dataset path."""

    candidates = [dataset_root_from_manifest_path(path)]
    if path.is_dir():
        candidates.insert(0, path)
    candidates.extend(path.parents)
    seen: set[Path] = set()
    for root in candidates:
        root = root.absolute()
        if root in seen:
            continue
        seen.add(root)
        if _publication_markers_exist(root):
            return root
    return None


def _publication_markers_exist(root: Path) -> bool:
    """Check both publication markers without treating a broken link as absent."""

    if not root.name:
        return False
    internal = root / _DIRECTORY_PUBLICATION_MANIFEST
    mirror = root.with_name(f".{root.name}.publication.json")
    return any(
        candidate.exists() or candidate.is_symlink() for candidate in (internal, mirror)
    )


def _open_dataset_publication_for_path(
    path: str | Path,
) -> PublishedDirectory | None:
    """Open a marked dataset directory, preserving explicit legacy layouts."""

    root = _publication_root_for_path(Path(path))
    if root is None:
        return None
    published = ArtifactPublication.directory(root).open()
    if not isinstance(published, PublishedDirectory):
        raise PublicationError(f"dataset publication is not a directory: {root}")
    return published


def dataset_root_from_manifest_path(manifest_path: str | Path) -> Path:
    """Resolve the dataset root for a dataset manifest path."""
    parent = Path(manifest_path).parent
    return parent.parent if parent.name == "meta" else parent


def resolve_dataset_path(
    dataset_path: str | Path,
    manifest_path: str | Path | None,
) -> Path:
    """Resolve a manifest dataset path against its dataset root."""
    path = Path(dataset_path)
    if path.is_absolute() or manifest_path is None:
        return path
    return dataset_root_from_manifest_path(manifest_path) / path


def _candidate_manifest_roots(candidate: Path) -> list[Path]:
    roots: list[Path] = []
    if candidate.parent.name == "data":
        roots.append(candidate.parent.parent)
    roots.extend(candidate.parents)

    unique_roots: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        if root in seen:
            continue
        seen.add(root)
        unique_roots.append(root)
    return unique_roots


__all__ = [
    "DEFAULT_MANIFEST_RELPATH",
    "MANIFEST_VERSION",
    "DatasetManifest",
    "DatasetManifestDict",
    "dataset_root_from_manifest_path",
    "demo_content_fingerprint",
    "find_dataset_manifest",
    "read_dataset_manifest",
    "resolve_dataset_path",
]
