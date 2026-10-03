"""Model artifact metadata and validation for SRB checkpoints."""

from __future__ import annotations

import json
import os
import re
import stat
import tempfile
from collections.abc import Callable, Iterable, Mapping
from contextlib import suppress
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TypeGuard

from srb.utils.artifact_identity import ArtifactIdentity, stable_hash
from srb.utils.publication import (
    ArtifactPublication,
    MaterializedDirectorySnapshot,
    PublicationResourceLimitError,
    PublishedDirectory,
    _copy_regular_file_to_snapshot,
    sha256_file,
)
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES,
    MAX_PUBLICATION_DIRECTORY_DEPTH,
    MAX_PUBLICATION_DIRECTORY_ENTRIES,
    ByteBudget,
    ResourceLimitError,
)

MODEL_MANIFEST_NAME = "model_manifest.json"
ARTIFACT_MEMBERS_NAME = "artifact_members.json"
ARTIFACT_MEMBERS_RELATIVE_PATH = f"meta/{ARTIFACT_MEMBERS_NAME}"
ARTIFACT_MEMBERS_SCHEMA = "srb-model-artifact-members/1"
ARTIFACT_MEMBERS_SCHEMA_VERSION = 1
MODEL_MANIFEST_CURRENT_SCHEMA_VERSION = 3
MODEL_MANIFEST_RELATIVE_PATH = f"meta/{MODEL_MANIFEST_NAME}"
_TRANSPORT_DIRECTORY_MANIFEST = ".srb-directory-publication.json"
_HF_REPO_ID_PART_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,95}")

#: V1 preserves the payload/hash; v2 adds checkpoint kind/scope; v3 adds layout.
MODEL_MANIFEST_SCHEMA_VERSIONS = (1, 2, MODEL_MANIFEST_CURRENT_SCHEMA_VERSION)
CHECKPOINT_KINDS = ("file", "directory")
CHECKPOINT_SCOPES = ("unspecified", "weights", "agent_evaluation")
RL_ZOO_NORMALIZATION_METADATA_KEY = "rl_zoo_normalization"
RL_ZOO_NORMALIZATION_NONE = {"kind": "none"}
RL_ZOO_NORMALIZATION_VECNORMALIZE_PATH = "normalization/vecnormalize.pkl"
RL_ZOO_NORMALIZATION_VECNORMALIZE = {
    "kind": "vecnormalize",
    "path": RL_ZOO_NORMALIZATION_VECNORMALIZE_PATH,
}
#: Conventional framework config locations, in snapshot lookup order.
_CONFIG_CANDIDATES = (
    "config.yaml",
    "config.yml",
    ".hydra/config.yaml",
    "checkpoints/config.yaml",
    "meta/config.yaml",
)


def _is_int(value: object) -> TypeGuard[int]:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_sha256_text(value: object) -> TypeGuard[str]:
    return isinstance(value, str) and re.fullmatch("[0-9a-f]{64}", value) is not None


def normalize_artifact_relpath(
    value: str | Path, *, field_name: str, root_name: str = "artifact root"
) -> str:
    """Normalize a path that must stay inside an artifact root."""
    raw_value = str(value)
    if "\x00" in raw_value:
        raise ValueError(f"{field_name} must not contain NUL bytes")
    if not raw_value.strip():
        raise ValueError(f"{field_name} must not be empty")
    relpath = Path(raw_value.strip())
    if relpath.is_absolute():
        raise ValueError(f"{field_name} must be relative to the {root_name}")
    if ".." in relpath.parts:
        raise ValueError(f"{field_name} must not escape the {root_name}")
    return relpath.as_posix()


def artifact_member_path(
    root: str | Path,
    relpath: str | Path,
    *,
    field_name: str,
    root_name: str = "artifact root",
) -> Path:
    """Resolve a normalized artifact member path below *root*."""
    root_path = Path(root)
    member = root_path / normalize_artifact_relpath(
        relpath, field_name=field_name, root_name=root_name
    )
    if not member.resolve().is_relative_to(root_path.resolve()):
        raise ValueError(f"{field_name} must not escape the {root_name}")
    return member


def _model_artifact_lock_path(artifact_root: Path) -> Path:
    """Place publication lock metadata beside the artifact root."""
    root = Path(artifact_root).expanduser().resolve()
    return root.parent / f".{root.name}.publication.lock"


@dataclass(frozen=True)
class ModelArtifactManifest:
    identity: ArtifactIdentity
    checkpoint_path: str
    framework: str
    dataset_repo_id: str | None = None
    dataset_revision: str | None = None
    metrics: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
    hf_repo_id: str | None = None
    schema_version: int = 1
    checkpoint_kind: str = "file"
    checkpoint_scope: str = "unspecified"
    artifact_members_path: str = ARTIFACT_MEMBERS_RELATIVE_PATH

    def __post_init__(self) -> None:
        if self.identity.algo is None:
            raise ValueError("model artifact identity requires algo")
        if not isinstance(self.checkpoint_path, (str, Path)):
            raise TypeError("checkpoint_path must be a relative string")
        if not _is_int(self.schema_version):
            raise TypeError("model manifest schema_version must be an integer")
        if not isinstance(self.artifact_members_path, str):
            raise TypeError("artifact_members_path must be a relative string")
        checkpoint_path = normalize_artifact_relpath(
            self.checkpoint_path, field_name="checkpoint_path"
        )
        object.__setattr__(self, "checkpoint_path", checkpoint_path)
        object.__setattr__(self, "framework", str(self.framework).strip() or "unknown")
        if self.hf_repo_id is None:
            object.__setattr__(self, "hf_repo_id", self.identity.model_repo_id())
        if self.schema_version not in MODEL_MANIFEST_SCHEMA_VERSIONS:
            raise ValueError(
                f"unsupported model manifest schema_version: {self.schema_version!r}"
            )
        if self.checkpoint_kind not in CHECKPOINT_KINDS:
            raise ValueError(
                f"unsupported checkpoint_kind {self.checkpoint_kind!r}; "
                f"use one of {CHECKPOINT_KINDS!r}"
            )
        if self.checkpoint_scope not in CHECKPOINT_SCOPES:
            raise ValueError(
                f"unsupported checkpoint_scope {self.checkpoint_scope!r}; "
                f"use one of {CHECKPOINT_SCOPES!r}"
            )
        if self.schema_version < 2 and (
            self.checkpoint_kind != "file" or self.checkpoint_scope != "unspecified"
        ):
            raise ValueError(
                "schema_version 1 manifests are implicitly file/unspecified; "
                "explicit checkpoint_kind/checkpoint_scope requires "
                "schema_version 2"
            )
        members_path = normalize_artifact_relpath(
            self.artifact_members_path, field_name="artifact_members_path"
        )
        if (
            self.schema_version >= MODEL_MANIFEST_CURRENT_SCHEMA_VERSION
            and members_path != ARTIFACT_MEMBERS_RELATIVE_PATH
        ):
            raise ValueError(
                "schema_version 3 artifacts must use "
                f"{ARTIFACT_MEMBERS_RELATIVE_PATH!r} for artifact_members_path"
            )
        object.__setattr__(self, "artifact_members_path", members_path)

    @property
    def artifact_hash(self) -> str:
        return stable_hash(self._payload_dict())

    def to_dict(self) -> dict[str, Any]:
        return {**self._payload_dict(), "artifact_hash": self.artifact_hash}

    def _payload_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "identity": self.identity.to_dict(),
            "checkpoint_path": self.checkpoint_path,
            "framework": self.framework,
            "dataset_repo_id": self.dataset_repo_id,
            "dataset_revision": self.dataset_revision,
            "metrics": dict(self.metrics),
            "metadata": dict(self.metadata),
            "hf_repo_id": self.hf_repo_id,
        }
        # Version-1 manifests keep their original payload (and hash) intact.
        if self.schema_version >= 2:
            data["schema_version"] = self.schema_version
            data["checkpoint_kind"] = self.checkpoint_kind
            data["checkpoint_scope"] = self.checkpoint_scope
        if self.schema_version >= MODEL_MANIFEST_CURRENT_SCHEMA_VERSION:
            data["artifact_members_path"] = self.artifact_members_path
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ModelArtifactManifest:
        schema_version = data.get("schema_version")
        if schema_version is None:
            schema_version = 1
        elif not _is_int(schema_version):
            raise TypeError("model manifest schema_version must be an integer")
        kind: Any = "file"
        scope: Any = "unspecified"
        if schema_version >= 2:
            kind = data.get("checkpoint_kind", "file")
            scope = data.get("checkpoint_scope", "unspecified")
            if not isinstance(kind, str):
                raise TypeError("model manifest checkpoint_kind must be a string")
            if not isinstance(scope, str):
                raise TypeError("model manifest checkpoint_scope must be a string")
        identity = data.get("identity")
        if not isinstance(identity, dict):
            raise TypeError("model manifest identity must be an object")
        checkpoint_path = data.get("checkpoint_path")
        if not isinstance(checkpoint_path, str):
            raise TypeError("model manifest checkpoint_path must be a string")
        framework = data.get("framework", "unknown")
        if framework is not None and not isinstance(framework, str):
            raise TypeError("model manifest framework must be a string")
        metrics = data.get("metrics", {})
        metadata = data.get("metadata", {})
        if not isinstance(metrics, dict) or not isinstance(metadata, dict):
            raise TypeError("model manifest metrics and metadata must be objects")
        for field_name in ("dataset_repo_id", "dataset_revision", "hf_repo_id"):
            value = data.get(field_name)
            if value is not None and not isinstance(value, str):
                raise TypeError(f"model manifest {field_name} must be a string or null")
        members_path = data.get("artifact_members_path", ARTIFACT_MEMBERS_RELATIVE_PATH)
        if not isinstance(members_path, str):
            raise TypeError("model manifest artifact_members_path must be a string")
        return cls(
            identity=ArtifactIdentity.from_dict(identity),
            checkpoint_path=checkpoint_path,
            framework=framework or "unknown",
            dataset_repo_id=data.get("dataset_repo_id"),
            dataset_revision=data.get("dataset_revision"),
            metrics=dict(metrics),
            metadata=dict(metadata),
            hf_repo_id=data.get("hf_repo_id"),
            schema_version=schema_version,
            checkpoint_kind=kind,
            checkpoint_scope=scope,
            artifact_members_path=members_path,
        )


@dataclass(frozen=True)
class ModelArtifactValidationReport:
    valid: bool
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    manifest: ModelArtifactManifest | None = None
    member_paths: tuple[str, ...] = ()
    member_hashes: Mapping[str, str] = field(default_factory=dict)
    checkpoint_sha256: str | None = None
    checkpoint_member_hashes: Mapping[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
        }


@dataclass(frozen=True)
class ModelArtifactReference:
    manifest_path: Path
    artifact_root: Path
    checkpoint_path: Path
    algo: str
    manifest: ModelArtifactManifest
    _checkpoint_snapshot: ModelCheckpointSnapshot | None = field(
        default=None, repr=False, compare=False
    )

    @property
    def load_path(self) -> Path:
        """Return the immutable checkpoint path intended for policy loaders."""
        if self._checkpoint_snapshot is None:
            return self.checkpoint_path
        return self._checkpoint_snapshot.checkpoint_path

    def snapshot_member(self, relative_path: str | Path) -> Path | None:
        """Return a verified snapshot member, or ``None`` if absent/unselected.

        Never fall back to the caller-owned :attr:`artifact_root`.
        """
        normalized = normalize_artifact_relpath(
            relative_path, field_name="snapshot member path"
        )
        snapshot = self._checkpoint_snapshot
        if snapshot is None or normalized not in snapshot.member_paths:
            return None
        member = snapshot.root / normalized
        try:
            regular = not snapshot._closed and stat.S_ISREG(os.lstat(member).st_mode)
        except FileNotFoundError:
            return None
        if not regular or not member.resolve().is_relative_to(snapshot.root.resolve()):
            return None
        return member

    def close(self) -> None:
        if self._checkpoint_snapshot is not None:
            self._checkpoint_snapshot.close()

    def __del__(self) -> None:  # pragma: no cover - interpreter cleanup
        with suppress(Exception):
            self.close()


_SnapshotOwner = tempfile.TemporaryDirectory[str] | MaterializedDirectorySnapshot


def _release_snapshot_owner(owner: _SnapshotOwner) -> None:
    if isinstance(owner, MaterializedDirectorySnapshot):
        owner.close()
    else:
        owner.cleanup()


@dataclass
class ModelCheckpointSnapshot:
    root: Path
    checkpoint_path: Path
    _owner: _SnapshotOwner = field(repr=False)
    member_paths: frozenset[str] = field(default_factory=frozenset)
    _closed: bool = field(default=False, init=False, repr=False)

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            _release_snapshot_owner(self._owner)

    def __del__(self) -> None:  # pragma: no cover - interpreter cleanup
        with suppress(Exception):
            self.close()


@dataclass(frozen=True)
class _LoadedModelArtifactManifest:
    root: Path
    manifest_path: Path
    raw: dict[str, Any]
    manifest: ModelArtifactManifest
    published: PublishedDirectory | None = None
    published_root_identity: tuple[int, int] | None = None
    published_transport_sha256: str | None = None


def build_model_artifact_manifest(
    *,
    identity: ArtifactIdentity,
    checkpoint_path: str | Path,
    framework: str | None = None,
    dataset_repo_id: str | None = None,
    dataset_revision: str | None = None,
    metrics: dict[str, Any] | None = None,
    metadata: dict[str, Any] | None = None,
    hf_repo_id: str | None = None,
    schema_version: int = 1,
    checkpoint_kind: str = "file",
    checkpoint_scope: str = "unspecified",
    artifact_members_path: str = ARTIFACT_MEMBERS_RELATIVE_PATH,
) -> ModelArtifactManifest:
    if identity.algo is None:
        raise ValueError("model artifact identity requires algo")
    return ModelArtifactManifest(
        identity=identity,
        checkpoint_path=str(checkpoint_path),
        framework=framework or "unknown",
        dataset_repo_id=dataset_repo_id,
        dataset_revision=dataset_revision,
        metrics=dict(metrics or {}),
        metadata=dict(metadata or {}),
        hf_repo_id=hf_repo_id,
        schema_version=schema_version,
        checkpoint_kind=checkpoint_kind,
        checkpoint_scope=checkpoint_scope,
        artifact_members_path=artifact_members_path,
    )


# Dreamer (Elements) checkpoint directories. Registered keys accepted as
# source-directory members; adding one requires an explicit versioned review.
_ELEMENTS_SOURCE_KEYS = ("agent", "step", "replay")
_ELEMENTS_DONE_MARKER = "done"
_ELEMENTS_MEMBER_PATTERN = re.compile(r"^(?P<key>[a-z_]+)(?:-(?P<index>\d{4}))?\.pkl$")
_DREAMER_LABEL = "sanitized Dreamer checkpoint directory"


def _elements_agent_members(
    label: str,
    members: Iterable[tuple[str, str]],
    *,
    keys: tuple[str, ...],
    done_is_nonempty: Callable[[], bool],
) -> list[str]:
    """Validate sorted member names and return agent payload names."""
    payload: dict[str, list[int | None]] = {}
    for name, kind in members:
        if kind == "symlink":
            raise ValueError(f"{label} member must not be a symlink: {name}")
        if kind == "directory":
            raise ValueError(f"{label} must not contain nested directories: {name}")
        if kind != "regular":
            raise ValueError(f"{label} member must be a regular file: {name}")
        match = _ELEMENTS_MEMBER_PATTERN.match(name)
        if name == _ELEMENTS_DONE_MARKER:
            payload.setdefault(name, []).append(None)
        elif match is None or match["key"] not in _ELEMENTS_SOURCE_KEYS:
            raise ValueError(f"{label} has unrecognized member: {name}")
        else:
            index = match["index"]
            key_shards = payload.setdefault(match["key"], [])
            key_shards.append(None if index is None else int(index))
    for key in payload:
        if key != _ELEMENTS_DONE_MARKER and key not in keys:
            raise ValueError(f"{label} has unrecognized payload key: {key}")
    if _ELEMENTS_DONE_MARKER not in payload:
        raise ValueError(f"{label} is missing its '{_ELEMENTS_DONE_MARKER}' marker")
    if done_is_nonempty():
        raise ValueError(
            f"{label} '{_ELEMENTS_DONE_MARKER}' marker must be empty "
            "(a nonempty marker indicates an unknown checkpoint format)"
        )
    agent_shards = payload.get("agent")
    if not agent_shards:
        raise ValueError(f"{label} is missing its agent payload")
    if None in agent_shards:
        if len(agent_shards) > 1:
            raise ValueError(
                f"{label} has simultaneous single and sharded agent payloads"
            )
        return ["agent.pkl"]
    shards = sorted(index for index in agent_shards if index is not None)
    if shards != list(range(len(shards))):
        raise ValueError(
            f"{label} agent shards are noncontiguous or duplicated: "
            f"{['agent-%04d.pkl' % index for index in shards]}"
        )
    return [f"agent-{index:04d}.pkl" for index in shards]


def _is_legacy_lock_residue(
    directory: Path, name: str, member_stat: os.stat_result
) -> bool:
    suffix = ".publication.lock"
    if not name.startswith(".") or not name.endswith(suffix):
        return False
    sibling = name[1 : -len(suffix)]
    if not sibling or "/" in sibling or sibling in {".", ".."}:
        return False
    return (
        stat.S_ISREG(member_stat.st_mode)
        and member_stat.st_size == 0
        and os.path.lexists(directory / sibling)
    )


def _agent_payload_members(
    directory: Path, *, label: str, keys: tuple[str, ...] = _ELEMENTS_SOURCE_KEYS
) -> list[str]:
    """Validate Elements members without unpickling them."""
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError(f"{label} must be a real directory: {directory}")

    def members() -> Iterable[tuple[str, str]]:
        for name in sorted(path.name for path in directory.iterdir()):
            entry_stat = os.lstat(directory / name)
            mode = entry_stat.st_mode
            if _is_legacy_lock_residue(directory, name, entry_stat):
                continue
            if stat.S_ISLNK(mode):
                yield name, "symlink"
            elif stat.S_ISDIR(mode):
                yield name, "directory"
            else:
                yield name, "regular" if stat.S_ISREG(mode) else "other"

    return _elements_agent_members(
        label,
        members(),
        keys=keys,
        done_is_nonempty=lambda: (directory / "done").stat().st_size != 0,
    )


def validate_dreamer_checkpoint_directory(path: str | Path) -> list[str]:
    """Validate a Dreamer directory against the exact member allowlist.

    Exactly the empty ``done`` marker plus one agent payload form
    (``agent.pkl`` XOR a contiguous shard set) is accepted. Payloads are never
    unpickled; policy loading remains a trusted-model code-execution boundary.
    """
    members = _agent_payload_members(Path(path), label=_DREAMER_LABEL, keys=("agent",))
    return [_ELEMENTS_DONE_MARKER, *members]


def _validate_dreamer_checkpoint_members_snapshot(
    checkpoint_relpath: str,
    path_kinds: Mapping[str, str],
    published: PublishedDirectory,
) -> list[str]:
    """Validate Dreamer members from the artifact descriptor."""
    prefix = checkpoint_relpath + "/"
    members = [
        # A top-level subdirectory is a non-regular member; deeper paths keep
        # their kind (nested directory) or fail as unrecognized names.
        (name, kind if "/" in name or kind == "regular" else "other")
        for relative, kind in sorted(path_kinds.items())
        if relative.startswith(prefix)
        for name in (relative[len(prefix) :],)
    ]
    return _elements_agent_members(
        _DREAMER_LABEL,
        members,
        keys=("agent",),
        done_is_nonempty=lambda: bool(published.read_member_bytes(prefix + "done")),
    )


def _deterministic_json_bytes(payload: object) -> bytes:
    text = json.dumps(
        payload, allow_nan=False, ensure_ascii=True, indent=2, sort_keys=True
    )
    return (text + "\n").encode("utf-8")


def _stream_byte_budget() -> ByteBudget:
    return ByteBudget("model artifact streamed bytes", MAX_EXTERNAL_FILE_BYTES)


def _artifact_tree_entries(
    root: Path,
    *,
    exclude_relpath: str,
    additional_exclude_relpaths: tuple[str, ...] = (),
    ignore_legacy_lock_residue: bool = False,
    max_depth: int | None = None,
    max_entries: int | None = None,
) -> list[tuple[str, Path, str]]:
    """Enumerate tree members and reject unsafe filesystem entries.

    Rejects symlinks and non-regular members; ``ignore_legacy_lock_residue``
    is reserved for read-only v1/v2 roots.
    """
    root_stat = os.lstat(root)
    if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
        raise ValueError(f"model artifact root must be a real directory: {root}")

    excluded = {exclude_relpath, *additional_exclude_relpaths}
    entries: list[tuple[str, Path, str]] = []
    visited_entries = 0
    byte_budget = ByteBudget("model artifact cumulative bytes", MAX_EXTERNAL_FILE_BYTES)
    pending = [root]
    while pending:
        current = pending.pop()
        with os.scandir(current) as children:
            child_entries = []
            for child in children:
                visited_entries += 1
                if max_entries is not None and visited_entries > max_entries:
                    raise ResourceLimitError(
                        "model artifact directory entries",
                        limit=max_entries,
                        observed=visited_entries,
                    )
                child_entries.append(child)
            for child in sorted(child_entries, key=lambda e: e.name, reverse=True):
                path = Path(child.path)
                relative = path.relative_to(root).as_posix()
                member_stat = os.lstat(path)
                is_dir = stat.S_ISDIR(member_stat.st_mode)
                # Depth counts directories: a file adds no level of its own.
                depth = len(Path(relative).parts) - (0 if is_dir else 1)
                if max_depth is not None and depth > max_depth:
                    raise ResourceLimitError(
                        "model artifact directory depth",
                        limit=max_depth,
                        observed=depth,
                    )
                if stat.S_ISLNK(member_stat.st_mode):
                    raise ValueError(
                        f"model artifact contains symlink member: {relative}"
                    )
                if is_dir:
                    entries.append((relative, path, "directory"))
                    pending.append(path)
                elif not stat.S_ISREG(member_stat.st_mode):
                    raise ValueError(
                        f"model artifact contains non-regular member: {relative}"
                    )
                elif ignore_legacy_lock_residue and _is_legacy_lock_residue(
                    Path(current), child.name, member_stat
                ):
                    continue
                elif relative not in excluded:
                    if member_stat.st_size > MAX_EXTERNAL_FILE_BYTES:
                        raise ResourceLimitError(
                            f"model artifact member {relative}",
                            limit=MAX_EXTERNAL_FILE_BYTES,
                            observed=member_stat.st_size,
                        )
                    byte_budget.consume(member_stat.st_size)
                    entries.append((relative, path, "regular"))
    return sorted(entries, key=lambda entry: entry[0])


def _legacy_tree_entries(
    root: Path, exclude_relpath: str
) -> list[tuple[str, Path, str]]:
    return _artifact_tree_entries(
        root,
        exclude_relpath=exclude_relpath,
        additional_exclude_relpaths=(_TRANSPORT_DIRECTORY_MANIFEST,),
        ignore_legacy_lock_residue=True,
        max_depth=MAX_PUBLICATION_DIRECTORY_DEPTH,
        max_entries=MAX_PUBLICATION_DIRECTORY_ENTRIES,
    )


def _build_artifact_members_descriptor(root: Path) -> dict[str, Any]:
    entries = _artifact_tree_entries(
        root, exclude_relpath=ARTIFACT_MEMBERS_RELATIVE_PATH
    )
    members: dict[str, dict[str, str | int]] = {}
    hash_budget = _stream_byte_budget()
    for relative, path, kind in entries:
        if kind == "regular":
            digest, size = _stream_model_member_identity(path, byte_budget=hash_budget)
            members[relative] = {"kind": kind, "sha256": digest, "size_bytes": size}
    identity_payload: dict[str, Any] = {
        "schema": ARTIFACT_MEMBERS_SCHEMA,
        "schema_version": ARTIFACT_MEMBERS_SCHEMA_VERSION,
        "descriptor_path": ARTIFACT_MEMBERS_RELATIVE_PATH,
        "paths": [relative for relative, _path, _kind in entries],
        "members": members,
    }
    return {**identity_payload, "identity_sha256": stable_hash(identity_payload)}


_DESCRIPTOR_IDENTITY_KEYS = (
    "schema",
    "schema_version",
    "descriptor_path",
    "paths",
    "members",
)


def _normalize_descriptor_path(value: object, *, field_name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"artifact member {field_name} must be a string")
    normalized = normalize_artifact_relpath(
        value, field_name=f"artifact member {field_name}"
    )
    if normalized != value:
        raise ValueError(f"artifact member {field_name} is not canonical")
    return normalized


def _check_reserved_descriptor_path(relative: str, members_path: str) -> None:
    if relative == members_path:
        raise ValueError("artifact member descriptor must exclude itself")
    if relative == _TRANSPORT_DIRECTORY_MANIFEST:
        raise ValueError("artifact member descriptor must exclude transport metadata")


def _check_set(
    errors: list[str], label: str, expected: set[str], actual: set[str]
) -> None:
    if actual != expected:
        missing, unexpected = sorted(expected - actual), sorted(actual - expected)
        errors.append(
            label
            + (f"; missing: {missing}" if missing else "")
            + (f"; unexpected: {unexpected}" if unexpected else "")
        )


def _member_hash_errors(
    expected_members: Mapping[str, Mapping[str, Any]],
    paths: Mapping[str, Path],
    verb: str,
) -> tuple[list[str], bool]:
    errors: list[str] = []
    budget = _stream_byte_budget()
    for relative, expected in expected_members.items():
        if (path := paths.get(relative)) is None:
            continue
        try:
            digest, size = _stream_model_member_identity(path, byte_budget=budget)
        except (ResourceLimitError, PublicationResourceLimitError) as exc:
            errors.append(f"could not {verb} artifact member {relative}: {exc}")
            return errors, True
        except Exception as exc:
            errors.append(f"could not {verb} artifact member {relative}: {exc}")
            continue
        if digest != expected["sha256"]:
            errors.append(f"artifact member hash mismatch: {relative}")
        if size != expected["size_bytes"]:
            errors.append(f"artifact member size mismatch: {relative}")
    return errors, False


def _validate_artifact_members_descriptor(
    root: Path, manifest: ModelArtifactManifest, published: PublishedDirectory
) -> tuple[list[str], dict[str, str]]:
    """Validate the descriptor against one publication snapshot.

    Returns the errors and the observed ``relative path -> kind`` tree map.
    """
    members_path = manifest.artifact_members_path
    try:
        payload = published.read_member_bytes(members_path)
        descriptor = json.loads(payload)
        if not isinstance(descriptor, dict):
            raise ValueError("artifact member descriptor must be a JSON object")
        if _deterministic_json_bytes(descriptor) != payload:
            raise ValueError("artifact member descriptor is not canonical JSON")
    except Exception as exc:
        return [str(exc)], {}

    errors: list[str] = []
    if set(descriptor) != {*_DESCRIPTOR_IDENTITY_KEYS, "identity_sha256"}:
        errors.append("artifact member descriptor has an invalid field set")
    if descriptor.get("schema") != ARTIFACT_MEMBERS_SCHEMA:
        errors.append("unsupported artifact member descriptor schema")
    if descriptor.get("schema_version") != ARTIFACT_MEMBERS_SCHEMA_VERSION:
        errors.append("unsupported artifact member descriptor version")
    if descriptor.get("descriptor_path") != members_path:
        errors.append("artifact member descriptor path does not match manifest")

    raw_paths = descriptor.get("paths")
    expected_paths: list[str] = []
    if not isinstance(raw_paths, list) or not all(
        isinstance(v, str) for v in raw_paths
    ):
        errors.append("artifact member descriptor paths are invalid")
    else:
        try:
            expected_paths = [
                _normalize_descriptor_path(value, field_name="path")
                for value in raw_paths
            ]
            if expected_paths != sorted(set(expected_paths)):
                raise ValueError("artifact member descriptor paths are not canonical")
            for reserved in (members_path, _TRANSPORT_DIRECTORY_MANIFEST):
                if reserved in expected_paths:
                    _check_reserved_descriptor_path(reserved, members_path)
        except ValueError as exc:
            errors.append(str(exc))

    raw_members = descriptor.get("members")
    expected_members: dict[str, dict[str, Any]] = {}
    if not isinstance(raw_members, dict):
        errors.append("artifact member descriptor members are invalid")
    else:
        if list(raw_members) != sorted(raw_members):
            errors.append("artifact member descriptor members are not canonical")
        for raw_relative, entry in raw_members.items():
            try:
                relative = _normalize_descriptor_path(
                    raw_relative, field_name="member path"
                )
                _check_reserved_descriptor_path(relative, members_path)
                if not isinstance(entry, dict):
                    raise ValueError("artifact member descriptor entry is invalid")
                size = entry.get("size_bytes")
                for part, valid in (
                    ("entry", set(entry) == {"kind", "sha256", "size_bytes"}),
                    ("kind", entry.get("kind") == "regular"),
                    ("hash", _is_sha256_text(entry.get("sha256"))),
                    ("size", _is_int(size) and size >= 0),
                ):
                    if not valid:
                        raise ValueError(
                            f"artifact member descriptor {part} is invalid: {relative}"
                        )
                expected_members[relative] = entry
            except (TypeError, ValueError) as exc:
                errors.append(str(exc))

    if set(expected_members) - set(expected_paths):
        errors.append("artifact member descriptor hashes an unlisted path")
    identity_sha256 = descriptor.get("identity_sha256")
    try:
        identity_matches = identity_sha256 == stable_hash(
            {key: descriptor.get(key) for key in _DESCRIPTOR_IDENTITY_KEYS}
        )
    except (TypeError, ValueError):
        identity_matches = False
    if not _is_sha256_text(identity_sha256) or not identity_matches:
        errors.append("artifact member descriptor identity mismatch")

    def tree() -> set[tuple[str, str]]:
        return {
            (relative, kind)
            for relative, _path, kind in _artifact_tree_entries(
                root,
                exclude_relpath=members_path,
                additional_exclude_relpaths=(_TRANSPORT_DIRECTORY_MANIFEST,),
            )
        }

    try:
        initial_tree = tree()
    except Exception as exc:
        return [*errors, str(exc)], {}
    path_kinds = dict(initial_tree)
    regular = {relative for relative, kind in initial_tree if kind == "regular"}
    _check_set(
        errors,
        "artifact member path set mismatch",
        set(expected_paths),
        set(path_kinds),
    )
    _check_set(
        errors,
        "artifact member regular-file set mismatch",
        set(expected_members),
        regular,
    )
    published_members = set(published.member_hashes)
    if _TRANSPORT_DIRECTORY_MANIFEST in published_members:
        errors.append("published member set contains transport metadata")
    published_members.discard(members_path)
    _check_set(
        errors,
        "published member set mismatch",
        set(expected_members),
        published_members,
    )
    for relative, expected in expected_members.items():
        published_hash = published.member_hashes.get(relative)
        if published_hash is not None and published_hash != expected["sha256"]:
            errors.append(f"artifact member publication hash mismatch: {relative}")

    hash_errors, budget_exhausted = _member_hash_errors(
        expected_members, {r: published.path / r for r in regular}, "read"
    )
    errors.extend(hash_errors)
    # Re-check the tree and member bytes after hashing to catch swaps.
    try:
        if tree() != initial_tree:
            errors.append("artifact member tree changed during validation")
    except Exception as exc:
        errors.append(str(exc))
    if not budget_exhausted:
        rereads = {r: published.path / r for r in expected_members}
        errors.extend(_member_hash_errors(expected_members, rereads, "reread")[0])
    try:
        published.read_member_bytes(members_path)
    except Exception as exc:
        errors.append(f"could not reread artifact member descriptor: {exc}")
    return errors, path_kinds


def _validate_rl_zoo_normalization_metadata(
    manifest: ModelArtifactManifest,
    *,
    member_hashes: Mapping[str, str],
    path_kinds: Mapping[str, str],
) -> list[str]:
    key = RL_ZOO_NORMALIZATION_METADATA_KEY
    if key not in manifest.metadata:
        return []
    value = manifest.metadata[key]
    if not isinstance(value, dict):
        return [f"{key} must be an object"]
    if value == RL_ZOO_NORMALIZATION_NONE:
        return []
    if value != RL_ZOO_NORMALIZATION_VECNORMALIZE:
        return [f"{key} has an unsupported or non-canonical declaration"]
    if manifest.schema_version < MODEL_MANIFEST_CURRENT_SCHEMA_VERSION:
        return ["vecnormalize model artifacts require the schema-v3 member descriptor"]
    path = RL_ZOO_NORMALIZATION_VECNORMALIZE_PATH
    if path not in member_hashes:
        return [
            "rl_zoo_normalization declares vecnormalize statistics that are not "
            f"an inventoried member: {path}"
        ]
    if path_kinds and path_kinds.get(path) != "regular":
        return [
            f"rl_zoo_normalization statistics member must be a regular file: {path}"
        ]
    return []


def read_model_artifact_manifest(path: str | Path) -> ModelArtifactManifest:
    return _load_model_artifact_manifest(Path(path)).manifest


def _read_model_manifest_json(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path)
    root = model_artifact_root_from_manifest_path(manifest_path)
    data = ArtifactPublication.file(
        manifest_path, lock_path=_model_artifact_lock_path(root)
    ).read(json.loads)
    if data is None:
        raise FileNotFoundError(f"model manifest does not exist: {manifest_path}")
    if not isinstance(data, dict):
        raise ValueError("model manifest must be a JSON object")
    return data


def _published_root_identity(root: Path) -> tuple[int, int]:
    root_stat = os.lstat(root)
    if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
        raise ValueError(f"model artifact root must be a real directory: {root}")
    return root_stat.st_dev, root_stat.st_ino


def _published_transport_sha256(root: Path) -> str:
    return _stream_model_member_hash(root / _TRANSPORT_DIRECTORY_MANIFEST)


def _published_binding_errors(
    root: Path, loaded: _LoadedModelArtifactManifest
) -> list[str]:
    errors: list[str] = []
    try:
        if _published_root_identity(root) != loaded.published_root_identity:
            errors.append(
                "published model artifact root was replaced during validation"
            )
    except Exception as exc:
        errors.append(f"published model artifact root changed: {exc}")
    try:
        if _published_transport_sha256(root) != loaded.published_transport_sha256:
            errors.append(
                "published model artifact transport metadata was changed during "
                "validation"
            )
    except Exception as exc:
        errors.append(f"published model artifact transport metadata changed: {exc}")
    return errors


def _load_model_artifact_manifest(manifest_path: Path) -> _LoadedModelArtifactManifest:
    """Load a manifest once, using a published directory for current artifacts.

    A schema-v3 root is never downgraded to path-by-path legacy reads; the
    file fallback is reserved for v1/v2 roots that predate publication.
    """
    root = model_artifact_root_from_manifest_path(manifest_path)
    try:
        manifest_relpath = manifest_path.relative_to(root).as_posix()
    except ValueError as exc:
        raise ValueError("model manifest path is outside its artifact root") from exc

    # Bind the published member snapshot to the still-owned root directory.
    binding_before: list[object] = []
    for probe in (_published_root_identity, _published_transport_sha256):
        try:
            binding_before.append(probe(root))
        except Exception:
            binding_before.append(None)
    try:
        published = ArtifactPublication.directory(
            root, lock_path=_model_artifact_lock_path(root)
        ).open()
    except Exception as directory_error:
        try:
            os.lstat(root / _TRANSPORT_DIRECTORY_MANIFEST)
        except FileNotFoundError:
            pass
        else:
            raise
        data = _read_model_manifest_json(manifest_path)
        manifest = ModelArtifactManifest.from_dict(data)
        if manifest.schema_version >= MODEL_MANIFEST_CURRENT_SCHEMA_VERSION:
            raise ValueError(
                "schema_version 3 model artifacts require a published directory"
            ) from directory_error
        return _LoadedModelArtifactManifest(root, manifest_path, data, manifest)

    if not isinstance(published, PublishedDirectory):
        raise ValueError("model artifact publication is not a directory publication")
    changed = (
        "published model artifact root or transport metadata changed while opening"
    )
    try:
        root_identity = _published_root_identity(root)
        transport_sha256 = _published_transport_sha256(root)
    except Exception as exc:
        raise ValueError(changed) from exc
    if binding_before != [root_identity, transport_sha256]:
        raise ValueError(changed)

    if manifest_relpath not in published.member_hashes:
        raise ValueError(
            f"model manifest is not a published member: {manifest_relpath}"
        )
    payload = published.read_member_bytes(manifest_relpath)
    data = json.loads(payload)
    if not isinstance(data, dict):
        raise ValueError("model manifest must be a JSON object")
    manifest = ModelArtifactManifest.from_dict(data)
    if manifest.schema_version >= MODEL_MANIFEST_CURRENT_SCHEMA_VERSION:
        if manifest_relpath != MODEL_MANIFEST_RELATIVE_PATH:
            raise ValueError(
                "schema_version 3 model artifacts must place the manifest at "
                f"{MODEL_MANIFEST_RELATIVE_PATH!r}"
            )
        if _deterministic_json_bytes(data) != payload:
            raise ValueError("model manifest is not canonical JSON")
    return _LoadedModelArtifactManifest(
        root, manifest_path, data, manifest, published, root_identity, transport_sha256
    )


def find_model_artifact_manifest(path: str | Path) -> Path:
    candidate = Path(path)
    if candidate.is_file() and candidate.name == MODEL_MANIFEST_NAME:
        return candidate
    manifest_paths: list[Path] = []
    if candidate.is_dir():
        manifest_paths += [candidate / "meta", candidate]
    if candidate.exists():
        # A file or directory checkpoint member: walk up to the artifact root.
        manifest_paths += [candidate.parent / "meta", candidate.parent.parent / "meta"]
    for directory in manifest_paths:
        if (directory / MODEL_MANIFEST_NAME).is_file():
            return directory / MODEL_MANIFEST_NAME
    raise FileNotFoundError(f"could not find {MODEL_MANIFEST_NAME} under {candidate}")


def model_artifact_root_from_manifest_path(manifest_path: str | Path) -> Path:
    parent = Path(manifest_path).parent
    return parent.parent if parent.name == "meta" else parent


def try_model_artifact_reference(path: str | Path) -> ModelArtifactReference | None:
    """Resolve *path* if it points at an SRB model artifact.

    Returns ``None`` when no nearby SRB model manifest exists. A present but
    invalid manifest raises, so callers never silently ignore a broken artifact.
    """
    try:
        manifest_path = find_model_artifact_manifest(path)
    except FileNotFoundError:
        return None

    report = validate_model_artifact_layout(manifest_path)
    manifest = report.manifest
    if not report.valid or manifest is None:
        message = "; ".join(report.errors) or "invalid model artifact"
        if report.errors and all(
            e.startswith("checkpoint does not exist:") for e in report.errors
        ):
            raise FileNotFoundError(message)
        raise ValueError(f"invalid model artifact: {message}")
    artifact_root = model_artifact_root_from_manifest_path(manifest_path)
    checkpoint_path = artifact_member_path(
        artifact_root, manifest.checkpoint_path, field_name="checkpoint_path"
    )
    if not (
        checkpoint_path.is_dir()
        if manifest.checkpoint_kind == "directory"
        else checkpoint_path.is_file()
    ):
        raise FileNotFoundError(
            f"checkpoint does not exist: {manifest.checkpoint_path}"
        )
    if manifest.identity.algo is None:
        raise ValueError("model artifact identity requires algo")
    return ModelArtifactReference(
        manifest_path=manifest_path,
        artifact_root=artifact_root,
        checkpoint_path=checkpoint_path,
        algo=manifest.identity.algo,
        manifest=manifest,
        _checkpoint_snapshot=_materialize_model_checkpoint_snapshot(
            artifact_root, manifest, report
        ),
    )


def _copy_model_file_snapshot(
    source: Path,
    destination: Path,
    *,
    expected_sha256: str,
    max_bytes: int | None = MAX_EXTERNAL_FILE_BYTES,
) -> None:
    """Copy one validated legacy member, bound to its admitted identity.

    Nothing (not even a lock) is written beside *source*.
    """
    if not os.path.lexists(source):
        raise FileNotFoundError(source)
    _copy_regular_file_to_snapshot(
        source, destination, expected_sha256=expected_sha256, max_bytes=max_bytes
    )


def _stream_model_member_hash(
    path: Path, *, byte_budget: ByteBudget | None = None
) -> str:
    """Hash a member through the bounded no-follow reader.

    Unlike ``open_snapshot`` this creates no sibling lock marker, which would
    otherwise become a member of a model tree under assembly.
    """
    return sha256_file(path, byte_budget=byte_budget)


def _stream_model_member_identity(
    path: Path, *, byte_budget: ByteBudget | None = None
) -> tuple[str, int]:
    before = os.lstat(path)
    if not stat.S_ISREG(before.st_mode):
        raise ValueError(f"model artifact member is not a regular file: {path}")
    digest = _stream_model_member_hash(path, byte_budget=byte_budget)
    after = os.lstat(path)
    if (
        not stat.S_ISREG(after.st_mode)
        or not os.path.samestat(before, after)
        or before.st_size != after.st_size
        or before.st_mtime_ns != after.st_mtime_ns
    ):
        raise ValueError(f"model artifact member changed while hashing: {path}")
    return digest, after.st_size


def _materialize_model_checkpoint_snapshot(
    root: Path, manifest: ModelArtifactManifest, report: ModelArtifactValidationReport
) -> ModelCheckpointSnapshot:
    hashes = report.member_hashes
    normalization = manifest.metadata.get(RL_ZOO_NORMALIZATION_METADATA_KEY)
    configs = (manifest.metadata.get("framework_config_path"), *_CONFIG_CANDIDATES)
    members = set(report.checkpoint_member_hashes)
    if report.checkpoint_sha256 is not None:
        members.add(manifest.checkpoint_path)
    for candidate in (
        normalization.get("path") if isinstance(normalization, dict) else None,
        *configs,
    ):
        if isinstance(candidate, str) and candidate in hashes:
            members.add(candidate)
    if manifest.checkpoint_kind == "directory":
        members.update(
            r for r in hashes if r.startswith(manifest.checkpoint_path + "/")
        )
    selected = tuple(sorted(members))
    if not selected:
        raise ValueError("model artifact has no validated checkpoint members")

    def copy(source_root: Path, relative: str, target: Path, what: str) -> None:
        if (expected := hashes.get(relative)) is None:
            raise ValueError(f"model snapshot {what} is not validated: {relative}")
        _copy_model_file_snapshot(
            source_root / relative, target, expected_sha256=expected
        )

    owner: _SnapshotOwner
    legacy = manifest.schema_version < MODEL_MANIFEST_CURRENT_SCHEMA_VERSION
    if legacy:
        owner = tempfile.TemporaryDirectory(prefix="srb-model-")
        snapshot_root = Path(owner.name)
    else:
        owner = PublishedDirectory(
            path=root, member_hashes=hashes
        ).materialize_snapshot(members=selected)
        snapshot_root = owner.path
    try:
        for relative in selected if legacy else ():
            copy(root, relative, snapshot_root / relative, "member")
        # Loaders historically find a config beside the checkpoint; serve it
        # from the verified snapshot, never from the live artifact.
        config = next(
            (
                c
                for c in configs
                if isinstance(c, str) and (snapshot_root / c).is_file()
            ),
            None,
        )
        config_target = snapshot_root / "config.yaml"
        if config is not None and not config_target.exists():
            copy(snapshot_root, config, config_target, "config")
            members.add("config.yaml")
    except BaseException:
        _release_snapshot_owner(owner)
        raise
    return ModelCheckpointSnapshot(
        root=snapshot_root,
        checkpoint_path=snapshot_root / manifest.checkpoint_path,
        _owner=owner,
        member_paths=frozenset(members),
    )


def _artifact_hash_mismatch(stored: object, computed: str) -> str:
    return (
        "model manifest artifact_hash mismatch (manifest was "
        f"modified after packaging): stored {stored} != computed {computed}"
    )


def _directory_checkpoint_errors(
    manifest: ModelArtifactManifest,
    *,
    is_directory: bool,
    validate_dreamer: Callable[[], object],
) -> list[str]:
    if not is_directory:
        return [
            "checkpoint does not exist or is not the declared "
            f"directory kind: {manifest.checkpoint_path}"
        ]
    # No member allowlist exists for other frameworks, so a hand-written
    # directory manifest must not reach policy loading unvalidated.
    if manifest.framework != "dreamer":
        return [
            "directory checkpoints are only supported for the dreamer "
            f"framework, not {manifest.framework!r}"
        ]
    try:
        validate_dreamer()
    except (OSError, ValueError) as exc:
        return [str(exc)]
    return []


def _checkpoint_hashes(
    manifest: ModelArtifactManifest, member_hashes: Mapping[str, str]
) -> tuple[str | None, dict[str, str]]:
    if manifest.checkpoint_kind == "file":
        return member_hashes.get(manifest.checkpoint_path), {}
    prefix = manifest.checkpoint_path + "/"
    return None, {r: d for r, d in member_hashes.items() if r.startswith(prefix)}


def _unloadable_artifact_report(
    manifest_or_path: ModelArtifactManifest | str | Path, error: Exception
) -> ModelArtifactValidationReport:
    manifest: ModelArtifactManifest | None = None
    errors = [str(error)]
    if not isinstance(manifest_or_path, ModelArtifactManifest):
        try:
            manifest_path = find_model_artifact_manifest(manifest_or_path)
            root = model_artifact_root_from_manifest_path(manifest_path)
            raw = _read_model_manifest_json(manifest_path)
            manifest = ModelArtifactManifest.from_dict(raw)
            stored_hash = raw.get("artifact_hash")
            if stored_hash is not None and str(stored_hash) != manifest.artifact_hash:
                errors.append(
                    _artifact_hash_mismatch(stored_hash, manifest.artifact_hash)
                )
            checkpoint_path = artifact_member_path(
                root, manifest.checkpoint_path, field_name="checkpoint_path"
            )
            if manifest.checkpoint_kind == "directory":
                if manifest.framework == "dreamer" and checkpoint_path.is_dir():
                    try:
                        validate_dreamer_checkpoint_directory(checkpoint_path)
                    except (OSError, ValueError) as exc:
                        errors.append(str(exc))
            elif checkpoint_path.is_dir():
                errors.append(
                    "checkpoint is a directory but the manifest declares "
                    f"kind 'file': {manifest.checkpoint_path}"
                )
        except Exception as exc:
            # Never replace the original failure, but do not hide this one.
            errors.append(
                "could not collect further diagnostics for this artifact: "
                f"{exc.__class__.__name__}: {exc}"
            )
    return ModelArtifactValidationReport(
        valid=False, errors=errors, warnings=[], manifest=manifest
    )


def validate_model_artifact_layout(
    manifest_or_path: ModelArtifactManifest | str | Path, *, strict: bool = False
) -> ModelArtifactValidationReport:
    """Validate the manifest, checkpoint, and artifact member snapshot."""
    errors: list[str] = []
    warnings: list[str] = []
    member_paths: tuple[str, ...] = ()
    member_hashes: dict[str, str] = {}
    checkpoint_sha256: str | None = None
    checkpoint_member_hashes: dict[str, str] = {}
    path_kinds: dict[str, str] = {}
    loaded: _LoadedModelArtifactManifest | None = None
    try:
        if isinstance(manifest_or_path, ModelArtifactManifest):
            manifest = manifest_or_path
            root = Path.cwd()
        else:
            loaded = _load_model_artifact_manifest(
                find_model_artifact_manifest(manifest_or_path)
            )
            manifest = loaded.manifest
            root = loaded.root
    except Exception as exc:
        return _unloadable_artifact_report(manifest_or_path, exc)

    published = loaded.published if loaded is not None else None
    current = manifest.schema_version >= MODEL_MANIFEST_CURRENT_SCHEMA_VERSION
    tree_bound = current and published is not None

    if loaded is not None:
        # Stored-hash integrity: any tampered payload field changes the hash.
        stored_hash = loaded.raw.get("artifact_hash")
        if stored_hash is not None:
            try:
                computed_hash = manifest.artifact_hash
            except (TypeError, ValueError) as exc:
                errors.append(f"model manifest contains invalid JSON values: {exc}")
            else:
                if str(stored_hash) != computed_hash:
                    errors.append(_artifact_hash_mismatch(stored_hash, computed_hash))

    if current and loaded is None:
        errors.append(
            "schema_version 3 model artifacts require a manifest path for "
            "tree validation"
        )
    elif current and loaded is not None and loaded.published is not None:
        # A schema-v3 manifest path always loads through a published directory.
        errors.extend(_published_binding_errors(root, loaded))
        member_errors, path_kinds = _validate_artifact_members_descriptor(
            root, manifest, loaded.published
        )
        errors.extend(member_errors)
        member_hashes = dict(loaded.published.member_hashes)
        checkpoint_sha256, checkpoint_member_hashes = _checkpoint_hashes(
            manifest, member_hashes
        )
        try:
            if path_kinds.get("README.md") != "regular":
                raise ValueError("model artifact README.md must be a regular file")
            actual_readme = loaded.published.read_member_bytes("README.md")
            from srb.utils.artifact_cards import build_model_card

            if actual_readme != build_model_card(manifest).encode("utf-8"):
                raise ValueError(
                    "model artifact canonical README.md differs from its manifest"
                )
        except FileNotFoundError:
            errors.append("model artifact README.md does not exist")
        except Exception as exc:
            errors.append(str(exc))

    checkpoint_relpath = manifest.checkpoint_path
    if current and published is not None:
        checkpoint_kind = path_kinds.get(checkpoint_relpath)
        if manifest.checkpoint_kind == "directory":
            snapshot = published
            errors.extend(
                _directory_checkpoint_errors(
                    manifest,
                    is_directory=checkpoint_kind == "directory",
                    validate_dreamer=lambda: (
                        _validate_dreamer_checkpoint_members_snapshot(
                            checkpoint_relpath, path_kinds, snapshot
                        )
                    ),
                )
            )
        elif checkpoint_kind != "regular":
            errors.append(f"checkpoint does not exist: {checkpoint_relpath}")
    else:
        try:
            # Resolves symlinks and rejects members escaping the artifact root.
            checkpoint_path = artifact_member_path(
                root, checkpoint_relpath, field_name="checkpoint_path"
            )
        except ValueError as exc:
            errors.append(str(exc))
        else:
            if manifest.checkpoint_kind == "directory":
                errors.extend(
                    _directory_checkpoint_errors(
                        manifest,
                        is_directory=not checkpoint_path.is_symlink()
                        and checkpoint_path.is_dir(),
                        validate_dreamer=lambda: validate_dreamer_checkpoint_directory(
                            checkpoint_path
                        ),
                    )
                )
            elif checkpoint_path.is_symlink():
                errors.append(f"checkpoint must not be a symlink: {checkpoint_relpath}")
            elif checkpoint_path.is_dir():
                errors.append(
                    "checkpoint is a directory but the manifest declares "
                    f"kind 'file': {checkpoint_relpath}"
                )
            elif not checkpoint_path.is_file():
                errors.append(f"checkpoint does not exist: {checkpoint_relpath}")

    if published is None:
        try:
            hash_budget = _stream_byte_budget()
            for relative, path, kind in _legacy_tree_entries(
                root, manifest.artifact_members_path
            ):
                if kind != "regular":
                    continue
                try:
                    digest, _size = _stream_model_member_identity(
                        path, byte_budget=hash_budget
                    )
                except (ResourceLimitError, PublicationResourceLimitError) as exc:
                    errors.append(str(exc))
                    break
                member_hashes[relative] = digest
            checkpoint_sha256, checkpoint_member_hashes = _checkpoint_hashes(
                manifest, member_hashes
            )
        except Exception as exc:
            errors.append(str(exc))

    if manifest.checkpoint_scope == "unspecified":
        warnings.append(
            "model artifact does not declare a checkpoint scope "
            "(weights vs agent_evaluation)"
        )

    config_relpath = manifest.metadata.get("framework_config_path")
    if config_relpath is not None and not isinstance(config_relpath, str):
        errors.append("framework_config_path must be a relative string")
    elif config_relpath is not None:
        try:
            if tree_bound:
                config_relpath = normalize_artifact_relpath(
                    config_relpath, field_name="framework_config_path"
                )
                config_exists = path_kinds.get(config_relpath) == "regular"
            else:
                config_path = artifact_member_path(
                    root, config_relpath, field_name="framework_config_path"
                )
                config_relpath = config_path.relative_to(root).as_posix()
                config_exists = not config_path.is_symlink() and config_path.is_file()
        except ValueError as exc:
            errors.append(str(exc))
        else:
            if not config_exists:
                errors.append(f"framework_config_path does not exist: {config_relpath}")

    if loaded is not None and tree_bound:
        errors.extend(_published_binding_errors(root, loaded))

    errors.extend(
        _validate_rl_zoo_normalization_metadata(
            manifest, member_hashes=member_hashes, path_kinds=path_kinds
        )
    )

    if published is not None:
        published_paths = set(published.member_hashes) - {_TRANSPORT_DIRECTORY_MANIFEST}
        member_paths = tuple(sorted(published_paths))
    elif loaded is not None:
        try:
            member_paths = tuple(
                relative
                for relative, _path, kind in _legacy_tree_entries(
                    root, "__srb_no_portable_descriptor__"
                )
                if kind == "regular"
            )
        except Exception as exc:
            errors.append(str(exc))

    if manifest.dataset_repo_id is None:
        warnings.append(
            "model artifact does not identify the training dataset repository"
        )
    elif not _is_huggingface_repo_id(str(manifest.dataset_repo_id)):
        errors.append(
            "training dataset repository id must use namespace/name with "
            "Hugging Face-compatible slug characters"
        )

    if strict:
        errors.extend(f"warning treated as error: {warning}" for warning in warnings)

    return ModelArtifactValidationReport(
        valid=not errors,
        errors=errors,
        warnings=warnings,
        manifest=manifest,
        member_paths=member_paths,
        member_hashes=member_hashes,
        checkpoint_sha256=checkpoint_sha256,
        checkpoint_member_hashes=checkpoint_member_hashes,
    )


def _is_huggingface_repo_id(value: str) -> bool:
    parts = value.split("/")
    return len(parts) == 2 and all(
        _HF_REPO_ID_PART_PATTERN.fullmatch(part) is not None
        and not part.endswith((".", "-"))
        for part in parts
    )


def find_saved_policy_config(model_path: Path) -> Path | None:
    """Find an artifact-aware framework config with legacy fallbacks.

    A valid artifact's declared config wins over conventional run-layout
    candidates. Missing artifacts and the historical missing-checkpoint
    ``FileNotFoundError`` retain legacy fallback behavior; malformed artifacts
    propagate their validation error.
    """
    checkpoint_path = Path(model_path)
    try:
        reference = try_model_artifact_reference(checkpoint_path)
    except FileNotFoundError:
        reference = None

    if reference is not None:
        # Serve the config from the verified snapshot, never the live artifact
        # root, which may have changed since validation.
        declared = reference.manifest.metadata.get("framework_config_path")
        for relpath in (
            *((str(declared),) if declared else ()),
            "config.yaml",
            "config.yml",
            "meta/config.yaml",
        ):
            if (member := reference.snapshot_member(relpath)) is not None:
                return _SnapshotMemberPath(member, owner=reference)

    run_dir, checkpoint_dir = checkpoint_path.parent.parent, checkpoint_path.parent
    return _first_file(
        [run_dir / name for name in ("config.yaml", "config.yml", ".hydra/config.yaml")]
        + [checkpoint_dir / name for name in ("config.yaml", "config.yml")]
    )


class _SnapshotMemberPath(type(Path())):
    """A snapshot member path that keeps its snapshot alive while referenced."""

    def __init__(self, *args: Any, owner: ModelArtifactReference | None = None):
        super().__init__(*args)
        self._snapshot_owner = owner


def _first_file(candidates: Iterable[Path]) -> Path | None:
    return next((candidate for candidate in candidates if candidate.is_file()), None)


__all__ = [
    "ARTIFACT_MEMBERS_NAME",
    "ARTIFACT_MEMBERS_RELATIVE_PATH",
    "ARTIFACT_MEMBERS_SCHEMA",
    "ARTIFACT_MEMBERS_SCHEMA_VERSION",
    "CHECKPOINT_KINDS",
    "CHECKPOINT_SCOPES",
    "MODEL_MANIFEST_CURRENT_SCHEMA_VERSION",
    "MODEL_MANIFEST_NAME",
    "MODEL_MANIFEST_RELATIVE_PATH",
    "MODEL_MANIFEST_SCHEMA_VERSIONS",
    "ModelArtifactManifest",
    "ModelArtifactReference",
    "ModelArtifactValidationReport",
    "ModelCheckpointSnapshot",
    "RL_ZOO_NORMALIZATION_METADATA_KEY",
    "RL_ZOO_NORMALIZATION_NONE",
    "RL_ZOO_NORMALIZATION_VECNORMALIZE",
    "RL_ZOO_NORMALIZATION_VECNORMALIZE_PATH",
    "artifact_member_path",
    "build_model_artifact_manifest",
    "find_model_artifact_manifest",
    "find_saved_policy_config",
    "model_artifact_root_from_manifest_path",
    "normalize_artifact_relpath",
    "read_model_artifact_manifest",
    "try_model_artifact_reference",
    "validate_model_artifact_layout",
]
