"""Model artifact metadata and validation for SRB checkpoints."""

from __future__ import annotations

import json
import os
import shutil
import stat
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from srb.integrations.algorithms import resolve_algorithm_adapter
from srb.integrations.run_layouts import (
    FrameworkConfigArtifact,
    discover_framework_config_artifact,
    discover_rl_zoo_normalization_artifact,
    discover_run_checkpoint,
)
from srb.utils.artifact_identity import ArtifactIdentity, slugify_underscore
from srb.utils.dataset_schema import DatasetSplit, StorageFormat
from srb.utils.failures import InvalidInputError
from srb.utils.model_artifacts import (
    _ELEMENTS_DONE_MARKER,
    _TRANSPORT_DIRECTORY_MANIFEST,
    ARTIFACT_MEMBERS_RELATIVE_PATH,
    ARTIFACT_MEMBERS_SCHEMA,
    ARTIFACT_MEMBERS_SCHEMA_VERSION,
    CHECKPOINT_KINDS,
    CHECKPOINT_SCOPES,
    MODEL_MANIFEST_CURRENT_SCHEMA_VERSION,
    MODEL_MANIFEST_NAME,
    MODEL_MANIFEST_RELATIVE_PATH,
    MODEL_MANIFEST_SCHEMA_VERSIONS,
    RL_ZOO_NORMALIZATION_METADATA_KEY,
    RL_ZOO_NORMALIZATION_NONE,
    RL_ZOO_NORMALIZATION_VECNORMALIZE,
    RL_ZOO_NORMALIZATION_VECNORMALIZE_PATH,
    ModelArtifactManifest,
    ModelArtifactReference,
    ModelArtifactValidationReport,
    ModelCheckpointSnapshot,
    _agent_payload_members,
    _artifact_tree_entries,
    _build_artifact_members_descriptor,
    _copy_model_file_snapshot,
    _deterministic_json_bytes,
    _load_model_artifact_manifest,
    _model_artifact_lock_path,
    _stream_model_member_hash,
    _stream_model_member_identity,
    artifact_member_path,
    find_model_artifact_manifest,
    model_artifact_root_from_manifest_path,
    normalize_artifact_relpath,
    read_model_artifact_manifest,
    try_model_artifact_reference,
    validate_dreamer_checkpoint_directory,
    validate_model_artifact_layout,
)
from srb.utils.model_artifacts import (
    build_model_artifact_manifest as _build_model_artifact_manifest,
)
from srb.utils.publication import (
    ArtifactPublication,
    PublicationIdentityError,
    _copy_regular_file_to_snapshot,
)

RUN_METADATA_NAME = "metadata.json"


@dataclass(frozen=True)
class ModelArtifactPreparationResult:
    """Result of creating a self-contained SRB model artifact directory."""

    manifest_path: Path
    checkpoint_path: Path
    manifest: ModelArtifactManifest

    def to_dict(self) -> dict[str, Any]:
        return {
            "manifest_path": str(self.manifest_path),
            "checkpoint_path": str(self.checkpoint_path),
            "hf_repo_id": self.manifest.hf_repo_id,
        }


@dataclass(frozen=True, slots=True)
class _ConsumedDemoReplay:
    """Validated demo replay provenance captured by the training run."""

    block: dict[str, Any]
    manifest: dict[str, Any]
    identity: dict[str, str]


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
    """Build a model manifest while preserving algorithm-owned inference."""
    if identity.algo is None:
        raise ValueError("model artifact identity requires algo")
    return _build_model_artifact_manifest(
        identity=identity,
        checkpoint_path=checkpoint_path,
        framework=framework or _infer_framework(identity.algo),
        dataset_repo_id=dataset_repo_id,
        dataset_revision=dataset_revision,
        metrics=metrics,
        metadata=metadata,
        hf_repo_id=hf_repo_id,
        schema_version=schema_version,
        checkpoint_kind=checkpoint_kind,
        checkpoint_scope=checkpoint_scope,
        artifact_members_path=artifact_members_path,
    )


def sanitize_dreamer_checkpoint_directory(
    source_dir: str | Path,
    staging_dir: str | Path,
) -> list[str]:
    """Copy the completion marker and agent payload to staging; return names."""
    source = Path(source_dir)
    agent_members = _agent_payload_members(
        source, label="Dreamer source checkpoint directory"
    )

    staging = Path(staging_dir)
    if staging.exists():
        if not staging.is_dir() or any(staging.iterdir()):
            raise FileExistsError(f"staging directory is not empty: {staging}")
    else:
        staging.mkdir(parents=True, exist_ok=False)
    # The completion marker is recreated empty rather than copied.
    (staging / _ELEMENTS_DONE_MARKER).touch()
    for name in agent_members:
        shutil.copy2(source / name, staging / name, follow_symlinks=False)
    return [_ELEMENTS_DONE_MARKER, *agent_members]


def prepare_model_artifact(
    *,
    checkpoint_path: str | Path,
    output_dir: str | Path,
    identity: ArtifactIdentity,
    framework: str | None = None,
    training_dataset_path: str | Path | None = None,
    dataset_repo_id: str | None = None,
    dataset_revision: str | None = None,
    metrics: dict[str, Any] | None = None,
    metadata: dict[str, Any] | None = None,
    hf_repo_id: str | None = None,
    checkpoint_scope: str | None = None,
    _framework_config_artifact: FrameworkConfigArtifact | None = None,
    artifact_members: Mapping[str, str | Path] | None = None,
) -> ModelArtifactPreparationResult:
    """Publish one complete, immutable SRB model artifact generation.

    The checkpoint, manifest, optional framework config, canonical README, and
    portable member descriptor are built in one API-owned staging directory.
    A Dreamer checkpoint is sanitized (``done`` marker + agent payload only)
    before promotion; same-logdir training state is never portable.
    """
    source_checkpoint = Path(checkpoint_path).expanduser()
    resolved_framework = framework or _infer_framework(
        identity.algo or ""  # __post_init__ enforces algo below
    )
    if source_checkpoint.is_symlink():
        raise ValueError(f"checkpoint must not be a symlink: {source_checkpoint}")
    if source_checkpoint.is_dir():
        checkpoint_kind = "directory"
        if resolved_framework != "dreamer":
            raise ValueError(
                "directory checkpoints are only supported for the dreamer "
                f"framework, got {resolved_framework!r}"
            )
        checkpoint_scope = checkpoint_scope or "agent_evaluation"
        if checkpoint_scope != "agent_evaluation":
            raise ValueError(
                "a sanitized Dreamer checkpoint directory is agent state "
                f"only; scope {checkpoint_scope!r} is not portable"
            )
        agent_members = _agent_payload_members(
            source_checkpoint,
            label="Dreamer source checkpoint directory",
        )
    elif source_checkpoint.is_file():
        checkpoint_kind = "file"
        # File checkpoints contain weights only; callers may specify a narrower
        # scope explicitly.
        checkpoint_scope = checkpoint_scope or "weights"
        source_stat = os.lstat(source_checkpoint)
        if not stat.S_ISREG(source_stat.st_mode):
            raise ValueError(f"checkpoint must be a regular file: {source_checkpoint}")
        agent_members = []
    else:
        raise FileNotFoundError(f"checkpoint does not exist: {source_checkpoint}")

    root = Path(output_dir)
    checkpoint_relpath = (Path("checkpoints") / source_checkpoint.name).as_posix()
    checkpoint_relpath = normalize_artifact_relpath(
        checkpoint_relpath,
        field_name="checkpoint_path",
    )

    dataset_repo_id, dataset_revision, dataset_metadata = (
        _resolve_training_dataset_provenance(
            training_dataset_path=training_dataset_path,
            dataset_repo_id=dataset_repo_id,
            dataset_revision=dataset_revision,
        )
    )
    model_metadata = dict(metadata or {})
    model_metadata.update(dataset_metadata)

    manifest = build_model_artifact_manifest(
        identity=identity,
        checkpoint_path=checkpoint_relpath,
        framework=framework,
        dataset_repo_id=dataset_repo_id,
        dataset_revision=dataset_revision,
        metrics=metrics,
        metadata=model_metadata,
        hf_repo_id=hf_repo_id,
        schema_version=MODEL_MANIFEST_CURRENT_SCHEMA_VERSION,
        checkpoint_kind=checkpoint_kind,
        checkpoint_scope=checkpoint_scope,
        artifact_members_path=ARTIFACT_MEMBERS_RELATIVE_PATH,
    )
    manifest_relpath = f"meta/{MODEL_MANIFEST_NAME}"
    descriptor_relpath = manifest.artifact_members_path
    readme_relpath = "README.md"
    config_relpath = (
        _framework_config_artifact.artifact_relpath
        if _framework_config_artifact is not None
        else None
    )
    additional_members: dict[str, Path] = {}
    for relative, source in (artifact_members or {}).items():
        normalized = normalize_artifact_relpath(
            relative,
            field_name="artifact member path",
        )
        if normalized in additional_members:
            raise ValueError(f"duplicate artifact member path: {normalized}")
        additional_members[normalized] = Path(source)
    normalization_metadata = model_metadata.get(RL_ZOO_NORMALIZATION_METADATA_KEY)
    if normalization_metadata is not None:
        if normalization_metadata == RL_ZOO_NORMALIZATION_NONE:
            pass
        elif normalization_metadata == RL_ZOO_NORMALIZATION_VECNORMALIZE:
            if RL_ZOO_NORMALIZATION_VECNORMALIZE_PATH not in additional_members:
                raise ValueError(
                    "rl_zoo_normalization requires the packaged vecnormalize "
                    "statistics member"
                )
        else:
            raise ValueError(
                "rl_zoo_normalization must use the closed none or vecnormalize "
                "declaration"
            )
    reserved_paths = {
        checkpoint_relpath,
        manifest_relpath,
        descriptor_relpath,
        readme_relpath,
    }
    if config_relpath in reserved_paths:
        raise ValueError(
            f"framework config path collides with a reserved model artifact member: "
            f"{config_relpath}"
        )
    collisions = sorted(reserved_paths.intersection(additional_members))
    if collisions:
        raise ValueError(
            "artifact member path collides with a reserved model artifact member: "
            + ", ".join(collisions)
        )

    # Reject symlinked output parents before staging the artifact.
    for field_name, relative in (
        ("checkpoint_path", checkpoint_relpath),
        ("manifest_path", manifest_relpath),
        ("artifact_members_path", descriptor_relpath),
        ("README_path", readme_relpath),
    ):
        artifact_member_path(
            root,
            relative,
            field_name=field_name,
        )
    if config_relpath is not None:
        artifact_member_path(
            root,
            config_relpath,
            field_name="framework_config_path",
        )
    for relative in additional_members:
        artifact_member_path(
            root,
            relative,
            field_name="artifact member path",
        )

    if root.exists() and not (root / _TRANSPORT_DIRECTORY_MANIFEST).is_file():
        raise PublicationIdentityError(
            path=root,
            expected_sha256="complete-directory-publication",
            actual_sha256="conflicting-existing-root",
        )

    from srb.utils.artifact_cards import build_model_card

    readme_payload = build_model_card(manifest).encode("utf-8")
    manifest_payload = _deterministic_json_bytes(manifest.to_dict())

    def builder(staging: Path) -> None:
        checkpoint_target = staging / checkpoint_relpath
        checkpoint_target.parent.mkdir(parents=True, exist_ok=True)
        if checkpoint_kind == "directory":
            sanitize_dreamer_checkpoint_directory(source_checkpoint, checkpoint_target)
            validate_dreamer_checkpoint_directory(checkpoint_target)
        else:
            shutil.copy2(
                source_checkpoint,
                checkpoint_target,
                follow_symlinks=False,
            )

        if _framework_config_artifact is not None:
            _materialize_framework_config_in_stage(
                _framework_config_artifact,
                staging,
            )
        for relative, source in additional_members.items():
            source_stat = os.lstat(source)
            if not stat.S_ISREG(source_stat.st_mode):
                raise ValueError(
                    f"artifact member source must be a regular file: {source}"
                )
            target = staging / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            expected_sha256 = _stream_model_member_hash(source)
            _copy_regular_file_to_snapshot(
                source,
                target,
                expected_sha256=expected_sha256,
            )

        manifest_target = staging / manifest_relpath
        manifest_target.parent.mkdir(parents=True, exist_ok=True)
        manifest_target.write_bytes(manifest_payload)
        (staging / readme_relpath).write_bytes(readme_payload)

        descriptor_target = staging / descriptor_relpath
        descriptor_target.parent.mkdir(parents=True, exist_ok=True)
        descriptor_target.write_bytes(
            _deterministic_json_bytes(_build_artifact_members_descriptor(staging))
        )

    required_members = [manifest_relpath, readme_relpath, descriptor_relpath]
    required_members.append(
        f"{checkpoint_relpath}/{agent_members[0]}"
        if checkpoint_kind == "directory"
        else checkpoint_relpath
    )
    if config_relpath is not None:
        required_members.append(config_relpath)
    required_members.extend(additional_members)
    ArtifactPublication.directory(
        root,
        lock_path=_model_artifact_lock_path(root),
    ).publish(
        builder,
        required_members=required_members,
    )

    artifact_checkpoint = root / checkpoint_relpath
    manifest_path = root / manifest_relpath
    return ModelArtifactPreparationResult(
        manifest_path=manifest_path,
        checkpoint_path=artifact_checkpoint,
        manifest=manifest,
    )


def _materialize_framework_config_in_stage(
    artifact: FrameworkConfigArtifact,
    staging_root: Path,
) -> Path:
    """Materialize a discovered framework config inside private staging."""

    target_path = staging_root / artifact.artifact_relpath
    target_path.parent.mkdir(parents=True, exist_ok=True)
    if artifact.generated_config is not None:
        import yaml

        target_path.write_text(
            yaml.safe_dump(artifact.generated_config, sort_keys=False),
            encoding="utf-8",
        )
    elif artifact.admitted_bytes is not None:
        target_path.write_bytes(artifact.admitted_bytes)
    elif artifact.source_path is not None:
        if artifact.source_path.is_symlink() or not artifact.source_path.is_file():
            raise ValueError(
                "framework config source must be a regular file: "
                f"{artifact.source_path}"
            )
        shutil.copy2(
            artifact.source_path,
            target_path,
            follow_symlinks=False,
        )
    return target_path


def prepare_model_artifact_from_run(
    *,
    run_dir: str | Path,
    output_dir: str | Path,
    task: str | None = None,
    embodiment: str | None = None,
    action_mode: str | None = None,
    algo: str | None = None,
    checkpoint_path: str | Path | None = None,
    framework: str | None = None,
    training_dataset_path: str | Path | None = None,
    dataset_repo_id: str | None = None,
    dataset_revision: str | None = None,
    metrics: dict[str, Any] | None = None,
    metadata: dict[str, Any] | None = None,
    hf_repo_id: str | None = None,
    checkpoint_scope: str | None = None,
) -> ModelArtifactPreparationResult:
    """Package the latest checkpoint from an SRB run directory."""
    run_path = Path(run_dir)
    if not run_path.is_dir():
        raise FileNotFoundError(f"run directory does not exist: {run_path}")

    run_metadata_path = run_path / RUN_METADATA_NAME
    run_metadata = _read_run_metadata(run_metadata_path)
    consumed_demo_replay = _read_consumed_demo_replay(run_metadata)
    resolved_algo = _resolve_run_algorithm(
        explicit_algo=algo,
        run_metadata=run_metadata,
    )
    identity = _resolve_run_model_identity(
        # The run's recorded task is authoritative when no dataset is supplied.
        task=(
            task
            if training_dataset_path is not None or consumed_demo_replay is not None
            else _resolve_run_task(explicit_task=task, run_metadata=run_metadata)
        ),
        embodiment=embodiment,
        action_mode=action_mode,
        algo=resolved_algo,
        training_dataset_path=training_dataset_path,
        consumed_identity=(
            None if consumed_demo_replay is None else consumed_demo_replay.identity
        ),
    )
    resolved_framework = framework or resolve_algorithm_adapter(resolved_algo).framework
    checkpoint = discover_model_checkpoint(
        run_path,
        resolved_algo,
        checkpoint_path=checkpoint_path,
    )

    model_metadata = dict(metadata or {})
    model_metadata.update(
        _source_run_metadata(
            run_path=run_path,
            run_metadata_path=run_metadata_path,
            run_metadata=run_metadata,
        )
    )
    # Preserve the replay contract so policy loading can detect drift.
    landscaping_contract = run_metadata.get("landscaping_contract")
    if isinstance(landscaping_contract, dict):
        model_metadata.setdefault("landscaping_contract", landscaping_contract)

    if consumed_demo_replay is not None:
        model_metadata["demo_replay"] = consumed_demo_replay.block
        dataset_revision = _resolve_consumed_demo_revision(
            consumed_demo_replay, dataset_revision=dataset_revision
        )
        if training_dataset_path is not None:
            # Keep the artifact bound to the demo recorded by the run.
            _require_consumed_demo_dataset(consumed_demo_replay, training_dataset_path)
        else:
            (
                dataset_repo_id,
                dataset_revision,
                consumed_dataset_metadata,
            ) = _resolve_consumed_demo_provenance(
                consumed_demo_replay,
                dataset_repo_id=dataset_repo_id,
                dataset_revision=dataset_revision,
            )
            model_metadata.update(consumed_dataset_metadata)

    framework_config_artifact = discover_framework_config_artifact(
        run_path,
        resolved_algo,
        run_metadata,
    )
    if framework_config_artifact is not None:
        model_metadata.setdefault(
            "framework_config_path",
            framework_config_artifact.artifact_relpath,
        )

    artifact_members: dict[str, Path] = {}
    if resolved_framework in {"sb3", "sbx"}:
        normalization = discover_rl_zoo_normalization_artifact(
            run_path,
            framework_config_artifact,
            checkpoint=checkpoint,
            algo_id=resolved_algo,
        )
        existing_normalization = model_metadata.get(RL_ZOO_NORMALIZATION_METADATA_KEY)
        if normalization.metadata is not None:
            if (
                existing_normalization is not None
                and existing_normalization != normalization.metadata
            ):
                raise ValueError(
                    "run metadata rl_zoo_normalization conflicts with the saved "
                    "RL-Zoo config"
                )
            model_metadata[RL_ZOO_NORMALIZATION_METADATA_KEY] = dict(
                normalization.metadata
            )
        elif existing_normalization is not None:
            raise ValueError(
                "rl_zoo_normalization requires explicit RL-Zoo normalize evidence"
            )
        if normalization.stats_source_path is not None:
            artifact_members["normalization/vecnormalize.pkl"] = (
                normalization.stats_source_path
            )

    result = prepare_model_artifact(
        checkpoint_path=checkpoint,
        output_dir=output_dir,
        identity=identity,
        framework=framework,
        training_dataset_path=training_dataset_path,
        dataset_repo_id=dataset_repo_id,
        dataset_revision=dataset_revision,
        metrics=metrics,
        metadata=model_metadata,
        hf_repo_id=hf_repo_id,
        checkpoint_scope=checkpoint_scope,
        _framework_config_artifact=framework_config_artifact,
        artifact_members=artifact_members,
    )
    return result


def discover_model_checkpoint(
    run_dir: str | Path,
    algo: str,
    *,
    checkpoint_path: str | Path | None = None,
) -> Path:
    """Discover the checkpoint to package for a known SRB algorithm layout."""
    run_path = Path(run_dir)
    if checkpoint_path is not None:
        checkpoint = Path(checkpoint_path)
        if not checkpoint.is_absolute():
            checkpoint = run_path / checkpoint
        if not (checkpoint.is_file() or checkpoint.is_dir()):
            raise FileNotFoundError(f"checkpoint does not exist: {checkpoint}")
        return checkpoint

    discovered_checkpoint = discover_run_checkpoint(
        run_path,
        slugify_underscore(algo),
        workflow="eval",
        continue_training=False,
        required=True,
    )
    if discovered_checkpoint is None:
        raise FileNotFoundError(f"No checkpoint found under {run_path}")
    return discovered_checkpoint


def write_model_artifact_manifest(
    manifest: ModelArtifactManifest,
    path: str | Path,
) -> Path:
    """Write a model artifact manifest as deterministic JSON."""
    if manifest.schema_version >= MODEL_MANIFEST_CURRENT_SCHEMA_VERSION:
        raise ValueError(
            "schema_version 3 model manifests must be published as a complete "
            "artifact generation"
        )
    manifest_path = Path(path)
    payload = _deterministic_json_bytes(manifest.to_dict())
    artifact_root = (
        manifest_path.parent.parent
        if manifest_path.parent.name == "meta"
        else manifest_path.parent
    )
    return ArtifactPublication.file(
        manifest_path,
        lock_path=_model_artifact_lock_path(artifact_root),
    ).publish_bytes(payload)


def _read_consumed_demo_replay(
    run_metadata: Mapping[str, Any],
) -> _ConsumedDemoReplay | None:
    """Validate the persisted demo binding without reopening its source path."""
    if "demo_replay" not in run_metadata:
        run_metadata_section = run_metadata.get("run")
        has_demo_request = (
            "demo_dataset" in run_metadata or "demo_max_transitions" in run_metadata
        )
        if isinstance(run_metadata_section, Mapping):
            has_demo_request = has_demo_request or (
                "demo_dataset" in run_metadata_section
                or "demo_max_transitions" in run_metadata_section
            )
        if has_demo_request:
            raise ValueError(
                "run metadata demo inputs are missing the persisted demo_replay binding"
            )
        return None
    raw_block = run_metadata["demo_replay"]
    block = _copy_json_mapping(raw_block, field_name="demo_replay")

    _required_nonempty_string(
        block.get("dataset"),
        field_name="demo_replay dataset",
    )
    max_transitions = _required_positive_integer(
        block.get("max_transitions"),
        field_name="demo_replay max_transitions",
    )
    loaded_transitions = _required_positive_integer(
        block.get("loaded_transitions"),
        field_name="demo_replay loaded_transitions",
    )
    if loaded_transitions > max_transitions:
        raise ValueError("demo_replay loaded_transitions exceeds max_transitions")
    dataset_fingerprint = _required_sha256(
        block.get("dataset_fingerprint"),
        field_name="demo_replay dataset_fingerprint",
    )

    binding = _copy_json_mapping(
        block.get("binding"),
        field_name="demo_replay binding",
    )
    binding_dataset_fingerprint = binding.get("dataset_fingerprint")
    if binding_dataset_fingerprint is None:
        raise ValueError("demo_replay binding is missing dataset_fingerprint")
    if (
        _required_sha256(
            binding_dataset_fingerprint,
            field_name="demo_replay binding dataset_fingerprint",
        )
        != dataset_fingerprint
    ):
        raise ValueError(
            "demo_replay binding dataset_fingerprint does not match demo_replay"
        )
    binding_transition_count = binding.get("transition_count")
    if type(binding_transition_count) is not int or binding_transition_count < 1:
        raise ValueError(
            "demo_replay binding transition_count must be a positive integer"
        )
    if binding_transition_count != loaded_transitions:
        raise ValueError(
            "demo_replay binding transition_count does not match loaded_transitions"
        )

    source_binding = binding
    if "dataset" in binding:
        if any(
            key in binding
            for key in ("content_binding", "manifest", "manifest_fingerprint")
        ):
            raise ValueError("demo_replay binding has ambiguous source records")
        source_binding = _copy_json_mapping(
            binding["dataset"], field_name="demo_replay binding dataset"
        )
        if (
            source_binding.get("dataset_fingerprint") != dataset_fingerprint
            or type(source_binding.get("transition_count")) is not int
            or source_binding["transition_count"] != loaded_transitions
        ):
            raise ValueError(
                "demo_replay binding dataset does not match the consumed source summary"
            )

    content_binding = _copy_json_mapping(
        source_binding.get("content_binding"),
        field_name="demo_replay binding content_binding",
    )
    manifest_sha256 = _required_sha256(
        content_binding.get("manifest_sha256"),
        field_name="demo_replay binding manifest_sha256",
    )
    member_fingerprints = content_binding.get("members")
    if not isinstance(member_fingerprints, Mapping) or not member_fingerprints:
        raise ValueError("demo_replay binding members must be a non-empty object")
    normalized_members: dict[str, str] = {}
    for member, member_hash in member_fingerprints.items():
        if not isinstance(member, str) or not member.strip():
            raise ValueError(
                "demo_replay binding member names must be non-empty strings"
            )
        member_path = Path(member)
        if (
            "\x00" in member
            or "\\" in member
            or member_path.is_absolute()
            or member_path.as_posix() != member
            or any(part in {"", ".", ".."} for part in member_path.parts)
        ):
            raise ValueError(
                f"demo_replay binding member path is not canonical: {member!r}"
            )
        normalized_members[member] = _required_sha256(
            member_hash,
            field_name=f"demo_replay binding member {member!r}",
        )
    content_sha256 = _required_sha256(
        content_binding.get("content_sha256"),
        field_name="demo_replay binding content_sha256",
    )
    if dataset_fingerprint != content_sha256:
        raise ValueError(
            "demo_replay dataset_fingerprint does not match content_sha256"
        )
    from srb.utils.dataset_manifest import demo_content_fingerprint

    expected_content_sha256 = demo_content_fingerprint(
        manifest_fingerprint=manifest_sha256,
        member_fingerprints=normalized_members,
    )
    if expected_content_sha256 != content_sha256:
        raise ValueError(
            "demo_replay binding content_sha256 does not match manifest and members"
        )
    manifest_fingerprint = source_binding.get("manifest_fingerprint")
    if (
        manifest_fingerprint is not None
        and _required_sha256(
            manifest_fingerprint,
            field_name="demo_replay binding manifest_fingerprint",
        )
        != manifest_sha256
    ):
        raise ValueError(
            "demo_replay binding manifest_fingerprint does not match "
            "content_binding manifest_sha256"
        )
    binding_fingerprint = source_binding.get("fingerprint")
    if (
        binding_fingerprint is not None
        and _required_sha256(
            binding_fingerprint,
            field_name="demo_replay binding fingerprint",
        )
        != content_sha256
    ):
        raise ValueError(
            "demo_replay binding fingerprint does not match content_sha256"
        )

    manifest = _copy_json_mapping(
        source_binding.get("manifest"),
        field_name="demo_replay binding manifest",
    )
    _required_nonempty_string(
        manifest.get("hf_repo_id"),
        field_name="demo_replay binding manifest hf_repo_id",
    )
    split = _required_nonempty_string(
        manifest.get("split"),
        field_name="demo_replay binding manifest split",
    )
    try:
        DatasetSplit(split)
    except ValueError as exc:
        raise ValueError(
            f"demo_replay binding manifest split is unknown: {split!r}"
        ) from exc
    storage_format = _required_nonempty_string(
        manifest.get("storage_format"),
        field_name="demo_replay binding manifest storage_format",
    )
    try:
        StorageFormat(storage_format)
    except ValueError as exc:
        raise ValueError(
            "demo_replay binding manifest storage_format is unknown: "
            f"{storage_format!r}"
        ) from exc
    _required_sha256(
        manifest.get("artifact_hash"),
        field_name="demo_replay binding manifest artifact_hash",
    )
    if "revision" not in manifest:
        raise ValueError("demo_replay binding manifest is missing revision")
    if manifest["revision"] is not None:
        _required_nonempty_string(
            manifest["revision"],
            field_name="demo_replay binding manifest revision",
        )

    identity_mapping = _copy_json_mapping(
        manifest.get("identity"),
        field_name="demo_replay binding manifest identity",
    )
    identity: dict[str, str] = {}
    for field_name in ("task", "embodiment", "action_mode"):
        identity[field_name] = _required_nonempty_string(
            identity_mapping.get(field_name),
            field_name=f"demo_replay binding manifest identity {field_name}",
        )
    if "algo" in identity_mapping and identity_mapping["algo"] is not None:
        _required_nonempty_string(
            identity_mapping["algo"],
            field_name="demo_replay binding manifest identity algo",
        )

    return _ConsumedDemoReplay(
        block=block,
        manifest=manifest,
        identity=identity,
    )


def _copy_json_mapping(value: object, *, field_name: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field_name} must be an object")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field_name} keys must be strings")
    try:
        copied = json.loads(json.dumps(dict(value), allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must contain finite JSON values") from exc
    if not isinstance(copied, dict):  # pragma: no cover - json object invariant
        raise ValueError(f"{field_name} must be an object")
    return copied


def _required_nonempty_string(value: object, *, field_name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string")
    return value


def _required_positive_integer(value: object, *, field_name: str) -> int:
    if type(value) is not int or value < 1:
        raise ValueError(f"{field_name} must be a positive integer")
    return value


def _required_sha256(value: object, *, field_name: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(char not in "0123456789abcdef" for char in value)
    ):
        raise ValueError(f"{field_name} must be a lowercase SHA256 hex digest")
    return value


def _resolve_training_dataset_provenance(
    *,
    training_dataset_path: str | Path | None,
    dataset_repo_id: str | None,
    dataset_revision: str | None,
) -> tuple[str | None, str | None, dict[str, Any]]:
    if training_dataset_path is None:
        return dataset_repo_id, dataset_revision, {}

    from srb.utils.dataset_manifest import find_dataset_manifest, read_dataset_manifest

    manifest_path = find_dataset_manifest(training_dataset_path)
    dataset_manifest = read_dataset_manifest(manifest_path)
    if dataset_repo_id is not None and dataset_repo_id != dataset_manifest.hf_repo_id:
        raise ValueError(
            f"dataset_repo_id {dataset_repo_id!r} does not match dataset "
            f"manifest hf_repo_id {dataset_manifest.hf_repo_id!r}"
        )

    resolved_revision = dataset_revision or _training_dataset_revision_from_metadata(
        dataset_manifest.metadata
    )
    return (
        dataset_repo_id or dataset_manifest.hf_repo_id,
        resolved_revision,
        {
            "training_dataset_artifact_hash": dataset_manifest.artifact_hash,
            "training_dataset_split": dataset_manifest.split.value,
            "training_dataset_storage_format": dataset_manifest.storage_format.value,
        },
    )


def _resolve_consumed_demo_provenance(
    consumed_demo_replay: _ConsumedDemoReplay,
    *,
    dataset_repo_id: str | None,
    dataset_revision: str | None,
) -> tuple[str, str | None, dict[str, Any]]:
    """Derive dataset labels from the immutable binding recorded by training."""
    manifest = consumed_demo_replay.manifest
    recorded_repo_id = _required_nonempty_string(
        manifest.get("hf_repo_id"),
        field_name="demo_replay binding manifest hf_repo_id",
    )
    if dataset_repo_id is not None and dataset_repo_id != recorded_repo_id:
        raise ValueError(
            f"dataset_repo_id {dataset_repo_id!r} does not match consumed demo "
            f"manifest hf_repo_id {recorded_repo_id!r}"
        )

    recorded_revision = manifest.get("revision")
    if recorded_revision is not None:
        recorded_revision = _required_nonempty_string(
            recorded_revision,
            field_name="demo_replay binding manifest revision",
        )
    return (
        dataset_repo_id or recorded_repo_id,
        dataset_revision or recorded_revision,
        {
            "training_dataset_artifact_hash": _required_sha256(
                manifest.get("artifact_hash"),
                field_name="demo_replay binding manifest artifact_hash",
            ),
            "training_dataset_split": _required_nonempty_string(
                manifest.get("split"),
                field_name="demo_replay binding manifest split",
            ),
            "training_dataset_storage_format": _required_nonempty_string(
                manifest.get("storage_format"),
                field_name="demo_replay binding manifest storage_format",
            ),
        },
    )


def _resolve_consumed_demo_revision(
    consumed_demo_replay: _ConsumedDemoReplay,
    *,
    dataset_revision: str | None,
) -> str | None:
    """Return the dataset revision, refusing one that contradicts the record."""
    recorded_revision = consumed_demo_replay.manifest.get("revision")
    if recorded_revision is None:
        return dataset_revision
    recorded_revision = _required_nonempty_string(
        recorded_revision,
        field_name="demo_replay binding manifest revision",
    )
    if dataset_revision is not None and dataset_revision != recorded_revision:
        raise ValueError(
            f"dataset_revision {dataset_revision!r} does not match consumed demo "
            f"manifest revision {recorded_revision!r}"
        )
    return recorded_revision


def _require_consumed_demo_dataset(
    consumed_demo_replay: _ConsumedDemoReplay,
    training_dataset_path: str | Path,
) -> None:
    """Refuse a training dataset that is not the demo the run consumed."""
    from srb.utils.dataset_manifest import find_dataset_manifest, read_dataset_manifest

    dataset_manifest = read_dataset_manifest(
        find_dataset_manifest(training_dataset_path)
    )
    recorded = consumed_demo_replay.manifest
    supplied_identity = {
        "task": dataset_manifest.identity.task,
        "embodiment": dataset_manifest.identity.embodiment,
        "action_mode": dataset_manifest.identity.action_mode,
    }
    for field_name, supplied, expected in (
        ("hf_repo_id", dataset_manifest.hf_repo_id, recorded.get("hf_repo_id")),
        (
            "artifact_hash",
            dataset_manifest.artifact_hash,
            recorded.get("artifact_hash"),
        ),
        ("identity", supplied_identity, consumed_demo_replay.identity),
    ):
        if supplied != expected:
            raise ValueError(
                f"training dataset {field_name} {supplied!r} does not match the "
                f"consumed demo manifest {field_name} {expected!r} recorded by "
                f"the run; pass the dataset the run consumed or omit --dataset"
            )


def _training_dataset_revision_from_metadata(
    metadata: Mapping[str, Any],
) -> str | None:
    for key in ("hf_revision", "dataset_revision", "revision"):
        value = metadata.get(key)
        if value is not None and str(value).strip():
            return str(value)
    return None


def _read_run_metadata(path: Path) -> dict[str, Any]:
    data = ArtifactPublication.file(path).read(
        lambda payload: json.loads(payload.decode("utf-8"))
    )
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"run metadata must be a JSON object: {path}")
    return data


def _resolve_run_algorithm(
    *,
    explicit_algo: str | None,
    run_metadata: dict[str, Any],
) -> str:
    metadata_algo = _nested_str(run_metadata, "run", "algorithm")
    if metadata_algo is None:
        metadata_algo = _nested_str(run_metadata, "run", "algo")
    if metadata_algo is None:
        metadata_algo = _top_level_str(run_metadata, "algorithm")

    if explicit_algo is not None:
        normalized_algo = slugify_underscore(explicit_algo)
        if metadata_algo is not None and normalized_algo != slugify_underscore(
            metadata_algo
        ):
            raise ValueError(
                f"algo {explicit_algo!r} does not match run metadata "
                f"algorithm {metadata_algo!r}"
            )
        return normalized_algo

    if metadata_algo is None:
        raise InvalidInputError(
            "model artifact from run requires --algo or metadata.json run.algorithm",
            component="srb.experiments.models",
        )
    return slugify_underscore(metadata_algo)


def _resolve_run_task(
    *,
    explicit_task: str | None,
    run_metadata: dict[str, Any],
) -> str | None:
    """Resolve the task from metadata; return ``None`` if absent."""
    metadata_task = _nested_str(run_metadata, "run", "env_id")
    if metadata_task is not None:
        metadata_task = metadata_task.rsplit("/", 1)[-1] or None

    if explicit_task is None:
        return metadata_task
    if metadata_task is not None and slugify_underscore(
        explicit_task
    ) != slugify_underscore(metadata_task):
        raise InvalidInputError(
            f"task {explicit_task!r} does not match run metadata env_id "
            f"{metadata_task!r}",
            component="srb.experiments.models",
        )
    return explicit_task


def _resolve_run_model_identity(
    *,
    task: str | None,
    embodiment: str | None,
    action_mode: str | None,
    algo: str,
    training_dataset_path: str | Path | None,
    consumed_identity: Mapping[str, str] | None = None,
) -> ArtifactIdentity:
    if training_dataset_path is None:
        if consumed_identity is not None:
            return ArtifactIdentity(
                task=_resolve_consumed_identity_value(
                    field_name="task",
                    explicit_value=task,
                    recorded_value=consumed_identity["task"],
                ),
                embodiment=_resolve_consumed_identity_value(
                    field_name="embodiment",
                    explicit_value=embodiment,
                    recorded_value=consumed_identity["embodiment"],
                ),
                action_mode=_resolve_consumed_identity_value(
                    field_name="action_mode",
                    explicit_value=action_mode,
                    recorded_value=consumed_identity["action_mode"],
                ),
                algo=algo,
            )
        missing = [
            flag
            for flag, value in (
                ("--task", task),
                ("--embodiment", embodiment),
                ("--action-mode", action_mode),
            )
            if value is None
        ]
        if missing:
            raise InvalidInputError(
                "model artifact from run requires "
                + ", ".join(missing)
                + " unless a training dataset is provided; pass them explicitly",
                component="srb.experiments.models",
            )
        return ArtifactIdentity(
            task=task,  # type: ignore[arg-type]
            embodiment=embodiment,  # type: ignore[arg-type]
            action_mode=action_mode,  # type: ignore[arg-type]
            algo=algo,
        )

    from srb.utils.dataset_manifest import find_dataset_manifest, read_dataset_manifest

    manifest = read_dataset_manifest(find_dataset_manifest(training_dataset_path))
    return ArtifactIdentity(
        task=_resolve_dataset_identity_value(
            field_name="task",
            explicit_value=task,
            manifest_value=manifest.identity.task,
        ),
        embodiment=_resolve_dataset_identity_value(
            field_name="embodiment",
            explicit_value=embodiment,
            manifest_value=manifest.identity.embodiment,
        ),
        action_mode=_resolve_dataset_identity_value(
            field_name="action_mode",
            explicit_value=action_mode,
            manifest_value=manifest.identity.action_mode,
        ),
        algo=algo,
    )


def _resolve_consumed_identity_value(
    *,
    field_name: str,
    explicit_value: str | None,
    recorded_value: str,
) -> str:
    if explicit_value is None:
        return recorded_value
    if slugify_underscore(explicit_value) != slugify_underscore(recorded_value):
        raise ValueError(
            f"{field_name} {explicit_value!r} does not match consumed demo "
            f"manifest {field_name} {recorded_value!r}"
        )
    return explicit_value


def _resolve_dataset_identity_value(
    *,
    field_name: str,
    explicit_value: str | None,
    manifest_value: str,
) -> str:
    if explicit_value is None:
        return manifest_value
    if slugify_underscore(explicit_value) != manifest_value:
        raise ValueError(
            f"{field_name} {explicit_value!r} does not match dataset "
            f"manifest {field_name} {manifest_value!r}"
        )
    return explicit_value


def _source_run_metadata(
    *,
    run_path: Path,
    run_metadata_path: Path,
    run_metadata: dict[str, Any],
) -> dict[str, Any]:
    metadata: dict[str, Any] = {"source_run_dir": run_path.as_posix()}
    if run_metadata_path.is_file():
        metadata["source_run_metadata_path"] = run_metadata_path.as_posix()

    for output_key, metadata_key in (
        ("source_run_algorithm", "algorithm"),
        ("source_run_workflow", "workflow"),
        ("source_run_env_id", "env_id"),
    ):
        value = _nested_str(run_metadata, "run", metadata_key)
        if value is not None:
            metadata[output_key] = value
    return metadata


def _nested_str(
    data: dict[str, Any],
    section: str,
    key: str,
) -> str | None:
    section_value = data.get(section)
    if not isinstance(section_value, dict):
        return None
    value = section_value.get(key)
    if value is None or not str(value).strip():
        return None
    return str(value)


def _top_level_str(data: dict[str, Any], key: str) -> str | None:
    value = data.get(key)
    if value is None or not str(value).strip():
        return None
    return str(value)


def _infer_framework(algo: str) -> str:
    try:
        return resolve_algorithm_adapter(slugify_underscore(algo)).framework
    except ValueError:
        return "unknown"


__all__ = [
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
    "ModelArtifactPreparationResult",
    "ModelArtifactReference",
    "ModelArtifactValidationReport",
    "ModelCheckpointSnapshot",
    "artifact_member_path",
    "build_model_artifact_manifest",
    "discover_model_checkpoint",
    "find_model_artifact_manifest",
    "model_artifact_root_from_manifest_path",
    "normalize_artifact_relpath",
    "prepare_model_artifact",
    "prepare_model_artifact_from_run",
    "read_model_artifact_manifest",
    "try_model_artifact_reference",
    "validate_model_artifact_layout",
    "write_model_artifact_manifest",
    "_ELEMENTS_DONE_MARKER",
    "_agent_payload_members",
    "_artifact_tree_entries",
    "_build_artifact_members_descriptor",
    "_copy_model_file_snapshot",
    "_deterministic_json_bytes",
    "_load_model_artifact_manifest",
    "_model_artifact_lock_path",
    "_stream_model_member_identity",
]
