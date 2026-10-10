"""Hugging Face Hub publication helpers for SRB datasets."""

from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from srb.datasets.manifest import (
    DatasetManifest,
    DatasetSnapshot,
    _open_dataset_publication_for_path,
    _validate_dataset_snapshot,
    _validate_manifest_contract,
    dataset_root_from_manifest_path,
    find_dataset_manifest,
    open_dataset_snapshot,
    read_dataset_manifest,
    resolve_dataset_path,
)
from srb.datasets.schema import StorageFormat
from srb.utils.artifact_identity import ArtifactIdentity
from srb.utils.failures import InvalidInputError
from srb.utils.huggingface import (
    HuggingFaceHubCardResult,
    HuggingFaceHubDownloadPlan,
    HuggingFaceHubUploadPlan,
    download_huggingface_snapshot,
    format_huggingface_card_title,
    format_huggingface_metadata_value,
    generated_huggingface_card_files,
    raise_for_blocking_huggingface_publication_warnings,
    require_huggingface_hub,
    upload_huggingface_folder,
)
from srb.utils.json_types import thaw_json_value
from srb.utils.publication import (
    ArtifactPublication,
    PublicationError,
    directory_member_hashes,
    materialize_file_snapshot,
)

_PUBLICATION_STORAGE_FORMATS = (StorageFormat.SRB_JSONL, StorageFormat.LEROBOT)
_BLOCKING_PUBLICATION_WARNING_PREFIXES = (
    "policy dataset does not identify the generating algorithm or checkpoint",
)


@dataclass(frozen=True)
class HuggingFaceUploadPlan(HuggingFaceHubUploadPlan[DatasetManifest]):
    """Resolved files and repository target for a dataset upload."""

    _dataset_snapshot: DatasetSnapshot | None = field(
        default=None,
        repr=False,
        compare=False,
    )

    def __post_init__(self) -> None:
        # Snapshot-owning plans without an upload directory need base validation.
        if self.upload_dir is None and (
            self._snapshot is not None or self._dataset_snapshot is not None
        ):
            super().__post_init__()

    def close(self) -> None:
        try:
            super().close()
        finally:
            if self._dataset_snapshot is not None:
                self._dataset_snapshot.close()


class HuggingFaceDatasetCardResult(HuggingFaceHubCardResult[DatasetManifest]):
    """Result of writing a Hugging Face dataset card."""


def pull_dataset_from_hub(
    *,
    repo_id: str | None = None,
    task: str | None = None,
    embodiment: str | None = None,
    action_mode: str | None = None,
    output: str | Path | None = None,
    revision: str | None = None,
    dry_run: bool = False,
) -> HuggingFaceHubDownloadPlan:
    """Download an SRB dataset repo from the Hub into a local directory.

    Supply ``repo_id`` or all identity fields. Identity-based names follow
    ``srb dataset push`` via :class:`ArtifactIdentity`. Dry-run mode returns the
    resolved plan without importing optional Hub dependencies or performing
    network operations.
    """
    if repo_id is None:
        if not (task and embodiment and action_mode):
            raise InvalidInputError(
                message="dataset pull requires --repo-id or "
                "--task + --embodiment + --action-mode",
                code="invalid_input.dataset_cli",
                component="cli.dataset",
            )
        identity = ArtifactIdentity(
            task=task,
            embodiment=embodiment,
            action_mode=action_mode,
        )
        repo_id = identity.dataset_repo_id()
    if "/" not in repo_id:
        raise ValueError(f"--repo-id must be '<org>/<name>', got {repo_id!r}")
    local_dir = Path(output) if output is not None else Path(repo_id.rsplit("/", 1)[-1])
    plan = HuggingFaceHubDownloadPlan(
        repo_id=repo_id,
        repo_type="dataset",
        local_dir=local_dir,
        revision=revision,
    )
    if dry_run:
        return plan

    require_huggingface_hub("Dataset")
    _, resolved_sha = download_huggingface_snapshot(
        repo_id=plan.repo_id,
        repo_type=plan.repo_type,
        local_dir=plan.local_dir,
        revision=plan.revision,
    )
    # Return the resolved commit SHA, not the requested branch or tag.
    return replace(plan, revision=resolved_sha)


def plan_huggingface_upload(
    dataset_path: str | Path,
    *,
    repo_id: str | None = None,
) -> HuggingFaceUploadPlan:
    """Deep-validate one owned dataset generation and plan its upload."""
    manifest_path = find_dataset_manifest(dataset_path)
    with ExitStack() as cleanup:
        snapshot = open_dataset_snapshot(manifest_path)
        cleanup.callback(snapshot.close)
        report = _validate_dataset_snapshot(snapshot, deep=True, strict=False)
        manifest = report.manifest
        if not report.valid or manifest is None:
            raise ValueError("dataset is not publishable: " + "; ".join(report.errors))
        _require_publication_format(manifest)
        raise_for_blocking_huggingface_publication_warnings(
            report.warnings,
            blocking_prefixes=_BLOCKING_PUBLICATION_WARNING_PREFIXES,
            artifact_label="dataset",
        )
        files = [
            path
            for path in sorted(snapshot.member_hashes)
            if not _is_publication_control_file(path)
        ]
        local_dir = dataset_root_from_manifest_path(manifest_path)
        owned_root = snapshot._materialized.path if snapshot._materialized else None
        complete_owned_tree = False
        if owned_root is not None and snapshot.manifest_path is not None:
            try:
                snapshot.manifest_path.resolve().relative_to(owned_root.resolve())
                snapshot.dataset_path.resolve().relative_to(owned_root.resolve())
                complete_owned_tree = snapshot.manifest_path.is_file()
            except ValueError:
                complete_owned_tree = False
        upload_snapshot = None
        if complete_owned_tree and owned_root is not None:
            # Reuse the complete validated view to avoid reopening the publication.
            for private_path in owned_root.rglob("*"):
                if private_path.is_file() and _is_publication_control_file(
                    private_path.relative_to(owned_root)
                ):
                    private_path.unlink()
            upload_dir = owned_root
        else:
            # Legacy snapshots hold only referenced data and manifest bytes;
            # rebuild the conventional root from those paths.
            if snapshot.external_reference:
                raise PublicationError(
                    "cannot upload a legacy dataset whose data reference escapes "
                    "its manifest root; export it into a contained dataset first"
                )
            data_relative = Path(manifest.dataset_path)
            if data_relative.is_absolute() or ".." in data_relative.parts:
                raise PublicationError(
                    "cannot upload a legacy dataset whose data path escapes "
                    "its manifest root"
                )
            manifest_relative = manifest_path.resolve().relative_to(local_dir.resolve())
            file_sources: dict[str, tuple[Path, str]] = {}
            for source_file in sorted(snapshot.dataset_path.rglob("*")):
                if not source_file.is_file():
                    continue
                relative = (
                    data_relative / source_file.relative_to(snapshot.dataset_path)
                ).as_posix()
                _identity_name, digest = snapshot.member_identity(source_file)
                file_sources[relative] = (source_file, digest)
            if snapshot.manifest_path is None:
                raise PublicationError("dataset snapshot has no manifest file")
            _identity_name, manifest_digest = snapshot.member_identity(
                snapshot.manifest_path
            )
            file_sources[manifest_relative.as_posix()] = (
                snapshot.manifest_path,
                manifest_digest,
            )
            upload_snapshot = materialize_file_snapshot(file_sources)
            cleanup.callback(upload_snapshot.close)
            upload_dir = upload_snapshot.path
        plan = HuggingFaceUploadPlan(
            repo_id=repo_id or manifest.hf_repo_id,
            repo_type="dataset",
            local_dir=local_dir,
            files=files,
            manifest=manifest,
            upload_dir=upload_dir,
            _snapshot=upload_snapshot,
            _dataset_snapshot=snapshot,
        )
        cleanup.pop_all()
        return plan


def _plan_huggingface_upload_dry_run(
    dataset_path: str | Path,
    *,
    repo_id: str | None = None,
) -> HuggingFaceUploadPlan:
    """Resolve bounded file inventory without snapshots or payload validation."""
    manifest_path = find_dataset_manifest(dataset_path)
    manifest = read_dataset_manifest(manifest_path)
    _require_publication_format(manifest)
    errors: list[str] = []
    warnings: list[str] = []
    _validate_manifest_contract(manifest, errors=errors, warnings=warnings)
    if errors:
        raise ValueError("dataset is not publishable: " + "; ".join(errors))
    raise_for_blocking_huggingface_publication_warnings(
        warnings,
        blocking_prefixes=_BLOCKING_PUBLICATION_WARNING_PREFIXES,
        artifact_label="dataset",
    )
    local_dir = dataset_root_from_manifest_path(manifest_path)
    try:
        referenced = resolve_dataset_path(manifest.dataset_path, manifest_path)
        if not referenced.resolve().is_relative_to(local_dir.resolve()):
            raise ValueError(
                "dataset reference escapes its manifest root; external references "
                "cannot be uploaded"
            )
    except (OSError, RuntimeError) as exc:
        raise ValueError(f"could not resolve dataset reference: {exc}") from exc
    publication = _open_dataset_publication_for_path(manifest_path)
    if publication is not None:
        hashes = dict(publication.member_hashes)
    else:
        inventory = directory_member_hashes(local_dir)
        relative_manifest = (
            manifest_path.resolve().relative_to(local_dir.resolve()).as_posix()
        )
        referenced_relative = (
            referenced.resolve().relative_to(local_dir.resolve()).as_posix()
        )
        if manifest.storage_format == StorageFormat.LEROBOT:
            prefixes = ("data/", "meta/")
            hashes = {
                path: digest
                for path, digest in inventory.items()
                if path == relative_manifest or path.startswith(prefixes)
            }
        elif referenced.is_dir():
            prefix = referenced_relative.rstrip("/") + "/"
            hashes = {
                path: digest
                for path, digest in inventory.items()
                if path == relative_manifest or path.startswith(prefix)
            }
        else:
            hashes = {
                path: digest
                for path, digest in inventory.items()
                if path in {relative_manifest, referenced_relative}
            }
    files = [path for path in sorted(hashes) if not _is_publication_control_file(path)]
    return HuggingFaceUploadPlan(
        repo_id=repo_id or manifest.hf_repo_id,
        repo_type="dataset",
        local_dir=local_dir,
        files=files,
        manifest=manifest,
    )


def _require_publication_format(manifest: DatasetManifest) -> None:
    if manifest.storage_format in _PUBLICATION_STORAGE_FORMATS:
        return
    expected = ", ".join(
        storage_format.value for storage_format in _PUBLICATION_STORAGE_FORMATS
    )
    raise ValueError(
        "dataset is not publication-ready: "
        f"storage_format={manifest.storage_format.value}. "
        f"Export to {expected} before pushing to Hugging Face."
    )


def build_huggingface_dataset_card(
    manifest: DatasetManifest,
) -> str:
    """Build a deterministic Hugging Face dataset card for an SRB dataset."""
    metadata = _thawed_manifest_metadata(manifest)
    title = format_huggingface_card_title(manifest.identity.dataset_name)
    lines = _dataset_card_front_matter(manifest=manifest, title=title) + [
        "",
        f"# {title}",
        "",
        "## Dataset Summary",
        "",
        (
            "This dataset was produced with Space Robotics Bench (SRB) and is "
            "packaged for robot learning, machine learning, and computer vision "
            "workflows."
        ),
        "",
        "## SRB Identity",
        "",
        "| Field | Value |",
        "| --- | --- |",
        f"| Task | `{manifest.identity.task}` |",
        f"| Embodiment | `{manifest.identity.embodiment}` |",
        f"| Action mode | `{manifest.identity.action_mode}` |",
        f"| Algorithm | `{manifest.identity.algo or ''}` |",
        f"| Split | `{manifest.split.value}` |",
        f"| Controller | `{manifest.controller}` |",
        f"| Storage format | `{manifest.storage_format.value}` |",
        f"| Dataset path | `{manifest.dataset_path}` |",
        "",
        *_dataset_card_action_schema_lines(metadata.get("action_schema")),
        "## Validation",
        "",
        "```bash",
        "srb dataset validate . --deep",
        "srb dataset replay .",
        "```",
    ]
    if metadata:
        lines.extend(
            [
                "",
                "## Provenance Metadata",
                "",
                "| Key | Value |",
                "| --- | --- |",
            ]
        )
        for key, value in sorted(metadata.items()):
            if key == "action_schema":
                continue
            lines.append(f"| `{key}` | `{format_huggingface_metadata_value(value)}` |")
    lines.append("")
    return "\n".join(lines)


def write_huggingface_dataset_card(
    dataset_path: str | Path,
    *,
    output_path: str | Path | None = None,
    overwrite: bool = False,
) -> HuggingFaceDatasetCardResult:
    """Write a Hugging Face README.md dataset card beside a publishable dataset."""
    manifest_path = find_dataset_manifest(dataset_path)
    manifest = read_dataset_manifest(manifest_path)
    _require_publication_format(manifest)
    local_dir = dataset_root_from_manifest_path(manifest_path)
    card_path = (
        Path(output_path) if output_path is not None else local_dir / "README.md"
    )
    published_dataset = _open_dataset_publication_for_path(manifest_path)
    if published_dataset is not None:
        relative_path: str | None
        try:
            relative_path = (
                card_path.resolve().relative_to(local_dir.resolve()).as_posix()
            )
        except ValueError:
            relative_path = None
        if (
            relative_path == "README.md"
            and relative_path in published_dataset.member_hashes
        ):
            if overwrite:
                raise PublicationError(
                    "cannot replace a card inside an immutable published dataset: "
                    f"{local_dir}"
                )
            published_dataset.read_member_bytes(relative_path)
            return HuggingFaceDatasetCardResult(
                path=card_path,
                written=False,
                repo_id=manifest.hf_repo_id,
                manifest=manifest,
            )
        if relative_path is not None:
            raise PublicationError(
                "immutable published dataset has no README.md; pass --output to "
                "write the card outside the publication"
            )
        if not overwrite:
            with ArtifactPublication.file(card_path).open_snapshot() as snapshot:
                if snapshot is not None:
                    return HuggingFaceDatasetCardResult(
                        path=card_path,
                        written=False,
                        repo_id=manifest.hf_repo_id,
                        manifest=manifest,
                    )
    else:
        plan = plan_huggingface_upload(manifest_path)
        try:
            if not overwrite:
                with ArtifactPublication.file(card_path).open_snapshot() as existing:
                    if existing is not None:
                        return HuggingFaceDatasetCardResult(
                            path=card_path,
                            written=False,
                            repo_id=plan.repo_id,
                            manifest=plan.manifest,
                        )
            ArtifactPublication.file(card_path).publish_bytes(
                build_huggingface_dataset_card(plan.manifest).encode("utf-8")
            )
            return HuggingFaceDatasetCardResult(
                path=card_path,
                written=True,
                repo_id=plan.repo_id,
                manifest=plan.manifest,
            )
        finally:
            plan.close()

    ArtifactPublication.file(card_path).publish_bytes(
        build_huggingface_dataset_card(manifest).encode("utf-8")
    )
    return HuggingFaceDatasetCardResult(
        path=card_path,
        written=True,
        repo_id=manifest.hf_repo_id,
        manifest=manifest,
    )


def push_dataset_to_hub(
    dataset_path: str | Path,
    *,
    repo_id: str | None = None,
    dry_run: bool = False,
    private: bool = False,
    commit_message: str = "Upload SRB dataset artifact",
) -> HuggingFaceUploadPlan:
    """Upload a dataset directory to Hugging Face Hub.

    Dry-run mode returns the resolved plan without importing optional Hub
    dependencies or performing network operations.
    """
    if dry_run:
        plan = replace(
            _plan_huggingface_upload_dry_run(dataset_path, repo_id=repo_id),
            commit_message=commit_message,
        )
        return replace(
            plan, generated_files=generated_huggingface_card_files(plan.files)
        )

    require_huggingface_hub("Dataset")
    plan = replace(
        plan_huggingface_upload(dataset_path, repo_id=repo_id),
        commit_message=commit_message,
    )
    generated = generated_huggingface_card_files(plan.files)
    try:
        if generated:
            card_path = (plan.upload_dir or plan.local_dir) / "README.md"
            card_path.write_bytes(
                build_huggingface_dataset_card(plan.manifest).encode("utf-8")
            )
        plan = replace(plan, generated_files=generated)
        upload_huggingface_folder(
            repo_id=plan.repo_id,
            repo_type=plan.repo_type,
            local_dir=plan.upload_dir or plan.local_dir,
            private=private,
            commit_message=commit_message,
        )
        # The snapshot is released below; return a plan that does not point
        # at its soon-deleted directory or own its handles.
        return replace(
            plan,
            upload_dir=plan.local_dir,
            _snapshot=None,
            _dataset_snapshot=None,
        )
    finally:
        plan.close()


def _thawed_manifest_metadata(manifest: DatasetManifest) -> dict[str, Any]:
    return {key: thaw_json_value(value) for key, value in manifest.metadata.items()}


def _is_publication_control_file(path: str | Path) -> bool:
    name = Path(path).name
    return name in {
        ".srb-directory-publication.json",
    } or name.endswith(
        (".publication.lock", ".publication.pending", ".publication.sha256")
    )


def _dataset_card_front_matter(
    *,
    manifest: DatasetManifest,
    title: str,
) -> list[str]:
    algo = manifest.identity.algo or "null"
    return [
        "---",
        "license: apache-2.0",
        "library_name: space-robotics-bench",
        f"pretty_name: {title}",
        "tags:",
        "- robotics",
        "- robot-learning",
        "- reinforcement-learning",
        "- imitation-learning",
        "- computer-vision",
        "- space-robotics",
        "configs:",
        "- config_name: default",
        "  data_files:",
        f"  - split: {manifest.split.value}",
        f"    path: {_dataset_data_file_pattern(manifest)}",
        "srb:",
        f"  task: {manifest.identity.task}",
        f"  embodiment: {manifest.identity.embodiment}",
        f"  action_mode: {manifest.identity.action_mode}",
        f"  algo: {algo}",
        f"  split: {manifest.split.value}",
        f"  controller: {manifest.controller}",
        f"  storage_format: {manifest.storage_format.value}",
        "---",
    ]


def _dataset_card_action_schema_lines(schema: Any) -> list[str]:
    if not isinstance(schema, list) or not schema:
        return []
    lines = [
        "## Action Schema",
        "",
        "| Name | Path | Shape | Dim | Slice | Dtype |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for component in schema:
        if not isinstance(component, dict):
            continue
        name = str(component.get("name") or "")
        path = component.get("path")
        if isinstance(path, list):
            path_value = ".".join(str(part) for part in path)
        else:
            path_value = ""
        shape_value = format_huggingface_metadata_value(component.get("shape"))
        dim = component.get("dim")
        start = component.get("start")
        end = component.get("end")
        dtype = "" if component.get("dtype") is None else str(component.get("dtype"))
        lines.append(
            f"| `{name}` | `{path_value}` | `{shape_value}` | `{dim}` | "
            f"`{start}:{end}` | `{dtype}` |"
        )
    lines.append("")
    return lines


def _dataset_data_file_pattern(manifest: DatasetManifest) -> str:
    dataset_path = manifest.dataset_path.strip("/") or "data"
    if manifest.storage_format == StorageFormat.SRB_JSONL:
        if dataset_path.endswith(".jsonl"):
            return dataset_path
        return f"{dataset_path}/*.jsonl"
    if manifest.storage_format == StorageFormat.LEROBOT:
        return f"{dataset_path}/**/*.parquet"
    return dataset_path
