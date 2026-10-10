"""Hugging Face Hub helpers for SRB model artifacts."""

from __future__ import annotations

import hashlib
from dataclasses import replace
from pathlib import Path

from srb.experiments.models import (
    ModelArtifactManifest,
    find_model_artifact_manifest,
    model_artifact_root_from_manifest_path,
    validate_model_artifact_layout,
)
from srb.utils.artifact_cards import (
    build_model_card as build_huggingface_model_card,
)
from srb.utils.huggingface import (
    HuggingFaceHubCardResult,
    HuggingFaceHubUploadPlan,
    generated_huggingface_card_files,
    raise_for_blocking_huggingface_publication_warnings,
    require_huggingface_hub,
    upload_huggingface_folder,
)
from srb.utils.publication import ArtifactPublication, PublishedDirectory

_BLOCKING_PUBLICATION_WARNING_PREFIXES = (
    "model artifact does not identify the training dataset repository",
)


class HuggingFaceModelUploadPlan(HuggingFaceHubUploadPlan[ModelArtifactManifest]):
    """Resolved files and repository target for a model upload."""


class HuggingFaceModelCardResult(HuggingFaceHubCardResult[ModelArtifactManifest]):
    """Result of writing a Hugging Face model card."""


def plan_huggingface_model_upload(
    artifact_path: str | Path,
    *,
    repo_id: str | None = None,
) -> HuggingFaceModelUploadPlan:
    """Resolve the model upload target and local files without network access."""
    manifest_path = find_model_artifact_manifest(artifact_path)
    report = validate_model_artifact_layout(manifest_path)
    if not report.valid or report.manifest is None:
        raise ValueError(
            "model artifact is not publishable: " + "; ".join(report.errors)
        )
    raise_for_blocking_huggingface_publication_warnings(
        report.warnings,
        blocking_prefixes=_BLOCKING_PUBLICATION_WARNING_PREFIXES,
        artifact_label="model artifact",
    )

    local_dir = model_artifact_root_from_manifest_path(manifest_path)
    # Use the validated member snapshot, excluding transport metadata.
    files = list(report.member_paths)
    snapshot = PublishedDirectory(
        path=local_dir,
        member_hashes=report.member_hashes,
    ).materialize_snapshot(members=files)
    return HuggingFaceModelUploadPlan(
        repo_id=repo_id or str(report.manifest.hf_repo_id),
        repo_type="model",
        local_dir=local_dir,
        files=files,
        manifest=report.manifest,
        upload_dir=snapshot.path,
        _snapshot=snapshot,
    )


def write_huggingface_model_card(
    artifact_path: str | Path,
    *,
    output_path: str | Path | None = None,
    overwrite: bool = False,
) -> HuggingFaceModelCardResult:
    """Write a model card without changing artifact identity."""
    plan = plan_huggingface_model_upload(artifact_path)
    card_path = (
        Path(output_path) if output_path is not None else plan.local_dir / "README.md"
    )

    canonical_path = plan.local_dir / "README.md"
    canonical = card_path.resolve(strict=False) == canonical_path.resolve(strict=False)
    card_payload = build_huggingface_model_card(plan.manifest).encode("utf-8")
    expected_sha256 = hashlib.sha256(card_payload).hexdigest()

    if canonical and plan.manifest.schema_version >= 3:
        # V3 README.md is part of the validated tree and cannot be rewritten.
        return HuggingFaceModelCardResult(
            path=card_path,
            written=False,
            repo_id=plan.repo_id,
            manifest=plan.manifest,
        )

    if not canonical:
        try:
            if card_path.resolve(strict=False).is_relative_to(plan.local_dir.resolve()):
                raise ValueError(
                    "custom model card output inside an artifact would change "
                    "its exact-tree identity"
                )
        except RuntimeError as exc:
            raise ValueError("invalid custom model card output path") from exc

        existed = card_path.exists() or card_path.is_symlink()
        if existed and not overwrite:
            return HuggingFaceModelCardResult(
                path=card_path,
                written=False,
                repo_id=plan.repo_id,
                manifest=plan.manifest,
            )

        ArtifactPublication.file(card_path).publish(
            lambda staging: staging.write_bytes(card_payload),
            expected_sha256=expected_sha256,
        )
        return HuggingFaceModelCardResult(
            path=card_path,
            written=True,
            repo_id=plan.repo_id,
            manifest=plan.manifest,
        )

    existed = card_path.exists() or card_path.is_symlink()
    if existed and not overwrite:
        return HuggingFaceModelCardResult(
            path=card_path,
            written=False,
            repo_id=plan.repo_id,
            manifest=plan.manifest,
        )

    # Legacy cards lack a tree descriptor; retain overwrite behavior and verify
    # publication through the shared file seam.
    legacy_lock_path = plan.local_dir.parent / (
        f".{plan.local_dir.name}.readme.publication.lock"
    )
    ArtifactPublication.file(card_path, lock_path=legacy_lock_path).publish(
        lambda staging: staging.write_bytes(card_payload),
        expected_sha256=expected_sha256,
    )
    return HuggingFaceModelCardResult(
        path=card_path,
        written=True,
        repo_id=plan.repo_id,
        manifest=plan.manifest,
    )


def push_model_to_hub(
    artifact_path: str | Path,
    *,
    repo_id: str | None = None,
    dry_run: bool = False,
    private: bool = False,
    commit_message: str = "Upload SRB model artifact",
    mirror: bool = False,
) -> HuggingFaceModelUploadPlan:
    """Upload a self-contained model artifact to Hugging Face Hub.

    ``mirror=True`` deletes remote files absent from the artifact in the same
    commit; the default only adds and overwrites.
    """
    plan = replace(
        plan_huggingface_model_upload(artifact_path, repo_id=repo_id),
        commit_message=commit_message,
    )
    if dry_run:
        return replace(
            plan,
            generated_files=generated_huggingface_card_files(plan.files),
        )

    require_huggingface_hub("Model")
    write_huggingface_model_card(artifact_path, overwrite=False)
    plan = replace(
        plan_huggingface_model_upload(artifact_path, repo_id=repo_id),
        commit_message=commit_message,
    )

    upload_huggingface_folder(
        repo_id=plan.repo_id,
        repo_type=plan.repo_type,
        local_dir=plan.upload_dir or plan.local_dir,
        private=private,
        commit_message=commit_message,
        mirror=mirror,
    )
    return plan
