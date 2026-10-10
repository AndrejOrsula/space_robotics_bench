"""Experiment and artifact identity helpers."""

from srb.utils.artifact_identity import (
    ArtifactIdentity,
    slugify_underscore,
    stable_hash,
)

from .models import (
    ModelArtifactManifest,
    ModelArtifactPreparationResult,
    ModelArtifactReference,
    ModelArtifactValidationReport,
    build_model_artifact_manifest,
    discover_model_checkpoint,
    model_artifact_root_from_manifest_path,
    prepare_model_artifact,
    prepare_model_artifact_from_run,
    read_model_artifact_manifest,
    try_model_artifact_reference,
    validate_model_artifact_layout,
    write_model_artifact_manifest,
)
from .policies import (
    CheckpointInference,
    infer_algo_from_model,
    infer_model_algorithm,
    load_policy,
    load_policy_from_checkpoint,
    resolve_agent_workflow_algo,
    resolve_model_artifact_policy_inputs,
    resolve_model_artifact_reference,
    resolve_model_path,
    supports_policy_collection,
)
from .run_layouts import (
    FrameworkConfigArtifact,
    discover_framework_config_artifact,
    discover_run_checkpoint,
)
from .workflows import (
    run_offline_learning,
    run_online_training,
    run_policy_evaluation,
)

__all__ = [
    "ArtifactIdentity",
    "FrameworkConfigArtifact",
    "ModelArtifactManifest",
    "ModelArtifactPreparationResult",
    "ModelArtifactReference",
    "ModelArtifactValidationReport",
    "build_model_artifact_manifest",
    "CheckpointInference",
    "discover_model_checkpoint",
    "discover_framework_config_artifact",
    "discover_run_checkpoint",
    "infer_algo_from_model",
    "infer_model_algorithm",
    "load_policy",
    "load_policy_from_checkpoint",
    "model_artifact_root_from_manifest_path",
    "prepare_model_artifact",
    "prepare_model_artifact_from_run",
    "read_model_artifact_manifest",
    "resolve_agent_workflow_algo",
    "resolve_model_artifact_policy_inputs",
    "resolve_model_artifact_reference",
    "resolve_model_path",
    "run_offline_learning",
    "run_online_training",
    "run_policy_evaluation",
    "slugify_underscore",
    "stable_hash",
    "supports_policy_collection",
    "try_model_artifact_reference",
    "validate_model_artifact_layout",
    "write_model_artifact_manifest",
]
