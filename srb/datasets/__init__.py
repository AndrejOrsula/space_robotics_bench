"""Framework-neutral SRB dataset metadata and validation."""

from importlib import import_module
from typing import TYPE_CHECKING, Any

from .collection import (
    BuiltinActionSource,
    CollectionRunResult,
    PolicyActionSource,
    TeleopActionSource,
    TeleopCollectionControls,
    run_robomimic_collection,
    run_robomimic_teleop_collection,
)
from .export import DatasetExportResult, export_dataset_with_result
from .formats import (
    DatasetFormatAdapter,
    DatasetStepIteratorFn,
    DatasetStepRecord,
    get_dataset_format_adapter,
    list_dataset_format_adapters,
    supported_dataset_export_paths,
)
from .huggingface import (
    HuggingFaceDatasetCardResult,
    HuggingFaceUploadPlan,
    build_huggingface_dataset_card,
    plan_huggingface_upload,
    write_huggingface_dataset_card,
)
from .jsonl import (
    REQUIRED_SRB_JSONL_KEYS,
    SrbJsonlEpisodes,
    SrbJsonlRow,
    build_srb_jsonl_row,
    iter_srb_jsonl_rows,
    read_srb_jsonl_episodes,
    write_srb_jsonl_row,
)
from .manifest import (
    DatasetManifest,
    DatasetManifestDict,
    DatasetValidationReport,
    build_dataset_manifest,
    dataset_root_from_manifest_path,
    find_dataset_manifest,
    read_dataset_manifest,
    resolve_dataset_path,
    validate_dataset_layout,
    write_dataset_manifest,
)
from .provenance import (
    DatasetProvenance,
    coerce_dataset_provenance,
    dataset_provenance_from_env,
    dataset_provenance_from_env_kwargs,
)
from .real import export_real_validation_session
from .replay import (
    DatasetReplaySummary,
    iter_dataset_steps,
    replay_dataset,
    summarize_dataset_replay,
)
from .schema import DatasetSplit, StorageFormat
from .transitions import (
    DemoTransition,
    ValidatedDemoSource,
    open_demo_transition_source,
)

if TYPE_CHECKING:
    from .event_camera_run import (
        EVENTS_MANIFEST_FILENAME,
        EventCameraRunArtifact,
        EventCameraRunSelection,
        EventCameraRunSummary,
        EventCameraRunValidationIssue,
        EventCameraRunValidationReport,
        EventManifestRow,
        append_event_manifest_row,
        create_event_manifest,
        parse_event_reward_summary,
    )

_EVENT_CAMERA_EXPORTS = frozenset(
    {
        "EVENTS_MANIFEST_FILENAME",
        "EventCameraRunArtifact",
        "EventCameraRunSelection",
        "EventCameraRunSummary",
        "EventCameraRunValidationIssue",
        "EventCameraRunValidationReport",
        "EventManifestRow",
        "append_event_manifest_row",
        "create_event_manifest",
        "parse_event_reward_summary",
    }
)


def __getattr__(name: str) -> Any:
    if name not in _EVENT_CAMERA_EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(".event_camera_run", __name__), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))


__all__ = [
    "BuiltinActionSource",
    "CollectionRunResult",
    "DatasetExportResult",
    "DatasetFormatAdapter",
    "DatasetManifest",
    "DatasetManifestDict",
    "DatasetProvenance",
    "DatasetReplaySummary",
    "DatasetSplit",
    "DatasetStepIteratorFn",
    "DatasetStepRecord",
    "DatasetValidationReport",
    "DemoTransition",
    "EVENTS_MANIFEST_FILENAME",
    "EventCameraRunArtifact",
    "EventCameraRunSelection",
    "EventCameraRunSummary",
    "EventCameraRunValidationIssue",
    "EventCameraRunValidationReport",
    "EventManifestRow",
    "HuggingFaceDatasetCardResult",
    "HuggingFaceUploadPlan",
    "PolicyActionSource",
    "REQUIRED_SRB_JSONL_KEYS",
    "SrbJsonlEpisodes",
    "SrbJsonlRow",
    "StorageFormat",
    "TeleopActionSource",
    "TeleopCollectionControls",
    "ValidatedDemoSource",
    "append_event_manifest_row",
    "build_huggingface_dataset_card",
    "build_dataset_manifest",
    "build_srb_jsonl_row",
    "coerce_dataset_provenance",
    "create_event_manifest",
    "dataset_provenance_from_env",
    "dataset_provenance_from_env_kwargs",
    "dataset_root_from_manifest_path",
    "export_dataset_with_result",
    "export_real_validation_session",
    "find_dataset_manifest",
    "get_dataset_format_adapter",
    "iter_dataset_steps",
    "iter_srb_jsonl_rows",
    "list_dataset_format_adapters",
    "open_demo_transition_source",
    "parse_event_reward_summary",
    "plan_huggingface_upload",
    "read_srb_jsonl_episodes",
    "read_dataset_manifest",
    "resolve_dataset_path",
    "replay_dataset",
    "run_robomimic_collection",
    "run_robomimic_teleop_collection",
    "summarize_dataset_replay",
    "supported_dataset_export_paths",
    "validate_dataset_layout",
    "write_dataset_manifest",
    "write_huggingface_dataset_card",
    "write_srb_jsonl_row",
]
