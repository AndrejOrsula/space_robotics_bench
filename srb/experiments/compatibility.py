"""Algorithm and dataset compatibility facts for SRB experiment artifacts."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

from srb.core.algorithms import (
    SupportedILAlgo,
    SupportedRLAlgo,
    canonical_algorithm_label,
)
from srb.integrations.algorithms import resolve_algorithm_adapter
from srb.utils.dataset_manifest import (
    DatasetManifest,
    find_dataset_manifest,
    read_dataset_manifest,
)
from srb.utils.dataset_schema import (
    DatasetSplit,
    StorageFormat,
    validate_demo_io_contract,
)

CompatibilityPurpose = Literal["train", "collect_eval", "publish"]

_ALL_SPLITS = (
    DatasetSplit.HUMAN,
    DatasetSplit.POLICY,
    DatasetSplit.RANDOM,
    DatasetSplit.ZERO,
    DatasetSplit.REAL,
)
_CANONICAL_PUBLICATION_FORMATS = (StorageFormat.SRB_JSONL, StorageFormat.LEROBOT)
_COLLECTION_FORMATS = (StorageFormat.SRB_JSONL, StorageFormat.ROBOMIMIC_HDF5)
_KNOWN_PURPOSES: tuple[CompatibilityPurpose, ...] = (
    "train",
    "collect_eval",
    "publish",
)


@dataclass(frozen=True)
class AlgorithmCompatibility:
    """Support facts for one algorithm entrypoint."""

    algo: str
    framework: str
    workflow: Literal["online_rl", "offline_il_rl"]
    direct_train_storage_formats: tuple[StorageFormat, ...]
    convertible_train_storage_formats: tuple[StorageFormat, ...] = ()
    preferred_train_export_format: StorageFormat | None = None
    collection_storage_formats: tuple[StorageFormat, ...] = _COLLECTION_FORMATS
    publication_storage_formats: tuple[StorageFormat, ...] = (
        _CANONICAL_PUBLICATION_FORMATS
    )
    supported_dataset_splits: tuple[DatasetSplit, ...] = _ALL_SPLITS
    notes: tuple[str, ...] = ()
    training_dataset_mode: Literal["unsupported", "offline", "replay_prefill"] = (
        "unsupported"
    )

    def to_dict(self) -> dict[str, Any]:
        return {
            "algo": self.algo,
            "framework": self.framework,
            "workflow": self.workflow,
            "training_dataset_mode": self.training_dataset_mode,
            "direct_train_storage_formats": [
                storage_format.value
                for storage_format in self.direct_train_storage_formats
            ],
            "convertible_train_storage_formats": [
                storage_format.value
                for storage_format in self.convertible_train_storage_formats
            ],
            "preferred_train_export_format": (
                self.preferred_train_export_format.value
                if self.preferred_train_export_format is not None
                else None
            ),
            "collection_storage_formats": [
                storage_format.value
                for storage_format in self.collection_storage_formats
            ],
            "publication_storage_formats": [
                storage_format.value
                for storage_format in self.publication_storage_formats
            ],
            "supported_dataset_splits": [
                split.value for split in self.supported_dataset_splits
            ],
            "notes": list(self.notes),
        }


@dataclass(frozen=True)
class AlgorithmDatasetCompatibilityReport:
    """Dataset compatibility for one algorithm and purpose."""

    compatible: bool
    purpose: CompatibilityPurpose
    compatibility: AlgorithmCompatibility
    dataset: DatasetManifest
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    conversion_required: bool = False
    recommended_export_format: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "compatible": self.compatible,
            "purpose": self.purpose,
            "algorithm": self.compatibility.to_dict(),
            "dataset": _dataset_to_dict(self.dataset),
            "errors": list(self.errors),
            "warnings": list(self.warnings),
            "conversion_required": self.conversion_required,
            "recommended_export_format": self.recommended_export_format,
        }


@dataclass(frozen=True)
class DatasetCompatibilityMatrixReport:
    """Dataset compatibility across registered algorithms."""

    purpose: CompatibilityPurpose
    dataset: DatasetManifest
    algorithm_reports: tuple[AlgorithmDatasetCompatibilityReport, ...]

    @property
    def compatible(self) -> bool:
        return all(report.compatible for report in self.algorithm_reports)

    def to_dict(self) -> dict[str, Any]:
        compatible_count = sum(
            1 for report in self.algorithm_reports if report.compatible
        )
        return {
            "compatible": self.compatible,
            "purpose": self.purpose,
            "dataset": _dataset_to_dict(self.dataset),
            "summary": {
                "total_algorithms": len(self.algorithm_reports),
                "compatible_algorithms": compatible_count,
                "incompatible_algorithms": len(self.algorithm_reports)
                - compatible_count,
            },
            "algorithms": [
                _algorithm_report_to_matrix_entry(report)
                for report in self.algorithm_reports
            ],
        }


def iter_algorithm_compatibilities() -> tuple[AlgorithmCompatibility, ...]:
    """Return support facts for every CLI algorithm."""
    algos = [str(algo) for algo in SupportedRLAlgo]
    algos.extend(str(algo) for algo in SupportedILAlgo)
    return tuple(compatibility_for_algorithm(algo) for algo in sorted(algos))


def compatibility_for_algorithm(algo: str) -> AlgorithmCompatibility:
    """Return compatibility facts for an SRB algorithm."""
    normalized_algo = canonical_algorithm_label(algo)
    adapter = resolve_algorithm_adapter(normalized_algo)
    if adapter.offline_learning:
        return AlgorithmCompatibility(
            algo=normalized_algo,
            framework=adapter.framework,
            workflow="offline_il_rl",
            direct_train_storage_formats=(StorageFormat.ROBOMIMIC_HDF5,),
            training_dataset_mode="offline",
            convertible_train_storage_formats=(StorageFormat.SRB_JSONL,),
            preferred_train_export_format=StorageFormat.ROBOMIMIC_HDF5,
            notes=(
                "robomimic trains directly on robomimic_hdf5; canonical "
                "srb_jsonl datasets can be exported to robomimic_hdf5 for "
                "training and to srb_jsonl or lerobot for publication.",
            ),
        )
    if adapter.training_dataset_mode == "replay_prefill":
        return AlgorithmCompatibility(
            algo=normalized_algo,
            framework=adapter.framework,
            workflow="online_rl",
            direct_train_storage_formats=(StorageFormat.SRB_JSONL,),
            convertible_train_storage_formats=(StorageFormat.ROBOMIMIC_HDF5,),
            preferred_train_export_format=StorageFormat.SRB_JSONL,
            training_dataset_mode="replay_prefill",
            notes=(
                f"{normalized_algo} prefills replay from certified canonical srb_jsonl "
                "through paired --demo-dataset and --demo-max-transitions inputs. "
                "The trainer checks the live I/O layout, replay capacity and "
                "complete native environment batches before insertion.",
            ),
        )
    return AlgorithmCompatibility(
        algo=normalized_algo,
        framework=adapter.framework,
        workflow="online_rl",
        direct_train_storage_formats=(),
        notes=(
            "This algorithm trains from live environments. SRB has no "
            "implemented demonstration-training or replay-prefill input for "
            "this label; collection, evaluation and publication remain separate.",
        ),
    )


def assess_algorithm_dataset_compatibility(
    algo: str,
    dataset: DatasetManifest | str | Path,
    *,
    purpose: CompatibilityPurpose = "train",
) -> AlgorithmDatasetCompatibilityReport:
    """Assess a dataset against one algorithm and purpose."""
    if purpose not in _KNOWN_PURPOSES:
        raise ValueError(
            f"unknown compatibility purpose {purpose!r}; expected one of "
            f"{', '.join(_KNOWN_PURPOSES)}"
        )

    manifest = (
        dataset
        if isinstance(dataset, DatasetManifest)
        else read_dataset_manifest(find_dataset_manifest(dataset))
    )
    compatibility = compatibility_for_algorithm(algo)
    errors: list[str] = []
    warnings: list[str] = []
    conversion_required = False
    recommended_export_format: str | None = None

    if manifest.split not in compatibility.supported_dataset_splits:
        errors.append(
            f"{compatibility.algo} does not support dataset split {manifest.split.value}"
        )

    if purpose == "train":
        if compatibility.training_dataset_mode == "replay_prefill":
            try:
                validate_demo_io_contract(
                    manifest.metadata.get("demo_io_contract"),
                    expected_task=manifest.identity.task,
                    required=True,
                )
            except (TypeError, ValueError) as exc:
                errors.append(f"{compatibility.algo}: {exc}")
        if compatibility.direct_train_storage_formats:
            if (
                manifest.storage_format
                not in compatibility.direct_train_storage_formats
            ):
                if (
                    manifest.storage_format
                    in compatibility.convertible_train_storage_formats
                ):
                    conversion_required = True
                    recommended_export_format = _preferred_train_export_format(
                        compatibility
                    )
                    warnings.append(
                        f"{compatibility.algo} requires export to "
                        f"{recommended_export_format} before direct training."
                    )
                else:
                    expected = ", ".join(
                        storage_format.value
                        for storage_format in compatibility.direct_train_storage_formats
                    )
                    errors.append(
                        f"{compatibility.algo} trains directly from {expected}, "
                        f"not {manifest.storage_format.value}"
                    )
        else:
            errors.append(
                f"{compatibility.algo} has no implemented demonstration-training "
                "or replay-prefill input. Dataset collection, evaluation and "
                "publication support does not make it a training input."
            )
    elif purpose == "collect_eval":
        if manifest.storage_format not in compatibility.collection_storage_formats:
            expected = ", ".join(
                storage_format.value
                for storage_format in compatibility.collection_storage_formats
            )
            errors.append(
                f"{compatibility.algo} collect/eval datasets use {expected}, "
                f"not {manifest.storage_format.value}"
            )
    elif purpose == "publish":
        if manifest.storage_format not in compatibility.publication_storage_formats:
            expected = ", ".join(
                storage_format.value
                for storage_format in compatibility.publication_storage_formats
            )
            errors.append(
                f"{compatibility.algo} publication datasets should use {expected}, "
                f"not {manifest.storage_format.value}"
            )

    return AlgorithmDatasetCompatibilityReport(
        compatible=not errors,
        purpose=purpose,
        compatibility=compatibility,
        dataset=manifest,
        errors=errors,
        warnings=warnings,
        conversion_required=conversion_required,
        recommended_export_format=recommended_export_format,
    )


def assess_dataset_compatibility_matrix(
    dataset: DatasetManifest | str | Path,
    *,
    purpose: CompatibilityPurpose = "train",
) -> DatasetCompatibilityMatrixReport:
    """Assess a dataset against every CLI algorithm."""
    if purpose not in _KNOWN_PURPOSES:
        raise ValueError(
            f"unknown compatibility purpose {purpose!r}; expected one of "
            f"{', '.join(_KNOWN_PURPOSES)}"
        )

    manifest = (
        dataset
        if isinstance(dataset, DatasetManifest)
        else read_dataset_manifest(find_dataset_manifest(dataset))
    )
    reports = tuple(
        assess_algorithm_dataset_compatibility(
            compatibility.algo,
            manifest,
            purpose=purpose,
        )
        for compatibility in iter_algorithm_compatibilities()
    )
    return DatasetCompatibilityMatrixReport(
        purpose=purpose,
        dataset=manifest,
        algorithm_reports=reports,
    )


def _dataset_to_dict(dataset: DatasetManifest) -> dict[str, Any]:
    return {
        "task": dataset.identity.task,
        "embodiment": dataset.identity.embodiment,
        "action_mode": dataset.identity.action_mode,
        "algo": dataset.identity.algo,
        "split": dataset.split.value,
        "storage_format": dataset.storage_format.value,
        "controller": dataset.controller,
        "hf_repo_id": dataset.hf_repo_id,
    }


def _algorithm_report_to_matrix_entry(
    report: AlgorithmDatasetCompatibilityReport,
) -> dict[str, Any]:
    return {
        "compatible": report.compatible,
        "algorithm": report.compatibility.to_dict(),
        "errors": list(report.errors),
        "warnings": list(report.warnings),
        "conversion_required": report.conversion_required,
        "recommended_export_format": report.recommended_export_format,
    }


def _preferred_train_export_format(compatibility: AlgorithmCompatibility) -> str:
    if compatibility.preferred_train_export_format is not None:
        return compatibility.preferred_train_export_format.value
    if compatibility.direct_train_storage_formats:
        return compatibility.direct_train_storage_formats[0].value
    return ""
