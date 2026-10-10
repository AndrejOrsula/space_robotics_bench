"""Route dataset validation and exports by storage format."""

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, cast

from srb.datasets.metadata_validation import validate_metadata_count
from srb.datasets.schema import StorageFormat
from srb.utils.json_types import ImmutableValueMixin
from srb.utils.resource_budget import ResourceLimitError

if TYPE_CHECKING:
    from srb.datasets.export import DatasetExportResult
    from srb.datasets.manifest import DatasetManifest


DatasetFormatValidationFn = Callable[..., None]
DatasetFormatCountValidationFn = Callable[..., None]
DatasetFormatExportFn = Callable[..., "DatasetExportResult"]
DatasetStepRecord = dict[str, Any]
DatasetStepIteratorFn = Callable[..., Iterator[DatasetStepRecord]]
DatasetExportPath = tuple[StorageFormat, StorageFormat]


@dataclass(frozen=True)
class DatasetFormatAdapter(ImmutableValueMixin):
    """Adapter for one physical dataset storage format."""

    storage_format: StorageFormat
    validate_contents_fn: DatasetFormatValidationFn | None = None
    validate_counts_fn: DatasetFormatCountValidationFn | None = None
    step_iterator_fn: DatasetStepIteratorFn | None = None
    export_fns: Mapping[StorageFormat, DatasetFormatExportFn] = field(
        default_factory=dict
    )

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "export_fns",
            MappingProxyType(dict(self.export_fns)),
        )

    def supported_export_formats(self) -> tuple[StorageFormat, ...]:
        """Return output formats this format can export to directly."""

        return tuple(self.export_fns)

    def supports_step_iteration(self) -> bool:
        """Return whether this format can expose canonical replay step rows."""

        return self.step_iterator_fn is not None

    def validate_contents(
        self,
        *,
        manifest: "DatasetManifest",
        dataset_path: Path,
        errors: list[str],
        warnings: list[str],
    ) -> None:
        """Validate dataset contents for this storage format."""

        if self.validate_contents_fn is None:
            warnings.append(f"unknown storage format: {self.storage_format.value}")
            return
        self.validate_contents_fn(
            manifest=manifest,
            dataset_path=dataset_path,
            errors=errors,
            warnings=warnings,
        )

    def validate_counts(
        self,
        *,
        manifest: "DatasetManifest",
        dataset_path: Path,
        errors: list[str],
        warnings: list[str],
    ) -> None:
        """Validate declared counts against physical format lengths."""

        if self.validate_counts_fn is None:
            return
        self.validate_counts_fn(
            manifest=manifest,
            dataset_path=dataset_path,
            errors=errors,
            warnings=warnings,
        )

    def iter_steps(
        self,
        *,
        manifest: "DatasetManifest",
        manifest_path: Path,
        dataset_path: Path,
        max_episodes: int | None = None,
        max_steps: int | None = None,
    ) -> Iterator[DatasetStepRecord]:
        """Yield canonical replay step rows for this storage format."""

        if self.step_iterator_fn is None:
            raise ValueError(
                "dataset replay does not support storage format: "
                f"{self.storage_format.value}"
            )
        yield from self.step_iterator_fn(
            manifest=manifest,
            manifest_path=manifest_path,
            dataset_path=dataset_path,
            max_episodes=max_episodes,
            max_steps=max_steps,
        )

    def export_to(
        self,
        *,
        output_format: StorageFormat | str,
        source_data_path: Path,
        source_manifest: "DatasetManifest",
        source_manifest_path: Path,
        output_dir: Path,
        source_provenance_path: Path | None = None,
    ) -> "DatasetExportResult":
        """Export from this format to another supported format."""

        output_format = StorageFormat(str(output_format))
        export_fn = self.export_fns.get(output_format)
        if export_fn is None:
            raise ValueError(
                "unsupported dataset export path: "
                f"{self.storage_format.value} -> {output_format.value}"
            )
        callback_kwargs: dict[str, Any] = {
            "source_data_path": source_data_path,
            "source_manifest": source_manifest,
            "source_manifest_path": source_manifest_path,
            "output_dir": output_dir,
        }
        if source_provenance_path is None:
            source_provenance_path = source_data_path

        # Legacy callbacks may omit provenance. Inspect the signature so
        # callback errors pass through unchanged.
        try:
            parameters = inspect.signature(export_fn).parameters
        except (TypeError, ValueError):
            return export_fn(**callback_kwargs)

        provenance_parameter = parameters.get("source_provenance_path")
        accepts_provenance = (
            provenance_parameter is not None
            and provenance_parameter.kind
            in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
        ) or any(
            parameter.kind is inspect.Parameter.VAR_KEYWORD
            for parameter in parameters.values()
        )
        if accepts_provenance:
            callback_kwargs["source_provenance_path"] = source_provenance_path
        return export_fn(**callback_kwargs)


def get_dataset_format_adapter(
    storage_format: StorageFormat | str,
) -> DatasetFormatAdapter:
    """Return the Adapter for a dataset storage format."""

    storage_format = StorageFormat(str(storage_format))
    try:
        return _FORMAT_ADAPTERS[storage_format]
    except KeyError:
        raise ValueError(
            f"unsupported dataset storage format: {storage_format.value}"
        ) from None


def list_dataset_format_adapters() -> tuple[DatasetFormatAdapter, ...]:
    """Return all registered dataset format Adapters."""

    return tuple(_FORMAT_ADAPTERS.values())


def supported_dataset_export_paths() -> tuple[DatasetExportPath, ...]:
    """Return direct dataset export paths declared by the format Adapters."""

    return tuple(
        (adapter.storage_format, output_format)
        for adapter in list_dataset_format_adapters()
        for output_format in adapter.supported_export_formats()
    )


def _validate_srb_jsonl_contents(
    *,
    manifest: "DatasetManifest",
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
) -> None:
    from srb.datasets.jsonl import validate_srb_jsonl_contents

    validate_srb_jsonl_contents(
        metadata=manifest.metadata,
        dataset_path=dataset_path,
        errors=errors,
        warnings=warnings,
    )


def _validate_srb_jsonl_counts(
    *,
    manifest: "DatasetManifest",
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
) -> None:
    """Compare canonical JSONL count metadata without row-structure checks."""

    del warnings
    if not any(
        key in manifest.metadata
        for key in ("num_episodes", "total_samples", "action_dim")
    ):
        return

    from srb.datasets.jsonl import (
        _scan_srb_jsonl_counts,
        srb_jsonl_episode_paths,
    )

    episode_paths = srb_jsonl_episode_paths(dataset_path)
    if not episode_paths:
        errors.append(f"canonical dataset has no JSONL episode files: {dataset_path}")
        return

    try:
        num_episodes, total_samples, action_dim, action_dim_error = (
            _scan_srb_jsonl_counts(dataset_path)
        )
    except (OSError, TypeError, ValueError) as exc:
        errors.append(f"could not read canonical JSONL dataset {dataset_path}: {exc}")
        return

    validate_metadata_count(
        metadata=manifest.metadata,
        key="num_episodes",
        actual=num_episodes,
        errors=errors,
    )
    validate_metadata_count(
        metadata=manifest.metadata,
        key="total_samples",
        actual=total_samples,
        errors=errors,
    )
    if action_dim is not None:
        validate_metadata_count(
            metadata=manifest.metadata,
            key="action_dim",
            actual=action_dim,
            errors=errors,
        )
    if action_dim_error is not None:
        errors.append(action_dim_error)


def _iter_srb_jsonl_steps(
    *,
    manifest: "DatasetManifest",
    manifest_path: Path,
    dataset_path: Path,
    max_episodes: int | None = None,
    max_steps: int | None = None,
) -> Iterator[DatasetStepRecord]:
    from srb.datasets.jsonl import iter_srb_jsonl_rows

    del manifest, manifest_path
    for row in iter_srb_jsonl_rows(
        dataset_path,
        max_episodes=max_episodes,
        max_steps=max_steps,
    ):
        yield cast(DatasetStepRecord, row)


def _validate_robomimic_hdf5_contents(
    *,
    manifest: "DatasetManifest",
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
) -> None:
    from srb.datasets.robomimic import validate_robomimic_hdf5_contents

    del warnings
    validate_robomimic_hdf5_contents(
        metadata=manifest.metadata,
        dataset_path=dataset_path,
        errors=errors,
    )


def _validate_robomimic_counts(
    *,
    manifest: "DatasetManifest",
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
) -> None:
    """Compare Robomimic declarations with HDF5 dataset dimensions."""

    del warnings
    if not dataset_path.is_file():
        errors.append(f"robomimic dataset is not an HDF5 file: {dataset_path}")
        return
    from srb.datasets.hdf5_native import inspect_robomimic_hdf5
    from srb.datasets.robomimic import _native_validation_limits

    try:
        result = inspect_robomimic_hdf5(
            dataset_path,
            mode="count",
            limits=_native_validation_limits(),
        )
    except ResourceLimitError as exc:
        errors.append(f"robomimic dataset rejected: {exc}")
        return
    except (OSError, TypeError, ValueError) as exc:
        errors.append(f"could not read robomimic HDF5: {exc}")
        return
    errors.extend(result.errors)
    if not result.complete:
        return
    if result.demo_count is None or result.total_samples is None:
        errors.append("robomimic HDF5 count inspection returned incomplete counts")
        return
    validate_metadata_count(
        metadata=manifest.metadata,
        key="num_episodes",
        actual=result.demo_count,
        errors=errors,
    )
    validate_metadata_count(
        metadata=manifest.metadata,
        key="num_demos",
        actual=result.demo_count,
        errors=errors,
    )
    validate_metadata_count(
        metadata=manifest.metadata,
        key="total_samples",
        actual=result.total_samples,
        errors=errors,
    )
    if result.train_count is not None:
        validate_metadata_count(
            metadata=manifest.metadata,
            key="num_train",
            actual=result.train_count,
            errors=errors,
        )
    if result.valid_count is not None:
        validate_metadata_count(
            metadata=manifest.metadata,
            key="num_valid",
            actual=result.valid_count,
            errors=errors,
        )


def _validate_lerobot_contents(
    *,
    manifest: "DatasetManifest",
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
) -> None:
    from srb.datasets.lerobot import validate_lerobot_contents

    del warnings
    validate_lerobot_contents(
        metadata=manifest.metadata,
        dataset_path=dataset_path,
        errors=errors,
    )


def _validate_lerobot_counts(
    *,
    manifest: "DatasetManifest",
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
) -> None:
    """Compare LeRobot count declarations with available table lengths."""

    del warnings
    from srb.datasets.lerobot import (
        _parquet_reader,
        _read_bounded_parquet_counts,
        _read_json_object,
        _validate_info_count,
        _validate_lerobot_table_count,
    )

    dataset_root = dataset_path.parent if dataset_path.name == "data" else dataset_path
    info_path = dataset_root / "meta" / "info.json"
    if not info_path.is_file():
        errors.append(f"meta/info.json does not exist: {info_path}")
        return
    info = _read_json_object(info_path, "meta/info.json", errors)
    if info is None:
        return

    _validate_info_count(
        info=info,
        info_key="total_episodes",
        metadata=manifest.metadata,
        metadata_key="num_episodes",
        errors=errors,
    )
    _validate_info_count(
        info=info,
        info_key="total_frames",
        metadata=manifest.metadata,
        metadata_key="total_samples",
        errors=errors,
    )

    frame_files = sorted((dataset_root / "data").glob("chunk-*/file-*.parquet"))
    episode_files = sorted(
        (dataset_root / "meta" / "episodes").glob("chunk-*/file-*.parquet")
    )
    if not frame_files:
        errors.append("LeRobot dataset has no frame parquet files under data/chunk-*")
    if not episode_files:
        errors.append(
            "LeRobot dataset has no episode parquet files under meta/episodes/chunk-*"
        )

    parquet = _parquet_reader(errors)
    if parquet is None:
        return

    try:
        total_frame_rows, total_episode_rows, _episode_lengths, _can_sum = (
            _read_bounded_parquet_counts(
                parquet=parquet,
                dataset_root=dataset_root,
                frame_files=frame_files,
                episode_files=episode_files,
                features={},
                errors=errors,
            )
        )
    except (
        AttributeError,
        OSError,
        OverflowError,
        TypeError,
        ValueError,
        ResourceLimitError,
    ) as exc:
        errors.append(f"could not read bounded LeRobot parquet tables: {exc}")
        return

    _validate_lerobot_table_count(
        label="LeRobot frame rows",
        actual=total_frame_rows,
        expected=info.get("total_frames"),
        errors=errors,
    )
    _validate_lerobot_table_count(
        label="LeRobot episode rows",
        actual=total_episode_rows,
        expected=info.get("total_episodes"),
        errors=errors,
    )
    validate_metadata_count(
        metadata=manifest.metadata,
        key="total_samples",
        actual=total_frame_rows,
        errors=errors,
    )
    validate_metadata_count(
        metadata=manifest.metadata,
        key="num_episodes",
        actual=total_episode_rows,
        errors=errors,
    )


def _export_robomimic_to_srb_jsonl(**kwargs: Any) -> "DatasetExportResult":
    from srb.datasets.export import _export_robomimic_to_srb_jsonl

    return _export_robomimic_to_srb_jsonl(**kwargs)


def _export_robomimic_to_lerobot_v3(**kwargs: Any) -> "DatasetExportResult":
    from srb.datasets.export import _export_robomimic_to_lerobot_v3

    return _export_robomimic_to_lerobot_v3(**kwargs)


def _export_srb_jsonl_to_robomimic_hdf5(**kwargs: Any) -> "DatasetExportResult":
    from srb.datasets.export import _export_srb_jsonl_to_robomimic_hdf5

    return _export_srb_jsonl_to_robomimic_hdf5(**kwargs)


def _export_srb_jsonl_to_srb_jsonl(**kwargs: Any) -> "DatasetExportResult":
    from srb.datasets.export import _export_srb_jsonl_to_srb_jsonl

    return _export_srb_jsonl_to_srb_jsonl(**kwargs)


def _export_srb_jsonl_to_lerobot_v3(**kwargs: Any) -> "DatasetExportResult":
    from srb.datasets.export import _export_srb_jsonl_to_lerobot_v3

    return _export_srb_jsonl_to_lerobot_v3(**kwargs)


def _iter_lerobot_steps(
    *,
    manifest: "DatasetManifest",
    manifest_path: Path,
    dataset_path: Path,
    max_episodes: int | None = None,
    max_steps: int | None = None,
) -> Iterator[DatasetStepRecord]:
    from srb.datasets.lerobot import iter_lerobot_steps

    del manifest_path
    yield from iter_lerobot_steps(
        metadata=manifest.metadata,
        dataset_path=dataset_path,
        max_episodes=max_episodes,
        max_steps=max_steps,
    )


_FORMAT_ADAPTERS: dict[StorageFormat, DatasetFormatAdapter] = {
    StorageFormat.SRB_JSONL: DatasetFormatAdapter(
        storage_format=StorageFormat.SRB_JSONL,
        validate_contents_fn=_validate_srb_jsonl_contents,
        validate_counts_fn=_validate_srb_jsonl_counts,
        step_iterator_fn=_iter_srb_jsonl_steps,
        export_fns={
            StorageFormat.SRB_JSONL: _export_srb_jsonl_to_srb_jsonl,
            StorageFormat.ROBOMIMIC_HDF5: _export_srb_jsonl_to_robomimic_hdf5,
            StorageFormat.LEROBOT: _export_srb_jsonl_to_lerobot_v3,
        },
    ),
    StorageFormat.ROBOMIMIC_HDF5: DatasetFormatAdapter(
        storage_format=StorageFormat.ROBOMIMIC_HDF5,
        validate_contents_fn=_validate_robomimic_hdf5_contents,
        validate_counts_fn=_validate_robomimic_counts,
        export_fns={
            StorageFormat.SRB_JSONL: _export_robomimic_to_srb_jsonl,
            StorageFormat.LEROBOT: _export_robomimic_to_lerobot_v3,
        },
    ),
    StorageFormat.LEROBOT: DatasetFormatAdapter(
        storage_format=StorageFormat.LEROBOT,
        validate_contents_fn=_validate_lerobot_contents,
        validate_counts_fn=_validate_lerobot_counts,
        step_iterator_fn=_iter_lerobot_steps,
    ),
}
