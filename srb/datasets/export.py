"""Dataset exporters for SRB canonical and interoperability layouts."""

from __future__ import annotations

import importlib
import importlib.util
import json
import math
import shutil
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from contextlib import closing
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    import h5py

from srb.datasets.export_index import JsonlExportIndex, open_jsonl_export_index
from srb.datasets.formats import get_dataset_format_adapter
from srb.datasets.jsonl import (
    SrbJsonlRow,
    build_srb_jsonl_row,
    to_jsonable,
    write_srb_jsonl_row,
)
from srb.datasets.lerobot_transitions import (
    build_transition_extension,
    encode_observation_path,
)
from srb.datasets.manifest import (
    DEFAULT_MANIFEST_RELPATH,
    DatasetManifest,
    _validate_dataset_snapshot,
    _write_staged_dataset_manifest,
    build_dataset_manifest,
    open_dataset_snapshot,
    validate_dataset_layout,
)
from srb.datasets.metadata_validation import validate_demo_io_contract
from srb.datasets.robomimic import (
    _encode_episode_recording_reference,
    _encode_hdf5_demo_io_contract,
    _iter_hdf5_observation_datasets,
    _preflight_hdf5_metadata_storage,
    _preflight_hdf5_storage,
    _read_hdf5_demo_io_contract_attr,
    _read_hdf5_episode_recording_reference,
    _read_hdf5_primitive_attribute,
    _require_hdf5_demo_io_contract_datasets,
    _robomimic_train_count,
    _validate_hdf5_observation_key,
    _validate_robomimic_demos,
    robomimic_env_args,
)
from srb.datasets.schema import StorageFormat
from srb.datasets.visual_contract import (
    VisualImageView,
    cast_declared,
    certified_image_views,
    declared_dtype_for,
    declared_observation_dtypes,
    declared_observation_shapes,
    describe_image_views,
    encode_png,
    has_legacy_batch_axis,
    view_pixels,
)
from srb.utils.dataset_manifest import _open_dataset_publication_for_path
from srb.utils.json_types import thaw_json_value
from srb.utils.publication import ArtifactPublication, PublicationError
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_DIRECTORY_ENTRIES,
    MAX_METADATA_AGGREGATE_BYTES,
    ByteBudget,
    ResourceLimitError,
    checked_shape_nbytes,
    require_dataset_file_size,
)

_CERTIFIED_SCHEMA_MISMATCH_SAMPLE_LIMIT = 8
_CERTIFIED_SCHEMA_ENTRY_OVERHEAD = 256
_CERTIFIED_SCHEMA_SAMPLE_OVERHEAD = 128
_CERTIFIED_SCHEMA_DIAGNOSTIC_PATH_LIMIT = 256
_CERTIFIED_SCHEMA_TEMP_PATH_OVERHEAD = 64
_CERTIFIED_SCHEMA_TEMP_COMPONENT_OVERHEAD = 32


class DatasetExportDestinationError(PublicationError):
    """An export destination cannot be adopted without unsafe final mutation."""

    def __init__(self, output_dir: Path, *, reason: str) -> None:
        self.output_dir = Path(output_dir)
        self.reason = reason
        super().__init__(
            f"dataset export destination is unavailable at {self.output_dir}: "
            f"{reason}. Choose a new, absent output directory outside any "
            "published dataset; SRB will not mutate existing publication state "
            "or adopt unmanaged output"
        )


@dataclass(frozen=True)
class DatasetExportResult:
    """Summary of one dataset export."""

    manifest_path: Path
    num_episodes: int
    total_samples: int
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "manifest_path": str(self.manifest_path),
            "num_episodes": self.num_episodes,
            "total_samples": self.total_samples,
        }
        if self.metadata:
            payload["metadata"] = dict(self.metadata)
        return payload


def export_dataset_with_result(
    *,
    source_path: str | Path,
    output_dir: str | Path,
    output_format: StorageFormat | str,
) -> DatasetExportResult:
    """Export a dataset and return a structured summary."""
    output_dir = Path(output_dir)
    output_format = StorageFormat(str(output_format))

    # Keep the validated source snapshot open through export.
    with open_dataset_snapshot(source_path) as source_snapshot:
        source_report = _validate_dataset_snapshot(
            source_snapshot,
            deep=True,
            strict=False,
        )
        if not source_report.valid:
            raise ValueError(
                "source dataset is not exportable: " + "; ".join(source_report.errors)
            )
        source_manifest = source_snapshot.manifest
        source_manifest_path = source_snapshot.source_manifest_path
        if source_manifest_path is None:
            raise ValueError("dataset export requires a source manifest path")
        source_provenance_path = source_snapshot.source_dataset_path
        if source_provenance_path is None:
            raise ValueError("dataset export requires a source data path")
        source_data_path = source_snapshot.dataset_path
        adapter = get_dataset_format_adapter(source_manifest.storage_format)
        _preflight_export_target_fidelity(
            source_manifest=source_manifest,
            output_format=output_format,
        )

        def build_stage(staging: Path) -> DatasetExportResult:
            return adapter.export_to(
                output_format=output_format,
                source_data_path=source_data_path,
                source_manifest=source_manifest,
                source_manifest_path=source_manifest_path,
                source_provenance_path=source_provenance_path,
                output_dir=staging,
            )

        return _publish_dataset_export(output_dir, build_stage)


def _publish_dataset_export(
    output_dir: Path,
    builder: Callable[[Path], DatasetExportResult],
) -> DatasetExportResult:
    """Build and validate one dataset tree before an atomic directory switch."""

    publication = ArtifactPublication.directory(output_dir)
    _prepare_dataset_export_publication(output_dir, publication=publication)
    built_result: DatasetExportResult | None = None
    manifest_relpath: Path | None = None

    def build(staging: Path) -> None:
        nonlocal built_result, manifest_relpath
        built_result = builder(staging)
        try:
            manifest_relpath = built_result.manifest_path.relative_to(staging)
        except ValueError as exc:
            raise ValueError(
                "dataset export manifest must be inside its private stage"
            ) from exc
        _write_export_dataset_card(built_result.manifest_path)
        report = validate_dataset_layout(built_result.manifest_path, deep=True)
        if not report.valid:
            raise ValueError(
                "exported dataset is not valid: " + "; ".join(report.errors)
            )

    publication.publish(
        build,
        required_members=(DEFAULT_MANIFEST_RELPATH.as_posix(),),
    )
    if built_result is None or manifest_relpath is None:
        raise RuntimeError("dataset export completed without a result")
    return DatasetExportResult(
        manifest_path=output_dir / manifest_relpath,
        num_episodes=built_result.num_episodes,
        total_samples=built_result.total_samples,
        metadata=built_result.metadata,
    )


def _prepare_dataset_export_publication(
    output_dir: Path,
    *,
    publication: ArtifactPublication,
) -> None:
    """Reject unmanaged collisions before an exporter can create a stage."""

    current = _open_dataset_publication_for_path(output_dir)
    if current is not None:
        if current.path != publication.path:
            raise DatasetExportDestinationError(
                output_dir,
                reason=(
                    "the requested output is inside immutable published dataset "
                    f"{current.path}"
                ),
            )
        return

    mirror = publication.path.with_name(f".{publication.path.name}.publication.json")
    if mirror.exists() or mirror.is_symlink():
        # Let publication report an unreadable mirror as partial state.
        publication.open()

    if output_dir.exists() or output_dir.is_symlink():
        raise DatasetExportDestinationError(
            output_dir,
            reason="an unmanaged path already exists",
        )


def _preflight_export_target_fidelity(
    *, source_manifest: DatasetManifest, output_format: StorageFormat
) -> None:
    """Reject source contracts that the selected target cannot represent."""

    if output_format is not StorageFormat.LEROBOT:
        return
    metadata = source_manifest.metadata
    validate_demo_io_contract(metadata.get("demo_io_contract"))


def _export_robomimic_to_srb_jsonl(
    *,
    source_data_path: Path,
    source_manifest: DatasetManifest,
    source_manifest_path: Path,
    source_provenance_path: Path,
    output_dir: Path,
) -> DatasetExportResult:
    h5py = _require_h5py()
    data_dir = output_dir / "data"

    num_episodes = 0
    total_samples = 0
    episode_recording_refs: list[dict[str, object]] = []
    source_metadata = _thawed_manifest_metadata(source_manifest)
    demo_io_contract = source_metadata.get("demo_io_contract")
    validate_demo_io_contract(demo_io_contract)
    decoded_budget = ByteBudget(
        "HDF5 export decoded bytes", MAX_DECODED_AGGREGATE_BYTES
    )
    # Check size before h5py materializes the source.
    require_dataset_file_size(source_data_path, resource="robomimic HDF5 source")
    with h5py.File(source_data_path, "r") as h5:
        _preflight_hdf5_export_source(h5, source_data_path)
        _require_hdf5_contract_matches_manifest(h5, demo_io_contract)
        declared_shapes = declared_observation_shapes(demo_io_contract)
        if demo_io_contract is not None:
            _require_hdf5_demo_io_contract_datasets(h5)
        for demo_key in _demo_keys(h5):
            _admit_hdf5_json_conversion(h5["data"][demo_key], decoded_budget)
        data_dir.mkdir(parents=True, exist_ok=True)
        for episode_index, demo_key in enumerate(_demo_keys(h5)):
            demo = h5["data"][demo_key]
            reference = _read_hdf5_episode_recording_reference(demo)
            if reference is not None:
                episode_recording_refs.append(
                    {
                        "episode_index": episode_index,
                        "episode_key": demo_key,
                        "reference": reference,
                    }
                )
            episode_path = data_dir / f"episode_{episode_index:06d}.jsonl"
            num_samples = int(demo["actions"].shape[0])
            with episode_path.open("w", encoding="utf-8") as f:
                for step_index in range(num_samples):
                    row = _demo_step_to_jsonable_row(
                        demo=demo,
                        episode_index=episode_index,
                        episode_key=demo_key,
                        step_index=step_index,
                        declared_shapes=declared_shapes,
                    )
                    write_srb_jsonl_row(f, row)
            num_episodes += 1
            total_samples += num_samples

    extra_metadata: dict[str, Any] = {}
    if demo_io_contract is not None:
        extra_metadata["demo_io_contract"] = demo_io_contract
    if episode_recording_refs:
        extra_metadata["episode_recording_refs"] = episode_recording_refs

    manifest = _build_export_manifest(
        source_manifest=source_manifest,
        source_manifest_path=source_manifest_path,
        source_provenance_path=source_provenance_path,
        output_format=StorageFormat.SRB_JSONL,
        dataset_path="data",
        num_episodes=num_episodes,
        total_samples=total_samples,
        extra_metadata=extra_metadata or None,
    )
    manifest_path = _write_staged_dataset_manifest(
        manifest,
        output_dir / DEFAULT_MANIFEST_RELPATH,
        stage_root=output_dir,
    )
    return DatasetExportResult(
        manifest_path=manifest_path,
        num_episodes=num_episodes,
        total_samples=total_samples,
    )


def _export_robomimic_to_lerobot_v3(
    *,
    source_data_path: Path,
    source_manifest: DatasetManifest,
    source_manifest_path: Path,
    source_provenance_path: Path,
    output_dir: Path,
) -> DatasetExportResult:
    fps = _require_lerobot_fps(source_manifest)
    _preflight_export_target_fidelity(
        source_manifest=source_manifest,
        output_format=StorageFormat.LEROBOT,
    )

    h5py = _require_h5py()
    _require_pyarrow()
    from srb.datasets.export_parquet import (
        preflight_parquet_tables,
        write_preflighted_parquet_tables,
    )

    data_dir = output_dir / "data" / "chunk-000"
    episodes_dir = output_dir / "meta" / "episodes" / "chunk-000"
    meta_dir = output_dir / "meta"

    total_samples = 0
    action_dim: int | None = None
    obs_feature_specs: dict[str, dict[str, Any]] = {}
    episode_recording_refs: list[dict[str, object]] = []
    feature_budget = ByteBudget(
        "LeRobot export feature bytes", MAX_METADATA_AGGREGATE_BYTES
    )
    source_metadata = _thawed_manifest_metadata(source_manifest)
    declared_demo_io_contract = source_metadata.get("demo_io_contract")
    certified = declared_demo_io_contract is not None
    transition_extension: dict[str, Any] | None = None

    require_dataset_file_size(source_data_path, resource="robomimic HDF5 source")
    with h5py.File(source_data_path, "r") as h5:
        _preflight_hdf5_export_source(h5, source_data_path)
        _require_hdf5_contract_matches_manifest(h5, declared_demo_io_contract)
        declared_shapes = declared_observation_shapes(declared_demo_io_contract)
        demo_keys = _demo_keys(h5)
        num_episodes = len(demo_keys)
        declared_budget = ByteBudget(
            "HDF5 export decoded bytes", MAX_DECODED_AGGREGATE_BYTES
        )
        for episode_index, demo_key in enumerate(demo_keys):
            demo = h5["data"][demo_key]
            if certified:
                required = {"dones", "terminated", "truncated", "obs", "next_obs"}
                missing = sorted(required.difference(demo.keys()))
                if missing:
                    raise ValueError(
                        f"certified HDF5 demo {demo_key!r} missing required "
                        f"fields: {', '.join(missing)}"
                    )
                for flag in ("dones", "terminated", "truncated"):
                    if np.dtype(demo[flag].dtype).kind != "b":
                        raise ValueError(
                            f"certified HDF5 demo {demo_key!r} {flag} must be boolean"
                        )
                _validate_hdf5_transition_observations(demo, demo_key)
            reference = _read_hdf5_episode_recording_reference(demo)
            if reference is not None:
                episode_recording_refs.append(
                    {
                        "episode_index": episode_index,
                        "episode_key": demo_key,
                        "reference": reference,
                    }
                )
            _admit_hdf5_json_conversion(demo, declared_budget)
            total_samples += int(demo["actions"].shape[0])
            if action_dim is None:
                action_dim = _demo_action_dim(demo)
            demo_specs = _demo_obs_feature_specs(demo, declared_shapes)
            if certified:
                if transition_extension is None:
                    transition_extension = build_transition_extension(
                        tuple(path.split("/")) for path in demo_specs
                    )
                demo_specs = {
                    encode_observation_path(tuple(path.split("/"))): spec
                    for path, spec in demo_specs.items()
                }
                expected_paths = set(transition_extension["observation_paths"])
                if set(demo_specs) != expected_paths:
                    raise ValueError(
                        f"certified HDF5 demo {demo_key!r} observation leaf paths "
                        "do not match the first episode"
                    )
                for key, spec in demo_specs.items():
                    previous = obs_feature_specs.get(key)
                    if previous is not None and (
                        previous.get("shape") != spec.get("shape")
                        or previous.get("source_shape") != spec.get("source_shape")
                    ):
                        raise ValueError(
                            f"certified HDF5 feature {key!r} shape changes between episodes"
                        )
                _require_lerobot_feature_ranks(demo_specs, context=demo_key)
            _merge_obs_feature_specs(obs_feature_specs, demo_specs, feature_budget)

        image_views: list[VisualImageView] = []
        if transition_extension is not None:
            feature_budget.consume(
                len(
                    json.dumps(
                        transition_extension, ensure_ascii=False, sort_keys=True
                    ).encode("utf-8")
                )
                + sum(
                    256 + 4 * len(key) + 4 * len(spec["source_shape"])
                    for key, spec in obs_feature_specs.items()
                )
            )
            _apply_certified_feature_dtypes(
                obs_feature_specs, transition_extension, declared_demo_io_contract
            )
            image_views = _certified_lerobot_image_views(
                declared_demo_io_contract, transition_extension
            )

        data_dir.mkdir(parents=True, exist_ok=True)
        episodes_dir.mkdir(parents=True, exist_ok=True)

        def frames() -> Iterator[Mapping[str, Any]]:
            global_index = 0
            budget = ByteBudget(
                "HDF5 export decoded bytes", MAX_DECODED_AGGREGATE_BYTES
            )
            for episode_index, demo_key in enumerate(demo_keys):
                demo = h5["data"][demo_key]
                _admit_hdf5_json_conversion(demo, budget)
                for step_index in range(int(demo["actions"].shape[0])):
                    row = _demo_step_to_jsonable_row(
                        demo=demo,
                        episode_index=episode_index,
                        episode_key=demo_key,
                        step_index=step_index,
                        declared_shapes=declared_shapes,
                    )
                    yield _lerobot_frame_row(
                        row,
                        global_index=global_index,
                        fps=fps,
                        transition_extension=transition_extension,
                        action_dim=action_dim,
                        obs_feature_specs=obs_feature_specs,
                        image_views=image_views,
                    )
                    global_index += 1

        def episodes() -> Iterator[Mapping[str, Any]]:
            offset = 0
            for episode_index, demo_key in enumerate(demo_keys):
                sample_count = int(h5["data"][demo_key]["actions"].shape[0])
                yield _lerobot_episode_row(
                    episode_index, sample_count, offset, source_manifest
                )
                offset += sample_count

        plans = preflight_parquet_tables(
            (
                (data_dir / "file-000.parquet", frames),
                (episodes_dir / "file-000.parquet", episodes),
                (
                    meta_dir / "tasks.parquet",
                    lambda: iter(
                        (
                            {
                                "task_index": 0,
                                "task": source_manifest.identity.task,
                            },
                        )
                    ),
                ),
            )
        )
        write_preflighted_parquet_tables(plans)
    (meta_dir / "stats.json").write_text("{}\n", encoding="utf-8")
    (meta_dir / "info.json").write_text(
        json.dumps(
            _lerobot_info_payload(
                source_manifest,
                action_dim=action_dim,
                obs_feature_specs=obs_feature_specs,
                total_episodes=num_episodes,
                total_frames=total_samples,
                episode_recording_refs=episode_recording_refs,
                transition_extension=transition_extension,
                image_views=image_views,
            ),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    manifest = _build_export_manifest(
        source_manifest=source_manifest,
        source_manifest_path=source_manifest_path,
        source_provenance_path=source_provenance_path,
        output_format=StorageFormat.LEROBOT,
        dataset_path="data",
        num_episodes=num_episodes,
        total_samples=total_samples,
        extra_metadata={
            # Keep top-level keys; LeRobot columns use encoded leaf paths.
            "obs_keys": (
                sorted(
                    {
                        components[0]
                        for components in transition_extension[
                            "observation_paths"
                        ].values()
                    }
                )
                if transition_extension is not None
                else sorted(obs_feature_specs)
            ),
            "obs_shapes": {
                key: (
                    spec["source_shape"]
                    if transition_extension is not None
                    else spec["shape"]
                )
                for key, spec in sorted(obs_feature_specs.items())
            },
            "action_dim": action_dim,
            **(
                {"episode_recording_refs": episode_recording_refs}
                if episode_recording_refs
                else {}
            ),
            **(
                {"srb_transition_extension": transition_extension}
                if transition_extension is not None
                else {}
            ),
        },
    )
    manifest_path = _write_staged_dataset_manifest(
        manifest,
        output_dir / DEFAULT_MANIFEST_RELPATH,
        stage_root=output_dir,
    )
    return DatasetExportResult(
        manifest_path=manifest_path,
        num_episodes=num_episodes,
        total_samples=total_samples,
    )


def _export_srb_jsonl_to_robomimic_hdf5(
    *,
    source_data_path: Path,
    source_manifest: DatasetManifest,
    source_manifest_path: Path,
    source_provenance_path: Path,
    output_dir: Path,
) -> DatasetExportResult:
    h5py = _require_h5py()
    source_metadata = _thawed_manifest_metadata(source_manifest)
    demo_io_contract = source_metadata.get("demo_io_contract")
    validate_demo_io_contract(demo_io_contract)
    episode_recording_by_index = _episode_recording_refs_by_index(source_metadata)
    train_split = float(source_metadata.get("train_split", 1.0))
    if not 0.0 <= train_split <= 1.0:
        raise ValueError(f"train_split must be in [0, 1], got {train_split}")

    data_dir = output_dir / "data"
    output_hdf5_path = data_dir / "demos.hdf5"
    decoded_budget = ByteBudget(
        "HDF5 export decoded bytes", MAX_DECODED_AGGREGATE_BYTES
    )

    episode_recording_refs: list[dict[str, object]] = []
    with open_jsonl_export_index(source_data_path) as index:
        obs_keys, action_dim = _jsonl_export_schema(index, source_metadata)
        if len(obs_keys) != len(set(obs_keys)):
            raise ValueError("HDF5 export observation keys must not contain duplicates")
        num_episodes = index.episode_count
        total_samples = index.total_samples
        num_train = _robomimic_train_count(num_episodes, train_split)
        data_dir.mkdir(parents=True, exist_ok=True)
        with h5py.File(output_hdf5_path, "w") as h5:
            data_group = h5.create_group("data")
            export_env_name = _export_env_name(source_manifest)
            export_env_kwargs = _export_env_kwargs(source_manifest)
            data_group.attrs["env_name"] = export_env_name
            data_group.attrs["env_kwargs"] = json.dumps(
                export_env_kwargs, sort_keys=True
            )
            data_group.attrs["env_args"] = robomimic_env_args(
                export_env_name, export_env_kwargs
            )
            action_schema = source_metadata.get("action_schema")
            if action_schema is not None:
                data_group.attrs["action_schema"] = json.dumps(
                    action_schema, sort_keys=True
                )
            if demo_io_contract is not None:
                data_group.attrs["demo_io_contract"] = _encode_hdf5_demo_io_contract(
                    demo_io_contract
                )

            for episode_index, episode in enumerate(index.iter_episodes()):
                demo_group = data_group.create_group(f"demo_{episode_index}")
                _write_jsonl_hdf5_episode(
                    demo_group,
                    index,
                    episode.episode_index,
                    episode.sample_count,
                    obs_keys,
                    decoded_budget,
                    declared_dtypes=declared_observation_dtypes(demo_io_contract),
                )
                reference = episode_recording_by_index.get(episode.episode_index)
                if reference is not None:
                    demo_group.attrs["srb_episode_recording"] = np.bytes_(
                        _encode_episode_recording_reference(reference).encode("utf-8")
                    )
                    # Record the output demo ordinal, not its source episode id.
                    episode_recording_refs.append(
                        {
                            "episode_index": episode_index,
                            "episode_key": f"demo_{episode_index}",
                            "reference": dict(reference),
                        }
                    )
            written = {episode.episode_index for episode in index.iter_episodes()}
            unmatched = sorted(set(episode_recording_by_index) - written)
            if unmatched:
                raise ValueError(
                    "episode_recording_refs refer to missing JSONL episodes: "
                    f"{unmatched}"
                )

            mask_group = h5.create_group("mask")
            for split, first, stop in (
                ("train", 0, num_train),
                ("valid", num_train, num_episodes),
            ):
                # Use S1 for empty splits, otherwise size for the longest key.
                width = len(f"demo_{stop - 1}") if stop > first else 1
                dtype = np.dtype(f"S{width}")
                decoded_budget.consume(
                    _checked_export_array((stop - first,), dtype, f"mask/{split}")
                )
                dataset = mask_group.create_dataset(
                    split, shape=(stop - first,), dtype=dtype
                )
                for offset, episode_index in enumerate(range(first, stop)):
                    dataset[offset] = f"demo_{episode_index}".encode("ascii")
            data_group.attrs["total_samples"] = total_samples
            data_group.attrs["num_demos"] = num_episodes
            data_group.attrs["num_train"] = num_train
            data_group.attrs["num_valid"] = num_episodes - num_train

    manifest = _build_export_manifest(
        source_manifest=source_manifest,
        source_manifest_path=source_manifest_path,
        source_provenance_path=source_provenance_path,
        output_format=StorageFormat.ROBOMIMIC_HDF5,
        dataset_path="data/demos.hdf5",
        num_episodes=num_episodes,
        total_samples=total_samples,
        extra_metadata={
            "env_name": _export_env_name(source_manifest),
            "env_kwargs": _export_env_kwargs(source_manifest),
            "obs_keys": obs_keys,
            "action_dim": action_dim,
            "num_demos": num_episodes,
            "num_train": num_train,
            "num_valid": num_episodes - num_train,
            "train_split": train_split,
            **(
                {"episode_recording_refs": episode_recording_refs}
                if episode_recording_refs
                else {}
            ),
        },
    )
    manifest_path = _write_staged_dataset_manifest(
        manifest,
        output_dir / DEFAULT_MANIFEST_RELPATH,
        stage_root=output_dir,
    )
    return DatasetExportResult(
        manifest_path=manifest_path,
        num_episodes=num_episodes,
        total_samples=total_samples,
    )


def _export_srb_jsonl_to_srb_jsonl(
    *,
    source_data_path: Path,
    source_manifest: DatasetManifest,
    source_manifest_path: Path,
    source_provenance_path: Path,
    output_dir: Path,
) -> DatasetExportResult:
    source_metadata = _thawed_manifest_metadata(source_manifest)
    validate_demo_io_contract(source_metadata.get("demo_io_contract"))
    with open_jsonl_export_index(source_data_path) as index:
        with closing(index.iter_rows()) as rows:
            _require_jsonl_demo_io_contract_rows(source_metadata, rows)
        num_episodes = index.episode_count
        total_samples = index.total_samples
    data_dir = output_dir / "data"
    if source_data_path.is_dir():
        if source_data_path.resolve() != data_dir.resolve():
            if data_dir.exists():
                shutil.rmtree(data_dir)
            shutil.copytree(source_data_path, data_dir)
        dataset_path = "data"
    else:
        data_dir.mkdir(parents=True, exist_ok=True)
        output_data_path = data_dir / source_data_path.name
        if source_data_path.resolve() != output_data_path.resolve():
            shutil.copy2(source_data_path, output_data_path)
        dataset_path = f"data/{source_data_path.name}"

    manifest = _build_export_manifest(
        source_manifest=source_manifest,
        source_manifest_path=source_manifest_path,
        source_provenance_path=source_provenance_path,
        output_format=StorageFormat.SRB_JSONL,
        dataset_path=dataset_path,
        num_episodes=num_episodes,
        total_samples=total_samples,
    )
    manifest_path = _write_staged_dataset_manifest(
        manifest,
        output_dir / DEFAULT_MANIFEST_RELPATH,
        stage_root=output_dir,
    )
    return DatasetExportResult(
        manifest_path=manifest_path,
        num_episodes=num_episodes,
        total_samples=total_samples,
    )


def _export_srb_jsonl_to_lerobot_v3(
    *,
    source_data_path: Path,
    source_manifest: DatasetManifest,
    source_manifest_path: Path,
    source_provenance_path: Path,
    output_dir: Path,
) -> DatasetExportResult:
    fps = _require_lerobot_fps(source_manifest)
    source_metadata = _thawed_manifest_metadata(source_manifest)
    _preflight_export_target_fidelity(
        source_manifest=source_manifest,
        output_format=StorageFormat.LEROBOT,
    )

    _require_pyarrow()
    from srb.datasets.export_parquet import (
        preflight_parquet_tables,
        write_preflighted_parquet_tables,
    )

    data_dir = output_dir / "data" / "chunk-000"
    episodes_dir = output_dir / "meta" / "episodes" / "chunk-000"
    meta_dir = output_dir / "meta"
    data_dir.mkdir(parents=True, exist_ok=True)
    episodes_dir.mkdir(parents=True, exist_ok=True)

    with open_jsonl_export_index(source_data_path) as index:
        obs_keys, action_dim = _jsonl_export_schema(
            index,
            source_metadata,
            allow_lerobot_path_components=True,
        )
        certified = source_metadata.get("demo_io_contract") is not None
        with closing(index.iter_rows()) as rows:
            obs_feature_specs = _canonical_obs_feature_specs(
                rows, obs_keys, include_next=certified
            )
        transition_extension = None
        image_views: list[VisualImageView] = []
        if certified:
            with closing(index.iter_rows()) as rows:
                first_row = next(rows, None)
            if first_row is None:
                raise ValueError("certified canonical dataset has no rows")
            transition_extension = build_transition_extension(
                path for path, _ in _iter_nested_observation_leaves(first_row["obs"])
            )
            _require_lerobot_feature_ranks(
                obs_feature_specs, context="canonical dataset"
            )
            feature_budget = ByteBudget(
                "LeRobot export feature bytes", MAX_METADATA_AGGREGATE_BYTES
            )
            feature_budget.consume(
                len(
                    json.dumps(
                        transition_extension, ensure_ascii=False, sort_keys=True
                    ).encode("utf-8")
                )
                + sum(
                    256 + 4 * len(key) + 4 * len(spec["source_shape"])
                    for key, spec in obs_feature_specs.items()
                )
            )
            demo_io_contract = source_metadata.get("demo_io_contract")
            _apply_certified_feature_dtypes(
                obs_feature_specs, transition_extension, demo_io_contract
            )
            image_views = _certified_lerobot_image_views(
                demo_io_contract, transition_extension
            )
        num_episodes = index.episode_count
        total_samples = index.total_samples
        episode_recording_refs = _remap_episode_recording_refs(
            source_metadata,
            index.iter_episodes(),
        )

        def frames() -> Iterator[Mapping[str, Any]]:
            global_index = 0
            for episode_index, episode in enumerate(index.iter_episodes()):
                for frame_index, row in enumerate(
                    index.iter_rows(episode.episode_index, sort_steps=True)
                ):
                    frame_row = dict(row)
                    frame_row["episode_index"] = episode_index
                    frame_row["step_index"] = frame_index
                    yield _lerobot_frame_row(
                        frame_row,
                        global_index=global_index,
                        fps=fps,
                        transition_extension=transition_extension,
                        action_dim=action_dim,
                        obs_feature_specs=obs_feature_specs,
                        image_views=image_views,
                    )
                    global_index += 1

        def episodes() -> Iterator[Mapping[str, Any]]:
            offset = 0
            for episode_index, episode in enumerate(index.iter_episodes()):
                yield _lerobot_episode_row(
                    episode_index, episode.sample_count, offset, source_manifest
                )
                offset += episode.sample_count

        plans = preflight_parquet_tables(
            (
                (data_dir / "file-000.parquet", frames),
                (episodes_dir / "file-000.parquet", episodes),
                (
                    meta_dir / "tasks.parquet",
                    lambda: iter(
                        (
                            {
                                "task_index": 0,
                                "task": source_manifest.identity.task,
                            },
                        )
                    ),
                ),
            )
        )
        write_preflighted_parquet_tables(plans)
    (meta_dir / "stats.json").write_text("{}\n", encoding="utf-8")
    (meta_dir / "info.json").write_text(
        json.dumps(
            _lerobot_info_payload(
                source_manifest,
                action_dim=action_dim,
                obs_feature_specs=obs_feature_specs,
                total_episodes=num_episodes,
                total_frames=total_samples,
                episode_recording_refs=episode_recording_refs,
                transition_extension=transition_extension,
                image_views=image_views,
            ),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    manifest = _build_export_manifest(
        source_manifest=source_manifest,
        source_manifest_path=source_manifest_path,
        source_provenance_path=source_provenance_path,
        output_format=StorageFormat.LEROBOT,
        dataset_path="data",
        num_episodes=num_episodes,
        total_samples=total_samples,
        extra_metadata={
            "obs_keys": obs_keys,
            "obs_shapes": {
                key: (
                    spec["source_shape"]
                    if transition_extension is not None
                    else spec["shape"]
                )
                for key, spec in sorted(obs_feature_specs.items())
            },
            "action_dim": action_dim,
            **(
                {"episode_recording_refs": episode_recording_refs}
                if episode_recording_refs
                else {}
            ),
            **(
                {"srb_transition_extension": transition_extension}
                if transition_extension is not None
                else {}
            ),
        },
    )
    manifest_path = _write_staged_dataset_manifest(
        manifest,
        output_dir / DEFAULT_MANIFEST_RELPATH,
        stage_root=output_dir,
    )
    return DatasetExportResult(
        manifest_path=manifest_path,
        num_episodes=num_episodes,
        total_samples=total_samples,
    )


def _lerobot_info_payload(
    source_manifest: DatasetManifest,
    *,
    action_dim: int | None,
    obs_feature_specs: dict[str, dict[str, Any]],
    total_episodes: int,
    total_frames: int,
    episode_recording_refs: list[dict[str, object]] | None = None,
    transition_extension: dict[str, Any] | None = None,
    image_views: Sequence[VisualImageView] = (),
) -> dict[str, Any]:
    source_metadata = _thawed_manifest_metadata(source_manifest)
    payload = {
        "codebase_version": "v3.0",
        "fps": source_metadata.get("fps"),
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
        "video_path": (
            "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
        ),
        "features": _lerobot_features(
            source_manifest,
            action_dim=action_dim,
            obs_feature_specs=obs_feature_specs,
            certified=transition_extension is not None,
            image_views=image_views,
        ),
        "total_episodes": total_episodes,
        "total_frames": total_frames,
    }
    action_schema = source_metadata.get("action_schema")
    if action_schema is not None:
        payload["srb_action_schema"] = action_schema
    if episode_recording_refs is None:
        source_refs = source_metadata.get("episode_recording_refs")
        if isinstance(source_refs, list):
            episode_recording_refs = source_refs
    if episode_recording_refs:
        payload["srb_episode_recording_refs"] = episode_recording_refs
    if transition_extension is not None:
        payload["srb_transition_extension"] = transition_extension
        payload["total_tasks"] = 1
        demo_io_contract = source_metadata.get("demo_io_contract")
        if demo_io_contract is not None:
            payload["srb_demo_io_contract"] = demo_io_contract
    if image_views:
        payload["srb_visual_views"] = describe_image_views(
            image_views, (transition_extension or {}).get("observation_paths", {})
        )
    return payload


def _episode_recording_refs_by_index(
    metadata: Mapping[str, Any],
) -> dict[int, Mapping[str, object]]:
    """Validate manifest episode references before rebuilding HDF5 demos."""
    raw_refs = metadata.get("episode_recording_refs")
    if raw_refs is None:
        return {}
    if not isinstance(raw_refs, list):
        raise ValueError("episode_recording_refs must be a list")
    references: dict[int, Mapping[str, object]] = {}
    for entry in raw_refs:
        if not isinstance(entry, Mapping):
            raise ValueError("episode_recording_refs entries must be objects")
        index = entry.get("episode_index")
        if type(index) is not int or index < 0:
            raise ValueError(
                "episode_recording_refs episode_index must be non-negative"
            )
        if index in references:
            raise ValueError(f"duplicate episode_recording_refs index: {index}")
        reference = entry.get("reference")
        if not isinstance(reference, Mapping):
            raise ValueError("episode_recording_refs reference must be an object")
        _encode_episode_recording_reference(reference)
        references[index] = dict(reference)
    return references


def _remap_episode_recording_refs(
    metadata: Mapping[str, Any],
    episodes: Iterable[Any],
) -> list[dict[str, object]]:
    """Map source JSONL episode ids to the export's contiguous ordinals."""
    references = _episode_recording_refs_by_index(metadata)
    if not references:
        return []

    remapped: list[dict[str, object]] = []
    matched: set[int] = set()
    for output_index, episode in enumerate(episodes):
        source_index = int(episode.episode_index)
        reference = references.get(source_index)
        if reference is None:
            continue
        matched.add(source_index)
        remapped.append(
            {
                "episode_index": output_index,
                "episode_key": f"episode_{output_index:06d}",
                "reference": dict(reference),
            }
        )

    unmatched = sorted(set(references) - matched)
    if unmatched:
        raise ValueError(
            f"episode_recording_refs refer to missing JSONL episodes: {unmatched}"
        )
    return remapped


def _build_export_manifest(
    *,
    source_manifest: DatasetManifest,
    source_manifest_path: Path,
    source_provenance_path: Path,
    output_format: StorageFormat,
    dataset_path: str,
    num_episodes: int,
    total_samples: int,
    extra_metadata: dict[str, Any] | None = None,
) -> DatasetManifest:
    source_metadata = _thawed_manifest_metadata(source_manifest)
    metadata = {
        "source_path": str(source_provenance_path),
        "source_manifest_path": str(source_manifest_path),
        "source_storage_format": source_manifest.storage_format.value,
        "num_episodes": num_episodes,
        "total_samples": total_samples,
    }
    for key in (
        "env_name",
        "env_kwargs",
        "obs_keys",
        "action_dim",
        "fps",
        "train_split",
        "episode_recording_refs",
        "provenance",
        "action_schema",
        "demo_io_contract",
    ):
        if key in source_metadata:
            metadata[key] = source_metadata[key]
    if extra_metadata:
        metadata.update(extra_metadata)
    return build_dataset_manifest(
        identity=source_manifest.identity,
        dataset_path=dataset_path,
        split=source_manifest.split,
        controller=source_manifest.controller,
        storage_format=output_format,
        metadata=metadata,
    )


def _write_export_dataset_card(manifest_path: Path) -> None:
    """Write a deterministic card inside the unpublished export stage."""
    from srb.datasets.manifest import read_dataset_manifest

    manifest = read_dataset_manifest(manifest_path)
    if manifest.storage_format not in {StorageFormat.SRB_JSONL, StorageFormat.LEROBOT}:
        return
    from srb.datasets.huggingface import build_huggingface_dataset_card

    (manifest_path.parent.parent / "README.md").write_text(
        build_huggingface_dataset_card(manifest), encoding="utf-8"
    )


def _jsonl_export_schema(
    index: JsonlExportIndex,
    metadata: Mapping[str, Any],
    *,
    allow_lerobot_path_components: bool = False,
) -> tuple[list[str], int]:
    """Retain the historical episode/arrival order for metadata inference."""

    declared = metadata.get("obs_keys")
    if declared and (
        not isinstance(declared, list)
        or any(not isinstance(key, str) for key in declared)
    ):
        raise ValueError("metadata obs_keys must be a list of strings")
    obs_keys = list(declared) if declared else []
    seen = set(obs_keys)
    action_dim: int | None = None
    certified_obs_schema: dict[str, tuple[int, ...]] | None = (
        {} if metadata.get("demo_io_contract") is not None else None
    )
    budget = ByteBudget("JSONL export schema bytes", MAX_METADATA_AGGREGATE_BYTES)
    for row in index.iter_rows():
        _require_jsonl_demo_io_contract_rows(metadata, (row,))
        if certified_obs_schema is not None:
            _validate_certified_jsonl_observation_schema(
                row,
                expected_schema=certified_obs_schema,
                schema_budget=budget,
                allow_lerobot_path_components=allow_lerobot_path_components,
            )
        if action_dim is None:
            shape, _ = _json_array_declaration(
                row["action"], resource="canonical action"
            )
            action_dim = math.prod(shape) if shape else 1
        if not declared:
            for key in row["obs"]:
                if key not in seen:
                    budget.consume(256 + 4 * len(key))
                    seen.add(key)
                    obs_keys.append(key)
    if not obs_keys:
        raise ValueError("canonical dataset has no observation keys")
    if action_dim is None:
        raise ValueError("canonical dataset has no recorded steps")
    return obs_keys, action_dim


def _checked_export_array(shape: tuple[int, ...], dtype: Any, resource: str) -> int:
    return checked_shape_nbytes(
        shape,
        dtype,
        resource=resource,
        max_bytes=MAX_ARRAY_BYTES,
        max_elements=MAX_ARRAY_ELEMENTS,
    )


def _json_array_declaration(
    value: object, *, resource: str
) -> tuple[tuple[int, ...], np.dtype]:
    """Infer dense shape and dtype before allocating padded NumPy arrays."""

    def inspect(item: object, depth: int) -> tuple[tuple[int, ...], np.dtype]:
        if depth > 8:
            raise ResourceLimitError(f"{resource} rank", limit=8, observed=depth)
        if isinstance(item, (list, tuple)):
            child_shape: tuple[int, ...] | None = None
            dtype: np.dtype | None = None
            for child in item:
                shape, child_dtype = inspect(child, depth + 1)
                if child_shape is not None and child_shape != shape:
                    raise ValueError(
                        f"{resource} values cannot be stacked: inconsistent shape"
                    )
                child_shape = shape
                dtype = (
                    child_dtype
                    if dtype is None
                    else (
                        dtype
                        if dtype == child_dtype
                        else np.result_type(dtype, child_dtype)
                    )
                )
            return (
                len(item),
                *(child_shape or ()),
            ), dtype if dtype is not None else np.dtype("float64")
        if isinstance(item, bool):
            return (), np.dtype("bool")
        if isinstance(item, int):
            if -(1 << 63) <= item < (1 << 63):
                return (), np.dtype("int64")
            if 0 <= item < (1 << 64):
                return (), np.dtype("uint64")
            return (), np.dtype("object")
        if isinstance(item, float):
            return (), np.dtype("float64")
        if isinstance(item, str):
            return (), np.dtype(f"U{max(1, len(item))}")
        if isinstance(item, np.ndarray):
            return item.shape, item.dtype
        if isinstance(item, np.generic):
            return (), item.dtype
        if item is None or isinstance(item, Mapping):
            return (), np.dtype("object")
        raise ValueError(
            f"{resource} has unsupported array value {type(item).__name__}"
        )

    shape, dtype = inspect(value, 0)
    _checked_export_array(shape, dtype, resource)
    return shape, dtype


def _validate_certified_jsonl_observation_schema(
    row: Mapping[str, Any],
    *,
    expected_schema: dict[tuple[str, ...], tuple[int, ...]],
    schema_budget: ByteBudget,
    allow_lerobot_path_components: bool = False,
) -> None:
    """Require matching explicit transition trees within the shared schema budget."""
    context = f"episode {row['episode_index']} row {row['step_index']}"
    obs = row["obs"]
    if not isinstance(obs, Mapping):
        raise ValueError(f"{context} observation must be a non-empty mapping")
    next_obs = row.get("next_obs")
    if not isinstance(next_obs, Mapping):
        raise ValueError(f"{context} requires explicit next_obs mapping")

    first_row = not expected_schema
    mismatch_samples: list[tuple[str, str]] = []
    mismatch_count = 0

    def admit_temporary(amount: int) -> None:
        observed = schema_budget.consumed + amount
        if observed > schema_budget.limit:
            raise ResourceLimitError(
                schema_budget.resource,
                limit=schema_budget.limit,
                observed=observed,
            )

    def clip_path(path: str) -> str:
        if len(path) <= _CERTIFIED_SCHEMA_DIAGNOSTIC_PATH_LIMIT:
            return path
        return path[: _CERTIFIED_SCHEMA_DIAGNOSTIC_PATH_LIMIT - 3] + "..."

    def display_path(path: tuple[str, ...]) -> str:
        if not path:
            return "<root>"
        result = ""
        for component in path:
            separator = "/" if result else ""
            remaining = _CERTIFIED_SCHEMA_DIAGNOSTIC_PATH_LIMIT - len(result)
            if remaining <= len(separator):
                return clip_path(result)
            remaining -= len(separator)
            if len(component) > remaining:
                marker = "..."
                if remaining <= len(marker):
                    return clip_path(result + separator)
                return clip_path(
                    result + separator + component[: remaining - len(marker)] + marker
                )
            result += separator + component
        return result

    def record_mismatch(kind: str, path: tuple[str, ...] | str) -> None:
        nonlocal mismatch_count
        mismatch_count += 1
        if len(mismatch_samples) >= _CERTIFIED_SCHEMA_MISMATCH_SAMPLE_LIMIT:
            return
        shown_path = display_path(path) if isinstance(path, tuple) else clip_path(path)
        schema_budget.consume(
            _CERTIFIED_SCHEMA_SAMPLE_OVERHEAD + 4 * (len(kind) + len(shown_path))
        )
        mismatch_samples.append((kind, shown_path))

    def compare(
        obs_value: object,
        next_value: object,
        path: tuple[str, ...],
        path_chars: int,
    ) -> None:
        obs_mapping = isinstance(obs_value, Mapping)
        next_mapping = isinstance(next_value, Mapping)
        if obs_mapping and not obs_value:
            raise ValueError(
                f"{context} observation {display_path(path)!r} must not be an empty mapping"
            )
        if next_mapping and not next_value:
            raise ValueError(
                f"{context} next_obs {display_path(path)!r} must not be an empty mapping"
            )
        if obs_mapping != next_mapping:
            record_mismatch("structure", path)
            return
        if obs_mapping:
            assert isinstance(obs_value, Mapping)
            assert isinstance(next_value, Mapping)
            for raw_key, child in obs_value.items():
                if allow_lerobot_path_components:
                    if not isinstance(raw_key, str) or not raw_key:
                        raise ValueError(
                            "LeRobot observation path components must be non-empty strings"
                        )
                else:
                    _validate_hdf5_observation_key(raw_key)
                child_path = (*path, raw_key)
                child_path_chars = path_chars + len(raw_key) + (1 if path else 0)
                if raw_key not in next_value:
                    record_mismatch("missing", child_path)
                    continue
                compare(
                    child,
                    next_value[raw_key],
                    child_path,
                    child_path_chars,
                )
            for raw_key in next_value:
                if allow_lerobot_path_components:
                    if not isinstance(raw_key, str) or not raw_key:
                        raise ValueError(
                            "LeRobot observation path components must be non-empty strings"
                        )
                else:
                    _validate_hdf5_observation_key(raw_key)
                if raw_key not in obs_value:
                    child_path = (*path, raw_key)
                    record_mismatch("extra", child_path)
            return
        if not path:
            raise ValueError(f"{context} observation must be a non-empty mapping")

        path_for_error = display_path(path)
        obs_shape, obs_dtype = _json_array_declaration(
            obs_value, resource=f"{context} obs/{path_for_error}"
        )
        next_shape, next_dtype = _json_array_declaration(
            next_value, resource=f"{context} next_obs/{path_for_error}"
        )
        if obs_shape != next_shape:
            raise ValueError(
                f"{context} next_obs/{path_for_error} shape {next_shape} does not match "
                f"obs/{path_for_error} shape {obs_shape}"
            )
        if obs_dtype.kind not in "biuf" or next_dtype.kind not in "biuf":
            raise ValueError(
                f"{context} obs/next_obs/{path_for_error} uses an unsupported HDF5 dtype"
            )
        if first_row:
            schema_budget.consume(
                _CERTIFIED_SCHEMA_ENTRY_OVERHEAD + 4 * path_chars + 8 * len(obs_shape)
            )
            expected_schema[path] = obs_shape
            return
        admit_temporary(_CERTIFIED_SCHEMA_TEMP_PATH_OVERHEAD + 4 * path_chars)
        expected_shape = expected_schema.get(path)
        if expected_shape is None:
            record_mismatch("schema-extra", path)
        elif expected_shape != obs_shape:
            raise ValueError(
                f"{context} certified observation leaf {path_for_error} shapes "
                f"{obs_shape} and {expected_shape} differ from an earlier row"
            )

    compare(obs, next_obs, (), 0)
    if not first_row:
        for path in expected_schema:
            current: object = obs
            present = True
            for component in path:
                admit_temporary(
                    _CERTIFIED_SCHEMA_TEMP_COMPONENT_OVERHEAD + 4 * len(component)
                )
                if not isinstance(current, Mapping) or component not in current:
                    present = False
                    break
                current = current[component]
            if present and isinstance(current, Mapping):
                present = False
            if not present:
                record_mismatch("schema-missing", path)

    if mismatch_count:
        details = ", ".join(
            f"{kind}={path!r}" for kind, path in sorted(mismatch_samples)
        )
        omitted = mismatch_count - len(mismatch_samples)
        if omitted:
            details += f", ... ({omitted} more)"
        raise ValueError(
            f"{context} certified obs/next_obs leaf paths differ: {details}"
        )


def _jsonl_hdf5_row_values(
    row: Mapping[str, Any], following: Mapping[str, Any], obs_keys: list[str]
) -> Iterator[tuple[str, object]]:
    def leaves(value: object, path: str) -> Iterator[tuple[str, object]]:
        if isinstance(value, Mapping):
            if not value:
                raise ValueError(f"observation {path!r} must not be an empty mapping")
            for key, child in value.items():
                _validate_hdf5_observation_key(key)
                yield from leaves(child, f"{path}/{key}")
        else:
            yield path, value

    for key in obs_keys:
        _validate_hdf5_observation_key(key)
        yield from leaves(_jsonl_observation_value(row, key), f"obs/{key}")
        next_obs = row.get("next_obs")
        next_value = (
            next_obs[key]
            if isinstance(next_obs, Mapping) and key in next_obs
            else _jsonl_observation_value(following, key)
        )
        yield from leaves(next_value, f"next_obs/{key}")
    yield "actions", row["action"]
    yield "rewards", float(row.get("reward", 0.0) or 0.0)
    yield "dones", bool(row.get("done", False))
    yield "terminated", _row_terminated(row)
    yield "truncated", _row_truncated(row)


def _jsonl_rows_with_following(
    index: JsonlExportIndex, episode_index: int
) -> Iterator[tuple[Mapping[str, Any], Mapping[str, Any]]]:
    rows = index.iter_rows(episode_index, sort_steps=True)
    try:
        current = next(rows)
        for following in rows:
            yield current, following
            current = following
        yield current, current
    finally:
        rows.close()


def _write_jsonl_hdf5_episode(
    demo: Any,
    index: JsonlExportIndex,
    episode_index: int,
    sample_count: int,
    obs_keys: list[str],
    decoded_budget: ByteBudget,
    *,
    declared_dtypes: Mapping[tuple[str, ...], np.dtype] | None = None,
) -> None:
    declarations: dict[str, tuple[tuple[int, ...], np.dtype]] = {}
    schema_budget = ByteBudget("HDF5 export schema bytes", MAX_METADATA_AGGREGATE_BYTES)
    success = False
    expected_paths: set[str] | None = None
    for row, following in _jsonl_rows_with_following(index, episode_index):
        seen: set[str] = set()
        for path, value in _jsonl_hdf5_row_values(row, following, obs_keys):
            shape, dtype = _json_array_declaration(value, resource=path)
            if dtype.kind not in "biuf":
                raise ValueError(
                    f"{path} uses an unsupported HDF5 observation/action dtype"
                )
            if path == "actions":
                shape = (math.prod(shape) if shape else 1,)
            declared = _declared_leaf_dtype(declared_dtypes, path)
            if declared is not None:
                # JSON has no dtype metadata; use the manifest declaration.
                cast_declared(value, declared, resource=path)
                dtype = declared
            previous = declarations.get(path)
            if previous is not None:
                if shape != previous[0]:
                    raise ValueError(
                        f"observation {path!r} values cannot be stacked: shape changes between rows"
                    )
                dtype = np.result_type(previous[1], dtype)
            else:
                if len(declarations) >= MAX_DIRECTORY_ENTRIES:
                    raise ResourceLimitError(
                        "HDF5 export datasets",
                        limit=MAX_DIRECTORY_ENTRIES,
                        observed=len(declarations) + 1,
                    )
                schema_budget.consume(256 + 4 * len(path))
            declarations[path] = shape, dtype
            seen.add(path)
        if expected_paths is not None and seen != expected_paths:
            raise ValueError("observation mapping structure changes between rows")
        expected_paths = seen
        success |= bool(row.get("success", False))

    # Reserve each dataset's output and conversion buffers before creation.
    for path, (shape, dtype) in declarations.items():
        decoded_bytes = _checked_export_array((sample_count, *shape), dtype, path)
        decoded_budget.consume(2 * decoded_bytes)
    for path, (shape, dtype) in declarations.items():
        parent_path, _, name = path.rpartition("/")
        parent = demo.require_group(parent_path) if parent_path else demo
        parent.create_dataset(name, shape=(sample_count, *shape), dtype=dtype)
    for step_index, (row, following) in enumerate(
        _jsonl_rows_with_following(index, episode_index)
    ):
        for path, value in _jsonl_hdf5_row_values(row, following, obs_keys):
            shape, dtype = declarations[path]
            demo[path][step_index] = cast_declared(value, dtype, resource=path).reshape(
                shape
            )
    demo.attrs["num_samples"] = sample_count
    demo.attrs["success"] = success


def _declared_leaf_dtype(
    declared_dtypes: Mapping[tuple[str, ...], np.dtype] | None, path: str
) -> np.dtype | None:
    """Contract dtype for an ``obs/...`` or ``next_obs/...`` HDF5 leaf path."""
    if not declared_dtypes:
        return None
    root, _, leaf = path.partition("/")
    if root not in ("obs", "next_obs") or not leaf:
        return None
    return declared_dtype_for(declared_dtypes, tuple(leaf.split("/")))


def _require_jsonl_demo_io_contract_rows(
    metadata: Mapping[str, Any],
    rows: Iterable[Mapping[str, Any]],
) -> None:
    """Reject legacy rows before an export can certify their metadata."""

    if metadata.get("demo_io_contract") is None:
        return
    for row in rows:
        context = f"episode {row['episode_index']} row {row['step_index']}"
        if "next_obs" not in row or row["next_obs"] is None:
            raise ValueError(f"{context} requires explicit next_obs")
        for flag in ("terminated", "truncated"):
            if type(row.get(flag)) is not bool:
                raise ValueError(f"{context} requires explicit {flag} boolean flag")
        done = row.get("done")
        if type(done) is not bool:
            raise ValueError(f"{context} requires explicit done boolean flag")
        if done != (row["terminated"] or row["truncated"]):
            raise ValueError(f"{context} done must equal (terminated or truncated)")


def _jsonl_observation_value(row: Mapping[str, Any], key: str) -> object:
    obs = row.get("obs")
    if not isinstance(obs, Mapping) or key not in obs:
        raise ValueError(f"missing observation {key!r} in canonical row")
    return obs[key]


def _flatten_observation_leaves(
    value: Mapping[str, Any],
    prefix: str = "",
    *,
    _seen_paths: set[str] | None = None,
) -> Iterator[tuple[str, Any]]:
    """Yield slash-delimited observation leaves for LeRobot columns."""

    seen_paths = _seen_paths if _seen_paths is not None else set()
    for raw_key in sorted(value):
        key = str(raw_key)
        path = f"{prefix}/{key}" if prefix else key
        child = value[raw_key]
        if isinstance(child, Mapping):
            yield from _flatten_observation_leaves(child, path, _seen_paths=seen_paths)
        else:
            if path in seen_paths:
                raise ValueError(
                    f"observation paths {path!r} collide after LeRobot flattening"
                )
            seen_paths.add(path)
            yield path, child


def _iter_nested_observation_leaves(
    value: Mapping[str, Any], path: tuple[str, ...] = ()
) -> Iterator[tuple[tuple[str, ...], object]]:
    if not isinstance(value, Mapping) or not value:
        raise ValueError("observation trees must be non-empty mappings")
    for key in sorted(value):
        if not isinstance(key, str) or not key:
            raise ValueError("observation path components must be non-empty strings")
        child = value[key]
        child_path = (*path, key)
        if isinstance(child, Mapping):
            yield from _iter_nested_observation_leaves(child, child_path)
        else:
            yield child_path, child


def _nested_observation_value(
    value: Mapping[str, Any], components: tuple[str, ...]
) -> object:
    current: object = value
    for component in components:
        if not isinstance(current, Mapping) or component not in current:
            raise ValueError(
                f"certified observation is missing path component {component!r}"
            )
        current = current[component]
    if isinstance(current, Mapping):
        raise ValueError("certified observation path resolves to a mapping, not a leaf")
    return current


def _require_lerobot_feature_ranks(
    feature_specs: Mapping[str, Mapping[str, Any]], *, context: str
) -> None:
    for key, spec in feature_specs.items():
        shape = spec.get("shape")
        if not isinstance(shape, list) or not 1 <= len(shape) <= 5:
            raise ValueError(
                f"{context} LeRobot feature {key!r} has unsupported rank "
                f"{len(shape) if isinstance(shape, list) else 'unknown'}; "
                "certified export supports ranks one through five"
            )


def _demo_keys(h5: h5py.File) -> list[str]:
    return sorted(h5["data"].keys(), key=_demo_sort_key)


def _demo_sort_key(key: str) -> tuple[int, str]:
    try:
        return int(key.rsplit("_", 1)[1]), key
    except (IndexError, ValueError):
        return 10**12, key


def _preflight_hdf5_export_source(h5: Any, path: Path) -> None:
    """Keep direct format adapters behind the same physical read admission."""
    errors: list[str] = []
    if not _preflight_hdf5_storage(h5, errors=errors):
        raise ValueError("source HDF5 is not exportable: " + "; ".join(errors))
    _preflight_hdf5_metadata_storage(h5, path=path)
    _validate_robomimic_demos(
        h5=h5,
        errors=errors,
        decoded_budget=ByteBudget(
            "HDF5 export structure bytes", MAX_DECODED_AGGREGATE_BYTES
        ),
    )
    if errors:
        raise ValueError("source HDF5 is not exportable: " + "; ".join(errors))


def _require_hdf5_contract_matches_manifest(
    h5: Any, declared_demo_io_contract: object
) -> None:
    """Require the ``data`` attribute contract to match the dataset manifest."""
    data_group = h5.get("data")
    if data_group is None:
        return
    physical = _read_hdf5_demo_io_contract_attr(data_group.attrs)
    if physical is not None and declared_demo_io_contract is None:
        raise ValueError(
            "HDF5 certified demo_io_contract disagrees with source manifest"
        )
    if declared_demo_io_contract is not None and physical is None:
        raise ValueError("certified HDF5 source is missing physical demo_io_contract")
    if physical is not None and physical != declared_demo_io_contract:
        raise ValueError(
            "HDF5 demo_io_contract does not match source manifest metadata"
        )


def _demo_step_to_jsonable_row(
    *,
    demo: h5py.Group,
    episode_index: int,
    episode_key: str,
    step_index: int,
    declared_shapes: Mapping[tuple[str, ...], tuple[int, ...]] | None = None,
) -> SrbJsonlRow:
    return build_srb_jsonl_row(
        episode_index=episode_index,
        episode_key=episode_key,
        step_index=step_index,
        obs=_group_step_to_jsonable(demo["obs"], step_index, declared_shapes),
        action=demo["actions"][step_index],
        reward=demo["rewards"][step_index],
        done=bool(demo["dones"][step_index]),
        next_obs=_group_step_to_jsonable(demo["next_obs"], step_index, declared_shapes),
        # HDF5 stores demo-wide success; expose it only on the final row.
        success=(
            step_index == int(demo["actions"].shape[0]) - 1
            and bool(
                _read_hdf5_primitive_attribute(demo.attrs, "success", boolean=True)
            )
        ),
        terminated=(
            bool(demo["terminated"][step_index]) if "terminated" in demo else None
        ),
        truncated=(
            bool(demo["truncated"][step_index]) if "truncated" in demo else None
        ),
    )


def _admit_hdf5_json_conversion(demo: Any, budget: ByteBudget) -> None:
    """Reserve decoded arrays before payload reads and bound per-row temporaries.

    Count aliases per use. Rows convert one at a time, so the Python/JSON
    overhead (scalar and container objects; uint8 uses a tighter scalar bound)
    is bounded per row while only the decoded arrays count toward ``budget``.
    """

    def datasets() -> Iterator[Any]:
        for name in ("actions", "rewards", "dones", "terminated", "truncated"):
            if name in demo:
                yield demo[name]
        for name in ("obs", "next_obs"):
            for _, dataset in _iter_hdf5_observation_datasets(demo[name]):
                yield dataset

    for dataset in datasets():
        shape = tuple(dataset.shape)
        dtype = np.dtype(dataset.dtype)
        if not shape or dtype.kind not in "biuf" or dtype.hasobject:
            raise ValueError(
                f"{dataset.name} has unsupported HDF5 conversion shape/dtype"
            )
        _checked_export_array(shape, dtype, dataset.name)
        row_shape = shape[1:]
        row_bytes = _checked_export_array(row_shape, dtype, dataset.name)
        elements = 1
        containers = 0
        for dimension in row_shape:
            containers += elements
            elements *= dimension
        scalar_bytes = 160 if dtype == np.dtype("uint8") else 256
        conversion_bytes = (
            4 * row_bytes + scalar_bytes * elements + 256 * (containers + 1)
        )
        if conversion_bytes > MAX_ARRAY_BYTES:
            raise ResourceLimitError(
                f"{dataset.name} row conversion bytes",
                limit=MAX_ARRAY_BYTES,
                observed=conversion_bytes,
            )
        budget.consume(shape[0] * row_bytes)


def _hdf5_row_shape(
    dataset: Any,
    path: str,
    declared_shapes: Mapping[tuple[str, ...], tuple[int, ...]] | None,
) -> tuple[tuple[int, ...], bool]:
    """Return a leaf's row shape and whether it keeps the legacy batch axis.

    Older HDF5 files stored ``(T, 1, *shape)``;
    readers drop that axis where the contract declares ``shape``.
    """
    row_shape = tuple(int(dim) for dim in getattr(dataset, "shape", ())[1:])
    declared = (declared_shapes or {}).get(tuple(path.split("/")))
    if declared is not None and has_legacy_batch_axis(row_shape, declared):
        return row_shape[1:], True
    return row_shape, False


def _group_step_to_jsonable(
    group: h5py.Group,
    step_index: int,
    declared_shapes: Mapping[tuple[str, ...], tuple[int, ...]] | None = None,
) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for path, dataset in _iter_hdf5_observation_datasets(group):
        parts = path.split("/")
        target = values
        for part in parts[:-1]:
            child = target.setdefault(part, {})
            if not isinstance(child, dict):
                raise ValueError(f"HDF5 observation path {path!r} collides")
            target = child
        value = dataset[step_index]
        if _hdf5_row_shape(dataset, path, declared_shapes)[1]:
            value = value[0]
        target[parts[-1]] = to_jsonable(value)
    return values


def _demo_action_dim(demo: Any) -> int:
    action_shape = tuple(int(dim) for dim in demo["actions"].shape)
    if len(action_shape) <= 1:
        return 1
    action_dim = 1
    for dim in action_shape[1:]:
        action_dim *= dim
    return action_dim


def _demo_obs_feature_specs(
    demo: Any,
    declared_shapes: Mapping[tuple[str, ...], tuple[int, ...]] | None = None,
) -> dict[str, dict[str, Any]]:
    if "obs" not in demo:
        return {}
    specs: dict[str, dict[str, Any]] = {}
    for obs_key, dataset in _iter_hdf5_observation_datasets(demo["obs"]):
        feature_shape, _ = _hdf5_row_shape(dataset, str(obs_key), declared_shapes)
        specs[str(obs_key)] = {
            "dtype": _lerobot_dtype(getattr(dataset, "dtype", None)),
            "shape": _feature_shape(feature_shape),
            "source_shape": list(feature_shape),
            "source_dtype": str(np.dtype(getattr(dataset, "dtype", "float32"))),
        }
    return specs


def _validate_hdf5_transition_observations(demo: Any, demo_key: str) -> None:
    """Preflight paired nested trees and per-leaf feature shapes."""
    if "obs" not in demo or "next_obs" not in demo:
        raise ValueError(f"certified HDF5 demo {demo_key!r} requires obs and next_obs")
    observations = dict(_iter_hdf5_observation_datasets(demo["obs"]))
    next_observations = dict(_iter_hdf5_observation_datasets(demo["next_obs"]))
    if not observations or set(observations) != set(next_observations):
        raise ValueError(
            f"certified HDF5 demo {demo_key!r} obs and next_obs leaf paths differ"
        )
    for path, observation in observations.items():
        if tuple(observation.shape[1:]) != tuple(next_observations[path].shape[1:]):
            raise ValueError(
                f"certified HDF5 demo {demo_key!r} next_obs/{path} shape "
                f"{tuple(next_observations[path].shape[1:])} does not match "
                f"obs/{path} shape {tuple(observation.shape[1:])}"
            )


def _canonical_obs_feature_specs(
    rows: Iterable[Mapping[str, Any]],
    obs_keys: list[str],
    *,
    include_next: bool = False,
) -> dict[str, dict[str, Any]]:
    specs: dict[str, dict[str, Any]] = {}
    allowed_keys = set(obs_keys)
    budget = ByteBudget("LeRobot export feature bytes", MAX_METADATA_AGGREGATE_BYTES)
    for row in rows:
        obs = row.get("obs")
        if not isinstance(obs, Mapping):
            continue
        observations = [(obs, False)]
        if include_next:
            next_obs = row.get("next_obs")
            if not isinstance(next_obs, Mapping):
                raise ValueError("certified row requires explicit next_obs mapping")
            observations.append((next_obs, True))
        for tree, is_next in observations:
            for path, raw_value in _iter_nested_observation_leaves(tree):
                original_path = "/".join(path)
                if original_path not in allowed_keys and path[0] not in allowed_keys:
                    continue
                obs_key = (
                    encode_observation_path(path) if include_next else original_path
                )
                if obs_key not in specs:
                    shape, dtype = _json_array_declaration(
                        raw_value,
                        resource=("next_observation/" if is_next else "observation/")
                        + original_path,
                    )
                    budget.consume(256 + 4 * len(obs_key) + 4 * len(shape))
                    specs[obs_key] = {
                        "dtype": _lerobot_dtype(dtype),
                        "shape": _feature_shape(shape),
                        "source_shape": list(shape),
                    }
                else:
                    shape, _dtype = _json_array_declaration(
                        raw_value,
                        resource=("next_observation/" if is_next else "observation/")
                        + original_path,
                    )
                    if tuple(shape) != tuple(specs[obs_key]["source_shape"]):
                        raise ValueError(
                            f"LeRobot observation {original_path!r} shape changes "
                            "across rows or between observation endpoints"
                        )
    return specs


def _merge_obs_feature_specs(
    target: dict[str, dict[str, Any]],
    source: dict[str, dict[str, Any]],
    budget: ByteBudget,
) -> None:
    for key, spec in source.items():
        if key not in target:
            budget.consume(256 + 4 * len(key))
            target[key] = spec


def _export_env_name(source_manifest: DatasetManifest) -> str:
    return str(
        _thawed_manifest_metadata(source_manifest).get("env_name")
        or source_manifest.identity.task
    )


def _export_env_kwargs(source_manifest: DatasetManifest) -> dict[str, Any]:
    env_kwargs = _thawed_manifest_metadata(source_manifest).get("env_kwargs")
    if isinstance(env_kwargs, dict):
        return env_kwargs
    inferred = {
        "task": source_manifest.identity.task,
        "embodiment": source_manifest.identity.embodiment,
        "action_mode": source_manifest.identity.action_mode,
    }
    if source_manifest.identity.algo is not None:
        inferred["algo"] = source_manifest.identity.algo
    return inferred


def _require_h5py() -> Any:
    if importlib.util.find_spec("h5py") is None:
        raise RuntimeError(
            "Dataset export from robomimic_hdf5 requires h5py. "
            "Install h5py or use the srb[data] extra."
        )
    return importlib.import_module("h5py")


def _require_pyarrow() -> None:
    if importlib.util.find_spec("pyarrow") is None:
        raise RuntimeError(
            "LeRobot export requires pyarrow for Parquet output. "
            "Install pyarrow or export --format srb_jsonl first."
        )


def _lerobot_episode_row(
    episode_index: int,
    sample_count: int,
    offset: int,
    source_manifest: DatasetManifest,
) -> dict[str, Any]:
    return {
        "episode_index": episode_index,
        "length": sample_count,
        "tasks": [source_manifest.identity.task],
        "data_from": offset,
        "data_to": offset + sample_count,
        "dataset_from_index": offset,
        "dataset_to_index": offset + sample_count,
    }


def _lerobot_frame_row(
    row: Mapping[str, Any],
    *,
    global_index: int,
    fps: float,
    transition_extension: Mapping[str, Any] | None = None,
    action_dim: int | None = None,
    obs_feature_specs: Mapping[str, Mapping[str, Any]] | None = None,
    image_views: Sequence[VisualImageView] = (),
) -> dict[str, Any]:
    action = row["action"]
    if transition_extension is not None and action_dim == 1:
        action = _lerobot_singleton_scalar(action, resource="action")
    elif transition_extension is None and action_dim == 1:
        action = _lerobot_singleton_vector(action, resource="action")
    elif np.ndim(action) > 1:
        # Features declare a flat ``[action_dim]`` vector, matching HDF5 export.
        action = np.asarray(action).reshape(-1).tolist()
    # Features declare float32; integral rows (JSON ``1``) must not keep int.
    if np.asarray(action).dtype.kind in "biuf":
        action = _lerobot_typed_value(
            action,
            np.dtype("float32"),
            singleton=transition_extension is not None and action_dim == 1,
            resource="action",
        )
    reward = row["reward"]
    if np.asarray(reward).dtype.kind in "biuf":
        reward = _lerobot_typed_value(
            reward, np.dtype("float32"), singleton=True, resource="reward"
        )
    frame: dict[str, Any] = {
        "index": global_index,
        "episode_index": row["episode_index"],
        "frame_index": row["step_index"],
        "timestamp": np.float32(row["step_index"] / fps),
        "task_index": 0,
        "action": action,
        "reward": reward,
        "done": row["done"],
        "terminated": _row_terminated(row),
        "truncated": _row_truncated(row),
        "success": bool(row.get("success", False)),
    }
    if transition_extension is None:
        for key, value in _flatten_observation_leaves(row["obs"]):
            feature_spec = (obs_feature_specs or {}).get(key, {})
            if (
                feature_spec.get("dtype") == "float32"
                and np.asarray(value).dtype.kind == "f"
            ):
                value = _lerobot_typed_value(
                    value,
                    np.dtype("float32"),
                    singleton=feature_spec.get("shape") == [1],
                    resource=f"observation.{key}",
                )
            frame[f"observation.{key}"] = value
    else:
        paths = transition_extension["observation_paths"]
        next_obs = row.get("next_obs")
        if not isinstance(next_obs, Mapping):
            raise ValueError("certified LeRobot row requires explicit next_obs")
        for suffix, raw_components in paths.items():
            feature_spec = (obs_feature_specs or {}).get(suffix, {})
            components = tuple(raw_components)
            observation_value = _nested_observation_value(row["obs"], components)
            next_observation_value = _nested_observation_value(next_obs, components)
            numpy_dtype = feature_spec.get("numpy_dtype")
            if numpy_dtype is not None:
                # Store the declared dtype, not JSON's widened type.
                observation_value = _lerobot_typed_value(
                    observation_value,
                    np.dtype(numpy_dtype),
                    singleton=feature_spec.get("shape") == [1],
                    resource=f"observation.{suffix}",
                )
                next_observation_value = _lerobot_typed_value(
                    next_observation_value,
                    np.dtype(numpy_dtype),
                    singleton=feature_spec.get("shape") == [1],
                    resource=f"next_observation.{suffix}",
                )
            elif feature_spec.get("shape") == [1]:
                observation_value = _lerobot_singleton_scalar(
                    observation_value, resource=f"observation.{suffix}"
                )
                next_observation_value = _lerobot_singleton_scalar(
                    next_observation_value, resource=f"next_observation.{suffix}"
                )
            frame[f"observation.{suffix}"] = observation_value
            frame[f"next_observation.{suffix}"] = next_observation_value
        for view in image_views:
            pixels = view_pixels(
                _nested_observation_value(row["obs"], view.source_path),
                view,
                resource=view.feature,
            )
            frame[view.feature] = {
                "bytes": encode_png(pixels),
                "path": f"{view.feature}/frame_{global_index:09d}.png",
            }
    return frame


def _lerobot_typed_value(
    value: Any, dtype: np.dtype, *, singleton: bool, resource: str
) -> Any:
    """Return a value Arrow stores with exactly ``dtype``.

    Arrow converts only one-dimensional NumPy arrays, so a rank-N array is
    passed as nested lists of typed rows; a singleton becomes a NumPy scalar.
    """
    array = cast_declared(value, dtype, resource=resource)
    if singleton:
        if array.size != 1:
            raise ValueError(
                f"certified LeRobot {resource} must have scalar or singleton shape"
            )
        return array.reshape(())[()]
    return _arrow_nested_rows(array)


def _arrow_nested_rows(array: np.ndarray) -> Any:
    if array.ndim <= 1:
        return array.reshape(-1)
    return [_arrow_nested_rows(child) for child in array]


def _certified_lerobot_image_views(
    contract: Mapping[str, Any] | None,
    transition_extension: Mapping[str, Any] | None,
) -> list[VisualImageView]:
    """Image views whose merged source leaf is a certified observation path."""
    if transition_extension is None:
        return []
    return certified_image_views(contract, transition_extension["observation_paths"])


def _apply_certified_feature_dtypes(
    obs_feature_specs: dict[str, dict[str, Any]],
    transition_extension: Mapping[str, Any],
    contract: Mapping[str, Any] | None,
) -> None:
    """Declare each certified leaf with its contract (or HDF5) dtype."""
    declared = declared_observation_dtypes(contract)
    for suffix, components in transition_extension["observation_paths"].items():
        spec = obs_feature_specs.get(suffix)
        if spec is None:
            continue
        dtype = declared_dtype_for(declared, tuple(components))
        if dtype is None and spec.get("source_dtype") is not None:
            dtype = np.dtype(spec["source_dtype"])
        if dtype is None and spec.get("dtype") in ("float32", "int64", "bool"):
            # Undeclared JSON leaf: JSON carries no precision, so floats keep
            # the float64 they parse to rather than narrowing silently.
            dtype = np.dtype("float64" if spec["dtype"] == "float32" else spec["dtype"])
        if dtype is None or dtype.kind not in "biuf":
            continue
        spec["dtype"] = str(dtype)
        spec["numpy_dtype"] = str(dtype)


def _lerobot_singleton_scalar(value: Any, *, resource: str) -> Any:
    array = np.asarray(value)
    if array.shape not in ((), (1,)):
        raise ValueError(
            f"certified LeRobot {resource} must have scalar or singleton shape"
        )
    return array.item()


def _lerobot_singleton_vector(value: Any, *, resource: str) -> list[Any]:
    """Keep one-dimensional legacy features as vectors with one element."""
    array = np.asarray(value)
    if array.size != 1:
        raise ValueError(f"LeRobot {resource} must contain exactly one value")
    return [array.reshape(-1)[0].item()]


def _require_lerobot_fps(source_manifest: DatasetManifest) -> float:
    value = _thawed_manifest_metadata(source_manifest).get("fps")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("LeRobot v3 export requires a finite positive fps")
    if not math.isfinite(value) or value <= 0:
        raise ValueError("LeRobot v3 export requires a finite positive fps")
    return float(value)


def _row_terminated(row: Mapping[str, Any]) -> bool:
    """Legacy rows treat ``done`` as ``terminated``."""
    if "terminated" in row:
        return bool(row["terminated"])
    return bool(row.get("done", False))


def _row_truncated(row: Mapping[str, Any]) -> bool:
    if "truncated" in row:
        return bool(row["truncated"])
    return False


def _lerobot_features(
    source_manifest: DatasetManifest,
    *,
    action_dim: int | None = None,
    obs_feature_specs: dict[str, dict[str, Any]] | None = None,
    certified: bool = False,
    image_views: Sequence[VisualImageView] = (),
) -> dict[str, dict[str, Any]]:
    source_metadata = _thawed_manifest_metadata(source_manifest)
    if action_dim is None:
        action_dim = source_metadata.get("action_dim")
    if obs_feature_specs is None:
        obs_feature_specs = _metadata_obs_feature_specs(source_manifest)
    features: dict[str, dict[str, Any]] = {
        "action": {"dtype": "float32", "shape": [action_dim]},
        "reward": {"dtype": "float32", "shape": [1]},
        "done": {"dtype": "bool", "shape": [1]},
        "terminated": {"dtype": "bool", "shape": [1]},
        "truncated": {"dtype": "bool", "shape": [1]},
        "success": {"dtype": "bool", "shape": [1]},
        "episode_index": {"dtype": "int64", "shape": [1]},
        "frame_index": {"dtype": "int64", "shape": [1]},
        "index": {"dtype": "int64", "shape": [1]},
        "timestamp": {"dtype": "float32", "shape": [1]},
        "task_index": {"dtype": "int64", "shape": [1]},
    }
    for obs_key, spec in sorted(obs_feature_specs.items()):
        features[f"observation.{obs_key}"] = {
            "dtype": str(spec.get("dtype") or "float32"),
            "shape": spec.get("shape"),
        }
        if certified:
            features[f"next_observation.{obs_key}"] = {
                "dtype": str(spec.get("dtype") or "float32"),
                "shape": spec.get("shape"),
            }
    for view in image_views:
        features[view.feature] = {
            "dtype": "image",
            "shape": view.shape,
            "names": ["height", "width", "channels"],
        }
    return features


def _metadata_obs_feature_specs(
    source_manifest: DatasetManifest,
) -> dict[str, dict[str, Any]]:
    source_metadata = _thawed_manifest_metadata(source_manifest)
    obs_shapes = source_metadata.get("obs_shapes")
    specs: dict[str, dict[str, Any]] = {}
    for obs_key in source_metadata.get("obs_keys", []):
        shape = None
        if isinstance(obs_shapes, dict) and obs_key in obs_shapes:
            shape = obs_shapes[obs_key]
        specs[str(obs_key)] = {"dtype": "float32", "shape": shape}
    return specs


def _thawed_manifest_metadata(source_manifest: DatasetManifest) -> dict[str, Any]:
    metadata = thaw_json_value(dict(source_manifest.metadata))
    if not isinstance(metadata, dict):
        raise TypeError("dataset manifest metadata must be an object")
    return metadata


def _feature_shape(shape: Any) -> list[int]:
    normalized = [int(dim) for dim in tuple(shape)]
    return normalized or [1]


def _lerobot_dtype(dtype: Any) -> str:
    if dtype is None:
        return "float32"
    dtype = np.dtype(dtype)
    if dtype.kind in "fc":
        return "float32"
    if dtype.kind in "iu":
        return "int64"
    if dtype.kind == "b":
        return "bool"
    return str(dtype)
