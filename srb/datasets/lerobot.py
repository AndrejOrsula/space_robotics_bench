"""LeRobot dataset layout validation."""

from __future__ import annotations

import importlib
import importlib.util
import struct
from collections.abc import Iterator, Mapping, Sequence
from contextlib import closing
from decimal import Decimal
from json import JSONDecodeError
from math import isfinite
from numbers import Integral, Real
from pathlib import Path
from typing import Any

from srb.datasets.metadata_validation import validate_metadata_count
from srb.utils.json_types import JsonDecodeBudget
from srb.utils.publication import ArtifactPublication, PublicationError
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_EVENT_BATCH_ROWS,
    MAX_EVENT_ROWS,
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_METADATA_AGGREGATE_BYTES,
    ByteBudget,
    ResourceLimitError,
    require_bounded_int,
    require_dataset_file_size,
)

_MISSING = object()
_INVALID_SHAPE = object()
_SCALAR_BOOKKEEPING_FEATURES = frozenset(
    {
        "reward",
        "done",
        "terminated",
        "truncated",
        "success",
        "episode_index",
        "frame_index",
        "timestamp",
        "task_index",
    }
)
_LEROBOT_DTYPE_ALIASES = {
    "bool": ("bool", None),
    "boolean": ("bool", None),
    "float": ("float", 64),
    "double": ("float", 64),
    "float16": ("float", 16),
    "float32": ("float", 32),
    "float64": ("float", 64),
    "int": ("int", 64),
    "long": ("int", 64),
    "int8": ("int", 8),
    "int16": ("int", 16),
    "int32": ("int", 32),
    "int64": ("int", 64),
    "uint": ("uint", 64),
    "uint8": ("uint", 8),
    "uint16": ("uint", 16),
    "uint32": ("uint", 32),
    "uint64": ("uint", 64),
    "str": ("string", None),
    "string": ("string", None),
    "object": ("object", None),
}
_LEROBOT_DIAGNOSTIC_RESERVE_BYTES = 1024
_LEROBOT_DIAGNOSTIC_ITEM_OVERHEAD = 64
_LEROBOT_DIAGNOSTIC_TEXT_CHARS = 512


class _LerobotDiagnosticReport:
    """Retain bounded, operation-wide diagnostics for physical row checks."""

    def __init__(self, errors: list[str]) -> None:
        self._errors = errors
        self._seen: set[tuple[str, str]] = set()
        self._budget = ByteBudget(
            "LeRobot validation diagnostics",
            MAX_METADATA_AGGREGATE_BYTES,
        )
        self._retention_limit = max(
            0,
            self._budget.limit - _LEROBOT_DIAGNOSTIC_RESERVE_BYTES,
        )
        self._exhausted = False

    def append_once(self, *, key: tuple[str, str], message: str) -> None:
        if self._exhausted:
            return
        normalized_key = (
            _bounded_lerobot_text(
                key[0],
                limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS,
            ),
            _bounded_lerobot_text(
                key[1],
                limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS,
            ),
        )
        if normalized_key in self._seen:
            return
        retained_message = _bounded_lerobot_text(
            message,
            limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS,
        )
        cost = sum(_lerobot_text_cost(part) for part in normalized_key)
        cost += _lerobot_text_cost(retained_message)
        observed = self._budget.consumed + cost
        if observed > self._retention_limit:
            self._exhausted = True
            self._errors.append(
                str(
                    ResourceLimitError(
                        self._budget.resource,
                        limit=self._budget.limit,
                        observed=observed,
                    )
                )
            )
            return
        self._budget.consume(cost)
        self._seen.add(normalized_key)
        self._errors.append(retained_message)


class _LerobotFrameSequence:
    """Validate bounded stream state across frame batches and files."""

    def __init__(self, *, fps: float | None, report: _LerobotDiagnosticReport) -> None:
        self.fps = fps
        self.report = report
        self.next_index = 0
        self.episode_index = 0
        self.frame_index = 0

    def consume(self, batch: Any) -> None:
        names = set(_schema_names(batch.schema))
        required = {"index", "episode_index", "frame_index", "timestamp"}
        missing = required - names
        if missing:
            for name in sorted(missing):
                self.report.append_once(
                    key=("required-frame-column", name),
                    message=f"LeRobot frame table missing required column {name!r}",
                )
            return
        for row in range(batch.num_rows):
            values = {
                name: _lerobot_scalar_value(_batch_value(batch, row, name))
                for name in ("index", "episode_index", "frame_index", "timestamp")
            }
            global_index = _coerce_int_value(values["index"])
            episode = _coerce_int_value(values["episode_index"])
            frame = _coerce_int_value(values["frame_index"])
            if global_index != self.next_index:
                self.report.append_once(
                    key=("frame-sequence", "global-index"),
                    message=(
                        "LeRobot global index must be contiguous from zero; "
                        f"expected {self.next_index}, got {values['index']!r}"
                    ),
                )
            if episode is None or frame is None:
                self.report.append_once(
                    key=("frame-sequence", "invalid-index"),
                    message="LeRobot episode_index and frame_index must be integers",
                )
            else:
                if episode == self.episode_index + 1:
                    self.episode_index += 1
                    self.frame_index = 0
                if episode != self.episode_index:
                    self.report.append_once(
                        key=("frame-sequence", "episode-order"),
                        message=(
                            "LeRobot episode_index rows must be ordered and contiguous; "
                            f"expected {self.episode_index}, got {episode}"
                        ),
                    )
                if frame != self.frame_index:
                    self.report.append_once(
                        key=("frame-sequence", "frame-index"),
                        message=(
                            "LeRobot frame_index must start at zero and increment "
                            f"within each episode; expected {self.frame_index}, got {frame}"
                        ),
                    )
                self.frame_index += 1
            if self.fps is not None:
                expected_time = 0.0
                # Use the row-local index, even if its global index is corrupt.
                if frame is not None:
                    expected_time = struct.unpack(
                        "=f", struct.pack("=f", frame / self.fps)
                    )[0]
                timestamp = values["timestamp"]
                if (
                    isinstance(timestamp, bool)
                    or not isinstance(timestamp, Real)
                    or not isfinite(float(timestamp))
                    or float(timestamp) != expected_time
                ):
                    self.report.append_once(
                        key=("frame-sequence", "timestamp"),
                        message=(
                            "LeRobot timestamp must equal the float32 representation "
                            "of frame_index / fps seconds"
                        ),
                    )
            self.next_index += 1


class _LerobotEpisodeSequence:
    """Validate episode table ordering and half-open global row offsets."""

    def __init__(self, *, report: _LerobotDiagnosticReport) -> None:
        self.report = report
        self.episode_index = 0
        self.offset = 0

    def consume(self, batch: Any) -> None:
        for row in range(batch.num_rows):
            values = {
                name: _lerobot_scalar_value(_batch_value(batch, row, name))
                for name in (
                    "episode_index",
                    "length",
                    "dataset_from_index",
                    "dataset_to_index",
                    "data_from",
                    "data_to",
                )
                if name in _schema_names(batch.schema)
            }
            episode = _coerce_int_value(values.get("episode_index"))
            length = _coerce_int_value(values.get("length"))
            start = _coerce_int_value(values.get("dataset_from_index"))
            end = _coerce_int_value(values.get("dataset_to_index"))
            if episode != self.episode_index:
                self.report.append_once(
                    key=("episode-sequence", "order"),
                    message=(
                        "LeRobot episode_index rows must be unique, ordered, and "
                        f"contiguous from zero; expected {self.episode_index}, got {episode}"
                    ),
                )
            if (
                length is None
                or length <= 0
                or start != self.offset
                or end != start + length
            ):
                self.report.append_once(
                    key=("episode-sequence", "offset"),
                    message=(
                        "LeRobot dataset_from_index/dataset_to_index offsets must "
                        "match the running half-open global frame interval and length"
                    ),
                )
            if "data_from" in values or "data_to" in values:
                alias_start = _coerce_int_value(values.get("data_from"))
                alias_end = _coerce_int_value(values.get("data_to"))
                if alias_start != start or alias_end != end:
                    self.report.append_once(
                        key=("episode-sequence", "alias-offset"),
                        message=(
                            "LeRobot data_from/data_to aliases must agree with "
                            "dataset_from_index/dataset_to_index"
                        ),
                    )
            if length is not None and length > 0:
                self.offset += length
            self.episode_index += 1


def _lerobot_text_cost(value: str) -> int:
    # Four bytes per code point bounds UTF-8 without allocating an encoded copy.
    return len(value) * 4 + _LEROBOT_DIAGNOSTIC_ITEM_OVERHEAD


def _bounded_lerobot_text(value: Any, *, limit: int) -> str:
    text = value if isinstance(value, str) else str(value)
    if len(text) <= limit:
        return text
    return text[:limit] + "...<truncated>"


def _bounded_lerobot_repr(value: Any) -> str:
    if isinstance(value, str):
        return repr(_bounded_lerobot_text(value, limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS))
    if value is None or isinstance(value, (bool, int, float)):
        return repr(value)
    return f"<{type(value).__name__}>"


def _bounded_lerobot_error_detail(error: BaseException) -> str:
    args = getattr(error, "args", ())
    detail = args[0] if args and isinstance(args[0], str) else type(error).__name__
    return _bounded_lerobot_text(
        detail,
        limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS,
    )


def validate_lerobot_contents(
    *,
    metadata: Mapping[str, Any],
    dataset_path: Path,
    errors: list[str],
) -> None:
    """Validate a LeRobot dataset layout against SRB manifest metadata."""

    dataset_root = _lerobot_dataset_root(dataset_path)
    data_dir = dataset_root / "data"
    meta_dir = dataset_root / "meta"

    _require_existing_dir(data_dir, "LeRobot data directory", errors)
    _require_existing_dir(meta_dir, "LeRobot meta directory", errors)

    info_path = meta_dir / "info.json"
    stats_path = meta_dir / "stats.json"
    tasks_path = meta_dir / "tasks.parquet"
    _require_existing_file(info_path, "meta/info.json", errors)
    _require_existing_file(stats_path, "meta/stats.json", errors)
    _require_existing_file(tasks_path, "meta/tasks.parquet", errors)

    _read_json_object(stats_path, "meta/stats.json", errors)

    frame_files = sorted(data_dir.glob("chunk-*/file-*.parquet"))
    if not frame_files:
        errors.append("LeRobot dataset has no frame parquet files under data/chunk-*")
    episode_files = sorted((meta_dir / "episodes").glob("chunk-*/file-*.parquet"))
    if not episode_files:
        errors.append(
            "LeRobot dataset has no episode parquet files under meta/episodes/chunk-*"
        )

    info = _read_json_object(info_path, "meta/info.json", errors)
    if info is None:
        return

    transition_extension, certified = _lerobot_transition_extension(
        metadata=metadata,
        info=info,
        errors=errors,
    )
    if certified and (
        not _is_int_value(info.get("total_tasks")) or info["total_tasks"] < 0
    ):
        errors.append(
            "certified meta/info.json total_tasks must be a non-negative integer"
        )

    fps_value = info.get("fps", _MISSING)
    fps_number = _finite_real_number(fps_value)
    valid_fps = fps_number is not None and fps_number > 0.0
    if not valid_fps:
        errors.append("meta/info.json fps must be a finite positive number")
    metadata_fps = metadata.get("fps", _MISSING)
    if metadata_fps is not _MISSING:
        manifest_fps = _finite_real_number(metadata_fps)
        if (
            manifest_fps is None
            or manifest_fps <= 0.0
            or (valid_fps and manifest_fps != fps_number)
        ):
            errors.append("meta/info.json fps does not match valid manifest fps")

    _validate_lerobot_action_schema(
        info=info,
        metadata=metadata,
        errors=errors,
    )

    features = info.get("features")
    if not isinstance(features, dict) or not features:
        errors.append("meta/info.json features must be a non-empty object")
    elif "action" not in features:
        errors.append("meta/info.json features missing required key 'action'")
    else:
        # Validate image views separately from numeric and transition columns.
        image_views = _lerobot_image_views(
            info=info, features=features, certified=certified, errors=errors
        )
        numeric_features = {
            name: feature
            for name, feature in features.items()
            if name not in image_views
        }
        _validate_lerobot_contract_identity(
            info=info,
            metadata=metadata,
            transition_extension=transition_extension,
            errors=errors,
        )
        if "srb_demo_io_contract" in info and transition_extension is not None:
            _validate_lerobot_contract_dtypes(
                contract=info["srb_demo_io_contract"],
                features=numeric_features,
                transition_extension=transition_extension,
                errors=errors,
            )
        _validate_lerobot_features(
            features=numeric_features,
            metadata=metadata,
            transition_extension=transition_extension,
            certified=certified,
            errors=errors,
        )
        _validate_lerobot_parquet_tables(
            dataset_root=dataset_root,
            frame_files=frame_files,
            episode_files=episode_files,
            info=info,
            metadata=metadata,
            features=numeric_features,
            transition_extension=transition_extension,
            certified=certified,
            fps=fps_number if valid_fps else None,
            errors=errors,
            ignored_columns=frozenset(image_views),
            exact_dtypes="srb_demo_io_contract" in info,
        )
        if image_views:
            _validate_lerobot_image_view_pixels(
                dataset_root=dataset_root,
                frame_files=frame_files,
                views=image_views,
                errors=errors,
            )

    _validate_info_count(
        info=info,
        info_key="total_episodes",
        metadata=metadata,
        metadata_key="num_episodes",
        errors=errors,
    )
    _validate_info_count(
        info=info,
        info_key="total_frames",
        metadata=metadata,
        metadata_key="total_samples",
        errors=errors,
    )


_LEROBOT_IMAGE_VIEW_CHANNELS = {"rgb_u8": 3, "depth_linear_u8": 1}


def _validate_lerobot_contract_identity(
    *,
    info: Mapping[str, Any],
    metadata: Mapping[str, Any],
    transition_extension: Mapping[str, Any] | None,
    errors: list[str],
) -> None:
    """Match ``info.json`` contract and image views to the manifest."""
    from srb.datasets.visual_contract import (
        certified_image_views,
        describe_image_views,
    )
    from srb.utils.json_types import thaw_json_value

    manifest_contract = metadata.get("demo_io_contract")
    info_contract = info.get("srb_demo_io_contract")
    if manifest_contract is None:
        if info_contract is not None:
            errors.append(
                "meta/info.json srb_demo_io_contract is not declared by the manifest"
            )
        return
    manifest_contract = thaw_json_value(manifest_contract)
    if info_contract != manifest_contract:
        errors.append(
            "meta/info.json srb_demo_io_contract does not match the manifest "
            "demo_io_contract"
        )
        return
    paths = (
        transition_extension.get("observation_paths")
        if isinstance(transition_extension, Mapping)
        else None
    )
    if not isinstance(paths, Mapping) or not all(
        isinstance(components, list)
        and all(isinstance(part, str) for part in components)
        for components in paths.values()
    ):
        return
    expected = describe_image_views(
        certified_image_views(manifest_contract, paths), paths
    )
    if info.get("srb_visual_views", {}) != expected:
        errors.append(
            "meta/info.json srb_visual_views does not match the image views the "
            "manifest demo_io_contract implies"
        )


def _validate_lerobot_contract_dtypes(
    *,
    contract: Any,
    features: Mapping[str, Any],
    transition_extension: Mapping[str, Any],
    errors: list[str],
) -> None:
    """Tie declared observation dtypes and shapes to the I/O contract.

    Accept the legacy ``[1, *shape]`` representation.
    """
    from srb.datasets.visual_contract import (
        contract_shape_error,
        declared_dtype_for,
        declared_observation_dtypes,
        declared_observation_shapes,
        declared_observation_terms,
    )

    if not isinstance(contract, Mapping):
        errors.append("meta/info.json srb_demo_io_contract must be an object")
        return
    declared = declared_observation_dtypes(contract)
    shapes = declared_observation_shapes(contract)
    paths = transition_extension.get("observation_paths")
    if not isinstance(paths, Mapping):
        return
    available_paths = {
        tuple(str(part) for part in components)
        for components in paths.values()
        if isinstance(components, (list, tuple))
    }
    for aliases, _dtype, _shape in declared_observation_terms(contract):
        if not any(alias in available_paths for alias in aliases):
            rendered = " or ".join("/".join(path) for path in aliases)
            errors.append(
                "certified LeRobot data is missing declared observation term; "
                f"expected one of {rendered} in observation and next_observation"
            )
    for suffix, components in paths.items():
        if not isinstance(components, (list, tuple)):
            continue
        path = tuple(str(part) for part in components)
        dtype = declared_dtype_for(declared, path)
        shape = shapes.get(path)
        for prefix in ("observation.", "next_observation."):
            name = f"{prefix}{suffix}"
            feature = features.get(name)
            if not isinstance(feature, Mapping):
                continue
            feature_shape = _normalize_lerobot_shape(feature.get("shape"))
            if shape is not None and feature_shape is not None:
                # LeRobot stores a scalar term as shape [1].
                error = contract_shape_error(
                    f"LeRobot feature {_bounded_lerobot_repr(name)}",
                    tuple(feature_shape),
                    shape or (1,),
                )
                if error is not None:
                    errors.append(error)
            if dtype is None:
                continue
            if _normalize_lerobot_dtype(feature.get("dtype")) != (
                _normalize_lerobot_dtype(str(dtype))
            ):
                errors.append(
                    f"LeRobot feature {_bounded_lerobot_repr(name)} declares dtype "
                    f"{_bounded_lerobot_repr(feature.get('dtype'))}, but "
                    f"srb_demo_io_contract declares {dtype}"
                )


def _lerobot_image_views(
    *,
    info: Mapping[str, Any],
    features: Mapping[str, Any],
    certified: bool,
    errors: list[str],
) -> dict[str, dict[str, Any]]:
    """Validate image features against ``srb_visual_views``.

    Every image feature needs a certified merged-source view whose shape holds
    its channel slice.
    """
    raw_views = info.get("srb_visual_views", {})
    image_features = {
        name
        for name, feature in features.items()
        if isinstance(feature, Mapping) and feature.get("dtype") == "image"
    }
    if not isinstance(raw_views, Mapping):
        errors.append("meta/info.json srb_visual_views must be an object")
        return {}
    views: dict[str, dict[str, Any]] = {}
    for name in sorted(image_features - set(raw_views)):
        errors.append(
            f"meta/info.json image feature {_bounded_lerobot_repr(name)} has no "
            "srb_visual_views entry"
        )
    for name, view in raw_views.items():
        label = _bounded_lerobot_repr(name)
        if name not in image_features:
            errors.append(f"srb_visual_views entry {label} is not an image feature")
            continue
        if not certified:
            errors.append(f"image view {label} requires a certified LeRobot export")
            continue
        if not isinstance(view, Mapping):
            errors.append(f"srb_visual_views entry {label} must be an object")
            continue
        encoding = view.get("encoding")
        channels = view.get("channels")
        source = view.get("source_feature")
        count = _LEROBOT_IMAGE_VIEW_CHANNELS.get(str(encoding))
        if (
            count is None
            or not isinstance(channels, list)
            or len(channels) != 2
            or not all(_is_int_value(value) for value in channels)
            or channels[1] - channels[0] != count
            or channels[0] < 0
        ):
            errors.append(
                f"srb_visual_views entry {label} needs an rgb_u8 (3) or "
                "depth_linear_u8 (1) channel slice"
            )
            continue
        if encoding == "depth_linear_u8":
            depth_range = view.get("depth_range_m")
            if (
                not isinstance(depth_range, list)
                or len(depth_range) != 2
                or any(_finite_real_number(value) is None for value in depth_range)
                or not depth_range[0] < depth_range[1]
            ):
                errors.append(f"depth view {label} needs depth_range_m [near, far]")
                continue
        feature_shape = _normalize_lerobot_shape(features[name].get("shape"))
        source_feature = features.get(source) if isinstance(source, str) else None
        source_shape = (
            _normalize_lerobot_shape(source_feature.get("shape"))
            if isinstance(source_feature, Mapping)
            else None
        )
        if source_shape is None or not str(source).startswith("observation."):
            errors.append(f"image view {label} names no observation source feature")
            continue
        image_shape = source_shape[1:] if source_shape[:1] == [1] else source_shape
        if (
            len(image_shape) != 3
            or image_shape[2] < channels[1]
            or feature_shape != [image_shape[0], image_shape[1], count]
        ):
            errors.append(
                f"image view {label} shape does not match its source channels"
            )
            continue
        views[str(name)] = {
            "source_feature": str(source),
            "source_shape": source_shape,
            "start": int(channels[0]),
            "end": int(channels[1]),
            "shape": feature_shape,
        }
    return views


def _arrow_fixed_nested_numpy(array: Any, row_shape: Sequence[int]) -> Any:
    """Convert a column of fixed-shape nested lists to one NumPy array.

    Flatten in Arrow to avoid Python objects per pixel channel; the final
    reshape rejects null or ragged rows.
    """
    import pyarrow as pa

    values = array
    for _ in row_shape:
        if not (
            pa.types.is_list(values.type)
            or pa.types.is_large_list(values.type)
            or pa.types.is_fixed_size_list(values.type)
        ):
            raise ValueError("source column is not a nested list of the declared rank")
        values = values.flatten()
    if array.null_count:
        raise ValueError("source column has null rows")
    flat = values.to_numpy(zero_copy_only=False)
    return flat.reshape((len(array), *row_shape))


def _validate_lerobot_image_view_pixels(
    *,
    dataset_root: Path,
    frame_files: list[Path],
    views: Mapping[str, Mapping[str, Any]],
    errors: list[str],
) -> None:
    """Decode every image view and compare it with its merged source pixels."""
    import numpy as np
    from PIL import Image

    from srb.datasets.visual_contract import decode_png

    parquet = _parquet_reader(errors)
    if parquet is None:
        return
    report = _LerobotDiagnosticReport(errors)
    budget = ByteBudget("LeRobot image view bytes", MAX_DECODED_AGGREGATE_BYTES)
    sources = sorted({str(view["source_feature"]) for view in views.values()})
    columns = sorted(set(views) | set(sources))
    for frame_file in frame_files:
        reader = _open_bounded_parquet(
            parquet=parquet, path=frame_file, dataset_root=dataset_root, errors=errors
        )
        if reader is None:
            continue
        with closing(reader):
            names = set(_schema_names(reader.schema_arrow))
            missing = [name for name in columns if name not in names]
            for name in missing:
                report.append_once(
                    key=(name, "image-view-column"),
                    message=(
                        f"LeRobot frame table {_dataset_relpath(dataset_root, frame_file)} "
                        f"missing image view column {_bounded_lerobot_repr(name)}"
                    ),
                )
            if missing:
                continue
            for batch in _iter_bounded_parquet_batches(reader, budget, columns=columns):
                merged_sources: dict[str, Any] = {}
                for source in sources:
                    shape = next(
                        view["source_shape"]
                        for view in views.values()
                        if view["source_feature"] == source
                    )
                    try:
                        merged_sources[source] = _arrow_fixed_nested_numpy(
                            batch.column(batch.schema.get_field_index(source)), shape
                        )
                    except (TypeError, ValueError) as exc:
                        report.append_once(
                            key=(source, "image-view-source"),
                            message=(
                                f"LeRobot image view source "
                                f"{_bounded_lerobot_repr(source)} is not a "
                                f"{shape} array column: "
                                f"{_bounded_lerobot_error_detail(exc)}"
                            ),
                        )
                for name, view in views.items():
                    height, width, count = view["shape"]
                    merged_rows = merged_sources.get(str(view["source_feature"]))
                    if merged_rows is None:
                        continue
                    cells = batch.column(batch.schema.get_field_index(name)).to_pylist()
                    for row_index, (cell, source) in enumerate(
                        zip(cells, merged_rows, strict=True)
                    ):
                        label = _bounded_lerobot_repr(name)
                        try:
                            data = cell["bytes"] if isinstance(cell, Mapping) else None
                            if (
                                not isinstance(data, bytes)
                                or len(data) > MAX_ARRAY_BYTES
                            ):
                                raise ValueError(
                                    "view cell carries no bounded PNG bytes"
                                )
                            pixels = decode_png(data, channels=count)
                            if pixels.shape != (height, width, count):
                                raise ValueError(
                                    f"decoded shape {pixels.shape} != "
                                    f"{(height, width, count)}"
                                )
                            merged = np.asarray(source)
                            if merged.ndim == 4 and merged.shape[0] == 1:
                                merged = merged[0]
                            expected = merged[:, :, view["start"] : view["end"]]
                            if not np.array_equal(pixels, expected):
                                raise ValueError("pixels differ from source channels")
                        except (
                            Image.DecompressionBombError,
                            IndexError,
                            KeyError,
                            OSError,
                            TypeError,
                            ValueError,
                        ) as exc:
                            report.append_once(
                                key=(name, "image-view-pixels"),
                                message=(
                                    f"LeRobot image view {label} row {row_index} is "
                                    f"invalid: {_bounded_lerobot_error_detail(exc)}"
                                ),
                            )
                            break


def _lerobot_transition_extension(
    *,
    metadata: Mapping[str, Any],
    info: Mapping[str, Any],
    errors: list[str],
) -> tuple[dict[str, Any] | None, bool]:
    manifest_marker = metadata.get("srb_transition_extension", _MISSING)
    info_marker = info.get("srb_transition_extension", _MISSING)
    certified = (
        metadata.get("demo_io_contract") is not None
        or manifest_marker is not _MISSING
        or info_marker is not _MISSING
    )
    if not certified:
        return None, False
    if manifest_marker is _MISSING or info_marker is _MISSING:
        errors.append(
            "certified LeRobot data requires matching transition extension markers "
            "in manifest metadata and meta/info.json"
        )
        return None, True
    from srb.datasets.lerobot_transitions import validate_transition_extension

    try:
        manifest_extension = validate_transition_extension(manifest_marker)
        info_extension = validate_transition_extension(info_marker)
    except ValueError as exc:
        errors.append(f"LeRobot transition extension is malformed: {exc}")
        return None, True
    if manifest_extension != info_extension:
        errors.append(
            "LeRobot transition extension markers in manifest metadata and "
            "meta/info.json do not match"
        )
        return None, True
    return manifest_extension, True


def iter_lerobot_steps(
    *,
    metadata: Mapping[str, Any],
    dataset_path: Path,
    max_episodes: int | None = None,
    max_steps: int | None = None,
) -> Iterator[dict[str, Any]]:
    """Yield bounded replay rows from the caller's held LeRobot snapshot."""

    if max_steps is not None and max_steps <= 0:
        return
    if max_episodes is not None and max_episodes <= 0:
        return

    errors: list[str] = []
    validate_lerobot_contents(
        metadata=metadata,
        dataset_path=dataset_path,
        errors=errors,
    )
    if errors:
        raise ValueError("LeRobot replay validation failed: " + "; ".join(errors))

    dataset_root = _lerobot_dataset_root(dataset_path)
    info = _read_json_object(dataset_root / "meta" / "info.json", "meta/info.json", [])
    if info is None:
        raise ValueError("LeRobot replay validation lost meta/info.json")
    features = info.get("features")
    if not isinstance(features, dict):
        raise ValueError("LeRobot replay validation lost the feature declarations")
    action_shape = _normalize_lerobot_shape(features.get("action", {}).get("shape"))
    transition_extension, certified = _lerobot_transition_extension(
        metadata=metadata,
        info=info,
        errors=errors,
    )
    if errors:
        raise ValueError("LeRobot replay validation failed: " + "; ".join(errors))

    parquet = _optional_import("pyarrow.parquet")
    if parquet is None or not hasattr(parquet, "ParquetFile"):
        raise ValueError("LeRobot replay requires pyarrow parquet support")
    frame_files = sorted((dataset_root / "data").glob("chunk-*/file-*.parquet"))
    decoded_budget = ByteBudget(
        "LeRobot replay decoded bytes", MAX_DECODED_AGGREGATE_BYTES
    )
    previous_episode: int | None = None
    selected_episodes = 0
    yielded = 0
    stopped = False

    for frame_file in frame_files:
        if stopped:
            break
        reader_errors: list[str] = []
        reader = _open_bounded_parquet(
            parquet=parquet,
            path=frame_file,
            dataset_root=dataset_root,
            errors=reader_errors,
        )
        if reader is None:
            raise ValueError("LeRobot replay could not reopen validated frame data")
        with closing(reader):
            available = set(_schema_names(reader.schema_arrow))
            columns = _select_lerobot_frame_columns(
                schema=reader.schema_arrow,
                features=features,
                errors=reader_errors,
            )
            columns = sorted(
                set(columns)
                | (
                    available
                    & {
                        "index",
                        "episode_index",
                        "frame_index",
                        "timestamp",
                        "action",
                        "reward",
                        "done",
                        "terminated",
                        "truncated",
                        "success",
                    }
                )
            )
            batches = _iter_bounded_parquet_batches(
                reader,
                decoded_budget,
                columns=columns,
            )
            try:
                for batch in batches:
                    for row_index in range(batch.num_rows):
                        episode = _coerce_int_value(
                            _lerobot_scalar_value(
                                _batch_value(batch, row_index, "episode_index")
                            )
                        )
                        frame_index = _coerce_int_value(
                            _lerobot_scalar_value(
                                _batch_value(batch, row_index, "frame_index")
                            )
                        )
                        if episode is None or frame_index is None:
                            raise ValueError(
                                "validated LeRobot replay row lost its episode/frame index"
                            )
                        if episode != previous_episode:
                            if (
                                max_episodes is not None
                                and selected_episodes >= max_episodes
                            ):
                                stopped = True
                                break
                            selected_episodes += 1
                            previous_episode = episode

                        observation: dict[str, Any] = {}
                        next_observation: dict[str, Any] = {}
                        if certified:
                            assert transition_extension is not None
                            paths = transition_extension["observation_paths"]
                            for suffix, components in paths.items():
                                restore_singleton = metadata["obs_shapes"][suffix] == [
                                    1
                                ]
                                _put_lerobot_observation_leaf(
                                    observation,
                                    components,
                                    _lerobot_scalar_value(
                                        _batch_value(
                                            batch,
                                            row_index,
                                            f"observation.{suffix}",
                                        )
                                    ),
                                    restore_singleton=restore_singleton,
                                )
                                _put_lerobot_observation_leaf(
                                    next_observation,
                                    components,
                                    _lerobot_scalar_value(
                                        _batch_value(
                                            batch,
                                            row_index,
                                            f"next_observation.{suffix}",
                                        )
                                    ),
                                    restore_singleton=restore_singleton,
                                )
                        else:
                            for feature_name in sorted(features):
                                if not feature_name.startswith("observation."):
                                    continue
                                suffix = feature_name.removeprefix("observation.")
                                components = tuple(suffix.split("/"))
                                _put_lerobot_observation_leaf(
                                    observation,
                                    components,
                                    _lerobot_scalar_value(
                                        _batch_value(batch, row_index, feature_name)
                                    ),
                                )

                        record: dict[str, Any] = {
                            "episode_index": episode,
                            "step_index": frame_index,
                            "obs": observation,
                        }
                        action = _batch_value(batch, row_index, "action")
                        if action is not _MISSING:
                            action_value = _lerobot_scalar_value(action)
                            if certified and action_shape == [1]:
                                action_value = [action_value]
                            record["action"] = action_value
                        for name in (
                            "reward",
                            "done",
                            "terminated",
                            "truncated",
                            "success",
                        ):
                            if name not in available:
                                continue
                            value = _batch_value(batch, row_index, name)
                            if value is not _MISSING:
                                record[name] = _primitive_value(value)
                        if certified:
                            record["next_obs"] = next_observation
                        yield record
                        yielded += 1
                        if max_steps is not None and yielded >= max_steps:
                            stopped = True
                            break
                    if stopped:
                        break
            finally:
                batches.close()


def _put_lerobot_observation_leaf(
    tree: dict[str, Any],
    components: tuple[str, ...] | list[str],
    value: Any,
    *,
    restore_singleton: bool = False,
) -> None:
    if not components or any(
        not isinstance(component, str) or not component for component in components
    ):
        raise ValueError("LeRobot observation feature has an invalid path")
    current = tree
    for component in components[:-1]:
        child = current.get(component)
        if child is None:
            child = {}
            current[component] = child
        if not isinstance(child, dict):
            raise ValueError("LeRobot observation paths conflict at a leaf")
        current = child
    leaf = components[-1]
    if leaf in current:
        raise ValueError("LeRobot observation paths contain a duplicate leaf")
    current[leaf] = [value] if restore_singleton else value


def _validate_lerobot_action_schema(
    *,
    info: dict[str, Any],
    metadata: Mapping[str, Any],
    errors: list[str],
) -> None:
    expected_schema = metadata.get("action_schema")
    actual_schema = info.get("srb_action_schema")
    if expected_schema is None:
        if actual_schema is not None:
            errors.append(
                "meta/info.json srb_action_schema present but metadata "
                "action_schema is missing"
            )
        return
    if actual_schema is None:
        errors.append(
            "meta/info.json missing srb_action_schema required by metadata "
            "action_schema"
        )
        return
    if actual_schema != expected_schema:
        errors.append(
            "meta/info.json srb_action_schema does not match metadata action_schema"
        )


def _validate_lerobot_features(
    *,
    features: dict[str, Any],
    metadata: Mapping[str, Any],
    transition_extension: Mapping[str, Any] | None = None,
    certified: bool = False,
    errors: list[str],
) -> None:
    action_dim = metadata.get("action_dim")
    if action_dim is not None and _is_int_value(action_dim):
        _validate_lerobot_action_shape(
            features=features,
            action_dim=int(action_dim),
            errors=errors,
        )

    for feature_name, feature in features.items():
        if isinstance(feature, dict):
            _validate_lerobot_declared_dtype(
                feature_name=feature_name,
                feature=feature,
                errors=errors,
            )

    if certified:
        if transition_extension is not None:
            _validate_lerobot_transition_features(
                features=features,
                metadata=metadata,
                transition_extension=transition_extension,
                errors=errors,
            )
        return

    obs_keys = _metadata_observation_roots(metadata)
    obs_shapes = metadata.get("obs_shapes")
    if obs_shapes is not None and not isinstance(obs_shapes, Mapping):
        errors.append("metadata obs_shapes must be a mapping when provided")
    for obs_key in obs_keys:
        matching_features = _observation_features_for_root(features, obs_key)
        if not matching_features:
            errors.append(
                "meta/info.json features missing required observation root "
                f"{_bounded_lerobot_repr(obs_key)}"
            )
            continue
        if not isinstance(obs_shapes, Mapping):
            continue
        for feature_name in matching_features:
            feature_path = feature_name.removeprefix("observation.")
            raw_shape = obs_shapes.get(feature_path)
            if raw_shape is None and len(matching_features) == 1:
                raw_shape = obs_shapes.get(obs_key)
            expected_shape = _normalize_lerobot_shape(raw_shape)
            if expected_shape is None:
                continue
            _validate_lerobot_feature_shape(
                features=features,
                feature_name=feature_name,
                expected_shape=expected_shape,
                expected_label=(
                    "metadata obs_shapes "
                    f"{_bounded_lerobot_text(feature_path, limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS)}="
                    f"{_bounded_lerobot_shape_text(expected_shape)}"
                ),
                errors=errors,
            )


def _validate_lerobot_transition_features(
    *,
    features: Mapping[str, Any],
    metadata: Mapping[str, Any],
    transition_extension: Mapping[str, Any],
    errors: list[str],
) -> None:
    observation_paths = transition_extension["observation_paths"]
    expected = {
        feature_name
        for suffix in observation_paths
        for feature_name in (f"observation.{suffix}", f"next_observation.{suffix}")
    }
    declared = {
        name
        for name in features
        if name.startswith("observation.") or name.startswith("next_observation.")
    }
    for name in sorted(expected - declared):
        errors.append(f"meta/info.json features missing certified feature {name!r}")
    for name in sorted(declared - expected):
        errors.append(f"meta/info.json features has orphan certified feature {name!r}")

    obs_shapes = metadata.get("obs_shapes")
    if not isinstance(obs_shapes, Mapping):
        errors.append(
            "metadata obs_shapes must be an object for certified LeRobot data"
        )
        obs_shapes = {}
    expected_shape_keys = set(observation_paths)
    for suffix in sorted(expected_shape_keys - set(obs_shapes)):
        errors.append(f"metadata obs_shapes missing certified path {suffix!r}")
    for suffix in sorted(set(obs_shapes) - expected_shape_keys):
        errors.append(f"metadata obs_shapes has extra certified path {suffix!r}")
    for suffix in observation_paths:
        observation_name = f"observation.{suffix}"
        next_name = f"next_observation.{suffix}"
        observation = features.get(observation_name)
        next_observation = features.get(next_name)
        if not isinstance(observation, dict) or not isinstance(next_observation, dict):
            continue
        observation_shape = _normalize_lerobot_shape(observation.get("shape"))
        next_shape = _normalize_lerobot_shape(next_observation.get("shape"))
        if observation_shape is None or next_shape is None:
            errors.append(
                f"certified LeRobot paired features {observation_name!r} and "
                f"{next_name!r} require valid declared shapes"
            )
            continue
        if observation_shape != next_shape:
            errors.append(
                f"certified LeRobot paired features {observation_name!r} and "
                f"{next_name!r} have different shapes"
            )
        if not 1 <= len(observation_shape) <= 5:
            errors.append(
                f"certified LeRobot feature {observation_name!r} rank must be "
                "between one and five"
            )
        if suffix in obs_shapes:
            metadata_shape = _normalize_lerobot_shape(obs_shapes[suffix])
            declared_shape = [1] if metadata_shape in ([], [1]) else metadata_shape
            if (
                metadata_shape is None
                or any(dim <= 0 for dim in metadata_shape)
                or declared_shape != observation_shape
            ):
                errors.append(
                    f"certified LeRobot feature {observation_name!r} shape does not "
                    "match metadata obs_shapes"
                )

    for name in ("done", "terminated", "truncated"):
        feature = features.get(name)
        if not isinstance(feature, dict):
            errors.append(f"certified LeRobot features missing required flag {name!r}")
            continue
        if _normalize_lerobot_dtype(feature.get("dtype")) != ("bool", None):
            errors.append(
                f"certified LeRobot feature {name!r} must declare Boolean dtype"
            )


def _validate_lerobot_action_shape(
    *,
    features: dict[str, Any],
    action_dim: int,
    errors: list[str],
) -> None:
    feature = features.get("action")
    if not isinstance(feature, dict):
        errors.append("meta/info.json feature action must be an object")
        return
    shape = _normalize_lerobot_shape(feature.get("shape"))
    if shape is None:
        return
    elements = 1
    for dimension in shape:
        elements *= dimension
    if elements != action_dim:
        errors.append(
            "meta/info.json feature action shape="
            f"{_bounded_lerobot_shape_text(feature.get('shape'))} does not match "
            f"metadata action_dim={action_dim}"
        )


def _validate_lerobot_declared_dtype(
    *,
    feature_name: str,
    feature: dict[str, Any],
    errors: list[str],
) -> None:
    if "dtype" not in feature or feature.get("dtype") is None:
        return
    declaration = _normalize_lerobot_dtype(feature.get("dtype"))
    if declaration is None or (
        declaration[0] == "object" and not feature_name.startswith("observation.")
    ):
        errors.append(
            "meta/info.json feature "
            f"{_bounded_lerobot_text(feature_name, limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS)} "
            "has unsupported dtype "
            f"{_bounded_lerobot_repr(feature.get('dtype'))}"
        )


def _metadata_observation_roots(metadata: Mapping[str, Any]) -> list[str]:
    raw_keys = metadata.get("obs_keys", [])
    if isinstance(raw_keys, (str, bytes, bytearray)):
        return []
    try:
        return [str(key) for key in raw_keys]
    except TypeError:
        return []


def _observation_features_for_root(
    features: dict[str, Any],
    observation_root: str,
) -> list[str]:
    prefix = f"observation.{observation_root}"
    return sorted(
        feature_name
        for feature_name in features
        if feature_name == prefix or feature_name.startswith(f"{prefix}/")
    )


def _validate_lerobot_feature_shape(
    *,
    features: dict[str, Any],
    feature_name: str,
    expected_shape: list[int],
    expected_label: str,
    errors: list[str],
) -> None:
    feature = features.get(feature_name)
    if not isinstance(feature, dict):
        errors.append(
            "meta/info.json feature "
            f"{_bounded_lerobot_text(feature_name, limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS)} "
            "must be an object"
        )
        return
    actual_shape_raw = feature.get("shape")
    actual_shape = _normalize_lerobot_shape(actual_shape_raw)
    if actual_shape != expected_shape:
        errors.append(
            "meta/info.json feature "
            f"{_bounded_lerobot_text(feature_name, limit=_LEROBOT_DIAGNOSTIC_TEXT_CHARS)} "
            f"shape={_bounded_lerobot_shape_text(actual_shape_raw)} "
            f"does not match {expected_label}"
        )


def _normalize_lerobot_shape(raw_shape: Any) -> list[int] | None:
    if raw_shape is None:
        return None
    if _is_int_value(raw_shape):
        return [int(raw_shape)]
    if not isinstance(raw_shape, (list, tuple)):
        return None
    normalized: list[int] = []
    for dim in raw_shape:
        if not _is_int_value(dim):
            return None
        normalized.append(int(dim))
    return normalized


def _finite_real_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        number = float(value)
    except (OverflowError, TypeError, ValueError):
        return None
    return number if isfinite(number) else None


def _validate_lerobot_parquet_tables(
    *,
    dataset_root: Path,
    frame_files: list[Path],
    episode_files: list[Path],
    info: dict[str, Any],
    metadata: Mapping[str, Any],
    features: dict[str, Any],
    transition_extension: Mapping[str, Any] | None = None,
    certified: bool = False,
    fps: float | None,
    errors: list[str],
    ignored_columns: frozenset[str] = frozenset(),
    exact_dtypes: bool = False,
) -> None:
    parquet = _parquet_reader(errors)
    if parquet is None:
        return
    try:
        total_frame_rows, total_episode_rows, episode_length_total, can_sum = (
            _read_bounded_parquet_counts(
                parquet=parquet,
                dataset_root=dataset_root,
                frame_files=frame_files,
                episode_files=episode_files,
                features=features,
                transition_extension=transition_extension,
                certified=certified,
                fps=fps,
                errors=errors,
                ignored_columns=ignored_columns,
                exact_dtypes=exact_dtypes,
                tasks_path=dataset_root / "meta" / "tasks.parquet",
                expected_task_count=info.get("total_tasks") if certified else None,
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
    validate_metadata_count(
        metadata=metadata,
        key="total_samples",
        actual=total_frame_rows,
        errors=errors,
    )
    _validate_lerobot_table_count(
        label="LeRobot episode rows",
        actual=total_episode_rows,
        expected=info.get("total_episodes"),
        errors=errors,
    )
    validate_metadata_count(
        metadata=metadata,
        key="num_episodes",
        actual=total_episode_rows,
        errors=errors,
    )
    if can_sum and episode_length_total != total_frame_rows:
        errors.append(
            "LeRobot episode length total "
            f"{episode_length_total} does not match frame rows {total_frame_rows}"
        )


def _parquet_reader(errors: list[str]) -> Any | None:
    """Return a metadata/batch Parquet reader after dependency checks."""

    pandas = _optional_import("pandas")
    if pandas is None or not hasattr(pandas, "read_parquet"):
        errors.append(
            "LeRobot physical count validation requires pandas with parquet support"
        )
        return None
    if not hasattr(pandas, "__version__"):
        return pandas
    parquet = _optional_import("pyarrow.parquet")
    if parquet is None or not hasattr(parquet, "ParquetFile"):
        errors.append(
            "LeRobot physical count validation requires pandas with parquet support"
        )
        return None
    return parquet


def _read_bounded_parquet_counts(
    *,
    parquet: Any,
    dataset_root: Path,
    frame_files: list[Path],
    episode_files: list[Path],
    features: dict[str, Any],
    transition_extension: Mapping[str, Any] | None = None,
    certified: bool = False,
    errors: list[str],
    fps: float | None = None,
    tasks_path: Path | None = None,
    expected_task_count: Any = None,
    ignored_columns: frozenset[str] = frozenset(),
    exact_dtypes: bool = False,
) -> tuple[int, int, int, bool]:
    """Inspect Parquet metadata and consume bounded Arrow batches."""

    # Tests use a tiny JSON-writing pandas double; real files use Arrow below.
    pandas = _optional_import("pandas")
    if pandas is not None and not hasattr(pandas, "__version__"):
        return _read_legacy_table_double(
            pandas=pandas,
            dataset_root=dataset_root,
            frame_files=frame_files,
            episode_files=episode_files,
            features=features,
            errors=errors,
        )

    decoded_budget = ByteBudget(
        "LeRobot decoded bytes",
        MAX_DECODED_AGGREGATE_BYTES,
    )
    metadata_budget = ByteBudget(
        "LeRobot declared parquet bytes",
        MAX_DECODED_AGGREGATE_BYTES,
    )
    report = _LerobotDiagnosticReport(errors)
    frame_sequence = _LerobotFrameSequence(fps=fps, report=report)
    total_frame_rows = 0
    for frame_file in frame_files:
        reader = _open_bounded_parquet(
            parquet=parquet,
            path=frame_file,
            dataset_root=dataset_root,
            errors=errors,
        )
        if reader is None:
            continue
        with closing(reader):
            if not _account_parquet_metadata(
                reader=reader,
                path=frame_file,
                dataset_root=dataset_root,
                metadata_budget=metadata_budget,
                errors=errors,
            ):
                continue
            physical_columns = _select_lerobot_frame_columns(
                schema=reader.schema_arrow,
                features=features,
                errors=errors,
            )
            columns = set(physical_columns)
            available_columns = {
                str(name) for name in getattr(reader.schema_arrow, "names", ())
            }
            if certified and transition_extension is not None:
                expected_transition_columns = {
                    feature_name
                    for suffix in transition_extension["observation_paths"]
                    for feature_name in (
                        f"observation.{suffix}",
                        f"next_observation.{suffix}",
                    )
                }
                physical_transition_columns = {
                    name
                    for name in available_columns - ignored_columns
                    if name.startswith("observation.")
                    or name.startswith("next_observation.")
                }
                for name in sorted(
                    expected_transition_columns - physical_transition_columns
                ):
                    errors.append(
                        f"LeRobot frame table missing certified feature column {name!r}"
                    )
                for name in sorted(
                    physical_transition_columns - expected_transition_columns
                ):
                    errors.append(
                        f"LeRobot frame table has orphan certified feature column {name!r}"
                    )
            for feature_name in sorted(features):
                if not _lerobot_feature_is_present(reader.schema_arrow, feature_name):
                    errors.append(
                        "LeRobot frame table "
                        f"{_dataset_relpath(dataset_root, frame_file)} missing "
                        "feature column "
                        f"{_bounded_lerobot_repr(feature_name)}"
                    )
            explicit_flags = {
                name
                for name in ("terminated", "truncated")
                if name in available_columns
            }
            if certified and explicit_flags != {"terminated", "truncated"}:
                report.append_once(
                    key=("certified-termination-columns", "pair"),
                    message=(
                        "certified LeRobot frame tables require both terminated "
                        "and truncated columns"
                    ),
                )
            if explicit_flags and explicit_flags != {"terminated", "truncated"}:
                report.append_once(
                    key=("termination-columns", "pair"),
                    message=(
                        "LeRobot frame table explicit termination columns "
                        "terminated and truncated must be present together"
                    ),
                )
            if explicit_flags and "done" not in available_columns:
                report.append_once(
                    key=("termination-columns", "done"),
                    message=(
                        "LeRobot frame table explicit termination columns require done"
                    ),
                )
            termination_columns = set(explicit_flags)
            if explicit_flags and "done" in available_columns:
                termination_columns.add("done")
            selected_columns = sorted(
                columns
                | termination_columns
                | (
                    set(("index", "episode_index", "frame_index", "timestamp"))
                    & available_columns
                )
            )
            _validate_lerobot_physical_dtypes(
                schema=reader.schema_arrow,
                feature_names=features,
                report=report,
                exact=exact_dtypes,
            )
            for batch in _iter_bounded_parquet_batches(
                reader,
                decoded_budget,
                columns=selected_columns,
            ):
                try:
                    _validate_lerobot_frame_batch(
                        batch=batch,
                        feature_names=features,
                        row_offset=total_frame_rows,
                        explicit_flags=explicit_flags,
                        report=report,
                        sequence=frame_sequence,
                        certified=certified,
                    )
                except Exception as exc:
                    report.append_once(
                        key=("physical-row", type(exc).__name__),
                        message=(
                            "could not validate bounded LeRobot frame rows: "
                            f"{_bounded_lerobot_error_detail(exc)}"
                        ),
                    )
                batch_rows = require_bounded_int(
                    batch.num_rows,
                    resource="LeRobot Parquet batch rows",
                    minimum=0,
                    maximum=MAX_EVENT_BATCH_ROWS,
                )
                total_frame_rows += batch_rows
            if total_frame_rows > MAX_EVENT_ROWS:
                raise ResourceLimitError(
                    "LeRobot Parquet frame rows",
                    limit=MAX_EVENT_ROWS,
                    observed=total_frame_rows,
                )

    total_episode_rows = 0
    episode_length_total = 0
    can_sum_episode_lengths = True
    episode_length_type_valid = True
    episode_sequence = _LerobotEpisodeSequence(report=report)
    for episode_file in episode_files:
        reader = _open_bounded_parquet(
            parquet=parquet,
            path=episode_file,
            dataset_root=dataset_root,
            errors=errors,
        )
        if reader is None:
            can_sum_episode_lengths = False
            continue
        with closing(reader):
            if not _account_parquet_metadata(
                reader=reader,
                path=episode_file,
                dataset_root=dataset_root,
                metadata_budget=metadata_budget,
                errors=errors,
            ):
                can_sum_episode_lengths = False
                continue
            columns = set(reader.schema_arrow.names)
            for column in (
                "episode_index",
                "length",
                "tasks",
                "data_from",
                "data_to",
                "dataset_from_index",
                "dataset_to_index",
            ):
                if column in {"data_from", "data_to"}:
                    continue
                if column not in columns:
                    errors.append(
                        "LeRobot episode table "
                        f"{_dataset_relpath(dataset_root, episode_file)} missing "
                        f"column {column!r}"
                    )
            length_type = None
            invalid_length_type = False
            if "length" in columns:
                length_type = _arrow_schema_field_type(
                    reader.schema_arrow,
                    "length",
                )
                if not _is_primitive_integer_type(length_type):
                    errors.append(
                        "LeRobot episode table "
                        f"{_dataset_relpath(dataset_root, episode_file)} length column "
                        "must use a primitive integer type; "
                        f"got {length_type!s}"
                    )
                    can_sum_episode_lengths = False
                    length_type = None
                    invalid_length_type = True
                    episode_length_type_valid = False

            if invalid_length_type:
                total_episode_rows += require_bounded_int(
                    getattr(reader.metadata, "num_rows", 0),
                    resource="LeRobot Parquet episode rows",
                    minimum=0,
                    maximum=MAX_EVENT_ROWS,
                )
                if total_episode_rows > MAX_EVENT_ROWS:
                    raise ResourceLimitError(
                        "LeRobot Parquet episode rows",
                        limit=MAX_EVENT_ROWS,
                        observed=total_episode_rows,
                    )
                continue

            selected_columns = [
                column
                for column in (
                    "episode_index",
                    "length",
                    "tasks",
                    "data_from",
                    "data_to",
                    "dataset_from_index",
                    "dataset_to_index",
                )
                if column in columns and (column != "length" or length_type is not None)
            ]
            for batch in _iter_bounded_parquet_batches(
                reader,
                decoded_budget,
                columns=selected_columns,
            ):
                batch_rows = require_bounded_int(
                    batch.num_rows,
                    resource="LeRobot Parquet batch rows",
                    minimum=0,
                    maximum=MAX_EVENT_BATCH_ROWS,
                )
                total_episode_rows += batch_rows
                if total_episode_rows > MAX_EVENT_ROWS:
                    raise ResourceLimitError(
                        "LeRobot Parquet episode rows",
                        limit=MAX_EVENT_ROWS,
                        observed=total_episode_rows,
                    )
                episode_sequence.consume(batch)
                if "length" not in selected_columns:
                    can_sum_episode_lengths = False
                    continue
                length_column = batch.column(batch.schema.get_field_index("length"))
                batch_length_total = _sum_primitive_episode_lengths(length_column)
                if batch_length_total is None:
                    can_sum_episode_lengths = False
                    continue
                episode_length_total += batch_length_total
                if episode_length_total > MAX_EVENT_ROWS:
                    raise ResourceLimitError(
                        "LeRobot episode length total",
                        limit=MAX_EVENT_ROWS,
                        observed=episode_length_total,
                    )

    if tasks_path is not None and tasks_path.is_file():
        reader = _open_bounded_parquet(
            parquet=parquet,
            path=tasks_path,
            dataset_root=dataset_root,
            errors=errors,
        )
        if reader is not None:
            with closing(reader):
                if _account_parquet_metadata(
                    reader=reader,
                    path=tasks_path,
                    dataset_root=dataset_root,
                    metadata_budget=metadata_budget,
                    errors=errors,
                ):
                    # Bound and read the provider-owned task table; leave its
                    # columns unconstrained for task-only datasets.
                    task_rows = require_bounded_int(
                        getattr(reader.metadata, "num_rows", 0),
                        resource="LeRobot task rows",
                        minimum=0,
                        maximum=MAX_EVENT_ROWS,
                    )
                    if (
                        expected_task_count is not None
                        and task_rows != expected_task_count
                    ):
                        errors.append(
                            "LeRobot task rows do not match meta/info.json total_tasks"
                        )
                    _consume_parquet_batches(reader, decoded_budget)
    if episode_length_type_valid:
        _validate_lerobot_episode_frame_counts(
            parquet=parquet,
            dataset_root=dataset_root,
            frame_files=frame_files,
            episode_files=episode_files,
            decoded_budget=decoded_budget,
            metadata_budget=metadata_budget,
            report=report,
        )
    return (
        total_frame_rows,
        total_episode_rows,
        episode_length_total,
        can_sum_episode_lengths,
    )


def _validate_lerobot_episode_frame_counts(
    *,
    parquet: Any,
    dataset_root: Path,
    frame_files: list[Path],
    episode_files: list[Path],
    decoded_budget: ByteBudget,
    metadata_budget: ByteBudget,
    report: _LerobotDiagnosticReport,
) -> None:
    """Compare episode lengths to frame ownership with bounded lockstep streams."""

    def values(paths: list[Path], column: str) -> Iterator[Any]:
        for path in paths:
            reader = _open_bounded_parquet(
                parquet=parquet,
                path=path,
                dataset_root=dataset_root,
                errors=report._errors,
            )
            if reader is None:
                continue
            with closing(reader):
                if not _account_parquet_metadata(
                    reader=reader,
                    path=path,
                    dataset_root=dataset_root,
                    metadata_budget=metadata_budget,
                    errors=report._errors,
                ):
                    continue
                if column not in reader.schema_arrow.names:
                    continue
                for batch in _iter_bounded_parquet_batches(
                    reader,
                    decoded_budget,
                    columns=[column],
                ):
                    for row in range(batch.num_rows):
                        yield _lerobot_scalar_value(_batch_value(batch, row, column))

    def records() -> Iterator[tuple[Any, Any]]:
        for path in episode_files:
            reader = _open_bounded_parquet(
                parquet=parquet,
                path=path,
                dataset_root=dataset_root,
                errors=report._errors,
            )
            if reader is None:
                continue
            with closing(reader):
                if not _account_parquet_metadata(
                    reader=reader,
                    path=path,
                    dataset_root=dataset_root,
                    metadata_budget=metadata_budget,
                    errors=report._errors,
                ):
                    continue
                names = set(reader.schema_arrow.names)
                if not {"episode_index", "length"}.issubset(names):
                    continue
                for batch in _iter_bounded_parquet_batches(
                    reader,
                    decoded_budget,
                    columns=["episode_index", "length"],
                ):
                    for row in range(batch.num_rows):
                        yield (
                            _lerobot_scalar_value(
                                _batch_value(batch, row, "episode_index")
                            ),
                            _lerobot_scalar_value(_batch_value(batch, row, "length")),
                        )

    frame_indices = iter(values(frame_files, "episode_index"))
    try:
        for expected_episode, raw_length in records():
            episode = _coerce_int_value(expected_episode)
            length = _coerce_int_value(raw_length)
            if episode is None or length is None or length < 0:
                continue
            length = require_bounded_int(
                length,
                resource="LeRobot episode frame length",
                minimum=0,
                maximum=MAX_EVENT_ROWS,
            )
            for _ in range(length):
                actual = next(frame_indices, _MISSING)
                if _coerce_int_value(actual) != episode:
                    report.append_once(
                        key=("episode-frame-count", "mismatch"),
                        message=(
                            "LeRobot per-episode physical frame count does not match "
                            "the episode table length"
                        ),
                    )
                    if actual is _MISSING:
                        break
        if next(frame_indices, _MISSING) is not _MISSING:
            report.append_once(
                key=("episode-frame-count", "extra-frames"),
                message="LeRobot frame rows remain after all declared episode lengths",
            )
    except (
        AttributeError,
        OSError,
        OverflowError,
        TypeError,
        ValueError,
        ResourceLimitError,
    ) as exc:
        report.append_once(
            key=("episode-frame-count", type(exc).__name__),
            message=f"could not compare bounded per-episode frame counts: {_bounded_lerobot_error_detail(exc)}",
        )


def _read_legacy_table_double(
    *,
    pandas: Any,
    dataset_root: Path,
    frame_files: list[Path],
    episode_files: list[Path],
    features: dict[str, Any],
    errors: list[str],
) -> tuple[int, int, int, bool]:
    """Consume the repository's non-Parquet test double only."""

    total_frame_rows = 0
    for frame_file in frame_files:
        try:
            require_dataset_file_size(frame_file, resource="LeRobot Parquet dataset")
            table = pandas.read_parquet(frame_file)
        except Exception as exc:
            errors.append(f"could not read LeRobot parquet table {frame_file}: {exc}")
            continue
        rows = require_bounded_int(
            _table_len(table) or 0,
            resource="LeRobot frame table rows",
            minimum=0,
            maximum=MAX_EVENT_ROWS,
        )
        total_frame_rows += rows
        if total_frame_rows > MAX_EVENT_ROWS:
            raise ResourceLimitError(
                "LeRobot frame rows",
                limit=MAX_EVENT_ROWS,
                observed=total_frame_rows,
            )
        columns = _table_columns(table) or set()
        for feature_name in sorted(features):
            if feature_name not in columns:
                errors.append(
                    "LeRobot frame table "
                    f"{_dataset_relpath(dataset_root, frame_file)} missing "
                    f"feature column {feature_name!r}"
                )

    total_episode_rows = 0
    episode_length_total = 0
    can_sum_episode_lengths = True
    for episode_file in episode_files:
        try:
            require_dataset_file_size(
                episode_file,
                resource="LeRobot Parquet dataset",
            )
            table = pandas.read_parquet(episode_file)
        except Exception as exc:
            errors.append(f"could not read LeRobot parquet table {episode_file}: {exc}")
            can_sum_episode_lengths = False
            continue
        episode_rows = require_bounded_int(
            _table_len(table) or 0,
            resource="LeRobot episode table rows",
            minimum=0,
            maximum=MAX_EVENT_ROWS,
        )
        total_episode_rows += episode_rows
        if total_episode_rows > MAX_EVENT_ROWS:
            raise ResourceLimitError(
                "LeRobot episode rows",
                limit=MAX_EVENT_ROWS,
                observed=total_episode_rows,
            )
        columns = _table_columns(table) or set()
        for column in ("episode_index", "length", "tasks", "data_from", "data_to"):
            if column not in columns:
                errors.append(
                    "LeRobot episode table "
                    f"{_dataset_relpath(dataset_root, episode_file)} missing "
                    f"column {column!r}"
                )
        values = _table_column_values(table, "length")
        if values is None:
            can_sum_episode_lengths = False
            continue
        for value in values:
            length = _coerce_int_value(value)
            if length is None:
                can_sum_episode_lengths = False
                continue
            episode_length_total += require_bounded_int(
                length,
                resource="LeRobot episode length",
                minimum=0,
                maximum=MAX_EVENT_ROWS,
            )
            if episode_length_total > MAX_EVENT_ROWS:
                raise ResourceLimitError(
                    "LeRobot episode length total",
                    limit=MAX_EVENT_ROWS,
                    observed=episode_length_total,
                )
    return (
        total_frame_rows,
        total_episode_rows,
        episode_length_total,
        can_sum_episode_lengths,
    )


def _open_bounded_parquet(
    *,
    parquet: Any,
    path: Path,
    dataset_root: Path,
    errors: list[str],
) -> Any | None:
    try:
        require_dataset_file_size(path, resource="LeRobot Parquet dataset")
        return parquet.ParquetFile(path)
    except (OSError, OverflowError, TypeError, ValueError, ResourceLimitError) as exc:
        errors.append(
            "could not read LeRobot parquet table "
            f"{_dataset_relpath(dataset_root, path)}: {exc}"
        )
        return None


def _schema_names(schema: Any) -> list[str]:
    names = getattr(schema, "names", None)
    if names is not None:
        return [str(name) for name in names]
    try:
        return [str(field.name) for field in schema]
    except TypeError:
        return []


def _select_lerobot_frame_columns(
    *,
    schema: Any,
    features: dict[str, Any],
    errors: list[str],
) -> list[str]:
    del errors
    selected: set[str] = set()
    for feature_name in features:
        selected.update(_physical_columns_for_feature(schema, feature_name))
    return sorted(selected)


def _physical_columns_for_feature(schema: Any, feature_name: str) -> list[str]:
    names = _schema_names(schema)
    if feature_name in names:
        return [feature_name]

    top_level = _arrow_top_level_field_name(schema, feature_name)
    if (
        top_level is not None
        and _arrow_schema_path_type(schema, feature_name) is not None
    ):
        # Dotted selectors return the top-level struct; retain the declared
        # path for provider spies and row-group accounting.
        return [feature_name]

    return []


def _lerobot_feature_is_present(schema: Any, feature_name: str) -> bool:
    return bool(_physical_columns_for_feature(schema, feature_name))


def _arrow_top_level_field_name(schema: Any, path: str) -> str | None:
    names = _schema_names(schema)
    matching = [name for name in names if path == name or path.startswith(f"{name}.")]
    if matching:
        return max(matching, key=len)
    first = path.partition(".")[0]
    return first if first in names else None


def _validate_lerobot_physical_dtypes(
    *,
    schema: Any,
    feature_names: dict[str, Any],
    report: _LerobotDiagnosticReport,
    exact: bool = False,
) -> None:
    for feature_name, feature in sorted(feature_names.items()):
        if not isinstance(feature, dict) or "dtype" not in feature:
            continue
        declaration = _normalize_lerobot_dtype(feature.get("dtype"))
        if declaration is None:
            continue
        physical_type = _feature_physical_type(schema, feature_name)
        physical_kind = _arrow_primitive_kind(physical_type)
        if physical_kind is None:
            continue
        if physical_kind == "null" and feature_name.startswith("observation."):
            # All-null columns provide no fidelity evidence; check non-null rows.
            continue
        if _dtype_is_compatible_with_arrow(declaration, physical_kind):
            if (
                not exact
                or not feature_name.startswith(("observation.", "next_observation."))
                or _arrow_leaf_matches_declaration(physical_type, declaration)
            ):
                continue
            # Require the declared type, not just a compatible numeric kind.
            report.append_once(
                key=(feature_name, "physical-dtype"),
                message=(
                    "LeRobot feature "
                    f"{_bounded_lerobot_repr(feature_name)} declares dtype "
                    f"{_bounded_lerobot_repr(feature.get('dtype'))}, but stores "
                    f"{_bounded_lerobot_repr(str(_arrow_leaf_type(physical_type)))}"
                ),
            )
            continue
        report.append_once(
            key=(feature_name, "physical-dtype"),
            message=(
                "LeRobot feature "
                f"{_bounded_lerobot_repr(feature_name)} declares dtype "
                f"{_bounded_lerobot_repr(feature.get('dtype'))}, "
                f"incompatible with physical {physical_kind} values"
            ),
        )


def _arrow_leaf_type(dtype: Any) -> Any:
    pyarrow = _optional_import("pyarrow")
    types = getattr(pyarrow, "types", None)
    while types is not None and _arrow_is_list_type(types, dtype):
        dtype = dtype.value_type
    return dtype


def _arrow_leaf_matches_declaration(
    dtype: Any, declaration: tuple[str, int | None]
) -> bool:
    pyarrow = _optional_import("pyarrow")
    types = getattr(pyarrow, "types", None)
    if types is None:
        return True
    leaf = _arrow_leaf_type(dtype)
    kind, width = declaration
    if kind == "bool":
        return bool(types.is_boolean(leaf))
    bit_width = getattr(leaf, "bit_width", None)
    if kind == "float":
        return bool(types.is_floating(leaf)) and bit_width == width
    if kind == "int":
        return bool(types.is_signed_integer(leaf)) and bit_width == width
    if kind == "uint":
        return bool(types.is_unsigned_integer(leaf)) and bit_width == width
    return True


def _feature_physical_type(schema: Any, feature_name: str) -> Any | None:
    if not _lerobot_feature_is_present(schema, feature_name):
        return None
    return _arrow_schema_path_type(schema, feature_name)


def _arrow_primitive_kind(dtype: Any) -> str | None:
    if dtype is None:
        return None
    pyarrow = _optional_import("pyarrow")
    types = getattr(pyarrow, "types", None)
    if types is None:
        return None
    try:
        if types.is_dictionary(dtype):
            return _arrow_primitive_kind(dtype.value_type)
        if _arrow_is_list_type(types, dtype):
            return _arrow_primitive_kind(dtype.value_type)
        if types.is_map(dtype):
            return _arrow_primitive_kind(dtype.item_type)
        if types.is_struct(dtype):
            return "struct"
        if types.is_boolean(dtype):
            return "bool"
        if types.is_integer(dtype):
            return "integer"
        if types.is_floating(dtype) or types.is_decimal(dtype):
            return "float"
        if types.is_string(dtype) or types.is_large_string(dtype):
            return "string"
        if types.is_binary(dtype) or types.is_large_binary(dtype):
            return "binary"
        if types.is_null(dtype):
            return "null"
    except (AttributeError, TypeError, ValueError):
        return None
    return "other"


def _dtype_is_compatible_with_arrow(
    declaration: tuple[str, int | None],
    physical_kind: str,
) -> bool:
    kind, _width = declaration
    if kind in {"int", "uint", "float"}:
        return physical_kind in {"integer", "float"}
    if kind == "bool":
        return physical_kind == "bool"
    if kind == "string":
        return physical_kind == "string"
    if kind == "object":
        return physical_kind == "struct"
    return False


def _validate_lerobot_frame_batch(
    *,
    batch: Any,
    feature_names: dict[str, Any],
    row_offset: int,
    explicit_flags: set[str],
    report: _LerobotDiagnosticReport,
    sequence: _LerobotFrameSequence,
    certified: bool = False,
) -> None:
    rows = require_bounded_int(
        getattr(batch, "num_rows", 0),
        resource="LeRobot Parquet batch rows",
        minimum=0,
        maximum=MAX_EVENT_BATCH_ROWS,
    )
    for feature_name, feature in sorted(feature_names.items()):
        if not isinstance(feature, dict):
            continue
        expected_shape = _normalize_lerobot_shape(feature.get("shape"))
        if (
            feature_name == "index"
            or (certified and feature_name == "action")
            or (
                certified
                and feature_name.startswith(("observation.", "next_observation."))
            )
        ) and expected_shape == [1]:
            # LeRobot v0.4.0 declares global index as logical [1], stored scalar.
            expected_shape = []
        declared_dtype = None
        if "dtype" in feature and feature.get("dtype") is not None:
            declared_dtype = _normalize_lerobot_dtype(feature.get("dtype"))
        if expected_shape is None and declared_dtype is None:
            continue
        for row_index in range(rows):
            value = _batch_value(batch, row_index, feature_name)
            if value is _MISSING:
                continue
            _validate_lerobot_value(
                value=value,
                feature_name=feature_name,
                row_index=row_offset + row_index,
                expected_shape=expected_shape,
                declared_dtype=declared_dtype,
                dtype_raw=feature.get("dtype"),
                report=report,
                allow_null_observation=not certified,
            )

    if explicit_flags:
        _validate_lerobot_termination_batch(
            batch=batch,
            row_offset=row_offset,
            explicit_flags=explicit_flags,
            report=report,
        )
    sequence.consume(batch)


def _batch_value(batch: Any, row_index: int, feature_name: str) -> Any:
    schema = getattr(batch, "schema", None)
    if schema is None:
        return _MISSING
    names = _schema_names(schema)
    if feature_name in names:
        field_name = feature_name
        path_parts: list[str] = []
    else:
        matching = [
            name
            for name in names
            if feature_name == name or feature_name.startswith(f"{name}.")
        ]
        if not matching:
            return _MISSING
        field_name = max(matching, key=len)
        path_parts = feature_name[len(field_name) + 1 :].split(".")
    field_index = schema.get_field_index(field_name)
    if field_index < 0:
        return _MISSING
    # A schema-present column has already passed physical selection.  Decoder
    # failures must reach the bounded batch owner so the result is invalid.
    value = batch.column(field_index)[row_index]
    for part in path_parts:
        if _is_arrow_scalar(value):
            if _arrow_scalar_category(value) != "mapping":
                return _MISSING
            value = value[part]
        else:
            if not isinstance(value, Mapping) or part not in value:
                return _MISSING
            value = value[part]
    return value


def _lerobot_scalar_value(value: Any) -> Any:
    as_py = getattr(value, "as_py", None)
    return as_py() if callable(as_py) else value


def _validate_lerobot_value(
    *,
    value: Any,
    feature_name: str,
    row_index: int,
    expected_shape: list[int] | None,
    declared_dtype: tuple[str, int | None] | None,
    dtype_raw: Any,
    report: _LerobotDiagnosticReport,
    allow_null_observation: bool = True,
) -> None:
    if allow_null_observation and _lerobot_observation_value_is_null(
        feature_name, value
    ):
        return
    object_mode = declared_dtype is not None and declared_dtype[0] == "object"
    actual_shape = _lerobot_value_shape(value, object_mode=object_mode)
    if expected_shape is not None:
        shape_compatible = _lerobot_shapes_compatible(
            feature_name=feature_name,
            expected_shape=expected_shape,
            actual_shape=actual_shape,
        )
        if not shape_compatible:
            if actual_shape is _INVALID_SHAPE:
                detail = "an invalid or ragged shape"
            elif actual_shape is None:
                detail = "a mapping or unknown shape"
            else:
                detail = _bounded_lerobot_shape_text(actual_shape)
            report.append_once(
                key=(feature_name, "shape"),
                message=(
                    "LeRobot feature "
                    f"{_bounded_lerobot_repr(feature_name)} row {row_index} shape "
                    f"{detail} does not "
                    "match declared shape "
                    f"{_bounded_lerobot_shape_text(expected_shape)}"
                ),
            )

    if declared_dtype is None or object_mode:
        return
    for leaf in _lerobot_value_leaves(value):
        dtype_detail = _lerobot_dtype_value_error(leaf, declared_dtype)
        if dtype_detail is None:
            continue
        report.append_once(
            key=(feature_name, "dtype"),
            message=(
                "LeRobot feature "
                f"{_bounded_lerobot_repr(feature_name)} row {row_index} value is "
                "incompatible with "
                f"declared dtype {_bounded_lerobot_repr(dtype_raw)}: {dtype_detail}"
            ),
        )
        break


def _lerobot_shapes_compatible(
    *,
    feature_name: str,
    expected_shape: list[int],
    actual_shape: list[int] | None | object,
) -> bool:
    if actual_shape == expected_shape:
        return True
    if (
        feature_name in _SCALAR_BOOKKEEPING_FEATURES
        and expected_shape == [1]
        and actual_shape == []
    ):
        return True
    if (
        feature_name.startswith("observation.")
        and expected_shape == [1]
        and actual_shape == []
    ):
        return True
    return False


def _is_arrow_scalar(value: Any) -> bool:
    return (
        callable(getattr(value, "as_py", None))
        and hasattr(value, "is_valid")
        and hasattr(value, "type")
    )


def _arrow_scalar_category(value: Any) -> str | None:
    if not _is_arrow_scalar(value):
        return None
    pyarrow = _optional_import("pyarrow")
    types = getattr(pyarrow, "types", None)
    if types is None:
        return None
    dtype = value.type
    if _arrow_is_list_type(types, dtype):
        return "sequence"
    if types.is_struct(dtype) or types.is_map(dtype):
        return "mapping"
    return "primitive"


def _arrow_scalar_children(value: Any) -> Any:
    if not _is_arrow_scalar(value):
        return _MISSING
    if not bool(value.is_valid):
        return ()
    category = _arrow_scalar_category(value)
    if category == "sequence":
        return value.values
    if category == "mapping":
        dtype = value.type
        pyarrow = _optional_import("pyarrow")
        types = getattr(pyarrow, "types", None)
        if types is not None and types.is_map(dtype):
            return value.values
        return iter(value.values())
    return ()


def _lerobot_observation_value_is_null(feature_name: str, value: Any) -> bool:
    if not feature_name.startswith("observation."):
        return False
    if value is None:
        return True
    return _is_arrow_scalar(value) and not bool(value.is_valid)


def _bounded_lerobot_shape_text(value: Any) -> str:
    if not isinstance(value, (list, tuple)):
        return _bounded_lerobot_repr(value)
    displayed = value[:16]
    suffix = ", ..." if len(value) > len(displayed) else ""
    return (
        "[" + ", ".join(str(int(dimension)) for dimension in displayed) + suffix + "]"
    )


def _lerobot_value_shape(
    value: Any,
    depth: int = 0,
    *,
    object_mode: bool = False,
    object_root: bool = True,
) -> list[int] | None | object:
    if value is None:
        return _INVALID_SHAPE
    arrow_children = _arrow_scalar_children(value)
    if arrow_children is not _MISSING:
        if not bool(value.is_valid):
            return _INVALID_SHAPE
        category = _arrow_scalar_category(value)
        if category == "mapping":
            return [1] if object_mode and object_root else ([] if object_mode else None)
        if category == "primitive":
            return []
        if depth >= 64:
            return _INVALID_SHAPE
        child_shape: list[int] | None = None
        count = 0
        for child in arrow_children:
            count += 1
            if count > MAX_EVENT_ROWS:
                return _INVALID_SHAPE
            shape = _lerobot_value_shape(
                child,
                depth + 1,
                object_mode=object_mode,
                object_root=False,
            )
            if shape is _INVALID_SHAPE:
                return _INVALID_SHAPE
            if shape is None:
                return None
            if not isinstance(shape, list):
                return _INVALID_SHAPE
            if child_shape is None:
                child_shape = shape
            elif child_shape != shape:
                return _INVALID_SHAPE
        return [count, *(child_shape or [])]
    if isinstance(value, Mapping):
        return [1] if object_mode and object_root else ([] if object_mode else None)
    if not isinstance(value, (list, tuple)):
        return []
    if depth >= 64:
        return _INVALID_SHAPE
    python_child_shape: list[int] | None = None
    for child in value:
        shape = _lerobot_value_shape(
            child,
            depth + 1,
            object_mode=object_mode,
            object_root=False,
        )
        if shape is _INVALID_SHAPE:
            return _INVALID_SHAPE
        if shape is None:
            return None
        if not isinstance(shape, list):
            return _INVALID_SHAPE
        if python_child_shape is None:
            python_child_shape = shape
        elif python_child_shape != shape:
            return _INVALID_SHAPE
    return [len(value), *(python_child_shape or [])]


def _lerobot_value_leaves(value: Any, depth: int = 0) -> Iterator[Any]:
    if depth >= 64:
        yield _INVALID_SHAPE
        return
    arrow_children = _arrow_scalar_children(value)
    if arrow_children is not _MISSING:
        if not bool(value.is_valid):
            yield None
            return
        if _arrow_scalar_category(value) == "primitive":
            yield value
            return
        for child in arrow_children:
            yield from _lerobot_value_leaves(child, depth + 1)
        return
    if isinstance(value, Mapping):
        for child in value.values():
            yield from _lerobot_value_leaves(child, depth + 1)
        return
    if isinstance(value, (list, tuple)):
        for child in value:
            yield from _lerobot_value_leaves(child, depth + 1)
        return
    yield value


def _primitive_value(value: Any) -> Any:
    if not _is_arrow_scalar(value):
        return value
    if not bool(value.is_valid):
        return None
    return value.as_py()


def _lerobot_dtype_value_error(
    value: Any,
    declaration: tuple[str, int | None],
) -> str | None:
    if _is_arrow_scalar(value):
        if not bool(value.is_valid):
            value = None
        else:
            value = value.as_py()
    kind, width = declaration
    if value is None:
        return "null value"
    if kind == "bool":
        return None if type(value) is bool else "expected a Boolean value"
    if kind == "string":
        return None if type(value) is str else "expected a text value"
    if kind in {"int", "uint"}:
        if isinstance(value, bool):
            return "Boolean value cannot satisfy an integer declaration"
        if isinstance(value, Integral):
            integer = int(value)
        elif isinstance(value, Real):
            if not isfinite(float(value)) or not float(value).is_integer():
                return (
                    "nonintegral floating value cannot satisfy an integer declaration"
                )
            integer = int(float(value))
        elif isinstance(value, Decimal):
            if not value.is_finite() or value != value.to_integral_value():
                return (
                    "nonintegral floating value cannot satisfy an integer declaration"
                )
            integer = int(value)
        else:
            return "expected a numeric value"
        assert width is not None
        if kind == "uint":
            minimum, maximum = 0, (1 << width) - 1
        else:
            minimum, maximum = -(1 << (width - 1)), (1 << (width - 1)) - 1
        if not minimum <= integer <= maximum:
            return f"integer value is outside {kind}{width} range"
        return None
    if kind == "float":
        if isinstance(value, bool):
            return "Boolean value cannot satisfy a numeric declaration"
        if isinstance(value, (Integral, Real, Decimal)):
            return None
        return "expected a numeric value"
    return "unsupported declared dtype"


def _validate_lerobot_termination_batch(
    *,
    batch: Any,
    row_offset: int,
    explicit_flags: set[str],
    report: _LerobotDiagnosticReport,
) -> None:
    for row_index in range(
        require_bounded_int(
            getattr(batch, "num_rows", 0),
            resource="LeRobot Parquet batch rows",
            minimum=0,
            maximum=MAX_EVENT_BATCH_ROWS,
        )
    ):
        values = {
            name: _primitive_value(_batch_value(batch, row_index, name))
            for name in ("done", "terminated", "truncated")
        }
        for name in sorted(explicit_flags):
            value = values[name]
            if type(value) is bool:
                continue
            report.append_once(
                key=(name, "termination-type"),
                message=(
                    "LeRobot frame "
                    f"{name} row {row_offset + row_index} must be a non-null "
                    "Boolean value"
                ),
            )
        if not explicit_flags.issubset(values) or "done" not in values:
            continue
        if not all(type(values[name]) is bool for name in explicit_flags | {"done"}):
            continue
        if values["done"] != (values["terminated"] or values["truncated"]):
            report.append_once(
                key=("termination", "agreement"),
                message=(
                    "LeRobot frame row "
                    f"{row_offset + row_index} done must equal "
                    "(terminated or truncated)"
                ),
            )


def _normalize_lerobot_dtype(raw_dtype: Any) -> tuple[str, int | None] | None:
    if not isinstance(raw_dtype, str):
        return None
    return _LEROBOT_DTYPE_ALIASES.get(raw_dtype.strip().lower())


def _consume_parquet_batches(
    reader: Any,
    decoded_budget: ByteBudget,
    *,
    columns: list[str] | None = None,
) -> int:
    rows = 0
    for batch in _iter_bounded_parquet_batches(
        reader,
        decoded_budget,
        columns=columns,
    ):
        batch_rows = require_bounded_int(
            batch.num_rows,
            resource="LeRobot Parquet batch rows",
            minimum=0,
            maximum=MAX_EVENT_BATCH_ROWS,
        )
        rows += batch_rows
        if rows > MAX_EVENT_ROWS:
            raise ResourceLimitError(
                "LeRobot Parquet rows",
                limit=MAX_EVENT_ROWS,
                observed=rows,
            )
    return rows


def _iter_bounded_parquet_batches(
    reader: Any,
    decoded_budget: ByteBudget,
    *,
    columns: list[str] | None = None,
) -> Any:
    """Yield bounded batches, planning size from row-group metadata first.

    Variable-width columns charge encoded chunk bytes to every possible
    nested value in a row, conservatively covering dictionary and nested data.
    """

    metadata = getattr(reader, "metadata", None)
    schema = getattr(reader, "schema_arrow", None)
    if metadata is None or schema is None:
        raise ValueError(
            "native LeRobot Parquet decoder does not expose row-group metadata "
            "and schema; refusing an unbounded batch read"
        )
    row_groups = require_bounded_int(
        getattr(metadata, "num_row_groups", 0),
        resource="LeRobot Parquet row groups",
        minimum=0,
        maximum=MAX_EVENT_ROWS,
    )
    available_columns = [str(name) for name in getattr(schema, "names", ())]
    if columns is None:
        selected_columns = available_columns
    else:
        selected_columns = [
            str(column)
            for column in columns
            if _arrow_schema_path_type(schema, str(column)) is not None
        ]

    for row_group_index in range(row_groups):
        row_group = metadata.row_group(row_group_index)
        row_count = require_bounded_int(
            getattr(row_group, "num_rows", 0),
            resource=f"LeRobot Parquet row-group {row_group_index} rows",
            minimum=0,
            maximum=MAX_EVENT_ROWS,
        )
        if row_count == 0:
            continue
        total_row_bound, max_array_row_bound = _parquet_row_group_row_bound(
            reader=reader,
            row_group=row_group,
            row_count=row_count,
            selected_columns=selected_columns,
        )
        batch_size = _bounded_parquet_batch_size(
            row_count=row_count,
            total_row_bound=total_row_bound,
            max_array_row_bound=max_array_row_bound,
            decoded_budget=decoded_budget,
        )
        batch_kwargs: dict[str, Any] = {
            "batch_size": batch_size,
            "row_groups": [row_group_index],
            "columns": selected_columns,
            "use_threads": False,
        }
        for batch in _iter_reader_batches(
            reader,
            batch_kwargs,
            decoded_budget=decoded_budget,
            total_row_bound=total_row_bound,
            max_array_row_bound=max_array_row_bound,
            row_count=row_count,
        ):
            _consume_arrow_batch(batch, decoded_budget)
            yield batch


def _iter_reader_batches(
    reader: Any,
    kwargs: dict[str, Any],
    *,
    decoded_budget: ByteBudget,
    total_row_bound: int,
    max_array_row_bound: int,
    row_count: int,
) -> Any:
    """Call the native provider only after its bounded batch plan is ready."""

    iterator = iter(reader.iter_batches(**kwargs))
    rows_read = 0
    batch_size = int(kwargs["batch_size"])
    try:
        while rows_read < row_count:
            remaining_rows = row_count - rows_read
            planned_rows = min(batch_size, remaining_rows)
            _require_next_batch_budget(
                decoded_budget=decoded_budget,
                total_row_bound=total_row_bound,
                max_array_row_bound=max_array_row_bound,
                planned_rows=planned_rows,
            )
            try:
                batch = next(iterator)
            except StopIteration:
                raise ValueError(
                    "native LeRobot Parquet decoder stopped before selected "
                    "row group was complete"
                ) from None
            batch_rows = require_bounded_int(
                batch.num_rows,
                resource="LeRobot Parquet batch rows",
                minimum=0,
                maximum=MAX_EVENT_BATCH_ROWS,
            )
            if batch_rows == 0:
                raise ValueError(
                    "native LeRobot Parquet decoder returned an empty batch "
                    "before selected row group was complete"
                )
            rows_read += batch_rows
            if rows_read > row_count:
                raise ValueError(
                    "native LeRobot Parquet decoder returned more rows than "
                    "the selected row group"
                )
            yield batch
    finally:
        close = getattr(iterator, "close", None)
        if callable(close):
            close()


def _require_next_batch_budget(
    *,
    decoded_budget: ByteBudget,
    total_row_bound: int,
    max_array_row_bound: int,
    planned_rows: int,
) -> None:
    planned_bytes = total_row_bound * planned_rows
    planned_array_bytes = max_array_row_bound * planned_rows
    if planned_array_bytes > MAX_ARRAY_BYTES:
        raise ResourceLimitError(
            "LeRobot Arrow array decoded bytes before native batch read",
            limit=MAX_ARRAY_BYTES,
            observed=planned_array_bytes,
        )
    remaining = decoded_budget.limit - decoded_budget.consumed
    if planned_bytes > remaining:
        raise ResourceLimitError(
            "LeRobot decoded bytes before native batch read",
            limit=decoded_budget.limit,
            observed=decoded_budget.consumed + planned_bytes,
        )


def _parquet_row_group_row_bound(
    *,
    reader: Any,
    row_group: Any,
    row_count: int,
    selected_columns: list[str],
) -> tuple[int, int]:
    """Return a conservative decoded-byte bound for one selected row."""

    if not selected_columns:
        return 0, 0
    schema = reader.schema_arrow
    try:
        schema_fields = tuple(schema)
    except TypeError:
        schema_fields = ()
    field_types = {str(field.name): field.type for field in schema_fields}
    # Arrow schemas are iterable; lightweight wrappers may expose only fields/names.
    if not field_types and hasattr(schema, "field"):
        for name in selected_columns:
            try:
                field_types[name] = schema.field(name).type
            except (KeyError, IndexError, TypeError):
                continue

    selected = set(selected_columns)
    bounds_by_column = {name: 0 for name in selected}
    matched: set[str] = set()
    selected_leaf_paths: set[str] = set()
    column_count = require_bounded_int(
        getattr(row_group, "num_columns", 0),
        resource="LeRobot Parquet row-group columns",
        minimum=0,
        maximum=MAX_EVENT_ROWS,
    )
    for index in range(column_count):
        column = row_group.column(index)
        path = str(getattr(column, "path_in_schema", ""))
        matches = [
            name for name in selected if path == name or path.startswith(f"{name}.")
        ]
        if not matches:
            continue
        # Literal dots and nested separators share metadata paths. Avoid using
        # one physical column's inferred type to authorize another's buffers.
        if path in selected_leaf_paths:
            raise ValueError(
                f"native LeRobot Parquet schema has ambiguous physical leaf path {path!r}"
            )
        selected_leaf_paths.add(path)
        selected_name = max(matches, key=len)
        matched.add(selected_name)
        metadata_bytes = _parquet_column_metadata_bytes(column)
        dtype = field_types.get(selected_name)
        if dtype is None:
            dtype = _arrow_schema_path_type(schema, selected_name)
        type_chain = _arrow_type_chain(
            dtype=dtype,
            path=path,
            selected_name=selected_name,
        )
        arrow_types = getattr(_optional_import("pyarrow"), "types", None)
        if (
            not type_chain
            or arrow_types is None
            or _arrow_is_nested_type(arrow_types, type_chain[-1])
        ):
            raise ValueError(
                "native LeRobot Parquet decoder could not resolve selected "
                f"leaf path {path!r} for feature {selected_name!r}"
            )
        leaf_type = type_chain[-1] if type_chain else None
        fixed_width = _arrow_leaf_fixed_width(leaf_type)
        value_count_raw = getattr(column, "num_values", None)
        if value_count_raw is None:
            raise ValueError(
                "native LeRobot Parquet decoder does not expose leaf value counts"
            )
        value_count = require_bounded_int(
            value_count_raw,
            resource="LeRobot Parquet column values",
            minimum=0,
            maximum=MAX_EVENT_ROWS,
        )
        repeated_depth = _arrow_repeated_depth(dtype)
        occurrence_bound = max(value_count if repeated_depth else 1, 1)
        column_bound = _arrow_recursive_buffer_bound(
            type_chain=type_chain,
            metadata_bytes=metadata_bytes,
            fixed_width=fixed_width,
            occurrence_bound=occurrence_bound,
        )
        bounds_by_column[selected_name] += column_bound

    if matched != selected:
        missing = sorted(selected - matched)
        raise ValueError(
            f"native LeRobot Parquet decoder lacks selected column metadata: {missing}"
        )
    total_bound = sum(bounds_by_column.values())
    return total_bound, max(bounds_by_column.values(), default=0)


def _arrow_type_chain(*, dtype: Any, path: str, selected_name: str) -> list[Any]:
    """Resolve a Parquet leaf path to its Arrow ancestor types."""

    if dtype is None:
        return []
    if path == selected_name:
        suffix = ""
    elif path.startswith(f"{selected_name}."):
        suffix = path[len(selected_name) + 1 :]
    else:
        return []
    parts = suffix.split(".") if suffix else []
    current = dtype
    chain = [current]
    skip_element = False
    map_entry_pending = False
    types = getattr(_optional_import("pyarrow"), "types", None)
    if types is None:
        return chain
    part_index = 0
    while part_index < len(parts):
        part = parts[part_index]
        if current is None:
            return []
        if map_entry_pending:
            map_entry_pending = False
            if not types.is_map(current):
                return []
            if part == "key":
                current = getattr(current, "key_type", None)
            elif part == "value":
                current = getattr(current, "item_type", None)
            else:
                return []
            chain.append(current)
            part_index += 1
            continue
        while current is not None and types.is_dictionary(current):
            current = current.value_type
            chain.append(current)
        if current is None:
            return []
        if skip_element and part == "element":
            skip_element = False
            part_index += 1
            continue
        if _arrow_is_list_type(types, current):
            if part not in {"list", "large_list", "fixed_size_list"}:
                return []
            current = getattr(current, "value_type", None)
            chain.append(current)
            skip_element = True
            part_index += 1
            continue
        if types.is_struct(current):
            remaining = ".".join(parts[part_index:])
            candidates = [
                field
                for field in current
                if remaining == str(field.name)
                or remaining.startswith(f"{field.name}.")
            ]
            if len(candidates) != 1:
                return []
            field = candidates[0]
            field_name = str(field.name)
            current = field.type
            chain.append(current)
            part_index += field_name.count(".") + 1
            continue
        if types.is_map(current):
            if part == "key_value":
                map_entry_pending = True
                part_index += 1
                continue
            if part == "key":
                current = getattr(current, "key_type", None)
                chain.append(current)
                part_index += 1
                continue
            if part == "value":
                current = getattr(current, "item_type", None)
                chain.append(current)
                part_index += 1
                continue
            return []
        return []
    while current is not None and types.is_dictionary(current):
        current = current.value_type
        chain.append(current)
    if current is None or _arrow_is_nested_type(types, current):
        return []
    return chain


def _arrow_dictionary_value_type(dtype: Any) -> Any:
    types = getattr(_optional_import("pyarrow"), "types", None)
    if types is None or dtype is None:
        return dtype
    try:
        while types.is_dictionary(dtype):
            dtype = dtype.value_type
    except (AttributeError, TypeError, ValueError):
        return dtype
    return dtype


def _arrow_is_list_type(types: Any, dtype: Any) -> bool:
    try:
        return bool(
            types.is_list(dtype)
            or types.is_large_list(dtype)
            or types.is_fixed_size_list(dtype)
        )
    except (AttributeError, TypeError, ValueError):
        return False


def _arrow_is_nested_type(types: Any, dtype: Any) -> bool:
    try:
        return bool(
            _arrow_is_list_type(types, dtype)
            or types.is_map(dtype)
            or types.is_struct(dtype)
            or types.is_dictionary(dtype)
        )
    except (AttributeError, TypeError, ValueError):
        return True


def _arrow_leaf_fixed_width(dtype: Any) -> int | None:
    if dtype is None:
        return None
    types = getattr(_optional_import("pyarrow"), "types", None)
    if types is None:
        return None
    dtype = _arrow_dictionary_value_type(dtype)
    try:
        if _arrow_is_list_type(types, dtype):
            return _arrow_leaf_fixed_width(dtype.value_type)
        if types.is_map(dtype):
            return _arrow_leaf_fixed_width(dtype.item_type)
        return _arrow_fixed_width(dtype)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _arrow_repeated_depth(dtype: Any) -> int:
    if dtype is None:
        return 0
    types = getattr(_optional_import("pyarrow"), "types", None)
    if types is None:
        return 0
    dtype = _arrow_dictionary_value_type(dtype)
    try:
        if _arrow_is_list_type(types, dtype):
            return 1 + _arrow_repeated_depth(dtype.value_type)
        if types.is_map(dtype):
            return 1 + _arrow_repeated_depth(dtype.item_type)
        if types.is_struct(dtype):
            return max(
                (_arrow_repeated_depth(field.type) for field in dtype), default=0
            )
    except (AttributeError, TypeError, ValueError):
        return 0
    return 0


def _arrow_recursive_buffer_bound(
    *,
    type_chain: list[Any],
    metadata_bytes: int,
    fixed_width: int | None,
    occurrence_bound: int,
) -> int:
    """Bound a selected leaf and its Arrow ancestor buffers.

    Leaf ``num_values`` is a row-group total, so repeated descendants use it
    as the possible count for one row. Schema shapes do not tighten this bound.
    """

    types = getattr(_optional_import("pyarrow"), "types", None)
    if types is None or not type_chain:
        return MAX_ARRAY_BYTES + 1

    occurrences = max(occurrence_bound, 1)
    item_count = 1
    bound = 0
    dictionary_value_bound = 0
    for dtype in type_chain:
        if dtype is None:
            return MAX_ARRAY_BYTES + 1
        try:
            if types.is_dictionary(dtype):
                index_type = dtype.index_type
                index_width = max(1, int(index_type.bit_width) // 8)
                bound += index_width * item_count
                bound += _arrow_bitmap_bytes(item_count)
                # Before materialization, encoded bytes are the only
                # provider-independent bound for the separate dictionary buffer.
                dictionary_value_bound += metadata_bytes
                continue

            if types.is_fixed_size_list(dtype):
                bound += _arrow_bitmap_bytes(item_count)
                list_size = require_bounded_int(
                    getattr(dtype, "list_size", 0),
                    resource="LeRobot fixed-size-list child count",
                    minimum=0,
                    maximum=MAX_EVENT_ROWS,
                )
                item_count = max(item_count * list_size, occurrences, 1)
                continue

            if types.is_list(dtype) or types.is_large_list(dtype):
                offset_width = 8 if types.is_large_list(dtype) else 4
                bound += offset_width * (item_count + 1)
                bound += _arrow_bitmap_bytes(item_count)
                item_count = max(item_count, occurrences)
                continue

            if types.is_map(dtype):
                bound += 4 * (item_count + 1)
                bound += _arrow_bitmap_bytes(item_count)
                item_count = max(item_count, occurrences)
                continue

            if types.is_struct(dtype):
                bound += _arrow_bitmap_bytes(item_count)
                continue

            if types.is_large_string(dtype) or types.is_large_binary(dtype):
                bound += 8 * (item_count + 1)
                bound += _arrow_bitmap_bytes(item_count)
                bound += metadata_bytes * item_count
                continue

            if types.is_string(dtype) or types.is_binary(dtype):
                bound += 4 * (item_count + 1)
                bound += _arrow_bitmap_bytes(item_count)
                bound += metadata_bytes * item_count
                continue

            if types.is_null(dtype):
                bound += _arrow_bitmap_bytes(item_count)
                continue

            # Fixed-width leaves may have validity bitmaps; unsupported leaves
            # cannot be bounded from encoded bytes alone.
            bound += _arrow_bitmap_bytes(item_count)
            if fixed_width is None:
                return MAX_ARRAY_BYTES + 1
            bound += fixed_width * item_count
        except (AttributeError, TypeError, ValueError, OverflowError):
            return MAX_ARRAY_BYTES + 1
    return bound + dictionary_value_bound


def _arrow_bitmap_bytes(item_count: int) -> int:
    return (max(item_count, 1) + 7) // 8


def _parquet_column_metadata_bytes(column: Any) -> int:
    values = [
        getattr(column, "total_uncompressed_size", None),
        getattr(column, "total_compressed_size", None),
    ]
    present = [value for value in values if value is not None]
    if not present:
        # Reject undeclared variable-width columns before provider allocation.
        return MAX_ARRAY_BYTES + 1
    return max(
        require_bounded_int(
            value,
            resource="LeRobot Parquet column bytes",
            minimum=0,
            maximum=MAX_DECODED_AGGREGATE_BYTES,
        )
        for value in present
    )


def _bounded_parquet_batch_size(
    *,
    row_count: int,
    total_row_bound: int,
    max_array_row_bound: int,
    decoded_budget: ByteBudget,
) -> int:
    if row_count <= 0:
        return 1
    if max_array_row_bound > MAX_ARRAY_BYTES:
        raise ResourceLimitError(
            "LeRobot Parquet single-row decoded bytes",
            limit=MAX_ARRAY_BYTES,
            observed=max_array_row_bound,
        )
    # Empty-column batches need a positive planning unit despite decoding no bytes.
    planning_total_bound = max(total_row_bound, 1)
    planning_array_bound = max(max_array_row_bound, 1)
    remaining = decoded_budget.limit - decoded_budget.consumed
    if remaining < planning_total_bound:
        raise ResourceLimitError(
            "LeRobot decoded bytes",
            limit=decoded_budget.limit,
            observed=decoded_budget.consumed + planning_total_bound,
        )
    return max(
        1,
        min(
            MAX_EVENT_BATCH_ROWS,
            row_count,
            MAX_ARRAY_BYTES // planning_array_bound,
            remaining // planning_total_bound,
        ),
    )


def _arrow_schema_field_type(schema: Any, name: str) -> Any | None:
    try:
        return schema.field(name).type
    except (AttributeError, KeyError, IndexError, TypeError):
        return None


def _arrow_schema_path_type(schema: Any, path: str) -> Any | None:
    """Resolve flat or dotted feature names in an Arrow schema."""

    direct = _arrow_named_field(schema, path)
    if direct is not None:
        return direct.type
    current = schema
    parts = path.split(".")
    part_index = 0
    while part_index < len(parts):
        remaining = ".".join(parts[part_index:])
        try:
            candidates = [
                field
                for field in current
                if remaining == str(field.name)
                or remaining.startswith(f"{field.name}.")
            ]
        except TypeError:
            return None
        if len(candidates) != 1:
            return None
        field = candidates[0]
        current = field.type
        part_index += str(field.name).count(".") + 1
    return current


def _arrow_named_field(container: Any, name: str) -> Any | None:
    try:
        for field in container:
            if str(field.name) == name:
                return field
    except TypeError:
        return None
    return None


def _is_primitive_integer_type(dtype: Any) -> bool:
    pyarrow = _optional_import("pyarrow")
    types = getattr(pyarrow, "types", None)
    return bool(types is not None and types.is_integer(dtype))


def _sum_primitive_episode_lengths(column: Any) -> int | None:
    """Sum a primitive integer Arrow column without creating a Python list."""

    null_count = getattr(column, "null_count", 0)
    has_nulls = bool(
        require_bounded_int(
            null_count,
            resource="LeRobot episode length null count",
            minimum=0,
            maximum=MAX_EVENT_BATCH_ROWS,
        )
    )
    compute = _optional_import("pyarrow.compute")
    if (
        compute is None
        or not hasattr(compute, "min")
        or not hasattr(compute, "max")
        or not hasattr(compute, "sum")
    ):
        raise ValueError(
            "native LeRobot Arrow decoder lacks bounded integer reductions"
        )
    minimum = _arrow_scalar_as_python(compute.min(column))
    maximum = _arrow_scalar_as_python(compute.max(column))
    if minimum is None or maximum is None:
        return None
    require_bounded_int(
        minimum,
        resource="LeRobot episode length",
        minimum=0,
        maximum=MAX_EVENT_ROWS,
    )
    require_bounded_int(
        maximum,
        resource="LeRobot episode length",
        minimum=0,
        maximum=MAX_EVENT_ROWS,
    )
    if has_nulls:
        # Null lengths remain readable but cannot certify a total.
        return None
    total = _arrow_scalar_as_python(compute.sum(column))
    if total is None:
        return None
    return require_bounded_int(
        total,
        resource="LeRobot episode length total",
        minimum=0,
        maximum=MAX_EVENT_ROWS,
    )


def _arrow_scalar_as_python(value: Any) -> Any:
    as_py = getattr(value, "as_py", None)
    return as_py() if callable(as_py) else value


def _arrow_fixed_width(dtype: Any) -> int | None:
    if dtype is None:
        return None
    pyarrow = _optional_import("pyarrow")
    types = getattr(pyarrow, "types", None)
    if types is None:
        return None
    try:
        if types.is_boolean(dtype):
            return 1
        if types.is_integer(dtype) or types.is_floating(dtype):
            return int(dtype.bit_width) // 8
        if types.is_decimal(dtype):
            return int(dtype.bit_width) // 8
        if types.is_date32(dtype) or types.is_time32(dtype):
            return 4
        if (
            types.is_date64(dtype)
            or types.is_time64(dtype)
            or types.is_timestamp(dtype)
            or types.is_duration(dtype)
        ):
            return 8
        if types.is_fixed_size_binary(dtype):
            return int(dtype.byte_width)
        if types.is_fixed_size_list(dtype):
            child_width = _arrow_fixed_width(dtype.value_type)
            if child_width is None:
                return None
            return int(dtype.list_size) * child_width
        if types.is_struct(dtype):
            widths = [_arrow_fixed_width(field.type) for field in dtype]
            if any(width is None for width in widths):
                return None
            return sum(width for width in widths if width is not None)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
    return None


def _account_parquet_metadata(
    *,
    reader: Any,
    path: Path,
    dataset_root: Path,
    metadata_budget: ByteBudget,
    errors: list[str],
) -> bool:
    """Reject obviously oversized Parquet row groups before batch reads."""

    metadata = getattr(reader, "metadata", None)
    if metadata is None:
        return True
    try:
        require_bounded_int(
            getattr(metadata, "num_rows", 0),
            resource=f"LeRobot Parquet rows {_dataset_relpath(dataset_root, path)}",
            minimum=0,
            maximum=MAX_EVENT_ROWS,
        )
        row_groups = require_bounded_int(
            getattr(metadata, "num_row_groups", 0),
            resource=(
                f"LeRobot Parquet row groups {_dataset_relpath(dataset_root, path)}"
            ),
            minimum=0,
            maximum=MAX_EVENT_ROWS,
        )
        for index in range(row_groups):
            row_group = metadata.row_group(index)
            declared_bytes = getattr(row_group, "total_byte_size", None)
            if declared_bytes is None:
                continue
            metadata_budget.consume(
                require_bounded_int(
                    declared_bytes,
                    resource=(
                        "LeRobot Parquet row-group bytes "
                        f"{_dataset_relpath(dataset_root, path)}[{index}]"
                    ),
                    minimum=0,
                    maximum=MAX_DECODED_AGGREGATE_BYTES,
                )
            )
    except (AttributeError, TypeError, ValueError, ResourceLimitError) as exc:
        errors.append(
            "LeRobot Parquet metadata exceeds bounded limits for "
            f"{_dataset_relpath(dataset_root, path)}: {exc}"
        )
        return False
    return True


def _consume_arrow_batch(batch: Any, decoded_budget: ByteBudget) -> None:
    get_total_buffer_size = getattr(batch, "get_total_buffer_size", None)
    if not callable(get_total_buffer_size):
        raise ValueError(
            "native LeRobot Arrow decoder does not expose complete buffer size"
        )
    nbytes = get_total_buffer_size()
    if nbytes is None:
        raise ValueError(
            "native LeRobot Arrow decoder returned no complete buffer size"
        )
    decoded_bytes = require_bounded_int(
        nbytes,
        resource="LeRobot Arrow batch decoded bytes",
        minimum=0,
        maximum=MAX_DECODED_AGGREGATE_BYTES,
    )
    columns = getattr(batch, "columns", None)
    if columns is None:
        raise ValueError(
            "native LeRobot Arrow decoder does not expose per-array buffers"
        )
    for column in columns:
        get_column_buffer_size = getattr(column, "get_total_buffer_size", None)
        if not callable(get_column_buffer_size):
            raise ValueError(
                "native LeRobot Arrow decoder does not expose per-array buffer size"
            )
        column_bytes = require_bounded_int(
            get_column_buffer_size(),
            resource="LeRobot Arrow array decoded bytes",
            minimum=0,
            maximum=MAX_DECODED_AGGREGATE_BYTES,
        )
        if column_bytes > MAX_ARRAY_BYTES:
            raise ResourceLimitError(
                "LeRobot Arrow array decoded bytes",
                limit=MAX_ARRAY_BYTES,
                observed=column_bytes,
            )
    decoded_budget.consume(decoded_bytes)


def _optional_import(module_name: str) -> Any | None:
    try:
        if importlib.util.find_spec(module_name) is None:
            return None
    except (ImportError, ValueError):
        pass
    try:
        return importlib.import_module(module_name)
    except ImportError:
        return None


def _table_columns(table: Any) -> set[str] | None:
    columns = getattr(table, "columns", None)
    if columns is None:
        return None
    return {str(column) for column in columns}


def _table_len(table: Any) -> int | None:
    try:
        return len(table)
    except TypeError:
        return None


def _table_column_values(table: Any, column_name: str) -> list[Any] | None:
    try:
        column = table[column_name]
    except (KeyError, IndexError, TypeError):
        # Treat absent or non-indexable columns as unavailable; surface other
        # errors instead of skipping dependent validation.
        return None
    if hasattr(column, "tolist"):
        values = column.tolist()
    else:
        try:
            values = list(column)
        except TypeError:
            return None
    return list(values)


def _validate_lerobot_table_count(
    *,
    label: str,
    actual: int,
    expected: Any,
    errors: list[str],
) -> None:
    expected_count = _coerce_int_value(expected)
    if expected_count is None:
        return
    if actual != expected_count:
        errors.append(f"{label} {actual} does not match info count {expected_count}")


def _coerce_int_value(value: Any) -> int | None:
    if isinstance(value, bool) or value is None:
        return None
    try:
        coerced = int(value)
    except (TypeError, ValueError):
        return None
    try:
        if value != coerced:
            return None
    except (TypeError, ValueError):
        # Keep the coerced int for incomparable Parquet/JSON scalars; surface
        # other comparison failures as defects.
        pass
    return coerced


def _dataset_relpath(dataset_root: Path, path: Path) -> str:
    try:
        return str(path.relative_to(dataset_root))
    except ValueError:
        return str(path)


def _lerobot_dataset_root(dataset_path: Path) -> Path:
    if dataset_path.name == "data":
        return dataset_path.parent
    return dataset_path


def _require_existing_dir(path: Path, label: str, errors: list[str]) -> None:
    if not path.is_dir():
        errors.append(f"{label} does not exist: {path}")


def _require_existing_file(path: Path, label: str, errors: list[str]) -> None:
    if not path.is_file():
        errors.append(f"{label} does not exist: {path}")


def _read_json_object(
    path: Path,
    label: str,
    errors: list[str],
) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        require_dataset_file_size(
            path,
            resource=f"{label} file",
            max_bytes=MAX_IN_MEMORY_FILE_BYTES,
        )
        source_path = path.expanduser().resolve(strict=False)
        payload = ArtifactPublication.file(source_path).read(
            lambda raw: raw,
            max_bytes=MAX_IN_MEMORY_FILE_BYTES,
        )
        if payload is None:
            raise FileNotFoundError(source_path)
        data = JsonDecodeBudget().loads(
            payload.decode("utf-8"),
            context=label,
            max_bytes=MAX_IN_MEMORY_FILE_BYTES,
        )
    except JSONDecodeError as exc:
        errors.append(f"{label} invalid JSON: {exc.msg}")
        return None
    except (
        OSError,
        PublicationError,
        TypeError,
        ValueError,
        ResourceLimitError,
    ) as exc:
        errors.append(f"{label} rejected: {exc}")
        return None
    if not isinstance(data, dict):
        errors.append(f"{label} must be a JSON object")
        return None
    return data


def _validate_info_count(
    *,
    info: dict[str, Any],
    info_key: str,
    metadata: Mapping[str, Any],
    metadata_key: str,
    errors: list[str],
) -> None:
    if info_key not in info or metadata_key not in metadata:
        return
    actual = info[info_key]
    expected = metadata[metadata_key]
    if not _is_int_value(actual):
        errors.append(f"meta/info.json {info_key} must be an integer")
        return
    if not _is_int_value(expected):
        errors.append(f"metadata {metadata_key} must be an integer")
        return
    if int(actual) != int(expected):
        errors.append(
            f"meta/info.json {info_key}={actual} does not match "
            f"metadata {metadata_key}={expected}"
        )


def _is_int_value(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


__all__ = ["validate_lerobot_contents"]
