"""Owned, bounded admission for native Robomimic training datasets.

Robomimic reopens the path supplied in its configuration.  This adapter gives
it a private verified copy and completes the structural and allocation checks
before the provider is allowed to construct a dataset.
"""

from __future__ import annotations

import importlib
import json
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, cast

import numpy as np

from srb.datasets.hdf5_metadata import (
    Hdf5MetadataCleanupError,
    Hdf5MetadataLimitError,
    Hdf5MetadataLimits,
    Hdf5MetadataTimeoutError,
    read_hdf5_text_attributes,
)
from srb.datasets.manifest import (
    DatasetSnapshot,
    find_dataset_manifest,
    open_dataset_snapshot,
    read_dataset_manifest,
    resolve_dataset_path,
)
from srb.datasets.metadata_validation import normalize_int_value
from srb.datasets.robomimic import (
    _account_hdf5_dataset,
    _hdf5_dataset_identity,
    _hdf5_first_dim,
    _hdf5_object_identity,
    _iter_hdf5_observation_datasets,
    _iter_hdf5_string_values,
    _preflight_hdf5_metadata_storage,
    _preflight_hdf5_storage,
    _read_hdf5_count_attributes,
    _validate_declared_sample_count,
    _validate_hdf5_first_dim,
    _validate_hdf5_mask_declaration,
    _validate_robomimic_demos,
    _validate_robomimic_field_declaration,
    _validate_termination_flag_agreement,
)
from srb.utils.dataset_schema import StorageFormat
from srb.utils.failures import InvalidInputError, attempt_cleanup
from srb.utils.json_types import (
    FrozenJSONObject,
    JsonDecodeBudget,
    JSONValue,
    freeze_json_value,
    thaw_json_value,
    validate_json_object,
)
from srb.utils.publication import (
    MaterializedDirectorySnapshot,
    PublicationError,
    materialize_file_snapshot,
    sha256_file,
)
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_DIRECTORY_ENTRIES,
    MAX_EVENT_ROWS,
    MAX_EXTERNAL_FILE_BYTES,
    MAX_METADATA_AGGREGATE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    ByteBudget,
    ResourceLimitError,
    checked_shape_nbytes,
    require_bounded_int,
    require_dataset_file_size,
)

_KNOWN_DATASET_KEYS = frozenset(
    {"actions", "rewards", "dones", "terminated", "truncated"}
)
_MAX_ERROR_COUNT = 64
_MAX_HDF5_KEY_CHARS = 4096
_MAX_HDF5_KEY_COMPONENTS = 32
_DEFAULT_DATASET_KEYS = ("actions", "rewards", "dones")
_ADMISSION_TIMEOUT_S = 120.0
_HDF5_TEXT_ATTRIBUTES = frozenset(
    {"env_name", "env_kwargs", "env_args", "action_schema"}
)


class _BoundedErrors(list[str]):
    def append(self, value: str) -> None:
        if len(self) < _MAX_ERROR_COUNT:
            super().append(value)


@dataclass(frozen=True, slots=True)
class RobomimicProviderDeclaration:
    """Bounded shape and conversion declarations for one provider setup.

    Full-array totals sum across demos. ``*_row_bytes`` use an eight-byte
    arithmetic width; ``*_source_row_bytes`` retain source dtype widths.
    Row widths are per-demo maxima, allowing callers to account for sequence,
    frame-stack, batch, and goal duplication without retaining per-demo data.
    ``max_demo_multiplicity`` records the most repeated demo ID in a filter;
    unfiltered providers report one. ``mask_decoded_bytes`` includes selected
    mask rows and bounded conversion storage.
    """

    selected_source_bytes: int
    converted_float32_bytes: int
    dataset_source_bytes: int
    dataset_converted_float32_bytes: int
    observation_source_bytes: int
    observation_converted_float32_bytes: int
    next_observation_source_bytes: int
    next_observation_converted_float32_bytes: int
    selected_source_row_bytes: int
    selected_source_row_elements: int
    selected_row_bytes: int
    selected_row_elements: int
    dataset_source_row_bytes: int
    dataset_source_row_elements: int
    dataset_row_bytes: int
    dataset_row_elements: int
    observation_source_row_bytes: int
    observation_source_row_elements: int
    observation_row_bytes: int
    observation_row_elements: int
    next_observation_source_row_bytes: int
    next_observation_source_row_elements: int
    next_observation_row_bytes: int
    next_observation_row_elements: int
    max_source_row_bytes: int
    max_source_row_elements: int
    max_row_bytes: int
    max_row_elements: int
    selected_array_count: int
    mask_rows: int
    max_mask_rows: int
    max_demo_multiplicity: int
    mask_decoded_bytes: int


@dataclass(frozen=True, slots=True)
class RobomimicProviderAdmission:
    """Declared source and retained allocations for one provider setup.

    ``metadata_decoded_bytes`` covers decoded HDF5 attributes and retained
    model-file values. ``model_file_bytes`` reports their Python object sizes
    for separately charging provider cache and worker copies.
    """

    dataset_path: Path
    source_path: Path
    sha256: str
    source_bytes: int
    num_demos: int
    total_samples: int
    declared_decoded_bytes: int
    metadata_decoded_bytes: int
    model_file_bytes: int
    dataset_keys: tuple[str, ...]
    obs_keys: tuple[str, ...] | None
    mask_keys: tuple[str, ...]
    declaration: RobomimicProviderDeclaration


@dataclass(slots=True)
class _OwnedInput:
    source_path: Path
    dataset_path: Path
    sha256: str
    source_bytes: int
    owner: DatasetSnapshot | MaterializedDirectorySnapshot

    def close(self) -> None:
        self.owner.close()

    def defer_finalizer_cleanup(self) -> None:
        self.owner.defer_finalizer_cleanup()


def _attach_cleanup_owner(error: BaseException, owner: object) -> None:
    _, annotation_error = attempt_cleanup(
        lambda: setattr(error, "cleanup_owner", owner)
    )
    if annotation_error is not None and not isinstance(annotation_error, Exception):
        raise annotation_error


def _checked_limit(
    value: int | None,
    *,
    default: int,
    maximum: int,
    resource: str,
) -> int:
    if value is None:
        value = default
    return require_bounded_int(
        value,
        resource=resource,
        minimum=1,
        maximum=maximum,
    )


def _normalize_key_sequence(
    value: Sequence[str] | None,
    *,
    field_name: str,
    default: tuple[str, ...] | None,
) -> tuple[str, ...] | None:
    if value is None:
        return default
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise TypeError(f"{field_name} must be a sequence of strings or None")
    checked: list[str] = []
    if len(value) > MAX_DIRECTORY_ENTRIES:
        raise ResourceLimitError(
            f"{field_name} entries",
            limit=MAX_DIRECTORY_ENTRIES,
            observed=len(value),
        )
    for item in value:
        if not isinstance(item, str):
            raise TypeError(f"{field_name} must contain only strings")
        if (
            not item
            or len(item) > _MAX_HDF5_KEY_CHARS
            or item.startswith("/")
            or any(component in {"", ".", ".."} for component in item.split("/"))
            or len(item.split("/")) > _MAX_HDF5_KEY_COMPONENTS
        ):
            raise ValueError(f"{field_name} contains an invalid HDF5 key: {item!r}")
        if item not in checked:
            checked.append(item)
    return tuple(checked)


def _config_value(config: object, path: tuple[str, ...], default: object) -> object:
    current = config
    for name in path:
        if isinstance(current, Mapping):
            if name not in current:
                return default
            current = current[name]
        else:
            try:
                current = getattr(current, name)
            except AttributeError:
                return default
    return current


def _provider_options(
    config: object | None,
    *,
    dataset_keys: Sequence[str] | None,
    action_keys: Sequence[str] | None,
    obs_keys: Sequence[str] | None,
    load_next_obs: bool | None,
    mask_keys: Sequence[str] | None,
) -> tuple[tuple[str, ...], tuple[str, ...] | None, bool, tuple[str, ...]]:
    if config is not None:
        if action_keys is None:
            configured_actions = _config_value(config, ("train", "action_keys"), ())
            action_keys = cast(Sequence[str], configured_actions)
        if dataset_keys is None:
            configured = _config_value(
                config, ("train", "dataset_keys"), _DEFAULT_DATASET_KEYS
            )
            dataset_keys = cast(Sequence[str], configured)
        if obs_keys is None:
            configured_obs = _config_value(config, ("all_obs_keys",), None)
            if configured_obs is None:
                configured_obs = _config_value(
                    config, ("observation", "all_obs_keys"), None
                )
            obs_keys = cast(Sequence[str] | None, configured_obs)
        if load_next_obs is None:
            configured_next = _config_value(
                config, ("train", "hdf5_load_next_obs"), True
            )
            load_next_obs = (
                configured_next if isinstance(configured_next, bool) else True
            )
        if mask_keys is None:
            configured_masks: list[str] = []
            for path in (
                ("train", "hdf5_filter_key"),
                ("train", "hdf5_validation_filter_key"),
            ):
                candidate = _config_value(config, path, None)
                if candidate is not None:
                    if not isinstance(candidate, str):
                        raise TypeError(f"{path[-1]} must be a string or None")
                    configured_masks.append(candidate)
            mask_keys = configured_masks
    if load_next_obs is None:
        load_next_obs = True
    if not isinstance(load_next_obs, bool):
        raise TypeError("load_next_obs must be a boolean")
    checked_dataset_keys = _normalize_key_sequence(
        dataset_keys,
        field_name="dataset_keys",
        default=_DEFAULT_DATASET_KEYS,
    )
    assert checked_dataset_keys is not None
    checked_action_keys = _normalize_key_sequence(
        action_keys,
        field_name="action_keys",
        default=(),
    )
    assert checked_action_keys is not None
    checked_dataset_keys = tuple(
        dict.fromkeys((*checked_dataset_keys, *checked_action_keys))
    )
    checked_obs_keys = _normalize_key_sequence(
        obs_keys,
        field_name="obs_keys",
        default=None,
    )
    checked_mask_keys = _normalize_key_sequence(
        mask_keys,
        field_name="mask_keys",
        default=(),
    )
    assert checked_mask_keys is not None
    return checked_dataset_keys, checked_obs_keys, load_next_obs, checked_mask_keys


def _raise_errors(errors: _BoundedErrors, *, path: Path) -> None:
    if errors:
        suffix = ""
        if len(errors) == _MAX_ERROR_COUNT:
            suffix = f" (+additional errors omitted after {_MAX_ERROR_COUNT})"
        raise ValueError(
            f"robomimic training dataset rejected {path}: " + "; ".join(errors) + suffix
        )


def _parse_env_meta(
    raw: str | bytes | None,
    *,
    path: Path,
    json_budget: JsonDecodeBudget | None = None,
) -> FrozenJSONObject:
    if raw is None:
        raise ValueError(f"robomimic training dataset {path} is missing data/env_args")
    if isinstance(raw, bytes):
        try:
            encoded = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError(
                f"robomimic training dataset {path} has invalid UTF-8 env_args"
            ) from exc
    elif isinstance(raw, str):
        encoded = raw
    else:  # pragma: no cover - metadata worker only returns text or bytes.
        raise TypeError("robomimic env_args must be text")
    try:
        decoder = JsonDecodeBudget() if json_budget is None else json_budget
        decoded = decoder.loads(encoded, context="robomimic env_args")
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError(
            f"robomimic training dataset {path} has invalid JSON env_args"
        ) from exc
    try:
        validated = validate_json_object(decoded, context="robomimic env_args")
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"robomimic training dataset {path} has invalid JSON env_args"
        ) from exc
    if not isinstance(validated.get("env_name"), str):
        raise ValueError("robomimic env_args.env_name must be a string")
    env_type = validated.get("type")
    if isinstance(env_type, bool) or not isinstance(env_type, int):
        raise ValueError("robomimic env_args.type must be an integer")
    if not isinstance(validated.get("env_kwargs"), dict):
        raise ValueError("robomimic env_args.env_kwargs must be an object")
    frozen = freeze_json_value(cast(JSONValue, validated))
    if not isinstance(frozen, Mapping):  # pragma: no cover - validated object.
        raise TypeError("robomimic env_args must be an object")
    return frozen


def _read_model_files(
    *,
    data_group: Any,
    demo_keys: Sequence[str],
    source_path: Path,
    sha256: str,
    max_model_file_bytes: int,
    metadata_budget: ByteBudget,
    deadline_at: float,
) -> tuple[Mapping[str, str | bytes], int]:
    """Read provider model paths through the bounded text worker."""

    values: dict[str, str | bytes] = {}
    model_file_bytes = 0
    for demo_key in demo_keys:
        remaining_timeout = deadline_at - time.monotonic()
        if remaining_timeout <= 0:
            raise Hdf5MetadataTimeoutError(
                "Robomimic training metadata admission deadline expired"
            )
        demo = data_group[demo_key]
        attrs = getattr(demo, "attrs", None)
        if attrs is None or "model_file" not in attrs:
            continue
        remaining = metadata_budget.limit - metadata_budget.consumed
        if remaining <= 0:
            raise ResourceLimitError(
                "Robomimic model_file metadata bytes",
                limit=metadata_budget.limit,
                observed=metadata_budget.consumed + 1,
            )
        limits = Hdf5MetadataLimits(
            max_value_bytes=min(max_model_file_bytes, remaining),
            max_aggregate_bytes=remaining,
            timeout_s=min(_ADMISSION_TIMEOUT_S, remaining_timeout),
        )
        try:
            result = read_hdf5_text_attributes(
                source_path,
                names=("model_file",),
                object_path=f"data/{demo_key}",
                expected_sha256=sha256,
                limits=limits,
            )
        except Hdf5MetadataLimitError as exc:
            raise ResourceLimitError(
                "Robomimic model_file metadata bytes",
                limit=max_model_file_bytes,
                observed=exc.observed,
            ) from exc
        value = result.get("model_file")
        if value is None:
            raise ValueError(
                f"robomimic training dataset {source_path} model_file disappeared"
            )
        # ``SequenceDataset`` retains this attribute in its in-memory cache.
        # Charge the object representation rather than only its wire bytes so
        # non-ASCII Unicode values cannot evade the retained-text allowance.
        retained_size = sys.getsizeof(value)
        metadata_budget.consume(retained_size)
        model_file_bytes += retained_size
        values[demo_key] = value
    if deadline_at - time.monotonic() <= 0:
        raise Hdf5MetadataTimeoutError(
            "Robomimic training metadata admission deadline expired"
        )
    return MappingProxyType(values), model_file_bytes


def _validate_optional_masks(
    h5: Any,
    *,
    demo_keys: set[str],
    mask_keys: Sequence[str],
    budget: ByteBudget,
    errors: _BoundedErrors,
    deadline_at: float,
) -> tuple[dict[str, int], int]:
    deadline_check = _admission_deadline_check(deadline_at)
    deadline_check()
    if not mask_keys:
        # Without a filter every admitted demo occurs once.
        return {}, 1
    mask_group = h5.get("mask")
    if mask_group is None or not hasattr(mask_group, "keys"):
        errors.append("robomimic HDF5 missing required group 'mask'")
        return {}, 0
    counts: dict[str, int] = {}
    max_demo_multiplicity = 0
    seen_datasets: set[int] = set()
    approved_datasets: set[int] = set()
    for mask_key in mask_keys:
        deadline_check()
        path = f"mask/{mask_key}"
        if mask_key not in mask_group:
            errors.append(f"mask missing required dataset {mask_key!r}")
            continue
        dataset = mask_group[mask_key]
        if not _validate_hdf5_mask_declaration(
            dataset=dataset,
            path=path,
            errors=errors,
        ):
            continue
        if not _account_hdf5_dataset(
            dataset=dataset,
            path=path,
            decoded_budget=budget,
            errors=errors,
            seen_datasets=seen_datasets,
            approved_datasets=approved_datasets,
        ):
            continue
        count = 0
        occurrences: dict[str, int] = {}
        for key in _iter_hdf5_string_values(
            dataset,
            errors=errors,
            path=path,
            conversion_budget=budget,
            raw_budget_accounted=True,
            deadline_check=deadline_check,
        ):
            count += 1
            if key not in demo_keys:
                errors.append(f"{path} references unknown demo {key!r}")
                continue
            previous = occurrences.get(key)
            if previous is None:
                if len(occurrences) >= MAX_DIRECTORY_ENTRIES:
                    raise ResourceLimitError(
                        f"{path} distinct demo IDs",
                        limit=MAX_DIRECTORY_ENTRIES,
                        observed=len(occurrences) + 1,
                    )
                previous = 0
            occurrence = previous + 1
            occurrences[key] = occurrence
            max_demo_multiplicity = max(max_demo_multiplicity, occurrence)
        if count == 0:
            # Robomimic rejects an empty split only after training starts, so
            # name the requested filter key here.
            errors.append(
                f"{path} selects no demos; record more demos or change "
                "train_split. To train without validation, set both "
                "agent.experiment.validate=false and "
                "agent.train.hdf5_validation_filter_key=null"
            )
        counts[mask_key] = count
    return counts, max_demo_multiplicity


def _validate_extra_dataset_keys(
    data_group: Any,
    *,
    demo_keys: Sequence[str],
    dataset_keys: Sequence[str],
    budget: ByteBudget,
    errors: _BoundedErrors,
    allow_missing: bool,
    validate_payload: bool = True,
) -> None:
    for dataset_key in dataset_keys:
        if dataset_key in _KNOWN_DATASET_KEYS:
            if not allow_missing:
                for demo_key in demo_keys:
                    demo = data_group[demo_key]
                    if hasattr(demo, "get") and dataset_key not in demo:
                        errors.append(f"data/{demo_key}/{dataset_key} is missing")
            continue
        for demo_key in demo_keys:
            demo = data_group[demo_key]
            if not hasattr(demo, "get"):
                continue
            path = f"data/{demo_key}/{dataset_key}"
            if dataset_key not in demo:
                if allow_missing:
                    continue
                errors.append(f"{path} is missing")
                continue
            if not validate_payload:
                continue
            dataset = demo[dataset_key]
            if hasattr(dataset, "keys") and not hasattr(dataset, "shape"):
                errors.append(f"{path} must be an HDF5 dataset")
                continue
            if not _account_hdf5_dataset(
                dataset=dataset,
                path=path,
                decoded_budget=budget,
                errors=errors,
                seen_datasets=set(),
                approved_datasets=set(),
            ):
                continue
            sample_count = _hdf5_first_dim(demo.get("actions"))
            if sample_count is not None:
                _validate_hdf5_first_dim(
                    dataset=dataset,
                    path=path,
                    expected=sample_count,
                    errors=errors,
                )


def _validate_provider_dataset(
    *,
    dataset: Any,
    path: str,
    sample_count: int,
    scalar_row: bool,
    require_boolean: bool,
    budget: ByteBudget,
    errors: _BoundedErrors,
    seen_datasets: set[int],
    approved_datasets: set[int],
) -> None:
    if not _validate_robomimic_field_declaration(
        dataset=dataset,
        path=path,
        require_sample_axis=True,
        scalar_row=scalar_row,
        require_boolean=require_boolean,
        errors=errors,
    ):
        return
    if _account_hdf5_dataset(
        dataset=dataset,
        path=path,
        decoded_budget=budget,
        errors=errors,
        seen_datasets=seen_datasets,
        approved_datasets=approved_datasets,
    ):
        actual = _hdf5_first_dim(dataset)
        if actual is None:
            errors.append(f"{path} must have a leading sample dimension")
        elif actual != sample_count:
            errors.append(
                f"{path} length {actual} does not match the demo's declared "
                f"num_samples={sample_count}"
            )


def _provider_declared_samples(
    demo: Any,
    *,
    demo_path: str,
    errors: _BoundedErrors,
) -> int | None:
    """Read bounded num_samples metadata used to size the provider index."""

    attrs = getattr(demo, "attrs", None)
    if attrs is None or "num_samples" not in attrs:
        # ``_admit_hdf5`` already reported the missing attribute for this demo.
        return None
    try:
        declared = normalize_int_value(attrs["num_samples"])
    except (AttributeError, KeyError, OSError, TypeError, ValueError):
        declared = None
    if declared is None:
        errors.append(f"{demo_path} num_samples must be a scalar integer")
        return None
    try:
        return require_bounded_int(
            declared,
            resource=f"{demo_path} num_samples",
            minimum=0,
            maximum=MAX_EVENT_ROWS,
        )
    except (TypeError, ValueError) as exc:
        errors.append(f"{demo_path} num_samples is out of bounds: {exc}")
        return None


def _admission_deadline_check(deadline_at: float) -> Callable[[], None]:
    def check() -> None:
        if time.monotonic() >= deadline_at:
            raise Hdf5MetadataTimeoutError(
                "Robomimic training admission deadline expired"
            )

    return check


def _provider_flag_agreement(
    demo: Any,
    *,
    demo_path: str,
    sample_count: int,
    approved_datasets: set[int],
    errors: _BoundedErrors,
    deadline_at: float,
) -> None:
    """Check episode-flag agreement under the scratch-byte budget."""

    names = ("dones", "terminated", "truncated")
    if not all(name in demo for name in names):
        return
    checked = set(approved_datasets)
    scratch_errors: list[str] = []
    for name in names:
        dataset = demo[name]
        if _hdf5_dataset_identity(dataset) in checked:
            continue
        # Unselected flags reject only through the agreement result itself;
        # their payloads are not part of the provider's decoded declaration.
        if not _validate_robomimic_field_declaration(
            dataset=dataset,
            path=f"{demo_path}/{name}",
            require_sample_axis=False,
            scalar_row=True,
            require_boolean=name != "dones",
            errors=scratch_errors,
        ):
            return
        if not _account_hdf5_dataset(
            dataset=dataset,
            path=f"{demo_path}/{name}",
            decoded_budget=ByteBudget(
                "Robomimic termination flag bytes", MAX_ARRAY_BYTES
            ),
            errors=scratch_errors,
            seen_datasets=set(),
            approved_datasets=checked,
        ):
            return
        if _hdf5_first_dim(dataset) != sample_count:
            return
    _validate_termination_flag_agreement(
        demo=demo,
        demo_path=demo_path,
        errors=errors,
        approved_datasets=checked,
        deadline_check=_admission_deadline_check(deadline_at),
    )


def _provider_observation_leaves(
    group: Any,
    *,
    prefix: str,
    selected_keys: Sequence[str] | None,
    sample_count: int,
    budget: ByteBudget,
    errors: _BoundedErrors,
    seen_datasets: set[int],
    approved_datasets: set[int],
) -> dict[str, Any]:
    if selected_keys is None:
        leaves = dict(
            _iter_hdf5_observation_datasets(
                group,
            )
        )
    else:
        leaves = {}
        for key in selected_keys:
            dataset = group.get(key)
            path = f"{prefix}/{key}"
            if dataset is None:
                errors.append(f"{path} is missing")
                continue
            leaves[key] = dataset
    for key, dataset in leaves.items():
        _validate_provider_dataset(
            dataset=dataset,
            path=f"{prefix}/{key}",
            sample_count=sample_count,
            scalar_row=False,
            require_boolean=False,
            budget=budget,
            errors=errors,
            seen_datasets=seen_datasets,
            approved_datasets=approved_datasets,
        )
    return leaves


def _validate_provider_observations(
    *,
    demo: Any,
    demo_path: str,
    obs_keys: Sequence[str] | None,
    load_next_obs: bool,
    sample_count: int,
    budget: ByteBudget,
    errors: _BoundedErrors,
) -> None:
    # An explicit empty observation selection is valid for dataset-only native
    # providers. ``None`` means retain the provider's automatic all-leaf scan.
    if obs_keys is not None and not obs_keys:
        return
    obs_group = demo.get("obs")
    if obs_group is None or not hasattr(obs_group, "keys"):
        errors.append(f"{demo_path} requires an HDF5 obs group")
        return
    seen_datasets: set[int] = set()
    approved_datasets: set[int] = set()
    obs_leaves = _provider_observation_leaves(
        obs_group,
        prefix=f"{demo_path}/obs",
        selected_keys=obs_keys,
        sample_count=sample_count,
        budget=budget,
        errors=errors,
        seen_datasets=seen_datasets,
        approved_datasets=approved_datasets,
    )
    if not obs_leaves:
        errors.append(f"{demo_path}/obs has no observation datasets")
    if not load_next_obs:
        return
    next_obs_group = demo.get("next_obs")
    if next_obs_group is None or not hasattr(next_obs_group, "keys"):
        errors.append(f"{demo_path} requires an HDF5 next_obs group")
        return
    next_obs_leaves = _provider_observation_leaves(
        next_obs_group,
        prefix=f"{demo_path}/next_obs",
        selected_keys=obs_keys,
        sample_count=sample_count,
        budget=budget,
        errors=errors,
        seen_datasets=seen_datasets,
        approved_datasets=approved_datasets,
    )
    if set(obs_leaves) != set(next_obs_leaves):
        errors.append(
            f"{demo_path}/obs keys do not match next_obs keys: "
            f"obs={sorted(obs_leaves)} next_obs={sorted(next_obs_leaves)}"
        )


def _validate_configured_provider_demos(
    *,
    h5: Any,
    demo_keys: Sequence[str],
    dataset_keys: Sequence[str],
    obs_keys: Sequence[str] | None,
    load_next_obs: bool,
    errors: _BoundedErrors,
    decoded_budget: ByteBudget,
    deadline_at: float,
) -> tuple[int, int]:
    """Validate selected native fields, including nested action paths."""

    data_group = h5.get("data")
    if data_group is None or not hasattr(data_group, "keys"):
        errors.append("robomimic HDF5 missing required group 'data'")
        return 0, 0
    total_samples = 0
    for demo_key in demo_keys:
        demo_path = f"data/{demo_key}"
        demo = data_group[demo_key]
        if not hasattr(demo, "keys"):
            errors.append(f"{demo_path} must be an HDF5 group")
            continue
        # Robomimic sizes and indexes demos from ``num_samples``. Check selected
        # per-step datasets and ``actions`` against it before training.
        sample_count = _provider_declared_samples(
            demo, demo_path=demo_path, errors=errors
        )
        if sample_count is None:
            continue
        # A selected ``actions`` is compared by the dataset pass below.
        if "actions" in demo and "actions" not in dataset_keys:
            _validate_declared_sample_count(
                demo=demo,
                demo_path=demo_path,
                sample_count=_hdf5_first_dim(demo["actions"]),
                errors=errors,
            )
        total_samples += sample_count
        seen_datasets: set[int] = set()
        approved_datasets: set[int] = set()
        for dataset_key in dataset_keys:
            path = f"{demo_path}/{dataset_key}"
            dataset = demo.get(dataset_key)
            # SequenceDataset synthesizes absent dataset keys as float32
            # ``(num_samples, 1)`` arrays when caching is enabled. Provider
            # admission accounts that allocation in its declaration pass.
            if dataset is None:
                continue
            _validate_provider_dataset(
                dataset=dataset,
                path=path,
                sample_count=sample_count,
                scalar_row=dataset_key
                in {"rewards", "dones", "terminated", "truncated"},
                require_boolean=dataset_key in {"terminated", "truncated"},
                budget=decoded_budget,
                errors=errors,
                seen_datasets=seen_datasets,
                approved_datasets=approved_datasets,
            )
        _provider_flag_agreement(
            demo,
            demo_path=demo_path,
            sample_count=sample_count,
            approved_datasets=approved_datasets,
            errors=errors,
            deadline_at=deadline_at,
        )
        _validate_provider_observations(
            demo=demo,
            demo_path=demo_path,
            obs_keys=obs_keys,
            load_next_obs=load_next_obs,
            sample_count=sample_count,
            budget=decoded_budget,
            errors=errors,
        )
    return len(demo_keys), total_samples


def _provider_array_metrics(
    dataset: Any,
    *,
    path: str,
    errors: _BoundedErrors,
) -> tuple[int, int, int, int, int] | None:
    """Declare source and float32 sizes without reading the payload."""

    shape_value = getattr(dataset, "shape", None)
    dtype_value = getattr(dataset, "dtype", None)
    if shape_value is None:
        errors.append(f"{path} has no provider shape declaration")
        return None
    try:
        shape = tuple(int(dimension) for dimension in shape_value)
        dtype = np.dtype(dtype_value)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        errors.append(f"{path} has an invalid provider shape or dtype: {exc}")
        return None
    if not shape:
        errors.append(f"{path} must have a leading sample dimension")
        return None
    if (
        dtype.hasobject
        or dtype.fields is not None
        or dtype.subdtype is not None
        or dtype.kind not in "biuf"
    ):
        errors.append(f"{path} uses an unsupported provider dtype {dtype}")
        return None
    try:
        source_bytes = checked_shape_nbytes(
            shape,
            dtype,
            resource=f"{path} source payload",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        source_row_bytes = checked_shape_nbytes(
            (1, *shape[1:]),
            dtype,
            resource=f"{path} source row",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        # This check is deliberately performed from the declaration before a
        # provider can call ``dataset[()]`` followed by ``astype(float32)``.
        converted_bytes = checked_shape_nbytes(
            shape,
            np.dtype(np.float32),
            resource=f"{path} float32 conversion",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        converted_row_bytes = checked_shape_nbytes(
            (1, *shape[1:]),
            np.dtype(np.float32),
            resource=f"{path} float32 row",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        row_elements = converted_row_bytes // np.dtype(np.float32).itemsize
        safe_row_bytes = max(source_row_bytes, row_elements * 8)
    except (TypeError, ValueError, ResourceLimitError) as exc:
        errors.append(f"{path} provider conversion exceeds bounds: {exc}")
        return None
    return source_bytes, converted_bytes, source_row_bytes, safe_row_bytes, row_elements


def _provider_sample_count(demo: Any) -> int | None:
    """Read an admitted demo length used for native zero synthesis."""

    try:
        # Robomimic synthesizes absent keys with ``num_samples`` rows, so the
        # declaration charges that count; admission has already required it
        # to equal every selected dataset's length.
        attrs = getattr(demo, "attrs", None)
        sample_count = (
            normalize_int_value(attrs["num_samples"])
            if attrs is not None and "num_samples" in attrs
            else None
        )
        if sample_count is None:
            actions = demo.get("actions")
            sample_count = _hdf5_first_dim(actions) if actions is not None else None
        if sample_count is None:
            return None
        return require_bounded_int(
            sample_count,
            resource="Robomimic provider demo samples",
            minimum=0,
            maximum=MAX_EVENT_ROWS,
        )
    except (AttributeError, KeyError, OSError, TypeError, ValueError):
        return None


def _provider_declaration(
    data_group: Any,
    *,
    demo_keys: Sequence[str],
    dataset_keys: Sequence[str],
    obs_keys: Sequence[str] | None,
    load_next_obs: bool,
    mask_counts: Mapping[str, int],
    max_demo_multiplicity: int,
    mask_decoded_bytes: int,
    errors: _BoundedErrors,
) -> RobomimicProviderDeclaration:
    totals: dict[str, int] = {
        "selected_source_bytes": 0,
        "converted_float32_bytes": 0,
        "dataset_source_bytes": 0,
        "dataset_converted_float32_bytes": 0,
        "observation_source_bytes": 0,
        "observation_converted_float32_bytes": 0,
        "next_observation_source_bytes": 0,
        "next_observation_converted_float32_bytes": 0,
        "selected_source_row_bytes": 0,
        "selected_source_row_elements": 0,
        "selected_array_count": 0,
        "max_source_row_bytes": 0,
        "max_source_row_elements": 0,
        "max_row_bytes": 0,
        "max_row_elements": 0,
    }
    max_rows: dict[str, int] = {
        "dataset_source_row_bytes": 0,
        "dataset_source_row_elements": 0,
        "dataset_row_bytes": 0,
        "dataset_row_elements": 0,
        "observation_source_row_bytes": 0,
        "observation_source_row_elements": 0,
        "observation_row_bytes": 0,
        "observation_row_elements": 0,
        "next_observation_source_row_bytes": 0,
        "next_observation_source_row_elements": 0,
        "next_observation_row_bytes": 0,
        "next_observation_row_elements": 0,
        "selected_source_row_bytes": 0,
        "selected_source_row_elements": 0,
        "selected_row_bytes": 0,
        "selected_row_elements": 0,
    }

    for demo_key in demo_keys:
        demo = data_group[demo_key]
        if not hasattr(demo, "get"):
            continue
        sample_count = _provider_sample_count(demo)
        per_demo: dict[str, int] = {
            "dataset_source_row_bytes": 0,
            "dataset_source_row_elements": 0,
            "dataset_row_bytes": 0,
            "dataset_row_elements": 0,
            "observation_source_row_bytes": 0,
            "observation_source_row_elements": 0,
            "observation_row_bytes": 0,
            "observation_row_elements": 0,
            "next_observation_source_row_bytes": 0,
            "next_observation_source_row_elements": 0,
            "next_observation_row_bytes": 0,
            "next_observation_row_elements": 0,
        }

        def add_synthesized_dataset(path: str) -> None:
            if sample_count is None:
                errors.append(f"{path} cannot determine demo sample count")
                return
            try:
                converted_bytes = checked_shape_nbytes(
                    (sample_count, 1),
                    np.dtype(np.float32),
                    resource=f"{path} synthesized float32 payload",
                    max_elements=MAX_ARRAY_ELEMENTS,
                    max_bytes=MAX_ARRAY_BYTES,
                )
            except (TypeError, ValueError, ResourceLimitError) as exc:
                errors.append(f"{path} provider synthesis exceeds bounds: {exc}")
                return
            row_elements = 1
            row_bytes = 8
            totals["converted_float32_bytes"] += converted_bytes
            totals["dataset_converted_float32_bytes"] += converted_bytes
            totals["selected_array_count"] += 1
            totals["max_row_bytes"] = max(totals["max_row_bytes"], row_bytes)
            totals["max_row_elements"] = max(totals["max_row_elements"], row_elements)
            per_demo["dataset_row_bytes"] += row_bytes
            per_demo["dataset_row_elements"] += row_elements

        def add_dataset(dataset: Any, path: str, category: str) -> None:
            metrics = _provider_array_metrics(dataset, path=path, errors=errors)
            if metrics is None:
                return
            (
                source_bytes,
                converted_bytes,
                source_row_bytes,
                row_bytes,
                row_elements,
            ) = metrics
            totals["selected_source_bytes"] += source_bytes
            totals["converted_float32_bytes"] += converted_bytes
            totals[f"{category}_source_bytes"] += source_bytes
            totals[f"{category}_converted_float32_bytes"] += converted_bytes
            totals["selected_source_row_bytes"] = max(
                totals["selected_source_row_bytes"], source_row_bytes
            )
            totals["selected_source_row_elements"] = max(
                totals["selected_source_row_elements"], row_elements
            )
            totals["selected_array_count"] += 1
            totals["max_source_row_bytes"] = max(
                totals["max_source_row_bytes"], source_row_bytes
            )
            totals["max_source_row_elements"] = max(
                totals["max_source_row_elements"], row_elements
            )
            totals["max_row_bytes"] = max(totals["max_row_bytes"], row_bytes)
            totals["max_row_elements"] = max(totals["max_row_elements"], row_elements)
            per_demo[f"{category}_row_bytes"] += row_bytes
            per_demo[f"{category}_row_elements"] += row_elements
            per_demo[f"{category}_source_row_bytes"] += source_row_bytes
            per_demo[f"{category}_source_row_elements"] += row_elements

        for dataset_key in dataset_keys:
            dataset = demo.get(dataset_key)
            if dataset is None:
                add_synthesized_dataset(f"data/{demo_key}/{dataset_key}")
                continue
            add_dataset(
                dataset,
                f"data/{demo_key}/{dataset_key}",
                "dataset",
            )

        if obs_keys is not None:
            obs_group = demo.get("obs")
            next_obs_group = demo.get("next_obs") if load_next_obs else None
            for obs_key in obs_keys:
                if obs_group is not None and hasattr(obs_group, "get"):
                    dataset = obs_group.get(obs_key)
                    if dataset is not None:
                        add_dataset(
                            dataset,
                            f"data/{demo_key}/obs/{obs_key}",
                            "observation",
                        )
                if next_obs_group is not None and hasattr(next_obs_group, "get"):
                    dataset = next_obs_group.get(obs_key)
                    if dataset is not None:
                        add_dataset(
                            dataset,
                            f"data/{demo_key}/next_obs/{obs_key}",
                            "next_observation",
                        )

        for category in (
            "dataset",
            "observation",
            "next_observation",
        ):
            max_rows[f"{category}_source_row_bytes"] = max(
                max_rows[f"{category}_source_row_bytes"],
                per_demo[f"{category}_source_row_bytes"],
            )
            max_rows[f"{category}_source_row_elements"] = max(
                max_rows[f"{category}_source_row_elements"],
                per_demo[f"{category}_source_row_elements"],
            )
            max_rows[f"{category}_row_bytes"] = max(
                max_rows[f"{category}_row_bytes"],
                per_demo[f"{category}_row_bytes"],
            )
            max_rows[f"{category}_row_elements"] = max(
                max_rows[f"{category}_row_elements"],
                per_demo[f"{category}_row_elements"],
            )
        selected_row_bytes = sum(
            per_demo[f"{category}_row_bytes"]
            for category in ("dataset", "observation", "next_observation")
        )
        selected_row_elements = sum(
            per_demo[f"{category}_row_elements"]
            for category in ("dataset", "observation", "next_observation")
        )
        selected_source_row_bytes = sum(
            per_demo[f"{category}_source_row_bytes"]
            for category in ("dataset", "observation", "next_observation")
        )
        selected_source_row_elements = sum(
            per_demo[f"{category}_source_row_elements"]
            for category in ("dataset", "observation", "next_observation")
        )
        max_rows["selected_source_row_bytes"] = max(
            max_rows["selected_source_row_bytes"], selected_source_row_bytes
        )
        max_rows["selected_source_row_elements"] = max(
            max_rows["selected_source_row_elements"], selected_source_row_elements
        )
        max_rows["selected_row_bytes"] = max(
            max_rows["selected_row_bytes"], selected_row_bytes
        )
        max_rows["selected_row_elements"] = max(
            max_rows["selected_row_elements"], selected_row_elements
        )

    return RobomimicProviderDeclaration(
        selected_source_bytes=totals["selected_source_bytes"],
        converted_float32_bytes=totals["converted_float32_bytes"],
        dataset_source_bytes=totals["dataset_source_bytes"],
        dataset_converted_float32_bytes=totals["dataset_converted_float32_bytes"],
        observation_source_bytes=totals["observation_source_bytes"],
        observation_converted_float32_bytes=totals[
            "observation_converted_float32_bytes"
        ],
        next_observation_source_bytes=totals["next_observation_source_bytes"],
        next_observation_converted_float32_bytes=totals[
            "next_observation_converted_float32_bytes"
        ],
        selected_source_row_bytes=max_rows["selected_source_row_bytes"],
        selected_source_row_elements=max_rows["selected_source_row_elements"],
        selected_row_bytes=max_rows["selected_row_bytes"],
        selected_row_elements=max_rows["selected_row_elements"],
        dataset_source_row_bytes=max_rows["dataset_source_row_bytes"],
        dataset_source_row_elements=max_rows["dataset_source_row_elements"],
        dataset_row_bytes=max_rows["dataset_row_bytes"],
        dataset_row_elements=max_rows["dataset_row_elements"],
        observation_source_row_bytes=max_rows["observation_source_row_bytes"],
        observation_source_row_elements=max_rows["observation_source_row_elements"],
        observation_row_bytes=max_rows["observation_row_bytes"],
        observation_row_elements=max_rows["observation_row_elements"],
        next_observation_source_row_bytes=max_rows["next_observation_source_row_bytes"],
        next_observation_source_row_elements=max_rows[
            "next_observation_source_row_elements"
        ],
        next_observation_row_bytes=max_rows["next_observation_row_bytes"],
        next_observation_row_elements=max_rows["next_observation_row_elements"],
        max_source_row_bytes=totals["max_source_row_bytes"],
        max_source_row_elements=totals["max_source_row_elements"],
        max_row_bytes=totals["max_row_bytes"],
        max_row_elements=totals["max_row_elements"],
        selected_array_count=totals["selected_array_count"],
        mask_rows=sum(mask_counts.values()),
        max_mask_rows=max(mask_counts.values(), default=0),
        max_demo_multiplicity=max_demo_multiplicity,
        mask_decoded_bytes=mask_decoded_bytes,
    )


def _metadata_decoded_cost(h5: Any, text_metadata: Mapping[str, str | bytes]) -> int:
    """Account for decoded text instead of its encoded HDF5 storage."""

    from srb.core.sensor.event_camera.storage import _attribute_storage_size

    data = h5.get("data")
    data_identity = _hdf5_object_identity(data) if data is not None else None
    pending: list[Any] = [h5]
    seen_objects: set[int] = set()
    stored_text = 0
    stored_total = 0
    while pending:
        current = pending.pop()
        identity = _hdf5_object_identity(current)
        if identity in seen_objects:
            continue
        seen_objects.add(identity)
        attrs = getattr(current, "attrs", None)
        if attrs is not None:
            for name in attrs:
                attribute = attrs.get_id(name)
                try:
                    stored = int(_attribute_storage_size(attribute))
                finally:
                    attribute.close()
                stored_total += stored
                if (
                    name == "model_file"
                    or identity == data_identity
                    and name in _HDF5_TEXT_ATTRIBUTES
                ):
                    stored_text += stored
        if hasattr(current, "keys"):
            for key in current.keys():
                pending.append(current[key])
    decoded_text = sum(
        len(value.encode("utf-8", "surrogateescape"))
        if isinstance(value, str)
        else len(value)
        for value in text_metadata.values()
    )
    return stored_total - stored_text + decoded_text


def _admit_hdf5(
    source_path: Path,
    *,
    sha256: str,
    max_decoded_bytes: int,
    max_metadata_bytes: int,
    max_model_file_bytes: int,
    dataset_keys: Sequence[str] | None = None,
    obs_keys: Sequence[str] | None = None,
    load_next_obs: bool = True,
    mask_keys: Sequence[str] | None = None,
    cache_mode: str | None = None,
    provider_specific: bool = False,
    enforce_missing_dataset_keys: bool = False,
    json_budget: JsonDecodeBudget | None = None,
    admitted_env_meta: FrozenJSONObject | None = None,
) -> tuple[
    FrozenJSONObject,
    Mapping[str, str | bytes],
    int,
    int,
    int,
    RobomimicProviderDeclaration,
    int,
    int,
]:
    deadline_at = time.monotonic() + _ADMISSION_TIMEOUT_S
    h5py = importlib.import_module("h5py")
    errors = _BoundedErrors()
    with h5py.File(source_path, "r") as h5:
        if not _preflight_hdf5_storage(h5, errors=errors):
            _raise_errors(errors, path=source_path)
        try:
            text_metadata = _preflight_hdf5_metadata_storage(
                h5,
                path=source_path,
                max_aggregate_bytes=max_metadata_bytes,
            )
        except Hdf5MetadataCleanupError:
            raise
        except (OSError, TypeError, ValueError) as exc:
            raise ValueError(
                f"robomimic training metadata rejected {source_path}: {exc}"
            ) from exc
        if time.monotonic() >= deadline_at:
            raise Hdf5MetadataTimeoutError(
                "Robomimic training metadata admission deadline expired"
            )
        metadata_budget = ByteBudget(
            "Robomimic training metadata bytes",
            max_metadata_bytes,
        )
        metadata_budget.consume(_metadata_decoded_cost(h5, text_metadata))
        if admitted_env_meta is None:
            env_meta = _parse_env_meta(
                text_metadata.get("env_args"),
                path=source_path,
                json_budget=json_budget,
            )
        else:
            if "env_args" not in text_metadata:
                raise ValueError(
                    f"robomimic training dataset {source_path} is missing data/env_args"
                )
            env_meta = admitted_env_meta

        data_group = h5.get("data")
        if data_group is None or not hasattr(data_group, "keys"):
            raise ValueError("robomimic training dataset must contain data group")
        try:
            demo_keys = sorted(str(key) for key in data_group.keys())
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            raise ValueError("cannot inspect robomimic training demos") from exc
        if not demo_keys:
            raise ValueError("robomimic training dataset contains no demos")
        for demo_key in demo_keys:
            demo = data_group[demo_key]
            attrs = getattr(demo, "attrs", None)
            if attrs is None or "num_samples" not in attrs:
                errors.append(
                    f"data/{demo_key} is missing required scalar attribute "
                    "'num_samples'"
                )

        model_files, model_file_bytes = _read_model_files(
            data_group=data_group,
            demo_keys=demo_keys,
            source_path=source_path,
            sha256=sha256,
            max_model_file_bytes=max_model_file_bytes,
            metadata_budget=metadata_budget,
            deadline_at=deadline_at,
        )
        decoded_budget = ByteBudget(
            "Robomimic training decoded bytes", max_decoded_bytes
        )
        checked_dataset_keys, checked_obs_keys, _load_next, checked_mask_keys = (
            _provider_options(
                None,
                dataset_keys=dataset_keys,
                action_keys=None,
                obs_keys=obs_keys,
                load_next_obs=load_next_obs,
                mask_keys=mask_keys,
            )
        )
        if provider_specific:
            demo_count, total_samples = _validate_configured_provider_demos(
                h5=h5,
                demo_keys=demo_keys,
                dataset_keys=checked_dataset_keys,
                obs_keys=checked_obs_keys,
                load_next_obs=_load_next,
                errors=errors,
                decoded_budget=decoded_budget,
                deadline_at=deadline_at,
            )
        else:
            demo_count, total_samples = _validate_robomimic_demos(
                h5=h5,
                demo_keys=demo_keys,
                errors=errors,
                decoded_budget=decoded_budget,
                deadline_check=_admission_deadline_check(deadline_at),
            )
        if time.monotonic() >= deadline_at:
            raise Hdf5MetadataTimeoutError(
                "Robomimic training admission deadline expired"
            )
        _validate_extra_dataset_keys(
            data_group,
            demo_keys=demo_keys,
            dataset_keys=checked_dataset_keys,
            budget=decoded_budget,
            errors=errors,
            allow_missing=(
                not enforce_missing_dataset_keys or cache_mode in ("all", "low_dim")
            ),
            validate_payload=not provider_specific,
        )
        if not provider_specific and checked_obs_keys is not None:
            for demo_key in demo_keys:
                demo = data_group[demo_key]
                if not hasattr(demo, "get"):
                    continue
                obs = demo.get("obs")
                for obs_key in checked_obs_keys:
                    if obs is None or obs_key not in obs:
                        errors.append(f"data/{demo_key}/obs/{obs_key} is missing")
                if _load_next:
                    next_obs = demo.get("next_obs")
                    for obs_key in checked_obs_keys:
                        if next_obs is None or obs_key not in next_obs:
                            errors.append(
                                f"data/{demo_key}/next_obs/{obs_key} is missing"
                            )
        mask_budget_before = decoded_budget.consumed
        mask_counts, max_demo_multiplicity = _validate_optional_masks(
            h5,
            demo_keys=set(demo_keys),
            mask_keys=checked_mask_keys,
            budget=decoded_budget,
            errors=errors,
            deadline_at=deadline_at,
        )
        mask_decoded_bytes = decoded_budget.consumed - mask_budget_before
        declaration = _provider_declaration(
            data_group,
            demo_keys=demo_keys,
            dataset_keys=checked_dataset_keys,
            obs_keys=checked_obs_keys,
            load_next_obs=_load_next,
            mask_counts=mask_counts,
            max_demo_multiplicity=max_demo_multiplicity,
            mask_decoded_bytes=mask_decoded_bytes,
            errors=errors,
        )

        counts = _read_hdf5_count_attributes(data_group.attrs)
        for name, actual in (
            ("num_demos", demo_count),
            ("total_samples", total_samples),
        ):
            declared = counts.get(name)
            normalized_declared = normalize_int_value(declared)
            if declared is not None:
                if normalized_declared is None:
                    errors.append(f"data/{name} must be an integer")
                elif normalized_declared != actual:
                    errors.append(
                        f"data/{name}={normalized_declared} does not match actual {actual}"
                    )
        _raise_errors(errors, path=source_path)
        if time.monotonic() >= deadline_at:
            raise Hdf5MetadataTimeoutError(
                "Robomimic training admission deadline expired"
            )
        return (
            env_meta,
            model_files,
            demo_count,
            total_samples,
            decoded_budget.consumed,
            declaration,
            metadata_budget.consumed,
            model_file_bytes,
        )


class RobomimicTrainingDataset:
    """Private verified Robomimic input with explicit owner lifetime."""

    __slots__ = (
        "_dataset_path",
        "_source_path",
        "_sha256",
        "_source_bytes",
        "_env_meta",
        "_model_files",
        "_num_demos",
        "_total_samples",
        "_declared_decoded_bytes",
        "_metadata_decoded_bytes",
        "_model_file_bytes",
        "_declaration",
        "_max_decoded_bytes",
        "_max_metadata_bytes",
        "_max_model_file_bytes",
        "_json_budget",
        "_source_owner",
        "_pending_metadata_cleanup",
        "_closed",
    )

    def __init__(
        self,
        source: _OwnedInput,
        *,
        env_meta: FrozenJSONObject,
        model_files: Mapping[str, str | bytes],
        num_demos: int,
        total_samples: int,
        declared_decoded_bytes: int,
        metadata_decoded_bytes: int,
        model_file_bytes: int,
        declaration: RobomimicProviderDeclaration,
        max_decoded_bytes: int,
        max_metadata_bytes: int,
        max_model_file_bytes: int,
        json_budget: JsonDecodeBudget,
    ) -> None:
        self._dataset_path = source.dataset_path
        self._source_path = source.source_path
        self._sha256 = source.sha256
        self._source_bytes = source.source_bytes
        self._env_meta = env_meta
        self._model_files = MappingProxyType(dict(model_files))
        self._num_demos = num_demos
        self._total_samples = total_samples
        self._declared_decoded_bytes = declared_decoded_bytes
        self._metadata_decoded_bytes = metadata_decoded_bytes
        self._model_file_bytes = model_file_bytes
        self._declaration = declaration
        self._max_decoded_bytes = max_decoded_bytes
        self._max_metadata_bytes = max_metadata_bytes
        self._max_model_file_bytes = max_model_file_bytes
        self._json_budget = json_budget
        self._source_owner = source
        self._pending_metadata_cleanup: Hdf5MetadataCleanupError | None = None
        self._closed = False

    @property
    def dataset_path(self) -> Path:
        return self._dataset_path

    @property
    def source_path(self) -> Path:
        return self._source_path

    @property
    def sha256(self) -> str:
        return self._sha256

    @property
    def source_bytes(self) -> int:
        return self._source_bytes

    @property
    def env_meta(self) -> dict[str, JSONValue]:
        return cast(dict[str, JSONValue], thaw_json_value(self._env_meta))

    @property
    def model_files(self) -> Mapping[str, str | bytes]:
        return MappingProxyType(dict(self._model_files))

    @property
    def num_demos(self) -> int:
        return self._num_demos

    @property
    def total_samples(self) -> int:
        return self._total_samples

    @property
    def declared_decoded_bytes(self) -> int:
        return self._declared_decoded_bytes

    @property
    def metadata_decoded_bytes(self) -> int:
        return self._metadata_decoded_bytes

    @property
    def model_file_bytes(self) -> int:
        return self._model_file_bytes

    @property
    def declaration(self) -> RobomimicProviderDeclaration:
        return self._declaration

    def admit_provider(
        self,
        config: object | None = None,
        *,
        dataset_keys: Sequence[str] | None = None,
        action_keys: Sequence[str] | None = None,
        obs_keys: Sequence[str] | None = None,
        load_next_obs: bool | None = None,
        mask_keys: Sequence[str] | None = None,
        max_decoded_bytes: int | None = None,
    ) -> RobomimicProviderAdmission:
        """Validate provider fields using this owner’s admitted environment metadata."""

        if self._closed:
            raise PublicationError("robomimic training dataset is closed")
        if self._pending_metadata_cleanup is not None:
            raise PublicationError(
                "robomimic training dataset metadata cleanup is pending"
            )
        checked_limit = _checked_limit(
            max_decoded_bytes,
            default=self._max_decoded_bytes,
            maximum=self._max_decoded_bytes,
            resource="Robomimic provider decoded bytes",
        )
        (
            checked_dataset_keys,
            checked_obs_keys,
            checked_load_next,
            checked_mask_keys,
        ) = _provider_options(
            config,
            dataset_keys=dataset_keys,
            action_keys=action_keys,
            obs_keys=obs_keys,
            load_next_obs=load_next_obs,
            mask_keys=mask_keys,
        )
        provider_specific = config is not None or any(
            "/" in key for key in (*checked_dataset_keys, *(checked_obs_keys or ()))
        )
        cache_mode = _config_value(
            config,
            ("train", "hdf5_cache_mode"),
            None,
        )
        if cache_mode not in (None, "all", "low_dim"):
            raise ValueError(f"unsupported robomimic cache mode: {cache_mode!r}")
        try:
            (
                _,
                _,
                demo_count,
                total_samples,
                decoded_bytes,
                declaration,
                metadata_decoded_bytes,
                model_file_bytes,
            ) = _admit_hdf5(
                self._dataset_path,
                sha256=self._sha256,
                max_decoded_bytes=checked_limit,
                max_metadata_bytes=self._max_metadata_bytes,
                max_model_file_bytes=self._max_model_file_bytes,
                dataset_keys=checked_dataset_keys,
                obs_keys=checked_obs_keys,
                load_next_obs=checked_load_next,
                mask_keys=checked_mask_keys,
                cache_mode=cast(str | None, cache_mode),
                provider_specific=provider_specific,
                enforce_missing_dataset_keys=True,
                json_budget=self._json_budget,
                admitted_env_meta=self._env_meta,
            )
        except Hdf5MetadataCleanupError as error:
            self._source_owner.defer_finalizer_cleanup()
            self._pending_metadata_cleanup = error
            error.retain_cleanup(self._release_source_after_metadata_cleanup)
            raise
        return RobomimicProviderAdmission(
            dataset_path=self._dataset_path,
            source_path=self._source_path,
            sha256=self._sha256,
            source_bytes=self._source_bytes,
            num_demos=demo_count,
            total_samples=total_samples,
            declared_decoded_bytes=decoded_bytes,
            metadata_decoded_bytes=metadata_decoded_bytes,
            model_file_bytes=model_file_bytes,
            dataset_keys=checked_dataset_keys,
            obs_keys=checked_obs_keys,
            mask_keys=checked_mask_keys,
            declaration=declaration,
        )

    def _release_source_after_metadata_cleanup(self) -> None:
        self._source_owner.close()
        self._pending_metadata_cleanup = None
        self._closed = True

    def close(self) -> None:
        if self._closed:
            return
        if self._pending_metadata_cleanup is not None:
            self._pending_metadata_cleanup.retry_cleanup()
            return
        self._source_owner.close()
        self._closed = True

    def __enter__(self) -> "RobomimicTrainingDataset":
        if self._closed:
            raise PublicationError("robomimic training dataset is closed")
        if self._pending_metadata_cleanup is not None:
            raise PublicationError(
                "robomimic training dataset metadata cleanup is pending"
            )
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: object | None,
    ) -> None:
        self.close()

    def __del__(self) -> None:  # pragma: no cover - interpreter cleanup
        try:
            self.close()
        except Exception:
            pass


def _open_owned_input(
    path: str | Path,
    *,
    max_source_bytes: int,
    json_budget: JsonDecodeBudget,
) -> _OwnedInput:
    candidate = Path(path).expanduser()
    try:
        manifest_path = find_dataset_manifest(candidate)
    except FileNotFoundError:
        manifest_hints = [candidate.with_suffix(".srb.json")]
        if candidate.suffix == ".json":
            manifest_hints.append(candidate)
        if candidate.is_dir():
            manifest_hints.extend(
                (
                    candidate / "meta" / "dataset_manifest.json",
                    candidate / "dataset_manifest.json",
                )
            )
        manifest_hints.extend(
            root / "meta" / "dataset_manifest.json" for root in candidate.parents
        )
        if any(hint.exists() or hint.is_symlink() for hint in manifest_hints):
            raise
        source_path = candidate.absolute()
        require_dataset_file_size(
            source_path,
            resource="Robomimic training source",
            max_bytes=max_source_bytes,
        )
        hash_budget = ByteBudget(
            "Robomimic training source bytes",
            max_source_bytes,
        )
        sha256 = sha256_file(source_path, byte_budget=hash_budget)
        copy_budget = ByteBudget(
            "Robomimic training source bytes",
            max_source_bytes,
        )
        materialized = materialize_file_snapshot(
            {source_path.name: (source_path, sha256)},
            max_entries=1,
            max_bytes=max_source_bytes,
            byte_budget=copy_budget,
        )
        return _OwnedInput(
            source_path=source_path,
            dataset_path=materialized.path / source_path.name,
            sha256=sha256,
            source_bytes=(materialized.path / source_path.name).stat().st_size,
            owner=materialized,
        )
    # Check the referenced file before the snapshot copier allocates its
    # private destination. The snapshot still revalidates identity, size, and
    # digest after copying to close the source race.
    manifest = read_dataset_manifest(manifest_path, json_budget=json_budget)
    if manifest.storage_format != StorageFormat.ROBOMIMIC_HDF5:
        raise InvalidInputError(
            f"{manifest_path} declares storage_format "
            f"{manifest.storage_format.value}; robomimic training needs "
            f"{StorageFormat.ROBOMIMIC_HDF5.value} (convert the dataset first)",
            component="srb.datasets.robomimic",
        )
    manifest_source = resolve_dataset_path(manifest.dataset_path, manifest_path)
    require_dataset_file_size(
        manifest_source,
        resource="Robomimic training source",
        max_bytes=max_source_bytes,
    )
    snapshot = open_dataset_snapshot(
        manifest_path,
        max_source_bytes=max_source_bytes,
        json_budget=json_budget,
    )
    ownership_transferred = False
    try:
        dataset_path = snapshot.dataset_path
        if not dataset_path.is_file():
            raise ValueError(
                f"robomimic training dataset is not a regular HDF5 file: {dataset_path}"
            )
        source_bytes = dataset_path.stat().st_size
        if source_bytes > max_source_bytes:
            raise ResourceLimitError(
                "Robomimic training source",
                limit=max_source_bytes,
                observed=source_bytes,
            )
        _provenance_name, sha256 = snapshot.member_identity(dataset_path)
        source_path = snapshot.source_dataset_path or candidate
        owned = _OwnedInput(
            source_path=Path(source_path).absolute(),
            dataset_path=dataset_path,
            sha256=sha256,
            source_bytes=source_bytes,
            owner=snapshot,
        )
        ownership_transferred = True
        return owned
    finally:
        if not ownership_transferred:
            startup_error = sys.exc_info()[1]
            _, cleanup_error = attempt_cleanup(snapshot.close)
            if cleanup_error is not None:
                _attach_cleanup_owner(cleanup_error, snapshot)
                if startup_error is not None:
                    raise cleanup_error from startup_error
                raise cleanup_error


def open_robomimic_training_dataset(
    path: str | Path,
    *,
    max_source_bytes: int | None = None,
    max_decoded_bytes: int | None = None,
    max_metadata_bytes: int | None = None,
    max_model_file_bytes: int | None = None,
    dataset_keys: Sequence[str] | None = None,
    action_keys: Sequence[str] | None = None,
    obs_keys: Sequence[str] | None = None,
    load_next_obs: bool | None = None,
    json_budget: JsonDecodeBudget | None = None,
) -> RobomimicTrainingDataset:
    """Own and admit one bare or manifest-backed Robomimic HDF5 file.

    Explicit provider fields admit nested action paths and optional datasets.
    Without them, require the SRB actions/rewards/dones contract.
    """

    checked_source = _checked_limit(
        max_source_bytes,
        default=MAX_EXTERNAL_FILE_BYTES,
        maximum=MAX_EXTERNAL_FILE_BYTES,
        resource="Robomimic training source bytes",
    )
    checked_decoded = _checked_limit(
        max_decoded_bytes,
        default=MAX_DECODED_AGGREGATE_BYTES,
        maximum=MAX_DECODED_AGGREGATE_BYTES,
        resource="Robomimic training decoded bytes",
    )
    checked_metadata = _checked_limit(
        max_metadata_bytes,
        default=MAX_METADATA_AGGREGATE_BYTES,
        maximum=MAX_METADATA_AGGREGATE_BYTES,
        resource="Robomimic training metadata bytes",
    )
    checked_model_file = _checked_limit(
        max_model_file_bytes,
        default=MAX_METADATA_ATTRIBUTE_BYTES,
        maximum=MAX_METADATA_ATTRIBUTE_BYTES,
        resource="Robomimic model_file bytes",
    )
    if json_budget is None:
        checked_json_budget = JsonDecodeBudget()
    elif isinstance(json_budget, JsonDecodeBudget):
        checked_json_budget = json_budget
    else:
        raise TypeError("json_budget must be a JsonDecodeBudget or None")
    provider_specific = any(
        value is not None
        for value in (dataset_keys, action_keys, obs_keys, load_next_obs)
    )
    if provider_specific:
        (
            checked_dataset_keys,
            checked_obs_keys,
            checked_load_next_obs,
            checked_mask_keys,
        ) = _provider_options(
            None,
            dataset_keys=dataset_keys,
            action_keys=action_keys,
            obs_keys=obs_keys,
            load_next_obs=load_next_obs,
            mask_keys=None,
        )
    else:
        checked_dataset_keys = None
        checked_obs_keys = None
        checked_load_next_obs = True
        checked_mask_keys = ()
    source: _OwnedInput | None = None
    ownership_transferred = False
    try:
        source = _open_owned_input(
            path,
            max_source_bytes=checked_source,
            json_budget=checked_json_budget,
        )
        (
            env_meta,
            model_files,
            num_demos,
            total_samples,
            decoded_bytes,
            declaration,
            metadata_decoded_bytes,
            model_file_bytes,
        ) = _admit_hdf5(
            source.dataset_path,
            sha256=source.sha256,
            max_decoded_bytes=checked_decoded,
            max_metadata_bytes=checked_metadata,
            max_model_file_bytes=checked_model_file,
            dataset_keys=checked_dataset_keys,
            obs_keys=checked_obs_keys,
            load_next_obs=checked_load_next_obs,
            mask_keys=checked_mask_keys,
            provider_specific=provider_specific,
            json_budget=checked_json_budget,
        )
        owner = RobomimicTrainingDataset(
            source,
            env_meta=env_meta,
            model_files=model_files,
            num_demos=num_demos,
            total_samples=total_samples,
            declared_decoded_bytes=decoded_bytes,
            metadata_decoded_bytes=metadata_decoded_bytes,
            model_file_bytes=model_file_bytes,
            declaration=declaration,
            max_decoded_bytes=checked_decoded,
            max_metadata_bytes=checked_metadata,
            max_model_file_bytes=checked_model_file,
            json_budget=checked_json_budget,
        )
        ownership_transferred = True
        return owner
    except Hdf5MetadataCleanupError as error:
        if source is not None:
            source.defer_finalizer_cleanup()
            error.retain_cleanup(source.close)
            source = None
        raise
    finally:
        if not ownership_transferred and source is not None:
            startup_error = sys.exc_info()[1]
            _, cleanup_error = attempt_cleanup(source.close)
            if cleanup_error is not None:
                _attach_cleanup_owner(cleanup_error, source)
                if startup_error is not None:
                    raise cleanup_error from startup_error
                raise cleanup_error


__all__ = [
    "RobomimicProviderDeclaration",
    "RobomimicProviderAdmission",
    "RobomimicTrainingDataset",
    "open_robomimic_training_dataset",
]
