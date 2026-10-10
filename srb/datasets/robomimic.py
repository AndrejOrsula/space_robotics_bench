# SPDX-License-Identifier: Apache-2.0 OR MIT
"""Robomimic HDF5 dataset writer utilities."""

from __future__ import annotations

import importlib
import importlib.util
import json
import math
import os
import sys
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass, field, replace
from itertools import zip_longest
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from srb.datasets.metadata_validation import (
    normalize_int_value as _normalize_int_value,
)
from srb.datasets.metadata_validation import (
    validate_demo_io_contract as _validate_demo_io_contract,
)
from srb.datasets.metadata_validation import (
    validate_metadata_count as _validate_metadata_count,
)
from srb.datasets.metadata_validation import (
    validate_metadata_observation_keys as _validate_metadata_obs_keys,
)
from srb.datasets.recording import TrajectoryBuffer, snapshot_action_space
from srb.utils.failures import attempt_cleanup
from srb.utils.json_types import JsonDecodeBudget, bounded_json_dumps
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_ARRAY_ELEMENTS,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_DIRECTORY_ENTRIES,
    MAX_EVENT_BATCH_ROWS,
    MAX_EVENT_ROWS,
    MAX_EXTERNAL_FILE_BYTES,
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_METADATA_AGGREGATE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    MAX_METADATA_ATTRIBUTES,
    MAX_PUBLICATION_DIRECTORY_BYTES,
    MAX_PUBLICATION_DIRECTORY_DEPTH,
    ByteBudget,
    ResourceLimitError,
    checked_shape_nbytes,
    require_bounded_int,
    require_dataset_file_size,
)

if TYPE_CHECKING:
    from typing import Any, Sequence

    from srb.datasets.hdf5_native import Hdf5NativeLimits
    from srb.utils.publication import PublicationCleanupError


#: ``robomimic.envs.env_base.EnvType.GYM_TYPE``. SRB environments are
#: Gymnasium environments, and robomimic only uses this to pick the env class
#: it would rebuild for rollouts, which offline learning never does.
ROBOMIMIC_GYM_ENV_TYPE = 2
_MAX_DEMO_IO_CONTRACT_BYTES = 1 << 20
_MASK_UNKNOWN_REPORT_LIMIT = 8
# Covers the temporary bytes and str object headers for one fixed-width value.
_MASK_STRING_CONVERSION_OVERHEAD = 128
_HDF5_COUNT_ATTRIBUTES = ("num_demos", "total_samples", "num_train", "num_valid")
_HDF5_TEXT_ATTRIBUTES = ("env_name", "env_kwargs", "env_args", "action_schema")
_EPISODE_RECORDING_ATTRIBUTE = "srb_episode_recording"
# Errors a demo declaration read reports as "unreadable" instead of raising.
_UNREADABLE_DECLARATION_ERRORS = (
    AttributeError,
    KeyError,
    OSError,
    TypeError,
    ValueError,
)


def _encode_episode_recording_reference(
    reference: Mapping[str, object] | None,
) -> str | None:
    """Validate and encode one bounded generic-to-sidecar episode reference."""
    if reference is None:
        return None
    if not isinstance(reference, Mapping):
        raise TypeError("episode_recording must be a mapping")
    payload = bounded_json_dumps(
        dict(reference),
        context="recorder episode recording reference",
        sort_keys=True,
        max_bytes=MAX_METADATA_ATTRIBUTE_BYTES,
    )
    value = JsonDecodeBudget().loads(
        payload,
        context="recorder episode recording reference",
        max_bytes=MAX_METADATA_ATTRIBUTE_BYTES,
    )
    if not isinstance(value, dict):
        raise TypeError("episode_recording must encode a JSON object")
    if value.get("schema") != "srb.episode_recording.v1":
        raise ValueError("episode_recording has an unsupported schema")
    for name in ("record_dir", "episode_dir"):
        if type(value.get(name)) is not str or not value[name]:
            raise ValueError(f"episode_recording.{name} must be a non-empty string")
    for name in ("env_id", "episode_id"):
        candidate = value.get(name)
        if type(candidate) is not int or candidate < 0:
            raise ValueError(f"episode_recording.{name} must be a non-negative integer")
    manifest = value.get("event_manifest")
    if manifest is not None and (type(manifest) is not str or not manifest):
        raise ValueError("episode_recording.event_manifest must be a non-empty string")
    return payload


def _read_hdf5_episode_recording_reference(demo: Any) -> dict[str, object] | None:
    """Read one bounded per-demo sidecar reference from a Robomimic group."""
    name = _EPISODE_RECORDING_ATTRIBUTE
    attrs = getattr(demo, "attrs", None)
    if attrs is None or name not in attrs:
        return None
    attribute = None
    try:
        attribute = attrs.get_id(name)
        storage_size = int(attribute.get_storage_size())
        dtype = np.dtype(attribute.dtype)
        shape = tuple(int(dim) for dim in attribute.shape)
    except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
        raise ValueError("cannot inspect HDF5 episode recording reference") from exc
    finally:
        if attribute is not None:
            attribute.close()
    if storage_size > MAX_METADATA_ATTRIBUTE_BYTES:
        raise ValueError("HDF5 episode recording reference exceeds its size limit")
    if shape or dtype.kind != "S" or dtype.hasobject:
        raise ValueError(
            "HDF5 episode recording reference must be a fixed-width byte scalar"
        )
    raw = attrs[name]
    if isinstance(raw, np.ndarray):
        if raw.ndim != 0:
            raise ValueError("HDF5 episode recording reference must be scalar")
        raw = raw.item()
    elif isinstance(raw, np.generic):
        raw = raw.item()
    if not isinstance(raw, (bytes, bytearray)):
        raise ValueError("HDF5 episode recording reference must contain JSON bytes")
    try:
        value = JsonDecodeBudget().loads(
            bytes(raw).decode("utf-8"),
            context="HDF5 episode recording reference",
            max_bytes=MAX_METADATA_ATTRIBUTE_BYTES,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("HDF5 episode recording reference is not valid JSON") from exc
    if not isinstance(value, dict):
        raise ValueError("HDF5 episode recording reference must be a JSON object")
    _encode_episode_recording_reference(value)
    return cast(dict[str, object], value)


class _RobomimicCleanupOwner:
    """Retain publication and dependent cleanup operations for one retry."""

    def __init__(
        self,
        operations: Sequence[Callable[[], object]],
        on_complete: Callable[[], None] | None = None,
    ) -> None:
        self._operations = list(operations)
        self._on_complete = on_complete
        self._completed = False

    def retry_cleanup(self) -> None:
        if self._completed:
            return
        pending: list[Callable[[], object]] = []
        errors: list[BaseException] = []
        for operation in tuple(self._operations):
            _result, error = attempt_cleanup(operation)
            if error is not None:
                pending.append(operation)
                errors.append(error)
        self._operations = pending
        if errors:
            if len(errors) == 1:
                raise errors[0]
            aggregate = RuntimeError("Robomimic cleanup remains pending")
            aggregate.errors = tuple(errors)  # type: ignore[attr-defined]
            raise aggregate from errors[0]
        self._completed = True
        if self._on_complete is not None:
            self._on_complete()


def _publication_committed_path(error: PublicationCleanupError, target: Path) -> bool:
    """Check commit truth reported by the publication owner."""

    expected = Path(os.path.abspath(os.fspath(target)))
    return any(
        Path(os.path.abspath(os.fspath(path))) == expected
        for path in error.committed_paths
    )


def _require_hdf5_primitive_attribute(
    attrs: Any, name: str, *, boolean: bool = False
) -> None:
    """Inspect scalar primitive storage before h5py can decode its value."""
    if name not in attrs:
        return
    attribute = None
    try:
        attribute = attrs.get_id(name)
        shape = attribute.shape
        dtype = np.dtype(attribute.dtype)
        allowed = "biu" if boolean else "iu"
        if (
            shape is None
            or tuple(shape)
            or dtype.hasobject
            or dtype.kind not in allowed
            or dtype.itemsize > 8
        ):
            expected = "a boolean or integer" if boolean else "an integer"
            raise ValueError(
                f"HDF5 {name} must be {expected} scalar stored with fixed width"
            )
    finally:
        if attribute is not None:
            attribute.close()


def _read_hdf5_primitive_attribute(
    attrs: Any, name: str, *, boolean: bool = False
) -> Any:
    _require_hdf5_primitive_attribute(attrs, name, boolean=boolean)
    return attrs[name] if name in attrs else None


def _read_hdf5_count_attributes(attrs: Any) -> dict[str, object]:
    return {
        name: _read_hdf5_primitive_attribute(attrs, name)
        for name in _HDF5_COUNT_ATTRIBUTES
        if name in attrs
    }


def _preflight_robomimic_primitive_metadata(h5: Any) -> None:
    data = h5.get("data")
    if data is None or not hasattr(data, "keys"):
        return
    for name in _HDF5_COUNT_ATTRIBUTES:
        _require_hdf5_primitive_attribute(data.attrs, name)
    for key in data:
        demo = data[key]
        if not hasattr(demo, "keys"):
            continue
        _require_hdf5_primitive_attribute(demo.attrs, "num_samples")
        _require_hdf5_primitive_attribute(demo.attrs, "success", boolean=True)


def _snapshot_recorder_success(value: object) -> tuple[Any, int]:
    """Own one scalar with the same primitive contract as the HDF5 reader."""
    if isinstance(value, bool):
        return value, 1
    if isinstance(value, int):
        return require_bounded_int(
            value,
            resource="recorder success integer",
            minimum=-(1 << 63),
            maximum=(1 << 64) - 1,
        ), 8
    if isinstance(value, np.ndarray):
        # Inspect a base view; array subclasses cannot substitute a conversion.
        scalar = np.ndarray.view(value, np.ndarray)
        dtype = scalar.dtype
        if (
            scalar.shape
            or dtype.kind not in "biu"
            or dtype.hasobject
            or dtype.itemsize > 8
        ):
            raise TypeError("success must be a boolean or integer scalar")
        value = scalar[()]
    if isinstance(value, np.generic):
        dtype = value.dtype
        if dtype.kind in "biu" and not dtype.hasobject and dtype.itemsize <= 8:
            return value, dtype.itemsize
    raise TypeError("success must be a boolean or integer scalar")


def _recorder_json_values_match(left: object, right: object) -> bool:
    """Compare already admitted JSON graphs without Python's bool/int aliasing."""
    if type(left) is not type(right):
        return (
            type(left) in (int, float) and type(right) in (int, float) and left == right
        )
    if isinstance(left, dict) and isinstance(right, dict):
        return left.keys() == right.keys() and all(
            _recorder_json_values_match(value, right[key])
            for key, value in left.items()
        )
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(
            _recorder_json_values_match(a, b) for a, b in zip(left, right, strict=True)
        )
    return left == right


def robomimic_env_args(
    env_name: str,
    env_kwargs: Mapping[str, Any],
    *,
    max_bytes: int | None = None,
) -> str:
    """Build the ``data/env_args`` JSON attribute expected by robomimic.

    Keep separate attributes for SRB readers and validation.
    """

    byte_limit = require_bounded_int(
        MAX_METADATA_ATTRIBUTE_BYTES if max_bytes is None else max_bytes,
        resource="Robomimic env_args byte limit",
        minimum=1,
        maximum=MAX_METADATA_ATTRIBUTE_BYTES,
    )
    return bounded_json_dumps(
        {
            "env_name": env_name,
            "type": ROBOMIMIC_GYM_ENV_TYPE,
            "env_kwargs": env_kwargs,
        },
        context="Robomimic env_args attribute",
        sort_keys=True,
        max_bytes=byte_limit,
    )


def _decode_hdf5_demo_io_contract(value: object) -> dict[str, object]:
    """Decode one bounded HDF5 contract attribute for recorder preflight."""

    if isinstance(value, np.ndarray):
        if value.ndim != 0:
            raise TypeError("HDF5 demo_io_contract attribute must be scalar JSON text")
        value = value.item()
    elif isinstance(value, np.generic):
        value = value.item()

    if isinstance(value, str):
        payload = value.encode("utf-8")
    elif isinstance(value, bytes):
        payload = value
    else:
        raise TypeError("HDF5 demo_io_contract attribute must contain JSON text")
    if len(payload) > _MAX_DEMO_IO_CONTRACT_BYTES:
        raise ValueError("HDF5 demo_io_contract attribute exceeds its size limit")
    try:
        parsed = JsonDecodeBudget().loads(
            payload.decode("utf-8"),
            context="HDF5 demo_io_contract attribute",
            max_bytes=_MAX_DEMO_IO_CONTRACT_BYTES,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("HDF5 demo_io_contract attribute is not valid JSON") from exc
    validated = _validate_demo_io_contract(parsed)
    if validated is None:
        raise ValueError("HDF5 demo_io_contract attribute is null")
    return cast(dict[str, object], validated)


def _encode_hdf5_demo_io_contract(value: Mapping[str, object]) -> np.bytes_:
    """Encode one contract as a bounded fixed-width HDF5 byte scalar."""

    payload = bounded_json_dumps(
        value,
        context="demo_io_contract",
        sort_keys=True,
        max_bytes=_MAX_DEMO_IO_CONTRACT_BYTES,
    ).encode("utf-8")
    if len(payload) > _MAX_DEMO_IO_CONTRACT_BYTES:
        raise ValueError("demo_io_contract exceeds its HDF5 attribute size limit")
    return np.bytes_(payload)


def _read_hdf5_demo_io_contract_attr(attrs: Any) -> dict[str, object] | None:
    """Read a fixed-width contract only after checking HDF5 storage bounds."""

    if "demo_io_contract" not in attrs:
        return None
    try:
        attribute = attrs.get_id("demo_io_contract")
        storage_size = int(attribute.get_storage_size())
        dtype = np.dtype(attribute.dtype)
        shape = tuple(int(dim) for dim in attribute.shape)
    except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
        raise ValueError("cannot inspect HDF5 demo_io_contract attribute") from exc
    if storage_size > _MAX_DEMO_IO_CONTRACT_BYTES:
        raise ValueError("HDF5 demo_io_contract attribute exceeds its size limit")
    if shape or dtype.kind != "S" or dtype.hasobject:
        raise ValueError(
            "HDF5 demo_io_contract attribute must be a fixed-width byte scalar"
        )
    return _decode_hdf5_demo_io_contract(attrs["demo_io_contract"])


def demo_io_contract_sha256(contract: object) -> str | None:
    """Digest a contract so a physical HDF5 copy can be compared remotely.

    Validation compares the native child's digest with the manifest contract.
    """

    if contract is None:
        return None
    import hashlib

    from srb.utils.json_types import thaw_json_value

    payload = json.dumps(
        thaw_json_value(contract),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode("ascii")).hexdigest()


def _hdf5_rows_keep_legacy_batch_axis(data: Any, contract: object) -> bool:
    """Whether the first demo stores ``(T, 1, *shape)`` rows."""

    from srb.datasets.visual_contract import (
        declared_observation_shapes,
        has_legacy_batch_axis,
    )

    shapes = declared_observation_shapes(cast(Mapping[str, Any], contract))
    demo_keys = sorted(str(key) for key in data.keys() if str(key).startswith("demo_"))
    if not shapes or not demo_keys or "obs" not in data[demo_keys[0]]:
        return False
    return any(
        (declared := shapes.get(tuple(path.split("/")))) is not None
        and has_legacy_batch_axis(tuple(dataset.shape[1:]), declared)
        for path, dataset in _iter_hdf5_observation_datasets(data[demo_keys[0]]["obs"])
    )


def _require_hdf5_demo_io_contract_datasets(h5: Any) -> None:
    """Require row fields needed to preserve a certified transition source."""

    data = h5.get("data")
    if data is None:
        raise ValueError("certified HDF5 demos require a data group")
    demo_keys = sorted(str(key) for key in data.keys() if str(key).startswith("demo_"))
    if not demo_keys:
        raise ValueError("certified HDF5 demos contain no demo rows")

    required = (
        "actions",
        "rewards",
        "dones",
        "obs",
        "next_obs",
        "terminated",
        "truncated",
    )
    for demo_key in demo_keys:
        demo = data[demo_key]
        missing = [name for name in required if name not in demo]
        if missing:
            raise ValueError(
                f"certified HDF5 demo {demo_key!r} is missing required "
                f"field(s): {', '.join(missing)}"
            )
        sample_count = _hdf5_first_dim(demo["actions"])
        if sample_count is None:
            raise ValueError(
                f"certified HDF5 demo {demo_key!r} actions has no sample axis"
            )
        for name in ("rewards", "dones", "terminated", "truncated"):
            if _hdf5_first_dim(demo[name]) != sample_count:
                raise ValueError(
                    f"certified HDF5 demo {demo_key!r} {name} length does not "
                    "match actions"
                )
        obs_group = demo.get("obs")
        next_obs_group = demo.get("next_obs")
        if (
            obs_group is None
            or next_obs_group is None
            or not hasattr(obs_group, "keys")
            or not hasattr(next_obs_group, "keys")
        ):
            # ``next_obs`` is already named above; this keeps the error clear
            # if a malformed HDF5 object shadows a required group.
            raise ValueError(
                f"certified HDF5 demo {demo_key!r} requires obs and next_obs groups"
            )
        obs_leaves = dict(_iter_hdf5_observation_datasets(obs_group))
        next_obs_leaves = dict(_iter_hdf5_observation_datasets(next_obs_group))
        if not obs_leaves or set(obs_leaves) != set(next_obs_leaves):
            raise ValueError(
                f"certified HDF5 demo {demo_key!r} obs and next_obs keys differ"
            )
        for key in sorted(obs_leaves):
            if _hdf5_first_dim(obs_leaves[key]) != sample_count:
                raise ValueError(
                    f"certified HDF5 demo {demo_key!r} obs/{key} length does not "
                    "match actions"
                )
            if _hdf5_first_dim(next_obs_leaves[key]) != sample_count:
                raise ValueError(
                    f"certified HDF5 demo {demo_key!r} next_obs/{key} length does "
                    "not match actions"
                )


def _validate_hdf5_observation_key(key: object) -> None:
    """Reject keys that could escape the intended HDF5 observation group."""

    if not isinstance(key, str) or not key:
        raise ValueError("observation mapping keys must be non-empty strings")
    if key.startswith("/") or "/" in key or "\x00" in key or key in {".", ".."}:
        raise ValueError(
            "observation mapping keys must not be absolute, contain '/', NUL, "
            "or dot path components"
        )


def _iter_hdf5_observation_datasets(
    group: Any,
    prefix: str = "",
    *,
    max_entries: int = MAX_DIRECTORY_ENTRIES,
    max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
) -> Iterator[tuple[str, Any]]:
    """Yield nested HDF5 observation leaves as relative paths.

    Reject symbolic/external links before dereference; detect hard-link cycles
    by identity and bound depth and member count by publication ceilings.
    """

    if not hasattr(group, "keys"):
        return
    try:
        hard_link_type = getattr(importlib.import_module("h5py"), "HardLink")
    except (AttributeError, ImportError) as exc:
        raise ValueError(
            "HDF5 observation traversal requires h5py link support"
        ) from exc

    pending: list[tuple[Any, str, int, frozenset[int]]] = [
        (group, prefix, 0, frozenset())
    ]
    member_count = 0
    while pending:
        current, current_prefix, depth, ancestors = pending.pop()
        current_id = _hdf5_object_identity(current)
        if current_id in ancestors:
            raise ValueError(
                "HDF5 observation group contains a repeated hard-link cycle"
            )
        child_ancestors = ancestors | {current_id}
        if depth > max_depth:
            raise ResourceLimitError(
                "HDF5 observation depth",
                limit=max_depth,
                observed=depth,
            )

        raw_keys: list[Any] = []
        for raw_key in current.keys():
            member_count += 1
            if member_count > max_entries:
                raise ResourceLimitError(
                    "HDF5 observation entries",
                    limit=max_entries,
                    observed=member_count,
                )
            raw_keys.append(raw_key)
        for raw_key in sorted(raw_keys, key=str, reverse=True):
            key = str(raw_key)
            path = f"{current_prefix}/{key}" if current_prefix else key
            try:
                link = current.get(raw_key, getlink=True)
            except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"cannot inspect HDF5 observation link {path!r}"
                ) from exc
            if not isinstance(link, hard_link_type):
                raise ValueError(f"HDF5 observation link {path!r} must be a hard link")
            try:
                value = current[raw_key]
            except (KeyError, OSError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"cannot read HDF5 observation member {path!r}"
                ) from exc
            if hasattr(value, "keys") and not hasattr(value, "shape"):
                child_depth = depth + 1
                if child_depth > max_depth:
                    raise ResourceLimitError(
                        "HDF5 observation depth",
                        limit=max_depth,
                        observed=child_depth,
                    )
                pending.append((value, path, child_depth, child_ancestors))
            else:
                yield path, value


def _require_same_hdf5_observation_layout(left: Any, right: Any) -> None:
    """Compare bounded nested declarations without reading observation values."""
    for first, second in zip_longest(
        _iter_hdf5_observation_datasets(left),
        _iter_hdf5_observation_datasets(right),
    ):
        if first is None or second is None:
            raise ValueError("recorder observation layout has different leaf paths")
        first_path, first_dataset = first
        second_path, second_dataset = second
        if (
            first_path != second_path
            or first_dataset.shape[1:] != second_dataset.shape[1:]
        ):
            raise ValueError(
                "recorder observation layout differs at "
                f"{first_path[:128]!r} and {second_path[:128]!r}"
            )


def _preflight_hdf5_storage(
    h5: Any,
    *,
    errors: list[str],
    max_entries: int = MAX_DIRECTORY_ENTRIES,
    max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
) -> bool:
    """Inspect HDF5 storage indirections before any payload access."""

    try:
        h5py = importlib.import_module("h5py")
        hard_link_type = h5py.HardLink
        external_link_type = h5py.ExternalLink
        soft_link_type = h5py.SoftLink
        group_type = h5py.Group
        dataset_type = h5py.Dataset
        datatype_type = h5py.Datatype
    except (AttributeError, ImportError) as exc:
        errors.append(f"HDF5 storage preflight requires h5py link support: {exc}")
        return False

    pending: list[tuple[Any, str, int]] = [(h5, "", 0)]
    seen_objects: set[int] = set()
    member_count = 0
    while pending:
        current, current_path, depth = pending.pop()
        if depth > max_depth:
            errors.append(
                f"HDF5 storage depth exceeds {max_depth} at {current_path or '/'}"
            )
            return False
        current_id = _hdf5_object_identity(current)
        if current_id in seen_objects:
            continue
        seen_objects.add(current_id)

        try:
            raw_keys: list[Any] = []
            for raw_key in current.keys():
                member_count += 1
                if member_count > max_entries:
                    errors.append(
                        "HDF5 storage entries exceed "
                        f"{max_entries} at {current_path or '/'}"
                    )
                    return False
                raw_keys.append(raw_key)
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            errors.append(
                f"cannot inspect HDF5 group {current_path or '/'} before payload access: "
                f"{exc}"
            )
            return False

        for raw_key in sorted(raw_keys, key=str):
            key = str(raw_key)
            path = f"{current_path}/{key}" if current_path else key
            try:
                link = current.get(raw_key, getlink=True)
            except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
                errors.append(f"cannot inspect HDF5 link {path!r}: {exc}")
                return False
            if not isinstance(link, hard_link_type):
                if isinstance(link, external_link_type):
                    link_kind = "external link"
                elif isinstance(link, soft_link_type):
                    link_kind = "symbolic link"
                else:
                    link_kind = "unsupported link"
                errors.append(f"HDF5 {link_kind} {path!r} is not admitted")
                return False
            try:
                value = current[raw_key]
            except (KeyError, OSError, TypeError, ValueError) as exc:
                errors.append(f"cannot read HDF5 member {path!r}: {exc}")
                return False
            if isinstance(value, group_type):
                child_depth = depth + 1
                if child_depth > max_depth:
                    errors.append(f"HDF5 storage depth exceeds {max_depth} at {path}")
                    return False
                pending.append((value, path, child_depth))
                continue
            if not isinstance(value, dataset_type):
                if isinstance(value, datatype_type):
                    continue
                errors.append(f"unsupported HDF5 object {path!r}")
                return False
            try:
                is_virtual = bool(getattr(value, "is_virtual", False))
            except (AttributeError, OSError, TypeError, ValueError) as exc:
                errors.append(f"cannot inspect HDF5 dataset {path!r}: {exc}")
                return False
            if is_virtual:
                errors.append(f"HDF5 virtual dataset {path!r} is not admitted")
                return False
            try:
                external_storage = getattr(value, "external", ())
            except (AttributeError, OSError, TypeError, ValueError) as exc:
                errors.append(f"cannot inspect HDF5 storage {path!r}: {exc}")
                return False
            if external_storage:
                errors.append(f"HDF5 external raw storage {path!r} is not admitted")
                return False
    return True


def _hdf5_object_identity(value: Any) -> int:
    """Return the object identity that survives alternate HDF5 hard-link paths."""

    try:
        h5py = importlib.import_module("h5py")
        info = h5py.h5o.get_info(value.id)
        address = getattr(info, "addr", None)
        filename = getattr(getattr(value, "file", None), "filename", None)
        if isinstance(address, int):
            return hash((str(filename), address))
    except (AttributeError, ImportError, OSError, TypeError, ValueError):
        pass
    object_id = getattr(getattr(value, "id", None), "id", None)
    return int(object_id) if isinstance(object_id, int) else id(value)


def _hdf5_scalar_attribute(attribute: Any) -> bool:
    """Recognize an ordinary scalar attribute without adopting NULL shapes."""

    shape = getattr(attribute, "shape", None)
    if shape is None:
        return False
    try:
        return not tuple(shape)
    except (TypeError, ValueError):
        return False


def _robomimic_train_count(num_demos: int, train_split: float) -> int:
    """Return the train-mask size for ``num_demos``.

    ``train_split`` lies in ``[0, 1]`` and uses integer truncation. A positive
    split with demos keeps at least one train demo to avoid late Robomimic failure.
    """
    if (
        isinstance(train_split, bool)
        or not isinstance(train_split, (int, float))
        or not math.isfinite(train_split)
        or not 0 <= train_split <= 1
    ):
        raise ValueError("train_split must be a finite number within [0, 1]")
    count = int(num_demos * train_split)
    if num_demos > 0 and train_split > 0 and count == 0:
        return 1
    return count


class TrajectoryRecorder:
    """Record environment trajectories to robomimic-compatible HDF5."""

    def __init__(
        self,
        output_path: "str | Path",
        env_name: str,
        obs_keys: "Sequence[str]",
        action_dim: int,
        max_demos: int = 1000,
        max_episode_steps: int = 100_000,
        train_split: float = 0.9,
        env_kwargs: "dict[str, Any] | None" = None,
        action_space: "Any | None" = None,
        dataset_provenance: "Any | None" = None,
        demo_io_contract: "Mapping[str, object] | None" = None,
        *,
        max_decoded_bytes: int | None = None,
        max_batch_bytes: int | None = None,
        max_metadata_bytes: int | None = None,
        max_file_bytes: int | None = None,
        fps: float | None = None,
    ):
        if fps is not None:
            if isinstance(fps, bool) or not isinstance(fps, (int, float)):
                raise TypeError("fps must be a finite positive number")
            if not math.isfinite(fps) or fps <= 0:
                raise ValueError("fps must be a finite positive number")
            fps = float(fps)
        self.fps = fps
        self._recording_fps = fps
        self._max_decoded_bytes = require_bounded_int(
            MAX_DECODED_AGGREGATE_BYTES
            if max_decoded_bytes is None
            else max_decoded_bytes,
            resource="recorder decoded byte limit",
            minimum=1,
            maximum=MAX_DECODED_AGGREGATE_BYTES,
        )
        self._max_batch_bytes = require_bounded_int(
            MAX_IN_MEMORY_FILE_BYTES if max_batch_bytes is None else max_batch_bytes,
            resource="recorder batch byte limit",
            minimum=1,
            maximum=MAX_IN_MEMORY_FILE_BYTES,
        )
        self._max_metadata_bytes = require_bounded_int(
            MAX_METADATA_AGGREGATE_BYTES
            if max_metadata_bytes is None
            else max_metadata_bytes,
            resource="recorder metadata byte limit",
            minimum=1,
            maximum=MAX_METADATA_AGGREGATE_BYTES,
        )
        self._max_file_bytes = require_bounded_int(
            MAX_EXTERNAL_FILE_BYTES if max_file_bytes is None else max_file_bytes,
            resource="recorder file byte limit",
            minimum=1,
            maximum=MAX_EXTERNAL_FILE_BYTES,
        )
        self.output_path = Path(output_path)
        if not isinstance(env_name, str):
            raise TypeError("env_name must be a string")
        self.env_name = env_name
        self._metadata_budget = ByteBudget(
            "recorder retained metadata", self._max_metadata_bytes
        )
        if self._recording_fps is not None:
            self._metadata_budget.consume(32)
        self._json_budget = JsonDecodeBudget()
        if isinstance(obs_keys, (str, bytes, bytearray)):
            raise TypeError("obs_keys must be a sequence of strings")
        self.obs_keys: list[str] = []
        for key in obs_keys:
            _validate_hdf5_observation_key(key)
            require_bounded_int(
                len(self.obs_keys) + 1,
                resource="recorder observation keys",
                maximum=MAX_DIRECTORY_ENTRIES,
            )
            self._metadata_text(key, context="recorder observation key")
            self._metadata_budget.consume(128)
            self.obs_keys.append(key)
        if len(set(self.obs_keys)) != len(self.obs_keys):
            raise ValueError("recorder observation keys must be unique")
        self.action_dim = require_bounded_int(
            action_dim,
            resource="recorder action dimension",
            maximum=MAX_ARRAY_ELEMENTS,
        )
        self.max_demos = require_bounded_int(
            max_demos,
            resource="recorder demo limit",
            maximum=MAX_DIRECTORY_ENTRIES,
        )
        if max_episode_steps <= 0:
            raise ValueError("max_episode_steps must be positive")
        self.max_episode_steps = require_bounded_int(
            max_episode_steps,
            resource="recorder episode step limit",
            minimum=1,
            maximum=MAX_EVENT_ROWS,
        )
        if isinstance(train_split, bool) or not isinstance(train_split, (int, float)):
            raise TypeError("train_split must be a finite number between 0 and 1")
        if not math.isfinite(train_split) or not 0 <= train_split <= 1:
            raise ValueError("train_split must be a finite number between 0 and 1")
        self.train_split = float(train_split)

        from srb.datasets.provenance import DatasetProvenance, coerce_dataset_provenance

        if env_kwargs is not None and not isinstance(env_kwargs, Mapping):
            raise TypeError("env_kwargs must be a mapping")
        self._metadata_snapshot(env_name, context="recorder environment name")
        kwargs_snapshot = self._metadata_snapshot(
            env_kwargs or {},
            context="recorder environment kwargs",
        )
        if isinstance(dataset_provenance, DatasetProvenance):
            extra = self._metadata_snapshot(
                dataset_provenance.extra,
                context="recorder provenance extra",
            )
            dataset_provenance = replace(dataset_provenance, extra=extra)
        elif dataset_provenance is not None:
            dataset_provenance = self._metadata_snapshot(
                dataset_provenance,
                context="recorder provenance",
            )

        self.dataset_provenance = coerce_dataset_provenance(
            dataset_provenance,
            env_name=env_name,
            env_kwargs=kwargs_snapshot,
        )
        self.env_kwargs = (
            self.dataset_provenance.to_env_kwargs()
            if dataset_provenance is not None and env_kwargs is None
            else kwargs_snapshot
        )
        self.action_space: Any | None = None
        if action_space is not None:
            self.action_space, action_metadata_bytes = snapshot_action_space(
                action_space,
                max_metadata_bytes=self._metadata_budget.limit
                - self._metadata_budget.consumed,
            )
            self._metadata_budget.consume(action_metadata_bytes)

        from srb.core.env.runtime.action_space import (
            action_schema_for_space,
            flat_action_schema,
        )

        self.action_schema = (
            action_schema_for_space(self.action_space)
            if self.action_space is not None
            else flat_action_schema(action_dim)
        )
        contract_snapshot = (
            self._metadata_snapshot(
                demo_io_contract, context="recorder demo_io_contract"
            )
            if demo_io_contract is not None
            else None
        )
        self.demo_io_contract = _validate_demo_io_contract(contract_snapshot)
        self._demo_io_contract_hdf5 = (
            _encode_hdf5_demo_io_contract(self.demo_io_contract)
            if self.demo_io_contract is not None
            else None
        )
        if self.demo_io_contract is not None:
            self._validate_demo_io_contract_recorder_fields()

        self._metadata_payloads = {
            "env_name": self.env_name,
            "env_kwargs": self._metadata_text(
                self.env_kwargs, context="recorder env_kwargs"
            ),
            "env_args": self._metadata_text(
                {
                    "env_name": self.env_name,
                    "type": ROBOMIMIC_GYM_ENV_TYPE,
                    "env_kwargs": self.env_kwargs,
                },
                context="recorder env_args",
                sort_keys=True,
            ),
            "action_schema": self._metadata_text(
                self.action_schema, context="recorder action_schema", sort_keys=True
            ),
        }
        self._metadata_budget.consume(len(self.env_name.encode("utf-8")))
        if self._demo_io_contract_hdf5 is not None:
            self._metadata_budget.consume(self._demo_io_contract_hdf5.nbytes)
        provenance_snapshot = self._metadata_snapshot(
            self.dataset_provenance.to_env_kwargs(),
            context="recorder sidecar provenance",
        )
        self._recording_provenance = coerce_dataset_provenance(
            provenance_snapshot, env_name=env_name
        )
        self._recording_obs_keys = tuple(self.obs_keys)
        self._recording_action_dim = self.action_dim
        self._recording_action_space = self.action_space
        self._recording_contract = (
            self._metadata_snapshot(
                self.demo_io_contract, context="recorder captured contract"
            )
            if self.demo_io_contract is not None
            else None
        )

        self._episode: TrajectoryBuffer | None = None
        self._episode_recording_payload: str | None = None
        self._demo_count = 0
        self._total_samples = 0
        self._pending_publication_cleanup: PublicationCleanupError | None = None

        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def _require_publication_cleanup_complete(self) -> None:
        pending = self._pending_publication_cleanup
        if pending is not None:
            raise pending

    def _retain_publication_cleanup(self, error: PublicationCleanupError) -> None:
        owner = _RobomimicCleanupOwner(
            (error.owner.retry_cleanup,),
            on_complete=self._clear_publication_cleanup,
        )
        error.owner = owner
        error.cleanup_owner = owner
        self._pending_publication_cleanup = error

    def _clear_publication_cleanup(self) -> None:
        self._pending_publication_cleanup = None

    def _metadata_snapshot(self, value: object, *, context: str) -> Any:
        """Admit wire size and structure before retaining a private JSON graph."""
        payload = self._metadata_text(value, context=context)
        return self._json_budget.loads(payload, context=context)

    def _metadata_text(
        self, value: object, *, context: str, sort_keys: bool = False
    ) -> str:
        payload = bounded_json_dumps(
            value,
            context=context,
            sort_keys=sort_keys,
            max_bytes=min(
                MAX_METADATA_ATTRIBUTE_BYTES,
                self._metadata_budget.limit - self._metadata_budget.consumed,
            ),
        )
        self._metadata_budget.consume(len(payload))
        return payload

    def start_episode(
        self, *, episode_recording: Mapping[str, object] | None = None
    ) -> None:
        """Start recording a new episode and optionally bind sidecar artifacts."""
        self._require_publication_cleanup_complete()
        self._episode_recording_payload = _encode_episode_recording_reference(
            episode_recording
        )
        self._episode = TrajectoryBuffer(
            obs_keys=self._recording_obs_keys,
            action_dim=self._recording_action_dim,
            action_space=self._recording_action_space,
            max_episode_steps=self.max_episode_steps,
            max_decoded_bytes=self._max_decoded_bytes,
            max_batch_bytes=self._max_batch_bytes,
            max_metadata_bytes=self._max_metadata_bytes
            - self._metadata_budget.consumed,
        )

    def record_step(
        self,
        obs: "dict[str, np.ndarray]",
        action: np.ndarray,
        reward: float,
        done: bool,
        next_obs: "dict[str, np.ndarray]",
        *,
        terminated: bool | None = None,
        truncated: bool | None = None,
    ) -> None:
        """Record a single environment step.

        ``terminated``/``truncated`` carry the Gymnasium distinction to disk;
        when omitted the legacy ``done`` collapses to
        ``terminated=done, truncated=False``.
        """
        self._require_publication_cleanup_complete()
        if self._episode is None:
            raise RuntimeError("start_episode() must be called before record_step()")
        if self._recording_contract is not None and (
            type(terminated) is not bool or type(truncated) is not bool
        ):
            raise ValueError(
                "demo_io_contract requires explicit terminated and truncated "
                "flags as genuine boolean values on every recorded row"
            )
        if self._recording_contract is not None and not isinstance(next_obs, Mapping):
            raise ValueError(
                "demo_io_contract requires an explicit next_obs mapping on every "
                "recorded row"
            )

        self._episode.append(
            obs,
            action,
            reward,
            done,
            next_obs,
            terminated=terminated,
            truncated=truncated,
        )

    def save_episode(self, success: bool = True) -> bool:
        """Publish the current episode; retain it for retry if writing fails."""
        self._require_publication_cleanup_complete()
        if self._episode is None or self._episode.num_steps == 0:
            return False

        if self._demo_count >= self.max_demos:
            raise ResourceLimitError(
                "recorder demo capacity",
                limit=self.max_demos,
                observed=self._demo_count + 1,
            )

        traj_len = self._episode.num_steps

        success, success_bytes = _snapshot_recorder_success(success)

        from srb.datasets.manifest import validate_dataset_manifest_output_path
        from srb.utils.publication import (
            FilePublication,
            PublicationCleanupError,
            stage_file_publications,
        )

        validate_dataset_manifest_output_path(self.output_path)
        manifest_path = validate_dataset_manifest_output_path(
            self.output_path.with_suffix(".srb.json")
        )
        h5py = _require_h5py()
        committed_state: tuple[int, int] | None = None
        demo_name = f"demo_{self._demo_count}"
        try:
            with stage_file_publications(
                (FilePublication(self.output_path), FilePublication(manifest_path))
            ) as (stage, _manifest_stage):
                existed = stage.copy_current(max_bytes=self._max_file_bytes)
                metadata: dict[str, str | bytes] = {}
                existing_demo_keys: list[str] = []
                demo_count, previous_samples = self._demo_count, 0
                decoded_budget = ByteBudget(
                    "recorder dataset decoded bytes", self._max_decoded_bytes
                )
                decoded_budget.consume(self._episode.decoded_nbytes)
                decoded_budget.consume(self._episode.write_temporary_nbytes)
                metadata_budget = ByteBudget(
                    "recorder dataset metadata bytes", self._max_metadata_bytes
                )
                metadata_budget.consume(self._metadata_budget.consumed)
                metadata_budget.consume(self._episode.metadata_nbytes)
                # Reserve fixed metadata for num_samples, success and total_samples.
                metadata_budget.consume(3 * 128 + 16 + success_bytes)
                if self._episode_recording_payload is not None:
                    metadata_budget.consume(
                        128 + len(self._episode_recording_payload.encode("utf-8"))
                    )
                self._preflight_recording_sidecar(manifest_path, metadata_budget)
                if existed:
                    existing_demo_keys, demo_count, previous_samples, metadata = (
                        self._inspect_recording(
                            stage.path,
                            decoded_budget=decoded_budget,
                            metadata_budget=metadata_budget,
                            validate_masks=True,
                        )
                    )
                for name, payload in self._metadata_payloads.items():
                    if name not in metadata:
                        metadata_budget.consume(128 + len(payload.encode("utf-8")))
                if self._demo_io_contract_hdf5 is not None:
                    metadata_budget.consume(128 + self._demo_io_contract_hdf5.nbytes)
                with h5py.File(stage.path, "a" if existed else "w") as f:
                    # Certified recordings may extend only equally certified files;
                    # legacy rows cannot acquire a contract after the fact.
                    self._preflight_demo_io_contract(f)
                    if demo_count >= self.max_demos:
                        raise ResourceLimitError(
                            "recorder demo capacity",
                            limit=self.max_demos,
                            observed=demo_count + 1,
                        )
                    if "data" not in f:
                        f.create_group("data")

                    demo_name = f"demo_{demo_count}"
                    # A failed write discards the private stage. Never treat an
                    # existing pending name as permission to delete input data.
                    pending_name = f"_pending_{demo_name}"
                    if pending_name in f["data"]:
                        raise ValueError(
                            f"recorder pending group already exists: {pending_name!r}"
                        )
                    demo_grp = f["data"].create_group(pending_name)
                    self._write_demo(
                        f,
                        demo_grp,
                        traj_len=traj_len,
                        success=success,
                        total_samples=previous_samples + traj_len,
                        existing_metadata=metadata,
                        episode_recording_payload=self._episode_recording_payload,
                    )
                    _require_same_hdf5_observation_layout(
                        demo_grp["obs"], demo_grp["next_obs"]
                    )
                    if existing_demo_keys:
                        _require_same_hdf5_observation_layout(
                            f["data"][existing_demo_keys[0]]["obs"], demo_grp["obs"]
                        )
                    if (
                        self._recording_contract is not None
                        and "demo_io_contract" not in f["data"].attrs
                    ):
                        contract_bytes = self._demo_io_contract_hdf5
                        if contract_bytes is None:
                            raise RuntimeError(
                                "contract recorder lost its encoded demo_io_contract"
                            )
                        f["data"].attrs["demo_io_contract"] = contract_bytes
                    f["data"].move(pending_name, demo_name)
                committed_state = (demo_count + 1, previous_samples + traj_len)
                stage.publish(max_bytes=self._max_file_bytes)
                self._demo_count, self._total_samples = committed_state
                self._episode = None
                self._episode_recording_payload = None
        except PublicationCleanupError as error:
            if committed_state is not None and _publication_committed_path(
                error, self.output_path
            ):
                self._demo_count, self._total_samples = committed_state
                self._episode = None
                self._episode_recording_payload = None
            self._retain_publication_cleanup(error)
            raise
        print(
            f"[INFO] Saved demo {demo_name} with {traj_len} samples "
            f"(total: {self._demo_count} demos, {self._total_samples} samples)"
        )
        return True

    def _write_demo(
        self,
        f: Any,
        demo_grp: Any,
        *,
        traj_len: int,
        success: bool,
        total_samples: int,
        existing_metadata: Mapping[str, str | bytes],
        episode_recording_payload: str | None,
    ) -> None:
        """Write one demo's datasets and attributes into ``demo_grp``."""

        if self._episode is None:
            raise RuntimeError("recorded episode disappeared before writing")
        self._episode.write(demo_grp)

        demo_grp.attrs["num_samples"] = traj_len
        demo_grp.attrs["success"] = success
        if episode_recording_payload is not None:
            demo_grp.attrs[_EPISODE_RECORDING_ATTRIBUTE] = np.bytes_(
                episode_recording_payload.encode("utf-8")
            )

        for name, payload in self._metadata_payloads.items():
            if name not in existing_metadata:
                f["data"].attrs[name] = payload
        f["data"].attrs["total_samples"] = total_samples

    def _preflight_demo_io_contract(self, h5: Any) -> None:
        """Validate certification compatibility before changing an HDF5 file."""

        data = h5.get("data")
        if data is None:
            return

        existing = _read_hdf5_demo_io_contract_attr(data.attrs)
        has_existing_rows = any(str(key).startswith("demo_") for key in data.keys())
        if self._recording_contract is None:
            if existing is not None:
                raise ValueError(
                    "cannot append uncertified rows to an HDF5 file with a "
                    "demo_io_contract"
                )
            return

        if existing is None:
            if has_existing_rows:
                raise ValueError(
                    "cannot append a certified demo_io_contract to an existing "
                    "legacy HDF5 file"
                )
            return

        if existing != self._recording_contract:
            raise ValueError(
                "HDF5 demo_io_contract does not match the recorder contract"
            )
        if has_existing_rows and _hdf5_rows_keep_legacy_batch_axis(data, existing):
            raise ValueError(
                "cannot append contract-shape rows to an HDF5 file whose rows "
                "keep the legacy single-environment axis; record into a "
                "new file or re-export this one first"
            )
        if has_existing_rows:
            _require_hdf5_demo_io_contract_datasets(h5)

    def _validate_demo_io_contract_recorder_fields(self) -> None:
        """Ensure the recorder dimensions and keys match its certification."""

        # Constructor validation already establishes the nested contract shape.
        contract = cast(dict[str, Any] | None, self.demo_io_contract)
        if contract is None:
            return
        observation = contract["observation"]
        declared_obs_keys = observation["keys"]
        if declared_obs_keys != self.obs_keys:
            raise ValueError(
                "demo_io_contract observation keys do not match recorder obs_keys"
            )

        action = contract["action"]
        schema = action["schema"]
        schema_dim = sum(int(term["dim"]) for term in schema)
        declared_action_dim = action.get("action_dim")
        if schema_dim != self.action_dim or (
            declared_action_dim is not None
            and int(declared_action_dim) != self.action_dim
        ):
            raise ValueError(
                "demo_io_contract action dimension does not match recorder action_dim"
            )

    def _inspect_recording(
        self,
        path: Path,
        *,
        decoded_budget: ByteBudget,
        metadata_budget: ByteBudget,
        validate_masks: bool = False,
    ) -> tuple[list[str], int, int, dict[str, str | bytes]]:
        """Inspect the owned current file before a writable HDF5 handle exists."""
        h5py = _require_h5py()
        from srb.datasets.hdf5_metadata import Hdf5MetadataLimitError

        require_dataset_file_size(
            path,
            resource="recorder current HDF5",
            max_bytes=self._max_file_bytes,
        )
        errors: list[str] = []
        with h5py.File(path, "r") as current:
            if not _preflight_hdf5_storage(current, errors=errors):
                raise ValueError(
                    "cannot use invalid recorder HDF5: " + "; ".join(errors)
                )
            try:
                text = _preflight_hdf5_metadata_storage(
                    current,
                    path=path,
                    max_aggregate_bytes=self._max_metadata_bytes,
                    metadata_budget=metadata_budget,
                )
            except Hdf5MetadataLimitError as exc:
                raise ResourceLimitError(
                    exc.resource,
                    limit=exc.limit,
                    observed=exc.observed,
                ) from exc
            data = current.get("data")
            if data is None:
                return [], self._demo_count, 0, text
            if not isinstance(data, h5py.Group):
                raise ValueError("recorder HDF5 data must be a group")
            demo_keys: list[str] = []
            next_index = self._demo_count
            for key in data.keys():
                metadata_budget.consume(len(key.encode("utf-8")) + 128)
                demo_keys.append(key)
                if key.startswith("demo_") and key[5:].isascii() and key[5:].isdigit():
                    suffix = key[5:].lstrip("0") or "0"
                    if len(suffix) > len(str(self.max_demos)):
                        raise ResourceLimitError(
                            "recorder demo capacity",
                            limit=self.max_demos,
                            observed=self.max_demos + 1,
                        )
                    index = int(suffix)
                    require_bounded_int(
                        index,
                        resource="recorder demo index",
                        maximum=MAX_DIRECTORY_ENTRIES,
                    )
                    next_index = max(next_index, index + 1)
            demo_keys.sort()
            # The owned byte copy preserves opaque fields without decoding them.
            # Charge every fixed-width numeric array and fixed mask before the
            # semantic validator reads any termination payload.
            pending = [current]
            seen: set[int] = set()
            while pending:
                value = pending.pop()
                identity = _hdf5_object_identity(value)
                if identity in seen:
                    continue
                seen.add(identity)
                if isinstance(value, h5py.Group):
                    pending.extend(value[key] for key in value.keys())
                elif isinstance(value, h5py.Dataset):
                    dtype = np.dtype(value.dtype)
                    if (
                        not dtype.hasobject
                        and dtype.kind in "biufS"
                        and dtype.fields is None
                        and dtype.subdtype is None
                    ):
                        decoded_budget.consume(
                            checked_shape_nbytes(
                                value.shape,
                                dtype,
                                resource="recorder existing decoded array",
                            )
                        )
            _, total_samples = _validate_robomimic_demos(
                h5=current,
                demo_keys=demo_keys,
                errors=errors,
                max_batch_bytes=min(
                    self._max_batch_bytes,
                    decoded_budget.limit - decoded_budget.consumed,
                ),
                decoded_budget=ByteBudget(
                    "recorder validation decoded bytes", self._max_decoded_bytes
                ),
            )
            if errors:
                raise ValueError(
                    "cannot use invalid recorder HDF5: " + "; ".join(errors)
                )
            self._require_recording_compatibility(current, demo_keys, text)
            if validate_masks and "mask" in current:
                _validate_robomimic_masks(
                    h5=current,
                    demo_keys=set(demo_keys),
                    errors=errors,
                    decoded_budget=decoded_budget,
                    raw_budget_accounted=True,
                    max_batch_bytes=self._max_batch_bytes,
                )
                if errors:
                    raise ValueError(
                        "cannot use invalid recorder HDF5: " + "; ".join(errors)
                    )
            self._preflight_demo_io_contract(current)
            return demo_keys, next_index, total_samples, text

    def _require_recording_compatibility(
        self,
        current: Any,
        demo_keys: Sequence[str],
        text: Mapping[str, str | bytes],
    ) -> None:
        """Reject known configuration differences before modifying the stage."""
        json_budget = JsonDecodeBudget()
        for name, existing in text.items():
            expected = self._metadata_payloads[name]
            if name == "env_name":
                value = (
                    existing.decode("utf-8")
                    if isinstance(existing, bytes)
                    else existing
                )
                compatible = value == expected
            else:
                compatible = _recorder_json_values_match(
                    json_budget.loads(existing, context=f"existing recorder {name}"),
                    json_budget.loads(expected, context=f"recorder {name}"),
                )
            if not compatible:
                raise ValueError(
                    f"HDF5 {name} does not match the recorder configuration"
                )
        for key in demo_keys:
            demo = current["data"][key]
            if _robomimic_action_dim(demo) != self._recording_action_dim:
                raise ValueError(
                    f"data/{key} action dimension does not match the recorder"
                )
            for group in ("obs", "next_obs"):
                if group in demo and set(demo[group].keys()) != set(
                    self._recording_obs_keys
                ):
                    raise ValueError(
                        f"data/{key}/{group} roots do not match recorder obs_keys"
                    )
            if "next_obs" in demo:
                _require_same_hdf5_observation_layout(demo["obs"], demo["next_obs"])
            _require_same_hdf5_observation_layout(
                current["data"][demo_keys[0]]["obs"], demo["obs"]
            )

    def discard_episode(self) -> None:
        """Discard the current episode without saving."""
        self._episode = None
        self._episode_recording_payload = None

    def _preflight_recording_sidecar(
        self, manifest_path: Path, metadata_budget: ByteBudget
    ) -> None:
        """Check present sidecar identity while the grouped writer lease is held."""
        from srb.datasets.manifest import DatasetManifest, resolve_dataset_path
        from srb.datasets.schema import StorageFormat
        from srb.utils.publication import (
            ArtifactPublication,
            PublicationResourceLimitError,
        )

        byte_limit = min(
            MAX_METADATA_ATTRIBUTE_BYTES,
            metadata_budget.limit - metadata_budget.consumed,
        )
        if byte_limit == 0:
            raise ResourceLimitError(
                "recorder sidecar metadata bytes", limit=0, observed=1
            )
        try:
            payload = ArtifactPublication.file(manifest_path).read(
                lambda raw: raw, max_bytes=byte_limit
            )
        except PublicationResourceLimitError as exc:
            raise ResourceLimitError(
                "recorder sidecar metadata bytes",
                limit=exc.limit,
                observed=exc.observed,
            ) from exc
        if payload is None:
            return
        metadata_budget.consume(len(payload))
        decoder = JsonDecodeBudget()
        value = decoder.loads(payload, context="recorder existing sidecar")
        manifest = DatasetManifest.from_dict(value)
        provenance = self._recording_provenance
        target = resolve_dataset_path(manifest.dataset_path, manifest_path)
        if (
            manifest.storage_format != StorageFormat.ROBOMIMIC_HDF5
            or os.path.abspath(target) != os.path.abspath(self.output_path)
            or manifest.identity != provenance.identity()
            or manifest.split.value != provenance.split
            or manifest.controller != provenance.controller
        ):
            raise ValueError("SRB sidecar identity does not match the recorder")
        metadata = value.get("metadata", {})
        expected = {
            "env_name": self._metadata_payloads["env_name"],
            "env_kwargs": decoder.loads(self._metadata_payloads["env_kwargs"]),
            "action_schema": decoder.loads(self._metadata_payloads["action_schema"]),
            "obs_keys": list(self._recording_obs_keys),
            "action_dim": self._recording_action_dim,
            "provenance": provenance.to_env_kwargs(),
            "demo_io_contract": self._recording_contract,
        }
        if self._recording_fps is not None:
            expected["fps"] = self._recording_fps
        for name, expected_value in expected.items():
            if name in metadata and not _recorder_json_values_match(
                metadata[name], expected_value
            ):
                raise ValueError(
                    f"SRB sidecar {name} does not match the recorder configuration"
                )

    def finalize(self) -> None:
        """Publish train/valid masks, then publish the separate SRB sidecar."""
        self._require_publication_cleanup_complete()
        if not self.output_path.exists() and not self.output_path.is_symlink():
            print("[WARNING] No data recorded. Skipping finalization.")
            return

        from contextlib import contextmanager

        from srb.datasets.manifest import validate_dataset_manifest_output_path
        from srb.utils.publication import (
            FilePublication,
            PublicationCleanupError,
            stage_file_publications,
        )

        manifest_path = validate_dataset_manifest_output_path(
            self.output_path.with_suffix(".srb.json"),
        )
        validate_dataset_manifest_output_path(self.output_path)

        h5py = _require_h5py()
        committed_state: tuple[int, int] | None = None

        @contextmanager
        def publication_stages() -> Iterator[Any]:
            try:
                with stage_file_publications(
                    (
                        FilePublication(self.output_path),
                        FilePublication(manifest_path),
                    )
                ) as stages:
                    yield stages
            except PublicationCleanupError as error:
                if committed_state is not None and _publication_committed_path(
                    error, self.output_path
                ):
                    self._demo_count, self._total_samples = committed_state
                self._retain_publication_cleanup(error)
                raise

        with publication_stages() as (stage, manifest_stage):
            if not stage.copy_current(max_bytes=self._max_file_bytes):
                return
            decoded_budget = ByteBudget(
                "recorder dataset decoded bytes", self._max_decoded_bytes
            )
            metadata_budget = ByteBudget(
                "recorder dataset metadata bytes", self._max_metadata_bytes
            )
            metadata_budget.consume(self._metadata_budget.consumed)
            metadata_budget.consume(3 * (128 + 8))  # Final split/count attributes.
            self._preflight_recording_sidecar(manifest_path, metadata_budget)
            if self._episode is not None:
                metadata_budget.consume(self._episode.metadata_nbytes)
                decoded_budget.consume(self._episode.decoded_nbytes)
            demo_keys, demo_count, total_samples, _ = self._inspect_recording(
                stage.path,
                decoded_budget=decoded_budget,
                metadata_budget=metadata_budget,
            )
            if not demo_keys:
                return
            n_train = _robomimic_train_count(len(demo_keys), self.train_split)
            mask_plans: list[tuple[str, int, int, np.dtype[Any]]] = []
            for name, start, stop in (
                ("train", 0, n_train),
                ("valid", n_train, len(demo_keys)),
            ):
                width = max(
                    (
                        len(demo_keys[index].encode("utf-8"))
                        for index in range(start, stop)
                    ),
                    default=1,
                )
                dtype = np.dtype(f"S{width}")
                decoded_budget.consume(
                    checked_shape_nbytes(
                        (stop - start,),
                        dtype,
                        resource=f"recorder {name} mask",
                    )
                )
                mask_plans.append((name, start, stop, dtype))
            headroom = min(
                self._max_batch_bytes, decoded_budget.limit - decoded_budget.consumed
            )
            for name, start, stop, dtype in mask_plans:
                if stop > start and headroom < dtype.itemsize:
                    raise ResourceLimitError(
                        f"recorder {name} mask batch bytes",
                        limit=headroom,
                        observed=dtype.itemsize,
                    )
            manifest_payload = self._srb_dataset_manifest_bytes(
                num_demos=len(demo_keys),
                num_train=n_train,
                num_valid=len(demo_keys) - n_train,
                total_samples=total_samples,
                metadata_budget=metadata_budget,
            )
            with h5py.File(stage.path, "a") as f:
                if "mask" not in f:
                    f.create_group("mask")
                for name, start, stop, dtype in mask_plans:
                    if name in f["mask"]:
                        del f["mask"][name]
                    dataset = f["mask"].create_dataset(
                        name, shape=(stop - start,), dtype=dtype
                    )
                    batch_rows = max(
                        1, min(MAX_EVENT_BATCH_ROWS, headroom // dtype.itemsize)
                    )
                    if start < stop:
                        batch = np.empty(min(batch_rows, stop - start), dtype=dtype)
                        for offset in range(start, stop, batch_rows):
                            count = min(batch_rows, stop - offset)
                            for index in range(count):
                                batch[index] = demo_keys[offset + index].encode("utf-8")
                            dataset[offset - start : offset - start + count] = batch[
                                :count
                            ]
                        del batch
                f["data"].attrs["num_demos"] = len(demo_keys)
                f["data"].attrs["num_train"] = n_train
                f["data"].attrs["num_valid"] = len(demo_keys) - n_train
            committed_state = (demo_count, total_samples)
            stage.publish(max_bytes=self._max_file_bytes)
            self._demo_count, self._total_samples = committed_state
            manifest_stage.path.write_bytes(manifest_payload)
            manifest_stage.publish(max_bytes=MAX_METADATA_ATTRIBUTE_BYTES)
        print(
            f"\n[INFO] Dataset finalized: {self.output_path}"
            f"\n  Total demos: {len(demo_keys)}"
            f"\n  Training: {n_train}"
            f"\n  Validation: {len(demo_keys) - n_train}"
            f"\n  Total samples: {self._total_samples}"
        )

    def _srb_dataset_manifest_bytes(
        self,
        *,
        num_demos: int,
        num_train: int,
        num_valid: int,
        total_samples: int,
        metadata_budget: ByteBudget,
    ) -> bytes:
        """Admit and encode the separate SRB sidecar before HDF5 promotion."""
        from srb.datasets.manifest import (
            build_dataset_manifest,
            dataset_manifest_bytes,
            dataset_root_from_manifest_path,
        )
        from srb.datasets.schema import DatasetSplit, StorageFormat

        provenance = self._recording_provenance
        split = DatasetSplit(provenance.split)

        json_budget = JsonDecodeBudget()
        metadata: dict[str, object] = {
            "env_name": self._metadata_payloads["env_name"],
            "env_kwargs": json_budget.loads(
                self._metadata_payloads["env_kwargs"],
                context="recorder sidecar env_kwargs",
            ),
            "provenance": provenance.to_env_kwargs(),
            "obs_keys": self._recording_obs_keys,
            "action_dim": self._recording_action_dim,
            "action_schema": json_budget.loads(
                self._metadata_payloads["action_schema"],
                context="recorder sidecar action_schema",
            ),
            "num_demos": num_demos,
            "num_train": num_train,
            "num_valid": num_valid,
            "total_samples": total_samples,
            "train_split": self.train_split,
        }
        if self._recording_fps is not None:
            metadata["fps"] = self._recording_fps
        if self._recording_contract is not None:
            metadata["demo_io_contract"] = self._recording_contract

        # Store the data path relative to the sidecar's dataset root so the run
        # directory stays valid after it is moved or copied out of a container.
        manifest_path = self.output_path.with_suffix(".srb.json")
        manifest_dataset_path = self.output_path.relative_to(
            dataset_root_from_manifest_path(manifest_path)
        )
        metadata_text = bounded_json_dumps(
            metadata,
            context="recorder sidecar metadata",
            max_bytes=min(
                MAX_METADATA_ATTRIBUTE_BYTES,
                metadata_budget.limit - metadata_budget.consumed,
            ),
        )
        metadata_budget.consume(len(metadata_text))
        owned_metadata = JsonDecodeBudget().loads(
            metadata_text, context="recorder sidecar metadata"
        )
        manifest = build_dataset_manifest(
            identity=provenance.identity(),
            dataset_path=manifest_dataset_path,
            split=split,
            controller=provenance.controller,
            storage_format=StorageFormat.ROBOMIMIC_HDF5,
            metadata=owned_metadata,
        )
        payload = dataset_manifest_bytes(
            manifest,
            max_bytes=min(
                MAX_METADATA_ATTRIBUTE_BYTES,
                metadata_budget.limit - metadata_budget.consumed,
            ),
        )
        metadata_budget.consume(len(payload))
        return payload

    @property
    def demo_count(self) -> int:
        """Return the number of recorded demos."""
        return self._demo_count

    @property
    def total_samples(self) -> int:
        """Return the total number of recorded samples."""
        return self._total_samples


def merge_datasets(
    input_paths: "Sequence[str | Path]",
    output_path: "str | Path",
    train_split: float = 0.9,
    *,
    native_limits: Hdf5NativeLimits | None = None,
) -> Path:
    """Merge datasets with native copying isolated behind lowering-only limits."""
    from srb.datasets.hdf5_native import (
        Hdf5NativeCleanupError,
        Hdf5NativeError,
        Hdf5NativeLimits,
        merge_private_hdf5,
    )
    from srb.utils.publication import (
        ArtifactPublication,
        OwnedTemporaryDirectory,
        PublicationCleanupError,
        PublicationCommitCleanupError,
        materialize_file_snapshot,
        sha256_file,
    )

    limits = Hdf5NativeLimits() if native_limits is None else native_limits
    if not isinstance(limits, Hdf5NativeLimits):
        raise TypeError("native_limits must be Hdf5NativeLimits or None")
    if isinstance(train_split, bool) or not isinstance(train_split, (int, float)):
        raise TypeError("robomimic merge train_split must be finite")
    if not math.isfinite(train_split) or not 0 <= train_split <= 1:
        raise ValueError(
            "robomimic merge train_split must be a finite number within [0, 1]"
        )
    limits = replace(
        limits,
        max_array_bytes=min(limits.max_array_bytes, MAX_ARRAY_BYTES),
        max_array_elements=min(limits.max_array_elements, MAX_ARRAY_ELEMENTS),
        max_decoded_bytes=min(limits.max_decoded_bytes, MAX_DECODED_AGGREGATE_BYTES),
        max_entries=min(limits.max_entries, MAX_DIRECTORY_ENTRIES),
        max_metadata_bytes=min(limits.max_metadata_bytes, MAX_METADATA_AGGREGATE_BYTES),
        max_attribute_bytes=min(
            limits.max_attribute_bytes, MAX_METADATA_ATTRIBUTE_BYTES
        ),
        max_attributes=min(limits.max_attributes, MAX_METADATA_ATTRIBUTES),
        max_source_bytes=min(
            limits.max_source_bytes,
            MAX_EXTERNAL_FILE_BYTES,
            MAX_PUBLICATION_DIRECTORY_BYTES,
        ),
    )

    require_bounded_int(
        len(input_paths),
        resource="robomimic merge input entries",
        maximum=limits.max_entries,
    )
    resolved_inputs = [Path(input_path) for input_path in input_paths]
    output_path = Path(output_path)
    wire_budget = ByteBudget("robomimic merge input bytes", limits.max_source_bytes)
    members = {
        f"input_{index:06d}.hdf5": (
            path,
            sha256_file(path, byte_budget=wire_budget),
        )
        for index, path in enumerate(resolved_inputs)
    }

    snapshot = materialize_file_snapshot(
        members,
        max_bytes=limits.max_source_bytes,
        max_entries=limits.max_entries,
    )
    cleanup_retained = False
    committed_output: Path | None = None
    private_owner: OwnedTemporaryDirectory | None = None
    operation_completed = False
    try:
        private_owner = OwnedTemporaryDirectory.create(prefix="srb-robomimic-merge-")
        result = merge_private_hdf5(
            [(snapshot.path / name, digest) for name, (_, digest) in members.items()],
            private_owner,
            train_split=train_split,
            limits=limits,
        )
        if result.output_path is None:
            raise Hdf5NativeError("native HDF5 merge returned no private output")
        try:
            demo_count, total_samples = result.demo_count, result.total_samples
            committed_output = ArtifactPublication.file(output_path).publish_staged(
                result.output_path,
                expected_sha256=result.output_sha256,
                max_bytes=limits.max_output_bytes,
            )
        except (PublicationCommitCleanupError, PublicationCleanupError) as error:
            cleanup_retained = True
            callbacks: list[Callable[[], object]] = [snapshot.close]
            if private_owner is not None:
                callbacks.insert(0, private_owner.cleanup)
            owner = _RobomimicCleanupOwner((error.owner.retry_cleanup, *callbacks))
            error.owner = owner
            error.cleanup_owner = owner
            raise
        operation_completed = True
    except Hdf5NativeCleanupError as error:
        cleanup_retained = True
        snapshot.defer_finalizer_cleanup()
        native_callbacks: list[Callable[[], object]] = [snapshot.close]
        if private_owner is not None:
            native_callbacks.insert(0, private_owner.cleanup)
        error.retain_cleanup(*native_callbacks)
        raise
    finally:
        if not cleanup_retained:
            primary_error = sys.exc_info()[1] if not operation_completed else None
            cleanup_operations: list[Callable[[], object]] = []
            if private_owner is not None:
                cleanup_operations.append(private_owner.cleanup)
            cleanup_operations.append(snapshot.close)
            cleanup_errors: list[BaseException] = []
            for cleanup_operation in cleanup_operations:
                _result, cleanup_error = attempt_cleanup(cleanup_operation)
                if cleanup_error is not None:
                    cleanup_errors.append(cleanup_error)
            if cleanup_errors:
                cleanup_error = cleanup_errors[0]
                owner = _RobomimicCleanupOwner(tuple(cleanup_operations))
                if not operation_completed:
                    if primary_error is None:
                        raise cleanup_error
                    committed_paths: tuple[Path, ...] = ()
                    committed_result: object | None = None
                    if isinstance(primary_error, PublicationCleanupError):
                        owner = _RobomimicCleanupOwner(
                            (primary_error.owner.retry_cleanup, snapshot.close)
                        )
                        committed_paths = primary_error.committed_paths
                        committed_result = primary_error.committed_result
                    raise PublicationCleanupError(
                        "merge failed and input snapshot cleanup is pending",
                        owner=owner,
                        committed_paths=committed_paths,
                        committed_result=committed_result,
                        errors=(primary_error, *cleanup_errors),
                    ) from primary_error
                if committed_output is None:
                    raise cleanup_error
                raise PublicationCommitCleanupError(
                    "merge committed but input snapshot cleanup is pending",
                    owner=owner,
                    committed_paths=(committed_output,),
                    committed_result=committed_output,
                    errors=tuple(cleanup_errors),
                ) from cleanup_error

    print(
        f"\n[INFO] Merged {len(input_paths)} datasets into {output_path}"
        f"\n  Total demos: {demo_count}"
        f"\n  Total samples: {total_samples}"
    )
    return output_path


def _preflight_hdf5_metadata_storage(
    h5: "Any",
    *,
    path: Path,
    max_aggregate_bytes: int | None = None,
    metadata_budget: ByteBudget | None = None,
    max_attribute_bytes: int = MAX_METADATA_ATTRIBUTE_BYTES,
    max_attributes: int = MAX_METADATA_ATTRIBUTES,
    text_reader: Callable[[Any, tuple[str, ...], int], Mapping[str, str | bytes]]
    | None = None,
    attribute_storage_size: Callable[[Any], int] | None = None,
    copied_roots: Sequence[Any] | None = None,
) -> dict[str, str | bytes]:
    """Bound encoded attributes and decode known text through the native owner.

    ``max_aggregate_bytes`` may only lower the repository ceiling; callers
    charge retained decoded text against the same allowance.
    """

    aggregate_limit = require_bounded_int(
        MAX_METADATA_AGGREGATE_BYTES
        if max_aggregate_bytes is None
        else max_aggregate_bytes,
        resource="Robomimic HDF5 metadata aggregate bytes",
        minimum=1,
        maximum=MAX_METADATA_AGGREGATE_BYTES,
    )
    max_attribute_bytes = require_bounded_int(
        max_attribute_bytes,
        resource="Robomimic HDF5 metadata attribute bytes",
        minimum=1,
        maximum=MAX_METADATA_ATTRIBUTE_BYTES,
    )
    max_attributes = require_bounded_int(
        max_attributes,
        resource="Robomimic HDF5 metadata attribute count",
        minimum=1,
        maximum=MAX_METADATA_ATTRIBUTES,
    )
    if metadata_budget is not None:
        remaining_budget = metadata_budget.limit - metadata_budget.consumed
        if remaining_budget < 0:
            raise ResourceLimitError(
                metadata_budget.resource,
                limit=metadata_budget.limit,
                observed=metadata_budget.consumed,
            )
        aggregate_limit = min(aggregate_limit, remaining_budget)

    _preflight_robomimic_primitive_metadata(h5)

    # Reuse the event reader's attribute limits and storage inspection. The
    # native child supplies helpers directly to avoid importing Torch; vlen
    # payloads are bounded after decoding because HDF5 storage size can underreport them.
    metadata_storage_validator: Callable[..., None] | None = None
    if attribute_storage_size is None:
        from srb.core.sensor.event_camera.storage import (
            _attribute_storage_size,
        )
        from srb.core.sensor.event_camera.storage import (
            _validate_hdf5_metadata_storage as validate_metadata_storage,
        )

        attribute_storage_size = _attribute_storage_size
        metadata_storage_validator = validate_metadata_storage
    assert attribute_storage_size is not None

    whole_file_budget = ByteBudget(
        "Robomimic HDF5 metadata aggregate",
        aggregate_limit,
    )
    data = h5.get("data")
    data_identity = _hdf5_object_identity(data) if data is not None else None
    text_names = (
        tuple(name for name in _HDF5_TEXT_ATTRIBUTES if name in data.attrs)
        if data is not None
        else ()
    )
    copied_text_requests: list[tuple[Any, str]] = []
    copied_object_ids: set[int] = set()
    if copied_roots:
        pending_copied = list(copied_roots)
        while pending_copied:
            current = pending_copied.pop()
            identity = _hdf5_object_identity(current)
            if identity in copied_object_ids:
                continue
            copied_object_ids.add(identity)
            if not hasattr(current, "keys"):
                continue
            try:
                pending_copied.extend(current[key] for key in current.keys())
            except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"cannot inspect copied HDF5 members in {path}"
                ) from exc
        h5py = importlib.import_module("h5py")
    else:
        h5py = None
    pending: list[Any] = [h5]
    seen_objects: set[int] = set()
    while pending:
        current = pending.pop()
        identity = _hdf5_object_identity(current)
        if identity in seen_objects:
            continue
        seen_objects.add(identity)

        attrs = getattr(current, "attrs", None)
        if attrs is not None:
            if metadata_storage_validator is not None:
                try:
                    metadata_storage_validator(attrs, path=path)
                except RuntimeError as exc:
                    raise ValueError(
                        f"cannot inspect HDF5 metadata storage in {path}"
                    ) from exc
            attribute_count = 0
            for name in attrs:
                attribute_count += 1
                if attribute_count > max_attributes:
                    raise ResourceLimitError(
                        "Robomimic HDF5 metadata attribute count",
                        limit=max_attributes,
                        observed=attribute_count,
                    )
                attribute = None
                try:
                    attribute = attrs.get_id(name)
                    known_root_text = identity == data_identity and name in text_names
                    copied_vlen_text = (
                        h5py is not None
                        and identity in copied_object_ids
                        and not known_root_text
                        and _hdf5_scalar_attribute(attribute)
                        and np.dtype(attribute.dtype).kind == "O"
                        and h5py.check_dtype(vlen=np.dtype(attribute.dtype))
                        in (str, bytes)
                    )
                    if copied_vlen_text:
                        # Charge scalar text by decoded wire size below, not by
                        # its short descriptor as well.
                        stored_bytes = 0
                    else:
                        stored_bytes = attribute_storage_size(attribute)
                    if stored_bytes > max_attribute_bytes:
                        raise ResourceLimitError(
                            f"Robomimic HDF5 metadata attribute {name!r}",
                            limit=max_attribute_bytes,
                            observed=stored_bytes,
                        )
                    if copied_vlen_text:
                        copied_text_requests.append((current, name))
                    elif not known_root_text:
                        whole_file_budget.consume(stored_bytes)
                except ResourceLimitError:
                    raise
                except (
                    AttributeError,
                    KeyError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    raise ValueError(
                        f"cannot inspect HDF5 metadata attribute {name!r} in {path}"
                    ) from exc
                finally:
                    if attribute is not None:
                        try:
                            attribute.close()
                        except (AttributeError, OSError, TypeError, ValueError):
                            pass

        # Unsafe links were rejected by preflight; inspect child declarations
        # without reading dataset payloads.
        if not hasattr(current, "keys"):
            continue
        try:
            for key in current.keys():
                pending.append(current[key])
        except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
            raise ValueError(f"cannot inspect HDF5 metadata members in {path}") from exc

    if not text_names and not copied_text_requests:
        if metadata_budget is not None:
            metadata_budget.consume(whole_file_budget.consumed)
        return {}
    # Charge selected text by decoded size; unconsumed metadata retains its
    # encoded descriptor cost.
    remaining = aggregate_limit - whole_file_budget.consumed
    if remaining < 0:
        raise ResourceLimitError(
            "Robomimic HDF5 metadata aggregate",
            limit=aggregate_limit,
            observed=whole_file_budget.consumed,
        )

    if text_names:
        if text_reader is None:
            from srb.datasets.hdf5_metadata import (
                Hdf5MetadataLimits,
                read_hdf5_text_attributes,
            )

            values = read_hdf5_text_attributes(
                path,
                names=text_names,
                limits=Hdf5MetadataLimits(
                    max_value_bytes=min(max_attribute_bytes, max(1, remaining)),
                    max_aggregate_bytes=max(1, remaining),
                ),
            )
        else:
            values = dict(text_reader(data, text_names, remaining))
            if set(values) - set(text_names):
                raise ValueError("HDF5 text reader returned an unexpected attribute")
    else:
        values = {}

    text_decoded_bytes = 0
    for name, value in values.items():
        if not isinstance(value, (str, bytes)):
            raise TypeError("HDF5 text reader returned a non-text value")
        value_size = len(
            value.encode("utf-8", "surrogateescape")
            if isinstance(value, str)
            else value
        )
        if value_size > max_attribute_bytes:
            raise ResourceLimitError(
                f"Robomimic HDF5 metadata attribute {name!r}",
                limit=max_attribute_bytes,
                observed=value_size,
            )
        text_decoded_bytes += value_size
        if text_decoded_bytes > remaining:
            raise ResourceLimitError(
                "Robomimic HDF5 metadata aggregate",
                limit=remaining,
                observed=text_decoded_bytes,
            )

    if copied_text_requests:
        if text_reader is None:
            raise ValueError("copied HDF5 text requires a native reader")
        for current, name in copied_text_requests:
            value_remaining = remaining - text_decoded_bytes
            copied_values = dict(text_reader(current, (name,), max(1, value_remaining)))
            if set(copied_values) - {name}:
                raise ValueError("HDF5 text reader returned an unexpected attribute")
            if name not in copied_values:
                raise ValueError(f"HDF5 text reader omitted copied attribute {name!r}")
            value = copied_values[name]
            if not isinstance(value, (str, bytes)):
                raise TypeError("HDF5 text reader returned a non-text value")
            value_size = len(
                value.encode("utf-8", "surrogateescape")
                if isinstance(value, str)
                else value
            )
            if value_size > max_attribute_bytes:
                raise ResourceLimitError(
                    f"Robomimic HDF5 metadata attribute {name!r}",
                    limit=max_attribute_bytes,
                    observed=value_size,
                )
            text_decoded_bytes += value_size
            if text_decoded_bytes > remaining:
                raise ResourceLimitError(
                    "Robomimic HDF5 metadata aggregate",
                    limit=remaining,
                    observed=text_decoded_bytes,
                )
    if metadata_budget is not None:
        metadata_budget.consume(whole_file_budget.consumed)
        metadata_budget.consume(text_decoded_bytes)
    return values


def _native_validation_limits() -> "Hdf5NativeLimits":
    """Carry current Robomimic ceilings into the isolated native worker."""

    from srb.datasets.hdf5_native import Hdf5NativeLimits

    base = Hdf5NativeLimits()
    return replace(
        base,
        max_array_bytes=min(base.max_array_bytes, MAX_ARRAY_BYTES),
        max_array_elements=min(base.max_array_elements, MAX_ARRAY_ELEMENTS),
        max_decoded_bytes=min(base.max_decoded_bytes, MAX_DECODED_AGGREGATE_BYTES),
        max_entries=min(base.max_entries, MAX_DIRECTORY_ENTRIES),
        max_metadata_bytes=min(base.max_metadata_bytes, MAX_METADATA_AGGREGATE_BYTES),
        max_attribute_bytes=min(base.max_attribute_bytes, MAX_METADATA_ATTRIBUTE_BYTES),
        max_attributes=min(base.max_attributes, MAX_METADATA_ATTRIBUTES),
        max_source_bytes=min(
            base.max_source_bytes,
            MAX_EXTERNAL_FILE_BYTES,
            MAX_PUBLICATION_DIRECTORY_BYTES,
        ),
    )


def validate_robomimic_hdf5_contents(
    *,
    metadata: "Mapping[str, Any]",
    dataset_path: Path,
    errors: list[str],
) -> None:
    """Validate a robomimic HDF5 dataset against SRB manifest metadata."""

    if not dataset_path.is_file():
        errors.append(f"robomimic dataset is not an HDF5 file: {dataset_path}")
        return
    demo_io_contract = metadata.get("demo_io_contract")
    requires_demo_io_contract = demo_io_contract is not None
    try:
        _validate_demo_io_contract(demo_io_contract)
    except ValueError as exc:
        errors.append(str(exc))
    from srb.datasets.hdf5_native import Hdf5NativeCleanupError, inspect_robomimic_hdf5

    try:
        result = inspect_robomimic_hdf5(
            dataset_path,
            mode="deep",
            limits=_native_validation_limits(),
            require_demo_io_contract=requires_demo_io_contract,
        )
    except Hdf5NativeCleanupError:
        raise
    except ResourceLimitError as exc:
        errors.append(f"robomimic HDF5 exceeds resource limits: {exc}")
        return
    except (OSError, TypeError, ValueError) as exc:
        errors.append(f"could not read robomimic HDF5: {exc}")
        return
    errors.extend(result.errors)
    if not result.complete:
        return
    if result.demo_count is None or result.total_samples is None:
        errors.append("robomimic HDF5 inspection returned incomplete counts")
        return
    # Readers trust this attribute for dtypes and shapes; match the manifest contract.
    expected_contract_sha256 = demo_io_contract_sha256(demo_io_contract)
    if result.demo_io_contract_sha256 != expected_contract_sha256:
        if expected_contract_sha256 is None:
            errors.append(
                "robomimic HDF5 data carries a demo_io_contract the manifest "
                "does not declare"
            )
        elif result.demo_io_contract_sha256 is None:
            errors.append(
                "robomimic HDF5 data is missing the manifest demo_io_contract"
            )
        else:
            errors.append(
                "robomimic HDF5 data demo_io_contract does not match the "
                "manifest demo_io_contract"
            )
    _validate_metadata_count(
        metadata=metadata,
        key="num_episodes",
        actual=result.demo_count,
        errors=errors,
    )
    _validate_metadata_count(
        metadata=metadata,
        key="num_demos",
        actual=result.demo_count,
        errors=errors,
    )
    _validate_metadata_count(
        metadata=metadata,
        key="total_samples",
        actual=result.total_samples,
        errors=errors,
    )
    if result.action_dim is not None:
        _validate_metadata_count(
            metadata=metadata,
            key="action_dim",
            actual=result.action_dim,
            errors=errors,
        )
    if result.train_count is not None:
        _validate_metadata_count(
            metadata=metadata,
            key="num_train",
            actual=result.train_count,
            errors=errors,
        )
    if result.valid_count is not None:
        _validate_metadata_count(
            metadata=metadata,
            key="num_valid",
            actual=result.valid_count,
            errors=errors,
        )
    _validate_metadata_obs_keys(
        metadata=metadata,
        actual=set(result.obs_keys),
        errors=errors,
    )


def _validate_robomimic_demo(
    *,
    demo: "Any",
    demo_path: str,
    obs_keys: set[str],
    errors: list[str],
    decoded_budget: ByteBudget | None = None,
    require_demo_io_contract: bool = False,
    max_batch_bytes: int = MAX_IN_MEMORY_FILE_BYTES,
    metadata_budget: ByteBudget | None = None,
    max_entries: int = MAX_DIRECTORY_ENTRIES,
    max_array_bytes: int = MAX_ARRAY_BYTES,
    max_array_elements: int = MAX_ARRAY_ELEMENTS,
    max_rows: int = MAX_EVENT_ROWS,
    deadline_check: Callable[[], None] | None = None,
    declared_leaves: Mapping[tuple[str, ...], tuple[Any, tuple[int, ...] | None]]
    | None = None,
) -> int | None:
    if decoded_budget is None:
        decoded_budget = ByteBudget(
            "robomimic decoded bytes",
            MAX_DECODED_AGGREGATE_BYTES,
        )
    for dataset_name in ("actions", "rewards", "dones"):
        if dataset_name not in demo:
            errors.append(f"{demo_path} missing required dataset {dataset_name!r}")
    for group_name in ("obs", "next_obs"):
        if group_name not in demo:
            errors.append(f"{demo_path} missing required group {group_name!r}")

    sample_count = _robomimic_sample_count(demo)
    valid_declarations: set[str] = set()
    for dataset_name in ("actions", "rewards", "dones", "terminated", "truncated"):
        if dataset_name not in demo:
            continue
        if _validate_robomimic_field_declaration(
            dataset=demo[dataset_name],
            path=f"{demo_path}/{dataset_name}",
            require_sample_axis=dataset_name == "actions",
            scalar_row=dataset_name != "actions",
            require_boolean=dataset_name in {"terminated", "truncated"},
            errors=errors,
        ):
            valid_declarations.add(dataset_name)
    _validate_declared_sample_count(
        demo=demo,
        demo_path=demo_path,
        sample_count=sample_count,
        errors=errors,
    )

    # Termination flags appear together and align with ``dones`` rows.
    if ("terminated" in demo) != ("truncated" in demo):
        errors.append(
            f"{demo_path} must contain both terminated and truncated datasets "
            "or neither"
        )
    if require_demo_io_contract:
        for dataset_name in ("terminated", "truncated"):
            if dataset_name not in demo:
                errors.append(
                    f"{demo_path} certified demo_io_contract requires dataset "
                    f"{dataset_name!r}"
                )
        if "next_obs" not in demo:
            errors.append(
                f"{demo_path} certified demo_io_contract requires group 'next_obs'"
            )

    seen_datasets: set[int] = set()
    datasets_to_check: list[tuple[str, Any]] = []
    for dataset_name in ("actions", "rewards", "dones", "terminated", "truncated"):
        if dataset_name in valid_declarations:
            datasets_to_check.append(
                (f"{demo_path}/{dataset_name}", demo[dataset_name])
            )
    for group_name in ("obs", "next_obs"):
        if group_name not in demo:
            continue
        group = demo[group_name]
        if not hasattr(group, "keys"):
            errors.append(f"{demo_path}/{group_name} must be an HDF5 group")
            continue
        for key, dataset in _iter_hdf5_observation_datasets(
            group,
            max_entries=max_entries,
            max_depth=MAX_PUBLICATION_DIRECTORY_DEPTH,
        ):
            datasets_to_check.append((f"{demo_path}/{group_name}/{key}", dataset))
    approved_datasets: set[int] = set()
    for path, dataset in datasets_to_check:
        if _account_hdf5_dataset(
            dataset=dataset,
            path=path,
            decoded_budget=decoded_budget,
            errors=errors,
            seen_datasets=seen_datasets,
            approved_datasets=approved_datasets,
            max_array_bytes=max_array_bytes,
            max_array_elements=max_array_elements,
            max_rows=max_rows,
        ):
            approved_datasets.add(_hdf5_dataset_identity(dataset))

    if sample_count is not None:
        for dataset_name in ("rewards", "dones", "terminated", "truncated"):
            if dataset_name in demo:
                _validate_hdf5_first_dim(
                    dataset=demo[dataset_name],
                    path=f"{demo_path}/{dataset_name}",
                    expected=sample_count,
                    errors=errors,
                )

    # Pairing and length checks do not catch contradictory row values.
    safe_flags = all(
        name in valid_declarations
        and _hdf5_dataset_identity(demo[name]) in approved_datasets
        for name in ("dones", "terminated", "truncated")
    )
    if safe_flags:
        _validate_termination_flag_agreement(
            demo=demo,
            demo_path=demo_path,
            errors=errors,
            approved_datasets=approved_datasets,
            max_batch_bytes=max_batch_bytes,
            deadline_check=deadline_check,
        )

    if "obs" not in demo or "next_obs" not in demo:
        return sample_count

    obs_group = demo["obs"]
    next_obs_group = demo["next_obs"]
    if not hasattr(obs_group, "keys") or not hasattr(next_obs_group, "keys"):
        return sample_count
    obs_group_keys = {str(key) for key in obs_group.keys()}
    next_obs_group_keys = {str(key) for key in next_obs_group.keys()}
    if not obs_group_keys:
        errors.append(f"{demo_path}/obs has no observation datasets")
    if obs_group_keys != next_obs_group_keys:
        errors.append(
            f"{demo_path}/obs keys do not match next_obs keys: "
            f"obs={sorted(obs_group_keys)} next_obs={sorted(next_obs_group_keys)}"
        )
    obs_datasets = dict(
        _iter_hdf5_observation_datasets(
            obs_group,
            max_entries=max_entries,
            max_depth=MAX_PUBLICATION_DIRECTORY_DEPTH,
        )
    )
    next_obs_datasets = dict(
        _iter_hdf5_observation_datasets(
            next_obs_group,
            max_entries=max_entries,
            max_depth=MAX_PUBLICATION_DIRECTORY_DEPTH,
        )
    )
    if not obs_datasets:
        errors.append(f"{demo_path}/obs has no observation datasets")
    if set(obs_datasets) != set(next_obs_datasets):
        errors.append(
            f"{demo_path}/obs leaf paths do not match next_obs leaf paths: "
            f"obs={sorted(obs_datasets)} next_obs={sorted(next_obs_datasets)}"
        )
    for obs_key in sorted(obs_group_keys):
        if obs_key in obs_keys:
            continue
        if metadata_budget is not None:
            metadata_budget.consume(len(obs_key.encode("utf-8")) + 128)
        obs_keys.add(obs_key)

    if sample_count is not None:
        for obs_key in sorted(obs_datasets):
            _validate_hdf5_first_dim(
                dataset=obs_datasets[obs_key],
                path=f"{demo_path}/obs/{obs_key}",
                expected=sample_count,
                errors=errors,
            )
            if obs_key in next_obs_datasets:
                _validate_hdf5_first_dim(
                    dataset=next_obs_datasets[obs_key],
                    path=f"{demo_path}/next_obs/{obs_key}",
                    expected=sample_count,
                    errors=errors,
                )

    if declared_leaves:
        _validate_hdf5_contract_leaves(
            datasets={"obs": obs_datasets, "next_obs": next_obs_datasets},
            declared_leaves=declared_leaves,
            demo_path=demo_path,
            errors=errors,
            approved_datasets=approved_datasets,
            max_batch_bytes=max_batch_bytes,
            deadline_check=deadline_check,
        )

    return sample_count


def _validate_hdf5_contract_leaves(
    *,
    datasets: Mapping[str, Mapping[str, Any]],
    declared_leaves: list[
        tuple[tuple[tuple[str, ...], ...], Any, tuple[int, ...] | None]
    ],
    demo_path: str,
    errors: list[str],
    approved_datasets: set[int],
    max_batch_bytes: int,
    deadline_check: Callable[[], None] | None,
) -> None:
    """Stored observation leaves must match their contract terms.

    Rows use the term shape or legacy ``(1, *shape)``; ``obs`` and ``next_obs``
    agree, and storage can hold the declared dtype. Scan widened values in
    bounded batches; each term needs an alias per group, and every alias is checked.
    """

    from srb.datasets.visual_contract import (
        contract_shape_error,
        floating_overflow_error,
        needs_range_scan,
        range_error,
        storage_dtype_error,
    )

    for paths, dtype, shape in declared_leaves:
        term_row_shapes: dict[str, dict[str, tuple[int, ...]]] = {}
        for group_name, group_datasets in datasets.items():
            present = [
                (path, group_datasets["/".join(path)])
                for path in paths
                if "/".join(path) in group_datasets
            ]
            if not present:
                aliases = " or ".join("/".join(path) for path in paths)
                errors.append(
                    f"{demo_path}/{group_name} is missing declared observation "
                    f"term; expected one of {aliases}"
                )
                continue
            for path, dataset in present:
                key = "/".join(path)
                label = f"{demo_path}/{group_name}/{key}"
                try:
                    row_shape = tuple(int(dim) for dim in dataset.shape[1:])
                    stored = np.dtype(dataset.dtype)
                except (AttributeError, TypeError, ValueError) as exc:
                    errors.append(f"{label} declaration is unreadable: {exc}")
                    continue
                term_row_shapes.setdefault(key, {})[group_name] = row_shape
                if shape is not None:
                    error = contract_shape_error(label, row_shape, shape)
                    if error is not None:
                        errors.append(error)
                if dtype is None:
                    continue
                error = storage_dtype_error(label, stored, dtype)
                if error is not None:
                    errors.append(error)
                    continue
                if not needs_range_scan(stored, dtype):
                    continue
                if _hdf5_dataset_identity(dataset) not in approved_datasets:
                    continue
                row_bytes = max(1, stored.itemsize * max(1, math.prod(row_shape)))
                if max_batch_bytes < row_bytes:
                    raise ResourceLimitError(
                        "robomimic observation range batch bytes",
                        limit=max_batch_bytes,
                        observed=row_bytes,
                    )
                batch_rows = max(
                    1, min(MAX_EVENT_BATCH_ROWS, max_batch_bytes // row_bytes)
                )
                rows = int(dataset.shape[0]) if dataset.shape else 0
                for start in range(0, rows, batch_rows):
                    if deadline_check is not None:
                        deadline_check()
                    try:
                        batch = np.asarray(dataset[start : start + batch_rows])
                    except (OSError, TypeError, ValueError) as exc:
                        errors.append(f"{label} is unreadable: {exc}")
                        break
                    error = (
                        range_error(label, batch, dtype)
                        if dtype.kind in "iu"
                        else floating_overflow_error(label, batch, dtype)
                    )
                    del batch
                    if error is not None:
                        errors.append(error)
                        break
        for key, row_shapes in term_row_shapes.items():
            if row_shapes.get("obs") != row_shapes.get("next_obs"):
                errors.append(
                    f"{demo_path} obs/{key} shape {row_shapes.get('obs')} does not "
                    f"match next_obs/{key} shape {row_shapes.get('next_obs')}"
                )


@dataclass(slots=True)
class _RobomimicInspectionStats:
    """Physical fields retained by a bounded native inspection report."""

    action_dim: int | None = None
    obs_keys: set[str] = field(default_factory=set)
    demo_io_contract_sha256: str | None = None


def _inspect_robomimic_counts(
    *,
    h5: "Any",
    errors: list[str],
    decoded_budget: ByteBudget,
    max_batch_bytes: int = MAX_IN_MEMORY_FILE_BYTES,
    max_entries: int = MAX_DIRECTORY_ENTRIES,
    max_array_bytes: int = MAX_ARRAY_BYTES,
    max_array_elements: int = MAX_ARRAY_ELEMENTS,
    max_rows: int = MAX_EVENT_ROWS,
    stats: _RobomimicInspectionStats | None = None,
) -> tuple[int, int, int | None, int | None, int | None]:
    """Inspect only count declarations and optional split masks.

    Leaves demo fields, observation trees and termination flags untouched, but
    visits every ``data`` member to preserve Robomimic physical count semantics.
    """

    data_group = h5.get("data")
    if data_group is None:
        errors.append("robomimic HDF5 missing required group 'data'")
        return 0, 0, None, None, None
    if not hasattr(data_group, "keys"):
        errors.append("robomimic HDF5 'data' must be an HDF5 group")
        return 0, 0, None, None, None
    try:
        demo_keys = sorted(str(key) for key in data_group.keys())
    except (AttributeError, OSError, TypeError, ValueError) as exc:
        errors.append(f"cannot inspect robomimic demos before payload access: {exc}")
        return 0, 0, None, None, None
    require_bounded_int(
        len(demo_keys),
        resource="Robomimic HDF5 demo entries",
        maximum=max_entries,
    )

    total_samples = 0
    for demo_key in demo_keys:
        demo_path = f"data/{demo_key}"
        try:
            demo = data_group[demo_key]
        except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
            errors.append(f"cannot read HDF5 demo {demo_path}: {exc}")
            continue
        sample_count: int | None = None
        if hasattr(demo, "attrs"):
            try:
                actions = demo.get("actions") if hasattr(demo, "get") else None
                sample_count = _hdf5_first_dim(actions) if actions is not None else None
                _validate_declared_sample_count(
                    demo=demo,
                    demo_path=demo_path,
                    sample_count=sample_count,
                    errors=errors,
                )
            except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
                errors.append(f"{demo_path} has an unreadable count declaration: {exc}")
        if sample_count is not None:
            total_samples += sample_count

    num_train: int | None = None
    num_valid: int | None = None
    mask_group = h5.get("mask")
    if mask_group is not None and not hasattr(mask_group, "keys"):
        _validate_hdf5_mask_declaration(
            dataset=mask_group,
            path="mask",
            errors=errors,
        )
    elif mask_group is not None:
        seen_datasets: set[int] = set()
        approved_datasets: set[int] = set()
        for split in ("train", "valid"):
            if split not in mask_group:
                continue
            dataset = mask_group[split]
            if not _validate_hdf5_mask_declaration(
                dataset=dataset,
                path=f"mask/{split}",
                errors=errors,
            ):
                continue
            if not _account_hdf5_dataset(
                dataset=dataset,
                path=f"mask/{split}",
                decoded_budget=decoded_budget,
                errors=errors,
                seen_datasets=seen_datasets,
                approved_datasets=approved_datasets,
                max_array_bytes=max_array_bytes,
                max_array_elements=max_array_elements,
                max_rows=max_rows,
            ):
                continue
            count = 0
            for _value in _iter_hdf5_string_values(
                dataset,
                errors=errors,
                path=f"mask/{split}",
                conversion_budget=decoded_budget,
                raw_budget_accounted=True,
                max_batch_bytes=max_batch_bytes,
                max_array_bytes=max_array_bytes,
                max_array_elements=max_array_elements,
                max_rows=max_rows,
            ):
                count += 1
            if split == "train":
                num_train = count
            else:
                num_valid = count

    try:
        data_counts = _read_hdf5_count_attributes(data_group.attrs)
    except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
        errors.append(f"cannot read robomimic data count attributes: {exc}")
    else:
        _validate_metadata_count(
            metadata=data_counts,
            key="num_demos",
            actual=len(demo_keys),
            errors=errors,
        )
        _validate_metadata_count(
            metadata=data_counts,
            key="total_samples",
            actual=total_samples,
            errors=errors,
        )
        if num_train is not None:
            _validate_metadata_count(
                metadata=data_counts,
                key="num_train",
                actual=num_train,
                errors=errors,
            )
        if num_valid is not None:
            _validate_metadata_count(
                metadata=data_counts,
                key="num_valid",
                actual=num_valid,
                errors=errors,
            )
    # Count inspection leaves action dimensions unset; deep inspection fills them.
    if stats is not None:
        stats.action_dim = None
    return len(demo_keys), total_samples, None, num_train, num_valid


def _validate_robomimic_demos(
    *,
    h5: "Any",
    demo_keys: list[str] | None = None,
    errors: list[str],
    decoded_budget: ByteBudget,
    max_batch_bytes: int = MAX_IN_MEMORY_FILE_BYTES,
    require_demo_io_contract: bool = False,
    metadata_budget: ByteBudget | None = None,
    max_entries: int = MAX_DIRECTORY_ENTRIES,
    max_array_bytes: int = MAX_ARRAY_BYTES,
    max_array_elements: int = MAX_ARRAY_ELEMENTS,
    max_rows: int = MAX_EVENT_ROWS,
    stats: _RobomimicInspectionStats | None = None,
    deadline_check: Callable[[], None] | None = None,
) -> tuple[int, int]:
    """Admit every demo declaration before a consumer creates output.

    An exception raised by ``deadline_check`` propagates unchanged, even when
    its type would otherwise be reported as an unreadable declaration.
    """

    deadline_errors: list[Exception] = []
    guarded_deadline: Callable[[], None] | None = None
    if deadline_check is not None:
        checker = deadline_check

        def guarded_deadline() -> None:
            try:
                checker()
            except _UNREADABLE_DECLARATION_ERRORS as error:
                # Only these types would be reported as unreadable below;
                # every other deadline error already propagates unchanged.
                deadline_errors.append(error)
                raise

    data_group = h5.get("data")
    if data_group is None:
        errors.append("robomimic HDF5 missing required group 'data'")
        return 0, 0
    if not hasattr(data_group, "keys"):
        errors.append("robomimic HDF5 'data' must be an HDF5 group")
        return 0, 0

    if demo_keys is None:
        try:
            selected_demo_keys = sorted(str(key) for key in data_group.keys())
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            errors.append(
                f"cannot inspect robomimic demos before payload access: {exc}"
            )
            return 0, 0
    else:
        selected_demo_keys = list(demo_keys)
    require_bounded_int(
        len(selected_demo_keys),
        resource="Robomimic HDF5 demo entries",
        maximum=max_entries,
    )
    if not selected_demo_keys:
        errors.append("robomimic HDF5 has no demos under data")

    declared_leaves: list[
        tuple[tuple[tuple[str, ...], ...], Any, tuple[int, ...] | None]
    ] = []
    try:
        physical_contract = _read_hdf5_demo_io_contract_attr(data_group.attrs)
    except ResourceLimitError:
        raise
    except (TypeError, ValueError) as exc:
        errors.append(f"robomimic HDF5 data demo_io_contract is unreadable: {exc}")
    else:
        if physical_contract is not None:
            observation_contract = physical_contract.get("observation")
            raw_terms = (
                observation_contract.get("terms")
                if isinstance(observation_contract, Mapping)
                else None
            )
            if isinstance(raw_terms, list):
                for term in raw_terms:
                    if not isinstance(term, Mapping):
                        continue
                    raw_dtype = term.get("dtype")
                    try:
                        declared_dtype = np.dtype(raw_dtype.removeprefix("torch."))
                    except (AttributeError, TypeError, ValueError):
                        declared_dtype = None
                    if declared_dtype is None or declared_dtype.kind not in "biuf":
                        errors.append(
                            "robomimic HDF5 data demo_io_contract declares "
                            f"unsupported observation dtype {raw_dtype!r}"
                        )
            from srb.datasets.visual_contract import declared_observation_terms

            declared_leaves = declared_observation_terms(physical_contract)
            if stats is not None:
                stats.demo_io_contract_sha256 = demo_io_contract_sha256(
                    physical_contract
                )

    total_samples = 0
    action_dim: int | None = None
    obs_keys = stats.obs_keys if stats is not None else set()
    for demo_key in selected_demo_keys:
        demo_path = f"data/{demo_key}"
        try:
            demo = data_group[demo_key]
        except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
            errors.append(f"cannot read HDF5 demo {demo_path}: {exc}")
            continue
        if not hasattr(demo, "keys"):
            errors.append(f"{demo_path} must be an HDF5 group")
            continue
        try:
            sample_count = _validate_robomimic_demo(
                demo=demo,
                demo_path=demo_path,
                obs_keys=obs_keys,
                errors=errors,
                decoded_budget=decoded_budget,
                require_demo_io_contract=require_demo_io_contract,
                max_batch_bytes=max_batch_bytes,
                metadata_budget=metadata_budget,
                max_entries=max_entries,
                max_array_bytes=max_array_bytes,
                max_array_elements=max_array_elements,
                max_rows=max_rows,
                deadline_check=guarded_deadline,
                declared_leaves=declared_leaves,
            )
        except ResourceLimitError:
            raise
        except _UNREADABLE_DECLARATION_ERRORS as exc:
            if deadline_errors:
                raise
            errors.append(f"{demo_path} has an unreadable HDF5 declaration: {exc}")
            sample_count = None
        if sample_count is not None:
            total_samples += sample_count
        demo_action_dim = _robomimic_action_dim(demo)
        if demo_action_dim is not None:
            if action_dim is None:
                action_dim = demo_action_dim
            elif demo_action_dim != action_dim:
                errors.append(
                    f"{demo_path}/actions action dim {demo_action_dim} "
                    f"does not match first action dim {action_dim}"
                )
    if stats is not None:
        stats.action_dim = action_dim
    return len(selected_demo_keys), total_samples


#: How many disagreeing rows to name before summarising.
_FLAG_DISAGREEMENT_REPORT_LIMIT = 8


def _flag_batch_values(values: "Any", rows: int) -> np.ndarray:
    """Return one truth value per row, as ``bool(row)`` would, vectorized."""

    array = np.asarray(values)
    flat = array.reshape(rows, -1)
    if flat.shape[1] != 1:
        raise ValueError("termination flags must hold exactly one value per row")
    return flat[:, 0].astype(np.bool_, copy=False)


def _validate_termination_flag_agreement(
    *,
    demo: "Any",
    demo_path: str,
    errors: list[str],
    approved_datasets: set[int] | None = None,
    max_batch_bytes: int = MAX_IN_MEMORY_FILE_BYTES,
    deadline_check: Callable[[], None] | None = None,
) -> None:
    """Require ``dones`` to equal ``terminated or truncated``, row by row.

    Validate converted datasets too: ``done=False`` on a terminal row makes an
    offline learner bootstrap past the episode end. Compare bounded batches;
    call ``deadline_check`` before each batch, outside unreadable-flag handling.
    """

    try:
        if approved_datasets is not None and not all(
            _hdf5_dataset_identity(demo[name]) in approved_datasets
            for name in ("dones", "terminated", "truncated")
        ):
            return
        sample_count = _hdf5_first_dim(demo["dones"])
        terminated_count = _hdf5_first_dim(demo["terminated"])
        truncated_count = _hdf5_first_dim(demo["truncated"])
        if not (
            sample_count is not None
            and sample_count == terminated_count == truncated_count
        ):
            return
        row_bytes = sum(
            np.dtype(demo[name].dtype).itemsize
            for name in ("dones", "terminated", "truncated")
        )
    except (KeyError, OSError, TypeError, ValueError) as error:
        errors.append(f"{demo_path} termination flags are unreadable: {error}")
        return
    if sample_count == 0:
        return
    temporary_row_bytes = 5 + np.dtype(np.intp).itemsize
    minimum_batch_bytes = row_bytes + temporary_row_bytes
    if max_batch_bytes < minimum_batch_bytes:
        raise ResourceLimitError(
            "recorder termination flag batch bytes",
            limit=max_batch_bytes,
            observed=minimum_batch_bytes,
        )
    # Include boolean intermediates and flatnonzero's np.intp index array. The
    # index array may hold one entry per row on fully disagreeing input.
    batch_rows = min(
        MAX_EVENT_BATCH_ROWS,
        max_batch_bytes // minimum_batch_bytes,
    )
    disagreeing: list[int] = []
    disagreement_count = 0
    for start in range(0, sample_count, batch_rows):
        if deadline_check is not None:
            deadline_check()
        stop = min(start + batch_rows, sample_count)
        rows = stop - start
        try:
            dones = _flag_batch_values(demo["dones"][start:stop], rows)
            terminated = _flag_batch_values(demo["terminated"][start:stop], rows)
            truncated = _flag_batch_values(demo["truncated"][start:stop], rows)
        except (KeyError, OSError, TypeError, ValueError) as error:
            errors.append(f"{demo_path} termination flags are unreadable: {error}")
            return
        mismatch = np.flatnonzero(dones != (terminated | truncated))
        del dones, terminated, truncated
        disagreement_count += int(mismatch.size)
        room = _FLAG_DISAGREEMENT_REPORT_LIMIT - len(disagreeing)
        if room > 0:
            disagreeing.extend(start + int(index) for index in mismatch[:room])
        del mismatch

    if not disagreeing:
        return
    shown = disagreeing[:_FLAG_DISAGREEMENT_REPORT_LIMIT]
    remaining = disagreement_count - len(shown)
    suffix = f" (+{remaining} more)" if remaining else ""
    errors.append(
        f"{demo_path} rows where dones != (terminated or truncated): {shown}{suffix}"
    )


def _validate_declared_sample_count(
    *,
    demo: "Any",
    demo_path: str,
    sample_count: int | None,
    errors: list[str],
) -> None:
    """Compare one optional Robomimic declaration with its sample dimension."""
    try:
        declared_samples = _read_hdf5_primitive_attribute(demo.attrs, "num_samples")
    except (TypeError, ValueError) as exc:
        errors.append(f"{demo_path}: {exc}")
        return
    normalized_declared_samples = _normalize_int_value(declared_samples)
    if declared_samples is not None and normalized_declared_samples is None:
        errors.append(f"{demo_path} num_samples must be an integer")
    elif (
        normalized_declared_samples is not None
        and sample_count is not None
        and normalized_declared_samples != sample_count
    ):
        errors.append(
            f"{demo_path} num_samples={normalized_declared_samples} "
            f"does not match dataset length {sample_count}"
        )


def _robomimic_sample_count(demo: "Any") -> int | None:
    if "actions" in demo:
        return _hdf5_first_dim(demo["actions"])
    num_samples = _read_hdf5_primitive_attribute(demo.attrs, "num_samples")
    normalized_num_samples = _normalize_int_value(num_samples)
    if normalized_num_samples is not None:
        return normalized_num_samples
    return None


def _robomimic_action_dim(demo: "Any") -> int | None:
    if "actions" not in demo:
        return None
    shape_value = getattr(demo["actions"], "shape", None)
    if shape_value is None:
        return None
    try:
        shape = tuple(int(dim) for dim in shape_value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not shape:
        return None
    if len(shape) == 1:
        return 1
    dim = 1
    for axis in shape[1:]:
        dim *= axis
    return dim


def _hdf5_first_dim(dataset: "Any") -> int | None:
    shape = getattr(dataset, "shape", ())
    if not shape:
        return None
    return int(shape[0])


def _validate_robomimic_field_declaration(
    *,
    dataset: "Any",
    path: str,
    require_sample_axis: bool,
    scalar_row: bool,
    require_boolean: bool,
    errors: list[str],
) -> bool:
    """Validate field rank/dtype before accounting or reading its payload."""

    shape = getattr(dataset, "shape", None)
    if shape is None:
        errors.append(f"{path} has no inspectable HDF5 shape")
        return False
    try:
        dimensions = tuple(int(dimension) for dimension in shape)
    except (TypeError, ValueError, OverflowError):
        errors.append(f"{path} has an invalid HDF5 shape")
        return False

    if require_sample_axis:
        if not dimensions:
            errors.append(f"{path} must have a leading sample dimension")
            return False
    if scalar_row and len(dimensions) != 1:
        errors.append(
            f"{path} must have rank 1 (one scalar value per sample), "
            f"got rank {len(dimensions)}"
        )
        return False

    if require_boolean:
        try:
            dtype = np.dtype(dataset.dtype)
        except (AttributeError, TypeError, ValueError):
            errors.append(f"{path} must use boolean dtype")
            return False
        if dtype.kind != "b":
            errors.append(f"{path} must use boolean dtype, got {dtype}")
            return False
    return True


def _validate_hdf5_mask_declaration(
    *,
    dataset: "Any",
    path: str,
    errors: list[str],
) -> bool:
    """Validate a Robomimic split mask before any string conversion."""

    shape = getattr(dataset, "shape", None)
    try:
        dimensions = (
            tuple(int(dimension) for dimension in shape) if shape is not None else ()
        )
    except (TypeError, ValueError, OverflowError):
        dimensions = ()
    if len(dimensions) != 1:
        errors.append(
            f"{path} must have rank 1 (one demo ID per row), got rank {len(dimensions)}"
        )
        return False
    try:
        dtype = np.dtype(dataset.dtype)
    except (AttributeError, TypeError, ValueError):
        errors.append(f"{path} must use fixed-width byte-string dtype")
        return False
    if dtype.kind != "S" or dtype.hasobject or dtype.itemsize <= 0:
        errors.append(f"{path} must use fixed-width byte-string dtype, got {dtype}")
        return False
    return True


def _hdf5_dataset_identity(dataset: "Any") -> int:
    """Return a stable identity without reading an HDF5 dataset."""

    dataset_name = getattr(dataset, "name", None)
    if isinstance(dataset_name, str):
        file_obj = getattr(dataset, "file", None)
        filename = getattr(file_obj, "filename", None)
        return hash((str(filename), dataset_name))
    dataset_id = getattr(getattr(dataset, "id", None), "id", None)
    return int(dataset_id) if isinstance(dataset_id, int) else id(dataset)


def _hdf5_dataset_is_bounded(dataset: "Any", *, allow_strings: bool = False) -> bool:
    """Return whether fixed metadata bounds and supports the payload."""

    try:
        dtype = np.dtype(dataset.dtype)
    except (AttributeError, TypeError, ValueError):
        return False
    # Object/vlen heap size is absent from ``itemsize``; structured and subarray
    # dtypes have no conversion policy here.
    if dtype.hasobject or dtype.fields is not None or dtype.subdtype is not None:
        return False
    allowed_kinds = "S" if allow_strings else "biuf"
    return dtype.kind in allowed_kinds


def _account_hdf5_dataset(
    *,
    dataset: "Any",
    path: str,
    decoded_budget: ByteBudget,
    errors: list[str],
    seen_datasets: set[int],
    approved_datasets: set[int],
    max_array_bytes: int = MAX_ARRAY_BYTES,
    max_array_elements: int = MAX_ARRAY_ELEMENTS,
    max_rows: int = MAX_EVENT_ROWS,
) -> bool:
    """Inspect one HDF5 declaration and reserve decoded bytes before reads."""

    identity = _hdf5_dataset_identity(dataset)
    if identity in seen_datasets:
        return identity in approved_datasets
    seen_datasets.add(identity)
    shape = getattr(dataset, "shape", None)
    dtype = getattr(dataset, "dtype", None)
    if shape is None or dtype is None:
        errors.append(f"{path} has no inspectable HDF5 shape/dtype")
        return False
    if path.startswith("mask/"):
        try:
            rank = len(shape)
        except TypeError:
            rank = -1
        if rank != 1:
            errors.append(
                f"{path} must have rank 1 (one demo ID per row), got rank {rank}"
            )
            return False
    if not _hdf5_dataset_is_bounded(
        dataset,
        allow_strings=path.startswith("mask/"),
    ):
        errors.append(
            f"{path} uses an unsupported, variable-length, or object dtype; "
            "refusing an unbounded HDF5 payload"
        )
        return False
    try:
        if shape:
            require_bounded_int(
                shape[0],
                resource=f"{path} rows",
                minimum=0,
                maximum=max_rows,
            )
        decoded_bytes = checked_shape_nbytes(
            shape,
            np.dtype(dtype),
            resource=path,
            max_elements=max_array_elements,
            max_bytes=max_array_bytes,
        )
        decoded_budget.consume(decoded_bytes)
    except (TypeError, ValueError, ResourceLimitError) as exc:
        errors.append(f"{path} exceeds HDF5 decoded bounds: {exc}")
        return False
    approved_datasets.add(identity)
    return True


def _validate_hdf5_first_dim(
    *,
    dataset: "Any",
    path: str,
    expected: int,
    errors: list[str],
) -> None:
    actual = _hdf5_first_dim(dataset)
    if actual is None:
        errors.append(f"{path} must have a leading sample dimension")
    elif actual != expected:
        errors.append(
            f"{path} length {actual} does not match actions length {expected}"
        )


def _validate_robomimic_masks(
    *,
    h5: "Any",
    demo_keys: set[str],
    errors: list[str],
    decoded_budget: ByteBudget,
    raw_budget_accounted: bool = False,
    max_batch_bytes: int = MAX_ARRAY_BYTES,
    max_array_bytes: int = MAX_ARRAY_BYTES,
    max_array_elements: int = MAX_ARRAY_ELEMENTS,
    max_rows: int = MAX_EVENT_ROWS,
) -> tuple[int | None, int | None]:
    if "mask" not in h5:
        errors.append("robomimic HDF5 missing required group 'mask'")
        return None, None
    mask_group = h5["mask"]
    if not hasattr(mask_group, "keys"):
        _validate_hdf5_mask_declaration(
            dataset=mask_group,
            path="mask",
            errors=errors,
        )
        return None, None
    counts: dict[str, int] = {}
    split_keys: dict[str, set[str]] = {}
    seen_datasets: set[int] = set()
    approved_datasets: set[int] = set()
    for split in ("train", "valid"):
        if split not in mask_group:
            errors.append(f"mask missing required dataset {split!r}")
            continue
        dataset = mask_group[split]
        if not _validate_hdf5_mask_declaration(
            dataset=dataset,
            path=f"mask/{split}",
            errors=errors,
        ):
            continue
        if not raw_budget_accounted and not _account_hdf5_dataset(
            dataset=dataset,
            path=f"mask/{split}",
            decoded_budget=decoded_budget,
            errors=errors,
            seen_datasets=seen_datasets,
            approved_datasets=approved_datasets,
            max_array_bytes=max_array_bytes,
            max_array_elements=max_array_elements,
            max_rows=max_rows,
        ):
            continue
        count = 0
        unknown_count = 0
        unknown_examples: list[str] = []
        keys = split_keys.setdefault(split, set())
        duplicate_count = 0
        for key in _iter_hdf5_string_values(
            dataset,
            errors=errors,
            path=f"mask/{split}",
            conversion_budget=decoded_budget,
            raw_budget_accounted=True,
            max_batch_bytes=max_batch_bytes,
            max_array_bytes=max_array_bytes,
            max_array_elements=max_array_elements,
            max_rows=max_rows,
        ):
            count += 1
            if key in keys:
                duplicate_count += 1
            keys.add(key)
            if key not in demo_keys:
                unknown_count += 1
                if len(unknown_examples) < _MASK_UNKNOWN_REPORT_LIMIT:
                    unknown_examples.append(key[:128])
        counts[split] = count
        if duplicate_count:
            errors.append(f"mask/{split} lists {duplicate_count} duplicate demo key(s)")
        if unknown_count:
            remaining = unknown_count - len(unknown_examples)
            suffix = f" (+{remaining} more)" if remaining else ""
            errors.append(
                f"mask/{split} references unknown demos: {unknown_examples}{suffix}"
            )
    overlap = split_keys.get("train", set()) & split_keys.get("valid", set())
    if overlap:
        errors.append(
            f"mask/train and mask/valid share {len(overlap)} demo key(s): "
            f"{sorted(k[:128] for k in overlap)[:_MASK_UNKNOWN_REPORT_LIMIT]}"
        )
    return counts.get("train"), counts.get("valid")


def _iter_hdf5_string_values(
    dataset: "Any",
    *,
    errors: list[str] | None = None,
    path: str = "HDF5 string values",
    conversion_budget: ByteBudget | None = None,
    raw_budget_accounted: bool = False,
    max_batch_bytes: int = MAX_ARRAY_BYTES,
    max_array_bytes: int = MAX_ARRAY_BYTES,
    max_array_elements: int = MAX_ARRAY_ELEMENTS,
    max_rows: int = MAX_EVENT_ROWS,
    deadline_check: Callable[[], None] | None = None,
) -> Iterator[str]:
    """Yield bounded fixed-byte string conversions without ``tolist()``."""

    try:
        dimensions = tuple(int(dimension) for dimension in dataset.shape)
        dtype = np.dtype(dataset.dtype)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        message = f"{path} has an invalid HDF5 string declaration"
        if errors is not None:
            errors.append(message)
            return
        raise ValueError(message) from exc
    if len(dimensions) != 1:
        message = f"{path} must have rank 1 (one demo ID per row)"
        if errors is not None:
            errors.append(message)
            return
        raise ValueError(message)
    if dtype.kind != "S" or dtype.itemsize <= 0 or dtype.hasobject:
        message = f"{path} must use fixed-width byte-string dtype, got {dtype}"
        if errors is not None:
            errors.append(message)
            return
        raise ValueError(message)
    if conversion_budget is None:
        conversion_budget = ByteBudget(
            "HDF5 UTF-8 string conversion bytes",
            MAX_DECODED_AGGREGATE_BYTES,
        )
    try:
        sample_count = require_bounded_int(
            dimensions[0],
            resource=f"{path} rows",
            minimum=0,
            maximum=max_rows,
        )
        decoded_bytes = checked_shape_nbytes(
            dimensions,
            dtype,
            resource=path,
            max_elements=max_array_elements,
            max_bytes=max_array_bytes,
        )
        if not raw_budget_accounted:
            conversion_budget.consume(decoded_bytes)
    except (TypeError, ValueError, ResourceLimitError) as exc:
        message = f"{path} exceeds HDF5 decoded bounds: {exc}"
        if errors is not None:
            errors.append(message)
            return
        raise
    batch_limit = require_bounded_int(
        max_batch_bytes,
        resource=f"{path} batch byte limit",
        maximum=max_array_bytes,
    )
    start = 0
    while start < sample_count:
        if deadline_check is not None:
            deadline_check()
        # Reserve the current raw slice alongside aggregate raw storage and
        # cumulative conversions before h5py reads it.
        row_conversion_bytes = (
            5 * int(dtype.itemsize) + _MASK_STRING_CONVERSION_OVERHEAD
        )
        row_peak_bytes = int(dtype.itemsize) + row_conversion_bytes
        remaining = conversion_budget.limit - conversion_budget.consumed
        chunk_rows = min(
            MAX_EVENT_BATCH_ROWS,
            batch_limit // int(dtype.itemsize),
            remaining // row_peak_bytes,
        )
        if chunk_rows == 0:
            resource_error = ResourceLimitError(
                f"{conversion_budget.resource}: {path} UTF-8 conversion batch bytes",
                limit=min(batch_limit, remaining),
                observed=(
                    int(dtype.itemsize)
                    if batch_limit < int(dtype.itemsize)
                    else row_peak_bytes
                ),
            )
            if errors is not None:
                errors.append(str(resource_error))
                return
            raise resource_error
        stop = min(start + chunk_rows, sample_count)
        rows = stop - start
        conversion_bytes = rows * (
            int(dtype.itemsize)
            + 4 * int(dtype.itemsize)
            + _MASK_STRING_CONVERSION_OVERHEAD
        )
        try:
            # Reserve bytes(value), maximum decoded text and object overhead
            # before reading; the raw fixed-width array is already accounted.
            conversion_budget.consume(conversion_bytes)
        except ResourceLimitError as exc:
            message = f"{path} UTF-8 conversion exceeds resource bounds: {exc}"
            if errors is not None:
                errors.append(message)
                return
            raise
        raw = dataset[start:stop]
        for offset, value in enumerate(raw):
            try:
                raw_value = bytes(value)
            except (TypeError, ValueError) as exc:
                if errors is not None:
                    errors.append(
                        f"{path} contains an unreadable string at row "
                        f"{start + offset}: {exc}"
                    )
                    return
                raise
            try:
                yield raw_value.decode("utf-8")
            except UnicodeDecodeError as exc:
                if errors is not None:
                    errors.append(
                        f"{path} contains invalid UTF-8 at row {start + offset}"
                    )
                    return
                raise exc
        # Release the previous raw batch before allocating its successor.
        del raw, raw_value, value
        start = stop


def _require_h5py() -> Any:
    if importlib.util.find_spec("h5py") is None:
        raise ImportError(
            "Robomimic HDF5 dataset writing requires h5py. "
            "Install h5py or use the srb[data] extra."
        )
    return importlib.import_module("h5py")


def inspect_robomimic_hdf5(*args: Any, **kwargs: Any) -> Any:
    """Inspect Robomimic HDF5 through the bounded native decoder."""

    from srb.datasets.hdf5_native import inspect_robomimic_hdf5 as inspect_native

    return inspect_native(*args, **kwargs)


__all__ = [
    "TrajectoryRecorder",
    "inspect_robomimic_hdf5",
    "merge_datasets",
    "validate_robomimic_hdf5_contents",
]
