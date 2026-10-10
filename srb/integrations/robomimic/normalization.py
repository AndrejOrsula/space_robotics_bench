"""Bounded, transactional action normalization for Robomimic HDF5 files."""

from __future__ import annotations

import hashlib
import os
import posixpath
import stat
import sys
from copy import copy
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

from srb.utils.failures import attempt_cleanup
from srb.utils.publication import (
    ArtifactPublication,
    FilePublication,
    PublicationCleanupError,
    PublicationError,
    materialize_file_snapshot,
    sha256_file,
    stage_file_publications,
)
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES as _MAX_ARRAY_BYTES,
)
from srb.utils.resource_budget import (
    MAX_ARRAY_ELEMENTS as _MAX_ARRAY_ELEMENTS,
)
from srb.utils.resource_budget import (
    MAX_DECODED_AGGREGATE_BYTES as _MAX_DECODED_AGGREGATE_BYTES,
)
from srb.utils.resource_budget import (
    MAX_DIRECTORY_ENTRIES as _MAX_DIRECTORY_ENTRIES,
)
from srb.utils.resource_budget import (
    MAX_EVENT_ROWS as _MAX_EVENT_ROWS,
)
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES as _MAX_EXTERNAL_FILE_BYTES,
)
from srb.utils.resource_budget import (
    MAX_METADATA_AGGREGATE_BYTES as _MAX_METADATA_AGGREGATE_BYTES,
)
from srb.utils.resource_budget import (
    MAX_METADATA_ATTRIBUTE_BYTES as _MAX_METADATA_ATTRIBUTE_BYTES,
)
from srb.utils.resource_budget import (
    MAX_METADATA_ATTRIBUTES as _MAX_METADATA_ATTRIBUTES,
)
from srb.utils.resource_budget import (
    MAX_PUBLICATION_DIRECTORY_DEPTH as _MAX_PUBLICATION_DIRECTORY_DEPTH,
)
from srb.utils.resource_budget import (
    ByteBudget,
    ResourceLimitError,
    checked_shape_nbytes,
    require_bounded_int,
)

if TYPE_CHECKING:
    from srb.datasets.hdf5_native import Hdf5NativeLimits

# Local aliases allow tighter integration-specific limits.
MAX_ARRAY_BYTES = _MAX_ARRAY_BYTES
MAX_ARRAY_ELEMENTS = _MAX_ARRAY_ELEMENTS
MAX_DECODED_AGGREGATE_BYTES = _MAX_DECODED_AGGREGATE_BYTES
MAX_DIRECTORY_ENTRIES = _MAX_DIRECTORY_ENTRIES
MAX_EVENT_ROWS = _MAX_EVENT_ROWS
MAX_EXTERNAL_FILE_BYTES = _MAX_EXTERNAL_FILE_BYTES
MAX_METADATA_AGGREGATE_BYTES = _MAX_METADATA_AGGREGATE_BYTES
MAX_METADATA_ATTRIBUTE_BYTES = _MAX_METADATA_ATTRIBUTE_BYTES
MAX_METADATA_ATTRIBUTES = _MAX_METADATA_ATTRIBUTES

_MAX_DTYPE_ITEMSIZE = 64
_MAX_ATTRIBUTE_DECODE_DEPTH = 16
_PARAMETER_FILE_BYTES = 4096
_NORMALIZED_STAGE_PREFIX = ".srb_normalized_"


@dataclass(frozen=True)
class _ActionDeclaration:
    path: str
    shape: tuple[int, ...]
    dtype: Any
    decoded_bytes: int
    row_bytes: int
    normalized_dtype: Any | None = None
    normalized_bytes: int = 0
    normalized_row_bytes: int = 0
    global_row_bytes: int = 0
    finite_row_bytes: int = 0
    batch_rows: int = 1


@dataclass(frozen=True)
class _AttributeDeclaration:
    name: str
    shape: tuple[int, ...] | None
    dtype: Any
    stored_bytes: int


@dataclass(frozen=True)
class _HardLinkEdge:
    parent: Any
    name: str
    parent_identity: int
    target_identity: int

    @property
    def identity(self) -> tuple[int, str]:
        return self.parent_identity, self.name


@dataclass(frozen=True)
class _RewriteResult:
    source_identity: int
    target: Any
    anchor: tuple[Any, str] | None = None


def _require_h5py() -> Any:
    try:
        import h5py
    except ImportError as exc:  # pragma: no cover - dependency-gated callers
        raise ImportError(
            "Robomimic action normalization requires h5py; install srb[data]"
        ) from exc
    return h5py


def _require_numpy() -> Any:
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - dependency-gated callers
        raise ImportError("Robomimic action normalization requires NumPy") from exc
    return np


def _as_path(value: str | os.PathLike[str], *, role: str) -> Path:
    try:
        path_value = os.fspath(value)
    except TypeError as exc:
        raise TypeError(f"{role} must be a path string") from exc
    if isinstance(path_value, bytes):
        raise TypeError(f"{role} must be a text path")
    if not path_value:
        raise ValueError(f"{role} must not be empty")
    return Path(path_value).expanduser()


def _same_path(left: Path, right: Path) -> bool:
    """Compare path spellings without opening either path."""

    def canonical_parent(path: Path) -> Path:
        absolute = Path(os.path.abspath(os.fspath(path)))
        return absolute.parent.resolve(strict=False) / absolute.name

    return canonical_parent(left) == canonical_parent(right)


def _is_internal_link(link: Any, h5py: Any) -> bool:

    return isinstance(link, (h5py.HardLink, h5py.SoftLink))


def _require_internal_link(parent: Any, name: str, *, path: str, h5py: Any) -> None:
    try:
        link = parent.get(name, getlink=True)
    except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
        raise ValueError(f"cannot inspect HDF5 link {path}") from exc
    if link is None:
        raise ValueError(f"HDF5 path is missing: {path}")
    if not _is_internal_link(link, h5py):
        raise ValueError(f"HDF5 link is not internal for {path}")


def _preflight_internal_soft_links(h5: Any, h5py: Any) -> None:
    """Admit contained soft aliases after shared storage preflight.

    Reject external links, virtual datasets, and external raw storage. Track
    ancestor identities to prevent alias cycles.
    """

    try:
        external_link_type = h5py.ExternalLink
        soft_link_type = h5py.SoftLink
        group_type = h5py.Group
        dataset_type = h5py.Dataset
        datatype_type = h5py.Datatype
    except AttributeError as exc:
        raise ValueError("HDF5 storage preflight requires link support") from exc

    pending: list[tuple[Any, str, int, frozenset[int]]] = [(h5, "", 0, frozenset())]
    seen_objects: set[int] = set()
    member_count = 0
    while pending:
        current, current_path, depth, ancestors = pending.pop()
        if depth > _MAX_PUBLICATION_DIRECTORY_DEPTH:
            raise ResourceLimitError(
                "HDF5 normalization storage depth",
                limit=_MAX_PUBLICATION_DIRECTORY_DEPTH,
                observed=depth,
            )
        current_id = _hdf5_object_identity(current, h5py)
        if current_id in ancestors:
            raise ValueError(f"HDF5 normalization link cycle at {current_path or '/'}")
        if current_id in seen_objects:
            continue
        seen_objects.add(current_id)

        raw_keys: list[Any] = []
        try:
            for raw_key in current.keys():
                member_count += 1
                if member_count > MAX_DIRECTORY_ENTRIES:
                    raise ResourceLimitError(
                        "HDF5 normalization storage entries",
                        limit=MAX_DIRECTORY_ENTRIES,
                        observed=member_count,
                    )
                raw_keys.append(raw_key)
        except ResourceLimitError:
            raise
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            raise ValueError(
                f"cannot inspect HDF5 group {current_path or '/'}"
            ) from exc
        child_ancestors = ancestors | {current_id}
        for raw_key in sorted(raw_keys, key=str, reverse=True):
            key = str(raw_key)
            path = f"{current_path}/{key}" if current_path else key
            try:
                link = current.get(raw_key, getlink=True)
            except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
                raise ValueError(f"cannot inspect HDF5 link {path!r}") from exc
            if isinstance(link, external_link_type):
                raise ValueError(f"HDF5 external link {path!r} is not admitted")
            if not isinstance(link, (h5py.HardLink, soft_link_type)):
                raise ValueError(f"HDF5 unsupported link {path!r}")
            try:
                value = current[raw_key]
            except (KeyError, OSError, RuntimeError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"HDF5 normalization link cycle or broken link at {path!r}"
                ) from exc
            value_id = _hdf5_object_identity(value, h5py)
            if value_id in child_ancestors:
                raise ValueError(f"HDF5 normalization link cycle at {path!r}")
            if isinstance(value, group_type):
                child_depth = depth + 1
                if child_depth > _MAX_PUBLICATION_DIRECTORY_DEPTH:
                    raise ResourceLimitError(
                        "HDF5 normalization storage depth",
                        limit=_MAX_PUBLICATION_DIRECTORY_DEPTH,
                        observed=child_depth,
                    )
                pending.append((value, path, child_depth, child_ancestors))
                continue
            if not isinstance(value, dataset_type):
                if isinstance(value, datatype_type):
                    continue
                raise ValueError(f"unsupported HDF5 object {path!r}")
            try:
                if bool(value.is_virtual):
                    raise ValueError(f"HDF5 virtual dataset {path!r} is not admitted")
                if value.external:
                    raise ValueError(
                        f"HDF5 external raw storage {path!r} is not admitted"
                    )
            except (AttributeError, OSError, TypeError, ValueError) as exc:
                if isinstance(exc, ValueError) and str(exc).startswith("HDF5 "):
                    raise
                raise ValueError(f"cannot inspect HDF5 dataset {path!r}") from exc


def _hdf5_object_identity(value: Any, h5py: Any) -> int:
    """Return one identity shared by hard and soft aliases."""

    try:
        info = h5py.h5o.get_info(value.id)
        address = getattr(info, "addr", None)
        filename = getattr(getattr(value, "file", None), "filename", None)
        if isinstance(address, int):
            return hash((str(filename), address))
    except (AttributeError, OSError, TypeError, ValueError):
        pass
    object_id = getattr(getattr(value, "id", None), "id", None)
    return int(object_id) if isinstance(object_id, int) else id(value)


def _hard_link_edges(h5: Any, h5py: Any) -> dict[int, tuple[_HardLinkEdge, ...]]:
    """Index physical hard-link edges without reading dataset payloads."""

    group_type = h5py.Group
    external_link_type = h5py.ExternalLink
    soft_link_type = h5py.SoftLink
    pending: list[tuple[Any, int, frozenset[int]]] = [(h5, 0, frozenset())]
    seen_groups: set[int] = set()
    member_count = 0
    by_target: dict[int, list[_HardLinkEdge]] = {}
    while pending:
        current, depth, ancestors = pending.pop()
        if depth > _MAX_PUBLICATION_DIRECTORY_DEPTH:
            raise ResourceLimitError(
                "HDF5 normalization alias depth",
                limit=_MAX_PUBLICATION_DIRECTORY_DEPTH,
                observed=depth,
            )
        current_identity = _hdf5_object_identity(current, h5py)
        if current_identity in ancestors:
            raise ValueError(f"HDF5 normalization alias cycle at {current.name!r}")
        if current_identity in seen_groups:
            continue
        seen_groups.add(current_identity)
        try:
            raw_keys: list[Any] = []
            for raw_key in current.keys():
                member_count += 1
                if member_count > MAX_DIRECTORY_ENTRIES:
                    raise ResourceLimitError(
                        "HDF5 normalization alias entries",
                        limit=MAX_DIRECTORY_ENTRIES,
                        observed=member_count,
                    )
                raw_keys.append(raw_key)
        except ResourceLimitError:
            raise
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            raise ValueError(
                f"cannot inspect HDF5 alias group {current.name!r}"
            ) from exc
        child_ancestors = ancestors | {current_identity}
        for raw_key in sorted(raw_keys, key=str, reverse=True):
            name = str(raw_key)
            try:
                link = current.get(raw_key, getlink=True)
            except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"cannot inspect HDF5 alias link {current.name!r}/{name}"
                ) from exc
            if isinstance(link, external_link_type):
                raise ValueError(
                    f"HDF5 external alias link {current.name!r}/{name} is not admitted"
                )
            if not isinstance(link, (h5py.HardLink, soft_link_type)):
                raise ValueError(f"HDF5 unsupported alias link {current.name!r}/{name}")
            try:
                value = current[raw_key]
            except (KeyError, OSError, RuntimeError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"HDF5 normalization alias is broken: {current.name!r}/{name}"
                ) from exc
            value_identity = _hdf5_object_identity(value, h5py)
            if isinstance(link, h5py.HardLink):
                by_target.setdefault(value_identity, []).append(
                    _HardLinkEdge(
                        parent=current,
                        name=name,
                        parent_identity=current_identity,
                        target_identity=value_identity,
                    )
                )
            if isinstance(value, group_type):
                child_depth = depth + 1
                if value_identity in child_ancestors:
                    raise ValueError(
                        f"HDF5 normalization alias cycle at {current.name!r}/{name}"
                    )
                pending.append((value, child_depth, child_ancestors))
    return {identity: tuple(edges) for identity, edges in by_target.items()}


def _soft_target_edge(
    h5: Any,
    parent: Any,
    link: Any,
    *,
    h5py: Any,
) -> tuple[int, str] | None:
    """Resolve soft aliases to their final physical edge."""

    target_text = getattr(link, "path", None)
    if not isinstance(target_text, str) or not target_text:
        return None
    parent_name = getattr(parent, "name", None)
    if not isinstance(parent_name, str) or not parent_name:
        return None
    absolute = (
        target_text
        if target_text.startswith("/")
        else posixpath.join(parent_name, target_text)
    )
    normalized = posixpath.normpath(absolute)
    if not normalized.startswith("/"):
        normalized = f"/{normalized}"
    target_parent_path, separator, target_name = normalized.rpartition("/")
    if not separator or not target_name:
        return None
    try:
        target_parent = h5[target_parent_path or "/"]
        target_parent[target_name]
    except (KeyError, OSError, RuntimeError, TypeError, ValueError):
        return None
    return _hdf5_object_identity(target_parent, h5py), target_name


def _attribute_storage_size(attribute: Any) -> int:

    try:
        return int(attribute.get_storage_size())
    except RuntimeError:
        shape = getattr(attribute, "shape", None)
        if shape is None or any(int(dimension) == 0 for dimension in shape):
            return 0
        raise


def _admit_action_attributes(
    source: Any,
    *,
    metadata_budget: ByteBudget,
    np: Any,
    h5py: Any,
) -> tuple[_AttributeDeclaration, ...]:
    """Check attribute storage before decoding attribute values."""

    declarations: list[_AttributeDeclaration] = []
    try:
        for index, raw_name in enumerate(source.attrs):
            if index >= MAX_METADATA_ATTRIBUTES:
                raise ResourceLimitError(
                    f"{source.name} action attribute count",
                    limit=MAX_METADATA_ATTRIBUTES,
                    observed=index + 1,
                )
            name = str(raw_name)
            attribute = None
            try:
                attribute = h5py.h5a.open(source.id, name.encode("utf-8"))
                stored_bytes = _attribute_storage_size(attribute)
                if stored_bytes > MAX_METADATA_ATTRIBUTE_BYTES:
                    raise ResourceLimitError(
                        f"{source.name} attribute {name!r}",
                        limit=MAX_METADATA_ATTRIBUTE_BYTES,
                        observed=stored_bytes,
                    )
                metadata_budget.consume(stored_bytes)
                raw_shape = getattr(attribute, "shape", None)
                shape = (
                    None
                    if raw_shape is None
                    else tuple(int(dimension) for dimension in raw_shape)
                )
                dtype = np.dtype(attribute.dtype)
                if shape is not None:
                    checked_shape_nbytes(
                        shape,
                        dtype,
                        resource=f"{source.name} attribute {name!r}",
                        max_elements=MAX_ARRAY_ELEMENTS,
                        max_bytes=MAX_METADATA_ATTRIBUTE_BYTES,
                    )
                declarations.append(
                    _AttributeDeclaration(
                        name=name,
                        shape=shape,
                        dtype=dtype,
                        stored_bytes=stored_bytes,
                    )
                )
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
                    f"cannot inspect HDF5 action attribute {name!r}: {source.name}"
                ) from exc
            finally:
                if attribute is not None:
                    attribute.close()
    except ResourceLimitError:
        raise
    except (AttributeError, OSError, TypeError, ValueError) as exc:
        raise ValueError(
            f"cannot inspect HDF5 action attributes: {source.name}"
        ) from exc
    return tuple(declarations)


def _decoded_attribute_payload_bytes(value: Any, dtype: Any, *, depth: int) -> int:
    """Count variable-length bytes within one decoded value."""

    if depth > _MAX_ATTRIBUTE_DECODE_DEPTH:
        raise ResourceLimitError(
            "HDF5 action attribute decode depth",
            limit=_MAX_ATTRIBUTE_DECODE_DEPTH,
            observed=depth,
        )
    subdtype = getattr(dtype, "subdtype", None)
    if subdtype is not None:
        # NumPy expands a compound subarray field into the selected value's
        # dimensions, while the field declaration retains its subarray dtype.
        return _decoded_attribute_payload_bytes(value, subdtype[0], depth=depth + 1)
    fields = getattr(dtype, "fields", None)
    if fields is not None:
        if len(fields) > MAX_METADATA_ATTRIBUTES:
            raise ResourceLimitError(
                "HDF5 action attribute field count",
                limit=MAX_METADATA_ATTRIBUTES,
                observed=len(fields),
            )
        return sum(
            _decoded_attribute_payload_bytes(value[name], field[0], depth=depth + 1)
            for name, field in fields.items()
        )
    if dtype.kind != "O":
        return 0
    metadata = getattr(dtype, "metadata", None) or {}
    if "ref" in metadata or "vlen" not in metadata:
        return 0
    try:
        flattened = value.reshape(-1)
        if int(flattened.size) > MAX_ARRAY_ELEMENTS:
            raise ResourceLimitError(
                "HDF5 action attribute decoded elements",
                limit=MAX_ARRAY_ELEMENTS,
                observed=int(flattened.size),
            )
        payload_bytes = 0
        for item in flattened:
            if isinstance(item, str):
                payload_bytes += len(item.encode("utf-8"))
            elif isinstance(item, (bytes, bytearray, memoryview)):
                payload_bytes += len(item)
            else:
                item_bytes = getattr(item, "nbytes", None)
                if not isinstance(item_bytes, int):
                    raise ValueError(
                        "HDF5 action variable-length value has no byte size"
                    )
                payload_bytes += int(item_bytes)
                item_dtype = getattr(item, "dtype", None)
                if item_dtype is not None:
                    payload_bytes += _decoded_attribute_payload_bytes(
                        item, item_dtype, depth=depth + 1
                    )
        return payload_bytes
    except ResourceLimitError:
        raise
    except (AttributeError, TypeError, ValueError, UnicodeError) as exc:
        raise ValueError("cannot measure decoded HDF5 action attribute") from exc


def _decoded_attribute_bytes(value: Any, dtype: Any) -> int:
    """Count decoded storage without traversing object references."""

    try:
        decoded_bytes = int(value.nbytes)
        return decoded_bytes + _decoded_attribute_payload_bytes(value, dtype, depth=0)
    except ResourceLimitError:
        raise
    except (AttributeError, TypeError, ValueError, UnicodeError) as exc:
        raise ValueError("cannot measure decoded HDF5 action attribute") from exc


def _copy_action_attributes(
    source: Any,
    target: Any,
    declarations: tuple[_AttributeDeclaration, ...],
    *,
    metadata_budget: ByteBudget,
    np: Any,
    h5py: Any,
) -> None:
    """Preserve admitted attributes with their source HDF5 type and shape."""

    for declaration in declarations:
        source_attribute = None
        target_attribute = None
        value = None
        try:
            source_attribute = h5py.h5a.open(
                source.id, declaration.name.encode("utf-8")
            )
            target_attribute = h5py.h5a.create(
                target.id,
                declaration.name.encode("utf-8"),
                source_attribute.get_type(),
                source_attribute.get_space(),
            )
            if declaration.shape is not None:
                value = np.empty(declaration.shape, dtype=declaration.dtype)
                # The native worker contains h5py's VLEN allocations. Decode
                # accounting below enforces the separate logical byte limits.
                source_attribute.read(value)
                decoded_bytes = _decoded_attribute_bytes(value, declaration.dtype)
                if decoded_bytes > MAX_METADATA_ATTRIBUTE_BYTES:
                    raise ResourceLimitError(
                        f"{source.name} attribute {declaration.name!r}",
                        limit=MAX_METADATA_ATTRIBUTE_BYTES,
                        observed=decoded_bytes,
                    )
                metadata_budget.consume(
                    max(0, decoded_bytes - declaration.stored_bytes)
                )
                target_attribute.write(value)
        finally:
            if target_attribute is not None:
                target_attribute.close()
            if source_attribute is not None:
                source_attribute.close()
            del value


def _preflight_storage(h5: Any, h5py: Any) -> None:
    """Apply shared storage limits before the legacy soft-link traversal."""

    try:
        from srb.datasets.robomimic import (
            _preflight_hdf5_storage as shared_preflight,
        )
    except ImportError as exc:
        raise ValueError("HDF5 storage preflight is unavailable") from exc
    errors: list[str] = []
    if shared_preflight(h5, errors=errors):
        return
    if errors and all(
        "symbolic link" in error or "soft link" in error for error in errors
    ):
        _preflight_internal_soft_links(h5, h5py)
        return
    details = "; ".join(errors) if errors else "unknown storage error"
    raise ValueError(f"HDF5 normalization storage is not admitted: {details}")


def _action_paths(h5: Any, h5py: Any) -> list[tuple[str, Any]]:
    """Find canonical demo actions while rejecting unsafe links."""

    try:
        data_link = h5.get("data", getlink=True)
    except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
        raise ValueError("cannot inspect HDF5 data link") from exc
    if data_link is None:
        return []
    if not _is_internal_link(data_link, h5py):
        raise ValueError("HDF5 data link must be an internal link")
    try:
        data = h5["data"]
    except (KeyError, OSError, TypeError, ValueError) as exc:
        raise ValueError("cannot open HDF5 data group") from exc
    if not hasattr(data, "keys"):
        raise ValueError("HDF5 data object must be a group")

    paths: list[tuple[str, Any]] = []
    try:
        keys: list[Any] = []
        for raw_key in data.keys():
            if len(keys) >= MAX_DIRECTORY_ENTRIES:
                raise ResourceLimitError(
                    "robomimic action directory entries",
                    limit=MAX_DIRECTORY_ENTRIES,
                    observed=len(keys) + 1,
                )
            keys.append(raw_key)
        for raw_key in sorted(keys, key=str):
            key = str(raw_key)
            if not key.startswith("demo_"):
                continue
            demo_path = f"/data/{key}"
            _require_internal_link(data, key, path=demo_path, h5py=h5py)
            demo = data[key]
            if not hasattr(demo, "keys"):
                raise ValueError(f"HDF5 demo object must be a group: {demo_path}")
            action_link = demo.get("actions", getlink=True)
            if action_link is None:
                continue
            action_path = f"{demo_path}/actions"
            if not _is_internal_link(action_link, h5py):
                raise ValueError(
                    f"HDF5 action link must be an internal link: {action_path}"
                )
            action = demo["actions"]
            paths.append((action_path, action))
            if len(paths) > MAX_DIRECTORY_ENTRIES:
                raise ResourceLimitError(
                    "robomimic action dataset entries",
                    limit=MAX_DIRECTORY_ENTRIES,
                    observed=len(paths),
                )
    except ResourceLimitError:
        raise
    except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith("HDF5 "):
            raise
        raise ValueError("cannot inspect HDF5 demo action members") from exc
    return paths


def _initial_declaration(
    path: str,
    dataset: Any,
    *,
    np: Any,
) -> _ActionDeclaration:
    try:
        shape = tuple(int(dimension) for dimension in dataset.shape)
        dtype = np.dtype(dataset.dtype)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"cannot inspect HDF5 action declaration: {path}") from exc
    if not shape:
        raise ValueError(f"HDF5 action dataset has no sample axis: {path}")
    if dtype.kind not in "iuf" or dtype.hasobject:
        raise ValueError(
            f"HDF5 action dataset uses an unsupported dtype {dtype}: {path}"
        )
    if dtype.fields is not None or dtype.subdtype is not None:
        raise ValueError(f"HDF5 action dataset uses a compound dtype: {path}")
    if dtype.itemsize <= 0 or dtype.itemsize > _MAX_DTYPE_ITEMSIZE:
        raise ResourceLimitError(
            f"{path} dtype itemsize",
            limit=_MAX_DTYPE_ITEMSIZE,
            observed=int(dtype.itemsize),
        )
    sample_count = require_bounded_int(
        shape[0],
        resource=f"{path} rows",
        minimum=0,
        maximum=MAX_EVENT_ROWS,
    )
    del sample_count
    decoded_bytes = checked_shape_nbytes(
        shape,
        dtype,
        resource=path,
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
    )
    row_shape = (1, *shape[1:])
    row_bytes = checked_shape_nbytes(
        row_shape,
        dtype,
        resource=f"{path} row",
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
    )
    try:
        chunks = dataset.chunks
    except (AttributeError, OSError, TypeError, ValueError) as exc:
        raise ValueError(f"cannot inspect HDF5 action chunking: {path}") from exc
    if chunks is not None:
        checked_shape_nbytes(
            chunks,
            dtype,
            resource=f"{path} chunk",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
    try:
        if bool(dataset.is_virtual):
            raise ValueError(f"HDF5 virtual action dataset is not allowed: {path}")
        if dataset.external:
            raise ValueError(
                f"HDF5 externally stored action dataset is not allowed: {path}"
            )
    except (AttributeError, OSError, TypeError, ValueError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith("HDF5 "):
            raise
        raise ValueError(f"cannot inspect HDF5 action storage: {path}") from exc
    return _ActionDeclaration(
        path=path,
        shape=shape,
        dtype=dtype,
        decoded_bytes=decoded_bytes,
        row_bytes=row_bytes,
    )


def _normalization_dtype(input_dtype: Any, scalar_dtype: Any, *, np: Any) -> Any:
    """Match NumPy's weak-scalar result dtype for compatibility."""

    probe = np.zeros((1,), dtype=input_dtype)
    minimum = np.asarray(0, dtype=scalar_dtype)[()]
    maximum = np.asarray(1, dtype=scalar_dtype)[()]
    try:
        result = 2 * ((probe - minimum) / (maximum - minimum)) - 1
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"cannot normalize HDF5 action dtype {input_dtype}") from exc
    dtype = np.dtype(result.dtype)
    if dtype.kind not in "f" or dtype.hasobject:
        raise ValueError(f"normalization produced unsupported dtype {dtype}")
    return dtype


def _normalization_peak_row_bytes(
    *,
    source_row_bytes: int,
    normalized_row_bytes: int,
    global_row_bytes: int,
    finite_row_bytes: int,
    source_dtype: Any,
    scalar_dtype: Any,
) -> int:
    peak = source_row_bytes + max(
        normalized_row_bytes,
        global_row_bytes,
        finite_row_bytes if source_dtype.kind == "f" else 0,
    )
    if (
        scalar_dtype.kind in "iu"
        and source_dtype.kind in "iu"
        and source_dtype != scalar_dtype
    ):
        # The caller retains its source slice while the widened integer delta
        # and normalized result are both live.
        return source_row_bytes + global_row_bytes + normalized_row_bytes
    return peak


def _complete_declarations(
    initial: list[_ActionDeclaration],
    *,
    np: Any,
    max_decoded_bytes: int,
    max_batch_bytes: int,
) -> tuple[list[_ActionDeclaration], Any]:
    try:
        scalar_dtype = np.result_type(*(declaration.dtype for declaration in initial))
        scalar_dtype = np.dtype(scalar_dtype)
    except (TypeError, ValueError) as exc:
        raise ValueError("cannot determine the global HDF5 action dtype") from exc
    if scalar_dtype.kind not in "iuf" or scalar_dtype.kind == "b":
        raise ValueError(f"global HDF5 action dtype is unsupported: {scalar_dtype}")
    if scalar_dtype.hasobject or scalar_dtype.fields is not None:
        raise ValueError(f"global HDF5 action dtype is unsupported: {scalar_dtype}")

    completed: list[_ActionDeclaration] = []
    native_budget = ByteBudget(
        "robomimic normalization decoded arrays", max_decoded_bytes
    )
    output_bytes_total = 0
    for declaration in initial:
        normalized_dtype = _normalization_dtype(
            declaration.dtype,
            scalar_dtype,
            np=np,
        )
        normalized_bytes = checked_shape_nbytes(
            declaration.shape,
            normalized_dtype,
            resource=f"{declaration.path} normalized array",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        row_shape = (1, *declaration.shape[1:])
        normalized_row_bytes = checked_shape_nbytes(
            row_shape,
            normalized_dtype,
            resource=f"{declaration.path} normalized row",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        global_row_bytes = checked_shape_nbytes(
            row_shape,
            scalar_dtype,
            resource=f"{declaration.path} extrema row",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        finite_row_bytes = checked_shape_nbytes(
            row_shape,
            np.dtype(np.bool_),
            resource=f"{declaration.path} finite-check row",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
        peak_row_bytes = _normalization_peak_row_bytes(
            source_row_bytes=declaration.row_bytes,
            normalized_row_bytes=normalized_row_bytes,
            global_row_bytes=global_row_bytes,
            finite_row_bytes=finite_row_bytes,
            source_dtype=declaration.dtype,
            scalar_dtype=scalar_dtype,
        )
        if peak_row_bytes > max_batch_bytes:
            raise ResourceLimitError(
                f"{declaration.path} normalization batch",
                limit=max_batch_bytes,
                observed=peak_row_bytes,
            )
        if peak_row_bytes > max_decoded_bytes:
            raise ResourceLimitError(
                f"{declaration.path} normalization live arrays",
                limit=max_decoded_bytes,
                observed=peak_row_bytes,
            )
        sample_count = declaration.shape[0]
        effective_batch_bytes = min(max_batch_bytes, max_decoded_bytes)
        batch_rows = max(
            1,
            min(
                sample_count or 1,
                effective_batch_bytes // max(1, peak_row_bytes),
            ),
        )

        # Reserve source and transformed declarations before any payload slice.
        native_budget.consume(declaration.decoded_bytes)
        native_budget.consume(normalized_bytes)
        output_bytes_total += normalized_bytes
        completed.append(
            _ActionDeclaration(
                path=declaration.path,
                shape=declaration.shape,
                dtype=declaration.dtype,
                decoded_bytes=declaration.decoded_bytes,
                row_bytes=declaration.row_bytes,
                normalized_dtype=normalized_dtype,
                normalized_bytes=normalized_bytes,
                normalized_row_bytes=normalized_row_bytes,
                global_row_bytes=global_row_bytes,
                finite_row_bytes=finite_row_bytes,
                batch_rows=batch_rows,
            )
        )
    return completed, scalar_dtype


def _decoded_byte_limit(value: int | None) -> int:
    if value is None:
        return int(MAX_DECODED_AGGREGATE_BYTES)
    return int(
        require_bounded_int(
            value,
            resource="robomimic normalization decoded arrays",
            minimum=1,
            maximum=MAX_DECODED_AGGREGATE_BYTES,
        )
    )


def _batch_byte_limit(value: int | None) -> int:
    if value is None:
        return int(MAX_ARRAY_BYTES)
    return int(
        require_bounded_int(
            value,
            resource="robomimic normalization batch bytes",
            minimum=1,
            maximum=MAX_ARRAY_BYTES,
        )
    )


def _finite_or_raise(values: Any, *, dtype: Any, path: str, np: Any) -> None:
    if dtype.kind != "f":
        return
    try:
        finite = bool(np.isfinite(values).all())
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"cannot inspect HDF5 action finiteness: {path}") from exc
    if not finite:
        raise ValueError(f"HDF5 action dataset contains non-finite values: {path}")


def _global_extrema(
    h5: Any,
    declarations: list[_ActionDeclaration],
    *,
    scalar_dtype: Any,
    np: Any,
) -> tuple[Any | None, Any | None]:
    minimum: Any | None = None
    maximum: Any | None = None
    for declaration in declarations:
        dataset = h5[declaration.path]
        sample_count = declaration.shape[0]
        for start in range(0, sample_count, declaration.batch_rows):
            stop = min(start + declaration.batch_rows, sample_count)
            values = np.asarray(dataset[start:stop])
            if values.size == 0:
                del values
                continue
            _finite_or_raise(
                values, dtype=declaration.dtype, path=declaration.path, np=np
            )
            cast = np.asarray(values, dtype=scalar_dtype)
            local_min = np.min(cast)
            local_max = np.max(cast)
            if minimum is None or bool(local_min < minimum):
                minimum = local_min
            if maximum is None or bool(local_max > maximum):
                maximum = local_max
            del cast
            del values
    return minimum, maximum


def _normalized_batch(
    values: Any,
    *,
    declaration: _ActionDeclaration,
    minimum: Any,
    maximum: Any,
    scalar_dtype: Any,
    np: Any,
) -> Any:
    output_dtype = declaration.normalized_dtype
    assert output_dtype is not None
    if scalar_dtype.kind in "iu" and declaration.dtype.kind in "iu":
        # Integer extrema can span the full dtype range, so the signed delta
        # overflows.  ``value - minimum`` always lies in [0, 2**bits), hence
        # wrapping subtraction in the same-width unsigned dtype is exact.
        unsigned = np.dtype(f"u{scalar_dtype.itemsize}").newbyteorder(
            scalar_dtype.byteorder
        )
        lower = np.asarray(minimum, dtype=scalar_dtype).view(unsigned)
        span = float(int(maximum) - int(minimum))
        if declaration.dtype == scalar_dtype:
            delta = values.view(unsigned)
        else:
            delta = np.asarray(values, dtype=scalar_dtype).view(unsigned)
        del values
        np.subtract(delta, lower, out=delta, casting="unsafe")
        normalized = np.asarray(delta, dtype=output_dtype)
        del delta
    else:
        span = maximum - minimum
        normalized = np.asarray(values, dtype=output_dtype)
        np.subtract(normalized, minimum, out=normalized, casting="unsafe")
    np.divide(normalized, span, out=normalized, casting="unsafe")
    np.multiply(normalized, 2, out=normalized, casting="unsafe")
    np.subtract(normalized, 1, out=normalized, casting="unsafe")
    return normalized


def _refresh_declaration(
    declaration: _ActionDeclaration,
    source: Any,
    *,
    scalar_dtype: Any,
    max_decoded_bytes: int,
    max_batch_bytes: int,
    np: Any,
) -> _ActionDeclaration:
    """Re-admit a dataset after alias rewiring may have changed its target."""

    current = _initial_declaration(declaration.path, source, np=np)
    if current.shape == declaration.shape and current.dtype == declaration.dtype:
        return declaration
    if current.shape != declaration.shape:
        raise ValueError(
            f"HDF5 action alias changed shape for {declaration.path}: "
            f"{current.shape} != {declaration.shape}"
        )
    normalized_dtype = _normalization_dtype(current.dtype, scalar_dtype, np=np)
    normalized_bytes = checked_shape_nbytes(
        current.shape,
        normalized_dtype,
        resource=f"{current.path} normalized array",
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
    )
    row_shape = (1, *current.shape[1:])
    normalized_row_bytes = checked_shape_nbytes(
        row_shape,
        normalized_dtype,
        resource=f"{current.path} normalized row",
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
    )
    global_row_bytes = checked_shape_nbytes(
        row_shape,
        scalar_dtype,
        resource=f"{current.path} extrema row",
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
    )
    finite_row_bytes = checked_shape_nbytes(
        row_shape,
        np.dtype(np.bool_),
        resource=f"{current.path} finite-check row",
        max_elements=MAX_ARRAY_ELEMENTS,
        max_bytes=MAX_ARRAY_BYTES,
    )
    peak_row_bytes = _normalization_peak_row_bytes(
        source_row_bytes=current.row_bytes,
        normalized_row_bytes=normalized_row_bytes,
        global_row_bytes=global_row_bytes,
        finite_row_bytes=finite_row_bytes,
        source_dtype=current.dtype,
        scalar_dtype=scalar_dtype,
    )
    if peak_row_bytes > max_batch_bytes:
        raise ResourceLimitError(
            f"{current.path} normalization batch",
            limit=max_batch_bytes,
            observed=peak_row_bytes,
        )
    if peak_row_bytes > max_decoded_bytes:
        raise ResourceLimitError(
            f"{current.path} normalization live arrays",
            limit=max_decoded_bytes,
            observed=peak_row_bytes,
        )
    batch_rows = max(
        1,
        min(
            current.shape[0] or 1,
            min(max_batch_bytes, max_decoded_bytes) // max(1, peak_row_bytes),
        ),
    )
    return replace(
        current,
        normalized_dtype=normalized_dtype,
        normalized_bytes=normalized_bytes,
        normalized_row_bytes=normalized_row_bytes,
        global_row_bytes=global_row_bytes,
        finite_row_bytes=finite_row_bytes,
        batch_rows=batch_rows,
    )


def _representable_fill_value(source: Any, output_dtype: Any, *, np: Any) -> Any:
    """Reject fill values that overflow the normalized dtype."""

    try:
        source_value = np.asarray(source.fillvalue)
        if source_value.shape:
            raise ValueError("HDF5 action fill value must be scalar")
        safe_cast = bool(np.can_cast(source_value.dtype, output_dtype, casting="safe"))
        cast_value = np.asarray(source.fillvalue, dtype=output_dtype)
        needs_exact_value = (
            not safe_cast
            or source_value.dtype.kind in "iu"
            and output_dtype.kind == "f"
        )
        if needs_exact_value:
            roundtrip = np.asarray(cast_value, dtype=source_value.dtype)
            equal = bool(np.array_equal(roundtrip, source_value))
            if not equal and source_value.dtype.kind == "f":
                equal = bool(np.isnan(source_value).all() and np.isnan(roundtrip).all())
            if not equal:
                raise ValueError(
                    f"HDF5 action fill value cannot be represented as {output_dtype}"
                )
        if output_dtype.kind == "f" and source_value.dtype.kind in "fiu":
            if bool(np.isfinite(source_value).all()) and not bool(
                np.isfinite(cast_value).all()
            ):
                raise ValueError(
                    f"HDF5 action fill value overflows normalized dtype {output_dtype}"
                )
    except (TypeError, ValueError, OverflowError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith("HDF5 action"):
            raise
        raise ValueError("cannot inspect HDF5 action fill value") from exc
    return cast_value


def _creation_property_list(
    source: Any, output_dtype: Any, *, np: Any, h5py: Any
) -> Any:
    """Preserve lossless dataset creation properties when changing dtype."""

    try:
        dcpl = copy(source.id.get_create_plist())
        filter_ids = [
            int(dcpl.get_filter(index)[0]) for index in range(int(dcpl.get_nfilters()))
        ]
        scaleoffset_filter = int(h5py.h5z.FILTER_SCALEOFFSET)
        if scaleoffset_filter in filter_ids:
            dcpl.remove_filter(scaleoffset_filter)
        fill_status = int(dcpl.fill_value_defined())
        if fill_status == int(h5py.h5d.FILL_VALUE_USER_DEFINED):
            dcpl.set_fill_value(_representable_fill_value(source, output_dtype, np=np))
        elif fill_status not in (
            int(h5py.h5d.FILL_VALUE_DEFAULT),
            int(h5py.h5d.FILL_VALUE_UNDEFINED),
        ):
            raise ValueError(f"unsupported HDF5 action fill status: {fill_status}")
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
        raise ValueError(
            f"cannot preserve HDF5 action creation properties: {source.name}"
        ) from exc
    if source.chunks is not None:
        checked_shape_nbytes(
            source.chunks,
            output_dtype,
            resource=f"{source.name} normalized chunk",
            max_elements=MAX_ARRAY_ELEMENTS,
            max_bytes=MAX_ARRAY_BYTES,
        )
    return dcpl


def _create_normalized_dataset(
    parent: Any,
    name: str,
    source: Any,
    *,
    shape: tuple[int, ...],
    output_dtype: Any,
    np: Any,
    h5py: Any,
) -> Any:
    """Retain source creation properties and compatible dataspace semantics."""

    try:
        maxshape = tuple(
            h5py.h5s.UNLIMITED if dimension is None else int(dimension)
            for dimension in source.maxshape
        )
        dataspace = h5py.h5s.create_simple(shape, maxshape)
        datatype = h5py.h5t.py_create(output_dtype)
        dcpl = _creation_property_list(source, output_dtype, np=np, h5py=h5py)
        dataset_id = h5py.h5d.create(
            parent.id,
            name.encode("utf-8"),
            datatype,
            dataspace,
            dcpl=dcpl,
        )
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
        raise ValueError(
            f"cannot create normalized HDF5 action dataset: {source.name}"
        ) from exc
    return h5py.Dataset(dataset_id)


def _rewrite_dataset(
    h5: Any,
    declaration: _ActionDeclaration,
    *,
    minimum: Any,
    maximum: Any,
    scalar_dtype: Any,
    index: int,
    max_decoded_bytes: int,
    max_batch_bytes: int,
    metadata_budget: ByteBudget,
    h5py: Any,
    preserve_soft_link: bool,
    np: Any,
) -> _RewriteResult:
    parent_path, _, leaf = declaration.path.rpartition("/")
    if not parent_path or not leaf:
        raise ValueError(f"invalid HDF5 action path: {declaration.path}")
    parent = h5[parent_path]
    temporary_leaf = f"{_NORMALIZED_STAGE_PREFIX}{index:08d}_{leaf}"
    if temporary_leaf in parent:
        raise ValueError(
            f"HDF5 normalization temporary path already exists: {parent_path}"
        )
    source = h5[declaration.path]
    active = _refresh_declaration(
        declaration,
        source,
        scalar_dtype=scalar_dtype,
        max_decoded_bytes=max_decoded_bytes,
        max_batch_bytes=max_batch_bytes,
        np=np,
    )
    output_dtype = active.normalized_dtype
    assert output_dtype is not None
    attribute_declarations = _admit_action_attributes(
        source,
        metadata_budget=metadata_budget,
        np=np,
        h5py=h5py,
    )
    target = _create_normalized_dataset(
        parent,
        temporary_leaf,
        source,
        shape=active.shape,
        output_dtype=output_dtype,
        np=np,
        h5py=h5py,
    )
    completed = False
    try:
        _copy_action_attributes(
            source,
            target,
            attribute_declarations,
            metadata_budget=metadata_budget,
            np=np,
            h5py=h5py,
        )
        sample_count = active.shape[0]
        for start in range(0, sample_count, active.batch_rows):
            stop = min(start + active.batch_rows, sample_count)
            values = np.asarray(source[start:stop])
            if values.size == 0:
                del values
                continue
            normalized = _normalized_batch(
                values,
                declaration=active,
                minimum=minimum,
                maximum=maximum,
                scalar_dtype=scalar_dtype,
                np=np,
            )
            target[start:stop] = normalized
            del normalized
            del values
        h5.flush()
        if preserve_soft_link:
            anchor = (parent, temporary_leaf)
        else:
            del parent[leaf]
            parent.move(temporary_leaf, leaf)
            anchor = None
        completed = True
        return _RewriteResult(
            source_identity=_hdf5_object_identity(source, h5py),
            target=target,
            anchor=anchor,
        )
    finally:
        # The surrounding file publication stage owns the whole HDF5 copy.
        # Remove this temporary object when the provider lets us, while the
        # stage context remains the final cleanup authority on failure.
        if not completed:

            def discard_temporary() -> None:
                if temporary_leaf in parent:
                    del parent[temporary_leaf]

            _, cleanup_error = attempt_cleanup(discard_temporary)
            if cleanup_error is not None and not isinstance(
                cleanup_error, (KeyError, OSError, TypeError, ValueError)
            ):
                raise cleanup_error


def _relink_hard_aliases(
    h5: Any,
    edges_by_target: dict[int, tuple[_HardLinkEdge, ...]],
    replacements: dict[int, Any],
    *,
    canonical_edges: set[tuple[int, str]],
    h5py: Any,
) -> None:
    """Rebind non-action hard links to the replacement dataset."""

    resolved: dict[int, Any] = {}

    def replacement_for(source_identity: int) -> Any:
        cached = resolved.get(source_identity)
        if cached is not None:
            return cached
        target = replacements[source_identity]
        chain: list[int] = [source_identity]
        visited: set[int] = {source_identity}
        while True:
            target_identity = _hdf5_object_identity(target, h5py)
            if target_identity in visited:
                raise ValueError(
                    "HDF5 normalization replacement chain contains a cycle"
                )
            visited.add(target_identity)
            chain.append(target_identity)
            next_target = replacements.get(target_identity)
            if next_target is None:
                break
            target = next_target
            if len(chain) > MAX_DIRECTORY_ENTRIES:
                raise ResourceLimitError(
                    "HDF5 normalization replacement chain",
                    limit=MAX_DIRECTORY_ENTRIES,
                    observed=len(chain),
                )
        for identity in chain:
            resolved[identity] = target
        return target

    for source_identity, target in replacements.items():
        target = replacement_for(source_identity)
        for edge in edges_by_target.get(source_identity, ()):
            if edge.identity in canonical_edges:
                continue
            try:
                link = edge.parent.get(edge.name, getlink=True)
            except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"cannot inspect HDF5 hard alias {edge.name!r}"
                ) from exc
            if not isinstance(link, h5py.HardLink):
                continue
            try:
                current = edge.parent[edge.name]
                current_identity = _hdf5_object_identity(current, h5py)
            except (KeyError, OSError, RuntimeError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"cannot resolve HDF5 hard alias {edge.name!r}"
                ) from exc
            if current_identity != source_identity:
                continue
            del edge.parent[edge.name]
            edge.parent[edge.name] = target
    h5.flush()


def _discard_normalization_anchors(results: list[_RewriteResult]) -> None:

    for result in results:
        if result.anchor is None:
            continue
        parent, name = result.anchor
        if name in parent:
            del parent[name]


def _parameter_payload(minimum: Any, maximum: Any) -> bytes:
    payload = f"min: {minimum}\nmax: {maximum}\n".encode("utf-8")
    if len(payload) > _PARAMETER_FILE_BYTES:
        raise ResourceLimitError(
            "robomimic normalization parameter bytes",
            limit=_PARAMETER_FILE_BYTES,
            observed=len(payload),
        )
    return payload


def _current_sha256(path: Path) -> str | None:
    if not path.parent.is_dir():
        return None
    with ArtifactPublication.file(path).open_snapshot(
        max_bytes=MAX_EXTERNAL_FILE_BYTES
    ) as snapshot:
        return None if snapshot is None else snapshot.sha256


def _current_sha256_locked(path: Path) -> str | None:
    """Hash a destination while its publication lease is held."""

    no_follow = getattr(os, "O_NOFOLLOW", None)
    if no_follow is None:
        raise PublicationError("normalization rollback requires O_NOFOLLOW")
    descriptor: int | None = None
    try:
        try:
            descriptor = os.open(
                path,
                os.O_RDONLY
                | getattr(os, "O_CLOEXEC", 0)
                | getattr(os, "O_NONBLOCK", 0)
                | no_follow,
            )
        except FileNotFoundError:
            return None
        except OSError as exc:
            raise PublicationError(
                f"cannot open normalization rollback target: {path}"
            ) from exc
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise PublicationError(
                f"normalization rollback target is not regular: {path}"
            )
        if before.st_size > MAX_EXTERNAL_FILE_BYTES:
            raise ResourceLimitError(
                "robomimic rollback publication bytes",
                limit=MAX_EXTERNAL_FILE_BYTES,
                observed=before.st_size,
            )
        digest = hashlib.sha256()
        total = 0
        while True:
            read_size = max(
                1,
                min(1 << 20, MAX_EXTERNAL_FILE_BYTES - total + 1),
            )
            block = os.read(descriptor, read_size)
            if not block:
                break
            total += len(block)
            if total > MAX_EXTERNAL_FILE_BYTES:
                raise ResourceLimitError(
                    "robomimic rollback publication bytes",
                    limit=MAX_EXTERNAL_FILE_BYTES,
                    observed=total,
                )
            digest.update(block)
        after = os.fstat(descriptor)
        try:
            current_stat = os.stat(path, follow_symlinks=False)
        except FileNotFoundError as exc:
            raise PublicationError(
                f"normalization rollback target disappeared while checking: {path}"
            ) from exc
        if (
            not stat.S_ISREG(current_stat.st_mode)
            or before.st_dev != after.st_dev
            or before.st_ino != after.st_ino
            or before.st_size != after.st_size
            or current_stat.st_dev != after.st_dev
            or current_stat.st_ino != after.st_ino
            or total != after.st_size
        ):
            raise PublicationError(
                f"normalization rollback target changed while checking: {path}"
            )
        return digest.hexdigest()
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _regular_file_identity(path: Path) -> tuple[int, int]:
    """Identify one regular file generation without following symlinks."""

    no_follow = getattr(os, "O_NOFOLLOW", None)
    if no_follow is None:
        raise PublicationError("normalization generation identity requires O_NOFOLLOW")
    descriptor: int | None = None
    try:
        try:
            descriptor = os.open(
                path,
                os.O_RDONLY
                | getattr(os, "O_CLOEXEC", 0)
                | getattr(os, "O_NONBLOCK", 0)
                | no_follow,
            )
        except OSError as exc:
            raise PublicationError(
                f"cannot open normalization generation identity: {path}"
            ) from exc
        value = os.fstat(descriptor)
        if not stat.S_ISREG(value.st_mode):
            raise PublicationError(
                f"normalization generation is not a regular file: {path}"
            )
        return value.st_dev, value.st_ino
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _unlink_if_matches_locked(path: Path, *, expected_sha256: str) -> None:
    current = _current_sha256_locked(path)
    if current is None:
        return
    if current != expected_sha256:
        raise PublicationError(
            f"cannot rollback replaced publication with unexpected identity: {path}"
        )
    os.unlink(path)


def _remove_if_current_locked(path: Path, *, expected_sha256: str | None) -> None:
    if expected_sha256 is None:
        return
    current = _current_sha256_locked(path)
    if current != expected_sha256:
        raise PublicationError(
            f"normalization parameter publication changed before removal: {path}"
        )
    _unlink_if_matches_locked(path, expected_sha256=expected_sha256)


def _copy_backup_to_stage(
    source: Path,
    destination: Path,
    *,
    expected_sha256: str,
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES,
) -> None:

    no_follow = getattr(os, "O_NOFOLLOW", None)
    if no_follow is None:
        raise PublicationError("normalization rollback requires O_NOFOLLOW")
    with ArtifactPublication.file(source).open_snapshot(
        expected_sha256=expected_sha256,
        max_bytes=max_bytes,
    ) as snapshot:
        if snapshot is None:
            raise PublicationError(f"rollback backup is missing: {source}")
        descriptor = os.open(
            destination,
            os.O_WRONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NONBLOCK", 0)
            | no_follow,
        )
        try:
            before = os.fstat(descriptor)
            if not stat.S_ISREG(before.st_mode):
                raise PublicationError(
                    f"normalization rollback stage is not regular: {destination}"
                )
            os.ftruncate(descriptor, 0)
            digest = hashlib.sha256()
            total = 0
            while block := snapshot.file.read(1 << 20):
                total += len(block)
                if total > max_bytes:
                    raise ResourceLimitError(
                        "robomimic rollback publication bytes",
                        limit=max_bytes,
                        observed=total,
                    )
                digest.update(block)
                view = memoryview(block)
                while view:
                    written = os.write(descriptor, view)
                    if written <= 0:
                        raise PublicationError(
                            f"could not copy rollback backup: {source}"
                        )
                    view = view[written:]
            if digest.hexdigest() != expected_sha256:
                raise PublicationError(
                    f"rollback backup changed while copying: {source}"
                )
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def _restore_publication(
    *,
    path: Path,
    previous_sha256: str | None,
    previous_identity: tuple[int, int] | None,
    transformed_sha256: str | None,
    transformed_identity: tuple[int, int] | None,
    restored_identity: tuple[int, int] | None,
    backup_path: Path | None,
    backup_sha256: str | None,
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES,
    stage: Any | None = None,
    restore_attempt: "_RestoreAttempt | None" = None,
) -> tuple[int, int] | None:
    if stage is None:
        with ArtifactPublication.file(path).stage() as owned_stage:
            return _restore_publication(
                path=path,
                previous_sha256=previous_sha256,
                previous_identity=previous_identity,
                transformed_sha256=transformed_sha256,
                transformed_identity=transformed_identity,
                restored_identity=restored_identity,
                backup_path=backup_path,
                backup_sha256=backup_sha256,
                max_bytes=max_bytes,
                stage=owned_stage,
                restore_attempt=restore_attempt,
            )

    current = _current_sha256_locked(path)
    if current == previous_sha256:
        if current is not None:
            current_identity = _regular_file_identity(path)
            if current_identity not in {previous_identity, restored_identity}:
                raise PublicationError(
                    f"cannot rollback publication with unexpected generation: {path}"
                )
            return current_identity
        return None
    if current != transformed_sha256:
        raise PublicationError(
            f"cannot rollback publication with unexpected identity: {path}"
        )
    if transformed_sha256 is not None:
        if transformed_identity is None:
            raise PublicationError(f"missing rollback generation identity for {path}")
        current_identity = _regular_file_identity(path)
        if current_identity != transformed_identity:
            raise PublicationError(
                f"cannot rollback publication with unexpected generation: {path}"
            )
    if backup_path is None:
        if transformed_sha256 is None:
            raise PublicationError(
                f"missing rollback identity for absent publication: {path}"
            )
        _unlink_if_matches_locked(path, expected_sha256=transformed_sha256)
        return None
    if backup_sha256 is None:
        raise PublicationError(f"missing rollback digest for {path}")
    _copy_backup_to_stage(
        backup_path,
        stage.path,
        expected_sha256=backup_sha256,
        max_bytes=max_bytes,
    )
    backup_identity = _regular_file_identity(stage.path)
    if restore_attempt is not None:
        restore_attempt.resulting_identity = backup_identity
    # Rebind the destination after the backup copy, still under the same
    # publication lease.  A direct external replacement therefore fails
    # closed instead of being overwritten by rollback.
    if _current_sha256_locked(path) != transformed_sha256:
        raise PublicationError(
            f"cannot rollback publication after destination replacement: {path}"
        )
    stage.publish(
        expected_sha256=backup_sha256,
        max_bytes=max_bytes,
    )
    return backup_identity


class _RollbackCleanupFailure(RuntimeError):
    def __init__(self, errors: tuple[BaseException, ...]) -> None:
        self.errors = errors
        detail = "; ".join(str(error) for error in errors)
        super().__init__(detail or "normalization rollback cleanup failed")


@dataclass(frozen=True)
class _RollbackEntry:
    path: Path
    previous_sha256: str | None
    transformed_sha256: str | None
    backup_path: Path | None
    backup_sha256: str | None
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES
    previous_identity: tuple[int, int] | None = None
    transformed_identity: tuple[int, int] | None = None
    restored_identity: tuple[int, int] | None = None


@dataclass
class _RestoreAttempt:
    resulting_identity: tuple[int, int] | None = None


@dataclass
class _NormalizationRollbackOwner:
    entries: tuple[_RollbackEntry, ...]
    previous_snapshot: Any | None
    pair_paths: tuple[Path, Path] | None = None
    output_restored: bool = False
    output_restored_sha256: str | None = None
    output_restored_identity: tuple[int, int] | None = None
    publication_cleanups: list[PublicationCleanupError] = field(default_factory=list)
    dependent_snapshots: list[Any] = field(default_factory=list)

    def retry_cleanup(self) -> None:
        _, rollback_error = attempt_cleanup(self._retry_entries)
        errors: list[BaseException] = []
        if rollback_error is not None:
            errors.extend(
                rollback_error.errors
                if isinstance(rollback_error, _RollbackCleanupFailure)
                else (rollback_error,)
            )
        pending: list[PublicationCleanupError] = []
        for failure in self.publication_cleanups:
            _, error = attempt_cleanup(failure.retry_cleanup)
            if error is not None:
                pending.append(failure)
                errors.append(error)
        self.publication_cleanups = pending
        if errors:
            raise _RollbackCleanupFailure(tuple(errors)) from errors[0]
        self.release_snapshot()

    def retain_publication_cleanup(self, error: BaseException) -> None:
        if isinstance(error, PublicationCleanupError) and not any(
            retained.owner is error.owner for retained in self.publication_cleanups
        ):
            self.publication_cleanups.append(error)

    def _retry_entries(self) -> None:
        if not self.entries:
            return
        if self.pair_paths is not None:
            self._retry_pair_cleanup()
            return

        errors: list[BaseException] = []
        pending: list[_RollbackEntry] = []
        for entry in self.entries:
            restore_attempt = _RestoreAttempt()
            _, error = attempt_cleanup(
                lambda: _restore_publication(
                    path=entry.path,
                    previous_sha256=entry.previous_sha256,
                    previous_identity=entry.previous_identity,
                    transformed_sha256=entry.transformed_sha256,
                    transformed_identity=entry.transformed_identity,
                    restored_identity=entry.restored_identity,
                    backup_path=entry.backup_path,
                    backup_sha256=entry.backup_sha256,
                    max_bytes=entry.max_bytes,
                    restore_attempt=restore_attempt,
                )
            )
            if error is not None:
                errors.append(error)
                self.retain_publication_cleanup(error)
                if (
                    isinstance(error, PublicationCleanupError)
                    and FilePublication(entry.path).path in error.committed_paths
                ):
                    continue
                if restore_attempt.resulting_identity is not None:
                    entry = replace(
                        entry,
                        restored_identity=restore_attempt.resulting_identity,
                    )
                pending.append(entry)
        self.entries = tuple(pending)
        if errors:
            raise _RollbackCleanupFailure(tuple(errors)) from errors[0]

    def _record_output_state(
        self,
        output_path: Path,
        output_entry: _RollbackEntry,
        resulting_identity: tuple[int, int] | None,
    ) -> None:
        current_sha256 = _current_sha256_locked(output_path)
        if current_sha256 != output_entry.previous_sha256:
            raise PublicationError(
                f"normalization output was not restored: {output_path}"
            )
        current_identity = (
            None if current_sha256 is None else _regular_file_identity(output_path)
        )
        if current_identity != resulting_identity:
            raise PublicationError(
                f"normalization output changed after rollback: {output_path}"
            )
        self.output_restored = True
        self.output_restored_sha256 = current_sha256
        self.output_restored_identity = current_identity

    def _output_guard_error(
        self,
        output_path: Path,
        output_entry: _RollbackEntry | None,
    ) -> BaseException | None:
        current_sha256 = _current_sha256_locked(output_path)
        if self.output_restored:
            if current_sha256 != self.output_restored_sha256:
                return PublicationError(
                    "cannot restore absent parameters after output replacement"
                )
            if current_sha256 is not None:
                expected_identity = self.output_restored_identity
                if (
                    expected_identity is None
                    or _regular_file_identity(output_path) != expected_identity
                ):
                    return PublicationError(
                        "cannot restore absent parameters after output generation change"
                    )
            return None

        if output_entry is None or output_entry.transformed_sha256 is None:
            return PublicationError("missing paired output rollback identity")
        if current_sha256 != output_entry.transformed_sha256:
            return PublicationError(
                "cannot restore absent parameters after output replacement"
            )
        expected_identity = output_entry.transformed_identity
        if expected_identity is None or _regular_file_identity(output_path) != (
            expected_identity
        ):
            return PublicationError(
                "cannot restore absent parameters after output generation change"
            )
        return None

    def _retry_pair_cleanup(self) -> None:
        assert self.pair_paths is not None
        output_path, parameter_path = self.pair_paths
        output_publication = FilePublication(output_path)
        parameter_publication = FilePublication(parameter_path)
        canonical_output = output_publication.path
        canonical_parameter = parameter_publication.path
        output_entry = next(
            (
                entry
                for entry in self.entries
                if FilePublication(entry.path).path == canonical_output
            ),
            None,
        )
        errors: list[BaseException] = []
        pending = list(self.entries)

        def run_pair_cleanup() -> None:
            with stage_file_publications(
                (output_publication, parameter_publication)
            ) as stages:
                stage_by_path = {
                    canonical_output: stages[0],
                    canonical_parameter: stages[1],
                }
                expected_output_sha256 = (
                    output_entry.previous_sha256
                    if output_entry is not None
                    else self.output_restored_sha256
                )
                parameter_entry = next(
                    (
                        entry
                        for entry in self.entries
                        if FilePublication(entry.path).path == canonical_parameter
                    ),
                    None,
                )
                current_output_sha256 = _current_sha256_locked(canonical_output)
                current_parameter_sha256 = _current_sha256_locked(canonical_parameter)
                already_previous = (
                    output_entry is None
                    or current_output_sha256 == expected_output_sha256
                ) and (
                    parameter_entry is None
                    or current_parameter_sha256 == parameter_entry.previous_sha256
                )
                if already_previous:
                    pending.clear()
                else:
                    for entry in self.entries:
                        canonical_entry = FilePublication(entry.path).path
                        stage = stage_by_path.get(canonical_entry)
                        if stage is None:
                            errors.append(
                                PublicationError(
                                    f"missing paired rollback stage for {entry.path}"
                                )
                            )
                            continue
                        restore_attempt = _RestoreAttempt()

                        def restore_entry() -> tuple[int, int] | None:
                            if (
                                canonical_entry == canonical_parameter
                                and entry.transformed_sha256 is None
                            ):
                                guard_error = self._output_guard_error(
                                    canonical_output,
                                    output_entry,
                                )
                                if guard_error is not None:
                                    raise guard_error
                            resulting_identity = _restore_publication(
                                path=entry.path,
                                previous_sha256=entry.previous_sha256,
                                previous_identity=entry.previous_identity,
                                transformed_sha256=entry.transformed_sha256,
                                transformed_identity=entry.transformed_identity,
                                restored_identity=entry.restored_identity,
                                backup_path=entry.backup_path,
                                backup_sha256=entry.backup_sha256,
                                max_bytes=entry.max_bytes,
                                stage=stage,
                                restore_attempt=restore_attempt,
                            )
                            if canonical_entry == canonical_output:
                                self._record_output_state(
                                    canonical_output,
                                    entry,
                                    resulting_identity,
                                )
                            return resulting_identity

                        _, error = attempt_cleanup(restore_entry)
                        if error is None:
                            pending.remove(entry)
                        else:
                            errors.append(error)
                            self.retain_publication_cleanup(error)
                            if (
                                isinstance(error, PublicationCleanupError)
                                and canonical_entry in error.committed_paths
                            ):
                                if canonical_entry == canonical_output:
                                    self._record_output_state(
                                        canonical_output,
                                        entry,
                                        restore_attempt.resulting_identity,
                                    )
                                pending.remove(entry)
                                continue
                            if restore_attempt.resulting_identity is not None:
                                replacement = replace(
                                    entry,
                                    restored_identity=restore_attempt.resulting_identity,
                                )
                                pending[pending.index(entry)] = replacement

        _, scope_error = attempt_cleanup(run_pair_cleanup)
        if scope_error is not None:
            errors.append(scope_error)
            self.retain_publication_cleanup(scope_error)
        self.entries = tuple(pending)
        if errors:
            raise _RollbackCleanupFailure(tuple(errors)) from errors[0]

    def release_snapshot(self) -> None:
        snapshot = self.previous_snapshot
        if snapshot is not None:
            snapshot.close()
            self.previous_snapshot = None
        while self.dependent_snapshots:
            self.dependent_snapshots[0].close()
            del self.dependent_snapshots[0]


class RobomimicNormalizationCleanupError(RuntimeError):
    """A failed publication rollback retains an owner for retry."""

    def __init__(
        self,
        message: str,
        *,
        owner: _NormalizationRollbackOwner,
        errors: tuple[BaseException, ...],
    ) -> None:
        self.owner = owner
        self.cleanup_owner = owner
        self.errors = errors
        detail = "; ".join(f"{type(error).__name__}: {error}" for error in errors)
        super().__init__(f"{message}: {detail}" if detail else message)

    def retry_cleanup(self) -> None:

        _, error = attempt_cleanup(self.owner.retry_cleanup)
        if error is not None:
            errors = (
                error.errors if isinstance(error, _RollbackCleanupFailure) else (error,)
            )
            raise RobomimicNormalizationCleanupError(
                "normalization publication rollback remains incomplete",
                owner=self.owner,
                errors=tuple(errors),
            ) from error


def _close_normalization_snapshot(
    snapshot: Any,
    *,
    primary: BaseException | None,
) -> None:
    """Retain failed cleanup without replacing the operation error."""

    _, cleanup_error = attempt_cleanup(snapshot.close)
    if cleanup_error is None:
        return
    owner = _NormalizationRollbackOwner(entries=(), previous_snapshot=snapshot)
    if primary is not None:
        owner.retain_publication_cleanup(primary)
    raise RobomimicNormalizationCleanupError(
        "normalization snapshot cleanup failed",
        owner=owner,
        errors=(cleanup_error,) if primary is None else (primary, cleanup_error),
    ) from (cleanup_error if primary is None else primary)


def _publish_transformed_pair(
    *,
    output_path: Path,
    materialized: Path,
    parameter_path: Path,
    parameter_payload: bytes,
    previous_output_sha256: str | None,
    previous_parameter_sha256: str | None,
    previous_snapshot: Any,
) -> None:
    """Publish output and parameters under one grouped lease.

    Digest and generation checks reject cooperating writers before promotion; rollback
    leaves an already-restored previous pair untouched.
    """

    if _same_path(parameter_path, output_path):
        raise ValueError("normalization output cannot be the parameter file")
    parameter_private = materialized.with_name("normalization_params.txt")
    ArtifactPublication.file(parameter_private).publish_bytes(
        parameter_payload,
        max_bytes=_PARAMETER_FILE_BYTES,
    )
    transformed_sha256 = sha256_file(
        materialized,
        chunk_size=max(1, min(1 << 20, MAX_EXTERNAL_FILE_BYTES)),
        byte_budget=ByteBudget(
            "robomimic transformed output bytes", MAX_EXTERNAL_FILE_BYTES
        ),
    )
    parameter_sha256 = hashlib.sha256(parameter_payload).hexdigest()
    previous_output = (
        None
        if previous_snapshot is None or previous_output_sha256 is None
        else previous_snapshot.path / "previous-output.hdf5"
    )
    previous_parameter = (
        None
        if previous_snapshot is None or previous_parameter_sha256 is None
        else previous_snapshot.path / "previous-parameters.txt"
    )
    rollback_owner = _NormalizationRollbackOwner(
        entries=(
            _RollbackEntry(
                path=output_path,
                previous_sha256=previous_output_sha256,
                transformed_sha256=transformed_sha256,
                backup_path=previous_output,
                backup_sha256=previous_output_sha256,
                max_bytes=MAX_EXTERNAL_FILE_BYTES,
            ),
            _RollbackEntry(
                path=parameter_path,
                previous_sha256=previous_parameter_sha256,
                transformed_sha256=parameter_sha256,
                backup_path=previous_parameter,
                backup_sha256=previous_parameter_sha256,
                max_bytes=_PARAMETER_FILE_BYTES,
            ),
        ),
        previous_snapshot=previous_snapshot,
        pair_paths=(output_path, parameter_path),
    )
    promotion_started = False
    completed = False
    try:
        with stage_file_publications(
            (FilePublication(output_path), FilePublication(parameter_path))
        ) as (
            output_stage,
            parameter_stage,
        ):
            current_output_sha256 = _current_sha256_locked(output_path)
            current_parameter_sha256 = _current_sha256_locked(parameter_path)
            current_output_identity = (
                None
                if current_output_sha256 is None
                else _regular_file_identity(output_path)
            )
            current_parameter_identity = (
                None
                if current_parameter_sha256 is None
                else _regular_file_identity(parameter_path)
            )
            if (
                current_output_sha256 != previous_output_sha256
                or current_parameter_sha256 != previous_parameter_sha256
            ):
                raise PublicationError(
                    "normalization publication pair changed before promotion"
                )
            _copy_backup_to_stage(
                materialized,
                output_stage.path,
                expected_sha256=transformed_sha256,
                max_bytes=MAX_EXTERNAL_FILE_BYTES,
            )
            _copy_backup_to_stage(
                parameter_private,
                parameter_stage.path,
                expected_sha256=parameter_sha256,
                max_bytes=_PARAMETER_FILE_BYTES,
            )
            output_entry, parameter_entry = rollback_owner.entries
            rollback_owner.entries = (
                replace(
                    output_entry,
                    previous_identity=current_output_identity,
                    transformed_identity=_regular_file_identity(output_stage.path),
                ),
                replace(
                    parameter_entry,
                    previous_identity=current_parameter_identity,
                    transformed_identity=_regular_file_identity(parameter_stage.path),
                ),
            )
            promotion_started = True
            output_stage.publish(
                expected_sha256=transformed_sha256,
                max_bytes=MAX_EXTERNAL_FILE_BYTES,
            )
            parameter_stage.publish(
                expected_sha256=parameter_sha256,
                max_bytes=_PARAMETER_FILE_BYTES,
            )
        completed = True
    finally:
        if not completed:
            original = sys.exc_info()[1]
            if original is not None and promotion_started:
                rollback_owner.retain_publication_cleanup(original)
                _, cleanup_error = attempt_cleanup(rollback_owner.retry_cleanup)
                if cleanup_error is not None:
                    errors = (
                        cleanup_error.errors
                        if isinstance(cleanup_error, _RollbackCleanupFailure)
                        else (cleanup_error,)
                    )
                    raise RobomimicNormalizationCleanupError(
                        "normalization publication rollback failed",
                        owner=rollback_owner,
                        errors=(original, *errors),
                    ) from original
        else:
            rollback_owner.entries = ()
            _, cleanup_error = attempt_cleanup(rollback_owner.release_snapshot)
            if cleanup_error is not None:
                raise RobomimicNormalizationCleanupError(
                    "normalization previous snapshot cleanup failed",
                    owner=rollback_owner,
                    errors=(cleanup_error,),
                ) from cleanup_error


def _publish_output_without_parameters(
    *,
    output_path: Path,
    materialized: Path,
    parameter_path: Path,
    previous_output_sha256: str | None,
    previous_parameter_sha256: str | None,
    previous_snapshot: Any | None,
) -> None:
    """Publish an unchanged-action copy and remove stale transform stats.

    Admission checks destination digests and generation; rollback uses the same
    boundary.
    """

    transformed_sha256 = sha256_file(
        materialized,
        chunk_size=max(1, min(1 << 20, MAX_EXTERNAL_FILE_BYTES)),
        byte_budget=ByteBudget(
            "robomimic transformed output bytes", MAX_EXTERNAL_FILE_BYTES
        ),
    )
    previous_output = (
        None
        if previous_snapshot is None or previous_output_sha256 is None
        else previous_snapshot.path / "previous-output.hdf5"
    )
    previous_parameter = (
        None
        if previous_snapshot is None or previous_parameter_sha256 is None
        else previous_snapshot.path / "previous-parameters.txt"
    )
    rollback_owner = _NormalizationRollbackOwner(
        entries=(
            _RollbackEntry(
                path=output_path,
                previous_sha256=previous_output_sha256,
                transformed_sha256=transformed_sha256,
                backup_path=previous_output,
                backup_sha256=previous_output_sha256,
                max_bytes=MAX_EXTERNAL_FILE_BYTES,
            ),
            _RollbackEntry(
                path=parameter_path,
                previous_sha256=previous_parameter_sha256,
                transformed_sha256=None,
                backup_path=previous_parameter,
                backup_sha256=previous_parameter_sha256,
                max_bytes=_PARAMETER_FILE_BYTES,
            ),
        ),
        previous_snapshot=previous_snapshot,
        pair_paths=(output_path, parameter_path),
    )
    promotion_started = False
    completed = False
    try:
        with stage_file_publications(
            (FilePublication(output_path), FilePublication(parameter_path))
        ) as (
            output_stage,
            parameter_stage,
        ):
            current_output_sha256 = _current_sha256_locked(output_path)
            current_parameter_sha256 = _current_sha256_locked(parameter_path)
            current_output_identity = (
                None
                if current_output_sha256 is None
                else _regular_file_identity(output_path)
            )
            current_parameter_identity = (
                None
                if current_parameter_sha256 is None
                else _regular_file_identity(parameter_path)
            )
            if (
                current_output_sha256 != previous_output_sha256
                or current_parameter_sha256 != previous_parameter_sha256
            ):
                raise PublicationError(
                    "normalization publication pair changed before promotion"
                )
            _copy_backup_to_stage(
                materialized,
                output_stage.path,
                expected_sha256=transformed_sha256,
                max_bytes=MAX_EXTERNAL_FILE_BYTES,
            )
            output_entry, parameter_entry = rollback_owner.entries
            rollback_owner.entries = (
                replace(
                    output_entry,
                    previous_identity=current_output_identity,
                    transformed_identity=_regular_file_identity(output_stage.path),
                ),
                replace(
                    parameter_entry,
                    previous_identity=current_parameter_identity,
                ),
            )
            promotion_started = True
            output_stage.publish(
                expected_sha256=transformed_sha256,
                max_bytes=MAX_EXTERNAL_FILE_BYTES,
            )
            _remove_if_current_locked(
                parameter_path,
                expected_sha256=previous_parameter_sha256,
            )
        completed = True
    finally:
        if not completed:
            original = sys.exc_info()[1]
            if original is not None and promotion_started:
                rollback_owner.retain_publication_cleanup(original)
                _, cleanup_error = attempt_cleanup(rollback_owner.retry_cleanup)
                if cleanup_error is not None:
                    errors = (
                        cleanup_error.errors
                        if isinstance(cleanup_error, _RollbackCleanupFailure)
                        else (cleanup_error,)
                    )
                    raise RobomimicNormalizationCleanupError(
                        "normalization publication rollback failed",
                        owner=rollback_owner,
                        errors=(original, *errors),
                    ) from original
        else:
            rollback_owner.entries = ()
            _, cleanup_error = attempt_cleanup(rollback_owner.release_snapshot)
            if cleanup_error is not None:
                raise RobomimicNormalizationCleanupError(
                    "normalization previous snapshot cleanup failed",
                    owner=rollback_owner,
                    errors=(cleanup_error,),
                ) from cleanup_error


def _transform_materialized_hdf5(
    stream: Any,
    *,
    max_decoded_bytes: int,
    max_batch_bytes: int,
) -> bytes | None:
    """Transform an owned descriptor within the bounded native process."""
    np = _require_numpy()
    h5py = _require_h5py()
    decoded_byte_limit = _decoded_byte_limit(max_decoded_bytes)
    batch_byte_limit = _batch_byte_limit(max_batch_bytes)
    normalized = False
    minimum: Any | None = None
    maximum: Any | None = None
    with h5py.File(stream, "r+") as h5:
        _preflight_storage(h5, h5py)
        paths = _action_paths(h5, h5py)
        initial = [
            _initial_declaration(path, dataset, np=np) for path, dataset in paths
        ]
        if initial:
            edges_by_target = _hard_link_edges(h5, h5py)
            canonical_edges: set[tuple[int, str]] = set()
            initial_source_identities: list[int] = []
            for path, dataset in paths:
                parent_path, _, leaf = path.rpartition("/")
                parent = h5[parent_path or "/"]
                canonical_edges.add((_hdf5_object_identity(parent, h5py), leaf))
                initial_source_identities.append(_hdf5_object_identity(dataset, h5py))
            last_initial_source: dict[int, int] = {}
            for index, source_identity in enumerate(initial_source_identities):
                last_initial_source[source_identity] = index
            noncanonical_edges_by_target = {
                source_identity: {
                    edge.identity
                    for edge in edges
                    if edge.identity not in canonical_edges
                }
                for source_identity, edges in edges_by_target.items()
            }
            declarations, scalar_dtype = _complete_declarations(
                initial,
                np=np,
                max_decoded_bytes=decoded_byte_limit,
                max_batch_bytes=batch_byte_limit,
            )
            transformed_size = os.fstat(stream.fileno()).st_size + sum(
                declaration.normalized_bytes for declaration in declarations
            )
            if transformed_size > MAX_EXTERNAL_FILE_BYTES:
                raise ResourceLimitError(
                    "robomimic normalized publication bytes",
                    limit=MAX_EXTERNAL_FILE_BYTES,
                    observed=transformed_size,
                )
            minimum, maximum = _global_extrema(
                h5,
                declarations,
                scalar_dtype=scalar_dtype,
                np=np,
            )
            if (
                minimum is not None
                and maximum is not None
                and not bool(maximum == minimum)
            ):
                metadata_budget = ByteBudget(
                    "robomimic normalization action metadata",
                    MAX_METADATA_AGGREGATE_BYTES,
                )
                rewrite_results: list[_RewriteResult] = []
                replacements: dict[int, Any] = {}
                for index, declaration in enumerate(declarations):
                    parent_path, _, leaf = declaration.path.rpartition("/")
                    parent = h5[parent_path or "/"]
                    action_link = parent.get(leaf, getlink=True)
                    source_identity = _hdf5_object_identity(h5[declaration.path], h5py)
                    soft_target = _soft_target_edge(
                        h5,
                        parent,
                        action_link,
                        h5py=h5py,
                    )
                    preserve_soft_link = bool(
                        isinstance(action_link, h5py.SoftLink)
                        and soft_target
                        and soft_target
                        in noncanonical_edges_by_target.get(source_identity, ())
                        and last_initial_source.get(source_identity) == index
                    )
                    result = _rewrite_dataset(
                        h5,
                        declaration,
                        minimum=minimum,
                        maximum=maximum,
                        scalar_dtype=scalar_dtype,
                        index=index,
                        max_decoded_bytes=decoded_byte_limit,
                        max_batch_bytes=batch_byte_limit,
                        metadata_budget=metadata_budget,
                        h5py=h5py,
                        preserve_soft_link=preserve_soft_link,
                        np=np,
                    )
                    rewrite_results.append(result)
                    replacements[result.source_identity] = result.target
                _relink_hard_aliases(
                    h5,
                    edges_by_target,
                    replacements,
                    canonical_edges=canonical_edges,
                    h5py=h5py,
                )
                _discard_normalization_anchors(rewrite_results)
                h5.flush()
                normalized = True
        h5.flush()

    if normalized:
        assert minimum is not None and maximum is not None
        return _parameter_payload(minimum, maximum)
    return None


def normalize_hdf5_actions(
    dataset_path: str,
    log_dir: str,
    output_path: str | None = None,
    *,
    max_decoded_bytes: int | None = None,
    max_batch_bytes: int | None = None,
    native_limits: Hdf5NativeLimits | None = None,
) -> str:
    """Normalize demo actions across one global scalar range.

    Preserve the source. Optional byte limits can lower aggregate and batch
    allowances.
    """

    from srb.datasets.hdf5_native import (
        Hdf5NativeCleanupError,
        Hdf5NativeLimits,
        normalize_materialized_hdf5,
    )

    checked_native_limits = (
        Hdf5NativeLimits() if native_limits is None else native_limits
    )
    if not isinstance(checked_native_limits, Hdf5NativeLimits):
        raise TypeError("native_limits must be Hdf5NativeLimits or None")
    checked_native_limits = replace(
        checked_native_limits,
        max_array_bytes=min(checked_native_limits.max_array_bytes, MAX_ARRAY_BYTES),
        max_array_elements=min(
            checked_native_limits.max_array_elements, MAX_ARRAY_ELEMENTS
        ),
        max_entries=min(checked_native_limits.max_entries, MAX_DIRECTORY_ENTRIES),
        max_source_bytes=min(
            checked_native_limits.max_source_bytes, MAX_EXTERNAL_FILE_BYTES
        ),
        max_output_bytes=min(
            checked_native_limits.max_output_bytes, MAX_EXTERNAL_FILE_BYTES
        ),
        max_metadata_bytes=min(
            checked_native_limits.max_metadata_bytes, MAX_METADATA_AGGREGATE_BYTES
        ),
        max_attribute_bytes=min(
            checked_native_limits.max_attribute_bytes, MAX_METADATA_ATTRIBUTE_BYTES
        ),
        max_attributes=min(
            checked_native_limits.max_attributes, MAX_METADATA_ATTRIBUTES
        ),
    )
    decoded_byte_limit = _decoded_byte_limit(max_decoded_bytes)
    batch_byte_limit = _batch_byte_limit(max_batch_bytes)
    dataset_text = os.fspath(dataset_path)
    if isinstance(dataset_text, bytes):
        raise TypeError("dataset_path must be a text path")
    source = _as_path(dataset_path, role="dataset_path")
    if output_path is None:
        # Keep the historical returned spelling for relative inputs.  The
        # validated ``source`` path remains the operational path below.
        base, extension = os.path.splitext(dataset_text)
        output_text = base + "_normalized" + extension
    else:
        output_text = os.fspath(output_path)
        if isinstance(output_text, bytes):
            raise TypeError("output_path must be a text path")
    output = _as_path(output_text, role="output_path")
    log_path = _as_path(log_dir, role="log_dir")
    parameter_path = log_path / "normalization_params.txt"
    if _same_path(parameter_path, output):
        raise ValueError("normalization output cannot be the parameter file")
    if _same_path(parameter_path, source):
        # Publication replaces the parameter file, which would destroy the input.
        raise ValueError("normalization source cannot be the parameter file")
    if _same_path(source, output):
        raise ValueError("normalization output must differ from its source")

    print(f"Creating normalized dataset at {output_text}")
    # Hash once, then let the verified materialization seam copy the exact
    # digest into a private directory.  It rechecks source identity and bytes,
    # including same-size mutation, before this function opens HDF5.
    source_limit = checked_native_limits.max_source_bytes
    source_budget = ByteBudget("robomimic normalization source bytes", source_limit)
    source_sha256 = sha256_file(
        source,
        chunk_size=max(1, min(1 << 20, source_limit)),
        byte_budget=source_budget,
    )
    materialized_snapshot = materialize_file_snapshot(
        {"input.hdf5": (source, source_sha256)},
        max_bytes=source_limit,
    )
    cleanup_retained = False
    operation_completed = False
    try:
        materialized = materialized_snapshot.path / "input.hdf5"
        try:
            parameter_payload = normalize_materialized_hdf5(
                materialized,
                max_decoded_bytes=decoded_byte_limit,
                max_batch_bytes=batch_byte_limit,
                worker_script=Path(__file__).with_name("_normalization_worker.py"),
                limits=checked_native_limits,
            )
        except Hdf5NativeCleanupError as error:
            cleanup_retained = True
            materialized_snapshot.defer_finalizer_cleanup()
            error.retain_cleanup(materialized_snapshot.close)
            raise

        if parameter_payload is not None:
            previous_output_sha256 = _current_sha256(output)
            previous_parameter_sha256 = _current_sha256(parameter_path)
            unchanged_previous_members: dict[str, tuple[Path, str]] = {}
            if previous_output_sha256 is not None:
                unchanged_previous_members["previous-output.hdf5"] = (
                    output,
                    previous_output_sha256,
                )
            if previous_parameter_sha256 is not None:
                unchanged_previous_members["previous-parameters.txt"] = (
                    parameter_path,
                    previous_parameter_sha256,
                )
            previous_snapshot = None
            if unchanged_previous_members:
                previous_snapshot = materialize_file_snapshot(
                    unchanged_previous_members,
                    max_bytes=MAX_EXTERNAL_FILE_BYTES,
                )
                previous_snapshot.__enter__()
            publication_completed = False
            try:
                _publish_transformed_pair(
                    output_path=output,
                    materialized=materialized,
                    parameter_path=parameter_path,
                    parameter_payload=parameter_payload,
                    previous_output_sha256=previous_output_sha256,
                    previous_parameter_sha256=previous_parameter_sha256,
                    previous_snapshot=previous_snapshot,
                )
                publication_completed = True
            except RobomimicNormalizationCleanupError:
                # The recovery owner retains this snapshot until its retry
                # succeeds.  The outer materialized source context must not
                # close it on the way out.
                previous_snapshot = None
                raise
            finally:
                if previous_snapshot is not None:
                    _close_normalization_snapshot(
                        previous_snapshot,
                        primary=None if publication_completed else sys.exc_info()[1],
                    )
        else:
            previous_output_sha256 = _current_sha256(output)
            previous_parameter_sha256 = _current_sha256(parameter_path)
            previous_members: dict[str, tuple[Path, str]] = {}
            if previous_output_sha256 is not None:
                previous_members["previous-output.hdf5"] = (
                    output,
                    previous_output_sha256,
                )
            if previous_parameter_sha256 is not None:
                previous_members["previous-parameters.txt"] = (
                    parameter_path,
                    previous_parameter_sha256,
                )
            previous_snapshot = None
            if previous_members:
                previous_snapshot = materialize_file_snapshot(
                    previous_members,
                    max_bytes=MAX_EXTERNAL_FILE_BYTES,
                )
                previous_snapshot.__enter__()
            publication_completed = False
            try:
                _publish_output_without_parameters(
                    output_path=output,
                    materialized=materialized,
                    parameter_path=parameter_path,
                    previous_output_sha256=previous_output_sha256,
                    previous_parameter_sha256=previous_parameter_sha256,
                    previous_snapshot=previous_snapshot,
                )
                publication_completed = True
            except RobomimicNormalizationCleanupError:
                previous_snapshot = None
                raise
            finally:
                if previous_snapshot is not None:
                    _close_normalization_snapshot(
                        previous_snapshot,
                        primary=None if publication_completed else sys.exc_info()[1],
                    )
        operation_completed = True

    except RobomimicNormalizationCleanupError as error:
        cleanup_retained = True
        error.owner.dependent_snapshots.append(materialized_snapshot)
        raise
    except PublicationCleanupError as error:
        cleanup_retained = True
        owner = _NormalizationRollbackOwner(
            entries=(),
            previous_snapshot=materialized_snapshot,
            publication_cleanups=[error],
        )
        raise RobomimicNormalizationCleanupError(
            "normalization private publication cleanup failed",
            owner=owner,
            errors=(error,),
        ) from error
    finally:
        if not cleanup_retained:
            _close_normalization_snapshot(
                materialized_snapshot,
                primary=None if operation_completed else sys.exc_info()[1],
            )
    print(f"Normalized actions saved to {output_text}")
    return output_text


__all__ = [
    "RobomimicNormalizationCleanupError",
    "normalize_hdf5_actions",
]
