"""Overflow-safe resource ceilings for untrusted artifact boundaries."""

from __future__ import annotations

import math
import operator
import os
import zipfile
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO

GIB = 1 << 30
MAX_EXTERNAL_FILE_BYTES = 16 * GIB
MAX_IN_MEMORY_FILE_BYTES = 256 * (1 << 20)
MAX_DECODED_AGGREGATE_BYTES = 16 * GIB
MAX_PUBLICATION_FILE_BYTES = MAX_EXTERNAL_FILE_BYTES
MAX_PUBLICATION_DIRECTORY_BYTES = MAX_DECODED_AGGREGATE_BYTES
MAX_PUBLICATION_DIRECTORY_DEPTH = 32
MAX_ARRAY_BYTES = 2 * GIB
MAX_ARRAY_ELEMENTS = 268_435_456
MAX_ARCHIVE_MEMBERS = 4_096
MAX_DIRECTORY_ENTRIES = 100_000
MAX_PUBLICATION_DIRECTORY_ENTRIES = MAX_DIRECTORY_ENTRIES
#: Maximum entries visited by control-directory cleanup scans.
MAX_CLEANUP_SCAN_ENTRIES = MAX_DIRECTORY_ENTRIES
#: Maximum matching entries retained; unrelated siblings count only as visits.
MAX_CLEANUP_CANDIDATES = 1_024
MAX_EVENT_ROWS = 100_000_000
MAX_EVENT_HDF5_BYTES = 8 * GIB
MAX_EVENT_BATCH_ROWS = 1_000_000
MAX_EVT3_INPUT_BYTES = 8 * GIB
MAX_EVT3_FEED_BYTES = 4 * (1 << 20)
MAX_EVT3_FEED_WORDS = MAX_EVT3_FEED_BYTES // 2
MAX_EVT3_BUFFER_BYTES = 64 * (1 << 20)
MAX_EVT3_GAP_NS = 60_000_000_000
MAX_IMAGE_PIXELS = 67_108_864
MAX_METADATA_ATTRIBUTES = 256
MAX_METADATA_ATTRIBUTE_BYTES = 64 * (1 << 20)
MAX_METADATA_AGGREGATE_BYTES = 128 * (1 << 20)
MAX_JSON_NODES = 1_000_000
MAX_JSONL_ROW_NODES = 1_500_000
MAX_JSON_DEPTH = 64
MAX_JSON_AGGREGATE_NODES = 16_000_000
#: A landscaping target patch is a square float32 elevation grid; 256 MiB
#: is 8192x8192 cells, far beyond any usable patch resolution.
MAX_TARGET_MAP_BYTES = 256 * (1 << 20)
MAX_MESH_VERTICES = 5_000_000
MAX_MESH_FACES = 5_000_000
MAX_AUGMENTATION_ROWS = 5_000_000
MAX_ILLUMINATION_ARCHIVE_BYTES = 2 * GIB
MAX_ILLUMINATION_DECODED_BYTES = 2 * GIB
MAX_ILLUMINATION_ARRAY_BYTES = 512 * (1 << 20)


class ResourceLimitError(ValueError):
    """External input exceeds a declared finite resource ceiling."""

    def __init__(self, resource: str, *, limit: int, observed: int) -> None:
        self.resource = resource
        self.limit = limit
        self.observed = observed
        super().__init__(
            f"{resource} exceeds resource limit: observed {observed}, limit {limit}"
        )


def require_bounded_int(
    value: object,
    *,
    resource: str,
    minimum: int = 0,
    maximum: int,
) -> int:
    """Return one integer within inclusive bounds; reject bool and coercion."""
    if isinstance(value, bool):
        raise TypeError(f"{resource} must be an integer")
    try:
        parsed = operator.index(value)
    except TypeError as exc:
        raise TypeError(f"{resource} must be an integer") from exc
    if parsed < minimum:
        raise ValueError(f"{resource} must be >= {minimum}, got {parsed}")
    if parsed > maximum:
        raise ResourceLimitError(resource, limit=maximum, observed=parsed)
    return parsed


def checked_shape_nbytes(
    shape: Iterable[object],
    dtype: Any,
    *,
    resource: str,
    max_elements: int = MAX_ARRAY_ELEMENTS,
    max_bytes: int = MAX_ARRAY_BYTES,
    max_rank: int = 8,
) -> int:
    """Validate an array declaration and return its decoded byte count."""
    dimensions = tuple(shape)
    if len(dimensions) > max_rank:
        raise ResourceLimitError(
            f"{resource} rank", limit=max_rank, observed=len(dimensions)
        )
    elements = 1
    for index, raw_dimension in enumerate(dimensions):
        dimension = require_bounded_int(
            raw_dimension,
            resource=f"{resource} shape[{index}]",
            minimum=0,
            maximum=max_elements,
        )
        elements *= dimension
        if elements > max_elements:
            raise ResourceLimitError(
                f"{resource} elements", limit=max_elements, observed=elements
            )
    try:
        itemsize = int(dtype.itemsize)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise TypeError(f"{resource} has invalid dtype") from exc
    if itemsize <= 0 or not math.isfinite(float(itemsize)):
        raise ValueError(f"{resource} dtype itemsize must be positive")
    decoded = elements * itemsize
    if decoded > max_bytes:
        raise ResourceLimitError(resource, limit=max_bytes, observed=decoded)
    return decoded


def require_file_size(
    path: Path | str,
    *,
    resource: str,
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES,
) -> int:
    """Validate a regular file's current size before parsing it."""
    candidate = Path(path)
    if candidate.is_symlink() or not candidate.is_file():
        raise ValueError(f"{resource} is not a regular file: {candidate}")
    size = candidate.stat().st_size
    if size > max_bytes:
        raise ResourceLimitError(resource, limit=max_bytes, observed=size)
    return size


def require_dataset_file_size(
    path: Path | str,
    *,
    resource: str,
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES,
) -> int:
    """Validate a dataset file's size before a reader opens it.

    Same ceiling as :func:`require_file_size`, but symlinks are resolved
    rather than rejected: a dataset path routinely points into shared storage
    through a link, while the property that matters is that the file actually
    opened is a bounded regular file. The resolved target still has to be one.
    """

    return require_file_size(
        Path(path).expanduser().resolve(strict=False),
        resource=resource,
        max_bytes=max_bytes,
    )


def require_directory_budget(
    path: Path | str,
    *,
    resource: str,
    max_entries: int = MAX_DIRECTORY_ENTRIES,
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES,
    max_file_bytes: int = MAX_EXTERNAL_FILE_BYTES,
    max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
) -> tuple[int, int]:
    """Validate a no-symlink directory tree and return entries and bytes."""

    root = Path(path)
    if root.is_symlink() or not root.is_dir():
        raise ValueError(f"{resource} is not a regular directory: {root}")
    entry_limit = require_bounded_int(
        max_entries,
        resource=f"{resource} entry limit",
        minimum=1,
        maximum=MAX_DIRECTORY_ENTRIES,
    )
    byte_limit = require_bounded_int(
        max_bytes,
        resource=f"{resource} byte limit",
        minimum=1,
        maximum=MAX_EXTERNAL_FILE_BYTES,
    )
    file_limit = require_bounded_int(
        max_file_bytes,
        resource=f"{resource} file limit",
        minimum=1,
        maximum=MAX_EXTERNAL_FILE_BYTES,
    )
    depth_limit = require_bounded_int(
        max_depth,
        resource=f"{resource} depth limit",
        minimum=1,
        maximum=128,
    )
    entries = 0
    bytes_budget = ByteBudget(resource, byte_limit)
    pending: list[tuple[Path, int]] = [(root, 0)]
    while pending:
        directory, depth = pending.pop()
        with os.scandir(directory) as children:
            for child in children:
                entries += 1
                if entries > entry_limit:
                    raise ResourceLimitError(
                        f"{resource} entries",
                        limit=entry_limit,
                        observed=entries,
                    )
                if child.is_symlink():
                    raise ValueError(f"{resource} contains symlink: {child.path}")
                if child.is_dir(follow_symlinks=False):
                    if depth + 1 > depth_limit:
                        raise ResourceLimitError(
                            f"{resource} depth",
                            limit=depth_limit,
                            observed=depth + 1,
                        )
                    pending.append((Path(child.path), depth + 1))
                    continue
                if not child.is_file(follow_symlinks=False):
                    raise ValueError(
                        f"{resource} contains non-regular entry: {child.path}"
                    )
                size = child.stat(follow_symlinks=False).st_size
                if size > file_limit:
                    raise ResourceLimitError(
                        f"{resource} file {child.name!r}",
                        limit=file_limit,
                        observed=size,
                    )
                bytes_budget.consume(size)
    return entries, bytes_budget.consumed


@dataclass(slots=True)
class ByteBudget:
    """Monotone cumulative byte counter with a hard ceiling."""

    resource: str
    limit: int
    consumed: int = 0

    def __post_init__(self) -> None:
        self.limit = require_bounded_int(
            self.limit,
            resource=f"{self.resource} limit",
            minimum=0,
            maximum=MAX_DECODED_AGGREGATE_BYTES,
        )
        self.consumed = require_bounded_int(
            self.consumed,
            resource=f"{self.resource} consumed",
            minimum=0,
            maximum=self.limit,
        )

    def consume(self, amount: object) -> int:
        value = require_bounded_int(
            amount,
            resource=f"{self.resource} increment",
            minimum=0,
            maximum=self.limit,
        )
        observed = self.consumed + value
        if observed > self.limit:
            raise ResourceLimitError(self.resource, limit=self.limit, observed=observed)
        self.consumed = observed
        return observed


@dataclass(frozen=True, slots=True)
class NumpyArrayDeclaration:
    """One validated ``.npy`` member declaration inside an NPZ archive."""

    name: str
    shape: tuple[int, ...]
    dtype: Any
    fortran_order: bool
    decoded_bytes: int


def _seekable_size(source: BinaryIO) -> int:
    try:
        return os.fstat(source.fileno()).st_size
    except (AttributeError, OSError):
        position = source.tell()
        source.seek(0, os.SEEK_END)
        size = source.tell()
        source.seek(position)
        return size


def inspect_npz_archive(
    source: Path | str | BinaryIO,
    *,
    resource: str,
    max_archive_bytes: int = MAX_EXTERNAL_FILE_BYTES,
    max_decoded_bytes: int = MAX_DECODED_AGGREGATE_BYTES,
    max_members: int = MAX_ARCHIVE_MEMBERS,
    max_array_bytes: int = MAX_ARRAY_BYTES,
    max_array_elements: int = MAX_ARRAY_ELEMENTS,
) -> dict[str, NumpyArrayDeclaration]:
    """Validate NPZ member counts, decoded totals, and NPY headers only."""

    archive_limit = require_bounded_int(
        max_archive_bytes,
        resource=f"{resource} archive limit",
        minimum=1,
        maximum=MAX_EXTERNAL_FILE_BYTES,
    )
    decoded_limit = require_bounded_int(
        max_decoded_bytes,
        resource=f"{resource} decoded limit",
        minimum=1,
        maximum=MAX_DECODED_AGGREGATE_BYTES,
    )
    member_limit = require_bounded_int(
        max_members,
        resource=f"{resource} member limit",
        minimum=1,
        maximum=MAX_ARCHIVE_MEMBERS,
    )
    array_limit = require_bounded_int(
        max_array_bytes,
        resource=f"{resource} array limit",
        minimum=1,
        maximum=MAX_ARRAY_BYTES,
    )
    element_limit = require_bounded_int(
        max_array_elements,
        resource=f"{resource} element limit",
        minimum=1,
        maximum=MAX_ARRAY_ELEMENTS,
    )

    is_path = isinstance(source, (str, Path))
    if is_path:
        size = require_file_size(
            Path(source),
            resource=f"{resource} archive",
            max_bytes=archive_limit,
        )
        archive_source: Path | BinaryIO = Path(source)
        original_position: int | None = None
    else:
        archive_source = source
        original_position = source.tell()
        size = _seekable_size(source)
        source.seek(0)
        if size > archive_limit:
            raise ResourceLimitError(
                f"{resource} archive",
                limit=archive_limit,
                observed=size,
            )

    decoded_budget = ByteBudget(f"{resource} decoded bytes", decoded_limit)
    declarations: dict[str, NumpyArrayDeclaration] = {}
    try:
        with zipfile.ZipFile(archive_source) as archive:
            infos = archive.infolist()
            if len(infos) > member_limit:
                raise ResourceLimitError(
                    f"{resource} archive members",
                    limit=member_limit,
                    observed=len(infos),
                )
            for info in infos:
                if info.is_dir() or not info.filename.endswith(".npy"):
                    raise ValueError(
                        f"{resource} contains non-array member {info.filename!r}"
                    )
                if info.flag_bits & 0x1:
                    raise ValueError(
                        f"{resource} contains encrypted member {info.filename!r}"
                    )
                decoded_budget.consume(info.file_size)
                key = info.filename[:-4]
                if not key or key in declarations:
                    raise ValueError(f"{resource} contains duplicate array key {key!r}")
                with archive.open(info, "r") as member:
                    import numpy as np

                    version = np.lib.format.read_magic(member)
                    if version == (1, 0):
                        shape, fortran_order, dtype = (
                            np.lib.format.read_array_header_1_0(
                                member,
                                max_header_size=10_000,
                            )
                        )
                    elif version == (2, 0):
                        shape, fortran_order, dtype = (
                            np.lib.format.read_array_header_2_0(
                                member,
                                max_header_size=10_000,
                            )
                        )
                    else:
                        raise ValueError(
                            f"{resource} member {info.filename!r} uses unsupported "
                            f"NPY version {version}"
                        )
                    dtype = np.dtype(dtype)
                    if dtype.hasobject:
                        raise ValueError(
                            f"{resource} member {info.filename!r} has object dtype"
                        )
                    decoded_bytes = checked_shape_nbytes(
                        shape,
                        dtype,
                        resource=f"{resource} array {key!r}",
                        max_elements=element_limit,
                        max_bytes=array_limit,
                    )
                    payload_bytes = info.file_size - member.tell()
                    if payload_bytes != decoded_bytes:
                        raise ValueError(
                            f"{resource} member {info.filename!r} payload size "
                            f"{payload_bytes} does not match declared array bytes "
                            f"{decoded_bytes}"
                        )
                declarations[key] = NumpyArrayDeclaration(
                    name=key,
                    shape=tuple(int(value) for value in shape),
                    dtype=dtype,
                    fortran_order=bool(fortran_order),
                    decoded_bytes=decoded_bytes,
                )
    finally:
        if original_position is not None:
            source.seek(original_position)
    return declarations


def inspect_npy_array(
    source: Path | str | BinaryIO,
    *,
    resource: str,
    max_file_bytes: int = MAX_EXTERNAL_FILE_BYTES,
    max_array_bytes: int = MAX_ARRAY_BYTES,
    max_array_elements: int = MAX_ARRAY_ELEMENTS,
) -> NumpyArrayDeclaration:
    """Validate one NPY header and exact payload without materializing it."""

    file_limit = require_bounded_int(
        max_file_bytes,
        resource=f"{resource} file limit",
        minimum=1,
        maximum=MAX_EXTERNAL_FILE_BYTES,
    )
    array_limit = require_bounded_int(
        max_array_bytes,
        resource=f"{resource} array limit",
        minimum=1,
        maximum=MAX_ARRAY_BYTES,
    )
    element_limit = require_bounded_int(
        max_array_elements,
        resource=f"{resource} element limit",
        minimum=1,
        maximum=MAX_ARRAY_ELEMENTS,
    )

    if isinstance(source, (str, Path)):
        path = Path(source)
        require_file_size(path, resource=f"{resource} file", max_bytes=file_limit)
        with path.open("rb") as stream:
            return inspect_npy_array(
                stream,
                resource=resource,
                max_file_bytes=file_limit,
                max_array_bytes=array_limit,
                max_array_elements=element_limit,
            )

    original_position = source.tell()
    try:
        size = _seekable_size(source)
        if size > file_limit:
            raise ResourceLimitError(
                f"{resource} file",
                limit=file_limit,
                observed=size,
            )
        source.seek(0)
        import numpy as np

        version = np.lib.format.read_magic(source)
        if version == (1, 0):
            shape, fortran_order, dtype = np.lib.format.read_array_header_1_0(
                source,
                max_header_size=10_000,
            )
        elif version == (2, 0):
            shape, fortran_order, dtype = np.lib.format.read_array_header_2_0(
                source,
                max_header_size=10_000,
            )
        else:
            raise ValueError(f"{resource} uses unsupported NPY version {version}")
        dtype = np.dtype(dtype)
        if dtype.hasobject:
            raise ValueError(f"{resource} has object dtype")
        decoded_bytes = checked_shape_nbytes(
            shape,
            dtype,
            resource=resource,
            max_elements=element_limit,
            max_bytes=array_limit,
        )
        payload_bytes = size - source.tell()
        if payload_bytes != decoded_bytes:
            raise ValueError(
                f"{resource} payload size {payload_bytes} does not match declared "
                f"array bytes {decoded_bytes}"
            )
        return NumpyArrayDeclaration(
            name=resource,
            shape=tuple(int(value) for value in shape),
            dtype=dtype,
            fortran_order=bool(fortran_order),
            decoded_bytes=decoded_bytes,
        )
    finally:
        source.seek(original_position)


__all__ = [
    "ByteBudget",
    "GIB",
    "MAX_ARCHIVE_MEMBERS",
    "MAX_AUGMENTATION_ROWS",
    "MAX_CLEANUP_CANDIDATES",
    "MAX_CLEANUP_SCAN_ENTRIES",
    "MAX_ARRAY_BYTES",
    "MAX_ARRAY_ELEMENTS",
    "MAX_DECODED_AGGREGATE_BYTES",
    "MAX_DIRECTORY_ENTRIES",
    "MAX_EVENT_HDF5_BYTES",
    "MAX_EVENT_BATCH_ROWS",
    "MAX_EVENT_ROWS",
    "MAX_EXTERNAL_FILE_BYTES",
    "MAX_IN_MEMORY_FILE_BYTES",
    "MAX_JSON_AGGREGATE_NODES",
    "MAX_JSON_DEPTH",
    "MAX_JSON_NODES",
    "MAX_JSONL_ROW_NODES",
    "MAX_PUBLICATION_DIRECTORY_BYTES",
    "MAX_PUBLICATION_DIRECTORY_DEPTH",
    "MAX_PUBLICATION_DIRECTORY_ENTRIES",
    "MAX_PUBLICATION_FILE_BYTES",
    "MAX_EVT3_GAP_NS",
    "MAX_EVT3_BUFFER_BYTES",
    "MAX_EVT3_FEED_BYTES",
    "MAX_EVT3_FEED_WORDS",
    "MAX_EVT3_INPUT_BYTES",
    "MAX_IMAGE_PIXELS",
    "MAX_ILLUMINATION_ARCHIVE_BYTES",
    "MAX_ILLUMINATION_ARRAY_BYTES",
    "MAX_ILLUMINATION_DECODED_BYTES",
    "MAX_METADATA_AGGREGATE_BYTES",
    "MAX_METADATA_ATTRIBUTE_BYTES",
    "MAX_METADATA_ATTRIBUTES",
    "MAX_MESH_FACES",
    "MAX_MESH_VERTICES",
    "MAX_TARGET_MAP_BYTES",
    "NumpyArrayDeclaration",
    "ResourceLimitError",
    "checked_shape_nbytes",
    "inspect_npz_archive",
    "inspect_npy_array",
    "require_bounded_int",
    "require_dataset_file_size",
    "require_directory_budget",
    "require_file_size",
]
