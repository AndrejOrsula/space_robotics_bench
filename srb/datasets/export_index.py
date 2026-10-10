"""Bounded, disk-backed ordering for canonical SRB JSONL exports."""

from __future__ import annotations

import sqlite3
import tempfile
from collections import OrderedDict
from collections.abc import Iterator
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from hashlib import sha256
from numbers import Integral
from pathlib import Path
from typing import BinaryIO, Generator

from srb.datasets.jsonl import (
    MAX_JSONL_LINE_BYTES,
    SrbJsonlRow,
    _parse_srb_jsonl_line,
    iter_srb_jsonl_row_locations,
)
from srb.utils.resource_budget import (
    MAX_DECODED_AGGREGATE_BYTES,
    ResourceLimitError,
)

_SQLITE_PAGE_SIZE = 4096
_MAX_INDEX_BYTES = MAX_DECODED_AGGREGATE_BYTES
_MAX_INDEX_PAGES = _MAX_INDEX_BYTES // _SQLITE_PAGE_SIZE
_SQLITE_CACHE_KIB = 4096
_MAX_SOURCE_HANDLES = 16


class JsonlExportIndexError(ValueError):
    """The source changed after a JSONL export index was built."""


@dataclass(frozen=True, slots=True)
class JsonlExportEpisode:
    """One numeric episode identifier and its indexed sample count."""

    episode_index: int
    sample_count: int


class JsonlExportIndex:
    """Private SQLite index of JSONL row locations."""

    def __init__(
        self,
        connection: sqlite3.Connection,
        *,
        episode_count: int,
        total_samples: int,
    ) -> None:
        self._connection = connection
        self._episode_count = episode_count
        self._total_samples = total_samples
        self._source_handles: OrderedDict[Path, BinaryIO] = OrderedDict()
        self._closed = False

    @property
    def episode_count(self) -> int:
        """Return the number of distinct indexed episode identifiers."""

        self._ensure_open()
        return self._episode_count

    @property
    def total_samples(self) -> int:
        """Return the number of indexed rows."""

        self._ensure_open()
        return self._total_samples

    def __enter__(self) -> "JsonlExportIndex":
        self._ensure_open()
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()

    def close(self) -> None:
        """Close source handles and the private index deterministically."""

        if self._closed:
            return
        try:
            self._close_source_handles()
        finally:
            self._connection.close()
        self._closed = True

    def iter_episodes(self) -> Iterator[JsonlExportEpisode]:
        """Yield episodes in Python integer order without collecting them."""

        self._ensure_open()
        try:
            cursor = self._connection.execute(
                """
                SELECT episode_id, sample_count
                FROM episodes
                ORDER BY episode_id COLLATE PY_INTEGER
                """
            )
        except sqlite3.Error as exc:
            _raise_sqlite_full(exc)
            raise
        try:
            for episode_id, sample_count in cursor:
                yield JsonlExportEpisode(
                    episode_index=int(episode_id),
                    sample_count=int(sample_count),
                )
        finally:
            self._close_cursor(cursor)

    def iter_rows(
        self,
        episode_index: int | None = None,
        *,
        sort_steps: bool = False,
    ) -> Generator[SrbJsonlRow, None, None]:
        """Yield validated rows in numeric and stable source order."""

        self._ensure_open()
        parameters: tuple[str, ...] = ()
        if episode_index is None:
            if sort_steps:
                ordering = (
                    "episode_id COLLATE PY_INTEGER, "
                    "step_id COLLATE PY_INTEGER, source_ordinal"
                )
            else:
                ordering = "episode_id COLLATE PY_INTEGER, source_ordinal"
        else:
            episode_id = _integer_text(episode_index)
            parameters = (episode_id,)
            ordering = (
                "step_id COLLATE PY_INTEGER, source_ordinal"
                if sort_steps
                else "source_ordinal"
            )

        query = """
            SELECT source_path, byte_offset, byte_length, line_number,
                   episode_id, step_id, row_sha256
            FROM rows
        """
        if episode_index is not None:
            query += " WHERE episode_id = ?"
        query += f" ORDER BY {ordering}"
        try:
            cursor = self._connection.execute(query, parameters)
        except sqlite3.Error as exc:
            _raise_sqlite_full(exc)
            raise
        try:
            for record in cursor:
                yield self._read_indexed_row(record)
        finally:
            try:
                self._close_cursor(cursor)
            finally:
                self._close_source_handles()

    def _read_indexed_row(
        self,
        record: tuple[str, int, int, int, str, str, str],
    ) -> SrbJsonlRow:
        (
            source_path,
            byte_offset,
            byte_length,
            source_line,
            episode_id,
            step_id,
            row_sha256,
        ) = record
        path = Path(source_path)
        handle = self._source_handle(path)
        handle.seek(byte_offset)
        raw_line = handle.read(byte_length)
        if len(raw_line) != byte_length:
            raise JsonlExportIndexError(
                f"{path}:{source_line} changed after JSONL indexing"
            )
        if sha256(raw_line).hexdigest() != row_sha256:
            raise JsonlExportIndexError(
                f"{path}:{source_line} changed after JSONL indexing"
            )
        row = _parse_srb_jsonl_line(
            raw_line,
            episode_path=path,
            line_number=source_line,
        )
        if row is None:
            raise JsonlExportIndexError(
                f"{path}:{source_line} became blank after JSONL indexing"
            )
        if str(row["episode_index"]) != str(episode_id) or str(
            row["step_index"]
        ) != str(step_id):
            raise JsonlExportIndexError(
                f"{path}:{source_line} identifiers changed after JSONL indexing"
            )
        return row

    def _source_handle(self, path: Path) -> BinaryIO:
        handle = self._source_handles.pop(path, None)
        if handle is None:
            handle = path.open("rb")
        self._source_handles[path] = handle
        if len(self._source_handles) > _MAX_SOURCE_HANDLES:
            _evicted_path, evicted = next(iter(self._source_handles.items()))
            evicted.close()
            self._source_handles.pop(_evicted_path, None)
        return handle

    def _close_source_handles(self) -> None:
        handles = tuple(self._source_handles.items())
        with ExitStack() as cleanup:
            for path, handle in handles:
                cleanup.callback(self._close_source_handle, path, handle)

    def _close_source_handle(self, path: Path, handle: BinaryIO) -> None:
        handle.close()
        self._source_handles.pop(path, None)

    def _ensure_open(self) -> None:
        if self._closed:
            raise JsonlExportIndexError("JSONL export index is closed")

    def _close_cursor(self, cursor: sqlite3.Cursor) -> None:
        if not self._closed:
            cursor.close()


@contextmanager
def open_jsonl_export_index(dataset_path: Path) -> Iterator[JsonlExportIndex]:
    """Build and own a temporary bounded index for JSONL export."""

    with ExitStack() as cleanup:
        temporary_directory = cleanup.enter_context(
            tempfile.TemporaryDirectory(prefix="srb-jsonl-index-")
        )
        index = _build_index(Path(dataset_path), Path(temporary_directory))
        try:
            yield index
        finally:
            index.close()


def _build_index(dataset_path: Path, temporary_directory: Path) -> JsonlExportIndex:
    database_path = temporary_directory / "index.sqlite3"
    with ExitStack() as cleanup:
        connection = sqlite3.connect(database_path)
        cleanup.callback(connection.close)
        try:
            _configure_connection(connection)
            _create_schema(connection)
            total_samples = _populate_index(connection, dataset_path)
            if total_samples == 0:
                raise ValueError("canonical dataset contains no steps")
            episode_count = _count_episodes(connection)
        except sqlite3.Error as exc:
            _raise_sqlite_full(exc)
            raise
        index = JsonlExportIndex(
            connection,
            episode_count=episode_count,
            total_samples=total_samples,
        )
        cleanup.pop_all()
        return index


def _configure_connection(connection: sqlite3.Connection) -> None:
    connection.create_collation("PY_INTEGER", _compare_integer_text)
    for pragma in (
        f"PRAGMA page_size={_SQLITE_PAGE_SIZE}",
        f"PRAGMA max_page_count={_MAX_INDEX_PAGES}",
        f"PRAGMA cache_size=-{_SQLITE_CACHE_KIB}",
        "PRAGMA mmap_size=0",
        "PRAGMA temp_store=FILE",
        "PRAGMA journal_mode=OFF",
        "PRAGMA synchronous=OFF",
    ):
        connection.execute(pragma).close()


def _create_schema(connection: sqlite3.Connection) -> None:
    connection.executescript(
        """
        CREATE TABLE episodes (
            episode_id TEXT COLLATE PY_INTEGER PRIMARY KEY,
            sample_count INTEGER NOT NULL
        );
        CREATE TABLE rows (
            source_ordinal INTEGER NOT NULL,
            source_path TEXT NOT NULL,
            byte_offset INTEGER NOT NULL,
            byte_length INTEGER NOT NULL,
            line_number INTEGER NOT NULL,
            episode_id TEXT COLLATE PY_INTEGER NOT NULL,
            step_id TEXT COLLATE PY_INTEGER NOT NULL,
            row_sha256 TEXT NOT NULL
        );
        CREATE INDEX rows_global_order
            ON rows (episode_id COLLATE PY_INTEGER, source_ordinal);
        CREATE INDEX rows_episode_step_order
            ON rows (
                episode_id COLLATE PY_INTEGER,
                step_id COLLATE PY_INTEGER,
                source_ordinal
            );
        """
    )


def _populate_index(connection: sqlite3.Connection, dataset_path: Path) -> int:
    total_samples = 0
    connection.execute("BEGIN IMMEDIATE")
    row_cursor = connection.cursor()
    episode_cursor = connection.cursor()
    source_handles: OrderedDict[Path, BinaryIO] = OrderedDict()
    try:
        for source_ordinal, location in enumerate(
            iter_srb_jsonl_row_locations(
                dataset_path,
                max_line_bytes=MAX_JSONL_LINE_BYTES,
            )
        ):
            episode_id = str(location.row["episode_index"])
            step_id = str(location.row["step_index"])
            source = source_handles.pop(location.path, None)
            if source is None:
                source = location.path.open("rb")
            source_handles[location.path] = source
            if len(source_handles) > _MAX_SOURCE_HANDLES:
                _evicted_path, evicted = source_handles.popitem(last=False)
                evicted.close()
            source.seek(location.offset)
            raw_line = source.read(location.length)
            if len(raw_line) != location.length:
                raise JsonlExportIndexError(
                    f"{location.path}:{location.line_number} changed during indexing"
                )
            current_row = _parse_srb_jsonl_line(
                raw_line,
                episode_path=location.path,
                line_number=location.line_number,
            )
            if current_row != location.row:
                raise JsonlExportIndexError(
                    f"{location.path}:{location.line_number} changed during indexing"
                )
            row_cursor.execute(
                """
                INSERT INTO rows (
                    source_ordinal, source_path, byte_offset, byte_length,
                    line_number, episode_id, step_id, row_sha256
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    source_ordinal,
                    str(location.path.absolute()),
                    location.offset,
                    location.length,
                    location.line_number,
                    episode_id,
                    step_id,
                    sha256(raw_line).hexdigest(),
                ),
            )
            episode_cursor.execute(
                """
                INSERT INTO episodes (episode_id, sample_count)
                VALUES (?, 1)
                ON CONFLICT(episode_id) DO UPDATE SET
                    sample_count = episodes.sample_count + 1
                """,
                (episode_id,),
            )
            total_samples += 1
        connection.commit()
    finally:
        try:
            for source in source_handles.values():
                source.close()
        finally:
            try:
                row_cursor.close()
            finally:
                episode_cursor.close()
    return total_samples


def _count_episodes(connection: sqlite3.Connection) -> int:
    cursor = connection.execute("SELECT COUNT(*) FROM episodes")
    try:
        result = cursor.fetchone()
    finally:
        cursor.close()
    if result is None:
        raise RuntimeError("SQLite episode count query returned no row")
    return int(result[0])


def _integer_text(value: object) -> str:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise TypeError("episode_index must be an integer or None")
    return str(int(value))


def _integer_parts(value: str | bytes) -> tuple[bool, str]:
    if isinstance(value, bytes):
        value = value.decode("ascii")
    negative = value.startswith("-")
    digits = value[1:] if negative else value
    digits = digits.lstrip("0") or "0"
    return negative and digits != "0", digits


def _compare_integer_text(left: str, right: str) -> int:
    left_negative, left_digits = _integer_parts(left)
    right_negative, right_digits = _integer_parts(right)
    if left_negative != right_negative:
        return -1 if left_negative else 1
    if len(left_digits) != len(right_digits):
        result = -1 if len(left_digits) < len(right_digits) else 1
    else:
        result = (left_digits > right_digits) - (left_digits < right_digits)
    return -result if left_negative else result


def _is_sqlite_full(error: sqlite3.Error) -> bool:
    return (
        getattr(error, "sqlite_errorcode", None) == sqlite3.SQLITE_FULL
        or "database or disk is full" in str(error).lower()
    )


def _raise_sqlite_full(error: sqlite3.Error) -> None:
    if _is_sqlite_full(error):
        raise ResourceLimitError(
            "JSONL export index",
            limit=_MAX_INDEX_BYTES,
            observed=_MAX_INDEX_BYTES + 1,
        ) from error


__all__ = [
    "JsonlExportEpisode",
    "JsonlExportIndex",
    "JsonlExportIndexError",
    "open_jsonl_export_index",
]
