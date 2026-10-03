"""Atomic publication of files, directories and generations.

Writers build a private stage beside the destination, verify it and rename it
into place while holding a sibling lock file. Readers take no lock: renames are
atomic and published payloads are never modified in place.

On-disk layout (also read by the Rust GUI and the shell setup scripts):

- file ``<dir>/<name>``: lock ``.<name>.publication.lock``; optional identity
  marker ``.<name>.publication.sha256`` for immutable files.
- directory ``<dir>/<name>/``: embedded ``.srb-directory-publication.json``.
- generation root ``<root>/``: pointer ``current.json`` selecting
  ``generations/<id>/`` with its ``.srb-publication.json`` manifest.
"""

from __future__ import annotations

import errno
import glob
import hashlib
import json
import os
import shutil
import stat as stat_module
import tempfile
import uuid
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType, TracebackType
from typing import BinaryIO, ContextManager, Protocol, TypeVar

from filelock import FileLock
from filelock import Timeout as FileLockTimeout

from srb.utils.failures import attempt_cleanup
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES,
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_PUBLICATION_DIRECTORY_BYTES,
    MAX_PUBLICATION_DIRECTORY_DEPTH,
    MAX_PUBLICATION_DIRECTORY_ENTRIES,
    MAX_PUBLICATION_FILE_BYTES,
    ByteBudget,
    ResourceLimitError,
)

T = TypeVar("T")

_GENERATION_SCHEMA = "srb-publication/1"
_GENERATION_MANIFEST = ".srb-publication.json"
_GENERATION_PENDING = ".srb-publication.pending"
_GENERATION_ID_LENGTH = 32
_DIRECTORY_SCHEMA = "srb-directory-publication/1"
_DIRECTORY_INTERNAL_MANIFEST = ".srb-directory-publication.json"
_CURRENT_POINTER = "current.json"
_STAGE_INFIX = ".publication-"
_STAGE_SUFFIX = ".stage"
_MAX_METADATA_BYTES = 64 << 20
_CHUNK = 1 << 20


class PublicationError(RuntimeError):
    """Base error for publication failures."""


class _CleanupRetryOwner(Protocol):
    def retry_cleanup(self) -> None:
        """Retry the cleanup represented by this owner."""


class PublicationCleanupError(PublicationError):
    """A publication cleanup owner remains available for a bounded retry."""

    def __init__(
        self,
        message: str,
        *,
        owner: _CleanupRetryOwner,
        committed_paths: Sequence[Path] = (),
        committed_result: object | None = None,
        errors: Sequence[BaseException] = (),
    ) -> None:
        self.owner = owner
        self.cleanup_owner = owner
        self.committed_paths = tuple(Path(path) for path in committed_paths)
        self.committed_result = committed_result
        self.errors = tuple(errors)
        detail = "; ".join(f"{type(error).__name__}: {error}" for error in self.errors)
        super().__init__(f"{message}: {detail}" if detail else message)

    def retry_cleanup(self) -> None:
        """Retry cleanup through the owner retaining its original identities."""

        _result, retry_error = attempt_cleanup(self.owner.retry_cleanup)
        if retry_error is None:
            return
        if retry_error is self:
            raise retry_error
        committed_paths = list(self.committed_paths)
        if isinstance(retry_error, PublicationCleanupError):
            for path in retry_error.committed_paths:
                if path not in committed_paths:
                    committed_paths.append(path)
        raise type(self)(
            "publication cleanup retry remains pending",
            owner=self.owner,
            committed_paths=tuple(committed_paths),
            committed_result=self.committed_result,
            errors=(*self.errors, retry_error),
        ) from retry_error


class PublicationCommitCleanupError(PublicationCleanupError):
    """A committed destination remains while private cleanup is pending."""


class PublicationIntegrityError(PublicationError):
    """Bytes do not match their declared identity."""

    def __init__(self, *, path: Path, expected_sha256: str, actual_sha256: str) -> None:
        self.path = path
        self.expected_sha256 = expected_sha256
        self.actual_sha256 = actual_sha256
        super().__init__(
            f"SHA256 mismatch for {path}: expected {expected_sha256}, "
            f"got {actual_sha256}"
        )


class PublicationIdentityError(PublicationIntegrityError):
    """One immutable destination was requested with a different identity."""


class PublicationTimeoutError(PublicationError):
    """A destination lock could not be acquired before its deadline."""


class PublicationNotFoundError(PublicationError):
    """No current publication exists at the requested destination."""


class PublicationReuseError(PublicationError):
    """An existing publication failed an explicit reuse check."""


class PublicationResourceLimitError(PublicationError):
    """A publication exceeds its finite size ceiling."""

    def __init__(self, path: Path, *, limit: int, observed: int) -> None:
        self.path = path
        self.limit = limit
        self.observed = observed
        super().__init__(
            f"publication file exceeds resource limit: {path}: "
            f"observed {observed} bytes, limit {limit} bytes"
        )


def default_new_file_mode() -> int:
    """Return the umask-respecting mode a freshly created file should get.

    Stages are created ``0o600``; promotion applies this mode to a stage whose
    mode the writer left unchanged. The umask is read from procfs so it is
    never mutated; without procfs ``0o600`` is returned.
    """

    try:
        with open("/proc/self/status", "rb") as status:
            for line in status:
                if line.startswith(b"Umask:"):
                    return 0o666 & ~(int(line.split()[1], 8) & 0o777)
    except (OSError, ValueError, IndexError):
        pass
    return 0o600


def _canonical(path: Path | str) -> Path:
    """Absolute path with resolved parents and the final leaf kept as given."""

    absolute = Path(os.path.abspath(os.fspath(Path(path).expanduser())))
    return absolute.parent.resolve() / absolute.name


def _limit(value: int | None, default: int) -> int:
    return default if value is None else int(value)


def _canonical_json(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _parse_json_object(payload: bytes) -> dict[str, object]:
    try:
        parsed = json.loads(payload)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise PublicationError(
            "publication metadata must be a valid JSON object"
        ) from exc
    if not isinstance(parsed, dict):
        raise PublicationError("publication metadata must be a valid JSON object")
    return parsed


def _is_sha256(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(c in "0123456789abcdef" for c in value)
    )


def _is_generation_id(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == _GENERATION_ID_LENGTH
        and all(c in "0123456789abcdef" for c in value)
    )


def _normalize_directory_member(member: str) -> str:
    path = Path(member)
    if path.is_absolute() or ".." in path.parts or not path.parts:
        raise ValueError(f"invalid publication member path: {member!r}")
    return path.as_posix()


def _normalize_member(member: str) -> str:
    normalized = _normalize_directory_member(member)
    if normalized in {_GENERATION_MANIFEST, _GENERATION_PENDING}:
        raise ValueError(f"reserved generation member path: {member!r}")
    return normalized


def _normalize_required_members(members: Sequence[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(_normalize_member(member) for member in members))


def _open_regular_or_none(path: Path) -> BinaryIO | None:
    """Open a regular file without following a final symlink; None if missing."""

    # O_NONBLOCK keeps a FIFO at the path from blocking the open.
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        descriptor = os.open(path, flags)
    except (FileNotFoundError, NotADirectoryError):
        return None
    except OSError as exc:
        if exc.errno == errno.ELOOP:
            raise PublicationError(f"publication path is a symlink: {path}") from exc
        raise
    if not stat_module.S_ISREG(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise PublicationError(f"publication path is not a regular file: {path}")
    return os.fdopen(descriptor, "rb")


def _open_regular(path: Path) -> BinaryIO:
    handle = _open_regular_or_none(path)
    if handle is None:
        raise PublicationError(f"publication file is missing: {path}")
    return handle


def _charge(budget: ByteBudget | None, amount: int, *, path: Path) -> None:
    if budget is None:
        return
    try:
        budget.consume(amount)
    except ResourceLimitError as exc:
        raise PublicationResourceLimitError(
            path, limit=budget.limit, observed=exc.observed
        ) from exc


def _copy_hashed(
    source: BinaryIO,
    sink: BinaryIO | None,
    *,
    path: Path,
    max_bytes: int,
    byte_budget: ByteBudget | None = None,
    chunk_size: int = _CHUNK,
) -> tuple[str, int]:
    """Stream ``source`` (optionally into ``sink``) and return SHA256 and size."""

    digest = hashlib.sha256()
    size = 0
    while chunk := source.read(chunk_size):
        size += len(chunk)
        if size > max_bytes:
            raise PublicationResourceLimitError(path, limit=max_bytes, observed=size)
        _charge(byte_budget, len(chunk), path=path)
        digest.update(chunk)
        if sink is not None:
            sink.write(chunk)
    return digest.hexdigest(), size


def _fsync(path: Path) -> None:
    flags = (
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) if path.is_dir() else os.O_RDONLY
    )
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _fsync_tree(root: Path) -> None:
    for directory, _dirnames, filenames in os.walk(root, onerror=_raise_walk_error):
        for name in filenames:
            _fsync(Path(directory, name))
        _fsync(Path(directory))


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    """Replace one file with ``payload`` (caller holds the relevant lock)."""

    descriptor, name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, default_new_file_mode())
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    _fsync(path.parent)


def _remove_path(path: Path) -> None:
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path)
    else:
        path.unlink(missing_ok=True)


def _raise_walk_error(error: OSError) -> None:
    raise PublicationError(
        f"could not enumerate publication path: {error.filename}"
    ) from error


def _remove_stale_stages(destination: Path) -> None:
    """Remove stages left by a crashed writer (the caller holds the lock)."""

    pattern = f".{glob.escape(destination.name)}{_STAGE_INFIX}*{_STAGE_SUFFIX}"
    for stale in destination.parent.glob(pattern):
        _remove_path(stale)


def _lock_file(path: Path) -> Path:
    return path.with_name(f".{path.name}.publication.lock")


@contextmanager
def _locked(paths: Iterable[Path | None], *, timeout_s: float) -> Iterator[None]:
    """Hold every lock file in deterministic order. Locks do not re-enter."""

    unique = sorted({Path(path) for path in paths if path is not None}, key=os.fspath)
    with ExitStack() as stack:
        for path in unique:
            path.parent.mkdir(parents=True, exist_ok=True)
            try:
                stack.enter_context(FileLock(path, timeout=timeout_s))
            except FileLockTimeout as exc:
                raise PublicationTimeoutError(
                    f"timed out acquiring publication lock {path} after {timeout_s:g}s"
                ) from exc
        yield


def sha256_file(
    path: Path,
    *,
    chunk_size: int = _CHUNK,
    byte_budget: ByteBudget | None = None,
    max_bytes: int = MAX_PUBLICATION_FILE_BYTES,
) -> str:
    """Hash a regular file, optionally charging a shared streamed-byte budget."""

    with _open_regular(Path(path)) as handle:
        digest, _size = _copy_hashed(
            handle,
            None,
            path=Path(path),
            max_bytes=max_bytes,
            byte_budget=byte_budget,
            chunk_size=chunk_size,
        )
    return digest


def read_regular_file_bytes(
    path: Path, *, max_bytes: int = MAX_IN_MEMORY_FILE_BYTES
) -> bytes | None:
    """Read a regular file within ``max_bytes``; a missing path returns None."""

    handle = _open_regular_or_none(Path(path))
    if handle is None:
        return None
    with handle:
        payload = handle.read(max_bytes + 1)
    if len(payload) > max_bytes:
        raise PublicationResourceLimitError(
            Path(path), limit=max_bytes, observed=len(payload)
        )
    return payload


@contextmanager
def open_directory_descriptor(path: Path, *, create: bool = False) -> Iterator[int]:
    """Yield a directory descriptor for descriptor-relative operations."""

    path = Path(path)
    if create:
        path.mkdir(parents=True, exist_ok=True)
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        yield descriptor
    finally:
        os.close(descriptor)


def _stat_key(value: os.stat_result) -> tuple[int, int, int, int]:
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns)


def _copy_regular_file_to_snapshot(
    source: Path,
    destination: Path,
    *,
    expected_sha256: str | None,
    max_bytes: int | None = MAX_PUBLICATION_FILE_BYTES,
    byte_budget: ByteBudget | None = None,
    expected_source_stat: os.stat_result | None = None,
) -> None:
    """Copy one regular file to a new destination, verifying its hash."""

    source = Path(source)
    destination = Path(destination)
    with _open_regular(source) as reader:

        def source_changed() -> bool:
            return expected_source_stat is not None and _stat_key(
                os.fstat(reader.fileno())
            ) != _stat_key(expected_source_stat)

        if source_changed():
            raise PublicationError(f"source file changed before copy: {source}")
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
        with os.fdopen(os.open(destination, flags, 0o600), "wb") as writer:
            try:
                digest, _size = _copy_hashed(
                    reader,
                    writer,
                    path=source,
                    max_bytes=_limit(max_bytes, MAX_PUBLICATION_FILE_BYTES),
                    byte_budget=byte_budget,
                )
                # An in-place write during the copy would leave a mixed snapshot.
                if source_changed():
                    raise PublicationError(f"source file changed during copy: {source}")
            except BaseException:
                destination.unlink(missing_ok=True)
                raise
    if expected_sha256 is not None and digest != expected_sha256:
        destination.unlink(missing_ok=True)
        raise PublicationIntegrityError(
            path=source, expected_sha256=expected_sha256, actual_sha256=digest
        )


def copy_verified_file_snapshot(
    source: Path,
    destination: Path,
    *,
    expected_source_stat: os.stat_result,
) -> None:
    """Copy one file after verifying the caller's expected source ``lstat``."""

    _copy_regular_file_to_snapshot(
        source,
        destination,
        expected_sha256=None,
        expected_source_stat=expected_source_stat,
    )


def _walk_members(
    root: Path,
    *,
    max_entries: int,
    max_depth: int,
    excluded: frozenset[str] = frozenset(),
) -> Iterator[tuple[str, bool]]:
    """Yield ``(relative, is_directory)`` for a tree without symlinks."""

    count = 0
    for directory, dirnames, filenames in os.walk(root, onerror=_raise_walk_error):
        dirnames.sort()
        base = Path(directory)
        for name in (*dirnames, *sorted(filenames)):
            path = base / name
            relative = path.relative_to(root).as_posix()
            if relative in excluded:
                continue
            count += 1
            if count > max_entries:
                raise PublicationResourceLimitError(
                    root, limit=max_entries, observed=count
                )
            if len(Path(relative).parts) > max_depth + 1:
                raise PublicationError(
                    f"publication member is nested too deeply: {path}"
                )
            mode = os.lstat(path).st_mode
            if stat_module.S_ISLNK(mode):
                raise PublicationError(f"publication contains a symlink: {path}")
            if stat_module.S_ISDIR(mode):
                yield relative, True
            elif stat_module.S_ISREG(mode):
                yield relative, False
            else:
                raise PublicationError(f"publication contains a special file: {path}")


def _inventory(
    root: Path,
    *,
    excluded: frozenset[str],
    max_bytes: int | None = MAX_PUBLICATION_DIRECTORY_BYTES,
    max_entries: int = MAX_PUBLICATION_DIRECTORY_ENTRIES,
    max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
    byte_budget: ByteBudget | None = None,
) -> tuple[list[str], dict[str, dict[str, object]]]:
    """Return the directories and ``{member: {sha256, size_bytes}}`` of a tree."""

    total = _limit(max_bytes, MAX_PUBLICATION_DIRECTORY_BYTES)
    budget = byte_budget or ByteBudget("publication directory", limit=total)
    directories: list[str] = []
    members: dict[str, dict[str, object]] = {}
    for relative, is_directory in _walk_members(
        root, max_entries=max_entries, max_depth=max_depth, excluded=excluded
    ):
        if is_directory:
            directories.append(relative)
            continue
        path = root / relative
        with _open_regular(path) as handle:
            digest, size = _copy_hashed(
                handle, None, path=path, max_bytes=total, byte_budget=budget
            )
        members[relative] = {"sha256": digest, "size_bytes": size}
    return directories, members


def directory_member_hashes(
    root: Path,
    *,
    max_bytes: int | None = MAX_PUBLICATION_DIRECTORY_BYTES,
    max_entries: int = MAX_PUBLICATION_DIRECTORY_ENTRIES,
    max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
    byte_budget: ByteBudget | None = None,
) -> dict[str, str]:
    """Hash every regular file of a caller-owned tree.

    The reserved directory-publication manifest is excluded.
    """

    _directories, members = _inventory(
        Path(root),
        excluded=frozenset({_DIRECTORY_INTERNAL_MANIFEST}),
        max_bytes=max_bytes,
        max_entries=max_entries,
        max_depth=max_depth,
        byte_budget=byte_budget,
    )
    return {name: str(entry["sha256"]) for name, entry in members.items()}


def _read_member(
    root: Path,
    member_hashes: Mapping[str, str],
    member: str,
    parser: Callable[[bytes], T],
    *,
    max_bytes: int | None,
) -> T:
    normalized = _normalize_directory_member(member)
    expected = member_hashes.get(normalized)
    if expected is None:
        raise PublicationError(f"published member is not manifested: {member}")
    payload = read_regular_file_bytes(
        root / normalized, max_bytes=_limit(max_bytes, MAX_IN_MEMORY_FILE_BYTES)
    )
    if payload is None:
        raise PublicationError(f"published member is missing: {root / normalized}")
    actual = hashlib.sha256(payload).hexdigest()
    if actual != expected:
        raise PublicationIntegrityError(
            path=root / normalized, expected_sha256=expected, actual_sha256=actual
        )
    return parser(payload)


@dataclass
class OwnedTemporaryDirectory:
    """A temporary directory removed only while it is still the same inode."""

    path: Path
    _identity: tuple[int, int] = field(repr=False)
    _cleaned: bool = field(default=False, init=False, repr=False)

    @classmethod
    def create(cls, *, prefix: str) -> "OwnedTemporaryDirectory":
        if os.sep in prefix:
            raise ValueError("temporary directory prefix must be one path component")
        parent = Path(tempfile.gettempdir()).resolve()
        return cls.bind(Path(tempfile.mkdtemp(prefix=prefix, dir=parent)))

    @classmethod
    def bind(cls, path: Path) -> "OwnedTemporaryDirectory":
        path = Path(os.path.abspath(os.fspath(path)))
        current = os.lstat(path)
        if not stat_module.S_ISDIR(current.st_mode):
            raise PublicationError(f"temporary directory is not a directory: {path}")
        return cls(path=path, _identity=(current.st_dev, current.st_ino))

    def cleanup(self) -> None:
        if self._cleaned:
            return
        try:
            current = os.lstat(self.path)
        except FileNotFoundError:
            self._cleaned = True
            return
        if (current.st_dev, current.st_ino) != self._identity:
            raise PublicationError(f"temporary directory was replaced: {self.path}")
        shutil.rmtree(self.path)
        self._cleaned = True

    def retry_cleanup(self) -> None:
        self.cleanup()


@dataclass
class MaterializedDirectorySnapshot:
    """Private on-disk copy of an exact verified-member set."""

    path: Path
    member_hashes: Mapping[str, str]
    _temporary_directory: OwnedTemporaryDirectory = field(repr=False)
    _closed: bool = field(default=False, init=False, repr=False)
    _finalizer_cleanup_deferred: bool = field(default=False, init=False, repr=False)

    def read_member(
        self,
        member: str,
        parser: Callable[[bytes], T],
        *,
        max_bytes: int | None = MAX_IN_MEMORY_FILE_BYTES,
    ) -> T:
        if self._closed:
            raise PublicationError("materialized publication snapshot is closed")
        return _read_member(
            self.path, self.member_hashes, member, parser, max_bytes=max_bytes
        )

    def read_member_bytes(
        self, member: str, *, max_bytes: int | None = MAX_IN_MEMORY_FILE_BYTES
    ) -> bytes:
        return self.read_member(member, lambda payload: payload, max_bytes=max_bytes)

    def __enter__(self) -> "MaterializedDirectorySnapshot":
        if self._closed:
            raise PublicationError("materialized publication snapshot is closed")
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self.close()

    def defer_finalizer_cleanup(self) -> None:
        """Leave deletion to an explicit owner instead of the finalizer."""

        self._finalizer_cleanup_deferred = True

    def close(self) -> None:
        if not self._closed:
            self._temporary_directory.cleanup()
            self._closed = True
        self._finalizer_cleanup_deferred = False

    def retry_cleanup(self) -> None:
        self.close()

    def __del__(self) -> None:  # pragma: no cover - interpreter cleanup
        try:
            if not self._finalizer_cleanup_deferred:
                self.close()
        except Exception:
            pass


def materialize_file_snapshot(
    files: Mapping[str, tuple[Path, str]],
    *,
    max_bytes: int | None = MAX_PUBLICATION_DIRECTORY_BYTES,
    max_entries: int = MAX_PUBLICATION_DIRECTORY_ENTRIES,
    max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
    byte_budget: ByteBudget | None = None,
) -> MaterializedDirectorySnapshot:
    """Copy named files (``{destination: (source, sha256)}``) into a private snapshot."""

    if len(files) > max_entries:
        raise PublicationResourceLimitError(
            Path(tempfile.gettempdir()), limit=max_entries, observed=len(files)
        )
    budget = byte_budget or ByteBudget(
        "publication snapshot", limit=_limit(max_bytes, MAX_PUBLICATION_DIRECTORY_BYTES)
    )
    owner = OwnedTemporaryDirectory.create(prefix="srb-publication-")
    hashes: dict[str, str] = {}
    try:
        for name, (source, expected_sha256) in files.items():
            relative = _normalize_directory_member(name)
            if relative in hashes:
                raise PublicationError(f"duplicate snapshot member: {name}")
            if len(Path(relative).parts) > max_depth + 1:
                raise PublicationError(f"snapshot member is nested too deeply: {name}")
            destination = owner.path / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            _copy_regular_file_to_snapshot(
                Path(source),
                destination,
                expected_sha256=expected_sha256,
                byte_budget=budget,
            )
            hashes[relative] = expected_sha256
    except BaseException:
        owner.cleanup()
        raise
    return MaterializedDirectorySnapshot(owner.path, MappingProxyType(hashes), owner)


def _materialize_members(
    root: Path,
    member_hashes: Mapping[str, str],
    members: Sequence[str] | None,
    **limits: object,
) -> MaterializedDirectorySnapshot:
    selected = tuple(member_hashes) if members is None else tuple(members)
    files: dict[str, tuple[Path, str]] = {}
    for member in selected:
        normalized = _normalize_directory_member(member)
        if normalized not in member_hashes:
            raise PublicationError(f"published member is not manifested: {member}")
        files[normalized] = (root / normalized, member_hashes[normalized])
    return materialize_file_snapshot(files, **limits)  # type: ignore[arg-type]


@dataclass(frozen=True)
class PublishedGeneration:
    """One immutable generation selected by the ``current.json`` pointer."""

    generation_id: str
    path: Path
    member_hashes: Mapping[str, str]

    def read_member(
        self,
        member: str,
        parser: Callable[[bytes], T],
        *,
        max_bytes: int | None = MAX_IN_MEMORY_FILE_BYTES,
    ) -> T:
        return _read_member(
            self.path, self.member_hashes, member, parser, max_bytes=max_bytes
        )

    def read_member_bytes(
        self, member: str, *, max_bytes: int | None = MAX_IN_MEMORY_FILE_BYTES
    ) -> bytes:
        return self.read_member(member, lambda payload: payload, max_bytes=max_bytes)

    def copy_member_to(self, member: str, destination: Path) -> None:
        """Copy one verified member to a new caller-owned path."""

        normalized = _normalize_member(member)
        expected = self.member_hashes.get(normalized)
        if expected is None:
            raise PublicationError(f"published member is not manifested: {member}")
        _copy_regular_file_to_snapshot(
            self.path / normalized, Path(destination), expected_sha256=expected
        )

    def materialize_snapshot(
        self,
        *,
        members: Sequence[str] | None = None,
        max_bytes: int | None = MAX_PUBLICATION_DIRECTORY_BYTES,
        max_entries: int = MAX_PUBLICATION_DIRECTORY_ENTRIES,
        max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
    ) -> MaterializedDirectorySnapshot:
        return _materialize_members(
            self.path,
            self.member_hashes,
            members,
            max_bytes=max_bytes,
            max_entries=max_entries,
            max_depth=max_depth,
        )

    def __enter__(self) -> "PublishedGeneration":
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self.close()

    def close(self) -> None:
        """Readers hold no lease; kept for context-manager callers."""


@dataclass(frozen=True)
class PublishedDirectory:
    """One immutable directory with its embedded manifest."""

    path: Path
    member_hashes: Mapping[str, str]

    def read_member(
        self,
        member: str,
        parser: Callable[[bytes], T],
        *,
        max_bytes: int | None = MAX_IN_MEMORY_FILE_BYTES,
    ) -> T:
        return _read_member(
            self.path, self.member_hashes, member, parser, max_bytes=max_bytes
        )

    def read_member_bytes(
        self, member: str, *, max_bytes: int | None = MAX_IN_MEMORY_FILE_BYTES
    ) -> bytes:
        return self.read_member(member, lambda payload: payload, max_bytes=max_bytes)

    def materialize_snapshot(
        self,
        *,
        members: Sequence[str] | None = None,
        max_bytes: int | None = MAX_PUBLICATION_DIRECTORY_BYTES,
        max_entries: int = MAX_PUBLICATION_DIRECTORY_ENTRIES,
        max_depth: int = MAX_PUBLICATION_DIRECTORY_DEPTH,
    ) -> MaterializedDirectorySnapshot:
        return _materialize_members(
            self.path,
            self.member_hashes,
            members,
            max_bytes=max_bytes,
            max_entries=max_entries,
            max_depth=max_depth,
        )


@dataclass
class PublicationSnapshot:
    """One open, hash-verified file; replacing the path does not change it."""

    path: Path
    sha256: str
    size_bytes: int
    file: BinaryIO

    def __enter__(self) -> "PublicationSnapshot":
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self.close()

    def close(self) -> None:
        self.file.close()


@dataclass(frozen=True)
class FilePublication:
    """Publish one complete file by renaming a verified sibling stage."""

    path: Path
    lock_timeout_s: float = 30.0
    lock_path: Path | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "path", _canonical(self.path))
        if self.lock_path is not None:
            object.__setattr__(self, "lock_path", _canonical(self.lock_path))
            if self.lock_path == self.path:
                raise ValueError("publication lock_path must not be the destination")

    @property
    def _identity_path(self) -> Path:
        return self.path.with_name(f".{self.path.name}.publication.sha256")

    def _lock_paths(self) -> tuple[Path | None, ...]:
        return (_lock_file(self.path), self.lock_path)

    def _locked(self) -> ContextManager[None]:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        return _locked(self._lock_paths(), timeout_s=self.lock_timeout_s)

    def _new_stage(self) -> Path:
        """Create an empty ``0o600`` stage (the caller holds the lock)."""

        _remove_stale_stages(self.path)
        descriptor, name = tempfile.mkstemp(
            prefix=f".{self.path.name}{_STAGE_INFIX}",
            suffix=_STAGE_SUFFIX,
            dir=self.path.parent,
        )
        os.close(descriptor)
        return Path(name)

    def _read_identity(self) -> str | None:
        payload = read_regular_file_bytes(self._identity_path, max_bytes=128)
        if payload is None:
            return None
        identity = payload.decode("ascii", errors="replace").strip()
        if not _is_sha256(identity):
            raise PublicationError(
                f"invalid publication identity marker: {self._identity_path}"
            )
        return identity

    def _current_sha256(self, max_bytes: int) -> str | None:
        handle = _open_regular_or_none(self.path)
        if handle is None:
            return None
        with handle:
            return _copy_hashed(handle, None, path=self.path, max_bytes=max_bytes)[0]

    def _check_immutable(self, expected_sha256: str, max_bytes: int) -> None:
        """Reject a request whose identity differs from the established one."""

        established = self._read_identity()
        current = self._current_sha256(max_bytes)
        if established is not None and current is None:
            # The payload was deleted; the stale marker must not pin it forever.
            self._identity_path.unlink(missing_ok=True)
            return
        if established is not None and current != established:
            raise PublicationIntegrityError(
                path=self.path, expected_sha256=established, actual_sha256=str(current)
            )
        established = established or current
        if established is not None and established != expected_sha256:
            raise PublicationIdentityError(
                path=self.path,
                expected_sha256=expected_sha256,
                actual_sha256=established,
            )
        if current is not None and not self._identity_path.exists():
            _atomic_write_bytes(self._identity_path, f"{current}\n".encode("ascii"))

    def _promote_locked(
        self,
        stage: Path,
        *,
        expected_sha256: str | None,
        immutable_identity: bool,
        max_bytes: int | None,
    ) -> Path:
        limit = _limit(max_bytes, MAX_PUBLICATION_FILE_BYTES)
        with _open_regular(stage) as handle:
            actual, _size = _copy_hashed(handle, None, path=stage, max_bytes=limit)
        if expected_sha256 is not None and actual != expected_sha256:
            raise PublicationIntegrityError(
                path=self.path, expected_sha256=expected_sha256, actual_sha256=actual
            )
        established = self._read_identity()
        if established is None and immutable_identity:
            established = self._current_sha256(limit)
        if established is not None and established != actual:
            raise PublicationIdentityError(
                path=self.path, expected_sha256=actual, actual_sha256=established
            )
        if stat_module.S_IMODE(os.lstat(stage).st_mode) == 0o600:
            os.chmod(stage, default_new_file_mode())
        _fsync(stage)
        os.replace(stage, self.path)
        finish = _FileCommitFinisher(self, actual if immutable_identity else None)
        try:
            finish.retry_cleanup()
        except Exception as error:
            raise PublicationCommitCleanupError(
                "publication committed but finishing it failed",
                owner=finish,
                committed_paths=(self.path,),
                committed_result=self.path,
                errors=(error,),
            ) from error
        return self.path

    def _publish_from_stage(
        self,
        fill: Callable[[Path], object],
        *,
        expected_sha256: str | None = None,
        immutable_identity: bool = False,
        max_bytes: int | None = MAX_PUBLICATION_FILE_BYTES,
    ) -> Path:
        stage = self._new_stage()
        try:
            fill(stage)
            return self._promote_locked(
                stage,
                expected_sha256=expected_sha256,
                immutable_identity=immutable_identity,
                max_bytes=max_bytes,
            )
        finally:
            stage.unlink(missing_ok=True)

    def publish(
        self,
        writer: Callable[[Path], object],
        *,
        expected_sha256: str | None = None,
        reuse_if_valid: bool = False,
        immutable_identity: bool = False,
        max_bytes: int | None = MAX_PUBLICATION_FILE_BYTES,
    ) -> Path:
        """Run ``writer`` on a fresh stage path and publish the result.

        ``reuse_if_valid`` keeps an existing file whose hash equals
        ``expected_sha256``. ``immutable_identity`` records the hash in the
        identity marker and rejects any later publication with another hash.
        """

        if immutable_identity and expected_sha256 is None:
            raise ValueError("immutable publication requires expected_sha256")
        limit = _limit(max_bytes, MAX_PUBLICATION_FILE_BYTES)
        with self._locked():
            if immutable_identity:
                assert expected_sha256 is not None
                self._check_immutable(expected_sha256, limit)
            if (
                reuse_if_valid
                and expected_sha256 is not None
                and self._current_sha256(limit) == expected_sha256
            ):
                return self.path
            return self._publish_from_stage(
                writer,
                expected_sha256=expected_sha256,
                immutable_identity=immutable_identity,
                max_bytes=limit,
            )

    def publish_bytes(
        self, payload: bytes, *, max_bytes: int | None = MAX_PUBLICATION_FILE_BYTES
    ) -> Path:
        return self.publish(
            lambda staging: staging.write_bytes(payload), max_bytes=max_bytes
        )

    def publish_staged(
        self,
        staging: Path,
        *,
        expected_sha256: str | None = None,
        immutable_identity: bool = False,
        max_bytes: int | None = MAX_PUBLICATION_FILE_BYTES,
    ) -> Path:
        """Publish a copy of a caller-owned file; the source is left in place."""

        if immutable_identity and expected_sha256 is None:
            raise ValueError("immutable publication requires expected_sha256")
        with self._locked():
            return self._publish_from_stage(
                lambda stage: shutil.copyfile(staging, stage),
                expected_sha256=expected_sha256,
                immutable_identity=immutable_identity,
                max_bytes=max_bytes,
            )

    def append_bytes(self, payload: bytes) -> Path:
        """Publish the current bytes followed by ``payload`` as one replacement."""

        if not isinstance(payload, bytes):
            raise TypeError("publication payload must be bytes")

        def fill(stage: Path) -> None:
            with stage.open("wb") as sink:
                source = _open_regular_or_none(self.path)
                if source is not None:
                    with source:
                        shutil.copyfileobj(source, sink, _CHUNK)
                sink.write(payload)

        with self._locked():
            return self._publish_from_stage(fill)

    def stage(self) -> "PublicationStage":
        """Return a locked stage for incremental writers.

        Write ``stage.path`` and call ``stage.publish()``; leaving the context
        without publishing discards the stage. The lock is held until then.
        """

        stack = ExitStack()
        stack.enter_context(self._locked())
        try:
            path = self._new_stage()
        except BaseException:
            stack.close()
            raise
        return PublicationStage(self, path, _release=stack.close)

    def read(
        self,
        parser: Callable[[bytes], T],
        *,
        remove_invalid: bool = False,
        max_bytes: int | None = MAX_IN_MEMORY_FILE_BYTES,
    ) -> T | None:
        """Parse the current file; a missing file returns ``None``.

        Parser errors propagate. With ``remove_invalid`` the rejected file is
        removed first, unless another writer already replaced it.
        """

        payload = read_regular_file_bytes(
            self.path, max_bytes=_limit(max_bytes, MAX_IN_MEMORY_FILE_BYTES)
        )
        if payload is None:
            return None
        established = self._read_identity()
        if established is not None:
            actual = hashlib.sha256(payload).hexdigest()
            if actual != established:
                raise PublicationIntegrityError(
                    path=self.path, expected_sha256=established, actual_sha256=actual
                )
        try:
            return parser(payload)
        except Exception:
            if remove_invalid:
                with self._locked():
                    if (
                        read_regular_file_bytes(self.path, max_bytes=len(payload))
                        == payload
                    ):
                        self.path.unlink(missing_ok=True)
            raise

    @contextmanager
    def open_snapshot(
        self,
        *,
        expected_sha256: str | None = None,
        max_bytes: int | None = MAX_EXTERNAL_FILE_BYTES,
    ) -> Iterator[PublicationSnapshot | None]:
        """Yield the verified, open current file, or ``None`` when missing."""

        handle = _open_regular_or_none(self.path)
        if handle is None:
            yield None
            return
        with handle:
            actual, size = _copy_hashed(
                handle,
                None,
                path=self.path,
                max_bytes=_limit(max_bytes, MAX_EXTERNAL_FILE_BYTES),
            )
            for expected in (expected_sha256, self._read_identity()):
                if expected is not None and actual != expected:
                    raise PublicationIntegrityError(
                        path=self.path, expected_sha256=expected, actual_sha256=actual
                    )
            handle.seek(0)
            yield PublicationSnapshot(self.path, actual, size, handle)


@dataclass(frozen=True)
class _FileCommitFinisher:
    """Make a replaced file durable and record its immutable identity."""

    publication: FilePublication
    identity: str | None

    def retry_cleanup(self) -> None:
        _fsync(self.publication.path.parent)
        marker = self.publication._identity_path
        if self.identity is not None and not marker.exists():
            _atomic_write_bytes(marker, f"{self.identity}\n".encode("ascii"))


@dataclass(eq=False)
class PublicationStage:
    """A locked file stage returned by :meth:`FilePublication.stage`."""

    publication: FilePublication
    path: Path
    _release: Callable[[], object] | None = field(default=None, repr=False)
    _state: str = "open"

    def __enter__(self) -> "PublicationStage":
        if self._state != "open":
            raise PublicationError(f"publication stage is already {self._state}")
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self.discard()

    def _require_open(self) -> None:
        if self._state != "open":
            raise PublicationError(f"publication stage is already {self._state}")

    def copy_current(
        self, *, max_bytes: int | None = MAX_PUBLICATION_FILE_BYTES
    ) -> bool:
        """Seed the stage with the current file; ``False`` when there is none."""

        self._require_open()
        source = _open_regular_or_none(self.publication.path)
        if source is None:
            return False
        with source, self.path.open("wb") as sink:
            _copy_hashed(
                source,
                sink,
                path=self.publication.path,
                max_bytes=_limit(max_bytes, MAX_PUBLICATION_FILE_BYTES),
            )
        return True

    def publish(
        self,
        *,
        expected_sha256: str | None = None,
        immutable_identity: bool = False,
        max_bytes: int | None = MAX_PUBLICATION_FILE_BYTES,
    ) -> Path:
        self._require_open()
        if immutable_identity and expected_sha256 is None:
            raise ValueError("immutable publication requires expected_sha256")
        try:
            result = self.publication._promote_locked(
                self.path,
                expected_sha256=expected_sha256,
                immutable_identity=immutable_identity,
                max_bytes=max_bytes,
            )
        except PublicationCommitCleanupError:
            self._state = "published"
            self._release_lock()
            raise
        except BaseException:
            self.discard()
            raise
        self._state = "published"
        self._release_lock()
        return result

    def discard(self) -> None:
        if self._state == "open":
            self.path.unlink(missing_ok=True)
            self._state = "discarded"
        self._release_lock()

    def retry_cleanup(self) -> None:
        self.discard()

    def _release_lock(self) -> None:
        release, self._release = self._release, None
        if release is not None:
            release()


@contextmanager
def stage_file_publications(
    publications: Iterable[FilePublication],
) -> Iterator[tuple[PublicationStage, ...]]:
    """Stage several files under one combined lock, in input order.

    Publishing a stage keeps the combined lock; unpublished stages are
    discarded and every lock is released when the context exits.
    """

    selected = tuple(publications)
    if not selected:
        raise ValueError("stage_file_publications requires at least one publication")
    if len({publication.path for publication in selected}) != len(selected):
        raise ValueError("stage_file_publications requires distinct destinations")
    for publication in selected:
        publication.path.parent.mkdir(parents=True, exist_ok=True)
    with _locked(
        (path for publication in selected for path in publication._lock_paths()),
        timeout_s=min(publication.lock_timeout_s for publication in selected),
    ):
        stages: list[PublicationStage] = []
        try:
            for publication in selected:
                stages.append(PublicationStage(publication, publication._new_stage()))
            yield tuple(stages)
        finally:
            for stage in stages:
                stage.discard()


def _directory_manifest(root: Path) -> tuple[dict[str, object], dict[str, str]]:
    directories, members = _inventory(
        root, excluded=frozenset({_DIRECTORY_INTERNAL_MANIFEST})
    )
    identity_payload: dict[str, object] = {
        "schema": _DIRECTORY_SCHEMA,
        "directories": sorted(directories),
        "members": dict(sorted(members.items())),
    }
    manifest = {
        **identity_payload,
        "identity_sha256": hashlib.sha256(
            _canonical_json(identity_payload)
        ).hexdigest(),
    }
    return manifest, {name: str(entry["sha256"]) for name, entry in members.items()}


def _open_directory(
    destination: Path, *, required_members: Sequence[str] = ()
) -> tuple[PublishedDirectory, str]:
    """Open a published directory; checks the manifest and member sizes."""

    if destination.is_symlink() or not destination.is_dir():
        raise PublicationError(f"directory publication is missing: {destination}")
    payload = read_regular_file_bytes(
        destination / _DIRECTORY_INTERNAL_MANIFEST, max_bytes=_MAX_METADATA_BYTES
    )
    if payload is None:
        raise PublicationError(f"directory has no publication manifest: {destination}")
    manifest = _parse_json_object(payload)
    members = manifest.get("members")
    identity = manifest.get("identity_sha256")
    if manifest.get("schema") != _DIRECTORY_SCHEMA or not isinstance(members, dict):
        raise PublicationError(
            f"unsupported directory publication manifest: {destination}"
        )
    identity_payload = {
        key: manifest.get(key) for key in ("schema", "directories", "members")
    }
    if identity != hashlib.sha256(_canonical_json(identity_payload)).hexdigest():
        raise PublicationError(
            f"directory publication manifest is corrupt: {destination}"
        )
    hashes: dict[str, str] = {}
    for name, entry in members.items():
        relative = _normalize_directory_member(str(name))
        if not isinstance(entry, dict) or not _is_sha256(entry.get("sha256")):
            raise PublicationError(f"invalid directory publication member: {name}")
        try:
            size = os.lstat(destination / relative).st_size
        except FileNotFoundError as exc:
            raise PublicationError(
                f"published member is missing: {destination / relative}"
            ) from exc
        if size != entry.get("size_bytes"):
            raise PublicationError(
                f"published member changed size: {destination / relative}"
            )
        hashes[relative] = str(entry["sha256"])
    missing = sorted(set(required_members) - set(hashes))
    if missing:
        raise PublicationError(
            f"directory publication missing required member(s): {missing}"
        )
    return PublishedDirectory(destination, MappingProxyType(hashes)), str(identity)


def _publish_directory(
    destination: Path,
    builder: Callable[[Path], object],
    *,
    required_members: Sequence[str],
    lock_timeout_s: float,
    lock_path: Path | None,
) -> PublishedDirectory:
    """Publish a write-once directory: an identical republish is a no-op."""

    destination.parent.mkdir(parents=True, exist_ok=True)
    with _locked((_lock_file(destination), lock_path), timeout_s=lock_timeout_s):
        _remove_stale_stages(destination)
        staging = Path(
            tempfile.mkdtemp(
                prefix=f".{destination.name}{_STAGE_INFIX}",
                suffix=_STAGE_SUFFIX,
                dir=destination.parent,
            )
        )
        try:
            builder(staging)
            manifest, hashes = _directory_manifest(staging)
            missing = sorted(set(required_members) - set(hashes))
            if missing:
                raise PublicationError(
                    f"directory publication missing required member(s): {missing}"
                )
            if os.path.lexists(destination):
                established, identity = _open_directory(destination)
                if identity != manifest["identity_sha256"]:
                    raise PublicationIdentityError(
                        path=destination,
                        expected_sha256=str(manifest["identity_sha256"]),
                        actual_sha256=identity,
                    )
                return established
            (staging / _DIRECTORY_INTERNAL_MANIFEST).write_bytes(
                _canonical_json(manifest)
            )
            file_mode = default_new_file_mode()
            os.chmod(staging, file_mode | (file_mode & 0o444) >> 2)
            _fsync_tree(staging)
            os.rename(staging, destination)
            _fsync(destination.parent)
            return PublishedDirectory(destination, MappingProxyType(hashes))
        finally:
            if os.path.lexists(staging):
                shutil.rmtree(staging)


def _read_pointer_generation_id(pointer_path: Path) -> str | None:
    """Return the generation selected by a pointer, or None when unreadable."""

    try:
        payload = read_regular_file_bytes(pointer_path, max_bytes=_MAX_METADATA_BYTES)
        if payload is None:
            return None
        generation_id = _parse_json_object(payload).get("generation_id")
    except PublicationError:
        return None
    return generation_id if _is_generation_id(generation_id) else None  # type: ignore[return-value]


@dataclass(frozen=True)
class GenerationPublication:
    """Immutable directory generations selected by one atomic pointer.

    A writer builds ``.staging-<id>``, renames it to ``generations/<id>`` and
    then replaces ``current.json``. The new and the previous generation are
    kept so a reader that opened the previous one can finish.
    """

    root: Path
    lock_timeout_s: float = 30.0
    lock_path: Path | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "root", _canonical(self.root))
        if self.lock_path is not None:
            object.__setattr__(self, "lock_path", _canonical(self.lock_path))

    @property
    def path(self) -> Path:
        return self.root

    @property
    def _generations(self) -> Path:
        return self.root / "generations"

    @property
    def _pointer(self) -> Path:
        return self.root / _CURRENT_POINTER

    def _locked(self) -> ContextManager[None]:
        self.root.parent.mkdir(parents=True, exist_ok=True)
        return _locked(
            (_lock_file(self.root), self.lock_path), timeout_s=self.lock_timeout_s
        )

    def _prepare_locked(self) -> None:
        """Create the layout and drop state left by a crashed writer."""

        if self.root.is_symlink():
            raise PublicationError(f"generation root is a symlink: {self.root}")
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        # Legacy sidecar reads, removals, and recovery share this publication lock.
        name = glob.escape(self.root.name)
        parent = self.root.parent
        for stale in (
            *self.root.glob(".staging-*"),
            *self.root.glob(".current.json.*.tmp"),
            *self.root.glob(".generation-recovery-*.json"),
            *parent.glob(f".{name}.publication.readers.lock"),
            *parent.glob(f".{name}.publication.reader." + "[0-9a-f]" * 32),
            *parent.glob(f".{name}.publication.remove.*.owner"),
            *parent.glob(f".{name}.publication.tombstone.*"),
        ):
            _remove_path(stale)
        self._generations.mkdir(exist_ok=True)
        current = _read_pointer_generation_id(self._pointer)
        for generation in self._generations.iterdir():
            if (generation / _GENERATION_PENDING).exists():
                if generation.name == current:
                    (generation / _GENERATION_PENDING).unlink()
                else:
                    _remove_path(generation)

    def _open(self, required_members: Sequence[str]) -> PublishedGeneration:
        if not self.root.is_dir():
            raise PublicationNotFoundError(f"no publication at {self.root}")
        payload = read_regular_file_bytes(self._pointer, max_bytes=_MAX_METADATA_BYTES)
        if payload is None:
            raise PublicationNotFoundError(f"no current generation at {self.root}")
        pointer = _parse_json_object(payload)
        generation_id = pointer.get("generation_id")
        if pointer.get("schema") != _GENERATION_SCHEMA or not _is_generation_id(
            generation_id
        ):
            raise PublicationError(f"unsupported generation pointer: {self._pointer}")
        path = self._generations / str(generation_id)
        manifest_bytes = read_regular_file_bytes(
            path / _GENERATION_MANIFEST, max_bytes=_MAX_METADATA_BYTES
        )
        if manifest_bytes is None:
            raise PublicationError(f"current generation is missing: {path}")
        actual = hashlib.sha256(manifest_bytes).hexdigest()
        if actual != pointer.get("manifest_sha256"):
            raise PublicationIntegrityError(
                path=path / _GENERATION_MANIFEST,
                expected_sha256=str(pointer.get("manifest_sha256")),
                actual_sha256=actual,
            )
        manifest = _parse_json_object(manifest_bytes)
        members = manifest.get("members")
        if (
            manifest.get("schema") != _GENERATION_SCHEMA
            or manifest.get("generation_id") != generation_id
            or not isinstance(members, dict)
        ):
            raise PublicationError(f"invalid generation manifest: {path}")
        hashes: dict[str, str] = {}
        for name, entry in members.items():
            relative = _normalize_member(str(name))
            if not isinstance(entry, dict) or not _is_sha256(entry.get("sha256")):
                raise PublicationError(f"invalid generation member: {name}")
            try:
                size = os.lstat(path / relative).st_size
            except FileNotFoundError as exc:
                raise PublicationError(
                    f"generation member is missing: {path / relative}"
                ) from exc
            if size != entry.get("size_bytes"):
                raise PublicationError(
                    f"generation member changed size: {path / relative}"
                )
            hashes[relative] = str(entry["sha256"])
        missing = sorted(set(required_members) - set(hashes))
        if missing:
            raise PublicationError(
                f"generation is missing required member(s): {missing}"
            )
        return PublishedGeneration(str(generation_id), path, MappingProxyType(hashes))

    def _publish_locked(
        self, builder: Callable[[Path], object], required_members: Sequence[str]
    ) -> PublishedGeneration:
        generation_id = uuid.uuid4().hex
        staging = self.root / f".staging-{generation_id}"
        target = self._generations / generation_id
        staging.mkdir(mode=0o700)
        try:
            builder(staging)
            _directories, members = _inventory(
                staging, excluded=frozenset({_GENERATION_MANIFEST, _GENERATION_PENDING})
            )
            missing = sorted(set(required_members) - set(members))
            if missing:
                raise PublicationError(
                    f"generation publication missing required member(s): {missing}"
                )
            manifest = _canonical_json(
                {
                    "schema": _GENERATION_SCHEMA,
                    "generation_id": generation_id,
                    "members": dict(sorted(members.items())),
                }
            )
            (staging / _GENERATION_MANIFEST).write_bytes(manifest)
            # Marks a generation that is not yet behind the pointer.
            (staging / _GENERATION_PENDING).write_bytes(b"{}")
            _fsync_tree(staging)
            os.rename(staging, target)
            # The new entry must be durable before the pointer names it.
            _fsync(self._generations)
        finally:
            if os.path.lexists(staging):
                shutil.rmtree(staging)
        previous = _read_pointer_generation_id(self._pointer)
        _atomic_write_bytes(
            self._pointer,
            _canonical_json(
                {
                    "schema": _GENERATION_SCHEMA,
                    "generation_id": generation_id,
                    "manifest_sha256": hashlib.sha256(manifest).hexdigest(),
                }
            ),
        )
        (target / _GENERATION_PENDING).unlink()
        _fsync(target)
        if previous is not None:
            for generation in self._generations.iterdir():
                if generation.name not in {generation_id, previous}:
                    shutil.rmtree(generation, ignore_errors=True)
        hashes = {name: str(entry["sha256"]) for name, entry in members.items()}
        return PublishedGeneration(generation_id, target, MappingProxyType(hashes))

    def publish(
        self,
        builder: Callable[[Path], object],
        *,
        required_members: Sequence[str],
        **_limits: object,
    ) -> PublishedGeneration:
        """Build a new generation with ``builder(staging)`` and select it."""

        required = _normalize_required_members(required_members)
        with self._locked():
            self._prepare_locked()
            return self._publish_locked(builder, required)

    def publish_or_reuse(
        self,
        builder: Callable[[Path], object],
        *,
        required_members: Sequence[str],
        reuse_if: Callable[[PublishedGeneration], bool] | None = None,
        repair_invalid: bool = False,
        **_limits: object,
    ) -> PublishedGeneration:
        """Reuse the current generation when ``reuse_if`` accepts it, else build.

        A rejected generation is rebuilt only with ``repair_invalid``;
        exceptions raised by ``reuse_if`` propagate.
        """

        required = _normalize_required_members(required_members)
        with self._locked():
            self._prepare_locked()
            try:
                current = self._open(required)
            except PublicationNotFoundError:
                current = None
            if current is not None:
                if reuse_if is None or reuse_if(current):
                    return current
                if not repair_invalid:
                    raise PublicationReuseError(
                        f"current generation failed its reuse check: {self.root}"
                    )
            return self._publish_locked(builder, required)

    def update(
        self,
        updater: Callable[[Path, PublishedGeneration], object],
        *,
        required_members: Sequence[str],
        **_limits: object,
    ) -> PublishedGeneration:
        """Build a new generation from the current one under one lock."""

        required = _normalize_required_members(required_members)
        if not self.root.is_dir():
            raise PublicationNotFoundError(f"no publication at {self.root}")
        with self._locked():
            self._prepare_locked()
            current = self._open(required)
            return self._publish_locked(
                lambda staging: updater(staging, current), required
            )

    def open(
        self, *, required_members: Sequence[str] = (), **_limits: object
    ) -> PublishedGeneration:
        """Open the current generation without taking the writer lock."""

        return self._open(_normalize_required_members(required_members))

    def remove(self) -> bool:
        """Remove the whole publication root; ``False`` when it is absent."""

        with self._locked():
            if not os.path.lexists(self.root):
                return False
            if self.root.is_symlink() or not self.root.is_dir():
                raise PublicationError(
                    f"generation root is not a directory: {self.root}"
                )
            tombstone = self.root.with_name(
                f".{self.root.name}.publication.tombstone.{uuid.uuid4().hex}"
            )
            os.rename(self.root, tombstone)
            _fsync(self.root.parent)
            shutil.rmtree(tombstone)
            return True

    def remove_unpublished_root(self, *, allowed_members: Sequence[str] = ()) -> bool:
        """Remove a root that never got a pointer.

        Only an empty ``generations/``, stale staging and ``allowed_members``
        may be present; anything else raises. ``False`` when the root is
        absent or published.
        """

        allowed = set(_normalize_required_members(allowed_members))
        with self._locked():
            if self.root.is_symlink() or not self.root.is_dir():
                return False
            if os.path.lexists(self._pointer):
                return False
            # Without a pointer every generation is still pending.
            self._prepare_locked()
            for entry in self.root.iterdir():
                if (
                    entry.name == "generations"
                    and entry.is_dir()
                    and not entry.is_symlink()
                ):
                    if any(entry.iterdir()):
                        raise PublicationError(
                            "unpublished generation root contains live or unknown stages"
                        )
                elif entry.name.startswith((".staging-", ".current.json.")) or (
                    entry.name in allowed and entry.is_file() and not entry.is_symlink()
                ):
                    continue
                else:
                    raise PublicationError(
                        f"unpublished generation root contains unknown state: {entry}"
                    )
            shutil.rmtree(self.root)
            _fsync(self.root.parent)
            return True


@dataclass(frozen=True)
class ArtifactPublication:
    """One entry point: pick a file, directory or generation destination."""

    path: Path
    kind: str = "file"
    lock_timeout_s: float = 30.0
    lock_path: Path | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "path", _canonical(self.path))
        if self.lock_path is not None:
            object.__setattr__(self, "lock_path", _canonical(self.lock_path))
        if self.kind not in {"file", "directory", "generation"}:
            raise ValueError(
                "artifact publication kind must be 'file', 'directory', or 'generation'"
            )

    @classmethod
    def file(
        cls,
        path: Path | str,
        *,
        lock_timeout_s: float = 30.0,
        lock_path: Path | str | None = None,
    ) -> "ArtifactPublication":
        return cls(
            Path(path),
            "file",
            lock_timeout_s,
            None if lock_path is None else Path(lock_path),
        )

    @classmethod
    def directory(
        cls,
        path: Path | str,
        *,
        lock_timeout_s: float = 30.0,
        lock_path: Path | str | None = None,
    ) -> "ArtifactPublication":
        return cls(
            Path(path),
            "directory",
            lock_timeout_s,
            None if lock_path is None else Path(lock_path),
        )

    @classmethod
    def generation(
        cls,
        root: Path | str,
        *,
        lock_timeout_s: float = 30.0,
        lock_path: Path | str | None = None,
    ) -> "ArtifactPublication":
        return cls(
            Path(root),
            "generation",
            lock_timeout_s,
            None if lock_path is None else Path(lock_path),
        )

    def _require(self, kind: str) -> None:
        if self.kind != kind:
            raise ValueError(
                f"operation requires a {kind} publication, not {self.kind}"
            )

    @property
    def _file(self) -> FilePublication:
        self._require("file")
        return FilePublication(self.path, self.lock_timeout_s, self.lock_path)

    @property
    def _generation(self) -> GenerationPublication:
        self._require("generation")
        return GenerationPublication(self.path, self.lock_timeout_s, self.lock_path)

    def publish(
        self,
        writer: Callable[[Path], object],
        *,
        expected_sha256: str | None = None,
        reuse_if_valid: bool = False,
        immutable_identity: bool = False,
        required_members: Sequence[str] = (),
        max_bytes: int | None = None,
        **_limits: object,
    ) -> Path | PublishedDirectory | PublishedGeneration:
        if self.kind == "file":
            if required_members:
                raise ValueError(
                    "required_members are only valid for generation publication"
                )
            return self._file.publish(
                writer,
                expected_sha256=expected_sha256,
                reuse_if_valid=reuse_if_valid,
                immutable_identity=immutable_identity,
                max_bytes=max_bytes,
            )
        if expected_sha256 is not None or reuse_if_valid or immutable_identity:
            raise ValueError("file publication options are not valid for directories")
        if self.kind == "directory":
            return _publish_directory(
                self.path,
                writer,
                required_members=_normalize_required_members(required_members),
                lock_timeout_s=self.lock_timeout_s,
                lock_path=self.lock_path,
            )
        return self._generation.publish(writer, required_members=required_members)

    def open(
        self, *, required_members: Sequence[str] = (), **_limits: object
    ) -> PublishedDirectory | PublishedGeneration:
        if self.kind == "directory":
            return _open_directory(
                self.path,
                required_members=_normalize_required_members(required_members),
            )[0]
        return self._generation.open(required_members=required_members)

    # File operations.
    def publish_bytes(self, payload: bytes, *, max_bytes: int | None = None) -> Path:
        return self._file.publish_bytes(payload, max_bytes=max_bytes)

    def publish_staged(self, staging: Path, **kwargs: object) -> Path:
        return self._file.publish_staged(staging, **kwargs)  # type: ignore[arg-type]

    def append_bytes(self, payload: bytes) -> Path:
        return self._file.append_bytes(payload)

    def stage(self) -> PublicationStage:
        return self._file.stage()

    def read(self, parser: Callable[[bytes], T], **kwargs: object) -> T | None:
        return self._file.read(parser, **kwargs)  # type: ignore[arg-type]

    def open_snapshot(self, **kwargs: object):  # type: ignore[no-untyped-def]
        return self._file.open_snapshot(**kwargs)  # type: ignore[arg-type]

    # Generation operations.
    def publish_or_reuse(
        self, builder: Callable[[Path], object], **kwargs: object
    ) -> PublishedGeneration:
        return self._generation.publish_or_reuse(builder, **kwargs)  # type: ignore[arg-type]

    def update(
        self, updater: Callable[[Path, PublishedGeneration], object], **kwargs: object
    ) -> PublishedGeneration:
        return self._generation.update(updater, **kwargs)  # type: ignore[arg-type]

    def remove(self) -> bool:
        return self._generation.remove()

    def remove_unpublished_root(self, *, allowed_members: Sequence[str] = ()) -> bool:
        return self._generation.remove_unpublished_root(allowed_members=allowed_members)


__all__ = [
    "ArtifactPublication",
    "FilePublication",
    "GenerationPublication",
    "MaterializedDirectorySnapshot",
    "OwnedTemporaryDirectory",
    "PublicationCleanupError",
    "PublicationCommitCleanupError",
    "PublicationError",
    "PublicationIdentityError",
    "PublicationIntegrityError",
    "PublicationNotFoundError",
    "PublicationResourceLimitError",
    "PublicationReuseError",
    "PublicationSnapshot",
    "PublicationStage",
    "PublicationTimeoutError",
    "PublishedDirectory",
    "PublishedGeneration",
    "copy_verified_file_snapshot",
    "default_new_file_mode",
    "directory_member_hashes",
    "materialize_file_snapshot",
    "open_directory_descriptor",
    "read_regular_file_bytes",
    "sha256_file",
    "stage_file_publications",
]
