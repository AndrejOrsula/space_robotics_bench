#!/usr/bin/env python3
"""Bounded source inventory, verification, and private snapshot helper.

This module deliberately has no SRB imports.  It is used before a container
build, where the source checkout is an untrusted filesystem boundary and the
normal project environment may not exist yet.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import secrets
import selectors
import stat
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import IO, Any, Iterable, Mapping, Sequence, cast

SCHEMA_NAME = "srb.source-provenance"
SCHEMA_VERSION = 1
TREE_ALGORITHM = "srb-tree-sha256-v1"
PRODUCER_ID = "srb-source-provenance"
PRODUCER_VERSION = 1
BASH_PRODUCER_PATH = ".docker/helpers/release/source_provenance.bash"
PYTHON_PRODUCER_PATH = ".docker/helpers/release/source_provenance.py"
PRODUCER_PATHS = (BASH_PRODUCER_PATH, PYTHON_PRODUCER_PATH)

# Literal exclusions are part of the source identity contract.
CANONICAL_EXCLUSIONS = (
    ".cache",
    ".coverage",
    ".docker/.history",
    ".git",
    ".hypothesis",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".source-provenance.json",
    ".srb-source-provenance-owned",
    ".venv",
    ".worktrees",
    "__pycache__",
    "build",
    "dist",
    "docs/book",
    "docs/index.html",
    "log",
    "logs",
    "outputs",
    "wandb",
    "wheels",
)

MAX_ENTRIES = 100_000
MAX_DEPTH = 32
MAX_SUBMODULES = 100_000
MAX_SUBMODULE_DEPTH = 32
MAX_FILE_BYTES = 16 * 1024 * 1024 * 1024
MAX_TOTAL_BYTES = 16 * 1024 * 1024 * 1024
MAX_MANIFEST_BYTES = 64 * 1024 * 1024
MAX_GIT_OUTPUT_BYTES = 32 * 1024 * 1024
GIT_TIMEOUT_SECONDS = 30.0
READ_CHUNK = 1024 * 1024

_HEX64 = re.compile(r"[0-9a-f]{64}\Z")
_HEX_GIT = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})\Z")
_UUID = re.compile(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}\Z")


class ProvenanceError(RuntimeError):
    """A fail-closed source provenance error."""


def _canonical_json(value: Any) -> bytes:
    try:
        encoded = json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except (TypeError, ValueError, RecursionError) as exc:
        raise ProvenanceError(f"cannot encode canonical JSON: {exc}") from exc
    return (encoded + "\n").encode("utf-8")


def _canonical_path(value: str, *, label: str = "path") -> str:
    if not isinstance(value, str) or not value:
        raise ProvenanceError(f"{label} must be a non-empty string")
    if "\\" in value or any(
        ord(character) < 0x20 or ord(character) == 0x7F for character in value
    ):
        raise ProvenanceError(f"{label} contains a forbidden character")
    if value.startswith("/") or value.endswith("/") or "//" in value:
        raise ProvenanceError(f"{label} must be a canonical relative path")
    try:
        value.encode("utf-8")
    except UnicodeEncodeError as exc:
        raise ProvenanceError(f"{label} must be valid UTF-8") from exc
    parts = value.split("/")
    if any(part in ("", ".", "..") for part in parts):
        raise ProvenanceError(f"{label} contains traversal or empty components")
    if str(PurePosixPath(value)) != value:
        raise ProvenanceError(f"{label} is not canonical")
    if len(value.encode("utf-8")) > 4096:
        raise ProvenanceError(f"{label} exceeds the path-length bound")
    return value


def _validate_exclusions(values: Iterable[str]) -> tuple[str, ...]:
    normalized = tuple(_canonical_path(value, label="exclusion") for value in values)
    if len(set(normalized)) != len(normalized):
        raise ProvenanceError("exclusions contain duplicates")
    if normalized != tuple(sorted(normalized)):
        raise ProvenanceError("exclusions are not sorted")
    if normalized != CANONICAL_EXCLUSIONS:
        raise ProvenanceError(
            "manifest exclusions do not match the canonical helper list"
        )
    return normalized


def _is_excluded(path: str, exclusions: Sequence[str] = CANONICAL_EXCLUSIONS) -> bool:
    components = path.split("/")
    for exclusion in exclusions:
        nested_component = exclusion in {
            ".cache",
            ".coverage",
            ".git",
            ".hypothesis",
            ".mypy_cache",
            ".pytest_cache",
            ".ruff_cache",
            ".venv",
            ".worktrees",
            "__pycache__",
        }
        if "/" not in exclusion and nested_component and exclusion in components:
            return True
        if path == exclusion or path.startswith(exclusion + "/"):
            return True
    return False


def _check_regular_path(path: Path, *, label: str) -> os.stat_result:
    try:
        result = os.lstat(path)
    except OSError as exc:
        raise ProvenanceError(f"cannot stat {label} {path}: {exc}") from exc
    if stat.S_ISLNK(result.st_mode):
        raise ProvenanceError(f"symbolic links are not admitted: {path}")
    if not stat.S_ISREG(result.st_mode):
        raise ProvenanceError(f"special files are not admitted: {path}")
    return result


def _check_directory_path(path: Path, *, label: str) -> os.stat_result:
    try:
        result = os.lstat(path)
    except OSError as exc:
        raise ProvenanceError(f"cannot stat {label} {path}: {exc}") from exc
    if stat.S_ISLNK(result.st_mode):
        raise ProvenanceError(f"symbolic links are not admitted: {path}")
    if not stat.S_ISDIR(result.st_mode):
        raise ProvenanceError(f"{label} is not a directory: {path}")
    return result


def _same_file_identity(left: os.stat_result, right: os.stat_result) -> bool:
    return (
        left.st_dev == right.st_dev
        and left.st_ino == right.st_ino
        and stat.S_IFMT(left.st_mode) == stat.S_IFMT(right.st_mode)
    )


def _same_file_snapshot(left: os.stat_result, right: os.stat_result) -> bool:
    return (
        _same_file_identity(left, right)
        and stat.S_IMODE(left.st_mode) == stat.S_IMODE(right.st_mode)
        and left.st_size == right.st_size
        and left.st_mtime_ns == right.st_mtime_ns
        and left.st_ctime_ns == right.st_ctime_ns
    )


def _open_regular_for_read(path: Path) -> tuple[int, os.stat_result]:
    before = _check_regular_path(path, label="file")
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise ProvenanceError(f"cannot open regular file {path}: {exc}") from exc
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode) or not _same_file_snapshot(before, opened):
            raise ProvenanceError(f"file changed while opening: {path}")
        return descriptor, before
    except BaseException:
        os.close(descriptor)
        raise


def _check_regular_at(parent_fd: int, name: str, *, label: str) -> os.stat_result:
    try:
        result = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    except OSError as exc:
        raise ProvenanceError(f"cannot stat {label} {name}: {exc}") from exc
    if stat.S_ISLNK(result.st_mode):
        raise ProvenanceError(f"symbolic links are not admitted: {name}")
    if not stat.S_ISREG(result.st_mode):
        raise ProvenanceError(f"special files are not admitted: {name}")
    return result


def _open_regular_at(
    parent_fd: int, name: str, *, label: str
) -> tuple[int, os.stat_result]:
    before = _check_regular_at(parent_fd, name, label=label)
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    try:
        descriptor = os.open(name, flags, dir_fd=parent_fd)
    except OSError as exc:
        raise ProvenanceError(f"cannot open regular file {name}: {exc}") from exc
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode) or not _same_file_snapshot(before, opened):
            raise ProvenanceError(f"file changed while opening: {name}")
        return descriptor, before
    except BaseException:
        os.close(descriptor)
        raise


def _open_directory_at(
    parent_fd: int, name: str, *, label: str
) -> tuple[int, os.stat_result]:
    try:
        expected = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    except OSError as exc:
        raise ProvenanceError(f"cannot stat {label} {name}: {exc}") from exc
    if stat.S_ISLNK(expected.st_mode) or not stat.S_ISDIR(expected.st_mode):
        raise ProvenanceError(f"{label} is not a real directory: {name}")
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(name, flags, dir_fd=parent_fd)
    except OSError as exc:
        raise ProvenanceError(f"cannot open directory {name}: {exc}") from exc
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISDIR(opened.st_mode) or not _same_file_snapshot(
            expected, opened
        ):
            raise ProvenanceError(f"directory changed while opening: {name}")
        return descriptor, expected
    except BaseException:
        os.close(descriptor)
        raise


def _open_root_directory(root: Path) -> tuple[int, os.stat_result]:
    expected = _check_directory_path(root, label="source root")
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(root, flags)
    except OSError as exc:
        raise ProvenanceError(f"cannot open source root {root}: {exc}") from exc
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISDIR(opened.st_mode) or not _same_file_snapshot(
            expected, opened
        ):
            raise ProvenanceError(f"source root changed while opening: {root}")
        return descriptor, expected
    except BaseException:
        os.close(descriptor)
        raise


def _open_relative_parent(root: Path, relative_path: str) -> tuple[int, str]:
    """Open a source entry's parent using descriptor-relative no-follow hops."""

    components = relative_path.split("/")
    if not components or any(not component for component in components):
        raise ProvenanceError(f"invalid source path: {relative_path}")
    descriptor, _ = _open_root_directory(root)
    try:
        for component in components[:-1]:
            child, _ = _open_directory_at(
                descriptor, component, label="source directory"
            )
            os.close(descriptor)
            descriptor = child
        return descriptor, components[-1]
    except BaseException:
        os.close(descriptor)
        raise


@dataclass
class _Budget:
    entries: int = 0
    total_bytes: int = 0

    def add_entry(self, path: str) -> None:
        self.entries += 1
        if self.entries > MAX_ENTRIES:
            raise ProvenanceError(
                f"entry budget exceeded at {self.entries} (maximum {MAX_ENTRIES}): {path}"
            )


@dataclass(frozen=True)
class _SnapshotDirectory:
    relative_path: str
    identity: tuple[int, int]
    mode: int


def _hash_regular_at(
    parent_fd: int, name: str, display_path: str, budget: _Budget
) -> tuple[int, int, str, int]:
    descriptor, before = _open_regular_at(parent_fd, name, label="file")
    digest = hashlib.sha256()
    size = 0
    try:
        if (
            before.st_size > MAX_FILE_BYTES
            or budget.total_bytes + before.st_size > MAX_TOTAL_BYTES
        ):
            raise ProvenanceError(f"file byte budget exceeded: {display_path}")
        while True:
            chunk = os.read(descriptor, READ_CHUNK)
            if not chunk:
                break
            size += len(chunk)
            if (
                size > MAX_FILE_BYTES
                or budget.total_bytes + len(chunk) > MAX_TOTAL_BYTES
            ):
                raise ProvenanceError(
                    f"file byte budget exceeded while reading: {display_path}"
                )
            digest.update(chunk)
            budget.total_bytes += len(chunk)
        after_descriptor = os.fstat(descriptor)
        after_path = _check_regular_at(parent_fd, name, label="file")
        if not _same_file_snapshot(before, after_descriptor) or not _same_file_snapshot(
            before, after_path
        ):
            raise ProvenanceError(f"file changed while hashing: {display_path}")
        if size != before.st_size:
            raise ProvenanceError(f"file size changed while hashing: {display_path}")
        return size, stat.S_IMODE(before.st_mode), digest.hexdigest(), before.st_dev
    finally:
        os.close(descriptor)


def _safe_name(name: str, *, label: str) -> str:
    return _canonical_path(name, label=label)


def _walk_inventory(root: Path) -> list[dict[str, Any]]:
    root_descriptor, _ = _open_root_directory(root)
    budget = _Budget()
    entries: list[dict[str, Any]] = []
    seen: set[str] = set()

    def visit(directory_descriptor: int, relative_parent: str, depth: int) -> None:
        if depth > MAX_DEPTH:
            raise ProvenanceError(
                f"directory depth exceeds {MAX_DEPTH}: {relative_parent}"
            )
        try:
            children = os.scandir(directory_descriptor)
        except OSError as exc:
            raise ProvenanceError(
                f"cannot enumerate {relative_parent or '.'}: {exc}"
            ) from exc
        with children:
            for child in children:
                component = _safe_name(child.name, label="directory entry")
                relative = (
                    component
                    if not relative_parent
                    else f"{relative_parent}/{component}"
                )
                _canonical_path(relative)
                if _is_excluded(relative):
                    continue
                if relative in seen:
                    raise ProvenanceError(f"duplicate inventory path: {relative}")
                seen.add(relative)
                budget.add_entry(relative)
                try:
                    metadata = os.stat(
                        child.name, dir_fd=directory_descriptor, follow_symlinks=False
                    )
                except OSError as exc:
                    raise ProvenanceError(
                        f"cannot stat directory entry {relative}: {exc}"
                    ) from exc
                if stat.S_ISLNK(metadata.st_mode):
                    raise ProvenanceError(
                        f"symbolic links are not admitted: {relative}"
                    )
                if stat.S_ISDIR(metadata.st_mode):
                    if depth + 1 > MAX_DEPTH:
                        raise ProvenanceError(
                            f"directory depth exceeds {MAX_DEPTH}: {relative}"
                        )
                    entries.append(
                        {
                            "path": relative,
                            "kind": "directory",
                            "mode": stat.S_IMODE(metadata.st_mode),
                            "size": 0,
                        }
                    )
                    child_descriptor, _ = _open_directory_at(
                        directory_descriptor, child.name, label="source directory"
                    )
                    try:
                        visit(child_descriptor, relative, depth + 1)
                    finally:
                        os.close(child_descriptor)
                elif stat.S_ISREG(metadata.st_mode):
                    size, mode, digest, _ = _hash_regular_at(
                        directory_descriptor, child.name, relative, budget
                    )
                    entries.append(
                        {
                            "path": relative,
                            "kind": "file",
                            "mode": mode,
                            "size": size,
                            "sha256": digest,
                        }
                    )
                else:
                    raise ProvenanceError(f"special files are not admitted: {relative}")

    try:
        visit(root_descriptor, "", 0)
        entries.sort(key=lambda item: item["path"])
        return entries
    finally:
        os.close(root_descriptor)


def _git_command(repository: Path, arguments: Sequence[str]) -> str | None:
    command = ["git", "-C", os.fspath(repository), *arguments]
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=False,
        )
    except OSError:
        return None
    assert process.stdout is not None
    assert process.stderr is not None
    selector = selectors.DefaultSelector()
    buffers = {process.stdout: bytearray(), process.stderr: bytearray()}
    active = set(buffers)
    for stream in buffers:
        os.set_blocking(stream.fileno(), False)
        selector.register(stream, selectors.EVENT_READ)
    deadline = time.monotonic() + GIT_TIMEOUT_SECONDS
    try:
        while active:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                process.kill()
                process.wait()
                raise ProvenanceError("Git metadata command exceeded its time bound")
            events = selector.select(min(remaining, 0.25))
            for key, _ in events:
                stream = cast(IO[bytes], key.fileobj)
                try:
                    chunk = os.read(stream.fileno(), 64 * 1024)
                except BlockingIOError:
                    continue
                if not chunk:
                    selector.unregister(stream)
                    active.remove(stream)
                    stream.close()
                    continue
                buffer = buffers[stream]
                buffer.extend(chunk)
                if len(buffer) > MAX_GIT_OUTPUT_BYTES:
                    process.kill()
                    process.wait()
                    raise ProvenanceError(
                        "Git metadata output exceeds the bounded limit"
                    )
        return_code = process.wait(timeout=max(0.0, deadline - time.monotonic()))
    except subprocess.TimeoutExpired as exc:
        process.kill()
        process.wait()
        raise ProvenanceError("Git metadata command exceeded its time bound") from exc
    finally:
        selector.close()
        for stream in active:
            stream.close()
    output = bytes(buffers[process.stdout])
    if return_code != 0:
        return None
    try:
        return output.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ProvenanceError("Git metadata is not valid UTF-8") from exc


def _git_path(repository: Path) -> Path | None:
    output = _git_command(repository, ["rev-parse", "--show-toplevel"])
    if output is None:
        return None
    candidate = Path(output.strip())
    try:
        if candidate.resolve() != repository.resolve():
            return None
    except OSError as exc:
        raise ProvenanceError(f"cannot resolve Git root {repository}: {exc}") from exc
    return candidate


def _status_paths(repository: Path) -> list[str]:
    output = _git_command(
        repository,
        [
            "status",
            "--porcelain=v1",
            "-z",
            "--untracked-files=all",
            "--ignored=matching",
            "--no-renames",
        ],
    )
    if output is None:
        raise ProvenanceError(f"cannot inspect Git worktree: {repository}")
    dirty: list[str] = []
    for record in output.split("\0"):
        if not record:
            continue
        if len(record) < 4:
            dirty.append("")
            continue
        status = record[:2]
        path = record[3:].rstrip("/")
        if not path:
            dirty.append("")
            continue
        if status in {"??", "!!"} and _is_excluded(path):
            continue
        dirty.append(path)
    return dirty


def _git_submodule_records(
    root: Path, *, prefix: str = "", _count: list[int] | None = None
) -> list[dict[str, Any]]:
    if _count is None:
        _count = [0]
    output = _git_command(root, ["ls-tree", "-r", "-z", "HEAD"])
    if output is None:
        raise ProvenanceError(f"cannot inspect HEAD tree: {root}")
    records: list[dict[str, Any]] = []
    for record in output.split("\0"):
        if not record:
            continue
        if "\t" not in record:
            raise ProvenanceError("malformed Git tree record")
        header, path_value = record.split("\t", 1)
        fields = header.split()
        if len(fields) != 3:
            raise ProvenanceError("malformed Git tree header")
        mode, object_type, object_id = fields
        if mode != "160000":
            continue
        if object_type != "commit":
            raise ProvenanceError("malformed Git submodule tree record")
        path = _canonical_path(path_value, label="Git submodule path")
        full_path = path if not prefix else f"{prefix}/{path}"
        _canonical_path(full_path, label="Git submodule path")
        depth = full_path.count("/") + 1
        if depth > MAX_SUBMODULE_DEPTH:
            raise ProvenanceError(
                f"Git submodule depth exceeds {MAX_SUBMODULE_DEPTH}: {full_path}"
            )
        _count[0] += 1
        if _count[0] > MAX_SUBMODULES:
            raise ProvenanceError(
                f"Git submodule count exceeds {MAX_SUBMODULES}: {full_path}"
            )
        gitlink = object_id
        if not _HEX_GIT.fullmatch(gitlink):
            raise ProvenanceError(f"invalid Git submodule object id: {full_path}")
        submodule = root / Path(*path.split("/"))
        _check_directory_path(submodule, label="submodule")
        if _git_path(submodule) is None:
            raise ProvenanceError(
                f"submodule is not a populated Git checkout: {full_path}"
            )
        head = (_git_command(submodule, ["rev-parse", "HEAD"]) or "").strip()
        tree = (_git_command(submodule, ["rev-parse", "HEAD^{tree}"]) or "").strip()
        if not _HEX_GIT.fullmatch(head) or not _HEX_GIT.fullmatch(tree):
            raise ProvenanceError(f"submodule has no readable HEAD/tree: {full_path}")
        if head != gitlink:
            raise ProvenanceError(
                f"submodule HEAD does not match its gitlink: {full_path}"
            )
        status = _status_paths(submodule)
        records.append(
            {
                "path": full_path,
                "gitlink": gitlink,
                "head": head,
                "tree": tree,
                "worktree": "dirty" if status else "clean",
            }
        )
        records.extend(
            _git_submodule_records(submodule, prefix=full_path, _count=_count)
        )
    records.sort(key=lambda item: item["path"])
    return records


def _source_metadata(root: Path, *, require_git: bool) -> dict[str, Any]:
    git_root = _git_path(root)
    if git_root is None:
        if require_git:
            raise ProvenanceError("source generation requires a Git checkout")
        return {
            "root_commit": None,
            "root_tree": None,
            "worktree": "unknown",
            "submodules": [],
        }
    root_commit = (_git_command(root, ["rev-parse", "HEAD"]) or "").strip()
    root_tree = (_git_command(root, ["rev-parse", "HEAD^{tree}"]) or "").strip()
    if not _HEX_GIT.fullmatch(root_commit) or not _HEX_GIT.fullmatch(root_tree):
        raise ProvenanceError("Git checkout has no readable HEAD/tree")
    status = _status_paths(root)
    submodules = _git_submodule_records(root)
    if any(item["worktree"] == "dirty" for item in submodules):
        status.append("<submodule>")
    return {
        "root_commit": root_commit,
        "root_tree": root_tree,
        "worktree": "dirty" if status else "clean",
        "submodules": submodules,
    }


def _tree_digest(
    entries: Sequence[Mapping[str, Any]], source: Mapping[str, Any]
) -> str:
    payload = {
        "tree_algorithm": TREE_ALGORITHM,
        "exclusions": list(CANONICAL_EXCLUSIONS),
        "source": {
            "root_commit": source["root_commit"],
            "root_tree": source["root_tree"],
            "worktree": source["worktree"],
            "submodules": source["submodules"],
        },
        "entries": list(entries),
    }
    domain = b"srb.source-provenance\0tree\0srb-tree-sha256-v1\0"
    return hashlib.sha256(domain + _canonical_json(payload)).hexdigest()


def _producer_files(entries: Sequence[Mapping[str, Any]]) -> list[dict[str, str]]:
    by_path = {entry["path"]: entry for entry in entries}
    result: list[dict[str, str]] = []
    for path in PRODUCER_PATHS:
        entry = by_path.get(path)
        if (
            entry is None
            or entry.get("kind") != "file"
            or not isinstance(entry.get("sha256"), str)
        ):
            raise ProvenanceError(
                f"producer file is missing from the source inventory: {path}"
            )
        result.append({"path": path, "sha256": entry["sha256"]})
    return result


def _manifest_payload(
    *,
    snapshot_id: str,
    entries: Sequence[Mapping[str, Any]],
    source: Mapping[str, Any],
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "snapshot_id": snapshot_id,
        "tree_algorithm": TREE_ALGORITHM,
        "tree_digest": _tree_digest(entries, source),
        "exclusions": list(CANONICAL_EXCLUSIONS),
        "source": {
            "root_commit": source["root_commit"],
            "root_tree": source["root_tree"],
            "worktree": source["worktree"],
            "submodules": list(source["submodules"]),
        },
        "entries": list(entries),
        "producer": {
            "id": PRODUCER_ID,
            "version": PRODUCER_VERSION,
            "files": _producer_files(entries),
        },
    }
    return payload


def _read_bounded_file(path: Path, limit: int, *, label: str) -> bytes:
    before = _check_regular_path(path, label=label)
    if before.st_size > limit:
        raise ProvenanceError(f"{label} exceeds the {limit}-byte bound: {path}")
    descriptor, opened_before = _open_regular_for_read(path)
    data = bytearray()
    try:
        while True:
            chunk = os.read(descriptor, min(READ_CHUNK, limit - len(data) + 1))
            if not chunk:
                break
            data.extend(chunk)
            if len(data) > limit:
                raise ProvenanceError(f"{label} exceeds the {limit}-byte bound: {path}")
        after_descriptor = os.fstat(descriptor)
        after_path = _check_regular_path(path, label=label)
        if not _same_file_snapshot(
            opened_before, after_descriptor
        ) or not _same_file_snapshot(opened_before, after_path):
            raise ProvenanceError(f"{label} changed while being read: {path}")
        if len(data) != opened_before.st_size:
            raise ProvenanceError(f"{label} size changed while being read: {path}")
        return bytes(data)
    finally:
        os.close(descriptor)


class _RejectDuplicateKeys(ValueError):
    pass


def _object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise _RejectDuplicateKeys(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number: {value}")


def _load_manifest(path: Path) -> dict[str, Any]:
    raw = _read_bounded_file(path, MAX_MANIFEST_BYTES, label="manifest")
    return decode_manifest(raw)


def decode_manifest(raw: bytes) -> dict[str, Any]:
    """Decode and validate bounded canonical version-1 source manifest bytes."""

    if not isinstance(raw, bytes):
        raise ProvenanceError("manifest input must be bytes")
    if len(raw) > MAX_MANIFEST_BYTES:
        raise ProvenanceError(f"manifest exceeds the {MAX_MANIFEST_BYTES}-byte bound")
    try:
        text = raw.decode("utf-8")
        value = json.loads(
            text,
            object_pairs_hook=_object_pairs,
            parse_constant=_reject_json_constant,
        )
    except (
        UnicodeDecodeError,
        json.JSONDecodeError,
        RecursionError,
        ValueError,
    ) as exc:
        raise ProvenanceError(f"manifest is not valid UTF-8 JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ProvenanceError("manifest root must be an object")
    try:
        canonical = _canonical_json(value)
    except UnicodeEncodeError as exc:
        raise ProvenanceError("manifest is not canonical UTF-8 JSON") from exc
    if raw != canonical:
        raise ProvenanceError("manifest is not canonical UTF-8 JSON")
    _validate_manifest(value)
    return value


def _validate_uuid(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or _UUID.fullmatch(value) is None:
        raise ProvenanceError(f"{label} must be a lowercase canonical UUID")
    try:
        if str(uuid.UUID(value)) != value:
            raise ValueError
    except ValueError as exc:
        raise ProvenanceError(f"{label} must be a lowercase canonical UUID") from exc
    return value


def _validate_digest(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or _HEX64.fullmatch(value) is None:
        raise ProvenanceError(f"{label} must be a lowercase SHA-256 digest")
    return value


def _validate_git_id(value: Any, *, label: str, nullable: bool = True) -> str | None:
    if value is None and nullable:
        return None
    if not isinstance(value, str) or _HEX_GIT.fullmatch(value) is None:
        raise ProvenanceError(f"{label} must be a Git object id or null")
    return value


def _validate_manifest(value: Mapping[str, Any]) -> None:
    required = {
        "schema",
        "schema_version",
        "snapshot_id",
        "tree_algorithm",
        "tree_digest",
        "exclusions",
        "source",
        "entries",
        "producer",
    }
    if set(value) != required:
        raise ProvenanceError("manifest has unknown or missing top-level fields")
    if (
        value["schema"] != SCHEMA_NAME
        or type(value["schema_version"]) is not int
        or value["schema_version"] != SCHEMA_VERSION
    ):
        raise ProvenanceError("unsupported source provenance schema")
    _validate_uuid(value["snapshot_id"], label="snapshot_id")
    if value["tree_algorithm"] != TREE_ALGORITHM:
        raise ProvenanceError("unsupported tree algorithm")
    _validate_digest(value["tree_digest"], label="tree_digest")
    exclusions = value["exclusions"]
    if not isinstance(exclusions, list) or any(
        not isinstance(item, str) for item in exclusions
    ):
        raise ProvenanceError("exclusions must be a list of strings")
    _validate_exclusions(exclusions)

    source = value["source"]
    if not isinstance(source, dict) or set(source) != {
        "root_commit",
        "root_tree",
        "worktree",
        "submodules",
    }:
        raise ProvenanceError("source has unknown or missing fields")
    _validate_git_id(source["root_commit"], label="source.root_commit")
    _validate_git_id(source["root_tree"], label="source.root_tree")
    if not isinstance(source["worktree"], str) or source["worktree"] not in {
        "clean",
        "dirty",
        "unknown",
    }:
        raise ProvenanceError("source.worktree must be clean, dirty, or unknown")
    submodules = source["submodules"]
    if not isinstance(submodules, list) or len(submodules) > MAX_SUBMODULES:
        raise ProvenanceError(
            f"source.submodules must contain at most {MAX_SUBMODULES} records"
        )
    previous_path: str | None = None
    for index, item in enumerate(submodules):
        if not isinstance(item, dict) or set(item) != {
            "path",
            "gitlink",
            "head",
            "tree",
            "worktree",
        }:
            raise ProvenanceError(f"invalid source.submodules[{index}]")
        path = _canonical_path(item["path"], label=f"source.submodules[{index}].path")
        if path.count("/") + 1 > MAX_SUBMODULE_DEPTH:
            raise ProvenanceError(
                f"source submodule depth exceeds {MAX_SUBMODULE_DEPTH}: {path}"
            )
        if previous_path is not None and path <= previous_path:
            raise ProvenanceError("source.submodules are not sorted and unique")
        previous_path = path
        _validate_git_id(
            item["gitlink"], label=f"source.submodules[{index}].gitlink", nullable=False
        )
        _validate_git_id(
            item["head"], label=f"source.submodules[{index}].head", nullable=True
        )
        _validate_git_id(
            item["tree"], label=f"source.submodules[{index}].tree", nullable=True
        )
        if not isinstance(item["worktree"], str) or item["worktree"] not in {
            "clean",
            "dirty",
            "unknown",
        }:
            raise ProvenanceError(f"invalid source.submodules[{index}].worktree")

    entries = value["entries"]
    if not isinstance(entries, list) or len(entries) > MAX_ENTRIES:
        raise ProvenanceError(
            f"entries must be a list of at most {MAX_ENTRIES} records"
        )
    previous_path = None
    total_bytes = 0
    entry_kinds: dict[str, str] = {}
    for index, item in enumerate(entries):
        if not isinstance(item, dict):
            raise ProvenanceError(f"invalid entries[{index}]")
        kind = item.get("kind")
        if not isinstance(kind, str) or kind not in {"file", "directory"}:
            raise ProvenanceError(f"invalid entries[{index}].kind")
        expected_keys = {"path", "kind", "mode", "size"}
        if kind == "file":
            expected_keys.add("sha256")
        if set(item) != expected_keys:
            raise ProvenanceError(f"invalid fields in entries[{index}]")
        raw_path = item.get("path")
        if not isinstance(raw_path, str):
            raise ProvenanceError(f"invalid entries[{index}].path")
        path = _canonical_path(raw_path, label=f"entries[{index}].path")
        if _is_excluded(path):
            raise ProvenanceError(f"excluded path appears in entries: {path}")
        if previous_path is not None and path <= previous_path:
            raise ProvenanceError("entries are not sorted and unique")
        previous_path = path
        parent_depth = path.count("/")
        if kind == "directory" and parent_depth + 1 > MAX_DEPTH:
            raise ProvenanceError(f"directory depth exceeds {MAX_DEPTH}: {path}")
        if kind == "file" and parent_depth > MAX_DEPTH:
            raise ProvenanceError(f"directory depth exceeds {MAX_DEPTH}: {path}")
        parent = PurePosixPath(path).parent
        while str(parent) != ".":
            if entry_kinds.get(str(parent)) != "directory":
                raise ProvenanceError(f"inventory path has no directory parent: {path}")
            parent = parent.parent
        entry_kinds[path] = kind
        mode = item.get("mode")
        if type(mode) is not int or mode < 0 or mode > 0o7777:
            raise ProvenanceError(f"invalid entries[{index}].mode")
        size = item.get("size")
        if type(size) is not int or size < 0 or size > MAX_FILE_BYTES:
            raise ProvenanceError(f"invalid entries[{index}].size")
        if kind == "directory":
            if size != 0:
                raise ProvenanceError(f"directory entry size must be zero: {path}")
        else:
            total_bytes += size
            if total_bytes > MAX_TOTAL_BYTES:
                raise ProvenanceError(
                    f"aggregate byte budget exceeded at {total_bytes} "
                    f"(maximum {MAX_TOTAL_BYTES})"
                )
            _validate_digest(item.get("sha256"), label=f"entries[{index}].sha256")

    producer = value["producer"]
    if not isinstance(producer, dict) or set(producer) != {"id", "version", "files"}:
        raise ProvenanceError("producer has unknown or missing fields")
    if (
        producer["id"] != PRODUCER_ID
        or type(producer["version"]) is not int
        or producer["version"] != PRODUCER_VERSION
    ):
        raise ProvenanceError("unsupported producer identity")
    files = producer["files"]
    if not isinstance(files, list) or len(files) != len(PRODUCER_PATHS):
        raise ProvenanceError("producer.files must contain both helper files")
    actual_paths: list[str] = []
    for index, item in enumerate(files):
        if not isinstance(item, dict) or set(item) != {"path", "sha256"}:
            raise ProvenanceError(f"invalid producer.files[{index}]")
        path = _canonical_path(item["path"], label=f"producer.files[{index}].path")
        actual_paths.append(path)
        _validate_digest(item["sha256"], label=f"producer.files[{index}].sha256")
    if tuple(actual_paths) != PRODUCER_PATHS:
        raise ProvenanceError("producer files are not in canonical order")
    expected_files = _producer_files(entries)
    if files != expected_files:
        raise ProvenanceError("producer file digests do not match source inventory")
    if value["tree_digest"] != _tree_digest(entries, source):
        raise ProvenanceError("tree_digest does not match manifest payload")


def _manifest_identity(manifest: Mapping[str, Any]) -> tuple[str, str]:
    return str(manifest["snapshot_id"]), str(manifest["tree_digest"])


def _manifest_entries(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    return [dict(item) for item in manifest["entries"]]


def _compare_entries(
    expected: Sequence[Mapping[str, Any]], actual: Sequence[Mapping[str, Any]]
) -> None:
    expected_bytes = _canonical_json(list(expected))
    actual_bytes = _canonical_json(list(actual))
    if expected_bytes != actual_bytes:
        expected_by_path = {item["path"]: item for item in expected}
        actual_by_path = {item["path"]: item for item in actual}
        missing = sorted(set(expected_by_path) - set(actual_by_path))
        extra = sorted(set(actual_by_path) - set(expected_by_path))
        changed = sorted(
            path
            for path in set(expected_by_path) & set(actual_by_path)
            if _canonical_json(expected_by_path[path])
            != _canonical_json(actual_by_path[path])
        )
        details: list[str] = []
        if missing:
            details.append("missing=" + ",".join(missing[:5]))
        if extra:
            details.append("extra=" + ",".join(extra[:5]))
        if changed:
            details.append("changed=" + ",".join(changed[:5]))
        raise ProvenanceError(
            "source inventory mismatch"
            + (" (" + "; ".join(details) + ")" if details else "")
        )


def _manifest_path_for_root(root: Path) -> Path:
    return root / ".source-provenance.json"


# Reject symlinks before resolving paths to preserve source and output boundaries.
def _absolute_path(value: str, *, label: str) -> Path:
    if not value:
        raise ProvenanceError(f"{label} must not be empty")
    path = Path(value).absolute()
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current = current / component
        if os.path.lexists(current):
            try:
                metadata = os.lstat(current)
            except OSError as exc:
                raise ProvenanceError(
                    f"cannot inspect {label} {current}: {exc}"
                ) from exc
            if stat.S_ISLNK(metadata.st_mode):
                raise ProvenanceError(
                    f"symbolic links are not admitted in {label}: {current}"
                )
    return path


def _ensure_manifest_location(path: Path, source: Path) -> None:
    parent = path.parent
    _check_directory_path(parent, label="manifest parent")
    try:
        source_resolved = source.resolve()
        parent_resolved = parent.resolve()
    except OSError as exc:
        raise ProvenanceError(f"cannot resolve manifest location: {exc}") from exc
    try:
        path_resolved = path.resolve(strict=False)
        if os.path.commonpath(
            (os.fspath(path_resolved), os.fspath(source_resolved))
        ) == os.fspath(source_resolved) and path_resolved != _manifest_path_for_root(
            source
        ):
            raise ProvenanceError(
                "manifest inside source must use .source-provenance.json"
            )
        _ = parent_resolved
    except ValueError as exc:
        raise ProvenanceError(
            "manifest and source are on incompatible path roots"
        ) from exc


def _write_new_manifest(path: Path, payload: Mapping[str, Any]) -> None:
    encoded = _canonical_json(payload)
    if len(encoded) > MAX_MANIFEST_BYTES:
        raise ProvenanceError("generated manifest exceeds the 64 MiB bound")
    if os.path.lexists(path):
        raise ProvenanceError(f"refusing to overwrite existing manifest: {path}")
    try:
        descriptor = os.open(
            path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
            0o644,
        )
    except OSError as exc:
        raise ProvenanceError(f"cannot create manifest {path}: {exc}") from exc
    descriptor_identity: tuple[int, int] | None = None
    try:
        metadata = os.fstat(descriptor)
        descriptor_identity = (metadata.st_dev, metadata.st_ino)
        view = memoryview(encoded)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ProvenanceError(f"cannot write manifest {path}")
            view = view[written:]
        os.fsync(descriptor)
    except BaseException:
        os.close(descriptor)
        try:
            if descriptor_identity is None:
                raise FileNotFoundError
            current = os.lstat(path)
            if (current.st_dev, current.st_ino) == descriptor_identity:
                os.unlink(path)
        except OSError:
            pass
        raise
    else:
        os.close(descriptor)


def _capture_inventory_and_metadata(
    root: Path, *, require_git: bool, allow_dirty: bool
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata = _source_metadata(root, require_git=require_git)
    if metadata["worktree"] == "dirty" and not allow_dirty:
        raise ProvenanceError(
            "source Git worktree is dirty; pass --allow-dirty for local use"
        )
    entries = _walk_inventory(root)
    after_inventory = _source_metadata(root, require_git=require_git)
    if _canonical_json(metadata) != _canonical_json(after_inventory):
        raise ProvenanceError(
            "Git identity or worktree changed during provenance operation"
        )
    return metadata, entries


def _ensure_stable_source(
    root: Path,
    *,
    first_metadata: Mapping[str, Any],
    first_entries: Sequence[Mapping[str, Any]],
    require_git: bool,
    allow_dirty: bool,
) -> None:
    second_metadata, second_entries = _capture_inventory_and_metadata(
        root, require_git=require_git, allow_dirty=allow_dirty
    )
    if _canonical_json(dict(first_metadata)) != _canonical_json(second_metadata):
        raise ProvenanceError(
            "Git identity or worktree changed during provenance operation"
        )
    _compare_entries(first_entries, second_entries)


def _check_manifest_git_identity(
    root: Path, source: Mapping[str, Any], *, allow_dirty: bool
) -> dict[str, Any] | None:
    git_root = _git_path(root)
    if git_root is None:
        return None
    current = _source_metadata(root, require_git=True)
    if (
        current["root_commit"] != source["root_commit"]
        or current["root_tree"] != source["root_tree"]
    ):
        raise ProvenanceError("Git HEAD/tree does not match the manifest")
    if current["worktree"] != source["worktree"]:
        raise ProvenanceError("Git worktree state does not match the manifest")
    if current["worktree"] == "dirty" and not allow_dirty:
        raise ProvenanceError(
            "manifest records a dirty worktree; pass --allow-dirty for local use"
        )
    if _canonical_json(current["submodules"]) != _canonical_json(source["submodules"]):
        raise ProvenanceError("Git submodule identities do not match the manifest")
    return current


def _compare_git_metadata(
    expected: Mapping[str, Any] | None, actual: Mapping[str, Any] | None
) -> None:
    if expected is None or actual is None:
        if expected is not actual:
            raise ProvenanceError(
                "Git availability changed during provenance operation"
            )
        return
    if _canonical_json(dict(expected)) != _canonical_json(dict(actual)):
        raise ProvenanceError(
            "Git identity or worktree changed during provenance operation"
        )


def _verify_root(
    root: Path,
    manifest_path: Path,
    *,
    expected_snapshot_id: str | None,
    expected_tree_digest: str | None,
    allow_dirty: bool,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    _check_directory_path(root, label="verification root")
    manifest = _load_manifest(manifest_path)
    source = manifest["source"]
    if expected_snapshot_id is not None:
        _validate_uuid(expected_snapshot_id, label="expected-snapshot-id")
        if manifest["snapshot_id"] != expected_snapshot_id:
            raise ProvenanceError("snapshot_id does not match expected-snapshot-id")
    if expected_tree_digest is not None:
        _validate_digest(expected_tree_digest, label="expected-tree-digest")
        if manifest["tree_digest"] != expected_tree_digest:
            raise ProvenanceError("tree_digest does not match expected-tree-digest")
    if source["worktree"] == "dirty" and not allow_dirty:
        raise ProvenanceError(
            "manifest records a dirty worktree; pass --allow-dirty for local use"
        )
    before_git = _check_manifest_git_identity(root, source, allow_dirty=allow_dirty)
    entries = _walk_inventory(root)
    after_first_inventory_git = _check_manifest_git_identity(
        root, source, allow_dirty=allow_dirty
    )
    _compare_git_metadata(before_git, after_first_inventory_git)
    expected_entries = _manifest_entries(manifest)
    _compare_entries(expected_entries, entries)
    computed = _tree_digest(entries, source)
    if computed != manifest["tree_digest"]:
        raise ProvenanceError("tree digest does not match the complete inventory")
    by_path = {entry["path"]: entry for entry in entries}
    for producer in manifest["producer"]["files"]:
        entry = by_path.get(producer["path"])
        if (
            entry is None
            or entry.get("kind") != "file"
            or entry.get("sha256") != producer["sha256"]
        ):
            raise ProvenanceError(
                f"running producer file does not match the manifest: {producer['path']}"
            )
    second = _walk_inventory(root)
    after_second_inventory_git = _check_manifest_git_identity(
        root, source, allow_dirty=allow_dirty
    )
    _compare_git_metadata(after_first_inventory_git, after_second_inventory_git)
    _compare_entries(entries, second)
    return manifest, entries


def _make_marker(output: Path) -> tuple[Path, tuple[int, int], str]:
    marker = output / ".srb-source-provenance-owned"
    token = secrets.token_hex(24)
    content = f"srb-source-provenance-output-v1\n{token}\n".encode("ascii")
    descriptor = os.open(
        marker,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
        0o600,
    )
    metadata = None
    try:
        metadata = os.fstat(descriptor)
        offset = 0
        while offset < len(content):
            written = os.write(descriptor, content[offset:])
            if written <= 0:
                raise OSError("short write while creating snapshot ownership marker")
            offset += written
        to_close = descriptor
        descriptor = -1
        os.close(to_close)
        return marker, (metadata.st_dev, metadata.st_ino), token
    except BaseException:
        if descriptor >= 0:
            try:
                os.close(descriptor)
            except OSError:
                pass
        if metadata is not None:
            try:
                current = os.lstat(marker)
                if (current.st_dev, current.st_ino) == (
                    metadata.st_dev,
                    metadata.st_ino,
                ):
                    os.unlink(marker)
            except OSError:
                pass
        raise


def _remove_owned_marker(marker: Path, identity: tuple[int, int], token: str) -> None:
    try:
        metadata = os.lstat(marker)
        if (
            stat.S_ISLNK(metadata.st_mode)
            or (metadata.st_dev, metadata.st_ino) != identity
        ):
            raise ProvenanceError("snapshot ownership marker changed")
        content = _read_bounded_file(marker, 4096, label="snapshot ownership marker")
        if content != f"srb-source-provenance-output-v1\n{token}\n".encode("ascii"):
            raise ProvenanceError("snapshot ownership marker content changed")
        os.unlink(marker)
    except FileNotFoundError as exc:
        raise ProvenanceError("snapshot ownership marker disappeared") from exc


def _make_output_directory(output: Path) -> tuple[Path, tuple[int, int], str]:
    if os.path.lexists(output):
        raise ProvenanceError(
            f"refusing to overwrite existing snapshot destination: {output}"
        )
    parent = output.parent
    _check_directory_path(parent, label="snapshot destination parent")
    try:
        os.mkdir(output, 0o755)
    except OSError as exc:
        raise ProvenanceError(
            f"cannot create snapshot destination {output}: {exc}"
        ) from exc
    metadata = os.lstat(output)
    try:
        return _make_marker(output)
    except BaseException:
        try:
            current = os.lstat(output)
            if stat.S_ISDIR(current.st_mode) and (current.st_dev, current.st_ino) == (
                metadata.st_dev,
                metadata.st_ino,
            ):
                os.rmdir(output)
        except OSError:
            pass
        raise


def _ensure_not_nested(source: Path, output: Path) -> None:
    try:
        source_resolved = source.resolve()
        output_resolved = output.resolve(strict=False)
        source_value = os.fspath(source_resolved)
        output_value = os.fspath(output_resolved)
        if output_resolved == source_resolved:
            raise ProvenanceError("snapshot destination equals source")
        if os.path.commonpath((source_value, output_value)) == source_value:
            raise ProvenanceError("snapshot destination is inside source")
        if os.path.commonpath((source_value, output_value)) == output_value:
            raise ProvenanceError("source is inside snapshot destination")
    except ValueError as exc:
        raise ProvenanceError(
            "source and snapshot destination use incompatible roots"
        ) from exc


def _mkdir_output_path(path: Path, mode: int) -> tuple[int, int]:
    if os.path.lexists(path):
        metadata = os.lstat(path)
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
            raise ProvenanceError(f"snapshot path is not a directory: {path}")
        os.chmod(path, mode, follow_symlinks=False)
        return metadata.st_dev, metadata.st_ino
    parent = path.parent
    if parent != path:
        _check_directory_path(parent, label="snapshot destination parent")
    try:
        os.mkdir(path, mode)
        os.chmod(path, mode, follow_symlinks=False)
    except OSError as exc:
        raise ProvenanceError(
            f"cannot create snapshot directory {path}: {exc}"
        ) from exc
    metadata = os.lstat(path)
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
        raise ProvenanceError(f"snapshot path is not a directory: {path}")
    return metadata.st_dev, metadata.st_ino


def _set_snapshot_directory_mode(
    output: Path, directory: _SnapshotDirectory, mode: int | None
) -> None:
    parent_fd, name = _open_relative_parent(output, directory.relative_path)
    try:
        descriptor, before = _open_directory_at(
            parent_fd, name, label="snapshot directory"
        )
        try:
            identity = (before.st_dev, before.st_ino)
            if identity != directory.identity:
                raise ProvenanceError(
                    f"snapshot directory changed: {directory.relative_path}"
                )
            target_mode = stat.S_IMODE(before.st_mode) | 0o700 if mode is None else mode
            os.fchmod(descriptor, target_mode)
            after = os.fstat(descriptor)
            if (
                not stat.S_ISDIR(after.st_mode)
                or (after.st_dev, after.st_ino) != identity
                or stat.S_IMODE(after.st_mode) != target_mode
            ):
                raise ProvenanceError(
                    f"snapshot directory changed while setting mode: "
                    f"{directory.relative_path}"
                )
        finally:
            os.close(descriptor)
    finally:
        os.close(parent_fd)


def _restore_snapshot_directory_modes(
    output: Path, directories: Sequence[_SnapshotDirectory]
) -> None:
    for directory in sorted(
        directories,
        key=lambda item: (-item.relative_path.count("/"), item.relative_path),
    ):
        _set_snapshot_directory_mode(output, directory, directory.mode)


def _owned_marker_is_intact(
    marker: Path, identity: tuple[int, int], token: str
) -> bool:
    try:
        metadata = os.lstat(marker)
        if (
            stat.S_ISLNK(metadata.st_mode)
            or (metadata.st_dev, metadata.st_ino) != identity
        ):
            return False
        content = _read_bounded_file(marker, 4096, label="snapshot ownership marker")
        return content == f"srb-source-provenance-output-v1\n{token}\n".encode("ascii")
    except (OSError, ProvenanceError):
        return False


def _restore_owned_directory_writability(
    output: Path,
    marker: Path,
    marker_identity: tuple[int, int],
    marker_token: str,
    directories: Sequence[_SnapshotDirectory],
) -> None:
    if not _owned_marker_is_intact(marker, marker_identity, marker_token):
        return
    for directory in sorted(
        directories,
        key=lambda item: (item.relative_path.count("/"), item.relative_path),
    ):
        try:
            _set_snapshot_directory_mode(output, directory, None)
        except (OSError, ProvenanceError):
            continue


def _copy_regular_file(
    source: Path,
    destination: Path,
    expected: Mapping[str, Any],
    budget: _Budget,
    *,
    source_parent_fd: int | None = None,
    source_name: str | None = None,
) -> None:
    display_path = source / Path(*str(expected["path"]).split("/"))
    if source_parent_fd is not None and source_name is not None:
        descriptor, before = _open_regular_at(
            source_parent_fd, source_name, label="source file"
        )
    else:
        descriptor, before = _open_regular_for_read(display_path)
    digest = hashlib.sha256()
    copied = 0
    output_descriptor: int | None = None
    output_identity: tuple[int, int] | None = None
    try:
        if (
            before.st_size > MAX_FILE_BYTES
            or budget.total_bytes + before.st_size > MAX_TOTAL_BYTES
        ):
            raise ProvenanceError(f"snapshot byte budget exceeded: {display_path}")
        output_descriptor = os.open(
            destination,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
            int(expected["mode"]),
        )
        output_stat = os.fstat(output_descriptor)
        output_identity = (output_stat.st_dev, output_stat.st_ino)
        if not stat.S_ISREG(output_stat.st_mode) or _same_file_identity(
            before, output_stat
        ):
            raise ProvenanceError(
                f"snapshot destination is not an independent regular file: {destination}"
            )
        while True:
            chunk = os.read(descriptor, READ_CHUNK)
            if not chunk:
                break
            copied += len(chunk)
            if (
                copied > MAX_FILE_BYTES
                or budget.total_bytes + len(chunk) > MAX_TOTAL_BYTES
            ):
                raise ProvenanceError(
                    f"snapshot byte budget exceeded while copying: {display_path}"
                )
            view = memoryview(chunk)
            while view:
                written = os.write(output_descriptor, view)
                if written <= 0:
                    raise ProvenanceError(f"cannot write snapshot file: {destination}")
                view = view[written:]
            digest.update(chunk)
            budget.total_bytes += len(chunk)
        os.fchmod(output_descriptor, int(expected["mode"]))
        after_descriptor = os.fstat(descriptor)
        if source_parent_fd is not None and source_name is not None:
            after_path = _check_regular_at(
                source_parent_fd, source_name, label="source file"
            )
        else:
            after_path = _check_regular_path(display_path, label="source file")
        if not _same_file_snapshot(before, after_descriptor) or not _same_file_snapshot(
            before, after_path
        ):
            raise ProvenanceError(f"source file changed while copying: {display_path}")
        if copied != expected["size"] or digest.hexdigest() != expected["sha256"]:
            raise ProvenanceError(
                f"source file differs from admitted inventory: {display_path}"
            )
        output_stat = os.fstat(output_descriptor)
        if output_stat.st_size != copied or stat.S_IMODE(output_stat.st_mode) != int(
            expected["mode"]
        ):
            raise ProvenanceError(f"snapshot file differs after copy: {destination}")
    except BaseException:
        if output_descriptor is not None:
            try:
                os.close(output_descriptor)
            except OSError:
                pass
        if output_identity is not None:
            try:
                current = os.lstat(destination)
                if (current.st_dev, current.st_ino) == output_identity:
                    os.unlink(destination)
            except OSError:
                pass
        raise
    finally:
        os.close(descriptor)
        if output_descriptor is not None:
            try:
                os.close(output_descriptor)
            except OSError:
                pass


def _copy_snapshot_tree(
    source: Path,
    output: Path,
    entries: Sequence[Mapping[str, Any]],
    staged_directories: list[_SnapshotDirectory],
) -> None:
    directory_entries = sorted(
        (entry for entry in entries if entry["kind"] == "directory"),
        key=lambda item: (item["path"].count("/"), item["path"]),
    )
    for entry in directory_entries:
        destination = output / Path(*entry["path"].split("/"))
        identity = _mkdir_output_path(destination, 0o700)
        staged_directories.append(
            _SnapshotDirectory(
                relative_path=str(entry["path"]),
                identity=identity,
                mode=int(entry["mode"]),
            )
        )
    budget = _Budget()
    for entry in entries:
        if entry["kind"] != "file":
            continue
        destination = output / Path(*entry["path"].split("/"))
        parent = destination.parent
        _check_directory_path(parent, label="snapshot destination parent")
        source_parent_fd, source_name = _open_relative_parent(
            source, str(entry["path"])
        )
        try:
            _copy_regular_file(
                source,
                destination,
                entry,
                budget,
                source_parent_fd=source_parent_fd,
                source_name=source_name,
            )
        finally:
            os.close(source_parent_fd)


def _snapshot(
    source: Path,
    output: Path,
    requested_exclusions: Sequence[str],
    *,
    allow_dirty: bool,
) -> tuple[str, str]:
    _check_directory_path(source, label="source root")
    _ensure_not_nested(source, output)
    if requested_exclusions:
        for value in requested_exclusions:
            if value not in CANONICAL_EXCLUSIONS:
                raise ProvenanceError(f"caller exclusion is not canonical: {value}")
        if len(set(requested_exclusions)) != len(requested_exclusions):
            raise ProvenanceError("caller exclusions contain duplicates")
    metadata, entries = _capture_inventory_and_metadata(
        source, require_git=False, allow_dirty=allow_dirty
    )
    marker, marker_identity, marker_token = _make_output_directory(output)
    staged_directories: list[_SnapshotDirectory] = []
    try:
        _copy_snapshot_tree(source, output, entries, staged_directories)
        _restore_snapshot_directory_modes(output, staged_directories)
        output_entries = _walk_inventory(output)
        _compare_entries(entries, output_entries)
        _ensure_stable_source(
            source,
            first_metadata=metadata,
            first_entries=entries,
            require_git=False,
            allow_dirty=allow_dirty,
        )
        generated_source = dict(metadata)
        payload = _manifest_payload(
            snapshot_id=str(uuid.uuid4()),
            entries=output_entries,
            source=generated_source,
        )
        _write_new_manifest(_manifest_path_for_root(output), payload)
        _remove_owned_marker(marker, marker_identity, marker_token)
        return _manifest_identity(payload)
    except BaseException:
        _restore_owned_directory_writability(
            output,
            marker,
            marker_identity,
            marker_token,
            staged_directories,
        )
        raise


def _generate(
    source: Path, manifest_path: Path, *, allow_dirty: bool
) -> tuple[str, str]:
    _check_directory_path(source, label="source root")
    _ensure_manifest_location(manifest_path, source)
    metadata, entries = _capture_inventory_and_metadata(
        source, require_git=True, allow_dirty=allow_dirty
    )
    _ensure_stable_source(
        source,
        first_metadata=metadata,
        first_entries=entries,
        require_git=True,
        allow_dirty=allow_dirty,
    )
    snapshot_id = str(uuid.uuid4())
    payload = _manifest_payload(
        snapshot_id=snapshot_id, entries=entries, source=metadata
    )
    _write_new_manifest(manifest_path, payload)
    return _manifest_identity(payload)


def _result(command: str, identity: tuple[str, str]) -> dict[str, Any]:
    return {
        "ok": True,
        "command": command,
        "snapshot_id": identity[0],
        "tree_digest": identity[1],
    }


def _emit(value: Mapping[str, Any], *, as_json: bool) -> None:
    if as_json:
        sys.stdout.buffer.write(_canonical_json(dict(value)))
    else:
        sys.stdout.write(f"{value['tree_digest']}\n")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="bounded SRB source provenance helper")
    commands = parser.add_subparsers(dest="command", required=True)

    generate = commands.add_parser("generate")
    generate.add_argument("--source", required=True)
    generate.add_argument("--manifest", required=True)
    generate.add_argument("--allow-dirty", action="store_true")
    generate.add_argument("--json", action="store_true")

    verify = commands.add_parser("verify")
    verify.add_argument("--root", required=True)
    verify.add_argument("--manifest", required=True)
    verify.add_argument("--expected-snapshot-id")
    verify.add_argument("--expected-tree-digest")
    verify.add_argument("--allow-dirty", action="store_true")
    verify.add_argument("--json", action="store_true")

    snapshot = commands.add_parser("snapshot")
    snapshot.add_argument("--source", required=True)
    snapshot.add_argument("--output", required=True)
    snapshot.add_argument("--exclude", action="append", default=[])
    snapshot.add_argument("--allow-dirty", action="store_true")
    snapshot.add_argument("--json", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "generate":
            source = _absolute_path(arguments.source, label="source")
            manifest = _absolute_path(arguments.manifest, label="manifest")
            identity = _generate(source, manifest, allow_dirty=arguments.allow_dirty)
            _emit(_result("generate", identity), as_json=arguments.json)
        elif arguments.command == "verify":
            root = _absolute_path(arguments.root, label="root")
            manifest = _absolute_path(arguments.manifest, label="manifest")
            loaded, _ = _verify_root(
                root,
                manifest,
                expected_snapshot_id=arguments.expected_snapshot_id,
                expected_tree_digest=arguments.expected_tree_digest,
                allow_dirty=arguments.allow_dirty,
            )
            _emit(_result("verify", _manifest_identity(loaded)), as_json=arguments.json)
        elif arguments.command == "snapshot":
            source = _absolute_path(arguments.source, label="source")
            output = _absolute_path(arguments.output, label="output")
            identity = _snapshot(
                source,
                output,
                arguments.exclude,
                allow_dirty=arguments.allow_dirty,
            )
            _emit(_result("snapshot", identity), as_json=arguments.json)
        else:  # pragma: no cover - argparse enforces the command set.
            raise ProvenanceError("unknown command")
        return 0
    except (ProvenanceError, OSError, ValueError) as exc:
        print(f"source provenance: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
