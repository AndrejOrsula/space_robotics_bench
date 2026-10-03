"""Descriptor-anchored ownership and removal for SRB-managed directories.

This module depends only on the standard library and on the stdlib-only
failure taxonomy in :mod:`srb.utils.failures`. Host-facing cleanup and
installer code must prove that a directory is both safe and SRB-owned before
recursively changing it. Confirmation flags are never ownership evidence.
"""

from __future__ import annotations

import json
import os
import re
import secrets
import stat
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from srb.utils.failures import InvalidInputError

OWNERSHIP_MARKER_NAME = ".srb-owned-directory.json"
_MARKER_SCHEMA = "srb-owned-directory/1"
_MAX_MARKER_BYTES = 4096
_OWNER_PATTERN = re.compile(r"[a-z0-9][a-z0-9._-]{0,127}")


class OwnedDirectoryError(InvalidInputError, RuntimeError):
    """A directory failed ownership or safe-removal validation.

    It is an :class:`InvalidInputError` so CLI entry points render it through
    the failure taxonomy (code, component, remedy) instead of a raw traceback,
    and it stays a :class:`RuntimeError` for callers that catch it as one.
    """

    default_code = "invalid_input.owned_directory"


@dataclass(frozen=True)
class _InventoryEntry:
    parts: tuple[str, ...]
    device: int
    inode: int
    file_type: int
    is_directory: bool


@dataclass(frozen=True)
class _DirectoryInventory:
    entries: tuple[_InventoryEntry, ...]
    directory_identities: dict[tuple[str, ...], tuple[int, int]]


def _identity(value: os.stat_result) -> tuple[int, int]:
    return value.st_dev, value.st_ino


def _validate_owner(owner: str) -> None:
    if not isinstance(owner, str) or _OWNER_PATTERN.fullmatch(owner) is None:
        raise OwnedDirectoryError(
            "owned-directory owner must match [a-z0-9][a-z0-9._-]{0,127}"
        )


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _validate_root_path(
    root: Path,
    *,
    protected_roots: Iterable[Path],
    must_exist: bool,
) -> Path:
    if not root.is_absolute():
        raise OwnedDirectoryError(f"owned-directory path must be absolute: {root}")

    try:
        root_lstat = root.lstat()
    except FileNotFoundError as error:
        if must_exist:
            raise OwnedDirectoryError(
                f"owned-directory root does not exist: {root}"
            ) from error
        resolved = root.resolve(strict=False)
    else:
        if stat.S_ISLNK(root_lstat.st_mode):
            raise OwnedDirectoryError(
                f"owned-directory root must not be a symlink: {root}"
            )
        if not stat.S_ISDIR(root_lstat.st_mode):
            raise OwnedDirectoryError(
                f"owned-directory root must be a directory: {root}"
            )
        resolved = root.resolve(strict=True)

    filesystem_root = Path(resolved.anchor)
    if resolved == filesystem_root:
        raise OwnedDirectoryError(f"refusing to operate on filesystem root: {resolved}")

    protected = (Path.home(), *tuple(protected_roots))
    for candidate in protected:
        candidate_resolved = candidate.expanduser().resolve(strict=False)
        if resolved == candidate_resolved:
            raise OwnedDirectoryError(
                f"refusing to operate on protected root: {resolved}"
            )
        if _is_relative_to(candidate_resolved, resolved):
            raise OwnedDirectoryError(
                "refusing to operate on directory that contains protected root "
                f"{candidate_resolved}: {resolved}"
            )

    if resolved.exists() and os.path.ismount(resolved):
        raise OwnedDirectoryError(f"refusing to operate on mount root: {resolved}")

    # Two named components below `/` is the smallest accepted target. This
    # rejects broad roots such as `/tmp`, `/var`, and `/home` even when they do
    # not currently contain one of the explicitly protected paths.
    if len(resolved.parts) < 3:
        raise OwnedDirectoryError(
            f"refusing to operate on too shallow directory: {resolved}"
        )
    return resolved


def _directory_open_flags() -> int:
    no_follow = getattr(os, "O_NOFOLLOW", None)
    directory = getattr(os, "O_DIRECTORY", None)
    if no_follow is None or directory is None:
        raise OwnedDirectoryError(
            "descriptor-anchored owned-directory operations are unsupported "
            "on this platform"
        )
    return os.O_RDONLY | directory | no_follow | getattr(os, "O_CLOEXEC", 0)


def _open_validated_root(root: Path, resolved: Path) -> int:
    try:
        before = root.lstat()
        descriptor = os.open(root, _directory_open_flags())
    except OSError as exc:
        raise OwnedDirectoryError(
            f"could not open owned-directory root without following links: {root}"
        ) from exc
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISDIR(opened.st_mode) or _identity(opened) != _identity(before):
            raise OwnedDirectoryError(
                f"owned-directory root changed while opening: {root}"
            )
        try:
            current = resolved.stat()
        except OSError as exc:
            raise OwnedDirectoryError(
                f"owned-directory root changed while opening: {root}"
            ) from exc
        if _identity(current) != _identity(opened):
            raise OwnedDirectoryError(
                f"owned-directory root changed while opening: {root}"
            )
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _read_marker_at(root_fd: int) -> tuple[dict[str, object], os.stat_result]:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(OWNERSHIP_MARKER_NAME, flags, dir_fd=root_fd)
    except FileNotFoundError as exc:
        raise OwnedDirectoryError("owned-directory marker is missing") from exc
    except OSError as exc:
        raise OwnedDirectoryError(
            "owned-directory marker is not a readable non-symlink file"
        ) from exc
    try:
        marker_stat = os.fstat(descriptor)
        if not stat.S_ISREG(marker_stat.st_mode):
            raise OwnedDirectoryError(
                "owned-directory marker must be a regular non-symlink file"
            )
        if marker_stat.st_size > _MAX_MARKER_BYTES:
            raise OwnedDirectoryError("owned-directory marker exceeds size limit")
        payload = bytearray()
        while len(payload) <= _MAX_MARKER_BYTES:
            chunk = os.read(descriptor, min(1024, _MAX_MARKER_BYTES + 1 - len(payload)))
            if not chunk:
                break
            payload.extend(chunk)
        if len(payload) > _MAX_MARKER_BYTES:
            raise OwnedDirectoryError("owned-directory marker exceeds size limit")
    finally:
        os.close(descriptor)
    try:
        decoded = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise OwnedDirectoryError("owned-directory marker is invalid JSON") from exc
    if type(decoded) is not dict:
        raise OwnedDirectoryError("owned-directory marker must be a JSON object")
    return decoded, marker_stat


def _validate_marker_at(root_fd: int, *, resolved: Path, owner: str) -> None:
    marker, _ = _read_marker_at(root_fd)
    expected_fields = {"schema", "owner", "path", "device", "inode"}
    if set(marker) != expected_fields or marker.get("schema") != _MARKER_SCHEMA:
        raise OwnedDirectoryError("owned-directory marker schema is invalid")
    if marker.get("owner") != owner:
        raise OwnedDirectoryError(
            "owned-directory marker owner mismatch: "
            f"expected {owner!r}, found {marker.get('owner')!r}"
        )
    root_stat = os.fstat(root_fd)
    # Report each differing identity field; a copied marker must not authorize
    # another path, even when its device and inode happen to match.
    differences: list[str] = []
    if marker.get("path") != str(resolved):
        differences.append(f"path {marker.get('path')!r} != {str(resolved)!r}")
    if type(marker.get("device")) is not int or marker["device"] != root_stat.st_dev:
        differences.append(f"device {marker.get('device')!r} != {root_stat.st_dev!r}")
    if type(marker.get("inode")) is not int or marker["inode"] != root_stat.st_ino:
        differences.append(f"inode {marker.get('inode')!r} != {root_stat.st_ino!r}")
    if differences:
        raise OwnedDirectoryError(
            f"owned-directory marker identity mismatch for {resolved}: "
            + "; ".join(differences)
        )


def _write_marker_at(root_fd: int, *, resolved: Path, owner: str) -> None:
    root_stat = os.fstat(root_fd)
    payload = json.dumps(
        {
            "schema": _MARKER_SCHEMA,
            "owner": owner,
            "path": str(resolved),
            "device": root_stat.st_dev,
            "inode": root_stat.st_ino,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        descriptor = os.open(
            OWNERSHIP_MARKER_NAME,
            flags,
            0o600,
            dir_fd=root_fd,
        )
    except FileExistsError:
        _validate_marker_at(root_fd, resolved=resolved, owner=owner)
        return
    except OSError as exc:
        raise OwnedDirectoryError("could not create owned-directory marker") from exc
    try:
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise OwnedDirectoryError("could not write owned-directory marker")
            written += count
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    os.fsync(root_fd)


def claim_owned_directory(
    root: Path,
    *,
    owner: str,
    protected_roots: Iterable[Path] = (),
) -> Path:
    """Create and mark a new/empty directory, or validate an existing claim.

    A populated unmarked directory is never adopted. The returned path is the
    canonical path stored in the ownership marker.
    """

    _validate_owner(owner)
    root = Path(root)
    resolved_candidate = _validate_root_path(
        root,
        protected_roots=protected_roots,
        must_exist=False,
    )
    try:
        root.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise OwnedDirectoryError(
            f"could not create owned-directory root: {root}"
        ) from exc
    resolved = _validate_root_path(
        root,
        protected_roots=protected_roots,
        must_exist=True,
    )
    if resolved != resolved_candidate:
        raise OwnedDirectoryError(
            f"owned-directory root changed while creating: {root}"
        )
    root_fd = _open_validated_root(root, resolved)
    try:
        names = os.listdir(root_fd)
        if OWNERSHIP_MARKER_NAME in names:
            _validate_marker_at(root_fd, resolved=resolved, owner=owner)
            return resolved
        if names:
            raise OwnedDirectoryError(
                f"refusing to claim populated unmarked directory: {resolved}"
            )
        _write_marker_at(root_fd, resolved=resolved, owner=owner)
        _validate_marker_at(root_fd, resolved=resolved, owner=owner)
    finally:
        os.close(root_fd)
    return resolved


def adopt_owned_directory(
    root: Path,
    *,
    owner: str,
    expected_root: Path,
    protected_roots: Iterable[Path] = (),
) -> Path:
    """Mark a populated directory created by the caller.

    ``expected_root`` must be the caller's independently fixed creation target;
    a second copy of a user-controlled path does not prove ownership.
    """

    _validate_owner(owner)
    root = Path(root)
    expected_root = Path(expected_root)
    resolved = _validate_root_path(
        root,
        protected_roots=protected_roots,
        must_exist=True,
    )
    expected_resolved = _validate_root_path(
        expected_root,
        protected_roots=protected_roots,
        must_exist=True,
    )
    if resolved != expected_resolved:
        raise OwnedDirectoryError(
            "owned-directory authorization root mismatch: "
            f"expected {expected_resolved}, found {resolved}"
        )
    root_fd = _open_validated_root(root, resolved)
    try:
        if OWNERSHIP_MARKER_NAME in os.listdir(root_fd):
            _validate_marker_at(root_fd, resolved=resolved, owner=owner)
        else:
            _write_marker_at(root_fd, resolved=resolved, owner=owner)
            _validate_marker_at(root_fd, resolved=resolved, owner=owner)
    finally:
        os.close(root_fd)
    return resolved


def _nested_mount_points(root: Path) -> frozenset[tuple[str, ...]]:
    """Return mount points strictly below ``root`` as relative path parts.

    A same-device bind mount keeps ``st_dev``, so the device check alone
    cannot see it; Linux lists every mount in ``/proc/self/mountinfo``.
    """
    try:
        with open(
            "/proc/self/mountinfo", encoding="utf-8", errors="surrogateescape"
        ) as stream:
            lines = stream.read().splitlines()
    except OSError as exc:
        if sys.platform != "linux":
            return frozenset()
        raise OwnedDirectoryError(
            "could not enumerate mount points before owned-directory removal"
        ) from exc
    mounts: set[tuple[str, ...]] = set()
    for line in lines:
        fields = line.split(" ")
        if len(fields) < 5:
            continue
        # The kernel octal-escapes space, tab, newline, and backslash.
        mount_point = Path(
            re.sub(r"\\([0-7]{3})", lambda m: chr(int(m.group(1), 8)), fields[4])
        )
        if mount_point != root and _is_relative_to(mount_point, root):
            mounts.add(mount_point.relative_to(root).parts)
    return frozenset(mounts)


def _inventory_directory(
    root_fd: int, *, root_device: int, resolved: Path
) -> _DirectoryInventory:
    nested_mounts = _nested_mount_points(resolved)
    entries: list[_InventoryEntry] = []
    directory_identities: dict[tuple[str, ...], tuple[int, int]] = {
        (): _identity(os.fstat(root_fd))
    }

    def visit(directory_fd: int, parent_parts: tuple[str, ...]) -> None:
        try:
            names = sorted(os.listdir(directory_fd))
        except OSError as exc:
            raise OwnedDirectoryError(
                "could not inventory owned directory before removal"
            ) from exc
        for name in names:
            if not parent_parts and name == OWNERSHIP_MARKER_NAME:
                continue
            try:
                entry_stat = os.stat(
                    name,
                    dir_fd=directory_fd,
                    follow_symlinks=False,
                )
            except OSError as exc:
                raise OwnedDirectoryError(
                    f"owned-directory entry changed during preflight: {name!r}"
                ) from exc
            parts = (*parent_parts, name)
            file_type = stat.S_IFMT(entry_stat.st_mode)
            is_directory = stat.S_ISDIR(entry_stat.st_mode)
            if parts in nested_mounts:
                raise OwnedDirectoryError(
                    "owned-directory preflight crossed a mount point: "
                    + "/".join(parts)
                )
            if is_directory:
                if entry_stat.st_dev != root_device:
                    raise OwnedDirectoryError(
                        "owned-directory preflight crossed a mount/device boundary: "
                        + "/".join(parts)
                    )
                try:
                    child_fd = os.open(
                        name,
                        _directory_open_flags(),
                        dir_fd=directory_fd,
                    )
                except OSError as exc:
                    raise OwnedDirectoryError(
                        "owned-directory entry changed during preflight: "
                        + "/".join(parts)
                    ) from exc
                try:
                    opened = os.fstat(child_fd)
                    if _identity(opened) != _identity(entry_stat):
                        raise OwnedDirectoryError(
                            "owned-directory entry changed during preflight: "
                            + "/".join(parts)
                        )
                    directory_identities[parts] = _identity(opened)
                    visit(child_fd, parts)
                finally:
                    os.close(child_fd)
            entries.append(
                _InventoryEntry(
                    parts=parts,
                    device=entry_stat.st_dev,
                    inode=entry_stat.st_ino,
                    file_type=file_type,
                    is_directory=is_directory,
                )
            )

    visit(root_fd, ())
    return _DirectoryInventory(
        entries=tuple(entries),
        directory_identities=directory_identities,
    )


def _open_relative_directory(
    root_fd: int,
    parts: tuple[str, ...],
    *,
    identities: dict[tuple[str, ...], tuple[int, int]],
) -> int:
    descriptor = os.dup(root_fd)
    traversed: tuple[str, ...] = ()
    try:
        for part in parts:
            next_fd = os.open(part, _directory_open_flags(), dir_fd=descriptor)
            os.close(descriptor)
            descriptor = next_fd
            traversed = (*traversed, part)
            if _identity(os.fstat(descriptor)) != identities[traversed]:
                raise OwnedDirectoryError(
                    "owned-directory entry changed after preflight: "
                    + "/".join(traversed)
                )
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _remove_inventory(root_fd: int, inventory: _DirectoryInventory) -> None:
    for entry in inventory.entries:
        parent_parts = entry.parts[:-1]
        name = entry.parts[-1]
        parent_fd = _open_relative_directory(
            root_fd,
            parent_parts,
            identities=inventory.directory_identities,
        )
        try:
            try:
                current = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
            except OSError as exc:
                raise OwnedDirectoryError(
                    "owned-directory entry changed after preflight: "
                    + "/".join(entry.parts)
                ) from exc
            if (
                current.st_dev != entry.device
                or current.st_ino != entry.inode
                or stat.S_IFMT(current.st_mode) != entry.file_type
            ):
                raise OwnedDirectoryError(
                    "owned-directory entry changed after preflight: "
                    + "/".join(entry.parts)
                )
            try:
                if entry.is_directory:
                    os.rmdir(name, dir_fd=parent_fd)
                else:
                    os.unlink(name, dir_fd=parent_fd)
            except OSError as exc:
                raise OwnedDirectoryError(
                    "could not remove validated owned-directory entry: "
                    + "/".join(entry.parts)
                ) from exc
        finally:
            os.close(parent_fd)


def clear_owned_directory(
    root: Path,
    *,
    owner: str,
    protected_roots: Iterable[Path] = (),
) -> None:
    """Clear one marker-owned directory without removing its root or marker."""

    _validate_owner(owner)
    root = Path(root)
    resolved = _validate_root_path(
        root,
        protected_roots=protected_roots,
        must_exist=True,
    )
    root_fd = _open_validated_root(root, resolved)
    try:
        _validate_marker_at(root_fd, resolved=resolved, owner=owner)
        root_stat = os.fstat(root_fd)
        inventory = _inventory_directory(
            root_fd, root_device=root_stat.st_dev, resolved=resolved
        )
        # Revalidate the authorization marker after the complete read-only
        # inventory and before the first mutation.
        _validate_marker_at(root_fd, resolved=resolved, owner=owner)
        _remove_inventory(root_fd, inventory)
        remaining = sorted(os.listdir(root_fd))
        if remaining != [OWNERSHIP_MARKER_NAME]:
            raise OwnedDirectoryError(
                "owned-directory contents changed during removal; untracked "
                f"entries remain: {remaining!r}"
            )
        os.fsync(root_fd)
    finally:
        os.close(root_fd)


def validate_owned_directory(
    root: Path,
    *,
    owner: str,
    protected_roots: Iterable[Path] = (),
) -> Path:
    """Run every check ``clear_owned_directory`` runs, without mutating anything.

    Callers that remove several directories in one command use this to reject
    the whole request before the first removal, so a bad marker on the last
    target cannot leave the earlier ones already emptied. The returned path
    is the canonical resolved root.
    """

    _validate_owner(owner)
    root = Path(root)
    resolved = _validate_root_path(
        root,
        protected_roots=protected_roots,
        must_exist=True,
    )
    root_fd = _open_validated_root(root, resolved)
    try:
        _validate_marker_at(root_fd, resolved=resolved, owner=owner)
        # Read-only inventory: rejects mount crossings like clear does.
        _inventory_directory(
            root_fd, root_device=os.fstat(root_fd).st_dev, resolved=resolved
        )
    finally:
        os.close(root_fd)
    return resolved


def _unique_tombstone_name(parent_fd: int, root_name: str) -> str:
    for _ in range(16):
        candidate = f".{root_name}.srb-remove-{secrets.token_hex(8)}"
        try:
            os.stat(candidate, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            return candidate
        except OSError as exc:
            raise OwnedDirectoryError(
                "could not validate owned-directory removal tombstone"
            ) from exc
    raise OwnedDirectoryError("could not allocate owned-directory removal tombstone")


def remove_owned_directory(
    root: Path,
    *,
    owner: str,
    protected_roots: Iterable[Path] = (),
) -> None:
    """Remove an entire marker-owned directory through anchored descriptors."""

    _validate_owner(owner)
    root = Path(root)
    resolved = _validate_root_path(
        root,
        protected_roots=protected_roots,
        must_exist=True,
    )
    root_fd = _open_validated_root(root, resolved)
    parent_fd: int | None = None
    tombstone_name: str | None = None
    marker_removed = False
    try:
        _validate_marker_at(root_fd, resolved=resolved, owner=owner)
        root_stat = os.fstat(root_fd)
        inventory = _inventory_directory(
            root_fd, root_device=root_stat.st_dev, resolved=resolved
        )
        _validate_marker_at(root_fd, resolved=resolved, owner=owner)

        try:
            parent_fd = os.open(resolved.parent, _directory_open_flags())
            current = os.stat(
                resolved.name,
                dir_fd=parent_fd,
                follow_symlinks=False,
            )
        except OSError as exc:
            raise OwnedDirectoryError(
                f"could not anchor owned-directory parent: {resolved.parent}"
            ) from exc
        if _identity(current) != _identity(root_stat) or not stat.S_ISDIR(
            current.st_mode
        ):
            raise OwnedDirectoryError(
                f"owned-directory root changed after preflight: {resolved}"
            )

        tombstone_name = _unique_tombstone_name(parent_fd, resolved.name)
        try:
            os.rename(
                resolved.name,
                tombstone_name,
                src_dir_fd=parent_fd,
                dst_dir_fd=parent_fd,
            )
            renamed = os.stat(
                tombstone_name,
                dir_fd=parent_fd,
                follow_symlinks=False,
            )
        except OSError as exc:
            raise OwnedDirectoryError(
                f"could not isolate owned-directory root for removal: {resolved}"
            ) from exc
        if _identity(renamed) != _identity(root_stat):
            raise OwnedDirectoryError(
                f"owned-directory tombstone identity mismatch: {resolved}"
            )

        _remove_inventory(root_fd, inventory)
        if sorted(os.listdir(root_fd)) != [OWNERSHIP_MARKER_NAME]:
            raise OwnedDirectoryError("owned-directory contents changed during removal")
        _, marker_stat = _read_marker_at(root_fd)
        current_marker = os.stat(
            OWNERSHIP_MARKER_NAME,
            dir_fd=root_fd,
            follow_symlinks=False,
        )
        if _identity(current_marker) != _identity(marker_stat):
            raise OwnedDirectoryError("owned-directory marker changed during removal")
        os.unlink(OWNERSHIP_MARKER_NAME, dir_fd=root_fd)
        marker_removed = True
        os.fsync(root_fd)
        os.rmdir(tombstone_name, dir_fd=parent_fd)
        tombstone_name = None
        os.fsync(parent_fd)
    except BaseException:
        if marker_removed:
            try:
                _write_marker_at(root_fd, resolved=resolved, owner=owner)
            except BaseException:
                pass
        if parent_fd is not None and tombstone_name is not None:
            try:
                os.stat(resolved.name, dir_fd=parent_fd, follow_symlinks=False)
            except FileNotFoundError:
                try:
                    os.rename(
                        tombstone_name,
                        resolved.name,
                        src_dir_fd=parent_fd,
                        dst_dir_fd=parent_fd,
                    )
                except OSError:
                    pass
            except OSError:
                pass
        raise
    finally:
        if parent_fd is not None:
            os.close(parent_fd)
        os.close(root_fd)
