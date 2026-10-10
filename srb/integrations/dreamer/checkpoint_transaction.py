"""Crash-consistent same-logdir Dreamer training checkpoints.

The v2 manifest is authoritative. Elements' native ``ckpt/latest`` file is
maintained only as a compatibility pointer and is repaired from the manifest.
"""

from __future__ import annotations

import errno
import fcntl
import hashlib
import itertools
import json
import math
import os
import re
import shutil
import stat
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path, PurePosixPath
from typing import Any, Iterator, Mapping

from srb.utils.digest import sha256_file
from srb.utils.publication import (
    PublicationError,
    PublicationResourceLimitError,
    open_directory_descriptor,
    read_regular_file_bytes,
)
from srb.utils.resource_budget import MAX_IN_MEMORY_FILE_BYTES

CHECKPOINT_GENERATION_MANIFEST = "ckpt_generation.json"
_SCHEMA = "srb.dreamer.checkpoint"
_SCHEMA_VERSION = 2
# Keep checkpoint admission aligned with the parent-side Dreamer JSON limits.
_MAX_DEMO_REPLAY_DEPTH = 32
_MAX_DEMO_REPLAY_ITEMS = 10_000
_MAX_DEMO_REPLAY_BYTES = 1_048_576
_MIN_DEMO_REPLAY_INT = -(2**63)
_MAX_DEMO_REPLAY_INT = 2**63 - 1
_GENERATION_RE = re.compile(r"[0-9a-f]{32}")
_HASH_RE = re.compile(r"[0-9a-f]{64}")
_AGENT_SHARD_RE = re.compile(r"agent-([0-9]{4})\.pkl")
_GENERATION_NAME_RE = re.compile(r"[0-9]{8}T[0-9]{6}F[0-9]{6}-[0-9a-f]{12}-[0-9]{12}")
_REPLAY_GENERATIONS = PurePosixPath("replay", "generations")
_REPLAY_SEED_PREFIX = ".srb-replay-seed-"
_OWNERSHIP_DIR = ".srb-checkpoint-ownership"
_MAX_RETENTION_VISITS = 64


def _normalize_demo_replay(
    value: Mapping[str, Any] | None,
) -> tuple[dict[str, Any] | None, str | None]:
    if value is None:
        return None, None
    if not isinstance(value, Mapping):
        raise TypeError("demo_replay must be a mapping")
    budget = _DemoJSONBudget()
    normalized = _copy_demo_json(
        value,
        label="demo_replay",
        depth=0,
        budget=budget,
    )
    if not isinstance(normalized, dict):  # pragma: no cover - helper invariant
        raise TypeError("demo_replay must be a JSON object")
    encoded = json.dumps(
        normalized,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )
    if len(encoded.encode("utf-8")) != budget.encoded_size:  # pragma: no cover
        raise AssertionError("demo_replay JSON size accounting mismatch")
    return normalized, encoded


class _DemoJSONBudget:
    def __init__(self) -> None:
        self.items = 0
        self.encoded_size = 0

    @property
    def remaining_items(self) -> int:
        return _MAX_DEMO_REPLAY_ITEMS - self.items

    @property
    def remaining_bytes(self) -> int:
        return _MAX_DEMO_REPLAY_BYTES - self.encoded_size

    def visit(self, depth: int, *, label: str) -> None:
        if depth > _MAX_DEMO_REPLAY_DEPTH:
            raise ValueError(f"{label} exceeds maximum depth {_MAX_DEMO_REPLAY_DEPTH}")
        self.items += 1
        if self.items > _MAX_DEMO_REPLAY_ITEMS:
            raise ValueError(
                f"{label} exceeds maximum item count {_MAX_DEMO_REPLAY_ITEMS}"
            )

    def visit_key(self, *, label: str) -> None:
        self.items += 1
        if self.items > _MAX_DEMO_REPLAY_ITEMS:
            raise ValueError(
                f"{label} exceeds maximum item count {_MAX_DEMO_REPLAY_ITEMS}"
            )

    def add_bytes(self, amount: int, *, label: str) -> None:
        self.encoded_size += amount
        if self.encoded_size > _MAX_DEMO_REPLAY_BYTES:
            raise ValueError(
                f"{label} exceeds maximum serialized size {_MAX_DEMO_REPLAY_BYTES}"
            )


def _copy_demo_json(
    value: Any,
    *,
    label: str,
    depth: int,
    budget: _DemoJSONBudget,
) -> Any:
    budget.visit(depth, label=label)
    value_type = type(value)
    if value is None:
        budget.add_bytes(4, label=label)
        return None
    if value_type is bool:
        budget.add_bytes(4 if value else 5, label=label)
        return value
    if value_type is int:
        if not _MIN_DEMO_REPLAY_INT <= value <= _MAX_DEMO_REPLAY_INT:
            raise ValueError(f"{label} contains an out-of-range integer")
        budget.add_bytes(len(str(value)), label=label)
        return value
    if value_type is float:
        if not math.isfinite(value):
            raise ValueError(f"{label} must contain only finite numbers")
        encoded = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        budget.add_bytes(len(encoded), label=label)
        return value
    if value_type is str:
        budget.add_bytes(
            _demo_json_string_size(value, budget, label=label), label=label
        )
        return value
    if value_type is list:
        if len(value) > budget.remaining_items:
            raise ValueError(
                f"{label} exceeds maximum item count {_MAX_DEMO_REPLAY_ITEMS}"
            )
        budget.add_bytes(1, label=label)
        normalized_list: list[Any] = []
        for index, item in enumerate(value):
            if index:
                budget.add_bytes(1, label=label)
            normalized_list.append(
                _copy_demo_json(
                    item,
                    label=label,
                    depth=depth + 1,
                    budget=budget,
                )
            )
        budget.add_bytes(1, label=label)
        return normalized_list
    if isinstance(value, Mapping):
        budget.add_bytes(1, label=label)
        entry_count = len(value)
        if entry_count < 0 or entry_count > budget.remaining_items // 2:
            raise ValueError(
                f"{label} exceeds maximum item count {_MAX_DEMO_REPLAY_ITEMS}"
            )
        normalized_mapping: dict[str, Any] = {}
        iterator = iter(value)
        for index in range(entry_count):
            if budget.items + 2 > _MAX_DEMO_REPLAY_ITEMS:
                raise ValueError(
                    f"{label} exceeds maximum item count {_MAX_DEMO_REPLAY_ITEMS}"
                )
            try:
                key = next(iterator)
            except StopIteration as exc:
                raise ValueError(f"{label} mapping changed during traversal") from exc
            if type(key) is not str:
                raise TypeError(
                    f"{label} keys must be strings, got {type(key).__name__}"
                )
            if index:
                budget.add_bytes(1, label=label)
            budget.visit_key(label=label)
            budget.add_bytes(
                _demo_json_string_size(key, budget, label=label), label=label
            )
            budget.add_bytes(1, label=label)
            normalized_mapping[key] = _copy_demo_json(
                value[key],
                label=label,
                depth=depth + 1,
                budget=budget,
            )
        if len(normalized_mapping) != entry_count:
            raise ValueError(f"{label} mapping changed during traversal")
        budget.add_bytes(1, label=label)
        return normalized_mapping
    raise TypeError(f"{label} must be a JSON value, got {value_type.__name__}")


def _demo_json_string_size(
    value: str,
    budget: _DemoJSONBudget,
    *,
    label: str,
) -> int:
    remaining = budget.remaining_bytes
    if len(value) + 2 > remaining:
        raise ValueError(
            f"{label} exceeds maximum serialized size {_MAX_DEMO_REPLAY_BYTES}"
        )
    size = 2
    for character in value:
        codepoint = ord(character)
        if codepoint < 0x20:
            size += 2 if codepoint in {8, 9, 10, 12, 13} else 6
        elif codepoint in {0x22, 0x5C}:
            size += 2
        elif codepoint < 0x7F:
            size += 1
        elif codepoint <= 0xFFFF:
            size += 6
        else:
            size += 12
        if size > remaining:
            raise ValueError(
                f"{label} exceeds maximum serialized size {_MAX_DEMO_REPLAY_BYTES}"
            )
    return size


@dataclass(frozen=True)
class ResumePlan:
    checkpoint_path: Path
    replay_directory: Path
    generation_id: str
    step: int
    replay_items: int
    manifest_sha256: str


@dataclass(frozen=True)
class CheckpointRecord:
    manifest_path: Path
    checkpoint_path: Path
    generation_id: str
    step: int


class DreamerCheckpointTransaction:
    """Own the commit and restore seam for Dreamer training state."""

    def __init__(
        self,
        logdir: str | Path,
        *,
        demo_replay: Mapping[str, Any] | None = None,
    ):
        self.logdir = Path(logdir).expanduser().resolve()
        self.checkpoint_dir = self.logdir / "ckpt"
        self.replay_dir = self.logdir / "replay"
        self.manifest_path = self.logdir / CHECKPOINT_GENERATION_MANIFEST
        normalized_demo, normalized_demo_json = _normalize_demo_replay(demo_replay)
        self._demo_replay = normalized_demo
        self._demo_replay_json = normalized_demo_json
        self._expected_generation: str | None = None
        self._prepared = False

    def prepare(
        self, expected_checkpoint: str | Path | None = None
    ) -> ResumePlan | None:
        """Validate committed state before replay construction/load."""
        with self._lock():
            entry = self._read_manifest()
            if entry is None:
                if expected_checkpoint is not None:
                    raise RuntimeError(
                        "Requested same-run Dreamer checkpoint has no authoritative "
                        "v2 manifest"
                    )
                if self._legacy_state_exists():
                    raise RuntimeError(
                        "Refusing same-logdir Dreamer training resume without an "
                        "authoritative v2 checkpoint manifest; migrate the legacy "
                        "run explicitly or use a new log directory"
                    )
                self._expected_generation = None
                self._prepared = True
                return None

            manifest, digest = entry
            if expected_checkpoint is not None:
                requested = Path(expected_checkpoint).expanduser().resolve()
                committed_rel = _safe_relative_path(
                    _require_mapping(manifest.get("checkpoint"), "checkpoint").get(
                        "path"
                    ),
                    "checkpoint.path",
                )
                committed = (self.logdir / committed_rel).resolve()
                if requested != committed:
                    raise RuntimeError(
                        "Requested Dreamer checkpoint is not the authoritative "
                        "same-run generation"
                    )
            self._check_demo_replay_binding(manifest)
            plan = self._validate_training_manifest(
                manifest,
                manifest_sha256=digest,
            )
            self._repair_latest(plan.checkpoint_path.name)
            self._expected_generation = plan.generation_id
            self._prepared = True
            return plan

    def restore(
        self, plan: ResumePlan, *, checkpoint: Any, step: Any, replay: Any
    ) -> None:
        """Restore exactly the generation authorized by ``plan``."""
        self._require_prepared()
        with self._lock():
            entry = self._read_manifest()
            if entry is None:
                raise RuntimeError("Committed Dreamer checkpoint manifest disappeared")
            manifest, digest = entry
            self._check_demo_replay_binding(manifest)
            if (
                self._manifest_identity(manifest) != plan.generation_id
                or digest != plan.manifest_sha256
            ):
                raise RuntimeError(
                    "Dreamer checkpoint generation changed between prepare and restore"
                )
            current = self._validate_training_manifest(
                manifest,
                manifest_sha256=digest,
            )
            replay_manifest = _require_mapping(manifest.get("replay"), "replay")
            replay_snapshot_rel = _safe_relative_path(
                replay_manifest.get("directory"), "replay.directory"
            )
            quarantine, owned_quarantine_files = self._seed_replay_work(
                self.logdir.joinpath(*replay_snapshot_rel.parts),
                _parse_records(replay_manifest.get("chunks"), label="replay.chunks"),
                current.generation_id,
            )
            checkpoint.load(path=str(current.checkpoint_path))
            if int(step) != current.step:
                raise RuntimeError(
                    "Restored Dreamer step does not match committed manifest: "
                    f"restored={int(step)}, committed={current.step}"
                )
            if len(replay) != current.replay_items:
                raise RuntimeError(
                    "Restored Dreamer replay size does not match committed manifest: "
                    f"restored={len(replay)}, committed={current.replay_items}"
                )
            if quarantine is not None:
                for name in owned_quarantine_files:
                    try:
                        (quarantine / name).unlink(missing_ok=True)
                    except OSError:
                        pass
                try:
                    quarantine.rmdir()
                except OSError:
                    pass
            self._repair_latest(current.checkpoint_path.name)
            self._expected_generation = current.generation_id

    def commit(
        self,
        *,
        checkpoint: Any,
        step: Any,
        replay: Any,
    ) -> CheckpointRecord:
        """Commit one native Elements generation through the v2 manifest."""
        self._require_prepared()
        with self._lock():
            self._check_parent_generation()
            self._require_checkpoint_directory()

            replay_directory = self._replay_directory(replay)
            if replay_directory is not None:
                if not bool(getattr(replay, "save_wait", False)):
                    raise RuntimeError(
                        "Transactional Dreamer replay persistence requires "
                        "save_wait=True"
                    )
                replay.save()
                _fsync_replay_files(replay, replay_directory)

            step_value = int(step)
            generation_id = uuid.uuid4().hex
            timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SF%f")
            generation_name = f"{timestamp}-{generation_id[:12]}-{step_value:012d}"
            checkpoint_staging = self.checkpoint_dir / f".srb-txn-{generation_id}"
            checkpoint_final = self.checkpoint_dir / generation_name
            replay_staging: Path | None = None
            replay_final: Path | None = None
            replay_members: list[dict[str, Any]] | None = None

            try:
                if replay_directory is not None:
                    replay_generations = self._require_replay_generations_directory()
                    replay_staging = replay_generations / (f".srb-txn-{generation_id}")
                    replay_final = replay_generations / generation_name
                    _copy_replay_snapshot(replay, replay_directory, replay_staging)
                    replay_members = _snapshot_replay(replay_staging)
                    _fsync_snapshot(replay_staging, replay_members)

                checkpoint.save(path=str(checkpoint_staging))
                checkpoint_members = _snapshot_checkpoint(checkpoint_staging)
                _fsync_snapshot(checkpoint_staging, checkpoint_members)

                if replay_staging is not None and replay_final is not None:
                    if replay_final.exists():
                        raise RuntimeError(
                            f"Dreamer replay generation already exists: {replay_final}"
                        )
                    os.rename(replay_staging, replay_final)
                    replay_staging = None
                    _fsync_directory(replay_final.parent)
                    _fsync_directory(self.replay_dir)

                if checkpoint_final.exists():
                    raise RuntimeError(
                        f"Dreamer checkpoint generation already exists: {checkpoint_final}"
                    )
                os.rename(checkpoint_staging, checkpoint_final)
                checkpoint_staging = None
                _fsync_directory(self.checkpoint_dir)
            except BaseException:
                _remove_path(checkpoint_staging)
                _remove_path(replay_staging)
                raise

            if replay_directory is None:
                state_scope = "weights_only"
                replay_payload = None
            else:
                assert replay_members is not None
                state_scope = "training_state"
                replay_payload = {
                    "format": "embodied-replay-chunk-v1",
                    "directory": (
                        f"{_REPLAY_GENERATIONS.as_posix()}/{generation_name}"
                    ),
                    "item_count": int(len(replay)),
                    "chunks": replay_members,
                }

            manifest = {
                "schema": _SCHEMA,
                "schema_version": _SCHEMA_VERSION,
                "generation_id": generation_id,
                "parent_generation_id": self._expected_generation,
                "framework": "dreamer-elements",
                "state_scope": state_scope,
                "step": step_value,
                "checkpoint": {
                    "format": "elements-directory-v1",
                    "path": f"ckpt/{generation_name}",
                    "members": checkpoint_members,
                },
                "replay": replay_payload,
            }
            if self._demo_replay is not None:
                manifest["demo_replay"] = self._demo_replay
            _atomic_write(
                self.manifest_path,
                (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode(),
            )
            # The manifest is the commit point: own the generation before the
            # compatibility pointer, which prepare()/restore() repair from the
            # manifest if this write fails.
            self._expected_generation = generation_id
            try:
                _atomic_write(
                    self.checkpoint_dir / "latest",
                    f"{generation_name}\n".encode(),
                )
            except (OSError, PublicationError):
                pass
            try:
                self._write_ownership_record(manifest, replay_final, replay_directory)
                self._prune_owned_generations(manifest)
            except (OSError, RuntimeError, TypeError, ValueError):
                # Retention is best-effort; the durable commit already succeeded.
                pass
            return CheckpointRecord(
                manifest_path=self.manifest_path,
                checkpoint_path=checkpoint_final,
                generation_id=generation_id,
                step=step_value,
            )

    def _require_prepared(self) -> None:
        if not self._prepared:
            raise RuntimeError("Dreamer checkpoint transaction was not prepared")

    @contextmanager
    def _lock(self) -> Iterator[None]:
        self.logdir.mkdir(parents=True, exist_ok=True)
        lock_path = self.logdir / ".srb-checkpoint.lock"
        flags = os.O_RDWR | os.O_CREAT
        flags |= getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(lock_path, flags, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)

    def _read_manifest(self) -> tuple[dict[str, Any], str] | None:
        raw = read_regular_file_bytes(
            self.manifest_path, max_bytes=MAX_IN_MEMORY_FILE_BYTES
        )
        if raw is None:
            return None
        try:
            parsed = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"Invalid Dreamer checkpoint manifest {self.manifest_path}: {exc}"
            ) from exc
        if not isinstance(parsed, dict):
            raise RuntimeError("Dreamer checkpoint manifest must be a JSON object")
        return parsed, hashlib.sha256(raw).hexdigest()

    def _manifest_identity(self, manifest: Mapping[str, Any]) -> str:
        if (
            manifest.get("schema") != _SCHEMA
            or manifest.get("schema_version") != _SCHEMA_VERSION
        ):
            raise RuntimeError(
                "Dreamer same-logdir training state requires checkpoint manifest v2"
            )
        generation_id = manifest.get("generation_id")
        if not isinstance(generation_id, str) or not _GENERATION_RE.fullmatch(
            generation_id
        ):
            raise RuntimeError("Dreamer checkpoint manifest has invalid generation_id")
        return generation_id

    def _check_parent_generation(self) -> None:
        entry = self._read_manifest()
        if entry is not None:
            self._check_demo_replay_binding(entry[0])
        actual = None if entry is None else self._manifest_identity(entry[0])
        if actual != self._expected_generation:
            raise RuntimeError(
                "Dreamer checkpoint compare-and-swap failed: "
                f"expected parent {self._expected_generation!r}, found {actual!r}"
            )

    def _write_ownership_record(
        self,
        manifest: Mapping[str, Any],
        replay_final: Path | None,
        replay_work_directory: Path | None,
    ) -> None:
        generation_id = self._manifest_identity(manifest)
        checkpoint = _require_mapping(manifest.get("checkpoint"), "checkpoint")
        checkpoint_rel = _safe_relative_path(checkpoint.get("path"), "checkpoint.path")
        checkpoint_path = self.logdir.joinpath(*checkpoint_rel.parts)
        replay_value = manifest.get("replay")
        replay_path: Path | None = None
        replay_members: list[dict[str, Any]] | None = None
        replay_work_inventory: list[dict[str, Any]] | None = None
        if replay_final is not None:
            replay = _require_mapping(replay_value, "replay")
            replay_rel = _safe_relative_path(
                replay.get("directory"), "replay.directory"
            )
            replay_path = self.logdir.joinpath(*replay_rel.parts)
            replay_members = list(replay.get("chunks", []))

        def identity(path: Path | None) -> dict[str, int] | None:
            if path is None:
                return None
            value = path.stat()
            return {"device": value.st_dev, "inode": value.st_ino}

        def inventory(path: Path, members: Any) -> list[dict[str, Any]]:
            result = []
            for member in members:
                name = str(member["path"])
                value = (path / name).stat()
                result.append(
                    {
                        "path": name,
                        "size_bytes": int(member["size_bytes"]),
                        "sha256": str(member["sha256"]),
                        "device": value.st_dev,
                        "inode": value.st_ino,
                    }
                )
            return result

        if replay_work_directory is not None and replay_members is not None:
            replay_work_inventory = inventory(replay_work_directory, replay_members)

        payload = {
            "schema": "srb.dreamer.checkpoint-ownership",
            "version": 1,
            "generation_id": generation_id,
            "parent_generation_id": manifest.get("parent_generation_id"),
            "manifest_sha256": hashlib.sha256(
                (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()
            ).hexdigest(),
            "roots": {
                "run": identity(self.logdir),
                "checkpoint_parent": identity(self.checkpoint_dir),
                "replay_parent": identity(
                    None if replay_path is None else replay_path.parent
                ),
            },
            "checkpoint": {
                "path": checkpoint_rel.as_posix(),
                "identity": identity(checkpoint_path),
                "inventory": inventory(checkpoint_path, checkpoint.get("members")),
            },
            "replay": None
            if replay_path is None
            else {
                "path": _safe_relative_path(
                    _require_mapping(replay_value, "replay").get("directory"),
                    "replay.directory",
                ).as_posix(),
                "identity": identity(replay_path),
                "inventory": inventory(replay_path, replay_members),
                "work_inventory": replay_work_inventory,
            },
        }
        root = self.logdir / _OWNERSHIP_DIR
        if os.path.lexists(root) and (root.is_symlink() or not root.is_dir()):
            raise RuntimeError("Dreamer ownership record path is not a real directory")
        root.mkdir(mode=0o700, exist_ok=True)
        _atomic_write(
            root / f"{generation_id}.json",
            (
                json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n"
            ).encode(),
        )

    def _prune_owned_generations(self, manifest: Mapping[str, Any]) -> None:
        root = self.logdir / _OWNERSHIP_DIR
        if not root.is_dir() or root.is_symlink():
            return
        keep = {self._manifest_identity(manifest)}
        current_checkpoint = _safe_relative_path(
            _require_mapping(manifest.get("checkpoint"), "checkpoint").get("path"),
            "checkpoint.path",
        ).name
        protected_names = {current_checkpoint}
        parent_id = manifest.get("parent_generation_id")
        if isinstance(parent_id, str) and _GENERATION_RE.fullmatch(parent_id):
            keep.add(parent_id)
            parent_record = root / f"{parent_id}.json"
            try:
                raw_parent = read_regular_file_bytes(
                    parent_record, max_bytes=MAX_IN_MEMORY_FILE_BYTES
                )
                if raw_parent is not None:
                    parsed_parent = json.loads(raw_parent)
                    parent_checkpoint = parsed_parent.get("checkpoint")
                    if isinstance(parent_checkpoint, dict):
                        protected_names.add(
                            _safe_relative_path(
                                parent_checkpoint.get("path"),
                                "ownership.checkpoint.path",
                            ).name
                        )
            except (OSError, RuntimeError, TypeError, ValueError, json.JSONDecodeError):
                pass
        try:
            entries = os.scandir(root)
        except OSError:
            return
        with entries:
            for entry in itertools.islice(entries, _MAX_RETENTION_VISITS):
                if entry.name.endswith(".json"):
                    self._prune_owned_record(root / entry.name, keep, protected_names)

    def _prune_owned_record(
        self, record_path: Path, keep: set[str], protected_names: set[str]
    ) -> None:
        try:
            if record_path.name[:-5] in keep:
                return
            raw = read_regular_file_bytes(
                record_path, max_bytes=MAX_IN_MEMORY_FILE_BYTES
            )
            if raw is None:
                return
            record = json.loads(raw)
            generation_id = record.get("generation_id")
            if (
                record.get("schema") != "srb.dreamer.checkpoint-ownership"
                or record.get("version") != 1
                or not isinstance(generation_id, str)
                or not _GENERATION_RE.fullmatch(generation_id)
                or record_path.name != f"{generation_id}.json"
                or generation_id in keep
            ):
                return
            roots = record.get("roots")
            if not isinstance(roots, dict) or not all(
                _matches_directory_identity(path, roots.get(key))
                for path, key in (
                    (self.logdir, "run"),
                    (self.checkpoint_dir, "checkpoint_parent"),
                )
            ):
                return
            checkpoint = record.get("checkpoint")
            if not isinstance(checkpoint, dict):
                return
            checkpoint_rel = _safe_relative_path(
                checkpoint.get("path"), "ownership.checkpoint.path"
            )
            if (
                len(checkpoint_rel.parts) != 2
                or checkpoint_rel.parts[0] != "ckpt"
                or checkpoint_rel.name in protected_names
            ):
                return
            replay = record.get("replay")
            replay_rel: PurePosixPath | None = None
            if replay is not None:
                if not isinstance(replay, dict):
                    return
                replay_rel = _safe_relative_path(
                    replay.get("path"), "ownership.replay.path"
                )
                if (
                    len(replay_rel.parts) != 3
                    or PurePosixPath(*replay_rel.parts[:2]) != _REPLAY_GENERATIONS
                    or replay_rel.name != checkpoint_rel.name
                    or replay_rel.name in protected_names
                    or not _matches_directory_identity(
                        self.logdir.joinpath(*replay_rel.parts).parent,
                        roots.get("replay_parent"),
                    )
                ):
                    return
            checkpoint_removed = _remove_owned_payload(
                self.checkpoint_dir,
                checkpoint_rel.name,
                checkpoint.get("identity"),
                checkpoint.get("inventory"),
                roots.get("checkpoint_parent"),
            )
            replay_removed = replay_rel is None
            if replay_rel is not None and checkpoint_removed:
                replay_removed = _remove_owned_payload(
                    self.logdir.joinpath(*replay_rel.parts[:-1]),
                    replay_rel.name,
                    replay.get("identity"),
                    replay.get("inventory"),
                    roots.get("replay_parent"),
                )
            if checkpoint_removed and replay_removed:
                _unlink_owned_record(record_path, raw)
        except (OSError, RuntimeError, TypeError, ValueError, json.JSONDecodeError):
            return

    def _check_demo_replay_binding(self, manifest: Mapping[str, Any]) -> None:
        if "demo_replay" not in manifest:
            recorded = None
        else:
            raw = manifest["demo_replay"]
            if raw is None:
                raise RuntimeError(
                    "Dreamer checkpoint manifest field demo_replay must be an object"
                )
            try:
                _normalized, recorded = _normalize_demo_replay(raw)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    "Dreamer checkpoint manifest field demo_replay is invalid"
                ) from exc

        if recorded == self._demo_replay_json:
            return
        if recorded is None:
            raise RuntimeError(
                "Dreamer checkpoint demo_replay binding is missing for this run"
            )
        if self._demo_replay_json is None:
            raise RuntimeError(
                "Dreamer checkpoint demo_replay binding is present for a no-demo run"
            )
        raise RuntimeError(
            "Dreamer checkpoint demo_replay binding does not match this run"
        )

    def _validate_training_manifest(
        self,
        manifest: Mapping[str, Any],
        *,
        manifest_sha256: str,
    ) -> ResumePlan:
        generation_id = self._manifest_identity(manifest)
        parent_generation_id = manifest.get("parent_generation_id")
        if parent_generation_id is not None and (
            not isinstance(parent_generation_id, str)
            or not _GENERATION_RE.fullmatch(parent_generation_id)
        ):
            raise RuntimeError(
                "Dreamer checkpoint manifest has invalid parent_generation_id"
            )
        if manifest.get("framework") != "dreamer-elements":
            raise RuntimeError("Dreamer checkpoint manifest has invalid framework")
        if manifest.get("state_scope") != "training_state":
            raise RuntimeError(
                "Dreamer checkpoint contains weights only and cannot resume training; "
                "set agent.replay.save=true agent.replay.save_wait=true for future "
                "resumable runs"
            )
        step = _require_nonnegative_int(manifest.get("step"), "step")

        checkpoint = _require_mapping(manifest.get("checkpoint"), "checkpoint")
        if checkpoint.get("format") != "elements-directory-v1":
            raise RuntimeError("Unsupported Dreamer checkpoint directory format")
        checkpoint_rel = _safe_relative_path(checkpoint.get("path"), "checkpoint.path")
        if len(checkpoint_rel.parts) != 2 or checkpoint_rel.parts[0] != "ckpt":
            raise RuntimeError("Dreamer checkpoint path must be ckpt/<generation>")
        if not checkpoint_rel.name.endswith(f"-{step:012d}"):
            raise RuntimeError(
                "Dreamer checkpoint path step suffix does not match manifest step"
            )
        checkpoint_path = self.logdir / checkpoint_rel
        checkpoint_records = _parse_records(
            checkpoint.get("members"), label="checkpoint.members"
        )
        _validate_checkpoint_member_names(set(checkpoint_records))
        _verify_snapshot(checkpoint_path, checkpoint_records)

        replay = _require_mapping(manifest.get("replay"), "replay")
        if replay.get("format") != "embodied-replay-chunk-v1":
            raise RuntimeError("Unsupported Dreamer replay format")
        replay_rel = _safe_relative_path(replay.get("directory"), "replay.directory")
        if (
            len(replay_rel.parts) != 3
            or PurePosixPath(*replay_rel.parts[:2]) != _REPLAY_GENERATIONS
            or replay_rel.name != checkpoint_rel.name
        ):
            raise RuntimeError(
                "Dreamer replay directory must be the checkpoint's immutable "
                "replay/generations/<generation> snapshot"
            )
        replay_items = _require_nonnegative_int(
            replay.get("item_count"), "replay.item_count"
        )
        replay_records = _parse_records(replay.get("chunks"), label="replay.chunks")
        if any(PurePosixPath(name).suffix != ".npz" for name in replay_records):
            raise RuntimeError("Dreamer replay manifest may contain only .npz chunks")
        replay_snapshot = self.logdir / replay_rel
        _verify_replay_snapshot(replay_snapshot, replay_records)
        return ResumePlan(
            checkpoint_path=checkpoint_path,
            replay_directory=self.replay_dir,
            generation_id=generation_id,
            step=step,
            replay_items=replay_items,
            manifest_sha256=manifest_sha256,
        )

    def _seed_replay_work(
        self,
        snapshot: Path,
        records: Mapping[str, tuple[int, str]],
        generation_id: str,
    ) -> tuple[Path | None, list[str]]:
        self._ensure_replay_work_directory()
        seed = self.replay_dir / f"{_REPLAY_SEED_PREFIX}{uuid.uuid4().hex}"
        seed.mkdir()
        quarantine: Path | None = None
        owned_quarantine_files: list[str] = []
        try:
            owned_work = _owned_replay_work_inventory(self.logdir / _OWNERSHIP_DIR)
            for name in sorted(records):
                _link_or_copy_file(snapshot / name, seed / name)
            _fsync_directory(seed)

            old_entries: list[Path] = []
            for path in sorted(self.replay_dir.iterdir(), key=lambda item: item.name):
                if path.name == "generations":
                    if path.is_symlink() or not path.is_dir():
                        raise RuntimeError(
                            "Dreamer replay generations path is not a real directory"
                        )
                    continue
                if path == seed:
                    continue
                if path.is_dir() and not path.is_symlink():
                    if not path.name.startswith(_REPLAY_SEED_PREFIX):
                        raise RuntimeError(
                            "Dreamer replay work directory contains an unexpected "
                            f"nested directory: {path}"
                        )
                else:
                    _require_regular_file(path)
                if path.is_file() and path.suffix == ".npz":
                    old_entries.append(path)

            if old_entries:
                quarantine_root = self.logdir / ".srb-replay-quarantine"
                quarantine_root.mkdir(parents=True, exist_ok=True)
                quarantine = quarantine_root / f"{generation_id}-{uuid.uuid4().hex}"
                quarantine.mkdir()
                for path in old_entries:
                    destination = _quarantine_destination(quarantine, path.name)
                    os.rename(path, destination)
                    identity = destination.stat()
                    expected_values = owned_work.get(path.name, set())
                    if any(
                        identity.st_size == size
                        and _sha256(destination) == digest
                        and (device is None or identity.st_dev == device)
                        and (inode is None or identity.st_ino == inode)
                        for size, digest, device, inode in expected_values
                    ):
                        owned_quarantine_files.append(destination.name)
                _fsync_directory(quarantine)
                _fsync_directory(quarantine.parent)

            for path in sorted(seed.iterdir(), key=lambda item: item.name):
                os.rename(path, self.replay_dir / path.name)
            seed.rmdir()
            _fsync_directory(self.replay_dir)
            return quarantine, owned_quarantine_files
        except BaseException:
            _remove_path(seed)
            raise

    def _repair_latest(self, generation_name: str) -> None:
        latest = self.checkpoint_dir / "latest"
        expected = generation_name.encode()
        try:
            raw = read_regular_file_bytes(latest, max_bytes=MAX_IN_MEMORY_FILE_BYTES)
        except PublicationResourceLimitError:
            raw = None
        except PublicationError:
            # After admission, replace an invalid provider leaf without
            # decoding it; the anchored writer still rejects redirected parents.
            if not latest.is_symlink():
                raise
            raw = None
        if raw is None or raw.strip() != expected:
            _atomic_write(latest, expected + b"\n")

    def _legacy_state_exists(self) -> bool:
        if self._legacy_agent_only_state_is_safe():
            return False
        if os.path.lexists(self.checkpoint_dir):
            return True
        if os.path.lexists(self.replay_dir):
            if self.replay_dir.is_symlink() or not self.replay_dir.is_dir():
                return True
            return any(path.name != "generations" for path in self.replay_dir.iterdir())
        return False

    def _legacy_agent_only_state_is_safe(self) -> bool:
        if os.path.lexists(self.replay_dir):
            if self.replay_dir.is_symlink() or not self.replay_dir.is_dir():
                return False
            for path in self.replay_dir.iterdir():
                if path.name != "generations":
                    return False
                if path.is_symlink() or not path.is_dir():
                    return False

        if not os.path.lexists(self.checkpoint_dir):
            return False
        if self.checkpoint_dir.is_symlink() or not self.checkpoint_dir.is_dir():
            return False

        children = list(self.checkpoint_dir.iterdir())
        if not children:
            return False
        for child in children:
            if child.name == "latest":
                if not _is_regular_file_or_false(child):
                    return False
                if not self._legacy_latest_is_agent_only(child):
                    return False
            elif child.is_symlink() or not child.is_dir():
                return False
            elif not _is_agent_only_checkpoint(child):
                return False
        return True

    def _legacy_latest_is_agent_only(self, latest: Path) -> bool:
        try:
            raw = read_regular_file_bytes(latest, max_bytes=MAX_IN_MEMORY_FILE_BYTES)
            if raw is None:
                return False
            name = raw.decode("utf-8").strip()
            relative = _safe_relative_path(name, "ckpt/latest")
            candidate = self.checkpoint_dir.joinpath(*relative.parts)
            if not candidate.resolve().is_relative_to(self.checkpoint_dir.resolve()):
                return False
        except (OSError, RuntimeError, UnicodeDecodeError):
            return False
        return _is_agent_only_checkpoint(candidate)

    def _require_checkpoint_directory(self) -> None:
        self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
        if not self.checkpoint_dir.is_dir() or self.checkpoint_dir.is_symlink():
            raise RuntimeError("Dreamer checkpoint path is not a real directory")

    def _ensure_replay_work_directory(self) -> None:
        if os.path.lexists(self.replay_dir):
            if self.replay_dir.is_symlink() or not self.replay_dir.is_dir():
                raise RuntimeError("Dreamer replay path is not a real directory")
            return
        self.replay_dir.mkdir(parents=True, exist_ok=True)
        _fsync_directory(self.replay_dir.parent)

    def _require_replay_generations_directory(self) -> Path:
        self._ensure_replay_work_directory()
        generations = self.replay_dir / "generations"
        if os.path.lexists(generations):
            if generations.is_symlink() or not generations.is_dir():
                raise RuntimeError(
                    "Dreamer replay generations path is not a real directory"
                )
            return generations
        generations.mkdir()
        _fsync_directory(self.replay_dir)
        return generations

    def _replay_directory(self, replay: Any) -> Path | None:
        raw = getattr(replay, "directory", None)
        if not raw:
            return None
        directory = Path(str(raw)).expanduser()
        if directory.is_symlink():
            raise RuntimeError("Dreamer replay path is not a real directory")
        expected = self.replay_dir
        if directory.resolve(strict=False) != expected.resolve(strict=False):
            raise RuntimeError(
                "Dreamer replay directory must be the run-local replay path: "
                f"expected={expected.resolve()}, actual={directory.resolve(strict=False)}"
            )
        self._ensure_replay_work_directory()
        return expected


@contextmanager
def checkpoint_read_lock(checkpoint_path: str | Path) -> Iterator[Path]:
    """Lock an SRB generation and yield its bound path; preserve external paths."""
    original_path = Path(checkpoint_path).expanduser()
    path = original_path.resolve(strict=False)
    checkpoint_dir = path.parent if path.name == "agent.pkl" else path
    if (
        checkpoint_dir.parent.name != "ckpt"
        or not _GENERATION_NAME_RE.fullmatch(checkpoint_dir.name)
        or not checkpoint_dir.is_dir()
        or checkpoint_dir.is_symlink()
        or not (checkpoint_dir / "done").is_file()
    ):
        yield original_path
        return
    run_dir = checkpoint_dir.parent.parent
    if not (run_dir / CHECKPOINT_GENERATION_MANIFEST).is_file():
        yield original_path
        return
    lock_path = run_dir / ".srb-checkpoint.lock"
    if lock_path.exists() and not stat.S_ISREG(lock_path.lstat().st_mode):
        raise RuntimeError("Dreamer checkpoint lock path is not a regular file")
    flags = os.O_RDWR | os.O_CREAT
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(lock_path, flags, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        yield path
    finally:
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


def _matches_directory_identity(path: Path, identity: Any) -> bool:
    if not isinstance(identity, Mapping) or not path.is_dir() or path.is_symlink():
        return False
    try:
        value = path.stat()
    except OSError:
        return False
    return (
        identity.get("device") == value.st_dev and identity.get("inode") == value.st_ino
    )


def _remove_owned_payload(
    parent: Path,
    name: str,
    identity: Any,
    inventory: Any,
    expected_parent: Any,
) -> bool:
    if not isinstance(identity, Mapping) or not isinstance(expected_parent, Mapping):
        raise RuntimeError("Dreamer ownership record lacks directory identity")
    if not isinstance(inventory, list):
        raise RuntimeError("Dreamer ownership record lacks member inventory")
    with open_directory_descriptor(parent) as parent_fd:
        parent_stat = os.fstat(parent_fd)
        if (parent_stat.st_dev, parent_stat.st_ino) != (
            expected_parent.get("device"),
            expected_parent.get("inode"),
        ):
            raise RuntimeError("Dreamer owned generation parent was replaced")
        try:
            before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            return True
        if not stat.S_ISDIR(before.st_mode) or (before.st_dev, before.st_ino) != (
            identity.get("device"),
            identity.get("inode"),
        ):
            raise RuntimeError("Dreamer owned generation directory was replaced")
        flags = (
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
        )
        directory_fd = os.open(name, flags, dir_fd=parent_fd)
        try:
            opened = os.fstat(directory_fd)
            if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
                raise RuntimeError("Dreamer owned generation directory changed")
            expected: dict[str, Mapping[str, Any]] = {}
            for row in inventory:
                if not isinstance(row, Mapping):
                    raise RuntimeError("Dreamer ownership member inventory is invalid")
                member_name = row.get("path")
                digest = row.get("sha256")
                if (
                    not isinstance(member_name, str)
                    or Path(member_name).name != member_name
                    or not isinstance(digest, str)
                    or not _HASH_RE.fullmatch(digest)
                    or member_name in expected
                ):
                    raise RuntimeError("Dreamer ownership member inventory is invalid")
                expected[member_name] = row
            actual_names: list[str] = []
            with os.scandir(directory_fd) as entries:
                for entry in entries:
                    if entry.name not in expected:
                        raise RuntimeError(
                            "Dreamer owned generation member set changed"
                        )
                    actual_names.append(entry.name)
                    if len(actual_names) > len(expected):
                        raise RuntimeError(
                            "Dreamer owned generation member set changed"
                        )
            for member_name in actual_names:
                row = expected[member_name]
                current = os.stat(
                    member_name, dir_fd=directory_fd, follow_symlinks=False
                )
                if (
                    not stat.S_ISREG(current.st_mode)
                    or current.st_size != row.get("size_bytes")
                    or current.st_dev != row.get("device")
                    or current.st_ino != row.get("inode")
                ):
                    raise RuntimeError("Dreamer owned generation member was replaced")
                member_fd = os.open(
                    member_name,
                    os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
                    dir_fd=directory_fd,
                )
                try:
                    opened_member = os.fstat(member_fd)
                    if (opened_member.st_dev, opened_member.st_ino) != (
                        current.st_dev,
                        current.st_ino,
                    ):
                        raise RuntimeError("Dreamer owned generation member changed")
                    digest = hashlib.sha256()
                    while chunk := os.read(member_fd, 1024 * 1024):
                        digest.update(chunk)
                    if digest.hexdigest() != row.get("sha256"):
                        raise RuntimeError("Dreamer owned generation member changed")
                finally:
                    os.close(member_fd)
            for member_name in actual_names:
                row = expected[member_name]
                current = os.stat(
                    member_name, dir_fd=directory_fd, follow_symlinks=False
                )
                if (current.st_dev, current.st_ino) != (
                    row.get("device"),
                    row.get("inode"),
                ):
                    raise RuntimeError("Dreamer owned generation member was replaced")
                os.unlink(member_name, dir_fd=directory_fd)
        finally:
            os.close(directory_fd)
        current_directory = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        if (current_directory.st_dev, current_directory.st_ino) != (
            identity.get("device"),
            identity.get("inode"),
        ):
            raise RuntimeError("Dreamer owned generation directory was replaced")
        os.rmdir(name, dir_fd=parent_fd)
        return True


def _unlink_owned_record(record_path: Path, admitted_bytes: bytes) -> None:
    """Remove the control record only while its anchored identity and bytes match."""
    parent = record_path.parent
    parent_before = parent.stat()
    record_before = record_path.lstat()
    if not stat.S_ISREG(record_before.st_mode):
        return
    with open_directory_descriptor(parent) as parent_fd:
        parent_opened = os.fstat(parent_fd)
        if (parent_opened.st_dev, parent_opened.st_ino) != (
            parent_before.st_dev,
            parent_before.st_ino,
        ):
            return
        try:
            current = os.stat(record_path.name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            return
        if not stat.S_ISREG(current.st_mode) or (current.st_dev, current.st_ino) != (
            record_before.st_dev,
            record_before.st_ino,
        ):
            return
        descriptor = os.open(
            record_path.name,
            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=parent_fd,
        )
        try:
            opened = os.fstat(descriptor)
            if (opened.st_dev, opened.st_ino) != (current.st_dev, current.st_ino):
                return
            data = bytearray()
            while chunk := os.read(descriptor, 64 * 1024):
                data.extend(chunk)
                if len(data) > MAX_IN_MEMORY_FILE_BYTES:
                    return
            if bytes(data) != admitted_bytes:
                return
        finally:
            os.close(descriptor)
        current = os.stat(record_path.name, dir_fd=parent_fd, follow_symlinks=False)
        if (current.st_dev, current.st_ino) == (
            record_before.st_dev,
            record_before.st_ino,
        ):
            os.unlink(record_path.name, dir_fd=parent_fd)


def _safe_relative_path(value: Any, label: str) -> PurePosixPath:
    if not isinstance(value, str) or not value or "\\" in value:
        raise RuntimeError(f"{label} must be a normalized relative POSIX path")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise RuntimeError(f"{label} must be a contained relative path")
    if path.as_posix() != value:
        raise RuntimeError(f"{label} must be normalized")
    return path


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RuntimeError(
            f"Dreamer checkpoint manifest field {label} must be an object"
        )
    return value


def _require_nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise RuntimeError(
            f"Dreamer checkpoint manifest field {label} must be a nonnegative integer"
        )
    return value


def _parse_records(value: Any, *, label: str) -> dict[str, tuple[int, str]]:
    if not isinstance(value, list):
        raise RuntimeError(f"Dreamer checkpoint manifest field {label} must be a list")
    records: dict[str, tuple[int, str]] = {}
    for raw in value:
        record = _require_mapping(raw, label)
        path = _safe_relative_path(record.get("path"), f"{label}.path")
        if len(path.parts) != 1:
            raise RuntimeError(f"{label} members must be immediate files")
        size = _require_nonnegative_int(record.get("size_bytes"), f"{label}.size")
        digest = record.get("sha256")
        if not isinstance(digest, str) or not _HASH_RE.fullmatch(digest):
            raise RuntimeError(f"{label} member has invalid sha256")
        if path.name in records:
            raise RuntimeError(f"{label} contains duplicate path {path.name!r}")
        records[path.name] = (size, digest)
    return records


def _owned_replay_work_inventory(
    ownership_dir: Path,
) -> dict[str, set[tuple[int, str, int, int]]]:
    inventory: dict[str, set[tuple[int, str, int, int]]] = {}
    if not ownership_dir.is_dir() or ownership_dir.is_symlink():
        return inventory
    try:
        entries = os.scandir(ownership_dir)
    except OSError:
        return inventory
    with entries:
        for entry in itertools.islice(entries, _MAX_RETENTION_VISITS):
            if not entry.name.endswith(".json"):
                continue
            try:
                raw = read_regular_file_bytes(
                    ownership_dir / entry.name,
                    max_bytes=MAX_IN_MEMORY_FILE_BYTES,
                )
                if raw is None:
                    continue
                record = json.loads(raw)
                if not isinstance(record, Mapping):
                    continue
                if (
                    record.get("schema") != "srb.dreamer.checkpoint-ownership"
                    or record.get("version") != 1
                ):
                    continue
                replay = record.get("replay")
                rows = (
                    replay.get("work_inventory") if isinstance(replay, dict) else None
                )
                if not isinstance(rows, list):
                    continue
                for row in rows:
                    if not isinstance(row, Mapping):
                        continue
                    name = row.get("path")
                    size = row.get("size_bytes")
                    digest = row.get("sha256")
                    device = row.get("device")
                    inode = row.get("inode")
                    if (
                        isinstance(name, str)
                        and Path(name).name == name
                        and name.endswith(".npz")
                        and isinstance(size, int)
                        and not isinstance(size, bool)
                        and isinstance(digest, str)
                        and _HASH_RE.fullmatch(digest)
                        and isinstance(device, int)
                        and isinstance(inode, int)
                    ):
                        inventory.setdefault(name, set()).add(
                            (size, digest, device, inode)
                        )
            except (OSError, RuntimeError, TypeError, ValueError, json.JSONDecodeError):
                continue
    return inventory


def _validate_checkpoint_member_names(names: set[str]) -> None:
    required = {"done", "step.pkl", "replay.pkl"}
    if not required.issubset(names):
        raise RuntimeError(
            f"Dreamer checkpoint is missing native members: {sorted(required - names)}"
        )
    has_single = "agent.pkl" in names
    shards = sorted(
        int(match.group(1))
        for name in names
        if (match := _AGENT_SHARD_RE.fullmatch(name)) is not None
    )
    if has_single == bool(shards):
        raise RuntimeError(
            "Dreamer checkpoint must contain either agent.pkl or agent shards"
        )
    if shards and shards != list(range(len(shards))):
        raise RuntimeError("Dreamer checkpoint agent shards must be contiguous")
    allowed = required | ({"agent.pkl"} if has_single else set())
    allowed.update(f"agent-{index:04d}.pkl" for index in shards)
    if names != allowed:
        raise RuntimeError(
            f"Dreamer checkpoint contains unknown members: {sorted(names - allowed)}"
        )


def _snapshot_checkpoint(directory: Path) -> list[dict[str, Any]]:
    if not directory.is_dir() or directory.is_symlink():
        raise RuntimeError("Elements did not create a real checkpoint directory")
    children = sorted(directory.iterdir(), key=lambda path: path.name)
    for child in children:
        _require_regular_file(child)
    names = {child.name for child in children}
    _validate_checkpoint_member_names(names)
    if (directory / "done").stat().st_size != 0:
        raise RuntimeError("Dreamer checkpoint done marker must be empty")
    return [_file_record(path) for path in children]


def _snapshot_replay(directory: Path) -> list[dict[str, Any]]:
    paths = sorted(directory.glob("*.npz"), key=lambda path: path.name)
    for path in paths:
        _require_regular_file(path)
    return [_file_record(path) for path in paths]


def _file_record(path: Path) -> dict[str, Any]:
    return {
        "path": path.name,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _verify_snapshot(directory: Path, records: Mapping[str, tuple[int, str]]) -> None:
    if not directory.is_dir() or directory.is_symlink():
        raise RuntimeError(
            f"Committed Dreamer checkpoint directory is missing: {directory}"
        )
    actual_paths = {path.name: path for path in directory.iterdir()}
    if set(actual_paths) != set(records):
        raise RuntimeError("Committed Dreamer checkpoint member set changed")
    for name, (size, digest) in records.items():
        path = actual_paths[name]
        _require_regular_file(path)
        if path.stat().st_size != size or _sha256(path) != digest:
            raise RuntimeError(f"Committed Dreamer checkpoint member changed: {name}")
    done = actual_paths.get("done")
    if done is None or done.stat().st_size != 0:
        raise RuntimeError("Committed Dreamer checkpoint done marker is invalid")


def _verify_replay_snapshot(
    directory: Path,
    records: Mapping[str, tuple[int, str]],
) -> None:
    if not directory.is_dir() or directory.is_symlink():
        raise RuntimeError("Committed Dreamer replay directory is missing")
    actual_paths = {path.name: path for path in directory.iterdir()}
    if set(actual_paths) != set(records):
        raise RuntimeError("Committed Dreamer replay chunk set changed")
    for name, (size, digest) in records.items():
        path = actual_paths[name]
        if path.suffix != ".npz":
            raise RuntimeError("Committed Dreamer replay may contain only .npz chunks")
        _require_regular_file(path)
        if path.stat().st_size != size or _sha256(path) != digest:
            raise RuntimeError(f"Committed Dreamer replay chunk changed: {name}")


def _is_regular_file_or_false(path: Path) -> bool:
    try:
        _require_regular_file(path)
    except RuntimeError:
        return False
    return True


def _is_agent_only_checkpoint(directory: Path) -> bool:
    try:
        if not directory.is_dir() or directory.is_symlink():
            return False
        children = sorted(directory.iterdir(), key=lambda path: path.name)
        names = {child.name for child in children}
        if "done" not in names:
            return False
        has_single = "agent.pkl" in names
        shards = sorted(
            int(match.group(1))
            for name in names
            if (match := _AGENT_SHARD_RE.fullmatch(name)) is not None
        )
        if has_single == bool(shards):
            return False
        if shards and shards != list(range(len(shards))):
            return False
        allowed = {"done"} | ({"agent.pkl"} if has_single else set())
        allowed.update(f"agent-{index:04d}.pkl" for index in shards)
        if names != allowed:
            return False
        for child in children:
            _require_regular_file(child)
        return (directory / "done").stat().st_size == 0
    except (OSError, RuntimeError):
        return False


def _require_regular_file(path: Path) -> None:
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"Required Dreamer checkpoint file is missing: {path}"
        ) from exc
    if not stat.S_ISREG(mode):
        raise RuntimeError(f"Dreamer checkpoint member is not a regular file: {path}")


def _sha256(path: Path) -> str:
    return sha256_file(path)


def _copy_replay_snapshot(replay: Any, source: Path, staging: Path) -> None:
    if os.path.lexists(staging):
        raise RuntimeError(f"Dreamer replay staging path already exists: {staging}")
    staging.mkdir()
    try:
        for path in _live_replay_paths(replay, source):
            _require_regular_file(path)
            _link_or_copy_file(path, staging / path.name)
        _fsync_directory(staging)
    except BaseException:
        _remove_path(staging)
        raise


def _live_replay_paths(replay: Any, directory: Path) -> list[Path]:
    chunks = getattr(replay, "chunks", None)
    if not isinstance(chunks, Mapping):
        raise RuntimeError(
            "Transactional Dreamer replay persistence requires the pinned "
            "live replay chunk inventory"
        )
    selected: dict[str, Any] = {}
    for chunk in chunks.values():
        if int(getattr(chunk, "length", 0)) > 0:
            filename = getattr(chunk, "filename", None)
            if not isinstance(filename, str) or Path(filename).name != filename:
                raise RuntimeError("Dreamer live replay chunk has invalid filename")
            if not filename.endswith(".npz"):
                raise RuntimeError("Dreamer live replay chunk must be an .npz file")
            selected[filename] = chunk
    pending = list(selected.values())
    seen: set[int] = set()
    while pending:
        chunk = pending.pop()
        if id(chunk) in seen:
            continue
        seen.add(id(chunk))
        successor = getattr(chunk, "succ", None)
        successor_id = getattr(successor, "uuid", successor)
        next_chunk = chunks.get(successor_id)
        if next_chunk is not None and int(getattr(next_chunk, "length", 0)) > 0:
            filename = getattr(next_chunk, "filename", None)
            if not isinstance(filename, str) or Path(filename).name != filename:
                raise RuntimeError("Dreamer live replay successor has invalid filename")
            if filename not in selected:
                selected[filename] = next_chunk
                pending.append(next_chunk)
    return [directory / name for name in sorted(selected)]


def _copy_file(source: Path, destination: Path) -> None:
    _require_regular_file(source)
    if os.path.lexists(destination):
        raise RuntimeError(f"Dreamer copy destination already exists: {destination}")
    try:
        with source.open("rb") as input_stream, destination.open("xb") as output_stream:
            shutil.copyfileobj(input_stream, output_stream, length=1024 * 1024)
            output_stream.flush()
            os.fsync(output_stream.fileno())
    except BaseException:
        _remove_path(destination)
        raise


def _link_or_copy_file(source: Path, destination: Path) -> None:
    _require_regular_file(source)
    if os.path.lexists(destination):
        raise RuntimeError(f"Dreamer copy destination already exists: {destination}")
    try:
        os.link(source, destination)
    except OSError as exc:
        if exc.errno not in {
            errno.EXDEV,
            errno.EOPNOTSUPP,
            errno.ENOTSUP,
            errno.ENOSYS,
        }:
            raise
        _copy_file(source, destination)


def _quarantine_destination(directory: Path, name: str) -> Path:
    destination = directory / name
    if not os.path.lexists(destination):
        return destination
    return directory / f"{uuid.uuid4().hex}-{name}"


def _remove_path(path: Path | None) -> None:
    if path is None or not os.path.lexists(path):
        return
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path, ignore_errors=True)
    else:
        path.unlink(missing_ok=True)


def _fsync_snapshot(directory: Path, records: list[dict[str, Any]]) -> None:
    for record in records:
        _fsync_file(directory / str(record["path"]))
    _fsync_directory(directory)


def _fsync_replay_files(replay: Any, directory: Path) -> None:
    for path in _live_replay_paths(replay, directory):
        _require_regular_file(path)
        _fsync_file(path)
    _fsync_directory(directory)


def _fsync_file(path: Path) -> None:
    with path.open("rb") as stream:
        os.fsync(stream.fileno())


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write(path: Path, payload: bytes) -> None:
    if len(payload) > MAX_IN_MEMORY_FILE_BYTES:
        raise PublicationResourceLimitError(
            path, limit=MAX_IN_MEMORY_FILE_BYTES, observed=len(payload)
        )
    temporary = f".{path.name}.{uuid.uuid4().hex}.tmp"
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    with open_directory_descriptor(path.parent, create=True) as parent_fd:
        descriptor = os.open(temporary, flags, 0o600, dir_fd=parent_fd)
        try:
            try:
                remaining = memoryview(payload)
                while remaining:
                    written = os.write(descriptor, remaining)
                    if written <= 0:
                        raise OSError("Checkpoint control-file write made no progress")
                    remaining = remaining[written:]
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            os.replace(temporary, path.name, src_dir_fd=parent_fd, dst_dir_fd=parent_fd)
            os.fsync(parent_fd)
        except BaseException:
            try:
                os.unlink(temporary, dir_fd=parent_fd)
            except FileNotFoundError:
                pass
            raise
