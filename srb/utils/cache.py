from collections.abc import Callable, Mapping, Sequence
from contextvars import ContextVar
from dataclasses import dataclass, field
from pathlib import Path
from typing import cast

import orjson as json

from srb.utils.failures import IntegrityFailureError
from srb.utils.path import (
    SRB_ACTION_MODES_CACHE_PATH,
    SRB_AGENT_CONFIG_KEYS_CACHE_PATH,
    SRB_AGENT_ENUM_VALUES_CACHE_PATH,
    SRB_CACHE_PATH,
    SRB_CONFIG_KEYS_CACHE_PATH,
    SRB_ENUM_VALUES_CACHE_PATH,
    SRB_ENV_CACHE_PATH,
    SRB_GUI_CATALOG_CACHE_PATH,
    SRB_HARDWARE_INTERFACE_CACHE_PATH,
    SRB_OBJECT_CACHE_PATH,
    SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH,
    SRB_ROBOT_CACHE_PATH,
    SRB_SCENERY_CACHE_PATH,
    SRB_TASK_DEFAULTS_CACHE_PATH,
)
from srb.utils.publication import (
    ArtifactPublication,
    FilePublication,
    PublicationError,
    PublicationNotFoundError,
    PublishedGeneration,
    sha256_file,
)

_SRB_CACHE_META_KEY = "__srb_cache__"
_SRB_CACHE_PAYLOAD_KEY = "payload"
_SRB_CACHE_SCHEMA_VERSION = 1
_SRB_CACHE_GENERATION_SCHEMA_VERSION = 1

_CACHE_BUILD_ROOT: ContextVar[Path | None] = ContextVar(
    "srb_offline_cache_build_root", default=None
)
_DEFAULT_CACHE_PATHS: dict[str, Path] = {}

#: Failures diagnostic and boolean APIs report as an unavailable generation.
#: Reader APIs keep raising the typed integrity error.
_DIAGNOSTIC_FAILURES = (
    OSError,
    PublicationError,
    IntegrityFailureError,
    TypeError,
    ValueError,
)


def _json_loads(data: bytes) -> object:
    return json.loads(data)


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    path = _cache_path_for_context(path)
    build_root = _CACHE_BUILD_ROOT.get()
    if build_root is not None and path.parent == build_root:
        # Sidecars become generation members; write staged payloads directly so
        # the generation publisher alone switches the pointer.
        path.write_bytes(payload)
        return
    FilePublication(path).publish_bytes(payload)


def _wrap_cache_payload(payload: object) -> dict[str, object]:
    metadata: dict[str, object] = {"version": _SRB_CACHE_SCHEMA_VERSION}
    if _CACHE_BUILD_ROOT.get() is not None:
        fingerprint = registry_source_fingerprint()
        if fingerprint is not None:
            metadata["registry_source_sha256"] = fingerprint
    return {
        _SRB_CACHE_META_KEY: metadata,
        _SRB_CACHE_PAYLOAD_KEY: payload,
    }


def _unwrap_cache_payload(raw: object) -> object:
    if (
        isinstance(raw, dict)
        and _SRB_CACHE_META_KEY in raw
        and _SRB_CACHE_PAYLOAD_KEY in raw
    ):
        return raw.get(_SRB_CACHE_PAYLOAD_KEY)
    return raw


def _cache_schema_version(raw: object) -> int | None:
    if isinstance(raw, dict) and _SRB_CACHE_META_KEY in raw:
        meta = raw.get(_SRB_CACHE_META_KEY)
        if isinstance(meta, dict):
            version = meta.get("version")
            if isinstance(version, int):
                return version
    return None


def _cache_integrity_failure(path: Path, exc: BaseException) -> IntegrityFailureError:
    """Describe cache corruption without collapsing it into a cache miss."""

    return IntegrityFailureError(
        code="srb.cache.integrity",
        message=f"offline cache {path} failed integrity validation: {exc}",
        component="srb.utils.cache",
        details={
            "path": str(path),
            "cause": str(exc),
            "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
        },
        fallback_allowed=False,
        evidence_valid=False,
    )


@dataclass(frozen=True)
class OfflineCacheArtifact:
    name: str
    path: Path


@dataclass(frozen=True)
class OfflineCacheSnapshot:
    """One validated immutable offline-cache generation.

    Resolve its pointer once so concurrent publication cannot mix member generations.
    Member hashes are validated on each read; ``member_path`` only resolves a name.
    """

    generation_id: str
    path: Path
    member_hashes: Mapping[str, str]
    _published: PublishedGeneration | None = field(
        default=None,
        repr=False,
        compare=False,
    )

    def __enter__(self) -> "OfflineCacheSnapshot":
        if self._published is not None:
            self._published.__enter__()
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: object | None,
    ) -> None:
        self.close()

    def close(self) -> None:
        if self._published is not None:
            self._published.close()

    def __del__(self) -> None:  # pragma: no cover - interpreter cleanup
        try:
            self.close()
        except Exception:
            pass

    def member_path(self, name: str) -> Path:
        member = _cache_member_name(name)
        if member not in self.member_hashes:
            raise PublicationError(f"offline cache member is not published: {name}")
        return self.path / member

    def read_bytes(self, name: str) -> bytes:
        member = _cache_member_name(name)
        published = self._published or PublishedGeneration(
            generation_id=self.generation_id,
            path=self.path,
            member_hashes=self.member_hashes,
        )
        try:
            return published.read_member_bytes(member)
        except IntegrityFailureError:
            raise
        except (OSError, PublicationError) as exc:
            raise _cache_integrity_failure(self.path / member, exc) from exc

    def read[T](
        self,
        name: str,
        parser: Callable[[object], T | None],
    ) -> T | None:
        try:
            payload = self.read_bytes(name)
            raw = _json_loads(payload)
            result = parser(_unwrap_cache_payload(raw))
            if result is None:
                raise ValueError(f"parser rejected offline cache member {name}")
            return result
        except IntegrityFailureError:
            raise
        except Exception as exc:  # noqa: BLE001 - normalize public cache reads
            raise _cache_integrity_failure(self.member_path(name), exc) from exc


@dataclass(frozen=True)
class OfflineCacheEntryStatus:
    name: str
    path: Path
    exists: bool
    is_file: bool
    valid_json: bool | None
    size_bytes: int | None
    modified_time: float | None

    @property
    def state(self) -> str:
        if not self.exists:
            return "missing"
        if not self.is_file or self.valid_json is False:
            return "invalid"
        return "ok"

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "path": str(self.path),
            "state": self.state,
            "exists": self.exists,
            "is_file": self.is_file,
            "valid_json": self.valid_json,
            "size_bytes": self.size_bytes,
            "modified_time": self.modified_time,
        }


@dataclass(frozen=True)
class OfflineCacheStatus:
    root: Path
    entries: tuple[OfflineCacheEntryStatus, ...]

    @property
    def total_count(self) -> int:
        return len(self.entries)

    @property
    def present_count(self) -> int:
        return sum(1 for entry in self.entries if entry.exists)

    @property
    def ok_count(self) -> int:
        return sum(1 for entry in self.entries if entry.state == "ok")

    @property
    def missing_names(self) -> tuple[str, ...]:
        return tuple(entry.name for entry in self.entries if entry.state == "missing")

    @property
    def invalid_names(self) -> tuple[str, ...]:
        return tuple(entry.name for entry in self.entries if entry.state == "invalid")

    @property
    def ok(self) -> bool:
        return self.ok_count == self.total_count

    def to_dict(self) -> dict[str, object]:
        return {
            "root": str(self.root),
            "ok": self.ok,
            "total_count": self.total_count,
            "present_count": self.present_count,
            "ok_count": self.ok_count,
            "missing": list(self.missing_names),
            "invalid": list(self.invalid_names),
            "entries": [entry.to_dict() for entry in self.entries],
        }


@dataclass(frozen=True)
class OfflineCacheDoctorEntry:
    name: str
    path: Path
    exists: bool
    is_file: bool
    wrapped: bool
    schema_version: int | None
    valid_json: bool | None
    valid_payload: bool | None

    @property
    def schema_ok(self) -> bool:
        if not self.wrapped:
            return True
        return self.schema_version == _SRB_CACHE_SCHEMA_VERSION

    @property
    def state(self) -> str:
        if not self.exists:
            return "missing"
        if not self.is_file:
            return "invalid"
        if self.valid_json is False:
            return "invalid_json"
        if not self.schema_ok:
            return "schema"
        if self.valid_payload is False:
            return "invalid_payload"
        return "ok"

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "path": str(self.path),
            "state": self.state,
            "exists": self.exists,
            "is_file": self.is_file,
            "wrapped": self.wrapped,
            "schema_version": self.schema_version,
            "valid_json": self.valid_json,
            "valid_payload": self.valid_payload,
        }


@dataclass(frozen=True)
class OfflineCacheDoctorStatus:
    root: Path
    entries: tuple[OfflineCacheDoctorEntry, ...]

    @property
    def total_count(self) -> int:
        return len(self.entries)

    @property
    def ok_count(self) -> int:
        return sum(1 for entry in self.entries if entry.state == "ok")

    @property
    def ok(self) -> bool:
        return self.ok_count == self.total_count

    def to_dict(self) -> dict[str, object]:
        return {
            "root": str(self.root),
            "ok": self.ok,
            "total_count": self.total_count,
            "ok_count": self.ok_count,
            "entries": [entry.to_dict() for entry in self.entries],
        }


def offline_srb_cache_artifacts() -> tuple[OfflineCacheArtifact, ...]:
    return (
        OfflineCacheArtifact("env", SRB_ENV_CACHE_PATH),
        OfflineCacheArtifact("robot", SRB_ROBOT_CACHE_PATH),
        OfflineCacheArtifact("scenery", SRB_SCENERY_CACHE_PATH),
        OfflineCacheArtifact("object", SRB_OBJECT_CACHE_PATH),
        OfflineCacheArtifact("hardware", SRB_HARDWARE_INTERFACE_CACHE_PATH),
        OfflineCacheArtifact("config_keys", SRB_CONFIG_KEYS_CACHE_PATH),
        OfflineCacheArtifact("enum_values", SRB_ENUM_VALUES_CACHE_PATH),
        OfflineCacheArtifact("agent_config_keys", SRB_AGENT_CONFIG_KEYS_CACHE_PATH),
        OfflineCacheArtifact("agent_enum_values", SRB_AGENT_ENUM_VALUES_CACHE_PATH),
        OfflineCacheArtifact("action_modes", SRB_ACTION_MODES_CACHE_PATH),
        OfflineCacheArtifact("task_defaults", SRB_TASK_DEFAULTS_CACHE_PATH),
        OfflineCacheArtifact("gui_catalog", SRB_GUI_CATALOG_CACHE_PATH),
    )


_DEFAULT_CACHE_PATHS.update(
    {artifact.name: artifact.path for artifact in offline_srb_cache_artifacts()}
)
OFFLINE_CACHE_REQUIRED_MEMBERS = tuple(
    artifact.path.name for artifact in offline_srb_cache_artifacts()
)


def _cache_member_name(name: str) -> str:
    """Translate a logical cache name or filename to its generation member."""
    for artifact in offline_srb_cache_artifacts():
        if name in {artifact.name, artifact.path.name}:
            return artifact.path.name
    return name


def _cache_path_for_context(path: Path) -> Path:
    """Resolve legacy constants into the active API-owned build stage."""
    build_root = _CACHE_BUILD_ROOT.get()
    if build_root is None:
        return path
    if path in _DEFAULT_CACHE_PATHS.values():
        return build_root / path.name
    return path


def open_offline_srb_cache_snapshot(
    *,
    generation_root: Path | None = None,
) -> OfflineCacheSnapshot:
    """Bind the current complete generation and validate every member hash."""
    root = generation_root or SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH
    try:
        published = ArtifactPublication.generation(root).open(
            required_members=OFFLINE_CACHE_REQUIRED_MEMBERS
        )
    except (PublicationNotFoundError, IntegrityFailureError):
        raise
    except Exception as exc:  # noqa: BLE001 - normalize public cache integrity
        raise _cache_integrity_failure(root, exc) from exc
    assert isinstance(published, PublishedGeneration)
    return OfflineCacheSnapshot(
        generation_id=published.generation_id,
        path=published.path,
        member_hashes=published.member_hashes,
        _published=published,
    )


def _offline_cache_artifacts_at(root: Path) -> tuple[OfflineCacheArtifact, ...]:
    return tuple(
        OfflineCacheArtifact(artifact.name, root / artifact.path.name)
        for artifact in offline_srb_cache_artifacts()
    )


def _write_cache_if_changed(
    path: Path,
    payload: object,
    current: object,
    *,
    label: str,
) -> None:
    """Write *payload* as JSON unless it equals *current*.

    ``label`` is used in cache log messages; supply a noun phrase.
    """
    from srb.utils import logging

    path = _cache_path_for_context(path)
    if payload == current:
        logging.trace(f"The cache of {label} is up-to-date")
        return

    try:
        path.parent.mkdir(parents=True, exist_ok=True)
    except PermissionError:
        logging.warning(
            f"Skipping cache write for {label}; cache directory not writable: {path.parent}"
        )
        return

    try:
        _atomic_write_bytes(path, json.dumps(_wrap_cache_payload(payload)))
    except PermissionError:
        logging.warning(
            f"Skipping cache write for {label}; cache file not writable: {path}"
        )
        return

    logging.debug(f"Updated the cache of {label} to {path}")


def _read_typed_cache[T](
    path: Path,
    parser: Callable[[object], T | None],
    *,
    unlink_on_error: bool = True,
    raise_on_error: bool = True,
) -> T | None:
    """Load JSON from *path* and decode via *parser*.

    Missing files return ``None``. Read or parse failures raise a typed integrity
    error; ``raise_on_error=False`` lets diagnostic APIs report invalid entries.
    """

    path = _cache_path_for_context(path)

    def parse(payload: bytes) -> T:
        raw = _json_loads(payload)
        result = parser(_unwrap_cache_payload(raw))
        if result is None:
            raise ValueError("parser rejected payload")
        return result

    try:
        return cast(
            T | None,
            FilePublication(path).read(parse, remove_invalid=unlink_on_error),
        )
    except FileNotFoundError:
        return None
    except Exception as exc:  # noqa: BLE001 - convert arbitrary decoders to one type
        if not raise_on_error:
            return None
        raise _cache_integrity_failure(path, exc) from exc


def _read_offline_cache[T](
    path: Path,
    parser: Callable[[object], T | None],
    *,
    snapshot: OfflineCacheSnapshot | None = None,
) -> T | None:
    """Read one generation member. Non-canonical paths read legacy files for
    compatibility; canonical paths never fall back."""
    if _CACHE_BUILD_ROOT.get() is not None:
        return _read_typed_cache(path, parser, unlink_on_error=True)
    if path not in _DEFAULT_CACHE_PATHS.values():
        return _read_typed_cache(path, parser, unlink_on_error=True)
    try:
        bound = snapshot or open_offline_srb_cache_snapshot()
    except PublicationNotFoundError:
        return None
    except IntegrityFailureError:
        raise
    except Exception as exc:  # noqa: BLE001 - preserve a typed public seam
        raise _cache_integrity_failure(path, exc) from exc
    try:
        return bound.read(path.name, parser)
    except IntegrityFailureError:
        raise
    except Exception as exc:  # noqa: BLE001 - preserve corruption vs miss
        raise _cache_integrity_failure(path, exc) from exc


def _parse_non_null_json(raw: object) -> object | None:
    return raw


def _parse_generation_manifest(raw: object) -> dict[str, object] | None:
    if not isinstance(raw, dict):
        return None
    if raw.get("schema_version") != _SRB_CACHE_GENERATION_SCHEMA_VERSION:
        return None
    artifacts = raw.get("artifacts")
    if not isinstance(artifacts, dict) or not artifacts:
        return None
    if not all(
        isinstance(name, str) and isinstance(digest, str) and len(digest) == 64
        for name, digest in artifacts.items()
    ):
        return None
    return raw


def read_offline_srb_cache_generation(
    *, generation_path: Path | None = None
) -> dict[str, object] | None:
    """Read the current generation identity.

    Explicit ``generation_path`` reads the compatibility manifest; defaults
    resolve the immutable pointer.
    """
    if generation_path is None:
        try:
            snapshot = open_offline_srb_cache_snapshot()
        except PublicationNotFoundError:
            return None
        except IntegrityFailureError:
            raise
        except Exception as exc:  # noqa: BLE001 - current generation is corrupt
            raise _cache_integrity_failure(
                generation_path or SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH,
                exc,
            ) from exc
        return {
            "schema_version": _SRB_CACHE_GENERATION_SCHEMA_VERSION + 1,
            "generation_id": snapshot.generation_id,
            "artifacts": {
                artifact.name: snapshot.member_hashes[artifact.path.name]
                for artifact in offline_srb_cache_artifacts()
            },
            **_registry_fingerprint_metadata(snapshot),
        }
    return _read_typed_cache(
        generation_path,
        _parse_generation_manifest,
        unlink_on_error=False,
    )


def _registry_fingerprint_metadata(
    snapshot: OfflineCacheSnapshot,
) -> dict[str, str]:
    """Read optional source identity carried by newly published cache members."""
    try:
        raw = _json_loads(snapshot.read_bytes(SRB_ENV_CACHE_PATH.name))
    except (IntegrityFailureError, OSError, PublicationError):
        raise
    except (TypeError, ValueError):
        return {}
    if not isinstance(raw, dict):
        return {}
    metadata = raw.get(_SRB_CACHE_META_KEY)
    fingerprint = (
        metadata.get("registry_source_sha256") if isinstance(metadata, dict) else None
    )
    if (
        not isinstance(fingerprint, str)
        or len(fingerprint) != 64
        or any(character not in "0123456789abcdef" for character in fingerprint)
    ):
        return {}
    return {"registry_source_sha256": fingerprint}


def _cache_generation_semantic_problems(
    artifacts: Sequence[OfflineCacheArtifact],
    *,
    snapshot: OfflineCacheSnapshot | None = None,
) -> list[str]:
    """Reject empty or task-incomplete payloads in a full cache generation."""
    expected_names = {artifact.name for artifact in offline_srb_cache_artifacts()}
    actual = {artifact.name for artifact in artifacts}
    if actual != expected_names:
        # Small custom artifact sets are useful to callers/tests of the marker
        # primitive; cross-artifact completeness applies to the full SRB set.
        return []

    by_name = {artifact.name: artifact for artifact in artifacts}
    problems: list[str] = []

    def read(name: str, parser: Callable[[object], object | None]) -> object | None:
        if snapshot is not None:
            return snapshot.read(name, parser)
        return _read_typed_cache(
            by_name[name].path,
            parser,
            unlink_on_error=False,
        )

    env = read("env", _parse_list_of_str)
    runtime_ids = (
        {_short_cache_task_id(task) for task in env}
        if isinstance(env, Sequence) and not isinstance(env, (str, bytes))
        else set()
    )
    if not runtime_ids:
        problems.append("env cache is empty")

    for name in ("robot", "scenery", "object", "hardware"):
        payload = read(name, _parse_list_of_str if name == "hardware" else _parse_dict)
        if not payload:
            problems.append(f"{name} cache is empty")

    for name in ("config_keys", "task_defaults"):
        payload = read(name, _parse_dict)
        if not isinstance(payload, dict):
            problems.append(f"{name} cache is invalid")
            continue
        missing = sorted(runtime_ids - set(payload))
        if missing:
            problems.append(
                f"{name} cache is missing runtime tasks: {', '.join(missing)}"
            )

    enum_values = read("enum_values", _parse_dict)
    if not isinstance(enum_values, dict) or "__common__" not in enum_values:
        problems.append("enum_values cache lacks __common__")

    agent_keys = read("agent_config_keys", _parse_dict)
    if not isinstance(agent_keys, dict) or "__common__" not in agent_keys:
        problems.append("agent_config_keys cache lacks __common__")

    for name in ("agent_enum_values", "action_modes", "task_defaults", "gui_catalog"):
        payload = read(name, _parse_dict)
        if not payload:
            problems.append(f"{name} cache is empty")

    gui_catalog = read("gui_catalog", _parse_dict)
    if isinstance(gui_catalog, dict):
        records = gui_catalog.get("tasks")
        gui_task_ids: list[str] = []
        if isinstance(records, list):
            for record in records:
                if isinstance(record, str):
                    gui_task_ids.append(_short_cache_task_id(record))
                elif isinstance(record, Mapping) and isinstance(record.get("id"), str):
                    gui_task_ids.append(_short_cache_task_id(record["id"]))
                else:
                    problems.append("gui_catalog contains a task without an id")
        else:
            problems.append("gui_catalog tasks are invalid")

        if set(gui_task_ids) != runtime_ids or len(gui_task_ids) != len(
            set(gui_task_ids)
        ):
            problems.append(
                "gui_catalog tasks do not match the generated environment cache"
            )

        options = gui_catalog.get("options")
        if not isinstance(options, Mapping) or "tasks" not in options:
            problems.append("gui_catalog task options are invalid")
        else:
            option_tasks = options.get("tasks")
            if (
                not isinstance(option_tasks, list)
                or not all(isinstance(task, str) for task in option_tasks)
                or {_short_cache_task_id(task) for task in option_tasks} != runtime_ids
                or len({_short_cache_task_id(task) for task in option_tasks})
                != len(option_tasks)
            ):
                problems.append(
                    "gui_catalog task options do not match the environment cache"
                )
    return problems


def _short_cache_task_id(task: object) -> str:
    return str(task).split("/", 1)[-1]


#: Package initializers that define registry membership.
#:
#: Registration occurs while importing these packages; task and asset bodies
#: do not affect membership and are excluded from the fingerprint.
_REGISTRY_SOURCE_GLOBS: tuple[tuple[str, str], ...] = (
    ("tasks", "**/__init__.py"),
    ("assets", "**/__init__.py"),
)


def registry_source_fingerprint(root: Path | None = None) -> str | None:
    """Hash sources that determine registry ids.

    Return ``None`` when unavailable, such as in an installed wheel; callers
    must treat that as unknown, not as drift.
    """
    import hashlib

    base = root if root is not None else Path(__file__).resolve().parents[1]
    digest = hashlib.sha256()
    seen_any = False
    for package, pattern in _REGISTRY_SOURCE_GLOBS:
        package_root = base / package
        if not package_root.is_dir():
            continue
        for path in sorted(package_root.glob(pattern)):
            seen_any = True
            digest.update(path.relative_to(base).as_posix().encode("utf-8"))
            digest.update(path.read_bytes())
    return digest.hexdigest() if seen_any else None


def cache_generation_complete(
    *,
    artifacts: Sequence[OfflineCacheArtifact] | None = None,
    generation_path: Path | None = None,
    snapshot: OfflineCacheSnapshot | None = None,
) -> bool:
    """Return whether the current or explicitly supplied cache is complete."""
    if artifacts is None and generation_path is None:
        try:
            bound = snapshot or open_offline_srb_cache_snapshot()
            published_artifacts = _offline_cache_artifacts_at(bound.path)
        except _DIAGNOSTIC_FAILURES:
            return False
        doctor = doctor_offline_srb_cache(artifacts=published_artifacts)
        if not doctor.ok:
            return False
        try:
            return not _cache_generation_semantic_problems(
                published_artifacts,
                snapshot=bound,
            )
        except _DIAGNOSTIC_FAILURES:
            return False

    cache_artifacts = tuple(
        offline_srb_cache_artifacts() if artifacts is None else artifacts
    )
    doctor = doctor_offline_srb_cache(artifacts=cache_artifacts)
    if not doctor.ok:
        return False
    try:
        manifest = read_offline_srb_cache_generation(generation_path=generation_path)
        if manifest is None:
            return False
        recorded = manifest.get("artifacts")
        if not isinstance(recorded, dict):
            return False
        if _cache_generation_semantic_problems(cache_artifacts):
            return False
        expected = {
            artifact.name: sha256_file(artifact.path) for artifact in cache_artifacts
        }
    except _DIAGNOSTIC_FAILURES:
        return False
    return recorded == expected


def inspect_offline_srb_cache(
    *,
    artifacts: Sequence[OfflineCacheArtifact] | None = None,
) -> OfflineCacheStatus:
    """Inspect the current immutable cache without launching Isaac Sim."""
    bound_snapshot: OfflineCacheSnapshot | None = None
    if artifacts is None:
        try:
            bound_snapshot = open_offline_srb_cache_snapshot()
        except _DIAGNOSTIC_FAILURES:
            return _unavailable_cache_status()
        cache_artifacts = _offline_cache_artifacts_at(bound_snapshot.path)
    else:
        cache_artifacts = tuple(artifacts)
    return OfflineCacheStatus(
        root=SRB_CACHE_PATH,
        entries=tuple(
            _inspect_cache_artifact(artifact, snapshot=bound_snapshot)
            for artifact in cache_artifacts
        ),
    )


def _unavailable_cache_status() -> OfflineCacheStatus:
    root = SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH
    pointer_exists = root.joinpath("current.json").exists()
    state_root = root / "generations" / "<current>"
    entries = tuple(
        OfflineCacheEntryStatus(
            name=artifact.name,
            path=state_root / artifact.path.name,
            exists=pointer_exists,
            is_file=pointer_exists,
            valid_json=False if pointer_exists else None,
            size_bytes=None,
            modified_time=None,
        )
        for artifact in offline_srb_cache_artifacts()
    )
    return OfflineCacheStatus(root=SRB_CACHE_PATH, entries=entries)


def doctor_offline_srb_cache(
    *,
    artifacts: Sequence[OfflineCacheArtifact] | None = None,
) -> OfflineCacheDoctorStatus:
    """Validate cache payload shapes without modifying cache files."""

    def parser_for(name: str) -> Callable[[object], object | None]:
        if name == "env":
            return _parse_list_of_str
        if name == "hardware":
            return _parse_list_of_str
        if name == "robot":
            return _parse_mapping_of_string_lists
        if name == "scenery":
            return _parse_mapping_of_string_lists
        if name == "object":
            return _parse_mapping_of_string_lists
        return _parse_dict

    bound_snapshot: OfflineCacheSnapshot | None = None
    if artifacts is None:
        try:
            bound_snapshot = open_offline_srb_cache_snapshot()
        except _DIAGNOSTIC_FAILURES:
            unavailable = _unavailable_cache_status()
            unavailable_entries = tuple(
                OfflineCacheDoctorEntry(
                    name=entry.name,
                    path=entry.path,
                    exists=entry.exists,
                    is_file=entry.is_file,
                    wrapped=False,
                    schema_version=None,
                    valid_json=entry.valid_json,
                    valid_payload=False if entry.exists else None,
                )
                for entry in unavailable.entries
            )
            return OfflineCacheDoctorStatus(
                root=SRB_CACHE_PATH,
                entries=unavailable_entries,
            )
        cache_artifacts = _offline_cache_artifacts_at(bound_snapshot.path)
    else:
        cache_artifacts = tuple(artifacts)
    entries: list[OfflineCacheDoctorEntry] = []

    for artifact in cache_artifacts:
        path = artifact.path
        if not path.exists():
            entries.append(
                OfflineCacheDoctorEntry(
                    name=artifact.name,
                    path=path,
                    exists=False,
                    is_file=False,
                    wrapped=False,
                    schema_version=None,
                    valid_json=None,
                    valid_payload=None,
                )
            )
            continue

        is_file = path.is_file()
        if not is_file:
            entries.append(
                OfflineCacheDoctorEntry(
                    name=artifact.name,
                    path=path,
                    exists=True,
                    is_file=False,
                    wrapped=False,
                    schema_version=None,
                    valid_json=False,
                    valid_payload=None,
                )
            )
            continue

        try:
            raw = (
                _json_loads(bound_snapshot.read_bytes(artifact.path.name))
                if bound_snapshot is not None
                else _json_loads(path.read_bytes())
            )
        except Exception:
            entries.append(
                OfflineCacheDoctorEntry(
                    name=artifact.name,
                    path=path,
                    exists=True,
                    is_file=True,
                    wrapped=False,
                    schema_version=None,
                    valid_json=False,
                    valid_payload=None,
                )
            )
            continue

        wrapped = (
            isinstance(raw, dict)
            and _SRB_CACHE_META_KEY in raw
            and _SRB_CACHE_PAYLOAD_KEY in raw
        )
        schema_version = _cache_schema_version(raw) if wrapped else None
        payload = _unwrap_cache_payload(raw)

        try:
            parsed = parser_for(artifact.name)(payload)
            valid_payload = parsed is not None
        except Exception:
            valid_payload = False

        entries.append(
            OfflineCacheDoctorEntry(
                name=artifact.name,
                path=path,
                exists=True,
                is_file=True,
                wrapped=wrapped,
                schema_version=schema_version,
                valid_json=True,
                valid_payload=valid_payload,
            )
        )

    return OfflineCacheDoctorStatus(root=SRB_CACHE_PATH, entries=tuple(entries))


def _inspect_cache_artifact(
    artifact: OfflineCacheArtifact,
    *,
    snapshot: OfflineCacheSnapshot | None = None,
) -> OfflineCacheEntryStatus:
    path = artifact.path

    if snapshot is not None:
        exists = path.exists()
        is_file = path.is_file() if exists else False
        try:
            stat = path.stat()
            size_bytes = stat.st_size if is_file else None
            modified_time = stat.st_mtime
        except OSError:
            size_bytes = None
            modified_time = None

        if not exists:
            valid_json = None
        else:
            try:
                valid_json = (
                    _parse_non_null_json(_json_loads(snapshot.read_bytes(path.name)))
                    is not None
                )
            except _DIAGNOSTIC_FAILURES:
                valid_json = False
        return OfflineCacheEntryStatus(
            name=artifact.name,
            path=path,
            exists=exists,
            is_file=is_file,
            valid_json=valid_json,
            size_bytes=size_bytes,
            modified_time=modified_time,
        )

    if not path.exists():
        return OfflineCacheEntryStatus(
            name=artifact.name,
            path=path,
            exists=False,
            is_file=False,
            valid_json=None,
            size_bytes=None,
            modified_time=None,
        )

    is_file = path.is_file()
    try:
        stat = path.stat()
        size_bytes = stat.st_size if is_file else None
        modified_time = stat.st_mtime
    except OSError:
        size_bytes = None
        modified_time = None

    valid_json = (
        _read_typed_cache(
            path,
            _parse_non_null_json,
            unlink_on_error=False,
            raise_on_error=False,
        )
        is not None
        if is_file
        else False
    )
    return OfflineCacheEntryStatus(
        name=artifact.name,
        path=path,
        exists=True,
        is_file=is_file,
        valid_json=valid_json,
        size_bytes=size_bytes,
        modified_time=modified_time,
    )


def registry_source_drift() -> tuple[str, str] | None:
    """Return ``(recorded, current)`` when registry sources changed. Cache-only
    checks miss source drift; this digest closes that gap."""
    current = registry_source_fingerprint()
    if current is None:
        return None
    manifest = read_offline_srb_cache_generation()
    if manifest is None:
        return None
    recorded = manifest.get("registry_source_sha256")
    if not isinstance(recorded, str) or recorded == current:
        return None
    return recorded, current


def warn_if_offline_srb_cache_incomplete(
    *,
    status: OfflineCacheStatus | None = None,
) -> OfflineCacheStatus:
    """Warn about stale generated caches without regenerating them."""
    status = inspect_offline_srb_cache() if status is None else status
    generation_complete = (
        True if status.root != SRB_CACHE_PATH else cache_generation_complete()
    )

    # Warn without failing: stale registrations do not make the cache corrupt.
    try:
        drift = registry_source_drift() if status.root == SRB_CACHE_PATH else None
    except IntegrityFailureError:
        # A corrupt generation is reported below as incomplete.
        drift = None
    if drift is not None:
        from srb.utils import logging

        recorded, current = drift
        logging.warning(
            "The SRB offline cache was generated from different registry "
            f"sources than the current tree (recorded {recorded[:12]}, current "
            f"{current[:12]}). Task or asset registrations may have been added "
            "or removed since; ids listed from the cache can be stale. Run "
            "`srb cache update` to regenerate."
        )

    if status.ok and generation_complete:
        return status

    from srb.utils import logging

    details = []
    if status.missing_names:
        details.append(f"missing: {', '.join(status.missing_names)}")
    if status.invalid_names:
        details.append(f"invalid: {', '.join(status.invalid_names)}")
    if not generation_complete:
        details.append("cache generation marker missing or stale")
    detail = "; ".join(details) if details else "not all entries are valid"
    logging.warning(
        "SRB offline caches are incomplete "
        f"({detail}). Runtime launch will continue without regenerating caches; "
        "run `srb cache update` to refresh generated caches."
    )
    return status


def _parse_list_of_str(raw: object) -> Sequence[str] | None:
    if isinstance(raw, list) and all(isinstance(item, str) for item in raw):
        return raw
    return None


def _parse_dict(raw: object) -> dict | None:
    return raw if isinstance(raw, dict) else None


def _parse_mapping_of_string_lists(raw: object) -> dict | None:
    """Validate asset-cache JSON shape without importing Isaac asset enums."""
    if not isinstance(raw, dict):
        return None

    def valid(value: object) -> bool:
        if isinstance(value, list):
            return all(isinstance(item, str) for item in value)
        return isinstance(value, dict) and all(valid(child) for child in value.values())

    return raw if all(valid(value) for value in raw.values()) else None


def read_offline_srb_env_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> Sequence[str] | None:
    return _read_offline_cache(
        SRB_ENV_CACHE_PATH,
        _parse_list_of_str,
        snapshot=snapshot,
    )


def read_offline_srb_hardware_interface_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> Sequence[str] | None:
    return _read_offline_cache(
        SRB_HARDWARE_INTERFACE_CACHE_PATH,
        _parse_list_of_str,
        snapshot=snapshot,
    )


def read_offline_srb_config_keys_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict[str, list[str]] | None:
    return _read_offline_cache(
        SRB_CONFIG_KEYS_CACHE_PATH,
        _parse_dict,
        snapshot=snapshot,
    )


def read_offline_srb_enum_values_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict[str, dict[str, list[str]]] | None:
    return _read_offline_cache(
        SRB_ENUM_VALUES_CACHE_PATH,
        _parse_dict,
        snapshot=snapshot,
    )


def read_offline_srb_agent_config_keys_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict[str, list[str]] | None:
    return _read_offline_cache(
        SRB_AGENT_CONFIG_KEYS_CACHE_PATH,
        _parse_dict,
        snapshot=snapshot,
    )


def read_offline_srb_agent_enum_values_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict[str, list[str]] | None:
    return _read_offline_cache(
        SRB_AGENT_ENUM_VALUES_CACHE_PATH,
        _parse_dict,
        snapshot=snapshot,
    )


def read_offline_srb_action_modes_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict[str, list[str]] | None:
    return _read_offline_cache(
        SRB_ACTION_MODES_CACHE_PATH,
        _parse_dict,
        snapshot=snapshot,
    )


def read_offline_srb_task_defaults_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict[str, dict[str, str | bool | None]] | None:
    return _read_offline_cache(
        SRB_TASK_DEFAULTS_CACHE_PATH,
        _parse_dict,
        snapshot=snapshot,
    )


def read_offline_srb_gui_catalog_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict | None:
    # Preserve the miss-versus-integrity contract; the legacy GUI reader maps
    # malformed JSON to None and would hide corruption.
    return _read_offline_cache(
        SRB_GUI_CATALOG_CACHE_PATH,
        _parse_dict,
        snapshot=snapshot,
    )
