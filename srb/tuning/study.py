"""Optuna study storage, naming, and frozen manifests."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import stat
import statistics
import sys
import tempfile
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import yaml

from srb.core.algorithms import canonical_algorithm_label
from srb.tuning import require_optuna
from srb.utils.failures import attempt_cleanup
from srb.utils.publication import (
    FilePublication,
    stage_file_publications,
)


class StudyManifestMismatch(RuntimeError):
    """Raised when a resumed study's manifest conflicts with the request."""


class StudyManifestError(ValueError):
    """Raised when a study manifest is invalid or unreadable."""


# Frozen fields keep resumed tell() values comparable; seed/version drift is allowed.
_FROZEN_MANIFEST_FIELDS = (
    "space_id",
    "space_hash",
    "tasks",
    "obs",
    "trial_env_steps",
    "objective",
    "direction",
    "normalize",
    "aggregate",
    "pins",
    "profile_hash",
    "num_envs",
    "seeds_per_trial",
)
_RECOVERY_MANIFEST_ATTR = "srb.study.manifest.recovery.v1"
_PENDING_MANIFEST_NAME = ".study.manifest.pending.yaml"


@dataclass(frozen=True)
class StudyManifest:
    name: str
    space_id: str
    space_hash: str
    tasks: tuple[str, ...]
    obs: str
    trial_env_steps: int
    objective: str
    direction: str
    seed_base: int
    normalize: str
    aggregate: str
    optuna_version: str
    # None marks a legacy study with unverified frozen settings.
    pins: tuple[str, ...] | None = None
    profile_hash: str | None = None
    num_envs: int | None = None
    seeds_per_trial: int | None = None
    # Optional export label, including adapter; not frozen.
    algo: str = ""


def _task_set_hash(tasks: list[str]) -> str:
    # Order defines portfolio rungs and identity; sorting could alias studies.
    joined = "\n".join(tasks)
    return hashlib.sha256(joined.encode("utf-8")).hexdigest()[:8]


def study_name(algo: str, obs: str, tasks: list[str], version: int = 1) -> str:
    """Compose a study name: ``srb-{algo}-{obs}-{task|multi-<hash8>}-v{N}``."""
    algo = canonical_algorithm_label(algo)
    if len(tasks) == 1:
        scope = tasks[0]
    else:
        scope = f"multi-{_task_set_hash(tasks)}"
    return f"srb-{algo}-{obs}-{scope}-v{version}"


def _manifest_bytes(manifest: StudyManifest) -> bytes:
    if manifest.algo:
        manifest = replace(manifest, algo=canonical_algorithm_label(manifest.algo))
    data = asdict(manifest)
    data["tasks"] = list(manifest.tasks)  # YAML-friendly
    if manifest.pins is not None:
        data["pins"] = list(manifest.pins)
    return yaml.safe_dump(data, sort_keys=True).encode("utf-8")


def resolved_profile_hash(profile: object) -> str:
    """Hash resolved profile content independently of YAML key order."""
    try:
        canonical = yaml.safe_dump(profile, sort_keys=True).encode("utf-8")
    except (TypeError, ValueError, yaml.YAMLError) as exc:
        raise StudyManifestError(
            f"Resolved tuning profile is not safe-YAML materializable: {exc}"
        ) from exc
    return hashlib.sha256(canonical).hexdigest()


def read_manifest(path: str | Path) -> StudyManifest:
    destination = Path(path)
    try:
        data = yaml.safe_load(destination.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise TypeError("expected a YAML mapping")
        data = dict(data)

        required = {
            "name",
            "space_id",
            "space_hash",
            "tasks",
            "obs",
            "trial_env_steps",
            "objective",
            "direction",
            "seed_base",
            "normalize",
            "aggregate",
            "optuna_version",
        }
        missing = sorted(required - data.keys())
        if missing:
            raise ValueError("missing required field(s): " + ", ".join(missing))

        for field in (
            "name",
            "space_id",
            "space_hash",
            "obs",
            "objective",
            "direction",
            "normalize",
            "aggregate",
            "optuna_version",
        ):
            if not isinstance(data[field], str):
                raise TypeError(f"{field} must be a string")

        for field in ("trial_env_steps", "seed_base"):
            if isinstance(data[field], bool) or not isinstance(data[field], int):
                raise TypeError(f"{field} must be an integer")
        if data["trial_env_steps"] <= 0:
            raise ValueError("trial_env_steps must be positive")

        for field in ("num_envs", "seeds_per_trial"):
            if (
                field in data
                and data[field] is not None
                and (isinstance(data[field], bool) or not isinstance(data[field], int))
            ):
                raise TypeError(f"{field} must be an integer")
            if field in data and data[field] is not None and data[field] <= 0:
                raise ValueError(f"{field} must be positive")

        if "pins" in data:
            pins = data["pins"]
            if pins is not None and (
                not isinstance(pins, (list, tuple))
                or any(not isinstance(pin, str) for pin in pins)
            ):
                raise TypeError("pins must be a sequence of strings")
            if pins is not None:
                data["pins"] = tuple(pins)

        if (
            "profile_hash" in data
            and data["profile_hash"] is not None
            and not isinstance(data["profile_hash"], str)
        ):
            raise TypeError("profile_hash must be a string")

        if data["direction"] not in {"maximize", "minimize"}:
            raise ValueError("direction must be 'maximize' or 'minimize'")

        tasks = data.get("tasks", [])
        if not isinstance(tasks, (list, tuple)):
            raise TypeError("tasks must be a YAML sequence")
        if any(not isinstance(task, str) or not task.strip() for task in tasks):
            raise TypeError("tasks must contain non-empty strings")
        data["tasks"] = tuple(tasks)

        if "algo" in data:
            if not isinstance(data["algo"], str):
                raise TypeError("algo must be a string")
            if data["algo"]:
                data["algo"] = canonical_algorithm_label(data["algo"])
        return StudyManifest(**data)
    except OSError as exc:
        raise StudyManifestError(
            f"Cannot read study manifest {destination}: {exc}"
        ) from exc
    except yaml.YAMLError as exc:
        raise StudyManifestError(
            f"Invalid study manifest {destination}: {exc}"
        ) from exc
    except (TypeError, ValueError, KeyError) as exc:
        raise StudyManifestError(
            f"Invalid study manifest {destination}: {exc}"
        ) from exc


def _next_version_hint(name: str) -> str:
    m = re.search(r"-v(\d+)$", name)
    if not m:
        return f"{name}-v2"
    n = int(m.group(1))
    return re.sub(r"-v\d+$", f"-v{n + 1}", name)


def _manifest_frozen_values(manifest: StudyManifest) -> dict[str, object]:
    return {field: getattr(manifest, field) for field in _FROZEN_MANIFEST_FIELDS}


def _write_pending_manifest(path: Path, payload: bytes) -> None:
    """Durably install recovery metadata under lock before journal creation."""
    try:
        existing = os.lstat(path)
    except FileNotFoundError:
        existing = None
    if existing is not None:
        if stat.S_ISLNK(existing.st_mode) or not stat.S_ISREG(existing.st_mode):
            raise StudyManifestError(f"Invalid pending study manifest: {path}")
        return

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".stage",
        dir=path.parent,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        directory_descriptor = os.open(path.parent, directory_flags)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


@contextmanager
def _rdb_storage_lifetime(holder: dict[str, object], enabled: bool) -> Iterator[None]:
    """Dispose an owned RDB pool when study publication fails."""
    try:
        yield
    finally:
        if (
            enabled
            and holder.get("storage") is not None
            and sys.exc_info()[1] is not None
        ):
            from srb.tuning.storage import dispose_rdb_storage

            dispose_rdb_storage(holder["storage"])


def _check_frozen_manifest(
    name: str,
    existing: object,
    current: StudyManifest,
) -> None:
    if isinstance(existing, StudyManifest):
        if existing.name != name:
            raise StudyManifestMismatch(
                f"Study {name!r} has manifest name {existing.name!r}; "
                "refuse to mix study identities"
            )

        def value(field: str) -> object:
            return getattr(existing, field)
    elif isinstance(existing, dict):

        def value(field: str) -> object:
            return existing.get(field)
    else:
        raise StudyManifestError(
            f"Invalid study manifest recovery record for {name!r}: expected a mapping"
        )

    def normalized(field: str, value: object) -> object:
        # JSON encodes tuples as lists; compare values, not representation.
        if field in {"tasks", "pins"} and isinstance(value, (list, tuple)):
            return tuple(value)
        return value

    mismatched = [
        field
        for field in _FROZEN_MANIFEST_FIELDS
        if normalized(field, value(field)) != normalized(field, getattr(current, field))
    ]
    if not mismatched:
        return
    details = "; ".join(
        f"{field}: stored {value(field)!r} != current {getattr(current, field)!r}"
        for field in mismatched
    )
    raise StudyManifestMismatch(
        f"Study {name!r} was created with incompatible settings "
        f"({details}). Mixing spaces/budgets/objectives corrupts the tell "
        f"history — start a new study version, e.g. {_next_version_hint(name)}."
    )


def make_journal_storage(journal_path: str | Path):
    """Return a ``JournalStorage`` file backend (NFS-safe), version-tolerant."""
    optuna = require_optuna()
    path = str(journal_path)
    try:  # optuna >= 4.x modern path
        from optuna.storages.journal import JournalFileBackend

        backend = JournalFileBackend(path)
    except ImportError:  # pragma: no cover - older 4.6 fallback
        from optuna.storages import JournalFileStorage

        backend = JournalFileStorage(path)
    return optuna.storages.JournalStorage(backend)


def build_sampler(n_startup: int, *, seed: int | None = None):
    """TPE sampler with multivariate modeling and a startup budget."""
    optuna = require_optuna()
    return optuna.samplers.TPESampler(
        multivariate=True, n_startup_trials=int(n_startup), seed=seed
    )


def build_pruner(n_warmup_rungs: int):
    """MedianPruner that ignores rungs below the warmup fraction."""
    optuna = require_optuna()

    class CurrentRungMedianPruner(optuna.pruners.MedianPruner):
        """Compare same-rung values, not each trial's best value."""

        def prune(self, study, trial) -> bool:
            completed = study.get_trials(
                deepcopy=False,
                states=(optuna.trial.TrialState.COMPLETE,),
            )
            if len(completed) < self._n_startup_trials:
                return False
            step = trial.last_step
            if step is None or step < self._n_warmup_steps:
                return False
            if (step - self._n_warmup_steps) % self._interval_steps:
                return False

            value = trial.intermediate_values[step]
            if math.isnan(value):
                return True
            peers = [
                peer.intermediate_values[step]
                for peer in completed
                if step in peer.intermediate_values
                and not math.isnan(peer.intermediate_values[step])
            ]
            if len(peers) < self._n_min_trials:
                return False
            threshold = statistics.median(peers)
            if study.direction.name == "MAXIMIZE":
                return value < threshold
            return value > threshold

    return CurrentRungMedianPruner(n_warmup_steps=int(n_warmup_rungs))


@contextmanager
def _trial_budget_lock(
    study, *, lock_path: str | Path, storage: str | None
) -> Iterator[None]:
    if storage is None:
        import fcntl

        lock_file = Path(lock_path)
        lock_file.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(lock_file, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)
        return

    from sqlalchemy import text

    from srb.tuning.storage import advisory_lock_key

    owned_storage = getattr(study, "_srb_owned_rdb_storage", None)
    if owned_storage is None:
        owned_storage = getattr(study, "_storage", None)
        owned_storage = getattr(owned_storage, "_backend", owned_storage)
    engine = getattr(owned_storage, "engine", None)
    if engine is None:
        raise RuntimeError("PostgreSQL tuning study has no owned RDB engine")
    connection = engine.connect()
    lock_key = advisory_lock_key(f"{study.study_name}:trial-budget")
    locked = False
    try:
        connection.execute(
            text("SELECT pg_advisory_lock(:lock_key)"),
            {"lock_key": lock_key},
        )
        locked = True
        connection.commit()
        yield
    finally:
        unlock_error = None
        if locked:

            def unlock() -> None:
                connection.execute(
                    text("SELECT pg_advisory_unlock(:lock_key)"),
                    {"lock_key": lock_key},
                )
                connection.commit()

            _, unlock_error = attempt_cleanup(unlock)
            if unlock_error is not None:
                attempt_cleanup(connection.invalidate)
        _, close_error = attempt_cleanup(connection.close)
        if unlock_error is not None:
            raise unlock_error
        if close_error is not None:
            raise close_error


def reserve_study_trial(
    study,
    n_trials: int,
    *,
    lock_path: str | Path,
    storage: str | None = None,
):
    """Atomically ask for one trial while respecting a shared study budget."""
    with _trial_budget_lock(study, lock_path=lock_path, storage=storage):
        trials = study.get_trials(deepcopy=False)
        used = sum(
            trial.state.name in {"RUNNING", "COMPLETE", "PRUNED", "FAIL"}
            for trial in trials
        )
        if used >= n_trials:
            return None
        return study.ask()


def open_study(
    name: str,
    study_dir: str | Path,
    manifest: StudyManifest,
    *,
    sampler,
    pruner,
    load_if_exists: bool = True,
    storage: str | None = None,
):
    """Create or load a study with frozen-manifest validation.

    ``storage=None`` selects the journal file backend.  A supplied URL selects
    the owned PostgreSQL backend and its database identity coordinator; it is
    never silently redirected to a local journal.

    :raises StudyManifestMismatch: if stored frozen settings differ from
        *manifest* (suggests a version bump).
    :raises StudyManifestError: if a manifest or recovery record is malformed.
    """
    optuna = require_optuna()
    study_dir = Path(study_dir)
    study_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = study_dir / "study.yaml"
    pending_manifest_path = study_dir / _PENDING_MANIFEST_NAME
    journal_path = study_dir / "journal.log"
    is_rdb = storage is not None
    # Lock metadata read/check/create/publish together; keep trial work unlocked.
    # Optuna's journal coordinates concurrent trial state.
    metadata_lock = study_dir / ".study.metadata.lock"
    publication = FilePublication(
        manifest_path,
        lock_path=metadata_lock,
    )
    rdb_storage_holder: dict[str, object] = {"storage": None}
    with (
        _rdb_storage_lifetime(rdb_storage_holder, is_rdb),
        stage_file_publications((publication,)) as (metadata_stage,),
    ):
        existing: StudyManifest | None = None
        pending_manifest: StudyManifest | None = None
        if manifest_path.exists():
            existing = read_manifest(manifest_path)
            _check_frozen_manifest(name, existing, manifest)
        if pending_manifest_path.exists() and not is_rdb:
            pending_manifest = read_manifest(pending_manifest_path)
            _check_frozen_manifest(name, pending_manifest, manifest)
        if not is_rdb and existing is not None and not journal_path.is_file():
            raise StudyManifestError(
                f"Study {name!r} has a published manifest without a regular "
                "journal.log; restore its journal history or supply its "
                "PostgreSQL storage URL instead of creating an empty study"
            )
        if existing is None:
            if not is_rdb and journal_path.exists() and pending_manifest is None:
                raise StudyManifestError(
                    f"Study {name!r} has a journal without a manifest recovery "
                    "record; refuse to guess its frozen settings"
                )
            if pending_manifest is None:
                pending_manifest = replace(manifest, name=name)
                _write_pending_manifest(
                    pending_manifest_path,
                    _manifest_bytes(pending_manifest),
                )

        if is_rdb:
            from srb.tuning.storage import (
                StudyIdentityConflict,
                StudyStorageError,
                coordinate_rdb_study,
                make_rdb_storage,
                rdb_operation,
            )

            # Persist DB identity before create/load; pending file is local recovery,
            # not cross-host authority.
            rdb_storage = make_rdb_storage(storage)
            rdb_storage_holder["storage"] = rdb_storage
            identity_manifest = replace(manifest, name=name)
            identity_values = _manifest_frozen_values(identity_manifest)
            try:
                coordinator = coordinate_rdb_study(
                    rdb_storage,
                    name,
                    identity_values,
                    dsn=storage,
                )
                with coordinator:
                    with rdb_operation(dsn=storage, operation="study recovery lookup"):
                        try:
                            existing_study_id = rdb_storage.get_study_id_from_name(name)
                        except KeyError:
                            existing_recovery = None
                        else:
                            existing_recovery = rdb_storage.get_study_system_attrs(
                                existing_study_id
                            ).get(_RECOVERY_MANIFEST_ATTR)
                    if existing_recovery is not None:
                        _check_frozen_manifest(name, existing_recovery, manifest)
                    with rdb_operation(dsn=storage, operation="study create/load"):
                        study = optuna.create_study(
                            study_name=name,
                            storage=rdb_storage,
                            sampler=sampler,
                            pruner=pruner,
                            direction=manifest.direction,
                            load_if_exists=load_if_exists,
                        )
                        # Keep the owned backend; Optuna attaches a cached wrapper.
                        setattr(study, "_srb_owned_rdb_storage", rdb_storage)
                        recovery = study.system_attrs.get(_RECOVERY_MANIFEST_ATTR)
                    if recovery is None:
                        _check_frozen_manifest(name, identity_manifest, manifest)
                        with rdb_operation(
                            dsn=storage, operation="study recovery attribute write"
                        ):
                            study.set_system_attr(
                                _RECOVERY_MANIFEST_ATTR,
                                json.loads(
                                    json.dumps(
                                        _manifest_frozen_values(identity_manifest)
                                    )
                                ),
                            )
                    else:
                        _check_frozen_manifest(name, recovery, manifest)
            except StudyIdentityConflict as exc:
                # Preserve journal mismatch wording; DB identity is authoritative.
                _check_frozen_manifest(name, exc.stored, manifest)
                raise AssertionError("unreachable identity conflict") from exc
            except StudyStorageError:
                raise
        else:
            # Pending manifest makes retries idempotent after journal creation, even if promotion is interrupted.
            journal_storage = make_journal_storage(journal_path)
            if existing is not None:
                try:
                    existing_study_id = journal_storage.get_study_id_from_name(name)
                except KeyError as exc:
                    raise StudyManifestError(
                        f"Study {name!r} has a published manifest but its "
                        "journal does not contain the named study; restore "
                        "the matching journal history instead of creating "
                        "an empty study"
                    ) from exc
                existing_recovery = journal_storage.get_study_system_attrs(
                    existing_study_id
                ).get(_RECOVERY_MANIFEST_ATTR)
                if existing_recovery is not None:
                    _check_frozen_manifest(name, existing_recovery, manifest)
            study = optuna.create_study(
                study_name=name,
                storage=journal_storage,
                sampler=sampler,
                pruner=pruner,
                direction=manifest.direction,
                load_if_exists=load_if_exists,
            )
            # Validate recovery metadata on resume; initialize it for older studies.
            recovery = study.system_attrs.get(_RECOVERY_MANIFEST_ATTR)
            if recovery is None:
                identity_manifest = existing or pending_manifest
                assert identity_manifest is not None
                _check_frozen_manifest(name, identity_manifest, manifest)
                study.set_system_attr(
                    _RECOVERY_MANIFEST_ATTR,
                    json.loads(json.dumps(_manifest_frozen_values(identity_manifest))),
                )
            else:
                _check_frozen_manifest(name, recovery, manifest)
        if existing is None:
            metadata_stage.path.write_bytes(
                _manifest_bytes(replace(manifest, name=name))
            )
            metadata_stage.publish()
        else:
            metadata_stage.discard()
        # Retry cleanup under the publication lease without republishing study.yaml.
        try:
            pending_manifest_path.unlink()
        except FileNotFoundError:
            pass
        return study


def open_existing_study(study_dir: str | Path, *, storage: str | None = None):
    """Reopen a study directory for read and maintenance operations.

    Returns ``(study, manifest)``. Status, export, and repair share this path;
    sampler and pruner settings are irrelevant, so minimal instances are used.
    """
    study_dir = Path(study_dir)
    manifest = read_manifest(study_dir / "study.yaml")
    study = open_study(
        manifest.name,
        study_dir,
        manifest,
        sampler=build_sampler(1),
        pruner=build_pruner(1),
        storage=storage,
    )
    return study, manifest


__all__ = (
    "StudyManifest",
    "StudyManifestError",
    "StudyManifestMismatch",
    "study_name",
    "read_manifest",
    "resolved_profile_hash",
    "make_journal_storage",
    "build_sampler",
    "build_pruner",
    "reserve_study_trial",
    "open_study",
    "open_existing_study",
)
