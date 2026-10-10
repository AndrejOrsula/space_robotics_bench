"""Owned Optuna storage: journal by default, optional PostgreSQL RDB via psycopg2.

RDB commits an SRB identity row before study creation and checks the
additive-only schema on use. Operators manage migrations and backups.
"""

from __future__ import annotations

import hashlib
import json
import sys
import time
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any


class StudyStorageError(RuntimeError):
    """A storage configuration, connection, or schema failure."""


class StudyIdentityConflict(StudyStorageError):
    """The durable RDB identity differs from the requested frozen manifest."""

    def __init__(
        self,
        study_name: str,
        stored: Mapping[str, object],
        requested: Mapping[str, object],
    ) -> None:
        self.study_name = study_name
        self.stored = dict(stored)
        self.requested = dict(requested)
        super().__init__(f"durable identity conflict for study {study_name!r}")


RDB_POOL_SIZE = 4
RDB_MAX_OVERFLOW = 0
RDB_POOL_TIMEOUT_S = 30
RDB_POOL_RECYCLE_S = 1800
RDB_CONNECT_TIMEOUT_S = 10
RDB_LOCK_TIMEOUT_S = 30
RDB_LOCK_POLL_S = 0.05

_SUPPORTED_DRIVERS = frozenset({"postgresql", "postgresql+psycopg2"})
# Keep BaseException catches at cleanup seams; preserve body failures.
# Re-raise cancellation when cleanup is the only failure.
_CLEANUP_BASE_EXCEPTION = BaseException
_IDENTITY_TABLE = "srb_tuning_study_identity"
_IDENTITY_SCHEMA_VERSION = 1
_IDENTITY_COLUMNS = frozenset(
    {"study_name", "schema_version", "manifest_json", "created_at"}
)
_IDENTITY_TABLE_SQL = f"""
CREATE TABLE IF NOT EXISTS {_IDENTITY_TABLE} (
    study_name VARCHAR(255) PRIMARY KEY,
    schema_version INTEGER NOT NULL,
    manifest_json TEXT NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT CURRENT_TIMESTAMP
)
"""


@dataclass(frozen=True)
class RDBPoolPolicy:
    """Bounded connection policy used for every owned RDB storage."""

    pool_size: int = RDB_POOL_SIZE
    max_overflow: int = RDB_MAX_OVERFLOW
    pool_timeout: int = RDB_POOL_TIMEOUT_S
    pool_recycle: int = RDB_POOL_RECYCLE_S
    connect_timeout: int = RDB_CONNECT_TIMEOUT_S

    def engine_kwargs(self) -> dict[str, object]:
        return {
            "pool_size": self.pool_size,
            "max_overflow": self.max_overflow,
            "pool_timeout": self.pool_timeout,
            "pool_pre_ping": True,
            "pool_recycle": self.pool_recycle,
            "connect_args": {"connect_timeout": self.connect_timeout},
        }


def _url_parser() -> Any:
    try:
        from sqlalchemy.engine import make_url
    except ImportError:
        raise StudyStorageError(
            "PostgreSQL Optuna storage requires SQLAlchemy; install the tune extra"
        ) from None
    return make_url


def _url_parse_errors() -> tuple[type[BaseException], ...]:
    """Return parser errors without importing SQLAlchemy at module load."""
    try:
        from sqlalchemy.exc import ArgumentError
    except ImportError:
        return (TypeError, ValueError)
    return (ArgumentError, TypeError, ValueError)


def _storage_exception_types() -> tuple[type[BaseException], ...]:
    """Known dependency and boundary errors translated by this owner."""
    errors: list[type[BaseException]] = [
        AttributeError,
        KeyError,
        OSError,
        RuntimeError,
        TypeError,
        ValueError,
    ]
    try:
        from optuna.exceptions import OptunaError
    except ImportError:
        pass
    else:
        errors.append(OptunaError)
    try:
        from sqlalchemy.exc import SQLAlchemyError
    except ImportError:
        pass
    else:
        errors.append(SQLAlchemyError)
    return tuple(errors)


def redact_storage_url(dsn: str) -> str:
    """Render only the safe scheme, host, port, and database identity."""
    if not isinstance(dsn, str) or not dsn.strip():
        return "<invalid storage DSN>"
    try:
        url = _url_parser()(dsn)
        host = url.host or "<host>"
        if ":" in host and not host.startswith("["):
            host = f"[{host}]"
        port = f":{url.port}" if url.port is not None else ""
        database = url.database or "<database>"
        return f"{url.drivername}://{host}{port}/{database}"
    except _url_parse_errors():
        return "<invalid storage DSN>"


def validate_rdb_dsn(dsn: str) -> Any:
    """Validate a PostgreSQL URL without exposing it in errors."""
    if not isinstance(dsn, str) or not dsn.strip():
        raise StudyStorageError(
            "Optuna RDB storage requires a PostgreSQL DSN using "
            "postgresql:// or postgresql+psycopg2://"
        )
    try:
        url = _url_parser()(dsn)
    except _url_parse_errors():
        raise StudyStorageError(
            "Invalid Optuna RDB storage DSN; expected a PostgreSQL URL"
        ) from None

    if url.drivername not in _SUPPORTED_DRIVERS:
        raise StudyStorageError(
            "Unsupported Optuna RDB storage backend; only PostgreSQL with "
            "the psycopg2 driver is supported"
        )
    if not url.host or not url.database or not url.username:
        raise StudyStorageError(
            "PostgreSQL Optuna storage DSN must include host, database, and username"
        )
    return url


def rdb_engine_kwargs(policy: RDBPoolPolicy | None = None) -> dict[str, object]:
    """Return a fresh bounded SQLAlchemy engine policy for tests and callers."""
    return (policy or RDBPoolPolicy()).engine_kwargs()


def _invalidate_connection(connection: Any, engine: Any | None = None) -> None:
    """Make a connection unusable after a lock protocol failure."""
    try:
        connection.invalidate()
    except _storage_exception_types():
        pass
    if engine is not None:
        try:
            engine.dispose()
        except _storage_exception_types():
            pass


def _dispose_storage(storage: Any | None) -> None:
    """Dispose an RDB pool when bootstrap publication cannot complete."""
    backend = _unwrap_rdb_backend(storage)
    remove_session = getattr(backend, "remove_session", None)
    if callable(remove_session):
        try:
            remove_session()
        except _storage_exception_types():
            pass
    engine = getattr(backend, "engine", None)
    if engine is not None:
        try:
            engine.dispose()
        except _storage_exception_types():
            pass


def dispose_rdb_storage(storage: Any | None) -> None:
    """Dispose an owned RDB pool after a failed study bootstrap."""
    _dispose_storage(storage)


def remove_rdb_session(storage: Any | None) -> None:
    """Remove the current thread's Optuna scoped session, when present."""
    backend = _unwrap_rdb_backend(storage)
    remove_session = getattr(backend, "remove_session", None)
    if callable(remove_session):
        try:
            remove_session()
        except _storage_exception_types():
            pass


def _unwrap_rdb_backend(storage: Any | None) -> Any | None:
    """Return an Optuna RDB backend from its current cached wrapper."""
    backend = getattr(storage, "_backend", None)
    if backend is not None and getattr(backend, "engine", None) is not None:
        return backend
    return storage


def _scalar_bool(result: Any) -> bool:
    """Read the boolean returned by a PostgreSQL scalar query."""
    scalar_one = getattr(result, "scalar_one", None)
    if scalar_one is not None:
        return bool(scalar_one())
    scalar = getattr(result, "scalar", None)
    if scalar is not None:
        return bool(scalar())
    return bool(result)


def _acquire_advisory_lock(connection: Any, key: int, redacted: str) -> None:
    """Acquire a session lock with a bounded wait."""
    try:
        from sqlalchemy import text
    except ImportError:
        raise StudyStorageError(
            "PostgreSQL Optuna storage requires SQLAlchemy; install the tune extra"
        ) from None

    deadline = time.monotonic() + RDB_LOCK_TIMEOUT_S
    while True:
        try:
            result = connection.execute(
                text("SELECT pg_try_advisory_lock(:lock_key)"),
                {"lock_key": key},
            )
            acquired = _scalar_bool(result)
            connection.commit()
        except _storage_exception_types() as exc:
            _invalidate_connection(connection)
            raise _storage_error("advisory lock acquisition", redacted, exc) from None
        if acquired:
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise StudyStorageError(
                "PostgreSQL Optuna advisory lock timed out at "
                f"{redacted} after {RDB_LOCK_TIMEOUT_S} seconds"
            )
        time.sleep(min(RDB_LOCK_POLL_S, remaining))


def _release_advisory_lock(
    connection: Any, key: int, redacted: str, *, engine: Any | None = None
) -> None:
    """Release the session lock; invalidate its connection on failure."""
    released = False
    try:
        from sqlalchemy import text

        result = connection.execute(
            text("SELECT pg_advisory_unlock(:lock_key)"),
            {"lock_key": key},
        )
        unlocked = _scalar_bool(result)
        connection.commit()
        if not unlocked:
            raise StudyStorageError(
                f"PostgreSQL Optuna advisory lock was not held at {redacted}"
            )
        released = True
    except StudyStorageError:
        raise
    except _storage_exception_types() as exc:
        raise _storage_error("advisory lock release", redacted, exc) from None
    finally:
        if not released:
            _invalidate_connection(connection, engine)


@contextmanager
def _bootstrap_schema_lock(engine: Any, redacted: str) -> Iterator[None]:
    """Serialize identity DDL and Optuna's initial RDB schema creation."""
    connection: Any | None = None
    acquired = False
    body_error: BaseException | None = None
    key = advisory_lock_key("__srb_tuning_rdb_schema_v1__")
    try:
        connection = engine.connect()
        _acquire_advisory_lock(connection, key, redacted)
        acquired = True
        _ensure_identity_schema(connection, redacted)
        yield
    finally:
        body_error = sys.exc_info()[1]
        release_error: BaseException | None = None
        if connection is not None:
            try:
                if acquired:
                    try:
                        _release_advisory_lock(connection, key, redacted, engine=engine)
                    except _CLEANUP_BASE_EXCEPTION as exc:
                        release_error = exc
            finally:
                try:
                    connection.close()
                except _CLEANUP_BASE_EXCEPTION as exc:
                    if release_error is None:
                        release_error = exc
        if release_error is not None and body_error is None:
            raise release_error


def make_rdb_storage(dsn: str, *, policy: RDBPoolPolicy | None = None) -> Any:
    """Build an Optuna PostgreSQL ``RDBStorage`` without a journal fallback."""
    url = validate_rdb_dsn(dsn)
    redacted = redact_storage_url(dsn)
    bootstrap_engine: Any | None = None
    rdb_storage: Any | None = None
    storage_ready = False
    try:
        from srb.utils.extras import require_extra

        with require_extra("tune-rdb"):
            from optuna.storages import RDBStorage
            from sqlalchemy import create_engine

        # Serialize Optuna's check-then-create DDL. Dispose this single-connection
        # bootstrap pool before the returned pool serves trial traffic.
        bootstrap_kwargs = rdb_engine_kwargs(policy)
        bootstrap_kwargs.update({"pool_size": 1, "max_overflow": 0})
        bootstrap_engine = create_engine(
            url.render_as_string(hide_password=False), **bootstrap_kwargs
        )
        with _bootstrap_schema_lock(bootstrap_engine, redacted):
            rdb_storage = RDBStorage(
                url=url.render_as_string(hide_password=False),
                engine_kwargs=rdb_engine_kwargs(policy),
                # Fail closed on schema drift; never migrate implicitly.
                skip_compatibility_check=False,
            )
        storage_ready = True
        return rdb_storage
    except ImportError:
        raise StudyStorageError(
            "PostgreSQL Optuna storage needs the psycopg2 driver; "
            "install the `tune-rdb` extra"
        ) from None
    except StudyStorageError:
        raise
    except _storage_exception_types() as exc:
        raise StudyStorageError(
            "Could not initialize PostgreSQL Optuna storage at "
            f"{redacted} ({type(exc).__name__}); verify connectivity and schema "
            "compatibility, and back up before any operator-managed upgrade"
        ) from None
    finally:
        body_error = sys.exc_info()[1]
        if not storage_ready:
            _dispose_storage(rdb_storage)
        if bootstrap_engine is not None:
            dispose_error: BaseException | None = None
            try:
                bootstrap_engine.dispose()
            except _storage_exception_types():
                pass
            except _CLEANUP_BASE_EXCEPTION as exc:
                dispose_error = exc
                _dispose_storage(rdb_storage)
            if dispose_error is not None and body_error is None:
                raise dispose_error


def _manifest_json(values: Mapping[str, object]) -> tuple[str, dict[str, object]]:
    try:
        encoded = json.dumps(
            dict(values), sort_keys=True, separators=(",", ":"), ensure_ascii=True
        )
        decoded = json.loads(encoded)
    except (TypeError, ValueError):
        raise StudyStorageError(
            "Tuning study manifest contains values that cannot be stored in RDB"
        ) from None
    if not isinstance(decoded, dict):  # pragma: no cover - json.dumps(mapping)
        raise StudyStorageError("Tuning study manifest must be a JSON object")
    return encoded, decoded


def advisory_lock_key(study_name: str) -> int:
    """Return a stable signed PostgreSQL advisory-lock key for a study name."""
    digest = hashlib.sha256(study_name.encode("utf-8")).digest()[:8]
    return int.from_bytes(digest, byteorder="big", signed=True)


def _storage_error(
    operation: str, redacted: str, exc: BaseException
) -> StudyStorageError:
    return StudyStorageError(
        f"PostgreSQL Optuna storage {operation} failed at {redacted} "
        f"({type(exc).__name__}); check connectivity and compatible schema"
    )


@contextmanager
def rdb_operation(*, dsn: str, operation: str) -> Iterator[None]:
    """Translate raw Optuna/SQLAlchemy failures without exposing exception text."""
    redacted = redact_storage_url(dsn)
    try:
        yield
    except StudyStorageError:
        raise
    except _storage_exception_types() as exc:
        raise _storage_error(operation, redacted, exc) from None


def _ensure_identity_schema(connection: Any, redacted: str) -> None:
    """Create the additive-only identity table and verify its shape."""
    try:
        from sqlalchemy import text

        connection.execute(text(_IDENTITY_TABLE_SQL))
        connection.commit()
    except _storage_exception_types() as exc:
        raise _storage_error("identity schema initialization", redacted, exc) from None
    _check_identity_schema(connection, redacted)


def _check_identity_schema(connection: Any, redacted: str) -> None:
    """Verify the additive-only identity table without performing DDL."""
    try:
        from sqlalchemy import inspect

        columns = {
            str(column["name"])
            for column in inspect(connection).get_columns(_IDENTITY_TABLE)
        }
        connection.commit()
    except _storage_exception_types() as exc:
        raise _storage_error("identity schema check", redacted, exc) from None
    missing = sorted(_IDENTITY_COLUMNS - columns)
    if missing:
        raise StudyStorageError(
            "PostgreSQL Optuna identity schema is incompatible at "
            f"{redacted}; missing required columns {', '.join(missing)}. "
            "Back up and migrate it with an operator-managed procedure"
        )


@contextmanager
def coordinate_rdb_study(
    storage: Any,
    study_name: str,
    manifest_values: Mapping[str, object],
    *,
    dsn: str,
) -> Iterator[None]:
    """Lock, reserve, and verify study identity across PostgreSQL workers.

    The committed identity lets a later worker finish creation or repair
    Optuna's recovery attribute after a crash.
    """
    if not isinstance(study_name, str) or not study_name or len(study_name) > 255:
        raise StudyStorageError("Tuning study name must be 1..255 characters")
    values_json, values = _manifest_json(manifest_values)
    redacted = redact_storage_url(dsn)
    engine = getattr(storage, "engine", None)
    if engine is None:
        raise StudyStorageError("PostgreSQL Optuna storage has no SQLAlchemy engine")

    try:
        from sqlalchemy import text
    except ImportError:
        raise StudyStorageError(
            "PostgreSQL Optuna storage requires SQLAlchemy; install the tune extra"
        ) from None

    connection: Any | None = None
    acquired = False
    body_error: BaseException | None = None
    key = advisory_lock_key(study_name)
    try:
        try:
            connection = engine.connect()
            _acquire_advisory_lock(connection, key, redacted)
            acquired = True
            _check_identity_schema(connection, redacted)
            with connection.begin():
                row = (
                    connection.execute(
                        text(
                            f"SELECT schema_version, manifest_json FROM {_IDENTITY_TABLE} "
                            "WHERE study_name = :study_name FOR UPDATE"
                        ),
                        {"study_name": study_name},
                    )
                    .mappings()
                    .first()
                )
                if row is None:
                    # Without an identity row, adopt only if the Optuna study
                    # is also absent. The held advisory lock prevents a race
                    # between this check and the identity insert.
                    try:
                        storage.get_study_id_from_name(study_name)
                    except KeyError:
                        pass
                    except _storage_exception_types() as exc:
                        raise _storage_error(
                            "study identity existence check", redacted, exc
                        ) from None
                    else:
                        raise StudyStorageError(
                            "PostgreSQL Optuna study exists without an SRB durable "
                            f"identity at {redacted}; refuse to adopt it. Back up "
                            "and migrate the study with an operator-managed procedure"
                        )
                    connection.execute(
                        text(
                            f"INSERT INTO {_IDENTITY_TABLE} "
                            "(study_name, schema_version, manifest_json) "
                            "VALUES (:study_name, :schema_version, :manifest_json)"
                        ),
                        {
                            "study_name": study_name,
                            "schema_version": _IDENTITY_SCHEMA_VERSION,
                            "manifest_json": values_json,
                        },
                    )
                else:
                    if int(row["schema_version"]) != _IDENTITY_SCHEMA_VERSION:
                        raise StudyStorageError(
                            "PostgreSQL Optuna identity schema version is unsupported "
                            f"at {redacted}; back up and migrate it with an "
                            "operator-managed procedure"
                        )
                    try:
                        stored = json.loads(str(row["manifest_json"]))
                    except (TypeError, ValueError):
                        raise StudyStorageError(
                            "PostgreSQL Optuna identity record is invalid; "
                            "refuse to guess the frozen study definition"
                        ) from None
                    if not isinstance(stored, dict):
                        raise StudyStorageError(
                            "PostgreSQL Optuna identity record is not a JSON object; "
                            "refuse to guess the frozen study definition"
                        )
                    if (
                        json.dumps(
                            stored,
                            sort_keys=True,
                            separators=(",", ":"),
                            ensure_ascii=True,
                        )
                        != values_json
                    ):
                        raise StudyIdentityConflict(study_name, stored, values)
        except StudyStorageError:
            raise
        except _storage_exception_types() as exc:
            raise _storage_error("identity coordination", redacted, exc) from None

        # Keep the session lock through create/load and recovery publication;
        # preserve caller exception types.
        yield
    finally:
        body_error = sys.exc_info()[1]
        release_error: BaseException | None = None
        if connection is not None:
            try:
                if acquired:
                    try:
                        _release_advisory_lock(connection, key, redacted, engine=engine)
                    except _CLEANUP_BASE_EXCEPTION as exc:
                        release_error = exc
            finally:
                try:
                    connection.close()
                except _CLEANUP_BASE_EXCEPTION as exc:
                    if release_error is None:
                        release_error = exc
        if release_error is not None and body_error is None:
            raise release_error


__all__ = (
    "RDB_CONNECT_TIMEOUT_S",
    "RDB_MAX_OVERFLOW",
    "RDB_POOL_RECYCLE_S",
    "RDB_POOL_SIZE",
    "RDB_POOL_TIMEOUT_S",
    "RDB_LOCK_TIMEOUT_S",
    "RDBPoolPolicy",
    "StudyIdentityConflict",
    "StudyStorageError",
    "advisory_lock_key",
    "coordinate_rdb_study",
    "dispose_rdb_storage",
    "make_rdb_storage",
    "rdb_engine_kwargs",
    "rdb_operation",
    "redact_storage_url",
    "remove_rdb_session",
    "validate_rdb_dsn",
)
