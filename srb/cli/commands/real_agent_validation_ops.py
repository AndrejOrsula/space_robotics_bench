"""CLI adapters for validation operations under ``srb real_agent``."""

from __future__ import annotations

import json
import logging
import shutil
import tempfile
from pathlib import Path
from typing import Any

from srb.interfaces.sim_to_real.validation.artifact_reference import (
    BADGE_JSON,
    REPORT_MD,
    ROLLOUT_RRD,
    SPEC_YAML,
    SUMMARY_JSON,
)
from srb.interfaces.sim_to_real.validation.session_workspace import (
    ValidationSessionIdentity,
)
from srb.utils.publication import GenerationPublication, PublicationError
from srb.utils.validation_session_log import EPISODES_JSONL

_REQUIRED_SESSION_MEMBERS = (
    SPEC_YAML,
    EPISODES_JSONL,
    SUMMARY_JSON,
    REPORT_MD,
    BADGE_JSON,
)
# Rebuild regenerates the result members, so a source needs only its inputs.
_REBUILD_INPUT_MEMBERS = (SPEC_YAML, EPISODES_JSONL)


def execute_rebuild_summary_command(*, session_dir: str) -> int:
    """Rebuild validation summary artifacts for an existing session."""

    session = Path(session_dir)

    from srb.interfaces.sim_to_real.validation.comparator import evaluate
    from srb.interfaces.sim_to_real.validation.drift import DriftStore
    from srb.interfaces.sim_to_real.validation.report import write_session_report
    from srb.utils.failures import IntegrityFailureError
    from srb.utils.validation_spec import load_validation_spec

    try:
        publication_root, source, is_published = _resolve_rebuild_source(session)
        spec_path = source / SPEC_YAML
        if not spec_path.is_file():
            logging.error("spec.yaml not present in session evidence; cannot rebuild.")
            return 4

        spec = load_validation_spec(spec_path)
        drift = DriftStore(spec.storage_root)
        session_paths = {
            session.resolve(),
            source.resolve(),
            publication_root.resolve(),
        }
        history = [
            row
            for row in drift.lookup(
                spec.task,
                spec.algo,
                last_k=spec.drift_window + 1,
            )
            if not (
                (stored_path := row.get("session_dir"))
                and Path(stored_path).resolve() in session_paths
            )
        ][-spec.drift_window :]

        # Work on a private copy; only publication writes back to the session.
        with tempfile.TemporaryDirectory(
            prefix=f".{session.name}.rebuild-",
            dir=str(session.parent),
        ) as temporary:
            staging = Path(temporary)
            _copy_session_members(source, staging)
            result = evaluate(spec, staging, drift_history=history)
            identity = (
                _load_session_identity(staging / SUMMARY_JSON)
                if is_published and (staging / SUMMARY_JSON).is_file()
                else None
            )

            def build_rebuilt_generation(destination: Path) -> None:
                _copy_session_members(staging, destination)
                if identity is None:
                    write_session_report(spec, result, destination)
                else:
                    write_session_report(
                        spec,
                        result,
                        destination,
                        identity=identity,
                    )

            GenerationPublication(publication_root).publish(
                build_rebuilt_generation,
                required_members=_REQUIRED_SESSION_MEMBERS,
            )
    except (IntegrityFailureError, PublicationError, OSError, ValueError) as exc:
        logging.error("cannot publish rebuilt validation report: %s", exc)
        return 4
    return int(result.exit_code)


def _resolve_rebuild_source(session: Path) -> tuple[Path, Path, bool]:
    """Resolve a validated published source or retain a legacy source path."""

    if (session / "current.json").is_file():
        publication = GenerationPublication(session)
        current = publication.open(required_members=_REBUILD_INPUT_MEMBERS)
        return session, current.path, True

    candidate_root = session.parent.parent
    if (candidate_root / "current.json").is_file():
        publication = GenerationPublication(candidate_root)
        current = publication.open(required_members=_REBUILD_INPUT_MEMBERS)
        if current.path.resolve() != session.resolve():
            raise PublicationError(
                f"session is not the current published generation: {session}"
            )
        return candidate_root, current.path, True

    # Treat legacy evidence as read-only input under its own publication root.
    return session, session, False


def _load_session_identity(path: Path) -> ValidationSessionIdentity:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PublicationError(
            f"cannot read published session identity: {path}"
        ) from exc
    if not isinstance(payload, dict):
        raise PublicationError(f"published session summary is not an object: {path}")
    identity = payload.get("session_identity")
    if not isinstance(identity, dict):
        raise PublicationError(f"published session identity is missing: {path}")
    fields = (
        "session_id",
        "task",
        "algorithm",
        "code_sha",
        "config_digest",
        "artifact_digest",
    )
    if any(not isinstance(identity.get(field), str) for field in fields):
        raise PublicationError(f"published session identity is invalid: {path}")
    return ValidationSessionIdentity(
        session_id=identity["session_id"],
        task=identity["task"],
        algorithm=identity["algorithm"],
        code_sha=identity["code_sha"],
        config_digest=identity["config_digest"],
        artifact_digest=identity["artifact_digest"],
    )


def _copy_session_members(source: Path, destination: Path) -> None:
    if not source.is_dir():
        raise PublicationError(f"session evidence directory is missing: {source}")
    for path in source.rglob("*"):
        if path.is_symlink():
            raise PublicationError(f"published session contains symlink: {path}")
        if not path.is_file() or path.name in {
            ".srb-publication.json",
            ".publication.lock",
            ".publication.sha256",
        }:
            continue
        target = destination / path.relative_to(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)


def execute_replay_twin_command(*, session: str, baseline: str) -> int:
    """Open a Rerun replay comparing a validation session to a baseline."""

    session_rrd = Path(session) / ROLLOUT_RRD
    baseline_rrd = Path(baseline)
    for label, path in (("session", session_rrd), ("baseline", baseline_rrd)):
        if not path.is_file():
            logging.error("%s replay file not found: %s", label, path)
            return 4

    try:
        rr = _load_rerun()
    except ImportError:
        logging.error("rerun-sdk not installed.")
        return 4

    rr.init("srb-replay-twin", spawn=True)
    try:
        rr.log_file_from_path(str(session_rrd))
        rr.log_file_from_path(str(baseline_rrd))
    except AttributeError:
        logging.error("rerun-sdk does not expose log_file_from_path")
        return 4
    except Exception as exc:
        logging.error("cannot load replay files: %s", exc)
        return 4
    return 0


def _load_rerun() -> Any:
    """Import the Rerun SDK lazily so bare CLI imports stay lightweight."""

    import rerun as rr

    return rr


__all__ = [
    "execute_rebuild_summary_command",
    "execute_replay_twin_command",
]
