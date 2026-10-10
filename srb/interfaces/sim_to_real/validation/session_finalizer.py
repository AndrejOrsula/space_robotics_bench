"""Post-session finalization for sim-to-real validation sessions."""

from __future__ import annotations

import json
import logging
import shutil
import stat as stat_module
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

from srb.interfaces.sim_to_real.validation.artifact_reference import (
    BADGE_JSON,
    INDEX_HTML,
    REPORT_MD,
    SPEC_YAML,
    SUMMARY_JSON,
)
from srb.interfaces.sim_to_real.validation.comparator import evaluate
from srb.interfaces.sim_to_real.validation.criteria import (
    CriterionResult,
    session_is_incomplete,
)
from srb.interfaces.sim_to_real.validation.exit_codes import VALIDATION_EXIT_CODES
from srb.interfaces.sim_to_real.validation.github_status import post_github_status
from srb.interfaces.sim_to_real.validation.report import write_session_report
from srb.interfaces.sim_to_real.validation.report_index import write_validation_index
from srb.interfaces.sim_to_real.validation.result import SessionResult
from srb.interfaces.sim_to_real.validation.session_workspace import (
    ValidationSessionIdentity,
)
from srb.utils.failures import IntegrityFailureError
from srb.utils.publication import (
    ArtifactPublication,
    PublicationError,
    PublishedGeneration,
    copy_verified_file_snapshot,
)
from srb.utils.validation_session_log import EPISODES_JSONL
from srb.utils.validation_spec import ValidationSpec

_REQUIRED_SESSION_MEMBERS = (
    SPEC_YAML,
    EPISODES_JSONL,
    SUMMARY_JSON,
    REPORT_MD,
    BADGE_JSON,
)

logger = logging.getLogger(__name__)


class ValidationDriftStore(Protocol):
    path: Path

    def lookup(self, task: str, algo: str, *, last_k: int) -> list[dict[str, Any]]: ...

    def record(
        self,
        *,
        git_sha: str,
        timestamp: str,
        task: str,
        algo: str,
        session_dir: str,
        metrics: dict[str, float],
        exit_code: int,
        identity: ValidationSessionIdentity | None = None,
    ) -> None: ...


@dataclass(frozen=True)
class ValidationSessionFinalizer:
    """Finalize a validation session after rollout telemetry is closed."""

    spec: ValidationSpec
    drift_store: ValidationDriftStore
    session_dir: Path
    git_sha: str
    timestamp: str
    staging_dir: Path
    post_status: bool = False
    dry_run: bool = False
    identity: ValidationSessionIdentity | None = None
    published_session_dir: Path | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if not isinstance(self.staging_dir, Path):
            raise TypeError("staging_dir must be a pathlib.Path")
        if self.staging_dir.absolute() == self.session_dir.absolute():
            raise ValueError("staging_dir must be distinct from session_dir")

    def finalize(self) -> SessionResult:
        return self.publish(self.evaluate())

    def evaluate(self) -> SessionResult:
        """Evaluate the closed session without publishing anything.

        Raises on invalid evidence (baseline, drift history, episode log); the
        caller routes that to :meth:`finalize_invalid` so a report is always
        published.
        """

        return self._evaluate_result()

    def publish(self, result: SessionResult) -> SessionResult:
        """Publish an evaluated result: report, session, drift row, status."""

        return self._finalize_result(result)

    def finalize_preflight_failure(self, exc: Exception) -> SessionResult:
        """Finalize a session that failed before rollout could begin."""

        return self._finalize_result(self._preflight_failure_result(exc))

    def finalize_invalid(self, exc: Exception) -> SessionResult:
        """Publish an explicit non-hardware result for invalid session evidence."""

        return self._finalize_result(self._invalid_result(exc))

    def _evaluate_result(self) -> SessionResult:
        if self.dry_run:
            return self._dry_run_result()
        history = self.drift_store.lookup(
            self.spec.task,
            self.spec.algo,
            last_k=self.spec.drift_window,
        )
        return evaluate(self.spec, self._evaluation_dir, drift_history=history)

    def _finalize_result(self, result: SessionResult) -> SessionResult:
        self._write_report(result)
        self._validate_staging_result(result)
        # Publish evidence before indexes or status can reference it.
        published_dir = self._publish_session()
        record_kwargs: dict[str, Any] = {
            "git_sha": self.git_sha,
            "timestamp": self.timestamp,
            "task": self.spec.task,
            "algo": self.spec.algo,
            "session_dir": str(published_dir),
            "metrics": dict(result.metrics),
            "exit_code": result.exit_code,
        }
        if self.identity is not None:
            record_kwargs["identity"] = self.identity
        # Incomplete-session metrics do not represent the drift population.
        if result.hardware_evidence and not session_is_incomplete(
            self.spec, result.metrics
        ):
            self.drift_store.record(
                **record_kwargs,
            )
        self._write_index()
        # Only hardware results belong on the hardware status channel.
        if self.post_status and result.hardware_evidence:
            self._post_github_status(result)
        self._cleanup_staging()
        return result

    @property
    def _evaluation_dir(self) -> Path:
        return self.staging_dir

    def _write_report(self, result: SessionResult) -> None:
        if self.identity is None:
            write_session_report(self.spec, result, self._evaluation_dir)
        else:
            write_session_report(
                self.spec,
                result,
                self._evaluation_dir,
                identity=self.identity,
            )

    def _publish_session(self) -> Path:
        published = ArtifactPublication.generation(self.session_dir).publish(
            lambda destination: _copy_staging_members(self.staging_dir, destination),
            required_members=_REQUIRED_SESSION_MEMBERS,
        )
        if not isinstance(published, PublishedGeneration):
            raise PublicationError(
                "validation session publication returned no generation"
            )
        published_dir = Path(published.path)
        object.__setattr__(self, "published_session_dir", published_dir)
        return published_dir

    def _cleanup_staging(self) -> None:
        """Remove the private staging copy only after finalization succeeds."""

        staging_dir = self.staging_dir
        if (
            staging_dir.name != ".staging"
            or staging_dir.parent.absolute() != self.session_dir.absolute()
        ):
            logger.warning(
                "retaining unexpected validation staging path: %s", staging_dir
            )
            return
        if staging_dir.is_symlink():
            logger.warning(
                "retaining symlinked validation staging path: %s", staging_dir
            )
            return
        try:
            shutil.rmtree(staging_dir)
        except FileNotFoundError:
            return
        except OSError as exc:
            logger.warning(
                "failed to remove validation staging directory %s: %s", staging_dir, exc
            )

    def _validate_staging_result(self, result: SessionResult) -> None:
        """Ensure generated report members cannot contradict the result."""

        summary_path = self._evaluation_dir / SUMMARY_JSON
        report_path = self._evaluation_dir / REPORT_MD
        badge_path = self._evaluation_dir / BADGE_JSON
        if (
            not summary_path.is_file()
            or not report_path.is_file()
            or not badge_path.is_file()
        ):
            raise _integrity_failure(
                "validation report did not produce all required result members"
            )
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            badge = json.loads(badge_path.read_text(encoding="utf-8"))
            report = report_path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise _integrity_failure(
                "validation report members are not readable JSON/text", cause=exc
            ) from exc
        if not isinstance(summary, dict) or not isinstance(badge, dict):
            raise _integrity_failure("validation report payloads must be objects")
        if (
            type(summary.get("pass_overall")) is not bool
            or type(summary.get("hardware_evidence")) is not bool
            or type(summary.get("exit_code")) is not int
        ):
            raise _integrity_failure("validation summary has malformed result fields")
        if summary["pass_overall"] != result.pass_overall:
            raise _integrity_failure(
                "validation summary pass_overall disagrees with result"
            )
        if summary["hardware_evidence"] != result.hardware_evidence:
            raise _integrity_failure(
                "validation summary hardware_evidence disagrees with result"
            )
        if summary["exit_code"] != result.exit_code:
            raise _integrity_failure(
                "validation summary exit_code disagrees with result"
            )
        for name, criterion in (
            ("criterion_a", result.criterion_a),
            ("criterion_b", result.criterion_b),
            ("criterion_c", result.criterion_c),
        ):
            payload = summary.get(name)
            if (
                not isinstance(payload, dict)
                or payload.get("status") != criterion.status
            ):
                raise _integrity_failure(
                    f"validation summary {name} disagrees with result"
                )
        # Compare the dedicated result line only; criterion details elsewhere in
        # the report may carry arbitrary text, including a PASS marker.
        result_line = next(
            (line for line in report.splitlines() if line.startswith("**Result:** ")),
            None,
        )
        expected = "PASS" if result.pass_overall else "FAIL"
        if result_line != f"**Result:** {expected} (exit {result.exit_code})":
            raise _integrity_failure(
                "validation report PASS marker disagrees with result"
            )
        if result.pass_overall:
            if badge.get("color") != "green":
                raise _integrity_failure("passing validation badge is not green")
        elif badge.get("color") == "green":
            raise _integrity_failure("non-passing validation badge is green")

    def _write_index(self) -> None:
        write_validation_index(
            self.drift_store.path,
            Path(self.spec.storage_root) / INDEX_HTML,
            metric_keys=self.spec.metrics_to_track,
        )

    def _post_github_status(self, result: SessionResult) -> None:
        post_github_status(
            sha=self.git_sha,
            task=self.spec.task,
            algo=self.spec.algo,
            result=result,
        )

    def _dry_run_result(self) -> SessionResult:
        dry_run = CriterionResult(
            status="skipped",
            detail="dry-run: no hardware rollout; not a validation pass",
        )
        return SessionResult(
            pass_overall=False,
            criterion_a=dry_run,
            criterion_b=dry_run,
            criterion_c=dry_run,
            metrics={"n_episodes": 0.0, "crash_count": 0.0},
            sim_metrics=None,
            sim_real_ratio=None,
            drift_z=None,
            exit_code=_preflight_exit_code(),
            hardware_evidence=False,
        )

    def _preflight_failure_result(self, exc: Exception) -> SessionResult:
        skipped = CriterionResult(
            status="skipped",
            detail="preflight failed before rollout metrics were available",
        )
        failure = CriterionResult(
            status="fail",
            detail=f"preflight failed: {type(exc).__name__}: {exc}",
        )
        return SessionResult(
            pass_overall=False,
            criterion_a=skipped,
            criterion_b=failure,
            criterion_c=skipped,
            metrics={"n_episodes": 0.0, "crash_count": 0.0},
            sim_metrics=None,
            sim_real_ratio=None,
            drift_z=None,
            exit_code=_preflight_exit_code(),
            hardware_evidence=False,
        )

    def _invalid_result(self, exc: Exception) -> SessionResult:
        failure = CriterionResult(
            status="fail",
            detail=f"invalid validation evidence: {type(exc).__name__}: {exc}",
        )
        return SessionResult(
            pass_overall=False,
            criterion_a=failure,
            criterion_b=failure,
            criterion_c=failure,
            metrics={"n_episodes": 0.0, "crash_count": 0.0},
            sim_metrics=None,
            sim_real_ratio=None,
            drift_z=None,
            exit_code=_preflight_exit_code(),
            hardware_evidence=False,
        )


def _preflight_exit_code() -> int:
    for exit_code in VALIDATION_EXIT_CODES:
        if exit_code.name == "preflight_fail":
            return int(exit_code.code)
    raise RuntimeError("validation exit code table is missing preflight_fail")


def _copy_staging_members(source: Path, destination: Path) -> None:
    if not source.is_dir():
        raise PublicationError(f"validation staging directory is missing: {source}")
    members = sorted(
        source.rglob("*"),
        key=lambda path: path.relative_to(source).as_posix(),
    )
    for path in members:
        try:
            path_stat = path.lstat()
        except OSError as exc:
            raise PublicationError(
                f"cannot inspect validation staging member: {path}"
            ) from exc
        if stat_module.S_ISLNK(path_stat.st_mode):
            raise PublicationError(f"validation staging contains symlink: {path}")
        if stat_module.S_ISDIR(path_stat.st_mode):
            continue
        if not stat_module.S_ISREG(path_stat.st_mode):
            raise PublicationError(
                f"validation staging contains non-regular member: {path}"
            )
        if path.name.endswith(".publication.lock") or path.name.endswith(
            ".publication.sha256"
        ):
            continue
        relative = path.relative_to(source)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        copy_verified_file_snapshot(
            path,
            target,
            expected_source_stat=path_stat,
        )


def _integrity_failure(
    message: str,
    *,
    cause: BaseException | None = None,
) -> IntegrityFailureError:
    details: dict[str, object] = {}
    if cause is not None:
        details["cause"] = type(cause).__name__
    return IntegrityFailureError(
        code="validation.finalization.integrity",
        message=message,
        component="validation.session_finalizer",
        details=details,
        evidence_valid=False,
    )
