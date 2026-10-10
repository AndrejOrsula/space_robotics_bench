"""Event-camera run artifacts built around ``events_manifest.jsonl``."""

from __future__ import annotations

import json
import operator
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

from srb.core.sensor.event_camera.manifest import (
    EVENTS_MANIFEST_FILENAME,
    EventManifestRow,
    _require_event_sha256,
    append_event_manifest_row,
    append_event_manifest_rows,
    create_event_manifest,
    parse_event_reward_summary,
)
from srb.utils.artifact_paths import (
    resolve_artifact_member,
    validate_artifact_segment,
)
from srb.utils.policy_provenance import is_checkpoint_free_policy
from srb.utils.publication import ArtifactPublication

_REPRODUCIBILITY_WARNING_FIELDS = (
    "policy_checkpoint_hash",
    "srb_git_sha",
    "isaaclab_version",
)
_EVENT_HDF5_PROVENANCE_FIELDS = (
    "sensor_preset",
    "resolution",
    "sub_render_hz",
    "backend_name",
    "backend_version",
    "backend_commit_sha",
    "representation",
    "time_bins",
    "episode_id",
    "policy_id",
    "policy_checkpoint_hash",
    "task_id",
    "seed",
    "srb_git_sha",
    "isaaclab_version",
    "rgb_mp4_path",
    "success_flag",
    "reward_summary",
    "hot_pixel_mask",
    "hf_repo_ids",
    "hot_pixel_rate_hz",
)
_DEFAULT_HDF5_VALIDATION_CHUNK_SIZE = 4096
_EVENT_SHA256_HEX = frozenset("0123456789abcdefABCDEF")


@dataclass(frozen=True)
class EventCameraRunSummary:
    """Aggregate metadata for a selected event-camera run artifact."""

    sensors: tuple[str, ...]
    total_event_count: int
    first_row: EventManifestRow | None
    sensor_presets: tuple[str, ...]
    task_ids: tuple[str, ...]
    policy_ids: tuple[str, ...]
    hf_repo_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class EventCameraRunSelection:
    """Selected manifest rows plus their validated HDF5 paths."""

    root: Path
    manifest_path: Path
    rows: tuple[EventManifestRow, ...]
    hdf5_paths: tuple[Path, ...]

    @property
    def sensors(self) -> tuple[str, ...]:
        return tuple(sorted({row.sensor_name for row in self.rows}))

    @property
    def total_event_count(self) -> int:
        return sum(row.event_count for row in self.rows)

    @property
    def hf_repo_ids(self) -> tuple[str, ...]:
        """Sorted unique repo ids, skipping rows without task or preset ids."""
        return _hf_repo_ids_for_rows(self.rows)

    def summary(self) -> EventCameraRunSummary:
        return EventCameraRunSummary(
            sensors=self.sensors,
            total_event_count=self.total_event_count,
            first_row=self.rows[0] if self.rows else None,
            sensor_presets=tuple(sorted({row.sensor_preset for row in self.rows})),
            task_ids=tuple(sorted({row.task_id for row in self.rows})),
            policy_ids=tuple(sorted({row.policy_id for row in self.rows})),
            hf_repo_ids=self.hf_repo_ids,
        )


def _hf_repo_ids_for_rows(
    rows: Iterable[EventManifestRow],
) -> tuple[str, ...]:
    """Derive the sorted-unique HF repo ids implied by the given manifest rows."""
    from srb.datasets.event_camera_export import event_camera_hf_repo_id

    ids: set[str] = set()
    for row in rows:
        if not row.task_id or not row.sensor_preset:
            continue
        ids.add(event_camera_hf_repo_id(row.task_id, row.sensor_preset))
    return tuple(sorted(ids))


@dataclass(frozen=True)
class EventCameraRunValidationIssue:
    """One validation problem in an event-camera run manifest row."""

    field: str
    message: str
    row_index: int
    episode_id: int
    sensor_name: str
    event_hdf5_path: str
    line_number: int | None = None
    severity: Literal["error", "warning"] = "error"

    def to_dict(self) -> dict[str, Any]:
        return {
            "field": self.field,
            "message": self.message,
            "row_index": self.row_index,
            "episode_id": self.episode_id,
            "sensor_name": self.sensor_name,
            "event_hdf5_path": self.event_hdf5_path,
            "line_number": self.line_number,
            "severity": self.severity,
        }


@dataclass(frozen=True)
class EventCameraRunValidationReport:
    """Validation report for one event-camera run artifact."""

    root: Path
    manifest_path: Path
    row_count: int
    issues: tuple[EventCameraRunValidationIssue, ...] = ()
    strict: bool = False

    @property
    def error_issues(self) -> tuple[EventCameraRunValidationIssue, ...]:
        return tuple(issue for issue in self.issues if issue.severity == "error")

    @property
    def warning_issues(self) -> tuple[EventCameraRunValidationIssue, ...]:
        return tuple(issue for issue in self.issues if issue.severity == "warning")

    @property
    def ok(self) -> bool:
        return self.valid

    @property
    def valid(self) -> bool:
        return not self.errors

    @property
    def errors(self) -> list[str]:
        errors = [issue.message for issue in self.error_issues]
        if self.strict:
            errors.extend(
                f"warning treated as error: {issue.message}"
                for issue in self.warning_issues
            )
        return errors

    @property
    def warnings(self) -> list[str]:
        return [issue.message for issue in self.warning_issues]

    def raise_for_errors(self) -> None:
        if self.valid:
            return
        summaries = [_format_issue_summary(issue) for issue in self.error_issues]
        if self.strict:
            summaries.extend(
                "warning treated as error: " + _format_issue_summary(issue)
                for issue in self.warning_issues
            )
        preview = "; ".join(summaries[:3])
        extra = "" if len(summaries) <= 3 else f"; +{len(summaries) - 3} more"
        raise ValueError(
            "event-camera run artifact validation failed for "
            f"{self.manifest_path}: {preview}{extra}"
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "artifact_type": "event_camera_run",
            "valid": self.valid,
            "errors": self.errors,
            "warnings": self.warnings,
            "root": str(self.root),
            "manifest": str(self.manifest_path),
            "row_count": self.row_count,
            "error_count": len(self.error_issues),
            "warning_count": len(self.warning_issues),
            "issue_count": len(self.issues),
            "strict": self.strict,
            "issues": [issue.to_dict() for issue in self.issues],
        }


@dataclass(frozen=True)
class _EventManifestSelection:
    requested: tuple[str, ...] | None
    indexed_rows: tuple[tuple[int, EventManifestRow], ...]
    missing: tuple[str, ...] = ()

    @property
    def rows(self) -> tuple[EventManifestRow, ...]:
        return tuple(row for _, row in self.indexed_rows)

    @property
    def requested_label(self) -> list[str] | None:
        if self.requested is None:
            return None
        return list(self.requested)


@dataclass(frozen=True)
class EventCameraRunArtifact:
    """A timestamped EVENT_HDF5 run directory."""

    root: Path
    manifest_path: Path
    rows: tuple[EventManifestRow, ...]

    @classmethod
    def load(
        cls,
        root: Path | str,
        *,
        require_manifest: bool = True,
    ) -> "EventCameraRunArtifact":
        run_root = Path(root)
        if not run_root.is_dir():
            raise FileNotFoundError(f"event_hdf5_dir must exist: {run_root}")

        manifest_path = run_root / EVENTS_MANIFEST_FILENAME
        if not manifest_path.exists():
            if require_manifest:
                raise FileNotFoundError(
                    f"{EVENTS_MANIFEST_FILENAME} required at "
                    f"{manifest_path}; this is the canonical "
                    "cross-episode index written by EventRecorder."
                )
            return cls(root=run_root, manifest_path=manifest_path, rows=())

        rows = tuple(_read_manifest_rows(manifest_path))
        return cls(root=run_root, manifest_path=manifest_path, rows=rows)

    @classmethod
    def validate_path(
        cls,
        root: Path | str,
        *,
        episodes: Iterable[str] | None = None,
        validate_hdf5: bool = True,
        validate_hdf5_schema: bool = True,
        validate_reward_summary: bool = True,
        require_manifest: bool = True,
        strict: bool = False,
        hdf5_chunk_size: int = _DEFAULT_HDF5_VALIDATION_CHUNK_SIZE,
        allow_legacy_event_hdf5: bool = False,
    ) -> EventCameraRunValidationReport:
        """Validate a run artifact without raising on malformed rows."""
        run_root = Path(root)
        manifest_path = run_root / EVENTS_MANIFEST_FILENAME
        issues: list[EventCameraRunValidationIssue] = []
        chunk_bound = _validation_chunk_bound(
            hdf5_chunk_size,
            issues=issues,
            enabled=validate_hdf5 and validate_hdf5_schema,
        )

        if not run_root.is_dir():
            issues.append(
                _manifest_issue(
                    field="root",
                    message=f"event_hdf5_dir must exist: {run_root}",
                    row_index=-1,
                )
            )
            return EventCameraRunValidationReport(
                root=run_root,
                manifest_path=manifest_path,
                row_count=0,
                issues=tuple(issues),
                strict=strict,
            )

        if not manifest_path.exists():
            if require_manifest:
                issues.append(
                    _manifest_issue(
                        field="events_manifest_jsonl",
                        message=(
                            f"{EVENTS_MANIFEST_FILENAME} required at "
                            f"{manifest_path}; this is the canonical "
                            "cross-episode index written by EventRecorder."
                        ),
                        row_index=-1,
                    )
                )
            return EventCameraRunValidationReport(
                root=run_root,
                manifest_path=manifest_path,
                row_count=0,
                issues=tuple(issues),
                strict=strict,
            )

        indexed_rows, manifest_issues, row_count = _read_manifest_rows_for_validation(
            manifest_path
        )
        selected_rows, selection_issues = _select_indexed_rows_for_validation(
            manifest_path=manifest_path,
            indexed_rows=indexed_rows,
            episodes=episodes,
        )
        issues.extend(manifest_issues)
        issues.extend(selection_issues)
        issues.extend(
            _validate_manifest_rows(
                root=run_root,
                manifest_path=manifest_path,
                indexed_rows=selected_rows,
                validate_hdf5=validate_hdf5,
                validate_hdf5_schema=validate_hdf5_schema,
                validate_reward_summary=validate_reward_summary,
                hdf5_chunk_size=chunk_bound,
                allow_legacy_event_hdf5=allow_legacy_event_hdf5,
            )
        )
        if episodes is None:
            issues.extend(
                _validate_orphan_event_hdf5_paths(
                    root=run_root,
                    manifest_path=manifest_path,
                    indexed_rows=selected_rows,
                )
            )
        return EventCameraRunValidationReport(
            root=run_root,
            manifest_path=manifest_path,
            row_count=row_count,
            issues=tuple(issues),
            strict=strict,
        )

    @property
    def has_manifest(self) -> bool:
        return self.manifest_path.exists()

    def select(
        self,
        episodes: Iterable[str] | None = None,
        *,
        validate_hdf5: bool = True,
    ) -> EventCameraRunSelection:
        selection = _select_manifest_rows(
            indexed_rows=tuple(enumerate(self.rows)),
            episodes=episodes,
        )
        rows = selection.rows

        if selection.missing:
            raise ValueError(
                f"manifest {self.manifest_path} does not contain requested "
                f"episodes: {list(selection.missing)}"
            )

        if not rows:
            raise ValueError(
                f"no manifest rows selected from {self.manifest_path}; "
                f"requested episodes={selection.requested_label}"
            )

        hdf5_paths = tuple(row.hdf5_path(self.root) for row in rows)
        if validate_hdf5:
            _validate_hdf5_paths(self.manifest_path, hdf5_paths)

        return EventCameraRunSelection(
            root=self.root,
            manifest_path=self.manifest_path,
            rows=rows,
            hdf5_paths=hdf5_paths,
        )

    def validate(
        self,
        *,
        episodes: Iterable[str] | None = None,
        validate_hdf5: bool = True,
        validate_hdf5_schema: bool = True,
        validate_reward_summary: bool = True,
        strict: bool = False,
        hdf5_chunk_size: int = _DEFAULT_HDF5_VALIDATION_CHUNK_SIZE,
        allow_legacy_event_hdf5: bool = False,
    ) -> EventCameraRunValidationReport:
        """Validate manifest row references without raising on row failures."""
        bound_issues: list[EventCameraRunValidationIssue] = []
        chunk_bound = _validation_chunk_bound(
            hdf5_chunk_size,
            issues=bound_issues,
            enabled=validate_hdf5 and validate_hdf5_schema,
        )
        indexed_rows, selection_issues = _select_indexed_rows_for_validation(
            manifest_path=self.manifest_path,
            indexed_rows=tuple(enumerate(self.rows)),
            episodes=episodes,
        )
        issues = [*bound_issues, *selection_issues]
        issues.extend(
            _validate_manifest_rows(
                root=self.root,
                manifest_path=self.manifest_path,
                indexed_rows=indexed_rows,
                validate_hdf5=validate_hdf5,
                validate_hdf5_schema=validate_hdf5_schema,
                validate_reward_summary=validate_reward_summary,
                hdf5_chunk_size=chunk_bound,
                allow_legacy_event_hdf5=allow_legacy_event_hdf5,
            )
        )
        if episodes is None:
            issues.extend(
                _validate_orphan_event_hdf5_paths(
                    root=self.root,
                    manifest_path=self.manifest_path,
                    indexed_rows=indexed_rows,
                )
            )
        return EventCameraRunValidationReport(
            root=self.root,
            manifest_path=self.manifest_path,
            row_count=len(self.rows),
            issues=tuple(issues),
            strict=strict,
        )


def _normalize_requested_episodes(
    episodes: Iterable[str] | None,
) -> tuple[str, ...] | None:
    if episodes is None:
        return None
    return tuple(str(episode) for episode in episodes)


def _select_manifest_rows(
    *,
    indexed_rows: Iterable[tuple[int, EventManifestRow]],
    episodes: Iterable[str] | None,
) -> _EventManifestSelection:
    rows = tuple(indexed_rows)
    requested = _normalize_requested_episodes(episodes)
    if requested is None:
        return _EventManifestSelection(
            requested=None,
            indexed_rows=rows,
        )

    requested_set = set(requested)
    selected_rows = tuple(
        (row_index, row)
        for row_index, row in rows
        if row.event_hdf5_path in requested_set
    )
    selected_paths = {row.event_hdf5_path for _, row in selected_rows}
    missing = tuple(episode for episode in requested if episode not in selected_paths)
    return _EventManifestSelection(
        requested=requested,
        indexed_rows=selected_rows,
        missing=missing,
    )


def _select_indexed_rows_for_validation(
    *,
    manifest_path: Path,
    indexed_rows: Iterable[tuple[int, EventManifestRow]],
    episodes: Iterable[str] | None,
) -> tuple[
    tuple[tuple[int, EventManifestRow], ...],
    tuple[EventCameraRunValidationIssue, ...],
]:
    selection = _select_manifest_rows(
        indexed_rows=indexed_rows,
        episodes=episodes,
    )
    if selection.requested is None:
        return selection.indexed_rows, ()

    if not selection.requested or selection.missing:
        if selection.indexed_rows:
            message = (
                f"manifest {manifest_path} does not contain requested "
                f"episodes: {list(selection.missing)}"
            )
        else:
            message = (
                f"no manifest rows selected from {manifest_path}; "
                f"requested episodes={selection.requested_label}"
            )
        return selection.indexed_rows, (
            _manifest_issue(
                field="episodes",
                message=message,
                row_index=-1,
            ),
        )
    return selection.indexed_rows, ()


def _read_manifest_rows(manifest_path: Path) -> list[EventManifestRow]:
    rows: list[EventManifestRow] = []
    for line in _read_manifest_text(manifest_path).splitlines():
        line = line.strip()
        if not line:
            continue
        rows.append(EventManifestRow.from_mapping(json.loads(line)))
    return rows


def _read_manifest_rows_for_validation(
    manifest_path: Path,
) -> tuple[
    list[tuple[int, EventManifestRow]],
    list[EventCameraRunValidationIssue],
    int,
]:
    indexed_rows: list[tuple[int, EventManifestRow]] = []
    issues: list[EventCameraRunValidationIssue] = []
    row_index = 0
    try:
        manifest_text = _read_manifest_text(manifest_path)
    except UnicodeDecodeError as exc:
        return (
            [],
            [
                _manifest_issue(
                    field="events_manifest_jsonl",
                    message=f"manifest {manifest_path} is not valid UTF-8: {exc}",
                    row_index=-1,
                )
            ],
            0,
        )
    for line_number, raw_line in enumerate(manifest_text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as exc:
            issues.append(
                _manifest_issue(
                    field="events_manifest_jsonl",
                    message=(
                        f"manifest {manifest_path} line {line_number} "
                        f"row {row_index} has invalid JSON: {exc.msg}"
                    ),
                    row_index=row_index,
                    line_number=line_number,
                )
            )
            row_index += 1
            continue
        if not isinstance(payload, Mapping):
            issues.append(
                _manifest_issue(
                    field="events_manifest_jsonl",
                    message=(
                        f"manifest {manifest_path} line {line_number} "
                        f"row {row_index} must be a JSON object"
                    ),
                    row_index=row_index,
                    line_number=line_number,
                )
            )
            row_index += 1
            continue
        try:
            indexed_rows.append((row_index, EventManifestRow.from_mapping(payload)))
        # JSON ``Infinity`` reaches ``int()`` as an OverflowError.
        except (TypeError, ValueError, OverflowError) as exc:
            issues.append(
                _manifest_issue(
                    field="events_manifest_jsonl",
                    message=(
                        f"manifest {manifest_path} line {line_number} "
                        f"row {row_index} has invalid schema: {exc}"
                    ),
                    row_index=row_index,
                    line_number=line_number,
                )
            )
        row_index += 1
    return indexed_rows, issues, row_index


def _read_manifest_text(manifest_path: Path) -> str:
    payload = ArtifactPublication.file(manifest_path).read(
        lambda raw: raw.decode("utf-8")
    )
    if payload is None:
        raise FileNotFoundError(f"event manifest does not exist: {manifest_path}")
    return payload


def _validation_chunk_bound(
    value: object,
    *,
    issues: list[EventCameraRunValidationIssue],
    enabled: bool,
) -> int:
    if not enabled:
        return _DEFAULT_HDF5_VALIDATION_CHUNK_SIZE
    try:
        if isinstance(value, bool):
            raise TypeError("bool is not a chunk bound")
        bound = operator.index(value)
    except TypeError:
        bound = 0
    if bound > 0:
        return bound
    issues.append(
        _manifest_issue(
            field="hdf5_chunk_size",
            message=(
                "EVENT_HDF5 validation chunk bound must be a positive integer; "
                f"got {value!r}"
            ),
            row_index=-1,
        )
    )
    # Keep other checks deterministic; per-row validators require a valid bound.
    return 1


def _validate_manifest_rows(
    *,
    root: Path,
    manifest_path: Path,
    indexed_rows: Iterable[tuple[int, EventManifestRow]],
    validate_hdf5: bool,
    validate_hdf5_schema: bool,
    validate_reward_summary: bool,
    hdf5_chunk_size: int,
    allow_legacy_event_hdf5: bool,
) -> list[EventCameraRunValidationIssue]:
    issues: list[EventCameraRunValidationIssue] = []
    seen_episode_keys: set[tuple[int, int, str]] = set()
    seen_hdf5_paths: set[Path] = set()
    for row_index, row in indexed_rows:
        episode_key = (row.env_id, row.episode_id, row.sensor_name)
        if episode_key in seen_episode_keys:
            issues.append(
                _validation_issue(
                    row,
                    row_index=row_index,
                    field="episode_identity",
                    message=(
                        f"manifest {manifest_path} row {row_index} duplicates "
                        "the (env_id, episode_id, sensor_name) identity "
                        f"{episode_key!r}"
                    ),
                )
            )
        else:
            seen_episode_keys.add(episode_key)
        try:
            validate_artifact_segment(row.sensor_name, field="sensor_name")
            hdf5_path = row.hdf5_path(root)
            if hdf5_path in seen_hdf5_paths:
                issues.append(
                    _validation_issue(
                        row,
                        row_index=row_index,
                        field="event_hdf5_path",
                        message=(
                            f"manifest {manifest_path} row {row_index} reuses "
                            f"event HDF5 {row.event_hdf5_path!r}"
                        ),
                    )
                )
            seen_hdf5_paths.add(hdf5_path)
            rgb_mp4_path: Path | None = None
            if row.rgb_mp4_path:
                rgb_mp4_path = resolve_artifact_member(
                    root,
                    row.rgb_mp4_path,
                    field="rgb_mp4_path",
                )
            if row.event_sha256:
                _require_event_sha256(row.event_sha256)
        except (TypeError, ValueError) as exc:
            issues.append(
                _validation_issue(
                    row,
                    row_index=row_index,
                    field="artifact_path",
                    message=(
                        f"manifest {manifest_path} row {row_index} has an "
                        f"unsafe artifact reference: {exc}"
                    ),
                )
            )
            continue
        if rgb_mp4_path is not None and not rgb_mp4_path.is_file():
            issues.append(
                _validation_issue(
                    row,
                    row_index=row_index,
                    field="rgb_mp4_path",
                    message=(
                        f"manifest {manifest_path} row {row_index} "
                        f"references missing MP4 {rgb_mp4_path}"
                    ),
                )
            )
        if validate_hdf5:
            if not hdf5_path.is_file():
                issues.append(
                    _validation_issue(
                        row,
                        row_index=row_index,
                        field="event_hdf5_path",
                        message=(
                            f"manifest {manifest_path} row {row_index} "
                            f"references missing HDF5 {hdf5_path}"
                        ),
                    )
                )
            elif validate_hdf5_schema:
                _extend_hdf5_schema_issues(
                    issues,
                    root=root,
                    manifest_path=manifest_path,
                    row_index=row_index,
                    row=row,
                    hdf5_chunk_size=hdf5_chunk_size,
                    allow_legacy_event_hdf5=allow_legacy_event_hdf5,
                )
        if validate_reward_summary:
            try:
                row.reward_summary_payload()
            except ValueError as exc:
                issues.append(
                    _validation_issue(
                        row,
                        row_index=row_index,
                        field="reward_summary",
                        message=(
                            f"manifest {manifest_path} row {row_index} "
                            f"has invalid reward_summary: {exc}"
                        ),
                    )
                )
        for field in _REPRODUCIBILITY_WARNING_FIELDS:
            if field == "policy_checkpoint_hash" and is_checkpoint_free_policy(
                row.policy_id
            ):
                continue
            if not str(getattr(row, field)).strip():
                issues.append(
                    _validation_issue(
                        row,
                        row_index=row_index,
                        field=field,
                        message=(
                            f"manifest {manifest_path} row {row_index} "
                            f"has empty {field}; reproducibility metadata "
                            "is incomplete"
                        ),
                        severity="warning",
                    )
                )
    return issues


def _validate_orphan_event_hdf5_paths(
    *,
    root: Path,
    manifest_path: Path,
    indexed_rows: Iterable[tuple[int, EventManifestRow]],
) -> list[EventCameraRunValidationIssue]:
    """Reject published HDF5 files that are absent from the manifest index."""

    referenced: set[Path] = set()
    for _row_index, row in indexed_rows:
        try:
            referenced.add(row.hdf5_path(root).resolve())
        except (OSError, TypeError, ValueError):
            continue

    issues: list[EventCameraRunValidationIssue] = []
    candidates = {
        candidate
        for candidate in root.rglob("*")
        if candidate.suffix.lower() in {".h5", ".hdf5"}
    }
    for candidate in candidates:
        if not candidate.is_file() or candidate.resolve() in referenced:
            continue
        relative = candidate.relative_to(root).as_posix()
        issues.append(
            _manifest_issue(
                field="orphan_event_hdf5",
                message=(
                    f"manifest {manifest_path} does not reference EVENT_HDF5 "
                    f"file {relative}"
                ),
                row_index=-1,
            )
        )
    return issues


def _extend_hdf5_schema_issues(
    issues: list[EventCameraRunValidationIssue],
    *,
    root: Path,
    manifest_path: Path,
    row_index: int,
    row: EventManifestRow,
    hdf5_chunk_size: int,
    allow_legacy_event_hdf5: bool,
) -> None:
    from srb.core.sensor.event_camera.storage_validation import (
        validate_event_hdf5_schema,
    )

    hdf5_path = row.hdf5_path(root)
    report = validate_event_hdf5_schema(
        hdf5_path,
        chunk_size=hdf5_chunk_size,
        allow_legacy_without_digest=allow_legacy_event_hdf5,
    )
    for detail in report.issues:
        issues.append(
            _validation_issue(
                row,
                row_index=row_index,
                field="event_hdf5_schema",
                message=(
                    f"manifest {manifest_path} row {row_index} "
                    f"references invalid EVENT_HDF5 {hdf5_path}: {detail}"
                ),
            )
        )

    # Compare all provenance for current rows. Legacy rows may omit it only
    # when explicitly allowed and lacking an event digest.
    if report.valid and (row.event_sha256 or not allow_legacy_event_hdf5):
        hdf5_row = EventManifestRow.from_mapping(report.metadata)
        for field in _EVENT_HDF5_PROVENANCE_FIELDS:
            if getattr(row, field) != getattr(hdf5_row, field):
                issues.append(
                    _validation_issue(
                        row,
                        row_index=row_index,
                        field="event_hdf5_provenance",
                        message=(
                            f"manifest {manifest_path} row {row_index} "
                            f"{field} does not match EVENT_HDF5 metadata"
                        ),
                    )
                )

    if not row.event_sha256 and not allow_legacy_event_hdf5:
        issues.append(
            _validation_issue(
                row,
                row_index=row_index,
                field="event_sha256",
                message=(
                    f"manifest {manifest_path} row {row_index} does not bind "
                    "its EVENT_HDF5 payload with event_sha256; legacy rows "
                    "require allow_legacy_event_hdf5"
                ),
            )
        )
    if row.event_sha256:
        if report.metadata_event_sha256 is None:
            issues.append(
                _validation_issue(
                    row,
                    row_index=row_index,
                    field="event_sha256",
                    message=(
                        f"manifest {manifest_path} row {row_index} supplies "
                        "event_sha256 but EVENT_HDF5 metadata does not"
                    ),
                )
            )
        elif row.event_sha256.lower() != report.metadata_event_sha256.lower():
            issues.append(
                _validation_issue(
                    row,
                    row_index=row_index,
                    field="event_sha256",
                    message=(
                        f"manifest {manifest_path} row {row_index} event_sha256 "
                        "does not match EVENT_HDF5 metadata"
                    ),
                )
            )

    if report.metadata_event_count is not None:
        if row.event_count != report.metadata_event_count:
            issues.append(
                _validation_issue(
                    row,
                    row_index=row_index,
                    field="event_hdf5_schema",
                    message=(
                        f"manifest {manifest_path} row {row_index} "
                        f"references invalid EVENT_HDF5 {hdf5_path}: "
                        f"manifest event_count {row.event_count} does not "
                        f"match EVENT_HDF5 event_count "
                        f"{report.metadata_event_count}"
                    ),
                )
            )
        return

    if report.event_count is not None and row.event_count != report.event_count:
        issues.append(
            _validation_issue(
                row,
                row_index=row_index,
                field="event_hdf5_schema",
                message=(
                    f"manifest {manifest_path} row {row_index} "
                    f"references invalid EVENT_HDF5 {hdf5_path}: "
                    f"manifest event_count {row.event_count} does not match "
                    f"events/t length {report.event_count}"
                ),
            )
        )


def _validate_hdf5_paths(manifest_path: Path, hdf5_paths: tuple[Path, ...]) -> None:
    for hdf5_path in hdf5_paths:
        if not hdf5_path.is_file():
            raise FileNotFoundError(
                f"manifest {manifest_path} references missing HDF5 "
                f"{hdf5_path}; run artifact is incomplete."
            )


def _validation_issue(
    row: EventManifestRow,
    *,
    row_index: int,
    field: str,
    message: str,
    severity: Literal["error", "warning"] = "error",
) -> EventCameraRunValidationIssue:
    return EventCameraRunValidationIssue(
        field=field,
        message=message,
        row_index=row_index,
        episode_id=row.episode_id,
        sensor_name=row.sensor_name,
        event_hdf5_path=row.event_hdf5_path,
        severity=severity,
    )


def _manifest_issue(
    *,
    field: str,
    message: str,
    row_index: int,
    line_number: int | None = None,
) -> EventCameraRunValidationIssue:
    return EventCameraRunValidationIssue(
        field=field,
        message=message,
        row_index=row_index,
        episode_id=0,
        sensor_name="",
        event_hdf5_path="",
        line_number=line_number,
    )


def _format_issue_summary(issue: EventCameraRunValidationIssue) -> str:
    location = (
        f"line {issue.line_number} row {issue.row_index}"
        if issue.line_number is not None
        else f"row {issue.row_index}"
    )
    return (
        f"{location} field {issue.field} "
        f"episode {issue.episode_id} sensor {issue.sensor_name!r}: "
        f"{issue.message}"
    )


__all__ = [
    "EVENTS_MANIFEST_FILENAME",
    "EventCameraRunArtifact",
    "EventCameraRunSelection",
    "EventCameraRunSummary",
    "EventCameraRunValidationIssue",
    "EventCameraRunValidationReport",
    "EventManifestRow",
    "append_event_manifest_row",
    "append_event_manifest_rows",
    "create_event_manifest",
    "parse_event_reward_summary",
]
