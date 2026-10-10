"""Reference facts for real-validation session artifacts."""

from __future__ import annotations

from dataclasses import dataclass

from srb.utils.validation_session_log import EPISODES_JSONL

SPEC_YAML = "spec.yaml"
ROLLOUT_RRD = "rollout.rrd"
TENSORBOARD_DIR = "tensorboard"
SUMMARY_JSON = "summary.json"
REPORT_MD = "report.md"
BADGE_JSON = "badge.json"
INDEX_JSONL = "_index.jsonl"
INDEX_HTML = "index.html"


@dataclass(frozen=True)
class ValidationArtifact:
    path: str
    scope: str
    written_by: str
    produced_when: str
    purpose: str


@dataclass(frozen=True)
class ValidationArtifactReference:
    session_artifacts: tuple[ValidationArtifact, ...]
    storage_root_artifacts: tuple[ValidationArtifact, ...]

    @property
    def all_artifacts(self) -> tuple[ValidationArtifact, ...]:
        return self.session_artifacts + self.storage_root_artifacts


def validation_artifact_reference() -> ValidationArtifactReference:
    """Return the runtime artifact contract used by docs and CLI guidance."""

    return ValidationArtifactReference(
        session_artifacts=(
            ValidationArtifact(
                path=SPEC_YAML,
                scope="session directory",
                written_by="write_spec_snapshot",
                produced_when="pre-flight",
                purpose="Validation spec snapshot required by rebuild-summary.",
            ),
            ValidationArtifact(
                path=EPISODES_JSONL,
                scope="session directory",
                written_by="Telemetry",
                produced_when="rollout",
                purpose="Append-only step, event, and episode-end telemetry used for metrics.",
            ),
            ValidationArtifact(
                path=ROLLOUT_RRD,
                scope="session directory",
                written_by="Telemetry",
                produced_when="when `rerun.record_to_rrd=true` and Rerun starts",
                purpose="Rerun recording used for live inspection and twin replay.",
            ),
            ValidationArtifact(
                path=f"{TENSORBOARD_DIR}/",
                scope="session directory",
                written_by="Telemetry",
                produced_when="when TensorBoard logging is enabled and available",
                purpose="Optional scalar event logs for local dashboard inspection.",
            ),
            ValidationArtifact(
                path=SUMMARY_JSON,
                scope="session directory",
                written_by="write_session_report",
                produced_when="post-session finalization or rebuild-summary",
                purpose="Machine-readable pass/fail result and criterion payload.",
            ),
            ValidationArtifact(
                path=REPORT_MD,
                scope="session directory",
                written_by="write_session_report",
                produced_when="post-session finalization or rebuild-summary",
                purpose="Human-readable validation session report.",
            ),
            ValidationArtifact(
                path=BADGE_JSON,
                scope="session directory",
                written_by="write_session_report",
                produced_when="post-session finalization or rebuild-summary",
                purpose="Shields.io endpoint payload for validation status display.",
            ),
        ),
        storage_root_artifacts=(
            ValidationArtifact(
                path=INDEX_JSONL,
                scope="storage root",
                written_by="DriftStore",
                produced_when="post-session finalization",
                purpose="Append-only session index used for drift history and dashboard rows.",
            ),
            ValidationArtifact(
                path=INDEX_HTML,
                scope="storage root",
                written_by="write_validation_index",
                produced_when="post-session finalization",
                purpose="Static cross-session dashboard for pass/fail state and metrics.",
            ),
        ),
    )


def render_validation_session_layout() -> str:
    return "\n".join(
        (
            "```text",
            "logs/real_validation/",
            f"+-- {INDEX_JSONL}",
            f"+-- {INDEX_HTML}",
            "+-- <task>/",
            "    +-- <uuid>_<utc-timestamp>/",
            "        +-- current.json",
            "        +-- generations/",
            "            +-- <generation-uuid>/",
            f"                +-- {SPEC_YAML}",
            f"                +-- {EPISODES_JSONL}",
            f"                +-- {ROLLOUT_RRD}",
            f"                +-- {TENSORBOARD_DIR}/",
            f"                +-- {SUMMARY_JSON}",
            f"                +-- {REPORT_MD}",
            f"                +-- {BADGE_JSON}",
            "```",
        )
    )


def render_validation_artifact_table(
    artifacts: tuple[ValidationArtifact, ...],
) -> str:
    lines = [
        "| Path | Scope | Written by | Produced when | Purpose |",
        "| :--- | :--- | :--- | :--- | :--- |",
    ]
    for artifact in artifacts:
        lines.append(
            "| "
            f"`{artifact.path}` | "
            f"{_escape_table_cell(artifact.scope)} | "
            f"`{artifact.written_by}` | "
            f"{_escape_table_cell(artifact.produced_when)} | "
            f"{_escape_table_cell(artifact.purpose)} |"
        )
    return "\n".join(lines)


def _escape_table_cell(value: str) -> str:
    return value.replace("|", "\\|")


__all__ = [
    "BADGE_JSON",
    "INDEX_HTML",
    "INDEX_JSONL",
    "REPORT_MD",
    "ROLLOUT_RRD",
    "SPEC_YAML",
    "SUMMARY_JSON",
    "TENSORBOARD_DIR",
    "ValidationArtifact",
    "ValidationArtifactReference",
    "render_validation_artifact_table",
    "render_validation_session_layout",
    "validation_artifact_reference",
]
