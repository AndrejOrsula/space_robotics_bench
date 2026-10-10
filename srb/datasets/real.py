"""Dataset conversion helpers for real validation sessions."""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from srb.datasets.export import DatasetExportResult, _publish_dataset_export
from srb.datasets.jsonl import build_srb_jsonl_row, write_srb_jsonl_row
from srb.datasets.manifest import (
    DEFAULT_MANIFEST_RELPATH,
    build_dataset_manifest,
    write_dataset_manifest,
)
from srb.datasets.schema import DatasetController, DatasetSplit, StorageFormat
from srb.utils import validation_session_log as _validation_session_log
from srb.utils.artifact_identity import ArtifactIdentity
from srb.utils.validation_contract import TASK_TERMINATION_TERM_REASON


def export_real_validation_session(
    *,
    session_dir: str | Path,
    output_dir: str | Path,
    embodiment: str = "real_hardware",
    action_mode: str = "real",
    controller: str | None = None,
    task: str | None = None,
    algo: str | None = None,
) -> DatasetExportResult:
    """Convert a real validation session into canonical SRB JSONL."""
    session_dir = Path(session_dir)
    output_dir = Path(output_dir)
    spec = _load_session_spec(session_dir)
    task = task or _spec_value(spec, "task")
    algo = algo or _spec_value(spec, "algo")
    if task is None:
        raise ValueError("task is required when session has no spec.yaml")
    controller = _normalize_validation_controller(
        controller or _infer_validation_controller(spec)
    )

    episodes_path = _validation_session_log.validation_log_path(session_dir)
    episodes, episode_end = _validation_session_log.read_validation_episode_records(
        episodes_path
    )

    def build_stage(stage: Path) -> DatasetExportResult:
        data_dir = stage / "data"
        data_dir.mkdir(parents=True, exist_ok=True)

        total_samples = 0
        incomplete_episode_ids = sorted(set(episodes) - set(episode_end))
        # Ended episodes without step rows produce no JSONL file; list them so
        # the export is not mistaken for a complete copy of the session.
        endpoint_only_episode_ids = sorted(set(episode_end) - set(episodes))
        for output_episode_index, ep in enumerate(sorted(episodes)):
            rows = sorted(episodes[ep], key=lambda row: int(row.get("step", 0)))
            total_samples += len(rows)
            episode_path = data_dir / f"episode_{output_episode_index:06d}.jsonl"
            end_record = episode_end.get(ep, {})
            success = bool(end_record.get("success", False))
            with episode_path.open("w", encoding="utf-8") as f:
                for step_offset, record in enumerate(rows):
                    has_transition_snapshots = (
                        "pre_obs_summary" in record or "next_obs_summary" in record
                    )
                    if "pre_obs_summary" in record:
                        obs = record["pre_obs_summary"]
                    elif step_offset > 0:
                        # Legacy obs_summary records describe the post-step
                        # state, so the prior row supplies this transition's
                        # observation.
                        obs = rows[step_offset - 1].get("obs_summary", {})
                    else:
                        # A legacy log has no snapshot for the initial state.
                        obs = record.get("obs_summary", {})
                    explicit_next_obs = record.get("next_obs_summary")
                    if explicit_next_obs is not None:
                        next_obs = explicit_next_obs
                    elif not has_transition_snapshots:
                        next_obs = record.get("obs_summary", {})
                    else:
                        next_obs = (
                            rows[step_offset + 1].get(
                                "pre_obs_summary",
                                rows[step_offset + 1].get("obs_summary", {}),
                            )
                            if step_offset + 1 < len(rows)
                            else {}
                        )
                    is_final_row = step_offset == len(rows) - 1
                    if is_final_row and ep not in episode_end:
                        terminated, truncated = False, True
                        row_success = False
                    elif is_final_row:
                        terminated, truncated = _episode_end_flags(end_record)
                        row_success = success
                    else:
                        terminated, truncated = False, False
                        row_success = success
                    canonical = build_srb_jsonl_row(
                        episode_index=output_episode_index,
                        episode_key=f"real_{ep:06d}",
                        step_index=int(record.get("step", step_offset)),
                        obs=obs,
                        action=record.get("act", []),
                        reward=record.get("rew", 0.0),
                        done=is_final_row,
                        next_obs=next_obs,
                        success=row_success,
                        info=record.get("info_summary", {}),
                        terminated=terminated,
                        truncated=truncated,
                    )
                    write_srb_jsonl_row(f, canonical)

        metadata = {
            "source_path": str(episodes_path),
            "source_session_path": str(session_dir),
            "source_storage_format": "real_validation_jsonl",
            "num_episodes": len(episodes),
            "total_samples": total_samples,
            "incomplete_episode_ids": incomplete_episode_ids,
            "endpoint_only_episode_ids": endpoint_only_episode_ids,
        }
        validation_summary = _load_validation_summary(session_dir)
        if validation_summary is not None:
            metadata["validation_summary"] = validation_summary
        if spec is not None:
            metadata["hardware"] = list(getattr(spec, "hardware", []))
            metadata["checkpoint"] = getattr(spec, "checkpoint", None)
            metadata["metrics_to_track"] = list(getattr(spec, "metrics_to_track", []))

        manifest = build_dataset_manifest(
            identity=ArtifactIdentity(task, embodiment, action_mode, algo),
            dataset_path="data",
            split=DatasetSplit.REAL,
            controller=controller,
            storage_format=StorageFormat.SRB_JSONL,
            metadata=metadata,
        )
        manifest_path = write_dataset_manifest(
            manifest,
            stage / DEFAULT_MANIFEST_RELPATH,
        )
        return DatasetExportResult(
            manifest_path=manifest_path,
            num_episodes=len(episodes),
            total_samples=total_samples,
        )

    return _publish_dataset_export(output_dir, build_stage)


def _episode_end_flags(end_record: Mapping[str, Any]) -> tuple[bool, bool]:
    """Map task success/termination to terminated; other ends to truncated."""

    term_reason = str(end_record.get("term_reason", ""))
    if term_reason in ("success", TASK_TERMINATION_TERM_REASON):
        return True, False
    return False, True


def _load_session_spec(session_dir: Path) -> Any | None:
    spec_path = session_dir / "spec.yaml"
    if not spec_path.exists():
        return None
    from srb.utils import validation_spec as _validation_spec

    return _validation_spec.load_validation_spec(spec_path)


def _load_validation_summary(session_dir: Path) -> dict[str, Any] | None:
    summary_path = session_dir / "summary.json"
    if not summary_path.exists():
        return None
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if not isinstance(summary, dict):
        raise ValueError(f"validation summary must be a JSON object: {summary_path}")
    return summary


def _spec_value(spec: Any | None, name: str) -> str | None:
    if spec is None:
        return None
    value = getattr(spec, name, None)
    return str(value) if value is not None else None


def _infer_validation_controller(spec: Any | None) -> str:
    if spec is None:
        return DatasetController.REAL.value
    if _spec_value(spec, "algo") or _spec_value(spec, "checkpoint"):
        return DatasetController.POLICY.value
    return DatasetController.REAL.value


def _normalize_validation_controller(controller: str) -> str:
    value = str(controller).strip()
    allowed = sorted(item.value for item in DatasetController)
    if value not in allowed:
        raise ValueError("controller must be one of: " + ", ".join(allowed))
    return value
