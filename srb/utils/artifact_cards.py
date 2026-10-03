"""Pure, deterministic model-artifact card serialization."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from srb.utils.model_artifacts import ModelArtifactManifest


def build_model_card(manifest: ModelArtifactManifest) -> str:
    """Build a deterministic Hugging Face model card for an SRB checkpoint."""
    title = model_card_title(manifest)
    lines = model_card_front_matter(manifest=manifest, title=title) + [
        "",
        f"# {title}",
        "",
        "## Model Summary",
        "",
        (
            "This checkpoint was trained with Space Robotics Bench (SRB) and "
            "is packaged as a benchmarkable robot policy artifact."
        ),
        "",
        "## SRB Identity",
        "",
        "| Field | Value |",
        "| --- | --- |",
        f"| Task | `{manifest.identity.task}` |",
        f"| Embodiment | `{manifest.identity.embodiment}` |",
        f"| Action mode | `{manifest.identity.action_mode}` |",
        f"| Algorithm | `{manifest.identity.algo}` |",
        f"| Framework | `{manifest.framework}` |",
        f"| Checkpoint | `{manifest.checkpoint_path}` |",
        f"| Checkpoint kind | `{manifest.checkpoint_kind}` |",
        f"| Checkpoint scope | `{manifest.checkpoint_scope}` |",
        f"| Training dataset | `{manifest.dataset_repo_id or ''}` |",
        f"| Training dataset revision | `{manifest.dataset_revision or ''}` |",
        "",
        "## Usage",
        "",
        "```bash",
        "srb model validate .",
        "srb agent eval --model checkpoints/<checkpoint>",
        "```",
    ]
    if manifest.metrics:
        lines.extend(["", "## Metrics", "", "| Metric | Value |", "| --- | --- |"])
        for key, value in sorted(manifest.metrics.items()):
            lines.append(f"| `{key}` | `{format_metadata_value(value)}` |")
    if manifest.metadata:
        lines.extend(
            ["", "## Provenance Metadata", "", "| Key | Value |", "| --- | --- |"]
        )
        for key, value in sorted(manifest.metadata.items()):
            lines.append(f"| `{key}` | `{format_metadata_value(value)}` |")
    lines.append("")
    return "\n".join(lines)


def model_card_front_matter(
    *,
    manifest: ModelArtifactManifest,
    title: str,
) -> list[str]:
    return [
        "---",
        "license: apache-2.0",
        "library_name: space-robotics-bench",
        f"pretty_name: {title}",
        "tags:",
        "- robotics",
        "- robot-learning",
        "- reinforcement-learning",
        "- space-robotics",
        "srb:",
        f"  task: {manifest.identity.task}",
        f"  embodiment: {manifest.identity.embodiment}",
        f"  action_mode: {manifest.identity.action_mode}",
        f"  algo: {manifest.identity.algo}",
        f"  framework: {manifest.framework}",
        f"  checkpoint_path: {manifest.checkpoint_path}",
        f"  checkpoint_kind: {manifest.checkpoint_kind}",
        f"  checkpoint_scope: {manifest.checkpoint_scope}",
        f"  dataset_repo_id: {manifest.dataset_repo_id or 'null'}",
        f"  dataset_revision: {manifest.dataset_revision or 'null'}",
        "---",
    ]


def model_card_title(manifest: ModelArtifactManifest) -> str:
    return format_card_title(manifest.identity.model_name)


def format_card_title(value: str) -> str:
    """Render an SRB snake-case artifact name as a card title."""
    return " ".join(_title_word(part) for part in value.split("_"))


def format_metadata_value(value: Any) -> str:
    """Render deterministic scalar text for Hugging Face markdown tables."""
    if isinstance(value, (dict, list, tuple)):
        value = json.dumps(value, sort_keys=True)
    text = str(value).replace("\\", "\\\\").replace("|", r"\|")
    return text.replace("\r", " ").replace("\n", " ")


def _title_word(value: str) -> str:
    if value == "srb":
        return "SRB"
    return value.capitalize()


__all__ = [
    "build_model_card",
    "format_card_title",
    "format_metadata_value",
    "model_card_front_matter",
    "model_card_title",
]
