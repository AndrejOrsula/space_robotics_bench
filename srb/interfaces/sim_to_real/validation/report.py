"""Markdown + JSON + Jinja static-index output for validation sessions."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from jinja2 import Environment, FileSystemLoader, select_autoescape

from srb.interfaces.sim_to_real.validation.artifact_reference import (
    BADGE_JSON,
    REPORT_MD,
    SUMMARY_JSON,
)
from srb.interfaces.sim_to_real.validation.report_payload import (
    badge_payload,
    summary_payload,
)
from srb.interfaces.sim_to_real.validation.result import SessionResult
from srb.interfaces.sim_to_real.validation.session_workspace import (
    ValidationSessionIdentity,
)
from srb.utils.publication import ArtifactPublication
from srb.utils.validation_spec import ValidationSpec

_TEMPLATE_NAME = "_index_template.html.j2"


def _markdown(
    spec: ValidationSpec,
    r: SessionResult,
    *,
    identity: ValidationSessionIdentity | None = None,
) -> str:
    lines: list[str] = []
    lines.append(f"# Validation session — {spec.task} / {spec.algo}")
    lines.append("")
    pass_str = "PASS" if r.pass_overall else "FAIL"
    lines.append(f"**Result:** {pass_str} (exit {r.exit_code})")
    lines.append("")
    evidence = "hardware" if r.hardware_evidence else "non-hardware"
    lines.append(f"**Evidence level:** {evidence}")
    lines.append("")
    if identity is not None:
        lines.append("## Session identity")
        lines.append("")
        lines.append("| Field | Value |")
        lines.append("|---|---|")
        for key, value in identity.to_dict().items():
            lines.append(f"| {key} | {value} |")
        lines.append("")
    lines.append("## Criteria")
    lines.append("")
    lines.append("| | Status | Detail |")
    lines.append("|---|---|---|")
    lines.append(f"| (a) threshold | {r.criterion_a.status} | {r.criterion_a.detail} |")
    lines.append(f"| (b) crash | {r.criterion_b.status} | {r.criterion_b.detail} |")
    lines.append(f"| (c) drift | {r.criterion_c.status} | {r.criterion_c.detail} |")
    lines.append("")
    if r.metrics:
        lines.append("## Real metrics")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("|---|---|")
        for k in sorted(r.metrics):
            lines.append(f"| {k} | {r.metrics[k]:.4f} |")
        lines.append("")
    if r.sim_metrics:
        lines.append("## Sim baseline")
        lines.append("")
        lines.append("| Metric | Sim | Real | Ratio |")
        lines.append("|---|---|---|---|")
        for k in sorted(r.sim_metrics):
            sim = r.sim_metrics.get(k)
            real = r.metrics.get(k)
            ratio = (real / sim) if (sim and real is not None) else None
            ratio_str = f"{ratio:.3f}" if ratio is not None else "—"
            lines.append(
                f"| {k} | {sim:.4f} | {real if real is not None else '—'} | {ratio_str} |"
            )
        lines.append("")
    if r.drift_z:
        lines.append("## Drift z-scores")
        lines.append("")
        lines.append("| Metric | z |")
        lines.append("|---|---|")
        for k in sorted(r.drift_z):
            lines.append(f"| {k} | {r.drift_z[k]:.3f} |")
        lines.append("")
    if spec.notes:
        lines.append("## Operator notes")
        lines.append("")
        lines.append(spec.notes)
        lines.append("")
    return "\n".join(lines) + "\n"


def write_session_report(
    spec: ValidationSpec,
    result: SessionResult,
    session_dir: Path,
    *,
    identity: ValidationSessionIdentity | None = None,
) -> None:
    session_dir = Path(session_dir)
    ArtifactPublication.file(session_dir / SUMMARY_JSON).publish_bytes(
        json.dumps(
            summary_payload(spec, result, identity=identity),
            indent=2,
            allow_nan=False,
        ).encode("utf-8")
    )
    ArtifactPublication.file(session_dir / REPORT_MD).publish_bytes(
        _markdown(spec, result, identity=identity).encode("utf-8")
    )
    ArtifactPublication.file(session_dir / BADGE_JSON).publish_bytes(
        json.dumps(badge_payload(result), allow_nan=False).encode("utf-8")
    )


def write_index_html(
    rows: list[dict[str, Any]],
    out_path: Path,
    *,
    metric_keys: list[str],
) -> None:
    env = Environment(
        loader=FileSystemLoader(str(Path(__file__).parent)),
        autoescape=select_autoescape(["html"]),
    )
    template = env.get_template(_TEMPLATE_NAME)
    ArtifactPublication.file(out_path).publish_bytes(
        template.render(rows=rows, metric_keys=metric_keys).encode("utf-8")
    )
