"""Export a winning trial's config with provenance and production driver fields."""

from __future__ import annotations

import copy
import datetime
import difflib
import os
import stat
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from srb.tuning import git_short_sha
from srb.tuning.spaces import export_reset_paths, framework_shim
from srb.tuning.study import read_manifest
from srb.utils.agent_config_catalog import (
    AgentCfgNotFoundError,
    clear_agent_config_catalog_cache,
    resolve_agent_cfg_path,
)
from srb.utils.path import SRB_HYPERPARAMS_DIR
from srb.utils.publication import ArtifactPublication, default_new_file_mode


@dataclass
class ExportResult:
    target_path: Path
    wrote: bool
    dry_run: bool
    trial: int
    value: float | None
    diff: str = ""
    shadows: list[str] = field(default_factory=list)
    to: str = ""  # resolved direction ("task" | "global") — for CLI messaging


def _atomic_write_text(path: Path, content: str) -> None:
    """Publish under lock, reclaim stages, preserve mode, or use umask for new files."""
    target_mode = (
        stat.S_IMODE(path.stat().st_mode) if path.exists() else default_new_file_mode()
    )

    def write(staging: Path) -> None:
        staging.write_text(content, encoding="utf-8")
        os.chmod(staging, target_mode)

    ArtifactPublication.file(path).publish(write)


def _parse_space_id(space_id: str) -> tuple[str, str, str]:
    framework, algo, obs = space_id.split(":")
    return framework, algo, obs


def _slot_rel(shim, framework: str, algo: str, obs: str) -> Path:
    """Return the shared resolver slot; single-file frameworks use their L0 basename."""
    suffix = "_visual" if obs == "visual" else ""
    if shim.l0_basename:
        return Path(f"{shim.l0_basename}{suffix}.yaml")
    return Path(framework) / f"{algo}{suffix}.yaml"


def _get_path(d: dict, path: tuple[str, ...]):
    node = d
    for p in path:
        if not isinstance(node, dict) or p not in node:
            return None, False
        node = node[p]
    return node, True


def _set_path(d: dict, path: tuple[str, ...], value) -> None:
    node = d
    for p in path[:-1]:
        node = node.setdefault(p, {})
    node[path[-1]] = value


def _del_path(d: dict, path: tuple[str, ...]) -> None:
    node = d
    for p in path[:-1]:
        if not isinstance(node, dict) or p not in node:
            return
        node = node[p]
    if isinstance(node, dict):
        node.pop(path[-1], None)


def _reset_driver_fields(export_doc: dict, base: dict, framework: str) -> None:
    """Reset the budget and W&B fields using framework shim paths."""
    for path in export_reset_paths(framework):
        base_val, present = _get_path(base, path)
        if present:
            _set_path(export_doc, path, base_val)
        else:
            _del_path(export_doc, path)


def _resolve_value(
    study_dir: Path, trial: int | str, *, storage: str | None = None
) -> tuple[int, float | None]:
    """Resolve an explicit trial or the best trial and its objective value."""
    # Allow explicit export from legacy local artifacts without a journal.
    # Best-trial lookup still needs history; never ignore supplied storage or a
    # present but invalid journal.
    journal_path = study_dir / "journal.log"
    if trial != "best" and storage is None and not journal_path.exists():
        return int(trial), None

    from srb.tuning.study import open_existing_study

    study, _manifest = open_existing_study(study_dir, storage=storage)
    if trial == "best":
        best = study.best_trial
        return best.number, best.value
    num = int(trial)
    for t in study.get_trials(deepcopy=False):
        if t.number == num:
            return num, t.value
    return num, None


def export_config(
    study_dir: str | Path,
    *,
    trial: int | str = "best",
    to: str | None = None,
    dry_run: bool = False,
    force: bool = False,
    storage: str | None = None,
    hyperparams_root: str | Path = SRB_HYPERPARAMS_DIR,
) -> ExportResult:
    """Export to L1 for tasks or L0 globally; portfolios default to L0.

    ``to="task"`` is ambiguous for portfolios and raises.
    """
    # Any other value would silently select (and with force, overwrite) L0.
    if to not in (None, "task", "global"):
        raise ValueError(f"to must be 'task', 'global', or None, got {to!r}")
    study_dir = Path(study_dir)
    hp_root = Path(hyperparams_root)
    manifest = read_manifest(study_dir / "study.yaml")
    # The manifest's full algo label is authoritative; legacy studies reparse
    # the space id.
    if manifest.algo:
        from srb.integrations.algorithms import resolve_algorithm_adapter

        adapter = resolve_algorithm_adapter(manifest.algo)
        framework, algo, obs = adapter.framework, adapter.local_algo, manifest.obs
    else:
        framework, algo, obs = _parse_space_id(manifest.space_id)
    task = manifest.tasks[0] if manifest.tasks else None

    is_portfolio = len(manifest.tasks) > 1
    if to is None:
        to = "global" if is_portfolio else "task"
    if to == "task" and is_portfolio:
        raise ValueError(
            f"portfolio study targets {len(manifest.tasks)} tasks; --to task is "
            "ambiguous. Use --to global (the portfolio tunes one global set)."
        )

    trial_num, value = _resolve_value(study_dir, trial, storage=storage)
    trial_cfg_path = study_dir / "trials" / f"{trial_num:03d}" / "trial_cfg.yaml"
    if not trial_cfg_path.exists():
        raise FileNotFoundError(f"No trial config at {trial_cfg_path}")
    trial_cfg = yaml.safe_load(trial_cfg_path.read_text(encoding="utf-8"))
    export_doc = copy.deepcopy(trial_cfg.get("agent", {}))

    # Restore driver-owned values from the selected slot's base (L1 then L0).
    shim = framework_shim(framework)
    algo_label = framework if shim.l0_basename else f"{framework}_{algo}"
    try:
        base = yaml.safe_load(
            resolve_agent_cfg_path(
                algo_label, obs=obs, task=task if to == "task" else None, root=hp_root
            ).read_text(encoding="utf-8")
        )
    except AgentCfgNotFoundError:
        # Without an L0 base, drop driver-owned trial values. Other resolver
        # errors propagate; exporting a budget-less config could train forever.
        base = {}
    _reset_driver_fields(export_doc, base, framework)

    # Target slot (single source shared with shadow_report).
    rel = _slot_rel(shim, framework, algo, obs)
    if to == "task":
        if task is None:
            raise ValueError("Cannot export to a task slot: study has no task")
        target = hp_root / "task" / task / rel
    else:
        target = hp_root / rel

    recorded_seed = (trial_cfg.get("env") or {}).get("seed")
    header = _provenance_header(manifest, trial_num, value, recorded_seed)
    body = yaml.safe_dump(export_doc, sort_keys=False)
    content = header + body

    shadows = shadow_report(hp_root, framework, algo, obs, to=to, task=task)

    if dry_run:
        old = target.read_text(encoding="utf-8") if target.exists() else ""
        diff = (
            "".join(
                difflib.unified_diff(
                    old.splitlines(keepends=True),
                    content.splitlines(keepends=True),
                    fromfile=str(target),
                    tofile=f"{target} (new)",
                )
            )
            or "(new file)\n" + content
        )
        return ExportResult(target, False, True, trial_num, value, diff, shadows, to=to)

    if target.exists() and not force:
        raise FileExistsError(
            f"{target} exists — pass force=True to overwrite (adoption is via git review)"
        )
    target.parent.mkdir(parents=True, exist_ok=True)
    _atomic_write_text(target, content)
    clear_agent_config_catalog_cache()
    return ExportResult(target, True, False, trial_num, value, "", shadows, to=to)


def _provenance_header(
    manifest, trial_num: int, value: float | None, seed: int | None = None
) -> str:
    date = datetime.datetime.now(tz=datetime.timezone.utc).date().isoformat()
    # Use the recorded training seed; seed_base+trial is valid only with one
    # seed per trial. Reconstruct only for legacy configs without env.seed.
    seeds = str(seed) if seed is not None else f"{manifest.seed_base}+{trial_num}"
    return (
        "# Tuned by srb agent tune — do not edit the header\n"
        f"# study: {manifest.name}   trial: {trial_num}   value: {value}\n"
        f"# objective: {manifest.objective} tail-mean @ {manifest.trial_env_steps} "
        f"env-steps   seed(s): {seeds}\n"
        f"# date: {date}   git: {git_short_sha()}   optuna: {manifest.optuna_version}\n"
    )


def shadow_report(
    root: str | Path,
    framework: str,
    algo: str,
    obs: str,
    *,
    to: str,
    task: str | None = None,
) -> list[str]:
    """Report task overrides for global exports or the shadowed global file."""
    root = Path(root)
    shim = framework_shim(framework)
    rel = _slot_rel(shim, framework, algo, obs)

    if to == "task":
        global_file = root / rel
        return [str(global_file)] if global_file.exists() else []

    if to != "global":
        return []
    task_root = root / "task"
    if not task_root.is_dir():
        return []
    shadows = []
    for task_dir in sorted(task_root.iterdir()):
        if (task_dir / rel).exists():
            shadows.append(task_dir.name)
    return shadows


__all__ = ("ExportResult", "export_config", "shadow_report")
