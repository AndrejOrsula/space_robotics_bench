"""CLI adapter for ``srb agent tune`` — Optuna hyperparameter optimization.

Isaac-free: this drives an Optuna study that spawns ``srb agent train``
subprocesses (one per trial). Verbs: run / status / export / repair. Optuna is
imported lazily by the driver, so ``--help`` and non-run verbs stay light.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any


def execute_tune(tune_action: str, **kwargs: Any) -> None:
    """Dispatch a ``srb agent tune <verb>`` invocation."""
    # No tune action passes bare overrides or ``--`` tokens on; accepting them
    # silently would run a study without the requested settings.
    ignored = [
        *(kwargs.pop("hydra_args", None) or ()),
        *(kwargs.pop("forwarded_args", None) or ()),
    ]
    if ignored:
        raise SystemExit(
            f"srb agent tune {tune_action}: error: unsupported arguments "
            f"{' '.join(ignored)}; fix a setting for every trial with "
            "--pin KEY=VALUE"
        )
    if tune_action == "run":
        _run(**kwargs)
    elif tune_action == "status":
        _status(**kwargs)
    elif tune_action == "export":
        _export(**kwargs)
    elif tune_action == "repair":
        _repair(**kwargs)
    elif tune_action == "baseline":
        _baseline(**kwargs)
    elif tune_action == "check":
        _check(**kwargs)
    elif tune_action == "validate":
        _validate(**kwargs)
    else:  # pragma: no cover - argparse enforces the choices
        raise SystemExit(f"Unknown tune action: {tune_action!r}")


def _short_task(name: str) -> str:
    return name.rsplit("/", 1)[-1]


def _resolve_tasks(env_id: str | None, tasks: list[str] | None) -> list[str]:
    if env_id and tasks:
        raise SystemExit(
            "srb agent tune: error: pass either --env/--task (single-task study) "
            "or --tasks (portfolio study), not both."
        )
    if tasks:
        return [_short_task(t) for t in tasks]
    if env_id:
        return [_short_task(env_id)]
    raise SystemExit("srb agent tune run: error: one of --env or --tasks is required")


def _run(
    *,
    env_id: str | None = None,
    tasks: list[str] | None = None,
    algo: str,
    obs: str = "state",
    trial_env_steps: int,
    n_trials: int = 20,
    n_workers: int = 1,
    gpus: str | None = None,
    seed_base: int = 42,
    seeds_per_trial: int = 1,
    num_envs: int | None = None,
    pins: list[str] | None = None,
    max_wall_hours: float | None = None,
    timeout_s: float | None = None,
    objective: str = "ep_return_mean",
    fps_vec_n: float | None = None,
    n_startup: int | None = None,
    boot_stagger_s: float | None = None,
    version: int = 1,
    storage: str | None = None,
    keep_artifacts: bool = False,
    retry_failed: int = 0,
    normalize: str = "none",
    aggregate: str = "median",
    logdir_path: str | None = None,
    **_kwargs: Any,
) -> None:
    from srb.cli.preflight import require_rdb_storage

    require_rdb_storage(storage)
    from srb.tuning.driver import TuneConfig, run_study, summarize

    resolved_tasks = _resolve_tasks(env_id, tasks)
    gpu_list = [g for g in gpus.split(",") if g] if gpus else None
    cfg = TuneConfig(
        tasks=resolved_tasks,
        algo=algo,
        obs=obs,
        trial_env_steps=trial_env_steps,
        n_trials=n_trials,
        n_workers=n_workers,
        gpus=gpu_list,
        seed_base=seed_base,
        seeds_per_trial=seeds_per_trial,
        num_envs=num_envs,
        pins=list(pins or []),
        max_wall_hours=max_wall_hours,
        timeout_s=timeout_s,
        objective=objective,
        fps_vec_n=fps_vec_n,
        n_startup=n_startup,
        boot_stagger_s=boot_stagger_s,
        keep_artifacts=keep_artifacts,
        retry_failed=retry_failed,
        storage=storage,
        normalize=normalize,
        aggregate=aggregate,
        version=version,
        logdir_root=Path(logdir_path) if logdir_path else TuneConfig.logdir_root,
    )
    study = run_study(cfg)
    _print_status(summarize(study))


def _study_dir(study: str, logdir_path: str | None) -> Path:
    from srb.tuning.driver import TuneConfig

    root = Path(logdir_path) if logdir_path else TuneConfig.logdir_root
    return root / study


def _status(
    *,
    study: str,
    storage: str | None = None,
    logdir_path: str | None = None,
    **_kwargs: Any,
) -> None:
    from srb.tuning.driver import status

    _print_status(status(_study_dir(study, logdir_path), storage=storage))


def _export(
    *,
    study: str,
    trial: str = "best",
    to: str | None = None,
    dry_run: bool = False,
    force: bool = False,
    storage: str | None = None,
    logdir_path: str | None = None,
    **_kwargs: Any,
) -> None:
    from srb.tuning.export import export_config

    trial_arg: int | str = trial
    if isinstance(trial, str) and trial.isdigit():
        trial_arg = int(trial)
    result = export_config(
        _study_dir(study, logdir_path),
        trial=trial_arg,
        to=to,  # None → resolved from the study (portfolio→global, else task)
        dry_run=dry_run,
        force=force,
        storage=storage,
    )
    if result.dry_run:
        print(f"[dry-run] would write {result.target_path} ({result.to} slot)\n")
        print(result.diff)
    else:
        print(
            f"Wrote tuned config → {result.target_path} "
            f"({result.to} slot, trial {result.trial})"
        )
    if result.shadows:
        if result.to == "global":
            print(
                "\nNOTE: these task-specific configs will keep shadowing this "
                f"algorithm's global set: {', '.join(result.shadows)}\n"
                "Delete them to adopt the exported global config."
            )
        else:  # to == "task": the new L1 file shadows the global for this task
            print(
                f"\nNOTE: this task-specific config will shadow the global set at "
                f"{', '.join(result.shadows)} for this task only "
                "(e.g. a portfolio-tuned default no longer applies here).\n"
                "Delete the task file to revert to the global set."
            )


def _repair(
    *,
    study: str,
    stale_hours: float = 24.0,
    storage: str | None = None,
    logdir_path: str | None = None,
    **_kwargs: Any,
) -> None:
    from srb.tuning.driver import repair_study

    n = repair_study(
        _study_dir(study, logdir_path), stale_hours=stale_hours, storage=storage
    )
    print(f"Failed {n} stale RUNNING trial(s).")


def _baseline(
    *,
    env_id: str | None = None,
    tasks: list[str] | None = None,
    algo: str,
    obs: str = "state",
    budget: int,
    floor_steps: int = 200_000,
    num_envs: int | None = None,
    seed_base: int = 42,
    gpus: str | None = None,
    objective: str = "ep_return_mean",
    timeout_s: float | None = None,
    force: bool = False,
    dry_run: bool = False,
    logdir_path: str | None = None,
    **_kwargs: Any,
) -> None:
    from srb.tuning.baselines import BaselineConfig, run_baseline

    resolved_tasks = _resolve_tasks(env_id, tasks)
    gpu_list = [g for g in gpus.split(",") if g] if gpus else None
    out_root = (
        Path(logdir_path) / "baseline" if logdir_path else BaselineConfig.out_root
    )
    cfg = BaselineConfig(
        tasks=resolved_tasks,
        algo=algo,
        obs=obs,
        budget=budget,
        floor_budget=floor_steps,
        num_envs=num_envs,
        seed_base=seed_base,
        gpus=gpu_list,
        objective=objective,
        timeout_s=timeout_s,
        force=force,
        dry_run=dry_run,
        out_root=out_root,
    )
    entries = run_baseline(cfg)
    for (task, _a, _o), e in sorted(entries.items()):
        print(
            f"{task}: floor={e.floor:.4g} ref={e.ref:.4g} "
            f"(budget={e.budget}, obs={e.obs}, src={e.source_config_path})"
        )
    if dry_run:
        print("[dry-run] baselines.yaml not written")


def _check(
    *,
    env_id: str | None = None,
    tasks: list[str] | None = None,
    algo: str,
    obs: str = "state",
    trial_env_steps: int,
    num_envs: int | None = None,
    seed_base: int = 42,
    seeds_per_trial: int = 1,
    pins: list[str] | None = None,
    normalize: str = "none",
    aggregate: str = "median",
    objective: str = "ep_return_mean",
    fps_vec_n: float | None = None,
    n_trials: int = 20,
    n_workers: int = 1,
    version: int = 1,
    logdir_path: str | None = None,
    **_kwargs: Any,
) -> None:
    from srb.tuning.driver import TuneConfig, validate_config

    resolved_tasks = _resolve_tasks(env_id, tasks)
    cfg = TuneConfig(
        tasks=resolved_tasks,
        algo=algo,
        obs=obs,
        trial_env_steps=trial_env_steps,
        num_envs=num_envs,
        seed_base=seed_base,
        seeds_per_trial=seeds_per_trial,
        pins=list(pins or []),
        normalize=normalize,
        aggregate=aggregate,
        objective=objective,
        fps_vec_n=fps_vec_n,
        n_trials=n_trials,
        n_workers=n_workers,
        version=version,
        logdir_root=Path(logdir_path) if logdir_path else TuneConfig.logdir_root,
    )
    report = validate_config(cfg)
    _print_validation(report)
    if not report.ok:
        raise SystemExit(1)


def _print_validation(report) -> None:
    glyph = {"ok": "OK  ", "warn": "WARN", "error": "FAIL"}
    for c in report.checks:
        line = f"[{glyph.get(c.status, c.status)}] {c.name}"
        if c.detail:
            line += f": {c.detail}"
        print(line)
    n_err = sum(1 for c in report.checks if c.status == "error")
    n_warn = sum(1 for c in report.checks if c.status == "warn")
    if report.ok:
        print(f"\ncheck: OK ({n_warn} warning(s)) — config would run.")
    else:
        print(f"\ncheck: FAILED ({n_err} error(s), {n_warn} warning(s)).")


def _validate(
    *,
    study: str,
    top_k: int = 3,
    seeds: int = 3,
    seed_base: int = 10_000,
    min_ok: int | None = None,
    validate_steps: int | None = None,
    gpus: str | None = None,
    n_workers: int = 1,
    timeout_s: float | None = None,
    keep_artifacts: bool = False,
    storage: str | None = None,
    logdir_path: str | None = None,
    **_kwargs: Any,
) -> None:
    from srb.tuning.driver import validate_study

    gpu_list = [g for g in gpus.split(",") if g] if gpus else None
    report = validate_study(
        _study_dir(study, logdir_path),
        top_k=top_k,
        seeds=seeds,
        seed_base=seed_base,
        min_ok=min_ok,
        validate_steps=validate_steps,
        gpus=gpu_list,
        n_workers=n_workers,
        timeout_s=timeout_s,
        keep_artifacts=keep_artifacts,
        storage=storage,
    )
    _print_validation_report(report)


def _print_validation_report(report) -> None:
    print(f"Validation: {report.study}")
    print(
        f"  re-ran top-{report.top_k} trial(s) x {report.seeds} fresh seed(s) "
        f"at {report.validate_steps} env-steps (min-ok {report.min_ok})"
    )
    if not report.candidates:
        print("  no COMPLETE trials to validate.")
        return
    print(f"  {'trial':>6}  {'train':>10}  {'val mean':>10}  {'std':>9}  seeds")
    for cr in report.candidates:
        mean = "n/a" if cr.mean is None else f"{cr.mean:.4g}"
        std = "-" if cr.mean is None else f"{cr.std:.3g}"
        flag = "" if cr.n_ok >= report.min_ok else "  (below min-ok)"
        print(
            f"  {cr.trial_number:>6}  {cr.original_value:>10.4g}  "
            f"{mean:>10}  {std:>9}  {cr.n_ok}/{report.seeds} ok{flag}"
        )
    if report.winner is None:
        print(
            f"\n  no robust winner: no candidate COMPLETEd the required "
            f"{report.min_ok} seed(s). Lower --min-ok or investigate the failures."
        )
        return
    w = report.winner
    print(
        f"\n  robust winner: trial {w.trial_number} "
        f"(val mean {w.mean:.4g} +/- {w.std:.3g} over {w.n_ok} seed(s))"
    )
    if report.winner_changed:
        print(
            f"  NOTE: the study's best was trial {report.original_best}, but "
            f"trial {w.trial_number} holds up better across fresh seeds."
        )
    print(
        f"  export it:  srb agent tune export --study {report.study} "
        f"--trial {w.trial_number}"
    )


def _print_status(summary: dict) -> None:
    print(f"Study: {summary['name']}")
    print(f"  states: {summary['counts']}")
    print(f"  best:   trial {summary['best_trial']} value {summary['best_value']}")
    if summary["top5"]:
        print("  top-5:")
        for num, val in summary["top5"]:
            print(f"    trial {num}: {val}")


__all__ = ("execute_tune",)
