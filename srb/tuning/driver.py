"""Isaac-free Optuna driver for single-task and portfolio tuning."""

from __future__ import annotations

import json
import math
import os
import queue
import secrets
import signal
import statistics
import threading
import time
from collections.abc import Callable
from concurrent.futures import (
    FIRST_COMPLETED,
    ThreadPoolExecutor,
    as_completed,
    wait,
)
from contextlib import contextmanager
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, cast

import yaml

from srb.core.algorithms import canonical_algorithm_label
from srb.integrations.algorithms import resolve_algorithm_adapter
from srb.tuning import require_optuna
from srb.tuning.merge import deep_merge
from srb.tuning.normalize import aggregate as aggregate_scores
from srb.tuning.normalize import normalize_score, rank_normalize
from srb.tuning.spaces import (
    budget_overrides,
    has_space,
    registered_space_ids,
    reserved_pin_paths,
    space_hash,
    space_id,
)
from srb.tuning.study import (
    StudyManifest,
    build_pruner,
    build_sampler,
    open_study,
    reserve_study_trial,
    resolved_profile_hash,
    study_name,
)
from srb.tuning.trial_exec import (
    TrialOutcome,
    TrialSpec,
    build_train_argv,
    cancel_inflight,
    run_trial,
    trial_spec_from_plan,
    write_trial_cfg,
)
from srb.tuning.trial_plan import (  # noqa: F401
    _DEFAULT_TRIAL_NUM_ENVS,
    PinCollisionError,
    TrialKey,
    TrialPlan,
    TrialScalar,
    TrialSection,
    TrialValue,
    TuneConfig,
    _canonicalize_trial_section,
    _coerce,
    _config_invariant_errors,
    _effective_num_envs,
    _freeze_trial_section,
    _freeze_trial_value,
    _leaf_paths,
    _normalize_yaml_scalars,
    _paths_overlap,
    _require_exact_trial_seed,
    _set_path,
    _thaw_trial_section,
    _thaw_trial_value,
    _validated_recorded_num_envs,
    apply_pins,
    assemble_trial_doc,
    effective_boot_stagger,
    project_wall_clock,
    validate_pins,
)
from srb.utils.agent_config_catalog import resolve_agent_cfg_path
from srb.utils.process_supervisor import OwnedProcessHandle, ProcessCleanupError


def _publish_json(path: Path, payload: object) -> None:
    """Atomically publish tuning metadata."""
    from srb.utils.publication import ArtifactPublication

    data = (json.dumps(payload, indent=2) + "\n").encode("utf-8")
    ArtifactPublication.file(path).publish_bytes(data)


def _no_report(rung: int, val: float) -> None:
    """Keep segment children from reporting conflicting progress rungs."""


def _never_prune() -> bool:
    """Prune portfolios between segments, not inside a segment."""
    return False


def _portfolio_rung_is_prunable(index: int, n_tasks: int) -> bool:
    """The final portfolio segment cannot prune."""
    return index < n_tasks - 1


def _single_rung_is_prunable(rung: int, n_rungs: int = 20) -> bool:
    """The terminal progress rung cannot prune."""
    return rung < n_rungs


class PortfolioNormalizer:
    """Normalize task scores for portfolio aggregation and pruning."""

    def __init__(
        self,
        mode: str,
        *,
        floor_ref: dict | None = None,
        study=None,
        storage: str | None = None,
    ):
        self._mode = mode
        self._floor_ref = floor_ref or {}
        self._study = study
        self._storage = storage

    def normalize(self, task: str, raw: float) -> float:
        if self._mode == "none":
            return raw
        if self._mode == "baseline":
            floor, ref = self._floor_ref[task]
            return normalize_score(raw, floor=floor, ref=ref)
        if self._mode == "rank":
            trials = _storage_call(
                self._storage,
                "portfolio rank trial read",
                lambda: self._study.get_trials(deepcopy=False),
            )
            pool = [
                t.user_attrs[f"seg/{task}/raw"]
                for t in trials
                if t.state.name == "COMPLETE" and f"seg/{task}/raw" in t.user_attrs
            ]
            return rank_normalize(raw, pool)
        raise ValueError(
            f"unknown normalize mode {self._mode!r}; choose 'none', 'baseline', or 'rank'"
        )


def _finished(study, *, storage: str | None = None) -> int:
    trials = _storage_call(
        storage,
        "study trial status read",
        lambda: study.get_trials(deepcopy=False),
    )
    return sum(1 for t in trials if t.state.name in ("COMPLETE", "PRUNED", "FAIL"))


def _study_budget_counts(study, *, storage: str | None = None) -> tuple[int, int]:
    """Return terminal and active trials consuming the shared budget."""
    trials = _storage_call(
        storage,
        "study trial budget read",
        lambda: study.get_trials(deepcopy=False),
    )
    terminal = sum(
        trial.state.name in {"COMPLETE", "PRUNED", "FAIL"} for trial in trials
    )
    running = sum(trial.state.name == "RUNNING" for trial in trials)
    return terminal, running


def _new_sampler_seed(seed_base: int) -> int:
    """Give each shared-study worker an independent sampler seed."""
    return secrets.randbits(32) ^ (int(seed_base) & 0xFFFFFFFF)


def _storage_call(
    storage: str | None, operation: str, callback: Callable[[], Any]
) -> Any:
    """Run a storage operation through the typed failure seam."""
    if storage is None:
        return callback()
    from srb.tuning.storage import rdb_operation

    with rdb_operation(dsn=storage, operation=operation):
        return callback()


class _TrialStorageProxy:
    """Wrap suggestions with the storage failure seam."""

    def __init__(
        self,
        trial: Any,
        db_call: Callable[[str, Callable[[], Any]], Any],
    ) -> None:
        self._trial = trial
        self._db_call = db_call

    def __getattr__(self, name: str) -> Any:
        attribute = getattr(self._trial, name)
        if not name.startswith("suggest_") or not callable(attribute):
            return attribute

        def suggest(*args: Any, **kwargs: Any) -> Any:
            return self._db_call(
                "trial parameter sampling",
                lambda: attribute(*args, **kwargs),
            )

        return suggest


def reenqueue_failed(study, n: int, *, storage: str | None = None) -> int:
    """Re-enqueue up to *n* eligible failures, newest first; return the count."""
    if n <= 0:
        return 0
    from optuna.trial import TrialState

    trials = _storage_call(
        storage,
        "failed trial inspection",
        lambda: study.get_trials(deepcopy=False),
    )
    retried_numbers = set(study.user_attrs.get("srb.retried_trials", []))
    retryable = [
        t
        for t in trials
        if t.state == TrialState.FAIL
        and t.params
        and t.user_attrs.get("reason") != "timeout"
        and not any(
            value == "timeout"
            for key, value in t.user_attrs.items()
            if key.startswith("seg/") and key.endswith("/reason")
        )
        and t.number not in retried_numbers
    ]
    count = 0
    retried_numbers = set(retried_numbers)
    waiting_params = [
        t.system_attrs.get("fixed_params", {})
        for t in trials
        if t.state == TrialState.WAITING and t.system_attrs.get("fixed_params")
    ]
    for t in reversed(retryable):
        if count >= n:
            break
        if any(t.params == params for params in waiting_params):
            retried_numbers.add(t.number)
            _storage_call(
                storage,
                "failed trial retry marker write",
                lambda: study.set_user_attr(
                    "srb.retried_trials", sorted(retried_numbers)
                ),
            )
            continue
        _storage_call(
            storage,
            "failed trial requeue",
            lambda params=t.params: study.enqueue_trial(params),
        )
        waiting_params.append(t.params)
        # Finished trials cannot be updated through Optuna's user-attr API.
        # Study attrs are durable in journal and RDB storage alike.
        retried_numbers.add(t.number)
        _storage_call(
            storage,
            "failed trial retry marker write",
            lambda: study.set_user_attr("srb.retried_trials", sorted(retried_numbers)),
        )
        count += 1
    return count


@contextmanager
def _stop_signal_handlers():
    """Route termination signals through the study cleanup path."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    previous = {
        signum: signal.getsignal(signum) for signum in (signal.SIGTERM, signal.SIGHUP)
    }

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    try:
        for signum in previous:
            signal.signal(signum, interrupt)
        yield
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def run_study(
    cfg: TuneConfig,
    *,
    command_builder: Callable[[TrialSpec], list[str]] = build_train_argv,
):
    """Run or resume until terminal and active trials consume the shared
    ``cfg.n_trials`` budget."""
    invariant_errors = _config_invariant_errors(cfg)
    if invariant_errors:
        raise SystemExit(next(iter(invariant_errors.values())))

    try:
        canonical_algo = canonical_algorithm_label(cfg.algo)
    except ValueError as exc:
        raise SystemExit(str(exc)) from None
    cfg = replace(cfg, algo=canonical_algo)

    validate_pins(cfg.pins, reserved=set())

    optuna = require_optuna()

    is_portfolio = len(cfg.tasks) > 1

    # Portfolios share one HP set from the global (L0) config; single-task studies
    # resolve task-specific L1 then L0.
    base_task = None if is_portfolio else cfg.tasks[0]

    adapter = resolve_algorithm_adapter(cfg.algo)
    framework = adapter.framework
    local_algo = adapter.local_algo

    if not has_space(framework, local_algo, cfg.obs):
        hint = (
            "Visual studies need a visual profile + space (not yet registered)."
            if cfg.obs == "visual"
            else "Registered spaces: " + ", ".join(registered_space_ids()) + "."
        )
        raise SystemExit(
            f"No search space registered for {cfg.algo!r} obs={cfg.obs!r}. " + hint
        )

    reserved = {"env.seed", "env.num_envs", *reserved_pin_paths(framework)}
    validate_pins(cfg.pins, reserved=reserved)

    base_cfg_path = resolve_agent_cfg_path(
        cfg.algo, obs=cfg.obs, task=base_task, root=cfg.hyperparams_root
    )
    base_agent_cfg = yaml.safe_load(base_cfg_path.read_text(encoding="utf-8")) or {}

    num_envs = _effective_num_envs(cfg.num_envs)

    name = study_name(cfg.algo, cfg.obs, cfg.tasks, cfg.version)
    study_dir = Path(cfg.logdir_root) / name
    manifest = StudyManifest(
        name=name,
        space_id=space_id(framework, local_algo, cfg.obs),
        space_hash=space_hash(framework, local_algo, cfg.obs),
        tasks=tuple(cfg.tasks),
        obs=cfg.obs,
        trial_env_steps=cfg.trial_env_steps,
        objective=cfg.objective,
        direction="maximize",
        seed_base=cfg.seed_base,
        normalize=cfg.normalize,
        aggregate=cfg.aggregate,
        optuna_version=optuna.__version__,
        algo=cfg.algo,
        pins=tuple(cfg.pins),
        profile_hash=resolved_profile_hash(base_agent_cfg),
        num_envs=num_envs,
        seeds_per_trial=cfg.seeds_per_trial,
    )
    n_startup = (
        cfg.n_startup if cfg.n_startup is not None else (15 if is_portfolio else 10)
    )
    # Single-task pruner warms up over 6 of 20 progress rungs (30%). A
    # portfolio reports at K segment rungs (0..K-1), so a fixed 6 would never
    # fire for K ≤ 6 — scale the warmup to ~30% of the task count (≥1: one
    # task's score is too noisy to kill a candidate).
    n_warmup = 6 if not is_portfolio else max(1, len(cfg.tasks) // 3)
    sampler_seed = _new_sampler_seed(cfg.seed_base)
    study = open_study(
        name,
        study_dir,
        manifest,
        sampler=build_sampler(n_startup, seed=sampler_seed),
        pruner=build_pruner(n_warmup),
        storage=cfg.storage,
    )

    return _run_study_after_open(
        cfg=cfg,
        study=study,
        command_builder=command_builder,
        is_portfolio=is_portfolio,
        base_task=base_task,
        framework=framework,
        local_algo=local_algo,
        base_agent_cfg=base_agent_cfg,
        num_envs=num_envs,
        reserved=reserved,
        name=name,
        study_dir=study_dir,
        sampler_seed=sampler_seed,
    )


def _run_study_after_open(
    *,
    cfg: TuneConfig,
    study: Any,
    command_builder: Callable[[TrialSpec], list[str]],
    is_portfolio: bool,
    base_task: str | None,
    framework: str,
    local_algo: str,
    base_agent_cfg: dict[str, Any],
    num_envs: int,
    reserved: set[str],
    name: str,
    study_dir: Path,
    sampler_seed: int | None = None,
) -> Any:
    """Dispose owned RDB state if the opened study fails."""
    succeeded = False
    try:
        result = _run_study_after_open_body(
            cfg=cfg,
            study=study,
            command_builder=command_builder,
            is_portfolio=is_portfolio,
            base_task=base_task,
            framework=framework,
            local_algo=local_algo,
            base_agent_cfg=base_agent_cfg,
            num_envs=num_envs,
            reserved=reserved,
            name=name,
            study_dir=study_dir,
            sampler_seed=sampler_seed,
        )
        succeeded = True
        return result
    finally:
        if not succeeded and cfg.storage is not None:
            from srb.tuning.storage import dispose_rdb_storage, remove_rdb_session

            owned_storage = getattr(study, "_srb_owned_rdb_storage", None)
            if owned_storage is None:
                # Compatibility for a Study opened before the explicit owner
                # marker was attached; dispose_rdb_storage unwraps Optuna's
                # current _CachedStorage wrapper.
                owned_storage = getattr(study, "_storage", None)
            remove_rdb_session(owned_storage)
            dispose_rdb_storage(owned_storage)


def _run_study_after_open_body(
    *,
    cfg: TuneConfig,
    study: Any,
    command_builder: Callable[[TrialSpec], list[str]],
    is_portfolio: bool,
    base_task: str | None,
    framework: str,
    local_algo: str,
    base_agent_cfg: dict[str, Any],
    num_envs: int,
    reserved: set[str],
    name: str,
    study_dir: Path,
    sampler_seed: int | None = None,
) -> Any:
    from optuna.trial import TrialState

    def _db_call(operation: str, callback: Callable[[], Any]) -> Any:
        return _storage_call(cfg.storage, operation, callback)

    def _remove_storage_session() -> None:
        if cfg.storage is None:
            return
        from srb.tuning.storage import remove_rdb_session

        owned_storage = getattr(study, "_srb_owned_rdb_storage", None)
        if owned_storage is None:
            owned_storage = getattr(study, "_storage", None)
        remove_rdb_session(owned_storage)

    def _trial_call(
        trial: Any, operation: str, method_name: str, *args: Any, **kwargs: Any
    ) -> Any:
        """Run a Trial storage method through the typed failure seam."""
        return _db_call(
            operation,
            lambda: getattr(trial, method_name)(*args, **kwargs),
        )

    if cfg.retry_failed:
        retried = reenqueue_failed(
            study,
            cfg.retry_failed,
            storage=cfg.storage,
        )
        if retried:
            _log(f"[tune] re-enqueued {retried} failed trial(s) for retry")
        _remove_storage_session()

    import socket
    from datetime import datetime

    _publish_json(
        study_dir / "driver.json",
        {
            "host": socket.gethostname(),
            "pid": os.getpid(),
            "started": datetime.now().astimezone().isoformat(timespec="seconds"),
        },
    )

    proj = project_wall_clock(
        trial_env_steps=cfg.trial_env_steps,
        fps_vec_n=cfg.fps_vec_n,
        n_trials=cfg.n_trials,
        n_workers=cfg.n_workers,
        seeds_per_trial=cfg.seeds_per_trial,
        n_tasks=len(cfg.tasks),
    )
    if proj is not None:
        seeds_note = (
            f", up to {cfg.seeds_per_trial}x for surviving trials"
            if cfg.seeds_per_trial > 1
            else ""
        )
        _log(
            f"[tune] projected ~{proj['hours_per_trial']:.2f} h/trial, "
            f"~{proj['study_hours']:.1f} h total ({cfg.n_trials} trials, "
            f"{cfg.n_workers} workers{seeds_note})"
        )
    else:
        _log("[tune] wall-clock projection unavailable (pass --fps for an estimate)")

    floor_ref: dict | None = None
    if is_portfolio and cfg.normalize == "baseline":
        from srb.tuning.baselines import check_baselines_staleness, load_baseline

        floor_ref = {}
        for t in cfg.tasks:
            entry = load_baseline(t, cfg.algo, cfg.obs, root=cfg.hyperparams_root)
            if entry.objective != cfg.objective:
                raise SystemExit(
                    f"baseline for {t!r} records objective {entry.objective!r}, "
                    f"but study objective is {cfg.objective!r}"
                )
            expected_envs = _effective_num_envs(cfg.num_envs)
            recorded_envs = (
                _DEFAULT_TRIAL_NUM_ENVS if entry.num_envs is None else entry.num_envs
            )
            if entry.budget != cfg.trial_env_steps:
                _log(
                    f"[tune] WARNING: baseline for {t!r} uses budget {entry.budget}, "
                    f"study uses {cfg.trial_env_steps}"
                )
            if recorded_envs != expected_envs:
                _log(
                    f"[tune] WARNING: baseline for {t!r} uses num_envs "
                    f"{recorded_envs}, study uses {expected_envs}"
                )
            floor_ref[t] = (entry.floor, entry.ref)
        for w in check_baselines_staleness(
            cfg.tasks, cfg.algo, cfg.obs, root=cfg.hyperparams_root
        ):
            _log(f"[tune] WARNING: {w}")
    elif is_portfolio and cfg.normalize == "none":
        _log(
            "[tune] WARNING: portfolio study with --normalize none aggregates RAW "
            "returns across tasks of different scales; prefer --normalize baseline "
            "(run `srb agent tune baseline`) or --normalize rank."
        )
    normalizer = PortfolioNormalizer(
        cfg.normalize,
        floor_ref=floor_ref,
        study=study,
        storage=cfg.storage,
    )

    # Boot stagger serializes concurrent Isaac first-boots (shader-cache race,
    # Only meaningful with >1 worker; auto-default 30s there, else 0.
    boot_stagger_s = effective_boot_stagger(cfg.boot_stagger_s, cfg.n_workers)
    boot_lock = threading.Lock()
    # Set once any trial's metrics start flowing — later starts skip the
    # stagger (it guards only the cold shader-cache race on first boots).
    first_boot_evt = threading.Event()

    gpu_pool: queue.Queue[str] | None = None
    if cfg.gpus:
        gpu_pool = queue.Queue()
        for g in cfg.gpus:
            gpu_pool.put(g)

    # Live process groups let Ctrl-C reap every in-flight child.
    inflight_handles: set[OwnedProcessHandle] = set()
    inflight_lock = threading.Lock()

    # Set on Ctrl-C so workers still queued behind the boot lock / GPU pool
    # never spawn a child after the reap (it would train unsupervised).
    stop_evt = threading.Event()

    t0 = time.monotonic()

    def _within_wall() -> bool:
        return (
            cfg.max_wall_hours is None
            or (time.monotonic() - t0) / 3600.0 < cfg.max_wall_hours
        )

    def _acquire_gpu() -> str | None:
        if gpu_pool is None:
            return None
        while True:  # bounded waits so a Ctrl-C can't strand us on the queue
            try:
                return gpu_pool.get(timeout=0.5)
            except queue.Empty:
                if stop_evt.is_set():
                    raise KeyboardInterrupt from None

    def _assemble_trial_doc(trial: Any) -> TrialPlan:
        """Build the plan shared by portfolio segments or one child."""
        sampled_trial = (
            _TrialStorageProxy(trial, _db_call) if cfg.storage is not None else trial
        )
        return assemble_trial_doc(
            sampled_trial,
            cfg,
            framework=framework,
            local_algo=local_algo,
            base_agent_cfg=base_agent_cfg,
            num_envs=num_envs,
            reserved=reserved,
        )

    def _spec(
        trial: Any,
        task: Any,
        plan: TrialPlan,
        seed: int,
        trial_dir: Path,
        gpu: str | None,
    ) -> TrialSpec:
        return trial_spec_from_plan(
            plan,
            task=task,
            algo=cfg.algo,
            obs=cfg.obs,
            study=name,
            trial_number=trial.number,
            trial_env_steps=cfg.trial_env_steps,
            trial_dir=trial_dir,
            seed=seed,
            gpu=gpu,
            timeout_s=cfg.timeout_s,
        )

    def _run_child(
        spec: TrialSpec,
        *,
        report_cb: Callable[[int, float], None],
        should_prune_cb: Callable[[], bool],
    ) -> TrialOutcome:
        return run_trial(
            spec,
            report_cb=report_cb,
            should_prune_cb=should_prune_cb,
            poll_dt=cfg.poll_dt,
            objective_field=cfg.objective,
            command_builder=command_builder,
            grace_s=cfg.grace_s,
            boot_lock=boot_lock,
            boot_stagger_s=boot_stagger_s,
            stop_evt=stop_evt,
            first_boot_evt=first_boot_evt,
            on_start=lambda handle: _register_handle(
                inflight_lock, inflight_handles, handle
            ),
            on_stop=lambda handle: _unregister_handle(
                inflight_lock, inflight_handles, handle
            ),
        )

    def _cleanup_logdirs(specs: list[TrialSpec]) -> None:
        # Pruned/failed checkpoints + TB events are dead weight; keep the
        # provenance files (trial_cfg.yaml, metrics.jsonl).
        if cfg.keep_artifacts:
            return
        import shutil

        for s in specs:
            shutil.rmtree(s.logdir, ignore_errors=True)

    def _rep_seed(trial: Any, k: int) -> int:
        # seed(trial t, replicate k) = seed_base + t*K + k — disjoint across
        # BOTH trials and replicates. K is frozen; a resume with a different
        # seed_base (allowed drift) can reuse seeds.
        return cfg.seed_base + cast(int, trial.number) * cfg.seeds_per_trial + k

    def _run_single_rep(
        trial: Any,
        gpu: str | None,
        plan: TrialPlan,
        seed: int,
        rep_dir: Path,
        *,
        prune: bool,
    ) -> tuple[TrialSpec, TrialOutcome]:
        spec = _spec(trial, base_task, plan, seed, rep_dir, gpu)
        latest_rung = -1

        def report(rung: int, value: float) -> None:
            nonlocal latest_rung
            latest_rung = rung
            if _single_rung_is_prunable(rung):
                _safe_report(trial, value, rung, _db_call)

        outcome = _run_child(
            spec,
            report_cb=report if prune else _no_report,
            should_prune_cb=(
                (
                    lambda: (
                        _single_rung_is_prunable(latest_rung)
                        and _db_call("trial prune decision", trial.should_prune)
                    )
                )
                if prune
                else _never_prune
            ),
        )
        return spec, outcome

    def _execute_single(trial: Any, gpu: str | None) -> None:
        plan = _assemble_trial_doc(trial)
        trials_dir = study_dir / "trials" / f"{trial.number:03d}"
        values: list[float] = []
        # Replicate 0 is the gating run: it reports rungs + may be pruned, and
        # keeps the canonical trials/NNN dir. Replicates 1..K-1 only run if it
        # COMPLETEs, never prune, and their checkpoints are discarded (the
        # trial_cfg.yaml + metrics.jsonl provenance files are kept).
        for k in range(cfg.seeds_per_trial):
            rep_dir = trials_dir if k == 0 else trials_dir / f"rep-{k}"
            spec, outcome = _run_single_rep(
                trial,
                gpu,
                plan,
                _rep_seed(trial, k),
                rep_dir,
                prune=(k == 0),
            )
            if k == 0:
                for _ka, _va in outcome.user_attrs.items():
                    _trial_call(
                        trial, "trial user attribute write", "set_user_attr", _ka, _va
                    )
                if outcome.state != "COMPLETE":
                    if outcome.state == "PRUNED":
                        _db_call(
                            "trial prune state write",
                            lambda: study.tell(trial, state=TrialState.PRUNED),
                        )
                        _cleanup_logdirs([spec])
                    else:
                        _db_call(
                            "trial failure state write",
                            lambda: study.tell(trial, state=TrialState.FAIL),
                        )
                    _log(
                        f"[tune] trial {trial.number}: {outcome.state} value={outcome.value}"
                    )
                    return
                values.append(float(outcome.value))
            elif outcome.state == "COMPLETE":
                values.append(float(outcome.value))
                _cleanup_logdirs([spec])
            else:
                _cleanup_logdirs([spec])
                _log(
                    f"[tune] trial {trial.number}: seed replicate {k} "
                    f"{outcome.state} — dropped from the mean"
                )
        if cfg.seeds_per_trial > 1 and stop_evt.is_set():
            # A Ctrl-C/abort was reaped mid-averaging (a later replicate died to
            # the driver's stop). At K=1 the interrupt raises out of run_trial
            # and _one_trial FAILs the trial; the multi-seed loop reaps via a
            # returned outcome, so it must be reclassified here — not told
            # COMPLETE on the partial mean of the seeds that finished first.
            _trial_call(
                trial,
                "trial interruption attribute write",
                "set_user_attr",
                "reason",
                "interrupted",
            )
            _db_call(
                "trial interruption state write",
                lambda: study.tell(trial, state=TrialState.FAIL),
            )
            _log(f"[tune] trial {trial.number}: FAIL (interrupted mid-averaging)")
            return
        value = statistics.fmean(values)
        if cfg.seeds_per_trial > 1:
            _trial_call(
                trial,
                "trial seed count attribute write",
                "set_user_attr",
                "n_seeds",
                cfg.seeds_per_trial,
            )
            _trial_call(
                trial,
                "trial completed seed count attribute write",
                "set_user_attr",
                "n_seeds_ok",
                len(values),
            )
        _db_call("trial completion state write", lambda: study.tell(trial, value))
        suffix = (
            f" (mean of {len(values)}/{cfg.seeds_per_trial} seeds)"
            if cfg.seeds_per_trial > 1
            else ""
        )
        _log(f"[tune] trial {trial.number}: COMPLETE value={value}{suffix}")

    def _tell_segment_terminal(
        trial: Any,
        i: int,
        task: str,
        outcome: TrialOutcome,
        seg_specs: list[TrialSpec],
    ) -> None:
        for k, v in outcome.user_attrs.items():
            _trial_call(
                trial,
                "portfolio segment attribute write",
                "set_user_attr",
                f"seg/{task}/{k}",
                v,
            )
        if outcome.state == "PRUNED":
            _db_call(
                "portfolio segment prune state write",
                lambda: study.tell(trial, state=TrialState.PRUNED),
            )
            _log(f"[tune] trial {trial.number}: PRUNED (diverged) segment {i} ({task})")
        else:
            _trial_call(
                trial,
                "portfolio segment failure attribute write",
                "set_user_attr",
                f"seg/{task}/state",
                "FAIL",
            )
            reason = outcome.user_attrs.get("reason")
            if reason is not None:
                _trial_call(
                    trial,
                    "portfolio trial failure reason attribute write",
                    "set_user_attr",
                    "reason",
                    reason,
                )
            _db_call(
                "portfolio segment failure state write",
                lambda: study.tell(trial, state=TrialState.FAIL),
            )
            _log(
                f"[tune] trial {trial.number}: FAIL segment {i} ({task}) "
                f"reason={outcome.user_attrs.get('reason')}"
            )
        _cleanup_logdirs(seg_specs)

    def _portfolio_rep(
        trial: Any,
        gpu: str | None,
        plan: TrialPlan,
        seed: int,
        rep_root: Path,
        *,
        prune: bool,
    ) -> dict[str, Any]:
        """Run portfolio segments for one seed; caller records attrs and tells once."""
        seg_specs: list[TrialSpec] = []
        seg_attrs: dict[str, Any] = {}
        normalized: list[float] = []
        for i, task in enumerate(cfg.tasks):  # fixed order (frozen manifest.tasks)
            spec = _spec(
                trial,
                task,
                plan,
                seed,
                rep_root / f"seg-{task}",
                gpu,
            )
            seg_specs.append(spec)
            outcome = _run_child(
                spec, report_cb=_no_report, should_prune_cb=_never_prune
            )
            if outcome.state != "COMPLETE":
                return {
                    "kind": "terminal",
                    "outcome": outcome,
                    "i": i,
                    "task": task,
                    "specs": seg_specs,
                    "seg_attrs": seg_attrs,
                }
            raw = float(outcome.value)
            nrm = normalizer.normalize(task, raw)
            seg_attrs[f"seg/{task}/raw"] = raw
            seg_attrs[f"seg/{task}/norm"] = nrm
            normalized.append(nrm)
            if prune and _portfolio_rung_is_prunable(i, len(cfg.tasks)):
                # Running aggregate over the first i+1 normalized scores → segment
                # rung i, so MedianPruner compares like-for-like across trials.
                _safe_report(
                    trial,
                    aggregate_scores(normalized, cfg.aggregate),
                    i,
                    _db_call,
                )
                if _db_call("trial prune decision", trial.should_prune):
                    return {
                        "kind": "prune",
                        "i": i,
                        "task": task,
                        "specs": seg_specs,
                        "seg_attrs": seg_attrs,
                    }
        return {
            "kind": "complete",
            "value": aggregate_scores(normalized, cfg.aggregate),
            "specs": seg_specs,
            "seg_attrs": seg_attrs,
        }

    def _execute_portfolio(trial: Any, gpu: str | None) -> None:
        plan = _assemble_trial_doc(trial)
        trial_root = study_dir / "trials" / f"{trial.number:03d}"
        # One canonical trial_cfg.yaml at the trial root (export reads it; the
        # HP set is shared across every segment and replicate).
        write_trial_cfg(
            _spec(
                trial,
                cfg.tasks[0],
                plan,
                _rep_seed(trial, 0),
                trial_root,
                gpu,
            )
        )
        values: list[float] = []
        for k in range(cfg.seeds_per_trial):
            rep_root = trial_root if k == 0 else trial_root / f"rep-{k}"
            rep = _portfolio_rep(
                trial,
                gpu,
                plan,
                _rep_seed(trial, k),
                rep_root,
                prune=(k == 0),
            )
            if k == 0:
                # Completed segments' raw/norm are recorded even when a later
                # segment fails (parity with the pre-replicate loop).
                for _ka, _va in rep["seg_attrs"].items():
                    _trial_call(
                        trial,
                        "portfolio segment attribute write",
                        "set_user_attr",
                        _ka,
                        _va,
                    )
                if rep["kind"] == "terminal":
                    _tell_segment_terminal(
                        trial, rep["i"], rep["task"], rep["outcome"], rep["specs"]
                    )
                    return
                if rep["kind"] == "prune":
                    _db_call(
                        "portfolio trial prune state write",
                        lambda: study.tell(trial, state=TrialState.PRUNED),
                    )
                    _cleanup_logdirs(rep["specs"])
                    _log(
                        f"[tune] trial {trial.number}: PRUNED at segment rung "
                        f"{rep['i']} ({rep['task']})"
                    )
                    return
                values.append(rep["value"])
            else:
                # Confirmation replicate: discard checkpoints/TB events; the
                # provenance files (trial_cfg.yaml, metrics.jsonl) are retained.
                _cleanup_logdirs(rep["specs"])
                if rep["kind"] == "complete":
                    values.append(rep["value"])
                else:
                    _log(
                        f"[tune] trial {trial.number}: seed replicate {k} "
                        f"did not complete — dropped from the mean"
                    )
        if cfg.seeds_per_trial > 1 and stop_evt.is_set():
            _trial_call(
                trial,
                "trial interruption attribute write",
                "set_user_attr",
                "reason",
                "interrupted",
            )
            _db_call(
                "portfolio trial interruption state write",
                lambda: study.tell(trial, state=TrialState.FAIL),
            )
            _log(f"[tune] trial {trial.number}: FAIL (interrupted mid-averaging)")
            return
        final = statistics.fmean(values)
        if cfg.seeds_per_trial > 1:
            _trial_call(
                trial,
                "trial seed count attribute write",
                "set_user_attr",
                "n_seeds",
                cfg.seeds_per_trial,
            )
            _trial_call(
                trial,
                "trial completed seed count attribute write",
                "set_user_attr",
                "n_seeds_ok",
                len(values),
            )
        _db_call(
            "portfolio trial completion state write", lambda: study.tell(trial, final)
        )
        suffix = (
            f", mean of {len(values)}/{cfg.seeds_per_trial} seeds"
            if cfg.seeds_per_trial > 1
            else ""
        )
        _log(
            f"[tune] trial {trial.number}: COMPLETE value={final} "
            f"({len(cfg.tasks)} segments{suffix})"
        )

    def _one_trial() -> None:
        # Acquire the GPU BEFORE study.ask(): if acquisition is interrupted
        # (Ctrl-C while parked on an over-subscribed pool) it raises before any
        # trial exists, so there is nothing to orphan. Asking first would persist
        # a RUNNING trial that the pre-try interrupt then leaves un-told forever.
        gpu = _acquire_gpu()
        trial = None
        told = False
        try:
            trial = _db_call(
                "trial budget reservation",
                lambda: reserve_study_trial(
                    study,
                    cfg.n_trials,
                    lock_path=study_dir / ".trial-budget.lock",
                    storage=cfg.storage,
                ),
            )
            if trial is None:
                told = True
                return
            if sampler_seed is not None:
                _trial_call(
                    trial,
                    "trial sampler seed record",
                    "set_user_attr",
                    "srb.sampler_seed",
                    sampler_seed,
                )
            (_execute_portfolio if is_portfolio else _execute_single)(trial, gpu)
            told = True
        # Catch cancellation too: no trial may remain RUNNING.
        except BaseException as e:
            if trial is not None and not told:
                try:
                    _trial_call(
                        trial,
                        "trial failure reason attribute write",
                        "set_user_attr",
                        "reason",
                        "interrupted"
                        if isinstance(e, KeyboardInterrupt)
                        else f"error:{type(e).__name__}",
                    )
                    _db_call(
                        "trial failure cleanup state write",
                        lambda: study.tell(trial, state=TrialState.FAIL),
                    )
                except Exception as cleanup_error:  # noqa: BLE001
                    _log(
                        f"[tune] failed to mark trial {trial.number} as failed: "
                        f"{cleanup_error}"
                    )
            if isinstance(e, ProcessCleanupError):
                gpu = None  # A child may still hold the GPU; never lease it again.
            raise
        finally:
            if gpu_pool is not None and gpu is not None:
                gpu_pool.put(gpu)
            _remove_storage_session()

    def _remaining() -> int:
        terminal, running = _study_budget_counts(study, storage=cfg.storage)
        return max(0, cfg.n_trials - terminal - running)

    try:
        with _stop_signal_handlers():
            if cfg.n_workers <= 1:
                while _remaining() > 0 and _within_wall():
                    _one_trial()
            else:
                _run_workers(cfg, study, _one_trial, _within_wall, storage=cfg.storage)
    except BaseException as e:
        # Ctrl-C OR any unexpected worker error (surfaced by f.result()): reap
        # every in-flight child and stop new spawns. Catching only
        # KeyboardInterrupt would orphan the other workers' Isaac trainers on a
        # non-KI failure and hang the non-daemon pool threads at interpreter exit.
        kind = "interrupted" if isinstance(e, KeyboardInterrupt) else "aborting"
        _log(f"[tune] {kind} — terminating in-flight trial children")
        stop_evt.set()  # workers queued pre-spawn must not launch post-reap
        with inflight_lock:
            handles = tuple(inflight_handles)
        for result in cancel_inflight(handles):
            if not result.cleanup_complete:
                _log(
                    "[tune] in-flight child cleanup failed: "
                    f"{result.error or result.status}"
                )
        raise

    _remove_storage_session()

    if not _within_wall() and _remaining() > 0:
        _log(
            f"[tune] stopped early: --max-wall-hours {cfg.max_wall_hours} reached "
            f"({_finished(study, storage=cfg.storage)}/{cfg.n_trials} trials finished)"
        )

    return study


def _run_workers(
    cfg,
    study,
    one_trial,
    within_wall,
    *,
    storage: str | None = None,
) -> None:
    """Schedule up to ``n_workers`` trials; reap children on cancellation."""
    pool = ThreadPoolExecutor(max_workers=cfg.n_workers)
    inflight: set = set()

    def _need_more(n_inflight: int) -> bool:
        terminal, running = _study_budget_counts(study, storage=storage)
        pending_asks = max(0, n_inflight - running)
        return within_wall() and terminal + running + pending_asks < cfg.n_trials

    try:
        while _need_more(len(inflight)) or inflight:
            while _need_more(len(inflight)) and len(inflight) < cfg.n_workers:
                inflight.add(pool.submit(one_trial))
            if not inflight:
                break
            done, inflight = wait(inflight, return_when=FIRST_COMPLETED)
            for f in done:
                f.result()
    finally:
        pool.shutdown(wait=False, cancel_futures=True)


def _register_handle(
    lock: threading.Lock,
    handles: set[OwnedProcessHandle],
    handle: OwnedProcessHandle,
) -> None:
    with lock:
        handles.add(handle)


def _unregister_handle(
    lock: threading.Lock,
    handles: set[OwnedProcessHandle],
    handle: OwnedProcessHandle,
) -> None:
    with lock:
        handles.discard(handle)


def _safe_report(
    trial,
    value: float,
    rung: int,
    db_call: Callable[[str, Callable[[], Any]], Any] | None = None,
) -> None:
    if not math.isnan(value):

        def callback() -> None:
            trial.report(value, rung)

        if db_call is None:
            callback()
        else:
            db_call("trial intermediate value report", callback)


def repair_study(
    study_dir: str | Path, *, stale_hours: float, storage: str | None = None
) -> int:
    """Fail RUNNING trials older than *stale_hours*."""
    if not math.isfinite(stale_hours) or stale_hours < 0:
        raise ValueError("stale_hours must be finite and non-negative")
    from datetime import datetime

    optuna = require_optuna()
    TrialState = optuna.trial.TrialState

    from srb.tuning.study import open_existing_study

    study, _manifest = open_existing_study(Path(study_dir), storage=storage)
    # Optuna's journal storage stamps datetime_start as NAIVE LOCAL time —
    # age it as local time (astimezone() attaches the local zone to a naive
    # stamp); stamping UTC onto it would skew staleness by the host's offset.
    now = datetime.now().astimezone()
    n = 0
    for t in study.get_trials(deepcopy=False):
        if t.state != TrialState.RUNNING or t.datetime_start is None:
            continue
        start = t.datetime_start.astimezone()
        if (now - start).total_seconds() > stale_hours * 3600.0:
            study.tell(t.number, state=TrialState.FAIL)
            n += 1
    _log(f"[tune] repair: failed {n} stale RUNNING trial(s)")
    return n


def summarize(study) -> dict:
    """Return study state counts and top trials."""
    counts: dict[str, int] = {}
    for t in study.get_trials(deepcopy=False):
        counts[t.state.name] = counts.get(t.state.name, 0) + 1
    completed = [
        t
        for t in study.get_trials(deepcopy=False)
        if t.state.name == "COMPLETE" and t.value is not None
    ]
    completed.sort(
        key=lambda t: t.value,
        reverse=study.direction.name == "MAXIMIZE",
    )
    best = completed[0] if completed else None
    return {
        "name": study.study_name,
        "counts": counts,
        "best_trial": None if best is None else best.number,
        "best_value": None if best is None else best.value,
        "top5": [(t.number, t.value) for t in completed[:5]],
    }


def status(study_dir: str | Path, *, storage: str | None = None) -> dict:
    """Summarize a study directory."""
    from srb.tuning.study import open_existing_study

    study, _manifest = open_existing_study(Path(study_dir), storage=storage)
    return summarize(study)


@dataclass
class Check:
    """One preflight result with ``ok``, ``warn``, or ``error`` status."""

    name: str
    status: str
    detail: str = ""


@dataclass
class TuneValidation:
    checks: list[Check]

    @property
    def ok(self) -> bool:
        """Whether all checks avoid hard errors."""
        return all(c.status != "error" for c in self.checks)


def validate_config(cfg: TuneConfig) -> TuneValidation:
    """Validate config and assemble a trial without opening storage or spawning
    children.
    """
    checks: list[Check] = []

    def add(name: str, status: str, detail: str = "") -> None:
        checks.append(Check(name, status, detail))

    invariant_errors = _config_invariant_errors(cfg)
    if "tasks" in invariant_errors:
        add("tasks", "error", invariant_errors["tasks"])
    else:
        kind = "portfolio" if len(cfg.tasks) > 1 else "single-task"
        add("tasks", "ok", f"{len(cfg.tasks)} task(s), {kind}: {', '.join(cfg.tasks)}")
    for name, detail in invariant_errors.items():
        if name != "tasks":
            add(name, "error", detail)
    if invariant_errors:
        return TuneValidation(checks=checks)

    try:
        canonical_algo = canonical_algorithm_label(cfg.algo)
    except ValueError as exc:
        add("adapter", "error", str(exc))
        return TuneValidation(checks=checks)
    cfg = replace(cfg, algo=canonical_algo)

    # Pin syntax and scalar parsing do not need Optuna, an adapter, or a base
    # config. Keep malformed/non-finite pins on the same no-side-effect path as
    # malformed TuneConfig numerics.
    try:
        validate_pins(cfg.pins, reserved=set())
    except PinCollisionError as e:
        add("pins", "error", str(e))
        return TuneValidation(checks=checks)

    is_portfolio = len(cfg.tasks) > 1
    base_task = None if is_portfolio else (cfg.tasks[0] if cfg.tasks else None)
    num_envs = _effective_num_envs(cfg.num_envs)

    adapter = None
    try:
        adapter = resolve_algorithm_adapter(cfg.algo)
        add(
            "adapter",
            "ok",
            f"framework={adapter.framework} local_algo={adapter.local_algo}",
        )
    except Exception as e:  # noqa: BLE001 - report, don't crash
        add("adapter", "error", f"{cfg.algo!r}: {e}")

    if adapter is not None:
        if has_space(adapter.framework, adapter.local_algo, cfg.obs):
            add(
                "search-space",
                "ok",
                space_id(adapter.framework, adapter.local_algo, cfg.obs),
            )
        else:
            hint = (
                "visual profile + space not registered"
                if cfg.obs == "visual"
                else "registered: " + ", ".join(registered_space_ids())
            )
            add(
                "search-space",
                "error",
                f"none for {cfg.algo!r} obs={cfg.obs!r} ({hint})",
            )

    base_agent_cfg: dict[str, Any] | None = None
    try:
        p = resolve_agent_cfg_path(
            cfg.algo, obs=cfg.obs, task=base_task, root=cfg.hyperparams_root
        )
        base_agent_cfg = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        add("base-config", "ok", str(p))
    except Exception as e:  # noqa: BLE001 - report, don't crash
        add("base-config", "error", str(e))

    reserved: set[str] | None = None
    pins_ok = False
    if adapter is not None:
        reserved = {"env.seed", "env.num_envs", *reserved_pin_paths(adapter.framework)}
        try:
            validate_pins(cfg.pins, reserved=reserved)
            add("pins", "ok", f"{len(cfg.pins)} pin(s)")
            pins_ok = True
        except PinCollisionError as e:
            add("pins", "error", str(e))

    # dry-assemble one trial doc against an in-memory study (exercises the
    # space fn, deep_merge, pin↔sampled collision, budget conversion). Skipped
    # when pins already failed the reserved-key check — it would only re-raise
    # the same error before reaching its unique sampled-collision guard.
    if (
        adapter is not None
        and base_agent_cfg is not None
        and reserved is not None
        and pins_ok
    ):
        optuna: Any | None = None
        previous_verbosity: Any | None = None
        try:
            optuna = require_optuna()
            previous_verbosity = optuna.logging.get_verbosity()
            optuna.logging.set_verbosity(optuna.logging.WARNING)
            trial = optuna.create_study(direction="maximize").ask()
            plan = assemble_trial_doc(
                trial,
                cfg,
                framework=adapter.framework,
                local_algo=adapter.local_algo,
                base_agent_cfg=base_agent_cfg,
                num_envs=num_envs,
                reserved=reserved,
            )
            add(
                "assemble",
                "ok",
                f"trial doc assembles ({plan.validation_summary()})",
            )
        except PinCollisionError as e:
            add("assemble", "error", f"pin overlaps a search-space-sampled key: {e}")
        except Exception as e:  # noqa: BLE001 - report, don't crash
            add("assemble", "error", f"{type(e).__name__}: {e}")
        finally:
            if optuna is not None and previous_verbosity is not None:
                optuna.logging.set_verbosity(previous_verbosity)

    if is_portfolio and cfg.normalize == "baseline" and adapter is not None:
        from srb.tuning.baselines import (
            BaselineError,
            check_baselines_staleness,
            load_baseline,
        )

        missing = []
        compatibility_errors = []
        compatibility_warnings = []
        for t in cfg.tasks:
            try:
                entry = load_baseline(t, cfg.algo, cfg.obs, root=cfg.hyperparams_root)
                if entry.objective != cfg.objective:
                    compatibility_errors.append(
                        f"{t}: objective {entry.objective!r} != {cfg.objective!r}"
                    )
                if entry.budget != cfg.trial_env_steps:
                    compatibility_warnings.append(
                        f"{t}: budget {entry.budget} != trial budget {cfg.trial_env_steps}"
                    )
                expected_envs = _effective_num_envs(cfg.num_envs)
                recorded_envs = (
                    _DEFAULT_TRIAL_NUM_ENVS
                    if entry.num_envs is None
                    else entry.num_envs
                )
                if recorded_envs != expected_envs:
                    compatibility_warnings.append(
                        f"{t}: num_envs {recorded_envs} != trial num_envs {expected_envs}"
                    )
            except BaselineError:
                missing.append(t)
        if missing:
            add(
                "baselines",
                "error",
                f"missing/invalid baseline(s) for: {', '.join(missing)} "
                "(run `srb agent tune baseline`)",
            )
        else:
            if compatibility_errors:
                add("baselines", "error", "; ".join(compatibility_errors))
            if compatibility_warnings:
                add("baselines", "warn", "; ".join(compatibility_warnings))
            stale = list(
                check_baselines_staleness(
                    cfg.tasks, cfg.algo, cfg.obs, root=cfg.hyperparams_root
                )
            )
            if stale:
                add("baselines", "warn", "; ".join(stale))
            else:
                add("baselines", "ok", f"{len(cfg.tasks)} baseline(s) present")
    elif is_portfolio and cfg.normalize == "none":
        add(
            "normalize",
            "warn",
            "portfolio with --normalize none aggregates RAW cross-task returns "
            "(scale-dominated); prefer --normalize baseline or rank",
        )

    proj = project_wall_clock(
        trial_env_steps=cfg.trial_env_steps,
        fps_vec_n=cfg.fps_vec_n,
        n_trials=cfg.n_trials,
        n_workers=cfg.n_workers,
        seeds_per_trial=cfg.seeds_per_trial,
        n_tasks=len(cfg.tasks),
    )
    if proj is not None:
        add(
            "wall-clock",
            "ok",
            f"~{proj['hours_per_trial']:.2f} h/trial, ~{proj['study_hours']:.1f} h total",
        )

    return TuneValidation(checks=checks)


@dataclass
class SeedResult:
    seed: int
    value: float | None


@dataclass
class CandidateResult:
    trial_number: int
    original_value: float
    seed_results: list[SeedResult]

    @property
    def values(self) -> list[float]:
        return [s.value for s in self.seed_results if s.value is not None]

    @property
    def n_ok(self) -> int:
        return len(self.values)

    @property
    def mean(self) -> float | None:
        return statistics.fmean(self.values) if self.values else None

    @property
    def std(self) -> float:
        # Sample deviation, not population: the re-run seeds are a sample drawn
        # from every seed the configuration could have taken, and this is
        # reported as "mean +/- std over N seed(s)". `pstdev` understates a
        # three-seed spread by sqrt(2/3), roughly 18%, which is the usual N here.
        # The `> 1` guard below is the one `stdev` requires.
        return statistics.stdev(self.values) if len(self.values) > 1 else 0.0


@dataclass
class ValidationReport:
    study: str
    top_k: int
    seeds: int
    min_ok: int
    validate_steps: int
    candidates: list[CandidateResult]
    original_best: int | None
    winner: CandidateResult | None

    @property
    def winner_changed(self) -> bool:
        return (
            self.winner is not None
            and self.original_best is not None
            and self.winner.trial_number != self.original_best
        )


def validate_study(
    study_dir: str | Path,
    *,
    top_k: int = 3,
    seeds: int = 3,
    seed_base: int = 10_000,
    min_ok: int | None = None,
    validate_steps: int | None = None,
    gpus: list[str] | None = None,
    n_workers: int = 1,
    timeout_s: float | None = None,
    poll_dt: float = 10.0,
    grace_s: float = 60.0,
    keep_artifacts: bool = False,
    storage: str | None = None,
    command_builder: Callable[[TrialSpec], list[str]] = build_train_argv,
) -> ValidationReport:
    """Re-evaluate top completed trials on fresh seeds without updating the study."""
    from srb.tuning.study import open_existing_study

    if top_k < 1:
        raise SystemExit("srb agent tune validate: --top-k must be >= 1")
    if seeds < 1:
        raise SystemExit("srb agent tune validate: --seeds must be >= 1")

    study_dir = Path(study_dir)
    study, manifest = open_existing_study(study_dir, storage=storage)
    if not manifest.algo:
        raise SystemExit(
            "srb agent tune validate: study.yaml has no 'algo' "
            "field, so the framework can't be resolved. Re-run the study with a "
            "current build (which records algo) before validating."
        )
    try:
        manifest = replace(manifest, algo=canonical_algorithm_label(manifest.algo))
    except ValueError as exc:
        raise SystemExit(f"srb agent tune validate: {exc}") from None
    adapter = resolve_algorithm_adapter(manifest.algo)
    framework = adapter.framework
    # Default budget = the study's; an explicit 0/negative is a mistake, not
    # "use the default" (a bare ``x or default`` would silently swallow 0).
    validate_steps = (
        manifest.trial_env_steps if validate_steps is None else int(validate_steps)
    )
    if validate_steps < 1:
        raise SystemExit("srb agent tune validate: --validate-steps must be >= 1")

    min_ok = seeds // 2 + 1 if min_ok is None else int(min_ok)
    min_ok = max(1, min(min_ok, seeds))
    tasks = list(manifest.tasks)
    is_portfolio = len(tasks) > 1

    completed = [
        t
        for t in study.get_trials(deepcopy=False)
        if t.state.name == "COMPLETE" and t.value is not None
    ]
    maximize = manifest.direction == "maximize"
    completed.sort(key=lambda t: t.value, reverse=maximize)
    top = completed[:top_k]
    original_best = completed[0].number if completed else None

    normalizer: PortfolioNormalizer | None = None
    if is_portfolio:
        floor_ref = None
        if manifest.normalize == "baseline":
            from srb.tuning.baselines import BaselineError, load_baseline

            floor_ref = {}
            try:
                for t in tasks:
                    e = load_baseline(t, manifest.algo, manifest.obs)
                    if e.objective != manifest.objective:
                        raise SystemExit(
                            f"srb agent tune validate: baseline for {t!r} records "
                            f"objective {e.objective!r}, but study objective is "
                            f"{manifest.objective!r}"
                        )
                    floor_ref[t] = (e.floor, e.ref)
            except BaselineError as ex:
                raise SystemExit(f"srb agent tune validate: {ex}") from None
        if manifest.normalize == "rank" and validate_steps != manifest.trial_env_steps:
            _log(
                "[tune validate] WARNING: --normalize rank ranks re-run scores "
                "against the study's training-budget pool; at a different "
                "--validate-steps those ranks saturate and the re-ranking can "
                "collapse (winner unchanged). Use the default budget, or a "
                "baseline/none study, for higher-fidelity validation."
            )
        normalizer = PortfolioNormalizer(
            manifest.normalize, floor_ref=floor_ref, study=study
        )

    gpu_pool: queue.Queue[str] | None = None
    if gpus:
        gpu_pool = queue.Queue()
        for g in gpus:
            gpu_pool.put(g)
    boot_lock = threading.Lock()
    first_boot_evt = threading.Event()
    boot_stagger_s = effective_boot_stagger(None, n_workers)
    stop_evt = threading.Event()
    inflight_handles: set[OwnedProcessHandle] = set()
    inflight_lock = threading.Lock()
    val_root = study_dir / "validation"

    def _acquire_gpu() -> str | None:
        if gpu_pool is None:
            return None
        while True:
            try:
                return gpu_pool.get(timeout=0.5)
            except queue.Empty:
                if stop_evt.is_set():
                    raise KeyboardInterrupt from None

    def _load_candidate_plan(trial_number: int) -> TrialPlan:
        """Load a recorded trial config; malformed or missing files raise."""
        cfg_path = study_dir / "trials" / f"{trial_number:03d}" / "trial_cfg.yaml"
        loaded = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ValueError(  # noqa: TRY004 - invalid serialized value, not API type
                f"trial_cfg.yaml is not a mapping: {cfg_path}"
            )
        raw_env = loaded.get("env", {})
        if not isinstance(raw_env, dict):
            raise ValueError(f"recorded env section is not a mapping: {cfg_path}")
        env_section = dict(raw_env)
        agent_full = loaded.get("agent", {})
        if not isinstance(agent_full, dict):
            raise ValueError(f"recorded agent section is not a mapping: {cfg_path}")
        num_envs = _validated_recorded_num_envs(env_section)
        env_section["num_envs"] = num_envs

        agent_full = deep_merge(
            agent_full,
            budget_overrides(
                framework,
                trial_env_steps=validate_steps,
                num_envs=num_envs,
                agent_cfg=agent_full,
            ),
        )
        return TrialPlan(
            agent_section={"agent": agent_full},
            env_section=env_section,
        )

    def _run_seg(
        trial_number: int,
        task: str,
        seed: int,
        seg_dir: Path,
        gpu: str | None,
        plan: TrialPlan,
    ) -> TrialOutcome:
        spec = trial_spec_from_plan(
            plan,
            task=task,
            algo=manifest.algo,
            obs=manifest.obs,
            study=manifest.name,
            trial_number=trial_number,
            trial_env_steps=validate_steps,
            trial_dir=seg_dir,
            seed=seed,
            gpu=gpu,
            timeout_s=timeout_s,
        )
        outcome = run_trial(
            spec,
            report_cb=_no_report,
            should_prune_cb=_never_prune,
            poll_dt=poll_dt,
            objective_field=manifest.objective,
            command_builder=command_builder,
            grace_s=grace_s,
            boot_lock=boot_lock,
            boot_stagger_s=boot_stagger_s,
            stop_evt=stop_evt,
            first_boot_evt=first_boot_evt,
            on_start=lambda handle: _register_handle(
                inflight_lock, inflight_handles, handle
            ),
            on_stop=lambda handle: _unregister_handle(
                inflight_lock, inflight_handles, handle
            ),
        )
        if not keep_artifacts:
            import shutil

            shutil.rmtree(seg_dir / "logs", ignore_errors=True)
        return outcome

    # Load each candidate's recorded doc ONCE (pure fn of the trial number). A
    # missing/corrupt trial_cfg.yaml marks that candidate unscored instead of
    # aborting the whole run and reaping its healthy siblings' re-runs.
    plans: dict[int, TrialPlan | None] = {}
    for c in top:
        try:
            plans[c.number] = _load_candidate_plan(c.number)
        except (OSError, ValueError, yaml.YAMLError) as ex:
            _log(
                f"[tune validate] trial {c.number}: cannot load trial_cfg.yaml "
                f"({ex}); scoring it unscored"
            )
            plans[c.number] = None

    def _score_seed(trial_number: int, seed_idx: int) -> float | None:
        plan = plans.get(trial_number)
        if plan is None:
            return None
        seed = seed_base + seed_idx
        gpu = _acquire_gpu()
        try:
            base = val_root / f"cand-{trial_number:03d}" / f"seed-{seed}"
            if not is_portfolio:
                outcome = _run_seg(trial_number, tasks[0], seed, base, gpu, plan)
                return outcome.value if outcome.state == "COMPLETE" else None
            assert normalizer is not None
            normed: list[float] = []
            for task in tasks:
                outcome = _run_seg(
                    trial_number, task, seed, base / f"seg-{task}", gpu, plan
                )
                if outcome.state != "COMPLETE":
                    return None
                normed.append(normalizer.normalize(task, float(outcome.value)))
            return float(aggregate_scores(normed, manifest.aggregate))
        except ProcessCleanupError:
            gpu = None  # A child may still hold the GPU; never lease it again.
            raise
        finally:
            if gpu_pool is not None and gpu is not None:
                gpu_pool.put(gpu)

    results: dict[tuple[int, int], float | None] = {}

    def _job(trial_number: int, seed_idx: int) -> None:
        results[(trial_number, seed_idx)] = _score_seed(trial_number, seed_idx)

    jobs = [(c.number, s) for c in top for s in range(seeds)]
    try:
        with _stop_signal_handlers():
            if n_workers <= 1:
                for tn, si in jobs:
                    _job(tn, si)
            else:
                pool = ThreadPoolExecutor(max_workers=n_workers)
                try:
                    futures = [pool.submit(_job, tn, si) for tn, si in jobs]
                    # Surface the first failure; a withheld GPU may block others.
                    for f in as_completed(futures):
                        f.result()
                finally:
                    pool.shutdown(wait=False, cancel_futures=True)

    except BaseException as e:
        kind = "interrupted" if isinstance(e, KeyboardInterrupt) else "aborting"
        _log(f"[tune validate] {kind} — terminating in-flight children")
        stop_evt.set()
        with inflight_lock:
            handles = tuple(inflight_handles)
        for result in cancel_inflight(handles):
            if not result.cleanup_complete:
                _log(
                    "[tune validate] in-flight child cleanup failed: "
                    f"{result.error or result.status}"
                )
        raise

    candidates = [
        CandidateResult(
            trial_number=c.number,
            original_value=float(c.value),
            seed_results=[
                SeedResult(seed_base + s, results.get((c.number, s)))
                for s in range(seeds)
            ],
        )
        for c in top
    ]

    # Rank ELIGIBLE candidates (>= min_ok completed seeds) by validated mean;
    # ineligible/unscored ones sink below. A flaky config that only completed a
    # seed or two is not robust however high its surviving-seed mean, so it can
    # never be crowned winner.
    def _eligible(cr: CandidateResult) -> bool:
        return cr.n_ok >= min_ok

    candidates.sort(
        key=lambda cr: (
            not _eligible(cr),
            (-cr.mean if maximize else cr.mean)
            if cr.mean is not None
            else float("inf"),
        )
    )
    winner = next((cr for cr in candidates if _eligible(cr)), None)
    report = ValidationReport(
        study=manifest.name,
        top_k=len(top),
        seeds=seeds,
        min_ok=min_ok,
        validate_steps=validate_steps,
        candidates=candidates,
        original_best=original_best,
        winner=winner,
    )

    val_root.mkdir(parents=True, exist_ok=True)
    _publish_json(
        study_dir / "validation.json",
        {
            "study": report.study,
            "top_k": report.top_k,
            "seeds": report.seeds,
            "min_ok": report.min_ok,
            "validate_steps": report.validate_steps,
            "original_best": report.original_best,
            "winner": None if winner is None else winner.trial_number,
            "winner_changed": report.winner_changed,
            "candidates": [
                {
                    "trial": cr.trial_number,
                    "original_value": cr.original_value,
                    "mean": cr.mean,
                    "std": cr.std,
                    "n_ok": cr.n_ok,
                    "eligible": cr.n_ok >= report.min_ok,
                    "seeds": [
                        {"seed": s.seed, "value": s.value} for s in cr.seed_results
                    ],
                }
                for cr in candidates
            ],
        },
    )
    return report


def _log(msg: str) -> None:
    try:
        from srb.utils import logging

        logging.info(msg)
    except Exception:  # noqa: BLE001 - logging must never break the study
        print(msg)


__all__ = (
    "CandidateResult",
    "Check",
    "PinCollisionError",
    "SeedResult",
    "TrialPlan",
    "TuneConfig",
    "TuneValidation",
    "ValidationReport",
    "apply_pins",
    "assemble_trial_doc",
    "effective_boot_stagger",
    "project_wall_clock",
    "reenqueue_failed",
    "repair_study",
    "run_study",
    "status",
    "summarize",
    "validate_config",
    "validate_pins",
    "validate_study",
)
