"""Run a trial segment in a supervised ``srb agent train`` subprocess."""

from __future__ import annotations

import math
import os
import signal
import threading
import time
from _thread import LockType
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from srb.tuning.metrics import (
    MetricRecord,
    MetricsTailer,
    progress,
    rung_for_progress,
    smoothed_return,
    tail_mean_objective,
)
from srb.tuning.trial_plan import TrialPlan
from srb.utils.process_supervisor import (
    OwnedMonitoredProcess,
    OwnedProcessHandle,
    ProcessCleanupError,
    ProcessKind,
    ProcessResult,
    ProcessStatus,
    cancel_owned_processes,
)


@dataclass(frozen=True)
class TrialSpec:
    task: str
    algo: str
    obs: str
    # Full ``agent:`` config and partial ``env:`` overrides.
    agent_section: Mapping[str, object]
    env_section: Mapping[str, object]
    trial_env_steps: int
    num_envs: int
    metrics_path: Path
    trial_dir: Path
    logdir: Path
    gpu: str | None = None
    timeout_s: float | None = None
    # Copied to the child environment and metadata.
    study: str | None = None
    trial_number: int | None = None


@dataclass(frozen=True)
class TrialOutcome:
    state: str  # "COMPLETE" | "PRUNED" | "FAIL"
    value: float | None
    user_attrs: dict[str, object] = field(default_factory=dict)


def trial_spec_from_plan(
    plan: TrialPlan,
    *,
    task: str,
    algo: str,
    obs: str,
    study: str,
    trial_number: int,
    trial_env_steps: int,
    trial_dir: Path,
    seed: int,
    gpu: str | None,
    timeout_s: float | None,
) -> TrialSpec:
    """Convert an immutable plan into the execution-owned child contract."""
    document = plan.materialize(seed=seed)
    env_section = document.pop("env")
    if not isinstance(env_section, dict):
        raise TypeError("materialized trial env section must be a mapping")
    return TrialSpec(
        task=task,
        algo=algo,
        obs=obs,
        study=study,
        trial_number=trial_number,
        agent_section=document,
        env_section=env_section,
        trial_env_steps=trial_env_steps,
        num_envs=plan.num_envs,
        metrics_path=trial_dir / "metrics.jsonl",
        trial_dir=trial_dir,
        logdir=trial_dir / "logs",
        gpu=gpu,
        timeout_s=timeout_s,
    )


def write_trial_cfg(spec: TrialSpec) -> Path:
    """Write ``trial_cfg.yaml`` with full agent and partial env config.

    ``agent_section`` includes its top-level ``agent`` key and any sibling
    framework sections (such as skrl ``models``, ``memory``, and ``trainer``).
    It is written verbatim as the provenance record.
    """
    spec.trial_dir.mkdir(parents=True, exist_ok=True)
    doc = dict(spec.agent_section)  # full agent-side config (may hold >1 top key)
    doc["env"] = spec.env_section
    path = spec.trial_dir / "trial_cfg.yaml"
    # Replace atomically so export never reads a torn provenance record.
    staging = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        staging.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
        os.replace(staging, path)
    finally:
        staging.unlink(missing_ok=True)
    return path


def build_train_argv(spec: TrialSpec) -> list[str]:
    """Build the child ``srb agent train`` argv — no trailing hydra tokens."""
    cfg_path = (spec.trial_dir / "trial_cfg.yaml").as_posix()
    argv = [
        "srb",
        "agent",
        "train",
        "--env",
        spec.task,
        "--algo",
        spec.algo,
        "--headless",
        "--cfg",
        cfg_path,
        "--logdir",
        spec.logdir.as_posix(),
    ]
    if spec.obs == "visual":
        argv += ["--obs", "visual"]
    return argv


class RolloutError(RuntimeError):
    """A bounded rollout child diverged, timed out, or exited before its budget."""


def cancel_inflight(
    handles: Iterable[OwnedProcessHandle],
) -> tuple[ProcessResult, ...]:
    """Cancel trial owners with supervisor-managed TERM-to-KILL escalation."""

    return tuple(cancel_owned_processes(handles, first_signal=signal.SIGTERM))


def build_rand_argv(task: str, *, logdir: Path, num_envs: int, seed: int) -> list[str]:
    """Build random-policy rollout argv for the normalization floor.

    ``srb agent rand`` needs no ``--cfg`` because the floor ignores the policy.
    Without a config root, ``num_envs`` and ``seed`` safely use standard Hydra tokens.
    """
    return [
        "srb",
        "agent",
        "rand",
        "--env",
        task,
        "--headless",
        "--logdir",
        logdir.as_posix(),
        f"env.num_envs={num_envs}",
        f"env.seed={seed}",
    ]


def run_bounded_rollout(
    argv: list[str],
    *,
    metrics_path: Path,
    budget_env_steps: int,
    num_envs: int,
    gpu: str | None = None,
    poll_dt: float = 10.0,
    grace_s: float = 60.0,
    timeout_s: float | None = None,
) -> list[MetricRecord]:
    """Run ``srb agent rand`` to budget or raise :class:`RolloutError`.

    Returns records; fails on non-finite returns, timeout, early nonzero exit,
    or a clean exit before the budget.
    """
    metrics_path = Path(metrics_path)
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    if metrics_path.exists():
        metrics_path.unlink()

    env = dict(os.environ)
    env["SRB_TUNE_METRICS_PATH"] = metrics_path.as_posix()
    env["SRB_TUNE_METRICS_EVERY"] = str(max(10, num_envs // 4))
    if gpu is not None:
        env["CUDA_VISIBLE_DEVICES"] = str(gpu)

    tailer = MetricsTailer(metrics_path)
    process = OwnedMonitoredProcess.spawn(
        argv,
        kind=ProcessKind.MONITORED,
        deadline_s=timeout_s,
        env=env,
        term_grace_s=grace_s,
        kill_grace_s=5.0,
        reap_grace_s=1.0,
    )
    killed_at_budget = False
    try:
        while process.poll() is None:
            records = tailer.poll()
            if tailer.diverged:
                _terminate_group(process, grace_s=grace_s)
                raise RolloutError("bounded rollout diverged (non-finite return)")
            if records and progress(records[-1].env_steps, budget_env_steps) >= 1.0:
                _terminate_group(process, grace_s=grace_s)
                killed_at_budget = True
                break
            if process.deadline_expired():
                _terminate_group(process, grace_s=grace_s)
                raise RolloutError("bounded rollout timed out before its budget")
            time.sleep(poll_dt)
        returncode = process.poll()
        if not killed_at_budget and returncode not in (0, None):
            raise RolloutError(f"bounded rollout exited early (code {returncode})")
    finally:
        # Keep the owner registered through cleanup and leader reap.
        observed_returncode = process.poll()
        cleanup_result = process.cleanup(
            first_signal=(
                signal.SIGTERM if observed_returncode == 0 else signal.SIGKILL
            )
        )
        if not cleanup_result.cleanup_complete:
            raise ProcessCleanupError(cleanup_result)
    records = list(tailer.poll())
    # A clean early exit is not a full-budget rollout (same rule as run_trial).
    last_env_steps = records[-1].env_steps if records else 0
    if progress(last_env_steps, budget_env_steps) < 1.0:
        raise RolloutError(
            f"bounded rollout ended at {last_env_steps} of {budget_env_steps} env steps"
        )
    return records


def _terminate_group(
    process: OwnedMonitoredProcess,
    *,
    grace_s: float,
) -> ProcessResult:
    """Compatibility adapter to the shared owner's bounded cleanup."""

    del grace_s  # The owner captured this launch's grace at spawn time.
    result = process.terminate()
    if not result.cleanup_complete:
        raise ProcessCleanupError(result)
    return result


def run_trial(
    spec: TrialSpec,
    *,
    report_cb: Callable[[int, float], None],
    should_prune_cb: Callable[[], bool],
    poll_dt: float = 10.0,
    objective_field: str = "ep_return_mean",
    command_builder: Callable[[TrialSpec], list[str]] = build_train_argv,
    grace_s: float = 60.0,
    n_rungs: int = 20,
    smooth_window: int = 5,
    boot_lock: LockType | None = None,
    boot_stagger_s: float = 0.0,
    stop_evt: threading.Event | None = None,
    first_boot_evt: threading.Event | None = None,
    on_start: Callable[[OwnedProcessHandle], None] | None = None,
    on_stop: Callable[[OwnedProcessHandle], None] | None = None,
) -> TrialOutcome:
    """Monitor a trial child and return its :class:`TrialOutcome`.

    The boot lock serializes starts; staggering applies only to cold boots.
    ``stop_evt`` aborts queued starts. Lifecycle events report progress and
    register cancellation ownership.
    """

    def _interrupted() -> bool:
        return stop_evt is not None and stop_evt.is_set()

    if _interrupted():
        return TrialOutcome("FAIL", None, {"reason": "interrupted"})

    write_trial_cfg(spec)
    spec.logdir.mkdir(parents=True, exist_ok=True)
    # Avoid stale rungs when reusing a trial directory.
    if spec.metrics_path.exists():
        spec.metrics_path.unlink()

    env = dict(os.environ)
    env["SRB_TUNE_METRICS_PATH"] = spec.metrics_path.as_posix()
    # Scale flush frequency with vector size.
    env["SRB_TUNE_METRICS_EVERY"] = str(max(10, spec.num_envs // 4))
    if spec.study is not None:
        env["SRB_TUNE_STUDY"] = spec.study
    if spec.trial_number is not None:
        env["SRB_TUNE_TRIAL"] = str(spec.trial_number)
    if spec.gpu is not None:
        env["CUDA_VISIBLE_DEVICES"] = str(spec.gpu)

    def _stagger_needed() -> bool:
        return boot_stagger_s > 0 and (
            first_boot_evt is None or not first_boot_evt.is_set()
        )

    argv = command_builder(spec)
    tailer = MetricsTailer(spec.metrics_path)

    def _spawn() -> tuple[OwnedMonitoredProcess, OwnedProcessHandle]:
        # Register before staggering so Ctrl-C can reap the group leader.
        deadline_s = spec.timeout_s
        if deadline_s is not None and boot_lock is not None and _stagger_needed():
            # Start the trial deadline after the boot stagger.
            deadline_s += boot_stagger_s
        process = OwnedMonitoredProcess.spawn(
            argv,
            kind=ProcessKind.MONITORED,
            deadline_s=deadline_s,
            env=env,
            term_grace_s=grace_s,
            kill_grace_s=5.0,
            reap_grace_s=1.0,
        )
        handle = process.ownership_handle
        if on_start is not None:
            try:
                on_start(handle)
            except BaseException as error:
                cleanup_result = process.cleanup(status=ProcessStatus.INTERRUPTED)
                if not cleanup_result.cleanup_complete:
                    raise ProcessCleanupError(cleanup_result) from error
                if on_stop is not None:
                    on_stop(handle)
                raise
        return process, handle

    if boot_lock is not None:
        with boot_lock:
            if _interrupted():  # the reap may have run while we queued here
                return TrialOutcome("FAIL", None, {"reason": "interrupted"})
            process, handle = _spawn()
            try:
                if _stagger_needed():
                    # Event.wait provides an interruptible delay.
                    if stop_evt is not None:
                        stop_evt.wait(boot_stagger_s)
                    else:
                        time.sleep(boot_stagger_s)
            except BaseException as error:
                cleanup_result = process.cleanup()
                if not cleanup_result.cleanup_complete:
                    # Preserve the trial error if cleanup fails too.
                    raise ProcessCleanupError(cleanup_result) from error
                if on_stop is not None:
                    on_stop(handle)
                raise
    else:
        process, handle = _spawn()

    # Reap queued or staggering children after interruption, including just after spawn;
    # overlapping reaps are idempotent.
    if _interrupted():
        _terminate_group(process, grace_s=grace_s)
        if on_stop is not None:
            on_stop(handle)
        return TrialOutcome("FAIL", None, {"reason": "interrupted"})

    last_rung = 0

    def _objective_missing(
        records: Sequence[MetricRecord], *, min_records: int = 3
    ) -> bool:
        # Fail if the objective metric was never emitted.
        return (
            objective_field != "ep_return_mean"
            and len(records) >= min_records
            and all(getattr(r, objective_field, None) is None for r in records)
        )

    try:
        while process.poll() is None:
            if _interrupted():
                # Driver reaping may overlap; cleanup is idempotent.
                _terminate_group(process, grace_s=grace_s)
                return TrialOutcome("FAIL", None, {"reason": "interrupted"})
            records = tailer.poll()
            if records:
                if first_boot_evt is not None and not first_boot_evt.is_set():
                    first_boot_evt.set()  # metrics flowing → boot proven
                if tailer.diverged:
                    _terminate_group(process, grace_s=grace_s)
                    return TrialOutcome("PRUNED", None, {"diverged": True})
                if _objective_missing(records):
                    _terminate_group(process, grace_s=grace_s)
                    return TrialOutcome(
                        "FAIL", None, {"reason": f"{objective_field}_unavailable"}
                    )
                p = progress(records[-1].env_steps, spec.trial_env_steps)
                rung = rung_for_progress(p, n_rungs)
                if rung > last_rung:
                    val = smoothed_return(
                        records, window=smooth_window, field=objective_field
                    )
                    if math.isfinite(val):
                        last_rung = rung
                        report_cb(rung, val)
                        if should_prune_cb():
                            _terminate_group(process, grace_s=grace_s)
                            return TrialOutcome("PRUNED", None, {})
            if process.deadline_expired():
                _terminate_group(process, grace_s=grace_s)
                return TrialOutcome("FAIL", None, {"reason": "timeout"})
            time.sleep(poll_dt)
    finally:
        # Sweep descendants after a clean leader exit too.
        observed_returncode = process.poll()
        cleanup_result = process.cleanup(
            first_signal=(
                signal.SIGTERM if observed_returncode == 0 else signal.SIGKILL
            )
        )
        if not cleanup_result.cleanup_complete:
            raise ProcessCleanupError(cleanup_result)
        if on_stop is not None:
            on_stop(handle)

    returncode = process.poll()
    if returncode != 0:
        return TrialOutcome("FAIL", None, {"reason": f"exit:{returncode}"})

    records = tailer.poll()
    if records and first_boot_evt is not None and not first_boot_evt.is_set():
        # Metrics prove boot if the child exits during the stagger.
        first_boot_evt.set()
    if tailer.diverged:
        return TrialOutcome("PRUNED", None, {"diverged": True})
    if records and _objective_missing(records, min_records=1):
        return TrialOutcome("FAIL", None, {"reason": f"{objective_field}_unavailable"})
    # Require metrics to prove the step budget before accepting a clean exit.
    reached_env_steps = max((int(r.env_steps) for r in records), default=0)
    if reached_env_steps < int(spec.trial_env_steps):
        return TrialOutcome(
            "FAIL",
            None,
            {
                "reason": "step_budget_incomplete",
                "env_steps": reached_env_steps,
                "trial_env_steps": int(spec.trial_env_steps),
            },
        )
    value = tail_mean_objective(records, spec.trial_env_steps, field=objective_field)
    if math.isnan(value):
        return TrialOutcome("FAIL", None, {"reason": "no_metrics"})
    return TrialOutcome("COMPLETE", float(value), {})


__all__ = (
    "RolloutError",
    "TrialOutcome",
    "TrialSpec",
    "cancel_inflight",
    "_terminate_group",
    "build_rand_argv",
    "build_train_argv",
    "run_bounded_rollout",
    "run_trial",
    "trial_spec_from_plan",
    "write_trial_cfg",
)
