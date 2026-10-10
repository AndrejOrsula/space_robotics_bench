import collections
import copy
import inspect
import json
import time
from collections.abc import Callable, Mapping
from functools import partial as bind
from pathlib import Path
from typing import Any, Protocol

import elements
import embodied
import numpy as np

from srb.integrations.dreamer.checkpoint_transaction import (
    DreamerCheckpointTransaction,
    checkpoint_read_lock,
)
from srb.integrations.dreamer.cleanup import annotate_primary, run_cleanup_steps
from srb.integrations.dreamer.driver import DriverParallelEnv
from srb.utils.resource_budget import (
    MAX_ARRAY_ELEMENTS,
    MAX_IN_MEMORY_FILE_BYTES,
    checked_shape_nbytes,
    require_bounded_int,
)


def _find_srb_wrapper(env):
    """Walk the embodied wrapper chain to SRB's EmbodiedEnvWrapper (or None)."""
    node = env
    for _ in range(16):
        try:
            object.__getattribute__(node, "conversion_latency_stats")
        except AttributeError:
            pass
        else:
            return node
        try:
            node = object.__getattribute__(node, "env")
        except AttributeError:
            return None
    return None


class _DreamerAuditSink(Protocol):
    """Private evidence sink for detached learner-boundary observations."""

    def on_transition(self, transition: Mapping[str, Any], worker: int) -> None: ...

    def on_train_batch(self, batch: Mapping[str, Any]) -> None: ...

    def on_model_batch(self, batch: Mapping[str, Any]) -> None: ...

    def on_update(self, outs: Mapping[str, Any], mets: Mapping[str, Any]) -> None: ...


def _snapshot_for_audit(value: Any) -> Any:
    """Detach audit values so observers cannot mutate learner-owned state."""
    if isinstance(value, Mapping):
        return {key: _snapshot_for_audit(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return tuple(_snapshot_for_audit(item) for item in value)
    if isinstance(value, list):
        return tuple(_snapshot_for_audit(item) for item in value)
    if isinstance(value, set):
        return frozenset(_snapshot_for_audit(item) for item in value)
    if isinstance(value, np.ndarray):
        snapshot = np.array(value, copy=True)
        snapshot.setflags(write=False)
        return snapshot
    if isinstance(value, np.generic):
        return value.copy()
    if hasattr(value, "shape") and hasattr(value, "dtype"):
        try:
            snapshot = np.array(value, copy=True)
        except (TypeError, ValueError):
            pass
        else:
            snapshot.setflags(write=False)
            return snapshot
    return copy.deepcopy(value)


def _wrap_replay_add(
    replay: Any, audit_sink: _DreamerAuditSink | None
) -> Callable[[Mapping[str, Any], int], Any]:
    """Preserve Replay.add while optionally recording its submitted transition."""
    if audit_sink is None:
        return replay.add

    def replay_add(transition: Mapping[str, Any], worker: int) -> Any:
        snapshot = _snapshot_for_audit(transition)
        result = replay.add(transition, worker)
        audit_sink.on_transition(snapshot, worker)
        return result

    return replay_add


def _observe_update(
    audit_sink: _DreamerAuditSink | None,
    outs: Mapping[str, Any],
    mets: Mapping[str, Any],
) -> None:
    if audit_sink is not None:
        audit_sink.on_update(_snapshot_for_audit(outs), _snapshot_for_audit(mets))


def _validate_behavior_policy(policy: Any) -> Callable[..., Any]:
    """Validate the private off-policy control-composer call shape."""
    if not callable(policy):
        raise TypeError(
            "Dreamer private _policy override must be callable and accept "
            "(carry, obs, *, base_policy_result=...)"
        )
    try:
        inspect.signature(policy).bind(
            None,
            {},
            base_policy_result=(None, {}, {}),
        )
    except (TypeError, ValueError) as exc:
        raise TypeError(
            "Dreamer private _policy override must accept "
            "(carry, obs, *, base_policy_result=...)"
        ) from exc
    return policy


def _audit_values_equal(left: Any, right: Any) -> bool:
    """Compare detached policy trees without array truth-value ambiguity."""
    if isinstance(left, Mapping) or isinstance(right, Mapping):
        return (
            isinstance(left, Mapping)
            and isinstance(right, Mapping)
            and set(left) == set(right)
            and all(_audit_values_equal(left[key], right[key]) for key in left)
        )
    if isinstance(left, tuple) or isinstance(right, tuple):
        return (
            isinstance(left, tuple)
            and isinstance(right, tuple)
            and len(left) == len(right)
            and all(_audit_values_equal(a, b) for a, b in zip(left, right))
        )
    if isinstance(left, frozenset) or isinstance(right, frozenset):
        return (
            isinstance(left, frozenset)
            and isinstance(right, frozenset)
            and left == right
        )
    if isinstance(left, np.ndarray) or isinstance(right, np.ndarray):
        if not isinstance(left, np.ndarray) or not isinstance(right, np.ndarray):
            return False
        if left.dtype != right.dtype or left.shape != right.shape:
            return False
        try:
            return bool(np.array_equal(left, right, equal_nan=True))
        except TypeError:
            return bool(np.array_equal(left, right))
    try:
        return bool(left == right)
    except (TypeError, ValueError):
        return False


def _validate_composed_policy_result(
    result: Any,
    base_policy_result: Any,
    base_policy_snapshot: Any,
) -> tuple[Any, Mapping[str, Any], Mapping[str, Any]]:
    """Require private control composition to preserve Dreamer state schema."""
    if not isinstance(base_policy_result, tuple) or len(base_policy_result) != 3:
        raise TypeError("Dreamer agent.policy returned an invalid result")
    if not isinstance(base_policy_snapshot, tuple) or len(base_policy_snapshot) != 3:
        raise TypeError("Dreamer agent.policy snapshot is invalid")
    if not isinstance(result, tuple) or len(result) != 3:
        raise TypeError("Dreamer private _policy override returned an invalid result")
    base_carry, base_actions, base_outs = base_policy_result
    snapshot_carry, snapshot_actions, snapshot_outs = base_policy_snapshot
    carry, actions, outs = result
    if not all(
        isinstance(value, Mapping)
        for value in (
            base_actions,
            base_outs,
            snapshot_actions,
            snapshot_outs,
            actions,
            outs,
        )
    ):
        raise TypeError(
            "Dreamer composed policy actions and outputs must contain mappings"
        )
    if "control" not in snapshot_actions:
        raise ValueError("Dreamer private _policy requires a control action")
    if carry is not base_carry:
        raise ValueError("Dreamer private _policy override replaced recurrent carry")
    if not _audit_values_equal(_snapshot_for_audit(base_carry), snapshot_carry):
        raise ValueError("Dreamer private _policy override mutated recurrent carry")
    if set(base_actions) != set(snapshot_actions) or set(actions) != set(
        snapshot_actions
    ):
        raise ValueError("Dreamer private _policy override changed action schema")
    for key in snapshot_actions:
        if key == "control":
            continue
        if not _audit_values_equal(
            _snapshot_for_audit(base_actions[key]), snapshot_actions[key]
        ) or not _audit_values_equal(
            _snapshot_for_audit(actions[key]), snapshot_actions[key]
        ):
            raise ValueError(
                "Dreamer private _policy override changed a non-control action"
            )
    if outs is not base_outs or set(base_outs) != set(snapshot_outs):
        raise ValueError("Dreamer private _policy override changed output schema")
    if not _audit_values_equal(
        _snapshot_for_audit(base_outs), snapshot_outs
    ) or not _audit_values_equal(_snapshot_for_audit(outs), snapshot_outs):
        raise ValueError("Dreamer private _policy override mutated policy outputs")
    return carry, actions, outs


def _admit_demo_replay_storage(agent, replay, env, args, payload) -> None:
    """Bound native replay arrays, including context and spare chunk rows."""
    from srb.integrations.dreamer.driver import CALLBACK_ROW_METADATA_KEYS

    if (
        getattr(replay, "directory", None) is None
        or getattr(replay, "save_wait", None) is not True
    ):
        raise ValueError("Dreamer demo replay requires synchronous persistent storage")
    sizes = {
        name: require_bounded_int(
            value,
            resource=f"Dreamer demo {name}",
            minimum=1,
            maximum=MAX_ARRAY_ELEMENTS,
        )
        for name, value in (
            ("capacity", replay.capacity),
            ("chunk size", replay.chunksize),
            ("sequence length", replay.length),
            ("environment count", args.envs),
        )
    }
    required_items = len(payload.rows) - sizes["sequence length"] + 1
    if required_items < 1:
        raise ValueError("Dreamer demonstrations do not contain one native sequence")
    if required_items > sizes["capacity"]:
        raise ValueError("Dreamer demonstrations exceed native replay capacity")
    spaces = {
        key: space for key, space in env.obs_space.items() if not key.startswith("log/")
    }
    ext_space = getattr(agent, "ext_space", None)
    if ext_space is None:
        ext_space = agent.model.ext_space
    for mapping in (
        {key: space for key, space in env.act_space.items() if key != "reset"},
        {key: space for key, space in ext_space.items() if key != "consec"},
    ):
        if spaces.keys() & mapping.keys():
            raise ValueError("Dreamer demo native replay fields collide")
        spaces.update(mapping)
    row_bytes = len(CALLBACK_ROW_METADATA_KEYS) * np.dtype(np.int64).itemsize
    for name, space in spaces.items():
        row_bytes += checked_shape_nbytes(
            space.shape,
            space.dtype,
            resource=f"Dreamer demo replay field {name}",
            max_bytes=MAX_IN_MEMORY_FILE_BYTES,
        )
    # Capacity counts window starts; each worker also holds an unfinished
    # window and up to two edge chunks. Demos use a separate worker so online
    # collection cannot append to the last demonstration episode.
    worker_count = sizes["environment count"] + 1
    storage_rows = sizes["capacity"] + worker_count * (
        sizes["sequence length"] - 1 + 2 * sizes["chunk size"]
    )
    storage_bytes = storage_rows * row_bytes
    if storage_bytes > MAX_IN_MEMORY_FILE_BYTES:
        raise ValueError(
            "Dreamer demo configured replay exceeds native array budget: "
            f"{storage_bytes} > {MAX_IN_MEMORY_FILE_BYTES} bytes"
        )
    staged_bytes = len(payload.rows) * row_bytes
    if staged_bytes > MAX_IN_MEMORY_FILE_BYTES:
        raise ValueError(
            "Dreamer demo rows and recurrent context exceed staging budget: "
            f"{staged_bytes} > {MAX_IN_MEMORY_FILE_BYTES} bytes"
        )


def _insert_demo_replay_rows(replay_add, rows, *, is_running) -> None:
    """Keep native row alignment and use a separate demonstration stream."""
    from srb.integrations.dreamer.driver import (
        CALLBACK_ENV_ID_KEY,
        CALLBACK_EPISODE_ID_KEY,
        CALLBACK_STEP_INDEX_KEY,
    )

    episode = -1
    step_index = 0
    for row in rows:
        if not is_running():
            raise RuntimeError("Dreamer demonstration replay insertion cancelled")
        if bool(row["is_first"]):
            episode += 1
            step_index = 0
        replay_add(
            {
                **row,
                CALLBACK_ENV_ID_KEY: np.int64(-1),
                CALLBACK_EPISODE_ID_KEY: np.int64(episode),
                CALLBACK_STEP_INDEX_KEY: np.int64(step_index),
            },
            -1,
        )
        step_index += 1
    if not is_running():
        raise RuntimeError("Dreamer demonstration replay insertion cancelled")


def make_episode_logfn(episodes, logger, epstats):
    """Aggregate per-worker episode stats; log them when an episode ends."""

    @elements.timer.section("logfn")
    def logfn(tran, worker):
        episode = episodes[worker]
        tran["is_first"] and episode.reset()  # type: ignore[misc]
        episode.add("score", tran["reward"], agg="sum")
        episode.add("length", 1, agg="sum")
        episode.add("rewards", tran["reward"], agg="stack")
        for key, value in tran.items():
            if value.dtype == np.uint8 and value.ndim == 3:
                if worker == 0:
                    if value.shape[-1] >= 3:
                        episode.add(f"policy_{key}_ch0-2", value[..., :3], agg="stack")
                        for i in range(3, value.shape[-1]):
                            episode.add(
                                f"policy_{key}_ch{i}",
                                value[..., i : i + 1],
                                agg="stack",
                            )
                    else:
                        for i in range(value.shape[-1]):
                            episode.add(
                                f"policy_{key}_ch{i}",
                                value[..., i : i + 1],
                                agg="stack",
                            )
            elif key.startswith("log/"):
                assert value.ndim == 0, (key, value.shape, value.dtype)
                episode.add(f"{key}/avg", value, agg="avg")
                episode.add(f"{key}/max", value, agg="max")
                episode.add(f"{key}/sum", value, agg="sum")
        if tran["is_last"]:
            result = episode.result()
            logger.add(
                {
                    "score": result.pop("score"),
                    "length": result.pop("length"),
                },
                prefix="episode",
            )
            rew = result.pop("rewards")
            if len(rew) > 1:
                result["reward_rate"] = (np.abs(rew[1:] - rew[:-1]) >= 0.01).mean()
            epstats.add(result)

    return logfn


def train(
    make_agent,
    make_replay,
    make_env,
    make_stream,
    make_logger,
    args,
    is_running: Callable[[], bool],
    resource_metrics: Callable[[], Mapping[str, Any]] | None = None,
    *,
    _audit_sink: _DreamerAuditSink | None = None,
    _policy: Callable[..., Any] | None = None,
    demo_payload: Any = None,
    _prepare_run: Callable[[], None] | None = None,
    _resume_checkpoint: str | Path | None = None,
):
    checkpoint_transaction = DreamerCheckpointTransaction(
        Path(str(args.logdir)),
        demo_replay=None if demo_payload is None else demo_payload.binding,
    )
    resume_plan = checkpoint_transaction.prepare(expected_checkpoint=_resume_checkpoint)
    if _prepare_run is not None:
        _prepare_run()
    agent = make_agent()
    if args.from_checkpoint:
        with checkpoint_read_lock(args.from_checkpoint) as load_path:
            elements.checkpoint.load(load_path, dict(agent=bind(agent.load)))
    replay = make_replay()
    logger = make_logger()

    logdir = elements.Path(args.logdir)
    step = logger.step
    usage = elements.Usage(**args.usage)
    train_agg = elements.Agg()
    epstats = elements.Agg()
    episodes = collections.defaultdict(elements.Agg)
    policy_fps = elements.FPS()
    train_fps = elements.FPS()

    batch_steps = args.batch_size * args.batch_length
    should_train = elements.when.Ratio(args.train_ratio / batch_steps)
    should_log = embodied.LocalClock(args.log_every)
    should_report = embodied.LocalClock(args.report_every)
    should_save = embodied.LocalClock(args.save_every)

    logfn = make_episode_logfn(episodes, logger, epstats)

    driver = DriverParallelEnv(env=make_env(), num_envs=args.envs)
    primary_error = None
    try:
        if demo_payload is not None:
            _admit_demo_replay_storage(agent, replay, driver.env, args, demo_payload)
        # Private off-policy evidence support only: this is not normal
        # Dreamer-policy verification. Learner updates below still call the
        # real agent.train on data collected by this behavior policy.
        behavior_policy = (
            None if _policy is None else _validate_behavior_policy(_policy)
        )

        def policy(*policy_args, **policy_kwargs):
            base_policy_result = agent.policy(
                *policy_args,
                **policy_kwargs,
                mode="train",
            )
            if behavior_policy is None:
                return base_policy_result
            base_policy_snapshot = _snapshot_for_audit(base_policy_result)
            result = behavior_policy(
                *policy_args,
                **policy_kwargs,
                base_policy_result=base_policy_result,
            )
            return _validate_composed_policy_result(
                result,
                base_policy_result,
                base_policy_snapshot,
            )

        driver.on_step(lambda tran, _: step.increment())
        driver.on_step(lambda tran, _: policy_fps.step())
        replay_add = _wrap_replay_add(replay, _audit_sink)
        driver.on_step(replay_add)
        driver.on_step(logfn)

        update_counter = [0]

        def trainfn(tran, worker):
            if len(replay) < args.batch_size * args.batch_length:
                return
            for _ in range(should_train(step)):
                with elements.timer.section("stream_next"):
                    batch = next(stream_train)
                carry_train[0], outs, mets = agent.train(carry_train[0], batch)
                _observe_update(_audit_sink, outs, mets)
                train_fps.step(batch_steps)
                update_counter[0] += 1
                if "replay" in outs:
                    replay.update(outs["replay"])
                train_agg.add(mets, prefix="train")

        driver.on_step(trainfn)

        cp = elements.Checkpoint(logdir / "ckpt")
        cp.step = step
        cp.agent = agent
        cp.replay = replay
        if resume_plan is None:
            if demo_payload is not None:
                if len(replay):
                    raise ValueError("Dreamer demo prefill requires fresh replay")
                from srb.integrations.dreamer.demo_context import stage_demo_replay_rows

                staged_rows = stage_demo_replay_rows(
                    agent,
                    demo_payload.rows,
                    obs_space={
                        key: space
                        for key, space in driver.env.obs_space.items()
                        if not key.startswith("log/")
                    },
                    act_space={
                        key: space
                        for key, space in driver.env.act_space.items()
                        if key != "reset"
                    },
                    is_running=is_running,
                )
                _insert_demo_replay_rows(replay_add, staged_rows, is_running=is_running)
                del staged_rows
            checkpoint_transaction.commit(
                checkpoint=cp,
                step=step,
                replay=replay,
            )
        else:
            checkpoint_transaction.restore(
                resume_plan,
                checkpoint=cp,
                step=step,
                replay=replay,
            )
            print(
                "Training-state resume: step="
                f"{int(step)}, replay_items={len(replay)} "
                f"(manifest: generation={resume_plan.generation_id}, "
                f"step={resume_plan.step}, replay_items={resume_plan.replay_items})"
            )

        # Prefetch starts native threads in __iter__. Start only after all
        # demonstrations are inserted or the committed agent/replay is loaded.
        stream_train = iter(agent.stream(make_stream(replay, "train")))
        stream_report = iter(agent.stream(make_stream(replay, "report")))
        carry_train = [agent.init_train(args.batch_size)]
        carry_report = agent.init_report(args.batch_size)

        def _save_checkpoint():
            checkpoint_transaction.commit(
                checkpoint=cp,
                step=step,
                replay=replay,
            )

        print("Start training loop")
        driver.reset(agent.init_policy)
        loop_start_s = time.time()
        loop_start_step = int(step)
        loop_start_updates = update_counter[0]
        while step < args.steps:
            if not is_running():
                break
            driver(policy, steps=10)

            if should_report(step) and len(replay):
                agg = elements.Agg()
                for _ in range(args.consec_report * args.report_batches):
                    carry_report, mets = agent.report(carry_report, next(stream_report))
                    agg.add(mets)
                logger.add(agg.result(), prefix="report")

            if should_log(step):
                logger.add(train_agg.result())
                logger.add(epstats.result(), prefix="epstats")
                logger.add(replay.stats(), prefix="replay")
                logger.add(usage.stats(), prefix="usage")
                logger.add({"fps/policy": policy_fps.result()})
                logger.add({"fps/train": train_fps.result()})
                timer_stats = elements.timer.stats()
                if "summary" in timer_stats:
                    logger.add({"timer": timer_stats["summary"]})
                logger.write()

            if should_save(step):
                _save_checkpoint()

        # Final synchronous checkpoint + generation manifest so short runs
        # always leave a resumable state at the loop boundary.
        _save_checkpoint()

        _write_throughput_report(
            logdir=Path(str(logdir)),
            env=driver.env,
            elapsed_s=time.time() - loop_start_s,
            env_steps=int(step) - loop_start_step,
            learner_updates=update_counter[0] - loop_start_updates,
            batch_steps=batch_steps,
            resource_metrics=resource_metrics,
        )

    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        cleanup_errors = run_cleanup_steps(
            (("logger.close", logger.close), ("driver.close", driver.close))
        )
        if primary_error is not None:
            annotate_primary(primary_error, cleanup_errors)
        elif cleanup_errors:
            raise cleanup_errors[0][1]


def _write_throughput_report(
    *,
    logdir: Path,
    env,
    elapsed_s: float,
    env_steps: int,
    learner_updates: int,
    batch_steps: int,
    resource_metrics: Callable[[], Mapping[str, Any]] | None = None,
) -> None:
    """Measure the actual simulator->learner boundary; never claims zero-copy."""
    report: dict = {
        "elapsed_s": elapsed_s,
        "env_steps": env_steps,
        "env_steps_per_s": (env_steps / elapsed_s) if elapsed_s > 0 else None,
        "learner_updates": learner_updates,
        "learner_updates_per_s": (
            learner_updates / elapsed_s if elapsed_s > 0 else None
        ),
        "batch_steps_per_update": batch_steps,
        "observation_boundary": "torch->cpu-numpy->jax (not zero-copy)",
    }

    wrapper = _find_srb_wrapper(env)
    if wrapper is not None:
        report["boundary_conversion_latency"] = wrapper.conversion_latency_stats()

    if resource_metrics is not None:
        metrics = resource_metrics()
        if not isinstance(metrics, Mapping):
            raise TypeError("Dreamer resource metrics callback must return a mapping")
        report.update(metrics)

    path = logdir / "throughput.json"
    path.write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")
    print(f"Throughput report saved to {path}")
