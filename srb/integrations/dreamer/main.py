from __future__ import annotations

# ruff: noqa: E402
import inspect
import math
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from functools import partial as bind
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Literal

from srb.integrations.eval_quota import eval_episode_budget
from srb.utils.path import new_eval_logdir
from srb.utils.process_resources import ProcessTreeRssSampler

from .process_protocol import DreamerProcessProtocolError

if TYPE_CHECKING:
    import gymnasium

    from srb._typing import AnyEnv, AnyEnvCfg
    from srb.integrations.dreamer.process_env import DreamerProcessEnv

_AUDIT_SINK_RUN_OPTION = "_audit_sink"
_POLICY_RUN_OPTION = "_policy"
_learner_imports_ready = False

# Populated lazily; declarations keep parent imports safe and satisfy type checkers.
UPSTREAM_CONFIG_PATH: Path | None = None
EmbodiedEnvWrapper: Any
portal: Any = None
train: Callable[..., Any] | None = None
train_real: Callable[..., Any] | None = None
eval_only: Callable[..., Any] | None = None


def _reject_bounded_eval(workflow: str) -> None:
    """Reject bounded evaluation requests before provider/checkpoint allocation."""
    if workflow != "eval":
        return
    if eval_episode_budget() is None:
        return
    raise ValueError(
        "Dreamer deterministic evaluation is unavailable: the pinned "
        "provider samples its policy and posterior even with mode='eval'; "
        "unset SRB_EVAL_EPISODES for legacy open-loop evaluation"
    )


@dataclass(frozen=True, slots=True)
class DreamerLearnerRequest:
    """All runtime inputs for a learner operating on an already-built env."""

    agent_cfg: Mapping[str, Any]
    task_id: str
    base_seed: int
    env_count: int
    logdir: Path
    model: Path | None
    is_running: Callable[[], bool]
    evaluation_logdir: Path | None = None
    workflow: Literal["train", "train_real", "eval"] = "train"
    continue_training: bool | None = None
    run_metadata: Mapping[str, Any] | None = None
    resource_metrics: Callable[[], Mapping[str, Any]] | None = None
    eval_env_count: int | None = None
    checkpoint_preflight: Callable[[str], None] | None = None
    run_options: Mapping[str, Any] = field(default_factory=dict)


def _load_learner_modules() -> None:
    """Load Dreamer-owned modules only after a learner branch is selected."""
    global _learner_imports_ready
    global _safe_update_config, dreamer_task_config_is_strict
    global split_launcher_replay_controls, find_dreamer_checkpoint
    global elements, embodied, np, yaml, dreamer_agent, dreamer_main, logging

    if _learner_imports_ready:
        return

    # Dreamer's XLA flags must be installed before importing Dreamer/JAX.
    from srb.integrations.dreamer import configure_xla_flags  # isort: skip

    configure_xla_flags()

    import elements
    import embodied
    import numpy as np
    from dreamerv3 import agent as dreamer_agent
    from dreamerv3 import main as dreamer_main
    from ruamel import yaml

    from srb.integrations.dreamer.config import (
        _safe_update_config,
        dreamer_task_config_is_strict,
        split_launcher_replay_controls,
    )
    from srb.utils import logging
    from srb.utils.checkpoint import find_dreamer_checkpoint

    globals()["UPSTREAM_CONFIG_PATH"] = Path(dreamer_agent.__file__).parent.joinpath(
        "configs.yaml"
    )
    _learner_imports_ready = True


def _local_resource_metrics() -> Mapping[str, Any]:
    """Collect simulator-side metrics only for the local simulator adapter."""
    report: dict[str, Any] = {}
    try:
        import psutil

        report["host_rss_bytes"] = psutil.Process().memory_info().rss
    except Exception:
        report["host_rss_bytes"] = None
    try:
        import torch

        if torch.cuda.is_available():
            report["torch_cuda_allocated_bytes"] = int(torch.cuda.memory_allocated())
            report["torch_cuda_reserved_bytes"] = int(torch.cuda.memory_reserved())
    except Exception:
        pass
    try:
        import jax

        stats = jax.local_devices()[0].memory_stats() or {}
        report["jax_device_bytes_in_use"] = stats.get("bytes_in_use")
        report["jax_device_peak_bytes_in_use"] = stats.get("peak_bytes_in_use")
    except Exception:
        pass
    return report


class _LivenessAdapter:
    """Adapt the callable learner seam for legacy eval/train-real helpers."""

    def __init__(self, is_running: Callable[[], bool]) -> None:
        self._is_running = is_running

    def is_running(self) -> bool:
        return self._is_running()


def _prepare_learner_env(env: Any):
    """Apply the established Dreamer action/dtype adapters to exact env spaces."""
    learner_env = env
    for name, space in learner_env.act_space.items():
        if not space.discrete:
            learner_env = embodied.wrappers.NormalizeAction(learner_env, name)
    learner_env = embodied.wrappers.UnifyDtypes(learner_env)
    for name, space in learner_env.act_space.items():
        if not space.discrete:
            learner_env = embodied.wrappers.ClipAction(learner_env, name)
    return learner_env


def _thaw_metadata(value: Any) -> Any:
    """Copy JSON-shaped metadata out of read-only process seams."""

    if isinstance(value, Mapping):
        return {str(key): _thaw_metadata(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_thaw_metadata(item) for item in value]
    return value


_MISSING_ENV_ATTRIBUTE = object()


def _read_declared_env_attribute(env: Any, name: str) -> Any:
    """Read declared attributes through Dreamer's ``.env`` chain. Static lookup
    preserves missing-attribute errors; declared descriptors use their normal getter."""
    source = env
    seen: set[int] = set()
    for _ in range(16):
        if id(source) in seen:
            raise RuntimeError("Dreamer environment wrapper chain contains a cycle")
        seen.add(id(source))
        if inspect.getattr_static(source, name, _MISSING_ENV_ATTRIBUTE) is not (
            _MISSING_ENV_ATTRIBUTE
        ):
            return object.__getattribute__(source, name)
        if inspect.getattr_static(source, "env", _MISSING_ENV_ATTRIBUTE) is (
            _MISSING_ENV_ATTRIBUTE
        ):
            return _MISSING_ENV_ATTRIBUTE
        source = object.__getattribute__(source, "env")
    raise RuntimeError("Dreamer environment wrapper chain exceeds 16 layers")


def _process_run_metadata(
    env: Any,
    request: DreamerLearnerRequest,
) -> dict[str, Any]:
    """Build the learner-owned metadata publication for process training."""

    raw = request.run_metadata
    metadata = _thaw_metadata(raw) if isinstance(raw, Mapping) else {}
    if not isinstance(metadata, dict):  # pragma: no cover - helper invariant
        metadata = {}

    run = metadata.get("run")
    run_values = dict(run) if isinstance(run, Mapping) else {}
    # Phase 1 has no simulator env; worker-composed values override run metadata.
    run_values.update(
        {
            "algorithm": "dreamer",
            "env_id": request.task_id,
            "seed": request.base_seed,
            "workflow": request.workflow,
            "agent_cfg": _thaw_metadata(request.agent_cfg),
        }
    )
    metadata["run"] = run_values

    process_values = metadata.get("dreamer_process")
    process = dict(process_values) if isinstance(process_values, Mapping) else {}
    process["effective_seed"] = request.base_seed
    effective_config = request.run_options.get("effective_config")
    if effective_config is not None:
        process["effective_config"] = _thaw_metadata(effective_config)
    metadata["dreamer_process"] = process

    contract = _read_declared_env_attribute(env, "landscaping_contract")
    if contract is _MISSING_ENV_ATTRIBUTE:
        contract = None
    if contract is None:
        unwrapped = _read_declared_env_attribute(env, "unwrapped")
        if unwrapped is _MISSING_ENV_ATTRIBUTE:
            unwrapped = env
        metadata_hook = _read_declared_env_attribute(
            unwrapped, "landscaping_contract_metadata"
        )
        if callable(metadata_hook):
            from srb.integrations.dreamer.agent_runtime import (
                DreamerAgentRuntimeError,
            )
            from srb.utils.metadata import collect_landscaping_contract_metadata

            try:
                contract = collect_landscaping_contract_metadata(
                    env=env,
                    env_id=request.task_id,
                    algo="dreamer",
                )
            except (
                AttributeError,
                OSError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                raise DreamerAgentRuntimeError(
                    "local landscaping contract metadata collection failed"
                ) from exc
            if contract is None:
                raise DreamerAgentRuntimeError(
                    "local landscaping contract metadata collection failed"
                )
    if contract is not None:
        from srb.integrations.dreamer.agent_runtime import DreamerAgentRuntimeError
        from srb.integrations.dreamer.process_protocol import (
            validate_landscaping_contract,
        )

        try:
            contract = validate_landscaping_contract(contract)
        except DreamerProcessProtocolError as exc:
            raise DreamerAgentRuntimeError(
                "local landscaping contract metadata is not a bounded flat block"
            ) from exc
        metadata["landscaping_contract"] = _thaw_metadata(contract)
    return metadata


def run(
    workflow: Literal["train", "train_real", "eval"],
    env: "AnyEnv | gymnasium.Env",
    sim_app: Any,
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    agent_cfg: Mapping[str, Any],
    logdir: Path,
    model: Path | None,
    continue_training: bool | None = None,
    evaluation_logdir: Path | None = None,
    **kwargs: Any,
):
    """Run Dreamer for a local simulator env through the shared learner seam."""
    _reject_bounded_eval(workflow)
    from srb.integrations.dreamer.wrapper import EmbodiedEnvWrapper

    run_metadata = kwargs.pop("run_metadata", None)
    wrapped_env = EmbodiedEnvWrapper(env)

    def checkpoint_preflight(checkpoint: str) -> None:
        from srb.integrations.dreamer.policy import (
            validate_landscaping_checkpoint_contract,
        )

        validate_landscaping_checkpoint_contract(
            checkpoint,
            env=wrapped_env.env,
            wrapped_env=wrapped_env,
        )

    env_count = env_cfg.scene.num_envs if env_cfg else 1
    eval_env_count = env_cfg.scene.num_envs if env_cfg else 0
    return run_learner(
        wrapped_env,
        DreamerLearnerRequest(
            agent_cfg=agent_cfg,
            task_id=env_id,
            base_seed=env_cfg.seed if env_cfg else 0,
            env_count=env_count,
            logdir=Path(logdir),
            evaluation_logdir=(
                Path(evaluation_logdir) if evaluation_logdir is not None else None
            ),
            model=Path(model) if model is not None else None,
            is_running=sim_app.is_running,
            workflow=workflow,
            continue_training=continue_training,
            run_metadata=run_metadata,
            resource_metrics=_local_resource_metrics,
            eval_env_count=eval_env_count,
            checkpoint_preflight=checkpoint_preflight,
            run_options=dict(kwargs),
        ),
    )


def run_process(
    env: "DreamerProcessEnv",
    agent_cfg: Mapping[str, Any],
    task_id: str,
    base_seed: int,
    env_count: int,
    logdir: Path,
    model: Path | None,
    continue_training: bool | None = None,
    *,
    run_metadata: Mapping[str, Any] | None = None,
    resource_metrics: Callable[[], Mapping[str, Any]] | None = None,
    eval_env_count: int | None = None,
    workflow: Literal["train"] = "train",
    **run_options: Any,
):
    """Run the parent learner against a prebuilt process-backed env."""
    with ProcessTreeRssSampler() as rss_sampler:

        def measured_resources() -> Mapping[str, Any]:
            measured = dict(rss_sampler.metrics())
            if resource_metrics is None:
                return measured
            injected = resource_metrics()
            if not isinstance(injected, Mapping):
                raise TypeError(
                    "Dreamer resource metrics callback must return a mapping"
                )
            # An explicit caller may own a wider sampling window than this
            # learner-local sampler. Its values are
            # authoritative for duplicate metric names.
            measured.update(injected)
            return measured

        return run_learner(
            env,
            DreamerLearnerRequest(
                agent_cfg=agent_cfg,
                task_id=task_id,
                base_seed=base_seed,
                env_count=env_count,
                logdir=Path(logdir),
                evaluation_logdir=None,
                model=Path(model) if model is not None else None,
                is_running=env.is_running,
                workflow=workflow,
                continue_training=continue_training,
                run_metadata=run_metadata,
                resource_metrics=measured_resources,
                eval_env_count=eval_env_count,
                run_options=run_options,
            ),
        )


def make_learner_stream(
    config: Any,
    replay: Any,
    mode: str,
    *,
    _audit_sink: Any = None,
):
    """Build an upstream stream without SRB's callback-only replay metadata."""
    _load_learner_modules()
    from srb.integrations.dreamer.driver import CALLBACK_ROW_METADATA_KEYS

    stream = dreamer_main.make_stream(config, replay, mode)

    if _audit_sink is None or mode != "train":
        return (
            {
                key: value
                for key, value in batch.items()
                if key not in CALLBACK_ROW_METADATA_KEYS
            }
            for batch in stream
        )

    from srb.integrations.dreamer.train import _snapshot_for_audit

    def audited_stream():
        for batch in stream:
            _audit_sink.on_train_batch(_snapshot_for_audit(batch))
            model_batch = {
                key: value
                for key, value in batch.items()
                if key not in CALLBACK_ROW_METADATA_KEYS
            }
            _audit_sink.on_model_batch(_snapshot_for_audit(model_batch))
            yield model_batch

    return audited_stream()


def run_learner(env: Any, request: DreamerLearnerRequest):
    """Execute the shared Dreamer learner runtime for an already-built env."""
    _reject_bounded_eval(request.workflow)
    demo_dataset = request.run_options.get("demo_dataset")
    demo_max_transitions = request.run_options.get("demo_max_transitions")
    has_demo_request = demo_dataset is not None or demo_max_transitions is not None
    if has_demo_request:
        if request.workflow != "train":
            raise ValueError(
                "Dreamer demonstration replay prefill is only supported for training"
            )
        replay_config = request.agent_cfg.get("replay")
        if (
            not isinstance(replay_config, Mapping)
            or replay_config.get("save") is not True
            or replay_config.get("save_wait") is not True
        ):
            raise ValueError(
                "Dreamer demonstration replay requires "
                "replay.save=true and replay.save_wait=true"
            )
        from srb.integrations.algorithms import validate_demo_training_inputs

        validate_demo_training_inputs("dreamer", demo_dataset, demo_max_transitions)
    _load_learner_modules()

    agent_cfg, replay_controls = split_launcher_replay_controls(dict(request.agent_cfg))
    save_replay = replay_controls["save"]
    replay_save_wait = replay_controls["save_wait"]

    from_checkpoint = find_dreamer_checkpoint(
        request.logdir,
        request.model,
        request.workflow,
        request.continue_training,
        logger=logging,
    )
    if not from_checkpoint and request.continue_training:
        logging.info(
            "continue_training requested but no checkpoint found — starting fresh"
        )

    checkpoint_operation = "fresh"
    resume_checkpoint: Path | None = None
    if request.workflow == "eval":
        checkpoint_operation = "evaluation_load"
    elif from_checkpoint:
        source = Path(str(from_checkpoint)).resolve()
        if (
            request.workflow == "train"
            and (request.logdir.resolve() / "ckpt_generation.json").is_file()
            and (request.logdir.resolve() / "ckpt") in (source, *source.parents)
        ):
            checkpoint_operation = "training_state_resume"
            # The train/transaction owner compares this canonical request with
            # the authoritative manifest under its run lock before allocation.
            resume_checkpoint = source
            from_checkpoint = ""
        else:
            checkpoint_operation = "agent_import"
    logging.info(f"Dreamer checkpoint operation: {checkpoint_operation}")

    if request.workflow == "train_real" and from_checkpoint:
        source_ckpt = Path(from_checkpoint).resolve()
        target_root = request.logdir.resolve()
        if source_ckpt.parent.parent == target_root:
            request = replace(
                request,
                logdir=request.logdir.joinpath("real_transfer", elements.timestamp()),
            )

    if from_checkpoint:
        logging.info(f"Loading model from {from_checkpoint}")

    logdir = request.logdir
    if request.workflow == "eval":
        logdir = (
            Path(request.evaluation_logdir)
            if request.evaluation_logdir is not None
            else new_eval_logdir(request.logdir)
        )

    logdir = elements.Path(logdir)

    configs: dict[str, Any] = yaml.YAML(typ="safe").load(
        globals()["UPSTREAM_CONFIG_PATH"].read_text()
    )
    config = elements.Config(configs["defaults"])
    config = config.update(
        {
            "seed": request.base_seed,
            "task": request.task_id.replace("/", "_"),
            "logdir": logdir,
            "run.from_checkpoint": from_checkpoint,
            "run.envs": request.env_count,
            "run.eval_envs": (
                request.eval_env_count
                if request.eval_env_count is not None
                else request.env_count
            ),
        }
    )

    runtime_agent_cfg: dict[str, Any] = {}
    if isinstance(agent_cfg, Mapping):
        runtime_agent_cfg = dict(agent_cfg)
        nested_agent_cfg = runtime_agent_cfg.get("agent_cfg")
        if isinstance(nested_agent_cfg, Mapping):
            runtime_agent_cfg.pop("agent_cfg", None)
            runtime_agent_cfg.update(nested_agent_cfg)

    strict_task_config = dreamer_task_config_is_strict(
        request.task_id, agent_cfg=runtime_agent_cfg
    )
    config, _ = _safe_update_config(
        config, runtime_agent_cfg, strict=strict_task_config
    )

    preflight_checkpoint = resume_checkpoint or from_checkpoint
    if preflight_checkpoint and request.checkpoint_preflight is not None:
        if resume_checkpoint is not None:
            from srb.integrations.dreamer.checkpoint_transaction import (
                checkpoint_read_lock,
            )

            with checkpoint_read_lock(resume_checkpoint) as admitted_checkpoint:
                request.checkpoint_preflight(str(admitted_checkpoint))
        else:
            request.checkpoint_preflight(str(preflight_checkpoint))

    learner_env = _prepare_learner_env(env)
    learner_obs_space = {
        key: space
        for key, space in learner_env.obs_space.items()
        if not key.startswith("log/")
    }
    learner_act_space = {
        key: space for key, space in learner_env.act_space.items() if key != "reset"
    }

    demo_payload = None
    if has_demo_request:
        from srb.integrations.dreamer.demo_replay import build_demo_replay_payload

        demo_payload = build_demo_replay_payload(
            demo_dataset,
            learner_env,
            demo_max_transitions,
            config=config,
            task_id=request.task_id,
        )

    process_training = isinstance(request.run_metadata, Mapping) and (
        "dreamer_process" in request.run_metadata
        or "effective_config" in request.run_options
    )
    published_run_metadata: dict[str, Any] | None = None
    if request.workflow == "train" and (process_training or demo_payload is not None):
        published_run_metadata = _process_run_metadata(env, request)
        # Keep the identity supplied by the CLI so demo replay publication can
        # append its binding to the same in-memory record after this write.
        if isinstance(request.run_metadata, dict):
            request.run_metadata.clear()
            request.run_metadata.update(published_run_metadata)
            published_run_metadata = request.run_metadata

    def prepare_run() -> None:
        # Training calls this only after the authoritative generation binding
        # matches. A rejected resume must not overwrite the previous run's
        # configuration or descriptive metadata.
        logdir.mkdir()
        print("Agent logdir:", logdir)
        config.save(logdir / "config.yaml")
        _report_preallocation(
            wrapped_env=env,
            config=config,
            env_id=request.task_id,
            logdir=logdir,
            checkpoint_operation=checkpoint_operation,
            save_replay=save_replay,
            replay_save_wait=replay_save_wait,
            strict_task_config=strict_task_config,
            run_metadata=request.run_metadata,
        )
        if published_run_metadata is not None:
            from srb.utils.metadata import write_run_metadata

            write_run_metadata(published_run_metadata, Path(str(logdir)))
        if demo_payload is not None:
            from srb.utils.metadata import persist_demo_replay_metadata

            persist_demo_replay_metadata(
                published_run_metadata or request.run_metadata,
                logdir=Path(str(logdir)),
                payload=demo_payload,
                max_transitions=demo_max_transitions,
                framework="Dreamer",
            )

    if request.workflow != "train":
        prepare_run()

    learner_agent_config = elements.Config(
        **config.agent,
        logdir=config.logdir,
        seed=config.seed,
        jax=config.jax,
        batch_size=config.batch_size,
        batch_length=config.batch_length,
        replay_context=config.replay_context,
        report_length=config.report_length,
        replica=config.replica,
        replicas=config.replicas,
    )
    run_args = elements.Config(
        **config.run,
        replica=config.replica,
        replicas=config.replicas,
        logdir=config.logdir,
        batch_size=config.batch_size,
        batch_length=config.batch_length,
        report_length=config.report_length,
        consec_train=config.consec_train,
        consec_report=config.consec_report,
        replay_context=config.replay_context,
    )
    make_agent = lambda: dreamer_agent.Agent(  # noqa: E731
        learner_obs_space, learner_act_space, learner_agent_config
    )
    make_replay_fn = lambda: make_replay(  # noqa: E731
        config, "replay" if save_replay else None, save_wait=replay_save_wait
    )
    make_env = lambda: learner_env  # noqa: E731

    make_stream = bind(make_learner_stream, config)

    make_logger = bind(dreamer_main.make_logger, config)

    if request.workflow == "train":
        audit_sink = request.run_options.get(_AUDIT_SINK_RUN_OPTION)
        policy = request.run_options.get(_POLICY_RUN_OPTION)
        if audit_sink is not None:
            make_stream = bind(
                make_learner_stream,
                config,
                _audit_sink=audit_sink,
            )
        from srb.integrations.dreamer.train import train

        return train(
            make_agent,
            make_replay_fn,
            make_env,
            make_stream,
            make_logger,
            run_args,
            is_running=request.is_running,
            resource_metrics=request.resource_metrics,
            _audit_sink=audit_sink,
            _policy=policy,
            demo_payload=demo_payload,
            _prepare_run=prepare_run,
            _resume_checkpoint=resume_checkpoint,
        )
    if request.workflow == "eval":
        from srb.integrations.dreamer.eval import eval_only

        return eval_only(
            make_agent,
            make_env,
            make_logger,
            run_args,
            _LivenessAdapter(request.is_running),
        )
    from srb.integrations.dreamer.train_real import train_real

    return train_real(
        make_agent,
        make_replay_fn,
        make_env,
        make_stream,
        make_logger,
        run_args,
        _LivenessAdapter(request.is_running),
    )


def _git_head(path: Path) -> str | None:
    """Resolve a repository HEAD SHA (shared implementation with run metadata)."""
    from srb.utils.metadata import _git

    return _git("rev-parse", "HEAD", cwd=Path(path))


def _flat_config_items(config) -> dict[str, Any]:
    """Flatten an elements.Config into dotted-key scalars (best effort)."""
    flat: dict[str, Any] = {}

    def _walk(node: Any, prefix: str) -> None:
        items = None
        if hasattr(node, "items"):
            try:
                items = list(node.items())
            except Exception:
                items = None
        if items is None:
            flat[prefix] = node
            return
        for key, value in items:
            _walk(value, f"{prefix}.{key}" if prefix else str(key))

    _walk(config, "")
    return flat


def _report_preallocation(
    *,
    wrapped_env: Any,
    config,
    env_id: str,
    logdir,
    checkpoint_operation: str,
    save_replay: bool,
    replay_save_wait: bool,
    strict_task_config: bool,
    run_metadata: Mapping[str, Any] | None,
) -> None:
    obs_space = {
        key: space
        for key, space in wrapped_env.obs_space.items()
        if not key.startswith("log/")
    }
    obs_bytes_per_step = 0
    obs_shapes: dict[str, Any] = {}
    for key, space in obs_space.items():
        dtype = np.dtype(space.dtype)
        size = int(np.prod(space.shape, dtype=np.int64)) if space.shape else 1
        obs_bytes_per_step += size * dtype.itemsize
        obs_shapes[key] = {"shape": list(space.shape), "dtype": str(dtype)}

    flat_config = _flat_config_items(config)
    replay_capacity = int(flat_config.get("replay.size", 0))
    model_preset = {
        key: value
        for key, value in sorted(flat_config.items())
        if key.endswith((".deter", ".hidden", ".classes", ".units", ".depth"))
    }
    symlog_keys = {
        key: value for key, value in sorted(flat_config.items()) if "symlog" in key
    }

    srb_root = Path(__file__).resolve().parents[3]
    upstream_root = Path(dreamer_agent.__file__).resolve().parent.parent

    report: dict[str, Any] = {
        "env_id": env_id,
        "checkpoint_operation": checkpoint_operation,
        "strict_task_config": strict_task_config,
        "observation_bytes_per_step": obs_bytes_per_step,
        "observation_leaves": obs_shapes,
        "replay_capacity_steps": replay_capacity,
        "projected_replay_observation_bytes": obs_bytes_per_step * replay_capacity,
        "replay_persistence": {"save": save_replay, "save_wait": replay_save_wait},
        "model_preset": model_preset,
        "simulator_device": (
            str(wrapped_env.simulator_device)
            if getattr(wrapped_env, "simulator_device", None) is not None
            else None
        ),
        "learner_platform": str(flat_config.get("jax.platform")),
        "srb_sha": _git_head(srb_root),
        "dreamer_sha": _git_head(upstream_root),
        # The observation boundary is GPU-PhysX -> CPU/USD/NumPy -> JAX; it
        # is a real transfer, never zero-copy.
        "observation_boundary": "torch->cpu-numpy->jax (not zero-copy)",
    }

    projection = getattr(wrapped_env, "observation_projection", None)
    if projection is not None:
        contract_block: dict[str, Any] = {
            "task_id": env_id.rsplit("/", 1)[-1],
            "io_schema_fingerprint": projection.io_schema_fingerprint,
            "projection_layout_fingerprint": projection.layout_fingerprint,
            "normalization_policy": (
                "metric observation units; upstream Dreamer symlog transform"
            ),
            "symlog_config": symlog_keys,
        }
        metadata_fn = getattr(
            getattr(wrapped_env, "unwrapped", wrapped_env),
            "landscaping_contract_metadata",
            None,
        )
        if callable(metadata_fn):
            task_meta = metadata_fn()
            contract_block["target_manifest_sha256"] = task_meta.get(
                "target_manifest_sha256"
            )
            contract_block["mapper_profile_sha256"] = task_meta.get(
                "mapper_profile_sha256"
            )
            contract_block["mapper_profile"] = task_meta.get("mapper_profile")
            for field in (
                "target_identity_kind",
                "target_generator_version",
                "target_profile_sha256",
                "target_reset_layout_sha256",
            ):
                if field in task_meta:
                    contract_block[field] = task_meta[field]
        report["landscaping_contract"] = contract_block

    if isinstance(run_metadata, Mapping):
        seed = run_metadata.get("run", {})
        if isinstance(seed, Mapping):
            report["seed"] = seed.get("seed")

    print("=" * 72)
    print("Dreamer pre-allocation report")
    for key in (
        "env_id",
        "checkpoint_operation",
        "observation_bytes_per_step",
        "replay_capacity_steps",
        "projected_replay_observation_bytes",
        "simulator_device",
        "learner_platform",
        "srb_sha",
        "dreamer_sha",
        "observation_boundary",
    ):
        print(f"  {key}: {report[key]}")
    if "landscaping_contract" in report:
        for key, value in report["landscaping_contract"].items():
            if key not in ("mapper_profile", "symlog_config"):
                print(f"  landscaping_contract.{key}: {value}")
    print(f"  model_preset: {model_preset}")
    print("=" * 72)

    import json

    report_path = Path(str(logdir)) / "preallocation.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
    logging.info(f"Pre-allocation report saved to {report_path}")


def make_replay(
    config,
    folder: str | Path | None,
    mode: str = "train",
    *,
    save_wait: bool = False,
):
    _load_learner_modules()
    batlen = config.batch_length if mode == "train" else config.report_length
    consec = config.consec_train if mode == "train" else config.consec_report
    capacity = config.replay.size if mode == "train" else config.replay.size / 10
    length = consec * batlen + config.replay_context
    if config.batch_size * length > capacity:
        from srb.utils.failures import InvalidInputError

        raise InvalidInputError(
            f"Dreamer replay capacity {capacity:g} is smaller than one batch "
            f"(batch_size {config.batch_size} x length {length}); raise "
            "replay.size or lower batch_size/batch_length/consec_train"
        )

    directory = elements.Path(config.logdir) / folder if folder else None
    if directory and config.replicas > 1:
        directory /= f"{config.replica:05}"

    replay_kwargs = {
        "length": length,
        "capacity": int(capacity),
        "online": config.replay.online,
        "chunksize": config.replay.chunksize,
        "directory": directory,
        # Synchronous chunk writes: Replay.save() must not return (and the
        # checkpoint 'done' marker must not be written) while chunk writes are
        # still in flight, or a "successful" checkpoint can silently reload an
        # incomplete replay.
        "save_wait": bool(save_wait),
    }

    if mode == "train":
        fractions = {
            "uniform": config.replay.fracs.uniform,
            "priority": config.replay.fracs.priority,
            "recency": config.replay.fracs.recency,
        }
        if any(
            not math.isfinite(fraction) or fraction < 0.0
            for fraction in fractions.values()
        ):
            from srb.utils.failures import InvalidInputError

            raise InvalidInputError(
                "Dreamer replay.fracs values must be finite and nonnegative"
            )
        fracs_total = sum(fractions.values())
        if not math.isclose(fracs_total, 1.0, rel_tol=0.0, abs_tol=1e-9):
            from srb.utils.failures import InvalidInputError

            raise InvalidInputError(
                "Dreamer replay.fracs (uniform + priority + recency) must sum to "
                f"1.0, got {fracs_total!r}"
            )
        if config.replay.fracs.recency > 0.0:
            recency = 1.0 / np.arange(1, capacity + 1) ** config.replay.recexp

        selectors = {
            "uniform": embodied.replay.selectors.Uniform(seed=config.seed),
            "priority": embodied.replay.selectors.Prioritized(
                seed=config.seed, **config.replay.prio
            ),
            "recency": embodied.replay.selectors.Recency(recency, seed=config.seed)
            if config.replay.fracs.recency > 0.0
            else None,
        }

        # Filter out disabled selectors
        enabled_selectors = {
            k: v
            for k, v in selectors.items()
            if config.replay.fracs[k] > 0.0 and v is not None
        }
        enabled_fractions = {
            k: v for k, v in config.replay.fracs.items() if k in enabled_selectors
        }
        enabled_total = sum(enabled_fractions.values())
        if (
            not enabled_selectors
            or not math.isfinite(enabled_total)
            or enabled_total <= 0.0
        ):
            from srb.utils.failures import InvalidInputError

            raise InvalidInputError(
                "Dreamer replay.fracs must assign positive weight to an enabled selector"
            )
        enabled_fractions = {
            key: value / enabled_total for key, value in enabled_fractions.items()
        }
        # Upstream Mixture asserts the fractions sum to exactly 1.
        *head, last = enabled_fractions
        enabled_fractions[last] = 1.0 - sum(enabled_fractions[key] for key in head)

        if len(enabled_selectors) == 1:
            replay_kwargs["selector"] = next(iter(enabled_selectors.values()))
        else:
            from srb.integrations.dreamer.selector import Mixture

            replay_kwargs["selector"] = Mixture(
                selectors=enabled_selectors,
                fractions=enabled_fractions,
                seed=config.seed,
            )

    return embodied.replay.Replay(**replay_kwargs)
