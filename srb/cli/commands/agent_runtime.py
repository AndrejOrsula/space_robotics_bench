from __future__ import annotations

import datetime
import os
import shutil
import sys
import tempfile
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from srb.cli._utils import wrap_env, wrap_env_in_performance_test
from srb.cli.commands import agent_dispatch as _dispatch_cli
from srb.cli.commands import agent_launch as _launch_cli
from srb.cli.commands import agent_policy as _policy_cli
from srb.cli.commands._hydra_exit import (
    exit_nonzero_if_failed,
    invoke_hydra_entrypoint,
)
from srb.interfaces.runtime import EnvInterfaceWrapper
from srb.utils.path import DEFAULT_DATETIME_FORMAT, SRB_LOGS_DIR, new_eval_logdir
from srb.utils.redaction import redact_secret_arguments

if TYPE_CHECKING:
    from srb.core.env.runtime import CapabilityDecision


def _raise_invalid_input(message: str) -> None:
    from srb.utils.failures import InvalidInputError

    class _DirectCliInvalidInputError(InvalidInputError, SystemExit):
        pass

    raise _DirectCliInvalidInputError(
        code="cli.agent.invalid_input",
        message=message,
        component="srb.cli.agent",
    )


def workflow_name(algo: str | None, obs: str) -> str:
    """Visual runs use a separate workflow so checkpoint discovery cannot
    cross-load state and visual policies."""
    base = algo or ""
    return f"{base}-visual" if obs == "visual" else base


_MODEL_REFERENCE_SUBCOMMANDS = frozenset(("eval", "collect", "teleop"))


# Nested collect and teleop runs keep their metadata and Hydra output out of
# the reused policy run.
_NESTED_RUN_SUBCOMMANDS = frozenset(("collect", "teleop"))


def _new_nested_run_logdir(parent_logdir: Path, name: str) -> Path:
    root = Path(parent_logdir).joinpath(name)
    root.mkdir(parents=True, exist_ok=True)
    prefix = datetime.datetime.now().strftime(DEFAULT_DATETIME_FORMAT) + "-"
    return Path(tempfile.mkdtemp(prefix=prefix, dir=str(root)))


def _metadata_logdir(
    logdir: Path,
    agent_subcommand: str,
    *,
    evaluation_logdir: Path | None = None,
    reuses_run: bool = True,
) -> Path:
    if agent_subcommand == "eval":
        return evaluation_logdir or new_eval_logdir(logdir)
    if agent_subcommand in _NESTED_RUN_SUBCOMMANDS and reuses_run:
        return _new_nested_run_logdir(logdir, agent_subcommand)
    return logdir


def _resolve_repl_backend(repl: str | None, env_value: str | None) -> str | None:
    if repl:
        return repl
    value = (env_value or "").strip().lower()
    if value in ("1", "true", "ptpython"):
        return "ptpython"
    if value == "bpython":
        return "bpython"
    return None


def _resolve_model_runtime_paths(
    *,
    model_path: Path,
    env_id: str,
    workflow: str,
    logdir_root: Path,
    new_logdir: Callable[..., Path],
    preserve_model_path: bool = False,
) -> tuple[Path, Path, Path]:
    model = model_path
    if preserve_model_path:
        return (
            model,
            new_logdir(env_id=env_id, workflow=workflow, root=logdir_root),
            model.parent,
        )

    logdir = model.parent
    hydra_cfg_search_dir = logdir
    while not (
        logdir.parent.name == workflow
        and (logdir.parent.parent.name == env_id.rsplit("/", 1)[-1])
    ):
        new_parent = logdir.parent
        if logdir == new_parent:
            logdir = new_logdir(env_id=env_id, workflow=workflow, root=logdir_root)
            model_symlink_path = logdir.joinpath(model.name)
            model_symlink_path.parent.mkdir(parents=True, exist_ok=True)
            if model_symlink_path.is_symlink() or model_symlink_path.exists():
                model_symlink_path.unlink()
            os.symlink(model, model_symlink_path)
            model = model_symlink_path
            break
        logdir = new_parent
    return model, logdir, hydra_cfg_search_dir


def _is_admitted_model_artifact(*, model_path: Path, algo: str | None) -> bool:
    reference = _policy_cli.resolve_model_artifact_reference(
        algo=algo,
        model=model_path,
    )
    if reference is None:
        return False
    close = getattr(reference, "close", None)
    if callable(close):
        close()
    return True


def agent_cfg_entry_key(algo: str | None, obs: str) -> str | None:
    if not algo:
        return None
    from srb.core.algorithms import canonical_algorithm_label

    try:
        algo = canonical_algorithm_label(algo)
    except ValueError as exc:
        _raise_invalid_input(
            f"Unknown algorithm label {algo!r}: {exc}. Add its framework to "
            "SUPPORTED_FRAMEWORKS in srb/utils/agent_config_catalog.py."
        )
    from srb.utils.agent_config_catalog import entry_key_for

    key = entry_key_for(algo, obs)
    if key is None:
        _raise_invalid_input(
            f"Unknown algorithm label {algo!r}: no agent-config entry key rule "
            "matches it. Add its framework to SUPPORTED_FRAMEWORKS in "
            "srb/utils/agent_config_catalog.py."
        )
    return key


def requires_physx_gpu_dynamics(env_cfg: Any) -> bool:
    """CUDA devices and PBD particles require PhysX GPU dynamics, including
    when simulation readback stays on CPU."""
    sim = getattr(env_cfg, "sim", None)
    device = str(getattr(sim, "device", "")).lower()
    return device.startswith(("cuda", "gpu")) or bool(
        getattr(env_cfg, "particles", False)
    )


def tune_provenance_from_env() -> dict | None:
    import os

    study = os.environ.get("SRB_TUNE_STUDY")
    trial = os.environ.get("SRB_TUNE_TRIAL")
    if not study or trial is None:
        return None
    try:
        return {"study": study, "trial": int(trial)}
    except ValueError:
        return {"study": study, "trial": trial}


def ensure_obs_available(obs: str, *, hyperparams_root=None) -> None:
    if obs != "visual":
        return
    from srb.cli.preflight import validate_visual_observation

    validate_visual_observation(root=hyperparams_root)


def _configure_dreamer_launcher_env(algo: object) -> None:
    if not isinstance(algo, str) or "dreamer" not in algo:
        return
    from srb.integrations.dreamer import configure_xla_flags

    configure_xla_flags()


def _disable_xla_gpu_preallocation() -> None:
    """Observed on Isaac Sim 6: PhysX particle kernels require this; mechanism
    unknown. `setdefault` preserves explicit user values."""
    from os import environ

    environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")


_FALLBACK_VIDEO_LENGTH = 1000


def _episode_video_length(env_cfg: Any) -> int:
    episode_length_s = getattr(env_cfg, "episode_length_s", None)
    decimation = getattr(env_cfg, "decimation", None)
    physics_dt = getattr(getattr(env_cfg, "sim", None), "dt", None)
    try:
        step_dt = float(physics_dt) * int(decimation)  # type: ignore[arg-type]
        steps = round(float(episode_length_s) / step_dt)  # type: ignore[arg-type]
    except (TypeError, ValueError, ZeroDivisionError):
        return _FALLBACK_VIDEO_LENGTH
    return steps if steps > 0 else _FALLBACK_VIDEO_LENGTH


def _preload_skrl_optimizer_dependencies(algo: object) -> None:
    if not isinstance(algo, str) or not algo.startswith("skrl_"):
        return

    import importlib

    try:
        importlib.import_module("torch._dynamo")
    except Exception as exc:  # noqa: BLE001 - optional optimizer boundary
        from srb.utils.failures import OptionalDependencyUnavailableError

        raise OptionalDependencyUnavailableError(
            code="cli.algorithm.skrl_torch_dynamo_unavailable",
            message=(
                "SKRL requires a healthy torch._dynamo optimizer backend before "
                "Isaac Sim launch; install a supported PyTorch/SKRL environment."
            ),
            component="srb.cli.agent.algorithm.skrl",
            details={
                "algorithm": algo,
                "module": "torch._dynamo",
                "exception_type": type(exc).__name__,
                "exception": str(exc),
            },
        ) from exc


_AGENT_METADATA_KWARG_KEYS = frozenset(
    (
        "model",
        "continue_training",
        "algo",
        "rendering_mode",
        "enable_cameras",
        "xr",
        "kit_args",
        "dataset",
        "num_epochs",
        "normalize_actions",
        "model_artifact_output",
        "model_artifact_checkpoint",
        "model_artifact_task",
        "model_artifact_embodiment",
        "model_artifact_action_mode",
        "model_artifact_framework",
        "model_artifact_dataset",
        "model_artifact_dataset_repo_id",
        "model_artifact_dataset_revision",
        "model_artifact_repo_id",
        "demo_dataset",
        "demo_max_transitions",
        "teleop_device",
        "pos_sensitivity",
        "rot_sensitivity",
        "invert_controls",
        "collector",
        "num_episodes",
        "horizon",
        "success_only",
    )
)


def execute_runtime_command(
    *,
    agent_subcommand: Literal[
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
    ],
    **kwargs: Any,
) -> None:
    if agent_subcommand == "train" and (
        kwargs.get("demo_dataset") is not None
        or kwargs.get("demo_max_transitions") is not None
    ):
        from srb.integrations.algorithms import validate_demo_training_inputs

        validate_demo_training_inputs(
            algo=kwargs.get("algo"),
            demo_dataset=kwargs.get("demo_dataset"),
            demo_max_transitions=kwargs.get("demo_max_transitions"),
        )
    if (
        agent_subcommand == "eval"
        and not kwargs.get("algo")
        and not kwargs.get("model")
    ):
        _raise_invalid_input(
            "srb agent eval: error: at least one of --algo or --model is required"
        )
    _run_agent_with_env(agent_subcommand=agent_subcommand, **kwargs)


def _build_agent_metadata_cli_kwargs(
    *,
    config: str,
    headless: bool,
    hide_ui: bool,
    video_enable: bool,
    record_events: bool = False,
    interface: Sequence[str],
    perf_enable: bool,
    perf_output: str,
    perf_duration: float,
    logdir_path: str,
    kwargs: dict[str, Any],
    obs: str = "state",
) -> dict[str, Any]:
    return {
        "agent_command": "agent",
        "config": config,
        "headless": headless,
        "hide_ui": hide_ui,
        "video_enable": video_enable,
        "record_events": record_events,
        "interface": list(interface),
        "perf_enable": perf_enable,
        "perf_output": perf_output,
        "perf_duration": perf_duration,
        "logdir_path": logdir_path,
        "obs": obs,
        **{k: v for k, v in kwargs.items() if k in _AGENT_METADATA_KWARG_KEYS},
    }


def _dreamer_process_launcher_settings(
    kwargs: dict[str, Any], *, video_enable: bool
) -> dict[str, Any]:
    settings: dict[str, Any] = {
        "enable_cameras": bool(kwargs.get("enable_cameras", False)),
        "video": video_enable,
        "xr": bool(kwargs.get("xr", False)),
        "kit_args": kwargs.get("kit_args", ""),
    }
    for key in ("device", "experience", "visualizer"):
        if key in kwargs and kwargs[key] is not None:
            settings[key] = kwargs[key]
    if "rendering_mode" in kwargs and kwargs["rendering_mode"] is not None:
        settings["rendering_mode"] = kwargs["rendering_mode"]
    return settings


def _prepare_dreamer_process_run_options(kwargs: dict[str, Any]) -> dict[str, Any]:
    run_options = kwargs.pop("run_options", None)
    if run_options is None:
        options: dict[str, Any] = {}
    elif isinstance(run_options, dict):
        options = dict(run_options)
    else:
        raise TypeError("Dreamer run_options must be a mapping")

    canonical_demo_keys = ("demo_dataset", "demo_max_transitions")
    conflicts = sorted(set(options).intersection(canonical_demo_keys))
    if conflicts:
        raise ValueError(
            "run_options cannot override canonical fields: " + ", ".join(conflicts)
        )

    options.update(
        {key: kwargs[key] for key in canonical_demo_keys if kwargs.get(key) is not None}
    )
    return options


def _resolve_dreamer_process_logdir(
    *,
    env_id: str,
    obs: str,
    logdir_path: str,
    model_path: Path | None,
    continue_training: bool,
    collector: Any,
) -> tuple[Path, Path | None]:
    from srb.utils.cfg import last_logdir, new_logdir

    workflow = workflow_name("dreamer", obs)
    logdir_root = Path(logdir_path).resolve()
    if model_path is not None:
        model = model_path.resolve()
        logdir = model.parent
        while not (
            logdir.parent.name == workflow
            and logdir.parent.parent.name == env_id.rsplit("/", 1)[-1]
        ):
            new_parent = logdir.parent
            if logdir == new_parent:
                logdir = new_logdir(
                    env_id=env_id,
                    workflow=workflow,
                    root=logdir_root,
                )
                model_symlink_path = logdir.joinpath(model.name)
                model_symlink_path.parent.mkdir(parents=True, exist_ok=True)
                if model_symlink_path.is_symlink() or model_symlink_path.exists():
                    model_symlink_path.unlink()
                os.symlink(model, model_symlink_path)
                model = model_symlink_path
                break
            logdir = new_parent
        return logdir, model

    if continue_training or _policy_cli.should_use_latest_policy_logdir(
        "train", algo="dreamer", collector=collector
    ):
        return (
            last_logdir(env_id=env_id, workflow=workflow, root=logdir_root),
            None,
        )
    return new_logdir(env_id=env_id, workflow=workflow, root=logdir_root), None


def _run_agent_with_env(
    agent_subcommand: Literal[
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
    ],
    env_id: str,
    config: str,
    logdir_path: str,
    interface: Sequence[str],
    video_enable: bool,
    perf_enable: bool,
    perf_output: str | None,
    perf_duration: float | None,
    headless: bool,
    hide_ui: bool,
    record_events: bool = False,
    forwarded_args: Sequence[str] = (),
    hydra_args: Sequence[str] = (),
    **kwargs: Any,
) -> None:
    # Demo training inputs are validated once by execute_runtime_command.
    if not perf_enable and (perf_output is not None or perf_duration is not None):
        _raise_invalid_input("--perf_output and --perf_duration need --perf")
    perf_output = "STDOUT" if perf_output is None else perf_output
    perf_duration = 150.0 if perf_duration is None else perf_duration

    obs = kwargs.pop("obs", "state")
    ensure_obs_available(obs)
    runtime_hydra_args = tuple(hydra_args)

    from srb.cli.preflight import (
        require_skrl_algorithm_capability,
        validate_agent_policy_config,
        validate_hydra_config_path,
        validate_model_path,
    )

    hydra_config_path = validate_hydra_config_path(config)
    kwargs["algo"] = _policy_cli.resolve_agent_workflow_algo(
        agent_subcommand=agent_subcommand,
        algo=kwargs.get("algo"),
        model=kwargs.get("model"),
    )
    model_path = validate_model_path(kwargs.get("model"), algo=kwargs["algo"])
    dreamer_run_options: dict[str, Any] | None = None
    if agent_subcommand == "train" and kwargs.get("algo") == "dreamer":
        dreamer_run_options = _prepare_dreamer_process_run_options(kwargs)

    algo = kwargs.get("algo")
    policy_config_path: Path | None = None
    if algo:
        require_skrl_algorithm_capability(algo)
        policy_config_path = validate_agent_policy_config(
            algo,
            obs=obs,
            task=env_id,
        )

    snapshot_path = kwargs.pop("snapshot_path", None)
    snapshot_warmup = kwargs.pop("snapshot_warmup", None)
    snapshot_settle = kwargs.pop("snapshot_settle", None)
    if not snapshot_path and (
        snapshot_warmup is not None or snapshot_settle is not None
    ):
        _raise_invalid_input("--snapshot-warmup and --snapshot-settle need --snapshot")
    snapshot_warmup = 16 if snapshot_warmup is None else snapshot_warmup
    snapshot_settle = 0 if snapshot_settle is None else snapshot_settle

    if record_events:
        kwargs["enable_cameras"] = True

    if snapshot_path:
        kwargs["enable_cameras"] = True

    kwargs = _launch_cli.plan_agent_app_launch(
        env_id=env_id,
        headless=headless,
        video_enable=video_enable,
        hydra_args=runtime_hydra_args,
        kwargs=kwargs,
    ).kwargs

    if agent_subcommand == "train" and kwargs.get("algo") == "dreamer":
        # Dreamer trains in worker processes that never reach these hooks.
        if perf_enable or hide_ui or kwargs.get("repl"):
            _raise_invalid_input(
                "Dreamer training does not support --perf, --hide_ui or --repl"
            )
        if policy_config_path is None:
            raise ValueError("Dreamer training requires a resolved policy config")
        from srb.integrations.dreamer.agent_runtime import (
            run_dreamer_process_training,
        )

        process_logdir, process_model = _resolve_dreamer_process_logdir(
            env_id=env_id,
            obs=obs,
            logdir_path=logdir_path,
            model_path=model_path,
            continue_training=bool(kwargs.get("continue_training", False)),
            collector=kwargs.get("collector"),
        )
        process_custom_config = hydra_config_path
        if process_custom_config is None and config.upper() == "DEFAULT":
            maybe_config_path = process_logdir.joinpath(".hydra", "config.yaml")
            if maybe_config_path.exists():
                process_custom_config = maybe_config_path
        assert dreamer_run_options is not None

        from srb.utils.metadata import collect_run_metadata

        process_metadata = collect_run_metadata(
            env_id=env_id,
            workflow=workflow_name("dreamer", obs),
            algo="dreamer",
            logdir=process_logdir,
            agent_subcommand=agent_subcommand,
            cli_kwargs=_build_agent_metadata_cli_kwargs(
                config=config,
                headless=headless,
                hide_ui=hide_ui,
                video_enable=video_enable,
                record_events=record_events,
                interface=interface,
                perf_enable=perf_enable,
                perf_output=perf_output,
                perf_duration=perf_duration,
                logdir_path=logdir_path,
                kwargs=kwargs,
                obs=obs,
            ),
            forwarded_args=list(redact_secret_arguments(forwarded_args)),
            hydra_overrides=list(runtime_hydra_args),
        )
        tune_provenance = tune_provenance_from_env()
        if tune_provenance:
            process_metadata["tune"] = tune_provenance
        return run_dreamer_process_training(
            task_id=env_id,
            obs=obs,
            policy_config_path=policy_config_path,
            custom_config_path=process_custom_config,
            hydra_overrides=tuple(hydra_args),
            launcher_settings=_dreamer_process_launcher_settings(
                kwargs, video_enable=video_enable
            ),
            headless=headless,
            logdir=process_logdir,
            model=process_model,
            continue_training=kwargs.get("continue_training"),
            run_metadata=process_metadata,
            run_options=dreamer_run_options,
            forwarded_args=tuple(forwarded_args),
        )

    from srb.cli.capabilities import require_algorithm, require_isaac_sim, require_ros

    require_algorithm(kwargs.get("algo"), isolate=True)
    if agent_subcommand == "ros":
        require_ros()
    require_isaac_sim()
    from srb.core.app import AppLauncher

    _preload_skrl_optimizer_dependencies(kwargs.get("algo"))
    _configure_dreamer_launcher_env(kwargs.get("algo"))

    # Check an enabled REPL backend before AppLauncher starts.
    repl_backend = _resolve_repl_backend(kwargs.get("repl"), os.environ.get("SRB_REPL"))
    if repl_backend:
        from importlib.util import find_spec

        if not find_spec(repl_backend):
            raise ImportError(
                f'The "{repl_backend}" package is required for --repl {repl_backend}. '
                f"Install it with: pip install {repl_backend}"
            )

    # Set this before AppLauncher on every agent path; particle kernels depend on it.
    _disable_xla_gpu_preallocation()
    launcher = AppLauncher(headless=headless, **kwargs)

    from srb.cli.runtime_startup import run_agent_startup_checks

    run_agent_startup_checks()

    from srb.hydra.sim import hydra_task_config
    from srb.interfaces.teleop import EventOmniKeyboardTeleopInterface
    from srb.utils import logging
    from srb.utils.cfg import DEFAULT_DATETIME_FORMAT, last_logdir, new_logdir
    from srb.utils.isaacsim import hide_isaacsim_ui

    if hide_ui:
        hide_isaacsim_ui()

    workflow = workflow_name(kwargs.get("algo") or agent_subcommand, obs)
    logdir_root = Path(logdir_path).resolve()
    hydra_cfg_search_dir: Path | None = None

    allocated_logdirs: list[Path] = []

    def _allocate_logdir(**logdir_kwargs: Any) -> Path:
        allocated = new_logdir(**logdir_kwargs)
        allocated_logdirs.append(allocated)
        return allocated

    if model_path is not None:
        preserve_model_path = (
            agent_subcommand in _MODEL_REFERENCE_SUBCOMMANDS
            and _is_admitted_model_artifact(
                model_path=model_path,
                algo=kwargs.get("algo"),
            )
        )
        model, logdir, hydra_cfg_search_dir = _resolve_model_runtime_paths(
            model_path=model_path,
            env_id=env_id,
            workflow=workflow,
            logdir_root=logdir_root,
            new_logdir=_allocate_logdir,
            preserve_model_path=preserve_model_path,
        )
        kwargs["model"] = model
    elif (
        agent_subcommand == "train" and kwargs["continue_training"]
    ) or _policy_cli.should_use_latest_policy_logdir(
        agent_subcommand,
        algo=kwargs.get("algo"),
        collector=kwargs.get("collector"),
    ):
        logdir = last_logdir(env_id=env_id, workflow=workflow, root=logdir_root)
    else:
        logdir = _allocate_logdir(env_id=env_id, workflow=workflow, root=logdir_root)

    from srb.utils.metadata import collect_run_metadata, write_run_metadata

    evaluation_logdir = new_eval_logdir(logdir) if agent_subcommand == "eval" else None
    metadata_logdir = _metadata_logdir(
        logdir,
        agent_subcommand,
        evaluation_logdir=evaluation_logdir,
        reuses_run=not allocated_logdirs,
    )
    metadata_common_kwargs = {
        "env_id": env_id,
        "workflow": workflow,
        "algo": kwargs.get("algo"),
        "logdir": metadata_logdir,
        "agent_subcommand": agent_subcommand,
        "cli_kwargs": _build_agent_metadata_cli_kwargs(
            config=config,
            headless=headless,
            hide_ui=hide_ui,
            video_enable=video_enable,
            record_events=record_events,
            interface=interface,
            perf_enable=perf_enable,
            perf_output=perf_output,
            perf_duration=perf_duration,
            logdir_path=logdir_path,
            kwargs=kwargs,
            obs=obs,
        ),
        "forwarded_args": redact_secret_arguments(forwarded_args),
        "hydra_overrides": list(runtime_hydra_args),
    }
    run_metadata = collect_run_metadata(**metadata_common_kwargs)
    tune_provenance = tune_provenance_from_env()
    if tune_provenance:
        run_metadata["tune"] = tune_provenance
    write_run_metadata(run_metadata, metadata_logdir)

    config_path = None
    if config.upper() == "DEFAULT":
        if not hydra_cfg_search_dir:
            hydra_cfg_search_dir = logdir
        while hydra_cfg_search_dir != hydra_cfg_search_dir.parent:
            maybe_path = hydra_cfg_search_dir.joinpath(
                ".hydra", "config.yaml"
            ).resolve()
            if maybe_path.exists():
                config_path = maybe_path
                logging.info(f"Using default Hydra config from: {config_path}")
                break
            hydra_cfg_search_dir = hydra_cfg_search_dir.parent
    elif hydra_config_path is not None:
        config_path = hydra_config_path
        logging.info(f"Using custom Hydra config from: {config_path}")
        if config_path.suffix.lower() not in (".yaml", ".yml"):
            logging.warning(
                f"The provided Hydra config path does not appear to be a YAML file: {config_path}"
            )

    maybe_config_path = Path(logdir).joinpath(".hydra", "config.yaml").resolve()

    if not any(arg.startswith("hydra.run.dir=") for arg in sys.argv[1:]):
        if metadata_logdir != logdir:
            sys.argv.extend([f"hydra.run.dir={metadata_logdir.as_posix()}"])
            if maybe_config_path.exists():
                run_hydra_dir = metadata_logdir.joinpath(".hydra")
                os.makedirs(run_hydra_dir, exist_ok=True)
                shutil.copytree(
                    maybe_config_path.parent, run_hydra_dir, dirs_exist_ok=True
                )
        else:
            sys.argv.extend([f"hydra.run.dir={logdir.as_posix()}"])

    @hydra_task_config(
        task_name=env_id,
        agent_cfg_entry_point=agent_cfg_entry_key(kwargs.get("algo"), obs),
        config_path=config_path.as_posix() if config_path else None,
    )
    def hydra_main(
        env_cfg: dict[str, Any], agent_cfg: dict[str, Any] | None = None
    ) -> None:
        import gymnasium
        from omni.physx import get_physx_interface

        physx_gpu = requires_physx_gpu_dynamics(env_cfg)
        get_physx_interface().overwrite_gpu_setting(1 if physx_gpu else 0)
        logging.info(
            "PhysX GPU dynamics %s (env_cfg.sim.device=%s, particles=%s)",
            "enabled" if physx_gpu else "disabled",
            getattr(getattr(env_cfg, "sim", None), "device", None),
            getattr(env_cfg, "particles", False),
        )

        if snapshot_path:
            env_cfg.sim.use_fabric = False

        if record_events:
            from srb.utils.policy_provenance import build_event_provenance

            env_cfg.camera_record = True

            # This marker makes event recording fail closed when a task lacks
            # `EVENT_CAMERA`, avoiding empty-success output.
            env_cfg.event_record_requested = True
            env_cfg.event_provenance = build_event_provenance(
                env_id=env_id,
                agent_subcommand=agent_subcommand,
                collector=kwargs.get("collector"),
                algo=kwargs.get("algo"),
                model_path=kwargs.get("model"),
            )
        if getattr(env_cfg, "camera_record", False) and Path(
            env_cfg.camera_record_dir
        ) == SRB_LOGS_DIR.joinpath("camera_videos"):
            # Keep recordings with this run, under `--logs`, unless overridden.
            env_cfg.camera_record_dir = metadata_logdir.joinpath("camera_videos")

        env = gymnasium.make(
            id=env_id,
            cfg=env_cfg,
            render_mode="rgb_array" if video_enable else None,
        )
        env = wrap_env(env)
        env.reset()

        from srb.core.env.runtime import decide_agent_capability_support

        _emit_agent_capability_decision(
            decide_agent_capability_support(
                env_id=env_id,
                agent_subcommand=agent_subcommand,
                env=env,
            )
        )

        if snapshot_path:
            try:
                if snapshot_settle and int(snapshot_settle) > 0:
                    _settle_robots(env, launcher.app, steps=int(snapshot_settle))
                _capture_snapshot(
                    env,
                    launcher.app,
                    snapshot_path,
                    warmup=snapshot_warmup,
                )
            finally:
                env.close()
            return

        if video_enable:
            from datetime import datetime

            from srb.wrappers import ReplicatorRenderShim, RolloutRecordVideo

            # Keep segments finite: SRB auto-resets without Gym reset, so unlimited
            # recording buffers frames and never flushes.
            env = RolloutRecordVideo(
                ReplicatorRenderShim(env),
                video_folder=logdir.joinpath("videos")
                .joinpath(datetime.now().astimezone().strftime(DEFAULT_DATETIME_FORMAT))
                .as_posix(),
                name_prefix=env_id.rsplit("/", 1)[-1],
                disable_logger=True,
                episode_trigger=lambda _: True,
                video_length=_episode_video_length(env_cfg),
            )

        if perf_enable:
            env = wrap_env_in_performance_test(
                env=env,  # type: ignore[misc]
                sim_app=launcher.app,
                perf_output=perf_output,
                perf_duration=perf_duration,
            )

        if not headless and agent_subcommand not in ["teleop", "collect", "train"]:
            import omni.appwindow

            # Without a display, Kit has no app window and thus no keyboard to bind.
            bind_reset_key = omni.appwindow.get_default_app_window() is not None
        else:
            bind_reset_key = False
        if bind_reset_key:
            kb_reset_hooks: list = []
            env.unwrapped._kb_reset_hooks = kb_reset_hooks

            def _cb_keyboard_reset():
                for hook in kb_reset_hooks:
                    hook()
                env.reset()

            _cb_keyboard = EventOmniKeyboardTeleopInterface({"L": _cb_keyboard_reset})

        ros_node: Any | None = None
        if interface:
            from srb.interfaces import InterfaceType
            from srb.interfaces._ros_node import acquire_ros_node_runtime
            from srb.utils.ros import enable_ros2_bridge

            resolved_ifaces = []
            for _name in interface:
                _iface = InterfaceType.from_str(_name)
                if _iface is None:
                    raise ValueError(f"Unsupported interface: {_name!r}")
                resolved_ifaces.append(_iface)
            interface_factories = tuple(
                iface.implementer() for iface in dict.fromkeys(resolved_ifaces)
            )

            enable_ros2_bridge()

            ros_runtime = acquire_ros_node_runtime(
                None,
                name="sim",
                namespace="srb",
                spin=True,
            )
            ros_node = ros_runtime.node

            env = EnvInterfaceWrapper(
                env,
                interface_factories=interface_factories,
                node=ros_node,
                ros_runtime=ros_runtime,
            )

        interface_wrapper = env if isinstance(env, EnvInterfaceWrapper) else None

        def _run_agent() -> None:
            if interface_wrapper is not None:
                interface_wrapper.unwrapped.ros_node = ros_node
                interface_wrapper.unwrapped.cfg.extras = True  # type: ignore[attr-defined]

            def agent_impl(**kwargs: Any) -> None:
                phase2_metadata = collect_run_metadata(
                    **metadata_common_kwargs,
                    env_cfg=env_cfg,
                    agent_cfg=agent_cfg,
                    env=env,
                )
                write_run_metadata(phase2_metadata, metadata_logdir)
                kwargs["run_metadata"] = phase2_metadata

                kwargs.update(
                    {
                        "env_id": env_id,
                        "agent_cfg": agent_cfg,
                        "env_cfg": env_cfg,
                    }
                )
                if evaluation_logdir is not None:
                    kwargs["evaluation_logdir"] = evaluation_logdir
                elif agent_subcommand == "teleop":
                    kwargs["evaluation_logdir"] = metadata_logdir

                _dispatch_agent_workflow(
                    agent_subcommand,
                    kwargs,
                    headless=headless,
                    package_model_artifact_from_run=(
                        _policy_cli.maybe_package_model_artifact_from_run
                    ),
                )

            repl = _resolve_repl_backend(
                kwargs.pop("repl", None), os.environ.get("SRB_REPL")
            )
            if repl:
                import threading
                from importlib.util import find_spec

                from srb.cli.commands import agent as agent_cli

                repl_ns = {
                    **vars(agent_cli),
                    **globals(),
                    "env": env.unwrapped,
                    "env_wrapped": env,
                    "sim_app": launcher.app,
                }

                saved_fds: dict[str, int] = {}

                def mute() -> None:
                    if saved_fds:
                        print("[REPL] Simulator output is already muted.")
                        return
                    saved_fds["stdout"] = os.dup(1)
                    saved_fds["stderr"] = os.dup(2)
                    devnull_fd = os.open(os.devnull, os.O_WRONLY)
                    os.dup2(devnull_fd, 1)
                    os.dup2(devnull_fd, 2)
                    os.close(devnull_fd)
                    sys.stdout = open(  # noqa: SIM115  # type: ignore[assignment]
                        saved_fds["stdout"], "w", buffering=1, closefd=False
                    )
                    sys.stderr = open(  # noqa: SIM115  # type: ignore[assignment]
                        saved_fds["stderr"], "w", buffering=1, closefd=False
                    )
                    print(
                        "[REPL] Simulator stdout/stderr muted.  Run unmute() to restore."
                    )

                def unmute() -> None:
                    if not saved_fds:
                        print("[REPL] Simulator output is not muted.")
                        return
                    os.dup2(saved_fds.pop("stdout"), 1)
                    os.dup2(saved_fds.pop("stderr"), 2)
                    sys.stdout = sys.__stdout__  # type: ignore[assignment]
                    sys.stderr = sys.__stderr__  # type: ignore[assignment]
                    print("[REPL] Simulator stdout/stderr restored.")

                repl_ns["mute"] = mute
                repl_ns["unmute"] = unmute

                if repl == "bpython":
                    if not find_spec("bpython"):
                        raise ImportError(
                            'The "bpython" package is required for --repl bpython. '
                            "Install it with: pip install bpython"
                        )

                    def _run_repl() -> None:
                        import bpython

                        mute()
                        bpython.embed(locals_=repl_ns, banner="Space Robotics Bench")

                else:
                    if not find_spec("ptpython"):
                        raise ImportError(
                            'The "ptpython" package is required for --repl ptpython. '
                            "Install it with: pip install ptpython"
                        )

                    def _run_repl() -> None:
                        import ptpython.repl

                        mute()
                        ptpython.repl.embed(
                            globals=repl_ns,
                            locals={},
                            title="Space Robotics Bench",
                        )

                repl_thread = threading.Thread(target=_run_repl, daemon=True)
                repl_thread.start()

            try:
                agent_impl(env=env, sim_app=launcher.app, logdir=logdir, **kwargs)
            finally:
                # Also on errors and Ctrl-C: closing finalizes video/event recordings.
                if interface_wrapper is None:
                    env.close()

        if interface_wrapper is None:
            _run_agent()
        else:
            with interface_wrapper:
                _run_agent()

    failed = invoke_hydra_entrypoint(
        hydra_main,  # type: ignore[arg-type]
        label=f"[agent {agent_subcommand}]",
    )

    # Propagate failures before Kit close; `app.close()` exits with status 0
    # and can mask workflow errors.
    exit_nonzero_if_failed(failed)
    launcher.app.close()


def _settle_robots(env, sim_app, *, steps: int) -> None:
    from srb.utils import logging
    from srb.utils.isaacsim import update_app_without_physics

    unwrapped = env.unwrapped
    sim = getattr(unwrapped, "sim", None)
    if sim is None:
        logging.warning("No sim handle on env; skipping robot settle.")
        return

    scene = getattr(unwrapped, "scene", None)
    physics_dt = getattr(unwrapped, "physics_dt", None) or getattr(
        unwrapped, "step_dt", 0.0
    )

    logging.info(f"Settling robots over {steps} physics step(s)...")
    for _ in range(steps):
        if scene is not None:
            scene.write_data_to_sim()
        sim.step(render=False)
        if scene is not None:
            scene.update(dt=physics_dt)
    update_app_without_physics(sim_app)
    logging.info("Robots settled.")


def _capture_snapshot(env, sim_app, out_path: str, *, warmup: int = 16) -> None:
    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    from PIL import Image
    from pxr import Gf, UsdGeom

    from srb.core.scene_introspection import viewer_pose_w
    from srb.utils import logging
    from srb.utils.isaacsim import update_app_without_physics

    unwrapped = env.unwrapped
    stage = omni.usd.get_context().get_stage()

    viewer = getattr(unwrapped, "cfg", None)
    viewer = getattr(viewer, "viewer", None)
    eye, lookat = viewer_pose_w(unwrapped, viewer)
    resolution = getattr(viewer, "resolution", None) or (1280, 720)
    width, height = int(resolution[0]), int(resolution[1])

    cam = UsdGeom.Camera.Define(stage, "/World/SnapshotCamera")
    cam.CreateHorizontalApertureAttr().Set(36.0)
    cam.CreateFocalLengthAttr().Set(35.0)
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 1_000_000.0))
    xformable = UsdGeom.Xformable(cam.GetPrim())
    xformable.ClearXformOpOrder()
    xf_op = xformable.AddTransformOp()

    from srb.utils.spatial import pick_camera_up

    up = pick_camera_up(eye, lookat, preferred_up=(0.0, 0.0, 1.0))
    look_at = Gf.Matrix4d()
    look_at.SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*lookat), Gf.Vec3d(*up))
    xf_op.Set(look_at.GetInverse())

    rp = rep.create.render_product(cam.GetPath().pathString, (width, height))
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([rp])

    update_app_without_physics(sim_app, max(int(warmup), 1))
    frame = rgb_annot.get_data()

    if frame is None or getattr(frame, "size", 0) == 0:
        raise RuntimeError(
            "RGB annotator returned no data — snapshot capture needs "
            "--enable_cameras (camera rendering active)."
        )

    arr = np.asarray(frame)
    if arr.ndim == 3 and arr.shape[-1] >= 3:
        arr = arr[..., :3]

    out = Path(out_path).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(arr.astype("uint8")).save(out.as_posix())
    logging.info(f"Saved snapshot ({width}x{height}) to {out}")

    try:
        rgb_annot.detach([rp])
    except Exception as exc:  # noqa: BLE001
        logging.debug("RGB annotator detach cleanup failed: %s", exc)


_dispatch_agent_workflow = _dispatch_cli.dispatch_agent_workflow


def _emit_agent_capability_decision(
    decision: CapabilityDecision,
    *,
    warn: Callable[[str], None] | None = None,
) -> None:
    if decision.message is None:
        return
    if decision.should_fail:
        raise RuntimeError(decision.message)
    if warn is None:
        from srb.utils import logging

        warn = logging.warning
    warn(decision.message)


_with_runtime_default_kit_args = _launch_cli.with_runtime_default_kit_args
_should_auto_enable_cameras = _launch_cli.should_auto_enable_cameras
_get_hydra_bool_override = _launch_cli.get_hydra_bool_override


__all__ = ("execute_runtime_command",)
