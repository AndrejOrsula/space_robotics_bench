"""Isaac-free capability and configuration gates for the public CLI."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, MutableMapping, Sequence
from pathlib import Path
from typing import Any, NoReturn, cast

from srb.utils.failures import (
    InvalidInputError,
    OptionalDependencyUnavailableError,
    UnsupportedCapabilityError,
)


class CliPreflightError(InvalidInputError):
    """A typed invalid CLI request that must not reach a runtime launch."""


class _CliUnsupportedCapabilityError(UnsupportedCapabilityError, SystemExit):
    """Typed capability failure with legacy direct-command exit semantics."""


def _reject(message: str) -> NoReturn:
    raise CliPreflightError(
        code="cli.preflight.invalid_input",
        message=message,
        component="srb.cli.preflight",
    )


def _unsupported(message: str, *, code: str) -> NoReturn:
    raise _CliUnsupportedCapabilityError(
        code=code,
        message=message,
        component="srb.cli.preflight",
    )


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - dependency is a project extra
        raise OptionalDependencyUnavailableError(
            code="cli.preflight.yaml_dependency_missing",
            message=f"Cannot validate YAML config {path}: PyYAML is unavailable",
            component="srb.cli.preflight.yaml",
            details={"path": str(path), "install_hint": "pip install PyYAML"},
        ) from exc

    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001 - convert parser details to CLI error
        _reject(f"Invalid YAML config path {path}: {exc}")
    if not isinstance(raw, dict) or not raw:
        _reject(f"Config path {path} must contain a non-empty YAML mapping")
    return raw


def validate_hydra_config_path(config: str | Path | None) -> Path | None:
    """Validate a user-supplied Hydra config before any runtime imports."""
    if config is None:
        return None
    value = str(config)
    if value.upper() in {"DEFAULT", "IGNORE", "NONE", "NULL"}:
        return None

    path = Path(value).expanduser().resolve()
    if not path.is_file():
        _reject(f"Hydra config path does not exist or is not a file: {path}")
    if path.suffix.lower() not in {".yaml", ".yml"}:
        _reject(f"Hydra config path must be YAML (.yaml/.yml): {path}")
    _load_yaml_mapping(path)
    return path


def validate_model_path(
    model: str | Path | None, *, algo: str | None = None
) -> Path | None:
    """Require a supplied checkpoint or model-artifact path before launch."""
    if model is None or not str(model):
        return None
    path = Path(model).expanduser().resolve()
    if not path.exists():
        # SB3 appends ``.zip`` to missing paths. Preserve that behavior here;
        # other integrations require an existing artifact.
        zipped = path.with_name(f"{path.name}.zip")
        # ``None`` retains the direct-helper default; CLI dispatch passes an
        # empty string when algorithm resolution fails.
        sb3_or_sbx = algo is None or algo.startswith(("sb3_", "sbx_"))
        if sb3_or_sbx and path.suffix != ".zip" and zipped.is_file():
            path = zipped
        else:
            _reject(f"Policy/model path does not exist: {path}")
    return path


def validate_model_algorithm(
    *,
    agent_subcommand: str,
    algo: str | None,
    model: str | Path | None,
) -> str | None:
    """Validate model-only algorithm inference before runtime dispatch."""
    if (
        algo is not None
        or model is None
        or agent_subcommand not in {"collect", "eval", "teleop"}
    ):
        return algo

    from srb.experiments.policies import resolve_agent_workflow_algo

    try:
        return cast(
            str | None,
            resolve_agent_workflow_algo(
                agent_subcommand=agent_subcommand,
                algo=algo,
                model=model,
            ),
        )
    except ValueError as exc:
        _reject(str(exc))
    raise AssertionError  # pragma: no cover


def validate_agent_policy_config(
    algo: str,
    *,
    obs: str = "state",
    task: str | None = None,
    root: str | Path | None = None,
) -> Path:
    """Resolve and parse the exact policy config selected by the CLI."""
    from srb.utils.agent_config_catalog import (
        AgentCfgNotFoundError,
        resolve_agent_cfg_path,
    )

    try:
        path = cast(
            Path,
            resolve_agent_cfg_path(
                algo,
                obs=obs,
                task=_short_task(task) if task else None,
                root=root if root is not None else _hyperparams_root(),
            ),
        )
    except AgentCfgNotFoundError as exc:
        _reject(str(exc))
    if not path.is_file():  # pragma: no cover - resolver returns catalog files
        _reject(f"Resolved policy config path is not a file: {path}")
    _load_yaml_mapping(path)
    return path


def validate_visual_observation(*, root: str | Path | None = None) -> Path:
    """Require a non-empty, parseable visual observation profile catalog."""
    profiles = (
        _hyperparams_root() if root is None else Path(root)
    ) / "visual_profiles.yaml"
    if not profiles.is_file():
        _unsupported(
            "--obs visual is unavailable: visual observation profiles are missing "
            f"({profiles}). Use --obs state or provide validated profiles.",
            code="cli.preflight.visual_unsupported",
        )
    _load_yaml_mapping(profiles)
    return profiles


def require_skrl_algorithm_capability(algo: str) -> None:
    """Reject unknown algorithm labels before runtime launch."""
    from srb.core.algorithms import canonical_algorithm_label

    try:
        canonical_algorithm_label(algo)
    except ValueError as exc:
        _reject(str(exc))


def require_runtime_task_sidecar(
    runtime_task_ids: Iterable[str],
    *,
    sidecar_path: str | Path | None = None,
) -> None:
    """Require exact agreement between runtime task IDs and the task sidecar."""
    runtime = {_short_task(task) for task in runtime_task_ids}
    if not runtime:
        _reject("Runtime task registry is empty; run `srb cache update` first")

    from srb.utils.path import SRB_TASKS_SIDECAR

    path = Path(sidecar_path) if sidecar_path is not None else SRB_TASKS_SIDECAR
    if not path.is_file():
        _reject(f"Task sidecar is missing: {path}")
    import tomllib

    try:
        raw = tomllib.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        _reject(f"Invalid task sidecar {path}: {exc}")
    documented = {key for key, value in raw.items() if isinstance(value, dict)}
    missing = sorted(runtime - documented)
    stale = sorted(documented - runtime)
    if missing or stale:
        details: list[str] = []
        if missing:
            details.append("missing from docs/data/tasks.toml: " + ", ".join(missing))
        if stale:
            details.append("not present in runtime registry: " + ", ".join(stale))
        _reject("Runtime task/catalog mismatch; " + "; ".join(details))


def offline_cache_ready() -> bool:
    """Return whether the generated cache files share one complete snapshot."""
    from srb.utils.cache import cache_generation_complete

    return bool(cache_generation_complete())


def require_offline_cache() -> Sequence[str]:
    """Require a complete cache and return its runtime task IDs."""
    from srb.utils.cache import (
        cache_generation_complete,
        open_offline_srb_cache_snapshot,
        read_offline_srb_env_cache,
    )

    try:
        snapshot = open_offline_srb_cache_snapshot()
    except Exception as exc:  # noqa: BLE001 - convert identity failures to CLI error
        _reject(
            "SRB offline cache generation is incomplete or corrupt; run "
            f"`srb cache update` ({exc})"
        )
    if not cache_generation_complete(snapshot=snapshot):
        _reject(
            "SRB offline cache generation is incomplete or stale; run "
            "`srb cache update` before launching a runtime task"
        )

    runtime = cast(Sequence[str], read_offline_srb_env_cache(snapshot=snapshot))
    if not runtime:
        _reject("SRB environment cache is empty; run `srb cache update`")
    require_runtime_task_sidecar(runtime)
    return runtime


def require_rdb_storage(storage: str | None) -> None:
    if not storage:
        return
    try:
        from srb.tuning.storage import StudyStorageError, validate_rdb_dsn

        validate_rdb_dsn(storage)
    except StudyStorageError as exc:
        _reject(f"Invalid --storage DSN: {exc}")


def validate_kit_path(
    *,
    headless: bool,
    enable_cameras: bool,
    xr: bool,
    rendering_mode: str | None = None,
    experience: str | Path | None = None,
) -> Path:
    """Validate the exact Kit experience and reject impossible XR requests."""
    if rendering_mode == "xr" and not xr:
        _reject("XR rendering mode requires --xr")
    if xr and not enable_cameras:
        _reject("XR requires camera rendering; combine --xr with --enable_cameras")
    from srb.utils.path import SRB_APPS_DIR, SRB_ASSETS_DIR_SRB

    missing_resources = [
        label
        for label, path in (
            ("Kit experiences", SRB_APPS_DIR),
            ("simulator assets", SRB_ASSETS_DIR_SRB),
        )
        if not path.is_dir()
    ]
    if missing_resources:
        _reject(
            "Simulator resources are unavailable ("
            + ", ".join(missing_resources)
            + "); install SRB from a source checkout or SRB container with "
            "the assets checkout. The installed wheel supports data/CLI "
            "commands but does not bundle simulator assets."
        )
    if experience is None:
        apps = SRB_APPS_DIR
        filename = (
            "srb."
            + ("headless." if headless else "")
            + ("rendering." if enable_cameras else "")
            + ("xr." if xr else "")
            + "kit"
        )
        path = apps / filename
    else:
        path = Path(experience).expanduser().resolve()
    if not path.is_file():
        _reject(f"Kit experience path is unavailable or not a file: {path}")
    return path


def preflight_cli_namespace(namespace: Mapping[str, object]) -> None:
    """Run light, command-specific gates after parsing and before dispatch."""
    subcommand = str(namespace.get("subcommand") or "")
    if subcommand == "gui":
        forwarded_args = namespace.get("forwarded_args")
        if not _contains_help_argument(forwarded_args):
            require_offline_cache()
        return

    if subcommand in {"doc", "docs"}:
        action = str(namespace.get("doc_action") or "serve")
        if action in {"build", "serve"}:
            import shutil

            if not shutil.which("mdbook"):
                _unsupported(
                    f"srb doc {action} requires mdbook; install it with "
                    "`cargo install mdbook`",
                    code="cli.doc.mdbook_unavailable",
                )
        if action == "thumbnail" or (
            action == "docgen" and not namespace.get("static_only")
        ):
            require_offline_cache()
        return

    if subcommand == "real_agent":
        validate_hydra_config_path(_optional_str(namespace.get("config")))
        model = _optional_str(namespace.get("model"))
        if model and not Path(model).expanduser().exists():
            _reject(f"Policy/model path does not exist: {model}")
        return

    if subcommand != "agent":
        return

    agent_subcommand = str(namespace.get("agent_subcommand") or "")
    if agent_subcommand == "tune":
        if namespace.get("tune_action") == "run":
            require_rdb_storage(_optional_str(namespace.get("storage")))
            runtime = require_offline_cache()
            _require_selected_tasks(runtime, namespace)
            _validate_algo_config(namespace)
        if namespace.get("tune_action") in {"run", "baseline"}:
            _reject_unsupported_tdmpc2_tasks(
                _optional_str(namespace.get("algo")), namespace
            )
        return

    if agent_subcommand == "learn":
        runtime = require_offline_cache()
        _require_selected_tasks(runtime, namespace)
        _validate_algo_config(namespace)
        config_path = validate_hydra_config_path(_optional_str(namespace.get("config")))
        if config_path is not None and "agent" not in _load_yaml_mapping(config_path):
            _reject(f"Config path {config_path} has no 'agent' section")
        dataset = _optional_str(namespace.get("dataset"))
        if dataset is not None and not Path(dataset).expanduser().is_file():
            _reject(f"Dataset path does not exist or is not a file: {dataset}")
        return

    if agent_subcommand not in {
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
    }:
        return

    if agent_subcommand == "ros":
        from srb.cli.capabilities import require_ros

        require_ros()

    runtime = require_offline_cache()
    _require_selected_tasks(runtime, namespace)
    obs = str(namespace.get("obs") or "state")
    if obs == "visual":
        validate_visual_observation()
    validate_hydra_config_path(_optional_str(namespace.get("config")))
    model = _optional_str(namespace.get("model"))
    # Report a missing path before inference, then limit ``.zip`` fallback to
    # SB3/SBX once the algorithm is known.
    validate_model_path(model)
    resolved_algo = validate_model_algorithm(
        agent_subcommand=agent_subcommand,
        algo=_optional_str(namespace.get("algo")),
        model=model,
    )
    validate_model_path(model, algo=resolved_algo or "")
    if isinstance(namespace, MutableMapping) and resolved_algo is not None:
        namespace["algo"] = resolved_algo
    _validate_algo_config(namespace, obs=obs)
    _reject_unsupported_tdmpc2_tasks(resolved_algo, namespace)

    from srb.cli.commands.agent_launch import plan_agent_app_launch

    hydra_args = _namespace_hydra_args(namespace)
    plan_agent_app_launch(
        env_id=str(namespace.get("env_id") or ""),
        headless=bool(namespace.get("headless")),
        video_enable=bool(namespace.get("video_enable")),
        hydra_args=hydra_args,
        kwargs={
            "enable_cameras": bool(namespace.get("enable_cameras")),
            "rendering_mode": namespace.get("rendering_mode"),
            "xr": bool(namespace.get("xr")),
        },
    )


def _validate_algo_config(
    namespace: Mapping[str, object],
    *,
    obs: str | None = None,
) -> None:
    algo = _optional_str(namespace.get("algo"))
    if not algo:
        return
    require_skrl_algorithm_capability(algo)
    task = _optional_str(namespace.get("env_id"))
    if task is None:
        tasks = namespace.get("tasks")
        if (
            isinstance(tasks, Sequence)
            and not isinstance(tasks, (str, bytes))
            and tasks
        ):
            task = str(tasks[0])
    selected_obs = obs or _optional_str(namespace.get("obs")) or "state"
    validate_agent_policy_config(algo, obs=selected_obs, task=task)
    from srb.cli.capabilities import require_algorithm

    # Before Isaac Sim boots, check provider presence only. Importing it loads
    # TensorFlow and Kit and can crash startup.
    require_algorithm(algo, isolate=True)


def _reject_unsupported_tdmpc2_tasks(
    algo: str | None, namespace: Mapping[str, object]
) -> None:
    """Reject TD-MPC2 tasks it cannot train before Isaac Sim launches."""
    if algo != "tdmpc2":
        return
    from srb.integrations.tdmpc2.preflight import (
        Tdmpc2UnsupportedTaskError,
        assert_task_supported,
    )

    selected = [_optional_str(namespace.get("env_id"))]
    tasks = namespace.get("tasks")
    if isinstance(tasks, Sequence) and not isinstance(tasks, (str, bytes)):
        selected.extend(str(task) for task in tasks)
    try:
        for task in selected:
            assert_task_supported(env_id=task)
    except Tdmpc2UnsupportedTaskError as exc:
        _unsupported(str(exc), code="cli.preflight.tdmpc2_task_unsupported")


def _require_selected_tasks(
    runtime: Sequence[str], namespace: Mapping[str, object]
) -> None:
    selected: list[str] = []
    env_id = _optional_str(namespace.get("env_id"))
    if env_id:
        selected.append(env_id)
    tasks = namespace.get("tasks")
    if isinstance(tasks, Sequence) and not isinstance(tasks, (str, bytes)):
        selected.extend(str(task) for task in tasks)
    # Compare qualified IDs so a known short name in a wrong namespace fails.
    known = {_qualified_task(task) for task in runtime}
    unknown = sorted({_qualified_task(task) for task in selected} - known)
    if unknown:
        _reject(
            "Selected runtime task is not in the generated registry: "
            + ", ".join(unknown)
        )


def _hyperparams_root() -> Path:
    from srb.utils.path import SRB_HYPERPARAMS_DIR

    return SRB_HYPERPARAMS_DIR


def _short_task(task: object) -> str:
    value = str(task)
    return value.split("/", 1)[-1]


def _qualified_task(task: object) -> str:
    value = str(task)
    return value if "/" in value else f"srb/{value}"


def _optional_str(value: object) -> str | None:
    if value is None:
        return None
    text = str(value)
    return text if text else None


def _namespace_hydra_args(namespace: Mapping[str, object]) -> tuple[str, ...]:
    hydra_args = namespace.get("hydra_args")
    result: list[str] = []
    if isinstance(hydra_args, Sequence) and not isinstance(hydra_args, (str, bytes)):
        result.extend(str(value) for value in hydra_args)
    return tuple(result)


def _contains_help_argument(values: object) -> bool:
    return (
        isinstance(values, Sequence)
        and not isinstance(values, (str, bytes))
        and any(str(value) in {"--help", "-h"} for value in values)
    )


__all__ = [
    "CliPreflightError",
    "offline_cache_ready",
    "preflight_cli_namespace",
    "require_offline_cache",
    "require_rdb_storage",
    "require_runtime_task_sidecar",
    "require_skrl_algorithm_capability",
    "validate_agent_policy_config",
    "validate_hydra_config_path",
    "validate_kit_path",
    "validate_model_algorithm",
    "validate_model_path",
    "validate_visual_observation",
]
