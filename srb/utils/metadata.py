from __future__ import annotations

import configparser
import datetime
import hashlib
import json
import math
import os
import platform
import shlex
import shutil
import socket
import sys
from collections.abc import Mapping, Sequence
from dataclasses import fields, is_dataclass
from numbers import Integral
from pathlib import Path
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any
from urllib.parse import unquote, urlparse

from srb.utils import logging
from srb.utils.process_supervisor import ProcessKind, ProcessStatus, run_owned
from srb.utils.publication import ArtifactPublication, PublicationCleanupError
from srb.utils.redaction import (
    REDACTED,
    is_secret_key,
    redact_secret_arguments,
    redact_secret_text,
)
from srb.utils.resource_budget import MAX_IN_MEMORY_FILE_BYTES

if TYPE_CHECKING:
    import gymnasium

_SRB_ROOT = Path(__file__).resolve().parent.parent.parent

_ENV_VAR_ALLOWLIST = (
    "SRB_LOG_LEVEL",
    "SRB_LOGFIRE_ENABLE",
    "SRB_RICH_TRACEBACK",
    "SRB_DISPLAY",
    "ISAAC_PATH",
    "ISAACSIM_PATH",
    "ISAACSIM_PYTHON",
    "NVIDIA_VISIBLE_DEVICES",
    "NVIDIA_DRIVER_CAPABILITIES",
    "CUDA_VISIBLE_DEVICES",
    "DISPLAY",
    "ROS_DOMAIN_ID",
    "LOGFIRE_SEND_TO_LOGFIRE",
)

_DEPENDENCY_PACKAGES = (
    "torch",
    "gymnasium",
    "numpy",
    "pydantic",
    "orjson",
    "scipy",
    "pillow",
    "h5py",
    "isaacsim",
    "isaaclab",
    "oxidasim",
    "simforge",
    "omegaconf",
    "hydra-core",
    "wandb",
    "tensorboard",
    "stable-baselines3",
    "sb3-contrib",
    "rl-zoo3",
    "skrl",
    "rsl-rl",
    "tdmpc2",
    "dreamerv3",
    "robomimic",
    "psutil",
)


def collect_run_metadata(
    env_id: str,
    workflow: str,
    algo: str | None,
    logdir: Path,
    agent_subcommand: str,
    cli_kwargs: dict[str, Any] | None = None,
    forwarded_args: list[str] | None = None,
    env_cfg: Any | None = None,
    agent_cfg: Any | None = None,
    env: "gymnasium.Env | None" = None,
    hydra_overrides: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Collect run metadata, keeping Hydra overrides separate from forwarded arguments."""
    metadata = {
        "srb": _collect_srb_info(),
        "run": _collect_run_info(
            env_id=env_id,
            workflow=workflow,
            algo=algo,
            logdir=logdir,
            agent_subcommand=agent_subcommand,
            cli_kwargs=cli_kwargs,
            forwarded_args=forwarded_args,
            env_cfg=env_cfg,
            agent_cfg=agent_cfg,
            hydra_overrides=hydra_overrides,
        ),
        "environment": _collect_environment_info(env),
        "system": _collect_system_info(logdir),
        "platform": _collect_platform_info(),
        "environment_variables": _collect_environment_variables(),
        "dependencies": _collect_dependencies(),
    }
    srb_info = metadata["srb"]
    srb_sha = srb_info.get("git_commit") if isinstance(srb_info, Mapping) else None
    landscaping_contract = collect_landscaping_contract_metadata(
        env=env,
        env_id=env_id,
        algo=algo,
        srb_sha=srb_sha,
    )
    if landscaping_contract is not None:
        metadata["landscaping_contract"] = landscaping_contract
    return metadata


def collect_landscaping_contract_metadata(
    *,
    env: "gymnasium.Env | Any | None",
    env_id: str,
    algo: str | None = None,
    srb_sha: str | None = None,
) -> dict[str, Any] | None:
    """Return task-published landscaping contract metadata, if available.

    The projection-layout fingerprint also pins flat observation packing.
    """
    if env is None:
        return None
    unwrapped = getattr(env, "unwrapped", env)
    metadata_fn = getattr(unwrapped, "landscaping_contract_metadata", None)
    if not callable(metadata_fn):
        return None
    try:
        task_meta = metadata_fn()
    except Exception as exc:
        logging.warning(f"Failed to collect landscaping contract metadata: {exc}")
        return None

    block: dict[str, Any] = {
        "task_id": str(env_id).rsplit("/", 1)[-1],
        "contract_id": task_meta.get("contract_id"),
        "io_schema_fingerprint": task_meta.get("io_schema_fingerprint"),
        # Keep task-published to avoid depending on runtime and terrain layers.
        "projection_layout_fingerprint": task_meta.get("projection_layout_fingerprint"),
        "target_manifest_sha256": task_meta.get("target_manifest_sha256"),
        "mapper_profile_sha256": task_meta.get("mapper_profile_sha256"),
        # Record the learner transform and absence of a running normalizer.
        "normalization_policy": (
            "metric observation units; upstream Dreamer symlog transform; "
            "no running normalizer"
        ),
        "srb_sha": srb_sha if srb_sha is not None else _checkout_head(_SRB_ROOT),
    }
    # Checkpoint identity uses the seed-independent mission family.
    for field in (
        "target_identity_kind",
        "target_generator_version",
        "target_profile_sha256",
        "target_reset_layout_sha256",
    ):
        if field in task_meta:
            block[field] = task_meta[field]
    if algo and "dreamer" in str(algo):
        block["dreamer_sha"] = _learner_repo_sha("dreamerv3")
    return block


def _learner_repo_sha(package: str) -> str | None:
    try:
        from importlib.util import find_spec

        spec = find_spec(package)
        if spec is None or not spec.origin:
            return None
        repo_root = Path(spec.origin).resolve().parent.parent
        # Ignore an enclosing checkout when the learner is installed in a venv.
        return _checkout_head(repo_root)
    except (ImportError, AttributeError, ValueError, OSError):
        return None


def metadata_json_text(metadata: Mapping[str, Any]) -> str:
    """Render run metadata with the same normalization used for publication."""

    return json.dumps(
        _normalize_metadata_json(dict(metadata)),
        allow_nan=False,
        ensure_ascii=True,
        indent=2,
        sort_keys=True,
    )


def persist_demo_replay_metadata(
    run_metadata: Mapping[str, Any] | None,
    *,
    logdir: Path,
    payload: Any,
    max_transitions: int,
    framework: str = "SKRL",
) -> None:
    """Publish and verify the consumed demonstration source binding.

    ``payload`` exposes the four source fields used by the run metadata
    contract, without requiring a training provider's payload class.
    """

    metadata = dict(run_metadata) if isinstance(run_metadata, Mapping) else {}
    metadata["demo_replay"] = {
        "dataset": str(_demo_payload_value(payload, "dataset_path")),
        "max_transitions": int(max_transitions),
        "loaded_transitions": int(_demo_payload_value(payload, "transition_count")),
        "dataset_fingerprint": str(_demo_payload_value(payload, "dataset_fingerprint")),
        "binding": _demo_payload_value(payload, "binding"),
    }

    output_path = Path(logdir) / "metadata.json"
    expected_text = metadata_json_text(metadata) + "\n"
    try:
        Path(logdir).mkdir(parents=True, exist_ok=True)
        write_run_metadata(metadata, Path(logdir))
        recorded_payload = ArtifactPublication.file(output_path).read(
            lambda payload: payload,
            max_bytes=MAX_IN_MEMORY_FILE_BYTES,
        )
        if recorded_payload is None:
            raise ValueError("published metadata readback is missing")
        recorded_text = recorded_payload.decode("utf-8")
        if recorded_text != expected_text:
            raise ValueError("published metadata differs from staged demo binding")
        recorded = json.loads(recorded_text)
        if (
            not isinstance(recorded, Mapping)
            or recorded.get("demo_replay")
            != json.loads(metadata_json_text(metadata))["demo_replay"]
        ):
            raise ValueError("published metadata omitted the demo replay binding")
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(
            f"could not publish verified {framework} demo replay metadata: "
            f"{output_path}"
        ) from exc

    if isinstance(run_metadata, dict):
        run_metadata.clear()
        run_metadata.update(metadata)


def _demo_payload_value(payload: Any, field: str) -> Any:
    if isinstance(payload, Mapping):
        return payload[field]
    return getattr(payload, field)


#: Compact per-run record written beside ``metadata.json``.
RUN_RECORD_FILENAME = "run.json"

#: Container launcher environment variables recorded in ``run.json``.
IMAGE_ID_ENV = "SRB_IMAGE_ID"
HOST_ENV = "SRB_HOST"

_BUDGET_OVERRIDE_KEYS = (
    "agent.n_timesteps",
    "agent.total_timesteps",
    "agent.run.steps",
    "agent.trainer.timesteps",
    "agent.steps",
    "agent.max_iterations",
)

_CONFIG_BUDGET_PATHS = (
    ("n_timesteps",),
    ("total_timesteps",),
    ("steps",),
    ("run", "steps"),
)
_RSL_BUDGET_PATHS = (
    ("max_iterations",),
    ("num_steps_per_env",),
)
_SEED_PATHS = (("seed",), ("env", "seed"))
_ENV_COUNT_PATHS = (("num_envs",), ("scene", "num_envs"))
_MISSING = object()


def _read_first_line(path: Path) -> str | None:
    try:
        with path.open(encoding="utf-8") as handle:
            return handle.readline().strip() or None
    except (OSError, UnicodeError):
        return None


def _installed_version(distribution: str) -> str | None:
    from importlib.metadata import PackageNotFoundError, version

    try:
        return version(distribution)
    except PackageNotFoundError:
        return None


def _read_build_metadata(path: Path) -> dict[str, str] | None:
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        if path.is_symlink():
            return {}
        return None
    except (OSError, UnicodeError):
        return {}
    values: dict[str, str] = {}
    for line in text.splitlines():
        key, separator, value = line.partition("=")
        if not separator or not key or not value or key in values:
            return {}
        values[key] = value
    return values


def _is_full_lower_hex_commit(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 40
        and all(character in "0123456789abcdef" for character in value)
    )


def _checkout_head(candidate: Path) -> str | None:
    """Return ``HEAD`` only when ``candidate`` is the checkout root.

    Exact-root checking prevents ``git`` from reporting an enclosing checkout.
    """

    git_root_text = _git("rev-parse", "--show-toplevel", cwd=candidate)
    if git_root_text is None:
        return None
    try:
        if Path(git_root_text).resolve() != candidate.resolve():
            return None
    except (OSError, RuntimeError):
        return None
    git_head = _git("rev-parse", "HEAD", cwd=candidate)
    return git_head if _is_full_lower_hex_commit(git_head) else None


def _is_source_checkout(candidate: Path) -> bool:
    return _checkout_head(candidate) is not None


def _isaac_lab_product_version() -> str | None:
    """Read Isaac Lab's product version from its installed source checkout."""

    from importlib.metadata import PackageNotFoundError, distribution

    try:
        package = distribution("isaaclab")
        direct_url_text = package.read_text("direct_url.json")
    except (OSError, UnicodeError, PackageNotFoundError, ValueError):
        return None
    if not isinstance(direct_url_text, str):
        return None
    try:
        direct_url = json.loads(direct_url_text)
    except json.JSONDecodeError:
        return None
    if not isinstance(direct_url, Mapping):
        return None
    dir_info = direct_url.get("dir_info")
    if not isinstance(dir_info, Mapping) or dir_info.get("editable") is not True:
        return None
    url = direct_url.get("url")
    if not isinstance(url, str):
        return None
    try:
        parsed = urlparse(url)
    except ValueError:
        return None
    if parsed.scheme != "file" or parsed.netloc not in ("", "localhost"):
        return None
    try:
        source_path = Path(unquote(parsed.path)).resolve()
    except (OSError, RuntimeError):
        return None

    candidate = source_path
    for _ in range(7):
        source_version = _read_first_line(candidate / "VERSION")
        build_metadata = _read_build_metadata(candidate / ".srb-build-metadata")
        if source_version is not None:
            if build_metadata is not None:
                if build_metadata.get(
                    "source_version"
                ) == source_version and _is_full_lower_hex_commit(
                    build_metadata.get("source_commit")
                ):
                    return source_version
                return None
            if _is_source_checkout(candidate):
                return source_version
            return None
        parent = candidate.parent
        if parent == candidate:
            break
        candidate = parent
    return None


def _collect_platform_info() -> dict[str, Any]:
    """Simulator pair, image, host, GPU and driver: what a result is attributed to."""

    isaac_sim_path = Path(os.environ.get("ISAACSIM_PATH", "/root/isaac-sim"))
    isaac_sim_version = _read_first_line(isaac_sim_path / "VERSION")
    if isaac_sim_version is None:
        isaac_sim_version = _installed_version("isaacsim")

    gpu_name: str | None = None
    torch = sys.modules.get("torch")
    if torch is not None:
        try:
            if torch.cuda.is_available():
                gpu_name = str(torch.cuda.get_device_name(0))
        except (RuntimeError, AssertionError, OSError) as exc:  # pragma: no cover
            logging.debug(f"Could not resolve the CUDA device name: {exc}")

    driver_version: str | None = None
    driver_line = _read_first_line(Path("/proc/driver/nvidia/version"))
    if driver_line:
        tokens = driver_line.split()
        if "Module" in tokens:
            index = tokens.index("Module") + 1
            driver_version = tokens[index] if index < len(tokens) else None

    return {
        "isaac_sim": isaac_sim_version,
        "isaac_lab": _isaac_lab_product_version(),
        "image_id": os.environ.get(IMAGE_ID_ENV) or None,
        "host": os.environ.get(HOST_ENV) or socket.gethostname(),
        "gpu": gpu_name,
        "driver": driver_version,
        "process_title": os.environ.get("SRB_PROCESS_TITLE") or None,
    }


def _config_value(config: Any, path: tuple[str, ...]) -> Any:
    """Read one known config path without walking arbitrary runtime objects."""

    value = config
    for key in path:
        if isinstance(value, Mapping):
            if key not in value:
                return _MISSING
            value = value[key]
            continue
        try:
            value = getattr(value, key)
        except AttributeError:
            return _MISSING
    return value


def _first_config_value(
    config: Any, paths: Sequence[tuple[str, ...]]
) -> tuple[bool, Any]:
    for path in paths:
        value = _config_value(config, path)
        if value is not _MISSING and value is not None:
            return True, value
    return False, None


def _exact_integer(value: Any, *, positive: bool) -> int | None:
    """Accept integral values without coercing floats, strings or booleans."""

    if isinstance(value, bool) or not isinstance(value, Integral):
        return None
    result = int(value)
    if result < (1 if positive else 0):
        return None
    return result


def _positive_integer_text(value: str) -> int | None:
    """Parse a CLI integer override without float/``repr`` coercion."""

    if not value or value.startswith("-"):
        return None
    if value.startswith("+"):
        value = value[1:]
    if not value or value.startswith("_") or value.endswith("_"):
        return None
    if "__" in value:
        return None
    compact = value.replace("_", "")
    if not compact.isdecimal():
        return None
    try:
        parsed = int(compact, 10)
    except ValueError:
        return None
    return parsed if parsed > 0 else None


def _positive_integer(value: Any) -> int | None:
    return _exact_integer(value, positive=True)


def _budget_from_overrides(arguments: Sequence[Any]) -> int | None:
    """Read a legacy/configured budget override, or ``None`` if unknown."""

    if isinstance(arguments, str):
        arguments = (arguments,)
    for arg in reversed(arguments):
        if not isinstance(arg, str):
            continue
        key, separator, value = arg.partition("=")
        if separator and key in _BUDGET_OVERRIDE_KEYS:
            return _positive_integer_text(value)
    return None


def _seed_from_configs(env_cfg: Any, agent_cfg: Any) -> tuple[bool, int | None]:
    for config in (env_cfg, agent_cfg):
        found, value = _first_config_value(config, _SEED_PATHS)
        if found:
            return True, _exact_integer(value, positive=False)
    return False, None


def _num_envs_from_configs(
    env_cfg: Any,
    agent_cfg: Any,
    environment: Mapping[str, Any] | None = None,
) -> int | None:
    candidates: list[tuple[bool, Any]] = []
    if environment is not None:
        candidates.append(("num_envs" in environment, environment.get("num_envs")))
    for config in (env_cfg, agent_cfg):
        candidates.append(_first_config_value(config, _ENV_COUNT_PATHS))
    for found, value in candidates:
        if found:
            count = _exact_integer(value, positive=True)
            if count is not None:
                return count
    return None


def _budget_from_configs(
    algorithm: str | None,
    env_cfg: Any,
    agent_cfg: Any,
    environment: Mapping[str, Any] | None = None,
) -> tuple[bool, int | None]:
    """Return ``(configured, budget)`` from effective structured configs."""

    implementation = _implementation_for(algorithm)
    if implementation == "rsl_rl":
        found_iteration, iterations = _first_config_value(
            agent_cfg, (_RSL_BUDGET_PATHS[0],)
        )
        found_steps, steps = _first_config_value(agent_cfg, (_RSL_BUDGET_PATHS[1],))
        if found_iteration or found_steps:
            iteration_count = _positive_integer(iterations)
            step_count = _positive_integer(steps)
            env_count = _num_envs_from_configs(
                env_cfg, agent_cfg, environment=environment
            )
            if iteration_count is None or step_count is None or env_count is None:
                return True, None
            return True, iteration_count * step_count * env_count

    if implementation == "skrl":
        found_timesteps, timesteps = _first_config_value(
            agent_cfg, (("trainer", "timesteps"),)
        )
        if found_timesteps:
            timestep_count = _positive_integer(timesteps)
            env_count = _num_envs_from_configs(
                env_cfg, agent_cfg, environment=environment
            )
            if timestep_count is None or env_count is None:
                return True, None
            return True, timestep_count * env_count
        return False, None

    for path in _CONFIG_BUDGET_PATHS:
        found, value = _first_config_value(agent_cfg, (path,))
        if found:
            return True, _positive_integer(value)
    return False, None


def _budget_for_run(
    algorithm: str | None,
    env_cfg: Any,
    agent_cfg: Any,
    environment: Mapping[str, Any] | None,
    forwarded_args: Sequence[Any],
    hydra_overrides: Sequence[Any] | None,
) -> int | None:
    configured, budget = _budget_from_configs(
        algorithm,
        env_cfg,
        agent_cfg,
        environment,
    )
    if configured:
        return budget
    if _implementation_for(algorithm) in {"rsl_rl", "skrl"}:
        # Iteration/vector-step counts are not env-step budgets without the
        # provider's conversion dimensions and actual parallel-env count.
        return None
    # Post-``--`` framework arguments are retained for compatibility with
    # direct callers. Explicit Hydra provenance is the authoritative source
    # for new callers and must not be mixed with framework arguments.
    arguments = forwarded_args if hydra_overrides is None else hydra_overrides
    return _budget_from_overrides(arguments)


#: Snapshot containers through this depth so nested secrets remain redacted.
_SNAPSHOT_MAX_DEPTH = 64


def _snapshot_config(
    value: Any,
    *,
    depth: int = 0,
    seen: frozenset[int] = frozenset(),
) -> Any:
    """Snapshot config values while redacting secret keys and strings."""

    if isinstance(value, str):
        return redact_secret_text(value)
    if value is None or isinstance(value, (bool, int, Path)):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, float):
        return value if math.isfinite(value) else repr(value)
    is_container = (
        isinstance(value, (Mapping, SimpleNamespace, list, tuple))
        or is_dataclass(value)
        and not isinstance(value, type)
    )
    if is_container and id(value) in seen:
        # Preserve key redaction for nested containers at the depth limit.
        return f"<recursive {type(value).__name__}>"
    if is_container and depth >= _SNAPSHOT_MAX_DEPTH:
        return f"<truncated {type(value).__name__}>"
    child_seen = seen | {id(value)}

    def _member(key: Any, item: Any) -> Any:
        if (
            isinstance(key, str)
            and is_secret_key(key)
            and item is not None
            and not isinstance(item, bool)
        ):
            return REDACTED
        return _snapshot_config(item, depth=depth + 1, seen=child_seen)

    if isinstance(value, Mapping):
        return {key: _member(key, item) for key, item in value.items()}
    if isinstance(value, SimpleNamespace):
        return {key: _member(key, item) for key, item in vars(value).items()}
    if is_dataclass(value) and not isinstance(value, type):
        return {
            item.name: _member(item.name, getattr(value, item.name))
            for item in fields(value)
        }
    if isinstance(value, (list, tuple)):
        return [
            _snapshot_config(item, depth=depth + 1, seen=child_seen) for item in value
        ]
    return value


def _redacted_arguments(arguments: Sequence[str] | None) -> list[str]:
    if arguments is None:
        return []
    if isinstance(arguments, str):
        arguments = (arguments,)
    return [str(item) for item in redact_secret_arguments(list(arguments))]


#: Prefixes mirror ``srb.integrations.algorithms``; longest match wins.
IMPLEMENTATION_PREFIXES: tuple[str, ...] = (
    "rsl_rl",
    "robomimic",
    "dreamer",
    "tdmpc2",
    "skrl",
    "sb3",
    "sbx",
)


def _implementation_for(algo: str | None) -> str | None:
    if not algo:
        return None
    name = str(algo).strip().lower()
    for prefix in sorted(IMPLEMENTATION_PREFIXES, key=len, reverse=True):
        if name == prefix or name.startswith(prefix + "_"):
            return prefix
    return None


def run_record_view(metadata: Mapping[str, Any]) -> dict[str, Any]:
    """Project metadata onto ``run.json`` with a stable field set.

    Unknown values remain ``null`` rather than becoming omitted fields.
    """
    srb = dict(metadata.get("srb") or {})
    run = dict(metadata.get("run") or {})
    platform_info = dict(metadata.get("platform") or {})
    system = dict(metadata.get("system") or {})
    env_cfg = run.get("env_cfg")
    agent_cfg = run.get("agent_cfg")
    hyperparameters_digest: str | None = None
    agent_snapshot = _snapshot_config(agent_cfg)
    if isinstance(agent_snapshot, Mapping) and agent_snapshot:
        hyperparameters_digest = hashlib.sha256(
            metadata_json_text(agent_snapshot).encode("utf-8")
        ).hexdigest()
    forwarded = _redacted_arguments(run.get("forwarded_args") or [])
    hydra_values = run.get("hydra_overrides", _MISSING)
    overrides = (
        forwarded
        if hydra_values is _MISSING or hydra_values is None
        else _redacted_arguments(hydra_values)
    )
    algorithm = run.get("algorithm")
    environment = metadata.get("environment")
    environment_info = environment if isinstance(environment, Mapping) else None
    seed_configured, config_seed = _seed_from_configs(env_cfg, agent_cfg)
    seed = (
        config_seed
        if seed_configured
        else _exact_integer(run.get("seed"), positive=False)
    )
    submodules = srb.get("git_submodules") or {}
    assets = (
        submodules.get("assets/srb_assets") if isinstance(submodules, Mapping) else None
    )
    return {
        "schema_version": 1,
        "timestamp": run.get("timestamp"),
        "srb_commit": srb.get("git_commit"),
        "srb_dirty": srb.get("git_dirty"),
        "srb_assets_commit": assets.get("commit")
        if isinstance(assets, Mapping)
        else None,
        "simulator": {
            "isaac_sim": platform_info.get("isaac_sim"),
            "isaac_lab": platform_info.get("isaac_lab"),
        },
        "image_id": platform_info.get("image_id"),
        "host": platform_info.get("host") or system.get("hostname"),
        "arch": system.get("arch"),
        "gpu": platform_info.get("gpu"),
        "driver": platform_info.get("driver"),
        "seed": seed,
        "algorithm": algorithm,
        "implementation": _implementation_for(algorithm),
        "task": run.get("env_id"),
        "workflow": run.get("workflow"),
        "budget": _budget_for_run(
            algorithm,
            env_cfg,
            agent_cfg,
            environment_info,
            forwarded,
            hydra_values if hydra_values is not _MISSING else None,
        ),
        "hyperparameters_digest": hyperparameters_digest,
        "overrides": overrides,
        "logdir": run.get("logdir"),
    }


def write_run_metadata(metadata: dict[str, Any], logdir: Path) -> None:
    """Publish deterministic, finite ``metadata.json`` and ``run.json``."""
    try:
        output_path = logdir / "metadata.json"
        payload = (metadata_json_text(metadata) + "\n").encode("utf-8")
        lock_path = logdir.parent / f".{logdir.name}.metadata.publication.lock"
        ArtifactPublication.file(output_path, lock_path=lock_path).publish_bytes(
            payload
        )
        logging.info(f"Run metadata saved to {output_path}")
        record_path = logdir / RUN_RECORD_FILENAME
        record_payload = (metadata_json_text(run_record_view(metadata)) + "\n").encode(
            "utf-8"
        )
        record_lock = logdir.parent / f".{logdir.name}.run.publication.lock"
        ArtifactPublication.file(record_path, lock_path=record_lock).publish_bytes(
            record_payload
        )
        logging.info(f"Run record saved to {record_path}")
    except PublicationCleanupError:
        # Preserve the publication owner so callers can retry cleanup.
        raise
    except Exception as exc:
        logging.warning(f"Failed to write run metadata: {exc}")


def _normalize_metadata_json(value: Any, *, path: str = "metadata") -> Any:
    """Convert metadata to finite JSON values before publication."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{path} must contain only finite numbers")
        return value
    if isinstance(value, Path):
        return value.as_posix()
    if isinstance(value, Mapping):
        normalized: dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError(f"{path} keys must be strings")
            normalized[key] = _normalize_metadata_json(item, path=f"{path}.{key}")
        return normalized
    if isinstance(value, (list, tuple)):
        return [
            _normalize_metadata_json(item, path=f"{path}[{index}]")
            for index, item in enumerate(value)
        ]
    # Keep arbitrary diagnostics finite and deterministic.
    return str(value)


def _git(*args: str, cwd: Path | None = None) -> str | None:
    """Run a git command and return stripped stdout, or None on failure."""
    try:
        result = run_owned(
            ("git", *args),
            kind=ProcessKind.FINITE,
            cwd=cwd or _SRB_ROOT,
            capture_output=True,
            timeout=10,
        )
        if (
            result.status is not ProcessStatus.EXITED
            or result.returncode != 0
            or not result.cleanup_complete
        ):
            return None
        return (result.stdout or "").strip()
    except Exception:
        return None


def _collect_srb_info() -> dict[str, Any]:
    # The distribution is named ``srb`` (pyproject ``[project].name``).
    version = _installed_version("srb")

    # Only an exact-root source checkout contributes git metadata.
    git_commit = _checkout_head(_SRB_ROOT)
    git_commit_short: str | None = None
    git_branch: str | None = None
    git_dirty: bool | None = None
    git_diff_stat: str | None = None
    if git_commit is not None:
        git_commit_short = _git("rev-parse", "--short", "HEAD")
        git_branch = _git("rev-parse", "--abbrev-ref", "HEAD")

        porcelain = _git("status", "--porcelain")
        if porcelain is not None:
            git_dirty = len(porcelain) > 0

        diff_output = _git("diff", "--stat", "HEAD")
        if diff_output is not None:
            lines = diff_output.strip().splitlines()
            git_diff_stat = lines[-1] if lines else ""

    # Record the checked-out submodule commit and pinned gitlink separately.
    git_submodules: dict[str, dict[str, Any]] | None = None
    gitmodules_path = _SRB_ROOT / ".gitmodules"
    if git_commit is not None and gitmodules_path.exists():
        try:
            cfg = configparser.ConfigParser()
            cfg.read(str(gitmodules_path))
            git_submodules = {}
            for section in cfg.sections():
                path = cfg.get(section, "path", fallback=None)
                if path is None:
                    continue
                url = cfg.get(section, "url", fallback="")
                branch = cfg.get(section, "branch", fallback="")
                gitlink = _git("rev-parse", f"HEAD:{path}")
                sub_root = _SRB_ROOT / path
                sub_commit = _checkout_head(sub_root)
                sub_dirty: bool | None = None
                if sub_commit is not None:
                    sub_status = _git(
                        "status",
                        "--porcelain",
                        "--untracked-files=all",
                        cwd=sub_root,
                    )
                    if sub_status is not None:
                        sub_dirty = len(sub_status) > 0
                git_submodules[path] = {
                    "url": url,
                    "branch": branch,
                    "commit": sub_commit,
                    "gitlink": gitlink,
                    "dirty": sub_dirty,
                }
        except Exception:
            git_submodules = None

    return {
        "version": version,
        "git_commit": git_commit,
        "git_commit_short": git_commit_short,
        "git_branch": git_branch,
        "git_dirty": git_dirty,
        "git_diff_stat": git_diff_stat,
        "git_submodules": git_submodules,
    }


def _collect_system_info(logdir: Path) -> dict[str, Any]:
    info: dict[str, Any] = {
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "os": platform.system(),
        "os_version": platform.release(),
        "arch": platform.machine(),
        "python_version": platform.python_version(),
        "python_executable": sys.executable,
        "cpu_count": os.cpu_count(),
    }

    try:
        import psutil

        info["memory_total_gb"] = round(psutil.virtual_memory().total / 1e9, 1)
    except Exception:
        info["memory_total_gb"] = None

    try:
        disk = shutil.disk_usage(logdir if logdir.exists() else Path.home())
        info["disk_total_gb"] = round(disk.total / 1e9, 1)
        info["disk_free_gb"] = round(disk.free / 1e9, 1)
    except Exception:
        info["disk_total_gb"] = None
        info["disk_free_gb"] = None

    info["container"] = _detect_container()

    try:
        import torch

        info["cuda_available"] = torch.cuda.is_available()
        info["cuda_version"] = getattr(torch.version, "cuda", None)
        try:
            info["cudnn_version"] = str(torch.backends.cudnn.version())
        except Exception:
            info["cudnn_version"] = None
        info["cudnn_enabled"] = getattr(torch.backends.cudnn, "enabled", None)
        info["cudnn_benchmark"] = getattr(torch.backends.cudnn, "benchmark", None)
        info["gpu_count"] = torch.cuda.device_count()
        info["gpu_devices"] = []
        for i in range(info["gpu_count"]):
            props = torch.cuda.get_device_properties(i)
            info["gpu_devices"].append(
                {
                    "index": i,
                    "name": props.name,
                    "memory_total_mb": props.total_memory // (1024 * 1024),
                    "compute_capability": f"{props.major}.{props.minor}",
                }
            )
        info["torch"] = {
            "version": torch.__version__,
            "cuda_arch_list": (
                os.environ.get("TORCH_CUDA_ARCH_LIST")
                or (
                    " ".join(torch.cuda.get_arch_list())
                    if hasattr(torch.cuda, "get_arch_list")
                    else None
                )
            ),
            "default_dtype": str(torch.get_default_dtype()),
            "compile_available": hasattr(torch, "compile"),
        }
    except Exception:
        info.setdefault("cuda_available", False)
        info["cuda_version"] = None
        info["cudnn_version"] = None
        info["cudnn_enabled"] = None
        info["cudnn_benchmark"] = None
        info["gpu_count"] = 0
        info["gpu_devices"] = []
        info["torch"] = None

    return info


def _detect_container() -> dict[str, Any]:
    if os.path.exists("/.dockerenv"):
        return {
            "is_container": True,
            "type": "docker",
            "image": os.environ.get("SRB_DOCKER_IMAGE"),
        }
    if os.environ.get("REMOTE_CONTAINERS"):
        return {
            "is_container": True,
            "type": "devcontainer",
            "image": os.environ.get("SRB_DOCKER_IMAGE"),
        }
    try:
        cgroup = Path("/proc/1/cgroup").read_text()
        if "docker" in cgroup or "containerd" in cgroup:
            return {
                "is_container": True,
                "type": "docker",
                "image": os.environ.get("SRB_DOCKER_IMAGE"),
            }
    except Exception as exc:
        logging.debug(f"Could not inspect container cgroup: {exc}")
    return {"is_container": False, "type": None, "image": None}


def _collect_environment_variables() -> dict[str, str | None]:
    return {var: os.environ.get(var) for var in _ENV_VAR_ALLOWLIST}


def _collect_dependencies() -> dict[str, str | None]:
    from importlib.metadata import PackageNotFoundError
    from importlib.metadata import version as get_version

    deps: dict[str, str | None] = {}
    for pkg in _DEPENDENCY_PACKAGES:
        if pkg == "torch":
            try:
                import torch

                deps["torch"] = torch.__version__
            except Exception:
                deps["torch"] = None
        else:
            try:
                deps[pkg] = get_version(pkg)
            except PackageNotFoundError:
                deps[pkg] = None
    return deps


def _collect_run_info(
    env_id: str,
    workflow: str,
    algo: str | None,
    logdir: Path,
    agent_subcommand: str,
    cli_kwargs: dict[str, Any] | None = None,
    forwarded_args: list[str] | None = None,
    env_cfg: Any | None = None,
    agent_cfg: Any | None = None,
    hydra_overrides: Sequence[str] | None = None,
) -> dict[str, Any]:
    cli_kwargs = cli_kwargs or {}
    forwarded = _redacted_arguments(forwarded_args)
    hydra = _redacted_arguments(hydra_overrides)
    env_snapshot = _snapshot_config(env_cfg)
    agent_snapshot = _snapshot_config(agent_cfg)
    _, seed = _seed_from_configs(env_cfg, agent_cfg)

    run_info = {
        "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "agent_command": str(cli_kwargs.get("agent_command") or "agent"),
        "agent_subcommand": agent_subcommand,
        "workflow": workflow,
        "env_id": env_id,
        "algorithm": algo,
        "logdir": logdir.as_posix(),
        "command_line_args": redact_secret_arguments(sys.argv),
        "forwarded_args": forwarded,
        "cli_reproduce": _build_cli_reproduce(
            agent_subcommand=agent_subcommand,
            env_id=env_id,
            cli_kwargs=cli_kwargs,
            forwarded_args=forwarded,
            logdir=logdir,
        ),
        "seed": seed,
        "config_path": cli_kwargs.get("config"),
        "model_path": str(cli_kwargs["model"]) if cli_kwargs.get("model") else None,
        "continue_training": bool(cli_kwargs.get("continue_training", False)),
        "headless": bool(cli_kwargs.get("headless", False)),
        "hide_ui": bool(cli_kwargs.get("hide_ui", False)),
        "video_enable": bool(cli_kwargs.get("video_enable", False)),
        "enable_cameras": bool(cli_kwargs.get("enable_cameras", False)),
        "rendering_mode": cli_kwargs.get("rendering_mode"),
        "xr": bool(cli_kwargs.get("xr", False)),
        "kit_args": _redact_kit_args(cli_kwargs.get("kit_args", "")),
        "interfaces": _cli_list(cli_kwargs.get("interface")),
        "hardware": _cli_list(cli_kwargs.get("hardware")),
        "collector": cli_kwargs.get("collector"),
        "num_episodes": cli_kwargs.get("num_episodes"),
        "horizon": cli_kwargs.get("horizon"),
        "success_only": bool(cli_kwargs.get("success_only", False)),
        "teleop_device": _cli_list(cli_kwargs.get("teleop_device")),
        "env_cfg": env_snapshot,
        "agent_cfg": agent_snapshot,
    }
    if hydra_overrides is not None:
        run_info["hydra_overrides"] = hydra
    if (
        cli_kwargs.get("demo_dataset") is not None
        or cli_kwargs.get("demo_max_transitions") is not None
    ):
        run_info["demo_dataset"] = cli_kwargs.get("demo_dataset")
        run_info["demo_max_transitions"] = cli_kwargs.get("demo_max_transitions")
    return run_info


def _redact_kit_args(value: Any) -> Any:
    if not isinstance(value, str):
        return value
    try:
        arguments = shlex.split(value)
    except ValueError:
        arguments = value.split()
    redacted = redact_secret_arguments(arguments)
    if redacted == arguments:
        return redact_secret_text(value)
    return shlex.join(redacted)


def _cli_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, (str, bytes)):
        return [value]
    return list(value)


def _build_cli_reproduce(
    agent_subcommand: str,
    env_id: str,
    cli_kwargs: dict[str, Any],
    forwarded_args: list[str],
    logdir: Path,
) -> str:
    agent_command = str(cli_kwargs.get("agent_command") or "agent")
    parts = ["srb", agent_command, agent_subcommand]
    parts.extend(["--env", env_id])

    if hardware := _cli_list(cli_kwargs.get("hardware")):
        parts.extend(["--hardware", *[str(item) for item in hardware]])
    if algo := cli_kwargs.get("algo"):
        parts.extend(["--algo", str(algo)])
    if cli_kwargs.get("demo_dataset") is not None:
        parts.extend(["--demo-dataset", str(cli_kwargs["demo_dataset"])])
    if cli_kwargs.get("demo_max_transitions") is not None:
        parts.extend(
            ["--demo-max-transitions", str(cli_kwargs["demo_max_transitions"])]
        )
    if cli_kwargs.get("headless"):
        parts.append("--headless")
    if cli_kwargs.get("hide_ui"):
        parts.append("--hide_ui")
    if cli_kwargs.get("video_enable"):
        parts.append("--video")
    if cli_kwargs.get("enable_cameras"):
        parts.append("--enable_cameras")
    if ifaces := _cli_list(cli_kwargs.get("interface")):
        parts.extend(["--interface", *[str(i) for i in ifaces]])
    if model := cli_kwargs.get("model"):
        parts.extend(["--model", str(model)])
    if cli_kwargs.get("continue_training"):
        parts.append("--continue")
    if rendering_mode := cli_kwargs.get("rendering_mode"):
        parts.extend(["--rendering_mode", str(rendering_mode)])
    if teleop_device := _cli_list(cli_kwargs.get("teleop_device")):
        parts.extend(["--teleop_device", *[str(item) for item in teleop_device]])
    if collector := cli_kwargs.get("collector"):
        parts.extend(["--collector", str(collector)])
    if cli_kwargs.get("num_episodes") is not None:
        parts.extend(["--num_episodes", str(cli_kwargs["num_episodes"])])
    if cli_kwargs.get("horizon") is not None:
        parts.extend(["--horizon", str(cli_kwargs["horizon"])])
    if cli_kwargs.get("success_only"):
        parts.append("--success_only")

    hydra_cfg = logdir / ".hydra" / "config.yaml"
    parts.extend(["--cfg", hydra_cfg.as_posix()])

    if forwarded_args:
        parts.append("--")
        parts.extend(forwarded_args)

    return " ".join(shlex.quote(part) for part in parts)


def _num_envs(env: "gymnasium.Env | Any") -> int | None:
    """Return the parallel environment count, or ``None`` if unreadable.

    Check the unwrapped environment too; unreadable counts remain unknown.
    """
    for candidate in (env, getattr(env, "unwrapped", None)):
        if candidate is None:
            continue
        value = getattr(candidate, "num_envs", None)
        # Reject coercion so strings and floats cannot become plausible counts.
        count = _exact_integer(value, positive=True)
        if count is not None:
            return count
    return None


def _collect_environment_info(
    env: "gymnasium.Env | Any | None",
) -> dict[str, Any] | None:
    if env is None:
        return None
    try:
        info: dict[str, Any] = {}
        info["num_envs"] = _num_envs(env)
        info["max_episode_steps"] = getattr(env, "max_episode_steps", None) or getattr(
            getattr(env, "spec", None), "max_episode_steps", None
        )
        info["observation_space"] = _serialize_space(env.observation_space)
        info["action_space"] = _serialize_space(env.action_space)
        return info
    except Exception:
        return None


def _json_safe_bound(value: Any) -> Any:
    """Encode non-finite space bounds as strings.

    Gym spaces allow infinite bounds, but the metadata normalizer rejects them.
    """
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, list):
        return [_json_safe_bound(item) for item in value]
    return value


def _uniform_bound(bound: Any) -> Any:
    """Return one scalar for uniform Box bounds, else the full nested list."""
    flat = _json_safe_bound(bound.ravel().tolist())
    # Check every element: comparing outer rows would collapse a dimension.
    if len(set(map(str, flat))) == 1:
        return flat[0]
    return _json_safe_bound(bound.tolist())


def _serialize_space(space: "gymnasium.spaces.Space") -> dict[str, Any]:
    import gymnasium.spaces

    if isinstance(space, gymnasium.spaces.Box):
        low = _uniform_bound(space.low)
        high = _uniform_bound(space.high)
        return {
            "type": "Box",
            "shape": list(space.shape),
            "dtype": str(space.dtype),
            "low": low,
            "high": high,
        }
    elif isinstance(space, gymnasium.spaces.Dict):
        return {
            "type": "Dict",
            "spaces": {k: _serialize_space(v) for k, v in space.spaces.items()},
        }
    elif isinstance(space, gymnasium.spaces.Discrete):
        return {"type": "Discrete", "n": int(space.n), "start": int(space.start)}
    elif isinstance(space, gymnasium.spaces.MultiBinary):
        n = int(space.n) if isinstance(space.n, Integral) else list(space.n)
        return {"type": "MultiBinary", "n": n}
    elif isinstance(space, gymnasium.spaces.MultiDiscrete):
        return {
            "type": "MultiDiscrete",
            "nvec": space.nvec.tolist(),
            "start": space.start.tolist(),
        }
    elif isinstance(space, gymnasium.spaces.Tuple):
        return {
            "type": "Tuple",
            "spaces": [_serialize_space(s) for s in space.spaces],
        }
    else:
        return {"type": type(space).__name__, "repr": str(space)}
