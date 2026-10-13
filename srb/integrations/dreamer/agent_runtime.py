"""Owned single-environment SRB runtime for Dreamer process workers.

The module is safe to import before Isaac Sim starts.  Production imports that
touch Isaac, Hydra runtime state, or Torch conversion live behind method calls
made only inside the spawned worker process.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import random
import re
import shlex
import sys
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Callable, NoReturn, Protocol

import numpy as np

from .process_projection import (
    DreamerProcessProjection,
)
from .process_protocol import (
    INFO_ALLOWLIST,
    DreamerProcessProtocolError,
    schema_hashes_for,
    validate_landscaping_contract,
)

if TYPE_CHECKING:
    from .worker_bootstrap import DreamerWorkerLaunchSpec

_MAX_CLEANUP_DIAGNOSTIC_BYTES = 512


class DreamerAgentRuntimeError(RuntimeError):
    """Raised when the owned child runtime cannot be built or closed safely."""


_PARENT_CONFIG_VERSION = 1
_MAX_PARENT_CONFIG_DEPTH = 32
_MAX_PARENT_CONFIG_ITEMS = 10_000
_MAX_PARENT_CONFIG_BYTES = 1_048_576
_MAX_PARENT_NAME_BYTES = 4_096
_MAX_WORKER_COUNT = 65_536
_MAX_WORKER_SEED = 2**32 - 1
_MIN_INT64 = -(2**63)
_MAX_INT64 = 2**63 - 1
_OVERRIDE_KEY = re.compile(
    r"^(?:agent(?:\.[A-Za-z_][A-Za-z0-9_-]*)*|env(?:\.[A-Za-z_][A-Za-z0-9_-]*)*)$"
)
_DEVICE = re.compile(r"^(?:cpu|cuda(?::(?:0|[1-9][0-9]{0,4}))?)$")
_PARENT_ENV_OVERRIDE_KEYS = frozenset(
    {
        "env.seed",
        "env.num_envs",
        "env.scene.num_envs",
        "env.sim.device",
    }
)
_LAUNCHER_SETTING_KEYS = frozenset(
    {
        "device",
        "enable_cameras",
        "experience",
        "kit_args",
        "rendering",
        "rendering_mode",
        "video",
        "visualizer",
        "xr",
    }
)
_WORKER_AUTHORITY_OVERRIDE_KEYS = frozenset({"env.seed", "env.scene.num_envs"})
_RUN_OPTION_AUTHORITY_KEYS = frozenset(
    {
        "agent_cfg",
        "base_seed",
        "config_fingerprint",
        "continue_training",
        "env_count",
        "forwarded_args",
        "hydra_config_path",
        "hydra_overrides",
        "launcher_settings",
        "logdir",
        "model",
        "obs",
        "sim_device",
        "task_id",
        "workflow",
        "env",
    }
)
_MAX_FORWARD_ARG_BYTES = 4096
_DREAMER_OBSERVATION_MODES = frozenset({"state", "visual"})


@dataclass(frozen=True, slots=True)
class DreamerParentConfig:
    """Immutable, canonical parent-side Dreamer configuration."""

    task_id: str
    obs: str
    seed: int
    worker_count: int
    sim_device: str | None
    fingerprint: str
    _agent_cfg_json: str = field(repr=False)
    _metadata_json: str = field(repr=False)

    @property
    def agent_cfg(self) -> dict[str, Any]:
        """Return an isolated copy of the effective Dreamer agent mapping."""
        value = json.loads(self._agent_cfg_json)
        if not isinstance(value, dict):  # pragma: no cover - constructor invariant
            raise AssertionError("stored Dreamer agent config is not a mapping")
        return value

    @property
    def metadata(self) -> dict[str, Any]:
        """Return an isolated copy of the canonical fingerprint metadata."""
        value = json.loads(self._metadata_json)
        if not isinstance(value, dict):  # pragma: no cover - constructor invariant
            raise AssertionError("stored Dreamer metadata is not a mapping")
        return value


def _copy_json_safe(
    value: Any,
    *,
    label: str,
    depth: int = 0,
    count: list[int] | None = None,
) -> Any:
    if count is None:
        count = [0]
    if depth > _MAX_PARENT_CONFIG_DEPTH:
        raise DreamerAgentRuntimeError(
            f"{label} exceeds maximum depth {_MAX_PARENT_CONFIG_DEPTH}"
        )
    count[0] += 1
    if count[0] > _MAX_PARENT_CONFIG_ITEMS:
        raise DreamerAgentRuntimeError(
            f"{label} exceeds maximum item count {_MAX_PARENT_CONFIG_ITEMS}"
        )

    value_type = type(value)
    if value is None or value_type in (str, bool):
        return value
    if value_type is int:
        if not _MIN_INT64 <= value <= _MAX_INT64:
            raise DreamerAgentRuntimeError(f"{label} contains an out-of-range integer")
        return value
    if value_type is float:
        if not math.isfinite(value):
            raise DreamerAgentRuntimeError(f"{label} contains a non-finite float")
        return value
    if value_type is list:
        return [
            _copy_json_safe(item, label=label, depth=depth + 1, count=count)
            for item in value
        ]
    if value_type is dict:
        result: dict[str, Any] = {}
        for key, item in value.items():
            if type(key) is not str:
                raise DreamerAgentRuntimeError(
                    f"{label} contains a non-string mapping key"
                )
            count[0] += 1
            if count[0] > _MAX_PARENT_CONFIG_ITEMS:
                raise DreamerAgentRuntimeError(
                    f"{label} exceeds maximum item count {_MAX_PARENT_CONFIG_ITEMS}"
                )
            result[key] = _copy_json_safe(
                item, label=label, depth=depth + 1, count=count
            )
        return result
    raise DreamerAgentRuntimeError(
        f"{label} contains unsupported value type {value_type.__name__}"
    )


def _canonical_json(value: Any, *, label: str) -> tuple[Any, str]:
    copied = _copy_json_safe(value, label=label)
    encoded = json.dumps(
        copied,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )
    if len(encoded.encode("utf-8")) > _MAX_PARENT_CONFIG_BYTES:
        raise DreamerAgentRuntimeError(
            f"{label} exceeds maximum serialized size {_MAX_PARENT_CONFIG_BYTES}"
        )
    return copied, encoded


def _bounded_parent_name(value: Any, *, label: str) -> str:
    if type(value) is not str or not value:
        raise DreamerAgentRuntimeError(f"{label} must be a non-empty string")
    if len(value.encode("utf-8")) > _MAX_PARENT_NAME_BYTES:
        raise DreamerAgentRuntimeError(f"{label} is too long")
    return value


def _parent_config_path(value: Any, *, label: str) -> Path:
    try:
        return Path(value).expanduser().resolve()
    except (TypeError, ValueError, OSError) as error:
        raise DreamerAgentRuntimeError(f"{label} is not a valid path") from error


def _load_parent_yaml(path: Path, *, label: str) -> dict[str, Any]:
    try:
        import yaml

        with path.open("rb") as stream:
            raw = stream.read(_MAX_PARENT_CONFIG_BYTES + 1)
        if len(raw) > _MAX_PARENT_CONFIG_BYTES:
            raise DreamerAgentRuntimeError(
                f"{label} exceeds maximum serialized size {_MAX_PARENT_CONFIG_BYTES}"
            )
        value = yaml.safe_load(raw.decode("utf-8"))
    except DreamerAgentRuntimeError:
        raise
    except Exception as error:
        raise DreamerAgentRuntimeError(
            f"could not safely load {label}: {path}"
        ) from error
    if type(value) is not dict:
        raise DreamerAgentRuntimeError(f"{label} must contain a plain mapping")
    copied, _encoded = _canonical_json(value, label=label)
    return copied


def _deep_parent_merge(
    base: dict[str, Any], override: dict[str, Any]
) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        current = result.get(key)
        if type(current) is dict and type(value) is dict:
            result[key] = _deep_parent_merge(current, value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _reject_custom_resolvers(value: Any, *, label: str) -> None:
    if type(value) is str:
        cursor = 0
        while True:
            start = value.find("${", cursor)
            if start < 0:
                return
            end = value.find("}", start + 2)
            if end < 0:
                return
            if ":" in value[start + 2 : end]:
                raise DreamerAgentRuntimeError(
                    f"{label} contains unsupported resolver interpolation"
                )
            cursor = end + 1
    elif type(value) is list:
        for item in value:
            _reject_custom_resolvers(item, label=label)
    elif type(value) is dict:
        for item in value.values():
            _reject_custom_resolvers(item, label=label)


def _reject_unresolved_interpolation(value: Any) -> None:
    if type(value) is str and "${" in value:
        raise DreamerAgentRuntimeError(
            "configuration contains unresolved interpolation"
        )
    if type(value) is list:
        for item in value:
            _reject_unresolved_interpolation(item)
    elif type(value) is dict:
        for item in value.values():
            _reject_unresolved_interpolation(item)


def _unquoted_override_view(value: str) -> str:
    result: list[str] = []
    quote: str | None = None
    escaped = False
    nesting: list[str] = []
    pairs = {"]": "[", "}": "{"}
    for character in value:
        if quote is not None:
            result.append(" ")
            if escaped:
                escaped = False
            elif character == "\\" and quote == '"':
                escaped = True
            elif character == quote:
                quote = None
            continue
        if character in {"'", '"'}:
            quote = character
            result.append(" ")
        elif character in "[{":
            nesting.append(character)
            result.append(character)
        elif character in "]}":
            if nesting and nesting[-1] == pairs[character]:
                nesting.pop()
            result.append(character)
        elif character == "," and not nesting:
            raise DreamerAgentRuntimeError("Hydra sweep overrides are unsupported")
        else:
            result.append(character)
    return "".join(result)


def _validate_parent_override(override: Any) -> str:
    if type(override) is not str:
        raise DreamerAgentRuntimeError("Hydra overrides must be strings")
    if override.startswith(("+", "~")):
        raise DreamerAgentRuntimeError(
            "Hydra add, force-add, and delete operations are unsupported"
        )
    key, separator, value = override.partition("=")
    if not separator or not key or not _OVERRIDE_KEY.fullmatch(key):
        raise DreamerAgentRuntimeError(f"unsupported Hydra override: {override!r}")
    if not (key == "agent" or key.startswith(("agent.", "env."))):
        raise DreamerAgentRuntimeError(
            f"Hydra override requires task-registry composition: {key!r}"
        )
    view = _unquoted_override_view(value)
    if "(" in view or ")" in view:
        raise DreamerAgentRuntimeError("Hydra function overrides are unsupported")
    _reject_custom_resolvers(value, label="Hydra override")
    return override


def _parent_applied_overrides(overrides: tuple[str, ...]) -> tuple[str, ...]:
    """Overrides the bounded parent composition applies to itself.

    The parent applies ``agent.*`` and its runtime-defining ``env`` keys.
    Other ``env.*`` task parameters stay in the recorded list for worker-side
    task-registry composition.
    """
    return tuple(
        override
        for override in overrides
        if (
            (key := override.partition("=")[0]) == "agent"
            or key.startswith("agent.")
            or key in _PARENT_ENV_OVERRIDE_KEYS
        )
    )


def _ordered_parent_overrides(hydra_overrides: Any) -> tuple[str, ...]:
    if isinstance(hydra_overrides, (str, bytes)):
        raise DreamerAgentRuntimeError("hydra_overrides must be a sequence of strings")
    try:
        values = tuple(hydra_overrides)
    except TypeError as error:
        raise DreamerAgentRuntimeError(
            "hydra_overrides must be a sequence of strings"
        ) from error
    return tuple(_validate_parent_override(value) for value in values)


def _resolve_parent_composition(
    composed: dict[str, Any], overrides: tuple[str, ...]
) -> dict[str, Any]:
    _reject_custom_resolvers(composed, label="configuration")
    try:
        from omegaconf import OmegaConf
    except ImportError as error:
        raise DreamerAgentRuntimeError(
            "OmegaConf is required to compose Dreamer parent configuration"
        ) from error
    try:
        config = OmegaConf.create(composed)
        for override in overrides:
            config = OmegaConf.merge(config, OmegaConf.from_dotlist([override]))
        resolved = OmegaConf.to_container(config, resolve=True, throw_on_missing=True)
    except Exception as error:
        raise DreamerAgentRuntimeError(
            "Dreamer parent configuration composition failed"
        ) from error
    _reject_unresolved_interpolation(resolved)
    copied, _encoded = _canonical_json(resolved, label="effective configuration")
    if type(copied) is not dict:  # pragma: no cover - root invariant
        raise DreamerAgentRuntimeError("effective configuration must be a mapping")
    return copied


def _required_parent_int(value: Any, *, label: str, minimum: int, maximum: int) -> int:
    if type(value) is not int:
        raise DreamerAgentRuntimeError(f"{label} must be an integer")
    if not minimum <= value <= maximum:
        raise DreamerAgentRuntimeError(
            f"{label} must be between {minimum} and {maximum}"
        )
    return value


def _derive_parent_runtime(env_cfg: dict[str, Any]) -> tuple[int, int, str | None]:
    scene_cfg = env_cfg.get("scene")
    if type(scene_cfg) is not dict:
        raise DreamerAgentRuntimeError("env.scene must be a mapping")
    worker_value = env_cfg.get("num_envs")
    if worker_value is None:
        worker_value = scene_cfg.get("num_envs", 1)
    worker_count = _required_parent_int(
        worker_value,
        label="env.num_envs",
        minimum=1,
        maximum=_MAX_WORKER_COUNT,
    )
    seed = _required_parent_int(
        env_cfg.get("seed", 0),
        label="env.seed",
        minimum=0,
        maximum=_MAX_WORKER_SEED - (worker_count - 1),
    )

    sim_device: str | None = None
    if "sim" in env_cfg:
        sim_cfg = env_cfg["sim"]
        if type(sim_cfg) is not dict:
            raise DreamerAgentRuntimeError("env.sim must be a mapping")
        if "device" in sim_cfg:
            device = sim_cfg["device"]
            if type(device) is not str or not _DEVICE.fullmatch(device):
                raise DreamerAgentRuntimeError(
                    "env.sim.device must be cpu, cuda, or cuda:<non-negative index>"
                )
            sim_device = device
    return seed, worker_count, sim_device


def load_dreamer_parent_config(
    *,
    task_id: str,
    obs: str,
    policy_config_path: str | os.PathLike[str],
    hydra_config_path: str | os.PathLike[str] | None,
    hydra_overrides: Sequence[str],
) -> DreamerParentConfig:
    """Compose bounded parent configuration without importing child runtimes."""
    canonical_task_id = _bounded_parent_name(task_id, label="task_id")
    canonical_obs = _bounded_parent_name(obs, label="obs")
    if canonical_obs not in _DREAMER_OBSERVATION_MODES:
        raise DreamerAgentRuntimeError("obs must be exactly 'state' or 'visual'")
    policy_path = _parent_config_path(policy_config_path, label="policy_config_path")
    custom_path = (
        None
        if hydra_config_path is None
        else _parent_config_path(hydra_config_path, label="hydra_config_path")
    )
    overrides = _ordered_parent_overrides(hydra_overrides)

    policy_source = _load_parent_yaml(policy_path, label="policy configuration")
    if "agent" in policy_source:
        policy_agent = policy_source["agent"]
        if type(policy_agent) is not dict:
            raise DreamerAgentRuntimeError(
                "wrapped policy configuration agent section must be a mapping"
            )
    else:
        policy_agent = policy_source

    custom_source: dict[str, Any] | None = None
    custom_agent: dict[str, Any] = {}
    custom_env: dict[str, Any] = {}
    if custom_path is not None:
        custom_source = _load_parent_yaml(custom_path, label="custom configuration")
        if "agent" in custom_source:
            if type(custom_source["agent"]) is not dict:
                raise DreamerAgentRuntimeError(
                    "custom configuration agent section must be a mapping"
                )
            custom_agent = custom_source["agent"]
        if "env" in custom_source:
            if type(custom_source["env"]) is not dict:
                raise DreamerAgentRuntimeError(
                    "custom configuration env section must be a mapping"
                )
            custom_env = custom_source["env"]

    composed = {
        "agent": _deep_parent_merge(policy_agent, custom_agent),
        "env": _deep_parent_merge(
            {"seed": 0, "num_envs": None, "scene": {"num_envs": 1}},
            custom_env,
        ),
    }
    effective = _resolve_parent_composition(
        composed, _parent_applied_overrides(overrides)
    )
    agent_cfg = effective.get("agent")
    env_cfg = effective.get("env")
    if type(agent_cfg) is not dict:
        raise DreamerAgentRuntimeError(
            "effective agent configuration must be a mapping"
        )
    if type(env_cfg) is not dict:
        raise DreamerAgentRuntimeError("effective env configuration must be a mapping")
    seed, worker_count, sim_device = _derive_parent_runtime(env_cfg)
    _agent_copy, agent_json = _canonical_json(agent_cfg, label="agent_cfg")

    metadata = {
        "version": _PARENT_CONFIG_VERSION,
        "task_id": canonical_task_id,
        "obs": canonical_obs,
        "sources": {
            "policy": {"path": str(policy_path), "config": policy_source},
            "custom": (
                None
                if custom_path is None
                else {"path": str(custom_path), "config": custom_source}
            ),
            "hydra_overrides": list(overrides),
        },
        "effective": {"agent": agent_cfg, "env": env_cfg},
        "derived": {
            "seed": seed,
            "worker_count": worker_count,
            "sim_device": sim_device,
        },
        "fingerprint_algorithm": "sha256",
    }
    _metadata_copy, metadata_json = _canonical_json(
        metadata, label="parent configuration metadata"
    )
    fingerprint = hashlib.sha256(metadata_json.encode("utf-8")).hexdigest()
    return DreamerParentConfig(
        task_id=canonical_task_id,
        obs=canonical_obs,
        seed=seed,
        worker_count=worker_count,
        sim_device=sim_device,
        fingerprint=fingerprint,
        _agent_cfg_json=agent_json,
        _metadata_json=metadata_json,
    )


class _RuntimeHooks(Protocol):
    def apply_determinism(self, seed: int) -> None: ...

    def require_isaac_sim(self) -> None: ...

    def launch_app(self, kwargs: Mapping[str, Any]) -> Any: ...

    def run_startup(self) -> None: ...

    def compose_env_cfg(self, task_id: str, overrides: Sequence[str]) -> Any: ...

    def make_env(self, task_id: str, *, cfg: Any, render_mode: str | None) -> Any: ...

    def wrap_env(self, env: Any) -> Any: ...

    def inspect_env(self, env: Any) -> Any: ...

    def to_host(self, value: Any) -> np.ndarray: ...


def _bounded_text(value: str, *, maximum: int) -> str:
    raw = value.encode("utf-8", errors="replace")[:maximum]
    return raw.decode("utf-8", errors="ignore")


def _cleanup_diagnostic(errors: Sequence[tuple[str, BaseException]]) -> str:
    detail = "; ".join(
        f"{name}: {type(error).__name__}: {error}" for name, error in errors
    )
    return _bounded_text(detail, maximum=_MAX_CLEANUP_DIAGNOSTIC_BYTES)


def _close_owned(
    env: Any | None, launcher: Any | None
) -> list[tuple[str, BaseException]]:
    errors: list[tuple[str, BaseException]] = []
    if env is not None:
        try:
            env.close()
        except BaseException as exc:
            errors.append(("env.close", exc))
    if launcher is not None:
        try:
            launcher.app.close()
        except BaseException as exc:
            errors.append(("launcher.app.close", exc))
    return errors


def _raise_runtime(message: str, error: BaseException | None = None) -> NoReturn:
    if error is None:
        raise DreamerAgentRuntimeError(message)
    raise DreamerAgentRuntimeError(message) from error


def _host_scalar(value: Any, *, name: str, to_host: Any) -> Any:
    try:
        array = np.asarray(to_host(value))
    except (TypeError, ValueError, RuntimeError) as exc:
        _raise_runtime(f"{name} is not CPU-readable", exc)
    if array.shape == ():
        return array.item()
    if array.shape == (1,):
        return array[0].item()
    _raise_runtime(f"{name} must be scalar or one-env batched, got {array.shape}")


def _reward(value: Any, *, to_host: Any) -> float:
    scalar = _host_scalar(value, name="reward", to_host=to_host)
    if type(scalar) not in (int, float) or isinstance(scalar, bool):
        _raise_runtime("reward must be a real scalar")
    result = float(scalar)
    if not math.isfinite(result):
        _raise_runtime("reward must be finite")
    return result


def _flag(value: Any, *, name: str, to_host: Any) -> bool:
    scalar = _host_scalar(value, name=name, to_host=to_host)
    if type(scalar) not in (bool, np.bool_):
        _raise_runtime(f"{name} must have boolean dtype")
    return bool(scalar)


def _numeric_info(value: Any, *, to_host: Any) -> dict[str, bool | int | float]:
    if not isinstance(value, Mapping):
        _raise_runtime("environment info must be a mapping")
    result: dict[str, bool | int | float] = {}
    for key, item in value.items():
        if type(key) is not str:
            _raise_runtime("environment info keys must be strings")
        if key not in INFO_ALLOWLIST:
            continue
        scalar = _host_scalar(item, name=f"info.{key}", to_host=to_host)
        if type(scalar) in (bool, np.bool_):
            result[key] = bool(scalar)
        elif isinstance(scalar, (int, np.integer)) and not isinstance(scalar, bool):
            result[key] = int(scalar)
        elif isinstance(scalar, (float, np.floating)) and math.isfinite(float(scalar)):
            result[key] = float(scalar)
        else:
            _raise_runtime(f"info.{key} must be finite numeric data")
    return result


def _collect_landscaping_contract(
    env: Any, *, env_id: str
) -> Mapping[str, str | None] | None:
    """Collect the canonical block from the live worker environment.

    Tasks without the hook need no block. If present, collection must succeed;
    omitting a failed block could make an incompatible checkpoint look portable.
    """

    unwrapped = getattr(env, "unwrapped", env)
    metadata_fn = getattr(unwrapped, "landscaping_contract_metadata", None)
    if not callable(metadata_fn):
        return None

    from srb.utils.metadata import collect_landscaping_contract_metadata

    try:
        contract = collect_landscaping_contract_metadata(
            env=env,
            env_id=env_id,
            algo="dreamer",
        )
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
        raise DreamerAgentRuntimeError(
            "live landscaping contract metadata collection failed"
        ) from exc
    if contract is None:
        # Shared callers retain warning-only behavior; worker hooks must
        # produce a canonical block.
        raise DreamerAgentRuntimeError(
            "live landscaping contract metadata collection failed"
        )
    try:
        return MappingProxyType(validate_landscaping_contract(contract))
    except DreamerProcessProtocolError as exc:
        raise DreamerAgentRuntimeError(
            "live landscaping contract metadata is not a bounded flat block"
        ) from exc


class SrbDreamerWorkerEnvironment:
    """One owned wrapped SRB environment projected onto the worker seam."""

    __slots__ = (
        "action_projection",
        "demo_projection",
        "_landscaping_contract",
        "learner_projection",
        "observation_projection",
        "schema_hashes",
        "_device",
        "_env",
        "_launcher",
        "_to_env_action",
        "_to_host",
        "_closed",
        "_env_closed",
    )

    def __init__(
        self,
        *,
        env: Any,
        launcher: Any,
        learner_projection: Any,
        device: Any,
        to_host: Any,
        demo_projection: Mapping[str, Any] | None = None,
        landscaping_contract: Mapping[str, Any] | None = None,
    ) -> None:
        self._env = env
        self._launcher = launcher
        if type(getattr(learner_projection, "num_envs", None)) is not int or (
            learner_projection.num_envs != 1
        ):
            _raise_runtime("Dreamer learner projection must be configured for one env")
        self.learner_projection = learner_projection
        self._device = device
        self._to_host = to_host
        self.action_projection = DreamerProcessProjection(
            learner_projection.action_space
        )
        self.observation_projection = DreamerProcessProjection(
            learner_projection.observation_space
        )
        self.schema_hashes = schema_hashes_for(
            self.action_projection.schema_hash,
            self.observation_projection.schema_hash,
            info_keys=INFO_ALLOWLIST,
        )
        self.demo_projection = demo_projection
        if landscaping_contract is None:
            self._landscaping_contract = None
        else:
            try:
                self._landscaping_contract = MappingProxyType(
                    validate_landscaping_contract(landscaping_contract)
                )
            except DreamerProcessProtocolError as exc:
                _raise_runtime(
                    "worker landscaping contract metadata is invalid",
                    exc,
                )
        self._closed = False
        self._env_closed = False

    @property
    def landscaping_contract(self) -> Mapping[str, Any] | None:
        """Return a fresh read-only copy of the live contract block."""

        if self._landscaping_contract is None:
            return None
        return MappingProxyType(dict(self._landscaping_contract))

    def _observation(self, value: Any, *, name: str) -> Any:
        try:
            projected = self.learner_projection.project_observation_batch(value)
            return self.learner_projection.unpack_observation_row(projected)
        except Exception as exc:
            _raise_runtime(f"{name} disagrees with the shared learner projection", exc)

    def reset(self, *, seed: int | None) -> Any:
        if self._closed:
            _raise_runtime("worker environment is closed")
        result = self._env.reset(seed=seed)
        if type(result) is not tuple or len(result) != 2:
            _raise_runtime("SRB reset must return (observation, info)")
        observation, info = result
        from .worker_bootstrap import WorkerResetResult

        return WorkerResetResult(
            observation=self._observation(observation, name="reset observation"),
            info=_numeric_info(info, to_host=self._to_host),
        )

    def step(self, action: Any) -> Any:
        if self._closed:
            _raise_runtime("worker environment is closed")
        try:
            env_action = self.learner_projection.coerce_action_row(
                action, device=self._device
            )
        except Exception as exc:
            _raise_runtime("action disagrees with the shared learner projection", exc)
        result = self._env.step(env_action)
        if type(result) is not tuple or len(result) != 5:
            _raise_runtime(
                "SRB step must return observation, reward, terminated, truncated, info"
            )
        observation, reward, terminated, truncated, info = result
        from .worker_bootstrap import WorkerStepResult

        terminated_value = _flag(terminated, name="terminated", to_host=self._to_host)
        truncated_value = _flag(truncated, name="truncated", to_host=self._to_host)
        post_reset_observation = None
        if terminated_value or truncated_value:
            try:
                # The outermost owner (e.g. observation stacking) wins.
                get_attr = self._env.get_wrapper_attr
                final_observation = get_attr("last_final_observation")
                final_ids = get_attr("last_final_observation_ids")
                final_mask = get_attr("last_final_observation_mask")
            except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
                _raise_runtime("SRB final observation seam is unavailable", exc)
            if final_observation is None or (
                isinstance(final_observation, Mapping) and not final_observation
            ):
                _raise_runtime("SRB final observation is missing or malformed")
            try:
                ids = np.asarray(self._to_host(final_ids))
            except (TypeError, ValueError, RuntimeError) as exc:
                _raise_runtime("SRB final observation IDs are not CPU-readable", exc)
            if (
                ids.shape != (1,)
                or ids.dtype.kind not in "iu"
                or ids.dtype.kind == "b"
                or int(ids[0]) != 0
            ):
                _raise_runtime("SRB final observation IDs are not the scalar env row")
            try:
                mask = np.asarray(self._to_host(final_mask))
            except (TypeError, ValueError, RuntimeError) as exc:
                _raise_runtime("SRB final observation mask is not CPU-readable", exc)
            if mask.dtype != np.dtype(np.bool_) or mask.shape != (1,):
                _raise_runtime(
                    "SRB final observation mask must be bool with shape (1,)"
                )
            expected_mask = np.zeros((1,), dtype=np.bool_)
            expected_mask[ids] = True
            if not np.array_equal(mask, expected_mask):
                _raise_runtime(
                    "SRB final observation mask disagrees with its terminal IDs"
                )
            final_observation = self._observation(
                final_observation, name="terminal observation"
            )
            post_reset_observation = self._observation(
                observation, name="post-reset first observation"
            )
            observation = final_observation

        return WorkerStepResult(
            observation=(
                observation
                if post_reset_observation is not None
                else self._observation(observation, name="step observation")
            ),
            reward=_reward(reward, to_host=self._to_host),
            terminated=terminated_value,
            truncated=truncated_value,
            info=_numeric_info(info, to_host=self._to_host),
            post_reset_observation=post_reset_observation,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        errors: list[tuple[str, BaseException]] = []
        if not self._env_closed:
            self._env_closed = True
            errors.extend(_close_owned(self._env, None))
        errors.extend(_close_owned(None, self._launcher))
        if errors:
            raise DreamerAgentRuntimeError(
                f"Dreamer worker cleanup failed: {_cleanup_diagnostic(errors)}"
            )

    def prepare_close(self) -> None:
        """Close simulation state while preserving Kit for the protocol ACK."""

        if self._closed or self._env_closed:
            return
        self._env_closed = True
        errors = _close_owned(self._env, None)
        if errors:
            raise DreamerAgentRuntimeError(
                f"Dreamer worker cleanup failed: {_cleanup_diagnostic(errors)}"
            )


def _apply_process_determinism(seed: int) -> None:
    if os.environ.get("PYTHONHASHSEED") != str(seed):
        _raise_runtime(
            "Dreamer worker must start with PYTHONHASHSEED equal to worker_seed"
        )
    os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
    random.seed(seed)
    np.random.seed(seed)

    import torch

    torch.manual_seed(seed)


def _launcher_kwargs(spec: DreamerWorkerLaunchSpec) -> dict[str, Any]:
    kwargs: dict[str, Any] = {
        "device": spec.device,
        "enable_cameras": spec.enable_cameras,
        "headless": spec.headless,
        "xr": spec.xr,
    }
    if spec.visualizer:
        kwargs["visualizer"] = ("kit",)
    if spec.rendering_mode is not None:
        kwargs["rendering_mode"] = spec.rendering_mode
    if spec.experience is not None:
        kwargs["experience"] = spec.experience
    if spec.kit_args:
        kwargs["kit_args"] = " ".join(spec.kit_args)
    return kwargs


def _force_worker_authority(cfg: Any, *, seed: int, device: str) -> None:
    scene = (
        cfg.get("scene") if isinstance(cfg, Mapping) else getattr(cfg, "scene", None)
    )
    if scene is None:
        _raise_runtime("composed task config has no scene section")
    sim = cfg.get("sim") if isinstance(cfg, Mapping) else getattr(cfg, "sim", None)
    if sim is None:
        _raise_runtime("composed task config has no sim section")
    missing = object()
    try:
        if isinstance(scene, Mapping):
            scene["num_envs"] = 1
        else:
            scene.num_envs = 1
        if isinstance(sim, Mapping):
            sim["device"] = device
        else:
            sim.device = device
        if isinstance(cfg, Mapping):
            cfg["seed"] = seed
            if "num_envs" in cfg:
                cfg["num_envs"] = 1
        else:
            cfg.seed = seed
            if getattr(cfg, "num_envs", missing) is not missing:
                cfg.num_envs = 1
    except (AttributeError, TypeError, ValueError) as exc:
        _raise_runtime(
            "task config rejected worker seed/device/num_envs authority", exc
        )
    scene_num_envs = (
        scene.get("num_envs")
        if isinstance(scene, Mapping)
        else getattr(scene, "num_envs", None)
    )
    if type(scene_num_envs) is not int or scene_num_envs != 1:
        _raise_runtime("task config did not retain scene.num_envs=1")
    sim_device = (
        sim.get("device") if isinstance(sim, Mapping) else getattr(sim, "device", None)
    )
    if sim_device != device:
        _raise_runtime("task config did not retain the worker device")
    cfg_seed = (
        cfg.get("seed") if isinstance(cfg, Mapping) else getattr(cfg, "seed", None)
    )
    if type(cfg_seed) is not int or cfg_seed != seed:
        _raise_runtime("task config did not retain the worker seed")
    if isinstance(cfg, Mapping) and "num_envs" in cfg:
        top_num_envs = cfg.get("num_envs")
        if type(top_num_envs) is not int or top_num_envs != 1:
            _raise_runtime("task config did not retain env.num_envs=1")
    elif getattr(cfg, "num_envs", missing) is not missing:
        if type(cfg.num_envs) is not int or cfg.num_envs != 1:
            _raise_runtime("task config did not retain env.num_envs=1")


class _ProductionRuntimeHooks:
    def apply_determinism(self, seed: int) -> None:
        _apply_process_determinism(seed)

    def require_isaac_sim(self) -> None:
        from srb.utils.isaacsim import require_isaac_sim

        require_isaac_sim()

    def launch_app(self, kwargs: Mapping[str, Any]) -> Any:
        from srb.core.app import AppLauncher

        return AppLauncher(**dict(kwargs))

    def run_startup(self) -> None:
        from srb.tasks import ensure_registered
        from srb.utils.cache import warn_if_offline_srb_cache_incomplete
        from srb.utils.isaac_log_filters import install_isaac_startup_log_filters

        install_isaac_startup_log_filters()
        warn_if_offline_srb_cache_incomplete()
        ensure_registered()

    def compose_env_cfg(self, task_id: str, overrides: Sequence[str]) -> Any:
        from srb.hydra.sim import hydra_task_config

        captured: list[Any] = []
        original_argv = sys.argv
        sys.argv = [original_argv[0], *overrides]
        try:

            @hydra_task_config(task_name=task_id)
            def _capture(env_cfg: Any, _agent_cfg: Any = None) -> None:
                captured.append(env_cfg)

            _capture()
        finally:
            sys.argv = original_argv
        if len(captured) != 1:
            _raise_runtime("Hydra task composition did not produce exactly one config")
        return captured[0]

    def make_env(self, task_id: str, *, cfg: Any, render_mode: str | None) -> Any:
        import gymnasium

        return gymnasium.make(id=task_id, cfg=cfg, render_mode=render_mode)

    def wrap_env(self, env: Any) -> Any:
        from srb.wrappers.composition import wrap_env

        return wrap_env(env)

    def inspect_env(self, env: Any) -> Any:
        from srb.core.env.introspect import EnvIntrospector

        return EnvIntrospector.of(env)

    def to_host(self, value: Any) -> np.ndarray:
        from srb.core.spaces.coerce import to_numpy

        return to_numpy(value)


def _build_srb_worker_environment(
    spec: DreamerWorkerLaunchSpec, hooks: _RuntimeHooks
) -> SrbDreamerWorkerEnvironment:
    hooks.apply_determinism(spec.worker_seed)
    hooks.require_isaac_sim()

    launcher: Any | None = None
    env: Any | None = None
    try:
        launcher = hooks.launch_app(_launcher_kwargs(spec))
        hooks.run_startup()
        cfg = hooks.compose_env_cfg(spec.task_id, spec.hydra_overrides)
        _force_worker_authority(
            cfg,
            seed=spec.worker_seed,
            device=spec.device,
        )
        env = hooks.make_env(
            spec.task_id,
            cfg=cfg,
            render_mode="rgb_array" if spec.video else None,
        )
        env = hooks.wrap_env(env)
        view = hooks.inspect_env(env)
        if type(view.num_envs) is not int or view.num_envs != 1:
            _raise_runtime(
                f"Dreamer process worker requires num_envs=1, got {view.num_envs!r}"
            )
        from .learner_projection import DreamerLearnerProjection

        learner_projection = DreamerLearnerProjection.from_spaces(
            single_observation_space=view.single_observation_space,
            single_action_space=view.single_action_space,
            num_envs=view.num_envs,
            device=view.device,
            mode=spec.observation_mode,
            introspection=view,
            unwrapped=getattr(env, "unwrapped", env),
        )
        demo_projection: Mapping[str, Any] | None = None
        try:
            from .demo_projection import DreamerDemoProjection

            demo_projection = DreamerDemoProjection.from_live(
                env,
                learner_projection=learner_projection,
            ).metadata
        except (AttributeError, TypeError, ValueError, RuntimeError, ImportError):
            # Ordinary training needs no demo contract; demo callers fail
            # closed if this descriptor is absent.
            demo_projection = None
        landscaping_contract = _collect_landscaping_contract(
            env,
            env_id=spec.task_id,
        )
        return SrbDreamerWorkerEnvironment(
            env=env,
            launcher=launcher,
            learner_projection=learner_projection,
            device=view.device,
            to_host=hooks.to_host,
            demo_projection=demo_projection,
            landscaping_contract=landscaping_contract,
        )
    except BaseException as primary:
        cleanup_errors = _close_owned(env, launcher)
        if cleanup_errors and hasattr(primary, "add_note"):
            primary.add_note(
                "Dreamer worker cleanup diagnostics: "
                f"{_cleanup_diagnostic(cleanup_errors)}"
            )
        raise


def build_srb_worker_environment(
    spec: DreamerWorkerLaunchSpec,
) -> SrbDreamerWorkerEnvironment:
    """Build one fully owned SRB environment inside a spawned worker."""

    return _build_srb_worker_environment(spec, _ProductionRuntimeHooks())


def _normalize_launcher_settings(
    settings: Mapping[str, Any] | None, *, headless: bool
) -> dict[str, Any]:
    """Normalize the JSON-only launcher subset accepted by worker specs."""
    if type(headless) is not bool:
        raise DreamerAgentRuntimeError("headless must be a boolean")
    if settings is None:
        raw: Mapping[str, Any] = {}
    elif isinstance(settings, Mapping):
        raw = settings
    else:
        raise DreamerAgentRuntimeError("launcher_settings must be a mapping")

    for key in raw:
        if type(key) is not str or key not in _LAUNCHER_SETTING_KEYS:
            raise DreamerAgentRuntimeError(
                f"launcher_settings contains unsupported key {key!r}"
            )

    normalized: dict[str, Any] = {}
    device = raw.get("device")
    if device is not None:
        if type(device) is not str or not _DEVICE.fullmatch(device):
            raise DreamerAgentRuntimeError(
                "launcher_settings.device must be cpu, cuda, or cuda:<index>"
            )
        normalized["device"] = device

    for key in ("enable_cameras", "rendering", "video", "xr"):
        if key in raw:
            value = raw[key]
            if type(value) is not bool:
                raise DreamerAgentRuntimeError(
                    f"launcher_settings.{key} must be a boolean"
                )
            normalized[key] = value

    if "rendering_mode" in raw:
        rendering_mode = raw["rendering_mode"]
        if rendering_mode is not None:
            if type(rendering_mode) is not str or rendering_mode not in {
                "performance",
                "balanced",
                "quality",
                "xr",
            }:
                raise DreamerAgentRuntimeError(
                    "launcher_settings.rendering_mode is unsupported"
                )
            if "rendering" in raw:
                raise DreamerAgentRuntimeError(
                    "launcher_settings cannot mix rendering and rendering_mode"
                )
            normalized["rendering"] = True
            normalized["rendering_mode"] = rendering_mode

    experience = raw.get("experience")
    if experience is not None:
        if type(experience) is not str or not experience:
            raise DreamerAgentRuntimeError(
                "launcher_settings.experience must be a non-empty string"
            )
        if len(experience.encode("utf-8")) > 1024:
            raise DreamerAgentRuntimeError("launcher_settings.experience is too long")
        normalized["experience"] = experience

    kit_args = raw.get("kit_args", ())
    if type(kit_args) is str:
        try:
            kit_args = tuple(shlex.split(kit_args))
        except ValueError as exc:
            raise DreamerAgentRuntimeError(
                "launcher_settings.kit_args is not valid shell text"
            ) from exc
    elif type(kit_args) in (list, tuple):
        kit_args = tuple(kit_args)
    else:
        raise DreamerAgentRuntimeError(
            "launcher_settings.kit_args must be a string or JSON string list"
        )
    if len(kit_args) > 96 or any(
        type(item) is not str or len(item.encode("utf-8")) > 1024 for item in kit_args
    ):
        raise DreamerAgentRuntimeError(
            "launcher_settings.kit_args contains an invalid argument"
        )
    normalized["kit_args"] = kit_args

    visualizer = raw.get("visualizer")
    if visualizer is None:
        normalized["visualizer"] = False
    elif type(visualizer) is bool:
        normalized["visualizer"] = visualizer
    elif type(visualizer) in (list, tuple) and all(
        type(item) is str for item in visualizer
    ):
        normalized["visualizer"] = "kit" in visualizer and "none" not in visualizer
    else:
        raise DreamerAgentRuntimeError(
            "launcher_settings.visualizer must be a boolean or JSON string list"
        )
    return normalized


def _worker_hydra_overrides(parent: DreamerParentConfig) -> tuple[str, ...]:
    metadata = parent.metadata
    sources = metadata.get("sources")
    if not isinstance(sources, Mapping):
        raise DreamerAgentRuntimeError("parent configuration has no source metadata")
    values = sources.get("hydra_overrides")
    if not isinstance(values, list) or any(type(value) is not str for value in values):
        raise DreamerAgentRuntimeError(
            "parent configuration has invalid Hydra override metadata"
        )
    # Workers compose only env config; agent overrides already shaped the
    # parent learner config. Parent-derived seed and worker count stay fixed.
    return tuple(
        override
        for override in values
        if (
            (key := override.partition("=")[0].lstrip("+~")).startswith("env.")
            and key not in _WORKER_AUTHORITY_OVERRIDE_KEYS
        )
    )


def _worker_specs(
    parent: DreamerParentConfig,
    *,
    launcher_settings: Mapping[str, Any] | None,
    headless: bool,
) -> tuple[tuple[Any, ...], str]:
    normalized = _normalize_launcher_settings(launcher_settings, headless=headless)
    requested_device = normalized.get("device")
    if parent.sim_device is not None and requested_device is not None:
        if requested_device != parent.sim_device:
            raise DreamerAgentRuntimeError(
                "launcher device conflicts with canonical env.sim.device"
            )
    device = parent.sim_device or requested_device or "cpu"
    try:
        from .worker_bootstrap import DreamerWorkerLaunchSpec

        overrides = _worker_hydra_overrides(parent)
        specs = tuple(
            DreamerWorkerLaunchSpec(
                task_id=parent.task_id,
                hydra_overrides=overrides,
                device=device,
                headless=headless,
                enable_cameras=bool(normalized.get("enable_cameras", False)),
                rendering=bool(normalized.get("rendering", False)),
                rendering_mode=normalized.get("rendering_mode"),
                experience=normalized.get("experience"),
                kit_args=normalized["kit_args"],
                xr=bool(normalized.get("xr", False)),
                visualizer=bool(normalized.get("visualizer", False)),
                video=bool(normalized.get("video", False)),
                observation_mode=parent.obs,
                base_seed=parent.seed,
                worker_index=index,
            )
            for index in range(parent.worker_count)
        )
    except DreamerAgentRuntimeError:
        raise
    except Exception as exc:
        raise DreamerAgentRuntimeError(
            "canonical Dreamer worker configuration is unsupported"
        ) from exc
    if len(specs) != parent.worker_count:
        raise DreamerAgentRuntimeError("Dreamer worker spec count is not canonical")
    return specs, device


def _normalize_forwarded_args(forwarded_args: Sequence[str]) -> tuple[str, ...]:
    if isinstance(forwarded_args, (str, bytes)):
        raise DreamerAgentRuntimeError("forwarded_args must be a sequence")
    try:
        values = tuple(forwarded_args)
    except TypeError as exc:
        raise DreamerAgentRuntimeError("forwarded_args must be a sequence") from exc
    if any(
        type(value) is not str or len(value.encode("utf-8")) > _MAX_FORWARD_ARG_BYTES
        for value in values
    ):
        raise DreamerAgentRuntimeError("forwarded_args must contain bounded strings")
    return values


def _learner_options(
    run_options: Mapping[str, Any] | None,
    *,
    parent: DreamerParentConfig,
    device: str,
    forwarded_args: tuple[str, ...],
) -> dict[str, Any]:
    if run_options is None:
        options: dict[str, Any] = {}
    elif isinstance(run_options, Mapping):
        options = dict(run_options)
    else:
        raise DreamerAgentRuntimeError("run_options must be a mapping")
    if any(type(key) is not str for key in options):
        raise DreamerAgentRuntimeError("run_options keys must be strings")
    reserved = sorted(set(options) & _RUN_OPTION_AUTHORITY_KEYS)
    if reserved:
        raise DreamerAgentRuntimeError(
            f"run_options cannot override canonical fields: {', '.join(reserved)}"
        )

    metadata_value = options.get("run_metadata")
    if metadata_value is None:
        metadata: dict[str, Any] = {}
    elif isinstance(metadata_value, Mapping):
        metadata = dict(metadata_value)
    else:
        raise DreamerAgentRuntimeError("run_options.run_metadata must be a mapping")
    process_metadata = metadata.get("dreamer_process")
    if process_metadata is None:
        process_metadata = {}
    elif isinstance(process_metadata, Mapping):
        process_metadata = dict(process_metadata)
    else:
        raise DreamerAgentRuntimeError("run_metadata.dreamer_process must be a mapping")
    process_metadata.update(
        {
            "config_fingerprint": parent.fingerprint,
            "task_id": parent.task_id,
            "obs": parent.obs,
            "base_seed": parent.seed,
            "worker_count": parent.worker_count,
            "sim_device": device,
            "hydra_overrides": list(_worker_hydra_overrides(parent)),
            "effective_seed": parent.seed,
            "effective_config": parent.metadata.get("effective"),
        }
    )
    metadata["dreamer_process"] = process_metadata
    options["run_metadata"] = metadata
    options["config_fingerprint"] = parent.fingerprint
    options["effective_config"] = parent.metadata.get("effective")
    options["sim_device"] = device
    options["forwarded_args"] = forwarded_args
    return options


def _close_worker_owners(owners: Sequence[Any]) -> list[tuple[str, BaseException]]:
    from srb.integrations.dreamer.cleanup import close_method_step, run_cleanup_steps

    return run_cleanup_steps(
        close_method_step(f"worker[{index}].close", owner)
        for index, owner in reversed(tuple(enumerate(owners)))
    )


def _validate_process_schema(schema: Any) -> None:
    if schema is None:
        raise DreamerAgentRuntimeError("Dreamer worker negotiation returned no schema")
    for name in ("action_projection", "observation_projection"):
        projection = getattr(schema, name, None)
        metadata = getattr(projection, "metadata", None)
        if not isinstance(metadata, Mapping) or metadata.get("version") != 2:
            raise DreamerAgentRuntimeError(
                f"Dreamer {name} did not negotiate projection metadata v2"
            )


def _bind_transport_worker_count(transport: Any, worker_count: int) -> None:
    current = getattr(transport, "num_envs", None)
    if type(current) is not int or current != worker_count:
        raise DreamerAgentRuntimeError(
            "Dreamer transport worker count disagrees with parent configuration"
        )


def _cleanup_process_runtime(
    process_env: Any | None,
    transport: Any | None,
    owners: Sequence[Any],
) -> list[tuple[str, BaseException]]:
    from srb.integrations.dreamer.cleanup import close_method_step, run_cleanup_steps

    # One ordered owner list: environment facade, transport, then workers in
    # reverse launch order. Every step runs even when an earlier one fails.
    steps = [
        close_method_step(name, owner)
        for name, owner in (("process_env", process_env), ("transport", transport))
        if owner is not None and callable(getattr(owner, "close", None))
    ]
    errors = run_cleanup_steps(steps)
    errors.extend(_close_worker_owners(owners))
    return errors


def run_dreamer_process_training(
    *,
    task_id: str,
    obs: str,
    policy_config_path: str | os.PathLike[str],
    custom_config_path: str | os.PathLike[str] | None = None,
    hydra_overrides: Sequence[str] = (),
    launcher_settings: Mapping[str, Any] | None = None,
    headless: bool = True,
    logdir: str | os.PathLike[str],
    model: str | os.PathLike[str] | None = None,
    continue_training: bool | None = None,
    run_metadata: Mapping[str, Any] | None = None,
    run_options: Mapping[str, Any] | None = None,
    forwarded_args: Sequence[str] = (),
    worker_factory: Any | None = None,
    transport_factory: Callable[[Sequence[Any]], Any] | None = None,
    process_env_factory: Callable[[Any], Any] | None = None,
    learner_runner: Callable[..., Any] | None = None,
    transport_timeout_s: float = 5.0,
    # Cold Isaac Sim worker startup can take several minutes
    startup_timeout_s: float = 600.0,
) -> Any:
    """Run Dreamer training with Isaac owned exclusively by child workers.

    Every value that can influence a worker is normalized before the first
    child starts. The learner is imported only after all workers have launched
    and the process protocol has negotiated one common projection schema.
    """
    parent = load_dreamer_parent_config(
        task_id=task_id,
        obs=obs,
        policy_config_path=policy_config_path,
        hydra_config_path=custom_config_path,
        hydra_overrides=hydra_overrides,
    )
    specs, device = _worker_specs(
        parent,
        launcher_settings=launcher_settings,
        headless=headless,
    )
    forwarded = _normalize_forwarded_args(forwarded_args)
    learner_run_options: Mapping[str, Any] | None = run_options
    if run_metadata is not None:
        if not isinstance(run_metadata, Mapping):
            raise DreamerAgentRuntimeError("run_metadata must be a mapping")
        if run_options is not None and not isinstance(run_options, Mapping):
            raise DreamerAgentRuntimeError("run_options must be a mapping")
        merged_options = dict(run_options) if run_options is not None else {}
        existing_metadata = merged_options.get("run_metadata")
        if existing_metadata is None:
            merged_metadata = dict(run_metadata)
        elif isinstance(existing_metadata, Mapping):
            merged_metadata = dict(run_metadata)
            for key, value in existing_metadata.items():
                if isinstance(merged_metadata.get(key), Mapping) and isinstance(
                    value, Mapping
                ):
                    nested = dict(merged_metadata[key])
                    nested.update(value)
                    merged_metadata[key] = nested
                else:
                    merged_metadata[key] = value
        else:
            raise DreamerAgentRuntimeError("run_options.run_metadata must be a mapping")
        merged_options["run_metadata"] = merged_metadata
        learner_run_options = merged_options
    learner_options = _learner_options(
        learner_run_options,
        parent=parent,
        device=device,
        forwarded_args=forwarded,
    )
    logdir_path = Path(logdir).expanduser().resolve()
    model_path = None if model is None else Path(model).expanduser().resolve()

    owners: list[Any] = []
    transport: Any | None = None
    process_env: Any | None = None
    primary: BaseException | None = None
    result: Any = None
    try:
        if worker_factory is None:
            from .process_transport import DreamerPopenWorkerFactory

            worker_factory = DreamerPopenWorkerFactory()
        launch = getattr(worker_factory, "launch", None)
        if not callable(launch) and callable(worker_factory):
            launch = worker_factory
        if not callable(launch):
            raise DreamerAgentRuntimeError("worker_factory must expose launch(spec)")
        for spec in specs:
            owners.append(launch(spec))

        if transport_factory is None:
            from .process_transport import DreamerProcessTransport

            transport = DreamerProcessTransport(
                tuple(owners),
                timeout_s=transport_timeout_s,
                startup_timeout_s=startup_timeout_s,
            )
        else:
            transport = transport_factory(tuple(owners))
        _bind_transport_worker_count(transport, parent.worker_count)
        handshake = getattr(transport, "handshake", None)
        if not callable(handshake):
            raise DreamerAgentRuntimeError(
                "Dreamer transport does not expose handshake()"
            )
        _validate_process_schema(handshake())

        if process_env_factory is None:
            from .process_env import DreamerProcessEnv

            process_env = DreamerProcessEnv(transport)
        else:
            process_env = process_env_factory(transport)

        if learner_runner is None:
            # This is intentionally after worker launch and protocol
            # negotiation. main.py is parent-safe, but it is the gateway to
            # the first Dreamer/JAX learner import.
            from .main import run_process

            learner_runner = run_process
        result = learner_runner(
            env=process_env,
            agent_cfg=parent.agent_cfg,
            task_id=parent.task_id,
            base_seed=parent.seed,
            env_count=parent.worker_count,
            logdir=logdir_path,
            model=model_path,
            continue_training=continue_training,
            workflow="train",
            **learner_options,
        )
    except BaseException as exc:
        primary = exc

    cleanup_errors = _cleanup_process_runtime(process_env, transport, owners)
    if primary is not None:
        if cleanup_errors and hasattr(primary, "add_note"):
            primary.add_note(
                "Dreamer process cleanup diagnostics: "
                f"{_cleanup_diagnostic(cleanup_errors)}"
            )
        raise primary
    if cleanup_errors:
        detail = _cleanup_diagnostic(cleanup_errors)
        raise DreamerAgentRuntimeError(
            f"Dreamer process cleanup failed: {detail}"
        ) from cleanup_errors[0][1]
    return result


__all__ = [
    "DreamerAgentRuntimeError",
    "DreamerParentConfig",
    "SrbDreamerWorkerEnvironment",
    "build_srb_worker_environment",
    "load_dreamer_parent_config",
    "run_dreamer_process_training",
]
