"""Typed, dependency-free trial planning for tuning studies."""

from __future__ import annotations

import copy
import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import date, datetime
from pathlib import Path
from types import MappingProxyType
from typing import Any, cast

import yaml

from srb.tuning.merge import deep_merge
from srb.tuning.spaces import (
    TuneContext,
    budget_overrides,
    get_space,
    wandb_off_overrides,
)
from srb.utils.json_types import ImmutableValueMixin
from srb.utils.path import SRB_HYPERPARAMS_DIR, SRB_LOGS_DIR

# Default vector size; the driver pins it so budget conversion matches the child.
_DEFAULT_TRIAL_NUM_ENVS = 1024


class PinCollisionError(ValueError):
    """A pin collides with a driver-owned or sampled key."""


@dataclass
class TuneConfig:
    tasks: list[str]
    algo: str
    obs: str = "state"
    trial_env_steps: int = 10_000_000
    n_trials: int = 20
    n_workers: int = 1
    gpus: list[str] | None = None
    seed_base: int = 42
    # Run K distinct seeds and report their mean; prune on replicate 0.
    seeds_per_trial: int = 1
    num_envs: int | None = None
    pins: list[str] = field(default_factory=list)
    max_wall_hours: float | None = None
    timeout_s: float | None = None
    objective: str = "ep_return_mean"
    n_startup: int | None = None
    version: int = 1
    # None means automatic: 30s with multiple workers and 0 otherwise.
    boot_stagger_s: float | None = None
    # Keep pruned trial child logdirs (checkpoints, TB events). Off by default.
    keep_artifacts: bool = False
    # Re-enqueue up to N failed parameter sets at startup.
    retry_failed: int = 0
    # Optional owned PostgreSQL RDB storage DSN. None keeps journal default.
    storage: str | None = None
    # Portfolio scoring; single-task studies retain these for resume checks.
    normalize: str = "none"
    aggregate: str = "median"
    fps_vec_n: float | None = None
    hyperparams_root: Path = SRB_HYPERPARAMS_DIR
    logdir_root: Path = SRB_LOGS_DIR / "tune"
    poll_dt: float = 10.0
    grace_s: float = 60.0


def _config_invariant_errors(cfg: TuneConfig) -> dict[str, str]:
    """Return every dependency-free configuration error in run order."""
    errors: dict[str, str] = {}

    def option_name(name: str) -> str:
        return {
            "trial_env_steps": "trial-steps",
            "num_envs": "trial-num-envs",
            "timeout_s": "trial-timeout",
            "boot_stagger_s": "boot-stagger",
            "fps_vec_n": "fps",
        }.get(name, name.replace("_", "-"))

    if not cfg.tasks:
        errors["tasks"] = "srb agent tune: at least one task is required"
    elif len(set(cfg.tasks)) != len(cfg.tasks):
        errors["tasks"] = (
            "srb agent tune: --tasks must be unique (seg-<task> dirs collide)"
        )
    if cfg.normalize not in ("none", "baseline", "rank"):
        errors["normalize"] = (
            f"--normalize must be none|baseline|rank (got {cfg.normalize!r})"
        )
    if cfg.aggregate not in ("median", "mean", "min"):
        errors["aggregate"] = (
            f"--aggregate must be median|mean|min (got {cfg.aggregate!r})"
        )

    def exact_int(name: str, value: object, *, minimum: int) -> None:
        option = option_name(name)
        if type(value) is not int:
            errors[option] = f"--{option} must be an exact int (got {value!r})"
        elif value < minimum:
            errors[option] = f"--{option} must be >= {minimum} (got {value})"

    def optional_exact_int(name: str, value: object, *, minimum: int) -> None:
        if value is not None:
            exact_int(name, value, minimum=minimum)

    def finite_number(name: str, value: object, *, minimum: float) -> None:
        if value is None:
            return
        option = option_name(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            errors[option] = f"--{option} must be a finite number (got {value!r})"
            return
        try:
            finite = math.isfinite(float(value))
        except (OverflowError, ValueError):
            finite = False
        if not finite:
            errors[option] = f"--{option} must be a finite number (got {value!r})"
        elif value < minimum:
            errors[option] = f"--{option} must be >= {minimum:g} (got {value})"

    exact_int("trial_env_steps", cfg.trial_env_steps, minimum=1)
    if cfg.num_envs is not None:
        exact_int("num_envs", cfg.num_envs, minimum=1)
    exact_int("seed_base", cfg.seed_base, minimum=0)
    exact_int("seeds_per_trial", cfg.seeds_per_trial, minimum=1)
    exact_int("n_trials", cfg.n_trials, minimum=1)
    exact_int("n_workers", cfg.n_workers, minimum=1)
    optional_exact_int("n_startup", cfg.n_startup, minimum=0)
    exact_int("retry_failed", cfg.retry_failed, minimum=0)
    exact_int("version", cfg.version, minimum=1)

    # Validate timing inputs before consumers can interpret them as hangs,
    # disabled guards, or NaNs.
    finite_number("max_wall_hours", cfg.max_wall_hours, minimum=0.0)
    finite_number("timeout_s", cfg.timeout_s, minimum=0.0)
    finite_number("boot_stagger_s", cfg.boot_stagger_s, minimum=0.0)
    finite_number("fps_vec_n", cfg.fps_vec_n, minimum=0.0)
    finite_number("poll_dt", cfg.poll_dt, minimum=0.0)
    finite_number("grace_s", cfg.grace_s, minimum=0.0)

    # Require positive durations and rates; grace periods may be zero.
    for name in ("max_wall_hours", "timeout_s", "fps_vec_n", "poll_dt"):
        value = getattr(cfg, name)
        option = option_name(name)
        if option not in errors and value is not None and value <= 0:
            errors[option] = f"--{option} must be > 0 (got {value})"
    return errors


def _coerce(value: str) -> None | bool | int | float | str:
    """Coerce a CLI pin value to a scalar without YAML 1.1 surprises."""
    low = value.strip().lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("null", "none", "~"):
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _effective_num_envs(value: int | None) -> int:
    """Use the default only when no value was supplied."""
    return _DEFAULT_TRIAL_NUM_ENVS if value is None else value


def _validated_recorded_num_envs(env_section: Mapping[object, object]) -> int:
    """Read a recorded environment count without coercion."""
    if "num_envs" not in env_section:
        return _DEFAULT_TRIAL_NUM_ENVS
    value = env_section["num_envs"]
    if type(value) is not int or value < 1:
        raise ValueError(
            f"recorded env.num_envs must be an exact positive int (got {value!r})"
        )
    return value


def _leaf_paths(d: Mapping[object, object], prefix: str = "") -> set[str]:
    out: set[str] = set()
    for k, v in d.items():
        key = str(k)
        p = f"{prefix}.{key}" if prefix else key
        if isinstance(v, Mapping):
            out |= _leaf_paths(v, p)
        else:
            out.add(p)
    return out


def _set_path(root: dict[str, Any], dotted: str, value: Any) -> None:
    """Set a pin below its agent or environment section."""
    section, _, rest = dotted.partition(".")
    node = root.setdefault(section, {})
    if isinstance(node, dict) and rest in node:
        node[rest] = value
        return
    parts = rest.split(".")
    for p in parts[:-1]:
        nxt = node.get(p)
        if not isinstance(nxt, dict):
            nxt = {}
            node[p] = nxt
        node = nxt
    node[parts[-1]] = value


def _paths_overlap(a: str, b: str) -> bool:
    """Return whether dotted paths are equal or ancestor/descendant paths."""
    return a == b or a.startswith(b + ".") or b.startswith(a + ".")


def validate_pins(pins: list[str], *, reserved: set[str]) -> None:
    """Validate pin syntax and reserved-key overlap."""
    for pin in pins:
        if not isinstance(pin, str):
            raise PinCollisionError(f"pin must be a string: {pin!r}")
        key, sep, value = pin.partition("=")
        if not sep:
            raise PinCollisionError(f"pin must be key=value: {pin!r}")
        coerced = _coerce(value)
        if isinstance(coerced, float) and not math.isfinite(coerced):
            raise PinCollisionError(f"pin value must be finite: {key!r}={value!r}")
        if not key.startswith(("agent.", "env.")):
            raise PinCollisionError(
                f"pin key must start with 'agent.' or 'env.': {key!r}"
            )
        if any(not part for part in key.split(".")):
            raise PinCollisionError(f"pin key has an empty path component: {key!r}")
        clash = next((r for r in reserved if _paths_overlap(key, r)), None)
        if clash is not None:
            raise PinCollisionError(
                f"pin {key!r} overlaps reserved (driver-owned) key {clash!r}"
            )


def apply_pins(
    wrapped: dict[str, Any],
    pins: list[str],
    *,
    reserved: set[str],
    sampled: set[str],
) -> dict[str, Any]:
    """Apply pins, rejecting collisions with driver-owned or sampled keys."""
    validate_pins(pins, reserved=reserved)
    for pin in pins:
        key, _sep, value = pin.partition("=")
        clash = next((s for s in sampled if _paths_overlap(key, s)), None)
        if clash is not None:
            raise PinCollisionError(
                f"pin {key!r} overlaps search-space-sampled key {clash!r}"
            )
        _set_path(wrapped, key, _coerce(value))
    return wrapped


def effective_boot_stagger(boot_stagger_s: float | None, n_workers: int) -> float:
    """Resolve explicit boot stagger or its worker-count default."""
    if boot_stagger_s is not None:
        return boot_stagger_s
    return 30.0 if n_workers > 1 else 0.0


def project_wall_clock(
    *,
    trial_env_steps: int,
    fps_vec_n: float | None,
    n_trials: int,
    n_workers: int,
    seeds_per_trial: int = 1,
    n_tasks: int = 1,
) -> dict[str, float] | None:
    """Project per-trial and total wall clock, or None without throughput."""
    if not fps_vec_n:
        return None
    hours_per_trial = trial_env_steps / fps_vec_n / 3600.0 * max(n_tasks, 1)
    study_hours = (
        n_trials * hours_per_trial * max(seeds_per_trial, 1) / max(n_workers, 1)
    )
    return {"hours_per_trial": hours_per_trial, "study_hours": study_hours}


type TrialScalar = None | bool | int | float | str | bytes | date | datetime
type TrialKey = TrialScalar
type TrialValue = (
    TrialScalar
    | tuple[TrialValue, ...]
    | frozenset[TrialValue]
    | Mapping[TrialKey, TrialValue]
)
type TrialSection = Mapping[str, TrialValue]


def _normalize_yaml_scalars(value: object, active: set[int]) -> object:
    """Copy containers and normalize scalar subclasses for safe YAML."""
    value_type = type(value)
    if value_type in (type(None), bool, int, str, bytes, date, datetime):
        return value
    if value_type is float:
        if not math.isfinite(cast(float, value)):
            raise ValueError("trial plan values must be finite")
        return value
    if isinstance(value, str):
        return str(value)
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        normalized = float(value)
        if not math.isfinite(normalized):
            raise ValueError("trial plan values must be finite")
        return normalized
    if isinstance(value, bytes):
        return bytes(value)
    if isinstance(value, datetime):
        return datetime(
            value.year,
            value.month,
            value.day,
            value.hour,
            value.minute,
            value.second,
            value.microsecond,
            tzinfo=value.tzinfo,
            fold=value.fold,
        )
    if isinstance(value, date):
        return date(value.year, value.month, value.day)

    if not isinstance(value, (Mapping, list, tuple, set)):
        return value
    identity = id(value)
    if identity in active:
        raise ValueError("trial plan values must not contain cycles")
    active.add(identity)
    try:
        if isinstance(value, Mapping):
            return {
                _normalize_yaml_scalars(key, active): _normalize_yaml_scalars(
                    item, active
                )
                for key, item in value.items()
            }
        if isinstance(value, list):
            return [_normalize_yaml_scalars(item, active) for item in value]
        if isinstance(value, tuple):
            return tuple(_normalize_yaml_scalars(item, active) for item in value)
        return {_normalize_yaml_scalars(item, active) for item in value}
    finally:
        active.remove(identity)


def _canonicalize_trial_section(
    section: Mapping[Any, Any], *, name: str
) -> Mapping[Any, Any]:
    """Round-trip a section through execution's safe YAML representation."""
    try:
        normalized = _normalize_yaml_scalars(section, set())
        encoded = yaml.safe_dump(normalized, sort_keys=False)
        canonical = yaml.safe_load(encoded)
    except (TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValueError(f"{name} is not safe-YAML materializable") from exc
    if not isinstance(canonical, Mapping):
        raise ValueError(f"{name} must be a mapping")
    if any(type(key) is not str for key in canonical):
        raise ValueError(f"{name} top-level keys must be strings")
    return canonical


def _freeze_trial_value(value: object) -> TrialValue:
    if isinstance(value, Mapping):
        frozen: dict[TrialKey, TrialValue] = {}
        for key, item in value.items():
            frozen[cast(TrialKey, key)] = _freeze_trial_value(item)
        return MappingProxyType(frozen)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_trial_value(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(_freeze_trial_value(item) for item in value)
    if type(value) in (type(None), bool, int, float, str, bytes, date, datetime):
        return cast(TrialScalar, value)
    raise ValueError(f"unsupported canonical trial plan value: {type(value).__name__}")


def _freeze_trial_section(section: Mapping[Any, Any], *, name: str) -> TrialSection:
    canonical = _canonicalize_trial_section(section, name=name)
    return MappingProxyType(
        {cast(str, key): _freeze_trial_value(value) for key, value in canonical.items()}
    )


def _thaw_trial_value(value: TrialValue) -> Any:
    if isinstance(value, Mapping):
        return {key: _thaw_trial_value(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw_trial_value(item) for item in value]
    if isinstance(value, frozenset):
        return {_thaw_trial_value(item) for item in value}
    return value


def _thaw_trial_section(section: TrialSection) -> dict[str, Any]:
    return {key: _thaw_trial_value(value) for key, value in section.items()}


def _require_exact_trial_seed(seed: object) -> int:
    if type(seed) is not int:
        raise TypeError("trial seed override must be an exact int")
    return seed


@dataclass(frozen=True, slots=True, init=False)
class TrialPlan(ImmutableValueMixin):
    """Immutable sampled trial configuration shared by run and preflight."""

    agent_section: TrialSection
    env_section: TrialSection

    def __init__(
        self,
        agent_section: Mapping[str, object],
        env_section: Mapping[str, object],
    ) -> None:
        object.__setattr__(
            self,
            "agent_section",
            _freeze_trial_section(agent_section, name="agent_section"),
        )
        object.__setattr__(
            self,
            "env_section",
            _freeze_trial_section(env_section, name="env_section"),
        )

    @property
    def num_envs(self) -> int:
        """Return the driver-owned vectorization value."""
        value = self.env_section["num_envs"]
        if type(value) is not int or value < 1:
            raise ValueError("trial plan env.num_envs must be an exact positive int")
        return value

    def materialize(self, *, seed: int | None = None) -> dict[str, Any]:
        """Return a fresh complete trial document, optionally reseeded."""
        document = _thaw_trial_section(self.agent_section)
        env_section = _thaw_trial_section(self.env_section)
        if seed is not None:
            env_section["seed"] = _require_exact_trial_seed(seed)
        document["env"] = env_section
        return document

    def validation_summary(self) -> str:
        """Summarize the plan without exposing its sections."""
        agent = self.agent_section["agent"]
        if not isinstance(agent, Mapping):
            raise TypeError("trial plan agent section must contain a mapping")
        leaves = _leaf_paths(cast(Mapping[object, object], agent))
        return f"{len(leaves)} agent leaves, num_envs={self.num_envs}"


def assemble_trial_doc(
    trial: Any,
    cfg: TuneConfig,
    *,
    framework: str,
    local_algo: str,
    base_agent_cfg: dict[str, Any],
    num_envs: int,
    reserved: set[str],
) -> TrialPlan:
    """Build the shared sample, merge, pin, budget, and canonicalization plan."""
    # Some spaces sample conditionally (SB3 SAC uses ``use_sde``), so inspect
    # pinned base values, then merge from the base and check pin collisions.
    space_view = apply_pins(
        {"agent": copy.deepcopy(base_agent_cfg)},
        cfg.pins,
        reserved=reserved,
        sampled=set(),
    )["agent"]
    ctx = TuneContext(base_agent_cfg=space_view)
    space_override = get_space(framework, local_algo, cfg.obs)(trial, ctx)
    merged = deep_merge(base_agent_cfg, space_override)
    env_section = {"seed": cfg.seed_base + trial.number, "num_envs": num_envs}
    wrapped = {"agent": merged, "env": env_section}
    sampled = {f"agent.{p}" for p in _leaf_paths(space_override)}
    wrapped = apply_pins(wrapped, cfg.pins, reserved=reserved, sampled=sampled)
    # Convert the post-pin rollout length while retaining the driver-owned budget.
    wrapped["agent"] = deep_merge(
        wrapped["agent"],
        budget_overrides(
            framework,
            trial_env_steps=cfg.trial_env_steps,
            num_envs=num_envs,
            agent_cfg=wrapped["agent"],
        ),
    )
    wrapped["agent"] = deep_merge(wrapped["agent"], wandb_off_overrides(framework))
    return TrialPlan(
        agent_section={"agent": wrapped["agent"]},
        env_section=wrapped["env"],
    )


__all__ = (
    "PinCollisionError",
    "TrialPlan",
    "TuneConfig",
    "apply_pins",
    "assemble_trial_doc",
    "effective_boot_stagger",
    "project_wall_clock",
    "validate_pins",
)
