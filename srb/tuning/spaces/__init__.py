"""Registry of Optuna spaces and framework-specific tuning overrides.

Each :data:`SpaceFn` maps a trial and :class:`TuneContext` to a nested config
override. Framework modules register spaces on import; this module also owns
budget conversion, reserved paths, export resets, and W&B-off overrides.
"""

from __future__ import annotations

import copy
import hashlib
import inspect
import math
import types
from dataclasses import dataclass, field
from typing import Callable


@dataclass(frozen=True)
class TuneContext:
    """Everything a space function may need beyond the trial itself."""

    base_agent_cfg: dict  # base config the space splices list elements out of


# A space returns a nested agent-cfg override dict.
SpaceFn = Callable[["object", TuneContext], dict]

_SPACES: dict[tuple[str, str, str], SpaceFn] = {}


def register_space(framework: str, algo: str, obs: str, fn: SpaceFn) -> None:
    _SPACES[(framework, algo, obs)] = fn


def get_space(framework: str, algo: str, obs: str) -> SpaceFn:
    """Return the registered space, or raise ``KeyError`` (unknown / not-yet-phased)."""
    try:
        return _SPACES[(framework, algo, obs)]
    except KeyError as e:
        raise KeyError(
            f"No search space registered for framework={framework!r} "
            f"algo={algo!r} obs={obs!r}. Registered: {sorted(_SPACES)}"
        ) from e


def has_space(framework: str, algo: str, obs: str) -> bool:
    return (framework, algo, obs) in _SPACES


def registered_space_ids() -> list[str]:
    """Sorted ``framework:algo:obs`` ids of every registered space (operator view)."""
    return sorted(space_id(fw, al, ob) for (fw, al, ob) in _SPACES)


def space_id(framework: str, algo: str, obs: str) -> str:
    return f"{framework}:{algo}:{obs}"


def space_hash(framework: str, algo: str, obs: str) -> str:
    """Hash space behavior and its framework budget shim for frozen-study resume.

    Directly referenced globals and recursive helper sources are included;
    unrelated globals and framework shims are excluded.
    """
    fn = get_space(framework, algo, obs)
    src = _space_definition_source(fn, set())
    src += _space_definition_source(framework_shim(framework).budget, set())
    return hashlib.sha256(src.encode("utf-8")).hexdigest()


def _space_definition_source(fn: Callable, seen: set[int]) -> str:
    """Serialize a space function and the module globals its code reads."""
    if id(fn) in seen:
        return ""
    seen.add(id(fn))

    parts = [inspect.getsource(fn)]
    namespace = fn.__globals__
    for name in sorted(fn.__code__.co_names):
        if name not in namespace:
            continue
        value = namespace[name]
        if inspect.isfunction(value):
            parts.append(f"{name}:\n{_space_definition_source(value, seen)}")
        elif not isinstance(value, types.ModuleType):
            parts.append(f"{name}={value!r}")
    return "\n".join(parts)


# Framework shims centralize budget, reserved pin paths, export resets,
# W&B overrides, and (for single-file frameworks) catalog basenames.

# rsl_rl's per-iteration rollout length when the config omits it.
_RSL_RL_DEFAULT_STEPS_PER_ENV = 24


def _skrl_budget(S: int, N: int, agent_cfg: dict) -> dict:
    # One skrl trainer timestep = one vec-step across N envs.
    return {"trainer": {"timesteps": math.ceil(S / N)}}


def _sb3_budget(S: int, N: int, agent_cfg: dict) -> dict:
    # sb3/sbx (rl_zoo3) count total env-steps natively; sbx yamls are
    # byte-compatible with sb3's, so both frameworks share this conversion.
    return {"n_timesteps": S}


def _rsl_rl_budget(S: int, N: int, agent_cfg: dict) -> dict:
    nspe = int(agent_cfg.get("num_steps_per_env") or _RSL_RL_DEFAULT_STEPS_PER_ENV)
    return {"max_iterations": max(1, math.ceil(S / (nspe * N)))}


def _dreamer_budget(S: int, N: int, agent_cfg: dict) -> dict:
    # Keep this nested so it replaces the base value without a competing dotted
    # key. With multiple envs, DreamerV3 and wrapper step counts may differ by a
    # constant; see the docs for calibration guidance.
    return {"run": {"steps": S}}


def _tdmpc2_budget(S: int, N: int, agent_cfg: dict) -> dict:
    # TD-MPC2's online trainer treats ``steps`` as total environment steps.
    # Same num_envs calibration caveat as dreamer applies.
    return {"steps": S}


@dataclass(frozen=True)
class FrameworkShim:
    """Driver-owned per-framework knowledge (see block comment above)."""

    budget: Callable[[int, int, dict], dict]
    reserved_pin_paths: tuple[str, ...]
    export_reset_paths: tuple[tuple[str, ...], ...]
    wandb_off: dict = field(default_factory=dict)
    # Catalog file stem for single-file frameworks; None for per-algo dirs.
    l0_basename: str | None = None


_FRAMEWORK_SHIMS: dict[str, FrameworkShim] = {
    "skrl": FrameworkShim(
        budget=_skrl_budget,
        reserved_pin_paths=("agent.trainer.timesteps", "agent.agent.experiment.wandb"),
        export_reset_paths=(("trainer", "timesteps"), ("agent", "experiment", "wandb")),
        wandb_off={"agent": {"experiment": {"wandb": False}}},
    ),
    "sb3": FrameworkShim(
        budget=_sb3_budget,
        reserved_pin_paths=("agent.n_timesteps",),
        export_reset_paths=(("n_timesteps",),),
    ),
    "sbx": FrameworkShim(
        budget=_sb3_budget,
        reserved_pin_paths=("agent.n_timesteps",),
        export_reset_paths=(("n_timesteps",),),
    ),
    "rsl_rl": FrameworkShim(
        budget=_rsl_rl_budget,
        reserved_pin_paths=("agent.max_iterations",),
        export_reset_paths=(("max_iterations",),),
    ),
    "dreamer": FrameworkShim(
        budget=_dreamer_budget,
        reserved_pin_paths=("agent.run.steps",),
        export_reset_paths=(("run", "steps"),),
        l0_basename="dreamerv3",
    ),
    "tdmpc2": FrameworkShim(
        budget=_tdmpc2_budget,
        reserved_pin_paths=("agent.steps",),
        export_reset_paths=(("steps",),),
        l0_basename="tdmpc2",
    ),
}


def framework_shim(framework: str) -> FrameworkShim:
    """Return the framework's shim record, or raise ``ValueError``."""
    try:
        return _FRAMEWORK_SHIMS[framework]
    except KeyError:
        raise ValueError(
            f"No framework shim for {framework!r}. Known: {sorted(_FRAMEWORK_SHIMS)}"
        ) from None


def budget_overrides(
    framework: str, *, trial_env_steps: int, num_envs: int, agent_cfg: dict
) -> dict:
    """Convert env steps using merged config for rsl_rl; rung budgets use raw steps."""
    S = int(trial_env_steps)
    N = max(int(num_envs), 1)
    return framework_shim(framework).budget(S, N, agent_cfg)


def wandb_off_overrides(framework: str) -> dict:
    """Overrides that force W&B off for a trial child (only skrl defaults on)."""
    shim = _FRAMEWORK_SHIMS.get(framework)
    return copy.deepcopy(shim.wandb_off) if shim else {}


def reserved_pin_paths(framework: str) -> tuple[str, ...]:
    """Return driver-owned trial paths that reject ``--pin`` overrides."""
    return framework_shim(framework).reserved_pin_paths


def export_reset_paths(framework: str) -> tuple[tuple[str, ...], ...]:
    """Return framework-relative paths reset to production values on export."""
    return framework_shim(framework).export_reset_paths


# Register the framework spaces on import.
from srb.tuning.spaces import dreamer as _dreamer  # noqa: E402,F401
from srb.tuning.spaces import rsl_rl as _rsl_rl  # noqa: E402,F401
from srb.tuning.spaces import sb3 as _sb3  # noqa: E402,F401
from srb.tuning.spaces import sbx as _sbx  # noqa: E402,F401
from srb.tuning.spaces import skrl as _skrl  # noqa: E402,F401
from srb.tuning.spaces import tdmpc2 as _tdmpc2  # noqa: E402,F401

__all__ = (
    "TuneContext",
    "SpaceFn",
    "FrameworkShim",
    "register_space",
    "get_space",
    "has_space",
    "registered_space_ids",
    "space_id",
    "space_hash",
    "framework_shim",
    "budget_overrides",
    "wandb_off_overrides",
    "reserved_pin_paths",
    "export_reset_paths",
)
