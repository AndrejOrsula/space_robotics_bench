"""SB3 state spaces and formatters for config string fields.

``policy_kwargs`` uses a ``dict(...)`` string and ``learning_rate`` uses
``lin_<float>``; sampled values must retain those formats.
"""

from __future__ import annotations

import ast

_ARCH = {
    "256x256": [256, 256],
    "384x384": [384, 384],
    "512x512": [512, 512],
}


def synth_policy_kwargs(net_arch: list[int], log_std_init) -> str:
    """Format a ``dict(...)`` string for sb3 ``policy_kwargs`` (deterministic)."""
    return f"dict(log_std_init={log_std_init}, net_arch={list(net_arch)})"


def synth_policy_kwargs_net_only(net_arch: list[int]) -> str:
    """Format ``policy_kwargs`` with only ``net_arch`` for deterministic policies."""
    return f"dict(net_arch={list(net_arch)})"


def base_policy_kwarg(base_agent_cfg: dict | None, key: str):
    """Read a literal keyword from base ``policy_kwargs``, or return ``None``."""
    text = (base_agent_cfg or {}).get("policy_kwargs")
    if not isinstance(text, str):
        return None
    try:
        call = ast.parse(text.strip(), mode="eval").body
    except SyntaxError:
        return None
    if not isinstance(call, ast.Call):
        return None
    for keyword in call.keywords:
        if keyword.arg == key:
            try:
                return ast.literal_eval(keyword.value)
            except (TypeError, ValueError):
                return None
    return None


def synth_lr_schedule(lr: float) -> str:
    """Format a linear-schedule string ``lin_<lr>`` for sb3 ``learning_rate``."""
    return f"lin_{lr}"


def sb3_ppo_space(trial, ctx) -> dict:
    lr = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
    net_arch = _ARCH[trial.suggest_categorical("net_arch", list(_ARCH))]
    log_std_init = round(trial.suggest_float("log_std_init", -2.5, -0.5), 4)
    return {
        "learning_rate": synth_lr_schedule(lr),
        "batch_size": trial.suggest_categorical("batch_size", [512, 1024, 2048]),
        "n_steps": trial.suggest_categorical("n_steps", [64, 128, 256]),
        "n_epochs": trial.suggest_int("n_epochs", 4, 16),
        "gamma": trial.suggest_float("gamma", 0.99, 0.999),
        "gae_lambda": trial.suggest_float("gae_lambda", 0.9, 0.99),
        "ent_coef": trial.suggest_float("ent_coef", 1e-4, 3e-2, log=True),
        "clip_range": trial.suggest_float("clip_range", 0.1, 0.3),
        "max_grad_norm": trial.suggest_categorical("max_grad_norm", [0.5, 1.0]),
        "policy_kwargs": synth_policy_kwargs(net_arch, log_std_init),
    }


def sb3_sac_space(trial, ctx) -> dict:
    base = ctx.base_agent_cfg or {}
    lr = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
    net_arch = _ARCH[trial.suggest_categorical("net_arch", list(_ARCH))]
    # SAC/TQC reads ``log_std_init`` only with gSDE; otherwise retain the
    # profile value instead of searching an inert parameter.
    if base.get("use_sde"):
        log_std_init = round(trial.suggest_float("log_std_init", -2.5, -0.5), 4)
    else:
        log_std_init = base_policy_kwarg(base, "log_std_init")
    if log_std_init is None:
        policy_kwargs = synth_policy_kwargs_net_only(net_arch)
    else:
        policy_kwargs = synth_policy_kwargs(net_arch, log_std_init)
    return {
        "learning_rate": synth_lr_schedule(lr),
        "batch_size": trial.suggest_categorical("batch_size", [256, 512, 1024]),
        "gamma": trial.suggest_float("gamma", 0.99, 0.999),
        "tau": trial.suggest_float("tau", 1e-3, 2e-2, log=True),
        "gradient_steps": trial.suggest_categorical("gradient_steps", [1, 2, 4]),
        "learning_starts": trial.suggest_categorical(
            "learning_starts", [10000, 50000, 100000]
        ),
        "train_freq": trial.suggest_categorical("train_freq", [1, 2, 4, 8]),
        # ent_coef stays "auto" (SAC auto-entropy) — deliberately not tuned.
        "policy_kwargs": policy_kwargs,
    }


def sb3_td3_space(trial, ctx) -> dict:
    # TD3 is deterministic: tune exploration noise, with no entropy/log_std.
    lr = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
    net_arch = _ARCH[trial.suggest_categorical("net_arch", list(_ARCH))]
    return {
        "learning_rate": synth_lr_schedule(lr),
        "batch_size": trial.suggest_categorical("batch_size", [256, 512, 1024]),
        "gamma": trial.suggest_float("gamma", 0.99, 0.999),
        "tau": trial.suggest_float("tau", 1e-3, 2e-2, log=True),
        "gradient_steps": trial.suggest_categorical("gradient_steps", [1, 2, 4]),
        "learning_starts": trial.suggest_categorical(
            "learning_starts", [10000, 50000, 100000]
        ),
        "train_freq": trial.suggest_categorical("train_freq", [1, 2, 4, 8]),
        "noise_std": trial.suggest_float("noise_std", 0.05, 0.3),
        "policy_kwargs": synth_policy_kwargs_net_only(net_arch),
    }


def sb3_crossq_space(trial, ctx) -> dict:
    # CrossQ drops target networks, ``gradient_steps``, and ``tau`` from the
    # base config, leaving a smaller search space than SAC.
    lr = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
    net_arch = _ARCH[trial.suggest_categorical("net_arch", list(_ARCH))]
    return {
        "learning_rate": synth_lr_schedule(lr),
        "gamma": trial.suggest_float("gamma", 0.99, 0.999),
        "learning_starts": trial.suggest_categorical(
            "learning_starts", [10000, 50000, 100000]
        ),
        "policy_delay": trial.suggest_categorical("policy_delay", [1, 2, 3]),
        "policy_kwargs": synth_policy_kwargs_net_only(net_arch),
    }


def _register() -> None:
    from srb.tuning.spaces import register_space

    register_space("sb3", "ppo", "state", sb3_ppo_space)
    register_space("sb3", "sac", "state", sb3_sac_space)
    # TQC shares SAC's config keys, so reuse its space.
    register_space("sb3", "tqc", "state", sb3_sac_space)
    register_space("sb3", "td3", "state", sb3_td3_space)
    register_space("sb3", "crossq", "state", sb3_crossq_space)


_register()
