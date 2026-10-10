"""rsl_rl PPO search space for state observations.

Tunes algorithm and network fields. ``num_steps_per_env`` stays fixed because
the budget shim uses it to derive ``max_iterations``.
"""

from __future__ import annotations

_HIDDEN = {
    "256x256": [256, 256],
    "384x384": [384, 384],
    "512x512": [512, 512],
    "512x256": [512, 256],
}


def rsl_rl_ppo_space(trial, ctx) -> dict:
    hidden = _HIDDEN[trial.suggest_categorical("hidden_dims", list(_HIDDEN))]
    activation = trial.suggest_categorical("activation", ["elu", "relu", "tanh"])
    return {
        "algorithm": {
            "learning_rate": trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True),
            "num_learning_epochs": trial.suggest_int("num_learning_epochs", 3, 8),
            "num_mini_batches": trial.suggest_categorical(
                "num_mini_batches", [2, 4, 8]
            ),
            "gamma": trial.suggest_float("gamma", 0.99, 0.999),
            "lam": trial.suggest_float("lam", 0.9, 0.99),
            "entropy_coef": trial.suggest_float("entropy_coef", 1e-4, 3e-2, log=True),
            "clip_param": trial.suggest_float("clip_param", 0.1, 0.3),
            "max_grad_norm": trial.suggest_categorical("max_grad_norm", [0.5, 1.0]),
            "desired_kl": trial.suggest_float("desired_kl", 0.008, 0.02),
        },
        "actor": {"hidden_dims": list(hidden), "activation": activation},
        "critic": {"hidden_dims": list(hidden), "activation": activation},
    }


def _register() -> None:
    from srb.tuning.spaces import register_space

    register_space("rsl_rl", "ppo", "state", rsl_rl_ppo_space)


_register()
