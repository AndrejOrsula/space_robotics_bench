"""TD-MPC2 state space for optimization, MPPI, and loss settings.

Sampled fields exist in the base YAML. Architecture dimensions stay fixed
because ``parse_cfg`` derives them from ``model_size``; the driver sets the
environment-step budget through ``steps``.
"""

from __future__ import annotations


def tdmpc2_space(trial, ctx) -> dict:
    num_samples = trial.suggest_categorical("num_samples", [256, 512])
    num_elites = trial.suggest_categorical("num_elites", [32, 64])
    return {
        # Optimization
        "lr": trial.suggest_float("lr", 1e-4, 1e-3, log=True),
        "enc_lr_scale": trial.suggest_float("enc_lr_scale", 0.1, 1.0),
        "batch_size": trial.suggest_categorical("batch_size", [256, 512]),
        "grad_clip_norm": trial.suggest_categorical("grad_clip_norm", [10, 20]),
        "tau": trial.suggest_float("tau", 5e-3, 2e-2, log=True),
        # MPPI planner
        "horizon": trial.suggest_categorical("horizon", [3, 4, 5]),
        "num_samples": num_samples,
        # num_elites <= num_samples holds for every sampled combination above.
        "num_elites": num_elites,
        "num_pi_trajs": trial.suggest_categorical("num_pi_trajs", [12, 24]),
        "iterations": trial.suggest_categorical("iterations", [4, 6, 8]),
        # Loss coefficients and discount shaping.
        "consistency_coef": trial.suggest_categorical("consistency_coef", [10, 20, 40]),
        "rho": trial.suggest_float("rho", 0.5, 0.9),
        "entropy_coef": trial.suggest_float("entropy_coef", 1e-4, 1e-2, log=True),
    }


def _register() -> None:
    from srb.tuning.spaces import register_space

    register_space("tdmpc2", "tdmpc2", "state", tdmpc2_space)


_register()
