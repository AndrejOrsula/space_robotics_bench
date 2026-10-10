"""SKRL search spaces for PPO and the verified SAC profile."""

from __future__ import annotations

import copy

# Optuna categoricals require scalar choices.
_LAYERS = {
    "256x256": [256, 256],
    "384x384": [384, 384],
    "512x512": [512, 512],
    "512x256": [512, 256],
}


def _spliced_network(
    base_network: list | None, layers: list[int], activations: str
) -> list:
    """Tune layers while preserving task-specific fields from the base network."""
    if base_network:
        elem = copy.deepcopy(base_network[0])
    else:
        elem = {"name": "net"}
    elem["layers"] = list(layers)
    elem["activations"] = activations
    return [elem]


def skrl_ppo_space(trial, ctx) -> dict:
    base = ctx.base_agent_cfg or {}
    models = base.get("models", {})
    policy_net = models.get("policy", {}).get("network")
    value_net = models.get("value", {}).get("network")

    lr = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
    rollouts = trial.suggest_categorical("rollouts", [16, 32, 64, 128])
    learning_epochs = trial.suggest_int("learning_epochs", 4, 16)
    mini_batches = trial.suggest_categorical("mini_batches", [2, 4, 8, 16])
    discount_factor = trial.suggest_float("discount_factor", 0.99, 0.999)
    # Keep the study parameter name; SKRL 2.1 expects ``gae_lambda`` in config.
    lam = trial.suggest_float("lambda", 0.9, 0.99)
    entropy = trial.suggest_float("entropy_loss_scale", 1e-4, 3e-2, log=True)
    ratio_clip = trial.suggest_float("ratio_clip", 0.1, 0.3)
    grad_norm_clip = trial.suggest_categorical("grad_norm_clip", [0.5, 1.0])
    layers = _LAYERS[trial.suggest_categorical("layers", list(_LAYERS))]
    activations = trial.suggest_categorical("activations", ["elu", "tanh"])
    initial_log_std = trial.suggest_float("initial_log_std", -2.5, -0.5)

    return {
        "agent": {
            "learning_rate": lr,
            "rollouts": rollouts,
            "learning_epochs": learning_epochs,
            "mini_batches": mini_batches,
            "discount_factor": discount_factor,
            "gae_lambda": lam,
            "entropy_loss_scale": entropy,
            "ratio_clip": ratio_clip,
            "value_clip": ratio_clip,
            "grad_norm_clip": grad_norm_clip,
        },
        # Match memory capacity to the rollout size.
        "memory": {"memory_size": rollouts},
        "models": {
            "policy": {
                "initial_log_std": initial_log_std,
                "network": _spliced_network(policy_net, layers, activations),
            },
            "value": {
                "network": _spliced_network(value_net, layers, activations),
            },
        },
    }


def skrl_ppo_rnn_space(trial, ctx) -> dict:
    """Recurrent PPO space using ``hidden_size`` and ``num_layers`` fields."""
    lr = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
    rollouts = trial.suggest_categorical("rollouts", [16, 32, 64, 128])
    learning_epochs = trial.suggest_int("learning_epochs", 4, 16)
    mini_batches = trial.suggest_categorical("mini_batches", [2, 4, 8, 16])
    discount_factor = trial.suggest_float("discount_factor", 0.99, 0.999)
    lam = trial.suggest_float("lambda", 0.9, 0.99)
    entropy = trial.suggest_float("entropy_loss_scale", 1e-4, 3e-2, log=True)
    ratio_clip = trial.suggest_float("ratio_clip", 0.1, 0.3)
    grad_norm_clip = trial.suggest_categorical("grad_norm_clip", [0.5, 1.0])
    hidden_size = trial.suggest_categorical("hidden_size", [64, 128, 256])
    num_layers = trial.suggest_categorical("num_layers", [1, 2])
    initial_log_std = trial.suggest_float("initial_log_std", -2.5, -0.5)

    return {
        "agent": {
            "learning_rate": lr,
            "rollouts": rollouts,
            "learning_epochs": learning_epochs,
            "mini_batches": mini_batches,
            "discount_factor": discount_factor,
            "gae_lambda": lam,
            "entropy_loss_scale": entropy,
            "ratio_clip": ratio_clip,
            "value_clip": ratio_clip,
            "grad_norm_clip": grad_norm_clip,
        },
        "memory": {"memory_size": rollouts},
        "models": {
            "policy": {
                "initial_log_std": initial_log_std,
                "hidden_size": hidden_size,
                "num_layers": num_layers,
            },
            "value": {"hidden_size": hidden_size, "num_layers": num_layers},
        },
    }


def skrl_sac_space(trial, ctx) -> dict:
    """Tune fields consumed by the SRB SKRL SAC profile."""
    base = ctx.base_agent_cfg or {}
    models = base.get("models", {})
    policy_net = models.get("policy", {}).get("network")
    critic_nets = {
        role: models.get(role, {}).get("network")
        for role in (
            "critic_1",
            "critic_2",
            "target_critic_1",
            "target_critic_2",
        )
    }

    policy_learning_rate = trial.suggest_float(
        "actor_learning_rate", 1e-5, 1e-3, log=True
    )
    critics_learning_rate = trial.suggest_float(
        "critic_learning_rate", 1e-5, 1e-3, log=True
    )
    batch_size = trial.suggest_categorical("batch_size", [256, 512, 1024])
    gradient_steps = trial.suggest_categorical("gradient_steps", [1, 2, 4])
    learning_starts = trial.suggest_categorical(
        "learning_starts", [10000, 50000, 100000]
    )
    discount_factor = trial.suggest_float("discount_factor", 0.99, 0.999)
    polyak = trial.suggest_float("polyak", 1e-3, 2e-2, log=True)
    entropy_learning_rate = trial.suggest_float(
        "entropy_learning_rate", 1e-5, 1e-3, log=True
    )
    initial_entropy_value = trial.suggest_float("initial_entropy_value", 0.1, 2.0)
    grad_norm_clip = trial.suggest_categorical("grad_norm_clip", [0.0, 0.5, 1.0])
    layers = _LAYERS[trial.suggest_categorical("layers", list(_LAYERS))]
    activations = trial.suggest_categorical("activations", ["elu", "tanh"])
    initial_log_std = trial.suggest_float("initial_log_std", -2.5, -0.5)

    model_overrides = {
        "policy": {
            "initial_log_std": initial_log_std,
            "network": _spliced_network(policy_net, layers, activations),
        }
    }
    model_overrides.update(
        {
            role: {"network": _spliced_network(network, layers, activations)}
            for role, network in critic_nets.items()
        }
    )
    return {
        "agent": {
            # Preserve trial names; SKRL 2.1 uses this list for policy,
            # critics, and entropy learning rates.
            "learning_rate": [
                policy_learning_rate,
                critics_learning_rate,
                entropy_learning_rate,
            ],
            "batch_size": batch_size,
            "gradient_steps": gradient_steps,
            "learning_starts": learning_starts,
            "discount_factor": discount_factor,
            "polyak": polyak,
            "initial_entropy_value": initial_entropy_value,
            "grad_norm_clip": grad_norm_clip,
        },
        "models": model_overrides,
    }


def _register() -> None:
    from srb.tuning.spaces import register_space

    register_space("skrl", "ppo", "state", skrl_ppo_space)
    register_space("skrl", "ppo_rnn", "state", skrl_ppo_rnn_space)
    register_space("skrl", "sac", "state", skrl_sac_space)


_register()
