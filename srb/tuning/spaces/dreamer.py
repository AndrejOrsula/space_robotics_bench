"""DreamerV3 state space using keys recognized by the SRB base config.

It tunes ``run.train_ratio`` and the base file's regex-keyed capacity presets;
unknown config keys would be silently dropped. The driver sets ``run.steps``.
"""

from __future__ import annotations

# Upstream DreamerV3 ``size*`` capacity presets. Omit the
# 200M/400M sizes to keep trials inexpensive.
_MODEL_SIZE = {
    "12M": {
        "rssm": {"deter": 2048, "hidden": 256, "classes": 16},
        "depth": 16,
        "units": 256,
    },
    "25M": {
        "rssm": {"deter": 3072, "hidden": 384, "classes": 24},
        "depth": 24,
        "units": 384,
    },
    "50M": {
        "rssm": {"deter": 4096, "hidden": 512, "classes": 32},
        "depth": 32,
        "units": 512,
    },
    "100M": {
        "rssm": {"deter": 6144, "hidden": 768, "classes": 48},
        "depth": 48,
        "units": 768,
    },
}


def dreamer_space(trial, ctx) -> dict:
    preset = _MODEL_SIZE[trial.suggest_categorical("model_size", list(_MODEL_SIZE))]
    # Include 512: the landscaping mound task missed its margin at 16 and
    # cleared it at 512, with 13x more updates in 45% of the wall time.
    train_ratio = trial.suggest_categorical("train_ratio", [16, 32, 64, 128, 256, 512])
    return {
        # Merge into the base ``run:`` mapping.
        "run": {"train_ratio": float(train_ratio)},
        # Opaque regex keys replace the base leaves and are applied by DreamerV3.
        r".*\.rssm": dict(preset["rssm"]),
        r".*\.depth": preset["depth"],
        r".*\.units": preset["units"],
    }


def _register() -> None:
    from srb.tuning.spaces import register_space

    register_space("dreamer", "dreamer", "state", dreamer_space)


_register()
