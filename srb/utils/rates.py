from __future__ import annotations

import math

# Allow floating-point representation error when checking the rate ratio.
_RATIO_RTOL = 1e-6


def resolve_decimation(agent_rate: float, env_rate: float) -> int:
    """Resolve the physics-step decimation from an agent/environment rate pair.

    The ratio must be an exact integer, not merely ``>= 1``. Flooring would
    silently accept e.g. ``agent_rate / env_rate == 2.5``: the environment
    would advance two physics steps per agent step while every consumer that
    recomputes the quantum from the rates (such as the delay subsystem in
    ``srb/core/env/runtime/delay.py``) would assume 2.5. A rate pair that
    cannot be represented is a configuration error, so it is rejected here
    rather than approximated.

    Lives in ``srb.utils`` -- which ``BaseEnvCfg`` already imports and which
    imports nothing from ``srb.core`` -- rather than beside
    ``srb/core/env/runtime/delay.py``, its conceptual sibling. Importing
    ``srb.core.env.runtime.rates`` executes that package's ``__init__``, which
    eagerly pulls in ``lifecycle``, ``delay``, ``outcome`` and ``spaces``;
    ``srb/core/env/common/base/env_cfg.py`` needs none of them, and a
    validation helper should not couple it to that package.

    Raises:
        ValueError: if either rate is not finite and positive, if the ratio
            rounds below 1, or if it is not a whole number of physics steps.
    """

    # Validate first: zero cannot divide, non-finite values cannot round, and
    # negative periods are invalid even when their ratio is positive.
    for name, rate in (("agent_rate", agent_rate), ("env_rate", env_rate)):
        if isinstance(rate, bool) or not math.isfinite(rate) or rate <= 0.0:
            raise ValueError(
                f"Invalid {name}={rate!r}: rates are periods in seconds and "
                "must be finite and positive."
            )

    ratio = agent_rate / env_rate
    # Extreme finite rates can overflow the ratio, which ``round`` rejects.
    if not math.isfinite(ratio):
        raise ValueError(
            f"Invalid agent_rate/env_rate combination: agent_rate="
            f"{agent_rate!r}, env_rate={env_rate!r} yields a non-finite ratio."
        )
    decimation = round(ratio)
    if decimation < 1:
        raise ValueError(
            f"Invalid agent_rate/env_rate combination: agent_rate="
            f"{agent_rate!r}, env_rate={env_rate!r} yields "
            f"decimation={decimation} (must be >= 1). agent_rate must be "
            "greater than or equal to env_rate; check for swapped or "
            "misconfigured rate overrides."
        )
    if abs(ratio - decimation) > _RATIO_RTOL * decimation:
        raise ValueError(
            f"Invalid agent_rate/env_rate combination: agent_rate="
            f"{agent_rate!r}, env_rate={env_rate!r} yields a "
            f"non-integral ratio {ratio!r}. agent_rate must be an exact "
            "integer multiple of env_rate; pick rates whose ratio is a "
            "whole number of physics steps."
        )
    return decimation
