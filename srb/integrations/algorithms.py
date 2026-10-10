"""Algorithm-family metadata for SRB framework integrations."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from srb.core.algorithms import canonical_algorithm_label

DemoTrainingMode = Literal["unsupported", "offline", "replay_prefill"]

# Keep this allowlist deliberately label-specific.  A framework family name is
# not evidence that every algorithm in that family can consume demonstrations.
DEMO_REPLAY_PREFILL_ALGORITHMS = frozenset(
    {"skrl_sac", "skrl_ddpg", "skrl_td3", "dreamer"}
)


@dataclass(frozen=True)
class AlgorithmAdapter:
    """Resolved framework adapter for an SRB algorithm label.

    ``local_algo`` holds the framework-local name (for example,
    ``skrl_ppo`` becomes ``ppo``) used by tuning and export. ``passes_algo_kwarg``
    separately controls forwarding it to integrations whose ``run()`` accepts
    ``algo=`` (sb3, sbx, skrl, and robomimic).
    """

    framework: str
    integration: str
    local_algo: str
    online_training: bool = True
    real_training: bool = False
    offline_learning: bool = False
    policy_evaluation: bool = True
    policy_collection: bool = True
    passes_algo_kwarg: bool = False
    training_dataset_mode: DemoTrainingMode = "unsupported"

    def integration_kwargs(self) -> dict[str, str]:
        """Return kwargs expected by the integration entry point."""
        return {"algo": self.local_algo} if self.passes_algo_kwarg else {}


def resolve_algorithm_adapter(algo: str) -> AlgorithmAdapter:
    """Resolve an SRB algorithm label to its experiment adapter metadata."""
    algo = canonical_algorithm_label(algo)
    if algo == "dreamer":
        return AlgorithmAdapter(
            framework="dreamer",
            integration="dreamer",
            local_algo="dreamer",
            real_training=True,
            training_dataset_mode="replay_prefill",
        )
    if algo == "tdmpc2":
        return AlgorithmAdapter(
            framework="tdmpc2", integration="tdmpc2", local_algo="tdmpc2"
        )
    if algo.startswith("rsl_rl_"):
        return AlgorithmAdapter(
            framework="rsl_rl",
            integration="rsl_rl",
            local_algo=algo.removeprefix("rsl_rl_"),
        )
    if algo.startswith("skrl_"):
        return AlgorithmAdapter(
            framework="skrl",
            integration="skrl",
            local_algo=algo.removeprefix("skrl_"),
            passes_algo_kwarg=True,
            training_dataset_mode=(
                "replay_prefill"
                if algo in DEMO_REPLAY_PREFILL_ALGORITHMS
                else "unsupported"
            ),
        )
    if algo.startswith("sb3_"):
        return AlgorithmAdapter(
            framework="sb3",
            integration="sb3",
            local_algo=algo.removeprefix("sb3_"),
            passes_algo_kwarg=True,
        )
    if algo.startswith("sbx_"):
        return AlgorithmAdapter(
            framework="sbx",
            integration="sbx",
            local_algo=algo.removeprefix("sbx_"),
            passes_algo_kwarg=True,
        )
    if algo.startswith("robomimic_"):
        return AlgorithmAdapter(
            framework="robomimic",
            integration="robomimic",
            local_algo=algo.removeprefix("robomimic_"),
            online_training=False,
            offline_learning=True,
            passes_algo_kwarg=True,
            training_dataset_mode="offline",
        )
    raise ValueError(f"Unsupported algorithm: {algo}")


def find_algorithm_adapter(algo: str) -> AlgorithmAdapter | None:
    """Return adapter metadata for *algo*, or ``None`` if unsupported."""
    try:
        return resolve_algorithm_adapter(algo)
    except ValueError:
        return None


def supports_policy_collection(algo: str) -> bool:
    """Return whether *algo* supports policy rollout into a dataset collector."""
    adapter = find_algorithm_adapter(algo)
    return bool(adapter and adapter.policy_collection)


def supports_demo_replay_prefill(algo: str) -> bool:
    """Return whether *algo* has a validated demonstration replay input."""
    try:
        normalized_algo = canonical_algorithm_label(algo)
    except ValueError:
        return False
    return normalized_algo in DEMO_REPLAY_PREFILL_ALGORITHMS


def validate_demo_training_inputs(
    algo: str,
    demo_dataset: str | Path | None = None,
    demo_max_transitions: object | None = None,
) -> None:
    """Validate optional bounded demonstration-training inputs.

    Checks the input contract only; the replay-prefill consumer validates the
    dataset. If both values are absent, existing training calls are unchanged.
    """
    if demo_dataset is None and demo_max_transitions is None:
        return

    normalized_algo = canonical_algorithm_label(algo)
    if normalized_algo not in DEMO_REPLAY_PREFILL_ALGORITHMS:
        raise ValueError(
            "Demonstration replay prefill is supported only for the exact "
            f"algorithm labels {', '.join(sorted(DEMO_REPLAY_PREFILL_ALGORITHMS))}; "
            f"got {normalized_algo!r}"
        )
    if demo_dataset is None or demo_max_transitions is None:
        raise ValueError(
            "--demo-dataset and --demo-max-transitions must be provided as a pair"
        )
    if not isinstance(demo_dataset, (str, Path)) or not str(demo_dataset).strip():
        raise ValueError("demo_dataset must be a non-empty path")
    if (
        isinstance(demo_max_transitions, bool)
        or not isinstance(demo_max_transitions, int)
        or demo_max_transitions <= 0
    ):
        raise ValueError("demo_max_transitions must be a positive integer")


__all__ = (
    "AlgorithmAdapter",
    "DEMO_REPLAY_PREFILL_ALGORITHMS",
    "DemoTrainingMode",
    "find_algorithm_adapter",
    "resolve_algorithm_adapter",
    "supports_demo_replay_prefill",
    "supports_policy_collection",
    "validate_demo_training_inputs",
)
