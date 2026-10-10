"""Shared contract for framework policy adapters.

The pure callable result contract lives in ``srb.utils.policy_contract`` so
framework adapters can consume it without depending on this higher layer.
This module owns the integration-specific loader protocol and re-exports the
lower model-artifact configuration lookup used by framework loaders.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol

from srb.utils.model_artifacts import artifact_member_path, find_saved_policy_config
from srb.utils.policy_contract import (
    EpisodeStarter,
    LoadedPolicy,
    PolicyFunction,
    policy_action_to_numpy,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.integrations.algorithms import AlgorithmAdapter
    from srb.utils.model_artifacts import ModelArtifactReference


class PolicyLoader(Protocol):
    """Interface implemented by one framework policy-loading adapter."""

    def __call__(
        self,
        adapter: AlgorithmAdapter,
        model_path: Path,
        env: AnyEnv,
        *,
        agent_cfg: dict[str, Any] | None = None,
        env_cfg: Any = None,
        logdir: Path | None = None,
        artifact_reference: ModelArtifactReference | None = None,
    ) -> LoadedPolicy: ...


__all__ = [
    "EpisodeStarter",
    "LoadedPolicy",
    "PolicyFunction",
    "PolicyLoader",
    "artifact_member_path",
    "find_saved_policy_config",
    "policy_action_to_numpy",
]
