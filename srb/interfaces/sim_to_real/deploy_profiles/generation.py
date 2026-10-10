"""Registry of task-specific profiles used during generation."""

from __future__ import annotations

from dataclasses import dataclass

from srb.interfaces.sim_to_real.core.hardware import DeploymentCapability


@dataclass(frozen=True, slots=True)
class DeployProfileSpec:
    """Import target for a generated RealEnv deployment profile."""

    module: str
    symbol: str


@dataclass(frozen=True, slots=True)
class CanonicalGenerationPolicy:
    """Per-env policy for canonical (contract-backed) RealEnv generation.

    Canonical envs must generate through the typed ``env.srb_spec`` seam with
    contract metadata present; generation fails otherwise. The
    policy also pins the deployment identity the generated module declares.

    ``requires_runtime_target_manifest=True`` means the generated module
    carries no static target-manifest hash (``TARGET_MANIFEST_SHA256=None``)
    and remains non-ready until a per-episode manifest is supplied (general
    landscaping). ``RealEnv`` calls the manifest's ``verify()`` before
    hardware start but does not compare ``manifest_sha256`` at reset.
    ``False`` pins the static manifest hash reported by the simulator (crater
    task).
    """

    requires_runtime_target_manifest: bool
    deployment_requirements: frozenset[DeploymentCapability]
    task_name: str = ""


_LANDSCAPING_PROFILE = DeployProfileSpec(
    module="srb.interfaces.sim_to_real.deploy_profiles.terrain_landscaping",
    symbol="configure_terrain_landscaping_hardware",
)

# The canonical landscaping deployment requires all four capability tags
# before ROS acquisition, hardware start, or policy load.
LANDSCAPING_DEPLOYMENT_REQUIREMENTS: frozenset[DeploymentCapability] = frozenset(
    {
        DeploymentCapability.RAPH_DRIVE_VELOCITY,
        DeploymentCapability.RAPH_SHOVEL_PRISMATIC_VELOCITY,
        DeploymentCapability.LANDSCAPING_EXTERNAL_HEIGHTMAP_BATCH,
        DeploymentCapability.LANDSCAPING_OBSERVABLE_TASK_EVALUATOR,
    }
)

_DEPLOY_PROFILES_BY_ENV_NAME: dict[str, DeployProfileSpec] = {
    # Both canonical landscaping task IDs map to the same canonical
    # landscaping configurator; the crater task must never lose the profile.
    "terrain_landscaping": _LANDSCAPING_PROFILE,
    "terrain_landscaping_crater": _LANDSCAPING_PROFILE,
    "formation_following": DeployProfileSpec(
        module="srb.interfaces.sim_to_real.deploy_profiles.multi_robot",
        symbol="configure_formation_following_hardware",
    ),
    "beam_transport": DeployProfileSpec(
        module="srb.interfaces.sim_to_real.deploy_profiles.multi_robot",
        symbol="configure_beam_transport_hardware",
    ),
    "waypoint_navigation": DeployProfileSpec(
        module="srb.interfaces.sim_to_real.deploy_profiles.multi_robot",
        symbol="configure_waypoint_navigation_hardware",
    ),
    # Training heightmap grid only; excavation is not yet deployable (dated
    # blocker in the sim-to-real workflow).
    "excavation": DeployProfileSpec(
        module="srb.interfaces.sim_to_real.deploy_profiles.excavation",
        symbol="configure_excavation_hardware",
    ),
}

_CANONICAL_GENERATION_POLICIES_BY_ENV_NAME: dict[str, CanonicalGenerationPolicy] = {
    "terrain_landscaping": CanonicalGenerationPolicy(
        requires_runtime_target_manifest=True,
        deployment_requirements=LANDSCAPING_DEPLOYMENT_REQUIREMENTS,
        task_name="terrain_landscaping",
    ),
    "terrain_landscaping_crater": CanonicalGenerationPolicy(
        requires_runtime_target_manifest=False,
        deployment_requirements=LANDSCAPING_DEPLOYMENT_REQUIREMENTS,
        task_name="terrain_landscaping_crater",
    ),
}


def _env_name(env_spec_id: str) -> str:
    return env_spec_id.rsplit("/", 1)[-1]


def deploy_profile_for_env_spec_id(env_spec_id: str) -> DeployProfileSpec | None:
    """Return the profile to emit for a generated RealEnv module."""

    return _DEPLOY_PROFILES_BY_ENV_NAME.get(_env_name(env_spec_id))


def canonical_generation_policy_for_env_spec_id(
    env_spec_id: str,
) -> CanonicalGenerationPolicy | None:
    """Return the canonical policy, or ``None`` for legacy environments."""

    return _CANONICAL_GENERATION_POLICIES_BY_ENV_NAME.get(_env_name(env_spec_id))


__all__ = [
    "CanonicalGenerationPolicy",
    "DeployProfileSpec",
    "LANDSCAPING_DEPLOYMENT_REQUIREMENTS",
    "canonical_generation_policy_for_env_spec_id",
    "deploy_profile_for_env_spec_id",
]
