"""Isaac-free checks for per-env PhysX particle binding and settling.

Each env's particle set is bound to a sibling ``PhysxParticleSystem`` through
``physxParticle:particleSystem``. Under heterogeneous cloning, Isaac Lab 3.0
uses per-asset ``Sdf.CopySpec``; it remaps targets only inside the copied
subtree. Cloned sets can therefore keep targeting env_0's system, leaving one
live solver while other envs' particles fall through their terrain.

The system cannot be nested under ``UsdGeom.Points``: both are gprims, and
nested prims break USD encapsulation. A nested fluid isosurface would also
inherit the invisibility omni.physx applies to particle sets. Repoint sets to
their own env's system after cloning and before physics parses the stage;
validate the bindings at load time.

Only ``pxr`` is imported, so host tests can exercise these helpers without
Isaac Sim.
"""

from __future__ import annotations

import math
import re
from collections.abc import Sequence

from pxr import Sdf, Usd

PARTICLE_SYSTEM_REL = "physxParticle:particleSystem"
PARTICLE_SYSTEM_TYPE_NAME = "PhysxParticleSystem"

_ENV_NAME_RE = re.compile(r"env_\d+")
_MAX_REPORTED_PROBLEMS = 8
_BINDING_REMEDY = (
    "SRB re-points every cloned set to its own env's system in "
    "DirectEnv._setup_scene via "
    "srb.core.particle_binding.repoint_particle_sets_to_env_systems; a scene "
    "built outside DirectEnv must call it after cloning and before physics "
    "starts. See docs/src/config/particles.md."
)


def env_root_of(path: str | Sdf.Path) -> Sdf.Path | None:
    """Return the nearest ``env_<N>`` ancestor (or self) of ``path``, if any."""
    sdf_path = Sdf.Path(str(path))
    for prefix in reversed(sdf_path.GetPrefixes()):
        if _ENV_NAME_RE.fullmatch(prefix.name):
            return prefix
    return None


def repoint_particle_sets_to_env_systems(
    stage: Usd.Stage,
    env_prim_paths: Sequence[str],
) -> int:
    """Re-target each env's sets to its matching particle systems.

    Call after cloning and before physics parses the stage. Sets are direct
    children of the first env with a ``physxParticle:particleSystem`` relation.
    Targets in another env are replaced by the corresponding path in the set's
    env if it is a ``PhysxParticleSystem``. Other targets are left for
    :func:`validate_particle_system_binding` to report.

    Returns:
        The number of sets whose targets were re-pointed.
    """
    if len(env_prim_paths) < 2:
        return 0
    source_env = stage.GetPrimAtPath(str(env_prim_paths[0]))
    if not source_env:
        return 0
    leaves = [
        child.GetName()
        for child in source_env.GetChildren()
        if child.HasRelationship(PARTICLE_SYSTEM_REL)
    ]
    repointed = 0
    for env_path in env_prim_paths:
        env_root = Sdf.Path(str(env_path))
        for leaf in leaves:
            prim = stage.GetPrimAtPath(env_root.AppendChild(leaf))
            if not prim:
                continue
            rel = prim.GetRelationship(PARTICLE_SYSTEM_REL)
            targets = list(rel.GetTargets())
            new_targets = []
            for target in targets:
                target_env = env_root_of(target)
                if target_env is not None and target_env != env_root:
                    candidate = target.ReplacePrefix(target_env, env_root)
                    candidate_prim = stage.GetPrimAtPath(candidate)
                    if (
                        candidate_prim
                        and candidate_prim.GetTypeName() == PARTICLE_SYSTEM_TYPE_NAME
                    ):
                        target = candidate
                new_targets.append(target)
            if new_targets != targets:
                rel.SetTargets(new_targets)
                repointed += 1
    return repointed


def validate_particle_system_binding(
    stage: Usd.Stage,
    set_paths: Sequence[str],
) -> list[str]:
    """Check that each set has one unique, valid system in its own env.

    Each set needs exactly one target, an existing ``PhysxParticleSystem``.
    Sets under an ``env_<N>`` must target a system under that env; no sets may
    share a system.

    Returns:
        The bound system path of each set, in ``set_paths`` order.

    Raises:
        RuntimeError: If a requirement fails, listing offending sets so a
            misbound clone fails at load time instead of free-falling silently.
    """
    problems: list[str] = []
    systems: list[str] = []
    owners: dict[str, list[str]] = {}
    for set_path in set_paths:
        prim = stage.GetPrimAtPath(set_path)
        rel = prim.GetRelationship(PARTICLE_SYSTEM_REL) if prim else None
        targets = list(rel.GetTargets()) if rel else []
        if len(targets) != 1:
            problems.append(
                f"'{set_path}' has {len(targets)} particle-system target(s) "
                f"{[str(t) for t in targets]} (expected exactly 1)"
            )
            continue
        target = targets[0]
        system_prim = stage.GetPrimAtPath(target)
        if not system_prim or system_prim.GetTypeName() != PARTICLE_SYSTEM_TYPE_NAME:
            problems.append(
                f"'{set_path}' targets '{target}', which is not a "
                f"{PARTICLE_SYSTEM_TYPE_NAME} prim"
            )
        env_root = env_root_of(set_path)
        if env_root is not None and not target.HasPrefix(env_root):
            problems.append(
                f"'{set_path}' targets '{target}' outside its own env '{env_root}'"
            )
        systems.append(str(target))
        owners.setdefault(str(target), []).append(str(set_path))
    for system_path, sharing_sets in owners.items():
        if len(sharing_sets) > 1:
            problems.append(
                f"system '{system_path}' is shared by {len(sharing_sets)} sets "
                f"{sharing_sets}"
            )
    if problems:
        shown = problems[:_MAX_REPORTED_PROBLEMS]
        if len(problems) > len(shown):
            shown.append(f"... and {len(problems) - len(shown)} more")
        raise RuntimeError(
            f"Particle sets are not bound to {len(set_paths)} distinct env-local "
            f"particle systems ({len(owners)} distinct system(s) found): "
            + "; ".join(shown)
            + ". A particle set bound to another env's system is solved in that "
            "env's collision group and falls through its own terrain. "
            + _BINDING_REMEDY
        )
    return systems


def settle_batch_converged(
    median_speed: float,
    median_speed_change: float,
    *,
    speed_threshold: float,
    min_speed_change_threshold: float,
    plateau_max_speed: float,
) -> bool:
    """Return whether the batch meets the speed or bounded-plateau criteria."""
    if median_speed <= speed_threshold:
        return True
    return (
        median_speed_change <= min_speed_change_threshold
        and median_speed <= plateau_max_speed
    )


def check_settle_drop(
    start_median_z: Sequence[float],
    median_z: Sequence[float],
    max_drop: float,
) -> None:
    """Raise if any env's drop exceeds ``max_drop`` or is non-finite."""
    if not math.isfinite(max_drop):
        raise ValueError(f"max_drop must be finite, got {max_drop!r}")
    if len(start_median_z) != len(median_z):
        raise ValueError(
            f"env count mismatch: {len(start_median_z)} start vs "
            f"{len(median_z)} current median heights"
        )
    falling = []
    for env_id, (z0, z) in enumerate(zip(start_median_z, median_z, strict=True)):
        drop = float(z0) - float(z)
        if not math.isfinite(drop) or drop > max_drop:
            falling.append(f"env {env_id}: {float(z0):.3f} -> {float(z):.3f} m")
    if falling:
        raise RuntimeError(
            f"Particles fell more than settle_max_median_drop={max_drop} m while "
            f"settling ({'; '.join(falling)}); refusing to store this state as the "
            "reset default. Likely causes: a particle set bound to another "
            "env's particle system (see validate_particle_system_binding), or "
            "particles spawned below the terrain collider. If the spawn height "
            "is intentional, raise ParticleSystemCfg.settle_max_median_drop."
        )
