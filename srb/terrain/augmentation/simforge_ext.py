"""Build SimForge configs for optional terrain rock scatter.

Foundry is imported lazily. Rock density and size are passed when supported;
dust and crater preset fields are not exposed by SimForge.
"""

from __future__ import annotations

import hashlib
import inspect
import math
from typing import Any

from srb.terrain.augmentation.presets import AugmentPreset
from srb.terrain.errors import SimforgeMissingError


def _import_simforge_foundry() -> Any:
    """Lazy-import; raise SimforgeMissingError if unavailable."""
    import sys

    if (
        sys.modules.get("simforge_foundry") is None
        and "simforge_foundry" in sys.modules
    ):
        # Sentinel set by tests to force the missing path.
        raise SimforgeMissingError(
            "simforge_foundry import was disabled in this environment"
        )
    try:
        import simforge_foundry  # noqa: F401
    except ImportError as e:
        raise SimforgeMissingError(f"simforge_foundry is not installed: {e}") from e
    return sys.modules["simforge_foundry"]


def _seed_for(*, base_usd: str, seed_offset: int, body: str) -> int:
    """Deterministic per-(base_usd, seed_offset, body) seed via SHA256.

    Python's built-in ``hash()`` is randomized per process by default (PEP
    456), which would yield a different SimForge scatter every run for the
    same inputs.
    """
    seed_input = f"{base_usd}|{seed_offset}|{body}".encode()
    return int(hashlib.sha256(seed_input).hexdigest()[:8], 16)


def _build_rock_generator(sf, body: str, preset: AugmentPreset) -> Any:
    """Construct the body-appropriate SimForge rock generator with preset
    parameters, falling back to defaults if the constructor doesn't accept them."""
    if body == "moon":
        rock_cls = sf.MoonRock
    elif body == "mars":
        rock_cls = sf.MarsRock
    else:
        raise ValueError(
            f"no SimForge rock generator for body {body!r} (earth rock sets "
            "are a documented placeholder — augmentation presets ship "
            "rock_density_per_m2=0.0 there)"
        )
    candidate_kwargs = {
        "density": preset.rock_density_per_m2,
        "scale_range": preset.rock_size_range_m,
    }
    try:
        signature = inspect.signature(rock_cls)
    except (TypeError, ValueError):
        return rock_cls(**candidate_kwargs)
    try:
        signature.bind(**candidate_kwargs)
    except TypeError:
        return rock_cls()
    return rock_cls(**candidate_kwargs)


def build_real_terrain_augment_cfg(
    *,
    base_usd: str,
    body: str,
    preset: AugmentPreset,
    seed_offset: int = 0,
) -> Any:
    """Build a SimforgeAssetCfg for rock scatter, or None at zero density.

    A zero-density preset returns before importing Foundry; callers should
    omit the rock companion in that case.
    """
    density = preset.rock_density_per_m2
    if not math.isfinite(density) or density < 0.0:
        raise ValueError(
            f"rock density must be finite and nonnegative, got {density!r}"
        )
    if density == 0.0:
        return None

    sf = _import_simforge_foundry()

    from srb.core.sim import CollisionPropertiesCfg, SimforgeAssetCfg

    rock_asset = _build_rock_generator(sf, body, preset)

    seed = _seed_for(base_usd=base_usd, seed_offset=seed_offset, body=preset.body)

    return SimforgeAssetCfg(
        assets=[
            # The caller layers the baked terrain separately; SimForge adds
            # only the rock scatter.
            rock_asset,
        ],
        collision_props=CollisionPropertiesCfg(),
        seed=seed,
    )
