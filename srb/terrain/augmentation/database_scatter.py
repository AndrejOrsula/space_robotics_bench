"""Explicitly place rocks from the augmentation database."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import numpy as np

from srb.terrain.errors import SimforgeMissingError
from srb.utils.failures import (
    InternalInvariantError,
    OptionalDependencyUnavailableError,
)

if TYPE_CHECKING:
    from srb.terrain.augmentation.database import AugmentationDb
    from srb.terrain.augmentation.presets import AugmentPreset


def select_rows(rocks: np.ndarray, *, max_rocks: int) -> np.ndarray:
    """Return the first rows in draw order, capped deterministically."""
    return rocks[: max(int(max_rocks), 0)]


def rock_pose(
    row: np.ndarray,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Convert a row to position and yaw-only xyzw quaternion."""
    x, y, z, _scale, yaw, _idx = (float(v) for v in row[:6])
    half = 0.5 * yaw
    return (x, y, z), (0.0, 0.0, math.sin(half), math.cos(half))


def _bake_rock_variants(
    *, base_usd: str, body: str, preset: "AugmentPreset"
) -> list[str]:
    """Bake and return cached SimForge rock variant USD paths."""
    import simforge as _sf_check  # noqa: F401  (fail fast with a clear error)
    import simforge_foundry as sf
    from simforge import ModelFileFormat

    from srb.terrain.augmentation.database import _ROCK_ASSET_SLOTS
    from srb.terrain.augmentation.simforge_ext import _build_rock_generator, _seed_for

    asset = _build_rock_generator(sf, body, preset)
    generator = asset.generator_type(
        num_assets=_ROCK_ASSET_SLOTS,
        seed=_seed_for(base_usd=base_usd, seed_offset=0, body=preset.body),
        file_format=ModelFileFormat.USDZ,
        use_cache=True,
    )
    output = generator.generate_subprocess(asset, export_kwargs={})
    return [fp.as_posix() for fp, _meta in output]


def build_database_rock_cfgs(
    *,
    db: "AugmentationDb",
    base_usd: str,
    body: str,
    preset: "AugmentPreset",
    max_rocks: int = 256,
) -> list[Any]:
    """Build rock configs from recorded transforms.

    Missing dependencies and empty generator output raise typed errors;
    generator failures propagate instead of producing an empty scene.
    """
    from srb.utils import logging

    raw_rocks = np.asarray(db.rocks)
    if raw_rocks.ndim != 2 or raw_rocks.shape[1] != 6:
        raise ValueError(
            f"augmentation rock rows must have shape (N, 6), got {raw_rocks.shape}"
        )
    if not np.isfinite(raw_rocks).all():
        raise ValueError("augmentation rock rows must contain only finite values")
    if np.any(np.abs(raw_rocks) > np.finfo(np.float32).max):
        raise ValueError("augmentation rock rows exceed the float32 range")
    rows = select_rows(raw_rocks.astype(np.float32), max_rocks=max_rocks)
    if rows.size == 0:
        return []

    if db.meta.get("upgraded_from") == 1:
        logging.warning(
            '[terrain] augment="database" sidecar for base_usd=%r was '
            "auto-upgraded from v1 (no z_m column): rocks are placed at z=0 "
            "(patch-centre plane) and may float or be buried. Re-bake the "
            "site for a v2 sidecar with sampled z_m.",
            base_usd,
        )

    try:
        variants = _bake_rock_variants(base_usd=base_usd, body=body, preset=preset)
    except (ImportError, SimforgeMissingError) as exc:
        error = OptionalDependencyUnavailableError(
            code="terrain.augmentation.simforge_unavailable",
            message=(
                "database rock placement requires the SimForge and "
                "simforge-foundry dependencies"
            ),
            component="terrain.augmentation",
            details={
                "body": body,
                "base_usd": base_usd,
                "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
            },
            fallback_allowed=False,
            evidence_valid=False,
        )
        raise error from exc

    if not variants:
        raise InternalInvariantError(
            code="terrain.augmentation.empty_rock_variant_set",
            message="SimForge returned no rock variants for database placement",
            component="terrain.augmentation",
            details={"body": body, "base_usd": base_usd},
            fallback_allowed=False,
            evidence_valid=False,
        )

    from srb.core.asset import AssetBaseCfg
    from srb.core.sim import CollisionPropertiesCfg, UsdFileCfg

    cfgs: list[Any] = []
    for i, row in enumerate(rows):
        pos, quat = rock_pose(row)
        scale = float(row[3])
        usd = variants[int(row[5]) % len(variants)]
        cfgs.append(
            AssetBaseCfg(
                prim_path=f"{{ENV_REGEX_NS}}/augment_rocks/rock_{i:03d}",
                spawn=UsdFileCfg(
                    usd_path=usd,
                    scale=(scale, scale, scale),
                    collision_props=CollisionPropertiesCfg(),
                ),
                init_state=AssetBaseCfg.InitialStateCfg(pos=pos, rot=quat),
            )
        )
    return cfgs


__all__ = [
    "build_database_rock_cfgs",
    "rock_pose",
    "select_rows",
]
