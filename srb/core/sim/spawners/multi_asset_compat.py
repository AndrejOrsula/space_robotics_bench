"""Adapt internal multi-variant spawners to Isaac Lab clone planning."""

import copy
from typing import TYPE_CHECKING, Any, List, Optional

from isaaclab.sim.spawners.wrappers import MultiAssetSpawnerCfg, spawn_multi_asset
from pxr import Usd

from srb.utils import logging
from srb.utils.clone_variants import PerEnvironmentCloneStrategy
from srb.utils.color import seeded_color_palette_hue

if TYPE_CHECKING:
    from simforge.integrations.isaaclab.spawner.simforge_asset.cfg import (
        SimforgeAssetCfg,
    )

    from .shapes.multi_shape.cfg import MultiShapeSpawnerCfg

# Attributes of SimforgeAssetCfg that must not be forwarded to the per-file
# prototype cfg (mirrors simforge.integrations.isaaclab.spawner.simforge_asset).
_SIMFORGE_IGNORED_SPAWN_ATTRIBUTES = (
    "func",
    "assets",
    "export_kwargs",
    "num_assets",
    "seed",
    "use_cache",
    "random_choice",
    # Never forward planner-assigned spawn locations to prototypes.
    "spawn_path",
    "spawn_paths",
)


def spawn_multi_asset_compat(
    prim_path: str,
    cfg: MultiAssetSpawnerCfg,
    translation: Optional[tuple[float, float, float]] = None,
    orientation: Optional[tuple[float, float, float, float]] = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn one concrete-path variant; otherwise use upstream behavior.

    Planner-assigned ``spawn_paths`` and ``.*`` bases pass through unchanged.
    """
    if getattr(cfg, "spawn_paths", None) is None and hasattr(cfg, "spawn_paths"):
        base_name = prim_path.rsplit("/", 1)[-1]
        if ".*" not in base_name:
            num_assets = len(cfg.assets_cfg)
            if num_assets > 1:
                logging.warning(
                    f"Spawning only the first of {num_assets} asset variants at "
                    f'the concrete prim path "{prim_path}". Convert the spawner '
                    "to MultiAssetSpawnerCfg before scene construction to let "
                    "the clone planner distribute the variants across envs."
                )
            # Avoid leaving a stale path on a shared cfg used by later spawns.
            # A shallow copy also avoids rerunning configclass initialization.
            cfg = copy.copy(cfg)
            cfg.spawn_paths = [prim_path] + [None] * (num_assets - 1)
    return spawn_multi_asset(prim_path, cfg, translation, orientation, **kwargs)


def simforge_asset_cfgs(cfg: "SimforgeAssetCfg") -> List[Any]:
    """Generate SimForge assets and return planner-visible prototype configs."""
    from isaaclab.sim import PreviewSurfaceCfg
    from simforge import ModelFileFormat
    from simforge.integrations.isaaclab.spawner.from_files import UsdFileCfg

    num_assets_base, num_assets_rem = divmod(cfg.num_assets, len(cfg.assets))
    filepaths = []
    has_material = []
    for i, asset in enumerate(cfg.assets):
        num_assets = num_assets_base + int(i < num_assets_rem)
        if num_assets == 0:
            continue  # Fewer variants than assets; SimForge requires a positive count.
        generator = asset.generator_type(
            num_assets=num_assets,
            seed=cfg.seed,
            file_format=ModelFileFormat.USDZ,
            use_cache=cfg.use_cache,
        )
        output = generator.generate_subprocess(asset, export_kwargs=cfg.export_kwargs)
        filepaths.extend(filepath for filepath, _metadata in output)
        has_material.extend([generator.BAKER.enabled] * len(output))

    spawn_kwargs = {
        attr_name: attr_value
        for attr_name, attr_value in cfg.__dict__.items()
        if attr_name not in _SIMFORGE_IGNORED_SPAWN_ATTRIBUTES
    }
    # Baked USD materials win; otherwise apply the configured material, falling
    # back to a seeded palette color so unbaked variants stay distinguishable.
    configured_material = spawn_kwargs.get("visual_material")
    palette = (
        seeded_color_palette_hue(len(filepaths), cfg.seed)
        if configured_material is None and not all(has_material)
        else None
    )
    proto_cfg = UsdFileCfg(**spawn_kwargs)

    return [
        proto_cfg.replace(  # type: ignore[attr-defined]
            usd_path=filepath.as_posix(),
            visual_material=(
                None
                if has_material[i]
                else configured_material
                if configured_material is not None
                else PreviewSurfaceCfg(diffuse_color=palette[i])  # type: ignore[index]
            ),
        )
        for i, filepath in enumerate(filepaths)
    ]


def _to_multi_asset_cfg(cfg: Any, assets_cfg: List[Any]) -> MultiAssetSpawnerCfg:
    """Build a planner-visible config; the clone planner assigns variants."""
    converted = MultiAssetSpawnerCfg(
        assets_cfg=assets_cfg,
        random_choice=False,
        activate_contact_sensors=getattr(cfg, "activate_contact_sensors", False),
    )
    converted.func = spawn_multi_asset_compat  # type: ignore[assignment]
    return converted


def simforge_to_multi_asset_cfg(cfg: "SimforgeAssetCfg") -> MultiAssetSpawnerCfg:
    """Convert a ``SimforgeAssetCfg`` to ``MultiAssetSpawnerCfg``."""
    return _to_multi_asset_cfg(cfg, simforge_asset_cfgs(cfg))


def multi_shape_to_multi_asset_cfg(cfg: "MultiShapeSpawnerCfg") -> MultiAssetSpawnerCfg:
    """Convert a ``MultiShapeSpawnerCfg`` to ``MultiAssetSpawnerCfg``."""
    from .shapes.multi_shape.impl import multi_shape_asset_cfgs

    return _to_multi_asset_cfg(cfg, multi_shape_asset_cfgs(cfg))


def maybe_convert_spawner(spawn: Any) -> Optional[MultiAssetSpawnerCfg]:
    """Convert a planner-invisible multi-variant spawner, if supported."""
    if (SimforgeAssetCfg := _simforge_asset_cfg_type()) is not None and isinstance(
        spawn, SimforgeAssetCfg
    ):
        return simforge_to_multi_asset_cfg(spawn)

    from .shapes.multi_shape.cfg import MultiShapeSpawnerCfg

    if isinstance(spawn, MultiShapeSpawnerCfg):
        return multi_shape_to_multi_asset_cfg(spawn)

    return None


# Bound the clone planner's Cartesian product by reducing SimForge variants
# before asset generation.
MAX_CLONE_PLAN_COMBINATIONS = 65536


def _simforge_asset_cfg_type() -> Optional[type]:
    try:
        from simforge.integrations.isaaclab.spawner.simforge_asset.cfg import (
            SimforgeAssetCfg,
        )

        return SimforgeAssetCfg
    except Exception:
        return None


def _is_env_scoped(prim_path: str) -> bool:
    # Mirrors the env-scope test of IsaacLab's clone planner (applied before
    # the {ENV_REGEX_NS} macro is resolved by the scene).
    return prim_path.startswith("{ENV_REGEX_NS}") or "/World/envs" in prim_path


# Upstream strategies SRB replaces with `PerEnvironmentCloneStrategy`. Isaac
# Lab `release/3.0.0` at `fdc650c8` defaults `CloneCfg.clone_strategy` to
# `sequential` and calls it as `(weights, num_worlds)` (`clone_plan.py`).
# `sequential` assigns contiguous runs of world prototypes and `random` draws
# them independently; both leave groups sharing variants.
_UPSTREAM_CLONE_STRATEGIES = frozenset({"random", "sequential"})


def _is_upstream_clone_strategy(strategy: Any) -> bool:
    module = getattr(strategy, "__module__", None) or ""
    return (
        module.startswith("isaaclab.cloner")
        and getattr(strategy, "__name__", None) in _UPSTREAM_CLONE_STRATEGIES
    )


def clone_group_sizes(scene_cfg: object) -> tuple[int, ...]:
    """Return clone group sizes in planner order: declared entities, expanded
    rigid-object collections, then sensors. Single-variant groups count as one."""
    from isaaclab.assets import RigidObjectCollectionCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.sensors import SensorBaseCfg
    from isaaclab.sim.spawners.wrappers import MultiUsdFileCfg

    scene_fields = InteractiveSceneCfg.__dataclass_fields__
    flat: list[Any] = []
    for name, cfg in vars(scene_cfg).items():
        if name in scene_fields or cfg is None:
            continue
        if isinstance(cfg, RigidObjectCollectionCfg):
            flat.extend(cfg.rigid_objects.values())
        else:
            flat.append(cfg)
    flat.sort(key=lambda cfg: isinstance(cfg, SensorBaseCfg))

    sizes = []
    for cfg in flat:
        spawn = getattr(cfg, "spawn", None)
        prim_path = getattr(cfg, "prim_path", None)
        if spawn is None or not isinstance(prim_path, str):
            continue
        if not _is_env_scoped(prim_path):
            continue
        if isinstance(spawn, MultiAssetSpawnerCfg):
            sizes.append(len(spawn.assets_cfg))
        elif isinstance(spawn, MultiUsdFileCfg) and not isinstance(spawn.usd_path, str):
            sizes.append(len(spawn.usd_path))
        else:
            sizes.append(1)
    return tuple(sizes)


def install_per_environment_clone_strategy(scene_cfg: object) -> None:
    """Assign variant ``i`` to environment ``i`` in the clone plan.

    Preserve caller strategies and ``clone_combinations``; shallow-copy the
    clone config before replacing its strategy.
    """
    clone_cfg = getattr(scene_cfg, "clone_cfg", None)
    if clone_cfg is None or not hasattr(clone_cfg, "clone_strategy"):
        return
    strategy = clone_cfg.clone_strategy
    if not isinstance(
        strategy, PerEnvironmentCloneStrategy
    ) and not _is_upstream_clone_strategy(strategy):
        return
    if getattr(clone_cfg, "clone_combinations", None):
        if isinstance(strategy, PerEnvironmentCloneStrategy):
            return
        logging.warning(
            "The scene declares clone_combinations; keeping the upstream "
            f"'{strategy.__name__}' clone strategy, which may assign the same "
            "procedural variant to several environments."
        )
        return
    clone_cfg = copy.copy(clone_cfg)
    clone_cfg.clone_strategy = PerEnvironmentCloneStrategy(clone_group_sizes(scene_cfg))
    scene_cfg.clone_cfg = clone_cfg  # type: ignore[attr-defined]


def convert_scene_multi_variant_spawners(scene_cfg: object) -> None:
    """Convert spawners, cap variants, and install a strategy before scene setup."""
    from collections.abc import Iterable, Mapping

    from isaaclab.assets import AssetBaseCfg, RigidObjectCollectionCfg
    from isaaclab.sim.spawners.wrappers import MultiUsdFileCfg

    from srb.utils.config_introspection import iter_config_values

    from .shapes.multi_shape.cfg import MultiShapeSpawnerCfg
    from .shapes.multi_shape.impl import DEFAULT_SHAPES

    simforge_cfg_type = _simforge_asset_cfg_type()
    entities: list[Any] = []

    def _recursive_collect(attr: Any) -> None:
        if isinstance(attr, AssetBaseCfg):
            if getattr(attr, "spawn", None) is not None:
                entities.append(attr)
        elif isinstance(attr, RigidObjectCollectionCfg):
            for rigid_object_cfg in attr.rigid_objects.values():
                if getattr(rigid_object_cfg, "spawn", None) is not None:
                    entities.append(rigid_object_cfg)
        elif isinstance(attr, Mapping):
            for item in attr.values():
                _recursive_collect(item)
        elif isinstance(attr, Iterable) and not isinstance(attr, (str, bytes)):
            for item in attr:
                _recursive_collect(item)

    for value in iter_config_values(scene_cfg):
        _recursive_collect(value)

    # Count groups whose variant counts cannot be reduced here.
    fixed_product = 1
    for entity in entities:
        if not _is_env_scoped(entity.prim_path):
            continue
        spawn = entity.spawn
        if isinstance(spawn, MultiAssetSpawnerCfg):
            fixed_product *= max(1, len(spawn.assets_cfg))
        elif isinstance(spawn, MultiUsdFileCfg) and not isinstance(spawn.usd_path, str):
            fixed_product *= max(1, len(spawn.usd_path))
        elif isinstance(spawn, MultiShapeSpawnerCfg):
            fixed_product *= max(1, len(spawn.shapes or DEFAULT_SHAPES))
    # Only env-scoped SimForge groups can be reduced before generation.
    shrinkable = [
        entity.spawn
        for entity in entities
        if simforge_cfg_type is not None
        and isinstance(entity.spawn, simforge_cfg_type)
        and _is_env_scoped(entity.prim_path)
        and entity.spawn.num_assets > 1
    ]
    if fixed_product > MAX_CLONE_PLAN_COMBINATIONS:
        logging.warning(
            f"The scene contains multi-asset spawners with {fixed_product} "
            "variant combinations, exceeding the clone-plan budget of "
            f"{MAX_CLONE_PLAN_COMBINATIONS}. Scene creation may exhaust memory."
        )
    # Apply the cap even when fixed groups already exceed the budget; in that
    # case shrinkable groups collapse to one variant.
    if shrinkable:
        budget = max(1, MAX_CLONE_PLAN_COMBINATIONS // fixed_product)
        # Correct floating-point rounding below an exact integer root.
        max_variants = max(1, int(budget ** (1.0 / len(shrinkable)) + 1e-9))
        while max_variants ** len(shrinkable) > budget:
            max_variants -= 1
        max_variants = max(1, max_variants)
        capped = [s for s in shrinkable if s.num_assets > max_variants]
        if capped:
            logging.warning(
                f"Capping the variant count of {len(capped)} procedural "
                f"spawner(s) to {max_variants} (from "
                f"{', '.join(str(s.num_assets) for s in capped)}) to keep the "
                f"clone plan within {MAX_CLONE_PLAN_COMBINATIONS} variant "
                "combinations. Set env cfg `n_procgen_variants` to control "
                "the variant count per asset explicitly."
            )
            for spawn in capped:
                spawn.num_assets = max_variants

    for entity in entities:
        converted = maybe_convert_spawner(entity.spawn)
        if converted is not None:
            entity.spawn = converted
        elif isinstance(entity.spawn, MultiAssetSpawnerCfg):
            # Global configs may also be spawned outside a planned scene.
            entity.spawn.func = spawn_multi_asset_compat  # type: ignore[assignment]

    install_per_environment_clone_strategy(scene_cfg)
