"""MDP consumers of Isaac-free illumination products.

Provides masked reset spawning and raster sampling for rewards or observations.
Isaac-only imports stay inside the functions that need them.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import torch

from srb.core.ephemeris.products import PSR_NODATA, IlluminationProduct, load_product

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.manager import SceneEntityCfg

# path -> (mtime_ns, product); a changed file invalidates the cached product.
_PRODUCT_CACHE: dict[str, tuple[int, IlluminationProduct]] = {}

_REMEDIATION_HINT = (
    "no illumination product resolved -- generate one via `srb ephemeris import`"
)


def resolve_illumination_product(
    env: "AnyEnv", product_path: str | None = None
) -> IlluminationProduct:
    """Resolve and load a product from the override or environment config.

    Raises ``ValueError`` with the import command when no path resolves.
    """
    path = product_path
    if path is None:
        unwrapped = getattr(env, "unwrapped", env)
        cfg = getattr(unwrapped, "cfg", None)
        scenery = getattr(cfg, "_scenery", None)
        path = getattr(scenery, "illumination_product_path", None)
    if path is None:
        raise ValueError(_REMEDIATION_HINT)

    path = str(path)
    try:
        mtime_ns = Path(path).stat().st_mtime_ns
    except OSError as exc:
        raise ValueError(f"illumination product not found at {path!r}") from exc
    cached = _PRODUCT_CACHE.get(path)
    if cached is not None and cached[0] == mtime_ns:
        return cached[1]
    product = load_product(path)
    _PRODUCT_CACHE[path] = (mtime_ns, product)
    return product


def _get_illumination_rng(env: "AnyEnv") -> np.random.Generator:
    rng = getattr(env, "_illumination_rng", None)
    if rng is None:
        rng = np.random.default_rng(env.cfg.seed)
        env._illumination_rng = rng
    return rng


def _candidate_mask(
    product: IlluminationProduct, mask: str, threshold: float, invert: bool
) -> np.ndarray:
    """Boolean ``(H, W)`` candidate-cell mask (masked-spawn semantics 1)."""
    if mask == "psr":
        arr = product.psr
        if arr is None:
            raise ValueError("product carries no 'psr' raster")
        valid = arr != PSR_NODATA
        selected = valid & (arr == 1)
    elif mask == "lit_fraction":
        arr = product.lit_fraction
        if arr is None:
            raise ValueError("product carries no 'lit_fraction' raster")
        valid = np.isfinite(arr)
        selected = valid & (arr >= threshold)
    else:
        raise ValueError(f"unknown mask {mask!r}; expected 'psr' or 'lit_fraction'")

    if invert:
        # Complement WITHIN valid cells -- nodata/NaN cells stay excluded
        # either way.
        selected = valid & ~selected

    if not np.any(selected):
        raise ValueError(
            f"no candidate cells for mask={mask!r} threshold={threshold} "
            f"invert={invert}"
        )
    return selected


def _sample_masked_positions(
    product: IlluminationProduct,
    mask: str,
    threshold: float,
    invert: bool,
    rng: np.random.Generator,
    n: int,
) -> np.ndarray:
    """Return ``n`` patch-local ``(x, y)`` samples in meters.

    Choose candidate cells uniformly, then jitter each axis by half a pixel.
    """
    candidate = _candidate_mask(product, mask, threshold, invert)
    rows, cols = np.nonzero(candidate)

    idx = rng.integers(0, rows.size, size=n)
    chosen_rows = rows[idx].astype(np.float64)
    chosen_cols = cols[idx].astype(np.float64)

    x0, y0 = product.grid.rowcol_to_local_xy(chosen_rows, chosen_cols)
    gsd = product.grid.gsd_m
    jitter_x = rng.uniform(-0.5 * gsd, 0.5 * gsd, size=n)
    jitter_y = rng.uniform(-0.5 * gsd, 0.5 * gsd, size=n)

    return np.stack([x0 + jitter_x, y0 + jitter_y], axis=-1)


def reset_root_state_uniform_illumination_masked(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    *,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]] | None,
    asset_cfg: "SceneEntityCfg",
    mask: str = "psr",
    threshold: float = 0.5,
    invert: bool = False,
    product_path: str | None = None,
) -> None:
    """Spawn an asset uniformly over candidate product cells on reset.

    Product cells determine x/y; ``pose_range`` supplies z and orientation,
    while ``velocity_range`` supplies the six velocity components.
    """
    from srb.core.scene_introspection import get_scene_root_asset
    from srb.utils.math import quat_from_euler_xyz, quat_mul
    from srb.utils.sampling import sample_uniform

    asset = get_scene_root_asset(env.scene, asset_cfg.name)
    n = len(env_ids)
    root_states = asset.data.default_root_state[env_ids].clone()

    rng = _get_illumination_rng(env)
    product = resolve_illumination_product(env, product_path)
    xy_local_np = _sample_masked_positions(product, mask, threshold, invert, rng, n)
    xy_local = torch.as_tensor(xy_local_np, dtype=root_states.dtype, device=env.device)

    zrpy_range = [
        pose_range.get(key, (0.0, 0.0)) for key in ["z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(zrpy_range, dtype=torch.float32, device=env.device)
    rand_zrpy = sample_uniform(ranges[:, 0], ranges[:, 1], (n, 4), device=env.device)

    env_origins_xyz = env.scene.env_origins[env_ids]

    positions = root_states[:, 0:3].clone()
    positions[:, 0:2] = xy_local + env_origins_xyz[:, 0:2]
    positions[:, 2] = root_states[:, 2] + env_origins_xyz[:, 2] + rand_zrpy[:, 0]

    orientations_delta = quat_from_euler_xyz(
        rand_zrpy[:, 1], rand_zrpy[:, 2], rand_zrpy[:, 3]
    )
    orientations = quat_mul(root_states[:, 3:7], orientations_delta)

    asset.write_root_pose_to_sim(
        torch.cat([positions, orientations], dim=-1),
        env_ids=env_ids,  # type: ignore[misc]
    )

    if velocity_range is not None:
        vel_range_list = [
            velocity_range.get(key, (0.0, 0.0))
            for key in ["x", "y", "z", "roll", "pitch", "yaw"]
        ]
        vel_ranges = torch.tensor(
            vel_range_list, dtype=torch.float32, device=env.device
        )
        velocities = root_states[:, 7:13] + sample_uniform(
            vel_ranges[:, 0], vel_ranges[:, 1], (n, 6), device=env.device
        )
        asset.write_root_velocity_to_sim(velocities, env_ids=env_ids)  # type: ignore[misc]


#: Float rasters only: the uint8 ``psr`` mask uses 255 for nodata, not NaN.
_FRACTION_KEYS = ("lit_fraction", "max_sun_el")


def illumination_fraction_at(
    env: "AnyEnv",
    positions_w: torch.Tensor,
    *,
    key: str = "lit_fraction",
    product_path: str | None = None,
    env_ids: torch.Tensor | Sequence[int] | None = None,
) -> torch.Tensor:
    """Sample a float raster at world positions, mapping NaN to zero.

    Row ``i`` of ``positions_w`` belongs to environment ``env_ids[i]`` (all
    environments when ``None``); its origin is subtracted to reach the
    patch-local raster frame. Returns an ``(N,)`` tensor matching the input
    device and dtype. Supported keys are ``lit_fraction`` and ``max_sun_el``;
    use ``sample_static`` for the uint8 ``psr`` mask.
    """
    if key not in _FRACTION_KEYS:
        raise ValueError(
            f"illumination_fraction_at only supports float rasters "
            f"{_FRACTION_KEYS}, not {key!r}; sample it via "
            f"IlluminationProduct.sample_static({key!r}, ...) directly"
        )

    product = resolve_illumination_product(env, product_path)

    n = positions_w.shape[0]
    origins_xy = env.scene.env_origins[:, 0:2]
    if env_ids is not None:
        origins_xy = origins_xy[
            torch.as_tensor(env_ids, dtype=torch.long, device=origins_xy.device)
        ]
    if origins_xy.shape[0] != n:
        raise ValueError(
            f"positions_w has {n} rows but {origins_xy.shape[0]} env origins are "
            "selected; pass env_ids for a subset of environments"
        )
    origins_xy = origins_xy.to(device=positions_w.device, dtype=positions_w.dtype)

    local_xy = positions_w[:, 0:2] - origins_xy
    x_np = local_xy[:, 0].detach().cpu().numpy()
    y_np = local_xy[:, 1].detach().cpu().numpy()

    values = np.asarray(product.sample_static(key, x_np, y_np), dtype=np.float64)
    values = np.nan_to_num(values, nan=0.0)

    return torch.as_tensor(values, dtype=positions_w.dtype, device=positions_w.device)
