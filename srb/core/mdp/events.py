import logging
import re
from typing import TYPE_CHECKING, Any, Sequence

import numpy as np  # noqa: F401
import torch
import torch.nn.functional as F
from pxr import Gf, PhysxSchema, Usd, UsdPhysics

from srb.core.asset import (  # DeformableObject,
    Articulation,
    AssetBase,
    RigidObject,
    RigidObjectCollection,
)
from srb.core.ephemeris import EphemerisCfg, SiteSpec
from srb.core.ephemeris.env_support import (
    kernels_present,  # noqa: F401
    orbit_sun_light_params,  # noqa: F401
    query_bodies,  # noqa: F401
    sample_epoch,  # noqa: F401
    sample_epoch_windows,  # noqa: F401
    shift_epoch,  # noqa: F401
    surface_occluder_body,  # noqa: F401
    surface_sun_light_params,  # noqa: F401
    warn_eclipse_occluder_missing_once,  # noqa: F401
)
from srb.core.ephemeris.orbit import OrbitSpec
from srb.core.manager import SceneEntityCfg
from srb.core.scene_introspection import (
    find_scene_entity,  # noqa: F401
    get_scene_entity,
    get_scene_rigid_object_collection,
    get_scene_root_asset,
    iter_scene_articulations,
    iter_scene_rigid_object_collections,
    iter_scene_rigid_objects,
    iter_scene_root_assets,
    replace_scene_extra,
)
from srb.core.sim.prim_utils import get_current_stage
from srb.utils.math import quat_from_angle_axis, quat_from_euler_xyz, quat_mul, slerp
from srb.utils.poisson_disk import sample_poisson_disk
from srb.utils.sampling import sample_uniform
from srb.utils.usd import safe_set_attribute_on_usd_prim

from .ephemeris_events import (
    _EMITTED_EPHEMERIS_ECLIPSE_NO_OCCLUDER_WARNINGS,  # noqa: F401
    _EMITTED_EPHEMERIS_RESET_DARK_EPISODE_WARNINGS,  # noqa: F401
    _EMITTED_EPHEMERIS_RESET_KERNELS_MISSING_WARNINGS,  # noqa: F401
)
from .ephemeris_events import (
    _apply_sun_state as _ephemeris_apply_sun_state,
)
from .ephemeris_events import (
    _resolve_sun_event_context as _ephemeris_resolve_sun_event_context,
)
from .ephemeris_events import (
    _sim_time as _ephemeris_sim_time,
)
from .ephemeris_events import (
    _warn_ephemeris_eclipse_no_occluder_once as _ephemeris_warn_eclipse_no_occluder,
)
from .ephemeris_events import (
    _warn_ephemeris_reset_dark_episode_once as _ephemeris_warn_dark_episode,
)
from .ephemeris_events import (
    _warn_ephemeris_reset_kernels_missing_once as _ephemeris_warn_kernels_missing,
)
from .ephemeris_events import (
    apply_skydome_from_ephemeris as apply_skydome_from_ephemeris,
)
from .ephemeris_events import (
    apply_sun_ephemeris_fixed as apply_sun_ephemeris_fixed,
)
from .ephemeris_events import (
    progress_sun_ephemeris as _ephemeris_progress,
)
from .ephemeris_events import (
    randomize_sun_ephemeris as randomize_sun_ephemeris,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def _xform_view_count(asset: Any) -> int:
    prim_paths = getattr(asset, "prim_paths", None)
    if prim_paths is not None:
        return len(prim_paths)
    prims = getattr(asset, "prims", None)
    if prims is not None:
        return len(prims)
    count = getattr(asset, "count", None)
    if count is not None:
        return int(count)
    num_instances = getattr(asset, "num_instances", None)
    if num_instances is not None:
        return int(num_instances)
    return 0


def _resolve_xform_view(env: "AnyEnv", asset_cfg: SceneEntityCfg) -> Any:
    asset: Any = get_scene_entity(env.scene, asset_cfg.name)
    if _xform_view_count(asset) > 0:
        return asset

    scene_cfg = getattr(env.cfg, "scene", None)
    cfg_asset = getattr(scene_cfg, asset_cfg.name, None)
    prim_path = getattr(cfg_asset, "prim_path", None)
    if not prim_path:
        return asset

    # Rebuild a view emptied before cloning created its prims; SRB's view takes
    # torch indices (Isaac Lab's FrameView expects Warp arrays).
    from srb.core.static_asset_view import XformPrimView

    rebound_asset = XformPrimView(
        prim_path,
        device=str(getattr(env.scene, "device", env.device)),
        name=asset_cfg.name,
    )
    if _xform_view_count(rebound_asset) == 0:
        return asset

    replace_scene_extra(env.scene, asset_cfg.name, rebound_asset)
    return rebound_asset


def reset_scene_to_default(env: "AnyEnv", env_ids: torch.Tensor | None):
    reset_rigid_objects_default(env, env_ids)
    reset_articulations_default(env, env_ids)
    reset_deformable_objects_default(env, env_ids)


def reset_rigid_objects_default(env: "AnyEnv", env_ids: torch.Tensor | None):
    stage = get_current_stage()
    state_ids = slice(None) if env_ids is None else env_ids
    for _, rigid_object in iter_scene_rigid_objects(env.scene):
        default_root_state = rigid_object.data.default_root_state[state_ids].clone()
        default_root_state[:, 0:3] += env.scene.env_origins[state_ids]
        rigid_object.write_root_pose_to_sim(
            default_root_state[:, :7],
            env_ids=env_ids,  # type: ignore[misc]
        )
        if not _is_rigid_object_kinematic(stage, rigid_object):
            rigid_object.write_root_velocity_to_sim(
                default_root_state[:, 7:],
                env_ids=env_ids,  # type: ignore[misc]
            )


def _is_rigid_object_kinematic(stage: Usd.Stage, rigid_object: RigidObject) -> bool:
    kinematic_attr = UsdPhysics.RigidBodyAPI(
        stage.GetPrimAtPath(rigid_object.root_physx_view.prim_paths[0])
    ).GetKinematicEnabledAttr()
    return kinematic_attr.Get() if kinematic_attr else False


def reset_articulations_default(env: "AnyEnv", env_ids: torch.Tensor | None):
    state_ids = slice(None) if env_ids is None else env_ids
    for _, articulation_asset in iter_scene_articulations(env.scene):
        default_root_state = articulation_asset.data.default_root_state[
            state_ids
        ].clone()
        default_root_state[:, 0:3] += env.scene.env_origins[state_ids]
        articulation_asset.write_root_pose_to_sim(
            default_root_state[:, :7],
            env_ids=env_ids,  # type: ignore[misc]
        )
        articulation_asset.write_root_velocity_to_sim(
            default_root_state[:, 7:],
            env_ids=env_ids,  # type: ignore[misc]
        )
        default_joint_pos = articulation_asset.data.default_joint_pos[state_ids].clone()
        default_joint_vel = articulation_asset.data.default_joint_vel[state_ids].clone()
        articulation_asset.write_joint_state_to_sim(
            default_joint_pos,
            default_joint_vel,
            env_ids=env_ids,  # type: ignore[misc]
        )


def reset_deformable_objects_default(env: "AnyEnv", env_ids: torch.Tensor | None):
    state_ids = slice(None) if env_ids is None else env_ids
    for deformable_object in env.scene.deformable_objects.values():
        nodal_state = deformable_object.data.default_nodal_state_w[state_ids].clone()
        deformable_object.write_nodal_state_to_sim(nodal_state, env_ids=env_ids)  # type: ignore[misc]


def randomize_pose(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
    pose_range: dict[str, tuple[float, float]],
):
    _env: "AnyEnv" = env.unwrapped  # type: ignore[misc]
    if env_ids is None:
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )

    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=_env.device)
    rand_samples = sample_uniform(
        ranges[:, 0],
        ranges[:, 1],
        (len(env_ids), 6),
        device=_env.device,
    )
    positions = env.scene.env_origins[env_ids] + rand_samples[:, 0:3]
    orientations = quat_from_euler_xyz(
        rand_samples[:, 3], rand_samples[:, 4], rand_samples[:, 5]
    )

    pose_attr = getattr(env, env_attr_name)
    pose_attr[env_ids] = torch.cat([positions, orientations], dim=-1)


def randomize_pos(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
    pos_range: dict[str, tuple[float, float]],
):
    _env: "AnyEnv" = env.unwrapped  # type: ignore[misc]
    if env_ids is None:
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )

    range_list = [pos_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z"]]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=_env.device)
    rand_samples = sample_uniform(
        ranges[:, 0],
        ranges[:, 1],
        (len(env_ids), 3),
        device=_env.device,
    )
    positions = env.scene.env_origins[env_ids] + rand_samples[:, 0:3]

    pos_attr = getattr(env, env_attr_name)
    pos_attr[env_ids] = positions


def offset_pose_natural(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
    pos_axes: Sequence[str],
    pos_step_range: tuple[float, float],
    pos_smoothness: float,
    pos_step_smoothness: float,
    pos_bounds: dict[str, tuple[float, float]],
    orient_yaw_only: bool,
    orient_smoothness: float,
):
    """Apply smoothed random movement to selected pose components."""
    _env: "AnyEnv" = env.unwrapped  # type: ignore[misc]
    if env_ids is None:
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )

    pos_state_key = f"__{env_attr_name}_natural_movement_velocities"
    if not hasattr(_env, pos_state_key):
        pos_velocities = torch.randn((_env.num_envs, 3), device=_env.device)
        setattr(_env, pos_state_key, pos_velocities)

    pos_velocities = getattr(_env, pos_state_key)[env_ids]
    pos_random_direction = torch.randn_like(pos_velocities)
    pos_velocities = F.normalize(
        pos_smoothness * pos_velocities + (1 - pos_smoothness) * pos_random_direction,
        p=2,
        dim=1,
    )

    step_size_state_key = f"__{env_attr_name}_natural_movement_step_sizes"
    if not hasattr(_env, step_size_state_key):
        pos_step_sizes = sample_uniform(
            pos_step_range[0], pos_step_range[1], (_env.num_envs,), device=_env.device
        )
        setattr(_env, step_size_state_key, pos_step_sizes)

    last_pos_step_sizes = getattr(_env, step_size_state_key)[env_ids]
    new_random_step = sample_uniform(
        pos_step_range[0], pos_step_range[1], (len(env_ids),), device=_env.device
    )
    pos_step_sizes = (
        pos_step_smoothness * last_pos_step_sizes
        + (1 - pos_step_smoothness) * new_random_step
    )

    delta_pos = pos_velocities * pos_step_sizes.unsqueeze(1)
    getattr(_env, pos_state_key)[env_ids] = pos_velocities
    getattr(_env, step_size_state_key)[env_ids] = pos_step_sizes

    pose_attr = getattr(env, env_attr_name)
    current_pose = pose_attr[env_ids].clone()
    current_positions = current_pose[:, :3]

    orient_state_key = f"__{env_attr_name}_natural_movement_orientations"
    if not hasattr(_env, orient_state_key):
        # Keep full-env indexing valid for subset calls and per-env resets.
        setattr(_env, orient_state_key, getattr(env, env_attr_name)[:, 3:7].clone())

    last_orientations = getattr(_env, orient_state_key)[env_ids]

    active_pos_velocities = pos_velocities.clone()
    pos_axis_indices = {"x": 0, "y": 1, "z": 2}
    inactive_pos_axes = [
        axis_idx for axis, axis_idx in pos_axis_indices.items() if axis not in pos_axes
    ]
    for axis_idx in inactive_pos_axes:
        active_pos_velocities[:, axis_idx] = 0.0

    move_direction = F.normalize(active_pos_velocities, p=2, dim=1)

    if orient_yaw_only:
        yaw_vec = move_direction.clone()
        yaw_vec[:, 2] = 0.0
        # Purely vertical movement has no yaw direction.
        is_zero_mask = torch.all(yaw_vec == 0, dim=1)
        yaw_vec[is_zero_mask] = torch.tensor([1.0, 0.0, 0.0], device=_env.device)
        yaw_vec = F.normalize(yaw_vec, p=2, dim=1)

        yaw_angle = torch.atan2(yaw_vec[:, 1], yaw_vec[:, 0])
        target_orientations = quat_from_euler_xyz(
            torch.zeros_like(yaw_angle), torch.zeros_like(yaw_angle), yaw_angle
        )
    else:
        forward_vec = torch.tensor(
            [1.0, 0.0, 0.0], device=_env.device, dtype=torch.float32
        ).expand_as(move_direction)
        axis = torch.cross(forward_vec, move_direction, dim=1)
        dot = torch.sum(forward_vec * move_direction, dim=1).clamp(-1.0, 1.0)
        angle = torch.acos(dot)
        axis_norm = torch.norm(axis, dim=1, keepdim=True)
        is_degenerate = axis_norm.squeeze(-1) < 1e-6
        fallback_axis = torch.tensor(
            [0.0, 1.0, 0.0], device=_env.device, dtype=torch.float32
        ).expand_as(axis)
        axis = torch.where(is_degenerate.unsqueeze(-1), fallback_axis, axis)
        target_orientations = quat_from_angle_axis(angle, axis)

    new_orientations = slerp(
        last_orientations, target_orientations, 1.0 - orient_smoothness
    )
    getattr(_env, orient_state_key)[env_ids] = new_orientations.clone()

    pos_axis_indices = {"x": 0, "y": 1, "z": 2}
    pos_active_axes = [
        pos_axis_indices[axis] for axis in pos_axes if axis in pos_axis_indices
    ]
    new_positions = current_positions.clone()
    for axis_idx in pos_active_axes:
        new_positions[:, axis_idx] += delta_pos[:, axis_idx]

    # Reflect at bounds and reverse the corresponding velocity component.
    for axis in pos_axes:
        if axis in pos_bounds:
            axis_idx = pos_axis_indices[axis]
            min_bound, max_bound = pos_bounds[axis]
            actual_min = min_bound + env.scene.env_origins[env_ids, axis_idx]
            actual_max = max_bound + env.scene.env_origins[env_ids, axis_idx]
            below_min = new_positions[:, axis_idx] < actual_min
            above_max = new_positions[:, axis_idx] > actual_max
            if below_min.any():
                new_positions[below_min, axis_idx] = (
                    2 * actual_min[below_min] - new_positions[below_min, axis_idx]
                )
                pos_velocities[below_min, axis_idx] *= -1.0
            if above_max.any():
                new_positions[above_max, axis_idx] = (
                    2 * actual_max[above_max] - new_positions[above_max, axis_idx]
                )
                pos_velocities[above_max, axis_idx] *= -1.0
    getattr(_env, pos_state_key)[env_ids] = pos_velocities

    pose_attr[env_ids] = torch.cat([new_positions, new_orientations], dim=-1)


def reset_pose_natural(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
) -> None:
    """Reset ``offset_pose_natural`` state for selected envs after pose reset.

    Call after resetting ``env_attr_name``. This prevents drift and orientation
    state from carrying across episodes; it is a no-op before natural movement runs.
    """
    _env: "AnyEnv" = env.unwrapped  # type: ignore[misc]
    if env_ids is None:
        env_ids = torch.arange(_env.num_envs, device=_env.device)

    vel_key = f"__{env_attr_name}_natural_movement_velocities"
    if hasattr(_env, vel_key):
        getattr(_env, vel_key)[env_ids] = torch.randn(
            (len(env_ids), 3), device=_env.device
        )

    step_key = f"__{env_attr_name}_natural_movement_step_sizes"
    if hasattr(_env, step_key):
        getattr(_env, step_key)[env_ids] = 0.0

    orient_key = f"__{env_attr_name}_natural_movement_orientations"
    if hasattr(_env, orient_key):
        pose_attr = getattr(_env, env_attr_name, None)
        if pose_attr is not None and pose_attr.shape[-1] >= 7:
            getattr(_env, orient_key)[env_ids] = pose_attr[env_ids, 3:7]


def randomize_command(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
    magnitude: float | Sequence[float] = 1.0,
):
    """Sample commands in ``[-magnitude, magnitude]``, scalar or per component."""
    _env: "AnyEnv" = env.unwrapped  # type: ignore[misc]
    if env_ids is None:
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )
    cmd_attr = getattr(env, env_attr_name)
    bound = torch.as_tensor(magnitude, dtype=cmd_attr.dtype, device=_env.device)
    cmd_attr[env_ids] = sample_uniform(
        -bound,
        bound,
        (len(env_ids), *cmd_attr.shape[1:]),
        device=_env.device,
    )


def release_assembly_root_joins_on_action(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    assembly_key: str,
    env_joint_assemblies_attr_name: str = "joint_assemblies",
    env_action_manager_attr_name: str = "action_manager",
    action_idx: int = 0,
    cmp_op: str = ">",
    cmp_value: float = 0.0,
):
    if env_ids is None:
        _env: "AnyEnv" = env.unwrapped  # type: ignore[misc]
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )

    joint_assemblies = getattr(env, env_joint_assemblies_attr_name)[assembly_key]
    actions = getattr(env, env_action_manager_attr_name).action[env_ids, action_idx]

    import operator

    _CMP_OPS = {
        ">": operator.gt,
        ">=": operator.ge,
        "<": operator.lt,
        "<=": operator.le,
        "==": operator.eq,
        "!=": operator.ne,
    }
    cmp_fn = _CMP_OPS.get(cmp_op)
    if cmp_fn is None:
        raise ValueError(
            f"Unsupported comparison operator: '{cmp_op}'. "
            f"Supported: {list(_CMP_OPS.keys())}"
        )

    env_indices = env_ids.detach().cpu().tolist()
    for env_index, action in zip(env_indices, actions, strict=True):
        joint_assemblies[env_index].set_attach_path_root_joints_enabled(
            bool(cmp_fn(action.item(), cmp_value))
        )


def reset_xform_positions_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    position_distribution_params: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg,
):
    """Randomize XForm position globally or per environment."""
    asset: Any = _resolve_xform_view(env, asset_cfg)

    # ``count`` can diverge from the prim list when env_spacing=0 collapses clones.
    num_prims = _xform_view_count(asset)
    if num_prims == 0:
        return
    is_global = num_prims == 1

    valid_ids = env_ids[env_ids < num_prims] if not is_global else env_ids
    num_samples = 1 if is_global else len(valid_ids)

    range_list = [
        position_distribution_params.get(key, (0.0, 0.0)) for key in ["x", "y", "z"]
    ]
    ranges = torch.tensor(range_list, device=asset._device)
    rand_samples = sample_uniform(
        ranges[:, 0], ranges[:, 1], (num_samples, 3), device=asset._device
    )

    if is_global:
        origin = (
            env.scene.env_origins[env_ids[:1]]
            if len(env_ids) > 0
            else torch.zeros(1, 3, device=asset._device)
        )
        positions = origin + rand_samples
        asset.set_world_poses(positions=positions)
    else:
        positions = env.scene.env_origins[valid_ids] + rand_samples
        asset.set_world_poses(positions=positions, indices=valid_ids)


def reset_xform_orientation_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    orientation_distribution_params: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg,
):
    """Randomize XForm orientation globally or per environment."""
    asset: Any = _resolve_xform_view(env, asset_cfg)

    num_prims = _xform_view_count(asset)
    if num_prims == 0:
        return
    is_global = num_prims == 1

    valid_ids = env_ids[env_ids < num_prims] if not is_global else env_ids
    num_samples = 1 if is_global else len(valid_ids)

    range_list = [
        orientation_distribution_params.get(key, (0.0, 0.0))
        for key in ["roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, device=asset._device)

    rand_samples = sample_uniform(
        ranges[:, 0], ranges[:, 1], (num_samples, 3), device=asset._device
    )

    orientations = quat_from_euler_xyz(
        rand_samples[:, 0], rand_samples[:, 1], rand_samples[:, 2]
    )

    if is_global:
        asset.set_world_poses(orientations=orientations)
    else:
        asset.set_world_poses(orientations=orientations, indices=valid_ids)


def randomize_usd_prim_attribute_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    attr_name: str,
    distribution_params: tuple[float | Sequence[float], float | Sequence[float]],
    asset_cfg: SceneEntityCfg,
):
    asset: Any = get_scene_entity(env.scene, asset_cfg.name)
    if isinstance(distribution_params[0], Sequence):
        dist_len = len(distribution_params[0])
        distribution_params = (  # type: ignore[misc]
            torch.tensor(distribution_params[0]),
            torch.tensor(distribution_params[1]),
        )
    else:
        dist_len = 1
    for i, prim in enumerate(asset.prims):
        if env_ids is not None and i not in env_ids:
            continue
        value = sample_uniform(
            distribution_params[0],  # type: ignore[misc]
            distribution_params[1],  # type: ignore[misc]
            (dist_len,),
            device="cpu",
        )
        value = value.item() if dist_len == 1 else value.tolist()
        safe_set_attribute_on_usd_prim(
            prim, f"inputs:{attr_name}", value, camel_case=True
        )


def _warn_ephemeris_reset_kernels_missing_once(body: str, term_name: str) -> None:
    return _ephemeris_warn_kernels_missing(body, term_name)


def _warn_ephemeris_eclipse_no_occluder_once(body: str) -> None:
    return _ephemeris_warn_eclipse_no_occluder(body)


def _warn_ephemeris_reset_dark_episode_once(
    site: SiteSpec, elevation_deg: float
) -> None:
    return _ephemeris_warn_dark_episode(site, elevation_deg)


def _sim_time(env: "AnyEnv") -> float:
    return _ephemeris_sim_time(env)


def _resolve_sun_event_context(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    *,
    ephemeris_cfg: EphemerisCfg,
    site: SiteSpec,
    asset_cfg: SceneEntityCfg | None,
    term_name: str = "randomize_sun_ephemeris",
) -> tuple["AnyEnv", Any] | None:
    return _ephemeris_resolve_sun_event_context(
        env,
        env_ids,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        asset_cfg=asset_cfg,
        term_name=term_name,
    )


def _apply_sun_state(
    env: "AnyEnv",
    *,
    ephemeris_cfg: EphemerisCfg,
    site: SiteSpec,
    orbit: OrbitSpec | None,
    state,
    asset: Any,
    dt_s: float,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
) -> None:
    return _ephemeris_apply_sun_state(
        env,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        orbit=orbit,
        state=state,
        asset=asset,
        dt_s=dt_s,
        domain_light_intensity=domain_light_intensity,
        domain_light_angular_diameter=domain_light_angular_diameter,
    )


def progress_sun_ephemeris(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    *,
    ephemeris_cfg: EphemerisCfg,
    site: SiteSpec,
    orbit: OrbitSpec | None = None,
    asset_cfg: SceneEntityCfg | None = None,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
) -> None:
    return _ephemeris_progress(
        env,
        env_ids,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        orbit=orbit,
        asset_cfg=asset_cfg,
        domain_light_intensity=domain_light_intensity,
        domain_light_angular_diameter=domain_light_angular_diameter,
    )


def randomize_gravity_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    distribution_params: tuple[tuple[float, float, float], tuple[float, float, float]],
):
    # Resolve the physics scene from the stage; SimulationContext does not cache it.
    physics_scene = UsdPhysics.Scene.Get(env.sim.stage, env.sim.cfg.physics_prim_path)
    gravity = sample_uniform(
        torch.tensor(distribution_params[0]),
        torch.tensor(distribution_params[1]),
        (3,),
        device="cpu",
    )
    gravity_magnitude = torch.norm(gravity)
    if gravity_magnitude == 0.0:
        gravity_direction = torch.tensor([0.0, 0.0, -1.0], device=gravity.device)
    else:
        gravity_direction = gravity / gravity_magnitude

    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(*gravity_direction.tolist()))
    physics_scene.CreateGravityMagnitudeAttr(gravity_magnitude.item())


def randomize_external_force_torque_bernoulli(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    *,
    force_range: tuple[float, float],
    torque_range: tuple[float, float],
    probability: float,
    asset_cfg: SceneEntityCfg | None = None,
):
    """Apply a probabilistic one-interval wrench, zeroing inactive environments."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")

    if not 0.0 <= probability <= 1.0:
        raise ValueError("probability must be in [0, 1]")

    asset: RigidObject | Articulation = get_scene_root_asset(
        env.scene,
        asset_cfg.name,
    )
    if env_ids is None:
        env_ids = torch.arange(
            env.scene.num_envs,
            device=asset.device,
            dtype=torch.int32,
        )
    else:
        env_ids = torch.as_tensor(env_ids, device=asset.device, dtype=torch.int32)

    # Resolved body ids are tensors; only a slice means every body.
    num_bodies = (
        asset.num_bodies
        if isinstance(asset_cfg.body_ids, slice)
        else len(asset_cfg.body_ids)
    )
    shape = (len(env_ids), num_bodies, 3)
    forces = torch.empty(shape, device=asset.device).uniform_(*force_range)
    torques = torch.empty(shape, device=asset.device).uniform_(*torque_range)
    active = torch.rand(
        (len(env_ids), 1, 1),
        device=asset.device,
    ) < float(probability)
    forces.mul_(active)
    torques.mul_(active)

    composer = getattr(asset, "permanent_wrench_composer", None)
    if composer is not None:
        composer.set_forces_and_torques_index(
            forces=forces,
            torques=torques,
            body_ids=asset_cfg.body_ids,
            env_ids=env_ids,
        )
    else:
        asset.set_external_force_and_torque(
            forces,
            torques,
            body_ids=asset_cfg.body_ids,
            env_ids=env_ids,
        )


_ENV_INDEX_RE = re.compile(r"/env_(\d+)(?:/|$)")


def _env_index_from_prim_path(path_str: str) -> int | None:
    match = _ENV_INDEX_RE.search(path_str)
    return int(match.group(1)) if match else None


def randomize_particle_material_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    *,
    friction_range: tuple[float, float] | None = None,
    density_range: tuple[float, float] | None = None,
    damping_range: tuple[float, float] | None = None,
    cohesion_range: tuple[float, float] | None = None,
    adhesion_range: tuple[float, float] | None = None,
):
    """Randomize bound PhysX PBD material properties for particle domain randomization.

    Per-env materials follow ``env_ids`` and receive independent samples. With
    ``env_ids`` set, shared materials are sampled once because each affects every
    env using it. Density updates to spawned particles may be backend-dependent.
    """
    ranges: dict[str, tuple[float, float] | None] = {
        "friction": friction_range,
        "density": density_range,
        "damping": damping_range,
        "cohesion": cohesion_range,
        "adhesion": adhesion_range,
    }
    if all(rng is None for rng in ranges.values()):
        return

    _env: "AnyEnv" = env.unwrapped  # type: ignore[misc]
    stage = _env.sim.stage

    # Cache material prims to avoid traversing the stage on every reset.
    cached = getattr(_env, "_srb_pbd_material_prims", None)
    if cached is None:
        cached = [
            prim
            for prim in stage.Traverse()
            if prim.HasAPI(PhysxSchema.PhysxPBDMaterialAPI)
        ]
        _env._srb_pbd_material_prims = cached  # type: ignore[attr-defined]
        if not cached:
            logging.warning(
                "randomize_particle_material_uniform: no PhysxPBDMaterialAPI "
                "prims found; particle-material randomization is a no-op."
            )
        else:
            logging.info(
                "randomize_particle_material_uniform: randomizing %d PBD "
                "particle material(s) over {%s}.",
                len(cached),
                ", ".join(k for k, v in ranges.items() if v is not None),
            )

    env_ids_set = (
        {int(i) for i in torch.as_tensor(env_ids).flatten().tolist()}
        if env_ids is not None
        else None
    )

    shared_material_randomized = getattr(
        _env, "_srb_shared_pbd_material_randomized", False
    )

    for prim in cached:
        if env_ids_set is not None:
            env_idx = _env_index_from_prim_path(prim.GetPath().pathString)
            if env_idx is None:
                if shared_material_randomized:
                    continue
            elif env_idx not in env_ids_set:
                continue
        mat = PhysxSchema.PhysxPBDMaterialAPI(prim)
        accessors = {
            "friction": mat.CreateFrictionAttr,
            "density": mat.CreateDensityAttr,
            "damping": mat.CreateDampingAttr,
            "cohesion": mat.CreateCohesionAttr,
            "adhesion": mat.CreateAdhesionAttr,
        }
        for name, rng in ranges.items():
            if rng is None:
                continue
            value = float(
                sample_uniform(
                    torch.tensor(float(rng[0])),
                    torch.tensor(float(rng[1])),
                    (1,),
                    device="cpu",
                ).item()
            )
            accessors[name]().Set(value)

    _env._srb_shared_pbd_material_randomized = True  # type: ignore[attr-defined]


def reset_root_state_uniform_poisson_disk_2d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]] | None,
    radius: float,
    asset_cfg: Sequence[SceneEntityCfg],
):
    assets: list[RigidObject | Articulation] = [
        get_scene_root_asset(env.scene, cfg.name) for cfg in asset_cfg
    ]
    root_states = torch.stack(
        [asset.data.default_root_state[env_ids].clone() for asset in assets],
    ).swapaxes(0, 1)

    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=env.device)
    samples_pos_xy = sample_poisson_disk(
        (len(env_ids), len(asset_cfg)),
        (
            (range_list[0][0], range_list[1][0]),
            (range_list[0][1], range_list[1][1]),
        ),
        radius,
        device=env.device,
    )
    rand_samples = sample_uniform(
        ranges[2:, 0],
        ranges[2:, 1],
        (len(env_ids), len(asset_cfg), 4),
        device=env.device,
    )
    rand_samples = torch.cat([samples_pos_xy, rand_samples], dim=-1)

    positions = (
        root_states[:, :, 0:3]
        + env.scene.env_origins[env_ids].repeat(len(asset_cfg), 1, 1).swapaxes(0, 1)
        + rand_samples[:, :, 0:3]
    )
    orientations_delta = quat_from_euler_xyz(
        rand_samples[:, :, 3], rand_samples[:, :, 4], rand_samples[:, :, 5]
    )
    orientations = quat_mul(root_states[:, :, 3:7], orientations_delta)

    for asset, position, orientation in zip(
        assets,
        positions.unbind(1),
        orientations.unbind(1),
        strict=True,
    ):
        asset.write_root_pose_to_sim(
            torch.cat([position, orientation], dim=-1),
            env_ids=env_ids,  # type: ignore[misc]
        )

    if velocity_range is not None:
        range_list = [
            velocity_range.get(key, (0.0, 0.0))
            for key in ["x", "y", "z", "roll", "pitch", "yaw"]
        ]
        ranges = torch.tensor(range_list, dtype=torch.float32, device=env.device)
        rand_samples = sample_uniform(
            ranges[:, 0],
            ranges[:, 1],
            (len(env_ids), len(asset_cfg), 6),
            device=env.device,
        )
        velocities = root_states[:, :, 7:13] + rand_samples

        for asset, velocity in zip(
            assets,
            velocities.unbind(1),
            strict=True,
        ):
            asset.write_root_velocity_to_sim(velocity, env_ids=env_ids)  # type: ignore[misc]


def _entity_pos_w(entity: Any) -> torch.Tensor:
    data = getattr(entity, "data", None)
    positions = data.root_pos_w if data is not None else entity.get_world_poses()[0]
    return getattr(positions, "torch", positions)


def reset_xforms_uniform_poisson_disk_2d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    radius: float,
    asset_cfg: Sequence[SceneEntityCfg],
    keep_clear_of: Sequence[SceneEntityCfg] = (),
):
    assets: list[Any] = [get_scene_entity(env.scene, cfg.name) for cfg in asset_cfg]
    asset_count = (
        getattr(assets[0], "count", getattr(assets[0], "num_instances", 0))
        if getattr(env.cfg, "stack", False)
        else len(env_ids)
    )
    if asset_count == 0:
        return

    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=env.device)
    occupied = None
    if keep_clear_of:
        roots_xy = torch.stack(
            [
                _entity_pos_w(get_scene_entity(env.scene, cfg.name))[:, :2]
                for cfg in keep_clear_of
            ],
            dim=1,
        )
        if env.cfg.stack:
            # Stacked environments share one set of placed assets.
            occupied = roots_xy.reshape(1, -1, 2).expand(asset_count, -1, -1)
        else:
            occupied = roots_xy[env_ids] - env.scene.env_origins[env_ids, None, :2]
    samples_pos_xy = sample_poisson_disk(
        (asset_count, len(asset_cfg)),
        (
            (range_list[0][0], range_list[1][0]),
            (range_list[0][1], range_list[1][1]),
        ),
        radius,
        device=env.device,
        occupied=occupied,
    )
    rand_samples = sample_uniform(
        ranges[2:, 0],
        ranges[2:, 1],
        (asset_count, len(asset_cfg), 4),
        device=env.device,
    )
    rand_samples = torch.cat([samples_pos_xy, rand_samples], dim=-1)

    positions = (
        rand_samples[:, :, 0:3]
        if env.cfg.stack
        else (
            env.scene.env_origins[env_ids].repeat(len(asset_cfg), 1, 1).swapaxes(0, 1)
            + rand_samples[:, :, 0:3]
        )
    )
    orientations = quat_from_euler_xyz(
        rand_samples[:, :, 3], rand_samples[:, :, 4], rand_samples[:, :, 5]
    )

    for asset, position, orientation in zip(
        assets,
        positions.unbind(1),
        orientations.unbind(1),
        strict=True,
    ):
        asset_count_val = getattr(asset, "count", getattr(asset, "num_instances", 0))
        if asset_count_val == 0:
            continue

        indices_to_use = None if getattr(env.cfg, "stack", False) else env_ids
        if indices_to_use is not None:
            valid_mask = indices_to_use < asset_count_val
            indices_to_use = indices_to_use[valid_mask]
            if len(indices_to_use) == 0:
                continue
            position = position[valid_mask]
            orientation = orientation[valid_mask]

        asset.set_world_poses(
            positions=position,
            orientations=orientation,
            indices=indices_to_use,
        )


def reset_collection_root_state_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]] | None,
    asset_cfg: SceneEntityCfg,
):
    assets: RigidObjectCollection = get_scene_rigid_object_collection(
        env.scene, asset_cfg.name
    )

    root_states = assets.data.default_object_state[env_ids]

    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets.device)
    rand_samples = sample_uniform(
        ranges[:, 0],
        ranges[:, 1],
        (len(env_ids), assets.num_objects, 6),
        device=assets.device,
    )
    positions = (
        root_states[:, :, 0:3]
        + env.scene.env_origins[env_ids].repeat(assets.num_objects, 1, 1).swapaxes(0, 1)
        + rand_samples[:, :, 0:3]
    )
    orientations_delta = quat_from_euler_xyz(
        rand_samples[:, :, 3], rand_samples[:, :, 4], rand_samples[:, :, 5]
    )
    orientations = quat_mul(root_states[:, :, 3:7], orientations_delta)
    assets.write_object_pose_to_sim(
        torch.cat([positions, orientations], dim=-1), env_ids=env_ids
    )

    if velocity_range is not None:
        range_list = [
            velocity_range.get(key, (0.0, 0.0))
            for key in ["x", "y", "z", "roll", "pitch", "yaw"]
        ]
        ranges = torch.tensor(range_list, dtype=torch.float32, device=assets.device)
        rand_samples = sample_uniform(
            ranges[:, 0],
            ranges[:, 1],
            (len(env_ids), assets.num_objects, 6),
            device=assets.device,
        )
        velocities = root_states[:, :, 7:13] + rand_samples
        assets.write_object_velocity_to_sim(velocities, env_ids=env_ids)


def reset_collection_root_state_uniform_poisson_disk_2d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]] | None,
    radius: float,
    asset_cfg: SceneEntityCfg,
):
    assets: RigidObjectCollection = get_scene_rigid_object_collection(
        env.scene, asset_cfg.name
    )

    root_states = assets.data.default_object_state[env_ids]

    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets.device)
    samples_pos_xy = sample_poisson_disk(
        (len(env_ids), assets.num_objects),
        (
            (range_list[0][0], range_list[1][0]),
            (range_list[0][1], range_list[1][1]),
        ),
        radius,
        device=assets.device,
    )
    rand_samples = sample_uniform(
        ranges[2:, 0],
        ranges[2:, 1],
        (len(env_ids), assets.num_objects, 4),
        device=assets.device,
    )
    rand_samples = torch.cat([samples_pos_xy, rand_samples], dim=-1)

    positions = (
        root_states[:, :, 0:3]
        + env.scene.env_origins[env_ids].repeat(assets.num_objects, 1, 1).swapaxes(0, 1)
        + rand_samples[:, :, 0:3]
    )
    orientations_delta = quat_from_euler_xyz(
        rand_samples[:, :, 3], rand_samples[:, :, 4], rand_samples[:, :, 5]
    )
    orientations = quat_mul(root_states[:, :, 3:7], orientations_delta)
    assets.write_object_pose_to_sim(
        torch.cat([positions, orientations], dim=-1), env_ids=env_ids
    )

    if velocity_range is not None:
        range_list = [
            velocity_range.get(key, (0.0, 0.0))
            for key in ["x", "y", "z", "roll", "pitch", "yaw"]
        ]
        ranges = torch.tensor(range_list, dtype=torch.float32, device=assets.device)
        rand_samples = sample_uniform(
            ranges[:, 0],
            ranges[:, 1],
            (len(env_ids), assets.num_objects, 6),
            device=assets.device,
        )
        velocities = root_states[:, :, 7:13] + rand_samples
        assets.write_object_velocity_to_sim(velocities, env_ids=env_ids)


def reset_collection_root_state_uniform_poisson_disk_3d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]] | None,
    radius: float,
    asset_cfg: SceneEntityCfg,
):
    assets: RigidObjectCollection = get_scene_rigid_object_collection(
        env.scene, asset_cfg.name
    )

    root_states = assets.data.default_object_state[env_ids]

    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets.device)
    samples_pos = sample_poisson_disk(
        (len(env_ids), assets.num_objects),
        (
            (range_list[0][0], range_list[1][0], range_list[2][0]),
            (range_list[0][1], range_list[1][1], range_list[2][1]),
        ),
        radius,
        device=assets.device,
    )
    rand_samples = sample_uniform(
        ranges[3:, 0],
        ranges[3:, 1],
        (len(env_ids), assets.num_objects, 3),
        device=assets.device,
    )
    rand_samples = torch.cat([samples_pos, rand_samples], dim=-1)

    positions = (
        root_states[:, :, 0:3]
        + env.scene.env_origins[env_ids].repeat(assets.num_objects, 1, 1).swapaxes(0, 1)
        + rand_samples[:, :, 0:3]
    )
    orientations_delta = quat_from_euler_xyz(
        rand_samples[:, :, 3], rand_samples[:, :, 4], rand_samples[:, :, 5]
    )
    orientations = quat_mul(root_states[:, :, 3:7], orientations_delta)
    assets.write_object_pose_to_sim(
        torch.cat([positions, orientations], dim=-1), env_ids=env_ids
    )

    if velocity_range is not None:
        range_list = [
            velocity_range.get(key, (0.0, 0.0))
            for key in ["x", "y", "z", "roll", "pitch", "yaw"]
        ]
        ranges = torch.tensor(range_list, dtype=torch.float32, device=assets.device)
        rand_samples = sample_uniform(
            ranges[:, 0],
            ranges[:, 1],
            (len(env_ids), assets.num_objects, 6),
            device=assets.device,
        )
        velocities = root_states[:, :, 7:13] + rand_samples
        assets.write_object_velocity_to_sim(velocities, env_ids=env_ids)


def _reset_particles_with_xy_offset(
    ps: "AssetBase",
    env_ids: torch.Tensor,
    offset_range_m: float,
) -> None:
    """Restore settled particles with a per-env XY offset, preserving pile shape."""
    ids = (
        env_ids
        if isinstance(env_ids, torch.Tensor)
        else torch.tensor(list(env_ids), device=ps.device, dtype=torch.long)  # type: ignore[attr-defined]
    )
    pos = ps.data.default_particle_pos_w[ids].clone()  # (n_reset, n_particles, 3)
    vel = ps.data.default_particle_vel_w[ids]
    offset = sample_uniform(
        -offset_range_m,
        offset_range_m,
        (pos.shape[0], 1, 2),
        device=pos.device,
    )
    pos[..., :2] += offset
    ps.write_state_to_sim(pos_w=pos, vel_w=vel, env_ids=env_ids)  # type: ignore[attr-defined]


def settle_and_reset_particle_systems(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    particle_system_names: Sequence[str],
    freeze: bool = False,
    conform_to_terrain: bool = False,
    conform_surface_offset: float = 0.0,
    conform_xy_jitter: float = 0.0,
    conform_jitter_seed: int = 0,
    pile_center_offset_range_m: float = 0.0,
):
    """Settle and reset registered particle systems.

    The first call caches the settled state; later calls restore it. Terrain
    conformation replaces settling; ``freeze`` disables the solver after setup.
    ``conform_surface_offset`` and ``conform_xy_jitter`` are in meters;
    ``conform_jitter_seed`` controls reproducible jitter.
    """
    _env = env.unwrapped  # type: ignore[misc]
    particle_systems = getattr(_env, "particle_systems", {})

    if not particle_systems:
        logging.warning(
            "settle_and_reset_particle_systems called but no particle systems "
            "were registered on env."
        )
        return

    for ps_name in particle_system_names:
        ps = particle_systems.get(ps_name)
        if ps is None:
            logging.warning(f"Particle system '{ps_name}' not found on env.")
            continue

        if not ps.is_initialized:
            logging.warning(f"Particle system '{ps_name}' not initialized. Skipping.")
            continue

        if not ps.data.has_default_state:
            # Rewind spawner-authored state to prevent startup drift becoming a default.
            # Align assemblies because settling advances physics inside this reset.
            align_assemblies = getattr(
                _env, "_update_assembly_fixed_joint_transforms", None
            )
            if callable(align_assemblies):
                align_assemblies(env_ids)
            ps.restore_authored_initial_state()
            root_state_snapshots = []
            for _, asset in iter_scene_root_assets(_env.scene):
                # Shorthand caches may be stale after reset; snapshot the aggregate state.
                root_state = getattr(
                    asset.data.root_state_w,
                    "torch",
                    asset.data.root_state_w,
                ).clone()
                root_state_snapshots.append(
                    (
                        asset,
                        root_state[:, :7],
                        root_state[:, 7:13],
                        asset.data.joint_pos.clone()
                        if isinstance(asset, Articulation)
                        else None,
                        asset.data.joint_vel.clone()
                        if isinstance(asset, Articulation)
                        else None,
                    )
                )
            collection_state_snapshots = [
                (collection, collection.data.object_state_w.clone())
                for _, collection in iter_scene_rigid_object_collections(_env.scene)
            ]

            def _evacuate(
                root_snapshots=root_state_snapshots,
                collection_snapshots=collection_state_snapshots,
            ):
                for asset, root_pose, root_velocity, _, _ in root_snapshots:
                    evacuated_pose = root_pose.clone()
                    evacuated_pose[:, 2] -= 10000.0
                    asset.write_root_link_pose_to_sim_index(root_pose=evacuated_pose)
                    asset.write_root_com_velocity_to_sim_index(
                        root_velocity=root_velocity
                    )
                for collection, object_state in collection_snapshots:
                    new_state = object_state.clone()
                    new_state[:, :, 2] -= 10000.0
                    collection.write_object_state_to_sim(new_state)

            def _restore(
                root_snapshots=root_state_snapshots,
                collection_snapshots=collection_state_snapshots,
            ):
                for (
                    asset,
                    root_pose,
                    root_velocity,
                    joint_pos,
                    joint_vel,
                ) in root_snapshots:
                    # Preserve randomization applied by earlier reset events.
                    asset.write_root_link_pose_to_sim_index(root_pose=root_pose)
                    asset.write_root_com_velocity_to_sim_index(
                        root_velocity=root_velocity
                    )
                    if joint_pos is not None and joint_vel is not None:
                        asset.write_joint_position_to_sim_index(position=joint_pos)
                        asset.write_joint_velocity_to_sim_index(velocity=joint_vel)
                for collection, object_state in collection_snapshots:
                    collection.write_object_state_to_sim(object_state)

            if conform_to_terrain:
                # Conform directly because PBD settling is unavailable in this build.
                ps.conform_to_terrain(
                    surface_offset=conform_surface_offset,
                    xy_jitter=conform_xy_jitter,
                    jitter_seed=conform_jitter_seed,
                    evacuate_fn=_evacuate,
                    restore_fn=_restore,
                )
            else:
                ps.settle(
                    sim_step_fn=_env.sim.step,
                    step_dt=_env.physics_dt,
                    evacuate_fn=_evacuate,
                    restore_fn=_restore,
                )
            ps.discard_authored_initial_state()
            if freeze:
                # Disable simulation while retaining the settled point geometry.
                ps.disable()
        else:
            if pile_center_offset_range_m > 0.0 and not freeze:
                _reset_particles_with_xy_offset(ps, env_ids, pile_center_offset_range_m)
            else:
                ps.reset(env_ids)
