from typing import TYPE_CHECKING, Dict, List, Sequence, Tuple

import torch
import torch.nn.functional as F
from pxr import Gf

import srb.core.sim.spawners.particles.utils as particle_utils
from srb.core.asset import (
    Articulation,
    AssetBase,
    RigidObject,
    RigidObjectCollection,
    XFormPrim,
)
from srb.core.manager import SceneEntityCfg
from srb.utils.math import quat_from_angle_axis, quat_from_euler_xyz, quat_mul, slerp
from srb.utils.sampling import (
    sample_poisson_disk_2d_looped,
    sample_poisson_disk_3d_looped,
    sample_uniform,
)
from srb.utils.usd import safe_set_attribute_on_usd_prim

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def reset_scene_to_default(env: "AnyEnv", env_ids: torch.Tensor):
    reset_rigid_objects_default(env, env_ids)
    reset_articulations_default(env, env_ids)
    reset_deformable_objects_default(env, env_ids)


def reset_rigid_objects_default(env: "AnyEnv", env_ids: torch.Tensor | None):
    for rigid_object in env.scene.rigid_objects.values():
        # Obtain default and deal with the offset for env origins
        default_root_state = rigid_object.data.default_root_state[env_ids].clone()
        default_root_state[:, 0:3] += env.scene.env_origins[env_ids]
        # Set into the physics simulation
        rigid_object.write_root_pose_to_sim(
            default_root_state[:, :7],
            env_ids=env_ids,  # type: ignore
        )
        # TODO[mid]: Do not reset velocity for kinematic objects
        rigid_object.write_root_velocity_to_sim(
            default_root_state[:, 7:],
            env_ids=env_ids,  # type: ignore
        )


def reset_articulations_default(env: "AnyEnv", env_ids: torch.Tensor | None):
    for articulation_asset in env.scene.articulations.values():
        # Obtain default and deal with the offset for env origins
        default_root_state = articulation_asset.data.default_root_state[env_ids].clone()
        default_root_state[:, 0:3] += env.scene.env_origins[env_ids]
        # Set into the physics simulation
        articulation_asset.write_root_pose_to_sim(
            default_root_state[:, :7],
            env_ids=env_ids,  # type: ignore
        )
        articulation_asset.write_root_velocity_to_sim(
            default_root_state[:, 7:],
            env_ids=env_ids,  # type: ignore
        )
        # Obtain default joint positions
        default_joint_pos = articulation_asset.data.default_joint_pos[env_ids].clone()
        default_joint_vel = articulation_asset.data.default_joint_vel[env_ids].clone()
        # Set into the physics simulation
        articulation_asset.write_joint_state_to_sim(
            default_joint_pos,
            default_joint_vel,
            env_ids=env_ids,  # type: ignore
        )


def reset_deformable_objects_default(env: "AnyEnv", env_ids: torch.Tensor | None):
    for deformable_object in env.scene.deformable_objects.values():
        # Obtain default and set into the physics simulation
        nodal_state = deformable_object.data.default_nodal_state_w[env_ids].clone()
        deformable_object.write_nodal_state_to_sim(nodal_state, env_ids=env_ids)  # type: ignore


def randomize_pose(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
    pose_range: Dict[str, Tuple[float, float]],
):
    _env: "AnyEnv" = env.unwrapped  # type: ignore
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
    pos_range: Dict[str, Tuple[float, float]],
):
    _env: "AnyEnv" = env.unwrapped  # type: ignore
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


def offset_pos_natural(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
    axes: Sequence[str],
    step_range: Tuple[float, float],
    smoothness: float,
    pos_bounds: Dict[str, Tuple[float, float]],
):
    """Move the target position naturally with smoothed random changes in direction.

    Args:
        env: Environment instance
        env_ids: Indices of environments to update
        env_attr_name: Name of the attribute to modify
        axes: Which axes to apply movement to (e.g., ["x", "y"])
        step_range: Range of step sizes per update
        smoothness: Value between 0-1 controlling continuity of movement (higher = smoother)
        pos_bounds: Dictionary of position bounds for each axis
    """
    _env: "AnyEnv" = env.unwrapped  # type: ignore
    if env_ids is None:
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )

    state_key = f"__{env_attr_name}_natural_movement_velocities"

    # Get or initialize velocity vectors
    if not hasattr(_env, state_key):
        # Initialize with random velocities
        velocities = torch.randn((_env.num_envs, 3), device=_env.device)
        setattr(_env, state_key, velocities)

    # Get the velocities for the current environments
    velocities = getattr(_env, state_key)[env_ids]

    # Apply random perturbation with smoothing
    random_direction = torch.randn_like(velocities)

    # Update velocity with smoothed random changes
    velocities = F.normalize(
        smoothness * velocities + (1 - smoothness) * random_direction, p=2, dim=1
    )

    # Sample step sizes from step_range
    step_sizes = sample_uniform(
        step_range[0], step_range[1], (len(env_ids),), device=_env.device
    )

    # Apply step sizes to velocities
    delta_pos = velocities * step_sizes.unsqueeze(1)

    # Get current positions
    pos_attr = getattr(env, env_attr_name)
    current_positions = pos_attr[env_ids].clone()

    # Apply movement only to specified axes
    axis_indices = {"x": 0, "y": 1, "z": 2}
    active_axes = [axis_indices[axis] for axis in axes if axis in axis_indices]

    # Calculate new positions
    new_positions = current_positions.clone()
    for axis_idx in active_axes:
        new_positions[:, axis_idx] += delta_pos[:, axis_idx]

    # Handle boundaries by reflecting velocities when hitting bounds
    for axis in axes:
        if axis in pos_bounds:
            axis_idx = axis_indices[axis]
            min_bound, max_bound = pos_bounds[axis]

            # Calculate bounds relative to environment origins
            actual_min = min_bound + env.scene.env_origins[env_ids, axis_idx]
            actual_max = max_bound + env.scene.env_origins[env_ids, axis_idx]

            # Check if any positions exceed bounds
            below_min = new_positions[:, axis_idx] < actual_min
            above_max = new_positions[:, axis_idx] > actual_max

            # Reflect positions and velocities at boundaries
            if below_min.any():
                new_positions[below_min, axis_idx] = (
                    2 * actual_min[below_min] - new_positions[below_min, axis_idx]
                )
                velocities[below_min, axis_idx] *= -1.0

            if above_max.any():
                new_positions[above_max, axis_idx] = (
                    2 * actual_max[above_max] - new_positions[above_max, axis_idx]
                )
                velocities[above_max, axis_idx] *= -1.0

    # Save updated velocities back to the state dictionary
    getattr(_env, state_key)[env_ids] = velocities

    # Update the positions in the environment
    pos_attr[env_ids] = new_positions


def offset_pose_natural(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
    pos_axes: Sequence[str],
    pos_step_range: Tuple[float, float],
    pos_smoothness: float,
    pos_step_smoothness: float,
    pos_bounds: Dict[str, Tuple[float, float]],
    orient_yaw_only: bool,
    orient_smoothness: float,
):
    """Move the target pose naturally with smoothed random changes in direction and orientation.

    Args:
        env: Environment instance
        env_ids: Indices of environments to update
        env_attr_name: Name of the attribute to modify
        pos_axes: Which position axes to apply movement to (e.g., ["x", "y"])
        pos_step_range: Range of position step sizes per update
        pos_smoothness: Value between 0-1 controlling continuity of movement (higher = smoother)
        pos_step_smoothness: Value between 0-1 controlling continuity of velocity (higher = smoother)
        pos_bounds: Dictionary of position bounds for each axis
        orient_yaw_only: If True, only the yaw of the orientation will be updated to match the direction of movement.
        orient_smoothness: Value between 0-1 controlling continuity of orientation (higher = smoother)
    """
    _env: "AnyEnv" = env.unwrapped  # type: ignore
    if env_ids is None:
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )

    # -- Handle Position --
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

    # -- Handle Velocity --
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

    # -- Apply changes --
    pose_attr = getattr(env, env_attr_name)
    current_pose = pose_attr[env_ids].clone()
    current_positions, current_orientations = current_pose[:, :3], current_pose[:, 3:]

    # -- Handle Orientation --
    orient_state_key = f"__{env_attr_name}_natural_movement_orientations"
    if not hasattr(_env, orient_state_key):
        setattr(_env, orient_state_key, current_orientations.clone())

    last_orientations = getattr(_env, orient_state_key)[env_ids]

    # Create a zero vector for axes that are not active
    active_pos_velocities = pos_velocities.clone()
    pos_axis_indices = {"x": 0, "y": 1, "z": 2}
    inactive_pos_axes = [
        axis_idx for axis, axis_idx in pos_axis_indices.items() if axis not in pos_axes
    ]
    for axis_idx in inactive_pos_axes:
        active_pos_velocities[:, axis_idx] = 0.0

    # Normalize the velocity to get the direction
    move_direction = F.normalize(active_pos_velocities, p=2, dim=1)

    # -- Calculate orientation based on movement direction
    # Yaw (z-axis rotation)
    if orient_yaw_only:
        # Project movement direction onto XY plane
        yaw_vec = move_direction.clone()
        yaw_vec[:, 2] = 0.0
        # If yaw_vec is zero (movement is purely vertical), use a default forward direction
        # This avoids normalization issues and keeps orientation consistent
        is_zero_mask = torch.all(yaw_vec == 0, dim=1)
        yaw_vec[is_zero_mask] = torch.tensor([1.0, 0.0, 0.0], device=_env.device)
        yaw_vec = F.normalize(yaw_vec, p=2, dim=1)

        yaw_angle = torch.atan2(yaw_vec[:, 1], yaw_vec[:, 0])
        target_orientations = quat_from_euler_xyz(
            torch.zeros_like(yaw_angle), torch.zeros_like(yaw_angle), yaw_angle
        )
    else:
        # Full 3D orientation
        # Create a rotation from the forward vector (1, 0, 0) to the move_direction
        forward_vec = torch.tensor(
            [1.0, 0.0, 0.0], device=_env.device, dtype=torch.float32
        ).expand_as(move_direction)
        axis = torch.cross(forward_vec, move_direction, dim=1)
        angle = torch.acos(torch.sum(forward_vec * move_direction, dim=1))
        target_orientations = quat_from_angle_axis(angle, axis)

    # Slerp between last orientation and target orientation
    new_orientations = slerp(
        last_orientations, target_orientations, 1.0 - orient_smoothness
    )
    getattr(_env, orient_state_key)[env_ids] = new_orientations.clone()

    # Apply position movement
    pos_axis_indices = {"x": 0, "y": 1, "z": 2}
    pos_active_axes = [
        pos_axis_indices[axis] for axis in pos_axes if axis in pos_axis_indices
    ]
    new_positions = current_positions.clone()
    for axis_idx in pos_active_axes:
        new_positions[:, axis_idx] += delta_pos[:, axis_idx]

    # Handle position boundaries
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

    # Update the pose in the environment
    pose_attr[env_ids] = torch.cat([new_positions, new_orientations], dim=-1)


def randomize_command(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    env_attr_name: str,
    magnitude: float = 1.0,
):
    _env: "AnyEnv" = env.unwrapped  # type: ignore
    if env_ids is None:
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )
    cmd_attr = getattr(env, env_attr_name)
    cmd_attr[env_ids] = sample_uniform(
        -magnitude,
        magnitude,
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
        _env: "AnyEnv" = env.unwrapped  # type: ignore
        env_ids = torch.arange(
            _env.num_envs,
            device=_env.device,
        )

    joint_assembly = getattr(env, env_joint_assemblies_attr_name)[assembly_key]
    actions = getattr(env, env_action_manager_attr_name).action[env_ids, action_idx]

    for assembly, action in zip(joint_assembly, actions):
        assembly.set_attach_path_root_joints_enabled(
            eval(f"{action}{cmp_op}{cmp_value}")
        )


def reset_xform_orientation_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    orientation_distribution_params: Dict[str, Tuple[float, float]],
    asset_cfg: SceneEntityCfg,
):
    asset: XFormPrim = env.scene[asset_cfg.name]

    range_list = [
        orientation_distribution_params.get(key, (0.0, 0.0))
        for key in ["roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, device=asset._device)
    rand_samples = sample_uniform(
        ranges[:, 0], ranges[:, 1], (1, 3), device=asset._device
    )

    orientations = quat_from_euler_xyz(
        rand_samples[:, 0], rand_samples[:, 1], rand_samples[:, 2]
    )

    asset.set_world_poses(orientations=orientations)


def randomize_usd_prim_attribute_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    attr_name: str,
    distribution_params: Tuple[float | Sequence[float], float | Sequence[float]],
    asset_cfg: SceneEntityCfg,
):
    asset: XFormPrim = env.scene[asset_cfg.name]
    if isinstance(distribution_params[0], Sequence):
        dist_len = len(distribution_params[0])
        distribution_params = (  # type: ignore
            torch.tensor(distribution_params[0]),
            torch.tensor(distribution_params[1]),
        )
    else:
        dist_len = 1
    for i, prim in enumerate(asset.prims):
        if env_ids is not None and i not in env_ids:
            continue
        value = sample_uniform(
            distribution_params[0],  # type: ignore
            distribution_params[1],  # type: ignore
            (dist_len,),
            device="cpu",
        )
        value = value.item() if dist_len == 1 else value.tolist()
        safe_set_attribute_on_usd_prim(
            prim, f"inputs:{attr_name}", value, camel_case=True
        )


def randomize_gravity_uniform(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    distribution_params: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
):
    physics_scene = env.sim._physics_context._physics_scene  # type: ignore
    gravity = sample_uniform(
        torch.tensor(distribution_params[0]),
        torch.tensor(distribution_params[1]),
        (3,),
        device="cpu",
    )
    gravity_magnitude = torch.norm(gravity)
    if gravity_magnitude == 0.0:
        gravity_direction = gravity
    else:
        gravity_direction = gravity / gravity_magnitude

    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(*gravity_direction.tolist()))
    physics_scene.CreateGravityMagnitudeAttr(gravity_magnitude.item())


def follow_xform_orientation_linear_trajectory(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    orientation_step_params: Dict[str, float],
    asset_cfg: SceneEntityCfg,
):
    asset: XFormPrim = env.scene[asset_cfg.name]

    _, current_quat = asset.get_world_poses()

    steps = torch.tensor(
        [orientation_step_params.get(key, 0.0) for key in ["roll", "pitch", "yaw"]],
        device=asset._device,
    )
    step_quat = quat_from_euler_xyz(steps[0], steps[1], steps[2]).unsqueeze(0)

    orientations = quat_mul(current_quat, step_quat)  # type: ignore

    asset.set_world_poses(orientations=orientations)


def reset_root_state_uniform_poisson_disk_2d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]] | None,
    radius: float,
    asset_cfg: Sequence[SceneEntityCfg],
):
    # Extract the used quantities (to enable type-hinting)
    assets: List[RigidObject | Articulation] = [
        env.scene[cfg.name] for cfg in asset_cfg
    ]
    # Get default root state
    root_states = torch.stack(
        [asset.data.default_root_state[env_ids].clone() for asset in assets],
    ).swapaxes(0, 1)

    # Poses
    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=env.device)
    samples_pos_xy = torch.tensor(
        sample_poisson_disk_2d_looped(
            (len(env_ids), len(asset_cfg)),
            (
                (range_list[0][0], range_list[1][0]),
                (range_list[0][1], range_list[1][1]),
            ),
            radius,
        ),
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

    # Set into the physics simulation
    for asset, position, orientation in zip(
        assets,
        positions.unbind(1),
        orientations.unbind(1),
    ):
        asset.write_root_pose_to_sim(
            torch.cat([position, orientation], dim=-1),
            env_ids=env_ids,  # type: ignore
        )

    # Velocities
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

        # Set into the physics simulation
        for asset, velocity in zip(
            assets,
            velocities.unbind(1),
        ):
            asset.write_root_velocity_to_sim(velocity, env_ids=env_ids)  # type: ignore


def reset_xforms_uniform_poisson_disk_2d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    radius: float,
    asset_cfg: Sequence[SceneEntityCfg],
):
    # Extract the used quantities (to enable type-hinting)
    assets: List[XFormPrim] = [env.scene[cfg.name] for cfg in asset_cfg]
    asset_count = assets[0].count

    # Poses
    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=env.device)
    samples_pos_xy = torch.tensor(
        sample_poisson_disk_2d_looped(
            (asset_count, len(asset_cfg)),
            (
                (range_list[0][0], range_list[1][0]),
                (range_list[0][1], range_list[1][1]),
            ),
            radius,
        ),
        device=env.device,
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

    # Set into the physics simulation
    for asset, position, orientation in zip(
        assets,
        positions.unbind(1),
        orientations.unbind(1),
    ):
        asset.set_world_poses(
            positions=position,
            orientations=orientation,
            indices=None if env.cfg.stack else env_ids,
        )


def reset_collection_root_state_uniform_poisson_disk_2d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]] | None,
    radius: float,
    asset_cfg: SceneEntityCfg,
):
    # Extract the used quantities (to enable type-hinting)
    assets: RigidObjectCollection = env.scene[asset_cfg.name]

    # Get default root state
    root_states = assets.data.default_object_state[env_ids]

    # Poses
    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets.device)
    samples_pos_xy = torch.tensor(
        sample_poisson_disk_2d_looped(
            (len(env_ids), assets.num_objects),
            (
                (range_list[0][0], range_list[1][0]),
                (range_list[0][1], range_list[1][1]),
            ),
            radius,
        ),
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

    # Velocities
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


def reset_xforms_uniform_poisson_disk_3d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float, float]],
    radius: float,
    asset_cfg: Sequence[SceneEntityCfg],
):
    # Extract the used quantities (to enable type-hinting)
    assets: List[XFormPrim] = [env.scene[cfg.name] for cfg in asset_cfg]
    asset_count = assets[0].count

    # Poses
    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=env.device)
    samples_pos = torch.tensor(
        sample_poisson_disk_3d_looped(
            (asset_count, len(asset_cfg)),
            (
                (range_list[0][0], range_list[1][0], range_list[2][0]),
                (range_list[0][1], range_list[1][1], range_list[2][1]),
            ),
            radius,
        ),
        device=env.device,
    )
    rand_samples = sample_uniform(
        ranges[3:, 0],
        ranges[3:, 1],
        (asset_count, len(asset_cfg), 3),
        device=env.device,
    )
    rand_samples = torch.cat([samples_pos, rand_samples], dim=-1)

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

    # Set into the physics simulation
    for asset, position, orientation in zip(
        assets, positions.unbind(1), orientations.unbind(1)
    ):
        asset.set_world_poses(
            positions=position,
            orientations=orientation,
            indices=None if env.cfg.stack else env_ids,
        )


def reset_root_state_uniform_poisson_disk_3d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float, float]],
    velocity_range: dict[str, tuple[float, float, float]] | None,
    radius: float,
    asset_cfg: Sequence[SceneEntityCfg],
):
    # Extract the used quantities (to enable type-hinting)
    assets: List[RigidObject | Articulation] = [
        env.scene[cfg.name] for cfg in asset_cfg
    ]
    # Get default root state
    root_states = torch.stack(
        [asset.data.default_root_state[env_ids].clone() for asset in assets],
    ).swapaxes(0, 1)

    # Poses
    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=env.device)
    samples_pos = torch.tensor(
        sample_poisson_disk_3d_looped(
            (len(env_ids), len(asset_cfg)),
            (
                (range_list[0][0], range_list[1][0], range_list[2][0]),
                (range_list[0][1], range_list[1][1], range_list[2][1]),
            ),
            radius,
        ),
        device=env.device,
    )
    rand_samples = sample_uniform(
        ranges[3:, 0],
        ranges[3:, 1],
        (len(env_ids), len(asset_cfg), 3),
        device=env.device,
    )
    rand_samples = torch.cat([samples_pos, rand_samples], dim=-1)

    positions = (
        root_states[:, :, 0:3]
        + env.scene.env_origins[env_ids].repeat(len(asset_cfg), 1, 1).swapaxes(0, 1)
        + rand_samples[:, :, 0:3]
    )
    orientations_delta = quat_from_euler_xyz(
        rand_samples[:, :, 3], rand_samples[:, :, 4], rand_samples[:, :, 5]
    )
    orientations = quat_mul(root_states[:, :, 3:7], orientations_delta)

    # Set into the physics simulation
    for asset, position, orientation in zip(
        assets,
        positions.unbind(1),
        orientations.unbind(1),
    ):
        asset.write_root_pose_to_sim(
            torch.cat([position, orientation], dim=-1),
            env_ids=env_ids,  # type: ignore
        )

    # Velocities
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

        # Set into the physics simulation
        for asset, velocity in zip(
            assets,
            velocities.unbind(1),
        ):
            asset.write_root_velocity_to_sim(velocity, env_ids=env_ids)  # type: ignore


def reset_collection_root_state_uniform_poisson_disk_3d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float, float]],
    velocity_range: dict[str, tuple[float, float, float]] | None,
    radius: float,
    asset_cfg: SceneEntityCfg,
):
    # Extract the used quantities (to enable type-hinting)
    assets: RigidObjectCollection = env.scene[asset_cfg.name]

    # Get default root state
    root_states = assets.data.default_object_state[env_ids]

    # Poses
    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets.device)
    samples_pos = torch.tensor(
        sample_poisson_disk_3d_looped(
            (len(env_ids), assets.num_objects),
            (
                (range_list[0][0], range_list[1][0], range_list[2][0]),
                (range_list[0][1], range_list[1][1], range_list[2][1]),
            ),
            radius,
        ),
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

    # Velocities
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


def settle_and_reset_particles(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    asset_cfg: Sequence[SceneEntityCfg],
    particles_settle_max_steps: int = 25,
    particles_settle_step_time: float = 20.0,
    particles_settle_vel_threshold: float = 0.0025,
):
    num_particle_systems = len(asset_cfg)
    particles: Sequence[AssetBase] = tuple(env.scene[cfg.name] for cfg in asset_cfg)
    initial_pos_ident: Sequence[str] = tuple(
        f"__particles_{cfg.name}_initial_pos" for cfg in asset_cfg
    )
    initial_vel_ident: Sequence[str] = tuple(
        f"__particles_{cfg.name}_initial_vel" for cfg in asset_cfg
    )

    ## Let the particles settle on the first reset, then remember their positions for future resets
    if not hasattr(env, initial_pos_ident[0]):
        for _ in range(particles_settle_max_steps):
            for _ in range(round(particles_settle_step_time / env.step_dt)):
                env.sim.step(render=False)

            for i in range(num_particle_systems):
                if (
                    torch.median(
                        torch.linalg.norm(
                            particle_utils.get_particles_vel_w(env, particles[i]),
                            dim=-1,
                        )
                    )
                    > particles_settle_vel_threshold
                ):
                    break
            else:
                break

        # Extract statistics about the initial state of the particles
        for i in range(num_particle_systems):
            particles_pos = particle_utils.get_particles_pos_w(env, particles[i])
            setattr(env, initial_pos_ident[i], particles_pos)
            setattr(env, initial_vel_ident[i], torch.zeros_like(particles_pos))
    else:
        for i in range(num_particle_systems):
            particle_utils.set_particles_pos_w(
                env,
                particles[i],
                getattr(env, initial_pos_ident[i]),
                env_ids=env_ids,  # type: ignore
            )
            particle_utils.set_particles_vel_w(
                env,
                particles[i],
                getattr(env, initial_vel_ident[i]),
                env_ids=env_ids,  # type: ignore
            )
