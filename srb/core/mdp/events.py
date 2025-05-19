from typing import TYPE_CHECKING, Dict, List, Sequence, Tuple

import torch
from pxr import Gf

from srb.core.asset import Articulation, RigidObject, RigidObjectCollection, XFormPrim
from srb.core.manager import SceneEntityCfg
from srb.utils.math import quat_from_euler_xyz, quat_mul
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


# Global dictionary to store velocity vectors for natural movement
_natural_movement_velocities = {}


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

    # Create a state key for this specific environment and attribute
    state_key = f"{id(env)}:{env_attr_name}"

    # Get or initialize velocity vectors
    if state_key not in _natural_movement_velocities:
        # Initialize with random velocities
        _natural_movement_velocities[state_key] = torch.zeros(
            (_env.num_envs, 3), device=_env.device
        )
        # Set initial random direction for each environment
        for i in range(_env.num_envs):
            _natural_movement_velocities[state_key][i] = torch.randn(
                3, device=_env.device
            )
            # Normalize to ensure we start with unit vector
            _natural_movement_velocities[state_key][i] /= (
                torch.norm(_natural_movement_velocities[state_key][i]) + 1e-8
            )

    # Get the velocities for the current environments
    velocities = _natural_movement_velocities[state_key][env_ids]

    # Apply random perturbation with smoothing
    random_direction = torch.randn_like(velocities)
    # Normalize random direction
    random_direction /= torch.norm(random_direction, dim=1, keepdim=True) + 1e-8

    # Update velocity with smoothed random changes
    velocities = smoothness * velocities + (1 - smoothness) * random_direction
    # Normalize velocities to prevent acceleration
    velocities /= torch.norm(velocities, dim=1, keepdim=True) + 1e-8

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
    _natural_movement_velocities[state_key][env_ids] = velocities

    # Update the positions in the environment
    pos_attr[env_ids] = new_positions


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
    velocity_range: dict[str, tuple[float, float]],
    radius: float,
    asset_cfg: List[SceneEntityCfg],
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
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets[0].device)
    samples_pos_xy = torch.tensor(
        sample_poisson_disk_2d_looped(
            (len(env_ids), len(asset_cfg)),
            (
                (range_list[0][0], range_list[1][0]),
                (range_list[0][1], range_list[1][1]),
            ),
            radius,
        ),
        device=assets[0].device,
    )
    rand_samples = sample_uniform(
        ranges[2:, 0],
        ranges[2:, 1],
        (len(env_ids), len(asset_cfg), 4),
        device=assets[0].device,
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

    # Velocities
    range_list = [
        velocity_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets[0].device)
    rand_samples = sample_uniform(
        ranges[:, 0],
        ranges[:, 1],
        (len(env_ids), len(asset_cfg), 6),
        device=assets[0].device,
    )
    velocities = root_states[:, :, 7:13] + rand_samples

    # Set into the physics simulation
    for asset, position, orientation, velocity in zip(
        assets,
        positions.unbind(1),
        orientations.unbind(1),
        velocities.unbind(1),
    ):
        asset.write_root_pose_to_sim(
            torch.cat([position, orientation], dim=-1),
            env_ids=env_ids,  # type: ignore
        )
        asset.write_root_velocity_to_sim(velocity, env_ids=env_ids)  # type: ignore


def reset_collection_root_state_uniform_poisson_disk_2d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]],
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

    # Velocities
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

    # Set into the physics simulation
    assets.write_object_pose_to_sim(
        torch.cat([positions, orientations], dim=-1), env_ids=env_ids
    )
    assets.write_object_velocity_to_sim(velocities, env_ids=env_ids)


def reset_root_state_uniform_poisson_disk_3d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float, float]],
    velocity_range: dict[str, tuple[float, float, float]],
    radius: float,
    asset_cfg: List[SceneEntityCfg],
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
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets[0].device)
    samples_pos = torch.tensor(
        sample_poisson_disk_3d_looped(
            (len(env_ids), len(asset_cfg)),
            (
                (range_list[0][0], range_list[1][0], range_list[2][0]),
                (range_list[0][1], range_list[1][1], range_list[2][1]),
            ),
            radius,
        ),
        device=assets[0].device,
    )
    rand_samples = sample_uniform(
        ranges[3:, 0],
        ranges[3:, 1],
        (len(env_ids), len(asset_cfg), 3),
        device=assets[0].device,
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

    # Velocities
    range_list = [
        velocity_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, dtype=torch.float32, device=assets[0].device)
    rand_samples = sample_uniform(
        ranges[:, 0],
        ranges[:, 1],
        (len(env_ids), len(asset_cfg), 6),
        device=assets[0].device,
    )
    velocities = root_states[:, :, 7:13] + rand_samples

    # Set into the physics simulation
    for asset, position, orientation, velocity in zip(
        assets,
        positions.unbind(1),
        orientations.unbind(1),
        velocities.unbind(1),
    ):
        asset.write_root_pose_to_sim(
            torch.cat([position, orientation], dim=-1),
            env_ids=env_ids,  # type: ignore
        )
        asset.write_root_velocity_to_sim(velocity, env_ids=env_ids)  # type: ignore


def reset_collection_root_state_uniform_poisson_disk_3d(
    env: "AnyEnv",
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float, float]],
    velocity_range: dict[str, tuple[float, float, float]],
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

    # Velocities
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

    # Set into the physics simulation
    assets.write_object_pose_to_sim(
        torch.cat([positions, orientations], dim=-1), env_ids=env_ids
    )
    assets.write_object_velocity_to_sim(velocities, env_ids=env_ids)
