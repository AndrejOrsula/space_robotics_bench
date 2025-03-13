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
        cmd_attr.shape,
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
        if env_ids and i not in env_ids:
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
