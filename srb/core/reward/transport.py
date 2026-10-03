import torch


def compute_planar_target_progress_velocity(
    *,
    current_xy: torch.Tensor,
    target_xy: torch.Tensor,
    velocity_xy: torch.Tensor,
    eps: float = 1.0e-6,
) -> torch.Tensor:
    """Signed planar velocity component toward a target point."""
    if current_xy.ndim != 2 or current_xy.shape[1] != 2:
        raise ValueError("current_xy must have shape (num_envs, 2)")
    if target_xy.shape != current_xy.shape:
        raise ValueError("target_xy must match current_xy shape")
    if velocity_xy.shape != current_xy.shape:
        raise ValueError("velocity_xy must match current_xy shape")
    if eps <= 0.0:
        raise ValueError("eps must be positive")

    target_delta = target_xy - current_xy
    target_dist = torch.linalg.norm(target_delta, dim=-1, keepdim=True)
    target_dir = target_delta / target_dist.clamp_min(float(eps))
    return torch.sum(velocity_xy * target_dir, dim=-1)


def compute_two_robot_mounted_beam_pose(
    *,
    robot_xyz: torch.Tensor,
    beam_z: float | torch.Tensor,
    fallback_pos: torch.Tensor | None = None,
    fallback_quat: torch.Tensor | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Compute a kinematic beam pose from the two-robot support frame."""
    if robot_xyz.ndim != 3 or robot_xyz.shape[1:] != (2, 3):
        raise ValueError("robot_xyz must have shape (num_envs, 2, 3)")
    num_envs = robot_xyz.shape[0]
    if (fallback_pos is None) != (fallback_quat is None):
        raise ValueError("fallback_pos and fallback_quat must be provided together")
    if fallback_pos is not None and fallback_pos.shape != (num_envs, 3):
        raise ValueError("fallback_pos must have shape (num_envs, 3)")
    if fallback_quat is not None and fallback_quat.shape != (num_envs, 4):
        raise ValueError("fallback_quat must have shape (num_envs, 4)")

    finite_support = torch.isfinite(robot_xyz).reshape(num_envs, -1).all(dim=1)
    robot_xyz_safe = torch.nan_to_num(robot_xyz, nan=0.0, posinf=0.0, neginf=0.0)

    beam_pos = robot_xyz_safe.mean(dim=1)
    beam_pos[:, 2] = torch.as_tensor(
        beam_z, dtype=beam_pos.dtype, device=beam_pos.device
    )

    support_axis = robot_xyz_safe[:, 1, :2] - robot_xyz_safe[:, 0, :2]
    yaw = torch.atan2(support_axis[:, 1], support_axis[:, 0])
    half_yaw = 0.5 * yaw
    beam_quat = torch.zeros(
        (num_envs, 4), dtype=robot_xyz.dtype, device=robot_xyz.device
    )
    beam_quat[:, 2] = torch.sin(half_yaw)
    beam_quat[:, 3] = torch.cos(half_yaw)

    finite_pose = (
        finite_support
        & torch.isfinite(beam_pos).all(dim=1)
        & torch.isfinite(beam_quat).all(dim=1)
    )
    if fallback_pos is not None and fallback_quat is not None:
        finite_fallback = torch.isfinite(fallback_pos).all(dim=1) & torch.isfinite(
            fallback_quat
        ).all(dim=1)
        use_fallback = (~finite_pose) & finite_fallback
        beam_pos = torch.where(use_fallback[:, None], fallback_pos, beam_pos)
        beam_quat = torch.where(use_fallback[:, None], fallback_quat, beam_quat)

    beam_pos = torch.nan_to_num(beam_pos, nan=0.0, posinf=0.0, neginf=0.0)
    beam_quat = torch.nan_to_num(beam_quat, nan=0.0, posinf=0.0, neginf=0.0)
    invalid_quat = torch.linalg.norm(beam_quat, dim=1) <= 1.0e-6
    if torch.any(invalid_quat):
        identity = torch.tensor(
            (0.0, 0.0, 0.0, 1.0), dtype=beam_quat.dtype, device=beam_quat.device
        )
        beam_quat[invalid_quat] = identity
    return beam_pos, beam_quat


def compute_two_robot_beam_support_error(
    *,
    robot_xy: torch.Tensor,
    beam_xy: torch.Tensor,
    beam_yaw: torch.Tensor,
    half_length: float,
) -> torch.Tensor:
    """Assignment-invariant mean distance to two beam support slots."""
    if robot_xy.ndim != 3 or robot_xy.shape[1:] != (2, 2):
        raise ValueError("robot_xy must have shape (num_envs, 2, 2)")
    if beam_xy.shape != (robot_xy.shape[0], 2):
        raise ValueError("beam_xy must have shape (num_envs, 2)")
    if beam_yaw.shape != (robot_xy.shape[0],):
        raise ValueError("beam_yaw must have shape (num_envs,)")
    if half_length <= 0.0:
        raise ValueError("half_length must be positive")

    forward = torch.stack((torch.cos(beam_yaw), torch.sin(beam_yaw)), dim=-1)
    offset = forward * float(half_length)
    slots = torch.stack((beam_xy - offset, beam_xy + offset), dim=1)

    direct = torch.linalg.norm(robot_xy - slots, dim=-1).mean(dim=1)
    swapped = torch.linalg.norm(robot_xy - slots.flip(dims=(1,)), dim=-1).mean(dim=1)
    return torch.minimum(direct, swapped)
