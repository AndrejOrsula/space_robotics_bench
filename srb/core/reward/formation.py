import torch


def compute_mean_planar_formation_progress(
    *,
    previous_dist2d: torch.Tensor,
    current_dist2d: torch.Tensor,
    step_dt: float,
) -> torch.Tensor:
    """Signed mean slot-distance reduction rate per environment."""
    if previous_dist2d.ndim != 2:
        raise ValueError("previous_dist2d must have shape (num_envs, num_robots)")
    if current_dist2d.shape != previous_dist2d.shape:
        raise ValueError("current_dist2d must match previous_dist2d shape")
    if step_dt <= 0.0:
        raise ValueError("step_dt must be positive")

    return torch.mean((previous_dist2d - current_dist2d) / float(step_dt), dim=1)


def compute_mean_planar_velocity_tracking(
    *,
    robot_vel_xy: torch.Tensor,
    target_vel_xy: torch.Tensor,
    std: float,
) -> torch.Tensor:
    """Mean per-robot planar velocity tracking score in [0, 1]."""
    if robot_vel_xy.ndim != 3 or robot_vel_xy.shape[-1] != 2:
        raise ValueError("robot_vel_xy must have shape (num_envs, num_robots, 2)")
    if target_vel_xy.shape != (robot_vel_xy.shape[0], 2):
        raise ValueError("target_vel_xy must have shape (num_envs, 2)")
    if std <= 0.0:
        raise ValueError("std must be positive")

    velocity_error = torch.linalg.norm(
        robot_vel_xy - target_vel_xy.unsqueeze(1),
        dim=-1,
    )
    return torch.mean(1.0 - torch.tanh(velocity_error / float(std)), dim=1)
