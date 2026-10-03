import math
import re
from collections.abc import Sequence
from typing import Any, NamedTuple

import torch


def compute_locomotion_safety_termination(
    *,
    undesired_contact_force: torch.Tensor,
    projected_gravity_b: torch.Tensor,
    contact_force_threshold: float,
    max_projected_gravity_xy: float,
    terminate_on_undesired_contacts: bool = True,
    terminate_on_fall: bool = True,
) -> torch.Tensor:
    """Return a fail-closed per-environment locomotion termination mask."""
    if undesired_contact_force.ndim != 1:
        raise ValueError("undesired_contact_force must have shape (num_envs,)")
    if projected_gravity_b.ndim != 2 or projected_gravity_b.shape[1] != 3:
        raise ValueError("projected_gravity_b must have shape (num_envs, 3)")
    if projected_gravity_b.shape[0] != undesired_contact_force.shape[0]:
        raise ValueError("contact force and projected gravity batch sizes must match")
    if projected_gravity_b.device != undesired_contact_force.device:
        raise ValueError("contact force and projected gravity devices must match")
    if not math.isfinite(contact_force_threshold) or contact_force_threshold < 0.0:
        raise ValueError("contact_force_threshold must be finite and non-negative")
    if not math.isfinite(max_projected_gravity_xy) or not (
        0.0 < max_projected_gravity_xy <= 1.0
    ):
        raise ValueError("max_projected_gravity_xy must be in (0, 1]")

    termination = (
        ~torch.isfinite(undesired_contact_force)
        | (undesired_contact_force < 0.0)
        | ~torch.isfinite(projected_gravity_b).all(dim=1)
    )
    if terminate_on_undesired_contacts:
        termination |= undesired_contact_force > contact_force_threshold
    if terminate_on_fall:
        # XY magnitude alone shrinks back to zero once the base is upside down,
        # so also treat gravity pointing along body +Z (tilt > 90 deg) as a fall.
        termination |= (
            torch.linalg.vector_norm(projected_gravity_b[:, :2], dim=1)
            > max_projected_gravity_xy
        ) | (projected_gravity_b[:, 2] > 0.0)
    return termination


def _moving_mask(
    command_xy: torch.Tensor,
    body_xy_vel: torch.Tensor,
    velocity_threshold: float,
) -> torch.Tensor:
    command_speed = torch.linalg.norm(command_xy[..., :2], dim=-1)
    body_speed = torch.linalg.norm(body_xy_vel[..., :2], dim=-1)
    return (command_speed > 1.0e-6) | (body_speed > velocity_threshold)


def compute_stance_air_time_metric(
    *,
    current_air_time: torch.Tensor,
    current_contact_time: torch.Tensor,
    command_xy: torch.Tensor,
    body_xy_vel: torch.Tensor,
    mode_time: float,
    velocity_threshold: float,
) -> torch.Tensor:
    """Continuous foot-timing score for legged locomotion."""
    if current_air_time.shape != current_contact_time.shape:
        raise ValueError("current_air_time and current_contact_time shapes must match")
    if mode_time <= 0.0:
        raise ValueError("mode_time must be positive")
    if current_air_time.shape[1] == 0:
        return torch.zeros(current_air_time.shape[0], device=current_air_time.device)

    phase_time = torch.maximum(current_air_time, current_contact_time)
    # Ignore phases at or above ``mode_time`` to avoid rewarding stalled feet.
    moving_score = (
        torch.where(
            phase_time < mode_time,
            phase_time.clamp(min=0.0),
            torch.zeros_like(phase_time),
        )
        / mode_time
    )
    # Keep nonnegative for use with ``LinearRewardTerm``.
    stance_score = (current_contact_time - current_air_time).clamp(
        min=0.0, max=mode_time
    ) / mode_time

    moving = _moving_mask(command_xy, body_xy_vel, velocity_threshold).unsqueeze(-1)
    return torch.where(moving, moving_score, stance_score).mean(dim=1)


def compute_quadruped_trot_metric(
    *,
    current_air_time: torch.Tensor,
    current_contact_time: torch.Tensor,
    synced_feet_pair_ids: tuple[tuple[int, int], tuple[int, int]] | None,
    command_xy: torch.Tensor,
    body_xy_vel: torch.Tensor,
    velocity_threshold: float,
    std: float = 0.1,
    max_err: float = 0.2,
) -> torch.Tensor:
    """Diagonal-pair gait score, zero when stationary or pair mapping unavailable."""
    if current_air_time.shape != current_contact_time.shape:
        raise ValueError("current_air_time and current_contact_time shapes must match")
    if std <= 0.0:
        raise ValueError("std must be positive")
    if max_err <= 0.0:
        raise ValueError("max_err must be positive")

    num_envs, num_feet = current_air_time.shape
    if synced_feet_pair_ids is None or num_feet < 4:
        return torch.zeros(num_envs, device=current_air_time.device)

    (sync_a, sync_b) = synced_feet_pair_ids
    ids = (*sync_a, *sync_b)
    if any(idx < 0 or idx >= num_feet for idx in ids):
        return torch.zeros(num_envs, device=current_air_time.device)

    max_sq = max_err**2

    def sync_reward(foot_a: int, foot_b: int) -> torch.Tensor:
        se_air = torch.square(
            current_air_time[:, foot_a] - current_air_time[:, foot_b]
        ).clamp(max=max_sq)
        se_contact = torch.square(
            current_contact_time[:, foot_a] - current_contact_time[:, foot_b]
        ).clamp(max=max_sq)
        return torch.exp(-(se_air + se_contact) / std)

    def async_reward(foot_a: int, foot_b: int) -> torch.Tensor:
        se_air_contact = torch.square(
            current_air_time[:, foot_a] - current_contact_time[:, foot_b]
        ).clamp(max=max_sq)
        se_contact_air = torch.square(
            current_contact_time[:, foot_a] - current_air_time[:, foot_b]
        ).clamp(max=max_sq)
        return torch.exp(-(se_air_contact + se_contact_air) / std)

    score = (
        sync_reward(sync_a[0], sync_a[1])
        * sync_reward(sync_b[0], sync_b[1])
        * async_reward(sync_a[0], sync_b[0])
        * async_reward(sync_a[1], sync_b[1])
        * async_reward(sync_a[0], sync_b[1])
        * async_reward(sync_b[0], sync_a[1])
    )
    moving = _moving_mask(command_xy, body_xy_vel, velocity_threshold)
    return torch.where(moving, score, torch.zeros_like(score))


def compute_foot_slip_metric(
    *,
    contact_forces_w: torch.Tensor,
    foot_lin_vel_w: torch.Tensor,
    contact_threshold: float,
) -> torch.Tensor:
    """Planar speed of contacting feet, averaged over all feet."""
    if contact_forces_w.shape[:2] != foot_lin_vel_w.shape[:2]:
        raise ValueError("contact force and foot velocity batch/foot shapes must match")
    if contact_forces_w.shape[1] == 0:
        return torch.zeros(contact_forces_w.shape[0], device=contact_forces_w.device)

    contact = torch.linalg.norm(contact_forces_w[..., :3], dim=-1) > contact_threshold
    planar_speed = torch.linalg.norm(foot_lin_vel_w[..., :2], dim=-1)
    return (contact * planar_speed).sum(dim=1) / float(contact_forces_w.shape[1])


class FeetContactIndices(NamedTuple):
    """Feet and non-foot indices for their respective arrays."""

    feet_body: tuple[int, ...]
    """Feet in articulation body order."""
    feet_sensor: tuple[int, ...]
    """Feet in contact-sensor order, matching ``feet_body``."""
    foot_names: tuple[str, ...]
    undesired_sensor: tuple[int, ...]
    """Other contact-sensor bodies, in sensor order."""


def resolve_feet_contact_indices(
    robot: Any, contact_sensor: Any, regex_feet_links: str | Sequence[str]
) -> FeetContactIndices:
    """Resolve feet in both arrays, preserving articulation order by name."""
    feet_body, foot_names = robot.find_bodies(regex_feet_links)
    feet_sensor, sensor_names = contact_sensor.find_sensors(
        [re.escape(name) for name in foot_names], preserve_order=True
    )
    if list(sensor_names) != list(foot_names):
        missing = sorted(set(foot_names) - set(sensor_names))
        raise ValueError(
            "contact sensor does not cover every foot body exactly once; "
            f"missing={missing}, sensor={list(sensor_names)}"
        )
    feet_sensor_set = set(feet_sensor)
    undesired_sensor = tuple(
        index
        for index in range(len(contact_sensor.body_names))
        if index not in feet_sensor_set
    )
    return FeetContactIndices(
        feet_body=tuple(feet_body),
        feet_sensor=tuple(feet_sensor),
        foot_names=tuple(foot_names),
        undesired_sensor=undesired_sensor,
    )


def find_quadruped_trot_pairs(
    body_names: Sequence[str],
) -> tuple[tuple[int, int], tuple[int, int]] | None:
    """Find diagonal trot pairs from common quadruped foot link names."""
    role_to_index: dict[str, int] = {}
    for index, name in enumerate(body_names):
        role = _classify_foot_name(name)
        if role and role not in role_to_index:
            role_to_index[role] = index

    required = ("left_front", "right_front", "left_hind", "right_hind")
    if any(role not in role_to_index for role in required):
        return None
    return (
        (role_to_index["left_front"], role_to_index["right_hind"]),
        (role_to_index["right_front"], role_to_index["left_hind"]),
    )


def _classify_foot_name(name: str) -> str | None:
    normalized = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    tokens = {token for token in normalized.split("_") if token}

    front_left = {"fl", "lf", "front_left", "left_front"}
    front_right = {"fr", "rf", "front_right", "right_front"}
    hind_left = {"hl", "lh", "rl", "hind_left", "left_hind", "rear_left"}
    hind_right = {"hr", "rh", "rr", "hind_right", "right_hind", "rear_right"}

    if tokens & front_left or "front_left" in normalized or "left_front" in normalized:
        return "left_front"
    if (
        tokens & front_right
        or "front_right" in normalized
        or "right_front" in normalized
    ):
        return "right_front"
    if tokens & hind_left or "hind_left" in normalized or "rear_left" in normalized:
        return "left_hind"
    if tokens & hind_right or "hind_right" in normalized or "rear_right" in normalized:
        return "right_hind"
    return None
