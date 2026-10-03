"""Isaac-free torch filters for smoothing batched or deployment actions.

Filters accept arbitrary leading dimensions and share the same math in sim
and deployment. Stateful steps return cloned state, preserving autograd.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

__all__ = [
    "CompositeFilter",
    "CriticallyDampedFilter",
    "DeadbandGate",
    "JerkLimiter",
    "RateLimiter",
    "filter_chain",
]


class CriticallyDampedFilter:
    """2nd-order critically-damped tracker.

    Tracks a target signal with no overshoot (zeta=1). The time constant
    ``tau_s`` sets the natural frequency: a zero-state step reaches about 26%
    at ``tau_s``, 80% at ``3*tau_s``, and 96% at ``5*tau_s``.
    Each step uses the exact zero-order-hold solution for a constant target
    over ``dt_s``. Unlike explicit Euler, this remains bounded when the control
    period is comparable to ``tau_s``.
    """

    def __init__(
        self,
        shape: tuple[int, ...],
        tau_s: float,
        dt_s: float,
        device: torch.device | str = "cpu",
    ) -> None:
        self.shape = shape
        self.tau_s = float(tau_s)
        self.dt_s = float(dt_s)
        if not math.isfinite(self.tau_s) or self.tau_s <= 0.0:
            raise ValueError(f"tau_s must be finite and positive, got {self.tau_s}")
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError(f"dt_s must be finite and positive, got {self.dt_s}")
        self.device = (
            torch.device(device) if not isinstance(device, torch.device) else device
        )
        self._wn = 1.0 / self.tau_s
        self._decay = math.exp(-self._wn * self.dt_s)
        self.pos = torch.zeros(shape, device=self.device)
        self.vel = torch.zeros(shape, device=self.device)

    def reset(self, env_ids, init_pos: torch.Tensor) -> None:
        """Set position state; zero velocity. Supports slice or index tensor."""
        env_ids = slice(None) if env_ids is None else env_ids
        self.pos[env_ids] = init_pos
        self.vel[env_ids] = 0.0

    def step(self, target: torch.Tensor) -> torch.Tensor:
        """Advance one timestep toward `target`; return updated position."""
        # Exact solution of e'' + 2*wn*e' + wn^2*e = 0 over one step, where
        # e = pos - target and the target is held constant during the step.
        error = self.pos - target
        common = self.vel + self._wn * error
        next_error = (error + common * self.dt_s) * self._decay
        self.vel = (self.vel - self._wn * common * self.dt_s) * self._decay
        self.pos = target + next_error
        # Keep returned storage caller-owned while preserving autograd history.
        return self.pos.clone()


class RateLimiter:
    """Clamps |Δpos| per call to `max_rate_per_step`.

    Prevents step-changes larger than the configured rate, regardless of
    how large the requested target jump is. State is just the last output.
    """

    def __init__(
        self,
        shape: tuple[int, ...],
        max_rate_per_step: float | torch.Tensor,
        device: torch.device | str = "cpu",
    ) -> None:
        self.shape = shape
        self.device = (
            torch.device(device) if not isinstance(device, torch.device) else device
        )
        self.max_rate = torch.as_tensor(
            max_rate_per_step, device=self.device, dtype=torch.float32
        )
        if not bool(torch.isfinite(self.max_rate).all()) or bool(
            (self.max_rate < 0.0).any()
        ):
            raise ValueError(
                "max_rate_per_step must contain finite non-negative values"
            )
        self.pos = torch.zeros(shape, device=self.device)

    def reset(self, env_ids, init_pos: torch.Tensor) -> None:
        env_ids = slice(None) if env_ids is None else env_ids
        self.pos[env_ids] = init_pos

    def step(self, target: torch.Tensor) -> torch.Tensor:
        delta = (target - self.pos).clamp(min=-self.max_rate, max=self.max_rate)
        self.pos = self.pos + delta
        # Keep returned storage caller-owned while preserving autograd history.
        return self.pos.clone()


class JerkLimiter:
    """Braking-aware acceleration limiter; despite the name, it limits the
    second derivative, not jerk. ``max_jerk_per_step`` sets a per-step
    acceleration budget of ``max_jerk_per_step * dt``. Names remain stable for
    API/config compatibility; ``clip_frequency`` reports the per-env fraction
    of steps where that budget clipped acceleration.

    Desired velocity uses the discrete stopping distance. A float-resolution
    reserve reduces rounding errors while braking toward a held target. Output
    stays between its previous value and the current target. If a moving target
    makes stopping infeasible, output stops at the target or its prior value;
    this can exceed the acceleration budget while only reducing velocity.
    """

    def __init__(
        self,
        shape: tuple[int, ...],
        max_jerk_per_step: float | torch.Tensor,
        dt_s: float,
        device: torch.device | str = "cpu",
    ) -> None:
        self.shape = shape
        self.dt_s = float(dt_s)
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError(f"dt_s must be finite and positive, got {self.dt_s}")
        self.device = (
            torch.device(device) if not isinstance(device, torch.device) else device
        )
        self.max_jerk = torch.as_tensor(
            max_jerk_per_step, device=self.device, dtype=torch.float32
        )
        if not bool(torch.isfinite(self.max_jerk).all()) or bool(
            (self.max_jerk < 0.0).any()
        ):
            raise ValueError(
                "max_jerk_per_step must contain finite non-negative values"
            )
        self.pos = torch.zeros(shape, device=self.device)
        self.vel = torch.zeros(shape, device=self.device)
        # Track clip statistics per leading (batch/env) dimension.
        self._clip_count = torch.zeros(shape[:-1], device=self.device)
        self._step_count = torch.zeros(shape[:-1], device=self.device)

    def reset(
        self,
        env_ids,
        init_pos: torch.Tensor,
        init_vel: torch.Tensor | None = None,
    ) -> None:
        env_ids = slice(None) if env_ids is None else env_ids
        self.pos[env_ids] = init_pos
        self.vel[env_ids] = 0.0 if init_vel is None else init_vel
        # Preserve clip statistics for envs that were not reset.
        self._clip_count[env_ids] = 0.0
        self._step_count[env_ids] = 0.0

    def step(self, target: torch.Tensor) -> torch.Tensor:
        dt = self.dt_s
        max_accel = self.max_jerk * dt  # second-diff budget per step
        error = target - self.pos
        distance = error.abs()
        # Reserve a few output float resolutions (at most half the budget) to
        # keep rounding from exceeding the acceleration budget while braking.
        resolution = torch.finfo(error.dtype).eps * (
            self.pos.abs() + target.abs() + self.vel.abs() * dt
        )
        reserve = torch.minimum(4.0 * resolution / (dt * dt), 0.5 * max_accel)
        plan_accel = max_accel - reserve
        # Largest per-step speed v with a discrete stopping distance
        # dt * sum_i max(v - i*a*dt, 0) <= distance at the plan acceleration a.
        # With E = distance / (a * dt**2) and k = floor((sqrt(1 + 8E) - 1) / 2)
        # full decrements, v = distance / (dt * (k + 1)) + a * dt * k / 2.
        # k = 0 gives distance / dt: reach the target in one step when feasible.
        # The tiny floor and cap keep a zero budget finite (the clamp below then
        # holds the velocity).
        step_budget = (plan_accel * dt * dt).clamp_min(torch.finfo(torch.float32).tiny)
        budget_steps = (distance / step_budget).clamp_max(1.0e30)
        full_decrements = torch.floor(
            (torch.sqrt(1.0 + 8.0 * budget_steps) - 1.0) * 0.5
        )
        stop_speed = distance / (dt * (full_decrements + 1.0))
        stop_speed = stop_speed + plan_accel * dt * full_decrements * 0.5
        # Allow a final step when the planned move is below float resolution.
        stop_speed = torch.maximum(stop_speed, torch.minimum(distance, resolution) / dt)
        desired_vel = torch.sign(error) * stop_speed
        accel = (desired_vel - self.vel) / dt
        clipped = accel.clamp(min=-max_accel, max=max_accel)
        # Per-env: did ANY action dim clip this step? Reduce only the last dim.
        self._clip_count = self._clip_count + (clipped != accel).any(dim=-1).float()
        self._step_count = self._step_count + 1.0
        vel = self.vel + clipped * dt
        candidate = self.pos + vel * dt
        # Never pass the target and never move away from it: the output stays
        # between the previous output and the current target.
        low = torch.minimum(self.pos, target)
        high = torch.maximum(self.pos, target)
        pos = torch.maximum(torch.minimum(candidate, high), low)
        self.vel = torch.where(pos != candidate, (pos - self.pos) / dt, vel)
        self.pos = pos
        # Keep returned storage caller-owned while preserving autograd history.
        return self.pos.clone()

    @property
    def clip_frequency(self) -> torch.Tensor:
        """Per-env fraction of steps where the jerk limit was active.

        Shape is ``self.shape[:-1]`` — ``(num_envs,)`` in sim, ``()`` scalar in
        single-env deployment.
        """
        return self._clip_count / torch.clamp_min(self._step_count, 1.0)


class DeadbandGate:
    """Zero commands whose absolute value is below ``threshold``."""

    def __init__(self, threshold: float | torch.Tensor) -> None:
        self.threshold = torch.as_tensor(threshold, dtype=torch.float32)
        if not bool(torch.isfinite(self.threshold).all()) or bool(
            (self.threshold < 0.0).any()
        ):
            raise ValueError("threshold must contain finite non-negative values")

    def reset(self, _env_ids, _init_pos: torch.Tensor) -> None:
        pass  # Stateless; nothing to reset.

    def step(self, target: torch.Tensor) -> torch.Tensor:
        return torch.where(
            target.abs() < self.threshold.to(target.device),
            torch.zeros_like(target),
            target,
        )


class CompositeFilter:
    """Apply filters in order, passing each output to the next input."""

    def __init__(self, filters: Sequence) -> None:
        self.filters = list(filters)

    def reset(self, env_ids, init_pos: torch.Tensor) -> None:
        for f in self.filters:
            if hasattr(f, "reset"):
                f.reset(env_ids, init_pos)

    def step(self, target: torch.Tensor) -> torch.Tensor:
        x = target
        for f in self.filters:
            x = f.step(x)
        return x


def filter_chain(filters: Sequence) -> CompositeFilter:
    """Wrap a list of filters into a single composable filter."""
    return CompositeFilter(filters)
