from __future__ import annotations

from typing import Any

import torch

from srb._typing.step_return import StepReturn

# Public re-export from the step-return runtime surface; keep noqa to retain it.
from .observation_layout import flatten_observations  # noqa: F401


def _to_torch(value, device=None):
    value = value.torch if hasattr(value, "torch") else value
    if device is not None and isinstance(value, torch.Tensor):
        # Device residency contract: batched step data lives on ``env.device``
        # until a framework/dataset/CLI/hardware seam moves it.
        return value.to(device=device)
    return value


def coerce_step_return_to_torch(
    step_return: StepReturn, *, device: torch.device | str | None = None
) -> StepReturn:
    observation = {
        category: {key: _to_torch(value, device) for key, value in group.items()}
        for category, group in step_return.observation.items()
    }
    reward = {
        key: _to_torch(value, device) for key, value in step_return.reward.items()
    }
    termination = _to_torch(step_return.termination, device)
    truncation = _to_torch(step_return.truncation, device)
    # `info` is outside the validated contract: unwrap values and move only
    # top-level tensors; nested mappings and scalars remain task-owned.
    info = (
        {key: _to_torch(value, device) for key, value in step_return.info.items()}
        if step_return.info is not None
        else None
    )
    return StepReturn(observation, reward, termination, truncation, info)


def truthy_any(value: Any) -> bool:
    """Return whether a boolean, tensor, array-like, iterable, or scalar is true."""
    if isinstance(value, bool):
        return value
    if hasattr(value, "detach"):
        return bool(value.detach().cpu().any().item())
    if hasattr(value, "any"):
        any_value = value.any()
        return bool(any_value.item() if hasattr(any_value, "item") else any_value)
    try:
        return any(bool(item) for item in value)
    except TypeError:
        return bool(value)


def _validate_step_return_shapes(step_return: StepReturn, *, num_envs: int) -> None:
    """Check metadata before reductions; these checks do not sync device work."""
    if not isinstance(num_envs, int) or isinstance(num_envs, bool) or num_envs <= 0:
        raise ValueError(f"num_envs must be a positive integer, got {num_envs!r}")

    for name, value in (
        ("Termination", step_return.termination),
        ("Truncation", step_return.truncation),
    ):
        if not isinstance(value, torch.Tensor):
            raise TypeError(f"{name} must be a torch.Tensor")
        if value.shape != (num_envs,):
            raise ValueError(
                f"{name} shape must be ({num_envs},), got {tuple(value.shape)}"
            )
        if value.dtype != torch.bool:
            raise TypeError(f"{name} dtype must be torch.bool, got {value.dtype}")

    for category, group in step_return.observation.items():
        for key, value in group.items():
            if not isinstance(value, torch.Tensor):
                raise TypeError(
                    f"Observation component '{category}/{key}' must be a torch.Tensor"
                )
            if value.ndim == 0 or value.shape[0] != num_envs:
                raise ValueError(
                    f"Observation component '{category}/{key}' first dimension "
                    f"must be {num_envs}, got {tuple(value.shape)}"
                )

    for key, value in step_return.reward.items():
        if not isinstance(value, torch.Tensor):
            raise TypeError(f"Reward component '{key}' must be a torch.Tensor")
        if value.shape != (num_envs,):
            raise ValueError(
                f"Reward component '{key}' shape must be ({num_envs},), "
                f"got {tuple(value.shape)}"
            )


def validate_step_return(
    step_return: StepReturn,
    *,
    num_envs: int,
    device: torch.device | str | None = None,
    assume_finite: bool = False,
) -> None:
    """Validate batched tensors; ``info`` remains an unvalidated diagnostic channel.

    ``device`` enforces residency until an adapter moves tensors. Set
    ``assume_finite`` only after scrubbing the same return to avoid repeated
    host synchronizations.
    """
    _validate_step_return_shapes(step_return, num_envs=num_envs)
    expected_device = torch.device(device) if device is not None else None

    def _check_device(name: str, value: torch.Tensor) -> None:
        if expected_device is None:
            return
        if value.device.type != expected_device.type or (
            expected_device.index is not None
            and value.device.index not in (None, expected_device.index)
        ):
            raise ValueError(
                f"{name} must reside on {expected_device}, got {value.device}"
            )

    _check_device("Termination", step_return.termination)
    _check_device("Truncation", step_return.truncation)

    for category, group in step_return.observation.items():
        for key, value in group.items():
            if not assume_finite and not bool(torch.isfinite(value).all()):
                raise ValueError(
                    f"Observation component '{category}/{key}' contains "
                    "non-finite values"
                )
            _check_device(f"Observation component '{category}/{key}'", value)

    for key, value in step_return.reward.items():
        if not assume_finite and not bool(torch.isfinite(value).all()):
            raise ValueError(f"Reward component '{key}' contains non-finite values")
        _check_device(f"Reward component '{key}'", value)


def _invalid_env_mask(value: torch.Tensor, *, num_envs: int) -> torch.Tensor:
    """Return one bit per environment, true for rows with non-finite values."""
    finite = torch.isfinite(value)
    if value.ndim == 1:
        return ~finite
    return ~finite.reshape(num_envs, -1).all(dim=1)


def _scrub_by_env_mask(value: torch.Tensor, invalid_envs: torch.Tensor) -> torch.Tensor:
    scrubbed = value.clone()
    mask = invalid_envs.view(value.shape[0], *((1,) * (value.ndim - 1)))
    return torch.where(mask, torch.zeros_like(scrubbed), scrubbed)


def scrub_non_finite_step_return(
    step_return: StepReturn, *, num_envs: int, device: torch.device | str
) -> tuple[StepReturn, torch.Tensor, list[str]]:
    """Zero each environment with non-finite observations or rewards.

    When components exist, reduce masks on-device and transfer component verdicts
    with one ``.tolist()`` host sync, regardless of component count. Validate original
    shapes before reductions; metadata checks do not synchronize.
    """
    _validate_step_return_shapes(step_return, num_envs=num_envs)

    # Preserve diagnostic order: observation components, then reward terms.
    entries: list[
        tuple[str, dict[str, torch.Tensor], str, torch.Tensor, torch.Tensor]
    ] = []
    masks: list[torch.Tensor] = []

    def _record(
        name: str, container: dict[str, torch.Tensor], key: str, value: torch.Tensor
    ) -> None:
        container[key] = value
        # Shape validation guarantees `(num_envs,)`, so all masks share one
        # stacked reduction without per-component synchronization.
        mask = _invalid_env_mask(value, num_envs=num_envs).to(device=device)
        entries.append((name, container, key, value, mask))
        masks.append(mask)

    observation: dict[str, dict[str, torch.Tensor]] = {}
    for category, group in step_return.observation.items():
        observation[category] = {}
        for key, value in group.items():
            _record(f"{category}/{key}", observation[category], key, value)

    reward: dict[str, torch.Tensor] = {}
    for key, value in step_return.reward.items():
        _record(key, reward, key, value)

    invalid_keys: list[str] = []
    if masks:
        # (num_components, num_envs) -> one reduction along each axis, then a
        # single transfer of the per-component verdict.
        stacked = torch.stack(masks)
        invalid_envs = stacked.any(dim=0)
        verdicts = iter(stacked.any(dim=1).tolist())
    else:
        invalid_envs = torch.zeros(num_envs, dtype=torch.bool, device=device)
        verdicts = iter(())

    for name, container, key, value, mask in entries:
        if not next(verdicts):
            continue
        invalid_keys.append(name)
        container[key] = _scrub_by_env_mask(value, mask.to(device=value.device))

    termination = step_return.termination.clone()
    termination |= invalid_envs.to(device=termination.device)

    return (
        StepReturn(
            observation,
            reward,
            termination,
            step_return.truncation,
            step_return.info,
        ),
        invalid_envs,
        invalid_keys,
    )
