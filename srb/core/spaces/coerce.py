from __future__ import annotations

from typing import Any, Literal

import numpy as np
import torch
from gymnasium import spaces

from srb.core.spaces.walk import apply_to_obs

CoerceTarget = Literal["torch", "numpy", "cpu_tensor"]


def to_torch(
    value: Any,
    *,
    device: torch.device,
    dtype: torch.dtype | None = None,
) -> torch.Tensor:
    if isinstance(value, torch.Tensor):
        if dtype is not None and value.dtype != dtype:
            value = value.to(dtype=dtype)
        if value.device != device:
            value = value.to(device=device)
        return value
    tensor = torch.as_tensor(value, device=device)
    if dtype is not None and tensor.dtype != dtype:
        tensor = tensor.to(dtype=dtype)
    return tensor


def to_numpy(value: Any) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    if isinstance(value, np.ndarray):
        return value
    return np.asarray(value)


def coerce_obs(
    obs: Any,
    space: spaces.Space,
    *,
    to: CoerceTarget,
    device: torch.device | None = None,
) -> Any:
    if to == "torch" and device is None:
        raise ValueError("coerce_obs(to='torch') requires `device`.")

    def _leaf(leaf_obs: Any, _leaf_space: spaces.Space) -> Any:
        if to == "numpy":
            return to_numpy(leaf_obs)
        if to == "cpu_tensor":
            if isinstance(leaf_obs, torch.Tensor):
                return leaf_obs.detach().cpu()
            return torch.as_tensor(leaf_obs).cpu()
        assert device is not None
        return to_torch(leaf_obs, device=device)

    return apply_to_obs(obs, space, leaf_fn=_leaf)


#: Wrappers combine these flags with ``terminated | truncated``, which requires
#: boolean dtypes in both NumPy and Torch.
_FLAG_DTYPES: tuple[np.dtype[Any], torch.dtype] = (np.dtype(np.bool_), torch.bool)

#: Dreamer's declared reward space is float32; use the same dtype for bare
#: Python scalar rewards while preserving array and tensor dtypes.
_SCALAR_REWARD_DTYPE: np.dtype[Any] = np.dtype(np.float32)


def coerce_gym_step_outputs(
    reward: Any,
    terminated: Any,
    truncated: Any,
    *,
    output: Literal["numpy", "cpu_tensor"],
) -> tuple[Any, Any, Any]:
    return (
        _coerce_step_reward(reward, output=output),
        _coerce_step_flag(terminated, output=output),
        _coerce_step_flag(truncated, output=output),
    )


def _coerce_step_reward(
    value: Any,
    *,
    output: Literal["numpy", "cpu_tensor"],
) -> Any:
    """Keep reward dtype; ``output`` selects representation only for Torch inputs.

    Arrays pass through by identity; Python scalars become NumPy in both modes.
    """
    if isinstance(value, torch.Tensor):
        cpu = value.detach().cpu()
        return cpu.numpy() if output == "numpy" else cpu
    if isinstance(value, np.ndarray):
        return value
    return np.array((value,), dtype=_SCALAR_REWARD_DTYPE)


def _coerce_step_flag(
    value: Any,
    *,
    output: Literal["numpy", "cpu_tensor"],
) -> Any:
    """Return a boolean flag in ``output``'s representation.

    Every input is normalized because wrappers combine flags with ``|``.
    Boolean ndarrays pass through by identity; other arrays are converted.
    """
    np_dtype, torch_dtype = _FLAG_DTYPES
    if isinstance(value, torch.Tensor):
        cpu = value.detach().cpu()
        if cpu.dtype != torch_dtype:
            cpu = cpu.to(torch_dtype)
        return cpu.numpy() if output == "numpy" else cpu
    if isinstance(value, np.ndarray):
        return value if value.dtype == np_dtype else value.astype(np_dtype)
    return np.array((value,), dtype=np_dtype)
