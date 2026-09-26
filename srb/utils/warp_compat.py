"""Compatibility helpers for Isaac Lab ``ProxyArray`` Warp interop."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

_LAUNCH_ARRAY_ARGUMENTS = {
    "inputs": 2,
    "outputs": 3,
    "adj_inputs": 4,
    "adj_outputs": 5,
}


def _unwrap_proxy_arrays(values: Any, proxy_array_type: type[Any]) -> Any:
    if values is None:
        return None
    return [
        value.warp if isinstance(value, proxy_array_type) else value for value in values
    ]


def install_proxy_array_launch_compat(
    warp_module: Any,
    proxy_array_type: type[Any],
) -> None:
    """Unwrap ProxyArrays in Warp launch argument sequences, once per process."""
    original: Callable[..., Any] = warp_module.launch
    if getattr(original, "_srb_proxy_array_compat_applied", False):
        return

    def proxy_array_safe_launch(*args: Any, **kwargs: Any) -> Any:
        positional = list(args)
        for name, index in _LAUNCH_ARRAY_ARGUMENTS.items():
            if name in kwargs:
                kwargs[name] = _unwrap_proxy_arrays(kwargs[name], proxy_array_type)
            elif len(positional) > index:
                positional[index] = _unwrap_proxy_arrays(
                    positional[index], proxy_array_type
                )
        return original(*positional, **kwargs)

    proxy_array_safe_launch._srb_proxy_array_compat_applied = True  # type: ignore[attr-defined]
    warp_module.launch = proxy_array_safe_launch


__all__ = ["install_proxy_array_launch_compat"]
