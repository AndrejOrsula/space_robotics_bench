from typing import Any, Mapping, Set

from isaaclab.utils.dict import *  # noqa: F403
from isaaclab.utils.dict import string_to_slice


def replace_slices_with_strings(data: dict[Any, Any]) -> dict[Any, Any]:
    """Recursively replace slices with their string representations."""
    if isinstance(data, Mapping):
        return {k: replace_slices_with_strings(v) for k, v in data.items()}
    if isinstance(data, tuple):
        return tuple(replace_slices_with_strings(v) for v in data)
    if isinstance(data, list):
        return [replace_slices_with_strings(v) for v in data]
    if isinstance(data, Set):
        return {replace_slices_with_strings(v) for v in data}
    if isinstance(data, slice):
        return f"slice({data.start},{data.stop},{data.step})"
    return data


def replace_strings_with_slices(data: dict[Any, Any]) -> dict[Any, Any]:
    """Recursively replace slice strings with slice objects."""
    if isinstance(data, Mapping):
        return {k: replace_strings_with_slices(v) for k, v in data.items()}
    if isinstance(data, tuple):
        return tuple(replace_strings_with_slices(v) for v in data)
    if isinstance(data, list):
        return [replace_strings_with_slices(v) for v in data]
    if isinstance(data, Set):
        return {replace_strings_with_slices(v) for v in data}
    if isinstance(data, str) and data.startswith("slice("):
        return string_to_slice(data)
    return data
