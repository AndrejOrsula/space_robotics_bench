from typing import Any


def get_wrapper_attr(env: Any, name: str, default: Any = None) -> Any:
    getter = getattr(env, "get_wrapper_attr", None)
    if callable(getter):
        try:
            return getter(name)
        except AttributeError:
            return default
    return getattr(env, name, default)
