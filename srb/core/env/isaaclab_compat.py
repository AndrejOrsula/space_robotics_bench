from __future__ import annotations

from importlib import import_module
from typing import Any


def _missing_isaaclab_env_class(name: str) -> type:
    class MissingIsaacLabEnv:
        def __init_subclass__(cls, **kwargs: Any) -> None:
            raise RuntimeError(f"{name} requires Isaac Lab to be installed.")

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise RuntimeError(f"{name} requires Isaac Lab to be installed.")

    MissingIsaacLabEnv.__module__ = __name__
    MissingIsaacLabEnv.__name__ = name
    MissingIsaacLabEnv.__qualname__ = name
    return MissingIsaacLabEnv


try:
    _isaaclab_envs = import_module("isaaclab.envs")
    if getattr(_isaaclab_envs, "__file__", None) is None and (
        not hasattr(_isaaclab_envs, "DirectRLEnv")
        or not hasattr(_isaaclab_envs, "ManagerBasedRLEnv")
    ):
        raise ModuleNotFoundError("No real isaaclab.envs module found")
    IsaacLabDirectRLEnv = _isaaclab_envs.DirectRLEnv
    IsaacLabManagerBasedRLEnv = _isaaclab_envs.ManagerBasedRLEnv
except ModuleNotFoundError as exc:
    if exc.name is not None and not exc.name.startswith("isaaclab"):
        raise
    IsaacLabDirectRLEnv = _missing_isaaclab_env_class("IsaacLabDirectRLEnv")
    IsaacLabManagerBasedRLEnv = _missing_isaaclab_env_class("IsaacLabManagerBasedRLEnv")


__all__ = ["IsaacLabDirectRLEnv", "IsaacLabManagerBasedRLEnv"]
