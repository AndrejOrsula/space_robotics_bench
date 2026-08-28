from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any

_EXPORTS: dict[str, tuple[str, str | None]] = {
    "ActionType": ("isaaclab.envs.common", "ActionType"),
    "AgentID": ("isaaclab.envs.common", "AgentID"),
    "EnvStepReturn": ("isaaclab.envs.common", "EnvStepReturn"),
    "ObsType": ("isaaclab.envs.common", "ObsType"),
    "SpaceType": ("isaaclab.envs.common", "SpaceType"),
    "StateType": ("isaaclab.envs.common", "StateType"),
    "VecEnvObs": ("isaaclab.envs.common", "VecEnvObs"),
    "VecEnvStepReturn": ("isaaclab.envs.common", "VecEnvStepReturn"),
    "ObservationGroup": ("srb._typing.observation_group", None),
    "AnyEnv": ("srb._typing.cfg", "AnyEnv"),
    "AnyEnvCfg": ("srb._typing.cfg", "AnyEnvCfg"),
    "EnumNameSerializer": ("srb._typing.enum_utils", "EnumNameSerializer"),
    "StepReturn": ("srb._typing.step_return", "StepReturn"),
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attr_name = _EXPORTS[name]
    module = import_module(module_name)
    value = module if attr_name is None else getattr(module, attr_name)
    globals()[name] = value
    return value


if TYPE_CHECKING:
    from isaaclab.envs import common as _isaac_common

    from srb._typing import observation_group as _observation_group
    from srb._typing.cfg import AnyEnv as _AnyEnv
    from srb._typing.cfg import AnyEnvCfg as _AnyEnvCfg
    from srb._typing.enum_utils import EnumNameSerializer as _EnumNameSerializer
    from srb._typing.step_return import StepReturn as _StepReturn

    ActionType = _isaac_common.ActionType
    AgentID = _isaac_common.AgentID
    EnvStepReturn = _isaac_common.EnvStepReturn
    ObsType = _isaac_common.ObsType
    SpaceType = _isaac_common.SpaceType
    StateType = _isaac_common.StateType
    VecEnvObs = _isaac_common.VecEnvObs
    VecEnvStepReturn = _isaac_common.VecEnvStepReturn

    ObservationGroup = _observation_group
    AnyEnv = _AnyEnv
    AnyEnvCfg = _AnyEnvCfg
    EnumNameSerializer = _EnumNameSerializer
    StepReturn = _StepReturn
