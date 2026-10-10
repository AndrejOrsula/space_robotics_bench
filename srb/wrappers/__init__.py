from importlib import import_module
from typing import TYPE_CHECKING, Any

import gymnasium.wrappers as _gym_wrappers
from gymnasium.wrappers import RecordVideo

_LOCAL_EXPORTS = {
    "ActionSmoothingCfg": ("action_smoothing", "ActionSmoothingCfg"),
    "ActionSmoothingWrapper": ("action_smoothing", "ActionSmoothingWrapper"),
    "MergeDictObsWrapper": ("merge_dict_obs", "MergeDictObsWrapper"),
    "ObservationStackingCfg": ("observation_stacking", "ObservationStackingCfg"),
    "ObservationStackingWrapper": (
        "observation_stacking",
        "ObservationStackingWrapper",
    ),
    "ReplicatorRenderShim": ("replicator_render", "ReplicatorRenderShim"),
    "RolloutRecordVideo": ("record_video", "RolloutRecordVideo"),
    "SmoothingMethod": ("action_smoothing", "SmoothingMethod"),
    "TuneReporterWrapper": ("tune_reporter", "TuneReporterWrapper"),
}

if TYPE_CHECKING:
    from .action_smoothing import ActionSmoothingCfg as ActionSmoothingCfg
    from .action_smoothing import ActionSmoothingWrapper as ActionSmoothingWrapper
    from .action_smoothing import SmoothingMethod as SmoothingMethod
    from .merge_dict_obs import MergeDictObsWrapper as MergeDictObsWrapper
    from .observation_stacking import (
        ObservationStackingCfg as ObservationStackingCfg,
    )
    from .observation_stacking import (
        ObservationStackingWrapper as ObservationStackingWrapper,
    )
    from .record_video import RolloutRecordVideo as RolloutRecordVideo
    from .replicator_render import ReplicatorRenderShim as ReplicatorRenderShim
    from .tune_reporter import TuneReporterWrapper as TuneReporterWrapper


def __getattr__(name: str) -> Any:
    """Load feature wrappers and Gymnasium re-exports only when requested."""
    local = _LOCAL_EXPORTS.get(name)
    if local is not None:
        module_name, attribute = local
        value = getattr(import_module(f".{module_name}", __name__), attribute)
        globals()[name] = value
        return value
    return getattr(_gym_wrappers, name)


__all__ = sorted(
    {
        *getattr(_gym_wrappers, "__all__", ()),
        "ActionSmoothingCfg",
        "ActionSmoothingWrapper",
        "MergeDictObsWrapper",
        "ObservationStackingCfg",
        "ObservationStackingWrapper",
        "RecordVideo",
        "ReplicatorRenderShim",
        "RolloutRecordVideo",
        "SmoothingMethod",
        "TuneReporterWrapper",
    }
)
