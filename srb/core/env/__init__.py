from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any

_EXPORTS = {
    "ViewerCfg": "isaaclab.envs",
    "IsaacLabDirectRLEnv": "srb.core.env.isaaclab_compat",
    "IsaacLabManagerBasedRLEnv": "srb.core.env.isaaclab_compat",
    "InteractiveScene": "isaaclab.scene",
    "InteractiveSceneCfg": "isaaclab.scene",
    "BaseEnvCfg": "srb.core.env.common",
    "BaseEventCfg": "srb.core.env.common",
    "BaseSceneCfg": "srb.core.env.common",
    "DirectEnv": "srb.core.env.common",
    "DirectEnvCfg": "srb.core.env.common",
    "DirectMarlEnv": "srb.core.env.common",
    "DirectMarlEnvCfg": "srb.core.env.common",
    "ManagedEnv": "srb.core.env.common",
    "ManagedEnvCfg": "srb.core.env.common",
    "assert_privileged_absent_from_onnx": "srb.core.env.privileged",
    "onnx_export_input_whitelist": "srb.core.env.privileged",
    "ManipulationEnv": "srb.core.env.manipulation",
    "ManipulationEnvCfg": "srb.core.env.manipulation",
    "ManipulationEventCfg": "srb.core.env.manipulation",
    "ManipulationRewardCfg": "srb.core.env.manipulation",
    "ManipulationSceneCfg": "srb.core.env.manipulation",
    "AerialEnv": "srb.core.env.mobile",
    "AerialEnvCfg": "srb.core.env.mobile",
    "AerialEventCfg": "srb.core.env.mobile",
    "AerialMobileRewardCfg": "srb.core.env.mobile",
    "AerialSceneCfg": "srb.core.env.mobile",
    "GroundEnv": "srb.core.env.mobile",
    "GroundEnvCfg": "srb.core.env.mobile",
    "GroundEventCfg": "srb.core.env.mobile",
    "GroundMobileRewardCfg": "srb.core.env.mobile",
    "GroundSceneCfg": "srb.core.env.mobile",
    "OrbitalEnv": "srb.core.env.mobile",
    "OrbitalEnvCfg": "srb.core.env.mobile",
    "OrbitalEventCfg": "srb.core.env.mobile",
    "OrbitalMobileRewardCfg": "srb.core.env.mobile",
    "OrbitalSceneCfg": "srb.core.env.mobile",
    "AerialManipulationEnv": "srb.core.env.mobile_manipulation",
    "AerialManipulationEnvCfg": "srb.core.env.mobile_manipulation",
    "AerialManipulationEventCfg": "srb.core.env.mobile_manipulation",
    "AerialManipulationRewardCfg": "srb.core.env.mobile_manipulation",
    "AerialManipulationSceneCfg": "srb.core.env.mobile_manipulation",
    "GroundManipulationEnv": "srb.core.env.mobile_manipulation",
    "GroundManipulationEnvCfg": "srb.core.env.mobile_manipulation",
    "GroundManipulationEventCfg": "srb.core.env.mobile_manipulation",
    "GroundManipulationRewardCfg": "srb.core.env.mobile_manipulation",
    "GroundManipulationSceneCfg": "srb.core.env.mobile_manipulation",
    "OrbitalManipulationEnv": "srb.core.env.mobile_manipulation",
    "OrbitalManipulationEnvCfg": "srb.core.env.mobile_manipulation",
    "OrbitalManipulationEventCfg": "srb.core.env.mobile_manipulation",
    "OrbitalManipulationRewardCfg": "srb.core.env.mobile_manipulation",
    "OrbitalManipulationSceneCfg": "srb.core.env.mobile_manipulation",
    "SrbEnv": "srb.core.env.srb_env",
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(_EXPORTS[name])
    value = getattr(module, name)
    globals()[name] = value
    return value


if TYPE_CHECKING:
    from isaaclab.envs import ViewerCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg

    from srb.core.env.common import (
        BaseEnvCfg,
        BaseEventCfg,
        BaseSceneCfg,
        DirectEnv,
        DirectEnvCfg,
        DirectMarlEnv,
        DirectMarlEnvCfg,
        ManagedEnv,
        ManagedEnvCfg,
    )
    from srb.core.env.isaaclab_compat import (
        IsaacLabDirectRLEnv,
        IsaacLabManagerBasedRLEnv,
    )
    from srb.core.env.manipulation import (
        ManipulationEnv,
        ManipulationEnvCfg,
        ManipulationEventCfg,
        ManipulationRewardCfg,
        ManipulationSceneCfg,
    )
    from srb.core.env.mobile import (
        AerialEnv,
        AerialEnvCfg,
        AerialEventCfg,
        AerialMobileRewardCfg,
        AerialSceneCfg,
        GroundEnv,
        GroundEnvCfg,
        GroundEventCfg,
        GroundMobileRewardCfg,
        GroundSceneCfg,
        OrbitalEnv,
        OrbitalEnvCfg,
        OrbitalEventCfg,
        OrbitalMobileRewardCfg,
        OrbitalSceneCfg,
    )
    from srb.core.env.mobile_manipulation import (
        AerialManipulationEnv,
        AerialManipulationEnvCfg,
        AerialManipulationEventCfg,
        AerialManipulationRewardCfg,
        AerialManipulationSceneCfg,
        GroundManipulationEnv,
        GroundManipulationEnvCfg,
        GroundManipulationEventCfg,
        GroundManipulationRewardCfg,
        GroundManipulationSceneCfg,
        OrbitalManipulationEnv,
        OrbitalManipulationEnvCfg,
        OrbitalManipulationEventCfg,
        OrbitalManipulationRewardCfg,
        OrbitalManipulationSceneCfg,
    )
    from srb.core.env.privileged import (
        assert_privileged_absent_from_onnx,
        onnx_export_input_whitelist,
    )
    from srb.core.env.srb_env import SrbEnv
