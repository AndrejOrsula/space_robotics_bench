import space_robotics_bench.core.assets as asset_utils
import space_robotics_bench.core.envs as env_utils

from .anymal import anymal_b_cfg, anymal_c_cfg, anymal_d_cfg  # noqa: F401
from .canadarm3_large import canadarm3_large_cfg  # noqa: F401
from .cubesat import cubesat_cfg
from .franka import franka_cfg
from .ingenuity import ingenuity_cfg
from .perseverance import perseverance_cfg

# TODO: Add a configurable selector of robots


def rover_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    **kwargs,
) -> asset_utils.MobileRobotCfg:
    return perseverance_cfg(prim_path=prim_path, **kwargs)


def manipulator_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    **kwargs,
) -> asset_utils.ManipulatorCfg:
    return franka_cfg(prim_path=prim_path, **kwargs)


def legged_robot_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    **kwargs,
) -> asset_utils.LeggedRobotCfg:
    return anymal_c_cfg(prim_path=prim_path, **kwargs)


def aerial_robot_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    **kwargs,
) -> asset_utils.AerialRobotCfg:
    return ingenuity_cfg(prim_path=prim_path, **kwargs)


def spacecraft_robot_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    **kwargs,
) -> asset_utils.SpacecraftCfg:
    return cubesat_cfg(prim_path=prim_path, **kwargs)
