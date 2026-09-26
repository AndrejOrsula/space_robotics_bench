from functools import cache
from importlib.util import find_spec
from os import environ
from pathlib import Path
from typing import Literal


# Cache successful initialization so a failed attempt can be retried after the
# Kit runtime or ROS Python modules become available.
def enable_ros2_bridge(
    distro: str = "jazzy",
    rmw_implementation: Literal["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"]
    | str = "rmw_fastrtps_cpp",
) -> bool:
    cache_key = (distro, rmw_implementation)
    if cache_key in _ENABLED_ROS2_BRIDGES:
        return True
    from srb.utils import logging

    ## Skip if ROS 2 is already sourced
    if find_spec("rclpy"):
        logging.debug(
            'ROS 2 is already sourced in the current environment, so "ros2_bridge" does not need to be enabled'
        )
        _ENABLED_ROS2_BRIDGES.add(cache_key)
        return True

    ## Update environment
    _append_ld_library_path(_get_ros2_bridge_lib_path(distro))
    if not environ.get("RMW_IMPLEMENTATION"):
        environ["RMW_IMPLEMENTATION"] = rmw_implementation

    ## Enable extension
    # Isaac Sim 6.1 has no ``isaacsim.core.utils.extensions``; Isaac Lab's
    # helper raises instead of returning ``False``. The call must not live
    # inside an ``assert``: under ``python -O`` the whole statement is stripped
    # and the bridge is never enabled.
    from srb.utils.isaacsim import enable_isaacsim_extension

    try:
        enable_isaacsim_extension("isaacsim.ros2.bridge")
    except RuntimeError as exc:
        logging.error(
            'Failed to enable the "isaacsim.ros2.bridge" extension; ROS 2 interfaces are unavailable: %s',
            exc,
        )
        return False

    ## Check if ROS 2 is now available
    if not find_spec("rclpy"):
        logging.error(
            'ROS 2 Python client library "rclpy" is still not available after trying to enable the "ros2_bridge" extension'
        )
        return False

    _ENABLED_ROS2_BRIDGES.add(cache_key)
    return True


_ENABLED_ROS2_BRIDGES: set[tuple[str, str]] = set()


@cache
def _get_ros2_bridge_lib_path(distro: str) -> Path:
    from srb.utils.isaacsim import get_isaacsim_path

    ros2_lib_path = (
        Path(get_isaacsim_path())
        .joinpath("exts")
        .joinpath("isaacsim.ros2.bridge")
        .joinpath(distro)
        .joinpath("lib")
    )
    if not ros2_lib_path.is_dir():
        raise FileNotFoundError(
            f'ROS 2 bridge libraries for distro "{distro}" are missing at {ros2_lib_path}'
        )
    return ros2_lib_path


@cache
def _append_ld_library_path(lib_path: Path):
    entries = [
        entry for entry in environ.get("LD_LIBRARY_PATH", "").split(":") if entry
    ]
    # Compare whole entries: a substring test matches ``/x/lib`` inside ``/x/lib64``.
    if lib_path.as_posix() not in entries:
        entries.append(lib_path.as_posix())
    environ["LD_LIBRARY_PATH"] = ":".join(entries)
