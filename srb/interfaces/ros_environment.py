"""Validate required ROS 2 process settings without supplying defaults."""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass

ROS_DISTRO = "jazzy"
SUPPORTED_RMW_IMPLEMENTATIONS = frozenset({"rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"})
ROS_DOMAIN_ID_MIN = 0
ROS_DOMAIN_ID_MAX = 232


class RosEnvironmentError(RuntimeError):
    """Raised when a ROS process environment is absent or unsupported."""


@dataclass(frozen=True, slots=True)
class RosEnvironment:
    """Validated values required for the SRB ROS 2 graph."""

    distro: str
    rmw_implementation: str
    domain_id: int
    ament_prefix_path: tuple[str, ...]
    ld_library_path: tuple[str, ...]


def require_ros_environment(
    environ: Mapping[str, str] | None = None,
    *,
    require_prefixes: bool = False,
) -> RosEnvironment:
    """Validate ROS settings; optionally require sourced overlay paths."""

    values = os.environ if environ is None else environ
    distro = _required(values, "ROS_DISTRO")
    if distro != ROS_DISTRO:
        raise RosEnvironmentError(
            f"ROS_DISTRO={distro!r} is unsupported; SRB requires ROS_DISTRO={ROS_DISTRO!r}"
        )

    rmw_implementation = _required(values, "RMW_IMPLEMENTATION")
    if rmw_implementation not in SUPPORTED_RMW_IMPLEMENTATIONS:
        supported = ", ".join(sorted(SUPPORTED_RMW_IMPLEMENTATIONS))
        raise RosEnvironmentError(
            "RMW_IMPLEMENTATION must be one of "
            f"{supported}; received {rmw_implementation!r}"
        )

    domain_text = _required(values, "ROS_DOMAIN_ID")
    if not domain_text.isdecimal():
        raise RosEnvironmentError(
            f"ROS_DOMAIN_ID must be a decimal integer in "
            f"[{ROS_DOMAIN_ID_MIN}, {ROS_DOMAIN_ID_MAX}]; received {domain_text!r}"
        )
    domain_id = int(domain_text, 10)
    if not ROS_DOMAIN_ID_MIN <= domain_id <= ROS_DOMAIN_ID_MAX:
        raise RosEnvironmentError(
            f"ROS_DOMAIN_ID must be in [{ROS_DOMAIN_ID_MIN}, {ROS_DOMAIN_ID_MAX}]; "
            f"received {domain_id}"
        )

    ament_prefix_path = _path_values(values, "AMENT_PREFIX_PATH")
    ld_library_path = _path_values(values, "LD_LIBRARY_PATH")
    if require_prefixes:
        if not ament_prefix_path:
            raise RosEnvironmentError(
                "AMENT_PREFIX_PATH is empty; source the ROS Jazzy setup before starting SRB"
            )
        if not ld_library_path:
            raise RosEnvironmentError(
                "LD_LIBRARY_PATH is empty; source the ROS Jazzy setup before starting SRB"
            )

    return RosEnvironment(
        distro=distro,
        rmw_implementation=rmw_implementation,
        domain_id=domain_id,
        ament_prefix_path=ament_prefix_path,
        ld_library_path=ld_library_path,
    )


def _required(environ: Mapping[str, str], key: str) -> str:
    raw_value = environ.get(key, "")
    value = raw_value.strip()
    if not value:
        raise RosEnvironmentError(
            f"{key} is unset; source ROS Jazzy and set the SRB ROS graph explicitly"
        )
    if value != raw_value:
        raise RosEnvironmentError(
            f"{key} contains surrounding whitespace; set its exact ROS value"
        )
    return value


def _path_values(environ: Mapping[str, str], key: str) -> tuple[str, ...]:
    return tuple(part for part in environ.get(key, "").split(os.pathsep) if part)


__all__ = [
    "ROS_DISTRO",
    "ROS_DOMAIN_ID_MAX",
    "ROS_DOMAIN_ID_MIN",
    "SUPPORTED_RMW_IMPLEMENTATIONS",
    "RosEnvironment",
    "RosEnvironmentError",
    "require_ros_environment",
]
