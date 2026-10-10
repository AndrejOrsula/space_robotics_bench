"""Deployment hardware profiles for wheeled ``cmd_vel`` tasks.

Covers the multi-robot CADRE tasks and the single-robot waypoint task. Every
profile keeps operator-configured interface instances (validating them
against the task contract) and replaces only bare hardware-id strings with
configured defaults.
"""

from __future__ import annotations

import math
from typing import Sequence

from srb.interfaces.sim_to_real.core.hardware import HardwareInterface
from srb.interfaces.sim_to_real.hardware.ros_cmd_vel import (
    RosCmdVelInterface,
    RosCmdVelInterfaceCfg,
)
from srb.interfaces.sim_to_real.hardware.ros_tf import (
    PositionRepresentation,
    RosTfInterface,
    RosTfInterfaceCfg,
    RotationRepresentation,
)

_CMD_BASE_KEYS = frozenset({"cmd_vel", "wheeled_drive", "velocity_command"})

# Waypoint observation frame contract: ``state/tf_pos2d_robot_to_target`` and
# ``state/tf_rot2dtrigyaw_robot_to_target`` are the target pose expressed in
# the robot frame (``lookup_transform("robot", "target")``). The ``target``
# frame matches ``srb.interfaces.sim_to_real.utils.ros_tf_traj``.
_WAYPOINT_ROBOT_FRAME = "robot"
_WAYPOINT_TARGET_FRAME = "target"
_WAYPOINT_TF_PAIR = (_WAYPOINT_ROBOT_FRAME, _WAYPOINT_TARGET_FRAME)


def configure_formation_following_hardware(
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: object | None = None,
) -> tuple[HardwareInterface | str, ...]:
    """Configure command interfaces for CADRE formation-following deployments."""
    payload_hardware = _configure_formation_tf_policy(
        hardware,
        action_layout=action_layout,
    )
    payload_hardware = _configure_multi_robot_cmd_vel(
        payload_hardware,
        action_layout=action_layout,
    )
    if _supports_required_observations(payload_hardware, action_layout=action_layout):
        return payload_hardware
    return (
        *payload_hardware,
        _formation_following_observation_interface(action_layout),
    )


def _configure_formation_tf_policy(
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: object | None,
) -> tuple[HardwareInterface | str, ...]:
    """Bind the formation observation frame contract to the TF health gate."""

    robot_names = _extract_robot_names_for_formation(action_layout)
    required_pairs = _formation_required_tf_pairs(robot_names)
    configured: list[HardwareInterface | str] = []
    found_tf = False
    for hw in hardware:
        if hw == "ros_tf":
            found_tf = True
            configured.append(
                RosTfInterface(
                    RosTfInterfaceCfg(
                        required_dynamic_frame_pairs=required_pairs,
                    )
                )
            )
            continue
        if isinstance(hw, RosTfInterface):
            found_tf = True
            configured_pairs = {
                tuple(pair) for pair in hw.cfg.required_dynamic_frame_pairs
            }
            missing = tuple(
                pair for pair in required_pairs if pair not in configured_pairs
            )
            if missing:
                raise ValueError(
                    "formation_following TF policy requires RosTfInterfaceCfg."
                    "required_dynamic_frame_pairs to include "
                    f"{missing!r}; configure the adapter from the known "
                    "formation observation frame contract"
                )
        configured.append(hw)

    if not found_tf:
        raise ValueError(
            "formation_following TF policy requires a ros_tf hardware interface "
            "with required_dynamic_frame_pairs; add ros_tf to --hardware or "
            "provide a configured RosTfInterface"
        )
    return tuple(configured)


def _formation_required_tf_pairs(
    robot_names: Sequence[str],
) -> tuple[tuple[str, str], ...]:
    """Return the exact target/source pairs consumed by the formation adapter."""

    target_frame = "formation_center"
    source_frame_for_velocity = "world"
    return (
        *((robot_name, target_frame) for robot_name in robot_names),
        (source_frame_for_velocity, target_frame),
        *((source_frame_for_velocity, robot_name) for robot_name in robot_names),
    )


def configure_waypoint_navigation_hardware(
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: object | None = None,
) -> tuple[HardwareInterface | str, ...]:
    """Configure the single-robot waypoint deployment.

    A bare ``ros_tf`` becomes a ``RosTfInterface`` that declares and
    health-gates the robot-to-target pair before start; a bare
    ``ros_cmd_vel`` becomes a filtered, clamped drive interface.
    """

    payload_hardware = _configure_waypoint_tf_policy(hardware)
    return _configure_multi_robot_cmd_vel(
        payload_hardware,
        action_layout=action_layout,
    )


def _configure_waypoint_tf_policy(
    hardware: Sequence[HardwareInterface | str],
) -> tuple[HardwareInterface | str, ...]:
    configured: list[HardwareInterface | str] = []
    for hw in hardware:
        if isinstance(hw, str) and hw == "ros_tf":
            configured.append(
                RosTfInterface(
                    RosTfInterfaceCfg(
                        position_repr=(PositionRepresentation.POS_2D,),
                        rotation_repr=(RotationRepresentation.ROT_2D_TRIG_YAW,),
                        required_dynamic_frame_pairs=(_WAYPOINT_TF_PAIR,),
                    )
                )
            )
            continue
        if isinstance(hw, RosTfInterface):
            configured_pairs = {
                tuple(pair) for pair in hw.cfg.required_dynamic_frame_pairs
            }
            if _WAYPOINT_TF_PAIR not in configured_pairs:
                raise ValueError(
                    "waypoint_navigation TF policy requires RosTfInterfaceCfg."
                    "required_dynamic_frame_pairs to include "
                    f"{_WAYPOINT_TF_PAIR!r}; the policy observes the target "
                    "pose in the robot frame"
                )
            missing_repr = [
                str(representation)
                for representation, configured_repr in (
                    (PositionRepresentation.POS_2D, hw.cfg.position_repr),
                    (RotationRepresentation.ROT_2D_TRIG_YAW, hw.cfg.rotation_repr),
                )
                if representation not in configured_repr
            ]
            if missing_repr:
                raise ValueError(
                    "waypoint_navigation TF policy requires RosTfInterfaceCfg "
                    f"representations {missing_repr!r}"
                )
        configured.append(hw)
    return tuple(configured)


def configure_beam_transport_hardware(
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: object | None = None,
) -> tuple[HardwareInterface | str, ...]:
    """Configure command interfaces for cooperative beam-transport deployments."""
    return _configure_multi_robot_cmd_vel(hardware, action_layout=action_layout)


def _configure_multi_robot_cmd_vel(
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: object | None,
) -> tuple[HardwareInterface | str, ...]:
    """Keep operator drive instances; replace only bare ``ros_cmd_vel`` ids.

    Operator-supplied ``RosCmdVelInterface`` instances carry the operator's
    bounds (filtering, clamps) and are never substituted. They must cover
    every commanded robot exactly once. Without operator instances, each
    commanded robot gets a generated filtered, clamped default.
    """

    robot_names = _extract_robot_command_names(action_layout)
    if not robot_names:
        return tuple(hardware)

    operator_cmd_vel = [hw for hw in hardware if isinstance(hw, RosCmdVelInterface)]
    if operator_cmd_vel:
        _validate_operator_cmd_vel_coverage(operator_cmd_vel, robot_names)
        return tuple(
            hw for hw in hardware if not (isinstance(hw, str) and hw == "ros_cmd_vel")
        )

    payload_hardware = tuple(hw for hw in hardware if not _is_cmd_vel_hardware(hw))
    return payload_hardware + tuple(
        _make_robot_cmd_vel_interface(robot_name, action_layout=action_layout)
        for robot_name in robot_names
    )


def _validate_operator_cmd_vel_coverage(
    operator_cmd_vel: Sequence[RosCmdVelInterface],
    robot_names: Sequence[str],
) -> None:
    covered: dict[str, str] = {}
    for hw in operator_cmd_vel:
        robot_name = hw._action_namespace(hw.cfg.topic)
        if robot_name not in robot_names:
            raise ValueError(
                f"RosCmdVelInterface {hw.name!r} (topic {hw.cfg.topic!r}) "
                f"commands robot {robot_name!r}, which is not in the action "
                f"layout {tuple(robot_names)!r}"
            )
        if robot_name in covered:
            raise ValueError(
                f"more than one RosCmdVelInterface commands robot "
                f"{robot_name!r}: {covered[robot_name]!r} and {hw.name!r}"
            )
        covered[robot_name] = hw.name
    missing = tuple(name for name in robot_names if name not in covered)
    if missing:
        raise ValueError(
            "operator-configured RosCmdVelInterface instances must cover every "
            f"commanded robot; missing {missing!r}. Configure one per robot "
            "(topic '<robot>/cmd_vel', or 'cmd_vel' for 'robot') or pass the "
            "bare 'ros_cmd_vel' id to use the generated defaults"
        )


def _formation_following_observation_interface(
    action_layout: object | None,
) -> HardwareInterface:
    from srb.interfaces.sim_to_real.hardware.ros_formation_center_velocity import (
        RosFormationFollowingObservationInterface,
        RosFormationFollowingObservationInterfaceCfg,
    )

    robot_names = _extract_robot_names_for_formation(action_layout)
    return RosFormationFollowingObservationInterface(
        RosFormationFollowingObservationInterfaceCfg(
            robot_names=robot_names,
            target_frame="formation_center",
            source_frame_for_velocity="world",
            formation_center_frame="formation_center",
            timeout_duration=0.05,
        )
    )


def _supports_required_observations(
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: object | None = None,
) -> bool:
    from srb.interfaces.sim_to_real.hardware.ros_formation_center_velocity import (
        RosFormationFollowingObservationInterface,
    )

    required_keys = (
        "state/tf_pos2d_robots_to_targets",
        "state/tf_rot2dtrigyaw_robots_to_targets",
        "state/formation_center_vel_2d",
        "state_dyn/robot_vel_xy",
    )
    expected_names = (
        _extract_robot_names_for_formation(action_layout)
        if action_layout is not None
        else None
    )

    for hw in hardware:
        if isinstance(hw, str):
            continue
        if _hardware_supports_keys(
            hw, required_keys, interface_type=RosFormationFollowingObservationInterface
        ):
            if expected_names is not None and tuple(hw.robot_names) != expected_names:
                raise ValueError(
                    f"formation observation robot_names {tuple(hw.robot_names)!r} "
                    f"do not match action layout {expected_names!r}"
                )
            return True
    return False


def _extract_robot_command_names(action_layout: object | None) -> tuple[str, ...]:
    source_spaces = getattr(action_layout, "source_spaces", None)
    if not callable(source_spaces):
        return ()

    robot_names: list[str] = []
    seen: set[str] = set()
    for action_key, _ in source_spaces():
        if not isinstance(action_key, str):
            continue
        if "/" in action_key:
            robot_name, base_key = action_key.rsplit("/", 1)
        else:
            robot_name, base_key = "robot", action_key

        if base_key not in _CMD_BASE_KEYS:
            continue
        if robot_name in seen:
            continue
        seen.add(robot_name)
        robot_names.append(robot_name)

    return tuple(robot_names)


def _extract_robot_names_for_formation(
    action_layout: object | None,
) -> tuple[str, ...]:
    if action_layout is None:
        return ("robot", "robot_1", "robot_2")
    return _extract_robot_command_names(action_layout)


def _hardware_supports_keys(
    hardware: HardwareInterface,
    keys: tuple[str, ...],
    *,
    interface_type: type[HardwareInterface] | None = None,
) -> bool:
    try:
        supported = hardware.supported_observation_spaces.spaces
    except Exception:
        return False
    if interface_type is not None and not isinstance(hardware, interface_type):
        return False
    return all(key in supported for key in keys)


def _robot_drive_scales(
    action_layout: object | None, robot_name: str
) -> tuple[float, float]:
    """Return simulator linear and angular scales for ``robot_name``.

    Per-component ``<key>_linear`` and ``<key>_angular`` values override
    ``<key>``. Missing scale metadata defaults to 1.
    """

    scale = getattr(action_layout, "scale", None) or {}
    source_spaces = getattr(action_layout, "source_spaces", None)
    linear = angular = 1.0
    if callable(source_spaces):
        for action_key, _ in source_spaces():
            if not isinstance(action_key, str):
                continue
            name, base_key = (
                action_key.rsplit("/", 1)
                if "/" in action_key
                else ("robot", action_key)
            )
            if name != robot_name or base_key not in _CMD_BASE_KEYS:
                continue
            linear = scale.get(f"{action_key}_linear", scale.get(action_key, 1.0))
            angular = scale.get(f"{action_key}_angular", scale.get(action_key, 1.0))
            break
    for label, value in (("linear", linear), ("angular", angular)):
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(
                f"robot {robot_name!r} {label} drive scale must be finite and "
                f"positive, got {value!r}"
            )
    return float(linear), float(angular)


def _make_robot_cmd_vel_interface(
    robot_name: str, *, action_layout: object | None = None
) -> RosCmdVelInterface:
    linear, angular = _robot_drive_scales(action_layout, robot_name)
    cfg = RosCmdVelInterfaceCfg(
        topic=f"{robot_name}/cmd_vel" if robot_name != "robot" else "cmd_vel",
        name=f"ros_cmd_vel_{robot_name}",
    )
    # Scale normalized actions by the simulator scale; clamp published
    # commands to the same bounds.
    cfg.filter_enabled = True
    cfg.v_lin_max = linear
    cfg.v_ang_max = angular
    cfg.max_linear_velocity = linear
    cfg.max_angular_velocity = angular
    return RosCmdVelInterface(cfg)


def _is_cmd_vel_hardware(hw: HardwareInterface | str) -> bool:
    if isinstance(hw, str):
        return hw == "ros_cmd_vel"
    return isinstance(hw, RosCmdVelInterface)


__all__ = [
    "configure_beam_transport_hardware",
    "configure_formation_following_hardware",
    "configure_waypoint_navigation_hardware",
]
