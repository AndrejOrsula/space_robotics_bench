"""Canonical Raph deploy profile for terrain-landscaping real environments.

The canonical landscaping deployment is the
Raph rover with a prismatic velocity shovel and one external heightmap
source. It has **no** onboard depth camera, no IMU observation, no blade
pitch (measured or synthetic), no previous-action feedback, and no Leo
profile.

Readiness is enforced by ``RealEnv.DEPLOYMENT_REQUIREMENTS`` (the four
``DeploymentCapability`` tags in
``srb.interfaces.sim_to_real.deploy_profiles.generation.LANDSCAPING_DEPLOYMENT_REQUIREMENTS``):
construction raises a typed ``DeploymentNotReadyError`` before ROS
acquisition or hardware start until components claiming those tags exist.

Operator dependencies requiring verification
---------------------------------------------
No Raph ROS shovel command is defined in this repository. Before any
component may claim ``raph.shovel_prismatic_velocity``, deployment must record:

- command topic name and message type;
- command unit and sign convention for the prismatic axis;
- saturation/limit behaviour on the controller side;
- command watchdog/timeout semantics;
- acknowledgement mechanism (if any);
- measured joint-state feedback topic/message evidence.

Equivalent contracts are required for ``raph.drive_velocity`` (measured Raph
    drive limits matching the simulator), the live external-mapper batch
    (``landscaping.external_heightmap_batch`` frame, coverage, and
freshness), and the observable task evaluator
(``landscaping.observable_task_evaluator``).
"""

from __future__ import annotations

from collections.abc import Sequence

from srb.interfaces.sim_to_real.core.hardware import HardwareInterface
from srb.terrain.landscaping_contract import LANDSCAPING_IO_CONTRACT

_DRIVE_TERM = LANDSCAPING_IO_CONTRACT.action.terms[0]
_DRIVE_LINEAR, _DRIVE_ANGULAR = _DRIVE_TERM.components
_DRIVE_FILTER = LANDSCAPING_IO_CONTRACT.action.drive_filter


def configure_terrain_landscaping_hardware(
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: object | None = None,
) -> tuple[HardwareInterface | str, ...]:
    """Configure canonical Raph landscaping hardware defaults.

    The drive interface is parameterized from the frozen IO contract
    (scale-then-saturation physical bounds plus the drive filter facts) so a
    deployment can never silently diverge from the fingerprinted simulator
    mapping; a supplied ``RosCmdVelInterface`` must already match it. All
    other entries pass through untouched: no shipped hardware
    component claims the landscaping deployment-capability tags, so canonical
    construction stops at the ``DeploymentNotReadyError`` preflight until the
    matching hardware contracts exist.
    """

    del action_layout  # The canonical action schema is fixed by the contract.
    return tuple(_configure_hardware_entry(hw) for hw in hardware)


def _configure_hardware_entry(
    hardware: HardwareInterface | str,
) -> HardwareInterface | str:
    if isinstance(hardware, str) and hardware != "ros_cmd_vel":
        return hardware

    from srb.interfaces.sim_to_real.hardware.ros_cmd_vel import (
        RosCmdVelInterface,
        RosCmdVelInterfaceCfg,
    )

    expected = {
        "filter_enabled": True,
        "v_lin_max": _DRIVE_LINEAR.physical_bounds()[1],
        "v_ang_max": _DRIVE_ANGULAR.physical_bounds()[1],
        "accel_lin_max": _DRIVE_FILTER.accel_lin_max_m_s2,
        "accel_ang_max": _DRIVE_FILTER.accel_ang_max_rad_s2,
        "filter_tau_s": _DRIVE_FILTER.filter_tau_s,
    }
    if isinstance(hardware, str):
        return RosCmdVelInterface(RosCmdVelInterfaceCfg(**expected))
    if isinstance(hardware, RosCmdVelInterface):
        mismatched = [
            f"{field}={getattr(hardware.cfg, field)!r} (contract {value!r})"
            for field, value in expected.items()
            if getattr(hardware.cfg, field) != value
        ]
        if mismatched:
            raise ValueError(
                f"landscaping RosCmdVelInterface {hardware.name!r} must match the "
                f"frozen drive contract; mismatched: {', '.join(mismatched)}"
            )
    return hardware


__all__ = [
    "configure_terrain_landscaping_hardware",
]
