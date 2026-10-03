from dataclasses import _MISSING_TYPE
from dataclasses import MISSING as MISSING_MANIPULATOR
from dataclasses import MISSING as MISSING_MOBILE_BASE

from srb.core.action import ActionGroup
from srb.core.asset import (
    AerialManipulator,
    AerialRobot,
    ArticulationCfg,
    Frame,
    GroundManipulator,
    GroundRobot,
    Manipulator,
    OrbitalManipulator,
    OrbitalRobot,
    RigidObjectCfg,
)


class GenericAerialManipulator(AerialManipulator, arbitrary_types_allowed=True):
    ## Model
    mobile_base: AerialRobot | None = None
    manipulator: Manipulator | _MISSING_TYPE = MISSING_MANIPULATOR

    ## Other fields are updated via `mobile_base`
    asset_cfg: RigidObjectCfg | ArticulationCfg | _MISSING_TYPE = MISSING_MOBILE_BASE
    action_modes: dict[str, ActionGroup] | _MISSING_TYPE = MISSING_MOBILE_BASE
    action_mode: str | None = None
    frame_base: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    frame_payload_mount: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    frame_manipulator_mount: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    sensor_cfgs: dict = {}


class GenericGroundManipulator(GroundManipulator, arbitrary_types_allowed=True):
    ## Model
    mobile_base: GroundRobot | None = None
    manipulator: Manipulator | _MISSING_TYPE = MISSING_MANIPULATOR

    ## Other fields are updated via `mobile_base`
    asset_cfg: RigidObjectCfg | ArticulationCfg | _MISSING_TYPE = MISSING_MOBILE_BASE
    action_modes: dict[str, ActionGroup] | _MISSING_TYPE = MISSING_MOBILE_BASE
    action_mode: str | None = None
    frame_base: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    frame_payload_mount: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    frame_manipulator_mount: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    sensor_cfgs: dict = {}


class GenericOrbitalManipulator(OrbitalManipulator, arbitrary_types_allowed=True):
    ## Model
    mobile_base: OrbitalRobot | None = None
    manipulator: Manipulator | _MISSING_TYPE = MISSING_MANIPULATOR

    ## Other fields are updated via `mobile_base`
    asset_cfg: RigidObjectCfg | ArticulationCfg | _MISSING_TYPE = MISSING_MOBILE_BASE
    action_modes: dict[str, ActionGroup] | _MISSING_TYPE = MISSING_MOBILE_BASE
    action_mode: str | None = None
    frame_base: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    frame_payload_mount: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    frame_manipulator_mount: Frame | _MISSING_TYPE = MISSING_MOBILE_BASE
    sensor_cfgs: dict = {}
