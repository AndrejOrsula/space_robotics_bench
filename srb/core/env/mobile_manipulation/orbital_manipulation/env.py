from dataclasses import MISSING

from srb import assets
from srb.core.action import (
    InverseKinematicsActionGroup,
    JointPositionRelativeActionGroup,
    OperationalSpaceControlActionGroup,
)
from srb.core.asset import Articulation, AssetVariant, OrbitalManipulator
from srb.core.env.manipulation.env import (
    ManipulationEnv,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.env.mobile.orbital.env import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
)
from srb.core.manager import SceneEntityCfg
from srb.core.sensor import ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import combine_frame_transforms_tuple


@configclass
class OrbitalManipulationSceneCfg(OrbitalSceneCfg, ManipulationSceneCfg):
    pass


@configclass
class OrbitalManipulationEventCfg(OrbitalEventCfg, ManipulationEventCfg):
    pass


@configclass
class OrbitalManipulationEnvCfg(OrbitalEnvCfg):
    ## Assets
    robot: OrbitalManipulator | AssetVariant = assets.GenericOrbitalManipulator(
        mobile_base=assets.Cubesat(), manipulator=assets.Franka()
    )
    _robot: OrbitalManipulator = MISSING  # type: ignore

    ## Scene
    scene: OrbitalManipulationSceneCfg = OrbitalManipulationSceneCfg()

    ## Events
    events: OrbitalManipulationEventCfg = OrbitalManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 250.0
    agent_rate: float = 1.0 / 50.0

    def __post_init__(self):
        ## Jacobian-based actions are currently not supported for free-floating manipulators
        if isinstance(self.robot, OrbitalManipulator) and isinstance(
            self.robot.manipulator.actions,
            (InverseKinematicsActionGroup, OperationalSpaceControlActionGroup),
        ):
            self.robot.manipulator.actions = JointPositionRelativeActionGroup()

        super().__post_init__()
        assert self.scene.manipulator is not None

        ## Adapted from ManipulationEnvCfg
        # Sensor: End-effector transform
        self.scene.tf_end_effector.prim_path = f"{self.scene.manipulator.prim_path}/{self._robot.manipulator.frame_base.prim_relpath}"
        self.scene.tf_end_effector.target_frames[
            0
        ].prim_path = f"{self.scene.manipulator.prim_path}/{self._robot.manipulator.frame_flange.prim_relpath}"
        if self._robot.manipulator.end_effector is not None:
            (
                self.scene.tf_end_effector.target_frames[0].offset.pos,
                self.scene.tf_end_effector.target_frames[0].offset.rot,
            ) = combine_frame_transforms_tuple(
                self._robot.manipulator.frame_flange.offset.pos,
                self._robot.manipulator.frame_flange.offset.rot,
                self._robot.manipulator.end_effector.frame_tool_centre_point.offset.pos,
                self._robot.manipulator.end_effector.frame_tool_centre_point.offset.rot,
            )
        else:
            (
                self.scene.tf_end_effector.target_frames[0].offset.pos,
                self.scene.tf_end_effector.target_frames[0].offset.rot,
            ) = (
                self._robot.manipulator.frame_flange.offset.pos,
                self._robot.manipulator.frame_flange.offset.rot,
            )

        # Sensor: Robot contacts
        self.scene.contacts_robot.prim_path = f"{self.scene.manipulator.prim_path}/.*"

        # Sensor: End-effector contacts
        self.scene.contacts_end_effector = (
            ContactSensorCfg(
                prim_path=f"{self._robot.manipulator.end_effector.asset_cfg.prim_path}/.*",
            )
            if self._robot.manipulator.end_effector is not None
            else None
        )

        # Event: Randomize robot joints
        self.events.randomize_robot_joints.params["asset_cfg"] = SceneEntityCfg(
            "manipulator"
        )


class OrbitalManipulationEnv(OrbitalEnv, ManipulationEnv):
    cfg: OrbitalManipulationEnvCfg

    def __init__(self, cfg: OrbitalManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._manipulator: Articulation = self.scene["manipulator"]
