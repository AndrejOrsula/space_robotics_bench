from dataclasses import MISSING

from srb import assets
from srb.core.asset import Articulation, AssetVariant, Manipulator, RigidObject
from srb.core.env import BaseEventCfg, BaseSceneCfg, DirectEnv, DirectEnvCfg, ViewerCfg
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.marker import FRAME_MARKER_SMALL_CFG
from srb.core.mdp import reset_joints_by_offset
from srb.core.sensor import (
    ContactSensor,
    ContactSensorCfg,
    FrameTransformer,
    FrameTransformerCfg,
)
from srb.utils.cfg import configclass
from srb.utils.math import combine_frame_transforms_tuple, deg_to_rad


@configclass
class ManipulationSceneCfg(BaseSceneCfg):
    env_spacing = 4.0

    ## Sensors
    tf_end_effector: FrameTransformerCfg = FrameTransformerCfg(
        prim_path=MISSING,  # type: ignore
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                name="robot_ee",
                prim_path=MISSING,  # type: ignore
            ),
        ],
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/robot_ee"),
    )
    contacts_robot: ContactSensorCfg = ContactSensorCfg(
        prim_path=MISSING,  # type: ignore
    )
    contacts_end_effector: ContactSensorCfg | None = None


@configclass
class ManipulationEventCfg(BaseEventCfg):
    randomize_robot_joints: EventTermCfg = EventTermCfg(
        func=reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (-deg_to_rad(5.0), deg_to_rad(5.0)),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class ManipulationEnvCfg(DirectEnvCfg):
    ## Assets
    robot: Manipulator | AssetVariant = assets.Franka()
    _robot: Manipulator = MISSING  # type: ignore

    ## Scene
    scene: ManipulationSceneCfg = ManipulationSceneCfg()

    ## Events
    events: ManipulationEventCfg = ManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    agent_rate: float = 1.0 / 50.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(1.5, 0.0, 1.75), lookat=(0.25, 0.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()

        # Sensor: End-effector transform
        self.scene.tf_end_effector.prim_path = (
            f"{self.scene.robot.prim_path}/{self._robot.frame_base.prim_relpath}"
        )
        self.scene.tf_end_effector.target_frames[
            0
        ].prim_path = (
            f"{self.scene.robot.prim_path}/{self._robot.frame_flange.prim_relpath}"
        )
        if self._robot.end_effector is not None:
            (
                self.scene.tf_end_effector.target_frames[0].offset.pos,
                self.scene.tf_end_effector.target_frames[0].offset.rot,
            ) = combine_frame_transforms_tuple(
                self._robot.frame_flange.offset.pos,
                self._robot.frame_flange.offset.rot,
                self._robot.end_effector.frame_tool_centre_point.offset.pos,
                self._robot.end_effector.frame_tool_centre_point.offset.rot,
            )
        else:
            (
                self.scene.tf_end_effector.target_frames[0].offset.pos,
                self.scene.tf_end_effector.target_frames[0].offset.rot,
            ) = (
                self._robot.frame_flange.offset.pos,
                self._robot.frame_flange.offset.rot,
            )

        # Sensor: Robot contacts
        self.scene.contacts_robot.prim_path = f"{self.scene.robot.prim_path}/.*"

        # Sensor: End-effector contacts
        self.scene.contacts_end_effector = (
            ContactSensorCfg(
                prim_path=f"{self._robot.end_effector.asset_cfg.prim_path}/.*",
            )
            if self._robot.end_effector is not None
            else None
        )


class ManipulationEnv(DirectEnv):
    cfg: ManipulationEnvCfg

    def __init__(self, cfg: ManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._tf_end_effector: FrameTransformer = self.scene["tf_end_effector"]
        self._contacts_robot: ContactSensor = self.scene["contacts_robot"]
        self._end_effector: Articulation | RigidObject | None = (
            self.scene.articulations.get("end_effector", None)
            or self.scene.rigid_objects.get("end_effector", None)
            or None
        )
        self._contacts_end_effector: ContactSensor | None = self.scene.sensors.get(  # type: ignore
            "contacts_end_effector", None
        )
