from dataclasses import MISSING

from srb.core.asset import AssetVariant, Humanoid, MobileRobot
from srb.core.env import BaseEventCfg, BaseSceneCfg, DirectEnv, DirectEnvCfg
from srb.core.marker import RED_ARROW_X_MARKER_CFG
from srb.core.sensor import Imu, ImuCfg
from srb.utils.cfg import configclass


@configclass
class MobileSceneCfg(BaseSceneCfg):
    imu_robot: ImuCfg = ImuCfg(
        prim_path=MISSING,  # type: ignore
        gravity_bias=(0.0, 0.0, 0.0),
        visualizer_cfg=RED_ARROW_X_MARKER_CFG.replace(  # type: ignore
            prim_path="/Visuals/imu_robot/lin_acc"
        ),
    )


@configclass
class MobileEventCfg(BaseEventCfg):
    pass


@configclass
class MobileEnvCfg(DirectEnvCfg):
    ## Assets
    robot: MobileRobot | Humanoid | AssetVariant = MISSING  # type: ignore
    _robot: MobileRobot | Humanoid = MISSING  # type: ignore

    ## Scene
    scene: MobileSceneCfg = MobileSceneCfg()

    ## Events
    events: MobileEventCfg = MobileEventCfg()

    def __post_init__(self):
        super().__post_init__()

        # Sensor: Robot IMU
        if self._robot.frame_imu:
            self.scene.imu_robot.prim_path = (
                f"{self.scene.robot.prim_path}/{self._robot.frame_base.prim_relpath}"
            )
            self.scene.imu_robot.offset.pos = self._robot.frame_imu.offset.pos
            self.scene.imu_robot.offset.rot = self._robot.frame_imu.offset.rot
        else:
            self.scene.imu_robot.prim_path = (
                f"{self.scene.robot.prim_path}/{self._robot.frame_base.prim_relpath}"
                if self._robot.frame_base.prim_relpath
                else self.scene.robot.prim_path
            )
            self.scene.imu_robot.offset.pos = self._robot.frame_base.offset.pos
            self.scene.imu_robot.offset.rot = self._robot.frame_base.offset.rot


class MobileEnv(DirectEnv):
    cfg: MobileEnvCfg

    def __init__(self, cfg: MobileEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._imu_robot: Imu = self.scene["imu_robot"]
