"""Base environment for mobile robotics tasks.

Provides :class:`MobileEnv` and its configuration, which set up a mobile robot
(ground, aerial, or orbital), an IMU sensor, and the standard scene layout
shared by all mobile and mobile-manipulation tasks in SRB.
"""

from dataclasses import MISSING

import torch

from srb.core.asset import AssetVariant, Humanoid, MobileRobot
from srb.core.env import BaseEventCfg, BaseSceneCfg, DirectEnv, DirectEnvCfg
from srb.core.scene_introspection import find_scene_sensor
from srb.core.sensor import Imu
from srb.utils.cfg import configclass
from srb.utils.math import quat_apply_inverse


@configclass
class MobileSceneCfg(BaseSceneCfg):
    ## Sensors (populated from robot.sensor_cfgs via _add_sensors)
    pass


@configclass
class MobileEventCfg(BaseEventCfg):
    pass


@configclass
class MobileEnvCfg(DirectEnvCfg):
    ## Assets
    robot: MobileRobot | Humanoid | AssetVariant = MISSING  # type: ignore[assignment]
    _robot: MobileRobot | Humanoid = MISSING  # type: ignore[assignment]
    ## Scene
    scene: MobileSceneCfg = MobileSceneCfg()

    ## Events
    events: MobileEventCfg = MobileEventCfg()

    ## IMU sensor is configured via _add_sensors in BaseEnvCfg


class MobileEnv(DirectEnv):
    cfg: MobileEnvCfg

    def __init__(self, cfg: MobileEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        imu_robot = find_scene_sensor(self.scene, "imu_robot")
        self._imu_robot: Imu | None = imu_robot

        ## Planar air-bearing robots (e.g. `floating_platform`) anchor their
        ## articulation root to the world; track their base link instead.
        self._base_body_idx: int | None = None
        if getattr(self._robot, "is_fixed_base", False):
            base_body = cfg._robot.frame_base.prim_relpath.rsplit("/", 1)[-1]
            body_names = list(self._robot.body_names)
            if base_body in body_names:
                self._base_body_idx = body_names.index(base_body)

    def _robot_base_state(
        self,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return base position and orientation (world) and velocities (base frame)."""
        data = self._robot.data
        if self._base_body_idx is None:
            return (
                data.root_pos_w,
                data.root_quat_w,
                data.root_lin_vel_b,
                data.root_ang_vel_b,
            )
        quat = data.body_quat_w[:, self._base_body_idx]
        return (
            data.body_pos_w[:, self._base_body_idx],
            quat,
            quat_apply_inverse(quat, data.body_lin_vel_w[:, self._base_body_idx]),
            quat_apply_inverse(quat, data.body_ang_vel_w[:, self._base_body_idx]),
        )
