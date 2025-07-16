from srb.core.env.common.extension.visual import VisualExtCfg
from srb.core.sensor import CameraCfg, PinholeCameraCfg
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat  # noqa: F401

from .env import GroundEnvCfg


@configclass
class GroundEnvVisualExtCfg(VisualExtCfg):
    def wrap(self, env_cfg: GroundEnvCfg):
        self.cameras_cfg = {
            # "cam_scene": CameraCfg(
            #     prim_path=f"{env_cfg._robot.asset_cfg.prim_path}{('/' + env_cfg._robot.frame_base.prim_relpath) if env_cfg._robot.frame_base.prim_relpath else ''}/camera_scene",
            #     offset=CameraCfg.OffsetCfg(
            #         convention="world",
            #         pos=(0.0, 7.5, 5.0),
            #         rot=rpy_to_quat(0.0, 30.0, -90.0),
            #     ),
            #     spawn=PinholeCameraCfg(
            #         clipping_range=(0.01, 20.0 + 0.01),
            #     ),
            # ),
            "cam_front": CameraCfg(
                prim_path=f"{env_cfg._robot.asset_cfg.prim_path}/{env_cfg._robot.frame_front_camera.prim_relpath}",
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=env_cfg._robot.frame_front_camera.offset.pos,
                    rot=env_cfg._robot.frame_front_camera.offset.rot,
                ),
                spawn=PinholeCameraCfg(
                    focal_length=5.0,
                    horizontal_aperture=12.0,
                    clipping_range=(0.05, 25.0 + 0.05),
                ),
            ),
        }

        super().wrap(env_cfg)
