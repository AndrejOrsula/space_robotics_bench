from srb.core.env.common.extension.visual import VisualExtCfg
from srb.core.sensor import CameraCfg, PinholeCameraCfg
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat  # noqa: F401

from .env import OrbitalEnvCfg


@configclass
class OrbitalEnvVisualExtCfg(VisualExtCfg):
    def wrap(self, env_cfg: OrbitalEnvCfg):
        self.cameras_cfg = {
            # "cam_scene": CameraCfg(
            #     prim_path=f"{env_cfg._robot.asset_cfg.prim_path}{('/' + env_cfg._robot.frame_base.prim_relpath) if env_cfg._robot.frame_base.prim_relpath else ''}/camera_scene",
            #     offset=CameraCfg.OffsetCfg(
            #         convention="world",
            #         pos=(-2.5, 0.0, 2.5),
            #         rot=rpy_to_quat(0.0, 45.0, 0.0),
            #     ),
            #     spawn=PinholeCameraCfg(
            #         clipping_range=(0.25, 250.0 + 0.25),
            #     ),
            # ),
            "cam_onboard": CameraCfg(
                prim_path=f"{env_cfg._robot.asset_cfg.prim_path}/{env_cfg._robot.frame_onboard_camera.prim_relpath}",
                offset=CameraCfg.OffsetCfg(
                    convention="world",
                    pos=env_cfg._robot.frame_onboard_camera.offset.pos,
                    rot=env_cfg._robot.frame_onboard_camera.offset.rot,
                ),
                spawn=PinholeCameraCfg(
                    focal_length=4.0,
                    horizontal_aperture=12.0,
                    clipping_range=(1.0, 25.0 + 1.0),
                ),
                data_types=[
                    "rgb",
                    "depth",
                    "normals",
                    "semantic_segmentation",
                ],
            )
        }

        super().wrap(env_cfg)
