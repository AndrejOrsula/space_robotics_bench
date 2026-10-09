import torch

from srb import assets
from srb.core.asset import AssetVariant, OrbitalRobot, Subterrane
from srb.core.domain import Domain
from srb.core.env import OrbitalEventCfg, ViewerCfg
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.sensor import CameraCfg, PinholeCameraCfg
from srb.core.sensor.event_camera import (
    build_event_camera_cfg,
    ros_rot_from_world_convention,
)
from srb.tasks.mobile._orbital.task import TaskCfg as _OrbitalTaskCfg
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat

##############
### Config ###
##############

# Front camera mount on the robot base: above the deck, looking along +X and
# pitched 20 deg down at the regolith (world convention: +X forward, +Z up).
_CAM_FRONT_POS = (0.3, 0.0, 0.5)
_CAM_FRONT_ROT_WORLD = rpy_to_quat(0.0, 20.0, 0.0)


@configclass
class EventCfg(OrbitalEventCfg):
    # Hover level above the regolith (up to ~0.4 m) and clear of the walls.
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-2.0, 2.0),
                "y": (-3.0, 3.0),
                "z": (0.5, 0.9),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {},
        },
    )


@configclass
class TaskCfg(_OrbitalTaskCfg):
    ## Scenario
    domain: Domain = Domain.MOON

    ## Assets
    robot: OrbitalRobot | AssetVariant = assets.FloatingPlatform()
    scenery: Subterrane | AssetVariant | None = assets.Lunalab()

    ## Scene
    # Share one world for this physical facility (env_spacing=0). SimForge's
    # multi-asset MoonSurface has no source prim to clone per environment.
    stack: bool = True

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 100000.0
    truncate_episodes: bool = False

    ## Sensors
    # The facility spans ~12 m; 20 m keeps uint8 depth at ~8 cm per level.
    camera_clipping_range: tuple[float, float] | None = (0.01, 20.0)

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(-4.0, -5.0, 1.5), lookat=(0.0, -1.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()

        # Mount on the resolved robot's base; robot sensors live under `robot`.
        base_path = f"{{ENV_REGEX_NS}}/robot/{self._robot.frame_base.prim_relpath}"
        # `env_sensor_cfgs` are validated even when VISUAL is inactive.
        width, height = self.camera_resolution or (64, 64)
        self.env_sensor_cfgs["cam_front"] = CameraCfg(
            prim_path=f"{base_path}/camera_front",
            offset=CameraCfg.OffsetCfg(
                pos=_CAM_FRONT_POS, rot=_CAM_FRONT_ROT_WORLD, convention="world"
            ),
            spawn=PinholeCameraCfg(),
            width=width,
            height=height,
        )
        self.env_sensor_cfgs["event_camera_front"] = build_event_camera_cfg(
            f"{base_path}/event_camera_front",
            _CAM_FRONT_POS,
            rot_xyzw=ros_rot_from_world_convention(_CAM_FRONT_ROT_WORLD),
            preset_name=self.event_camera_preset,
            backend=self.event_camera_backend,
            sub_render_hz=self.event_camera_sub_render_hz,
            resolution=self.event_camera_resolution,
        )
