from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    AssetBaseCfg,
    AssetVariant,
    IntravehicularScenery,
    OrbitalManipulator,
    Robot,
)
from srb.core.domain import Domain
from srb.core.env import (
    OrbitalEnvCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
)
from srb.core.env.mobile import OrbitalMobileRewardCfg
from srb.core.scene_introspection import get_scene_articulation
from srb.core.sensor.event_camera import (
    build_event_camera_cfg,
    ros_rot_from_world_convention,
)
from srb.tasks.mobile._orbital.task import Task as _OrbitalTask
from srb.utils.cfg import configclass
from srb.utils.math import scale_transform

##############
### Config ###
##############


class RewardCfg(OrbitalMobileRewardCfg):
    """Reward terms for the base orbital mobile task."""

    pass


@configclass
class SceneCfg(OrbitalSceneCfg):
    pass


@configclass
class EventCfg(OrbitalEventCfg):
    # Zeroglab scenery sets the robot reset range to the facility floor.
    pass


@configclass
class TaskCfg(OrbitalEnvCfg):
    ## Scenario
    # Orbital free-fall sets gravity to zero and disables gravity
    # randomization, matching the floating platform and intravehicular scenery.
    domain: Domain = Domain.ORBIT

    ## Assets
    robot: OrbitalManipulator | Robot | AssetVariant = assets.GenericOrbitalManipulator(
        mobile_base=assets.FloatingPlatform(), manipulator=assets.LevionArms()
    )

    scenery: IntravehicularScenery | AssetVariant | None = assets.Zeroglab()

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    env_rate: float = 1.0 / 60.0
    agent_rate: float = 1.0 / 10.0
    episode_length_s: float = 100000.0
    is_finite_horizon: bool = False
    truncate_episodes: bool = False

    ## Sensors
    # The lab spans ~6.7 m corner to corner; 8 m keeps uint8 depth at ~3 cm per level.
    camera_clipping_range: tuple[float, float] | None = (0.01, 8.0)

    def __post_init__(self):
        super().__post_init__()

        import copy

        zeroglab_arms = assets.ZeroglabArms()

        # USD spawner.
        self.scene.zeroglab_arms_spawner = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/zeroglab_arms",
            spawn=zeroglab_arms.asset_cfg.spawn,
        )

        # Extract ceiling arm articulation.
        self.scene.arm_ceiling = copy.deepcopy(zeroglab_arms.asset_cfg)
        self.scene.arm_ceiling.prim_path = "{ENV_REGEX_NS}/zeroglab_arms/robot_ceiling"
        self.scene.arm_ceiling.spawn = None

        # Extract wall arm articulation.
        self.scene.arm_wall = copy.deepcopy(zeroglab_arms.asset_cfg)
        self.scene.arm_wall.prim_path = "{ENV_REGEX_NS}/zeroglab_arms/robot_wall"
        self.scene.arm_wall.spawn = None

        # Attach both manual controllers to the managed action group.
        self.actions.arm_ceiling_osc = zeroglab_arms.action_group.arm_ceiling
        self.actions.arm_wall_osc = zeroglab_arms.action_group.arm_wall

        # Fabric avoids a CPU PhysX bug: the fixed-base platform's anchor
        # world offset is written as a local offset, accumulating each render
        # until the welded manipulator diverges. Fabric keeps state finite.
        self.sim.use_fabric = True

        # Event camera co-located with the platform's onboard camera, so its
        # stream pairs with `image_onboard_rgb`.
        cam_onboard = self._robot.sensor_cfgs.get("cam_onboard")
        if cam_onboard is not None and cam_onboard.offset.convention == "world":
            # Robot sensor roots are rewritten to `robot`; keep the parent link.
            mount = cam_onboard.prim_path.split("/")[2:-1]
            self.env_sensor_cfgs["event_camera_onboard"] = build_event_camera_cfg(
                "/".join(("{ENV_REGEX_NS}/robot", *mount, "event_camera_onboard")),
                cam_onboard.offset.pos,
                rot_xyzw=ros_rot_from_world_convention(cam_onboard.offset.rot),
                preset_name=self.event_camera_preset,
                backend=self.event_camera_backend,
                sub_render_hz=self.event_camera_sub_render_hz,
                resolution=self.event_camera_resolution,
            )


############
### Task ###
############


class Task(_OrbitalTask):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._arms = {
            name: get_scene_articulation(self.scene, name)
            for name in ("manipulator", "arm_ceiling", "arm_wall")
        }

    def extract_step_return(self) -> StepReturn:
        step_return = super().extract_step_return()

        # Joint positions of the platform's arms and the facility arms,
        # normalized to their soft limits.
        proprio = step_return.observation[ObservationGroup.PROPRIO]
        for name, arm in self._arms.items():
            limits = arm.data.soft_joint_pos_limits
            proprio[f"joint_pos_{name}"] = scale_transform(
                arm.data.joint_pos, limits[..., 0], limits[..., 1]
            )

        return step_return
