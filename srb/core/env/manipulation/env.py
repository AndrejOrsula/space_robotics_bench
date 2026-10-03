"""Base environment and configuration for fixed-base manipulation tasks."""

from dataclasses import MISSING

from srb.core._physx_fixups import (
    wrap_spawn_to_disable_joints,
    wrap_spawn_to_fix_end_effector_mass,
)
from srb.core.action.term.manipulation._frame_jacobian import (
    end_effector_frame_offset,
)
from srb.core.asset import (
    Articulation,
    ArticulationCfg,
    AssetBaseCfg,
    AssetVariant,
    Manipulator,
    MobileRobot,
    Pedestal,
    RigidObject,
    RigidObjectCfg,
    Subterrane,
    Terrain,
)
from srb.core.default_assets import get_default_asset_factories
from srb.core.env import BaseEventCfg, BaseSceneCfg, DirectEnv, DirectEnvCfg, ViewerCfg
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import reset_joints_by_offset
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.scene_introspection import (
    find_scene_articulation,
    find_scene_rigid_object,
    find_scene_sensor,
)
from srb.core.sensor import (
    ContactSensor,
    ContactSensorCfg,
    FrameTransformer,
    FrameTransformerCfg,
)
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms_tuple,
    radians,
    subtract_frame_transforms_tuple,
)


@configclass
class ManipulationSceneCfg(BaseSceneCfg):
    env_spacing: float = 4.0

    ## Assets
    pedestal: AssetBaseCfg | None = None

    ## Sensors
    contacts_end_effector: ContactSensorCfg | None = ContactSensorCfg(
        prim_path=MISSING,  # type: ignore[arg-type]
    )


@configclass
class ManipulationEventCfg(BaseEventCfg):
    randomize_robot_joints: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_joints_by_offset,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (-radians(5.0), radians(5.0)),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class ManipulationEnvCfg(DirectEnvCfg):
    ## Assets
    robot: Manipulator | AssetVariant = (
        get_default_asset_factories().manipulation_robot()
    )
    _robot: Manipulator = MISSING  # type: ignore[assignment]
    pedestal: Pedestal | MobileRobot | None = get_default_asset_factories().pedestal()
    scenery: Terrain | Subterrane | AssetVariant | None = AssetVariant.PROCEDURAL

    ## Scene
    scene: ManipulationSceneCfg = ManipulationSceneCfg()

    ## Events
    events: ManipulationEventCfg = ManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    agent_rate: float = 1.0 / 50.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(1.85, 0.0, 1.85), lookat=(0.125, 0.0, 0.25), origin_type="env"
    )

    def __post_init__(self):
        self._strip_pedestal_sensor_cfgs()

        super().__post_init__()

        self._apply_robot_spawner_fixups()
        self._setup_pedestal_geometry()
        self._setup_end_effector_sensors()

    def _strip_pedestal_sensor_cfgs(self) -> None:
        """Drop pedestal sensors before validation; reused mobile assets may have
        incomplete sensor configs.
        """
        if self.pedestal is not None and hasattr(self.pedestal, "sensor_cfgs"):
            self.pedestal.sensor_cfgs = {}

    def _apply_robot_spawner_fixups(self) -> None:
        """Add default mass and inertia to massless wrist frames before PhysX
        validation.
        """
        robot_scene_cfg = getattr(self.scene, "robot", None)
        if robot_scene_cfg is not None:
            wrap_spawn_to_fix_end_effector_mass(robot_scene_cfg)

    def _setup_pedestal_geometry(self) -> None:
        """Align the pedestal mount, rewrite its prim path, and disable its joints."""
        if self.pedestal is None:
            return
        # Lift only default-height robots; tasks with custom heights own placement.
        mount_z = self.pedestal.frame_manipulator_mount.offset.pos[2]
        robot_init_pos = self._robot.asset_cfg.init_state.pos or (0.0, 0.0, 0.0)
        if robot_init_pos[2] == 0.0 and mount_z != 0.0:
            lifted_pos = (robot_init_pos[0], robot_init_pos[1], mount_z)
            self._robot.asset_cfg.init_state.pos = lifted_pos
            # The scene spawns a deep copy, so mirror the lift onto it.
            scene_robot = getattr(self.scene, "robot", None)
            if scene_robot is not None and hasattr(scene_robot, "init_state"):
                scene_robot.init_state.pos = lifted_pos

        # InteractiveScene replicates this base config across environments.
        self.scene.pedestal = self.pedestal.as_asset_base_cfg()
        inverse_mount_pos, inverse_mount_rot = subtract_frame_transforms_tuple(
            self.pedestal.frame_manipulator_mount.offset.pos,
            self.pedestal.frame_manipulator_mount.offset.rot,
        )
        self.scene.pedestal.init_state.pos, self.scene.pedestal.init_state.rot = (
            combine_frame_transforms_tuple(
                self._robot.asset_cfg.init_state.pos,
                self._robot.asset_cfg.init_state.rot,
                inverse_mount_pos,
                inverse_mount_rot,
            )
        )
        self.scene.pedestal.prim_path = (
            "/World/pedestal" if self.stack else "{ENV_REGEX_NS}/pedestal"
        )
        wrap_spawn_to_disable_joints(self.scene.pedestal)

    def _setup_end_effector_sensors(self) -> None:
        """Set the end-effector frame to the TCP and configure contacts when its body
        supports them.
        """
        tf_end_effector = getattr(self.scene, "tf_end_effector", None)
        if (
            isinstance(tf_end_effector, FrameTransformerCfg)
            and len(tf_end_effector.target_frames) > 0
        ):
            ee_frame = end_effector_frame_offset(
                self._robot, combine=combine_frame_transforms_tuple
            )
            if ee_frame is not None:
                (
                    tf_end_effector.target_frames[0].offset.pos,
                    tf_end_effector.target_frames[0].offset.rot,
                ) = ee_frame

        # Only rigid objects and articulations provide a contact-reporting body.
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            end_effector = self._robot.end_effector
            if end_effector is not None and isinstance(
                end_effector.asset_cfg, (RigidObjectCfg, ArticulationCfg)
            ):
                self.scene.contacts_end_effector.prim_path = (
                    f"{end_effector.asset_cfg.prim_path}/.*"
                )
            else:
                self.scene.contacts_end_effector = None


class ManipulationEnv(DirectEnv):
    cfg: ManipulationEnvCfg

    def __init__(self, cfg: ManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._tf_end_effector: FrameTransformer | None = find_scene_sensor(
            self.scene, "tf_end_effector"
        )
        self._contacts_robot: ContactSensor | None = find_scene_sensor(
            self.scene, "contacts_robot"
        )
        end_effector = find_scene_articulation(self.scene, "end_effector")
        if end_effector is None:
            end_effector = find_scene_rigid_object(self.scene, "end_effector")
        self._end_effector: Articulation | RigidObject | None = end_effector
        self._contacts_end_effector: ContactSensor | None = find_scene_sensor(
            self.scene, "contacts_end_effector"
        )
