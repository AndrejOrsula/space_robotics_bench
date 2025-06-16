import bisect
import math
import types
from dataclasses import MISSING
from os import environ
from typing import Dict, Literal, get_type_hints

import torch
from simforge import BakeType

from srb import assets
from srb.core.action import (
    ActionGroup,
    ActionTermCfg,
    DifferentialInverseKinematicsActionCfg,
    OperationalSpaceControllerActionCfg,
)
from srb.core.action.term import DummyActionCfg
from srb.core.asset import (
    ActiveTool,
    ArticulationCfg,
    AssetBaseCfg,
    AssetVariant,
    CombinedMobileManipulator,
    Manipulator,
    MobileRobot,
    RigidObjectCfg,
    RigidObjectCollectionCfg,
    Robot,
    Scenery,
    Tool,
)
from srb.core.domain import Domain
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.marker import VisualizationMarkersCfg
from srb.core.mdp import follow_xform_orientation_linear_trajectory  # noqa F401
from srb.core.mdp import reset_scene_to_default  # noqa F401
from srb.core.mdp import reset_xform_orientation_uniform
from srb.core.sensor import SensorBaseCfg
from srb.core.sim import (
    DistantLightCfg,
    DomeLightCfg,
    ParticlesSpawnerCfg,
    PhysxCfg,
    PyramidParticlesSpawnerCfg,
    RenderCfg,
    RigidBodyMaterialCfg,
    SimforgeAssetCfg,
    SimulationCfg,
)
from srb.core.sim.robot_setup import RobotAssemblerCfg
from srb.core.visuals import VisualsCfg
from srb.utils import logging
from srb.utils.cfg import configclass
from srb.utils.math import combine_frame_transforms_tuple, rpy_to_quat
from srb.utils.path import (
    SRB_ASSETS_DIR_SRB_SKYDOME_HIGH_RES,
    SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES,
)

from .event_cfg import BaseEventCfg
from .scene_cfg import BaseSceneCfg


@configclass
class BaseEnvCfg:
    ## Scenario
    seed: int = 0
    domain: Domain = Domain.MOON
    lunar_orbit: bool = False

    ## Assets
    scenery: Scenery | AssetVariant | None = AssetVariant.PROCEDURAL
    _scenery: Scenery | None = MISSING  # type: ignore
    robot: Robot | AssetVariant = AssetVariant.DATASET
    _robot: Robot = MISSING  # type: ignore
    skydome: Literal["low_res", "high_res"] | bool | None = "low_res"

    ## Assemblies (dynamic joints)
    joint_assemblies: Dict[str, RobotAssemblerCfg] = {}

    ## Scene
    scene: BaseSceneCfg = BaseSceneCfg()
    stack: bool = False
    num_envs: int | None = None
    spacing: float | None = None

    ## Events
    events: BaseEventCfg = BaseEventCfg()

    ## Time
    env_rate: float = MISSING  # type: ignore
    agent_rate: float = MISSING  # type: ignore

    ## Simulation
    sim = SimulationCfg(
        dt=MISSING,  # type: ignore
        render_interval=MISSING,  # type: ignore
        gravity=MISSING,  # type: ignore
        device="cpu",  # Note: Changed to GPU in __main__.py because initializing with CPU improves compatibility
        physx=PhysxCfg(
            min_position_iteration_count=2,
            min_velocity_iteration_count=1,
            enable_ccd=True,
            enable_stabilization=False,
            bounce_threshold_velocity=0.0,
            friction_offset_threshold=0.01,
            friction_correlation_distance=0.005,
        ),
        physics_material=RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
        ),
        render=RenderCfg(
            enable_translucency=True,
            enable_reflections=True,
        ),
    )
    malloc_scale: float = 1.0

    ## Visuals
    visuals: VisualsCfg = VisualsCfg()

    ## Misc
    truncate_episodes: bool = True
    extras: bool = False
    debug_vis: bool = (
        environ.get("DEBUG_VIS") or environ.get("SRB_DEBUG_VIS", "false")
    ).lower() in ("true", "1")

    ## Particles
    # Note: This option is likely to be removed in the future
    scatter_particles: bool = False
    particles_size: float = 0.025
    particles_ratio: float = 0.001

    def __post_init__(self):
        ## Scene
        if self.num_envs is not None:
            self.scene.num_envs = self.num_envs
        if self.spacing is None:
            self.spacing = self.scene.env_spacing
        self.scene.env_spacing = 0.0 if self.stack else self.spacing

        ## Assets -> Scene
        self._add_sunlight()
        self._add_skydome()
        self._add_scenery()
        self._add_robot()
        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()

        ## Particles
        self._maybe_add_particles()
        self._maybe_disable_fabric_for_particles()

        ## Events
        self.events.update(self)

        ## Simulation
        self.decimation = math.floor(self.agent_rate / self.env_rate)
        self.sim.dt = self.env_rate
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -self.domain.gravity_magnitude)
        self._update_memory_allocation()

        ## Misc
        self._update_debug_vis()

    def _update_memory_allocation(self):
        _pow = math.floor(self.scene.num_envs**0.375) - 1

        self.sim.physx.gpu_max_rigid_contact_count = math.floor(
            self.malloc_scale * 2 ** min(13 + _pow, 31),
        )
        self.sim.physx.gpu_max_rigid_patch_count = math.floor(
            self.malloc_scale * 2 ** min(12 + _pow, 31),
        )
        self.sim.physx.gpu_found_lost_pairs_capacity = math.floor(
            self.malloc_scale * 2 ** min(12 + _pow, 31),
        )
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = math.floor(
            self.malloc_scale * 2 ** min(13 + _pow, 31),
        )
        self.sim.physx.gpu_total_aggregate_pairs_capacity = math.floor(
            self.malloc_scale * 2 ** min(12 + _pow, 31),
        )
        self.sim.physx.gpu_collision_stack_size = math.floor(
            self.malloc_scale * 2 ** min(20 + _pow, 31),
        )
        self.sim.physx.gpu_heap_capacity = math.floor(
            self.malloc_scale * 2 ** min(20 + _pow, 31),
        )
        self.sim.physx.gpu_temp_buffer_capacity = math.floor(
            self.malloc_scale * 2 ** min(10 + _pow, 31),
        )
        self.sim.physx.gpu_max_soft_body_contacts = math.floor(
            self.malloc_scale * 2 ** min(10 + _pow, 31),
        )
        self.sim.physx.gpu_max_particle_contacts = math.floor(
            self.malloc_scale * 2 ** min(10 + _pow, 31),
        )

        self.sim.physx.gpu_max_num_partitions = 1 << bisect.bisect_left(
            (3, 15, 127, 511, 1023), self.scene.num_envs
        )

    def _add_sunlight(self, *, prim_path: str = "/World/sunlight", **kwargs):
        if self.domain.light_intensity <= 0.0:
            return
        self.scene.sunlight = AssetBaseCfg(
            prim_path=prim_path,
            spawn=DistantLightCfg(
                intensity=self.domain.light_intensity,
                angle=self.domain.light_angular_diameter,
                color_temperature=self.domain.light_color_temperature,
                enable_color_temperature=True,
                **kwargs,
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                rot=rpy_to_quat(
                    45.0,
                    30.0,
                    0.0,
                ),
            ),
        )

    def _add_skydome(self, *, prim_path: str = "/World/skydome", **kwargs):
        if not self.skydome:
            self.scene.skydome = None
            self.events.randomize_skydome_orientation = None
            return
        elif isinstance(self.skydome, str):
            match self.skydome:
                case "low_res":
                    skydome_dir = SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES
                case "high_res":
                    skydome_dir = SRB_ASSETS_DIR_SRB_SKYDOME_HIGH_RES
                case _:
                    raise ValueError(f"Invalid skydome option: {self.skydome}")
        else:
            skydome_dir = SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES

        match self.domain:
            case Domain.EARTH:
                self.scene.skydome = AssetBaseCfg(
                    prim_path=prim_path,
                    spawn=DomeLightCfg(
                        intensity=0.25 * self.domain.light_intensity,
                        texture_file=skydome_dir.joinpath("cloudy_sky.exr").as_posix(),
                        **kwargs,
                    ),
                )
            case Domain.MOON:
                self.scene.skydome = AssetBaseCfg(
                    prim_path=prim_path,
                    spawn=DomeLightCfg(
                        intensity=0.25 * self.domain.light_intensity,
                        texture_file=skydome_dir.joinpath(
                            "stars.exr"
                            # "milky_way.exr
                        ).as_posix(),
                        **kwargs,
                    ),
                )
                self.events.randomize_skydome_orientation = EventTermCfg(
                    func=reset_xform_orientation_uniform,
                    mode="interval",
                    is_global_time=True,
                    interval_range_s=(10.0, 60.0),
                    params={
                        "asset_cfg": SceneEntityCfg("skydome"),
                        "orientation_distribution_params": {
                            "roll": (-torch.pi, torch.pi),
                            "pitch": (-torch.pi, torch.pi),
                            "yaw": (-torch.pi, torch.pi),
                        },
                    },
                )
            case Domain.MARS:
                self.scene.skydome = AssetBaseCfg(
                    prim_path=prim_path,
                    spawn=DomeLightCfg(
                        intensity=0.25 * self.domain.light_intensity,
                        texture_file=skydome_dir.joinpath("mars_sky.exr").as_posix(),
                        **kwargs,
                    ),
                )
                self.events.randomize_skydome_orientation = None
            case Domain.ORBIT:
                self.scene.skydome = AssetBaseCfg(
                    prim_path=prim_path,
                    spawn=DomeLightCfg(
                        intensity=0.25 * self.domain.light_intensity,
                        texture_file=skydome_dir.joinpath(
                            "low_lunar_orbit.jpg"
                            if self.lunar_orbit
                            else "low_earth_orbit.exr"
                        ).as_posix(),
                        **kwargs,
                    ),
                )
                self.events.randomize_skydome_orientation = EventTermCfg(
                    func=reset_xform_orientation_uniform,
                    mode="interval",
                    is_global_time=True,
                    interval_range_s=(10.0, 60.0),
                    params={
                        "asset_cfg": SceneEntityCfg("skydome"),
                        "orientation_distribution_params": {
                            "roll": (-torch.pi, torch.pi),
                            "pitch": (-torch.pi, torch.pi),
                            "yaw": (-torch.pi, torch.pi),
                        },
                    },
                )
            case _:
                self.scene.skydome = None
                self.events.randomize_skydome_orientation = None

    def _add_scenery(
        self,
        *,
        prim_path: str = "{ENV_REGEX_NS}/scenery",
        prim_path_stacked: str = "/World/scenery",
        **kwargs,
    ):
        if self.scenery is None:
            self._scenery = None
            return

        ## Select scenery from registry based on the selected variant
        scenery = self.scenery
        if isinstance(scenery, AssetVariant):
            type_hints = get_type_hints(self)["scenery"]

            ## Attributes
            assert self.spacing is not None
            size = (self.spacing, self.spacing)
            scale = (
                self.spacing,
                self.spacing,
                0.1 * self.spacing,
            )
            _dyn_res = max(
                1,  # Min multiplier
                min(
                    10,  # Max multiplier
                    2
                    * round(
                        min(
                            8.0,
                            math.pow(
                                self.spacing,
                                0.4,  # Exponential scaling
                            ),
                        )
                        / 2
                    ),
                ),
            )
            texture_resolution = {
                BakeType.ALBEDO: _dyn_res * 1024,
                BakeType.EMISSION: _dyn_res * 128,
                BakeType.METALLIC: _dyn_res * 256,
                BakeType.NORMAL: _dyn_res * 1024,
                BakeType.ROUGHNESS: _dyn_res * 512,
            }
            density = 0.0075 * (_dyn_res**2)
            flat_area_size = 0.8 * (_dyn_res**1.2)

            if isinstance(type_hints, types.UnionType):
                for typ in type_hints.__args__:
                    if issubclass(typ, AssetVariant):
                        continue
                    assert issubclass(typ, Scenery)
                    for registered_scenery in typ.scenery_registry():
                        if issubclass(registered_scenery, typ):
                            try:
                                _scenery = registered_scenery(
                                    size=size,
                                    scale=scale,
                                    texture_resolution=texture_resolution,
                                    density=density,
                                    flat_area_size=flat_area_size,
                                    **kwargs,
                                )
                            except Exception as e:
                                logging.warning(
                                    f'Failed to instantiate "{registered_scenery.__name__}": {e}'
                                )
                            if _scenery.is_variant(scenery):
                                if (
                                    len(_scenery.DOMAINS) == 0
                                    or self.domain in _scenery.DOMAINS
                                ):
                                    scenery = _scenery
                                    logging.info(
                                        f'Selected scenery "{scenery.__class__.__name__}" of variant "{scenery.asset_variant}" for domain "{self.domain}"'
                                    )
                                    break
                                else:
                                    logging.debug(
                                        f'Registered scenery "{registered_scenery.__name__}" is a subclass of suitable type "{typ.__name__}" and "{_scenery.asset_variant}" variant but it does not support the "{self.domain}" domain'
                                    )
                            else:
                                logging.debug(
                                    f'Registered scenery "{registered_scenery.__name__}" is a subclass of suitable type "{typ.__name__}" but its "{_scenery.asset_variant}" variant does not match the requested "{scenery}" variant'
                                )
                    else:
                        continue
                    break
            else:
                logging.error(
                    f"Unsupported type hints for scenery specified via {AssetVariant}: {type_hints} ({type(type_hints)})"
                )
        assert isinstance(scenery, Scenery), (
            f"Failed to instantiate scenery from {repr(scenery)}"
        )

        # Update prim path
        scenery.asset_cfg.prim_path = (
            prim_path_stacked
            if self.stack or isinstance(self.scenery, assets.GroundPlane)
            else prim_path
        )

        # Add to the scene
        self.scene.scenery = scenery.asset_cfg

        # Store the updated config in an internal state
        self._scenery = scenery

    def _add_robot(
        self,
        *,
        prim_path: str = "{ENV_REGEX_NS}/robot",
        prim_path_manipulator: str = "{ENV_REGEX_NS}/manipulator",
        prim_path_payload: str = "{ENV_REGEX_NS}/payload",
        prim_path_end_effector: str = "{ENV_REGEX_NS}/end_effector",
        **kwargs,
    ):
        robot_name = prim_path.rstrip("/").rsplit("/", 1)[-1]
        manipulator_name = prim_path_manipulator.rstrip("/").rsplit("/", 1)[-1]
        payload_name = prim_path_payload.rstrip("/").rsplit("/", 1)[-1]
        end_effector_name = prim_path_end_effector.rstrip("/").rsplit("/", 1)[-1]

        ## Select robot from registry based on the selected variant
        robot = self.robot
        if isinstance(robot, AssetVariant):
            type_hints = get_type_hints(self)["robot"]
            if isinstance(type_hints, types.UnionType):
                for typ in type_hints.__args__:
                    if issubclass(typ, AssetVariant):
                        continue
                    assert issubclass(typ, Robot)
                    for registered_robot in typ.robot_registry():
                        if issubclass(registered_robot, typ):
                            try:
                                _robot = registered_robot(**kwargs)
                            except Exception as e:
                                logging.warning(
                                    f'Failed to instantiate "{registered_robot.__name__}": {e}'
                                )
                            if _robot.is_variant(robot):
                                if (
                                    len(_robot.DOMAINS) == 0
                                    or self.domain in _robot.DOMAINS
                                ):
                                    robot = _robot
                                    logging.info(
                                        f'Selected robot "{robot.__class__.__name__}" of variant "{robot.asset_variant}" for domain "{self.domain}"'
                                    )
                                    break
                                else:
                                    logging.debug(
                                        f'Registered robot "{registered_robot.__name__}" is a subclass of suitable type "{typ.__name__}" and "{_robot.asset_variant}" variant but it does not support the "{self.domain}" domain'
                                    )
                            else:
                                logging.debug(
                                    f'Registered robot "{registered_robot.__name__}" is a subclass of suitable type "{typ.__name__}" but its "{_robot.asset_variant}" variant does not match the requested "{robot}" variant'
                                )
                    else:
                        continue
                    break
            else:
                logging.error(
                    f"Unsupported type hints for robot specified via {AssetVariant}: {type_hints} ({type(type_hints)})"
                )
        assert isinstance(robot, Robot), (
            f"Failed to instantiate robot from {repr(robot)}"
        )

        ## Reuse existing action group or create a new one
        # if hasattr(self, "actions") and isinstance(self.actions, ActionGroup):
        #     map_cmd_to_action_fns = [self.actions.map_cmd_to_action]
        # else:
        #     self.actions = ActionGroup()
        #     map_cmd_to_action_fns = []
        self.actions = ActionGroup()
        map_cmd_to_action_fns = []

        ## Common: Robot
        robot.asset_cfg.prim_path = prim_path
        setattr(self.scene, robot_name, robot.asset_cfg)
        # Actions
        for action_key, action_term in robot.actions.__dict__.items():
            if not isinstance(action_term, ActionTermCfg):
                continue
            # Ensure the actions terms are applied to the correct asset
            action_term.asset_name = robot_name
            # Add action terms to the action group
            setattr(self.actions, f"{robot_name}__{action_key}", action_term)
        # Add the command mapping function to the action group
        map_cmd_to_action_fns.append(robot.actions.map_cmd_to_action)

        ## Extra: Mobile robot - Payload
        if isinstance(robot, MobileRobot):
            if robot.payload is not None:
                if isinstance(
                    robot.payload.asset_cfg, (RigidObjectCfg, ArticulationCfg)
                ):
                    robot.payload.asset_cfg.prim_path = prim_path_payload
                    (
                        robot.payload.asset_cfg.init_state.pos,
                        robot.payload.asset_cfg.init_state.rot,
                    ) = combine_frame_transforms_tuple(
                        robot.asset_cfg.init_state.pos,
                        robot.asset_cfg.init_state.rot,
                        robot.frame_payload_mount.offset.pos,
                        robot.frame_payload_mount.offset.rot,
                    )
                    self.joint_assemblies[payload_name] = RobotAssemblerCfg(
                        base_path=robot.asset_cfg.prim_path,
                        attach_path=robot.payload.asset_cfg.prim_path,
                        base_mount_frame=f"/{robot.frame_payload_mount.prim_relpath}"
                        if robot.frame_payload_mount.prim_relpath
                        else "",
                        attach_mount_frame="",
                        fixed_joint_offset=robot.frame_payload_mount.offset.pos,
                        fixed_joint_orient=robot.frame_payload_mount.offset.rot,
                        mask_all_collisions=True,
                    )
                else:
                    self.joint_assemblies.pop(payload_name, None)
                    robot.payload.asset_cfg.prim_path = f"{robot.asset_cfg.prim_path}/{robot.frame_base.prim_relpath}/{payload_name}"
                    (
                        robot.payload.asset_cfg.init_state.pos,
                        robot.payload.asset_cfg.init_state.rot,
                    ) = (
                        robot.frame_payload_mount.offset.pos,
                        robot.frame_payload_mount.offset.rot,
                    )
                setattr(self.scene, payload_name, robot.payload.asset_cfg)
            else:
                setattr(self.scene, payload_name, None)
                self.joint_assemblies.pop(payload_name, None)

        ## Extra: Mobile manipulator (combined) - Manipulator
        if isinstance(robot, CombinedMobileManipulator):
            ## Manipulator
            robot.manipulator.asset_cfg.prim_path = prim_path_manipulator
            setattr(self.scene, manipulator_name, robot.manipulator.asset_cfg)
            manipulator_needs_jacobian = bool(
                next(
                    (
                        action_term
                        for action_term in robot.manipulator.actions.__dict__.values()
                        if isinstance(
                            action_term,
                            (
                                DifferentialInverseKinematicsActionCfg,
                                OperationalSpaceControllerActionCfg,
                            ),
                        )
                    ),
                    None,
                )
            )
            (
                robot.manipulator.asset_cfg.init_state.pos,
                robot.manipulator.asset_cfg.init_state.rot,
            ) = combine_frame_transforms_tuple(
                robot.asset_cfg.init_state.pos,
                robot.asset_cfg.init_state.rot,
                robot.frame_manipulator_mount.offset.pos,
                robot.frame_manipulator_mount.offset.rot,
            )
            self.joint_assemblies[manipulator_name] = RobotAssemblerCfg(
                base_path=prim_path,
                attach_path=prim_path_manipulator,
                base_mount_frame=f"/{robot.frame_base.prim_relpath}"
                if robot.frame_base.prim_relpath
                else "",
                attach_mount_frame=f"/{robot.manipulator.frame_base.prim_relpath}"
                if robot.manipulator.frame_base.prim_relpath
                else "",
                fixed_joint_offset=robot.frame_manipulator_mount.offset.pos,
                fixed_joint_orient=robot.frame_manipulator_mount.offset.rot,
                mask_all_collisions=True,
                disable_root_joints=not manipulator_needs_jacobian,
            )
            # Actions
            for (
                action_key,
                action_term,
            ) in robot.manipulator.actions.__dict__.items():
                if not isinstance(action_term, ActionTermCfg):
                    continue
                # Ensure the actions terms are applied to the correct asset
                action_term.asset_name = manipulator_name
                # Add action terms to the action group
                setattr(self.actions, f"{manipulator_name}__{action_key}", action_term)
            # Add the command mapping function to the action group
            map_cmd_to_action_fns.append(robot.manipulator.actions.map_cmd_to_action)
        else:
            self.joint_assemblies.pop(manipulator_name, None)

        ## Extra: Manipulator & Mobile manipulator (combined) - End-effector
        if isinstance(robot, (Manipulator, CombinedMobileManipulator)):
            manipulator = robot if isinstance(robot, Manipulator) else robot.manipulator
            if isinstance(manipulator.end_effector, Tool):
                if isinstance(
                    manipulator.end_effector.asset_cfg,
                    (RigidObjectCfg, ArticulationCfg),
                ):
                    manipulator.end_effector.asset_cfg.prim_path = (
                        prim_path_end_effector
                    )
                    (_fixed_joint_offset, _fixed_joint_orient) = (
                        combine_frame_transforms_tuple(
                            manipulator.frame_flange.offset.pos,
                            manipulator.frame_flange.offset.rot,
                            manipulator.end_effector.frame_mount.offset.pos,
                            manipulator.end_effector.frame_mount.offset.rot,
                        )
                    )
                    (
                        manipulator.end_effector.asset_cfg.init_state.pos,
                        manipulator.end_effector.asset_cfg.init_state.rot,
                    ) = (_fixed_joint_offset, _fixed_joint_orient)
                    self.joint_assemblies[end_effector_name] = RobotAssemblerCfg(
                        base_path=manipulator.asset_cfg.prim_path,
                        attach_path=manipulator.end_effector.asset_cfg.prim_path,
                        base_mount_frame=f"/{manipulator.frame_flange.prim_relpath}"
                        if manipulator.frame_flange.prim_relpath
                        else "",
                        attach_mount_frame=f"/{manipulator.end_effector.frame_mount.prim_relpath}"
                        if manipulator.end_effector.frame_mount.prim_relpath
                        else "",
                        fixed_joint_offset=_fixed_joint_offset,
                        fixed_joint_orient=_fixed_joint_orient,
                        mask_all_collisions=True,
                    )
                else:
                    self.joint_assemblies.pop(end_effector_name, None)
                    manipulator.end_effector.asset_cfg.prim_path = f"{manipulator.asset_cfg.prim_path}/{manipulator.frame_flange.prim_relpath}/{end_effector_name}"
                    (
                        manipulator.end_effector.asset_cfg.init_state.pos,
                        manipulator.end_effector.asset_cfg.init_state.rot,
                    ) = (
                        manipulator.frame_flange.offset.pos,
                        manipulator.frame_flange.offset.rot,
                    )
                setattr(
                    self.scene, end_effector_name, manipulator.end_effector.asset_cfg
                )
                # Offset TCP
                for action_term in manipulator.actions.__dict__.values():
                    if isinstance(
                        action_term,
                        (
                            DifferentialInverseKinematicsActionCfg,
                            OperationalSpaceControllerActionCfg,
                        ),
                    ):
                        if action_term.body_offset is None:
                            action_term.body_offset = action_term.__class__.OffsetCfg(  # type: ignore
                                pos=manipulator.end_effector.frame_tool_centre_point.offset.pos,
                                rot=manipulator.end_effector.frame_tool_centre_point.offset.rot,
                            )
                        (
                            action_term.body_offset.pos,
                            action_term.body_offset.rot,
                        ) = combine_frame_transforms_tuple(
                            action_term.body_offset.pos,
                            action_term.body_offset.rot,
                            manipulator.end_effector.frame_tool_centre_point.offset.pos,
                            manipulator.end_effector.frame_tool_centre_point.offset.rot,
                        )
                # Actions
                if isinstance(manipulator.end_effector, ActiveTool):
                    for (
                        action_key,
                        action_term,
                    ) in manipulator.end_effector.actions.__dict__.items():
                        if not isinstance(action_term, ActionTermCfg):
                            continue
                        # Ensure the actions terms are applied to the correct asset
                        action_term.asset_name = end_effector_name
                        # Add action terms to the action group
                        setattr(
                            self.actions,
                            f"{end_effector_name}__{action_key}",
                            action_term,
                        )
                    # Add the command mapping function to the action group
                    map_cmd_to_action_fns.append(
                        manipulator.end_effector.actions.map_cmd_to_action
                    )
            else:
                setattr(self.scene, end_effector_name, None)
                self.joint_assemblies.pop(end_effector_name, None)

        ## WORKAROUND: Mobile manipulator (combined) - Dummy action
        ## Note: The term is used by mobile manipulators that require Jacobian for their actions to enable/disable the root joint (kept enabled by default)
        if isinstance(robot, CombinedMobileManipulator) and manipulator_needs_jacobian:
            # Add action terms to the action group
            setattr(
                self.actions,
                f"{robot_name}_{manipulator_name}_dummy_switch",
                DummyActionCfg(),
            )
            # Add the command mapping function to the action group
            map_cmd_to_action_fns.append(
                lambda twist, event: torch.Tensor((-1.0 if event else 1.0,)).to(
                    device=twist.device
                )
            )

        ## Update command mapping function
        self.actions.map_cmd_to_action = lambda twist, event: torch.cat(
            [func(twist, event) for func in map_cmd_to_action_fns]
        )

        # Store the updated config in an internal state
        self._robot = robot

    def _update_procedural_assets(self):
        def _update_simforge_asset(asset_cfg: AssetBaseCfg):
            if not isinstance(asset_cfg.spawn, SimforgeAssetCfg):
                return
            if asset_cfg.spawn.seed == 0:
                asset_cfg.spawn.seed = self.seed
            if asset_cfg.spawn.num_assets == 1 and (
                asset_cfg.prim_path.startswith("{ENV_REGEX_NS}")
                or asset_cfg.prim_path.startswith("/World/envs/env_.*")
            ):
                asset_cfg.spawn.num_assets = self.scene.num_envs

        for asset_cfg in self.scene.__dict__.values():
            if isinstance(asset_cfg, AssetBaseCfg):
                _update_simforge_asset(asset_cfg)
            elif isinstance(asset_cfg, RigidObjectCollectionCfg):
                if not isinstance(asset_cfg.rigid_objects, Dict):
                    continue
                for rigid_object_cfg in asset_cfg.rigid_objects.values():
                    _update_simforge_asset(rigid_object_cfg)

    def _maybe_add_particles(self):
        assert self.spacing is not None
        if self.scatter_particles and self.spacing > 0.0:
            self.scene.particles = AssetBaseCfg(  # type: ignore
                prim_path="{ENV_REGEX_NS}/particles",
                spawn=PyramidParticlesSpawnerCfg(
                    ratio=self.particles_ratio,
                    particle_size=self.particles_size,
                    dim_x=round(self.spacing / self.particles_size),
                    dim_y=round(self.spacing / self.particles_size),
                    dim_z=round(0.5 * self.spacing / self.particles_size),
                    velocity=((-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.0)),
                    fluid=False,
                    density=1500.0,
                    friction=0.85,
                    cohesion=0.65,
                ),
                init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.5)),
            )

    def _maybe_disable_fabric_for_particles(self):
        for asset_cfg in self.scene.__dict__.values():
            if isinstance(asset_cfg, AssetBaseCfg) and isinstance(
                asset_cfg.spawn, ParticlesSpawnerCfg
            ):
                self.sim.use_fabric = False
                return
            elif isinstance(asset_cfg, RigidObjectCollectionCfg):
                if not isinstance(asset_cfg.rigid_objects, Dict):
                    continue
                for rigid_object_cfg in asset_cfg.rigid_objects.values():
                    if isinstance(rigid_object_cfg, AssetBaseCfg) and isinstance(
                        rigid_object_cfg.spawn, ParticlesSpawnerCfg
                    ):
                        self.sim.use_fabric = False
                        return

    def _update_debug_vis(self):
        for asset_cfg in self.scene.__dict__.values():
            if isinstance(asset_cfg, SensorBaseCfg):
                asset_cfg.debug_vis = self.debug_vis

        for action_template in self.actions.__dict__.values():
            if isinstance(action_template, ActionTermCfg):
                action_template.debug_vis = self.debug_vis

        for attr in self.__dict__.values():
            if isinstance(attr, VisualizationMarkersCfg):
                for marker in attr.markers.values():
                    marker.visible = self.debug_vis
