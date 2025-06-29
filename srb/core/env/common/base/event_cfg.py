from dataclasses import MISSING
from typing import TYPE_CHECKING

from srb.core.action import (
    DifferentialInverseKinematicsActionCfg,
    OperationalSpaceControllerActionCfg,
)
from srb.core.asset import CombinedMobileManipulator
from srb.core.domain import Domain
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import follow_xform_orientation_linear_trajectory  # noqa F401
from srb.core.mdp import reset_scene_to_default  # noqa F401
from srb.core.mdp import (
    randomize_gravity_uniform,
    randomize_usd_prim_attribute_uniform,
    release_assembly_root_joins_on_action,
    reset_articulations_default,
    reset_xform_orientation_uniform,
)
from srb.utils.cfg import configclass
from srb.utils.math import deg_to_rad

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


@configclass
class BaseEventCfg:
    ## Default reset
    reset_scene: EventTermCfg | None = EventTermCfg(
        # func=reset_scene_to_default,
        func=reset_articulations_default,
        mode="reset",
    )

    ## Gravity
    randomize_gravity: EventTermCfg | None = EventTermCfg(
        func=randomize_gravity_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={"distribution_params": MISSING},
    )

    ## Light
    randomize_sunlight_orientation: EventTermCfg | None = EventTermCfg(
        func=reset_xform_orientation_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={
            "asset_cfg": SceneEntityCfg("sunlight"),
            "orientation_distribution_params": {
                "roll": (
                    deg_to_rad(-60.0),
                    deg_to_rad(60.0),
                ),
                "pitch": (
                    deg_to_rad(-60.0),
                    deg_to_rad(60.0),
                ),
            },
        },
    )
    # progress_sunlight_orientation: EventTermCfg | None = EventTermCfg(
    #     func=follow_xform_orientation_linear_trajectory,
    #     mode="interval",
    #     interval_range_s=(0.5, 0.5),
    #     is_global_time=True,
    #     params={
    #         "asset_cfg": SceneEntityCfg("sunlight"),
    #         "orientation_step_params": {
    #             "roll": deg_to_rad(0.1),
    #             "pitch": deg_to_rad(0.1),
    #         },
    #     },
    # )
    randomize_sunlight_intensity: EventTermCfg | None = EventTermCfg(
        func=randomize_usd_prim_attribute_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={
            "asset_cfg": SceneEntityCfg("sunlight"),
            "attr_name": "intensity",
            "distribution_params": MISSING,
        },
    )
    randomize_sunlight_angular_diameter: EventTermCfg | None = EventTermCfg(
        func=randomize_usd_prim_attribute_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={
            "asset_cfg": SceneEntityCfg("sunlight"),
            "attr_name": "angle",
            "distribution_params": MISSING,
        },
    )
    randomize_sunlight_color_temperature: EventTermCfg | None = EventTermCfg(
        func=randomize_usd_prim_attribute_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={
            "asset_cfg": SceneEntityCfg("sunlight"),
            "attr_name": "color_temperature",
            "distribution_params": MISSING,
        },
    )
    randomize_skydome_orientation: EventTermCfg | None = None

    ## Mobile manipulation
    mobile_manipulator_dynamic_root_joint_release: EventTermCfg | None = None

    def update(self, env_cfg: "AnyEnvCfg"):
        self._update_gravity(env_cfg)
        self._update_sunlight(env_cfg)
        self._update_mobile_manipulator(env_cfg)

        for term in (
            self.randomize_gravity,
            self.randomize_sunlight_orientation,
            self.randomize_sunlight_intensity,
            self.randomize_sunlight_angular_diameter,
            self.randomize_sunlight_color_temperature,
            self.randomize_skydome_orientation,
        ):
            if term is not None:
                term.mode = "reset" if env_cfg.num_envs == 1 else "interval"

    def _update_gravity(self, env_cfg: "AnyEnvCfg"):
        domain: Domain = env_cfg.gravity or env_cfg.domain  # type: ignore
        if domain.gravity_variation <= 0.0:
            self.randomize_gravity = None
        elif self.randomize_gravity:
            gravity_z_range = domain.gravity_range
            self.randomize_gravity.params["distribution_params"] = (
                (0.0, 0.0, -gravity_z_range[0]),
                (0.0, 0.0, -gravity_z_range[1]),
            )

    def _update_sunlight(self, env_cfg: "AnyEnvCfg"):
        if env_cfg.scene.sunlight is None:
            self.randomize_sunlight_orientation = None
            self.progress_sunlight_orientation = None
            self.randomize_sunlight_intensity = None
            self.randomize_sunlight_angular_diameter = None
            self.randomize_sunlight_color_temperature = None
            return

        # Intensity
        if env_cfg.domain.light_intensity_variation <= 0.0:
            self.randomize_sunlight_intensity = None
        elif self.randomize_sunlight_intensity:
            self.randomize_sunlight_intensity.params["distribution_params"] = (
                env_cfg.domain.light_intensity_range
            )

        # Angular diameter
        if env_cfg.domain.light_angular_diameter_variation <= 0.0:
            self.randomize_sunlight_angular_diameter = None
        elif self.randomize_sunlight_angular_diameter:
            self.randomize_sunlight_angular_diameter.params["distribution_params"] = (
                env_cfg.domain.light_angular_diameter_range
            )

        # Color temperature
        if env_cfg.domain.light_color_temperature_variation <= 0.0:
            self.randomize_sunlight_color_temperature = None
        elif self.randomize_sunlight_color_temperature:
            self.randomize_sunlight_color_temperature.params["distribution_params"] = (
                env_cfg.domain.light_color_temperature_range
            )

    def _update_mobile_manipulator(self, env_cfg: "AnyEnvCfg"):
        if isinstance(env_cfg._robot, CombinedMobileManipulator) and bool(
            next(
                (
                    action_term
                    for action_term in env_cfg._robot.manipulator.actions.__dict__.values()
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
        ):
            self.mobile_manipulator_dynamic_root_joint_release = EventTermCfg(
                func=release_assembly_root_joins_on_action,
                mode="interval",
                is_global_time=False,
                interval_range_s=(0.2, 0.2),
                params={"assembly_key": "manipulator", "action_idx": -1},
            )
