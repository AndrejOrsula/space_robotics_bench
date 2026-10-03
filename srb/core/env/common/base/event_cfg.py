from dataclasses import MISSING
from typing import TYPE_CHECKING

from srb.core.action import (
    DifferentialInverseKinematicsActionCfg,
    OperationalSpaceControllerActionCfg,
)
from srb.core.asset import CombinedMobileManipulator
from srb.core.domain import Domain
from srb.core.ephemeris.env_support import resolve_orbit_site, resolve_site
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import (
    apply_sun_ephemeris_fixed,
    progress_sun_ephemeris,
    randomize_gravity_uniform,
    randomize_sun_ephemeris,
    randomize_usd_prim_attribute_uniform,
    release_assembly_root_joins_on_action,
    reset_articulations_default,
    reset_xform_orientation_uniform,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.utils.cfg import configclass
from srb.utils.config_introspection import iter_config_values_by_type
from srb.utils.math import radians

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


@configclass
class BaseEventCfg:
    ## Default reset
    reset_scene: SrbEventTermCfg | None = SrbEventTermCfg(
        func=reset_articulations_default,
        mode="reset",
        category=RandomizationCategory.LIFECYCLE,
    )

    ## Gravity
    randomize_gravity: SrbEventTermCfg | None = SrbEventTermCfg(
        func=randomize_gravity_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={"distribution_params": MISSING},
        category=RandomizationCategory.PHYSICS,
    )

    ## Light
    randomize_sunlight_orientation: SrbEventTermCfg | None = SrbEventTermCfg(
        func=reset_xform_orientation_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={
            "asset_cfg": SceneEntityCfg("sunlight"),
            "orientation_distribution_params": {
                "roll": (
                    radians(-60.0),
                    radians(60.0),
                ),
                "pitch": (
                    radians(-60.0),
                    radians(60.0),
                ),
            },
        },
        category=RandomizationCategory.VISUAL,
    )
    randomize_sunlight_intensity: SrbEventTermCfg | None = SrbEventTermCfg(
        func=randomize_usd_prim_attribute_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={
            "asset_cfg": SceneEntityCfg("sunlight"),
            "attr_name": "intensity",
            "distribution_params": MISSING,
        },
        category=RandomizationCategory.VISUAL,
    )
    randomize_sunlight_angular_diameter: SrbEventTermCfg | None = SrbEventTermCfg(
        func=randomize_usd_prim_attribute_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={
            "asset_cfg": SceneEntityCfg("sunlight"),
            "attr_name": "angle",
            "distribution_params": MISSING,
        },
        category=RandomizationCategory.VISUAL,
    )
    randomize_sunlight_color_temperature: SrbEventTermCfg | None = SrbEventTermCfg(
        func=randomize_usd_prim_attribute_uniform,
        mode="interval",
        is_global_time=True,
        interval_range_s=(10.0, 60.0),
        params={
            "asset_cfg": SceneEntityCfg("sunlight"),
            "attr_name": "color_temperature",
            "distribution_params": MISSING,
        },
        category=RandomizationCategory.VISUAL,
    )
    ## Sampled-epoch sun update; always uses reset mode.
    randomize_sun_ephemeris: SrbEventTermCfg | None = None
    ## Fixed-epoch update when live state is needed at reset; always reset mode.
    apply_sun_ephemeris_fixed: SrbEventTermCfg | None = None
    ## Moving sun update for positive time_scale; uses a deterministic interval.
    progress_sun_ephemeris: SrbEventTermCfg | None = None
    randomize_skydome_orientation: SrbEventTermCfg | None = None

    ## Intravehicular lamp
    randomize_lamp_position: SrbEventTermCfg | None = None
    randomize_lamp_orientation: SrbEventTermCfg | None = None

    ## Mobile manipulation
    mobile_manipulator_dynamic_root_joint_release: SrbEventTermCfg | None = None

    def update(self, env_cfg: "AnyEnvCfg"):
        self._update_gravity(env_cfg)
        self._update_sunlight(env_cfg)
        self._update_mobile_manipulator(env_cfg)

        # Ephemeris terms keep reset/interval modes regardless of env count.
        for term in (
            self.randomize_gravity,
            self.randomize_sunlight_orientation,
            self.randomize_sunlight_intensity,
            self.randomize_sunlight_angular_diameter,
            self.randomize_sunlight_color_temperature,
            self.randomize_skydome_orientation,
        ):
            if term is not None:
                term.mode = "reset" if env_cfg.scene.num_envs == 1 else "interval"

    def _update_gravity(self, env_cfg: "AnyEnvCfg"):
        domain: Domain = env_cfg.gravity or env_cfg.domain  # type: ignore[misc]
        if domain.gravity_variation <= 0.0:
            self.randomize_gravity = None
        elif self.randomize_gravity:
            gravity_z_range = domain.gravity_range
            self.randomize_gravity.params["distribution_params"] = (
                (0.0, 0.0, -gravity_z_range[0]),
                (0.0, 0.0, -gravity_z_range[1]),
            )

    def _update_sunlight(self, env_cfg: "AnyEnvCfg"):
        # Ephemeris terms are enabled only by the branch below.
        self.randomize_sun_ephemeris = None
        self.apply_sun_ephemeris_fixed = None
        self.progress_sun_ephemeris = None

        if (
            getattr(env_cfg, "ephemeris", None) is not None
            and env_cfg.ephemeris.enabled
            and env_cfg.ephemeris.drive_sunlight
        ):
            # Prevent runtime randomizers from overwriting ephemeris lighting.
            self.randomize_sunlight_orientation = None
            self.randomize_sunlight_intensity = None
            self.randomize_sunlight_angular_diameter = None
            self.randomize_sunlight_color_temperature = None

            cfg = env_cfg.ephemeris
            # Keep config-time and reset registration on the compatibility seam.
            from .env_cfg import resolve_ephemeris_site

            orbit_mode = cfg.orbit is not None
            site = resolve_ephemeris_site(
                env_cfg,
                orbit_mode=orbit_mode,
                resolve_orbit_site_fn=resolve_orbit_site,
                resolve_site_fn=resolve_site,
            )

            # Share arguments; copy per term because the manager owns its dict.
            def _sun_event_params() -> dict:
                return {
                    "ephemeris_cfg": cfg,
                    "site": site,
                    "orbit": cfg.orbit,
                    "asset_cfg": SceneEntityCfg("sunlight"),
                    "domain_light_intensity": env_cfg.domain.light_intensity,
                    "domain_light_angular_diameter": env_cfg.domain.light_angular_diameter,
                }

            if cfg.epoch.utc_range is not None or cfg.epoch.utc_windows is not None:
                # Re-query sampled epochs for each episode's lighting.
                self.randomize_sun_ephemeris = SrbEventTermCfg(
                    func=randomize_sun_ephemeris,
                    mode="reset",
                    category=RandomizationCategory.VISUAL,
                    params=_sun_event_params(),
                )
            elif (
                cfg.eclipse
                or orbit_mode
                or cfg.expose_observations
                or cfg.drive_skydome
            ):
                # These features need live state that config-time lighting lacks.
                self.apply_sun_ephemeris_fixed = SrbEventTermCfg(
                    func=apply_sun_ephemeris_fixed,
                    mode="reset",
                    category=RandomizationCategory.VISUAL,
                    params=_sun_event_params(),
                )

            if cfg.epoch.time_scale > 0.0:
                # Advance a moving sun at deterministic global intervals.
                self.progress_sun_ephemeris = SrbEventTermCfg(
                    func=progress_sun_ephemeris,
                    mode="interval",
                    is_global_time=True,
                    interval_range_s=(
                        cfg.epoch.update_interval_s,
                        cfg.epoch.update_interval_s,
                    ),
                    category=RandomizationCategory.VISUAL,
                    params=_sun_event_params(),
                )

            if cfg.drive_skydome:
                # Avoid random yaw fighting the ephemeris-driven skydome.
                self.randomize_skydome_orientation = None
            return

        if env_cfg.scene.sunlight is None:
            self.randomize_sunlight_orientation = None
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
                    for action_term in iter_config_values_by_type(
                        env_cfg._robot.manipulator.action_group,
                        (
                            DifferentialInverseKinematicsActionCfg,
                            OperationalSpaceControllerActionCfg,
                        ),
                    )
                ),
                None,
            )
        ):
            self.mobile_manipulator_dynamic_root_joint_release = SrbEventTermCfg(
                func=release_assembly_root_joins_on_action,
                mode="interval",
                is_global_time=False,
                interval_range_s=(0.2, 0.2),
                category=RandomizationCategory.LIFECYCLE,
                params={"assembly_key": "manipulator", "action_idx": -1},
            )
