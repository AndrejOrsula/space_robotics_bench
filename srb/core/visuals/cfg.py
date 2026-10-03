from typing import Callable, Union

from srb.core.visuals import rtx_post
from srb.utils.cfg import configclass


def set_visuals(cfg: "VisualsCfg"):
    rtx_post.ambient_light(cfg.ambient_light_intensity)
    rtx_post.auto_exposure(**cfg.auto_exposure.to_dict())  # type: ignore[misc]
    rtx_post.chromatic_aberration(**cfg.chromatic_aberration.to_dict())  # type: ignore[misc]
    rtx_post.depth_of_field(**cfg.depth_of_field.to_dict())  # type: ignore[misc]
    rtx_post.fog(**cfg.fog.to_dict())  # type: ignore[misc]
    rtx_post.lens_flare(**cfg.lens_flare.to_dict())  # type: ignore[misc]
    rtx_post.motion_blur(**cfg.motion_blur.to_dict())  # type: ignore[misc]
    rtx_post.reshade(**cfg.reshade.to_dict())  # type: ignore[misc]
    rtx_post.tv_noise(**cfg.tv_noise.to_dict())  # type: ignore[misc]


@configclass
class FogCfg:
    enable: bool = False
    color: tuple[float, float, float] = (0.75, 0.75, 0.75)
    intensity: float = 1.0
    z_up: bool = True
    start_height: float = 1.0
    height_density: float = 1.0
    height_falloff: float = 1.0
    distance_range: tuple[float, float] = (0.0, 1024.0)
    fog_distance_density: float = 1.0


@configclass
class AutoExposureCfg:
    enable: bool = False
    filter_type: int = 0  # 0 (Median) | 1 (Average)
    tau: float = 3.5
    white_scale: float = 10.0
    use_exposure_clamping: bool = True
    min_ev: float = 50.0
    max_ev: float = 100000.0


@configclass
class ChromaticAberrationCfg:
    enable: bool = False
    strength: tuple[float, float, float] = (-0.055, -0.075, 0.015)
    mode: Union[tuple[int, int, int], int] = (0, 0, 0)  # 0 (Radial) | 1 (Barrel)
    lanczos: bool = False


@configclass
class DepthOfFieldCfg:
    enable: bool = False
    subject_distance: float = 2.0
    focal_length: float = 35.0
    f_number: float = 5.0
    anisotropy: float = 0.0


@configclass
class MotionBlurCfg:
    enable: bool = False
    diameter_fraction: float = 0.02
    exposure_fraction: float = 1.0
    num_samples: int = 8


@configclass
class LensFlareCfg:
    enable: bool = False
    scale: float = 1.0
    cutoff_point: Union[tuple[float, float, float], float] = (2.0, 2.0, 2.0)
    cutoff_fuzziness: float = 0.5
    alpha_exposure_scale: float = 1.0
    energy_constraining_blend: bool = False
    blades: int = 5
    aperture_rotation: float = 5.0
    sensor_diagonal: float = 60.0
    sensor_aspect_ratio: float = 1.5
    f_number: float = 5.0
    focal_length: float = 35.0
    noise_strength: float = 0.0
    dust_strength: float = 0.0
    scratch_strength: float = 0.0
    spectral_blur_samples: int = 0
    spectral_blur_intensity: float = 10.0
    spectral_blur_wavelength_range: tuple[float, float, float] = (380.0, 550.0, 770.0)


@configclass
class TVNoiseCfg:
    enable_scanlines: bool = False
    scanline_spread: float = 1.0
    enable_scroll_bug: bool = False
    enable_vignetting: bool = False
    vignetting_size: float = 107.0
    vignetting_strength: float = 0.7
    enable_vignetting_flickering: bool = False
    enable_ghost_flickering: bool = False
    enable_wave_distortion: bool = False
    enable_vertical_lines: bool = False
    enable_random_splotches: bool = False
    enable_film_grain: bool = False
    grain_amount: float = 0.05
    color_amount: float = 0.6
    lum_amount: float = 1.0
    grain_size: float = 1.6


@configclass
class ReshadeCfg:
    enable: bool = False
    preset_file_path: str = ""
    effect_search_dir_path: str = ""
    texture_search_dir_path: str = "/root/isaac-sim/kit/reshade"


@configclass
class VisualsCfg:
    func: Callable = set_visuals

    ambient_light_intensity: float = 0.0
    """``/rtx/sceneDb/ambientLightIntensity``; zero keeps space scenes lit only by
    their authored lights."""

    fog: FogCfg = FogCfg()
    auto_exposure: AutoExposureCfg = AutoExposureCfg()
    chromatic_aberration: ChromaticAberrationCfg = ChromaticAberrationCfg()
    depth_of_field: DepthOfFieldCfg = DepthOfFieldCfg()
    motion_blur: MotionBlurCfg = MotionBlurCfg()
    lens_flare: LensFlareCfg = LensFlareCfg()
    tv_noise: TVNoiseCfg = TVNoiseCfg()
    reshade: ReshadeCfg = ReshadeCfg()
