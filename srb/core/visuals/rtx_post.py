from typing import Iterable

import carb.settings


def ambient_light(intensity: float):
    """Scene-wide ambient light term of the RTX renderer.

    Isaac Lab 3 dropped ``RenderCfg`` (renderer settings now live on each
    camera's ``IsaacRtxRendererCfg``), so the one global carb setting SRB
    always applied is set here instead.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/sceneDb/ambientLightIntensity", float(intensity))


def auto_exposure(
    enable: bool,
    filter_type: int = 0,
    tau: float = 3.5,
    white_scale: float = 10.0,
    use_exposure_clamping: bool = True,
    min_ev: float = 50.0,
    max_ev: float = 100000.0,
):
    """
    filter_type: 0 (Median) | 1 (Average)
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/histogram/enabled", enable)
    settings.set("/rtx/post/histogram/filterType", filter_type)
    settings.set("/rtx/post/histogram/tau", tau)
    settings.set("/rtx/post/histogram/whiteScale", white_scale)
    settings.set("/rtx/post/histogram/useExposureClamping", use_exposure_clamping)
    settings.set("/rtx/post/histogram/minEV", min_ev)
    settings.set("/rtx/post/histogram/maxEV", max_ev)


def chromatic_aberration(
    enable: bool,
    strength: tuple[float, float, float] = (-0.055, -0.075, 0.015),
    mode: tuple[int, int, int] | int = (0, 0, 0),
    lanczos: bool = False,
):
    """
    mode: 0 (Radial) | 1 (Barrel)
    """

    if not isinstance(mode, Iterable):
        mode = (mode, mode, mode)

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/enabled", enable)
    settings.set("/rtx/post/chromaticAberration/strengthR", strength[0])
    settings.set("/rtx/post/chromaticAberration/strengthG", strength[1])
    settings.set("/rtx/post/chromaticAberration/strengthB", strength[2])
    settings.set("/rtx/post/chromaticAberration/modeR", mode[0])
    settings.set("/rtx/post/chromaticAberration/modeG", mode[1])
    settings.set("/rtx/post/chromaticAberration/modeB", mode[2])
    settings.set("/rtx/post/chromaticAberration/enableLanczos", lanczos)


def depth_of_field(
    enable: bool,
    subject_distance: float = 2.0,
    focal_length: float = 35.0,
    f_number: float = 5.0,
    anisotropy: float = 0.0,
):
    settings = carb.settings.get_settings()
    settings.set("/rtx/post/dof/overrideEnabled", enable)
    settings.set("/rtx/post/dof/enabled", enable)
    settings.set("/rtx/post/dof/subjectDistance", subject_distance)
    settings.set("/rtx/post/dof/focalLength", focal_length)
    settings.set("/rtx/post/dof/fNumber", f_number)
    settings.set("/rtx/post/dof/anisotropy", anisotropy)


def fog(
    enable: bool,
    color: tuple[float, float, float] = (0.75, 0.75, 0.75),
    intensity: float = 1.0,
    z_up: bool = True,
    start_height: float = 1.0,
    height_density: float = 1.0,
    height_falloff: float = 1.0,
    distance_range: tuple[float, float] = (0.0, 1024.0),
    fog_distance_density: float = 1.0,
):
    settings = carb.settings.get_settings()
    settings.set("/rtx/fog/enabled", enable)
    settings.set("/rtx/fog/fogColor", color)
    settings.set("/rtx/fog/fogColorIntensity", intensity)
    settings.set("/rtx/fog/fogZup/enabled", z_up)
    settings.set("/rtx/fog/fogStartHeight", start_height)
    settings.set("/rtx/fog/fogHeightDensity", height_density)
    settings.set("/rtx/fog/fogHeightFalloff", height_falloff)
    settings.set("/rtx/fog/fogStartDist", distance_range[0])
    settings.set("/rtx/fog/fogEndDist", distance_range[1])
    settings.set("/rtx/fog/fogDistanceDensity", fog_distance_density)


def lens_flare(
    enable: bool,
    scale: float = 1.0,
    cutoff_point: tuple[float, float, float] | float = (2.0, 2.0, 2.0),
    cutoff_fuzziness: float = 0.5,
    alpha_exposure_scale: float = 1.0,
    energy_constraining_blend: bool = False,
    blades: int = 5,
    aperture_rotation: float = 5.0,
    sensor_diagonal: float = 60.0,
    sensor_aspect_ratio: float = 1.5,
    f_number: float = 5.0,
    focal_length: float = 35.0,
    noise_strength: float = 0.0,
    dust_strength: float = 0.0,
    scratch_strength: float = 0.0,
    spectral_blur_samples: int = 0,
    spectral_blur_intensity: float = 10.0,
    spectral_blur_wavelength_range: tuple[float, float, float] = (380.0, 550.0, 770.0),
):
    if not isinstance(cutoff_point, Iterable):
        cutoff_point = (cutoff_point, cutoff_point, cutoff_point)

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/enabled", enable)
    settings.set("/rtx/post/lensFlares/flareScale", scale)
    settings.set("/rtx/post/lensFlares/cutoffPoint", cutoff_point)
    settings.set("/rtx/post/lensFlares/cutoffFuzziness", cutoff_fuzziness)
    settings.set("/rtx/post/lensFlares/alphaExposureScale", alpha_exposure_scale)
    settings.set(
        "/rtx/post/lensFlares/energyConstrainingBlend", energy_constraining_blend
    )
    settings.set("/rtx/post/lensFlares/physicalSettings", True)
    settings.set("/rtx/post/lensFlares/apertureShapeCircular", blades < 3)
    settings.set("/rtx/post/lensFlares/blades", blades)
    settings.set("/rtx/post/lensFlares/apertureRotation", aperture_rotation)
    settings.set("/rtx/post/lensFlares/sensorDiagonal", sensor_diagonal)
    settings.set("/rtx/post/lensFlares/sensorAspectRatio", sensor_aspect_ratio)
    settings.set("/rtx/post/lensFlares/fNumber", f_number)
    settings.set("/rtx/post/lensFlares/focalLength", focal_length)
    settings.set("/rtx/post/lensFlares/noiseStrength", noise_strength)
    settings.set("/rtx/post/lensFlares/dustStrength", dust_strength)
    settings.set("/rtx/post/lensFlares/scratchStrength", scratch_strength)
    settings.set("/rtx/post/lensFlares/spectralBlurSamples", spectral_blur_samples)
    settings.set("/rtx/post/lensFlares/spectralBlurIntensity", spectral_blur_intensity)
    settings.set(
        "/rtx/post/lensFlares/spectralBlurWavelengthRange",
        spectral_blur_wavelength_range,
    )


def motion_blur(
    enable: bool,
    diameter_fraction: float = 0.02,
    exposure_fraction: float = 1.0,
    num_samples: int = 8,
):
    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/enabled", enable)
    settings.set("/rtx/post/motionblur/maxBlurDiameterFraction", diameter_fraction)
    settings.set("/rtx/post/motionblur/exposureFraction", exposure_fraction)
    settings.set("/rtx/post/motionblur/numSamples", num_samples)


def reshade(
    enable: bool,
    preset_file_path: str = "",
    effect_search_dir_path: str = "",
    texture_search_dir_path: str = "/root/isaac-sim/kit/reshade",
):
    settings = carb.settings.get_settings()
    settings.set("/rtx/reshade/enabled", enable)
    settings.set("/rtx/reshade/presetFilePath", preset_file_path)
    settings.set("/rtx/reshade/effectSearchDirPath", effect_search_dir_path)
    settings.set("/rtx/reshade/textureSearchDirPath", texture_search_dir_path)


def tv_noise(
    enable_scanlines: bool = False,
    scanline_spread: float = 1.0,
    enable_scroll_bug: bool = False,
    enable_vignetting: bool = False,
    vignetting_size: float = 107.0,
    vignetting_strength: float = 0.7,
    enable_vignetting_flickering: bool = False,
    enable_ghost_flickering: bool = False,
    enable_wave_distortion: bool = False,
    enable_vertical_lines: bool = False,
    enable_random_splotches: bool = False,
    enable_film_grain: bool = False,
    grain_amount: float = 0.05,
    color_amount: float = 0.6,
    lum_amount: float = 1.0,
    grain_size: float = 1.6,
):
    settings = carb.settings.get_settings()
    settings.set(
        "/rtx/post/tvNoise/enabled",
        enable_scanlines
        or enable_scroll_bug
        or enable_vignetting
        or enable_vignetting_flickering
        or enable_ghost_flickering
        or enable_wave_distortion
        or enable_vertical_lines
        or enable_random_splotches
        or enable_film_grain,
    )
    settings.set("/rtx/post/tvNoise/enableScanlines", enable_scanlines)
    settings.set("/rtx/post/tvNoise/scanlineSpread", scanline_spread)
    settings.set("/rtx/post/tvNoise/enableScrollBug", enable_scroll_bug)
    settings.set("/rtx/post/tvNoise/enableVignetting", enable_vignetting)
    settings.set("/rtx/post/tvNoise/vignettingSize", vignetting_size)
    settings.set("/rtx/post/tvNoise/vignettingStrength", vignetting_strength)
    settings.set(
        "/rtx/post/tvNoise/enableVignettingFlickering", enable_vignetting_flickering
    )
    settings.set("/rtx/post/tvNoise/enableGhostFlickering", enable_ghost_flickering)
    settings.set("/rtx/post/tvNoise/enableWaveDistortion", enable_wave_distortion)
    settings.set("/rtx/post/tvNoise/enableVerticalLines", enable_vertical_lines)
    settings.set("/rtx/post/tvNoise/enableRandomSplotches", enable_random_splotches)
    settings.set("/rtx/post/tvNoise/enableFilmGrain", enable_film_grain)
    settings.set("/rtx/post/tvNoise/grainAmount", grain_amount)
    settings.set("/rtx/post/tvNoise/colorAmount", color_amount)
    settings.set("/rtx/post/tvNoise/lumAmount", lum_amount)
    settings.set("/rtx/post/tvNoise/grainSize", grain_size)
