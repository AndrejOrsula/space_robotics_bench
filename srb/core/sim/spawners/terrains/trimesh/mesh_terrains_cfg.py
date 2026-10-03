from srb.core.sim.spawners.terrains import SubTerrainBaseCfg
from srb.utils.cfg import configclass
from srb.utils.math import radians

from . import mesh_terrains


@configclass
class CustomSubTerrainBaseCfg(SubTerrainBaseCfg):
    # Base Mesh Parameters
    resolution: float = 0.05
    """Grid resolution for generating the final mesh."""
    border_width: float = 0.25
    """The width of the border region where the terrain smoothly fades to zero for seamless tiling."""
    thickness: float = 0.2
    """The thickness of the solid terrain mesh base, ensuring a watertight geometry."""

    # Noise Parameters (Ranges scaled by difficulty)
    noise_scale_range: tuple[float, float] = (0.01, 0.08)
    """The min/max range for the noise amplitude, scaled by difficulty."""
    noise_period_range: tuple[float, float] = (0.8, 1.5)
    """The min/max range for the noise period in meters, scaled by difficulty."""
    noise_octaves_range: tuple[int, int] = (1, 3)
    """The min/max range for the number of noise octaves, scaled by difficulty."""
    noise_lacunarity_range: tuple[float, float] = (1.8, 2.2)
    """The min/max range for the noise lacunarity, scaled by difficulty."""


@configclass
class MeshWaveTerrainCfg(CustomSubTerrainBaseCfg):
    """Configuration for a terrain with procedurally generated waves."""

    function = mesh_terrains.wave_terrain  # type: ignore[misc]
    num_ripples_range: tuple[int, int] = (2, 8)
    """The min/max range for the number of wave ripples to generate, scaled by difficulty."""
    amplitude_range: tuple[float, float] = (0.1, 0.5)
    """The min/max range for the amplitude of each wave, scaled by difficulty."""
    wavelength_range: tuple[float, float] = (0.25, 1.5)
    """The min/max range for the wavelength of each wave, scaled by difficulty."""
    smoothing_range: tuple[float, float] = (0.1, 0.9)
    """The min/max range for the smoothing factor (0=sharp, 1=sinusoidal), scaled by difficulty."""
    gap_range: tuple[float, float] = (0.1, 1.5)
    """The range for the random spacing between waves."""
    start_offset_range: tuple[float, float] = (0.25, 2.5)
    """The range for the random starting coordinate for the first ripple."""
    orientation_range: tuple[float, float] = (-radians(180), radians(180))
    """The range for the random orientation of the waves in radians."""
    z_offset: float = 0.0
    """A constant vertical offset added to the waves."""
    phase: float = 0.0
    """Phase shift for the wave function."""


@configclass
class MeshHillTerrainCfg(CustomSubTerrainBaseCfg):
    """Configuration for a terrain with one or more symmetric, radial hills."""

    function = mesh_terrains.hill_terrain  # type: ignore[misc]
    num_hills_range: tuple[int, int] = (1, 4)
    """The min/max range for the number of hills to generate, scaled by difficulty."""
    slope_range: tuple[float, float] = (radians(5.0), radians(20.0))
    """The min/max range for the slope angle of each hill in radians, scaled by difficulty."""
    beta_range: tuple[float, float] = (0.3, 0.9)
    """The min/max range for the curvature of the hill transitions, scaled by difficulty."""
    hill_radius_range: tuple[float, float] = (0.5, 2.5)
    """The min/max range for the radius of each hill's base, scaled by difficulty."""


@configclass
class MeshTiltedTerrainCfg(CustomSubTerrainBaseCfg):
    """Configuration for a tilted ground plane with fractal noise."""

    function = mesh_terrains.tilted_ground_terrain  # type: ignore[misc]
    max_abs_pitch_range: tuple[float, float] = (radians(0.5), radians(5.0))
    """The min/max absolute pitch in radians, scaled by difficulty. The final pitch will be sampled uniformly from [-max, max]."""
    max_abs_roll_range: tuple[float, float] = (radians(0.5), radians(5.0))
    """The min/max absolute roll in radians, scaled by difficulty. The final roll will be sampled uniformly from [-max, max]."""
