"""Isaac-free source of truth for the excavation heightmap grid."""

from __future__ import annotations

from collections.abc import Mapping
from types import MappingProxyType

#: Keys match ``srb.core.sensor.heightmap.HeightmapSpec``.
EXCAVATION_HEIGHTMAP_SPEC: Mapping[str, float | int | str] = MappingProxyType(
    {
        "nx": 16,
        "ny": 16,
        "cell_size_m": 0.0375,  # 60×60 cm grid, 3.75 cm cells
        "forward_offset_m": 0.30,
        "lateral_offset_m": 0.0,
        "dead_zone_m": 0.30,
        "frame": "kinova_base_link",
        "max_range_m": 1.5,
    }
)


def heightmap_spec_for_target_xy(
    target_xy_m: tuple[float, float],
) -> dict[str, float | int | str]:
    """Return the grid centred forward/laterally on a scoop target.

    ``target_xy_m`` is expressed in ``kinova_base_link``; the grid's nearest
    row sits half a grid depth before the target.
    """

    spec = dict(EXCAVATION_HEIGHTMAP_SPEC)
    grid_depth_m = float(spec["ny"]) * float(spec["cell_size_m"])
    spec["forward_offset_m"] = float(target_xy_m[0]) - 0.5 * grid_depth_m
    spec["lateral_offset_m"] = float(target_xy_m[1])
    return spec
