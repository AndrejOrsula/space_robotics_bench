from dataclasses import dataclass


@dataclass(frozen=True)
class ThumbnailCameraPose:
    """Camera eye position and look-at target for thumbnail rendering."""

    eye: tuple[float, float, float]
    lookat: tuple[float, float, float] = (0.0, 0.0, 0.0)
    up: tuple[float, float, float] = (0.0, 0.0, 1.0)
