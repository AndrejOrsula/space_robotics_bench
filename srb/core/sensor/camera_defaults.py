"""Isaac-free policies for environment-level camera defaults.

``BaseEnvCfg`` uses these while wiring sensors. Camera resets need a render
refresh before the first observation, and depth cameras need task-sized far
planes when their authored range is missing or still the render default.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from srb.utils.failures import InvalidInputError

#: Far planes at or beyond this render-default threshold (m) are replaced by
#: task ranges. Keep it aligned with
#: :func:`srb.core.sensor.visual_output.visual_depth_observation_range`.
RENDER_DEFAULT_FAR_PLANE_M: float = 1.0e3


def num_rerenders_on_reset_for_cameras(current: int, *, visual_active: bool) -> int:
    """Return the ``num_rerenders_on_reset`` to use for this configuration.

    A positive configured value is an explicit request and is returned
    unchanged. Otherwise one re-render is required when VISUAL or
    EVENT_CAMERA sensors are active. The default 0 cannot be told apart from
    an explicit 0, so camera environments always re-render at least once.
    """
    if current > 0:
        return current
    return 1 if visual_active else current


def coerce_camera_clipping_range(value: object) -> tuple[float, float] | None:
    """Validate an environment-level ``(near, far)`` clipping range in metres.

    ``None`` means no environment default. Sequences (Hydra delivers lists)
    are coerced to a float tuple. ``near`` must be positive, ``far`` must
    exceed it and stay below :data:`RENDER_DEFAULT_FAR_PLANE_M`; otherwise the
    filled range would itself be discarded as a render default.
    """
    if value is None:
        return None
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise InvalidInputError(
            f"camera_clipping_range must be a (near, far) pair in metres, got {value!r}"
        )
    if len(value) != 2:
        raise InvalidInputError(
            f"camera_clipping_range must have exactly two entries, got {value!r}"
        )
    for entry in value:
        if isinstance(entry, bool) or not isinstance(entry, (int, float)):
            raise InvalidInputError(
                f"camera_clipping_range entries must be numbers, got {value!r}"
            )
    near, far = float(value[0]), float(value[1])
    if not (math.isfinite(near) and math.isfinite(far)):
        raise InvalidInputError(f"camera_clipping_range must be finite, got {value!r}")
    if near <= 0.0:
        raise InvalidInputError(
            f"camera_clipping_range near plane must be positive, got {near}"
        )
    if far <= near:
        raise InvalidInputError(
            f"camera_clipping_range far plane must exceed the near plane, got {value!r}"
        )
    if far >= RENDER_DEFAULT_FAR_PLANE_M:
        raise InvalidInputError(
            "camera_clipping_range far plane must be below "
            f"{RENDER_DEFAULT_FAR_PLANE_M} m (treated as a render default), got {far}"
        )
    return (near, far)


def camera_clipping_range_fill(
    current: Sequence[float] | None,
    default: object,
) -> tuple[float, float] | None:
    """Return the clipping range to write onto a camera spawner, or ``None``.

    The environment default replaces only a missing range or one whose far
    plane is a render default (``>= RENDER_DEFAULT_FAR_PLANE_M``); a range
    authored on the camera is kept. ``None`` means leave the camera as it is.
    """
    resolved = coerce_camera_clipping_range(default)
    if resolved is None:
        return None
    if current is not None and float(current[1]) < RENDER_DEFAULT_FAR_PLANE_M:
        return None
    return resolved
