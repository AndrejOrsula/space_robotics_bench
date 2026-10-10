import math


def parse_gravity_magnitude(value: object) -> float | None:
    """Return a finite, nonnegative gravity magnitude in m/s², else ``None``."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        magnitude = float(value)
    except OverflowError:  # an int too large for a float
        return None
    if not math.isfinite(magnitude) or magnitude < 0.0:
        return None
    return magnitude
