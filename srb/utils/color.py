import colorsys
import random
from typing import List, Tuple


def seeded_color_palette_hue(
    num_colors: int,
    seed: int,
    hue_start_range: Tuple[float, float] = (0.0, 1.0),
    saturation_range: Tuple[float, float] = (0.6, 0.8),
    value_range: Tuple[float, float] = (0.4, 0.6),
) -> List[Tuple[float, float, float]]:
    """Evenly spaced hues with jittered saturation and value, drawn from ``seed``.

    Same distribution as ``simforge.utils.color.color_palette_hue``, which draws
    from the process-global ``random`` state and caches the first result per
    ``num_colors``; that made the colours (and every configuration digest that
    includes them) differ between processes for the same seed.
    """
    rng = random.Random(seed)
    hue_start = rng.uniform(*hue_start_range)
    return [
        colorsys.hsv_to_rgb(
            (hue_start + i / num_colors) % 1.0,
            rng.uniform(*saturation_range),
            rng.uniform(*value_range),
        )
        for i in range(num_colors)
    ]
