"""Shared PIL image fitting used by the docs collage and thumbnail renderers.

``_collage.py`` and ``_thumbnail.py`` each carried a private ``_fit_to_canvas``
with the same body under different parameter names. The copies
had already drifted apart in signature -- one annotated, one not -- so a fix to
the content-detection heuristic in either one would silently miss the other.

Every import here is deferred into the function body on purpose: ``_thumbnail``
defers all of its rendering dependencies until after ``SimulationApp`` exists,
so this module must stay free of runtime imports at module load.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PIL import Image


def fit_to_canvas(
    img: "Image.Image",
    width: int,
    height: int,
    *,
    margin_pct: float = 0.025,
    threshold: int = 15,
) -> "Image.Image":
    """Crop to content, then center-fit onto a width x height transparent RGBA canvas.

    For RGBA images content is detected via non-zero alpha.
    For RGB images content is detected by deviation from the corner background.
    The output canvas is always RGBA with a transparent background.
    """
    import numpy as np
    from PIL import Image

    is_rgba = img.mode == "RGBA"
    if is_rgba:
        alpha_ch = np.asarray(img)[:, :, 3]
        rows_idx = np.where(np.any(alpha_ch > 0, axis=1))[0]
        cols_idx = np.where(np.any(alpha_ch > 0, axis=0))[0]
    else:
        arr = np.asarray(img.convert("RGB")).astype(np.int32)
        corners = np.array(
            [arr[0, 0], arr[0, -1], arr[-1, 0], arr[-1, -1]], dtype=np.float32
        )
        bg = np.median(corners, axis=0)
        diff = np.abs(arr - bg)
        mask = np.any(diff > threshold, axis=2)
        rows_idx = np.where(np.any(mask, axis=1))[0]
        cols_idx = np.where(np.any(mask, axis=0))[0]

    if rows_idx.size > 0 and cols_idx.size > 0:
        r0 = int(rows_idx[0])
        r1 = int(rows_idx[-1]) + 1
        c0 = int(cols_idx[0])
        c1 = int(cols_idx[-1]) + 1
        cropped = img.crop((c0, r0, c1, r1))
    else:
        cropped = img

    avail_w = max(1, int(width * (1.0 - 2.0 * margin_pct)))
    avail_h = max(1, int(height * (1.0 - 2.0 * margin_pct)))
    scale = min(avail_w / cropped.width, avail_h / cropped.height)
    scaled_w = max(1, int(cropped.width * scale))
    scaled_h = max(1, int(cropped.height * scale))
    scaled = cropped.resize((scaled_w, scaled_h), Image.LANCZOS)

    canvas = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    canvas.alpha_composite(
        scaled.convert("RGBA"), ((width - scaled_w) // 2, (height - scaled_h) // 2)
    )
    return canvas


__all__ = ["fit_to_canvas"]
