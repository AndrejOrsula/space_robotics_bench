"""Collage generation utilities for asset documentation.

Assembles banner collage images from individual asset thumbnails using PIL.
No Isaac Sim dependency — only Pillow and standard library.
"""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image

from ._image import fit_to_canvas

COLLAGE_CELL_GAP = 4  # Pixels between cells
COLLAGE_PADDING = 8  # Outer padding
COLLAGE_BANNER_WIDTH = 2048
COLLAGE_BANNER_HEIGHT = 512
COLLAGE_MASTER_HEIGHT = 768
MAX_COLLAGE_ITEMS = 64
MASTER_COLLAGE_NAME = "_collage_all.webp"

PAGE_COLLAGE_NAMES: dict[str, str] = {
    "manipulators.md": "_collage_manipulators.webp",
    "mobile_robots.md": "_collage_mobile_robots.webp",
    "mobile_manipulators.md": "_collage_mobile_manipulators.webp",
    "tools.md": "_collage_tools.webp",
    "objects.md": "_collage_objects.webp",
    "scenery.md": "_collage_scenery.webp",
}


def _collage_grid_layout(n: int, canvas_w: int, canvas_h: int) -> tuple[int, int]:
    """Return (cols, rows) for *n* items targeting a landscape banner.

    Searches for the grid dimensions that produce cells closest to 4:3 aspect
    ratio while minimizing wasted slots.
    """
    if n <= 1:
        return (1, 1)
    best_cols, best_rows = 1, 1
    best_waste = float("inf")
    for rows in range(1, n + 1):
        cols = math.ceil(n / rows)
        cell_w = (canvas_w - COLLAGE_PADDING * 2 - COLLAGE_CELL_GAP * (cols - 1)) / cols
        cell_h = (canvas_h - COLLAGE_PADDING * 2 - COLLAGE_CELL_GAP * (rows - 1)) / rows
        if cell_w <= 0 or cell_h <= 0:
            continue
        cell_aspect = cell_w / cell_h
        target_aspect = 4.0 / 3.0
        waste = abs(cell_aspect - target_aspect) + abs(cols * rows - n) * 0.1
        if waste < best_waste:
            best_waste = waste
            best_cols, best_rows = cols, rows
    return (best_cols, best_rows)


def _pick_representative(names: list[str], max_items: int) -> list[str]:
    """Pick evenly-spaced representative items from the list."""
    if len(names) <= max_items:
        return names
    step = len(names) / max_items
    return [names[int(i * step)] for i in range(max_items)]


def _symmetric_row_counts(n: int, cols: int, rows: int) -> list[int]:
    """Distribute *n* items across *rows* rows (max *cols* each).

    Center rows are filled first so the collage looks symmetric
    with empty slots pushed to the outer corners.
    """
    if n >= cols * rows:
        return [cols] * rows

    base = min(cols, n // rows)
    counts = [base] * rows
    remainder = n - base * rows

    center = (rows - 1) / 2.0
    order = sorted(range(rows), key=lambda r: (abs(r - center), r))
    for i in range(remainder):
        if counts[order[i]] < cols:
            counts[order[i]] += 1
        else:
            for j in range(i, len(order)):
                if counts[order[j]] < cols:
                    counts[order[j]] += 1
                    break
    return counts


def generate_collage(
    out_dir: Path,
    asset_names: list[str],
    output_path: Path,
    *,
    banner_width: int = COLLAGE_BANNER_WIDTH,
    banner_height: int = COLLAGE_BANNER_HEIGHT,
    max_items: int = MAX_COLLAGE_ITEMS,
) -> bool:
    """Generate a collage banner image from existing asset thumbnails.

    Loads individual ``.webp`` thumbnails from *out_dir*, arranges them in a
    symmetric grid, and writes the result to *output_path*.

    Returns True if a collage was generated successfully.
    """
    available = [n for n in asset_names if (out_dir / f"{n}.webp").exists()]
    if len(available) < 2:
        return False

    selected = _pick_representative(available, max_items)

    cols, rows = _collage_grid_layout(len(selected), banner_width, banner_height)
    cell_w = (
        banner_width - COLLAGE_PADDING * 2 - COLLAGE_CELL_GAP * (cols - 1)
    ) // cols
    cell_h = (
        banner_height - COLLAGE_PADDING * 2 - COLLAGE_CELL_GAP * (rows - 1)
    ) // rows

    if cell_w <= 0 or cell_h <= 0:
        return False

    cell_images: list[Image.Image] = []
    for name in selected:
        try:
            img = Image.open(out_dir / f"{name}.webp").convert("RGBA")
            fitted = fit_to_canvas(img, cell_w, cell_h, margin_pct=0.05)
            cell_images.append(fitted)
        except Exception as e:
            from srb.utils import logging

            logging.logger.debug(f"Skipping {name} in collage: {e}")
            continue

    if len(cell_images) < 2:
        return False

    canvas = Image.new("RGBA", (banner_width, banner_height), (0, 0, 0, 0))

    row_counts = _symmetric_row_counts(len(cell_images), cols, rows)

    grid_h = rows * cell_h + (rows - 1) * COLLAGE_CELL_GAP
    y0 = (banner_height - grid_h) // 2

    img_idx = 0
    for row, count in enumerate(row_counts):
        if count <= 0:
            continue
        row_w = count * cell_w + (count - 1) * COLLAGE_CELL_GAP
        rx0 = (banner_width - row_w) // 2
        py = y0 + row * (cell_h + COLLAGE_CELL_GAP)
        for c in range(count):
            if img_idx >= len(cell_images):
                break
            px = rx0 + c * (cell_w + COLLAGE_CELL_GAP)
            canvas.alpha_composite(cell_images[img_idx], (px, py))
            img_idx += 1

    canvas.save(output_path, quality=92)
    return True
