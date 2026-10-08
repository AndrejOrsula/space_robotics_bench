"""Layer-based mask compositing for material textures.

A ``Layer`` is `(color, mask)` where `color` is RGB or grayscale and `mask` is
[0, 1]. ``blend_layers`` lerps each layer onto the base in order. ``ramp_mask``
turns a scalar field into a mask via a linear ramp between two thresholds.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class Layer:
    color: np.ndarray
    mask: np.ndarray


def ramp_mask(field: np.ndarray, *, lo: float, hi: float) -> np.ndarray:
    f = field.astype(np.float32, copy=False)
    if lo == hi:
        return (f >= lo).astype(np.float32)
    if lo < hi:
        return np.clip((f - lo) / (hi - lo), 0.0, 1.0).astype(np.float32)
    return np.clip((lo - f) / (lo - hi), 0.0, 1.0).astype(np.float32)


def blend_layers(base: np.ndarray, layers: Sequence[Layer]) -> np.ndarray:
    out = base.astype(np.float32, copy=True)
    for layer in layers:
        m = layer.mask.astype(np.float32, copy=False)
        if out.ndim == 3 and m.ndim == 2:
            m = m[..., None]
        color = layer.color.astype(np.float32, copy=False)
        if out.ndim == 3 and color.ndim == 2:
            color = color[..., None]
        out = out * (1.0 - m) + color * m
    return out
