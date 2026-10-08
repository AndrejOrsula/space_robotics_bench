"""Multi-source DEM and colour fusion primitives and orchestrator.

`FUSION_VERSION` participates in the v2 cache key. Modules provide grid
warping, co-registration, blending, colour processing, provenance, and
budgets; `engine.py` wires them together. The package avoids scipy and
Isaac Sim dependencies.
"""

from __future__ import annotations

FUSION_VERSION = 2

__all__ = ["FUSION_VERSION"]
