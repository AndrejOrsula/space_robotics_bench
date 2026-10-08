"""Material baking: slope/curvature/AO derivatives → PBR textures + USD shader.

`MATERIALS_VERSION` is part of the cache key — bump on any algorithmic change
to the bake to force re-bakes.
"""

MATERIALS_VERSION: int = 3
