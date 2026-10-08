"""Mesh and texture budgets for each LOD on the v2 fused bake path."""

from __future__ import annotations

from srb.terrain.mesh.builder import Lod

FACE_BUDGET: dict[Lod, int] = {"high": 4_000_000, "medium": 1_000_000, "low": 250_000}
"""Maximum mesh face count per LOD tier for a v2 fused bake. Applied by
`srb.terrain.mesh.builder.heightmap_to_mesh`'s `face_budget` parameter via
`trimesh.Trimesh.simplify_quadric_decimation` -- grid only; skirts and
the solid shell are appended afterwards, never decimated."""

TEXTURE_CAP: dict[Lod, int] = {"high": 8192, "medium": 4096, "low": 2048}
"""Maximum texture side length (px, per axis) per LOD tier for a v2 fused
bake's albedo/roughness/normal/AO PNGs. Applied by
`srb.terrain.materials.baker.bake_textures`'s `texture_cap_px` parameter
(box-filter downsample, never upscale)."""

AO_MAX_PX = 2048
"""Ambient-occlusion horizon-sweep resolution cap (px, per axis), applied by
`bake_textures`'s `ao_max_px` parameter -- bounds the cost of the sweep
itself (not just its output), since AO is then upsampled back to match the
other textures' final size. A single tier, not a per-LOD table: it never
exceeds any `TEXTURE_CAP` entry, so it is never the binding constraint above
what the texture itself already allows."""
