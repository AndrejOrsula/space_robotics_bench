"""Mesh cleanup helpers: decimate, weld, recenter, rescale."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
import trimesh

from srb.terrain.errors import MeshDecimationUnavailableError
from srb.utils.failures import FailureCategory, FailureInfo, HealthEvent


def cleanup_rock(
    mesh: trimesh.Trimesh,
    *,
    target_face_count: int,
    health: Callable[[HealthEvent], None] | None = None,
) -> trimesh.Trimesh:
    """Return a cleaned copy of ``mesh``.

    Steps:
      1. Weld duplicate vertices (distance < 1e-6).
      2. Quadric decimation toward ``target_face_count``. A deterministic
         face-cull fallback is used only for a proven missing decimator and
         only when ``health`` is supplied to record degraded health.
      3. Recenter so the centroid sits at the origin.
      4. Rescale so the bounding-box diagonal equals 1.0.

    Output is a new ``trimesh.Trimesh`` (input untouched).
    """
    out = mesh.copy()
    out.merge_vertices()
    health_event: HealthEvent | None = None
    if out.faces.shape[0] > target_face_count:
        try:
            out = out.simplify_quadric_decimation(face_count=target_face_count)
        except (ImportError, MeshDecimationUnavailableError) as exc:
            if health is None:
                raise
            failure = FailureInfo(
                category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
                code="terrain.rocks.decimation_unavailable",
                message=(
                    f"quadric rock decimation unavailable: {type(exc).__name__}: {exc}"
                ),
                component="terrain.rocks.mesh_cleanup",
                details={
                    "target_face_count": target_face_count,
                    "natural_face_count": int(out.faces.shape[0]),
                },
                fallback_allowed=True,
            )
            health_event = HealthEvent.degraded_fallback(
                failure,
                component="terrain.rocks.mesh_cleanup",
                code="terrain.rocks.decimation_fallback",
                message=(
                    "deterministic rock face culling used because quadric "
                    "decimation is unavailable"
                ),
            )
            health(health_event)
            keep = np.random.RandomState(0).choice(
                out.faces.shape[0], size=target_face_count, replace=False
            )
            out = trimesh.Trimesh(vertices=out.vertices, faces=out.faces[keep])
            out.merge_vertices()

    # Use the vertex centroid; volume-based center of mass may be undefined.
    out.apply_translation(-out.centroid)

    # Rescale so diagonal = 1.
    diag = float(np.linalg.norm(out.bounds[1] - out.bounds[0]))
    if diag > 0:
        out.apply_scale(1.0 / diag)

    if health_event is not None:
        out.metadata["terrain_health_events"] = [health_event.to_mapping()]

    return out
