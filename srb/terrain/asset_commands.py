"""Canonical CLI command strings for terrain asset remediation."""

from __future__ import annotations

import shlex


def asset_list_command(kind: str) -> str:
    return f"srb asset list {kind}"


def asset_validate_manifest_command(*, check_shas: bool = False) -> str:
    parts = ["srb", "asset", "validate-manifest"]
    if check_shas:
        parts.append("--check-shas")
    return " ".join(parts)


def asset_download_command(
    site: str,
    *,
    patch: str = "default",
    lod: str = "medium",
    seed: int = 0,
    patch_size_m: float | None = None,
    patch_center_xy_m: tuple[float, float] | None = None,
    use_ortho: bool = True,
    deshade: bool = False,
) -> str:
    parts = ["srb", "asset", "download", "terrain", site]
    if patch != "default":
        parts.extend(("--patch", patch))
    parts.extend(("--lod", lod))
    if patch_size_m is not None:
        parts.append(f"--patch-size={float(patch_size_m)!r}")
    if patch_center_xy_m is not None:
        x, y = patch_center_xy_m
        parts.append(f"--center={float(x)!r},{float(y)!r}")
    if seed != 0:
        parts.extend(("--seed", str(seed)))
    if not use_ortho:
        parts.append("--no-ortho")
    if deshade:
        # Preserve the assessed request because deshade affects the cache key.
        parts.append("--deshade")
    return " ".join(shlex.quote(part) for part in parts)


__all__ = [
    "asset_download_command",
    "asset_list_command",
    "asset_validate_manifest_command",
]
