"""``srb asset cache-stats``."""

from __future__ import annotations


def _human_bytes(n: int) -> str:
    size = float(n)
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if size < 1024.0:
            return f"{size:.1f} {unit}"
        size /= 1024.0
    return f"{size:.1f} PiB"


def run(**_unused) -> int:
    from srb.terrain.cache import inspect_cache

    stats = inspect_cache()

    if not stats.exists:
        print(f"cache root not present: {stats.root}")
        return 0

    print(f"cache root: {stats.root}")
    print(f"total size: {_human_bytes(stats.total_size_bytes)}")

    for body in stats.body_stats:
        print(f"  {body.body}: {body.entries} entries across {body.sites} sites")
    print(f"  stale tmp: {stats.stale_tmp_dirs}")
    return 0
