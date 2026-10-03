"""Deterministic per-env seeds and persistent hot-pixel masks."""

from __future__ import annotations

from collections.abc import Iterable
from hashlib import blake2b

_UINT64_MASK = (1 << 64) - 1
_SPLITMIX_C1 = 0x9E3779B97F4A7C15
_SPLITMIX_C2 = 0xBF58476D1CE4E5B9
_SPLITMIX_C3 = 0x94D049BB133111EB


def _mix64(value: int) -> int:
    """Single splitmix64 round, kept inside 64 bits."""
    value = (value + _SPLITMIX_C1) & _UINT64_MASK
    value ^= value >> 30
    value = (value * _SPLITMIX_C2) & _UINT64_MASK
    value ^= value >> 27
    value = (value * _SPLITMIX_C3) & _UINT64_MASK
    value ^= value >> 31
    return value


def _sensor_identity_mix(sensor_id: str | None) -> int:
    """Return a stable 64-bit sensor salt, or zero for legacy derivations."""
    if sensor_id is None:
        return 0
    if not isinstance(sensor_id, str):
        raise TypeError(
            f"sensor_id must be str or None, got {type(sensor_id).__name__}"
        )
    digest = blake2b(
        sensor_id.encode("utf-8"), digest_size=8, person=b"SRB-evcam"
    ).digest()
    return _mix64(int.from_bytes(digest, "little"))


def derive_event_camera_env_seed(
    episode_seed: int, env_id: int, *, sensor_id: str | None = None
) -> int:
    """Return a stable per-env seed in ``[0, 2**63)``."""
    if not isinstance(episode_seed, int):
        raise TypeError(f"episode_seed must be int, got {type(episode_seed).__name__}")
    if not isinstance(env_id, int) or env_id < 0:
        raise ValueError(f"env_id must be a non-negative int, got {env_id!r}")
    state = (
        (episode_seed & _UINT64_MASK)
        ^ _mix64(env_id & _UINT64_MASK)
        ^ _sensor_identity_mix(sensor_id)
    )
    return _mix64(state) & ((1 << 63) - 1)


def distinct_sensor_base_seeds(
    sensors: Iterable[tuple[str, int]],
) -> dict[str, int]:
    """Map each ``(sensor name, base seed)`` to a base seed no other sensor uses.

    The first sensor on a base seed keeps it; a later one on the same base
    seed gets a seed salted with its name, so two sensors never share noise
    while a single sensor keeps its seeds.
    """
    seen: set[int] = set()
    seeds: dict[str, int] = {}
    for name, seed in sensors:
        while seed in seen:
            seed = derive_event_camera_env_seed(seed, 0, sensor_id=name)
        seen.add(seed)
        seeds[name] = seed
    return seeds


def bake_hot_pixel_mask(
    seed: int,
    *,
    width: int,
    height: int,
    fraction: float,
) -> tuple[tuple[int, int], ...]:
    """Return distinct, in-bounds ``(y, x)`` hot pixels in deterministic order."""
    if width <= 0 or height <= 0:
        raise ValueError(f"width/height must be positive; got {width}x{height}")
    if not (0.0 <= fraction <= 1.0):
        raise ValueError(f"fraction must be in [0, 1]; got {fraction!r}")

    n_pixels = width * height
    n_hot = round(fraction * n_pixels)
    if n_hot <= 0:
        return ()
    n_hot = min(n_hot, n_pixels)

    # Linear-congruential walk seeded by splitmix64; pulls flat-indexed
    # candidates until ``n_hot`` distinct entries are accepted. For the
    # realistic ``fraction <= 1e-3`` regime expected of IMX636 presets the
    # walk converges in O(n_hot) iterations.
    state = _mix64(seed & _UINT64_MASK) or 1
    picked: set[int] = set()
    while len(picked) < n_hot:
        state = (state * 6364136223846793005 + 1442695040888963407) & _UINT64_MASK
        picked.add(state % n_pixels)
    coords = sorted(picked)
    return tuple((idx // width, idx % width) for idx in coords)


def derive_episode_event_seed_pack(
    episode_seed: int,
    env_id: int,
    *,
    sensor_id: str | None = None,
) -> tuple[int, int]:
    """Return per-env RNG and persistent hot-pixel seeds for one sensor."""
    env_rng_seed = derive_event_camera_env_seed(
        episode_seed, env_id, sensor_id=sensor_id
    )
    hot_pixel_seed = _mix64(env_id & _UINT64_MASK)
    if sensor_id is not None:
        hot_pixel_seed = _mix64(hot_pixel_seed ^ _sensor_identity_mix(sensor_id))
    hot_pixel_seed &= (1 << 63) - 1
    return env_rng_seed, hot_pixel_seed


__all__ = [
    "bake_hot_pixel_mask",
    "derive_episode_event_seed_pack",
    "derive_event_camera_env_seed",
    "distinct_sensor_base_seeds",
]
