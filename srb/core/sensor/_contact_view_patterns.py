"""Isaac-free helpers for building filtered PhysX contact-view patterns."""

from __future__ import annotations

import re
from collections.abc import Callable, Sequence

_ENV_ROOT_RE = re.compile(r"^(/World/envs/[^/]+)(?:/|$)")
_ENV_INDEX_RE = re.compile(r"_(\d+)$")
_PHYSX_WILDCARD_RE = re.compile(r"(?<!\.)\*")


def physx_glob_to_prim_regex(pattern: str) -> str:
    """Convert PhysX's ``*`` path wildcard to a single-segment prim regex.

    Isaac Lab matches the regex against the whole prim path, where ``.``
    crosses ``/``; ``.*`` would let ``env_*/bolt`` also match
    ``env_0/bolt/bolt`` and its collision mesh.
    """
    return _PHYSX_WILDCARD_RE.sub("[^/]*", pattern)


def _env_root(path: str) -> str | None:
    match = _ENV_ROOT_RE.match(path)
    return match.group(1) if match is not None else None


def _env_order(roots: Sequence[str]) -> list[int]:
    """Return the positions of ``roots`` ordered by their ``env_<i>`` index.

    Isaac Lab's env id is the ``<i>`` of ``env_<i>``; a lexicographic prim
    listing would put ``env_10`` before ``env_2``. Roots without an integer
    suffix keep their resolved order.
    """
    indices: list[int] = []
    for root in roots:
        match = _ENV_INDEX_RE.search(root)
        if match is None:
            return list(range(len(roots)))
        indices.append(int(match.group(1)))
    return sorted(range(len(roots)), key=indices.__getitem__)


def expand_env_scoped_contact_patterns(
    sensor_pattern: str,
    filter_patterns: Sequence[str],
    resolve_paths: Callable[[str], Sequence[str]],
    resolve_filter_paths: Callable[[str], Sequence[str]] | None = None,
) -> tuple[list[str], list[list[str]]] | None:
    """Expand a batched one-to-many contact view into explicit per-env paths.

    PhysX accepts a wildcard sensor plus flat filter patterns, but that form is
    ambiguous when both sides match multiple environments. The documented
    one-to-many form is one concrete sensor path per environment paired with a
    nested list of that environment's concrete filter paths. Sensors come out
    in ``env_<i>`` order.

    ``resolve_filter_paths`` resolves filter patterns; it defaults to
    ``resolve_paths``. The PhysX backend passes a resolver that maps each
    matched prim to its rigid bodies, because PhysX attributes a contact to
    the rigid-body prim and a filter naming its parent Xform matches nothing.

    ``None`` means that the patterns are not env-scoped or cannot be expanded
    without guessing, so the caller should preserve the original arguments.
    """
    sensor_paths = list(resolve_paths(sensor_pattern))
    roots = [_env_root(path) for path in sensor_paths]
    if (
        not sensor_paths
        or any(root is None for root in roots)
        or len(set(roots)) != len(roots)
    ):
        return None
    order = _env_order([root for root in roots if root is not None])
    sensor_paths = [sensor_paths[i] for i in order]
    sensor_roots = [roots[i] for i in order]

    if resolve_filter_paths is None:
        resolve_filter_paths = resolve_paths
    resolved_filters = [
        list(resolve_filter_paths(pattern)) for pattern in filter_patterns
    ]
    if not resolved_filters or any(not matches for matches in resolved_filters):
        return None

    filters_by_sensor: list[list[str]] = []
    for sensor_root in sensor_roots:
        sensor_filters: list[str] = []
        for matches in resolved_filters:
            local = [path for path in matches if _env_root(path) == sensor_root]
            shared = [path for path in matches if _env_root(path) is None]
            has_foreign_env = any(
                root is not None and root != sensor_root
                for root in map(_env_root, matches)
            )
            if not local and has_foreign_env:
                return None
            sensor_filters.extend(local)
            sensor_filters.extend(shared)
        if not sensor_filters:
            return None
        filters_by_sensor.append(sensor_filters)

    if len({len(filters) for filters in filters_by_sensor}) != 1:
        return None
    return sensor_paths, filters_by_sensor


def expand_contact_view_patterns(
    sensor_patterns: str | Sequence[str],
    filter_patterns: Sequence[str] | Sequence[Sequence[str]],
    resolve_paths: Callable[[str], Sequence[str]],
    resolve_filter_paths: Callable[[str], Sequence[str]] | None = None,
) -> tuple[list[str], list[list[str]]] | None:
    """Expand either ``create_rigid_contact_view`` argument form per environment.

    Older Isaac Lab passed one sensor pattern and a flat filter list. Isaac Lab
    3 passes one sensor pattern per contact-reporting body and one filter list
    per sensor pattern, and reads the view body-major
    (``body_id * num_envs + env_id``). Each body pattern is expanded with
    :func:`expand_env_scoped_contact_patterns` and the results are
    concatenated in body order, which keeps that layout.

    ``None`` means that some pattern cannot be expanded; the caller should
    preserve the original arguments.
    """
    if isinstance(sensor_patterns, str):
        if not all(isinstance(item, str) for item in filter_patterns):
            return None
        return expand_env_scoped_contact_patterns(
            sensor_patterns,
            [str(item) for item in filter_patterns],
            resolve_paths,
            resolve_filter_paths,
        )

    patterns = list(sensor_patterns)
    nested = list(filter_patterns)
    if (
        not patterns
        or len(nested) != len(patterns)
        or any(isinstance(item, str) for item in nested)
    ):
        return None

    sensor_paths: list[str] = []
    filters_by_sensor: list[list[str]] = []
    env_count: int | None = None
    for pattern, filters in zip(patterns, nested):
        expanded = expand_env_scoped_contact_patterns(
            pattern,
            list(filters),
            resolve_paths,
            resolve_filter_paths,
        )
        if expanded is None:
            return None
        paths, per_sensor = expanded
        if env_count is None:
            env_count = len(paths)
        elif len(paths) != env_count:
            return None
        sensor_paths.extend(paths)
        filters_by_sensor.extend(per_sensor)

    if len({len(filters) for filters in filters_by_sensor}) != 1:
        return None
    return sensor_paths, filters_by_sensor
