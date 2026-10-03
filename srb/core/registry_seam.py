"""Snapshot, restore, or clear SRB class registries.

Use :func:`registry_snapshot` around temporary declarations. Restoration
preserves registered object identity and updates captured containers in place.
Clearing leaves registries empty; re-importing does not repopulate them.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from typing import Any

__all__ = [
    "RegistrySnapshot",
    "clear_registries",
    "register_asset_registry",
    "register_registry_container",
    "registry_containers",
    "registry_snapshot",
    "restore_registries",
    "snapshot_registries",
]


class RegistrySnapshot(dict[tuple[type, str], Any]):
    """Inspectable registry copies with references for in-place restoration."""

    __slots__ = (
        "_roots",
        "_references",
        "_registered_containers",
    )

    def __init__(
        self,
        values: dict[tuple[type, str], Any] | None = None,
        *,
        roots: dict[tuple[type, str], Any] | None = None,
        references: dict[int, Any] | None = None,
        registered_containers: tuple[tuple[type, str], ...] | None = None,
    ) -> None:
        super().__init__({} if values is None else values)
        self._roots = roots
        self._references = references
        self._registered_containers = registered_containers


#: Registry modules register containers here, avoiding imports of those layers.
_CONTAINERS: list[tuple[type, str]] = []


def register_registry_container(owner: type, attribute: str) -> None:
    """Register a container once; owners call this at import time."""

    if not hasattr(owner, attribute):
        raise AttributeError(
            f"{owner.__module__}.{owner.__name__} has no attribute {attribute!r}"
        )
    pair = (owner, attribute)
    if pair not in _CONTAINERS:
        _CONTAINERS.append(pair)


def register_asset_registry(owner: type) -> None:
    """Register the standard containers of an asset-style registry."""

    for attribute in ("registry", "base_types", "meta_types"):
        register_registry_container(owner, attribute)


def registry_containers() -> tuple[tuple[type, str], ...]:
    """Return registered (class, attribute) pairs in declaration order."""

    return tuple(_CONTAINERS)


def _snapshot_value(
    value: Any,
    memo: dict[int, Any],
    references: dict[int, Any],
    active: set[int],
) -> Any:
    """Copy builtin container structure without copying registry leaves."""

    if not isinstance(value, (dict, list, set, tuple)):
        return value

    value_id = id(value)
    if value_id in memo:
        return memo[value_id]
    if value_id in active:
        raise ValueError(
            "registry snapshots do not support cycles through immutable containers"
        )
    active.add(value_id)

    try:
        if isinstance(value, dict):
            clone: Any = {}
            memo[value_id] = clone
            references[id(clone)] = value
            for key, item in value.items():
                clone[_snapshot_value(key, memo, references, active)] = _snapshot_value(
                    item, memo, references, active
                )
            return clone

        if isinstance(value, list):
            clone = []
            memo[value_id] = clone
            references[id(clone)] = value
            clone.extend(
                _snapshot_value(item, memo, references, active) for item in value
            )
            return clone

        if isinstance(value, set):
            clone = set()
            memo[value_id] = clone
            references[id(clone)] = value
            clone.update(
                _snapshot_value(item, memo, references, active) for item in value
            )
            return clone

        # Restore mutable tuple children before returning the original tuple.
        # Cycles through tuples were rejected above because they need a
        # placeholder this copy cannot provide.
        clone = tuple(_snapshot_value(item, memo, references, active) for item in value)
        memo[value_id] = clone
        references[id(clone)] = value
        return clone
    finally:
        active.remove(value_id)


def _restore_value(
    snapshot_value: Any,
    references: dict[int, Any],
    restored: set[int],
) -> Any:
    """Return the original leaf/container and restore nested containers."""

    original = references.get(id(snapshot_value))
    if original is None:
        return snapshot_value
    original_id = id(original)
    if original_id in restored:
        return original
    restored.add(original_id)

    if isinstance(snapshot_value, dict):
        items = [
            (
                _restore_value(key, references, restored),
                _restore_value(value, references, restored),
            )
            for key, value in snapshot_value.items()
        ]
        original.clear()
        original.update(items)
    elif isinstance(snapshot_value, list):
        values = [
            _restore_value(value, references, restored) for value in snapshot_value
        ]
        original.clear()
        original.extend(values)
    elif isinstance(snapshot_value, set):
        values = {
            _restore_value(value, references, restored) for value in snapshot_value
        }
        original.clear()
        original.update(values)
    elif isinstance(snapshot_value, tuple):
        # Tuples cannot be mutated; restore their mutable children in place.
        for value in snapshot_value:
            _restore_value(value, references, restored)
    return original


def snapshot_registries() -> RegistrySnapshot:
    """Capture registry values and references for in-place restoration.

    Keep registry activity quiescent until restore completes. Temporary
    declarations are removed from the seam during restoration.
    """

    registered = registry_containers()
    memo: dict[int, Any] = {}
    references: dict[int, Any] = {}
    active: set[int] = set()
    roots: dict[tuple[type, str], Any] = {}
    values: dict[tuple[type, str], Any] = {}
    for owner, attribute in registered:
        root = getattr(owner, attribute)
        roots[(owner, attribute)] = root
        values[(owner, attribute)] = _snapshot_value(root, memo, references, active)
    return RegistrySnapshot(
        values,
        roots=roots,
        references=references,
        registered_containers=registered,
    )


def _restore_legacy_snapshot(
    snapshot: dict[tuple[type, str], Any],
) -> None:
    """Restore snapshots made by the former exported dict type alias."""

    for (owner, attribute), saved in snapshot.items():
        container = getattr(owner, attribute)
        container.clear()
        if isinstance(container, dict):
            container.update(saved)
        else:
            container.extend(saved)


def restore_registries(
    snapshot: RegistrySnapshot | dict[tuple[type, str], Any],
) -> None:
    """Restore containers captured by :func:`snapshot_registries` in place.

    Restores captured mutable containers in place, so held references see the
    restored contents. If a registry attribute was replaced, reattach its
    captured root. Other registry activity must have remained quiescent since
    capture and must remain so until restoration finishes, as documented by
    :func:`snapshot_registries`.
    """

    if not isinstance(snapshot, RegistrySnapshot) or snapshot._roots is None:
        _restore_legacy_snapshot(snapshot)
        return

    restored: set[int] = set()
    assert snapshot._references is not None
    for (owner, attribute), saved in snapshot.items():
        root = snapshot._roots[(owner, attribute)]
        _restore_value(saved, snapshot._references, restored)
        if getattr(owner, attribute) is not root:
            setattr(owner, attribute, root)

    # Drop temporary declarations without changing their newly created objects.
    assert snapshot._registered_containers is not None
    _CONTAINERS[:] = snapshot._registered_containers


def clear_registries() -> None:
    """Empty every registry container.

    Re-importing does not repopulate them. Use :func:`registry_snapshot` for
    temporary changes unless an empty process is intended.
    """

    for owner, attribute in registry_containers():
        getattr(owner, attribute).clear()


@contextmanager
def registry_snapshot() -> Iterator[RegistrySnapshot]:
    """Restore registries to their entry state when the block exits.

    Keep registry activity quiescent across the block. Nested snapshots are
    supported; see :func:`snapshot_registries` for ownership rules.
    """

    snapshot = snapshot_registries()
    try:
        yield snapshot
    finally:
        restore_registries(snapshot)
