"""Runtime helpers for materialized Isaac Lab event managers.

Keep Isaac Lab's private event-storage details localized here.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, MutableMapping, MutableSequence
from typing import Any

_MISSING = object()


def live_event_terms(event_manager: object) -> Iterator[tuple[str, Any]]:
    """Yield currently executable ``(name, config)`` pairs from a manager."""

    mode_term_names, mode_term_cfgs = _validated_mode_storage(event_manager)
    _validate_mode_bookkeeping(event_manager, mode_term_names, mode_term_cfgs)
    for mode, event_names in mode_term_names.items():
        for event_name, event_cfg in zip(event_names, mode_term_cfgs[mode]):
            yield str(event_name), event_cfg


def live_event_term_names_by_mode(
    event_manager: object,
) -> dict[str, tuple[str, ...]]:
    """Return detached names by mode, preserving order and empty groups."""

    mode_term_names, mode_term_cfgs = _validated_mode_storage(event_manager)
    _validate_mode_bookkeeping(event_manager, mode_term_names, mode_term_cfgs)
    return {
        str(mode): tuple(str(name) for name in names)
        for mode, names in mode_term_names.items()
    }


def remove_event_terms_by_name_fragments(
    event_manager: object,
    name_fragments: Iterable[str],
) -> int:
    """Remove terms containing any name fragment; return the number removed."""

    fragments = tuple(fragment for fragment in name_fragments if fragment)
    if not fragments:
        return 0

    mode_term_names, mode_term_cfgs = _validated_mode_storage(event_manager)
    bookkeeping = _validate_mode_bookkeeping(
        event_manager,
        mode_term_names,
        mode_term_cfgs,
    )

    matching_indices: dict[str, list[int]] = {}
    removed_cfgs: dict[str, list[Any]] = {}
    for mode, event_names in mode_term_names.items():
        indices = [
            event_id
            for event_id, event_name in enumerate(event_names)
            if any(fragment in str(event_name) for fragment in fragments)
        ]
        matching_indices[mode] = indices
        removed_cfgs[mode] = [mode_term_cfgs[mode][index] for index in indices]

    removed = 0
    for mode, indices in matching_indices.items():
        event_names = mode_term_names[mode]
        event_cfgs = mode_term_cfgs[mode]
        for event_id in reversed(indices):
            event_names.pop(event_id)
            event_cfgs.pop(event_id)
            removed += 1

    class_term_cfgs, interval_time_left, reset_step_ids, reset_once = bookkeeping
    if class_term_cfgs is not None:
        for mode, cfgs in class_term_cfgs.items():
            remaining = list(removed_cfgs[mode])
            for event_id in range(len(cfgs) - 1, -1, -1):
                match = next(
                    (
                        removed_id
                        for removed_id, removed_cfg in enumerate(remaining)
                        if cfgs[event_id] is removed_cfg
                    ),
                    None,
                )
                if match is not None:
                    cfgs.pop(event_id)
                    remaining.pop(match)

    for mode, indices in matching_indices.items():
        if mode == "interval" and interval_time_left is not None:
            for event_id in reversed(indices):
                interval_time_left.pop(event_id)
        if mode == "reset":
            if reset_step_ids is not None:
                for event_id in reversed(indices):
                    reset_step_ids.pop(event_id)
            if reset_once is not None:
                for event_id in reversed(indices):
                    reset_once.pop(event_id)
    return removed


def _validated_mode_storage(
    event_manager: object,
) -> tuple[
    MutableMapping[str, MutableSequence[Any]],
    MutableMapping[str, MutableSequence[Any]],
]:
    mode_term_names = _runtime_event_storage(
        event_manager,
        attr="_mode_term_names",
    )
    mode_term_cfgs = _runtime_event_storage(
        event_manager,
        attr="_mode_term_cfgs",
    )
    missing_modes = set(mode_term_names) - set(mode_term_cfgs)
    if missing_modes:
        mode = next(iter(missing_modes))
        raise ValueError(f"Missing event configs for event mode: {mode!r}")
    extra_modes = set(mode_term_cfgs) - set(mode_term_names)
    if extra_modes:
        raise ValueError(
            "Event manager event modes differ between names and configs: "
            f"names={sorted(mode_term_names)!r}, configs={sorted(mode_term_cfgs)!r}"
        )
    for mode, event_names in mode_term_names.items():
        event_cfgs = mode_term_cfgs[mode]
        if len(event_names) != len(event_cfgs):
            raise ValueError(
                f"Event manager has {len(event_names)} names but "
                f"{len(event_cfgs)} configs for event mode: {mode!r}"
            )
    return mode_term_names, mode_term_cfgs


def _validate_mode_bookkeeping(
    event_manager: object,
    mode_term_names: MutableMapping[str, MutableSequence[Any]],
    mode_term_cfgs: MutableMapping[str, MutableSequence[Any]],
) -> tuple[
    MutableMapping[str, MutableSequence[Any]] | None,
    MutableSequence[Any] | None,
    MutableSequence[Any] | None,
    MutableSequence[Any] | None,
]:
    """Validate all known Isaac bookkeeping before any list is mutated."""

    class_term_cfgs = _optional_runtime_storage(
        event_manager,
        attr="_mode_class_term_cfgs",
        mapping=True,
    )
    if class_term_cfgs is not None:
        if set(class_term_cfgs) != set(mode_term_names):
            raise ValueError(
                "Event manager class-term modes differ from event modes: "
                f"class={sorted(class_term_cfgs)!r}, modes={sorted(mode_term_names)!r}"
            )
        for mode, class_cfgs in class_term_cfgs.items():
            for class_cfg in class_cfgs:
                if not any(
                    class_cfg is event_cfg for event_cfg in mode_term_cfgs[mode]
                ):
                    raise ValueError(
                        "Event manager class-term bookkeeping contains a config "
                        f"not present in mode {mode!r}"
                    )

    interval_time_left = _optional_runtime_storage(
        event_manager,
        attr="_interval_term_time_left",
    )
    interval_count = len(mode_term_names.get("interval", ()))
    advertises_runtime_storage = any(
        getattr(event_manager, attr, _MISSING) is not _MISSING
        for attr in (
            "_mode_class_term_cfgs",
            "_interval_term_time_left",
            "_reset_term_last_triggered_step_id",
            "_reset_term_last_triggered_once",
        )
    )
    if advertises_runtime_storage and interval_count and interval_time_left is None:
        raise ValueError(
            "Event manager exposes runtime bookkeeping but is missing "
            "interval time-left storage"
        )
    if interval_time_left is not None and len(interval_time_left) != interval_count:
        raise ValueError(
            "Event manager interval bookkeeping has "
            f"{len(interval_time_left)} entries for {interval_count} terms"
        )

    reset_step_ids = _optional_runtime_storage(
        event_manager,
        attr="_reset_term_last_triggered_step_id",
    )
    reset_once = _optional_runtime_storage(
        event_manager,
        attr="_reset_term_last_triggered_once",
    )
    if (reset_step_ids is None) != (reset_once is None):
        raise ValueError(
            "Event manager reset bookkeeping must expose both step-id and "
            "triggered-once sequences"
        )
    reset_count = len(mode_term_names.get("reset", ()))
    if advertises_runtime_storage and reset_count and reset_step_ids is None:
        raise ValueError(
            "Event manager exposes runtime bookkeeping but is missing reset "
            "triggered-step storage"
        )
    if reset_step_ids is not None and len(reset_step_ids) != reset_count:
        raise ValueError(
            "Event manager reset bookkeeping has "
            f"{len(reset_step_ids)} step-id entries for {reset_count} terms"
        )
    if reset_once is not None and len(reset_once) != reset_count:
        raise ValueError(
            "Event manager reset bookkeeping has "
            f"{len(reset_once)} triggered-once entries for {reset_count} terms"
        )

    return class_term_cfgs, interval_time_left, reset_step_ids, reset_once


def _optional_runtime_storage(
    event_manager: object,
    *,
    attr: str,
    mapping: bool = False,
) -> MutableMapping[str, MutableSequence[Any]] | MutableSequence[Any] | None:
    storage = getattr(event_manager, attr, _MISSING)
    if storage is _MISSING:
        return None
    if mapping:
        return _runtime_event_storage(event_manager, attr=attr)
    if not isinstance(storage, MutableSequence):
        raise TypeError(
            "Event manager runtime bookkeeping must be a mutable sequence, "
            f"got {type(storage).__name__} for {attr!r}"
        )
    return storage


def _runtime_event_storage(
    event_manager: object,
    *,
    attr: str,
) -> MutableMapping[str, MutableSequence[Any]]:
    try:
        storage = getattr(event_manager, attr)
    except AttributeError as exc:
        raise TypeError(
            "Event manager does not expose Isaac Lab runtime event storage "
            f"attribute {attr!r}"
        ) from exc
    if not isinstance(storage, MutableMapping):
        raise TypeError(
            "Event manager runtime event storage must be a mutable mapping, "
            f"got {type(storage).__name__} for {attr!r}"
        )
    for mode, terms in storage.items():
        if not isinstance(terms, MutableSequence):
            raise TypeError(
                "Event manager runtime event mode storage must be a mutable "
                f"sequence, got {type(terms).__name__} for mode {mode!r}"
            )
    return storage
