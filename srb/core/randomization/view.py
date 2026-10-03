"""Queryable view over event-config fields and their active event terms."""

from __future__ import annotations

from typing import Iterator

from srb.core.randomization.term import (
    EventTermCfg,
    RandomizationCategory,
    RandomizationTrigger,
    SrbEventTermCfg,
    is_event_term_cfg,
)
from srb.utils.config_introspection import iter_config_items


class RandomizationView:
    """Read-only view that reflects changes to its events config."""

    def __init__(self, events_cfg) -> None:
        self._events_cfg = events_cfg

    def _iter_field_names(self) -> Iterator[str]:
        """Yield candidate field names using shared config traversal order."""
        for name, _ in iter_config_items(self._events_cfg):
            if name.startswith("_"):
                continue
            yield name

    def active(self) -> Iterator[tuple[str, EventTermCfg]]:
        """Yield active ``(name, term)`` pairs from event-term fields."""
        for name in self._iter_field_names():
            term = getattr(self._events_cfg, name, None)
            if is_event_term_cfg(term):
                yield name, term

    @staticmethod
    def _normalize_category(category: object) -> RandomizationCategory | None:
        if isinstance(category, RandomizationCategory):
            return category
        try:
            return RandomizationCategory(category)
        except (TypeError, ValueError):
            pass
        if isinstance(category, str):
            try:
                return RandomizationCategory[category.strip().upper()]
            except KeyError:
                pass
        return None

    @staticmethod
    def _category_of(term: EventTermCfg) -> RandomizationCategory:
        if not isinstance(term, SrbEventTermCfg):
            return RandomizationCategory.UNTAGGED
        category = RandomizationView._normalize_category(term.category)
        if category is not None:
            return category
        return RandomizationCategory.UNTAGGED

    @staticmethod
    def _category_filter(
        category: RandomizationCategory | str,
    ) -> RandomizationCategory:
        result = RandomizationView._normalize_category(category)
        if result is not None:
            return result
        supported = [item.value for item in RandomizationCategory]
        raise ValueError(
            f"Randomization category {category!r} is unsupported; "
            f"use a RandomizationCategory value or enum name: {supported}."
        )

    @staticmethod
    def _trigger_filter(trigger: RandomizationTrigger | str) -> RandomizationTrigger:
        if isinstance(trigger, RandomizationTrigger):
            return trigger
        try:
            return RandomizationTrigger(trigger)
        except (TypeError, ValueError):
            pass
        if isinstance(trigger, str):
            try:
                return RandomizationTrigger[trigger.strip().upper()]
            except KeyError:
                pass
        supported = [item.value for item in RandomizationTrigger]
        raise ValueError(
            f"Randomization trigger {trigger!r} is unsupported; "
            f"use a RandomizationTrigger value or enum name: {supported}."
        )

    def categories(self) -> set[RandomizationCategory]:
        """Return categories among active terms, including ``UNTAGGED`` if present."""
        return {self._category_of(term) for _, term in self.active()}

    def triggers(self) -> set[RandomizationTrigger]:
        """Return distinct recognized triggers among active terms."""
        result: set[RandomizationTrigger] = set()
        for _, term in self.active():
            trigger = RandomizationTrigger.from_mode(term.mode)
            if trigger is not None:
                result.add(trigger)
        return result

    def where(
        self,
        category: RandomizationCategory | str | None = None,
        trigger: RandomizationTrigger | str | None = None,
    ) -> Iterator[tuple[str, EventTermCfg]]:
        """Yield terms matching all supplied filters; accept enum names/values."""
        category_filter = (
            self._category_filter(category) if category is not None else None
        )
        trigger_filter = self._trigger_filter(trigger) if trigger is not None else None
        for name, term in self.active():
            if (
                category_filter is not None
                and self._category_of(term) is not category_filter
            ):
                continue
            if (
                trigger_filter is not None
                and RandomizationTrigger.from_mode(term.mode) is not trigger_filter
            ):
                continue
            yield name, term

    def untagged(self) -> Iterator[tuple[str, EventTermCfg]]:
        """Yield vanilla ``EventTermCfg`` terms without SRB category tags."""
        for name, term in self.active():
            if not isinstance(term, SrbEventTermCfg):
                yield name, term
