from __future__ import annotations

from copy import deepcopy

from srb.utils.failures import InvalidInputError


def _make_action_group_property():
    """Build the shared ``action_group`` property for robots and active tools.

    It resolves ``action_mode`` from ``action_modes`` and caches a deep copy
    per instance, keeping later binding changes isolated from the class mode.
    The property raises :class:`ValueError` if no modes are declared or the
    selected mode is unavailable.
    """

    def _action_group(self):
        modes = self.action_modes
        if not modes:
            raise ValueError(
                f"{type(self).__name__}.action_modes is empty — "
                f"at least one control mode must be declared."
            )
        key = self.action_mode
        if key is None:
            key = next(iter(modes))
        if key not in modes:
            raise InvalidInputError(
                f"{type(self).__name__}.action_mode={key!r} is not in "
                f"available modes {list(modes)}"
            )
        source_group = modes[key]
        # Deep-copy the selected class-level mode once per (instance, mode) and
        # cache it on the instance so later binding mutations stay isolated. Key
        # on the mode selector, not ``id(source_group)``: ``modes[key]`` is a
        # stable ClassVar so an id-equality check never fires, and a freed id can
        # be reused by a new object — a false cache hit returning a stale copy.
        # Use a dunder ``__``-prefixed key: IsaacLab's ``class_to_dict`` and the
        # configclass ``_validate`` walk skip only ``__``-prefixed ``__dict__``
        # keys, so this private cache is never serialized into a config dump nor
        # traversed during ``cfg.validate()``.
        cache = vars(self).setdefault("__action_group_cache", {})
        if key not in cache:
            cache[key] = deepcopy(source_group)
        return cache[key]

    return property(_action_group)
