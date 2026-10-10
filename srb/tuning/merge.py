"""Deep-merge sampled overrides into a trial's complete ``agent:`` config.

Mappings merge recursively; lists and other values replace atomically. Keys
selected by ``opaque_key_pred`` replace wholesale, preserving dotted and regex
DSL keys. A mapping override on a non-mapping base raises :class:`MergeError`.
"""

from __future__ import annotations

import copy
from typing import Callable

# Characters that mark a key as an opaque DSL leaf (regex metacharacters or the
# dotted-path separator used by dreamer/tdmpc2 config files).
_OPAQUE_METACHARS = set(r".*+?[](){}|^$\\")


class MergeError(ValueError):
    """Raised on an unsafe merge (mapping override onto a non-mapping base)."""


def default_opaque_key_pred(key: str) -> bool:
    """Return whether *key* is a non-recursive dotted/regex DSL key."""
    return any(ch in _OPAQUE_METACHARS for ch in key)


def deep_merge(
    base: dict,
    override: dict,
    *,
    opaque_key_pred: Callable[[str], bool] | None = None,
) -> dict:
    """Return the deep-merged result without mutating either input."""
    if opaque_key_pred is None:
        opaque_key_pred = default_opaque_key_pred

    result = copy.deepcopy(base)
    for key, ov in override.items():
        ov = copy.deepcopy(ov)
        if key not in result or opaque_key_pred(key):
            result[key] = ov
            continue
        bv = result[key]
        if isinstance(ov, dict):
            if not isinstance(bv, dict):
                raise MergeError(
                    f"Cannot merge a mapping override onto non-mapping base value "
                    f"for key {key!r} (base is {type(bv).__name__}). A synthesizer "
                    f"is required to format structured values into this field."
                )
            result[key] = deep_merge(bv, ov, opaque_key_pred=opaque_key_pred)
        else:
            # Scalar / list / None override replaces the base value atomically,
            # even when the base was a mapping (an explicit collapse).
            result[key] = ov
    return result


__all__ = ("MergeError", "deep_merge", "default_opaque_key_pred")
