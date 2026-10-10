"""Shared type-hint resolution for Hydra config handling.

The public seam is intentionally small: callers ask for safe type hints and do
not need to know about SRB annotation preservation, Pydantic fields,
``typing.get_type_hints`` failures, or fallback diagnostics.
"""

from __future__ import annotations

import importlib
import logging
import os
import re
import sys
from typing import Any, ForwardRef, get_type_hints
from weakref import WeakKeyDictionary, WeakSet

from pydantic import BaseModel

TYPE_HINT_FALLBACK_DEBUG_ENV = "SRB_DEBUG_TYPE_HINT_FALLBACKS"

__all__ = (
    "DEFAULT_TYPE_HINT_RESOLVER",
    "TYPE_HINT_FALLBACK_DEBUG_ENV",
    "TypeHintResolver",
    "get_type_hints_safe",
)

_TRUE_ENV_VALUES = frozenset(("1", "true", "yes", "on"))
_FALLBACK_DIAGNOSTIC_DETAIL_LIMIT = 16
_log = logging.getLogger(__name__)
_TYPE_CHECKING_ALIAS_MODULES = {
    "schemas": "isaaclab.sim.schemas",
    "materials": "isaaclab.sim.spawners.materials",
    "Usd": "pxr.Usd",
}


def _fallback_diagnostics_enabled() -> bool:
    return (
        os.environ.get(TYPE_HINT_FALLBACK_DEBUG_ENV, "").strip().lower()
        in _TRUE_ENV_VALUES
    )


#: Annotation keys already reported, to deduplicate diagnostics.
_REPORTED_UNRESOLVED: set[tuple[str, str, str, str]] = set()


def _eval_annotation(
    annotation: Any, owning_cls: type, *, field: str = "<unknown>"
) -> Any:
    """Evaluate a string/ForwardRef annotation in its owning class namespace.

    On failure the UNRESOLVED annotation string is returned, and every consumer
    downstream then treats the field as being of type `str`: Hydra overrides for
    it bypass coercion entirely. That degradation is routine after an Isaac Lab
    bump moves a symbol, so it is reported at WARNING level -- once per distinct
    (class, field, expression) -- rather than only under a debug env var.
    """
    if not isinstance(annotation, (str, ForwardRef)):
        return annotation
    expr = (
        annotation.__forward_arg__ if isinstance(annotation, ForwardRef) else annotation
    )
    globalns: dict[str, Any] = {}
    localns: dict[str, Any] = {}
    try:
        module = sys.modules.get(getattr(owning_cls, "__module__", ""))
        globalns = getattr(module, "__dict__", {}) if module else {}
        localns = dict(vars(owning_cls))
        return eval(expr, globalns, localns)  # noqa: S307
    except Exception:
        pass

    try:
        globalns = _with_type_checking_aliases(expr, globalns)
        return eval(expr, globalns, localns)  # noqa: S307
    except Exception:
        pass

    # configclass copies annotations, so names may exist only in a base module.
    for base in getattr(owning_cls, "__mro__", ())[1:]:
        module = sys.modules.get(getattr(base, "__module__", ""))
        if module is None:
            continue
        try:
            base_globalns = _with_type_checking_aliases(expr, vars(module))
            return eval(expr, base_globalns, dict(vars(base)))  # noqa: S307
        except Exception:
            continue

    _warn_unresolved_annotation(owning_cls, field, expr)
    return annotation


def _warn_unresolved_annotation(owning_cls: type, field: str, expr: str) -> None:
    # Report the field once, against the base class that first declared it.
    declaring_cls = next(
        (
            base
            for base in reversed(getattr(owning_cls, "__mro__", ()))
            if field in base.__dict__.get("__annotations__", {})
        ),
        owning_cls,
    )
    key = (
        getattr(declaring_cls, "__module__", "<unknown>"),
        getattr(declaring_cls, "__qualname__", getattr(declaring_cls, "__name__", "?")),
        field,
        expr,
    )
    if key in _REPORTED_UNRESOLVED:
        return
    _REPORTED_UNRESOLVED.add(key)
    # Upstream TYPE_CHECKING-only names are expected; warn only for SRB fields.
    log = _log.warning if key[0].split(".")[0] == "srb" else _log.debug
    log(
        "Unresolved type annotation %r for field %r of %s.%s; the field will be "
        "treated as its literal annotation string, so Hydra overrides for it "
        "bypass type coercion",
        expr,
        field,
        key[0],
        key[1],
    )


def _with_type_checking_aliases(expr: str, globalns: dict[str, Any]) -> dict[str, Any]:
    """Add common TYPE_CHECKING-only module aliases needed for annotation eval."""
    resolved = dict(globalns)
    for name, module_name in _TYPE_CHECKING_ALIAS_MODULES.items():
        if name in resolved or re.search(rf"\b{re.escape(name)}\b", expr) is None:
            continue
        try:
            resolved[name] = importlib.import_module(module_name)
        except Exception:
            pass
    return resolved


def _get_srb_annotations(cls: type) -> dict[str, Any]:
    """Return merged ``__srb_annotations__`` from *cls* and its MRO."""
    if not isinstance(cls, type):
        return {}
    merged: dict[str, Any] = {}
    for base in reversed(cls.__mro__):
        own = base.__dict__.get("__srb_annotations__")
        if not own:
            continue
        for name, annotation in own.items():
            merged[name] = _eval_annotation(annotation, base, field=name)
    return merged


class TypeHintResolver:
    """Resolve class type hints with cached fallbacks and deduped diagnostics."""

    def __init__(self) -> None:
        self._cache: WeakKeyDictionary[type, dict[str, Any]] = WeakKeyDictionary()
        self._diagnosed_fallbacks: WeakSet[type] = WeakSet()
        self._fallback_diagnostic_count = 0
        self._fallback_suppression_logged = False

    def get_type_hints_safe(self, cls: type) -> dict[str, Any]:
        """Get type hints without failing on unresolvable forward refs.

        For ``srb`` Asset subclasses, prefers ``__srb_annotations__`` which
        preserves original field types before ``Asset.__init_subclass__``
        rewrites them to ``Any``.

        For other Pydantic models, uses ``model_fields`` which holds types
        resolved at class definition time.

        For dataclasses and other types, uses ``typing.get_type_hints`` with
        fallback to merged ``__annotations__`` from the MRO.
        """
        if not isinstance(cls, type):
            return self._resolve_type_hints(cls)

        try:
            cached = self._cache.get(cls)
        except TypeError:
            cached = None
        if cached is not None:
            return dict(cached)

        hints = self._resolve_type_hints(cls)
        try:
            self._cache[cls] = dict(hints)
        except TypeError:
            pass
        return hints

    def _resolve_type_hints(self, cls: type) -> dict[str, Any]:
        if isinstance(cls, type):
            srb_hints = _get_srb_annotations(cls)
            if srb_hints:
                return srb_hints

        if isinstance(cls, type) and issubclass(cls, BaseModel):
            try:
                return {
                    name: field_info.annotation
                    for name, field_info in cls.model_fields.items()
                    if field_info.annotation is not None
                }
            except Exception:
                pass

        try:
            return get_type_hints(cls)
        except Exception:
            self._debug_type_hint_fallback(cls)
            return self._merge_annotation_fallback(cls)

    def _debug_type_hint_fallback(self, cls: type) -> None:
        if not _fallback_diagnostics_enabled():
            return
        if isinstance(cls, type):
            try:
                if cls in self._diagnosed_fallbacks:
                    return
                self._diagnosed_fallbacks.add(cls)
            except TypeError:
                pass
        self._fallback_diagnostic_count += 1
        if self._fallback_diagnostic_count <= _FALLBACK_DIAGNOSTIC_DETAIL_LIMIT:
            _log.debug("get_type_hints failed for %s, trying per-base evaluation", cls)
            return
        if not self._fallback_suppression_logged:
            self._fallback_suppression_logged = True
            _log.debug(
                "get_type_hints fallback diagnostics suppressed after %d "
                "distinct classes; further details omitted",
                _FALLBACK_DIAGNOSTIC_DETAIL_LIMIT,
            )

    def _merge_annotation_fallback(self, cls: type) -> dict[str, Any]:
        merged: dict[str, Any] = {}
        if not isinstance(cls, type):
            merged.update(getattr(cls, "__annotations__", {}))
            return merged

        for base in reversed(cls.__mro__):
            raw = getattr(base, "__annotations__", {})
            if not raw:
                continue
            try:
                merged.update(get_type_hints(base))
                continue
            except Exception:
                pass

            for name, annotation in raw.items():
                resolved = _eval_annotation(annotation, base, field=name)
                if resolved is not annotation or name not in merged:
                    merged[name] = resolved
        return merged


DEFAULT_TYPE_HINT_RESOLVER = TypeHintResolver()


def get_type_hints_safe(cls: type) -> dict[str, Any]:
    """Resolve type hints through the process-wide Hydra resolver."""
    return DEFAULT_TYPE_HINT_RESOLVER.get_type_hints_safe(cls)
