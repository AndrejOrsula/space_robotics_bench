"""SpecRegistry — module-level singleton populated at import time."""

from __future__ import annotations

from collections import Counter
from threading import Lock
from typing import Any, ClassVar, Literal

from pydantic import BaseModel, Field

from srb.core.registry_seam import register_registry_container
from srb.core.spec.models import TermSpec


class ComponentDescriptor(BaseModel):
    kind: Literal["action", "observation", "sensor"]
    schema_version: int = 1
    fields: list[TermSpec]
    extra_metadata: dict[str, Any] = Field(default_factory=dict)


class SpecRegistry:
    _store: ClassVar[dict[type, ComponentDescriptor]] = {}
    _lock: ClassVar[Lock] = Lock()

    @classmethod
    def register(cls, target_cls: type, descriptor: ComponentDescriptor) -> None:
        with cls._lock:
            if target_cls in cls._store:
                raise ValueError(
                    f"class {target_cls.__module__}.{target_cls.__name__} already registered"
                )
            # Count once: repeated ``names.count`` is quadratic under this lock.
            name_counts = Counter(t.name for t in descriptor.fields)
            duplicates = {name for name, count in name_counts.items() if count > 1}
            if duplicates:
                raise ValueError(
                    f"duplicate TermSpec names in descriptor for "
                    f"{target_cls.__name__}: {sorted(duplicates)}"
                )
            cls._store[target_cls] = descriptor

    @classmethod
    def lookup(cls, target_cls: type) -> ComponentDescriptor | None:
        return cls._store.get(target_cls)

    @classmethod
    def lookup_with_mro(cls, target_cls: type) -> ComponentDescriptor | None:
        for parent in target_cls.__mro__:
            d = cls._store.get(parent)
            if d is not None:
                return d
        return None

    @classmethod
    def clear(cls) -> None:
        """Wipe this registry only; use ``registry_seam`` to snapshot all."""
        with cls._lock:
            cls._store.clear()


# Append-only; included in the ``registry_seam`` snapshot/restore set.
register_registry_container(SpecRegistry, "_store")
