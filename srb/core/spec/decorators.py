"""@spec class decorator + spec_field attribute helper."""

from __future__ import annotations

import sys
from typing import Any, Callable, Literal

from srb.core.spec.models import DType, TermSpec
from srb.core.spec.registry import ComponentDescriptor, SpecRegistry

_SPEC_FIELDS_ATTR = "_spec_fields"
_SPEC_FIELD_PLACEHOLDER = object()


class _SpecFieldDescriptor:
    """Returned by spec_field. On instance access, yields _SPEC_FIELD_PLACEHOLDER."""

    __slots__ = ("name",)

    def __init__(self, name: str) -> None:
        self.name = name

    def __get__(self, instance: Any, owner: Any) -> Any:
        if instance is None:
            return self
        return _SPEC_FIELD_PLACEHOLDER


def spec_field(
    name: str,
    *,
    dtype: DType,
    shape: tuple[int, ...],
    units: str | None = None,
    frame: str | None = None,
    semantic_label: str | None = None,
    privileged: bool = False,
    min: float | list[float] | None = None,
    max: float | list[float] | None = None,
) -> Any:
    """Class-body attribute descriptor; appends a TermSpec to the owning class's
    ``_spec_fields`` list (consumed by ``@spec``)."""
    term = TermSpec(
        name=name,
        dtype=dtype,
        shape=shape,
        units=units,
        frame=frame,
        semantic_label=semantic_label,
        privileged=privileged,
        min=min,
        max=max,
    )
    # Walk one frame up — we are in a class body whose locals are the
    # forming class's namespace.
    frame = sys._getframe(1)
    cls_namespace = frame.f_locals
    cls_namespace.setdefault(_SPEC_FIELDS_ATTR, []).append(term)
    return _SpecFieldDescriptor(name)


def spec(
    *,
    kind: Literal["action", "observation", "sensor"],
    schema_version: int = 1,
    fields: list[TermSpec] | None = None,
    **extras: Any,
) -> Callable[[type], type]:
    """Class decorator. Registers ``cls`` in SpecRegistry."""

    def _wrap(cls: type) -> type:
        collected = list(getattr(cls, _SPEC_FIELDS_ATTR, []))
        if fields is not None and collected:
            raise ValueError(
                f"{cls.__name__}: cannot pass `fields=` and use spec_field() "
                "in the same class — pick exactly one authoring style."
            )
        descriptor = ComponentDescriptor(
            kind=kind,
            schema_version=schema_version,
            fields=fields if fields is not None else collected,
            extra_metadata=dict(extras),
        )
        SpecRegistry.register(cls, descriptor)
        return cls

    return _wrap
